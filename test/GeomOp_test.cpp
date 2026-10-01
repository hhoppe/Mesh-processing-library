// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GeomOp.h"

using namespace hh;

namespace {

float rounded(float v) { return round_fraction_digits(v, 1e4f); }

bool near(const Vec3<float>& v1, const Vec3<float>& v2, float tolerance = 1e-5f) { return dist(v1, v2) <= tolerance; }

}  // namespace

int main() {
  {
    // The axis points map exactly between lon-lat and sph.
    SHOW(sph_from_lonlat(Uv(0.f, 0.f)));
    SHOW(sph_from_lonlat(Uv(.3f, 1.f)));
    SHOW(sph_from_lonlat(Uv(0.f, .5f)));
    SHOW(sph_from_lonlat(Uv(.25f, .5f)));
    SHOW(sph_from_lonlat(Uv(.5f, .5f)));
    SHOW(sph_from_lonlat(Uv(.75f, .5f)));
    SHOW(lonlat_from_sph(Point(0.f, 0.f, -1.f)));
    SHOW(lonlat_from_sph(Point(0.f, 0.f, 1.f)));
    SHOW(lonlat_from_sph(Point(-1.f, 0.f, 0.f)));
    SHOW(lonlat_from_sph(Point(0.f, -1.f, 0.f)));
    SHOW(lonlat_from_sph(Point(1.f, 0.f, 0.f)));
  }
  {
    // Round trip lonlat -> sph -> lonlat, including latitudes near the poles, where sph_from_lonlat() once snapped a
    // coordinate to +-1 without the others, and lonlat_from_sph() once lost precision in std::asin().
    double max_mag_error = 0.;
    float max_lat_error = 0.f, max_lon_error = 0.f;
    const int n = 400;
    for_int(i, n + 1) {
      for_int(j, n + 1) {
        const float lat = j == 1 ? .0004f : j == n - 1 ? .9996f : float(j) / n;
        const Uv lonlat(float(i) / n, lat);
        const Point sph = sph_from_lonlat(lonlat);
        max_mag_error = max(max_mag_error, abs(mag<double>(sph) - 1.));
        const Uv lonlat2 = lonlat_from_sph(sph);
        max_lat_error = max(max_lat_error, abs(lonlat2[1] - lonlat[1]));
        // The longitude is undefined at the poles and ill-conditioned near them, and it wraps at the prime meridian.
        if (i > 0 && i < n)
          max_lon_error = max(max_lon_error, abs(lonlat2[0] - lonlat[0]) * std::sin(lat * (TAU / 2)));
      }
    }
    SHOW(max_mag_error < 4e-7);
    SHOW(max_lat_error < 1e-6f);
    SHOW(max_lon_error < 1e-6f);
    // A latitude within 1.4e-3 radians of a pole used to return as the pole itself.
    SHOW(abs(lonlat_from_sph(sph_from_lonlat(Uv(.3f, .0004f)))[1] - .0004f) < 1e-6f);
  }
  {
    // Round trip sph -> lonlat -> sph, for points at geometrically spaced angles from each pole.  The error is
    // dominated by the snapping of a latitude within 1e-6 of 0 or 1, i.e. within 3.1e-6 radians of a pole.
    float max_error = 0.f;
    for_int(i, 1000) {
      const double angle = 1e-7 * std::pow(1e7, i / 999.), lon = i * .618;
      for (const double sign_z : {-1., 1.}) {
        const Point sph(float(std::sin(angle) * std::cos(lon)), float(std::sin(angle) * std::sin(lon)),
                        float(sign_z * std::cos(angle)));
        max_error = max(max_error, dist(sph_from_lonlat(lonlat_from_sph(sph)), sph));
      }
    }
    SHOW(max_error < 1e-5f);
  }
  {
    // Triangle radii and aspect ratio.
    const Point p0(0.f, 0.f, 0.f), p1(3.f, 0.f, 0.f), p2(0.f, 4.f, 0.f);  // A 3-4-5 right triangle.
    SHOW(rounded(circum_radius(p0, p1, p2)), rounded(inscribed_radius(p0, p1, p2)), rounded(aspect_ratio(p0, p1, p2)));
    const Point q0(1.f, 0.f, 0.f), q1(0.f, 1.f, 0.f), q2(0.f, 0.f, 1.f);  // An equilateral triangle.
    const float edge_length = std::sqrt(2.f);
    SHOW(rounded(circum_radius(q0, q1, q2) / edge_length * std::sqrt(3.f)));      // R = a / sqrt(3).
    SHOW(rounded(inscribed_radius(q0, q1, q2) / edge_length * std::sqrt(12.f)));  // r = a / sqrt(12).
    SHOW(rounded(aspect_ratio(q0, q1, q2)));  // The minimum possible aspect ratio is 2.
    // The radii are invariant to the vertex order.
    assertx(abs(circum_radius(p1, p2, p0) - circum_radius(p0, p1, p2)) < 1e-5f);
    assertx(abs(inscribed_radius(p2, p1, p0) - inscribed_radius(p0, p1, p2)) < 1e-5f);
    // A degenerate (collinear) triangle has zero inscribed radius and an infinite aspect ratio.
    const Point r0(0.f, 0.f, 0.f), r1(1.f, 1.f, 1.f), r2(3.f, 3.f, 3.f);
    SHOW(inscribed_radius(r0, r1, r2), aspect_ratio(r0, r1, r2));
    SHOW(inscribed_radius(r0, r0, r0));
  }
  {
    // Dihedral angles about the edge (p1, p2), for the two faces (p1, p2, po1) and (p1, po2, p2).
    const Point p1(0.f, 0.f, 0.f), p2(1.f, 0.f, 0.f), po1(.5f, 1.f, 0.f);
    const Point po2_flat(.5f, -1.f, 0.f), po2_convex(.5f, 0.f, -1.f), po2_concave(.5f, 0.f, 1.f);
    SHOW(dihedral_angle_cos(p1, p2, po1, po2_flat), signed_dihedral_angle(p1, p2, po1, po2_flat));
    SHOW(rounded(dihedral_angle_cos(p1, p2, po1, po2_convex)),
         rounded(signed_dihedral_angle(p1, p2, po1, po2_convex)));
    SHOW(rounded(dihedral_angle_cos(p1, p2, po1, po2_concave)),
         rounded(signed_dihedral_angle(p1, p2, po1, po2_concave)));
    assertx(abs(signed_dihedral_angle(p1, p2, po1, po2_convex) - TAU / 4) < 1e-6f);
    assertx(abs(signed_dihedral_angle(p1, p2, po1, po2_concave) + TAU / 4) < 1e-6f);
    // A complete foldover, with the two faces coincident.
    const Point po2_folded(.5f, 1.f, 0.f);
    SHOW(dihedral_angle_cos(p1, p2, po1, po2_folded));
    // (Whether the result equals TAU / 2 exactly depends on the platform's floating-point math.)
    assertx(abs(abs(signed_dihedral_angle(p1, p2, po1, po2_folded)) - TAU / 2) < 1e-6f);
    // cos(signed_dihedral_angle()) == dihedral_angle_cos() for general configurations.
    for_int(i, 12) {
      const float angle = TAU * (float(i) + .5f) / 12.f;
      const Point po2(.3f, -std::cos(angle), std::sin(angle));
      const float dcos = dihedral_angle_cos(p1, p2, po1, po2);
      const float sangle = signed_dihedral_angle(p1, p2, po1, po2);
      assertx(abs(std::cos(sangle) - dcos) < 1e-5f);
      assertx(sangle * std::sin(angle) < 0.f);  // Concave if po2 is above the plane z == 0.
    }
    // A degenerate face.
    SHOW(dihedral_angle_cos(p1, p2, Point(2.f, 0.f, 0.f), po2_flat));
    SHOW(signed_dihedral_angle(p1, p2, Point(2.f, 0.f, 0.f), po2_flat));
  }
  {
    // Solid angles.
    const Point origin(0.f, 0.f, 0.f);
    const Point px(1.f, 0.f, 0.f), py(0.f, 1.f, 0.f), pz(0.f, 0.f, 1.f);
    SHOW(rounded(solid_angle(origin, V(px, py, pz)) / (TAU / 4)));  // An octant.
    SHOW(rounded(solid_angle(origin, V(px, pz, py)) / (TAU / 4)));  // Its complement.
    // A point at the center of a planar square loop sees a hemisphere.
    const Array<Point> square{Point(1.f, 1.f, 0.f), Point(-1.f, 1.f, 0.f), Point(-1.f, -1.f, 0.f),
                              Point(1.f, -1.f, 0.f)};
    SHOW(rounded(solid_angle(origin, square) / TAU));
    // The solid angle at a corner of a cube, bounded by a hexagonal loop of its neighboring vertices, is an octant.
    const Array<Point> loop{Point(1.f, 0.f, 0.f), Point(1.f, 1.f, 0.f), Point(0.f, 1.f, 0.f),
                            Point(0.f, 1.f, 1.f), Point(0.f, 0.f, 1.f), Point(1.f, 0.f, 1.f)};
    SHOW(rounded(solid_angle(origin, loop) / (TAU / 4)));
  }
  {
    // Turning angle at p2 along the path (p1, p2, p3).
    const Point p1(0.f, 0.f, 0.f), p2(1.f, 0.f, 0.f);
    SHOW(angle_cos(p1, p2, Point(2.f, 0.f, 0.f)), angle_cos(p1, p2, Point(1.f, 1.f, 0.f)));
    SHOW(angle_cos(p1, p2, Point(0.f, 0.f, 0.f)), angle_cos(p1, p2, p2));
  }
  {
    // Orthogonality of frames.
    const Frame frame1 = Frame::rotation(0, .3f) * Frame::rotation(2, 1.1f);
    assertx(nearly_orthonormal(frame1, 1e-5f) && nearly_orthogonal(frame1, 1e-5f));
    const Frame frame2 = Frame::scaling(V(1.f, 2.f, 3.f)) * frame1;
    assertx(!nearly_orthonormal(frame2, 1e-5f) && nearly_orthogonal(frame2, 1e-5f));
    const Frame frame3(Vector(1.f, 0.f, 0.f), Vector(.5f, 2.f, 0.f), Vector(.2f, .1f, 3.f), Point(1.f, 2.f, 3.f));
    assertx(!nearly_orthogonal(frame3, 1e-2f));
    const Frame frame4 = orthogonalized(frame3);
    SHOW(frame4.v(0), transformed(frame4.v(1), rounded), transformed(frame4.v(2), rounded), frame4.p());
    assertx(nearly_orthogonal(frame4, 1e-6f));
    for_int(c, 3) assertx(abs(mag(frame4.v(c)) - mag(frame3.v(c))) < 1e-5f);  // Magnitudes are preserved.
    const Frame frame5 = orthonormalized(frame3);
    assertx(nearly_orthonormal(frame5, 1e-6f) && frame5.p() == frame3.p());
    SHOW(transformed(frame5.v(2), rounded));
    // A left-handed frame stays left-handed.
    const Frame frame6 = orthonormalized(Frame::scaling(V(1.f, 1.f, -1.f)) * frame1);
    assertx(dot(cross(frame6.v(0), frame6.v(1)), frame6.v(2)) < 0.f);
    // normalized_frame() only modifies frames that are nearly orthogonal.
    assertx(normalized_frame(frame3) == frame3);
    Frame frame7 = frame2;
    frame7[0, 1] += 1e-5f;
    assertx(nearly_orthogonal(normalized_frame(frame7), 1e-6f));
    assertx(abs(mag(normalized_frame(frame7).v(2)) - 3.f) < 1e-5f);
  }
  {
    // Euler angles round trip, preserving the axis scales and origin.
    const Frame prev_frame = Frame::scaling(V(2.f, 3.f, 4.f)) * Frame::translation(V(1.f, 2.f, 3.f));
    for (const Vec3<float> ang : {V(0.f, 0.f, 0.f), V(.5f, .2f, -.3f), V(-2.f, 1.f, 2.5f), V(3.f, -1.2f, .1f)}) {
      const Frame frame = frame_from_euler_angles(ang, prev_frame);
      assertx(nearly_orthogonal(frame, 1e-5f) && frame.p() == prev_frame.p());
      for_int(c, 3) assertx(abs(mag(frame.v(c)) - mag(prev_frame.v(c))) < 1e-5f);
      const Vec3<float> ang2 = euler_angles_from_frame(frame);
      assertx(near(ang2, ang));
    }
    // The yaw rotates about the world z axis.
    const Frame frame = frame_from_euler_angles(V(TAU / 4, 0.f, 0.f), Frame::identity());
    assertx(near(frame.v(0), Vector(0.f, 1.f, 0.f)) && near(frame.v(1), Vector(-1.f, 0.f, 0.f)));
    SHOW(transformed(euler_angles_from_frame(Frame::rotation(2, .4f)), rounded));
    SHOW(transformed(euler_angles_from_frame(Frame::rotation(1, -.4f)), rounded));
    SHOW(transformed(euler_angles_from_frame(Frame::rotation(0, .4f)), rounded));
  }
  {
    // Aiming and leveling frames.
    Frame frame = Frame::scaling(V(2.f, 2.f, 2.f)) * Frame::translation(V(5.f, 6.f, 7.f));
    const Vector dir(1.f, 1.f, 1.f);
    frame_aim_at(frame, dir);
    assertx(near(normalized(frame.v(0)), normalized(dir)));
    assertx(abs(frame.v(1)[2]) < 1e-6f);          // The y axis is horizontal.
    assertx(frame.p() == Point(5.f, 6.f, 7.f));   // The origin is unchanged.
    assertx(abs(mag(frame.v(0)) - 2.f) < 1e-5f);  // The scale is unchanged.
    const Frame frame2 = Frame::rotation(0, .3f) * Frame::rotation(1, .2f) * Frame::rotation(2, .7f);
    const Frame level = make_level(frame2);
    assertx(nearly_orthonormal(level, 1e-5f));
    assertx(abs(level.v(1)[2]) < 1e-6f);
    assertx(near(level.v(0), frame2.v(0)));  // The x axis (yaw and pitch) is unchanged.
    const Frame horiz = make_horiz(frame2);
    assertx(abs(horiz.v(0)[2]) < 1e-6f && abs(horiz.v(1)[2]) < 1e-6f);
    assertx(near(horiz.v(2), Vector(0.f, 0.f, 1.f)));
  }
  {
    // Widening a triangle preserves its centroid.
    const Vec3<Point> triangle{Point(0.f, 0.f, 0.f), Point(6.f, 0.f, 0.f), Point(0.f, 3.f, 0.f)};
    assertx(widen_triangle(triangle, 0.f) == triangle);
    const Vec3<Point> wide = widen_triangle(triangle, .2f);
    SHOW(wide);
    assertx(near(mean(wide), mean(triangle)));
    assertx(point_inside(Point(6.f, 0.f, 0.f), wide) && !point_inside(Point(6.f, 0.f, 0.f) * 1.2f, wide));
  }
  {
    // Intersections.
    const Plane plane{Vector(0.f, 0.f, 1.f), 2.f};
    SHOW(intersect_line_with_plane(Line{Point(1.f, 1.f, 0.f), Vector(1.f, 0.f, 1.f)}, plane).value());
    SHOW(intersect_line_with_plane(Line{Point(1.f, 1.f, 5.f), Vector(0.f, 0.f, -2.f)}, plane).value());
    assertx(!intersect_line_with_plane(Line{Point(1.f, 1.f, 0.f), Vector(1.f, 0.f, 0.f)}, plane));  // Parallel.
    SHOW(intersect_segment_with_plane(Point(0.f, 0.f, 0.f), Point(0.f, 0.f, 4.f), plane).value());
    SHOW(intersect_segment_with_plane(Point(0.f, 0.f, 2.f), Point(1.f, 1.f, 4.f), plane).value());  // At p1.
    SHOW(intersect_segment_with_plane(Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 2.f), plane).value());  // At p2.
    assertx(!intersect_segment_with_plane(Point(0.f, 0.f, 0.f), Point(0.f, 0.f, 1.f), plane));
    assertx(!intersect_segment_with_plane(Point(0.f, 0.f, 3.f), Point(0.f, 0.f, 4.f), plane));
    assertx(!intersect_segment_with_plane(Point(0.f, 0.f, 2.f), Point(1.f, 0.f, 2.f), plane));  // In the plane.
    const Vec3<Point> triangle{Point(0.f, 0.f, 2.f), Point(4.f, 0.f, 2.f), Point(0.f, 4.f, 2.f)};
    SHOW(intersect_line_with_triangle(Line{Point(1.f, 1.f, 0.f), Vector(0.f, 0.f, 1.f)}, triangle).value());
    assertx(!intersect_line_with_triangle(Line{Point(3.f, 3.f, 0.f), Vector(0.f, 0.f, 1.f)}, triangle));
    assertx(!intersect_line_with_triangle(Line{Point(1.f, 1.f, 0.f), Vector(1.f, 0.f, 0.f)}, triangle));
    SHOW(intersect_segment_with_triangle(Point(1.f, 2.f, 0.f), Point(1.f, 2.f, 3.f), triangle).value());
    assertx(!intersect_segment_with_triangle(Point(1.f, 2.f, 0.f), Point(1.f, 2.f, 1.f), triangle));
    assertx(!intersect_segment_with_triangle(Point(3.f, 3.f, 0.f), Point(3.f, 3.f, 3.f), triangle));
  }
  {
    // Signed volume of a tetrahedron.
    const Point p1(0.f, 0.f, 0.f), p2(1.f, 0.f, 0.f), p3(0.f, 1.f, 0.f), p4(0.f, 0.f, 1.f);
    SHOW(rounded(signed_volume(p1, p2, p3, p4)), rounded(signed_volume(p1, p3, p2, p4)));
    SHOW(signed_volume(p1, p2, p3, Point(5.f, 7.f, 0.f)));  // Coplanar points.
    // The volume is invariant to translation and scales cubically.
    const Vector t(10.f, -20.f, 30.f);
    assertx(abs(signed_volume(p1 + t, p2 + t, p3 + t, p4 + t) - 1.f / 6.f) < 1e-5f);
    assertx(abs(signed_volume(p1 * 2.f, p2 * 2.f, p3 * 2.f, p4 * 2.f) - 8.f / 6.f) < 1e-5f);
  }
}
