// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GeomOp.h"

#include "libHh/MathOp.h"

namespace hh {

// *** Radii

// Deduced from book: Coxeter "Geometry".
float circum_radius(const Point& p0, const Point& p1, const Point& p2) {
  using Precision = double;
  Precision a = dist<Precision>(p0, p1), b = dist<Precision>(p1, p2), c = dist<Precision>(p2, p0);
  Precision s = (a + b + c) * .5;
  Precision d2 = s * (s - a) * (s - b) * (s - c);
  if (d2 <= 0.) {
    Warning("circum_radius degenerate");
    return 1e10f;
  }
  return float(a * b * c * .25 / my_sqrt(d2));
}

// From Coxeter "Intro to Geometry, Second ed.", page 12, equation  1.531
float inscribed_radius(const Point& p0, const Point& p1, const Point& p2) {
  // r = d / s
  // d = sqrt(s * (s - a) * (s - b) * (s -c))
  // s = (a + b + c) / 2
  using Precision = double;
  Precision a = dist<Precision>(p0, p1), b = dist<Precision>(p1, p2), c = dist<Precision>(p2, p0);
  Precision s = (a + b + c) * .5f;
  Precision d2 = s * (s - a) * (s - b) * (s - c);
  if (d2 <= 0.f) {
    Warning("inscribed_radius degenerate");
    return 0.f;
  }
  return float(sqrt(d2) / s);
}

// Should normalize to be 1.f for an equilateral triangle?
float aspect_ratio(const Point& p0, const Point& p1, const Point& p2) {
  using Precision = double;
  Precision a = dist<Precision>(p0, p1), b = dist<Precision>(p1, p2), c = dist<Precision>(p2, p0);
  Precision s = (a + b + c) * .5;
  Precision d2 = s * (s - a) * (s - b) * (s - c);
  if (d2 <= 0.) {
    // Warning("aspect_ratio degenerate");
    return 1e10f;
  }
  return float(a * b * c * .25 * s / d2);
}

// *** Misc

float dihedral_angle_cos(const Point& p1, const Point& p2, const Point& po1, const Point& po2) {
  Vector ves1 = cross(p1, p2, po1);
  Vector ves2 = cross(p1, po2, p2);
  if (!ves1.normalize() || !ves2.normalize()) return -2.f;
  return clamp(dot(ves1, ves2), -1.f, 1.f);
}

float signed_dihedral_angle(const Point& p1, const Point& p2, const Point& po1, const Point& po2) {
  Vector ves1 = cross(p1, p2, po1);
  Vector ves2 = cross(p1, po2, p2);
  // no need to normalize since we obtain both fcos and fsin
  float fcos = dot(ves1, ves2);
  Vector vcross = cross(ves1, ves2);
  float fsin = mag(vcross);
  if (dot(vcross, p2 - p1) < 0.f) fsin = -fsin;
  if (!fsin && !fcos) return -10.f;
  return std::atan2(fsin, fcos);
}

float solid_angle(const Point& p, CArrayView<Point> pa) {
  // The solid angle is the fraction of area covered on a sphere centered about p.  Range is [0, TAU * 2].
  // Idea: Gauss-Bonnet theorem: integral of curvature + line integral + exterior angles = TAU.
  // Integral of curvature on unit sphere is equal to area; line integral along geodesics (great circles) is zero.
  // Therefore, solid angle = TAU - sum of exterior angles on sphere.
  //
  // NOTE: This is imprecise for small triangles, due to TAU - TAU, but changing computation to double fixes that.
  //
  // Alternative definition (only valid for ang < TAU / 4):
  // sin(ang / 2) = sqrt(sin(s) * sin(s - a) * sin(s - b) * sin(s - c)) / (2 * cos(a / 2) * cos(b / 2) * cos(c / 2)),
  //  where  s = (a + b + c) / 2,   a = arclen(BC) on sphere, b = arclen(CA), and c = arclen(AB).
  //
  // Another alternative definition:
  // tan(ang / 4) = sqrt(tan(s / 2) tan((s - a) / 2) tan((s - b) / 2) tan((s - c) / 2)).
  //
  const int np = pa.num();
  double sum_ang = 0.;
  if (0) {
    for_int(i, np) {
      int ip = (i - 1 + np) % np, in = (i + 1) % np;
      Vector top = pa[i] - p;
      if (!assertw(top.normalize())) continue;
      Vector v1 = pa[i] - pa[ip];
      v1 -= top * dot(v1, top);
      if (!assertw(v1.normalize())) continue;
      Vector v2 = pa[in] - pa[i];
      v2 -= top * dot(v2, top);
      if (!assertw(v2.normalize())) continue;
      float vcos = dot(v1, v2);
      float vsin = dot(cross(v1, v2), top);
      float ang = std::atan2(vsin, vcos);
      sum_ang += ang;
    }
  } else {
    // Can correctly handle duplicate points in loop
    Point pp = pa[0];
    Vector vp;
    dummy_init(vp);
    Vector topp = vp;
    bool have_prior = false;
    for (int i = 1;; i++) {
      Point pc = pa[i % np];
      Vector vc = pc - pp;
      if (is_zero(vc)) {
        Warning("is_zero(vc)");
        if (0) SHOW(pa);
        if (!have_prior && i >= np) {
          Warning("Degenerate solid angle");
          sum_ang = D_TAU;
          break;
        }
        continue;
      }
      Vector top = pc - p;
      if (!assertw(top.normalize())) return 0.f;
      if (have_prior) {
        // project onto sphere tangent vectors at topp
        Vector v1 = project_orthogonally(vp, topp);
        if (!assertw(v1.normalize())) return 0.f;
        Vector v2 = project_orthogonally(vc, topp);
        if (!assertw(v2.normalize())) return 0.f;
        float vcos = dot(v1, v2);
        float vsin = dot(cross(v1, v2), topp);
        float ang = std::atan2(vsin, vcos);
        // Ambiguity between -TAU / 2 and +TAU / 2 does matter here!
        if (0) SHOW(ang, ang + TAU / 2);
        if (ang < -TAU / 2 + 1e-6f) {
          Warning("Near-degenerate angle");
          ang = +TAU / 2;
        }
        sum_ang += ang;
      }
      if (i >= np + 1) break;
      pp = pc;
      have_prior = true;
      vp = vc;
      topp = top;
    }
  }
  float solid_ang = float(D_TAU - sum_ang);
  if (0 && solid_ang > TAU * 2 - 1e-6f) solid_ang = 0.f;
  if (0) solid_ang = clamp(solid_ang, 0.f, TAU * 2);
  return solid_ang;
}

float angle_cos(const Point& p1, const Point& p2, const Point& p3) {
  Vector v1 = p2 - p1;
  Vector v2 = p3 - p2;
  if (!v1.normalize() || !v2.normalize()) return -2.f;
  return clamp(dot(v1, v2), -1.f, 1.f);
}

// *** Frames and Euler angles

namespace {

template <typename T> T my_atan2(T y, T x) { return !y && !x ? T{0} : std::atan2(y, x); }

}  // namespace

bool nearly_orthogonal(const Frame& frame, float tolerance) {
  if (abs(dot(frame.v(0), frame.v(1))) > tolerance) return false;
  if (abs(dot(frame.v(1), frame.v(2))) > tolerance) return false;
  if (abs(dot(frame.v(2), frame.v(0))) > tolerance) return false;
  return true;
}

bool nearly_orthonormal(const Frame& frame, float tolerance) {
  for_int(c, 3) if (abs(mag2(frame.v(c)) - 1.f) > tolerance) return false;
  return nearly_orthogonal(frame, tolerance);
}

Frame orthogonalized(const Frame& frame) {
  // Use Gram-Schmidt orthogonalization, but preserve original magnitudes.  Handles both left and right handedness.
  using Precision = double;
  const Vec3<Precision> v0 = convert<Precision>(frame.v(0));
  Vec3<Precision> v1 = convert<Precision>(frame.v(1));
  Vec3<Precision> v2 = convert<Precision>(frame.v(2));
  const Precision orig_v1_mag2 = mag2(v1);
  const Precision orig_v2_mag2 = mag2(v2);

  // Make v1 orthogonal to v0.
  const Precision v0_inv_mag2 = 1.f / mag2(v0);
  v1 -= v0 * (dot(v1, v0) * v0_inv_mag2);

  // Make v2 orthogonal to both v0 and v1.
  const Precision v1_inv_mag2 = 1.f / mag2(v1);
  v2 -= v0 * (dot(v2, v0) * v0_inv_mag2);
  v2 -= v1 * (dot(v2, v1) * v1_inv_mag2);

  // Rescale v1 and v2 to their original magnitudes.
  v1 *= sqrt(orig_v1_mag2 * v1_inv_mag2);
  v2 *= sqrt(orig_v2_mag2 / mag2(v2));

  return Frame(frame.v(0), convert<float>(v1), convert<float>(v2), frame.p());
}

Frame orthonormalized(const Frame& frame) {
  // Use Gram-Schmidt orthonormalization.  It correctly handles both left and right handedness.
  using Precision = double;
  const Vec3<Precision> v0_orig = convert<Precision>(frame.v(0));
  const Vec3<Precision> v1_orig = convert<Precision>(frame.v(1));
  const Vec3<Precision> v2_orig = convert<Precision>(frame.v(2));
  const Vec3<Precision> v0 = normalized(v0_orig);
  const Vec3<Precision> v1 = normalized(v1_orig - dot(v1_orig, v0) * v0);
  const Vec3<Precision> v2a = v2_orig - dot(v2_orig, v0) * v0;
  const Vec3<Precision> v2 = normalized(v2a - dot(v2a, v1) * v1);
  return Frame(convert<float>(v0), convert<float>(v1), convert<float>(v2), frame.p());
}

Frame normalized_frame(const Frame& frame, float tolerance) {
  if (nearly_orthonormal(frame, tolerance)) return orthonormalized(frame);
  if (nearly_orthogonal(frame, tolerance)) return orthogonalized(frame);
  return frame;
}

// See https://en.wikipedia.org/wiki/Euler_angles
//  Tait-Bryan angles / Nautical angles / Cardan angles : sometimes called Euler angles, not "proper Euler angles"
//
//  We use (z, -y', x''):
//   angle 0 is measured wrt  z  axis (alpha) (yaw)   (heading)
//   angle 1 is measured wrt -y' axis (beta)  (pitch) (elevation)
//   angle 2 is measured wrt x'' axis (phi)   (roll)  (bank)
//
//  This is similar to case 5 of Tait-Bryan angles (z, y', x'') except that y' is negated.
//
//  It is also similar to aircraft principal axes: (-z (down), -y' (right), x'' (front)) except that z is negated,
//   so my yaw/heading is reversed wrt compass directions.
//
// To extract angles, look at tostdf:
//      cos(b)cos(a)  cos(b)sin(a)  -sin(b)
//      ?             ?             sin(p)cos(b)
//      ?             ?             cos(p)cos(b)

Vec3<float> euler_angles_from_frame(const Frame& frame) {
  return V(my_atan2(frame[0][1], frame[0][0]),                            //
           my_atan2(-frame[0][2], std::hypot(frame[0][0], frame[0][1])),  //
           my_atan2(frame[1][2] / mag(frame[1]), frame[2][2] / mag(frame[2])));
}

Frame frame_from_euler_angles(const Vec3<float>& ang, const Frame& prev_frame) {
  Frame frame = Frame::identity();  // Note: modifying local rather than prev_frame to preserve axes scale and origin.
  Vec3<float> prev_mags = map(prev_frame.head<3>(), [](const Vec3<float>& v) { return float(mag<double>(v)); });
  for_int(c, 3) frame[c][c] = prev_mags[c];
  for_int(c, 3) frame = frame * Frame::rotation(c, ang[2 - c]);  // World Z yaw, then world Y pitch, then world X roll.
  if (1) frame = orthogonalized(frame);                          // Optional, for precision.
  frame.p() = prev_frame.p();
  return frame;
}

void frame_aim_at(Frame& frame, const Vector& v) {
  Vec3<float> ang;
  ang[0] = my_atan2(v[1], v[0]);
  ang[1] = my_atan2(-v[2], std::hypot(v[0], v[1]));
  ang[2] = 0.f;
  frame = frame_from_euler_angles(ang, frame);
}

Frame make_level(const Frame& frame) {
  Frame frame_new = frame;
  static const bool world_zxy = getenv_bool("WORLD_ZXY");  // z forward, -x left, -y up
  const Frame from_zxy =
      Frame(Vector(0.f, 0.f, 1.f), Vector(-1.f, 0.f, 0.f), Vector(0.f, -1.f, 0.f), Point(0.f, 0.f, 0.f));
  if (world_zxy) frame_new *= ~from_zxy;
  Vec3<float> ang = euler_angles_from_frame(frame_new);
  ang[2] = 0.f;
  frame_new = frame_from_euler_angles(ang, frame_new);
  if (world_zxy) {
    frame_new *= from_zxy;
    frame_new.p() = frame.p();
  }
  return frame_new;
}

Frame make_horiz(const Frame& frame) {
  Frame frame_new = frame;
  static const bool world_zxy = getenv_bool("WORLD_ZXY");  // z forward, -x left, -y up
  const Frame from_zxy =
      Frame(Vector(0.f, 0.f, 1.f), Vector(-1.f, 0.f, 0.f), Vector(0.f, -1.f, 0.f), Point(0.f, 0.f, 0.f));
  if (world_zxy) frame_new *= ~from_zxy;
  Vec3<float> ang = euler_angles_from_frame(frame_new);
  ang[1] = 0.f;
  ang[2] = 0.f;
  frame_new = frame_from_euler_angles(ang, frame_new);
  if (world_zxy) {
    frame_new *= from_zxy;
    frame_new.p() = frame.p();
  }
  return frame_new;
}

Vec3<Point> widen_triangle(const Vec3<Point>& triangle, float eps) {
  return V<Point>(interp(triangle, 1.f + eps, -eps * 0.5f),  //
                  interp(triangle, -eps * 0.5f, 1.f + eps),  //
                  interp(triangle, -eps * 0.5f, -eps * 0.5f));
}

// *** Intersections

std::optional<Point> intersect_line_with_plane(const Line& line, const Plane& plane) {
  const float numerator = plane.d - dot(line.point, plane.nor);
  const float denominator = dot(plane.nor, line.vec);
  // When the line lies in the triangle plane, we report no intersection.  Is this reasonable?
  if (!denominator) return {};
  const float t = numerator / denominator;
  const Point pint = line.point + line.vec * t;
  return pint;
}

std::optional<Point> intersect_segment_with_plane(const Point& p1, const Point& p2, const Plane& plane) {
  const float s1 = dot(p1, plane.nor) - plane.d;
  const float s2 = dot(p2, plane.nor) - plane.d;
  if ((s1 < 0.f && s2 < 0.f) || (s1 > 0.f && s2 > 0.f)) return {};  // Equivalent to "s1 * s2 > 0.f"?
  const float denominator = s2 - s1;
  // When the segment lies in the plane, we report no intersection.  Is this reasonable?
  if (!denominator) return {};
  const Point pint = interp(p1, p2, s2 / denominator);
  return pint;
}

std::optional<Point> intersect_line_with_triangle(const Line& line, const Vec3<Point>& triangle) {
  const Plane plane = plane_of_triangle(triangle);
  const auto result = intersect_line_with_plane(line, plane);
  if (!result) return {};
  const Point& pint = *result;
  if (!point_inside(pint, triangle)) return {};
  return pint;
}

std::optional<Point> intersect_segment_with_triangle(const Point& p1, const Point& p2, const Vec3<Point>& triangle) {
  const Plane plane = plane_of_triangle(triangle);
  const auto result = intersect_segment_with_plane(p1, p2, plane);
  if (!result) return {};
  const Point& pint = *result;
  if (!point_inside(pint, triangle)) return {};
  return pint;
}

// *** Other

float signed_volume(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
  // Formula derived from the scalar triple product of vectors.
  if (0) {
    return dot(p2 - p1, cross(p3 - p1, p4 - p1)) / 6.f;
  } else {
    return dot(p2 - p1, cross(p1, p3, p4)) / 6.f;
  }
}

Uv lonlat_from_sph(const Point& sph) {
  // We assume: lon=0 at +Y, lon=.25 at -X, lat=0 at -Z.
  // We place lat=0 at -Z because the OpenGL Uv coordinate origin is at the image lower-left.
  const float lon = snap_coordinate(std::atan2(sph[0], -sph[1]) / TAU + .5f);  // azimuth; phi.
  const float lat = snap_coordinate(std::asin(sph[2]) / (TAU / 2) + .5f);      // zenith; theta.
  return Uv(lon, lat);
}

Point sph_from_lonlat(const Uv& lonlat) {
  // We assume: lon=0 at +Y, lon=.25 at -X, lat=0 at -Z.
  // We place lat=0 at -Z because the OpenGL Uv coordinate origin is at the image lower-left.
  const float lon = lonlat[0];
  const float lat = lonlat[1];
  const float ang_lon = lon * TAU;
  const float ang_lat = lat * (TAU / 2);
  const Point sph = snap_coordinates(
      Point(-std::sin(ang_lon) * std::sin(ang_lat), std::cos(ang_lon) * std::sin(ang_lat), -std::cos(ang_lat)));
  return sph;
}

}  // namespace hh
