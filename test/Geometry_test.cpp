// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Geometry.h"

#include "libHh/MatrixOp.h"
#include "libHh/RangeOp.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

template <ranges::forward_range R> R truncate_small_floats(R&& range) {
  using T = range_value_t<R>;
  static_assert(std::is_floating_point_v<T>, "range must contain elements of type float/double");
  for (auto& e : range)
    if (abs(e) < 1e-6f) e = 0.f;
  return std::forward<R>(range);
}

float rounded(float v) { return round_fraction_digits(v, 1e4f); }

bool near(const Vec3<float>& v1, const Vec3<float>& v2, float tolerance = 1e-5f) { return dist(v1, v2) <= tolerance; }

bool near(const Frame& frame1, const Frame& frame2, float tolerance = 1e-5f) {
  for_int(i, 4) if (!near(frame1[i], frame2[i], tolerance)) return false;
  return true;
}

}  // namespace

int main() {
  {
    Point p(1.f, 2.f, 3.f), q(8.f, 7.f, 6.f);
    const Vector v(1.f, 2.f, 3.f), w(1.f, 0.f, 1.f);
    Vector x(0.f, 1.f, 0.f), y = x;
    SHOW(p, q, v, w, x, y);
    p += 2.f * w - x;
    SHOW(p);
    x += (q - p) * 3.f + w;
    SHOW(x);
    p = Point(1.f, 2.f, 3.f);
    SHOW(p);
    q = Point(8.f, 7.f, 6.f);
    SHOW(q);
  }
  {
    const Point o(5.f, 4.f, 3.f);
    const Vector v1(0.f, 0.f, 2.f), v2(3.f, 0.f, 0.f), v3(0.f, 1.f, 1.f);
    const Point p(2.f, 3.f, 4.f);
    Point q;
    Frame frame(v1, v2, v3, o);
    Frame frame2 = Frame::identity();
    SHOW(frame2);
    frame2 = frame;
    const Frame frame3 = frame2;
    SHOW(frame);
    q = p * frame;
    SHOW(frame[1][0]);
    SHOW((frame[1, 0]));
    SHOW(frame.p()[1]);
    SHOW(frame.p());
    SHOW(q);
    // SHOW(~frame2);
    Frame frame2inv = ~frame2;
    truncate_small_floats(frame2inv.grid_view());
    SHOW(frame2inv);
    SHOW(frame2 * frame2);
    SHOW(q * inverse(frame2));
    Frame frame4 = frame3 * frame * ~frame2 * Frame::identity() * ~Frame::identity();
    truncate_small_floats(frame4.grid_view());
    SHOW(frame4);
    Frame zero = Frame::scaling(thrice(0.f));
    SHOW(zero);
    SHOW(Frame::identity());
    SHOW(dot(v1, v2));
    SHOW(dot(v1, v3));
  }
  {
    Frame frame(Vector(0.f, 0.f, 2.f), Vector(3.f, 0.f, 0.f), Vector(0.f, 1.f, 1.f), Point(5.f, 4.f, 3.f));
    SHOW(frame);
    SGrid<float, 4, 4> hf = to_Matrix(frame);
    SHOW(hf.const_grid_view());
    Point p1(2.f, 3.f, 4.f);
    SHOW(p1);
    SHOW(p1 * frame);
    Array<float> p2{2.f, 3.f, 4.f, 1.f};
    SHOW(p2);
    SHOW(mat_mul(p2, hf.grid_view()));
    hf[1, 3] = 4.f;
    hf[3, 3] = 0.f;
    SHOW(hf.const_grid_view());
    SHOW(mat_mul(p2, hf.grid_view()));
  }
  {
    constexpr Vector v1(1.f, 2.f, 3.f), v2(4.f, 5.f, 3.f);
    const float d = dot(v1, v2);
    SHOW(d);
    const float m = mag(v1);
    SHOW(m);
    const Vector vcross = cross(v1, v2);
    SHOW(vcross);
    const Frame frame(v1, v1, v2, Point(10.f, 10.f, 10.f));
    const Point origin = frame.p();
    SHOW(origin);
  }
  {
    constexpr auto vdeg = deg_from_rad(D_TAU / 5);
    SHOW(vdeg);
  }
  {
    const Point p(1.f, 2.f, 3.f), q(8.f, 7.f, 6.f);
    SHOW(dist2(p, q));
    SHOW(dist(p, q));
    SHOW(dist2(p, (p + q) / 2.f));
    SHOW(dist(p, (p + q) / 2.f));
    SHOW(dist(p + (q - p), p));
    SHOW(dist2((p + q) / 2.f, (p + q) / 2.f));
    SHOW(mag2(p - q));
    SHOW(mag2(p - p));
  }
  {
    constexpr Frame frame(Vector(1.f, 0.f, 0.f), Vector(0.f, 1.f, 0.f), Vector(0.f, 0.f, 1.f),
                          Point(10.f, 20.f, 30.f));
    constexpr Point p = frame[3];
    constexpr Vector v0 = frame[0];
    constexpr Vector v1 = frame[1];
    SHOW(p, v0, v1);
  }
  {
    // Cross products in 2D and 3D.
    SHOW(cross(V(1.f, 0.f), V(0.f, 1.f)), cross(V(0.f, 1.f), V(1.f, 0.f)), cross(V(2.f, 3.f), V(4.f, 6.f)));
    const Vector vx(1.f, 0.f, 0.f), vy(0.f, 1.f, 0.f), vz(0.f, 0.f, 1.f);
    assertx(Vector(cross(vx, vy)) == vz && Vector(cross(vy, vz)) == vx && Vector(cross(vz, vx)) == vy);
    assertx(Vector(cross(vy, vx)) == -vz);
    const Point p1(1.f, 1.f, 1.f), p2(3.f, 1.f, 1.f), p3(1.f, 4.f, 1.f);
    SHOW(cross(p1, p2, p3));
    const Vec3<Point> triangle{p1, p2, p3};
    SHOW(get_normal_dir(triangle), get_normal(triangle));
    SHOW(area2(triangle), area2(p1, p2, p3));  // The area is 3.
    const Plane plane = plane_of_triangle(triangle);
    SHOW(plane.nor, plane.d);
    for_int(i, 3) assertx(dot(triangle[i], plane.nor) == plane.d);
    // Parallel vectors have a zero cross product, so a degenerate triangle has zero area.
    assertx(area2(Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f), Point(2.f, 2.f, 2.f)) == 0.f);
  }
  {
    // Normalization.
    SHOW(normalized(Vector(3.f, 0.f, 4.f)), mag(normalized(Vector(3.f, 0.f, 4.f))));
    SHOW(ok_normalized(Vector(0.f, 0.f, 0.f)));  // A zero vector stays zero.
    assertx(is_unit(normalized(Vector(1.f, 2.f, 3.f))) && !is_unit(Vector(1.f, 1.f, 0.f)));
    assertx(is_unit(Point(0.f, -1.f, 0.f)));
    SHOW(project_orthogonally(Vector(1.f, 2.f, 3.f), Vector(0.f, 0.f, 1.f)));
    const Vector unitdir = normalized(Vector(1.f, 1.f, 1.f));
    assertx(abs(dot(project_orthogonally(Vector(4.f, -1.f, 2.f), unitdir), unitdir)) < 1e-6f);
  }
  {
    // Frame constructors and predicates.
    assertx(Frame::identity().is_ident());
    assertx(!Frame::translation(V(0.f, 0.f, 1.f)).is_ident());
    assertx(!Frame::scaling(V(1.f, 1.f, 2.f)).is_ident());
    assertx(Frame::scaling(V(1.f, 1.f, 1.f)).is_ident() && Frame::translation(V(0.f, 0.f, 0.f)).is_ident());
    SHOW(Point(1.f, 2.f, 3.f) * Frame::translation(V(10.f, 20.f, 30.f)));
    SHOW(Vector(1.f, 2.f, 3.f) * Frame::translation(V(10.f, 20.f, 30.f)));  // Vectors ignore translation.
    SHOW(Point(1.f, 2.f, 3.f) * Frame::scaling(V(2.f, 3.f, 4.f)));
    // Rotations by multiples of TAU / 4 are exact because Frame::rotation() snaps tiny sines and cosines to zero.
    for_int(axis, 3) {
      const Frame rot = Frame::rotation(axis, TAU / 4);
      SHOW(axis, Point(1.f, 2.f, 3.f) * rot);
      assertx(rot * Frame::rotation(axis, -TAU / 4) == Frame::identity());
      assertx(Frame::rotation(axis, 0.f).is_ident());
      // A rotation is orthonormal, so its inverse is its transpose.
      const Frame rot2 = Frame::rotation(axis, .7f);
      assertx(near(transpose(rot2), ~rot2));
      assertx(near(rot2 * transpose(rot2), Frame::identity()));
    }
    // Compose rotations about the same axis.
    assertx(near(Frame::rotation(2, .3f) * Frame::rotation(2, .4f), Frame::rotation(2, .7f)));
    // Frame composition applies frame1 first, then frame2.
    const Frame frame1 = Frame::rotation(0, .5f) * Frame::translation(V(1.f, 2.f, 3.f));
    const Frame frame2 = Frame::scaling(V(2.f, .5f, 3.f)) * Frame::rotation(1, -.8f);
    const Point p(.3f, -.4f, 1.5f);
    assertx(near((p * frame1) * frame2, p * (frame1 * frame2)));
    assertx(!near((p * frame1) * frame2, p * (frame2 * frame1)));
    Frame frame3 = frame1;
    frame3 *= frame2;
    assertx(frame3 == frame1 * frame2);
    Point p2 = p;
    p2 *= frame3;
    assertx(p2 == p * frame3);
    Vector v2(1.f, 2.f, 3.f);
    v2 *= frame3;
    assertx(v2 == Vector(1.f, 2.f, 3.f) * frame3);
    // Normals transform by the inverse frame (applied on the left), so that they remain orthogonal to tangents.
    const Vector tangent(1.f, -2.f, .5f), nor(2.f, 1.f, 0.f);
    assertx(dot(tangent, nor) == 0.f);
    assertx(abs(dot(tangent * frame3, ~frame3 * nor)) < 1e-5f);
    // Frame::invert() and invert() fail on a singular frame.
    Frame singular = Frame::scaling(V(1.f, 0.f, 1.f));
    Frame dummy;
    assertx(!invert(singular, dummy));
    assertx(!singular.invert());
    Frame frame4 = frame3;
    assertx(frame4.invert());
    assertx(near(frame4 * frame3, Frame::identity()));
    // invert() allows its two arguments to alias.
    Frame frame5 = frame3;
    assertx(invert(frame5, frame5));
    assertx(near(frame5, frame4));
    // make_right_handed() negates the first axis of a left-handed frame.
    Frame frame6 = Frame::scaling(V(1.f, -1.f, 1.f));
    frame6.make_right_handed();
    assertx(frame6 == Frame::scaling(V(-1.f, -1.f, 1.f)));
    Frame frame7 = Frame::rotation(1, .2f);
    frame7.make_right_handed();
    assertx(frame7 == Frame::rotation(1, .2f));
    Frame frame8 = frame3;
    frame8.zero();
    assertx(frame8 == Frame::scaling(thrice(0.f)));
    // Mutable access through v() and p().
    Frame frame9 = Frame::identity();
    frame9.v(1) = Vector(0.f, 2.f, 0.f);
    frame9.p() = Point(7.f, 8.f, 9.f);
    SHOW(Point(1.f, 1.f, 1.f) * frame9);
  }
  {
    // Bary and Uv.
    assertx(Bary(.2f, .3f, .5f).is_convex() && Bary(1.f, 0.f, 0.f).is_convex());
    assertx(!Bary(-.1f, .6f, .5f).is_convex() && !Bary(1.1f, -.1f, 0.f).is_convex());
    const Uv uv(.25f, .75f);
    SHOW(uv, uv[0], uv[1]);
  }
  {
    // Interpolation.
    const Vec2<float> a0(0.f, 0.f), a1(1.f, 0.f), a2(1.f, 1.f), a3(0.f, 1.f);
    SHOW(bilerp(a0, a1, a2, a3, 0.f, 0.f), bilerp(a0, a1, a2, a3, 1.f, 0.f));
    SHOW(bilerp(a0, a1, a2, a3, 1.f, 1.f), bilerp(a0, a1, a2, a3, 0.f, 1.f));
    SHOW(bilerp(a0, a1, a2, a3, .25f, .5f));
    const Vec3<float> b1(1.f, 0.f, 0.f), b2(0.f, 1.f, 0.f), b3(0.f, 0.f, 1.f), b4(0.f, 0.f, 0.f);
    SHOW(qinterp(b1, b2, b3, b4, Bary(.25f, .25f, .25f)));  // The last weight is 1.f - .75f.
    SHOW(qinterp(b1, b2, b3, b4, Bary(0.f, 0.f, 0.f)));
  }
  {
    // Angle conversions.
    SHOW(rad_from_deg(180.), rad_from_deg(90.f), deg_from_rad(TAU / 4));
    assertx(abs(deg_from_rad(rad_from_deg(37.f)) - 37.f) < 1e-5f);
    // angle_between_unit_vectors() is accurate in all three of its regimes (near-parallel, generic, near-opposite).
    for (const double angle : {0., 1e-6, 1e-3, .1, .3, .32, .33, 1., 2., 2.8, 2.82, 3., 3.14, D_TAU / 2}) {
      const Vector v1(1.f, 0.f, 0.f);
      const Vector v2(float(std::cos(angle)), float(std::sin(angle)), 0.f);
      const float ang = angle_between_unit_vectors(v1, v2);
      assertx(abs(ang - angle) < 2e-6);
      // The Vec2<float> overload returns the same unsigned angle for both counterclockwise and clockwise rotations.
      const Vec2<float> w1(1.f, 0.f), w2(v2[0], v2[1]), w2_clockwise(v2[0], -v2[1]);
      assertx(abs(angle_between_unit_vectors(w1, w2) - angle) < 2e-6);
      assertx(abs(angle_between_unit_vectors(w1, w2_clockwise) - angle) < 2e-6);
      assertx(abs(angle_between_unit_vectors(w2_clockwise, w1) - angle) < 2e-6);
    }
    SHOW(rounded(angle_between_unit_vectors(Vector(0.f, 0.f, 1.f), normalized(Vector(1.f, 1.f, 1.f)))));
  }
  {
    // Spherical geometry.
    const Point px(1.f, 0.f, 0.f), py(0.f, 1.f, 0.f), pz(0.f, 0.f, 1.f);
    // Here slerp(p1, p2, f) moves from p2 (f == 0.f) to p1 (f == 1.f).
    assertx(near(slerp(px, py, 1.f), px) && near(slerp(px, py, 0.f), py));
    const Point pmid = slerp(px, py, .5f);
    assertx(near(pmid, normalized(Vector(1.f, 1.f, 0.f))));
    assertx(near(slerp(px, py, 1.f / 3.f), Point(std::cos(TAU / 6), std::sin(TAU / 6), 0.f)));
    assertx(slerp(px, px, .5f) == px);
    // An octant of the unit sphere has area 4 * pi / 8 == TAU / 4; the reversed triangle covers the other 7 octants.
    SHOW(rounded(spherical_triangle_area(V(px, py, pz)) / (TAU / 4)));
    SHOW(rounded(spherical_triangle_area(V(px, pz, py)) / (TAU / 4)));
    assertx(!spherical_triangle_is_flipped(V(px, py, pz)));
    assertx(spherical_triangle_is_flipped(V(px, pz, py)));
    assertx(!spherical_triangle_is_flipped<float>(V(px, py, pz)));
    assertx(!spherical_triangle_is_flipped(V(px, pz, py), 2.f));  // The determinant is -1.
    assertx(!spherical_triangle_is_flipped(V(px, py, -px)));      // The determinant is zero.
  }
  {
    // 2D signed area.
    SHOW(signed_area(V(0.f, 0.f), V(2.f, 0.f), V(0.f, 3.f)));
    SHOW(signed_area(V(0.f, 0.f), V(0.f, 3.f), V(2.f, 0.f)));
    SHOW(signed_area<float>(V(1.f, 1.f), V(2.f, 2.f), V(3.f, 3.f)));
  }
  {
    // point_inside() for a point in the plane of the triangle.
    const Vec3<Point> triangle{Point(0.f, 0.f, 1.f), Point(4.f, 0.f, 1.f), Point(0.f, 4.f, 1.f)};
    assertx(point_inside(Point(1.f, 1.f, 1.f), triangle));
    assertx(point_inside(Point(2.f, 2.f, 1.f), triangle));  // On the hypotenuse.
    assertx(point_inside(Point(0.f, 0.f, 1.f), triangle));  // At a vertex.
    assertx(point_inside(Point(2.f, 0.f, 1.f), triangle));  // On an edge.
    assertx(!point_inside(Point(3.f, 3.f, 1.f), triangle));
    assertx(!point_inside(Point(-.1f, 1.f, 1.f), triangle));
    assertx(!point_inside(Point(1.f, -.1f, 1.f), triangle));
    // Degenerate triangles: collinear vertices and coincident vertices.
    const Vec3<Point> segment{Point(0.f, 0.f, 0.f), Point(1.f, 0.f, 0.f), Point(2.f, 0.f, 0.f)};
    assertx(point_inside(Point(.5f, 0.f, 0.f), segment) && point_inside(Point(1.5f, 0.f, 0.f), segment));
    assertx(!point_inside(Point(2.5f, 0.f, 0.f), segment) && !point_inside(Point(1.f, .1f, 0.f), segment));
    const Vec3<Point> segment2{Point(0.f, 0.f, 0.f), Point(0.f, 0.f, 0.f), Point(0.f, 2.f, 0.f)};
    assertx(point_inside(Point(0.f, 1.f, 0.f), segment2) && !point_inside(Point(0.f, 3.f, 0.f), segment2));
    const Vec3<Point> single{Point(1.f, 1.f, 1.f), Point(1.f, 1.f, 1.f), Point(1.f, 1.f, 1.f)};
    assertx(point_inside(Point(1.f, 1.f, 1.f), single) && !point_inside(Point(1.f, 1.f, 2.f), single));
  }
  {
    // Barycentric coordinates of vectors, and their round trip through vector_from_bary().
    const Vec3<Vec2<float>> triangle2{V(0.f, 0.f), V(2.f, 0.f), V(0.f, 4.f)};
    const Bary bary2 = bary_of_vector(triangle2, V(1.f, 1.f));
    SHOW(transformed(bary2, rounded));
    assertx(abs(bary2[0] + bary2[1] + bary2[2]) < 1e-6f);
    const Vec3<Point> triangle{Point(1.f, 0.f, 0.f), Point(0.f, 1.f, 0.f), Point(0.f, 0.f, 1.f)};
    const Vector vec(.5f, -.25f, -.25f);  // In the plane of the triangle.
    const Bary bary = bary_of_vector(triangle, vec);
    SHOW(transformed(bary, rounded));
    assertx(near(vector_from_bary(triangle, bary), vec));
    SHOW(vector_from_bary(triangle, Bary(-1.f, 1.f, 0.f)));  // The edge vector from vertex 0 to vertex 1.
    // Each edge vector has barycentric coordinates (-1, 1, 0) up to permutation.
    assertx(near(bary_of_vector(triangle, triangle[1] - triangle[0]), Bary(-1.f, 1.f, 0.f)));
    assertx(near(bary_of_vector(triangle, triangle[0] - triangle[2]), Bary(1.f, 0.f, -1.f)));
  }
}
