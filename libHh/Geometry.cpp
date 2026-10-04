// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Geometry.h"

#include "libHh/Array.h"
#include "libHh/Facedistance.h"
#include "libHh/GeomOp.h"       // solid_angle()
#include "libHh/Homogeneous.h"  // centroid
#include "libHh/Matrix.h"
#include "libHh/MatrixOp.h"
#include "libHh/Vec.h"

namespace hh {

namespace {

std::optional<Bary> bary_of_point(const Vec3<Point>& triangle, const Point& p) {
  const auto& [p1, p2, p3] = triangle;
  const Vector v2 = p2 - p1, v3 = p3 - p1, vp = p - p1;
  const float v2v2 = mag2(v2), v3v3 = mag2(v3), v2v3 = dot(v2, v3);
  if (!v2v2) return {};
  const float denom = v3v3 - v2v3 * v2v3 / v2v2;
  if (!denom) return {};
  const float v2vp = dot(v2, vp), v3vp = dot(v3, vp);
  const float b3 = (v3vp - v2v3 / v2v2 * v2vp) / denom;
  const float b2 = (v2vp - b3 * v2v3) / v2v2;
  const float b1 = 1.f - b2 - b3;
  return Bary(b1, b2, b3);
}

}  // namespace

// *** Frame

Frame Frame::translation(const Vec3<float>& ar) {
  return Frame(Vector(1.f, 0.f, 0.f), Vector(0.f, 1.f, 0.f), Vector(0.f, 0.f, 1.f), ar);
}

Frame Frame::scaling(const Vec3<float>& ar) {
  return Frame(Vector(ar[0], 0.f, 0.f), Vector(0.f, ar[1], 0.f), Vector(0.f, 0.f, ar[2]), Point(0.f, 0.f, 0.f));
}

Frame Frame::identity() {
  return Frame(Vector(1.f, 0.f, 0.f), Vector(0.f, 1.f, 0.f), Vector(0.f, 0.f, 1.f), Point(0.f, 0.f, 0.f));
}

Frame operator*(const Frame& frame1, const Frame& frame2) {
  return Frame(frame1.v(0)[0] * frame2.v(0) + frame1.v(0)[1] * frame2.v(1) + frame1.v(0)[2] * frame2.v(2),
               frame1.v(1)[0] * frame2.v(0) + frame1.v(1)[1] * frame2.v(1) + frame1.v(1)[2] * frame2.v(2),
               frame1.v(2)[0] * frame2.v(0) + frame1.v(2)[1] * frame2.v(1) + frame1.v(2)[2] * frame2.v(2),
               frame1.p()[0] * frame2.v(0) + frame1.p()[1] * frame2.v(1) + frame1.p()[2] * frame2.v(2) + frame2.p());
}

bool invert(const Frame& frame, Frame& frame_inv) {
  // Here, &frame == &frame_inv is OK.
  // The frame maps a point x to x * A + p, where the rows of A are frame.v(0..2), so its inverse maps y to
  // y * A^-1 - p * A^-1.  Inverting just A, rather than the homogeneous 4x4 matrix [A 0; p 1], keeps a large
  // translation from affecting the result: the condition number of the 4x4 matrix grows as |p|^2, so that a frame
  // with a well-conditioned A but a translation of about 1e8 would be deemed singular.
  SGrid<float, 3, 3> a;
  for_int(i, 3) a[i] = frame.v(i);
  if (!invert(a.const_grid_view(), a.grid_view())) return false;
  const Point p = frame.p();
  frame_inv = Frame(Vector(a[0]), Vector(a[1]), Vector(a[2]), Point(-(p[0] * a[0] + p[1] * a[1] + p[2] * a[2])));
  return true;
}

bool Frame::invert() { return hh::invert(const_cast<const Frame&>(*this), *this); }

Frame transpose(const Frame& frame) {
  assertx(frame.p() == Point(0.f, 0.f, 0.f));
  Frame frame2 = frame;
  std::swap(frame2[1, 0], frame2[0, 1]);
  std::swap(frame2[2, 0], frame2[0, 2]);
  std::swap(frame2[2, 1], frame2[1, 2]);
  return frame2;
}

bool Frame::is_ident() const {
  return (v(0) == Vector(1.f, 0.f, 0.f) && v(1) == Vector(0.f, 1.f, 0.f) && v(2) == Vector(0.f, 0.f, 1.f) &&
          p() == Point(0.f, 0.f, 0.f));
}

Frame Frame::rotation(int axis, float angle) {
  assertx(axis >= 0 && axis < 3);
  Frame frame = Frame::identity();
  float c = std::cos(angle), s = std::sin(angle);
  // Make rotations by multiples of TAU / 4 exact.  Snapping one value to zero also snaps the other one to +-1, because
  // a less accurate sin() or cos() (e.g., with MSVC /fp:fast, std::sin(TAU / 4) == 0.99999994f) need not give +-1.
  if (abs(c) < 1e-6f) c = 0.f, s = sign(s);
  if (abs(s) < 1e-6f) s = 0.f, c = sign(c);
  switch (axis) {
    case 0:
      frame[0, 0] = 1.f;
      frame[1, 1] = c;
      frame[1, 2] = s;
      frame[2, 1] = -s;
      frame[2, 2] = c;
      break;
    case 1:
      frame[1, 1] = 1.f;
      frame[2, 2] = c;
      frame[2, 0] = s;
      frame[0, 2] = -s;
      frame[0, 0] = c;
      break;
    case 2:
      frame[2, 2] = 1.f;
      frame[0, 0] = c;
      frame[0, 1] = s;
      frame[1, 0] = -s;
      frame[1, 1] = c;
      break;
    default: assertnever("");
  }
  return frame;
}

std::ostream& operator<<(std::ostream& os, const Frame& frame) {
  return os << "Frame {\n  v0 = " << frame.v(0) << "\n  v1 = " << frame.v(1) << "\n  v2 = " << frame.v(2)
            << "\n  p = " << frame.p() << "\n}\n";
}

// *** Misc

Point slerp(const Point& p1, const Point& p2, float ba) {
  const float ang = angle_between_unit_vectors(p1, p2) * ba;
  const float vdot = dot<float>(p1, p2);
  const Vector vv = ok_normalized(p1 - p2 * vdot);  // Tangent at p2 in direction of p1.
  const Vector v = p2 * std::cos(ang) + vv * std::sin(ang);
  ASSERTXX(is_unit(v));
  return v;
}

// Spherical triangle area.
float spherical_triangle_area(const Vec3<Point>& triangle) {
  for_int(i, 3) ASSERTXX(is_unit(triangle[i]));
  const float sang = solid_angle(Point(0.f, 0.f, 0.f), triangle);
  if (sang < 0.f) {
    assertx(sang >= -1e-6f);
    return 0.f;
  }
  if (sang > TAU * 2 - 1e-3f) {
    assertx(sang <= TAU * 2 + 1e-3f);
    return 0.f;
  }
  return sang;
}

bool point_inside(const Point& p, const Vec3<Point>& triangle) {
  if (const auto result = bary_of_point(triangle, p)) {
    const Bary& bary = *result;
    return bary[0] >= 0.f && bary[1] >= 0.f && bary[2] >= 0.f;
  }
  // If the triangle is degenerate, we defer to the more complicated algorithm that considers the triangle sides.
  return project_point_triangle(p, triangle).d2 < 1e-12f;
}

Bary bary_of_vector(const Vec3<Vec2<float>>& triangle, const Vec2<float>& vec) {
  const Vec2<float> v1 = triangle[1] - triangle[0];
  const Vec2<float> v2 = triangle[2] - triangle[0];
  const Vec2<float> vn = normalized(vec);
  const Vec2<float> vortho = V(-vn[1], vn[0]);
  const float x1 = dot(v1, vn);
  const float x2 = dot(v2, vn);
  const float y1 = dot(v1, vortho);
  const float y2 = dot(v2, vortho);
  const float scale = mag(vec);
  float denom = y1 * x2 - y2 * x1;
  if (!assertw(abs(denom) > 1e-10f)) denom = sign(denom) * 1e-10f;  // Here, sign() does not return zero.
  const float fac = scale / denom;
  const float b1 = -y2 * fac, b2 = y1 * fac;
  return Bary(-b1 - b2, b1, b2);
}

Bary bary_of_vector(const Vec3<Point>& triangle, const Vector& vec) {
  const Vector v1 = triangle[1] - triangle[0];
  const Vector v2 = triangle[2] - triangle[0];
  const Vector vn = normalized(vec);
  const Vector fnor = normalized(cross(v1, v2));
  const Vector vortho = cross(fnor, vn);
  assertx(is_unit(vortho));  // The vector must be in the plane of the triangle.
  const float x1 = dot(v1, vn);
  const float x2 = dot(v2, vn);
  const float y1 = dot(v1, vortho);
  const float y2 = dot(v2, vortho);
  const float scale = mag(vec);
  float denom = y1 * x2 - y2 * x1;
  if (!assertw(abs(denom) > 1e-10f)) denom = sign(denom) * 1e-10f;  // Here, sign() does not return zero.
  const float fac = scale / denom;
  const float b1 = -y2 * fac, b2 = y1 * fac;
  return Bary(-b1 - b2, b1, b2);
}

Vector vector_from_bary(const Vec3<Point>& triangle, const Bary& bary) {
  assertw(abs(bary[0] + bary[1] + bary[2] - 0.f) < 1e-4f);
  // Note that bary[0] + bary[1] + bary[2] == 0.f, not 1.f .
  return Vector(interp(triangle, bary));
}

}  // namespace hh
