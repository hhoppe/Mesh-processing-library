// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Geometry.h"

#include "libHh/Array.h"
#include "libHh/Facedistance.h"
#include "libHh/GeomOp.h"       // solid_angle()
#include "libHh/Homogeneous.h"  // centroid
#include "libHh/Matrix.h"
#include "libHh/MatrixOp.h"
#include "libHh/SGrid.h"

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

Frame operator*(const Frame& f1, const Frame& f2) {
  return Frame(f1.v(0)[0] * f2.v(0) + f1.v(0)[1] * f2.v(1) + f1.v(0)[2] * f2.v(2),
               f1.v(1)[0] * f2.v(0) + f1.v(1)[1] * f2.v(1) + f1.v(1)[2] * f2.v(2),
               f1.v(2)[0] * f2.v(0) + f1.v(2)[1] * f2.v(1) + f1.v(2)[2] * f2.v(2),
               f1.p()[0] * f2.v(0) + f1.p()[1] * f2.v(1) + f1.p()[2] * f2.v(2) + f2.p());
}

bool invert(const Frame& fi, Frame& fo) {
  // &fi == &fo is ok
  SGrid<float, 4, 4> m = to_Matrix(fi);
  if (!invert(m.const_view(), m.view())) return false;
  fo = to_Frame(m);
  return true;
}

bool Frame::invert() { return hh::invert(const_cast<const Frame&>(*this), *this); }

Frame transpose(const Frame& f) {
  assertx(f.p() == Point(0.f, 0.f, 0.f));
  Frame fo = f;
  std::swap(fo[1][0], fo[0][1]);
  std::swap(fo[2][0], fo[0][2]);
  std::swap(fo[2][1], fo[1][2]);
  return fo;
}

bool Frame::is_ident() const {
  return (v(0) == Vector(1.f, 0.f, 0.f) && v(1) == Vector(0.f, 1.f, 0.f) && v(2) == Vector(0.f, 0.f, 1.f) &&
          p() == Point(0.f, 0.f, 0.f));
}

Frame Frame::rotation(int axis, float angle) {
  assertx(axis >= 0 && axis < 3);
  Frame f = Frame::identity();
  float c = std::cos(angle), s = std::sin(angle);
  if (abs(c) < 1e-6f) c = 0.f;
  if (abs(s) < 1e-6f) s = 0.f;
  switch (axis) {
    case 0:
      f[0][0] = 1.f;
      f[1][1] = c;
      f[1][2] = s;
      f[2][1] = -s;
      f[2][2] = c;
      break;
    case 1:
      f[1][1] = 1.f;
      f[2][2] = c;
      f[2][0] = s;
      f[0][2] = -s;
      f[0][0] = c;
      break;
    case 2:
      f[2][2] = 1.f;
      f[0][0] = c;
      f[0][1] = s;
      f[1][0] = -s;
      f[1][1] = c;
      break;
    default: assertnever("");
  }
  return f;
}

std::ostream& operator<<(std::ostream& os, const Frame& f) {
  return os << "Frame {\n  v0 = " << f.v(0) << "\n  v1 = " << f.v(1) << "\n  v2 = " << f.v(2) << "\n  p = " << f.p()
            << "\n}\n";
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
  ASSERTX(sang >= -1e-6f && sang <= TAU * 2);
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
  if (!assertw(abs(denom) > 1e-10f)) denom = sign(denom) * 1e-10f;  // sign() does not return zero.
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
  assertx(is_unit(vortho));  // vector must be in plane of triangle.
  const float x1 = dot(v1, vn);
  const float x2 = dot(v2, vn);
  const float y1 = dot(v1, vortho);
  const float y2 = dot(v2, vortho);
  const float scale = mag(vec);
  float denom = y1 * x2 - y2 * x1;
  if (!assertw(abs(denom) > 1e-10f)) denom = sign(denom) * 1e-10f;  // sign() does not return zero.
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
