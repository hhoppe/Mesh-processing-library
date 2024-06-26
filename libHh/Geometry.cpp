// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Geometry.h"

#include "libHh/Array.h"
#include "libHh/GeomOp.h"       // solid_angle()
#include "libHh/Homogeneous.h"  // centroid
#include "libHh/Matrix.h"
#include "libHh/MatrixOp.h"
#include "libHh/SGrid.h"

namespace hh {

// *** Vector

Vector operator*(const Vector& v, const Frame& f) {
  float xi = v[0], yi = v[1], zi = v[2];
  return Vector(xi * f[0][0] + yi * f[1][0] + zi * f[2][0], xi * f[0][1] + yi * f[1][1] + zi * f[2][1],
                xi * f[0][2] + yi * f[1][2] + zi * f[2][2]);
}

Vector operator*(const Frame& f, const Vector& n) {
  float xi = n[0], yi = n[1], zi = n[2];
  return Vector(f[0][0] * xi + f[0][1] * yi + f[0][2] * zi, f[1][0] * xi + f[1][1] * yi + f[1][2] * zi,
                f[2][0] * xi + f[2][1] * yi + f[2][2] * zi);
}

// *** Point

Point operator*(const Point& p, const Frame& f) {
  float xi = p[0], yi = p[1], zi = p[2];
  return Point(xi * f[0][0] + yi * f[1][0] + zi * f[2][0] + f[3][0],
               xi * f[0][1] + yi * f[1][1] + zi * f[2][1] + f[3][1],
               xi * f[0][2] + yi * f[1][2] + zi * f[2][2] + f[3][2]);
}

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
  Vector v1 = to_Vector(p1), v2 = to_Vector(p2);
  float ang = angle_between_unit_vectors(v1, v2);
  ang *= ba;
  float vdot = dot(v1, v2);
  Vector vv = v1 - v2 * vdot;  // Tangent at p2 in direction of p1.
  vv.normalize();
  Vector v = v2 * std::cos(ang) + vv * std::sin(ang);
  ASSERTXX(is_unit(v));
  return to_Point(v);
}

// Spherical triangle area.
float spherical_triangle_area(const Vec3<Point>& pa) {
  for_int(i, 3) ASSERTXX(is_unit(pa[i]));
  float sang = solid_angle(Point(0.f, 0.f, 0.f), pa);
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

bool get_bary(const Point& p, const Vec3<Point>& pa, Bary& bary) {
  Vector nor = cross(pa[1] - pa[0], pa[2] - pa[0]);
  float area = float(mag<double>(nor));
  if (area <= 1e-10f) {
    fill(bary, 1.f / 3.f);
    return false;
  }
  for_int(i, 3) {
    Vector sn = cross(pa[mod3(i + 2)] - pa[mod3(i + 1)], p - pa[mod3(i + 1)]);
    bary[i] = mag(sn) / area * (dot(sn, nor) > 0.f ? 1.f : -1.f);
  }
  if (mag2(to_Vector(pa[0]) * bary[0] + to_Vector(pa[1]) * bary[1] + to_Vector(pa[2]) * bary[2] - to_Vector(p)) >=
      1e-10f)
    return false;
  return true;
}

Bary vector_bary(const Vec3<Point>& pa, const Vector& vec) {
  Vector vn = normalized(vec);
  Vector v1 = pa[1] - pa[0];
  Vector v2 = pa[2] - pa[0];
  Vector fnor = normalized(cross(v1, v2));
  Vector vortho = cross(fnor, vn);
  assertx(is_unit(vortho));  // vector must be in plane of triangle.
  float x1 = dot(v1, vn);
  float x2 = dot(v2, vn);
  float y1 = dot(v1, vortho);
  float y2 = dot(v2, vortho);
  float scale = mag(vec);
  float denom = y1 * x2 - y2 * x1;
  if (!assertw(abs(denom) > 1e-10f)) denom = sign(denom) * 1e-10f;  // sign() does not return zero
  float fac = scale / denom;
  float b1 = -y2 * fac, b2 = y1 * fac;
  return Bary(-b1 - b2, b1, b2);
}

Vector bary_vector(const Vec3<Point>& pa, const Bary& bary) {
  assertw(abs(bary[0] + bary[1] + bary[2] - 0.f) < 1e-4f);
  return Vector(interp(pa[0], pa[1], pa[2], bary));  // note that bary[0] + bary[1] + bary[2] == 0.f not 1.f
}

}  // namespace hh
