// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_GEOMETRY_H_
#define MESH_PROCESSING_LIBHH_GEOMETRY_H_

#include "libHh/SGrid.h"
#include "libHh/Vec.h"

namespace hh {

class Frame;
struct Point;

// My Vector, Point, Frame classes assumes row vectors (rather than column vectors).
// This is similar to RenderMan.
// https://en.wikipedia.org/wiki/Row_vector
// https://community.khronos.org/t/transpose/43582
//   OpenGL was long ago derived from SGI's proprietary graphics library 'Irix GL'
//   In its docs, matrix ops were specified as operating on row vectors on the matrix left.

// *** Vector (lives in 3D linear space; represents translation rather than position).
struct Vector : Vec3<float> {
  Vector() = default;
  constexpr Vector(float x, float y, float z) : Vec3<float>(x, y, z) {}
  constexpr Vector(Vec3<float> v) : Vec3<float>(v) {}
};
Vector operator*(const Vector& v, const Frame& f);
Vector operator*(const Frame& f, const Vector& normal);
inline Vector& operator*=(Vector& v, const Frame& f) { return v = v * f; }
constexpr Vector cross(const Vector& v1, const Vector& v2);
inline Vector normalized(Vector v) {
  assertx(v.normalize());
  return v;
}
inline Vector ok_normalized(Vector v) {
  v.normalize();
  return v;
}
// Vec.h defines explicit low-precision versions of these functions for Vec3<T>.  Because Vector only inherits from
// Vec3<float>, we must overload several versions to hide the influence of the templated functions in RangeOp.h.
inline constexpr float dot(const Vec3<float>& v1, const Vector& v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}
inline constexpr float dot(const Vector& v1, const Vec3<float>& v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}
inline constexpr float dot(const Vector& v1, const Vector& v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}
inline constexpr float mag2(const Vector& v1) { return dot(v1, v1); }
inline float mag(const Vector& v1) { return sqrt(mag2(v1)); }

// *** Point (lives in 3D affine space; represents position rather than translation).
struct Point : Vec3<float> {
  Point() = default;
  constexpr Point(float x, float y, float z) : Vec3<float>(x, y, z) {}
  constexpr Point(Vec3<float> p) : Vec3<float>(p) {}
};
Point operator*(const Point& p, const Frame& f);
inline Point& operator*=(Point& p, const Frame& f) { return p = p * f; }
inline Vector to_Vector(const Point& p) { return Vector(p[0], p[1], p[2]); }
inline Point to_Point(const Vector& v) { return Point(v[0], v[1], v[2]); }
inline float pvdot(const Point& p, const Vector& v) { return dot(to_Vector(p), v); }
Vector cross(const Point& p1, const Point& p2, const Point& p3);
inline Vector get_normal_dir(const Vec3<Point>& triangle) { return cross(triangle[0], triangle[1], triangle[2]); }
inline Vector get_normal(const Vec3<Point>& triangle) { return normalized(get_normal_dir(triangle)); }
inline float area2(const Point& p1, const Point& p2, const Point& p3) { return .25f * mag2(cross(p1, p2, p3)); }
inline float area2(const Vec3<Point>& triangle) { return area2(triangle[0], triangle[1], triangle[2]); }
inline float dist2(const Vec3<float>& v1, const Point& v2) {
  return square(v1[0] - v2[0]) + square(v1[1] - v2[1]) + square(v1[2] - v2[2]);
}
inline float dist2(const Point& v1, const Vec3<float>& v2) {
  return square(v1[0] - v2[0]) + square(v1[1] - v2[1]) + square(v1[2] - v2[2]);
}
inline float dist2(const Point& v1, const Point& v2) {
  return square(v1[0] - v2[0]) + square(v1[1] - v2[1]) + square(v1[2] - v2[2]);
}
inline float dist(const Vec3<float>& v1, const Point& v2) { return sqrt(dist2(v1, v2)); }
inline float dist(const Point& v1, const Vec3<float>& v2) { return sqrt(dist2(v1, v2)); }
inline float dist(const Point& v1, const Point& v2) { return sqrt(dist2(v1, v2)); }

inline bool is_unit(const Vec3<float>& v) { return abs(mag2(v) - 1.f) < 1e-4f; }

// *** Frame: either coordinate frame or transformation frame.
//  Affine transformation from 3D to 3D.  (new_Point, 1.f) = (Point, 1.f) * Frame,
//    where (Point, 1.f) is *row* 4-vector.
//  Can be interpreted as 4x3 matrix where rows 0..2 are mappings of axes 0..2, and row 3 is mapping of origin.
//  Note that Vector * Frame correctly assumes a row 4-vector (Vector, 0.f).
class Frame : public SGrid<float, 4, 3> {
  using base = SGrid<float, 4, 3>;

 public:
  Frame() = default;
  Frame(Vector v0, Vector v1, Vector v2, Point q) : base(V<Vec3<float>>(v0, v1, v2, q)) {}
  Vector& v(int i) { return (HH_CHECK_BOUNDS(i, 3), static_cast<Vector&>((*this)[i])); }
  const Vector& v(int i) const { return (HH_CHECK_BOUNDS(i, 3), static_cast<const Vector&>((*this)[i])); }
  Point& p() { return static_cast<Point&>((*this)[3]); }
  const Point& p() const { return static_cast<const Point&>((*this)[3]); }
  void zero() { fill(*this, 0.f); }
  bool is_ident() const;
  bool invert();
  void make_right_handed() {
    if (dot(cross(v(0), v(1)), v(2)) < 0) v(0) = -v(0);
  }
  static Frame translation(const Vec3<float>& ar);
  static Frame rotation(int axis, float angle);
  static Frame scaling(const Vec3<float>& ar);
  static Frame identity();
};
Frame operator*(const Frame& f1, const Frame& f2);
inline Frame& operator*=(Frame& f1, const Frame& f2) { return f1 = f1 * f2; }
bool invert(const Frame& fi, Frame& fo);
inline Frame inverse(const Frame& f) {
  Frame fr;
  assertx(invert(f, fr));
  return fr;
}
inline Frame operator~(const Frame& f) { return inverse(f); }
Frame transpose(const Frame& f);
std::ostream& operator<<(std::ostream& os, const Frame& f);
template <> HH_DECLARE_OSTREAM_EOL(Frame);

// *** Barycentric coordinates; sum to 1.f when expressing a Point as combination of 3 Points
//   or a Vector as a combination of 3 Vectors, but sums to 0.f when expressing a Vector as combination of 3 Points.
struct Bary : Vec3<float> {
  Bary() = default;
  constexpr Bary(float x, float y, float z) : Vec3<float>(x, y, z) {}
  constexpr Bary(Vec3<float> v) : Vec3<float>(v) {}
  bool is_convex() const;
};

// *** Texture coordinates; usually defined over unit square [0, 1]^2.
// In DirectX, Metal, and Vulkan, the Uv origin is at the top left corner of a texture image, whereas
// in OpenGL the Uv origin at the lower left.
// Unfortunately, my code/results uses the OpenGL convention (even though my yx pixel coordinates have their
// origin at the top left of the image).
struct Uv : Vec2<float> {
  Uv() = default;
  constexpr Uv(float u, float v) : Vec2<float>(u, v) {}
  constexpr Uv(Vec2<float> v) : Vec2<float>(v) {}
};

// *** Misc operations

// Project v into plane orthogonal to unitdir.
Vector project_orthogonally(const Vector& v, const Vector& unitdir);

// General affine combination of 4 points.  Note that interpolation domain is 3-dimensional (non-planar).
// "bary[3]" == 1.f - bary[0] - bary[1] - bary[2]
template <typename T, int n>
Vec<T, n> qinterp(const Vec<T, n>& a1, const Vec<T, n>& a2, const Vec<T, n>& a3, const Vec<T, n>& a4,
                  const Bary& bary);

// Bilinear interpolation within 4 points.
// p3 - p2   v
//  |    |   ^
// p0 - p1   |  ->u
template <typename T, int n>
Vec<T, n> bilerp(const Vec<T, n>& a0, const Vec<T, n>& a1, const Vec<T, n>& a2, const Vec<T, n>& a3, float u, float v);

// Spherical linear interpolation.  slerp(p1, p2, 1.f) == p1.
Point slerp(const Point& p1, const Point& p2, float ba);

// Spherical triangle area.
float spherical_triangle_area(const Vec3<Point>& triangle);

// Determine if an oriented spherical triangle spans more than a hemisphere.
template <typename Precision = double>
bool spherical_triangle_is_flipped(const Vec3<Point>& triangle, float tolerance = 0.f);

// Return signed area of 2D triangle (positive if counter-clockwise).
template <typename Precision = double>
float signed_area(const Vec2<float>& p1, const Vec2<float>& p2, const Vec2<float>& p3);

// Given a 3D point in the plane of a triangle, return the point's barycentric coordinates.
Bary bary_of_point(const Vec3<Point>& triangle, const Point& p);

// Given a 3D point in the plane of a triangle, return whether the point lies in the triangle convex hull.
bool point_inside(const Point& p, const Vec3<Point>& triangle);

// Given a 2D vector and a 2D triangle, return the vector's barycentric coordinates.
Bary bary_of_vector(const Vec3<Vec2<float>>& triangle, const Vec2<float>& vec);

// Given a 3D vector in the plane of a triangle, return the vector's barycentric coordinates.
Bary bary_of_vector(const Vec3<Point>& triangle, const Vector& vec);

// Given a triangle and barycentric coordinates, return the vector.
Vector vector_from_bary(const Vec3<Point>& triangle, const Bary& bary);

// Convert degrees to radians.
template <typename T> constexpr T to_rad(T deg) {
  static_assert(std::is_floating_point_v<T>);
  return deg * static_cast<T>(D_TAU / 360);
}

// Convert radians to degrees.
template <typename T> constexpr T to_deg(T rad) {
  static_assert(std::is_floating_point_v<T>);
  return rad * static_cast<T>(360 / D_TAU);
}

//----------------------------------------------------------------------------

// *** Vector

inline constexpr Vector cross(const Vector& v1, const Vector& v2) {
  return Vector(v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]);
}

// *** Point

inline Vector cross(const Point& p1, const Point& p2, const Point& p3) {
  // return cross(p2 - p1, p3 - p1);
  // It once seemed that "double" was needed below to overcome an apparent problem with poorly computed surface
  // normals.  However, the problem lay in the geometry. Prefiltering with "Filtermesh -taubinsmooth 4" solved it.
  float p1x = p1[0], p1y = p1[1], p1z = p1[2];
  float v1x = p2[0] - p1x, v1y = p2[1] - p1y, v1z = p2[2] - p1z;
  float v2x = p3[0] - p1x, v2y = p3[1] - p1y, v2z = p3[2] - p1z;
  return Vector(v1y * v2z - v1z * v2y, v1z * v2x - v1x * v2z, v1x * v2y - v1y * v2x);
}

// *** Bary

inline bool Bary::is_convex() const {
  auto& self = *this;
  return self[0] >= 0.f && self[0] <= 1.f && self[1] >= 0.f && self[1] <= 1.f && self[2] >= 0.f && self[2] <= 1.f;
}

// *** Uv

// *** Misc

inline Vector project_orthogonally(const Vector& v, const Vector& unitdir) {
  ASSERTXX(is_unit(unitdir));
  return v - unitdir * dot(v, unitdir);
}

template <typename T, int n>
Vec<T, n> qinterp(const Vec<T, n>& a1, const Vec<T, n>& a2, const Vec<T, n>& a3, const Vec<T, n>& a4,
                  const Bary& bary) {
  return bary[0] * a1 + bary[1] * a2 + bary[2] * a3 + (1.f - bary[0] - bary[1] - bary[2]) * a4;
}

template <typename T, int n>
Vec<T, n> bilerp(const Vec<T, n>& a0, const Vec<T, n>& a1, const Vec<T, n>& a2, const Vec<T, n>& a3, float u,
                 float v) {
  // return qinterp(a0, a1, a2, a3, Bary((1.f - u) * (1.f - v), u * (1.f - v), u * v));
  return interp(interp(a0, a1, 1.f - u), interp(a3, a2, 1.f - u), 1.f - v);
}

template <typename Precision> bool spherical_triangle_is_flipped(const Vec3<Point>& triangle, float tolerance) {
  // The signed volume of the tetrahedron formed by the origin and the points p1, p2, and p3 is given by
  //  (1.f/6.f) * dot(p1, cross(p2, p3).
  // return dot(triangle[0], cross(triangle[1], triangle[2])) < 0.f;
  const Point& p1 = triangle[1];
  const Point& p2 = triangle[2];
  const Vec3<Precision> vcross(Precision(p1[1]) * p2[2] - Precision(p1[2]) * p2[1],
                               Precision(p1[2]) * p2[0] - Precision(p1[0]) * p2[2],
                               Precision(p1[0]) * p2[1] - Precision(p1[1]) * p2[0]);
  return dot<Precision>(triangle[0], vcross) < -tolerance;
}

template <typename Precision> float signed_area(const Vec2<float>& p1, const Vec2<float>& p2, const Vec2<float>& p3) {
  const Precision y1 = p1[1], y2 = p2[1], y3 = p3[1];
  return 0.5f * float(p1[0] * (y2 - y3) + p2[0] * (y3 - y1) + p3[0] * (y1 - y2));
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_GEOMETRY_H_
