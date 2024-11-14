// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_GEOMETRY_H_
#define MESH_PROCESSING_LIBHH_GEOMETRY_H_

#include "libHh/SGrid.h"
#include "libHh/Vec.h"

namespace hh {

// For these templated functions, the caller should convert<> the inputs to obtain higher precision.

// Return the vector cross-product in 2D.  The signed area of a 2D triangle (origin, v1, v2) is 0.5f * cross(v1, v2).
template <typename T> T cross(const Vec2<T>& v1, const Vec2<T>& v2) {
  static_assert(std::is_floating_point_v<T>);
  return v1[0] * v2[1] - v1[1] * v2[0];
}

// Return the vector cross-product in 3D.
template <typename T> Vec3<T> cross(const Vec3<T>& v1, const Vec3<T>& v2) {
  static_assert(std::is_floating_point_v<T>);
  return Vec3<T>(v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]);
}

// Return the Unnormalized cross-product given by three 3D points.
template <typename T> Vec3<T> cross(const Vec3<T>& p1, const Vec3<T>& p2, const Vec3<T>& p3) {
  return cross(p2 - p1, p3 - p1);
}

class Frame;
struct Point;

// My Vector, Point, Frame classes assumes row vectors (rather than column vectors).
// This is similar to RenderMan.
// https://en.wikipedia.org/wiki/Row_vector
// https://community.khronos.org/t/transpose/43582
//   OpenGL was long ago derived from SGI's proprietary graphics library 'Irix GL'
//   In its docs, matrix ops were specified as operating on row vectors on the matrix left.

// *** Vector

// A vector in a 3D linear space; it represents translation rather than position.
struct Vector : Vec3<float> {
  Vector() = default;
  constexpr Vector(float x, float y, float z) : Vec3<float>(x, y, z) {}
  constexpr Vector(Vec3<float> v) : Vec3<float>(v) {}
};

inline Vector operator*(const Vector& v, const Frame& frame);    // Transform a vector by a frame.
inline Vector operator*(const Frame& frame, const Vector& nor);  // Transform a normal by a frame.
inline Vector& operator*=(Vector& v, const Frame& frame) { return v = v * frame; }

inline Vector normalized(Vector v) { return assertx(v.normalize()), v; }
inline Vector ok_normalized(Vector v) { return v.normalize(), v; }

// Overload these to refer to low-precision versions from Vec.h and not high-precision versions from RangeOp.h
inline float dot(const Vec3<float>& v1, const Vector& v2) { return dot(v1.vec(), v2.vec()); }
inline float dot(const Vector& v1, const Vec3<float>& v2) { return dot(v1.vec(), v2.vec()); }
inline float dot(const Vector& v1, const Vector& v2) { return dot(v1.vec(), v2.vec()); }
inline float mag2(const Vector& v1) { return mag2(v1.vec()); }
inline float mag(const Vector& v1) { return mag(v1.vec()); }
inline bool is_unit(const Vector& v) { return is_unit(v.vec()); }

// *** Point

// A point in a 3D affine space; it represents position rather than translation.
struct Point : Vec3<float> {
  Point() = default;
  constexpr Point(float x, float y, float z) : Vec3<float>(x, y, z) {}
  constexpr Point(Vec3<float> p) : Vec3<float>(p) {}
};

inline Point operator*(const Point& p, const Frame& frame);  // Transform a point by a frame.
inline Point& operator*=(Point& p, const Frame& frame) { return p = p * frame; }

inline Vector get_normal_dir(const Vec3<Point>& triangle) { return cross(triangle[0], triangle[1], triangle[2]); }
inline Vector get_normal(const Vec3<Point>& triangle) { return normalized(get_normal_dir(triangle)); }

inline float area2(const Point& p1, const Point& p2, const Point& p3) { return .25f * mag2(cross(p1, p2, p3)); }
inline float area2(const Vec3<Point>& triangle) { return area2(triangle[0], triangle[1], triangle[2]); }

// Overload these to refer to low-precision versions from Vec.h and not high-precision versions from RangeOp.h
inline float dist2(const Vec3<float>& v1, const Point& v2) { return dist2(v1.vec(), v2.vec()); }
inline float dist2(const Point& v1, const Vec3<float>& v2) { return dist2(v1.vec(), v2.vec()); }
inline float dist2(const Point& v1, const Point& v2) { return dist2(v1.vec(), v2.vec()); }
inline float dist(const Vec3<float>& v1, const Point& v2) { return dist(v1.vec(), v2.vec()); }
inline float dist(const Point& v1, const Vec3<float>& v2) { return dist(v1.vec(), v2.vec()); }
inline float dist(const Point& v1, const Point& v2) { return dist(v1.vec(), v2.vec()); }
inline float dot(const Point& v1, const Vector& v2) { return dot(v1.vec(), v2.vec()); }
inline bool is_unit(const Point& p) { return is_unit(p.vec()); }

// *** Frame

// It encodes a 4x3 matrix representing either a coordinate frame or an affine transformation frame.
// This matrix is applied to the *right* of a *row* vector:
//   rows 0..2 are mappings of the axes 0..2, and row 3 is the mapping of the origin.
// For the computation { Point p; Frame frame; Point newp = p * frame; },
//   the evaluation is:   (newp[0], newp[1], newp[2], 1.f) = (p[0], p[1], p[2], 1.f) * frame,
//   where (p[0], p[1], p[2], 1.f) is a *row* 4-vector.
// For the computation { Vector v; Frame frame; Vector newv = v * frame; },
//   the evaluation is:   (newv[0], newv[1], newv[2], 0.f) = (v[0], v[1], v[2], 0.f) * frame.
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
    if (dot(cross(v(0), v(1)), v(2)) < 0.f) v(0) = -v(0);
  }
  static Frame translation(const Vec3<float>& ar);
  static Frame rotation(int axis, float angle);
  static Frame scaling(const Vec3<float>& ar);
  static Frame identity();
};

Frame operator*(const Frame& frame1, const Frame& frame2);  // Compose two frames (frame2 after frame1).
inline Frame& operator*=(Frame& frame1, const Frame& frame2) { return frame1 = frame1 * frame2; }

bool invert(const Frame& frame, Frame& frame_inv);
inline Frame inverse(const Frame& frame) {
  Frame frame_inv;
  assertx(invert(frame, frame_inv));
  return frame_inv;
}
inline Frame operator~(const Frame& frame) { return inverse(frame); }

Frame transpose(const Frame& frame);

std::ostream& operator<<(std::ostream& os, const Frame& frame);
template <> HH_DECLARE_OSTREAM_EOL(Frame);

// *** Bary

// Barycentric coordinates;
// The coords sum to 1.f when expressing a Point as a combination of 3 Points, or a Vector as a combo of 3 Vectors.
// The coords sum to 0.f when expressing a Vector as combination of 3 Points.
struct Bary : Vec3<float> {
  Bary() = default;
  constexpr Bary(float x, float y, float z) : Vec3<float>(x, y, z) {}
  constexpr Bary(Vec3<float> v) : Vec3<float>(v) {}
  bool is_convex() const;
};

// *** Uv

// 2D coordinates, often used to represent texture coordinates; usually defined over unit square [0, 1]^2.
// In DirectX, Metal, and Vulkan, the Uv origin is at the top left corner of a texture image, whereas
// in OpenGL the Uv origin at the lower left.
// Unfortunately, my code/results uses the OpenGL convention (even though my yx pixel coordinates have their
// origin at the top left of the image).
struct Uv : Vec2<float> {
  Uv() = default;
  constexpr Uv(float u, float v) : Vec2<float>(u, v) {}
  constexpr Uv(Vec2<float> v) : Vec2<float>(v) {}
};

// *** Misc

// Line in 3D; is_on_line(Point p) = p == line.point + t * line.vec  (for some arbitrary float t).
struct Line {
  Point point;
  Vector vec;
};

// Plane in 3D; is_on_plane(Point p) = dot(p, plane.nor) == plane.d.
struct Plane {
  Vector nor;
  float d;
};

inline Plane plane_of_triangle(const Vec3<Point>& triangle) {
  const Vector nor = normalized(get_normal_dir(triangle));
  const float d = dot(sum(triangle), nor) / 3.f;
  return {nor, d};
}

// Project v into plane orthogonal to unitdir.
Vector project_orthogonally(const Vector& v, const Vector& unitdir);

// General affine combination of 4 points.  Note that interpolation domain is 3-dimensional (non-planar).
// "bary[3]" == 1.f - bary[0] - bary[1] - bary[2].
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

// Given a 3D point in the plane of a triangle, return whether the point lies in the triangle convex hull.
bool point_inside(const Point& p, const Vec3<Point>& triangle);

// Given a 2D vector and a 2D triangle, return the vector's barycentric coordinates.
Bary bary_of_vector(const Vec3<Vec2<float>>& triangle, const Vec2<float>& vec);

// Given a 3D vector in the plane of a triangle, return the vector's barycentric coordinates.
Bary bary_of_vector(const Vec3<Point>& triangle, const Vector& vec);

// Given a triangle and barycentric coordinates, return the vector.
Vector vector_from_bary(const Vec3<Point>& triangle, const Bary& bary);

// Convert degrees to radians.
template <typename T> constexpr T rad_from_deg(T deg) {
  static_assert(std::is_floating_point_v<T>);
  return deg * static_cast<T>(D_TAU / 360);
}

// Convert radians to degrees.
template <typename T> constexpr T deg_from_rad(T rad) {
  static_assert(std::is_floating_point_v<T>);
  return rad * static_cast<T>(360 / D_TAU);
}

// More robust than acos(dot()) for small angles.
template <typename T> T angle_between_unit_vectors(const Vec3<T>& v1, const Vec3<T>& v2) {
  ASSERTXX(is_unit(v1) && is_unit(v2));
  const T vdot = dot(v1, v2);
  const float thresh = 0.9475f;  // Empirically from Python determine_crossover_for_acos_angle_approximation().
  if (vdot > +thresh) {
    return std::asin(mag(cross(v1, v2)));
  } else if (vdot < -thresh) {
    return T(D_TAU / 2) - std::asin(mag(cross(v1, v2)));
  } else {
    return std::acos(vdot);
  }
}

// More robust than acos(dot()) for small angles.
template <typename T> T angle_between_unit_vectors(const Vec2<T>& v1, const Vec2<T>& v2) {
  ASSERTXX(is_unit(v1) && is_unit(v2));
  const T vdot = dot(v1, v2);
  const float thresh = 0.9475f;
  if (vdot > +thresh) {
    return std::asin(cross(v1, v2));
  } else if (vdot < -thresh) {
    return T(D_TAU / 2) - std::asin(cross(v1, v2));
  } else {
    return std::acos(vdot);
  }
}

//----------------------------------------------------------------------------

// *** Vector

inline Vector operator*(const Vector& v, const Frame& frame) {
  return v[0] * frame[0] + v[1] * frame[1] + v[2] * frame[2];
}

inline Vector operator*(const Frame& frame, const Vector& nor) {
  return Vector(dot(frame[0], nor), dot(frame[1], nor), dot(frame[2], nor));
}

// *** Point

inline Point operator*(const Point& p, const Frame& frame) {
  return p[0] * frame[0] + p[1] * frame[1] + p[2] * frame[2] + frame[3];
}

// *** Bary

inline bool Bary::is_convex() const {
  const auto& self = *this;
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
  //  (1.f / 6.f) * dot(p1, cross(p2, p3)).
  const auto triangle2 = map(triangle, [](const Point& p) { return convert<Precision>(p); });
  return dot(triangle2[0], cross(triangle2[1], triangle2[2])) < -tolerance;
}

template <typename Precision> float signed_area(const Vec2<float>& p1, const Vec2<float>& p2, const Vec2<float>& p3) {
  const auto v1 = convert<Precision>(p1), v2 = convert<Precision>(p2), v3 = convert<Precision>(p3);
  return 0.5f * float(cross(v2 - v1, v3 - v1));
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_GEOMETRY_H_
