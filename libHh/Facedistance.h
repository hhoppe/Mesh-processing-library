// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_FACEDISTANCE_H_
#define MESH_PROCESSING_LIBHH_FACEDISTANCE_H_

#include "libHh/Bbox.h"
#include "libHh/Geometry.h"

namespace hh {

// Find a lower bound on the distance between p and triangle (p1, p2, p3).
[[nodiscard]] float lb_dist_point_triangle(const Point& p, const Point& p1, const Point& p2, const Point& p3);

// Find a lower bound on the distance between p and a triangle.
[[nodiscard]] float lb_dist_point_triangle(const Point& p, const Vec3<Point>& triangle);

// Find a lower bound on the distance between p and an axis-aligned bounding box.
[[nodiscard]] float lb_dist_point_bbox(const Point& p, const Bbox<float, 3>& bbox);

struct TriangleProjectionResult {
  float d2;   // Squared distance to the closest point.
  Bary bary;  // Convex barycentric coordinates of the closest point.
  Point clp;  // Closest point (= interp(p1, p2, p3, bary)).
};

// Given point p and triangle (p1, p2, p3), return info about the point within the triangle that is closest to p.
[[nodiscard]] TriangleProjectionResult project_point_triangle(const Point& p, const Point& p1, const Point& p2,
                                                              const Point& p3);

// Given point p and a triangle, return info about the point within the triangle that is closest to p.
[[nodiscard]] TriangleProjectionResult project_point_triangle(const Point& p, const Vec3<Point>& triangle);

struct SegmentProjectionResult {
  float d2;    // Squared distance to the closest point.
  float bary;  // Convex barycentric coordinate of the closest point.
  Point clp;   // Closest point (= interp(p1, p2, bary)).
};

// Given point p and segment (p1, p2), return info about the point within the segment that is closest to p.
[[nodiscard]] SegmentProjectionResult project_point_segment(const Point& p, const Point& p1, const Point& p2);

//----------------------------------------------------------------------------

inline float lb_dist_point_triangle(const Point& p, const Point& p1, const Point& p2, const Point& p3) {
  float d = 0.f;
  for_int(c, 3) {
    const float v1 = p1[c], v2 = p2[c], v3 = p3[c];
    float v_min = v1, v_max = v1;
    if (v2 < v_min)
      v_min = v2;
    else if (v2 > v_max)
      v_max = v2;
    if (v3 < v_min)
      v_min = v3;
    else if (v3 > v_max)
      v_max = v3;
    const float v = p[c];
    if (const float a1 = v - v_max; a1 > d) {
      d = a1;
    } else if (const float a2 = v_min - v; a2 > d) {
      d = a2;
    }
  }
  return d;
}

inline float lb_dist_point_triangle(const Point& p, const Vec3<Point>& triangle) {
  return lb_dist_point_triangle(p, triangle[0], triangle[1], triangle[2]);
}

inline float lb_dist_point_bbox(const Point& p, const Bbox<float, 3>& bbox) {
  float d = 0.f;
  for_int(c, 3) {
    const float v = p[c];
    if (const float a1 = v - bbox[1][c]; a1 > d) {
      d = a1;
    } else if (const float a2 = bbox[0][c] - v; a2 > d) {
      d = a2;
    }
  }
  return d;
}

namespace details {

// Project p onto each side of the triangle; this handles a degenerate triangle (with collinear vertices).
inline TriangleProjectionResult project_point_triangle_sides(const Point& p, const Point& p1, const Point& p2,
                                                             const Point& p3) {
  const Vec3<Point> triangle{p1, p2, p3};
  TriangleProjectionResult result{BIGFLOAT, Bary(1.f, 0.f, 0.f), p1};
  for_int(i, 3) {
    const auto [d2, bary, clp] = project_point_segment(p, triangle[i], triangle[mod3(i + 1)]);
    if (d2 < result.d2) {
      result.d2 = d2;
      result.bary[i] = bary;
      result.bary[mod3(i + 1)] = 1.f - bary;
      result.bary[mod3(i + 2)] = 0.f;
      result.clp = clp;
    }
  }
  return result;
}

}  // namespace details

// The test for projection into the interior comes first, as it is the common case for nearby points.  Its
// barycentric weights w1, w2, w3 (which sum to nn) are dot products with the in-plane edge normals cross(n, v12) and
// cross(v13, n); the equivalent differences of products of dot products (as in the normal equations) would square the
// conditioning of thin triangles.  A point outside is then classified against the Voronoi regions of the vertices
// and edges, following Ericson, "Real-Time Collision Detection", 2005, section 5.1.5.
inline TriangleProjectionResult project_point_triangle(const Point& p, const Point& p1, const Point& p2,
                                                       const Point& p3) {
  const Vector v12 = p2 - p1, v13 = p3 - p1;
  const Vector v1p = p - p1;
  const Vector n = cross(v12, v13);
  const float nn = mag2(n);
  const float w2 = dot(v1p, cross(v13, n)), w3 = dot(v1p, cross(n, v12));
  if (const float w1 = nn - w2 - w3; w1 >= 0.f && w2 >= 0.f && w3 >= 0.f && nn > 0.f) {
    const float b1 = w1 / nn, b2 = w2 / nn, b3 = w3 / nn;
    const Point clp = p1 + v12 * b2 + v13 * b3;
    return {dist2(p, clp), Bary(b1, b2, b3), clp};
  }
  const float e1 = dot(v12, v1p), e2 = dot(v13, v1p);
  if (e1 <= 0.f && e2 <= 0.f) return {dist2(p, p1), Bary(1.f, 0.f, 0.f), p1};
  const Vector v2p = p - p2;
  const float e3 = dot(v12, v2p), e4 = dot(v13, v2p);
  if (e3 >= 0.f && e4 <= e3) return {dist2(p, p2), Bary(0.f, 1.f, 0.f), p2};
  if (w3 <= 0.f && e1 >= 0.f && e3 <= 0.f && e1 > e3) {
    const float a = e1 / (e1 - e3);
    const Point clp = p1 + v12 * a;
    return {dist2(p, clp), Bary(1.f - a, a, 0.f), clp};
  }
  const Vector v3p = p - p3;
  const float e5 = dot(v12, v3p), e6 = dot(v13, v3p);
  if (e6 >= 0.f && e5 <= e6) return {dist2(p, p3), Bary(0.f, 0.f, 1.f), p3};
  if (w2 <= 0.f && e2 >= 0.f && e6 <= 0.f && e2 > e6) {
    const float a = e2 / (e2 - e6);
    const Point clp = p1 + v13 * a;
    return {dist2(p, clp), Bary(1.f - a, 0.f, a), clp};
  }
  const Vector v23 = p3 - p2;
  const float w1 = dot(v2p, cross(n, v23));
  if (w1 <= 0.f && e4 >= e3 && e5 >= e6 && (e4 - e3) + (e5 - e6) > 0.f) {
    const float a = (e4 - e3) / ((e4 - e3) + (e5 - e6));
    const Point clp = p2 + v23 * a;
    return {dist2(p, clp), Bary(0.f, 1.f - a, a), clp};
  }
  // The triangle is degenerate, or rounding made the region tests inconsistent.
  return details::project_point_triangle_sides(p, p1, p2, p3);
}

inline TriangleProjectionResult project_point_triangle(const Point& p, const Vec3<Point>& triangle) {
  return project_point_triangle(p, triangle[0], triangle[1], triangle[2]);
}

inline SegmentProjectionResult project_point_segment(const Point& p, const Point& p1, const Point& p2) {
  const Vector v12 = p2 - p1, v1p = p - p1;
  const float d12sq = mag2(v12), vdot = dot(v12, v1p);
  const float bary2 = !d12sq ? .5f : clamp(vdot / d12sq, 0.f, 1.f);
  const float bary = 1.f - bary2;
  // const Point clp = interp(p1, p2, bary);
  const Point clp = p1 + bary2 * v12;
  const float d2 = dist2(p, clp);
  return {d2, bary, clp};
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_FACEDISTANCE_H_
