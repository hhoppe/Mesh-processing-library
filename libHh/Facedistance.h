// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_FACEDISTANCE_H_
#define MESH_PROCESSING_LIBHH_FACEDISTANCE_H_

#include "libHh/Bbox.h"
#include "libHh/Geometry.h"

namespace hh {

// Find a lower bound on the distance between p and triangle (p1, p2, p3).
float lb_dist_point_triangle(const Point& p, const Point& p1, const Point& p2, const Point& p3);

// Find a lower bound on the distance between p and a triangle.
float lb_dist_point_triangle(const Point& p, const Vec3<Point>& triangle);

// Find a lower bound on the distance between p and an axis-aligned bounding box.
float lb_dist_point_bbox(const Point& p, const Bbox<float, 3>& bbox);

struct TriangleProjectionResult {
  float d2;   // Squared distance to the closest point.
  Bary bary;  // Convex barycentric coordinates of the closest point.
  Point clp;  // Closest point (= interp(p1, p2, p3, bary)).
};

// Given point p and triangle (p1, p2, p3), return info about the point within the triangle that is closest to p.
TriangleProjectionResult project_point_triangle(const Point& p, const Point& p1, const Point& p2, const Point& p3);

// Given point p and a triangle, return info about the point within the triangle that is closest to p.
TriangleProjectionResult project_point_triangle(const Point& p, const Vec3<Point>& triangle);

struct SegmentProjectionResult {
  float d2;    // Squared distance to the closest point.
  float bary;  // Convex barycentric coordinate of the closest point.
  Point clp;   // Closest point (= interp(p1, p2, bary)).
};

// Given point p and segment (p1, p2), return info about the point within the segment that is closest to p.
SegmentProjectionResult project_point_segment(const Point& p, const Point& p1, const Point& p2);

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

// Project proj onto segment (pi0, pi1).
inline bool project_point_seg(int i, float b, const Point& pi0, const Point& pi1, const Point& proj, Bary& ret_cba,
                              Point& ret_clp, float& mind2) {
  // simplify??
  if (b >= 0.f) return false;
  float vix = pi1[0] - pi0[0], viy = pi1[1] - pi0[1], viz = pi1[2] - pi0[2];
  // float vpx = proj[0] - pi0[0], vpy = proj[1] - pi0[1], vpz = proj[2] - pi0[2];
  float d12sq = vix * vix + viy * viy + viz * viz;
  // float don12 = vix * vpx + viy * vpy + viz * vpz;
  float don12 = vix * (proj[0] - pi0[0]) + viy * (proj[1] - pi0[1]) + viz * (proj[2] - pi0[2]);
  if (don12 <= 0.f) {
    // float d2 = vpx * vpx + vpy * vpy +  vpz * vpz;  // optimized anyways
    float d2 = dist2(proj, pi0);
    if (d2 >= mind2) return false;
    mind2 = d2;
    ret_cba[i] = 1.f;
    ret_cba[mod3(i + 1)] = 0.f;
    ret_cba[mod3(i + 2)] = 0.f;
    ret_clp = pi0;
    return false;
  } else if (don12 >= d12sq) {
    float d2 = dist2(proj, pi1);
    if (d2 >= mind2) return false;
    mind2 = d2;
    ret_cba[i] = 0.f;
    ret_cba[mod3(i + 1)] = 1.f;
    ret_cba[mod3(i + 2)] = 0.f;
    ret_clp = pi1;
    return false;
  } else {
    float a = don12 / d12sq;
    ret_cba[i] = 1.f - a;
    ret_cba[mod3(i + 1)] = a;
    ret_cba[mod3(i + 2)] = 0.f;
    ret_clp = pi0 + V(vix, viy, viz) * a;
    return true;
  }
}

// Projection lies outside triangle, so more work is needed.
inline TriangleProjectionResult project_aux(const Point& p, const Point& p1, const Point& p2, const Point& p3,
                                            const Point& proj, float b1, float b2, float b3) {
  TriangleProjectionResult result;
  // Variable mind2 is necessary because p may project onto 2 different vertices (even in non-degenerate case).
  // For triangle ABC, on line AB it may project to B, then on line AC it may project to C, then on line BC it may
  // project inside the segment.
  float mind2 = BIGFLOAT;
  bool stop = project_point_seg(0, b3, p1, p2, proj, result.bary, result.clp, mind2);
  if (!stop) stop = project_point_seg(1, b1, p2, p3, proj, result.bary, result.clp, mind2);
  if (!stop) project_point_seg(2, b2, p3, p1, proj, result.bary, result.clp, mind2);
  result.d2 = dist2(p, result.clp);
  return result;
}

}  // namespace details

// Two bad cases:
// - v2 == 0 or v3 == 0 (two points of triangle are same) -> ok
// - v2v3 * v2v3 == v2v2 * v3v3 (!area but v2 != 0 && v3 != 0) -> project on sides
inline TriangleProjectionResult project_point_triangle(const Point& p, const Point& p1, const Point& p2,
                                                       const Point& p3) {
  // Try moving back to coordinate-free code??
  // const Vector v2 = p2 - p1, v3 = p3 - p1, vp = p - p1;
  // float v2v2 = mag2(v2), v3v3 = mag2(v3), v2v3 = dot(v2, v3);
  // const float v2vp = dot(v2, vp), v3vp = dot(v3, vp);
  const float px = p[0], py = p[1], pz = p[2];
  const float p1x = p1[0], p1y = p1[1], p1z = p1[2];
  const float v2x = p2[0] - p1x, v2y = p2[1] - p1y, v2z = p2[2] - p1z;
  const float v3x = p3[0] - p1x, v3y = p3[1] - p1y, v3z = p3[2] - p1z;
  float v2v2 = v2x * v2x + v2y * v2y + v2z * v2z;
  float v3v3 = v3x * v3x + v3y * v3y + v3z * v3z;
  const float v2v3 = v2x * v3x + v2y * v3y + v2z * v3z;
  if (!v2v2) v2v2 = 1.f;  // Recover if v2 == 0.f .
  if (!v3v3) v3v3 = 1.f;  // Recover if v3 == 0.f .
  const float denom = v3v3 - v2v3 * v2v3 / v2v2;
  if (!denom) {
    // Recover if v2v3 * v2v3 == v2v2 * v3v3; project on sides.
    // Note: set a = b = 1e-10f to force projection on 2 sides.
    return details::project_aux(p, p1, p2, p3, p, -1e-10f, -1e-10f, 1.f);
  }
  const float vpx = px - p1x, vpy = py - p1y, vpz = pz - p1z;
  const float v2vp = v2x * vpx + v2y * vpy + v2z * vpz;
  const float v3vp = v3x * vpx + v3y * vpy + v3z * vpz;
  const float b3 = (v3vp - v2v3 / v2v2 * v2vp) / denom;
  const float b2 = (v2vp - b3 * v2v3) / v2v2;
  const float b1 = 1.f - b2 - b3;
  if (b1 < 0.f || b2 < 0.f || b3 < 0.f) {
    // Point pp = interp(p1, p2, p3, b1, b2);
    const Point pp(p1x + b2 * v2x + b3 * v3x, p1y + b2 * v2y + b3 * v3y, p1z + b2 * v2z + b3 * v3z);
    return details::project_aux(p, p1, p2, p3, pp, b1, b2, b3);
  }
  // Fast common case (projection into interior):
  // ret_clp = interp(p1, p2, p3, b1, b2);
  const float clpx = p1x + b2 * v2x + b3 * v3x;
  const float clpy = p1y + b2 * v2y + b3 * v3y;
  const float clpz = p1z + b2 * v2z + b3 * v3z;
  // d2 = dist2(p, clp);
  const float x = px - clpx, y = py - clpy, z = pz - clpz;
  const float d2 = x * x + y * y + z * z;
  const Bary bary = V(b1, b2, b3);
  const Point clp = V(clpx, clpy, clpz);
  return {d2, bary, clp};
}

inline TriangleProjectionResult project_point_triangle(const Point& p, const Vec3<Point>& triangle) {
  return project_point_triangle(p, triangle[0], triangle[1], triangle[2]);
}

inline SegmentProjectionResult project_point_segment(const Point& p, const Point& p1, const Point& p2) {
  // simplify??
  // Vector v12 = p2 - p1, v1p = p - p1;
  // float d122 = mag2(v12), vdot = dot(v12, v1p);
  float v12x = p2[0] - p1[0], v12y = p2[1] - p1[1], v12z = p2[2] - p1[2];
  float d122 = square(v12x) + square(v12y) + square(v12z);
  float v1px = p[0] - p1[0], v1py = p[1] - p1[1], v1pz = p[2] - p1[2];
  float vdot = v12x * v1px + v12y * v1py + v12z * v1pz;
  // had float don12 = vdot / sqrt(d122);
  const float bary = !d122 ? .5f : clamp(1.f - vdot / d122, 0.f, 1.f);
  const Point clp = interp(p1, p2, bary);
  const float d2 = dist2(p, clp);
  return {d2, bary, clp};
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_FACEDISTANCE_H_
