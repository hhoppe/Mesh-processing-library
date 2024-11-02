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

// Within the triangle plane, project `proj` onto the segment (pi0, pi1).
inline bool project_onto_seg(const Point& proj, int i, float b, const Point& pi0, const Point& pi1,
                             TriangleProjectionResult& result, float& min_inplane_d2) {
  if (b >= 0.f) return false;
  const Vector vi = pi1 - pi0;
  const float d12sq = mag2(vi);
  const float don12 = dot(vi, proj - pi0);
  if (don12 <= 0.f) {
    const float in_plane_d2 = dist2(proj, pi0);
    if (in_plane_d2 < min_inplane_d2) {
      min_inplane_d2 = in_plane_d2;
      result.bary[i] = 1.f;
      result.bary[mod3(i + 1)] = 0.f;
      result.bary[mod3(i + 2)] = 0.f;
      result.clp = pi0;
    }
    return false;
  } else if (don12 >= d12sq) {
    const float in_plane_d2 = dist2(proj, pi1);
    if (in_plane_d2 < min_inplane_d2) {
      min_inplane_d2 = in_plane_d2;
      result.bary[i] = 0.f;
      result.bary[mod3(i + 1)] = 1.f;
      result.bary[mod3(i + 2)] = 0.f;
      result.clp = pi1;
    }
    return false;
  } else {
    const float a = don12 / d12sq;  // Note that d12sq > 0.f, else don12 == 0.f, which is detected earlier.
    result.bary[i] = 1.f - a;
    result.bary[mod3(i + 1)] = a;
    result.bary[mod3(i + 2)] = 0.f;
    result.clp = pi0 + vi * a;
    return true;
  }
}

// The planar projection lies outside the triangle, so more work is needed.
inline TriangleProjectionResult project_aux(const Point& p, const Point& p1, const Point& p2, const Point& p3,
                                            const Point& proj, float b1, float b2, float b3) {
  TriangleProjectionResult result;
  // We must track min_in_plane_d2 because p may project onto 2 different vertices.  For triangle ABC, on line AB it
  // may project to B, then on line AC it may project to C, then on line BC it may project inside the segment.
  float min_inplane_d2 = BIGFLOAT;  // (Distance within the plane, from proj, not from p.)
  bool stop = project_onto_seg(proj, 0, b3, p1, p2, result, min_inplane_d2);
  if (!stop) {
    stop = project_onto_seg(proj, 1, b1, p2, p3, result, min_inplane_d2);
    if (!stop) project_onto_seg(proj, 2, b2, p3, p1, result, min_inplane_d2);
  }
  result.d2 = dist2(p, result.clp);
  return result;
}

}  // namespace details

// Two bad cases:
// - v2 == 0 or v3 == 0 (two points of triangle are same) -> ok
// - v2v3 * v2v3 == v2v2 * v3v3 (!area but v2 != 0 && v3 != 0) -> project on sides
inline TriangleProjectionResult project_point_triangle(const Point& p, const Point& p1, const Point& p2,
                                                       const Point& p3) {
  const Vector v2 = p2 - p1, v3 = p3 - p1;
  float v2v2 = mag2(v2), v3v3 = mag2(v3);
  const float v2v3 = dot(v2, v3);
  float denom;
  if (!v2v2 || !v3v3 || !(denom = v3v3 - v2v3 * v2v3 / v2v2)) {
    // Triangle is degenerate, so project on its sides.  Set b1 = b2 = -1e-10f to force projection on 2 sides.
    return details::project_aux(p, p1, p2, p3, p, -1e-10f, -1e-10f, 1.f);
  }
  const Vector vp = p - p1;
  const float v2vp = dot(v2, vp), v3vp = dot(v3, vp);
  const float b3 = (v3vp - v2v3 / v2v2 * v2vp) / denom;
  const float b2 = (v2vp - b3 * v2v3) / v2v2;
  const float b1 = 1.f - b2 - b3;
  // const Point proj = interp(p1, p2, p3, b1, b2);
  const Point proj = p1 + b2 * v2 + b3 * v3;
  if (b1 < 0.f || b2 < 0.f || b3 < 0.f) return details::project_aux(p, p1, p2, p3, proj, b1, b2, b3);
  // Fast common case (projection into interior):
  const float d2 = dist2(p, proj);
  const Bary bary = V(b1, b2, b3);
  return {d2, bary, proj};
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
