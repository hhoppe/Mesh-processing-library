// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_FACEDISTANCE_H_
#define MESH_PROCESSING_LIBHH_FACEDISTANCE_H_

#include "libHh/Bbox.h"
#include "libHh/Geometry.h"

namespace hh {

// Find a lower bound on the distance between p and triangle (p1, p2, p3).
float lb_dist_point_triangle(const Point& p, const Point& p1, const Point& p2, const Point& p3);

// Find a lower bound on the distance between p and an axis-aligned bounding box.
float lb_dist_point_bbox(const Point& p, const Bbox& bbox);

// Given point p and triangle (p1, p2, p3), return the squared distance,
//  and compute the convex barycentric coordinates ret_cba of the closest point ret_clp within the triangle.
float project_point_triangle2(const Point& p, const Point& p1, const Point& p2, const Point& p3, Bary& ret_cba,
                              Point& ret_clp);

// Compute the squared distance between point p and triangle (p1, p2, p3).
float dist_point_triangle2(const Point& p, const Point& p1, const Point& p2, const Point& p3);

// Return the squared distance from p to segment (p1, p2), and optionally return the barycentric coordinate ret_cba
//  (in range [0, 1]) of the closest point interp(p1, p2, cba) within the segment.
float project_point_seg2(const Point& p, const Point& p1, const Point& p2, float* ret_cba = nullptr);

//----------------------------------------------------------------------------

inline float lb_dist_point_triangle(const Point& p, const Point& p1, const Point& p2, const Point& p3) {
  float d = 0.f;
  for_int(c, 3) {
    float v1 = p1[c], mi = v1, ma = v1, v2 = p2[c], v3 = p3[c];
    if (v2 < mi)
      mi = v2;
    else if (v2 > ma)
      ma = v2;
    if (v3 < mi)
      mi = v3;
    else if (v3 > ma)
      ma = v3;
    float v = p[c], a;
    if ((a = v - ma) > 0.f) {
      if (a > d) d = a;
    } else if ((a = mi - v) > 0.f) {
      if (a > d) d = a;
    }
  }
  return d;
}

inline float lb_dist_point_bbox(const Point& p, const Bbox& bbox) {
#if 0
  float d = 0.f;
  for_int(c, 3) {
    float v = p[c], a;
    if ((a = v - bbox[1][c]) > 0.f) {
      if (a > d) d = a;
    } else if ((a = bbox[0][c] - v) > 0.f) {
      if (a > d) d = a;
    }
  }
#else
  float d = 0.f, v, a;
  v = p[0];
  if ((a = v - bbox[1][0]) > 0.f) {
    d = a;
  } else if ((a = bbox[0][0] - v) > 0.f) {
    d = a;
  }
  v = p[1];
  if ((a = v - bbox[1][1]) > 0.f) {
    if (a > d) d = a;
  } else if ((a = bbox[0][1] - v) > 0.f) {
    if (a > d) d = a;
  }
  v = p[2];
  if ((a = v - bbox[1][2]) > 0.f) {
    if (a > d) d = a;
  } else if ((a = bbox[0][2] - v) > 0.f) {
    if (a > d) d = a;
  }
#endif
  return d;
}

namespace details {

// Project proj onto segment (pi0, pi1).
inline bool project_point_seg(int i, float b, const Point& pi0, const Point& pi1, const Point& proj, Bary& ret_cba,
                              Point& ret_clp, float& mind2) {
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
inline float project_aux(const Point& p, const Point& p1, const Point& p2, const Point& p3, Bary& ret_cba,
                         Point& ret_clp, const Point& proj, float b1, float b2, float b3) {
  // Variable mind2 necessary since p may project onto 2 different vertices (even in non-degenerate case).
  float mind2 = BIGFLOAT;
  bool stop = project_point_seg(0, b3, p1, p2, proj, ret_cba, ret_clp, mind2);
  if (!stop) stop = project_point_seg(1, b1, p2, p3, proj, ret_cba, ret_clp, mind2);
  if (!stop) project_point_seg(2, b2, p3, p1, proj, ret_cba, ret_clp, mind2);
  return dist2(p, ret_clp);
}

}  // namespace details

// Two bad cases:
// - v2 == 0 or v3 == 0 (two points of triangle are same) -> ok
// - v2v3 * v2v3 == v2v2 * v3v3 (!area but v2 != 0 && v3 != 0) -> project on sides
inline float project_point_triangle2(const Point& p, const Point& p1, const Point& p2, const Point& p3, Bary& ret_cba,
                                     Point& ret_clp) {
  dummy_init(ret_cba, ret_clp);
  // Vector v2 = p2 - p1, v3 = p3 - p1, vp = p - p1;
  // float v2v2 = mag2(v2), v3v3 = mag2(v3), v2v3 = dot(v2, v3);
  // float v2vp = dot(v2, vp), v3vp = dot(v3, vp);
  float v2x, v2y, v2z;
  float v3x, v3y, v3z;
  float v2v2, v3v3, v2v3;
  float denom;
  {
    {
      float p1x = p1[0], p1y = p1[1], p1z = p1[2];
      float lv2x = p2[0] - p1x, lv2y = p2[1] - p1y, lv2z = p2[2] - p1z;
      float lv3x = p3[0] - p1x, lv3y = p3[1] - p1y, lv3z = p3[2] - p1z;
      v2v2 = lv2x * lv2x + lv2y * lv2y + lv2z * lv2z;
      v3v3 = lv3x * lv3x + lv3y * lv3y + lv3z * lv3z;
      v2v3 = lv2x * lv3x + lv2y * lv3y + lv2z * lv3z;
      v2x = lv2x;
      v2y = lv2y;
      v2z = lv2z;
      v3x = lv3x;
      v3y = lv3y;
      v3z = lv3z;
    }
    if (!v2v2) v2v2 = 1.f;  // recover if v2 == 0
    if (!v3v3) v3v3 = 1.f;  // recover if v3 == 0
    denom = (v3v3 - v2v3 * v2v3 / v2v2);
    if (!denom) {
      // recover if v2v3 * v2v3 == v2v2 * v3v3; project on sides
      // Note: set a = b = 1e-10 to force projection on 2 sides
      return details::project_aux(p, p1, p2, p3, ret_cba, ret_clp, p, -1e-10f, -1e-10f, 1.f);
    }
  }
  // float vpx = p[0] - p1x, vpy = p[1] - p1y, vpz = p[2] - p1z;
  float vpx = p[0] - p1[0], vpy = p[1] - p1[1], vpz = p[2] - p1[2];
  float v2vp = v2x * vpx + v2y * vpy + v2z * vpz;
  float v3vp = v3x * vpx + v3y * vpy + v3z * vpz;
  // I have forgotten how I derived these; it sure is neat.
  float b3 = (v3vp - v2v3 / v2v2 * v2vp) / denom;
  float b2 = (v2vp - b3 * v2v3) / v2v2;
  float b1 = 1.f - b2 - b3;
  if (b1 < 0.f || b2 < 0.f || b3 < 0.f) {
    // Point pp = interp(p1, p2, p3, b1, b2);
    Point pp(p1[0] + b2 * v2x + b3 * v3x, p1[1] + b2 * v2y + b3 * v3y, p1[2] + b2 * v2z + b3 * v3z);
    return details::project_aux(p, p1, p2, p3, ret_cba, ret_clp, pp, b1, b2, b3);
  }
  // fast common case (projection into interior)
  // ret_clp = interp(p1, p2, p3, b1, b2);
  // float clpx = p1x + b2 * v2x + b3 * v3x;
  // float clpy = p1y + b2 * v2y + b3 * v3y;
  // float clpz = p1z + b2 * v2z + b3 * v3z;
  float clpx = p1[0] + b2 * v2x + b3 * v3x;
  float clpy = p1[1] + b2 * v2y + b3 * v3y;
  float clpz = p1[2] + b2 * v2z + b3 * v3z;
  // dist2(p, clp);
  float x = p[0] - clpx, y = p[1] - clpy, z = p[2] - clpz;
  float dis2 = x * x + y * y + z * z;
  ret_clp = V(clpx, clpy, clpz);
  ret_cba = V(b1, b2, b3);
  return dis2;
}

inline float dist_point_triangle2(const Point& p, const Point& p1, const Point& p2, const Point& p3) {
  Bary bary;
  Point clp(0.f, 0.f, 0.f);
  return project_point_triangle2(p, p1, p2, p3, bary, clp);
}

inline float project_point_seg2(const Point& p, const Point& p1, const Point& p2, float* ret_cba) {
  // Vector v12 = p2 - p1, v1p = p - p1;
  // float d122 = mag2(v12), vdot = dot(v12, v1p);
  float v12x = p2[0] - p1[0], v12y = p2[1] - p1[1], v12z = p2[2] - p1[2];
  float d122 = square(v12x) + square(v12y) + square(v12z);
  float v1px = p[0] - p1[0], v1py = p[1] - p1[1], v1pz = p[2] - p1[2];
  float vdot = v12x * v1px + v12y * v1py + v12z * v1pz;
  // had float don12 = vdot / sqrt(d122);
  float cba = !d122 ? .5f : clamp(1.f - vdot / d122, 0.f, 1.f);
  float d2 = dist2(interp(p1, p2, cba), p);
  if (ret_cba) *ret_cba = cba;
  return d2;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_FACEDISTANCE_H_
