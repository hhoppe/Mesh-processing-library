// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_TRIANGLEFACESPATIAL_H_
#define MESH_PROCESSING_LIBHH_TRIANGLEFACESPATIAL_H_

#include <optional>

#include "libHh/Facedistance.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"
#include "libHh/Spatial.h"
#include "libHh/Timer.h"

namespace hh {

struct TriangleFace {
  Vec3<Point> triangle;
  Face face;
};

namespace details {

struct triangleface_approx_distance2 {
  float operator()(const Point& p, Univ id) const {
    const TriangleFace& triangleface = *Conv<const TriangleFace*>::d(id);
    const Vec3<Point>& triangle = triangleface.triangle;
    return square(lb_dist_point_triangle(p, triangle));
  }
};

struct triangleface_distance2 {
  float operator()(const Point& p, Univ id) const {
    const TriangleFace& triangleface = *Conv<const TriangleFace*>::d(id);
    const Vec3<Point>& triangle = triangleface.triangle;
    return project_point_triangle(p, triangle).d2;
  }
};

}  // namespace details

// A spatial data structure over a collection of triangles, each associated with a Mesh Face.
class TriangleFaceSpatial
    : public ObjectSpatial<details::triangleface_approx_distance2, details::triangleface_distance2> {
 public:
  explicit TriangleFaceSpatial(CArrayView<TriangleFace> trianglefaces, int gridn) : ObjectSpatial(gridn) {
    HH_STIMER("__trianglefacespatial_build");
    for (const TriangleFace& triangleface : trianglefaces) {
      const Vec3<Point>& triangle = triangleface.triangle;
      // TODO: Speed this up by avoiding use of Polygon.
      Polygon poly{triangle}, opoly = poly;
      const Bbox bbox{triangle};
      const auto func_triangleface_in_bbox = [&](const Bbox<float, 3>& spatial_bbox) -> bool {
        for_int(c, 3) if (bbox[0][c] > spatial_bbox[1][c] || bbox[1][c] < spatial_bbox[0][c]) return false;
        const bool modified = poly.intersect_bbox(spatial_bbox);
        const bool ret = poly.num() > 0;
        if (modified) poly = opoly;
        return ret;
      };
      ObjectSpatial::enter(Conv<const TriangleFace*>::e(&triangleface), triangle[0], func_triangleface_in_bbox);
    }
  }
  // clear() inherited from ObjectSpatial

  struct SegmentResult {
    const TriangleFace* triangleface;
    Point pint;
  };
  std::optional<SegmentResult> first_along_segment(const Point& p1, const Point& p2) const {
    std::optional<SegmentResult> result;
    const Vector vray = p2 - p1;
    float tmin = BIGFLOAT;
    const auto func_test_triangleface_with_ray = [&](Univ id) -> bool {
      const TriangleFace* ptriangleface = Conv<const TriangleFace*>::d(id);
      const Vec3<Point>& triangle = ptriangleface->triangle;
      const auto pint = intersect_segment_with_triangle(p1, p2, triangle);
      if (!pint) return false;
      const float t = dot(*pint - p1, vray);
      if (t < tmin) {
        tmin = t;
        result = {ptriangleface, *pint};
      }
      return true;
    };
    search_segment(p1, p2, func_test_triangleface_with_ray);
    return result;
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_TRIANGLEFACESPATIAL_H_
