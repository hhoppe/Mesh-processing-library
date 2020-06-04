// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_POLYGONSPATIAL_H_
#define MESH_PROCESSING_LIBHH_POLYGONSPATIAL_H_

#include "libHh/Bbox.h"
#include "libHh/Facedistance.h"
#include "libHh/Polygon.h"
#include "libHh/Spatial.h"

namespace hh {

namespace details {

struct polygon_approx_distance2 {
  float operator()(const Point& p, Univ id) const {
    const Polygon& poly = *Conv<const Polygon*>::d(id);
    assertx(poly.num() == 3);
    return square(lb_dist_point_triangle(p, poly[0], poly[1], poly[2]));
  }
};

struct polygon_distance2 {
  float operator()(const Point& p, Univ id) const {
    const Polygon& poly = *Conv<const Polygon*>::d(id);
    assertx(poly.num() == 3);
    return dist_point_triangle2(p, poly[0], poly[1], poly[2]);
  }
};

}  // namespace details

// Spatial data structure for polygon elements.
class PolygonSpatial : public ObjectSpatial<details::polygon_approx_distance2, details::polygon_distance2> {
 public:
  explicit PolygonSpatial(int gn) : ObjectSpatial(gn) {}
  // clear() inherited from ObjectSpatial, does not delete Polygons!
  void enter(const Polygon* ppoly) {  // poly is not copied!
    assertx(ppoly->num() == 3);
    Polygon poly = *ppoly;  // copied because modified below
    Bbox bbox;
    poly.get_bbox(bbox);
    auto func_polygon_in_bbox = [&](const Bbox& spatial_bbox) -> bool {
      for_int(c, 3) {
        if (bbox[0][c] > spatial_bbox[1][c] || bbox[1][c] < spatial_bbox[0][c]) return false;
      }
      int modif = poly.intersect_bbox(spatial_bbox);
      bool ret = poly.num() > 0;
      if (modif) poly = *ppoly;
      return ret;
    };
    ObjectSpatial::enter(Conv<const Polygon*>::e(ppoly), poly[0], func_polygon_in_bbox);
  }
  bool first_along_segment(const Point& p1, const Point& p2, const Polygon*& ret_ppoly, Point& ret_pint) const {
    Vector vray = p2 - p1;
    bool foundint = false;
    float ret_fmin;
    auto func_test_polygon_with_ray = [&](Univ id) -> bool {
      const Polygon* ppoly = Conv<const Polygon*>::d(id);
      Point pint;
      if (!ppoly->intersect_segment(p1, p2, pint)) return false;
      float f = dot(pint - p1, vray);
      if (!foundint || f < ret_fmin) {
        ret_fmin = f;
        ret_pint = pint;
        ret_ppoly = ppoly;
        foundint = true;
      }
      return true;
    };
    search_segment(p1, p2, func_test_polygon_with_ray);
    return foundint;
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_POLYGONSPATIAL_H_
