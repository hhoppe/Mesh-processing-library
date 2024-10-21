// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MESHSEARCH_H_
#define MESH_PROCESSING_LIBHH_MESHSEARCH_H_

#include <optional>

#include "libHh/Bbox.h"
#include "libHh/Facedistance.h"
#include "libHh/GMesh.h"
#include "libHh/Spatial.h"

#if 0
{
  const MeshSearch msearch(mesh, {true});
  Face hintf = nullptr;
  const auto& [f, bary, clp, d2] = msearch(p, hintf);
}
#endif

namespace hh {

struct PolygonFace {
  PolygonFace() = default;
  explicit PolygonFace(Polygon p, Face f) : poly(std::move(p)), face(f) {}
  Polygon poly;
  Face face;
};

namespace details {

struct polygonface_approx_distance2 {
  float operator()(const Point& p, Univ id) const {
    const PolygonFace& polyface = *Conv<const PolygonFace*>::d(id);
    const Polygon& poly = polyface.poly;
    ASSERTX(poly.num() == 3);
    return square(lb_dist_point_triangle(p, poly[0], poly[1], poly[2]));
  }
};

struct polygonface_distance2 {
  float operator()(const Point& p, Univ id) const {
    const PolygonFace& polyface = *Conv<const PolygonFace*>::d(id);
    const Polygon& poly = polyface.poly;
    ASSERTX(poly.num() == 3);
    return dist_point_triangle2(p, poly[0], poly[1], poly[2]);
  }
};

}  // namespace details

// A spatial data structure over a collection of polygons (each associated with a Mesh Face).
class PolygonFaceSpatial
    : public ObjectSpatial<details::polygonface_approx_distance2, details::polygonface_distance2> {
 public:
  explicit PolygonFaceSpatial(int gn) : ObjectSpatial(gn) {}
  // clear() inherited from ObjectSpatial
  void enter(const PolygonFace* polyface);  // not copied, no ownership taken
  bool first_along_segment(const Point& p1, const Point& p2, const PolygonFace*& ret_ppolyface, Point& ret_pint) const;
};

// Construct a spatial data structure from a mesh, to enable fast closest-point queries from arbitrary points.
// Optionally, tries to speed up the search by caching the result of the previous search and incrementally
// walking over the mesh from that prior result.
class MeshSearch {
 public:
  struct Options {
    bool allow_local_project{false};
    bool allow_internal_boundaries{false};
    bool allow_off_surface{false};
    std::optional<Bbox<float, 3>> bbox;
  };
  explicit MeshSearch(const GMesh& mesh, Options options);

  struct Result {
    Face f;
    Bary bary;
    Point clp;
    float d2;
  };
  Result search(const Point& p, Face hintf) const;

  const GMesh& mesh() const { return _mesh; }

 private:
  const GMesh& _mesh;
  Options _options;
  Array<PolygonFace> _ar_polyface;
  unique_ptr<PolygonFaceSpatial> _ppsp;
  Frame _ftospatial;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MESHSEARCH_H_
