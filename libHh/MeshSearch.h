// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MESHSEARCH_H_
#define MESH_PROCESSING_LIBHH_MESHSEARCH_H_

#include "GMesh.h"
#include "Spatial.h"
#include "Facedistance.h"

#if 0
{
    MeshSearch msearch(mesh);
    msearch.allow_local_project(true);
    { HH_TIMER(_spatial_create); msearch.build_spatial(); }
    Bary bary; Point clp; float d2;
    Face f = msearch(p, bary, clp, d2);
}
#endif

namespace hh {

struct PolygonFace {
    PolygonFace()                               = default;
    PolygonFace(Polygon p, Face f)              : poly(std::move(p)), face(f) { }
    Polygon poly;
    Face face;
};

namespace details {

struct polygonface_approx_distance2 {
    float operator()(const Point& p, Univ id) const {
        const PolygonFace& polyface = *Conv<const PolygonFace*>::d(id);
        const Polygon& poly = polyface.poly;
        ASSERTX(poly.num()==3);
        return square(lb_dist_point_triangle(p, poly[0], poly[1], poly[2]));
    }
};

struct polygonface_distance2 {
    float operator()(const Point& p, Univ id) const {
        const PolygonFace& polyface = *Conv<const PolygonFace*>::d(id);
        const Polygon& poly = polyface.poly;
        ASSERTX(poly.num()==3);
        return dist_point_triangle2(p, poly[0], poly[1], poly[2]);
    }
};

} // namespace details

// A spatial data structure over a collection of polygons (each associated with a Mesh Face).
class PolygonFaceSpatial :
        public ObjectSpatial<details::polygonface_approx_distance2, details::polygonface_distance2> {
 public:
    explicit PolygonFaceSpatial(int gn)         : ObjectSpatial(gn) { }
    // clear() inherited from ObjectSpatial
    void enter(const PolygonFace* polyface); // not copied, no ownership taken
    bool first_along_segment(const Point& p1, const Point& p2,
                             const PolygonFace*& ret_ppolyface, Point& ret_pint) const;
};

// Construct a spatial data structure from a mesh, to enable fast closest-point queries from arbitrary points.
// Optionally, tries to speed up the search by caching the result of the previous search and incrementally
// walking over the mesh from that prior result.
class MeshSearch {
 public:
    MeshSearch(const GMesh& mesh, bool allow_local_project);
    void allow_internal_boundaries(bool b)      { _allow_internal_boundaries = b; }
    void allow_off_surface(bool b)              { _allow_off_surface = b; }
    // search() is thread-safe (except for Random::G?)
    Face search(const Point& p, Face hintf, Bary& bary, Point& clp, float& d2) const;
    const GMesh& mesh() const                   { return _mesh; }
 private:
    const GMesh& _mesh;
    bool _allow_local_project;
    Array<PolygonFace> _ar_polyface;
    unique_ptr<PolygonFaceSpatial> _ppsp;
    Frame _ftospatial;
    bool _allow_internal_boundaries {false};
    bool _allow_off_surface {false};
};

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_MESHSEARCH_H_
