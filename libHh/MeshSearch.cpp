// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshSearch.h"

#include "libHh/Bbox.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"

namespace hh {

// *** PolygonFaceSpatial

void PolygonFaceSpatial::enter(const PolygonFace* ppolyface) {
  const Polygon& opoly = ppolyface->poly;
  assertx(opoly.num() == 3);
  Polygon poly = opoly;
  const Bbox bbox{poly};
  const auto func_polygonface_in_bbox = [&](const Bbox<float, 3>& spatial_bbox) -> bool {
    for_int(c, 3) if (bbox[0][c] > spatial_bbox[1][c] || bbox[1][c] < spatial_bbox[0][c]) return false;
    const bool modified = poly.intersect_bbox(spatial_bbox);
    const bool ret = poly.num() > 0;
    if (modified) poly = opoly;
    return ret;
  };
  ObjectSpatial::enter(Conv<const PolygonFace*>::e(ppolyface), ppolyface->poly[0], func_polygonface_in_bbox);
}

bool PolygonFaceSpatial::first_along_segment(const Point& p1, const Point& p2, const PolygonFace*& ret_ppolyface,
                                             Point& ret_pint) const {
  Vector vray = p2 - p1;
  bool found_intersection = false;
  float ret_fmin;
  dummy_init(ret_fmin);
  const auto func_test_polygonface_with_ray = [&](Univ id) -> bool {
    const PolygonFace* ppolyface = Conv<const PolygonFace*>::d(id);
    const Polygon& poly = ppolyface->poly;
    Point pint;
    if (!poly.intersect_segment(p1, p2, pint)) return false;
    float f = dot(pint - p1, vray);
    if (!found_intersection || f < ret_fmin) {
      ret_fmin = f;
      ret_pint = pint;
      ret_ppolyface = ppolyface;
      found_intersection = true;
    }
    return true;
  };
  search_segment(p1, p2, func_test_polygonface_with_ray);
  return found_intersection;
}

// *** TriangleFaceSpatial

namespace {

struct triangleface_approx_distance2 {
  float operator()(const Point& p, Univ id) const {
    const TriangleFace& triangleface = *Conv<const TriangleFace*>::d(id);
    const Vec3<Point>& points = triangleface._points;
    return square(lb_dist_point_triangle(p, points[0], points[1], points[2]));
  }
};

struct triangleface_distance2 {
  float operator()(const Point& p, Univ id) const {
    const TriangleFace& triangleface = *Conv<const TriangleFace*>::d(id);
    const Vec3<Point>& points = triangleface._points;
    return dist_point_triangle2(p, points[0], points[1], points[2]);
  }
};

}  // namespace

// A spatial data structure over a collection of mesh triangle faces.
class MeshSearch::TriangleFaceSpatial : public ObjectSpatial<triangleface_approx_distance2, triangleface_distance2> {
 public:
  explicit TriangleFaceSpatial(int gn) : ObjectSpatial(gn) {}
  // clear() inherited from ObjectSpatial
  void enter(const TriangleFace* ptriangleface) {  // not copied, no ownership taken
    const Vec3<Point>& points = ptriangleface->_points;
    Polygon poly{points}, opoly = poly;
    const Bbox bbox{points};
    const auto func_triangleface_in_bbox = [&](const Bbox<float, 3>& spatial_bbox) -> bool {
      for_int(c, 3) if (bbox[0][c] > spatial_bbox[1][c] || bbox[1][c] < spatial_bbox[0][c]) return false;
      const bool modified = poly.intersect_bbox(spatial_bbox);  // TODO: Speed this up somehow.
      const bool ret = poly.num() > 0;
      if (modified) poly = opoly;
      return ret;
    };
    ObjectSpatial::enter(Conv<const TriangleFace*>::e(ptriangleface), points[0], func_triangleface_in_bbox);
  }
  bool first_along_segment(const Point& p1, const Point& p2, const TriangleFace*& ret_ptriangleface,
                           Point& ret_pint) const {
    Vector vray = p2 - p1;
    bool found_intersection = false;
    float ret_fmin;
    dummy_init(ret_fmin);
    const auto func_test_triangleface_with_ray = [&](Univ id) -> bool {
      const TriangleFace* ptriangleface = Conv<const TriangleFace*>::d(id);
      const Polygon poly{ptriangleface->_points};
      Point pint;
      if (!poly.intersect_segment(p1, p2, pint)) return false;  // TODO: Speed this up by avoiding use of Polygon.
      float f = dot(pint - p1, vray);
      if (!found_intersection || f < ret_fmin) {
        ret_fmin = f;
        ret_pint = pint;
        ret_ptriangleface = ptriangleface;
        found_intersection = true;
      }
      return true;
    };
    search_segment(p1, p2, func_test_triangleface_with_ray);
    return found_intersection;
  }
};

// *** MeshSearch

MeshSearch::MeshSearch(const GMesh& mesh, Options options) : _mesh(mesh), _options(std::move(options)) {
  if (getenv_bool("NO_LOCAL_PROJECT")) {
    Warning("MeshSearch NO_LOCAL_PROJECT");
    _options.allow_local_project = false;
  }
  int psp_size = int(sqrt(_mesh.num_faces() * .05f));
  if (_options.allow_local_project) psp_size /= 2;
  psp_size = clamp(psp_size, 15, 200);  // Revisit upper bound for nefertiti ??
  HH_STIMER("__meshsearch_build");
  if (!_options.bbox) _options.bbox.emplace(transform(_mesh.vertices(), [&](Vertex v) { return _mesh.point(v); }));
  _xform = _options.bbox->get_frame_to_small_cube();
  _trianglefaces.reserve(mesh.num_faces());
  for (Face f : _mesh.faces()) {
    Vec3<Point> points = mesh.triangle_points(f);
    for_int(i, 3) points[i] *= _xform;
    _trianglefaces.push({points, f});
  }
  _spatial = make_unique<TriangleFaceSpatial>(psp_size);
  for (const TriangleFace& triangleface : _trianglefaces) _spatial->enter(&triangleface);
}

MeshSearch::~MeshSearch() {}

MeshSearch::Result MeshSearch::search(const Point& p, Face hint_f) const {
  Result result;
  Face f = nullptr;
  if (_options.allow_local_project && hint_f) {
    f = hint_f;
    int nfchanges = 0;
    for (;;) {
      Vec3<Point> points = _mesh.triangle_points(f);
      result.d2 = project_point_triangle2(p, points[0], points[1], points[2], result.bary, result.clp);
      float dfrac = sqrt(result.d2) * _xform[0][0];
      // if (!nfchanges) { HH_SSTAT(Sms_dfrac0, dfrac); }
      if (dfrac > 2e-2f) {  // Failure.
        f = nullptr;
        break;
      }
      if (dfrac < 1e-6f) break;  // Success.
      Vec3<Vertex> va = _mesh.triangle_vertices(f);
      int side = result.bary.const_view().index(1.f);
      if (side >= 0) {
        if (0) {  // Slow: randomly choose ccw or clw.
          // side = mod3(side + 1 + (Random::G.unif() < 0.5f));
        } else if (0) {  // Works: always choose ccw.
          // side = mod3(side + 1);
        } else {  // Fastest: jump across vertex.
          Vertex v = va[side];
          int val = _mesh.degree(v);
          // const int nrot = ((val - 1) / 2) + (Random::G.unif() < 0.5f);  // Ideal, but Random is not thread-safe.
          constexpr auto pseudo_randoms = V(0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1);
          const int nrot = ((val - 1) / 2) + pseudo_randoms[nfchanges];
          for_int(i, nrot) {
            f = _mesh.ccw_face(v, f);
            if (!f) break;  // Failure.
          }
          side = -1;
        }
      } else {
        side = result.bary.const_view().index(0.f);
        if (side < 0) {
          if (_options.allow_off_surface) break;     // Success.
          if (_options.allow_internal_boundaries) {  // Failure.
            f = nullptr;
            break;
          }
        }
      }
      if (side >= 0) f = _mesh.opp_face(va[side], f);
      if (!f) {
        if (!_options.allow_internal_boundaries) assertnever("MeshSearch has hit surface boundary");
        break;  // Failure.
      }
      if (++nfchanges == 10) {  // Failure.
        f = nullptr;
        break;
      }
    }
    // HH_SSTAT(Sms_nfchanges, nfchanges);
  }
  HH_SSTAT(Sms_loc, !!f);
  if (!f) {
    const Point pbb = p * _xform;
    SpatialSearch<TriangleFace*> ss(_spatial.get(), pbb);
    const TriangleFace& triangleface = *assertx(ss.next());
    f = triangleface._f;
    const Vec3<Point> points = _mesh.triangle_points(f);  // (Without _xform transformation.)
    result.d2 = project_point_triangle2(p, points[0], points[1], points[2], result.bary, result.clp);
  }
  result.f = f;
  return result;
}

}  // namespace hh
