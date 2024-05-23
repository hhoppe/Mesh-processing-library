// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshSearch.h"

#include "libHh/Bbox.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"

namespace hh {

void PolygonFaceSpatial::enter(const PolygonFace* ppolyface) {
  const Polygon& opoly = ppolyface->poly;
  assertx(opoly.num() == 3);
  Polygon poly = opoly;
  const Bbox bbox{poly};
  auto func_polygonface_in_bbox = [&](const Bbox<float, 3>& spatial_bbox) -> bool {
    for_int(c, 3) {
      if (bbox[0][c] > spatial_bbox[1][c] || bbox[1][c] < spatial_bbox[0][c]) return false;
    }
    int modif = poly.intersect_bbox(spatial_bbox);
    bool ret = poly.num() > 0;
    if (modif) poly = opoly;
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
  auto func_test_polygonface_with_ray = [&](Univ id) -> bool {
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

MeshSearch::MeshSearch(const GMesh& mesh, Options options)
    : _mesh(mesh), _options(std::move(options)), _ar_polyface(_mesh.num_faces()) {
  if (getenv_bool("NO_LOCAL_PROJECT")) {
    Warning("MeshSearch NO_LOCAL_PROJECT");
    _options.allow_local_project = false;
  }
  int psp_size = int(sqrt(_mesh.num_faces() * .05f));
  if (_options.allow_local_project) psp_size /= 2;
  psp_size = clamp(10, psp_size, 150);
  HH_STIMER("__meshsearch_build");
  const Bbox bbox{transform(_mesh.vertices(), [&](Vertex v) { return _mesh.point(v); })};
  _ftospatial = bbox.get_frame_to_small_cube();
  int fi = 0;
  for (Face f : _mesh.faces()) {
    Polygon poly(3);
    _mesh.polygon(f, poly);
    assertx(poly.num() == 3);
    for_int(i, 3) poly[i] *= _ftospatial;
    _ar_polyface[fi] = PolygonFace(std::move(poly), f);
    fi++;
  }
  _ppsp = make_unique<PolygonFaceSpatial>(psp_size);
  for (PolygonFace& polyface : _ar_polyface) _ppsp->enter(&polyface);
}

MeshSearch::Result MeshSearch::search(const Point& p, Face hintf) const {
  Result result;
  Face f = nullptr;
  Polygon poly;
  if (_options.allow_local_project && hintf) {
    f = hintf;
    int nfchanges = 0;
    for (;;) {
      _mesh.polygon(f, poly);
      assertx(poly.num() == 3);
      result.d2 = project_point_triangle2(p, poly[0], poly[1], poly[2], result.bary, result.clp);
      float dfrac = sqrt(result.d2) * _ftospatial[0][0];
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
        for_int(i, 3) {
          if (result.bary[i] == 0.f) {
            side = i;
            break;
          }
        }
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
    Point pbb = p * _ftospatial;
    SpatialSearch<PolygonFace*> ss(_ppsp.get(), pbb);
    const PolygonFace& polyface = *assertx(ss.next());
    f = polyface.face;
    _mesh.polygon(f, poly);
    assertx(poly.num() == 3);
    result.d2 = project_point_triangle2(p, poly[0], poly[1], poly[2], result.bary, result.clp);
  }
  result.f = f;
  return result;
}

}  // namespace hh
