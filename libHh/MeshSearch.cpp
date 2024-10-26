// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshSearch.h"

#include "libHh/Stat.h"
#include "libHh/Timer.h"

namespace hh {

MeshSearch::MeshSearch(const GMesh& mesh, Options options) : _mesh(mesh), _options(std::move(options)) {
  if (getenv_bool("NO_LOCAL_PROJECT")) {
    Warning("MeshSearch NO_LOCAL_PROJECT");
    _options.allow_local_project = false;
  }
  HH_STIMER("__meshsearch_build");
  if (!_options.bbox) _options.bbox.emplace(transform(_mesh.vertices(), [&](Vertex v) { return _mesh.point(v); }));
  _xform = _options.bbox->get_frame_to_small_cube();
  _trianglefaces.reserve(mesh.num_faces());
  for (Face f : _mesh.faces()) {
    Vec3<Point> triangle = mesh.triangle_points(f);
    for_int(i, 3) triangle[i] *= _xform;
    _trianglefaces.push({triangle, f});
  }
  int gridn = int(sqrt(_mesh.num_faces() * .05f));
  if (_options.allow_local_project) gridn /= 2;
  gridn = clamp(gridn, 15, 200);  // Revisit upper bound for nefertiti ??
  _spatial = make_unique<TriangleFaceSpatial>(_trianglefaces, gridn);
}

MeshSearch::~MeshSearch() {}

MeshSearch::Result MeshSearch::search(const Point& p, Face hint_f) const {
  Result result;
  Face f = nullptr;
  if (_options.allow_local_project && hint_f) {
    f = hint_f;
    int nfchanges = 0;
    for (;;) {
      Vec3<Point> triangle = _mesh.triangle_points(f);
      result.d2 = project_point_triangle2(p, triangle[0], triangle[1], triangle[2], result.bary, result.clp);
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
    f = triangleface.face;
    const Vec3<Point> triangle = _mesh.triangle_points(f);  // (Without _xform transformation.)
    result.d2 = project_point_triangle2(p, triangle[0], triangle[1], triangle[2], result.bary, result.clp);
  }
  result.f = f;
  return result;
}

}  // namespace hh
