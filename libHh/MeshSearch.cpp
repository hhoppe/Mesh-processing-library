// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshSearch.h"

#include "libHh/GeomOp.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"

namespace hh {

namespace {

bool in_spherical_triangle(const Point& p, const Vec3<Point>& triangle) {
  ASSERTX(is_unit(p));
  for_int(i, 3) ASSERTX(is_unit(triangle[i]));
  const float dotcross_eps = 2e-7f;
  for_int(i, 3) if (dot(Vector(p), cross(p, triangle[i], triangle[mod3(i + 1)])) < -dotcross_eps) return false;
  return true;
}

// Given the point `p` on the unit sphere and a spherical triangle assumed to enclose it, return the barycentric
// coordinates of the spherical projection of `p` onto the planar triangle.
Bary gnomonic_get_bary(const Point& p, const Vec3<Point>& triangle) {
  assertx(in_spherical_triangle(p, triangle));
  const Line line{Point(0.f, 0.f, 0.f), p};
  const Plane plane = plane_of_triangle(triangle);
  const Point pint = intersect_line_with_plane(line, plane).value();
  const auto [d2, bary, clp] = project_point_triangle(pint, triangle);
  const float tolerance2 = 1e-9f;
  if (!assertw(d2 < tolerance2) && 0) SHOW(p), SHOW(triangle), SHOW(pint), SHOW(clp), SHOW(bary), SHOW(d2);
  return bary;
}

// Given point `p` on the unit sphere, and some "nearby" spherical triangle `f` in `mesh`, find the actual spherical
// triangle f containing p, and the barycentric coordinates of the spherical projection of p onto f.
void gnomonic_search_bary(const Point& p, const GMesh& mesh, Face& f, Bary& bary, float tolerance = 0.f) {
  Vec3<Point> triangle;
  {
    // Find the spherical triangle f (with vertex points `triangle`) containing p.
    int nfchanges = 0;
    for (;;) {
      triangle = mesh.triangle_points(f);
      // Adapted from MeshSearch.cpp .
      Vec3<bool> outside;
      for_int(i, 3) outside[i] =
          spherical_triangle_is_flipped(V(p, triangle[mod3(i + 1)], triangle[mod3(i + 2)]), tolerance);
      int num_outside = sum<int>(outside);
      if (num_outside == 0) break;
      const Vec3<Vertex> va = mesh.triangle_vertices(f);
      if (num_outside == 2) {
        const int side = index(outside, false);
        // Fastest: jump across the vertex.
        Vertex v = va[side];
        const int val = mesh.degree(v);
        // const int nrot = ((val - 1) / 2) + (Random::G.unif() < 0.5f);  // Ideal, but Random is not thread-safe.
        constexpr auto pseudo_randoms = V(0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1);
        const int nrot = ((val - 1) / 2) + pseudo_randoms[nfchanges % pseudo_randoms.Num];
        for_int(i, nrot) f = assertx(mesh.ccw_face(v, f));
      } else if (num_outside == 1) {
        const int side = index(outside, true);
        Face f2 = mesh.opp_face(va[side], f);
        if (!assertw(f2)) break;
        f = f2;
      } else {
        assertnever("");
      }
      nfchanges++;
      assertx(nfchanges < 200);
    }
    HH_SSTAT(Snfchanges, nfchanges);
  }
  bary = gnomonic_get_bary(p, triangle);
}

}  // namespace

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
      const Vec3<Point> triangle = _mesh.triangle_points(f);
      const auto proj = project_point_triangle(p, triangle);
      result.d2 = proj.d2, result.bary = proj.bary, result.clp = proj.clp;
      const float dfrac = sqrt(result.d2) * _xform[0][0];
      // if (!nfchanges) { HH_SSTAT(Sms_dfrac0, dfrac); }
      if (dfrac > 2e-2f) {  // Failure.
        f = nullptr;
        break;
      }
      if (dfrac < 1e-6f) break;  // Success.
      const Vec3<Vertex> va = _mesh.triangle_vertices(f);
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
  HH_SSTAT(Sms_local, !!f);
  if (!f) {
    const Point pbb = p * _xform;
    SpatialSearch<TriangleFace*> ss(_spatial.get(), pbb);
    const TriangleFace& triangleface = *ss.next().id;
    f = triangleface.face;
    const Vec3<Point> triangle = _mesh.triangle_points(f);  // (Without _xform transformation.)
    const auto proj = project_point_triangle(p, triangle);
    result.d2 = proj.d2, result.bary = proj.bary, result.clp = proj.clp;
  }
  result.f = f;
  return result;
}

MeshSearch::ResultOnSphere MeshSearch::search_on_sphere(const Point& p, Face hint_f, const Point* final_p) const {
  auto [f, bary, unused_clp, unused_d2] = search(p, hint_f);
  gnomonic_search_bary(p, _mesh, f, bary);  // Modifies f and bary.
  if (final_p) {
    // Now use the obtained face f but search for the final position final_p and use some nonzero tolerance to
    // hopefully avoid crossing over to the wrong side of the parametric uv discontinuity.
    const float tolerance = 1e-7f;
    gnomonic_search_bary(*final_p, _mesh, f, bary, tolerance);  // Modifies f and bary.
  }
  return {f, bary};
}

}  // namespace hh
