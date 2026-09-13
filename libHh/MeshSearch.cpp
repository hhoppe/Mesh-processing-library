// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshSearch.h"

#include "libHh/GeomOp.h"
#include "libHh/RangeOp.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"

namespace hh {

namespace {

// Tolerance on the signed volumes dot(p, cross(p, triangle[i], triangle[i + 1])) when testing if the point p on the
// unit sphere lies within a spherical triangle.
constexpr float k_dotcross_eps = 5e-7f;  // Was 2e-7f.

bool in_spherical_triangle(const Point& p, const Vec3<Point>& triangle) {
  ASSERTX(is_unit(p));
  for_int(i, 3) ASSERTX(is_unit(triangle[i]));
  for_int(i, 3) if (dot(Vector(p), cross(p, triangle[i], triangle[mod3(i + 1)])) < -k_dotcross_eps) return false;
  return true;
}

// Given the point `p` on the unit sphere and a spherical triangle assumed to enclose it, return the barycentric
// coordinates of the spherical projection of `p` onto the planar triangle.
Bary gnomonic_get_bary(const Point& p, const Vec3<Point>& triangle) {
  ASSERTX(is_unit(p));
  for_int(i, 3) ASSERTX(is_unit(triangle[i]));
  // The barycentric coordinates are proportional to the signed volumes of the tetrahedra (origin, p, triangle[i + 1],
  // triangle[i + 2]); these same signed volumes determine the enclosing face in `gnomonic_search_bary()`.
  // This closed form avoids explicitly forming the triangle plane, whose normal is ill-conditioned for slivers.
  using Precision = double;
  const auto q = convert<Precision>(p);
  const auto tri = transformed(triangle, [](const Point& p2) { return convert<Precision>(p2); });
  Vec3<Precision> weights;
  for_int(i, 3) weights[i] = dot(q, cross(tri[mod3(i + 1)], tri[mod3(i + 2)]));
  assertw(min(weights) >= -Precision{k_dotcross_eps});  // The point p should lie within the spherical triangle.
  for_int(i, 3) weights[i] = max(weights[i], Precision{0});  // Tolerate p lying just outside the triangle.
  const Precision sum_weights = sum(weights);
  if (!assertw(sum_weights > Precision{0})) return Bary(1.f / 3.f, 1.f / 3.f, 1.f / 3.f);
  return convert<float>(weights / sum_weights);
}

// Whether to rank candidate faces using the exact distance from p to each spherical triangle rather than a cheaper
// lower bound.  Both measures are monotonic in the angular distance, but they are not in the same units, so the
// choice must be made once here rather than per call.  The two select the same face in about 98% of calls, and the
// search converges equally fast either way because a suboptimal choice merely costs an extra iteration.
constexpr bool k_use_exact_spherical_dist = false;

// Return the sine of a lower bound on the angular distance from the point p on the unit sphere to the spherical
// triangle; the bound is zero exactly when p lies within the triangle, and is tight when the closest point of the
// triangle lies in the interior of an edge.  Where the closest point is instead a triangle vertex whose interior
// angle is `angle`, the bound under-estimates by as much as a factor sin(angle / 2), so it is only suitable for
// ranking triangles that share that vertex.  (The distance from a point to a great circle never exceeds TAU / 4,
// so the sine is monotonic over the range of the bound; a caller wanting radians applies std::asin().)
float sin_spherical_dist_lower_bound(const Point& p, const Vec3<Point>& triangle) {
  using Precision = double;
  const auto q = convert<Precision>(p);
  const auto tri = transformed(triangle, [](const Point& p2) { return convert<Precision>(p2); });
  Precision max_sin_outside = 0.;
  for_int(i, 3) {
    // The great circle containing the triangle edge (i + 1, i + 2) has unit normal `normal / normal_mag`, and the
    // signed distance of p from that great circle is asin(dot(q, normal) / normal_mag), positive on the inside.
    const auto normal = cross(tri[mod3(i + 1)], tri[mod3(i + 2)]);
    const Precision normal_mag = mag(normal);
    if (normal_mag) max_sin_outside = max(max_sin_outside, -dot(q, normal) / normal_mag);
  }
  return float(max_sin_outside);
}

// Return the squared Euclidean (chordal) distance from the point p on the unit sphere to the spherical triangle, or
// zero if p lies within the triangle.  The chordal distance 2 * sin(angle / 2) is monotonic in the angular distance
// over its full range [0, TAU / 2], so it is a valid substitute for ranking, and 2 * asin(sqrt(result) / 2) recovers
// the angle.  Unlike the lower bound above, this is correct for a point lying beyond a triangle vertex.
float dist2_to_spherical_triangle(const Point& p, const Vec3<Point>& triangle) {
  using Precision = double;
  const auto q = convert<Precision>(p);
  const auto tri = transformed(triangle, [](const Point& p2) { return convert<Precision>(p2); });
  // The great circle containing the triangle edge (i + 1, i + 2) has normal `normals[i]`, pointing to the inside.
  Vec3<Vec3<Precision>> normals;
  for_int(i, 3) normals[i] = cross(tri[mod3(i + 1)], tri[mod3(i + 2)]);
  bool inside = true;
  for_int(i, 3) if (dot(q, normals[i]) < Precision{0}) inside = false;
  if (inside) return 0.f;
  Precision min_dist2 = 4.;  // The largest possible squared chordal distance on the unit sphere.
  for_int(i, 3) {  // The closest point lies on the triangle boundary, so consider each of the three edge arcs.
    const auto& u = tri[mod3(i + 1)];
    const auto& w = tri[mod3(i + 2)];
    const Precision normal_mag = mag(normals[i]);
    // The perpendicular foot of q on the great circle lies within the arc (u, w) iff it is ccw of u and cw of w.
    const bool foot_within_arc = dot(cross(u, q), normals[i]) >= 0. && dot(cross(q, w), normals[i]) >= 0.;
    const Precision cos_dist = foot_within_arc && normal_mag
                                   ? sqrt(max(1. - square(dot(q, normals[i]) / normal_mag), Precision{0}))
                                   : max(dot(q, u), dot(q, w));  // Else the closest point is an arc endpoint.
    min_dist2 = min(min_dist2, 2. - 2. * cos_dist);
  }
  return float(min_dist2);
}

struct GnomonicSearchOptions {
  float tolerance = 0.f;
  bool avoid_crossing_axial_planes = false;
  bool warn_no_opp_face = true;
};

// Given point `p` on the unit sphere, and some "nearby" spherical triangle `f` in `mesh`, find the actual spherical
// triangle f containing p, and the barycentric coordinates of the spherical projection of p onto f.
void gnomonic_search_bary(const Point& p, const GMesh& mesh, Face& f, Bary& bary,
                          const GnomonicSearchOptions& options) {
  Vec3<Point> triangle;
  {
    // Find the spherical triangle f (with vertex points `triangle`) containing p.
    int nfchanges = 0;
    for (;;) {
      triangle = mesh.triangle_points(f);
      // Adapted from MeshSearch.cpp .
      Vec3<bool> outside;
      for_int(i, 3) {
        const Point& p1 = triangle[mod3(i + 1)];
        const Point& p2 = triangle[mod3(i + 2)];
        outside[i] = spherical_triangle_is_flipped(V(p, p1, p2), options.tolerance);
        if (options.avoid_crossing_axial_planes && outside[i]) {
          const bool edge_is_along_an_axial_plane = (!p1[0] && !p2[0]) || (!p1[1] && !p2[1]) || (!p1[2] && !p2[2]);
          if (edge_is_along_an_axial_plane) outside[i] = false;
        }
      }
      int num_outside = sum<int>(outside);
      if (num_outside == 0) break;
      const Vec3<Vertex> va = mesh.triangle_vertices(f);

      if (num_outside == 1) {  // Jump across the edge.
        const int side = index(outside, true);
        Face f2 = mesh.opp_face(va[side], f);
        if (!f2) {
          if (options.warn_no_opp_face) Warning("gnomonic_search_bary: no opp_face");
          break;
        }
        f = f2;

      } else if (num_outside == 2) {  // Jump across the vertex.
        const int side = index(outside, false);
        Vertex v = va[side];
        // We find the face with smallest distance from p.
        float min_dist = BIGFLOAT;
        Face min_f{};
        for (Face f2 : mesh.faces(v)) {
          if (f2 == f) continue;
          const Vec3<Point> triangle2 = mesh.triangle_points(f2);
          const float dist = k_use_exact_spherical_dist ? dist2_to_spherical_triangle(p, triangle2)
                                                        : sin_spherical_dist_lower_bound(p, triangle2);
          if (dist < min_dist) min_dist = dist, min_f = f2;
        }
        f = min_f;

      } else if (num_outside == 3) {  // Likely a degenerate spherical triangle.
        // Occurred on:
        // SphereSample -domain octaflat -grid 4096 -domain_file domains/octaflat_eg128.uv.sphparam.m -param $tmp/$r.octaflat.sphparam.m -signal N -write_texture images/$r.octaflat.normalmap.png
        // SHOW(gnomonic_get_bary(p, triangle)), SHOW_PRECISE(spherical_triangle_area(triangle));
        assertw(spherical_triangle_is_flipped(triangle));
        assertw(spherical_triangle_area(triangle) == 0.f);
        assertw(in_spherical_triangle(p, triangle));
        Warning("num_outside=3; likely a degenerate spherical triangle");
        break;

      } else {
        assertnever("");
      }
      nfchanges++;
      assertx(nfchanges < 2000);  // Was 200, but raised due to high-valence vertex at center of triangulated face.
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
  if (!_options.bbox)
    _options.bbox.emplace(_mesh.vertices() | views::transform([&](Vertex v) { return _mesh.point(v); }));
  _xform = _options.bbox->get_frame_to_small_cube();
  _trianglefaces.reserve(mesh.num_faces());
  for (Face f : _mesh.faces()) {
    Vec3<Point> triangle = mesh.triangle_points(f);
    for_int(i, 3) triangle[i] *= _xform;
    _trianglefaces.push({triangle, f});
  }
  int gridn = int(sqrt(_mesh.num_faces() * .02f) * _options.gridn_factor);  // Was .05f.
  if (_options.allow_local_project) gridn /= 2;
  gridn = clamp(gridn, 10, Spatial::k_max_gn);
  _spatial.emplace(_trianglefaces, gridn);
}

MeshSearch::Result MeshSearch::search(const Point& p, Face hint_f) const {
  Result result{};
  Face f = nullptr;
  if (_options.allow_local_project && hint_f) {
    f = hint_f;
    int nfchanges = 0;
    for (;;) {
      const Vec3<Point> triangle = _mesh.triangle_points(f);
      const auto proj = project_point_triangle(p, triangle);
      result.d2 = proj.d2, result.bary = proj.bary, result.clp = proj.clp;
      const float dfrac = sqrt(result.d2) * _xform[0, 0];
      // if (!nfchanges) { HH_SSTAT(Sms_dfrac0, dfrac); }
      if (dfrac > 2e-2f) {  // Failure.
        f = nullptr;
        break;
      }
      if (dfrac < 1e-6f) break;  // Success.
      const Vec3<Vertex> va = _mesh.triangle_vertices(f);
      int side = find_index(result.bary, 1.f).value_or(-1);
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
        side = find_index(result.bary, 0.f).value_or(-1);
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
    const float max_dis_bb = _options.max_dis * _xform[0, 0];
    SpatialSearch<TriangleFace*> ss(&*_spatial, pbb, max_dis_bb);
    if (ranges::empty(ss)) {  // No triangle within max_dis;
      result.bary = thrice(NAN);
      result.clp = thrice(NAN);
      result.d2 = NAN;
    } else {
      const TriangleFace& triangleface = *(*ss.begin()).id;
      f = triangleface.face;
      const Vec3<Point> triangle = _mesh.triangle_points(f);  // (Without _xform transformation.)
      const auto proj = project_point_triangle(p, triangle);
      result.bary = proj.bary, result.clp = proj.clp, result.d2 = proj.d2;
    }
  }
  result.f = f;
  return result;
}

MeshSearch::ResultOnSphere MeshSearch::search_on_sphere(const Point& p, Face hint_f, const Point* final_p) const {
  auto [f, bary, unused_clp, unused_d2] = search(p, hint_f);
  assertx(f);
  // Modifies f and bary.
  gnomonic_search_bary(p, _mesh, f, bary, {.warn_no_opp_face = _options.gnomonic_search_warn_no_opp_face});
  if (final_p) {
    // Starting from the obtained face f, repeat the search but (1) search instead for final_p and (2) avoid
    // crossing the octaflat axial planes (because these may contain parametric uv discontinuities).
    // Modifies f and bary.
    gnomonic_search_bary(*final_p, _mesh, f, bary, {.tolerance = 1e-7f, .avoid_crossing_axial_planes = true});
  }
  return {f, bary};
}

}  // namespace hh
