// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "SphereMapper.h"

#include "boost_minima.h"

#include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"
#include "libHh/Random.h"
#include "libHh/SingularValueDecomposition.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"

namespace hh {

namespace {

// Return spherical area of ordered triangle, in the range [0, 2 * TAU].
template <typename T> T spherical_triangle_area(const Vec3<T>& pd0, const Vec3<T>& pd1, const Vec3<T>& pd2) {
  using TT = double;
  // Calculate the angular lengths of the triangle sides using the norms of the cross products of the vertices.
  const Vec3<T> cross_a = cross(pd1, pd2);
  const Vec3<T> cross_b = cross(pd2, pd0);
  const Vec3<T> cross_c = cross(pd0, pd1);
  const bool larger_than_hemisphere = dot(cross_a, pd0) < 0.f;  // (Only relevant for tetra base mesh.)
  const TT a = mag<TT>(cross_a);
  const TT b = mag<TT>(cross_b);
  const TT c = mag<TT>(cross_c);
  // Compute the semi-perimeter.
  const TT s = (a + b + c) / 2;
  // Use L'Huilier's Theorem to find the spherical excess e (== area) of the unoriented spherical triangle.
  const T tan_half_omega = sqrt(
      max(std::tan(T(s / 2)) * std::tan(T((s - a) / 2)) * std::tan(T((s - b) / 2)) * std::tan(T((s - c) / 2)), T(0)));
  T e = 4 * std::atan(tan_half_omega);
  if (larger_than_hemisphere) e = T(2 * D_TAU) - e;
  return e;
}

inline bool vertex_is_on_sharp_edge(const AWMesh& awmesh, int v, int someface, int& vl, int& vr) {
  int w_last = -1;
  for (const auto& [_, ff] : awmesh.ccw_vertices(v, someface)) w_last = awmesh.get_wvf(v, ff);
  vl = vr = -1;
  for (const auto& [vv, ff] : awmesh.ccw_vertices(v, someface)) {
    const int w = awmesh.get_wvf(v, ff);
    if (w != w_last) {
      if (vl < 0) {
        vl = vv;
        w_last = w;
      } else {
        vr = vv;
        return true;
      }
    }
  }
  assertx(vl < 0);
  return false;
}

inline Vector average_normal_for_vertex(const AWMesh& awmesh, int v, int someface) {
  Vector normal{};
  for (const int f : awmesh.ccw_faces(v, someface)) {
    for_int(j, 3) {
      const int w = awmesh._faces[f].wedges[j];
      if (awmesh._wedges[w].vertex == v)
        normal += awmesh._wedges[w].attrib.normal;  // Could use a face-weighted sum as in Vnors.
    }
  }
  return normalized(normal);
}

inline Vector interp_on_arc(const Vector& x0, const Vector& dir, float t) {
  return x0 * std::cos(t) + dir * std::sin(t);
}

inline Vector get_random_unit_vector(float rand1, float rand2) {
  ASSERTX(rand1 >= 0.f && rand1 < 1.f && rand2 >= 0.f && rand2 < 1.f);
  const float theta = rand1 * TAU;  // Longitude (angle in x-y plane).
  const float z = rand2 * 2.f - 1.f;
  const float r = sqrt(1.f - square(z));  // Radius in x-y plane.
  return Vector(r * std::cos(theta), r * sin(theta), z);
}

}  // namespace

// ***

class SphereMapper::Implementation {
 public:
  Implementation(PMeshIter& pmi, Options options) : _pmi(pmi), _options(std::move(options)) {
    const auto initialize_param = [](auto& param, const string& name, auto default_value) {
      param = getenv_type("spheremapper_" + name, default_value);
    };
    assertx(_options.effort >= 0 && options.effort <= 5);
    const int effort = _options.effort;
    initialize_param(_optim_line_search_iter, "optim_line_search_iter", V(6, 8, 10, 40, 100, 200)[effort]);
    initialize_param(_optim_vsplit_vt_iter, "optim_vsplit_vt_iter", V(1, 1, 2, 3, 5, 5)[effort]);
    initialize_param(_optim_vsplit_nei_iter, "optim_vsplit_nei_iter", V(0, 0, 0, 3, 5, 20)[effort]);
    initialize_param(_optim_global_iter, "optim_global_iter", V(1, 2, 3, 5, 8, 20)[effort]);
    initialize_param(_optim_movetol, "optim_movetol", V(1e-2f, 5e-3f, 1e-3f, 1e-3f, 5e-4f, 3e-4f)[effort]);
    initialize_param(_optim_nv_ratio, "optim_nv_ratio", V(1.7f, 1.25f, 1.2f, 1.1f, 1.05f, 1.04f)[effort]);

    _conformal_weight = _options.conformal_weight;
    _num_fixed_vertices = _options.fix_base ? _pmi._vertices.num() : 0;

    const Materials& materials = _pmi._materials;
    for_int(matid, materials.num()) _matid_is_hole.push(contains(materials.get(matid), "hole"));

    // Compute and store the surface area of the base mesh.
    for_int(f, _pmi._faces.num()) _surface_area += sqrt(area2(_pmi.face_points(f)));

    if (_options.visualize) create_visualizer();
  }
  ~Implementation() { finalize_visualizer(); }

  void show_parameters() const {
    showdf("effort=%d optim_line_search_iter=%d optim_vsplit_vt_iter=%d optim_vsplit_nei_iter=%d\n",  //
           _options.effort, _optim_line_search_iter, _optim_vsplit_vt_iter, _optim_vsplit_nei_iter);
    showdf("optim_global_iter=%d optim_movetol=%g optim_nv_ratio=%g stretch_precision=%s conformal_weight=%g\n",  //
           _optim_global_iter, _optim_movetol, _optim_nv_ratio, type_name<Precision>().c_str(), _conformal_weight);
  }

  CArrayView<Point> compute(CArrayView<Point> base_sphmap) {
    assertx(base_sphmap.num() == _pmi._vertices.num());
    _sphmap.init(_pmi.rstream()._info._full_nvertices);
    _sphmap.head(_pmi._vertices.num()).assign(base_sphmap);
    for_int(f, _pmi._faces.num()) assertx(!face_flipped(f));
    initialize_visualizer();
    if (_options.flatten_to_x0)
      for (auto& vertex : _pmi._vertices) vertex.attrib.point[0] = 0.f;
    goto_nverts(std::numeric_limits<int>::max());
    return _sphmap;
  }

  void show_total_stretch() {
    const float old_conformal_weight = std::exchange(_conformal_weight, 0.f);
    set_stretch_scaling();
    Stat stat_tri_stretch;
    for_int(f, _pmi._faces.num()) stat_tri_stretch.enter(stretch_for_face(f));
    showdf("tri_stretch: %s\n", stat_tri_stretch.short_string().c_str());
    const float stretch = stat_tri_stretch.sum();

    const float area = float(_surface_area);
    const float rms_stretch = sqrt(stretch / (_options.optimize_inverse ? 2 * TAU : area));
    // The l2 stretch efficiency is: surface area / domain area / rms_stretch^2.
    const float l2_efficiency = (_options.optimize_inverse ? 2 * TAU : area) / stretch;
    const int num_faces = _pmi._faces.num();
    showdf("faces=%-7d area=%-11g  rms_stretch=%-7g  l2_efficiency=%.6f\n",  //
           num_faces, area, rms_stretch, l2_efficiency);
    _conformal_weight = old_conformal_weight;
  }

  Frame frame_aligning_sphmap_to_surface_normals() const {
    // Compute the rotation that best aligns each faces's centroid sphere point to its surface face normal.
    // Uses Horn's closed form solution with unit quaternions.
    SGrid<float, 3, 3> cov{};  // Weighted covariance matrix.
    for_int(f, _pmi._faces.num()) {
      const Vec3<int> vertices = _pmi.face_vertices(f);
      const Vec3<Point> triangle = _pmi.face_points(f);
      const Vec3<Point> sphs = map(vertices, [&](int v) { return _sphmap[v]; });
      Vector face_normal = get_normal_dir(triangle);
      if (!face_normal.normalize()) continue;
      const float face_area = sqrt(area2(triangle));
      const Point sph_center = normalized(sum(sphs));
      for_int(i, 3) for_int(j, 3) cov[i][j] += sph_center[i] * face_normal[j] * face_area;
    }

    SGrid<float, 4, 4> n;
    const float trace = cov[0][0] + cov[1][1] + cov[2][2];
    n[0][0] = trace;
    n[1][0] = n[0][1] = cov[1][2] - cov[2][1];
    n[2][0] = n[0][2] = cov[2][0] - cov[0][2];
    n[3][0] = n[0][3] = cov[0][1] - cov[1][0];
    for_int(i, 3) for_int(j, 3) n[i + 1][j + 1] = i == j ? (2 * cov[i][i] - trace) : (cov[i][j] + cov[j][i]);

    SGrid<float, 4, 4> u, vt;
    Vec4<float> s;
    assertx(singular_value_decomposition(n.const_view(), u.view(), s.view(), vt.view()));
    sort_singular_values(u.view(), s.view(), vt.view());
    // (Because matrix n is symmetric, the vectors in u and vt are generally identical.)

    const float q0 = u[0][0], qx = u[1][0], qy = u[2][0], qz = u[3][0];
    const Frame frame(
        Vector(q0 * q0 + qx * qx - qy * qy - qz * qz, 2.f * (qy * qx + q0 * qz), 2.f * (qz * qx - q0 * qy)),
        Vector(2.f * (qx * qy - q0 * qz), q0 * q0 - qx * qx + qy * qy - qz * qz, 2.f * (qz * qy + q0 * qx)),
        Vector(2.f * (qx * qz + q0 * qy), 2.f * (qy * qz - q0 * qx), q0 * q0 - qx * qx - qy * qy + qz * qz),
        Point(0.f, 0.f, 0.f));
    if (0) SHOW(cov, n, u, s, vt, frame);
    return frame;
  }

 private:
  using Precision = float;  // Floating-point type for neighborhood stretch computation.

  PMeshIter& _pmi;

  const Options _options;

  int _optim_line_search_iter;
  int _optim_vsplit_vt_iter;
  int _optim_vsplit_nei_iter;
  int _optim_global_iter;
  float _optim_movetol;
  float _optim_nv_ratio;

  float _conformal_weight;  // Modifiable copy of _options.conformal_weight.
  int _num_fixed_vertices;
  Array<bool> _matid_is_hole;
  Array<Point> _sphmap;
  double _surface_area{0.};
  Precision _stretch_scaling1, _stretch_scaling2, _stretch_scaling3;

  // Given a parametric arc on the sphere (from point x0, rotate by theta in direction l over the angle
  // interval [itmin, itmax], where l is tangent to the sphere at x0), determine the shrunken theta interval (modified
  // [itmin, itmax] for which the resulting point is in the positive halfspace of the provided normal.
  // It assumes all the given vectors are unit length.
  // The interval must be either the whole circle (given as (-TAU, TAU)), or smaller than a halfcircle.
  // Value itmin must be between -TAU/2 and TAU/2, so that the intersection can be expressed as a single interval.
  static void intersect_param_circle_halfspace(const Vector& x0, const Vector& l, const Vector& nhs, float& itmin,
                                               float& itmax) {
    float tmin;
    {
      const Precision x0n = Precision(dot<double>(x0, nhs)), dnhsl = Precision(dot<double>(nhs, l));
      if (!x0n && !dnhsl) return;
      tmin = possible_cast<float>(std::atan2(-x0n, dnhsl));
    }
    if (itmin == -TAU && itmax == TAU) {
      itmin = tmin;
      itmax = tmin + TAU / 2;
    } else {
      ASSERTX(itmax - itmin <= TAU / 2 + 1e-6f);
      ASSERTX(itmin >= -TAU / 2 && itmin <= TAU / 2);
      float ttmin = max(tmin, itmin), ttmax = min(tmin + TAU / 2, itmax);
      if (ttmin > ttmax && tmin > 0.f) {  // Empty interval; see if the intersection at the "other" end is also empty.
        ttmin = max(tmin - TAU, itmin);
        ttmax = min(tmin - TAU / 2, itmax);
      }
      if (ttmin > ttmax && itmax > TAU / 2) {
        ttmin = max(tmin, itmin - TAU);
        ttmax = min(tmin + TAU / 2, itmax - TAU);
      }
      itmin = ttmin;
      itmax = ttmax;
    }
  }

  // The kernel is a "convex" spherical polygon, defined as the intersection of the sphere and of each of the
  // halfspaces defined by the edges of the original polygon.
  // The initial edges are given as a list of normals to their supporting plane (great circle).
  // The centroid is computed using linear average and reprojection to sphere.
  // If the kernel is larger than a hemisphere, the centroid is flipped.
  Point kernel_centroid_of_spherical_polygon(CArrayView<Vector> enormals) {
    Vector center{};
    int n = 0;
    for_int(i, enormals.num()) {
      const Vector& enormal = enormals[i];
      // Get a point x0 on the edge.
      Vector x0;
      if (0) {  // Seemed like an improvement, but may be worse?
        if (abs(enormal[0]) < abs(enormal[1])) {
          x0 = normalized(Vector(1.f, 0.f, 0.f) - enormal[0] * enormal);
        } else {
          x0 = normalized(Vector(0.f, 1.f, 0.f) - enormal[1] * enormal);
        }
      } else {
        if (abs(enormal[0]) < 1.f - 2e-7f) {
          x0 = normalized(Vector(1.f, 0.f, 0.f) - enormal[0] * enormal);
        } else {
          x0 = Vector(0.f, 1.f, 0.f);
        }
      }
      // Note that a Vector(1.f, 1e-4f, 1e-4f) is in fact normalized wrt to float32 precision.
      ASSERTX(abs(dot(enormal, x0)) < 3e-3f);
      const Vector dir0 = convert<float>(cross(convert<double>(enormal), convert<double>(x0)));
      float tmin = -TAU, tmax = TAU;
      for_int(j, enormals.num()) {
        if (i != j) intersect_param_circle_halfspace(x0, dir0, enormals[j], tmin, tmax);
      }
      if (tmin < tmax) {
        const float tmid = (tmin + tmax) / 2;
        center += x0 * std::cos(tmid) + dir0 * std::sin(tmid);
        n += 1;
      }
    }
    if (!center.normalize() && 1)
      assertnever("kernel_centroid_of_spherical_polygon fails: " +
                  SSHOW_PRECISE(enormals, center, n, _pmi._vertices.num()));
    if (dot(center, enormals[0]) < 0.f) center = -center;
    for_int(i, enormals.num()) ASSERTX(dot(enormals[i], center) >= -1e-6f);
    return center;
  }

  bool face_flipped(int f) const {
    const Vec3<int> vertices = _pmi.face_vertices(f);
    const Vec3<Point> triangle = map(vertices, [&](int v) { return _sphmap[v]; });
    if (0) return -signed_volume(triangle[0], triangle[1], triangle[2], Point(0.f, 0.f, 0.f)) < -1e-5f;  // Slower.
    const Vector center = mean(triangle);
    return dot(center, cross(triangle[1] - triangle[0], triangle[2] - triangle[0])) < -1e-5f;
  }

  bool any_adjacent_face_flipped(int v, int someface) const {
    for (const int f : _pmi.ccw_faces(v, someface))
      if (face_flipped(f)) return true;
    return false;
  }

  auto gather_1ring_external_edges(int v, int someface) const {
    PArray<Vector, 12> edge_normals;
    const auto normalized_cross = [](const Vector& vec1, const Vector& vec2) {
      return convert<float>(fast_normalized(cross(convert<double>(vec1), convert<double>(vec2))));
    };
    int f_last = -1;
    Vector v_first, v_last;
    dummy_init(v_first, v_last);
    for (const auto& [vv, ff] : _pmi.ccw_vertices(v, someface)) {
      const Vector vp = _sphmap[vv];
      if (f_last < 0) {
        v_first = vp;
      } else {
        edge_normals.push(normalized_cross(v_last, vp));
      }
      v_last = vp;
      f_last = ff;
    }
    if (_pmi._fnei[f_last].faces[mod3(_pmi.get_jvf(v, f_last) + 1)] == someface)  // Not on boundary.
      edge_normals.push(normalized_cross(v_last, v_first));
    return edge_normals;
  }

  struct OptimizerFace {
    Vec3<Vec3<Precision>> pd;  // On spherical domain; [0]==sph_value_being_optimized.
    Vec3<Vec3<Precision>> ps;  // On mesh surface;     [0]==center_vertex.
    bool is_hole;
  };

  auto get_ar_faces(int v, int someface) const {
    PArray<OptimizerFace, 12> ar_faces;
    for (const int ff : _pmi.ccw_faces(v, someface)) {
      const Vec3<int> vertices = _pmi.face_vertices(ff);
      const int ii = (vertices[1] == v) * 1 + (vertices[2] == v) * 2;
      ASSERTX(vertices[ii] == v);
      OptimizerFace optf;
      for_int(j, 3) {
        const int vv = vertices[mod3(ii + j)];
        // (pd[0] is a variable to be filled in during the optimization.)
        optf.pd[j] = convert<Precision>(_sphmap[vv]);
        optf.ps[j] = convert<Precision>(_pmi._vertices[vv].attrib.point);
      }
      const int matid = _pmi._faces[ff].attrib.matid;
      optf.is_hole = _matid_is_hole[matid];
      ar_faces.push(std::move(optf));
    }
    return ar_faces;
  }

  void get_optimization_dir(int v, int someface, float rand1, float rand2, Vector& dir, bool& on_boundary) const {
    on_boundary = false;
    const Vector x0 = _sphmap[v];
    if (int vl, vr; _options.respect_sharp_edges && vertex_is_on_sharp_edge(_pmi, v, someface, vl, vr)) {
      on_boundary = true;
      dir = _sphmap[vr] - _sphmap[vl];
      dir = normalized(dir - x0 * dot(x0, dir));
      assertx(dot(cross(_sphmap[vl], _sphmap[vr]), x0) < 1e-5f);
    } else if (_pmi.is_boundary(v, someface)) {
      on_boundary = true;
      const int fclw = _pmi.most_clw_face(v, someface), fccw = _pmi.most_ccw_face(v, someface);
      const int vclw = _pmi._wedges[_pmi._faces[fclw].wedges[mod3(_pmi.get_jvf(v, fclw) + 1)]].vertex;
      const int vccw = _pmi._wedges[_pmi._faces[fccw].wedges[mod3(_pmi.get_jvf(v, fccw) + 2)]].vertex;
      dir = _sphmap[vccw] - _sphmap[vclw];
      dir = normalized(dir - x0 * dot(x0, dir));
    } else {
      // Pick random direction (tangent to sphere at x0) to do line search.
      dir = get_random_unit_vector(rand1, rand2);
      dir -= x0 * dot(x0, dir);
      if (!dir.normalize()) dir = normalized(Vector(x0[1], x0[2], x0[0]));  // Extremely rare case.
    }
  }

  Point optimize_vertex_rand(int v, int someface, float rand1, float rand2) const {
    Vector x0 = _sphmap[v];
    if (v < _num_fixed_vertices) return x0;  // Hold the base vertices constant.
    auto ar_faces = get_ar_faces(v, someface);
    const auto edge_normals = gather_1ring_external_edges(v, someface);
    Vector dir;
    bool on_boundary;
    get_optimization_dir(v, someface, rand1, rand2, dir, on_boundary);

    const auto stretch_on_neighborhood = [&](float t) {
      const Point pd0 = interp_on_arc(x0, dir, t);
      double total_stretch = 0.;
      for (auto& optimizer_face : ar_faces) {
        optimizer_face.pd[0] = convert<Precision>(pd0);
        Precision face_stretch = accurate_stretch_for_spherical_triangle(optimizer_face.pd, optimizer_face.ps);
        if (optimizer_face.is_hole) face_stretch *= _options.hole_weight;
        total_stretch += face_stretch;
      }
      return float(total_stretch);
    };

    for_int(dir_iter, on_boundary ? 1 : 2) {
      // Find valid search interval (intersect with neighborhood kernel).
      float tmin = -TAU, tmax = TAU;
      for_int(i, edge_normals.num()) intersect_param_circle_halfspace(x0, dir, edge_normals[i], tmin, tmax);
      if (!assertw(tmin <= tmax)) break;
      if (1) {  // Keep some distance away from kernel boundary to prevent completely collapsing any triangle.
        const float k_pad_fraction = .01f;
        const float extent = tmax - tmin;
        tmin += extent * k_pad_fraction;
        tmax -= extent * k_pad_fraction;
      }
      if (_options.verbose >= 2) assertw(tmin <= 0.f && tmax >= 0.f);

      const int bits = 12;  // Should not exceed half the number of bits in the mantissa of T (thus, 12 for float).
      std::uintmax_t max_iter = _optim_line_search_iter;
      const auto [tbest, fmin] = boost_minima::brent_find_minima(stretch_on_neighborhood, tmin, tmax, bits, max_iter);
      // const int num_iterations = int(_optim_line_search_iter - max_iter);

      if (fmin > stretch_on_neighborhood(0.f)) {
        if (0) Warning("Objective function increased");  // Due to k_pad_fraction or !(tmin <= 0.f && tmax >= 0.f).
      } else {
        x0 = normalized(interp_on_arc(x0, dir, tbest));
        ASSERTX(!any_adjacent_face_flipped(v, someface));
      }
      dir = normalized(cross(x0, dir));  // Now try the perpendicular direction.
    }
    return x0;
  }

  Point optimize_vertex(int v, int someface) const {
    const float rand1 = Random::G.unif(), rand2 = Random::G.unif();
    return optimize_vertex_rand(v, someface, rand1, rand2);
  }

  void optimize_vertex_split(int v, int someface) {
    HH_STIMER("_optimize_vertex_split");
    if (int vl, vr; _options.respect_sharp_edges && vertex_is_on_sharp_edge(_pmi, v, someface, vl, vr)) {
      Vector sp = normalized(_sphmap[vl] + _sphmap[vr]);
      if (0) SHOW(_sphmap[vl], _sphmap[vr], sp);
      _sphmap[v] = sp;
    } else if (_pmi.is_boundary(v, someface)) {
      // Find neighboring boundary vertices and average them.
      const int fclw = _pmi.most_clw_face(v, someface), fccw = _pmi.most_ccw_face(v, someface);
      const int vclw = _pmi._wedges[_pmi._faces[fclw].wedges[mod3(_pmi.get_jvf(v, fclw) + 1)]].vertex;
      const int vccw = _pmi._wedges[_pmi._faces[fccw].wedges[mod3(_pmi.get_jvf(v, fccw) + 2)]].vertex;
      const Vector sp = normalized(_sphmap[vclw] + _sphmap[vccw]);
      _sphmap[v] = sp;
    } else {
      const auto edge_normals = gather_1ring_external_edges(v, someface);
      _sphmap[v] = kernel_centroid_of_spherical_polygon(edge_normals);
      if (1) assertx(is_unit(_sphmap[v]));
    }
    ASSERTX(!any_adjacent_face_flipped(v, someface));

    set_stretch_scaling();
    for_int(i, _optim_vsplit_vt_iter) _sphmap[v] = optimize_vertex(v, someface);

    for_int(i, _optim_vsplit_nei_iter) {
      for (const auto& [vv, ff] : _pmi.ccw_vertices(v, someface)) _sphmap[vv] = optimize_vertex(vv, ff);
      _sphmap[v] = optimize_vertex(v, someface);
    }
  }

  bool apply_vsplit() {
    const Vsplit* vspl = _pmi.rstream().peek_next_vsplit();
    if (!vspl) return false;
    int vs;
    {
      // (This code also works for the rare case when vspl->vlr_offset1 == 0.)
      const int flclw = vspl->flclw;
      const int vs_index = (vspl->code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
      const int wlclw = _pmi._faces[flclw].wedges[vs_index];
      vs = _pmi._wedges[wlclw].vertex;
      for (const int f : _pmi.ccw_faces(vs, flclw)) _surface_area -= sqrt(area2(_pmi.face_points(f)));
    }
    assertx(_pmi.next());  // Perform the vertex split.
    const int vt = _pmi._vertices.num() - 1;
    const int fnew = _pmi._faces.num() - 1;
    const bool isr = vspl->vlr_offset1 > 1;
    const int fl = _pmi._faces.num() - 1 - isr, fr = isr ? _pmi._faces.num() - 1 : -1;
    const int vl = _pmi._wedges[_pmi._faces[fl].wedges[2]].vertex;
    const int vr = fr >= 0 ? _pmi._wedges[_pmi._faces[fr].wedges[1]].vertex : -1;
    if (_options.flatten_to_x0) _pmi._vertices[vt].attrib.point[0] = 0.f;
    for (const int v : {vs, vt})
      for (const int f : _pmi.ccw_faces(v, fnew))
        if (!(v == vt && (f == fl || f == fr)))  // Avoid double-counting the shared faces fl and/or fr.
          _surface_area += sqrt(area2(_pmi.face_points(f)));
    optimize_vertex_split(vt, fnew);
    update_visualizer_vsplit(vs, vt, vl, vr, fnew);
    return true;
  }

  static constexpr float k_spatial_fade = 0.3f;

  void optimize_all() {
    HH_STIMER("_optimize_all");
    set_stretch_scaling();
    const int nv = _pmi._vertices.num();
    if (_options.verbose >= 2) std::cerr << sform("nv=%d", nv) << std::flush;
    const Array<int> someface = _pmi.gather_someface();
    Array<Point> ar_sph(nv);
    Array<float> ar_displacement(nv);
    Array<int> ar_vertex(range(nv));
    Matrix<float> ar_random(nv, 2);  // For reproducible results under parallelism.
    const auto by_decreasing_displacement = [&](int v1, int v2) { return ar_displacement[v1] > ar_displacement[v2]; };

    for_int(iter, _optim_global_iter) {
      fill(ar_displacement, 3.f);  // Some value larger than the unit sphere diameter.
      int num_active = nv;
      for (int update_iter = 0;; update_iter++) {
        if (!assertw(update_iter < 2000) || !num_active) break;

        ArrayView<int> active_vertices = ar_vertex.head(num_active);
        for (const int v : active_vertices) for_int(c, 2) ar_random[v][c] = Random::G.unif();

        parallel_for_each({10'000}, range(num_active), [&](int i) {
          const int v = active_vertices[i];
          ar_sph[v] = optimize_vertex_rand(v, someface[v], ar_random[v][0], ar_random[v][1]);
          ar_displacement[v] += dist(ar_sph[v], _sphmap[v]);
        });

        sort(active_vertices, by_decreasing_displacement);

        Array<bool> locked(nv, false);
        for (const int v : active_vertices) {
          if (locked[v]) continue;
          assertx(ar_displacement[v] > _optim_movetol);
          const float displacement = dist(ar_sph[v], _sphmap[v]);
          _sphmap[v] = ar_sph[v];
          ar_displacement[v] = 0.f;
          for (const auto& [vv, unused_ff] : _pmi.ccw_vertices(v, someface[v])) {
            locked[vv] = true;
            ar_displacement[vv] += displacement * k_spatial_fade;
          }
        }
        if (_options.verbose >= 2 && update_iter % 10 == 0) std::cerr << "." << std::flush;

        sort(ar_vertex, by_decreasing_displacement);
        num_active = 0;
        while (num_active < nv && ar_displacement[ar_vertex[num_active]] > _optim_movetol) num_active++;
      }
      if (_options.verbose >= 2) std::cerr << "|" << std::flush;
      update_visualizer_optimize_all();
    }
  }

  void goto_nverts(int n) {
    n = min(n, _pmi.rstream()._info._full_nvertices);
    assertx(_pmi._vertices.num() <= n);
    if (_pmi._vertices.num() == n) return;
    optimize_all();

    const int n0 = _pmi._vertices.num();
    int nv_last_optimize_all = n0;
    ConsoleProgress cprogress;
    for (;;) {
      if (_pmi._vertices.num() == n) break;
      if (!apply_vsplit()) break;
      const int nv = _pmi._vertices.num();
      cprogress.update(float(nv - n0) / (n - n0));
      const int nv_next_optimize_all = int(nv_last_optimize_all * _optim_nv_ratio + 0.5f);
      bool want_optimize_all = nv >= nv_next_optimize_all;
      if (want_optimize_all) {
        if (_options.verbose >= 2) cprogress.clear();
        optimize_all();
        nv_last_optimize_all = nv;
      }
    }
    if (_options.verbose >= 2) cprogress.clear();
    if (_options.verbose >= 2) std::cerr << "finest_" << std::flush;

    optimize_all();
    if (_options.verbose >= 2) std::cerr << "\n" << std::flush;
    for_int(f, _pmi._faces.num()) assertx(!face_flipped(f));
  }

  void set_stretch_scaling() {
    // Surface area is introduced here is to induce a scale-invariant weighting of the conformal_weight term.
    _stretch_scaling1 = Precision(.5 / _surface_area * (2 * D_TAU) * .5);
    _stretch_scaling2 = Precision(_conformal_weight * std::pow(1.f / (2 * D_TAU) * _surface_area, 3.) * .5);
    _stretch_scaling3 = Precision(.5 / (2 * D_TAU) * _surface_area);
  }

  // Measure the squared stretch from the spherical domain "d" to the surface mesh "s".
  Precision naive_stretch(const Vec3<Precision>& pd0, const Vec3<Precision>& pd1, const Vec3<Precision>& pd2,
                          const Vec3<Precision>& ps0, const Vec3<Precision>& ps1, const Vec3<Precision>& ps2) const {
    // Determine a local frame on the spherical domain triangle.
    const Vec3<Precision> d_e01 = pd1 - pd0, d_e02 = pd2 - pd0;
    // Compute triangle coordinates in a canonical isometric frame: (0, 0), (d_x1, 0), (d_x2, d_y2).
    const Precision d_x1 = mag(d_e01), recip_d_x1 = 1.f / (d_x1 + 1e-20f);

    const Precision d_x2 = dot(d_e02, d_e01) * recip_d_x1;
    const Precision d_y2 = sqrt(mag2(d_e02) - square(d_x2));

    const Vec3<Precision> s_e01 = ps1 - ps0, s_e02 = ps2 - ps0;
    const Vec3<Precision> dfds = s_e01 * recip_d_x1;
    const Vec3<Precision> dfdt = (s_e01 * (-d_x2 * recip_d_x1) + s_e02) / d_y2;
    const Precision a = mag2(dfds);
    const Precision c = mag2(dfdt);
    // const Precision b = dot(dfds, dfdt);
    const Precision s_area = mag(cross(s_e01, s_e02));  // * .5f;

    // The eigenvalues of the metric tensor M are [Sander et al. Texture Mapping Progressive Meshes. 2001]:
    //  gamma2 = ( (a + c) \pm sqrt(square(a - c) + 4 * b * b) ) / 2.
    //  L2 stretch^2 = mean(gamma2) = (a + c) / 2.
    // BTW, the eigenvalues of M^-1 are [ChatGPT]:
    //  gamma2 = ( (a + c) \pm sqrt(square(a - c) + 4 * b * b) ) / (2 * det), with det = a * c - b * b.
    //  L2 stretch^2 = mean(gamma2) = (a + c ) / (2 * det).

    Precision stretch = (a + c) * _stretch_scaling1;  // * .5f / _surface_area * (2 * D_TAU).
    if (1 || _conformal_weight) {
      // Anti-cruft : power 6 of the inverse L^infinity stretch.
      // Formula for \Gamma^2 on M^-1, as described in SIGGRAPH 2003 paper:
      // const Precision inv_gamma2 = ((a + c) + sqrt(square(a - c) + 4 * b * b)) / (2 * (a * c - b * b));
      // Simpler approximation that behaves even better:
      const Precision inv_gamma2 = 1.f / min(a, c);
      stretch += inv_gamma2 * inv_gamma2 * inv_gamma2 * _stretch_scaling2;
    }
    return stretch * s_area;
  }

  // Measure the squared stretch from the surface mesh "s" to the spherical domain "d".
  Precision naive_inverse_stretch(const Vec3<Precision>& pd0, const Vec3<Precision>& pd1, const Vec3<Precision>& pd2,
                                  const Vec3<Precision>& ps0, const Vec3<Precision>& ps1,
                                  const Vec3<Precision>& ps2) const {
    // Determine a local frame on the surface mesh triangle.
    const Vec3<Precision> s_e01 = ps1 - ps0, s_e02 = ps2 - ps0;
    const Precision s_x1 = mag(s_e01), recip_s_x1 = 1.f / (s_x1 + 1e-20f);

    const Precision s_x2 = dot(s_e02, s_e01) * recip_s_x1;
    const Precision s_y2 = sqrt(mag2(s_e02) - square(s_x2));

    const Vec3<Precision> d_e01 = pd1 - pd0, d_e02 = pd2 - pd0;
    const Vec3<Precision> dfds = d_e01 * recip_s_x1;
    const Vec3<Precision> dfdt = (d_e01 * (-s_x2 * recip_s_x1) + d_e02) / s_y2;
    const Precision a = mag2(dfds);
    const Precision c = mag2(dfdt);
    // const Precision b = dot(dfds, dfdt);

    const Precision stretch = (a + c) * _stretch_scaling3;  // * .5f / (2 * D_TAU) * _surface_area.
    const Precision d_area = spherical_triangle_area(pd0, pd1, pd2);
    return stretch * d_area;
  }

  // Measure the squared stretch from the spherical domain "d" to the surface mesh "s".
  Precision accurate_stretch_for_spherical_triangle(const Vec3<Vec3<Precision>>& pd,
                                                    const Vec3<Vec3<Precision>>& ps) const {
    if (0) return naive_stretch(pd[0], pd[1], pd[2], ps[0], ps[1], ps[2]);  // "-no_perp_split" -> accordion effect.
    // Split the triangle to avoid the "accordion" effect (sliver spherical triangles not being penalized
    // because the area of the corresponding subtended linear triangle does not go to zero as it rotates).
    const Vec3<Precision> elen(mag2(pd[1] - pd[2]), mag2(pd[2] - pd[0]), mag2(pd[0] - pd[1]));
    // Index of vertex opposite to the longest spherical triangle arc.
    const int i = elen[0] > elen[1] ? (elen[0] > elen[2] ? 0 : 2) : (elen[1] > elen[2] ? 1 : 2);
    const int j = mod3(i + 1), k = mod3(i + 2);

    const Vec3<Precision> nor = fast_normalized(cross(pd[j], pd[k]));
    const Vec3<Precision> perp0 = pd[i] - nor * dot(nor, pd[i]);  // Projection of i on plane containing 0, j, k.
    const Vec3<Precision> perp = fast_normalized(perp0);          // Point on arc(j, k) closest to i.
    // (This could fail in the rare case that the spherical triangle is large (e.g. larger than an octant).

    // Matches https://chat.openai.com/share/98d692e1-b9dc-4c0c-9116-cf11811d66b4.
    const Vec3<Precision> dq = pd[j] - pd[k];
    const Precision dq2 = elen[i];  // == mag2(dq).
    const Precision dpdq = dot(perp, dq);
    const Precision tkj = (dpdq * dot(perp, pd[k]) - dot(dq, pd[k])) / (dq2 - square(dpdq));  // Not arc-length.
    const Vec3<Precision>& perp_d = perp;
    const Vec3<Precision> perp_s = ps[j] * tkj + ps[k] * (1.f - tkj);  // interp(ps[j], ps[k], float(tkj));
    Precision total_stretch;
    if (_options.optimize_inverse)
      total_stretch = (naive_inverse_stretch(perp_d, pd[i], pd[j], perp_s, ps[i], ps[j]) +
                       naive_inverse_stretch(perp_d, pd[k], pd[i], perp_s, ps[k], ps[i]));
    else
      total_stretch = (naive_stretch(perp_d, pd[i], pd[j], perp_s, ps[i], ps[j]) +
                       naive_stretch(perp_d, pd[k], pd[i], perp_s, ps[k], ps[i]));
    // Surprisingly, the following line does not result in any compiled code under clang.
    if (!std::isfinite(total_stretch)) total_stretch = std::numeric_limits<Precision>::infinity();
    return total_stretch;
  }

  Precision stretch_for_face(int f) const {
    const Vec3<int> vertices = _pmi.face_vertices(f);
    const auto pd = map(vertices, [&](int v) { return convert<Precision>(_sphmap[v]); });
    const auto ps = map(_pmi.face_points(f), [&](const Point& p) { return convert<Precision>(p); });
    return accurate_stretch_for_spherical_triangle(pd, ps);
  }

  // *** Visualizer.

  static constexpr bool wait_for_user_to_close_visualizer_window = false;
  WFile* _visualizer{nullptr};  // (Never deleted if !wait_for_user_to_close_visualizer_window.)
  int _visualizer_nsplits_since_end_frame{0};

  void create_visualizer() {
    my_setenv("G3D_TWOLIGHTS", "1");  // Exported environment variable; enables two-sided lighting in G3dOGL.
    string filename = "| G3dOGL -geom 1500x1500 -terse -st none -key CDe---";
    if (0) filename = "debug_output.txt";
    _visualizer = new WFile(filename);
  }

  void initialize_visualizer() {
    if (!_visualizer) return;
    std::ostream& os = (*_visualizer)();
    // Viewer frame from unitsphere.s3d
    os << "F 0  0.577347 0.577382 -0.577363  -0.707142 0.707087 0  0.408249 0.408274 0.816517"
          "  -19.0715 -19.0767 19.0628  0.0319575\n";
    GMesh gmesh = _pmi.extract_gmesh();
    for (Vertex gv : gmesh.vertices()) gmesh.set_point(gv, _sphmap[gmesh.vertex_id(gv) - 1]);
    for (Face gf : gmesh.faces())
      for (Corner c : gmesh.corners(gf)) gmesh.set_string(c, nullptr);  // Remove normals.
    const Array<int> someface = _pmi.gather_someface();
    string str;
    for_int(v, _pmi._vertices.num()) {
      const Vector normal = average_normal_for_vertex(_pmi, v, someface[v]);
      gmesh.update_string(gmesh.id_vertex(v + 1), "normal", csform_vec(str, normal));
    }
    gmesh.write(os);
  }

  void update_visualizer_vertex_position(std::ostream& os, int v, const char* sinfo) const {
    const Point& sph = _sphmap[v];
    os << sform("MVertex %d %g %g %g%s\n", v + 1, sph[0], sph[1], sph[2], sinfo);
  }

  void update_visualizer_vsplit(int vs, int vt, int vl, int vr, int fnew) {
    if (!_visualizer) return;
    std::ostream& os = (*_visualizer)();
    assertx(vs >= 0 && vt >= 0 && vl >= 0 && vr >= -1 && fnew >= 0);
    os << "Vspl " << vs + 1 << " " << vl + 1 << " " << vr + 1 << " " << vt + 1 << "\n";
    os << "Face 0  " << vs + 1 << " " << vt + 1 << " " << vl + 1 << "\n";
    if (vr >= 0) os << "Face 0  " << vs + 1 << " " << vr + 1 << " " << vt + 1 << "\n";
    string str;
    const Vector normal = average_normal_for_vertex(_pmi, vt, fnew);
    const string sinfo = string(" {normal=") + csform_vec(str, normal) + "}";
    update_visualizer_vertex_position(os, vt, sinfo.c_str());
    for (const auto& [vv, unused_ff] : _pmi.ccw_vertices(vt, fnew)) update_visualizer_vertex_position(os, vv, "");
    const int nf = _pmi._faces.num();
    const int nsplits_threshold = max(1, int((nf - 1000) * .03f));
    if (++_visualizer_nsplits_since_end_frame >= nsplits_threshold) {
      // Some alternative speedups as the mesh becomes fine would be to:
      // - Press "De" in the viewer to turn off the edges.
      // - Press "u" in the viewer to turn off rendering altogether.
      os << "f 0 0 0\n" << std::flush;
      _visualizer_nsplits_since_end_frame = 0;
    }
  }

  void update_visualizer_optimize_all() {
    if (!_visualizer) return;
    std::ostream& os = (*_visualizer)();
    for_int(v, _pmi._vertices.num()) update_visualizer_vertex_position(os, v, "");
    os << "f 0 0 0\n" << std::flush;
    _visualizer_nsplits_since_end_frame = 0;
  }

  void finalize_visualizer() {
    if (!_visualizer) return;
    std::ostream& os = (*_visualizer)();
    os << "f 0 0 0\n" << std::flush;
    os << "keys DeDC\n";           // Turn off edges; turn on display-list caching.
    os << "f 0 0 0\nkeys DCDC\n";  // Hack to get display-list caching to reset properly.
    os << "keys J\n";              // Start rotating.
    os << std::flush;
    if (wait_for_user_to_close_visualizer_window) {
      delete _visualizer;
      _visualizer = nullptr;
    }
  }
};

// ***

SphereMapper::SphereMapper(PMeshIter& pmi, Options options)
    : _impl(make_unique<SphereMapper::Implementation>(pmi, options)) {}

SphereMapper::~SphereMapper() {}

void SphereMapper::SphereMapper::show_parameters() const { _impl->show_parameters(); }

CArrayView<Point> SphereMapper::compute(CArrayView<Point> base_sphmap) { return _impl->compute(base_sphmap); }

void SphereMapper::show_total_stretch() { _impl->show_total_stretch(); }

Frame SphereMapper::frame_aligning_sphmap_to_surface_normals() const {
  return _impl->frame_aligning_sphmap_to_surface_normals();
}

}  // namespace hh
