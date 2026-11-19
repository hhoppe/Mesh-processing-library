// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#include "libHh/MeshOp.h"

#include <mutex>  // once_flag, call_once()

#include "libHh/Array.h"
#include "libHh/Facedistance.h"
#include "libHh/GeomOp.h"
#include "libHh/Lls.h"
#include "libHh/MathOp.h"  // Trig
#include "libHh/Polygon.h"
#include "libHh/PriorityQueue.h"
#include "libHh/RangeOp.h"  // sort()
#include "libHh/Set.h"
#include "libHh/Stack.h"

namespace hh {

// *** helper

namespace {

int retriangulate(GMesh& mesh, Set<Edge>& sete, bool recurse, Set<Vertex>* setvr, float mincos, EDGEF fdoswap,
                  EDGEF fdel, EDGEF fadd) {
  assertx(fdoswap);
  int neswapped = 0;
  while (!sete.empty()) {
    Edge e = sete.remove_one();
    assertx(!mesh.is_boundary(e));
    const Point& p1 = mesh.point(mesh.vertex1(e));
    const Point& p2 = mesh.point(mesh.vertex2(e));
    const Point& po1 = mesh.point(mesh.side_vertex1(e));
    const Point& po2 = mesh.point(mesh.side_vertex2(e));
    if (dihedral_angle_cos(p1, p2, po1, po2) < mincos) continue;
    if (dihedral_angle_cos(po1, po2, p2, p1) < mincos) continue;
    if (!mesh.legal_edge_swap(e)) continue;
    if (!fdoswap(mesh, e)) continue;
    if (fdel) fdel(mesh, e);
    for (Face f : mesh.faces(e))
      for (Edge ee : mesh.edges(f)) sete.remove(ee);
    Edge ne = mesh.swap_edge(e);
    // e = nullptr;  // Now undefined.
    neswapped++;
    if (fadd) fadd(mesh, ne);
    if (!recurse) continue;
    for (Face f : mesh.faces(ne)) {
      for (Edge ee : mesh.edges(f)) {
        if (ee == ne || mesh.is_boundary(ee) ||
            (setvr && (!setvr->contains(mesh.side_vertex1(ee)) || !setvr->contains(mesh.side_vertex2(ee)))))
          continue;
        sete.add(ee);
      }
    }
  }
  return neswapped;
}

const FlagMask fflag_visited = Mesh::allocate_Face_flag();

}  // namespace

// *** Misc

Queue<Edge> gather_boundary(const Mesh& mesh, Edge e) {
  Queue<Edge> queuee;
  Edge ef = e;
  for (;;) {
    queuee.enqueue(e);
    e = mesh.ccw_boundary(e);
    if (e == ef) break;
  }
  return queuee;
}

Set<Face> gather_component(const Mesh& mesh, Face f) {
  Set<Face> setf;
  Queue<Face> queue;
  setf.enter(f);
  for (;;) {
    for (Face f2 : mesh.faces(f))
      if (setf.add(f2)) queue.enqueue(f2);
    if (queue.empty()) break;
    f = queue.dequeue();
  }
  return setf;
}

Set<Face> gather_component_v(const Mesh& mesh, Face f) {
  Set<Face> setf;
  Queue<Face> queue;
  setf.enter(f);
  for (;;) {
    for (Vertex v : mesh.vertices(f))
      for (Face f2 : mesh.faces(v))
        if (setf.add(f2)) queue.enqueue(f2);
    if (queue.empty()) break;
    f = queue.dequeue();
  }
  return setf;
}

Array<Set<Face>> gather_components(const Mesh& mesh) {
  Array<Set<Face>> components;
  Set<Face> setfvis;  // Faces already considered.
  for (Face f : mesh.faces()) {
    if (setfvis.contains(f)) continue;
    components.push(gather_component(mesh, f));
    const Set<Face>& setf = components.last();
    if (setf.num() == mesh.num_faces()) return components;  // Shortcut for single component.
    for (Face ff : setf) setfvis.enter(ff);
  }
  const auto by_increasing_size = [](const Set<Face>& a, const Set<Face>& b) { return a.num() < b.num(); };
  sort(components, by_increasing_size);
  return components;
}

Stat mesh_stat_boundaries(const Mesh& mesh) {
  Stat Sbound;
  Set<Edge> setevis;  // Boundary edges already considered.
  for (Edge e : mesh.edges()) {
    if (!mesh.is_boundary(e)) continue;
    if (setevis.contains(e)) continue;
    const Queue<Edge> queuee = gather_boundary(mesh, e);
    for (Edge ee : queuee) setevis.enter(ee);
    Sbound.enter(queuee.length());
  }
  return Sbound;
}

Stat mesh_stat_components(const Mesh& mesh) {
  Stat Scompf;
  if (0) {
    Set<Face> setfvis;  // Faces already considered.
    for (Face f : mesh.faces()) {
      if (setfvis.contains(f)) continue;
      const Set<Face> setf = gather_component(mesh, f);
      for (Face ff : setf) setfvis.enter(ff);
      Scompf.enter(setf.num());
    }
  } else {  // Faster.
    Mesh& mesh2 = const_cast<Mesh&>(mesh);
    for (Face f : mesh.faces()) mesh2.flags(f).flag(fflag_visited) = false;
    for (Face f : mesh.faces()) {
      if (!mesh2.flags(f).flag(fflag_visited).set(true)) {
        int num_faces = 0;
        Stack<Face> stack;
        stack.push(f);
        while (!stack.empty()) {
          f = stack.pop();
          num_faces++;
          for (Face f2 : mesh.faces(f))
            if (!mesh2.flags(f2).flag(fflag_visited).set(true)) stack.push(f2);
        }
        Scompf.enter(num_faces);
      }
    }
  }
  return Scompf;
}

float mesh_genus(const Mesh& mesh) {
  const int nv = mesh.num_vertices();
  const int nf = mesh.num_faces();
  const int ne = mesh.num_edges();
  // Notes:
  //  For a mesh without boundary, nf = nv * 2 + (genus - 1) * 4.
  //  For a mesh without boundary, ec = 2 - 2 * genus.
  const Stat Sbound = mesh_stat_boundaries(mesh);
  const int nb = Sbound.inum();
  const Stat Scompf = mesh_stat_components(mesh);
  const int nc = Scompf.inum();
  const int ec = nv - ne + nf;  // Euler characteristic.
  const float genus = (nc * 2 - ec - nb) / 2.f;
  return genus;
}

string mesh_genus_string(const Mesh& mesh) {
  const int nv = mesh.num_vertices();
  const int nf = mesh.num_faces();
  const int ne = mesh.num_edges();
  // Notes:
  //  For a mesh without boundary, nf = nv * 2 + (genus - 1) * 4.
  //  For a mesh without boundary, ec = 2 - 2 * genus.
  const Stat Sbound = mesh_stat_boundaries(mesh);
  const int nb = Sbound.inum();
  const Stat Scompf = mesh_stat_components(mesh);
  const int nc = Scompf.inum();
  const int ec = nv - ne + nf;  // Euler characteristic.
  const float genus = (nc * 2 - ec - nb) / 2.f;
  int nse = 0, ncv = 0;
  for (Edge e : mesh.edges())
    if (mesh.flags(e).flag(GMesh::eflag_sharp)) nse++;
  for (Vertex v : mesh.vertices())
    if (mesh.flags(v).flag(GMesh::vflag_cusp)) ncv++;
  return sform("Genus: c=%d b=%d  v=%d f=%d e=%d  genus=%g%s",  //
               nc, nb, nv, nf, ne, genus, (nse + ncv) ? sform("  sharpe=%d cuspv=%d", nse, ncv).c_str() : "");
}

bool triangulate_face(GMesh& mesh, Face f) {
  Array<Vertex> va;
  mesh.get_vertices(f, va);
  const int nv = va.num();
  if (!assertw(nv > 3)) return true;
  for_intL(i, 2, nv - 1) {
    if (!assertw(!mesh.query_edge(va[0], va[i]))) return false;
  }
  mesh.destroy_face(f);
  for_int(i, nv - 2) mesh.create_face(va[0], va[i + 1], va[i + 2]);
  Set<Vertex> setvr;  // Vertices on the ring of the original face.
  for_int(i, nv) setvr.enter(va[i]);
  Set<Edge> sete;  // Initially, the inner edges.
  for_intL(i, 2, nv - 1) sete.enter(mesh.edge(va[0], va[i]));
  retriangulate(mesh, sete, true, &setvr, -2, circum_radius_swap_criterion, nullptr, nullptr);
  return true;
}

// *** fair_triangulate_face

namespace {

// The angle between two unit normals, or pi (the worst) if one of them is zero (a degenerate triangle).
float dihedral_angle_between(const Vector& n1, const Vector& n2) {
  if (is_zero(n1) || is_zero(n2)) return float(D_TAU / 2);
  return std::acos(clamp(dot(n1, n2), -1.f, 1.f));
}

}  // namespace

FairTriangulateResult fair_triangulate_face(GMesh& mesh, Face f, const FairTriangulateOptions& options) {
  assertx(options.fairing_order >= 0 && options.fairing_order <= 2 && options.density_factor > 0.f);
  FairTriangulateResult result;
  Array<Vertex> va;
  mesh.get_vertices(f, va);
  const int n = va.num();
  if (!assertw(n >= 3)) return result;
  const Array<Point> pa = transformed(va, [&](Vertex v) { return mesh.point(v); });
  // The strings of the face and of its corners, given to the new faces and their corners at the loop vertices.
  const string fstring = mesh.get_string(f) ? mesh.get_string(f) : "";
  // Whether the loop vertices all have normals (in their corners of f), so that the new vertices should have them.
  const bool loop_has_normals = ranges::all_of(mesh.corners(f), [&](Corner c) {
    Vector normal;
    return mesh.parse_corner_key_vec(c, "normal", normal);
  });
  Map<Vertex, string> corner_strings;
  for (Corner c : mesh.corners(f))
    if (mesh.get_string(c)) corner_strings.enter(mesh.corner_vertex(c), mesh.get_string(c));
  // The unit normal of the mesh face beyond each loop edge (va[i], va[i + 1]), or zero if none.
  Array<Vector> outer_normals(n, Vector(0.f, 0.f, 0.f));
  for_int(i, n) {
    if (const Face fo = mesh.opp_face(f, mesh.edge(va[i], va[(i + 1) % n]))) {
      Polygon poly;
      mesh.polygon(fo, poly);
      outer_normals[i] = ok_normalized(poly.get_normal_dir());
    }
  }
  // The density at each loop vertex: the mean length of its edges.
  Map<Vertex, float> density;
  for (Vertex v : va) {
    float sum = 0.f;
    int num = 0;
    for (Edge e : mesh.edges(v)) sum += mesh.length(e), num++;
    density.enter(v, sum / float(max(num, 1)));
  }

  // (1) Triangulate the loop.
  Array<Vec3<int>> triangles;  // Indices into va.
  if (n <= options.max_dp_vertices) {
    // For the sub-loop va[i..k] (closed by the edge (i, k)), the weight of its best triangulation (largest dihedral
    // angle, then area), the apex m of its triangle on edge (i, k), and the unit normal of that triangle.
    const float k_invalid = BIGFLOAT;
    Matrix<float> max_angle(V(n, n), 0.f);
    Matrix<double> area(V(n, n), 0.);
    Matrix<int> apex(V(n, n), -1);
    Matrix<Vector> normal(V(n, n), Vector(0.f, 0.f, 0.f));
    const auto across_normal = [&](int i, int k) { return k == i + 1 ? outer_normals[i] : normal[i, k]; };
    for_intL(len, 2, n) {
      for (int i = 0; i + len < n; i++) {
        const int k = i + len;
        float best_angle = k_invalid;
        double best_area = 0.;
        // A diagonal that is already a mesh edge would make the mesh non-manifold.
        if (!(i == 0 && k == n - 1) && mesh.query_edge(va[i], va[k])) {
          max_angle[i, k] = k_invalid;
          continue;
        }
        for_intL(m, i + 1, k) {
          if (max_angle[i, m] == k_invalid || max_angle[m, k] == k_invalid) continue;
          const Vector dir = cross(pa[i], pa[m], pa[k]);
          const Vector nor = ok_normalized(dir);
          float angle = max(max_angle[i, m], max_angle[m, k]);
          // A (nearly) degenerate triangle, whose height is tiny relative to its longest edge, is worse than any fold.
          const float max_len2 = max({dist2(pa[i], pa[m]), dist2(pa[m], pa[k]), dist2(pa[k], pa[i])});
          if (mag(dir) <= 1e-3f * max_len2) angle = max(angle, float(D_TAU / 2) + 1.f);
          for (const Vector& n2 : {across_normal(i, m), across_normal(m, k)})
            if (!is_zero(n2)) angle = max(angle, dihedral_angle_between(nor, n2));
          if (i == 0 && k == n - 1 && !is_zero(outer_normals[n - 1]))
            angle = max(angle, dihedral_angle_between(nor, outer_normals[n - 1]));
          const double sum_area = area[i, m] + area[m, k] + .5 * mag(dir);
          if (angle < best_angle || (angle == best_angle && sum_area < best_area)) {
            best_angle = angle, best_area = sum_area;
            apex[i, k] = m;
            normal[i, k] = nor;
          }
        }
        max_angle[i, k] = best_angle;
        area[i, k] = best_area;
      }
    }
    // This fails only if every triangulation of the loop would create an edge already in the mesh.  (It has not
    // happened in practice.)  Splitting the loop along some diagonal (e.g., the shortest absent "halfway" diagonal)
    // and recursing would fail in the same cases, since it also yields a triangulation of the loop; a fallback that
    // always works would instead start from the fan (as for a large loop below), then refine and fair it.
    if (max_angle[0, n - 1] == k_invalid) return result;
    Array<Vec2<int>> stack{V(0, n - 1)};
    while (!stack.empty()) {
      const auto [i, k] = stack.pop();
      if (k - i < 2) continue;
      const int m = apex[i, k];
      triangles.push(V(i, m, k));
      stack.push(V(i, m));
      stack.push(V(m, k));
    }
  }
  // Give the corners of a new face at the loop vertices the strings of the corners of f (e.g., uv), before any
  // center split interpolates them.
  const auto set_corner_strings = [&](Face ff) {
    for (Corner c : mesh.corners(ff))
      if (const string* ps = corner_strings.find_ptr(mesh.corner_vertex(c))) mesh.set_string(c, ps->c_str());
  };
  Set<Face> patch;
  // Swap the edges of the patch faces whose two opposite angles sum to more than pi (as for a Delaunay
  // triangulation), where legal, until none remains.
  const auto relax_edges = [&] {
    const auto angle_at = [&](Vertex v, Vertex w1, Vertex w2) {
      return angle_between_unit_vectors(ok_normalized(mesh.point(w1) - mesh.point(v)),
                                        ok_normalized(mesh.point(w2) - mesh.point(v)));
    };
    for_int(iter, 100) {
      // The interior edges of the patch, as vertex pairs because a swap destroys edges.
      Array<Vec2<Vertex>> edges;
      for (Face fp : patch)
        for (Edge e : mesh.edges(fp))
          if (!mesh.is_boundary(e) && fp == mesh.face1(e) && patch.contains(mesh.face2(e)))
            edges.push(V(mesh.vertex1(e), mesh.vertex2(e)));
      int num_swaps = 0;
      for (const auto& [v1, v2] : edges) {
        const Edge e = mesh.query_edge(v1, v2);
        if (!e || !mesh.legal_edge_swap(e)) continue;
        const Vertex vs1 = mesh.side_vertex1(e), vs2 = mesh.side_vertex2(e);
        if (angle_at(vs1, v1, v2) + angle_at(vs2, v1, v2) <= float(D_TAU / 2) + 1e-5f) continue;
        for (Face fe : mesh.faces(e)) patch.remove(fe);
        const Edge enew = mesh.swap_edge(e);
        for (Face fe : mesh.faces(enew))
          patch.enter(fe), set_corner_strings(fe);  // The swap loses the corner strings.
        num_swaps++;
      }
      if (!num_swaps) break;
    }
  };
  mesh.destroy_face(f);
  if (triangles.num()) {
    for (const auto& [i, m, k] : triangles) {
      const Face fn = mesh.create_face(va[i], va[m], va[k]);
      set_corner_strings(fn);
      patch.enter(fn);
    }
  } else {
    const Face fn = mesh.create_face(va);
    set_corner_strings(fn);
    const Vertex vn = mesh.center_split_face(fn);
    for (Face ff : mesh.faces(vn)) patch.enter(ff);
    density.enter(vn, float(mean(density.values())));
  }
  if (fstring != "")
    for (Face ff : patch) mesh.set_string(ff, fstring.c_str());

  // (2) Refine the triangles that are larger than the local density, alternated with edge relaxation.
  const float alpha = options.density_factor;
  Array<Vertex> added_vertices;
  for (Face ff : patch)
    for (Vertex v : mesh.vertices(ff))
      if (!density.contains(v)) added_vertices.push(v);  // The center of a fan.
  relax_edges();
  while (options.refine) {
    int num_splits = 0;
    for (Face ff : Array<Face>(patch)) {
      const Vec3<Vertex> tv = mesh.triangle_vertices(ff);
      const Point center = mean(V(mesh.point(tv[0]), mesh.point(tv[1]), mesh.point(tv[2])));
      const float center_density = (density.get(tv[0]) + density.get(tv[1]) + density.get(tv[2])) / 3.f;
      const bool split = ranges::all_of(tv, [&](Vertex v) {
        const float d = alpha * dist(center, mesh.point(v));
        return d > center_density && d > density.get(v);
      });
      if (!split) continue;
      patch.remove(ff);
      const Vertex vn = mesh.center_split_face(ff);
      mesh.set_point(vn, center);
      density.enter(vn, center_density);
      added_vertices.push(vn);
      for (Face fn : mesh.faces(vn)) patch.enter(fn);
      num_splits++;
    }
    if (!num_splits) break;
    relax_edges();
  }

  // (3) Fair the new vertices: the scale-dependent umbrella operator U(v) = sum_u w_vu (p_u - p_v) / sum_u w_vu with
  // w_vu = 1 / |p_u - p_v|, and the equations U(v) = 0 (membrane) or U(U(v)) = 0 (thin plate) at the new vertices.
  if (options.fair && added_vertices.num()) {
    const bool closed_rings = ranges::none_of(va, [&](Vertex v) { return mesh.is_boundary(v); });
    if (options.fairing_order == 2 && !closed_rings)
      assertnever("fair_triangulate_face: the thin-plate fairing needs the loop vertices to have closed rings");
    const bool thin_plate = options.fairing_order == 2 || (options.fairing_order == 0 && closed_rings);
    Map<Vertex, int> index;
    for_int(i, added_vertices.num()) index.enter(added_vertices[i], i);
    const auto umbrella = [&](Vertex v) {  // The coefficients of U(v) on the vertices.
      Map<Vertex, float> coefs;
      float sum_w = 0.f;
      for (Vertex u : mesh.vertices(v)) sum_w += 1.f / max(mesh.length(mesh.edge(v, u)), 1e-20f);
      for (Vertex u : mesh.vertices(v)) {
        coefs[u] += 1.f / max(mesh.length(mesh.edge(v, u)), 1e-20f) / sum_w;
      }
      coefs[v] -= 1.f;
      return coefs;
    };
    const int nn = added_vertices.num();
    unique_ptr<Lls> lls = Lls::make(nn, nn, 3, min(1.f, 40.f / float(nn)));
    for_int(r, nn) {
      const Vertex v = added_vertices[r];
      Map<Vertex, float> row;
      if (thin_plate) {
        const Map<Vertex, float> uv = umbrella(v);
        for (const auto& [u, w] : uv)
          for (const auto& [x, c] : umbrella(u)) row[x] += w * c;
      } else {
        row = umbrella(v);
      }
      Vector rhs(0.f, 0.f, 0.f);
      for (const auto& [x, c] : row) {
        if (const int* pi = index.find_ptr(x)) {
          lls->enter_a_rc(r, *pi, c);
        } else {
          rhs -= c * Vector(mesh.point(x));
        }
      }
      lls->enter_b_r(r, rhs);
      lls->enter_xest_r(r, mesh.point(v));
    }
    if (assertw(lls->solve())) {
      for_int(i, nn) {
        Point p;
        for_int(c, 3) p[c] = lls->get_x_rc(i, c);
        mesh.set_point(added_vertices[i], p);
      }
    }
  }
  if (added_vertices.num()) {
    // The new vertices get normals from the faired geometry (the sum of the normals of their faces), if the loop
    // vertices have normals.
    string str;
    for (Vertex v : added_vertices) {
      Vector sum_normal(0.f, 0.f, 0.f);
      for (Face ff : mesh.faces(v)) sum_normal += get_normal_dir(mesh.triangle_points(ff));
      mesh.update_string(v, "normal", loop_has_normals ? csform_vec(str, ok_normalized(sum_normal)) : nullptr);
    }
  }
  result.success = true;
  result.new_faces = Array<Face>(patch);
  result.new_vertices = std::move(added_vertices);
  return result;
}

// Returns cos(angle) of non-boundary edge.  Ranges from -1 to 1.
// Returns -2 if either face is degenerate.
float edge_dihedral_angle_cos(const GMesh& mesh, Edge e) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  assertx(f2);
  if (mesh.is_triangle(f1) && mesh.is_triangle(f2)) {
    return dihedral_angle_cos(mesh.point(v1), mesh.point(v2), mesh.point(mesh.side_vertex1(e)),
                              mesh.point(mesh.side_vertex2(e)));
  } else {
    return dihedral_angle_cos(mesh.point(v1), mesh.point(v2),
                              interp(mesh.point(mesh.ccw_vertex(f1, v2)), mesh.point(mesh.clw_vertex(f1, v1))),
                              interp(mesh.point(mesh.ccw_vertex(f2, v1)), mesh.point(mesh.clw_vertex(f2, v2))));
  }
}

float edge_signed_dihedral_angle(const GMesh& mesh, Edge e) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  assertx(f2);
  if (mesh.is_triangle(f1) && mesh.is_triangle(f2)) {
    return signed_dihedral_angle(mesh.point(v1), mesh.point(v2), mesh.point(mesh.side_vertex1(e)),
                                 mesh.point(mesh.side_vertex2(e)));
  } else {
    return signed_dihedral_angle(mesh.point(v1), mesh.point(v2),
                                 interp(mesh.point(mesh.ccw_vertex(f1, v2)), mesh.point(mesh.clw_vertex(f1, v1))),
                                 interp(mesh.point(mesh.ccw_vertex(f2, v1)), mesh.point(mesh.clw_vertex(f2, v2))));
  }
}

float vertex_solid_angle(const GMesh& mesh, Vertex v) {
  assertx(!mesh.num_boundaries(v));
  const int np = mesh.degree(v);
  int i = np;
  Array<Point> pa(np);
  // Really want clockwise order so that solid angle points toward inside of mesh.
  for (Vertex vv : mesh.ccw_vertices(v)) pa[--i] = mesh.point(vv);
  return solid_angle(mesh.point(v), pa);
}

float collapse_edge_inscribed_criterion(const GMesh& mesh, Edge e, int ii) {
  const Point& p1 = mesh.point(mesh.vertex1(e));
  const Point& p2 = mesh.point(mesh.vertex2(e));
  const Point& po1 = mesh.point(mesh.side_vertex1(e));
  Vertex vo2 = mesh.side_vertex2(e);
  float rc = inscribed_radius(p1, p2, po1);
  if (vo2) {
    const Point& po2 = mesh.point(vo2);
    rc = min(rc, inscribed_radius(p1, po2, p2));
  }
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  Array<Vector> ar_normals;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (f == f1 || f == f2) continue;
      const Vec3<Point> triangle = mesh.triangle_points(f);
      ar_normals.push(get_normal_dir(triangle));
    }
  }
  const Point newp = interp(mesh.point(v1), mesh.point(v2), ii * .5f);
  int nnor = 0;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (f == f1 || f == f2) continue;
      const Vec3<Vertex> va = mesh.triangle_vertices(f);
      Vec3<Point> triangle = mesh.triangle_points(f);
      for_int(i, 3) if (va[i] == v1 || va[i] == v2) triangle[i] = newp;
      if (dot(ar_normals[nnor], get_normal_dir(triangle)) < 0.f) return BIGFLOAT;  // Flipped normal.
      nnor++;
    }
  }
  assertx(nnor == ar_normals.num());
  return dist(p1, p2) * rc;
}

float collapse_edge_volume_criterion(const GMesh& mesh, Edge e) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  if (mesh.is_boundary(v1) || mesh.is_boundary(v2)) return BIGFLOAT;
  Array<Vector> ar_normals;
  float vol_b = 0.f;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (v == v2 && (f == f1 || f == f2)) continue;
      const Vec3<Point> triangle = mesh.triangle_points(f);
      vol_b += dot<float>(cross(triangle[0], triangle[1]), triangle[2]);
      if (f == f1 || f == f2) continue;
      ar_normals.push(get_normal_dir(triangle));
    }
  }
  const Point newp = interp(mesh.point(v1), mesh.point(v2));
  int nnor = 0;
  float vol_a = 0.f;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (f == f1 || f == f2) continue;
      const Vec3<Vertex> va = mesh.triangle_vertices(f);
      Vec3<Point> triangle = mesh.triangle_points(f);
      for_int(i, 3) if (va[i] == v1 || va[i] == v2) triangle[i] = newp;
      vol_a += dot<float>(cross(triangle[0], triangle[1]), triangle[2]);
      if (dot(ar_normals[nnor], get_normal_dir(triangle)) < 0.f) return BIGFLOAT;  // A flipped normal.
      nnor++;
    }
  }
  assertx(nnor == ar_normals.num());
  return abs(vol_a - vol_b);
}

float collapse_edge_qem_criterion(const GMesh& mesh, Edge e) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  const bool isb1 = mesh.is_boundary(v1), isb2 = mesh.is_boundary(v2);
  const int ii = isb1 && !isb2 ? 2 : isb2 && !isb1 ? 0 : 1;
  const Point newp = interp(mesh.point(v1), mesh.point(v2), ii * .5f);
  double qem = 0.;
  InlinedArray<Vector, 32> ar_normals;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (v == v2 && (f == f1 || f == f2)) continue;
      const Vec3<Point> triangle = mesh.triangle_points(f);
      const Vector normal = ok_normalized(cross(triangle[0], triangle[1], triangle[2]));
      const float d = -dot(triangle[0], normal);
      qem += square(dot(newp, normal) + d);
      if (f == f1 || f == f2) continue;
      ar_normals.push(normal);
    }
  }
  int nnor = 0;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (f == f1 || f == f2) continue;
      const Vec3<Vertex> va = mesh.triangle_vertices(f);
      Vec3<Point> triangle;
      for_int(i, 3) triangle[i] = va[i] == v1 || va[i] == v2 ? newp : mesh.point(va[i]);
      if (dot(ar_normals[nnor], cross(triangle[0], triangle[1], triangle[2])) < 0.f)
        return BIGFLOAT;  // Flipped normal.
      nnor++;
    }
  }
  assertx(nnor == ar_normals.num());
  return float(qem);
}

Set<Face> mesh_remove_boundary(GMesh& mesh, Edge erep) {
  Set<Face> setf;
  Queue<Edge> queuee = gather_boundary(mesh, erep);
  while (!queuee.empty()) {
    Queue<Edge> qc;
    {
      Set<Vertex> setv;
      Vertex v = nullptr;
      for (Edge e : queuee) {
        Vertex vv = mesh.vertex2(e);
        if (!setv.add(vv)) {
          v = vv;
          break;
        }
      }
      if (!v) {
        // qc.add_to_end(queuee);  // Clears queuee.
        while (!queuee.empty()) qc.enqueue(queuee.dequeue());
      } else {
        // Rotate queuee to put v at the front.
        while (mesh.vertex2(queuee.front()) != v) queuee.enqueue(queuee.dequeue());
        // Extract the loop from queuee into qc.
        for (;;) {
          Edge e = queuee.dequeue();
          qc.enqueue(e);
          if (mesh.vertex1(e) == v) break;
        }
      }
    }
    // Remove the simple boundary in qc.
    {
      Array<Vertex> va;
      // vertex1(e) ok but slower.
      for (Edge e : qc) va.push(mesh.vertex2(e));
      Face fn = mesh.create_face(va);
      // Heuristically propagate string info from adjacent corners onto new corners.
      for (Corner c : mesh.corners(fn)) {
        const char* s1 = mesh.get_string(mesh.ccw_corner(c));
        const char* s2 = mesh.get_string(mesh.clw_corner(c));
        if (s1 || s2) {
          if (!s1 || !s2) Warning("fill_hole: absent corner info");
          if (!s1) s1 = s2;
          if (!s2) s2 = s1;
          if (strcmp(s1, s2) != 0) Warning("fill_holes: inconsistent corner info");
          mesh.set_string(c, s1);
        }
      }
      setf.enter(fn);
    }
  }
  return setf;
}

void split_valence(GMesh& mesh, int max_valence) {
  assertx(max_valence > 6);
  UpdatablePriorityQueue<Vertex> pqv;
  const int large_int = 1 << 24;  // Precision of float in UpdatablePriorityQueue.
  for (Vertex v : mesh.vertices())
    if (mesh.degree(v) >= max_valence) pqv.enter(v, float(large_int - mesh.degree(v)));
  int nsplit = 0;
  while (!pqv.empty()) {
    Vertex v = pqv.remove_min();
    assertx(mesh.degree(v) >= max_valence);
    assertx(!mesh.is_boundary(v));  // Not implemented.
    HH_SSTAT(Sval, mesh.degree(v));
    nsplit++;
    const Array prev_faces(mesh.faces(v));
    const Array va(mesh.ccw_vertices(v));
    Vertex vs1, vs2;
    Vector offset{}, offsetn{};
    if (1) {
      // Compute the mean of the radial vectors to find the tangent direction towards the most adjacent vertices.
      // Split the center vertex and push the new vertex in that direction, keeping the old vertex constant.
      // Determine the two side vertices of the vertex split using -33% and +33% cuts of the adjacent vertices loop,
      // rather than -25% and +25% cuts as might be desired under a uniform distribution of edges.
      Vector vec_majority{};
      for_int(i, va.num()) vec_majority += mesh.point(va[i]) - mesh.point(v);
      const int j =
          arg_max(va | views::transform([&](Vertex vv) { return dot(mesh.point(vv) - mesh.point(v), vec_majority); }));
      vs1 = va[(j + va.num() / 3) % va.num()];
      vs2 = va[(j + va.num() - va.num() / 3) % va.num()];
      offset = Vector{};
      if (0) {
        offsetn = (mesh.point(va[j]) - mesh.point(v)) * 0.2f;
      } else {
        const Point centroid = mean(va | views::transform([&](Vertex vv) { return mesh.point(vv); }));
        const float frac1 = 0.7f;
        offsetn = interp(mesh.point(v), centroid, frac1) - mesh.point(v);
      }
    }
    if (mag2(offsetn) < 1e-8f) {
      // Use sine weights to compute an arbitrary tangent vector.  It works well with a uniform distribution of edges.
      Vector vec{};
      for_int(i, va.num()) vec += (mesh.point(va[i]) - mesh.point(v)) * std::sin(float(i) / va.num() * TAU);
      vs1 = va[0];
      vs2 = va[va.num() / 2];
      const float frac = 1e-3f;
      offset = vec * frac;
      offsetn = -vec * frac;
    }
    // Note that all corners (and their strings) are preserved in split_vertex().
    Vertex vn = mesh.split_vertex(v, vs1, vs2, 0);
    Vec2<Face> new_faces{mesh.create_face(v, vn, vs1), mesh.create_face(v, vs2, vn)};
    {
      Array<char> key, val;
      string str;
      for_cstring_key_value(mesh.get_string(prev_faces[0]), key, val, [&] {
        const bool all_same = ranges::all_of(prev_faces, [&](Face f) {
          const char* s = GMesh::string_key(str, mesh.get_string(f), key.data());
          return s && !strcmp(s, val.data());
        });
        if (all_same)
          for (Face f : new_faces) mesh.update_string(f, key.data(), val.data());
      });
    }
    mesh.set_point(vn, mesh.point(v) + offsetn);
    mesh.set_point(v, mesh.point(v) + offset);
    mesh.set_string(vn, mesh.get_string(v));
    mesh.update_string(vn, "newvertex", "");
    mesh.update_string(v, "newvertex", "");
    // We assign string info to new corners if defined on adjacent corners (see also CloseMinCycles::close_cycle()).
    for (Face f : new_faces) {
      for (Corner c : mesh.corners(f)) {
        for (Corner c2 : V(mesh.ccw_corner(c), mesh.clw_corner(c))) {
          if (const char* s = mesh.get_string(c2)) {
            mesh.set_string(c, s);
            break;
          }
        }
      }
    }
    pqv.remove(vs1);
    pqv.remove(vs2);
    for (Vertex vv : V(vs1, vs2, v, vn))
      if (mesh.degree(vv) >= max_valence) pqv.enter(vv, float(large_int - mesh.degree(vv)));
  }
  showdf("Split high-valence vertices %d times\n", nsplit);
}

// *** Retriangulate

int retriangulate_all(GMesh& mesh, float mincos, EDGEF fdoswap, EDGEF fdel, EDGEF fadd) {
  Set<Edge> sete;
  for (Edge e : mesh.edges())
    if (!mesh.is_boundary(e)) sete.enter(e);
  return retriangulate(mesh, sete, true, nullptr, mincos, fdoswap, fdel, fadd);
}

int retriangulate_from_edge(GMesh& mesh, Edge e, float mincos, EDGEF fdoswap, EDGEF fdel, EDGEF fadd) {
  Set<Edge> sete;
  sete.enter(e);
  return retriangulate(mesh, sete, true, nullptr, mincos, fdoswap, fdel, fadd);
}

int retriangulate_one_edge(GMesh& mesh, Edge e, float mincos, EDGEF fdoswap, EDGEF fdel, EDGEF fadd) {
  Set<Edge> sete;
  sete.enter(e);
  return retriangulate(mesh, sete, false, nullptr, mincos, fdoswap, fdel, fadd);
}

bool circum_radius_swap_criterion(const GMesh& mesh, Edge e) {
  assertx(!mesh.is_boundary(e));
  const Point& p1 = mesh.point(mesh.vertex1(e));
  const Point& p2 = mesh.point(mesh.vertex2(e));
  const Point& po1 = mesh.point(mesh.side_vertex1(e));
  const Point& po2 = mesh.point(mesh.side_vertex2(e));
  const float rc1 = circum_radius(p1, p2, po1);
  const float rc2 = circum_radius(p1, po2, p2);
  const float rs1 = circum_radius(p1, po2, po1);
  const float rs2 = circum_radius(p2, po1, po2);
  return max(rs1, rs2) < max(rc1, rc2);
}

bool diagonal_distance_swap_criterion(const GMesh& mesh, Edge e) {
  assertx(!mesh.is_boundary(e));
  const Point& p1 = mesh.point(mesh.vertex1(e));
  const Point& p2 = mesh.point(mesh.vertex2(e));
  const Point& po1 = mesh.point(mesh.side_vertex1(e));
  const Point& po2 = mesh.point(mesh.side_vertex2(e));
  return dist2(po1, po2) < dist2(p1, p2);
}

// *** Normal estimation

namespace {

inline bool sharp(const GMesh& mesh, Vertex v, Edge e) {
  if (mesh.is_boundary(e)) return true;
  if (mesh.flags(e).flag(GMesh::eflag_sharp)) return true;
  const char* s1 = mesh.get_string(mesh.ccw_corner(v, e));
  if (s1 && GMesh::string_has_key(s1, "wid")) {
    const char* s2 = mesh.get_string(mesh.clw_corner(v, e));
    string str1, str2;
    const char* sk1 = assertx(GMesh::string_key(str1, s1, "wid"));
    const char* sk2 = GMesh::string_key(str2, s2, "wid");
    if (sk2 && strcmp(sk1, sk2) != 0) return true;
  }
  return false;
}

bool extraordinary_crease_vertex(const GMesh& mesh, Vertex v) {
  const int ne = mesh.degree(v);
  if (mesh.is_boundary(v)) return ne != 4;
  if (ne != 6) return true;
  int nside = 0, sharpef = 0;
  for (Vertex vv : mesh.ccw_vertices(v)) {
    if (sharpef == 1) nside++;
    if (sharp(mesh, v, mesh.edge(v, vv))) sharpef++;
  }
  assertx(sharpef == 2);
  return nside != 3;
}

Vnors::EType get_default_nor_type() {
  static Vnors::EType default_nor_type;
  static std::once_flag flag;
  const auto initialize_default_nor_type = [] {
    default_nor_type = (getenv_bool("SUM_NOR")      ? Vnors::EType::sum
                        : getenv_bool("AREA_NOR")   ? Vnors::EType::area
                        : getenv_bool("SLOAN_NOR")  ? Vnors::EType::sloan
                        : getenv_bool("SUBDIV_NOR") ? Vnors::EType::subdiv
                        : getenv_bool("ANGLE_NOR")  ? Vnors::EType::angle
                                                    : Vnors::EType::unspecified);
    if (default_nor_type == Vnors::EType::unspecified)
      default_nor_type = Vnors::EType::angle;
    else
      Warning("Normal computation method is explicitly specified");
  };
  std::call_once(flag, initialize_default_nor_type);
  return default_nor_type;
}

}  // namespace

Vnors::Vnors(const GMesh& mesh, Vertex v, EType nortype) {
  static const bool ignore_mesh_normals = getenv_bool("IGNORE_MESH_NORMALS");
  const bool hasvnor = ignore_mesh_normals ? false : parse_key_vec(mesh.get_string(v), "normal", _nor);
  const auto corner_has_normal = [&](Corner c) { return GMesh::string_has_key(mesh.get_string(c), "normal"); };
  const int ncnor = ignore_mesh_normals ? 0 : int(ranges::count_if(mesh.corners(v), corner_has_normal));
  if (hasvnor && !ncnor) return;
  if (hasvnor && ncnor) Warning("Have both vertex and corner normals");

  if (nortype == EType::unspecified) nortype = get_default_nor_type();
  if (nortype == EType::subdiv && !mesh.is_nice(v)) {
    Warning("attempt to eval subdiv normal at non-nice vertex");
    nortype = EType::angle;
  }
  const Point& vp = mesh.point(v);
  const bool is_cusp = mesh.flags(v).flag(GMesh::vflag_cusp);
  const int nfaces = narrow_cast<int>(ranges::size(mesh.faces(v)));
  if (!nfaces) {
    Warning("Isolated vertex has undefined normal");
    return;
  }
  const int nsharpe = int(ranges::count_if(mesh.edges(v), [&](Edge e) { return sharp(mesh, v, e); }));
  InlinedArray<Face, 10> faces_visited;
  for (Face frep : mesh.faces(v)) {
    if (contains(faces_visited, frep)) continue;
    bool closed = false;
    InlinedArray<Vertex, 10> av;
    InlinedArray<Face, 10> af;
    Face f = frep;
    for (;;) {  // Find f: most_clw, or frep if closed.
      Edge e = mesh.ccw_edge(f, v);
      if (sharp(mesh, v, e)) break;
      f = mesh.opp_face(f, e);
      if (f == frep) {
        closed = true;
        break;
      }
    }
    for (;;) {  // Now go ccw.
      av.push(mesh.opp_vertex(v, mesh.ccw_edge(f, v)));
      af.push(f);
      Edge e = mesh.clw_edge(f, v);
      if (is_cusp && nsharpe < 2) {
        closed = false;
        av.push(mesh.opp_vertex(v, e));
        break;
      }
      if (!closed && sharp(mesh, v, e)) {
        // Could still be a dart (!closed and opp_vertex(v, e) == va[0]).
        if (mesh.opp_vertex(v, e) != av[0]) av.push(mesh.opp_vertex(v, e));
        break;
      }
      f = mesh.opp_face(f, e);
      if (closed && f == frep) break;
    }
    dummy_use(closed);
    const int avn = av.num();
    Vector vec{};
    // It once seemed that "double" was needed below to overcome an apparent problem with poorly computed surface
    // normals.  However, the problem lay in the geometry. Prefiltering with "Filtermesh -taubinsmooth 4" solved it.
    // EType::angle still seems like a good scheme.
    Polygon poly;
    switch (nortype) {
      case EType::angle:
        for_int(i, af.num()) {
          int i1 = i + 1;
          if (i1 == av.num()) i1 = 0;
          mesh.polygon(af[i], poly);
          const Vector face_normal = poly.get_normal();
          // A degenerate face (e.g., with coincident vertices) has a zero normal and contributes nothing; skipping it
          // also avoids passing a zero vector to angle_between_unit_vectors(), which asserts in debug builds.
          if (is_zero(face_normal)) continue;
          const float ang = angle_between_unit_vectors(ok_normalized(mesh.point(av[i]) - vp),
                                                       ok_normalized(mesh.point(av[i1]) - vp));
          vec += face_normal * ang;
        }
        break;
      case EType::sum:
        for_int(i, af.num()) {
          mesh.polygon(af[i], poly);
          vec += poly.get_normal();
        }
        break;
      case EType::area:
        for_int(i, af.num()) {
          mesh.polygon(af[i], poly);
          const float area = poly.get_area();
          vec += poly.get_normal() * area;
        }
        break;
      case EType::sloan:
        for_int(i, af.num()) {
          mesh.polygon(af[i], poly);
          const float area = poly.get_area();
          if (!assertw(area)) continue;
          vec += poly.get_normal() / square(area);
        }
        break;
      default:
        if (nsharpe > 2 || is_cusp) {  // A corner.
          vec = cross(vp, mesh.point(av[0]), mesh.point(av.last()));
          if (avn > 2) {  // From Polygon::get_normal_dir().
            Vector pnor{};
            for_int(i, avn - 1) pnor += cross(vp, mesh.point(av[i]), mesh.point(av[i + 1]));
            if (dot(vec, pnor) < 0.f) {
              Warning("flipnor_corner");
              vec = -vec;
            }
          }
        } else if (nsharpe == 2 && !extraordinary_crease_vertex(mesh, v)) {
          // Regular crease vertex 1, .5, -1, -1, .5.
          assertx(avn == 4);
          const Vector v1 = mesh.point(av[3]) - mesh.point(av[0]);
          const Vector v2 =
              vp + mesh.point(av[0]) * .5f + mesh.point(av[3]) * .5f - mesh.point(av[1]) - mesh.point(av[2]);
          vec = cross(v1, v2);
          {  // The direction could be wrong (very bad case).
            Vector pnor{};
            for_int(i, avn - 1) pnor += cross(vp, mesh.point(av[i]), mesh.point(av[i + 1]));
            if (dot(vec, pnor) < 0.f) {
              Warning("flipnor_regcrease");
              vec = -vec;
            }
          }
        } else if (nsharpe == 2) {  // Non-regular crease vertex.
          const Vector v1 = mesh.point(av.last()) - mesh.point(av[0]);
          if (avn == 2) {  // 2, -1, -1
            vec = vp * 2.f - mesh.point(av[0]) - mesh.point(av[1]);
          } else if (avn == 3) {  // 1, 0, -1, 0
            vec = vp - mesh.point(av[1]);
          } else {
            // float theta = (TAU / 2) / (avn - 1.f);
            const int idenom = 2 * (avn - 1);
            vec = (mesh.point(av[0]) + mesh.point(av.last())) * Trig::sin(1, idenom);
            const float fcom = (2 * Trig::cos(1, idenom) - 2);
            for_intL(i, 1, avn - 1) vec += mesh.point(av[i]) * (fcom * Trig::sin(i, idenom));
          }
          vec = cross(v1, vec);
          if (avn > 2) {
            Vector pnor{};
            for_int(i, avn - 1) pnor += cross(vp, mesh.point(av[i]), mesh.point(av[i + 1]));
            if (dot(vec, pnor) < 0.f) {
              Warning("flipnor_crease");
              vec = -vec;
            }
          }
        } else if (avn == 2) {
          Warning("Degree-2 vertex --- assuming bi-cubic quad special");
          const Vector t1 = mesh.point(av[1]) - mesh.point(av[0]);
          for_int(i, 2) {
            Vertex vo = mesh.clw_vertex(av[i], v);
            assertx(vo == mesh.ccw_vertex(av[1 - i], v));  // Check is_quad.
          }
          const Vector t2 = mesh.point(mesh.clw_vertex(av[1], v)) - mesh.point(mesh.clw_vertex(av[0], v));
          vec = cross(t1, t2);
        } else {  // Interior or dart.
          Vector v1{}, v2{};
          for_int(i, avn) {
            const Point& p = mesh.point(av[i]);
            v1 += p * Trig::cos(i, avn);
            v2 += p * Trig::sin(i, avn);
          }
          vec = cross(v1, v2);
        }
    }
    assertw(vec.normalize());
    if (af.num() == nfaces) {
      if (!hasvnor) _nor = vec;
      break;
    }
    if (!_mfnor) _mfnor = make_unique<Map<Face, Vector>>();
    for (Face ff : af) {
      faces_visited.push(ff);
      _mfnor->enter(ff, vec);
    }
  }
  if (ncnor) {
    if (!_mfnor) _mfnor = make_unique<Map<Face, Vector>>();
    for (Corner c : mesh.corners(v)) {
      Vector nor;
      if (!parse_key_vec(mesh.get_string(c), "normal", nor)) {
        if (0) SHOW(c);
        if (hasvnor)
          Warning("Missing corner normal, using vertex normal");
        else
          Warning("Missing corner normal, using estimated normal");
        nor = _nor;
      }
      Face f = mesh.corner_face(c);
      (*_mfnor)[f] = nor;
    }
  }
}

// *** Project Point near face

float project_point_neighborhood(const GMesh& mesh, const Point& p, Face& pf, Bary& ret_bary, Point& ret_clp,
                                 bool fast) {
  static const bool slow_project = getenv_bool("SLOW_PROJECT");
  assertw(!slow_project);
  if (slow_project) fast = false;
  bool pfsmooth = fast;
  for (Edge e : mesh.edges(pf))
    if (mesh.flags(e).flag(GMesh::eflag_sharp)) pfsmooth = false;
  const float bnearedge = .08f;
  const Vec3<Point> triangle1 = mesh.triangle_points(pf);
  auto [mind2, minbary, clp1] = project_point_triangle(p, triangle1);
  ret_clp = clp1;
  const float nearest_edge = min(minbary);
  ASSERTX(nearest_edge >= 0.f && nearest_edge < .34f);  // Optional.
  const bool nearedge = nearest_edge < bnearedge;
  const bool projquick = pfsmooth && !nearedge;
  HH_SSTAT(Sprojquick, projquick);
  if (projquick) {
    ret_bary = minbary;
    return mind2;
  }
  // The visited faces are few (typically 10 to 20), so inline storage avoids heap allocation.
  InlinedSet<Face, 64> faces_visited;
  faces_visited.enter(pf);
  int nvis = 1;
  int ni = 0;
  for (;; ni++) {
    // Look at faces adjacent to vertices with minbary > bnearvertex.
    InlinedSet<Face, 32> faces;  // Faces not yet visited.
    const Vec3<Vertex> va = mesh.triangle_vertices(pf);
    for_int(j, 3) {
      if (pfsmooth && minbary[mod3(j + 1)] > bnearedge && minbary[mod3(j + 2)] > bnearedge) continue;
      for (Face f : mesh.faces(va[j]))
        if (!faces_visited.contains(f)) faces.add(f);
    }
    struct S {
      Face f;
      float d2;
    };
    InlinedArray<S, 32> ar;
    for (Face f : faces) {
      const Vec3<Point> triangle2 = mesh.triangle_points(f);
      const float d2 = square(lb_dist_point_triangle(p, triangle2));
      ar.push(S{f, d2});
    }
    nvis += ar.num();
    const auto by_increasing_distance = [](const S& s1, const S& s2) { return s1.d2 < s2.d2; };
    sort(ar, by_increasing_distance);
    for (const auto& pa : ar) {
      if (pa.d2 >= mind2) break;
      Face f = pa.f;
      const Vec3<Point> triangle2 = mesh.triangle_points(f);
      const auto [d2, bary, clp] = project_point_triangle(p, triangle2);
      if (d2 >= mind2) continue;
      pf = f;
      mind2 = d2;
      minbary = bary;
      ret_clp = clp;
      pfsmooth = fast;
      for (Edge e : mesh.edges(pf))
        if (mesh.flags(e).flag(GMesh::eflag_sharp)) pfsmooth = false;
    }
    if (faces_visited.contains(pf)) break;
    for (Face f : faces) faces_visited.enter(f);
  }
  HH_SSTAT(Sprojnei, ni);
  HH_SSTAT(Sprojf, nvis);
  if (ni > 0) HH_SSTAT(Sprojunexp, !nearedge);
  ret_bary = minbary;
  return mind2;
}

}  // namespace hh
