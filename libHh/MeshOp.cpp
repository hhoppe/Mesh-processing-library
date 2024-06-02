// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshOp.h"

#include <cstring>  // strncmp() etc.
#include <mutex>    // std::once_flag, std::call_once()

#include "libHh/Array.h"
#include "libHh/Facedistance.h"  // lb_dist_point_triangle(), project_point_triangle2()
#include "libHh/GeomOp.h"
#include "libHh/MathOp.h"  // Trig
#include "libHh/Polygon.h"
#include "libHh/RangeOp.h"  // sort()
#include "libHh/Set.h"
#include "libHh/Stack.h"

namespace hh {

// *** helper

namespace {

struct hash_edge {
  size_t operator()(Edge e) const {
    return _mesh.vertex_id(_mesh.vertex1(e)) + intptr_t{_mesh.vertex_id(_mesh.vertex2(e))} * 76541;
  }
  const GMesh& _mesh;
};
using SetEdge = Set<Edge, hash_edge>;  // hashing does not use pointer values, for portable random.

int retriangulate(GMesh& mesh, SetEdge& sete, bool recurse, Set<Vertex>* setvr, float mincos, EDGEF fdoswap,
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
    for (Face f : mesh.faces(e)) {
      for (Edge ee : mesh.edges(f)) sete.remove(ee);
    }
    Edge ne = mesh.swap_edge(e);
    // e = nullptr;  // now undefined
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
    for (Face f2 : mesh.faces(f)) {
      if (setf.add(f2)) queue.enqueue(f2);
    }
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
    for (Vertex v : mesh.vertices(f)) {
      for (Face f2 : mesh.faces(v)) {
        if (setf.add(f2)) queue.enqueue(f2);
      }
    }
    if (queue.empty()) break;
    f = queue.dequeue();
  }
  return setf;
}

Stat mesh_stat_boundaries(const Mesh& mesh) {
  Stat Sbound;
  Set<Edge> setevis;  // boundary edges already considered
  for (Edge e : mesh.edges()) {
    if (!mesh.is_boundary(e)) continue;
    if (setevis.contains(e)) continue;
    Queue<Edge> queuee = gather_boundary(mesh, e);
    for (Edge ee : queuee) setevis.enter(ee);
    Sbound.enter(queuee.length());
  }
  return Sbound;
}

Stat mesh_stat_components(const Mesh& mesh) {
  Stat Scompf;
  if (0) {
    Set<Face> setfvis;  // faces already considered
    for (Face f : mesh.faces()) {
      if (setfvis.contains(f)) continue;
      Set<Face> setf = gather_component(mesh, f);
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
  int nv = mesh.num_vertices();
  int nf = mesh.num_faces();
  int ne = mesh.num_edges();
  // Notes:
  //  For mesh without boundary, nf = nv * 2 + (genus - 1) * 4
  //  For mesh without boundary, ec = 2 - 2 * genus
  Stat Sbound = mesh_stat_boundaries(mesh);
  int nb = Sbound.inum();
  Stat Scompf = mesh_stat_components(mesh);
  int nc = Scompf.inum();
  int ec = nv - ne + nf;  // euler characteristic
  float genus = (nc * 2 - ec - nb) / 2.f;
  return genus;
}

string mesh_genus_string(const Mesh& mesh) {
  int nv = mesh.num_vertices();
  int nf = mesh.num_faces();
  int ne = mesh.num_edges();
  // Notes:
  //  For mesh without boundary, nf = nv * 2 + (genus - 1) * 4
  //  For mesh without boundary, ec = 2 - 2 * genus
  Stat Sbound = mesh_stat_boundaries(mesh);
  int nb = Sbound.inum();
  Stat Scompf = mesh_stat_components(mesh);
  int nc = Scompf.inum();
  int ec = nv - ne + nf;  // euler characteristic
  float genus = (nc * 2 - ec - nb) / 2.f;
  int nse = 0, ncv = 0;
  for (Edge e : mesh.edges()) {
    if (mesh.flags(e).flag(GMesh::eflag_sharp)) nse++;
  }
  for (Vertex v : mesh.vertices()) {
    if (mesh.flags(v).flag(GMesh::vflag_cusp)) ncv++;
  }
  return sform("Genus: c=%d b=%d  v=%d f=%d e=%d  genus=%g%s",  //
               nc, nb, nv, nf, ne, genus, (nse + ncv) ? sform("  sharpe=%d cuspv=%d", nse, ncv).c_str() : "");
}

bool triangulate_face(GMesh& mesh, Face f) {
  Array<Vertex> va;
  mesh.get_vertices(f, va);
  int nv = va.num();
  if (!assertw(nv > 3)) return true;
  for_intL(i, 2, nv - 1) {
    if (!assertw(!mesh.query_edge(va[0], va[i]))) return false;
  }
  mesh.destroy_face(f);
  for_int(i, nv - 2) mesh.create_face(va[0], va[i + 1], va[i + 2]);
  Set<Vertex> setvr;  // vertices on ring of original face
  for_int(i, nv) setvr.enter(va[i]);
  hash_edge he{mesh};
  SetEdge sete(he);  // initially, inner edges
  for_intL(i, 2, nv - 1) sete.enter(mesh.edge(va[0], va[i]));
  retriangulate(mesh, sete, true, &setvr, -2, circum_radius_swap_criterion, nullptr, nullptr);
  return true;
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
  int np = mesh.degree(v), i = np;
  Array<Point> pa(np);
  // Really want clockwise order so that solid angle points toward inside of mesh.
  for (Vertex vv : mesh.ccw_vertices(v)) pa[--i] = mesh.point(vv);
  return solid_angle(mesh.point(v), pa);
}

float collapse_edge_inscribed_criterion(const GMesh& mesh, Edge e) {
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
  Polygon poly;
  Array<Vector> ar_normals;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (f == f1 || f == f2) continue;
      mesh.polygon(f, poly);
      assertx(poly.num() == 3);
      ar_normals.push(poly.get_normal_dir());
    }
  }
  Point newp = interp(mesh.point(v1), mesh.point(v2));
  Array<Vertex> va;
  int nnor = 0;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (f == f1 || f == f2) continue;
      mesh.polygon(f, poly);
      assertx(poly.num() == 3);
      mesh.get_vertices(f, va);
      for_int(i, 3) {
        if (va[i] == v1 || va[i] == v2) poly[i] = newp;
      }
      if (dot(ar_normals[nnor], poly.get_normal_dir()) < 0) return BIGFLOAT;  // flipped normal
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
  Polygon poly;
  Array<Vector> ar_normals;
  float vol_b = 0.f;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (v == v2 && (f == f1 || f == f2)) continue;
      mesh.polygon(f, poly);
      assertx(poly.num() == 3);
      vol_b += dot(cross(to_Vector(poly[0]), to_Vector(poly[1])), to_Vector(poly[2]));
      if (f == f1 || f == f2) continue;
      ar_normals.push(poly.get_normal_dir());
    }
  }
  Point newp = interp(mesh.point(v1), mesh.point(v2));
  Array<Vertex> va;
  int nnor = 0;
  float vol_a = 0.f;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (f == f1 || f == f2) continue;
      mesh.polygon(f, poly);
      assertx(poly.num() == 3);
      mesh.get_vertices(f, va);
      for_int(i, 3) {
        if (va[i] == v1 || va[i] == v2) poly[i] = newp;
      }
      vol_a += dot(cross(to_Vector(poly[0]), to_Vector(poly[1])), to_Vector(poly[2]));
      if (dot(ar_normals[nnor], poly.get_normal_dir()) < 0.f) return BIGFLOAT;  // flipped normal
      nnor++;
    }
  }
  assertx(nnor == ar_normals.num());
  return abs(vol_a - vol_b);
}

float collapse_edge_qem_criterion(const GMesh& mesh, Edge e) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  bool isb1 = mesh.is_boundary(v1), isb2 = mesh.is_boundary(v2);
  int ii = isb1 && !isb2 ? 2 : isb2 && !isb1 ? 0 : 1;
  Point newp = interp(mesh.point(v1), mesh.point(v2), ii * .5f);
  double qem = 0.;
  Vec3<Point> poly;
  PArray<Vector, 12> ar_normals;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (v == v2 && (f == f1 || f == f2)) continue;
      Vec3<Vertex> va = mesh.triangle_vertices(f);
      for_int(i, 3) poly[i] = mesh.point(va[i]);
      Vector normal = ok_normalized(cross(poly[0], poly[1], poly[2]));
      float d = -pvdot(poly[0], normal);
      qem += square(pvdot(newp, normal) + d);
      if (f == f1 || f == f2) continue;
      ar_normals.push(normal);
    }
  }
  int nnor = 0;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (f == f1 || f == f2) continue;
      Vec3<Vertex> va = mesh.triangle_vertices(f);
      for_int(i, 3) poly[i] = va[i] == v1 || va[i] == v2 ? newp : mesh.point(va[i]);
      if (dot(ar_normals[nnor], cross(poly[0], poly[1], poly[2])) < 0.f) return BIGFLOAT;  // flipped normal
      nnor++;
    }
  }
  assertx(nnor == ar_normals.num());
  return float(qem);
}

Set<Face> mesh_remove_boundary(Mesh& mesh, Edge erep) {
  Set<Face> ret_setf;
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
        // qc.add_to_end(queuee);  // clears queuee
        while (!queuee.empty()) qc.enqueue(queuee.dequeue());
      } else {
        // Rotate queuee to put v at front
        while (mesh.vertex2(queuee.front()) != v) queuee.enqueue(queuee.dequeue());
        // Extract loop from queuee into qc
        for (;;) {
          Edge e = queuee.dequeue();
          qc.enqueue(e);
          if (mesh.vertex1(e) == v) break;
        }
      }
    }
    // Remove the simple boundary in qc
    {
      Array<Vertex> va;
      // vertex1(e) ok but slower
      for (Edge e : qc) va.push(mesh.vertex2(e));
      Face fn = mesh.create_face(va);
      ret_setf.enter(fn);
    }
  }
  return ret_setf;
}

// *** Retriangulate

int retriangulate_all(GMesh& mesh, float mincos, EDGEF fdoswap, EDGEF fdel, EDGEF fadd) {
  hash_edge he{mesh};
  SetEdge sete(he);
  for (Edge e : mesh.edges()) {
    if (!mesh.is_boundary(e)) sete.enter(e);
  }
  return retriangulate(mesh, sete, true, nullptr, mincos, fdoswap, fdel, fadd);
}

int retriangulate_from_edge(GMesh& mesh, Edge e, float mincos, EDGEF fdoswap, EDGEF fdel, EDGEF fadd) {
  hash_edge he{mesh};
  SetEdge sete(he);
  sete.enter(e);
  return retriangulate(mesh, sete, true, nullptr, mincos, fdoswap, fdel, fadd);
}

int retriangulate_one_edge(GMesh& mesh, Edge e, float mincos, EDGEF fdoswap, EDGEF fdel, EDGEF fadd) {
  hash_edge he{mesh};
  SetEdge sete(he);
  sete.enter(e);
  return retriangulate(mesh, sete, true, nullptr, mincos, fdoswap, fdel, fadd);
}

bool circum_radius_swap_criterion(const GMesh& mesh, Edge e) {
  assertx(!mesh.is_boundary(e));
  const Point& p1 = mesh.point(mesh.vertex1(e));
  const Point& p2 = mesh.point(mesh.vertex2(e));
  const Point& po1 = mesh.point(mesh.side_vertex1(e));
  const Point& po2 = mesh.point(mesh.side_vertex2(e));
  float rc1 = circum_radius(p1, p2, po1);
  float rc2 = circum_radius(p1, po2, p2);
  float rs1 = circum_radius(p1, po2, po1);
  float rs2 = circum_radius(p2, po1, po2);
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
  int ne = mesh.degree(v);
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
  std::call_once(flag, [] {
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
  });
  return default_nor_type;
}

}  // namespace

Vnors::Vnors(const GMesh& mesh, Vertex v, EType nortype) {
  static const bool ignore_mesh_normals = getenv_bool("IGNORE_MESH_NORMALS");
  const bool hasvnor = ignore_mesh_normals ? false : parse_key_vec(mesh.get_string(v), "normal", _nor);
  const int ncnor = ignore_mesh_normals ? 0 : int(count_if(mesh.corners(v), [&](Corner c) {
    return GMesh::string_has_key(mesh.get_string(c), "normal");
  }));
  if (hasvnor && !ncnor) return;
  if (hasvnor && ncnor) Warning("Have both vertex and corner normals");

  if (nortype == EType::unspecified) nortype = get_default_nor_type();
  if (nortype == EType::subdiv && !mesh.is_nice(v)) {
    Warning("attempt to eval subdiv normal at non-nice vertex");
    nortype = EType::angle;
  }
  const Point& vp = mesh.point(v);
  const bool is_cusp = mesh.flags(v).flag(GMesh::vflag_cusp);
  const int nfaces = int(distance(mesh.faces(v)));
  if (!nfaces) {
    Warning("Isolated vertex has undefined normal");
    return;
  }
  const int nsharpe = int(count_if(mesh.edges(v), [&](Edge e) { return sharp(mesh, v, e); }));
  PArray<Face, 10> faces_visited;
  for (Face frep : mesh.faces(v)) {
    if (faces_visited.contains(frep)) continue;
    bool closed = false;
    PArray<Vertex, 10> av;
    PArray<Face, 10> af;
    Face f = frep;
    for (;;) {  // find f: most_clw, or frep if closed
      Edge e = mesh.ccw_edge(f, v);
      if (sharp(mesh, v, e)) break;
      f = mesh.opp_face(f, e);
      if (f == frep) {
        closed = true;
        break;
      }
    }
    for (;;) {  // now go ccw
      av.push(mesh.opp_vertex(v, mesh.ccw_edge(f, v)));
      af.push(f);
      Edge e = mesh.clw_edge(f, v);
      if (is_cusp && nsharpe < 2) {
        closed = false;
        av.push(mesh.opp_vertex(v, e));
        break;
      }
      if (!closed && sharp(mesh, v, e)) {
        // could still be a dart (!closed and opp_vertex(v, e) == va[0])
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
    Polygon& poly = _tmp_poly;
    switch (nortype) {
      case EType::angle:
        for_int(i, af.num()) {
          int i1 = i + 1;
          if (i1 == av.num()) i1 = 0;
          const float ang = angle_between_unit_vectors(ok_normalized(mesh.point(av[i]) - vp),
                                                       ok_normalized(mesh.point(av[i1]) - vp));
          mesh.polygon(af[i], poly);
          vec += poly.get_normal() * ang;
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
        if (nsharpe > 2 || is_cusp) {  // corner
          vec = cross(vp, mesh.point(av[0]), mesh.point(av.last()));
          if (avn > 2) {  // from Polygon::get_normal_dir()
            Vector pnor{};
            for_int(i, avn - 1) pnor += cross(vp, mesh.point(av[i]), mesh.point(av[i + 1]));
            if (dot(vec, pnor) < 0) {
              Warning("flipnor_corner");
              vec = -vec;
            }
          }
        } else if (nsharpe == 2 && !extraordinary_crease_vertex(mesh, v)) {
          // regular crease vertex 1, .5, -1, -1, .5
          assertx(avn == 4);
          const Vector v1 = mesh.point(av[3]) - mesh.point(av[0]);
          const Vector v2 =
              vp + mesh.point(av[0]) * .5f + mesh.point(av[3]) * .5f - mesh.point(av[1]) - mesh.point(av[2]);
          vec = cross(v1, v2);
          {  // direction could be wrong (very bad case)
            Vector pnor{};
            for_int(i, avn - 1) pnor += cross(vp, mesh.point(av[i]), mesh.point(av[i + 1]));
            if (dot(vec, pnor) < 0) {
              Warning("flipnor_regcrease");
              vec = -vec;
            }
          }
        } else if (nsharpe == 2) {  // non-regular crease vertex
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
            if (dot(vec, pnor) < 0) {
              Warning("flipnor_crease");
              vec = -vec;
            }
          }
        } else if (avn == 2) {
          Warning("Degree-2 vertex --- assuming bi-cubic quad special");
          const Vector t1 = mesh.point(av[1]) - mesh.point(av[0]);
          for_int(i, 2) {
            Vertex vo = mesh.clw_vertex(av[i], v);
            assertx(vo == mesh.ccw_vertex(av[1 - i], v));  // is_quad
          }
          const Vector t2 = (mesh.point(mesh.clw_vertex(av[1], v)) - mesh.point(mesh.clw_vertex(av[0], v)));
          vec = cross(t1, t2);
        } else {  // interior or dart
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

float project_point_neighb(const GMesh& mesh, const Point& p, Face& pf, Bary& ret_bary, Point& ret_clp, bool fast) {
  static const bool slow_project = getenv_bool("SLOW_PROJECT");
  assertw(!slow_project);
  if (slow_project) fast = false;
  bool pfsmooth = fast;
  for (Edge e : mesh.edges(pf)) {
    if (mesh.flags(e).flag(GMesh::eflag_sharp)) pfsmooth = false;
  }
  const float bnearedge = .08f;
  Set<Face> setfvis;
  Polygon poly;
  mesh.polygon(pf, poly);
  Bary minbary;
  float mind2 = project_point_triangle2(p, poly[0], poly[1], poly[2], minbary, ret_clp);
  float nearestedge = min(min(minbary[0], minbary[1]), minbary[2]);
  ASSERTX(nearestedge >= 0 && nearestedge < .34f);  // optional
  bool nearedge = nearestedge < bnearedge;
  bool projquick = pfsmooth && !nearedge;
  HH_SSTAT(Sprojquick, projquick);
  if (projquick) {
    ret_bary = minbary;
    return mind2;
  }
  int nvis = 1;
  setfvis.enter(pf);
  int ni = 0;
  for (;; ni++) {
    // Look at faces adjacent to vertices with minbary>bnearvertex
    Set<Face> setf;
    Array<Vertex> va;
    mesh.get_vertices(pf, va);
    assertx(va.num() == 3);
    for_int(j, 3) {
      if (pfsmooth && minbary[mod3(j + 1)] > bnearedge && minbary[mod3(j + 2)] > bnearedge) continue;
      for (Face f : mesh.faces(va[j])) {
        if (!setfvis.contains(f)) setf.add(f);
      }
    }
    struct S {
      Face f;
      float d2;
    };
    Array<S> ar;
    ar.reserve(setf.num());
    for (Face f : setf) {
      mesh.polygon(f, poly);
      float d2 = square(lb_dist_point_triangle(p, poly[0], poly[1], poly[2]));
      ar.push(S{f, d2});
    }
    nvis += ar.num();
    sort(ar, [](const S& s1, const S& s2) { return s1.d2 < s2.d2; });
    for (const auto& pa : ar) {
      if (pa.d2 >= mind2) break;
      Face f = pa.f;
      mesh.polygon(f, poly);
      Bary bary;
      Point clp;
      float d2 = project_point_triangle2(p, poly[0], poly[1], poly[2], bary, clp);
      if (d2 >= mind2) continue;
      pf = f;
      mind2 = d2;
      minbary = bary;
      ret_clp = clp;
      pfsmooth = fast;
      for (Edge e : mesh.edges(pf)) {
        if (mesh.flags(e).flag(GMesh::eflag_sharp)) pfsmooth = false;
      }
    }
    if (setfvis.contains(pf)) break;
    for (Face f : setf) setfvis.enter(f);
  }
  HH_SSTAT(Sprojnei, ni);
  HH_SSTAT(Sprojf, nvis);
  if (ni > 0) {
    HH_SSTAT(Sprojunexp, !nearedge);
  }
  ret_bary = minbary;
  return mind2;
}

}  // namespace hh
