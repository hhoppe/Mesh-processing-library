// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "progs/MeshToPm/EcolEval.h"

#include "libHh/GeomOp.h"  // aspect_ratio()
#include "libHh/Geometry.h"
#include "libHh/RangeOp.h"

namespace hh {

namespace {

// The edge (v1, v2) of a collapse, with face f1 = (v1, v2, vo1) and the face f2 = (v2, v1, vo2) or -1 on a
// boundary, in the notation of MeshSimplify.
struct EdgeContext {
  int v1, v2, vo1, vo2, f1, f2;
  [[nodiscard]] bool is_boundary() const { return f2 < 0; }
};

EdgeContext edge_context(const WorkMesh& mesh, EdgeHandle e) {
  EdgeContext ec;
  ec.f1 = e.f;
  ec.v1 = mesh.vertex(e.f, mod3(e.j + 1));
  ec.v2 = mesh.vertex(e.f, mod3(e.j + 2));
  ec.vo1 = mesh.vertex(e.f, e.j);
  const int fn = mesh._fnei[e.f].faces[e.j];
  ec.f2 = fn == AWMesh::k_undefined ? -1 : fn;
  ec.vo2 = -1;
  if (ec.f2 >= 0) for_int(j, 3) if (const int v = mesh.vertex(ec.f2, j); v != ec.v1 && v != ec.v2) ec.vo2 = v;
  return ec;
}

// Information about the mesh neighborhood following a speculative edge collapse, as in MeshSimplify.
struct NewMeshNei {
  InlinedArray<int, 24> va;                    // Ring vertices, ccw; va[0] is repeated at the end if closed.
  InlinedArray<Vec3<MCorner>, 24> ar_corners;  // 3 corners for each ring face (the third is at v1 or v2).
  InlinedArray<int, 24> ar_nwid;               // For each ring face, its new wedge (an index into ar_rwid_v*).
  InlinedArray<int, 4> ar_rwid_v1;             // For each new wedge, the would-be real wedge if v1 is kept.
  InlinedArray<int, 4> ar_rwid_v2;             // For each new wedge, the would-be real wedge if v2 is kept.
};

// Gather the ring of faces adjacent to v1 or v2 other than f1 and f2, ordered ccw around the would-be vertex.
// (Unlike MeshSimplify's gather_nn_1(), a closed ring starts at the first face of v1 after f1.)
void gather_ring(const WorkMesh& mesh, const EdgeContext& ec, NewMeshNei& nn) {
  InlinedArray<Vec3<MCorner>, 24> faces;
  for (const int u : {ec.v1, ec.v2}) {
    for (const int g : mesh.ccw_faces(u, ec.f1)) {
      if (g == ec.f1 || g == ec.f2) continue;
      const int ju = mesh.get_jvf(u, g);
      faces.push(V(MCorner{g, mod3(ju + 1)}, MCorner{g, mod3(ju + 2)}, MCorner{g, ju}));
    }
  }
  const int n = faces.num();
  assertx(n > 0);
  const auto from = [&](int i) { return mesh.corner_vertex(faces[i][0]); };
  const auto to = [&](int i) { return mesh.corner_vertex(faces[i][1]); };
  const auto find_from = [&](int v) {
    for_int(i, n) if (from(i) == v) return i;
    return -1;
  };
  int start = 0;  // For a closed ring, the first face of v1 after f1 (or the first face of v2 if v1 has none).
  for_int(i, n) {
    bool is_target = false;
    for_int(i2, n) is_target |= to(i2) == from(i);
    if (!is_target) start = i;  // The ring is open and starts here.
  }
  for (int i = start, k = 0; k < n; k++) {
    if (i < 0) assertnever("Ring of faces is not a chain");
    nn.ar_corners.push(faces[i]);
    nn.va.push(from(i));
    if (k == n - 1) {
      nn.va.push(to(i));
    } else {
      i = find_from(to(i));
    }
  }
}

// Returns false if the collapse would create wedges that the progressive mesh cannot encode, as MeshSimplify's
// gather_nn_valid() does.
bool gather_nn_valid(const WorkMesh& mesh, const EdgeContext& ec) {
  const auto wid = [&](MCorner c) { return mesh.corner_wedge(c); };
  const int vs = ec.v1, vt = ec.v2;
  const MCorner cvsfl = mesh.corner(vs, ec.f1), cvtfl = WorkMesh::ccw_face_corner(cvsfl);
  const MCorner cvsflo = mesh.ccw_corner(cvsfl);
  const MCorner cvtflo = mesh.clw_corner(cvtfl);
  const MCorner cvsfr = ec.f2 < 0 ? MCorner{} : mesh.clw_corner(cvsfl);
  const MCorner cvtfr = ec.f2 < 0 ? MCorner{} : mesh.ccw_corner(cvtfl);
  const MCorner cvsfro = !cvsfr ? MCorner{} : mesh.clw_corner(cvsfr);
  const MCorner cvtfro = !cvtfr ? MCorner{} : mesh.ccw_corner(cvtfr);
  const bool thru_sl = cvsflo && wid(cvsfl) == wid(cvsflo);
  const bool thru_sr = cvsfro && wid(cvsfr) == wid(cvsfro);
  const bool thru_tl = cvtflo && wid(cvtfl) == wid(cvtflo);
  const bool thru_tr = cvtfro && wid(cvtfr) == wid(cvtfro);
  const auto has_wedge_bnd = [&](int v) { return mesh.is_boundary_vertex(v) || mesh.vertex_has_multiple_wedges(v); };
  const bool vs_has_wedge_bnd = has_wedge_bnd(vs);
  const bool vt_has_wedge_bnd = has_wedge_bnd(vt);
  if (thru_sl && thru_sr && wid(cvsfl) == wid(cvsfr) && vs_has_wedge_bnd && vt_has_wedge_bnd) return false;
  if (thru_tl && thru_tr && wid(cvtfl) == wid(cvtfr) && vt_has_wedge_bnd && vs_has_wedge_bnd) return false;
  if ((thru_sl && thru_sr && wid(cvsfl) == wid(cvsfr) && wid(cvtfl) != wid(cvtfr)) ||
      (thru_tl && thru_tr && wid(cvtfl) == wid(cvtfr) && wid(cvsfl) != wid(cvsfr)))
    return false;  // The collapse would squash a dart.
  if (thru_sl && thru_sr && wid(cvsfl) != wid(cvsfr) && thru_tl && thru_tr && wid(cvtfl) == wid(cvtfr))
    assertnever("Edge collapse would squash sharp half-edge at vs");
  if (thru_tl && thru_tr && wid(cvtfl) != wid(cvtfr) && thru_sl && thru_sr && wid(cvsfl) == wid(cvsfr))
    assertnever("Edge collapse would squash sharp half-edge at vt");
  return true;
}

// Group the ring corners into new wedges, as MeshSimplify's gather_nn_2() does.  Returns false if the grouping
// would split a wedge.
bool gather_nn_2(const WorkMesh& mesh, const EdgeContext& ec, NewMeshNei& nn) {
  const auto wid = [&](MCorner c) { return mesh.corner_wedge(c); };
  const auto in_f1_or_f2 = [&](MCorner c) { return c.f == ec.f1 || c.f == ec.f2; };
  const int nf = nn.ar_corners.num();
  const bool closed = nn.va[0] == nn.va.last();
  for_int(i, nf) nn.ar_nwid.push(-1);
  for_int(irep, nf) {
    if (nn.ar_nwid[irep] >= 0) continue;
    const MCorner crep = nn.ar_corners[irep][2];
    const int w = nn.ar_rwid_v1.num();
    nn.ar_nwid[irep] = w;
    int rwid_v1 = wid(crep), rwid_v2 = wid(crep);
    const auto set_rwid = [&](MCorner c2) {
      const int vo = mesh.corner_vertex(c2);
      if (vo == ec.v1)
        rwid_v1 = wid(c2);
      else if (vo == ec.v2)
        rwid_v2 = wid(c2);
      else
        assertnever("");
    };
    for_int(dir, 2) {  // Two directions (ccw, clw).
      int i = irep;
      MCorner pc = crep;  // Previous corner.
      for (;;) {
        if (!dir) {
          if (++i >= nf) {
            if (closed) {
              i = 0;
            } else {
              const MCorner c = mesh.ccw_corner(pc);
              if (c && wid(c) == wid(pc)) {
                assertx(in_f1_or_f2(c));
                set_rwid(WorkMesh::clw_face_corner(c));
              }
              break;
            }
          }
        } else if (--i < 0) {
          if (closed) {
            i = nf - 1;
          } else {
            const MCorner c = mesh.clw_corner(pc);
            if (c && wid(c) == wid(pc)) {
              assertx(in_f1_or_f2(c));
              set_rwid(WorkMesh::ccw_face_corner(c));
            }
            break;
          }
        }
        const int cnwid = nn.ar_nwid[i];
        const MCorner c = nn.ar_corners[i][2];
        const MCorner c1 = !dir ? mesh.clw_corner(c) : mesh.ccw_corner(c);
        const MCorner pc1 = !dir ? mesh.ccw_corner(pc) : mesh.clw_corner(pc);
        assertx(c1 && pc1);
        assertx((c1 == pc) == (pc1 == c));
        if (c1 == pc) {  // Two faces adjacent on the same vertex.
          const bool same = wid(c) == wid(pc);
          if (cnwid >= 0 && same != (w == cnwid)) assertnever("Edge collapse would cause problem1");
          if (!same) break;
          if (cnwid >= 0) {
            assertx(cnwid == w);
            break;
          }
        } else {  // Two faces across from f1 or f2.
          if (!dir) assertx(WorkMesh::ccw_face_corner(c1) == pc1);
          if (dir) assertx(WorkMesh::clw_face_corner(c1) == pc1);
          const bool same = wid(pc) == wid(pc1) && wid(c) == wid(c1);
          if (!same) {
            if (wid(pc) == wid(pc1)) {
              assertx(in_f1_or_f2(c1));
              set_rwid(c1);
            }
            break;
          }
          if (cnwid >= 0) {
            assertx(cnwid == w);
            break;
          }
        }
        nn.ar_nwid[i] = w;
        if (mesh.corner_vertex(c) == ec.v1) rwid_v1 = wid(c);
        if (mesh.corner_vertex(c) == ec.v2) rwid_v2 = wid(c);
        pc = c;
      }
    }
    for (const int r : nn.ar_rwid_v1)
      if (r == rwid_v1 || r == rwid_v2) return false;
    for (const int r : nn.ar_rwid_v2)
      if (r == rwid_v1 || r == rwid_v2) return false;
    nn.ar_rwid_v1.push(rwid_v1);
    nn.ar_rwid_v2.push(rwid_v2);
  }
  return true;
}

// Returns the index of the new wedge of the ring corner c.
int find_nwid(const NewMeshNei& nn, MCorner c) {
  assertx(c);
  for_int(i, nn.ar_corners.num()) if (nn.ar_corners[i][2] == c) return nn.ar_nwid[i];
  assertnever("");
}

// The attributes of the new wedges when the vertex at position ii (2: v1, 0: v2) is kept, as MeshSimplify's
// update_initial_wi() computes them.
Array<PmWedgeAttrib> initial_wedge_attribs(const WorkMesh& mesh, const EdgeContext& ec, const NewMeshNei& nn, int ii) {
  assertx(ii == 0 || ii == 2);
  const auto attrib = [&](MCorner c) -> const PmWedgeAttrib& { return mesh._wedges[mesh.corner_wedge(c)].attrib; };
  const auto interp_wi = [&](MCorner c1, MCorner c2) -> const PmWedgeAttrib& {
    return ii == 2 ? attrib(c1) : attrib(c2);
  };
  Array<PmWedgeAttrib> ar_wi;
  for (const int rwid : nn.ar_rwid_v1) ar_wi.push(mesh._wedges[rwid].attrib);
  const auto wid = [&](MCorner c) { return mesh.corner_wedge(c); };
  constexpr int k_not_used = std::numeric_limits<int>::max();
  const MCorner cv1f1 = mesh.corner(ec.v1, ec.f1);
  const MCorner cv2f1 = WorkMesh::ccw_face_corner(cv1f1);
  const MCorner cv1f1o = mesh.ccw_corner(cv1f1);
  const MCorner cv2f1o = mesh.clw_corner(cv2f1);
  const int nwidv1f1o = !cv1f1o ? k_not_used : find_nwid(nn, cv1f1o);
  const int nwidv2f1o = !cv2f1o ? k_not_used : find_nwid(nn, cv2f1o);
  const bool cv1f1smooth = cv1f1o && wid(cv1f1o) == wid(cv1f1);
  const bool cv2f1smooth = cv2f1o && wid(cv2f1o) == wid(cv2f1);
  const MCorner cv1f2 = ec.f2 < 0 ? MCorner{} : mesh.corner(ec.v1, ec.f2);
  const MCorner cv2f2 = !cv1f2 ? MCorner{} : WorkMesh::clw_face_corner(cv1f2);
  const MCorner cv1f2o = !cv1f2 ? MCorner{} : mesh.clw_corner(cv1f2);
  const MCorner cv2f2o = !cv1f2 ? MCorner{} : mesh.ccw_corner(cv2f2);
  const int nwidv1f2o = !cv1f2o ? k_not_used : find_nwid(nn, cv1f2o);
  const int nwidv2f2o = !cv2f2o ? k_not_used : find_nwid(nn, cv2f2o);
  const bool cv1f2smooth = cv1f2o && wid(cv1f2o) == wid(cv1f2);
  const bool cv2f2smooth = cv2f2o && wid(cv2f2o) == wid(cv2f2);
  assertx(!((cv2f1smooth && cv2f2smooth && nwidv2f1o == nwidv2f2o && wid(cv1f1) != wid(cv1f2)) ||
            (cv1f1smooth && cv1f2smooth && nwidv1f1o == nwidv1f2o && wid(cv2f1) != wid(cv2f2))));
  if (cv1f1smooth && cv2f1smooth) assertx(nwidv1f1o == nwidv2f1o);
  if (cv1f1smooth)
    ar_wi[nwidv1f1o] = interp_wi(cv1f1, cv2f1);
  else if (cv2f1smooth)
    ar_wi[nwidv2f1o] = interp_wi(cv1f1, cv2f1);
  if (cv1f2smooth && cv2f2smooth) assertx(nwidv1f2o == nwidv2f2o);
  if (cv1f2smooth)
    ar_wi[nwidv1f2o] = interp_wi(cv1f2, cv2f2);
  else if (cv2f2smooth)
    ar_wi[nwidv2f2o] = interp_wi(cv1f2, cv2f2);
  return ar_wi;
}

// Returns true if the collapse satisfies Mesh::nice_edge_collapse(): the link condition and the exclusion of
// collapses that would leave a single face or a tetrahedron, or join two boundaries through an interior edge.
bool nice_edge_collapse(const WorkMesh& mesh, const EdgeContext& ec) {
  if (!ec.is_boundary() && mesh.is_boundary_vertex(ec.v1) && mesh.is_boundary_vertex(ec.v2)) return false;
  InlinedArray<int, 32> set;
  for (const auto [vv, ff] : mesh.ccw_vertices(ec.v1, ec.f1))
    if (vv != ec.vo1 && vv != ec.vo2) set.push(vv);
  for (const auto [vv, ff] : mesh.ccw_vertices(ec.v2, ec.f1)) {
    if (vv == ec.vo1 || vv == ec.vo2) continue;
    if (contains(set, vv)) return false;
    set.push(vv);
  }
  if (set.num() == 2 && ec.is_boundary()) return false;  // A single face.
  if (set.num() == 2 && !mesh.is_boundary_vertex(ec.v1) && !mesh.is_boundary_vertex(ec.v2)) return false;  // Tetra.
  return true;
}

// Returns false if the collapse would make a sharp edge not adjacent to the collapsed edge non-sharp, as MeshSimplify's
// gather_nn() does (for metrics other than the quadric one), since the progressive mesh could not encode it.
bool gather_nn_no_external_retire(const WorkMesh& mesh, const EdgeContext& ec, const NewMeshNei& nn) {
  const int nc = nn.ar_corners.num();
  const bool closed = nn.va[0] == nn.va.last();
  for_int(i, nc - !closed) {
    const int j = i < nc - 1 ? i + 1 : 0;  // Consider the edge between this ring face and the next one.
    const MCorner c0 = nn.ar_corners[i][1], cc0 = nn.ar_corners[i][2];
    const MCorner c1 = nn.ar_corners[j][0], cc1 = nn.ar_corners[j][2];
    const int vc0 = mesh.corner_vertex(cc0), vc1 = mesh.corner_vertex(cc1), vo = mesh.corner_vertex(c0);
    assertx(mesh.corner_vertex(c1) == vo);
    // The edges (vc0, vo) in face i and (vc1, vo) in face j.
    const bool expect_sharp = edge_sharp(mesh, EdgeHandle{cc0.f, nn.ar_corners[i][0].j}) ||
                              (vc1 != vc0 && edge_sharp(mesh, EdgeHandle{cc1.f, nn.ar_corners[j][1].j}));
    if (!expect_sharp) continue;
    const bool will_be_sharp = mesh._faces[cc0.f].attrib.matid != mesh._faces[cc1.f].attrib.matid ||
                               mesh.corner_wedge(c0) != mesh.corner_wedge(c1) || nn.ar_nwid[i] != nn.ar_nwid[j];
    if (will_be_sharp) continue;
    if (vo != ec.vo1 && vo != ec.vo2) return false;  // The collapse would retire an external edge.
  }
  return true;
}

// As MeshSimplify's intersect_segments().
std::optional<std::pair<float, float>> intersect_segments(const Uv& p1, const Uv& p2, const Uv& p3, const Uv& p4) {
  const double dx12 = double{p2[0]} - p1[0], dy12 = double{p2[1]} - p1[1];
  const double dx34 = double{p4[0]} - p3[0], dy34 = double{p4[1]} - p3[1];
  const double denominator = dx12 * dy34 - dy12 * dx34;
  if (denominator == 0.) return {};  // The segments are parallel or collinear.
  const double dx13 = double{p3[0]} - p1[0], dy13 = double{p3[1]} - p1[1];
  const double t12 = (dx13 * dy34 - dy13 * dx34) / denominator;
  const double t34 = (dx13 * dy12 - dy13 * dx12) / denominator;
  if (t12 < 0. || t12 > 1. || t34 < 0. || t34 > 1.) return {};  // The intersection is outside the segments.
  return std::pair{float(t12), float(t34)};
}

// Returns the face containing the half-edge (a, b), i.e., Mesh::face(a, b).
int face_with_hedge(const WorkMesh& mesh, int a, int b) {
  for (const int f : mesh.ccw_faces(a, mesh._someface[a]))
    if (mesh.vertex(f, mod3(mesh.get_jvf(a, f) + 1)) == b) return f;
  assertnever("");
}

// Returns the maximum displacement of any point on the mesh surface to its corresponding point with the same
// parametric (uv) coordinate after the collapse that keeps the vertex at position ii (2: v1, 0: v2), or -1. if the
// collapse is illegal, as MeshSimplify's evaluate_aps() computes it ("Appearance-preserving simplification" [Cohen
// et al 1998]).  The maximum occurs at the removed vertex or at an edge-edge crossing in the parametric domain.
double evaluate_aps(const WorkMesh& mesh, const EdgeContext& ec, int ii, float aps_bnd_fac) {
  assertx(ii == 0 || ii == 2);
  const int v1 = ii == 2 ? ec.v2 : ec.v1;  // The removed vertex.
  const int v2 = ii == 2 ? ec.v1 : ec.v2;  // The kept vertex.
  const int f1 = ec.f1, f2 = ec.f2;
  const auto point = [&](int v) -> const Point& { return mesh._vertices[v].attrib.point; };
  const auto uv = [&](MCorner c) -> const Uv& { return mesh._wedges[mesh.corner_wedge(c)].attrib.uv; };
  const auto matid = [&](int f) { return mesh._faces[f].attrib.matid; };
  const int wid_v1f1 = mesh.corner_wedge(mesh.corner(v1, f1));
  const int wid_v1f2 = f2 >= 0 ? mesh.corner_wedge(mesh.corner(v1, f2)) : -1;
  // The face (f1 or f2) whose wedge at v1 is that of corner c.
  const auto face_of_wedge = [&](MCorner c) {
    if (mesh.corner_wedge(c) == wid_v1f1) return f1;
    assertx(f2 >= 0 && mesh.corner_wedge(c) == wid_v1f2);
    return f2;
  };
  // The squared displacement at the removed vertex v1; also verify the parametric bijectivity.
  double max_mag2 = -1.;
  for (const int f : mesh.ccw_faces(v1, mesh._someface[v1])) {
    const MCorner c = mesh.corner(v1, f);
    const MCorner cs1 = WorkMesh::ccw_face_corner(c), cs2 = WorkMesh::clw_face_corner(c);
    const int vs1 = mesh.corner_vertex(cs1), vs2 = mesh.corner_vertex(cs2);
    if (vs1 == v2 || vs2 == v2) continue;  // The face becomes degenerate after the collapse.
    const int ft = face_of_wedge(c);
    assertx(matid(f) == matid(ft));  // (Necessary but not sufficient.)
    const Vec3<Uv> pt{uv(mesh.corner(v2, ft)), uv(cs1), uv(cs2)};
    if (signed_area(pt[0], pt[1], pt[2]) <= 0.f) return -1.;  // Parametric foldover (non-bijectivity).
    const Uv& pt1 = uv(c);
    const float b0 = signed_area(pt1, pt[1], pt[2]), b1 = signed_area(pt1, pt[2], pt[0]);
    const float b2 = signed_area(pt1, pt[0], pt[1]);
    if (b0 >= 0.f && b1 >= 0.f && b2 >= 0.f) {
      const float reciprocal = 1.f / (b0 + b1 + b2);
      const Bary bary(b0 * reciprocal, b1 * reciprocal, b2 * reciprocal);
      const Point v1_correspondence = interp(point(v2), point(vs1), point(vs2), bary);
      max_mag2 = max(max_mag2, dist2<double>(v1_correspondence, point(v1)));
    }
  }
  if (max_mag2 == -1.) return -1.;  // The vertex is inside no new face in the texture domain.
  // Also consider the displacements at all edge-edge crossings.
  InlinedArray<int, 32> verts;  // The neighbors v of v1 other than v2 with (v1, v) not sharp.
  for (const auto [v, f] : mesh.ccw_vertices(v1, mesh._someface[v1])) {
    if (v == v2) continue;
    const int j1 = mesh.get_jvf(v1, f);
    // The edge (v1, v) in face f is opposite the corner of f that is neither v1 nor v.
    const int jv = mesh.vertex(f, mod3(j1 + 1)) == v ? mod3(j1 + 1) : mod3(j1 + 2);
    if (!edge_sharp(mesh, EdgeHandle{f, 3 - j1 - jv})) verts.push(v);
  }
  for (const int v : verts) {
    const int fvv1 = face_with_hedge(mesh, v, v1);
    const Uv& p = uv(mesh.corner(v, fvv1));
    const Uv& p1 = uv(mesh.corner(v1, fvv1));
    const Point& pv = point(v);
    for (const int vv : verts) {
      if (vv == v) continue;
      const MCorner cvv = mesh.corner(v1, face_with_hedge(mesh, v1, vv));
      const int ft = face_of_wedge(cvv);
      const Uv& p2 = uv(mesh.corner(v2, ft));
      const Uv& pp = uv(mesh.corner(vv, face_with_hedge(mesh, vv, v1)));
      if (const auto result = intersect_segments(p1, p, p2, pp)) {
        const auto [t12, t34] = *result;
        const Point pp1 = interp(pv, point(v1), t12), pp2 = interp(point(vv), point(v2), t34);
        max_mag2 = max(max_mag2, dist2<double>(pp1, pp2));
      }
    }
  }
  double max_mag = std::sqrt(max_mag2);
  if (mesh.is_boundary_vertex(v1)) max_mag *= aps_bnd_fac;
  return max_mag;
}

constexpr float k_frac_diam = 1e-3f;     // As frac_diam in MeshSimplify.
constexpr float k_bad_dih = 1e28f;       // Cost penalty of a bad dihedral angle under "-dihallow".
constexpr float k_gmindih = -1.f / 3.f;  // Minimum acceptable dihedral angle cosine (tetrahedron angle).

// The hh99 quadric of a face (unweighted), in a form that is evaluated directly rather than as a matrix.
struct FaceQuadric {
  int nattrib{0};
  Vec3<double> nor;  // Unit normal, or zero if the face is degenerate.
  Vec3<double> p0;
  Vec<double, 6> s0;               // The attributes at p0.
  Vec<Vec3<double>, 6> gradients;  // The gradient of each attribute over the face plane.

  void set(CArrayView<float> c0, CArrayView<float> c1, CArrayView<float> c2) {
    nattrib = c0.num() - 3;
    assertx(nattrib >= 0 && nattrib <= 6 && c1.num() == c0.num() && c2.num() == c0.num());
    for_int(c, 3) p0[c] = c0[c];
    Vec3<double> e1, e2;
    for_int(c, 3) e1[c] = double(c1[c]) - c0[c], e2[c] = double(c2[c]) - c0[c];
    const Vec3<double> n = cross(e1, e2);
    const double n2 = mag2(n);
    if (!n2) {  // As in Qem::set_distance_hh99(), a degenerate face contributes nothing.
      nor = Vec3<double>(0., 0., 0.);
      nattrib = 0;
      return;
    }
    nor = n / std::sqrt(n2);
    // The gradient g satisfies dot(g, e1) = s1 - s0, dot(g, e2) = s2 - s0, and dot(g, n) = 0.
    const Vec3<double> b1 = cross(e2, n) / n2, b2 = cross(n, e1) / n2;
    for_int(k, nattrib) {
      s0[k] = c0[3 + k];
      gradients[k] = b1 * (double(c1[3 + k]) - c0[3 + k]) + b2 * (double(c2[3 + k]) - c0[3 + k]);
    }
  }

  [[nodiscard]] double evaluate(CArrayView<float> x) const {
    Vec3<double> dp;
    for_int(c, 3) dp[c] = x[c] - p0[c];
    double sum = square(dot(nor, dp));
    for_int(k, nattrib) sum += square(x[3 + k] - s0[k] - dot(gradients[k], dp));
    return sum;
  }
};

// The vector of a wedge in the space of the quadric: the position, then the scaled color and normal if used.
struct QemVector {
  Vec<float, 9> values;
  int num{0};
  // NOLINTNEXTLINE(google-explicit-constructor)
  operator CArrayView<float>() const { return CArrayView<float>(values.data(), num); }
};

QemVector qem_vector(const Point& p, const PmWedgeAttrib& wa, float cfac, float nfac) {
  QemVector v;
  for_int(c, 3) v.values[v.num++] = p[c];
  if (cfac) for_int(c, 3) v.values[v.num++] = wa.rgb[c] * cfac;
  if (nfac) for_int(c, 3) v.values[v.num++] = wa.normal[c] * nfac;
  return v;
}

// Returns the smallest dihedral angle cosine in the would-be neighborhood of newp with ring va (in which va[0] is
// repeated at the end if the ring is closed), as MeshSimplify's min_local_dihedral() computes it.
float min_local_dihedral(const WorkMesh& mesh, CArrayView<int> va, const Point& newp) {
  const int nw = va.num();
  assertx(nw > 1);
  const bool open = va[0] != va.last();
  const auto point = [&](int v) -> const Point& { return mesh._vertices[v].attrib.point; };
  // The unit normal of each face (newp, va[k], va[k + 1]), computed once with the operations of dihedral_angle_cos().
  InlinedArray<Vector, 32> normals(nw - 1);
  InlinedArray<bool, 32> normals_ok(nw - 1);
  for_int(k, nw - 1) {
    normals[k] = cross(newp, point(va[k]), point(va[k + 1]));
    normals_ok[k] = normals[k].normalize();
  }
  float min_dih = 10.f;
  for_intL(i, 1, nw - open) {
    const int im1 = i - 1;
    // As dihedral_angle_cos(newp, point(va[i]), point(va[ip1]), point(va[im1])) with ip1 = i + 1 (or 1 if i + 1 ==
    // nw, where the closed ring has va[nw - 1] == va[0], so the face is (newp, va[0], va[1])).
    const int k1 = i + 1 == nw ? 0 : i;
    const float dih = !normals_ok[k1] || !normals_ok[im1] ? -2.f : clamp(dot(normals[k1], normals[im1]), -1.f, 1.f);
    min_dih = min(min_dih, dih);
  }
  return min_dih;
}

float min_dihedral_about_vertex(const WorkMesh& mesh, int v) {
  InlinedArray<int, 24> va;
  for (const auto [vv, ff] : mesh.ccw_vertices(v, mesh._someface[v])) va.push(vv);
  if (!mesh.is_boundary_vertex(v)) va.push(va[0]);
  return min_local_dihedral(mesh, va, mesh._vertices[v].attrib.point);
}

// Returns true if the face is so thin (relative to its size) that it is degenerate, as in MeshSimplify's
// dihedral_penalty().
bool is_sliver(const Point& p0, const Point& p1, const Point& p2) {
  const Vec3<double> d0 = convert<double>(p0);
  const Vec3<double> v1 = convert<double>(p1) - d0, v2 = convert<double>(p2) - d0;
  const double max_edge2 = max({mag2(v1), mag2(v2), mag2(v2 - v1)});
  if (!max_edge2) return true;
  const float max_coord = max({max_abs_element(p0), max_abs_element(p1), max_abs_element(p2)});
  const double rounding_measure = 8. * std::numeric_limits<float>::epsilon() * max_coord / std::sqrt(max_edge2);
  return mag2(cross(v1, v2)) < square(max(1e-6, rounding_measure) * max_edge2);
}

}  // namespace

double hh99_face_distance2(CArrayView<float> p0, CArrayView<float> p1, CArrayView<float> p2, CArrayView<float> x) {
  FaceQuadric q;
  q.set(p0, p1, p2);
  return q.evaluate(x);
}

bool edge_sharp(const WorkMesh& mesh, EdgeHandle e) {
  const int f2 = mesh._fnei[e.f].faces[e.j];
  if (f2 == AWMesh::k_undefined) return true;
  if (mesh._faces[e.f].attrib.matid != mesh._faces[f2].attrib.matid) return true;
  for (const int jj : {mod3(e.j + 1), mod3(e.j + 2)}) {
    const int v = mesh.vertex(e.f, jj);
    if (mesh._faces[e.f].wedges[jj] != mesh.get_wvf(v, f2)) return true;
  }
  return false;
}

int vertex_num_sharp_edges(const WorkMesh& mesh, int v) {
  int n = mesh.is_boundary_vertex(v);  // The clw boundary edge of the most clw face.
  for (const int f : mesh.ccw_faces(v, mesh._someface[v]))
    n += edge_sharp(mesh, EdgeHandle{f, mod3(mesh.get_jvf(v, f) + 1)});
  return n;
}

InlinedArray<int, 8> charts_with_boundary_vertex(const WorkMesh& mesh, int v) {
  InlinedArray<int, 8> charts;
  for (const int f : mesh.ccw_faces(v, mesh._someface[v]))
    if (!contains(charts, mesh._faces[f].attrib.matid)) charts.push(mesh._faces[f].attrib.matid);
  if (charts.num() < 2 && !mesh.is_boundary_vertex(v)) charts.init(0);
  return charts;
}

bool strict_mat_neighbors(const WorkMesh& mesh, int f, const Vec3<int>& fnei) {
  assertx(f >= 0);
  const auto matid = [&](int ff) { return ff < 0 ? -1 : mesh._faces[ff].attrib.matid; };
  const int matfc = matid(f), matf0 = matid(fnei[0]), matf1 = matid(fnei[1]), matf2 = matid(fnei[2]);
  // If all neighbors differ, the face is a single chart or part of one adjacent chart but at a corner.
  if (matf0 != matf1 && matf0 != matf2 && matf1 != matf2) return true;
  // If all neighbors are the same, the face is not along any boundary.
  if (matf0 == matf1 && matf0 == matf2) {
    if (matf0 != -1) assertx(matfc == matf0);  // There cannot be an isolated triangle chart without 3 corners.
    return true;
  }
  // Two neighbors are the same, and one differs: the face must have the material of the two same neighbors.
  if (matf0 == matf1) return matfc == matf0;
  if (matf0 == matf2) return matfc == matf0;
  return matfc == matf1;
}

EcolResult evaluate_ecol(const WorkMesh& mesh, EdgeHandle e, const EcolOptions& options) {
  const EdgeContext ec = edge_context(mesh, e);
  if (!nice_edge_collapse(mesh, ec)) return {};
  if (options.maxvalence && mesh.degree(ec.v1) + mesh.degree(ec.v2) - (ec.is_boundary() ? 3 : 4) > options.maxvalence)
    return {};
  const bool e_sharp = edge_sharp(mesh, e);
  const int v1nse = vertex_num_sharp_edges(mesh, ec.v1), v2nse = vertex_num_sharp_edges(mesh, ec.v2);
  if (options.strict_sharp) {  // As in MeshSimplify's try_ecol_legal().
    if (!e_sharp) {
      if (v1nse >= 1 && v2nse >= 1) return {};  // The collapse would offend sharp edges.
    } else {
      if (v1nse >= 3 && v2nse >= 3) return {};
      for (const int f : {ec.f1, ec.f2}) {
        if (f < 0) continue;
        int jv1 = 0, jv2 = 0;  // The corners of v1 and v2 in f.
        for_int(j, 3) {
          if (mesh.vertex(f, j) == ec.v1) jv1 = j;
          if (mesh.vertex(f, j) == ec.v2) jv2 = j;
        }
        // The edges (v1, vs) and (v2, vs) are opposite the corners of v2 and v1.
        if (edge_sharp(mesh, EdgeHandle{f, jv2}) && edge_sharp(mesh, EdgeHandle{f, jv1})) return {};
      }
      if ((v1nse == 1 && v2nse != 2) || (v2nse == 1 && v1nse != 2)) return {};  // A dart would vanish.
    }
  }
  NewMeshNei nn;
  gather_ring(mesh, ec, nn);
  if (!gather_nn_valid(mesh, ec)) return {};
  if (!gather_nn_2(mesh, ec, nn)) return {};
  if (options.minaps && !gather_nn_no_external_retire(mesh, ec, nn)) return {};
  const auto point = [&](int v) -> const Point& { return mesh._vertices[v].attrib.point; };
  assertx(options.diam > 0.f);
  const float cfac = mesh._has_rgb ? options.colfac * options.diam : 0.f;
  const float nfac = options.norfac * options.diam;
  const double gcostnorm = 1. / square(double(options.diam));
  float dihb_cached = BIGFLOAT;  // The min dihedral before the collapse, computed only when needed.
  const auto dihb = [&] {
    if (dihb_cached == BIGFLOAT)
      dihb_cached = min(min_dihedral_about_vertex(mesh, ec.v1), min_dihedral_about_vertex(mesh, ec.v2));
    return dihb_cached;
  };
  // The face quadrics (with their weights) and the new wedge at which each is evaluated, as in gather_nn_qem().
  InlinedArray<std::pair<std::pair<FaceQuadric, double>, int>, 32> face_quadrics;
  const auto add_face_quadric = [&](int f, int nwid) {
    const Vec3<Point> triangle = mesh.face_points(f);
    Vec3<QemVector> vectors;
    for_int(j, 3) vectors[j] = qem_vector(triangle[j], mesh._wedges[mesh._faces[f].wedges[j]].attrib, cfac, nfac);
    FaceQuadric q;
    q.set(vectors[0], vectors[1], vectors[2]);
    const float face_area =
        mag(cross(triangle[0], triangle[1], triangle[2])) * (1.f / square(k_frac_diam * options.diam));
    face_quadrics.push(std::pair{std::pair{q, double(face_area)}, nwid});
  };
  InlinedArray<std::pair<Vec3<double>, double>, 8> sharp_planes;
  if (!options.minaps) {  // The quadric terms.
    for_int(i, nn.ar_corners.num()) add_face_quadric(nn.ar_corners[i][2].f, nn.ar_nwid[i]);
    for (const int f : {ec.f1, ec.f2}) {  // Try all neighboring corners with the same wedge to see if any survive.
      if (f < 0) continue;
      const int nwid = [&] {
        for (const int v : {ec.v1, ec.v2}) {
          const MCorner ci = mesh.corner(v, f);
          const int wid = mesh.corner_wedge(ci);
          for_int(dir, 2) {
            for (MCorner c = ci;;) {
              c = dir ? mesh.clw_corner(c) : mesh.ccw_corner(c);
              assertx(c != ci);
              if (!c || mesh.corner_wedge(c) != wid) break;
              for_int(i, nn.ar_corners.num()) if (nn.ar_corners[i][2] == c) return nn.ar_nwid[i];
            }
          }
        }
        return -1;
      }();
      if (nwid >= 0) add_face_quadric(f, nwid);  // Else the wedge is lost.
    }
    // The planes through the sharp edges adjacent to v1 or v2, weighted by neptfac, as in get_sharp_edge_qem().
    const float sqrt_neptfac = std::sqrt(options.neptfac);
    const auto add_sharp_plane = [&](EdgeHandle ee) {
      if (!edge_sharp(mesh, ee)) return;
      const Point& pa = point(mesh.vertex(ee.f, mod3(ee.j + 1)));
      const Point& pb = point(mesh.vertex(ee.f, mod3(ee.j + 2)));
      Vector nor{};
      for (const int f : {ee.f, mesh._fnei[ee.f].faces[ee.j]}) {
        if (f == AWMesh::k_undefined) continue;
        Vector fnor = get_normal_dir(mesh.face_points(f));
        assertw(fnor.normalize());
        nor += fnor;
      }
      nor = cross(nor, pb - pa);
      assertw(nor.normalize());
      nor *= dist(pa, pb) / (k_frac_diam * options.diam);
      nor *= sqrt_neptfac;
      const float d = -dot(pa, nor);
      sharp_planes.push(std::pair{convert<double>(nor), double(d)});
    };
    for (const int v : {ec.v1, ec.v2}) {
      for (const int f : mesh.ccw_faces(v, mesh._someface[v])) {
        const int jv = mesh.get_jvf(v, f);
        const EdgeHandle ee{f, mod3(jv + 1)};  // The edge (v, vertex(f, jv + 2)).
        if (!(v == ec.v2 && mesh.vertex(f, mod3(jv + 2)) == ec.v1)) add_sharp_plane(ee);
        const bool clw_boundary_edge = mesh._fnei[f].faces[mod3(jv + 2)] == AWMesh::k_undefined;
        if (clw_boundary_edge && !(v == ec.v2 && mesh.vertex(f, mod3(jv + 1)) == ec.v1))
          add_sharp_plane(EdgeHandle{f, mod3(jv + 2)});
      }
    }
  }
  double min_rssa = BIGFLOAT;
  int min_ii = -1;
  Array<PmWedgeAttrib> min_ar_wi;
  for (const int ii : {0, 2}) {
    // The "-minii2" restrictions of MeshSimplify, to avoid crease and corner migration.
    if (ec.is_boundary()) {
      if (ii != 2) continue;
      // Reject if the next edge ccw in f1, (v2, vo1), is also a boundary edge.
      if (mesh._fnei[ec.f1].faces[mod3(e.j + 1)] == AWMesh::k_undefined) continue;
    } else {
      if (mesh.is_boundary_vertex(ec.v1) && ii != 2) continue;
      if (mesh.is_boundary_vertex(ec.v2) && ii != 0) continue;
    }
    if (options.keepglobalv && ii != 2 && (mesh._vflags[ec.v1] & WorkMesh::k_vflag_global)) continue;
    if (options.keepglobalv && ii != 0 && (mesh._vflags[ec.v2] & WorkMesh::k_vflag_global)) continue;
    {
      // If only one vertex is on a sharp edge (or, for a sharp edge, is a corner), it is kept.
      const int min_nse = e_sharp ? 3 : 1;
      const bool v1_kept = v1nse >= min_nse, v2_kept = v2nse >= min_nse;
      if (v1_kept && !v2_kept && ii < 2) continue;
      if (v2_kept && !v1_kept && ii > 0) continue;
    }
    if (options.strict_sharp >= 2) {  // Chart-compliant, as in MeshSimplify's try_ecol().
      const int tv1 = ii == 2 ? ec.v1 : ec.v2, tv2 = ii == 2 ? ec.v2 : ec.v1;  // Here tv1 is kept and tv2 is lost.
      const auto matid = [&](int f) { return mesh._faces[f].attrib.matid; };
      const bool ok = [&] {
        if (options.min_chart_bnd_vertices) {
          assertx(options.chart_num_boundary_vertices);
          for (const int chart : charts_with_boundary_vertex(mesh, tv2))
            if ((*options.chart_num_boundary_vertices)[chart] <= options.min_chart_bnd_vertices) return false;
        }
        for (const int f : {ec.f1, ec.f2}) {
          if (f < 0) continue;
          const int vv = f == ec.f1 ? ec.vo1 : ec.vo2;
          const int fc = mesh.face_across(f, tv2, vv);
          if (fc < 0) return false;  // The edge (tv2, vv) is a boundary.
          assertx(matid(fc) == matid(f));
          int vvv = -1;  // The third vertex of face fc = (tv2, vv, vvv).
          for_int(j, 3) if (const int v = mesh.vertex(fc, j); v != tv2 && v != vv) vvv = v;
          const Vec3<int> fnei =
              V(mesh.face_across(fc, tv2, vvv), mesh.face_across(fc, vv, vvv), mesh.face_across(f, tv1, vv));
          if (!strict_mat_neighbors(mesh, fc, fnei)) return false;
        }
        InlinedArray<int, 16> v1mats;  // The materials adjacent to tv1.
        for (const int f : mesh.ccw_faces(tv1, mesh._someface[tv1]))
          if (!contains(v1mats, matid(f))) v1mats.push(matid(f));
        // Check whether any other vertex around tv2 is on the same chart boundary as tv1.
        for (const auto [v, ff] : mesh.ccw_vertices(tv2, mesh._someface[tv2])) {
          if (v == tv1 || v == ec.vo1 || v == ec.vo2) continue;
          // The faces on the two sides of edge (tv2, v).
          const int fa = ff, fb = mesh.face_across(ff, tv2, v);
          const int imat = matid(fa);
          if (fb >= 0 && matid(fb) == imat) {
            for (const int f : mesh.ccw_faces(v, ff))
              if (matid(f) != imat && contains(v1mats, matid(f))) return false;
          }
        }
        return true;
      }();
      if (!ok) continue;
    }
    Array<PmWedgeAttrib> ar_wi = initial_wedge_attribs(mesh, ec, nn, ii);
    const Point& newp = point(ii == 2 ? ec.v1 : ec.v2);
    float dihpenalty = 0.f;
    {  // As MeshSimplify's dihedral_penalty().
      bool bad = false;
      const float diha = min_local_dihedral(mesh, nn.va, newp);
      assertx(std::isfinite(diha));
      bad = diha < k_gmindih && diha < dihb();
      if (!bad)
        bad = ranges::any_of(nn.ar_corners, [&](const Vec3<MCorner>& corners) {
          const Point& p0 = point(mesh.corner_vertex(corners[0]));
          const Point& p1 = point(mesh.corner_vertex(corners[1]));
          return is_sliver(p0, p1, newp) && !is_sliver(p0, p1, point(mesh.corner_vertex(corners[2])));
        });
      if (bad) {
        if (!options.dihallow) continue;
        dihpenalty = k_bad_dih;
      }
    }
    double rssa = 0.;
    if (options.minaps) {
      rssa = evaluate_aps(mesh, ec, ii, options.aps_bnd_fac);
      if (rssa < 0.) continue;  // The collapse is illegal for the parameterization.
    }
    for (const auto& [q, nwid] : face_quadrics) {
      const double weight = q.second;
      rssa += weight * q.first.evaluate(qem_vector(newp, ar_wi[nwid], cfac, nfac));
    }
    for (const auto& [nor, d] : sharp_planes) rssa += square(dot(nor, convert<double>(newp)) + d);
    rssa = max(rssa, 0.);
    if (options.trishapeafac) {
      for_int(i, nn.va.num() - 1) {
        const Point& p0 = point(nn.va[i]);
        const Point& p1 = point(nn.va[i + 1]);
        rssa += options.trishapeafac * aspect_ratio(p0, p1, newp) * std::sqrt(area2(p0, p1, newp));
      }
    }
    if (options.bndfac) {
      if (ec.is_boundary()) {
        rssa /= options.bndfac;
        if (dihpenalty) {
          rssa *= std::sqrt(options.bndfac);
          dihpenalty = 0.f;
        }
      } else if (options.maxvalence && ((ec.vo1 >= 0 && mesh.degree(ec.vo1) > options.maxvalence / 2) ||
                                        (ec.vo2 >= 0 && mesh.degree(ec.vo2) > options.maxvalence / 2))) {
        rssa /= options.bndfac;
      }
    }
    rssa *= gcostnorm;
    assertx(rssa < k_bad_dih * .1f);
    rssa += dihpenalty;
    assertx(std::isfinite(float(rssa)));
    if (ranges::any_of(ar_wi, [](const PmWedgeAttrib& wa) { return is_zero(wa.normal); })) continue;
    if (rssa < min_rssa) {
      min_rssa = rssa;
      min_ii = ii;
      min_ar_wi = std::move(ar_wi);
    }
  }
  if (min_ii < 0) return {};
  EcolResult result;
  result.cost = max(float(min_rssa) + 1e-10f, 1e-20f);  // As offset_cost and smallcost in MeshSimplify.
  EcolConfig& config = result.config;
  const bool bswap = min_ii == 0;  // With "-minii2", the kept vertex becomes vs.
  config.vs = bswap ? ec.v2 : ec.v1;
  config.vt = bswap ? ec.v1 : ec.v2;
  config.vl = bswap ? ec.vo2 : ec.vo1;
  config.vr = bswap ? ec.vo1 : ec.vo2;
  config.fl = bswap ? ec.f2 : ec.f1;
  config.fr = bswap ? ec.f1 : ec.f2;
  const auto& ar_rwid = bswap ? nn.ar_rwid_v2 : nn.ar_rwid_v1;
  for_int(i, nn.ar_corners.num()) {
    const MCorner c = nn.ar_corners[i][2];
    config.corner_wedges.push(std::pair{3 * c.f + c.j, ar_rwid[nn.ar_nwid[i]]});
  }
  for_int(i, ar_rwid.num()) config.wedge_attribs.push(std::pair{ar_rwid[i], min_ar_wi[i]});
  return result;
}

}  // namespace hh
