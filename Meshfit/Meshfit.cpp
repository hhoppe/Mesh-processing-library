// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <cstring>  // strcmp() etc.

#include "libHh/A3dStream.h"
#include "libHh/Args.h"
#include "libHh/Array.h"
#include "libHh/Bbox.h"
#include "libHh/BinarySearch.h"
#include "libHh/Facedistance.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"  // dihedral_angle_cos()
#include "libHh/LLS.h"
#include "libHh/Map.h"
#include "libHh/MathOp.h"
#include "libHh/MeshOp.h"
#include "libHh/MeshSearch.h"
#include "libHh/NonlinearOptimization.h"
#include "libHh/Polygon.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Set.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

// To improve use in geometry image fitting:
//  - possibly switch from acos(dot()) to angvec(), for dih energy etc.
//  - add springs on both diagonals on quads?

// Notes:
//  Could check dihedral(starbar) instead of dihedral(star)
//   using GetEdgeRing() ->vertices

Array<const Point*> gather_vertex_ring(const GMesh& mesh, Vertex v) {
  Array<const Point*> wa;
  Vertex w = mesh.most_clw_vertex(v), wf = w;
  for (;;) {
    wa.push(&mesh.point(w));
    w = mesh.ccw_vertex(v, w);
    if (!w || w == wf) break;
  }
  if (w) wa.push(&mesh.point(w));
  return wa;
}

Array<const Point*> gather_edge_ring(const GMesh& mesh, Edge e) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  // current Mesh implementation boundary Edge direction
  if (mesh.is_boundary(e)) assertx(mesh.most_clw_vertex(v2) != v1);
  Vertex cv = v2;  // current vertex for rotation
  if (!mesh.is_boundary(e) && mesh.is_boundary(v1)) cv = v1;
  Vertex ov = mesh.opp_vertex(cv, e);  // other vertex of rotation (v1 or v2)
  Vertex w = mesh.most_clw_vertex(cv);
  if (w == ov) w = mesh.clw_vertex(cv, w), assertx(!mesh.is_boundary(cv));
  Vertex wf = assertx(w);
  Array<const Point*> wa;
  for (;;) {
    wa.push(&mesh.point(w));
    Vertex w2 = mesh.ccw_vertex(cv, w);
    if (w2 == ov) {
      ov = cv;
      cv = w2;
      w2 = mesh.ccw_vertex(cv, w);
    }
    w = w2;
    if (!w || w == wf) break;
  }
  if (w == wf) wa.push(&mesh.point(w));
  assertx(wa.num() >= (wa[0] == wa.last() ? 4 : 2));
  return wa;
}

float min_local_dihedral(CArrayView<const Point*> wa, const Point& newp) {
  int nw = wa.num();
  assertx(nw > 1);
  bool open = wa[0] != wa[nw - 1];
  float mindic = 2;
  for_intL(i, 1, nw - open) {
    int i1 = i + 1;
    if (i1 == nw) i1 = 1;
    float dic = dihedral_angle_cos(newp, *wa[i], *wa[i - 1], *wa[i1]);
    mindic = min(mindic, dic);
  }
  return mindic;
}

float min_dihedral_about_vertex(const GMesh& mesh, Vertex v) {
  Array<const Point*> wa = gather_vertex_ring(mesh, v);
  return min_local_dihedral(wa, mesh.point(v));
}

class DataPts {
 public:
  void enter(const Point& p) {
    co.push(p);
    cmf.push(nullptr);
    clp.add(1);
  }
  void clear() {
    co.clear();
    cmf.clear();
    clp.clear();
  }
  void ok() const;
  Array<Point> co;
  Array<Face> cmf;
  Array<Point> clp;  // clp[i] only defined if cmf[i]
};

using SetInt = Set<int>;
HH_SACABLE(SetInt);
HH_SAC_ALLOCATE_CD_FUNC(Mesh::MFace, SetInt, f_setpts);

GMesh mesh;
DataPts pt;

float spring = 0.f;
float spbf = 1.f;
float dihfac = 0.f;
float dihpower = 3.f;
int restrictfproject = 0;  // 0=never, 1=first_iter, 2=always
float fliter = 1.f;
float crep = 1e-5f;
float crbf = 3.f;
bool boundaryfixed = false;
bool nooutput = false;
int verb = 1;
float feswaasym = .01f;

WSA3dStream oa3d{std::cout};
const int sdebug = getenv_int("MESHFIT_DEBUG");  // 0, 1, or 2
Frame xform;                                     // original mesh + pts -> mesh + pts in unit cube
Frame xformi;                                    // inverse
enum EOperation { OP_ecol, OP_espl, OP_eswa, OP_NUM };
const Vec<string, OP_NUM> opname = {"ecol", "espl", "eswa"};
enum EResult { R_success, R_energy, R_dih, R_illegal, R_NUM };
const Vec<string, R_NUM> orname = {"success", "positive_energy", "bad_dihedral", "illegal_move"};

struct S_opstat {
  Vec<int, OP_NUM> na, ns;
  Vec<int, R_NUM> nor;
  int notswaps;
} opstat;

constexpr float k_gim_diagonal_factor = 1.0f;  // was 1.1f
constexpr bool k_simp96 = true;                // improvements
constexpr float k_mincos = -1.f / 3.f;         // acos(109.471) == tetrahedron angle
const Array<float> k_spring_sched = {1e-2f, 1e-3f, 1e-4f, 1e-8f};
constexpr int k_max_gfit_iter = 30;
unique_ptr<WFile> file_spawn;
float gdiam;
Bary dummy_bary;
bool have_quads = false;

struct hash_edge {
  size_t operator()(Edge e) const {
    return mesh.vertex_id(mesh.vertex1(e)) + intptr_t{mesh.vertex_id(mesh.vertex2(e))} * 76541;
  }
};
Set<Edge, hash_edge> ecand;  // Set of candidate edges in stoc; hash without pointers for portable random.

// *** DataPts

float project_point(const Point& p, Face f, Bary& ret_bary, Point& ret_clp);

void DataPts::ok() const {
  HH_DTIMER(__pt_ok);
  Polygon poly;
  for_int(i, co.num()) {
    Face f = assertx(cmf[i]);
    mesh.valid(f);
    assertx(f_setpts(f).contains(i));
    Point tclp;
    project_point(co[i], f, dummy_bary, tclp);
    assertx(dist2(tclp, clp[i]) < 1e-10f);
  }
}

// *** utility

void mesh_transform(const Frame& f) {
  for (Vertex v : mesh.vertices()) mesh.set_point(v, mesh.point(v) * f);
}

void compute_xform() {
  Bbox bbox;
  for_int(i, pt.co.num()) bbox.union_with(pt.co[i]);
  for (Vertex v : mesh.vertices()) bbox.union_with(mesh.point(v));
  gdiam = bbox.max_side();
  xform = bbox.get_frame_to_small_cube();
  if (verb >= 2) showdf("Applying xform: %s", FrameIO::create_string(xform, 1, 0.f).c_str());
  xformi = ~xform;
  for_int(i, pt.co.num()) pt.co[i] *= xform;
  mesh_transform(xform);
}

int get_nbv() {
  int nbv = 0;
  for (Vertex v : mesh.vertices()) {
    if (mesh.is_boundary(v)) nbv++;
  }
  return nbv;
}

float spring_energy(Vertex v1, Vertex v2) {
  return dist2(mesh.point(v1), mesh.point(v2)) * (mesh.is_boundary(mesh.edge(v1, v2)) ? spring * spbf : spring);
}

float spring_energy(Edge e) {
  return dist2(mesh.point(mesh.vertex1(e)), mesh.point(mesh.vertex2(e))) *
         (mesh.is_boundary(e) ? spring * spbf : spring);
}

Vertex edge_face_opp_vertex(Edge e, Face f) {
  if (mesh.is_triangle(f)) return mesh.opp_vertex(e, f);
  assertx(k_gim_diagonal_factor == 1.f);
  Vertex v1 = mesh.vertex1(e);
  Vertex v2 = mesh.vertex2(e);
  if (f == mesh.face2(e)) std::swap(v1, v2);
  assertx(mesh.ccw_vertex(f, v1) == v2);
  Vertex v2o = mesh.ccw_vertex(f, v2);
  Vertex v1o = mesh.clw_vertex(f, v1);
  // Ideally, test should be based on vertex ordering within face
  //  to break tie correctly if diagonal distances are exactly equal.
  return (dist2(mesh.point(v1), mesh.point(v2o)) < dist2(mesh.point(v2), mesh.point(v1o)) ? v2o : v1o);
}

float edge_dihedral_energy(const Point& p1, const Point& p2, const Point& ps1, const Point& ps2) {
  float angcos = dihedral_angle_cos(p1, p2, ps1, ps2);
  // ang is the unsigned angle away from planarity (range 0 .. TAU / 2)
  float ang = angcos < -1.f ? TAU / 2 : acos(angcos);
  HH_SSTAT(Sang, ang);
  return pow(ang, dihpower) * dihfac;
}

float edge_dihedral_energy(Edge e) {
  if (mesh.is_boundary(e)) return 0.f;
  Vertex vs1 = edge_face_opp_vertex(e, mesh.face1(e));
  Vertex vs2 = edge_face_opp_vertex(e, mesh.face2(e));
  return edge_dihedral_energy(mesh.point(mesh.vertex1(e)), mesh.point(mesh.vertex2(e)), mesh.point(vs1),
                              mesh.point(vs2));
}

double get_edis() {
  double edis = 0.;
  for_int(i, pt.co.num()) edis += dist2(pt.co[i], pt.clp[i]);
  return edis;
}

double get_espr() {
  if (spring == 0.f) return 0.f;
  double espr = 0.;
  for (Edge e : mesh.edges()) espr += spring_energy(e);
  return espr;
}

double get_edih() {
  if (dihfac == 0.f) return 0.f;
  double edih = 0.;
  for (Edge e : mesh.edges()) edih += edge_dihedral_energy(e);
  return edih;
}

double get_erep() { return crep * (mesh.num_vertices() + get_nbv() * (crbf - 1)); }

double show_energies(const string& s) {
  double edis = get_edis(), espr = get_espr(), edih = get_edih();
  double etot = edis + espr + edih;
  if (s != "") {
    showdf("%s F=%-12g S=%-12g D=%-12g T=%-12g\n", s.c_str(), edis, espr, edih, etot);
  }
  return etot;
}

void analyze_mesh(const string& s) {
  double edis = get_edis(), espr = get_espr(), edih = get_edih();
  double erep = get_erep();
  double etot = edis + espr + edih + erep;
  showdf("%-12s: mesh v=%d f=%d e=%d (nbv=%d)\n", s.c_str(), mesh.num_vertices(), mesh.num_faces(), mesh.num_edges(),
         get_nbv());
  showdf("  F=%g S=%g D=%g R=%g T=%g\n", edis, espr, edih, erep, etot);
  float drms = float(sqrt(edis / pt.co.num()) * xformi[0][0]), dmax2 = 0.f;
  for_int(i, pt.co.num()) dmax2 = max(dmax2, dist2(pt.co[i], pt.clp[i]));
  float dmax = my_sqrt(dmax2) * xformi[0][0];
  showdf("  distances: rms=%g (%.3f%%)  max=%g (%.3f%% of bbox)\n", drms, drms / gdiam * 100, dmax,
         dmax / gdiam * 100);
}

void push_face_points(Face f, Array<int>& ar_pts) {
  // Note: appends to existing ar_pts array.
  for (int pi : f_setpts(f)) ar_pts.push(pi);
}

void point_change_face(int i, Face newf) {
  Face oldf = pt.cmf[i];
  if (newf == oldf) return;
  if (oldf) assertx(f_setpts(oldf).remove(i));
  pt.cmf[i] = newf;
  if (newf) f_setpts(newf).enter(i);
}

// Make face's points project nowhere and remove from pt
void remove_face(Face f) {
  Set<int>& set = f_setpts(f);
  while (!set.empty()) point_change_face(set.get_one(), nullptr);
}

void mark_mesh() {
  for (Edge e : mesh.edges()) {
    mesh.set_string(e, mesh.flags(e).flag(GMesh::eflag_sharp) ? "sharp" : nullptr);
  }
}

// *** projection

// Note: for tri  face, bary[0] + bary[1] + bary[2] == 1.f
//       for quad face, bary[3](undefined) == 1.f - bary[0] - bary[1] - bary[2]

float project_point(const Point& p, Face f, Bary& ret_bary, Point& ret_clp) {
  Polygon poly;
  mesh.polygon(f, poly);
  if (poly.num() == 3) {
    return project_point_triangle2(p, poly[0], poly[1], poly[2], ret_bary, ret_clp);
  } else {
    assertx(poly.num() == 4);
    const bool other_diag = (dist2(poly[0], poly[2]) > dist2(poly[1], poly[3]) * square(k_gim_diagonal_factor));
    if (other_diag) {
      Point pp = poly[0];
      poly[0] = poly[1];
      poly[1] = poly[2];
      poly[2] = poly[3];
      poly[3] = pp;
    }
    Bary bary0;
    Point clp0;
    float d0 = project_point_triangle2(p, poly[0], poly[1], poly[2], bary0, clp0);
    Bary bary1;
    Point clp1;
    float d1 = project_point_triangle2(p, poly[0], poly[2], poly[3], bary1, clp1);
    if (d1 < d0) {
      d0 = d1;
      clp0 = clp1;
      bary0[0] = bary1[0];
      bary0[1] = 0.f;
      bary0[2] = bary1[1];
      float bary3 = bary1[2];
      assertx(abs(1.f - bary0[0] - bary0[1] - bary0[2] - bary3) < 1e-6f);
    } else {
      float bary3 = 0.f;
      assertx(abs(1.f - bary0[0] - bary0[1] - bary0[2] - bary3) < 1e-6f);
    }
    ret_bary = bary0;
    ret_clp = clp0;
    if (other_diag) {
      float bary3 = 1.f - bary0[0] - bary0[1] - bary0[2];
      ret_bary[0] = bary3;
      ret_bary[1] = bary0[0];
      ret_bary[2] = bary0[1];
    }
    return d0;
  }
}

float project_point_neighb(const Point& p, Face& cf, Bary& ret_bary, Point& ret_clp) {
  if (1 && !have_quads) {
    float d2 = project_point_neighb(mesh, p, cf, ret_bary, ret_clp, false);
    Polygon poly;
    mesh.polygon(cf, poly);
    assertx(poly.num() == 3);
    return d2;
  }
  // For now, slow method to re-project on all neighboring faces.
  Set<Face> setfvis;
  setfvis.enter(cf);
  float mind2 = project_point(p, cf, ret_bary, ret_clp);
  for (;;) {
    Face cf0 = cf;
    for (Vertex vv : mesh.vertices(cf)) {
      for (Face f : mesh.faces(vv)) {
        if (!setfvis.add(f)) continue;
        Bary bary;
        Point clp;
        float d2 = project_point(p, f, bary, clp);
        if (d2 < mind2) {
          mind2 = d2;
          ret_bary = bary;
          ret_clp = clp;
          cf = f;
        }
      }
    }
    if (cf == cf0) break;
  }
  return mind2;
}

void global_project_aux() {
  if (!have_quads) {
    MeshSearch msearch(&mesh, false);
    if (1) {
      Face hintf = nullptr;
      for_int(i, pt.co.num()) {
        Bary bary;
        Point clp;
        float d2;
        Face f = msearch.search(pt.co[i], hintf, bary, clp, d2);
        hintf = f;
        point_change_face(i, f);
        pt.clp[i] = clp;
      }
    } else {  // TODO: parallel; it crashes?
      // c:\hh\src\libhh\hh.cpp(143): hh::`anonymous namespace'::show_call_stack
      // c:\hh\src\libhh\hh.cpp(236): hh::`anonymous namespace'::my_top_level_exception_filter
      // c:\hh\src\libhh\spatial.h(190): hh::ObjectSpatial<hh::details::polygonface_approx_distance2,hh::details::polygonface_distance2>::add_cell
      // c:\hh\src\libhh\spatial.cpp(105): hh::BSpatialSearch::BSpatialSearch
      // c:\hh\src\libhh\meshsearch.cpp(126): hh::MeshSearch::search
      // c:\hh\src\meshfit\meshfit.cpp(365): `anonymous namespace'::global_project_aux$omp$1
      //  why does gfit not change geometry significantly?
      //  is it possible to remove FORCE_GLOBAL_PROJECT ?
      //  try fgfit?
      Array<Face> ar_face(pt.co.num());
      Array<Point> ar_clp(pt.co.num());
      parallel_for_each(range(pt.co.num()), [&](const int i) {
        Bary bary;
        float d2;
        Face hintf = pt.cmf[i];  // different semantics now
        ar_face[i] = msearch.search(pt.co[i], hintf, bary, ar_clp[i], d2);
      });
      for_int(i, pt.co.num()) {
        point_change_face(i, ar_face[i]);
        pt.clp[i] = ar_clp[i];
      }
    }
  } else {
    const int nv = mesh.num_vertices();
    Array<PolygonFace> ar_polyface;
    for (Face f : mesh.faces()) {
      if (mesh.is_triangle(f)) {
        Polygon poly(3);
        mesh.polygon(f, poly);
        ar_polyface.push(PolygonFace(std::move(poly), f));
      } else {
        Polygon poly(4);
        mesh.polygon(f, poly);
        assertx(poly.num() == 4);
        if (dist2(poly[0], poly[2]) > dist2(poly[1], poly[3]) * square(k_gim_diagonal_factor)) {
          rotate(poly, poly[1]);
        }
        ar_polyface.push(PolygonFace(Polygon(V(poly[0], poly[1], poly[2])), f));
        ar_polyface.push(PolygonFace(Polygon(V(poly[0], poly[2], poly[3])), f));
      }
    }
    PolygonFaceSpatial psp(nv < 10000 ? 15 : nv < 30000 ? 25 : 35);
    for (PolygonFace& polyface : ar_polyface) psp.enter(&polyface);
    for_int(i, pt.co.num()) {
      SpatialSearch<PolygonFace*> ss(&psp, pt.co[i]);
      PolygonFace* polyface = ss.next();
      Face f = polyface->face;
      point_change_face(i, f);
      project_point(pt.co[i], f, dummy_bary, pt.clp[i]);
    }
  }
}

void local_project_aux() {
  for_int(i, pt.co.num()) {
    Face cf = pt.cmf[i];
    if (restrictfproject) {
      project_point(pt.co[i], cf, dummy_bary, pt.clp[i]);
    } else {
      project_point_neighb(pt.co[i], cf, dummy_bary, pt.clp[i]);
    }
    point_change_face(i, cf);
  }
}

void global_project() {
  int nzcmf = 0;
  for_int(i, pt.co.num()) nzcmf += !pt.cmf[i];
  // Problem with FORCE_GLOBAL_PROJECT is that with fgfit, the first
  //  optimization iteration often pushes vertices out of the unit bbox.
  static const bool force_global_project = getenv_bool("FORCE_GLOBAL_PROJECT");
  if (nzcmf == pt.co.num() || force_global_project)
    global_project_aux();
  else if (!nzcmf)
    local_project_aux();
  else
    assertnever("");
}

void initial_projection() {
  {
    HH_TIMER(_initialproj);
    global_project();
  }
  assertw(!sdebug);
  if (sdebug) pt.ok();
  analyze_mesh("INITIAL");
}

// *** commands

void do_mfilename(Args& args) {
  HH_TIMER(_mfilename);
  assertx(!mesh.num_vertices());
  RFile is(args.get_filename());
  mesh.read(is());
  for (Face f : mesh.faces()) {
    int nv = mesh.num_vertices(f);
    assertx(nv <= 4);
    if (nv == 4) have_quads = true;
  }
  if (have_quads) Warning("Input mesh has quads; mesh assumed to be geometry image");
}

void do_filename(Args& args) {
  HH_TIMER(_filename);
  assertx(!pt.co.num());
  RFile is(args.get_filename());
  RSA3dStream ia3d(is());
  A3dElem el;
  for (;;) {
    ia3d.read(el);
    if (el.type() == A3dElem::EType::endfile) break;
    if (el.type() == A3dElem::EType::comment) continue;
    if (el.type() != A3dElem::EType::point) {
      Warning("Non-point input ignored");
      continue;
    }
    pt.enter(el[0].p);
  }
  showdf("%d points read)\n", pt.co.num());
}

void perhaps_initialize() {
  assertx(pt.co.num() && mesh.num_vertices());
  assertw(spring > 0);    // just warn user
  if (pt.cmf[0]) return;  // already initialized
  compute_xform();
  initial_projection();
}

void do_outlierdelete(Args& args) {
  float vdist = args.get_float();
  perhaps_initialize();
  Array<Point> ar_pt;
  for_int(i, pt.co.num()) {
    assertx(pt.cmf[i]);
    if (dist(pt.co[i], pt.clp[i]) <= vdist) ar_pt.push(pt.co[i]);
    point_change_face(i, nullptr);
  }
  showdf("Removing %d outlier points (dist>%g): #pts reduced from %d to %d\n", pt.co.num() - ar_pt.num(), vdist,
         pt.co.num(), ar_pt.num());
  pt.clear();
  for (const Point& p : ar_pt) pt.enter(p);
  initial_projection();
  // TODO: perform outlier removal during initial projection,
  //   where vdist is used to set the maximum spatial search radius.
}

void global_fit() {
  Map<Vertex, int> mvi;
  Array<Vertex> gva;
  for (Vertex v : mesh.vertices()) {
    mvi.enter(v, gva.num());
    gva.push(v);
  }
  int m = pt.co.num(), n = mesh.num_vertices();
  if (spring) m += mesh.num_edges();
  if (verb >= 2) showf("GlobalFit: about to solve a %dx%d LLS system\n", m, n);
  SparseLLS lls(m, n, 3);
  lls.set_max_iter(200);
  // Add point constraints
  Array<Vertex> va;
  for_int(i, pt.co.num()) {
    Face cmf = assertx(pt.cmf[i]);
    mesh.get_vertices(cmf, va);
    assertx(va.num() == 3);
    Bary bary;
    Point clp;
    project_point_triangle2(pt.co[i], mesh.point(va[0]), mesh.point(va[1]), mesh.point(va[2]), bary, clp);
    for_int(j, 3) lls.enter_a_rc(i, mvi.get(va[j]), bary[j]);
    lls.enter_b_r(i, pt.co[i]);
  }
  // Add spring constraints
  if (spring) {
    float sqrtit = sqrt(spring), sqrtbt = sqrt(spring * spbf);
    Vector vzero(0.f, 0.f, 0.f);
    int ri = pt.co.num();
    for (Edge e : mesh.edges()) {
      float sqrt_tension = mesh.is_boundary(e) ? sqrtbt : sqrtit;
      lls.enter_a_rc(ri, mvi.get(mesh.vertex1(e)), sqrt_tension);
      lls.enter_a_rc(ri, mvi.get(mesh.vertex2(e)), -sqrt_tension);
      lls.enter_b_r(ri, vzero);
      ri++;
    }
    assertx(ri == lls.num_rows());
  }
  // Suggest current solution
  for_int(i, n) lls.enter_xest_r(i, mesh.point(gva[i]));
  // Solve
  lls.solve();
  // Update solution
  for_int(i, n) {
    Point p;
    lls.get_x_r(i, p);
    mesh.set_point(gva[i], p);
  }
}

void do_gfit(Args& args) {
  perhaps_initialize();
  HH_TIMER(_gfit);
  int niter = args.get_int();
  assertw(!dihfac);
  if (verb >= 2) showdf("\n");
  if (verb >= 1) showdf("Beginning gfit, %d iterations, spr=%g\n", niter, spring);
  // constant simplicial complex
  double etot = show_energies(verb >= 2 ? "init   " : "");
  int i;
  for (i = 0; !niter || i < niter;) {
    if (!niter && i >= k_max_gfit_iter) break;
    i++;
    HH_STIMER(__gfit_iter);
    {
      HH_STIMER(__glls);
      global_fit();
    }
    {
      HH_STIMER(__gproject);
      global_project();
    }
    if (sdebug) {
      pt.ok();
      mesh.ok();
    }
    double oetot = etot;
    etot = show_energies(verb >= 3 ? sform("it%2d/%-2d", i, niter) : "");
    double echange = etot - oetot;
    assertw(echange < 0);
    if (!niter && echange > -1e-4) break;
  }
  if (verb >= 2) {
    showdf("Finished gfit, did %d iterations\n", i);
    show_energies("end    ");
    analyze_mesh("after_gfit");
  }
}

// *** fgfit

// Test using:
// Meshfit -mf ~/data/recon/new/cactus.m -fi ~/data/recon/new/cactus.3337.pts -spr 1e-2 -verb 3 -fgfit 60 >~/tmp/cactus.fgfit.m && G3dcmp ~/data/recon/new/cactus.m ~/tmp/cactus.fgfit.m -key DmDe
void do_fgfit(Args& args) {
  // Improve fit for constant simplicial complex.
  perhaps_initialize();
  HH_TIMER(_fgfit);
  int niter = args.get_int();
  assertx(pt.co.num() && mesh.num_vertices());
  if (verb >= 2) showdf("\n");
  if (verb >= 1) showdf("Beginning fgfit, %d iterations, spr=%g dihfac=%g\n", niter, spring, dihfac);
  // Evaluation objective for nonlinear optimization.
  struct FG {
    Map<Vertex, int> _mvi;  // vertex -> index in _x
    Array<Vertex> _iv;      // index -> mesh vertex
    Array<double> _x;       // linearized unknown vertex coordinates
    int _iter{0};
    int _niter;
    double _etot{0.};
    FG() {
      for (Vertex v : mesh.vertices()) {
        if (boundaryfixed && mesh.is_boundary(v)) continue;
        _mvi.enter(v, _iv.num());
        _iv.push(v);
      }
      _x.init(_iv.num() * 3);
      _etot = show_energies(verb >= 2 ? "init   " : "");
      pack_vertices();
    }
    void pack_vertices() {
      for_int(j, _iv.num()) {
        const Point& p = mesh.point(_iv[j]);
        for_int(c, 3) _x[j * 3 + c] = p[c];
      }
    }
    void unpack_vertices() const {
      bool vertices_moved = false;
      for_int(j, _iv.num()) {
        Point p;
        for_int(c, 3) p[c] = float(_x[j * 3 + c]);
        if (p != mesh.point(_iv[j])) {
          vertices_moved = true;
          mesh.set_point(_iv[j], p);
        }
      }
      if (vertices_moved) {
        HH_STIMER(__fgproject);
        global_project();
      }
    }
    double feval(ArrayView<double> ret_grad) {  // evaluate function and its gradient
      assertx(ret_grad.num() == _iv.num() * 3);
      HH_STIMER(__feval);
      // Only for the first iteration (because the mesh varies wildly),
      //  restrict each point to project onto same initial face.
      // This prevents bad folds as the mesh snaps back to place in the second iteration.
      if (restrictfproject == 1 && _iter >= 2) restrictfproject = 0;
      unpack_vertices();
      if (sdebug) {
        pt.ok();
        mesh.ok();
      }
      double prev_etot = _etot;
      _etot = show_energies(verb >= 3 ? sform("it%2d/%-2d", _iter, _niter) : "");
      double echange = _etot - prev_etot;
      assertw(echange < 0.);
      // Compute gradient.
      fill(ret_grad, 0.);
      // D edis
      Array<Vertex> va;
      for_int(i, pt.co.num()) {
        mesh.get_vertices(pt.cmf[i], va);
        assertx(va.num() <= 4);
        Bary bary;
        Point clp;
        project_point(pt.co[i], pt.cmf[i], bary, clp);
        Vector vtop = pt.co[i] - clp;
        for_int(k, va.num()) {
          float baryk = k < 3 ? bary[k] : 1.f - bary[0] - bary[1] - bary[2];
          Vector vd = vtop * (-2.f * baryk);
          bool present;
          int vi = _mvi.retrieve(va[k], present);
          if (!present) continue;
          for_int(c, 3) ret_grad[vi * 3 + c] += vd[c];
        }
      }
      // D espr
      if (spring) {
        for (Vertex v : mesh.vertices()) {
          bool present;
          int vi = _mvi.retrieve(v, present);
          if (!present) continue;
          for (Edge e : mesh.edges(v)) {
            Vertex vv = mesh.opp_vertex(v, e);
            Vector vtovv = mesh.point(vv) - mesh.point(v);
            float sp = mesh.is_boundary(e) ? spring * spbf : spring;
            Vector vd = vtovv * (-2 * sp);
            for_int(c, 3) ret_grad[vi * 3 + c] += vd[c];
          }
        }
      }
      assertx(!dihfac);
      _iter++;
      return _etot;
    }
  };
  FG fg;
  fg._niter = niter;
  if (getenv_string("NLOPT_DEBUG") == "") my_setenv("NLOPT_DEBUG", "1");
  auto func_eval = [&](ArrayView<double> ret_grad) { return fg.feval(ret_grad); };
  NonlinearOptimization<decltype(func_eval)> opt(fg._x, func_eval);
  assertx(niter > 0);
  opt.set_max_neval(niter + 1);
  assertw(opt.solve());
  if (0) fg.unpack_vertices();  // unnecessary because fg._x was the last state evaluated using FG::feval()
  if (verb >= 2) show_energies("end    ");
  if (verb >= 2) analyze_mesh("after_fgfit");
}

// *** lfit

// *** UPointLLS
// Solve a linear least squares problem involving a single point,
// in which the problem decomposes into 3 independent univariate LLS problems.
// The system is U * x = b,
// the design matrix U is a column vector with m rows
// (the m constraints coming from either the springs or the projected points).
// The unknown x is simply a scalar (actually, one scalar for each dimension).
// The vector b is also a column vector with m rows.
// To solve it using (Ut U) x=Ut b, note that Ut U is simply the norm of U,
// and Ut b is simply the dot product of U and b.
// To do this efficiently, Ut U and Ut b can be accumulated for all 3
// coordinates simultaneously, while traversing U and b row-by-row.
// To compute the rss (||U x - b||^2), Werner observed that
// rss= ||b||^2-||U x||^2 = ||b||^2 - x^2 * ||U||^2.
class UPointLLS {
 public:
  UPointLLS(Point& p) : _p(p) {}
  void enter_spring(const Point& pother, float sqrt_tension);
  // Constraint between point pdata and the point on triangle (p, p1, p2)
  // with barycentric coordinates (1-param1-param2, param1, param2)
  void enter_projection(const Point& pdata, const Point& p1, const Point& p2, float param1, float param2);
  void solve(double* prss0, double* prss1);  // updates point p!
 private:
  Point& _p;
  Vec3<double> _vUtU{0., 0., 0.};
  Vec3<double> _vUtb{0., 0., 0.};
  Vec3<double> _btb{0., 0., 0.};
  double _rss0{0.};
};

inline void UPointLLS::enter_spring(const Point& pother, float sqrt_tension) {
  for_int(c, 3) {
    double u = sqrt_tension;
    double b = double(pother[c]) * sqrt_tension;
    _vUtU[c] += u * u;
    _vUtb[c] += u * b;
    _btb[c] += b * b;
    _rss0 += square(u * _p[c] - b);
  }
}

inline void UPointLLS::enter_projection(const Point& pdata, const Point& p1, const Point& p2, float param1,
                                        float param2) {
  double u = 1.f - param1 - param2;
  double pa1 = param1, pa2 = param2;
  for_int(c, 3) {
    double b = pdata[c] - pa1 * p1[c] - pa2 * p2[c];
    _vUtU[c] += u * u;
    _vUtb[c] += u * b;
    _btb[c] += b * b;
    _rss0 += square(u * _p[c] - b);
  }
}

void UPointLLS::solve(double* prss0, double* prss1) {
  double rss1 = 0.;
  for_int(c, 3) {
    double newv = assertw(_vUtU[c]) ? _vUtb[c] / _vUtU[c] : _p[c];
    _p[c] = float(newv);
    double a = _btb[c] - _vUtU[c] * square(newv);
    assertw(a > -1e-8);
    if (a > 0) rss1 += a;
  }
  assertw(rss1 - _rss0 < 1e-13);
  if (prss0) *prss0 = _rss0;
  if (prss1) *prss1 = rss1;
}

void reproject_locally(CArrayView<int> ar_pts, CArrayView<Face> ar_faces) {
  int nf = ar_faces.num();
  Array<Bbox> ar_bbox(nf);
  for_int(i, nf) {
    Face f = ar_faces[i];
    Bbox& bbox = ar_bbox[i];
    for (Vertex v : mesh.vertices(f)) bbox.union_with(mesh.point(v));
  }
  Polygon poly;
  for (int pi : ar_pts) {
    const Point& p = pt.co[pi];
    static Array<float> ar_d2;
    ar_d2.init(nf);
    for_int(i, nf) ar_d2[i] = square(lb_dist_point_bbox(p, ar_bbox[i]));
    float mind2 = BIGFLOAT;
    Face minf = nullptr;
    for (;;) {
      int tmini = arg_min(ar_d2);
      float tmind2 = ar_d2[tmini];
      if (tmind2 == BIGFLOAT) break;  // ok, no more triangles to consider
      if (tmind2 >= mind2) break;
      ar_d2[tmini] = BIGFLOAT;
      Face f = ar_faces[tmini];
      mesh.polygon(f, poly);
      assertx(poly.num() == 3);
      Point clp;
      float d2 = project_point_triangle2(p, poly[0], poly[1], poly[2], dummy_bary, clp);
      if (d2 < mind2) {
        mind2 = d2;
        minf = f;
        pt.clp[pi] = clp;
      }
    }
    assertx(minf);
    point_change_face(pi, minf);
  }
}

// Fit a set of points to a ring of vertices while optimizing the center
// vertex.  Does niter iterations of projection + refit.
// Afterwards, should call reproject_locally() or equivalent to do final
// reprojection and update pt projections.
//  * Given:
// ar_pts: the set of point indices that project locally
// wa: array of vertex positions (wa[0] == wa[nw - 1] if closed loop)
// niter: number of iterations to do
// newp: the initial position to use for center vertex
//  * Return:
// newp: the final fitted position
// rss0: energy after first projection (true)
// rss1: energy after final refit, before final reprojection (over-estimate)
void local_fit(CArrayView<int> ar_pts, CArrayView<const Point*> wa, int niter, Point& newp, double& prss0,
               double& prss1) {
  int nw = wa.num();
  assertx(nw > 1 && niter > 0);  // at least one face
  bool closed = wa[0] == wa[nw - 1];
  float sqrtit = sqrt(spring), sqrtbt = sqrt(spring * spbf);
  double rss1;
  dummy_init(rss1);
  for_int(ni, niter) {
    static Array<Bbox> ar_bbox;
    ar_bbox.init(nw - 1);
    for_int(i, nw - 1) {
      Bbox& bbox = ar_bbox[i];
      bbox.clear();
      bbox.union_with(newp);
      bbox.union_with(*wa[i]);
      bbox.union_with(*wa[i + 1]);
    }
    UPointLLS ulls(newp);
    for (int pi : ar_pts) {
      // HH_SSTAT(SLFconsid, nw - 1);
      const Point& p = pt.co[pi];
      static Array<float> ar_d2;
      ar_d2.init(nw - 1);
      for_int(i, nw - 1) {
        // ar_d2[i] = square(lb_dist_point_triangle(p, newp, *wa[i], *wa[i + 1]));
        ar_d2[i] = square(lb_dist_point_bbox(p, ar_bbox[i]));
      }
      int nproj = 0;
      float mind2 = BIGFLOAT;
      int mini = -1;
      Bary minbary;
      dummy_init(minbary);
      for (;;) {
        int tmini = arg_min(ar_d2);
        float tmind2 = ar_d2[tmini];
        if (tmind2 == BIGFLOAT) break;  // ok, no more triangles to consider
        if (tmind2 >= mind2) break;
        ar_d2[tmini] = BIGFLOAT;
        Bary bary;
        Point dummy_clp;
        float d2 = project_point_triangle2(p, newp, *wa[tmini], *wa[tmini + 1], bary, dummy_clp);
        nproj++;
        if (d2 < mind2) {
          mind2 = d2;
          mini = tmini;
          minbary = bary;
        }
      }
      // HH_SSTAT(SLFproj, nproj);
      // Found closest face mini and corresponding minbary
      assertx(mini >= 0);
      ulls.enter_projection(p, *wa[mini], *wa[mini + 1], minbary[1], minbary[2]);
    }
    if (spring) {
      for_int(i, nw - closed) {
        bool is_be = !closed && (i == 0 || i == nw - 1);
        ulls.enter_spring(*wa[i], is_be ? sqrtbt : sqrtit);
      }
    }
    double rss0;
    ulls.solve(&rss0, &rss1);
    if (!ni) prss0 = rss0;
  }
  prss1 = rss1;
}

void fit_ring(Vertex v, int niter) {
  assertx(niter > 0);
  Array<int> ar_pts;
  Array<Face> ar_faces;
  Array<const Point*> wa = gather_vertex_ring(mesh, v);
  for (Face f : mesh.faces(v)) {
    ar_faces.push(f);
    for (int pi : f_setpts(f)) ar_pts.push(pi);
  }
  Point newp = mesh.point(v);
  float minb = min_local_dihedral(wa, newp);
  double rss0, rss1;
  local_fit(ar_pts, wa, niter, newp, rss0, rss1);
  float mina = min_local_dihedral(wa, newp);
  if (mina < k_mincos && mina < minb) return;  // change disallowed
  mesh.set_point(v, newp);
  reproject_locally(ar_pts, ar_faces);
}

void cleanup_neighborhood(Vertex v, int nri) {
  if (nri) {
    for (Vertex w : mesh.vertices(v)) fit_ring(w, nri);
    fit_ring(v, nri);
  }
}

void do_lfit(Args& args) {
  perhaps_initialize();
  HH_STIMER(_lfit);
  int ni = args.get_int();
  int nli = args.get_int();
  if (verb >= 2) showdf("\n");
  if (verb >= 1) showdf("Beginning lfit, %d iters (nli=%d), spr=%g\n", ni, nli, spring);
  for_int(i, ni) {
    for (Vertex v : mesh.vertices()) fit_ring(v, nli);
  }
  if (verb >= 2) showdf("Finished lfit\n");
  if (verb >= 2) analyze_mesh("after_lfit");
}

void do_four1split() {
  // Currently loses edge flags and face strings
  perhaps_initialize();
  HH_TIMER(_four1split);
  Array<Edge> are;
  for (Edge e : mesh.edges()) are.push(e);
  Array<Face> arf;
  for (Face f : mesh.faces()) arf.push(f);
  Map<Edge, Vertex> menewv;  // old Edge -> Vertex
  // Create new vertices and compute their positions
  for (Edge e : are) {
    Vertex v = mesh.create_vertex();
    menewv.enter(e, v);
    mesh.set_point(v, interp(mesh.point(mesh.vertex1(e)), mesh.point(mesh.vertex2(e))));
  }
  // Subdivide faces and update projections
  Array<Vertex> va;
  Vec3<Vertex> vs;
  for (Face f : arf) {
    mesh.get_vertices(f, va);
    assertx(va.num() == 3);
    for_int(i, 3) vs[i] = menewv.get(mesh.edge(va[i], va[mod3(i + 1)]));
    Array<int> ar_pts;
    push_face_points(f, ar_pts);
    Array<Face> ar_faces;
    for_int(i, 3) {
      Face ff = mesh.create_face(va[i], vs[i], vs[mod3(i + 2)]);
      ar_faces.push(ff);
    }
    {
      Face ff = mesh.create_face(vs[0], vs[1], vs[2]);
      ar_faces.push(ff);
    }
    reproject_locally(ar_pts, ar_faces);
    assertx(f_setpts(f).empty());
  }
  for (Face f : arf) mesh.destroy_face(f);
  if (sdebug) pt.ok(), assertx(mesh.is_nice());
}

void do_outmesh(Args& args) {
  perhaps_initialize();
  WFile os(args.get_filename());
  mesh_transform(xformi);
  mark_mesh();
  mesh.write(os());
  mesh_transform(xform);
}

void do_pclp() {
  nooutput = true;
  perhaps_initialize();
  A3dElem el;
  for_int(i, pt.co.num()) {
    el.init(A3dElem::EType::polyline);
    Point pco = pt.co[i], pclp = pt.clp[i];
    pco *= xformi;
    pclp *= xformi;
    el.push(A3dVertex(pco, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red())));
    el.push(A3dVertex(pclp, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red())));
    oa3d.write(el);
  }
  oa3d.flush();
}

void do_record() {
  // xform not undone!
  mesh.record_changes(&std::cout);
  nooutput = true;
}

void do_spawn(Args& args) {
  // xform not undone!
  file_spawn = make_unique<WFile>(args.get_filename());
  mesh.write((*file_spawn)());
  mesh.record_changes(&(*file_spawn)());
}

int get_vertex_normal(Vertex v, Vector& ret_nor) { return parse_key_vec(mesh.get_string(v), "normal", ret_nor); }

int get_vertex_uv(Vertex v, UV& ret_uv) { return parse_key_vec(mesh.get_string(v), "uv", ret_uv); }

// *** stoc

bool edge_sharp(Edge e) { return mesh.is_boundary(e) || mesh.flags(e).flag(GMesh::eflag_sharp); }

int vertex_num_sharp_edges(Vertex v) {
  int nsharpe = 0;
  for (Edge e : mesh.edges(v)) {
    if (edge_sharp(e)) nsharpe++;
  }
  return nsharpe;
}

EResult try_ecol(Edge e, int ni, int nri, float& edrss) {
  HH_STIMER(__try_ecol);
  if (!mesh.nice_edge_collapse(e)) return R_illegal;  // not a legal move
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  // Also see additional restrictions on minii below based on eflag_sharp
  if (!k_simp96) {
    // ignore sharp edges
  } else if (!edge_sharp(e)) {
    if (vertex_num_sharp_edges(v1) >= 1 && vertex_num_sharp_edges(v2) >= 1) {
      Warning("Edge collapse would offend sharp edges (a)");
      return R_illegal;
    }
  } else {
    if (vertex_num_sharp_edges(v1) >= 3 && vertex_num_sharp_edges(v2) >= 3) {
      Warning("Edge collapse would offend sharp edges (b)");
      return R_illegal;
    }
    for (Face f : mesh.faces(e)) {
      Vertex vs = mesh.opp_vertex(e, f);
      if (edge_sharp(mesh.edge(v1, vs)) && edge_sharp(mesh.edge(v2, vs))) {
        Warning("Edge collapse would offend sharp edges (c)");
        return R_illegal;
      }
    }
  }
  Array<const Point*> wa = gather_edge_ring(mesh, e);
  int nbvb = mesh.is_boundary(v1) + mesh.is_boundary(v2);
  float minb = min(min_dihedral_about_vertex(mesh, v1), min_dihedral_about_vertex(mesh, v2));
  double rssf = 0.;
  Array<int> ar_pts;
  Array<Face> ar_faces;
  for (Face f : mesh.faces(v1)) {
    push_face_points(f, ar_pts);
    if (f == f1 || f == f2) continue;
    ar_faces.push(f);
  }
  for (Face f : mesh.faces(v2)) {
    if (f == f1 || f == f2) continue;
    push_face_points(f, ar_pts);
    ar_faces.push(f);
  }
  for (int pi : ar_pts) rssf += dist2(pt.co[pi], pt.clp[pi]);
  for (Vertex v : mesh.vertices(v1)) rssf += spring_energy(v1, v);
  for (Vertex v : mesh.vertices(v2)) {
    if (v != v1) rssf += spring_energy(v2, v);
  }
  // Find the best starting location by exploring one iteration.
  double minrss1 = BIGFLOAT;
  int minii = -1;
  Point minp;
  for_int(ii, 3) {
    if (k_simp96) {
      // Added these tests to prevent boundary/crease migration inwards
      //  and to prevent corner migration
      if (!edge_sharp(e)) {
        if (vertex_num_sharp_edges(v1) >= 1 && ii < 2) continue;
        if (vertex_num_sharp_edges(v2) >= 1 && ii > 0) continue;
      } else {
        if (vertex_num_sharp_edges(v1) >= 3 && ii < 2) continue;
        if (vertex_num_sharp_edges(v2) >= 3 && ii > 0) continue;
      }
    }
    Point newp = interp(mesh.point(v1), mesh.point(v2), ii * .5f);
    if (k_simp96) {
      float mina = min_local_dihedral(wa, newp);
      if (mina < k_mincos && mina < minb) continue;  // change disallowed
    }
    double rss0, rss1;
    local_fit(ar_pts, wa, 1, newp, rss0, rss1);
    {
      float mina = min_local_dihedral(wa, newp);
      if (mina < k_mincos && mina < minb) continue;  // change disallowed
    }
    if (rss1 < minrss1) {
      minrss1 = rss1;
      minii = ii;
      minp = newp;
    }
  }
  if (minii < 0) return R_dih;  // no dihedrally admissible configuration
  // Then, explore ni iterations from that chosen starting point
  float w1 = minii * .5f;
  if (ni) {
    double rss0;
    local_fit(ar_pts, wa, ni, minp, rss0, minrss1);
    float mina = min_local_dihedral(wa, minp);
    if (mina < k_mincos && mina < minb) return R_dih;  // change disallowed
  }
  double drss = minrss1 - rssf - (nbvb == 2 ? crbf : 1) * double(crep);
  edrss = float(drss);
  if (verb >= 4) SHOW("ecol:", rssf, minrss1, drss);
  if (drss >= 0) return R_energy;  // energy function does not decrease
  // ALL SYSTEMS GO
  HH_SSTAT(Sminii, minii == 1);
  HH_STIMER(__doecol);
  if (k_simp96) {
    Vector nor1, nor2, nnor;
    UV uv1, uv2, uvn;
    string str;
    if (get_vertex_normal(v1, nor1) && get_vertex_normal(v2, nor2)) {
      nnor = w1 * nor1 + (1.f - w1) * nor2;
      assertx(nnor.normalize());
      mesh.update_string(v1, "normal", csform_vec(str, nnor));
    }
    if (get_vertex_uv(v1, uv1) && get_vertex_uv(v2, uv2)) {
      uvn[0] = w1 * uv1[0] + (1.f - w1) * uv2[0];
      uvn[1] = w1 * uv1[1] + (1.f - w1) * uv2[1];
      mesh.update_string(v1, "uv", csform_vec(str, uvn));
    }
  }
  for (Vertex v : mesh.vertices(e)) {
    for (Edge ee : mesh.edges(v)) ecand.remove(ee);
  }
  remove_face(f1);
  if (f2) remove_face(f2);
  mesh.collapse_edge(e);  // v1 kept
  // add about 12 to 16 edges
  for (Edge ee : mesh.edges(v1)) ecand.add(ee);
  for (Face f : mesh.faces(v1)) ecand.add(mesh.opp_edge(v1, f));
  if (sdebug >= 2) assertx(mesh.is_nice());
  mesh.set_point(v1, minp);
  reproject_locally(ar_pts, ar_faces);
  cleanup_neighborhood(v1, nri);
  return R_success;
}

EResult try_espl(Edge e, int ni, int nri, float& edrss) {
  HH_STIMER(__try_espl);
  // always legal
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Vertex vo1 = mesh.side_vertex1(e), vo2 = mesh.side_vertex2(e);
  Array<const Point*> wa{&mesh.point(v2), &mesh.point(vo1), &mesh.point(v1)};
  if (vo2) wa.push_array(V(&mesh.point(vo2), &mesh.point(v2)));
  float minb = vo2 ? edge_dihedral_angle_cos(mesh, e) : 2;
  double rssf = spring_energy(e);
  Array<int> ar_pts;
  for (Face f : mesh.faces(e)) push_face_points(f, ar_pts);
  for (int pi : ar_pts) rssf += dist2(pt.co[pi], pt.clp[pi]);
  Point newp = interp(*wa[0], *wa[2]);
  double rss0, rss1;
  local_fit(ar_pts, wa, (ni ? ni : 1), newp, rss0, rss1);
  float mina = min_local_dihedral(wa, newp);
  if (mina < k_mincos && mina < minb) return R_dih;  // change disallowed
  double drss = rss1 - rssf + (vo2 ? 1.f : crbf) * double(crep);
  edrss = float(drss);
  if (verb >= 4) SHOW("espl:", rssf, rss1, drss);
  if (drss >= 0) return R_energy;  // energy function does not decrease
  // ALL SYSTEMS GO
  HH_STIMER(__doespl);
  for (Face f : mesh.faces(e)) {
    for (Edge ee : mesh.edges(f)) {  // one duplication
      ecand.remove(ee);
    }
  }
  Vertex v = mesh.split_edge(e);
  mesh.set_point(v, newp);
  // add 8 edges (5 if boundary)
  for (Face f : mesh.faces(v)) {
    for (Edge ee : mesh.edges(f)) {  // four duplications
      ecand.add(ee);
    }
  }
  // Since ar_pts project onto f1 + f2 (which are still there), it is easy to update the projections:
  fit_ring(v, 2);
  cleanup_neighborhood(v, nri);
  return R_success;
}

// Note: correspondences on Edge e such as vertex1(e) == v1 may fail here!
// Try swapping edge (v1, v2) into edge (vo1, vo2), allowing vertex vo1 to move.
// To do this, gather points in current ring of vo1 plus points on f2.
EResult check_half_eswa(Edge e, Vertex vo1, Vertex v1, Vertex vo2, Vertex v2, Face f2, Face f1, int ni, int nri,
                        float& edrss) {
  Array<const Point*> wa;
  {
    assertx(mesh.ccw_vertex(vo1, v1) == v2);
    Vertex w = mesh.most_clw_vertex(vo1), wf = w;
    for (;;) {
      wa.push(&mesh.point(w));
      if (w == v1) wa.push(&mesh.point(vo2));
      w = mesh.ccw_vertex(vo1, w);
      if (!w || w == wf) break;
    }
    if (w) wa.push(&mesh.point(w));
  }
  Array<int> ar_pts;
  Array<Face> ar_faces;
  for (Face f : mesh.faces(vo1)) {
    if (f == f1) continue;
    assertx(f != f2);
    ar_faces.push(f);
    push_face_points(f, ar_pts);
  }
  push_face_points(f1, ar_pts);
  push_face_points(f2, ar_pts);
  double rssf = 0.;
  for (int pi : ar_pts) rssf += dist2(pt.co[pi], pt.clp[pi]);
  for (Vertex v : mesh.vertices(vo1)) rssf += spring_energy(vo1, v);
  rssf += spring_energy(e);
  Point newp = mesh.point(vo1);
  float minb = min(min_dihedral_about_vertex(mesh, vo1), edge_dihedral_angle_cos(mesh, e));
  double rss0, rss1;
  local_fit(ar_pts, wa, (ni ? ni : 1), newp, rss0, rss1);
  float mina = min_local_dihedral(wa, newp);
  if (mina < k_mincos && mina < minb) return R_dih;  // change disallowed
  double drss = rss1 - rssf + crep * feswaasym;
  edrss = float(drss);
  if (verb >= 4) SHOW("eswa:", rssf, rss1, drss);
  if (drss > 0) return R_energy;
  const char* finfo1 = mesh.get_string(f1);
  const char* finfo2 = mesh.get_string(f2);
  if (!k_simp96) {
    // ignore face strings
  } else if (!finfo1 && !finfo2) {
    // ok, no face info to worry about
  } else if (finfo1 && finfo2 && !strcmp(finfo1, finfo2)) {
    // ok, they match
  } else {
    Warning("Edge swap would lose face info");
    return R_illegal;
  }
  // ALL SYSTEMS GO
  HH_STIMER(__doeswa);
  ecand.remove(e);
  remove_face(f1);
  remove_face(f2);
  Edge enew = assertx(mesh.swap_edge(e));
  mesh.set_point(vo1, newp);
  // add about 9 edges
  ecand.add(mesh.edge(vo2, v1));
  ecand.add(mesh.edge(vo2, v2));
  for (Edge ee : mesh.edges(vo1)) ecand.add(ee);
  for (Face f : mesh.faces(enew)) ar_faces.push(f);
  reproject_locally(ar_pts, ar_faces);
  if (nri) {
    fit_ring(vo2, nri);
    fit_ring(v1, nri);
    fit_ring(v2, nri);
    fit_ring(vo1, nri);
    cleanup_neighborhood(vo1, nri);
    fit_ring(vo2, nri);
    fit_ring(v1, nri);
    fit_ring(v2, nri);
    fit_ring(vo1, nri);
  }
  return R_success;
}

EResult try_eswa(Edge e, int ni, int nri, float& edrss) {
  HH_STIMER(__try_eswa);
  if (!mesh.legal_edge_swap(e)) return R_illegal;  // not legal move
  if (k_simp96 && edge_sharp(e)) {
    Warning("Not swapping sharp edges");
    return R_illegal;
  }
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  Vertex vo1 = mesh.side_vertex1(e), vo2 = mesh.side_vertex2(e);
  // Compare angles immediately before and after swap
  float minb = edge_dihedral_angle_cos(mesh, e);
  float mina = dihedral_angle_cos(mesh.point(vo1), mesh.point(vo2), mesh.point(v1), mesh.point(v2));
  if (mina < k_mincos && mina < minb) return R_dih;
  // Will try both cases, but randomly select which one to try first
  EResult result;
  if (Random::G.get_unsigned(2)) {
    result = check_half_eswa(e, vo1, v1, vo2, v2, f2, f1, ni, nri, edrss);
    if (result == R_success) return result;
    result = check_half_eswa(e, vo2, v2, vo1, v1, f1, f2, ni, nri, edrss);
  } else {
    result = check_half_eswa(e, vo2, v2, vo1, v1, f1, f2, ni, nri, edrss);
    if (result == R_success) return result;
    result = check_half_eswa(e, vo1, v1, vo2, v2, f2, f1, ni, nri, edrss);
  }
  return result;
}

EResult try_op(Edge e, EOperation op, float& edrss) {
  HH_STIMER(__try_op);
  EResult result;
  result = (op == OP_ecol
                ? try_ecol(e, int(4.f * fliter + .5f), int(2.f * fliter + .5f), edrss)
                : op == OP_espl ? try_espl(e, int(3.f * fliter + .5f), int(4.f * fliter + .5f), edrss)
                                : op == OP_eswa ? try_eswa(e, int(3.f * fliter + .5f), int(2.f * fliter + .5f), edrss)
                                                : (assertnever(""), R_success));
  opstat.na[op]++;
  if (result == R_success) opstat.ns[op]++;
  opstat.nor[result]++;
  return result;
}

void do_stoc() {
  perhaps_initialize();
  HH_STIMER(_stoc);
  assertx(!dihfac);
  if (verb >= 2) showdf("\n");
  if (verb >= 1) showdf("Beginning stoc, spring=%g, fliter=%g\n", spring, fliter);
  fill(opstat.na, 0);
  fill(opstat.ns, 0);
  fill(opstat.nor, 0);
  opstat.notswaps = 0;
  int ni = 0, nbad = 0, lecol = 0, lespl = 0, leswa = 0;
  for (Edge e : mesh.edges()) ecand.enter(e);
  while (!ecand.empty()) {
    ni++;
    Edge e = ecand.remove_random(Random::G);
    ASSERTX(mesh.valid(e));
    EOperation op;
    op = OP_ecol;  // dummy_init(op)
    EResult result = R_illegal;
    float edrss;
    dummy_init(edrss);
    if (result != R_success) {
      op = OP_ecol;
      result = try_op(e, op, edrss);
    }
    // do not try edge_splits under zippysimplify
    if (result != R_success && fliter) {
      op = OP_espl;
      result = try_op(e, op, edrss);
    }
    if (result != R_success) {
      op = OP_eswa;
      result = try_op(e, op, edrss);
    }
    if (verb >= 2 && ni % 100 == 0) {
      showdf("it %5d, ecol=%2d  espl=%2d  eswa=%2d   [%5d/%-5d]\n", ni, opstat.ns[OP_ecol] - lecol,
             opstat.ns[OP_espl] - lespl, opstat.ns[OP_eswa] - leswa, ecand.num(), mesh.num_edges());
      lecol = opstat.ns[OP_ecol];
      lespl = opstat.ns[OP_espl];
      leswa = opstat.ns[OP_eswa];
    }
    if (result != R_success) {
      nbad++;
      continue;
    }
    if (verb >= 3)
      showf("it %5d, %s (after %3d) [%5d/%-5d] edrss=%e\n", ni, opname[op].c_str(), nbad, ecand.num(),
            mesh.num_edges(), edrss);
    if (file_spawn) (*file_spawn)().flush();
    nbad = 0;
  }
  if (verb >= 2) showdf("it %d, last search: %d wasted attempts\n", ni, nbad);
  const int nat = narrow_cast<int>(sum(opstat.na));
  const int nst = narrow_cast<int>(sum(opstat.ns));
  if (verb >= 2) {
    showdf("%s(ecol=%d/%d, espl=%d/%d eswa=%d/%d tot=%d/%d)\n", "Endstoc: ", opstat.ns[OP_ecol], opstat.na[OP_ecol],
           opstat.ns[OP_espl], opstat.na[OP_espl], opstat.ns[OP_eswa], opstat.na[OP_eswa], nst, nat);
    showdf("         (otswaps=%d)\n", opstat.notswaps);
    showdf("Result of %d attempted operations:\n", nat);
    for_int(i, opstat.nor.num()) showdf("  %5d %s\n", opstat.nor[i], orname[i].c_str());
    analyze_mesh("after_stoc");
  }
}

// *** other commands

void apply_schedule() {
  while (spring > k_spring_sched[0]) {
    do_lfit(as_lvalue(Args{"2", "3"}));  // -lfit 2 3
    do_stoc();                           // -stoc
    do_lfit(as_lvalue(Args{"2", "3"}));  // -lfit 2 3
    spring *= .1f;                       // -spring f
  }
  for (float spr : k_spring_sched) {
    spring = spr;                        // -spring f
    do_lfit(as_lvalue(Args{"2", "3"}));  // -lfit 2 3
    do_stoc();                           // -stoc
    do_lfit(as_lvalue(Args{"2", "3"}));  // -lfit 2 3
  }
}

void do_reconstruct() {
  HH_TIMER(reconstruct);
  if (!spring) spring = k_spring_sched[0];
  perhaps_initialize();
  do_fgfit(as_lvalue(Args{"100"}));  // was "20"
  do_gfit(as_lvalue(Args{"30"}));    // -gfit 30
  apply_schedule();
}

void do_simplify() {
  HH_TIMER(simplify);
  if (!spring) spring = k_spring_sched[0];
  perhaps_initialize();
  apply_schedule();
}

void do_quicksimplify() {
  HH_TIMER(quicksimplify);
  float gfliter = fliter;
  const Array<float> k_spring_sched2 = {1e-2f, 1e-4f};
  spring = k_spring_sched2[0];
  perhaps_initialize();
  for_int(i, k_spring_sched2.num()) {
    spring = k_spring_sched2[i];         // -spring f
    do_lfit(as_lvalue(Args{"1", "3"}));  // -lfit 1 3
    fliter = gfliter * (i + 1 < k_spring_sched2.num() ? .50f : .25f);
    do_stoc();
    do_lfit(as_lvalue(Args{"1", "3"}));  // -lfit 1 3
  }
}

void do_zippysimplify() {
  HH_TIMER(zippysimplify);
  // Note: I think I prefer the 1e-2f, 1e-4f schedule because it biases
  //  triangulations of planar regions to avoid long skinny triangles,
  //  as opposed to ending with no spring energy at all.
  // Removing springs completely causes problems.
  const Array<float> k_spring_sched2 = {1e-2f, 1e-4f};
  // const Array<float> k_spring_sched2 = {0.f};        // try no springs
  // const Array<float> k_spring_sched2 = {1e-2f, 0.f};  // try no springs at end
  spring = k_spring_sched2[0];
  perhaps_initialize();
  for (float spr : k_spring_sched2) {
    spring = spr;  // -spring f
    fliter = 0;
    do_stoc();
  }
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSD(mfilename, "file.m : initial mesh (can be -)");
  HH_ARGSD(filename, "file.pts : point data (can be -)");
  HH_ARGSP(crep, "v : set representation energy");
  HH_ARGSD(reconstruct, ": apply surface reconstruction schedule");
  HH_ARGSD(simplify, ": apply mesh simplification schedule");
  HH_ARGSD(quicksimplify, ": fast simplification");
  HH_ARGSD(zippysimplify, ": faster simplification");
  HH_ARGSC("", ":");
  HH_ARGSP(spring, "tension : set sprint constant");
  HH_ARGSP(dihfac, "val : set edge dihedral energy factor");
  HH_ARGSP(dihpower, "pow : set edge dihedral energy exponent");
  HH_ARGSP(restrictfproject, "int : 0=never, 1=first_iter, 2=always");
  HH_ARGSC("", ":");
  HH_ARGSD(outlierdelete, "dist : remove points more than given distance from initial mesh");
  HH_ARGSD(gfit, "niter : do global fit (0=until convergence)");
  HH_ARGSD(fgfit, "niter : use conjugate gradients");
  HH_ARGSD(stoc, ": do local stochastic mesh operations");
  HH_ARGSD(lfit, "ni nli : do ni iters, each nli local fits");
  HH_ARGSD(four1split, ": do global four-to-one split");
  HH_ARGSD(outmesh, "file.m : output current mesh to file");
  HH_ARGSC("", ":");
  HH_ARGSD(pclp, ": print projections onto mesh (lines)");
  HH_ARGSD(record, ": print mesh changes on std::cout, -noout");
  HH_ARGSD(spawn, "'command': send record to popen");
  HH_ARGSF(nooutput, ": don't print final mesh on stdout");
  HH_ARGSP(verb, "i : verbosity level (1=avg, 2=more, 3=lots, 4=huge)");
  HH_ARGSC("", ":");
  HH_ARGSF(boundaryfixed, ": fix the boundary for fgfit");
  HH_ARGSP(crbf, "ratio : set repr. energy boundary factor");
  HH_ARGSP(spbf, "ratio : set spring constant boundary factor");
  HH_ARGSP(fliter, "factor : modify # local iters done in stoc");
  HH_ARGSP(feswaasym, "f : set drss threshold (fraction of crep)");
  HH_TIMER(Meshfit);
  showdf("%s", args.header().c_str());
  args.parse();
  perhaps_initialize();
  analyze_mesh("FINAL");
  HH_TIMER_END(Meshfit);
  hh_clean_up();
  if (!nooutput) {
    mesh_transform(xformi);
    mark_mesh();
    mesh.write(std::cout);
  }
  file_spawn = nullptr;
  return 0;
}
