// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/A3dStream.h"
#include "libHh/Args.h"
#include "libHh/Array.h"
#include "libHh/Bbox.h"
#include "libHh/Facedistance.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GeomOp.h"
#include "libHh/Homogeneous.h"
#include "libHh/LLS.h"
#include "libHh/Map.h"
#include "libHh/MathOp.h"
#include "libHh/MeshOp.h"
#include "libHh/MeshSearch.h"
#include "libHh/NonlinearOptimization.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Set.h"
#include "libHh/Stat.h"
#include "libHh/SubMesh.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

// weighta is subdivision mask weight, not averaging mask weight, so it must be adjusted.
int nsubdiv = 2;
bool nolimit = false;
float selective = 0.f;  // set to 180 to refine near sharp only
bool s222 = false;      // use s222 instead of n222 (not C1 for n=3)
float weighta = 0.f;    // for extraord. interior case, interior weight
float xformsize = .8f;
bool markcuts = false;
bool outn = false;
bool nooutput = false;
bool ecol = false;
bool gecol = false;
bool esha = false;
bool eswa = false;
bool espl = false;
float feshaasym = .05f;
float feswaasym = .05f;
float crep = 0.f;
float csharp = 0.f;
float wcrep = 0.f;
float wcsharp = 0.f;
float spring = 0.f;
float areafac = 0.f;
int verb = 1;

unique_ptr<WFile> wf_record;
constexpr float k_min_cos = -1.f / 3.f;  // acos(109.471) == tetrahedron angle

Array<Point> co;  // points
GMesh gmesh;      // current control mesh
Bbox gbbox;
Frame xform;
float xform_scale;
SubMesh* g_psmesh;
bool g_force_global_project = false;

const FlagMask eflag_cut = Mesh::allocate_Edge_flag();

// always updated
Array<float> gdis2;  // squared distance associated with each point

// for stoc
Map<Face, Set<int>> mfpts;  // Face in gmesh -> Set of point indices
Array<Face> gcmf;           // pi -> Face in gmesh
Array<int> gscmfi;          // smesh closest face index (in face gcmf)

// for general procedures
Array<Face> gscmf;  // face point projects to in some_smesh
Array<Bary> gbary;  // barycentric coordinates in some_smesh
Array<Point> gclp;  // closest point on some_smesh

enum EOperation { OP_ecol, OP_espl, OP_eswa, OP_esha, OP_NUM };
const Vec<string, OP_NUM> opname = {"ecol", "espl", "eswa", "esha"};
enum EResult { R_success, R_energy, R_dih, R_sharp, R_illegal, R_NUM };
const Vec<string, R_NUM> orname = {"success", "positive_energy", "bad_dihedral", "bad_sharp", "illegal_move"};
SGrid<int, OP_NUM, R_NUM> opstat;

struct hash_edge {
  size_t operator()(Edge e) const {
    return gmesh.vertex_id(gmesh.vertex1(e)) + intptr_t{gmesh.vertex_id(gmesh.vertex2(e))} * 76541;
  }
};
Set<Edge, hash_edge> ecand;  // Set of candidate edges in stoc; hash without pointers for portable random.

void mark_mesh(GMesh& m) {
  for (Vertex v : m.vertices()) {
    m.update_string(v, "cusp", m.flags(v).flag(GMesh::vflag_cusp) ? "" : nullptr);
  }
  for (Edge e : m.edges()) {
    m.update_string(e, "sharp", m.flags(e).flag(GMesh::eflag_sharp) ? "" : nullptr);
  }
  if (markcuts) {
    for (Edge e : m.edges()) {
      if (m.flags(e).flag(eflag_cut)) m.update_string(e, "cut", "");
    }
  }
}

void do_mfilename(Args& args) {
  assertx(!gmesh.num_vertices());
  RFile is(args.get_filename());
  gmesh.read(is());
  showdf("Initial mesh: %s\n", mesh_genus_string(gmesh).c_str());
  for (Vertex v : gmesh.vertices()) gbbox.union_with(gmesh.point(v));
  if (getenv_bool("FORCE_GLOBAL_PROJECT")) {
    Warning("Using FORCE_GLOBAL_PROJECT");
    g_force_global_project = true;
  }
}

void do_filename(Args& args) {
  assertx(!co.num());
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
    co.push(el[0].p);
    gbbox.union_with(el[0].p);
    gcmf.push(nullptr);
    gscmfi.push(0);
    gdis2.push(0.f);
    gscmf.push(nullptr);
    gbary.push(Bary(0.f, 0.f, 0.f));
    gclp.push(Point(0.f, 0.f, 0.f));
  }
  showdf("%d points read\n", co.num());
}

void initialize() {
  if (!xform[0][0]) {
    xform = gbbox.get_frame_to_small_cube(xformsize);
    if (verb >= 2) showdf(" internal xform: %s", FrameIO::create_string(xform, 1, 0.f).c_str());
    xform_scale = assertx(xform[0][0]);
  }
  wcrep = crep / square(xform_scale);
  wcsharp = csharp / square(xform_scale);
}

// *** auxiliary

void wf_frame() {
  if (!wf_record) return;
  (*wf_record)() << "# frame\n";
  (*wf_record)().flush();
}

void subdivide(SubMesh& smesh, bool triang) {
  HH_STIMER(___submesh);
  smesh.mask_parameters(s222, weighta);
  smesh.subdivide_n(nsubdiv, !nolimit, std::cos(to_rad(selective)), triang);
}

// translate Vertex from one Mesh to another Mesh
Vertex trvmm(Vertex v, const Mesh& mf, const Mesh& mt) { return mt.id_vertex(mf.vertex_id(v)); }

// translate Face from one Mesh to another Mesh
Face trfmm(Face f, const Mesh& mf, const Mesh& mt) { return mt.id_face(mf.face_id(f)); }

// translate Edge from one Mesh to another Mesh
Edge tremm(Edge e, const Mesh& mf, const Mesh& mt) {
  return mt.edge(trvmm(mf.vertex1(e), mf, mt), trvmm(mf.vertex2(e), mf, mt));
}

int vertex_num_sharp_edges(const GMesh& mesh, Vertex v) {
  int nsharpe = 0;
  for (Edge e : mesh.edges(v)) {
    if (mesh.is_boundary(e) || mesh.flags(e).flag(GMesh::eflag_sharp)) nsharpe++;
  }
  return nsharpe;
}

float min_dihedral_about_vertices(const GMesh& mesh, Vertex v) {
  Set<Edge> sete;
  for (Vertex vv : mesh.vertices(v)) {
    for (Edge e : mesh.edges(vv)) sete.add(e);
  }
  float mindic = 2;
  for (Edge e : sete) {
    if (mesh.is_boundary(e)) continue;
    float dic = edge_dihedral_angle_cos(mesh, e);
    mindic = min(mindic, dic);
  }
  return mindic;
}

bool ok_sharp_edge_change(Edge e) {
  if (!gmesh.flags(e).flag(GMesh::eflag_sharp)) {
    if (vertex_num_sharp_edges(gmesh, gmesh.vertex1(e)) >= 1 && vertex_num_sharp_edges(gmesh, gmesh.vertex2(e)) >= 1)
      return false;
  } else {
    if (vertex_num_sharp_edges(gmesh, gmesh.vertex1(e)) >= 3 && vertex_num_sharp_edges(gmesh, gmesh.vertex2(e)) >= 3)
      return false;
  }
  return true;
}

double get_edis() { return sum(gdis2); }

double get_espr() {
  if (!spring) return 0.;
  double sum = 0.;
  if (0) {
    for (Edge e : gmesh.edges()) sum += gmesh.length2(e) * spring;
  } else {
    for (Vertex v : gmesh.vertices()) {
      Homogeneous h(gmesh.point(v));
      float fac = 1.f / gmesh.degree(v);
      for (Vertex vv : gmesh.vertices(v)) h -= fac * Homogeneous(gmesh.point(vv));
      sum += mag2(to_Vector(h)) * spring;
    }
  }
  return sum;
}

double get_earea() {
  if (!areafac) return 0.;
  // may be a bug here: g_psmesh may not be defined everywhere?
  SubMesh& smesh = *assertx(g_psmesh);
  double sum = 0.;
  for (Face f : smesh.mesh().faces()) sum += smesh.mesh().area(f);
  return sum * areafac;
}

int num_sharp_edges() {
  int nsharpe = 0;
  // boundary edges are never sharp!
  for (Edge e : gmesh.edges()) {
    if (gmesh.flags(e).flag(GMesh::eflag_sharp)) {
      if (gmesh.is_boundary(e)) {
        Warning("Found boundary edge");
        gmesh.flags(e).flag(GMesh::eflag_sharp) = false;
      }
      nsharpe++;
    }
  }
  return nsharpe;
}

double get_etot() {
  return get_edis() + get_espr() + get_earea() + gmesh.num_vertices() * wcrep + num_sharp_edges() * wcsharp;
}

void analyze_mesh(const string& s) {
  showdf("%s: v=%d nse=%d/%d  edis=%g espr=%g edih=%g earea=%g etot=%g\n", s.c_str(), gmesh.num_vertices(),
         num_sharp_edges(), gmesh.num_edges(), get_edis(), get_espr(), 0., get_earea(), get_etot());
}

void global_all_project(const SubMesh& smesh) {
  HH_STIMER(___gallproject);
  const GMesh& mesh = smesh.mesh();
  MeshSearch msearch(&mesh, false);
  Face hintf = nullptr;
  for_int(i, co.num()) {
    Bary bary;
    Point clp;
    float d2;
    gscmf[i] = msearch.search(co[i], hintf, bary, clp, d2);
    hintf = gscmf[i];
    gbary[i] = bary;
    gclp[i] = clp;
    gdis2[i] = d2;
  }
}

void global_neighb_project(const SubMesh& smesh) {
  HH_STIMER(___gneighproject);
  if (g_force_global_project) {
    global_all_project(smesh);
  } else {
    for_int(i, co.num()) gdis2[i] = project_point_neighb(smesh.mesh(), co[i], gscmf[i], gbary[i], gclp[i], true);
  }
}

void global_lls(SubMesh& smesh, double& rss0, double& rss1) {
  HH_STIMER(___glls);
  GMesh& omesh = smesh.orig_mesh();
  GMesh& mesh = smesh.mesh();
  Map<Vertex, int> mvi;
  Array<Vertex> iv;
  for (Vertex v : omesh.vertices()) {
    mvi.enter(v, iv.num());
    iv.push(v);
  }
  int m = co.num(), n = iv.num();
  int mm = spring ? m + n : m;
  SparseLLS lls(mm, n, 3);
  lls.set_max_iter(10);
  Array<Vertex> va;
  for_int(i, m) {
    mesh.get_vertices(gscmf[i], va);
    assertx(va.num() == 3);
    Combvh tricomb;
    for_int(j, 3) tricomb.c[va[j]] = gbary[i][j];
    Combvh comb = smesh.compose_c_mvcvh(tricomb);
    HH_SSTAT(Scombnum, comb.c.num());
    Homogeneous h = Homogeneous(co[i]) - comb.h;
    lls.enter_b_r(i, h.head(3));
    for_combination(comb.c, [&](Vertex v, float val) { lls.enter_a_rc(i, mvi.get(v), val); });
  }
  if (spring) {
    // These are vertex-based springs, unlike edge-based in Meshfit.
    float sqrt_spring = sqrt(spring);
    for_int(i, n) {
      lls.enter_a_rc(m + i, i, +sqrt_spring);
      int deg = omesh.degree(iv[i]);
      for (Vertex v : omesh.vertices(iv[i])) {
        lls.enter_a_rc(m + i, mvi.get(v), -sqrt_spring / deg);
      }
      Vector zero(0.f, 0.f, 0.f);
      lls.enter_b_r(m + i, zero);
    }
  }
  for_int(i, n) lls.enter_xest_r(i, omesh.point(iv[i]));
  // Since specify max_iter, do not solve until convergence.
  {
    HH_STIMER(____gsolve);
    lls.solve(&rss0, &rss1);
  }
  for_int(i, n) {
    Point p;
    lls.get_x_r(i, p);
    omesh.set_point(iv[i], p);
  }
}

// *** main procedures

void do_record(Args& args) {
  wf_record = make_unique<WFile>(args.get_filename());
  mark_mesh(gmesh);
  gmesh.write((*wf_record)());
  gmesh.record_changes(&(*wf_record)());
}

void do_gfit(Args& args) {
  assertx(co.num() && gmesh.num_vertices());
  initialize();
  std::ostream* os = gmesh.record_changes(nullptr);
  if (verb >= 2) showdf("\n");
  HH_TIMER(_gfit);
  int niter = args.get_int();
  for (Vertex v : gmesh.vertices()) gmesh.flags(v).flag(SubMesh::vflag_variable) = true;
  SubMesh smesh(gmesh);
  subdivide(smesh, true);
  smesh.update_vertex_positions();
  global_all_project(smesh);
  if (verb >= 2) analyze_mesh("gfit_before");
  for_int(ni, niter) {
    HH_STIMER(__gfit_iter);
    double rss0, rss1;
    global_lls(smesh, rss0, rss1);
    if (verb >= 3) showf(" gopt %d/%d lls rss0=%g rss1=%g\n", ni + 1, niter, rss0, rss1);
    smesh.update_vertex_positions();
    global_neighb_project(smesh);
  }
  if (os) {
    gmesh.record_changes(os);
    for (Vertex v : gmesh.vertices()) gmesh.set_point(v, gmesh.point(v));
    wf_frame();
  }
  if (verb >= 2) analyze_mesh("gfit_after ");
}

// *** fgfit

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Vector, v_grad);

// Test using:
// Filtermesh ~/data/recon/new/cactus.crep1e-5.m -angle 55 -mark | Subdivfit -mf - -fi ~/data/recon/new/cactus.3337.pts -verb 3 -fgfit 60 >~/tmp/cactus.fgfit.m && G3dcmp ~/data/recon/new/cactus.nsub2.crep1e-5p.0.m ~/tmp/cactus.fgfit.m -key DmDe
// verdict: not stable enough; often jumps out of initial minimum to worse state; use do_gfit instead.
void do_fgfit(Args& args) {
  HH_TIMER(_fgfit);
  int niter = args.get_int();
  assertx(co.num() && gmesh.num_vertices());
  initialize();
  if (verb >= 2) showdf("\n");
  if (verb >= 1) showdf("fgfit with %d iterations\n", niter);
  std::ostream* os = gmesh.record_changes(nullptr);
  for (Vertex v : gmesh.vertices()) gmesh.flags(v).flag(SubMesh::vflag_variable) = true;
  SubMesh smesh(gmesh);
  g_psmesh = &smesh;
  subdivide(smesh, true);
  smesh.update_vertex_positions();
  global_all_project(smesh);
  if (verb >= 2) analyze_mesh("fgfit_before");
  struct FG {
    Map<Vertex, int> _mvi;  // vertex -> index in _x
    Array<Vertex> _iv;      // index -> mesh vertex
    Array<double> _x;       // linearized unknown vertex coordinates
    SubMesh& _smesh;
    int _iter{0};
    int _niter;
    double _etot{0.};
    bool _desire_global_project{false};
    explicit FG(SubMesh& smesh) : _smesh(smesh) {
      for (Vertex v : gmesh.vertices()) {
        _mvi.enter(v, _iv.num());
        _iv.push(v);
      }
      _x.init(_iv.num() * 3);
      _etot = get_etot();
      pack_vertices();
    }
    void pack_vertices() {
      for_int(j, _iv.num()) {
        const Point& p = gmesh.point(_iv[j]);
        for_int(c, 3) _x[j * 3 + c] = p[c];
      }
    }
    void unpack_vertices() const {
      for_int(j, _iv.num()) {
        Point p;
        for_int(c, 3) p[c] = float(_x[j * 3 + c]);
        gmesh.set_point(_iv[j], p);
      }
    }
    static inline Point proj_line(const Point& p, const Point& p0, const Point& p1) {
      Vector v01 = p1 - p0;
      if (!v01.normalize()) {
        Warning("proj_line fails");
        return p0;
      }
      return p0 + dot(p - p0, v01) * v01;
    }
    double feval(ArrayView<double> ret_grad) {  // evaluate function and its gradient
      assertx(ret_grad.num() == _iv.num() * 3);
      HH_STIMER(___computegrad);
      unpack_vertices();
      _smesh.update_vertex_positions();
      {
        bool bu_force_global_project = g_force_global_project;
        if (_desire_global_project) {
          g_force_global_project = true;
          _desire_global_project = false;
        }
        global_neighb_project(_smesh);
        g_force_global_project = bu_force_global_project;
      }
      if (verb >= 2) analyze_mesh(sform("it%2d/%-2d", _iter, _niter));
      // float edis = float(get_edis()); ?
      // float espr = float(get_espr());
      // float earea = float(get_earea());
      {
        double prev_etot = _etot;
        _etot = get_etot();  // edis + espr + earea;
        if (_etot > prev_etot * 1.1) {
          showdf("Large increase in energy after it%d, so next iteration uses global projection\n", _iter);
          _desire_global_project = true;
        }
      }
      // Computer gradient
      for (Vertex v : gmesh.vertices()) v_grad(v) = Vector(0.f, 0.f, 0.f);
      Array<Vertex> va;
      for_int(i, co.num()) {
        _smesh.mesh().get_vertices(gscmf[i], va);
        assertx(va.num() == 3);
        Vector vtop = co[i] - gclp[i];
        // this is faster than compose_c_mvcv(tricomb, comb);
        for_int(j, 3) {
          float a = -2 * gbary[i][j];
          if (!a) continue;
          const Combvh& comb = _smesh.combination(va[j]);
          assertx(is_zero(comb.h));
          for_combination(comb.c, [&](Vertex v, float val) { v_grad(v) += vtop * (a * val); });
        }
      }
      if (spring) {
        float sqrt_spring = sqrt(spring);
        for (Vertex v : gmesh.vertices()) {
          Homogeneous h(gmesh.point(v));
          float fac = 1.f / gmesh.degree(v);
          for (Vertex vv : gmesh.vertices(v)) h -= fac * Homogeneous(gmesh.point(vv));
          v_grad(v) += to_Vector(h) * sqrt_spring;
        }
      }
      if (areafac) {
        // area = 0.5 * w * h    darea / dh = 0.5 * w = A / h
        // grad area = 0.5 * w * normalized(hv) = hv * (area / mag2(hv))
        const GMesh& mesh = _smesh.mesh();
        for (Face f : mesh.faces()) {
          float farea = mesh.area(f);
          for (Vertex v : mesh.vertices(f)) {
            Point p = mesh.point(v);
            Point p0 = mesh.point(mesh.clw_vertex(f, v));
            Point p1 = mesh.point(mesh.ccw_vertex(f, v));
            Point pp = proj_line(p, p0, p1);
            Vector h = p - pp;
            float weight = farea / assertx(mag2(h));
            h *= weight;
            const Combvh& comb = _smesh.combination(v);
            assertx(is_zero(comb.h));
            for_combination(comb.c, [&](Vertex vv, float val) { v_grad(vv) += h * val; });
          }
        }
      }
      for_int(j, gmesh.num_vertices()) {
        const Vector& grad = v_grad(_iv[j]);
        for_int(c, 3) ret_grad[j * 3 + c] = grad[c];
      }
      _iter++;
      return _etot;
    }
  };
  FG fg(smesh);
  fg._niter = niter;
  if (getenv_string("NLOPT_DEBUG") == "") my_setenv("NLOPT_DEBUG", "1");
  auto func_eval = [&](ArrayView<double> ret_grad) { return fg.feval(ret_grad); };
  NonlinearOptimization<decltype(func_eval)> opt(fg._x, func_eval);
  assertx(niter > 0);
  opt.set_max_neval(niter + 1);
  assertw(opt.solve());
  if (os) {
    gmesh.record_changes(os);
    for (Vertex v : gmesh.vertices()) gmesh.set_point(v, gmesh.point(v));
    wf_frame();
  }
  if (verb >= 2) analyze_mesh("fgfit_after ");
  g_psmesh = nullptr;
}

void do_interp() {
  assertx(gmesh.num_vertices());
  initialize();
  if (verb >= 2) showdf("\n");
  HH_TIMER(_interp);
  for (Vertex v : gmesh.vertices()) gmesh.flags(v).flag(SubMesh::vflag_variable) = true;
  SubMesh smesh(gmesh);
  subdivide(smesh, true);
  Map<Vertex, int> mvi;
  Array<Vertex> iv;
  for (Vertex v : gmesh.vertices()) {
    mvi.enter(v, iv.num());
    iv.push(v);
  }
  int n = iv.num();
  // auto up_lls = LLS::make(n, n, 3, 12.f / n); LLS& lls = *up_lls;
  SparseLLS lls(n, n, 3);
  for_int(i, n) {
    Vertex v = iv[i];
    Point p = gmesh.point(v);
    lls.enter_b_r(i, p);
    Vertex vs = trvmm(v, gmesh, smesh.mesh());
    const Combvh& comb = smesh.combination(vs);
    assertx(is_zero(comb.h));
    for_combination(comb.c, [&](Vertex vv, float val) { lls.enter_a_rc(i, mvi.get(vv), val); });
  }
  for_int(i, n) {
    Point p = gmesh.point(iv[i]);
    lls.enter_xest_r(i, p);
  }
  double rss0, rss1;
  {
    HH_STIMER(__isolve);
    assertx(lls.solve(&rss0, &rss1));
  }
  if (verb >= 2) showdf("lls rss0=%g, rss1=%g\n", rss0, rss1);
  for_int(i, n) {
    Point p;
    lls.get_x_r(i, p);
    gmesh.set_point(iv[i], p);
  }
  if (co.num()) {
    smesh.update_vertex_positions();
    global_all_project(smesh);
    if (verb >= 2) analyze_mesh("interp_after ");
  }
}

void do_imagefit() {
  assertx(co.num() && gmesh.num_vertices());
  initialize();
  HH_TIMER(_imagefit);
  for (Vertex v : gmesh.vertices()) gmesh.flags(v).flag(SubMesh::vflag_variable) = true;
  SubMesh smesh(gmesh);
  subdivide(smesh, false);
  Map<Vertex, int> mvi;
  Array<Vertex> iv;
  for (Vertex v : gmesh.vertices()) {
    mvi.enter(v, iv.num());
    iv.push(v);
  }
  int m = smesh.mesh().num_vertices();
  const bool sprlimit = false;
  if (spring && !sprlimit) m += iv.num();
  if (spring && sprlimit) m *= 2;
  int n = iv.num();
  SparseLLS lls(m, n, 3);
  int row = 0;
  string str;
  for (Vertex v : smesh.mesh().vertices()) {
    Face f1 = assertx(smesh.mesh().most_ccw_face(v));
    Corner c1 = smesh.mesh().corner(v, f1);
    int imagen = to_int(assertx(smesh.mesh().corner_key(str, c1, "imagen")));
    assertx(imagen >= 1 && imagen <= co.num());
    Point p = co[imagen - 1];
    lls.enter_b_r(row, p);
    const Combvh& comb = smesh.combination(v);
    assertx(is_zero(comb.h));
    for_combination(comb.c, [&](Vertex vv, float val) { lls.enter_a_rc(row, mvi.get(vv), val); });
    row++;
  }
  if (spring && !sprlimit) {
    // These are vertex-based springs, unlike edge-based in Meshfit.
    float sqrt_spring = sqrt(spring);
    for_int(i, n) {
      lls.enter_a_rc(row, i, +sqrt_spring);
      int deg = gmesh.degree(iv[i]);
      for (Vertex v : gmesh.vertices(iv[i])) lls.enter_a_rc(row, mvi.get(v), -sqrt_spring / deg);
      Vector zero(0.f, 0.f, 0.f);
      lls.enter_b_r(row, zero);
      row++;
    }
  }
  if (spring && sprlimit) {
    float sqrt_spring = sqrt(spring);
    for (Vertex v : smesh.mesh().vertices()) {
      int deg = smesh.mesh().degree(v);
      Combvh barycomb;
      barycomb.c[v] = +sqrt_spring;
      for (Vertex vv : smesh.mesh().vertices(v)) barycomb.c[vv] = -sqrt_spring / deg;
      Combvh comb = smesh.compose_c_mvcvh(barycomb);
      for_combination(comb.c, [&](Vertex vv, float val) { lls.enter_a_rc(row, mvi.get(vv), val); });
      Vector zero(0.f, 0.f, 0.f);
      lls.enter_b_r(row, zero);
      row++;
    }
  }
  assertx(row == lls.num_rows());
  for_int(i, n) {
    Point p = gmesh.point(iv[i]);
    lls.enter_xest_r(i, p);
  }
  static const float tolerance = getenv_float("LLS_TOLERANCE", 1e-9f, true);  // larger than default 1e-10f
  lls.set_tolerance(tolerance);
  int max_iter = getenv_int("LLS_MAXITER", std::numeric_limits<int>::max(), true);
  lls.set_max_iter(max_iter);
  double rss0, rss1;
  {
    HH_STIMER(__isolve);
    assertw(lls.solve(&rss0, &rss1));
  }
  if (verb >= 1) showdf("lls rss0=%g, rss1=%g\n", rss0, rss1);
  for_int(i, n) {
    Point p;
    lls.get_x_r(i, p);
    gmesh.set_point(iv[i], p);
  }
}

// *** stoc

// defined in smesh
struct Combvih {
  Array<float> c;  // partial comb.: vertex index in iv -> float
  Homogeneous h;   // remainder of combination (constant term)
};
HH_SACABLE(Combvih);
HH_SAC_ALLOCATE_CD_FUNC(Mesh::MVertex, Combvih, v_combvih);

// defined in omesh() if vertex is in setmv
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, int, v_index);

struct Mvcvih {
  Array<Vertex> iv;  // int -> mesh Vertex
};

void create_mvcvih(const SubMesh& smesh, const Set<Vertex>& setmv, Mvcvih& mvcvih) {
  const GMesh& omesh = smesh.orig_mesh();
  dummy_use(omesh);
  const GMesh& m = smesh.mesh();
  for (Vertex v : setmv) {
    v_index(v) = mvcvih.iv.num();
    mvcvih.iv.push(v);
  }
  int nv = setmv.num();
  for (Vertex v : m.vertices()) {
    Combvih& cvih = v_combvih(v);
    cvih.c.init(nv, 0.f);
    const Combvh& comb = smesh.combination(v);
    cvih.h = comb.h;
    for_combination(comb.c, [&](Vertex vl, float val) { cvih.c[v_index(vl)] = val; });
  }
}

void update_local(SubMesh& smesh, const Mvcvih& mvcvih) {
  GMesh& m = smesh.mesh();
  int nv = mvcvih.iv.num();
  for (Vertex v : m.vertices()) {
    const Combvih& cvih = v_combvih(v);
    Homogeneous h(cvih.h);
    for_int(j, nv) {
      float b = cvih.c[j];
      if (b) h += b * Homogeneous(smesh.orig_mesh().point(mvcvih.iv[j]));
    }
    m.set_point(v, to_Point(h));
  }
}

void local_all_project(const SubMesh& smesh, const Set<Face>& setgoodf, const Set<int>& setpts,
                       const Set<int>& setbadpts) {
  HH_STIMER(___lallproject);
  const GMesh& mesh = smesh.mesh();
  Array<PolygonFace> ar_polyface;
  PolygonFaceSpatial psp(60);
  {
    HH_STIMER(____lmakespatial);
    for (Face f : mesh.faces()) {
      if (!setgoodf.contains(smesh.orig_face(f))) continue;
      Polygon poly(3);
      mesh.polygon(f, poly);
      for_int(i, poly.num()) poly[i] *= xform;
      ar_polyface.push(PolygonFace(std::move(poly), f));
    }
    for (PolygonFace& polyface : ar_polyface) psp.enter(&polyface);
  }
  HH_STIMER(____lspatialproject);
  for (int i : setpts) {
    if (setbadpts.contains(i)) {
      SpatialSearch<PolygonFace*> ss(&psp, co[i] * xform);
      PolygonFace* polyface = ss.next();
      gscmf[i] = polyface->face;
    } else {
      Face f = trfmm(gcmf[i], gmesh, smesh.orig_mesh());
      gscmf[i] = smesh.get_face(f, gscmfi[i]);
    }
  }
}

// Optimization of setmv given setpts, gscmf, mvcvih
void optimize_local(SubMesh& smesh, const Set<Vertex>& setmv, const Set<int>& setpts, const Mvcvih& mvcvih,
                    double& rss1) {
  HH_STIMER(___loptimize);
  update_local(smesh, mvcvih);
  const GMesh& mesh = smesh.mesh();
  {
    HH_STIMER(____lneighproject);
    for (int pi : setpts) {
      Point dummy_clp;
      project_point_neighb(mesh, co[pi], gscmf[pi], gbary[pi], dummy_clp, true);
    }
  }
  int m = setpts.num(), n = mvcvih.iv.num();
  assertx(n == setmv.num());  // optional
  // auto up_lls = make_unique<SparseLLS>(m, n, 3); SparseLLS& lls = *up_lls;
  auto up_lls = LLS::make(m, n, 3, 1.f);
  LLS& lls = *up_lls;
  int rowi = 0;
  {
    HH_STIMER(____lcombinations);
    Array<Vertex> va;
    for (int pi : setpts) {
      mesh.get_vertices(gscmf[pi], va);
      assertx(va.num() == 3);
      const Bary& bary = gbary[pi];
      double b0 = bary[0], b1 = bary[1], b2 = bary[2];
      Vec3<const Combvih*> cviha;
      for_int(c, 3) cviha[c] = &v_combvih(va[c]);
      Vec3<const float*> cvihcf;
      for_int(c, 3) cvihcf[c] = cviha[c]->c.data();
      Vec3<const float*> cvihhf;
      for_int(c, 3) cvihhf[c] = cviha[c]->h.data();
      Vec3<double> h;
      h[0] = b0 * cvihhf[0][0] + b1 * cvihhf[1][0] + b2 * cvihhf[2][0];
      h[1] = b0 * cvihhf[0][1] + b1 * cvihhf[1][1] + b2 * cvihhf[2][1];
      h[2] = b0 * cvihhf[0][2] + b1 * cvihhf[1][2] + b2 * cvihhf[2][2];
      for_int(j, n) lls.enter_a_rc(rowi, j, float(b0 * cvihcf[0][j] + b1 * cvihcf[1][j] + b2 * cvihcf[2][j]));
      // Homogeneous hrh = Homogeneous(co[pi])-h;
      Vector vrh;
      for_int(c, 3) vrh[c] = float(co[pi][c] - h[c]);
      lls.enter_b_r(rowi, vrh);
      rowi++;
    }
    assertx(rowi == lls.num_rows());
  }
  GMesh& omesh = smesh.orig_mesh();
  for_int(i, n) {
    const Point& p = omesh.point(mvcvih.iv[i]);
    lls.enter_xest_r(i, p);
  }
  {
    HH_STIMER(____lsolve);
    assertx(lls.solve(nullptr, &rss1));
  }
  if (0) SHOW("local optimization", m, n, rss1);
  for_int(i, n) {
    Point p;
    lls.get_x_r(i, p);
    omesh.set_point(mvcvih.iv[i], p);
  }
}

void build_lmesh1(const Set<Vertex>& setgmv, const Set<Face>& setbadfg, GMesh& lmesh, Set<int>& setpts,
                  Set<int>& setbadpts, double& rssf) {
  Set<Vertex> setvg;
  Set<Face> setfg;
  Set<Face> setmfg;
  for (Vertex v : setgmv) {
    for (Vertex vv : gmesh.vertices(v)) {
      for (Vertex vvv : gmesh.vertices(vv)) {
        for (Vertex vvvv : gmesh.vertices(vvv)) setvg.add(vvvv);
        for (Face f : gmesh.faces(vvv)) setfg.add(f);
      }
      for (Face f : gmesh.faces(vv)) setmfg.add(f);
    }
  }
  for (Vertex v : setvg) {
    Vertex vn = lmesh.create_vertex_private(gmesh.vertex_id(v));
    lmesh.set_point(vn, gmesh.point(v));
  }
  Array<Vertex> va;
  for (Face f : setfg) {
    gmesh.get_vertices(f, va);
    assertx(va.num() == 3);
    for_int(i, 3) va[i] = trvmm(va[i], gmesh, lmesh);
    lmesh.create_face_private(gmesh.face_id(f), va);
  }
  // Add flags to lmesh
  for (Edge e : lmesh.edges()) lmesh.flags(e) = gmesh.flags(tremm(e, lmesh, gmesh));
  for (Vertex v : lmesh.vertices()) lmesh.flags(v) = gmesh.flags(trvmm(v, lmesh, gmesh));
  // Gather into setpts the points projecting on faces in setmfg
  for (Face f : setmfg) {
    for (int pi : mfpts.get(f)) setpts.enter(pi);
  }
  {
    double sum = 0.;
    for (int pi : setpts) sum += gdis2[pi];
    rssf = sum;
  }
  // Gather into setbadpts the points projecting on faces in setbadfg
  for (Face f : setbadfg) {
    for (int pi : mfpts.get(f)) setbadpts.enter(pi);
  }
}

void build_lmesh2(GMesh& lmesh, const Set<Vertex>& setmv, Set<Face>& setmf) {
  for (Vertex v : lmesh.vertices()) {
    lmesh.flags(v).flag(SubMesh::vflag_variable) = setmv.contains(v);
  }
  for (Vertex v : setmv) {
    for (Vertex vv : lmesh.vertices(v)) {
      for (Face f : lmesh.faces(vv)) setmf.add(f);
    }
  }
}

// subdivide submesh; trim its outlying faces and vertices.
void subdiv_trim(SubMesh& smesh, const Set<Face>& setmf) {
  subdivide(smesh, true);
  {  // trim faces from smesh outside setmf
    Set<Face> setfrem;
    for (Face f : smesh.mesh().faces()) {
      if (!setmf.contains(smesh.orig_face(f))) setfrem.enter(f);
    }
    HH_SSTAT(Ssetfrem, setfrem.num());
    for (Face f : setfrem) smesh.mesh().destroy_face(f);
  }
  {  // trim isolated vertices
    Set<Vertex> setvrem;
    for (Vertex v : smesh.mesh().vertices()) {
      if (!smesh.mesh().degree(v)) setvrem.enter(v);
    }
    HH_SSTAT(Ssetvrem, setvrem.num());
    for (Vertex v : setvrem) smesh.mesh().destroy_vertex(v);
  }
}

void local_update_gmesh(const SubMesh& smesh, const Set<Vertex>& setmv, const Set<int>& setpts, Face f1l = nullptr,
                        Face f1g = nullptr, Face f2l = nullptr, Face f2g = nullptr, Vertex v1l = nullptr,
                        Vertex v1g = nullptr) {
  const GMesh& lmesh = smesh.orig_mesh();
  // Update positions of setmv in gmesh
  for (Vertex v : setmv) {
    Vertex vg = v == v1l ? v1g : trvmm(v, lmesh, gmesh);
    gmesh.set_point(vg, lmesh.point(v));
  }
  Array<Vertex> va;
  for (int pi : setpts) {
    const GMesh& mesh = smesh.mesh();
    mesh.get_vertices(gscmf[pi], va);
    assertx(va.num() == 3);
    gdis2[pi] = dist_point_triangle2(co[pi], mesh.point(va[0]), mesh.point(va[1]), mesh.point(va[2]));
    Face fl;
    int index;
    smesh.orig_face_index(gscmf[pi], fl, index);
    Face fg = fl == f1l ? f1g : fl == f2l ? f2g : trfmm(fl, lmesh, gmesh);
    gscmfi[pi] = index;
    if (fg == gcmf[pi]) continue;
    assertx(mfpts.get(gcmf[pi]).remove(pi));
    gcmf[pi] = fg;
    mfpts.get(fg).enter(pi);
  }
}

bool try_opt(SubMesh& smesh, const Set<Vertex>& setmv, const Set<int>& setpts, const Mvcvih& mvcvih, double threshrss,
             double& edrss) {
  double lrss = BIGFLOAT, rss1;
  dummy_init(rss1);
  const float anticipation = 3;
  const int maxtni = 10;
  const int minni = 6, maxni = 12;
  // It is worthwhile investing into the fit since it may eliminate
  // future operations.
  int ni;
  for (ni = 0; ni < maxtni; ni++) {
    optimize_local(smesh, setmv, setpts, mvcvih, rss1);
    ni++;
    if (rss1 - (lrss - rss1) * anticipation > threshrss) break;
    lrss = rss1;
  }
  HH_SSTAT(Soptnit, ni);
  edrss = rss1 - threshrss;
  if (edrss >= 0) return false;
  // do some more fitting
  while (ni < maxni) {
    optimize_local(smesh, setmv, setpts, mvcvih, rss1);
    ni++;
    if (ni < minni) continue;
    if (lrss - rss1 < wcrep * .1) break;
    lrss = rss1;
  }
  HH_SSTAT(Soptnig, ni);
  edrss = rss1 - threshrss;
  if (edrss >= 0) {
    Warning("try_opt strange");
    return false;
  }
  update_local(smesh, mvcvih);
  return true;
}

void remove_face(Face f) {
  assertx(mfpts.get(f).empty());
  mfpts.remove(f);
}

EResult try_ecol(Edge eg, double& edrss) {
  // SHOW("try_ecol");
  if (!gmesh.nice_edge_collapse(eg)) return R_illegal;
  if (!gecol && !ok_sharp_edge_change(eg)) return R_sharp;
  HH_STIMER(__try_ecol);
  GMesh lmesh;
  Set<int> setpts, setbadpts;
  double rssf;
  {
    Set<Vertex> setgmv;
    Set<Face> setbadfg;
    for (Vertex v : gmesh.vertices(eg)) {
      for (Vertex vv : gmesh.vertices(v)) setgmv.add(vv);
      for (Face f : gmesh.faces(v)) setbadfg.add(f);
    }
    build_lmesh1(setgmv, setbadfg, lmesh, setpts, setbadpts, rssf);
  }
  float minb = BIGFLOAT;
  for (Vertex v : gmesh.vertices(eg)) minb = min(minb, min_dihedral_about_vertices(gmesh, v));
  Edge e = tremm(eg, gmesh, lmesh);
  Vertex v1 = lmesh.vertex1(e), v2 = lmesh.vertex2(e);
  Point p1 = lmesh.point(v1), p2 = lmesh.point(v2);
  int ndsharp = lmesh.flags(e).flag(GMesh::eflag_sharp) ? 1 : 0;
  {
    Vertex vo1 = lmesh.side_vertex1(e), vo2 = lmesh.side_vertex2(e);
    if (lmesh.flags(lmesh.edge(v1, vo1)).flag(GMesh::eflag_sharp) &&
        lmesh.flags(lmesh.edge(v2, vo1)).flag(GMesh::eflag_sharp))
      ndsharp++;
    if (vo2 && lmesh.flags(lmesh.edge(v1, vo2)).flag(GMesh::eflag_sharp) &&
        lmesh.flags(lmesh.edge(v2, vo2)).flag(GMesh::eflag_sharp))
      ndsharp++;
  }
  lmesh.collapse_edge(e);  // keep v1
  Set<Vertex> setmv;
  setmv.enter(v1);
  for (Vertex vv : lmesh.vertices(v1)) setmv.enter(vv);
  Set<Face> setmf;
  build_lmesh2(lmesh, setmv, setmf);
  Set<Face> setgoodf;
  for (Face f : lmesh.faces(v1)) setgoodf.enter(f);
  SubMesh smesh(lmesh);
  subdiv_trim(smesh, setmf);
  Mvcvih mvcvih;
  create_mvcvih(smesh, setmv, mvcvih);
  HH_SSTAT(Secolpts, setpts.num());
  HH_SSTAT(Secolmf, setmf.num());
  HH_SSTAT(Secolmv, setmv.num());
  HH_SSTAT(Secolsmv, smesh.mesh().num_vertices());
  lmesh.set_point(v1, interp(p1, p2));
  update_local(smesh, mvcvih);
  local_all_project(smesh, setgoodf, setpts, setbadpts);  // get guess
  double minrss = BIGFLOAT;
  int minii = -1;
  for_int(ii, 3) {
    lmesh.set_point(v1, interp(p1, p2, ii * .5f));
    float mina = min_dihedral_about_vertices(lmesh, v1);
    if (mina < k_min_cos && mina < minb) continue;  // disallow
    double rss1;
    optimize_local(smesh, setmv, setpts, mvcvih, rss1);
    mina = min_dihedral_about_vertices(lmesh, v1);
    // Return to initial state.
    for (Vertex v : setmv) lmesh.set_point(v, gmesh.point(trvmm(v, lmesh, gmesh)));
    if (mina < k_min_cos && mina < minb) continue;  // disallow
    if (rss1 < minrss) {
      minrss = rss1;
      minii = ii;
    }
  }
  if (minii < 0) return R_dih;
  double threshrss = rssf + wcrep + ndsharp * wcsharp;
  // SHOW(minii, threshrss);
  lmesh.set_point(v1, interp(p1, p2, minii * .5f));
  if (!try_opt(smesh, setmv, setpts, mvcvih, threshrss, edrss)) return R_energy;
  float mina = min_dihedral_about_vertices(lmesh, v1);
  if (mina < k_min_cos && mina < minb) return R_dih;
  // ALL SYSTEMS GO
  Face f1g = gmesh.face1(eg), f2g = gmesh.face2(eg);
  for (Vertex v : gmesh.vertices(eg)) {
    for (Edge ee : gmesh.edges(v)) ecand.remove(ee);
  }
  Vertex v1g = gmesh.vertex1(eg);
  gmesh.collapse_edge(eg);  // keep v1g
  // add about 12-16 edges
  for (Edge ee : gmesh.edges(v1g)) ecand.add(ee);
  for (Face f : gmesh.faces(v1g)) ecand.add(gmesh.opp_edge(v1g, f));
  local_update_gmesh(smesh, setmv, setpts);
  remove_face(f1g);
  if (f2g) remove_face(f2g);
  return R_success;
}

EResult try_esha(Edge eg, double& edrss) {
  // SHOW("try_esha");
  const bool testdih = false;
  if (gmesh.is_boundary(eg)) return R_illegal;
  bool is_sharp = gmesh.flags(eg).flag(GMesh::eflag_sharp);
  float vcos = edge_dihedral_angle_cos(gmesh, eg);
  static const float k_cos30d = std::cos(to_rad(30.f));
  if (!is_sharp && vcos > k_cos30d) return R_sharp;  // quick culling
  // if is_sharp then always consider smoothing it
  HH_STIMER(__try_esha);
  // setbadpts and setgoodf are empty
  GMesh lmesh;
  Set<int> setpts, setbadpts;
  double rssf;
  {
    Set<Vertex> setgmv;
    Set<Face> setbadfg;
    for (Vertex v : gmesh.vertices(eg)) {
      for (Vertex vv : gmesh.vertices(v)) setgmv.add(vv);
    }
    build_lmesh1(setgmv, setbadfg, lmesh, setpts, setbadpts, rssf);
  }
  float minb = BIGFLOAT;
  if (testdih) {
    for (Vertex v : gmesh.vertices(eg)) minb = min(minb, min_dihedral_about_vertices(gmesh, v));
  }
  Edge e = tremm(eg, gmesh, lmesh);
  lmesh.flags(e).flag(GMesh::eflag_sharp) = !is_sharp;
  Set<Vertex> setmv;
  for (Vertex v : lmesh.vertices(e)) {
    for (Vertex vv : lmesh.vertices(v)) setmv.add(vv);
  }
  Set<Face> setmf;
  build_lmesh2(lmesh, setmv, setmf);
  Set<Face> setgoodf;
  SubMesh smesh(lmesh);
  subdiv_trim(smesh, setmf);
  Mvcvih mvcvih;
  create_mvcvih(smesh, setmv, mvcvih);
  HH_SSTAT(Seshapts, setpts.num());
  HH_SSTAT(Seshamf, setmf.num());
  HH_SSTAT(Seshamv, setmv.num());
  HH_SSTAT(Seshasmv, smesh.mesh().num_vertices());
  update_local(smesh, mvcvih);
  local_all_project(smesh, setgoodf, setpts, setbadpts);
  double threshrss = rssf - wcrep * feshaasym + (is_sharp ? 1. : -1.) * wcsharp;
  if (!try_opt(smesh, setmv, setpts, mvcvih, threshrss, edrss)) return R_energy;
  if (testdih) {
    float mina = BIGFLOAT;
    for (Vertex v : lmesh.vertices(e)) mina = min(mina, min_dihedral_about_vertices(lmesh, v));
    if (mina < k_min_cos && mina < minb) return R_dih;
  }
  // ALL SYSTEMS GO
  is_sharp = !is_sharp;
  gmesh.flags(eg).flag(GMesh::eflag_sharp) = is_sharp;
  if (wf_record) {
    (*wf_record)() << "Edge " << gmesh.vertex_id(gmesh.vertex1(e)) << " " << gmesh.vertex_id(gmesh.vertex2(e)) << " {"
                   << (is_sharp ? "sharp" : "") << "}\n";
  }
  for (Vertex v : gmesh.vertices(eg)) {
    for (Edge ee : gmesh.edges(v)) ecand.add(ee);
    for (Face f : gmesh.faces(v)) ecand.add(gmesh.opp_edge(v, f));
  }
  local_update_gmesh(smesh, setmv, setpts);
  return R_success;
}

EResult try_eswa(Edge eg, double& edrss) {
  // SHOW("try_eswa");
  if (!gmesh.legal_edge_swap(eg)) return R_illegal;
  bool is_sharp = gmesh.flags(eg).flag(GMesh::eflag_sharp);
  Vertex v1g = gmesh.vertex1(eg), v2g = gmesh.vertex2(eg);
  Vertex vo1g = gmesh.side_vertex1(eg), vo2g = gmesh.side_vertex2(eg);
  Face of1g = gmesh.face1(eg), of2g = gmesh.face2(eg);
  float minb = edge_dihedral_angle_cos(gmesh, eg);
  float mina = dihedral_angle_cos(gmesh.point(vo1g), gmesh.point(vo2g), gmesh.point(v1g), gmesh.point(v2g));
  if (mina < k_min_cos && mina < minb) return R_dih;
  // could do culling check if mina>cos5 && minb>cos5 ?
  HH_STIMER(__try_eswa);
  GMesh lmesh;
  Set<int> setpts, setbadpts;
  double rssf;
  {
    Set<Vertex> setgmv;
    Set<Face> setbadfg;
    setgmv.enter(v1g);
    setgmv.enter(v2g);
    setgmv.enter(vo1g);
    setgmv.enter(vo2g);
    setbadfg.enter(of1g);
    setbadfg.enter(of2g);
    build_lmesh1(setgmv, setbadfg, lmesh, setpts, setbadpts, rssf);
  }
  Edge oe = tremm(eg, gmesh, lmesh);
  Vertex v1 = lmesh.vertex1(oe), v2 = lmesh.vertex2(oe);
  Vertex vo1 = lmesh.side_vertex1(oe), vo2 = lmesh.side_vertex2(oe);
  Edge ne = lmesh.swap_edge(oe);  // new edge is not sharp.  make sharp?
  Face nf1l = lmesh.face1(ne), nf2l = lmesh.face2(ne);
  Set<Vertex> setmv;
  setmv.enter(v1);
  setmv.enter(v2);
  setmv.enter(vo1);
  setmv.enter(vo2);
  Set<Face> setmf;
  build_lmesh2(lmesh, setmv, setmf);
  Set<Face> setgoodf;
  setgoodf.enter(nf1l);
  setgoodf.enter(nf2l);
  SubMesh smesh(lmesh);
  subdiv_trim(smesh, setmf);
  Mvcvih mvcvih;
  create_mvcvih(smesh, setmv, mvcvih);
  HH_SSTAT(Seswapts, setpts.num());
  HH_SSTAT(Seswamf, setmf.num());
  HH_SSTAT(Seswamv, setmv.num());
  HH_SSTAT(Seswasmv, smesh.mesh().num_vertices());
  update_local(smesh, mvcvih);
  local_all_project(smesh, setgoodf, setpts, setbadpts);
  double threshrss = rssf - wcrep * feswaasym + (is_sharp ? 1. : 0.) * wcsharp;
  if (!try_opt(smesh, setmv, setpts, mvcvih, threshrss, edrss)) return R_energy;
  // ALL SYSTEMS GO
  ecand.remove(eg);
  Edge neg = gmesh.swap_edge(eg);  // new edge is not sharp!
  Face nf1g = gmesh.face1(neg), nf2g = gmesh.face2(neg);
  ecand.add(neg);
  ecand.add(gmesh.edge(v1g, vo1g));
  ecand.add(gmesh.edge(v2g, vo1g));
  ecand.add(gmesh.edge(v1g, vo2g));
  ecand.add(gmesh.edge(v2g, vo2g));
  if (nf1g != of1g && nf1g != of2g) mfpts.enter(nf1g, Set<int>());
  if (nf2g != of1g && nf2g != of2g) mfpts.enter(nf2g, Set<int>());
  local_update_gmesh(smesh, setmv, setpts, nf1l, nf1g, nf2l, nf2g);
  if (of1g != nf1g && of1g != nf2g) remove_face(of1g);
  if (of2g != nf1g && of2g != nf2g) remove_face(of2g);
  return R_success;
}

EResult try_espl(Edge eg, double& edrss) {
  // SHOW("try_espl");
  // always legal
  bool is_sharp = gmesh.flags(eg).flag(GMesh::eflag_sharp);
  Vertex v1g = gmesh.vertex1(eg), v2g = gmesh.vertex2(eg);
  Vertex vo1g = gmesh.side_vertex1(eg), vo2g = gmesh.side_vertex2(eg);
  Face f1g = gmesh.face1(eg), f2g = gmesh.face2(eg);
  // vo2g and f2g may be zero
  HH_STIMER(__try_espl);
  GMesh lmesh;
  Set<int> setpts, setbadpts;
  double rssf;
  {
    Set<Vertex> setgmv;
    Set<Face> setbadfg;
    setgmv.enter(v1g);
    setgmv.enter(v2g);
    setgmv.enter(vo1g);
    if (vo2g) setgmv.enter(vo2g);
    setbadfg.enter(f1g);
    if (f2g) setbadfg.enter(f2g);
    build_lmesh1(setgmv, setbadfg, lmesh, setpts, setbadpts, rssf);
  }
  Edge e = tremm(eg, gmesh, lmesh);
  Vertex v2 = lmesh.vertex2(e);
  Vertex vn = lmesh.split_edge(e);
  Edge eul = lmesh.edge(vn, v2);
  Face nf1l = lmesh.face1(eul), nf2l = lmesh.face2(eul);
  // edges (vn, vo1) [and (vn, vo2)] are not sharp
  Set<Vertex> setmv;
  setmv.enter(vn);
  for (Vertex v : lmesh.vertices(vn)) setmv.enter(v);
  Set<Face> setmf;
  build_lmesh2(lmesh, setmv, setmf);
  Set<Face> setgoodf;
  for (Face f : lmesh.faces(vn)) setgoodf.enter(f);
  SubMesh smesh(lmesh);
  subdiv_trim(smesh, setmf);
  Mvcvih mvcvih;
  create_mvcvih(smesh, setmv, mvcvih);
  HH_SSTAT(Sesplpts, setpts.num());
  HH_SSTAT(Sesplmf, setmf.num());
  HH_SSTAT(Sesplmv, setmv.num());
  HH_SSTAT(Sesplsmv, smesh.mesh().num_vertices());
  update_local(smesh, mvcvih);
  local_all_project(smesh, setgoodf, setpts, setbadpts);
  double threshrss = rssf - wcrep - (is_sharp ? 1. : 0.) * wcsharp;
  if (!try_opt(smesh, setmv, setpts, mvcvih, threshrss, edrss)) return R_energy;
  // ALL SYSTEMS GO
  for (Face f : gmesh.faces(eg)) {
    for (Edge ee : gmesh.edges(f)) ecand.remove(ee);
  }
  Vertex vng = gmesh.split_edge(eg);
  for (Face f : gmesh.faces(vng)) {
    for (Edge ee : gmesh.edges(f)) ecand.add(ee);
  }
  Edge eug = gmesh.edge(vng, v2g);
  Face nf1g = gmesh.face1(eug), nf2g = gmesh.face2(eug);
  mfpts.enter(nf1g, Set<int>());
  if (nf2g) mfpts.enter(nf2g, Set<int>());
  local_update_gmesh(smesh, setmv, setpts, nf1l, nf1g, nf2l, nf2g, vn, vng);
  return R_success;
}

EResult try_op(Edge e, EOperation op, double& edrss) {
  EResult result;
  result = (op == OP_ecol
                ? try_ecol(e, edrss)
                : op == OP_esha ? try_esha(e, edrss)
                                : op == OP_eswa ? try_eswa(e, edrss)
                                                : op == OP_espl ? try_espl(e, edrss) : (assertnever(""), R_success));
  opstat[op][result]++;
  return result;
}

void stoc_init() {
  HH_DTIMER(_initial_fit);
  for (Face f : gmesh.faces()) mfpts.enter(f, Set<int>());
  {
    for (Vertex v : gmesh.vertices()) {
      gmesh.flags(v).flag(SubMesh::vflag_variable) = false;
    }
    SubMesh smesh(gmesh);
    subdivide(smesh, true);
    smesh.update_vertex_positions();
    global_all_project(smesh);
    // Build up global projection information
    const GMesh& mesh = smesh.mesh();
    Array<Vertex> va;
    for_int(i, co.num()) {
      smesh.orig_face_index(gscmf[i], gcmf[i], gscmfi[i]);
      mfpts.get(gcmf[i]).enter(i);
      mesh.get_vertices(gscmf[i], va);
      assertx(va.num() == 3);
      gdis2[i] = dist_point_triangle2(co[i], mesh.point(va[0]), mesh.point(va[1]), mesh.point(va[2]));
    }
  }
  fill(opstat, 0);
}

void stoc_end() {
  showdf("Summary of attempts and results:\n");
  {
    string s = sform("%20s", "");
    for_int(i, opname.num()) s += sform("%10s", opname[i].c_str());
    showdf("%s\n", s.c_str());
  }
  {
    string s = sform("%20s", " total_attempts");
    for_int(i, opstat.num()) s += sform("%10d", narrow_cast<int>(sum(opstat[i])));
    showdf("%s\n", s.c_str());
  }
  for_int(j, opstat.dim(1)) {
    string s = sform("%20s", orname[j].c_str());
    for_int(i, opstat.dim(0)) s += sform("%10d", opstat[i][j]);
    showdf("%s\n", s.c_str());
  }
  {
    for (Vertex v : gmesh.vertices()) {
      gmesh.flags(v).flag(SubMesh::vflag_variable) = false;
    }
    SubMesh smesh(gmesh);
    subdivide(smesh, true);
    smesh.update_vertex_positions();
    global_all_project(smesh);
  }
  for (Face f : gmesh.faces()) mfpts.remove(f);
  assertx(!mfpts.num());
}

void do_stoc() {
  if (verb >= 2) showdf("\n");
  HH_TIMER(_stoc);
  assertx(co.num() && gmesh.num_vertices());
  initialize();
  if (verb >= 1) showdf("Stoc, crep=%g csharp=%g wcrep=%g wcsharp=%g\n", crep, csharp, wcrep, wcsharp);
  stoc_init();
  if (verb >= 2) analyze_mesh("stoc_before");
  if (gecol) ecol = true;
  for (Edge e : gmesh.edges()) ecand.enter(e);
  double cedis = get_edis(), cetot = get_etot();
  int i = 0, nbad = 0;
  while (!ecand.empty()) {
    std::cout.flush();
    HH_STIMER(__lattempt);
    i++;
    Edge e = ecand.remove_random(Random::G);
    gmesh.valid(e);  // optional
    EOperation op = OP_ecol;
    EResult result = R_illegal;
    double edrss = 0.;
    if (result != R_success && ecol) {
      op = OP_ecol;
      result = try_op(e, op, edrss);
    }
    if (result != R_success && esha) {
      op = OP_esha;
      result = try_op(e, op, edrss);
    }
    if (result != R_success && eswa) {
      op = OP_eswa;
      result = try_op(e, op, edrss);
    }
    if (result != R_success && espl) {
      op = OP_espl;
      result = try_op(e, op, edrss);
    }
    if (result == R_success) wf_frame();
    if (verb >= 3)
      showf(
          "# it %5d, %s (after %3d) [%5d/%-5d] %s\n", i, opname[op].c_str(), nbad, ecand.num(), gmesh.num_edges(),
          (result == R_success ? sform("* success e=%e", edrss).c_str()
                               : result == R_energy ? sform("positive e=%e", edrss).c_str() : orname[result].c_str()));
    if (result == R_success)
      nbad = 0;
    else
      nbad++;
    if (result == R_success) {
      double nedis = get_edis(), netot = get_etot();
      // showf("edis:%g->%g  etot:%g->%g\n", cedis, nedis, cetot, netot);
      HH_SSTAT(Sechange, netot - cetot);
      if (!assertw(netot <= cetot)) {
        HH_SSTAT(HHH_PECHANGE, netot - cetot);
      }
      cedis = nedis;
      cetot = netot;
    }
  }
  if (verb >= 2) showdf("it %d, last search: %d wasted attempts\n", i, nbad);
  showdf("New mesh: %s\n", mesh_genus_string(gmesh).c_str());
  stoc_end();
  if (verb >= 2) showdf(" last cedis=%g cetot=%g\n", cedis, cetot);
  if (verb >= 2) analyze_mesh("stoc_after ");
  ecol = gecol = esha = eswa = espl = false;  // reset flags (for perhaps other stoc)
}

void do_reconstruct() {
  showdf("Starting reconstruction sequence\n");
  showdf(" crep=%g, csharp=%g\n", crep, csharp);
  initialize();
  // do_gfit(as_lvalue(Args{"20"}));  // -gfit 20
  // do_fgfit(as_lvalue(Args{"40"}));  // -fgfit 40
  do_gfit(as_lvalue(Args{"6"}));  // -gfit 6
  do_gfit(as_lvalue(Args{"6"}));  // -gfit 6
  do_gfit(as_lvalue(Args{"6"}));  // -gfit 6
  ecol = true;                    // -ecol
  do_stoc();                      // -stoc
  // do_fgfit(as_lvalue(Args{"10"}));  // -fgfit 10
  do_gfit(as_lvalue(Args{"6"}));  // -gfit 6
  esha = true;                    // -esha
  do_stoc();                      // -stoc
  // do_fgfit(as_lvalue(Args{"10"}));  // -fgfit 10
  do_gfit(as_lvalue(Args{"6"}));  // -gfit 6
  gecol = true;                   // -gecol
  esha = true;                    // -esha
  eswa = true;                    // -eswa
  do_stoc();                      // -stoc
  // do_fgfit(as_lvalue(Args{"10"}));  // -fgfit 10
  do_gfit(as_lvalue(Args{"6"}));  // -gfit 6
}

void do_outmesh(Args& args) {
  WFile os(args.get_filename());
  mark_mesh(gmesh);
  gmesh.write(os());
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSD(mfilename, "file.m : read initial mesh (can be -)");
  HH_ARGSD(filename, "file.pts : read point data (can be -)");
  HH_ARGSP(verb, "i : set verbosity level");
  HH_ARGSF(outn, ": at end, write meshn instead of mesh0");
  HH_ARGSC("", ":");
  HH_ARGSD(record, "file.m : send orig + changes to file");
  HH_ARGSP(crep, "val : set repr. constant");
  HH_ARGSP(csharp, "val : penalize sharp edges");
  HH_ARGSP(spring, "val : edge neighborhood spring (only for *fit)");
  HH_ARGSP(areafac, "val : penalize area of limit surface");
  HH_ARGSD(reconstruct, ": run standard optimization");
  HH_ARGSC("", ":");
  HH_ARGSD(gfit, "niter : run global optimization (fixed K)");
  HH_ARGSD(fgfit, "niter : conjugate gradient method");
  HH_ARGSD(imagefit, ": fit using vertex imagen to index points");
  HH_ARGSD(interp, ": find interpolating mesh");
  HH_ARGSF(ecol, ":  try edge collapses, keep sharp topology");
  HH_ARGSF(gecol, ":  try general edge collapses");
  HH_ARGSF(esha, ":  try sharp changes");
  HH_ARGSF(eswa, ":  try edge swaps");
  HH_ARGSF(espl, ":  try edge splits");
  HH_ARGSD(stoc, ": run local discrete optimization");
  HH_ARGSC("", ":");
  HH_ARGSD(outmesh, "file.m : output current mesh0 to file");
  HH_ARGSF(nooutput, ": do not print final mesh0");
  HH_ARGSC("", ":");
  HH_ARGSP(feshaasym, "f : set drss threshold (fraction of crep)");
  HH_ARGSP(feswaasym, "f : set drss threshold (fraction of crep)");
  HH_ARGSC("", ":");
  HH_ARGSP(nsubdiv, "i : number of subdivision iters");
  HH_ARGSF(nolimit, ": do not send final vertices to limit");
  HH_ARGSC("", ":");
  HH_ARGSP(selective, "deg : refine sharpe edges and others>deg");
  HH_ARGSF(s222, ": use s222 mask instead of n222");
  HH_ARGSP(weighta, "a : override interior extraord. weight");
  HH_ARGSP(xformsize, "s : override internal xform");
  HH_ARGSF(markcuts, ": mark refined base mesh edges with 'cut'");
  HH_ARGSC("", ":");
  HH_ARGSC("  Examples", ": Subdivfit -mf - -outn");
  HH_ARGSC("", ": Subdivfit -mf v.m -fi v.pts  -gfit 10  -ecol -stoc >v");
  HH_ARGSC("", ":");
  HH_TIMER(Subdivfit);
  showdf("%s", args.header().c_str());
  args.parse();
  if (outn) {
    assertx(gmesh.num_vertices());
    for (Vertex v : gmesh.vertices()) gmesh.flags(v).flag(SubMesh::vflag_variable) = false;
    if (markcuts) {
      for (Edge e : gmesh.edges()) gmesh.flags(e).flag(eflag_cut) = true;
    }
    SubMesh smesh(gmesh);
    subdivide(smesh, false);
    smesh.update_vertex_positions();
    HH_TIMER_END(Subdivfit);
    hh_clean_up();
    if (!nooutput) {
      GMesh& m = smesh.mesh();
      mark_mesh(m);
      m.write(std::cout);
    }
  } else {
    HH_TIMER_END(Subdivfit);
    hh_clean_up();
    if (!nooutput) {
      mark_mesh(gmesh);
      gmesh.write(std::cout);
    }
  }
  if (wf_record) {
    gmesh.record_changes(nullptr);
    wf_record = nullptr;
  }
  return 0;
}
