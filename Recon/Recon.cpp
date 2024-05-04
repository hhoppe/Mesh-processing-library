// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
// From a set of points, build an implicit function that represents the signed
// distance to the manifold.  Then extract its zero set.
// 1993-03-16 ported from C version

// Notes:
//  The pcnor's should point outwards. (The distance function is defined to be negative inside the object.)
//  The growing of pc's is not ideal.
//  The goals are:
//   - to only consider points on the current sheet
//   - to get a good tangent plane pcnor[], so need sufficient number of
//     points to form good estimator (e.g. 4 points near sharp corner can be
//     terrible) in the presence of noise, should consider a larger set
//   - to be adaptive to sampling density and noise

// 1993-04-27: idea: use samplingd for everything, don't use npertp
//  to estimate tp, gather all points within samplingd
//  to connect tp, connect tp's whose original co's are within samplingd
//  signed distance is undefined if projp >samplingd from nearest point
//  no longer need EMST, assume this new graph represents components
// 1993-09-09: all coordinates transformed internally into unit box.

#include "libHh/A3dStream.h"
#include "libHh/Args.h"
#include "libHh/Array.h"
#include "libHh/Bbox.h"
#include "libHh/Contour.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/Graph.h"
#include "libHh/GraphOp.h"  // graph_edge_stats(), graph_num_components(), graph_mst()
#include "libHh/Homogeneous.h"
#include "libHh/MeshOp.h"
#include "libHh/Mk3d.h"
#include "libHh/Mklib.h"
#include "libHh/Polygon.h"
#include "libHh/Principal.h"
#include "libHh/Set.h"
#include "libHh/Spatial.h"
#include "libHh/Stat.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

// necessary
string rootname;
string what = "m";
float samplingd = 0.f;
int gridsize = 0;

// optional
float unsigneddis = 0.f;
int prop = 2;
int usenormals = 0;  // 1=use them in optimization, 2=skip opt+use to orient, 3=use_exactly
int maxkintp = 20;
int minkintp = 4;

int num;            // # data points
bool is_3D;         // is it a 3D problem (vs. 2D)
int minora;         // minor axis: 1 in 2D, 2 in 3D
bool have_normals;  // data contains normal information

string g_header;  // header to place at top of output files
Frame xform;      // original pts -> pts in unit cube
Frame xformi;     // inverse

unique_ptr<PointSpatial<int>> SPp;   // spatial partition on co
unique_ptr<PointSpatial<int>> SPpc;  // spatial partition on pcorg
unique_ptr<Graph<int>> gpcpseudo;    // Riemannian on pc centers (based on co)
unique_ptr<Graph<int>> gpcpath;      // path of orientation propagation

Map<Mk3d*, unique_ptr<WFile>> g_map_mk3d_wfile;

GMesh mesh;
unique_ptr<Stat> pScorr;  // correlation in orientation propagation

unique_ptr<Mk3d> iod;  // lines: co[i] to pcorg[i]
unique_ptr<Mk3d> iob;  // unoriented pc boxes (volumes)
unique_ptr<Mk3d> iou;  // unoriented tangent planes (polygons)
unique_ptr<Mk3d> iof;  // flat tangent subspaces (polyline)
unique_ptr<Mk3d> iog;  // pseudograph over pcorg
unique_ptr<Mk3d> iop;  // orientation propagation path
unique_ptr<Mk3d> ioo;  // oriented tangent planes
unique_ptr<Mk3d> ioh;  // iog with normals
unique_ptr<Mk3d> iol;  // projections co[i] to nearest tp
unique_ptr<Mk3d> ioc;  // visited marching cubes
unique_ptr<Mk3d> iom;  // final mesh (not a3d!), a3d curve if !is_3D

Array<Point> co;    // data points
Array<Vector> nor;  // normal at point (is_zero() if none)
// The following are allocated after points have been read in
Array<Point> pcorg;    // origins of tangent planes
Array<Vector> pcnor;   // normals of tangent planes
Array<bool> pciso;     // tangent plane has been oriented
Array<Frame> pctrans;  // defined if ioo

void close_mk(unique_ptr<Mk3d>& pmk) {
  if (!pmk) return;
  auto pfi = assertx(g_map_mk3d_wfile.remove(pmk.get()));
  WSA3dStream* oa3d = down_cast<WSA3dStream*>(&pmk->oa3d());
  pmk = nullptr;  // destruct Mk3d
  delete oa3d;    // destruct WSA3dStream
  pfi = nullptr;  // destruct WFile
}

void process_read() {
  HH_TIMER("_read");
  RSA3dStream ia3d(std::cin);
  A3dElem el;
  int nnor = 0;
  for (;;) {
    ia3d.read(el);
    if (el.type() == A3dElem::EType::endfile) break;
    if (el.type() == A3dElem::EType::comment) continue;
    assertx(el.type() == A3dElem::EType::point);
    co.push(el[0].p);
    nor.push(el[0].n);
    if (co[num][0]) is_3D = true;
    if (!is_zero(nor[num])) {
      nnor++;
      if (usenormals) assertw(nor[num].normalize());
    }
    num++;
  }
  showdf("%d points (with %d normals), %dD analysis\n", num, nnor, (is_3D ? 3 : 2));
  assertx(num > 1);
  minora = is_3D ? 2 : 1;
  if (nnor > 0 && !usenormals) showdf("ignoring normals!\n");
  have_normals = usenormals && nnor > 0;
  // if (have_normals) assertx(prop == 2);
  co.shrink_to_fit();
  if (!have_normals) nor.init(0);
  nor.shrink_to_fit();
}

void compute_xform() {
  Bbox bbox;
  for_int(i, num) bbox.union_with(co[i]);
  xform = bbox.get_frame_to_small_cube();
  if (!is_3D) xform.p()[0] = 0.f;  // preserve x == 0
  float xform_scale = xform[0][0];
  showdf("Applying xform: %s", FrameIO::create_string(ObjectFrame{xform, 1}).c_str());
  xformi = ~xform;
  for_int(i, num) co[i] *= xform;
  // nor[] is unchanged
  samplingd *= xform_scale;
  unsigneddis *= xform_scale;
}

unique_ptr<Mk3d> process_arg(char ch) {
  if (!contains(what, ch)) return nullptr;
  if (rootname == "") assertx(what.size() == 1);
  auto pfi = make_unique<WFile>(rootname != "" ? sform("%s.%c", rootname.c_str(), ch) : "-");
  WFile& fi = *pfi;
  if (rootname != "") fi() << g_header;
  WSA3dStream* oa3d = new WSA3dStream(fi());  // deallocated by "delete oa3d" in close_mk()
  auto pmk = make_unique<Mk3d>(*oa3d);
  pmk->oa3d().write_comment(sform(" Output of option '%c'", ch));
  pmk->oa3d().flush();
  g_map_mk3d_wfile.enter(pmk.get(), std::move(pfi));
  return pmk;
}

void init_output() {
  iod = process_arg('d');
  iob = process_arg('b');
  iou = process_arg('u');
  iof = process_arg('f');
  iog = process_arg('g');
  iop = process_arg('p');
  ioo = process_arg('o');
  ioh = process_arg('h');
  iol = process_arg('l');
  ioc = process_arg('c');
  iom = process_arg('m');
}

void compute_tp(int i, int& n, Frame& f) {
  PArray<Point, 40> pa;
  SpatialSearch<int> ss(SPp.get(), co[i]);
  for (;;) {
    assertx(!ss.done());
    float dis2;
    int pi = ss.next(&dis2);
    if ((pa.num() >= minkintp && dis2 > square(samplingd)) || pa.num() >= maxkintp) break;
    pa.push(co[pi]);
    if (pi != i && !gpcpseudo->contains(i, pi)) gpcpseudo->enter_undirected(i, pi);
  }
  Vec3<float> eimag;
  principal_components(pa, f, eimag);
  n = pa.num();
}

void draw_pc_extent(Mk3d& mk) {
  MkSave mk_save(mk);
  mk.scale(2);
  Mklib mklib(mk);
  // frame is defined in terms of principal frame!
  if (is_3D) {
    mklib.cubeO();
  } else {
    mklib.squareU();
  }
}

void draw_pc_linear(Mk3d& mk) {
  if (is_3D) {
    MkSave mksave(mk);
    mk.scale(2);
    Mklib mklib(mk);
    mklib.squareU();
  } else {
    mk.point(+1.f, 0.f, 0.f);
    mk.point(-1.f, 0.f, 0.f);
    mk.end_polyline();
  }
}

void print_principal(const Frame& f) {
  if (iob) {
    // iob->diffuse(.8f, .5f, .4f); iob->specular(.5f, .5f, .5f); iob->phong(3.f);
    iob->diffuse(.6f, .6f, .6f);
    iob->specular(.5f, .5f, .5f);
    iob->phong(4.f);
    iob->push();
    iob->apply(f * xformi);
    draw_pc_extent(*iob);
    iob->pop();
  }
  if (iof) {
    iof->diffuse(1.f, 1.f, 1.f);
    iof->specular(0.f, 0.f, 0.f);
    iof->phong(1.f);
    iof->begin_force_polyline(true);
    {
      iof->push();
      iof->apply(f * xformi);
      draw_pc_linear(*iof);
      iof->pop();
    }
    iof->end_force_polyline();
  }
  if (iou) {
    iou->diffuse(.7f, .3f, .1f);
    iou->specular(.6f, .6f, .2f);
    iou->phong(3.f);
    iou->push();
    iou->apply(f * xformi);
    draw_pc_linear(*iou);
    iou->pop();
  }
}

void process_principal() {
  HH_TIMER("_principal");
  // Statistics in reverse order of printout
  HH_STAT(Sr21);
  HH_STAT(Sr20);
  HH_STAT(Sr10);
  HH_STAT(Slen2);
  HH_STAT(Slen1);
  HH_STAT(Slen0);
  HH_STAT(Snei);
  for_int(i, num) {
    int n;
    Frame f;
    compute_tp(i, n, f);
    if (ioo) pctrans[i] = f;
    Snei.enter(n);
    float len0 = mag(f.v(0)), len1 = mag(f.v(1)), len2 = mag(f.v(2));
    assertx(len2 > 0);  // principal_components() should do this
    Slen0.enter(len0);
    Slen1.enter(len1);
    Slen2.enter(len2);
    Sr10.enter(len1 / len0);
    Sr20.enter(len2 / len0);
    Sr21.enter(len2 / len1);
    pcorg[i] = f.p();
    pcnor[i] = usenormals < 3 ? f.v(minora) : nor[i];
    assertx(pcnor[i].normalize());
    print_principal(f);
  }
  close_mk(iob);
  close_mk(iof);
  close_mk(iou);
  if (iod) {
    iod->diffuse(1.f, 1.f, .3f);
    iod->specular(0.f, 0.f, 0.f);
    iod->phong(1.f);
    for_int(i, num) {
      iod->point(co[i] * xformi);
      iod->point(pcorg[i] * xformi);
      iod->end_polyline();
    }
  }
  close_mk(iod);
}

float pc_corr(int i, int j) {
  if (j == num && i < num) return pc_corr(j, i);
  assertx(i >= 0 && j >= 0 && i <= num && j < num);
  if (have_normals) assertx(prop == 2);
  float vdot, corr;
  switch (prop) {
    case 0:
      corr = 1.f;  // any path is ok
      break;
    case 1:
      if (i == num) {
        corr = 0.f;  // single exterior link
      } else {
        corr = dist2(pcorg[i], pcorg[j]);
      }
      break;
    case 2:
      if (i == num) {
        if (have_normals) {
          assertx(!is_zero(nor[j]));
          vdot = dot(nor[j], pcnor[j]);
        } else {
          vdot = 1.f;  // single exterior link
        }
      } else {
        vdot = dot(pcnor[i], pcnor[j]);
      }
      corr = 2.f - abs(vdot);
      break;
    default: assertnever("");
  }
  return corr;
}

float pc_dot(int i, int j) {
  assertx(i >= 0 && j >= 0 && i <= num && j < num);
  if (i == num) {
    if (have_normals) {
      return dot(nor[j], pcnor[j]);
    } else {
      return pcnor[j][2] < 0.f ? -1.f : 1.f;
    }
  } else {
    return dot(pcnor[i], pcnor[j]);
  }
}

void add_exterior_orientation(const Set<int>& nodes) {
  // vertex num is a pseudo-node used for outside orientation
  gpcpseudo->enter(num);
  if (have_normals) {
    // add pseudo-edges from "exterior" to points with normals
    for (int i : nodes) {
      if (is_zero(nor[i])) continue;
      gpcpseudo->enter_undirected(i, num);
    }
  } else {
    // add 1 pseudo-edge to point with largest z value
    float maxz = -BIGFLOAT;
    int maxi = -1;
    for (int i : nodes) {
      if (pcorg[i][2] > maxz) {
        maxz = pcorg[i][2];
        maxi = i;
      }
    }
    gpcpseudo->enter_undirected(maxi, num);
  }
}

void remove_exterior_orientation() {
  for (int i : Array<int>(gpcpseudo->edges(num))) gpcpseudo->remove_undirected(num, i);
  gpcpseudo->remove(num);
}

void show_propagation(int i, int j, float dotp) {
  assertx(i >= 0 && i <= num && j >= 0 && j < num && dotp >= 0);
  if (i == num || !iop) return;
  float f = min((1.f - dotp) * 10.f, 1.f);  // low dotp, high f are signific.
  iop->diffuse(.2f + .8f * f, .8f + .2f * f, .5f + .5f * f);
  iop->specular(0.f, 0.f, 0.f);
  iop->phong(3.f);
  iop->point(pcorg[i] * xformi);
  iop->normal(pcnor[i]);
  iop->point(pcorg[j] * xformi);
  iop->normal(pcnor[j]);
  iop->end_polyline();
}

// Propagate orientation along tree gpcpath from vertex i (orig. num) using recursive DFS.
void propagate_along_path(int i) {
  assertx(i >= 0 && i <= num);
  if (i < num) assertx(pciso[i]);
  for (int j : gpcpath->edges(i)) {
    assertx(j >= 0 && j <= num);
    if (j == num || pciso[j]) continue;  // immediate caller
    float corr = pc_dot(i, j);
    pScorr->enter(abs(corr));
    if (corr < 0) pcnor[j] = -pcnor[j];
    pciso[j] = true;
    show_propagation(i, j, abs(corr));
    propagate_along_path(j);
  }
}

void orient_set(const Set<int>& nodes) {
  showdf("component with %d points\n", nodes.num());
  add_exterior_orientation(nodes);
  gpcpath = make_unique<Graph<int>>();
  for (int i : nodes) gpcpath->enter(i);
  gpcpath->enter(num);
  {
    HH_TIMER("__graphmst");
    // must be connected here!
    assertx(graph_mst<int>(*gpcpseudo, pc_corr, *gpcpath));
  }
  int nextlink = gpcpath->out_degree(num);
  if (nextlink > 1) showdf(" num_exteriorlinks_used=%d\n", nextlink);
  propagate_along_path(num);
  gpcpath = nullptr;
  remove_exterior_orientation();
}

float pc_dist(int e1, int e2) { return dist(pcorg[e1], pcorg[e2]); }

void draw_oriented_tps() {
  ioo->diffuse(.7f, .3f, .1f);
  ioo->specular(.6f, .6f, .2f);
  ioo->phong(3.f);
  for_int(i, num) {
    Frame& f = pctrans[i];
    if (dot(f.v(minora), pcnor[i]) < 0) {
      // flip 2 of the axes to keep right hand rule
      f.v(minora) = -f.v(minora);
      f.v(0) = -f.v(0);
    }
    ioo->push();
    ioo->apply(f * xformi);
    draw_pc_linear(*ioo);
    if (!is_3D) {
      ioo->point(0.f, 0.f, 0.f);
      ioo->point(0.f, 1.f, 0.f);
      ioo->end_polyline();
    }
    ioo->pop();
  }
}

void print_graph(Mk3d& mk, const Graph<int>& g, CArrayView<Point> pa, CArrayView<Vector>* pn) {
  for_int(i, num) {
    for (int j : g.edges(i)) {
      if (j >= i) continue;
      mk.point(pa[i] * xformi);
      if (pn) mk.normal((*pn)[i]);
      mk.point(pa[j] * xformi);
      if (pn) mk.normal((*pn)[j]);
      mk.end_polyline();
    }
  }
}

void orient_tp() {
  HH_TIMER("_orient");
  if (usenormals == 3 && have_normals) return;
  if (usenormals == 2 && have_normals) {
    for_int(i, num) {
      if (dot(pcnor[i], nor[i]) < 0.f) pcnor[i] = -pcnor[i];
    }
    return;
  }
  {
    Stat stat = graph_edge_stats<int>(*gpcpseudo, pc_dist);
    stat.set_name("gpcpseudo");
    stat.set_print(true);
  }
  {
    HH_TIMER("__graphnumcompon");
    int nc = graph_num_components(*gpcpseudo);
    showdf("Number of components: %d\n", nc);
    if (nc > 1) showdf("*** #comp > 1, may want larger -samp\n");
  }
  if (iog) {
    iog->diffuse(.7f, .7f, .7f);
    iog->specular(0.f, 0.f, 0.f);
    iog->phong(1.f);
    print_graph(*iog, *gpcpseudo, pcorg, nullptr);
    close_mk(iog);
  }
  // Now treat each connected component of gpcpseudo separately.
  Set<int> setnotvis;
  for_int(i, num) setnotvis.enter(i);
  while (!setnotvis.empty()) {
    Set<int> nodes;
    Queue<int> queue;
    int fi = setnotvis.get_one();
    nodes.enter(fi);
    queue.enqueue(fi);
    while (!queue.empty()) {
      int i = queue.dequeue();
      assertx(setnotvis.remove(i));
      for (int j : gpcpseudo->edges(i)) {
        if (nodes.add(j)) queue.enqueue(j);
      }
    }
    pScorr = make_unique<Stat>("Scorr", true);
    orient_set(nodes);
    pScorr = nullptr;
  }
  for_int(i, num) assertx(pciso[i]);
  close_mk(iop);
  if (ioo) draw_oriented_tps();
  close_mk(ioo);
  if (ioh) {
    ioh->diffuse(.7f, .7f, .7f);
    ioh->specular(0.f, 0.f, 0.f);
    ioh->phong(1.f);
    print_graph(*ioh, *gpcpseudo, pcorg, &pcnor);
    close_mk(ioh);
  }
}

// Find the unsigned distance to the centroid of the k nearest data points.
float compute_unsigned(const Point& p, Point& proj) {
  SpatialSearch<int> ss(SPp.get(), p);
  Homogeneous h;
  const int k = 1;  // make a parameter?
  for_int(i, k) {
    assertx(!ss.done());
    int pi = ss.next();
    h += co[pi];
  }
  proj = to_Point(h / float(k));
  return dist(p, proj) - unsigneddis;
}

// Find the closest tangent plane origin and compute the signed distance to that tangent plane.
// Was: check to see if the projection onto the tangent plane lies farther than samplingd from any data point.
// Now: check to see if the sample point is farther than samplingd + cube_size from any data point.
float compute_signed(const Point& p, Point& proj) {
  int tpi;
  {
    SpatialSearch<int> ss(SPpc.get(), p);
    tpi = ss.next();
  }
  Vector vptopc = p - pcorg[tpi];
  float dis = dot(vptopc, pcnor[tpi]);
  proj = p - dis * pcnor[tpi];
  if (!is_3D) assertx(!proj[0]);
  if ((is_3D && (proj[0] <= 0 || proj[0] >= 1)) || proj[1] <= 0 || proj[1] >= 1 || proj[2] <= 0 || proj[2] >= 1)
    return k_Contour_undefined;
  if (1) {
    // check that projected point is close to a data point
    SpatialSearch<int> ss(SPp.get(), proj);
    float dis2;
    ss.next(&dis2);
    if (dis2 > square(samplingd)) return k_Contour_undefined;
  }
  if (prop) {
    // check that grid point is close to a data point
    SpatialSearch<int> ss(SPp.get(), p);
    float dis2;
    ss.next(&dis2);
    float grid_diagonal2 = square(1.f / gridsize) * 3.f;
    const float fudge = 1.2f;
    if (dis2 > grid_diagonal2 * square(fudge)) return k_Contour_undefined;
  }
  return dis;
}

void print_directed_seg(Mk3d& mk, const Point& p1, const Point& p2, const A3dColor& col) {
  Vector v = p2 - p1;
  assertx(v.normalize());
  mk.diffuse(col);
  mk.specular(0.f, 0.f, 0.f);
  mk.phong(3.f);
  mk.point(p1 * xformi);
  mk.normal(v);
  mk.point(p2 * xformi);
  mk.normal(v);
  mk.end_polyline();
}

Point build_Point(const Vec3<float>& p) { return p; }
Point build_Point(const Vec2<float>& p) { return Point(0.f, p[0], p[1]); }

template <int D> struct eval_point {
  float operator()(const Vec<float, D>& pp) const {
    ASSERTX((D == 3) == is_3D);
    Point p = build_Point(pp);
    Point proj;
    float dis = unsigneddis ? compute_unsigned(p, proj) : compute_signed(p, proj);
    if (dis == k_Contour_undefined) return dis;
    if (iol) {
      if (dis < 0)
        print_directed_seg(*iol, p, proj, A3dColor(1.f, 1.f, .3f));
      else
        print_directed_seg(*iol, proj, p, A3dColor(1.f, 1.f, 1.f));
    }
    return dis;
  }
};

struct output_border3D {
  void operator()(const Array<Vec3<float>>& poly) const {
    assertx(ioc);
    A3dElem el(A3dElem::EType::polygon);
    for_int(i, poly.num()) {
      el.push(A3dVertex(Point(poly[i]) * xformi, Vector(0.f, 0.f, 0.f),
                        A3dVertexColor(A3dColor(1.f, .16f, 0.f), A3dColor(1.f, .5f, .3f), A3dColor(3.f, 0.f, 0.f))));
    }
    ioc->oa3d().write(el);
  }
};

struct output_border2D {
  void operator()(const Array<Vec2<float>>& poly) const {
    assertx(ioc);
    ASSERTX(poly.num() == 2);
    A3dElem el(A3dElem::EType::polyline);
    for_int(i, poly.num()) {
      el.push(A3dVertex(Point(0.f, poly[i][0], poly[i][1]) * xformi, Vector(0.f, 0.f, 0.f),
                        A3dVertexColor(A3dColor(1.f, .16f, 0.f), A3dColor(1.f, .5f, .3f), A3dColor(3.f, 0.f, 0.f))));
    }
    ioc->oa3d().write(el);
  }
};

struct output_contour2D {
  void operator()(const Array<Vec2<float>>& poly) const {
    if (!iom) return;
    ASSERTX(poly.num() == 2);
    A3dElem el(A3dElem::EType::polyline);
    for_int(i, poly.num()) {
      el.push(A3dVertex(Point(0.f, poly[i][0], poly[i][1]) * xformi, Vector(0.f, 0.f, 0.f),
                        A3dVertexColor(A3dColor(.7f, .3f, .1f))));
    }
    iom->oa3d().write(el);
  }
};

template <typename Contour> void contour_3D(Contour& contour) {  // with or without border
  contour.set_ostream(&std::cout);
  if (unsigneddis) {
    Point p = co[0];
    for_intL(i, 1, num) {
      if (co[i][2] > p[2]) p = co[i];
    }
    for_int(i, 20) {
      p[2] += float(i) / gridsize;
      if (contour.march_from(p) > 1) break;
    }
  } else {
    for_int(i, num) contour.march_from(pcorg[i]);
  }
}

template <typename Contour> void contour_2D(Contour& contour) {  // with or without border
  contour.set_ostream(&std::cout);
  assertx(!pcorg[0][0]);
  if (unsigneddis) {
    Point p = co[0];
    for_intL(i, 1, num) {
      if (co[i][2] > p[2]) p = co[i];
    }
    for_int(i, 20) {
      p[2] += float(i) / gridsize;
      if (contour.march_from(V(p[1], p[2])) > 1) break;
    }
  } else {
    for_int(i, num) {
      Point p = pcorg[i];
      contour.march_from(V(p[1], p[2]));
    }
  }
}

void process_contour() {
  HH_TIMER("_contour");
  if (is_3D) {
    // Note: now mesh is always created even if !iom.
    if (ioc) {
      Contour3DMesh<eval_point<3>, output_border3D> contour(gridsize, &mesh);
      contour_3D(contour);
    } else {
      Contour3DMesh<eval_point<3>> contour(gridsize, &mesh);
      contour_3D(contour);
    }
  } else {
    if (ioc) {
      Contour2D<eval_point<2>, output_border2D> contour(gridsize);
      contour_2D(contour);
    } else {
      Contour2D<eval_point<2>, output_contour2D> contour(gridsize);
      contour_2D(contour);
    }
  }
  // iom closed back in main
  close_mk(ioc);
  close_mk(iol);
}

void process() {
  HH_TIMER("Recon");
  process_read();
  compute_xform();
  if (!gridsize) gridsize = int(1.f / samplingd + .5f);
  showdf("gridsize=%d\n", gridsize);
  assertx(gridsize >= 2);
  init_output();
  {
    pcorg.init(num);
    pcnor.init(num);
    pciso.init(num);
    if (ioo) pctrans.init(num);
    fill(pciso, false);
  }
  {
    HH_TIMER("_SPp");
    int n = is_3D ? (num > 100'000 ? 60 : num > 5000 ? 36 : 20) : (num > 1000 ? 36 : 20);
    SPp = make_unique<PointSpatial<int>>(n);
    for_int(i, num) SPp->enter(i, &co[i]);
  }
  if (!unsigneddis) {
    gpcpseudo = make_unique<Graph<int>>();
    for_int(i, num) gpcpseudo->enter(i);
    process_principal();
    {
      HH_TIMER("_SPpc");
      int n = is_3D ? (num > 100'000 ? 60 : num > 5000 ? 36 : 20) : (num > 1000 ? 36 : 20);
      SPpc = make_unique<PointSpatial<int>>(n);
      for_int(i, num) SPpc->enter(i, &pcorg[i]);
    }
    orient_tp();
    gpcpseudo = nullptr;
  }
  if (iol || ioc || iom) process_contour();
  if (iom && is_3D) showdf("%s\n", mesh_genus_string(mesh).c_str());
  SPp = nullptr;
  SPpc = nullptr;
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSP(rootname, "string : name for output files (optional)");
  HH_ARGSP(what, "string : output codes 'dbufgpohlcm' (default mesh 'm')");
  HH_ARGSP(samplingd, "f : sampling density + noise (delta + rho)");
  HH_ARGSP(gridsize, "n : contouring # grid cells (opt.)");
  HH_ARGSP(maxkintp, "k : max # points in tp");
  HH_ARGSP(minkintp, "k : min # points in tp");
  HH_ARGSP(unsigneddis, "f : use unsigned distance, set value");
  HH_ARGSP(prop, "i : orient. prop. (0=naive, 1=emst, 2=mst)");
  HH_ARGSP(usenormals, "i : use data normals (1=orient_opt, 2=orient, 3=exact)");
  args.parse();
  assertx(samplingd);
  g_header = args.header();
  showdf("%s", g_header.c_str());
  process();
  hh_clean_up();
  // We close iom here so that the mesh comes after everything else in the file.
  if (iom && is_3D) {
    for (Vertex v : mesh.vertices()) mesh.set_point(v, mesh.point(v) * xformi);
    // mesh.write(assertx(dynamic_cast<WSA3dStream*>(&iom->oa3d()))->os());  // note: would require RTTI
    mesh.write(down_cast<WSA3dStream*>(&iom->oa3d())->os());
  }
  close_mk(iom);
  return 0;
}
