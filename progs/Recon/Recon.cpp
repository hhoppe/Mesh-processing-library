// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
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
//  signed distance is undefined if projp more than samplingd from nearest point
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
#include "libHh/RangeOp.h"  // truncate()
#include "libHh/Set.h"
#include "libHh/Spatial.h"
#include "libHh/Stat.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

// Necessary.
string rootname;
string filename = "-";  // Input points; "-" is stdin.
string what = "m";
float samplingd = 0.f;
int gridsize = 0;

// Optional.
float unsigneddis = 0.f;
int prop = 2;
int usenormals = 0;  // 1 = use them in optimization, 2 = skip opt and use to orient, 3 = use_exactly.
int maxkintp = 20;
int minkintp = 4;

int num;            // # data points
bool is_3d;         // Is it a 3D problem (vs. 2D)?
int minora;         // Minor axis: 1 in 2D, 2 in 3D.
bool have_normals;  // The data contains normal information.
string g_header;  // Header to place at the top of output files.

// All geometry is in the original coordinates.  The unit cube is the domain of the spatial structures and the
// contouring grid, which are therefore accessed using xform and xform_inverse.
Frame xform;  // Original pts -> pts in the unit cube.
Frame xform_inverse;
float xform_scale;
float samplingd_cube;  // Parameter samplingd in the unit cube.

std::optional<PointSpatial<int>> SPp;   // Spatial partition on co_cube.
std::optional<PointSpatial<int>> SPpc;  // Spatial partition on pcorg_cube.
std::optional<Graph<int>> gpcpseudo;    // Riemannian on pc centers (based on co).
std::optional<Graph<int>> gpcpath;      // Path of orientation propagation.

Map<Mk3d*, unique_ptr<WFile>> g_map_mk3d_wfile;

GMesh mesh;
std::optional<Stat> pScorr;  // Correlation in orientation propagation.

unique_ptr<Mk3d> iod;  // Lines: co[i] to pcorg[i].
unique_ptr<Mk3d> iob;  // Unoriented pc boxes (volumes).
unique_ptr<Mk3d> iou;  // Unoriented tangent planes (polygons).
unique_ptr<Mk3d> iof;  // Flat tangent subspaces (polyline).
unique_ptr<Mk3d> iog;  // Pseudograph over pcorg.
unique_ptr<Mk3d> iop;  // Orientation propagation path.
unique_ptr<Mk3d> ioo;  // Oriented tangent planes.
unique_ptr<Mk3d> ioh;  // Here, iog with normals.
unique_ptr<Mk3d> iol;  // Projections co[i] to the nearest tp.
unique_ptr<Mk3d> ioc;  // Visited marching cubes.

bool want_mesh;                         // Write to stdout the final mesh, or the contour a3d curve if !is_3d.
std::optional<WSA3dStream> oa3d_curve;  // Contour curve on stdout, if !is_3d and want_mesh.

Array<Point> co;       // Data points.
Array<Point> co_cube;  // Data points in the unit cube.
Array<Vector> nor;     // Normal at the point (is_zero() if none).
// The following are allocated after points have been read in.
Array<Point> pcorg;       // Origins of tangent planes.
Array<Point> pcorg_cube;  // Origins of tangent planes in the unit cube.
Array<Vector> pcnor;      // Normals of tangent planes.
Array<bool> pciso;        // The tangent plane has been oriented.
Array<Frame> pctrans;     // Defined if ioo.

void close_mk(unique_ptr<Mk3d>& pmk) {
  if (!pmk) return;
  auto pfi = assertx(g_map_mk3d_wfile.remove(pmk.get()));
  WSA3dStream* oa3d = down_cast<WSA3dStream*>(&pmk->oa3d());
  pmk = nullptr;  // Destruct Mk3d.
  delete oa3d;    // Destruct WSA3dStream.
  pfi = nullptr;  // Destruct WFile.
}

void process_read() {
  HH_TIMER("_read");
  RFile fi(filename);
  std::istream& is = fi();
  for (string line; is.peek() == '#';) {
    assertx(my_getline(is, line));
    if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
  }
  showff("%s", g_header.c_str());
  RSA3dStream ia3d(is);
  A3dElem el;
  int nnor = 0;
  for (;;) {
    ia3d.read(el);
    if (el.type() == A3dElem::EType::endfile) break;
    if (el.type() == A3dElem::EType::comment) continue;
    assertx(el.type() == A3dElem::EType::point);
    co.push(el[0].p);
    nor.push(el[0].n);
    if (co[num][0]) is_3d = true;
    if (!is_zero(nor[num])) {
      nnor++;
      if (usenormals) assertw(nor[num].normalize());
    }
    num++;
  }
  showdf("%d points (with %d normals), %dD analysis\n", num, nnor, (is_3d ? 3 : 2));
  assertx(num > 1);
  minora = is_3d ? 2 : 1;
  if (nnor > 0 && !usenormals) showdf("ignoring normals!\n");
  have_normals = usenormals && nnor > 0;
  // if (have_normals) assertx(prop == 2);
  co.shrink_to_fit();
  if (!have_normals) nor.init(0);
  nor.shrink_to_fit();
}

void compute_xform() {
  assertx(co.num() == num);
  const Bbox bbox{co};
  xform = bbox.get_frame_to_small_cube();
  if (!is_3d) xform.p()[0] = 0.f;  // Preserve x == 0.
  xform_scale = xform[0, 0];
  showdf("Spatial xform: %s", FrameIO::create_string(ObjectFrame{xform, 1}).c_str());
  xform_inverse = ~xform;
  co_cube.init(num);
  for_int(i, num) co_cube[i] = co[i] * xform;
  samplingd_cube = samplingd * xform_scale;
}

unique_ptr<Mk3d> process_arg(char ch) {
  if (!what.contains(ch)) return nullptr;
  const string out_filename = what.size() == 1 ? "-" : sform("%s.%c.a3d", rootname.c_str(), ch);
  auto pfi = make_unique<WFile>(out_filename);
  WFile& fi = *pfi;
  // On stdout, main() has already written the header.
  if (out_filename != "-") fi() << "# " << g_header;
  WSA3dStream* oa3d = new WSA3dStream(fi());  // Deallocated by "delete oa3d" in close_mk().
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
  want_mesh = what.contains('m');
}

void compute_tp(int i, int& n, Frame& frame) {
  InlinedArray<Point, 40> pa;
  SpatialSearch<int> ss(&*SPp, co_cube[i]);
  for (const auto [pi, d2] : ss) {
    if ((pa.num() >= minkintp && d2 > square(samplingd_cube)) || pa.num() >= maxkintp) break;
    pa.push(co[pi]);
    if (pi != i && !gpcpseudo->contains(i, pi)) gpcpseudo->enter_undirected(i, pi);
  }
  Vec3<float> eimag;
  principal_components(pa, frame, eimag);
  n = pa.num();
}

void draw_pc_extent(Mk3d& mk) {
  const MkSave mk_save(mk);
  mk.scale(2);
  Mklib mklib(mk);
  // The frame is defined in terms of the principal frame!
  if (is_3d) {
    mklib.cubeO();
  } else {
    mklib.squareU();
  }
}

void draw_pc_linear(Mk3d& mk) {
  if (is_3d) {
    const MkSave mksave(mk);
    mk.scale(2);
    Mklib mklib(mk);
    mklib.squareU();
  } else {
    mk.point(+1.f, 0.f, 0.f);
    mk.point(-1.f, 0.f, 0.f);
    mk.end_polyline();
  }
}

void print_principal(const Frame& frame) {
  if (iob) {
    // iob->diffuse(.8f, .5f, .4f); iob->specular(.5f, .5f, .5f); iob->phong(3.f);
    iob->diffuse(.6f, .6f, .6f);
    iob->specular(.5f, .5f, .5f);
    iob->phong(4.f);
    iob->push();
    iob->apply(frame);
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
      iof->apply(frame);
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
    iou->apply(frame);
    draw_pc_linear(*iou);
    iou->pop();
  }
}

void process_principal() {
  HH_TIMER("_principal");
  // Statistics in reverse order of printout.
  HH_STAT(Sr21);
  HH_STAT(Sr20);
  HH_STAT(Sr10);
  HH_STAT(Slen2);
  HH_STAT(Slen1);
  HH_STAT(Slen0);
  HH_STAT(Snei);
  for_int(i, num) {
    int n;
    Frame frame;
    compute_tp(i, n, frame);
    if (ioo) pctrans[i] = frame;
    Snei.enter(n);
    const float len0 = mag(frame.v(0)), len1 = mag(frame.v(1)), len2 = mag(frame.v(2));
    assertx(len2 > 0.f);  // The function principal_components() should do this
    Slen0.enter(len0);
    Slen1.enter(len1);
    Slen2.enter(len2);
    Sr10.enter(len1 / len0);
    Sr20.enter(len2 / len0);
    Sr21.enter(len2 / len1);
    pcorg[i] = frame.p();
    pcnor[i] = usenormals == 3 && have_normals ? nor[i] : frame.v(minora);
    assertx(pcnor[i].normalize());
    print_principal(frame);
  }
  close_mk(iob);
  close_mk(iof);
  close_mk(iou);
  if (iod) {
    iod->diffuse(1.f, 1.f, .3f);
    iod->specular(0.f, 0.f, 0.f);
    iod->phong(1.f);
    for_int(i, num) {
      iod->point(co[i]);
      iod->point(pcorg[i]);
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
      corr = 1.f;  // Any path is OK.
      break;
    case 1:
      if (i == num) {
        corr = 0.f;  // A single exterior link.
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
          vdot = 1.f;  // A single exterior link.
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
      return sign(pcnor[j][2]);
    }
  } else {
    return dot(pcnor[i], pcnor[j]);
  }
}

void add_exterior_orientation(const Set<int>& nodes) {
  // Vertex num is a pseudo-node used for outside orientation.
  gpcpseudo->enter(num);
  if (have_normals) {
    // Add pseudo-edges from "exterior" to points with normals.
    for (const int i : nodes) {
      if (is_zero(nor[i])) continue;
      gpcpseudo->enter_undirected(i, num);
    }
  } else {
    // Add 1 pseudo-edge to the point with the largest z value.
    float maxz = -BIGFLOAT;
    int maxi = -1;
    for (const int i : nodes) {
      if (pcorg[i][2] > maxz) {
        maxz = pcorg[i][2];
        maxi = i;
      }
    }
    gpcpseudo->enter_undirected(maxi, num);
  }
}

void remove_exterior_orientation() {
  for (const int i : Array(gpcpseudo->edges(num))) gpcpseudo->remove_undirected(num, i);
  gpcpseudo->remove(num);
}

void show_propagation(int i, int j, float dotp) {
  assertx(i >= 0 && i <= num && j >= 0 && j < num && dotp >= 0.f);
  if (i == num || !iop) return;
  const float f = min((1.f - dotp) * 10.f, 1.f);  // Low dotp and high f are significant.
  iop->diffuse(.2f + .8f * f, .8f + .2f * f, .5f + .5f * f);
  iop->specular(0.f, 0.f, 0.f);
  iop->phong(3.f);
  iop->point(pcorg[i]);
  iop->normal(pcnor[i]);
  iop->point(pcorg[j]);
  iop->normal(pcnor[j]);
  iop->end_polyline();
}

// Propagate orientation along tree gpcpath from vertex i (orig. num) using recursive DFS.
void propagate_along_path(int i) {
  assertx(i >= 0 && i <= num);
  if (i < num) assertx(pciso[i]);
  for (const int j : gpcpath->edges(i)) {
    assertx(j >= 0 && j <= num);
    if (j == num || pciso[j]) continue;  // The immediate caller.
    const float corr = pc_dot(i, j);
    pScorr->enter(abs(corr));
    if (corr < 0.f) pcnor[j] = -pcnor[j];
    pciso[j] = true;
    show_propagation(i, j, abs(corr));
    propagate_along_path(j);
  }
}

void orient_set(const Set<int>& nodes) {
  showdf("component with %d points\n", nodes.num());
  add_exterior_orientation(nodes);
  gpcpath.emplace();
  {
    HH_TIMER("__graphmst");
    Graph<int> component;  // This component of gpcpseudo, together with the exterior node.
    for (const int i : nodes) component.enter(i);
    component.enter(num);
    for (const int i : nodes)
      for (const int j : gpcpseudo->edges(i)) component.enter(i, j);
    for (const int j : gpcpseudo->edges(num)) component.enter(num, j);
    // Must be connected here!
    auto [graph, is_connected] = graph_mst<int>(component, pc_corr);
    assertx(is_connected);
    *gpcpath = std::move(graph);
  }
  const int nextlink = gpcpath->out_degree(num);
  if (nextlink > 1) showdf(" num_exteriorlinks_used=%d\n", nextlink);
  propagate_along_path(num);
  gpcpath.reset();
  remove_exterior_orientation();
}

float pc_dist(int e1, int e2) { return dist(pcorg[e1], pcorg[e2]); }

void draw_oriented_tps() {
  ioo->diffuse(.7f, .3f, .1f);
  ioo->specular(.6f, .6f, .2f);
  ioo->phong(3.f);
  for_int(i, num) {
    Frame& frame = pctrans[i];
    if (dot(frame.v(minora), pcnor[i]) < 0.f) {
      // Flip 2 of the axes to keep the right-hand rule.
      frame.v(minora) = -frame.v(minora);
      frame.v(0) = -frame.v(0);
    }
    ioo->push();
    ioo->apply(frame);
    draw_pc_linear(*ioo);
    if (!is_3d) {
      ioo->point(0.f, 0.f, 0.f);
      ioo->point(0.f, 1.f, 0.f);
      ioo->end_polyline();
    }
    ioo->pop();
  }
}

void print_graph(Mk3d& mk, const Graph<int>& g, CArrayView<Point> pa, CArrayView<Vector>* pn) {
  for_int(i, num) {
    for (const int j : g.edges(i)) {
      if (j >= i) continue;
      mk.point(pa[i]);
      if (pn) mk.normal((*pn)[i]);
      mk.point(pa[j]);
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
    const int nc = graph_num_components(*gpcpseudo);
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
    const int fi = setnotvis.get_one();
    nodes.enter(fi);
    queue.enqueue(fi);
    while (!queue.empty()) {
      const int i = queue.dequeue();
      assertx(setnotvis.remove(i));
      for (const int j : gpcpseudo->edges(i))
        if (nodes.add(j)) queue.enqueue(j);
    }
    pScorr.emplace("Scorr", true);
    orient_set(nodes);
    pScorr.reset();
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

// Given the point p and its version p_cube in the unit cube, find the unsigned distance to the centroid of the k
// nearest data points.
float compute_unsigned(const Point& p, const Point& p_cube, Point& proj) {
  SpatialSearch<int> ss(&*SPp, p_cube);
  Homogeneous h;
  const int k = 1;  // Make a parameter?
  for (const auto [pi, unused_d2] : ss | truncate(k)) h += co[pi];
  proj = to_Point(h / float(k));
  return dist(p, proj) - unsigneddis;
}

// Given the point p and its version p_cube in the unit cube, find the closest tangent plane origin and compute the
// signed distance to that tangent plane.
// Was: check to see if the projection onto the tangent plane lies farther than samplingd from any data point.
// Now: check to see if the sample point is farther than samplingd + cube_size from any data point.
float compute_signed(const Point& p, const Point& p_cube, Point& proj) {
  int tpi;
  {
    SpatialSearch<int> ss(&*SPpc, p_cube);
    tpi = (*ss.begin()).id;
  }
  const Vector vptopc = p - pcorg[tpi];
  const float dis = dot(vptopc, pcnor[tpi]);
  proj = p - dis * pcnor[tpi];
  if (!is_3d) assertx(!proj[0]);
  const Point proj_cube = proj * xform;
  if ((is_3d && (proj_cube[0] <= 0.f || proj_cube[0] >= 1.f)) || proj_cube[1] <= 0.f || proj_cube[1] >= 1.f ||
      proj_cube[2] <= 0.f || proj_cube[2] >= 1.f)
    return k_Contour_undefined;
  if (1) {
    // Check that the projected point is close to a data point.
    SpatialSearch<int> ss(&*SPp, proj_cube);
    if ((*ss.begin()).d2 > square(samplingd_cube)) return k_Contour_undefined;
  }
  if (prop) {
    // Check that the grid point is close to a data point.
    SpatialSearch<int> ss(&*SPp, p_cube);
    const float d2 = (*ss.begin()).d2;
    const float grid_diagonal2 = square(1.f / gridsize) * 3.f;
    const float fudge = 1.2f;
    if (d2 > grid_diagonal2 * square(fudge)) return k_Contour_undefined;
  }
  return dis;
}

void print_directed_seg(Mk3d& mk, const Point& p1, const Point& p2, const A3dColor& col) {
  Vector v = p2 - p1;
  assertx(v.normalize());
  mk.diffuse(col);
  mk.specular(0.f, 0.f, 0.f);
  mk.phong(3.f);
  mk.point(p1);
  mk.normal(v);
  mk.point(p2);
  mk.normal(v);
  mk.end_polyline();
}

Point build_Point(const Vec3<float>& p) { return p; }
Point build_Point(const Vec2<float>& p) { return Point(0.f, p[0], p[1]); }

template <int D> struct EvalPoint {
  float operator()(const Vec<float, D>& pp) const {
    ASSERTX((D == 3) == is_3d);
    const Point p_cube = build_Point(pp);  // The contouring grid lies in the unit cube.
    const Point p = p_cube * xform_inverse;
    Point proj;
    const float dis = unsigneddis ? compute_unsigned(p, p_cube, proj) : compute_signed(p, p_cube, proj);
    if (dis == k_Contour_undefined) return dis;
    if (iol) {
      if (dis < 0.f)
        print_directed_seg(*iol, p, proj, A3dColor(1.f, 1.f, .3f));
      else
        print_directed_seg(*iol, proj, p, A3dColor(1.f, 1.f, 1.f));
    }
    return dis * xform_scale;  // The contouring works in the unit cube.
  }
};

struct OutputBorder3d {
  void operator()(CArrayView<Vec3<float>> poly) const {
    assertx(ioc);
    A3dElem el(A3dElem::EType::polygon);
    for_int(i, poly.num()) {
      el.push(A3dVertex(Point(poly[i]) * xform_inverse, Vector(0.f, 0.f, 0.f),
                        A3dVertexColor(A3dColor(1.f, .16f, 0.f), A3dColor(1.f, .5f, .3f), A3dColor(3.f, 0.f, 0.f))));
    }
    ioc->oa3d().write(el);
  }
};

struct OutputBorder2d {
  void operator()(CArrayView<Vec2<float>> poly) const {
    assertx(ioc);
    ASSERTX(poly.num() == 2);
    A3dElem el(A3dElem::EType::polyline);
    for_int(i, poly.num()) {
      el.push(A3dVertex(Point(0.f, poly[i][0], poly[i][1]) * xform_inverse, Vector(0.f, 0.f, 0.f),
                        A3dVertexColor(A3dColor(1.f, .16f, 0.f), A3dColor(1.f, .5f, .3f), A3dColor(3.f, 0.f, 0.f))));
    }
    ioc->oa3d().write(el);
  }
};

struct OutputContour2d {
  void operator()(CArrayView<Vec2<float>> poly) const {
    if (!oa3d_curve) return;
    ASSERTX(poly.num() == 2);
    A3dElem el(A3dElem::EType::polyline);
    for_int(i, poly.num()) {
      el.push(A3dVertex(Point(0.f, poly[i][0], poly[i][1]) * xform_inverse, Vector(0.f, 0.f, 0.f),
                        A3dVertexColor(A3dColor(.7f, .3f, .1f))));
    }
    oa3d_curve->write(el);
  }
};

template <typename Contour> void contour_3d(Contour& contour) {  // With or without a border.
  contour.set_ostream(&std::cout);
  if (unsigneddis) {
    Point p = co_cube[0];
    for_intL(i, 1, num) {
      if (co_cube[i][2] > p[2]) p = co_cube[i];
    }
    for_int(i, 20) {
      p[2] += float(i) / gridsize;
      if (contour.march_from(p) > 1) break;
    }
  } else {
    for_int(i, num) contour.march_from(pcorg_cube[i]);
  }
}

template <typename Contour> void contour_2d(Contour& contour) {  // With or without a border.
  contour.set_ostream(&std::cout);
  assertx(!pcorg[0][0]);
  if (unsigneddis) {
    Point p = co_cube[0];
    for_intL(i, 1, num) {
      if (co_cube[i][2] > p[2]) p = co_cube[i];
    }
    for_int(i, 20) {
      p[2] += float(i) / gridsize;
      if (contour.march_from(V(p[1], p[2])) > 1) break;
    }
  } else {
    for_int(i, num) {
      const Point& p = pcorg_cube[i];
      contour.march_from(V(p[1], p[2]));
    }
  }
}

void process_contour() {
  HH_TIMER("_contour");
  if (is_3d) {
    if (ioc) {
      Contour3dMesh<EvalPoint<3>, OutputBorder3d> contour(gridsize, &mesh);
      contour_3d(contour);
    } else {
      Contour3dMesh<EvalPoint<3>> contour(gridsize, &mesh);
      contour_3d(contour);
    }
  } else {
    if (want_mesh) oa3d_curve.emplace(std::cout);
    if (ioc) {
      Contour2d<EvalPoint<2>, OutputContour2d, OutputBorder2d> contour(gridsize);
      contour_2d(contour);
    } else {
      Contour2d<EvalPoint<2>, OutputContour2d> contour(gridsize);
      contour_2d(contour);
    }
    oa3d_curve.reset();  // Flush the stream, because main() may skip the destructors of globals.
  }
  close_mk(ioc);
  close_mk(iol);
}

void process() {
  HH_TIMER("Recon");
  process_read();
  compute_xform();
  if (!gridsize) gridsize = int(1.f / samplingd_cube + .5f);
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
    const int n = is_3d ? (num > 100'000 ? 60 : num > 5000 ? 36 : 20) : (num > 1000 ? 36 : 20);
    SPp.emplace(n);
    for_int(i, num) SPp->enter(i, &co_cube[i]);
  }
  if (!unsigneddis) {
    gpcpseudo.emplace();
    for_int(i, num) gpcpseudo->enter(i);
    process_principal();
    {
      HH_TIMER("_SPpc");
      const int n = is_3d ? (num > 100'000 ? 60 : num > 5000 ? 36 : 20) : (num > 1000 ? 36 : 20);
      SPpc.emplace(n);
      pcorg_cube.init(num);
      for_int(i, num) {
        pcorg_cube[i] = pcorg[i] * xform;
        SPpc->enter(i, &pcorg_cube[i]);
      }
    }
    orient_tp();
    gpcpseudo.reset();
  }
  if (iol || ioc || want_mesh) process_contour();
  if (want_mesh && is_3d) showdf("%s\n", mesh_genus_string(mesh).c_str());
  SPp.reset();
  SPpc.reset();
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSC("An 'a3d stream' of points is read from stdin or first arg.  Subsequent options are:");
  HH_ARGSP(rootname, "string : for multiple `what` options: write rootname.X.a3d");
  HH_ARGSP(what, "string : output codes 'dbufgpohlcm'; 'm' to stdout");
  HH_ARGSP(samplingd, "f : sampling density + noise (delta + rho)");
  HH_ARGSP(gridsize, "n : contouring # grid cells (opt.)");
  HH_ARGSP(maxkintp, "k : max # points in tp");
  HH_ARGSP(minkintp, "k : min # points in tp");
  HH_ARGSP(unsigneddis, "f : use unsigned distance, set value");
  HH_ARGSP(prop, "i : orient. prop. (0=naive, 1=emst, 2=mst)");
  HH_ARGSP(usenormals, "i : use data normals (1=orient_opt, 2=orient, 3=exact)");
  {
    const string arg0 = args.num() ? args.peek_string() : "";
    if (ParseArgs::special_arg(arg0)) args.parse(), exit(0);
    if (args.num() && (arg0 == "-" || arg0[0] != '-')) filename = args.get_filename();
  }
  g_header = args.header();
  args.parse();
  assertx(samplingd);
  assertx((what.size() > 1) == (rootname != ""));
  process();
  hh_clean_up();
  if (want_mesh && is_3d) {
    for (Vertex v : mesh.vertices()) mesh.set_point(v, mesh.point(v) * xform_inverse);
    mesh.write(std::cout);
  }
  if (k_fast_exit) exit_immediately(0);  // Skip ~GMesh().
  return 0;
}
