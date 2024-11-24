// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <cstring>  // strcmp(), strlen(), etc.

#include "libHh/A3dStream.h"
#include "libHh/Args.h"
#include "libHh/Array.h"
#include "libHh/Bbox.h"
#include "libHh/BinarySearch.h"
#include "libHh/ConsoleProgress.h"
#include "libHh/Contour.h"
#include "libHh/Facedistance.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"  // dihedral_angle_cos()
#include "libHh/HashPoint.h"
#include "libHh/HashTuple.h"  // hash<pair<...>>
#include "libHh/Homogeneous.h"
#include "libHh/Image.h"
#include "libHh/LinearFunc.h"
#include "libHh/Lls.h"
#include "libHh/Map.h"
#include "libHh/MathOp.h"
#include "libHh/MeshOp.h"  // Vnors, ...
#include "libHh/MeshSearch.h"
#include "libHh/Polygon.h"
#include "libHh/Pqueue.h"
#include "libHh/Principal.h"  // principal_components()
#include "libHh/Queue.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Set.h"
#include "libHh/Stack.h"
#include "libHh/Stat.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
#include "libHh/TriangleFaceSpatial.h"
using namespace hh;

#if defined(HH_HAVE_SIMPLEX)
#include "recipes.h"
#endif

namespace {

WSA3dStream oa3d{std::cout};
GMesh mesh;
// const A3dVertexColor k_surf_color{A3dColor(.8f, .5f, .4f), A3dColor(.5f, .5f, .5f), A3dColor(5.f, 0.f, 0.f)};
const A3dVertexColor k_surf_color{A3dColor(.6f, .6f, .6f), A3dColor(.5f, .5f, .5f), A3dColor(4.f, 0.f, 0.f)};
constexpr float k_undefined_cosangle = -1e31f;

bool alltriangulate = false;
float cosangle = k_undefined_cosangle;
float solidangle = 0.f;
float checkflat = 0.f;
bool nooutput = false;
bool bndmerge = false;
float raymaxdispfrac = .03f;
int nfaces = 0;
float maxcrit = 1e20f;
enum class EReduceCriterion { undefined, length, inscribed, volume, qem };
EReduceCriterion reducecrit = EReduceCriterion::undefined;

// *** helper

bool sharp(Edge e) { return mesh.is_boundary(e) || mesh.flags(e).flag(GMesh::eflag_sharp); }

inline bool same_string(const char* s1, const char* s2) {
  return !s1 && !s2 ? true : !s1 || !s2 ? false : !strcmp(s1, s2);
}

void output_mesh_as_a3d() { mesh.write(oa3d, k_surf_color); }

void output_edge(Edge e, A3dVertexColor col = A3dVertexColor(Pixel::black())) {
  A3dElem el(A3dElem::EType::polyline);
  el.push(A3dVertex(mesh.point(mesh.vertex1(e)), Vector(0.f, 0.f, 0.f), col));
  el.push(A3dVertex(mesh.point(mesh.vertex2(e)), Vector(0.f, 0.f, 0.f), col));
  oa3d.write(el);
}

void assign_normals() {
  string str;
  for (Vertex v : mesh.vertices()) {
    Vnors vnors(mesh, v);
    if (vnors.is_unique()) {
      const Vector& nor = vnors.unique_nor();
      mesh.update_string(v, "normal", csform_vec(str, nor));
      for (Corner c : mesh.corners(v)) mesh.update_string(c, "normal", nullptr);
    } else {
      mesh.update_string(v, "normal", nullptr);
      for (Corner c : mesh.corners(v)) {
        Face f = mesh.corner_face(c);
        const Vector& nor = vnors.face_nor(f);
        mesh.update_string(c, "normal", csform_vec(str, nor));
      }
    }
  }
}

Uv get_uv(Vertex v) {
  Uv uv;
  assertx(parse_key_vec(mesh.get_string(v), "uv", uv));
  return uv;
}

float mesh_area() {
  double sumarea = 0.;
  for (Face f : mesh.faces()) sumarea += mesh.area(f);
  return float(sumarea);
}

bool mesh_single_disk() {
  Stat Scompf = mesh_stat_components(mesh);
  Stat Sbound = mesh_stat_boundaries(mesh);
  return Scompf.num() == 1 && Sbound.num() == 1;
}

void do_creategrid(Args& args) {
  int ny = args.get_int(), nx = args.get_int();
  assertx(ny > 0 && nx > 0);
  assertx(mesh.empty());
  Matrix<Vertex> matv(ny, nx);
  for_int(y, ny) for_int(x, nx) {
    matv[y][x] = mesh.create_vertex();
    mesh.set_point(matv[y][x], Point(float(x) / (nx - 1.f), float(y) / (ny - 1.f), 0.f));
  }
  for_int(y, ny - 1) for_int(x, nx - 1) {
    mesh.create_face(V(matv[y + 0][x + 0], matv[y + 0][x + 1], matv[y + 1][x + 1], matv[y + 1][x + 0]));
  }
}

void do_fromgrid(Args& args) {
  int ny = args.get_int(), nx = args.get_int();
  assertx(ny > 0 && nx > 0);
  string filename = args.get_filename();
  assertx(mesh.empty());
  Matrix<Vertex> matv(ny, nx);
  RFile fi(filename);
  for_int(y, ny) for_int(x, nx) {
    float val;
    assertx(fi() >> val);
    matv[y][x] = mesh.create_vertex();
    mesh.set_point(matv[y][x], Point(float(x) / (nx - 1.f), float(y) / (ny - 1.f), val));
  }
  if (1) {
    float dummy_val;
    fi() >> dummy_val;
    assertw(!fi());
  }
  for_int(y, ny - 1) for_int(x, nx - 1) {
    mesh.create_face(V(matv[y + 0][x + 0], matv[y + 0][x + 1], matv[y + 1][x + 1], matv[y + 1][x + 0]));
  }
}

void do_frompointgrid(Args& args) {
  int ny = args.get_int(), nx = args.get_int();
  assertx(ny > 0 && nx > 0);
  string filename = args.get_filename();
  assertx(mesh.empty());
  Matrix<Vertex> matv(ny, nx);
  RFile fi(filename);
  RSA3dStream a3dstream(fi());
  A3dElem el;
  for_int(y, ny) for_int(x, nx) {
    a3dstream.read(el);
    assertx(el.type() != A3dElem::EType::endfile);
    if (el.type() == A3dElem::EType::comment) continue;
    assertx(el.type() == A3dElem::EType::point);
    Point p = el[0].p;
    matv[y][x] = mesh.create_vertex();
    mesh.set_point(matv[y][x], p);
  }
  for_int(y, ny - 1) for_int(x, nx - 1) {
    mesh.create_face(V(matv[y + 0][x + 0], matv[y + 0][x + 1], matv[y + 1][x + 1], matv[y + 1][x + 0]));
  }
}

// *** createobject

inline float lerp(float a, float b, float f) { return (1.f - f) * a + f * b; }

void do_createobject(Args& args) {
  assertx(mesh.empty());
  string ob_name = args.get_string();
  Matrix<Vertex> matv;
  bool closed = false;
  string str;
  if (0) {
  } else if (ob_name == "cylinder64") {
    const int ny = 64, nx = 64;
    matv.init(ny, nx);
    for_int(y, ny) for_int(x, nx) {
      Vertex v = mesh.create_vertex();
      matv[y][x] = v;
      float xf = float(x) / (nx - 1.f), yf = float(y) / (ny - 1.f);
      float ang = xf * TAU;
      mesh.set_point(v, Point(std::cos(ang), std::sin(ang), yf * (TAU / 2)));
      Uv uv(xf * 2.f, 1.f - yf);
      mesh.update_string(v, "uv", csform_vec(str, uv));
    }
  } else if (ob_name == "cup64") {
    // for ravg cup sdkmesh
    // r1 = 0.396313    y1 = 0.436485
    // r2 = 0.249999    y2 = -0.480285
    // u_min = 0.013743 u_max = 0.982733
    // v_min = 0.005718 v_max = 0.993877
    const int ny = 64, nx = 64;
    matv.init(ny, nx);
    for_int(y, ny) for_int(x, nx) {
      Vertex v = mesh.create_vertex();
      matv[y][x] = v;
      float xf = float(x) / (nx - 1.f), yf = float(y) / (ny - 1.f);
      float ang = xf * TAU;
      float r = lerp(0.396313f, +0.249999f, yf);
      float z = lerp(0.436485f, -0.480285f, yf);
      mesh.set_point(v, Point(r * std::cos(ang), r * std::sin(ang), z));
      Uv uv(lerp(0.013743f, 0.982733f, xf), lerp(0.005718f, 0.993877f, yf));
      mesh.update_string(v, "uv", csform_vec(str, uv));
    }
  } else if (ob_name == "wavy64") {
    const int ny = 64, nx = 64;
    matv.init(ny, nx);
    for_int(y, ny) for_int(x, nx) {
      Vertex v = mesh.create_vertex();
      matv[y][x] = v;
      float xf = float(x) / (nx - 1.f), yf = float(y) / (ny - 1.f);
      mesh.set_point(v, Point(xf, yf, std::sin(xf * TAU * 3.f) * .05f));
      Uv uv(xf, 1.f - yf);
      mesh.update_string(v, "uv", csform_vec(str, uv));
    }
  } else if (starts_with(ob_name, "torus")) {
    const int ny = 64, nx = 64;
    matv.init(ny, nx);
    closed = true;
    float rx_major = 1.5f, ry_major = 1.f;
    if (ob_name == "torus1") {
    } else if (ob_name == "torus2") {
      rx_major = 1.f;
    } else {
      assertnever("");
    }
    for_int(y, ny) for_int(x, nx) {
      Vertex v = mesh.create_vertex();
      matv[y][x] = v;
      float ang_major = x / (nx - 1.f) * TAU, ang_minor = y / (ny - 1.f) * TAU;
      float rx_minor = .7f + std::cos(ang_major) * .2f, ry_minor = rx_minor;
      float x_minor = rx_minor * std::cos(ang_minor), y_minor = ry_minor * std::sin(ang_minor);
      float xc = (rx_major + x_minor) * std::cos(ang_major), yc = (ry_major + x_minor) * std::sin(ang_major);
      float zc = y_minor;
      mesh.set_point(v, Point(xc, yc, zc));
    }
  } else {
    assertnever("");
  }
  if (matv.size()) {
    const int ny = matv.ysize(), nx = matv.xsize();
    const int mody = closed ? ny - 1 : ny, modx = closed ? nx - 1 : nx;
    for_int(y, ny - 1) for_int(x, nx - 1) {
      mesh.create_face(
          V(matv[y][x], matv[y][(x + 1) % modx], matv[(y + 1) % mody][(x + 1) % modx], matv[(y + 1) % mody][x]));
    }
  }
}

// *** froma3d

void do_froma3d() {
  HH_TIMER("_froma3d");
  HH_STAT(Sppdist2);
  RSA3dStream ia3d(std::cin);
  HashPoint hp;
  Array<Vertex> gva, va;
  A3dElem el;
  for (;;) {
    ia3d.read(el);
    if (el.type() == A3dElem::EType::endfile) break;
    if (el.type() == A3dElem::EType::comment) {
      showff("|%s\n", el.comment().c_str());
      continue;
    }
    if (el.type() != A3dElem::EType::polygon) {
      Warning("Non-polygon input ignored");
      continue;
    }
    va.init(0);
    for_int(i, el.num()) {
      int k = hp.enter(el[i].p);
      if (k == gva.num()) {
        Vertex v = mesh.create_vertex();
        mesh.set_point(v, el[i].p);
        gva.push(v);
        va.push(v);
      } else {
        assertx(gva.ok(k));
        va.push(gva[k]);
        Sppdist2.enter(dist2(el[i].p, mesh.point(gva[k])));
      }
    }
    if (!assertw(mesh.legal_create_face(va))) continue;
    mesh.create_face(va);
  }
}

void do_rawfroma3d() {
  RSA3dStream ia3d(std::cin);
  Array<Vertex> va;
  A3dElem el;
  string str;
  for (;;) {
    ia3d.read(el);
    if (el.type() == A3dElem::EType::endfile) break;
    if (el.type() == A3dElem::EType::comment) continue;
    if (el.type() != A3dElem::EType::polygon) {
      Warning("Non-polygon input ignored");
      continue;
    }
    va.init(0);
    for_int(i, el.num()) {
      Vertex v = mesh.create_vertex();
      mesh.set_point(v, el[i].p);
      const Vector& n = el[i].n;
      if (!is_zero(n)) mesh.update_string(v, "normal", csform_vec(str, n));
      const A3dColor& co = el[i].c.d;
      if (el[i].c.g[0]) mesh.update_string(v, "rgb", csform_vec(str, co));
      va.push(v);
    }
    mesh.create_face(va);
  }
}

// *** gmerge

// Note: vertex_id() should be called on mn, not mesh, but no difference in current implementation.
void normalize_arrayv(ArrayView<Vertex> ar) {
  assertx(ar.num() >= 3);
  int mini, minvid = std::numeric_limits<int>::max();
  dummy_init(mini);
  for_int(i, ar.num()) {
    int vid = mesh.vertex_id(ar[i]);
    if (vid < minvid) {
      minvid = vid;
      mini = i;
    }
  }
  Array<Vertex> art(ar);
  for_int(i, ar.num()) ar[i] = art[(mini + i) % ar.num()];
}

// Given mesh mo, merge the vertices, producing mesh mn.
GMesh geometric_merge(const GMesh& mo) {
  // Adapted from GMesh::merge() and do_froma3d().
  // Use bbox so that error between
  //  Vertex x 0.00502754 31.3495 30.7251
  //  Vertex x 0.00504071 31.3495 30.7251
  // is irrelevant if the bbox is of size 200.
  // Thus, gmerge is always more robust than do_froma3d().
  const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
  const Frame xform = bbox.get_frame_to_small_cube();
  GMesh mn;
  // Create new vertices.
  Map<Vertex, Vertex> mvvn;
  {
    HashPoint hp;
    Array<Vertex> gva;
    for (Vertex vo : mo.ordered_vertices()) {
      int i;
      if (bndmerge && !mo.num_boundaries(vo)) {
        i = -1;
      } else {
        Point p = mo.point(vo) * xform;
        i = hp.enter(p);
      }
      if (0) SHOW(i, mo.point(vo));
      Vertex vn;
      if (i == -1 || i == gva.num()) {
        vn = mn.create_vertex_private(mo.vertex_id(vo));
        mn.set_point(vn, mo.point(vo));
        mn.flags(vn) = mo.flags(vo);
        mn.set_string(vn, mo.get_string(vo));
        // mn.update_string(vn, "Ovi", csform(str, "%d", mn.vertex_id(vn)));
        if (i == gva.num()) gva.push(vn);
      } else {
        assertx(gva.ok(i));
        vn = gva[i];
        HH_SSTAT(Smerged, dist(mo.point(vo), mn.point(vn)));
      }
      mvvn.enter(vo, vn);
    }
    showdf("gmerge: will keep %d/%d vertices\n", mn.num_vertices(), mo.num_vertices());
  }
  // First pass: find faces with exact opposites and remove both.
  Set<Face> setbadof;
  {
    struct hash_ArrayVertex {
      size_t operator()(const Array<Vertex>& ar) const {
        size_t h = 0;
        for (Vertex v : ar) h = h * 13 + mesh.vertex_id(v);
        return h;
      }
    };
    Set<Array<Vertex>, hash_ArrayVertex> setarraynv;
    // Ordered to consistently keep first ok face.
    Array<Vertex> van;
    for (Face fo : mo.ordered_faces()) {
      van.init(0);
      for (Vertex vo : mo.vertices(fo)) van.push(mvvn.get(vo));
      normalize_arrayv(van);
      if (!setarraynv.add(van)) {
        Warning("Eliminating duplicate face");
        setbadof.enter(fo);
      }
    }
    for (Face fo : mo.faces()) {
      if (setbadof.contains(fo)) continue;
      van.init(0);
      for (Vertex vo : mo.vertices(fo)) van.push(mvvn.get(vo));
      reverse(van);
      normalize_arrayv(van);
      if (setarraynv.contains(van)) {
        Warning("Eliminating reverse-duplicate face");
        setbadof.enter(fo);
      }
    }
    showdf("gmerge: will keep %d/%d faces\n", mo.num_faces() - setbadof.num(), mo.num_faces());
  }
  // Second pass: add faces.
  Map<Face, Face> mffn;
  {
    Array<Vertex> van;
    // Ordered to keep faces in same order.
    for (Face fo : mo.ordered_faces()) {
      if (setbadof.contains(fo)) continue;
      van.init(0);
      for (Vertex vo : mo.vertices(fo)) {
        Vertex vn = mvvn.get(vo);
        if (van.num() && (van.last() == vn || van[0] == vn)) continue;
        van.push(vn);
      }
      if (van.num() < 3) continue;
      if (!assertw(mn.legal_create_face(van))) continue;
      Face fn = mn.create_face(van);
      mffn.enter(fo, fn);
      mn.flags(fn) = mo.flags(fo);
      mn.set_string(fn, mo.get_string(fo));
    }
  }
  // Add edge info.
  for (Edge eo : mo.edges()) {
    Edge en = mn.query_edge(mvvn.get(mo.vertex1(eo)), mvvn.get(mo.vertex2(eo)));
    if (!assertw(en)) continue;
    mn.flags(en) = mo.flags(eo);
    mn.set_string(en, mo.get_string(eo));
    if (mo.is_boundary(eo)) {
      mn.flags(en).flag(GMesh::eflag_sharp) = true;
      mn.update_string(en, "sharp", "");
    }
  }
  // Remove info from merged vertices.
  Map<Vertex, int> mvnn;
  {
    for (Vertex vn : mn.vertices()) mvnn.enter(vn, 0);
    for (Vertex vn : mvvn.values()) mvnn.replace(vn, mvnn.get(vn) + 1);
    for (Vertex vn : mn.vertices()) {
      assertx(mvnn.get(vn) > 0);
      if (mvnn.get(vn) > 1) mn.set_string(vn, nullptr);
    }
  }
  // Add corner info as necessary.
  Array<char> key, val;
  string str;
  for (Vertex vo : mo.vertices()) {
    Vertex vn = mvvn.get(vo);
    for (Corner co : mo.corners(vo)) {
      Face fo = mo.corner_face(co);
      Face fn = mffn.retrieve(fo);
      if (!fn) {
        Warning("corner no longer exists");
        continue;
      }
      Corner cn = mn.corner(vn, fn);
      if (mvnn.get(vn) == 1) {
        // vertex info already copied above
      } else {
        mn.set_string(cn, mo.get_string(vo));
        if (1) mn.update_string(cn, "Ovi", csform(str, "%d", mo.vertex_id(vo)));
      }
      for_cstring_key_value(mo.get_string(co), key, val, [&] { mn.update_string(cn, key.data(), val.data()); });
    }
  }
  return mn;
}

void do_gmerge() {
  GMesh nmesh = geometric_merge(mesh);
  mesh.copy(nmesh);
}

bool sharp_vertex_edge(const GMesh& mo, Vertex v, Edge e, bool split_matbnd) {
  Face f1 = mo.face1(e), f2 = mo.face2(e);
  if (!f2) return true;
  if (split_matbnd) return !same_string(mo.get_string(f1), mo.get_string(f2));
  if (mo.flags(e).flag(GMesh::eflag_sharp)) return true;
  // This may split unnecessarily, eg. based on "groups" key string!
  //  eg. you support multiple normals per vertex, but want colors split.
  if (0 && !same_string(mo.get_string(f1), mo.get_string(f2))) return true;
  if (!same_string(mo.get_string(mo.corner(v, f1)), mo.get_string(mo.corner(v, f2)))) return true;
  return false;
}

// Given mesh mo, split the corners, producing mesh mn.
GMesh split_corners(const GMesh& mo, bool split_matbnd) {
  int max_ovi = 0;
  string str;
  for (Vertex vo : mo.vertices()) {
    for (Corner corep : mo.corners(vo)) {
      const char* s = GMesh::string_key(str, mesh.get_string(corep), "Ovi");
      if (s) max_ovi = max(max_ovi, assertx(to_int(s)));
    }
  }
  GMesh mn;
  if (max_ovi) {
    // artificially raise mn._vertexnum
    Vertex v1 = mn.create_vertex_private(max_ovi + 1);
    Vertex v2 = mn.create_vertex_private(max_ovi + 2);
    mn.destroy_vertex(v1);
    mn.destroy_vertex(v2);
  }
  Map<Corner, Vertex> mcv;
  Map<Vertex, Vertex> mvv;  // only if unique
  Array<char> key, val;
  for (Vertex vo : mo.ordered_vertices()) {
    Set<Face> setfovis;
    int ncomp = 0;
    for (Corner corep : mo.corners(vo)) {
      Face forep = mesh.corner_face(corep);
      if (setfovis.contains(forep)) continue;
      ncomp++;
      Vertex vn;
      const char* s = GMesh::string_key(str, mesh.get_string(corep), "Ovi");
      int ovi = s ? assertx(to_int(s)) : 0;
      if (ovi && mn.id_retrieve_vertex(ovi)) {
        Warning("Ovi already present --- strange");
        vn = mn.create_vertex();
      } else if (ovi) {
        vn = mn.create_vertex_private(ovi);
      } else if (ncomp == 1 && !mn.id_retrieve_vertex(mesh.vertex_id(vo))) {
        vn = mn.create_vertex_private(mesh.vertex_id(vo));
      } else {
        if (max_ovi) Warning("Missing Ovi, or extra sharp edges?");
        vn = mn.create_vertex();
      }
      mn.flags(vn) = mo.flags(vo);
      mn.set_point(vn, mo.point(vo));
      mn.set_string(vn, mo.get_string(vo));
      if (!split_matbnd) {
        // Corner attributes no longer needed in new mesh.
        for_cstring_key_value(mo.get_string(mo.corner(vo, forep)), key, val, [&] {  //
          mn.update_string(vn, key.data(), val.data());
        });
      }
      Face fo = forep;
      for (;;) {  // find f: most_clw, or frep if closed
        Edge eo = mo.ccw_edge(fo, vo);
        if (sharp_vertex_edge(mo, vo, eo, split_matbnd)) break;
        fo = assertx(mo.opp_face(fo, eo));
        if (fo == forep) break;
      }
      forep = fo;
      for (;;) {  // now go ccw
        assertx(setfovis.add(fo));
        mcv.enter(mo.corner(vo, fo), vn);
        Edge eo = mo.clw_edge(fo, vo);
        if (sharp_vertex_edge(mo, vo, eo, split_matbnd)) break;
        fo = assertx(mo.opp_face(fo, eo));
        if (fo == forep) break;
      }
    }
    if (ncomp == 1) mvv.enter(vo, mcv.get(mo.corner(vo, mo.most_clw_face(vo))));
  }
  for (Face fo : mo.ordered_faces()) {
    Array<Vertex> van;
    for (Corner co : mo.corners(fo)) van.push(mcv.get(co));
    Face fn = mn.create_face(van);
    if (split_matbnd) {
      // Corners attributes within new vertex may be non-unique
      for (Corner co : mo.corners(fo)) {
        Corner cn = mesh.corner(mcv.get(co), fn);
        mesh.set_string(cn, mesh.get_string(co));
      }
    }
    mn.flags(fn) = mo.flags(fo);
    mn.set_string(fn, mo.get_string(fo));
  }
  for (Edge eo : mo.edges()) {
    Vertex vn1 = mvv.retrieve(mo.vertex1(eo));
    Vertex vn2 = mvv.retrieve(mo.vertex2(eo));
    if (vn1 && vn2) {
      Edge en = mn.edge(vn1, vn2);
      mn.flags(en) = mo.flags(eo);
      mn.set_string(en, mo.get_string(eo));
    }
  }
  return mn;
}

void do_splitcorners() {
  GMesh nmesh = split_corners(mesh, false);
  mesh.copy(nmesh);
}

void do_splitmatbnd() {
  GMesh nmesh = split_corners(mesh, true);
  mesh.copy(nmesh);
}

// Given mesh mo, split the corners, producing mesh mn.
GMesh split_corners_debug(const GMesh& mo) {
  GMesh mn;
  Map<Corner, Vertex> mcv;
  Map<Vertex, Vertex> mvv;  // only if unique
  for (Vertex vo : mo.ordered_vertices()) {
    Set<Face> setfovis;
    int ncomp = 0;
    for (Face forep : mo.faces(vo)) {
      if (setfovis.contains(forep)) continue;
      ncomp++;
      Vertex vn = mn.create_vertex();
      mn.set_point(vn, mo.point(vo));
      Face fo = forep;
      for (;;) {  // find f: most_clw, or frep if closed
        Edge eo = mo.ccw_edge(fo, vo);
        if (sharp_vertex_edge(mo, vo, eo, false)) break;
        fo = assertx(mo.opp_face(fo, eo));
        if (fo == forep) break;
      }
      forep = fo;
      for (;;) {  // now go ccw
        assertx(setfovis.add(fo));
        mcv.enter(mo.corner(vo, fo), vn);
        Edge eo = mo.clw_edge(fo, vo);
        if (sharp_vertex_edge(mo, vo, eo, false)) break;
        fo = assertx(mo.opp_face(fo, eo));
        if (fo == forep) break;
      }
    }
    if (ncomp == 1) mvv.enter(vo, mcv.get(mo.corner(vo, mo.most_clw_face(vo))));
  }
  Map<Edge, Vertex> mev;
  for (Edge eo : mo.edges()) {
    Vertex v = mn.create_vertex();
    mn.set_point(v, interp(mo.point(mo.vertex1(eo)), mo.point(mo.vertex2(eo))));
    mev.enter(eo, v);
  }
  for (Face fo : mo.ordered_faces()) {
    Array<Vertex> van;
    for (Corner co : mo.corners(fo)) {
      van.push(mcv.get(co));
      van.push(mev.get(mo.ccw_edge(fo, mo.corner_vertex(co))));
    }
    mn.create_face(van);
  }
  return mn;
}

void do_debugsplitcorners() {
  GMesh nmesh = split_corners_debug(mesh);
  mesh.copy(nmesh);
}

void do_checkcorners(Args& args) {
  const char* key = args.get_string().c_str();
  int nv_none = 0, nv_just_v = 0, nv_all_c = 0, nv_both_v_and_c = 0, nv_some_c = 0;
  for (Vertex v : mesh.vertices()) {
    const bool has_v = GMesh::string_has_key(mesh.get_string(v), key);
    int num_f = 0, num_c = 0;
    for (Corner c : mesh.corners(v)) {
      num_f++;
      num_c += GMesh::string_has_key(mesh.get_string(c), key);
    }
    if (!has_v) {
      if (!num_c)
        nv_none++;
      else if (num_c < num_f)
        nv_some_c++;
      else
        nv_all_c++;
    } else {
      if (!num_c)
        nv_just_v++;
      else
        nv_both_v_and_c++;
    }
  }
  showdf("Key=%-8s nv_none=%d nv_just_v=%d nv_all_c=%d  nv_both_v_and_c=%d nv_some_c=%d\n",  //
         key, nv_none, nv_just_v, nv_all_c, nv_both_v_and_c, nv_some_c);
  assertw(!nv_both_v_and_c);
  assertw(!nv_some_c);
}

void record_sharpe() {
  assertx(cosangle != k_undefined_cosangle);
  assertw(cosangle >= -1.f && cosangle <= 1.f);
  HH_STAT(Ssharp);
  HH_STAT(Ssmooth);
  HH_STAT(Sang);
  for (Edge e : mesh.edges()) {
    if (mesh.is_boundary(e)) continue;
    float angcos = edge_dihedral_angle_cos(mesh, e);
    float ang = deg_from_rad(my_acos(angcos));
    Sang.enter(ang);
    if (angcos > cosangle) {
      Ssmooth.enter(ang);
      mesh.flags(e).flag(GMesh::eflag_sharp) = false;
    } else {
      Ssharp.enter(ang);
      mesh.flags(e).flag(GMesh::eflag_sharp) = true;
    }
  }
}

void record_cuspv() {
  for (Vertex v : mesh.vertices()) {
    if (mesh.is_boundary(v)) continue;
    bool is_cusp = vertex_solid_angle(mesh, v) < solidangle;
    mesh.flags(v).flag(GMesh::vflag_cusp) = is_cusp;
  }
}

void do_angle(Args& args) {
  float angle = args.get_float();
  assertx(angle >= 0.f && angle <= 180.f);
  cosangle = std::cos(rad_from_deg(angle));
  record_sharpe();
}

void do_cosangle(Args& args) {
  cosangle = args.get_float();
  record_sharpe();
}

void do_solidangle(Args& args) {
  solidangle = args.get_float();
  assertx(solidangle >= 0.f && solidangle <= TAU * 2);
  record_cuspv();
}

void do_setb3d() { my_setenv("A3D_BINARY", "1"); }

// *** toa3d, tob3d

void do_toa3d() {
  HH_TIMER("_toa3d");
  nooutput = true;
  output_mesh_as_a3d();
}

void do_tob3d() {
  HH_TIMER("_tob3d");
  my_setenv("A3D_BINARY", "1");
  nooutput = true;
  output_mesh_as_a3d();
}

void do_endobject() { oa3d.write_end_object(); }

// *** other

void do_renumber() {
  HH_TIMER("_renumber");
  mesh.renumber();
}

void do_nidrenumberv() {
  HH_TIMER("_nidrenumberv");
  Set<Vertex> setv;
  for (Vertex v : mesh.vertices()) setv.enter(v);
  const int large = std::numeric_limits<int>::max() / 2;
  {
    int i = large;
    for (Vertex v : setv) mesh.vertex_renumber_id_private(v, i++);
  }
  string str;
  for (Vertex v : setv) {
    int nid = to_int(assertx(GMesh::string_key(str, mesh.get_string(v), "Nid")));
    assertx(nid > 0 && nid < large);
    mesh.vertex_renumber_id_private(v, nid);
  }
  for (Vertex v : mesh.vertices()) mesh.update_string(v, "Nid", nullptr);
}

void do_merge(Args& args) {
  HH_TIMER("_merge");
  int i = 0;
  for (;;) {
    if (!args.num()) break;
    if (args.peek_string()[0] == '-') break;
    string filename = args.get_filename();
    GMesh omesh;
    omesh.read(RFile(filename)());
    showdf("%s:\n", filename.c_str());
    showdf("  %s\n", mesh_genus_string(omesh).c_str());
    mesh.merge(omesh);
    i++;
  }
  assertw(i >= 2);
}

void do_outmesh() {
  HH_TIMER("_outmesh");
  mesh.write(std::cout);
}

void do_addmesh() {
  HH_TIMER("_addmesh");
  A3dElem el(A3dElem::EType::endfile);
  oa3d.write(el);
  mesh.write(std::cout);
}

void do_writemesh(Args& args) {
  string filename = args.get_filename();
  WFile fi(filename);
  mesh.write(fi());
}

void do_record() {
  nooutput = true;
  mesh.record_changes(&std::cout);
}

void do_mark() {
  for (Vertex v : mesh.vertices()) mesh.update_string(v, "cusp", mesh.flags(v).flag(GMesh::vflag_cusp) ? "" : nullptr);
  for (Edge e : mesh.edges()) mesh.update_string(e, "sharp", mesh.flags(e).flag(GMesh::eflag_sharp) ? "" : nullptr);
}

// *** delaunay

void do_delaunay() {
  HH_TIMER("_delaunay");
  assertx(cosangle != k_undefined_cosangle);
  int ns = retriangulate_all(mesh, cosangle, circum_radius_swap_criterion, nullptr, nullptr);
  showdf("Swapped %d edges\n", ns);
}

// *** diagonal

void do_diagonal() {
  HH_TIMER("_diagonal");
  assertx(cosangle != k_undefined_cosangle);
  int ns = retriangulate_all(mesh, cosangle, diagonal_distance_swap_criterion, nullptr, nullptr);
  showdf("Swapped %d edges\n", ns);
}

// *** segment

void gather_follow_seg(Face f, Map<Face, int>& mfseg, int segnum, int& pnf, Point& pcentroid) {
  int nf = 0;
  Homogeneous h;
  Queue<Face> queuef;
  mfseg.enter(f, segnum);
  Polygon poly;
  for (;;) {
    nf++;
    mesh.polygon(f, poly);
    h += poly.get_area() * Homogeneous(mean(poly));
    for (Edge e : mesh.edges(f)) {
      if (sharp(e)) continue;
      Face f2 = mesh.opp_face(f, e);
      bool is_new;
      mfseg.enter(f2, segnum, is_new);
      if (is_new) queuef.enqueue(f2);
    }
    if (queuef.empty()) break;
    f = queuef.dequeue();
  }
  if (!h[3]) {
    Warning("Segment has zero area");
    h[3] = 1.f;
  }
  pnf = nf;
  pcentroid = to_Point(normalized(h));
}

void gather_segments(Map<Face, int>& mfseg, Array<Face>& arepf) {
  assertx(mfseg.empty() && !arepf.num());
  HPqueue<Face> pq;
  {
    HH_STAT(Sseg);
    int segnum = 0;
    for (Face f : mesh.faces()) {
      if (mfseg.contains(f)) continue;
      segnum++;
      int nf;
      Point pc;
      gather_follow_seg(f, mfseg, segnum, nf, pc);
      Sseg.enter(nf);
      float pri = abs(pc[0] * 37.f + pc[1] * 17.f + pc[2]);
      pq.enter(f, pri);
    }
  }
  while (!pq.empty()) {
    Face f = pq.remove_min();
    arepf.push(f);
  }
}

void edge_angle_stats(const Map<Face, int>& mfseg) {
  HH_STAT(Sbndang);
  HH_STAT(Sintang);
  for (Edge e : mesh.edges()) {
    if (mesh.is_boundary(e)) continue;
    Face f1 = mesh.face1(e), f2 = mesh.face2(e);
    if (!assertw(mesh.is_triangle(f1) && mesh.is_triangle(f2))) continue;
    float angle = edge_signed_dihedral_angle(mesh, e);
    if (mfseg.get(f1) == mfseg.get(f2))
      Sintang.enter(angle);
    else
      Sbndang.enter(angle);
  }
}

void output_segment(Face f, Map<Face, int>& mfseg) {
  Queue<Face> queuef;
  assertx(mfseg.remove(f));
  A3dElem el;
  for (;;) {
    mesh.write_face(oa3d, el, k_surf_color, f);
    for (Edge e : mesh.edges(f)) {
      if (sharp(e)) {
        output_edge(e);
        continue;
      }
      Face f2 = mesh.opp_face(f, e);
      if (mfseg.remove(f2)) queuef.enqueue(f2);
    }
    if (queuef.empty()) break;
    f = queuef.dequeue();
  }
}

void do_segment() {
  HH_TIMER("_segment");
  nooutput = true;
  Map<Face, int> mfseg;  // Face -> segment number
  Array<Face> arepf;
  gather_segments(mfseg, arepf);
  edge_angle_stats(mfseg);
  showdf("Detected %d segments\n", arepf.num());
  for (Face f : arepf) {
    output_segment(f, mfseg);
    oa3d.write_end_object();
  }
  assertx(mfseg.empty());
}

// *** Record segment numbers

void record_segment(Face f, Map<Face, int>& mfseg) {
  Queue<Face> queuef;
  int segnum = mfseg.remove(f);
  assertx(segnum);
  string str;
  for (;;) {
    mesh.update_string(f, "segn", csform(str, "%d", segnum - 1));
    for (Edge e : mesh.edges(f)) {
      if (sharp(e)) continue;
      Face f2 = mesh.opp_face(f, e);
      if (mfseg.remove(f2)) queuef.enqueue(f2);
    }
    if (queuef.empty()) break;
    f = queuef.dequeue();
  }
}

void do_recordsegments() {
  HH_TIMER("_recordsegments");
  Map<Face, int> mfseg;  // Face -> segment number
  Array<Face> arepf;
  gather_segments(mfseg, arepf);
  showdf("Detected %d segments\n", arepf.num());
  for (Face f : arepf) record_segment(f, mfseg);
  assertx(mfseg.empty());
}

// *** selsmooth

void selectively_smooth() {
  assign_normals();
  Map<Face, int> mfseg;  // Face -> segment number
  Array<Face> arepf;
  gather_segments(mfseg, arepf);
  showdf("Detected %d segments\n", arepf.num());
  Queue<Face> queuef;
  A3dElem el;
  for (Face f : arepf) {
    assertx(mfseg.remove(f));
    oa3d.write_comment(" beginning of new segment");
    for (;;) {
      mesh.write_face(oa3d, el, k_surf_color, f);
      for (Edge e : mesh.edges(f)) {
        if (sharp(e)) continue;
        Face f2 = mesh.opp_face(f, e);
        if (mfseg.remove(f2)) queuef.enqueue(f2);
      }
      if (queuef.empty()) break;
      f = queuef.dequeue();
    }
  }
  oa3d.flush();
}

void do_selsmooth() {
  HH_TIMER("_selsmooth");
  nooutput = true;
  selectively_smooth();
}

bool edge_matbnd(Edge e) {
  if (1) {
    // Note: material boundary based on equivalence of strings.
    if (mesh.is_boundary(e)) return false;
    const char* s1 = mesh.get_string(mesh.face1(e));
    const char* s2 = mesh.get_string(mesh.face2(e));
    if (!s1 && !s2) return false;
    return !s1 || !s2 || strcmp(s1, s2) != 0;
  } else {
    // Note: material boundary is currently based solely on rgb attributes
    //   of faces!
    if (mesh.is_boundary(e)) return false;
    Vec3<float> f1, f2;
    if (!parse_key_vec(mesh.get_string(mesh.face1(e)), "rgb", f1) ||
        !parse_key_vec(mesh.get_string(mesh.face2(e)), "rgb", f2))
      return false;
    return f1 != f2;
  }
}

void do_showsharpe() {
  nooutput = true;
  int ne = 0;
  for (Edge e : mesh.edges()) {
    if (sharp(e)) {
      output_edge(e, A3dVertexColor(A3dColor(1.f, 1.f, 0.f)));
      ne++;
    }
  }
  showdf("Output %d sharp edges\n", ne);
}

void do_showmatbnd() {
  nooutput = true;
  int ne = 0;
  for (Edge e : mesh.edges()) {
    if (edge_matbnd(e)) {
      output_edge(e, A3dVertexColor(A3dColor(.5f, 1.f, .5f)));
      ne++;
    }
  }
  showdf("Output %d matbnd edges\n", ne);
}

void do_tagmateriale() {
  int nfound = 0, nfoundsharp = 0;
  for (Edge e : mesh.edges()) {
    if (mesh.is_boundary(e)) continue;
    bool is_matbnd = edge_matbnd(e);
    if (is_matbnd) nfound++;
    if (is_matbnd && sharp(e)) nfoundsharp++;
    mesh.flags(e).flag(GMesh::eflag_sharp) = is_matbnd;
  }
  showdf("Tagged %d material edges (of which %d were already sharp)\n", nfound, nfoundsharp);
}

// *** trisubdiv

// Create a new mesh in situ with the old vertices
// (each old vertex will temporarily have 2 rings of faces!).
void do_trisubdiv() {
  Warning("Older (simpler) rules than in Subdivfit (SubMesh)");
  HH_TIMER("_trisubdiv");
  Array<Vertex> arv(mesh.vertices());
  Array<Face> arf(mesh.faces());
  Array<Edge> are(mesh.edges());
  Map<Edge, Vertex> menewv;
  // Create new vertices and compute their positions.
  string str;
  for (Edge e : are) {
    Vertex v = mesh.create_vertex();
    menewv.enter(e, v);
    Homogeneous h = (3.f * Homogeneous(mesh.point(mesh.vertex1(e))) + 3.f * Homogeneous(mesh.point(mesh.vertex2(e))));
    if (!sharp(e)) {
      h += mesh.point(mesh.side_vertex1(e));
      h += mesh.point(mesh.side_vertex2(e));
    }
    mesh.set_point(v, to_Point(normalized(h)));
    // Added just uv support for
    //  Filtermesh brian615e.uv.m -angle 0 -trisubdiv -renumber >brian615e.subdiv.uv.m
    if (0) {
      Uv uv = interp(get_uv(mesh.vertex1(e)), get_uv(mesh.vertex2(e)));
      mesh.update_string(v, "uv", csform_vec(str, uv));
    }
  }
  // Update old vertex positions.
  Map<Vertex, Point> mapvp;
  for (Vertex v : arv) {
    int ne = 0, nesharp = 0;
    Homogeneous h, hsharp;
    for (Edge e : mesh.edges(v)) {
      const Point& p = mesh.point(mesh.opp_vertex(v, e));
      ne++;
      h += p;
      if (sharp(e)) {
        nesharp++;
        hsharp += p;
      }
    }
    if (nesharp >= 3) {  // corner: unmoved
      continue;
    } else if (nesharp == 2 && ne == 2) {  // boundary corner: unmoved
      continue;
    } else if (nesharp == 2) {  // crease: cubic bspline boundary
      h = hsharp + 6.f * Homogeneous(mesh.point(v));
    } else {  // interior: quartic bspline interior
      // (if dart (nesharp == 1), treat as interior vertex)
      int n = ne;
      float a = 5.f / 8.f - square((3.f + 2.f * std::cos(TAU / n)) / 8.f);
      float centerw = n * (1.f - a) / a;
      h += centerw * Homogeneous(mesh.point(v));
    }
    mapvp.enter(v, to_Point(normalized(h)));
  }
  for (auto& [v, p] : mapvp) mesh.set_point(v, p);
  // Create new triangulation.
  Array<Vertex> va;
  Vec3<Vertex> vs;
  for (Face f : arf) {
    mesh.get_vertices(f, va);
    assertx(va.num() == 3);
    for_int(i, 3) {
      Edge e = mesh.edge(va[i], va[mod3(i + 1)]);
      vs[i] = menewv.get(e);
    }
    Face ff;
    ff = mesh.create_face(vs[1], vs[2], vs[0]), mesh.set_string(ff, mesh.get_string(f));  // center
    ff = mesh.create_face(va[0], vs[0], vs[2]), mesh.set_string(ff, mesh.get_string(f));
    ff = mesh.create_face(vs[0], va[1], vs[1]), mesh.set_string(ff, mesh.get_string(f));
    ff = mesh.create_face(vs[2], vs[1], va[2]), mesh.set_string(ff, mesh.get_string(f));
  }
  // Update sharp edges.
  for (Edge e : are) {
    if (!mesh.flags(e).flag(GMesh::eflag_sharp)) continue;
    Vertex vnew = menewv.get(e);
    mesh.flags(mesh.edge(vnew, mesh.vertex1(e))).flag(GMesh::eflag_sharp) = true;
    mesh.flags(mesh.edge(vnew, mesh.vertex2(e))).flag(GMesh::eflag_sharp) = true;
  }
  // Remove old triangulation.
  for (Face f : arf) mesh.destroy_face(f);
}

// *** silsubdiv

void do_silsubdiv() {
  // e.g.: Filtermesh ~/data/mesh/cat.m -angle 40 -mark -silsubdiv -silsubdiv | G3d - -st cat -key De
  Warning("Older (simpler) rules than in Subdivfit (SubMesh)");
  HH_TIMER("_silsubdiv");
  const Array<Face> arf(mesh.faces());
  // Determine which edges will be subdivided.
  Set<Edge> subde;  // edges to subdivide
  for (Edge e : mesh.edges())
    if (sharp(e)) subde.enter(e);
  Queue<Face> queuef;
  for (Face f : arf) queuef.enqueue(f);
  while (!queuef.empty()) {
    Face f = queuef.dequeue();
    int nnew = 0;
    for (Edge e : mesh.edges(f))
      if (subde.contains(e)) nnew++;
    if (nnew != 2) continue;  // ok, no propagating changes
    for (Edge e : mesh.edges(f)) {
      if (!subde.add(e)) continue;
      Face f2 = mesh.opp_face(f, e);
      if (f2) queuef.enqueue(f2);
    }
  }
  // Introduce new vertices at midpoints.
  using PairVV = std::pair<Vertex, Vertex>;
  struct Vb {
    Vertex vnew;
    bool is_sharp;
  };
  Map<PairVV, Vb> mvv_nvb;  // -> (new vertex, is_sharp)
  for (Edge e : subde) {
    Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
    if (v2 < v1) std::swap(v1, v2);
    Vertex vn = mesh.create_vertex();
    mesh.set_point(vn, interp(mesh.point(v1), mesh.point(v2)));
    mvv_nvb.enter(PairVV(v1, v2), Vb{vn, mesh.flags(e).flag(GMesh::eflag_sharp)});
  }
  // Subdivide faces, destroys validity of flags(e).
  Array<Vertex> va;
  Vec3<Vertex> vs;
  for (Face f : arf) {
    int nnew = 0, i0 = -1;
    mesh.get_vertices(f, va);
    assertx(va.num() == 3);
    for_int(i, 3) {
      Edge e = mesh.edge(va[i], va[mod3(i + 1)]);
      Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
      if (v2 < v1) std::swap(v1, v2);
      bool present;
      const Vb& nvb = mvv_nvb.retrieve(PairVV(v1, v2), present);
      vs[i] = present ? nvb.vnew : nullptr;
      if (vs[i]) {
        nnew++;
        i0 = i;
      }
    }
    if (!nnew) continue;
    assertx(nnew == 1 || nnew == 3);
    unique_ptr<char[]> fstring = mesh.extract_string(f);  // may be nullptr
    mesh.destroy_face(f);
    if (nnew == 1) {
      int i1 = mod3(i0 + 1), i2 = mod3(i0 + 2);
      Face f1 = mesh.create_face(va[i0], vs[i0], va[i2]);
      mesh.set_string(f1, fstring.get());
      Face f2 = mesh.create_face(vs[i0], va[i1], va[i2]);
      mesh.set_string(f2, fstring.get());
    } else {
      for_int(i, 3) {
        Face fn = mesh.create_face(va[i], vs[i], vs[mod3(i + 2)]);
        mesh.set_string(fn, fstring.get());
      }
      Face fn = mesh.create_face(vs[0], vs[1], vs[2]);
      mesh.set_string(fn, fstring.get());
    }
  }
  for (auto& [vv, nvb] : mvv_nvb) {
    if (nvb.is_sharp) {
      mesh.flags(mesh.edge(nvb.vnew, vv.first)).flag(GMesh::eflag_sharp) = true;
      mesh.flags(mesh.edge(nvb.vnew, vv.second)).flag(GMesh::eflag_sharp) = true;
    }
  }
  // Update vertex positions.
  for (Vertex v : mesh.vertices()) {
    int nesharp = 0;
    Homogeneous h;
    for (Edge e : mesh.edges(v)) {
      if (sharp(e)) {
        nesharp++;
        h += mesh.point(mesh.opp_vertex(v, e));
      }
    }
    if (nesharp != 2) continue;
    // Important note: here order does not matter because newly
    //  inserted midpoints remain midpoints.
    mesh.set_point(v, to_Point(.5f * Homogeneous(mesh.point(v)) + .25f * h));
  }
}

// *** Taubin

void do_taubinsmooth(Args& args) {
  HH_TIMER("_taubinsmooth");
  int niter = args.get_int();
  float lambda, mu;
  if (0) {
    // My original guess from reading SIGGRAPH '95.
    lambda = 0.6307f;
    mu = -0.6307f;
  } else {
    // Explicit parameters used in ICCV '95.
    // Less shrinkage on an octahedron, but a bit less smoothing overall
    //  as seen on cat mesh.
    lambda = 0.33f;
    mu = -0.34f;
  }
  int nnewv = 0;
  for (Vertex v : mesh.vertices())
    if (GMesh::string_has_key(mesh.get_string(v), "newvertex")) nnewv++;
  if (nnewv) Warning("Only smoothing new vertices");
  Map<Vertex, Point> mvp;
  for (Vertex v : mesh.vertices()) mvp.enter(v, Point());
  // HH: introduced the factor * 2 on niter on 1999-01-04.
  for_int(i, niter * 2) {
    float disp = i % 2 == 0 ? lambda : mu;
    for (Vertex v : mesh.vertices()) {
      Homogeneous h;
      int n = 0;
      for (Vertex vv : mesh.vertices(v)) {
        h += mesh.point(vv);
        n++;
      }
      h += float(-n) * Homogeneous(mesh.point(v));
      assertx(n);
      h /= float(n);
      Vector vec = to_Vector(h) * disp;
      if (nnewv && !GMesh::string_has_key(mesh.get_string(v), "newvertex")) vec = Vector(0.f, 0.f, 0.f);
      Point p = mesh.point(v) + vec;
      mvp.get(v) = p;
    }
    for (Vertex v : mesh.vertices()) mesh.set_point(v, mvp.get(v));
  }
}

// *** Desbrun

// Return cotangent of angle about point p1 in triangle, or BIGFLOAT if triangle is degenerate about p1.
float cotan(const Point& p1, const Point& p2, const Point& p3) {
  Vector v = p2 - p1, w = p3 - p1;
  // cos(ang) = dot(v, w) / (mag(v) * mag(w))
  // sin(ang) = mag(cross(v, w)) / (mag(v) * mag(w))
  // -> cot(ang) = dot(v, w) / mag(cross(v, w))
  float vcos = dot(v, w);
  float vsin = mag(cross(v, w));
  if (!vsin) Warning("cotan: found degenerate triangle");
  return !vsin ? BIGFLOAT : vcos / vsin;
}

void do_desbrunsmooth(Args& args) {
  HH_TIMER("_desbrunsmooth");
  float lambda = args.get_float();
  assertx(lambda > 0.f);
  const bool use_taubin_laplacian = false;
  Array<Vertex> a_v;
  Map<Vertex, int> m_vi;
  for (Vertex v : mesh.vertices()) {
    m_vi.enter(v, a_v.num());
    a_v.push(v);
  }
  SparseLls lls(a_v.num(), a_v.num(), 3);
  lls.set_tolerance(1e-6f);
  lls.set_verbose(1);
  Array<int> nei_vi;
  Array<float> nei_w;
  for (Vertex v : mesh.vertices()) {
    if (mesh.is_boundary(v)) {
      Warning("Boundary vertex not smoothed");
      lls.enter_a_rc(m_vi.get(v), m_vi.get(v), 1.f);
      lls.enter_b_r(m_vi.get(v), mesh.point(v));
      continue;
    }
    float sum_area = 0.f;
    for (Face f : mesh.faces(v)) sum_area += mesh.area(f);
    assertx(sum_area > 0.f);
    float sum_w = 0.f;
    nei_vi.init(0);
    nei_w.init(0);
    for (Vertex vv : mesh.vertices(v)) {  // unsorted!
      nei_vi.push(m_vi.get(vv));
      float w;
      if (use_taubin_laplacian) {
        w = 1.f;
      } else {
        Vertex vp = mesh.clw_vertex(v, vv);
        Vertex vn = mesh.ccw_vertex(v, vv);
        float cotp = cotan(mesh.point(vp), mesh.point(vv), mesh.point(v));
        float cotn = cotan(mesh.point(vn), mesh.point(v), mesh.point(vv));
        w = cotn + cotp;
        if (w < 0.f) Warning("Desbrun: negative edge weight, hope OK");
        if (cotp == BIGFLOAT || cotn == BIGFLOAT) {
          Warning("Degenerate triangle");
          // OK to have two edges each have BIGFLOAT / (2 * BIGFLOAT) == .5f weight.
          if (0) w = 0.f;
        }
      }
      nei_w.push(w);
      sum_w += w;
    }
    assertx(sum_w);
    if (0) {  // this makes lambda scale-dependent (bad)
      for_int(j, nei_vi.num()) nei_w[j] /= (4.f * sum_area);
      sum_w /= (4.f * sum_area);
    } else {  // "normalized" version from Section 5.5
      for_int(j, nei_vi.num()) nei_w[j] /= sum_w;
      sum_w = 1.f;
    }
    lls.enter_a_rc(m_vi.get(v), m_vi.get(v), 1.f + lambda * sum_w);
    for_int(j, nei_vi.num()) lls.enter_a_rc(m_vi.get(v), nei_vi[j], -lambda * nei_w[j]);
    lls.enter_b_r(m_vi.get(v), mesh.point(v));
    lls.enter_xest_r(m_vi.get(v), mesh.point(v));
  }
  assertx(lls.solve());
  for_int(i, a_v.num()) {
    Point p;
    lls.get_x_r(i, p);
    mesh.set_point(a_v[i], p);
  }
}

// *** LSCM parameterization

Vertex farthest_vertex(CArrayView<Vertex> bndverts, Vertex v0) {
  assertx(bndverts.num() >= 2);
  Vertex vret = nullptr;
  float maxval = -BIGFLOAT;
  for (Vertex v : bndverts) {
    float val = dist(mesh.point(v), mesh.point(v0));
    if (val > maxval) {
      maxval = val;
      vret = v;
    }
  }
  assertx(vret);
  return vret;
}

// finds an approximation
Vec2<Vertex> find_diameter_of_boundary_vertices() {
  Vec2<Vertex> vb(nullptr, nullptr);
  Array<Vertex> bndverts;
  for (Vertex v : mesh.vertices())
    if (mesh.is_boundary(v)) bndverts.push(v);
  assertx(bndverts.num() >= 2);
  if (0) {
    Vertex v0 = bndverts[0];
    vb[0] = farthest_vertex(bndverts, v0);
    vb[1] = farthest_vertex(bndverts, vb[0]);
  } else {
    float maxval = -BIGFLOAT;
    for_int(iter, 100) {
      Vec2<Vertex> vbt;
      {
        Vector dir;
        for_int(c, 3) dir[c] = Random::G.unif() * 2 - 1;
        assertx(dir.normalize());
        float mindot = BIGFLOAT, maxdot = -BIGFLOAT;
        Vertex vmin = nullptr, vmax = nullptr;
        for (Vertex v : bndverts) {
          float vdot = dot(mesh.point(v), dir);
          if (vdot < mindot) {
            mindot = vdot;
            vmin = v;
          }
          if (vdot > maxdot) {
            maxdot = vdot;
            vmax = v;
          }
        }
        vbt[0] = vmin;
        vbt[1] = vmax;
      }
      for (;;) {
        bool progress = false;
        for_int(c, 2) {
          Vertex vnew = farthest_vertex(bndverts, vbt[c]);
          if (vnew != vbt[1 - c]) {
            progress = true;
            vbt[1 - c] = vnew;
          }
        }
        if (!progress) break;
      }
      float val = dist(mesh.point(vbt[0]), mesh.point(vbt[1]));
      if (val > maxval) {
        maxval = val;
        vb[0] = vbt[0];
        vb[1] = vbt[1];
      }
    }
  }
  assertx(vb[0] && vb[1] && vb[1] != vb[0]);
  SHOW(dist(mesh.point(vb[0]), mesh.point(vb[1])));
  if (1) {
    mesh.update_string(vb[0], "cusp", "");
    mesh.update_string(vb[1], "cusp", "");
  }
  return vb;
}

// Filtermesh ~/prevproj/2004/_poissonparam/data/cathead.m -lscm | G3duv -
void do_lscm() {
  HH_TIMER("_lscm");
  assertx(mesh_single_disk());
  int m = (2 + mesh.num_faces()) * 2;
  int n = mesh.num_vertices() * 2;
  Array<Vertex> a_v;
  Map<Vertex, int> m_vi;
  for (Vertex v : mesh.vertices()) {
    m_vi.enter(v, a_v.num());
    a_v.push(v);
  }
  SparseLls lls(m, n, 1);
  lls.set_verbose(1);
  // lls.set_tolerance(1e-8f);
  Vec2<Vertex> vb = find_diameter_of_boundary_vertices();
  {
    float w = sqrt(mesh_area());
    lls.enter_a_rc(0, m_vi.get(vb[0]) * 2 + 0, w);
    lls.enter_b_rc(0, 0, w * 0.0f);
    lls.enter_a_rc(1, m_vi.get(vb[0]) * 2 + 1, w);
    lls.enter_b_rc(1, 0, w * 0.5f);
    lls.enter_a_rc(2, m_vi.get(vb[1]) * 2 + 0, w);
    lls.enter_b_rc(2, 0, w * 1.0f);
    lls.enter_a_rc(3, m_vi.get(vb[1]) * 2 + 1, w);
    lls.enter_b_rc(3, 0, w * 0.5f);
  }
  {
    int i = 4;
    for (Face f : mesh.faces()) {
      float w = sqrt(mesh.area(f));
      const Vec3<Vertex> va = mesh.triangle_vertices(f);
      const Vec3<Point> triangle = mesh.triangle_points(f);
      Vec2<Vector> vsa;
      vsa[0] = normalized(triangle[1] - triangle[0]);
      const Vector nor = normalized(cross(vsa[0], Vector(triangle[2] - triangle[0])));
      vsa[1] = cross(vsa[0], nor);
      assertx(is_unit(vsa[1]));
      const Vec2<Bary> barya{bary_of_vector(triangle, vsa[0]), bary_of_vector(triangle, vsa[1])};
      for_int(d0, 2) {
        int d1 = 1 - d0;
        float s0 = !d0 ? 1.f : -1.f, s1 = 1.f;
        for_int(j, 3) {
          lls.enter_a_rc(i, m_vi.get(va[j]) * 2 + 0, w * s0 * barya[d0][j]);
          lls.enter_a_rc(i, m_vi.get(va[j]) * 2 + 1, w * s1 * barya[d1][j]);
        }
        lls.enter_b_rc(i, 0, 0.0f);
        i++;
      }
    }
    assertx(i == lls.num_rows());
  }
  for_int(i, a_v.num()) {
    Uv uv(0.f, 0.f);
    lls.enter_xest_rc(i * 2 + 0, 0, uv[0]);
    lls.enter_xest_rc(i * 2 + 1, 0, uv[1]);
  }
  assertx(lls.solve());
  string str;
  for_int(i, a_v.num()) {
    Uv uv;
    uv[0] = lls.get_x_rc(i * 2 + 0, 0);
    uv[1] = lls.get_x_rc(i * 2 + 1, 0);
    mesh.update_string(a_v[i], "uv", csform_vec(str, uv));
  }
}

// *** Poisson parameterization

// Filtermesh ~/prevproj/2004/_poissonparam/data/cathead.m -lscm -poisson | G3duv -
void do_poissonparam() {
  HH_TIMER("_poissonparam");
  assertx(mesh_single_disk());
  int m = 2 + mesh.num_faces() * 4;
  int n = mesh.num_vertices() * 2;
  Array<Vertex> a_v;
  Map<Vertex, int> m_vi;
  for (Vertex v : mesh.vertices()) {
    m_vi.enter(v, a_v.num());
    a_v.push(v);
  }
  SparseLls lls(m, n, 1);
  lls.set_verbose(1);
  // lls.set_tolerance(1e-8f);
  Vertex v0;
  Uv uv0;
  dummy_init(v0);
  {
    float minval = BIGFLOAT;
    for (Vertex v : mesh.vertices()) {
      Uv uv = get_uv(v);
      float val = float(dot(uv, Uv(1.f, 1.f)));
      if (val < minval) {
        minval = val;
        v0 = v;
        uv0 = uv;
      }
    }
    // SHOW(get_uv(v0));
  }
  float gscale = 1.f / sqrt(mesh_area());
  {
    float w = sqrt(mesh_area());
    lls.enter_a_rc(0, m_vi.get(v0) * 2 + 0, w);
    lls.enter_b_rc(0, 0, w * 0.f);
    lls.enter_a_rc(1, m_vi.get(v0) * 2 + 1, w);
    lls.enter_b_rc(1, 0, w * 0.f);
  }
  {
    int i = 2;
    for (Face f : mesh.faces()) {
      const float w = sqrt(mesh.area(f));
      const Vec3<Vertex> va = mesh.triangle_vertices(f);
      const Vec3<Point> triangle = mesh.triangle_points(f);
      const Vec3<Uv> uva = map(va, get_uv);
      for_int(dir, 2) {
        const int d0 = dir, d1 = 1 - dir;
        const Uv vecd = twice(0.f).with(d0, 1.f);
        Bary bary = bary_of_vector(convert<Vec2<float>>(uva), vecd);
        const Vector vecs = vector_from_bary(triangle, bary);
        const float len = mag(vecs);
        assertx(len);
        for_int(k, 3) bary[k] /= len;
        for_int(j, 3) lls.enter_a_rc(i, m_vi.get(va[j]) * 2 + d0, w * bary[j]);
        lls.enter_b_rc(i, 0, w * 1.f * gscale);
        i++;
        for_int(j, 3) lls.enter_a_rc(i, m_vi.get(va[j]) * 2 + d1, w * bary[j]);
        lls.enter_b_rc(i, 0, w * 0.f);
        i++;
      }
    }
    assertx(i == lls.num_rows());
  }
  for_int(i, a_v.num()) {
    Uv uv = get_uv(a_v[i]);
    uv[0] -= uv0[0];
    uv[1] -= uv0[1];
    if (1) fill(uv, 0.f);
    lls.enter_xest_rc(i * 2 + 0, 0, uv[0]);
    lls.enter_xest_rc(i * 2 + 1, 0, uv[1]);
  }
  assertx(lls.solve());
  string str;
  for_int(i, a_v.num()) {
    Uv uv;
    uv[0] = lls.get_x_rc(i * 2 + 0, 0);
    uv[1] = lls.get_x_rc(i * 2 + 1, 0);
    mesh.update_string(a_v[i], "uv", csform_vec(str, uv));
  }
}

// *** fillholes

void do_fillholes(Args& args) {
  HH_TIMER("_fillholes");
  int maxnume = args.get_int();  // == maxnumv
  Set<Edge> setbe;
  for (Edge e : mesh.edges())
    if (mesh.is_boundary(e)) setbe.enter(e);
  HH_STAT(Sbndlen);  // Length of each boundary edge loop.
  HH_STAT(Sbndsub);  // Number of sub-holes, i.e., number of multisided faces created for each boundary edge loop.
  while (!setbe.empty()) {
    Edge e = setbe.get_one();
    const char* es = mesh.get_string(mesh.face1(e));
    Queue<Edge> queuee = gather_boundary(mesh, e);
    for (Edge ee : queuee) assertx(setbe.remove(ee));
    int ne = queuee.length();
    if (ne > maxnume) continue;
    Sbndlen.enter(ne);
    Set<Face> setf = mesh_remove_boundary(mesh, e);
    Sbndsub.enter(setf.num());
    for (Face f : setf) mesh.set_string(f, es);
    if (getenv_bool("WRITE_HOLE"))
      for (Face f : setf) mesh.update_string(f, "hole", "");
    if (getenv_bool("HOLE_SHARP"))
      for (Face f : setf)
        for (Edge ee : mesh.edges(f)) mesh.update_string(ee, "sharp", "");
    if (0)
      for (Face f : setf) showdf(" filling in hole with %d sides\n", mesh.num_vertices(f));
  }
  showdf("Filled in %d holes\n", Sbndlen.inum());
}

// *** triangulate

void do_triangulate() {
  HH_STAT(Sfverts);
  Stack<Face> stackf;
  for (Face f : mesh.faces()) {
    int nv = mesh.num_vertices(f);
    if (nv == 3 && !alltriangulate) continue;
    Sfverts.enter(nv);
    stackf.push(f);
  }
  showdf("Found %d faces to triangulate\n", stackf.height());
  Polygon poly;
  Array<Vertex> va;
  while (!stackf.empty()) {
    Face f = stackf.pop();
    bool is_flat = false;
    if (checkflat) {
      mesh.polygon(f, poly);
      Vector nor = poly.get_normal();
      float d = poly.get_planec(nor);
      float tol = poly.get_tolerance(nor, d);
      if (tol) tol /= my_sqrt(poly.get_area());
      is_flat = tol < checkflat;
    }
    if (is_flat) {
      Warning("face flat enough, so triangulated without centersplit");
      mesh.get_vertices(f, va);
      if (mesh.query_edge(va[0], va[2])) {
        Warning("Edge already exists, so give up and do centersplit");
        mesh.center_split_face(f);
        continue;
      }
      Edge enew = mesh.split_face(f, va[0], va[2]);
      for (Face fnew : mesh.faces(enew))
        if (mesh.num_vertices(fnew) > 3) stackf.push(fnew);
    } else {
      mesh.center_split_face(f);
    }
  }
  if (!Sfverts.num()) Sfverts.set_print(false);
}

void do_splitvalence(Args& args) {
  int maxvalence = args.get_int();
  assertx(maxvalence > 6);
  const bool write_hole = getenv_bool("WRITE_HOLE");
  HPqueue<Vertex> pqv;
  int large_int = 1 << 24;
  for (Vertex v : mesh.vertices())
    if (mesh.degree(v) >= maxvalence) pqv.enter(v, float(large_int - mesh.degree(v)));
  int nsplit = 0;
  while (!pqv.empty()) {
    Vertex v = pqv.remove_min();
    assertx(mesh.degree(v) >= maxvalence);
    HH_SSTAT(Sval, mesh.degree(v));
    nsplit++;
    assertx(!mesh.is_boundary(v));  // not implemented
    const bool is_hole =
        write_hole && all_of(mesh.faces(v), [&](Face f) { return GMesh::string_has_key(mesh.get_string(f), "hole"); });
    Array<Vertex> va(mesh.ccw_vertices(v));
    Vector vec{};
    for_int(i, va.num()) vec += (mesh.point(va[i]) - mesh.point(v)) * std::sin(float(i) / va.num() * TAU);
    Vertex vs1 = va[0];
    Vertex vs2 = va[va.num() / 2];
    Vertex vn = mesh.split_vertex(v, vs1, vs2, 0);
    mesh.create_face(v, vn, vs1);
    mesh.create_face(v, vs2, vn);
    if (is_hole)
      for (Face f : mesh.faces(mesh.edge(v, vn))) mesh.update_string(f, "hole", "");
    mesh.set_point(vn, mesh.point(v) - vec * 1e-3f);
    mesh.update_string(vn, "newvertex", "");
    mesh.update_string(v, "newvertex", "");
    pqv.remove(vs1);
    pqv.remove(vs2);
    if (mesh.degree(vs1) >= maxvalence) pqv.enter(vs1, float(large_int - mesh.degree(vs1)));
    if (mesh.degree(vs2) >= maxvalence) pqv.enter(vs2, float(large_int - mesh.degree(vs2)));
    if (mesh.degree(v) >= maxvalence) pqv.enter(v, float(large_int - mesh.degree(v)));
    if (mesh.degree(vn) >= maxvalence) pqv.enter(vn, float(large_int - mesh.degree(vn)));
  }
  showdf("Split high-valence vertices %d times\n", nsplit);
}

void do_splitbnd2valence() {
  Set<Vertex> setv;
  for (Vertex v : mesh.vertices())
    if (mesh.degree(v) == 2) setv.enter(v);
  SHOW(setv.num());
  for (Vertex v : setv) {
    if (!assertw(mesh.degree(v) == 2)) continue;
    Vertex v1 = mesh.most_ccw_vertex(v);
    assertx(v1);
    Vertex v2 = mesh.most_clw_vertex(v);
    assertx(v2);
    Vertex vs2 = mesh.clw_vertex(v2, v1);
    assertx(vs2 && vs2 != v);
    Vertex vn = mesh.split_vertex(v2, v, vs2, 0);
    assertx(vn);
    mesh.set_point(vn, interp(mesh.point(v1), mesh.point(v2)));
    mesh.create_face(v2, vn, v);
    mesh.create_face(v2, vs2, vn);
  }
}

// *** triangulate_quads

int sqrt_nfaces;

enum class ETriType { dshort, dlong, alternating, xuvdiag, duvdiag, odddiag };

void triangulate_quads(ETriType type) {
  Stack<Face> stackf;
  for (Face f : mesh.faces())
    if (mesh.num_vertices(f) == 4) stackf.push(f);
  showdf("Found %d quads to triangulate\n", stackf.height());
  Array<Vertex> va;
  while (!stackf.empty()) {
    Face f = stackf.pop();
    mesh.get_vertices(f, va);
    assertx(va.num() == 4);
    Vertex va0 = va[0], va2 = va[2];
    bool other_diag;
    const auto uv_distance_from_center = [&](int i) {
      Uv uv = get_uv(va[i]);
      return abs(uv[0] - .5f) + abs(uv[1] - .5f);
    };
    switch (type) {
      case ETriType::dshort: {  // Use GIM rule to select shorter diagonal.
        const float gim_diagonal_factor = 1.0f;
        other_diag = (dist2(mesh.point(va[0]), mesh.point(va[2])) >
                      dist2(mesh.point(va[1]), mesh.point(va[3])) * square(gim_diagonal_factor));
        break;
      }
      case ETriType::dlong: {  // Use opposite rule
        const float gim_diagonal_factor = 1.0f;
        other_diag = (dist2(mesh.point(va[0]), mesh.point(va[2])) <=
                      dist2(mesh.point(va[1]), mesh.point(va[3])) * square(gim_diagonal_factor));
        break;
      }
      case ETriType::alternating: {  // Use alternating diagonals
        int fid = mesh.face_id(f) - 1;
        int iy = fid / sqrt_nfaces;
        int ix = fid - iy * sqrt_nfaces;
        other_diag = (ix + iy) % 2 == 1;
        break;
      }
      case ETriType::xuvdiag: {  // Use X shaped diagonal pattern on [0..1][0..1] domain
        const int maxi = arg_max(transform(range(4), uv_distance_from_center));
        other_diag = maxi == 1 || maxi == 3;
        break;
      }
      case ETriType::duvdiag: {  // Use diamond shaped diagonal pattern on [0..1][0..1] domain
        const int maxi = arg_max(transform(range(4), uv_distance_from_center));
        other_diag = maxi == 0 || maxi == 2;
        break;
      }
      case ETriType::odddiag: {  // Create odd-valence vertices
        int fid = mesh.face_id(f) - 1;
        int iy = fid / sqrt_nfaces;
        int ix = fid - iy * sqrt_nfaces;
        other_diag = (iy % 2) == 1 && (ix % 2) == 1;
        break;
      }
      default: assertnever("");
    }
    if (other_diag) {
      va0 = va[1];
      va2 = va[3];
    }
    if (mesh.query_edge(va0, va2)) {
      Warning("Diagonal already exists; choosing other one");
      if (other_diag) {
        va0 = va[0];
        va2 = va[2];
      } else {
        va0 = va[1];
        va2 = va[3];
      }
    }
    if (mesh.query_edge(va0, va2)) {
      Warning("Both diagonals exist; not triangulating quad");
      continue;
    }
    mesh.split_face(f, va0, va2);
  }
}

void do_quadshortdiag() { triangulate_quads(ETriType::dshort); }

void do_quadlongdiag() { triangulate_quads(ETriType::dlong); }

void do_quadaltdiag() {
  sqrt_nfaces = int(sqrt(mesh.num_faces() + .01f));
  assertx(mesh.num_faces() == square(sqrt_nfaces));
  triangulate_quads(ETriType::alternating);
}

void do_quadxuvdiag() { triangulate_quads(ETriType::xuvdiag); }

void do_quadduvdiag() { triangulate_quads(ETriType::duvdiag); }

void do_quadodddiag() {
  sqrt_nfaces = int(sqrt(mesh.num_faces() + .01f));
  assertx(mesh.num_faces() == square(sqrt_nfaces));
  triangulate_quads(ETriType::odddiag);
}

// *** rmcomponents

void remove_component(const Set<Face>& setf) {
  Set<Vertex> setv;
  for (Face f : setf) {
    for (Vertex v : mesh.vertices(f)) setv.add(v);  // May already be there.
    mesh.destroy_face(f);
  }
  for (Vertex v : setv)
    if (!mesh.degree(v)) mesh.destroy_vertex(v);
}

void remove_isolated_vertices() {
  Array<Vertex> vdestroy;
  for (Vertex v : mesh.vertices())
    if (!mesh.degree(v)) vdestroy.push(v);
  for (Vertex v : vdestroy) mesh.destroy_vertex(v);
  if (vdestroy.num()) showdf("Removed %d isolated vertices\n", vdestroy.num());
}

void do_rmcomp(Args& args) {
  HH_TIMER("_rmcomp");
  int maxnumf = args.get_int();
  const Array<Set<Face>> components = gather_components(mesh);
  HH_STAT(Sfacesrem);
  for (const Set<Face>& setf : components) {
    const int nf = setf.num();
    if (nf >= maxnumf) continue;
    Sfacesrem.enter(nf);
    remove_component(setf);
  }
  showdf("Removed %d out of %d mesh components\n", Sfacesrem.inum(), components.num());
  remove_isolated_vertices();
}

void do_rmcompn(Args& args) {
  HH_TIMER("_rmcompn");
  const int ncomp = args.get_int();
  HH_STAT(Sfacesrem);
  const Array<Set<Face>> components = gather_components(mesh);
  const int num_to_remove = max(components.num() - ncomp, 0);
  for_int(i, num_to_remove) {
    const Set<Face>& setf = components[i];
    const int nf = setf.num();
    Sfacesrem.enter(nf);
    remove_component(setf);
  }
  showdf("Removed %d out of %d mesh components\n", Sfacesrem.inum(), components.num());
  remove_isolated_vertices();
}

// *** coalesce

// Return BIGFLOAT if not legal.
float try_coalesce(Edge e) {
  if (mesh.is_boundary(e)) return BIGFLOAT;
  if (!mesh.legal_coalesce_faces(e)) return BIGFLOAT;
  Array<Point> pa;
  for (Face f : mesh.faces(e))
    for (Vertex v : mesh.vertices(f)) pa.push(mesh.point(v));
  Frame frame;
  Vec3<float> eimag;
  principal_components(pa, frame, eimag);
  // ratio of thickness to width
  // width judged better than length (two adjacent skinny triangles)
  return eimag[2] / eimag[1];
}

// This version of coalesce has a weakness.  It only attempts to merge 2 faces
// by removing one consecutive set of edges common to them.
// But, after several coalescences, 2 faces may share more than one such set.
void do_coalesce(Args& args) {
  HH_TIMER("_coalesce");
  float fcrit = args.get_float();
  int nerem = 0;
  Set<Edge> sete;
  for (Edge e : mesh.edges()) sete.enter(e);
  while (!sete.empty()) {
    Edge e = sete.remove_one();
    {
      float f = try_coalesce(e);
      if (f > fcrit) continue;
    }
    // All systems go.
    // Neighboring edges may change, so remove them from sete
    //  (especially if > 1 common edges between 2 faces!).
    for (Face f : mesh.faces(e))
      for (Edge ee : mesh.edges(f)) sete.remove(ee);
    Face fnew = mesh.coalesce_faces(e);
    nerem++;
    // Reenter affected edges.
    for (Edge ee : mesh.edges(fnew)) assertx(sete.add(ee));
  }
  showdf("Removed %d edges\n", nerem);
}

void do_makequads(Args& args) {
  HH_TIMER("_makequads");
  float p_tol = args.get_float();
  int nerem = 0;
  int nf = mesh.num_faces();
  HPqueue<Edge> pqe;
  for (Edge e : mesh.edges()) {
    if (mesh.is_boundary(e)) continue;
    if (!mesh.is_triangle(mesh.face1(e)) || !mesh.is_triangle(mesh.face2(e))) {
      Warning("non-triangles not considered in makequads");
      continue;
    }
    // Test co-planarity.
    float angcos = edge_dihedral_angle_cos(mesh, e);
    float deviation = 1.f - angcos;
    if (deviation > p_tol) continue;  // not co-planar enough
    // Test shape of resulting quad.
    const Point& p1 = mesh.point(mesh.vertex1(e));
    const Point& p2 = mesh.point(mesh.vertex2(e));
    const Point& po1 = mesh.point(mesh.side_vertex1(e));
    const Point& po2 = mesh.point(mesh.side_vertex2(e));
    if (dihedral_angle_cos(po1, po2, p2, p1) < 0.f) continue;  // quad not convex
    const float smalloffset = 1.f;
    pqe.enter(e, deviation + smalloffset);
  }
  while (!pqe.empty()) {
    Edge e = pqe.remove_min();
    // All systems go.
    // Remove neighboring edges from pqe.
    for (Face f : mesh.faces(e))
      for (Edge ee : mesh.edges(f)) pqe.remove(ee);
    // Coalesce faces into new face fnew.
    Face fnew = mesh.coalesce_faces(e);
    dummy_use(fnew);
    nerem++;
  }
  showdf("Applied %d coalesces: (%d tris) -> (%d tris, %d quads)\n", nerem, nf, nf - nerem, nerem);
}

void do_cornermerge() {
  Array<char> key, val;
  int nmerge = 0;
  for (Vertex v : mesh.vertices()) {
    const char* s = nullptr;
    for (Corner c : mesh.corners(v)) {
      if (!mesh.get_string(c)) {
        s = nullptr;
        break;
      }
      if (!s) {
        s = mesh.get_string(c);
      } else if (strcmp(s, mesh.get_string(c)) != 0) {
        s = nullptr;
        break;
      }
    }
    if (!s) continue;
    nmerge++;
    for_cstring_key_value(s, key, val, [&] {
      assertx(!GMesh::string_has_key(mesh.get_string(v), key.data()));
      mesh.update_string(v, key.data(), val.data());
    });
    for (Corner c : mesh.corners(v)) mesh.set_string(c, nullptr);
  }
  showdf("at %d/%d vertices, merged corner strings\n", nmerge, mesh.num_vertices());
}

void do_slowcornermerge() {
  string s;
  Array<char> key, val;
  int nmerge = 0;
  string str;
  for (Vertex v : mesh.vertices()) {
    // get a copy s of the string for one of the corners
    s = "";
    for (Corner c : mesh.corners(v)) {
      const char* s2 = mesh.get_string(c);
      if (s2) s = s2;
      if (1) break;  // "if (1)" to avoid warning about unreachable code
    }
    for_cstring_key_value(s.c_str(), key, val, [&] {
      bool all_same = true;
      for (Corner c : mesh.corners(v)) {
        const char* s2 = GMesh::string_key(str, mesh.get_string(c), key.data());
        if (!s2 || strcmp(val.data(), s2) != 0) {
          all_same = false;
          break;
        }
      }
      if (!all_same) return;
      assertx(!GMesh::string_has_key(mesh.get_string(v), key.data()));
      nmerge++;
      mesh.update_string(v, key.data(), val.data());
      for (Corner c : mesh.corners(v)) mesh.update_string(c, key.data(), nullptr);
    });
  }
  showdf("slowcornermerge merged %d keys\n", nmerge);
}

void do_cornerunmerge() {
  Array<char> key, val;
  for (Vertex v : mesh.vertices()) {
    const char* s = mesh.get_string(v);
    if (!s) continue;
    for (Corner c : mesh.corners(v)) {
      if (!mesh.get_string(c)) {
        mesh.set_string(c, s);
      } else {
        for_cstring_key_value(s, key, val, [&] { mesh.update_string(c, key.data(), val.data()); });
      }
    }
    mesh.set_string(v, nullptr);
  }
}

void do_rgbvertexmerge() {
  int nmerge = 0;
  string str;
  for (Vertex v : mesh.vertices()) {
    const char* s = nullptr;
    for (Corner c : mesh.corners(v)) {
      const char* ck = mesh.corner_key(str, c, "rgb");
      // if (!ck) { s = nullptr; break; }
      if (!ck) continue;
      if (!s) {
        s = ck;
      } else if (strcmp(s, ck) != 0) {
        s = nullptr;
        break;
      }
    }
    if (!s) continue;
    nmerge++;
    mesh.update_string(v, "rgb", s);
    for (Corner c : mesh.corners(v)) mesh.update_string(c, "rgb", nullptr);
  }
  showdf("at %d/%d vertices, merged rgb strings\n", nmerge, mesh.num_vertices());
}

void do_facemerge() {
  int nmerge = 0;
  for (Face f : mesh.faces()) {
    if (mesh.get_string(f)) continue;
    const char* s = nullptr;
    for (Corner c : mesh.corners(f)) {
      if (!mesh.get_string(c)) {
        s = nullptr;
        break;
      }
      if (!s) {
        s = mesh.get_string(c);
      } else if (strcmp(s, mesh.get_string(c)) != 0) {
        s = nullptr;
        break;
      }
    }
    if (!s) continue;
    nmerge++;
    mesh.set_string(f, s);
    for (Corner c : mesh.corners(f)) mesh.set_string(c, nullptr);
  }
  showdf("at %d/%d faces, merged corner strings\n", nmerge, mesh.num_faces());
}

void do_rgbfacemerge() {
  int nmerge = 0;
  string str;
  for (Face f : mesh.faces()) {
    const char* s = nullptr;
    for (Corner c : mesh.corners(f)) {
      const char* ck = mesh.corner_key(str, c, "rgb");
      if (!ck) {
        s = nullptr;
        break;
      }
      if (!s) {
        s = ck;
      } else if (strcmp(s, ck) != 0) {
        s = nullptr;
        break;
      }
    }
    if (!s) continue;
    nmerge++;
    mesh.update_string(f, "rgb", s);
    for (Corner c : mesh.corners(f)) mesh.update_string(c, "rgb", nullptr);
  }
  showdf("at %d/%d faces, merged rgb strings\n", nmerge, mesh.num_faces());
}

// *** other

void do_nice() { assertx(mesh.is_nice()); }

void do_flip() {
  // Loses all flags, and face strings, and face_ids, and vertex_ids.
  Array<Array<Vertex>> arva;
  arva.reserve(mesh.num_faces());
  for (Face f : mesh.ordered_faces()) {
    arva.add(1);
    mesh.get_vertices(f, arva.last());
  }
  for (Face f : Array<Face>(mesh.faces())) mesh.destroy_face(f);
  for (Array<Vertex>& va : arva) {
    reverse(va);
    mesh.create_face(va);
  }
  mesh.renumber();  // compact the vertex indices; why?
}

void do_fixvertices() {
  Array<Vertex> arv;
  for (Vertex v : mesh.vertices())
    if (!mesh.is_nice(v)) arv.push(v);
  HH_STAT(Svnrings);
  for (Vertex v : arv) {
    Array<Vertex> new_vertices = mesh.fix_vertex(v);
    Svnrings.enter(new_vertices.num() + 1);
  }
  showdf("Fixed %d vertices\n", Svnrings.inum());
}

void do_fixfaces() {
  Array<Face> arf;
  for (Face f : mesh.faces())
    if (!mesh.is_nice(f)) arf.push(f);
  int nf = 0;
  for (Face f : arf) {
    if (mesh.is_nice(f)) continue;  // other face may be gone now
    nf++;
    mesh.destroy_face(f);
  }
  showdf("Fixed %d faces\n", nf);
}

void do_smootha3d() {
  HH_TIMER("_smootha3d");
  nooutput = true;
  // Not too efficient because normals are completely shared at vertices.
  selectively_smooth();
}

void do_bbox() {
  const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
  showdf("Bbox %g %g %g  %g %g %g\n", bbox[0][0], bbox[0][1], bbox[0][2], bbox[1][0], bbox[1][1], bbox[1][2]);
  nooutput = true;
}

void do_tobbox() {
  const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
  const Frame xform = bbox.get_frame_to_cube();
  showdf("Applying xform: %s", FrameIO::create_string(ObjectFrame{xform, 1}).c_str());
  for (Vertex v : mesh.vertices()) mesh.set_point(v, mesh.point(v) * xform);
}

void do_genus() { showdf("%s\n", mesh_genus_string(mesh).c_str()); }

void do_removeinfo() {
  for (Vertex v : mesh.vertices()) {
    mesh.set_string(v, nullptr);
    mesh.flags(v) = 0;
  }
  for (Face f : mesh.faces()) {
    mesh.set_string(f, nullptr);
    mesh.flags(f) = 0;
  }
  for (Edge e : mesh.edges()) {
    mesh.set_string(e, nullptr);
    mesh.flags(e) = 0;
  }
  for (Face f : mesh.faces())
    for (Corner c : mesh.corners(f)) mesh.set_string(c, nullptr);
}

void do_removekey(Args& args) {
  string s_key = args.get_string();
  const char* key = s_key.c_str();
  for (Vertex v : mesh.vertices()) mesh.update_string(v, key, nullptr);
  for (Face f : mesh.faces()) mesh.update_string(f, key, nullptr);
  for (Edge e : mesh.edges()) mesh.update_string(e, key, nullptr);
  for (Face f : mesh.faces())
    for (Corner c : mesh.corners(f)) mesh.update_string(c, key, nullptr);
  if (!strcmp(key, "sharp"))
    for (Edge e : mesh.edges()) mesh.flags(e).flag(GMesh::eflag_sharp) = false;
  if (!strcmp(key, "cusp"))
    for (Edge v : mesh.edges()) mesh.flags(v).flag(GMesh::vflag_cusp) = false;
}

void do_renamekey(Args& args) {
  string elems = args.get_string();
  assertx(elems.find_first_not_of("vfec") == string::npos);
  string s_okey = args.get_string();
  const char* okey = s_okey.c_str();
  string s_nkey = args.get_string();
  const char* nkey = s_nkey == "" ? nullptr : s_nkey.c_str();
  string str;
  if (contains(elems, 'v')) {
    if (s_okey == "P") {
      for (Vertex v : mesh.vertices()) {
        const Point& p = mesh.point(v);
        mesh.update_string(v, nkey, csform_vec(str, p));
      }
    } else if (s_nkey == "P") {
      for (Vertex v : mesh.vertices()) {
        const char* s = assertx(GMesh::string_key(str, mesh.get_string(v), okey));
        assertx(*s++ == '(');
        Point p{};
        for_int(c, 3) {
          p[c] = float_from_chars(s);
          if (*s == ')') break;  // We support value vectors of length 1, 2, or 3.
          assertx(*s++ == ' ');
        }
        assertx(*s++ == ')');
        assert_no_more_chars(s);
        mesh.set_point(v, p);
        mesh.update_string(v, okey, nullptr);
      }
    } else {
      for (Vertex v : mesh.vertices()) {
        if (nkey) mesh.update_string(v, nkey, GMesh::string_key(str, mesh.get_string(v), okey));
        mesh.update_string(v, okey, nullptr);
      }
    }
  }
  if (contains(elems, 'f')) {
    for (Face f : mesh.faces()) {
      if (nkey) mesh.update_string(f, nkey, GMesh::string_key(str, mesh.get_string(f), okey));
      mesh.update_string(f, okey, nullptr);
    }
  }
  if (contains(elems, 'e')) {
    for (Edge e : mesh.edges()) {
      if (nkey) mesh.update_string(e, nkey, GMesh::string_key(str, mesh.get_string(e), okey));
      mesh.update_string(e, okey, nullptr);
    }
  }
  if (contains(elems, 'c')) {
    for (Face f : mesh.faces()) {
      for (Corner c : mesh.corners(f)) {
        if (nkey) mesh.update_string(c, nkey, GMesh::string_key(str, mesh.get_string(c), okey));
        mesh.update_string(c, okey, nullptr);
      }
    }
  }
}

void do_copykey(Args& args) {
  string elems = args.get_string();
  assertx(elems.find_first_not_of("vfec") == string::npos);
  string s_okey = args.get_string();
  const char* okey = s_okey.c_str();
  string s_nkey = args.get_string();
  const char* nkey = s_nkey.c_str();
  string str;
  if (contains(elems, 'v'))
    for (Vertex v : mesh.vertices()) mesh.update_string(v, nkey, GMesh::string_key(str, mesh.get_string(v), okey));
  if (contains(elems, 'f'))
    for (Face f : mesh.faces()) mesh.update_string(f, nkey, GMesh::string_key(str, mesh.get_string(f), okey));
  if (contains(elems, 'e'))
    for (Edge e : mesh.edges()) mesh.update_string(e, nkey, GMesh::string_key(str, mesh.get_string(e), okey));
  if (contains(elems, 'c'))
    for (Face f : mesh.faces())
      for (Corner c : mesh.corners(f)) mesh.update_string(c, nkey, GMesh::string_key(str, mesh.get_string(c), okey));
}

void do_assignkey(Args& args) {
  string elems = args.get_string();
  assertx(elems.find_first_not_of("vfec") == string::npos);
  string s_key = args.get_string();
  const char* key = s_key.c_str();
  string s_value = args.get_string();
  const char* value = s_value.c_str();
  if (contains(elems, 'v'))
    for (Vertex v : mesh.vertices()) mesh.update_string(v, key, value);
  if (contains(elems, 'f'))
    for (Face f : mesh.faces()) mesh.update_string(f, key, value);
  if (contains(elems, 'e'))
    for (Edge e : mesh.edges()) mesh.update_string(e, key, value);
  if (contains(elems, 'c'))
    for (Face f : mesh.faces())
      for (Corner c : mesh.corners(f)) mesh.update_string(c, key, value);
}

const char* copy_normal_to_rgb(string& str, const char* s) {
  if (!s) return nullptr;
  Vector nor;
  if (!parse_key_vec(s, "normal", nor)) return nullptr;
  A3dColor col;
  for_int(c, 3) {
    assertx(nor[c] >= -1.f && nor[c] <= +1.f);
    col[c] = (nor[c] + 1.f) * .5f;
    assertx(col[c] >= 0.f && col[c] <= 1.f);
  }
  return csform_vec(str, col);
}

void do_copynormaltorgb() {
  string str;
  for (Vertex v : mesh.vertices()) mesh.update_string(v, "rgb", copy_normal_to_rgb(str, mesh.get_string(v)));
  for (Face f : mesh.faces())
    for (Corner c : mesh.corners(f)) mesh.update_string(c, "rgb", copy_normal_to_rgb(str, mesh.get_string(c)));
}

void do_info() {
  showdf("%s\n", mesh_genus_string(mesh).c_str());
  {
    HH_STAT(Sbound);
    Sbound.add(mesh_stat_boundaries(mesh));
  }
  {
    HH_STAT(Scompf);
    Scompf.add(mesh_stat_components(mesh));
  }
  {
    HH_STAT(Snormsolida);
    HH_STAT(Ssolidang);
    for (Vertex v : mesh.vertices()) {
      if (mesh.num_boundaries(v) || !mesh.degree(v)) continue;
      float solidang = vertex_solid_angle(mesh, v);
      Ssolidang.enter(solidang);
      Snormsolida.enter(1.f - solidang / TAU);
    }
  }
  {
    HH_STAT(Sfacearea);
    for (Face f : mesh.faces()) Sfacearea.enter(mesh.area(f));
    showdf("Area is %g\n", Sfacearea.sum());
  }
  {
    HH_STAT(Selen);
    for (Edge e : mesh.edges()) Selen.enter(mesh.length(e));
  }
  {
    HH_STAT(Sfvertices);
    for (Face f : mesh.faces()) Sfvertices.enter(mesh.num_vertices(f));
  }
  {
    HH_STAT(Sbvalence);
    HH_STAT(Sivalence);
    HH_STAT(Svalence);
    for (Vertex v : mesh.vertices()) {
      Svalence.enter(mesh.degree(v));
      if (!mesh.is_nice(v) || !mesh.degree(v)) continue;
      if (mesh.is_boundary(v))
        Sbvalence.enter(mesh.degree(v));
      else
        Sivalence.enter(mesh.degree(v));
    }
  }
  {
    double vol = 0.;
    bool have_all_triangles = true;
    // To make volume meaningful on mesh with boundaries, use centroid.
    Point centroid(0.f, 0.f, 0.f);
    if (!mesh.empty()) {
      Homogeneous h;
      for (Vertex v : mesh.vertices()) h += mesh.point(v);
      centroid = to_Point(normalized(h));
    }
    Polygon poly;
    for (Face f : mesh.faces()) {
      mesh.polygon(f, poly);
      if (poly.num() != 3) {
        have_all_triangles = false;
        continue;
      }
      vol += dot(cross(poly[0] - centroid, poly[1] - centroid), poly[2] - centroid);
    }
    vol /= 6.f;  // divide by factorial(ndimensions)
    if (have_all_triangles)
      showdf("Volume is %g\n", vol);
    else
      showdf("Non-triangular faces; volume not computed.\n");
  }
  {
    HH_STAT(Sfinscribedr);
    for (Face f : mesh.faces()) {
      if (mesh.is_triangle(f)) {
        const Vec3<Point> triangle = mesh.triangle_points(f);
        Sfinscribedr.enter(inscribed_radius(triangle[0], triangle[1], triangle[2]));
      }
    }
  }
  {
    const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
    showdf("Bbox %g %g %g  %g %g %g\n", bbox[0][0], bbox[0][1], bbox[0][2], bbox[1][0], bbox[1][1], bbox[1][2]);
  }
  {
    HH_STAT(Sdiha);
    for (Edge e : mesh.edges()) {
      if (mesh.is_boundary(e)) continue;
      float angle = edge_signed_dihedral_angle(mesh, e);
      if (angle == -10.f) {
        Warning("Edge dihedral undefined next to degenerate face");
        angle = 0.f;
      }
      Sdiha.enter(angle);
    }
  }
}

void do_stat() {
  do_info();
  nooutput = true;
}

Point get_dp(Vertex v) {
  Point dp;
  assertx(parse_key_vec(mesh.get_string(v), "domainp", dp));
  return dp;
}

// Possibly split spherical triangle into two pieces.
void do_obtusesplit() {
  if (1) assertnever("abandoned for now");
  bool is_sphere = true;
  for (Vertex v : mesh.vertices()) {
    if (!is_unit(mesh.point(v))) {
      is_sphere = false;
      break;
    }
  }
  float maxelen = 0.f;
  for (Edge e : mesh.edges()) maxelen = max(maxelen, mesh.length(e));
  maxelen *= 1.1f;
  is_sphere = false;  // ?
  // TAU / 4 would be critical point in plane for infinite recursion. actually, 1.3f seems to already cause problems.
  const float thresh_ang = rad_from_deg(135.f);  // TAU * (3.f / 8.f)
  HPqueue<Edge> pqe;
  pqe.reserve(mesh.num_edges());
  for (Edge e : mesh.edges()) pqe.enter_unsorted(e, maxelen - mesh.length(e));
  int nsplit = 0;
  pqe.sort();
  string str;
  while (!pqe.empty()) {
    Edge e = pqe.remove_min();
    const bool want_split = [&]() {
      for (Face ff : mesh.faces(e)) {
        Point po = mesh.point(mesh.opp_vertex(e, ff));
        Point p1 = mesh.point(mesh.vertex1(e));
        Point p2 = mesh.point(mesh.vertex2(e));
        Vector vto1, vto2;
        if (is_sphere) {  // Spherical angle within triangle.
          vto1 = project_orthogonally(p1 - po, po);
          vto2 = project_orthogonally(p2 - po, po);
        } else {
          vto1 = p1 - po;
          vto2 = p2 - po;
        }
        if (!assertw(vto1.normalize() && vto2.normalize())) continue;
        const float ang = angle_between_unit_vectors(vto1, vto2);
        if (ang > thresh_ang) return true;
      }
      return false;
    }();
    if (!want_split) continue;
    // ALL GO.
    Vec2<Vertex> va{mesh.vertex1(e), mesh.vertex2(e)};
    for (Face f : mesh.faces(e))
      for (Edge ee : mesh.edges(f)) pqe.remove(ee);
    Vertex vnew = mesh.split_edge(e);
    e = nullptr;
    nsplit++;
    if (nsplit > 500) break;  // ?
    float bary0 = .5f;
    Point pint = interp(mesh.point(va[0]), mesh.point(va[1]), bary0);
    if (is_sphere) pint = ok_normalized(pint);
    mesh.set_point(vnew, pint);
    if (GMesh::string_has_key(mesh.get_string(va[0]), "uv")) {
      Uv uv = interp(get_uv(va[0]), get_uv(va[1]), bary0);
      mesh.update_string(vnew, "uv", csform_vec(str, uv));
    }
    if (GMesh::string_has_key(mesh.get_string(va[0]), "domainp")) {
      Point dp = interp(get_dp(va[0]), get_dp(va[1]), bary0);
      mesh.update_string(vnew, "domainp", csform_vec(str, dp));
    }
    for (Face f : mesh.faces(vnew))
      for (Edge ee : mesh.edges(f))
        if (pqe.retrieve(ee) < 0.f) pqe.enter(ee, maxelen - mesh.length(ee));
  }
  showdf("obtusesplit split %d edges on %s\n", nsplit, is_sphere ? "sphere" : "surface");
}

void do_analyzestretch() {
  HH_TIMER("_analyzestretch");
  bool is_sphere = true;
  if (getenv_bool("PLANAR_STRETCH")) {
    showdf("Using PLANAR_STRETCH\n");
    is_sphere = false;
  }
  for (Vertex v : mesh.vertices()) {
    if (!is_unit(mesh.point(v))) {
      is_sphere = false;
      break;
    }
  }
  showf("Analyzing stretch for %s surface\n", is_sphere ? "sphere" : "mesh");
  double d_l2_integ_stretch = 0.;
  HH_STAT(Stri_isotropy);
  HH_STAT(Stri_li);
  HH_STAT(Stri_l2);
  HH_STAT(Stri_maxsv);
  HH_STAT(Stri_minsv);
  HH_STAT(Ssurfarea);
  HH_STAT(Sstarea);
  int num_domainp = 0, num_uv = 0;
  for (Face f : mesh.faces()) {
    const Vec3<Vertex> va = mesh.triangle_vertices(f);
    const Vec3<Point> triangle = mesh.triangle_points(f);
    Vec3<Uv> uvs;
    if (GMesh::string_has_key(mesh.get_string(va[0]), "domainp")) {
      num_domainp += 1;
      Vec3<Point> triangle_d;
      for_int(i, 3) assertx(parse_key_vec(mesh.get_string(va[i]), "domainp", triangle_d[i]));
      const Vector v01 = triangle_d[1] - triangle_d[0], v01n = normalized(v01), v02 = triangle_d[2] - triangle_d[0];
      uvs[0] = Uv(0.f, 0.f);
      uvs[1] = Uv(mag(v01), 0.f);
      uvs[2] = Uv(dot(v02, v01n), mag(v02 - v01n * dot(v02, v01n)));
    } else {
      num_uv += 1;
      for_int(i, 3) uvs[i] = get_uv(va[i]);
    }
    Vector dfds, dfdt;
    // compute_derivatives(va, uv, dfds, dfdt);
    float starea = float((double(uvs[1][0]) - uvs[0][0]) * (double(uvs[2][1]) - uvs[0][1]) -
                         (double(uvs[2][0]) - uvs[0][0]) * (double(uvs[1][1]) - uvs[0][1])) /
                   2.f;
    assertw(starea != 0.f);
    if (starea < 0.f) {
      Warning("Negating negative starea");
      starea = -starea;
    }
    Sstarea.enter(starea);
    float surfarea = is_sphere ? spherical_triangle_area(triangle) : sqrt(area2(triangle));
    Ssurfarea.enter(surfarea);
    float recip_area = .5f / starea;
    dfds = (triangle[0] * (uvs[1][1] - uvs[2][1]) + triangle[1] * (uvs[2][1] - uvs[0][1]) +
            triangle[2] * (uvs[0][1] - uvs[1][1])) *
           recip_area;
    dfdt = (triangle[0] * (uvs[2][0] - uvs[1][0]) + triangle[1] * (uvs[0][0] - uvs[2][0]) +
            triangle[2] * (uvs[1][0] - uvs[0][0])) *
           recip_area;
    double a = dot(dfds, dfds), b = dot(dfds, dfdt), c = dot(dfdt, dfdt);
    double tsqrt = my_sqrt(square(a - c) + 4. * square(b));
    float minsv2 = float(((a + c) - tsqrt) * .5);
    float maxsv2 = float(((a + c) + tsqrt) * .5);
    float minsv = my_sqrt(minsv2);
    float maxsv = my_sqrt(maxsv2);
    Stri_minsv.enter(minsv);
    Stri_maxsv.enter(maxsv);
    float l2_stretch = my_sqrt((minsv2 + maxsv2) * .5f);
    float li_stretch = maxsv;
    Stri_l2.enter(l2_stretch);
    Stri_li.enter(li_stretch);
    d_l2_integ_stretch += surfarea * square(l2_stretch);
    Stri_isotropy.enter(maxsv / minsv);
  }
  showdf("Inferred stretch domain from %d 'domainp' and %d 'uv'.\n", num_domainp, num_uv);
  float starea = Sstarea.sum();
  float surfarea = Ssurfarea.sum();
  float l2_integ_stretch = float(d_l2_integ_stretch);
  float rms_stretch = sqrt(l2_integ_stretch / surfarea);
  float l2_efficiency = surfarea / starea / square(rms_stretch);
  float li_efficiency = surfarea / starea / square(Stri_li.max());
  float max_area_ratio = Ssurfarea.max() / Ssurfarea.min();
  showdf("starea=%g surfarea=%g rms_stretch=%g\n", starea, surfarea, rms_stretch);
  showdf("Stretch: li=%.3f l2=%.3f maxani=%-5.4g avgani=%-5.4g arear=%-5.4g\n",  //
         li_efficiency, l2_efficiency, Stri_isotropy.max(), Stri_isotropy.avg(), max_area_ratio);
}

void do_renormalizenor() {
  const float thresh = 1e-6f;
  int renor = 0;
  string str;
  for (Vertex v : mesh.vertices()) {
    Vector nor;
    if (!parse_key_vec(mesh.get_string(v), "normal", nor)) continue;
    if (abs(mag2(nor) - 1.f) < thresh) continue;
    renor++;
    assertw(nor.normalize());
    mesh.update_string(v, "normal", csform_vec(str, nor));
  }
  for (Face f : mesh.faces()) {
    for (Corner c : mesh.corners(f)) {
      Vector nor;
      if (!parse_key_vec(mesh.get_string(c), "normal", nor)) continue;
      if (abs(mag2(nor) - 1.f) < thresh) continue;
      renor++;
      assertw(nor.normalize());
      mesh.update_string(c, "normal", csform_vec(str, nor));
    }
  }
  showdf("Renormalized %d normals\n", renor);
}

// *** reduce

float reduce_criterion(Edge e) {
  // Create a function attribute_safe_edge_collapse() and check it?
  if (!mesh.nice_edge_collapse(e)) return BIGFLOAT;
  switch (reducecrit) {
    case EReduceCriterion::length: return mesh.length(e);
    case EReduceCriterion::inscribed: return collapse_edge_inscribed_criterion(mesh, e);
    case EReduceCriterion::volume: return collapse_edge_volume_criterion(mesh, e);
    case EReduceCriterion::qem: return collapse_edge_qem_criterion(mesh, e);
    default: assertnever("");
  }
}

void do_reduce() {
  HH_TIMER("_reduce");
  assertx(reducecrit != EReduceCriterion::undefined);  // Also use: nfaces, maxcrit.
  HPqueue<Edge> pqe;
  {
    HH_TIMER("__initpq");
    Array<Edge> ar_edge(mesh.edges());
    Array<float> ar_cost(ar_edge.num());
    parallel_for(range(ar_edge.num()), [&](int i) { ar_cost[i] = reduce_criterion(ar_edge[i]); });
    for_int(i, ar_edge.num()) { pqe.enter_unsorted(ar_edge[i], ar_cost[i]); }
    pqe.sort();
  }
  const int orig_nf = mesh.num_faces();
  Set<Edge> edges_to_update;
  ConsoleProgress cprogress;
  for (;;) {
    assertx(pqe.num() + edges_to_update.num() == mesh.num_edges());
    cprogress.update(float(orig_nf - mesh.num_faces()) / max(orig_nf - nfaces, 1));
    if (mesh.num_faces() <= nfaces) break;
    const float fraction = 0.6f;
    if ((pqe.min_priority() > maxcrit || pqe.num() < max(int(mesh.num_edges() * fraction), 500)) &&
        edges_to_update.num()) {
      Array<Edge> ar_edge(edges_to_update);
      Array<float> ar_cost(ar_edge.num());
      if (0) sort(ar_edge);  // No resulting improvement in memory access coherence.
      // With parallelism, 1.3x faster than original version; without parallelism, 1.6x slower.
      parallel_for(range(ar_edge.num()), [&](int i) { ar_cost[i] = reduce_criterion(ar_edge[i]); });
      for_int(i, ar_edge.num()) pqe.enter(ar_edge[i], ar_cost[i]);
      edges_to_update.clear();
    }
    if (pqe.min_priority() > maxcrit && !edges_to_update.num()) break;
    Edge e = pqe.remove_min();
    for (Vertex v : mesh.vertices(e))
      for (Vertex vv : mesh.vertices(v))
        for (Edge e2 : mesh.edges(vv))
          if (pqe.remove(e2) < 0.f) edges_to_update.remove(e2);
    Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
    const bool isb1 = mesh.is_boundary(v1), isb2 = mesh.is_boundary(v2);
    const int ii = isb1 && !isb2 ? 2 : isb2 && !isb1 ? 0 : 1;  // ii == 2 : v1;  ii == 0 : v2
    const Point newp = interp(mesh.point(v1), mesh.point(v2), ii * .5f);
    // Use attribute info to determine which vertex to keep?
    Vertex vkept = ii == 0 ? v2 : v1;
    mesh.collapse_edge_vertex_saving_attribs(e, vkept);
    e = nullptr;
    mesh.set_point(vkept, newp);
    for (Vertex v : mesh.vertices(vkept))
      for (Edge e2 : mesh.edges(v)) edges_to_update.add(e2);
  }
  cprogress.clear();
}

void do_normalized_maxcrit(Args& args) {
  const float value = args.get_float();
  const float avg_elen = float(mean(transform(mesh.edges(), [](Edge e) { return mesh.length(e); })));
  maxcrit = value * square(avg_elen);
}

void do_lengthc() { reducecrit = EReduceCriterion::length; }

void do_inscribedc() { reducecrit = EReduceCriterion::inscribed; }

void do_volumec() { reducecrit = EReduceCriterion::volume; }

void do_qemc() { reducecrit = EReduceCriterion::qem; }

// *** Point sampling.

void output_point(const Point& p, const Vector& n) {
  A3dElem el(A3dElem::EType::point, 0);
  el.push(A3dVertex(p, n, A3dVertexColor(Pixel::black())));
  oa3d.write(el);
}

void do_randpts(Args& args) {
  HH_TIMER("_randpts");
  nooutput = true;
  int npoints = args.get_int();
  int nf = mesh.num_faces();
  Array<Face> fface;  // Face of this index
  fface.reserve(nf);
  Array<float> farea;  // area of face
  farea.reserve(nf);
  Array<float> fcarea;  // cumulative area
  fcarea.reserve(nf + 1);
  for (Face f : mesh.faces()) {
    if (!assertw(mesh.is_triangle(f))) continue;
    fface.push(f);
    farea.push(mesh.area(f));
  }
  float sumarea = float(sum(farea));
  showdf("Total area %g over %d faces\n", sumarea, nf);
  {
    double area = 0.;  // for accuracy
    for_int(i, nf) {
      fcarea.push(float(area));
      area += farea[i] / sumarea;
    }
    assertx(abs(area - 1.f) < 1e-5f);
  }
  fcarea.push(1.00001f);
  Map<Vertex, Vnors> mvnors;
  for (Vertex v : mesh.vertices()) mvnors.enter(v, Vnors(mesh, v));
  for_int(i, npoints) {
    int fi = discrete_binary_search(fcarea, 0, nf, Random::G.unif());
    Face f = fface[fi];
    Bary bary(Random::G.unif(), Random::G.unif(), 0.f);
    if (bary[0] + bary[1] > 1.f) {
      bary[0] = 1.f - bary[0];
      bary[1] = 1.f - bary[1];
    }
    bary[2] = 1.f - bary[0] - bary[1];
    Vec3<Vertex> va = mesh.triangle_vertices(f);
    Vec3<Point> triangle = mesh.triangle_points(f);
    Point p = interp(triangle, bary);
    Vector nor{};
    for_int(j, 3) nor += mvnors.get(va[j]).get_nor(f) * bary[j];
    assertx(nor.normalize());
    output_point(p, nor);
  }
  showdf("Printed %d random points\n", npoints);
}

void do_vertexpts() {
  HH_TIMER("_vertexpts");
  nooutput = true;
  for (Vertex v : mesh.vertices()) {
    Vnors vnors(mesh, v);
    Vector nor = vnors.is_unique() ? vnors.unique_nor() : Vector(0.f, 0.f, 0.f);
    output_point(mesh.point(v), nor);
  }
  showdf("Printed %d vertex points\n", mesh.num_vertices());
}

void do_orderedvertexpts() {
  HH_TIMER("_orderedvertexpts");
  nooutput = true;
  for (Vertex v : mesh.ordered_vertices()) {
    Vnors vnors(mesh, v);
    Vector nor = vnors.is_unique() ? vnors.unique_nor() : Vector(0.f, 0.f, 0.f);
    output_point(mesh.point(v), nor);
  }
  showdf("Printed %d vertex points\n", mesh.num_vertices());
}

void do_bndpts(Args& args) {
  HH_TIMER("_bndpts");
  nooutput = true;
  int nperbnd = args.get_int();
  assertx(nperbnd > 0);
  int noutput = 0;
  for (Edge e : mesh.edges()) {
    if (!mesh.is_boundary(e)) continue;
    const Point& p1 = mesh.point(mesh.vertex1(e));
    const Point& p2 = mesh.point(mesh.vertex2(e));
    for_int(i, nperbnd) output_point(interp(p1, p2, (i + .5f) / nperbnd), Vector(0.f, 0.f, 0.f));
    noutput += nperbnd;
  }
  showdf("Printed %d points on boundary edges\n", noutput);
}

void do_assign_normals() { assign_normals(); }

// *** norgroup

bool ng_orig_sharp(Edge e) {
  assertx(!mesh.is_boundary(e));
  if (mesh.flags(e).flag(GMesh::eflag_sharp)) return true;
  if (!same_string(mesh.get_string(mesh.ccw_corner(mesh.vertex1(e), e)),
                   mesh.get_string(mesh.clw_corner(mesh.vertex1(e), e))))
    return true;
  if (!same_string(mesh.get_string(mesh.ccw_corner(mesh.vertex2(e), e)),
                   mesh.get_string(mesh.clw_corner(mesh.vertex2(e), e))))
    return true;
  return false;
}

// First idea: just form smoothing groups
//  problem: lots of them (savanna:3650, schooner:8077)
// Second idea: use graph coloring to reduce number of groups
//  good results (savanna:6, schooner:5)
//  problem: model simplification may make different groups adjacent, and therefore crease is suddenly lost
// Third idea: number smoothing groups on separate connected components independently (without coloring)
//  good results (savanna:348, schooner:173)
void do_norgroup() {
  Map<Face, int> mapfg;
  {
    int totgroups = 0, max_norgroup = 0, ncomponents = 0;
    Set<Face> setfff;
    for (Face fff : mesh.faces()) {
      if (setfff.contains(fff)) continue;
      Set<Face> setff = gather_component_v(mesh, fff);
      for (Face ff : setff) setfff.enter(ff);
      int norgroup = 0;
      for (Face ff : setff) {
        if (mapfg.contains(ff)) continue;
        norgroup++;
        totgroups++;
        mapfg.enter(ff, norgroup);
        Queue<Face> queuef;
        queuef.enqueue(ff);
        while (!queuef.empty()) {
          Face f = queuef.dequeue();
          // mesh.update_string(f, "norgroup", csform(str, "%d", norgroup));
          for (Edge e : mesh.edges(f)) {
            if (mesh.is_boundary(e)) continue;
            if (!ng_orig_sharp(e)) {
              Face f2 = mesh.opp_face(f, e);
              bool is_new;
              mapfg.enter(f2, norgroup, is_new);
              if (is_new) queuef.enqueue(f2);
            }
          }
        }
      }
      max_norgroup = max(max_norgroup, norgroup);
      ncomponents++;
    }
    showdf("Found %d norgroups on %d components; named up to %d\n", totgroups, ncomponents, max_norgroup);
  }
  {
    int ncrease = 0, ncreasegone = 0, ncreasenew = 0;
    for (Edge e : mesh.edges()) {
      if (mesh.is_boundary(e)) continue;
      int norgroup1 = mapfg.get(mesh.face1(e));
      int norgroup2 = mapfg.get(mesh.face2(e));
      bool b_edge_sharp = norgroup1 != norgroup2;
      // bool b_edge_sharp = !same_string(mesh.get_string(mesh.face1(e)), mesh.get_string(mesh.face2(e)));
      if (!ng_orig_sharp(e)) {
        if (b_edge_sharp) ncreasenew++;
      } else {
        ncrease++;
        if (!b_edge_sharp) ncreasegone++;
      }
    }
    showdf("Originally %d creases; now %d gone, %d new ones.\n", ncrease, ncreasegone, ncreasenew);
  }
  string str;
  for (Face f : mesh.faces()) {
    int norgroup = mapfg.get(f);
    mesh.update_string(f, "norgroup", csform(str, "%d", norgroup));
  }
}

float face_inscribed_radius(Face f) {
  const Vec3<Point> triangle = mesh.triangle_points(f);
  return inscribed_radius(triangle[0], triangle[1], triangle[2]);
}

bool is_degenerate(Face f) { return face_inscribed_radius(f) < 1e-6f; }

// Should check if the swap is attribute-safe?  And then preserve normal and uv strings.
void do_swapdegendiag() {
  Set<Edge> set_bad_edges;
  for (Face f : mesh.faces())
    if (is_degenerate(f))
      for (Edge e : mesh.edges(f)) set_bad_edges.add(e);
  Array<Edge> bad_edges{set_bad_edges};
  showdf("swapdegendiag: found %d bad edges to swap\n", bad_edges.num());
  const auto by_decreasing_length = [&](Edge e1, Edge e2) { return mesh.length2(e1) > mesh.length2(e2); };
  sort(bad_edges, by_decreasing_length);
  for (Edge e : bad_edges)
    if (any_of(mesh.faces(e), &is_degenerate) && assertw(mesh.legal_edge_swap(e))) mesh.swap_edge(e);
}

void do_transf(Args& args) {
  Frame frame = FrameIO::parse_frame(args.get_string());
  if (0) {
    Frame frameinv;
    if (!invert(frame, frameinv)) showdf("Warning: uninvertible frame, normals lost\n");
  }
  showdf("Applying transform; normals (if any) may be wrong\n");
  for (Vertex v : mesh.vertices()) mesh.set_point(v, mesh.point(v) * frame);
}

void do_keepsphere(Args& args) {
  Point p;
  for_int(c, 3) p[c] = args.get_float();
  float radius = args.get_float();
  Set<Face> setfrem;
  for (Face f : mesh.faces()) {
    bool keep = true;
    for (Vertex v : mesh.vertices(f)) {
      if (dist2(mesh.point(v), p) > square(radius)) {
        keep = false;
        break;
      }
    }
    if (!keep) setfrem.enter(f);
  }
  for (Face f : setfrem) mesh.destroy_face(f);
  Set<Vertex> vdestroy;
  for (Vertex v : mesh.vertices())
    if (!mesh.degree(v)) vdestroy.enter(v);
  for (Vertex v : vdestroy) mesh.destroy_vertex(v);
}

void do_colorheight(Args& args) {
  float zfac = args.get_float();
  string str;
  for (Vertex v : mesh.vertices()) {
    float z = mesh.point(v)[2];
    float col = z * zfac;
    col = clamp(col, 0.f, 1.f);
    mesh.update_string(v, "rgb", csform_vec(str, thrice(col)));
  }
}

void do_colorsqheight(Args& args) {
  float zfac = args.get_float();
  string str;
  for (Vertex v : mesh.vertices()) {
    float z = mesh.point(v)[2];
    float col = square(z) * zfac;
    col = clamp(col, 0.f, 1.f);
    mesh.update_string(v, "rgb", csform_vec(str, thrice(col)));
  }
}

void do_colorbizarre() {
  Vec3<Vec2<float>> lfb;
  fill(lfb, V(BIGFLOAT, -BIGFLOAT));
  Vec3<LinearFunc> lf(LinearFunc(Vector(1.f, 1.f, 1.f), Point(0.f, 0.f, 0.f)),
                      LinearFunc(Vector(0.f, 1.f, 0.f), Point(0.f, 0.f, 0.f)),
                      LinearFunc(Vector(-1.f, -1.f, 0.f), Point(0.f, 0.f, 0.f)));
  for (Vertex v : mesh.vertices()) {
    for_int(i, 3) {
      float a = lf[i].eval(mesh.point(v));
      lfb[i][0] = min(lfb[i][0], a);
      lfb[i][1] = max(lfb[i][1], a);
    }
  }
  string str;
  for (Vertex v : mesh.vertices()) {
    A3dColor col;
    float sum = 0.f;
    for_int(i, 3) {
      float a = lf[i].eval(mesh.point(v));
      a = (a - lfb[i][0]) / (lfb[i][1] - lfb[i][0]);
      float c;
      switch (i) {
        case 0: c = (a < .25f ? 0.f + a * 4.f : a < .50f ? 0.f : 1.f - square(a - .75f) * 16.f); break;
        case 1: c = (a < .25f ? 0.f + a * 4.f : a < .50f ? 1.f : a < .75f ? 1.f - (a - .5f) * 4.f : 0.f); break;
        case 2: c = (a < .25f ? 1.f - a * 4.f : 1.f - (a - .25f) * 1.33333f); break;
        default: assertnever("");
      }
      if (0) assertx(c >= 0.f && c <= 1.f);
      sum += c;
    }
    sum = sum / 3.f;                                        // 0 .. 1
    sum = sum - .5f;                                        // -.5 .. .5
    sum = pow(abs(sum), .5f) * sign(sum) * .707106f + .5f;  // 0 .. 1
    assertx(sum >= 0.f && sum <= 1.f);
    fill(col, .1f + .8f * sum);
    mesh.update_string(v, "rgb", csform_vec(str, col));
  }
}

void do_colortransf(Args& args) {
  Frame frame = FrameIO::parse_frame(args.get_string());
  Point p;
  string str;
  for (Vertex v : mesh.vertices()) {
    for (Corner c : mesh.corners(v)) {
      if (parse_key_vec(mesh.get_string(c), "rgb", p)) {
        p = p * frame;
        for_int(i, 3) p[i] = clamp(p[i], 0.f, 1.f);
        mesh.update_string(c, "rgb", csform_vec(str, p));
      }
    }
    if (parse_key_vec(mesh.get_string(v), "rgb", p)) {
      p = p * frame;
      for_int(i, 3) p[i] = clamp(p[i], 0.f, 1.f);
      mesh.update_string(v, "rgb", csform_vec(str, p));
    }
  }
  for (Face f : mesh.faces()) {
    if (parse_key_vec(mesh.get_string(f), "rgb", p)) {
      p = p * frame;
      for_int(i, 3) p[i] = clamp(p[i], 0.f, 1.f);
      mesh.update_string(f, "rgb", csform_vec(str, p));
    }
  }
}

void do_normaltransf(Args& args) {
  Frame frame = FrameIO::parse_frame(args.get_string());
  assertw(is_zero(frame.p()));  // no translation
  frame.p() = Point(0.f, 0.f, 0.f);
  Vector n;
  string str;
  for (Vertex v : mesh.vertices()) {
    for (Corner c : mesh.corners(v)) {
      if (parse_key_vec(mesh.get_string(c), "normal", n)) {
        n = n * frame;
        if (0) assertw(is_unit(n));
        mesh.update_string(c, "normal", csform_vec(str, n));
      }
    }
    if (parse_key_vec(mesh.get_string(v), "normal", n)) {
      n = n * frame;
      if (0) assertw(is_unit(n));
      mesh.update_string(v, "normal", csform_vec(str, n));
    }
  }
}

void do_rmfarea(Args& args) {
  float area = args.get_float();
  Array<Face> arf;
  for (Face f : mesh.faces())
    if (mesh.area(f) < area) arf.push(f);
  showdf("Removing %d faces with area <%g\n", arf.num(), area);
  for (Face f : arf) mesh.destroy_face(f);
}

void do_rmflarea(Args& args) {
  float area = args.get_float();
  Array<Face> arf;
  for (Face f : mesh.faces())
    if (mesh.area(f) > area) arf.push(f);
  showdf("Removing %d faces with area >%g\n", arf.num(), area);
  for (Face f : arf) mesh.destroy_face(f);
}

void do_keepfmatid(Args& args) {
  int keep_id = args.get_int();
  Array<Face> arf;
  string str;
  for (Face f : mesh.faces()) {
    const char* s = GMesh::string_key(str, mesh.get_string(f), "matid");
    if (!s) Warning("Found face without any matid");
    if (!s || to_int(s) != keep_id) arf.push(f);
  }
  showdf("Keeping %d out of %d faces\n", mesh.num_faces() - arf.num(), mesh.num_faces());
  for (Face f : arf) mesh.destroy_face(f);
}

void do_uvtopos() {
  const bool keep_uv = getenv_bool("KEEPUV");
  for (Vertex v : mesh.vertices()) {
    Uv uv = get_uv(v);
    Point p(uv[0], uv[1], 0.f);
    mesh.set_point(v, p);
    if (!keep_uv) mesh.update_string(v, "uv", nullptr);
  }
}

void do_perturbz(Args& args) {
  float scale = args.get_float();
  for (Vertex v : mesh.vertices()) {
    Point p = mesh.point(v);
    p[2] += (Random::G.unif() - .5f) * 2.f * scale;
    mesh.set_point(v, p);
  }
}

void do_splitdiaguv() {
  Array<Edge> esplit;
  Array<Uv> uvs;
  for (Edge e : mesh.edges()) {
    if (mesh.is_boundary(e)) continue;
    uvs.init(0);
    for (Vertex v : mesh.vertices(e)) uvs.push(get_uv(v));
    if (abs(uvs[0][0] - uvs[1][0]) > 1e-4f && abs(uvs[0][1] - uvs[1][1]) > 1e-4f) esplit.push(e);
  }
  showdf("removing %d diagonals\n", esplit.num());
  for (Edge e : esplit) {
    Point p = interp(interp(mesh.point(mesh.vertex1(e)), mesh.point(mesh.vertex2(e))),
                     interp(mesh.point(mesh.side_vertex1(e)), mesh.point(mesh.side_vertex2(e))));
    Vertex v = mesh.split_edge(e);
    mesh.set_point(v, p);
  }
}

void do_rmdiaguv() {
  Array<Edge> esplit;
  Array<Uv> uvs;
  for (Edge e : mesh.edges()) {
    if (mesh.is_boundary(e)) continue;
    uvs.init(0);
    for (Vertex v : mesh.vertices(e)) {
      // arbitrarily pick one corner
      Corner c = mesh.ccw_corner(v, e);
      Uv uv;
      assertx(mesh.parse_corner_key_vec(c, "uv", uv));
      uvs.push(uv);
    }
    if (abs(uvs[0][0] - uvs[1][0]) > 1e-4f && abs(uvs[0][1] - uvs[1][1]) > 1e-4f) esplit.push(e);
  }
  showdf("removing %d diagonals\n", esplit.num());
  for (Edge e : esplit) {
    Map<Vertex, unique_ptr<char[]>> mvs;  // often nullptr's
    for (Face f : mesh.faces(e)) {
      for (Corner c : mesh.corners(f)) {
        Vertex v = mesh.corner_vertex(c);
        if (v != mesh.vertex1(e) && v != mesh.vertex2(e)) mvs.enter(v, mesh.extract_string(c));
      }
    }
    for (Vertex v : mesh.vertices(e)) {
      bool same = same_string(mesh.get_string(mesh.ccw_corner(v, e)), mesh.get_string(mesh.clw_corner(v, e)));
      mvs.enter(v, same ? mesh.extract_string(mesh.ccw_corner(v, e)) : nullptr);
    }
    Face f = mesh.coalesce_faces(e);
    for (Corner c : mesh.corners(f)) {
      Vertex v = mesh.corner_vertex(c);
      mesh.set_string(c, std::move(mvs.get(v)));
    }
  }
}

// Filtermesh ~/data/simplify/bunny.orig.m -projectimage "`cat ~/data/s3d/bunny.s3d`" ~/prevproj/2002/ssp/data/projectimage/bunny.proj.png | G3d - -st bunny.s3d -lighta 1 -lights 0
void do_projectimage(Args& args) {
  const string frame_string = args.get_string();
  const string image_name = args.get_filename();
  std::istringstream iss(frame_string);
  const ObjectFrame object_frame = FrameIO::read(iss).value();
  const Frame frameinv = ~object_frame.frame;
  const Image image(image_name);
  Matrix<Vector4> imagev(image.dims());
  convert(image, imagev);
  Vec2<FilterBnd> filterbs = twice(FilterBnd(Filter::get("spline"), Bndrule::reflected));
  if (filterbs[0].filter().has_inv_convolution()) filterbs = inverse_convolution(imagev, filterbs);
  string str;
  for (Vertex v : mesh.vertices()) {
    Point p = mesh.point(v);
    p *= frameinv;
    if (p[0] <= 0.f) {
      Warning("Mesh vertex behind view projection is not sampled");
      continue;
    }
    p[1] = p[1] / p[0] / object_frame.zoom * -0.5f + 0.5f;
    p[2] = -p[2] / p[0] / object_frame.zoom * +0.5f + 0.5f;  // negate because my image origin is at upper left
    if (1) {
      HH_SSTAT(Sx, p[1]);
      HH_SSTAT(Sy, p[2]);
    }
    Vector4 vec4 = sample_domain(imagev, V(p[2], p[1]), filterbs);
    vec4 = general_clamp(vec4, Vector4(0.f), Vector4(1.f));
    mesh.update_string(v, "rgb", csform_vec(str, V(vec4[0], vec4[1], vec4[2])));
  }
}

void do_quantizeverts(Args& args) {
  int nbits = args.get_int();
  assertx(nbits >= 1 && nbits <= 32);
  const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
  const Frame xform = bbox.get_frame_to_cube(), xform_inverse = ~xform;
  const float scale = pow(2.f, float(nbits));
  const float eps = 1e-6f;
  for (Vertex v : mesh.vertices()) {
    Point p = mesh.point(v);
    p *= xform;
    for_int(c, 3) {
      float f = p[c];
      assertx(f >= -eps && f <= 1.f + eps);
      int i = int(f * scale - 1e-6f);
      f = (i * 2 + 1) / (2.f * scale);
      assertx(f >= 0.f && f <= 1.f);
      p[c] = f;
    }
    p *= xform_inverse;
    mesh.set_point(v, p);
  }
}

void do_procedure(Args& args) {
  string name = args.get_string();
  if (0) {
    //

  } else if (name == "remove_hole_faces") {
    Array<Face> ar_faces;
    for (Face f : mesh.faces())
      if (GMesh::string_has_key(mesh.get_string(f), "hole")) ar_faces.push(f);
    showdf("Removing %d out of %d faces\n", ar_faces.num(), mesh.num_faces());
    for (Face f : ar_faces) mesh.destroy_face(f);

  } else if (name == "remove_filled_faces") {
    Array<Face> ar_faces;
    for (Face f : mesh.faces())
      if (GMesh::string_has_key(mesh.get_string(f), "filled")) ar_faces.push(f);
    showdf("Removing %d out of %d faces\n", ar_faces.num(), mesh.num_faces());
    for (Face f : ar_faces) mesh.destroy_face(f);

  } else if (name == "remove_yellow_vertices") {
    Set<Vertex> setv;
    Set<Face> setf;
    for (Vertex v : mesh.vertices()) {
      Vector col;
      if (!parse_key_vec(mesh.get_string(v), "rgb", col)) continue;
      if (col == Vector(1.f, 1.f, 0.f)) {
        setv.enter(v);
        for (Face f : mesh.faces(v)) setf.add(f);
      }
    }
    for (Face f : setf) mesh.destroy_face(f);
    for (Vertex v : setv) mesh.destroy_vertex(v);

  } else if (name == "mark_holes") {
    const int large_vertex_degree = args.get_int();
    for (Vertex v : mesh.vertices()) {
      if (mesh.degree(v) < large_vertex_degree) continue;
      showdf("Marking hole faces around a vertex with degree %d\n", mesh.degree(v));
      for (Face f : mesh.faces(v)) mesh.update_string(f, "hole", "");
    }

  } else if (name == "remove_stmatface_genus_5") {
    Set<Face> setfrem;
    for (Face f : mesh.faces()) {
      bool keep = true;
      for (Vertex v : mesh.vertices(f)) {
        const Point& p = mesh.point(v);
        if (p[0] > 185.f) {
          keep = false;
          break;
        }
      }
      if (!keep) setfrem.enter(f);
    }
    for (Face f : setfrem) mesh.destroy_face(f);
    Set<Vertex> vdestroy;
    for (Vertex v : mesh.vertices())
      if (!mesh.degree(v)) vdestroy.enter(v);
    for (Vertex v : vdestroy) mesh.destroy_vertex(v);

  } else if (name == "create_sphere") {
    assertx(mesh.empty());
    const int nlat = 31, nlon = 60;
    Matrix<Vertex> mv(nlat, nlon);
    string str;
    for_int(i, nlat) {
      for_int(j, nlon) {
        if ((i == 0 || i == nlat - 1) && j > 0) {
          mv[i][j] = mv[i][j - 1];
          continue;
        }
        Vertex v = mesh.create_vertex();
        mv[i][j] = v;
        float anglat = float(i) / (nlat - 1.f) * (TAU / 2);
        float anglon = float(j) / (nlon - 0.f) * TAU;
        Point p(std::cos(anglon) * std::sin(anglat), std::sin(anglon) * std::sin(anglat), std::cos(anglat));
        mesh.set_point(v, p);
        Uv uv(float(j) / (nlon - 1.f), float(i) / (nlat - 1.f));
        mesh.update_string(v, "uv", csform_vec(str, uv));
        if (i == 0) {
          assertx(dist2(mesh.point(v), Point(0.f, 0.f, 1.f)) < 1e-6f);
          mesh.set_point(v, Point(0.f, 0.f, 1.f));
        }
        if (i == nlat - 1) {
          assertx(dist2(mesh.point(v), Point(0.f, 0.f, -1.f)) < 1e-6f);
          mesh.set_point(v, Point(0.f, 0.f, -1.f));
        }
      }
    }
    const bool center_split = getenv_bool("CENTER_SPLIT");
    for_int(i, nlat - 1) {
      for_int(j, nlon) {
        int j1 = (j + 1) % nlon;
        if (0) {
        } else if (mv[i][j] == mv[i][j1]) {
          mesh.create_face(mv[i][j1], mv[i + 1][j], mv[i + 1][j1]);
        } else if (mv[i + 1][j] == mv[i + 1][j1]) {
          mesh.create_face(mv[i][j], mv[i + 1][j], mv[i][j1]);
        } else if (!center_split) {  // 2 triangles
          mesh.create_face(mv[i][j], mv[i + 1][j], mv[i][j1]);
          mesh.create_face(mv[i][j1], mv[i + 1][j], mv[i + 1][j1]);
        } else {  // 4 triangles
          Face f = mesh.create_face(V(mv[i][j], mv[i + 1][j], mv[i + 1][j1], mv[i][j1]));
          mesh.center_split_face(f);
        }
      }
    }
    //
    do_assign_normals();
    //
    const int cutlon = 20;
    for (int j = 0; j < nlon; j += cutlon) {
      for_int(i, (nlat - 1) / 2) {
        Edge e = mesh.edge(mv[i][j], mv[i + 1][j]);
        mesh.set_string(e, "sharp");
      }
    }

  } else if (name == "get_vup") {
    for (Face f : mesh.faces()) {
      Vector vec;
      assertx(parse_key_vec(mesh.get_string(f), "Vup", vec));
      HH_SSTAT(Slen, mag(vec));
    }

  } else if (name == "show_vup") {
    Polygon poly;
    A3dElem el;
    for (Face f : mesh.ordered_faces()) {
      Vector vec;
      if (!parse_key_vec(mesh.get_string(f), "Vup", vec)) continue;
      mesh.polygon(f, poly);
      Vector nor = poly.get_normal();
      if (1) vec = project_orthogonally(vec, nor);
      if (1) vec.normalize();
      Point pc = mean(poly);
      Stat stat;
      for (Vertex v : mesh.vertices(f)) stat.enter(dist(pc, mesh.point(v)));
      if (1) vec *= stat.avg() * 0.5f;
      pc += nor * stat.avg() * 1e-2f;
      el.init(A3dElem::EType::polyline);
      el.push(A3dVertex(pc, nor, A3dVertexColor(Pixel::black())));
      el.push(A3dVertex(pc + vec, nor, A3dVertexColor(Pixel::red())));
      oa3d.write(el);
    }
    for (Vertex v : mesh.ordered_vertices()) {
      Vector vec;
      if (!parse_key_vec(mesh.get_string(v), "Vup", vec)) continue;
      Vnors vnors(mesh, v);
      assertx(vnors.is_unique());
      const Vector& nor = vnors.unique_nor();
      if (1) vec = project_orthogonally(vec, nor);
      if (1) vec.normalize();
      Point pc = mesh.point(v);
      Stat stat;
      for (Vertex vv : mesh.vertices(v)) stat.enter(dist(pc, mesh.point(vv)));
      if (1) vec *= stat.avg() * 0.5f;
      pc += nor * stat.avg() * 1e-2f;
      el.init(A3dElem::EType::polyline);
      el.push(A3dVertex(pc, nor, A3dVertexColor(Pixel::black())));
      el.push(A3dVertex(pc + vec, nor, A3dVertexColor(Pixel::red())));
      oa3d.write(el);
    }

  } else if (name == "print_bnd_verts") {
    for (Vertex v : mesh.vertices()) {
      if (mesh.is_boundary(v)) {
        SHOW(mesh.vertex_id(v));
        SHOW(mesh.point(v));
      }
    }

  } else {
    args.problem("procedure '" + name + "' unrecognized");
  }
}

// signed distance

float signed_distance(const Point& p, Face f) {
  const Vec3<Point> triangle = mesh.triangle_points(f);
  const auto [d2, bary, clp] = project_point_triangle(p, triangle);
  int nzero = 0, jzero, jpos;
  dummy_init(jpos, jzero);
  for_int(j, 3) {
    if (bary[j] == 0.f) {
      nzero++;
      jzero = j;
    } else {
      jpos = j;
    }
  }
  switch (nzero) {
    case 0: {  // triangle interior
      // == dot(get_normal(triangle), p - clp)
      return sqrt(d2) * sign(dot(get_normal_dir(triangle), p - clp));
    }
    case 1: {  // edge
      Vec3<Vertex> va = mesh.triangle_vertices(f);
      Edge e = mesh.edge(va[mod3(jzero + 1)], va[mod3(jzero + 2)]);
      if (mesh.is_boundary(e)) return k_Contour_undefined;
      const Vec3<Point> triangle2 = mesh.triangle_points(mesh.opp_face(f, e));
      const Vector nor = get_normal(triangle) + get_normal(triangle2);
      return sqrt(d2) * sign(dot(nor, p - clp));
    }
    case 2: {  // vertex
      Vec3<Vertex> va = mesh.triangle_vertices(f);
      Vertex v = va[jpos];
      if (mesh.is_boundary(v)) return k_Contour_undefined;
      Vector nor{};
      for (Face ff : mesh.faces(v)) nor += get_normal(mesh.triangle_points(ff));
      return sqrt(d2) * sign(dot(nor, p - clp));
    }
    default: assertnever("");
  }
}

void do_signeddistcontour(Args& args) {
  // e.g.:
  // Filtermesh ~/data/mesh/icosahedron.m -signeddistcontour 50 | G3d - -key DmDe
  // Filtermesh -createobject torus1 -transf "`Filterframe -create_euler 18 23 37`" -triang -signeddistcontour 40 | G3d - -key DmDe
  int grid = args.get_int();
  assertx(grid >= 2);
  {
    const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
    const Frame xform = bbox.get_frame_to_small_cube();
    showdf("Applying xform: %s", FrameIO::create_string(ObjectFrame{xform, 1}).c_str());
    for (Vertex v : mesh.vertices()) mesh.set_point(v, mesh.point(v) * xform);
  }
  const MeshSearch mesh_search(mesh, {});
  const auto func_mesh_signed_distance = [&](const Vec3<float>& p) {
    Face f = mesh_search.search(p, nullptr).f;
    return signed_distance(p, f);
  };
  GMesh nmesh;
  {
    Contour3DMesh contour(grid, &nmesh, func_mesh_signed_distance);
    contour.set_ostream(&std::cout);
    for (Vertex v : mesh.vertices()) contour.march_from(mesh.point(v));
  }
  mesh.copy(nmesh);
}

// *** hull

Point compute_hull_point(Vertex v, float offset) {
  assertx(!mesh.is_boundary(v));
  // Set up a linear programming problem.
  // Recall that all variables are expected to be >= 0, annoying!
  //  workaround: translate all variables into x, y, z > 0 octant
  // simplx: epsilon is not scale-invariant -> scale data as well.
  // p'= p * scale + translate .   p' in range [border, border + size]
  float scale;
  Vector translate;
  const float transf_border = 10.f;  // allow vertex to go anywhere
  // const float transf_border = .25f;  // severely constrain the vertex
  // const float transf_border = 1.f;
  const float transf_size = 1.f;
  {
    Bbox bbox{transform(concatenate(V(v), mesh.vertices(v)), [&](Vertex vv) { return mesh.point(vv); })};
    bbox[0] -= Vector(abs(offset), abs(offset), abs(offset));
    bbox[1] += Vector(abs(offset), abs(offset), abs(offset));
    scale = transf_size / assertx(bbox.max_side());
    translate = (Vector(transf_border, transf_border, transf_border) + (Point(0.f, 0.f, 0.f) - bbox[0]) * scale);
  }
  // Gather constraints.
  Array<LinearFunc> ar_lf;
  {
    for (Face f : mesh.faces(v)) {
      Vec3<Point> triangle = mesh.triangle_points(f);
      Vector normal = get_normal_dir(triangle);
      // Normalize normal for numerical precision in simplx.
      if (!normal.normalize()) {
        Warning("Degenerate face ignored");
        continue;
      }
      Point pold = triangle[0];
      pold += normal * offset;
      if (offset < 0.f) normal = -normal;
      Point pnew = pold * scale + translate;
      for_int(c, 3) assertx(pnew[c] > transf_border - 1e-4f && pnew[c] < transf_border + transf_size + 1e-4f);
      ar_lf.push(LinearFunc(normal, pnew));
    }
  }
  // Gather objective function.
  Vector link_normal;
  {
    Polygon poly;
    for (Vertex vv : mesh.ccw_vertices(v)) poly.push(mesh.point(vv));
    link_normal = poly.get_normal_dir();
    // Normalize for numerical precision in simplx.
    assertx(link_normal.normalize());
    if (offset < 0.f) link_normal = -link_normal;
  }
#if defined(HH_HAVE_SIMPLEX)
  // Count number of constraints which are of form lfunc(x, y, z) >= d >= 0
  int n_m1 = int(count_if(ar_lf, [](const LinearFunc& lf) { return lf.offset > 0.f; }));
  // Use Numerical Recipes code (converted to double precision)
  int n = 3;                                       // number of variables (x, y, z)
  int m1 = n_m1, m2 = ar_lf.num() - n_m1, m3 = 0;  // '<=', '>=', '==' constraints
  int m = m1 + m2 + m3;                            // total number of constraints
  double** a = dmatrix(1, m + 2, 1, n + 1);
  int* iposv = ivector(1, m);
  int* izrov = ivector(1, n);
  int icase;
  // Maximize the linear functional: -x*link_normal[0]-y*link_normal[1]...
  a[1][1] = 0.f;  // always
  a[1][2] = -link_normal[0];
  a[1][3] = -link_normal[1];
  a[1][4] = -link_normal[2];
  // Subject to the constraint: lf.eval(Point(x, y, z)) >= 0
  //  equivalently, lf.v[0] * x + lf.v[1] * y + lf.v[2] * z >= -lf.offset
  //  or possibly, -lf.v[0] * x - lf.v[1] * y - lf.v[2] * z <= lf.offset
  {
    int nr = 0;
    for_int(i, ar_lf.num()) {
      const LinearFunc& lf = ar_lf[i];
      if (!(lf.offset > 0.f)) continue;
      a[2 + nr][1] = lf.offset;
      a[2 + nr][2] = lf.v[0];
      a[2 + nr][3] = lf.v[1];
      a[2 + nr][4] = lf.v[2];
      nr++;
    }
    assertx(nr == m1);
    for_int(i, ar_lf.num()) {
      const LinearFunc& lf = ar_lf[i];
      if (lf.offset > 0.f) continue;
      a[2 + nr][1] = -lf.offset;
      a[2 + nr][2] = -lf.v[0];
      a[2 + nr][3] = -lf.v[1];
      a[2 + nr][4] = -lf.v[2];
      nr++;
    }
    assertx(nr == m);
  }
  {
    // We could instead use the Raymond Seidel (or Raimund Seidel) fast linear programming scheme, copied from
    // Seth Teller into rseidel_lp.tar.gz, then into linprog.tar.gz .
    simplx(a, m, n, m1, m2, m3, &icase, izrov, iposv);
  }
  Point newpoint;
  dummy_init(newpoint);
  bool ret = false;
  if (icase == -2) {
    Warning("simplx: hhiter exceeded");
  } else if (icase == -1) {
    Warning("simplx: no solution satisfies constraints");
  } else if (icase < 0) {
    assertnever("");
  } else if (icase > 0) {
    Warning("simplx: objective function is unbounded");
  } else {
    // A finite solution is found.
    Point pnew(0.f, 0.f, 0.f);  // must be initialized to zero's.
    for_intL(i, 1, m + 1) {
      assertx(iposv[i] >= 1);
      if (iposv[i] <= n) pnew[iposv[i] - 1] = float(a[1 + i][1]);
    }
    if (1) {
      for_int(i, ar_lf.num()) {
        if (ar_lf[i].eval(pnew) < -1e-4f) {
          Warning("Simplx linear inequality not satisfied");
          HH_SSTAT(Ssimplxbad, ar_lf[i].eval(pnew));
        }
      }
    }
    bool good = true;
    for_int(c, 3) {
      assertw(pnew[c] >= -1e-4f);
      if (pnew[c] < .01f) {
        Warning("hull point outside border_min");
        good = false;
        break;
      }
      if (pnew[c] > transf_border * 2 + transf_size) {
        Warning("hull point outside border_max");
        good = false;
        break;
      }
    }
    if (good) {
      newpoint = (pnew - translate) / scale;
      ret = true;
    }
  }
  free_dmatrix(a, 1, m + 2, 1, n + 1);
  free_ivector(iposv, 1, m);
  free_ivector(izrov, 1, n);
  assertx(ret);
  return newpoint;
#else   // defined(HH_HAVE_SIMPLEX)
  assertnever_ret("Linear programming (simplex) is unavailable");
  return Point(thrice(BIGFLOAT));
#endif  // defined(HH_HAVE_SIMPLEX)
}

// E.g.: (f=~/data/mesh/bunny.nf400.m; Filtermesh $f -hull 5e-3 | G3dOGL $f -input -key DmDeNN)
void do_hull(Args& args) {
  float offset = args.get_float();
  Map<Vertex, Point> mapvp;
  for (Vertex v : mesh.vertices()) {
    Point p = compute_hull_point(v, offset);
    mapvp.enter(v, p);
  }
  for (Vertex v : mesh.vertices()) mesh.set_point(v, mapvp.get(v));
}

// Output frame to rigidly transform from one mesh to another presumably identical mesh.
// The reason that we don't simply align the current mesh is to encourage proper usage:
// Two *original* meshes should be aligned.  Then, the resulting alignment frame should be used to align the
//  corresponding *processed* meshes.
// Aligning the processed meshes to each other is likely invalid since they no longer share the same geometry.
// Proper use:
//  set frame="`Filtermesh oldmesh.orig.m -alignment newmesh.orig.m`"
//  Filtermesh oldmesh.result.m -transf "`cat $frame`" >newmesh.resultcmp.m
void do_alignmentframe(Args& args) {
  string filename = args.get_filename();
  const GMesh& cmesh = mesh;
  GMesh nmesh;
  nmesh.read(RFile(filename)());
  showdf("Computing alignment of current mesh:\n");
  showdf("  %s\n", mesh_genus_string(cmesh).c_str());
  showdf("with mesh %s:\n", filename.c_str());
  showdf("  %s\n", mesh_genus_string(nmesh).c_str());
  assertx(!cmesh.empty() && !nmesh.empty());
  assertw(cmesh.num_vertices() == nmesh.num_vertices());  // just warn
  assertw(cmesh.num_faces() == nmesh.num_faces());        // just warn
  const Bbox cbb{transform(cmesh.vertices(), [&](Vertex v) { return cmesh.point(v); })};
  const Bbox nbb{transform(nmesh.vertices(), [&](Vertex v) { return nmesh.point(v); })};
  // Point corig = cbb[0];
  // Point norig = nbb[0];
  Point corig = interp(cbb[0], cbb[1]);  // align centroids
  Point norig = interp(nbb[0], nbb[1]);
  float scale;
  {
    HH_STAT(Sscale);
    for_int(c, 3) Sscale.enter((nbb[1][c] - nbb[0][c]) / assertx(cbb[1][c] - cbb[0][c]));
    // float scale = Sscale.max();
    scale = Sscale.avg();
    showdf("Sscale.max() / Sscale.min() = %g\n", Sscale.max() / Sscale.min());
    assertw(Sscale.max() / Sscale.min() < 1.0001f);
  }
  Frame frame = (Frame::translation(Point(0.f, 0.f, 0.f) - corig) * Frame::scaling(thrice(scale)) *
                 Frame::translation(norig - Point(0.f, 0.f, 0.f)));
  std::cout << FrameIO::create_string(ObjectFrame{frame, 1});
  // for (Vertex v : mesh.vertices()) mesh.set_point(v, mesh.point(v) * frame);
  nooutput = true;
}

inline Point avg_cubic(const Point& p0, const Point& p1, const Point& p2) { return p0 * .25f + p1 * .50f + p2 * .25f; }

auto smoothgim_subdiv(CMatrixView<Point> opoints) {
  int nn = opoints.ysize() - 1;
  Matrix<Point> npoints(nn * 2 + 1, nn * 2 + 1);
  // copy old points
  for_int(y, nn + 1) for_int(x, nn + 1) npoints[y * 2][x * 2] = opoints[y][x];
  // splitting step
  for_int(y, nn + 1) {
    for_int(x, nn) npoints[y * 2][x * 2 + 1] = interp(npoints[y * 2][x * 2 + 0], npoints[y * 2][x * 2 + 2]);
  }
  for_int(y, nn) {
    for_int(x, nn * 2 + 1) npoints[y * 2 + 1][x] = interp(npoints[y * 2 + 0][x], npoints[y * 2 + 2][x]);
  }
  // averaging step
  nn = nn * 2;
  Matrix<Point> tpoints(nn + 1, nn + 1);
  for_int(y, nn + 1) for_int(x, nn + 1) tpoints[y][x] = npoints[y][x];
  for_int(y, nn + 1) {
    for_int(x, nn + 1) {
      npoints[y][x] = avg_cubic(x > 0 ? tpoints[y][x - 1] : tpoints[nn - y][x + 1], tpoints[y][x],
                                x < nn ? tpoints[y][x + 1] : tpoints[nn - y][x - 1]);
    }
  }
  for_int(y, nn + 1) for_int(x, nn + 1) tpoints[y][x] = npoints[y][x];
  for_int(y, nn + 1) {
    for_int(x, nn + 1) {
      npoints[y][x] = avg_cubic(y > 0 ? tpoints[y - 1][x] : tpoints[y + 1][nn - x], tpoints[y][x],
                                y < nn ? tpoints[y + 1][x] : tpoints[y - 1][nn - x]);
    }
  }
  return npoints;
}

void do_smoothgim(Args& args) {
  int nsubdiv = args.get_int();
  // expect opened gim
  int nn = int(sqrt(mesh.num_vertices() + .01f)) - 1;
  assertx(square(nn + 1) == mesh.num_vertices());
  Matrix<Point> points(nn + 1, nn + 1);
  // extract mesh points
  for_int(y, nn + 1) for_int(x, nn + 1) points[y][x] = mesh.point(mesh.id_vertex(y * (nn + 1) + x + 1));
  // respect C2 rule at side (cut) vertices
  if (1) {
    if (nn >= 4) {
      for_int(i, 2) {
        int d = i == 0 ? 1 : -1;
        int y, x;
        x = i * nn;
        y = nn / 2;
        assertx(!compare(points[y - 1][x], points[y + 1][x], 1e-6f));
        points[y][x] =
            interp(interp(points[y - 1][x + d], points[y + 1][x + d]), interp(points[y][x + d], points[y - 1][x]));
        y = i * nn;
        x = nn / 2;
        assertx(!compare(points[y][x - 1], points[y][x + 1], 1e-6f));
        points[y][x] =
            interp(interp(points[y + d][x - 1], points[y + d][x + 1]), interp(points[y + d][x], points[y][x - 1]));
      }
    } else {
      Warning("Gim too small to make C2");
    }
  }
  // subdivide
  for_int(i, nsubdiv) {
    Matrix<Point> tpoints = smoothgim_subdiv(points);
    nn = tpoints.ysize() - 1;
    points.init(0, 0);
    points.init(nn + 1, nn + 1);
    for_int(y, nn + 1) for_int(x, nn + 1) points[y][x] = tpoints[y][x];
  }
  // recreate mesh
  mesh.clear();
  Matrix<Vertex> vertices(nn + 1, nn + 1);
  string str;
  for_int(y, nn + 1) {
    for_int(x, nn + 1) {
      Vertex v = mesh.create_vertex();
      vertices[y][x] = v;
      mesh.set_point(v, points[y][x]);
      Vector nor;
      if (0) {
      } else if (y == nn / 2 && (x == 0 || x == nn)) {
        // Are border points really just simply reflections?
        assertx(!compare(points[y - 1][x], points[y + 1][x], 1e-6f));
        int xr = x == 0 ? x + 1 : x - 1;
        Vector tb = points[y][xr] - points[y - 1][x];
        Vector tc = points[y - 1][xr] - points[y + 1][xr];
        if (x == 0) tc = -tc;
        nor = cross(tb, tc);
      } else if (x == nn / 2 && (y == 0 || y == nn)) {
        assertx(!compare(points[y][x - 1], points[y][x + 1], 1e-6f));
        int yr = y == 0 ? y + 1 : y - 1;
        Vector tb = points[yr][x] - points[y][x - 1];
        Vector tc = points[yr][x - 1] - points[yr][x + 1];
        if (y == nn) tc = -tc;
        nor = cross(tb, tc);
      } else {
        Vector tx =
            ((x > 0 ? points[y][x - 1] : points[nn - y][x + 1]) - (x < nn ? points[y][x + 1] : points[nn - y][x - 1]));
        Vector ty =
            ((y > 0 ? points[y - 1][x] : points[y + 1][nn - x]) - (y < nn ? points[y + 1][x] : points[y - 1][nn - x]));
        nor = cross(tx, ty);
      }
      assertw(nor.normalize());
      mesh.update_string(v, "normal", csform_vec(str, nor));
      if (y * x > 0)
        mesh.create_face(
            V(vertices[y - 1][x - 1], vertices[y - 1][x - 0], vertices[y - 0][x - 0], vertices[y - 0][x - 1]));
    }
  }
}

void do_subsamplegim(Args& args) {
  int nsubsamp = args.get_int();
  // expect opened gim
  int nn = int(sqrt(mesh.num_vertices() + .01f)) - 1;
  assertx(square(nn + 1) == mesh.num_vertices());
  assertx((nn / nsubsamp) * nsubsamp == nn);
  nn /= nsubsamp;
  Matrix<Vertex> vertices(nn + 1, nn + 1);
  const GMesh& omesh = mesh;
  GMesh nmesh;
  for_int(y, nn + 1) {
    for_int(x, nn + 1) {
      Vertex vn = nmesh.create_vertex();
      vertices[y][x] = vn;
      Vertex vo = omesh.id_vertex(y * nsubsamp * (nn * nsubsamp + 1) + x * nsubsamp + 1);
      nmesh.set_point(vn, omesh.point(vo));
      nmesh.set_string(vn, omesh.get_string(vo));
      if (y * x > 0)
        nmesh.create_face(
            V(vertices[y - 1][x - 1], vertices[y - 1][x - 0], vertices[y - 0][x - 0], vertices[y - 0][x - 1]));
    }
  }
  mesh.copy(nmesh);
}

// (cd ~/prevproj/2009/catwalk/data; Filtermesh manikin_ballerina_filter.ohull.nf10000.m -shootrays manikin_ballerina_filter.wids.m | G3d manikin_ballerina_filter.ohull.nf10000.m v.a3d - -key DmDe -st manikin_ballerina_filter.s3d; rm v.a3d)  # Press 'P'.
void do_shootrays(Args& args) {
  string filename = args.get_filename();
  GMesh omesh;  // Original mesh.
  omesh.read(RFile(filename)());
  showdf("Shooting ray to: %s\n", mesh_genus_string(omesh).c_str());
  assertx(!mesh.empty() && !omesh.empty());
  const Bbox bbox{transform(concatenate(mesh.vertices(), omesh.vertices()), [&](Vertex v) { return mesh.point(v); })};
  const Frame xform = bbox.get_frame_to_small_cube(0.5f);
  const Frame xform_inverse = ~xform;
  Array<TriangleFace> trianglefaces;
  trianglefaces.reserve(omesh.num_faces());
  {
    for (Face f : omesh.faces()) {
      Vec3<Point> triangle = omesh.triangle_points(f);
      if (1) triangle = widen_triangle(triangle, 1e-4f);
      for_int(i, 3) triangle[i] *= xform;
      trianglefaces.push({triangle, f});
    }
  }
  // const int gridn = 120;
  const int gridn = clamp(int(sqrt(mesh.num_faces() * .05f)), 15, 200);
  const TriangleFaceSpatial spatial(trianglefaces, gridn);  // Not MeshSearch because of widen_triangle().

  HH_TIMER("__shoot_rays");
  const bool show_a3d = true;
  const auto up_fi = show_a3d ? make_unique<WFile>("v.a3d") : nullptr;
  const auto up_oa3d = up_fi ? make_unique<WSA3dStream>((*up_fi)()) : nullptr;
  const float negdisp = -1e-5f * bbox.max_side() * xform[0][0];
  const float maxdisp = raymaxdispfrac * bbox.max_side() * xform[0][0];
  string str;
  for (Vertex v : mesh.vertices()) {
    const Point p = mesh.point(v) * xform;
    Vector nor(0.f, 0.f, 0.f);
    if (parse_key_vec(mesh.get_string(v), "normal", nor)) {
    } else {
      Warning("No vertex normal; looking for a corner normal");
      for (Corner c : mesh.corners(v))
        if (parse_key_vec(mesh.get_string(c), "normal", nor)) break;
      assertx(!is_zero(nor));
    }
    mesh.update_string(v, "Onormal", csform_vec(str, nor));
    mesh.update_string(v, "normal", nullptr);
    assertx(is_unit(nor));
    float mindist = BIGFLOAT;
    Point minp = p;
    for_int(dir, 2) {
      const float vdir = dir ? 1.f : -1.f;
      const Point p1 = p + nor * (negdisp * vdir);
      const Point p2 = p + nor * (maxdisp * vdir);
      if (const auto result = spatial.first_along_segment(p1, p2)) {
        if (const Point pint = result->pint; dist(p, pint) < abs(mindist)) {
          mindist = dist(p, pint) * vdir;
          minp = pint;
        }
      }
    }
    if (mindist == BIGFLOAT) {
      Warning("No ray intersection");
      mindist = 0.f;
      // mindist = .02f;  // for debugging
    } else {
      HH_SSTAT(Smindistc, mindist);
      HH_SSTAT(Smindisto, mindist / xform[0][0]);
    }
    // Point minpint = p + nor * mindist;
    if (up_oa3d) {
      A3dElem el(A3dElem::EType::polyline, 0);
      el.push(A3dVertex(p * xform_inverse, nor, A3dVertexColor(Pixel::black())));
      el.push(A3dVertex(minp * xform_inverse, nor, A3dVertexColor(Pixel::black())));
      up_oa3d->write(el);
    }
    const Point op = mesh.point(v);
    mesh.update_string(v, "Opos", csform_vec(str, op));
    mesh.set_point(v, minp * xform_inverse);
    mesh.update_string(v, "sdisp", csform_vec(str, V(mindist / xform[0][0])));
  }
}

void do_transferkeysfrom(Args& args) {
  string filename = args.get_filename();
  GMesh omesh;
  omesh.read(RFile(filename)());
  showdf("Transferring strings from: %s\n", mesh_genus_string(omesh).c_str());
  assertx(!mesh.empty() && !omesh.empty());
  HashPoint hp;  // (4, 0.f, 1.f);
  Array<Vertex> arv;
  const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
  const Frame xform = bbox.get_frame_to_small_cube();
  if (1) {
    for (Vertex v : mesh.vertices()) hp.pre_consider(mesh.point(v) * xform);
    for (Vertex ov : omesh.vertices()) hp.pre_consider(omesh.point(ov) * xform);
  }
  for (Vertex v : mesh.ordered_vertices()) {
    Point p = mesh.point(v) * xform;
    int i = hp.enter(p);
    if (i < arv.num()) SHOW(p, mesh.point(arv[i]) * xform, i, arv.num());
    assertx(i == arv.num());  // no two vertices should have same position
    arv.push(v);
  }
  Array<char> key, val;
  Map<Vertex, Vertex> movv;
  for (Vertex ov : omesh.vertices()) {
    const Point& p = omesh.point(ov);
    int i = hp.enter(p * xform);
    if (i == arv.num()) SHOW(omesh.point(ov), omesh.vertex_id(ov));
    assertx(i < arv.num());  // all vertex position must already be present
    Vertex v = arv[i];
    movv.enter(ov, v);
    for_cstring_key_value(omesh.get_string(ov), key, val, [&] { mesh.update_string(v, key.data(), val.data()); });
  }
  Map<Face, Face> moff;
  Array<Vertex> va;
  for (Face of : omesh.faces()) {
    omesh.get_vertices(of, va);
    Face f = mesh.face(movv.get(va[0]), movv.get(va[1]));
    assertx(f);
    moff.enter(of, f);
    for_cstring_key_value(omesh.get_string(of), key, val, [&] { mesh.update_string(f, key.data(), val.data()); });
    for (Corner oc : omesh.corners(of)) {
      if (!mesh.get_string(oc)) continue;
      Vertex ov = mesh.corner_vertex(oc);
      Corner c = mesh.corner(movv.get(ov), f);
      for_cstring_key_value(omesh.get_string(oc), key, val, [&] { mesh.update_string(c, key.data(), val.data()); });
    }
  }
  for (Edge oe : omesh.edges()) {
    if (!mesh.get_string(oe)) continue;
    Edge e = mesh.edge(movv.get(mesh.vertex1(oe)), movv.get(mesh.vertex2(oe)));
    for_cstring_key_value(omesh.get_string(oe), key, val, [&] { mesh.update_string(e, key.data(), val.data()); });
  }
}

void do_transferwidkeysfrom(Args& args) {
  string filename = args.get_filename();
  GMesh omesh;  // An original mesh containing a superset of vertices.
  omesh.read(RFile(filename)());
  showdf("Transferring strings from: %s\n", mesh_genus_string(omesh).c_str());
  assertx(!mesh.empty() && !omesh.empty());
  assertx(omesh.num_vertices() >= mesh.num_vertices());
  Map<int, const char*> mwidstring;
  string str;
  for (Vertex ov : omesh.vertices()) {
    int wid = to_int(assertx(GMesh::string_key(str, assertx(mesh.get_string(ov)), "wid")));
    assertx(wid);
    mwidstring.enter(wid, mesh.get_string(ov));
  }
  Array<char> key, val;
  for (Vertex v : mesh.vertices()) {
    int wid = to_int(assertx(GMesh::string_key(str, assertx(mesh.get_string(v)), "wid")));
    assertx(wid);
    for_cstring_key_value(mwidstring.get(wid), key, val, [&] { mesh.update_string(v, key.data(), val.data()); });
  }
}

// *** fromObj

// Assert that group is a convex connected component.
// Maybe flip all faces so that most faces in the group point away from the group centroid.
void convex_group_flip_faces(const Set<Face>& group) {
  // compute centroid
  Homogeneous h;
  Set<Vertex> setv;
  for (Face f : group)
    for (Vertex v : mesh.vertices(f))
      if (setv.add(v)) h += mesh.point(v);
  Point ctr = to_Point(normalized(h));
  int vote_flip = 0, vote_keep = 0;
  for (Face f : group) {
    Vector toctr{};
    for (Vertex v : mesh.vertices(f)) toctr += ctr - mesh.point(v);
    Polygon poly;
    mesh.polygon(f, poly);
    if (dot(toctr, poly.get_normal()) > 0.f)
      vote_flip++;
    else
      vote_keep++;
  }
  if (vote_flip > vote_keep) {
    Array<Vertex> va;
    for (Face f : group) {
      int fid = mesh.face_id(f);
      mesh.get_vertices(f, va);
      reverse(va);
      mesh.destroy_face(f);
      mesh.create_face_private(fid, va);
    }
    string str;
    for (Vertex v : setv) {
      Vector n;
      if (!parse_key_vec(mesh.get_string(v), "normal", n)) continue;
      n = -n;
      mesh.update_string(v, "normal", csform_vec(str, n));
    }
  }
}

// will clear the old mesh
void do_fromObj(Args& args) {
  // build mesh from Obj input. Specify <flip> to flip face to point to
  //  the outside of convex components
  Array<Vector> ar_nor;
  Array<Uv> ar_uv;
  RFile fi(args.get_filename());
  bool flip = false;
  if (args.num() && args.peek_string() == "flip") {
    args.get_string();
    flip = true;
  }
  int v = 0, gid = 1;
  Set<Face> group;
  string str;
  for (string line; my_getline(fi(), line);) {
    const char* sline = line.c_str();
    if (const char* s = after_prefix(sline, "v ")) {
      Point p;
      for_int(c, 3) p[c] = float_from_chars(s);
      assert_no_more_chars(s);
      mesh.create_vertex();
      v++;
      Vertex vv = mesh.id_vertex(v);
      mesh.set_point(vv, p);
      mesh.update_string(vv, "group", csform(str, "%d", gid));
      continue;
    }
    if (const char* s = after_prefix(sline, "vt ")) {
      Uv uv;
      for_int(c, 2) uv[c] = float_from_chars(s);
      assert_no_more_chars(s);
      ar_uv.push(uv);
      continue;
    }
    if (const char* s = after_prefix(sline, "vn ")) {
      Vector nor;
      for_int(c, 3) nor[c] = float_from_chars(s);
      assert_no_more_chars(s);
      ar_nor.push(nor);
      continue;
    }
    if (const char* s = after_prefix(sline, "f ")) {
      Vector fn{};
      Array<Vertex> va;
      Polygon poly;
      bool have_nors = true;
      while (*s) {
        const int i = int_from_chars(s);  // It is 1-based.
        assertx(*s++ == '/');
        const int j = int_from_chars(s) - 1;  // Convert from 1-based to 0-based.
        assertx(*s++ == '/');
        const int k = int_from_chars(s) - 1;
        dummy_use(j);
        if (k < ar_nor.num())
          fn += ar_nor[k];
        else
          have_nors = false;
        Vertex vv = mesh.id_vertex(i);
        va.push(vv);
        poly.push(mesh.point(vv));
        while (std::isspace(*s)) s++;
      }
      // maybe flip face winding to match the vertex normals orientation
      if (have_nors && dot(fn, poly.get_normal()) < 0.f) reverse(va);
      if (mesh.legal_create_face(va)) {
        Face f = mesh.create_face(va);
        group.add(f);
        s = sline + 2;  // Rewind and parse again, to update corner attributes, if supplied.
        while (*s) {
          const int i = int_from_chars(s);
          assertx(*s++ == '/');
          const int j = int_from_chars(s) - 1;  // Convert from 1-based to 0-based.
          assertx(*s++ == '/');
          const int k = int_from_chars(s) - 1;
          Corner c = mesh.corner(mesh.id_vertex(i), f);
          if (k < ar_nor.num()) mesh.update_string(c, "normal", csform_vec(str, ar_nor[k]));
          if (j < ar_uv.num()) mesh.update_string(c, "uv", csform_vec(str, ar_uv[j]));
          while (std::isspace(*s)) s++;
        }
      } else {
        Warning("Illegal face");
      }
      continue;
    }
    if (after_prefix(sline, "s ")) {
      gid++;
      if (flip) convex_group_flip_faces(group);
      group.clear();
      continue;
    }
  }

  if (flip) convex_group_flip_faces(group);
}

void do_to_obj() {
  // ??
}

void do_sphparam_to_tangentfield(Args& args) {
  // Filtermesh ~/proj/sphere/sphparams/cow.sphparam.m -sphparam_to_tang 0 0 1 -renamekey f dir Vup -procedure show_vup | G3d - -st cow -lighta 1 -lights 0
  Vector gdir;
  for_int(c, 3) gdir[c] = args.get_float();
  string str;
  for (Face f : mesh.faces()) {
    const Vec3<Point> triangle = mesh.triangle_points(f);
    const Vec3<Point> sphs = map(mesh.triangle_vertices(f), [&](Vertex v) {
      Point sph;
      assertx(parse_key_vec(mesh.get_string(v), "sph", sph));
      return sph;
    });
    const Vector splnor = get_normal(sphs);
    const Vector ssnor = normalized(interp(sphs));
    const Vector sdir = normalized(project_orthogonally(project_orthogonally(gdir, ssnor), splnor));
    const Bary bary = bary_of_vector(sphs, sdir);
    const Vector dir = normalized(vector_from_bary(triangle, bary));
    mesh.update_string(f, "dir", csform_vec(str, dir));
  }
}

void do_trim(Args& args) {
  const float dtrim = args.get_float();
  const float dtrim2 = square(dtrim);
  Array<Face> faces_to_destroy;
  for (Face f : mesh.faces()) {
    const Vec3<Point> triangle = mesh.triangle_points(f);
    if (dist2(triangle[0], triangle[1]) >= dtrim2 || dist2(triangle[1], triangle[2]) >= dtrim2 ||
        dist2(triangle[2], triangle[0]) >= dtrim2)
      faces_to_destroy.push(f);
  }
  showdf("Destroying %d faces\n", faces_to_destroy.num());
  for (Face f : faces_to_destroy) mesh.destroy_face(f);
}

void do_trimpts(Args& args) {
  string filename = args.get_filename();
  float dtrim = args.get_float();
  assertx(!mesh.empty());
  const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
  const Frame xform = bbox.get_frame_to_small_cube();
  PointSpatial<int> spatial(800);
  Array<Point> points;
  {
    RFile fi(filename);
    RSA3dStream ia3d(fi());
    A3dElem el;
    int totpts = 0;
    for (;;) {
      ia3d.read(el);
      if (el.type() == A3dElem::EType::endfile) break;
      if (el.type() == A3dElem::EType::comment) continue;
      if (el.type() != A3dElem::EType::point) {
        Warning("Non-point input ignored");
        continue;
      }
      totpts++;
      Point p = el[0].p * xform;
      if (p[0] <= 0.f || p[0] >= 1.f || p[1] <= 0.f || p[1] >= 1.f || p[2] <= 0.f || p[2] >= 1.f) {
        Warning("point out of bounds");
        continue;
      }
      points.push(p);
    }
    for_int(i, points.num()) spatial.enter(i, &points[i]);
    showdf("Read %d inbound points out of %d points from %s\n", points.num(), totpts, filename.c_str());
  }
  if (getenv_bool("CIRCUMRADIUS")) {  // Note: not really the circumradius!
    // But probably we should use the minimum bounding sphere anyways.
    Array<Face> faces_to_destroy;
    for (Face f : mesh.faces()) {
      const Vec3<Point> triangle = mesh.triangle_points(f);
      const Point pc = interp(triangle);  // Note: not really the circumcenter!
      const float circumd = dist(pc, triangle[0]);
      const float maxd = circumd * dtrim * xform[0][0];
      SpatialSearch<int> ss(&spatial, pc * xform, maxd);
      if (ss.done() || ss.next().d2 > square(maxd)) faces_to_destroy.push(f);
    }
    showdf("Destroying %d faces\n", faces_to_destroy.num());
    for (Face f : faces_to_destroy) mesh.destroy_face(f);
  } else {
    const FlagMask vflag_toofar = Mesh::allocate_Vertex_flag();
    int nvtoofar = 0;
    float maxd = dtrim * xform[0][0];
    for (Vertex v : mesh.vertices()) {
      Point p = mesh.point(v) * xform;
      assertx(p[0] > 0.f && p[0] < 1.f && p[1] > 0.f && p[1] < 1.f && p[2] > 0.f && p[2] < 1.f);
      SpatialSearch<int> ss(&spatial, p, maxd);
      if (ss.done() || ss.next().d2 > square(maxd)) {
        nvtoofar++;
        mesh.flags(v).flag(vflag_toofar) = true;
      }
    }
    showdf("Found %d vertices too far\n", nvtoofar);
    Array<Face> faces_to_destroy;
    for (Face f : mesh.faces())
      if (any_of(mesh.vertices(f), [&](Vertex v) { return mesh.flags(v).flag(vflag_toofar); }))
        faces_to_destroy.push(f);
    showdf("Destroying %d faces\n", faces_to_destroy.num());
    for (Face f : faces_to_destroy) mesh.destroy_face(f);
  }
}

void do_assignwids() {
  string str;
  for (Vertex v : mesh.vertices()) mesh.update_string(v, "wid", csform(str, "%d", mesh.vertex_id(v)));
}

}  // namespace

// *** main

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSC("A mesh is read from stdin or first arg except with the following arguments:");
  HH_ARGSD(creategrid, "ny nx : create grid of quads");
  HH_ARGSD(fromgrid, "ny nx file_of_z_values : create grid of quads");
  HH_ARGSD(frompointgrid, "ny nx file_of_points : create grid of quads");
  HH_ARGSD(createobject, "name : create mesh");
  HH_ARGSD(froma3d, ": build mesh from a3d input");
  HH_ARGSD(rawfroma3d, ": build mesh from a3d input (isolated tris)");
  HH_ARGSD(fromObj, "file.obj [flip] : import obj");
  HH_ARGSC("", ":");
  HH_ARGSD(renumber, ": renumber vertices and faces");
  HH_ARGSD(nidrenumberv, ": renumber vertices to have id=key{'Nid'}");
  HH_ARGSD(merge, "mesh1 mesh2 ... : merge other meshes");
  HH_ARGSD(outmesh, ": output mesh now");
  HH_ARGSD(writemesh, "mesh.m : output mesh to file now");
  HH_ARGSD(record, ": record mesh changes on stdout");
  HH_ARGSF(nooutput, ": do not print mesh at program end");
  HH_ARGSD(setb3d, ": set a3d output to binary");
  HH_ARGSD(toa3d, ": output a3d version of mesh");
  HH_ARGSD(tob3d, ": output binary a3d version of mesh");
  HH_ARGSD(endobject, ": output EndObject marker");
  HH_ARGSD(to_obj, ": output Alias/Wavefront .obj file");
  HH_ARGSC("", ":");
  HH_ARGSD(angle, "deg : tag sharp edges");
  HH_ARGSD(cosangle, "fcos : tag sharp edges, acos(fcos)");
  HH_ARGSD(solidangle, "sterad. : tag cusp vertices (def. 0)");
  HH_ARGSD(tagmateriale, ": tag as sharp edges separating materials");
  HH_ARGSC("", ":");
  HH_ARGSD(transf, "'frame' : affine transform by frame");
  HH_ARGSD(keepsphere, "x y z r : delete faces with vertex outside");
  HH_ARGSD(delaunay, ": retriangulate based on circumradii");
  HH_ARGSD(diagonal, ": retriangulate based on diagonal lengths");
  HH_ARGSD(segment, ": segment mesh and output a3d objects");
  HH_ARGSD(recordsegments, ": record segment number on faces");
  HH_ARGSD(selsmooth, ": selectively smooth, output a3d object");
  HH_ARGSD(showsharpe, ": output sharp edges");
  HH_ARGSD(showmatbnd, ": output edges on material boundaries");
  HH_ARGSC("", ": (next two are obsolete, use Subdivfit)");
  HH_ARGSD(trisubdiv, ": 1 iter of triangular subdivision");
  HH_ARGSD(silsubdiv, ": 1 iter of silhouette subdivision");
  HH_ARGSD(taubinsmooth, "n : n iter of Taubin smoothing");
  HH_ARGSD(desbrunsmooth, "l : Desbrun smoothing with lambda (e.g. 1.)");
  HH_ARGSD(lscm, ": least-squares conformal map parameterization");
  HH_ARGSD(poissonparam, ": Poisson parameterization");
  HH_ARGSC("", ":");
  HH_ARGSD(mark, ": mark tagged elements on output");
  HH_ARGSD(assign_normals, ": save normals as strings on {v, c}");
  HH_ARGSD(removeinfo, ": remove all info (keys, tags) at {v, f, e, c}");
  HH_ARGSD(removekey, "key : remove key strings on all elements");
  HH_ARGSD(renamekey, "elems oldkey newkey : elems=string{vfec}");
  HH_ARGSD(copykey, "elems oldkey newkey : elems=string{vfec}");
  HH_ARGSD(assignkey, "elems key value : elems=string{vfec}");
  HH_ARGSD(copynormaltorgb, ": and rescales into [0, 1]");
  HH_ARGSD(bbox, ": print bounding box");
  HH_ARGSD(tobbox, ": rescale to bounding box");
  HH_ARGSD(genus, ": print mesh genus");
  HH_ARGSD(info, ": print statistics about mesh");
  HH_ARGSD(stat, ": equivalent to '-info -noo'");
  HH_ARGSD(analyzestretch, ": analyze stretch given uv");
  HH_ARGSD(fixvertices, ": disconnect non-nice vertices");
  HH_ARGSD(fixfaces, ": remove contradictory faces");
  HH_ARGSD(nice, ": assert mesh is nice");
  HH_ARGSD(flip, ": flip orientation of faces");
  HH_ARGSD(smootha3d, ": equivalent to ;-angle 180 -selsmooth'");
  HH_ARGSD(fillholes, "nedges : fill holes with <= nedges");
  HH_ARGSP(checkflat, "tol : if > 0, triang faces dif. (rec. 1e-5)");
  HH_ARGSF(alltriangulate, ": subdivide even the triangle");
  HH_ARGSD(triangulate, ": triangulate large faces (centersplit)");
  HH_ARGSD(splitvalence, "val : split vertices with valence>=val");
  HH_ARGSD(splitbnd2valence, ": split boundary vertices of valence 2");
  HH_ARGSD(quadshortdiag, ": triangulate quads using shortest diagonal");
  HH_ARGSD(quadlongdiag, ": triangulate quads using longest diagonal");
  HH_ARGSD(quadaltdiag, ": triangulate quads using alternating diagonal");
  HH_ARGSD(quadxuvdiag, ": triangulate quads using X pattern in uv");
  HH_ARGSD(quadduvdiag, ": triangulate quads using diamond uv pattern");
  HH_ARGSD(quadodddiag, ": triangulate quads to form odd-valence vertices");
  HH_ARGSD(rmcomp, "nfaces : remove components with <=nfaces");
  HH_ARGSD(rmcompn, "ncomp : remove components until <= ncomp");
  HH_ARGSD(coalesce, "fcrit : coalesce planar faces into polygons");
  HH_ARGSD(makequads, "p_tol : coalesce coplanar tris into quads");
  HH_ARGSF(bndmerge, ":  only allow gmerge at boundary vertices");
  HH_ARGSD(gmerge, ": use geometry to merge vertices");
  HH_ARGSD(cornermerge, ": merge info at vertices if possible");
  HH_ARGSD(cornerunmerge, ": unmerge info at vertices");
  HH_ARGSD(slowcornermerge, ": merge info at vertices if possible");
  HH_ARGSD(splitcorners, ": split corners into vertices if different");
  HH_ARGSD(debugsplitcorners, ": split corners into vertices if different");
  HH_ARGSD(checkcorners, "key : count key strings at vertices and corners");
  HH_ARGSD(facemerge, ": merge info at faces if possible");
  HH_ARGSD(rgbvertexmerge, ": merge rgb at vertices if possible");
  HH_ARGSD(rgbfacemerge, ": merge rgb at faces if possible");
  HH_ARGSD(splitmatbnd, ": split vertices if face strings different");
  HH_ARGSD(renormalizenor, ": renormalize normals");
  HH_ARGSD(norgroup, ": assign faces norgroup id (smooth normals)");
  HH_ARGSD(swapdegendiag, ": swap edge next to zero-inscribed-radius faces");
  HH_ARGSD(colorheight, "fac : assign vertex rgb based on elevation");
  HH_ARGSD(colorsqheight, "fac : same but with squared elevation");
  HH_ARGSD(colorbizarre, ": assign vertex rgb interestingly");
  HH_ARGSD(colortransf, "'frame' : affine transform by frame");
  HH_ARGSD(normaltransf, "'frame' : affine transform by frame");
  HH_ARGSD(rmfarea, "area : remove faces with <area");
  HH_ARGSD(rmflarea, "area : remove faces with >area");
  HH_ARGSD(keepfmatid, "id : keep only faces with matid=id");
  HH_ARGSD(uvtopos, ": replace vertex positions by uv");
  HH_ARGSD(perturbz, "scale : perturb z positions by [-1, 1]*scale");
  HH_ARGSD(signeddistcontour, "grid : contour signed distance to mesh");
  HH_ARGSD(splitdiaguv, ": for uv grid, split diagonal edges");
  HH_ARGSD(rmdiaguv, ": for uv grid, remove diagonal edges");
  HH_ARGSD(obtusesplit, ": split obtuse tris, possibly on sphere");
  HH_ARGSD(projectimage, "transf image : color mesh vertices");
  HH_ARGSD(quantizeverts, "bits : apply quantization to each coord.");
  HH_ARGSD(hull, "r : morphological bloat/shrink");
  HH_ARGSD(alignmentframe, "mesh : output rigid frame to align with mesh");
  HH_ARGSD(smoothgim, "nsubdiv : tessellate bicubic gim");
  HH_ARGSD(subsamplegim, "nsubsamp : subsample a square grid");
  HH_ARGSP(raymaxdispfrac, "frac : maximum ray displacement");
  HH_ARGSD(shootrays, "mesh.orig.m : shoot displacement rays");
  HH_ARGSD(transferkeysfrom, "mesh.m : transfer info from other mesh");
  HH_ARGSD(transferwidkeysfrom, "mesh.m : transfer info from other mesh");
  HH_ARGSD(sphparam_to_tangentfield, "dirx diry dirz : sph param to generate dir field");
  HH_ARGSD(trim, "d : remove large faces");
  HH_ARGSD(trimpts, "file.pts d : remove faces away from points");
  HH_ARGSD(assignwids, ": assign wedge ids");
  HH_ARGSD(procedure, "name... : apply named procedure to mesh");
  HH_ARGSC("", ":");
  HH_ARGSP(nfaces, "n :  stop when mesh has <=n faces");
  HH_ARGSD(normalized_maxcrit, "f :  stop when edge crit >= f * avg_elen^2");
  HH_ARGSP(maxcrit, "f :  stop when edge crit >= f");
  HH_ARGSD(lengthc, ":  use edge length as reduction criterion");
  HH_ARGSD(inscribedc, ":  use face inscribed radius as criterion");
  HH_ARGSD(volumec, ":  use preservation of volume criterion");
  HH_ARGSD(qemc, ":  use qem criterion");
  HH_ARGSD(reduce, ": reduce mesh (must follow these args)");
  HH_ARGSC("", ":");
  HH_ARGSD(randpts, "n : print n random points on mesh");
  HH_ARGSD(vertexpts, ": print mesh vertices as points");
  HH_ARGSD(orderedvertexpts, ": print mesh vertices as points");
  HH_ARGSD(bndpts, "n : print n points on each boundary edge");
  HH_ARGSD(addmesh, ": output a3d endfile + mesh now");
  {
    const string arg0 = args.num() ? args.peek_string() : "";
    if (ParseArgs::special_arg(arg0)) args.parse(), exit(0);
    const bool from_other = contains(
        V<string>("-froma3d", "-rawfroma3d", "-creategrid", "-fromgrid", "-frompointgrid", "-createobject"), arg0);
    if (!from_other) {
      const string filename = args.num() && (arg0 == "-" || arg0[0] != '-') ? args.get_filename() : "-";
      RFile fi(filename);
      for (string line; fi().peek() == '#';) {
        assertx(my_getline(fi(), line));
        if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
      }
      showff("%s", args.header().c_str());
      HH_TIMER("_readmesh");
      mesh.read(fi());
    } else {
      showff("%s", args.header().c_str());
    }
    HH_TIMER("Filtermesh_postread");
    args.parse();
  }
  hh_clean_up();
  if (!nooutput) {
    mesh.write(std::cout);
    std::cout.flush();
  }
  mesh.record_changes(nullptr);  // do not record mesh destruction
  oa3d.flush();
  if (!k_debug) exit_immediately(0);
  return 0;
}
