// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <cctype>  // std::isdigit()

#include "G3dOGL/G3d.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"
#include "libHh/MeshOp.h"  // mesh_genus_string()
#include "libHh/Polygon.h"
#include "libHh/Quaternion.h"
#include "libHh/StringOp.h"
using namespace hh;

extern float ambient;  // access from G3dOGL

namespace g3d {

int demofly_mode = 0;
float demofly_idle_time = 0.f;
float demofly_idle_time_thresh = 17.f;

namespace {

bool keylock;

const bool b_g3d_demofly = getenv_bool("G3D_DEMOFLY");

void press_keys(const string& str) {
  for (char ch : str) KeyPressed(string(1, ch));
}

void save_state() {
  ExpandStateFilename();
  try {
    WFile fi(statefile);  // may throw
    Frame ft;
    FrameIO::create_not_a_frame(ft);
    for (int i = 0; i <= g_obs.last; i++) {
      bool is_vis = g_obs[i].visible() || (g_obs.first == 1 && i == 0);
      const Frame& f = is_vis ? g_obs[i].t() : ft;
      if (i && f.is_ident()) continue;
      assertw(FrameIO::write(fi(), f, i, (!i ? zoom : 0.f), false));
    }
    if (!tview.is_ident()) assertw(FrameIO::write(fi(), tview, -1, 0.f, false));
    assertw(fi());
  } catch (const std::runtime_error& ex) {
    SHOW("Cannot save to '" + statefile + "' : " + ex.what());
  }
}

void read_state() {
  ExpandStateFilename();
  try {
    RFile fi(statefile);  // may throw
    string sline;
    while (fi().peek() >= 0) {
      if (fi().peek() == '#') {
        assertx(my_getline(fi(), sline));
        continue;
      }
      Frame f;
      int obn;
      float z;
      bool bin;
      if (!assertw(FrameIO::read(fi(), f, obn, z, bin))) break;
      if (obn == -1) {
        tview = f;
      } else {
        UpdateFrame(obn, f, z);
      }
    }
  } catch (const std::runtime_error& ex) {
    SHOW("Cannot read from '" + statefile + "' : " + ex.what());
  }
}

bool ob_prev() {
  if (cob == g_obs.first) {
    if (g_obs[cob + 1].visible()) return false;
    for (int i = g_obs.first; i <= g_obs.last; i++) g_obs[i].set_vis(true);
  } else {
    g_obs[cob].set_vis(false);
    --cob;
    g_obs[cob].set_vis(true);
    HB::set_current_object(cob);
  }
  for (int i = 0; i <= g_obs.last; i++) g_obs[i].update();
  return true;
}

bool ob_next() {
  if (cob == g_obs.last) return false;
  if (g_obs[cob].visible() && g_obs[cob + 1].visible()) {
    for (int i = g_obs.first; i <= g_obs.last; i++) g_obs[i].set_vis(false);
    g_obs[cob].set_vis(true);
  } else {
    g_obs[cob].set_vis(false);
    cob++;
    g_obs[cob].set_vis(true);
    HB::set_current_object(cob);
  }
  for (int i = 0; i <= g_obs.last; i++) g_obs[i].update();
  return true;
}

void set_hither_yonder() {
  if (cob == obview) return;
  float d = dist(g_obs[obview].t().p(), g_obs[cob].center() * g_obs[cob].t());
  if (!d) return;
  auto_hither = false;
  HB::set_hither(d * .999f);
  HB::set_yonder(d * 1.001f);
}

void reset_hither_yonder() {
  auto_hither = true;
  HB::set_hither(0.f);
  HB::set_yonder(0.f);
}

void all_reset() {
  if (b_g3d_demofly) {
    demofly_idle_time = 0.f;
    while (demofly_mode) KeyPressed(" ");
  }
  for (int i = 0; i <= g_obs.last; i++) g_obs[i].tm() = Frame::identity();
  tview = Frame::identity();
  cob = 1;
  zoom = 1.f;
  object_mode = true;
  eye_move = true;
  ratemode = ERatemode::move;
  globemode = false;
  flightmode = EFlightmode::none;
  sizemode = false;
  ddistance = 1.f;
  auto_level = false;
  viewmode = false;
  editmode = false;
  mode_centroid = true;
  output = false;
  keylock = false;
  if (!b_g3d_demofly) reset_hither_yonder();
  obview = 0;
  for (int i = g_obs.first; i <= g_obs.last; i++) {
    g_obs[i].set_vis(true);
    g_obs[i].update();
  }
  want_jump = 0;
  rec_point = Point(0.f, 0.f, 0.f);
  expo = true;
  geomorph = true;
  timingtest_nframes = 0;
  assertw(HB::special_keypress('Z'));
  if (b_g3d_demofly) {
    press_keys(",eoDe0------lDtDrDbDrDb");
    zoom = .79f;
    auto_hither = false;
    HB::set_hither(.0001f);
    HB::set_yonder(0.f);
    // ambient = .7f;
  }
}

// turn viewing direction toward world point (zeroes the roll)
void aim_towards(const Point& p) { frame_aim_at(g_obs[obview].tm(), p - g_obs[obview].t().p()); }

#if 0
void enter_aim() {
  int obn = cob != obview ? cob : 1;
  Point paim = g_obs[obn].center() * g_obs[obn].t();
  Vector vaim = paim - g_obs[obview].t().p();
  // Make g_obs[obview].tm() point in direction vaim without zeroing roll.
  Vector vf = g_obs[obview].t().v(0);
  assertx(vf.normalize());
  Vector vt = vaim;
  assertx(vt.normalize());
  Quaternion rot(vf, vt);  // get twice the angle!
  rot = pow(rot, .5f);     // divide angle in half
  Frame& frame = g_obs[obview].tm();
  Point pt = frame.p();
  frame.p() = Point(0.f, 0.f, 0.f);
  frame *= to_Frame(rot);
  frame.p() = pt;
}
#endif

void rotate_around() { Applyq(Frame::rotation(2, TAU / 2)); }

// (area is determ / 2)
float determ2d(const Point& p1, const Point& p2, const Point& p3) {
  return (p1[0] * p2[1] - p1[1] * p2[0] + p2[0] * p3[1] - p2[1] * p3[0] + p3[0] * p1[1] - p3[1] * p1[0]);
}

bool select_vertex(const Vec2<float>& yx, int& ret_obn, GMesh*& ret_mesh, Vertex& ret_v) {
  Vec2<int> win_dims = HB::get_extents();
  ret_v = nullptr;
  // must be this close (4 pixel radius)
  // for all vertices in that range, pick closest one
  // select first object for which this is true
  const float maxd = 5 * 1.42f / max(win_dims);
  float minz = BIGFLOAT;
  for (int obn = g_obs.first; obn <= g_obs.last; obn++) {
    if (!g_obs[obn].visible()) continue;
    GMesh& mesh = *g_obs[obn].get_mesh();
    for (Vertex v : mesh.vertices()) {
      Point p = mesh.point(v) * g_obs[obn].t();
      float xp, yp, zp;
      HB::world_to_vdc(p, xp, yp, zp);
      float d = abs(xp - yx[1]) + abs(yp - yx[0]);
      if (d > maxd || zp > minz) continue;
      minz = zp;
      ret_obn = obn;
      ret_mesh = &mesh;
      ret_v = v;
    }
    if (ret_v) break;
    if (subdivmode && obn == 1) break;  // no more objects
  }
  return !!ret_v;
}

bool select_edge(const Vec2<float>& yx, int& ret_obn, GMesh*& ret_mesh, Edge& ret_e, Point& ret_inter) {
  Vec2<int> win_dims = HB::get_extents();
  ret_e = nullptr;
  // must be this close (3 pixels)
  // for all vertices in that range, pick closest one
  // select first object for which this is true
  static const bool prune_backfacing = getenv_bool("PRUNE_BACKFACING");
  const float maxd = 3.f / max(win_dims);
  float minz = BIGFLOAT;
  for (int obn = g_obs.first; obn <= g_obs.last; obn++) {
    if (!g_obs[obn].visible()) continue;
    GMesh& mesh = *g_obs[obn].get_mesh();
    const Frame& t = g_obs[obn].t();
    for (Edge e : mesh.edges()) {
      Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
      Point p1 = mesh.point(v1), p2 = mesh.point(v2);
      Point ps1(0.f, 0.f, 0.f), ps2(0.f, 0.f, 0.f), ps(concat(yx.rev(), V(0.f)));
      float zs1, zs2;
      if (!HB::world_to_vdc(p1 * t, ps1[0], ps1[1], zs1)) continue;
      if (!HB::world_to_vdc(p2 * t, ps2[0], ps2[1], zs2)) continue;
      Vector vd = ok_normalized(ps2 - ps1);
      float a = clamp(dot(vd, ps - ps1) / mag(ps2 - ps1), 0.f, 1.f);
      float d = dist(ps, interp(ps2, ps1, a));
      float z = max(zs1, zs2);
      if (d > maxd || z > minz) continue;
      float minzsn = BIGFLOAT;
      for (Face f : mesh.faces(e)) {
        Polygon poly;
        mesh.polygon(f, poly);
        Point pn = p1 + poly.get_normal_dir();
        float xs, ys, zs;
        HB::world_to_vdc(pn * t, xs, ys, zs);
        minzsn = min(minzsn, zs);
      }
      if (minzsn > zs1 && prune_backfacing) continue;
      minz = z;
      ret_obn = obn;
      ret_mesh = &mesh;
      ret_e = e;
      ret_inter = interp(p2, p1, a);  // not correct for perspective!
    }
    if (ret_e) break;
    if (subdivmode && obn == 1) break;  // no more objects
  }
  return !!ret_e;
}

bool select_face(const Vec2<float>& yx, int& ret_obn, GMesh*& ret_mesh, Face& ret_f) {
  ret_f = nullptr;
  float minz = BIGFLOAT;
  for (int obn = g_obs.first; obn <= g_obs.last; obn++) {
    if (!g_obs[obn].visible()) continue;
    GMesh& mesh = *g_obs[obn].get_mesh();
    const Frame& t = g_obs[obn].t();
    Array<Vertex> va;
    for (Face f : mesh.faces()) {
      mesh.get_vertices(f, va);
      for_int(vi, va.num() - 2) {  // for non-triangles, create fan.
        Vec3<Point> pa;
        pa[0] = mesh.point(va[0]);
        pa[1] = mesh.point(va[vi + 1]);
        pa[2] = mesh.point(va[vi + 2]);
        Vec3<Point> psa;
        Vec3<float> psz, areas;
        bool all_in = true;
        for_int(i, 3) {
          psa[i] = Point(0.f, 0.f, 0.f);
          if (!HB::world_to_vdc(pa[i] * t, psa[i][0], psa[i][1], psz[i])) {
            all_in = false;
            break;
          }
        }
        if (!all_in) continue;
        Point ps(concat(yx.rev(), V(0.f)));
        for_int(i, 3) areas[i] = determ2d(psa[mod3(i + 1)], psa[i], ps) * .5f;
        if (areas[0] < 0 || areas[1] < 0 || areas[2] < 0) continue;
        float z = max({psz[0], psz[1], psz[2]});
        if (z > minz) continue;
        minz = z;
        ret_obn = obn;
        ret_mesh = &mesh;
        ret_f = f;
      }
    }
    if (ret_f) break;
    if (subdivmode && obn == 1) break;  // no more objects
  }
  return !!ret_f;
}

void swap_edge() {
  Vec2<float> yx;
  if (!HB::get_pointer(yx)) {
    HB::beep();
    return;
  }
  int obn;
  GMesh* mesh;
  Edge e;
  Point pint;
  if (!select_edge(yx, obn, mesh, e, pint)) {
    HB::beep();
    return;
  }
  if (!mesh->legal_edge_swap(e)) {
    HB::beep();
    return;
  }
  Edge enew = mesh->swap_edge(e);
  mesh->gflags().flag(mflag_ok) = false;
  for (Face f : mesh->faces(enew)) {
    for (Vertex v : mesh->vertices(f)) mesh->flags(v).flag(vflag_ok) = false;
  }
  if (subdivmode) ClearSubMesh();
  HB::redraw_later();
}

void split_edge() {
  Vec2<float> yx;
  if (!HB::get_pointer(yx)) {
    HB::beep();
    return;
  }
  int obn;
  GMesh* mesh;
  Edge e;
  Point pint;
  if (!select_edge(yx, obn, mesh, e, pint)) {
    HB::beep();
    return;
  }
  Vertex vnew = mesh->split_edge(e);
  mesh->set_point(vnew, pint);
  mesh->gflags().flag(mflag_ok) = false;
  for (Vertex v : mesh->vertices(vnew)) mesh->flags(v).flag(vflag_ok) = false;
  if (subdivmode) ClearSubMesh();
  HB::redraw_later();
}

void collapse_edge(bool tovertex) {
  Vec2<float> yx;
  if (!HB::get_pointer(yx)) {
    HB::beep();
    return;
  }
  int obn;
  GMesh* mesh;
  Edge e;
  Point pint;
  if (!select_edge(yx, obn, mesh, e, pint)) {
    HB::beep();
    return;
  }
  if (!mesh->legal_edge_collapse(e)) {
    HB::beep();
    return;
  }
  Vertex v1 = mesh->vertex1(e), v2 = mesh->vertex2(e);
  Point p1 = mesh->point(v1), p2 = mesh->point(v2);
  mesh->collapse_edge(e);
  if (tovertex) pint = dist2(p1, pint) < dist2(pint, p2) ? p1 : p2;
  mesh->set_point(v1, pint);
  mesh->gflags().flag(mflag_ok) = false;
  mesh->flags(v1).flag(vflag_ok) = false;
  if (subdivmode) ClearSubMesh();
  HB::redraw_later();
}

bool try_toggle_vertex() {
  Vec2<float> yx;
  if (!HB::get_pointer(yx)) return false;
  int obn;
  GMesh* mesh;
  Vertex v;
  if (!select_vertex(yx, obn, mesh, v)) return false;
  bool tagged = mesh->flags(v).flag(GMesh::vflag_cusp);
  showf("Setting vertex %d tag to %d\n", mesh->vertex_id(v), !tagged);
  mesh->flags(v).flag(GMesh::vflag_cusp) = !tagged;
  mesh->set_string(v, mesh->flags(v).flag(GMesh::vflag_cusp) ? "cusp" : nullptr);
  mesh->gflags().flag(mflag_ok) = false;
  mesh->flags(v).flag(vflag_ok) = false;
  return true;
}

bool try_toggle_edge() {
  Vec2<float> yx;
  if (!HB::get_pointer(yx)) return false;
  int obn;
  GMesh* pmesh;
  Edge e;
  Point pint;
  if (!select_edge(yx, obn, pmesh, e, pint)) return false;
  bool tagged = pmesh->flags(e).flag(GMesh::eflag_sharp);
  showf("Setting edge (%d, %d) tag to %d\n", pmesh->vertex_id(pmesh->vertex1(e)), pmesh->vertex_id(pmesh->vertex2(e)),
        !tagged);
  pmesh->flags(e).flag(GMesh::eflag_sharp) = !tagged;
  pmesh->set_string(e, pmesh->flags(e).flag(GMesh::eflag_sharp) ? "sharp" : nullptr);
  pmesh->gflags().flag(mflag_ok) = false;
  for (Vertex v : pmesh->vertices(e)) pmesh->flags(v).flag(vflag_ok) = false;
  return true;
}

void toggle_tag() {
  static const bool tagvertices = getenv_bool("TAG_VERTICES");
  if (!(tagvertices && try_toggle_vertex()) && !try_toggle_edge()) {
    HB::beep();
    return;
  }
  if (subdivmode) ClearSubMesh();
  HB::redraw_later();
}

Array<Edge> get_sharp_edges(GMesh& mesh, Vertex v) {
  Array<Edge> are;
  for (Edge e : mesh.edges(v)) {
    if (mesh.flags(e).flag(GMesh::eflag_sharp)) are.push(e);
  }
  return are;
}

void untag(GMesh& mesh, Edge e) {
  mesh.flags(e).flag(GMesh::eflag_sharp) = false;
  mesh.set_string(e, nullptr);
  mesh.gflags().flag(mflag_ok) = false;
  for (Vertex v : mesh.vertices(e)) mesh.flags(v).flag(vflag_ok) = false;
}

void untag_cut() {
  Vec2<float> yx;
  if (!HB::get_pointer(yx)) return;
  int obn;
  GMesh* mesh;
  Edge e;
  Point pint;
  if (!select_edge(yx, obn, mesh, e, pint)) return;
  if (!mesh->flags(e).flag(GMesh::eflag_sharp)) return;
  if (subdivmode) ClearSubMesh();
  HB::redraw_later();
  {
    untag(*mesh, e);
    Edge er = e;
    Array<Edge> are;
    {
      Edge ee = er;
      Vertex v = mesh->vertex1(ee);
      for (;;) {
        are = get_sharp_edges(*mesh, v);
        if (are.num() != 1) break;
        ee = are[0];
        if (ee == er) return;  // have loop!
        untag(*mesh, ee);
        v = mesh->opp_vertex(v, ee);
      }
    }
    {
      Edge ee = er;
      Vertex v = mesh->vertex2(ee);
      for (;;) {
        are = get_sharp_edges(*mesh, v);
        if (are.num() != 1) break;
        ee = are[0];
        untag(*mesh, ee);
        v = mesh->opp_vertex(v, ee);
      }
    }
  }
}

void destroy_face() {
  Vec2<float> yx;
  if (!HB::get_pointer(yx)) {
    HB::beep();
    return;
  }
  int obn;
  GMesh* mesh;
  Face f;
  if (!select_face(yx, obn, mesh, f)) {
    HB::beep();
    return;
  }
  for (Vertex v : mesh->vertices(f)) mesh->flags(v).flag(vflag_ok) = false;
  mesh->destroy_face(f);
  mesh->gflags().flag(mflag_ok) = false;
  if (subdivmode) ClearSubMesh();
  HB::redraw_later();
}

void print_info_mesh_elements() {
  Vec2<float> yx;
  if (!HB::get_pointer(yx)) {
    HB::beep();
    return;
  }
  SHOW("Retrieving info:");
  int obn;
  GMesh* pmesh;
  Vertex v;
  Face f;
  Edge e;
  Point point;
  if (select_vertex(yx, obn, pmesh, v)) {
    const GMesh& mesh = *pmesh;
    SHOW(mesh.vertex_id(v), mesh.point(v), mesh.get_string(v));
  }
  if (select_edge(yx, obn, pmesh, e, point)) {
    const GMesh& mesh = *pmesh;
    const int v1 = mesh.vertex_id(mesh.vertex1(e));
    const int v2 = mesh.vertex_id(mesh.vertex2(e));
    SHOW("On edge", v1, v2, point, mesh.get_string(e));
  }
  if (select_face(yx, obn, pmesh, f)) {
    const GMesh& mesh = *pmesh;
    SHOW(mesh.face_id(f), mesh.get_string(f));
  }
  if (v && f) {
    // It is possible that v is in a different mesh, or that v and f are not adjacent.
    const GMesh& mesh = *pmesh;
    Array<Vertex> vertices;
    mesh.get_vertices(f, vertices);
    if (vertices.contains(v)) {
      Corner c = mesh.corner(v, f);
      SHOW(mesh.get_string(c));
    } else {
      SHOW("Vertex is not adjacent to face");
    }
  }
}

void crop_mesh_to_view() {
  GMesh& mesh = *g_obs[cob].get_mesh();
  if (!assertw(mesh.num_faces())) {
    HB::beep();
    return;
  }
  {
    const Frame& t = g_obs[cob].t();
    Array<Face> ar_destroy;
    for (Face f : mesh.faces()) {
      bool all_out = true;
      for (Vertex v : mesh.vertices(f)) {
        float xo, yo, zo;
        if (HB::world_to_vdc(mesh.point(v) * t, xo, yo, zo) && xo >= 0.f && xo <= 1.f && yo >= 0.f && yo <= 1.f) {
          all_out = false;
          break;
        }
      }
      if (all_out) ar_destroy.push(f);
    }
    showf("Cropping mesh: %d faces removed\n", ar_destroy.num());
    if (ar_destroy.num()) {
      for (Face f : ar_destroy) mesh.destroy_face(f);
      mesh.gflags().flag(mflag_ok) = false;
      HB::redraw_later();
    }
  }
  {
    Array<Vertex> ar_destroy;
    for (Vertex v : mesh.vertices()) {
      if (!mesh.degree(v)) ar_destroy.push(v);
    }
    for (Vertex v : ar_destroy) mesh.destroy_vertex(v);
    showf(" and %d vertices removed\n", ar_destroy.num());
  }
}

void write_mesh(GMesh& mesh) {
  assertx(cob >= 1 && cob <= g_aargs1.num());
  string filename = g_aargs1[cob - 1];
  showf("overwriting %s...\n", filename.c_str());
  try {
    WFile fi(filename);  // may throw
    mesh.write(fi());
    SHOW("...mesh written");
  } catch (std::runtime_error& ex) {
    SHOW("Could not write to file '" + filename + "' : " + ex.what());
    HB::draw_row_col_text(V(2, 0), "Could not write to file!");
    HB::beep();
  }
}

void reinit() {
  if (!assertw(cob >= g_obs.first)) return;
  GMesh& mesh = *g_obs[cob].get_mesh();
  mesh.gflags().flag(mflag_ok) = false;
  for (Vertex v : mesh.vertices()) mesh.flags(v).flag(vflag_ok) = false;
  for (Face f : mesh.faces()) mesh.flags(f).flag(fflag_ok) = false;
  HB::redraw_now();
}

void set_recpoint() {
  Vec2<float> yx;
  if (!HB::get_pointer(yx)) {
    HB::beep();
    return;
  }
  int obn;
  GMesh* mesh;
  Face f;
  if (!select_face(yx, obn, mesh, f)) {
    if (!mode_centroid) {
      rec_point = Point(0.f, 0.f, 0.f);
      mode_centroid = true;  // turn on mode
    } else {
      HB::beep();
    }
    return;
  }
  assertx(g_obs[obn].visible());
  if (obn != cob) {
    cob = obn;
    HB::set_current_object(cob);
  }
  // Ideally, would compute actual intersection point within face.
  // For now, just use centroid.
  Polygon poly;
  mesh->polygon(f, poly);
  Point p = mean(poly);
  if (1) showf("Hit face %d (centroid [%g, %g, %g])\n", mesh->face_id(f), p[0], p[1], p[2]);
  rec_point = p;
  mode_centroid = false;
}

void show_all_info() {
  SHOW("Frames:");
  for (int i = 0; i <= g_obs.last; i++) {
    showf("%s", g_obs[i].defined() ? "*" : " ");
    assertw(FrameIO::write(std::cerr, g_obs[i].t(), i, (!i ? zoom : 0.f), false));
  }
  SHOW("Viewing offset:");
  assertw(FrameIO::write(std::cerr, tview, -1, 0.f, false));
  SHOW(ddistance, expo, keep_active, mode_centroid, want_jump, obinary, keystring);
  SHOW(g_obs.first, g_obs.last, HB::get_hither(), HB::get_yonder(), obview, cob);
}

void do_g3d_demofly() {
  static float bu_hither;
  static float bu_ambient;
  demofly_mode++;
  switch (demofly_mode) {
    case 1:  // turn off texture, showing edges
      press_keys("Dt");
      break;
    case 2:  // zoom out
      press_keys("Do");
      break;
    case 3:  // go to upper view
      press_keys("Do");
      obview = 2;
      // zoom is taken care of in set_viewing()
      bu_hither = HB::get_hither();
      HB::set_hither(.05f);
      bu_ambient = ambient;
      ambient = .3f;
      break;
    case 4:  // back to normal view
      demofly_mode = 0;
      press_keys("Dt");
      obview = 0;
      HB::set_hither(bu_hither);
      ambient = bu_ambient;
      break;
    default: assertnever("");
  }
}

void show_help() {
  const string s = &R"(
   Commands:
OBJECT FOCUS:
        select using digits 0-9, 0==eyepoint
REFERENCE Modes:
o       toggle reference frame: object-relative(o) vs. object-absolute
e       toggle move eyepoint instead of object
g       toggle globe mode (virtual sphere)
C       toggle centroid reference frame vs. origin
E       toggle mesh edit mode
S       size mode (translation resizes)  (left button sets hither/yonder)
w       set eyeobject           W       reset eyeobject to 0
v       edit view offset        V       reset view offset to null
a       set object visible
FLIGHT Modes (toggles, mutually exclusive)
f       fly                     J       auto-rotate
DISPLACEMENT Modes (mutually exclusive):
p       position        m       motion          s       step      $ expo
MOUSE BUTTON DRAG:
left:           yaw & pitch (z&y rotation)
middle:         pan (left/right/up/down translation)
right:          dolly (front/back translation)
shift-left:     pan (left/right/up/down translation)
shift-middle:   roll (x rotation)
shift-right:    zoom (change focal length)
" "\
OTHER KEYS:
+ =     movement magnitude (x2) _ +     bigger increments (x10)
|       reverse 180 about xy    H       level horizon (roll & pitch)
l       toggle auto_level       L       level roll
y       set hither/yonder       Y       clear hither/yonder
j       'jump' to good viewing location
r       reread data             u       toggle current object visibility
N       next object             P       previous object
z       zero current frame      Z       zero all frames
i       toggle info (3states)   #       dump out all status info
I       toggle use_input        {       toggle asynchronous input
O       output current frame    !       output single frame
&       toggle output binary/ascii
*       keep motion through button release
B       bobble motion           @       object cycling (movie)
< ,     read s3d file           > .     save all frames in s3d file
'       record current point    \"       compute distance to recorded point
ctrl-C  quit
DEVICE COMMANDS:
D       device prefix           D ?     list device commands
)"[1];
  std::cerr << s;
}

void select_frel() {
  Frame& frel = selected.frel;
  float f1 = -1, f2 = -1;  // best axis correlation
  int oax1 = -1, oax2 = -1, vax1 = -1, vax2 = -1, sign1 = 0, sign2 = 0;
  Frame fob = g_obs[cob].t();
  Frame vob = g_obs[obview].t();
  for_int(i, 3) {
    assertw(fob.v(i).normalize());
    assertw(vob.v(i).normalize());
  }
  for_int(i, 3) {
    for_int(j, 3) {
      float v = dot(fob.v(i), vob.v(j));
      int sign = 1;
      if (v < 0) {
        v = -v;
        sign = -1;
      }
      if (v > f1) {
        f2 = f1;
        oax2 = oax1;
        vax2 = vax1;
        sign2 = sign1;
        f1 = v;
        oax1 = i;
        vax1 = j;
        sign1 = sign;
      } else if (v > f2) {
        f2 = v;
        oax2 = i;
        vax2 = j;
        sign2 = sign;
      }
    }
  }
  assertx(oax1 != oax2 && vax1 != vax2);  // -> unique correspondence
  frel.zero();
  frel[vax1][oax1] = float(sign1);
  frel[vax2][oax2] = float(sign2);
  int a1 = min(vax1, vax2), a2 = max(vax1, vax2);
  int vax3 = a1 == 0 && a2 == 1 ? 2 : a1 == 0 && a2 == 2 ? 1 : 0;
  int dir = (vax2 == mod3(vax1 + 1)) ? 1 : -1;
  frel.v(vax3) = cross(frel.v(vax1), frel.v(vax2)) * float(dir);
}

}  // namespace

void DoJump() {
  if (!assertw(g_obs[cob].defined())) return;
  if (cob == obview) return;
  const Bbox& bbox = g_obs[cob].bbox();
  Vector diag = bbox[1] - bbox[0];
  Point centerbb = bbox[0] + diag / 2.f;
  if (!object_mode) {
    zoom = .2f;
    int minc = arg_min(diag);
    float mind = diag[minc];
    float maxd = max(diag);
    Point newvp = centerbb;
    float a = mind * .5f + maxd * .5f / zoom * 1.1f;
    newvp[minc] += (minc == 2 ? 1 : -1) * a;
    g_obs[obview].tm().p() = newvp * g_obs[cob].t();
    aim_towards(centerbb * g_obs[cob].t());
  } else {
    zoom = .2f;
    Vector v = diag * (.55f / zoom);
    v[0] = -v[0];
    v[1] = -v[1];
    Point newvp = (Point(g_obs[cob].center() + v)) * g_obs[cob].t();
    g_obs[obview].tm().p() = newvp;
    int obn = cob != obview ? cob : 1;
    aim_towards(g_obs[obn].center() * g_obs[obn].t());
  }
}

bool KeyPressed(const string& ps) {
  string s = ps;
  if (s == "<f1>") s = "?";
  if (s.size() != 1) return false;
  char ch = s[0];
  if (b_g3d_demofly && contains(" Zf", ch)) {
    if (demofly_idle_time > demofly_idle_time_thresh) {
      demofly_idle_time = 0.f;  // avoid infinite recursion
      if (ch != ' ') {
        while (demofly_mode) KeyPressed(" ");
      }
    } else {
      demofly_idle_time = 0.f;
    }
  }
  static char lastch;
  char thisch = ch;
  if (keylock && !contains("-=_+xd\tc~ Di? BQ[] avV be ol ^\x03", ch)) return false;
  if (lastch == 'D') {
    if (ch == 'D') return true;
    if (HB::special_keypress(ch)) {
      lastch = 0;
      return true;
    } else {
      return false;
    }
  }
  if (editmode) {
    switch (ch) {
      case 'w':
        swap_edge();
        thisch = ch = 0;
        break;
      case 's':
        split_edge();
        thisch = ch = 0;
        break;
      case 'c':
        collapse_edge(false);
        thisch = ch = 0;
        break;
      case 'v':
        collapse_edge(true);
        thisch = ch = 0;
        break;
      case 't':
        toggle_tag();
        thisch = ch = 0;
        break;
      case 'u':
        untag_cut();
        thisch = ch = 0;
        break;
      case 'f':
        destroy_face();
        thisch = ch = 0;
        break;
      case 'C':
        crop_mesh_to_view();
        thisch = ch = 0;
        break;
      case '@':
        ch = 0;
        if (!assertw(cob >= g_obs.first)) return true;
        if (!g_obs[cob].get_mesh()->num_vertices()) {
          SHOW("empty mesh");
          break;
        }
        if (lastch != thisch) {
          if (cob < 1 || cob > g_aargs1.num()) {
            SHOW("cannot write");
            break;
          }
          showf("about to overwrite %s\n", g_aargs1[cob - 1].c_str());
          break;
        }
        if (cob > g_aargs1.num()) {
          SHOW("cannot write");
          break;
        }
        write_mesh(*g_obs[cob].get_mesh());
        break;
      default: void();
    }
  }
  bool understood = true;
  switch (ch) {
    case 0:
      // previously processed
      break;
    case 'o':
      object_mode = !object_mode;
      button_active = 0;
      HB::redraw_later();
      break;
    case 'e':
      eye_move = !eye_move;
      button_active = 0;
      HB::redraw_later();
      break;
    case 'p':
      ratemode = ERatemode::position;
      button_active = 0;
      HB::redraw_later();
      break;
    case 'm':
      ratemode = ERatemode::move;
      button_active = 0;
      HB::redraw_later();
      break;
    case 's':
      ratemode = ERatemode::step;
      button_active = 0;
      HB::redraw_later();
      break;
    case 'g':
      globemode = !globemode;
      HB::redraw_now();
      break;
    case 'f':
      flightmode = flightmode == EFlightmode::fly ? EFlightmode::none : EFlightmode::fly;
      button_active = 0;
      HB::redraw_now();
      break;
    case 'F':
      flightmode = flightmode == EFlightmode::flight ? EFlightmode::none : EFlightmode::flight;
      button_active = 0;
      HB::redraw_now();
      break;
    case 'J':
      flightmode = flightmode == EFlightmode::automatic ? EFlightmode::none : EFlightmode::automatic;
      button_active = 0;
      HB::redraw_now();
      break;
    case 'B':
      flightmode = flightmode == EFlightmode::bobble ? EFlightmode::none : EFlightmode::bobble;
      button_active = 0;
      HB::redraw_now();
      break;
    case 'S':
      sizemode = !sizemode;
      HB::redraw_later();
      break;
    case '-':
      if (lastch == 'w') {
        thisch = 'w';
        while (obview)
          if (g_obs[--obview].visible()) break;
        HB::redraw_now();
      } else {
        ddistance *= .5f;
      }
      break;
    case '=':
      if (lastch == 'w') {
        thisch = 'w';
        for (;;)
          if (obview == g_obs.last || g_obs[++obview].visible()) break;
        HB::redraw_now();
      } else {
        ddistance *= 2.f;
      }
      break;
    case '_': ddistance *= .1f; break;
    case '+': ddistance *= 10.f; break;
    case 'l':
      auto_level = !auto_level;
      HB::redraw_later();
      if (auto_level) HB::redraw_now();
      break;
    case 'L':
      g_obs[obview].tm() = make_level(g_obs[obview].t());
      HB::redraw_now();
      break;
    case 'H':
      g_obs[obview].tm() = make_horiz(g_obs[obview].t());
      HB::redraw_now();
      break;
    case '|':
      rotate_around();
      HB::redraw_now();
      break;
    case 'i':
      info = (info + 1) % 3;  // 0..2
      HB::redraw_later();
      break;
    case 'v':
      viewmode = !viewmode;
      HB::redraw_later();
      break;
    case 'E':
      editmode = !editmode;
      HB::redraw_later();
      break;
    case 'V':
      tview = Frame::identity();
      HB::redraw_now();
      break;
    case '?':
      show_help();
      HB::draw_row_col_text(V(3, 0), "G3d C++");
      break;
    case 'a':
      // prefix code
      break;
    case 'C':
      mode_centroid = !mode_centroid;
      HB::redraw_now();
      break;
    case '*':
      keep_active = !keep_active;
      if (!keep_active) button_active = 0;
      break;
    case 'I':
      input = !input;
      HB::watch_fd0(input ? InputArrived : nullptr);
      HB::redraw_later();
      break;
    case '!': WriteOutput(); break;
    case 'O':
      output = !output;
      HB::redraw_later();
      break;
    case 'r':
      HB::draw_row_col_text(V(2, 0), "Reading...");
      HB::flush();
      ReadFiles(false);
      HB::reload_textures();
      HB::redraw_now();
      break;
    case 'z':
      g_obs[cob].tm() = Frame::identity();
      HB::redraw_now();
      break;
    case 'Z':
      all_reset();
      HB::redraw_now();
      break;
    case '<':
    case ',':
      read_state();
      HB::redraw_now();
      break;
    case '.':
    case '>':
      if (0 && b_g3d_demofly) break;
      save_state();
      break;
    case '&': obinary = !obinary; break;
    case '#': show_all_info(); break;
    case '^': keylock = !keylock; break;
    case 'y':
      set_hither_yonder();
      HB::redraw_now();
      break;
    case 'Y':
      reset_hither_yonder();
      HB::redraw_now();
      break;
    case ' ':
      if (spacekill) HB::quit();
      if (b_g3d_demofly) {
        do_g3d_demofly();
        HB::redraw_now();
      }
      break;
    case 'w':
      // prefix code
      break;
    case 'W':
      obview = 0;
      HB::redraw_now();
      break;
    case 'N':
      if (ob_next()) HB::redraw_now();
      break;
    case 'P':
      if (ob_prev()) HB::redraw_now();
      break;
    case 'D':
      // prefix code
      break;
    case 'u':
      g_obs[cob].set_vis(!g_obs[cob].visible());
      g_obs[cob].update();
      HB::redraw_now();
      break;
    case 'j':
      if (++want_jump == 3) want_jump = 0;
      DoJump();
      HB::redraw_now();
      break;
    case '\'':
      rec_point = g_obs[obview].t().p();
      SHOW(rec_point);
      break;
    case '"':
      SHOW(g_obs[obview].t().p());
      SHOW(dist(rec_point, g_obs[obview].t().p()));
      break;
    case '{': asynchronousinput = !asynchronousinput; break;
    case '$': expo = !expo; break;
    case 'L' - 64:  // C-l  (== uchar{12})
      reinit();
      break;
    case 'G': geomorph = !geomorph; break;
    case ':':
      flightmode = flightmode == EFlightmode::automatic ? EFlightmode::none : EFlightmode::automatic;
      button_active = 0;
      if (flightmode == EFlightmode::automatic) timingtest_nframes = full_timingtest_nframes + 1;
      HB::redraw_now();
      break;
    case 'x':
      set_recpoint();
      HB::redraw_later();
      break;
    case 'X':
      if (!mode_centroid) {
        rec_point = Point(0.f, 0.f, 0.f);
        mode_centroid = true;  // turn off mode
      } else {
        HB::beep();
      }
      break;
    case '@':
      play = !play;
      HB::redraw_now();
      break;
    case '%': print_info_mesh_elements(); break;
    default:
      if (!std::isdigit(ch)) {
        understood = false;
        break;
      }
      if (lastch == 'w') {
        thisch = 'w';
        obview = ch - '0';
      } else if (lastch == 'a') {
        thisch = 'a';
        cob = clamp(ch - '0', 0, g_obs.last);
        for_intL(ob, g_obs.first, g_obs.last + 1) {
          g_obs[ob].set_vis(ob == cob);
          g_obs[ob].update();
        }
        HB::redraw_now();
      } else {
        cob = clamp(ch - '0', 0, g_obs.last);
        HB::set_current_object(cob);
      }
      HB::redraw_now();
  }
  if (!understood && !HB::special_keypress(ch)) return false;
  lastch = thisch;
  return true;
}

void ButtonPressed(int butnum, bool pressed, bool shift, const Vec2<float>& yx) {
  if (pressed) {
    if (button_active) return;
    if (butnum == 4) {
      if (ob_prev()) HB::redraw_now();
      return;
    } else if (butnum == 5) {
      if (ob_next()) HB::redraw_now();
      return;
    } else if (butnum > 3) {
      SHOW(butnum);
      return;
    }
    if (flightmode == EFlightmode::automatic) {  // added 20020501
      flightmode = EFlightmode::none;
    }
    keep_active = false;
    button_active = butnum;
    selected.shift = shift;
    selected.yxpressed = yx;
    selected.yxfo = twice(0.f);
    selected.yxio = twice(0.f);
    if (editmode && butnum != 1) {
      if (!select_vertex(yx, selected.obn, selected.mesh, selected.v)) HB::beep();
    }
    selected.frel = Frame::identity();
    if (!object_mode && cob != obview) select_frel();
    if (ratemode == ERatemode::step)
      HB::redraw_now();
    else
      HB::redraw_later();
  } else {
    if (butnum == button_active && !keep_active) button_active = 0;
  }
}

void WheelTurned(float v) {
  float yq = v * .15f;
  Dolly(V(yq, 0.f));
  HB::redraw_later();
}

void InputArrived() { ReadInput(false); }

}  // namespace g3d
