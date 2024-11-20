// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "G3dOGL/G3d.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"
#include "libHh/Histogram.h"
#include "libHh/MeshOp.h"  // edge_dihedral_angle_cos()
#include "libHh/Quaternion.h"
#include "libHh/Random.h"
#include "libHh/StringOp.h"
using namespace hh;

namespace g3d {

constexpr float k_globe_radius = .65f;  // radius on screen (max = 1)

static int screenrate;
static HH_STAT_NP(Sipf);
static HH_STAT_NP(Sspf);
static float cumtime = 0.f;

static const int g_g3d_ellipse = getenv_int("G3D_ELLIPSE");

template <typename RangeEdges> static void recompute_sharpe(GMesh& mesh, const RangeEdges& range_edges) {
  assertx(anglethresh >= 0.f);
  float vcos = std::cos(rad_from_deg(anglethresh));
  for (Edge e : range_edges) {
    if (mesh.is_boundary(e)) continue;
    bool is_sharp = edge_dihedral_angle_cos(mesh, e) < vcos;
    if (mesh.flags(e).flag(GMesh::eflag_sharp) == is_sharp) continue;
    mesh.flags(e).flag(GMesh::eflag_sharp) = is_sharp;
    mesh.set_string(e, is_sharp ? "sharp" : nullptr);
    mesh.gflags().flag(mflag_ok) = false;
  }
}

void RecomputeSharpEdges(GMesh& mesh) { recompute_sharpe(mesh, mesh.edges()); }

static void recompute_all_sharpe() {
  for (int obn = g_obs.first; obn <= g_obs.last; obn++) RecomputeSharpEdges(*g_obs[obn].get_mesh());
}

void Applyq(const Frame& tq) {
  Vector vtran = viewmode || cob == 0 ? Vector(0.f, 0.f, 0.f) : Vector(g_obs[cob].center());
  if (sizemode && !editmode && !lod_mode) {
    Frame xform = Frame::identity();
    for_int(c, 3) xform[c][c] = std::exp(tq.p()[c] / ddistance);
    Frame& frame = g_obs[cob].tm();
    frame = normalized_frame(Frame::translation(-vtran) * xform * Frame::translation(vtran) * frame);
    return;
  }
  static Frame frame_edit;
  bool ledit = editmode && button_active != 1;  // button1 has old semantics
  if (ledit && !(button_active && selected.selected_vertex)) return;
  if (ledit) frame_edit = g_obs[selected.selected_vertex->obn].t();
  Frame& frame_to_modify = ledit ? frame_edit : viewmode ? tview : g_obs[cob].tm();
  bool is_eye_move = eye_move && !viewmode && !ledit && cob != obview;
  Frame frame_old = frame_to_modify;
  if (object_mode) {
    // Complicated change-of-frame to apply the correct transformation.
    const int ob = obview;
    Frame xform = Frame::translation(vtran) * frame_to_modify * ~g_obs[ob].t();
    Vector vtran2 = xform.p();
    xform *= Frame::translation(-vtran2) * tq * Frame::translation(vtran2);
    frame_to_modify = normalized_frame(Frame::translation(-vtran) * xform * g_obs[ob].t());
  } else {
    Frame frel = selected.frel * Frame::translation(vtran);
    frame_to_modify = normalized_frame(~frel * tq * frel * frame_to_modify);
  }
  if (is_eye_move) {
    Frame& frame = g_obs[obview].tm();
    frame = normalized_frame(frame * ~g_obs[cob].t() * frame_old);
    g_obs[cob].tm() = frame_old;
  }
  if (ledit) {
    auto& [_, mesh, v] = *selected.selected_vertex;
    Point p = mesh->point(v) * frame_to_modify * ~frame_old;
    mesh->set_point(v, p);
    mesh->gflags().flag(mflag_ok) = false;
    if (sizemode && anglethresh >= 0.f) {
      Set<Edge> eredo;
      for (Edge e : mesh->edges(v)) eredo.enter(e);
      for (Face f : mesh->faces(v))
        if (mesh->is_triangle(f)) eredo.enter(mesh->opp_edge(v, f));
      recompute_sharpe(*mesh, eredo);
    }
  }
}

static void get_time() {
  static int nscreens;
  static double lastti;
  double ti = get_precise_time();
  fchange = float(ti - lastti);
  if (override_frametime) fchange = override_frametime;
  lastti = ti;
  if (fchange < 0 || (!prev_needed_redraw && fchange > .1f)) fchange = .1f;
  if (timestat) {
    static unique_ptr<Histogram> hist_spf;
    if (!hist_spf) hist_spf = make_unique<Histogram>("G3d_hist_spf.txt", 20);
    hist_spf->add(fchange);
  }
  if (timestat) Sspf.enter(fchange);
  if (fchange > 1.f) fchange = 1.f;
  nscreens++;
  static double lastsec;
  if (ti > lastsec) {
    lastsec = ceil(ti);
    screenrate = nscreens;
    nscreens = 0;
    if (iostat) {
      showf("%2ds %2di inputs/frame:%s\n", screenrate, int(Sipf.sum()), Sipf.short_string().c_str());
      Sipf.zero();
    }
    if (timestat) {
      showf("sec/frame:%s\n", Sspf.short_string().c_str());
      Sspf.zero();
    }
  }
}

static void to_spherical(Vector& v) {
  v /= k_globe_radius;
  float m2 = mag2(v);
  if (m2 > 1.f)
    v.normalize();
  else
    v[0] = -sqrt(1.f - m2);
}

static void act_globe(const Vec2<float>& yxi, const Vec2<float>& yxi_d) {
  Vector vori(0.f, 1.f - selected.yxpressed[1] * 2.f, 1.f - selected.yxpressed[0] * 2.f);
  Vector vnew = vori - Vector(0.f, yxi[1], yxi[0]);
  Vector vold = (ratemode == ERatemode::move) ? vori : vori - Vector(0.f, yxi[1] - yxi_d[1], yxi[0] - yxi_d[0]);
  to_spherical(vold);
  to_spherical(vnew);
  Quaternion q(vold, vnew);  // twice rotation from vold to vnew
  if (ratemode == ERatemode::move) q = pow(q, fchange);
  Applyq(to_Frame(q));
}

static void get_lod(float flevel, int last, int& obi, float& finterp) {
  float fracobi = 1.f + last * flevel;
  if (geomorph) {
    obi = int(fracobi);
    if (obi == last + 1) obi = last;
    finterp = fracobi - obi;
  } else {
    obi = int(fracobi + .5f);
    if (obi == last + 1) obi = last;
    finterp = 0.f;
  }
}

// Note: also called from G3dOGL
void update_lod() {
  if (!lod_mode) return;
  float flevel = min(lod_level, 1.f);
  int obi;
  float finterp;
  get_lod(flevel, g_obs.last, obi, finterp);
  for (int i = g_obs.first; i <= g_obs.last; i++) g_obs[i].set_vis(i == obi);
  cob = obi;
  HB::set_current_object(cob);
  for (int i = 0; i <= g_obs.last; i++) g_obs[i].update();
  HB::segment_morph_mesh(obi, finterp);
}

static void handle_sliders(bool show, float yq) {
  struct Slider {
    string name;
    float* val;
  };
  Array<Slider> sliders;
  float hither = HB::get_hither(), yonder = HB::get_yonder();
  if (lod_mode) {
    sliders.push(Slider{"L.O.D.", &lod_level});
  } else {
    sliders.push(Slider{"hither", &hither});
    sliders.push(Slider{"anglethresh", &anglethresh});
    sliders.push(Slider{"yonder", &yonder});
  }
  if (!show) {
    int i = int(selected.yxpressed[1] * sliders.num() * .9999f);
    float* val = sliders[i].val;
    float oldval = *val;
    if (val == &lod_level)
      *val = 1.1f - (selected.yx[0]) * 1.2f;
    else
      *val *= std::exp(-yq);
    if (val == &anglethresh) {
      if (anglethresh <= 0.f) anglethresh = 45.f;
      if (anglethresh > 180.f) anglethresh = 180.f;
      recompute_all_sharpe();
    } else if (val == &lod_level) {
      lod_level = clamp(lod_level, 0.f, 1.f);
      if (lod_level != oldval) update_lod();
    } else {
      if (yonder) hither = min(hither, yonder);
      if (hither && yonder) yonder = max(yonder, hither);
      HB::set_hither(hither), HB::set_yonder(yonder);
      auto_hither = false;
    }
  } else {
    if (lod_mode) {
      float flevel = min(lod_level, 1.f);
      int obi = 1 + int(g_obs.last * flevel);
      if (obi == g_obs.last + 1) obi = g_obs.last;
      int nfaces = g_obs[obi].get_mesh()->num_faces();
      HB::draw_row_col_text(V(1, 10), sliders[0].name);
      HB::draw_row_col_text(V(2, 11), sform("%.3f", *sliders[0].val));
      HB::draw_row_col_text(V(1, 21), "#Faces");
      HB::draw_row_col_text(V(2, 22), sform("%d", nfaces));
      float xleft = .05f, yline = .004f;
      {  // current level
        float lod = clamp(lod_level, 0.f, 1.f);
        float x1 = xleft + .01f, x2 = xleft + .05f;
        float y1 = (1.1f - lod) / 1.2f - .002f;
        float y2 = y1 + yline;
        HB::draw_segment(V(y1, x1), V(y1, x2));
        HB::draw_segment(V(y2, x1), V(y2, x2));
        HB::draw_segment(V(y1, x1), V(y2, x1));
        HB::draw_segment(V(y1, x2), V(y2, x2));
      }
      {  // slider
        float y1 = (1.1f - 0) / 1.2f + .004f, y2 = (1.1f - 1) / 1.2f - .004f, yd = .01f;
        float x1 = xleft + .02f, x2 = xleft + .04f, xm = xleft + .03f;
        HB::draw_segment(V(y1 + yd, x1), V(y1, xm));
        HB::draw_segment(V(y1 + yd, x2), V(y1, xm));
        HB::draw_segment(V(y1 + yd, x1), V(y1 + yd, x2));
        HB::draw_segment(V(y2 - yd, x1), V(y2, xm));
        HB::draw_segment(V(y2 - yd, x2), V(y2, xm));
        HB::draw_segment(V(y2 - yd, x1), V(y2 - yd, x2));
        HB::draw_segment(V(y1, xm), V(y2, xm));
      }
      int n = g_obs.last;
      for_int(i, n + 1) {  // intervals
        float y = (1.1f - float(i) / n) / 1.2f;
        HB::draw_segment(V(y, xleft + .023f), V(y, xleft + .037f));
      }
    } else {
      for_int(i, sliders.num()) {
        float x = (i + .2f) / sliders.num();
        HB::draw_text(V(.12f, x), sliders[i].name);
        HB::draw_text(V(.17f, x), sform("%g", *sliders[i].val));
      }
    }
  }
}

void Dolly(const Vec2<float>& yxq) {
  Vector v(yxq[0] * ddistance, 0.f, 0.f);
  if (0) v[1] = yxq[1] * ddistance;  // control y too (no)
  if (cob != obview) {
    float d = (viewmode ? dist(tview.p(), Point(0.f, 0.f, 0.f))
                        : dist(g_obs[obview].t().p(), g_obs[cob].center() * g_obs[cob].t()));
    d = max(d, .001f) * 2;
    v *= d;
  }
  if (cob != obview) v = -v;
  float d = dist(g_obs[obview].t().p(), g_obs[cob].center() * g_obs[cob].t());
  Applyq(Frame::translation(v));
  static const bool dolly_lod = getenv_bool("G3D_DOLLY_LOD");
  if (lod_mode && sizemode && dolly_lod) {
    float dn = dist(g_obs[obview].t().p(), g_obs[cob].center() * g_obs[cob].t());
    float ratio = d / (max(dn, 1e-10f));
    lod_level *= ratio;
    update_lod();
  }
}

static void pan(const Vec2<float>& yxq) {
  Vector v(0.f, yxq[1] * ddistance, yxq[0] * ddistance);
  if (cob != obview && mode_centroid) {
    float d = (viewmode ? dist(tview.p(), Point(0.f, 0.f, 0.f))
                        : dist(g_obs[obview].t().p(), g_obs[cob].center() * g_obs[cob].t()));
    d = max(d, .001f) * zoom * 2;
    v *= d;
  }
  if (cob != obview) v = -v;
  Applyq(Frame::translation(v));
}

static void act_button1(const Vec2<float>& yxq) {
  if (sizemode && !(lod_mode && selected.yxpressed[1] > .2)) {
    handle_sliders(false, yxq[0]);
  } else if (selected.shift) {  // pan
    pan(yxq);
  } else {  // rotate
    // Applyq(Frame::rotation(2, yxq[1]) * Frame::rotation(1, -yxq[0]));
    Vector axis(0.f, -yxq[0], yxq[1]);
    Quaternion q(axis, mag(yxq));
    Applyq(to_Frame(q));
  }
}

static void act_button2(const Vec2<float>& pyxq) {
  Vec2<float> yxq = pyxq;
  if (selected.shift) {  // rotation x
    yxq[0] *= -1.f;      // since moving to shift key
    if (cob != obview) yxq[0] *= -1.f;
    Applyq(Frame::rotation(0, yxq[0]));
  } else {  // pan
    pan(yxq);
  }
}

static void act_button3(const Vec2<float>& yxq) {
  if (selected.shift) {  // zoom
    float a = std::exp(-yxq[0]);
    zoom *= a;
    if (object_mode && cob != obview) {
      float d = dist(g_obs[obview].t().p(), g_obs[cob].center() * g_obs[cob].t());
      float dnew = d / a, disp = d - dnew;
      g_obs[obview].tm() = Frame::translation(V(disp, 0.f, 0.f)) * g_obs[obview].t();
    }
  } else {  // dolly (translation on x)
    Dolly(yxq);
  }
}

static void act_button() {
  assertx(button_active);
  const auto pointer = assertw(HB::get_pointer());
  if (!pointer) {
    if (!keep_active) button_active = 0;
    return;
  }
  const Vec2<float> yx = *pointer;
  selected.yx = yx;
  Vec2<float> yxi;
  if (ratemode == ERatemode::step) {
    yxi = selected.yxpressed * 2.f - 1.f;
  } else {
    yxi = (yx - selected.yxpressed) * 2.f;
  }
  // i +- 0 to 1 (half-screen) to 2 (full-screen)
  Vec2<float> yxi_d = yxi - selected.yxio;
  selected.yxio = yxi;
  Vec2<float> yxf;
  if (expo && ratemode != ERatemode::position) {
    // f +- 0 to 8 to 64  * .2 == 0 to 1.6 to 13
    float c1 = 1.f;  // 2012-04-17 was previously .2f;
    for_int(c, 2) yxf[c] = pow(abs(yxi[c]) * 2.f, 3.f) * sign(yxi[c]) * c1;
  } else {
    yxf = yxi;
  }
  Vec2<float> yxfd = yxf - selected.yxfo;
  selected.yxfo = yxf;
  Vec2<float> yxq;
  switch (ratemode) {
    case ERatemode::position: yxq = yxfd; break;
    case ERatemode::move: yxq = yxf * fchange; break;
    case ERatemode::step: yxq = yxf; break;
    default: assertnever("");  // yxq = twice(0.f);
  }
  switch (button_active) {
    case 1: globemode ? act_globe(yxi, yxi_d) : act_button1(yxq); break;
    case 2: act_button2(yxq); break;
    case 3: act_button3(yxq); break;
    default: Warning("unrecognized button");
  }
  if (ratemode == ERatemode::step) button_active = 0;
}

static void act_fly() {
  Vec2<float> yxf = twice(.5f);
  if (const auto pointer = HB::get_pointer())
    if (!viewmode && !button_active) yxf = min(max(*pointer, twice(0.f)), twice(1.f));
  // Convert from (0..1)screen to (-1..1)math.
  yxf = (2.f * yxf - twice(1.f)) * V(-1.f, 1.f);
  const float a = .1f;
  yxf *= a;
  Frame frame1 = Frame::translation(V(ddistance * .05f, 0.f, 0.f));
  Frame frame2 = to_Frame(Quaternion(Vector(0.f, -yxf[0], -yxf[1]), mag(yxf)));
  Frame frame = frame1 * frame2;
  static const bool g3d_fly_use_frame_speed = getenv_bool("G3D_FLY_USE_FRAME_SPEED");
  assertw(!g3d_fly_use_frame_speed);
  if (!g3d_fly_use_frame_speed) {
    frame = pow(frame, 10.f * fchange);
  } else {
    const float frame_rate = 32.f;  // frames / sec;
    frame = pow(frame, 10.f * (1.f / frame_rate));
  }
  Applyq(frame);
}

static void act_flight() {
  Vec2<float> yxf = twice(.5f);
  if (const auto pointer = HB::get_pointer())
    if (!viewmode && !button_active) yxf = min(max(*pointer, twice(0.f)), twice(1.f));
  // Convert from (0..1)screen to (-1..1)math.
  yxf = (2.f * yxf - twice(1.f)) * V(-1.f, 1.f);
  // following coded adapted from g3dfly.c
  float speed = .7f;
  float fturn = abs(speed) > .1f ? .8f / pow(abs(speed), .7f) : .3f;
  Frame& obframe = g_obs[cob].tm();
  Vec3<float> ang = euler_angles_from_frame(obframe);
  {
    Frame frame1 = Frame::rotation(0, yxf[1] * fturn * .037f);         // roll
    Frame frame2 = Frame::rotation(1, yxf[0] * fturn * .037f);         // pitch
    Frame frame3 = Frame::translation(V(ddistance * .05f, 0.f, 0.f));  // forward
    Frame frame = frame3 * frame2 * frame1;
    frame = pow(frame, 10.f * fchange);
    obframe = normalized_frame(frame * obframe);
  }
  {
    float a = deg_from_rad(ang[2]);
    if (abs(a) > 90.f) a = (180.f - abs(a)) * sign(a);
    if (abs(a) > 45.f) a = 45.f * sign(a);
    a = sign(a) * pow(abs(a) / 45.f, .5f);
    // Doing "translation * rotation * ~translation" is incorrect!
    // It gives rise to a small secondary translation.  It is unclear why g3dfly.c doesn't show that problem.
    Point savep = obframe.p();
    Frame frame1 = Frame::translation(-obframe.p());
    const float yaw_factor = 0.032f;                                                      // was 0.025f in g3dfly.c
    Frame frame2 = Frame::rotation(2, -a * yaw_factor * fturn - yxf[1] * fturn * .004f);  // yaw
    Frame frame = frame1 * frame2;
    frame = pow(frame, 10.f * fchange);
    obframe = normalized_frame(obframe * frame);
    obframe.p() = savep;
  }
}

static void act_auto() {
  bool osizemode = sizemode;
  sizemode = false;
  // default at 60fps is fchange == .016667 ddistance==1, so rotation of .016667 / TAU * 360 = 0.954929896 degree/frame
  const float fudge = 1. / 0.954929896;  // desire 1 degree/frame at 60fps -> 360 frames == 6 sec for one rotation
  float ch = fchange * ddistance * fudge;
  //
  if (timingtest_nframes) {
    static unique_ptr<Stat> pstat;
    if (!pstat) {
      pstat = make_unique<Stat>();
    } else {
      pstat->enter(fchange);
    }
    --timingtest_nframes;
    if (!timingtest_nframes) {
      flightmode = EFlightmode::none;
      showf("Timing:%s\n", pstat->short_string().c_str());
      pstat = nullptr;
      return;
    }
    ch = TAU * ddistance / full_timingtest_nframes;
    if (object_mode) ch *= 10.f;
  }
  if (object_mode) {
    static float t = 0.f;
    static float tt = 0.f;
    t += ch;
    static const bool g3d_rev_change_axis = getenv_bool("G3D_REV_CHANGE_AXIS");
    if (g3d_rev_change_axis && t > 3 && Random::G.unif() < 1.f / 3.f) {
      tt += t;
      t = 0.f;
    }
    float th = -tt * (TAU / 21.f);
    act_button1(V(abs(std::sin(th)) * ch, abs(std::cos(th)) * ch));
  } else {
    static const bool g3d_rev_auto = getenv_bool("G3D_REV_AUTO");
    if (g3d_rev_auto) ch = -ch;
    act_button1(V(0.f, ch));
  }
  sizemode = osizemode;
}

static void act_bobble() {
  // Make the tview frame origin (e.g. left eye offset) orbit about the yz plane (perpendicular to view direction).
  const int axis = 0;                                                 // +X is forward
  const float bobble_speed = getenv_float("G3D_BOBBLE_SPEED", 1.0f);  // cycles per second
  const float angle = TAU * bobble_speed * fchange;
  tview.p() = tview.p() * Frame::rotation(axis, angle);
}

static void g3d_michael1() {
  float d = dist(g_obs[0].t().p(), g_obs[1].center());
  float relsize = g_obs[1].radius() / zoom / d;
  relsize *= 15;
  int obn = relsize < 0.3 ? 1 : relsize < 0.6 ? 2 : relsize < 1.0 ? 3 : relsize < 7 ? 4 : relsize < 25 ? 5 : 6;
  for (int i = g_obs.first; i <= g_obs.last; i++) g_obs[i].set_vis(i == obn);
}

static void ellipse_config(int inst, int nlod, Frame& frame_ellipse, Frame& frame, int& obi, float& finterp) {
  int ninst = g_obs.last / nlod;
  assertx(nlod * ninst == g_obs.last);
  float obradius = g_obs[nlod].radius();
  float r1 = obradius * 3;
  float r2 = obradius * 20;
  const bool object_up_y = true;
  const int timeperiod = 12;  // seconds/revolution
  float ang = (cumtime / timeperiod + float(inst) / ninst) * TAU;
  Frame frame_ob_up = Frame::identity();
  {
    Point p;
    Vector v1, v2, v3;
    p = Point(std::cos(ang) * r1, std::sin(ang) * r2, 0.f);
    if (0) {
      v1 = ok_normalized(Vector(-std::sin(ang) * r2, std::cos(ang) * r1, 0.f));
      v2 = ok_normalized(Point(0.f, 0.f, 0.f) - p);
    } else {
      v1 = Vector(1.f, 0.f, 0.f);
      v2 = Vector(0.f, 1.f, 0.f);
    }
    v3 = Vector(0.f, 0.f, 1.f);
    frame_ellipse = Frame(v1, v2, v3, p);
  }
  if (object_up_y)
    frame_ob_up = Frame(Vector(1.f, 0.f, 0.f), Vector(0.f, 0.f, 1.f), Vector(0.f, -1.f, 0.f), Point(0.f, 0.f, 0.f));
  frame = frame_ob_up * frame_ellipse;
  float flevel = .5f + .5f * std::sin(ang);
  get_lod(flevel, nlod, obi, finterp);
}

static void g3d_ellipse1() {
  int ninst = g_g3d_ellipse;
  int nlod = g_obs.last / ninst;
  assertx(nlod * ninst == g_obs.last);
  for_int(i, ninst) {
    Frame frame_ellipse, frame;
    int obi;
    float finterp;
    ellipse_config(i, nlod, frame_ellipse, frame, obi, finterp);
    obi = i * nlod + obi;
    for_int(j, nlod) {
      int ob = 1 + i * nlod + j;
      g_obs[ob].tm() = frame;
      g_obs[ob].set_vis(ob == obi);
    }
    HB::segment_morph_mesh(obi, finterp);
  }
  for (int i = 0; i <= g_obs.last; i++) g_obs[i].update();
}

static void g3d_ellipse2() {
  int ninst = g_g3d_ellipse;
  int nlod = g_obs.last / ninst;
  assertx(nlod * ninst == g_obs.last);
  float obradius = g_obs[nlod].radius();
  for_int(i, ninst) {
    Frame frame_ellipse, frame;
    int obi;
    float finterp;
    ellipse_config(i, nlod, frame_ellipse, frame, obi, finterp);
    obi = i * nlod + obi;
    int nfaces = g_obs[obi].get_mesh()->num_faces();
    Point pabove = Point(0.f, 0.f, obradius * 1.8f) * frame_ellipse;
    const auto [zs, xys] = HB::vdc_from_world(pabove);
    if (xys) {
      const auto [xs, ys] = *xys;
      HB::draw_text(V(ys - .01f, xs), sform("%d", nfaces));
    } else {
      SHOW(pabove);
      SHOW(zs);
    }
  }
}

static void change_frames() {
  switch (flightmode) {
    case EFlightmode::none:
      if (button_active) act_button();
      break;
    case EFlightmode::fly:
      act_fly();
      if (viewmode && button_active) act_button();
      break;
    case EFlightmode::flight:
      act_flight();
      if (viewmode && button_active) act_button();
      break;
    case EFlightmode::automatic:
      act_auto();
      if (button_active) act_button();
      break;
    case EFlightmode::bobble:
      act_bobble();
      if (button_active) act_button();
      break;
    default: assertnever("");
  }
  static const bool g3d_michael = getenv_bool("G3D_MICHAEL");
  if (g3d_michael) g3d_michael1();
  if (g_g3d_ellipse) g3d_ellipse1();
}

static void set_viewing() {
  bool is_view = !tview.is_ident();
  if (auto_level) g_obs[obview].tm() = make_level(g_obs[obview].t());
  // while (obview && !g_obs[obview].visible()) --obview;
  Frame tpos = g_obs[obview].t();  // original frame
  // =~FrameMakeStdDir()
  Frame thead = tpos;  // view dir., after view offset, before aim
  static const bool g3d_radar = getenv_bool("G3D_RADAR");
  if (g3d_radar && is_view && auto_level)  // auto_level radar view
    thead = Frame(Vector(1.f, 0.f, 0.f), Vector(0.f, 1.f, 0.f), Vector(0.f, 0.f, 1.f), tpos.p());
  if (is_view) thead = tview * thead;
  Frame frame_camera = thead;  // final camera transform
  float vzoom = zoom;
  if (is_view) {
    const float g3d_view_zoom = getenv_float("G3D_VIEW_ZOOM", 0.f);  // 2017-02-21
    if (g3d_view_zoom) vzoom = g3d_view_zoom;                        // was 0.2f
  }
  // HB::set_camera(tpos, zoom, frame_camera, vzoom);
  HB::set_camera(g_obs[0].t(), zoom, frame_camera, vzoom);
  if (1 && g_obs.first == 0) g_obs[0].set_vis(is_view || obview != 0);
  if (auto_hither) {
    Bbox<float, 3> gbb;
    Frame frame_camera_inv = inverse(frame_camera);
    int firstobn = g_obs.first == 0 && (is_view || obview != 0) ? 0 : 1;
    for (int obn = firstobn; obn <= g_obs.last; obn++)
      gbb.union_with(g_obs[obn].bbox().transform(g_obs[obn].t() * frame_camera_inv));
    float minx = gbb[0][0];          // could be negative
    if (minx > 0.f) minx *= .9999f;  // for -key ojo on zero-height data.
    float diam = gbb.max_side();
    // float thresh = diam*1e-4;
    float thresh = diam * 2e-3f;  // made larger for OpenGL glPolygonOffsetEXT()
    if (minx < thresh) minx = thresh;
    HB::set_hither(minx);
  }
}

static void update_segs() {
  for (int i = 0; i <= g_obs.last; i++) g_obs[i].update();
}

void ShowInfo() {
  if (info) {
    Vec3<float> ang = euler_angles_from_frame(g_obs[obview].t());
    const Point& p = g_obs[obview].t().p();
    string s;
    s = sform("%2d%c%c%c%c%c%c%c%c%c%c%c%c%s%3d%s",  //
              cob,
              (ratemode == ERatemode::position ? 'p'
               : ratemode == ERatemode::move   ? 'm'
               : ratemode == ERatemode::step   ? 's'
                                               : '?'),
              (flightmode == EFlightmode::none        ? ' '
               : flightmode == EFlightmode::fly       ? 'f'
               : flightmode == EFlightmode::flight    ? 'F'
               : flightmode == EFlightmode::automatic ? 'J'
               : flightmode == EFlightmode::bobble    ? 'B'
                                                      : '?'),
              object_mode ? 'o' : ' ', eye_move ? 'e' : ' ', globemode ? 'g' : ' ', sizemode ? 'S' : ' ',
              (viewmode            ? 'v'
               : !tview.is_ident() ? 'V'
                                   : ' '),
              editmode ? 'E' : ' ', obview ? '0' + obview : ' ', auto_level ? 'l' : ' ', input ? 'I' : ' ',
              output ? 'O' : ' ', HB::show_info().c_str(), screenrate,
              (info != 2 ? ""
                         : sform(" [%5.2f] x%12g y%12g z%12g a%+4.0f b%+4.0f p%+4.0f",  //
                                 zoom, p[0], p[1], p[2],                                //
                                 deg_from_rad(ang[0]), deg_from_rad(ang[1]), deg_from_rad(ang[2]))
                               .c_str()));
    static const bool show_fps = getenv_bool("G3D_SHOW_FPS");
    // if (HB::get_font_dims()[1] > 9 ...)
    if (show_fps) s = sform("[fps%2d]", screenrate);
    HB::draw_row_col_text(V(0, 0), s);
  }
  if (info == 2 && cob >= g_obs.first) {
    static const GMesh* opmesh = nullptr;
    static int onfaces = 0;
    static string str;
    const GMesh* pmesh = g_obs[cob].get_mesh();
    int nfaces = pmesh->num_faces();
    if (pmesh != opmesh || nfaces != onfaces) {
      opmesh = pmesh;
      onfaces = nfaces;
      if (pmesh->num_vertices() <= 50'000) {  // For speed, only compute on smaller meshes.
        str = mesh_genus_string(*pmesh);
      } else {
        str = sform("vertices=%d faces=%d", pmesh->num_vertices(), pmesh->num_faces());
      }
    }
    HB::draw_row_col_text(V(1, 0), str);
  }
}

static void show_caption() {
  if (caption == "") return;
  int loc = -1;
  string s = caption;
  if (s[0] == '-') {
    loc = to_int(s);
    if (!contains(s, ' ')) return;
    s.erase(0, s.find(' ') + 1);
  }
  HB::draw_row_col_text(V(loc, std::numeric_limits<int>::max()), s);
}

static void show_cross() {
  if (info < 2 || HB::get_font_dims()[1] > 11) return;  // Used to be "> 9".
  float r = .01f;
  HB::draw_segment(V(.5f - r, .5f - r), V(.5f + r, .5f + r));
  HB::draw_segment(V(.5f + r, .5f - r), V(.5f - r, .5f + r));
}

static void show_globe() {
  if (!info || !globemode) return;
  const int n = 40;
  Vec2<float> yxo;
  dummy_init(yxo);
  for_int(i, n) {
    float a = i * TAU / n;
    Vec2<float> yx = .5f + V(std::sin(a), std::cos(a)) * .5f * k_globe_radius;
    if (i & 0x1) HB::draw_segment(yxo, yx);
    yxo = yx;
  }
}

void Draw() {
  if (keystring != "") {
    for (char ch : keystring) KeyPressed(string(1, ch));
    keystring = "";
  }
  cur_needs_redraw = false;
  if (input && !tried_input) ReadInput(false);
  tried_input = false;
  CloseIfOpen();
  if (iostat) Sipf.enter(num_input_frames);
  num_input_frames = 0;
  if (want_jump > 1)
    DoJump();
  else
    want_jump = 0;
  get_time();
  cumtime += fchange;
  change_frames();
  if (play) {
    static float play_last = 0.f;
    static const float play_fps = getenv_float("G3D_PLAY_FPS", 30.f);
    if (g_obs.last > 1 && cob > 0 && cumtime >= play_last + 1.f / play_fps) {
      play_last = cumtime;
      g_obs[cob].set_vis(false);
      static bool backwards = false;
      static const bool mirror_loop = !getenv_bool("G3D_ORDINARY_LOOP");
      cob += backwards ? -1 : +1;
      if (cob > g_obs.last) {
        if (mirror_loop) {
          cob = g_obs.last;  // repeat the last object a second time
          backwards = true;
        } else {
          cob = 1;
        }
      } else if (cob < 1) {
        if (mirror_loop) {
          cob = 1;  // repeat the first object a second time
          backwards = false;
        } else {
          cob = g_obs.last;
        }
      }
      g_obs[cob].set_vis(true);
      HB::set_current_object(cob);
    }
    HB::redraw_later();
  }
  if (output) WriteOutput();
  set_viewing();
  update_segs();
  if (lod_mode) {
    static bool is_init = false;
    if (!is_init) {
      is_init = true;
      update_lod();
    }
  }
  HB::draw_space();
  ShowInfo();
  if (sizemode) handle_sliders(true, 0.f);
  show_caption();
  show_cross();
  show_globe();
  if (g_g3d_ellipse) g3d_ellipse2();
  if (button_active || flightmode != EFlightmode::none || g_g3d_ellipse) {
    HB::redraw_later();
    cur_needs_redraw = true;
  }
  prev_needed_redraw = cur_needs_redraw;
}

}  // namespace g3d
