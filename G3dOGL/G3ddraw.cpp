// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "G3d.h"
#include "Quaternion.h"
#include "Random.h"
#include "GeomOp.h"
#include "GMesh.h"
#include "MeshOp.h"             // edge_dihedral_angle_cos()
#include "SubMesh.h"
#include "Histogram.h"
#include "StringOp.h"
using namespace hh;

namespace g3d {

constexpr float k_globe_radius = .65f; // radius on screen (max=1)

static int screenrate;
static unique_ptr<SubMesh> smesh;
static HH_STATNP(Sipf);
static HH_STATNP(Sspf);
static float cumtime = 0.f;

static const int g_g3d_ellipse = getenv_int("G3D_ELLIPSE");
static const bool g_g3d_demofly = getenv_bool("G3D_DEMOFLY");

static void recompute_sharpe(GMesh& mesh, const Set<Edge>& eredo) {
    assertx(anglethresh>=0);
    float vcos = cos(to_rad(anglethresh));
    for (Edge e : eredo) {
        if (mesh.is_boundary(e)) continue;
        bool is_sharp = edge_dihedral_angle_cos(mesh, e)<vcos;
        if (mesh.flags(e).flag(GMesh::eflag_sharp)==is_sharp) continue;
        mesh.flags(e).flag(GMesh::eflag_sharp) = is_sharp;
        mesh.set_string(e, is_sharp ? "sharp" : nullptr);
        mesh.flags(mesh.vertex1(e)).flag(vflag_ok) = false;
        mesh.flags(mesh.vertex2(e)).flag(vflag_ok) = false;
        mesh.gflags().flag(mflag_ok) = false;
    }
    if (subdivmode) ClearSubMesh();
}

void RecomputeSharpEdges(GMesh& mesh) {
    Set<Edge> eredo; for (Edge e : mesh.edges()) { eredo.enter(e); }
    recompute_sharpe(mesh, eredo);
}

static void recompute_all_sharpe() {
    for (int obn = g_obs.first; obn<=g_obs.last; obn++) {
        RecomputeSharpEdges(*g_obs[obn].get_mesh());
    }
}

void Applyq(const Frame& tq) {
    Vector vtran = viewmode||cob==0 ? Vector(0.f, 0.f, 0.f) : to_Vector(g_obs[cob].center());
    if (sizemode && !editmode && !lod_mode) {
        Frame f = Frame::identity();
        for_int(c, 3) { f[c][c] = exp(tq.p()[c]/ddistance); }
        g_obs[cob].tm() = Frame::translation(-vtran)*f*Frame::translation(vtran)*g_obs[cob].t();
        return;
    }
    GMesh& mesh = *selected.mesh;
    Vertex v = selected.v;
    static Frame fedit;
    bool ledit = editmode && button_active!=1; // button1 has old semantics
    if (ledit && (!button_active || !v)) return;
    if (ledit) fedit = g_obs[selected.obn].t();
    // Frame::translation(mesh.point(v));
    Frame& fm = ledit ? fedit : viewmode ? tview : g_obs[cob].tm();
    bool is_eye_move = eye_move && !viewmode && !ledit && cob!=obview;
    Frame told = fm;
    if (object_mode) {
        // I have no idea what this does anymore, but it seems to work
        const int ob = obview;
        Frame f = Frame::translation(vtran)*fm*~g_obs[ob].t();
        Vector vtran2 = to_Vector(f.p());
        f *= Frame::translation(-vtran2)*tq*Frame::translation(vtran2);
        fm = Frame::translation(-vtran)*f*g_obs[ob].t();
    } else {
        Frame frel = selected.frel*Frame::translation(vtran);
        fm = ~frel*tq*frel*fm;
    }
    if (is_eye_move) {
        g_obs[obview].tm() *= ~g_obs[cob].t()*told;
        g_obs[cob].tm() = told;
    }
    if (ledit) {
        Point p = mesh.point(v)*fm*~told;
        mesh.set_point(v, p);
        mesh.flags(v).flag(vflag_ok) = false;
        mesh.gflags().flag(mflag_ok) = false;
        if (sizemode && anglethresh>=0) {
            Set<Edge> eredo; for (Edge e : mesh.edges(v)) { eredo.enter(e); }
            for (Face f : mesh.faces(v)) {
                if (!mesh.is_triangle(f)) continue;
                eredo.enter(mesh.opp_edge(v, f));
            }
            recompute_sharpe(mesh, eredo);
        }
    }
}

static void get_time() {
    static int nscreens;
    static double lastti;
    double ti = get_precise_time();
    fchange = float(ti-lastti);
    if (override_frametime) fchange = override_frametime;
    lastti = ti;
    if (fchange<0 || (!prev_needed_redraw && fchange>.1f)) fchange = .1f;
    if (timestat) {
        static unique_ptr<Histogram> hist_spf;
        if (!hist_spf) hist_spf = make_unique<Histogram>("G3d_hist_spf.txt", 20);
        hist_spf->add(fchange);
    }
    if (timestat) {
        Sspf.enter(fchange);
    }
    if (fchange>1.f) fchange = 1.f;
    nscreens++;
    static double lastsec;
    if (ti>lastsec) {
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
    if (m2>1.f) v.normalize();
    else v[0] = -sqrt(1.f-m2);
}

static void act_globe(const Vec2<float>& yxi, const Vec2<float>& yxi_d) {
    Vector vori(0.f, 1.f-selected.yxpressed[1]*2.f, 1.f-selected.yxpressed[0]*2.f);
    Vector vnew = vori-Vector(0.f, yxi[1], yxi[0]);
    Vector vold = (ratemode==ERatemode::move) ? vori : vori-Vector(0.f, yxi[1]-yxi_d[1], yxi[0]-yxi_d[0]);
    to_spherical(vold);
    to_spherical(vnew);
    Quaternion q(vold, vnew);   // twice rotation from vold to vnew
    if (ratemode==ERatemode::move) q = pow(q, fchange);
    Applyq(to_Frame(q));
}

static void get_lod(float flevel, int last, int& obi, float& finterp) {
    float fracobi = 1.f+last*flevel;
    if (geomorph) {
        obi = int(fracobi);
        if (obi==last+1) obi = last;
        finterp = fracobi-obi;
    } else {
        obi = int(fracobi+.5f);
        if (obi==last+1) obi = last;
        finterp = 0.f;
    }
}

// Note: also called from G3dOGL
void update_lod() {
    if (!lod_mode) return;
    float flevel = min(lod_level, 1.f);
    int obi; float finterp; get_lod(flevel, g_obs.last, obi, finterp);
    for (int i = g_obs.first; i<=g_obs.last; i++) {
        g_obs[i].set_vis(i==obi);
    }
    cob = obi;
    HB::set_current_object(cob);
    for (int i = 0; i<=g_obs.last; i++) { g_obs[i].update(); }
    HB::segment_morph_mesh(obi, finterp);
}

static void handle_sliders(bool show, float yq) {
    struct Slider { string name; float* val; };
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
        int i = int(selected.yxpressed[1]*sliders.num()*.9999f);
        float* val = sliders[i].val;
        float oldval = *val;
        if (val==&lod_level) *val = 1.1f-(selected.yx[0])*1.2f;
        else *val *= exp(-yq);
        if (val==&anglethresh) {
            if (anglethresh<=0) anglethresh = 45;
            if (anglethresh>180) anglethresh = 180;
            recompute_all_sharpe();
        } else if (val==&lod_level) {
            lod_level = clamp(lod_level, 0.f, 1.f);
            if (lod_level!=oldval) update_lod();
        } else {
            if (yonder) hither = min(hither, yonder);
            if (hither && yonder) yonder = max(yonder, hither);
            HB::set_hither(hither), HB::set_yonder(yonder);
            auto_hither = false;
        }
    } else {
        if (lod_mode) {
            float flevel = min(lod_level, 1.f);
            int obi = 1+int(g_obs.last*flevel);
            if (obi==g_obs.last+1) obi = g_obs.last;
            int nfaces = g_obs[obi].get_mesh()->num_faces();
            HB::draw_row_col_text(V(1, 10), sliders[0].name);
            HB::draw_row_col_text(V(2, 11), sform("%.3f", *sliders[0].val));
            HB::draw_row_col_text(V(1, 21), "#Faces");
            HB::draw_row_col_text(V(2, 22), sform("%d", nfaces));
            float xleft = .05f, yline = .004f;
            {                   // current level
                float lod = clamp(lod_level, 0.f, 1.f);
                float x1 = xleft+.01f, x2 = xleft+.05f;
                float y1 = (1.1f-lod)/1.2f-.002f;
                float y2 = y1+yline;
                HB::draw_segment(V(y1, x1), V(y1, x2));
                HB::draw_segment(V(y2, x1), V(y2, x2));
                HB::draw_segment(V(y1, x1), V(y2, x1));
                HB::draw_segment(V(y1, x2), V(y2, x2));
            }
            {                   // slider
                float y1 = (1.1f-0)/1.2f+.004f, y2 = (1.1f-1)/1.2f-.004f, yd = .01f;
                float x1 = xleft+.02f, x2 = xleft+.04f, xm = xleft+.03f;
                HB::draw_segment(V(y1+yd, x1), V(y1, xm));
                HB::draw_segment(V(y1+yd, x2), V(y1, xm));
                HB::draw_segment(V(y1+yd, x1), V(y1+yd, x2));
                HB::draw_segment(V(y2-yd, x1), V(y2, xm));
                HB::draw_segment(V(y2-yd, x2), V(y2, xm));
                HB::draw_segment(V(y2-yd, x1), V(y2-yd, x2));
                HB::draw_segment(V(y1, xm), V(y2, xm));
            }
            int n = g_obs.last;
            for_int(i, n+1) {   // intervals
                float y = (1.1f-float(i)/n)/1.2f;
                HB::draw_segment(V(y, xleft+.023f), V(y, xleft+.037f));
            }
        } else {
            for_int(i, sliders.num()) {
                float x = (i+.2f)/sliders.num();
                HB::draw_text(V(.12f, x), sliders[i].name);
                HB::draw_text(V(.17f, x), sform("%g", *sliders[i].val));
            }
        }
    }
}

void Dolly(const Vec2<float>& yxq) {
    Vector v(yxq[0]*ddistance, 0.f, 0.f);
    if (0) v[1] = yxq[1]*ddistance; // control y too (no)
    if (cob!=obview) {
        float d = (viewmode
                   ? dist(tview.p(), Point(0.f, 0.f, 0.f))
                   : dist(g_obs[obview].t().p(), g_obs[cob].center()*g_obs[cob].t()));
        d = max(d, .001f)*2;
        v *= d;
    }
    if (cob!=obview) v = -v;
    float d = dist(g_obs[obview].t().p(), g_obs[cob].center()*g_obs[cob].t());
    Applyq(Frame::translation(v));
    static const bool dolly_lod = getenv_bool("G3D_DOLLY_LOD");
    if (lod_mode && sizemode && dolly_lod) {
        float dn = dist(g_obs[obview].t().p(), g_obs[cob].center()*g_obs[cob].t());
        float ratio = d/(max(dn, 1e-10f));
        lod_level *= ratio;
        update_lod();
    }
}

static void pan(const Vec2<float>& yxq) {
    Vector v(0.f, yxq[1]*ddistance, yxq[0]*ddistance);
    if (cob!=obview && mode_centroid) {
        float d = (viewmode
                   ? dist(tview.p(), Point(0.f, 0.f, 0.f))
                   : dist(g_obs[obview].t().p(), g_obs[cob].center()*g_obs[cob].t()));
        d = max(d, .001f)*zoom*2;
        v *= d;
    }
    if (cob!=obview) v = -v;
    Applyq(Frame::translation(v));
}

static void act_button1(const Vec2<float>& yxq) {
    if (sizemode && !(lod_mode && selected.yxpressed[1]>.2)) {
        handle_sliders(false, yxq[0]);
    } else if (selected.shift) { // pan
        pan(yxq);
    } else {                    // rotate
        // Applyq(Frame::rotation(2, yxq[1])*Frame::rotation(1, -yxq[0]));
        Vector axis(0.f, -yxq[0], yxq[1]);
        Quaternion q(axis, float(mag(yxq)));
        Applyq(to_Frame(q));
    }
}

static void act_button2(const Vec2<float>& pyxq) {
    Vec2<float> yxq = pyxq;
    if (selected.shift) {       // rotation x
        yxq[0] *= -1.f;         // since moving to shift key
        if (cob!=obview) yxq[0] *= -1.f;
        Applyq(Frame::rotation(0, yxq[0]));
    } else {                    // pan
        pan(yxq);
    }
}

static void act_button3(const Vec2<float>& yxq) {
    if (selected.shift) {       // zoom
        float a = exp(-yxq[0]);
        zoom *= a;
        if (object_mode && cob!=obview) {
            float d = dist(g_obs[obview].t().p(), g_obs[cob].center()*g_obs[cob].t());
            float dnew = d/a, disp = d-dnew;
            g_obs[obview].tm() = Frame::translation(V(disp, 0.f, 0.f))*g_obs[obview].t();
        }
    } else {                    // dolly (translation on x)
        Dolly(yxq);
    }
}

static void act_button() {
    assertx(button_active);
    Vec2<float> yx;             // (0, 0)=(top, left)
    if (!assertw(HB::get_pointer(yx))) {
        if (!keep_active) button_active = 0;
        return;
    }
    selected.yx = yx;
    Vec2<float> yxi;
    if (ratemode==ERatemode::step) {
        yxi = selected.yxpressed*2.f-1.f;
    } else {
        yxi = (yx-selected.yxpressed)*2.f;
    }
    // i +- 0 to 1(half-screen) to 2(full-screen)
    Vec2<float> yxi_d = yxi-selected.yxio;
    selected.yxio = yxi;
    Vec2<float> yxf;
    if (expo && ratemode!=ERatemode::position) {
        // f +- 0 to 8 to 64  *.2 == 0 to 1.6 to 13
        float c1 = 1.f;         // 20120417 was previously .2f;
        for_int(c, 2) yxf[c] = pow(abs(yxi[c])*2.f, 3.f)*sign(yxi[c])*c1;
    } else {
        yxf = yxi;
    }
    Vec2<float> yxfd = yxf-selected.yxfo;
    selected.yxfo = yxf;
    Vec2<float> yxq;
    switch (ratemode) {
     bcase ERatemode::position:
        yxq = yxfd;
     bcase ERatemode::move:
        yxq = yxf*fchange;
     bcase ERatemode::step:
        yxq = yxf;
     bdefault: assertnever(""); // yxq=twice(0.f);
    }
    switch (button_active) {
     bcase 1:
        globemode ? act_globe(yxi, yxi_d) : act_button1(yxq);
     bcase 2:
        act_button2(yxq);
     bcase 3:
        act_button3(yxq);
     bdefault:
        Warning("unrecognized button");
    }
    if (ratemode==ERatemode::step) button_active = 0;
}

static void fly_g3d_demofly(Vec2<float>& yxf) {
    bool cursor_is_high = yxf[0]<0.f;
    static bool idle_mode;
    bool new_idle_mode = (yxf[1]<0.f || yxf[1]>1.f || yxf[0]<0.f || yxf[0]>1.f);
    bool out_of_idle = idle_mode && !new_idle_mode;
    idle_mode = new_idle_mode;
    static float idle_dist;
    static bool seeking_point = false;
    static float turn_rate;     // -1.f to 1.f;  0.f=straight
    if (!idle_mode) {
        demofly_idle_time = 0.f;
        idle_dist = 0.f;
        turn_rate = 0.f;
        seeking_point = false;
        if (obview!=0 || !tview.is_ident())
            yxf[0] = .5f;       // if top-view, do not climb or dive
        if (out_of_idle) {
            while (demofly_mode)
                KeyPressed(" ");
        }
    } else {
        yxf = twice(.5f);
        //
        if (seeking_point) turn_rate = 0.f;
        if (turn_rate) yxf[1] = .5f+turn_rate*.07f;
        //
        const Frame& frame = g_obs[0].t();
        Vec3<float> ang = frame_to_euler_angles(frame);
        //
        float pitch = ang[1];                // positive=going_down!
        const float min_pitch = to_rad(6.f); // 6 degrees
        if (frame.p()[2]<.014f && pitch>-min_pitch) {
            if (0) showf("Pulling up pitch=%f\n", pitch);
            yxf[0] = .4f;
        }
        if (frame.p()[2]>.025f && pitch<+min_pitch) {
            if (0) showf("Pushing down pitch=%f\n", pitch);
            yxf[0] = .6f;
        }
        //
        static Point point_desired;
        const float margin = 0.1f;
        bool within_margins = (frame.p()[0]>0.f+margin && frame.p()[0]<1.f-margin &&
                               frame.p()[1]>0.f+margin && frame.p()[1]<1.f-margin);
        if (seeking_point && within_margins) {
            seeking_point = false;
            if (0) SHOW("seeking_point false");
        }
        if (!seeking_point && !within_margins) {
            seeking_point = true;
            for_int(c, 2) point_desired[c] = .2f+.6f*Random::G.unif();
            assertx(!point_desired[2]);
            if (0) SHOW(point_desired);
        }
        if (seeking_point) {
            float yaw_angle_desired = atan2(point_desired[1]-frame.p()[1], point_desired[0]-frame.p()[0]);
            float yaw_angle_diff = yaw_angle_desired-ang[0];
            if (yaw_angle_diff<-TAU/2) yaw_angle_diff += TAU;
            if (yaw_angle_diff>+TAU/2) yaw_angle_diff -= TAU;
            float abs_diff = abs(yaw_angle_diff);
            assertx(abs_diff<=TAU/2);
            bool large_diff = abs_diff>to_rad(10.f);
            bool small_diff = abs_diff>to_rad(4.f);
            static bool honing;
            if (large_diff) honing = true;
            if (!small_diff) honing = false;
            if (honing) {
                if (yaw_angle_diff>0.f) {
                    yxf[1] = .5f-.25f*(ddistance/.0625f); // turning left
                } else {
                    yxf[1] = .5f+.25f*(ddistance/.0625f); // turning right
                }
            }
        }
        float prev_idle_time = demofly_idle_time, prev_idle_dist = idle_dist;
        demofly_idle_time += fchange;
        idle_dist += ddistance/.015625f*fchange;
        if (cursor_is_high && demofly_idle_time>demofly_idle_time_thresh) {
            if (int(demofly_idle_time)!=int(prev_idle_time) && int(demofly_idle_time)%4==0) {
                static int nskip;
                if (demofly_mode==0 && nskip<1) {
                    nskip++;
                } else {
                    nskip = 0;
                    float bu_demofly_idle_time = demofly_idle_time;
                    demofly_idle_time = 0.f;
                    KeyPressed(" ");
                    demofly_idle_time = bu_demofly_idle_time;
                }
            }
        }
        if (1) {
            if (int(idle_dist)!=int(prev_idle_dist) && 1) {
                turn_rate += (-1.f+2.f*Random::G.unif())*.3f;
                turn_rate = clamp(turn_rate, -1.f, +1.f);
            }
        }
    }
    if (1) {
        Frame f = g_obs[0].t();
        const float tol = 0.023f, min_z = 0.f, max_z = 0.06f;
        if (f.p()[2]<min_z) f.p()[2] = min_z;
        if (f.p()[2]>max_z) f.p()[2] = max_z;
        if (f.p()[0]<0.f-tol) f.p()[0] = 1.f+tol;
        if (f.p()[0]>1.f+tol) f.p()[0] = 0.f-tol;
        if (f.p()[1]<0.f-tol) f.p()[1] = 1.f+tol;
        if (f.p()[1]>1.f+tol) f.p()[1] = 0.f-tol;
        g_obs[0].tm() = f;
    }
}

static void act_fly() {
    Vec2<float> yxf;
    if (viewmode || button_active || !HB::get_pointer(yxf)) yxf = twice(.5f);
    if (g_g3d_demofly)
        fly_g3d_demofly(yxf);
    yxf = max(min(yxf, twice(1.f)), twice(0.f));
    // convert form (0..1)screen to (-1..1)math
    yxf = (2.f*yxf-twice(1.f))*V(-1.f, 1.f);
    const float a = .1f;
    yxf *= a;
    Frame f1 = Frame::translation(V(ddistance*.05f, 0.f, 0.f));
    Frame f2 = to_Frame(Quaternion(Vector(0.f, -yxf[0], -yxf[1]), float(mag(yxf))));
    Frame f = f1*f2;
    static const bool g3d_fly_use_frame_speed = getenv_bool("G3D_FLY_USE_FRAME_SPEED");
    assertw(!g3d_fly_use_frame_speed);
    if (!g3d_fly_use_frame_speed) {
        f = pow(f, 10.f*fchange);
    } else {
        const float frame_rate = 32.f; // frame/sec;
        f = pow(f, 10.f*(1.f/frame_rate));
    }
    Applyq(f);
}

static void act_flight() {
    Vec2<float> yxf;
    if (viewmode || button_active || !HB::get_pointer(yxf)) yxf = twice(.5f);
    yxf = max(min(yxf, twice(1.f)), twice(0.f));
    // convert form (0..1)screen to (-1..1)math
    yxf = (2.f*yxf-twice(1.f))*V(-1.f, 1.f);
    // following coded adapted from g3dfly.c
    float speed = .7f;
    float fturn = abs(speed)>.1f ? .8f/pow(abs(speed), .7f) : .3f;
    Frame& obframe = g_obs[cob].tm();
    Vec3<float> ang = frame_to_euler_angles(obframe);
    {
        Frame f1 = Frame::rotation(0, yxf[1]*fturn*.037f);          // roll
        Frame f2 = Frame::rotation(1, yxf[0]*fturn*.037f);          // pitch
        Frame f3 = Frame::translation(V(ddistance*.05f, 0.f, 0.f)); // forward
        Frame f = f3*f2*f1;
        f = pow(f, 10.f*fchange);
        obframe = f*obframe;
    }
    {
        float a = to_deg(ang[2]);
        if (abs(a)>90.f) a = (180.f-abs(a))*sign(a);
        if (abs(a)>45.f) a = 45.f*sign(a);
        a = sign(a)*pow(abs(a)/45.f, .5f);
        // I found that doing translation*rotation*~translation is incorrect!
        // It gives rise to a small secondary translation.
        // I don't know why g3dfly.c doesn't show that bug.
        Point savep = obframe.p();
        Frame f1 = Frame::translation(-to_Vector(obframe.p()));
        const float yaw_factor = 0.032f;                                       // was 0.025f in g3dfly.c
        Frame f2 = Frame::rotation(2, -a*yaw_factor*fturn-yxf[1]*fturn*.004f); // yaw
        Frame f = f1*f2;
        f = pow(f, 10.f*fchange);
        obframe = obframe*f;
        obframe.p() = savep;
    }
}

static void act_auto() {
    bool osizemode = sizemode; sizemode = false;
    // default at 60fps is fchange==.016667 ddistance==1, so rotation of .016667/TAU*360 = 0.954929896 degree/frame
    const float fudge = 1./0.954929896; // desire 1 degree/frame at 60fps -> 360 frames == 6 sec for one rotation
    float ch = fchange*ddistance*fudge;
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
        ch = TAU*ddistance/full_timingtest_nframes;
        if (object_mode) ch *= 10.f;
    }
    if (object_mode) {
        static float t = 0.f;
        static float tt = 0.f;
        t += ch;
        static const bool g3d_rev_change_axis = getenv_bool("G3D_REV_CHANGE_AXIS");
        if (g3d_rev_change_axis && t>3 && Random::G.unif()<1.f/3.f) { tt += t; t = 0.f; }
        float th = -tt*(TAU/21.f);
        act_button1(V(abs(sin(th))*ch, abs(cos(th))*ch));
    } else {
        static const bool g3d_rev_auto = getenv_bool("G3D_REV_AUTO");
        if (g3d_rev_auto) ch = -ch;
        act_button1(V(0.f, ch));
    }
    sizemode = osizemode;
}

static void act_bobble() {
    // Make the tview frame origin (e.g. left eye offset) orbit about the yz plane (perpendicular to view direction).
    const int axis = 0;                 // +X is forward
    const float bobble_speed = getenv_float("G3D_BOBBLE_SPEED", 1.0f); // cycles per second
    const float angle = TAU * bobble_speed * fchange;
    tview.p() = tview.p() * Frame::rotation(axis, angle);
}

static void g3d_michael1() {
    float d = dist(g_obs[0].t().p(), g_obs[1].center());
    float relsize = g_obs[1].radius()/zoom/d;
    relsize *= 15;
    int obn = relsize<0.3 ? 1 : relsize<0.6 ? 2 : relsize<1.0 ? 3 : relsize<7 ? 4 : relsize<25 ? 5 : 6;
    for (int i = g_obs.first; i<=g_obs.last; i++) {
        g_obs[i].set_vis(i==obn);
    }
}

static void ellipse_config(int inst, int nlod, Frame& frame_ellipse, Frame& frame, int& obi, float& finterp) {
    int ninst = g_obs.last/nlod;
    assertx(nlod*ninst==g_obs.last);
    float obradius = g_obs[nlod].radius();
    float r1 = obradius*3;
    float r2 = obradius*20;
    const bool object_up_y = true;
    const int timeperiod = 12;  // seconds/revolution
    float ang = (cumtime/timeperiod+float(inst)/ninst)*TAU;
    Frame frame_ob_up = Frame::identity();
    {
        Point p; Vector v1, v2, v3;
        p = Point(cos(ang)*r1, sin(ang)*r2, 0.f);
        if (0) {
            v1 = ok_normalized(Vector(-sin(ang)*r2, cos(ang)*r1, 0.f));
            v2 = ok_normalized(Point(0.f, 0.f, 0.f)-p);
        } else {
            v1 = Vector(1.f, 0.f, 0.f);
            v2 = Vector(0.f, 1.f, 0.f);
        }
        v3 = Vector(0.f, 0.f, 1.f);
        frame_ellipse = Frame(v1, v2, v3, p);
    }
    if (object_up_y)
        frame_ob_up = Frame(Vector(1.f, 0.f, 0.f), Vector(0.f, 0.f, 1.f), Vector(0.f, -1.f, 0.f),
                            Point(0.f, 0.f, 0.f));
    frame = frame_ob_up*frame_ellipse;
    float flevel = .5f+.5f*sin(ang);
    get_lod(flevel, nlod, obi, finterp);
}

static void g3d_ellipse1() {
    int ninst = g_g3d_ellipse;
    int nlod = g_obs.last/ninst;
    assertx(nlod*ninst==g_obs.last);
    for_int(i, ninst) {
        Frame fellipse, frame; int obi; float finterp; ellipse_config(i, nlod, fellipse, frame, obi, finterp);
        obi = i*nlod+obi;
        for_int(j, nlod) {
            int ob = 1+i*nlod+j;
            g_obs[ob].tm() = frame;
            g_obs[ob].set_vis(ob==obi);
        }
        HB::segment_morph_mesh(obi, finterp);
    }
    for (int i = 0; i<=g_obs.last; i++) { g_obs[i].update(); }
}

static void g3d_ellipse2() {
    int ninst = g_g3d_ellipse;
    int nlod = g_obs.last/ninst;
    assertx(nlod*ninst==g_obs.last);
    float obradius = g_obs[nlod].radius();
    for_int(i, ninst) {
        Frame fellipse, frame; int obi; float finterp; ellipse_config(i, nlod, fellipse, frame, obi, finterp);
        obi = i*nlod+obi;
        int nfaces = g_obs[obi].get_mesh()->num_faces();
        Point pabove = Point(0.f, 0.f, obradius*1.8f)*fellipse;
        float xo, yo, zo;
        if (HB::world_to_vdc(pabove, xo, yo, zo)) {
            yo -= .01f;
            HB::draw_text(V(yo, xo), sform("%d", nfaces));
        } else {
            SHOW(pabove);
            SHOW(zo);
        }
    }
}

static void change_frames() {
    if (flightmode==EFlightmode::fly) {
        act_fly();
        if (viewmode && button_active) act_button();
    } else if (flightmode==EFlightmode::flight) {
        act_flight();
        if (viewmode && button_active) act_button();
    } else if (flightmode==EFlightmode::automatic) {
        act_auto();
        if (button_active) act_button();
    } else if (flightmode==EFlightmode::bobble) {
        act_bobble();
        if (button_active) act_button();
    } else if (button_active) {
        act_button();
    }
    static const bool g3d_michael = getenv_bool("G3D_MICHAEL");
    if (g3d_michael) g3d_michael1();
    if (g_g3d_ellipse) g3d_ellipse1();
}

static void set_viewing() {
    bool is_view = !tview.is_ident();
    if (auto_level) g_obs[obview].tm() = make_level(g_obs[obview].t());
    // while (obview && !g_obs[obview].visible()) --obview;
    Frame tpos = g_obs[obview].t(); // original frame
    // =~FrameMakeStdDir()
    Frame thead = tpos;         // view dir., after view offset, before aim
    static const bool g3d_radar = getenv_bool("G3D_RADAR");
    if (g3d_radar && is_view && auto_level)  // auto_level radar view
        thead = Frame(Vector(1.f, 0.f, 0.f), Vector(0.f, 1.f, 0.f), Vector(0.f, 0.f, 1.f), tpos.p());
    if (is_view) thead = tview*thead;
    Frame tcam = thead;         // final camera transform
    float vzoom = zoom;
    if (is_view) {
        const float g3d_view_zoom = getenv_float("G3D_VIEW_ZOOM", 0.f); // 2017-02-21
        if (g3d_view_zoom) vzoom = g3d_view_zoom;                       // was 0.2f
    }
    if (g_g3d_demofly && obview!=0) vzoom = 0.2f;
    // HB::set_camera(tpos, zoom, tcam, vzoom);
    HB::set_camera(g_obs[0].t(), zoom, tcam, vzoom);
    if (1 && g_obs.first==0) {
        g_obs[0].set_vis(is_view || obview!=0);
    }
    if (auto_hither) {
        Bbox gbb; gbb.clear();
        Frame tcami = inverse(tcam);
        int firstobn = g_obs.first==0 && (is_view || obview!=0) ? 0 : 1;
        for (int obn = firstobn; obn<=g_obs.last; obn++) {
            Bbox bb = g_obs[obn].bb();
            bb.transform(g_obs[obn].t()*tcami);
            gbb.union_with(bb);
        }
        float minx = gbb[0][0];       // could be negative
        if (minx>0.f) minx *= .9999f; // for -key ojo on zero-height data.
        float diam = gbb.max_side();
        // float thresh = diam*1e-4;
        float thresh = diam*2e-3f; // made larger for OpenGL glPolygonOffsetEXT()
        if (minx<thresh) minx = thresh;
        HB::set_hither(minx);
    }
}

static void update_segs() {
    for (int i = 0; i<=g_obs.last; i++) { g_obs[i].update(); }
}

void ShowInfo() {
    if (info) {
        Vec3<float> ang = frame_to_euler_angles(g_obs[obview].t());
        const Point& p = g_obs[obview].t().p();
        string s;
        s = sform("%2d%c%c%c%c%c%c%c%c%c%c%c%c%s%3d%s",
                  cob,
                  (ratemode==ERatemode::position ? 'p' :
                   ratemode==ERatemode::move ? 'm' :
                   ratemode==ERatemode::step ? 's' :
                   '?'),
                  (flightmode==EFlightmode::none ? ' ' :
                   flightmode==EFlightmode::fly ? 'f' :
                   flightmode==EFlightmode::flight ? 'F' :
                   flightmode==EFlightmode::automatic ? 'J' :
                   flightmode==EFlightmode::bobble ? 'B' :
                   '?'),
                  object_mode ? 'o' : ' ',
                  eye_move ? 'e' : ' ',
                  globemode ? 'g' : ' ',
                  sizemode ? 'S' : ' ',
                  (viewmode ? 'v' : !tview.is_ident() ? 'V' : ' '),
                  editmode ? 'E' : ' ',
                  obview ? '0'+obview : ' ',
                  auto_level ? 'l' : ' ',
                  input ? 'I' : ' ',
                  output ? 'O' : ' ',
                  HB::show_info().c_str(),
                  screenrate,
                  (info!=2 ? "" :
                   sform(" [%5.2f] x%12g y%12g z%12g a%+4.0f b%+4.0f p%+4.0f",
                         zoom, p[0], p[1], p[2], to_deg(ang[0]), to_deg(ang[1]), to_deg(ang[2])).c_str()));
        static const bool show_fps = getenv_bool("G3D_SHOW_FPS");
        // if (HB::get_font_dims()[1]>9 ...)
        if (show_fps)
            s = sform("[fps%2d]", screenrate);
        HB::draw_row_col_text(V(0, 0), s);
    }
    if (info==2 && cob>=g_obs.first) {
        static const GMesh* opmesh = nullptr;
        static int onfaces = 0;
        static string str;
        const GMesh* pmesh = g_obs[cob].get_mesh();
        int nfaces = pmesh->num_faces();
        if (pmesh!=opmesh || nfaces!=onfaces) {
            opmesh = pmesh; onfaces = nfaces;
            if (1) {
                str = mesh_genus_string(*pmesh);
            } else {
                str = sform("vertices=%d faces=%d", pmesh->num_vertices(), pmesh->num_faces());
            }
        }
        HB::draw_row_col_text(V(1, 0), str.c_str());
    }
    if (info && g_g3d_demofly) {
        // velocity = 0.05*ddistance * 10*fchange  units/frame
        // fchange = sec/frame
        // -> velocity = 0.05*ddistance * 10  units/sec
        // 1 unit = 10m * 16384
        // 1 mach = 1220km/hour = 338.889 m/sec
        // -> velocity = 0.05*ddistance*10*(10*16384) = ddistance*81920 m/sec
        // -> velocity = above / 338.889 = ddistance*241.73 mach
        float mach = ddistance*241.73f;
        string s = sform("Mach: %.1f (-/=)", mach);
        HB::draw_row_col_text(V(4, 0), s);
        int alt = int(g_obs[0].t().p()[2] * 10*16384 /0.3048f);
        string s2 = sform(" Alt: %d ft", alt);
        HB::draw_row_col_text(V(5, 0), s2);
    }
}

static void show_caption() {
    if (caption=="") return;
    int loc = -1;
    string s = caption;
    if (s[0]=='-') {
        loc = to_int(s.c_str());
        if (!contains(s, ' ')) return;
        s.erase(0, s.find(' ')+1);
    }
    HB::draw_row_col_text(V(loc, INT_MAX), s);
}

static void show_cross() {
    if (!info || HB::get_font_dims()[1]>9) return;
    float r = .01f;
    HB::draw_segment(V(.5f-r, .5f-r), V(.5f+r, .5f+r));
    HB::draw_segment(V(.5f+r, .5f-r), V(.5f-r, .5f+r));
}

static void show_globe() {
    if (!info || !globemode) return;
    const int n = 40;
    Vec2<float> yxo; dummy_init(yxo);
    for_int(i, n) {
        float a = i*TAU/n;
        Vec2<float> yx = .5f+V(sin(a), cos(a))*.5f*k_globe_radius;
        if (i&0x1) HB::draw_segment(yxo, yx);
        yxo = yx;
    }
}

static void compute_subdivmode() {
    if (!g_obs[2].visible()) return;
    const int nsub = getenv_int("G3DNSUB", 2);
    bool force_recompute = false;
    if (!smesh || &smesh->orig_mesh()!=g_obs[1].get_mesh()) {
        SHOW("doing mesh refinement");
        g_obs.last = 2;
        g_obs[2].update();
        // g_obs[2].override_mesh(nullptr); // prevent clear() from deleting the mesh
        g_obs[2].clear();
        HB::clear_segment(2);
        HB::open_segment(2);
        const GMesh& mesh = *g_obs[1].get_mesh();
        // Give a rough idea of what the mesh extents will be
        for (Vertex v : mesh.vertices()) {
            g_obs[2].enter_point(mesh.point(v));
        }
        HB::close_segment();
        g_obs[2].update_stats();
        GMesh& omesh = *g_obs[1].get_mesh();
        for (Vertex v : omesh.vertices()) {
            omesh.flags(v).flag(SubMesh::vflag_variable) = true;
        }
        smesh = make_unique<SubMesh>(omesh);
        smesh->subdivide_n(nsub, 1);
        g_obs[2].override_mesh(&smesh->mesh());
        force_recompute = true;
    }
    GMesh& mesh0 = smesh->orig_mesh();
    GMesh& meshn = smesh->mesh();
    Set<Vertex> v0redo;
    for (Vertex v : mesh0.vertices()) {
        if (!mesh0.flags(v).flag(vflag_ok)) v0redo.enter(v);
    }
    if (force_recompute || v0redo.num()) meshn.gflags().flag(mflag_ok) = false;
    if (force_recompute || v0redo.num()>3) {
        SHOW("recompute all");
        for (Vertex vn : meshn.vertices()) {
            smesh->update_vertex_position(vn);
            meshn.flags(vn).flag(vflag_ok) = false;
        }
    } else {
        for (Vertex v : v0redo) {
            for (Vertex vn : meshn.vertices()) {
                const Combvh& comb = smesh->combination(vn);
                if (!comb.c[v]) continue;
                // we could be smarter and displace the vertex if we knew how much v had changed
                smesh->update_vertex_position(vn);
                meshn.flags(vn).flag(vflag_ok) = false;
            }
        }
    }
}

void Draw() {
    if (keystring!="") {
        for (char ch : keystring) { KeyPressed(string(1, ch)); }
        keystring = "";
    }
    cur_needs_redraw = false;
    if (input && !tried_input) ReadInput(false);
    tried_input = false;
    CloseIfOpen();
    if (iostat) Sipf.enter(num_input_frames);
    num_input_frames = 0;
    if (want_jump>1) DoJump();
    else want_jump = 0;
    get_time();
    cumtime += fchange;
    change_frames();
    if (play) {
        static float play_last = 0.f;
        static const float play_fps = getenv_float("G3D_PLAY_FPS", 30.f);
        if (g_obs.last>1 && cob>0 && cumtime>=play_last+1.f/play_fps) {
            play_last = cumtime;
            g_obs[cob].set_vis(false);
            static bool backwards = false;
            static const bool mirror_loop = !getenv_bool("G3D_ORDINARY_LOOP");
            cob += backwards ? -1 : +1;
            if (cob>g_obs.last) {
                if (mirror_loop) {
                    cob = g_obs.last;   // repeat the last object a second time
                    backwards = true;
                } else {
                    cob = 1;
                }
            } else if (cob<1) {
                if (mirror_loop) {
                    cob = 1;            // repeat the first object a second time
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
    if (subdivmode) compute_subdivmode();
    if (lod_mode) {
        static bool is_init = false;
        if (!is_init) { is_init = true; update_lod(); }
    }
    HB::draw_space();
    ShowInfo();
    if (sizemode) handle_sliders(true, 0.f);
    show_caption();
    show_cross();
    show_globe();
    if (g_g3d_ellipse) g3d_ellipse2();
    if (button_active || flightmode!=EFlightmode::none || g_g3d_ellipse) {
        HB::redraw_later();
        cur_needs_redraw = true;
    }
    prev_needed_redraw = cur_needs_redraw;
}

void ClearSubMesh() {
    smesh = nullptr;
    g_obs[2].override_mesh(nullptr);
}

} // namespace g3d
