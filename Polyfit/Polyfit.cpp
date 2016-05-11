// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Args.h"
#include "A3dStream.h"
#include "FileIO.h"
#include "Bbox.h"
#include "FrameIO.h"
#include "LLS.h"
#include "Principal.h"
#include "Polygon.h"
#include "Set.h"
#include "Array.h"
#include "Map.h"
#include "Random.h"
#include "Facedistance.h"
#include "Timer.h"
using namespace hh;

namespace {

struct mvertex; using vertex = mvertex*;
struct mvertex {
    Point p;                    // position of vertex
    Vec2<vertex> v;             // { previous_vertex, next_vertex }; either may be nullptr
    Set<int> pts;               // points projecting onto associated edge (defined if v[1]!=nullptr)
};
Set<vertex> verts;

struct S_pt {
    int n;
    Array<Point> co;            // position of point
    Array<vertex> cle;          // closest edge
    Array<float> dis2;          // distance squared to closest edge
} pt;

Set<vertex> ecand;              // set of candidate edges in stoc

float spring = 0.f;
float fliter = 1;
float crep = 1e-5f;
bool nooutput = false;
int verb = 1;

WSA3dStream g_oa3d{std::cout};
Frame xform;                    // original verts+pts -> verts+pts in unit cube
Frame xformi;                   // inverse
enum EOperation { OP_ecol, OP_espl, OP_NUM };
const Vec<string,OP_NUM> opname = {"ecol", "espl"};
enum EResult { R_success, R_energy, R_illegal, R_NUM };
const Vec<string,R_NUM> orname = {"success", "positive_energy", "illegal"};
struct S_opstat {
    Vec<int,OP_NUM> na, ns;
    Vec<int,R_NUM> nor;
} opstat;

const Array<float> spring_sched = {1e-2f, 1e-3f, 1e-4f, 1e-8f};
constexpr int k_max_gfit_iter = 30;
unique_ptr<WFile> file_spawn;
unique_ptr<WSA3dStream> a3d_spawn;

float get_edis() {
    float edis = 0.f; for_int(i, pt.n) { edis += pt.dis2[i]; }
    return edis;
}

float get_espr() {
    double espr = 0.;
    for (vertex v : verts) {
        if (v->v[1]) espr += spring*dist2(v->p, v->v[1]->p);
    }
    return float(espr);
}

float get_erep() {
    return crep*verts.num();
}

void analyze_poly(int indent, const string& s) {
    string str; for_int(i, indent) str += " ";
    int nv = verts.num();
    float edis = get_edis(), espr = get_espr(), erep = get_erep();
    float etot = edis+espr+erep;
    showdf("%sPoly analysis: %s\n", str.c_str(), s.c_str());
    showdf("%s  poly: v=%d\n", str.c_str(), nv);
    showdf("%s  parameters: crep=%g spring=%g\n", str.c_str(), crep, spring);
    showdf("%s  energies: edis=%g espr=%g erep=%g etot=%g\n", str.c_str(), edis, espr, erep, etot);
}

void poly_transform(const Frame& f) {
    for (vertex v : verts) { v->p *= f; }
}

void compute_xform() {
    Bbox bb;
    bb.clear();
    for_int(i, pt.n) { bb.union_with(pt.co[i]); }
    for (vertex v : verts) { bb.union_with(v->p); }
    xform = bb.get_frame_to_small_cube();
    if (verb>=1) showdf("Applying xform: %s", FrameIO::create_string(xform, 1, 0.f).c_str());
    xformi = ~xform;
    for_int(i, pt.n) { pt.co[i] *= xform; }
    poly_transform(xform);
}

void initial_projection() {
    HH_TIMER(_initialproj);
    // do it inefficiently for now ?
    for_int(i, pt.n) {
        if (!pt.cle[i]) {
            float mind2 = BIGFLOAT;
            vertex mine = nullptr;
            for (vertex v : verts) {
                if (!v->v[1]) continue;
                float d2 = project_point_seg2(pt.co[i], v->p, v->v[1]->p);
                if (d2<mind2) { mind2 = d2; mine = v; }
            }
            pt.cle[i] = assertx(mine);
        }
        vertex v = pt.cle[i];
        pt.dis2[i] = project_point_seg2(pt.co[i], v->p, v->v[1]->p);
        v->pts.enter(i);
    }
    analyze_poly(0, "INITIAL");
}

void perhaps_initialize() {
    if (xform[0][0]) return;    // already initialized
    assertx(pt.n && verts.num());
    assertw(spring>0);          // just warn user
    compute_xform();
    initial_projection();
}

void output_poly(WSA3dStream& oa3d, bool clearobject = false) {
    perhaps_initialize();
    poly_transform(xformi);
    if (clearobject) oa3d.write_clear_object();
    A3dElem el;
    Set<vertex> setv;
    for (vertex v : verts) { setv.enter(v); }
    while (!setv.empty()) {
        vertex vf = setv.get_one(), vf0 = vf;
        HH_ASSUME(vf);
        while (vf->v[0] && vf->v[0]!=vf0) {
            vf = vf->v[0];
            HH_ASSUME(vf);
        }
        el.init(A3dElem::EType::polyline);
        for (vertex v = vf; v; v = v->v[1]) {
            el.push(A3dVertex(v->p, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::black())));
            if (v==vf && el.num()>1) break;
            assertx(setv.remove(v));
        }
        oa3d.write(el);
    }
    oa3d.os().flush();
    poly_transform(xform);
}

void enter_point(const Point& p, vertex v) {
    pt.co.push(p);
    pt.cle.push(v);
    pt.dis2.push(0.f);
    pt.n++;
}

void initialize_poly(const Polygon& poly) {
    int num = poly.num();
    bool closed = !compare(poly[0], poly[num-1], 1e-6f);
    if (closed) --num;
    assertx(num>=(closed ? 3 : 2));
    vertex vf = nullptr, vl = nullptr;
    for_int(i, num) {
        vertex v = new mvertex;
        verts.enter(v);
        v->p = poly[i];
        v->v[0] = vl;
        if (vl) vl->v[1] = v;
        vl = v;
        if (!vf) vf = v;
    }
    assertx(vf); assertx(vl);   // HH_ASSUME(vf); HH_ASSUME(vl);
    vf->v[0] = closed ? vl : nullptr;
    vl->v[1] = closed ? vf : nullptr;
    showdf("created %s poly structure with %d vertices\n", (closed ? "closed" : "open"), num);
}

void reproject_locally(int pi) {
    vertex v = assertx(pt.cle[pi]);
    assertx(v->pts.contains(pi)); // optional
    assertx(v->v[1]);             // optional
    float a, mind2 = project_point_seg2(pt.co[pi], v->p, v->v[1]->p);
    vertex mine = v;
    if (v->v[0] &&
        (a = project_point_seg2(pt.co[pi], v->v[0]->p, v->p))<mind2) { mind2 = a; mine = v->v[0]; }
    if (v->v[1]->v[1] &&
        (a = project_point_seg2(pt.co[pi], v->v[1]->p, v->v[1]->v[1]->p))<mind2) { mind2 = a; mine = v->v[1]; }
    pt.dis2[pi] = mind2;
    if (mine==v) return;
    assertx(v->pts.remove(pi));
    mine->pts.enter(pi);
    pt.cle[pi] = mine;
}

void global_project() {
    // local projection
    for_int(i, pt.n) { reproject_locally(i); }
}

void global_fit() {
    Map<vertex,int> mvi;
    Array<vertex> va;
    for (vertex v : verts) {
        mvi.enter(v, va.num());
        va.push(v);
    }
    int m = pt.n, n = verts.num();
    if (spring) {
        for (vertex v : verts) { if (v->v[1]) m++; }
    }
    if (verb>=2) showf("GlobalFit: about to solve a %dx%d LLS system\n", m, n);
    SparseLLS lls(m, n, 3);
    // Add point constraints
    for_int(i, pt.n) {
        vertex cle = assertx(pt.cle[i]);
        Vec2<vertex> v { cle, assertx(cle->v[1]) };
        float bary;
        project_point_seg2(pt.co[i], v[0]->p, v[1]->p, &bary);
        for_int(j, 2) {
            lls.enter_a_rc(i, mvi.get(v[j]), (!j ? bary : 1.f-bary));
        }
        lls.enter_b_r(i, pt.co[i]);
    }
    // Add spring constraints
    if (spring) {
        float sqrtit = sqrt(spring);
        Vector vzero(0.f, 0.f, 0.f);
        int ri = pt.n;
        for (vertex v : verts) {
            if (!v->v[1]) continue;
            lls.enter_a_rc(ri, mvi.get(v), sqrtit);
            lls.enter_a_rc(ri, mvi.get(v->v[1]), -sqrtit);
            lls.enter_b_r(ri, vzero);
            ri++;
        }
        assertx(ri==lls.num_rows());
    }
    // Suggest current solution
    for_int(i, n) { lls.enter_xest_r(i, va[i]->p); }
    // Solve
    lls.solve();
    // Update solution
    for_int(i, n) {
        Point p; lls.get_x_r(i, p);
        va[i]->p = p;
    }
}

// v[0] or v[1] may be nullptr
void local_fit(CArrayView<int> arpts, Vec2<vertex>& v, int niter, Point& newp, double& prss0, double& prss1) {
    assertx(v[0] || v[1]);
    float sqrtit = sqrt(spring);
    for_int(ni, niter) {
        Vec3<double> UtU, Utb, btb; fill(UtU, 0.); fill(Utb, 0.); fill(btb, 0.);
        double rss0 = 0., rss1 = 0.;
        // Enter projections
        for (int pi : arpts) {
            const Point& p = pt.co[pi];
            float bary0, bary1; dummy_init(bary0, bary1);
            float d0 = v[0] ? project_point_seg2(p, v[0]->p, newp, &bary0) : BIGFLOAT;
            float d1 = v[1] ? project_point_seg2(p, v[1]->p, newp, &bary1) : BIGFLOAT;
            int mini = d0<d1 ? 0 : 1; assertx(v[mini]);
            float bary = mini ? bary1 : bary0;
            double u = 1.f-bary;
            for_int(c, 3) {
                double b = p[c]-bary*v[mini]->p[c];
                UtU[c] += u*u; Utb[c] += u*b; btb[c] += b*b;
                rss0 += square(u*newp[c]-b);
            }
        }
        // Enter springs
        for_int(i, 2) {
            if (!v[i]) continue;
            for_int(c, 3) {
                double u = sqrtit, b = double(v[i]->p[c])*sqrtit;
                UtU[c] += u*u; Utb[c] += u*b; btb[c] += b*b;
                rss0 += square(u*newp[c]-b);
            }
        }
        // Solve
        for_int(c, 3) {
            double newv = assertw(UtU[c]) ? Utb[c]/UtU[c] : newp[c];
            newp[c] = float(newv);
            double a = btb[c]-UtU[c]*square(newv);
            assertw(a>-1e-8);
            if (a>0) rss1 += a;
        }
        assertw(rss1-rss0<1e-13);
        if (!ni) prss0 = rss0;
        prss1 = rss1;
    }
}

void fit_ring(vertex v, int niter) {
    assertx(v);
    Vec2<vertex> va { v->v[0], v->v[1] };
    Array<int> arpts;
    if (va[0]) for (int pi : va[0]->pts) { arpts.push(pi); }
    if (va[1]) for (int pi : v->pts    ) { arpts.push(pi); }
    double rss0, rss1; local_fit(arpts, va, niter, v->p, rss0, rss1);
    for (int pi : arpts) { reproject_locally(pi); }
}

void cleanup_neighborhood(vertex v, int nri) {
    assertx(v);
    if (!nri) return;
    vertex v0 = v->v[0], v00 = v0 && v0->v[0] ? v0->v[0] : nullptr;
    vertex v1 = v->v[1], v11 = v1 && v1->v[1] ? v1->v[1] : nullptr;
    fit_ring(v, nri);
    if (v0) fit_ring(v0, nri);
    if (v1) fit_ring(v1, nri);
    if (v00) fit_ring(v00, nri);
    if (v11) fit_ring(v11, nri);
    if (v0) fit_ring(v0, nri);
    if (v1) fit_ring(v1, nri);
    fit_ring(v, nri);
    if (v0) fit_ring(v0, nri);
    if (v1) fit_ring(v1, nri);
    fit_ring(v, nri);
}

//    v0      v      v1
//    va[0]                 va[1]
//    ev[0]   ev[1]  ev[2]
EResult try_ecol(vertex v, int ni, int nri, float& edrss) {
    vertex v0 = v->v[0], v1 = assertx(v->v[1]), ov = v0 ? v0 : v1;
    Vec2<vertex> va { v0, v1->v[1] };
    Vec3<vertex> ev { v0, v, v1 };
    // closed triangle or single open segment
    if (va[0]==va[1]) return R_illegal;
    double rssf = 0.;
    Array<int> arpts;
    for_int(i, 3) {
        if (!ev[i] || !ev[i]->v[1]) continue;
        for (int pi : ev[i]->pts) { arpts.push(pi); }
        rssf += spring*dist2(ev[i]->p, ev[i]->v[1]->p);
    }
    for (int pi : arpts) { rssf += pt.dis2[pi]; }
    // Find the best starting location by exploring one iteration.
    double minrss1 = BIGFLOAT; int minii = -1;
    for_int(ii, 3) {
        Point newp = interp(v->p, v1->p, ii*.5f);
        double rss0, rss1; local_fit(arpts, va, 1, newp, rss0, rss1);
        if (rss1<minrss1) { minrss1 = rss1; minii = ii; }
    }
    if (minii<0) assertnever("");
    // Then, explore ni iterations from that chosen starting point
    Point newp = interp(v->p, v1->p, minii*.5f);
    double rss0, rss1; local_fit(arpts, va, ni, newp, rss0, rss1);
    double drss = rss1-rssf-double(crep);
    edrss = float(drss);
    if (verb>=4) SHOW("ecol:", rssf, rss1, drss);
    if (drss>=0) return R_energy; // energy function does not decrease
    // ALL SYSTEMS GO
    // move points off to other segment and reproject later
    for (int pi : v->pts) {
        ov->pts.enter(pi);
        pt.cle[pi] = ov;
    }
    v->pts.clear();
    assertx(verts.remove(v));
    delete v; v = nullptr;
    if (v0) v0->v[1] = v1;
    v1->v[0] = v0;
    if (v0) ecand.add(v0);
    if (v1->v[1]) ecand.add(v1);
    v1->p = newp;
    cleanup_neighborhood(v1, nri);
    return R_success;
}

EResult try_espl(vertex v, int ni, int nri, float& edrss) {
    // always legal
    Vec2<vertex> va { v, assertx(v->v[1]) };
    double rssf = spring*dist2(va[0]->p, va[1]->p);
    Array<int> arpts;
    for (int pi : v->pts) { arpts.push(pi); }
    for (int pi : arpts) { rssf += pt.dis2[pi]; }
    Point newp = interp(va[0]->p, va[1]->p);
    double rss0, rss1; local_fit(arpts, va, ni, newp, rss0, rss1);
    double drss = rss1-rssf+double(crep);
    edrss = float(drss);
    if (verb>=4) SHOW("espl:", rssf, rss1, drss);
    if (drss>=0) return R_energy; // energy function does not decrease
    // ALL SYSTEMS GO
    vertex vn = new mvertex;
    verts.enter(vn);
    vn->p = newp;
    vn->v[0] = va[0];
    vn->v[1] = va[1];
    va[0]->v[1] = vn;
    va[1]->v[0] = vn;
    if (va[0]->v[0]) ecand.add(va[0]->v[0]);
    ecand.add(va[0]);
    ecand.add(vn);
    if (va[1]->v[1]) ecand.add(va[1]);
    // v->pts not cleared, let refit reproject points
    cleanup_neighborhood(vn, nri);
    return R_success;
}

EResult try_op(vertex v, EOperation op, float& edrss) {
    HH_ATIMER(__try_op);
    EResult result;
    result = (op==OP_ecol ? try_ecol(v, int(4.f*fliter+.5f), int(2.f*fliter+.5f), edrss) :
               op==OP_espl ? try_espl(v, int(3.f*fliter+.5f), int(4.f*fliter+.5f), edrss) :
               (assertnever(""), R_success));
    opstat.na[op]++;
    if (result==R_success) opstat.ns[op]++;
    opstat.nor[result]++;
    return result;
}

void create_poly(bool pclosed, int n) {
    // initialize the manifold using principal components of data
    assertx(pt.n);
    Frame frame; Vec3<float> eimag; principal_components(pt.co, frame, eimag);
    Polygon poly;
    if (!pclosed) {
        assertx(n>1);
        float stdv = 1.1f;
        // generate n points along principal line (-1, 1)*stdv
        for_int(i, n) {
            poly.push(Point(-stdv+2.f*stdv*i/(n-1.f), 0.f, 0.f)*frame);
        }
    } else {
        assertx(n>2);
        float stdv = 1.5f;
        // generate n points along ellipse at radius stdv
        for_int(i, n+1) {
            float a = float(i)/n*TAU;
            poly.push(Point(stdv*cos(a), stdv*sin(a), 0.f)*frame);
        }
    }
    initialize_poly(poly);
}

void do_pfilename(Args& args) {
    HH_TIMER(_pfilename);
    RFile is(args.get_filename());
    RSA3dStream ia3d(is());
    A3dElem el;
    Polygon poly;
    for (;;) {
        ia3d.read(el);
        if (el.type()==A3dElem::EType::endfile) break;
        if (el.type()==A3dElem::EType::comment) continue;
        if (el.type()!=A3dElem::EType::polyline) { Warning("Non-polyline input ignored"); continue; }
        el.update(A3dElem::EType::polygon);
        el.get_polygon(poly);
        initialize_poly(poly);
    }
    assertx(verts.num());
}

void do_sample(Args& args) {
    assertx(!pt.n && verts.num());
    int n = args.get_int();
    // Note: could add sample points from the polygon based on edge length.
    Array<vertex> arv; for (vertex v : verts) { if (v->v[1]) arv.push(v); }
    for_int(i, n) {
        vertex v = arv[Random::G.get_unsigned(arv.num())];
        enter_point(interp(v->p, v->v[1]->p, Random::G.unif()), v);
    }
    // Enter vertices as points
    for (vertex v : verts) {
        enter_point(v->p, v->v[1] ? v : v->v[0]);
    }
    showdf("%d points read\n", pt.n);
}

void do_filename(Args& args) {
    HH_TIMER(_filename);
    assertx(!pt.n);
    RFile is(args.get_filename());
    RSA3dStream ia3d(is());
    A3dElem el;
    for (;;) {
        ia3d.read(el);
        if (el.type()==A3dElem::EType::endfile) break;
        if (el.type()==A3dElem::EType::comment) continue;
        if (el.type()!=A3dElem::EType::point) { Warning("Non-point input ignored"); continue; }
        enter_point(el[0].p, nullptr);
    }
    showdf("%d points read\n", pt.n);
}

void do_opencurve(Args& args) {
    create_poly(false, args.get_int());
}

void do_closedcurve(Args& args) {
    create_poly(true, args.get_int());
}

void do_gfit(Args& args) {
    perhaps_initialize();
    HH_TIMER(_gfit);
    int niter = args.get_int();
    if (verb>=2) showdf("\n");
    if (verb>=1) showdf("Beginning gfit, %d iterations, spr=%g\n", niter, spring);
    float ecsc = get_edis()+get_espr(); // energy constant simplicial complex
    int i;
    for (i = 0; !niter || i<niter; ) {
        if (!niter && i>=k_max_gfit_iter) break;
        i++;
        if (verb>=3) showdf("iter %d/%d\n", i, niter);
        std::cout.flush();
        { HH_ATIMER(__lls); global_fit(); }
        { HH_ATIMER(__project); global_project(); }
        float necsc = get_edis()+get_espr();
        float echange = necsc-ecsc;
        assertw(echange<0);
        if (verb>=4) {
            analyze_poly(2, "gfit_iter");
            showdf(" change in energy=%g\n", echange);
        }
        if (a3d_spawn) output_poly(*a3d_spawn, true);
        if (!niter && echange>-1e-4f) break;
        ecsc = necsc;
    }
    if (verb>=2) showdf("Finished gfit, did %d iterations\n", i);
    if (verb>=2) analyze_poly(0, "after_gfit");
}

void do_stoc() {
    perhaps_initialize();
    HH_STIMER(_stoc);
    if (verb>=2) showdf("\n");
    if (verb>=1) showdf("Beginning stoc, spring=%g, fliter=%g\n", spring, fliter);
    fill(opstat.na, 0); fill(opstat.ns, 0);
    fill(opstat.nor, 0);
    {
        int i = 0, nbad = 0, lasti = -INT_MAX;
        for (vertex v : verts) { if (v->v[1]) ecand.enter(v); }
        while (!ecand.empty()) {
            i++;
            vertex v = ecand.remove_random(Random::G);
            assertx(v->v[1]);
            EOperation op; op = OP_ecol; // dummy_init(op);
            EResult result = R_illegal;
            float edrss; dummy_init(edrss);
            if (result!=R_success) { op = OP_ecol; result = try_op(v, op, edrss); }
            if (result!=R_success) { op = OP_espl; result = try_op(v, op, edrss); }
            if (result!=R_success) { nbad++; continue; }
            if (verb>=3 || (verb>=2 && i>=lasti+100)) {
                showdf("it %5d, %s (after %3d) [%5d/%-5d] edrss=%e\n",
                       i, opname[op].c_str(), nbad, ecand.num(), verts.num(), edrss);
                lasti = i;
            }
            if (a3d_spawn) output_poly(*a3d_spawn, true);
            nbad = 0;
        }
        if (verb>=2) showdf("it %d, last search: %d wasted attempts\n", i, nbad);
    }
    const int nat = narrow_cast<int>(sum(opstat.na));
    const int nst = narrow_cast<int>(sum(opstat.ns));
    if (verb>=2) {
        showdf("Endstoc:  (col=%d/%d, espl=%d/%d tot=%d/%d)\n",
               opstat.ns[OP_ecol], opstat.na[OP_ecol], opstat.ns[OP_espl], opstat.na[OP_espl], nst, nat);
        showdf("Result of %d attempted operations:\n", nat);
        for_int(i, R_NUM) { showdf("  %5d %s\n", opstat.nor[i], orname[i].c_str()); }
    }
    if (verb>=2) analyze_poly(0, "after_stoc");
}

void do_lfit(Args& args) {
    perhaps_initialize();
    HH_STIMER(_lfit);
    int ni = args.get_int();
    int nli = args.get_int();
    if (verb>=2) showdf("\n");
    if (verb>=1) showdf("Beginning lfit, %d iters (nli=%d), spr=%g\n", ni, nli, spring);
    for_int(i, ni) {
        for (vertex v : verts) {
            fit_ring(v, nli);
        }
        if (a3d_spawn) output_poly(*a3d_spawn, true);
    }
    if (verb>=2) showdf("Finished lfit\n");
    if (verb>=2) analyze_poly(0, "after_lfit");
}

void apply_schedule() {
    while (spring>spring_sched[0]) {
        { Args args { "2", "3" }; do_lfit(args); } // -lfit 2 3
        do_stoc();                                 // -stoc
        { Args args { "2", "3" }; do_lfit(args); } // -lfit 2 3
        spring *= .1f;
    }
    for (float spr : spring_sched) {
        spring = spr;                              // -spring f
        { Args args { "2", "3" }; do_lfit(args); } // -lfit 2 3
        do_stoc();                                 // -stoc
        { Args args { "2", "3" }; do_lfit(args); } // -lfit 2 3
    }
}

void do_reconstruct() {
    HH_TIMER(_reconstruct);
    if (!spring) spring = spring_sched[0];
    perhaps_initialize();
    { Args args { "0" }; do_gfit(args); }
    apply_schedule();
}

void do_simplify() {
    HH_TIMER(_simplify);
    if (!spring) spring = spring_sched[0];
    perhaps_initialize();
    apply_schedule();
}

void do_outpoly(Args& args) {
    HH_TIMER(_outpoly);
    WFile os(args.get_filename());
    WSA3dStream oa3d(os());
    output_poly(oa3d);
}

void do_spawn(Args& args) {
    file_spawn = make_unique<WFile>(args.get_filename());
    a3d_spawn = make_unique<WSA3dStream>((*file_spawn)());
    output_poly(*a3d_spawn);
}

} // namespace

int main(int argc, const char** argv) {
    ParseArgs args(argc, argv);
    ARGSD(pfilename,            "file.a3d : initial polygon (can be -)");
    ARGSD(sample,               "n : sample n points from polygon");
    ARGSD(filename,             "file.pts : point data (can be -)");
    ARGSD(opencurve,            "n : create initial polyline from points");
    ARGSD(closedcurve,          "n : create initial polygon from points");
    ARGSP(crep,                 "v : set constant for representation energy");
    ARGSD(reconstruct,          ": apply reconstruction schedule");
    ARGSD(simplify,             ": apply simplification schedule");
    ARGSC("",                   ":");
    ARGSP(spring,               "tension : set sprint constant");
    ARGSC("",                   ":");
    ARGSD(gfit,                 "niter : do global fit (0=until convergence)");
    ARGSD(stoc,                 ": do stochastic operations");
    ARGSD(lfit,                 "ni nli : do ni iters, each nli local fits");
    ARGSD(outpoly,              "file.a3d : output current poly to file");
    ARGSD(spawn,                "'command': send record to popen");
    ARGSF(nooutput,             ": don't print final poly on stdout");
    ARGSP(fliter,               "factor : modify # local iters done in stoc");
    ARGSP(verb,                 "i : verbosity level (1=avg, 2=more, 3=lots, 4=huge)");
    HH_TIMER(Polyfit);
    showdf("%s", args.header().c_str());
    args.parse();
    perhaps_initialize();
    analyze_poly(0, "FINAL");
    HH_TIMER_END(Polyfit);
    if (file_spawn) {
        a3d_spawn.reset();
        file_spawn.reset();
    }
    hh_clean_up();
    if (!nooutput) output_poly(g_oa3d);
    for (vertex v : verts) { delete v; }
    return 0;
}
