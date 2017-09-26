// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Args.h"
#include "FileIO.h"
#include "A3dStream.h"
#include "FrameIO.h"
#include "Stat.h"
#include "Bbox.h"
#include "Random.h"             // for -noise
#include "Polygon.h"            // for -intersect
#include "Kdtree.h"             // for -intersect
#include "HashPoint.h"          // for -joinlines
#include "Graph.h"              // for -joinlines
#include "GraphOp.h"            // for -joinlines graph_symmetric_closure()
#include "Spatial.h"
#include "Array.h"
#include "Vec.h"
#include "MathOp.h"
using namespace hh;

namespace {

WSA3dStream oa3d{std::cout};

bool nopolygons = false;
bool nopolylines = false;
bool nopoints = false;
bool onlypoly = false;
bool tolines = false;
bool nonormals = false;
bool optnormals = false;
bool nocolor = false;
bool randcolor = false;
bool fixdegen = false;
bool gnormalize = false;
bool fixorient = false;
bool flipnormals = false;
float shownormals = 0.f;
float stretch = 0.f;
float noise = 0.f;
float offset = 0.f;
bool twosided = false;
bool triangulate = false;
bool intersect = false;
int tessellate = 0;
bool info = false;
bool box = false;
bool boxframe = false;
bool nooutput = false;
int every = 0;
int first = 0;
int split = 0;
float cusphr = 0.f;
Point cusphc;
float mindis = 0.f;
int outliern = 0;
float outlierd = 0.f;
float speedup = 0.f;
double frdelay = 0.;
double eldelay = 0.;
bool toasciit = false;          // "toascii" seems to be a reserved identifier in Win32
bool tobinary = false;
int minverts = 0;

int ndegen = 0;
bool is_restrictf = false;
Frame crestrictf;
bool is_transf = false;
Frame ctransf;
bool is_ctransfinv = false;
Frame ctransfinv;
A3dColor cdiff, cspec, cphong;
int nfixorient = 0;             // # polygons flipped due to fixorient
bool joinlines = false;
float sharpthresh = 180.f;
int smoothcurves = 0;
int ncullsphere = 0;
bool culloutside = false;
int nmindis = 0;

struct S_tri {
    int npolyb;
    int ntrib;
    int ntria;
} g_tri;

struct S_tess {
    int ntrib;
    int ntria;
} g_tess;

struct S_inter {
    Bbox bb;                          // global bounding box of all polygons
    Array<unique_ptr<Polygon>> vpoly; // (not Array<Polygon> as resizing would invalidate pointers)
    int nedges;
} g_inter;

struct S_join {
    unique_ptr<HashPoint> hp;
    Array<Point> pa;
    unique_ptr<Graph<int>> graph;
} g_join;

struct S_outlier {
    Array<Point> pa;
} g_outlier;

HH_STATNP(Slnvert);             // polyline # of vertices
HH_STATNP(Sledgel);             // polyline edge length
HH_STATNP(Slclosed);            // polyline closed
HH_STATNP(Spnvert);             // polygon # of vertices
HH_STATNP(Spedgel);             // polygon edge length
HH_STATNP(Sqdiagl);             // quad diagonal length
HH_STATNP(Sparea);              // polygon area
HH_STATNP(Splanar);             // polygon planarity (0=planar)
HH_STATNP(Sptnor);              // point, existence of normal
Bbox bbox;                      // box extent
float fsplit;                   // fsplit=split; { fsplit*=speedup; }
A3dVertexColor zero_vertexcolor;
Vec2<float> colorheight;
A3dVertexColor input_color;

A3dVertex affinely_combine(const A3dElem& el, CArrayView<float> ar_w) {
    A3dVertex vavg(Point(0.f, 0.f, 0.f), Vector(0.f, 0.f, 0.f), zero_vertexcolor);
    Vector pnor = el.pnormal();
    for_int(i, el.num()) {
        float a = ar_w[i];
        vavg.n += (is_zero(el[i].n) ? pnor : el[i].n) *a;
        vavg.p += el[i].p*a;
        vavg.c.d += el[i].c.d*a;
        vavg.c.s += el[i].c.s*a;
        vavg.c.g += el[i].c.g*a;
    }
    return vavg;
}

A3dVertex get_vertex_combination(const A3dElem& el, int i, int j, int k, int nt) {
    assertx(i>=0 && j>=0 && k>=0 && i+j+k==nt && el.num()==3);
    return affinely_combine(el, V(float(i), float(j), float(k))/float(nt));
}

void delay_frame() {
    if (!frdelay) return;
    assertx(frdelay>0);
    my_sleep(frdelay);
}

void delay_element() {
    if (!eldelay) return;
    assertx(eldelay>0.);
    my_sleep(eldelay);
}

bool is_degenerate(const A3dElem& el) {
    if (!assertw(el.num()<3)) return true;
    Vector vt(0.f, 0.f, 0.f);
    for_intL(i, 1, el.num()-1) {
        vt += cross(el[0].p, el[i].p, el[i+1].p);
    }
    float area = .5f*mag(vt);
    return !area;
}

bool out_of_bounds(const A3dElem& el) {
    for_int(i, el.num()) {
        Point p = el[i].p*crestrictf;
        for_int(c, 3) {
            if (p[c]<0.f || p[c]>1.f) return true;
        }
    }
    return false;
}

bool polygon_needs_flip(const A3dElem& el) {
    Vector va(0.f, 0.f, 0.f);
    Vector pnor = el.pnormal();
    for_int(i, el.num()) {
        va += is_zero(el[i].n) ? pnor : el[i].n;
    }
    return dot(va, pnor)<0;
}

void flip_polygon(A3dElem& el) {
    // reverse(el);
    A3dVertex v;
    for_intL(i, 1, (el.num()-1)/2+1) {
        v = el[i];
        el[i] = el[el.num()-i];
        el[el.num()-i] = v;
    }
    for_int(i, el.num()) {
        if (!is_zero(el[i].n)) el[i].n = -el[i].n;
    }
}

void compute_stats(const A3dElem& el) {
    if (el.type()==A3dElem::EType::polyline) {
        Slnvert.enter(el.num());
        for_int(i, el.num()-1) {
            Sledgel.enter(dist(el[i].p, el[i+1].p));
        }
        Slclosed.enter(el[0].p==el[el.num()-1].p);
        return;
    }
    if (el.type()==A3dElem::EType::point) {
        Sptnor.enter(!is_zero(el[0].n));
        return;
    }
    Spnvert.enter(el.num());
    for_int(i, el.num()) {
        Spedgel.enter(dist(el[i].p, el[(i+1)%el.num()].p));
    }
    if (el.num()==4) {
        Sqdiagl.enter(dist(el[0].p, el[2].p));
        Sqdiagl.enter(dist(el[1].p, el[3].p));
    }
    assertx(el.num()>=3);
    Vector vt(0.f, 0.f, 0.f);
    for_intL(i, 1, el.num()-1) {
        vt += cross(el[0].p, el[i].p, el[i+1].p);
    }
    float area = .5f*mag(vt);
    Sparea.enter(area);
    if (area) {
        assertw(vt.normalize());
        float sumd = 0.f;
        for_int(i, el.num()) {
            const Point& p = el[i].p;
            sumd += pvdot(p, vt);
        }
        float d = sumd/el.num();
        float tol = 0.f;
        for_int(i, el.num()) {
            const Point& p = el[i].p;
            tol = max(tol, pvdot(p, vt)-d);
        }
        Splanar.enter(tol/sqrt(area));
    }
}

// output element
void output_element(const A3dElem& el) {
    if (!nooutput) oa3d.write(el);
}

// split element and output statistics
void pass3(const A3dElem& el) {
    static int nelem = 0;
    if (split && nelem++>=split) {
        oa3d.write_end_frame(el.binary());
        delay_frame();
        oa3d.write_end_object(el.binary(), 1.f, 0.f);
        if (speedup) fsplit *= speedup;
        split = int(fsplit+.01f);
        nelem = 1;
    }
    if (info) compute_stats(el);
    if (box || boxframe)
        for_int(i, el.num()) { bbox.union_with(el[i].p); }
    output_element(el);
    delay_element();
}

// maybe tessellate element
void pass2(const A3dElem& el) {
    if (!tessellate || el.type()!=A3dElem::EType::polygon) { pass3(el); return; }
    if (el.num()!=3) { Warning("Cannot tessellate non-triangle"); pass3(el); return; }
    int nt = tessellate;
    g_tess.ntrib++;
    A3dElem el2(el.type(), el.binary(), 3);
    for_int(i, nt) {
        for_int(j, nt-i) {
            A3dVertex v0 = get_vertex_combination(el, nt-i-j, j, i, nt);
            A3dVertex v1 = get_vertex_combination(el, nt-i-j-1, j+1, i, nt);
            A3dVertex vn = get_vertex_combination(el, nt-i-j-1, j, i+1, nt);
            el2[0] = v0; el2[1] = v1; el2[2] = vn;
            pass3(el2);
            if (i) {
                A3dVertex vp = get_vertex_combination(el, nt-i-j, j+1, i-1, nt);
                el2[0] = v1; el2[1] = v0; el2[2] = vp;
                pass3(el2);
            }
        }
    }
    g_tess.ntria += nt*nt;
}

// maybe triangulate element
void pass1(const A3dElem& el) {
    if (!triangulate || el.type()!=A3dElem::EType::polygon) { pass2(el); return; }
    g_tri.npolyb++;
    if (el.num()==3) { g_tri.ntrib++; g_tri.ntria++; pass2(el); return; }
    A3dElem el2(el.type(), el.binary(), 3);
    if (el.num()==4) {
        int i;
        for (i = 1; i<el.num(); i++) {
            if (compare(el[i].n, el[0].n, 1e-6f) || el[i].c.d!=el[0].c.d ||
                compare(el[i].c.s, el[0].c.s) || el[i].c.g!=el[0].c.g) break;
        }
        if (i==el.num()) {      // they all match
            for_int(j, 3) { el2[j] = el[j]; }
            pass2(el2);
            for_int(j, 3) { el2[j] = el[(j+2)%4]; }
            pass2(el2);
            g_tri.ntria += 2;
            return;
        }
    }
    Array<float> ar_w; ar_w.init(el.num(), 1.f/el.num());
    A3dVertex vavg = affinely_combine(el, ar_w);
    for_int(i, el.num()) {
        el2[0] = el[i];
        el2[1] = el[(i+1)%el.num()];
        el2[2] = vavg;
        pass2(el2);
    }
    g_tri.ntria += el.num();
}

void show_normals(const A3dElem& el) {
    double c = 0.f;
    if (el.num()<2) {
        c = 1.;
    } else {
        Array<Point> pa; for_int(i, el.num()) { pa.push(el[i].p); } // el not necessarily Type::polygon
        Point pavg = centroid(pa);
        for_int(i, el.num()) { c += dist(pa[i], pavg); }
        c /= el.num();
    }
    A3dElem el2(A3dElem::EType::polyline, el.binary(), 2);
    Vector pnor(3.f, 0.f, 0.f);
    for_int(i, el.num()) {
        Vector nor = el[i].n;
        if (is_zero(nor)) {
            if (el.type()!=A3dElem::EType::polygon) continue;
            if (pnor[0]>2) pnor = el.pnormal();
            nor = pnor;
        }
        el2[0] = el2[1] = el[i];
        el2[0].n = el2[1].n = nor;
        el2[1].p += nor*float(c)*.2f*shownormals;
        el2[0].c = el2[1].c = A3dVertexColor(Pixel::red());
        pass1(el2);
    }
}

bool compute_mindis(const Point& p) {
    static int pn = 1;
    static unique_ptr<PointSpatial<int>> SPp;
    if (!SPp) SPp = make_unique<PointSpatial<int>>(30);
    SpatialSearch<int> ss(SPp.get(), p, mindis);  // look no farther than mindis
    if (!ss.done()) {
        float dis2; ss.next(&dis2);
        if (dis2<square(mindis)) return true;
    }
    SPp->enter(pn++, new Point(p)); // never deleted
    return false;
}

// process element
bool loop(A3dElem& el) {
    if (el.type()==A3dElem::EType::endfile) return true;
    bool polyg = el.type()==A3dElem::EType::polygon;
    bool polyl = el.type()==A3dElem::EType::polyline;
    bool point = el.type()==A3dElem::EType::point;
    if (!polyg && !polyl && !point) {
        if (!onlypoly) output_element(el);
        if (el.type()==A3dElem::EType::endframe) delay_frame();
        return false;
    }
    if (joinlines && polyl) {
        input_color = el[0].c;
        Array<int> ar_vi;
        for_int(i, el.num()) {
            int vi = g_join.hp->enter(el[i].p);
            assertx(vi<=g_join.pa.num());
            if (vi==g_join.pa.num()) {
                g_join.pa.push(el[i].p);
                g_join.graph->enter(vi);
            }
            ar_vi.push(vi);
        }
        for_int(i, el.num()-1) {
            if (ar_vi[i]==ar_vi[i+1]) {
                Warning("zero hashed line segment ignored");
                if (0) SHOW(i, el.num(), el[i+0].p, el[i+1].p);
                continue;
            }
            if (g_join.graph->contains(ar_vi[i], ar_vi[i+1])) { Warning("skipping duplicate line"); continue; }
            g_join.graph->enter(ar_vi[i], ar_vi[i+1]);
        }
        return false;
    }
    if (outliern && point) {
        g_outlier.pa.push(el[0].p);
        return false;
    }
    if (polyg && nopolygons) return false;
    if (polyl && nopolylines) return false;
    if (point && nopoints) return false;
    if ((polyg || polyl) && el.num()<minverts) return false;
    if (polyg && fixdegen && is_degenerate(el)) { ndegen++; return false; }
    if (is_restrictf && out_of_bounds(el)) return false;
    static int nevery = 0;
    nevery++;
    if (every>1 && nevery%every!=1) return false;
    if (first && nevery>first) return true;
    if (cusphr && point && ((dist2(el[0].p, cusphc)<=square(cusphr))^culloutside)) {
        ncullsphere++;
        return false;
    }
    if (mindis && point && compute_mindis(el[0].p)) {
        nmindis++;
        return false;
    }
    Vector pnor(3.f, 0.f, 0.f);
    if (optnormals && polyg) pnor = el.pnormal();
    if (smoothcurves && polyl) {
        bool closed = el[0].p==el[el.num()-1].p;
        Array<bool> ar_sharp;
        Array<Point> ar_p(el.num());
        for_int(iter, smoothcurves) {
            bool even = iter%2==0 || 1; // why 1?
            ar_sharp.init(0);
            for_int(i, el.num()) {
                if (!closed && (i==0 || i==el.num()-1)) { ar_sharp.push(true); continue; }
                int i0 = i-1; if (i0<0) i0 = el.num()-2;
                int i1 = i+1; if (i1==el.num()) i1 = 1;
                Vector n1 = el[i0].p-el[i].p;
                Vector n2 = el[i].p-el[i1].p;
                ar_sharp.push((n1.normalize() && n2.normalize() &&
                               angle_between_unit_vectors(normalized(el[i0].p-el[i].p),
                                                          normalized(el[i].p-el[i1].p))>to_rad(sharpthresh)));
            }
            for_int(i, el.num()) {
                if (ar_sharp[i]) { ar_p[i] = el[i].p; continue; }
                int i0 = i-1; if (i0<0) i0 = el.num()-2;
                int i1 = i+1; if (i1==el.num()) i1 = 1;
                Vector disp = interp(el[i0].p, el[i1].p)-el[i].p;
                float fac = even ? .65f : -.65f;
                ar_p[i] = el[i].p+disp*fac;
            }
            for_int(i, el.num()) { el[i].p = ar_p[i]; }
            // unnecessary: if (closed) el[el.num()-1].p = el[0].p;
        }
    }
    for_int(i, el.num()) {
        if (nonormals || (optnormals && !compare(el[i].n, pnor, 1e-6f)))
            el[i].n = Vector(0.f, 0.f, 0.f);
        if (nocolor) el[i].c = zero_vertexcolor;
        bool validcol = false;
        if (cdiff[0]>=0) { el[i].c.d = cdiff; validcol = true; }
        if (cspec[0]>=0) { el[i].c.s = cspec; validcol = true; }
        if (validcol && !el[i].c.g[0]) el[i].c.g[0] = 1.f;
        if (cphong[0]>=0) el[i].c.g = cphong;
        if (colorheight[1]!=colorheight[0]) {
            float c = (el[i].p[2]-colorheight[0])/(colorheight[1]-colorheight[0]);
            c = clamp(c, 0.f, 1.f);
            el[i].c.d = A3dColor(c, c, c);
//             // from lum.h in ~4Dgifts/iristools
//             const float RLUM = .3086f, GLUM = .6094f, BLUM = .0820f;
//             el[i].c.d = A3dColor(min(1.f, max(0.f, c/RLUM/3)),
//                                  min(1.f, max(0.f, c/RLUM/3)),
//                                  min(1.f, max(0.f, c/RLUM/3)));
        }
        if (is_transf) {
            el[i].p *= ctransf;
            if (!is_ctransfinv) {
                el[i].n = Vector(0.f, 0.f, 0.f);
            } else if (!is_zero(el[i].n)) {
                el[i].n = ctransfinv*el[i].n;
                assertw(el[i].n.normalize());
            }
        }
        if (gnormalize && !is_zero(el[i].n))
            assertx(el[i].n.normalize());
    }
    if (fixorient && polyg && polygon_needs_flip(el)) { flip_polygon(el); nfixorient++; }
    if (flipnormals) flip_polygon(el);
    if (stretch && polyl && el.num()==2) {
        if (stretch>0) {
            el[1].p += (el[1].p-el[0].p)*stretch;
        } else {
            el[0].p += (el[0].p-el[1].p)*stretch;
        }
    }
    if (shownormals) show_normals(el);
    for_int(i, el.num()) {
        if (offset) {
            Vector nor = el[i].n;
            if (is_zero(nor) && polyg) {
                if (pnor[0]>2) pnor = el.pnormal();
                nor = pnor;
            }
            if (!is_zero(nor)) el[i].p += nor*offset;
        }
        if (noise) for_int(c, 3) { el[i].p[c] += Random::G.gauss()*noise; }
    }
    if (randcolor) {
        A3dColor col; for_int(c, 3) { col[c] = Random::G.unif(); }
        for_int(i, el.num()) { el[i].c.d = col; }
    }
    if (intersect && polyg) {
        auto npoly = make_unique<Polygon>();
        el.get_polygon(*npoly);
        Bbox bb; npoly->get_bbox(bb);
        g_inter.bb.union_with(bb);
        g_inter.vpoly.push(std::move(npoly));
        return false;           // new: only output intersection edges
    }
    if (tolines && polyg) {
        A3dElem el2(A3dElem::EType::polyline, el.binary(), 2);
        for_int(i, el.num()) {
            el2[0] = el[i];
            el2[1] = el[(i+1)%el.num()];
            pass1(el2);
        }
        return false;
    }
    if (twosided && polyg) { pass1(el); flip_polygon(el); }
    pass1(el);
    return false;
}

using KD = Kdtree<Polygon*, 3>;

void compute_intersect() {
    // e.g.:  Filtermesh ~/data/mesh/peedy.orig.m -toa | Filtera3d -inter | G3d ~/data/mesh/peedy.orig.m -input -key NN
    if (!g_inter.vpoly.num()) return;
    Frame f = g_inter.bb.get_frame_to_cube();
    KD kd(8);
    A3dElem el;
    Array<Point> pa;
    for (auto& ppoly : g_inter.vpoly) {
        Polygon& poly = *ppoly;
        Bbox bb; poly.get_bbox(bb);
        bb[0] *= f;
        bb[1] *= f;
        auto func_considerpoly = [&](Polygon*const& id, ArrayView<float>,
                                     ArrayView<float>, KD::CBloc) -> KD::ECallbackReturn {
            const Polygon& p1 = *id;
            const Polygon& p2 = poly;
            pa = intersect_poly_poly(p1, p2);
            if (!pa.num()) return KD::ECallbackReturn::nothing;
            el.init(A3dElem::EType::polyline, false, 2);
            for_int(i, pa.num()/2) {
                el[0] = A3dVertex(pa[i*2+0], Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red()));
                el[1] = A3dVertex(pa[i*2+1], Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red()));
                output_element(el);
                g_inter.nedges++;
            }
            return KD::ECallbackReturn::nothing;
        };
        kd.search(bb[0], bb[1], func_considerpoly);
        kd.enter(&poly, bb[0], bb[1]);
    }
    g_inter.vpoly.clear();
}

void join_lines() {
    const bool directed_joined_lines = !getenv_bool("UNDIRECTED_LINES");
    joinlines = false;          // note that loop() is called below!
    A3dElem el;
    Graph<int>& graph = *g_join.graph;
    Set<int> candv;             // candidate vertices
    if (!directed_joined_lines) {
        // for undirected search, candidate vertices are ones with odd degree
        graph_symmetric_closure(graph);
        for (int v : graph.vertices()) {
            assertx(graph.out_degree(v)>0);
            if (graph.out_degree(v)%2) candv.enter(v);
        }
        for (;;) {
            int vi;
            if (!candv.empty()) {
                vi = candv.remove_one();
            } else {
                vi = -1;
                for (int v : graph.vertices()) { vi = v; if (1) break; }
                if (vi<0) break;
                candv.enter(vi);
            }
            el.init(A3dElem::EType::polyline);
            for (;;) {
                el.push(A3dVertex(g_join.pa[vi], Vector(0.f, 0.f, 0.f), input_color));
                int vn = -1;
                for (int v : graph.edges(vi)) { vn = v; if (1) break; }
                if (vn<0) break;
                assertx(graph.remove_undirected(vi, vn));
                if (!graph.out_degree(vi)) assertx(graph.remove(vi));
                vi = vn;
            }
            assertx(el.num()>=2);
            assertx(graph.remove(vi));
            assertx(candv.remove(vi));
            loop(el);
        }
    } else {
        // for directed search, candidate vertices are ones with no in_edges
        Graph<int> opp_graph;
        for (int v : graph.vertices()) { opp_graph.enter(v); }
        for (int v1 : graph.vertices()) {
            for (int v2 : graph.edges(v1)) { opp_graph.enter(v2, v1); }
        }
        for (int v : opp_graph.vertices()) {
            if (graph.out_degree(v)>opp_graph.out_degree(v)) candv.enter(v);
        }
        for (;;) {
            int vi;
            if (!candv.empty()) {
                vi = candv.get_one();
                if (graph.out_degree(vi)==1) assertx(candv.remove(vi));
            } else {
                vi = -1;
                for (int v : graph.vertices()) { vi = v; if (1) break; }
                if (vi<0) break;
            }
            el.init(A3dElem::EType::polyline);
            for (;;) {
                el.push(A3dVertex(g_join.pa[vi], Vector(0.f, 0.f, 0.f), input_color));
                int vn = -1;
                for (int v : graph.edges(vi)) { vn = v; if (1) break; }
                if (vn<0) break;
                assertx(graph.remove(vi, vn));
                assertx(opp_graph.remove(vn, vi));
                if (!graph.out_degree(vi) && !opp_graph.out_degree(vi)) {
                    assertx(graph.remove(vi));
                    assertx(opp_graph.remove(vi));
                    assertx(!candv.contains(vi));
                }
                vi = vn;
            }
            assertx(el.num()>=2);
            if (!graph.out_degree(vi) && !opp_graph.out_degree(vi)) {
                assertx(graph.remove(vi));
                assertx(opp_graph.remove(vi));
                assertx(!candv.contains(vi));
            }
            loop(el);
        }
    }
}

void compute_outlier() {
    Array<bool> ar_is_outlier;
    Bbox bb; bb.clear();
    for_int(i, g_outlier.pa.num()) {
        ar_is_outlier.push(false);
        bb.union_with(g_outlier.pa[i]);
    }
    Frame xform = bb.get_frame_to_cube(), xformi = ~xform;
    PointSpatial<int> SPp(30);
    for_int(i, g_outlier.pa.num()) {
        g_outlier.pa[i] *= xform;
        SPp.enter(i, &g_outlier.pa[i]);
    }
    int num_outliers = 0;
    for_int(i, g_outlier.pa.num()) {
        SpatialSearch<int> ss(&SPp, g_outlier.pa[i]);
        float dis2; dummy_init(dis2);
        for_int(j, outliern+1) { // +1 to include this point
            assertx(!ss.done());
            ss.next(&dis2);
        }
        float dis = my_sqrt(dis2)*xformi[0][0];
        HH_SSTAT(Soutlierd, dis);
        if (dis>=outlierd) { ar_is_outlier[i] = true; num_outliers++; }
    }
    showdf("found %d/%d outliers\n", num_outliers, g_outlier.pa.num());
    outliern = 0;            // note that loop() is called below!
    A3dElem el;
    for_int(i, g_outlier.pa.num()) {
        if (ar_is_outlier[i]) continue;
        el.init(A3dElem::EType::point);
        el.push(A3dVertex(g_outlier.pa[i]*~xform, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red())));
        loop(el);
    }
}

void process(RSA3dStream& ia3d) {
    A3dElem el;
    for (;;) {
        ia3d.read(el);
        if (loop(el)) break;
    }
    if (outliern) compute_outlier();
    if (joinlines) join_lines();
    if (triangulate)
        showdf("triangulation: %d polyg (%d triang) -> %d triang\n", g_tri.npolyb, g_tri.ntrib, g_tri.ntria);
    if (tessellate)
        showdf("tessellate: %d triang -> %d triang\n", g_tess.ntrib, g_tess.ntria);
    if (intersect) {
        compute_intersect();
        showdf("intersect: added %d edges\n", g_inter.nedges);
    }
    if (info) {
        showdf("Polygons\n");
        showdf(" %s", Spnvert.name_string().c_str());
        showdf(" %s", Spedgel.name_string().c_str());
        showdf(" %s", Sqdiagl.name_string().c_str());
        showdf(" %s", Sparea.name_string().c_str());
        showdf(" tot_area: %g\n", Sparea.sum());
        showdf(" %s", Splanar.name_string().c_str());
        if (Splanar.num()<Spnvert.num())
            showdf("  some zero area polygons not counted!\n");
        showdf("Polylines\n");
        showdf(" %s", Slnvert.name_string().c_str());
        showdf(" %s", Sledgel.name_string().c_str());
        showdf(" %s", Slclosed.name_string().c_str());
        showdf("Points:\n");
        showdf(" %s", Sptnor.name_string().c_str());
    }
    if (box) {
        showf("%g %g %g\n", bbox[0][0], bbox[0][1], bbox[0][2]);
        showf("%g %g %g\n", bbox[1][0], bbox[1][1], bbox[1][2]);
    }
    if (boxframe) FrameIO::write(std::cout, bbox.get_frame_to_cube(), 0, 0.f, false);
    if (ncullsphere) showdf("ncullsphere=%d\n", ncullsphere);
    if (nmindis) showdf("nmindis=%d\n", nmindis);
    if (nfixorient) showdf("nfixorient=%d\n", nfixorient);
    if (ndegen) showdf("ndegen=%d\n", ndegen);
}

} // namespace

int main(int argc, const char** argv) {
    Vec3<float> diff = { -1.f, 0.f, 0.f };
    Vec3<float> spec = { -1.f, 0.f, 0.f };
    Vec3<float> phong = { -1.f, 0.f, 0.f };
    Vec4<float> cullsphere = { 0.f, 0.f, 0.f, 0.f };
    Vec2<float> outlier = { 0.f, 0.f };
    string restrictf;
    string transf;
    bool stat = false;
    ParseArgs args(argc, argv);
    ARGSF(onlypoly,             ": remove special a3d commands");
    ARGSF(nopolygons,           ": cull polygons");
    ARGSF(nopolylines,          ": cull polylines");
    ARGSF(nopoints,             ": cull points");
    ARGSF(tolines,              ": convert polygons to lines");
    ARGSF(fixdegen,             ": remove zero area triangles");
    ARGSP(restrictf,            "'frame' : cull elems outside unit frame");
    ARGSP(every,                "i : use only every ith element");
    ARGSP(first,                "i : use only first i elements");
    ARGSF(joinlines,            ": join line segments into polyline");
    ARGSP(sharpthresh,          "deg : corners in smoothcurves");
    ARGSP(smoothcurves,         "iter : smooth the polylines");
    ARGSP(minverts,             "nv : only keep elements with >=nv verts");
    ARGSC("",                   ":**");
    ARGSF(culloutside ,         ": set to remove points outside");
    ARGSP(cullsphere,           "x y z r : remove points within sphere");
    ARGSP(mindis,               "f : make no pair of points closer than f");
    ARGSP(outlier,              "n d : remove points if n'th closest >d");
    ARGSC("",                   ":**");
    ARGSF(nonormals,            ": remove vertex normals");
    ARGSF(optnormals,           ": remove unnecessary polygon normals");
    ARGSF(nocolor,              ": remove color information");
    ARGSP(diff,                 "r g b : set diffuse color");
    ARGSP(spec,                 "r g b : set specular color");
    ARGSP(phong,                "r g b : set phong color");
    ARGSP(colorheight,          "minz maxz : from black to white");
    ARGSF(randcolor,            ": assign each element a random color");
    ARGSP(transf,               "'frame' : transform all elements");
    args.f("-normalize", gnormalize, ": normalize normals");
    ARGSF(fixorient,            ": vertex normals -> orient polygon");
    ARGSF(flipnormals,          ": flip orientations of normals and polygons");
    ARGSP(stretch,              "factor : stretch polylines");
    ARGSP(shownormals,          "fsize : print normals as small segments");
    ARGSP(offset,               "factor : move vertices along their normals");
    ARGSP(noise,                "sdv : add Gaussian noise to vertices");
    ARGSF(intersect,            ": intersect polygons to produce lines");
    ARGSF(twosided,             ": make polygons two-sided");
    ARGSF(triangulate,          ": triangulate all faces with >3 vertices");
    ARGSP(tessellate,           "n : subdivide each triangle into n*n faces");
    ARGSC("",                   ":**");
    ARGSF(info,                 ": print statistics");
    ARGSF(stat,                 ": print statistics");
    ARGSF(box,                  ": show bounding box");
    ARGSF(boxframe,             ": output frame that will box data");
    ARGSF(nooutput,             ": turn off a3d output");
    ARGSC("",                   ":**");
    ARGSP(split,                "i : output frame every ith element");
    ARGSP(speedup,              "factor : increase 'split' every frame");
    ARGSP(frdelay,              "fsec : pause after each frame");
    ARGSP(eldelay,              "fsec : pause after each element");
    ARGSF(toasciit,             ": make output be ascii text");
    ARGSF(tobinary,             ": make output be binary");
    string arg0 = args.num() ? args.peek_string() : "";
    string filename = "-"; if (args.num() && (arg0=="-" || arg0[0]!='-')) filename = args.get_filename();
    RFile is(filename);
    RSA3dStream ia3d(is());
    args.parse();
    if (restrictf!="") {
        is_restrictf = true;
        crestrictf = FrameIO::parse_frame(restrictf);
    }
    cusphr = cullsphere[3];
    if (cusphr)
        cusphc = Point(cullsphere[0], cullsphere[1], cullsphere[2]);
    if (outlier[0]) {
        outliern = int(outlier[0]);
        outlierd = outlier[1];
    }
    if (transf!="") {
        is_transf = true;
        ctransf = FrameIO::parse_frame(transf);
        is_ctransfinv = invert(ctransf, ctransfinv);
        if (!is_ctransfinv)
            showdf("Warning: uninvertible frame, normals lost\n");
    }
    if (joinlines) {
        g_join.hp = make_unique<HashPoint>();
        g_join.graph = make_unique<Graph<int>>();
    }
    for_int(c, 3) {
        cdiff[c] = diff[c]; cspec[c] = spec[c]; cphong[c] = phong[c];
    }
    if (stat) info = 1;
    if (stat || box || boxframe) nooutput = true;
    bbox.clear();
    g_inter.bb.clear();
    fsplit = float(split);
    assertx(!(toasciit && tobinary));
    if (toasciit) my_setenv("A3D_BINARY", "0");
    if (tobinary) my_setenv("A3D_BINARY", "1");
    process(ia3d);
    hh_clean_up();
    return 0;
}
