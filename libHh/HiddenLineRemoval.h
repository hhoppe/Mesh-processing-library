// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Polygon.h"
#include "Kdtree.h"
#include "Array.h"
#include "Bbox.h"
#include "PArray.h"

namespace hh {

// Given a set of polygons, render a set of segments with hidden-line removal (HLR).
// Assumes orthographic projection of objects lying in unit cube along -x axis.
// (Proper tolerancing requires depths to lie in unit cube, and Kdtree requires y and z to be in unit cube.)
// To do general perspective projection, transform all elements to lie in unit cube prior to calling hlr.
class HiddenLineRemoval {
    using draw_seg_type = void (*)(const Point&, const Point&);
 public:
    HiddenLineRemoval()                                 { init(); }
    // enter polygons.
    void enter(Polygon poly)                            { enter_i(std::move(poly)); }
    // draw objects (the code is reentrant).
    bool draw_point(const Point& p)                     { return draw_point_i(p); }   // ret: is_visible
    void set_draw_seg_cb(draw_seg_type func)            { _func_draw_seg_cb = func; } // could be templated
    void draw_segment(const Point& p1, const Point& p2) { HlrSegment s(p1, p2); render_seg_kd(s, 0); }
    void clear()                                        { _polygons.clear(); _kd.clear(); }
 private:
    struct HlrPolygon {
        Polygon p;
        Vector n;
        float d;
        float tol;
        Bbox bb;
    };
    struct HlrSegment {
        HlrSegment()                            = default;
        HlrSegment(const Point& p1, const Point& p2) : p(p1, p2) { }
        Vec2<Point> p;
    };
    using KD = Kdtree<int,2>;
    draw_seg_type _func_draw_seg_cb {nullptr};
    Array<HlrPolygon> _polygons;
    KD _kd {8};
    static constexpr int k_max_intersections = 20; // max # times a polygon can split a segment
    Vec<HlrSegment,k_max_intersections> _gsa;
    Vec<HlrSegment,k_max_intersections> _gsret;
    static constexpr float k_epsilon_a = 1e-6f;
    static constexpr float k_epsilon_b = 1e-12f;
    void init() {
        _kd.allow_duplication(1.0f); // value found empirically (on knot108s.m)
    }
    void enter_i(Polygon poly) {
        Vector nor = poly.get_normal();
        if (is_zero(nor)) return;
        if (nor[0]<0) nor = -nor;
        int pn = _polygons.add(1);
        HlrPolygon& p = _polygons[pn];
        p.p = std::move(poly);
        p.n = nor;
        p.d = p.p.get_planec(p.n);
        p.tol = max(p.p.get_tolerance(p.n, p.d), float(k_epsilon_a))*1.02f; // cast to avoid reference of constexpr
        p.p.get_bbox(p.bb);
        // enter coordinates [1..2] into Kdtree<int,2>
        _kd.enter(pn, p.bb[0].tail<2>(), p.bb[1].tail<2>());
    }

    bool draw_point_i(const Point& p) {
        Vec2<float> t {p[1], p[2]};
        auto func_hlr_point_consider_poly = [&](const int& pn, ArrayView<float>, ArrayView<float>, KD::CBloc)
            -> KD::ECallbackReturn {
            const HlrPolygon& hp = _polygons[pn];
            if (p[0]<hp.bb[0][0]) return KD::ECallbackReturn::nothing; // point in front of bbox of polygon
            const Polygon& poly = hp.p;
            float d = pvdot(p, hp.n)-hp.d;
            if (d<=hp.tol) return KD::ECallbackReturn::nothing; // point in front of plane of polygon
            return point_in_polygon(p, poly) ? KD::ECallbackReturn::stop : KD::ECallbackReturn::nothing;
        };
        return !_kd.search(t, t, func_hlr_point_consider_poly);
    }
    static bool point_in_polygon(const Point& p, const Polygon& poly) {
        float py = p[1];
        float pz = p[2];
        int n = poly.num();
        const Point& pt0 = poly[n-1];
        float y0 = pt0[1]-py;
        float z0 = pt0[2]-pz;
        int nint = 0;
        float y1, z1; dummy_init(y1, z1);
        for (int i = 0; i<n; i++, y0 = y1, z0 = z1) {
            const Point& pt = poly[i];
            y1 = pt[1]-py;
            z1 = pt[2]-pz;
            if (z0>=0 && z1>=0) continue;
            if (z0<0 && z1<0) continue;
            if (y0<0 && y1<0) continue;
            if (y0>=0 && y1>=1) { nint++; continue; }
            if (y0-(y1-y0)/(z1-z0)*z0>=0) nint++;
        }
        return (nint&0x1)!=0;
    }
    void render_seg_kd(HlrSegment& s, KD::CBloc kdloc) {
        assertx(_func_draw_seg_cb);
        float bbxmax = max(s.p[0][0], s.p[1][0]);
        Vec2<Vec2<float>> bb;
        bb[0][0] = min(s.p[0][1], s.p[1][1]); bb[1][0] = max(s.p[0][1], s.p[1][1]);
        bb[0][1] = min(s.p[0][2], s.p[1][2]); bb[1][1] = max(s.p[0][2], s.p[1][2]);
        auto func_hlr_seg_consider_poly = [&](const int& pn, ArrayView<float> bb0, ArrayView<float> bb1,
                                              KD::CBloc kdloc2) -> KD::ECallbackReturn {
            if (bbxmax<_polygons[pn].bb[0][0]) return KD::ECallbackReturn::nothing;
            auto ret = handle_polygon(s, pn, kdloc2);
            if (ret==KD::ECallbackReturn::bbshrunk) {
                bbxmax = max(s.p[0][0], s.p[1][0]);
                bb0[0] = min(s.p[0][1], s.p[1][1]); bb1[0] = max(s.p[0][1], s.p[1][1]);
                bb0[1] = min(s.p[0][2], s.p[1][2]); bb1[1] = max(s.p[0][2], s.p[1][2]);
            }
            return ret;
        };
        if (!_kd.search(bb[0], bb[1], func_hlr_seg_consider_poly, kdloc))
            _func_draw_seg_cb(s.p[0], s.p[1]);
    }
    // Intersect the segment s with the segment between p0 and p1.
    // If there is an intersection point, return 1 and subdivide the original
    //   segment into 2 pieces which are returned in *s and *news,
    //  else do not touch inputs and return false.
    static bool intersect_seg_side(HlrSegment& s, const Point& p0, const Point& p1, HlrSegment& news) {
        // Code adapted from Gems III p.200 and p.500
        float x1, y1, x2, y2, x3, y3, x4, y4, Ax, Ay, Bx, By;
        x1 = s.p[0][1]; y1 = s.p[0][2];
        x2 = s.p[1][1]; y2 = s.p[1][2];
        x3 = p0[1]; y3 = p0[2];
        x4 = p1[1]; y4 = p1[2];
        Ax = x2-x1; Bx = x3-x4;
        Ay = y2-y1; By = y3-y4;
        float f = Ay*Bx-Ax*By;
        if (!f) return false;       // colinear segments
        float Cx, Cy;
        Cx = x1-x3; Cy = y1-y3;
        float d = By*Cx-Bx*Cy;
        if (f>0) {
            if (d<0 || d>f) return false;
        } else {
            if (d>0 || d<f) return false;
        }
        float e = Ax*Cy-Ay*Cx;
        if (f>0) {
            if (e<0 || e>f) return false;
        } else {
            if (e>0 || e<f) return false;
        }
        float g = d/f;
        if (g<k_epsilon_a || g>1.f-k_epsilon_a) return false;
        float x5, y5;
        Point newp(s.p[0][0]+g*(s.p[1][0]-s.p[0][0]), x5 = x1+g*Ax, y5 = y1+g*Ay);
        if (square(x1-x5)+square(y1-y5)<k_epsilon_b) return false;
        if (square(x2-x5)+square(y2-y5)<k_epsilon_b) return false;
        news.p[0] = newp;
        news.p[1] = s.p[1];
        s.p[1] = newp;
        return true;
    }
    // Takes the segment s and the polygon numbered pn.
    // Computes what parts of s (sub-segments of s) are visible along the -x direction,
    //  returning these through the structure _gsret.
    // Returns the number of segments found.  If no segments are visible, returns 0.
    int intersect_seg_poly(const HlrSegment& s, const HlrPolygon& hp) {
        int ni = 1;                 // number of segments generated by splitting
        int no = 0;                 // number in subset of ni that are visible
        _gsa[0] = s;
        const Polygon& poly = hp.p;
        int pnum = poly.num();
        int il = pnum-1;
        const Point& pt0 = poly[il];
        float y0 = pt0[1];
        float z0 = pt0[2];
        for_int(i, pnum) {
            const Point& pt = poly[i];
            float y1 = pt[1];
            float z1 = pt[2];
            float miy, may, miz, maz;
            if (y0<y1) { miy = y0; may = y1; } else { miy = y1; may = y0; }
            if (z0<z1) { miz = z0; maz = z1; } else { miz = z1; maz = z0; }
            int tni = ni;
            for_int(j, tni) {
                const Point* sp = _gsa[j].p.data();
                float sy0 = sp[0][1], sz0 = sp[0][2];
                float sy1 = sp[1][1], sz1 = sp[1][2];
                if (sy0<sy1) {
                    if (sy0>may || sy1<miy) continue;
                } else {
                    if (sy1>may || sy0<miy) continue;
                }
                if (sz0<sz1) {
                    if (sz0>maz || sz1<miz) continue;
                } else {
                    if (sz1>maz || sz0<miz) continue;
                }
                if (intersect_seg_side(_gsa[j], poly[il], poly[i], _gsa[ni]))
                    assertx(++ni<k_max_intersections);
            }
            il = i; y0 = y1; z0 = z1;
        }
        for_int(i, ni) {
            const Point* sp = _gsa[i].p.data();
            Point midp = interp(sp[0], sp[1]);
            if (!point_in_polygon(midp, poly))
                _gsret[no++] = _gsa[i];
        }
        return no;
    }
    static void orient_segment(HlrSegment& s) {
        for_int(c, 3) {
            float d = s.p[0][c]-s.p[1][c];
            if (d<0) std::swap(s.p[0], s.p[1]);
            if (d) break;
        }
    }
    // Try to join the segment set _gsret containing nsi segments with segment s to
    //   form a set of segments of size smaller than nsi+1.
    // The new set is again in _gsret, and the new number of segments is returned.
    int join_set_and_seg(HlrSegment& s, int nsi) {
        if (nsi==1) {               // only handle common case
            orient_segment(_gsret[0]);
            orient_segment(s);
            if (dist2(_gsret[0].p[0], s.p[1])<k_epsilon_b) {
                _gsret[0].p[0] = s.p[0];
                return nsi;         // (nsi==1)
            }
            if (dist2(_gsret[0].p[1], s.p[0])<k_epsilon_b) {
                _gsret[0].p[1] = s.p[1];
                return nsi;         // (nsi==1)
            }
        }
        _gsret[nsi] = s;
        return nsi+1;
    }
    // ret: nothing=no_effect, bbshrunk=continue_with_changed_seg, stop=seg_gone.
    KD::ECallbackReturn handle_polygon(HlrSegment& s, int pn, KD::CBloc kdloc) {
        const HlrPolygon& hp = _polygons[pn];
        float d0 = pvdot(s.p[0], hp.n)-hp.d;
        float d1 = pvdot(s.p[1], hp.n)-hp.d;
        float tol = hp.tol;
        if (d0<=tol && d1<=tol) return KD::ECallbackReturn::nothing;
        if (d0>0 && d1<0) {
            std::swap(s.p[0], s.p[1]);
            std::swap(d0, d1);
        }
        int ns;                     // number of segments resulting
        if (d1>tol && d0<-tol) {    // point 1 is back
            Point midp = interp(s.p[0], s.p[1], d1/(d1-d0));
            HlrSegment front_s(s.p[0], midp), back_s(midp, s.p[1]);
            ns = intersect_seg_poly(back_s, hp);
            // try to reunite set _gsret and segment front_s
            ns = join_set_and_seg(front_s, ns);
        } else {
            ns = intersect_seg_poly(s, hp);
        }
        if (!ns) return KD::ECallbackReturn::stop; // segment is gone
        s = _gsret[0];
        --ns;
        if (!ns) return KD::ECallbackReturn::bbshrunk; // single possibly modified segment
        PArray<HlrSegment,4> sa(ns);
        for_int(i, ns) { sa[i] = _gsret[i+1]; }
        for_int(i, ns) { render_seg_kd(sa[i], kdloc); }
        return KD::ECallbackReturn::bbshrunk; // non-recursion with final segment
    }
};

} // namespace hh
