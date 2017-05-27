// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "MeshSearch.h"         // PolygonFaceSpatial
#include "Random.h"
#include "Facedistance.h"
#include "Array.h"
using namespace hh;

namespace {

void test2(int pspn) {
    const int np = 30;          // was 100
    Array<PolygonFace> ar_polyface; ar_polyface.reserve(np);
    for_int(i, np) {
        Polygon poly;
        for_int(j, 3) {
            Point p; for_int(c, 3) { p[c] = .1f+.8f*Random::G.unif(); }
            poly.push(p);
        }
        ar_polyface.push(PolygonFace(std::move(poly), Face(intptr_t{i})));
    }
    PolygonFaceSpatial psp(pspn);
    for (PolygonFace& polyface : ar_polyface) { psp.enter(&polyface); }
    const int ns = 100;
    for_int(j, ns) {
        Point p; for_int(c, 3) { p[c] = .1f+.8f*Random::G.unif(); }
        SpatialSearch<PolygonFace*> ss(psp, p);
        float dis2; PolygonFace& polyface = *ss.next(&dis2);
        int found_i = int(reinterpret_cast<intptr_t>(polyface.face));
        assertx(found_i==&polyface-ar_polyface.data());
        Polygon& poly1 = polyface.poly;
        float rdis2 = dist_point_triangle2(p, poly1[0], poly1[1], poly1[2]);
        assertw(rdis2==dis2);
        float mind2 = BIGFLOAT; int mini = 0;
        for_int(i, np) {
            Polygon& poly2 = ar_polyface[i].poly;
            float d2 = dist_point_triangle2(p, poly2[0], poly2[1], poly2[2]);
            float lbd2 = square(lb_dist_point_triangle(p, poly2[0], poly2[1], poly2[2]));
            assertw(d2>=lbd2-1e-12);
            if (d2<mind2) { mind2 = d2; mini = i; }
        }
        if (found_i!=mini) SHOW(j, dis2, mind2, found_i, mini);
    }
}

} // namespace

int main() {
    Face f1 = Face(intptr_t{1});
    Face f2 = Face(intptr_t{1});
    const Vec2<PolygonFace> ar_polyface = V(
        PolygonFace(Polygon(V(Point(.2f, .2f, .2f), Point(.2f, .8f, .8f), Point(.2f, .8f, .2f))), f1 ),
        PolygonFace(Polygon(V(Point(.8f, .2f, .2f), Point(.8f, .8f, .8f), Point(.8f, .8f, .2f))), f2 )
        );
    PolygonFaceSpatial psp(10);
    for (const PolygonFace& polyface : ar_polyface) { psp.enter(&polyface); }
    {
        Point p1(.1f, .5f, .3f);
        Point p2(.9f, .5f, .3f);
        const PolygonFace* polyface; Point pint;
        bool ret = psp.first_along_segment(p1, p2, polyface, pint);
        SHOW(ret);
        if (ret) { SHOW(pint); std::cerr << polyface->poly; }
    }
    {
        Point p1(.19f, .38f, .44f);
        Point p2(.85f, .7f, .3f);
        const PolygonFace* polyface; Point pint;
        bool ret = psp.first_along_segment(p1, p2, polyface, pint);
        SHOW(ret);
        if (ret) { SHOW(pint); std::cerr << polyface->poly; }
    }
    {
        SpatialSearch<PolygonFace*> ss(psp, Point(.4f, .3f, .3f));
        for (;;) {
            if (ss.done()) break;
            float dis2; PolygonFace* polyface = ss.next(&dis2);
            std::cerr << "At dis2 " << dis2 << " found poly:\n" << polyface->poly;
        }
    }
    test2(5);
    test2(20);
}
