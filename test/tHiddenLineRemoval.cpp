// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "HiddenLineRemoval.h"
#include "Polygon.h"
using namespace hh;

int main() {
    Polygon poly(3);
    poly[0] = Point(.2f, .2f, .2f);
    poly[1] = Point(.3f, .2f, .6f);
    poly[2] = Point(.4f, .7f, .7f);
    HiddenLineRemoval hlr;
    hlr.enter(poly);
    SHOW(hlr.draw_point(Point(.1f, .5f, .5f)));
    SHOW(hlr.draw_point(Point(.6f, .45f, .5f)));
    SHOW(hlr.draw_point(Point(.9f, .1f, .1f)));
    SHOW(hlr.draw_point(Point(.20f, .25f, .3f)));
    SHOW(hlr.draw_point(Point(.35f, .25f, .3f)));
    SHOW(hlr.draw_point(Point(.45f, .25f, .3f)));
    auto func_cbfunc = [](const Point& p1, const Point& p2) {
        showf("Segment between:\n");
        SHOW(p1);
        SHOW(p2);
    };
    hlr.set_draw_seg_cb(func_cbfunc);
    SHOW("1"); hlr.draw_segment(Point(.1f, .4f, .4f), Point(.9f, .5f, .5f));
    SHOW("2"); hlr.draw_segment(Point(.8f, .9f, .1f), Point(.9f, .25f, .7f));
}
