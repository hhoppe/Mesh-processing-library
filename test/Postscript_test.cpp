// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Postscript.h"
using namespace hh;

int main() {
  {
    Postscript ps(std::cout, 200, 200);
    ps.point(.2f, .1f);
    ps.point(.3f, .1f);
    ps.point(.4f, .12f);
    ps.line(.6f, .2f, .6f, .6f);
    ps.line(.6f, .2f, .8f, .2f);
    ps.line(.8f, .23f, .8f, .2f);
    ps.line(.8f, .23f, .77f, .23f);
    ps.line(-.1f, .2f, .3f, -.05f);
    ps.point(.4f, .15f);
  }
  {
    // A landscape page, with clipping of lines and points against the unit square, and changes of line width.
    Postscript ps(std::cout, 300, 200);
    ps.line(.1f, .1f, .9f, .1f);
    ps.line(.9f, .1f, .9f, .9f);    // Its first endpoint continues the polyline.
    ps.line(.5f, .5f, .9f, .9f);    // Its second endpoint continues the polyline.
    ps.line(.2f, .3f, .4f, .3f);    // A new polyline.
    ps.line(1.2f, .5f, 1.5f, .6f);  // Entirely outside, to the right.
    ps.line(.5f, -.5f, .6f, -.2f);  // Entirely outside, below.
    ps.line(-.5f, .5f, 1.5f, .5f);  // Clipped at both ends.
    ps.line(-.5f, .2f, .5f, .4f);   // Clipped at its first endpoint.
    ps.line(.5f, .5f, .6f, 2.f);    // Clipped at the top boundary, at x == .5f + .1f / 3.f.
    ps.line(.4f, -1.f, .5f, .5f);   // Clipped at the bottom boundary, at x == .5f - .1f / 3.f.
    ps.point(1.1f, .5f);            // Outside.
    ps.point(.5f, .5f);
    ps.edge_width(2.f);
    ps.line(.2f, .8f, .8f, .8f);
    ps.edge_width(2.f);  // Unchanged.
    ps.flush_write("% A comment.\n");
    ps.edge_width(1.f);
    ps.line(.2f, .7f, .8f, .7f);
  }
  {
    // A portrait page with no drawing.
    Postscript ps(std::cout, 100, 200);
  }
}
