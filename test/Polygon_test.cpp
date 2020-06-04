// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Polygon.h"

#include "libHh/GeomOp.h"
#include "libHh/RangeOp.h"  // round_elements()
using namespace hh;

int main() {
  {
    Polygon poly;
    poly.push(Point(1.f, 2.f, 3.f));
    poly.push(Point(4.f, 5.f, 6.f));
    poly.push(Point(8.f, 8.f, 10.f));
    SHOW(poly);
    poly.push(Point(7.f, 7.f, 7.f));
    SHOW(poly);
    SHOW(poly.get_normal());
  }
  {
    Polygon poly;
    poly.push(Point(0.f, 0.f, 0.f));
    poly.push(Point(1.f, 0.f, 0.f));
    poly.push(Point(0.f, 1.f, 0.f));
    Point pint;
    SHOW(poly.intersect_line(Point(.2f, .2f, 1.f), Vector(0.f, 0.f, 1.f), pint));
    SHOW(pint);
    SHOW(poly.intersect_line(Point(.7f, .7f, 1.f), Vector(0.f, 0.f, 1.f), pint));
    SHOW(pint);
    SHOW(poly.intersect_line(Point(0.f, 0.f, 1.f), Vector(.1f, .3f, -1.f), pint));
    SHOW(pint);
    SHOW(poly.intersect_line(Point(0.f, 0.f, 0.f), Vector(0.f, 0.f, 1.f), pint));
    SHOW(pint);
    SHOW(poly.intersect_line(Point(0.f, .5f, 0.f), Vector(0.f, 0.f, 1.f), pint));
    SHOW(pint);
  }
  {
    Polygon poly;
    if (0) {
      poly.push(Point(1.f, 2.f, 3.f));
      poly.push(Point(4.f, 5.f, 6.f));
      poly.push(Point(8.f, 8.f, 10.f));
    } else {
      poly.push(Point(1.f, 1.f, 1.f));
      poly.push(Point(1.f, 5.f, 2.f));
      poly.push(Point(2.f, 2.f, 7.f));
    }
    widen_triangle(poly, 1e-5f);
    for_int(i, 3) {
      round_elements(poly[i], 1e4f);
      showf("%.6f %.6f %.6f\n", poly[i][0], poly[i][1], poly[i][2]);
    }
  }
}
