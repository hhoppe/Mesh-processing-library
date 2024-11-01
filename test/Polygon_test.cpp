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
    const auto test = [&](const Point p1, const Point p2) {
      auto pint = poly.intersect_line(p1, p2);
      SHOW(p1, p2, bool(pint));
      if (pint) SHOW(*pint);
    };
    test(Point(.2f, .2f, 1.f), Vector(0.f, 0.f, 1.f));
    test(Point(.7f, .7f, 1.f), Vector(0.f, 0.f, 1.f));
    test(Point(0.f, 0.f, 1.f), Vector(.1f, .3f, -1.f));
    test(Point(0.f, 0.f, 0.f), Vector(0.f, 0.f, 1.f));
    test(Point(0.f, .5f, 0.f), Vector(0.f, 0.f, 1.f));
  }
  {
    Vec3<Point> triangle;
    if (0) {
      triangle = V(Point(1.f, 2.f, 3.f), Point(4.f, 5.f, 6.f), Point(8.f, 8.f, 10.f));
    } else {
      triangle = V(Point(1.f, 1.f, 1.f), Point(1.f, 5.f, 2.f), Point(2.f, 2.f, 7.f));
    }
    triangle = widen_triangle(triangle, 1e-5f);
    for_int(i, 3) {
      round_elements(triangle[i], 1e4f);
      showf("%.6f %.6f %.6f\n", triangle[i][0], triangle[i][1], triangle[i][2]);
    }
  }
}
