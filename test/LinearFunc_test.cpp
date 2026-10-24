// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/LinearFunc.h"

using namespace hh;

int main() {
  {
    const LinearFunc lf0{};
    SHOW(lf0);
    SHOW(lf0.eval(Point(1.f, 2.f, 3.f)));
  }
  {
    // The function vanishes on the plane through the point, and is the dot product with v elsewhere.
    const LinearFunc lf(Vector(1.f, 2.f, 3.f), Point(1.f, 1.f, 1.f));
    SHOW(lf);
    SHOW(lf.eval(Point(1.f, 1.f, 1.f)));
    SHOW(lf.eval(Point(2.f, 0.f, 0.f)));
    SHOW(lf.eval(Point(0.f, 0.f, 0.f)));
    assertx(lf.eval(Point(1.f, 1.f, 1.f) + Vector(2.f, -1.f, 0.f)) == 0.f);  // Displacement orthogonal to v.
    assertx(lf.eval(Point(1.f, 1.f, 1.f) + Vector(1.f, 2.f, 3.f)) == 14.f);  // Displacement along v.
  }
  {
    // Adding functions adds their values everywhere.
    const LinearFunc lf1(Vector(1.f, 0.f, 0.f), Point(2.f, 0.f, 0.f));
    const LinearFunc lf2(Vector(0.f, 0.f, 4.f), Point(0.f, 0.f, -1.f));
    LinearFunc lf = lf1;
    lf.add(lf2);
    SHOW(lf);
    for (const Point& p : {Point(0.f, 0.f, 0.f), Point(1.f, 5.f, 2.f), Point(-3.f, 1.f, .5f)}) {
      SHOW(p, lf.eval(p));
      assertx(lf.eval(p) == lf1.eval(p) + lf2.eval(p));
    }
  }
}
