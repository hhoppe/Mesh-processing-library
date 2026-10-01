// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/HashPoint.h"

#include "libHh/Random.h"
using namespace hh;

int main() {
  {
    HashPoint hp;
    SHOW(hp.enter(Point(1.f, 1.f, 1.f)));
    SHOW(hp.enter(Point(1.f, 1.f, 2.f)));
    SHOW(hp.enter(Point(1.f, 2.f, 1.f)));
    SHOW(hp.enter(Point(1.f, 1.f, 1.00001f)));
    SHOW(hp.enter(Point(1.f, 1.f, 2.01f)));
    SHOW(hp.enter(Point(1.f, 1.f, 2.00001f)));
    SHOW(hp.enter(Point(0.850299f, 0.453924f, 0.0222916f)));
    SHOW(hp.enter(Point(0.850299f, 0.453923f, 0.0222920f)));
  }
  {  // Coordinates of magnitude at most `small` (1e-4f by default) are all equivalent to zero.
    HashPoint hp;
    assertx(hp.enter(Point(0.f, 0.f, 0.f)) == 0);
    assertx(hp.enter(Point(5e-5f, -5e-5f, 1e-4f)) == 0);
    assertx(hp.enter(Point(-1.f, 0.f, 0.f)) == 1);
    assertx(hp.enter(Point(1.f, 0.f, 0.f)) == 2);  // The sign matters.
    assertx(hp.enter(Point(-1.000001f, 2e-5f, 0.f)) == 1);
    assertx(hp.enter(Point(0.f, 0.f, 0.f)) == 0);
  }
  {  // With nignorebits = 0 and small = 0, only identical coordinates are equivalent; even adjacent floats differ.
    HashPoint hp(0, 0.f);
    const float f = 0.5f, f_next = std::nextafter(f, 1.f);
    assertx(hp.enter(Point(f, f, f)) == 0);
    assertx(hp.enter(Point(f, f, f_next)) == 1);
    assertx(hp.enter(Point(f_next, f, f)) == 2);
    assertx(hp.enter(Point(0.f, 0.f, 0.f)) == 3);
    assertx(hp.enter(Point(f, f, f)) == 0);
  }
  {  // Random points are assigned successive indices, and slightly perturbed copies map to the same indices.
    HashPoint hp;
    Random random(1);
    const int n = 300;
    Array<Point> points(n);
    for_int(i, n) {
      for_int(c, 3) points[i][c] = random.unif() * 2.f - 1.f;
      assertx(hp.enter(points[i]) == i);
    }
    for_int(i, n) {
      Point p = points[i];
      for_int(c, 3) {
        const int nulps = int(random.get_unsigned(7)) - 3;  // Perturb each coordinate by up to 3 ulps.
        for_int(j, std::abs(nulps)) p[c] = std::nextafter(p[c], nulps > 0 ? 2.f : -2.f);
      }
      assertx(hp.enter(p) == i);
    }
    assertx(hp.enter(Point(2.f, 2.f, 2.f)) == n);
  }
  {  // A pre-pass with pre_consider() does not assign indices.
    HashPoint hp;
    hp.pre_consider(Point(1.f, 2.f, 3.f));
    hp.pre_consider(Point(1.00001f, 2.f, 3.f));
    assertx(hp.enter(Point(4.f, 5.f, 6.f)) == 0);
    assertx(hp.enter(Point(1.00001f, 2.f, 3.f)) == 1);
    assertx(hp.enter(Point(1.f, 2.00001f, 3.f)) == 1);
  }
}
