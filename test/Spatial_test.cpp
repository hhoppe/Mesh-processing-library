// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Spatial.h"

#include "Random.h"
using namespace hh;

int main() {
  {
    PointSpatial<int> sp(40);
    Vec<Point, 30> pa;
    Point p(.4f, .22f, .87621f);
    Vector v(.0065f, .0212f, -.01623f);
    for_int(i, pa.num()) {
      pa[i] = p;
      sp.enter(i, &pa[i]);
      p += v;
    }
    for (int i = 8; i < pa.num(); i += 7) {
      sp.remove(i, &pa[i]);
    }
    SpatialSearch<int> ss(&sp, Point(.7f, .2f, .8f));
    while (!ss.done()) {
      float dis2;
      int i = ss.next(&dis2);
      std::cerr << sform("Found p%-3d at dis2 %-9g  : ", i, dis2) << pa[i] << "\n";
    }
    {
      SpatialSearch<int> ss1(&sp, Point(.72f, .55f, .33f));
      for_int(i, 2) {
        float dis2;
        SHOW(ss1.next(&dis2));
        SHOW(round_fraction_digits(dis2, 1e6f));
      }
    }
  }
  {
    PointSpatial<int> sp(19);
    const int n = 1000;
    Array<Point> arpts;
    arpts.reserve(n);  // prevent reallocation
    for_int(i, n) {
      Point p;
      for_int(c, 3) p[c] = .1f + .8f * Random::G.unif();
      arpts.push(p);
      sp.enter(i, &arpts.last());
    }
    SpatialSearch<int> ss(&sp, Point(1.f / 7.f, .87f, .12f));
    float od2 = 0.f;
    int i;
    for (i = 0;; i++) {
      if (ss.done()) break;
      float d2;
      ss.next(&d2);
      assertx(d2 < 5.f && d2 >= od2);
      od2 = d2;
    }
    assertx(i == n);
  }
}
