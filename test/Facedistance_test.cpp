// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Facedistance.h"

#include "libHh/A3dStream.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // round_elements()
using namespace hh;

namespace {

void test1() {
  // Point p1(.2f, .3f, .6f);
  // Point p2(.3f, .7f, .2f);
  // Point p3(.7f, .5f, .5f);
  for_int(j, 100) {
    Point p1;
    for_int(c, 3) p1[c] = Random::G.unif();
    Point p2;
    for_int(c, 3) p2[c] = Random::G.unif();
    Point p3;
    for_int(c, 3) p3[c] = Random::G.unif();
    for_int(i, 100) {
      Point p;
      for_int(c, 3) p[c] = Random::G.unif();
      Bary bary1, bary2, bary3;
      Point clp1, clp2, clp3;
      float d1 = project_point_triangle2(p, p1, p2, p3, bary1, clp1);
      float d2 = project_point_triangle2(p, p2, p3, p1, bary2, clp2);
      float d3 = project_point_triangle2(p, p3, p1, p2, bary3, clp3);
      float dmin = min({d1, d2, d3});
      float dmax = max({d1, d2, d3});
      if (dmax - dmin < 3e-7) continue;
      SHOW(p1, p2, p3);
      SHOW(p, dmax - dmin, dmin, dmax);
      SHOW(d1, d2, d3);
      SHOW(bary1, bary2, bary3);
      SHOW(clp1, clp2, clp3);
    }
  }
}

void test2() {
  WSA3dStream oa3d(std::cout);
  Point p1(2.f, 3.f, 9.f);
  Point p2(4.f, 7.f, 10.f);
  Point p3(6.f, 5.f, 11.f);
  {
    A3dElem el(A3dElem::EType::polygon);
    el.push(A3dVertex(p1, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::blue())));
    el.push(A3dVertex(p2, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::blue())));
    el.push(A3dVertex(p3, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::blue())));
    oa3d.write(el);
  }
  oa3d.flush();
  const float vround = 1e2f;
  for_int(i, 21) {
    Point p = interp(Point(-1.f, 3.f, 5.f), Point(10.f, 7.f, 7.f), i / 20.f);
    Bary cba;
    Point clp;
    float dis2 = project_point_triangle2(p, p1, p2, p3, cba, clp);
    SHOW("");
    round_elements(p, vround);
    round_elements(clp, vround);
    round_elements(ArView(dis2), vround);
    round_elements(cba, vround);
    SHOW(p);
    SHOW(clp);
    SHOW(dis2);
    showf("cba = (%g, %g, %g)\n", cba[0], cba[1], cba[2]);
    A3dElem el(A3dElem::EType::polyline, false, 2);
    el[0] = A3dVertex(p, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red()));
    el[1] = A3dVertex(clp, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red()));
    oa3d.write(el);
    oa3d.flush();
  }
  {
    const A3dVertexColor color(A3dColor(.1f, .2f, .3f), A3dColor(.4f, .5f, .6f), A3dColor(4.f, 0.f, 0.f));
    const A3dColor specular = color.s;
    SHOW(specular);
  }
}

}  // namespace

int main() {
  test1();
  test2();
}
