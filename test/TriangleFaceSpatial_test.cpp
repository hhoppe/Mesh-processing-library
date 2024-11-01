// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/TriangleFaceSpatial.h"

#include "libHh/Array.h"
#include "libHh/Facedistance.h"
#include "libHh/Random.h"
using namespace hh;

namespace {

void test2(int gridn) {
  const int np = 30;  // was 100
  Array<TriangleFace> trianglefaces;
  trianglefaces.reserve(np);
  for_int(i, np) {
    Vec3<Point> triangle;
    for_int(j, 3) for_int(c, 3) triangle[j][c] = .1f + .8f * Random::G.unif();
    trianglefaces.push({triangle, Face(intptr_t{i})});
  }
  TriangleFaceSpatial spatial(trianglefaces, gridn);
  const int ns = 100;
  for_int(j, ns) {
    Point p;
    for_int(c, 3) p[c] = .1f + .8f * Random::G.unif();
    SpatialSearch<TriangleFace*> ss(&spatial, p);
    const auto [ptriangleface, d2] = ss.next();
    const TriangleFace& triangleface = *ptriangleface;
    int found_i = int(reinterpret_cast<intptr_t>(triangleface.face));
    assertx(found_i == &triangleface - trianglefaces.data());
    const Vec3<Point>& triangle1 = triangleface.triangle;
    float rd2 = project_point_triangle(p, triangle1).d2;
    assertx(abs(rd2 - d2) < 1e-8f);
    float mind2 = BIGFLOAT;
    int mini = 0;
    for_int(i, np) {
      const Vec3<Point>& triangle2 = trianglefaces[i].triangle;
      float d2 = project_point_triangle(p, triangle2).d2;
      float lbd2 = square(lb_dist_point_triangle(p, triangle2));
      assertw(d2 >= lbd2 - 1e-12);
      if (d2 < mind2) {
        mind2 = d2;
        mini = i;
      }
    }
    if (found_i != mini) SHOW(j, d2, mind2, found_i, mini);
  }
}

}  // namespace

int main() {
  my_setenv("SHOW_STATS", "-2");
  Timer::set_show_times(-1);
  Face f1 = Face(intptr_t{1});
  Face f2 = Face(intptr_t{1});
  const Vec2<TriangleFace> trianglefaces =
      V(TriangleFace{V(Point(.2f, .2f, .2f), Point(.2f, .8f, .8f), Point(.2f, .8f, .2f)), f1},
        TriangleFace{V(Point(.8f, .2f, .2f), Point(.8f, .8f, .8f), Point(.8f, .8f, .2f)), f2});
  TriangleFaceSpatial spatial(trianglefaces, 10);
  {
    Point p1(.1f, .5f, .3f);
    Point p2(.9f, .5f, .3f);
    const auto result = spatial.first_along_segment(p1, p2);
    SHOW(bool(result));
    if (result) {
      const auto [triangleface, pint] = *result;
      SHOW(pint);
      SHOW(triangleface->triangle);
    }
  }
  {
    Point p1(.19f, .38f, .44f);
    Point p2(.85f, .7f, .3f);
    const auto result = spatial.first_along_segment(p1, p2);
    SHOW(bool(result));
    if (result) {
      const auto [triangleface, pint] = *result;
      SHOW(pint);
      SHOW(triangleface->triangle);
    }
  }
  {
    SpatialSearch<TriangleFace*> ss(&spatial, Point(.4f, .3f, .3f));
    for (;;) {
      if (ss.done()) break;
      const auto [ptriangleface, d2] = ss.next();
      SHOW(d2, ptriangleface->triangle);
    }
  }
  test2(5);
  test2(20);
}
