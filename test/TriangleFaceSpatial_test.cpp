// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/TriangleFaceSpatial.h"

#include "libHh/Array.h"
#include "libHh/Facedistance.h"
#include "libHh/Random.h"
using namespace hh;

namespace {

void test2(int gridn) {
  const int np = 30;  // Was 100.
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
    const auto [ptriangleface, d2] = *ss.begin();
    const TriangleFace& triangleface = *ptriangleface;
    const int found_i = int(reinterpret_cast<intptr_t>(triangleface.face));
    assertx(found_i == &triangleface - trianglefaces.data());
    const Vec3<Point>& triangle1 = triangleface.triangle;
    const float rd2 = project_point_triangle(p, triangle1).d2;
    assertx(abs(rd2 - d2) < 1e-8f);
    // Compare with a linear scan over all triangles.
    float mind2 = BIGFLOAT;
    for_int(i, np) {
      const Vec3<Point>& triangle2 = trianglefaces[i].triangle;
      const float tmp_d2 = project_point_triangle(p, triangle2).d2;
      const float lbd2 = square(lb_dist_point_triangle(p, triangle2));
      assertx(tmp_d2 >= lbd2 - 1e-12f);
      mind2 = min(mind2, tmp_d2);
    }
    assertx(abs(d2 - mind2) < 1e-8f);  // The found triangle may differ from the linear-scan one only in a tie.
  }
  // The successive search results are in order of nondecreasing distance, and visit every triangle once.
  const Point p(.5f, .5f, .5f);
  SpatialSearch<TriangleFace*> ss(&spatial, p);
  float od2 = 0.f;
  int count = 0;
  for (const auto [ptriangleface, d2] : ss) {
    assertx(d2 >= od2 && abs(d2 - project_point_triangle(p, ptriangleface->triangle).d2) < 1e-8f);
    od2 = d2;
    count++;
  }
  assertx(count == np);
  // first_along_segment() finds an intersection along each segment that intersects some triangle.
  int num_hits = 0;
  for_int(j, ns) {
    Point p1, p2;
    for_int(c, 3) p1[c] = Random::G.unif();
    for_int(c, 3) p2[c] = Random::G.unif();
    float min_t = BIGFLOAT;
    for (const TriangleFace& triangleface : trianglefaces)
      if (const auto pint = intersect_segment_with_triangle(p1, p2, triangleface.triangle))
        min_t = min(min_t, dist(p1, *pint));
    const auto result = spatial.first_along_segment(p1, p2);
    assertx(bool(result) == (min_t != BIGFLOAT));
    if (result) {
      num_hits++;
      // The intersection is the first one along the segment, as in a linear scan.
      assertx(abs(dist(p1, result->pint) - min_t) < 1e-6f);
      assertx(intersect_segment_with_triangle(p1, p2, result->triangleface->triangle));
    }
  }
  assertx(num_hits > 10 && num_hits < ns);
}

}  // namespace

int main() {
  my_setenv("SHOW_STATS", "-2");
  Timer::set_show_times(-1);
  const Face f1 = Face(intptr_t{1});
  const Face f2 = Face(intptr_t{2});
  const Vec2<TriangleFace> trianglefaces =
      V(TriangleFace{V(Point(.2f, .2f, .2f), Point(.2f, .8f, .8f), Point(.2f, .8f, .2f)), f1},
        TriangleFace{V(Point(.8f, .2f, .2f), Point(.8f, .8f, .8f), Point(.8f, .8f, .2f)), f2});
  TriangleFaceSpatial spatial(trianglefaces, 10);
  {
    const Point p1(.1f, .5f, .3f);
    const Point p2(.9f, .5f, .3f);
    const auto result = spatial.first_along_segment(p1, p2);
    SHOW(bool(result));
    if (result) {
      const auto [triangleface, pint] = *result;
      SHOW(pint);
      SHOW(triangleface->triangle);
    }
  }
  {
    const Point p1(.19f, .38f, .44f);
    const Point p2(.85f, .7f, .3f);
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
    for (const auto [ptriangleface, d2] : ss) SHOW(d2, ptriangleface->triangle);
  }
  {
    // The segment in the reverse direction first hits the other triangle.
    const auto result = spatial.first_along_segment(Point(.9f, .5f, .3f), Point(.1f, .5f, .3f));
    assertx(result && result->triangleface->face == f2);
    SHOW(result->pint);
  }
  {
    // A segment that ends before reaching the first triangle, and one that misses both triangles.
    assertx(!spatial.first_along_segment(Point(.1f, .5f, .3f), Point(.15f, .5f, .3f)));
    assertx(!spatial.first_along_segment(Point(.1f, .5f, .9f), Point(.9f, .5f, .9f)));
    // A segment that starts between the two triangles.
    const auto result = spatial.first_along_segment(Point(.5f, .5f, .3f), Point(.9f, .5f, .3f));
    assertx(result && result->triangleface->face == f2);
  }
  {
    // A large slanted triangle, which is tested early because it occupies the cells near the start of the segment, but
    // which intersects the segment only near its end, at x == .9.  The first intersection is with a small triangle
    // at x == .5, which lies only in cells farther along the segment.
    const Vec2<TriangleFace> trianglefaces2 =
        V(TriangleFace{V(Point(.02f, .5f, .61f), Point(.98f, .3f, .49f), Point(.98f, .7f, .49f)), f1},
          TriangleFace{V(Point(.5f, .45f, .45f), Point(.5f, .55f, .45f), Point(.5f, .5f, .55f)), f2});
    TriangleFaceSpatial spatial2(trianglefaces2, 10);
    const auto result = spatial2.first_along_segment(Point(.05f, .5f, .5f), Point(.95f, .5f, .5f));
    assertx(result && result->triangleface->face == f2 && abs(result->pint[0] - .5f) < 1e-6f);
  }
  test2(5);
  test2(20);
}
