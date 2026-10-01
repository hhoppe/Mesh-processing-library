// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/HiddenLineRemoval.h"

#include "libHh/Polygon.h"
#include "libHh/Random.h"
using namespace hh;

namespace {

// The total length of the drawn segments, accumulated by a draw-segment callback (which cannot capture state).
double g_drawn_length = 0.;
int g_num_drawn = 0;
HiddenLineRemoval* g_hlr = nullptr;

void accumulate_segment(const Point& p1, const Point& p2) {
  g_drawn_length += dist(p1, p2);
  g_num_drawn++;
  // The interior of each drawn segment is visible.
  for (const float f : {.25f, .5f, .75f}) assertx(g_hlr->draw_point(interp(p1, p2, f)));
}

// Some polygons, viewed along the +x direction (from x == -infinity).
Array<Polygon> scene_polygons() {
  Array<Polygon> polygons;
  // An axis-aligned square in the plane x == .3.
  polygons.push(Polygon{Point(.3f, .2f, .2f), Point(.3f, .5f, .2f), Point(.3f, .5f, .5f), Point(.3f, .2f, .5f)});
  // A square behind it, partially occluded.
  polygons.push(Polygon{Point(.6f, .4f, .1f), Point(.6f, .8f, .1f), Point(.6f, .8f, .6f), Point(.6f, .4f, .6f)});
  // A tilted triangle.
  polygons.push(Polygon{Point(.4f, .1f, .6f), Point(.5f, .9f, .7f), Point(.45f, .3f, .95f)});
  return polygons;
}

// Brute-force visibility: whether some polygon intersects the ray from p in the -x direction.
bool brute_force_visible(const Point& p, CArrayView<Polygon> polygons) {
  for (const Polygon& poly : polygons) {
    const Vector nor = poly.get_normal();
    const float d = poly.get_planec(nor);
    const float x = (d - nor[1] * p[1] - nor[2] * p[2]) / nor[0];  // Intersection of the ray with the plane.
    if (x >= p[0]) continue;
    if (poly.point_inside(nor, Point(x, p[1], p[2]))) return false;
  }
  return true;
}

// Whether p is within distance eps of the boundary of some polygon (in the yz projection) or of its plane.
bool is_ambiguous(const Point& p, CArrayView<Polygon> polygons, float eps) {
  for (const Polygon& poly : polygons) {
    const Vector nor = poly.get_normal();
    if (abs(dot(p, nor) - poly.get_planec(nor)) < eps) return true;
    for_int(i, poly.num()) {
      const Point p1(0.f, poly[i][1], poly[i][2]),
          p2(0.f, poly[(i + 1) % poly.num()][1], poly[(i + 1) % poly.num()][2]);
      const Point q(0.f, p[1], p[2]);
      const Vector v = p2 - p1;
      const float t = clamp(dot(q - p1, v) / mag2(v), 0.f, 1.f);
      if (dist(q, p1 + v * t) < eps) return true;
    }
  }
  return false;
}

void test_scene() {
  const Array<Polygon> polygons = scene_polygons();
  HiddenLineRemoval hlr;
  g_hlr = &hlr;
  for (const Polygon& poly : polygons) hlr.enter(poly);
  // A degenerate polygon (with zero normal) is ignored.
  hlr.enter(Polygon{Point(0.f, 0.f, 0.f), Point(0.f, 1.f, 1.f), Point(0.f, .5f, .5f)});
  // Point visibility agrees with a brute-force ray test.
  Random random(1);
  int num_visible = 0, num_tested = 0;
  for_int(i, 2000) {
    Point p;
    for_int(c, 3) p[c] = random.unif();
    if (is_ambiguous(p, polygons, 1e-4f)) continue;
    const bool visible = hlr.draw_point(p);
    assertx(visible == brute_force_visible(p, polygons));
    num_visible += visible;
    num_tested++;
  }
  SHOW(num_tested, num_visible);
  // The drawn length of each segment agrees with a dense sampling of point visibility.
  hlr.set_draw_seg_cb(accumulate_segment);
  for_int(i, 50) {
    Point p1, p2;
    for_int(c, 3) p1[c] = random.unif();
    for_int(c, 3) p2[c] = random.unif();
    g_drawn_length = 0.;
    hlr.draw_segment(p1, p2);
    const int n = 2000;
    int nvis = 0;
    for_int(j, n) nvis += brute_force_visible(interp(p1, p2, (float(j) + .5f) / float(n)), polygons);
    const double expected_length = dist(p1, p2) * double(nvis) / double(n);
    assertx(abs(g_drawn_length - expected_length) < 2e-3);
  }
  SHOW(g_num_drawn > 50);
  // A segment entirely hidden behind the front square draws nothing.
  g_num_drawn = 0;
  hlr.draw_segment(Point(.9f, .25f, .25f), Point(.9f, .35f, .45f));
  SHOW(g_num_drawn);
  // A segment in front of all polygons is drawn whole.
  g_num_drawn = 0;
  g_drawn_length = 0.;
  hlr.draw_segment(Point(.1f, .1f, .1f), Point(.1f, .9f, .9f));
  SHOW(g_num_drawn, abs(g_drawn_length - dist(Point(.1f, .1f, .1f), Point(.1f, .9f, .9f))) < 1e-6);
  // A segment piercing the front square is drawn only in front of it.
  g_num_drawn = 0;
  g_drawn_length = 0.;
  hlr.draw_segment(Point(.1f, .35f, .35f), Point(.5f, .35f, .35f));
  SHOW(g_num_drawn, round_fraction_digits(g_drawn_length, 1e4));
  // After clear(), everything is visible.
  hlr.clear();
  assertx(hlr.draw_point(Point(.9f, .3f, .3f)));
  g_hlr = nullptr;
}

}  // namespace

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
  const auto func_cbfunc = [](const Point& p1, const Point& p2) {
    showf("Segment between:\n");
    SHOW(p1);
    SHOW(p2);
  };
  hlr.set_draw_seg_cb(func_cbfunc);
  SHOW("1");
  hlr.draw_segment(Point(.1f, .4f, .4f), Point(.9f, .5f, .5f));
  SHOW("2");
  hlr.draw_segment(Point(.8f, .9f, .1f), Point(.9f, .25f, .7f));
  test_scene();
}
