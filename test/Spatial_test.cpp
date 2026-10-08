// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Spatial.h"

#include "libHh/Random.h"
#include "libHh/RangeOp.h"
using namespace hh;

namespace {

// Verify that a complete search from query visits each of the points exactly once, in order of increasing distance,
// and with correct distances.
void verify_complete_search(const Spatial& sp, const Point& query, CArrayView<Point> points) {
  SpatialSearch<int> ss(&sp, query);
  Array<bool> visited(points.num(), false);
  float od2 = 0.f;
  int count = 0;
  for (const auto [i, d2] : ss) {
    assertx(i >= 0 && i < points.num() && !visited[i]);
    visited[i] = true;
    assertx(abs(d2 - dist2(query, points[i])) <= 1e-6f);
    assertx(d2 >= od2);
    od2 = d2;
    count++;
  }
  assertx(count == points.num());
}

// Verify that the first k results of a search match the k nearest points found by a linear scan.
void verify_nearest(const Spatial& sp, const Point& query, CArrayView<Point> points, int k) {
  SpatialSearch<int> ss(&sp, query);
  Array<float> expected;
  for (const Point& p : points) expected.push(dist2(query, p));
  sort(expected);
  int count = 0;
  for (const auto [i, d2] : ss | views::take(k)) {
    assertx(abs(d2 - expected[count]) <= 1e-6f);
    assertx(abs(dist2(query, points[i]) - expected[count]) <= 1e-6f);
    count++;
  }
  assertx(count == min(k, points.num()));
}

// Axis-aligned boxes, for testing ObjectSpatial; the object id is the box index plus one, as id 0 is disallowed.
Array<Bbox<float, 3>> g_boxes;

float box_dist2(const Point& p, const Bbox<float, 3>& box) {
  float d2 = 0.f;
  for_int(c, 3) d2 += square(max(box[0][c] - p[c], 0.f) + max(p[c] - box[1][c], 0.f));
  return d2;
}

// The squared distance to the bounding sphere of the box, which is a lower bound on the squared distance to the box.
struct BoxApprox2 {
  float operator()(const Point& p, Univ id) const {
    const Bbox<float, 3>& box = g_boxes[Conv<int>::d(id) - 1];
    const float radius = dist(box[0], box[1]) * .5f;
    return square(max(dist(p, interp(box[0], box[1])) - radius, 0.f)) * .999f;
  }
};

struct BoxExact2 {
  float operator()(const Point& p, Univ id) const { return box_dist2(p, g_boxes[Conv<int>::d(id) - 1]); }
};

}  // namespace

int main() {
  my_setenv("SHOW_STATS", "-2");
  {
    PointSpatial<int> sp(40);
    Vec<Point, 30> pa;
    Point p(.4f, .22f, .87621f);
    const Vector v(.0065f, .0212f, -.01623f);
    for_int(i, pa.num()) {
      pa[i] = p;
      sp.enter(i, &pa[i]);
      p += v;
    }
    for (int i = 8; i < pa.num(); i += 7) sp.remove(i, &pa[i]);
    SpatialSearch<int> ss(&sp, Point(.7f, .2f, .8f));
    for (const auto [i, d2] : ss) std::cerr << sform("Found p%-3d at d2=%-9g  : ", i, d2) << pa[i] << "\n";
    {
      SpatialSearch<int> ss1(&sp, Point(.72f, .55f, .33f));
      for (const auto [i2, d2] : ss1 | views::take(2)) {
        SHOW(i2);
        SHOW(round_fraction_digits(d2, 1e6f));
      }
    }
  }
  {
    PointSpatial<int> sp(19);
    const int n = 1000;
    Array<Point> arpts;
    arpts.reserve(n);  // Prevent reallocation.
    for_int(i, n) {
      Point p;
      for_int(c, 3) p[c] = .1f + .8f * Random::G.unif();
      arpts.push(p);
      sp.enter(i, &arpts.last());
    }
    // A complete search returns each point once, in order of increasing distance.
    verify_complete_search(sp, Point(1.f / 7.f, .87f, .12f), arpts);
    // The nearest neighbors match a linear scan, for queries inside and outside the region of the points.
    Random random(1);
    for_int(iquery, 40) {
      Point query;
      for_int(c, 3) query[c] = random.unif();
      verify_nearest(sp, query, arpts, 10);
    }
    // After removing every other point, the search finds only the remaining points, in order.
    for (int i = 1; i < n; i += 2) sp.remove(i, &arpts[i]);
    sp.shrink_to_fit();
    const Point query(.5f, .5f, .5f);
    Array<float> expected;
    for (int i = 0; i < n; i += 2) expected.push(dist2(query, arpts[i]));
    sort(expected);
    int count = 0;
    for (const auto [i, d2] : SpatialSearch<int>(&sp, query)) {
      assertx(i % 2 == 0 && abs(d2 - dist2(query, arpts[i])) <= 1e-6f);
      assertx(abs(d2 - expected[count]) <= 1e-6f);
      count++;
    }
    assertx(count == n / 2);
    // After clear(), a search finds nothing.
    sp.clear();
    const SpatialSearch<int> ss(&sp, query);
    assertx(ss.empty() && ranges::empty(ss));
  }
  {
    // An empty PointSpatial.
    PointSpatial<int> sp(10);
    const SpatialSearch<int> ss(&sp, Point(.5f, .5f, .5f));
    assertx(ss.empty());
  }
  {
    // Coincident points and points on the boundary of the unit cube.
    PointSpatial<int> sp(8);
    const Array<Point> pa{Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f), Point(.5f, .5f, .5f), Point(.5f, .5f, .5f),
                          Point(1.f, 0.f, 1.f)};
    for_int(i, pa.num()) sp.enter(i, &pa[i]);
    Array<int> found;
    SpatialSearch<int> ss(&sp, Point(.5f, .5f, .5f));
    for (const auto [i, d2] : ss | views::take(2)) {
      assertx(d2 == 0.f);
      found.push(i);
    }
    sort(found);
    SHOW(found);
    verify_complete_search(sp, Point(1.f, 1.f, 1.f), pa);
    verify_complete_search(sp, Point(0.f, 1.f, 0.f), pa);
  }
  {
    // IPointSpatial, for various grid resolutions including a single cell.
    Random random(2);
    Array<Point> pa(300);
    for (Point& p : pa) for_int(c, 3) p[c] = random.unif();
    for (const int gridn : {1, 2, 7, 30}) {
      const IPointSpatial sp(gridn, pa);
      for_int(iquery, 20) {
        Point query;
        for_int(c, 3) query[c] = random.unif();
        verify_nearest(sp, query, pa, 8);
      }
      verify_complete_search(sp, Point(.9f, .1f, .5f), pa);
    }
    // The single closest point, as used for a nearest-neighbor query.
    IPointSpatial sp(10, pa);
    const Point query(.25f, .75f, .5f);
    SpatialSearch<int> ss(&sp, query);
    const auto [i, d2] = *ss.begin();
    const int expected = narrow_cast<int>(arg_min(transformed(pa, [&](const Point& p) { return dist2(query, p); })));
    assertx(i == expected);
    SHOW(i, round_fraction_digits(d2, 1e6f));
  }
  {
    // ObjectSpatial, with small axis-aligned boxes as the objects.
    Random random(3);
    g_boxes.init(0);
    for_int(i, 60) {
      Point p0;
      for_int(c, 3) p0[c] = .05f + .8f * random.unif();
      Vector size;
      for_int(c, 3) size[c] = .01f + .1f * random.unif();
      g_boxes.push(Bbox<float, 3>(p0, p0 + size));
    }
    ObjectSpatial<BoxApprox2, BoxExact2> sp(10);
    for_int(i, g_boxes.num()) {
      const Bbox<float, 3>& box = g_boxes[i];
      sp.enter(Conv<int>::e(i + 1), interp(box[0], box[1]), [&](const Bbox<float, 3>& bb) { return bb.overlap(box); });
    }
    // The search returns the boxes in order of their exact distance.
    for_int(iquery, 20) {
      Point query;
      for_int(c, 3) query[c] = random.unif();
      Array<float> expected;
      for (const auto& box : g_boxes) expected.push(box_dist2(query, box));
      sort(expected);
      int count = 0;
      for (const auto [id, d2] : SpatialSearch<int>(&sp, query)) {
        assertx(abs(d2 - box_dist2(query, g_boxes[id - 1])) <= 1e-6f);
        assertx(abs(d2 - expected[count]) <= 1e-6f);
        count++;
      }
      assertx(count == g_boxes.num());
    }
    // search_segment() reports (at least) every box that the segment intersects, each at most once.
    for_int(iseg, 20) {
      Point p1, p2;
      for_int(c, 3) p1[c] = random.unif(), p2[c] = random.unif();
      Set<int> reported;
      sp.search_segment(p1, p2, [&](Univ id) {
        assertx(reported.add(Conv<int>::d(id)));
        return BIGFLOAT;  // Report no intersection, so that the search continues to the end of the segment.
      });
      for_int(i, g_boxes.num()) {
        bool intersects = false;
        for_int(j, 1001) intersects |= box_dist2(interp(p1, p2, float(j) / 1000.f), g_boxes[i]) == 0.f;
        if (intersects) assertx(reported.contains(i + 1));
      }
    }
    // When ftest reports an intersection at the start of the segment, the search stops early.
    int num_tested = 0;
    sp.search_segment(Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f), [&](Univ) {
      num_tested++;
      return 0.f;
    });
    assertx(num_tested >= 1 && num_tested < g_boxes.num());
    sp.clear();
  }
}
