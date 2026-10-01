// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/BoundingSphere.h"

#include "libHh/Random.h"
using namespace hh;

namespace {

// Does sphere s1 contain sphere s2 (up to a small tolerance)?
bool contains(const BoundingSphere& s1, const BoundingSphere& s2) {
  return dist(s1.point, s2.point) + s2.radius <= s1.radius * (1.f + 1e-5f) + 1e-5f;
}

}  // namespace

int main() {
  {
    const BoundingSphere big{Point(0.f, 0.f, 0.f), 10.f}, small{Point(1.f, 2.f, 3.f), 1.f};
    SHOW(bsphere_union(big, small), bsphere_union(small, big));  // Containment, in either order.
    const BoundingSphere s1{Point(0.f, 0.f, 0.f), 1.f}, s2{Point(4.f, 0.f, 0.f), 1.f};
    SHOW(bsphere_union(s1, s2));  // Disjoint spheres: the union spans from x = -1 to x = 5.
    const BoundingSphere s3{Point(1.5f, 0.f, 0.f), 1.f};
    SHOW(bsphere_union(s1, s3));  // Partly overlapping: the union spans from x = -1 to x = 2.5.
    const BoundingSphere s4{Point(1.f, 0.f, 0.f), 2.f};
    SHOW(bsphere_union(s1, s4));  // Internally tangent at x = -1, so s4 contains s1.
  }
  {
    // The union contains both spheres, and has the smallest possible radius.
    Random random(1);
    const auto random_sphere = [&] {
      const Point p(random.unif() * 10.f, random.unif() * 10.f, random.unif() * 10.f);
      return BoundingSphere{p, random.unif() * 5.f};
    };
    for_int(i, 1000) {
      const BoundingSphere s1 = random_sphere(), s2 = random_sphere();
      const BoundingSphere s = bsphere_union(s1, s2);
      assertx(contains(s, s1) && contains(s, s2));
      const float min_radius =
          std::max({s1.radius, s2.radius, (dist(s1.point, s2.point) + s1.radius + s2.radius) / 2});
      assertx(abs(s.radius - min_radius) <= 1e-5f * max(1.f, min_radius));
    }
  }
}
