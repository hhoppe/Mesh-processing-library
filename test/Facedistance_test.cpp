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
      const auto [d1, bary1, clp1] = project_point_triangle(p, p1, p2, p3);
      const auto [d2, bary2, clp2] = project_point_triangle(p, p2, p3, p1);
      const auto [d3, bary3, clp3] = project_point_triangle(p, p3, p1, p2);
      const float dmin = min({d1, d2, d3});
      const float dmax = max({d1, d2, d3});
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
  const Point p1(2.f, 3.f, 9.f);
  const Point p2(4.f, 7.f, 10.f);
  const Point p3(6.f, 5.f, 11.f);
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
    auto [d2, bary, clp] = project_point_triangle(p, p1, p2, p3);
    SHOW("");
    round_elements(p, vround);
    round_elements(clp, vround);
    round_elements(ArView(d2), vround);
    round_elements(bary, vround);
    SHOW(p);
    SHOW(clp);
    SHOW(d2);
    SHOW(bary);
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

// Closest point on the triangle, computed in double precision: the projection into the triangle interior if it lies
// there, else the closest of the projections onto the three sides.
Vec3<double> reference_closest_point(const Point& fp, const Vec3<Point>& ftriangle) {
  const Vec3<double> p = convert<double>(fp);
  const Vec3<Vec3<double>> triangle = transformed(ftriangle, [](const Point& pp) { return convert<double>(pp); });
  const Vec3<double> v12 = triangle[1] - triangle[0], v13 = triangle[2] - triangle[0], v1p = p - triangle[0];
  const Vec3<double> n = cross(v12, v13);
  const double nn = mag2(n);
  const double w2 = dot(v1p, cross(v13, n)), w3 = dot(v1p, cross(n, v12));
  if (nn > 0. && w2 >= 0. && w3 >= 0. && w2 + w3 <= nn) return triangle[0] + v12 * (w2 / nn) + v13 * (w3 / nn);
  double min_d2 = BIGFLOAT;
  Vec3<double> result = triangle[0];
  for_int(i, 3) {
    const Vec3<double> q0 = triangle[i], v = triangle[mod3(i + 1)] - q0;
    const double vv = mag2(v);
    const double t = vv > 0. ? clamp(dot(v, p - q0) / vv, 0., 1.) : 0.;
    const Vec3<double> q = q0 + v * t;
    if (const double d2 = dist2(p, q); d2 < min_d2) {
      min_d2 = d2;
      result = q;
    }
  }
  return result;
}

// Verify the accuracy of project_point_triangle() on families of challenging configurations.  Each error is relative
// to the longest triangle edge.  The tolerances are at least 6x the maximum errors of the current implementation (as
// measured with Clang at -O0 and -O3 and with GCC).  On these same inputs, the previous implementation (which solved
// the normal equations for the planar projection) had maximum closest-point errors of 1.5e-5 on "cube", 1.3e-2 on
// "sliver", and 1.1e-5 on "collinear", so it failed those tolerances.
void test3() {
  const auto random_point = [] {
    Point p;
    for_int(c, 3) p[c] = Random::G.unif();
    return p;
  };
  const auto random_offset = [&](float scale) { return (random_point() - Point(.5f, .5f, .5f)) * scale; };
  struct Family {
    string name;
    float tolerance;
  };
  const Array<Family> families = {
      {"cube", 1e-5f},   {"near_plane", 1e-4f}, {"offset_1000", 2e-3f}, {"far_point", 1e-3f},
      {"sliver", 2e-3f}, {"collinear", 5e-6f},  {"coincident", 1e-5f},  {"tiny", 1e-3f},
  };
  for (const Family& family : families) {
    const string& name = family.name;
    float max_err = 0.f;
    for_int(i, 20'000) {
      Point p = random_point();
      Vec3<Point> triangle{random_point(), random_point(), random_point()};
      if (name == "near_plane") {
        const Vector nor = normalized(cross(triangle[0], triangle[1], triangle[2]));
        const float b1 = Random::G.unif(), b2 = Random::G.unif() * (1.f - b1);
        p = interp(triangle, Bary(b1, b2, 1.f - b1 - b2)) + nor * ((Random::G.unif() - .5f) * 2e-3f);
      } else if (name == "offset_1000") {
        const Vector offset(1000.f, 1000.f, 1000.f);
        p += offset;
        for (Point& pt : triangle) pt += offset;
      } else if (name == "far_point") {
        p = triangle[0] + normalized(random_offset(1.f)) * 1000.f;
      } else if (name == "sliver") {
        // Offset the third vertex from the first edge by 1e-4, perpendicular to that edge.
        const float t = Random::G.unif();
        const Vector perp = normalized(cross(triangle[1] - triangle[0], random_offset(1.f)));
        triangle[2] = interp(triangle[0], triangle[1], t) + perp * 1e-4f;
      } else if (name == "collinear") {
        triangle[2] = interp(triangle[0], triangle[1], float(Random::G.get_unsigned(5)) / 4.f);
      } else if (name == "coincident") {
        const int k = Random::G.get_unsigned(3);
        triangle[k] = triangle[mod3(k + 1)];
      } else if (name == "tiny") {
        triangle[1] = triangle[0] + random_offset(1e-3f);
        triangle[2] = triangle[0] + random_offset(1e-3f);
      } else {
        assertx(name == "cube");
      }
      const float scale = max(
          {dist(triangle[0], triangle[1]), dist(triangle[1], triangle[2]), dist(triangle[2], triangle[0]), 1e-20f});
      const Vec3<double> ref_clp = reference_closest_point(p, triangle);
      Vec3<float> dists;
      for_int(rot, 3) {
        const Vec3<Point> rotated{triangle[rot], triangle[mod3(rot + 1)], triangle[mod3(rot + 2)]};
        const auto [d2, bary, clp] = project_point_triangle(p, rotated);
        assertx(min(bary) >= 0.f && abs(bary[0] + bary[1] + bary[2] - 1.f) < 1e-6f);
        assertx(dist(interp(rotated, bary), clp) <= family.tolerance * scale + 1e-6f * mag(p));
        max_err = max(max_err, float(mag(convert<double>(clp) - ref_clp)) / scale);
        dists[rot] = std::sqrt(d2);
      }
      // The three vertex orderings must agree on the distance.
      assertx(max(dists) - min(dists) <= family.tolerance * scale);
    }
    assertx(max_err < family.tolerance);
    showf("%-12s max closest-point error < %g\n", name.c_str(), family.tolerance);
  }
}

}  // namespace

int main() {
  test1();
  test2();
  test3();
}
