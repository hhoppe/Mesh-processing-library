// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Polygon.h"

#include "libHh/GeomOp.h"
#include "libHh/RangeOp.h"  // round_elements()
using namespace hh;

namespace {

float rounded(float v) { return round_fraction_digits(v, 1e4f); }

// An axis-aligned square [0, 2]^2 in the plane z == 0, oriented counterclockwise (with normal +z).
Polygon square() {
  return Polygon{Point(0.f, 0.f, 0.f), Point(2.f, 0.f, 0.f), Point(2.f, 2.f, 0.f), Point(0.f, 2.f, 0.f)};
}

// A concave L-shaped hexagon in the plane z == 1, oriented counterclockwise.
Polygon l_shape() {
  return Polygon{Point(0.f, 0.f, 1.f), Point(3.f, 0.f, 1.f), Point(3.f, 1.f, 1.f),
                 Point(1.f, 1.f, 1.f), Point(1.f, 3.f, 1.f), Point(0.f, 3.f, 1.f)};
}

}  // namespace

int main() {
  {
    Polygon poly;
    poly.push(Point(1.f, 2.f, 3.f));
    poly.push(Point(4.f, 5.f, 6.f));
    poly.push(Point(8.f, 8.f, 10.f));
    SHOW(poly);
    poly.push(Point(7.f, 7.f, 7.f));
    SHOW(poly);
    SHOW(poly.get_normal());
  }
  {
    Polygon poly;
    poly.push(Point(0.f, 0.f, 0.f));
    poly.push(Point(1.f, 0.f, 0.f));
    poly.push(Point(0.f, 1.f, 0.f));
    const auto test = [&](const Point& p, const Vector& v) {
      const auto pint = poly.intersect_line(p, v);
      SHOW(p, v, bool(pint));
      if (pint) SHOW(*pint);
    };
    test(Point(.2f, .2f, 1.f), Vector(0.f, 0.f, 1.f));
    test(Point(.7f, .7f, 1.f), Vector(0.f, 0.f, 1.f));
    test(Point(0.f, 0.f, 1.f), Vector(.1f, .3f, -1.f));
    test(Point(0.f, 0.f, 0.f), Vector(0.f, 0.f, 1.f));
    test(Point(0.f, .5f, 0.f), Vector(0.f, 0.f, 1.f));
  }
  {
    Vec3<Point> triangle = V(Point(1.f, 1.f, 1.f), Point(1.f, 5.f, 2.f), Point(2.f, 2.f, 7.f));
    triangle = widen_triangle(triangle, 1e-5f);
    for_int(i, 3) {
      round_elements(triangle[i], 1e4f);
      showf("%.6f %.6f %.6f\n", triangle[i][0], triangle[i][1], triangle[i][2]);
    }
  }
  {
    // Normals, planes, and areas.
    const Polygon sq = square(), ls = l_shape();
    SHOW(sq.get_normal_dir(), sq.get_normal(), sq.get_area());
    SHOW(ls.get_normal_dir(), ls.get_normal(), ls.get_area());
    SHOW(sq.get_planec(sq.get_normal()), ls.get_planec(ls.get_normal()));
    SHOW(ls.get_tolerance(ls.get_normal(), 1.f), ls.get_tolerance(ls.get_normal(), 1.5f));
    SHOW(mean(ls));
    // A reversed polygon has the opposite normal.
    Polygon rev = ls;
    reverse(rev);
    assertx(rev.get_normal() == -ls.get_normal());
    // The area is independent of the orientation and of the starting vertex, even for this concave polygon, whose
    // triangle fans about (0, 3) and (3, 0) are not valid triangulations.
    assertx(rev.get_area() == ls.get_area());
    Polygon rotated = ls;
    rotate(rotated, rotated.begin() + 1);
    assertx(rotated.get_area() == ls.get_area());
    // A degenerate polygon has a zero normal.
    const Polygon degenerate{Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f), Point(2.f, 2.f, 2.f)};
    SHOW(degenerate.get_normal(), degenerate.get_area());
    // A non-planar quadrilateral has a nonzero tolerance.
    const Polygon skew{Point(0.f, 0.f, 0.f), Point(1.f, 0.f, 0.f), Point(1.f, 1.f, 1.f), Point(0.f, 1.f, 0.f)};
    const Vector skew_nor = skew.get_normal();
    SHOW(transformed(skew_nor, rounded));
    assertx(skew.get_tolerance(skew_nor, skew.get_planec(skew_nor)) > .1f);
  }
  {
    // Convexity.
    assertx(square().is_convex() && !l_shape().is_convex());
    Polygon rev = l_shape();
    reverse(rev);
    assertx(!rev.is_convex());
    assertx(Polygon(V(Point(0.f, 0.f, 0.f), Point(1.f, 0.f, 0.f), Point(0.f, 1.f, 0.f))).is_convex());
    // Each rotation of the vertex order of the concave polygon (which moves the reflex vertex) is concave.
    Polygon poly = l_shape();
    for_int(i, poly.num()) {
      rotate(poly, poly.begin() + 1);
      assertx(!poly.is_convex());
    }
  }
  {
    // Point containment within the concave polygon, compared against its decomposition into two rectangles.
    const Polygon ls = l_shape();
    const Vector nor = ls.get_normal();
    int num_inside = 0;
    for_int(iy, 16) for_int(ix, 16) {
      const Point point((float(ix) + .5f) * .25f - .5f, (float(iy) + .5f) * .25f - .5f, 1.f);
      const bool expected = (point[0] > 0.f && point[0] < 3.f && point[1] > 0.f && point[1] < 1.f) ||
                            (point[0] > 0.f && point[0] < 1.f && point[1] > 0.f && point[1] < 3.f);
      assertx(ls.point_inside(nor, point) == expected);
      num_inside += expected;
    }
    SHOW(num_inside);
    // The point is projected along the dominant axis of the normal, so its offset from the plane is irrelevant.
    assertx(ls.point_inside(nor, Point(.5f, .5f, 100.f)) && !ls.point_inside(nor, Point(2.f, 2.f, -5.f)));
    // A polygon in the plane x == 0.
    const Polygon side{Point(0.f, 0.f, 0.f), Point(0.f, 1.f, 0.f), Point(0.f, 1.f, 1.f), Point(0.f, 0.f, 1.f)};
    const Vector side_nor = side.get_normal();
    SHOW(side_nor);
    assertx(side.point_inside(side_nor, Point(0.f, .5f, .5f)) && !side.point_inside(side_nor, Point(0.f, 1.5f, .5f)));
  }
  {
    // Clipping against a halfspace.
    Polygon poly = square();
    assertx(!poly.intersect_hyperplane(Point(-1.f, 0.f, 0.f), Vector(1.f, 0.f, 0.f)));  // Entirely inside.
    assertx(poly == square());
    assertx(poly.intersect_hyperplane(Point(1.f, 0.f, 0.f), Vector(1.f, 0.f, 0.f)));  // Keep x >= 1.
    SHOW(poly);
    assertx(abs(poly.get_area() - 2.f) < 1e-5f);
    assertx(poly.intersect_hyperplane(Point(0.f, 1.f, 0.f), Vector(1.f, 1.f, 0.f)));  // Keep x + y >= 1.
    SHOW(poly.num(), rounded(poly.get_area()));
    assertx(poly.intersect_hyperplane(Point(5.f, 0.f, 0.f), Vector(1.f, 0.f, 0.f)));  // Entirely outside.
    SHOW(poly.num());
    // Clipping the L shape by the line x == 2 gives a concave hexagon.
    Polygon ls = l_shape();
    assertx(ls.intersect_hyperplane(Point(2.f, 0.f, 0.f), Vector(-1.f, 0.f, 0.f)));
    SHOW(ls);
    assertx(abs(ls.get_area() - 4.f) < 1e-5f && !ls.is_convex());
  }
  {
    // Clipping against a bbox.
    Polygon poly = square();
    assertx(!poly.intersect_bbox(Bbox(Point(-1.f, -1.f, -1.f), Point(3.f, 3.f, 1.f))));
    assertx(poly.intersect_bbox(Bbox(Point(1.f, -1.f, -1.f), Point(3.f, 1.5f, 1.f))));
    SHOW(poly);
    assertx(abs(poly.get_area() - 1.5f) < 1e-5f);
    Polygon poly2 = square();
    assertx(poly2.intersect_bbox(Bbox(Point(0.f, 0.f, 1.f), Point(2.f, 2.f, 2.f))));  // Disjoint in z.
    SHOW(poly2.num());
  }
  {
    // Intersection with a segment.
    const Polygon ls = l_shape();
    SHOW(ls.intersect_segment(Point(.5f, 2.f, 0.f), Point(.5f, 2.f, 2.f)).value());
    assertx(!ls.intersect_segment(Point(2.f, 2.f, 0.f), Point(2.f, 2.f, 2.f)));   // Through the notch.
    assertx(!ls.intersect_segment(Point(.5f, 2.f, 1.5f), Point(.5f, 2.f, 2.f)));  // Above the plane.
    SHOW(ls.intersect_line(Point(2.5f, .5f, 5.f), Vector(0.f, 0.f, 1.f)).value());
    SHOW(intersect_plane_segment(Vector(0.f, 1.f, 0.f), 1.f, Point(0.f, 0.f, 0.f), Point(2.f, 4.f, 0.f)).value());
    assertx(!intersect_plane_segment(Vector(0.f, 1.f, 0.f), 1.f, Point(0.f, 2.f, 0.f), Point(2.f, 4.f, 0.f)));
    assertx(!intersect_plane_segment(Vector(0.f, 1.f, 0.f), 1.f, Point(0.f, 1.f, 0.f), Point(2.f, 1.f, 0.f)));
  }
  {
    // Intersection with a plane: the segments where the plane x == 2 crosses the L shape rotated into the xz plane.
    const Polygon ls = l_shape();
    Array<Point> pa;
    ls.intersect_plane(ls.get_normal(), Vector(1.f, 0.f, 0.f), .5f, 0.f, pa);
    SHOW(pa);
    ls.intersect_plane(ls.get_normal(), Vector(0.f, 1.f, 0.f), .5f, 0.f, pa);
    SHOW(pa);
    ls.intersect_plane(ls.get_normal(), Vector(0.f, 0.f, 1.f), 5.f, 0.f, pa);  // Disjoint.
    SHOW(pa.num());
    ls.intersect_plane(ls.get_normal(), Vector(0.f, 0.f, 1.f), 1.f, 0.f, pa);  // Coplanar.
    SHOW(pa.num());
  }
  {
    // Intersection of two polygons: a horizontal square and a vertical square crossing it.
    const Polygon horiz = square();
    const Polygon vert{Point(1.f, -1.f, -1.f), Point(1.f, 3.f, -1.f), Point(1.f, 3.f, 1.f), Point(1.f, -1.f, 1.f)};
    SHOW(intersect_poly_poly(horiz, vert));
    SHOW(intersect_poly_poly(vert, horiz));
    // A vertical rectangle crossing both arms of the L shape (and its notch) gives two segments.
    const Polygon vert2{Point(-1.f, 3.5f, 0.f), Point(3.5f, -1.f, 0.f), Point(3.5f, -1.f, 2.f),
                        Point(-1.f, 3.5f, 2.f)};
    SHOW(intersect_poly_poly(l_shape(), vert2));
    // A vertical rectangle along the diagonal leaves the L shape at its reflex vertex.
    const Polygon vert4{Point(-1.f, -1.f, 0.f), Point(4.f, 4.f, 0.f), Point(4.f, 4.f, 2.f), Point(-1.f, -1.f, 2.f)};
    SHOW(intersect_poly_poly(l_shape(), vert4));
    const Polygon vert3{Point(-1.f, 2.5f, 0.f), Point(4.f, 2.5f, 0.f), Point(4.f, 2.5f, 2.f), Point(-1.f, 2.5f, 2.f)};
    SHOW(intersect_poly_poly(l_shape(), vert3));
    // Disjoint polygons.
    const Polygon far{Point(5.f, -1.f, -1.f), Point(5.f, 3.f, -1.f), Point(5.f, 3.f, 1.f), Point(5.f, -1.f, 1.f)};
    SHOW(intersect_poly_poly(horiz, far).num());
  }
  {
    // Orthogonal vectors and standard directions.
    for (const Vector& v :
         {Vector(1.f, 0.f, 0.f), Vector(0.f, -3.f, 0.f), Vector(1.f, 2.f, 3.f), Vector(-5.f, .1f, 2.f)}) {
      const Vector vo = orthogonal_vector(v);
      assertx(dot(v, vo) == 0.f && mag(vo) > mag(v) * .5f);
      Vector v2 = -v;
      vector_standard_direction(v2);
      Vector v3 = v;
      vector_standard_direction(v3);
      assertx(v2 == v3 && max(v2) == max_abs_element(v2));
    }
    SHOW(orthogonal_vector(Vector(1.f, 2.f, 3.f)));
  }
  {
    // Pool allocation.
    auto upoly = make_unique<Polygon>(square());
    SHOW(upoly->get_area());
  }
}
