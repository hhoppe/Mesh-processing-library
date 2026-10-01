// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Bbox.h"

using namespace hh;

namespace {

float rounded(float v) { return round_fraction_digits(v, 1e4f); }

}  // namespace

int main() {
  {
    Bbox bb(Point(1, 2, 3), Point(3, 4, 5));
    SHOW(bb);
    bb.union_with(Point(3, 2, 6));
    SHOW(bb);
    bb.union_with(Bbox(V(Point(3, 1, 8), Point(3, 2, 2))));
    SHOW(bb);
    SHOW(bb.max_side());
    SHOW(bb.enclosing_hypercube());
    SHOW(bb.get_frame_to_cube());
    SHOW(bb.get_frame_to_small_cube());
    const Frame frame(Vector(0, 1, 0), Vector(2, 0, 0), Vector(0, 0, 1), Point(100, 200, 300));
    SHOW(bb.transform(frame));
  }
  {
    Bbox bb(V(V(9), V(5), V(3), V(2), V(4)));
    SHOW(bb);
  }
  {
    Bbox bb(V(V(9., 2.), V(4., 3.)));
    SHOW(bb);
  }
  {
    Bbox<float, 2> bb;
    SHOW(bb);
    bb.infinite();
    SHOW(bb);
    bb.clear();
    SHOW(bb);
  }
  {
    Bbox<int32_t, 3> bb;
    SHOW(bb);
  }
  {
    // A non-const lvalue Bbox is copied rather than treated as a range of two points.
    Bbox bb(V(1.f, 2.f), V(3.f, 5.f));
    Bbox bb2(bb);
    assertx(bb2 == bb);
    const Bbox<float, 2> bb3 = bb;
    assertx(bb3 == bb);
    // A range of one point gives a degenerate box.
    SHOW(Bbox(V(V(1.f, 2.f))));
    // An empty range gives the cleared (empty) box.
    assertx(Bbox<float, 2>(Array<Vec2<float>>{}) == Bbox<float, 2>());
  }
  {
    // Predicates.
    const Bbox<int, 2> bb(V(0, 0), V(4, 4));
    const Bbox<int, 2> inner(V(1, 1), V(3, 4)), touching(V(4, 2), V(6, 3)), apart(V(5, 0), V(6, 1));
    assertx(inner.inside(bb) && !bb.inside(inner) && bb.inside(bb));
    assertx(!touching.inside(bb));
    assertx(bb.overlap(inner) && inner.overlap(bb));
    assertx(bb.overlap(touching) && touching.overlap(bb));  // Touching boxes overlap.
    assertx(!bb.overlap(apart) && !apart.overlap(bb));
    assertx(!Bbox<int, 2>(V(0, 5), V(4, 6)).overlap(bb));  // Separated along the second axis only.
    SHOW(bb.max_side(), inner.max_side(), touching.max_side());
    // The empty box lies inside every box, and overlaps none.
    const Bbox<int, 2> empty;
    assertx(empty.inside(bb) && !empty.overlap(bb) && !bb.overlap(empty));
    // The infinite box contains every box.
    Bbox<int, 2> infinite;
    infinite.infinite();
    assertx(bb.inside(infinite) && infinite.overlap(bb) && !infinite.inside(bb));
  }
  {
    // Union and intersection.
    const Bbox<float, 2> bb1(V(0.f, 0.f), V(4.f, 2.f)), bb2(V(3.f, -1.f), V(5.f, 1.f));
    SHOW(bbox_union(bb1, bb2));
    Bbox<float, 2> bb = bb1;
    bb.intersect(bb2);
    SHOW(bb);
    // The union with an empty box is the identity.
    assertx(bbox_union(bb1, Bbox<float, 2>()) == bb1 && bbox_union(Bbox<float, 2>(), bb1) == bb1);
    // The intersection of disjoint boxes is empty (with some pmin > pmax), and overlaps nothing.
    Bbox<float, 2> bb3 = bb1;
    bb3.intersect(Bbox<float, 2>(V(10.f, 10.f), V(11.f, 11.f)));
    SHOW(bb3);
    assertx(!bb3.overlap(bb1));
    // The union of a box with each of its points leaves it unchanged.
    Bbox<float, 2> bb4 = bb1;
    for (const auto& pt : bb1) bb4.union_with(pt);
    assertx(bb4 == bb1);
  }
  {
    // Hypercubes.
    const Bbox<double, 2> bb(V(0., 0.), V(2., 2.));
    assertx(bb.enclosing_hypercube() == bb);
    const Bbox<double, 3> bb2(V(0., 0., 0.), V(1., 2., 4.));
    SHOW(bb2.enclosing_hypercube());
    const Bbox<double, 3> cube = bb2.enclosing_hypercube();
    assertx(bb2.inside(cube));
    const auto side = cube[1] - cube[0];
    assertx(side[0] == side[1] && side[1] == side[2]);
  }
  {
    // get_frame_to_cube() scales uniformly into the unit cube, centered in x and y, and resting on z == 0.
    const Bbox bb(Point(-3.f, 10.f, 5.f), Point(1.f, 12.f, 6.f));
    const Bbox<float, 3> bb_cube = bb.transform(bb.get_frame_to_cube());
    SHOW(bb_cube);
    assertx(bb_cube.inside(Bbox(Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f))));
    assertx(abs(bb_cube.max_side() - 1.f) < 1e-6f);
    const Bbox<float, 3> bb_small = bb.transform(bb.get_frame_to_small_cube());
    SHOW(bb_small);
    assertx(abs(bb_small.max_side() - .8f) < 1e-6f);
    const Bbox<float, 3> bb_small2 = bb.transform(bb.get_frame_to_small_cube(.5f));
    assertx(abs(bb_small2.max_side() - .5f) < 1e-6f);
    // A box that is already the unit cube has a get_frame_to_small_cube() that snaps to the identity.
    assertx(Bbox(Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f)).get_frame_to_cube().is_ident());
    assertx(Bbox(Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 1.f)).get_frame_to_small_cube(.98f).is_ident());
  }
  {
    // A transformed bbox bounds the transformed corners, including under rotation.
    const Bbox bb(Point(0.f, 0.f, 0.f), Point(2.f, 1.f, 1.f));
    const Bbox<float, 3> bb2 = bb.transform(Frame::rotation(2, TAU / 8));
    SHOW(transformed(bb2[0], rounded));
    SHOW(transformed(bb2[1], rounded));
    SHOW(bb.transform(Frame::rotation(2, TAU / 4)));
  }
}

template class hh::Bbox<float, 1>;
template class hh::Bbox<double, 2>;
template class hh::Bbox<int, 3>;
template class hh::Bbox<float, 3>;
