// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Bbox.h"

using namespace hh;

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
}

template class hh::Bbox<float, 1>;
template class hh::Bbox<double, 2>;
template class hh::Bbox<int, 3>;
template class hh::Bbox<float, 3>;
