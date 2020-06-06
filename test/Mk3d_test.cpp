// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Mk3d.h"
using namespace hh;

int main() {
  WSA3dStream os(std::cout);
  Mk3d mk(os);
  os.write_comment(" begin test of mk3d");
  mk.point(1, 2, 3);
  mk.point(4, 5, 6);
  mk.point(7, 8, 9);
  mk.end_polygon();
  {
    MkSave mk_save(mk);
    mk.translate(10, 0, 0);
    mk.rotate(Mk3d::Axis::z, TAU / 4);
    mk.scale(1, 1, .5);
    mk.point(1, 2, 3);
    mk.point(4, 5, 6);
    mk.point(7, 8, 9);
    mk.end_polygon();
  }
  mk.point(1, 2, 3);
  mk.normal(1, 0, 0);
  mk.point(4, 5, 6);
  mk.normal(1, 1, 0);
  mk.point(7, 8, 9);
  mk.normal(1, 1, 1);
  mk.end_polyline();
  {
    MkSaveColor mk_save_color(mk);
    mk.diffuse(1, 1, 1);
    mk.specular(.5f, .5f, .2f);
    mk.phong(4);
    mk.point(6, 7, 8);
    mk.normal(2, 0, 0);
    mk.end_point();
  }
}
