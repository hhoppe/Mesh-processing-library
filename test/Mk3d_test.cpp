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
    const MkSave mk_save(mk);
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
    const MkSaveColor mk_save_color(mk);
    mk.diffuse(1, 1, 1);
    mk.specular(.5f, .5f, .2f);
    mk.phong(4);
    mk.point(6, 7, 8);
    mk.normal(2, 0, 0);
    mk.end_point();
  }
  os.write_comment(" A two-sided polygon, output in both orientations.");
  mk.point(0, 0, 0);
  mk.point(1, 0, 0);
  mk.point(0, 1, 0);
  mk.point(-1, 1, 0);
  mk.end_2polygon();
  os.write_comment(" A polygon forced to a closed polyline, and a two-sided polygon forced to a polyline.");
  mk.begin_force_polyline(true);
  mk.point(0, 0, 0);
  mk.point(1, 0, 0);
  mk.point(0, 1, 0);
  mk.end_polygon();
  mk.point(0, 0, 1);
  mk.point(1, 0, 1);
  mk.point(0, 1, 1);
  mk.end_2polygon();
  mk.end_force_polyline();
  os.write_comment(" A flipped polygon, whose normals are also negated.");
  mk.begin_force_flip(true);
  mk.point(0, 0, 0);
  mk.normal(1, 2, 2);
  mk.point(1, 0, 0);
  mk.normal(2, 1, 2);
  mk.point(1, 1, 0);
  mk.normal(2, 2, 1);
  mk.point(0, 1, 0);
  mk.normal(1, 1, 1);
  mk.end_polygon();
  mk.end_force_flip();
  os.write_comment(" Normals transform by the inverse transpose, so they stay perpendicular to the scaled surface.");
  mk.save([&] {
    mk.scale(1, 2, 1);
    mk.point(0, 0, 0);
    mk.normal(1, 1, 0);
    mk.point(1, -1, 0);
    mk.normal(1, 1, 0);
    mk.point(0, 0, 1);
    mk.normal(1, 1, 0);
    mk.end_polygon();
  });
  os.write_comment(" Nested transforms: the most recent transform applies first.");
  mk.save([&] {
    mk.translate(Vector(100, 0, 0));
    mk.save([&] {
      mk.scale(2);
      mk.point(1, 1, 1);
      mk.end_point();
    });
    mk.point(1, 1, 1);
    mk.end_point();
    mk.apply(Frame::scaling(V(-1.f, 1.f, 1.f)));  // A mirror, applied before the translation.
    mk.point(1, 1, 1);
    mk.end_point();
  });
  os.write_comment(" Colors.");
  mk.save_color([&] {
    mk.color(A3dVertexColor(A3dColor(.5f, .5f, 1.f)));
    mk.scale_color(.5f, 1.f, .25f);
    mk.point(1, 1, 1);
    mk.end_point();
    mk.save_color([&] {
      mk.diffuse(A3dColor(0, 1, 0));
      mk.specular(A3dColor(0, 0, 1));
      mk.point(2, 2, 2);
      mk.end_point();
    });
    mk.point(3, 3, 3);
    mk.end_point();
  });
  mk.point(4, 4, 4);  // With the original color.
  mk.end_point();
  assertx(&mk.oa3d() == &os);
}
