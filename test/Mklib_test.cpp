// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Mklib.h"

#include <functional>  // std::function
#include <sstream>     // std::stringstream

#include "libHh/Bbox.h"
#include "libHh/Polygon.h"
#include "libHh/RangeOp.h"  // round_elements()
using namespace hh;

namespace {

// Generate a shape into a separate stream, and summarize its geometry: its number of polygons, its bounding box, and
// its enclosed volume (which is positive iff the polygons of a closed shape are oriented outward).
string summarize_shape(const std::function<void(Mklib&)>& func, bool closed) {
  std::stringstream ss;
  {
    WSA3dStream oa3d(ss);
    Mk3d mk(oa3d);
    Mklib mkl(mk);
    func(mkl);
  }
  RSA3dStream ia3d(ss);
  A3dElem el;
  Bbox<float, 3> bbox;
  double volume = 0.;
  int num_polygons = 0;
  Polygon poly;
  for (;;) {
    ia3d.read(el);
    if (el.type() == A3dElem::EType::endfile) break;
    if (el.type() != A3dElem::EType::polygon) continue;
    num_polygons++;
    el.get_polygon(poly);
    const Vector pnormal = poly.get_normal();
    for_int(i, el.num()) {
      bbox.union_with(el[i].p);
      // Any vertex normal lies on the front side of the polygon.
      if (!is_zero(el[i].n)) assertx(dot(el[i].n, pnormal) > 0.f);
    }
    for_intL(i, 1, el.num() - 1) volume += dot(el[0].p, cross(el[i].p, el[i + 1].p)) / 6.;
  }
  const auto rounded = [](const Point& p) {
    return transformed(p, [](float v) { return round_fraction_digits(v, 1e4f); });
  };
  string str =
      sform("npolygons=%d bbox=", num_polygons) + make_string(rounded(bbox[0])) + " " + make_string(rounded(bbox[1]));
  if (closed) str += sform(" volume=%.4f", volume);
  return str;
}

// Summarize various shapes, as comments in the output stream.
void summarize_shapes(WSA3dStream& os) {
  const auto show = [&](const string& name, const std::function<void(Mklib&)>& func, bool closed) {
    os.write_comment(" " + name + ": " + summarize_shape(func, closed));
  };
  show("squareO", [](Mklib& mkl) { mkl.squareO(); }, false);
  show("squareXY", [](Mklib& mkl) { mkl.squareXY(); }, false);
  show("squareU", [](Mklib& mkl) { mkl.squareU(); }, false);
  show("cubeO", [](Mklib& mkl) { mkl.cubeO(); }, true);
  show("cubeXYZ", [](Mklib& mkl) { mkl.cubeXYZ(); }, true);
  show("cubeU", [](Mklib& mkl) { mkl.cubeU(); }, true);
  show("polygonO(6)", [](Mklib& mkl) { mkl.polygonO(6); }, false);
  show("polygonU(6)", [](Mklib& mkl) { mkl.polygonU(6); }, false);
  // The volume of a prism over a regular n-gon of circumradius 1 is n / 2 * sin(TAU / n) == 2.5981 for n == 6.
  show("cylinderU(6)", [](Mklib& mkl) { mkl.cylinderU(6); }, true);
  show("tubeU(6)", [](Mklib& mkl) { mkl.tubeU(6); }, false);
  show("coneU(6)", [](Mklib& mkl) { mkl.coneU(6); }, true);  // The volume is 2.5981 / 3.
  show("capU(6)", [](Mklib& mkl) { mkl.capU(6); }, false);
  show("ringU", [](Mklib& mkl) { mkl.ringU(5, 2.f, 1.f, .5f, .1f, .2f); }, false);
  show("flat_ringU", [](Mklib& mkl) { mkl.flat_ringU(5, 1.f, 1.f, .5f); }, false);
  show("poly_hole", [](Mklib& mkl) { mkl.poly_hole(8, .5f); }, false);
  // The volume of the discrete torus is (1 - .25) * 2.8284 for n == 8 and r1 == .5.
  show("volume_ringU", [](Mklib& mkl) { mkl.volume_ringU(8, .5f); }, true);
  show("sphere", [](Mklib& mkl) { mkl.sphere(8, 12); }, true);  // The volume approaches 4.1888.
  const auto nonsmooth_sphere = [](Mklib& mkl) {
    mkl.begin_smooth(false);
    assertx(!mkl.smooth());
    mkl.sphere(8, 12);
    mkl.end_smooth();
    assertx(mkl.smooth());
  };
  show("sphere_nonsmooth", nonsmooth_sphere, true);
  show("hemisphere", [](Mklib& mkl) { mkl.hemisphere(4, 12); }, false);
  show("tetra", [](Mklib& mkl) { mkl.tetra(); }, true);  // The volume is 1 / (6 * sqrt(2)) == 0.1179.
  // KNOWN_BUG: Mklib::tetraU() should place its bottom face at z == 0, but it currently translates the tetrahedron
  // along x rather than z, so its bbox is not shown.
  if (0) show("tetraU", [](Mklib& mkl) { mkl.tetraU(); }, true);
}

}  // namespace

int main(int argc, char** argv) {
  dummy_use(argv);
  if (argc > 1) {
    RSA3dStream ia3d(std::cin);
    WSA3dStream oa3d(std::cout);
    A3dElem el;
    for (;;) {
      ia3d.read(el);
      if (el.type() == A3dElem::EType::endfile) break;
      if (el.type() == A3dElem::EType::polygon || el.type() == A3dElem::EType::polyline) {
        for_int(i, el.num()) {
          const float fac = 1e4f;
          round_elements(el[i].p, fac);
          round_elements(el[i].n, fac);
        }
      }
      oa3d.write(el);
    }
    return 0;
  }
  WSA3dStream os(std::cout);
  Mk3d mk(os);
  Mklib mkl(mk);
  os.write_comment(" begin test of mklib");
  os.write_comment("cubeO");
  {
    const MkSave mk_save(mk);
    mk.translate(2, 0, 0);
    mkl.cubeO();
  }
  os.write_comment("cubeU");
  {
    const MkSave mk_save(mk);
    mk.translate(4, 0, 0);
    mkl.cubeU();
  }
  {
    const MkSave mk_save(mk);
    mk.translate(6, 0, 0);
    mk.rotate(Mk3d::Axis::z, TAU / 4);
    mk.scale(1, 1, .5);
    os.write_comment("tetra");
    mkl.tetra();
    os.write_comment("polygon");
    mk.point(10, 2, 3);
    mk.point(11, 3, 5);
    mk.point(11, 5, 2);
    mk.end_polygon();
  }
  os.write_comment("tetra");
  {
    const MkSave mk_save(mk);
    mk.translate(2, 5, 0);
    mkl.tetra();
  }
  os.write_comment("cylinderU");
  {
    const MkSave mk_save(mk);
    mk.translate(5, 5, 0);
    mkl.cylinderU(7);
  }
  {
    const MkSaveColor mk_save_color(mk);
    mk.diffuse(1, 1, 1);
    mk.specular(.5f, .5f, .2f);
    mk.phong(4);
    os.write_comment("volume_ringU");
    {
      const MkSave mk_save(mk);
      mk.translate(8, 5, 0);
      mkl.volume_ringU(5, .7f);
    }
    os.write_comment("capU");
    {
      const MkSave mk_save(mk);
      mk.translate(2, 8, 0);
      mkl.capU(3);
    }
  }
  os.write_comment("sphere");
  {
    const MkSave mk_save(mk);
    mk.translate(4, 8, 2);
    mkl.sphere(4, 5);
  }
  os.write_comment("tetraU");
  {
    const MkSave mk_save(mk);
    mk.translate(7, 8, 2);
    mkl.tetraU();
  }
  summarize_shapes(os);
  return 0;
}
