// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MKLIB_H_
#define MESH_PROCESSING_LIBHH_MKLIB_H_

#include "libHh/Mk3d.h"

namespace hh {

// Create a stream of geometric shapes on a WA3dStream.
class Mklib : noncopyable {
 public:
  explicit Mklib(Mk3d& mk3d) : mk(mk3d) {}
  ~Mklib() { assertw(_stack_smooth.empty()); }
  [[nodiscard]] bool smooth() const { return _smooth; }
  void begin_smooth(bool b) { _stack_smooth.push(_smooth), _smooth = b; }
  void end_smooth() { assertx(!_stack_smooth.empty()), _smooth = _stack_smooth.pop(); }
  void squareO();        // Unit square centered at the origin, facing the +x axis.
  void squareXY();       // Unit square between (0, 0, 0) and (1, 1, 0), facing +z.
  void squareU();        // Square above the origin, in the xy plane, facing the +z axis.
  void cubeO();          // Unit cube centered at the origin.
  void cubeXYZ();        // Unit cube between (0, 0, 0) and (1, 1, 1).
  void cubeU();          // Cube with the center of the bottom face at the origin.
  void polygonO(int n);  // Regular polygon, radius 1, normal to the x axis, vertex on the y axis.
  void polygonU(int n);  // Polygon facing the +z axis, vertex on the +x axis.
  void ringU(int n, float h, float r0, float r1, float a0, float a1);
  void flat_ringU(int n, float h, float r0, float r1);  // Ring with angles such that it is flat (not smooth).
  void poly_hole(int n, float r1);                      // Annulus (bounded by two concentric circles).
  void volume_ringU(int n, float r1);                   // A "discrete torus" with rectangular cross section.
  void tubeU(int n);                                    // Height 1, radius 1, open in +z axis, vertex on +x axis.
  void cylinderU(int n);                                // Cylinder == tube with closed ends.
  void capU(int n);                                     // Height 1, radius 1, bottom at origin, peak at (1, 0, 0).
  void coneU(int n);                                    // Cone == cap with closed bottom.
  void sphere(int nlat, int nlong);                     // Radius 1, #latitudes (>= 2), #longitudes (>= 3).
  void hemisphere(int nlat, int nlong);                 // Radius 1, #latitudes (>= 2), #longitudes (>= 2).
  void tetra();                                         // Centered at centroid, edge = 1, height = sqrt(2 / 3).
  void tetraU();                                        // Bottom face centroid at origin, top at (0, 0, sqrt(2 / 3)).
  // Transformation on object: (-.5, -.5, -.5)..(.5, .5, .5) with primary axis +x, secondary axis +y ("O")
  //                        -->  (-.5, -.5, 0)..(.5, .5, 1)  with primary axis +z, secondary axis +x ("U")
  template <typename Func = void(int)> void OtoU(Func func, int n) {
    const MkSave mk_save(mk);
    mk.rotate(Mk3d::Axis::y, TAU / 4);
    mk.rotate(Mk3d::Axis::x, -TAU / 4);
    mk.translate(.5f, 0, 0);
    func(n);
  }
  // radius 1 along +x axis, calls func with +x axis normal to circle;  s = sin(TAU / 2 / n) h = cos(TAU / 2 / n);
  //  scaled to touch at (0, -s, 0) & (0, +s, 0) and center of circle at (-h, 0, 0).
  template <typename Func = void(int)> void circle_of(Func func, int n) {
    const float a = TAU / n, h = std::cos(a * .5f);
    const MkSave mk_save(mk);
    mk.rotate(Mk3d::Axis::y, TAU / 4);
    mk.rotate(Mk3d::Axis::z, TAU / 4);
    mk.rotate(Mk3d::Axis::z, a * .5f);
    for_int(i, n) {
      {
        const MkSave mk_save2(mk);
        mk.translate(h, 0, 0);
        func(i);
      }
      mk.rotate(Mk3d::Axis::z, a);
    }
  }
  template <typename Func = void(int)> void circle_ofU(Func func, int n) {
    const float a = TAU / n, h = std::cos(a * .5f);
    const MkSave mk_save(mk);
    mk.rotate(Mk3d::Axis::z, a * .5f);
    for_int(i, n) {
      {
        const MkSave mk_save2(mk);
        mk.translate(h, 0, 0);
        func(i);
      }
      mk.rotate(Mk3d::Axis::z, a);
    }
  }
  // radius 1 along +z axis, calls func with +x axis normal to circle;
  //  not scaled -> center of circle @(-1, 0, 0).
  template <typename Func = void(int)> void radius_ofU(Func func, int n) {
    const float a = TAU / n;
    const MkSave mk_save(mk);
    for_int(i, n) {
      {
        const MkSave mk_save2(mk);
        mk.translate(1, 0, 0);
        func(i);
      }
      mk.rotate(Mk3d::Axis::z, a);
    }
  }
  // Height h, 2 radii, and normal angles with respect to the XY plane.
 public:
  Mk3d& mk;

 private:
  bool _smooth{true};
  Stack<bool> _stack_smooth;
  void gsphere(int nlat, int nlong, bool hemi);
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MKLIB_H_
