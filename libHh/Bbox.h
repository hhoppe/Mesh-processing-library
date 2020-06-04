// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BBOX_H_
#define MESH_PROCESSING_LIBHH_BBOX_H_

#include "libHh/Geometry.h"
#include "libHh/RangeOp.h"

namespace hh {

// Axis-aligned bounding box in 3D.
struct Bbox : Vec2<Point> {
  Bbox() { clear(); }
  constexpr Bbox(const Point& pmin, const Point& pmax) : Vec2<Point>(pmin, pmax) {}
  constexpr Bbox(Vec2<Point> bbox) : Vec2<Point>(std::move(bbox)) {}
  void clear() { (*this)[0] = thrice(+big), (*this)[1] = thrice(-big); }
  void infinite() { (*this)[0] = thrice(-big), (*this)[1] = thrice(+big); }
  void union_with(const Bbox& bbox) {
    auto& self = *this;
    for_int(c, 3) {
      if (bbox[0][c] < self[0][c]) self[0][c] = bbox[0][c];
      if (bbox[1][c] > self[1][c]) self[1][c] = bbox[1][c];
    }
  }
  void union_with(const Point& pp) {
    auto& self = *this;
    for_int(c, 3) {
      float v = pp[c];
      if (v < self[0][c]) self[0][c] = v;
      if (v > self[1][c]) self[1][c] = v;
    }
  }
  void intersect(const Bbox& bbox) {
    auto& self = *this;
    for_int(c, 3) {
      if (bbox[0][c] > self[0][c]) self[0][c] = bbox[0][c];
      if (bbox[1][c] < self[1][c]) self[1][c] = bbox[1][c];
    }
  }
  bool inside(const Bbox& bbox) const {
    auto& self = *this;
    for_int(c, 3) {
      if (self[0][c] < bbox[0][c]) return false;
      if (self[1][c] > bbox[1][c]) return false;
    }
    return true;
  }
  bool overlap(const Bbox& bbox) const {
    auto& self = *this;
    for_int(c, 3) {
      if (bbox[0][c] > self[1][c] || bbox[1][c] < self[0][c]) return false;
    }
    return true;
  }
  Frame get_frame_to_cube() const {  // uniform scaling into unit cube, centered on x & y, rest at z == 0
    auto& self = *this;
    Vector di = self[1] - self[0];
    float maxdi = max(di);
    assertx(maxdi);
    Vector center;
    for_int(c, 3) center[c] = (1.f - di[c] / maxdi) * .5f;
    center[2] = 0.f;  // objects lie at bottom of cube
    return Frame::translation(-to_Vector(self[0])) * Frame::scaling(thrice(1.f / maxdi)) * Frame::translation(center);
  }
  Frame get_frame_to_small_cube(float cubesize = .8f) const {
    Frame f = get_frame_to_cube();
    float bnd = (1.f - cubesize) / 2.f;
    f = f * Frame::scaling(thrice(cubesize)) * Frame::translation(thrice(bnd));
    for_int(i, 3) {
      if (abs(f[i][i] - 1.f) < .05f) f[i][i] = 1.f;
      if (abs(f[3][i]) < .05f) f[3][i] = 0.f;
    }
    return f;
  }
  float max_side() const {
    auto& self = *this;
    return max({self[1][0] - self[0][0], self[1][1] - self[0][1], self[1][2] - self[0][2]});
  }
  void transform(const Frame& frame) {
    auto& self = *this;
    Bbox bbox;
    for_int(i0, 2) for_int(i1, 2) for_int(i2, 2) {
      Point corner(i0 ? self[1][0] : self[0][0], i1 ? self[1][1] : self[0][1], i2 ? self[1][2] : self[0][2]);
      bbox.union_with(corner * frame);
    }
    self = bbox;
  }
  friend std::ostream& operator<<(std::ostream& os, const Bbox& bbox) {
    return os << "Bbox{" << bbox[0] << ", " << bbox[1] << "}";
  }

 private:
  static constexpr float big = BIGFLOAT;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BBOX_H_
