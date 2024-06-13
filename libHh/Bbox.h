// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BBOX_H_
#define MESH_PROCESSING_LIBHH_BBOX_H_

#include "libHh/Geometry.h"
#include "libHh/Range.h"

namespace hh {

// Axis-aligned bounding box in arbitrary dimension.
template <typename T, int dim> class Bbox : public Vec2<Vec<T, dim>> {
 public:
  static_assert(std::is_arithmetic_v<T>);
  static_assert(dim > 0);
  using type = Bbox<T, dim>;
  using PointD = Vec<T, dim>;

  Bbox() { clear(); }
  constexpr Bbox(const PointD& pmin, const PointD& pmax) : Vec2<PointD>(pmin, pmax) {}
  constexpr Bbox(Vec2<PointD> bbox) : Vec2<PointD>(std::move(bbox)) {}
  template <typename Range, typename = enable_if_range_t<Range>> explicit Bbox(Range&& range) : Bbox() {
    using std::begin, std::end;
    auto b = begin(range), e = end(range);
    for (; b != e; ++b) union_with(*b);
  }

  void clear() {
    (*this)[0] = PointD::all(std::numeric_limits<T>::max());
    (*this)[1] = PointD::all(std::numeric_limits<T>::lowest());
  }
  void infinite() {
    (*this)[0] = PointD::all(std::numeric_limits<T>::lowest());
    (*this)[1] = PointD::all(std::numeric_limits<T>::max());
  }

  void union_with(const type& bbox) {
    auto& self = *this;
    for_int(c, dim) {
      if (bbox[0][c] < self[0][c]) self[0][c] = bbox[0][c];
      if (bbox[1][c] > self[1][c]) self[1][c] = bbox[1][c];
    }
  }
  void union_with(const PointD& pp) {
    auto& self = *this;
    for_int(c, dim) {
      T v = pp[c];
      if (v < self[0][c]) self[0][c] = v;
      if (v > self[1][c]) self[1][c] = v;
    }
  }

  void intersect(const type& bbox) {
    auto& self = *this;
    for_int(c, dim) {
      if (bbox[0][c] > self[0][c]) self[0][c] = bbox[0][c];
      if (bbox[1][c] < self[1][c]) self[1][c] = bbox[1][c];
    }
  }

  bool inside(const type& bbox) const {
    auto& self = *this;
    for_int(c, dim) {
      if (self[0][c] < bbox[0][c]) return false;
      if (self[1][c] > bbox[1][c]) return false;
    }
    return true;
  }

  bool overlap(const type& bbox) const {
    auto& self = *this;
    for_int(c, dim) {
      if (bbox[0][c] > self[1][c] || bbox[1][c] < self[0][c]) return false;
    }
    return true;
  }

  T max_side() const {
    auto& self = *this;
    return max(self[1] - self[0]);
  }

  friend type bbox_union(const type& bbox1, const type& bbox2) {
    type bbox = bbox1;
    bbox.union_with(bbox2);
    return bbox;
  }

  friend std::ostream& operator<<(std::ostream& os, const type& bbox) {
    return os << "Bbox{" << bbox[0] << ", " << bbox[1] << "}";
  }

  // ** Functions only for dim == 3:

  // Uniform scaling into unit cube, centered on x & y, rest at z == 0.
  template <int D = dim, typename = std::enable_if_t<D == 3>> Frame get_frame_to_cube() const {
    auto& self = *this;
    Vector di = self[1] - self[0];
    float maxdi = max(di);
    assertx(maxdi);
    Vector center;
    for_int(c, 3) center[c] = (1.f - di[c] / maxdi) * .5f;
    center[2] = 0.f;  // objects lie at bottom of cube
    return Frame::translation(-to_Vector(self[0])) * Frame::scaling(thrice(1.f / maxdi)) * Frame::translation(center);
  }

  template <int D = dim, typename = std::enable_if_t<D == 3>>
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

  template <int D = dim, typename = std::enable_if_t<D == 3>> [[nodiscard]] type transform(const Frame& frame) const {
    type bbox;
    auto& self = *this;
    for_int(i0, 2) for_int(i1, 2) for_int(i2, 2) {
      Point corner(i0 ? self[1][0] : self[0][0], i1 ? self[1][1] : self[0][1], i2 ? self[1][2] : self[0][2]);
      bbox.union_with(corner * frame);
    }
    return bbox;
  }
};

// Template deduction guides:
template <typename T, int n> Bbox(const Vec<T, n>&, const Vec<T, n>&) -> Bbox<T, n>;
template <typename T, int n> Bbox(const Vec2<Vec<T, n>>&) -> Bbox<T, n>;

template <typename Range, typename = enable_if_range_t<Range>, typename VecT = range_value_t<Range>>
Bbox(Range&& range) -> Bbox<typename VecT::value_type, VecT::Num>;

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BBOX_H_
