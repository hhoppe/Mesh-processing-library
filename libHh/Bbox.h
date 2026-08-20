// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BBOX_H_
#define MESH_PROCESSING_LIBHH_BBOX_H_

#include "libHh/Geometry.h"

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
  constexpr Bbox(const type& bbox) : Bbox(bbox[0], bbox[1]) {}
  template <ranges::input_range R> requires std::convertible_to<ranges::range_reference_t<R>, PointD>
  explicit Bbox(R&& range) : Bbox() {
    for (const auto& e : range) union_with(e);
  }
  type& operator=(const type&) = default;

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
      const T v = pp[c];
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

  [[nodiscard]] constexpr bool inside(const type& bbox) const {
    auto& self = *this;
    for_int(c, dim) {
      if (self[0][c] < bbox[0][c]) return false;
      if (self[1][c] > bbox[1][c]) return false;
    }
    return true;
  }

  [[nodiscard]] constexpr bool overlap(const type& bbox) const {
    const auto& self = *this;
    for_int(c, dim) if (bbox[0][c] > self[1][c] || bbox[1][c] < self[0][c]) return false;
    return true;
  }

  [[nodiscard]] constexpr T max_side() const {
    const auto& self = *this;
    return max(self[1] - self[0]);
  }

  [[nodiscard]] friend type bbox_union(const type& bbox1, const type& bbox2) {
    type bbox = bbox1;
    bbox.union_with(bbox2);
    return bbox;
  }

  // Return enclosing (centered) bbox that has all sides equal.
  [[nodiscard]] Bbox enclosing_hypercube() const requires std::is_floating_point_v<T> {
    const auto& self = *this;
    const PointD diagonal = self[1] - self[0];
    const T max_side = max(diagonal);
    const PointD offset = (max_side - diagonal) * T(0.5f);
    return Bbox(self[0] - offset, self[1] + offset);
  }

  friend std::ostream& operator<<(std::ostream& os, const type& bbox) {
    return os << "Bbox{" << bbox[0] << ", " << bbox[1] << "}";
  }

  // ** Functions only for dim == 3 && std::same_as<T, float>:

  // Uniform scaling into unit cube, centered on x & y, resting at z == 0.
  [[nodiscard]] Frame get_frame_to_cube() const requires(dim == 3 && std::same_as<T, float>) {
    const auto& self = *this;
    const Vector diagonal = self[1] - self[0];
    const float max_side = max(diagonal);
    assertx(max_side);
    Vector center;
    for_int(c, 3) center[c] = (1.f - diagonal[c] / max_side) * .5f;
    center[2] = 0.f;  // Let the object lie at the bottom of the cube.
    return Frame::translation(-self[0]) * Frame::scaling(thrice(1.f / max_side)) * Frame::translation(center);
  }

  [[nodiscard]] Frame get_frame_to_small_cube(float cubesize = .8f) const requires(dim == 3 && std::same_as<T, float>)
  {
    Frame frame = get_frame_to_cube();
    const float bnd = (1.f - cubesize) / 2.f;
    frame = frame * Frame::scaling(thrice(cubesize)) * Frame::translation(thrice(bnd));
    for_int(i, 3) {
      if (abs(frame[i][i] - 1.f) < .05f) frame[i][i] = 1.f;
      if (abs(frame[3][i]) < .05f) frame[3][i] = 0.f;
    }
    return frame;
  }

  [[nodiscard]] type transform(const Frame& frame) const requires(dim == 3 && std::same_as<T, float>) {
    const auto& self = *this;
    type bbox;
    for_int(i0, 2) for_int(i1, 2) for_int(i2, 2) {
      Point corner(i0 ? self[1][0] : self[0][0], i1 ? self[1][1] : self[0][1], i2 ? self[1][2] : self[0][2]);
      bbox.union_with(corner * frame);
    }
    return bbox;
  }
};

// Template deduction guides:a
template <typename T, int n> Bbox(const Vec<T, n>&, const Vec<T, n>&) -> Bbox<T, n>;
template <typename T, int n> Bbox(const Vec2<Vec<T, n>>&) -> Bbox<T, n>;
template <ranges::input_range R>
Bbox(R&&) -> Bbox<typename ranges::range_value_t<R>::value_type, ranges::range_value_t<R>::Num>;

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BBOX_H_
