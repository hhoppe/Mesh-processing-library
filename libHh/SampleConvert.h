// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_SAMPLECONVERT_H_
#define MESH_PROCESSING_LIBHH_SAMPLECONVERT_H_

#include <limits>

#include "libHh/Gamma.h"
#include "libHh/Grid.h"
#include "libHh/Parallel.h"
#include "libHh/Pixel.h"
#include "libHh/Vector4.h"

namespace hh {

// Conversions between the sample types stored in grids and the floating-point values that filtering operates on,
// through a component transfer function (Gamma): integer samples span [0, 1] after decoding.

namespace details {

template <typename T> struct SampleFloat {
  using type = T;  // Floating-point samples are filtered as they are.
};
template <> struct SampleFloat<uint8_t> {
  using type = float;
};
template <> struct SampleFloat<uint16_t> {
  using type = float;
};
template <> struct SampleFloat<uint32_t> {
  using type = double;
};
template <typename T, int N> struct SampleFloat<Vec<T, N>> {
  using type = Vec<typename SampleFloat<T>::type, N>;
};
template <> struct SampleFloat<Pixel> {
  using type = Vector4;
};

}  // namespace details

// The floating-point type in which samples of type T are filtered: T itself for floating-point types, float for
// 8- and 16-bit unsigned integers, double for 32-bit ones (elementwise for Vec), and Vector4 for Pixel.
template <typename T> using sample_float_t = details::SampleFloat<T>::type;

// True if T is an integer sample type (so that its values must be decoded and encoded).
template <typename T> constexpr bool is_integer_sample_v = !std::is_same_v<sample_float_t<T>, T>;

// Decode a sample value to its linear floating-point value.
[[nodiscard]] inline float decode_value(const Gamma& gamma, uint8_t v) { return gamma.decode_table_uint8()[v]; }
[[nodiscard]] inline float decode_value(const Gamma& gamma, uint16_t v) { return gamma.decode_table_uint16()[v]; }
[[nodiscard]] inline double decode_value(const Gamma& gamma, uint32_t v) { return gamma.decode(v / 4294967295.); }
[[nodiscard]] inline float decode_value(const Gamma& gamma, float v) {
  return gamma.is_identity() ? v : gamma.decode(v);
}
[[nodiscard]] inline double decode_value(const Gamma& gamma, double v) {
  return gamma.is_identity() ? v : gamma.decode(v);
}
template <typename T, int N>
[[nodiscard]] Vec<sample_float_t<T>, N> decode_value(const Gamma& gamma, const Vec<T, N>& v) {
  Vec<sample_float_t<T>, N> r;
  for_int(c, N) r[c] = decode_value(gamma, v[c]);
  return r;
}
[[nodiscard]] inline Vector4 decode_value(const Gamma& gamma, const Pixel& v) {
  if (gamma.is_identity()) return Vector4(v);  // (The same conversion as convert().)
  const CArrayView<float> table = gamma.decode_table_uint8();
  return Vector4(table[v[0]], table[v[1]], table[v[2]], table[v[3]]);
}
[[nodiscard]] inline Vector4 decode_value(const Gamma& gamma, const Vector4& v) {
  return gamma.is_identity() ? v
                             : Vector4(gamma.decode(v[0]), gamma.decode(v[1]), gamma.decode(v[2]), gamma.decode(v[3]));
}

namespace details {

template <typename I, typename F> [[nodiscard]] I encode_integer(const Gamma& gamma, F v) {
  const F maxv = F(std::numeric_limits<I>::max());
  const F e = (gamma.is_identity() ? v : gamma.encode(v)) * maxv + F(.5);
  return I(e <= F(0) ? F(0) : e >= maxv ? maxv : e);  // (Also maps NaN to 0.)
}

}  // namespace details

// Encode a linear floating-point value to a sample value of type T (integer types: rounding and clamping).
template <typename T> [[nodiscard]] T encode_value(const Gamma& gamma, const sample_float_t<T>& v) {
  if constexpr (std::is_same_v<T, Pixel>) {
    if (gamma.is_identity()) return v.pixel();  // (The same rounding and clamping as convert().)
    return Vector4(gamma.encode(v[0]), gamma.encode(v[1]), gamma.encode(v[2]), gamma.encode(v[3])).pixel();
  } else if constexpr (std::is_same_v<T, Vector4>) {
    if (gamma.is_identity()) return v;
    return Vector4(gamma.encode(v[0]), gamma.encode(v[1]), gamma.encode(v[2]), gamma.encode(v[3]));
  } else if constexpr (std::is_floating_point_v<T>) {
    return gamma.is_identity() ? v : gamma.encode(v);
  } else if constexpr (std::is_integral_v<T>) {
    return details::encode_integer<T>(gamma, v);
  } else {  // Vec<U, N>, elementwise.
    T r;
    for_int(c, T::Num) r[c] = encode_value<typename T::value_type>(gamma, v[c]);
    return r;
  }
}

// Decode a grid of samples into a grid of linear values.
template <int D, typename T>
[[nodiscard]] Grid<D, sample_float_t<T>> decode(const Gamma& gamma, CGridView<D, T> grid) {
  Grid<D, sample_float_t<T>> result(grid.dims());
  parallel_for({.cycles_per_elem = 8}, range(grid.size()),
               [&](const size_t i) { result.flat(i) = decode_value(gamma, grid.flat(i)); });
  return result;
}

// Encode a grid of linear values into a grid of samples.
template <int D, typename T>
void encode(const Gamma& gamma, CGridView<D, sample_float_t<T>> grid, GridView<D, T> out) {
  assertx(same_size(grid, out));
  parallel_for({.cycles_per_elem = 8}, range(grid.size()),
               [&](const size_t i) { out.flat(i) = encode_value<T>(gamma, grid.flat(i)); });
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SAMPLECONVERT_H_
