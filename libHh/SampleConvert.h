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

namespace details {

// The scalar type at the leaves of a sample type (uint8_t for Pixel).
template <typename T> struct SampleScalar {
  using type = T;
};
template <typename T, int N> struct SampleScalar<Vec<T, N>> {
  using type = SampleScalar<T>::type;
};
template <> struct SampleScalar<Pixel> {
  using type = uint8_t;
};

}  // namespace details

// Decodes sample values of type T to their linear floating-point values; it holds the lookup table of the 8- and
// 16-bit integer types so that a grid is decoded without fetching the table for each element.
template <typename T> class Decoder {
 public:
  explicit Decoder(const Gamma& gamma) : _gamma(gamma) {
    if constexpr (std::is_same_v<Scalar, uint8_t>) _table = gamma.decode_table_uint8();
    if constexpr (std::is_same_v<Scalar, uint16_t>) _table = gamma.decode_table_uint16();
  }
  [[nodiscard]] sample_float_t<T> operator()(const T& v) const { return decode(v); }

 private:
  using Scalar = details::SampleScalar<T>::type;
  const Gamma& _gamma;
  CArrayView<float> _table{nullptr, 0};  // The decoded 8- or 16-bit integer sample values, if T holds those.
  [[nodiscard]] float decode(uint8_t v) const { return _table[v]; }
  [[nodiscard]] float decode(uint16_t v) const { return _table[v]; }
  [[nodiscard]] double decode(uint32_t v) const { return _gamma.decode(v / 4294967295.); }
  [[nodiscard]] float decode(float v) const { return _gamma.is_identity() ? v : _gamma.decode(v); }
  [[nodiscard]] double decode(double v) const { return _gamma.is_identity() ? v : _gamma.decode(v); }
  template <typename U, int N> [[nodiscard]] Vec<sample_float_t<U>, N> decode(const Vec<U, N>& v) const {
    Vec<sample_float_t<U>, N> r;
    for_int(c, N) r[c] = decode(v[c]);
    return r;
  }
  [[nodiscard]] Vector4 decode(const Vector4& v) const {
    if (_gamma.is_identity()) return v;
    return Vector4(_gamma.decode(v[0]), _gamma.decode(v[1]), _gamma.decode(v[2]), _gamma.decode(v[3]));
  }
  [[nodiscard]] Vector4 decode(const Pixel& v) const {
    if (_gamma.is_identity()) return Vector4(v);  // (The same conversion as convert().)
    return Vector4(_table[v[0]], _table[v[1]], _table[v[2]], _table[v[3]]);
  }
};

// Decode a sample value to its linear floating-point value.
template <typename T> [[nodiscard]] sample_float_t<T> decode_value(const Gamma& gamma, const T& v) {
  return Decoder<T>(gamma)(v);
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
  const Decoder<T> decoder(gamma);
  parallel_for({.cycles_per_elem = 8}, range(grid.size()),
               [&](const size_t i) { result.flat(i) = decoder(grid.flat(i)); });
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
