// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ADVANCED_H_
#define MESH_PROCESSING_LIBHH_ADVANCED_H_

#include "libHh/Hh.h"

namespace hh {

// e.g.:  unroll<6>([&](int j) { _a[j] = min(l._a[j], r._a[j]); });
template <int n, typename Func> constexpr void unroll(Func func) {
  [&]<int... Is>(std::integer_sequence<int, Is...>) { (func(Is), ...); }(std::make_integer_sequence<int, n>());
}

template <int n, int nmax, typename Func = void(int)> void unroll_max(Func func) {
  if constexpr (n <= nmax) {
    unroll<n>(func);
  } else {
    for_int(i, n) func(i);
  }
}

// Convenience function for hashing.
template <typename T> [[nodiscard]] size_t my_hash(const T& v) { return std::hash<T>()(v); }

namespace details {

template <typename T = std::size_t> [[nodiscard]] T boost_hash_mix(T x) {  // Templated only to enable "if constexpr".
  static_assert(sizeof(std::size_t) == 4 || sizeof(std::size_t) == 8);
  if constexpr (sizeof(std::size_t) == 4) {
    uint32_t const m1 = 0x21f0aaad;
    uint32_t const m2 = 0x735a2d97;
    x ^= x >> 16;
    x *= m1;
    x ^= x >> 15;
    x *= m2;
    x ^= x >> 15;
  } else {
    uint64_t const m = 0xe9846af9b1a615d;
    x ^= x >> 32;
    x *= m;
    x ^= x >> 32;
    x *= m;
    x ^= x >> 28;
  }
  return x;
}

inline void boost_hash_combine_size_t(std::size_t& seed, std::size_t value) {
  seed = boost_hash_mix(seed + 0x9e3779b9 + value);
}

}  // namespace details

template <typename T> [[nodiscard]] size_t hash_combine(size_t seed, const T& v) {
  details::boost_hash_combine_size_t(seed, my_hash(v));
  return seed;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_ADVANCED_H_
