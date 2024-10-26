// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ADVANCED_H_
#define MESH_PROCESSING_LIBHH_ADVANCED_H_

#include "libHh/Hh.h"

namespace hh {

// Return a reference to a unique pointer object.  This reference may be invalid if the pointer is null!
// auto up_p = b ? make_unique<Point>(1.f, 2.f, 3.f) : nullptr;  Point& p = optional_reference(up_p);
template <typename T> T& optional_reference(const std::unique_ptr<T>& up) {
  return up ? *up : *implicit_cast<T*>(nullptr);
}

// Create a copy; make sure not to call on CArrayView, ArrayView, *View, etc. --- instead use Array<T>(v), etc.
template <typename T> std::decay_t<T> clone(T&& v) {
  // First, verify that T is default-constructible;
  //  my exotic classes like ArrayView and GridView are not default-constructible and should not be cloned.
  static_assert(std::is_default_constructible_v<std::decay_t<T>>);  // or maybe (void(T{}));
  // if r-value reference, return moved object;
  // else it is an l-value reference (const or not) and return a copy.
  // return std::is_rvalue_reference_v<T> ? std::move(v) : std::decay_t<T>(v);
  return std::decay_t<T>(std::forward<T>(v));  // std::decay_t<T> cast is necessary for explicit constructor
}
// Note: the following would lead to an extra move-construction if v is an rvalue
//    (see https://stackoverflow.com/questions/16724657/why-do-we-copy-then-move )
// Also, std::move is redundant?  see https://vmpstr.blogspot.com/2015/12/redundant-stdmove.html
// template<typename T> T clone(T v) { return std::move(v); }

// e.g.:  unroll<6>([&](int j) { _a[j] = min(l._a[j], r._a[j]); });
namespace details {
template <int i, int n> struct unroll_aux {
  template <typename Func = void(int)> void operator()(Func func) const {
    func(i);
    unroll_aux<i + 1, n>()(func);
  }
};
template <int n> struct unroll_aux<n, n> {
  template <typename Func = void(int)> void operator()(Func) const {}
};
template <int n> struct unroll_aux<n, -1> {  // awkward, avoid infinite recursive if n == -1 in addition to n == 0
  template <typename Func = void(int)> void operator()(Func) const {}
};
}  // namespace details
template <int n, typename Func = void(int)> void unroll(Func func) { details::unroll_aux<0, n>()(func); }

template <int n, int nmax, typename Func = void(int)> void unroll_max(Func func) {
  if (n <= nmax) {
    unroll<n>(func);
  } else {
    for_int(i, n) func(i);
  }
}

// Convenience function for hashing.
template <typename T> size_t my_hash(const T& v) { return std::hash<T>()(v); }

#if 0  // Adapted from older version of boost; may be intended for 32-bit size_t.

template <typename T> size_t hash_combine(size_t seed, const T& v) {
  return seed ^ (std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2));
}

#else  // Adapted from newer version of boost.

namespace details {

template <std::size_t Bits> struct boost_hash_mix_impl;

template <> struct boost_hash_mix_impl<64> {
  inline static uint64_t fn(uint64_t x) {
    uint64_t const m = 0xe9846af9b1a615d;
    x ^= x >> 32;
    x *= m;
    x ^= x >> 32;
    x *= m;
    x ^= x >> 28;
    return x;
  }
};

template <> struct boost_hash_mix_impl<32> {
  inline static uint32_t fn(uint32_t x) {
    uint32_t const m1 = 0x21f0aaad;
    uint32_t const m2 = 0x735a2d97;
    x ^= x >> 16;
    x *= m1;
    x ^= x >> 15;
    x *= m2;
    x ^= x >> 15;
    return x;
  }
};

inline std::size_t boost_hash_mix(std::size_t v) { return boost_hash_mix_impl<sizeof(std::size_t) * 8>::fn(v); }

inline void boost_hash_combine_size_t(std::size_t& seed, std::size_t value) {
  seed = boost_hash_mix(seed + 0x9e3779b9 + value);
}

}  // namespace details

template <typename T> size_t hash_combine(size_t seed, const T& v) {
  details::boost_hash_combine_size_t(seed, std::hash<T>()(v));
  return seed;
}

#endif

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_ADVANCED_H_
