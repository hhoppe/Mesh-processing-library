// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_UNIV_H_
#define MESH_PROCESSING_LIBHH_UNIV_H_

#include <bit>  // bit_cast().

#include "libHh/Hh.h"

namespace hh {

// Univ is my universal fundamental type, essentially a union of: { T*, int, unsigned, float } .
// It is not used much anymore.

class SUniv;
using Univ = SUniv*;

static_assert(sizeof(Univ) >= sizeof(int));
static_assert(sizeof(Univ) >= sizeof(float));

// Templated helper class for conversion to/from Univ.
template <typename T> struct Conv;

template <typename T> struct Conv<T*> {
  [[nodiscard]] static constexpr Univ e(T* v) { return Univ(const_cast<std::remove_const_t<T>*>(v)); }  // Encode.
  [[nodiscard]] static constexpr T* d(Univ v) { return reinterpret_cast<T*>(v); }                       // Decode.
};

template <> struct Conv<int> {
  [[nodiscard]] static Univ e(int v) { return Univ(intptr_t{v}); }
  [[nodiscard]] static int d(Univ v) { return narrow_cast<int>(reinterpret_cast<intptr_t>(v)); }
};

template <> struct Conv<unsigned> {
  [[nodiscard]] static Univ e(unsigned v) { return Univ(uintptr_t{v}); }
  [[nodiscard]] static unsigned d(Univ v) { return narrow_cast<unsigned>(reinterpret_cast<uintptr_t>(v)); }
};

template <> struct Conv<float> {
  [[nodiscard]] static Univ e(float v) { return Conv<unsigned>::e(std::bit_cast<uint32_t>(v)); }
  [[nodiscard]] static float d(Univ v) { return std::bit_cast<float>(narrow_cast<uint32_t>(Conv<unsigned>::d(v))); }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_UNIV_H_
