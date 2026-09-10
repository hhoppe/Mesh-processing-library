// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_NETWORKORDER_H_
#define MESH_PROCESSING_LIBHH_NETWORKORDER_H_

#include <bit>  // bit_cast(), byteswap(), std::endian.

#include "libHh/Hh.h"

namespace hh {

// Convert between native byte ordering and network byte ordering.
//
// Notes:
// - Network order is Big Endian (MSB first).  That is the convention used here for all binary files.
//
// - Intel_x86 and VAX are Little Endian
// - RISC is mostly Big Endian.  ARM is both.  All ARM versions of Windows run Little Endian

// The Internet Protocol defines big-endian as the standard network byte order used for all numeric values in
//  the packet headers and for many higher level protocols and file formats that are designed for use over IP.

// Big Endian is natural for dates/times (2014-12-22 12:34:56).
// It is also used for grid access (matrix[y, x] == matrix[yx]; matrix.dims() == V(matrix.ysize(), matrix.xsize()))
//   and for screen coordinates (const Vec2<int>& yx).

template <typename T> void my_swap_bytes(T* p) {
  if constexpr (sizeof(T) == 8) {
    *p = std::bit_cast<T>(std::byteswap(std::bit_cast<uint64_t>(*p)));
  } else if constexpr (sizeof(T) == 4) {
    *p = std::bit_cast<T>(std::byteswap(std::bit_cast<uint32_t>(*p)));
  } else if constexpr (sizeof(T) == 2) {
    *p = std::bit_cast<T>(std::byteswap(std::bit_cast<uint16_t>(*p)));
  } else {
    static_assert(sizeof(T) != sizeof(T), "Unsupported type size");  // (Delay evaluation until instantiation.)
  }
}

constexpr bool k_is_big_endian = std::endian::native == std::endian::big;

// Convert from native to network order.
template <typename T> void to_std(T* p) {
  if (!k_is_big_endian) my_swap_bytes(p);
}

// Convert from network order to native order.
template <typename T> void from_std(T* p) { to_std(p); }

// Convert from to native order to DOS order.
template <typename T> void to_dos(T* p) {
  if (k_is_big_endian) my_swap_bytes(p);
}

// Convert from DOS order to native order.
template <typename T> void from_dos(T* p) { to_dos(p); }

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_NETWORKORDER_H_
