// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BINARYIO_H_
#define MESH_PROCESSING_LIBHH_BINARYIO_H_

#include <ranges>
#include <span>

#include "libHh/Array.h"
#include "libHh/NetworkOrder.h"

#if 0
{
  Array<ushort> ar1(20);
  write_binary_std(std::cout, ar1);

  Vec<float, 8> ar2;
  read_binary_raw(std::cin, ar2);

  Grid<2, int> grid(V(3, 2));
  write_binary_std(std::cout, grid.array_view());
}
#endif

namespace hh {

// Read an array of elements without any Endian byte-reordering.
template <std::ranges::contiguous_range R> std::istream& read_binary_raw(std::istream& is, R&& range) {
  const auto sp = std::span(range);
  return is.read(reinterpret_cast<char*>(sp.data()), sp.size_bytes());
}

// Write an array of elements without any Endian byte-reordering.
template <std::ranges::contiguous_range R> std::ostream& write_binary_raw(std::ostream& os, const R& range) {
  const auto sp = std::span(range);
  return os.write(reinterpret_cast<const char*>(sp.data()), sp.size_bytes());
}

// Read an array of elements and perform Endian conversion from network (Big Endian) to native order.
template <std::ranges::contiguous_range R> std::istream& read_binary_std(std::istream& is, R&& range) {
  if (read_binary_raw(is, range))
    for (auto& e : range) from_std(&e);
  return is;
}

// Write an array of elements after Endian conversion from native to network (Big Endian) order.
template <std::ranges::contiguous_range R> std::ostream& write_binary_std(std::ostream& os, const R& range) {
  Array array(range);  // Copy is slow?
  for (auto& e : array) to_std(&e);
  return write_binary_raw(os, array);
}

// Read an array of elements without any Endian byte-reordering.  Ret: success.
template <std::ranges::contiguous_range R> [[nodiscard]] bool read_raw(FILE* file, R&& range) {
  const auto sp = std::span(range);
  return fread(sp.data(), sp.size_bytes(), 1, file) == 1;
}

// Write an array of elements without any Endian byte-reordering.  Ret: success.
template <std::ranges::contiguous_range R> [[nodiscard]] bool write_raw(FILE* file, const R& range) {
  const auto sp = std::span(range);
  return fwrite(sp.data(), sp.size_bytes(), 1, file) == 1;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BINARYIO_H_
