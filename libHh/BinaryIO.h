// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BINARYIO_H_
#define MESH_PROCESSING_LIBHH_BINARYIO_H_

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
template <typename T> std::istream& read_binary_raw(std::istream& is, ArrayView<T> ar) {
  return is.read(reinterpret_cast<char*>(ar.data()), ar.num() * sizeof(T));
}

// Read an array of elements and perform Endian conversion from network (Big Endian) to native order.
template <typename T> std::istream& read_binary_std(std::istream& is, ArrayView<T> ar) {
  if (read_binary_raw(is, ar)) for_int(i, ar.num()) from_std(&ar[i]);
  return is;
}

// Write an array of elements without any Endian byte-reordering.
template <typename T> std::ostream& write_binary_raw(std::ostream& os, CArrayView<T> ar) {
  return os.write(reinterpret_cast<const char*>(ar.data()), ar.num() * sizeof(T));
}

// Write an array of elements after Endian conversion from native to network (Big Endian) order.
template <typename T> std::ostream& write_binary_std(std::ostream& os, CArrayView<T> ar) {
  Array<T> ar2(ar);  // copy is slow?
  for_int(i, ar2.num()) to_std(&ar2[i]);
  return write_binary_raw(os, ar2);
}

// Read an array of elements without any Endian byte-reordering.  Ret: success.
template <typename T> [[nodiscard]] bool read_raw(FILE* file, ArrayView<T> ar) {
  return fread(ar.data(), ar.num() * sizeof(T), 1, file) == 1;
}

// Write an array of elements without any Endian byte-reordering.  Ret: success.
template <typename T> [[nodiscard]] bool write_raw(FILE* file, CArrayView<T> ar) {
  return fwrite(ar.data(), ar.num() * sizeof(T), 1, file) == 1;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BINARYIO_H_
