// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/NetworkOrder.h"

#include <array>
#include <cstring>  // memcpy()

#include "libHh/Array.h"
#include "libHh/BinaryIO.h"
#include "libHh/FileIO.h"
#include "libHh/Matrix.h"
#include "libHh/Stat.h"
using namespace hh;

namespace {

// Returns the bytes of a value in its in-memory order.
template <typename T> std::array<uint8_t, sizeof(T)> bytes_of(const T& value) {
  std::array<uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
  return bytes;
}

// Verifies that to_std() stores the value most significant byte first and to_dos() least significant byte first,
// and that from_std() and from_dos() invert them.  The value's bytes must not form a palindrome.
template <typename T> void test_byte_orders(T value, uint64_t bits) {
  T v = value;
  to_std(&v);
  const auto std_bytes = bytes_of(v);
  for_int(i, int(sizeof(T))) assertx(std_bytes[i] == uint8_t(bits >> (8 * (sizeof(T) - 1 - i))));
  from_std(&v);
  assertx(bytes_of(v) == bytes_of(value));
  to_dos(&v);
  const auto dos_bytes = bytes_of(v);
  for_int(i, int(sizeof(T))) assertx(dos_bytes[i] == uint8_t(bits >> (8 * i)));
  from_dos(&v);
  assertx(bytes_of(v) == bytes_of(value));
  // Swapping twice is the identity.
  my_swap_bytes(&v);
  if (sizeof(T) > 1) assertx(bytes_of(v) != bytes_of(value));
  my_swap_bytes(&v);
  assertx(bytes_of(v) == bytes_of(value));
}

}  // namespace

int main() {
  {
    test_byte_orders(uint16_t{0x0102}, 0x0102);
    test_byte_orders(int16_t{-2}, 0xFFFE);
    test_byte_orders(uint32_t{0x01020304}, 0x01020304);
    test_byte_orders(int32_t{-0x01020304}, uint32_t{0xFEFDFCFC});
    test_byte_orders(uint64_t{0x0102030405060708}, 0x0102030405060708);
    test_byte_orders(int64_t{-2}, 0xFFFFFFFFFFFFFFFE);
    test_byte_orders(1.5f, std::bit_cast<uint32_t>(1.5f));  // 0x3FC00000.
    test_byte_orders(-1.25, std::bit_cast<uint64_t>(-1.25));
    assertx(k_is_big_endian == (std::endian::native == std::endian::big));
  }
  {
    // The bytes 0x3F 0x80 0x00 0x00 in network order represent the float 1.f.
    float f;
    const std::array<uint8_t, 4> network_bytes{0x3F, 0x80, 0x00, 0x00};
    std::memcpy(&f, network_bytes.data(), sizeof(f));
    from_std(&f);
    SHOW(f);
    uint16_t u = 0x1234;
    to_std(&u);
    SHOW(int(bytes_of(u)[0]), int(bytes_of(u)[1]));
  }
  {
    const string ter_grid = "NetworkOrder_test.inp";
    RFile fi(ter_grid);
    int gridx, gridy;
    float fx;
    read_binary_std(fi(), ArView(fx));
    gridx = int(fx);
    float fy;
    read_binary_std(fi(), ArView(fy));
    gridy = int(fy);
    SHOW(gridx, gridy);
    assertx(gridx >= 4 && gridy >= 4);
    Matrix<float> ggridf(gridx, gridy);
    assertx(read_binary_std(fi(), ggridf.array_view()));
    HH_RSTAT(Sgrid, ggridf);
  }
}
