// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/BinaryIO.h"

#include <cstdio>   // fopen()
#include <sstream>  // std::ostringstream

#include "libHh/Array.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

// The bytes of a string, in hexadecimal.
string hex_bytes(const string& s) {
  string result;
  for (const char ch : s) result += sform(" %02X", unsigned{uint8_t(ch)});
  return result;
}

}  // namespace

int main() {
  {
    // Network order is big-endian, independent of the platform.
    const Array<uint16_t> ar1{0x1234, 0xABCD};
    std::ostringstream oss;
    assertx(write_binary_std(oss, ar1));
    SHOW(hex_bytes(oss.str()));
    std::istringstream iss(oss.str());
    Array<uint16_t> ar2(2);
    assertx(read_binary_std(iss, ar2));
    assertx(ar2 == ar1);
    SHOW(ar1, ar2);
  }
  {
    const Vec<float, 2> vec1(1.5f, -2.f);
    std::ostringstream oss;
    assertx(write_binary_std(oss, vec1));
    SHOW(hex_bytes(oss.str()));
    std::istringstream iss(oss.str());
    Vec<float, 2> vec2;
    assertx(read_binary_std(iss, vec2));
    assertx(vec2 == vec1);
  }
  {
    // The raw functions preserve the native byte order, so a round trip reproduces the values.
    const Array<int> ar1{1, -2, 1'000'000};
    std::ostringstream oss;
    assertx(write_binary_raw(oss, ar1));
    assertx(oss.str().size() == 3 * sizeof(int));
    std::istringstream iss(oss.str());
    Array<int> ar2(3);
    assertx(read_binary_raw(iss, ar2));
    assertx(ar2 == ar1);
    // Reading past the end of the data fails.
    Array<int> ar3(1);
    assertx(!read_binary_raw(iss, ar3));
  }
  {
    // The same through a FILE*.
    const string filename = "BinaryIO_test.tmp";
    const Array<double> ar1{1.25, -3.5, 1e100};
    {
      FILE* file = assertx(std::fopen(filename.c_str(), "wb"));
      assertx(write_raw(file, ar1));
      assertx(!std::fclose(file));
    }
    {
      FILE* file = assertx(std::fopen(filename.c_str(), "rb"));
      Array<double> ar2(3);
      assertx(read_raw(file, ar2));
      assertx(ar2 == ar1);
      assertx(!read_raw(file, ar2));  // At the end of the file.
      assertx(!std::fclose(file));
    }
    assertx(!std::remove(filename.c_str()));
  }
}
