// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Vector4i.h"

#include "libHh/Array.h"
#include "libHh/RangeOp.h"  // sum()
using namespace hh;

namespace {

// Deterministic pseudo-random integers in [-16384, 16383], identical on all platforms.
struct Lcg {
  uint32_t state = 1;
  int operator()() {
    state = state * 1'664'525u + 1'013'904'223u;
    return int(state >> 17) - 16384;
  }
};

// Compare the vectorized operations with scalar int arithmetic.  The values are small enough to avoid any
// overflow, which would be undefined behavior in the scalar implementation.
void test_consistency() {
  Lcg lcg;
  int num_arith = 0, num_scalar = 0, num_min_max = 0, num_bits = 0, num_shift = 0, num_unary = 0;
  for_int(iter, 10'000) {
    const Vector4i a(lcg(), lcg(), lcg(), lcg()), b(lcg(), lcg(), lcg(), lcg());
    const int i = lcg() / 128;  // Here |i| <= 128.
    const int n = int(unsigned(lcg()) % 16u);
    const Vector4i sum_ab = a + b, diff_ab = a - b, prod_ab = a * b, sum_i = a + i, diff_i = a - i;
    const Vector4i prod_i = a * i, prod_i2 = i * a, vmin = min(a, b), vmax = max(a, b);
    const Vector4i vand = a & b, vor = a | b, vxor = a ^ b, vshl = a << n, vshr = a >> n, neg = -a, vabs = abs(a);
    for_int(c, 4) {
      num_arith += sum_ab[c] != a[c] + b[c] || diff_ab[c] != a[c] - b[c] || prod_ab[c] != a[c] * b[c];
      num_scalar += sum_i[c] != a[c] + i || diff_i[c] != a[c] - i || prod_i[c] != a[c] * i || prod_i2[c] != i * a[c];
      num_min_max += vmin[c] != min(a[c], b[c]) || vmax[c] != max(a[c], b[c]);
      num_bits += vand[c] != (a[c] & b[c]) || vor[c] != (a[c] | b[c]) || vxor[c] != (a[c] ^ b[c]);
      num_shift += vshl[c] != a[c] * (1 << n) || vshr[c] != (a[c] >> n);  // Arithmetic right shift.
      num_unary += neg[c] != -a[c] || vabs[c] != std::abs(a[c]);
    }
  }
  SHOW(num_arith, num_scalar, num_min_max, num_bits, num_shift, num_unary);
}

// Element access, loads and stores, and the compound assignment operators.
void test_interface() {
  const Vector4i v1(1, 2, 3, 4), v2(8, 7, 6, 5);
  assertx(v1.size() == 4 && Vector4i::ok(3) && !Vector4i::ok(4) && !Vector4i::ok(-1));
  {
    const Vector4i v0{};  // Value-initialized to zero.
    for_int(c, 4) assertx(v0[c] == 0);
    Vector4i v = v1;
    v[3] = -9;
    assertx(v.data()[3] == -9 && v1[3] == 4);
  }
  {
    alignas(16) int ar[4];
    v2.store_aligned(ar);
    assertx(ar[0] == 8 && ar[3] == 5);
    Vector4i v;
    v.load_aligned(ar);
    for_int(c, 4) assertx(v[c] == v2[c]);
  }
  {
    Vector4i v = v1;
    v += v2;  // [9, 9, 9, 9]
    v -= v1;  // [8, 7, 6, 5]
    v *= v1;  // [8, 14, 18, 20]
    SHOW(v);
    v += 2;
    v -= 4;
    v *= 3;
    SHOW(v);
  }
}

}  // namespace

int main() {
  {
    Vector4i v1(1, 2, 3, 4), v2(8, 7, 6, 5);
    SHOW(v1);
    SHOW(v2);
    SHOW(v1[2]);
    SHOW(v2[0]);
    SHOW(v2.with(1, 17));
    SHOW(-v2);
    for (int j : v2) SHOW(j);
    {
      Vector4i v2copy(v2);
      SHOW(v2copy);
    }
    SHOW(v1 + v2);
    SHOW(v1 - v2);
    SHOW(v1 * v2);
    SHOW(v2 * 2);
    SHOW(2 * v2);
    SHOW(v1 - v2 - v1 + v2 * 2);
    SHOW(sum(v2));
    SHOW(v1 * 4);
    SHOW(min(v1 * 4, v2));
    SHOW(max(v1 * 4, v2));
    SHOW(v1 | v2);
    SHOW(v1 & v2);
    SHOW(v1 ^ v2);
    SHOW(v1 << 3);
    SHOW(v2 >> 1);
    SHOW(abs(v2));
    SHOW(abs(-v2));
    SHOW(abs(v1 - v2));
    SHOW(sizeof(v1));
  }
  {
    Vector4i v1(1, 2, 3, 4);
    v1.fill(5);
    SHOW(v1);
  }
  {
    Pixel pixel(11, 0, 255, 1);
    SHOW(pixel);
    SHOW(Vector4i(pixel));
    SHOW(Vector4i(pixel).pixel());
    SHOW((Vector4i(pixel) + 15).pixel());
    SHOW((Vector4i(pixel) + 250).pixel());
    SHOW((Vector4i(pixel) + 100'000).pixel());
    SHOW((Vector4i(pixel) + (std::numeric_limits<int>::max() - 255)).pixel());
    // (Any int overflow is undefined behavior in the scalar implementations, although SSE wraps around.)
    SHOW((Vector4i(pixel) - 1).pixel());
    SHOW((Vector4i(pixel) - 15).pixel());
    SHOW((Vector4i(pixel) - 100'000).pixel());
    SHOW((Vector4i(pixel) - std::numeric_limits<int>::max()).pixel());
  }
  {
    Array<int> ar1{31, std::numeric_limits<int>::max(), std::numeric_limits<int>::min() + 1, 0};
    SHOW(ar1);
    Vector4i v;
    v.load_unaligned(ar1.data());
    SHOW(v);
    (v - 1).store_unaligned(ar1.data());
    SHOW(ar1);
  }
  {
    Vec4<int> ar1{31, std::numeric_limits<int>::max(), std::numeric_limits<int>::min() + 1, 0};
    SHOW(ar1);
    Vector4i v;
    v.load_unaligned(ar1.data());
    SHOW(v);
    (v - 1).store_unaligned(ar1.data());
    SHOW(ar1);
  }
  test_consistency();
  test_interface();
}
