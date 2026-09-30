// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Vector4.h"

#include <iomanip>  // setprecision()

#include "libHh/Array.h"
#include "libHh/RangeOp.h"
#include "libHh/Vec.h"
using namespace hh;

// The SSE, NEON, and scalar implementations must all produce this same output; the NEON one is exercised on
// arm64 (e.g., macOS), and the scalar one with -DHH_NO_VECTOR4_VECTORIZATION.

namespace {

void to_norm(const Vector4& v) {
  SHOW(v);
  const Pixel pixel = v.pixel();
  Vec4<int> ar = convert<int>(pixel);
  SHOW(ar);
}

// Deterministic pseudo-random floats in [-8, 8), identical on all platforms.
struct Lcg {
  uint32_t state = 1;
  float operator()() {
    state = state * 1'664'525u + 1'013'904'223u;
    return float(int(state >> 8) - (1 << 23)) / float(1 << 20);
  }
};

// Compare the vectorized arithmetic with scalar float arithmetic, bit for bit.
void test_consistency() {
  Lcg lcg;
  int num_div = 0, num_div_scalar = 0, num_dot = 0, num_mul = 0;
  for_int(iter, 10'000) {
    const Vector4 a(lcg(), lcg(), lcg(), lcg());
    Vector4 b(lcg(), lcg(), lcg(), lcg());
    for_int(c, 4) if (b[c] == 0.f) b[c] = 1.f;
    const float f = b[0];
    const Vector4 quotient = a / b, quotient_scalar = a / f, product = a * b;
    for_int(c, 4) {
      num_div += quotient[c] != a[c] / b[c];
      num_div_scalar += quotient_scalar[c] != a[c] / f;
      num_mul += product[c] != a[c] * b[c];
    }
    // The volatile products prevent their contraction with the sums into fused multiply-adds, which
    // -ffp-contract=fast permits (although we have not observed it with gcc 15 or clang 21).
    const volatile float p0 = a[0] * b[0], p1 = a[1] * b[1], p2 = a[2] * b[2], p3 = a[3] * b[3];
    const float expected_dot = (p0 + p1) + (p2 + p3);
    num_dot += dot(a, b) != expected_dot;
  }
  SHOW(num_div, num_div_scalar, num_mul, num_dot);
}

// Conversions between floats and bytes.
void test_conversions() {
  int num_raw = 0, num_norm = 0;
  for_int(i, 256) {
    const Pixel pixel{uint8_t(i), uint8_t(255 - i), uint8_t(i / 2), uint8_t(255)};
    num_raw += to_Vector4_raw(pixel).raw_pixel() != pixel;
    num_norm += to_Vector4_norm(pixel).pixel() != pixel;
  }
  SHOW(num_raw, num_norm);
  // raw_to_byte4() truncates.
  SHOW(convert<int>(Vector4(0.999f, 1.f, 254.999f, 255.998f).raw_pixel()));
  // norm_to_byte4() rounds, with the exact tie 0.5f * 255.f == 127.5f rounded to the even 128; it clamps to [0, 255].
  SHOW(convert<int>(Vector4(0.5f, -0.f, -1e-8f, 1.f + 1e-6f).pixel()));
  // It rounds to the nearest integer (with ties to even) also at and next to each tie (k + 0.5) / 255.
  int num_tie = 0;
  for_int(k, 255) {
    const float x = (float(k) + .5f) / 255.f;
    for (const float v : {std::nextafter(x, 0.f), x, std::nextafter(x, 1.f)}) {
      const int expected = int(std::nearbyint(v * 255.f));
      num_tie += convert<int>(Vector4(v).pixel())[0] != expected;
    }
  }
  SHOW(num_tie);
}

}  // namespace

int main() {
  if (0) {
    // Setting the precision has no effect on SHOW() because it now uses a temporary std::ostringstream .
    std::cerr << std::setprecision(4) << std::setiosflags(std::ios::fixed);
    std::cerr.precision(4);
    std::cerr.setf(std::ios::fixed);
  }
  {
    SHOW(std::is_trivially_copyable_v<Vector4>);
    SHOW(std::is_trivially_default_constructible_v<Vector4>);
  }
  {
    const Vector4 v2{};
    assertx(is_zero(v2));
  }
  {
    Vector4 v1(1.f, 2.f, 3.f, 4.f), v2(8.f, 7.f, 6.f, 5.f);
    SHOW(v1);
    SHOW(v2);
    SHOW(v1 + v2);
    SHOW(v1 - v2);
    SHOW(v1 - v2 - v1 + v2 * 2.f);
    SHOW(dot(v1, v2));
    SHOW(mag2(v2));
    SHOW(dist2(v1, v2));
    SHOW(sum(v2));
    SHOW(min(v1, v2));
    SHOW(max(v1, v2));
    alignas(16) float ar1[4];
    v1.store_aligned(ar1);
    SHOW(ar1[0], ar1[3]);
    Vector4 v3;
    v3.load_aligned(ar1);
    SHOW(v3);
    Vector4 v4(v3);
    SHOW(v4);
    v4 += v2;
    SHOW(v4);
    SHOW(sizeof(v1));
    Vector4 va[2];
    SHOW(reinterpret_cast<uint8_t*>(&va[1]) - reinterpret_cast<uint8_t*>(&va[0]));
  }
  {
    const Vec4<uint8_t> ar0{uint8_t{23}, uint8_t{37}, uint8_t{45}, uint8_t{255}};
    const Vec4<uint8_t> ar1{uint8_t{12}, uint8_t{31}, uint8_t{37}, uint8_t{0}};
    SHOW(to_Vector4_raw(ar0));
    SHOW(to_Vector4_raw(ar1));
    SHOW(to_Vector4_norm(ar0));
    SHOW(to_Vector4_norm(ar1));
    Vec4<uint8_t> ar2;
    Vector4 v1 = to_Vector4_raw(ar0);
    // v1.raw_to_byte4(ar2);
    ar2 = v1.raw_pixel();
    SHOW(int(ar2[0]), int(ar2[1]), int(ar2[2]), int(ar2[3]));
    v1 = to_Vector4_norm(ar0);
    // v1.norm_to_byte4(ar2);
    ar2 = v1.pixel();
    SHOW(int(ar2[0]), int(ar2[1]), int(ar2[2]), int(ar2[3]));
  }
  {
    to_norm(Vector4(0.f, 0.49f / 255.f, 0.51f / 255.f, 1.51f / 255.f));
    to_norm(Vector4(-10.f, .5f, 1e4f, 0.f));
    to_norm(Vector4(23.f, 37.f, 45.f, 255.f) / 255.f);
    to_norm(Vector4(0.f, 1.f, 2.f, 3.f) / 255.f);
    to_norm(Vector4(100.f, 101.f, 102.f, 103.f) / 255.f);
  }
  test_consistency();
  test_conversions();
  if (0) {  // Huge numbers fail the conversion to int32_t.
    to_norm(Vector4(2147483583.f, 2147483584.f, 2147483647.f, BIGFLOAT) / 255.f);
    to_norm(Vector4(-2147483580.f, -2147483582.f, -2147483647.f, -BIGFLOAT) / 255.f);
  }
#if 0
  {
    // Fails: static_assert(std::is_trivially_copyable_v<Vector4>);
#if defined(HH_VECTOR4_SSE)
    static_assert(std::is_trivially_copyable_v<__m128>);  // True.
#endif
  }
#endif
}
