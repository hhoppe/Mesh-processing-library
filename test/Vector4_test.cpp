// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Vector4.h"

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
      // (The reference is not a[c] / f, which a compiler with /fp:fast may evaluate as a[c] * (1.f / f).)
      num_div_scalar += quotient_scalar[c] != (a / Vector4(f))[c];
      num_mul += product[c] != a[c] * b[c];
    }
    // The volatile products prevent their contraction with the sums into fused multiply-adds, which
    // -ffp-contract=fast permits (although we have not observed it with gcc 15 or clang 21).
    const volatile float p0 = a[0] * b[0], p1 = a[1] * b[1], p2 = a[2] * b[2], p3 = a[3] * b[3];
    const float expected_dot = (p0 + p1) + (p2 + p3);
    num_dot += dot(a, b) != expected_dot;
  }
  SHOW(num_div, num_div_scalar, num_mul, num_dot);
  // The remaining operations are also exact in IEEE arithmetic, so all implementations must agree bit for bit.
  // (A comparison with != ignores the sign of zero.)
  int num_add = 0, num_sub = 0, num_scalar = 0, num_min_max = 0, num_neg = 0, num_abs = 0, num_sqrt = 0;
  for_int(iter, 10'000) {
    const Vector4 a(lcg(), lcg(), lcg(), lcg()), b(lcg(), lcg(), lcg(), lcg());
    const float f = lcg();
    const Vector4 sum_ab = a + b, diff_ab = a - b, sum_f = a + f, diff_f = a - f, prod_f = a * f, prod_f2 = f * a;
    const Vector4 vmin = min(a, b), vmax = max(a, b), neg = -a, vabs = abs(a), vsqrt = sqrt(abs(a));
    for_int(c, 4) {
      num_add += sum_ab[c] != a[c] + b[c];
      num_sub += diff_ab[c] != a[c] - b[c];
      num_scalar += sum_f[c] != a[c] + f || diff_f[c] != a[c] - f || prod_f[c] != a[c] * f || prod_f2[c] != f * a[c];
      num_min_max += vmin[c] != min(a[c], b[c]) || vmax[c] != max(a[c], b[c]);
      num_neg += neg[c] != -a[c];
      num_abs += vabs[c] != std::abs(a[c]) || std::signbit(vabs[c]);
      num_sqrt += vsqrt[c] != std::sqrt(std::abs(a[c]));
    }
  }
  SHOW(num_add, num_sub, num_scalar, num_min_max, num_neg, num_abs, num_sqrt);
}

// Element access, iteration, loads and stores, and the compound assignment operators.
void test_interface() {
  const Vector4 v1(1.f, 2.f, 3.f, 4.f), v2(8.f, 7.f, 6.f, 5.f);
  assertx(v1.size() == 4 && Vector4::ok(3) && !Vector4::ok(4) && !Vector4::ok(-1));
  {
    Vector4 v = v1;
    v[2] = 9.f;
    assertx(v[0] == 1.f && v[2] == 9.f && v.data()[2] == 9.f);
    assertx(dist2(v1.with(2, 9.f), v) == 0.f);
    assertx(v1[2] == 3.f);  // with() does not modify the original.
    float sum_iter = 0.f;
    for (const float f : v1) sum_iter += f;
    assertx(sum_iter == 10.f && v1.end() - v1.begin() == 4);
  }
  {
    float ar[5] = {9.f, 1.f, 2.f, 3.f, 4.f};
    Vector4 v;
    v.load_unaligned(ar + 1);  // Possibly misaligned.
    assertx(dist2(v, v1) == 0.f);
    v2.store_unaligned(ar + 1);
    assertx(ar[0] == 9.f && ar[1] == 8.f && ar[4] == 5.f);
    assertx(dist2(Vector4(V(1.f, 2.f, 3.f, 4.f)), v1) == 0.f);  // Constructor from Vec4<float>.
  }
  {
    Vector4 v = v1;
    v += v2;  // [9, 9, 9, 9]
    v -= v1;  // [8, 7, 6, 5]
    v *= v1;  // [8, 14, 18, 20]
    v /= v2;  // [1, 2, 3, 4]
    assertx(dist2(v, v1) == 0.f);
    v += 2.f;
    v -= 1.f;
    v *= 4.f;
    v /= 2.f;
    SHOW(v);
    assertx(dist2(v, (v1 + 1.f) * 2.f) == 0.f);
    SHOW(interp(v1, v2, .25f), interp(v1, v2));
    SHOW(sqrt(Vector4(4.f, 9.f, .25f, 0.f)), abs(Vector4(-1.f, 2.f, -0.f, -3.5f)));
  }
  {
    // The constructor from a Pixel normalizes the values to [0.f, 1.f].
    const Pixel pixel(0, 51, 255, 102);
    const Vector4 v(pixel);
    SHOW(v);
    assertx(dist2(v, to_Vector4_norm(pixel)) == 0.f && v.pixel() == pixel);
  }
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

// uint8_from_unit() rounds 255 * v to the nearest integer, with ties to even, and agrees with Vector4::pixel().
void test_uint8_from_unit() {
  // Exact ties (where v * 255.f == k + .5f) round to the even neighbor, which is sometimes below and sometimes above.
  int num_tie_down = 0, num_tie_up = 0;
  for_int(k, 255) {
    const float x = (float(k) + .5f) / 255.f;
    for (const float v : {std::nextafter(x, 0.f), x, std::nextafter(x, 1.f)}) {
      if (v * 255.f != float(k) + .5f) continue;
      const int result = uint8_from_unit(v);
      assertx(result % 2 == 0 && (result == k || result == k + 1));
      (result == k ? num_tie_down : num_tie_up)++;
    }
  }
  assertx(num_tie_down > 0 && num_tie_up > 0);
  SHOW(int(uint8_from_unit(.5f)));  // The exact tie 127.5 rounds to the even 128.
  // Values outside [0, 1] are clamped, and a NaN gives 0.
  constexpr float inf = std::numeric_limits<float>::infinity();
  for (const float v : {-1.f, -0.f, -inf, 1.f + 1e-6f, 2.f, inf}) assertx(uint8_from_unit(v) == (v > 0.f ? 255 : 0));
  assertx(uint8_from_unit(std::numeric_limits<float>::quiet_NaN()) == 0);
  // It agrees with Vector4::pixel() (e.g., the SSE or NEON version), both on a sweep of all the floats in [0, 1] and
  // next to each rounding boundary (k + .5f) / 255.f and each value k / 255.f.
  const auto agrees = [](float v) { return uint8_from_unit(v) == Vector4(v).pixel()[0]; };
  int num_disagree = 0, num_tested = 0;
  for (uint32_t bits = 0; bits <= std::bit_cast<uint32_t>(1.f); bits += 997) {
    num_disagree += !agrees(std::bit_cast<float>(bits)), num_tested++;
  }
  for_int(k, 256) {
    for (const float x : {(float(k) + .5f) / 255.f, float(k) / 255.f}) {
      float v = x;
      for_int(i, 4) v = std::nextafter(v, 0.f);
      for_int(i, 9) num_disagree += !agrees(v), num_tested++, v = std::nextafter(v, 2.f);
    }
  }
  // (Vector4::pixel() requires that 255 * v fit in an int32, so infinities are excluded.)
  for (const float v : {-1.f, 2.f, -1e6f, 1e6f}) num_disagree += !agrees(v), num_tested++;
  assertx(num_tested > 1'000'000);
  SHOW(num_disagree);
}

}  // namespace

int main() {
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
  test_uint8_from_unit();
  test_interface();
  if (0) {  // Huge numbers fail the conversion to int32_t (the assertion in norm_to_byte4() in debug builds).
    to_norm(Vector4(2147483583.f, 2147483584.f, 2147483647.f, BIGFLOAT) / 255.f);
    to_norm(Vector4(-2147483580.f, -2147483582.f, -2147483647.f, -BIGFLOAT) / 255.f);
  }
}
