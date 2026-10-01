// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MathOp.h"

#include <bit>  // bit_cast(), bit_width(), popcount().

#include "libHh/RangeOp.h"  // min(), max(), reverse().

using namespace hh;

// (float)0., -0.      0x00000000
// (float)1            0x3f800000
// (float)3            0x40400000
// (float)-zero        0x80000000  (-1.f / 1e30f / 1e30f)
// (float)-1           0xbf800000
// (float)-3           0xc0400000
// (float)1.#INF       0x7f800000  (+1.f/0.f)  (C++11: INFINITY, std::numeric_limits<float>::infinity())
// (float)-1.#INF      0xff800000  (-1.f/0.f)
// (float)-1.#IND      0xffc00000  (0.f/0.f)  (C++11: NAN, std::numeric_limits<float>::quiet_NaN(), std::nanf(""))
//                     0x00400000  always forced on by hardware for any nanf (x86)

namespace {

// Create an infinite float value.  (Some compilers warn about overflow in constant arithmetic for INFINITY.)
inline float create_infinityf() { return std::numeric_limits<float>::infinity(); }

// Create a not-a-number float value which encodes integer i (0..4194303 or 22 bits).
// For i == 0, the result equals std::numeric_limits<float>::quiet_NaN() on x86 and arm64.
inline float create_nanf(unsigned i = 0) {
  assertx((i & 0xffc00000) == 0);
  // Could in principle retrieve 0x80000000 (sign) bit from i and use it, but forget it.
  const uint32_t v = 0x7fc00000 | (i & 0x003fffff);
  return std::bit_cast<float>(v);
}

// Retrieve the integer value encoded in the not-a-number value f.
inline unsigned nanf_value(float f) {
  assertx(std::isnan(f));
  const uint32_t v = std::bit_cast<uint32_t>(f);
  return v & 0x003fffff;
}

// Trig::cos() and Trig::sin() use tables for small denominators j and std::cos()/std::sin() otherwise.
void test_trig() {
  for_intL(j, 1, 20) {
    assertx(Trig::cos(0, j) == 1.f && Trig::sin(0, j) == 0.f);
    for_intL(i, -j + 1, j) {
      const double angle = i * D_TAU / j;
      // The float computation of the angle has a roundoff error of up to about 6e-7.
      assertx(abs(Trig::cos(i, j) - std::cos(angle)) < 3e-6);
      assertx(abs(Trig::sin(i, j) - std::sin(angle)) < 3e-6);
      assertx(Trig::cos(-i, j) == Trig::cos(i, j));   // Even function.
      assertx(Trig::sin(-i, j) == -Trig::sin(i, j));  // Odd function.
    }
  }
  SHOW(Trig::cos(1, 6), Trig::sin(3, 12), Trig::sin(-1, 4), Trig::cos(10, 20));
}

void test_my_mod() {
  static_assert(my_mod(-4, 7) == 3 && my_mod(-7, 7) == 0 && my_mod(-8, 7) == 6 && my_mod(13, 7) == 6);
  for_intL(b, 1, 9) for_intL(a, -30, 31) {
    const int expected = ((a % b) + b) % b;  // Brute-force reference.
    assertx(my_mod(a, b) == expected);
    assertx(my_mod(a, b) >= 0 && my_mod(a, b) < b);
  }
  assertx(my_mod(std::numeric_limits<int>::min(), 2) == 0);
  assertx(my_mod(std::numeric_limits<int>::min(), 3) == 1);  // -2147483648 == -715827883 * 3 + 1.
  // Floating-point version; these values are exactly representable.
  SHOW(my_mod(-.5f, 2.f), my_mod(5.25, 2.), my_mod(7.5f, 2.5f) == 0.f, my_mod(-.25, .5));
  assertx(my_mod(-4.f, 2.f) == 0.f);  // Possibly -0.f, which compares equal to 0.f.
  // KNOWN_BUG: my_mod(-1e-10f, 1.f) returns 1.f, i.e., b itself (and fails an ASSERTX in debug builds), so tiny
  // negative arguments are not tested.
  for_intL(i, -20, 21) {
    const float a = float(i) * .37f;
    const float r = my_mod(a, 1.5f);
    assertx(r >= 0.f && r < 1.5f);
    const float k = (a - r) / 1.5f;  // The quotient must be an integer.
    assertx(abs(k - std::round(k)) < 1e-5f);
  }
}

void test_smooth_step_frac_gaussian() {
  static_assert(smooth_step(0.) == 0. && smooth_step(1.) == 1. && smooth_step(.5f) == .5f);
  for_int(i, 11) {
    const double x = i / 10.;
    assertx(abs(smooth_step(1. - x) - (1. - smooth_step(x))) < 1e-15);  // Point symmetry about (.5, .5).
    if (i) assertx(smooth_step(x) > smooth_step(x - .1));               // Monotonic.
  }
  // The derivative 6 * x * (1 - x) vanishes at both ends.
  const double h = 1e-6;
  assertx(smooth_step(h) / h < 4. * h && (1. - smooth_step(1. - h)) / h < 4. * h);  // Secant slopes 3 * h.
  SHOW(frac(-.25f), frac(2.5), frac(3.f), frac(-3.f));
  // The integral of a Gaussian density is 1, for any standard deviation.
  for (const double sdv : {.5, 1., 3.}) {
    const int n = 20'000;
    const double dx = 20. * sdv / n;
    double sum = 0.;
    for_int(i, n) sum += gaussian(-10. * sdv + (i + .5) * dx, sdv) * dx;  // Midpoint rule.
    assertx(abs(sum - 1.) < 1e-6);
    assertx(abs(gaussian(.7 * sdv, sdv) - gaussian(.7) / sdv) < 1e-12);  // Scaling identity.
    assertx(gaussian(-.3, sdv) == gaussian(.3, sdv));                    // Symmetry.
  }
  SHOW(gaussian(0.f), 1.f / std::sqrt(TAU));
}

void test_clamped_functions() {
  // Arguments slightly outside [-1, 1] (from roundoff errors) are clamped rather than producing NaN.
  assertx(my_acos(1.0001f) == 0.f && my_acos(-1.0001) == std::acos(-1.));
  assertx(my_asin(1.0001) == std::asin(1.) && my_asin(-1.0001f) == std::asin(-1.f));
  assertx(my_acos(.5) == std::acos(.5) && my_asin(-.5f) == std::asin(-.5f));
  assertx(my_sqrt(-1e-7f) == 0.f && my_sqrt(-1e-12) == 0. && my_sqrt(0.f) == 0.f);
  assertx(my_sqrt(6.25f) == 2.5f && my_sqrt(2.) == std::sqrt(2.));
}

void test_pow2_log2() {
  for_int(i, 1100) {
    const unsigned u = unsigned(i);
    assertx(is_pow2(u) == (std::popcount(u) == 1));
    if (u) assertx(int_floor_log2(u) == std::bit_width(u) - 1);
  }
  for_int(k, 32) {
    const unsigned u = 1u << k;
    assertx(is_pow2(u) && int_floor_log2(u) == k && int_floor_log2(u | (u - 1)) == k);
    if (k > 1) assertx(!is_pow2(u + 1) && !is_pow2(u - 1) && int_floor_log2(u - 1) == k - 1);
  }
  assertx(!is_pow2(0) && is_pow2(1) && !is_pow2(std::numeric_limits<unsigned>::max()));
  assertx(int_floor_log2(0) == 0);  // Degenerate case: std::log2(0.) is -infinity.
}

void test_vec_functions() {
  const Vec4<float> v(-1.5f, 2.5f, -3.f, .25f);
  SHOW(floor(v), ceil(v), abs(v));
  const Vec3<double> vd(-.5, 1e10 + .5, -1e-20);
  SHOW(floor(vd)[0], ceil(vd)[2] == 0., abs(vd)[0]);
}

// Evaluate the Bezier curve with control points ar at parameter t using de Casteljau's algorithm.
double de_casteljau(CArrayView<float> ar, double t) {
  Array<double> b(ar.num());
  for_int(i, ar.num()) b[i] = ar[i];
  for (int n = ar.num() - 1; n > 0; n--) for_int(i, n) b[i] = (1. - t) * b[i] + t * b[i + 1];
  return b[0];
}

void test_bspline_properties() {
  const Array<float> ar = {1.f, 3.f, 2.f, 7.f, 5.f};
  const int n = ar.num() - 1;
  Array<float> ar_reversed(ar);
  reverse(ar_reversed);
  for_int(deg, n + 1) {
    for_int(i, 33) {
      const float t = i / 32.f;
      const float v = eval_uniform_bspline(ar, deg, t);
      // Partition of unity: a constant set of coefficients is reproduced.
      assertx(abs(eval_uniform_bspline(Array<float>(ar.num(), 2.f), deg, t) - 2.f) < 4e-6f);
      // Convex hull property.
      assertx(v >= min(ar) - 1e-5f && v <= max(ar) + 1e-5f);
      // The uniform clamped knot vector is symmetric.
      if (i > 0 && i < 32) assertx(abs(eval_uniform_bspline(ar_reversed, deg, 1.f - t) - v) < 1e-5f);
      // A linear spline interpolates linearly between the coefficients.
      if (deg == 1) {
        const float u = t * n;
        const int k = min(int(u), n - 1);
        assertx(abs(v - (ar[k] + (u - k) * (ar[k + 1] - ar[k]))) < 1e-5f);
      }
      // With a single segment (deg == n), the spline is a Bezier curve.
      if (deg == n) assertx(abs(v - de_casteljau(ar, t)) < 1e-5f);
    }
    // The endpoints are interpolated for all degrees.
    assertx(abs(eval_uniform_bspline(ar, deg, 0.f) - ar[0]) < 1e-6f);
    assertx(abs(eval_uniform_bspline(ar, deg, 1.f) - ar.last()) < 1e-5f);
  }
}

}  // namespace

#if defined(_MSC_VER) && !defined(__clang__)
// Under -fp:fast, the optimizer folds 0.f / float_zero to 0 rather than producing NaN.
#pragma float_control(precise, on)
#endif

int main() {
  {
    assertx(INFINITY == HUGE_VALF);
    // assertx(std::numeric_limits<float>::infinity() == HUGE_VALF);  // Warning: overflow in constant arithmetic.
    // assertx(std::numeric_limits<double>::infinity() == HUGE_VAL);
  }
  {
    const Array<float> ar = {100.f, 102.f, 103.f, 110.f, 100.f, 90.f, 80.f, 71.f};
    const int n = 20;
    for_int(i, n) {
      const float x = i / (n - 1.f);  // Here, x in [0, 1].
      const int degree = 3;
      const float v = eval_uniform_bspline(ar, degree, x);
      showf("x=%7.4f   v=%6.3f\n", x, v);
    }
  }
  if (1) {
    // IEEE 754 explicitly says the sign of a NaN carries no meaning and no operation interprets it. isnan ignores it.
    // In x86 and _MSVC_STL_VERSION, the invalid-operation default is the "real indefinite" QNaN 0xffc00000,
    // which is why printf shows -nan(ind).
    // In contrast, in ARM and glibc, the default NaN is 0x7fc00000 with the sign clear (positive).
    const bool clear_nan_bit31 = true;  // True for cross-platform consistency; false to inspect native sign bit.

    const auto func_show_float = [](float a) {
      uint32_t v = std::bit_cast<uint32_t>(a);
      if (std::isnan(a) && clear_nan_bit31) v &= 0x7fffffff;
      const float a2 = std::bit_cast<float>(v);
      const string s = sform("(float)%-15.9g 0x%08x  F%d I%d N%d%s\n",  //
                             a2, v, std::isfinite(a), std::isinf(a), std::isnan(a),
                             std::isnan(a) ? sform(" nanfv%08x", nanf_value(a)).c_str() : "");
      std::cerr << s;
    };
    const float float_zero = g_unoptimized_zero ? 1.f : 0.f;
    float a;
    a = +0.f;
    func_show_float(a);
    a = +1.f;
    func_show_float(a);
    a = +3.f;
    func_show_float(a);
    a = -0.f;
    func_show_float(a);
    a = -1.f / 1e30f / 1e30f;
    func_show_float(a);
    a = -1.f;
    func_show_float(a);
    a = -3.f;
    func_show_float(a);
    a = +1.f / float_zero;
    func_show_float(a);
    a = -1.f / float_zero;
    func_show_float(a);
    a = std::acos(2.f);
    func_show_float(a);
    a = std::acos(-2.f);
    func_show_float(a);
    a = 0.f / float_zero;
    func_show_float(a);
    a = create_infinityf();
    func_show_float(a);
    a = -create_infinityf();
    func_show_float(a);
    a = create_nanf();
    func_show_float(a);
    a = create_nanf(0x00000000);
    func_show_float(a);
    a = create_nanf(0x00000001);
    func_show_float(a);
    a = create_nanf(0x00000002);
    func_show_float(a);
    a = create_nanf(0x003fffff);
    func_show_float(a);
    SHOW(std::isfinite(0.f / float_zero));  // See "#pragma float_control(precise, on)" above.
  }
  {
    const int vm4mod7 = my_mod(-4, 7);
    SHOW(vm4mod7);
    constexpr double vmid = smooth_step(.5);
    SHOW(vmid);
    constexpr double vfurther = smooth_step(2. / 3.);
    SHOW(vfurther);
    const float vfrac = frac(TAU);
    SHOW(vfrac);
    const float vgauss1 = gaussian(1.f);
    SHOW(vgauss1);
    const double vmyacos = my_acos(-1.0001);
    SHOW(vmyacos);
    constexpr bool is_pow2_16 = is_pow2(16);
    SHOW(is_pow2_16);
    constexpr bool is_pow2_17 = is_pow2(17);
    SHOW(is_pow2_17);
  }
  {
    assertx(std::isnan(std::acos(-1.0001)));
  }
  test_trig();
  test_my_mod();
  test_smooth_step_frac_gaussian();
  test_clamped_functions();
  test_pow2_log2();
  test_vec_functions();
  test_bspline_properties();
}
