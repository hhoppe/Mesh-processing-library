// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MATHOP_H_
#define MESH_PROCESSING_LIBHH_MATHOP_H_

#include "libHh/Array.h"
#include "libHh/SGrid.h"

namespace hh {

// Precomputed cosf() and sinf() functions over integer divisions of TAU.
class Trig {
 public:
  // compute cos(i * TAU / j)
  static float cos(int i, int j) {
    Table& table = cos_table_instance();
    if (!table[1][0]) init();
    int ia = abs(i);
    ASSERTX(ia < j);
    float v = j < k_size ? table[j][ia] : std::cos(ia * TAU / j);
    return v;
  }
  // compute sin(i * TAU / j)
  static float sin(int i, int j) {
    Table& table = sin_table_instance();
    if (!table[1][0]) init();
    int ia = abs(i);
    ASSERTX(ia < j);
    float v = j < k_size ? table[j][ia] : std::sin(ia * TAU / j);
    return i < 0 ? -v : v;
  }

 private:
  static constexpr int k_size = 13;
  using Table = SGrid<float, k_size, k_size - 1>;
  static Table& cos_table_instance() {
    static auto& t = *new Table;
    return t;
  }
  static Table& sin_table_instance() {
    static auto& t = *new Table;
    return t;
  }
  static void init() {
    for_intL(j, 1, k_size) {
      for_int(i, j) {
        cos_table_instance()[j][i] = std::cos(i * TAU / j);
        sin_table_instance()[j][i] = std::sin(i * TAU / j);
      }
    }
  }
};

namespace details {
inline int fix_mod(int ret, int b) {
  return ret < 0 ? ret + b : ret;
  // return ret + b * (ret < 0);
  // b &= -(ret < 0); return ret + b;
}
template <typename T> bool my_mod_check(T ret, T b) {
  if (!(ret >= T{0} && ret < b)) {
    SHOW(ret, b);
    assertnever("");
  }
  return true;
}
}  // namespace details

// Modulo operation.  (The built-in C/C++ remainder operation (a % b) returns negative remainders if a < 0).
inline int my_mod(int a, int b) {
  // http://stackoverflow.com/questions/4003232/
  // Note: given int a >= 0, my_mod(a - 1, n) is still not as fast as (a - 1 + n) % n.
  return (ASSERTX(b > 0), details::fix_mod(a % b, b));
}

// Modulo operation on floating-point values.  (In contrast, std::fmod(a, b) returns negative remainders if a < 0.f).
template <typename T> T my_mod(T a, T b) {
  static_assert(std::is_floating_point<T>::value, "");
  ASSERTX(b > T{0});
  T ret = std::fmod(a, b);
  if (ret < T{0}) ret += b;
  ASSERTX(details::my_mod_check(ret, b));
  return ret;
}

// Evaluate a B-spline function; x lies in [0, 1] which spans over all coefficients in ar.
// The B-spline construction interpolates ar[0] at t == 0.f and ar.last() at t == 1.f.
float eval_uniform_bspline(CArrayView<float> ar, int deg, float t);

// Evaluate a smooth-step function; x in [0, 1] -> ret: [0, 1]  (with zero derivatives at x == 0 and x == 1).
template <typename T> constexpr T smooth_step(T x) {
  static_assert(std::is_floating_point<T>::value, "");
  return x * x * (T{3} - T{2} * x);
}

// Compute fractional part (as in HLSL).
template <typename T> T frac(T f) {
  static_assert(std::is_floating_point<T>::value, "");
  return f - floor(f);
}

// Evaluate a Gaussian function.
template <typename T> T gaussian(T x, T sdv = T{1}) {
  static_assert(std::is_floating_point<T>::value, "");
  return std::exp(-square(x / sdv) / T{2}) / (sqrt(static_cast<T>(D_TAU)) * sdv);
}

// Like std::acos() but prevent NaN's from appearing due to roundoff errors.
// my_acos() is discouraged due to poor accuracy for small angles! see angle_between_unit_vectors().
template <typename T> T my_acos(T a) {
  return (a < T{-1}   ? (assertw(a > T{-1.001f}), std::acos(T{-1}))
          : a > T{+1} ? (assertw(a < T{+1.001f}), std::acos(T{+1}))
                      : std::acos(a));
}

// Like std::asin() but prevent NaN's from appearing due to roundoff errors.
template <typename T> T my_asin(T a) {
  return (a < T{-1}   ? (assertw(a > T{-1.001f}), std::asin(T{-1}))
          : a > T{+1} ? (assertw(a < T{+1.001f}), std::asin(T{+1}))
                      : std::asin(a));
}

// Like std::sqrt() but prevent NaN's from appearing due to roundoff errors.
template <typename T> T my_sqrt(T a) {
  static_assert(std::is_floating_point<T>::value, "");
  return a < T{0} ? (assertx(a > (sizeof(T) == sizeof(float) ? T{-1e-5f} : T{-1e-10f})), T{0}) : sqrt(a);
}

// Is the integer i an even power of two?
inline constexpr bool is_pow2(unsigned i) { return i > 0 && (i & (i - 1)) == 0; }

// Fast version of static_cast<int>(floor(std::log2(x))).
inline int int_floor_log2(unsigned x) {
  int a = 0;
  while (x >>= 1) a++;
  return a;
}

// Generalize some scalar operations on Vec.
#define E(op)                                                      \
  template <typename T, int n> Vec<T, n> op(const Vec<T, n>& ar) { \
    return map(ar, [](T e) { return std::op(e); });                \
  }                                                                \
  HH_EAT_SEMICOLON
E(floor);
E(ceil);
E(abs);
#undef E

//----------------------------------------------------------------------------

namespace details {
inline float funct(int i, int gk, int n) {
  assertx(0 <= i && i <= n + gk);
  if (i < gk) return 0.f;
  if (i > n) return n - gk + 2.f;
  return i - gk + 1.f;
}
inline float funcn(int i, int k, float u, int n, int gk) {
  if (k == 1) return funct(i, gk, n) <= u && u < funct(i + 1, gk, n) ? 1.f : 0.f;
  float a = 0.f;
  float d1 = funct(i + k - 1, gk, n) - funct(i, gk, n);
  float n1a = u - funct(i, gk, n);
  float n1b = funcn(i, k - 1, u, n, gk);
  if (n1b) {
    if (!d1) {
      assertw(!n1a);
      a += n1b;
    } else {
      a += n1a / d1 * n1b;
    }
  }
  float d2 = funct(i + k, gk, n) - funct(i + 1, gk, n);
  float n2a = funct(i + k, gk, n) - u;
  float n2b = funcn(i + 1, k - 1, u, n, gk);
  if (n2b) {
    if (!d2) {
      assertw(!n2a);
      a += n2b;
    } else {
      a += n2a / d2 * n2b;
    }
  }
  return a;
}
}  // namespace details

// Adapted from Micheal Mortenson, Geometric Modeling; very inefficient.
inline float eval_uniform_bspline(CArrayView<float> ar, int deg, float t) {
  assertw(t >= 0.f && t <= 1.f);
  int n = ar.num() - 1;
  int k = deg + 1;
  if (t == 1.f) t = 1.f - 1e-7f;
  float u = t * (n - k + 2.f);
  float sum = 0.f;
  for_int(i, n + 1) sum += ar[i] * details::funcn(i, k, u, n, k);
  return sum;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MATHOP_H_
