// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BINARYSEARCH_H_
#define MESH_PROCESSING_LIBHH_BINARYSEARCH_H_

#include "libHh/Array.h"

namespace hh {

// Given xl < xh, feval(xl) <= y_desired < feval(xh), find x such that feval(x) == y_desired within some tolerance.
// More precisely, find x such that exists x' with x <= x' < x + xtol and feval(x') == y_desired .
template <typename T1, typename T2, typename Func = T2(const T1&)>
[[nodiscard]] T1 continuous_binary_search_func(Func feval, T1 xl, T1 xh, T1 xtol, T2 y_desired) {
  static_assert(std::is_floating_point_v<T1>);
  assertx(xl < xh);
  for (;;) {
    ASSERTXX(xl < xh && feval(xl) <= y_desired && y_desired < feval(xh));
    if (xh - xl < xtol) return xl;
    T1 xm = (xl + xh) / 2;
    T2 ym = feval(xm);
    if (y_desired >= ym)
      xl = xm;
    else
      xh = xm;
  }
}

// Given xl < xh, feval(xl) <= y_desired < feval(xh), find x such that feval(x) <= y_desired < feval(x + 1) .
template <typename T1, typename T2, typename Func = T2(const T1&)>
[[nodiscard]] T1 discrete_binary_search_func(Func feval, T1 xl, T1 xh, T2 y_desired) {
  static_assert(std::is_integral_v<T1>);
  using U = std::make_unsigned_t<T1>;
  assertx(xl < xh);
  for (;;) {
    ASSERTXX(xl < xh && feval(xl) <= y_desired && y_desired < feval(xh));
    // if (xh - xl == 1) return xl;
    // T1 xm = (xl + xh) / 2;  // Could overflow.
    const U diff = U(U(xh) - U(xl));  // Equals xh - xl, computed without overflow.
    if (diff == 1) return xl;
    T1 xm = xl + T1(diff / 2);  // Equals floor((xl + xh) / 2), computed without overflow.
    if constexpr (std::is_signed_v<T1>)
      if (xm < 0 && diff % 2) xm++;  // Round toward zero, like the integer division (xl + xh) / 2.
    T2 ym = feval(xm);
    if (y_desired >= ym)
      xl = xm;
    else
      xh = xm;
  }
}

// Given xl < xh, ar[xl] <= y_desired < ar[xh], find x such that ar[x] <= y_desired < ar[x + 1] .
template <typename T> [[nodiscard]] int discrete_binary_search(CArrayView<T> ar, int xl, int xh, T y_desired) {
  assertx(xl < xh);
  assertx(ar[xl] <= y_desired && y_desired < ar[xh]);
  // Use upper_bound() to find the first index x1 in (xl, xh] for which ar[x1] > y_desired, then obtain x = x1 - 1.
  return narrow_cast<int>(ranges::upper_bound(ar.slice(xl, xh), y_desired, std::less<>{}) - ar.begin()) - 1;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BINARYSEARCH_H_
