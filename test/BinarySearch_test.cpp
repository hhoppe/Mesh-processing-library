// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/BinarySearch.h"

#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // sort()
using namespace hh;

namespace {

// A type providing only the comparison operators required by the binary search functions.
struct Weight {
  float v;
  // We omit operator<=>() because the intent is to test with !std::totally_ordered<Weight>.
  bool operator<(const Weight& w) const { return v < w.v; }
  bool operator<=(const Weight& w) const { return v <= w.v; }
  bool operator>=(const Weight& w) const { return v >= w.v; }
};

// Verify the postcondition of discrete_binary_search() and its agreement with discrete_binary_search_func().
void verify(CArrayView<int> ar, int xl, int xh, int y_desired) {
  const int x = discrete_binary_search(ar, xl, xh, y_desired);
  assertx(xl <= x && x < xh);
  assertx(ar[x] <= y_desired && y_desired < ar[x + 1]);
  assertx(discrete_binary_search_func([&](int i) { return ar[i]; }, xl, xh, y_desired) == x);
}

void test_continuous_binary_search_func() {
  {  // Compute sqrt(2.) by inverting the function x -> x * x.
    const auto feval = [](double x) { return x * x; };
    const double xtol = 1e-6;
    const double x = continuous_binary_search_func(feval, 0., 2., xtol, 2.);
    SHOW(x);
    assertx(feval(x) <= 2. && 2. < feval(x + xtol));
  }
  {  // The function need only be non-decreasing; here it is a step function.
    const auto feval = [](double x) { return x < 1. ? 0. : x < 2.5 ? 1. : 2.; };
    const double xtol = .001;
    const double x = continuous_binary_search_func(feval, 0., 4., xtol, 1.);
    SHOW(x);
    assertx(feval(x) <= 1. && 1. < feval(x + xtol));
  }
  {  // The abscissa may be a float, and the range may include negative values.
    const auto feval = [](float x) { return x * x * x; };
    const float xtol = 1e-4f;
    const float x = continuous_binary_search_func(feval, -2.f, 1.f, xtol, -1.f);
    assertx(feval(x) <= -1.f && -1.f < feval(x + xtol));
    assertx(continuous_binary_search_func(feval, -2.f, 1.f, 4.f, -1.f) == -2.f);  // Within tolerance, xl is returned.
    // A zero tolerance terminates once xl and xh are adjacent floating-point values.
    assertx(continuous_binary_search_func(feval, -2.f, 1.f, 0.f, -1.f) == -1.f);
  }
}

void test_discrete_binary_search_func() {
  {  // Find the largest integer x such that x * x <= 50.
    const auto feval = [](int x) { return x * x; };
    SHOW(discrete_binary_search_func(feval, 0, 100, 50));
  }
  {  // The abscissa may be any integral type.
    const auto feval = [](int64_t x) { return x * x; };
    const int64_t x = discrete_binary_search_func(feval, int64_t{0}, int64_t{10'000'000}, int64_t{10'000'000'000'000});
    SHOW(x);
  }
  {  // The abscissa may be negative, and a range with xh == xl + 1 immediately returns xl.
    const auto feval = [](int x) { return x; };
    for_intL(y, -100, 100) assertx(discrete_binary_search_func(feval, -100, 100, y) == y);
    assertx(discrete_binary_search_func(feval, -10, -9, -10) == -10);
  }
  {  // The midpoint computation does not overflow, even for ranges spanning the whole type.
    const auto feval = [](int x) { return x; };
    assertx(discrete_binary_search_func(feval, 1'500'000'000, 2'000'000'000, 1'800'000'000) == 1'800'000'000);
    constexpr int imin = std::numeric_limits<int>::min(), imax = std::numeric_limits<int>::max();
    for (const int y : {imin, imin + 1, -1, 0, 1, imax - 1})
      assertx(discrete_binary_search_func(feval, imin, imax, y) == y);
    const auto feval_u = [](uint8_t x) { return x; };
    for_int(y, 255) assertx(discrete_binary_search_func(feval_u, uint8_t{0}, uint8_t{255}, uint8_t(y)) == y);
  }
  {  // As with the integer division (xl + xh) / 2, the midpoint is rounded toward zero, also for negative ranges.
    // This non-monotonic function has crossings at both -3 and -1, so the result reveals the first midpoint.
    const auto feval = [](int x) { return x == -2 || x == 0 ? 1 : 0; };
    assertx(discrete_binary_search_func(feval, -3, 0, 0) == -1);  // The midpoint is -1 rather than -2.
    const auto feval2 = [](int x) { return x == 1 || x == 3 ? 1 : 0; };
    assertx(discrete_binary_search_func(feval2, 0, 3, 0) == 0);  // The midpoint is 1 rather than 2.
  }
}

void test_discrete_binary_search() {
  {  // Look up values in a cumulative distribution.
    const Array<float> ar{0.f, .1f, .3f, .6f, 1.f};
    for (const float y : {0.f, .05f, .1f, .29f, .3f, .95f}) {
      const int x = discrete_binary_search(ar, 0, ar.num() - 1, y);
      SHOW(y, x);
    }
  }
  {  // With duplicate values, the largest index x satisfying ar[x] <= y_desired is returned.
    const Array<int> ar{0, 1, 1, 1, 2, 2, 5};
    for_int(y, 5) {
      const int x = discrete_binary_search(ar, 0, ar.num() - 1, y);
      SHOW(y, x);
    }
  }
  {  // The search may be restricted to a subrange [xl, xh] of the array.
    const Array<int> ar{0, 10, 20, 30, 40, 50};
    SHOW(discrete_binary_search(ar, 2, 4, 35));
  }
  {  // The element type need not be std::totally_ordered.
    const Array<Weight> ar{Weight{0.f}, Weight{1.f}, Weight{2.f}};
    assertx(discrete_binary_search(ar, 0, 2, Weight{1.5f}) == 1);
    assertx(discrete_binary_search_func([&](int i) { return ar[i]; }, 0, 2, Weight{1.5f}) == 1);
  }
}

void test_random_arrays() {
  for_int(iter, 100) {
    const int n = 2 + Random::G.get_unsigned(10);
    Array<int> ar(n);
    for (int& e : ar) e = Random::G.get_unsigned(8);
    sort(ar);
    for_int(xl, n) for_intL(xh, xl + 1, n) for_intL(y, ar[xl], ar[xh]) verify(ar, xl, xh, y);
  }
}

}  // namespace

int main() {
  test_continuous_binary_search_func();
  test_discrete_binary_search_func();
  test_discrete_binary_search();
  test_random_arrays();
}

template int hh::discrete_binary_search(CArrayView<float>, int, int, float);
template int hh::discrete_binary_search(CArrayView<double>, int, int, double);
template int hh::discrete_binary_search(CArrayView<int>, int, int, int);
