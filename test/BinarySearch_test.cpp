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
