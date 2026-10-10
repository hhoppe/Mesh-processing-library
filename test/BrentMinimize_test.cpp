// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/BrentMinimize.h"

#include "libHh/MathOp.h"  // square()
using namespace hh;

namespace {

void test_parabola() {
  {  // The minimum of a parabola is found with few iterations, thanks to the parabolic interpolation steps.
    const auto func = [](double x) { return square(x - 0.3) + 2.; };
    std::uintmax_t max_iter = 100;
    const auto [min_x, min_f_x] = brent_minimize(func, -1., 2., 26, max_iter);
    SHOW(round_fraction_digits(min_x), round_fraction_digits(min_f_x));
    assertx(abs(min_x - 0.3) < 1e-6);
    assertx(max_iter < 20);
  }
  {  // The abscissa may be a float, in which case bits should not exceed 12.
    const auto func = [](float x) { return square(x + 0.7f); };
    std::uintmax_t max_iter = 100;
    const auto [min_x, min_f_x] = brent_minimize(func, -2.f, 1.f, 12, max_iter);
    SHOW(round_fraction_digits(min_x, 1e3f));
    assertx(abs(min_x + 0.7f) < 1e-3f && min_f_x < 1e-6f);
  }
}

void test_interval_endpoint() {
  // For a monotonic function, the minimum lies at an endpoint of the interval (to within the tolerance).
  const auto func = [](double x) { return x; };
  std::uintmax_t max_iter = 100;
  const auto [min_x, unused_min_f_x] = brent_minimize(func, 1., 3., 20, max_iter);
  SHOW(round_fraction_digits(min_x, 1e3));
  assertx(min_x >= 1. && min_x < 1.001);
}

void test_max_iter() {
  // The iteration count is bounded by max_iter, which on output holds the number of iterations performed.
  const auto func = [](double x) { return std::cos(x); };
  std::uintmax_t max_iter = 3;
  brent_minimize(func, 0., 6., 26, max_iter);
  SHOW(max_iter);
  assertx(max_iter == 3);
  std::uintmax_t max_iter2 = 100;
  const auto [min_x2, min_f_x2] = brent_minimize(func, 0., 6., 26, max_iter2);
  SHOW(round_fraction_digits(min_x2), round_fraction_digits(min_f_x2));
  assertx(abs(min_x2 - D_TAU / 2) < 1e-6 && max_iter2 < 100);
}

}  // namespace

int main() {
  test_parabola();
  test_interval_endpoint();
  test_max_iter();
}
