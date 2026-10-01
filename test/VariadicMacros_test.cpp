// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Hh.h"  // It includes VariadicMacros.h.

#include "libHh/Array.h"
using namespace hh;

// Count the macro arguments, as used by SHOW() and similar macros.
static_assert(HH_NUM_ARGS(a) == 1);
static_assert(HH_NUM_ARGS(a, b) == 2);
static_assert(HH_NUM_ARGS(a, b, c) == 3);
static_assert(HH_NUM_ARGS(f(x, y), g(z)) == 2);  // Commas within parentheses do not separate arguments.
static_assert(HH_NUM_ARGS(a, b, c, d, e, f, g, h, i, j, k, l) == 12);

static_assert(HH_GT1_ARGS(a) == 0);
static_assert(HH_GT1_ARGS(f(x, y)) == 0);
static_assert(HH_GT1_ARGS(a, b) == 1);
static_assert(HH_GT1_ARGS(a, b, c, d, e, f, g, h, i, j, k, l) == 1);

#define TEST_SQUARE(x) ((x) * (x))
#define TEST_SUM_OF_SQUARES(...) HH_MAP_REDUCE((TEST_SQUARE, +, __VA_ARGS__))
static_assert(TEST_SUM_OF_SQUARES(3) == 9);
static_assert(TEST_SUM_OF_SQUARES(1, 2, 3) == 14);
static_assert(TEST_SUM_OF_SQUARES(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12) == 650);

#define TEST_DOUBLE(x) ((x) * 2)

int main() {
  const Array<int> ar{HH_APPLY((TEST_DOUBLE, 1, 2, 3, 4))};
  SHOW(ar);
  const Array<int> ar12{HH_APPLY((TEST_DOUBLE, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12))};
  SHOW(ar12.num(), ar12.last());
  // SHOW() of several arguments relies on these macros.
  const int a = 1, b = 2, c = 3;
  SHOW(a, b, c);
  SHOW(a + b);
}
