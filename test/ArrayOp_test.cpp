// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/ArrayOp.h"

#include <vector>

#include "libHh/Random.h"
using namespace hh;

namespace {

// Verifies the rank-based functions on a list of values against a fully sorted copy of the list.
template <typename T> void verify_against_sorted(CArrayView<T> array) {
  const Array<T> sorted = sort(Array<T>(array));
  const int n = sorted.num();
  for_int(i, n) assertx(rank_element(array, i) == sorted[i]);
  const auto two = median_two(array);
  assertx(two[0] == sorted[(n - 1) / 2] && two[1] == sorted[n / 2]);
  // The fractional rank rankf selects index floor(rankf * n), clamped to n - 1.
  for (const double rankf : {0., .25, .5, .75, 1.})
    assertx(rankf_element(array, rankf) == sorted[min(int(std::floor(rankf * n)), n - 1)]);
  Array<T> unique;
  for (const T& e : sorted)
    if (!unique.num() || unique.last() != e) unique.push(e);
  assertx(sort_unique(array) == unique);
}

}  // namespace

int main() {
  {
    const auto check_array = [](const auto& array) {
      SHOW(sort_unique(array));
      SHOW(median_two(array));
      SHOW(median(array));
      for_int(i, array.num()) SHOW(i, rank_element(array, i));
    };
    check_array(Array{4, 3, 2, 2, 5, 4});
    check_array(Array{1, 2, 3, 4});
    check_array(Array{4, 5, 2, 1, 3});
  }
  {
    // Lists of one and two elements.
    SHOW(median_two(V(7)), median(V(7)));
    SHOW(median_two(V(8, 3)), median(V(8, 3)));
    SHOW(sort_unique(V(5)));
    SHOW(rankf_element(V(5), 0.), rankf_element(V(5), 1.));
  }
  {
    // The median of integers is the mean of the two middle values, as a floating-point type.
    const auto med = median(V(1, 2));
    static_assert(std::is_same_v<decltype(med), const double>);
    SHOW(med);
    // The median of floats is computed in double.
    const auto medf = median(V(1.5f, 2.f, 3.f, 2.5f));
    static_assert(std::is_same_v<decltype(medf), const double>);
    SHOW(medf);
  }
  {
    // Fractional ranks, including the boundaries 0. (minimum) and 1. (maximum).
    const Array<int> ar{40, 10, 30, 20};
    for (const double rankf : {0., .2, .25, .49, .5, .74, .75, .99, 1.}) SHOW(rankf, rankf_element(ar, rankf));
  }
  {
    // Input ranges other than an Array: a std::vector, a C array view, and a transformed view.
    const std::vector<int> vec{3, 1, 3, 2, 1};
    SHOW(sort_unique(vec));
    SHOW(median(vec));
    SHOW(rank_element(vec, 4));
    const int carray[] = {9, 7, 8};
    SHOW(median(CArrayView(carray)));
    SHOW(sort_unique(vec | views::transform([](int i) { return i * 10; })));
  }
  {
    // A custom comparison orders the result, and equivalence is still determined by operator==.
    SHOW(sort_unique(V(1, 3, 2, 3, 1), std::greater<>{}));
    const Array<string> words{"pear", "apple", "fig", "apple", "pear"};
    SHOW(sort_unique(words));
    SHOW(rank_element(words, 0), rank_element(words, 4));
  }
  {
    // An empty range yields an empty sorted list.
    SHOW(sort_unique(Array<int>{}).num());
  }
  {
    // The input range is not modified.
    const Array<int> ar{5, 1, 4, 2, 3};
    dummy_use(median(ar), rank_element(ar, 2), rankf_element(ar, .5), sort_unique(ar));
    assertx(ar == Array<int>{5, 1, 4, 2, 3});
  }
  {
    // Randomized lists, with many duplicate values, compared against a reference model.
    Random random{17};
    for_int(iter, 200) {
      const int n = 1 + int(random.get_unsigned(20));
      Array<int> ar(n);
      for_int(i, n) ar[i] = int(random.get_unsigned(8)) - 3;
      verify_against_sorted<int>(ar);
    }
  }
}
