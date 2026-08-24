// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ARRAYOP_H_
#define MESH_PROCESSING_LIBHH_ARRAYOP_H_

#include "libHh/Array.h"
#include "libHh/RangeOp.h"  // sort()
#include "libHh/Vec.h"      // Vec2<>

namespace hh {

// Return a sorted, uniquified array of values gathered from a range.  Caller can subsequently call shrink_to_fit().
template <ranges::input_range R, typename Comp = std::less<>>
Array<range_value_t<R>> sort_unique(const R& range, Comp comp = Comp{}) {
  Array ar(range);
  sort(ar, comp);
  auto subspan_of_extras = ranges::unique(ar);
  ar.sub(narrow_cast<int>(ranges::size(subspan_of_extras)));
  return ar;
}

// Return the two closest values to the median of a list (or the same value twice if the list length is odd).
template <ranges::input_range R> auto median_two(const R& range) -> Vec2<range_value_t<R>> {
  Array ar(range);
  assertx(ar.num());
  const int median_index = (ar.num() - 1) / 2;
  ranges::nth_element(ar, ar.begin() + median_index);  // Place median element at median location.
  const auto val0 = ar[median_index];
  // List is partially sorted about the median value, so find the min of the second half.
  const auto val1 = ar.num() % 2 == 1 ? val0 : min(ar.slice(median_index + 1, ar.num()));
  return V(val0, val1);
}

// Return the median value of a list (or the mean of the two nearest values if the list length is even).
template <ranges::input_range R> auto median(const R& range) -> mean_type_t<range_value_t<R>> {
  return mean(median_two(range));
}

// Return the element with specified rank within range (where 0 <= rank < size(range) and rank == 0 is min element).
template <ranges::input_range R> range_value_t<R> rank_element(const R& range, int rank) {
  Array ar(range);
  assertx(ar.num());
  assertx(ar.ok(rank));
  ranges::nth_element(ar, ar.begin() + rank);  // Place rank element at rank location.
  return ar[rank];
}

// Return element with fractional ranking within range (where 0. <= rankf <= 1. and rankf == 0. is min element).
template <ranges::input_range R> range_value_t<R> rankf_element(const R& range, double rankf) {
  assertx(rankf >= 0. && rankf <= 1.);
  Array ar(range);
  assertx(ar.num());
  int rank = int(floor(rankf * ar.num()));
  if (rank == ar.num()) rank--;
  ranges::nth_element(ar, ar.begin() + rank);  // Place rank element at rank location.
  return ar[rank];
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_ARRAYOP_H_
