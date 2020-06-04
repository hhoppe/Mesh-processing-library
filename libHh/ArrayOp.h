// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ARRAYOP_H_
#define MESH_PROCESSING_LIBHH_ARRAYOP_H_

#include "libHh/Advanced.h"  // do_in_order
#include "libHh/Array.h"
#include "libHh/RangeOp.h"  // sort()
#include "libHh/Vec.h"      // Vec2<>

namespace hh {

// Concatenate two or more array views.
template <typename T, typename... A> Array<T> concat(CArrayView<T> ar1, A&&... arr) {
  Array<T> ar;
  int i = ar1.num();
  do_in_order{i += arr.num()...};
  ar.reserve(i);
  // ar.reserve(ar1.num() + narrow_cast<int>(sum(CArrayView<int>{arr.num()...})));  // requires sum()
  // Vec<int, sizeof...(arr)> t(arr.num()...); ar.reserve(ar1.num() + narrow_cast<int>(sum(t)));
  ar.push_array(ar1);
  do_in_order{(ar.push_array(arr), 0)...};
  return ar;
}

// Return a sorted, uniquified array of values gathered from a range.
template <typename R, typename = enable_if_range_t<R>> Array<iterator_t<R>> sort_unique(const R& range) {
  using T = iterator_t<R>;
  using std::begin;
  using std::end;
  Array<T> ar(begin(range), end(range));
  sort(ar);
  T* last = std::unique(ar.begin(), ar.end());
  ar.sub(narrow_cast<int>(ar.end() - last));  // leave it to client to do shrink_to_fit()
  return ar;
}

// Return the two closest values to the median of a list (or the same value twice if the list length is odd).
template <typename R, typename = enable_if_range_t<R>> Vec2<iterator_t<R>> median_two(const R& range) {
  using T = iterator_t<R>;
  using std::begin;
  using std::end;
  Array<T> ar(begin(range), end(range));
  assertx(ar.num());
  const int median_index = ar.num() / 2;
  std::nth_element(ar.begin(), &ar[median_index], ar.end());  // place median element at median location
  T val0 = ar[median_index];
  // List is partially sorted about the median value, so find the min of the second half.
  T val1 = ar.num() % 2 == 1 ? val0 : *std::min_element(&ar[median_index + 1], ar.end());
  return V(val0, val1);
}

// Return the median value of a list (or the mean of the two nearest values if the list length is even).
template <typename R, typename = enable_if_range_t<R>> mean_type_t<iterator_t<R>> median(const R& range) {
  return mean(median_two(range));
}

// Return the element with specified rank within range (where 0 <= rank < size(range) and rank == 0 is min element).
template <typename R, typename = enable_if_range_t<R>> iterator_t<R> rank_element(const R& range, int rank) {
  using T = iterator_t<R>;
  using std::begin;
  using std::end;
  Array<T> ar(begin(range), end(range));
  assertx(ar.num());
  assertx(ar.ok(rank));
  std::nth_element(ar.begin(), &ar[rank], ar.end());  // place rank element at rank location
  return ar[rank];
}

// Return element with fractional ranking within range (where 0. <= rankf <= 1. and rankf == 0. is min element).
template <typename R, typename = enable_if_range_t<R>> iterator_t<R> rankf_element(const R& range, double rankf) {
  assertx(rankf >= 0. && rankf <= 1.);
  int num = narrow_cast<int>(distance(range));
  int rank = static_cast<int>(floor(rankf * num));
  if (rank == num) rank--;
  return rank_element(range, rank);
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_ARRAYOP_H_
