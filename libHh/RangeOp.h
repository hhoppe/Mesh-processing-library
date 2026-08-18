// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_RANGEOP_H_
#define MESH_PROCESSING_LIBHH_RANGEOP_H_

#include "libHh/Range.h"

namespace hh {

// Set of algorithms for manipulating ranges (including containers), i.e. supporting begin() and end() functions.

// Extensions of functions from C++ <algorithm> as proposed by "Ranges" TS,
//  draft N4560 https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4560.pdf
// See https://en.cppreference.com/w/cpp/experimental/ranges#Ranges.

// Check unary predicate against elements; true if range is empty.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>> bool all_of(const Range& range, Pred p) {
  return std::all_of(ranges::begin(range), ranges::end(range), p);
}

// Check unary predicate against elements; false if range is empty.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>> bool any_of(const Range& range, Pred p) {
  return std::any_of(ranges::begin(range), ranges::end(range), p);
}

// Check unary predicate against elements; true if range is empty.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>>
bool none_of(const Range& range, Pred p) {
  return std::none_of(ranges::begin(range), ranges::end(range), p);
}

// Apply unary functor object to each element and returns functor.
template <typename Range, typename Func, typename = enable_if_range_t<Range>> Func for_each(Range&& range, Func func) {
  return std::for_each(ranges::begin(range), ranges::end(range), std::move(func));
}

// Return the address of the first element satisfying condition, or nullptr if none.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
Value* find_if(Range&& range, Pred p) {
  auto iter = std::find_if(ranges::begin(range), ranges::end(range), p);
  return iter == ranges::end(range) ? nullptr : &*iter;
}

// Return the address of the first element not satisfying condition, or nullptr if none.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
Value* find_if_not(Range&& range, Pred p) {
  auto iter = std::find_if_not(ranges::begin(range), ranges::end(range), p);
  return iter == ranges::end(range) ? nullptr : &*iter;
}

// Count the number of elements equal to specified one.
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
std::ptrdiff_t count(const Range& range, const Value& elem) {
  return std::count(ranges::begin(range), ranges::end(range), elem);
}

// Count the number of elements matching predicate.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>>
std::ptrdiff_t count_if(const Range& range, Pred p) {
  return std::count_if(ranges::begin(range), ranges::end(range), p);
}

// Return whether two ranges are equal element-wise.
template <typename Range1, typename Range2, typename Pred = std::equal_to<range_value_t<Range1>>,
          typename = enable_if_range_t<Range1>, typename = enable_if_range_t<Range2>>
bool equal(const Range1& range1, const Range2& range2, Pred p = Pred{}) {
  return std::equal(ranges::begin(range1), ranges::end(range1),  //
                    ranges::begin(range2), ranges::end(range2), p);
}

// Swap the contents of two ranges.
template <typename Range1, typename Range2, typename = enable_if_range_t<Range1>, typename = enable_if_range_t<Range2>>
void swap_ranges(Range1&& range1, Range2&& range2) {
  auto iter1 = ranges::begin(range1), itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2), itend2 = ranges::end(range2);
  if (0) {  // Draft N4560 swaps the first min(size(range1), size(range2)) elements.
    if (itend2 - iter2 < itend1 - iter1) {
      ranges::swap(iter1, iter2);
      ranges::swap(itend1, itend2);
    }
    std::swap_ranges(iter1, itend1, iter2);
  } else {  // Instead we require that they have the same size.
    auto it = std::swap_ranges(iter1, itend1, iter2);
    ASSERTX(it == itend2);  // Verify they have the same number of elements.
  }
}

// Assign the same value to all elements in a range.
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
Range fill(Range&& range, const Value& v) {
  // std::fill(ranges::begin(range), ranges::end(range), v);
  for (auto& e : range) e = v;
  return std::forward<Range>(range);
}

// Reverse the elements in a randomly accessible range.
template <typename Range, typename = enable_if_range_t<Range>> Range reverse(Range&& range_) {
  auto b = ranges::begin(range_), e = ranges::end(range_);
  ASSERTX(e >= b);  // Requires the range iterator to support random access.
  // std::reverse(b, e);
  size_t num = e - b;
  for (const size_t i : range(num / 2)) ranges::swap(b[i], b[num - 1 - i]);
  return std::forward<Range>(range_);
}
// Range reversed(const Range& range) { return reverse(clone(range)); }

// Rotate the elements in a randomly accessible range such that element middle becomes the new first element.
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
Range rotate(Range&& range, Value& middle) {
  std::rotate(ranges::begin(range), &middle, ranges::end(range));
  return std::forward<Range>(range);
}

// Sort the elements in a range (by default using less(a, b)).
template <typename Range, typename Comp = std::less<range_value_t<Range>>, typename = enable_if_range_t<Range>>
Range sort(Range&& range, Comp comp = Comp{}) {
  std::sort(ranges::begin(range), ranges::end(range), comp);
  return std::forward<Range>(range);
}
// Range sorted(const Range& range) { return sort(clone(range)); }

// Minimum value in a non-empty range (by default using less(a, b)).
template <typename Range, typename Comp = std::less<range_value_t<Range>>, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>>
Value min(const Range& range, Comp comp = Comp{}) {
  ASSERTXX(ranges::begin(range) != ranges::end(range));
  return *std::min_element(ranges::begin(range), ranges::end(range), comp);
}

// Maximum value in a non-empty range (using less(a, b)).
template <typename Range, typename Comp = std::less<range_value_t<Range>>, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>>
Value max(const Range& range, Comp comp = Comp{}) {
  ASSERTXX(ranges::begin(range) != ranges::end(range));
  return *std::max_element(ranges::begin(range), ranges::end(range), comp);
}

// Number of elements in a range (could also define size(Range) but for robustness that would require Concepts).
template <typename Range, typename = enable_if_range_t<Range>> std::ptrdiff_t distance(const Range& range) {
  return std::distance(ranges::begin(range), ranges::end(range));
}

// Also from std:
//  find(), find_end(), find_first_of(), adjacent_find(), mismatch(), equal(),
//  is_permutation(), search(), search_n(), copy(), copy_if(), copy_backward(), move(), move_backward(),
//  transform(), replace(), replace_if(), replace_copy(), replace_copy_if(),
//  generate(), remove(), remove_if(), remove_copy(), remove_copy_if(), unique(), unique_copy(),
//  reverse_copy(), rotate_copy(), shuffle(),
//  is_partitioned(), partition(), stable_partition(), partition_copy(), partition_move(),
//  stable_sort(), partial_sort(), partial_sort_copy(), is_sorted(), is_sorted_until(), nth_element(),
//  lower_bound(), upper_bound(), equal_range(), binary_search(),
//  merge(), merge_move(), inplace_merge(), includes(),
//  set_union(), set_intersection(), set_difference(), set_symmetric_difference(), *heap*(),
//  minmax(), minmax_element(), lexicographical_compare(), next_permutation(), prev_permutation().

// *** My custom range operations:

// Return the index of the first matching element, or die if not found.
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
int index(const Range& range, const Value& elem) {
  auto iter = std::find(ranges::begin(range), ranges::end(range), elem);
  if (iter == ranges::end(range)) assertnever(make_string(elem) + " not found in range");
  return assert_narrow_cast<int>(iter - ranges::begin(range));
}

// Return the index of the first matching element, or -1 if not found.
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
int maybe_index(const Range& range, const Value& elem) {
  auto iter = std::find(ranges::begin(range), ranges::end(range), elem);
  if (iter == ranges::end(range)) return -1;
  return assert_narrow_cast<int>(iter - ranges::begin(range));
}

// Higher-precision type to represent the mean of a set of elements.
template <typename T> struct mean_type {
  using type = std::conditional_t<
      !std::is_arithmetic_v<T>, T,
      std::conditional_t<std::is_floating_point_v<T>, double, std::conditional_t<sizeof(T) >= 4, double, float>>>;
};
template <typename T> using mean_type_t = typename mean_type<T>::type;

// Preferred floating-point type (either float or double) used to multiply a given element type.
template <typename T> struct factor_type {
  // Some user-defined types T (such as Vector4, Point, Vector) only support T * float, not T * double.
  // T * double is supported for all built-in arithmetic types; otherwise default is T * float.
  // User may override factor_type for a new type T supporting T * double.
  using type = std::conditional_t<std::is_arithmetic_v<T>, double, float>;
};
template <typename T> using factor_type_t = typename factor_type<T>::type;

// Index of minimum value in a non-empty range (using less(a, b)).
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
int arg_min(const Range& range) {
  ASSERTX(ranges::begin(range) != ranges::end(range));
  auto p = std::min_element(ranges::begin(range), ranges::end(range));
  return narrow_cast<int>(std::distance(ranges::begin(range), p));
}

// Index of maximum value in a non-empty range (using less(a, b)).
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
int arg_max(const Range& range) {
  ASSERTXX(ranges::begin(range) != ranges::end(range));
  auto p = std::max_element(ranges::begin(range), ranges::end(range));
  return narrow_cast<int>(std::distance(ranges::begin(range), p));
}

// Minimum over a non-empty range of values (using successive min(a, b) rather than less(a, b)).
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
Value transitive_min(const Range& range) {
  auto iter = ranges::begin(range), itend = ranges::end(range);
  ASSERTX(iter != itend);
  Value v = *iter;
  for (++iter; iter != itend; ++iter) v = min(v, *iter);
  return v;
}

// Maximum over a non-empty range of values (using successive min(a, b) rather than less(a, b)).
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
Value transitive_max(const Range& range) {
  auto iter = ranges::begin(range), itend = ranges::end(range);
  ASSERTX(iter != itend);
  Value v = *iter;
  for (++iter; iter != itend; ++iter) v = max(v, *iter);
  return v;
}

// Maximum absolute value in a non-empty range.
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
Value max_abs_element(const Range& range) {
  auto iter = ranges::begin(range), itend = ranges::end(range);
  ASSERTX(iter != itend);
  // return abs(*std::max_element(ibeg, iend, [](Value a, Value b) { return abs(a) < abs(b); }));
  Value v = static_cast<Value>(abs(*iter));
  for (++iter; iter != itend; ++iter) v = max(v, static_cast<Value>(abs(*iter)));
  return v;
}

// Sum of values in a range.
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Value>, DesiredType>>
SumType sum(const Range& range) {
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  // return std::accumulate(ranges::begin(range), ranges::end(range), SumType{});
  auto iter = ranges::begin(range), itend = ranges::end(range);
  if (iter == itend) return SumType{};
  SumType v = *iter;
  for (++iter; iter != itend; ++iter) v += *iter;
  return v;
}

// Average of values in a range.
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>,
          typename MeanType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Value>, DesiredType>>
MeanType mean(const Range& range) {
  static_assert(std::is_trivially_default_constructible_v<MeanType>);
  auto iter = ranges::begin(range), itend = ranges::end(range);
  if (iter == itend) {
    Warning("mean");
    return MeanType{};
  }
  MeanType v = *iter;
  size_t num = 1;
  for (++iter; iter != itend; ++iter) {
    v += *iter;
    num++;
  }
  using Factor = factor_type_t<Value>;
  return MeanType(v * (Factor(1) / num));
}

// Sum of squared values in a range (or zero if empty).
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Value>, DesiredType>>
SumType mag2(const Range& range) {
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  // return std::accumulate(ranges::begin(range), ranges::end(range), SumType{},
  //                        [](const SumType& sum, const Value& e) { return sum + square(e); });
  auto iter = ranges::begin(range), itend = ranges::end(range);
  if (iter == itend) return SumType{};
  SumType v = square(SumType(*iter));
  for (++iter; iter != itend; ++iter) v += square(SumType(*iter));
  return v;
}

// Root sum of squared values in a range.
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>,
          typename MeanType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Value>, DesiredType>>
MeanType mag(const Range& range) {
  return sqrt(mag2<MeanType>(range));
}

// Root mean square of values in a range.
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>,
          typename MeanType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Value>, DesiredType>>
MeanType rms(const Range& range) {
  static_assert(std::is_trivially_default_constructible_v<MeanType>);
  MeanType v{};
  size_t num = 0;
  for (const auto& e : range) {
    v += square(MeanType(e));
    num++;
  }
  if (num) {
    using Factor = factor_type_t<Value>;
    v = sqrt(MeanType(v * (Factor(1) / num)));
  } else {
    Warning("rms() of empty range");
  }
  return v;
}

// Variance of values in a range.
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>,
          typename MeanType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Value>, DesiredType>>
MeanType var(const Range& range) {
  static_assert(std::is_trivially_default_constructible_v<MeanType>);
  constexpr MeanType zero{};
  MeanType v{}, v2{};
  size_t num = 0;
  for (const auto& e : range) {
    v += e;
    v2 += square(MeanType(e));
    num++;
  }
  if (num < 2) {
    Warning("var() of fewer than 2 elements");
    return zero;
  }
  using Factor = factor_type_t<Value>;
  return max(MeanType((v2 - v * v * (Factor(1) / num)) * (Factor(1) / (num - Factor(1)))), zero);
}

// Product of values in a non-empty range.
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Value>, DesiredType>>
SumType product(const Range& range) {
  auto iter = ranges::begin(range), itend = ranges::end(range);
  ASSERTX(iter != itend);
  // return std::accumulate(ranges::begin(range), ranges::end(range), SumType{1},
  //                        [](const SumType& sum, const Value& e) { return sum * e; });
  SumType v = *iter;
  for (++iter; iter != itend; ++iter) v *= *iter;
  return v;
}

// Are all elements exactly zero?
template <typename Range, typename = enable_if_range_t<Range>> bool is_zero(const Range& range) {
  for (const auto& e : range)
    if (e) return false;
  return true;
}

// Does it have unit norm?
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Value = range_value_t<Range>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Value>, DesiredType>>
bool is_unit(const Range& range, SumType tolerance = 1e-4f) {
  return abs(mag2<SumType>(range) - 1.f) <= tolerance;
}

// Modify the range to have unit norm (or die if input has zero norm).
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
Range normalize(Range&& range) {
  Value v = static_cast<Value>(Value{1.f} / assertx(mag(range)));
  for (auto& e : range) e = static_cast<Value>(e * v);
  return std::forward<Range>(range);
}
// Range normalized(const Range& range) { return normalize(clone(range)); }

// Round the values in a range to the nearest 1/fac increment (by default fac == 1e5f).
template <typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
Range round_elements(Range&& range, Value fac = 1e5f) {
  static_assert(std::is_floating_point_v<Value>);
  for (auto& e : range) e = round_fraction_digits(e, fac);
  return std::forward<Range>(range);
}

// Compute the sum of squared differences of corresponding elements of two ranges.
template <typename DesiredType = void, typename Range1, typename Range2, typename = enable_if_range_t<Range1>,
          typename = enable_if_range_t<Range2>, typename Value = range_value_t<Range1>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Value>, DesiredType>>
SumType dist2(const Range1& range1, const Range2& range2) {
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  auto iter1 = ranges::begin(range1), itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2), itend2 = ranges::end(range2);
  SumType v{};
  for (; iter1 != itend1; ++iter1, ++iter2) v += square(SumType(*iter1) - SumType(*iter2));
  ASSERTX(iter2 == itend2);  // Verify they have the same number of elements.
  return v;
}

// Compute the Euclidean distance between two ranges interpreted as vectors.
template <typename DesiredType = void, typename Range1, typename Range2, typename = enable_if_range_t<Range1>,
          typename = enable_if_range_t<Range2>, typename Value = range_value_t<Range1>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Value>, DesiredType>>
SumType dist(const Range1& range1, const Range2& range2) {
  return sqrt(dist2<SumType>(range1, range2));
}

// Compute the inner product of two ranges.
template <typename DesiredType = void, typename Range1, typename Range2, typename = enable_if_range_t<Range1>,
          typename = enable_if_range_t<Range2>, typename Value = range_value_t<Range1>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Value>, DesiredType>>
SumType dot(const Range1& range1, const Range2& range2) {
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  auto iter1 = ranges::begin(range1), itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2), itend2 = ranges::end(range2);
  SumType v{};
  for (; iter1 != itend1; ++iter1, ++iter2) v += SumType(*iter1) * SumType(*iter2);
  ASSERTX(iter2 == itend2);  // Verify they have the same number of elements.
  return v;
}

// Compare two ranges of algebraic types lexicographically; ret -1, 0, 1 based on sign of range1 - range2.
template <typename Range1, typename Range2, typename = enable_if_range_t<Range1>, typename = enable_if_range_t<Range2>>
int compare(const Range1& range1, const Range2& range2) {
  auto iter1 = ranges::begin(range1), itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2), itend2 = ranges::end(range2);
  for (; iter1 != itend1; ++iter1, ++iter2) {
    auto d = *iter1 - *iter2;
    if (d) return d < 0 ? -1 : +1;
  }
  ASSERTX(iter2 == itend2);  // Verify they have the same number of elements.
  return 0;
}

// Similar comparison, but ignore differences smaller than tolerance.
template <typename Range1, typename Range2, typename = enable_if_range_t<Range1>, typename = enable_if_range_t<Range2>,
          typename Value = range_value_t<Range1>>
int compare(const Range1& range1, const Range2& range2, const Value& tolerance) {
  auto iter1 = ranges::begin(range1), itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2), itend2 = ranges::end(range2);
  for (; iter1 != itend1; ++iter1, ++iter2) {
    auto d = *iter1 - *iter2;
    if (d < -tolerance) return -1;
    if (d > tolerance) return +1;
  }
  ASSERTX(iter2 == itend2);  // Verify they have the same number of elements.
  return 0;
}

// Check if a container contains an element.
template <typename Range, typename = enable_if_range_t<Range>>
bool contains(const Range& range, const range_value_t<Range>& elem) {
  for (const auto& e : range)
    if (e == elem) return true;
  return false;
}

// For any container Range (e.g. Vec, Array, PArray, Grid, SGrid) supporting transformed(Range&, [](const T&) -> T):

// Convert all elements of the container to the new type U, e.g. convert<float>(V(1, 2)) == V(1.f, 2.f).
// Be careful to possibly use floor() before convert<int>() to avoid rounding negative values towards zero.
template <typename U, typename Range, typename = enable_if_range_t<Range>, typename Value = range_value_t<Range>>
auto convert(const Range& c) {
  return transformed(c, [](const Value& e) { return static_cast<U>(e); });
}

namespace details {

template <typename Iterator, typename Func> struct TransformedIterator {
  using type = TransformedIterator<Iterator, Func>;
  using value_type = std::decay_t<decltype(std::declval<Func>()(*std::declval<Iterator>()))>;
  using difference_type = typename std::iterator_traits<Iterator>::difference_type;
  Iterator _iter;
  const Func* _func{};
  bool operator==(const type& rhs) const { return _iter == rhs._iter; }
  decltype(auto) operator*() const { return (*_func)(*_iter); }
  type& operator++() { return ++_iter, *this; }
  type operator++(int) { return postfix_increment(*this); }
};

template <typename Range, typename Func> struct TransformedRange {
  Range _range;
  Func _func;
  auto begin() const {
    using Iterator = std::decay_t<decltype(ranges::begin(_range))>;
    return TransformedIterator<Iterator, Func>{ranges::begin(_range), &_func};
  }
  auto end() const {
    using Iterator = std::decay_t<decltype(ranges::begin(_range))>;
    return TransformedIterator<Iterator, Func>{ranges::end(_range), &_func};
  }
  auto size() const { return ranges::size(_range); }
};

}  // namespace details

// Return a view range in which all elements are mapped through a function `func`.
template <typename Range, typename Func, typename = enable_if_range_t<Range>>
auto transform(Range&& range, Func func = Func{}) {
  return details::TransformedRange<Range, Func>{std::forward<Range>(range), std::move(func)};
}

namespace details {

template <typename Iterator1, typename Iterator2> struct ConcatenatedIterator {
  using type = ConcatenatedIterator<Iterator1, Iterator2>;
  using value_type = typename std::iterator_traits<Iterator1>::value_type;
  using difference_type = typename std::iterator_traits<Iterator1>::difference_type;
  static_assert(std::is_same_v<value_type, typename std::iterator_traits<Iterator2>::value_type>);
  Iterator1 _begin1;
  Iterator1 _end1;
  Iterator2 _begin2;
  bool operator==(const type& rhs) const { return _begin1 == rhs._begin1 && _begin2 == rhs._begin2; }
  decltype(auto) operator*() const { return _begin1 != _end1 ? *_begin1 : *_begin2; }
  type& operator++() {
    if (_begin1 != _end1)
      ++_begin1;
    else
      ++_begin2;
    return *this;
  }
  type operator++(int) { return postfix_increment(*this); }
};

template <typename Range1, typename Range2> struct ConcatenatedRange {
  Range1 _range1;
  Range2 _range2;
  auto begin() const {
    using Iterator1 = std::decay_t<decltype(ranges::begin(_range1))>;
    using Iterator2 = std::decay_t<decltype(ranges::begin(_range2))>;
    return ConcatenatedIterator<Iterator1, Iterator2>{ranges::begin(_range1), ranges::end(_range1),
                                                      ranges::begin(_range2)};
  }
  auto end() const {
    using Iterator1 = std::decay_t<decltype(ranges::begin(_range1))>;
    using Iterator2 = std::decay_t<decltype(ranges::begin(_range2))>;
    return ConcatenatedIterator<Iterator1, Iterator2>{ranges::end(_range1), ranges::end(_range1),
                                                      ranges::end(_range2)};
  }
  auto size() const { return ranges::size(_range1) + ranges::size(_range2); }
};

}  // namespace details

// Return a view range that concatenates the elements of two or more ranges.
template <typename Range1, typename Range2, typename... Ranges>
[[HH_NO_DANGLING]] auto concatenate(Range1&& range1, Range2&& range2, Ranges&&... ranges_) {
  // ("Dangling" is a false positive, due to the fact that there are >=2 reference-binding parameters.)
  if constexpr (sizeof...(Ranges) == 0)
    return details::ConcatenatedRange<Range1, Range2>{std::forward<Range1>(range1), std::forward<Range2>(range2)};
  else
    return concatenate(std::forward<Range1>(range1),
                       concatenate(std::forward<Range2>(range2), std::forward<Ranges>(ranges_)...));
}

namespace details {

template <typename Iterator, typename Index> struct EnumeratedIterator {
  using type = EnumeratedIterator<Iterator, Index>;
  using value_type = typename std::iterator_traits<Iterator>::value_type;
  using difference_type = typename std::iterator_traits<Iterator>::difference_type;
  Iterator _iter;
  Index _index{0};
  bool operator==(const type& rhs) const { return _iter == rhs._iter; }
  // Note: "std::make_tuple(_index, *_iter)" would return a copied element.
  // Here we allow "for (auto&& [i, e] : enumerate(array)) e = int(i);".
  decltype(auto) operator*() const { return std::tuple<Index, decltype(*_iter)>(_index, *_iter); }
  type& operator++() { return ++_iter, ++_index, *this; }
  type operator++(int) { return postfix_increment(*this); }
};

template <typename Range, typename Index> struct EnumeratedRange {
  Range _range;
  auto begin() const {
    using Iterator = std::decay_t<decltype(ranges::begin(_range))>;
    return EnumeratedIterator<Iterator, Index>{ranges::begin(_range), 0};
  }
  auto end() const {
    using Iterator = std::decay_t<decltype(ranges::begin(_range))>;
    return EnumeratedIterator<Iterator, Index>{ranges::end(_range), 0};
  }
  auto size() const { return ranges::size(_range); }
};

}  // namespace details

// Return a view range containing a sequence of tuples [index, element] for all elements in `range`.
template <typename Index = size_t, typename Range, typename = enable_if_range_t<Range>> auto enumerate(Range&& range) {
  return details::EnumeratedRange<Range, Index>{std::forward<Range>(range)};
}

namespace details {

template <typename Iterator, typename Func> struct FilteredIterator {
  using type = FilteredIterator<Iterator, Func>;
  using value_type = typename std::iterator_traits<Iterator>::value_type;
  using difference_type = typename std::iterator_traits<Iterator>::difference_type;
  Iterator _iter;
  Iterator _end;
  const Func* _func{};
  bool operator==(const type& rhs) const { return _iter == rhs._iter; }
  decltype(auto) operator*() const { return *_iter; }
  type& operator++() {
    ++_iter;
    while (_iter != _end && !(*_func)(*_iter)) ++_iter;
    return *this;
  }
  type operator++(int) { return postfix_increment(*this); }
};

template <typename Range, typename Func> struct FilteredRange {
  Range _range;
  Func _func;
  // Type of the expression `_range` in a const member function (const does not apply if Range is a reference).
  using CRange = std::add_lvalue_reference_t<std::add_const_t<Range>>;
  using Iterator = std::decay_t<range_iterator_t<CRange>>;
  auto begin() const -> FilteredIterator<Iterator, Func> {
    auto iterator = FilteredIterator<Iterator, Func>{ranges::begin(_range), ranges::end(_range), &_func};
    while (iterator != this->end() && !_func(*iterator)) ++iterator;
    return iterator;
  }
  auto end() const -> FilteredIterator<Iterator, Func> {
    return FilteredIterator<Iterator, Func>{ranges::end(_range), ranges::end(_range), &_func};
  }
  // Note that size() is unknown.
};

}  // namespace details

// Return a view range that filters the elements in `range` according to a predicate `func`.
template <typename Range, typename Func, typename = enable_if_range_t<Range>>
auto filter(Range&& range, Func func = Func{}) {
  return details::FilteredRange<Range, Func>{std::forward<Range>(range), std::move(func)};
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RANGEOP_H_
