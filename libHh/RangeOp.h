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
template <ranges::range R, typename Pred> bool all_of(const R& range, Pred pred) {
  return std::all_of(ranges::begin(range), ranges::end(range), pred);
}

// Check unary predicate against elements; false if range is empty.
template <ranges::range R, typename Pred> bool any_of(const R& range, Pred pred) {
  return std::any_of(ranges::begin(range), ranges::end(range), pred);
}

// Check unary predicate against elements; true if range is empty.
template <ranges::range R, typename Pred> bool none_of(const R& range, Pred pred) {
  return std::none_of(ranges::begin(range), ranges::end(range), pred);
}

// Apply unary functor object to each element and returns functor.
template <ranges::range R, typename Func> Func for_each(R&& range, Func func) {
  return std::for_each(ranges::begin(range), ranges::end(range), std::move(func));
}

// Return the address of the first element satisfying condition, or nullptr if none.
template <ranges::range R, typename Pred> auto find_if(R&& range, Pred pred) -> range_value_t<R>* {
  auto iter = std::find_if(ranges::begin(range), ranges::end(range), pred);
  return iter == ranges::end(range) ? nullptr : &*iter;
}

// Return the address of the first element not satisfying condition, or nullptr if none.
template <ranges::range R, typename Pred> auto find_if_not(R&& range, Pred pred) -> range_value_t<R> {
  auto iter = std::find_if_not(ranges::begin(range), ranges::end(range), pred);
  return iter == ranges::end(range) ? nullptr : &*iter;
}

// Count the number of elements equal to specified one.
template <ranges::range R> std::ptrdiff_t count(const R& range, const range_value_t<R>& elem) {
  return std::count(ranges::begin(range), ranges::end(range), elem);
}

// Count the number of elements matching predicate.
template <ranges::range R, typename Pred> std::ptrdiff_t count_if(const R& range, Pred pred) {
  return std::count_if(ranges::begin(range), ranges::end(range), pred);
}

// Return whether two ranges are equal element-wise.
template <ranges::range R1, ranges::range R2, typename Pred = std::equal_to<range_value_t<R1>>>
bool equal(const R1& range1, const R2& range2, Pred pred = Pred{}) {
  return std::equal(ranges::begin(range1), ranges::end(range1), ranges::begin(range2), ranges::end(range2), pred);
}

// Swap the contents of two ranges.
template <ranges::range R1, ranges::range R2> void swap_ranges(R1&& range1, R2&& range2) {
  auto iter1 = ranges::begin(range1);
  const auto itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2);
  const auto itend2 = ranges::end(range2);
#if 0  // ??
  // Draft N4560 swaps the first min(size(range1), size(range2)) elements.
  if (itend2 - iter2 < itend1 - iter1) {
    ranges::swap(iter1, iter2);
    ranges::swap(itend1, itend2);
  }
  std::swap_ranges(iter1, itend1, iter2);
#else
  // Instead we require that they have the same size.
  auto it = std::swap_ranges(iter1, itend1, iter2);
  ASSERTX(it == itend2);  // Verify they have the same number of elements.
#endif
}

// Assign the same value to all elements in a range.
template <ranges::range R> R fill(R&& range, const range_value_t<R>& v) {
  // std::fill(ranges::begin(range), ranges::end(range), v);
  for (auto& e : range) e = v;
  return std::forward<R>(range);
}

// Reverse the elements in a randomly accessible range.
template <ranges::range R> R reverse(R&& range_) {
  auto b = ranges::begin(range_);
  const auto e = ranges::end(range_);
  ASSERTX(e >= b);  // Requires the range iterator to support random access. ??
  // std::reverse(b, e);
  size_t num = e - b;
  for (const size_t i : range(num / 2)) ranges::swap(b[i], b[num - 1 - i]);
  return std::forward<R>(range_);
}
// R reversed(const R& range) { return reverse(clone(range)); }

// Rotate the elements in a randomly accessible range such that element middle becomes the new first element.
template <ranges::range R> R rotate(R&& range, range_value_t<R>& middle) {
  std::rotate(ranges::begin(range), &middle, ranges::end(range));
  return std::forward<R>(range);
}

// Sort the elements in a range (by default using less(a, b)).
template <ranges::range R, typename Comp = std::less<>> requires std::sortable<ranges::iterator_t<R>, Comp>
R sort(R&& range, Comp comp = Comp{}) {
  std::sort(ranges::begin(range), ranges::end(range), comp);
  return std::forward<R>(range);
}
// R sorted(const R& range) { return sort(clone(range)); }

// Minimum value in a non-empty range (by default using less(a, b)).
template <typename R, typename Comp = std::less<>>
requires ranges::range<const R&> && std::indirect_strict_weak_order<Comp, ranges::iterator_t<const R&>>
auto min(const R& range, Comp comp = Comp{}) -> ranges::range_value_t<const R&> {
  ASSERTXX(ranges::begin(range) != ranges::end(range));
  return *std::min_element(ranges::begin(range), ranges::end(range), comp);
}

// Maximum value in a non-empty range (using less(a, b)).
template <typename R, typename Comp = std::less<>>
requires ranges::range<const R&> && std::indirect_strict_weak_order<Comp, ranges::iterator_t<const R&>>
auto max(const R& range, Comp comp = Comp{}) -> range_value_t<R> {
  ASSERTXX(ranges::begin(range) != ranges::end(range));
  return *std::max_element(ranges::begin(range), ranges::end(range), comp);
}

// Number of elements in a range (could also define size(R) but for robustness that would require Concepts).
template <ranges::range R> std::ptrdiff_t distance(const R& range) {
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
template <ranges::range R> int index(const R& range, const range_value_t<R>& elem) {
  auto iter = std::find(ranges::begin(range), ranges::end(range), elem);
  if (iter == ranges::end(range)) assertnever(make_string(elem) + " not found in range");
  return assert_narrow_cast<int>(iter - ranges::begin(range));
}

// Return the index of the first matching element, or -1 if not found.
template <ranges::range R> int maybe_index(const R& range, const range_value_t<R>& elem) {
  auto iter = std::find(ranges::begin(range), ranges::end(range), elem);
  if (iter == ranges::end(range)) return -1;
  return assert_narrow_cast<int>(iter - ranges::begin(range));
}

// Index of minimum value in a non-empty range (using less(a, b)).
template <ranges::range R> int arg_min(const R& range) {
  ASSERTX(ranges::begin(range) != ranges::end(range));
  auto iter = std::min_element(ranges::begin(range), ranges::end(range));
  return narrow_cast<int>(std::distance(ranges::begin(range), iter));
}

// Index of maximum value in a non-empty range (using less(a, b)).
template <ranges::range R> int arg_max(const R& range) {
  ASSERTXX(ranges::begin(range) != ranges::end(range));
  auto iter = std::max_element(ranges::begin(range), ranges::end(range));
  return narrow_cast<int>(std::distance(ranges::begin(range), iter));
}

// Minimum over a non-empty range of values (using successive min(a, b) rather than less(a, b)).
template <ranges::range R> range_value_t<R> transitive_min(const R& range) {
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
  ASSERTX(iter != itend);
  range_value_t<R> v = *iter;
  for (++iter; iter != itend; ++iter) v = min(v, *iter);
  return v;
}

// Maximum over a non-empty range of values (using successive min(a, b) rather than less(a, b)).
template <ranges::range R> range_value_t<R> transitive_max(const R& range) {
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
  ASSERTX(iter != itend);
  range_value_t<R> v = *iter;
  for (++iter; iter != itend; ++iter) v = max(v, *iter);
  return v;
}

// Maximum absolute value in a non-empty range.
template <ranges::range R> range_value_t<R> max_abs_element(const R& range) {
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
  ASSERTX(iter != itend);
  // return abs(*std::max_element(ibeg, iend, [](auto a, auto b) { return abs(a) < abs(b); }));
  auto v = static_cast<range_value_t<R>>(abs(*iter));
  for (++iter; iter != itend; ++iter) v = max(v, static_cast<range_value_t<R>>(abs(*iter)));
  return v;
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

template <typename DesiredType, typename R>
using mean_type_for_t = std::conditional_t<std::is_void_v<DesiredType>, mean_type_t<range_value_t<R>>, DesiredType>;

template <typename DesiredType, typename R>
using sum_type_for_t = std::conditional_t<std::is_void_v<DesiredType>, sum_type_t<range_value_t<R>>, DesiredType>;

// Sum of values in a range (widening the accumulator, unless DesiredType is specified).
template <typename DesiredType = void, ranges::range R> sum_type_for_t<DesiredType, R> sum(const R& range) {
  using SumType = sum_type_for_t<DesiredType, R>;
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  // return implicit_cast<SumType>(ranges::fold_left(range, SumType{}, std::plus<>{}));
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
  SumType v;
  if (iter == itend) {
    v = SumType{};
  } else {
    v = *iter;
    for (++iter; iter != itend; ++iter) v += *iter;
  }
  return v;
}

// Average of values in a range.
template <typename DesiredType = void, ranges::range R> auto mean(const R& range) -> mean_type_for_t<DesiredType, R> {
  using MeanType = mean_type_for_t<DesiredType, R>;
  static_assert(std::is_trivially_default_constructible_v<MeanType>);
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
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
  using FactorType = factor_type_t<range_value_t<R>>;
  return MeanType(v * (FactorType(1) / num));
}

// Sum of squared values in a range (or zero if empty).
template <typename DesiredType = void, ranges::range R> auto mag2(const R& range) -> sum_type_for_t<DesiredType, R> {
  using SumType = sum_type_for_t<DesiredType, R>;
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  // const auto op = [](const SumType& sum, const auto& e) { return sum + square(SumType(e)); };
  // return implicit_cast<SumType>(ranges::fold_left(range, SumType{}, op));
  // Or also:
  // const auto squared = ranges::views::transform(range, [](const auto& e) { return square(SumType(e)); });
  // return implicit_cast<SumType>(ranges::fold_left(squared, SumType{}, std::plus<>{}));
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
  if (iter == itend) return SumType{};
  SumType v = square(SumType(*iter));
  for (++iter; iter != itend; ++iter) v += square(SumType(*iter));
  return v;
}

// Root sum of squared values in a range.
template <typename DesiredType = void, ranges::range R> auto mag(const R& range) -> mean_type_for_t<DesiredType, R> {
  using MeanType = mean_type_for_t<DesiredType, R>;
  return sqrt(mag2<MeanType>(range));
}

// Root mean square of values in a range.
template <typename DesiredType = void, ranges::range R> auto rms(const R& range) -> mean_type_for_t<DesiredType, R> {
  using MeanType = mean_type_for_t<DesiredType, R>;
  static_assert(std::is_trivially_default_constructible_v<MeanType>);
  MeanType v{};
  size_t num = 0;
  for (const auto& e : range) {
    v += square(MeanType(e));
    num++;
  }
  if (num) {
    using FactorType = factor_type_t<range_value_t<R>>;
    v = sqrt(MeanType(v * (FactorType(1) / num)));
  } else {
    Warning("rms() of empty range");
  }
  return v;
}

// Variance of values in a range.
template <typename DesiredType = void, ranges::range R> auto var(const R& range) -> mean_type_for_t<DesiredType, R> {
  using MeanType = mean_type_for_t<DesiredType, R>;
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
  using FactorType = factor_type_t<range_value_t<R>>;
  return max(MeanType((v2 - v * v * (FactorType(1) / num)) * (FactorType(1) / (num - FactorType(1)))), zero);
}

// Product of values in a non-empty range.
template <typename DesiredType = void, ranges::range R>
auto product(const R& range) -> sum_type_for_t<DesiredType, R> {
  using ProductType = sum_type_for_t<DesiredType, R>;
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
  ASSERTX(iter != itend);
  // static_assert(std::is_trivially_default_constructible_v<ProductType>);
  // return implicit_cast<ProductType>(ranges::fold_left(range, ProductType{1}, std::multiplies<>{}));
  // - Or also (not supporting empty range):
  // const auto op = [](const ProductType& product, const auto& e) { return product * ProductType(e); };
  // return implicit_cast<ProductType>(*ranges::fold_left_first(range, op));
  ProductType v = *iter;
  for (++iter; iter != itend; ++iter) v *= *iter;
  return v;
}

// Are all elements exactly zero?
template <ranges::range R> bool is_zero(const R& range) {
  for (const auto& e : range)
    if (e) return false;
  return true;
}

// Does it have unit norm?
template <typename DesiredType = void, ranges::range R>
bool is_unit(const R& range, range_value_t<R> tolerance = 1e-4f) {
  using SumType = sum_type_for_t<DesiredType, R>;
  return abs(mag2<SumType>(range) - 1.f) <= tolerance;
}

// Modify the range to have unit norm (or die if input has zero norm).
template <ranges::range R> R normalize(R&& range) {
  using Value = range_value_t<R>;
  Value v = static_cast<Value>(Value{1.f} / assertx(mag(range)));
  for (auto& e : range) e = static_cast<Value>(e * v);
  return std::forward<R>(range);
}
// R normalized(const R& range) { return normalize(clone(range)); }

// Round the values in a range to the nearest 1/fac increment (by default fac == 1e5f).
template <ranges::range R> R round_elements(R&& range, range_value_t<R> fac = 1e5f) {
  static_assert(std::is_floating_point_v<range_value_t<R>>);
  for (auto& e : range) e = round_fraction_digits(e, fac);
  return std::forward<R>(range);
}

// Compute the sum of squared differences of corresponding elements of two ranges.
template <typename DesiredType = void, ranges::range R1, ranges::range R2>
auto dist2(const R1& range1, const R2& range2) -> sum_type_for_t<DesiredType, R1> {
  using SumType = sum_type_for_t<DesiredType, R1>;
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  auto iter1 = ranges::begin(range1);
  const auto itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2);
  const auto itend2 = ranges::end(range2);
  SumType v{};
  for (; iter1 != itend1; ++iter1, ++iter2) v += square(SumType(*iter1) - SumType(*iter2));
  ASSERTX(iter2 == itend2);  // Verify they have the same number of elements.
  return v;
}

// Compute the Euclidean distance between two ranges interpreted as vectors.
template <typename DesiredType = void, ranges::range R1, ranges::range R2>
auto dist(const R1& range1, const R2& range2) -> mean_type_for_t<DesiredType, R1> {
  using MeanType = mean_type_for_t<DesiredType, R1>;
  return sqrt(dist2<MeanType>(range1, range2));
}

// Compute the inner product of two ranges.
template <typename DesiredType = void, ranges::range R1, ranges::range R2>
auto dot(const R1& range1, const R2& range2) -> sum_type_for_t<DesiredType, R1> {
  using SumType = sum_type_for_t<DesiredType, R1>;
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  auto iter1 = ranges::begin(range1);
  const auto itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2);
  const auto itend2 = ranges::end(range2);
  SumType v{};
  for (; iter1 != itend1; ++iter1, ++iter2) v += SumType(*iter1) * SumType(*iter2);
  ASSERTX(iter2 == itend2);  // Verify they have the same number of elements.
  return v;
}

// Compare two ranges of algebraic types lexicographically; ret -1, 0, 1 based on sign of range1 - range2.
// Use <=> ??
template <ranges::range R1, ranges::range R2> int compare(const R1& range1, const R2& range2) {
  auto iter1 = ranges::begin(range1);
  const auto itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2);
  const auto itend2 = ranges::end(range2);
  for (; iter1 != itend1; ++iter1, ++iter2) {
    auto d = *iter1 - *iter2;
    if (d) return d < 0 ? -1 : +1;
  }
  ASSERTX(iter2 == itend2);  // Verify they have the same number of elements.
  return 0;
}

// Similar comparison, but ignore differences smaller than tolerance.
template <ranges::range R1, ranges::range R2>
int compare(const R1& range1, const R2& range2, const range_value_t<R1>& tolerance) {
  auto iter1 = ranges::begin(range1);
  const auto itend1 = ranges::end(range1);
  auto iter2 = ranges::begin(range2);
  const auto itend2 = ranges::end(range2);
  for (; iter1 != itend1; ++iter1, ++iter2) {
    auto d = *iter1 - *iter2;
    if (d < -tolerance) return -1;
    if (d > tolerance) return +1;
  }
  ASSERTX(iter2 == itend2);  // Verify they have the same number of elements.
  return 0;
}

// Check if a container contains an element.
template <ranges::range R> bool contains(const R& range, const range_value_t<R>& elem) {
  for (const auto& e : range)
    if (e == elem) return true;
  return false;
}

// For any container R (e.g. Vec, Array, PArray, Grid, SGrid) supporting transformed(R&, [](const T&) -> T):

// Convert all elements of the container to the new type U, e.g. convert<float>(V(1, 2)) == V(1.f, 2.f).
// Be careful to possibly use floor() before convert<int>() to avoid rounding negative values towards zero.
template <typename U, ranges::range R> auto convert(const R& c) {
  return transformed(c, [](const range_value_t<R>& e) { return static_cast<U>(e); });
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

template <ranges::range R, typename Func> struct TransformedRange {
  R _range;
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
// requires??
template <ranges::range R, typename Func> auto transform(R&& range, Func func = Func{}) {
  return details::TransformedRange<R, Func>{std::forward<R>(range), std::move(func)};
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

template <ranges::range R1, ranges::range R2> struct ConcatenatedRange {
  R1 _range1;
  R2 _range2;
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
template <ranges::range R1, ranges::range R2, typename... Rs>
[[HH_NO_DANGLING]] auto concatenate(R1&& range1, R2&& range2, Rs&&... ranges_) {
  // ("Dangling" is a false positive, due to the fact that there are >=2 reference-binding parameters.)
  if constexpr (sizeof...(Rs) == 0)
    return details::ConcatenatedRange<R1, R2>{std::forward<R1>(range1), std::forward<R2>(range2)};
  else
    return concatenate(std::forward<R1>(range1), concatenate(std::forward<R2>(range2), std::forward<Rs>(ranges_)...));
}

namespace details {

template <typename Iterator, typename Index> struct EnumeratedIterator {
  using type = EnumeratedIterator<Iterator, Index>;
  using value_type = typename std::iterator_traits<Iterator>::value_type;
  using difference_type = typename std::iterator_traits<Iterator>::difference_type;
  Iterator _iter;
  Index _index;
  bool operator==(const type& rhs) const { return _iter == rhs._iter; }
  // Note: "std::make_tuple(_index, *_iter)" would return a copied element.
  // Here we allow "for (auto&& [i, e] : enumerate(array)) e = int(i);".
  decltype(auto) operator*() const { return std::tuple<Index, decltype(*_iter)>(_index, *_iter); }
  type& operator++() { return ++_iter, ++_index, *this; }
  type operator++(int) { return postfix_increment(*this); }
};

template <ranges::range R, typename Index> struct EnumeratedRange {
  R _range;
  Index _start;
  auto begin() const {
    using Iterator = std::decay_t<decltype(ranges::begin(_range))>;
    return EnumeratedIterator<Iterator, Index>{ranges::begin(_range), _start};
  }
  auto end() const {
    using Iterator = std::decay_t<decltype(ranges::begin(_range))>;
    return EnumeratedIterator<Iterator, Index>{ranges::end(_range), _start};
  }
  auto size() const { return ranges::size(_range); }
};

}  // namespace details

// Return a view range containing a sequence of tuples [index, element] for all elements in `range`.
template <std::integral Index = size_t, ranges::range R> auto enumerate(R&& range, Index start = 0) {
  return details::EnumeratedRange<R, Index>{std::forward<R>(range), start};
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

template <ranges::range R, typename Func> struct FilteredRange {
  R _range;
  Func _func;
  // Type of the expression `_range` in a const member function (const does not apply if R is a reference).
  using CRange = std::add_lvalue_reference_t<std::add_const_t<R>>;
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
template <ranges::range R, typename Func> auto filter(R&& range, Func func = Func{}) {
  return details::FilteredRange<R, Func>{std::forward<R>(range), std::move(func)};
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RANGEOP_H_
