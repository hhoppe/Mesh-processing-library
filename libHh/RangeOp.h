// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_RANGEOP_H_
#define MESH_PROCESSING_LIBHH_RANGEOP_H_

#include "libHh/Hh.h"

namespace hh {

// *** Types:

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

// *** Const range operations:

// Check if a container contains an element.
template <ranges::input_range R> bool contains(const R& range, const range_value_t<R>& elem) {
  return ranges::find(range, elem) != ranges::end(range);
}

// (Optional) index of the first matching element, or std::nullopt if not found.
template <ranges::input_range R> std::optional<int> find_index(R&& range, const range_value_t<R>& elem) {
  if constexpr (ranges::random_access_range<R>) {
    const auto iter = ranges::find(range, elem);  // May lower to memchr or a vectorized search.
    if (iter != ranges::end(range)) return narrow_cast<int>(iter - ranges::begin(range));
    return {};
  } else {
    ranges::range_difference_t<R> i = 0;  // Single-pass-safe: count while traversing.
    for (auto&& e : range) {
      if (e == elem) return narrow_cast<int>(i);
      ++i;
    }
    return {};
  }
}

// Index of the first matching element, or die.
template <ranges::input_range R> int index(R&& range, const range_value_t<R>& elem) {
  const std::optional<int> i = find_index(range, elem);
  if (!i) assertnever(make_string(elem) + " not found in range");
  return *i;
}

// Address of the first element satisfying `pred`, or nullptr if none.
template <ranges::forward_range R, std::indirect_unary_predicate<ranges::iterator_t<R>> Pred>
requires std::is_lvalue_reference_v<ranges::range_reference_t<R>>
[[nodiscard]] auto find_if_ptr(R&& range, Pred pred) -> std::remove_reference_t<ranges::range_reference_t<R>>* {
  const auto iter = ranges::find_if(range, pred);
  return iter == ranges::end(range) ? nullptr : std::addressof(*iter);
}

// Minimum value in a non-empty range (by default using std::less(a, b)).
template <ranges::forward_range R, typename Comp = std::less<>>
requires std::indirect_strict_weak_order<Comp, ranges::iterator_t<const R&>>
auto min(const R& range, Comp comp = Comp{}) -> range_value_t<const R&> {
  auto iter = ranges::min_element(range, comp);
  ASSERTXX(iter != ranges::end(range));
  return *iter;
}

// Maximum value in a non-empty range (using std::less(a, b)).
template <ranges::forward_range R, typename Comp = std::less<>>
requires std::indirect_strict_weak_order<Comp, ranges::iterator_t<const R&>>
auto max(const R& range, Comp comp = Comp{}) -> range_value_t<R> {
  auto iter = ranges::max_element(range, comp);
  ASSERTXX(iter != ranges::end(range));
  return *iter;
}

// Maximum absolute value in a non-empty range.
template <ranges::input_range R> range_value_t<R> max_abs_element(const R& range) {
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
  ASSERTX(iter != itend);
  auto v = static_cast<range_value_t<R>>(abs(*iter));
  for (++iter; iter != itend; ++iter) v = max(v, static_cast<range_value_t<R>>(abs(*iter)));
  return v;
}

// Index of the minimum value in a non-empty range.
template <ranges::input_range R> int arg_min(const R& range) {
  auto iter = ranges::min_element(range);
  ASSERTX(iter != ranges::end(range));
  return narrow_cast<int>(std::distance(ranges::begin(range), iter));
}

// Index of the maximum value in a non-empty range.
template <ranges::input_range R> int arg_max(const R& range) {
  auto iter = ranges::max_element(range);
  ASSERTX(iter != ranges::end(range));
  return narrow_cast<int>(std::distance(ranges::begin(range), iter));
}

// *** Mutable range operations:

// Assign `value` to every element; return the range for chaining.
template <typename T, ranges::output_range<const T&> R> R fill(R&& range, const T& value) {
  // In the future, I could replace the return type from "R" to "R&&", to remove one move operation, and
  // similarly for all functions that apply 'std::forward<' on a '&&' parameter.
  // However, for a few uses in GraphOp_test and Multigrid_test, this requires P2718 (range-based for loop
  // lifetime extension) which Cygwin's gcc 14 (< 15) still lacks.
  ranges::fill(range, value);
  return std::forward<R>(range);
}

// Reverse the elements in-place in a randomly accessible range; return the range for chaining.
template <ranges::random_access_range R> R reverse(R&& range) {
  ranges::reverse(range);
  return std::forward<R>(range);
}

// Rotate the elements such that element middle becomes the new first element; return the range for chaining.
template <ranges::random_access_range R> R rotate(R&& range, ranges::iterator_t<R> middle) {
  ranges::rotate(range, middle);
  return std::forward<R>(range);
}

// Sort the elements in a range (by default using std::less(a, b)); return the range for chaining.
template <ranges::random_access_range R, typename Comp = std::less<>>
requires std::sortable<ranges::iterator_t<R>, Comp> R sort(R&& range, Comp comp = Comp{}) {
  ranges::sort(range, comp);
  return std::forward<R>(range);
}

// Return a sorted copy of the range (by default using std::less(a, b)), leaving the original unmodified.
template <ranges::random_access_range R, typename Comp = std::less<>>
requires std::sortable<ranges::iterator_t<std::decay_t<R>>, Comp>
[[nodiscard]] auto sorted(R&& range, Comp comp = Comp{}) -> std::decay_t<R> {
  auto result = clone(std::forward<R>(range));
  sort(result, comp);
  return result;  // Keep all 3 lines for NRVO.
}

// Swap the elements of two ranges, which must have the same number of elements.
template <ranges::input_range R1, ranges::input_range R2>
requires std::indirectly_swappable<ranges::iterator_t<R1>, ranges::iterator_t<R2>>
void swap_elements(R1&& range1, R2&& range2) {
  static_assert(std::same_as<range_value_t<R1>, range_value_t<R2>>, "ranges have different value types");
  if constexpr (ranges::sized_range<R1> && ranges::sized_range<R2>) {
    ASSERTX(ranges::size(range1) == ranges::size(range2));  // Detected before any swapping occurs.
    ranges::swap_ranges(range1, range2);
  } else {
    const auto result = ranges::swap_ranges(range1, range2);
    ASSERTX(result.in1 == ranges::end(range1) && result.in2 == ranges::end(range2));
  }
}

// *** Arithmetic range operations:

// Sum of values in a range (widening the accumulator, unless DesiredType is specified).
template <typename DesiredType = void, ranges::input_range R> sum_type_for_t<DesiredType, R> sum(const R& range) {
  using SumType = sum_type_for_t<DesiredType, R>;
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  // This could offer a 3-4x improvement on large ranges (currently, libstdc++ only), but it may reassociate
  // so it gives different results on floating_point:
  // if constexpr (ranges::common_range<R>)
  //   return implicit_cast<SumType>(std::reduce(ranges::begin(range), ranges::end(range), SumType{}));
  return implicit_cast<SumType>(ranges::fold_left(range, SumType{}, std::plus<>{}));
}

// Average of values in a range.
template <typename DesiredType = void, ranges::input_range R>
auto mean(const R& range) -> mean_type_for_t<DesiredType, R> {
  using MeanType = mean_type_for_t<DesiredType, R>;
  using FactorType = factor_type_t<range_value_t<R>>;
  static_assert(std::is_trivially_default_constructible_v<MeanType>);
  if constexpr (ranges::sized_range<R>) {
    const auto num = ranges::size(range);
    const auto sum_ = sum(range);
    if (num) return MeanType(sum_ * (FactorType(1) / num));
  } else {
    auto iter = ranges::begin(range);
    const auto itend = ranges::end(range);
    if (iter != itend) {
      MeanType sum_ = *iter;
      size_t num = 1;
      for (++iter; iter != itend; ++iter) {
        sum_ += *iter;
        num++;
      }
      return MeanType(sum_ * (FactorType(1) / num));
    }
  }
  Warning("mean() of empty range");
  return MeanType{};
}

// Sum of squared values in a range (or zero if empty).
template <typename DesiredType = void, ranges::input_range R>
auto mag2(const R& range) -> sum_type_for_t<DesiredType, R> {
  using SumType = sum_type_for_t<DesiredType, R>;
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  // const auto op = [](const SumType& sum, const auto& e) { return sum + square(SumType(e)); };
  // return implicit_cast<SumType>(ranges::fold_left(range, SumType{}, op));
  // Or also:
  // const auto squared = views::transform(range, [](const auto& e) { return square(SumType(e)); });
  // return implicit_cast<SumType>(ranges::fold_left(squared, SumType{}, std::plus<>{}));
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
  if (iter == itend) return SumType{};
  SumType v = square(SumType(*iter));
  for (++iter; iter != itend; ++iter) v += square(SumType(*iter));
  return v;
}

// Root sum of squared values in a range.
template <typename DesiredType = void, ranges::input_range R>
auto mag(const R& range) -> mean_type_for_t<DesiredType, R> {
  using MeanType = mean_type_for_t<DesiredType, R>;
  return sqrt(mag2<MeanType>(range));
}

// Root mean square of values in a range.
template <typename DesiredType = void, ranges::input_range R>
auto rms(const R& range) -> mean_type_for_t<DesiredType, R> {
  using MeanType = mean_type_for_t<DesiredType, R>;
  using FactorType = factor_type_t<range_value_t<R>>;
  static_assert(std::is_trivially_default_constructible_v<MeanType>);
  MeanType v{};
  size_t num = 0;
  for (const auto& e : range) {
    v += square(MeanType(e));
    num++;
  }
  if (num) {
    v = sqrt(MeanType(v * (FactorType(1) / num)));
  } else {
    Warning("rms() of empty range");
  }
  return v;
}

// Variance of values in a range.
template <typename DesiredType = void, ranges::input_range R>
auto var(const R& range) -> mean_type_for_t<DesiredType, R> {
  using MeanType = mean_type_for_t<DesiredType, R>;
  using FactorType = factor_type_t<range_value_t<R>>;
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
  return max(MeanType((v2 - v * v * (FactorType(1) / num)) * (FactorType(1) / (num - FactorType(1)))), zero);
}

// Product of values in a non-empty range.
template <typename DesiredType = void, ranges::input_range R>
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
template <ranges::input_range R> bool is_zero(const R& range) {
  for (const auto& e : range)
    if (e) return false;
  return true;
}

// Does it have unit norm?
template <typename DesiredType = void, ranges::input_range R>
bool is_unit(const R& range, range_value_t<R> tolerance = 1e-4f) {
  using SumType = sum_type_for_t<DesiredType, R>;
  return abs(mag2<SumType>(range) - 1.f) <= tolerance;
}

// Modify the range to have unit norm (or die if input has zero norm); return the range for chaining.
template <ranges::forward_range R> R normalize(R&& range) {
  using Value = range_value_t<R>;
  Value v = static_cast<Value>(Value{1.f} / assertx(mag(range)));
  for (auto& e : range) e = static_cast<Value>(e * v);
  return std::forward<R>(range);
}

// Round the values in a range to the nearest 1/fac increment (by default fac == 1e5f); return the range for chaining.
template <ranges::forward_range R> R round_elements(R&& range, range_value_t<R> fac = 1e5f) {
  static_assert(std::is_floating_point_v<range_value_t<R>>);
  for (auto& e : range) e = round_fraction_digits(e, fac);
  return std::forward<R>(range);
}

// Compute the sum of squared differences of corresponding elements of two ranges.
template <typename DesiredType = void, indexable_range R1, indexable_range R2>
auto dist2(const R1& range1, const R2& range2) -> sum_type_for_t<DesiredType, R1> {
  using SumType = sum_type_for_t<DesiredType, R1>;
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  const auto n = ranges::ssize(range1);
  ASSERTX(n == ranges::ssize(range2));
  const auto iter1 = ranges::begin(range1);  // A single induction variable enables unrolling.
  const auto iter2 = ranges::begin(range2);
  SumType v{};
  for (const auto i : range(n)) v += square(SumType(iter1[i]) - SumType(iter2[i]));
  return v;
}

// Compute the Euclidean distance between two ranges interpreted as vectors.
template <typename DesiredType = void, ranges::input_range R1, ranges::input_range R2>
auto dist(const R1& range1, const R2& range2) -> mean_type_for_t<DesiredType, R1> {
  using MeanType = mean_type_for_t<DesiredType, R1>;
  return sqrt(dist2<MeanType>(range1, range2));
}

// Compute the inner product of two ranges, which must have the same number of elements.
template <typename DesiredType = void, indexable_range R1, indexable_range R2>
auto dot(const R1& range1, const R2& range2) -> sum_type_for_t<DesiredType, R1> {
  using SumType = sum_type_for_t<DesiredType, R1>;
  static_assert(std::is_trivially_default_constructible_v<SumType>);
  const auto n = ranges::ssize(range1);
  ASSERTX(n == ranges::ssize(range2));
  const auto iter1 = ranges::begin(range1);
  const auto iter2 = ranges::begin(range2);
  SumType v{};
  for (const auto i : range(n)) v += SumType(iter1[i]) * SumType(iter2[i]);
  return v;
}

// Compare two ranges of algebraic types lexicographically; ret -1, 0, 1 based on sign of range1 - range2.
// Use <=> ??
template <indexable_range R1, indexable_range R2> int compare(const R1& range1, const R2& range2) {
  const auto n = ranges::ssize(range1);
  ASSERTX(n == ranges::ssize(range2));
  const auto iter1 = ranges::begin(range1);
  const auto iter2 = ranges::begin(range2);
  for (const auto i : range(n)) {
    if (auto d = iter1[i] - iter2[i]) return d < 0 ? -1 : +1;
  }
  return 0;
}

// Similar comparison, but ignore differences smaller than tolerance.
template <indexable_range R1, indexable_range R2>
int compare(const R1& range1, const R2& range2, const range_value_t<R1>& tolerance) {
  const auto n = ranges::ssize(range1);
  ASSERTX(n == ranges::ssize(range2));
  const auto iter1 = ranges::begin(range1);
  const auto iter2 = ranges::begin(range2);
  for (const auto i : range(n)) {
    const auto d = iter1[i] - iter2[i];
    if (d < -tolerance) return -1;
    if (d > tolerance) return +1;
  }
  return 0;
}

// For any container R (e.g. Vec, Array, PArray, Grid, SGrid) supporting transformed(R&, [](const T&) -> T):

// Convert all elements of the container to the new type U, e.g. convert<float>(V(1, 2)) == V(1.f, 2.f).
// Be careful to possibly use floor() before convert<int>() to avoid rounding negative values towards zero.
template <typename U, ranges::forward_range R> auto convert(const R& c) {
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

template <ranges::forward_range R, typename Func> struct TransformedRange {
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
template <ranges::forward_range R, typename Func> auto transform(R&& range, Func func = Func{}) {
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

template <ranges::forward_range R1, ranges::forward_range R2> struct ConcatenatedRange {
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
template <ranges::forward_range R1, ranges::forward_range R2, typename... Rs>
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

template <ranges::forward_range R, typename Index> struct EnumeratedRange {
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
template <std::integral Index = size_t, ranges::forward_range R> auto enumerate(R&& range, Index start = 0) {
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

template <ranges::forward_range R, typename Func> struct FilteredRange {
  R _range;
  Func _func;
  // Type of the expression `_range` in a const member function (const does not apply if R is a reference).
  using CRange = std::add_lvalue_reference_t<std::add_const_t<R>>;
  using Iterator = std::decay_t<ranges::iterator_t<CRange>>;
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
template <ranges::forward_range R, typename Func> auto filter(R&& range, Func func = Func{}) {
  return details::FilteredRange<R, Func>{std::forward<R>(range), std::move(func)};
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RANGEOP_H_
