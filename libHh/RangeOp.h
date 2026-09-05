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

// Most range operations in std take a "R&& range" parameter.  This lets filter_view and (some) transform_view,
// which are not const-iterable, be passed as either lvalues or rvalues and still be traversed non-const.
// Unfortunately, declaring the functions here as "R&& range" would give them precedence over any external
// overload of the same name declared for a particular class as "const T&": for an rvalue argument the generic
// wins on the rvalue-reference binding, and for a non-const lvalue argument it wins because R deduces to T&,
// which is less cv-qualified than const T&.  Only a const lvalue argument still selects the external overload.
// For example, (sum, mag2, mag, is_unit, dist2, dist, dot) have lower-precision or SIMD overloads for specific
// classes (Vec.h, Geometry.h, Vector4.h, VectorF.h), and other functions (contains, min, max, is_zero) may
// acquire such overloads in the future.
//
// Our workaround is to declare the range operations as "const R& range" (to preserve those overloads) and to
// obtain a traversable range within the RangeOp function using the helper `iterable`, which copies the view when
// it is not const-iterable.
// The drawback is that each call traverses a temporary (throwaway) copy, so the view's cached begin() is never
// established and every call must again skip the leading elements.  For example:
//
//   auto r = range(1000) | views::filter([](int i) { return i > 900; });
//   const bool b1 = contains(r, 920);  // 921 predicate evaluations.
//   const auto n = sum(r);             // 1000 rather than 98, because begin() is recomputed.
//
// The remedy is to bind the view to a named lvalue and hoist it into a subrange, which is const-iterable and so
// is passed through without copying.  The call to subrange() materializes a plain pair of iterators (no cache).
//
//   auto v = range(1000) | views::filter([](int i) { return i > 900; });
//   auto r = ranges::subrange(v);      // v must outlive r; this resolves begin() once, at a cost of 902 evaluations.
//   const bool b1 = contains(r, 920);  // 19 predicate evaluations.
//   const auto n = sum(r);             // 98.

// Return a range that is iterable through const.  Some view adaptors (e.g. filter_view, which caches begin())
// are not const-iterable; a view is cheap to copy, and the copy is iterable.  Usage: `auto&& r = iterable(range);`.
template <ranges::input_range R> [[nodiscard]] decltype(auto) iterable(const R& range) {
  // Can we call begin()/end() on the object held by const reference?
  if constexpr (ranges::range<const R&>)
    // For containers and most views, the answer is yes.
    return range;  // Return a const reference -- a no-op.
  else
    // For filter_view, drop_while_view, and some transform_view, begin() cannot be const.
    return R(range);  // We produce a prvalue copy of the view.  Copying a view is O(1) so it is affordable.
}

// *** Range operations that do not modify the elements:

// Check if a container contains an element.
template <ranges::input_range R> [[nodiscard]] bool contains(const R& range_, const range_value_t<R>& elem) {
  auto&& range = iterable(range_);
  return ranges::find(range, elem) != ranges::end(range);
}

// (Optional) index of the first matching element, or std::nullopt if not found.
template <ranges::input_range R> [[nodiscard]] std::optional<int> find_index(R&& range, const range_value_t<R>& elem) {
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
template <ranges::input_range R> [[nodiscard]] int index(R&& range, const range_value_t<R>& elem) {
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
requires std::indirect_strict_weak_order<Comp, ranges::iterator_t<R>>
[[nodiscard]] auto min(const R& range_, Comp comp = Comp{}) -> range_value_t<R> {
  auto&& range = iterable(range_);
  auto iter = ranges::min_element(range, comp);
  ASSERTXX(iter != ranges::end(range));
  return *iter;
}

// Maximum value in a non-empty range (using std::less(a, b)).
template <ranges::forward_range R, typename Comp = std::less<>>
requires std::indirect_strict_weak_order<Comp, ranges::iterator_t<R>>
[[nodiscard]] auto max(const R& range_, Comp comp = Comp{}) -> range_value_t<R> {
  auto&& range = iterable(range_);
  auto iter = ranges::max_element(range, comp);
  ASSERTXX(iter != ranges::end(range));
  return *iter;
}

// Maximum absolute value in a non-empty range.
template <ranges::input_range R> [[nodiscard]] range_value_t<R> max_abs_element(const R& range_) {
  auto&& range = iterable(range_);
  auto iter = ranges::begin(range);
  const auto itend = ranges::end(range);
  ASSERTX(iter != itend);
  auto v = static_cast<range_value_t<R>>(abs(*iter));
  for (++iter; iter != itend; ++iter) v = max(v, static_cast<range_value_t<R>>(abs(*iter)));
  return v;
}

// Index of the minimum value in a non-empty range.
template <ranges::forward_range R, typename Comp = std::less<>>
requires std::indirect_strict_weak_order<Comp, ranges::iterator_t<R>>
[[nodiscard]] int arg_min(const R& range_, Comp comp = Comp{}) {
  auto&& range = iterable(range_);
  auto iter = ranges::min_element(range, comp);
  ASSERTXX(iter != ranges::end(range));
  return narrow_cast<int>(ranges::distance(ranges::begin(range), iter));
}

// Index of the maximum value in a non-empty range.
template <ranges::forward_range R, typename Comp = std::less<>>
requires std::indirect_strict_weak_order<Comp, ranges::iterator_t<R>>
[[nodiscard]] int arg_max(const R& range_, Comp comp = Comp{}) {
  auto&& range = iterable(range_);
  auto iter = ranges::max_element(range, comp);
  ASSERTXX(iter != ranges::end(range));
  return narrow_cast<int>(ranges::distance(ranges::begin(range), iter));
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
template <typename DesiredType = void, ranges::input_range R> sum_type_for_t<DesiredType, R> sum(const R& range_) {
  auto&& range = iterable(range_);
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
[[nodiscard]] auto mean(const R& range_) -> mean_type_for_t<DesiredType, R> {
  auto&& range = iterable(range_);
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
[[nodiscard]] auto mag2(const R& range_) -> sum_type_for_t<DesiredType, R> {
  auto&& range = iterable(range_);
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
[[nodiscard]] auto mag(const R& range) -> mean_type_for_t<DesiredType, R> {
  using MeanType = mean_type_for_t<DesiredType, R>;
  return sqrt(mag2<MeanType>(range));
}

// Root mean square of values in a range.
template <typename DesiredType = void, ranges::input_range R>
[[nodiscard]] auto rms(const R& range_) -> mean_type_for_t<DesiredType, R> {
  auto&& range = iterable(range_);
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
[[nodiscard]] auto var(const R& range_) -> mean_type_for_t<DesiredType, R> {
  auto&& range = iterable(range_);
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
[[nodiscard]] auto product(const R& range_) -> sum_type_for_t<DesiredType, R> {
  auto&& range = iterable(range_);
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
template <ranges::input_range R> [[nodiscard]] bool is_zero(const R& range_) {
  auto&& range = iterable(range_);
  for (const auto& e : range)
    if (e) return false;
  return true;
}

// Does it have unit norm?
template <typename DesiredType = void, ranges::input_range R>
[[nodiscard]] bool is_unit(const R& range, range_value_t<R> tolerance = 1e-4f) {
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
[[nodiscard]] auto dist2(const R1& range1_, const R2& range2_) -> sum_type_for_t<DesiredType, R1> {
  auto&& range1 = iterable(range1_);
  auto&& range2 = iterable(range2_);
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
template <typename DesiredType = void, indexable_range R1, indexable_range R2>
[[nodiscard]] auto dist(const R1& range1, const R2& range2) -> mean_type_for_t<DesiredType, R1> {
  using MeanType = mean_type_for_t<DesiredType, R1>;
  return sqrt(dist2<MeanType>(range1, range2));
}

// Compute the inner product of two ranges, which must have the same number of elements.
template <typename DesiredType = void, indexable_range R1, indexable_range R2>
[[nodiscard]] auto dot(const R1& range1_, const R2& range2_) -> sum_type_for_t<DesiredType, R1> {
  auto&& range1 = iterable(range1_);
  auto&& range2 = iterable(range2_);
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

// Compare two ranges lexicographically; the result compares against 0 like a <=> b would.
template <indexable_range R1, indexable_range R2>
[[nodiscard]] std::compare_three_way_result_t<range_value_t<R1>, range_value_t<R2>> compare(const R1& range1_,
                                                                                            const R2& range2_) {
  auto&& range1 = iterable(range1_);
  auto&& range2 = iterable(range2_);
  const auto n = ranges::ssize(range1);
  ASSERTX(n == ranges::ssize(range2));
  const auto iter1 = ranges::begin(range1);
  const auto iter2 = ranges::begin(range2);
  for (const auto i : range(n)) {
    if (auto c = iter1[i] <=> iter2[i]; c != 0) return c;
  }
  return std::strong_ordering::equal;
}

// Similar comparison, but ignore differences smaller than tolerance.  Equivalence here is not transitive, so
// the result must not be used as an ordering for sorting or for associative containers.
template <indexable_range R1, indexable_range R2>
[[nodiscard]] std::weak_ordering compare(const R1& range1_, const R2& range2_, const range_value_t<R1>& tolerance) {
  auto&& range1 = iterable(range1_);
  auto&& range2 = iterable(range2_);
  const auto n = ranges::ssize(range1);
  ASSERTX(n == ranges::ssize(range2));
  const auto iter1 = ranges::begin(range1);
  const auto iter2 = ranges::begin(range2);
  for (const auto i : range(n)) {
    const auto d = iter1[i] - iter2[i];
    if (d < -tolerance) return std::weak_ordering::less;
    if (d > tolerance) return std::weak_ordering::greater;
  }
  return std::weak_ordering::equivalent;
}

// For any container R (e.g. Vec, Array, PArray, Grid) supporting transformed(R&, [](const T&) -> T):

// Convert all elements of the container to the new type U, e.g. convert<float>(V(1, 2)) == V(1.f, 2.f).
// Be careful to possibly use floor() before convert<int>() to avoid rounding negative values towards zero.
template <typename U, ranges::forward_range R> [[nodiscard]] auto convert(const R& c) {
  return transformed(c, [](const range_value_t<R>& e) { return static_cast<U>(e); });
}

namespace details {

template <typename Sentinel2> struct ConcatenatedSentinel {
  Sentinel2 _end2;
};

template <typename Iterator1, typename Sentinel1, typename Iterator2> struct ConcatenatedIterator {
  using type = ConcatenatedIterator<Iterator1, Sentinel1, Iterator2>;
  using iterator_concept = std::forward_iterator_tag;
  using value_type = std::iter_value_t<Iterator1>;
  using difference_type = std::iter_difference_t<Iterator1>;
  static_assert(std::is_same_v<value_type, std::iter_value_t<Iterator2>>);
  Iterator1 _it1;
  Sentinel1 _end1;
  Iterator2 _it2;
  [[nodiscard]] bool operator==(const type& rhs) const { return _it1 == rhs._it1 && _it2 == rhs._it2; }
  template <typename S2> [[nodiscard]] bool operator==(const ConcatenatedSentinel<S2>& rhs) const {
    return _it1 == _end1 && _it2 == rhs._end2;
  }
  [[nodiscard]] decltype(auto) operator*() const { return _it1 != _end1 ? *_it1 : *_it2; }
  type& operator++() {
    if (_it1 != _end1)
      ++_it1;
    else
      ++_it2;
    return *this;
  }
  type operator++(int) { return postfix_increment(*this); }
};

template <ranges::forward_range R1, ranges::forward_range R2> struct ConcatenatedRange {
  R1 _range1;
  R2 _range2;
  [[nodiscard]] auto begin() const {
    return ConcatenatedIterator{ranges::begin(_range1), ranges::end(_range1), ranges::begin(_range2)};
  }
  [[nodiscard]] auto end() const {
    // Name the type explicitly; CTAD would select the copy-deduction candidate if _range2 is itself concatenated.
    using Sentinel2 = decltype(ranges::end(_range2));
    return ConcatenatedSentinel<Sentinel2>{ranges::end(_range2)};
  }
  [[nodiscard]] auto size() const requires ranges::sized_range<const R1> && ranges::sized_range<const R2> {
    return ranges::size(_range1) + ranges::size(_range2);
  }
};

}  // namespace details

// Return a view range that concatenates the elements of two or more ranges.
// C++26: replace by views::concat().
template <ranges::forward_range R1, ranges::forward_range R2, typename... Rs>
[[HH_NO_DANGLING]] [[nodiscard]] auto concatenate(R1&& range1, R2&& range2, Rs&&... ranges_) {
  // ("Dangling" is a false positive, due to the fact that there are >=2 reference-binding parameters.)
  if constexpr (sizeof...(Rs) == 0)
    return details::ConcatenatedRange<R1, R2>{std::forward<R1>(range1), std::forward<R2>(range2)};
  else
    return concatenate(std::forward<R1>(range1), concatenate(std::forward<R2>(range2), std::forward<Rs>(ranges_)...));
}

namespace details {

// Like views::take(), but its iterator holds the remaining count, so operator++ on the last element does not
// advance the underlying iterator.  (views::take() uses counted_iterator, whose operator++ always advances and
// only then observes that the count reached zero.)
template <ranges::view V> class TruncatedRange : public ranges::view_interface<TruncatedRange<V>> {
  using Difference = ranges::range_difference_t<V>;

 public:
  TruncatedRange() = default;
  constexpr TruncatedRange(V base, Difference count) : _base(std::move(base)), _count(count) {}

  class Iterator {
   public:
    using iterator_concept = std::input_iterator_tag;
    using value_type = ranges::range_value_t<V>;
    using difference_type = Difference;
    Iterator() = default;
    constexpr Iterator(ranges::iterator_t<V> iter, ranges::sentinel_t<V> end, Difference count)
        : _iter(std::move(iter)), _end(std::move(end)), _count(count) {}
    [[nodiscard]] constexpr decltype(auto) operator*() const { return *_iter; }
    [[nodiscard]] constexpr bool operator==(std::default_sentinel_t) const { return !_count || _iter == _end; }
    constexpr Iterator& operator++() {
      if (--_count) ++_iter;  // Advance only if a further element can still be requested.
      return *this;
    }
    constexpr void operator++(int) { ++*this; }  // Single-pass, so the prior value cannot be returned.

   private:
    ranges::iterator_t<V> _iter{};
    ranges::sentinel_t<V> _end{};
    Difference _count{};
  };

  [[nodiscard]] constexpr Iterator begin() { return Iterator(ranges::begin(_base), ranges::end(_base), _count); }
  [[nodiscard]] constexpr std::default_sentinel_t end() const { return {}; }

 private:
  V _base{};
  Difference _count{};
};

template <typename V> TruncatedRange(V&&, std::ptrdiff_t) -> TruncatedRange<views::all_t<V>>;

struct TruncateClosure : ranges::range_adaptor_closure<TruncateClosure> {
  std::ptrdiff_t _count;
  template <ranges::viewable_range R> [[nodiscard]] constexpr auto operator()(R&& range) const {
    return TruncatedRange(views::all(std::forward<R>(range)), _count);
  }
};

}  // namespace details

// Return a view of the first count elements of range, leaving the range advanced to just past those elements.
// Unlike views::take(), the underlying iterator is never incremented past the last element that is yielded, so a
// single-pass source (e.g. SpatialSearch) remains positioned on its next unconsumed element and does no extra work.
[[nodiscard]] inline constexpr auto truncate(std::ptrdiff_t count) { return details::TruncateClosure{{}, count}; }

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RANGEOP_H_
