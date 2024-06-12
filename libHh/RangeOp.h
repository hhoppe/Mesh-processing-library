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
  using std::begin, std::end;
  return std::all_of(begin(range), end(range), p);
}

// Check unary predicate against elements; false if range is empty.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>> bool any_of(const Range& range, Pred p) {
  using std::begin, std::end;
  return std::any_of(begin(range), end(range), p);
}

// Check unary predicate against elements; true if range is empty.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>>
bool none_of(const Range& range, Pred p) {
  using std::begin, std::end;
  return std::none_of(begin(range), end(range), p);
}

// Apply unary functor object to each element and returns functor.
template <typename Range, typename Func, typename = enable_if_range_t<Range>> Func for_each(Range&& range, Func&& f) {
  using std::begin, std::end;
  return std::for_each(begin(range), end(range), std::forward<Func>(f));
}

// Return the address of the first element satisfying condition, or nullptr if none.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
Iterator* find_if(Range&& range, Pred p) {
  using std::begin, std::end;
  auto iter = std::find_if(begin(range), end(range), p);
  return iter == end(range) ? nullptr : &*iter;
}

// Return the address of the first element not satisfying condition, or nullptr if none.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
Iterator* find_if_not(Range&& range, Pred p) {
  using std::begin, std::end;
  auto iter = std::find_if_not(begin(range), end(range), p);
  return iter == end(range) ? nullptr : &*iter;
}

// Count the number of elements equal to specified one.
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
std::ptrdiff_t count(const Range& range, const Iterator& elem) {
  using std::begin, std::end;
  return std::count(begin(range), end(range), elem);
}

// Count the number of elements matching predicate.
template <typename Range, typename Pred, typename = enable_if_range_t<Range>>
std::ptrdiff_t count_if(const Range& range, Pred p) {
  using std::begin, std::end;
  return std::count_if(begin(range), end(range), p);
}

// Return whether two ranges are equal element-wise.
template <typename Range1, typename Range2, typename Pred = std::equal_to<iterator_t<Range1>>,
          typename = enable_if_range_t<Range1>, typename = enable_if_range_t<Range2>>
bool equal(const Range1& range1, const Range2& range2, Pred p = Pred{}) {
  using std::begin, std::end;
  return std::equal(begin(range1), end(range1), begin(range2), end(range2), p);
}

// Swap the contents of two ranges.
template <typename Range1, typename Range2, typename = enable_if_range_t<Range1>, typename = enable_if_range_t<Range2>>
void swap_ranges(Range1&& range1, Range2&& range2) {
  using std::begin, std::end;
  auto iter1 = begin(range1), itend1 = end(range1);
  auto iter2 = begin(range2), itend2 = end(range2);
  if (0) {  // Draft N4560 swaps the first min(size(range1), size(range2)) elements.
    if (itend2 - iter2 < itend1 - iter1) {
      using std::swap;
      swap(iter1, iter2);
      swap(itend1, itend2);
    }
    std::swap_ranges(iter1, itend1, iter2);
  } else {  // Instead we require that they have the same size.
    auto it = std::swap_ranges(iter1, itend1, iter2);
    ASSERTX(it == itend2);  // verify they have the same number of elements
  }
}

// Assign the same value to all elements in a range.
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
Range fill(Range&& range, const Iterator& v) {
  // using std::begin, std::end; std::fill(begin(range), end(range), v);
  for (auto& e : range) e = v;
  return std::forward<Range>(range);
}

// Reverse the elements in a randomly accessible range.
template <typename Range, typename = enable_if_range_t<Range>> Range reverse(Range&& range_) {
  using std::begin, std::end;
  auto b = begin(range_), e = end(range_);
  ASSERTX(e >= b);  // requires random-access iterator
  // std::reverse(b, e);
  size_t num = e - b;
  for_size_t(i, num / 2) {
    using std::swap;
    swap(b[i], b[num - 1 - i]);
  }
  return std::forward<Range>(range_);
}
// Range reversed(const Range& range) { return reverse(clone(range)); }

// Rotate the elements in a randomly accessible range such that element middle becomes the new first element.
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
Range rotate(Range&& range, Iterator& middle) {
  using std::begin, std::end;
  std::rotate(begin(range), &middle, end(range));
  return std::forward<Range>(range);
}

// Sort the elements in a range (by default using less(a, b)).
template <typename Range, typename Comp = std::less<iterator_t<Range>>, typename = enable_if_range_t<Range>>
Range sort(Range&& range, Comp comp = Comp{}) {
  using std::begin, std::end;
  std::sort(begin(range), end(range), comp);
  return std::forward<Range>(range);
}
// Range sorted(const Range& range) { return sort(clone(range)); }

// Minimum value in a non-empty range (by default using less(a, b)).
template <typename Range, typename Comp = std::less<iterator_t<Range>>, typename = enable_if_range_t<Range>,
          typename Iterator = iterator_t<Range>>
Iterator min(const Range& range, Comp comp = Comp{}) {
  using std::begin, std::end;
  ASSERTXX(begin(range) != end(range));
  return *std::min_element(begin(range), end(range), comp);
}

// Maximum value in a non-empty range (using less(a, b)).
template <typename Range, typename Comp = std::less<iterator_t<Range>>, typename = enable_if_range_t<Range>,
          typename Iterator = iterator_t<Range>>
Iterator max(const Range& range, Comp comp = Comp{}) {
  using std::begin, std::end;
  ASSERTXX(begin(range) != end(range));
  return *std::max_element(begin(range), end(range), comp);
}

// Number of elements in a range (could also define size(Range) but for robustness that would require Concepts).
template <typename Range, typename = enable_if_range_t<Range>> std::ptrdiff_t distance(const Range& range) {
  using std::begin, std::end;
  return std::distance(begin(range), end(range));
}

// Also from std: find(), find_end(), find_first_of(), adjacent_find(), mismatch(), equal()
//  is_permutation(), search(), search_n(), copy(), copy_if(), copy_backward(), move(), move_backward()
//  transform(), replace(), replace_if(), replace_copy(), replace_copy_if(),
//  generate(), remove(), remove_if(), remove_copy(), remove_copy_if(), unique(), unique_copy(),
//  reverse_copy(), rotate_copy(), shuffle()
//  is_partitioned(), partition(), stable_partition(), partition_copy(), partition_move()
//  stable_sort(), partial_sort(), partial_sort_copy(), is_sorted(), is_sorted_until(), nth_element(),
//  lower_bound(), upper_bound(), equal_range(), binary_search(),
//  merge(), merge_move(), inplace_merge(), includes(),
//  set_union(), set_intersection(), set_difference(), set_symmetric_difference(), *heap*()
//  minmax(), minmax_element(), lexicographical_compare(), next_permutation(), prev_permutation()

// *** My custom range operations:

// Return the index of the first matching element, or die if not found.
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
int index(const Range& range, const Iterator& elem) {
  using std::begin, std::end;
  auto iter = std::find(begin(range), end(range), elem);
  if (iter == end(range)) assertnever(make_string(elem) + " not found in range");
  return assert_narrow_cast<int>(iter - begin(range));
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
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
int arg_min(const Range& range) {
  using std::begin, std::end;
  ASSERTX(begin(range) != end(range));
  auto p = std::min_element(begin(range), end(range));
  return narrow_cast<int>(std::distance(begin(range), p));
}

// Index of maximum value in a non-empty range (using less(a, b)).
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
int arg_max(const Range& range) {
  using std::begin, std::end;
  ASSERTXX(begin(range) != end(range));
  auto p = std::max_element(begin(range), end(range));
  return narrow_cast<int>(std::distance(begin(range), p));
}

// Minimum over a non-empty range of values (using successive min(a, b) rather than less(a, b)).
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
Iterator transitive_min(const Range& range) {
  using std::begin, std::end;
  auto iter = begin(range), itend = end(range);
  ASSERTX(iter != itend);
  Iterator v = *iter;
  for (++iter; iter != itend; ++iter) v = min(v, *iter);
  return v;
}

// Maximum over a non-empty range of values (using successive min(a, b) rather than less(a, b)).
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
Iterator transitive_max(const Range& range) {
  using std::begin, std::end;
  auto iter = begin(range), itend = end(range);
  ASSERTX(iter != itend);
  Iterator v = *iter;
  for (++iter; iter != itend; ++iter) v = max(v, *iter);
  return v;
}

// Maximum absolute value in a non-empty range.
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
Iterator max_abs_element(const Range& range) {
  using std::begin, std::end;
  auto iter = begin(range), itend = end(range);
  ASSERTX(iter != itend);
  // return abs(*std::max_element(ibeg, iend, [](Iterator a, Iterator b) { return abs(a) < abs(b); }));
  Iterator v = static_cast<Iterator>(abs(*iter));
  for (++iter; iter != itend; ++iter) v = max(v, static_cast<Iterator>(abs(*iter)));
  return v;
}

// Sum of values in a range.
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Iterator = iterator_t<Range>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Iterator>, DesiredType>>
SumType sum(const Range& range) {
  using std::begin, std::end;
  // return std::accumulate(begin(range), end(range), SumType{});
  auto iter = begin(range), itend = end(range);
  if (iter == itend) {
    SumType v;
    my_zero(v);
    return v;
  }
  SumType v = *iter;
  for (++iter; iter != itend; ++iter) v += *iter;
  return v;
}

// Average of values in a range.
template <
    typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
    typename Iterator = iterator_t<Range>,
    typename MeanType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Iterator>, DesiredType>>
MeanType mean(const Range& range) {
  using std::begin, std::end;
  auto iter = begin(range), itend = end(range);
  if (iter == itend) {
    Warning("mean");
    MeanType v;
    my_zero(v);
    return v;
  }
  MeanType v = *iter;
  size_t num = 1;
  for (++iter; iter != itend; ++iter) {
    v += *iter;
    num++;
  }
  using Factor = factor_type_t<Iterator>;
  return MeanType(v * (Factor(1) / num));
}

// Sum of squared values in a range (or zero if empty).
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Iterator = iterator_t<Range>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Iterator>, DesiredType>>
SumType mag2(const Range& range) {
  using std::begin, std::end;
  // return std::accumulate(begin(range), end(range), SumType{},
  //                        [](const SumType& sum, const Iterator& e) { return sum + square(e); });
  auto iter = begin(range), itend = end(range);
  if (iter == itend) {
    SumType v;
    my_zero(v);
    return v;
  }
  SumType v = square(SumType(*iter));
  for (++iter; iter != itend; ++iter) v += square(SumType(*iter));
  return v;
}

// Root sum of squared values in a range.
template <
    typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
    typename Iterator = iterator_t<Range>,
    typename MeanType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Iterator>, DesiredType>>
MeanType mag(const Range& range) {
  return sqrt(mag2<MeanType>(range));
}

// Root mean square of values in a range.
template <
    typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
    typename Iterator = iterator_t<Range>,
    typename MeanType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Iterator>, DesiredType>>
MeanType rms(const Range& range) {
  MeanType v;
  my_zero(v);
  size_t num = 0;
  for (const auto& e : range) {
    v += square(MeanType(e));
    num++;
  }
  if (!num) {
    Warning("rms() of empty range");
    return v;
  }
  using Factor = factor_type_t<Iterator>;
  return sqrt(MeanType(v * (Factor(1) / num)));
}

// Variance of values in a range.
template <
    typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
    typename Iterator = iterator_t<Range>,
    typename MeanType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Iterator>, DesiredType>>
MeanType var(const Range& range) {
  MeanType zero, v, v2;
  my_zero(zero);
  my_zero(v);
  my_zero(v2);
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
  using Factor = factor_type_t<Iterator>;
  return max(MeanType((v2 - v * v * (Factor(1) / num)) * (Factor(1) / (num - Factor(1)))), zero);
}

// Product of values in a non-empty range.
template <typename DesiredType = void, typename Range, typename = enable_if_range_t<Range>,
          typename Iterator = iterator_t<Range>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Iterator>, DesiredType>>
SumType product(const Range& range) {
  using std::begin, std::end;
  auto iter = begin(range), itend = end(range);
  ASSERTX(iter != itend);
  // return std::accumulate(begin(range), end(range), SumType{1},
  //                        [](const SumType& sum, const Iterator& e) { return sum * e; });
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
          typename Iterator = iterator_t<Range>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Iterator>, DesiredType>>
bool is_unit(const Range& range, SumType tolerance = 1e-4f) {
  return abs(mag2<SumType>(range) - 1.f) <= tolerance;
}

// Modify the range to have unit norm (or die if input has zero norm).
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
Range normalize(Range&& range) {
  Iterator v = static_cast<Iterator>(Iterator{1.f} / assertx(mag(range)));
  for (auto& e : range) e = static_cast<Iterator>(e * v);
  return std::forward<Range>(range);
}
// Range normalized(const Range& range) { return normalize(clone(range)); }

// Round the values in a range to the nearest 1/fac increment (by default fac == 1e5f).
template <typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
Range round_elements(Range&& range, Iterator fac = 1e5f) {
  static_assert(std::is_floating_point_v<Iterator>);
  for (auto& e : range) e = round_fraction_digits(e, fac);
  return std::forward<Range>(range);
}

// Compute the sum of squared differences of corresponding elements of two ranges.
template <typename DesiredType = void, typename Range1, typename Range2, typename = enable_if_range_t<Range1>,
          typename = enable_if_range_t<Range2>, typename Iterator = iterator_t<Range1>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Iterator>, DesiredType>>
SumType dist2(const Range1& range1, const Range2& range2) {
  using std::begin, std::end;
  auto iter1 = begin(range1), itend1 = end(range1);
  auto iter2 = begin(range2), itend2 = end(range2);
  SumType v;
  my_zero(v);
  for (; iter1 != itend1; ++iter1, ++iter2) v += square(SumType(*iter1) - SumType(*iter2));
  ASSERTX(iter2 == itend2);  // verify they have the same number of elements
  return v;
}

// Compute the Euclidean distance between two ranges interpreted as vectors.
template <typename DesiredType = void, typename Range1, typename Range2, typename = enable_if_range_t<Range1>,
          typename = enable_if_range_t<Range2>, typename Iterator = iterator_t<Range1>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, mean_type_t<Iterator>, DesiredType>>
SumType dist(const Range1& range1, const Range2& range2) {
  return sqrt(dist2<SumType>(range1, range2));
}

// Compute the inner product of two ranges.
template <typename DesiredType = void, typename Range1, typename Range2, typename = enable_if_range_t<Range1>,
          typename = enable_if_range_t<Range2>, typename Iterator = iterator_t<Range1>,
          typename SumType = std::conditional_t<std::is_same_v<DesiredType, void>, sum_type_t<Iterator>, DesiredType>>
SumType dot(const Range1& range1, const Range2& range2) {
  using std::begin, std::end;
  auto iter1 = begin(range1), itend1 = end(range1);
  auto iter2 = begin(range2), itend2 = end(range2);
  SumType v;
  my_zero(v);
  for (; iter1 != itend1; ++iter1, ++iter2) v += SumType(*iter1) * SumType(*iter2);
  ASSERTX(iter2 == itend2);  // verify they have the same number of elements
  return v;
}

// Compare two ranges of algebraic types lexicographically; ret -1, 0, 1 based on sign of range1 - range2.
template <typename Range1, typename Range2, typename = enable_if_range_t<Range1>, typename = enable_if_range_t<Range2>>
int compare(const Range1& range1, const Range2& range2) {
  using std::begin, std::end;
  auto iter1 = begin(range1), itend1 = end(range1);
  auto iter2 = begin(range2), itend2 = end(range2);
  for (; iter1 != itend1; ++iter1, ++iter2) {
    auto d = *iter1 - *iter2;
    if (d) return d < 0 ? -1 : +1;
  }
  ASSERTX(iter2 == itend2);  // verify they have the same number of elements
  return 0;
}

// Similar comparison, but ignore differences smaller than tolerance.
template <typename Range1, typename Range2, typename = enable_if_range_t<Range1>, typename = enable_if_range_t<Range2>,
          typename Iterator = iterator_t<Range1>>
int compare(const Range1& range1, const Range2& range2, const Iterator& tolerance) {
  using std::begin, std::end;
  auto iter1 = begin(range1), itend1 = end(range1);
  auto iter2 = begin(range2), itend2 = end(range2);
  for (; iter1 != itend1; ++iter1, ++iter2) {
    auto d = *iter1 - *iter2;
    if (d < -tolerance) return -1;
    if (d > tolerance) return +1;
  }
  ASSERTX(iter2 == itend2);  // verify they have the same number of elements
  return 0;
}

// Check if a container contains an element.
template <typename Range, typename = enable_if_range_t<Range>>
bool contains(const Range& range, const iterator_t<Range>& elem) {
  for (const iterator_t<Range>& e : range)
    if (e == elem) return true;
  return false;
}

// For any container Range (e.g. Vec, Array, PArray, Grid, SGrid) supporting map(Range&, [](const T&) -> T)

// Convert all elements of the container to the new type U, e.g. convert<float>(V(1, 2)) == V(1.f, 2.f).
// Be careful to possibly use floor() before convert<int>() to avoid rounding negative values towards zero.
template <typename U, typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
auto convert(const Range& c) {
  return map(c, [](const Iterator& e) { return static_cast<U>(e); });
}

// Convert all elements of the container with runtime checking, e.g. narrow_convert<int>(V(1.f, 2.f)) == V(1, 2).
template <typename U, typename Range, typename = enable_if_range_t<Range>, typename Iterator = iterator_t<Range>>
auto narrow_convert(const Range& c) {
  return map(c, [](const Iterator& e) { return narrow_cast<U>(e); });
}

namespace details {
template <typename Iterator, typename Func> struct TransformedIterator {
  using type = TransformedIterator<Iterator, Func>;
  using iterator_category = std::forward_iterator_tag;
  using value_type = std::decay_t<decltype(std::declval<Func>()(*std::declval<Iterator>()))>;
  using difference_type = typename std::iterator_traits<Iterator>::difference_type;
  using pointer = value_type*;
  using reference = value_type&;
  Iterator _iter;
  const Func& _func;
  bool operator==(const type& rhs) const { return _iter == rhs._iter; }
  bool operator!=(const type& rhs) const { return !(*this == rhs); }
  decltype(auto) operator*() const { return _func(*_iter); }
  type& operator++() { return ++_iter, *this; }
  type& operator=(const type& rhs) {
    if (this != &rhs) _iter = rhs._iter;
    // Note that _func is a reference and there is no need to assign it.
    return *this;
  }
};

template <typename Range, typename Func> struct TransformedRange {
  Range _range;
  Func _func;
  auto begin() const {
    using std::begin;
    using Iterator = std::decay_t<decltype(begin(_range))>;
    return TransformedIterator<Iterator, Func>{begin(_range), _func};
  }
  auto end() const {
    using std::begin, std::end;
    using Iterator = std::decay_t<decltype(begin(_range))>;
    return TransformedIterator<Iterator, Func>{end(_range), _func};
  }
};
}  // namespace details

template <typename Range, typename Func, typename = enable_if_range_t<Range>>
auto transform(Range&& range, Func&& func = Func{}) {
  return details::TransformedRange<Range, Func>{std::forward<Range>(range), std::forward<Func>(func)};
}

namespace details {
template <typename Iterator1, typename Iterator2> struct ConcatenatedIterator {
  using type = ConcatenatedIterator<Iterator1, Iterator2>;
  using iterator_category = std::forward_iterator_tag;
  using value_type = typename std::iterator_traits<Iterator1>::value_type;
  using difference_type = typename std::iterator_traits<Iterator1>::difference_type;
  using pointer = typename std::iterator_traits<Iterator1>::pointer;
  using reference = typename std::iterator_traits<Iterator1>::reference;
  static_assert(std::is_same_v<value_type, typename std::iterator_traits<Iterator2>::value_type>);
  Iterator1 _begin1, _end1;
  Iterator2 _begin2;
  bool operator==(const type& rhs) const { return _begin1 == rhs._begin1 && _begin2 == rhs._begin2; }
  bool operator!=(const type& rhs) const { return !(*this == rhs); }
  decltype(auto) operator*() const { return _begin1 != _end1 ? *_begin1 : *_begin2; }
  type& operator++() {
    if (_begin1 != _end1)
      ++_begin1;
    else
      ++_begin2;
    return *this;
  }
};

template <typename Range1, typename Range2> struct ConcatenatedRange {
  Range1 _range1;
  Range2 _range2;
  auto begin() const {
    using std::begin, std::end;
    using Iterator1 = std::decay_t<decltype(begin(_range1))>;
    using Iterator2 = std::decay_t<decltype(begin(_range2))>;
    return ConcatenatedIterator<Iterator1, Iterator2>{begin(_range1), end(_range1), begin(_range2)};
  }
  auto end() const {
    using std::begin, std::end;
    using Iterator1 = std::decay_t<decltype(begin(_range1))>;
    using Iterator2 = std::decay_t<decltype(begin(_range2))>;
    return ConcatenatedIterator<Iterator1, Iterator2>{end(_range1), end(_range1), end(_range2)};
  }
};
}  // namespace details

template <typename Range1, typename Range2> auto concatenate(Range1&& range1, Range2&& range2) {
  return details::ConcatenatedRange<Range1, Range2>{std::forward<Range1>(range1), std::forward<Range2>(range2)};
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RANGEOP_H_
