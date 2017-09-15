// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_RANGEOP_H_
#define MESH_PROCESSING_LIBHH_RANGEOP_H_

#include "Range.h"

namespace hh {

// Set of algorithms for manipulating ranges (including containers), i.e. supporting begin() and end() functions.

#if 1
// Extensions of functions from C++ <algorithm> as proposed by "Ranges" TS,
//  draft N4560 http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4560.pdf
// See http://en.cppreference.com/w/cpp/experimental/ranges#Ranges.

// Check unary predicate against elements; true if range is empty.
template<typename R, typename Pred, typename = enable_if_range_t<R> > bool all_of(const R& range, Pred p) {
    using std::begin; using std::end;
    return std::all_of(begin(range), end(range), p);
}

// Check unary predicate against elements; false if range is empty.
template<typename R, typename Pred, typename = enable_if_range_t<R> > bool any_of(const R& range, Pred p) {
    using std::begin; using std::end;
    return std::any_of(begin(range), end(range), p);
}

// Check unary predicate against elements; true if range is empty.
template<typename R, typename Pred, typename = enable_if_range_t<R> > bool none_of(const R& range, Pred p) {
    using std::begin; using std::end;
    return std::none_of(begin(range), end(range), p);
}

// Apply unary functor object to each element and returns functor.
template<typename R, typename Func, typename = enable_if_range_t<R> > Func for_each(R&& range, Func f) {
    using std::begin; using std::end;
    return std::for_each(begin(range), end(range), std::move(f));
}

// Return the address of the first element satisfying condition, or nullptr if none.
template<typename R, typename Pred, typename = enable_if_range_t<R> > auto find_if(R&& range, Pred p)
    -> iterator_t<R>* {
    using std::begin; using std::end;
    auto iter = std::find_if(begin(range), end(range), p);
    return iter==end(range) ? nullptr : &*iter;
}

// Return the address of the first element not satisfying condition, or nullptr if none.
template<typename R, typename Pred, typename = enable_if_range_t<R> > auto find_if_not(R&& range, Pred p)
    -> iterator_t<R>* {
    using std::begin; using std::end;
    auto iter = std::find_if_not(begin(range), end(range), p);
    return iter==end(range) ? nullptr : &*iter;
}

// Count the number of elements equal to specified one.
template<typename R, typename = enable_if_range_t<R> > auto count(const R& range, const iterator_t<R>& elem)
    -> std::ptrdiff_t {         // should return auto
    using std::begin; using std::end;
    return std::count(begin(range), end(range), elem);
}

// Count the number of elements matching predicate.
template<typename R, typename Pred, typename = enable_if_range_t<R> > auto count_if(const R& range, Pred p)
    -> std::ptrdiff_t {         // should return auto
    using std::begin; using std::end;
    return std::count_if(begin(range), end(range), p);
}

// Are two ranges equal element-wise?
template<typename R1, typename R2, typename Pred = std::equal_to<iterator_t<R1> >,
         typename = enable_if_range_t<R1>, typename = enable_if_range_t<R2> >
bool equal(const R1& range1, const R2& range2, Pred p = Pred{}) {
    using std::begin; using std::end;
    return std::equal(begin(range1), end(range1), begin(range2), end(range2), p);
}

// Swap the contents of two ranges.
template<typename R, typename R2, typename = enable_if_range_t<R>, typename = enable_if_range_t<R2> >
void swap_ranges(R&& range1, R2&& range2) {
    using std::begin; using std::end;
    auto iter1 = begin(range1), itend1 = end(range1);
    auto iter2 = begin(range2), itend2 = end(range2);
    if (0) {                    // Draft N4560 swaps the first min(size(range1), size(range2)) elements.
        if (itend2-iter2<itend1-iter1) { using std::swap; swap(iter1, iter2); swap(itend1, itend2); }
        std::swap_ranges(iter1, itend1, iter2);
    } else {                    // Instead I require that they have the same size.
        auto it = std::swap_ranges(iter1, itend1, iter2);
        ASSERTX(it==itend2);        // verify they have the same number of elements
    }
}

// Assign the same value to all elements in a range.
template<typename R, typename = enable_if_range_t<R> > R fill(R&& range, const iterator_t<R>& v) {
    // using std::begin; using std::end; std::fill(begin(range), end(range), v);
    for (auto& e : range) { e = v; }
    return std::forward<R>(range);
}

// Reverse the elements in a randomly accessible range.
template<typename R, typename = enable_if_range_t<R> > R reverse(R&& range) {
    using std::begin; using std::end;
    auto b = begin(range), e = end(range);
    ASSERTX(e>=b);              // requires random-access iterator
    // std::reverse(b, e);
    size_t num = e-b;
    for_size_t(i, num/2) { using std::swap; swap(b[i], b[num-1-i]); }
    return std::forward<R>(range);
}
// R reversed(const R& range) { return reverse(clone(range)); }

// Rotate the elements in a randomly accessible range such that element middle becomes the new first element.
template<typename R, typename = enable_if_range_t<R> > R rotate(R&& range, iterator_t<R>& middle) {
    using std::begin; using std::end;
    std::rotate(begin(range), &middle, end(range));
    return std::forward<R>(range);
}

// Sort the elements in a range (by default using less(a, b)).
template<typename R, typename Comp = std::less<iterator_t<R> >, typename = enable_if_range_t<R> >
R sort(R&& range, Comp comp = Comp{}) {
    using std::begin; using std::end;
    std::sort(begin(range), end(range), comp);
    return std::forward<R>(range);
}
// R sorted(const R& range) { return sort(clone(range)); }

// Minimum value in a non-empty range (by default using less(a, b)).
template<typename R, typename Comp = std::less<iterator_t<R> >, typename = enable_if_range_t<R> >
iterator_t<R> min(const R& range, Comp comp = Comp{}) {
    using std::begin; using std::end;
    ASSERTXX(begin(range)!=end(range));
    return *std::min_element(begin(range), end(range), comp);
    // auto iter = begin(range), itend = end(range); ASSERTX(iter!=itend);
    // auto v = *iter; ++iter; for (; iter!=itend; ++iter) { v = min(v, *iter); }
    // return v;
}

// Maximum value in a non-empty range (using less(a, b)).
template<typename R, typename Comp = std::less<iterator_t<R> >, typename = enable_if_range_t<R> >
iterator_t<R> max(const R& range, Comp comp = Comp{}) {
    using std::begin; using std::end;
    ASSERTXX(begin(range)!=end(range));
    return *std::max_element(begin(range), end(range), comp);
    // auto iter = begin(range), itend = end(range); ASSERTX(iter!=itend);
    // auto v = *iter; ++iter; for (; iter!=itend; ++iter) { v = max(v, *iter); }
    // return v;
}

// Number of elements in a range (could also define size(R) but for robustness that would require Concepts).
template<typename R, typename = enable_if_range_t<R> > std::ptrdiff_t distance(const R& range) {
    using std::begin; using std::end;
    return std::distance(begin(range), end(range));
}

// Also: find(), find_end(), find_first_of(), adjacent_find(), mismatch(), equal()
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

#endif  // "Ranges" TS, draft N4560



// Higher-precision type to represent the mean of a set of elements.
template<typename T> struct mean_type {
    using type = std::conditional_t<!std::is_arithmetic<T>::value, T,
                                    std::conditional_t<std::is_floating_point<T>::value, double,
                                                       std::conditional_t<sizeof(T)>=4, double, float> > >;
};
template<typename T> using mean_type_t = typename mean_type<T>::type;

// Preferred floating-point type (either float or double) used to multiply a given element type.
template<typename T> struct factor_type {
    // Some user-defined types T (such as Vector4, Point, Vector) only support T*float, not T*double.
    // T*double is supported for all built-in arithmetic types; otherwise default is T*float.
    // User may override factor_type for a new type T supporting T*double.
    using type = std::conditional_t<std::is_arithmetic<T>::value, double, float>;
};
template<typename T> using factor_type_t = typename factor_type<T>::type;

// Minimum value in a non-empty range (using less(a, b)); store its index [0..size(range)-1] in *pi.
template<typename R, typename = enable_if_range_t<R> > iterator_t<R> min_index(const R& range, int* pi) {
    using std::begin; using std::end;
    ASSERTX(pi); ASSERTX(begin(range)!=end(range));
    auto p = std::min_element(begin(range), end(range));
    *pi = narrow_cast<int>(std::distance(begin(range), p));
    return *p;
}

// Maximum value in a non-empty range (using less(a, b)); store its index [0..size(range)-1] in *pi.
template<typename R, typename = enable_if_range_t<R> > iterator_t<R> max_index(const R& range, int* pi) {
    using std::begin; using std::end;
    ASSERTX(pi); ASSERTXX(begin(range)!=end(range));
    auto p = std::max_element(begin(range), end(range));
    *pi = narrow_cast<int>(std::distance(begin(range), p));
    return *p;
}

// Minimum over a non-empty range of values (using successive min(a, b) rather than less(a, b)).
template<typename R, typename = enable_if_range_t<R> > iterator_t<R> transitive_min(const R& range) {
    using T = iterator_t<R>;
    using std::begin; using std::end;
    auto iter = begin(range), itend = end(range); ASSERTX(iter!=itend);
    T v = *iter; ++iter; for (; iter!=itend; ++iter) { v = min(v, *iter); }
    return v;
}

// Maximum over a non-empty range of values (using successive min(a, b) rather than less(a, b)).
template<typename R, typename = enable_if_range_t<R> > iterator_t<R> transitive_max(const R& range) {
    using T = iterator_t<R>;
    using std::begin; using std::end;
    auto iter = begin(range), itend = end(range); ASSERTX(iter!=itend);
    T v = *iter; ++iter; for (; iter!=itend; ++iter) { v = max(v, *iter); }
    return v;
}

// Maximum absolute value in a non-empty range.
template<typename R, typename = enable_if_range_t<R> > iterator_t<R> max_abs_element(const R& range) {
    using T = iterator_t<R>;
    using std::begin; using std::end;
    auto iter = begin(range), itend = end(range); ASSERTX(iter!=itend);
    // return abs(*std::max_element(ibeg, iend, [](T l, T r) { return abs(l)<abs(r); }));
    T v = static_cast<T>(abs(*iter)); ++iter; for (; iter!=itend; ++iter) { v = max(v, static_cast<T>(abs(*iter))); }
    // auto iter_max = iter; T v = static_cast<T>(abs(*iter)); ++iter;
    // for (; iter!=itend; ++iter) { T vt = static_cast<T>(abs(*iter)); if (vt>v) { v = vt; iter_max = iter; } }
    // if (pi) *pi = narrow_cast<int>(std::distance(begin(range), iter_max));
    return v;
}

// Sum of values in a range.
template<typename R, typename = enable_if_range_t<R> > sum_type_t<iterator_t<R> > sum(const R& range) {
    using T = iterator_t<R>;
    using SumType = sum_type_t<T>;
    using std::begin; using std::end;
    // return std::accumulate(begin(range), end(range), SumType{});
    auto iter = begin(range), itend = end(range);
    if (iter==itend) { SumType v; my_zero(v); return v; }
    SumType v = *iter; ++iter; for (; iter!=itend; ++iter) { v += *iter; }
    return v;
}

// Average of values in a range.
template<typename R, typename = enable_if_range_t<R> > mean_type_t<iterator_t<R> > mean(const R& range) {
    using T = iterator_t<R>;
    using MeanType = mean_type_t<T>;
    using std::begin; using std::end;
    auto iter = begin(range), itend = end(range);
    if (iter==itend) { Warning("mean"); MeanType v; my_zero(v); return v; }
    MeanType v = *iter; size_t num = 1; ++iter; for (; iter!=itend; ++iter) { v += *iter; num++; }
    using Factor = factor_type_t<iterator_t<R>>;
    return MeanType(v*(Factor(1)/num));
}

// Sum of squared values in a range (or zero if empty).
template<typename R, typename = enable_if_range_t<R> > sum_type_t<iterator_t<R> > mag2(const R& range) {
    using T = iterator_t<R>;
    using SumType = sum_type_t<T>;
    using std::begin; using std::end;
    // return std::accumulate(begin(range), end(range), SumType{},
    //                        [](const SumType& l, const T& r) { return l+square(r); });
    auto iter = begin(range), itend = end(range);
    if (iter==itend) { SumType v; my_zero(v); return v; }
    SumType v = square(SumType(*iter)); ++iter; for (; iter!=itend; ++iter) { v += square(SumType(*iter)); }
    return v;
}

// Root sum of squared values in a range.
template<typename R, typename = enable_if_range_t<R> > mean_type_t<iterator_t<R> > mag(const R& range) {
    using T = iterator_t<R>;
    using MeanType = mean_type_t<T>;
    return sqrt(MeanType(mag2(range)));
}

// Root mean square of values in a range.
template<typename R, typename = enable_if_range_t<R> > mean_type_t<iterator_t<R> > rms(const R& range) {
    using T = iterator_t<R>;
    using MeanType = mean_type_t<T>;
    MeanType v; my_zero(v); size_t num = 0;
    for (const auto& e : range) { v += square(MeanType(e)); num++; }
    if (!num) { Warning("rms() of empty range"); return v; }
    using Factor = factor_type_t<iterator_t<R>>;
    return sqrt(MeanType(v*(Factor(1)/num)));
}

// Variance of values in a range.
template<typename R, typename = enable_if_range_t<R> > mean_type_t<iterator_t<R> > var(const R& range) {
    using T = iterator_t<R>;
    using MeanType = mean_type_t<T>;
    MeanType zero, v, v2; my_zero(zero); my_zero(v); my_zero(v2); size_t num = 0;
    for (const auto& e : range) { v += e; v2 += square(MeanType(e)); num++; }
    if (num<2) { Warning("var() of fewer than 2 elements"); return zero; }
    using Factor = factor_type_t<iterator_t<R>>;
    return max(MeanType((v2-v*v*(Factor(1)/num))*(Factor(1)/(num-Factor(1)))), zero);
}

// Product of values in a non-empty range.
template<typename R, typename = enable_if_range_t<R> > sum_type_t<iterator_t<R> > product(const R& range) {
    using T = iterator_t<R>;
    using SumType = sum_type_t<T>;
    using std::begin; using std::end;
    auto iter = begin(range), itend = end(range); ASSERTX(iter!=itend);
    // return std::accumulate(begin(range), end(range), SumType{1}, [](const SumType& l, const T& r) { return l*r; });
    SumType v = *iter; ++iter; for (; iter!=itend; ++iter) { v *= *iter; }
    return v;
}

// Are all elements exactly zero?
template<typename R, typename = enable_if_range_t<R> > bool is_zero(const R& range) {
    for (const auto& e : range) { if (e) return false; }
    return true;
}

// Modify the range to have unit norm (or die if input has zero norm).
template<typename R, typename = enable_if_range_t<R> > R normalize(R&& range) {
    using T = iterator_t<R>;
    T v = static_cast<T>(T{1.f}/assertx(mag(range)));
    for (auto& e : range) { e = static_cast<T>(e*v); }
    return std::forward<R>(range);
}
// R normalized(const R& range) { return normalize(clone(range)); }

// Round the values in a range to the nearest 1/fac increment (by default fac==1e5f).
template<typename R, typename = enable_if_range_t<R> > R round_elements(R&& range, iterator_t<R> fac = 1e5f) {
    using T = iterator_t<R>;
    static_assert(std::is_floating_point<T>::value, "");
    for (auto& e : range) { e = round_fraction_digits(e, fac); }
    return std::forward<R>(range);
}

// Compute the sum of squared differences of corresponding elements of two ranges.
template<typename R, typename R2, typename = enable_if_range_t<R>, typename = enable_if_range_t<R2> >
sum_type_t<iterator_t<R> > dist2(const R& range1, const R2& range2) {
    using T = iterator_t<R>;
    using SumType = sum_type_t<T>;
    using std::begin; using std::end;
    auto iter1 = begin(range1), itend1 = end(range1);
    auto iter2 = begin(range2), itend2 = end(range2);
    SumType v; my_zero(v);
    for (; iter1!=itend1; ++iter1, ++iter2) v += square(SumType(*iter1)-SumType(*iter2));
    ASSERTX(iter2==itend2);      // verify they have the same number of elements
    return v;
}

// Compute the Euclidean distance between two ranges interpreted as vectors.
template<typename R, typename R2, typename = enable_if_range_t<R>, typename = enable_if_range_t<R2> >
mean_type_t<iterator_t<R> > dist(const R& range1, const R2& range2) {
    using T = iterator_t<R>;
    using MeanType = mean_type_t<T>;
    return sqrt(MeanType(dist2(range1, range2)));
}

// Compute the inner product of two ranges.
template<typename R, typename R2, typename = enable_if_range_t<R>, typename = enable_if_range_t<R2> >
sum_type_t<iterator_t<R> > dot(const R& range1, const R2& range2) {
    using T = iterator_t<R>;
    using SumType = sum_type_t<T>;
    using std::begin; using std::end;
    auto iter1 = begin(range1), itend1 = end(range1);
    auto iter2 = begin(range2), itend2 = end(range2);
    SumType v; my_zero(v);
    for (; iter1!=itend1; ++iter1, ++iter2) v += SumType(*iter1)*SumType(*iter2);
    ASSERTX(iter2==itend2);      // verify they have the same number of elements
    return v;
}

// Compare two ranges of algebraic types lexicographically; ret -1, 0, 1 based on sign of range1-range2.
template<typename R, typename R2, typename = enable_if_range_t<R>, typename = enable_if_range_t<R2> >
int compare(const R& range1, const R2& range2) {
    using std::begin; using std::end;
    auto iter1 = begin(range1), itend1 = end(range1);
    auto iter2 = begin(range2), itend2 = end(range2);
    for (; iter1!=itend1; ++iter1, ++iter2) {
        auto d = *iter1-*iter2; if (d) return d<0 ? -1 : +1;
    }
    ASSERTX(iter2==itend2);      // verify they have the same number of elements
    return 0;
}

// Similar comparison, but ignore differences smaller than tolerance.
template<typename R, typename R2, typename = enable_if_range_t<R>, typename = enable_if_range_t<R2> >
int compare(const R& range1, const R2& range2, const iterator_t<R>& tolerance) {
    using std::begin; using std::end;
    auto iter1 = begin(range1), itend1 = end(range1);
    auto iter2 = begin(range2), itend2 = end(range2);
    for (; iter1!=itend1; ++iter1, ++iter2) {
        auto d = *iter1-*iter2; if (d<-tolerance) return -1; else if (d>tolerance) return +1;
    }
    ASSERTX(iter2==itend2);      // verify they have the same number of elements
    return 0;
}

// Check if a container contains an element.
template<typename R, typename = enable_if_range_t<R> > bool contains(const R& range, const iterator_t<R>& elem) {
    for (const iterator_t<R>& e : range) { if (e==elem) return true; }
    return false;
}

// For any container R (e.g. Vec, Array, PArray, Grid, SGrid) supporting map(R&, [](const T&)->T)

// Convert all elements of the container to the new type U, e.g. convert<float>(V(1, 2))==V(1.f, 2.f).
// Be careful to possibly use floor() before convert<int>() to avoid rounding negative values towards zero.
template<typename U, typename R, typename = enable_if_range_t<R> > auto convert(R&& c)
    -> decltype(map(c, std::declval<U(*)(const iterator_t<R>&)>())) {
    return map(c, [](const iterator_t<R>& e) { return static_cast<U>(e); });
}

// Convert all elements of the container with runtime checking, e.g. narrow_convert<int>(V(1.f, 2.f))==V(1, 2).
template<typename U, typename R, typename = enable_if_range_t<R> > auto narrow_convert(const R& c)
    -> decltype(map(c, std::declval<U(*)(const iterator_t<R>&)>())) {
    return map(c, [](const iterator_t<R>& e) { return narrow_cast<U>(e); });
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_RANGEOP_H_
