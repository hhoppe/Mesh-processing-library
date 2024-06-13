// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_RANGE_H_
#define MESH_PROCESSING_LIBHH_RANGE_H_

#include "libHh/Hh.h"

// Introduce structures to facilitate the processing of "ranges", containers or views whose elements can be
//  traversed using iterators initialized via begin() and end().
// Apparently this is moving towards standardization in the "Ranges" Technical Specification,
//  draft N4560 https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4560.pdf
#if 0
{
  // Count the number of elements equal to specified one; see RangeOp.h for other examples.
  template <typename Range, typename = enable_if_range_t<Range>>
  size_t count(Range && range, const range_value_t<Range>& elem) {
    using std::begin, std::end;
    return std::count(begin(range), end(range), elem);
  }
}
#endif

namespace hh {

namespace details {
template <typename T> struct has_begin;
template <typename T> struct range_value;
template <typename T, typename = std::void_t<>> struct range_has_size;
template <typename T> struct has_random_access;
}  // namespace details

// Determine if type T is a range (i.e. supports begin()).
template <typename T> inline constexpr bool is_range_v = details::has_begin<T>::value;

// SFINAE construct to enable a member function only if type T is a range (i.e. supports begin()).
template <typename T> using enable_if_range_t = std::enable_if_t<is_range_v<T>>;

// Identify the type of element in given range type.
template <typename Range> using range_value_t = typename details::range_value<Range>::type;

// Detect if Range has a size() member function.
template <typename Range> inline constexpr bool range_has_size_v = details::range_has_size<Range>::value;

// Detect if Range supports random-access iterators.
template <typename Range> inline constexpr bool range_has_random_access_v = details::has_random_access<Range>::value;

//----------------------------------------------------------------------------

namespace details {

// https://stackoverflow.com/questions/9402476
using std::begin;                                                  // fallback for ADL
template <typename T> auto adl_begin(T& t) -> decltype(begin(t));  // only for type signature
template <typename T> auto adl_begin(const T& t) -> decltype(begin(t));
template <typename T> struct has_begin {  // trait to identify if T has a begin(T) function.
  template <typename U> static char deduce(decltype(adl_begin(std::declval<const U&>()))*);
  template <typename> static void deduce(...);
  static constexpr bool value = !std::is_void_v<decltype(deduce<T>(nullptr))>;
};

template <typename Range> struct range_value {
  using type = std::decay_t<decltype(*begin(std::declval<Range&>()))>;
};

template <typename, typename> struct range_has_size : std::false_type {};
template <typename Range>
struct range_has_size<Range, std::void_t<decltype(std::declval<Range>().size())>> : std::true_type {};

template <typename Range> using range_iterator_t = decltype(std::begin(std::declval<Range>()));
template <typename Iterator> using iterator_category_t = typename std::iterator_traits<Iterator>::iterator_category;
template <typename Range>
struct has_random_access
    : std::is_same<iterator_category_t<range_iterator_t<Range>>, std::random_access_iterator_tag> {};

}  // namespace details

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RANGE_H_
