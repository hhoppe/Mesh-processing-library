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
    return std::count(ranges::begin(range), ranges::end(range), elem);
  }
}
#endif

namespace hh {

// Determine if type T is a range.
// template <typename T> inline constexpr bool is_range_v = ranges::range<T>;

// SFINAE construct to enable a member function only if type T is a range.
// template <typename T> using enable_if_range_t = std::enable_if_t<ranges::range<T>>;

// Identify the Iterator type for range R.
// template <typename R> using range_iterator_t = ranges::iterator_t<R>;

// Identify the element type in range R.
// template <ranges::range R> using range_value_t = ranges::range_value_t<R>;

// Detect if range R supports size().
// template <typename R> inline constexpr bool range_has_size_v = ranges::sized_range<R>;

// Detect if the iterators for range R support random access.
// template <typename R> inline constexpr bool random_access_range_v = ranges::random_access_range<R>;

// Determine if Iterator supports random access.
// template <typename Iterator> inline constexpr bool random_access_iterator_v = std::random_access_iterator<Iterator>;

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RANGE_H_
