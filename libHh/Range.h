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
  template <typename R, typename = enable_if_range_t<R>> size_t count(R && range, const iterator_t<R>& elem) {
    using std::begin, std::end;
    return std::count(begin(range), end(range), elem);
  }
}
#endif

namespace hh {

namespace details {
template <typename T> struct has_begin;
template <typename T> struct range_elem;
}  // namespace details

// Determine if type T is a range (i.e. supports begin()).
template <typename T> struct is_range { static constexpr bool value = details::has_begin<T>::value; };
template <typename T> constexpr bool is_range_v = is_range<T>::value;

// SFINAE construct to enable a member function only if type T is a range (i.e. supports begin()).
template <typename T> using enable_if_range_t = std::enable_if_t<is_range_v<T>>;

// Identify the type of element in given range type (see also ~/git/hh_src/test/native/container_element3.cpp).
template <typename T> using iterator_t = typename details::range_elem<T>::type;

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
template <typename T> struct range_elem { using type = std::decay_t<decltype(*begin(std::declval<T&>()))>; };
}  // namespace details

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RANGE_H_
