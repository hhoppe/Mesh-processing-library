// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_RANGE_H_
#define MESH_PROCESSING_LIBHH_RANGE_H_

#include "Hh.h"

// Introduce structures to facilitate the processing of "ranges", containers or views whose elements can be
//  traversed using iterators initialized via begin() and end().
// Apparently this is moving towards standardization in the "Ranges" Technical Specification,
//  draft N4560 http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4560.pdf
#if 0
{
  // Count the number of elements equal to specified one; see RangeOp.h for other examples.
  template <typename R, typename = enable_if_range_t<R> > size_t count(R && range, const iterator_t<R>& elem) {
    using std::begin;
    using std::end;
    return std::count(begin(range), end(range), elem);
  }
}
#endif

namespace hh {

namespace details { template<typename R> struct has_begin; template<typename R> struct range_elem; }

// Determine if type R is a range (i.e. supports begin()).
template<typename R> struct is_range { static constexpr bool value = details::has_begin<R>::value; };

// SFINAE construct to enable a member function only if type R is a range (i.e. supports begin()).
template<typename R> using enable_if_range_t = std::enable_if_t<is_range<R>::value>;

// Identify the type of element in given range type (see also ~/src/test/native/container_element3.cpp).
template<typename R> using iterator_t = typename details::range_elem<R>::type;

//----------------------------------------------------------------------------

namespace details {
// http://stackoverflow.com/questions/9402476/can-i-get-the-iterator-for-a-template-type-regardless-if-that-type-is-an-arra/9402618
using std::begin;                                                      // fallback for ADL
template<typename T> auto adl_begin(T& t)       -> decltype(begin(t)); // only for type signature
template<typename T> auto adl_begin(const T& t) -> decltype(begin(t));
template<typename T> struct has_begin { // trait to identify if T has a begin(T) function.
    template <typename U> static char deduce(decltype(adl_begin(std::declval<const U&>()))*);
    template <typename>   static void deduce(...);
    static constexpr bool value = !std::is_void<decltype(deduce<T>(nullptr))>::value;
};
template<typename T> struct range_elem {
    using type = std::decay_t<decltype(*begin(std::declval<T&>()))>;
};
} // namespace details

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_RANGE_H_
