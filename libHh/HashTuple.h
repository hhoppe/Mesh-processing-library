// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HASHTUPLE_H_
#define MESH_PROCESSING_LIBHH_HASHTUPLE_H_

#include <memory>  // _WIN32 bug: must appear before <tuple> to avoid warning 4548
#include <tuple>   // std::tuple

#include "Advanced.h"  // hash_combine()

// Define hash functions for std::tuple<> and std::pair<>.
// Also define std::ostream operators for std::tuple<>.   (std::ostream for std::pair<> is in Hh.h)

namespace hh {
namespace details {

// Inspired from http://stackoverflow.com/questions/3611951/building-an-unordered-map-with-tuples-as-keys
template <typename TU, size_t Index = (std::tuple_size<TU>::value - 1)> struct tuple_hash {
  size_t operator()(const TU& tu) const {
    return hh::hash_combine(tuple_hash<TU, Index - 1>()(tu), std::get<Index>(tu));
  }
};

template <typename TU> struct tuple_hash<TU, 0> {
  size_t operator()(const TU& tu) const { return hh::my_hash(std::get<0>(tu)); }
};

template <typename TU, size_t Index = (std::tuple_size<TU>::value - 1)> struct tuple_write {
  void operator()(std::ostream& os, const TU& tu) const {
    tuple_write<TU, Index - 1>()(os, tu);
    os << ", " << std::get<Index>(tu);
  }
};

template <typename TU> struct tuple_write<TU, 0> {
  void operator()(std::ostream& os, const TU& tu) const { os << std::get<0>(tu); }
};

}  // namespace details
}  // namespace hh

namespace std {

template <typename... Types> struct hash<std::tuple<Types...>> {
  using TU = std::tuple<Types...>;
  size_t operator()(TU const& tu) const { return hh::details::tuple_hash<TU>()(tu); }
};

template <typename T1, typename T2> struct hash<std::pair<T1, T2>> {
  size_t operator()(const std::pair<T1, T2>& p) const { return ::hh::hash_combine(::hh::my_hash(p.first), p.second); }
};

template <typename... Types> std::ostream& operator<<(std::ostream& os, const std::tuple<Types...>& tu) {
  os << "tuple<";
  hh::details::tuple_write<std::tuple<Types...>>()(os, tu);
  os << ">";
  return os;
}

}  // namespace std

#endif  // MESH_PROCESSING_LIBHH_HASHTUPLE_H_
