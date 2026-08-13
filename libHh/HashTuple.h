// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HASHTUPLE_H_
#define MESH_PROCESSING_LIBHH_HASHTUPLE_H_

#include <tuple>

#include "libHh/Advanced.h"  // hash_combine()

// Define hash functions for std::tuple<> and std::pair<>.
// Also define std::ostream operators for std::tuple<>.   (std::ostream for std::pair<> is in Hh.h)

template <typename... Types> struct std::hash<std::tuple<Types...>> {
  [[nodiscard]] size_t operator()(const std::tuple<Types...>& tu) const {
    // Inspired from https://stackoverflow.com/questions/3611951/building-an-unordered-map-with-tuples-as-keys
    return std::apply(
        [](const Types&... elements) {
          size_t h = 0;
          ((h = hh::hash_combine(h, elements)), ...);
          return h;
        },
        tu);
  }
};

template <typename T1, typename T2> struct std::hash<std::pair<T1, T2>> {
  [[nodiscard]] size_t operator()(const std::pair<T1, T2>& p) const {
    return hh::hash_combine(hh::hash_combine(0, p.first), p.second);
  }
};

#endif  // MESH_PROCESSING_LIBHH_HASHTUPLE_H_
