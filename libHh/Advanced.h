// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ADVANCED_H_
#define MESH_PROCESSING_LIBHH_ADVANCED_H_

#include "libHh/Hh.h"

namespace hh {

// Return a reference to a unique pointer object.  This reference may be invalid if the pointer is null!
// auto up_p = b ? make_unique<Point>(1.f, 2.f, 3.f) : nullptr;  Point& p = optional_reference(up_p);
template <typename T> T& optional_reference(const std::unique_ptr<T>& up) {
  return up ? *up : *implicit_cast<T*>(nullptr);
}

// Create a copy; make sure not to call on CArrayView, ArrayView, *View, etc. --- instead use Array<T>(v), etc.
template <typename T> std::decay_t<T> clone(T&& v) {
  // First, verify that T is default-constructible;
  //  my exotic classes like ArrayView and GridView are not default-constructible and should not be cloned.
  static_assert(std::is_default_constructible<std::decay_t<T>>::value, "");  // or maybe (void(T{}));
  // if r-value reference, return moved object;
  // else it is an l-value reference (const or not) and return a copy.
  // return std::is_rvalue_reference<T>::value ? std::move(v) : std::decay_t<T>(v);
  return std::decay_t<T>(std::forward<T>(v));  // std::decay_t<T> cast is necessary for explicit constructor
}
// Note: the following would lead to an extra move-construction if v is an rvalue
//    (see http://stackoverflow.com/questions/16724657/why-do-we-copy-then-move )
// Also, std::move is redundant?  see http://vmpstr.blogspot.com/2015/12/redundant-stdmove.html
// template<typename T> T clone(T v) { return std::move(v); }

// e.g.:  unroll<6>([&](int j) { _a[j] = min(l._a[j], r._a[j]); });
namespace details {
template <int i, int n> struct unroll_aux {
  template <typename Func = void(int)> void operator()(Func func) const {
    func(i);
    unroll_aux<i + 1, n>()(func);
  }
};
template <int n> struct unroll_aux<n, n> {
  template <typename Func = void(int)> void operator()(Func) const {}
};
template <int n> struct unroll_aux<n, -1> {  // awkward, avoid infinite recursive if n == -1 in addition to n == 0
  template <typename Func = void(int)> void operator()(Func) const {}
};
}  // namespace details
template <int n, typename Func = void(int)> void unroll(Func func) { details::unroll_aux<0, n>()(func); }

template <int n, int nmax, typename Func = void(int)> void unroll_max(Func func) {
  if (n <= nmax) {
    unroll<n>(func);
  } else {
    for_int(i, n) func(i);
  }
}

// Convenience function for hashing.
template <typename T> size_t my_hash(const T& v) { return std::hash<T>()(v); }

// Inspired from boost.  TODO: find a 64-bit version of this, for when size_t == uint64_t.
template <typename T> size_t hash_combine(size_t seed, const T& v) {
  return seed ^ (std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2));
}

// Nice syntax for in-order expansion of parameter packs.
//  examples: do_in_order { (process(args), 0)... };
//            do_in_order { i += args.num() ... };
// In C++17, replace by fold expression with comma operator; http://en.cppreference.com/w/cpp/language/fold
struct do_in_order {
  template <typename T> do_in_order(std::initializer_list<T>&&) {}
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_ADVANCED_H_
