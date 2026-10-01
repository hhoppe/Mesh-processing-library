// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_UNIONFIND_H_
#define MESH_PROCESSING_LIBHH_UNIONFIND_H_

#include "libHh/Array.h"
#include "libHh/Map.h"

namespace hh {

// Union-find is an efficient technique for tracking equivalence classes as pairs of elements are
// incrementally unified into the same class.
// We use path compression but without weight-balancing  -> worst case O(n log(n)), good case O(n).
template <typename T> class UnionFind {
  static_assert(Copyable<T>);

 public:
  void clear() { _m.clear(); }
  bool unify(T e1, T e2);                      // Put these two elements in the same class; returns: were_different.
  [[nodiscard]] bool equal(T e1, T e2) const;  // Are two elements in the same equivalence class?
  [[nodiscard]] T get_label(T e) const;        // Only valid until next unify().
  void promote(T e);                           // Ensure that e becomes the label for its equivalence class.
 private:
  // Default operator=() and copy constructor are safe.
  mutable Map<T, T> _m;  // Mutable because "irep()" performs path compression.
  T irep(T e) const;
};

//----------------------------------------------------------------------------

// Each non-root element maps to its parent; the root of each equivalence class (including any isolated element) is
// omitted from _m.

template <typename T> T UnionFind<T>::irep(T e) const {
  InlinedArray<T*, 10> ar;
  while (T* p = _m.find_ptr(e)) ar.push(p), e = *p;
  for (T* p : ar) *p = e;  // Path compression: e is now the root.
  return e;
}

template <typename T> bool UnionFind<T>::unify(T e1, T e2) {
  if (e1 == e2) return false;
  const T r1 = irep(e1);
  const T r2 = irep(e2);
  if (r1 == r2) return false;
  _m.enter(r1, r2);  // Root r1 (like root r2, absent from _m) becomes a child of r2, which remains a root.
  return true;
}

template <typename T> bool UnionFind<T>::equal(T e1, T e2) const { return e1 == e2 || irep(e1) == irep(e2); }

template <typename T> T UnionFind<T>::get_label(T e) const { return irep(e); }

template <typename T> void UnionFind<T>::promote(T e) {
  const T r = irep(e);
  if (r == e) return;
  _m.remove(e);    // Element e becomes the new root.
  _m.enter(r, e);  // And the old root points to it.
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_UNIONFIND_H_
