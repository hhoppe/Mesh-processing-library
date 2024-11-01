// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_UNIONFIND_H_
#define MESH_PROCESSING_LIBHH_UNIONFIND_H_

#include "libHh/Map.h"
#include "libHh/PArray.h"

namespace hh {

// Union-find is an efficient technique for tracking equivalence classes as pairs of elements are
//   incrementally unified into the same class.
// Uses path compression but without weight-balancing  -> worst case O(nlogn), good case O(n)
template <typename T> class UnionFind {
 public:
  void clear() { _m.clear(); }
  bool unify(T e1, T e2);        // put these two elements in the same class; returns: were_different
  bool equal(T e1, T e2) const;  // are two elements in the same equivalence class?
  T get_label(T e) const;        // only valid until next unify()
  void promote(T e);             // ensure that e becomes the label for its equivalence class
 private:
  // Default operator=() and copy constructor are safe.
  mutable Map<T, T> _m;  // Mutable because "equal(e1, e2)" can perform path compression.
  T irep(T e, bool& present) const;
};

//----------------------------------------------------------------------------

template <typename T> T UnionFind<T>::irep(T e, bool& present) const {
  T parent = _m.retrieve(e, present);
  if (!present || parent == e) return e;
  PArray<T*, 10> ar;
  {
    for (;;) {
      T* p = &_m.get(e);
      if (*p == e) break;
      ar.push(p);
      e = *p;
    }
  }
  // e now contains root; update all nodes along path to root.
  for (T* p : ar) *p = e;
  return e;
}

template <typename T> bool UnionFind<T>::unify(T e1, T e2) {
  if (e1 == e2) return false;
  bool present;
  T r1 = irep(e1, present);
  if (!present) _m.enter(e1, e1);
  T r2 = irep(e2, present);
  if (!present) _m.enter(e2, e2);
  if (r1 == r2) return false;
  _m.replace(r1, r2);
  return true;
}

template <typename T> bool UnionFind<T>::equal(T e1, T e2) const {
  if (e1 == e2) return true;
  bool present;
  T r1 = irep(e1, present);
  T r2 = irep(e2, present);
  return r1 == r2;
}

template <typename T> T UnionFind<T>::get_label(T e) const {
  bool present;
  T r = irep(e, present);
  return r;
}

template <typename T> void UnionFind<T>::promote(T e) {
  bool present;
  T r = irep(e, present);
  if (r == e) return;
  _m.replace(e, e);
  _m.replace(r, e);
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_UNIONFIND_H_
