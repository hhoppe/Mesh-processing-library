// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Map.h"

namespace hh {


// Union-find is an efficient technique for tracking equivalence classes as pairs of elements are
//   incrementally unified into the same class.
// Uses path compression but without weight-balancing  -> worst case O(nlogn), good case O(n)
template<typename T> class UnionFind : noncopyable {
 public:
    void clear()                                { _m.clear(); }
    bool unify(T e1, T e2);     // put these two elements in the same class; returns: were_different
    bool equal(T e1, T e2);     // are two elements in the same equivalence class?
    T get_label(T e);           // only valid until next unify()
 private:
    Map<T,T> _m;
    T irep(T e, bool& present);
};


//----------------------------------------------------------------------------

template<typename T> T UnionFind<T>::irep(T e, bool& present) {
    // Possible optimization: build up PArray<T*,10> of pointers into Map nodes
    T r = _m.retrieve(e, present);
    if (!present) return T();   // alone, ret anything
    T t = e;
    while (r!=t) r = _m.get(t = r);
    while (e!=r) e = _m.replace(e, r);
    return r;
}

template<typename T> bool UnionFind<T>::unify(T e1, T e2) {
    if (e1==e2) return false;
    bool present;
    T r1 = irep(e1, present); if (!present) _m.enter(r1 = e1, e1);
    T r2 = irep(e2, present); if (!present) _m.enter(r2 = e2, e2);
    if (r1==r2) return false;
    _m.replace(r1, r2);
    return true;
}

template<typename T> bool UnionFind<T>::equal(T e1, T e2) {
    if (e1==e2) return true;
    bool present;
    T r1 = irep(e1, present); if (!present) return false;
    T r2 = irep(e2, present); if (!present) return false;
    return r1==r2;
}

template<typename T> T UnionFind<T>::get_label(T e) {
    bool present; T r = irep(e, present);
    if (!present) return e;
    return r;
}

} // namespace hh
