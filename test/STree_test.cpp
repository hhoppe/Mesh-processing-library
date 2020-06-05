// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/STree.h"

#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Set.h"
#include "libHh/Vec.h"
using namespace hh;

int main() {
  {
    STree<int> stree;
    for (int i = 2; i < 60; i += 2) stree.enter(i);
    assertx(stree.succ(1) == 2);
    assertx(stree.succ(2) == 4);
    assertx(stree.succ(18) == 20);
    assertx(stree.succ(23) == 24);
    assertx(stree.succ(58) == 0);
    assertx(stree.succ(60) == 0);
    //
    assertx(stree.pred(23) == 22);
    assertx(stree.pred(1) == 0);
    assertx(stree.pred(60) == 58);
    assertx(stree.pred(4) == 2);
    assertx(stree.pred(58) == 56);
    assertx(stree.pred(18) == 16);
    //
    assertx(stree.retrieve(12) == 12);
    assertx(stree.enter(88));
    assertx(stree.remove(24));
    assertx(!stree.remove(33));
    assertx(!stree.remove(24));
    assertx(sum(stree) == (1 + 29) * 29 / 2 * 2 + 88 - 24);
    for (int i = 2; i < 60; i += 2) assertx(stree.remove(i) == (i != 24));
    for (int i = 2; i < 60; i += 2) assertx(!stree.remove(i));
    assertx(stree.remove(88));
  }
  {
    struct astruct {
      explicit astruct(int x = 0, int y = 0) {
        a[0] = x;
        a[1] = y;
      }
      Vec<int, 2> a;
    };
    auto func_compare_astruct = [](const astruct& s1, const astruct& s2) {
      if (s1.a[0] != s2.a[0]) return s1.a[0] - s2.a[0];
      return s1.a[1] - s2.a[1];
    };
    struct less_astruct {
      bool operator()(const astruct& s1, const astruct& s2) const {
        // return func_compare_astruct(s1, s2) < 0;
        return s1.a[0] != s2.a[0] ? s1.a[0] < s2.a[0] : s1.a[1] < s2.a[1];
      }
    };
    STree<astruct, less_astruct> stree;
    astruct s1(1, 2), s2(3, 4), s3(1, 2);
    assertx(!func_compare_astruct(stree.retrieve(s3), astruct()));
    assertx(stree.enter(s1));
    assertx(!stree.enter(s1));
    assertx(func_compare_astruct(stree.retrieve(s3), astruct()));
    assertx(!stree.enter(s1));
    assertx(stree.enter(s2));
    assertx(!stree.enter(s3));
    assertx(stree.remove(s3));
    assertx(!stree.remove(s1));
    assertx(stree.remove(s2));
    assertx(!stree.remove(s2));
  }
  {
    const int n = 1000;
    Vec<unsigned, n> val;
    Set<unsigned> setv;
    for_int(i, n) {
      for (;;) {
        val[i] = Random::G.get_unsigned();
        if (setv.add(val[i])) break;
      }
    }
    STree<unsigned> stree;
    for (int ib = 0; ib < n; ib += 23) {
      for_int(io, n - ib) {
        int i = ib + io;
        assertx(!stree.retrieve(val[i]));
        assertx(stree.enter(val[i]));
      }
      unsigned check_order_last = 0;
      for (auto& i : stree) {
        assertx(i >= check_order_last);
        check_order_last = i;
      }
      for_int(io, n - ib) {
        int i = ib + io;
        assertx(stree.remove(val[i]));
        assertx(!stree.retrieve(val[i]));
      }
    }
  }
}

template class hh::STree<unsigned>;
