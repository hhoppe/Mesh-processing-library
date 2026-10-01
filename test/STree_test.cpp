// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/STree.h"

#include <set>

#include "libHh/Array.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Set.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

// Random operations on an STree, compared with a std::set as a reference model.  The elements are nonzero so that
// they differ from the T{} returned when an element is absent.
void test_random_operations() {
  Random random(1);
  STree<int> stree;
  std::set<int> model;
  const auto model_pred = [&](int e) {  // The largest element less than e, or 0.
    auto it = model.lower_bound(e);
    return it != model.begin() ? *--it : 0;
  };
  const auto model_succ = [&](int e) {  // The smallest element greater than e, or 0.
    auto it = model.upper_bound(e);
    return it != model.end() ? *it : 0;
  };
  for_int(iter, 20'000) {
    const int e = 1 + int(random.get_unsigned(100));
    switch (random.get_unsigned(3)) {
      case 0: assertx(stree.enter(e) == model.insert(e).second); break;
      case 1: assertx(stree.remove(e) == (model.erase(e) == 1)); break;
      default: {
        const int e2 = int(random.get_unsigned(102));  // Also query values outside the range of the elements.
        assertx(stree.retrieve(e2) == (model.contains(e2) ? e2 : 0));
        assertx(stree.pred(e2) == model_pred(e2));
        assertx(stree.succ(e2) == model_succ(e2));
        assertx(stree.pred_eq(e2) == (model.contains(e2) ? e2 : model_pred(e2)));
        assertx(stree.succ_eq(e2) == (model.contains(e2) ? e2 : model_succ(e2)));
      }
    }
    assertx(stree.num() == narrow_cast<int>(model.size()) && stree.size() == model.size());
    assertx(stree.empty() == model.empty());
    if (!model.empty()) assertx(stree.min() == *model.begin() && stree.max() == *model.rbegin());
    if (iter % 1000 == 0) assertx(ranges::equal(stree, model));  // The iteration is in sorted order.
  }
  SHOW(stree.num());
  stree.clear();
  assertx(stree.empty() && stree.num() == 0 && stree.begin() == stree.end());
}

void test_edge_cases() {
  {
    STree<int> stree;
    assertx(stree.pred(5) == 0 && stree.succ(5) == 0 && stree.pred_eq(5) == 0 && stree.succ_eq(5) == 0);
    assertx(stree.enter(5));
    assertx(stree.min() == 5 && stree.max() == 5);
    assertx(stree.pred(5) == 0 && stree.succ(5) == 0 && stree.pred_eq(5) == 5 && stree.succ_eq(5) == 5);
    assertx(stree.pred(6) == 5 && stree.succ(4) == 5 && stree.pred_eq(4) == 0 && stree.succ_eq(6) == 0);
  }
  {  // With a reversed ordering, the predecessor is the next larger element.
    STree<int, std::greater<>> stree;
    for (const int e : {10, 30, 20}) stree.enter(e);
    SHOW(stree.min(), stree.max());
    SHOW(stree.pred(25), stree.succ(25), stree.pred_eq(20), stree.succ_eq(20), stree.pred(30), stree.succ(10));
    Array<int> ar(stree);
    SHOW(ar);
  }
  {
    STree<string> stree;
    for (const string s : {"pear", "apple", "fig", "apple"}) stree.enter(s);
    SHOW(Array<string>(stree));
    SHOW(stree.succ("b"), stree.pred("b"), stree.succ("pear") == "");
  }
}

}  // namespace

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

    assertx(stree.pred(23) == 22);
    assertx(stree.pred(1) == 0);
    assertx(stree.pred(60) == 58);
    assertx(stree.pred(4) == 2);
    assertx(stree.pred(58) == 56);
    assertx(stree.pred(18) == 16);

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
      Vec2<int> a;
    };
    const auto func_compare_astruct = [](const astruct& s1, const astruct& s2) {
      if (s1.a[0] != s2.a[0]) return s1.a[0] - s2.a[0];
      return s1.a[1] - s2.a[1];
    };
    struct less_astruct {
      bool operator()(const astruct& s1, const astruct& s2) const {
        return s1.a[0] != s2.a[0] ? s1.a[0] < s2.a[0] : s1.a[1] < s2.a[1];
      }
    };
    STree<astruct, less_astruct> stree;
    const astruct s1(1, 2), s2(3, 4), s3(1, 2);
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
        const int i = ib + io;
        assertx(!stree.retrieve(val[i]));
        assertx(stree.enter(val[i]));
      }
      unsigned check_order_last = 0;
      for (const auto& i : stree) {
        assertx(i >= check_order_last);
        check_order_last = i;
      }
      for_int(io, n - ib) {
        const int i = ib + io;
        assertx(stree.remove(val[i]));
        assertx(!stree.retrieve(val[i]));
      }
    }
  }
  test_random_operations();
  test_edge_cases();
}

template class hh::STree<unsigned>;
template class hh::STree<string>;
template class hh::STree<int, std::greater<int>>;
