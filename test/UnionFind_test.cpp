// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/UnionFind.h"

#include "libHh/Random.h"
#include "libHh/Set.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

// Random unifications, compared with a brute-force model that stores a class number for each element.
void test_random_operations() {
  Random random(1);
  const int n = 1000;
  UnionFind<int> uf;
  Array<int> model_class(n);
  for_int(e, n) model_class[e] = e;
  int nclasses = n;
  for_int(iter, 8000) {
    const int e1 = int(random.get_unsigned(n)), e2 = int(random.get_unsigned(n));
    switch (random.get_unsigned(8)) {
      case 0: {
        const int c1 = model_class[e1], c2 = model_class[e2];
        assertx(uf.unify(e1, e2) == (c1 != c2));
        if (c1 != c2) {
          for (int& c : model_class) c = c == c1 ? c2 : c;
          nclasses--;
        }
        break;
      }
      case 1:
        uf.promote(e1);
        assertx(uf.get_label(e1) == e1);
        break;
      default: assertx(uf.equal(e1, e2) == (model_class[e1] == model_class[e2]));
    }
    if (iter % 200 == 0) {  // Each label is a member of its class, and two elements share a label iff equivalent.
      Array<int> labels(n);
      for_int(e, n) labels[e] = uf.get_label(e);
      Set<int> distinct_labels;
      for_int(e, n) {
        assertx(model_class[labels[e]] == model_class[e]);
        assertx(uf.get_label(labels[e]) == labels[e]);  // The label is the root of its class.
        distinct_labels.add(labels[e]);
      }
      assertx(distinct_labels.num() == nclasses);
    }
  }
  SHOW(nclasses);
  uf.clear();  // All elements are again in separate classes.
  for_int(e, n) assertx(uf.get_label(e) == e && !uf.equal(e, (e + 1) % n));
}

void test_edge_cases() {
  UnionFind<int> uf;
  assertx(uf.equal(3, 3) && !uf.equal(3, 4));  // An element is always equivalent to itself.
  assertx(!uf.unify(3, 3));                    // Unifying an element with itself changes nothing.
  assertx(uf.get_label(3) == 3);
  uf.promote(3);  // Promoting an isolated element changes nothing.
  assertx(uf.get_label(3) == 3 && !uf.equal(3, 4));
  // A long chain of unifications, which path compression later shortens.
  for_int(i, 1000) assertx(uf.unify(i, i + 1));
  assertx(uf.equal(0, 1000) && !uf.equal(0, 1001));
  const int label = uf.get_label(0);
  for_int(i, 1001) assertx(uf.get_label(i) == label);
  SHOW(label);
}

}  // namespace

int main() {
  {
    // using PairInt = std::pair<int, int>;
    using PairInt = Vec2<int>;
    UnionFind<PairInt> uf;
    SHOW(uf.get_label(PairInt(1, 7)));
    SHOW(uf.unify(PairInt(1, 1), PairInt(3, 2)));
    SHOW(uf.get_label(PairInt(3, 2)));
    SHOW(uf.get_label(PairInt(1, 1)));
    SHOW(uf.get_label(PairInt(1, 7)));
    SHOW(uf.unify(PairInt(3, 2), PairInt(5, 4)));
    SHOW(uf.unify(PairInt(7, 6), PairInt(1, 1)));
    SHOW(uf.unify(PairInt(1, 1), PairInt(3, 2)));
    SHOW(uf.get_label(PairInt(3, 2)));
    SHOW(uf.get_label(PairInt(1, 1)));
    SHOW(uf.get_label(PairInt(1, 7)));
    SHOW(uf.get_label(PairInt(7, 6)));
    SHOW(uf.get_label(PairInt(5, 4)));
  }
  {
    UnionFind<int> uf;
    SHOW(uf.unify(1, 2));
    SHOW(uf.unify(11, 12));
    SHOW(uf.unify(1, 3));
    SHOW(uf.unify(1, 4));
    SHOW(uf.unify(4, 5));
    SHOW(uf.unify(5, 6));
    SHOW(uf.unify(7, 4));
    SHOW(uf.unify(11, 13));
    SHOW(uf.unify(13, 14));
    SHOW(uf.unify(15, 13));
    SHOW(uf.unify(16, 11));
    SHOW(uf.unify(0, 1));
    SHOW(uf.unify(20, 0));
    SHOW(uf.unify(19, 0));
    SHOW(uf.unify(19, 20));
    SHOW(uf.unify(16, 12));
    SHOW(uf.unify(5, 7));
    SHOW(uf.unify(12, 3));
    SHOW(uf.unify(12, 14));
    SHOW(uf.unify(13, 7));
    SHOW(uf.unify(1, 2));
  }
  {
    UnionFind<int> uf;
    SHOW(uf.get_label(5));
    uf.unify(1, 2);
    SHOW(uf.get_label(1));
    SHOW(uf.get_label(2));
    uf.promote(1);
    SHOW(uf.get_label(1));
    SHOW(uf.get_label(2));
    uf.promote(2);
    SHOW(uf.get_label(1));
    SHOW(uf.get_label(2));

    uf.unify(3, 4);
    SHOW(uf.get_label(3));
    SHOW(uf.get_label(4));
    uf.unify(1, 3);
    SHOW(uf.get_label(1));
    SHOW(uf.get_label(2));
    SHOW(uf.get_label(3));
    SHOW(uf.get_label(4));
    uf.promote(3);
    SHOW(uf.get_label(1));
    SHOW(uf.get_label(2));
    SHOW(uf.get_label(3));
    SHOW(uf.get_label(4));
    uf.promote(1);
    SHOW(uf.get_label(1));
    SHOW(uf.get_label(2));
    SHOW(uf.get_label(3));
    SHOW(uf.get_label(4));

    UnionFind<int> uf2;
    SHOW(uf2.get_label(1));
    SHOW(uf2.get_label(2));
    SHOW(uf2.get_label(3));
    SHOW(uf2.get_label(4));
    SHOW(uf2.get_label(5));
    uf2 = uf;  // NOLINT(performance-use-std-move): deliberately exercising the copy assignment.
    SHOW(uf2.get_label(1));
    SHOW(uf2.get_label(2));
    SHOW(uf2.get_label(3));
    SHOW(uf2.get_label(4));
    SHOW(uf2.get_label(5));
    SHOW(uf.equal(2, 4), uf.equal(2, 5), uf2.equal(4, 3));
  }
  test_random_operations();
  test_edge_cases();
}

template class hh::UnionFind<unsigned>;
template class hh::UnionFind<Vec2<int>>;
