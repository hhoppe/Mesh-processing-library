// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/HashTuple.h"

#include "libHh/Set.h"
using namespace hh;

int main() {
  {
    using TU = std::tuple<int, float, bool>;
    TU tu1 = std::tuple(1, 2.f, true);
    SHOW(tu1);
    assertx(my_hash(tu1) == my_hash(std::tuple(1, 2.f, true)));
    Set<size_t> set;  // Verify all unique.  (The hash values themselves differ across platforms.)
    assertx(set.add(my_hash(std::tuple(1, 2.f, true))));
    assertx(set.add(my_hash(std::tuple(1, 2.f, false))));
    assertx(set.add(my_hash(std::tuple(1, 3.f, true))));
    assertx(set.add(my_hash(std::tuple(2, 2.f, true))));
  }
  {
    using TU = std::tuple<int, float, double*>;
    double d1, d2;
    const TU tu1 = std::tuple(1, 2.f, &d1);
    assertx(my_hash(tu1) == my_hash(std::tuple(1, 2.f, &d1)));
    assertx(my_hash(tu1) != my_hash(std::tuple(1, 2.f, &d2)));
    assertx(my_hash(tu1) != my_hash(std::tuple(1, 3.f, &d1)));
    assertx(my_hash(tu1) != my_hash(std::tuple(2, 2.f, &d1)));
  }
  {
    std::tuple<int> tu2(3);
    SHOW(tu2);
    SHOW(std::tuple(1, 2.f, false, 5.));
    SHOW(std::pair(1, 2.f));
  }
  {
    std::pair p{5, true};
    SHOW(p);
  }
  {
    assertx(my_hash(std::tuple<>()) == 0);  // The hash of an empty tuple is the initial seed.
    // A pair and a two-element tuple hash identically.
    assertx(my_hash(std::pair(1, 2)) == my_hash(std::tuple(1, 2)));
    assertx(my_hash(std::pair(string("a"), 2.5)) == my_hash(std::tuple(string("a"), 2.5)));
    // The hash depends on the order of the elements.
    assertx(my_hash(std::pair(1, 2)) != my_hash(std::pair(2, 1)));
    assertx(my_hash(std::tuple(1, 2, 3)) != my_hash(std::tuple(3, 2, 1)));
    // The hash of a single-element tuple differs from that of its element.
    assertx(my_hash(std::tuple(7)) != my_hash(7));
    // Nested tuples and pairs are hashable.
    using Nested = std::tuple<std::tuple<int>, std::pair<int, string>>;
    const Nested nested1{std::tuple(1), std::pair(2, "b")}, nested2{std::tuple(1), std::pair(2, "c")};
    assertx(my_hash(nested1) == my_hash(Nested{std::tuple(1), std::pair(2, "b")}));
    assertx(my_hash(nested1) != my_hash(nested2));
  }
  {  // All pairs in a grid have distinct hash values, and they serve as keys of a Set.
    const int n = 100;
    Set<size_t> hashes;
    Set<std::pair<int, int>> pairs;
    for_int(i, n) for_int(j, n) {
      assertx(hashes.add(my_hash(std::pair(i, j))));
      pairs.enter(std::pair(i, j));
    }
    assertx(pairs.num() == n * n);
    for_int(i, n + 1) for_int(j, n + 1) assertx(pairs.contains(std::pair(i, j)) == (i < n && j < n));
  }
  {
    Set<std::tuple<int, string>> set;
    assertx(set.add(std::tuple(1, "one")) && set.add(std::tuple(1, "uno")) && !set.add(std::tuple(1, "one")));
    assertx(set.contains(std::tuple(1, "uno")) && !set.contains(std::tuple(2, "one")) && set.num() == 2);
  }
}

template struct std::hash<std::tuple<int, float, bool>>;
template struct std::hash<std::pair<int, string>>;
