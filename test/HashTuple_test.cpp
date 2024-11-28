// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/HashTuple.h"

#include "libHh/Set.h"
using namespace hh;

int main() {
  {
    using TU = std::tuple<int, float, bool>;
    TU tu1 = std::tuple(1, 2.f, true);
    SHOW(tu1);
    if (0) {
      SHOW(my_hash(tu1));
      SHOW(my_hash(std::tuple(1, 2.f, true)));
      SHOW(my_hash(std::tuple(1, 2.f, false)));
      SHOW(my_hash(std::tuple(1, 3.f, true)));
      SHOW(my_hash(std::tuple(2, 2.f, true)));
    }
    Set<size_t> set;  // verify all unique
    assertx(set.add(my_hash(std::tuple(1, 2.f, true))));
    assertx(set.add(my_hash(std::tuple(1, 2.f, false))));
    assertx(set.add(my_hash(std::tuple(1, 3.f, true))));
    assertx(set.add(my_hash(std::tuple(2, 2.f, true))));
  }
  {
    using TU = std::tuple<int, float, double*>;
    double d1, d2;
    TU tu1 = std::tuple(1, 2.f, &d1);
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
}
