// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Sac.h"

#include "Geometry.h"
using namespace hh;

namespace {

class A {
 public:
  A() = default;
  HH_MAKE_SAC(A);  // must be last entry of class!
};

class B {
 public:
  B() {
    SHOW("B allocation");
    dummy = 1;
  }
  ~B() {
    SHOW("B deallocation");
    SHOW(dummy);
  }
  int dummy;
};
HH_SACABLE(B);

class A2 {
 public:
  A2() = default;
  HH_MAKE_POOLED_SAC(A2);  // must be last entry of class!
};
HH_SAC_ALLOCATE_FUNC(A2, Point, point);
HH_ALLOCATE_POOL(A2);
HH_INITIALIZE_POOL(A2);

}  // namespace

int main() {
  {
    int key_p = HH_SAC_ALLOCATE(A, Point);
    int key_b = HH_SAC_ALLOCATE_CD(A, B);
    int key_i = HH_SAC_ALLOCATE(A, int);
    SHOW(key_p);
    SHOW(key_b);
    auto a = make_unique<A>();
    sac_access<Point>(a, key_p) = Point(1.f, 2.f, 3.f);
    SHOW(sac_access<Point>(a, key_p));
    SHOW(sac_access<B>(a, key_b).dummy);
    sac_access<B>(a, key_b).dummy = 2;
    sac_access<int>(a, key_i) = 3;
    SHOW(sac_access<int>(a, key_i));
  }
  {
    auto a2 = make_unique<A2>();
    point(a2.get()) = Point(1.f, 2.f, 3.f);
  }
}
