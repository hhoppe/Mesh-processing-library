// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Sac.h"

#include "libHh/Geometry.h"
using namespace hh;

namespace {

class A {
 public:
  A() = default;
  HH_MAKE_SAC(A);  // Must be the last entry of the class!
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
  HH_MAKE_POOLED_SAC(A2);  // Must be the last entry of the class!
};
HH_SAC_ALLOCATE_FUNC(A2, Point, point);
HH_ALLOCATE_POOL(A2);
HH_INITIALIZE_POOL(A2);

int g_num_live = 0;

struct Counted {
  Counted() { g_num_live++; }
  ~Counted() { g_num_live--; }
  int value{7};
};
HH_SACABLE(Counted);

struct alignas(16) Aligned16 {
  float v[4];
};

class C {
 public:
  C() = default;
  HH_MAKE_SAC(C);  // Must be the last entry of the class!
};
HH_SAC_ALLOCATE_CD_FUNC(C, Counted, c_counted);
HH_SAC_ALLOCATE_FUNC(C, Aligned16, c_aligned);
HH_SAC_ALLOCATE_FUNC(C, char, c_char);

}  // namespace

int main() {
  {
    int key_p = HH_SAC_ALLOCATE(A, Point);
    int key_b = HH_SAC_ALLOCATE_CD(A, B);
    const int key_i = HH_SAC_ALLOCATE(A, int);
    SHOW(key_p);
    SHOW(key_b);
    SHOW(key_i);
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
    auto a2b = make_unique<A2>();
    point(a2b.get()) = Point(4.f, 5.f, 6.f);
    SHOW(point(a2.get()), point(a2b.get()));
  }
  {
    // The fields are laid out in order of allocation, each at its required alignment.
    SHOW(Sac<C>::get_size(), Sac<C>::get_max_align());
    Array<unique_ptr<C>> objects;
    for_int(i, 3) objects.push(make_unique<C>());
    assertx(g_num_live == 3);  // The constructor is called for each object.
    for_int(i, 3) {
      C* c = objects[i].get();
      assertx(c_counted(c).value == 7);
      c_counted(c).value = i;
      c_aligned(c) = Aligned16{{float(i), 0.f, 0.f, 0.f}};
      c_char(c) = char('a' + i);
      assertx(reinterpret_cast<uintptr_t>(&c_aligned(c)) % 16 == 0);
    }
    // The fields of each object are independent.
    for_int(i, 3) {
      C* c = objects[i].get();
      assertx(c_counted(c).value == i && c_aligned(c).v[0] == float(i) && c_char(c) == char('a' + i));
    }
    for (auto& object : objects) object.reset();
    assertx(g_num_live == 0);  // The destructor is called for each object.
  }
}
