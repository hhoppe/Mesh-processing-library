// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Set.h"

#include "Advanced.h"  // my_hash()
#include "Array.h"
#include "Geometry.h"
#include "Random.h"
#include "RangeOp.h"  // compare()
using namespace hh;

namespace std {
template <> struct hash<Vector> {
  size_t operator()(const Vector& p) const { return std::hash<float>()(p[0]); }
};
template <> struct equal_to<Vector> {
  bool operator()(const Vector& p1, const Vector& p2) const { return !compare(p1, p2, 1e-4f); }
};
}  // namespace std

int main() {
  {
    auto func_get = [](const Set<Vector>& hs, const Vector& p) {
      SHOW("");
      SHOW(p);
      bool present;
      const Vector& po = hs.retrieve(p, present);
      SHOW(present);
      if (present) SHOW(po);
    };

    Set<Vector> hs;
    hs.enter(Vector(1.f, 2.f, 3.f));
    hs.enter(Vector(4.f, 5.f, 6.f));
    hs.enter(Vector(1.f, 3.f, 2.f));
    hs.enter(Vector(1.f, 1.f, 5.f));
    hs.enter(Vector(1.f, 1.f, 4.f));
    func_get(hs, Vector(1.f, 3.f, 2.f));
    func_get(hs, Vector(1.f, 3.f, 2.00001f));
    func_get(hs, Vector(1.f, 1.f, 7.f));
    func_get(hs, Vector(1.f, 1.f, 5.f));
    func_get(hs, Vector(4.f, 5.f, 8.f));
    func_get(hs, Vector(4.f, 5.f, 6.f));
  }
  {
    struct hash_Point {
      size_t operator()(const Point& p) const { return my_hash(p[0]); }
    };
    struct equal_Point {
      bool operator()(const Point& p1, const Point& p2) const { return !compare(p1, p2, 1e-4f); }
    };
    Set<Point, hash_Point, equal_Point> setpoints;
    assertx(setpoints.add(Point(1.f, 2.f, 3.f)));
    assertx(setpoints.add(Point(4.f, 5.f, 6.f)));
    assertx(!setpoints.add(Point(1.f, 2.f, 3.f)));
    assertx(!setpoints.add(Point(4.f, 5.f, 6.f)));
    assertx(setpoints.contains(Point(1.f, 2.f, 3.f)));
    assertx(setpoints.contains(Point(4.f, 5.f, 6.f)));
    assertx(!setpoints.contains(Point(7.f, 8.f, 9.f)));
    assertx(setpoints.remove(Point(4.f, 5.f, 6.f)));
    assertx(!setpoints.remove(Point(7.f, 8.f, 9.f)));
    assertx(!setpoints.remove(Point(4.f, 5.f, 6.f)));
    assertx(setpoints.contains(Point(1.f, 2.f, 3.f)));
    assertx(!setpoints.contains(Point(4.f, 5.f, 6.f)));
    assertx(!setpoints.add(Point(1.f, 2.f, 3.000001f)));  // same because hash only considers x coordinate
  }
  {
    Set<Point, std::hash<Vec3<float>>> setpoints;
    assertx(setpoints.add(Point(1.f, 2.f, 3.f)));
    assertx(setpoints.add(Point(4.f, 5.f, 6.f)));
    assertx(!setpoints.add(Point(1.f, 2.f, 3.f)));
    assertx(!setpoints.add(Point(4.f, 5.f, 6.f)));
    assertx(setpoints.contains(Point(1.f, 2.f, 3.f)));
    assertx(setpoints.contains(Point(4.f, 5.f, 6.f)));
    assertx(!setpoints.contains(Point(7.f, 8.f, 9.f)));
    assertx(setpoints.remove(Point(4.f, 5.f, 6.f)));
    assertx(!setpoints.remove(Point(7.f, 8.f, 9.f)));
    assertx(!setpoints.remove(Point(4.f, 5.f, 6.f)));
    assertx(setpoints.contains(Point(1.f, 2.f, 3.f)));
    assertx(!setpoints.contains(Point(4.f, 5.f, 6.f)));
    assertx(setpoints.add(Point(1.f, 2.f, 3.000001f)));  // hash considers all coordinates
  }
  {
    Set<int> s;
    assertx(s.num() == 0);
    for (int i : s) {
      dummy_use(i);
      if (1) assertnever("");
    }
    for_int(i, 50) s.enter(i);
    for_intL(i, 50, 100) assertx(s.add(i));
    assertw(s.num() == 100);
    for_int(i, 100) assertx(!s.add(i));
    assertw(s.num() == 100);
    assertw(s.contains(2));
    assertw(!s.contains(100));
    int se = 0;
    for (int i : s) {
      se += i;
    }
    assertw(se == (0 + 99) * (100 / 2));
    assertw(!s.remove(101));
    for_int(i, 50) assertw(s.remove(i));
    assertw(s.num() == 50);
    se = sum(s);
    assertw(se == (50 + 99) * (50 / 2));
    se = 0;
    while (s.num()) se += s.remove_one();
    assertw(se == (50 + 99) * (50 / 2));
  }
  {
    Set<int> s;
    for_int(i, 100) s.enter(i);
    Set<int> s2;
    for_int(i, 10000) {
      int e = s.get_random(Random::G);
      s2.add(e);
    }
    assertx(s2.num() == 100);
  }
  {
    Array<int> ar1;
    ar1.push(5);
    Array<int> ar2;
    ar2.push(5);
    Array<int> ar3;
    ar3.push(6);
    assertx(std::equal_to<Array<int>>()(ar1, ar1));
    assertx(std::equal_to<Array<int>>()(ar2, ar2));
    assertx(std::equal_to<Array<int>>()(ar1, ar2));
    assertx(!std::equal_to<Array<int>>()(ar1, ar3));
    assertx(ar1 == ar1);
    assertx(ar1 == ar2);
    assertx(ar2 == ar1);
    assertx(ar1 != ar3);
  }
#if 0  // *i calls U::U(const U&) for unknown reason, on both VS and gcc
  {
    using U = unique_ptr<int>;
    std::unordered_set<U> s;
    s.insert(make_unique<int>(31));
    s.insert(make_unique<int>(37));
    auto i = s.begin();
    dummy_use(i);
    U u = std::move(*i);
  }
  {
    using U = unique_ptr<int>;
    Set<U> s;
    s.enter(make_unique<int>(31));
    s.enter(make_unique<int>(37));
    s.enter(make_unique<int>(43));
    Array<int> ar;
    while (!s.empty()) {
      ar += *s.remove_one();
    }
    ar.sort();
    SHOW(ar);
  }
#endif
}

template class hh::Set<unsigned>;
template class hh::Set<const int*>;
template class hh::Set<Vector>;
// full instantiation of Set not possible for unique_ptr<T>
