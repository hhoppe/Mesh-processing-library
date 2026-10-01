// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Set.h"

#include <set>

#include "libHh/Advanced.h"  // my_hash()
#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // compare()
using namespace hh;

template <> struct std::hash<hh::Vector> {
  size_t operator()(const hh::Vector& p) const { return hh::my_hash(p[0]); }
};
template <> struct std::equal_to<hh::Vector> {
  bool operator()(const hh::Vector& p1, const hh::Vector& p2) const { return std::is_eq(hh::compare(p1, p2, 1e-4f)); }
};

namespace {

// Random operations on a Set, compared with a std::set as a reference model.
void test_random_operations() {
  Random random(1);
  Set<int> s;
  std::set<int> model;
  const auto verify_contents = [&] { assertx(ranges::equal(sort(Array<int>(s)), model)); };
  for_int(iter, 20'000) {
    const int e = 1 + int(random.get_unsigned(iter < 10'000 ? 200 : 20));  // Nonzero, unlike the default T{}.
    const bool present = model.contains(e);
    switch (random.get_unsigned(8)) {
      case 0:
        if (!present) {
          s.enter(e);
          model.insert(e);
        }
        break;
      case 1: {  // The variant with is_new returns a reference to the element in the set.
        bool is_new;
        const int& e2 = s.enter(e, is_new);
        assertx(is_new == !present && e2 == e && &e2 == s.find_ptr(e));
        model.insert(e);
        break;
      }
      case 2:
        assertx(s.add(e) == !present);
        model.insert(e);
        break;
      case 3:
        assertx(s.remove(e) == present);
        model.erase(e);
        break;
      case 4:
        if (!model.empty()) {
          const int e2 = random.get_unsigned(2) ? s.remove_one() : s.remove_random(random);
          assertx(model.erase(e2) == 1);
        }
        break;
      case 5:
        if (!model.empty()) assertx(model.contains(s.get_one()) && model.contains(s.get_random(random)));
        break;
      default:
        assertx((s.find_ptr(e) != nullptr) == present);
        assertx(s.retrieve(e) == (present ? e : 0));  // If absent, it returns the default T{}.
        if (present) assertx(s.get(e) == e);
    }
    assertx(s.num() == narrow_cast<int>(model.size()) && s.size() == model.size() && s.empty() == model.empty());
    assertx(s.contains(e) == model.contains(e));
    if (iter % 1000 == 0) verify_contents();
  }
  verify_contents();
}

void test_merge() {
  Set<int> s1{1, 2, 3, 4};
  Set<int> s2{3, 4, 5, 6};
  s1.merge(s2);  // The elements 5 and 6 are moved from s2; the duplicates 3 and 4 remain in s2.
  SHOW(sort(Array<int>(s1)));
  SHOW(sort(Array<int>(s2)));
  Set<int> s3;
  s3.merge(s2);
  assertx(s2.empty() && s3.num() == 2 && s3.contains(3) && s3.contains(4));
  s3.clear();
  assertx(s3.empty() && !s3.contains(3) && s3.begin() == s3.end());
}

void test_move_only() {
  using U = unique_ptr<int>;
  Set<U> s;
  for_int(i, 10) s.enter(make_unique<int>(i));
  Random random(2);
  int sum_removed = 0;
  while (s.num() > 5) {
    const U u = s.remove_random(random);  // The element is moved out of the set.
    sum_removed += *u;
  }
  int sum_remaining = 0;
  for (const U& u : s) sum_remaining += *u;
  assertx(sum_removed + sum_remaining == 45);
  const int* p = s.get_one().get();
  const U u = s.remove_one();
  assertx(u.get() == p && s.num() == 4);
}

void test_stateful_functors() {  // Hash and equality functors with state, passed to the constructor.
  struct HashMod {
    int modulus;
    size_t operator()(int i) const { return size_t(i % modulus); }
  };
  struct EqualMod {
    int modulus;
    bool operator()(int i1, int i2) const { return i1 % modulus == i2 % modulus; }
  };
  Set<int, HashMod, EqualMod> s(HashMod{10}, EqualMod{10});
  assertx(s.add(3) && !s.add(13) && s.add(14));
  assertx(s.num() == 2 && s.contains(23) && s.get(33) == 3 && s.retrieve(5) == 0);
  bool is_new;
  assertx(s.enter(24, is_new) == 14 && !is_new);
  assertx(s.remove(103) && !s.remove(3) && s.num() == 1 && s.get_one() == 14);
}

}  // namespace

int main() {
  {
    const Set<string> set = {"first", "second"};
    assertx(set.contains("second"));
    assertx(!set.contains("third"));
  }
  {
    const auto func_get = [](const Set<Vector>& hs, const Vector& p) {
      SHOW("");
      SHOW(p);
      const Vector* po = hs.find_ptr(p);
      SHOW(po != nullptr);
      if (po) SHOW(*po);
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
      bool operator()(const Point& p1, const Point& p2) const { return std::is_eq(compare(p1, p2, 1e-4f)); }
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
    assertx(!setpoints.add(Point(1.f, 2.f, 3.000001f)));  // Same because hash only considers x coordinate.
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
    assertx(setpoints.add(Point(1.f, 2.f, 3.000001f)));  // Hash considers all coordinates.
  }
  {
    Set<int> s;
    assertx(s.num() == 0);
    assertx(s.begin() == s.end());
    for_int(i, 50) s.enter(i);
    for_intL(i, 50, 100) assertx(s.add(i));
    assertx(s.num() == 100);
    for_int(i, 100) assertx(!s.add(i));
    assertx(s.num() == 100);
    assertx(s.contains(2));
    assertx(!s.contains(100));
    int se = 0;
    for (const int i : s) se += i;
    assertx(se == (0 + 99) * (100 / 2));
    assertx(!s.remove(101));
    for_int(i, 50) assertx(s.remove(i));
    assertx(s.num() == 50);
    se = int(sum(s));
    assertx(se == (50 + 99) * (50 / 2));
    se = 0;
    while (s.num()) se += s.remove_one();
    assertx(se == (50 + 99) * (50 / 2));
  }
  {
    Set<int> s;
    for_int(i, 100) s.enter(i);
    Set<int> s2;
    for_int(i, 10'000) {
      const int e = s.get_random(Random::G);
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
    assertx(ar1 == ar1);
    assertx(ar1 == ar2);
    assertx(ar2 == ar1);
    assertx(ar1 != ar3);
  }
  {
    using U = unique_ptr<int>;
    std::unordered_set<U> s;
    s.insert(make_unique<int>(31));
    s.insert(make_unique<int>(37));
    auto it = s.begin();
    // The type of *it is a const T& due to the container's requirement to maintain the uniqueness and ordering of
    // elements based on their hash values, so the following cannot work:
    //  U u = std::move(*it);
    //  s.erase(it);
    // We must use extract() instead:
    auto node = s.extract(it);
    const U u = std::move(node.value());
    assertx(*u == 31 || *u == 37);
    SHOW(s.size());
  }
  {
    using U = unique_ptr<int>;
    Set<U> s;
    s.enter(make_unique<int>(31));
    s.enter(make_unique<int>(37));
    s.enter(make_unique<int>(43));
    Array<int> ar;
    while (!s.empty()) ar.push(*s.remove_one());
    sort(ar);
    SHOW(ar);
  }
  {
    Set set(V(4, 1, 4, 5, 4, 1));
    SHOW(sort(Array(set)));
  }
  {
    InlinedSet<int, 4> set;
    SHOW(set.num(), set.empty());
    for (const int e : {3, 1, 4, 1, 5, 9, 2, 6, 5, 3}) {  // The fifth distinct element moves all into the hash Set.
      const bool is_new = set.add(e);
      SHOW(e, is_new, set.num());
    }
    SHOW(set.contains(9), set.contains(7));
    set.clear();  // Returns to the built-in storage.
    SHOW(set.num(), set.empty(), set.contains(3));
    set.enter(7);
    SHOW(set.num(), set.contains(7), set.contains(3));
  }
  {
    InlinedSet<string, 3> set;
    for (const string s : {"a", "b", "a", "c", "d", "b", "e"}) {
      const bool is_new = set.add(s);
      SHOW(s, is_new, set.num());
    }
  }
  {  // Compare with Set, both before and after the elements move into the hash Set.
    InlinedSet<int, 8> iset;
    Set<int> set;
    for_int(i, 1000) {
      const int e = (i * 37) % 23;
      assertx(iset.add(e) == set.add(e));
      assertx(iset.num() == set.num());
      for_int(j, 30) assertx(iset.contains(j) == set.contains(j));
    }
    SHOW(iset.num());
  }
  test_random_operations();
  test_merge();
  test_move_only();
  test_stateful_functors();
}

template class hh::Set<unsigned>;
template class hh::Set<const int*>;
template class hh::Set<Vector>;
template class hh::Set<unique_ptr<int>>;
template class hh::InlinedSet<unsigned, 4>;
template class hh::InlinedSet<const int*, 2>;
