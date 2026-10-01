// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Map.h"

#include <map>

#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/HashTuple.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // sort()
#include "libHh/Set.h"
using namespace hh;

namespace {

// Random operations on a Map, compared with a std::map as a reference model.
void test_random_operations() {
  Random random(1);
  Map<int, int> m;
  std::map<int, int> model;
  const auto verify_contents = [&] {
    Array<std::pair<int, int>> ar;
    for (const auto& [key, value] : m) ar.push({key, value});
    sort(ar);
    assertx(ar.num() == narrow_cast<int>(model.size()));
    int i = 0;
    for (const auto& [key, value] : model) assertx(ar[i++] == std::pair(key, value));
  };
  for_int(iter, 20'000) {
    const int key = int(random.get_unsigned(iter < 10'000 ? 200 : 20));  // Later, a smaller set of keys.
    const int value = 1 + int(random.get_unsigned(1000));                // Nonzero, unlike the default Value().
    const auto it = model.find(key);
    const bool present = it != model.end();
    const int old_value = present ? it->second : 0;
    switch (random.get_unsigned(8)) {
      case 0:
        if (!present) {
          m.enter(key, value);
          model.emplace(key, value);
        }
        break;
      case 1: {  // The variant with is_new does not modify an existing element.
        bool is_new;
        const int& v = m.enter(key, value, is_new);
        assertx(is_new == !present && v == (present ? old_value : value));
        if (!present) model.emplace(key, value);
        break;
      }
      case 2:  // Removing an absent key returns the default Value().
        assertx(m.remove(key) == old_value);
        model.erase(key);
        break;
      case 3:  // Replacing an absent key returns the default Value() and does not enter it.
        assertx(m.replace(key, value) == old_value);
        if (present) it->second = value;
        break;
      case 4: {
        int* p = m.find_ptr(key);
        assertx((p != nullptr) == present);
        if (p) *p = value, it->second = value;
        break;
      }
      case 5:  // The non-const operator[] enters a default Value() if the key is absent.
        m[key] += value;
        model[key] += value;
        break;
      case 6: {  // The const operator[] does not enter the key.
        const Map<int, int>& cm = m;
        assertx(cm[key] == old_value);
        assertx(std::as_const(m).find_ptr(key) == (present ? &m.get(key) : nullptr));
        break;
      }
      default:
        assertx(m.retrieve(key) == old_value);
        if (present) assertx(m.get(key) == old_value);
    }
    assertx(m.num() == narrow_cast<int>(model.size()) && m.size() == model.size() && m.empty() == model.empty());
    assertx(m.contains(key) == model.contains(key));
    if (iter % 1000 == 0) verify_contents();
  }
  verify_contents();
  SHOW(m.num());
}

void test_accessors() {
  Map<int, int> m;
  for_int(i, 10) m.enter(i, i * i);
  {
    const int key = m.get_one_key();
    assertx(m.get(key) == key * key && m.get_one_value() == key * key);
  }
  {  // Each key is eventually returned by get_random_key(), and each value by get_random_value().
    Random random(2);
    Set<int> keys, values;
    for_int(i, 2000) keys.add(m.get_random_key(random)), values.add(m.get_random_value(random));
    assertx(keys.num() == 10 && values.num() == 10);
  }
  {  // The values() view allows modification of the values.
    for (int& v : m.values()) v += 1;
    for_int(i, 10) assertx(m.get(i) == i * i + 1);
    assertx(sum(m.cvalues()) == 285 + 10);
    assertx(sum(m.keys()) == 45);
  }
  {  // A copy is independent of the original.
    Map<int, int> m2 = m;
    m2.get(3) = -1;
    assertx(m2.remove(4) == 17);
    assertx(m.get(3) == 10 && m.get(4) == 17 && m2.num() == 9 && m.num() == 10);
  }
  m.clear();
  assertx(m.empty() && m.keys().empty() && !m.contains(0));
}

void test_move_only_values() {
  Map<int, unique_ptr<int>> m;
  m.enter(1, make_unique<int>(10));
  m.enter(2, make_unique<int>(20));
  *m.get(2) += 1;
  assertx(*m.get(1) == 10 && **m.find_ptr(2) == 21 && !m.find_ptr(3));
  assertx(!m.retrieve(3));
  const unique_ptr<int> u = m.remove(1);  // The value is moved out of the map.
  assertx(*u == 10 && !m.contains(1) && m.num() == 1);
  assertx(!m.remove(1));  // Removing an absent key returns a default (null) value.
  for (unique_ptr<int>& p : m.values()) p = make_unique<int>(*p + 1);
  assertx(*m.get(2) == 22);
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
  Map<int, int, HashMod, EqualMod> m(HashMod{10}, EqualMod{10});
  m.enter(3, 1);
  m.enter(14, 2);
  assertx(m.contains(13) && m.get(23) == 1 && m.retrieve(4) == 2 && !m.contains(5));
  bool is_new;
  assertx(m.enter(33, 7, is_new) == 1 && !is_new);
  assertx(m.remove(103) == 1 && m.num() == 1 && m.get_one_key() == 14);
}

}  // namespace

int main() {
  if (0) {  // Timing test.
    Map<int, int> m;
    SHOW(m.num());
    for_int(i, 1'000'000) m.enter(i, 1);  // Now this is somewhat slow (4.5sec) in Debug under VC2012!
    SHOW("after end");
    m.clear();  // Slow with _ITERATOR_DEBUG_LEVEL == 2 (in Debug) under VC2010!
    SHOW("after clear");
  }
  {
    const Map<string, int> map = {{"first", 1}, {"second", 2}};
    assertx(map.get("second") == 2);
  }
  {
    Map<int, int> m;
    assertx(m.num() == 0);
    assertx(m.keys().begin() == m.keys().end());
    for_int(i, 100) m.enter(i, i * 8);
    assertx(m.num() == 100);
    m.enter(998, 999);
    assertx(m.contains(998));
    assertx(!m.contains(999));
    assertx(m.retrieve(998) == 999);
    assertx(m.get(998) == 999);
    assertx(m.remove(998) == 999);
    assertx(!m.contains(998));
    assertx(m.retrieve(2) == 2 * 8);
    for (const auto& [k, v] : m) assertx(k * 8 == v);
    int sk = 0, sv = 0;
    for (const auto& [k, v] : m) {
      sk += k;
      sv += v;
    }
    assertx(sk == (0 + 99) * (100 / 2));
    assertx(sv == (0 + 99 * 8) * (100 / 2));
    assertx(!m.contains(100));
    assertx(m.retrieve(44) == 44 * 8);
    for_int(i, 50) assertx(m.remove(i) == i * 8);
    assertx(m.num() == 50);
    sk = 0;
    sv = 0;
    for (const auto& [k, v] : m) {
      sk += k;
      sv += v;
    }
    assertx(sk == (50 + 99) * (50 / 2));
    assertx(sv == (50 * 8 + 99 * 8) * (50 / 2));
    sk = 0;
    sv = 0;
    for (const int k : m.keys()) sk += k;
    for (const int v : m.values()) sv += v;
    assertx(sk == (50 + 99) * (50 / 2));
    assertx(sv == (50 * 8 + 99 * 8) * (50 / 2));
    for_intL(i, 50, 100) m.remove(i);
    m.clear();
    assertx(m.empty());
    {
      const int num = 10000;
      for_int(i, num) m.enter(i, 0);
      for_int(i, num) m.remove(i);
      assertx(m.num() == 0);
    }

    m.clear();
    for_int(i, 100) m.enter(i, i);
    for_int(i, 100) {
      const int val = m.get_random_value(Random::G);
      const int key = val;
      assertx(m.contains(key));
      assertx(m.remove(key) == val);
    }
    assertx(m.empty());
  }
  {
    using TU = std::tuple<bool, unsigned>;
    Map<TU, int> m;
    m.enter(TU(true, 7), 3);
    assertx(m.get(TU(true, 7)) == 3 && !m.contains(TU(false, 7)) && !m.contains(TU(true, 8)));
  }
  {
    using TU = std::tuple<float, float>;
    Map<TU, int> m;
    m.enter(std::tuple(2.f, 2.f), 3);
    m.enter(std::tuple(2.f, 3.f), 4);
    m.enter(std::tuple(3.f, 3.f), 5);
    SHOW(m.get(std::tuple(2.f, 3.f)));
    for (const TU& tu : sort(Array<TU>(m.keys()))) SHOW(tu);  // Sorted, as the hash map order is unspecified.
    SHOW(sum(m.values()));
  }
  {
    Map<Point, int, std::hash<Vec3<float>>> m;
    m.enter(Point(1.f, 2.f, 3.f), 5);
    m.enter(Point(4.f, 5.f, 6.f), 6);
    m.enter(Point(1.f, 2.f, 7.f), 7);
    m.enter(Point(2.f, 2.f, 3.f), 8);
    assertx(m.contains(Point(4.f, 5.f, 6.f)));
    assertx(m.get(Point(1.f, 2.f, 7.f)) == 7);
  }
  {
    Map<string, string> m;
    m.enter("abc", "12");
    assertx(!m.contains("ab"));
    assertx(!m.contains("abcd"));
    assertx(m.contains("abc"));
    m.enter("abcd", "13");
    m.enter("ab", "14");
    assertx(m.contains("ab"));
    assertx(m.contains("abcd"));
    assertx(m.contains("abc"));
    assertx(!m.contains("abcde"));
    assertx(m.get("abc") == "12");
    assertx(m.get("ab") == "14");
    assertx(m.get("abcd") == "13");
    assertx(m.retrieve("abcd") == "13");
    assertx(m.retrieve("abcde") == "");
    assertx(m.num() == 3);
    assertx(m.remove("abc") == "12");
    assertx(m.num() == 2);
    assertx(!m.contains("abc"));
    assertx(m.retrieve("abc") == "");
    assertx(m.replace("abcd", "113") == "13");
    assertx(m.get("abcd") == "113");
    assertx(m.get("ab") == "14");
    assertx(m["abcd"] == "113");
    Array<string> ar(m.keys());
    sort(ar);
    for (const string& s : ar) SHOW(s, m[s]);
    assertx(m.remove("ab") == "14");
    SHOW(m);
  }
  {
    const Map<string, int> map = {{"first", 1}, {"second", 2}};
    SHOW(sort(Array(map.values())));
    Array ar_tuple(sort(Array(map.keys())) | enumerate);
    SHOW(ar_tuple[0]);
    SHOW(ar_tuple[1]);
    const auto str = (sort(Array(map.values())) | views::transform([](int v) { return std::to_string(v); }) |
                      views::join_with(',') | ranges::to<string>());
    SHOW(type_name<decltype(str)>());
    SHOW(str);
    SHOW(sort(map.values() | ranges::to<Array<int>>()));
  }
  {
    Map<string, int> map = {{"first", 1}, {"second", 2}};
    SHOW(sort(Array(map.keys() | views::transform([](const string& s) { return "<" + s + ">"; }))));
    SHOW(sort(InlinedArray<int, 1>(map.values() | views::transform([](int i) { return 100 + i; }))));
  }
  {
    static_assert(ranges::view<Map<int, int>::keys_range>);
    static_assert(ranges::view<Map<int, int>::values_range>);
    static_assert(ranges::view<Map<int, int>::cvalues_range>);
  }
  test_random_operations();
  test_accessors();
  test_move_only_values();
  test_stateful_functors();
}

template class hh::Map<int, unsigned>;
template class hh::Map<Point, int, std::hash<Vec3<float>>>;
template class hh::Map<string, string>;
template class hh::Map<void*, unique_ptr<int>>;
template class hh::Map<unique_ptr<int>, double*>;
template class hh::Map<unique_ptr<int>, unique_ptr<int>>;
