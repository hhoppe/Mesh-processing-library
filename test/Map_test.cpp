// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Map.h"

#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/HashTuple.h"
#include "libHh/PArray.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // sort()
#include "libHh/Set.h"
using namespace hh;

int main() {
  if (0) {  // timing test
    Map<int, int> m;
    SHOW(m.num());
    for_int(i, 1'000'000) m.enter(i, 1);  // now this is somewhat slow (4.5sec) in Debug under VC2012!
    SHOW("after end");
    m.clear();  // slow with _ITERATOR_DEBUG_LEVEL == 2 (in Debug) under VC2010!
    SHOW("after clear");
  }
  {
    const Map<string, int> map = {{"first", 1}, {"second", 2}};
    assertx(map.get("second") == 2);
  }
  {
    Map<int, int> m;
    assertx(m.num() == 0);
    for (int i : m.keys()) {
      (void(i));
      if (1) assertnever("");
    }
    for_int(i, 100) m.enter(i, i * 8);
    assertw(m.num() == 100);
    m.enter(998, 999);
    assertw(m.contains(998));
    assertw(!m.contains(999));
    assertw(m.retrieve(998) == 999);
    assertw(m.get(998) == 999);
    assertw(m.remove(998) == 999);
    assertw(!m.contains(998));
    assertw(m.retrieve(2) == 2 * 8);
    for (auto& [k, v] : m) assertw(k * 8 == v);
    int sk = 0, sv = 0;
    for (auto& [k, v] : m) {
      sk += k;
      sv += v;
    }
    assertw(sk == (0 + 99) * (100 / 2));
    assertw(sv == (0 + 99 * 8) * (100 / 2));
    assertw(!m.contains(100));
    assertw(m.retrieve(44) == 44 * 8);
    for_int(i, 50) assertw(m.remove(i) == i * 8);
    assertw(m.num() == 50);
    sk = 0;
    sv = 0;
    for (auto& [k, v] : m) {
      sk += k;
      sv += v;
    }
    assertw(sk == (50 + 99) * (50 / 2));
    assertw(sv == (50 * 8 + 99 * 8) * (50 / 2));
    sk = 0;
    sv = 0;
    for (int k : m.keys()) sk += k;
    for (int v : m.values()) sv += v;
    assertw(sk == (50 + 99) * (50 / 2));
    assertw(sv == (50 * 8 + 99 * 8) * (50 / 2));
    for_intL(i, 50, 100) m.remove(i);
    m.clear();
    assertx(m.empty());
    {
      int num = 10000;
      for_int(i, num) m.enter(i, 0);
      for_int(i, num) m.remove(i);
      assertx(m.num() == 0);
    }

    m.clear();
    for_int(i, 100) m.enter(i, i);
    for_int(i, 100) {
      int val = m.get_random_value(Random::G);
      int key = val;
      assertx(m.contains(key));
      assertx(m.remove(key) == val);
    }
    assertx(m.empty());
  }
  {
    using TU = std::tuple<bool, unsigned>;
    Map<TU, int> m;
    m.enter(TU(true, 7), 3);
  }
  {
    using TU = std::tuple<float, float>;
    Map<TU, int> m;
    m.enter(std::tuple(2.f, 2.f), 3);
    m.enter(std::tuple(2.f, 3.f), 4);
    m.enter(std::tuple(3.f, 3.f), 5);
    SHOW(m.get(std::tuple(2.f, 3.f)));
    // for (const TU& tu : m.keys()) SHOW(tu);
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
    SHOW(sorted(Array(map.values())));
    Array ar_tuple(sort(Array(map.keys())) | views::enumerate);
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
    SHOW(sort(Array(map.keys() | views::transform([](string s) { return "<" + s + ">"; }))));
    SHOW(sort(PArray<int, 1>(map.values() | views::transform([](int i) { return 100 + i; }))));
  }
  {
    static_assert(ranges::view<Map<int, int>::keys_range>);
    static_assert(ranges::view<Map<int, int>::values_range>);
    static_assert(ranges::view<Map<int, int>::cvalues_range>);
  }
}

template class hh::Map<int, unsigned>;
template class hh::Map<Point, int, std::hash<Vec3<float>>>;
template class hh::Map<string, string>;
template class hh::Map<void*, unique_ptr<int>>;
template class hh::Map<unique_ptr<int>, double*>;
template class hh::Map<unique_ptr<int>, unique_ptr<int>>;
