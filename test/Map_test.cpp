// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Map.h"

#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/HashTuple.h"
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
      // This is very slow (12 seconds) on VC 2010 x64 debug, for num=10000
      // Even for num=1000, it requires 0.28 seconds.
      int num = 1000;
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
}

namespace hh {

template class Map<int, int>;
template class Map<Point, int, std::hash<Vec3<float>>>;
template class Map<string, string>;

using U = unique_ptr<int>;
// Override illegal definitions for U:
template <> Map<int, U>::Map(std::initializer_list<std::pair<const int, U>>) {}
template <> U Map<int, U>::replace(const int&, const U&) { return U(); }
template <> void Map<int, U>::enter(const int&, const U&) {}
template <> void Map<int, U>::enter(int&&, const U&) {}
template <> U& Map<int, U>::enter(const int&, const U&, bool&) {
  static U u;
  return u;
}
template class Map<int, U>;

}  // namespace hh
