// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/InlinedMap.h"

#include <map>
#include <utility>  // as_const()

#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // sort(), sum()
using namespace hh;

namespace {

// Random operations on an InlinedMap, whose entries often exceed its inline capacity, compared with a std::map.
void test_random_operations() {
  Random random(1);
  InlinedMap<int, int, 4> m;
  std::map<int, int> model;
  for_int(iter, 20'000) {
    const int key = int(random.get_unsigned(iter < 10'000 ? 12 : 3));  // Later, a smaller set of keys.
    const int value = 1 + int(random.get_unsigned(1000));              // Nonzero, unlike the default Value().
    const auto it = model.find(key);
    const bool present = it != model.end();
    const int old_value = present ? it->second : 0;
    switch (random.get_unsigned(5)) {
      case 0:
        if (!present) {
          m.enter(key, value);
          model.emplace(key, value);
        }
        break;
      case 1:
        if (present) {
          assertx(m.remove(key) == old_value);
          model.erase(key);
        }
        break;
      case 2:  // The non-const operator[] enters a default Value() if the key is absent.
        m[key] += value;
        model[key] += value;
        break;
      case 3:
        if (present) assertx(std::as_const(m)[key] == old_value && m.get(key) == old_value);
        break;
      default:  // Retrieving an absent key returns the default Value().
        assertx(m.retrieve(key) == old_value);
    }
    assertx(m.num() == narrow_cast<int>(model.size()) && m.empty() == model.empty());
    assertx(m.contains(key) == model.contains(key));
  }
  Array<std::pair<int, int>> ar;
  for (const auto& [key, value] : m) ar.push({key, value});
  sort(ar);
  assertx(ar.num() == narrow_cast<int>(model.size()));
  int i = 0;
  for (const auto& [key, value] : model) assertx(ar[i++] == std::pair(key, value));
  SHOW(m.num());
}

// The iteration follows the order of insertion, except that remove() moves the last entry into the vacated place.
void test_order() {
  InlinedMap<char, int, 8> m;
  for (const char ch : string("abcde")) m[ch] = ch - 'a';
  assertx(m.remove('b') == 1);
  string s;
  for (const auto& [ch, value] : m) s += ch;
  SHOW(s);
  SHOW(sum(m.values()));
  string keys;
  for (const char ch : m.keys()) keys += ch;
  assertx(keys == s);
  for (int& value : m.values()) value *= 10;
  assertx(m.get('e') == 40);
  const InlinedMap<char, int, 8> m2 = m;  // A copy is independent of the original.
  m.clear();
  assertx(m.empty() && !m.contains('a') && m2.num() == 4 && m2.get('d') == 30);
}

}  // namespace

int main() {
  test_random_operations();
  test_order();
}

template class hh::InlinedMap<int, int, 4>;
template class hh::InlinedMap<char, int, 8>;
