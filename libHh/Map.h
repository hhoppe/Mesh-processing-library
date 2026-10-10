// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MAP_H_
#define MESH_PROCESSING_LIBHH_MAP_H_

#include <unordered_map>

#include "libHh/Random.h"

#if 0
{
  Map<Edge, Vertex> mev;  // Default uses std::hash<Key>and std::equal_to<Key> (which tries operator ==).
  for (auto& [e, v] : mev) func(e, v);
  for (Edge e : mev.keys()) func(e);
  for (Vertex v : mev.values()) func(v);

  struct mypair {
    unsigned _v1, _v2;
  };
  struct hash_mypair {
    size_t operator()(const mypair& e) const { return e._v1 + e._v2 * 761; }
  };
  struct equal_mypair {
    bool operator()(const mypair& e1, const mypair& e2) const { return e1._v1 == e2._v1 && e1._v2 == e2._v2; }
  };
  Map<mypair, int, hash_mypair, equal_mypair> map;
  // See Set.h for other examples using std::hash<> and explicit hash functional constructor.
}
#endif

namespace hh {

// Map is very similar to std::unordered_map but using my own accessor functions.
// (The typename Equal also goes by the name Pred in the C++ standard library)
template <typename Key, typename Value, typename Hash = std::hash<Key>, typename Equal = std::equal_to<Key>>
class Map {
  static_assert(Hashable<Key, Hash, Equal>);
  using type = Map<Key, Value, Hash, Equal>;
  using base = std::unordered_map<Key, Value, Hash, Equal>;
  using value_type = base::value_type;
  using biter = base::iterator;
  using bciter = base::const_iterator;

 public:
  // Views of the keys and values; each holds a reference to the map, like the earlier hand-written ranges.
  using keys_range = decltype(views::keys(std::declval<const base&>()));
  using keys_iterator = ranges::iterator_t<keys_range>;
  using values_range = decltype(views::values(std::declval<base&>()));
  using values_iterator = ranges::iterator_t<values_range>;
  using cvalues_range = decltype(views::values(std::declval<const base&>()));
  using cvalues_iterator = ranges::iterator_t<cvalues_range>;
  using Hashf = base::hasher;
  using Equalf = base::key_equal;
  Map() = default;
  explicit Map(Hashf hashf) : _map(0, hashf) {}
  explicit Map(Hashf hashf, Equalf equalf) : _map(0, hashf, equalf) {}
  Map(std::initializer_list<std::pair<const Key, Value>> l) requires Copyable<Key> && Copyable<Value>
      : _map(std::move(l)) {}
  void clear() { _map.clear(); }
  void enter(const Key& key, const Value& value) requires Copyable<Key> && Copyable<Value> {  // Key must be new!
    const auto [_, is_new] = _map.emplace(key, value);
    ASSERTX(is_new);
  }
  void enter(Key&& key, const Value& value) requires Copyable<Value> {
    const auto [_, is_new] = _map.emplace(std::move(key), value);
    ASSERTX(is_new);
  }
  void enter(const Key& key, Value&& value) requires Copyable<Key> {
    const auto [_, is_new] = _map.emplace(key, std::move(value));
    ASSERTX(is_new);
  }
  void enter(Key&& key, Value&& value) {
    const auto [_, is_new] = _map.emplace(std::move(key), std::move(value));
    ASSERTX(is_new);
  }
  Value& enter(const Key& key, const Value& value, bool& is_new) requires Copyable<Key> && Copyable<Value>
  {  // Does not modify element if it already exists.
    const auto [it, is_new_] = _map.emplace(key, value);
    is_new = is_new_;
    return it->second;
  }
  // Omit "Value& enter(const Key& key, Value&& value, bool& is_new)" because value could be lost if !is_new.
  // Note: force_enter using: { map[key] = std::move(value); }.
  [[nodiscard]] bool contains(const Key& key) const { return _map.find(key) != end(); }
  [[nodiscard]] const Value& retrieve(const Key& key) const {
    auto it = _map.find(key);
    if (it == end()) return def();
    return it->second;
  }
  [[nodiscard]] auto& get(this auto&& self, const Key& key) {
    auto it = self._map.find(key);
    ASSERTXX(it != self.end());
    return it->second;
  }
  // const Value& get(const Key& key) const { return (*this)[key]; } // Bad: throws exception if absent.
  [[nodiscard]] auto* find_ptr(this auto&& self, const Key& key) {  // Returns nullptr if key is absent.
    auto it = self._map.find(key);
    return it != self.end() ? &it->second : nullptr;
  }
  Value remove(const Key& key) { return remove_i(key); }
  Value replace(const Key& key, const Value& value) requires Copyable<Key> && Copyable<Value> {
    auto it = _map.find(key);
    if (it == end()) return Value();
    Value vo = it->second;
    it->second = value;
    return vo;
  }
  // Omit "Value replace(const Key& key, Value&& value)" because value could be lost if !present.
  [[nodiscard]] int num() const { return narrow_cast<int>(_map.size()); }
  [[nodiscard]] size_t size() const { return _map.size(); }
  [[nodiscard]] bool empty() const { return _map.empty(); }
  // Introduced for Combination:
  [[nodiscard]] Value& operator[](const Key& key) requires Copyable<Key> && Copyable<Value> { return _map[key]; }
  [[nodiscard]] const Value& operator[](const Key& key) const requires Copyable<Value> {
    auto it = _map.find(key);
    return it != end() ? it->second : def();
  }
  [[nodiscard]] const Key& get_one_key() const { return ASSERTXX(!empty()), begin()->first; }
  [[nodiscard]] const Value& get_one_value() const { return ASSERTXX(!empty()), begin()->second; }
  [[nodiscard]] const Key& get_random_key(Random& random) const { return crand(random)->first; }
  [[nodiscard]] const Value& get_random_value(Random& random) const { return crand(random)->second; }
  [[nodiscard]] keys_range keys() const { return views::keys(_map); }  // Keys are always constant.
  [[nodiscard]] auto values(this auto&& self) { return views::values(self._map); }
  [[nodiscard]] cvalues_range cvalues() const { return views::values(_map); }
  // For "for (auto& [key, value] : map)" and HH_DECLARE_OSTREAM_RANGE(Map<Key, Value>):
  [[nodiscard]] bciter begin() const { return _map.begin(); }
  [[nodiscard]] bciter end() const { return _map.end(); }

 private:
  // See my experiments in ~/git/hh_src/test/misc/test_hash_buckets.cpp.
  base _map;
  static const Value& def() {
    static const Value key_default = Value();
    return key_default;
  }
  Value remove_i(const Key& key) {
    auto it = _map.find(key);
    if (it == end()) return Value();
    auto value = std::move(it->second);
    _map.erase(it);
    // Shrink the table, which also bounds the expected number of attempts in crand().
    if (_map.size() < _map.bucket_count() / 16) _map.rehash(0);
    return value;
  }
  bciter crand(Random& random) const {  // See also similar code in Set.
    assertx(!empty());
    // Rejection sampling: uniformly draw (1) a bucket and (2) a position in [0, _crand_bound), and retry if the
    // bucket has no element at that position.  Every element is then equally likely, provided that _crand_bound is
    // at least the size of every bucket.  (Walking from a random bucket to the next element instead favors the
    // elements that follow empty buckets, whose number grows as random elements are removed.)  The bound is
    // recomputed after each rehash, and a bucket that has since grown beyond it raises it when drawn, discarding
    // that attempt.
    const size_t nbuckets = _map.bucket_count();
    if (nbuckets != _crand_nbuckets) {
      _crand_nbuckets = nbuckets;
      _crand_bound = 1;
      for (const size_t bn : range(nbuckets)) _crand_bound = max(_crand_bound, _map.bucket_size(bn));
      // A large bound indicates a poor hash function, which slows every lookup and proportionally increases the
      // expected number of rejection samples.  With good hashing, _crand_bound stays below about 12 even with 1e8
      // elements.
      assertw(_crand_bound <= 32);
    }
    for (;;) {
      const size_t bn = random.get_size_t() % nbuckets;
      const size_t ne = _map.bucket_size(bn);
      if (ne > _crand_bound) {
        _crand_bound = ne;
        assertw(_crand_bound <= 32);
        continue;
      }
      const size_t i = random.get_size_t() % _crand_bound;
      if (i >= ne) continue;
      auto li = _map.begin(bn);
      std::advance(li, i);
      return _map.find(li->first);  // Convert from const_local_iterator to const_iterator.
    }
  }
  mutable size_t _crand_nbuckets{0};  // Bucket count when _crand_bound was computed.
  mutable size_t _crand_bound{0};     // Upper bound on the bucket sizes, for crand().
  // Default operator=() and copy_constructor are safe.
};

template <typename Key, typename Value> HH_DECLARE_OSTREAM_RANGE(Map<Key, Value>);
template <typename Key, typename Value> HH_DECLARE_OSTREAM_EOL(Map<Key, Value>);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MAP_H_
