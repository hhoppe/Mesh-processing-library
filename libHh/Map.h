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
  using type = Map<Key, Value, Hash, Equal>;
  using base = std::unordered_map<Key, Value, Hash, Equal>;
  using value_type = typename base::value_type;
  using biter = typename base::iterator;
  using bciter = typename base::const_iterator;

 public:
  class keys_range;
  class values_range;
  class cvalues_range;
  using Hashf = typename base::hasher;
  using Equalf = typename base::key_equal;
  Map() = default;
  explicit Map(Hashf hashf) : _map(0, hashf) {}
  explicit Map(Hashf hashf, Equalf equalf) : _map(0, hashf, equalf) {}
  Map(std::initializer_list<std::pair<const Key, Value>> list) : _map(std::move(list)) {}
  void clear() { _map.clear(); }
  void enter(const Key& key, const Value& value) {  // Key must be new!
    auto [_, is_new] = _map.emplace(key, value);
    ASSERTX(is_new);
  }
  void enter(Key&& key, const Value& value) {
    auto [_, is_new] = _map.emplace(std::move(key), value);
    ASSERTX(is_new);
  }
  void enter(const Key& key, Value&& value) {
    auto [_, is_new] = _map.emplace(key, std::move(value));
    ASSERTX(is_new);
  }
  void enter(Key&& key, Value&& value) {
    auto [_, is_new] = _map.emplace(std::move(key), std::move(value));
    ASSERTX(is_new);
  }
  Value& enter(const Key& key, const Value& value, bool& is_new) {  // Does not modify element if it already exists.
    auto [it, is_new_] = _map.emplace(key, value);
    is_new = is_new_;
    return it->second;
  }
  // Omit "Value& enter(const Key& key, Value&& value, bool& is_new)" because value could be lost if !is_new.
  // Note: force_enter using: { map[key] = std::move(value); }.
  bool contains(const Key& key) const { return _map.find(key) != end(); }
  const Value& retrieve(const Key& key, bool& present) const {
    auto it = _map.find(key);
    present = it != end();
    return present ? it->second : def();
  }
  const Value& retrieve(const Key& key) const {
    auto it = _map.find(key);
    if (it == end()) return def();
    return it->second;
  }
  Value& get(const Key& key) {
    auto it = _map.find(key);
    ASSERTXX(it != end());
    return it->second;
  }
  const Value& get(const Key& key) const {
    auto it = _map.find(key);
    ASSERTXX(it != end());
    return it->second;
  }
  // const Value& get(const Key& key) const { return (*this)[key]; } // Bad: throws exception if absent.
  Value remove(const Key& key) { return remove_i(key); }
  Value replace(const Key& key, const Value& value) {
    auto it = _map.find(key);
    if (it == end()) return Value();
    Value vo = it->second;
    it->second = value;
    return vo;
  }
  // Omit "Value replace(const Key& key, Value&& value)" because value could be lost if !present.
  int num() const { return narrow_cast<int>(_map.size()); }
  size_t size() const { return _map.size(); }
  bool empty() const { return _map.empty(); }
  Value& operator[](const Key& key) { return _map[key]; }  // Introduced for Combination.
  const Value& operator[](const Key& key) const {
    auto it = _map.find(key);
    return it != end() ? it->second : def();
  }
  const Key& get_one_key() const { return (ASSERTXX(!empty()), begin()->first); }
  const Value& get_one_value() const { return (ASSERTXX(!empty()), begin()->second); }
  const Key& get_random_key(Random& random) const { return crand(random)->first; }
  const Value& get_random_value(Random& random) const { return crand(random)->second; }
  keys_range keys() const { return keys_range(*this); }  // Keys are always constant.
  values_range values() { return values_range(*this); }
  cvalues_range cvalues() { return cvalues_range(*this); }
  cvalues_range values() const { return cvalues_range(*this); }
  // For "for (auto& [key, value] : map)" and HH_DECLARE_OSTREAM_RANGE(Map<Key, Value>):
  bciter begin() const { return _map.begin(); }
  bciter end() const { return _map.end(); }

 public:
  class keys_iterator {
    using type = keys_iterator;

   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = Key;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    keys_iterator() = default;
    keys_iterator(bciter it) : _it(it) {}
    bool operator==(const type& rhs) const { return _it == rhs._it; }
    bool operator!=(const type& rhs) const { return !(*this == rhs); }
    const Key& operator*() const { return _it->first; }
    type& operator++() {
      ++_it;
      return *this;
    }

   private:
    bciter _it;
  };
  class keys_range {
   public:
    keys_range(const type& map) : _map(map) {}
    keys_iterator begin() const { return keys_iterator(_map.begin()); }
    keys_iterator end() const { return keys_iterator(_map.end()); }
    size_t size() const { return _map.size(); }

   private:
    const type& _map;
  };
  class cvalues_iterator {
    using type = cvalues_iterator;

   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = Value;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    cvalues_iterator() = default;
    cvalues_iterator(bciter it) : _it(it) {}
    bool operator==(const type& rhs) const { return _it == rhs._it; }
    bool operator!=(const type& rhs) const { return !(*this == rhs); }
    const Value& operator*() const { return _it->second; }
    type& operator++() {
      ++_it;
      return *this;
    }

   private:
    bciter _it;
  };
  class cvalues_range {
   public:
    cvalues_range(const type& map) : _map(map) {}
    cvalues_iterator begin() const { return cvalues_iterator(_map.begin()); }
    cvalues_iterator end() const { return cvalues_iterator(_map.end()); }
    size_t size() const { return _map.size(); }

   private:
    const type& _map;
  };
  class values_iterator {
    using type = values_iterator;

   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = Value;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    values_iterator() = default;
    values_iterator(biter it) : _it(it) {}
    bool operator==(const type& rhs) const { return _it == rhs._it; }
    bool operator!=(const type& rhs) const { return !(*this == rhs); }
    Value& operator*() const { return _it->second; }
    type& operator++() {
      ++_it;
      return *this;
    }

   private:
    biter _it;
  };
  class values_range {
   public:
    values_range(type& map) : _map(map) {}
    values_iterator begin() const { return values_iterator(_map._map.begin()); }
    values_iterator end() const { return values_iterator(_map._map.end()); }
    size_t size() const { return _map.size(); }

   private:
    type& _map;
  };

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
    if (1 && _map.size() < _map.bucket_count() / 16) _map.rehash(0);
    return value;
  }
  bciter crand(Random& random) const {  // See also similar code in Set.
    assertx(!empty());
    if (0) {
      return std::next(begin(), random.get_size_t() % _map.size());  // Likely slow; no improvement.
    } else {
      size_t nbuckets = _map.bucket_count();
      size_t bn = random.get_size_t() % nbuckets;
      size_t ne = _map.bucket_size(bn);
      size_t nskip = random.get_size_t() % (20 + ne);
      while (nskip >= _map.bucket_size(bn)) {
        nskip -= _map.bucket_size(bn);
        bn++;
        if (bn == nbuckets) bn = 0;
      }
      auto li = _map.begin(bn);
      while (nskip--) {
        ASSERTXX(li != _map.end(bn));
        ++li;
      }
      ASSERTXX(li != _map.end(bn));
      // Convert from const_local_iterator to const_iterator.
      auto it = _map.find(li->first);
      ASSERTXX(it != end());
      return it;
    }
  }
  // Default operator=() and copy_constructor are safe.
};

template <typename Key, typename Value> HH_DECLARE_OSTREAM_RANGE(Map<Key, Value>);
template <typename Key, typename Value> HH_DECLARE_OSTREAM_EOL(Map<Key, Value>);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MAP_H_
