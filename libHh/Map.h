// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MAP_H_
#define MESH_PROCESSING_LIBHH_MAP_H_

#include <unordered_map>

#include "libHh/Random.h"

#if 0
{
  Map<Edge, Vertex> mev;  // default uses std::hash<K>and std::equal_to<K> (which tries operator ==)
  for_map_key_value(mev, [&](Edge e, Vertex v) { func(e, v); });
  for (Edge e : mev.keys()) func(e);
  for (Vertex v : mev.values()) func(v);
  //
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
  // See Set.h for other examples using std::hash<> and explicit hash functional constructor
}
#endif

namespace hh {

// Map is very similar to std::unordered_map but using my own accessor functions.
// (The typename Equal also goes by the name Pred in the C++ standard library)
template <typename K, typename V, typename Hash = std::hash<K>, typename Equal = std::equal_to<K>> class Map {
  using type = Map<K, V, Hash, Equal>;
  using base = std::unordered_map<K, V, Hash, Equal>;
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
  explicit Map(Hashf hashf) : _m(0, hashf) {}
  explicit Map(Hashf hashf, Equalf equalf) : _m(0, hashf, equalf) {}
  void clear() { _m.clear(); }
  void enter(const K& k, const V& v) {  // k new!
    auto p = _m.emplace(k, v);
    ASSERTX(p.second);
  }
  void enter(K&& k, const V& v) {
    auto p = _m.emplace(std::move(k), v);
    ASSERTX(p.second);
  }
  void enter(const K& k, V&& v) {
    auto p = _m.emplace(k, std::move(v));
    ASSERTX(p.second);
  }
  void enter(K&& k, V&& v) {
    auto p = _m.emplace(std::move(k), std::move(v));
    ASSERTX(p.second);
  }
  V& enter(const K& k, const V& v, bool& is_new) {  // does not modify element if it already exists
    auto p = _m.emplace(k, v);
    is_new = p.second;
    return p.first->second;
  }
  // omit "V& enter(const K& k, V&& v, bool& is_new)" because v could be lost if !is_new
  // note: force_enter using: { map[k] = std::move(v); }
  bool contains(const K& k) const { return _m.find(k) != end(); }
  const V& retrieve(const K& k, bool& present) const {
    auto i = _m.find(k);
    present = i != end();
    return present ? i->second : def();
  }
  const V& retrieve(const K& k) const {
    auto i = _m.find(k);
    if (i == end()) return def();
    return i->second;
  }
  V& get(const K& k) {
    auto i = _m.find(k);
    ASSERTXX(i != end());
    return i->second;
  }
  const V& get(const K& k) const {
    auto i = _m.find(k);
    ASSERTXX(i != end());
    return i->second;
  }
  // const V& get(const K& k) const { return (*this)[k]; } // bad: throws exception if absent
  V remove(const K& k) { return remove_i(k); }
  V replace(const K& k, const V& v) {
    auto i = _m.find(k);
    if (i == end()) return V();
    V vo = i->second;
    i->second = v;
    return vo;
  }
  // omit "V replace(const K& k, V&& v)" because v could be lost if !present
  int num() const { return narrow_cast<int>(_m.size()); }
  size_t size() const { return _m.size(); }
  bool empty() const { return _m.empty(); }
  V& operator[](const K& k) { return _m[k]; }  // introduced for Combination
  const V& operator[](const K& k) const {
    auto i = _m.find(k);
    return i != end() ? i->second : def();
  }
  const K& get_one_key() const { return (ASSERTXX(!empty()), begin()->first); }
  const V& get_one_value() const { return (ASSERTXX(!empty()), begin()->second); }
  const K& get_random_key(Random& r) const { return crand(r)->first; }
  const V& get_random_value(Random& r) const { return crand(r)->second; }
  keys_range keys() const { return keys_range(*this); }  // keys are always constant
  values_range values() { return values_range(*this); }
  cvalues_range cvalues() { return cvalues_range(*this); }
  cvalues_range values() const { return cvalues_range(*this); }
  // for for_map_key_value and HH_DECLARE_OSTREAM_RANGE(Map<K, V>):
  bciter begin() const { return _m.begin(); }
  bciter end() const { return _m.end(); }

 public:
  class keys_iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = K;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    keys_iterator() = default;
    keys_iterator(bciter it) : _it(it) {}
    bool operator!=(const keys_iterator& rhs) const { return _it != rhs._it; }
    bool operator==(const keys_iterator& rhs) const { return _it == rhs._it; }
    const K& operator*() const { return _it->first; }
    keys_iterator& operator++() {
      ++_it;
      return *this;
    }

   private:
    bciter _it;
  };
  class keys_range {
   public:
    keys_range(const type& t) : _t(t) {}
    keys_iterator begin() const { return keys_iterator(_t.begin()); }
    keys_iterator end() const { return keys_iterator(_t.end()); }

   private:
    const type& _t;
  };
  class cvalues_iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = V;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    cvalues_iterator() = default;
    cvalues_iterator(bciter it) : _it(it) {}
    bool operator!=(const cvalues_iterator& rhs) const { return _it != rhs._it; }
    bool operator==(const cvalues_iterator& rhs) const { return _it == rhs._it; }
    const V& operator*() const { return _it->second; }
    cvalues_iterator& operator++() {
      ++_it;
      return *this;
    }

   private:
    bciter _it;
  };
  class cvalues_range {
   public:
    cvalues_range(const type& t) : _t(t) {}
    cvalues_iterator begin() const { return cvalues_iterator(_t.begin()); }
    cvalues_iterator end() const { return cvalues_iterator(_t.end()); }

   private:
    const type& _t;
  };
  class values_iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = V;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    values_iterator() = default;
    values_iterator(biter it) : _it(it) {}
    bool operator!=(const values_iterator& rhs) const { return _it != rhs._it; }
    bool operator==(const values_iterator& rhs) const { return _it == rhs._it; }
    V& operator*() const { return _it->second; }
    values_iterator& operator++() {
      ++_it;
      return *this;
    }

   private:
    biter _it;
  };
  class values_range {
   public:
    values_range(type& t) : _t(t) {}
    values_iterator begin() const { return values_iterator(_t._m.begin()); }
    values_iterator end() const { return values_iterator(_t._m.end()); }

   private:
    type& _t;
  };

 private:
  // See my experiments in ~/git/hh_src/test/misc/test_hash_buckets.cpp
  base _m;
  static const V& def() {
    static const V k_default = V();
    return k_default;
  }
  V remove_i(const K& k) {
    auto i = _m.find(k);
    if (i == end()) return V();
    auto v = std::move(i->second);
    _m.erase(i);
    if (1 && _m.size() < _m.bucket_count() / 16) _m.rehash(0);
    return v;
  }
  bciter crand(Random& r) const {  // see also similar code in Set
    assertx(!empty());
    if (0) {
      return std::next(begin(), r.get_size_t() % _m.size());  // likely slow; no improvement
    } else {
      size_t nbuckets = _m.bucket_count();
      size_t bn = r.get_size_t() % nbuckets;
      size_t ne = _m.bucket_size(bn);
      size_t nskip = r.get_size_t() % (20 + ne);
      while (nskip >= _m.bucket_size(bn)) {
        nskip -= _m.bucket_size(bn);
        bn++;
        if (bn == nbuckets) bn = 0;
      }
      auto li = _m.begin(bn);
      while (nskip--) {
        ASSERTXX(li != _m.end(bn));
        ++li;
      }
      ASSERTXX(li != _m.end(bn));
      // convert from const_local_iterator to const_iterator
      auto i = _m.find(li->first);
      ASSERTXX(i != end());
      return i;
    }
  }
  // Default operator=() and copy_constructor are safe.
};

// Iterate over both keys and values simultaneously
template <typename K, typename V, typename Hash = std::hash<K>, typename Equal = std::equal_to<K>,
          typename Func = void(const K& key, const V& val)>
inline void for_map_key_value(const Map<K, V, Hash, Equal>& map, Func func) {
  for (auto& kv : map) func(kv.first, kv.second);
}

template <typename K, typename V> HH_DECLARE_OSTREAM_RANGE(Map<K, V>);
template <typename K, typename V> HH_DECLARE_OSTREAM_EOL(Map<K, V>);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MAP_H_
