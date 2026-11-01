// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_SET_H_
#define MESH_PROCESSING_LIBHH_SET_H_

#include <unordered_set>

#include "libHh/Array.h"
#include "libHh/Map.h"
#include "libHh/Random.h"
#include "libHh/Vec.h"

#if 0
{
  Set<Edge> sete;
  for (Edge e : set) consider(e);

  struct mypair {
    unsigned _v1, _v2;
  };
  struct hash_mypair {
    size_t operator()(const mypair& e) const { return hash_combine(hash_combine(0, e._v1), e._v2); }
  };
  struct equal_mypair {
    bool operator()(const mypair& e1, const mypair& e2) const { return e1._v1 == e2._v1 && e1._v2 == e2._v2; }
  };
  Set<mypair, hash_mypair, equal_mypair> setpairs;

  struct hash_edge {  // From MeshOp.cpp.
    ...;
    const GMesh& _mesh;
  };
  hash_edge he(mesh);
  Set<Edge, hash_edge> sete(he);
}
#endif

namespace hh {

// My wrapper around std::unordered_set<>.  (typename Equal also goes by name Pred in C++ standard library).
template <typename T, typename Hash = std::hash<T>, typename Equal = std::equal_to<T>> class Set {
  static_assert(Hashable<T, Hash, Equal>);
  using type = Set<T, Hash, Equal>;
  using base = std::unordered_set<T, Hash, Equal>;

 public:
  using Hashf = base::hasher;
  using Equalf = base::key_equal;
  using value_type = T;
  using iterator = base::iterator;
  using const_iterator = base::const_iterator;
  Set() = default;
  explicit Set(Hashf hashf) : _set(0, hashf) {}
  explicit Set(Hashf hashf, Equalf equalf) : _set(0, hashf, equalf) {}
  Set(std::initializer_list<T> l) requires Copyable<T> : _set(std::move(l)) {}
  template <input_range_to<T> R> requires(!std::same_as<std::remove_cvref_t<R>, type>) explicit Set(R&& range) {
    for (const T& e : range) add(e);
  }
  void clear() { _set.clear(); }
  void enter(const T& e) requires Copyable<T> {  // Element e must be new.
    const auto [_, is_new] = _set.insert(e);
    ASSERTX(is_new);
  }
  void enter(T&& e) {  // Element e must be new.
    const auto [_, is_new] = _set.insert(std::move(e));
    ASSERTX(is_new);
  }
  const T& enter(const T& e, bool& is_new) requires Copyable<T> {
    const auto [it, is_new_] = _set.insert(e);
    is_new = is_new_;
    return *it;
  }
  // Omit "const T& enter(T&& e, bool& is_new)" because e could be lost if !is_new.
  bool add(const T& e) requires Copyable<T> {  // Returns true if e is new.
    const auto [_, is_new] = _set.insert(e);
    return is_new;
  }
  // Omit "bool add(T&& e)" because e could be lost if !is_new.
  bool remove(const T& e) { return remove_i(e); }  // Returns true if e was found.
  [[nodiscard]] bool contains(const T& e) const { return _set.contains(e); }
  [[nodiscard]] int num() const { return narrow_cast<int>(_set.size()); }
  [[nodiscard]] size_t size() const { return _set.size(); }
  [[nodiscard]] bool empty() const { return _set.empty(); }
  [[nodiscard]] const T* find_ptr(const T& e) const {  // Returns nullptr if e is absent.
    auto it = _set.find(e);
    return it != end() ? &*it : nullptr;
  }
  [[nodiscard]] const T& retrieve(const T& e) const {
    auto it = _set.find(e);
    return it != end() ? *it : def();
  }
  [[nodiscard]] const T& get(const T& e) const {
    auto it = _set.find(e);
    ASSERTXX(it != end());
    return *it;
  }
  [[nodiscard]] const T& get_one() const { return ASSERTXX(!empty()), *begin(); }
  [[nodiscard]] const T& get_random(Random& r) const {
    auto it = crand(r);
    return *it;
  }
  T remove_one() {
    ASSERTXX(!empty());
    // T e = std::move(*begin());  // See discussion in Set_test.cpp.
    // _set.erase(begin());
    auto node = _set.extract(begin());
    return std::move(node.value());
  }
  T remove_random(Random& r) {
    auto it = crand(r);
    auto node = _set.extract(it);
    if (_set.size() < _set.bucket_count() / 16) _set.rehash(0);  // As in remove_i().
    return std::move(node.value());
  }
  [[nodiscard]] auto begin(this auto&& self) { return self._set.begin(); }
  [[nodiscard]] auto end(this auto&& self) { return self._set.end(); }
  void merge(type& other) { _set.merge(other._set); }  // Elements are moved from `other` if not already in *this.

 private:
  base _set;
  static const T& def() {
    static const T k_default = T{};
    return k_default;
  }
  bool remove_i(const T& e) {
    if (_set.erase(e) == 0) return false;
    // Shrink the table, which also bounds the expected number of attempts in crand().
    if (_set.size() < _set.bucket_count() / 16) _set.rehash(0);
    return true;
  }
  const_iterator crand(Random& r) const {  // See also similar code in Map.
    assertx(!empty());
    // Rejection sampling: uniformly draw (1) a bucket and (2) a position in [0, _crand_bound), and retry if the
    // bucket has no element at that position.  Every element is then equally likely, provided that _crand_bound is
    // at least the size of every bucket.  (Walking from a random bucket to the next element instead favors the
    // elements that follow empty buckets, whose number grows as random elements are removed.)  The bound is
    // recomputed after each rehash, and a bucket that has since grown beyond it raises it when drawn, discarding
    // that attempt.
    const size_t nbuckets = _set.bucket_count();
    if (nbuckets != _crand_nbuckets) {
      _crand_nbuckets = nbuckets;
      _crand_bound = 1;
      for (const size_t bn : range(nbuckets)) _crand_bound = max(_crand_bound, _set.bucket_size(bn));
      // A large bound indicates a poor hash function, which slows every lookup and proportionally increases the
      // expected number of rejection samples.  With good hashing, _crand_bound stays below about 12 even with 1e8
      // elements.
      assertw(_crand_bound <= 32);
    }
    for (;;) {
      const size_t bn = r.get_size_t() % nbuckets;
      const size_t ne = _set.bucket_size(bn);
      if (ne > _crand_bound) {
        _crand_bound = ne;
        assertw(_crand_bound <= 32);
        continue;
      }
      const size_t i = r.get_size_t() % _crand_bound;
      if (i >= ne) continue;
      auto li = _set.begin(bn);
      std::advance(li, i);
      return _set.find(*li);  // Convert from const_local_iterator to const_iterator.
    }
  }
  mutable size_t _crand_nbuckets{0};  // Bucket count when _crand_bound was computed.
  mutable size_t _crand_bound{0};     // Upper bound on the bucket sizes, for crand().
  // Default operator=() and copy_constructor are safe.
};

// Set whose elements are stored in an InlinedArray, in the order of insertion (except that remove() moves the last
// element into the place of the removed one), which is also the order of iteration.  While the elements fit in the
// built-in storage for inline_capacity elements, they are searched linearly, without heap allocation; once they
// exceed it, a hash Map from each element to its array index is added for the searches (and kept until destruction,
// even after clear()).  Elements must be copyable and cheap to compare, and Hash and Equal must be consistent (as for
// any Set).
template <typename T, int inline_capacity, typename Hash = std::hash<T>, typename Equal = std::equal_to<T>>
class InlinedSet {
  static_assert(Hashable<T, Hash, Equal>);
  static_assert(inline_capacity > 0);

 public:
  void clear() {
    _elements.clear();
    if (_index) _index->clear();  // (Keeping its allocated storage for reuse.)
  }
  void enter(const T& e) {  // Element e must be new.
    [[maybe_unused]] const bool is_new = add(e);
    ASSERTX(is_new);
  }
  bool add(const T& e) {  // Returns true if e is new.
    if (find(e) >= 0) return false;
    _elements.push(e);
    if (_index) {
      _index->enter(e, _elements.num() - 1);
    } else if (_elements.num() > inline_capacity) {
      _index.emplace();
      for_int(i, _elements.num()) _index->enter(_elements[i], i);
    }
    return true;
  }
  bool remove(const T& e) {  // Returns true if e was present.
    const int i = find(e);
    if (i < 0) return false;
    if (_index) (void)_index->remove(e);
    if (i < _elements.num() - 1) {
      _elements[i] = _elements.last();
      if (_index) _index->get(_elements[i]) = i;
    }
    (void)_elements.pop();
    return true;
  }
  [[nodiscard]] bool contains(const T& e) const { return find(e) >= 0; }
  [[nodiscard]] int num() const { return _elements.num(); }
  [[nodiscard]] bool empty() const { return !num(); }
  [[nodiscard]] auto begin() const { return std::as_const(_elements).begin(); }
  [[nodiscard]] auto end() const { return std::as_const(_elements).end(); }

 private:
  InlinedArray<T, inline_capacity> _elements;
  std::optional<Map<T, int, Hash, Equal>> _index;  // Element -> index in _elements; created upon the first overflow.
  [[nodiscard]] int find(const T& e) const {  // Returns the index of e in _elements, or -1.
    if (_index) {
      const int* pi = _index->find_ptr(e);
      return pi ? *pi : -1;
    }
    for_int(i, _elements.num()) {
      if (Equal{}(_elements[i], e)) return i;
    }
    return -1;
  }
};

template <typename T> HH_DECLARE_OSTREAM_RANGE(Set<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(Set<T>);

// Template deduction guides:
template <ranges::input_range R> Set(R&&) -> Set<range_value_t<R>>;

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SET_H_
