// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_SET_H_
#define MESH_PROCESSING_LIBHH_SET_H_

#include <unordered_set>

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
template <typename T, typename Hash = std::hash<T>, typename Equal = std::equal_to<T>>
requires Hashable<T, Hash, Equal> class Set {
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
  [[nodiscard]] bool contains(const T& e) const { return _set.find(e) != end(); }
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
    if (1 && _set.size() < _set.bucket_count() / 16) _set.rehash(0);
    return true;
  }
  const_iterator crand(Random& r) const {  // See also similar code in Map.
    assertx(!empty());
    if (0) {
      return std::next(begin(), r.get_size_t() % _set.size());  // Likely slow; no improvement.
    } else {
      const size_t nbuckets = _set.bucket_count();
      size_t bn = r.get_size_t() % nbuckets;
      const size_t ne = _set.bucket_size(bn);
      size_t nskip = r.get_size_t() % (20 + ne);
      while (nskip >= _set.bucket_size(bn)) {
        nskip -= _set.bucket_size(bn);
        bn++;
        if (bn == nbuckets) bn = 0;
      }
      auto li = _set.begin(bn);
      while (nskip--) {
        ASSERTXX(li != _set.end(bn));
        ++li;
      }
      ASSERTXX(li != _set.end(bn));
      // Convert from const_local_iterator to const_iterator.
      auto it = _set.find(*li);
      ASSERTXX(it != _set.end());
      return it;
    }
  }
  // Default operator=() and copy_constructor are safe.
};

// Set with built-in storage for inline_capacity elements, which are searched linearly; when a new element exceeds
// this capacity, all elements move into a hash Set, which is used from then on.  Like InlinedArray, it avoids heap
// allocation while the set is small.  Elements must be default-constructible, copyable, and cheap to compare, and
// Hash and Equal must be consistent (as for any Set).  Unlike Set, it offers neither iteration nor references to its
// elements.
template <typename T, int inline_capacity, typename Hash = std::hash<T>, typename Equal = std::equal_to<T>>
requires Hashable<T, Hash, Equal> class InlinedSet {
  static_assert(inline_capacity > 0);

 public:
  void clear() { _n = 0, _large.clear(); }
  void enter(const T& e) {  // Element e must be new.
    [[maybe_unused]] const bool is_new = add(e);
    ASSERTX(is_new);
  }
  bool add(const T& e) {  // Returns true if e is new.
    if (_n <= inline_capacity) {
      if (contains_inline(e)) return false;
      if (_n < inline_capacity) return _builtin[_n++] = e, true;
      for (const T& ee : _builtin) _large.enter(ee);
      _n = inline_capacity + 1;
    }
    return _large.add(e);
  }
  [[nodiscard]] bool contains(const T& e) const {
    return _n <= inline_capacity ? contains_inline(e) : _large.contains(e);
  }
  [[nodiscard]] int num() const { return _n <= inline_capacity ? _n : _large.num(); }
  [[nodiscard]] bool empty() const { return !num(); }

 private:
  int _n{0};  // Number of elements in _builtin, or inline_capacity + 1 once the elements are in _large.
  Vec<T, inline_capacity> _builtin;
  Set<T, Hash, Equal> _large;
  [[nodiscard]] bool contains_inline(const T& e) const {
    for_int(i, _n) {
      if (Equal{}(_builtin[i], e)) return true;
    }
    return false;
  }
};

template <typename T> HH_DECLARE_OSTREAM_RANGE(Set<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(Set<T>);

// Template deduction guides:
template <ranges::input_range R> Set(R&&) -> Set<range_value_t<R>>;

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SET_H_
