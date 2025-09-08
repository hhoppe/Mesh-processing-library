// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_STREE_H_
#define MESH_PROCESSING_LIBHH_STREE_H_

#include <set>

#include "libHh/Hh.h"

#if 0
{
  STree<int> stree1;

  struct less_astruct {
    bool operator()(const astruct& s1, const astruct& s2) const { return compare_astruct(s1, s2) < 0; }
  };
  STree<astruct, less_astruct> stree2;
}
#endif

namespace hh {

// Splay Tree (originally); now implemented by std::map which is usually a red-black tree.
// (typename Less also goes by name Compare in C++ standard library)
template <typename T, typename Less = std::less<T>> class STree : noncopyable {
  using base = std::set<T, Less>;

 public:
  void clear() { _s.clear(); }
  bool empty() const { return _s.empty(); }
  int num() const { return narrow_cast<int>(_s.size()); }
  size_t size() const { return _s.size(); }
  // To avoid ambiguity, e should not equal T{}.
  bool enter(const T& e) {  // ret: is_new
    const auto [it, is_new] = _s.insert(e);
    return is_new;
  }
  const T& retrieve(const T& e) const {  // or ret=T{}
    auto it = _s.find(e);
    return it != end() ? *it : def();
  }
  bool remove(const T& e) { return _s.erase(e) > 0; }
  const T& pred(const T& e) const {
    auto it = _s.lower_bound(e);
    return it != begin() ? *--it : def();
  }
  const T& succ(const T& e) const {
    auto it = _s.upper_bound(e);
    return it != end() ? *it : def();
  }
  const T& pred_eq(const T& e) const {
    auto it = _s.upper_bound(e);
    return it != begin() ? *--it : def();
  }
  const T& succ_eq(const T& e) const {
    auto it = _s.lower_bound(e);
    return it != end() ? *it : def();
  }
  const T& min() const { return (ASSERTXX(!empty()), *begin()); }
  const T& max() const { return (ASSERTXX(!empty()), *--end()); }
  using value_type = T;
  using iterator = typename base::iterator;
  using const_iterator = typename base::const_iterator;
  iterator begin() { return _s.begin(); }
  const_iterator begin() const { return _s.begin(); }
  iterator end() { return _s.end(); }
  const_iterator end() const { return _s.end(); }

 private:
  base _s;
  static const T& def() {
    static const T k_default = T{};
    return k_default;
  }
};

template <typename T> HH_DECLARE_OSTREAM_RANGE(STree<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(STree<T>);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_STREE_H_
