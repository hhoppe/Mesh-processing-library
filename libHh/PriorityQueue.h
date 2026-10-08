// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PRIORITYQUEUE_H_
#define MESH_PROCESSING_LIBHH_PRIORITYQUEUE_H_

#include "libHh/Array.h"
#include "libHh/Map.h"
#include "libHh/Set.h"

namespace hh {

namespace details::PQ {
template <typename T> struct Node {
  Node() = default;
  explicit Node(const T& e, float pri) : _e(e), _pri(pri) {}
  explicit Node(T&& e, float pri) : _e(std::move(e)), _pri(pri) {}
  Node& operator=(Node&& n) noexcept {
    _e = std::move(n._e);
    _pri = n._pri;
    return *this;
  }
  T _e;
  float _pri;
};
}  // namespace details::PQ

// Self-resizing priority queue.
// If inline_capacity > 0, it has built-in storage for that many elements (as in InlinedArray).
template <typename T, int inline_capacity = 0> class PriorityQueue : noncopyable {
 public:
  void clear() { _ar.clear(); }
  void enter(const T& e, float pri) requires Copyable<T> { ASSERTX(pri >= 0.f), enter_i(e, pri); }
  void enter(T&& e, float pri) { ASSERTX(pri >= 0.f), enter_i(std::move(e), pri); }
  void reserve(int size) { _ar.reserve(size); }
  [[nodiscard]] int num() const { return _ar.num(); }
  [[nodiscard]] size_t size() const { return _ar.size(); }
  [[nodiscard]] bool empty() const { return !num(); }
  [[nodiscard]] const T& min() const { return ASSERTXX(!empty()), _ar[0]._e; }
  [[nodiscard]] float min_priority() const { return ASSERTXX(!empty()), _ar[0]._pri; }
  T remove_min() { return ASSERTXX(!empty()), remove_min_i(); }
  // Elements entered with enter_unsorted() must be followed by heapify() before any other operation.
  void enter_unsorted(const T& e, float pri) requires Copyable<T> {
    return ASSERTX(pri >= 0.f), _ar.push(Node(e, pri));
  }
  void enter_unsorted(T&& e, float pri) { ASSERTX(pri >= 0.f), _ar.push(Node(std::move(e), pri)); }
  void heapify() { heapify_i(); }  // Establish the heap order, in time O(num()).
  // Remove the elements for which pred(e, pri) is true, in time O(num()).  The remaining nodes keep their relative
  // order in the underlying array before it is heapified, so the result does not depend on how pred is evaluated.
  template <typename Pred> void remove_if(Pred pred) { remove_if_i(pred); }

 private:
  using Node = details::PQ::Node<T>;
  GeneralArray<Node, inline_capacity> _ar;
  void nmove(int n1, int n2) {
    _ar[n1]._e = std::move(_ar[n2]._e);
    _ar[n1]._pri = _ar[n2]._pri;
  }
  int adjust_up(int n, const float cp) {
    while (n > 0) {
      const int pn = (n - 1) / 2;  // Parent node.
      if (cp >= _ar[pn]._pri) break;
      nmove(n, pn), n = pn;
    }
    return n;
  }
  int adjust_down(int n, const float cp) {
    for (;;) {
      int c = n * 2 + 1;                                         // Left child node.
      if (c >= num()) break;                                     // No children.
      if (c + 1 < num() && _ar[c + 1]._pri <= _ar[c]._pri) c++;  // The right child is smaller (or equal).
      if (cp <= _ar[c]._pri) break;
      nmove(n, c), n = c;
    }
    return n;
  }
  void enter_i(const T& e, float pri) requires Copyable<T> {
    _ar.add(1);  // Leave this new node uninitialized.
    const int j = adjust_up(num() - 1, pri);
    _ar[j]._e = e;
    _ar[j]._pri = pri;
  }
  void enter_i(T&& e, float pri) {
    _ar.add(1);  // Leave this new node uninitialized.
    const int j = adjust_up(num() - 1, pri);
    _ar[j]._e = std::move(e);
    _ar[j]._pri = pri;
  }
  T remove_min_i() {
    T e = std::move(_ar[0]._e);
    if (num() == 1) {
      _ar.sub(1);
      return e;
    }
    T e0 = std::move(_ar.last()._e);
    const float pri = _ar.last()._pri;
    _ar.sub(1);
    const int j = adjust_down(0, pri);
    _ar[j]._e = std::move(e0);
    _ar[j]._pri = pri;
    return e;
  }
  template <typename Pred> void remove_if_i(Pred pred) {
    int n = 0;
    for_int(i, num()) {
      if (pred(std::as_const(_ar[i]._e), _ar[i]._pri)) continue;
      if (n != i) _ar[n] = std::move(_ar[i]);
      n++;
    }
    _ar.sub(num() - n);
    heapify_i();
  }
  void heapify_i() {
    for (int i = (num() - 2) / 2; i >= 0; --i) {
      T e = std::move(_ar[i]._e);
      const float pri = _ar[i]._pri;
      const int j = adjust_down(i, pri);
      // It would be nice to have faster case for j == i.
      _ar[j]._e = std::move(e);
      _ar[j]._pri = pri;
    }
  }
};

// Priority queue whose elements can also be looked up, updated, and removed, using a hash map from each element to
// its current priority.  An update or removal leaves the element's old node in the underlying heap, where it is
// discarded once it reaches the top ("lazy deletion"); the top node is always current.  When the discarded nodes
// would outnumber the current ones, the heap is rebuilt.
template <typename T, typename Hash = std::hash<T>, typename Equal = std::equal_to<T>>
class UpdatablePriorityQueue : noncopyable {
  static_assert(Copyable<T> && Hashable<T, Hash, Equal>);
  // The map and the set below are only used for lookups (rebuild() traverses the heap array instead), so their hash
  // does not affect the order of the elements.  If T is a pointer and the Hash parameter is left at its default
  // std::hash<T> (possibly specialized, as for mesh elements), they use hash_address instead, which, unlike a hash
  // that reads the element, remains valid for the stale keys of destroyed elements that the lazy deletion looks up.
  using LookupHash =
      std::conditional_t<std::is_pointer_v<T> && std::is_same_v<Hash, std::hash<T>>, hash_address, Hash>;

 public:
  void clear() { _pq.clear(), _m.clear(); }
  void enter(const T& e, float pri) { ASSERTX(pri >= 0.f), _m.enter(e, pri), _pq.enter(e, pri); }  // e must be new.
  void reserve(int size) { _pq.reserve(size); }
  [[nodiscard]] int num() const { return _m.num(); }
  [[nodiscard]] size_t size() const { return _m.size(); }
  [[nodiscard]] bool empty() const { return !num(); }
  [[nodiscard]] const T& min() const { return ASSERTXX(!empty()), _pq.min(); }
  [[nodiscard]] float min_priority() const { return ASSERTXX(!empty()), _pq.min_priority(); }
  T remove_min() {
    ASSERTXX(!empty());
    T e = _pq.remove_min();
    _m.remove(e);
    after_change();
    return e;
  }
  void enter_unsorted(const T& e, float pri) { ASSERTX(pri >= 0.f), _m.enter(e, pri), _pq.enter_unsorted(e, pri); }
  void heapify() { _pq.heapify(); }  // (See PriorityQueue::heapify().)
  [[nodiscard]] bool contains(const T& e) const { return _m.contains(e); }
  [[nodiscard]] float retrieve(const T& e) const {  // Returns the priority of e, or a negative value if e is absent.
    const float* p = _m.find_ptr(e);
    return p ? *p : -1.f;
  }
  float remove(const T& e) {  // Returns the priority of e, or a negative value if e is absent.
    const float* p = _m.find_ptr(e);
    if (!p) return -1.f;
    const float pri = *p;
    _m.remove(e);
    after_change();
    return pri;
  }
  float update(const T& e, float pri) {  // Returns the previous priority, or a negative value if e is absent.
    ASSERTX(pri >= 0.f);
    float* p = _m.find_ptr(e);
    if (!p) return -1.f;
    const float oldpri = *p;
    if (pri != oldpri) set_priority(*p, e, pri);
    return oldpri;
  }
  float enter_update(const T& e, float pri) {  // Returns the previous priority, or a negative value if e is absent.
    ASSERTX(pri >= 0.f);
    float* p = _m.find_ptr(e);
    if (!p) return enter(e, pri), -1.f;
    const float oldpri = *p;
    if (pri != oldpri) set_priority(*p, e, pri);
    return oldpri;
  }
  bool enter_update_if_smaller(const T& e, float pri) {
    ASSERTX(pri >= 0.f);
    float* p = _m.find_ptr(e);
    if (!p) return enter(e, pri), true;
    if (!(pri < *p)) return false;
    set_priority(*p, e, pri);
    return true;
  }
  bool enter_update_if_greater(const T& e, float pri) {
    ASSERTX(pri >= 0.f);
    float* p = _m.find_ptr(e);
    if (!p) return enter(e, pri), true;
    if (!(pri > *p)) return false;
    set_priority(*p, e, pri);
    return true;
  }

 private:
  PriorityQueue<T> _pq;                 // Nodes of current elements, plus stale nodes (never at the top).
  Map<T, float, LookupHash, Equal> _m;  // Element -> current priority.

  void set_priority(float& cur_pri, const T& e, float pri) {  // cur_pri is the entry of e in _m.
    cur_pri = pri;
    _pq.enter(e, pri);
    after_change();
  }
  void after_change() {  // Restore the invariants after an update or removal.
    discard_stale_top();
    if (_pq.num() > 2 * num() + 16) rebuild();
  }
  [[nodiscard]] bool is_current(const T& e, float pri) const {
    const float* p = _m.find_ptr(e);
    return p && *p == pri;
  }
  void discard_stale_top() {
    while (!_pq.empty() && !is_current(_pq.min(), _pq.min_priority())) _pq.remove_min();
  }
  // Remove the stale nodes, and all but one of any duplicate nodes, in time O(num()).  The removal preserves the order
  // of the remaining nodes in the underlying array (see PriorityQueue::remove_if()), rather than traversing the hash
  // map, so that the order of equal priorities does not depend on the hash values of the elements.
  void rebuild() {
    Set<T, LookupHash, Equal> kept;  // (A removed and re-entered element may have two current nodes.)
    _pq.remove_if([&](const T& e, float pri) { return !is_current(e, pri) || !kept.add(e); });
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_PRIORITYQUEUE_H_
