// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_QUEUE_H_
#define MESH_PROCESSING_LIBHH_QUEUE_H_

#include <deque>

#include "libHh/Hh.h"

#if 0
{
  Queue<Vertex> queuev;
  while (!queue.empty()) process(queue.dequeue());
  for (const Point& p : queuep) consider(p);
}
#endif

namespace hh {

// Queue is my wrapper around std::deque<>; it provides dequeue() which combines front() and pop_front().
template <typename T> class Queue {
  using base = std::deque<T>;

 public:
  void clear() { _dq.clear(); }
  void enqueue(const T& e) requires Copyable<T> { _dq.push_back(e); }
  void enqueue(T&& e) { _dq.push_back(std::move(e)); }
  T dequeue() {
    T e = std::move(front());
    _dq.pop_front();
    return e;
  }
  [[nodiscard]] auto& front(this auto&& self) { return ASSERTX(!self.empty()), self._dq.front(); }
  [[nodiscard]] auto& rear(this auto&& self) { return ASSERTX(!self.empty()), self._dq.back(); }
  void insert_first(const T& e) requires Copyable<T> { _dq.push_front(e); }
  void insert_first(T&& e) { _dq.push_front(std::move(e)); }
  [[nodiscard]] bool empty() const { return _dq.empty(); }
  [[nodiscard]] int length() const { return narrow_cast<int>(_dq.size()); }
  [[nodiscard]] size_t size() const { return _dq.size(); }
  [[nodiscard]] bool contains(const T& e) const {
    for (const T& v : *this)
      if (v == e) return true;
    return false;
  }
  void add_to_end(Queue<T>& q) {
    while (!q.empty()) enqueue(q.dequeue());
  }
  using value_type = T;
  using iterator = typename base::iterator;
  using const_iterator = typename base::const_iterator;
  [[nodiscard]] auto begin(this auto&& self) { return self._dq.begin(); }
  [[nodiscard]] auto end(this auto&& self) { return self._dq.end(); }

 private:
  base _dq;
  // Default operator=() and copy_constructor are safe.
};

template <typename T> HH_DECLARE_OSTREAM_RANGE(Queue<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(Queue<T>);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_QUEUE_H_
