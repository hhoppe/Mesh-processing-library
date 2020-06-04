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
  void enqueue(const T& e) { _dq.push_back(e); }
  void enqueue(T&& e) { _dq.push_back(std::move(e)); }
  T dequeue() {
    T e = std::move(front());
    _dq.pop_front();
    return e;
  }
  T& front() { return (ASSERTX(!empty()), _dq.front()); }
  const T& front() const { return (ASSERTX(!empty()), _dq.front()); }
  T& rear() { return (ASSERTX(!empty()), _dq.back()); }
  const T& rear() const { return (ASSERTX(!empty()), _dq.back()); }
  void insert_first(const T& e) { _dq.push_front(e); }
  void insert_first(T&& e) { _dq.push_front(std::move(e)); }
  bool empty() const { return _dq.empty(); }
  int length() const { return narrow_cast<int>(_dq.size()); }
  bool contains(const T& e) const {
    for (const T& v : *this) {
      if (v == e) return true;
    }
    return false;
  }
  void add_to_end(Queue<T>& q) {
    while (!q.empty()) enqueue(q.dequeue());
  }
  using value_type = T;
  using iterator = typename base::iterator;
  using const_iterator = typename base::const_iterator;
  iterator begin() { return _dq.begin(); }
  const_iterator begin() const { return _dq.begin(); }
  iterator end() { return _dq.end(); }
  const_iterator end() const { return _dq.end(); }

 private:
  base _dq;
  // Default operator=() and copy_constructor are safe.
};

template <typename T> HH_DECLARE_OSTREAM_RANGE(Queue<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(Queue<T>);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_QUEUE_H_
