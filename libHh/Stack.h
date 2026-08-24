// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_STACK_H_
#define MESH_PROCESSING_LIBHH_STACK_H_

#include <vector>

#include "libHh/RangeOp.h"  // contains()

#if 0
{
  Stack<Edge> stack;
  while (!stack.empty()) process(stack.pop());
  for (const Point& p : stackp) consider(p);
}
#endif

namespace hh {

// Remove an element from a vector; return was_there.
template <typename T> bool vec_remove_ordered(std::vector<T>& vec, const T& e) {
  auto it = ranges::find(vec, e);
  if (it == vec.end()) return false;
  vec.erase(it);
  return true;
}

// Pop an element from the back of a vector.
template <typename T> T vec_pop(std::vector<T>& vec) {
  ASSERTX(!vec.empty());
  T e = std::move(vec.back());
  vec.pop_back();
  return e;
}

// Implementation of a stack using a std::vector.
template <typename T> class Stack {
  using base = std::vector<T>;

 public:
  void clear() { _s.clear(); }
  void push(const T& e) requires Copyable<T> { _s.push_back(e); }
  void push(T&& e) { _s.push_back(std::move(e)); }
  T pop() { return vec_pop(_s); }
  [[nodiscard]] const T& top() const { return _s.back(); }
  [[nodiscard]] bool empty() const { return _s.empty(); }
  [[nodiscard]] int height() const { return narrow_cast<int>(_s.size()); }
  [[nodiscard]] size_t size() const { return _s.size(); }
  [[nodiscard]] bool contains(const T& e) const { return hh::contains(_s, e); }
  bool remove(const T& e) { return vec_remove_ordered(_s, e); }
  using value_type = T;
  using iterator = typename base::reverse_iterator;
  using const_iterator = typename base::const_reverse_iterator;
  [[nodiscard]] iterator begin() { return _s.rbegin(); }
  [[nodiscard]] const_iterator begin() const { return _s.rbegin(); }
  [[nodiscard]] iterator end() { return _s.rend(); }
  [[nodiscard]] const_iterator end() const { return _s.rend(); }

 private:
  base _s;
  // Default operator=() and copy_constructor are safe.
};

template <typename T> HH_DECLARE_OSTREAM_RANGE(Stack<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(Stack<T>);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_STACK_H_
