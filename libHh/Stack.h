// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_STACK_H_
#define MESH_PROCESSING_LIBHH_STACK_H_

#include <vector>

#include "Hh.h"

#if 0
{
  Stack<Edge> stack;
  while (!stack.empty()) process(stack.pop());
  for (const Point& p : stackp) consider(p);
}
#endif

namespace hh {

// Does a vector contain an element?
template<typename T> bool vec_contains(const std::vector<T>& vec, const T& e) {
    // return std::find(vec.begin(), vec.end(), e)!=vec.end();
    for (const T& ee : vec) { if (e==ee) return true; } return false;
}
// Remove an element from a vector; return was_there.
template<typename T> bool vec_remove_ordered(std::vector<T>& vec, const T& e) {
    // auto it = std::find(vec.begin(), vec.end(), e); if (it==vec.end()) return false; vec.erase(it); return true;
    for (auto it(vec.begin()); it!=vec.end(); ++it) { if (*it==e) { vec.erase(it); return true; } }
    return false;
}
// Pop an element from the back of a vector.
template<typename T> T vec_pop(std::vector<T>& vec) { T e = std::move(vec.back()); vec.pop_back(); return e; }

// Implementation of a stack using a std::vector.
template<typename T> class Stack {
    using base = std::vector<T>;
 public:
    void clear()                                { _s.clear(); }
    void push(const T& e)                       { _s.push_back(e); }
    void push(T&& e)                            { _s.push_back(std::move(e)); }
    T pop()                                     { return vec_pop(_s); }
    const T& top() const                        { return _s.back(); }
    bool empty() const                          { return _s.empty(); }
    int height() const                          { return narrow_cast<int>(_s.size()); }
    bool contains(const T& e) const             { return vec_contains(_s, e); }
    bool remove(const T& e)                     { return vec_remove_ordered(_s, e); }
    using value_type = T;
    using iterator = typename base::reverse_iterator;
    using const_iterator = typename base::const_reverse_iterator;
    iterator begin()                            { return _s.rbegin(); }
    const_iterator begin() const                { return _s.rbegin(); }
    iterator end()                              { return _s.rend(); }
    const_iterator end() const                  { return _s.rend(); }
 private:
    base _s;
    // Default operator=() and copy_constructor are safe.
};

template<typename T> HH_DECLARE_OSTREAM_RANGE(Stack<T>);
template<typename T> HH_DECLARE_OSTREAM_EOL(Stack<T>);

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_STACK_H_
