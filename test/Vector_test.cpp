// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <deque>
#include <list>
#include <vector>

#include "libHh/RangeOp.h"  // contains()
#include "libHh/Stack.h"    // vec_pop(), vec_remove_ordered()
using namespace hh;

namespace {

// *** adapted from Stack_test.cpp

void test_stack() {
  {
    struct S {
      explicit S(int i) : _i(i) {}
      int _i;
    };
    const S s1(1), s2(2), s3(3);  // The vector holds non-owning pointers to these.
    std::vector<const S*> s;
    assertx(s.empty());
    s.push_back(&s1);
    s.push_back(&s2);
    s.push_back(&s3);
    assertx(vec_pop(s)->_i == 3);
    assertx(vec_pop(s) == &s2);
    assertx(vec_pop(s)->_i == 1);
    assertx(s.empty());
  }
  {
    // vec_pop() moves the element out, so it also works with move-only types.
    std::vector<unique_ptr<int>> s;
    s.push_back(make_unique<int>(4));
    s.push_back(make_unique<int>(5));
    const unique_ptr<int> p = vec_pop(s);
    assertx(*p == 5 && s.size() == 1 && *s[0] == 4);
  }
  {
    // vec_remove_ordered() removes the first matching element and preserves the order of the others.
    std::vector<int> s{3, 1, 4, 1, 5};
    assertx(vec_remove_ordered(s, 1));
    assertx((s == std::vector<int>{3, 4, 1, 5}));
    assertx(vec_remove_ordered(s, 5));
    assertx((s == std::vector<int>{3, 4, 1}));
    assertx(!vec_remove_ordered(s, 7));
    assertx(s.size() == 3);
    assertx(vec_remove_ordered(s, 1) && vec_remove_ordered(s, 3) && vec_remove_ordered(s, 4));
    assertx(s.empty() && !vec_remove_ordered(s, 4));
  }
  {
    std::vector<int> s;
    s.push_back(0);
    s.push_back(1);
    s.push_back(2);
    int i = 0;
    for (const int j : s) assertx(j == i++);
    assertx(i == 3);
    assertx(vec_pop(s) == 2);
    assertx(vec_pop(s) == 1);
    assertx(vec_pop(s) == 0);
    assertx(s.empty());
  }
  {
    std::vector<float> s;
    s.push_back(1);
    s.push_back(4);
    s.push_back(9);
    int i = 0;
    for (const float v : s) {
      assertx(v == square(i + 1));
      i++;
    }
    assertx(i == 3);
    assertx(vec_pop(s) == 9);
    assertx(vec_pop(s) == 4);
    assertx(vec_pop(s) == 1);
    assertx(s.empty());
  }
  {
    std::vector<int> s;
    assertx(s.empty());
    for (const int i : s) {
      dummy_use(i);
      if (1) assertnever("");
    }
    for_int(i, 4) s.push_back(i);
    assertx(s.size() == 4);
    assertx(!s.empty());
    assertx(contains(s, 2));
    assertx(s.back() == 3);
    assertx(vec_pop(s) == 3);
    assertx(vec_pop(s) == 2);
    {
      int i = 0;
      for (const int j : s) assertx(j == i++);
    }
    assertx(!contains(s, 2));
    assertx(vec_pop(s) == 1);
    assertx(vec_pop(s) == 0);
    assertx(s.empty());
  }
}

// *** from Queue_test.cpp

template <typename T> bool queue_contains(const std::deque<T>& queue, const T& e) {
  for (auto& ee : queue)
    if (e == ee) return true;
  return false;
}

template <typename T> T queue_pop(std::deque<T>& queue) {
  T e = std::move(queue.front());
  queue.pop_front();
  return e;
}

void test_queue() {
  std::deque<int> q;
  assertx(q.empty());
  for (const int i : q) {
    dummy_use(i);
    if (1) assertnever("");
  }
  for_int(i, 4) q.push_back(i);
  assertx(q.size() == 4);
  assertx(!q.empty());
  assertx(queue_contains(q, 1));
  assertx(q.front() == 0);
  assertx(queue_pop(q) == 0);
  assertx(queue_pop(q) == 1);
  {
    int i = 0;
    for (const int j : q) assertx(j == 2 + i++);
  }
  assertx(!queue_contains(q, 1));
  q.push_front(5);
  q.push_front(4);
  assertx(queue_pop(q) == 4);
  assertx(queue_pop(q) == 5);
  assertx(queue_pop(q) == 2);
  assertx(queue_pop(q) == 3);
  assertx(q.empty());
}

// *** adapted from the former HList_test

template <typename T> bool list_contains(const std::list<T>& l, const T& e) {
  for (const auto& ee : l)
    if (e == ee) return true;
  return false;
}

void test_list() {
  std::list<int> l;
  assertx(l.empty());
  for (const int i : l) {
    dummy_use(i);
    if (1) assertnever("");
  }
  l.push_front(1);
  l.push_front(0);
  l.push_back(2);
  l.push_back(3);
  assertx(l.size() == 4);
  assertx(!l.empty());
  for_intL(i, -2, 6) assertx(list_contains(l, i) == (i >= 0 && i < 4));
  assertx(l.front() == 0);
  assertx(l.back() == 3);
  assertx(l.front() == 0);
  l.pop_front();
  assertx(l.front() == 1);
  l.pop_front();
  assertx(l.back() == 3);
  l.pop_back();
  assertx(l.back() == 2);
  l.pop_back();
  assertx(l.empty());
  assertx(l.size() == 0);
  l.push_back(1);
  l.push_back(2);
  l.insert(ranges::find(l, 1), 0);
  l.insert(++ranges::find(l, 2), 3);
  {
    int i = 0;
    for (const int j : l) assertx(j == i++);
  }
  l.clear();
}

}  // namespace

int main() {
  test_stack();
  test_queue();
  test_list();
}

template class std::vector<int>;
template class std::vector<void*>;
// template class std::vector<unique_ptr<int>>;  // Full instantiation is unsupported; we cannot modify std namespace.
