// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Queue.h"

#include <deque>

#include "libHh/Random.h"
using namespace hh;

namespace {

// Asserts that the queue holds exactly the elements of the reference deque, in the same order.
void verify_equal(const Queue<int>& q, const std::deque<int>& dq) {
  assertx(q.length() == int(dq.size()) && q.size() == dq.size() && q.empty() == dq.empty());
  assertx(ranges::equal(q, dq));
  if (!dq.empty()) assertx(q.front() == dq.front() && q.rear() == dq.back());
}

}  // namespace

int main() {
  {
    Queue<int> q;
    assertx(q.empty());
    for (const int i : q) {
      dummy_use(i);
      if (1) assertnever("");
    }
    for_int(i, 4) q.enqueue(i);
    assertx(q.length() == 4);
    assertx(!q.empty());
    assertx(q.contains(1));
    assertx(q.front() == 0);
    assertx(q.rear() == 3);
    assertx(q.dequeue() == 0);
    assertx(q.dequeue() == 1);
    {
      int i = 0;
      for (const int j : q) assertx(j == 2 + i++);
      assertx(i == 2);
    }
    assertx(!q.contains(1));
    q.insert_first(5);
    q.insert_first(4);
    SHOW(q);
    assertx(q.dequeue() == 4);
    assertx(q.dequeue() == 5);
    assertx(q.dequeue() == 2);
    assertx(q.dequeue() == 3);
    assertx(q.empty());
    assertx(!q.contains(3));
  }
  {
    // A single element is both the front and the rear, and both are modifiable.
    Queue<int> q;
    q.enqueue(7);
    assertx(&q.front() == &q.rear());
    q.front() = 8;
    assertx(q.rear() == 8);
    q.rear() += 1;
    for (int& e : q) e *= 10;  // The iterators of a mutable Queue are mutable.
    assertx(q.length() == 1 && q.dequeue() == 90 && q.empty());
    // A const Queue yields const elements.
    static_assert(std::is_same_v<decltype(std::as_const(q).front()), const int&>);
    static_assert(std::is_same_v<decltype(*std::as_const(q).begin()), const int&>);
    static_assert(std::is_same_v<decltype(q.front()), int&>);
  }
  {
    // add_to_end() transfers all the elements of another queue, in order, leaving it empty.
    Queue<int> q1, q2;
    for_int(i, 3) q1.enqueue(i);
    for_int(i, 3) q2.enqueue(10 + i);
    q1.add_to_end(q2);
    assertx(q2.empty());
    SHOW(q1);
    q1.add_to_end(q2);  // Transferring from an empty queue has no effect.
    assertx(q1.length() == 6);
    q2.add_to_end(q1);  // Transferring into an empty queue.
    assertx(q1.empty() && q2.length() == 6 && q2.front() == 0 && q2.rear() == 12);
    q2.clear();
    assertx(q2.empty() && q2.length() == 0);
  }
  {
    // Copies are independent.
    Queue<int> q1;
    q1.enqueue(1);
    q1.enqueue(2);
    Queue<int> q2 = q1;
    q2.enqueue(3);
    assertx(q1.length() == 2 && q2.length() == 3);
    q1 = q2;
    assertx(q1.length() == 3 && q1.rear() == 3);
  }
  {
    Queue<unique_ptr<int>> q;
    q.enqueue(make_unique<int>(4));
    q.enqueue(make_unique<int>(5));
    q.insert_first(make_unique<int>(3));
    assertx(*q.front() == 3 && *q.rear() == 5);
    auto up0 = q.dequeue();
    auto up1 = q.dequeue();
    auto up2 = q.dequeue();
    assertx(*up0 == 3);
    assertx(*up1 == 4);
    assertx(*up2 == 5);
    assertx(q.empty());
    // A move-only Queue can still be moved.
    q.enqueue(make_unique<int>(6));
    Queue<unique_ptr<int>> q2 = std::move(q);
    assertx(q2.length() == 1 && *q2.front() == 6);
  }
  {
    Queue<string> q;
    q.enqueue("a");
    string s = "b";
    q.enqueue(s);  // The const& overload copies.
    assertx(s == "b");
    q.enqueue(std::move(s));
    q.insert_first("z");
    SHOW(q);
    assertx(q.contains("b") && !q.contains("c"));
  }
  {
    // Random operations compared against std::deque as a reference model.
    Random random{3};
    Queue<int> q;
    std::deque<int> dq;
    for_int(iter, 2000) {
      const unsigned op = random.get_unsigned(10);
      const int value = int(random.get_unsigned(100));
      if (op < 4) {
        q.enqueue(value), dq.push_back(value);
      } else if (op < 6) {
        q.insert_first(value), dq.push_front(value);
      } else if (op < 9) {
        if (!dq.empty()) {
          assertx(q.dequeue() == dq.front());
          dq.pop_front();
        }
      } else {
        assertx(q.contains(value) == (ranges::find(dq, value) != dq.end()));
      }
      verify_equal(q, dq);
    }
  }
}

template class hh::Queue<unsigned>;
template class hh::Queue<double>;
template class hh::Queue<const int*>;
template class hh::Queue<unique_ptr<int>>;
