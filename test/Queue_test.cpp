// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Queue.h"
using namespace hh;

int main() {
  {
    Queue<int> q;
    assertx(q.empty());
    for (int i : q) {
      dummy_use(i);
      if (1) assertnever("");
    }
    for_int(i, 4) q.enqueue(i);
    assertx(q.length() == 4);
    assertx(!q.empty());
    assertx(q.contains(1));
    assertx(q.front() == 0);
    assertx(q.dequeue() == 0);
    assertx(q.dequeue() == 1);
    {
      int i = 0;
      for (int j : q) assertx(j == 2 + i++);
    }
    assertx(!q.contains(1));
    q.insert_first(5);
    q.insert_first(4);
    assertx(q.dequeue() == 4);
    assertx(q.dequeue() == 5);
    assertx(q.dequeue() == 2);
    assertx(q.dequeue() == 3);
    assertx(q.empty());
  }
  {
    Queue<unique_ptr<int>> q;
    q.enqueue(make_unique<int>(4));
    q.enqueue(make_unique<int>(5));
    auto up1 = q.dequeue();
    auto up2 = q.dequeue();
    assertx(*up1 == 4);
    assertx(*up2 == 5);
  }
}

template class hh::Queue<unsigned>;
template class hh::Queue<double>;
template class hh::Queue<const int*>;
template class hh::Queue<unique_ptr<int>>;
