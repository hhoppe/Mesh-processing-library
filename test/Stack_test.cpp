// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Stack.h"
using namespace hh;

int main() {
  struct ST {
    ST(int i) : _i(i) { showf("ST(%d)\n", _i); }
    ~ST() { showf("~ST(%d)\n", _i); }
    int _i;
  };
  {
    Stack<const ST*> s;
    assertx(s.empty());
    s.push(new ST(1));  // never deleted
    s.push(new ST(2));
    s.push(new ST(3));
    SHOW(s.top()->_i);
    assertx(!s.empty());
    assertw(s.pop()->_i == 3);
    assertw(s.pop()->_i == 2);
    assertw(s.pop()->_i == 1);
    assertx(s.empty());
  }
  {
    Stack<int> s;
    assertx(s.empty());
    for (int i : s) {
      dummy_use(i);
      if (1) assertnever("");
    }
    for_int(i, 4) s.push(i);
    assertw(s.height() == 4);
    assertw(!s.empty());
    assertw(s.contains(2));
    assertw(s.top() == 3);
    assertw(s.pop() == 3);
    assertw(s.pop() == 2);
    {
      int i = 0;
      for (int j : s) {
        assertw(j == 1 - i++);
      }
    }
    assertw(!s.contains(2));
    assertw(s.pop() == 1);
    assertw(s.pop() == 0);
    assertw(s.empty());
  }
  {
    Stack<int> s;
    s.push(0);
    s.push(1);
    s.push(2);
    int i = 0;
    for (int j : s) {
      assertw(j == 2 - i++);
    }
    assertx(i == 3);
    assertw(s.pop() == 2);
    assertw(s.pop() == 1);
    assertw(s.pop() == 0);
    assertx(s.empty());
  }
  {
    Stack<float> s;
    s.push(9);
    s.push(4);
    s.push(1);
    int i = 0;
    for (float v : s) {
      assertw(v == square(i + 1));
      i++;
    }
    assertx(i == 3);
    assertw(s.pop() == 1);
    assertw(s.pop() == 4);
    assertw(s.pop() == 9);
    assertx(s.empty());
  }
  {
    SHOW("beg");
    Stack<unique_ptr<ST>> stack;
    stack.push(make_unique<ST>(4));
    stack.push(make_unique<ST>(5));
    stack.push(make_unique<ST>(6));
    for (auto& e : stack) {
      SHOW(e->_i);
    }
    for (auto& e : stack) e = nullptr;  // otherwise, ~Stack() may destroy elements in unknown order
    SHOW("end");
  }
}

namespace hh {
template class Stack<unsigned>;
template class Stack<double>;
template class Stack<const int*>;

using U = unique_ptr<int>;
template <> void Stack<U>::push(const U&) {}  // non-&& definition illegal
template class Stack<U>;
}  // namespace hh
