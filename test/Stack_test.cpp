// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Stack.h"

#include <vector>

#include "libHh/Random.h"
using namespace hh;

int main() {
  struct S {
    explicit S(int i) : _i(i) { showf("S(%d)\n", _i); }
    ~S() { showf("~S(%d)\n", _i); }
    int _i;
  };
  {
    const S s1(1), s2(2), s3(3);  // The stack holds non-owning pointers to these.
    Stack<const S*> s;
    assertx(s.empty());
    s.push(&s1);
    s.push(&s2);
    s.push(&s3);
    SHOW(s.top()->_i);
    assertx(!s.empty());
    assertx(s.pop()->_i == 3);
    assertx(s.pop()->_i == 2);
    assertx(s.pop()->_i == 1);
    assertx(s.empty());
  }
  {
    Stack<int> s;
    assertx(s.empty());
    for (const int i : s) {
      dummy_use(i);
      if (1) assertnever("");
    }
    for_int(i, 4) s.push(i);
    assertx(s.height() == 4);
    assertx(!s.empty());
    assertx(s.contains(2));
    assertx(s.top() == 3);
    assertx(s.pop() == 3);
    assertx(s.pop() == 2);
    {
      int i = 0;
      for (const int j : s) assertx(j == 1 - i++);
    }
    assertx(!s.contains(2));
    assertx(s.pop() == 1);
    assertx(s.pop() == 0);
    assertx(s.empty());
  }
  {
    Stack<int> s;
    s.push(0);
    s.push(1);
    s.push(2);
    int i = 0;
    for (const int j : s) assertx(j == 2 - i++);
    assertx(i == 3);
    assertx(s.pop() == 2);
    assertx(s.pop() == 1);
    assertx(s.pop() == 0);
    assertx(s.empty());
  }
  {
    Stack<float> s;
    s.push(9);
    s.push(4);
    s.push(1);
    int i = 0;
    for (const float v : s) {
      assertx(v == square(i + 1));
      i++;
    }
    assertx(i == 3);
    assertx(s.pop() == 1);
    assertx(s.pop() == 4);
    assertx(s.pop() == 9);
    assertx(s.empty());
  }
  {
    SHOW("beg");
    Stack<unique_ptr<S>> stack;
    stack.push(make_unique<S>(4));
    stack.push(make_unique<S>(5));
    stack.push(make_unique<S>(6));
    for (auto& e : stack) SHOW(e->_i);
    for (auto& e : stack) e = nullptr;  // Otherwise, ~Stack() may destroy elements in unknown order.
    SHOW("end");
  }
  {
    // The output lists the elements from top to bottom, like the iteration.
    Stack<int> s;
    for_int(i, 3) s.push(i * 10);
    SHOW(s);
    assertx(s.size() == 3 && s.height() == 3);
    s.clear();
    assertx(s.empty() && s.height() == 0);
    assertx(s.begin() == s.end());
  }
  {
    // remove() deletes the bottom-most occurrence, keeping the order of the remaining elements.
    Stack<int> s;
    for (const int i : {1, 2, 3, 2, 4}) s.push(i);
    assertx(s.remove(2));
    SHOW(s);
    assertx(s.remove(4));  // The top element.
    assertx(s.top() == 2);
    assertx(!s.remove(9));
    assertx(s.remove(1));  // The bottom element.
    assertx(s.remove(2) && s.remove(3) && s.empty());
    assertx(!s.remove(3));
  }
  {
    // Copies are independent.
    Stack<string> s1;
    s1.push("a");
    string str = "b";
    s1.push(str);  // The const& overload copies.
    assertx(str == "b");
    Stack<string> s2 = s1;
    s2.push("c");
    assertx(s1.height() == 2 && s1.top() == "b" && s2.height() == 3 && s2.top() == "c");
    assertx(s2.contains("a") && !s1.contains("c"));
    static_assert(std::is_same_v<decltype(*s1.begin()), string&>);
    static_assert(std::is_same_v<decltype(*std::as_const(s1).begin()), const string&>);
  }
  {
    // The std::vector helper functions.
    std::vector<int> vec{5, 6, 5, 7};
    assertx(vec_remove_ordered(vec, 5));  // Removes only the first occurrence.
    assertx(!vec_remove_ordered(vec, 8));
    assertx(vec == (std::vector<int>{6, 5, 7}));
    assertx(vec_pop(vec) == 7);
    assertx(vec_pop(vec) == 5);
    assertx(vec == std::vector<int>{6});
    std::vector<unique_ptr<int>> vecp;
    vecp.push_back(make_unique<int>(3));
    const unique_ptr<int> p = vec_pop(vecp);
    assertx(*p == 3 && vecp.empty());
  }
  {
    // Random operations compared against std::vector as a reference model, whose back is the top of the stack.
    Random random{5};
    Stack<int> s;
    std::vector<int> vec;
    for_int(iter, 2000) {
      const unsigned op = random.get_unsigned(10);
      const int value = int(random.get_unsigned(20));
      if (op < 5) {
        s.push(value), vec.push_back(value);
      } else if (op < 8) {
        if (!vec.empty()) assertx(s.pop() == vec_pop(vec));
      } else if (op < 9) {
        assertx(s.remove(value) == vec_remove_ordered(vec, value));
      } else {
        assertx(s.contains(value) == (ranges::find(vec, value) != vec.end()));
      }
      assertx(s.height() == int(vec.size()) && s.empty() == vec.empty());
      if (!vec.empty()) assertx(s.top() == vec.back());
      assertx(ranges::equal(s, vec | views::reverse));
    }
  }
}

template class hh::Stack<unsigned>;
template class hh::Stack<double>;
template class hh::Stack<const int*>;
template class hh::Stack<unique_ptr<int>>;
