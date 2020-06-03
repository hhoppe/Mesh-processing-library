// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "EList.h"
using namespace hh;

int main() {
  {
    struct A {
      A(int i) : _i(i) {}
      int _i;
      EListNode _node;
    };
    EList list;
    A a1(1);
    a1._node.link_after(list.delim());
    A a2(2);
    a2._node.link_after(&a1._node);
    int count = 0;
    for (EListNode* node : list) {
      dummy_use(node);
      count++;
    }
    SHOW(count);
#if 0
    // Creating a member function (templated on the class Struct and offset) did not work on win,
    //  because it was not able to parse the offsetof() function as a template argument.
    // Instead, creating a templated subclass worked.
    for (auto pa : list.outer_range<A, 8>()) SHOW(pa->_i);  // works
    for (auto pa : list.outer_range<A, offsetof(A, _node)>()) SHOW(pa->_i);  // fails on win
    for (auto pa : list.outer_range<A, (offsetof(A, _node))>()) SHOW(pa->_i);  // fails on win
    const size_t off = offsetof(A, _node);
    for (auto pa : list.outer_range<A, off>()) SHOW(pa->_i);  // works
    for (auto pa : EList::OuterRange<A, off>{list}) SHOW(pa->_i);  // works
    for (auto pa : EList::OuterRange<A, offsetof(A, _node)>{list}) SHOW(pa->_i);  // works
#endif
    SHOW("2");
    for (A* pa : EList_outer_range(list, A, _node)) {
      SHOW(pa->_i);
    }
    a2._node.relink_before(&a1._node);
    SHOW("relink a2");
    for (A* pa : EList_outer_range(list, A, _node)) {
      SHOW(pa->_i);
    }
    a1._node.unlink();
    SHOW("unlink a1");
    for (A* pa : EList_outer_range(list, A, _node)) {
      SHOW(pa->_i);
    }
    a2._node.unlink();
    SHOW("unlink a2");
    for (A* pa : EList_outer_range(list, A, _node)) {
      SHOW(pa->_i);
    }
  }
}
