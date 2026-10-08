// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/IntrusiveList.h"

#include "libHh/Array.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

struct A {
  A() = default;
  explicit A(int i) : _i(i) {}
  int _i{0};
  IntrusiveListNode _node;  // Not the first member, so that the offset to the outer struct is nonzero.
};

// Returns the values of the elements of the list, in order, after verifying the list's links in both directions.
Array<int> values(const IntrusiveList& list) {
  Array<int> ar;
  for (const IntrusiveListNode* node : list) {
    node->ok();
    assertx(node->linked() && node->next()->prev() == node && node->prev()->next() == node);
  }
  for (const A* pa : HH_INTRUSIVE_LIST_RANGE(list, A, _node)) ar.push(pa->_i);
  Array<int> ar_reverse;
  for (const A* pa : HH_INTRUSIVE_LIST_RANGE(list, A, _node) | views::reverse) ar_reverse.push(pa->_i);
  assertx(ar_reverse == reverse(Array<int>(ar)));
  assertx(list.empty() == (ar.num() == 0));
  return ar;
}

}  // namespace

int main() {
  {
    IntrusiveList list;
    A a1(1);
    a1._node.link_after(list.delim());
    A a2(2);
    a2._node.link_after(&a1._node);
    int count = 0;
    for (const IntrusiveListNode* node : list) {
      dummy_use(node);
      count++;
    }
    SHOW(count);
    // HH_INTRUSIVE_LIST_RANGE() constructs the class template OuterRange rather than calling a member function
    // template, because MSVC could not parse offsetof() as an explicit template argument of a member function call.
    SHOW("2");
    for (A* pa : HH_INTRUSIVE_LIST_RANGE(list, A, _node)) SHOW(pa->_i);
    a2._node.relink_before(&a1._node);
    SHOW("relink a2");
    for (A* pa : HH_INTRUSIVE_LIST_RANGE(list, A, _node)) SHOW(pa->_i);
    a1._node.unlink();
    SHOW("unlink a1");
    for (A* pa : HH_INTRUSIVE_LIST_RANGE(list, A, _node)) SHOW(pa->_i);
    a2._node.unlink();
    SHOW("unlink a2");
    for (A* pa : HH_INTRUSIVE_LIST_RANGE(list, A, _node)) SHOW(pa->_i);
  }
  {
    // The iterators are bidirectional, and an Iter converts to a ConstIter but not the reverse.
    static_assert(std::bidirectional_iterator<IntrusiveList::Iter>);
    static_assert(std::bidirectional_iterator<IntrusiveList::ConstIter>);
    static_assert(std::convertible_to<IntrusiveList::Iter, IntrusiveList::ConstIter>);
    static_assert(!std::convertible_to<IntrusiveList::ConstIter, IntrusiveList::Iter>);
    static_assert(ranges::bidirectional_range<IntrusiveList> && ranges::bidirectional_range<const IntrusiveList>);
    static_assert(std::is_same_v<ranges::range_value_t<const IntrusiveList>, const IntrusiveListNode*>);
    using Range = decltype(HH_INTRUSIVE_LIST_RANGE(std::declval<IntrusiveList&>(), A, _node));
    static_assert(ranges::view<Range> && ranges::bidirectional_range<Range>);
    static_assert(std::is_same_v<ranges::range_value_t<Range>, A*>);
  }
  {
    // An empty list, and a node that is not linked.
    IntrusiveList list;
    assertx(list.empty());
    assertx(list.begin() == list.end());
    assertx(list.delim()->next() == list.delim() && list.delim()->prev() == list.delim());
    assertx(values(list).num() == 0);
    const A a(0);
    assertx(!a._node.linked());
    a._node.ok();  // An unlinked node is trivially consistent.
  }
  {
    // link_before(delim) appends (like a queue), and link_after(delim) prepends (like a stack).
    IntrusiveList list;
    A a1(1), a2(2), a3(3), a4(4);
    a1._node.link_before(list.delim());
    a2._node.link_before(list.delim());
    a3._node.link_after(list.delim());
    SHOW(values(list));
    assertx(!list.empty());
    a4._node.link_after(&a1._node);  // Insert in the middle.
    assertx(values(list) == V(3, 1, 4, 2).view());
    // HH_INTRUSIVE_LIST_OUTER() recovers the struct from a node.
    assertx(HH_INTRUSIVE_LIST_OUTER(A, _node, list.delim()->next()) == &a3);
    assertx(HH_INTRUSIVE_LIST_OUTER(A, _node, list.delim()->prev())->_i == 2);
    // relink_after() and relink_before() move an element within the list.
    a3._node.relink_after(&a2._node);  // To the end.
    assertx(values(list) == V(1, 4, 2, 3).view());
    a2._node.relink_before(&a1._node);  // To the front.
    assertx(values(list) == V(2, 1, 4, 3).view());
    a4._node.relink_before(list.delim());  // To the end, using the delimiter.
    assertx(values(list) == V(2, 1, 3, 4).view());
    a4._node.relink_after(&a3._node);  // Relinking at the same position leaves the list unchanged.
    assertx(values(list) == V(2, 1, 3, 4).view());
    // The ConstIter of a const list traverses the same nodes, also in reverse.
    const IntrusiveList& clist = list;
    assertx(*clist.begin() == &a2._node && *--clist.end() == &a4._node);
    IntrusiveList::ConstIter citer = list.begin();  // Conversion from Iter.
    assertx(*++citer == &a1._node && *citer++ == &a1._node && *citer == &a3._node);
    assertx(*--citer == &a1._node);
    // Unlinking makes a node reusable, here in a second list.
    a1._node.unlink();
    assertx(!a1._node.linked() && values(list) == V(2, 3, 4).view());
    IntrusiveList list2;
    a1._node.link_after(list2.delim());
    assertx(values(list2) == V(1).view());
    for (A* pa : {&a1, &a2, &a3, &a4}) pa->_node.unlink();  // Lists must be empty when destroyed.
    assertx(list.empty() && list2.empty());
  }
  {
    // A struct may be in two lists at once through two nodes.
    struct B {
      IntrusiveListNode _node_all;
      int _i{0};
      IntrusiveListNode _node_odd;
    };
    IntrusiveList list_all, list_odd;
    Array<B> bs(5);
    for_int(i, bs.num()) {
      B& b = bs[i];
      b._i = i;
      b._node_all.link_before(list_all.delim());
      if (i % 2) b._node_odd.link_after(list_odd.delim());
    }
    Array<int> all, odd;
    for (const B* pb : HH_INTRUSIVE_LIST_RANGE(list_all, B, _node_all)) all.push(pb->_i);
    for (const B* pb : HH_INTRUSIVE_LIST_RANGE(list_odd, B, _node_odd)) odd.push(pb->_i);
    SHOW(all, odd);
    // Modify the elements through the range.
    for (B* pb : HH_INTRUSIVE_LIST_RANGE(list_odd, B, _node_odd)) pb->_i *= 10;
    assertx(bs[3]._i == 30 && bs[2]._i == 2);
    for (B& b : bs) {
      b._node_all.unlink();
      if (b._node_odd.linked()) b._node_odd.unlink();
    }
  }
  {
    // Random operations compared against an Array as a reference model of the list order.
    Random random{11};
    constexpr int n = 8;
    Array<A> as(n);
    for_int(i, n) as[i]._i = i;
    IntrusiveList list;
    Array<int> model;
    for_int(iter, 1000) {
      const int i = int(random.get_unsigned(n));
      A& a = as[i];
      const unsigned op = random.get_unsigned(3);
      if (!a._node.linked()) {
        if (op == 0) {
          a._node.link_after(list.delim()), model.unshift(i);
        } else if (op == 1) {
          a._node.link_before(list.delim()), model.push(i);
        } else if (model.num()) {  // Insert after a random linked element.
          const int j = int(random.get_unsigned(unsigned(model.num())));
          a._node.link_after(&as[model[j]]._node), model.insert(j + 1, 1), model[j + 1] = i;
        }
      } else {
        if (op == 0) {
          a._node.unlink(), assertx(model.remove_ordered(i));
        } else {
          const int j = int(random.get_unsigned(unsigned(model.num())));
          const int other = model[j];
          if (other != i) {
            assertx(model.remove_ordered(i));
            const int jo = *find_index(model, other);
            if (op == 1) {
              a._node.relink_after(&as[other]._node), model.insert(jo + 1, 1), model[jo + 1] = i;
            } else {
              a._node.relink_before(&as[other]._node), model.insert(jo, 1), model[jo] = i;
            }
          }
        }
      }
      assertx(values(list) == model);
    }
    for (A& a : as)
      if (a._node.linked()) a._node.unlink();
  }
}
