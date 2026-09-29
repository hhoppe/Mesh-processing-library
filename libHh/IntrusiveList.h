// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_INTRUSIVELIST_H_
#define MESH_PROCESSING_LIBHH_INTRUSIVELIST_H_

#include <cstddef>  // offsetof()

#include "libHh/Hh.h"

#if 0
{
  struct SrAVertex {
    IntrusiveListNode _active;
    bool ok{true};
    ...
  };
  IntrusiveList list_activev;
  SrAVertex* v = new SrAVertex;
  v->_active.link_after(list_activev.delim());
  for (IntrusiveListNode* n : list_activev) assertx(n->linked());
  for (SrAVertex* v : HH_INTRUSIVE_LIST_RANGE(list, SrAVertex, _active)) assertx(v->ok);
}
#endif

namespace hh {

// Define a range to iterate over a list of Struct by following IntrusiveListNode named node_elem_name within it.
#define HH_INTRUSIVE_LIST_RANGE(list, Struct, node_elem_name) \
  hh::IntrusiveList::OuterRange<Struct, offsetof(Struct, node_elem_name)>(list)

// Given a pointer to IntrusiveListNode node_elem_name, a member of Struct, return a pointer the Struct.
#define HH_INTRUSIVE_LIST_OUTER(Struct, node_elem_name, node) \
  reinterpret_cast<Struct*>(const_cast<char*>(reinterpret_cast<const char*>(node) - offsetof(Struct, node_elem_name)))

// Implements a doubly-linked list node to embed within other struct.
// Place this node at the beginning of the struct so that the _next field is more likely in the same cache line.
class IntrusiveListNode : noncopyable {
 public:
  [[nodiscard]] bool linked() const { return _next != nullptr; }
  [[nodiscard]] IntrusiveListNode* prev() const { return _prev; }
  [[nodiscard]] IntrusiveListNode* next() const { return _next; }
  void unlink();
  void link_after(IntrusiveListNode* n);
  void link_before(IntrusiveListNode* n);
  void relink_after(IntrusiveListNode* n);   // { unlink(); link_after(); }
  void relink_before(IntrusiveListNode* n);  // { unlink(); link_before(); }
  void ok() const {
    if (linked()) assertx(_next->_prev == this && _prev->_next == this);
  }

 private:
  friend class IntrusiveList;
  IntrusiveListNode* _prev{nullptr};  // Optional initialization; performed to help clang-tidy.
  IntrusiveListNode* _next{nullptr};  // Placed second, to be close to data in rest of struct.
};

// The list object which heads a list of IntrusiveListNode.
// Note that a single IntrusiveListNode member can be assigned to one of several mutually exclusive lists,
//  but then the IntrusiveListNode by itself does not provide an efficient way to detect to which list the node
//  belongs.
class IntrusiveList {
 public:
  IntrusiveList() { _delim._prev = &_delim, _delim._next = &_delim; }
  ~IntrusiveList() {
    if (delim()->next() != delim()) Warning("~IntrusiveList(): not empty");
  }
  [[nodiscard]] IntrusiveListNode* delim() { return &_delim; }
  [[nodiscard]] const IntrusiveListNode* delim() const { return &_delim; }
  [[nodiscard]] bool empty() const { return delim()->next() == delim(); }
  // Use n->link_before(list.delim()) as in: Array::push(), Queue::enqueue(), or std::vector::push_back().
  // Use n->link_after(list.delim())  as in: Array::unshift() or Stack::push().

  // Iterator over Node, which is IntrusiveListNode for a mutable IntrusiveList and const IntrusiveListNode for a
  // const one.
  template <typename Node> struct Iterator {
    using type = Iterator;
    using iterator_concept = std::bidirectional_iterator_tag;
    using value_type = Node*;  // The operator*() yields the pointer, not the node.
    using difference_type = std::ptrdiff_t;
    explicit Iterator(Node* node) : _node(node) {}
    Iterator() = default;
    template <typename Node2> requires std::is_same_v<Node, const Node2>  // Conversion Iter to ConstIter.
    Iterator(const Iterator<Node2>& rhs) : _node(rhs._node) {}
    [[nodiscard]] bool operator==(const type& rhs) const { return _node == rhs._node; }
    [[nodiscard]] Node* operator*() const { return _node; }
    type& operator++() { return (_node = _node->next()), *this; }
    type& operator--() { return (_node = _node->prev()), *this; }
    type operator++(int) { return postfix_increment(*this); }
    type operator--(int) { return postfix_decrement(*this); }
    Node* _node{};
  };
  using Iter = Iterator<IntrusiveListNode>;
  using ConstIter = Iterator<const IntrusiveListNode>;

  [[nodiscard]] Iter begin() { return Iter(delim()->next()); }
  [[nodiscard]] Iter end() { return Iter(delim()); }
  [[nodiscard]] ConstIter begin() const { return ConstIter(delim()->next()); }
  [[nodiscard]] ConstIter end() const { return ConstIter(delim()); }
  template <typename Struct, size_t offset> struct OuterIter {
    using type = OuterIter;
    using iterator_concept = std::bidirectional_iterator_tag;
    using value_type = Struct*;
    using difference_type = std::ptrdiff_t;
    explicit OuterIter(IntrusiveListNode* node) : _node(node) {}
    OuterIter() = default;
    [[nodiscard]] bool operator==(const type& rhs) const { return _node == rhs._node; }
    [[nodiscard]] Struct* operator*() const {
      return reinterpret_cast<Struct*>(reinterpret_cast<uint8_t*>(_node) - offset);
    }
    type& operator++() { return (_node = _node->next()), *this; }
    type& operator--() { return (_node = _node->prev()), *this; }
    type operator++(int) { return postfix_increment(*this); }
    type operator--(int) { return postfix_decrement(*this); }
    IntrusiveListNode* _node{};
  };
  template <typename Struct, size_t offset> struct OuterRange : ranges::view_interface<OuterRange<Struct, offset>> {
    explicit OuterRange(const IntrusiveList& list)
        : _list(const_cast<IntrusiveList*>(&list)) {}  // Un-const right away.
    [[nodiscard]] OuterIter<Struct, offset> begin() const { return OuterIter<Struct, offset>(_list->delim()->next()); }
    [[nodiscard]] OuterIter<Struct, offset> end() const { return OuterIter<Struct, offset>(_list->delim()); }
    // Note that size() is not trivially computable.
    IntrusiveList* _list;
  };

 private:
  IntrusiveListNode _delim;
};

//----------------------------------------------------------------------------

inline void IntrusiveListNode::unlink() {
  ASSERTXX(linked());
  // _prev->_next = _next; _next->_prev = _prev;
  IntrusiveListNode* cp = _prev;
  IntrusiveListNode* cn = _next;
  cp->_next = cn;
  cn->_prev = cp;
  _next = nullptr;
}

inline void IntrusiveListNode::link_after(IntrusiveListNode* n) {
  ASSERTXX(!linked());
  IntrusiveListNode* nn = n->_next;
  _next = nn;
  _prev = n;
  n->_next = this;
  nn->_prev = this;
}

inline void IntrusiveListNode::link_before(IntrusiveListNode* n) {
  ASSERTXX(!linked());
  IntrusiveListNode* np = n->_prev;
  _prev = np;
  _next = n;
  n->_prev = this;
  np->_next = this;
}

inline void IntrusiveListNode::relink_after(IntrusiveListNode* n) {
  ASSERTXX(linked());
  IntrusiveListNode* cp = _prev;
  IntrusiveListNode* cn = _next;
  cp->_next = cn;
  cn->_prev = cp;
  IntrusiveListNode* nn = n->_next;
  _next = nn;
  _prev = n;
  n->_next = this;
  nn->_prev = this;
}

inline void IntrusiveListNode::relink_before(IntrusiveListNode* n) {
  ASSERTXX(linked());
  IntrusiveListNode* cp = _prev;
  IntrusiveListNode* cn = _next;
  cp->_next = cn;
  cn->_prev = cp;
  IntrusiveListNode* np = n->_prev;
  _prev = np;
  _next = n;
  n->_prev = this;
  np->_next = this;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_INTRUSIVELIST_H_
