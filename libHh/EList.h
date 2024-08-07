// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ELIST_H_
#define MESH_PROCESSING_LIBHH_ELIST_H_

#include <cstddef>  // offsetof()

#include "libHh/Hh.h"

#if 0
{
  struct SrAVertex {
    EListNode _active;
    bool ok{true};
    ...
  };
  EList list_activev;
  SrAVertex* v = new SrAVertex;
  v->_active.link_after(list_activev.delim());
  for (EListNode* n : list_activev) assertx(n->linked());
  for (SrAVertex* v : HH_ELIST_RANGE(list, SrAVertex, _active)) assertx(v->ok);
}
#endif

namespace hh {

// Define a range to iterate over a list of Struct by following EListNode named node_elem_name within it.
#define HH_ELIST_RANGE(list, Struct, node_elem_name) \
  hh::EList::OuterRange<Struct, offsetof(Struct, node_elem_name)>(list)

// Given a pointer to EListNode node_elem_name, a member of Struct, return a pointer the Struct.
#define HH_ELIST_OUTER(Struct, node_elem_name, node) \
  reinterpret_cast<Struct*>(const_cast<char*>(reinterpret_cast<const char*>(node) - offsetof(Struct, node_elem_name)))

// Implements a doubly-linked list node to embed within other struct.
// Place this node at the beginning of the struct so that the _next field is more likely in the same cache line.
class EListNode : noncopyable {
 public:
  bool linked() const { return _next != nullptr; }
  EListNode* prev() const { return _prev; }
  EListNode* next() const { return _next; }
  void unlink();
  void link_after(EListNode* n);
  void link_before(EListNode* n);
  void relink_after(EListNode* n);   // { unlink(); link_after(); }
  void relink_before(EListNode* n);  // { unlink(); link_before(); }
  void ok() const {
    if (linked()) assertx(_next->_prev == this && _prev->_next == this);
  }

 private:
  friend class EList;
  EListNode* _prev{nullptr};  // Optional initialization; performed to help clang-tidy.
  EListNode* _next{nullptr};  // Placed second, to be close to data in rest of struct.
};

// The list object which heads a list of EListNode.
// Note that a single EListNode member can be assigned to one of several mutually exclusive lists,
//  but then the EListNode by itself does not provide an efficient way to detect to which list the node belongs.
class EList {
 public:
  EList() { _delim._prev = &_delim, _delim._next = &_delim; }
  ~EList() {
    if (delim()->next() != delim()) Warning("~EList(): not empty");
  }
  EListNode* delim() { return &_delim; }
  const EListNode* delim() const { return &_delim; }
  bool empty() const { return delim()->next() == delim(); }
  // Use n->link_before(elist.delim()) as in: Array::push(), Queue::enqueue(), or std::vector::push_back().
  // Use n->link_after(elist.delim())  as in: Array::unshift() or Stack::push().
  class Iter {
    using type = Iter;

   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = EListNode;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    Iter(EListNode* node) : _node(node) {}
    bool operator==(const type& rhs) const { return _node == rhs._node; }
    bool operator!=(const type& rhs) const { return !(*this == rhs); }
    EListNode* operator*() const { return _node; }
    type& operator++();
    type& operator--();

   private:
    EListNode* _node;
  };
  class ConstIter {
    using type = ConstIter;

   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = EListNode;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    ConstIter(const EListNode* node) : _node(node) {}
    bool operator==(const type& rhs) const { return _node == rhs._node; }
    bool operator!=(const type& rhs) const { return !(*this == rhs); }
    const EListNode* operator*() const { return _node; }
    type& operator++();
    type& operator--();

   private:
    const EListNode* _node;
  };
  Iter begin() { return Iter(delim()->next()); }
  Iter end() { return Iter(delim()); }
  ConstIter begin() const { return ConstIter(delim()->next()); }
  ConstIter end() const { return ConstIter(delim()); }
  template <typename Struct, size_t offset> class OuterIter {
    using type = OuterIter;

   public:
    OuterIter(EListNode* node) : _node(node) {}
    bool operator==(const type& rhs) const { return _node == rhs._node; }
    bool operator!=(const type& rhs) const { return !(*this == rhs); }
    Struct* operator*() const { return reinterpret_cast<Struct*>(reinterpret_cast<uint8_t*>(_node) - offset); }
    type& operator++();
    type& operator--();

   private:
    EListNode* _node;
  };
  template <typename Struct, size_t offset> struct OuterRange {
    OuterRange(const EList& list) : _list(const_cast<EList&>(list)) {}  // un-const right away
    OuterIter<Struct, offset> begin() const { return OuterIter<Struct, offset>(_list.delim()->next()); }
    OuterIter<Struct, offset> end() const { return OuterIter<Struct, offset>(_list.delim()); }
    EList& _list;
  };

 private:
  EListNode _delim;
};

//----------------------------------------------------------------------------

inline void EListNode::unlink() {
  ASSERTXX(linked());
  // _prev->_next = _next; _next->_prev = _prev;
  EListNode* cp = _prev;
  EListNode* cn = _next;
  cp->_next = cn;
  cn->_prev = cp;
  _next = nullptr;
}

inline void EListNode::link_after(EListNode* n) {
  ASSERTXX(!linked());
  EListNode* nn = n->_next;
  _next = nn;
  _prev = n;
  n->_next = this;
  nn->_prev = this;
}

inline void EListNode::link_before(EListNode* n) {
  ASSERTXX(!linked());
  EListNode* np = n->_prev;
  _prev = np;
  _next = n;
  n->_prev = this;
  np->_next = this;
}

inline void EListNode::relink_after(EListNode* n) {
  ASSERTXX(linked());
  EListNode* cp = _prev;
  EListNode* cn = _next;
  cp->_next = cn;
  cn->_prev = cp;
  EListNode* nn = n->_next;
  _next = nn;
  _prev = n;
  n->_next = this;
  nn->_prev = this;
}

inline void EListNode::relink_before(EListNode* n) {
  ASSERTXX(linked());
  EListNode* cp = _prev;
  EListNode* cn = _next;
  cp->_next = cn;
  cn->_prev = cp;
  EListNode* np = n->_prev;
  _prev = np;
  _next = n;
  n->_prev = this;
  np->_next = this;
}

inline EList::Iter& EList::Iter::operator++() {
  _node = _node->next();
  return *this;
}

inline EList::Iter& EList::Iter::operator--() {
  _node = _node->prev();
  return *this;
}

inline EList::ConstIter& EList::ConstIter::operator++() {
  _node = _node->next();
  return *this;
}

inline EList::ConstIter& EList::ConstIter::operator--() {
  _node = _node->prev();
  return *this;
}

template <typename Struct, size_t offset>
EList::OuterIter<Struct, offset>& EList::OuterIter<Struct, offset>::operator++() {
  _node = _node->next();
  return *this;
}

template <typename Struct, size_t offset>
EList::OuterIter<Struct, offset>& EList::OuterIter<Struct, offset>::operator--() {
  _node = _node->prev();
  return *this;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_ELIST_H_
