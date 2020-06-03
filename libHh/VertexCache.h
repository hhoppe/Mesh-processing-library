// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_VERTEXCACHE_H_
#define MESH_PROCESSING_LIBHH_VERTEXCACHE_H_

#include "Array.h"
#include "EList.h"
#include "RangeOp.h"

namespace hh {

// Keep track of vertex references during traversal of mesh faces using FIFO or LRU caching strategies.
class VertexCache : noncopyable {
 public:
  enum class EType { notype, fifo, lru };
  static string type_string(EType type);
  static unique_ptr<VertexCache> make(EType type, int nverts1, int cs);  // vertex ids begin at 1

  virtual ~VertexCache() {}
  virtual EType type() const = 0;
  virtual void init(int nverts1, int cs) = 0;    // vertex ids begin at 1
  virtual void copy(const VertexCache& vc) = 0;  // must be init'ed !
  virtual bool access_hits(int vi) = 0;
  virtual bool contains(int vi) const = 0;
  virtual int location(int vi) const = 0;      // -1 .. cs - 1 (0 == recent) (may be slow)
  virtual int location_alt(int vi) const = 0;  // 0 .. cs (0 == recent) (may be slow)
  class Iter : noncopyable {                   // ordering of vertices undefined!
   public:
    virtual ~Iter() {}
    virtual int next() = 0;  // return 0 if no more vertices
   protected:
    Iter() = default;
  };
  virtual unique_ptr<Iter> make_iterator() const = 0;
  friend std::ostream& operator<<(std::ostream& os, const VertexCache& vc);
};

//----------------------------------------------------------------------------

class FifoVertexCache : public VertexCache {
 public:
  explicit FifoVertexCache(int nverts1, int cs) { init(nverts1, cs); }
  EType type() const override { return EType::fifo; }
  void init(int nverts1, int cs) override {
    assertx(nverts1 > 0 && cs >= 1);
    if (_vinqueue.num() != nverts1 || _queuev.num() != cs) {
      assertx(!_vinqueue.num());
      _vinqueue.init(nverts1, -1);
      _queuev.init(cs, int{k_no_entry});  // int{} cast to avoid taking (const int&) on the constexpr, for clang.
    } else {
      int* q = _queuev.data();
      for_int(qi, _queuev.num()) {
        int vif = q[qi];
        q[qi] = k_no_entry;
        ASSERTX(vif == k_no_entry || _vinqueue[vif] == qi);
        _vinqueue[vif] = -1;  // vif == k_no_entry is OK
      }
    }
    _iprev = 0;
  }
  void copy(const VertexCache& pvc) override {  // pvc must be a FifoVertexCache!
    const FifoVertexCache& vc = static_cast<const FifoVertexCache&>(pvc);
    ASSERTX(vc.type() == type() && vc._vinqueue.num() == _vinqueue.num() && vc._queuev.num() == _queuev.num());
    if (0) {
      for_int(qi, _queuev.num()) {  // copied from init()
        int vif = _queuev[qi];
        ASSERTX(vif == k_no_entry || _vinqueue[vif] == qi);
        _vinqueue[vif] = -1;  // vif == k_no_entry is OK
      }
      for_int(qi, _queuev.num()) {
        int vif = vc._queuev[qi];
        _vinqueue[vif] = qi;  // vif == k_no_entry is OK
        _queuev[qi] = vif;
      }
      _iprev = vc._iprev;
    } else {
      int* q = _queuev.data();
      int* vinq = _vinqueue.data();
      const int* vcq = vc._queuev.data();
      for_int(qi, _queuev.num()) {
        int vif = q[qi];
        ASSERTX(vif == k_no_entry || _vinqueue[vif] == qi);
        vinq[vif] = -1;
      }
      for_int(qi, _queuev.num()) {
        int vif = vcq[qi];
        q[qi] = vif;
        vinq[vif] = qi;
      }
      _iprev = vc._iprev;
    }
  }
  bool access_hits(int vi) override {
    ASSERTX(vi >= 1 && vi < _vinqueue.num());
    if (0) {
      if (_vinqueue[vi] >= 0) return true;  // if there, do nothing
      if (_iprev == 0) {
        if (!_queuev.num()) return false;
        _iprev = _queuev.num();
      }
      --_iprev;
      int vj = _queuev[_iprev];  // oldest vertex
      ASSERTX(vj == k_no_entry || _vinqueue[vj] == _iprev);
      _vinqueue[vj] = -1;  // vj == k_no_entry is OK
      _queuev[_iprev] = vi;
      _vinqueue[vi] = _iprev;
    } else {
      int* q = _queuev.data();
      int* vinq = _vinqueue.data();
      if (vinq[vi] >= 0) return true;  // if there, do nothing
      _iprev = _iprev - 1 + (_iprev == 0) * _queuev.num();
      int vj = q[_iprev];  // oldest vertex
      q[_iprev] = vi;
      ASSERTX(vj == k_no_entry || vinq[vj] == _iprev);
      vinq[vj] = -1;  // vj == k_no_entry is OK
      vinq[vi] = _iprev;
    }
    return false;
  }
  bool contains(int vi) const override {
    ASSERTX(vi >= 1 && vi < _vinqueue.num());
    return _vinqueue[vi] >= 0;
  }
  int location(int vi) const override {
    ASSERTX(vi >= 1 && vi < _vinqueue.num());
    int qi = _vinqueue[vi];
    if (qi < 0) return -1;
    ASSERTX(_queuev[qi] == vi);
    int loc = qi - _iprev;
    if (loc < 0) loc += _queuev.num();
    return loc;
  }
  int location_alt(int vi) const override {
    ASSERTX(vi >= 1 && vi < _vinqueue.num());
    int qi = _vinqueue[vi];
    if (qi < 0) return _queuev.num();
    ASSERTX(_queuev[qi] == vi);
    int loc = qi - _iprev;
    if (loc < 0) loc += _queuev.num();
    return loc;
  }
  unique_ptr<VertexCache::Iter> make_iterator() const override { return make_unique<Iter>(*this); }

 private:
  class Iter : public VertexCache::Iter {
   public:
    Iter(const FifoVertexCache& vcache) : _qv(vcache._queuev), _qi(vcache._iprev), _numi(vcache._queuev.num()) {}
    int next() override {
      if (!_numi) return 0;
      --_numi;
      int vi = _qv[_qi];
      _qi++;
      if (_qi == _qv.num()) _qi = 0;
      return vi;  // vi == k_no_entry is OK
    }

   private:
    const Array<int>& _qv;
    int _qi;
    int _numi;
  };
  static constexpr int k_no_entry = 0;
  Array<int> _queuev;    // dec. circ. FIFO queue, may contain k_no_entry's
  int _iprev{-1};        // 0 .. cs - 1: entry_last_added
  Array<int> _vinqueue;  // vid -> [0 .. cs - 1, -1];  _vinqueue[k_no_entry = 0] is trash
};

class LruVertexCache : public VertexCache {
 public:
  explicit LruVertexCache(int nverts1, int cs) { init(nverts1, cs); }
  ~LruVertexCache() {
    while (!_list.empty()) {
      EListNode* nodee = _list.delim()->next();
      nodee->unlink();
    }
  }
  EType type() const override { return EType::lru; }
  void init(int nverts1, int cs) override {
    assertx(nverts1 > 0 && cs >= 1);
    if (_vinlist.num() != nverts1 || _cs != cs) {
      assertx(!_vinlist.num());
      _vinlist.init(nverts1, nullptr);
      _cs = cs;
      _nodes.init(cs);
      for_int(ci, cs) {
        Node* node = &_nodes[ci];
        node->elist.link_after(_list.delim());
      }
    } else {
      for_int(ci, cs) {
        Node* node = &_nodes[ci];
        int vif = node->vert;
        ASSERTX(vif == k_no_entry || _vinlist[vif] == node);
        _vinlist[vif] = nullptr;  // vif == k_no_entry is OK
        node->vert = k_no_entry;
      }
    }
  }
  void copy(const VertexCache& pvc) override {  // pvc must be a LruVertexCache!
    const LruVertexCache& vc = static_cast<const LruVertexCache&>(pvc);
    ASSERTX(vc.type() == type() && vc._vinlist.num() == _vinlist.num() && vc._cs == _cs);
    init(vc._vinlist.num(), vc._cs);
    EListNode* onodee = vc._list.delim()->next();
    EListNode* tnodee = _list.delim()->next();
    EListNode* tdelime = _list.delim();
    while (tnodee != tdelime) {
      Node* onode = EListOuter(Node, elist, onodee);
      Node* tnode = EListOuter(Node, elist, tnodee);
      int vif = onode->vert;
      tnode->vert = vif;
      _vinlist[vif] = tnode;  // vif == k_no_entry is OK
      tnodee = tnodee->next();
      onodee = onodee->next();
    }
    ASSERTX(onodee == vc._list.delim());
  }
  bool access_hits(int vi) override {
    ASSERTX(vi >= 1 && vi < _vinlist.num());
    Node* node = _vinlist[vi];
    if (node) {  // if there, move to front
      ASSERTX(node->vert == vi);
      node->elist.relink_after(_list.delim());
      return true;
    }
    EListNode* nodee = _list.delim()->prev();  // rear node
    // if (nodee == _list.delim()) return false;  // cs == 0
    node = EListOuter(Node, elist, nodee);
    int vj = node->vert;
    ASSERTX(vj == k_no_entry || _vinlist[vj] == node);
    _vinlist[vj] = nullptr;  // vj == k_no_entry is OK
    nodee->relink_after(_list.delim());
    node->vert = vi;
    _vinlist[vi] = node;
    return false;
  }
  bool contains(int vi) const override {
    ASSERTX(vi >= 1 && vi < _vinlist.num());
    return !!_vinlist[vi];
  }
  int location(int vi) const override {
    ASSERTX(vi >= 1 && vi < _vinlist.num());
    if (!_vinlist[vi]) return -1;
    const EListNode* vinodee = &_vinlist[vi]->elist;
    const EListNode* delime = _list.delim();
    int num = 0;
    for (EListNode* nodee = delime->next(); nodee != vinodee; nodee = nodee->next()) {
      num++;
    }
    ASSERTX(num < _cs);
    return num;
  }
  int location_alt(int vi) const override {
    ASSERTX(vi >= 1 && vi < _vinlist.num());
    if (!_vinlist[vi]) return _cs;
    const EListNode* vinodee = &_vinlist[vi]->elist;
    const EListNode* delime = _list.delim();
    int num = 0;
    for (EListNode* nodee = delime->next(); nodee != vinodee; nodee = nodee->next()) {
      num++;
    }
    ASSERTX(num < _cs);
    return num;
  }
  unique_ptr<VertexCache::Iter> make_iterator() const override { return make_unique<Iter>(*this); }

 private:
  class Iter : public VertexCache::Iter {
   public:
    Iter(const LruVertexCache& vcache) : _delim(vcache._list.delim()), _n(vcache._list.delim()->next()) {}
    int next() override {
      if (_n == _delim) return 0;
      int vi = EListOuter(LruVertexCache::Node, elist, _n)->vert;
      _n = _n->next();
      return vi;  // vi == k_no_entry is OK
    }

   private:
    const EListNode* _delim;
    const EListNode* _n;
  };
  struct Node {
    EListNode elist;
    int vert{k_no_entry};  // k_no_entry if cache entry is empty
  };
  static constexpr int k_no_entry = 0;
  int _cs{0};
  Array<Node> _nodes;  // Note: Node is noncopyable but Array is never resized
  EList _list;
  Array<Node*> _vinlist;  // _vinlist[k_no_entry = 0] is trash
};

//----------------------------------------------------------------------------

inline string VertexCache::type_string(EType type) {
  if (type == EType::fifo) return "Fifo";
  if (type == EType::lru) return "Lru";
  assertnever("Cache type not specified");
}

inline unique_ptr<VertexCache> VertexCache::make(EType type, int nverts1, int cs) {
  if (type == EType::fifo) return make_unique<FifoVertexCache>(nverts1, cs);
  if (type == EType::lru) return make_unique<LruVertexCache>(nverts1, cs);
  assertnever("Cache type not specified");
}

inline std::ostream& operator<<(std::ostream& os, const VertexCache& vc) {
  auto up_vci = vc.make_iterator();
  VertexCache::Iter& vci = *up_vci;
  os << "{";
  for (;;) {
    int vi = vci.next();
    if (!vi) break;
    os << sform(" %d", vi);
  }
  return os << "}";
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_VERTEXCACHE_H_
