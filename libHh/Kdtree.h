// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_KDTREE_H_
#define MESH_PROCESSING_LIBHH_KDTREE_H_

#include "libHh/Array.h"
#include "libHh/RangeOp.h"
#include "libHh/SGrid.h"
#include "libHh/Stack.h"
#include "libHh/Stat.h"
#include "libHh/Vec.h"

namespace hh {

// A k-D tree is a spatial structure that splits space using hyperplanes in successive dimensions.
// In this implementation, the original bounding volume is the unit cube, and the hyperplane always splits
//  the current dimension at the midpoint of the current range.
// Each element is represented using a bounding box.
// When an element is entered into the tree, it is pushed down the tree until the box straddles the two child nodes.
// Large elements may be pushed down both children if the allow_duplication threshold is set.
template <typename T, int D> class Kdtree : noncopyable {
  using type = Kdtree<T, D>;

 public:
  explicit Kdtree(int maxlevel = 8) : _maxlevel(maxlevel) { constructor_i(); }
  ~Kdtree() { clear(); }
  void clear() { clear_i(); }
  void allow_duplication(float fsize) {
    if (!getenv("KD_FSIZE")) _fsize = fsize;
  }
  // bb0 and bb1 are copied internally
  void enter(const T& id, const Vec<float, D>& bb0, const Vec<float, D>& bb1) { enter_i(id, bb0, bb1); }
  void enter(T&& id, const Vec<float, D>& bb0, const Vec<float, D>& bb1) { enter_i(std::move(id), bb0, bb1); }
  // search is reentrant (for HiddenLineRemoval).
  // Start searching at loc (default _root) for objects whose bb intersect the one given.
  // For each object found, call cbfunc with id, bb, and current location in the tree.
  //  cbfunc may modify the bb by shrinking it.
  enum class ECallbackReturn { nothing, bbshrunk, stop };
  using CBloc = int;
  // ret: was_stopped
  template <typename Func = ECallbackReturn(const T& id, Vec<float, D>& bb0, Vec<float, D>& bb1, CBloc floc)>
  bool search(Vec<float, D>& bb0, Vec<float, D>& bb1, Func cbfunc, CBloc loc = 0) const {
    return search_i(bb0, bb1, cbfunc, loc);
  }
  void print() const { rec_print((!_arnode.num() ? -1 : 0), 0); }

 private:
  const int _maxlevel;  // maximum # of subdivision on each axis
  float _fsize{0.f};    // ok to duplicate if average edge length < _fsize
  struct Entry {
    T _id;
    SGrid<float, 2, D> _bb{};  // bounding box on entry
  };
  struct Node {
    int _axis;  // 0 .. D - 1
    float _val;
    Stack<int> _stackei{};  // Entry indices
    int _l{-1};             // lower-valued subtree
    int _h{-1};             // higher-valued subtree
  };
  Array<Entry> _arentry;
  Array<Node> _arnode;
  //
  void constructor_i() {
    assertx(_maxlevel > 0);
    _fsize = getenv_float("KD_FSIZE", _fsize);
  }
  void clear_i() {
    if (getenv_bool("KD_STATS") && _arnode.num()) {
      HH_STAT(SKDdepth);
      rec_depth(0, SKDdepth, 0);
    }
    _arentry.clear();
    _arnode.clear();
  }
  void rec_depth(int ni, Stat& stat, int depth) const {
    if (ni < 0) return;
    const Node& n = _arnode[ni];
    stat.enter_multiple(float(depth), n._stackei.height());
    rec_depth(n._l, stat, depth + 1);
    rec_depth(n._h, stat, depth + 1);
  }
  void enter_i(const T& id, const Vec<float, D>& bb0, const Vec<float, D>& bb1) {
    _arentry.push(Entry{id});
    enter_aux(bb0, bb1);
  }
  void enter_i(T&& id, const Vec<float, D>& bb0, const Vec<float, D>& bb1) {
    _arentry.push(Entry{std::move(id)});
    enter_aux(bb0, bb1);
  }
  void enter_aux(const Vec<float, D>& bb0, const Vec<float, D>& bb1) {
    int ei = _arentry.num() - 1;
    Entry& e = _arentry[ei];
    e._bb[0] = bb0;
    e._bb[1] = bb1;
    Vec<float, D> aval;
    fill(aval, .5f);
    float avgel = 0.f;
    if (_fsize) {
      for_int(i, D) avgel += e._bb[1][i] - e._bb[0][i];
      avgel /= D;
    }
    rec_enter(ei, 0, aval, 0, .5f, 0, avgel);
  }
  void rec_enter(int ei, int ni, Vec<float, D>& aval, int level, float inc, int axis, float avgel) {
    ASSERTX(axis >= 0 && axis < D);
    const Entry& e = _arentry[ei];
    for (;;) {
      const float val = aval[axis];
      if (ni == _arnode.num()) _arnode.push(Node{axis, val});
      if (!axis) {
        if (++level == _maxlevel) break;
        inc *= .5f;
      }
      bool want_l = e._bb[0][axis] <= val;
      bool want_h = e._bb[1][axis] >= val;
      if (want_l && want_h) {                         // single recursion
        if (!_fsize || avgel >= inc * _fsize) break;  // small enough
        {
          Vec<float, D> naval = aval;
          naval[axis] -= inc;
          if (_arnode[ni]._l < 0) _arnode[ni]._l = _arnode.num();
          rec_enter(ei, _arnode[ni]._l, naval, level, inc, (axis < D - 1 ? axis + 1 : 0), avgel);
        }
        if (_arnode[ni]._h < 0) _arnode[ni]._h = _arnode.num();
        ni = _arnode[ni]._h;
        aval[axis] += inc;
      } else if (want_l) {
        if (_arnode[ni]._l < 0) _arnode[ni]._l = _arnode.num();
        ni = _arnode[ni]._l;
        aval[axis] -= inc;
      } else if (want_h) {
        if (_arnode[ni]._h < 0) _arnode[ni]._h = _arnode.num();
        ni = _arnode[ni]._h;
        aval[axis] += inc;
      } else {
        assertnever("");
      }
      axis = axis < D - 1 ? axis + 1 : 0;
    }
    _arnode[ni]._stackei.push(ei);
  }
  template <typename Func> bool search_i(Vec<float, D>& bb0, Vec<float, D>& bb1, Func cbfunc, int ni) const {
    if (!_arnode.num()) return false;
    int nelemvis = 0;
    bool ret = rec_search(ni, ni, bb0, bb1, cbfunc, nelemvis);
    if (0) {  // not threadsafe
      static const bool b_stats = getenv_bool("KD_STATS");
      static Stat SKDsearchnel("SKDsearchnel", b_stats);
      if (b_stats) SKDsearchnel.enter(nelemvis);
    }
    return ret;
  }
  // nlca == lowest common ancestor
  template <typename Func>
  bool rec_search(int ni, int nlca, Vec<float, D>& bb0, Vec<float, D>& bb1, Func cbfunc, int& nelemvis) const {
    for (;;) {
      const Node& n = _arnode[ni];
      for (int ei : n._stackei) {
        const Entry& e = _arentry[ei];
        nelemvis++;
        bool overlaps = true;
        for_int(i, D) {
          // SHOW(bb0[i], bb1[i], e._bb[0][i], e._bb[1][i]);
          if (e._bb[0][i] >= bb1[i] || e._bb[1][i] <= bb0[i]) {
            overlaps = false;
            break;
          }
        }
        if (!overlaps) continue;
        if (cbfunc(e._id, bb0, bb1, nlca) == ECallbackReturn::stop) return true;
      }
      const int axis = n._axis;
      ASSERTX(axis >= 0 && axis < D);
      const float val = n._val;
      bool want_l = n._l >= 0 && bb0[axis] < val;
      bool want_h = n._h >= 0 && bb1[axis] > val;
      if (want_l && want_h) {  // single recursion
        if (rec_search(n._h, nlca, bb0, bb1, cbfunc, nelemvis)) return true;
        if (!(bb0[axis] < val)) return false;  // test again because bb may have changed
        ni = n._l;
      } else if (want_l) {
        if (nlca == ni) nlca = n._l;
        ni = n._l;
      } else if (want_h) {
        if (nlca == ni) nlca = n._h;
        ni = n._h;
      } else {
        return false;
      }
    }
  }
  void rec_print(int ni, int l) const {
    for_int(i, l) std::cerr << " ";
    if (ni < 0) {
      std::cerr << "<nil>\n";
      return;
    }
    const Node& n = _arnode[ni];
    std::cerr << sform("partition of axis %d along %g <<\n", n._axis, n._val);
    for (int ei : n._stackei) {
      const Entry& e = _arentry[ei];
      for_int(i, l) std::cerr << " ";
      std::cerr << e._id << "\n";
    }
    for_int(i, l) std::cerr << " ";
    std::cerr << ">>\n";
    rec_print(n._l, l + 1);
    rec_print(n._h, l + 1);
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_KDTREE_H_
