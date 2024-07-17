// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_G3DOGL_SIMPLICIALCOMPLEX_H_
#define MESH_PROCESSING_G3DOGL_SIMPLICIALCOMPLEX_H_

#include "libHh/A3dStream.h"
#include "libHh/Flags.h"
#include "libHh/GMesh.h"
#include "libHh/Map.h"
#include "libHh/MeshOp.h"
#include "libHh/PArray.h"
#include "libHh/Polygon.h"
#include "libHh/Pqueue.h"
#include "libHh/Queue.h"
#include "libHh/Stack.h"
#include "libHh/Timer.h"

namespace hh {

class ISimplex;
using Simplex = ISimplex*;

class ISimplex : noncopyable {
 public:
  friend class SimplicialComplex;
  static constexpr int MAX_DIM = 2;

  // 0-dim simplex has _child[0] which is ignored
  ISimplex(int dim, int id) : _dim(dim), _id(id) { for_int(i, _dim + 1) _child[i] = nullptr; }

  void setChild(int num, Simplex child);
  void addParent(Simplex parent);
  void removeParent(Simplex parent);

  Simplex getChild(int num) const { return _child[num]; }
  CArrayView<Simplex> children() const { return _child.head(getDim() + 1); }
  const std::vector<Simplex>& getParents() const { return _parent; }
  // All descendents (>= 1 dim); iterates by generations children first, grandchildren next, etc.
  Array<Simplex> all_faces() const;
  // All ancestors (>= 1 dim); iterates by generations parents first, grandparents next, etc.
  PArray<Simplex, 20> get_star() const;
  PArray<Simplex, 20> faces_of_vertex() const;  // Faces adjacent to simplex (which must be a vertex).
  int getDim() const { return _dim; }
  int getId() const { return _id; }

  float length2() const;
  float length() const { return sqrt(length2()); }
  void polygon(Polygon& p) const;

  // 2-simplices
  void vertices(Simplex va[3]);
  Simplex opp_edge(Simplex v);
  // Simplex opp_vertex(Simplex e);

  // 1-simplices
  Simplex opp_vertex(Simplex v);

  // 0-simplices
  Simplex edgeTo(Simplex opp_v);

  // attribute mod and access function
  void setPosition(const Point& pos) { assertx(_dim == 0), _position = pos; }
  void setVAttribute(int va) { _vattribute = va; }
  void setArea(float area) { _area = area; }
  Flags& flags() {
    assertnever("no longer supported");
    return _flags;
  }

  const Point& getPosition() const { return _position; }
  int getVAttribute() const { return _vattribute; }
  float getArea() const { return _area; }
  const Flags& flags() const {
    assertnever("no longer supported");
    return _flags;
  }

  // predicates
  bool hasColor() const { return _vattribute >= 0; }
  bool isPrincipal() const { return _parent.empty(); }
  bool is_boundary() const { return _parent.size() == 1; }
  bool isManifold() const { return _parent.size() == 2; }

  // for gemorph
  const char* get_string() const { return _string.get(); }
  void set_string(const char* s) { _string = make_unique_c_string(s); }
  void update_string(const char* key, const char* val) { GMesh::update_string_ptr(_string, key, val); }

  HH_POOL_ALLOCATION(ISimplex);

 private:
  int _dim;                          // dimension of the simplex
  int _id;                           // simplex id
  Vec<Simplex, MAX_DIM + 1> _child;  // simplices it contains
  std::vector<Simplex> _parent;      // simplices it belongs to
  // Attributes
  Flags _flags;
  Point _position;      // simplices of dimension 0 only
  int _vattribute{-1};  // visual attributes
  float _area{0.f};
  // for geomorph
  unique_ptr<char[]> _string;
};

class SimplicialComplex : noncopyable {
  struct OrderedSimplices_range;

 public:
  static constexpr int MAX_DIM = ISimplex::MAX_DIM;

  SimplicialComplex() { for_int(i, MAX_DIM + 1) _free_sid[i] = 1; }
  ~SimplicialComplex() { clear(); }

  void clear();

  // I/O
  void readGMesh(std::istream& is);
  void read(std::istream& is);
  void write(std::ostream& os) const;
  void readQHull(std::istream& is);

  // modification functions
  Simplex createSimplex(int dim);
  Simplex createSimplex(int dim, int id);
  void destroySimplex(Simplex s, int area_test = 0);
  void unify(Simplex vs, Simplex vt, int propagate_area = 0);
  void copy(const SimplicialComplex& orig);
  void skeleton(int dim);

  // access (const) functions
  int num(int dim) const { return _simplices[dim].num(); }
  int getMaxId(int dim) const { return _free_sid[dim]; }
  bool valid(Simplex s) const;
  Simplex getSimplex(Simplex s) const { return getSimplex(s->getDim(), s->getId()); }  // Convenience.
  Simplex getSimplex(int dim, int id) const;
  int materialNum() const { return _material_strings.num(); }
  const char* getMaterial(int matid) const { return _material_strings[matid].c_str(); }
  const Map<int, Simplex>::cvalues_range simplices_dim(int dim) const { return _simplices[dim].values(); }
  OrderedSimplices_range ordered_simplices_dim(int dim) const { return OrderedSimplices_range(*this, dim); }
  void starbar(Simplex s, SimplicialComplex& result) const;
  void star(Simplex s, Array<Simplex>& ares) const;
  void ok() const;
  void scUnion(const SimplicialComplex& s1, const SimplicialComplex& s2, SimplicialComplex& result) const;

  // static constexpr FlagMask ALL = ~0u, SHARP = 1;  // flags

 private:                              // functions
  void readLine(const char* str);      // connectivity
  void attrReadLine(const char* str);  // attributes
  bool equal(Simplex s1, Simplex s2) const;
  bool eq1simp(Simplex s1, Simplex s2) const;
  bool eq2simp(Simplex s1, Simplex s2) const;
  void replace(Simplex src, Simplex tgt, Stack<Simplex>& affected_parents);
  int compare_normal(const GMesh& mesh, Corner c1, Corner c2);

  struct OrderedSimplices_range {
    using Container = Array<Simplex>;
    OrderedSimplices_range(const SimplicialComplex& sc, int dim) : _simplices(sc.simplices_dim(dim)) {
      const auto by_increasing_id = [&](Simplex s1, Simplex s2) { return s1->getId() < s2->getId(); };
      sort(_simplices, by_increasing_id);
    }
    Container::iterator begin() const { return const_cast<Container&>(_simplices).begin(); }
    Container::iterator end() const { return const_cast<Container&>(_simplices).end(); }
    int size() const { return _simplices.num(); }

   private:
    Container _simplices;
  };

  // one array per dimension
  Vec<Map<int, Simplex>, MAX_DIM + 1> _simplices;
  Array<string> _material_strings;
  Vec<int, MAX_DIM + 1> _free_sid;
};

inline Simplex SimplicialComplex::getSimplex(int dim, int id) const {
  if (_simplices[dim].contains(id))
    return _simplices[dim].get(id);
  else
    return nullptr;
}

inline Simplex ISimplex::opp_vertex(Simplex v1) {
  assertx(getDim() == 1);
  if (_child[0] == v1) return _child[1];
  if (_child[1] == v1) return _child[0];
  // no opposite to v1 on this edge
  return nullptr;
}

inline Simplex ISimplex::opp_edge(Simplex v1) {
  assertx(getDim() == 2);
  for (Simplex edge : children())
    if (edge->_child[0] != v1 && edge->_child[1] != v1) return edge;
  // no opposite to v1 on this face
  return nullptr;
}

inline void ISimplex::setChild(int num, Simplex child) {
  assertx(child->_dim == _dim - 1);
  _child[num] = child;
}

inline void ISimplex::addParent(Simplex parent) {
  assertx(parent->_dim == _dim + 1);
  _parent.push_back(parent);
}

inline void ISimplex::removeParent(Simplex old_parent) {
  assertx(old_parent->_dim == _dim + 1);
  assertx(vec_remove_ordered(_parent, old_parent));
}

// inline const Point& ISimplex::getColor() const {
//     string str;
//     const char* s = assertx(GMesh::string_key(str, getVAttribute(), "rgb"));
//     Point co; for_int(c, 3) co[c] = float_from_chars(s);
//     assert_no_more_chars(s);
//     return co;
// }

inline float ISimplex::length2() const {
  assertx(getDim() == 1);
  return dist2(getChild(0)->getPosition(), getChild(1)->getPosition());
}

inline Simplex ISimplex::edgeTo(Simplex opp_v) {
  assertx(_dim == 0);
  for (Simplex e : getParents())
    if (e->opp_vertex(this) == opp_v) return e;
  return nullptr;
}

inline void ISimplex::vertices(Simplex va[3]) {
  assertx(_dim == 2);
  Simplex va0 = getChild(0)->getChild(0);
  Simplex va1 = getChild(0)->getChild(1);
  Simplex va2 = getChild(1)->getChild(0);
  if (va2 == va0 || va2 == va1) {
    va2 = getChild(1)->getChild(1);
    assertx(va2 != va0 && va2 != va1);
  }
  va[0] = va0;
  va[1] = va1;
  va[2] = va2;
}

HH_INITIALIZE_POOL(ISimplex);

inline bool SimplicialComplex::valid(Simplex s) const {
  if (!s) return false;
  if (s->getDim() > MAX_DIM || s->getDim() < 0) return false;
  return _simplices[s->getDim()].contains(s->getId());
}

}  // namespace hh

#endif  // MESH_PROCESSING_G3DOGL_SIMPLICIALCOMPLEX_H_
