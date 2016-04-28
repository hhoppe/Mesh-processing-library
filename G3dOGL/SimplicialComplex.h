// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Map.h"
#include "Pqueue.h"
#include "GMesh.h"
#include "Polygon.h"
#include "PArray.h"
#include "Queue.h"
#include "Timer.h"
#include "MeshOp.h"             // Vnors
#include "A3dStream.h"
#include "Stack.h"
#include "Flags.h"

namespace hh {

#define MAX_DIM 2

class ISimplex;
using Simplex = ISimplex*;

class SimplicialComplex : noncopyable {
 public:
    SimplicialComplex()                         { for_int(i, MAX_DIM+1) _free_sid[i] = 1; }
    ~SimplicialComplex()                        { clear(); }

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
    int num(int dim) const;
    int getMaxId(int dim) const;
    bool valid(Simplex s) const;
    Simplex getSimplex(Simplex s) const; // convenience
    Simplex getSimplex(int dim, int id) const;
    int materialNum() const;
    const char* getMaterial(int matid) const;
    const Map<int,Simplex>& getSimplices(int dim) const; // ForSCSimplex
    void starbar(Simplex s, SimplicialComplex& result) const;
    void star(Simplex s, Array<Simplex>& ares) const;
    void ok() const;
    void scUnion(const SimplicialComplex& s1, const SimplicialComplex& s2, SimplicialComplex& result) const;
    
    // static constexpr FlagMask ALL = ~0u, SHARP = 1; // flags

 private:                           // functions
    void readLine(const char* str); // connectivity
    void attrReadLine(const char* str); // attributes
    bool equal(Simplex s1, Simplex s2) const;
    bool eq1simp(Simplex s1, Simplex s2) const;
    bool eq2simp(Simplex s1, Simplex s2) const;
    void replace(Simplex src, Simplex with, Stack<Simplex>& affected);
    int compare_normal(const GMesh& mesh, Corner c1, Corner c2);

    // one array per dimension
    Vec<Map<int,Simplex>, MAX_DIM+1> _simplices;
    Array<string> _material_strings;
    Vec<int,MAX_DIM+1> _free_sid;
};

class ISimplex : noncopyable {
 public:
    friend SimplicialComplex;

    // 0-dim simplex has _child[0] which is ignored
    ISimplex(int dim, int id)                   : _dim(dim), _id(id) { for_int(i, _dim+1) _child[i] = nullptr; }

    void setChild(int num, Simplex child);
    void addParent(Simplex parent);
    void removeParent(Simplex parent);

    Simplex getChild(int num) const;
    const std::vector<Simplex>& getParents() const;
    int getDim() const;
    int getId() const;

    float length2() const;
    float length() const;
    void polygon(Polygon& p) const;

    // 2-simplices
    void vertices(Simplex va[3]);
    Simplex opp_edge(Simplex v);
//    Simplex opp_vertex(Simplex e);

    // 1-simplices
    Simplex opp_vertex(Simplex v);

    // 0-simplices
    Simplex edgeTo(Simplex opp_v);

    // attribute mod and access function
    void setPosition(const Point& pos);
    void setVAttribute(int va);
    void setArea(float area);
    Flags& flags()                              { assertnever("no longer supported"); return _flags; }

    const Point& getPosition() const;
    int getVAttribute() const;
    float getArea() const;
    const Flags& flags() const                  { assertnever("no longer supported"); return _flags; }

    // predicates
    bool hasColor() const;
    bool isPrincipal() const;
    bool is_boundary() const;
    // 1 if nbhd a manifold, 0 otherwise
    bool isManifold() const;

    // for gemorph
    const char* get_string() const;
    void set_string(const char* s);
    void update_string(const char* key, const char* val);

    HH_POOL_ALLOCATION(ISimplex);

 private:
    int _dim;                      // dimension of the simplex
    int _id;                       // simplex id
    Vec<Simplex,MAX_DIM+1> _child; // simplices it contains
    std::vector<Simplex> _parent;  // simplices it belongs to
    // Attributes
    Flags _flags;
    Point _position;            // simplices of dimension 0 only
    int _vattribute {-1};       // visual attributes
    float _area {0.f};
    // for geomorph
    unique_ptr<char[]> _string;
};

// Some iterators behave unpredictably if SC modified during traversal

#define ForSCSimplex(K, dim, s)                                               \
    { { for (hh::Simplex s : (K).getSimplices(dim).values())

#define ForSCOrderedSimplex(K, dim, s)                                                          \
    { hh::OrderedSimplexIter HH_ID(iter)(K, dim); while (hh::Simplex s = HH_ID(iter).next()) {

// iterate over immediate children of a simplex (1 dimension down)
// getChild() no longer return 0:
//  { Simplex c; for (int i = 0; c = s->getChild(i); i++) {
// JOVAN: Why can the child be zero?  move the test where needed!
#define ForSCSimplexChildIndex(s, c, i)                                                     \
    { for_int(i, (s)->getDim()+1) { hh::Simplex c = (s)->getChild(i); { if (!c) break; }
#define ForSCSimplexChild(s, c) ForSCSimplexChildIndex(s, c, HH_UNIQUE_ID(i))

// iterate over all descendents of a simplex (>= 1 dimension)
// iterates by generations children first, grandchildren next, etc.
#define ForSCSimplexFaces(s, c)                                                         \
    { hh::SimplexFacesIter HH_ID(iter)(s); while (hh::Simplex c = HH_ID(iter).next()) {

// iterate over immediate parents of a simplex (1 dimension up)
#define ForSCSimplexParent(s, p)                                              \
    { { for (hh::Simplex p : (s)->getParents())

// iterate over all ancestors of a simplex (>= 1 dimension)
// iterates by generations parents first, grandparents next, etc.
#define ForSCSimplexStar(s, p)                                                          \
    { hh::SimplexStarIter HH_ID(iter)(s); while (hh::Simplex p = HH_ID(iter).next()) {

// iterate through faces adjacent to a vertex
#define ForSCVertexFace(v, f)                                                                   \
    { hh::SimplexVertexFaceIter HH_ID(iter)(v); while (hh::Simplex f = HH_ID(iter).next()) {

#define EndFor } }

class OrderedSimplexIter {
 public:
    OrderedSimplexIter(const SimplicialComplex& K, int dim);
    Simplex next() { return pq.empty() ? nullptr : pq.remove_min(); }
 private:
    Pqueue<Simplex> pq;
};

class SimplexStarIter {
 public:
    SimplexStarIter(Simplex s);
    Simplex next();
 private:
    PArray<Simplex,20> _sq;
    int _index;
};

class SimplexVertexFaceIter {
 public:
    SimplexVertexFaceIter(Simplex s);
    Simplex next();
 private:
    PArray<Simplex,20> _sq;
    int _index;
};

class SimplexFacesIter {
 public:
    SimplexFacesIter(Simplex s);
    Simplex next();
 private:
    Queue<Simplex> _sq;
};

// SimplicialComplex
inline int SimplicialComplex::materialNum() const {
    return _material_strings.num();
}

inline const char* SimplicialComplex::getMaterial(int matid) const {
    return _material_strings[matid].c_str();
}

inline int SimplicialComplex::num(int dim) const {
    return _simplices[dim].num();
}

inline int SimplicialComplex::getMaxId(int dim) const {
    return _free_sid[dim];
}

inline int ISimplex::getDim() const {
    return _dim;
}

inline int ISimplex::getId() const {
    return _id;
}

inline Simplex SimplicialComplex::getSimplex(int dim, int id) const {
    if (_simplices[dim].contains(id))
        return _simplices[dim].get(id);
    else
        return nullptr;
}

inline Simplex SimplicialComplex::getSimplex(Simplex s) const {
    Simplex ts = getSimplex(s->getDim(), s->getId());
#if 0
    // this function is bad ie does the reasonable thing
    // iff simplex s belongs to the same simplicial complex
    // as this or is from some copy derived SC assured to have
    // same ids, I used it with different assumption at some places
    // so to keep myself sane I'll check things here.
    // but this should eventually disappear
    if (s->getDim()==1) {
        assertx(ts->getDim()==1);
        assertx(ts->_child[0]==s->_child[0]);
        assertx(ts->_child[1]==s->_child[1]);
    }
#endif
    return ts;
}

// ForSCSimplex needs it
inline const Map<int,Simplex>& SimplicialComplex::getSimplices(int dim) const {
    return _simplices[dim];
}

inline Simplex ISimplex::getChild(int num) const {
    // assertx(num >= 0);
    // BAD: if (num>_dim) return nullptr;
    // should have:
    // assertx(num>=0 && num<=_dim);
    return _child[num];
}

inline Simplex ISimplex::opp_vertex(Simplex v1) {
    assertx(getDim()==1);
    if (_child[0] == v1) return _child[1];
    if (_child[1] == v1) return _child[0];
    // no opposite to v1 on this edge
    return nullptr;
}

inline Simplex ISimplex::opp_edge(Simplex v1) {
    assertx(getDim()==2);
    ForSCSimplexChild(this, edge) {
        if (edge->_child[0]!=v1 && edge->_child[1]!=v1)
            return edge;
    } EndFor;
    // no opposite to v1 on this face
    return nullptr;
}

inline void ISimplex::setChild(int num, Simplex child) {
    assertx(child->_dim == _dim-1);
    _child[num] = child;
}

inline void ISimplex::addParent(Simplex parent) {
    assertx(parent->_dim == _dim+1);
    _parent.push_back(parent);
}

inline void ISimplex::removeParent(Simplex old_parent) {
    assertx(old_parent->_dim == _dim+1);
    assertx(vec_remove_ordered(_parent, old_parent));
}

inline const std::vector<Simplex>& ISimplex::getParents() const {
    return _parent;
}

inline bool ISimplex::isPrincipal() const {
    return _parent.empty();
}

inline bool ISimplex::is_boundary() const {
    return _parent.size()==1;
}

inline bool ISimplex::isManifold() const {
    return _parent.size()==2;
}


inline void ISimplex::setPosition(const Point& pos) {
    assertx(_dim == 0);
    _position = pos;
}

inline void ISimplex::setVAttribute(int va) {
    _vattribute = va;
}

inline void ISimplex::setArea(float area) {
    _area = area;
}

inline const Point& ISimplex::getPosition() const {
    // assertx(_dim == 0);
    return _position;
}

inline int ISimplex::getVAttribute() const {
    return _vattribute;
}

inline float ISimplex::getArea() const {
    return _area;
}


// inline const Point& ISimplex::getColor() const {
//     string str;
//     const char* s = assertx(GMesh::string_key(str, getVAttribute(), "rgb"));
//     Point co; assertx(sscanf(s, "( %g %g %g )", &co[0], &co[1], &co[2])==3);
//     return co;
// }

inline bool ISimplex::hasColor() const {
    return _vattribute >= 0;
}

inline float ISimplex::length2() const {
    assertx(getDim() == 1);
    return dist2(getChild(0)->getPosition(), getChild(1)->getPosition());
}


inline float ISimplex::length() const {
    return sqrt(length2());
}

inline Simplex ISimplex::edgeTo(Simplex opp_v) {
    assertx(_dim==0);
    ForSCSimplexParent(this, e) {
        if (e->opp_vertex(this) == opp_v)
            return e;
    } EndFor;

    return nullptr;
}

inline void ISimplex::vertices(Simplex va[3]) {
#if 0
    assertx(_dim==2);
    va[0] = getChild(0)->getChild(0);
    va[1] = getChild(0)->getChild(1);
    va[2] = getChild(1)->getChild(0);

    if (va[2] == va[0] || va[2] == va[1])
        va[2] = getChild(1)->getChild(1);

    assertx(va[2] != va[1] && va[2] != va[0]);
#endif
    assertx(_dim==2);
    Simplex va0 = getChild(0)->getChild(0);
    Simplex va1 = getChild(0)->getChild(1);
    Simplex va2 = getChild(1)->getChild(0);
    
    if (va2 == va0 || va2 == va1) {
        va2 = getChild(1)->getChild(1);
        assertx(va2 != va0 && va2 != va1);
    }
    va[0] = va0; va[1] = va1; va[2] = va2;
}

inline const char* ISimplex::get_string() const {
    return _string.get();
}

inline void ISimplex::set_string(const char* s) {
    _string = make_unique_c_string(s);
}

inline void ISimplex::update_string(const char* key, const char* val) {
    GMesh::update_string_ptr(_string, key, val);
}

HH_INITIALIZE_POOL(ISimplex);

inline bool SimplicialComplex::valid(Simplex s) const {
    if (!s) return false;
    if (s->getDim() > MAX_DIM || s->getDim() < 0) return false;

    return _simplices[s->getDim()].contains(s->getId());
}

}
