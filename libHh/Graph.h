// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_GRAPH_H_
#define MESH_PROCESSING_LIBHH_GRAPH_H_

#include "Map.h"
#include "Array.h"

#if 0
{
  Graph<int> g;
  for (int v1 : g) {
    for (int v2 : g.edges(v1)) process_edge(v1, v2);
  }
}
#endif

namespace hh {

// A Graph encodes a relation from a set of elements to itself, represented by a set of edge tuples (T, T).
// The Graph allows quick iteration over outgoing edges.  Duplicate edges are not allowed.
template<typename T> class Graph : noncopyable {
    using type = Graph<T>;
    using atype = Array<T>;
    using base = Map<T,atype>;
 public:
    using vertices_range = typename base::keys_range;
    using vertex_iterator = typename base::keys_iterator;
    using edges_range = const atype&;
    using edge_iterator = const typename atype::const_iterator;
    Graph()                                     = default;
    Graph(type&& g) noexcept                    { swap(*this, g); }
    type& operator=(type&& g) noexcept          { clear(); swap(*this, g); return *this; }
    void clear()                                { _m.clear(); }
    bool empty() const                          { return _m.num()==0; }
    // enter and remove domain vertices
    void enter(T v)                             { _m.enter(v, atype()); } // vertex v must be new
    bool contains(T v) const                    { return _m.contains(v); }
    bool remove(T v);           // must have 0 out_degree, ret: was_there
    // enter an edge
    void enter(T v1, T v2)                      { ASSERTXX(!contains(v1, v2)); _m.get(v1).push(v2); }
    // enter undirected edge
    void enter_undirected(T v1, T v2)           { enter(v1, v2); enter(v2, v1); } // v1 & v2 present; new edge
    bool contains(T v1, T v2) const             { return _m.get(v1).contains(v2); }         // O(n) slow
    bool remove(T v1, T v2)                     { return _m.get(v1).remove_unordered(v2); } // O(n) , ret: was_there
    bool remove_undirected(T v1, T v2); // O(n) , ret: was_there
    int out_degree(T v) const                   { return _m.get(v).num(); }
    void add(const Graph<T>& g);
    vertices_range vertices() const             { return _m.keys(); }
    edges_range edges(T v) const                { return _m.get(v); }
    friend void swap(Graph& l, Graph& r) noexcept { using std::swap; swap(l._m, r._m); }
 private:
    base _m;
};

//----------------------------------------------------------------------------

template<typename T> bool Graph<T>::remove(T v) {
    if (!contains(v)) return false;
    atype ar = _m.remove(v);
    assertx(!ar.num());
    return true;
}

template<typename T> bool Graph<T>::remove_undirected(T v1, T v2) {
    bool r1 = remove(v1, v2);
    bool r2 = remove(v2, v1);
    assertx(r1==r2);
    return r1;
}

template<typename T> void Graph<T>::add(const Graph& g) {
    for (const T& v1 : g.vertices()) {
        for (const T& v2 : g.edges(v1)) {
            // if (!contains(v1, v2)) enter(v1, v2);
            atype& ar = _m.get(v1);
            if (!ar.contains(v2)) ar.push(v2);
        }
    }
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_GRAPH_H_
