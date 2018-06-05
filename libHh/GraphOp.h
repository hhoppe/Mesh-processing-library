// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_GRAPHOP_H_
#define MESH_PROCESSING_LIBHH_GRAPHOP_H_

#include "Graph.h"
#include "Pqueue.h"
#include "Set.h"
#include "Queue.h"
#include "Geometry.h"
#include "UnionFind.h"
#include "Spatial.h"
#include "Stat.h"
#include "Array.h"
#include "RangeOp.h"            // fill()

namespace hh {

// Add edges to a graph to make it symmetric.
template<typename T> void graph_symmetric_closure(Graph<T>& g) {
    for (const T& v1 : g.vertices()) {
        for (const T& v2 : g.edges(v1)) {
            if (!g.contains(v2, v1)) g.enter(v2, v1);
        }
    }
}

// Given a graph (possibly directed), return vertices in order of increasing graph distance from vs.
// (Vertex vs itself is returned on first invocation of next().)
template<typename T, typename Func_dist = float (&)(const T& v1, const T& v2)> class Dijkstra : noncopyable {
 public:
    explicit Dijkstra(const Graph<T>* g, T vs, Func_dist fdist = Func_dist{}) : _g(*assertx(g)), _fdist(fdist) {
        _pq.enter(vs, 0.f);
    }
    bool done()                                 { return _pq.empty(); }
    T next(float& dis) {
        assertx(!_pq.empty());
        float dmin = _pq.min_priority();
        T vmin = _pq.remove_min();
        _set.enter(vmin);
        for (const T& v : _g.edges(vmin)) {
            if (_set.contains(v)) continue;
            float pnd = dmin+_fdist(vmin, v); // possibly smaller distance
            _pq.enter_update_if_smaller(v, pnd);
        }
        dis = dmin;
        return vmin;
    }
 private:
    const Graph<T>& _g;
    Func_dist _fdist;
    HPqueue<T> _pq;
    Set<T> _set;
};

// *** Kruskal MST

// Given a graph gnew consisting solely of vertices, computes the minimum spanning tree of undirectedg over
//  the vertices in gnew under the cost metric fdist.  Returns is_connected.
// Implementation: Kruskal's algorithm, O(e log(e))
//  (Prim's algorithm is recommended when e=~n^2, see below)
template<typename T, typename Func = float(const T&, const T&)>
bool graph_mst(const Graph<T>& undirectedg, Func fdist, Graph<T>& gnew) {
    int nv = 0, nebefore = 0;
    struct tedge { T v1, v2; float w; };
    Array<tedge> tedges;
    for (const T& v1 : gnew.vertices()) {
        nv++;
        ASSERTX(!gnew.out_degree(v1));
        for (const T& v2 : undirectedg.edges(v1)) {
            if (v1<v2) continue;
            ASSERTXX(gnew.contains(v2));
            nebefore++;
            tedges.push(tedge{v1, v2, fdist(v1, v2)});
        }
    }
    sort(tedges, [](const tedge& a, const tedge& b) { return a.w < b.w; });
    UnionFind<T> uf;
    int neconsidered = 0, neadded = 0;
    for (const tedge& t : tedges) {
        neconsidered++;
        T v1 = t.v1, v2 = t.v2;
        if (!uf.unify(v1, v2)) continue;
        gnew.enter_undirected(v1, v2);
        neadded++;
        if (neadded==nv-1) break;
    }
    showf("graph_mst: %d vertices, %d/%d edges considered, %d output\n", nv, neconsidered, nebefore, neadded);
    return neadded==nv-1;
}

// Returns an undirected graph that is the MST of undirectedg, or empty if g is not connected.
template<typename T, typename Func = float(const T&, const T&)>
Graph<T> graph_mst(const Graph<T>& undirectedg, Func fdist) {
    assertx(!undirectedg.empty());
    Graph<T> gnew;
    for (const T& v : undirectedg.vertices()) { gnew.enter(v); }
    if (!graph_mst(undirectedg, fdist, gnew))
        gnew.clear();
    return gnew;
}

// *** Prim MST

// Returns a undirected graph that is the minimum spanning tree of the full graph
//  between the num points, where the cost metric between two points v1 and v2 is fdist(v1, v2).
// Implementation: Prim's algorithm, complexity O(n^2)!
template<typename Func = float(int, int)> Graph<int> graph_mst(int num, Func fdist) {
    assertx(num>0);
    const float k_inf = 1e30f;
    Array<float> lowcost(num);
    Array<int> closest(num);
    Graph<int> gnew;
    for_int(i, num) { gnew.enter(i); }
    for_intL(i, 1, num) {
        lowcost[i] = fdist(0, i);
        closest[i] = 0;
    }
    for_intL(i, 1, num) {
        const int offset = 1;
        int minj; float minf = min_index(lowcost.slice(offset, num), &minj); minj += offset;
        assertx(minf<k_inf);
        gnew.enter_undirected(minj, closest[minj]);
        lowcost[minj] = k_inf;
        for_intL(j, 1, num) {
            if (lowcost[j]==k_inf) continue;
            float pnd = fdist(minj, j);
            if (pnd<lowcost[j]) { lowcost[j] = pnd; closest[j] = minj; }
        }
    }
    return gnew;
}

// *** graph_quick_emst

// Try to build the EMST of the num points pa using all edges with length less than thresh.
// Uses modified Prim's, where only edges of length<thresh are considered.
// Uses a HPqueue because it can no longer afford to find min in O(n) time.
// Returns an empty graph if not connected.
inline Graph<int> try_emst(float thresh, CArrayView<Point> pa, const PointSpatial<int>& sp) {
    Graph<int> gnew;
    Array<bool> inset(pa.num(), false); // vertices already added to mst
    Array<int> closest(pa.num());       // for !inset[i], closest inset[] so far
    for_int(i, pa.num()) { gnew.enter(i); }
    HPqueue<int> pq;
    pq.enter(0, 0.f);
    while (!pq.empty()) {
        int i = pq.remove_min();
        ASSERTXX(pa.ok(i) && !inset[i]);
        if (i) gnew.enter_undirected(i, closest[i]);
        inset[i] = true;
        SpatialSearch<int> ss(&sp, pa[i]);
        for (;;) {
            if (ss.done()) break;
            float dis2; int j = ss.next(&dis2);
            if (dis2>square(thresh)) break;
            if (inset[j]) continue;
            if (pq.enter_update_if_smaller(j, dis2))
                closest[j] = i;
        }
    }
    int nfound = 0; for_int(i, pa.num()) { if (inset[i]) nfound++; }
    if (nfound!=pa.num()) gnew.clear();
    return gnew;
}

// Same as graph_mst() but works specifically on an array of points and tries
// to do it more quickly by making use of a spatial data structure.
// Implementation: Prim's MST on series of subgraphs.
inline Graph<int> graph_quick_emst(CArrayView<Point> pa, const PointSpatial<int>& sp) {
    Graph<int> gnew;
    const float initf = .02f;
    int n = 0;
    for (float f = initf; ; f *= 1.6f) {
        n++;
        gnew = try_emst(f, pa, sp);
        if (!gnew.empty()) break;
        assertx(f<1.f);
    }
    showf("GraphQuickEmst: had to do %d approximate Emst's\n", n);
    return gnew;
}

// Return statistics about graph edge lengths.  If undirected, edges stats are duplicated.
template<typename T, typename Func = float(const T&, const T&)> Stat graph_edge_stats(const Graph<T>& g, Func fdist) {
    Stat stat;
    for (const T& v1 : g.vertices()) {
        for (const T& v2 : g.edges(v1)) {
            stat.enter(fdist(v1, v2));
        }
    }
    return stat;
}

// Returns a newly allocated directed graph that connects each vertex to its
// kcl closest neighbors (based on Euclidean distance).
// Consider applying graph_symmetric_closure() !
inline Graph<int> graph_euclidean_k_closest(CArrayView<Point> pa, int kcl, const PointSpatial<int>& sp) {
    Graph<int> gnew;
    for_int(i, pa.num()) { gnew.enter(i); }
    for_int(i, pa.num()) {
        SpatialSearch<int> ss(&sp, pa[i]);
        for_int(nn, kcl+1) {
            assertx(!ss.done());
            int j = ss.next();
            if (j==i) continue;
            gnew.enter(i, j);
        }
    }
    return gnew;
}

// *** GraphComponent

// Access each connected component of a graph.
// next() returns a representative vertex of each component.
template<typename T> class GraphComponent : noncopyable {
 public:
    explicit GraphComponent(const Graph<T>* g) : _g(*assertx(g)) {
        auto&& r = _g.vertices(); _vcur = r.begin(); _vend = r.end();
    }
    explicit operator bool() const              { return _vcur!=_vend; }
    T operator()() const                        { return *_vcur; }
    void next() {
        Queue<T> queue;
        _set.enter(*_vcur);
        queue.enqueue(*_vcur);
        while (!queue.empty()) {
            T v = queue.dequeue();
            for (const T& v2 : _g.edges(v)) {
                if (_set.add(v2)) queue.enqueue(v2);
            }
        }
        for (++_vcur; _vcur!=_vend; ++_vcur)
            if (!_set.contains(*_vcur)) break;
    }
 private:
    const Graph<T>& _g;
    typename Graph<T>::vertex_iterator _vcur;
    typename Graph<T>::vertex_iterator _vend;
    Set<T> _set;
};

template<typename T> int graph_num_components(const Graph<T>& g) {
    int n = 0;
    for (GraphComponent<T> gc(&g); gc; gc.next()) n++;
    return n;
}

// *** Graph coloring

// Color a graph (heuristically since optimal is NP-hard).
// Assign colors (starting with 1).  Return number colors assigned.
template<typename T> int graph_color(const Graph<T>& graph, Map<T,int>& ret_colors) {
    int num_colors = 0;
    ret_colors.clear();
    for (const T& vv : graph) {
        bool is_new; ret_colors.enter(vv, 0, is_new);
        if (!is_new) continue;
        Queue<T> queue;
        queue.enqueue(vv);
        while (!queue.empty()) {
            T v = queue.dequeue();
            ASSERTXX(ret_colors.get(v)==0);
            Set<int> setcol;
            for (const T& v2 : graph.edges(v)) {
                bool is_new2; int col2 = ret_colors.enter(v2, 0, is_new2);
                if (is_new2) {
                    queue.enqueue(v2);
                } else {
                    if (col2) setcol.enter(col2);
                }
            }
            int col = 1;
            while (setcol.contains(col)) col++;
            ret_colors.replace(v, col);
            if (col>num_colors) num_colors = col;
        }
    }
    return num_colors;
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_GRAPHOP_H_
