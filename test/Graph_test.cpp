// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Graph.h"

#include "libHh/RangeOp.h"
using namespace hh;

namespace {

void show_graph(const Graph<int>& g) {
  SHOW("Graph: vertices {");
  for (const int i : sort(Array(g.vertices()))) showf("  vertex %d\n", i);
  SHOW("}, edges {");
  for (const int i : sort(Array(g.vertices())))
    for (const int j : sort(Array(g.edges(i)))) showf(" edge (%d, %d)\n", i, j);
  SHOW("}");
}

}  // namespace

int main() {
  Graph<int> g;
  show_graph(g);
  for (const int i : {1, 2, 3, 4, 5, 6, 7}) g.enter(i);
  g.enter(1, 4);
  g.enter(3, 2);
  g.enter(4, 5);
  assertx(g.contains(3, 2));
  assertx(!g.contains(3, 5));
  assertx(g.out_degree(1) == 1);
  g.enter(1, 7);
  g.enter(1, 6);
  g.enter(6, 2);
  assertx(g.out_degree(1) == 3);
  show_graph(g);
  assertx(g.remove(1, 7));
  assertx(!g.remove(1, 5));
  assertx(g.remove(1, 4));
  show_graph(g);
  {
    // The outgoing edges of a vertex are kept in an array, so they are listed in insertion order.
    assertx(g.edges(1) == Array<int>{6});
    g.enter(1, 3);
    g.enter(1, 2);
    SHOW(g.edges(1));
    assertx(g.remove(1, 6));  // The removal is unordered, so the last edge replaces the removed one.
    SHOW(g.edges(1));
  }
  {
    // Vertices.
    assertx(g.contains(7) && !g.contains(8));
    assertx(!g.remove(8));  // Absent vertex.
    assertx(g.remove(7));   // A vertex with no outgoing edges.
    assertx(!g.contains(7));
    // A vertex with incoming edges may be removed, leaving dangling edges to it.
    assertx(g.remove(5) && g.contains(4, 5));
    assertx(g.remove(4, 5));
  }
  {
    // Undirected edges.
    Graph<int> g2;
    for_int(i, 4) g2.enter(i);
    g2.enter_undirected(0, 1);
    g2.enter_undirected(1, 2);
    g2.enter_undirected(2, 0);
    for_int(i, 3) assertx(g2.out_degree(i) == 2);
    assertx(g2.out_degree(3) == 0);
    assertx(g2.contains(1, 0) && g2.contains(0, 1));
    assertx(g2.remove_undirected(1, 0));
    assertx(!g2.contains(0, 1) && !g2.contains(1, 0));
    assertx(!g2.remove_undirected(1, 0));
    show_graph(g2);
    // Union with another graph, ignoring duplicate edges; the vertices must already be present.
    Graph<int> g3;
    for_int(i, 4) g3.enter(i);
    g3.enter(0, 2);  // Duplicate.
    g3.enter(3, 0);
    g3.enter(0, 3);
    g2.add(g3);
    show_graph(g2);
    // Move construction, move assignment, and swap.
    Graph<int> g4(std::move(g2));
    // NOLINTNEXTLINE(bugprone-use-after-move): the moved-from object is left empty.
    assertx(g2.empty() && !g4.empty() && g4.out_degree(0) == 2);
    Graph<int> g5;
    g5.enter(10);
    g5 = std::move(g4);
    // NOLINTNEXTLINE(bugprone-use-after-move): the moved-from object is left empty.
    assertx(g4.empty() && !g5.contains(10) && g5.contains(3, 0));
    swap(g5, g4);
    assertx(g5.empty() && g4.contains(3, 0));
    g4.clear();
    assertx(g4.empty() && !g4.contains(0));
  }
  {
    // A graph over pointer elements.
    const Array<int> ar{10, 20, 30};
    Graph<const int*> g2;
    for (const int& i : ar) g2.enter(&i);
    g2.enter(&ar[0], &ar[2]);
    g2.enter(&ar[2], &ar[1]);
    int sum = 0;
    for (const int* p1 : g2.vertices())
      for (const int* p2 : g2.edges(p1)) sum += *p1 * *p2;
    SHOW(sum);
  }
}

template class hh::Graph<unsigned>;
template class hh::Graph<const int*>;
