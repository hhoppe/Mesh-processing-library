// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Graph.h"

#include "RangeOp.h"
using namespace hh;

namespace {

void show_graph(const Graph<int>& g) {
  SHOW("Graph: vertices {");
  for (int i : sort(Array<int>(g.vertices().begin(), g.vertices().end()))) {
    showf("  vertex %d\n", i);
  }
  SHOW("}, edges {");
  for (int i : sort(Array<int>(g.vertices().begin(), g.vertices().end()))) {
    for (int j : sort(Array<int>(g.edges(i).begin(), g.edges(i).end()))) {
      showf(" edge (%d, %d)\n", i, j);
    }
  }
  SHOW("}");
}

}  // namespace

int main() {
  Graph<int> g;
  show_graph(g);
  for (int i : {1, 2, 3, 4, 5, 6, 7}) g.enter(i);
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
}

template class hh::Graph<unsigned>;
template class hh::Graph<const int*>;
