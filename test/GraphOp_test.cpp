// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GraphOp.h"

#include "libHh/Spatial.h"
#include "libHh/Stat.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

struct fdist {
  float operator()(int v1, int v2) const { return float(abs(v1 - v2)); }
};

struct fidist {
  float operator()(int v1, int v2) const { return float(abs(v1 - v2)); }
};

// float ffdist(const int& v1, const int& v2) { return float(abs(v1-v2)); }

void show_graph(const Graph<int>& g, bool directed = false) {
  float cost = 0.f;
  SHOW("Graph: edges {");
  for (int i : sort(Array<int>(g.vertices().begin(), g.vertices().end()))) {
    for (int j : sort(Array<int>(g.edges(i).begin(), g.edges(i).end()))) {
      if (!directed && i > j) continue;
      showf(" edge (%d, %d)\n", i, j);
      cost += fidist()(i, j);
    }
  }
  showf("}  (cost=%g)\n", cost);
}

void do_ints() {
  SHOW("do_ints");
  Graph<int> g;
  for (int i : {1, 2, 3, 4, 5, 6, 7}) g.enter(i);
  g.enter_undirected(1, 4);
  g.enter_undirected(3, 2);
  g.enter_undirected(4, 5);
  g.enter_undirected(1, 7);
  g.enter_undirected(1, 6);
  g.enter_undirected(6, 2);
  SHOW("orig:");
  show_graph(g, true);
  // SHOW(CArrayView<int>(std::vector<int>(g.vertices().begin(), g.vertices().end())));
  // SHOW(Array<int>(g.vertices().begin(), g.vertices().end()).sort());
  int vs = 2;
  {
    Dijkstra<int, fdist> di(&g, vs);
    // Dijkstra<int> di(&g, vs, ffdist);  // works too
    // auto func_fdist = [&](const int& v1, const int& v2) { return float(abs(v1-v2)); };
    // Dijkstra<int, decltype(func_fdist)> di(g, vs, func_fdist);  // works too
    for (;;) {
      if (di.done()) break;
      float dis;
      int v = di.next(dis);
      showf("%d at dist=%g\n", v, dis);
    }
  }
  SHOW(graph_edge_stats(g, fdist()));
  SHOW(graph_num_components(g));
  // graph_symmetric_closure(g);
  SHOW("symclosure:");
  show_graph(g, true);
  auto gmst = graph_mst(g, fdist());
  show_graph(gmst);
  g.enter_undirected(5, 6);
  gmst = graph_mst(g, fdist());
  show_graph(gmst);
  g.enter(8);
  g.enter(9);
  g.enter(10);
  g.enter_undirected(10, 8);
  g.enter_undirected(8, 9);
  SHOW(graph_num_components(g));
  g.enter_undirected(9, 7);
  SHOW(graph_num_components(g));
  // EMST of 7 points 0..6 on the Real line (easy)
  gmst = graph_mst(7, fidist());
  show_graph(gmst);
}

void do_points() {
  SHOW("do_points");
  Vec<Point, 20> pa;
  PointSpatial<int> psp(10);
  for_int(i, 10) {
    pa[i] = Point(.1f, .1f, .1f) + Vector(.3f, .2f, .1f) * (i * .2f);
    pa[i + 10] = Point(.12f, .1f, .1f) + Vector(.2f, .32f, .12f) * (i * .2f);
  }
  for_int(i, 20) psp.enter(i, &pa[i]);
  auto gmst = graph_quick_emst(pa, psp);
  show_graph(gmst);
  auto gkcl = graph_euclidean_k_closest(pa, 5, psp);
  show_graph(gkcl, true);
}

}  // namespace

int main() {
  do_ints();
  do_points();
}

template class hh::Dijkstra<int, fdist>;
template class hh::GraphComponent<const int*>;
