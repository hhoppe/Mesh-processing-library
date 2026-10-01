// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GraphOp.h"

#include "libHh/Matrix.h"
#include "libHh/Random.h"
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

float ffdist(const int& v1, const int& v2) { return float(abs(v1 - v2)); }

void show_graph(const Graph<int>& g, bool directed = false) {
  float cost = 0.f;
  SHOW("Graph: edges {");
  for (const int i : sort(Array(g.vertices()))) {
    for (const int j : sort(Array(g.edges(i)))) {
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
  for (const int i : {1, 2, 3, 4, 5, 6, 7}) g.enter(i);
  g.enter_undirected(1, 4);
  g.enter_undirected(3, 2);
  g.enter_undirected(4, 5);
  g.enter_undirected(1, 7);
  g.enter_undirected(1, 6);
  g.enter_undirected(6, 2);
  SHOW("orig:");
  show_graph(g, true);
  const int vs = 2;
  {
    Dijkstra di(&g, vs, fdist());
    for (const auto& [v, dis] : di) showf("V1: %d at dist=%g\n", v, dis);
  }
  {
    Dijkstra<int, fdist> di(&g, vs);
    for (const auto& [v, dis] : di) showf("V2: %d at dist=%g\n", v, dis);
  }
  {
    Dijkstra di(&g, vs, ffdist);
    for (const auto& [v, dis] : di) showf("V3: %d at dist=%g\n", v, dis);
  }
  {
    const auto func_fdist = [&](const int& v1, const int& v2) { return float(abs(v1 - v2)); };
    Dijkstra di(&g, vs, func_fdist);
    for (const auto& [v, dis] : di) showf("V4: %d at dist=%g\n", v, dis);
  }
  {  // The nearest vertex alone, read without consuming the search.
    Dijkstra di(&g, vs, fdist());
    const auto& [v, dis] = *di.begin();
    showf("nearest: %d at dist=%g\n", v, dis);
  }
  SHOW(graph_edge_stats(g, fdist()));
  SHOW(graph_num_components(g));
  // The graph is already symmetric, so its symmetric closure leaves it unchanged.
  graph_symmetric_closure(g);
  SHOW("symclosure:");
  show_graph(g, true);
  {
    const auto [gmst, is_connected] = graph_mst(g, fdist());
    assertx(is_connected);
    show_graph(gmst);
  }
  g.enter_undirected(5, 6);
  {
    const auto [gmst, is_connected] = graph_mst(g, fdist());
    assertx(is_connected);
    show_graph(gmst);
    g.enter(8);
    g.enter(9);
    g.enter(10);
    g.enter_undirected(10, 8);
    g.enter_undirected(8, 9);
    SHOW(graph_num_components(g));
    g.enter_undirected(9, 7);
    SHOW(graph_num_components(g));
  }
  {
    // EMST of 7 points 0..6 on the Real line (easy).
    auto gmst = graph_mst(7, fidist());
    show_graph(gmst);
  }
}

void do_points() {
  SHOW("do_points");
  Vec<Point, 20> pa;
  PointSpatial<int> spatial(10);
  for_int(i, 10) {
    pa[i] = Point(.1f, .1f, .1f) + Vector(.3f, .2f, .1f) * (i * .2f);
    pa[i + 10] = Point(.12f, .1f, .1f) + Vector(.2f, .32f, .12f) * (i * .2f);
  }
  for_int(i, 20) spatial.enter(i, &pa[i]);
  auto gmst = graph_quick_emst(pa, spatial);
  show_graph(gmst);
  auto gkcl = graph_euclidean_k_closest(pa, 5, spatial);
  show_graph(gkcl, true);
}

void do_directed() {
  SHOW("do_directed");
  Graph<int> g;
  for_int(i, 6) g.enter(i);
  g.enter(0, 1);
  g.enter(1, 2);
  g.enter(2, 0);
  g.enter(3, 4);
  g.enter(4, 3);
  // Dijkstra follows the edge directions, so it only visits the vertices reachable from the start.
  {
    Dijkstra di(&g, 1, fdist());
    assertx(!ranges::empty(di));
    for (const auto& [v, dis] : di) showf("from 1: %d at dist=%g\n", v, dis);
  }
  {
    Dijkstra di(&g, 5, fdist());  // An isolated vertex visits only itself.
    for (const auto& [v, dis] : di) showf("from 5: %d at dist=%g\n", v, dis);
  }
  SHOW(graph_edge_stats(g, fdist()));
  // Components are found by following the directed edges, so the count is well-defined only because each component
  // here is strongly connected.
  SHOW(graph_num_components(g));
  graph_symmetric_closure(g);
  show_graph(g, true);
  SHOW(graph_num_components(g));
  {
    // The representative vertices of the components are in distinct components.
    GraphComponent<int> graph_component(&g);
    Array<int> reps(graph_component);
    assertx(reps.num() == 3);
    for_int(i, reps.num()) {
      Dijkstra di(&g, reps[i], fdist());
      for (const auto& [v, dis] : di) for_int(j, reps.num()) assertx(j == i || v != reps[j]);
    }
  }
  {
    // The Kruskal MST of a disconnected graph is a spanning forest.
    const auto [gmst, is_connected] = graph_mst(g, fdist());
    assertx(!is_connected);
    show_graph(gmst);
  }
}

// Compare against brute-force computations on random points.
void do_random() {
  SHOW("do_random");
  Random random(1);
  const int n = 60;
  Array<Point> pa(n);
  for (Point& p : pa) for_int(c, 3) p[c] = .05f + .9f * random.unif();
  const auto fpdist = [&](int i, int j) { return dist(pa[i], pa[j]); };
  // A random sparse undirected graph, plus a Hamiltonian path to make it connected.
  Graph<int> g;
  for_int(i, n) g.enter(i);
  for_int(i, n - 1) g.enter_undirected(i, i + 1);
  for_int(k, 100) {
    const int i = random.get_unsigned(n), j = random.get_unsigned(n);
    if (i != j && !g.contains(i, j)) g.enter_undirected(i, j);
  }
  // Dijkstra distances match those from the Floyd-Warshall algorithm.
  {
    Matrix<double> d(n, n);
    fill(d, 1e30);
    for_int(i, n) d[i, i] = 0.;
    for_int(i, n) for (const int j : g.edges(i)) d[i, j] = fpdist(i, j);
    for_int(k, n) for_int(i, n) for_int(j, n) d[i, j] = min(d[i, j], d[i, k] + d[k, j]);
    for (const int vs : {0, 17, 59}) {
      Dijkstra di(&g, vs, fpdist);
      int count = 0;
      float od = 0.f;
      for (const auto& [v, dis] : di) {
        assertx(abs(dis - d[vs, v]) < 1e-5);
        assertx(dis >= od);
        od = dis;
        count++;
      }
      assertx(count == n);
    }
  }
  // The Kruskal MST of the graph is a spanning tree no heavier than the path, and the Prim MST of the complete graph
  // and the quick EMST have the same weight.
  const auto tree_weight = [&](const Graph<int>& tree) {
    double sum = 0.;
    int num_edges = 0;
    for (const int i : tree.vertices())
      for (const int j : tree.edges(i))
        if (i < j) sum += fpdist(i, j), num_edges++;
    assertx(num_edges == n - 1 && graph_num_components(tree) == 1);
    return sum;
  };
  {
    const auto [gmst, is_connected] = graph_mst(g, fpdist);
    assertx(is_connected);
    double path_weight = 0.;
    for_int(i, n - 1) path_weight += fpdist(i, i + 1);
    assertx(tree_weight(gmst) <= path_weight);
  }
  const Graph<int> gprim = graph_mst(n, fpdist);
  PointSpatial<int> spatial(10);
  for_int(i, n) spatial.enter(i, &pa[i]);
  const Graph<int> gquick = graph_quick_emst(pa, spatial);
  const double prim_weight = tree_weight(gprim), quick_weight = tree_weight(gquick);
  assertx(abs(prim_weight - quick_weight) < 1e-4);
  SHOW(round_fraction_digits(prim_weight, 1e4));
  // The Kruskal MST of the complete graph also has the same weight.
  {
    Graph<int> gcomplete;
    for_int(i, n) gcomplete.enter(i);
    for_int(i, n) for_int(j, i) gcomplete.enter_undirected(i, j);
    const auto [gmst, is_connected] = graph_mst(gcomplete, fpdist);
    assertx(is_connected && abs(tree_weight(gmst) - prim_weight) < 1e-4);
  }
  // Each vertex connects to its k closest points.
  const int k = 4;
  const Graph<int> gkcl = graph_euclidean_k_closest(pa, k, spatial);
  for_int(i, n) {
    assertx(gkcl.out_degree(i) == k);
    Array<float> dists;
    for_int(j, n) if (j != i) dists.push(fpdist(i, j));
    sort(dists);
    for (const int j : gkcl.edges(i)) assertx(fpdist(i, j) <= dists[k - 1]);
  }
}

}  // namespace

int main() {
  do_ints();
  do_points();
  do_directed();
  do_random();
}

template class hh::Dijkstra<int, fdist>;
template class hh::GraphComponent<const int*>;
