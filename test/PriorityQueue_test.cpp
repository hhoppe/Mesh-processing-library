// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/PriorityQueue.h"

#include <random>  // default_random_engine, mt19937

using namespace hh;

namespace {

void test1() {
#if 0
  {
    PriorityQueue<int> pq;
    PriorityQueue<int> pq2 = pq;  // Fails because it is not copyable.
    dummy_use(pq2);
  }
  {
    PriorityQueue<int> pq;
    PriorityQueue<int> pq2 = std::move(pq);  // Fails because move construction is not defined.
    dummy_use(pq2);
  }
#endif
}

void test2() {
  {
    PriorityQueue<int> pq;
    assertx(pq.num() == 0);
    assertx(pq.empty());
    for_int(i, 100) pq.enter(i, i * 2.f + 1.f);
    assertw(pq.num() == 100);
    for_int(i, 100) pq.enter(100 + i, i * 2.f);
    assertw(pq.num() == 200);
    assertw(pq.min() == 100);
    assertw(pq.min_priority() == 0.f);
    assertw(pq.remove_min() == 100);
    assertw(pq.min() == 0);
    assertw(pq.min_priority() == 1.f);
    pq.enter(100, 0 * 2.f);
    for_int(i, 100) {
      assertx(pq.min() >= 0.f);
      assertw(!pq.empty());
      assertw(pq.num() == 200 - i);
      assertw(pq.remove_min() == (i % 2 ? i / 2 : 100 + i / 2));
    }
  }
  {
    UpdatablePriorityQueue<int> pq;
    assertx(pq.num() == 0);
    assertx(pq.empty());
    for_int(i, 100) pq.enter(i, i * 2.f + 1.f);
    assertw(pq.num() == 100);
    for_int(i, 100) pq.enter(100 + i, i * 2.f);
    assertw(pq.num() == 200);
    assertw(pq.retrieve(2) == 2 * 2.f + 1.f);
    assertw(pq.retrieve(102) == 2 * 2.f);
    assertw(pq.min() == 100);
    assertw(pq.retrieve(200) < 0.f);
    assertw(pq.remove(100) >= 0.f);
    assertw(pq.min() == 0);
    pq.enter(100, 0 * 2.f);
    for_int(i, 100) {
      assertx(pq.min() >= 0.f);
      assertw(!pq.empty());
      assertw(pq.num() == 200 - i);
      assertw(pq.remove_min() == (i % 2 ? i / 2 : 100 + i / 2));
    }
    assertw(pq.update(177, 3.f) >= 0.f);
    assertw(pq.min() == 177);
    for_int(i, 100) {
      assertw(pq.min());
      pq.remove_min();
    }
    assertw(pq.empty());
    assertw(pq.num() == 0);
  }
}

void test3() {
  const int n = 1000;
  UpdatablePriorityQueue<int> pq;
  pq.reserve(n);
  for_int(i, n) pq.enter_unsorted(i, 2.f + std::sin(i * 7.f));
  pq.heapify();
  float a = 0.f;
  while (!pq.empty()) {
    const float b = pq.min_priority();
    assertx(b >= a);
    a = b;
    pq.remove_min();
  }
}

void test4() {
  PriorityQueue<int> pq;
  for_int(i, 1000) pq.enter(i, 2.f + std::sin(float(i)));
  float a = 0.f;
  while (!pq.empty()) {
    const float b = pq.min_priority();
    assertx(b >= a);
    a = b;
    pq.remove_min();
  }
}

void test5() {
  std::default_random_engine random_engine;
  for_int(itest, 500) {
    const int n = itest < 30 ? itest : 30 + random_engine() % 400;
    UpdatablePriorityQueue<int> pq;
    for_int(i, n) pq.enter_unsorted(i, float(random_engine()));
    pq.heapify();
    float a = 0.f;
    while (!pq.empty()) {
      const float b = pq.min_priority();
      assertx(b >= a);
      a = b;
      pq.remove_min();
    }
  }
}

void test6() {
  std::default_random_engine random_engine;
  for_int(itest, 100) {
    const int n = 70;
    UpdatablePriorityQueue<int> pq;
    for_int(i, n) pq.enter_unsorted(i, float(random_engine()));
    pq.heapify();
    for_int(i, n * 3) pq.update(i, float(random_engine()));
    float a = 0.f;
    while (!pq.empty()) {
      const float b = pq.min_priority();
      assertx(b >= a);
      a = b;
      pq.remove_min();
    }
  }
}

void test7() {
  for_int(k, 500) {
    const int n = 30;
    UpdatablePriorityQueue<int> pq;
    pq.reserve(n);
    Array<float> arval1;
    for_int(i, n) arval1.push(2.f + std::sin(i * 11.f + k * 1.2345f));
    Array<float> arval2;
    for_int(i, n) arval2.push(2.f + std::sin(i * 13.f + k * 2.7419f));
    Array<float> arval3;
    for_int(i, n) arval3.push(2.f + std::sin(i * 13.f + k * 3.1415f));
    for_int(i, n) pq.enter_unsorted(i, arval1[i]);
    pq.heapify();
    for_int(i, n) assertx(pq.update(i, arval2[i]) == arval1[i]);
    for_int(i, n) assertx(pq.update(i, arval3[i]) == arval2[i]);
    for_int(i, n) assertx(pq.retrieve(i) == arval3[i]);
    float a = 0.f;
    while (!pq.empty()) {
      const float b = pq.min_priority();
      assertx(b >= a);
      a = b;
      const int i = pq.remove_min();
      assertx(b == arval3[i]);
    }
  }
}

void test8() {
  PriorityQueue<unique_ptr<int>> pq;
  pq.reserve(4);
  pq.enter(make_unique<int>(3), 1.5f);
  pq.enter(make_unique<int>(4), 1.3f);
  pq.enter(make_unique<int>(5), 1.6f);
  pq.enter(make_unique<int>(6), 1.2f);
  pq.enter(make_unique<int>(7), 1.7f);
  pq.enter(make_unique<int>(8), 1.8f);
  assertx(*pq.min() == 6);
  assertx(pq.min_priority() == 1.2f);
  {
    const unique_ptr<int> p = pq.remove_min();
    assertx(*p == 6);
  }
  {
    const unique_ptr<int> p = pq.remove_min();
    assertx(*p == 4);
  }
  pq.clear();
  pq.enter_unsorted(make_unique<int>(3), 1.5f);
  pq.enter_unsorted(make_unique<int>(4), 1.3f);
  pq.enter_unsorted(make_unique<int>(5), 1.6f);
  pq.enter_unsorted(make_unique<int>(6), 1.2f);
  pq.enter_unsorted(make_unique<int>(7), 1.7f);
  pq.enter_unsorted(make_unique<int>(8), 1.8f);
  pq.heapify();
  assertx(*pq.min() == 6);
  assertx(pq.min_priority() == 1.2f);
  {
    const unique_ptr<int> p = pq.remove_min();
    assertx(*p == 6);
  }
  {
    const unique_ptr<int> p = pq.remove_min();
    assertx(*p == 4);
  }
}

void test9() {
  const int ntests = 100;
  const int n = 19;
  std::default_random_engine random_engine;
  for_int(k, ntests) {
    Array<int> ar1;
    for_int(i, n) ar1.push(i);
    ranges::shuffle(ar1, random_engine);
    Array<float> ar2;
    for_int(i, n) ar2.push(float(i));
    ranges::shuffle(ar2, random_engine);
    Array<float> ar3;
    for_int(i, n) ar3.push(float(i));
    ranges::shuffle(ar3, random_engine);
    UpdatablePriorityQueue<int> hpq;
    for_int(i, n) {
      const int j = ar1[i];
      hpq.enter(j, ar2[j]);
    }
    for_int(i, n) {
      const int j = i;
      hpq.update(j, ar3[j]);
    }
    {
      float expectedv = 0.f;
      while (!hpq.empty()) {
        const float pri = hpq.min_priority();
        const int j = hpq.remove_min();
        assertx(pri == expectedv);
        assertx(ar3[j] == pri);
        expectedv += 1.f;
      }
      assertx(expectedv == float(n));
    }
  }
}

void test10() {  // A PriorityQueue with built-in storage behaves identically, also beyond its inline_capacity.
  const Array<float> priorities{5.f, 1.f, 4.f, 1.5f, 9.f, 2.f, 6.f, 0.5f};
  PriorityQueue<int> pq0;
  PriorityQueue<int, 4> pq4;
  for_int(i, priorities.num()) pq0.enter(i, priorities[i]), pq4.enter(i, priorities[i]);
  Array<int> order;
  while (!pq0.empty()) {
    const int i = pq0.remove_min();
    assertx(pq4.remove_min() == i);
    order.push(i);
  }
  assertx(pq4.empty());
  SHOW(order);
}

void test11() {  // Dijkstra on a grid, compared with a search that scans all the vertices for the closest one.
  const int n = 40;
  std::mt19937 gen(1);
  Array<float> weights(2 * n * n);  // Edges to the right and lower neighbors; integers, so that distances are exact.
  for (float& w : weights) w = float(1 + gen() % 1000);
  const auto for_neighbors = [&](int v, auto func) {  // Call func(w, weight) for each neighbor w of vertex v.
    const int y = v / n, x = v % n;
    if (x + 1 < n) func(v + 1, weights[2 * v]);
    if (x > 0) func(v - 1, weights[2 * (v - 1)]);
    if (y + 1 < n) func(v + n, weights[2 * v + 1]);
    if (y > 0) func(v - n, weights[2 * (v - n) + 1]);
  };
  Array<float> dist(n * n, BIGFLOAT);
  {
    UpdatablePriorityQueue<int> pq;
    dist[0] = 0.f;
    pq.enter(0, 0.f);
    while (!pq.empty()) {
      const float d = pq.min_priority();
      const int v = pq.remove_min();
      for_neighbors(v, [&](int w, float weight) {
        if (d + weight < dist[w]) dist[w] = d + weight, pq.enter_update_if_smaller(w, d + weight);
      });
    }
  }
  // Reference distances, from the original O(n^4) form of Dijkstra's algorithm on these n^2 vertices: instead of a
  // priority queue, each step scans all the unvisited vertices for the closest one, and then relaxes its neighbors.
  Array<float> dist_scan(n * n, BIGFLOAT);
  {
    Array<bool> done(n * n, false);
    dist_scan[0] = 0.f;
    for_int(iter, n * n) {
      int v = -1;
      for_int(u, n * n) {
        if (!done[u] && (v < 0 || dist_scan[u] < dist_scan[v])) v = u;
      }
      done[v] = true;
      for_neighbors(v, [&](int w, float weight) { dist_scan[w] = min(dist_scan[w], dist_scan[v] + weight); });
    }
  }
  assertx(dist == dist_scan);
  int64_t sum = 0;
  for (const float d : dist) sum += int64_t(d);
  SHOW(sum, dist.last());
}

void test12() {  // Random operations, compared with a brute-force model of the queue.
  std::mt19937 gen(2);
  UpdatablePriorityQueue<int> pq;
  Map<int, float> model;  // Element -> priority.
  const auto model_set = [&](int e, float pri) {
    bool is_new;
    model.enter(e, pri, is_new) = pri;
  };
  const auto model_get = [&](int e) {  // Returns the priority of e, or -1.f if e is absent.
    const float* p = model.find_ptr(e);
    return p ? *p : -1.f;
  };
  for_int(op, 100'000) {
    const int e = int(gen() % 50);
    const float pri = float(gen() % 20);  // Frequent ties.
    const float old_pri = model_get(e);
    switch (gen() % 6) {
      case 0:
        assertx(pq.enter_update(e, pri) == old_pri);
        model_set(e, pri);
        break;
      case 1:
        assertx(pq.update(e, pri) == old_pri);
        if (old_pri >= 0.f) model_set(e, pri);
        break;
      case 2:
        assertx(pq.remove(e) == old_pri);
        if (old_pri >= 0.f) model.remove(e);
        break;
      case 3: {
        const bool expected = old_pri < 0.f || pri < old_pri;
        assertx(pq.enter_update_if_smaller(e, pri) == expected);
        if (expected) model_set(e, pri);
        break;
      }
      case 4: {
        const bool expected = old_pri < 0.f || pri > old_pri;
        assertx(pq.enter_update_if_greater(e, pri) == expected);
        if (expected) model_set(e, pri);
        break;
      }
      default:
        if (!model.empty()) {
          float min_pri = BIGFLOAT;
          for (const float p : model.values()) min_pri = min(min_pri, p);
          assertx(pq.min_priority() == min_pri);
          const int emin = pq.remove_min();
          assertx(model.remove(emin) == min_pri);  // Any element with the minimum priority is valid.
        }
    }
    assertx(pq.num() == model.num());
    assertx(pq.retrieve(e) == model_get(e));
  }
  SHOW(pq.num());
}

}  // namespace

int main() {
  test1();
  test2();
  test3();
  test4();
  test5();
  test6();
  test7();
  test8();
  test9();
  test10();
  test11();
  test12();
}

template class hh::PriorityQueue<unsigned>;
template class hh::PriorityQueue<float>;
template class hh::PriorityQueue<float, 4>;
template class hh::PriorityQueue<unique_ptr<int>>;

template class hh::UpdatablePriorityQueue<unsigned>;
template class hh::UpdatablePriorityQueue<float>;
template class hh::UpdatablePriorityQueue<double*>;
