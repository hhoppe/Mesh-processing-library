// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Pqueue.h"

#include <random>  // std::default_random_engine

using namespace hh;

namespace {

void test1() {
#if 0  // test noncopyable
  {
    Pqueue<int> pq;
    Pqueue<int> pq2 = pq;
    dummy_use(pq2);
  }
#endif
#if 0  // test noncopyable
  {
    Pqueue<int> pq;
    Pqueue<int> pq2 = std::move(pq);
    dummy_use(pq2);
  }
#endif
}

void test2() {
  {
    Pqueue<int> pq;
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
      pq.min();
      assertw(!pq.empty());
      assertw(pq.num() == 200 - i);
      assertw(pq.remove_min() == (i % 2 ? i / 2 : 100 + i / 2));
    }
  }
  {
    HPqueue<int> pq;
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
      pq.min();
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
  HPqueue<int> pq;
  pq.reserve(n);
  for_int(i, n) pq.enter_unsorted(i, 2.f + std::sin(i * 7.f));
  pq.sort();
  float a = 0.f;
  while (!pq.empty()) {
    float b = pq.min_priority();
    assertx(b >= a);
    a = b;
    pq.remove_min();
  }
}

void test4() {
  Pqueue<int> pq;
  for_int(i, 1000) pq.enter(i, 2.f + std::sin(float(i)));
  float a = 0.f;
  while (!pq.empty()) {
    float b = pq.min_priority();
    assertx(b >= a);
    a = b;
    pq.remove_min();
  }
}

void test5() {
  std::default_random_engine dre;
  for_int(itest, 500) {
    const int n = itest < 30 ? itest : 30 + dre() % 400;
    HPqueue<int> pq;
    for_int(i, n) pq.enter_unsorted(i, float(dre()));
    pq.sort();
    float a = 0.f;
    while (!pq.empty()) {
      float b = pq.min_priority();
      assertx(b >= a);
      a = b;
      pq.remove_min();
    }
  }
}

void test6() {
  std::default_random_engine dre;
  for_int(itest, 100) {
    const int n = 70;
    HPqueue<int> pq;
    for_int(i, n) pq.enter_unsorted(i, float(dre()));
    pq.sort();
    for_int(i, n * 3) pq.update(i, float(dre()));
    float a = 0.f;
    while (!pq.empty()) {
      float b = pq.min_priority();
      assertx(b >= a);
      a = b;
      pq.remove_min();
    }
  }
}

void test7() {
  for_int(k, 500) {
    int n = 30;
    HPqueue<int> pq;
    pq.reserve(n);
    Array<float> arval1;
    for_int(i, n) arval1.push(2.f + std::sin(i * 11.f + k * 1.2345f));
    Array<float> arval2;
    for_int(i, n) arval2.push(2.f + std::sin(i * 13.f + k * 2.7419f));
    Array<float> arval3;
    for_int(i, n) arval3.push(2.f + std::sin(i * 13.f + k * 3.1415f));
    for_int(i, n) pq.enter_unsorted(i, arval1[i]);
    pq.sort();
    for_int(i, n) assertx(pq.update(i, arval2[i]) == arval1[i]);
    for_int(i, n) assertx(pq.update(i, arval3[i]) == arval2[i]);
    for_int(i, n) assertx(pq.retrieve(i) == arval3[i]);
    float a = 0.f;
    while (!pq.empty()) {
      float b = pq.min_priority();
      assertx(b >= a);
      a = b;
      int i = pq.remove_min();
      assertx(b == arval3[i]);
    }
  }
}

void test8() {
  Pqueue<unique_ptr<int>> pq;
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
    unique_ptr<int> p = pq.remove_min();
    assertx(*p == 6);
  }
  {
    unique_ptr<int> p = pq.remove_min();
    assertx(*p == 4);
  }
  pq.clear();
  pq.enter_unsorted(make_unique<int>(3), 1.5f);
  pq.enter_unsorted(make_unique<int>(4), 1.3f);
  pq.enter_unsorted(make_unique<int>(5), 1.6f);
  pq.enter_unsorted(make_unique<int>(6), 1.2f);
  pq.enter_unsorted(make_unique<int>(7), 1.7f);
  pq.enter_unsorted(make_unique<int>(8), 1.8f);
  pq.sort();
  assertx(*pq.min() == 6);
  assertx(pq.min_priority() == 1.2f);
  {
    unique_ptr<int> p = pq.remove_min();
    assertx(*p == 6);
  }
  {
    unique_ptr<int> p = pq.remove_min();
    assertx(*p == 4);
  }
}

void test9() {
  const int ntests = 100;
  const int n = 19;
  std::default_random_engine dre;
  for_int(k, ntests) {
    Array<int> ar1;
    for_int(i, n) ar1.push(i);
    std::shuffle(ar1.begin(), ar1.end(), dre);
    Array<float> ar2;
    for_int(i, n) ar2.push(float(i));
    std::shuffle(ar2.begin(), ar2.end(), dre);
    Array<float> ar3;
    for_int(i, n) ar3.push(float(i));
    std::shuffle(ar3.begin(), ar3.end(), dre);
    HPqueue<int> hpq;
    for_int(i, n) {
      int j = ar1[i];
      hpq.enter(j, ar2[j]);
    }
    for_int(i, n) {
      int j = i;
      hpq.update(j, ar3[j]);
    }
    {
      float expectedv = 0.f;
      while (!hpq.empty()) {
        float pri = hpq.min_priority();
        int j = hpq.remove_min();
        assertx(pri == expectedv);
        assertx(ar3[j] == pri);
        expectedv += 1.f;
      }
      assertx(expectedv == float(n));
    }
  }
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
}

template class hh::Pqueue<unsigned>;
template class hh::Pqueue<float>;
