// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/VectorF.h"

#include "libHh/RangeOp.h"  // sum(), mag2(), dist2(), dot() on Vec.
using namespace hh;

namespace {

// Deterministic small integer values in [-8, 8], so that all sums and products are exact.
struct Lcg {
  uint32_t state = 1;
  float operator()() {
    state = state * 1'664'525u + 1'013'904'223u;
    return float(int(state >> 16) % 17 - 8);
  }
};

// Compare the operations of VectorF<n> against a scalar reference Vec<float, n>, element by element.
template <int n> void test_vs_reference() {
  using VecF = VectorF<n>;
  assertx(VecF().num() == n && VecF().size() == size_t{n});
  assertx(VecF::ok(0) && VecF::ok(n - 1) && !VecF::ok(-1) && !VecF::ok(n));
  Lcg lcg;
  for_int(iter, 20) {
    Vec<float, n> a, b;
    for_int(i, n) a[i] = lcg(), b[i] = lcg();
    for_int(i, n) if (b[i] == 0.f) b[i] = 4.f;  // Avoid division by zero.
    VecF va, vb;
    va.load_unaligned(a.data());
    for_int(i, n) vb[i] = b[i];
    for_int(i, n) assertx(va[i] == a[i] && vb.data()[i] == b[i]);
    const auto verify = [&](const VecF& v, auto func) { for_int(i, n) assertx(v[i] == func(i)); };
    verify(va + vb, [&](int i) { return a[i] + b[i]; });
    verify(va - vb, [&](int i) { return a[i] - b[i]; });
    verify(va * vb, [&](int i) { return a[i] * b[i]; });
    verify(va / vb, [&](int i) { return a[i] / b[i]; });
    verify(va + 3.f, [&](int i) { return a[i] + 3.f; });
    verify(va - 3.f, [&](int i) { return a[i] - 3.f; });
    verify(va * 3.f, [&](int i) { return a[i] * 3.f; });
    verify(3.f * va, [&](int i) { return a[i] * 3.f; });
    verify(va / 4.f, [&](int i) { return a[i] * (1.f / 4.f); });  // Implemented as multiplication by 1.f / f.
    verify(min(va, vb), [&](int i) { return min(a[i], b[i]); });
    verify(max(va, vb), [&](int i) { return max(a[i], b[i]); });
    assertx(dot(va, vb) == dot(a, b));
    assertx(mag2(va) == mag2(a));
    assertx(dist2(va, vb) == dist2(a, b));
    assertx(sum(va) == sum(a));
    VecF vc = va;
    vc += vb;
    verify(vc, [&](int i) { return a[i] + b[i]; });
    vc -= vb;
    verify(vc, [&](int i) { return a[i]; });
    vc *= vb;
    verify(vc, [&](int i) { return a[i] * b[i]; });
    vc /= vb;
    verify(vc, [&](int i) { return a[i]; });
    vc *= 2.f;
    vc /= 2.f;
    verify(vc, [&](int i) { return a[i]; });
    vc += 2.f;
    verify(vc, [&](int i) { return a[i] + 2.f; });
    vc -= 2.f;
    verify(vc, [&](int i) { return a[i]; });
    {
      float sum_iter = 0.f;
      int count = 0;
      for (const float f : va) sum_iter += f, count++;
      assertx(count == n && sum_iter == sum(a));
    }
    {
      alignas(16) float buf[n + 4];
      vb.store_aligned(buf);
      VecF vd;
      vd.load_aligned(buf);
      verify(vd, [&](int i) { return b[i]; });
      va.store_unaligned(buf + 1);  // Misaligned.
      vd.load_unaligned(buf + 1);
      verify(vd, [&](int i) { return a[i]; });
    }
    vc.zero();
    verify(vc, [](int) { return 0.f; });
    vc.fill(-2.5f);
    verify(vc, [](int) { return -2.5f; });
    verify(VecF(1.5f), [](int) { return 1.5f; });
  }
}

}  // namespace

int main() {
  {
    using VecF = VectorF<11>;
    Array<float> avec;
    for_int(i, 11) avec.push(float(i * 2 + 17));
    VecF v2;
    v2.load_unaligned(avec.data());
    VecF v3(v2);
    VecF v1;
    v1 = v2 + v3;
    SHOW(v2);
    SHOW(v3);
    SHOW(v1);
    SHOW(v2 - v3);
    SHOW(dot(v1, v3));
    v1 -= v3;
    SHOW(v1);
    SHOW(sum(v1));
    SHOW(mag2(v1));
    SHOW(mag2(v1 / 2.f));
    SHOW(dist2(v1, v1 / 2.f));
    SHOW(min(v1, v2 + VecF(2.f)));
    SHOW(max(v1, v2 + VecF(3.f)));
    SHOW(v1[7]);
    SHOW(v1[8]);
    SHOW(v1[9]);
    SHOW(v1[10]);
  }
  {
    VectorF<1> v1;
    SHOW(sizeof(v1));
    dummy_use(v1);
    VectorF<2> v2;
    SHOW(sizeof(v2));
    dummy_use(v2);
    VectorF<3> v3;
    SHOW(sizeof(v3));
    dummy_use(v3);
    VectorF<4> v4;
    SHOW(sizeof(v4));
    dummy_use(v4);
    VectorF<5> v5;
    SHOW(sizeof(v5));
    dummy_use(v5);
    VectorF<6> v6;
    SHOW(sizeof(v6));
    dummy_use(v6);
    VectorF<7> v7;
    SHOW(sizeof(v7));
    dummy_use(v7);
    VectorF<8> v8;
    SHOW(sizeof(v8));
    dummy_use(v8);
    VectorF<9> v9;
    SHOW(sizeof(v9));
    dummy_use(v9);
  }
  test_vs_reference<1>();
  test_vs_reference<3>();
  test_vs_reference<4>();
  test_vs_reference<5>();
  test_vs_reference<8>();
  test_vs_reference<11>();
  test_vs_reference<37>();
  showf("VectorF<n> operations match the scalar reference.\n");
}

template class hh::VectorF<1>;
template class hh::VectorF<2>;
template class hh::VectorF<3>;
template class hh::VectorF<4>;
template class hh::VectorF<5>;
template class hh::VectorF<8>;
template class hh::VectorF<9>;
template class hh::VectorF<37>;
