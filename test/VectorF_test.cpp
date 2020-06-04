// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/VectorF.h"
using namespace hh;

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
}

template class hh::VectorF<37>;
