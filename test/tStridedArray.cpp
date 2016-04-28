// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "StridedArrayView.h"
#include "Array.h"
#include "RangeOp.h"
using namespace hh;

int main() {
    {
        Array<int> ar(200, 0);
        assertx(sum(ar)==0);
        StridedArrayView<int> ar10(ar.data(), 5, 10); assertx(&ar10[3]==&ar[30]);
        for (int& e : ar10) { e = 7; }
        assertx(sum(ar)==5*7);
        CStridedArrayView<int> ar10c(ar.data(), 5, 10); assertx(&ar10c[3]==&ar[30]);
        for (int e : ar10c) { assertx(e==7); }
    }
}
