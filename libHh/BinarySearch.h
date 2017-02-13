// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BINARYSEARCH_H_
#define MESH_PROCESSING_LIBHH_BINARYSEARCH_H_

#include "Array.h"

namespace hh {

// Given xl<xh, feval(xl)<=y_desired<feval(xh), find x such that feval(x)=y_desired within some tolerance.
// More precisely, find x such that exists x', x<=x'<x+xtol, feval(x')==y_desired .
template<typename T1, typename T2, typename Func = T2(const T1&)>
T1 continuous_binary_search_func(Func feval, T1 xl, T1 xh, T1 xtol, T2 y_desired) {
    static_assert(std::is_floating_point<T1>::value, "");
    if (0) showdf("xl=%.9g xh=%.9g f(xl)=%.9g f(xh)=%.9g\n", xl, xh, feval(xl), feval(xh));
    assertx(xl<xh);
    for (;;) {
        ASSERTXX(xl<xh && feval(xl)<=y_desired && y_desired<feval(xh));
        if (xh-xl<xtol) return xl;
        T1 xm = (xl+xh)/2;
        T2 ym = feval(xm);
        if (y_desired>=ym)
            xl = xm;
        else
            xh = xm;
    }
}

// Given xl<xh, feval(xl)<=y_desired<feval(xh), find x such that feval(x)<=y_desired<feval(x+1) .
template<typename T1, typename T2, typename Func = T2(const T1&)>
T1 discrete_binary_search_func(Func feval, T1 xl, T1 xh, T2 y_desired) {
    static_assert(std::is_integral<T1>::value, "");
    if (0) showdf("xl=%d xh=%d f(xl)=%.9g f(xh)=%.9g\n", xl, xh, feval(xl), feval(xh));
    assertx(xl<xh);
    for (;;) {
        ASSERTXX(xl<xh && feval(xl)<=y_desired && y_desired<feval(xh));
        if (xh-xl==1) return xl;
        T1 xm = (xl+xh)/2;
        T2 ym = feval(xm);
        if (y_desired>=ym)
            xl = xm;
        else
            xh = xm;
    }
}

// Given xl<xh, ar[xl]<=y_desired<ar[xh], Find x such that ar[x]<=y_desired<ar[x+1] .
template<typename T>
int discrete_binary_search(CArrayView<T> ar, int xl, int xh, T y_desired) {
    if (0) showdf("xl=%d xh=%d f(xl)=%.9g f(xh)=%.9g\n", xl, xh, double(ar[xl]), double(ar[xh]));
    assertx(xl<xh);
    assertx(ar[xl]<=y_desired && y_desired<ar[xh]);
    // if (1) return std::lower_bound(&ar[xl], &ar[xh]+1, y_desired)-ar.data(); // untested
    for (;;) {
        ASSERTX(xl<xh && ar[xl]<=y_desired && y_desired<ar[xh]);
        if (xh-xl==1) return xl;
        int xm = (xl+xh)/2;
        if (y_desired>=ar[xm])
            xl = xm;
        else
            xh = xm;
    }
}

// See also STL function to find an element within a sorted list:
//  bool std::binary_search(iterator ibegin, iterator iend, const T& elem);

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_BINARYSEARCH_H_
