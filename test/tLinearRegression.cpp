// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "LinearRegression.h"

#include "ArrayOp.h"            // concat()
using namespace hh;

namespace {

template<int N> void try_xy(CArrayView<Vec2<float>> xydata) {
    using Eval = LinearRegressionPolynomialOrder<N>;
    LinearRegression<N, 1, Eval> regression(xydata.num());
    for (auto xy : xydata) {
        regression.enter(ArView(xy[0]), xy[1]);
    }
    auto ar = regression.get_solution(); SHOW(ar);
    for (auto xy : xydata) {
        float yfit = float(dot(ar, Eval()(ArView(xy[0]))));
        showf("x=%g  y=%g  yfit=%8g\n", xy[0], xy[1], yfit);
    }
}

template<int N> void try_xyz(CArrayView<Vec3<float>> xyzdata) {
    static constexpr int N2 = N*N;
    struct Eval {
        Vec<float,N2> operator()(const Vec2<float>& xy) const {
            Vec<float,N2> ar;
            Vec<float,N> xprod; { float prod = 1.f; for_int(i, N) { xprod[i] = prod; prod *= xy[0]; } }
            Vec<float,N> yprod; { float prod = 1.f; for_int(i, N) { yprod[i] = prod; prod *= xy[1]; } }
            for_int(i, N) for_int(j, N) { ar[i*N+j] = xprod[i]*yprod[j]; }
            return ar;
        }
    };
    LinearRegression<N2, 2, Eval> regression(xyzdata.num());
    for (auto xyz : xyzdata) {
        regression.enter(xyz.head<2>(), xyz[2]);
    }
    auto ar = regression.get_solution(); SHOW(ar);
    for (auto xyz : xyzdata) {
        float zfit = float(dot(ar, Eval()(xyz.head<2>())));
        showf("x=%g  y=%g  z=%8g  zfit=%8g\n", xyz[0], xyz[1], xyz[2], zfit);
    }
}

} // namespace

int main() {
    if (1) {
        const auto xydata = V(V(0.f, 4.f), V(1.f, 4.f), V(2.f, 5.f), V(3.f, 4.f));
        try_xy<2>(xydata);
        try_xy<3>(xydata);
        try_xy<4>(xydata);
    }
    if (1) {
        Array<Vec3<float>> xyzdata;
        const int n = 4;
        for_int(ix, n) for_int(iy, n) {
            auto xy = V(float(ix), float(iy));
            float z = float(mag(xy-V(n/2.f, n/2.f)));
            xyzdata.push(concat(xy.view(), ArView(z)));
        }
        try_xyz<2>(xyzdata);
        try_xyz<3>(xyzdata);
    }
}

template class hh::LinearRegression<4, 1, LinearRegressionPolynomialOrder<4>>;
