// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_LINEARREGRESSION_H_
#define MESH_PROCESSING_LIBHH_LINEARREGRESSION_H_

#include "libHh/Lls.h"

#if 0
{
  const auto xydata = {V(0.f, 4.f), V(1.f, 4.f), V(2.f, 5.f), V(3.f, 4.f)};
  using Eval = LinearRegressionPolynomialOrder<2>;
  LinearRegression<2, 1, Eval> regression(xydata.num());
  for (auto xy : xydata) regression.enter(xy.head<1>(), xy[1]);
  auto ar = regression.get_solution();
  for (auto xy : xydata) showf("x=%g y=%g yfit=%g\n", xy[0], xy[1], dot(ar, Eval()(xy.head<1>())));
}
#endif

namespace hh {

template <int N> struct LinearRegressionPolynomialOrder {
  static constexpr int D = 1;
  Vec<float, N> operator()(const Vec<float, D>& p) const {
    Vec<float, N> ar;
    float prod = 1.f;
    for_int(i, N) {
      ar[i] = prod;
      prod *= p[0];
    }
    return ar;
  }
};

// Given m input points of dimension D,
//  we want to fit a function Eval that linearly combines N terms which are functions of the input,
//  each multiplied by an unknown coefficient c.
// The goal is to find c by least-squares minimization.
template <int N, int D, typename Eval = Vec<float, N>(const Vec<float, D>&)> class LinearRegression {
 public:
  explicit LinearRegression(int m) : _lls(m, N, 1) {}
  void enter(const Vec<float, D>& p, float val) {
    assertx(_row < _lls.num_rows());
    Vec<float, N> ar = _eval(p);
    _lls.enter_a_r(_row, ar);
    _lls.enter_b_r(_row, ArView(val));
    _row++;
  }
  Vec<float, N> get_solution() {
    assertx(_lls.solve());
    Vec<float, N> ar;
    _lls.get_x_c(0, ar);
    return ar;
  }

 private:
  SvdDoubleLls _lls;
  Eval _eval;
  int _row{0};
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_LINEARREGRESSION_H_
