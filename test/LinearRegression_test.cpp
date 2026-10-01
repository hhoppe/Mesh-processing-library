// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/LinearRegression.h"

#include "libHh/RangeOp.h"  // max_abs_element()

using namespace hh;

namespace {

// Verify that the least-squares residual is orthogonal to each basis function (i.e., the normal equations hold).
template <int N, typename Eval, int D>
void verify_normal_equations(CArrayView<Vec<float, D + 1>> data, const Vec<float, N>& ar) {
  Vec<double, N> sums{};
  double scale = 0.;
  for (const auto& p : data) {
    const Vec<float, N> basis = Eval()(p.template head<D>());
    const double residual = dot(convert<double>(ar), convert<double>(basis)) - p[D];
    for_int(k, N) sums[k] += residual * basis[k];
    scale += mag2(convert<double>(basis));
  }
  assertx(max_abs_element(sums) < 1e-5 * scale);
}

template <int N> void try_xy(CArrayView<Vec2<float>> xydata) {
  using Eval = LinearRegressionPolynomialOrder<N>;
  LinearRegression<N, 1, Eval> regression(xydata.num());
  for (auto xy : xydata) regression.enter(xy.head<1>(), xy[1]);
  auto ar = regression.get_solution();
  SHOW(ar);
  for (auto xy : xydata) {
    const float yfit = float(dot(ar, Eval()(xy.head<1>())));
    showf("x=%g  y=%g  yfit=%8g\n", xy[0], xy[1], yfit);
  }
  verify_normal_equations<N, Eval, 1>(xydata, ar);
}

template <int N> void try_xyz(CArrayView<Vec3<float>> xyzdata) {
  static constexpr int N2 = N * N;
  struct Eval {
    Vec<float, N2> operator()(const Vec2<float>& xy) const {
      Vec<float, N2> ar;
      Vec<float, N> xprod;
      {
        float prod = 1.f;
        for_int(i, N) {
          xprod[i] = prod;
          prod *= xy[0];
        }
      }
      Vec<float, N> yprod;
      {
        float prod = 1.f;
        for_int(i, N) {
          yprod[i] = prod;
          prod *= xy[1];
        }
      }
      for_int(i, N) for_int(j, N) ar[i * N + j] = xprod[i] * yprod[j];
      return ar;
    }
  };
  LinearRegression<N2, 2, Eval> regression(xyzdata.num());
  for (auto xyz : xyzdata) regression.enter(xyz.head<2>(), xyz[2]);
  const auto ar = regression.get_solution();
  SHOW(ar);
  for (auto xyz : xyzdata) {
    const float zfit = float(dot(ar, Eval()(xyz.head<2>())));
    showf("x=%g  y=%g  z=%8.5g  zfit=%8.5g\n", xyz[0], xyz[1], xyz[2], zfit);
  }
  verify_normal_equations<N2, Eval, 2>(xyzdata, ar);
}

void test_polynomial_basis() {
  const Vec4<float> basis = LinearRegressionPolynomialOrder<4>()(V(2.f));
  SHOW(basis);
  assertx(basis == V(1.f, 2.f, 4.f, 8.f));
  assertx(LinearRegressionPolynomialOrder<1>()(V(5.f)) == V(1.f));
}

// Data sampled exactly from a function in the span of the basis is reproduced exactly.
void test_exact_recovery() {
  {
    // The quadratic y = 1 - 2 * x + .5 * x^2.
    const Vec3<float> coefs(1.f, -2.f, .5f);
    using Eval = LinearRegressionPolynomialOrder<3>;
    const int m = 6;
    LinearRegression<3, 1, Eval> regression(m);
    for_int(i, m) {
      const float x = i * .5f - 1.f;
      regression.enter(V(x), dot(coefs, Eval()(V(x))));
    }
    const Vec3<float> ar = regression.get_solution();
    assertx(max_abs_element(ar - coefs) < 1e-5f);
  }
  {
    // With N == m, the polynomial interpolates the data.
    using Eval = LinearRegressionPolynomialOrder<3>;
    LinearRegression<3, 1, Eval> regression(3);
    const auto xydata = V(V(-1.f, 2.f), V(0.f, 1.f), V(2.f, 5.f));
    for (const auto& xy : xydata) regression.enter(xy.head<1>(), xy[1]);
    const Vec3<float> ar = regression.get_solution();  // The parabola y = x^2 + 1.
    assertx(max_abs_element(ar - V(1.f, 0.f, 1.f)) < 1e-5f);
  }
  {
    // A plane z = 3 + 2 * x - y in two dimensions, using a captureless lambda as the Eval functor.
    // KNOWN_BUG: the default Eval of LinearRegression is a function type, which cannot be a data member, so an
    // explicit Eval is required.
    using Eval = decltype([](const Vec2<float>& p) { return V(1.f, p[0], p[1]); });
    LinearRegression<3, 2, Eval> regression(9);
    for_int(i, 3) for_int(j, 3) {
      const float x = float(i), y = float(j * 2 - 1);
      regression.enter(V(x, y), 3.f + 2.f * x - y);
    }
    const Vec3<float> ar = regression.get_solution();
    assertx(max_abs_element(ar - V(3.f, 2.f, -1.f)) < 1e-5f);
  }
  {
    // A constant fit (N == 1) is the mean of the values.
    using Eval = LinearRegressionPolynomialOrder<1>;
    LinearRegression<1, 1, Eval> regression(4);
    for (const float y : {1.f, 2.f, 4.f, 9.f}) regression.enter(V(0.f), y);
    const Vec1<float> ar = regression.get_solution();
    assertx(abs(ar[0] - 4.f) < 1e-6f);
  }
}

}  // namespace

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
      const auto xy = V(float(ix), float(iy));
      const float z = mag(xy - V(n / 2.f, n / 2.f));
      xyzdata.push(concat(xy, V(z)));
    }
    try_xyz<2>(xyzdata);
    try_xyz<3>(xyzdata);
  }
  test_polynomial_basis();
  test_exact_recovery();
}

template class hh::LinearRegression<4, 1, LinearRegressionPolynomialOrder<4>>;
