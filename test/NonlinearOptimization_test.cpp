// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/NonlinearOptimization.h"
using namespace hh;

namespace {

Array<double> g_x;

int g_func;

double feval(ArrayView<double> ret_grad) {
  assertx(ret_grad.num() == g_x.num());
  double f;
  switch (g_func) {
    case 0: {  // Here, f : R -> R :  x -> square(x - .5) + sin(x) * .1.
      assertx(g_x.num() == 1);
      const double x = g_x[0];
      f = square(x - .5) + std::sin(x) * .1;
      ret_grad[0] = 2. * (x - .5) + std::cos(x) * .1;
      break;
    }
    case 1: {  // Here, f : R^2 -> R :  (x, y) -> (x - .3) ^ 2 + (y - .4) ^ 2.
      assertx(g_x.num() == 2);
      const double x0 = g_x[0], x1 = g_x[1];
      f = square(x0 - .3) + square(x1 - .4);
      ret_grad[0] = 2. * (x0 - .3);
      ret_grad[1] = 2. * (x1 - .4);
      break;
    }
    case 2: {  // Here, f : R^3 -> R.
      assertx(g_x.num() == 3);
      const double x = g_x[0], y = g_x[1], z = g_x[2];
      f = square(x - .3) + square(y - .4) + square(z - .7) + std::sin(x) + std::cos(y);
      ret_grad.assign(V(2. * (x - .3) + std::cos(x), 2. * (y - .4) - std::sin(y), 2. * (z - .7)));
      break;
    }
    default: assertnever("");
  }
  return f;
}

// The magnitude of the gradient of feval() at g_x.
double gradient_magnitude() {
  Array<double> grad(g_x.num());
  dummy_use(feval(grad));
  return mag(grad);
}

// The Rosenbrock function f(x, y) = (1 - x)^2 + 100 * (y - x^2)^2, with its curved valley and minimum at (1, 1).
// The backtracking line search does not enforce the curvature condition, so some steps s and gradient differences y
// have dot(y, s) < 0 (e.g. at iteration 6); the solver must then skip their L-BFGS update to keep the search
// directions descent directions.
void test_rosenbrock() {
  Array<double> x{-1.2, 1.};
  const auto eval = [&](ArrayView<double> ret_grad) {
    const double t1 = 1. - x[0], t2 = x[1] - square(x[0]);
    ret_grad[0] = -2. * t1 - 400. * x[0] * t2;
    ret_grad[1] = 200. * t2;
    return square(t1) + 100. * square(t2);
  };
  NonlinearOptimization opt(x, eval);
  assertx(opt.solve());
  assertx(dist(x, V(1., 1.)) < 1e-5);
}

// A separable quadratic f(x) = sum_i (i + 1) * (x_i - sin(i))^2 in n dimensions with different curvatures.
struct Quadratic {
  const Array<double>& x;
  int& neval;
  double operator()(ArrayView<double> ret_grad) const {
    neval++;
    double f = 0.;
    for_int(i, x.num()) {
      const double weight = i + 1., target = std::sin(i * 1.);
      f += weight * square(x[i] - target);
      ret_grad[i] = 2. * weight * (x[i] - target);
    }
    return f;
  }
};

// With 20 dimensions, the optimization requires many iterations, so the L-BFGS history buffers wrap around.
void test_quadratic() {
  const int n = 20;
  Array<double> x(n, 0.);
  int neval = 0;
  NonlinearOptimization opt(x, Quadratic{x, neval});
  assertx(opt.solve());
  for_int(i, n) assertx(abs(x[i] - std::sin(i * 1.)) < 1e-6);
  assertx(neval > 10);  // (The exact number of evaluations depends on floating-point roundoff.)
}

// The maximum number of evaluations limits the optimization.
void test_max_neval() {
  const int n = 20;
  Array<double> x(n);
  Array<double> grad(n);
  int neval = 0;
  const Quadratic quadratic{x, neval};
  fill(x, 0.);
  const double finit = quadratic(grad);
  for (const int max_neval : {1, 4, 8}) {
    fill(x, 0.);
    neval = 0;
    NonlinearOptimization opt(x, quadratic);
    opt.set_max_neval(max_neval);
    assertx(opt.solve());
    // The limit is checked only after each line search, which may require several evaluations.
    assertx(neval >= max_neval && neval < max_neval + 20);
    assertx(quadratic(grad) < finit);  // The function value has decreased.
    assertx(mag(grad) > 1e-3);         // The solution has not yet converged.
  }
}

}  // namespace

int main() {
  if (1) {
    SHOW("try1");
    g_func = 1;
    g_x = Array{6. / 11., 5. / 7.};
    // The deduction guide gives Eval == double (*)(ArrayView<double>).  (Eval == double(ArrayView<double>) fails.)
    NonlinearOptimization opt(g_x, feval);
    static_assert(std::is_same_v<decltype(opt), NonlinearOptimization<double (*)(ArrayView<double>)>>);
    const int max_neval = 5;
    opt.set_max_neval(max_neval);
    assertx(opt.solve());
    assertx(dist(g_x, V(.3, .4)) < 1e-6);  // The quadratic converges within few evaluations.
    g_x.assign(V(6. / 11., 5. / 7.));
    NonlinearOptimization<double (&)(ArrayView<double>)> opt2(g_x, feval);  // A function reference also works.
    assertx(opt2.solve());
    assertx(dist(g_x, V(.3, .4)) < 1e-6);
    // Starting at a stationary point (where the gradient is exactly zero), solve() succeeds immediately.
    g_x.assign(V(.3, .4));
    assertx(gradient_magnitude() == 0.);
    NonlinearOptimization opt3(g_x, feval);
    assertx(opt3.solve());
    assertx(g_x[0] == .3 && g_x[1] == .4);
  }
  if (1) {
    for_int(ifunc, 3) {
      SHOW(ifunc);
      g_func = ifunc;
      g_x.init(ifunc + 1, .5);
      struct Eval {
        double operator()(ArrayView<double> ret_grad) const { return feval(ret_grad); }
      };
      NonlinearOptimization<Eval> opt(g_x);
      assertx(opt.solve());
      switch (ifunc) {
        case 0: assertx(dist(g_x, V(0.45509f)) < 1e-4f); break;
        case 1: assertx(dist(g_x, V(0.3f, 0.4f)) < 1e-5f); break;
        case 2: assertx(dist(g_x, V(-0.19092f, 0.73547f, 0.7f)) < 1e-5f); break;
        default: assertnever("");
      }
      assertx(gradient_magnitude() < 1e-6);  // The solution is a stationary point.
    }
  }
  test_rosenbrock();
  test_quadratic();
  test_max_neval();
}
