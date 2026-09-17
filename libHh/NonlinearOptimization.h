// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_NONLINEAROPTIMIZATION_H_
#define MESH_PROCESSING_LIBHH_NONLINEAROPTIMIZATION_H_

#include "libHh/Array.h"
#include "libHh/MathOp.h"  // my_mod()
#include "libHh/Matrix.h"

namespace hh {

// Nonlinear optimization: find the n-dimensional vector x that minimizes a function f(x) : R^n -> R.
// Given:
//  - a container vector x,
//  - a function eval that evaluates both f(x) and \grad f(x) == (df / dx_1, df / dx_2, ..., df / dx_n);
//     its signature is: double eval(ArrayView<double> ret_grad), where the return value is f(x).
// The member function solve() iteratively calls eval to minimizes f, starting from some initial guess x_0
//  provided in x, and places the obtained minimum in x.  It returns false if the solution fails to converge.
// The optimization iterates until machine-precision convergence, or until a maximum number of evaluations
//  provided using set_max_neval().
// Eval = double (&)(ArrayView<double>)
template <typename Eval> class NonlinearOptimization : noncopyable {
 public:
  explicit NonlinearOptimization(ArrayView<double> x, Eval eval = Eval()) : NonlinearOptimization(nullptr, x, eval) {
    _debug = getenv_int("NLOPT_DEBUG");
  }
  void set_max_neval(int max_neval) { _max_neval = max_neval; }  // Default is -1, which signifies infinity.
  [[nodiscard]] bool solve() { return solve_i(); }               // Returns success.
 private:
  // ** Approach-independent:
  ArrayView<double> _x;  // View of the user-supplied vector; stores the initial estimate and final solution.
  const int _n;          // _x.num().
  const Eval _eval;      // Callback function to evaluate both the value f and the gradient _grad at _x.
  int _max_neval{-1};    // Here, -1 is infinity.
  int _debug;            // 0 == no_output, 1 == show_final, 2 == show_each_iteration.
  void show_debug(const string& s, int iter, int neval, double f, double magg) {
    showf("NonlinearOptimization %s iter=%-3d neval=%-3d f=%.12g mag(g)=%.12g\n", s.c_str(), iter, neval, f, magg);
  }

  // ** Approach-dependent:
  // Simple version of Limited-memory Broyden-Fletcher-Goldfarb-Shanno algorithm.
  // See: https://en.wikipedia.org/wiki/Limited-memory_BFGS
  // Liu, D. C.; Nocedal, J. (1989). "On the limited memory method for large scale optimization".
  //  Mathematical Programming B 45 (3): 503-528.  doi:10.1007/BF01589116.
  // Nocedal (1980).  "Updating quasi-Newton matrices with limited storage".
  //  Mathematics of Computation 35(151), 773-782.  doi:10.1090/S0025-5718-1980-0572855-7.
  // Useful: https://web.archive.org/web/20231002054213/https://aria42.com/blog/2014/12/understanding-lbfgs
  // (e.g. fortran implementation in https://users.iems.northwestern.edu/~nocedal/lbfgs.html
  //   and threadsafe C version in https://github.com/chokkan/liblbfgs )
  // (_m not constexpr because V() later accesses it as a const reference)
  const int _m = 6;          // Number of stored prior gradients and differences; recommended range 3..7.
  const double _eps = 1e-6;  // mag(_g) < _eps*max(1., mag(_x)); default 1e-5 in github.com/chokkan/liblbfgs
  Array<double> _g;          // The current gradient.
  Array<double> _tmp;        // Temporary storage.
  Array<double> _xinit;      // Value of _x at the start of the line search.
  Array<double> _rho;        // Scalars \rho from [Nocedal 1980].
  Array<double> _alphak;     // Used in the formula that computes H * g.
  Matrix<double> _as;        // Last _m search step          (s_k = x_k - x_{k-1}).
  Matrix<double> _ay;        // Last _m gradient differences (y_k = g_k - g_{k-1}).
  NonlinearOptimization(void*, ArrayView<double> px, Eval eval)
      : _x(std::move(px)),
        _n(_x.num()),
        _eval(eval),
        _g(_n),
        _tmp(_n),
        _xinit(_n),
        _rho(_m),
        _alphak(_m),
        _as(V(_m, _n)),
        _ay(V(_m, _n)) {
    assertx(_n > 0 && _m > 0);
  }
  // Backtracking line search to find approximate minimum of f=_eval() along direction p,
  //  with initial step size alpha.  Updates number of evaluations neval.  Ret: success.
  [[nodiscard]] bool line_search(double& f, CArrayView<double> p, double& alpha, int& neval, int iter) {
    // https://en.wikipedia.org/wiki/Backtracking_line_search
    const double finit = f;  // The initial f.
    _xinit.assign(_x);       // The initial _x.
    const double c = .5;     // Factor for the Armijo-Goldstein condition.
    const double tau = .5;   // Backtracking factor (used in a geometric sequence on alpha).
    const double m = dot(_g, p);
    assertx(m < 0.);  // Must be a descent direction.
    for (;;) {
      for_int(i, _n) _x[i] = _xinit[i] + alpha * p[i];
      f = _eval(_g);  // Evaluate both the objective f and its gradient _g at _x.
      neval++;
      if (_debug >= 2) show_debug("", iter, neval, f, mag(_g));
      if (finit - f >= -alpha * (c * m)) return true;  // The Armijo-Goldstein condition is satisfied.
      alpha *= tau;
      if (alpha < 1e-10) {
        Warning("line_search fails to converge");
        return false;
      }
    }
  }
  bool solve_i() {
    int ic = 0;                          // Index into the circular buffers _as, _ay.
    double f = _eval(_g);                // Evaluate both the objective f and its gradient _g at _x.
    int neval = 1;                       // Number of times that _eval is called.
    for_int(i, _n) _as[ic, i] = -_g[i];  // The initial line search direction.
    double alpha = 1. / mag(_g);         // The initial step size.
    for_int(iter, 1000) {
      _tmp.assign(_g);  // Archive the current gradient.
      if (!line_search(f, _as[ic], alpha, neval, iter)) {
        if (_debug >= 1) show_debug("line_search_failed", iter, neval, f, mag(_g));
        return false;
      }
      if (_max_neval >= 0 && neval >= _max_neval) {  // Reached the maximum number of function evaluations?
        if (_debug >= 1) show_debug("max_neval", iter, neval, f, mag(_g));
        return true;
      }
      if (mag(_g) / max(mag(_x), 1.) <= _eps) {  // Numerically converged?
        if (_debug >= 1) show_debug("converged", iter, neval, f, mag(_g));
        return true;
      }
      _as[ic] *= alpha;                             // Record the search step.
      for_int(i, _n) _ay[ic, i] = _g[i] - _tmp[i];  // Record the gradient difference.
      const double ys = dot(_ay[ic], _as[ic]);
      const double yy = dot(_ay[ic], _ay[ic]);
      // Compute -H*_g using [Nocedal 1980].
      _rho[ic] = 1. / ys;
      ic = my_mod(ic + 1, _m);  // Roll the circular buffers.
      for_int(i, _n) _tmp[i] = -_g[i];
      const int mm = min(iter + 1, _m);
      for_int(i, mm) {
        ic = my_mod(ic - 1, _m);
        _alphak[ic] = _rho[ic] * dot(_as[ic], _tmp);
        _tmp += -_alphak[ic] * _ay[ic];
      }
      _tmp *= ys / yy;
      for_int(i, mm) {
        const double beta = _alphak[ic] - _rho[ic] * dot(_ay[ic], _tmp);
        _tmp += beta * _as[ic];
        ic = my_mod(ic + 1, _m);
      }
      _as[ic].assign(_tmp);  // The new search direction.
      alpha = 1.;            // The new step size.
    }
    assertnever("NonlinearOptimization fails unexpectedly");
  }
};

// Template deduction guide:
template <typename Eval> NonlinearOptimization(ArrayView<double>, Eval) -> NonlinearOptimization<Eval>;

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_NONLINEAROPTIMIZATION_H_
