// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Array.h"
#include "Matrix.h"
#include "MathOp.h"             // my_mod()

namespace hh {

// Nonlinear optimization: find n-dimensional vector x that minimize a function f(x) : R^n -> R
// Given:
//  - a container vector x,
//  - a function eval that evaluates both f(x) and \grad f(x)==(df/dx_1, df/dx_2, ..., df/dx_n);
//     its signature is: double eval(ArrayView<double> ret_grad), where the return value is f(x).
// The member function solve() iteratively calls eval to minimizes f, starting from some initial guess x_0
//  provided in x, and places the obtained minimum in x.  It returns false if the solution fails to converge.
// The optimization iterates until machine-precision convergence, or until a maximum number of evaluations
//  provided using set_max_neval().
template<typename Eval = double (&)(ArrayView<double>)> class NonlinearOptimization : noncopyable {
 public:
    // renamed x to x_ because of VS2015 bug warning "C4459: declaration of 'x' hides global declaration"
    NonlinearOptimization(ArrayView<double> x_) : NonlinearOptimization(x_, Eval()) { }
    NonlinearOptimization(ArrayView<double> x_, Eval eval)
        : NonlinearOptimization(nullptr, x_, eval) { _debug = getenv_int("NLOPT_DEBUG"); }
    void set_max_neval(int max_neval)           { _max_neval = max_neval; } // default is -1 which signifies infinity
    bool solve()                                { return solve_i(); }       // ret: success
 private:
// Approach-independent:
    ArrayView<double> _x;       // view of user-supplied vector; stores initial estimate and final solution
    const int _n;               // _x.num()
    const Eval _eval;           // callback function to evaluate both the value f and gradient _grad at _x
    int _max_neval {-1};        // -1 is infinity
    int _debug;                 // 0==no_output, 1==show_final, 2==show_each_iteration
    void show_debug(const string& s, int iter, int neval, double f, double magg) {
        showdf("NonlinearOptimization %s iter=%-3d neval=%-3d f=%.12g mag(g)=%.12g\n", s.c_str(),
               iter, neval, f, magg);
    }
// Approach-dependent:
    // Simple version of Limited-memory Broyden-Fletcher-Goldfarb-Shanno algorithm.
    // See: https://en.wikipedia.org/wiki/Limited-memory_BFGS
    // Liu, D. C.; Nocedal, J. (1989). "On the limited memory method for large scale optimization".
    //  Mathematical Programming B 45 (3): 503-528.  doi:10.1007/BF01589116.
    // Nocedal (1980).  "Updating quasi-Newton matrices with limited storage".
    //  Mathematics of Computation 35(151), 773-782.  doi:10.1090/S0025-5718-1980-0572855-7.
    // Useful: http://aria42.com/blog/2014/12/understanding-lbfgs/
    // (e.g. fortran implementation in http://users.iems.northwestern.edu/~nocedal/lbfgs.html
    //   and thread-safe C version in https://github.com/chokkan/liblbfgs)
    // (_m not constexpr because V() later accesses it as a const reference)
    const int _m = 6;          // number of stored prior gradients and differences; recommended range 3..7
    const double _eps = 1e-6;  // mag(_g) < _eps*max(1., mag(_x)); default 1e-5 in github.com/chokkan/liblbfgs
    Array<double> _g;          // current gradient
    Array<double> _tmp;        // temporary storage
    Array<double> _xinit;      // _x at start of line search
    Array<double> _rho;        // scalars \rho from [Nocedal 1980]
    Array<double> _alphak;     // used in the formula that computes H*g
    Matrix<double> _as;        // last _m search step          (s_k = x_k - x_{k-1})
    Matrix<double> _ay;        // last _m gradient differences (y_k = g_k - g_{k-1})
    NonlinearOptimization(std::nullptr_t, ArrayView<double> px, Eval eval) :
        _x(px), _n(_x.num()), _eval(eval),
        _g(_n), _tmp(_n), _xinit(_n), _rho(_m), _alphak(_m), _as(V(_m, _n)), _ay(V(_m, _n)) {
        assertx(_n>0 && _m>0);
    }
    // Backtracking line search to find approximate minimum of f=_eval() along direction p,
    //  with initial step size alpha.  Updates number of evaluations neval.  Ret: success.
    bool line_search(double& f, CArrayView<double> p, double& alpha, int &neval, int iter) {
        // https://en.wikipedia.org/wiki/Backtracking_line_search
        const double finit = f; // initial f
        _xinit.assign(_x);      // initial _x
        const double c = .5;    // factor for Armijo-Goldstein condition
        const double tau = .5;  // backtracking factor (used in geometric sequence on alpha)
        const double m = dot(_g, p);
        assertx(m<0.);          // must be a descent direction
        for (;;) {
            for_int(i, _n) _x[i] = _xinit[i]+alpha*p[i];
            f = _eval(_g);      // evaluate both objective f and its gradient _g at _x
            neval++;
            if (_debug>=2) show_debug("", iter, neval, f, mag(_g));
            if (finit-f>=-alpha*(c*m)) return true; // Armijo-Goldstein condition satisfied
            alpha *= tau;
            if (alpha<1e-10) { Warning("line_search fails to converge"); return false; }
        }
    }
    bool solve_i() {
        int ic = 0;                         // index into circular buffers _as, _ay
        double f = _eval(_g);               // evaluate both the objective f and its gradient _g at _x
        int neval = 1;                      // number of times that _eval is called
        for_int(i, _n) _as[ic][i] = -_g[i]; // initial line search direction
        double alpha = 1./mag(_g);          // initial step size
        for_int(iter, 1000) {
            _tmp.assign(_g);    // archive the current gradient
            if (!line_search(f, _as[ic], alpha, neval, iter)) {
                if (_debug>=1) show_debug("line_search_failed", iter, neval, f, mag(_g));
                return false;
            }
            if (_max_neval>=0 && neval>=_max_neval) { // reached maximum number of function evaluations?
                if (_debug>=1) show_debug("max_neval", iter, neval, f, mag(_g));
                return true;
            }
            if (mag(_g)/max(mag(_x), 1.)<=_eps) { // numerically converged?
                if (_debug>=1) show_debug("converged", iter, neval, f, mag(_g));
                return true;
            }
            _as[ic] *= alpha;                          // record the search step
            for_int(i, _n) _ay[ic][i] = _g[i]-_tmp[i]; // record the gradient difference
            const double ys = dot(_ay[ic], _as[ic]);
            const double yy = dot(_ay[ic], _ay[ic]);
            // Compute -H*_g using [Nocedal 1980].
            _rho[ic] = 1./ys;
            ic = my_mod(ic+1, _m); // roll the circular buffers
            for_int(i, _n) _tmp[i] = -_g[i];
            const int mm = min(iter+1, _m);
            for_int(i, mm) {
                ic = my_mod(ic-1, _m);
                _alphak[ic] = _rho[ic]*dot(_as[ic], _tmp);
                _tmp += -_alphak[ic]*_ay[ic];
            }
            _tmp *= ys/yy;
            for_int(i, mm) {
                const double beta = _alphak[ic]-_rho[ic]*dot(_ay[ic], _tmp);
                _tmp += beta*_as[ic];
                ic = my_mod(ic+1, _m);
            }
            _as[ic].assign(_tmp); // new search direction
            alpha = 1.;           // new step size
        }
        assertnever("NonlinearOptimization fails unexpectedly");
    }
};

} // namespace hh
