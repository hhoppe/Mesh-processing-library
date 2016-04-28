// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "NonlinearOptimization.h"
using namespace hh;

namespace {

Array<double> g_x;

int g_func;

double feval(ArrayView<double> ret_grad) {
    assertx(ret_grad.num()==g_x.num());
    double f;
    switch (g_func) {
     bcase 0: {                 // f : R -> R :  x -> square(x-.5)+sin(x)*.1
         assertx(g_x.num()==1); const double x = g_x[0];
         f = square(x-.5) + sin(x)*.1;
         ret_grad[0] = 2.*(x-.5) + cos(x)*.1;
     }
     bcase 1: {                 // f : R^2 -> R :  (x, y) -> (x-.3)^2 + (y-.4)^2
         assertx(g_x.num()==2);
         const double x0 = g_x[0], x1 = g_x[1];
         f = square(x0-.3)+square(x1-.4);
         ret_grad[0] = 2.*(x0-.3);
         ret_grad[1] = 2.*(x1-.4);
     }
     bcase 2: {                 // f : R^3 -> R
         assertx(g_x.num()==3); const double x = g_x[0], y = g_x[1], z = g_x[2];
         f = square(x-.3) + square(y-.4) + square(z-.7) + sin(x) + cos(y);
         ret_grad.assign(V(2.*(x-.3) + cos(x),
                           2.*(y-.4) - sin(y),
                           2.*(z-.7)));
     }
     bdefault: assertnever("");
    }
    if (1) {
        // was %18.12g but then get differences between different CONFIG
        string sx; for_int(i, g_x.num()) { sx += sform("%s %11.5g", (i ? "," : ""), g_x[i]); }
        showf("x=(%s) f=%11.5g mag(g)=%11.5g)\n", sx.c_str(), f, mag(ret_grad));
    }
    return f;
}

} // namespace

int main() {
    if (1) {
        SHOW("try1");
        g_func = 1;
        g_x = { 6./11., 5./7. };
        // NonlinearOptimization<double(ArrayView<double>)> opt(g_x, feval); // fails
        // NonlinearOptimization<double (*)(ArrayView<double>)> opt(g_x, feval); // works
        // NonlinearOptimization<double (&)(ArrayView<double>)> opt(g_x, feval); // works
        // NonlinearOptimization<decltype(&feval)> opt(g_x, feval); // works
        NonlinearOptimization<> opt(g_x, feval); // works
        const int niter = 5;
        opt.set_max_neval(niter);
        assertx(opt.solve());
    }
    if (1) {
        for_int(ifunc, 3) {
            SHOW(ifunc);
            g_func = ifunc;
            g_x.init(ifunc+1, .5);
            struct Eval {
                double operator()(ArrayView<double> ret_grad) const { return feval(ret_grad); }
            };
            NonlinearOptimization<Eval> opt(g_x);
            assertx(opt.solve());
        }
    }
}
