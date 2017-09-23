// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Filter.h"

#include <mutex>                // std::once_flag, std::call_once()

#include "Stat.h"
#include "MathOp.h"             // gaussian()
#include "StringOp.h"

namespace hh {

namespace {

using AF = Array<float>;

const LUfactorization g_reflected_spline_lu_factorization = LUfactorization(
    AF{ 100.f, 0.2f, 0.26315789f, 0.26760563f, 0.26792453f, 0.26794742f, 0.26794907f, 0.26794918f,
            0.26794919f, 0.26794919f },
    AF{ },
    AF{ },
    AF{ 1.2f, 1.5789474f, 1.6056338f, 1.6075472f, 1.6076845f, 1.6076944f, 1.6076951f, 1.6076952f,
            1.6076952f, 1.6076952f },
    AF{ 1.f, 1.25f, 1.2666667f, 1.2678571f, 1.2679426f, 1.2679487f, 1.2679492f, 1.2679492f, 1.2679492f,
            1.2679492f, 1.2679492f },
    AF{ },
    AF{ },
    0.16666667f
    );

const LUfactorization g_periodic_spline_lu_factorization = LUfactorization(
    AF{ 100.f, 0.25f, 0.26666667f, 0.26785714f, 0.26794258f, 0.26794872f, 0.26794916f, 0.26794919f,
            0.26794919f, 0.26794919f },
    AF{ 0.25f, -0.066666667f, 0.017857143f, -0.004784689f, 0.0012820513f, -0.00034352456f, 0.000092047128f,
            -0.000024663954f, 6.6086865e-6f, -1.7707922e-6f, 4.7448234e-7f, -1.2713716e-7f, 3.4066299e-8f },
    AF{ 100.f, 0.5f, 0.2f, 0.28571429f, 0.26315789f, 0.26923077f, 0.26760563f, 0.26804124f, 0.26792453f, 0.2679558f,
            0.26794742f, 0.26794967f, 0.26794907f, 0.26794923f, 0.26794918f, 0.26794919f, 0.26794919f },
    AF{ 1.5f, 1.6f, 1.6071429f, 1.6076555f, 1.6076923f, 1.607695f, 1.6076951f, 1.6076952f, 1.6076952f, 1.6076952f,
            1.6076952f, 1.6076952f, 1.6076952f },
    AF{ 1.f, 2.f, 1.6666667f, 1.75f, 1.7272727f, 1.7333333f, 1.7317073f, 1.7321429f, 1.7320261f, 1.7320574f,
            1.732049f, 1.7320513f, 1.7320507f, 1.7320508f, 1.7320508f },
    AF{ 0.16666667f, -0.041666667f, 0.011111111f, -0.0029761905f, 0.00079744817f, -0.00021367521f, 0.000057254094f,
            -0.000015341188f, 4.1106589e-6f, -1.1014477e-6f, 2.9513203e-7f, -7.908039e-8f, 2.1189527e-8f },
    AF{ 100.f, 0.33333333f, 0.125f, 0.17777778f, 0.16369048f, 0.16746411f, 0.16645299f, 0.16672392f, 0.16665133f,
            0.16667078f, 0.16666557f, 0.16666696f, 0.16666659f, 0.16666669f, 0.16666666f, 0.16666667f, 0.16666667f },
    0.16666667f
    );

const LUfactorization g_reflected_omoms_lu_factorization = LUfactorization(
    AF{ 100.f, 0.23529412f, 0.33170732f, 0.34266611f, 0.34395774f, 0.34411062f, 0.34412872f, 0.34413087f,
            0.34413112f, 0.34413115f },
    AF{ },
    AF{ },
    AF{ 1.2352941f, 1.7414634f, 1.7989971f, 1.8057781f, 1.8065807f, 1.8066758f, 1.806687f, 1.8066884f, 1.8066885f,
            1.8066886f },
    AF{ 1.f, 1.3076923f, 1.3398693f, 1.3436272f, 1.3440715f, 1.3441241f, 1.3441303f, 1.3441311f, 1.3441311f,
            1.3441312f, 1.3441312f },
    AF{ },
    AF{ },
    0.19047619f
    );

const LUfactorization g_periodic_omoms_lu_factorization = LUfactorization(
    AF{ 100.f, 0.30769231f, 0.33986928f, 0.34362718f, 0.34407148f, 0.34412409f, 0.34413032f, 0.34413106f,
            0.34413114f, 0.34413115f },
    AF{ 0.30769231f, -0.10457516f, 0.035934868f, -0.012364163f, 0.0042548064f, -0.0014642079f, 0.0005038794f,
            -0.00017340059f, 0.000059672546f, -0.000020535182f, 7.066796e-6f, -2.4319047e-6f, 8.3689415e-7f,
            -2.8800135e-7f, 9.9110237e-8f, -3.410692e-8f },
    AF{ 100.f, 0.61538462f, 0.23529412f, 0.37956204f, 0.33170732f, 0.34837889f, 0.34266611f, 0.34463493f, 0.34395774f,
            0.34419083f, 0.34411062f, 0.34413822f, 0.34412872f, 0.34413199f, 0.34413087f, 0.34413125f, 0.34413112f,
            0.34413117f, 0.34413115f, 0.34413116f },
    AF{ 1.6153846f, 1.7843137f, 1.8040427f, 1.8063753f, 1.8066515f, 1.8066842f, 1.806688f, 1.8066885f, 1.8066886f,
            1.8066886f, 1.8066886f, 1.8066886f, 1.8066886f, 1.8066886f, 1.8066886f, 1.8066886f },
    AF{ 1.f, 2.6f, 1.8888889f, 2.1076923f, 2.029703f, 2.0562092f, 2.0470488f, 2.0501965f, 2.0491127f, 2.0494856f,
            2.0493573f, 2.0494015f, 2.0493863f, 2.0493915f, 2.0493897f, 2.0493903f, 2.0493901f, 2.0493902f,
            2.0493901f, 2.0493902f },
    AF{ 0.19047619f, -0.058608059f, 0.019919079f, -0.0068447368f, 0.0023550787f, -0.00081043931f, 0.00027889674f,
            -0.000095977029f, 0.000033028685f, -0.000011366199f, 3.9114633e-6f, -1.3460564e-6f, 4.6321993e-7f,
            -1.5940841e-7f, 5.48574e-8f, -1.887814e-8f },
    AF{ 100.f, 0.38095238f, 0.13186813f, 0.21039527f, 0.18363145f, 0.19283127f, 0.18966575f, 0.19075509f, 0.19038021f,
            0.19050922f, 0.19046482f, 0.1904801f, 0.19047484f, 0.19047665f, 0.19047603f, 0.19047625f, 0.19047617f,
            0.1904762f, 0.19047619f, 0.19047619f },
    0.19047619f
    );


//----------------------------------------------------------------------------

// Filter kernels

inline double sinc_abs(double x) {
    // Sinc[x]  where here I assume that x>=0
    ASSERTX(x>=0.);
    return x<1e-9 ? 1. : sin(x)/x;
}

// normalized sinc function
inline double sinc_norm_abs(double x) {
    // SincNorm[x]  where here I assume that x>=0
    ASSERTX(x>=0.);
    x *= D_TAU/2;
    return sinc_abs(x);
}

inline double lanczos(double x, double R) { // a = W = 2*R
    // dirichlet[x_, a_] := If[x < -a/2, 0, If[x < a/2, Sinc[Pi x], 0]]
    // Sinc[z] = Sin[z]/z for z!=0, but is 1 for z=0.
    // lanczos[x_, a_] := dirichlet[x, a] Sinc[2 Pi x/a]
    // lanczos[x_, r_] := dirichlet[x, 2*r] Sinc[2 Pi x/(2*r)]
    x = abs(x);
    return x<R ? sinc_norm_abs(x) * sinc_norm_abs(x/R) : 0.;
}

inline double hamming(double x, double R) { // a = W = 2*R
    // hamming[x_, a_] := dirichlet[x, a] (0.54 + 0.46 Cos[2 Pi x/a])
    x = abs(x);
    return x<R ? sinc_norm_abs(x) * (0.54 + 0.46*cos(((D_TAU/2)/R)*x)) : 0.;
}


struct Filter_impulse final : Filter {
    using type = Filter_impulse;
    Filter_impulse() : Filter("impulse", nullptr, 0.) {
        _is_discontinuous = true;
        _is_trivial_magnify = true;
        _is_impulse = true;
    }
    static const type& s_f_get() { static type f; return f; } // singleton pattern function
};

struct Filter_box final : Filter {
    using type = Filter_box;
    Filter_box() : Filter("box", sfunc, .5) {
        _is_discontinuous = true;
        _is_trivial_magnify = true;
        _is_trivial_minify = true;
    }
    static double sfunc(double x) {
        // Discontinuous functions must be treated specially to create half-open intervals.
        return x<-.5 ? 0. : x<.5 ? 1. : 0.; // 1 over interval [-.5, .5)
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_triangle final : Filter {
    using type = Filter_triangle;
    Filter_triangle() : Filter("triangle", sfunc, 1.) {
    }
    static double sfunc(double x) {
        x = abs(x);
        return x<1. ? -x+1. : 0.;
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_quadratic final : Filter {
    using type = Filter_quadratic;
    Filter_quadratic() : Filter("quadratic", sfunc, 1.5) {
        _is_discontinuous = true;
    }
    static double sfunc(double x) {
        // (bspline2[x_] := With[{r = Abs[x]}, 1/8 If[r < 1/2, 6 - 8 r r, If[r <= 3/2, 9 + r (-12 + 4 r), 0]]];)
        // Use imoms2 even though it is not C0 continuous!
        // imoms2[x_] := With[{r = Abs[x]}, 1/8 If[r < 1/2, 8 - 8 r r,
        //   If[r == 1/2, 5, If[r <= 3/2, 8 + r (-12 + 4 r), 0]]]]
        // Plot[imoms2[x], {x, -3, 3}]
        // Integrate[imoms2[x], {x, -3, 3}]
        x = abs(x);
        if (x>=1.5) {
            return 0.;
        } else if (x>=.5) {
            return (.5*x-1.5)*x+1.;
        } else {
            return -x*x+1.;
        }
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_mitchell final : Filter {
    using type = Filter_mitchell;
    Filter_mitchell() : Filter("mitchell", sfunc, 2.) {
        _is_interpolating = false;
    }
    static double sfunc(double x) {
        // mitchell[x_] := With[{r = Abs[x]}, 1/18 If[r < 1, 16 + r r (-36 + 21 r),
        //   If[r < 2, 32 + r (-60 + (36 - 7 r) r), 0]]];
        // mitchell[0] = 8/9 : not interpolating!  (it does integrate to 1 though)
        x = abs(x);
        if (x>=2.) {
            return 0.;
        } else if (x>=1.) {
            return (((-7/18.)*x+2.)*x-10/3.)*x+16/9.;
        } else {
            return (((7/6.)*x-2.)*x)*x+8/9.;
        }
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_keys final : Filter { // also known as Catmull-Rom spline
    using type = Filter_keys;
    Filter_keys() : Filter("keys", sfunc, 2.) {
    }
    static double sfunc(double x) {
        // keys[x_] := With[{r = Abs[x]}, 1/2 If[r < 1, 2 + r r (-5 + 3 r),
        //   If[r < 2, 4 + r (-8 + (5 - r) r), 0]]];
        x = abs(x);
        if (x>=2.) {
            return 0.;
        } else if (x>=1.) {
            return ((-0.5*x+2.5)*x-4.)*x+2.;
        } else {
            return ((1.5*x-2.5)*x)*x+1.;
        }
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_spline final : Filter { // cubic B-spline
    using type = Filter_spline;
    Filter_spline() : Filter("spline", sfunc, 2.) {
        _has_inv_convolution = true;
    }
    static double sfunc(double x) {
        // bspline3[x_] := With[{r = Abs[x]}, 1/6 If[r < 1, 4 + r r (-6 + 3 r),
        //   If[r <= 2, 8 + r (-12 + (6 - r) r), 0]]];
        x = abs(x);
        if (x>=2.) {
            return 0.;
        } else if (x>=1.) {
            double t = (x-2.); return (-1./6.)*t*t*t;
        } else {
            return ((0.5*x-1.)*x)*x+2/3.;
        }
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_omoms final : Filter { // cubic OMOMS
    using type = Filter_omoms;
    Filter_omoms() : Filter("omoms", sfunc, 2.) {
        _has_inv_convolution = true;
        _is_omoms = true;
    }
    static double sfunc(double x) {
        // omoms3[x_] := With[{r = Abs[x]}, 1/42 If[r < 1, 26 + r (3 + r (-42 + 21 r)),
        //   If[r < 2, 58 + r (-85 + (42 - 7 r) r), 0]]];
        x = abs(x);
        if (x>=2.) {
            return 0.;
        } else if (x>=1.) {
            return (((-7/42.)*x+1.)*x-85/42.)*x+58/42.;
        } else {
            return ((0.5*x-1.)*x+3/42.)*x+26/42.;
        }
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_preprocess final : Filter {
    using type = Filter_preprocess;
    Filter_preprocess() : Filter("preprocess", nullptr, 0.) {
        _has_inv_convolution = true;
        _is_interpolating = false;
        _is_preprocess = true;
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_justspline final : Filter {
    using type = Filter_justspline;
    Filter_justspline() : Filter("justspline", Filter_spline::sfunc, 2.) {
        _is_interpolating = false;
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_justomoms final : Filter {
    using type = Filter_justomoms;
    Filter_justomoms() : Filter("justomoms", Filter_omoms::sfunc, 2.) {
        _is_interpolating = false;
        _is_omoms = true;
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_gaussian final : Filter {
    using type = Filter_gaussian;
    Filter_gaussian() : Filter("gaussian", sfunc, 4.) { // (radius==4. is sufficiently large for good approximation)
        _is_interpolating = false;
        _is_partition_of_unity = false; // 0.93503:1.06497  av=1  sd=0.0459422
    }
    static double sfunc(double x) {
        const double r = 1.25;      // corresponding to 3 standard deviations
        const double sdv = r/3.;
        // r==1.25 corresponds to kernel that is as-close-as-possible to partition of unity:
        //   0.93503:1.06497     av=1           sd=0.0459424
        return gaussian(x, sdv);
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_lanczos6 final : Filter {
    using type = Filter_lanczos6;
    Filter_lanczos6() : Filter("lanczos6", sfunc, 3.) {
        _is_partition_of_unity = false; // 0.994299:1  av=0.997055  sd=0.00200379
        _is_unit_integral = false;      // 0.997055
    }
    static double sfunc(double x) {
        // lanczos[x, 6]
        return lanczos(x, 3.);
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_lanczos10 final : Filter {
    using type = Filter_lanczos10;
    Filter_lanczos10() : Filter("lanczos10", sfunc, 5.) {
        _is_partition_of_unity = false; // 0.998746:1  av=0.999353  sd=0.000439056
        _is_unit_integral = false;      // 0.999353
    }
    static double sfunc(double x) {
        // lanczos[x, 10]
        return lanczos(x, 5.);
    }
    static const type& s_f_get() { static type f; return f; }
};

struct Filter_hamming6 final : Filter {
    using type = Filter_hamming6;
    Filter_hamming6() : Filter("hamming6", sfunc, 3.) {
        _is_partition_of_unity = false; // 1:1.00242  av=1.00188  sd=0.00052909
        _is_unit_integral = false;      // 1.00188
    }
    static double sfunc(double x) {
        // hamming[x, 6]
        return hamming(x, 3.);
    }
    static const type& s_f_get() { static type f; return f; }
};

} // namespace

const Filter& Filter::get(const string& name) {
    assertx(name!="");
    // Careful: Filter::get() may be called by some static constructor.
    static Array<const Filter*> filters;
    static std::once_flag flag;
    std::call_once(flag, [] {
        filters.push(&Filter_impulse::s_f_get());
        filters.push(&Filter_box::s_f_get());
        filters.push(&Filter_triangle::s_f_get());
        filters.push(&Filter_quadratic::s_f_get());
        filters.push(&Filter_mitchell::s_f_get());
        filters.push(&Filter_keys::s_f_get());
        filters.push(&Filter_spline::s_f_get());
        filters.push(&Filter_omoms::s_f_get());
        filters.push(&Filter_preprocess::s_f_get());
        filters.push(&Filter_justspline::s_f_get());
        filters.push(&Filter_justomoms::s_f_get());
        filters.push(&Filter_gaussian::s_f_get());
        filters.push(&Filter_lanczos6::s_f_get());
        filters.push(&Filter_lanczos10::s_f_get());
        filters.push(&Filter_hamming6::s_f_get());
    });
    assertx(filters.num());
    for (const Filter* filter : filters) {
        string fname = filter->name();
        if (fname==name ||
            (name.size()==1 && name[0]!='j' && fname[0]==name[0])) return *filter;
    }
    assertnever("Filter '" + name + "' not recognized");
}

const LUfactorization& FilterBnd::lu_factorization() const {
    // There is an inherent problem with using "generalized filtering" with clamped boundary rule:
    //  there is no solution to the discrete inverse convolution which can both (1) satisfy the clamped
    //  boundary condition and (2) interpolate the original data.
    // These constraints can only be approximated in a least-squares sense, so it is hopeless.
    assertx(filter().has_inv_convolution());
    bool is_bspline = !filter().is_omoms() || filter().is_preprocess();
    switch (bndrule()) {
     bcase Bndrule::reflected:
        return is_bspline ? g_reflected_spline_lu_factorization : g_reflected_omoms_lu_factorization;
     bcase Bndrule::periodic:
        return is_bspline ? g_periodic_spline_lu_factorization  : g_periodic_omoms_lu_factorization;
     bcase Bndrule::clamped:
        Warning("Clamped boundary only approximated by Reflected boundary; could pad data first");
        return is_bspline ? g_reflected_spline_lu_factorization : g_reflected_omoms_lu_factorization;
     bcase Bndrule::border:
        assertnever("Bndrule::border not supported in generalized-filter scale operation; could pad data first");
     bdefault: assertnever("");
    }
}

void FilterBnd::setup_kernel_weights(int cx, int nx, bool primal,
                                     Array<int>& ar_pixelindex0, Matrix<float>& mat_weights) const {
    assertx(cx >= (!primal ? 1 : 2) && nx >= (!primal ? 1 : 2));
    double scaling = !primal ? double(nx)/cx : double(nx-1)/(cx-1);
    const bool is_magnify = nx>cx;
    bool is_impulse = filter().is_impulse();
    KernelFunc kernel_func = is_impulse ? nullptr : filter().func();
    double kernel_radius = filter().radius();
    int nk = is_impulse ? 1 : is_magnify ? int(kernel_radius*2.) : int(kernel_radius*2./scaling+.999999);
    bool nkodd = (nk%2)>0;
    ar_pixelindex0.init(nx);
    mat_weights.init(nx, nk);
    for_int(x, nx) {
        double dx = (!primal ? (x+0.5)/nx : // range [0, 1]: ngrid[0] at (0.5)/nx, ngrid[nx-1] at (nx-0.5)/nx
                     x/(nx-1.));            // range [0, 1]: ngrid[0] at 0.,       ngrid[nx-1] at 1.
        double wcx = (!primal ? dx*cx-0.5 : // back to pixel coordinates of current grid
                      dx*(cx-1));
        double wf = floor(wcx+(nkodd ? 0.5 : 0.));
        int pixelindex0 = int(wf)-((nk-1)/2);
        ar_pixelindex0[x] = pixelindex0;
        double sum = 0.;
        // Here we cannot fold the boundary rule because Bndrule::periodic requires non-contiguous access.
        // Instead we use map_boundaryrule_1D() within evaluate_kernel_d().
        for_int(k, nk) {
            int i = pixelindex0+k;
            double w0 = double(i);
            double v = is_impulse ? 1. : is_magnify ? kernel_func(wcx-w0) : kernel_func((wcx-w0)*scaling)*scaling;
            if (0) SHOW(x, k, wcx, wf, pixelindex0, i, v);
            mat_weights[x][k] = float(v);
            sum += v;
        }
        if (0) { HH_SSTAT(Ssum, sum); }
        if (0) SHOW(ar_pixelindex0[x], mat_weights[x]);
        if (0 && k_debug) {
            bool discontinuous = filter().is_discontinuous();
            double thresh = is_magnify ? (filter().name()=="gaussian" ? .1 : 1e-5) : (discontinuous ? 1. : .2);
            if (abs(sum-1.)>thresh) { // e.g. fails for "quadratic" cx=2 nx=3 x=1
                SHOW(cx, nx, filter().name(), nk, x, sum);
                SHOW(mat_weights);
                assertnever("");
            }
        }
        mat_weights[x] /= float(sum);
    }
}

} // namespace hh
