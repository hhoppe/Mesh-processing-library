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
    AF{ 100., 0.2, 0.26315789, 0.26760563, 0.26792453, 0.26794742, 0.26794907, 0.26794918, 0.26794919, 0.26794919 },
    AF{ },
    AF{ },
    AF{ 1.2, 1.5789474, 1.6056338, 1.6075472, 1.6076845, 1.6076944, 1.6076951, 1.6076952, 1.6076952, 1.6076952 },
    AF{ 1., 1.25, 1.2666667, 1.2678571, 1.2679426, 1.2679487, 1.2679492, 1.2679492, 1.2679492, 1.2679492, 1.2679492 },
    AF{ },
    AF{ },
    0.16666667f
    );

const LUfactorization g_periodic_spline_lu_factorization = LUfactorization(
    AF{ 100., 0.25, 0.26666667, 0.26785714, 0.26794258, 0.26794872, 0.26794916, 0.26794919,
            0.26794919, 0.26794919 },
    AF{ 0.25, -0.066666667, 0.017857143, -0.004784689, 0.0012820513, -0.00034352456, 0.000092047128, -0.000024663954,
            6.6086865e-6, -1.7707922e-6, 4.7448234e-7, -1.2713716e-7, 3.4066299e-8 },
    AF{ 100., 0.5, 0.2, 0.28571429, 0.26315789, 0.26923077, 0.26760563, 0.26804124, 0.26792453, 0.2679558, 0.26794742,
            0.26794967, 0.26794907, 0.26794923, 0.26794918, 0.26794919, 0.26794919 },
    AF{ 1.5, 1.6, 1.6071429, 1.6076555, 1.6076923, 1.607695, 1.6076951, 1.6076952, 1.6076952, 1.6076952,
            1.6076952, 1.6076952, 1.6076952 },
    AF{ 1., 2., 1.6666667, 1.75, 1.7272727, 1.7333333, 1.7317073, 1.7321429, 1.7320261, 1.7320574, 1.732049,
            1.7320513, 1.7320507, 1.7320508, 1.7320508 },
    AF{ 0.16666667, -0.041666667, 0.011111111, -0.0029761905, 0.00079744817, -0.00021367521, 0.000057254094,
            -0.000015341188, 4.1106589e-6, -1.1014477e-6, 2.9513203e-7, -7.908039e-8, 2.1189527e-8 },
    AF{ 100., 0.33333333, 0.125, 0.17777778, 0.16369048, 0.16746411, 0.16645299, 0.16672392, 0.16665133, 0.16667078,
            0.16666557, 0.16666696, 0.16666659, 0.16666669, 0.16666666, 0.16666667, 0.16666667 },
    0.16666667f
    );

const LUfactorization g_reflected_omoms_lu_factorization = LUfactorization(
    AF{ 100., 0.23529412, 0.33170732, 0.34266611, 0.34395774, 0.34411062, 0.34412872, 0.34413087,
            0.34413112, 0.34413115 },
    AF{ },
    AF{ },
    AF{ 1.2352941, 1.7414634, 1.7989971, 1.8057781, 1.8065807, 1.8066758, 1.806687, 1.8066884, 1.8066885, 1.8066886 },
    AF{ 1., 1.3076923, 1.3398693, 1.3436272, 1.3440715, 1.3441241, 1.3441303, 1.3441311, 1.3441311, 1.3441312,
            1.3441312 },
    AF{ },
    AF{ },
    0.19047619f
    );

const LUfactorization g_periodic_omoms_lu_factorization = LUfactorization(
    AF{ 100., 0.30769231, 0.33986928, 0.34362718, 0.34407148, 0.34412409, 0.34413032, 0.34413106,
            0.34413114, 0.34413115 },
    AF{ 0.30769231, -0.10457516, 0.035934868, -0.012364163, 0.0042548064, -0.0014642079, 0.0005038794, -0.00017340059,
            0.000059672546, -0.000020535182, 7.066796e-6, -2.4319047e-6, 8.3689415e-7, -2.8800135e-7,
            9.9110237e-8, -3.410692e-8 },
    AF{ 100., 0.61538462, 0.23529412, 0.37956204, 0.33170732, 0.34837889, 0.34266611, 0.34463493, 0.34395774,
            0.34419083, 0.34411062, 0.34413822, 0.34412872, 0.34413199, 0.34413087, 0.34413125, 0.34413112,
            0.34413117, 0.34413115, 0.34413116 },
    AF{ 1.6153846, 1.7843137, 1.8040427, 1.8063753, 1.8066515, 1.8066842, 1.806688, 1.8066885, 1.8066886, 1.8066886,
            1.8066886, 1.8066886, 1.8066886, 1.8066886, 1.8066886, 1.8066886 },
    AF{ 1., 2.6, 1.8888889, 2.1076923, 2.029703, 2.0562092, 2.0470488, 2.0501965, 2.0491127, 2.0494856, 2.0493573,
            2.0494015, 2.0493863, 2.0493915, 2.0493897, 2.0493903, 2.0493901, 2.0493902, 2.0493901, 2.0493902 },
    AF{ 0.19047619, -0.058608059, 0.019919079, -0.0068447368, 0.0023550787, -0.00081043931, 0.00027889674,
            -0.000095977029, 0.000033028685, -0.000011366199, 3.9114633e-6, -1.3460564e-6, 4.6321993e-7,
            -1.5940841e-7, 5.48574e-8, -1.887814e-8 },
    AF{ 100., 0.38095238, 0.13186813, 0.21039527, 0.18363145, 0.19283127, 0.18966575, 0.19075509, 0.19038021,
            0.19050922, 0.19046482, 0.1904801, 0.19047484, 0.19047665, 0.19047603, 0.19047625, 0.19047617,
            0.1904762, 0.19047619, 0.19047619 },
    0.19047619f
    );


//----------------------------------------------------------------------------

// Filter kernels

inline double sinc_abs(double x) {
    // Sinc[x]  where here I assume that x>=0
    ASSERTX(x>=0.);
    return x<1e-9 ? 1. : sin(x)/x;
}

inline double lanczos(double x, double R) { // a = W = 2*R
    // dirichlet[x_, a_] := If[x < -a/2, 0, If[x < a/2, Sinc[Pi x], 0]]
    // Sinc[z] = Sin[z]/z for z!=0, but is 1 for z=0.
    // lanczos[x_, a_] := dirichlet[x, a] Sinc[2 Pi x/a]
    // lanczos[x_, r_] := dirichlet[x, 2*r] Sinc[2 Pi x/(2*r)]
    x = abs(x);
    return x<R ? sinc_abs((D_TAU/2)*x) * sinc_abs(((D_TAU/2)/R)*x) : 0.;
}

inline double hamming(double x, double R) { // a = W = 2*R
    // hamming[x_, a_] := dirichlet[x, a] (0.54 + 0.46 Cos[2 Pi x/a])
    x = abs(x);
    return x<R ? sinc_abs((D_TAU/2)*x) * (0.54 + 0.46*cos(((D_TAU/2)/R)*x)) : 0.;
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
    Filter_gaussian() : Filter("gaussian", sfunc, 4.) { // (radius==4. is sufficiently large approximation)
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
    std::call_once(flag, []() {
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
        assertnever("Bndrule::border not supported in scale operation; could pad data first");
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
