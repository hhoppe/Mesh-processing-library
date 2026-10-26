// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Filter.h"

#include "libHh/MathOp.h"  // gaussian()

namespace hh {

namespace {

//----------------------------------------------------------------------------

// Filter kernels.

inline double sinc_abs(double x) {
  // Sinc[x], but assuming here that x >= 0.
  ASSERTX(x >= 0.);
  return x < 1e-9 ? 1. : std::sin(x) / x;
}

// The normalized sinc function.
inline double sinc_norm_abs(double x) {
  // SincNorm[x], but assuming here that x >= 0.
  ASSERTX(x >= 0.);
  x *= D_TAU / 2;
  return sinc_abs(x);
}

inline double lanczos(double x, double R) {  // a = W = 2 * R
  // dirichlet[x_, a_] := If[x < -a / 2, 0, If[x < a / 2, Sinc[Pi x], 0]]
  // Sinc[z] = Sin[z] / z for z != 0, but is 1 for z == 0.
  // lanczos[x_, a_] := dirichlet[x, a] Sinc[2 Pi x / a]
  // lanczos[x_, r_] := dirichlet[x, 2 * r] Sinc[2 Pi x / (2 * r)]
  x = abs(x);
  return x < R ? sinc_norm_abs(x) * sinc_norm_abs(x / R) : 0.;
}

inline double hamming(double x, double R, double a0) {  // a = W = 2 * R
  // hamming[x_, a_] := dirichlet[x, a] (a0 + (1 - a0) Cos[2 Pi x / a])
  x = abs(x);
  return x < R ? sinc_norm_abs(x) * (a0 + (1. - a0) * std::cos(((D_TAU / 2) / R) * x)) : 0.;
}

// The modified Bessel function of the first kind of order 0, by its power series (converges for all z).
inline double bessel_i0(double z) {
  double sum = 1., term = 1.;
  const double q = square(z / 2.);
  for (int k = 1; k < 100; k++) {
    term *= q / square(double(k));
    sum += term;
    if (term < 1e-17 * sum) break;
  }
  return sum;
}

inline double kaiser(double x, double R, double beta) {
  x = abs(x);
  if (x >= R) return 0.;
  const double window = bessel_i0(beta * std::sqrt(1. - square(x / R))) / bessel_i0(beta);
  return sinc_norm_abs(x) * window;
}

struct FilterImpulse final : Filter {
  using type = FilterImpulse;
  FilterImpulse() : Filter("impulse", nullptr, 0.) {
    _is_discontinuous = true;
    _is_trivial_magnify = true;
    _is_impulse = true;
  }
};

struct FilterBox final : Filter {
  using type = FilterBox;
  FilterBox() : Filter("box", sfunc, .5) {
    _is_discontinuous = true;
    _is_trivial_magnify = true;
    _is_trivial_minify = true;
  }
  static double sfunc(double x) {
    // Discontinuous functions must be treated specially to create half-open intervals.
    return x < -.5 ? 0. : x < .5 ? 1. : 0.;  // Value 1 over the interval [-.5, .5).
  }
};

struct FilterTriangle final : Filter {
  using type = FilterTriangle;
  FilterTriangle() : Filter("triangle", sfunc, 1.) {}
  static double sfunc(double x) {
    x = abs(x);
    return x < 1. ? -x + 1. : 0.;
  }
};

struct FilterQuadratic final : Filter {
  using type = FilterQuadratic;
  FilterQuadratic() : Filter("quadratic", sfunc, 1.5) { _is_discontinuous = true; }
  static double sfunc(double x) {
    // (bspline2[x_] := With[{r = Abs[x]}, 1/8 If[r < 1 / 2, 6 - 8 r r, If[r <= 3 / 2, 9 + r (-12 + 4 r), 0]]];)
    // Use imoms2 even though it is not C0 continuous!
    // imoms2[x_] := With[{r = Abs[x]}, 1/8 If[r < 1 / 2, 8 - 8 r r,
    //   If[r == 1 / 2, 5, If[r <= 3 / 2, 8 + r (-12 + 4 r), 0]]]]
    // Plot[imoms2[x], {x, -3, 3}]
    // Integrate[imoms2[x], {x, -3, 3}]
    x = abs(x);
    if (x >= 1.5) {
      return 0.;
    } else if (x >= .5) {
      return (.5 * x - 1.5) * x + 1.;
    } else {
      return -x * x + 1.;
    }
  }
};

struct FilterMitchell final : Filter {
  using type = FilterMitchell;
  FilterMitchell() : Filter("mitchell", sfunc, 2.) { _is_interpolating = false; }
  static double sfunc(double x) {
    // mitchell[x_] := With[{r = Abs[x]}, 1 / 18 If[r < 1, 16 + r r (-36 + 21 r),
    //   If[r < 2, 32 + r (-60 + (36 - 7 r) r), 0]]];
    // mitchell[0] = 8 / 9 : not interpolating!  (it does integrate to 1 though)
    x = abs(x);
    if (x >= 2.) {
      return 0.;
    } else if (x >= 1.) {
      return (((-7. / 18.) * x + 2.) * x - 10. / 3.) * x + 16. / 9.;
    } else {
      return (((7. / 6.) * x - 2.) * x) * x + 8. / 9.;
    }
  }
};

struct FilterCubic final : Filter {  // The Keys filter, also known as the Catmull-Rom spline.
  using type = FilterCubic;
  FilterCubic() : Filter("cubic", sfunc, 2.) {}
  static double sfunc(double x) {
    // keys[x_] := With[{r = Abs[x]}, 1 / 2 If[r < 1, 2 + r r (-5 + 3 r),
    //   If[r < 2, 4 + r (-8 + (5 - r) r), 0]]];
    x = abs(x);
    if (x >= 2.) {
      return 0.;
    } else if (x >= 1.) {
      return ((-0.5 * x + 2.5) * x - 4.) * x + 2.;
    } else {
      return ((1.5 * x - 2.5) * x) * x + 1.;
    }
  }
};

struct FilterSharpCubic final : Filter {  // The Mitchell-Netravali cubic with b = 0, c = 3 / 4.
  using type = FilterSharpCubic;
  FilterSharpCubic() : Filter("sharpcubic", sfunc, 2.) {}
  static double sfunc(double x) {
    // With b = 0 and c = 3 / 4: f3 = 2 - c, f2 = -3 + c, f0 = 1; g3 = -c, g2 = 5 c, g1 = -8 c, g0 = 4 c.
    x = abs(x);
    if (x >= 2.) {
      return 0.;
    } else if (x >= 1.) {
      return ((-0.75 * x + 3.75) * x - 6.) * x + 3.;
    } else {
      return ((1.25 * x - 2.25) * x) * x + 1.;
    }
  }
};

struct FilterCardinal3 final : Filter {  // A cardinal cubic B-spline.
  using type = FilterCardinal3;
  FilterCardinal3() : Filter("cardinal3", sfunc, 2.) { _has_inv_convolution = true; }
  static double sfunc(double x) {
    // bspline3[x_] := With[{r = Abs[x]}, 1 / 6 If[r < 1, 4 + r r (-6 + 3 r),
    //   If[r <= 2, 8 + r (-12 + (6 - r) r), 0]]];
    x = abs(x);
    if (x >= 2.) {
      return 0.;
    } else if (x >= 1.) {
      const double t = x - 2.;
      return (-1. / 6.) * t * t * t;
    } else {
      return ((0.5 * x - 1.) * x) * x + 2. / 3.;
    }
  }
};

struct FilterCardinal5 final : Filter {  // A cardinal quintic B-spline.
  using type = FilterCardinal5;
  FilterCardinal5() : Filter("cardinal5", sfunc, 3.) { _has_inv_convolution = true; }
  static double sfunc(double x) {
    // The uniform quintic B-spline, with support [-3, 3].
    x = abs(x);
    const double x2 = x * x;
    if (x >= 3.) {
      return 0.;
    } else if (x >= 2.) {
      const double t = 3. - x;
      return t * t * t * t * t / 120.;
    } else if (x >= 1.) {
      return 17. / 40. + 5. / 8. * x - 7. / 4. * x2 + 5. / 4. * x2 * x - 3. / 8. * x2 * x2 + x2 * x2 * x / 24.;
    } else {
      return 11. / 20. - x2 / 2. + x2 * x2 / 4. - x2 * x2 * x / 12.;
    }
  }
};

struct FilterOmoms3 final : Filter {  // A cubic OMOMS.
  using type = FilterOmoms3;
  FilterOmoms3() : Filter("omoms3", sfunc, 2.) {
    _has_inv_convolution = true;
    _is_omoms = true;
  }
  static double sfunc(double x) {
    // omoms3[x_] := With[{r = Abs[x]}, 1 / 42 If[r < 1, 26 + r (3 + r (-42 + 21 r)),
    //   If[r < 2, 58 + r (-85 + (42 - 7 r) r), 0]]];
    x = abs(x);
    if (x >= 2.) {
      return 0.;
    } else if (x >= 1.) {
      return (((-7. / 42.) * x + 1.) * x - 85. / 42.) * x + 58. / 42.;
    } else {
      return ((0.5 * x - 1.) * x + 3. / 42.) * x + 26. / 42.;
    }
  }
};

struct FilterPreprocess final : Filter {
  using type = FilterPreprocess;
  FilterPreprocess() : Filter("preprocess", nullptr, 0.) {
    _has_inv_convolution = true;
    _is_interpolating = false;
    _is_preprocess = true;
  }
};

struct FilterOmoms5 final : Filter {  // A quintic OMOMS.
  using type = FilterOmoms5;
  FilterOmoms5() : Filter("omoms5", sfunc, 3.) {
    _has_inv_convolution = true;
    _is_omoms = true;
  }
  static double sfunc(double x) {
    // The polynomial pieces are those of OmomsFilter(degree=5) in the Python package resampler.
    x = abs(x);
    if (x >= 3.) {
      return 0.;
    } else if (x >= 2.) {
      return ((((-1. / 120. * x + 1. / 8.) * x - 299. / 396.) * x + 101. / 44.) * x - 27811. / 7920.) * x +
             5707. / 2640.;
    } else if (x >= 1.) {
      return ((((1. / 24. * x - 3. / 8.) * x + 505. / 396.) * x - 83. / 44.) * x + 1351. / 1584.) * x + 839. / 2640.;
    } else {
      return ((((-1. / 12. * x + 1. / 4.) * x - 5. / 99.) * x - 9. / 22.) * x - 1. / 792.) * x + 229. / 440.;
    }
  }
};

struct FilterBspline3 final : Filter {
  using type = FilterBspline3;
  FilterBspline3() : Filter("bspline3", FilterCardinal3::sfunc, 2.) { _is_interpolating = false; }
};

struct FilterBspline5 final : Filter {
  using type = FilterBspline5;
  FilterBspline5() : Filter("bspline5", FilterCardinal5::sfunc, 3.) { _is_interpolating = false; }
};

struct FilterJustOmoms final : Filter {
  using type = FilterJustOmoms;
  FilterJustOmoms() : Filter("justomoms", FilterOmoms3::sfunc, 2.) {
    _is_interpolating = false;
    _is_omoms = true;
  }
};

struct FilterJustOmoms5 final : Filter {
  using type = FilterJustOmoms5;
  FilterJustOmoms5() : Filter("justomoms5", FilterOmoms5::sfunc, 3.) {
    _is_interpolating = false;
    _is_omoms = true;
  }
};

struct FilterGaussian final : Filter {
  using type = FilterGaussian;
  FilterGaussian() : Filter("gaussian", sfunc, 4.) {  // (radius == 4. is sufficiently large for good approximation)
    _is_interpolating = false;
    _is_partition_of_unity = false;  // 0.93503:1.06497  av=1  sd=0.0459422
  }
  static double sfunc(double x) {
    const double r = 1.25;  // Corresponding to 3 standard deviations.
    const double sdv = r / 3.;
    // r == 1.25 corresponds to kernel that is as-close-as-possible to partition of unity:
    //   0.93503:1.06497     av=1           sd=0.0459424
    return gaussian(x, sdv);
  }
};

struct FilterLanczos3 final : Filter {
  using type = FilterLanczos3;
  FilterLanczos3() : Filter("lanczos3", sfunc, 3.) {
    _is_partition_of_unity = false;  // 0.994299:1  av=0.997055  sd=0.00200379
    _is_unit_integral = false;       // 0.997055
  }
  static double sfunc(double x) { return lanczos(x, 3.); }
};

struct FilterLanczos5 final : Filter {
  using type = FilterLanczos5;
  FilterLanczos5() : Filter("lanczos5", sfunc, 5.) {
    _is_partition_of_unity = false;  // 0.998746:1  av=0.999353  sd=0.000439056
    _is_unit_integral = false;       // 0.999353
  }
  static double sfunc(double x) { return lanczos(x, 5.); }
};

struct FilterLanczos10 final : Filter {
  using type = FilterLanczos10;
  FilterLanczos10() : Filter("lanczos10", sfunc, 10.) {
    _is_partition_of_unity = false;
    _is_unit_integral = false;
  }
  static double sfunc(double x) { return lanczos(x, 10.); }
};

struct FilterHamming3 final : Filter {
  using type = FilterHamming3;
  FilterHamming3() : Filter("hamming3", sfunc, 3.) {
    _is_partition_of_unity = false;  // 1:1.00292  av=1.00236  sd=0.00067597
    _is_unit_integral = false;       // 1.00236
  }
  static double sfunc(double x) { return hamming(x, 3., 25. / 46.); }
};

struct FilterKaiser3 final : Filter {
  using type = FilterKaiser3;
  FilterKaiser3() : Filter("kaiser3", sfunc, 3.) {
    _is_partition_of_unity = false;
    _is_unit_integral = false;
  }
  static double sfunc(double x) { return kaiser(x, 3., 7.12); }
};

struct FilterTrapezoid final : Filter {  // Its kernel depends on the scaling; see Resizer::compute_dim().
  using type = FilterTrapezoid;
  FilterTrapezoid() : Filter("trapezoid", nullptr, 0.) { _is_trapezoid = true; }
};

struct FilterAuto final : Filter {
  using type = FilterAuto;
  FilterAuto() : Filter("auto", nullptr, 0.) { _is_auto = true; }
};

// Former names (and single letters) that remain accepted by Filter::get().
constexpr std::pair<std::string_view, std::string_view> k_filter_aliases[] = {
    {"keys", "cubic"},        {"spline", "cardinal3"},  {"omoms", "omoms3"}, {"justspline", "bspline3"},
    {"lanczos6", "lanczos3"}, {"hamming6", "hamming3"}, {"i", "impulse"},    {"b", "box"},
    {"t", "triangle"},        {"q", "quadratic"},       {"m", "mitchell"},   {"k", "cubic"},
    {"s", "cardinal3"},       {"o", "omoms3"},          {"g", "gaussian"},   {"p", "preprocess"},
    {"l", "lanczos3"},        {"h", "hamming3"},
};

}  // namespace

const Filter& Filter::get(std::string_view name) {
  assertx(name != "");
  // Careful: Filter::get() may be called by some static constructor, so we use a function-local
  // static (thread-safe and lazily initialized) and intentionally never destroy the filters.
  static const auto& filters = *new Array<const Filter*>{
      new FilterImpulse,    new FilterBox,        new FilterTriangle,  new FilterQuadratic, new FilterMitchell,
      new FilterCubic,      new FilterSharpCubic, new FilterCardinal3, new FilterCardinal5, new FilterOmoms3,
      new FilterOmoms5,     new FilterPreprocess, new FilterBspline3,  new FilterBspline5,  new FilterJustOmoms,
      new FilterJustOmoms5, new FilterGaussian,   new FilterLanczos3,  new FilterLanczos5,  new FilterLanczos10,
      new FilterHamming3,   new FilterKaiser3,    new FilterTrapezoid, new FilterAuto,
  };
  assertx(filters.num());
  for (const auto& [alias, canonical] : k_filter_aliases)
    if (name == alias) name = canonical;
  for (const Filter* filter : filters)
    if (filter->name() == name) return *filter;
  assertnever("Filter '" + string(name) + "' not recognized");
}

const Filter& Filter::digital_filter_kernel() const {
  if (!_has_inv_convolution) return *this;
  if (_is_preprocess) return get("bspline3");  // The digital filter of the cubic B-spline.
  if (_name == "cardinal3") return get("bspline3");
  if (_name == "cardinal5") return get("bspline5");
  if (_name == "omoms3") return get("justomoms");
  if (_name == "omoms5") return get("justomoms5");
  assertnever("Filter '" + _name + "' has no digital-filter kernel");
}

}  // namespace hh
