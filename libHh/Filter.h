// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_FILTER_H_
#define MESH_PROCESSING_LIBHH_FILTER_H_

#include "libHh/Array.h"
#include "libHh/Matrix.h"

namespace hh {

using KernelFunc = double (*)(double);

class Filter : noncopyable {
 public:
  explicit Filter(string name_, KernelFunc func_, double radius_)
      : _name(std::move(name_)), _func(func_), _radius(radius_) {}
  [[nodiscard]] string name() const { return _name; }
  [[nodiscard]] KernelFunc func() const { return assertx(_func); }
  [[nodiscard]] double radius() const { return _radius; }
  [[nodiscard]] bool is_interpolating() const { return _is_interpolating; }
  [[nodiscard]] bool is_discontinuous() const { return _is_discontinuous; }
  [[nodiscard]] bool has_inv_convolution() const { return _has_inv_convolution; }
  // True if the kernel interpolates only after a digital filter (inverse convolution) of the samples.
  [[nodiscard]] bool requires_digital_filter() const { return _has_inv_convolution; }
  // The kernel to evaluate after the digital filter (e.g., bspline3 for cardinal3); the filter itself otherwise.
  [[nodiscard]] const Filter& digital_filter_kernel() const;
  [[nodiscard]] bool is_trivial_magnify() const { return _is_trivial_magnify; }  // For: impulse or box.
  [[nodiscard]] bool is_trivial_minify() const { return _is_trivial_minify; }    // For: box.
  [[nodiscard]] bool is_impulse() const { return _is_impulse; }
  [[nodiscard]] bool is_omoms() const { return _is_omoms; }            // For: omoms or justomoms.
  [[nodiscard]] bool is_preprocess() const { return _is_preprocess; }  // For: preprocess.
  [[nodiscard]] bool is_partition_of_unity() const { return _is_partition_of_unity; }
  [[nodiscard]] bool is_unit_integral() const { return _is_unit_integral; }
  [[nodiscard]] bool is_auto() const {
    return _is_auto;
  }  // The placeholder "auto", resolved by the resampling options.

  // The names follow the Python package `resampler`; the former names and single letters remain aliases.
  [[HH_NO_DANGLING]] [[nodiscard]] static const Filter& get(std::string_view name);
  // impulse:     (i) nearest sample (does not support func()).
  // box:         (b) nearest magnification; a.k.a. rectangle/rect func, top-hat, pi func, unit pulse, boxcar.
  // triangle:    (t) a.k.a. linear, tent, hat, triangular func.
  // quadratic:   (q) discontinuous like "box".
  // mitchell:    (m) somewhat blurry bicubic; not interpolating.
  // cubic:       (k, keys) sharper bicubic; Catmull-Rom spline; the default.
  // sharpcubic:  even sharper cubic (b = 0, c = 3 / 4), as in OpenCV and Photoshop.
  // cardinal3:   (s, spline) cardinal cubic B-spline; digital filter + bspline3.
  // cardinal5:   cardinal quintic B-spline; digital filter + bspline5.
  // omoms3:      (o, omoms) cubic maximal-order minimal-support; digital filter + justomoms.
  // omoms5:      quintic OMOMS; digital filter + justomoms5.
  // gaussian:    (g) not interpolating.
  // preprocess:  (p) cubic B-spline digital filter alone.
  // bspline3:    (justspline) the approximating cubic B-spline (evaluated after the digital filter).
  // bspline5:    the approximating quintic B-spline.
  // justomoms:   the approximating cubic OMOMS kernel.
  // justomoms5:  the approximating quintic OMOMS kernel.
  // lanczos3:    (l, lanczos6) Lanczos filter with support [-3, 3].
  // lanczos5:    (formerly lanczos10) Lanczos filter with support [-5, 5].
  // lanczos10:   Lanczos filter with support [-10, 10]; best but slowest.
  // hamming3:    (h, hamming6) Hamming-windowed sinc with support [-3, 3] (a0 = 25 / 46).
  // kaiser3:     Kaiser-windowed sinc with support [-3, 3] (beta = 7.12).
  // auto:        a placeholder resolved by the resampling options (e.g., prefilter "auto" means the filter).
  friend std::ostream& operator<<(std::ostream& os, const Filter& filter) {
    return os << "Filter{" << filter.name() << "}";
  }

 protected:
  ~Filter() = default;
  bool _is_interpolating{true};
  bool _is_discontinuous{false};
  bool _has_inv_convolution{false};
  bool _is_trivial_magnify{false};
  bool _is_trivial_minify{false};
  bool _is_impulse{false};
  bool _is_omoms{false};
  bool _is_preprocess{false};
  bool _is_partition_of_unity{true};
  bool _is_unit_integral{true};
  bool _is_auto{false};

 private:
  string _name;
  KernelFunc _func;
  double _radius;
};

struct LUfactorization;

class FilterBnd {
 public:
  explicit FilterBnd(const Filter& filter_, Bndrule bndrule_) : _filter(&filter_) { set_bndrule(bndrule_); }
  [[nodiscard]] const Filter& filter() const { return *_filter; }
  [[nodiscard]] Bndrule bndrule() const { return _bndrule; }
  void set_filter(const Filter& filter_) { _filter = &filter_; }
  void set_bndrule(Bndrule bndrule_) { _bndrule = bndrule_, assertx(_bndrule != Bndrule::undefined); }
  // Returns the LU factorization of the inverse convolution matrix for the filter and boundary rule.
  [[nodiscard]] const LUfactorization& lu_factorization() const;
  // Given old size cx and new size nx, for each new index i compute the interval of weights on old values
  //  starting at index ar_pixelindex0[i].  mat_weights is allocated of size [nx, nk] where nk is interval size.
  // Samples are assumed at half-integer locations (!primal) or integer locations (primal).
  void setup_kernel_weights(int cx, int nx, bool primal, Array<int>& ar_pixelindex0, Matrix<float>& mat_weights) const;
  friend std::ostream& operator<<(std::ostream& os, const FilterBnd& filterbnd) {
    return os << "FilterBnd{" << filterbnd._filter->name() << ", " << boundaryrule_name(filterbnd._bndrule) << "}";
  }

 private:
  const Filter* _filter;
  Bndrule _bndrule;
};

// LU factorization of inverse convolution matrix for a generalized sampling filter.
struct LUfactorization {
  using AF = Array<float>;
  explicit LUfactorization(AF a1, AF a2, AF a3, AF a4, AF a5, AF a6, AF a7, float f)
      : Llower(std::move(a1)),
        Llastrow(std::move(a2)),
        LlastrowPen(std::move(a3)),
        Uidiag(std::move(a4)),
        UidiagLast(std::move(a5)),
        Ulastcol(std::move(a6)),
        UlastcolPen(std::move(a7)),
        Uupper(f) {
    validate();
  }
  Array<float> Llower;       // Lower off-diagonal of L (except the last row).
  Array<float> Llastrow;     // Last row of L (except the last two elements).
  Array<float> LlastrowPen;  // Penultimate element of last row of L, as a function of matrix size ([0]=size1).
  Array<float> Uidiag;       // Inverse diagonal of U (except the last row).
  Array<float> UidiagLast;   // Last diagonal element of U, as a function of matrix size.
  Array<float> Ulastcol;     // Last column of U (except the last two elements).
  Array<float> UlastcolPen;  // Penultimate element of last column of U, as a function of matrix size ([0]=size1).
  float Uupper;              // Upper off-diagonal of U, constant along the diagonal.
  void validate() const {
    const bool last_special = Llastrow.num() > 0;
    assertx(last_special == (LlastrowPen.num() > 0));
    assertx(last_special == (Ulastcol.num() > 0));
    assertx(last_special == (UlastcolPen.num() > 0));
    if (last_special) assertx(Ulastcol.num() == Uidiag.num());
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_FILTER_H_
