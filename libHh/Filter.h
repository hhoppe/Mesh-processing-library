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
  // The placeholder "auto", resolved by the resampling options.
  [[nodiscard]] bool is_auto() const { return _is_auto; }
  // The scaling-dependent trapezoid kernel, usable only in scale().
  [[nodiscard]] bool is_trapezoid() const { return _is_trapezoid; }

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
  // trapezoid:   area-antialiasing kernel whose radius .5 + .5 min(s, 1 / s) depends on the scaling s; in scale().
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
  bool _is_trapezoid{false};

 private:
  string _name;
  KernelFunc _func;
  double _radius;
};

class FilterBnd {
 public:
  explicit FilterBnd(const Filter& filter_, Bndrule bndrule_) : _filter(&filter_) { set_bndrule(bndrule_); }
  [[nodiscard]] const Filter& filter() const { return *_filter; }
  [[nodiscard]] Bndrule bndrule() const { return _bndrule; }
  void set_filter(const Filter& filter_) { _filter = &filter_; }
  void set_bndrule(Bndrule bndrule_) { _bndrule = bndrule_, assertx(_bndrule != Bndrule::undefined); }
  friend std::ostream& operator<<(std::ostream& os, const FilterBnd& filterbnd) {
    return os << "FilterBnd{" << filterbnd._filter->name() << ", " << boundaryrule_name(filterbnd._bndrule) << "}";
  }

 private:
  const Filter* _filter;
  Bndrule _bndrule;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_FILTER_H_
