// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_FILTER_H_
#define MESH_PROCESSING_LIBHH_FILTER_H_

#include "Array.h"
#include "Matrix.h"

namespace hh {

using KernelFunc = double (*)(double);

class Filter : noncopyable {
 public:
  explicit Filter(string pname, KernelFunc pfunc, double pradius)
      : _name(std::move(pname)), _func(pfunc), _radius(pradius) {}
  virtual ~Filter() {}
  string name() const { return _name; }
  KernelFunc func() const { return assertx(_func); }
  double radius() const { return _radius; }
  bool is_interpolating() const { return _is_interpolating; }
  bool is_discontinuous() const { return _is_discontinuous; }
  bool has_inv_convolution() const { return _has_inv_convolution; }
  bool is_trivial_magnify() const { return _is_trivial_magnify; }  // impulse or box
  bool is_trivial_minify() const { return _is_trivial_minify; }    // box
  bool is_impulse() const { return _is_impulse; }
  bool is_omoms() const { return _is_omoms; }            // omoms or justomoms
  bool is_preprocess() const { return _is_preprocess; }  // preprocess
  bool is_partition_of_unity() const { return _is_partition_of_unity; }
  bool is_unit_integral() const { return _is_unit_integral; }
  //
  static const Filter& get(const string& name);
  // impulse:     (for nearest-sample minification) (does not support func())
  // box:         (nearest magnification), a.k.a. rectangle/rect func, top-hat, pi func, unit pulse, boxcar)
  // triangle:    a.k.a. linear, tent, hat, triangular func
  // quadratic:   (note: discontinuous like "box")
  // mitchell:    somewhat blurry bicubic
  // keys:        sharper bicubic; equivalent to Catmull-Rom spline
  // spline:      cardinal bicubic B-spline; inverse_convolution1 + justspline
  // omoms:       maximal-order minimal-support; inverse_convolution2 + justomoms
  // gaussian:    non-interpolating!
  // preprocess:  cubic B-spline inverse-convolution1 preprocess
  // justspline:  just the approximating bicubic B-spline (evaluated after inverse_convolution1)
  // justomoms:   just the approximating OMOMS kernel     (evaluated after inverse_convolution2)
  // lanczos6:    Lanczos filter with support [-3, 3]
  // lanczos10:   Lanczos filter with support [-5, 5] -- best but slowest
  // hamming6:    Hamming filter with support [-3, 3]
  friend std::ostream& operator<<(std::ostream& os, const Filter& filter) {
    return os << "Filter{" << filter.name() << "}";
  }

 private:
  string _name;
  KernelFunc _func;
  double _radius;

 protected:
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
};

struct LUfactorization;

class FilterBnd {
 public:
  explicit FilterBnd(const Filter& pfilter, Bndrule pbndrule) : _filter(&pfilter) { set_bndrule(pbndrule); }
  const Filter& filter() const { return *_filter; }
  Bndrule bndrule() const { return _bndrule; }
  void set_filter(const Filter& pfilter) { _filter = &pfilter; }
  void set_bndrule(Bndrule pbndrule) { _bndrule = pbndrule, assertx(_bndrule != Bndrule::undefined); }
  // Given a filtertype, return an evaluation function and its support radius (centered about origin).
  const LUfactorization& lu_factorization() const;
  // Given old size cx and new size cx, for each new index i compute the interval of weights on old values
  //  starting at index ar_pixelindex0[i].  mat_weights is allocated of size [nx][nk] where nk is interval size.
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
  Array<float> Llower;       // lower off-diagonal of L (except last row)
  Array<float> Llastrow;     // last row of L (except last two elements)
  Array<float> LlastrowPen;  // penultimate element of last row of L, as a function of matrix size ([0]=size1)
  Array<float> Uidiag;       // inverse diagonal of U (except last row)
  Array<float> UidiagLast;   // last diagonal element of U, as a function of matrix size
  Array<float> Ulastcol;     // last column of U (except last two elements)
  Array<float> UlastcolPen;  // penultimate element of last column of U, as a function of matrix size ([0]=size1)
  float Uupper;              // upper off-diagonal of U, constant along the diagonal
  void validate() const {
    bool last_special = Llastrow.num() > 0;
    assertx(last_special == (LlastrowPen.num() > 0));
    assertx(last_special == (Ulastcol.num() > 0));
    assertx(last_special == (UlastcolPen.num() > 0));
    if (last_special) assertx(Ulastcol.num() == Uidiag.num());
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_FILTER_H_
