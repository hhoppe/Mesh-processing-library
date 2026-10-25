// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_DIGITALFILTER_H_
#define MESH_PROCESSING_LIBHH_DIGITALFILTER_H_

#include "libHh/Boundary.h"
#include "libHh/Filter.h"
#include "libHh/Matrix.h"
#include "libHh/StridedArrayView.h"

namespace hh {

// The digital filter (inverse convolution) that makes a generalized kernel such as cardinal3 or omoms5 interpolate.
// For a 1D grid of `size` samples x_i, it computes the coefficients c_j such that the reconstruction with the kernel
// evaluated after the digital filter (e.g. bspline3 for cardinal3), sum_j k(i - j) c_j, where the sum honors the
// boundary rule, equals x_i at every sample.  The matrix is banded, plus corner entries for "wrap"; its LU
// factorization is computed once per (filter, boundary, grid type, size) and cached forever.
class DigitalFilter : noncopyable {
 public:
  [[HH_NO_DANGLING]] [[nodiscard]] static const DigitalFilter& get(const Filter& filter, const Boundary& boundary,
                                                                   Gridtype gridtype, int size);
  [[nodiscard]] int size() const { return _size; }
  // Replace the samples in `row` by the coefficients; cval is the constant value referenced by some boundary rules.
  template <typename T> void solve(StridedArrayView<T> row, const T* cval) const;

 private:
  DigitalFilter(const Filter& filter, const Boundary& boundary, Gridtype gridtype, int size);
  int _size;
  int _bw{0};      // Bandwidth of the banded block B (lower and upper).
  int _corner{0};  // Number of trailing rows and columns treated densely (for "wrap"); B has size - corner rows.
  bool _uses_cval{false};
  Matrix<float> _lower;       // [size - corner][bw]: the multipliers L(i, i - 1 - k) of the unit lower factor.
  Matrix<float> _upper;       // [size - corner][bw]: the entries U(i, i + 1 + k) of the upper factor.
  Array<float> _idiag;        // [size - corner]: 1 / U(i, i).
  Matrix<float> _binv_c;      // [size - corner][corner]: B^-1 C, where C holds the trailing columns.
  Matrix<float> _r;           // [corner][size - corner]: the trailing rows R.
  Matrix<float> _s_inv;       // [corner][corner]: the inverse of the Schur complement D - R B^-1 C.
  Array<float> _cval_weight;  // [size]: the weight of cval in each row (0 if none).
};

template <typename T> void DigitalFilter::solve(StridedArrayView<T> row, const T* cval) const {
  const int n = _size, m = _corner, nb = n - m, bw = _bw;
  ASSERTX(row.num() == n);
  if (_uses_cval) {
    assertx(cval);
    for_int(i, n) {
      if (_cval_weight[i]) row[i] -= _cval_weight[i] * *cval;
    }
  }
  // Forward substitution L y = x and backward substitution U c = y on the banded block.
  if (bw == 1) {  // The common case (cubic kernels): a tridiagonal system.
    for_intL(i, 1, nb) row[i] -= _lower[i, 0] * row[i - 1];
    if (nb > 0) row[nb - 1] = _idiag[nb - 1] * row[nb - 1];
    for (int i = nb - 2; i >= 0; --i) row[i] = _idiag[i] * (row[i] - _upper[i, 0] * row[i + 1]);
  } else {
    for_int(i, nb) {
      T v = row[i];
      for_int(k, min(bw, i)) v -= _lower[i, k] * row[i - 1 - k];
      row[i] = v;
    }
    for (int i = nb - 1; i >= 0; --i) {
      T v = row[i];
      for_int(k, min(bw, nb - 1 - i)) v -= _upper[i, k] * row[i + 1 + k];
      row[i] = _idiag[i] * v;
    }
  }
  if (m) {  // The corner: z = S^-1 (x2 - R y1); then x1 -= B^-1 C z and x2 = z.
    Array<T> z(m);
    for_int(c, m) {
      T v = row[nb + c];
      for_int(j, nb) {
        if (_r[c, j]) v -= _r[c, j] * row[j];
      }
      z[c] = v;
    }
    Array<T> z2(m);
    for_int(c, m) {
      T v{};
      for_int(e, m) v += _s_inv[c, e] * z[e];
      z2[c] = v;
    }
    for_int(j, nb) for_int(c, m) row[j] -= _binv_c[j, c] * z2[c];
    for_int(c, m) row[nb + c] = z2[c];
  }
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_DIGITALFILTER_H_
