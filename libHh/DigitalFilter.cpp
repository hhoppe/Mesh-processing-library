// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/DigitalFilter.h"

#include <map>
#include <mutex>

namespace hh {

const DigitalFilter& DigitalFilter::get(const Filter& filter, const Boundary& boundary, Gridtype gridtype, int size) {
  using Key = std::tuple<const Filter*, const Boundary*, Gridtype, int>;
  // The factorizations are intentionally never destroyed.
  static auto& cache = *new std::map<Key, const DigitalFilter*>;
  static auto& mutex = *new std::mutex;
  const std::lock_guard<std::mutex> lock(mutex);
  const DigitalFilter*& entry = cache[Key{&filter, &boundary, gridtype, size}];
  if (!entry) entry = new DigitalFilter(filter, boundary, gridtype, size);
  return *entry;
}

DigitalFilter::DigitalFilter(const Filter& filter, const Boundary& boundary, Gridtype gridtype, int size)
    : _size(size) {
  assertx(filter.requires_digital_filter() && !boundary.is_auto());
  assertx(size >= (gridtype == Gridtype::dual ? 1 : 2));
  const Filter& kernel = filter.digital_filter_kernel();
  const int l = int(std::ceil(kernel.radius())) - 1;  // The kernel is sampled at the integer offsets -l .. l.
  Array<float> taps(2 * l + 1);
  for_int(k, 2 * l + 1) taps[k] = float(kernel.func()(double(k - l)));
  // The rows of the matrix: the folded taps merged by column (a column of -1 denotes cval).
  Array<std::map<int, double>> rows(size);
  _cval_weight.init(size, 0.f);
  {
    Array<int> indices;
    Array<float> folded;
    for_int(i, size) {
      indices.clear(), folded.clear();
      boundary.fold(i - l, taps, size, gridtype, .5, 0., indices, folded);
      for_int(k, indices.num()) {
        if (indices[k] < 0) {
          _cval_weight[i] += folded[k], _uses_cval = true;
        } else {
          rows[i][indices[k]] += folded[k];
        }
      }
    }
  }
  if (gridtype == Gridtype::primal && boundary.name() == "wrap") {
    // The last sample repeats the first one and is otherwise unreferenced; keeping it as is makes the matrix
    // nonsingular (as in resampler).
    rows[size - 1].clear(), rows[size - 1][size - 1] = 1., _cval_weight[size - 1] = 0.f;
  }
  // Choose the number of corner rows and columns such that the remaining block is banded (with bandwidth l + 1,
  // allowing the extrapolating rules), e.g. for "wrap".
  for (;;) {
    const int nb = size - _corner;
    int bw = 0;
    for_int(i, nb) {
      for (const auto& [j, v] : rows[i]) {
        if (j < nb) bw = max(bw, abs(i - j));
      }
    }
    if (bw <= l + 1) {
      _bw = bw;
      break;
    }
    _corner++;
    assertx(_corner <= size);
  }
  const int m = _corner, nb = size - m, bw = _bw;
  // The banded LU factorization of B (without pivoting; the matrix is diagonally dominant), in double precision.
  const auto entry = [&](int i, int j) -> double {
    const auto it = rows[i].find(j);
    return it == rows[i].end() ? 0. : it->second;
  };
  Matrix<double> lower(V(nb, bw), 0.), upper(V(nb, bw), 0.);  // L(i, i - 1 - k) and U(i, i + 1 + k).
  Array<double> diag(nb);
  const auto get_lower = [&](int i, int j) { return j < i && i - j <= bw ? lower[i, i - 1 - j] : 0.; };
  const auto get_upper = [&](int i, int j) {
    return j == i ? diag[i] : j > i && j - i <= bw ? upper[i, j - 1 - i] : 0.;
  };
  for_int(i, nb) {
    for_intL(j, max(0, i - bw), i) {  // L(i, j).
      double sum = entry(i, j);
      for_intL(p, max(0, i - bw), j) sum -= get_lower(i, p) * get_upper(p, j);
      lower[i, i - 1 - j] = sum / diag[j];
    }
    for_intL(j, i, min(nb, i + bw + 1)) {  // U(i, j).
      double sum = entry(i, j);
      for_intL(p, max(0, i - bw), i) sum -= get_lower(i, p) * get_upper(p, j);
      if (j == i) {
        assertx(abs(sum) > 1e-12);
        diag[i] = sum;
      } else {
        upper[i, j - 1 - i] = sum;
      }
    }
  }
  const auto banded_solve = [&](ArrayView<double> x) {  // In place, B x = rhs.
    for_int(i, nb) {
      double v = x[i];
      for_int(k, min(bw, i)) v -= lower[i, k] * x[i - 1 - k];
      x[i] = v;
    }
    for (int i = nb - 1; i >= 0; --i) {
      double v = x[i];
      for_int(k, min(bw, nb - 1 - i)) v -= upper[i, k] * x[i + 1 + k];
      x[i] = v / diag[i];
    }
  };
  _lower.init(V(nb, bw)), _upper.init(V(nb, bw)), _idiag.init(nb);
  for_int(i, nb) {
    for_int(k, bw) _lower[i, k] = float(lower[i, k]), _upper[i, k] = float(upper[i, k]);
    _idiag[i] = float(1. / diag[i]);
  }
  // The corner: B^-1 C, the rows R, and the inverse of the Schur complement S = D - R B^-1 C.
  _binv_c.init(V(nb, m)), _r.init(V(m, nb)), _s_inv.init(V(m, m));
  Matrix<double> binv_c(V(nb, m));
  for_int(c, m) {
    Array<double> x(nb);
    for_int(i, nb) x[i] = entry(i, nb + c);
    banded_solve(x);
    for_int(i, nb) binv_c[i, c] = x[i];
  }
  Matrix<double> s(V(m, m));
  for_int(c, m) {
    for_int(e, m) {
      double v = entry(nb + c, nb + e);
      for_int(j, nb) v -= entry(nb + c, j) * binv_c[j, e];
      s[c, e] = v;
    }
  }
  {  // Invert the small matrix s by Gauss-Jordan elimination with partial pivoting.
    Matrix<double> inv(V(m, m), 0.);
    for_int(c, m) inv[c, c] = 1.;
    for_int(c, m) {
      int pivot = c;
      for_intL(e, c + 1, m) {
        if (abs(s[e, c]) > abs(s[pivot, c])) pivot = e;
      }
      assertx(abs(s[pivot, c]) > 1e-12);
      if (pivot != c) {
        for_int(e, m) std::swap(s[c, e], s[pivot, e]), std::swap(inv[c, e], inv[pivot, e]);
      }
      const double scale = 1. / s[c, c];
      for_int(e, m) s[c, e] *= scale, inv[c, e] *= scale;
      for_int(r, m) {
        if (r == c) continue;
        const double factor = s[r, c];
        for_int(e, m) s[r, e] -= factor * s[c, e], inv[r, e] -= factor * inv[c, e];
      }
    }
    for_int(c, m) for_int(e, m) _s_inv[c, e] = float(inv[c, e]);
  }
  for_int(i, nb) for_int(c, m) _binv_c[i, c] = float(binv_c[i, c]);
  for_int(c, m) for_int(j, nb) _r[c, j] = float(entry(nb + c, j));
}

}  // namespace hh
