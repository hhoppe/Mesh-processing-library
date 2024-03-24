// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_SINGULARVALUEDECOMPOSITION_H_
#define MESH_PROCESSING_LIBHH_SINGULARVALUEDECOMPOSITION_H_

#include "libHh/MatrixOp.h"  // column()
#include "libHh/Matrix.h"
#include "libHh/RangeOp.h"  // fill()

namespace hh {

// Given matrix A, compute its singular value decomposition, expressing A = U * diag(S) * VT^T,
//  where matrices U and VT have orthonormal columns.
// Return: success.
// Approach: one-sided Jacobi iterative algorithm,
//  based on 1989 report by James Demmel and Kresimir Veselic, Algorithm 4.1, p32.
// Implicitly computes the product A*A^T and then uses a sequence of Jacobi rotations to diagonalize it.
// Singular values are not sorted.
template <typename T>
bool singular_value_decomposition(CMatrixView<T> A, MatrixView<T> U, ArrayView<T> S, MatrixView<T> VT);

// Modify the singular value decomposition U * diag(S) * VT^T such that the singular values in S are
// in decreasing order.
template <typename T> void sort_singular_values(MatrixView<T> U, ArrayView<T> S, MatrixView<T> VT);

//----------------------------------------------------------------------------

template <typename T>
bool singular_value_decomposition(CMatrixView<T> A, MatrixView<T> U, ArrayView<T> S, MatrixView<T> VT) {
  static_assert(std::is_floating_point<T>::value, "");
  const int m = A.ysize(), n = A.xsize();
  assertx(n >= 1);
  assertx(m >= n);
  assertx(U.dims() == V(m, n));
  assertx(S.num() == n);
  assertx(VT.dims() == V(n, n));
  using TT = mean_type_t<T>;
  U.assign(A);
  fill(VT, T{0});
  for_int(i, n) VT(i, i) = T{1};
  T eps;
  {                        // compute a factor of machine-precision epsilon
    volatile T v1 = T{1};  // otherwise produces eps == 0 on gcc due to compiler optimizations
    eps = T{1};
    while (eps + v1 > v1) eps *= T{0.5};
    assertx(eps > T{0});
    eps *= T{8};
  }
  for (int iter = 0;; iter++) {
    T max_e = T{0};
    for_intL(j, 1, n) for_int(i, j) {  // for indices i < j of columns of U
      // SHOW(U, VT);
      TT a = TT{0}, b = TT{0}, c = TT{0};
      for_int(k, m) {  // construct 2 * 2 submatrix [ a, c; c, b ] of column inner products on U
        a += square(TT{U(k, i)});
        b += square(TT{U(k, j)});
        c += TT{U(k, i)} * TT{U(k, j)};
      }
      T e = static_cast<T>(abs(c) / sqrt(a * b));
      max_e = max(max_e, e);  // measure non-orthogonality of pair of columns
      // SHOW(iter, j, i, a, b, c, e);
      if (c == TT{0}) continue;  // columns are already orthogonal
      T cs, sn;
      {  // compute Jacobi rotation parameters: cos(theta), sin(theta)
        TT z = (b - a) / (TT{2} * c);
        TT t = sign(z) / (abs(z) + std::hypot(TT{1}, z));  // tan(theta); note that sign(z) is never zero
        cs = T{1} / static_cast<T>(std::hypot(TT{1}, t));
        sn = static_cast<T>(TT{cs} * t);
        // SHOW(z, t, cs, sn);
      }
      for_int(k, m) {  // apply Jacobi rotation to U
        T vlk = U(k, i);
        U(k, i) = cs * vlk - sn * U(k, j);
        U(k, j) = sn * vlk + cs * U(k, j);
      }
      for_int(k, n) {  // apply Jacobi rotation to VT
        T vlk = VT(k, i);
        VT(k, i) = cs * vlk - sn * VT(k, j);
        VT(k, j) = sn * vlk + cs * VT(k, j);
      }
    }
    // SHOW(max_e);
    if (max_e < eps) break;
    const int max_iter = 50;
    if (iter == max_iter) {
      if (Warning("singular_value_decomposition convergence failure")) {
        SHOW(m, n, max_e, iter, sizeof(T));
        if (A.size() < 1000) SHOW(A, U, VT);
      }
      return false;
    }
  }
  for_int(i, n) {
    S[i] = static_cast<T>(mag(column(U, i)));  // singular value is norm of column vector of U
    // normalize the column vector
    if (S[i]) {
      T recip = T{1} / S[i];
      for_int(j, m) U(j, i) *= recip;
    }
  }
  return true;
}

template <typename T> void sort_singular_values(MatrixView<T> U, ArrayView<T> S, MatrixView<T> VT) {
  static_assert(std::is_floating_point<T>::value, "");
  const int m = U.ysize(), n = U.xsize();
  assertx(n >= 1);
  assertx(m >= n);
  assertx(S.num() == n);
  assertx(VT.dims() == V(n, n));
  // Insertion sort
  if (n < 2) return;  // avoid gcc warning "-Waggressive-loop-optimizations"
  for_int(i0, n - 1) {
    int i1 = arg_max(S.slice(i0, n)) + i0;
    if (i0 == i1) continue;
    std::swap(S[i0], S[i1]);
    swap_ranges(column(U, i0), column(U, i1));
    swap_ranges(column(VT, i0), column(VT, i1));
    // swap_ranges(VT[i0], VT[i1]);  // would be the case if we computed VT^T instead of VT
  }
}

// To explore in future, for rectangular array A,
// first compute QR decomposition:  A = Q * [R; 0]  where R is square upper-triangular and Q^T * Q = I
// then perform SVD on R:   R = W * diag(s) * V^T   (hence  W^T * R * V = diag(s))
// then define U = Q * [W, 0; 0, I]
// note that U^T * A * V = [W^T, 0; 0, I] * Q^T * Q * [R; 0] * V
//                       = [W^T * R; 0] * V = [diag(s); 0]
// therefore A = U * [diag(s); 0] * V^T  as desired.

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SINGULARVALUEDECOMPOSITION_H_
