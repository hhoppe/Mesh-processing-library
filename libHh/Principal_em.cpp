// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Principal.h"

#include "LLS.h"
#include "Random.h"
#include "Timer.h"

namespace hh {

static void orthonormalize_columns(MatrixView<float> m) {
  for_int(x, m.xsize()) {
    for_int(x2, x) {
      double ddot = 0.;
      for_int(y, m.ysize()) ddot += m[y][x] * m[y][x2];
      float fdot = float(ddot);
      for_int(y, m.ysize()) m[y][x] -= m[y][x2] * fdot;
    }
    double mag2 = 0.;
    for_int(y, m.ysize()) mag2 += square(m[y][x]);
    float fac = 1.f / assertx(float(sqrt(mag2)));
    for_int(y, m.ysize()) m[y][x] *= fac;
  }
}

static void unused_orthonormalize_rows(MatrixView<float> m) {
  for_int(y, m.ysize()) {
    for_int(y2, y) {
      double ddot = 0.;
      for_int(x, m.xsize()) ddot += m[y][x] * m[y2][x];  // or: =dot(m[y], m[y2])
      float fdot = float(ddot);
      for_int(x, m.xsize()) m[y][x] -= m[y2][x] * fdot;
    }
    // or: normalize(m[y])
    double mag2 = 0.;
    for_int(x, m.xsize()) mag2 += square(m[y][x]);
    float fac = 1.f / assertx(float(sqrt(mag2)));
    for_int(x, m.xsize()) m[y][x] *= fac;
  }
}

bool em_principal_components(CMatrixView<float> mi, MatrixView<float> mo, ArrayView<float> eimag, int niter) {
  const int m = mi.ysize(), n = mi.xsize(), ne = mo.ysize();
  assertx(m >= 1 && n >= 1);
  assertx(ne >= 1 && ne <= m && ne <= n);
  assertx(mo.xsize() == n);
  assertx(niter >= 1);
  assertx(eimag.num() == ne);
  dummy_use(unused_orthonormalize_rows);
  //
  auto up_timer = m * n > 10000 * 100 ? make_unique<Timer>("__pca_em") : nullptr;
  // C = rand(p, k);
  Matrix<float> mc(n, ne);
  Random random;
  for_int(i, n) for_int(c, ne) mc[i][c] = random.unif();
  for_int(iter, niter) {
    Matrix<float> x(ne, m);
    {
      // Expectation step
      //  x = inv(CT * C) * CT * data;   i.e. least-squares solution of C x = data;
      //  C[n][ne] = mc;  x[ne][m];  data[n][m] = transpose(mi);  x is projection of data into C.
      QrdLLS lls(n, ne, m);
      lls.enter_a(mc);
      for_int(i, m) lls.enter_b_c(i, mi[i]);
      if (!lls.solve()) return false;
      lls.get_x(x);
    }
    {
      // Maximization step
      //  C = data*xT*inv(x*xT);   i.e. least-squares solution of xT CT = dataT
      QrdLLS lls(m, ne, n);
      for_int(i, ne) lls.enter_a_c(i, x[i]);
      lls.enter_b(mi);
      if (!lls.solve()) return false;
      lls.get_x(mo);  // use mo[ne][n] as temporary variable
      mc = transpose(mo);
    }
  }
  // C = orth(C);
  // [xevec,eval] = truepca(CT*data);   x=CT*data is [ne][m];  xevec is [ne][ne]
  // evec = C*xevec;   [n][ne]=[n][ne]*[ne][ne]
  orthonormalize_columns(mc);
  Matrix<float> xt(m, ne);
  // xt = (CT*data)T = dataT*C = mi*mc
  for_int(r, m) for_int(c, ne) {
    double s = 0.;
    for_int(j, n) s += mi[r][j] * mc[j][c];
    xt[r][c] = float(s);
  }
  Matrix<float> xtpc(ne, ne);
  principal_components(xt, xtpc, eimag);
  assertx(xtpc.ysize() == ne && xtpc.xsize() == ne);
  // mo = xevecT*CT = xtpc*CT   [ne][n] = [ne][ne]*[ne][n]
  for_int(r, ne) for_int(c, n) {
    double s = 0.;
    for_int(j, ne) s += xtpc[r][j] * mc[c][j];
    mo[r][c] = float(s);
  }
  // Orient eigenvectors canonically.
  Array<float> all1(n, 1.f);
  for_int(i, ne) {
    if (dot(mo[i], all1) < 0.) mo[i] *= -1.f;
  }
  return true;
}

// Inspired from Matlab routine by Roweis:
//
// function [evec, eval] = empca(data, k, iter, Cinit)
// %[evec, eval] = empca(data, k, iter, Cinit)
// %
// % EMPCA
// %
// % finds the first k principal components of a dataset
// % and their associated eigenvales using the EM-PCA algorithm
// %
// % Inputs:  data is a matrix holding the input data
// %               each COLUMN of data is one data vector
// %               NB: mean will be subtracted and discarded
// %          k    is # of principal components to find
// %
// % optional:
// %          iters is the number of iterations of EM to run (default 20)
// %          Cinit is the initial (current) guess for C (default random)
// %
// % Outputs:  evec holds the eigenvectors (one per column)
// %           eval holds the eigenvalues
// %
//
//
// [d, N]  = size(data);
// data = data - mean(data,2)*ones(1,N);
//
// if(nargin<4) Cinit=[]; end
// if(nargin<3) iter=20; end
//
// [evec, eval] = empca_orth(data, empca_iter(data, Cinit, k, iter));
//
//
//
// function [C] = empca_iter(data, Cinit, k, iter)
// %[C] = empca_iter(data, Cinit, k, iter)
// %
// % EMPCA_ITER
// %
// % (re)fits the model
// %
// %    data = Cx + gaussian noise
// %
// % with EM using x of dimension k
// %
// % Inputs:  data is a matrix holding the input data
// %               each COLUMN of data is one data vector
// %               NB: DATA SHOULD BE ZERO MEAN!
// %          k    is dimension of latent variable space
// %               (# of principal components)
// %          Cinit is the initial (current) guess for C
// %          iters is the number of iterations of EM to run
// %
// % Outputs: C is a (re)estimate of the matrix C
// %             whose columns span the principal subspace
// %
//
// % check sizes and stuff
// [p, N] = size(data);
// assert(k<=p);
// if(isempty(Cinit))
//   C = rand(p, k);
// else
//   assert(k==size(Cinit, 2));
//   assert(p==size(Cinit, 1));
//   C = Cinit;
// end
//
// % business part of the code -- looks just like the math!
// for i=1:iter
//        % e step -- estimate unknown x by random projection
//   x = inv(C'*C)*C'*data;
//        % m step -- maximize likelihood wrt C given these x values
//   C = data*x'*inv(x*x');
// end
//
//
//
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// function [evec, eval] = empca_orth(data, C)
// %[evec, eval] = empca_orth(data, Cfinal)
// %
// % EMPCA_ORTH
// %
// % Finds eigenvectors and eigenvalues given a matrix C whose columns span the
// % principal subspace.
// %
// % Inputs:  data is a matrix holding the input data
// %               each COLUMN of data is one data vector
// %               NB: DATA SHOULD BE ZERO MEAN!
// %          Cfinal is the final C matrix from empca.m
// %
// % Outputs: evec, eval are the eigenvectors and eigenvalues found
// %          by projecting the data into C's column space and finding and
// %          ordered orthogonal basis using a vanilla pca method
// %
//
//   C = orth(C);
//   [xevec, eval] = truepca(C'*data);
//   evec = C*xevec;

}  // namespace hh
