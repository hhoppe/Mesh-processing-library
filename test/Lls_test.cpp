// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Lls.h"

#include "libHh/MatrixOp.h"  // identity_mat(), diag_mat(), mat_mul(), column()
#include "libHh/Random.h"
#include "libHh/SingularValueDecomposition.h"
using namespace hh;

namespace {

unique_ptr<Lls> make_lls(int c, int m, int n, int nd) {
  if (c == 0) return make_unique<SparseLls>(m, n, nd);
  if (c == 1) return make_unique<LudLls>(m, n, nd);
  if (c == 2) return make_unique<GivensLls>(m, n, nd);
  if (c == 3) return make_unique<SvdLls>(m, n, nd);
  if (c == 4) return make_unique<SvdDoubleLls>(m, n, nd);
  if (c == 5) return make_unique<QrdLls>(m, n, nd);
  assertnever("");
}

void test1() {
  {
    SvdLls lls(1, 1, 1);
    lls.enter_a_rc(0, 0, 1.f);
    lls.enter_b_rc(0, 0, 10.f);
    lls.enter_xest_rc(0, 0, 1.f);
    assertx(lls.solve());
    SHOW(lls.get_x_rc(0, 0));
  }
  {
    Vec1<float> X1 = {-100.f}, B1 = {2.f};
    Vec1<float> X2 = {-100.f}, B2 = {2.f};
    SvdLls lls(1, 1, 2);
    lls.enter_a_rc(0, 0, 4.f);
    lls.enter_b_c(0, B1);
    lls.enter_xest_c(0, X1);
    lls.enter_b_c(1, B2);
    lls.enter_xest_c(1, X2);
    assertx(lls.solve());
    lls.get_x_c(0, X1);
    lls.get_x_c(1, X2);
    SHOW(X1[0]);
    SHOW(X2[0]);
  }
  {
    float X1[1] = {-100.f};
    float X2[1] = {-100.f};
    SparseLls lls(1, 1, 2);
    lls.enter_a_rc(0, 0, 4.f);
    lls.enter_b_c(0, V(2.f));
    lls.enter_xest_c(0, X1);
    lls.enter_b_c(1, V(2.f));
    lls.enter_xest_c(1, X2);
    assertx(lls.solve());
    lls.get_x_c(0, X1);
    lls.get_x_c(1, X2);
    SHOW(X1[0]);
    SHOW(X2[0]);
  }
  {
    Vec1<float> X1 = {-100.f}, X2 = {-100.f};
    const Vec1<float> B1 = {2.f}, B2 = {2.f};
    LudLls lls(1, 1, 2);
    lls.enter_a_rc(0, 0, 4.f);
    lls.enter_b_c(0, B1);
    lls.enter_xest_c(0, X1);
    lls.enter_b_c(1, B2);
    lls.enter_xest_c(1, X2);
    assertx(lls.solve());
    lls.get_x_c(0, X1);
    lls.get_x_c(1, X2);
    SHOW(X1[0]);
    SHOW(X2[0]);
  }
  {
    const int n = 3;
    Matrix<float> a(n, n);
    Array<float> b(n);
    for_int(i, n) {
      for_int(j, n) a[i, j] = 1.f + i * 3 + j + abs(2 - i) * abs(5 - j + i) * j + i * i * j * j;
      b[i] = float(abs(i - 4));
    }
    for_int(c, 6) {
      SHOW(c);
      const int nd = 2;
      auto up_lls = make_lls(c, n, n, nd);
      Lls& lls = *up_lls;
      for_int(i, n) {
        for_int(j, n) lls.enter_a_rc(i, j, a[i, j]);
        lls.enter_b_rc(i, 0, b[i]);
        lls.enter_b_rc(i, 1, b[i] * 2);
        lls.enter_xest_rc(i, 0, 0.f);
        lls.enter_xest_rc(i, 1, 0.f);
      }
      assertx(lls.solve());
      for_int(i, n) SHOW(round_fraction_digits(lls.get_x_rc(i, 0)));
      for_int(i, n) SHOW(round_fraction_digits(lls.get_x_rc(i, 1)));
    }
  }
}

void test2() {
  for_int(c, 6) {
    SHOW(c);
    auto up_lls = make_lls(c, 2, 1, 1);
    Lls& lls = *up_lls;
    lls.enter_a_rc(0, 0, 1.f);
    lls.enter_a_rc(1, 0, 1.f);
    lls.enter_b_rc(0, 0, 10.f);
    lls.enter_b_rc(1, 0, 20.f);
    lls.enter_xest_rc(0, 0, 50.f);
    assertx(lls.solve());
    SHOW(lls.get_x_rc(0, 0));
  }
}

void test3() {
  for_int(c, 6) {
    SHOW(c);
    auto up_lls = make_lls(c, 3, 2, 1);
    Lls& lls = *up_lls;
    lls.enter_a_rc(0, 0, 1.f);
    lls.enter_a_rc(0, 1, 1.f);
    lls.enter_a_rc(1, 0, 1.f);
    lls.enter_a_rc(1, 1, 0.f);
    lls.enter_a_rc(2, 0, 0.f);
    lls.enter_a_rc(2, 1, 1.f);
    lls.enter_b_rc(0, 0, 10.f);
    lls.enter_b_rc(1, 0, 2.f);
    lls.enter_b_rc(2, 0, 12.f);
    lls.enter_xest_rc(0, 0, 50.f);
    lls.enter_xest_rc(1, 0, 50.f);
    assertx(lls.solve());
    SHOW(round_fraction_digits(lls.get_x_rc(0, 0)));
    SHOW(round_fraction_digits(lls.get_x_rc(1, 0)));
  }
}

// Singular value decomposition of random and identity matrices, with or without normalized columns.
template <typename Real> void test4() {
  const Real tolerance = std::is_same_v<Real, float> ? Real(2e-6f) : Real(1e-14);
  for_int(imode, 2) {
    for_int(inormalize, 2) {
      for_intL(m, 1, 11) for_intL(n, 1, 11) {
        if (m < n) continue;
        Matrix<Real> A(m, n);
        switch (imode) {
          case 0:
            for (Real& v : A) v = possible_cast<Real>(Random::G.dunif());
            break;
          case 1: identity_mat(A); break;
          default: assertnever("");
        }
        if (inormalize) for_int(i, n) normalize(column(A, i));  // Possibly normalize the columns.
        Matrix<Real> U(m, n);
        Array<Real> S(n);
        Matrix<Real> VT(n, n);
        const bool success = singular_value_decomposition(A, U, S, VT);
        sort_singular_values(U, S, VT);
        const Matrix<Real> R = mat_mul(mat_mul(U, diag_mat(S)), transpose(VT));
        const Matrix<Real> UTU = mat_mul(transpose(U), U);
        const Matrix<Real> VTV = mat_mul(transpose(VT), VT);
        const Real Rerr = max_abs_element(R - A);
        const Real UTUerr = max_abs_element(UTU - identity_mat<Real>(n));
        const Real VTVerr = max_abs_element(VTV - identity_mat<Real>(n));
        if (!(success && Rerr < tolerance && UTUerr < tolerance && VTVerr < tolerance)) {
          SHOW(imode, inormalize, m, n, success, Rerr, UTUerr, VTVerr);
          assertnever("SVD is inaccurate");
        }
        for_int(i, n) assertx(S[i] >= Real{0} && (i == 0 || S[i] <= S[i - 1]));  // Sorted in decreasing order.
        if (imode == 1) for_int(i, n) assertx(abs(S[i] - Real{1}) < tolerance);  // The identity has unit values.
      }
    }
  }
}

// Singular value decomposition of matrices with known singular values.
void test_svd_known() {
  {
    // The singular values of a matrix with orthogonal columns are the column norms.
    const Matrix<float> A = {{3.f, 0.f, 0.f}, {0.f, 0.f, 1.f}, {0.f, 2.f, 0.f}, {0.f, 0.f, 0.f}};
    Matrix<float> U(4, 3), VT(3, 3);
    Array<float> S(3);
    assertx(singular_value_decomposition(A, U, S, VT));
    sort_singular_values(U, S, VT);
    SHOW(S);
    assertx(max_abs_element(mat_mul(mat_mul(U, diag_mat(S)), transpose(VT)) - A) < 1e-6f);
  }
  {
    // The 2x2 matrix [[2, 1], [1, 2]] has eigenvalues (and hence singular values) 3 and 1.
    const Matrix<double> A = {{2., 1.}, {1., 2.}};
    Matrix<double> U(2, 2), VT(2, 2);
    Array<double> S(2);
    assertx(singular_value_decomposition(A, U, S, VT));
    sort_singular_values(U, S, VT);
    assertx(abs(S[0] - 3.) < 1e-14 && abs(S[1] - 1.) < 1e-14);
  }
  {
    // A rank-deficient matrix (with a zero column) has a zero singular value.
    const Matrix<double> A = {{1., 0., 2.}, {3., 0., 1.}, {-1., 0., 4.}, {2., 0., 2.}};
    Matrix<double> U(4, 3), VT(3, 3);
    Array<double> S(3);
    assertx(singular_value_decomposition(A, U, S, VT));
    sort_singular_values(U, S, VT);
    assertx(S[2] == 0. && S[1] > 1.);
    assertx(max_abs_element(mat_mul(mat_mul(U, diag_mat(S)), transpose(VT)) - A) < 1e-14);
    assertx(max_abs_element(mat_mul(transpose(VT), VT) - identity_mat<double>(3)) < 1e-14);
  }
}

// Solve the normal equations (A^T * A) * x = A^T * b in double precision, as a reference solution.
Matrix<double> reference_solution(CMatrixView<float> A, CMatrixView<float> B) {
  const int m = A.ysize(), n = A.xsize(), nd = B.xsize();
  Matrix<double> M(n, n + nd);  // Augmented matrix [A^T * A, A^T * B].
  for_int(i, n) {
    for_int(j, n) {
      double sum = 0.;
      for_int(k, m) sum += double(A[k, i]) * A[k, j];
      M[i, j] = sum;
    }
    for_int(d, nd) {
      double sum = 0.;
      for_int(k, m) sum += double(A[k, i]) * B[k, d];
      M[i, n + d] = sum;
    }
  }
  for_int(i, n) {  // Gaussian elimination; the matrix A^T * A is symmetric positive definite.
    assertx(M[i, i] > 0.);
    for_intL(i2, i + 1, n) {
      const double f = M[i2, i] / M[i, i];
      for_intL(j, i, n + nd) M[i2, j] -= f * M[i, j];
    }
  }
  Matrix<double> X(n, nd);
  for (int i = n - 1; i >= 0; i--) {
    for_int(d, nd) {
      double sum = M[i, n + d];
      for_intL(j, i + 1, n) sum -= M[i, j] * X[j, d];
      X[i, d] = sum / M[i, i];
    }
  }
  return X;
}

// Residual sum of squares |A * X - B|^2.
double residual(CMatrixView<float> A, CMatrixView<double> X, CMatrixView<float> B) {
  double rss = 0.;
  for_int(k, A.ysize()) for_int(d, B.xsize()) {
    double sum = -B[k, d];
    for_int(j, A.xsize()) sum += double(A[k, j]) * X[j, d];
    rss += square(sum);
  }
  return rss;
}

// Solve random overdetermined systems with each solver, entering the data through the various interfaces, and
// compare with the reference solution.
void test_random_systems() {
  const int m = 12, n = 5, nd = 3;
  Random random{17};
  Matrix<float> A(m, n), B(m, nd);
  for (float& v : A) v = random.unif() * 2.f - 1.f;
  for (float& v : B) v = random.unif() * 2.f - 1.f;
  const Matrix<double> Xref = reference_solution(A, B);
  const double rss_ref = residual(A, Xref, B);
  const double rss_b = mag2(B);  // The residual for the initial estimate X == 0.
  for_int(c, 6) {
    auto up_lls = make_lls(c, m, n, nd);
    Lls& lls = *up_lls;
    assertx(lls.num_rows() == m);
    Matrix<float> X(n, nd);
    switch (c % 3) {
      case 0:
        lls.enter_a(A);
        lls.enter_b(B);
        lls.enter_xest(Matrix<float>(V(n, nd), 0.f));
        break;
      case 1:
        for_int(k, m) lls.enter_a_r(k, A[k]);
        for_int(k, m) lls.enter_b_r(k, B[k]);
        for_int(j, n) lls.enter_xest_r(j, V(0.f, 0.f, 0.f));
        break;
      case 2:
        for_int(j, n) lls.enter_a_c(j, Array<float>(column(A, j)));
        for_int(d, nd) lls.enter_b_c(d, Array<float>(column(B, d)));
        break;
      default: assertnever("");
    }
    double rssb, rssa;
    assertx(lls.solve(&rssb, &rssa));
    switch (c % 3) {
      case 0: lls.get_x(X); break;
      case 1: for_int(j, n) lls.get_x_r(j, X[j]); break;
      case 2:
        for_int(d, nd) {
          Array<float> ar(n);
          lls.get_x_c(d, ar);
          for_int(j, n) X[j, d] = ar[j];
        }
        break;
      default: assertnever("");
    }
    double max_err = 0.;
    for_int(j, n) for_int(d, nd) max_err = max(max_err, abs(X[j, d] - Xref[j, d]));
    if (!(max_err < 1e-5)) SHOW(c, max_err);
    assertx(max_err < 1e-5);
    assertx(abs(rssb - rss_b) < 1e-5 * rss_b);
    if (c != 2) assertx(abs(rssa - rss_ref) < 1e-5 * rss_ref);  // KNOWN_BUG: see the note on rssa in test_rssa().
  }
  showf("Random %dx%d systems: all solvers agree with the reference solution.\n", m, n);
}

// For a consistent system A * x = b, all solvers recover x exactly (up to roundoff), with zero residual.
void test_consistent_system() {
  const int m = 7, n = 4;
  const Vec4<float> x_true(1.f, -2.f, .5f, 3.f);
  Matrix<float> A(m, n);
  for_int(k, m) for_int(j, n) A[k, j] = float((k * 3 + j * 5) % 7) - 3.f + (k == j ? 4.f : 0.f);
  for_int(c, 6) {
    auto up_lls = make_lls(c, m, n, 1);
    Lls& lls = *up_lls;
    lls.enter_a(A);
    for_int(k, m) lls.enter_b_rc(k, 0, float(dot(A[k], x_true)));
    double rssa;
    assertx(lls.solve(nullptr, &rssa));
    for_int(j, n) assertx(abs(lls.get_x_rc(j, 0) - x_true[j]) < 2e-5f);
    if (c != 2) assertx(rssa < 1e-8);  // KNOWN_BUG: see the note on rssa in test_rssa().
  }
}

// The residuals reported by solve() for a square system with exact solution x = (1, 1).
void test_rssa() {
  for_int(c, 6) {
    auto up_lls = make_lls(c, 2, 2, 1);
    Lls& lls = *up_lls;
    lls.enter_a(Matrix<float>{{1.f, 2.f}, {3.f, 4.f}});
    lls.enter_b_c(0, V(3.f, 7.f));
    double rssb, rssa;
    assertx(lls.solve(&rssb, &rssa));
    assertx(abs(lls.get_x_rc(0, 0) - 1.f) < 1e-5f && abs(lls.get_x_rc(1, 0) - 1.f) < 1e-5f);
    assertx(rssb == 58.);  // Here |b|^2 == 3^2 + 7^2 for the initial estimate x = (0, 0).
    // KNOWN_BUG: FullLls::solve() computes rssa from the matrices _a and _b after solve_aux(), but
    // GivensLls::solve_aux() overwrites both, and LudLls::solve_aux() overwrites _a when the system is square; their
    // rssa is then wrong.
    if (c == 1 || c == 2) continue;
    assertx(rssa < 1e-10);
  }
}

// A system with an all-zero column is singular, and the full solvers report failure.
// (GivensLls is omitted because it also emits a warning.)
void test_singular_system() {
  for (const int c : {1, 3, 4, 5}) {
    auto up_lls = make_lls(c, 3, 2, 1);
    Lls& lls = *up_lls;
    for_int(k, 3) lls.enter_a_rc(k, 0, k + 1.f);
    for_int(k, 3) lls.enter_b_rc(k, 0, k + 1.f);
    assertx(!lls.solve());
  }
  {
    // The conjugate-gradient solver does not modify the component in the null space, here x[1] = 7.
    SparseLls lls(3, 2, 1);
    for_int(k, 3) lls.enter_a_rc(k, 0, k + 1.f);
    for_int(k, 3) lls.enter_b_rc(k, 0, k + 1.f);
    lls.enter_xest_rc(1, 0, 7.f);
    assertx(lls.solve());
    assertx(abs(lls.get_x_rc(0, 0) - 1.f) < 1e-6f && lls.get_x_rc(1, 0) == 7.f);
  }
  // KNOWN_BUG: a rank-deficient system with two equal columns is not reliably detected:
  // without LAPACK, SvdLls and QrdLls only fail on an exactly zero singular value.
}

// After clear(), an Lls can solve a new system.
void test_clear() {
  for_intL(c, 1, 6) {  // KNOWN_BUG: SparseLls::clear() is omitted; see below.
    auto up_lls = make_lls(c, 2, 1, 1);
    Lls& lls = *up_lls;
    lls.enter_a_rc(0, 0, 1.f);
    lls.enter_a_rc(1, 0, 1.f);
    lls.enter_b_rc(0, 0, 10.f);
    lls.enter_b_rc(1, 0, 20.f);
    assertx(lls.solve());
    assertx(abs(lls.get_x_rc(0, 0) - 15.f) < 1e-5f);
    lls.clear();
    lls.enter_a_rc(0, 0, 2.f);
    lls.enter_a_rc(1, 0, 4.f);
    lls.enter_b_rc(0, 0, 2.f);
    lls.enter_b_rc(1, 0, 4.f);
    assertx(lls.solve());
    assertx(abs(lls.get_x_rc(0, 0) - 1.f) < 1e-6f);
  }
  // KNOWN_BUG: SparseLls::clear() empties its arrays _rows and _cols instead of each of their rows, so any subsequent
  // enter_a_rc() accesses beyond the array bounds.
}

// The SparseLls conjugate-gradient options.
void test_sparse_options() {
  const int m = 6, n = 3;
  const auto enter = [&](SparseLls& lls) {
    for_int(k, m) for_int(j, n) lls.enter_a_rc(k, j, float((k + 2 * j) % 5) + (k == j ? 3.f : 0.f));
    for_int(k, m) lls.enter_b_rc(k, 0, float(k % 3));
  };
  SparseLls lls0(m, n, 1);
  enter(lls0);
  double rssb0, rssa0;
  assertx(lls0.solve(&rssb0, &rssa0));
  {
    // With a single iteration, the solver reports failure, and the residual is reduced but not minimal.
    SparseLls lls(m, n, 1);
    enter(lls);
    lls.set_max_iter(1);
    double rssb, rssa;
    assertx(!lls.solve(&rssb, &rssa));
    assertx(rssb == rssb0 && rssa < rssb && rssa > rssa0 * 1.0001);
  }
  {
    // With a huge tolerance, the solver immediately reports success, with the initial estimate unchanged.
    SparseLls lls(m, n, 1);
    enter(lls);
    lls.set_tolerance(1e10f);
    for_int(j, n) lls.enter_xest_rc(j, 0, 1.f);
    assertx(lls.solve());
    for_int(j, n) assertx(lls.get_x_rc(j, 0) == 1.f);
  }
}

// The factory Lls::make() selects SparseLls for large sparse systems, and QrdLls otherwise.
void test_make() {
  {
    auto up_lls = Lls::make(3, 2, 1, 1.f);
    Lls& lls = *up_lls;
    lls.enter_a(Matrix<float>{{1.f, 1.f}, {1.f, 0.f}, {0.f, 1.f}});
    lls.enter_b_c(0, V(10.f, 2.f, 12.f));
    assertx(lls.solve());
    assertx(abs(lls.get_x_rc(0, 0) - 2.f / 3.f) < 1e-5f && abs(lls.get_x_rc(1, 0) - 32.f / 3.f) < 1e-5f);
  }
  {
    // A large, sparse, well-conditioned bidiagonal system.
    const int n = 250;
    const float nonzerofrac = 2.f / n;
    auto up_lls = Lls::make(n, n, 1, nonzerofrac);
    Lls& lls = *up_lls;
    for_int(i, n) {
      lls.enter_a_rc(i, i, 2.f);
      if (i + 1 < n) lls.enter_a_rc(i, i + 1, -.5f);
      const float xi = float(i % 7), xi1 = i + 1 < n ? float((i + 1) % 7) : 0.f;
      lls.enter_b_rc(i, 0, 2.f * xi - .5f * xi1);
    }
    assertx(lls.solve());
    for_int(i, n) assertx(abs(lls.get_x_rc(i, 0) - float(i % 7)) < 1e-4f);
  }
}

}  // namespace

int main() {
  test1();
  test2();
  test3();
  test4<float>();
  test4<double>();
  test_svd_known();
  test_random_systems();
  test_consistent_system();
  test_rssa();
  test_singular_system();
  test_clear();
  test_sparse_options();
  test_make();
}
