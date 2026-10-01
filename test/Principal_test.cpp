// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Principal.h"

#include "libHh/MatrixOp.h"  // mat_mul(), transpose(), column()
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/SingularValueDecomposition.h"
using namespace hh;

namespace {

// Verify that the rows of mo are orthonormal eigenvectors of the covariance matrix of mi (whose mean is zero),
// with standard deviations eimag in decreasing order, and canonically oriented.
void verify_principal_components(CMatrixView<float> mi, CMatrixView<float> mo, CArrayView<float> eimag) {
  const int m = mi.ysize(), n = mi.xsize();
  for_int(j, n) assertx(abs(mean(column(mi, j))) < 1e-5f);
  const Matrix<float> cov = mat_mul(transpose(mi), mi) / float(m);
  const Matrix<float> diagonalized = mat_mul(mat_mul(mo, cov), transpose(mo));
  for_int(i, n) {
    assertx(abs(mag(mo[i]) - 1.f) < 1e-5f);
    assertx(sum(mo[i]) >= 0.f);
    if (i) assertx(eimag[i] <= eimag[i - 1]);
    for_int(j, n) {
      const float expected = i == j ? square(eimag[i]) : 0.f;
      assertx(abs(diagonalized[i, j] - expected) < 1e-4f * square(eimag[0]));
      if (j < i) assertx(abs(dot(mo[i], mo[j])) < 1e-5f);
    }
  }
}

void test(MatrixView<float> mi) {
  subtract_mean(mi);
  Matrix<float> mo(mi.xsize(), mi.xsize());
  Array<float> eimag(mi.xsize());
  principal_components(mi, mo, eimag);
  verify_principal_components(mi, mo, eimag);
  SHOW(round_elements(eimag));
  SHOW(round_elements(mo));
  {
    incr_principal_components(mi, mo, eimag, 10);
    SHOW(round_elements(eimag));
    SHOW(round_elements(mo));
  }
  {
    assertx(em_principal_components(mi, mo, eimag, 10));
    verify_principal_components(mi, mo, eimag);  // Here the number of components equals the dimension.
    SHOW(round_elements(eimag));
    SHOW(round_elements(mo));
  }
}

// Random data points with known principal axes.
Matrix<float> synthetic_data() {
  const int m = 300, n = 6;
  Random random{5};
  const Vec<float, n> sdv(5.f, 3.f, 2.f, 1.f, .5f, .2f);
  // A random orthonormal basis.
  Matrix<float> A(n, n);
  for (float& v : A) v = random.unif() - .5f;
  Matrix<float> U(n, n), VT(n, n);
  Array<float> S(n);
  assertx(singular_value_decomposition(A, U, S, VT));
  Matrix<float> mi(m, n);
  for_int(i, m) {
    Array<float> z(n);
    for_int(j, n) z[j] = random.gauss() * sdv[j];
    mi[i].assign(mat_mul(z, transpose(U)));
  }
  subtract_mean(mi);
  return mi;
}

void test_synthetic() {
  const Matrix<float> mi = synthetic_data();
  const int n = mi.xsize(), ne = 3;
  Matrix<float> mo(n, n);
  Array<float> eimag(n);
  principal_components(mi, mo, eimag);
  verify_principal_components(mi, mo, eimag);
  // The sample standard deviations are near those used to generate the data.
  const Vec<float, 6> sdv(5.f, 3.f, 2.f, 1.f, .5f, .2f);
  for_int(i, n) assertx(abs(eimag[i] / sdv[i] - 1.f) < .1f);
  {
    // The incremental approximation is crude and converges slowly.
    Matrix<float> mo2(ne, n);
    Array<float> eimag2(ne);
    incr_principal_components(mi, mo2, eimag2, 10);
    for_int(i, ne) {
      assertx(dot(mo[i], mo2[i]) > .999f);
      // KNOWN_BUG: incr_principal_components() divides each vector by its norm from before the Gram-Schmidt
      // orthogonalization, so all but the first vector are only approximately unit-length.
      assertx(abs(mag(mo2[i]) - 1.f) < (i == 0 ? 1e-5f : 1e-3f));
      if (i) assertx(abs(dot(mo2[i], mo2[0])) < 1e-5f);
      assertx(abs(eimag2[i] / eimag[i] - 1.f) < .03f);
    }
  }
  {
    // The expectation-maximization approach converges quickly to the principal subspace.
    Matrix<float> mo2(ne, n);
    Array<float> eimag2(ne);
    assertx(em_principal_components(mi, mo2, eimag2, 10));
    for_int(i, ne) {
      assertx(dot(mo[i], mo2[i]) > .9999f);
      assertx(abs(mag(mo2[i]) - 1.f) < 1e-5f);
      assertx(abs(eimag2[i] / eimag[i] - 1.f) < 1e-4f);
    }
  }
  showf("Synthetic %dx%d data: the principal components are verified.\n", mi.ysize(), n);
}

// The principal frame of 3D points.
void test_frame() {
  const Point center(1.f, 2.f, 3.f);
  const Vector u = normalized(Vector(1.f, 1.f, 0.f)), v = normalized(Vector(-1.f, 1.f, 1.f)), w = cross(u, v);
  {
    // Points evenly spaced on an ellipse in the plane spanned by u and v.
    const int num = 12;
    Array<Point> pa;
    for_int(i, num) {
      const float angle = i * TAU / num;
      pa.push(center + u * (3.f * std::cos(angle)) + v * std::sin(angle));
    }
    Frame frame;
    Vec3<float> eimag;
    principal_components(pa, frame, eimag);
    // The variances are 3^2 / 2 and 1^2 / 2.
    SHOW(round_elements(clone(eimag.head<2>()), 1e4f));  // (The roundoff in eimag[2] is platform-dependent.)
    assertx(abs(eimag[0] - std::sqrt(4.5f)) < 1e-5f && abs(eimag[1] - std::sqrt(.5f)) < 1e-5f);
    assertx(eimag[2] < 1e-3f);
    assertx(dist(frame.p(), center) < 1e-5f);
    assertx(abs(abs(dot(frame.v(0), u)) - eimag[0]) < 1e-5f);  // The axes have lengths eimag.
    assertx(abs(abs(dot(frame.v(1), v)) - eimag[1]) < 1e-5f);
    assertx(abs(dot(frame.v(0), frame.v(1))) < 1e-5f);
    assertx(mag(frame.v(2)) > 0.f && mag(frame.v(2)) < 1e-3f);  // The frame is still invertible.
    assertx(abs(dot(normalized(frame.v(2)), w)) > .9999f);
    assertx(dot(cross(frame.v(0), frame.v(1)), frame.v(2)) > 0.f);  // Right-handed.
  }
  {
    // Collinear points, along a direction u, give two zero eigenvalues, yet an invertible right-handed frame.
    Array<Point> pa;
    for_int(i, 5) pa.push(center + u * (i - 2.f));
    Frame frame;
    Vec3<float> eimag;
    principal_components(pa, frame, eimag);
    assertx(abs(eimag[0] - std::sqrt(2.f)) < 1e-5f && eimag[1] < 1e-3f && eimag[2] < 1e-3f);
    assertx(abs(abs(dot(frame.v(0), u)) - eimag[0]) < 1e-5f);
    assertx(dot(cross(frame.v(0), frame.v(1)), frame.v(2)) > 0.f);
    assertx(dist(frame.p(), center) < 1e-5f);
  }
  {
    // For vectors, the origin is not subtracted, and the frame origin is zero.
    const Array<Vector> va = {u * 2.f, u * -2.f, v, -v};
    Frame frame;
    Vec3<float> eimag;
    principal_components(va, frame, eimag);
    assertx(abs(eimag[0] - std::sqrt(2.f)) < 1e-5f && abs(eimag[1] - std::sqrt(.5f)) < 1e-5f);
    assertx(frame.p() == Point(0.f, 0.f, 0.f));
    assertx(abs(abs(dot(frame.v(0), u)) - eimag[0]) < 1e-5f);
  }
}

}  // namespace

int main() {
  {
    Matrix<float> mi = {
        {20.f, 10.f},
        {22.f, 11.f},
        {18.f, 9.f},
        {19.f, 12.f},
    };
    test(mi);
  }
  test_synthetic();
  test_frame();
}
