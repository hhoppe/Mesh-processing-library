// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MatrixOp.h"

#include "libHh/MathOp.h"  // TAU
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Stat.h"
using namespace hh;

namespace {

float rounded(float v) { return round_fraction_digits(v, 1e4f); }

template <typename T> T max_abs_diff(CMatrixView<T> m1, CMatrixView<T> m2) {
  T v{0};
  for_int(y, m1.ysize()) for_int(x, m1.xsize()) v = max(v, abs(m1[y, x] - m2[y, x]));
  return v;
}

void test_invert() {
  {
    Matrix<float> m(3, 3);
    const Array<float> values{2.f, 0.f, 1.f, 1.f, 3.f, 2.f, 1.f, 1.f, 2.f};  // Its determinant is 6.
    for_int(y, 3) for_int(x, 3) m[y, x] = values[y * 3 + x];
    const Matrix<float> mi = inverse(m);
    for_int(y, 3) SHOW(rounded(mi[y, 0]), rounded(mi[y, 1]), rounded(mi[y, 2]));
    assertx(max_abs_diff<float>(mat_mul(m, mi), identity_mat<float>(3)) < 1e-5f);
    Matrix<float> m2(m);
    assertx(invert(m2, m2));  // In place.
    assertx(max_abs_diff<float>(m2, mi) < 1e-6f);
  }
  {
    Matrix<float> m(3, 3);
    const Array<float> values{1.f, 2.f, 3.f, 2.f, 4.f, 6.f, 0.f, 1.f, 1.f};  // Row 1 is twice row 0.
    for_int(y, 3) for_int(x, 3) m[y, x] = values[y * 3 + x];
    Matrix<float> mo(3, 3);
    SHOW(invert(m, mo));
  }
  {
    // A singular matrix whose elimination leaves a pivot that is small but (due to rounding) not exactly zero.
    Matrix<float> m(3, 3);
    const Array<float> values{2.f, 0.f, 1.f, 1.f, 3.f, 2.f, 1.f, 1.f, 1.f};
    for_int(y, 3) for_int(x, 3) m[y, x] = values[y * 3 + x];
    Matrix<float> mo(3, 3);
    SHOW(invert(m, mo));
  }
  {
    // Diagonally dominant (hence well-conditioned) random matrices must invert accurately.
    Random random(1);
    for (const int n : {1, 2, 5, 20}) {
      Matrix<double> m(n, n);
      for (double& e : m) e = random.dunif() - .5;
      for_int(i, n) m[i, i] += double(n);
      assertx(max_abs_diff<double>(mat_mul(m, inverse(m)), identity_mat<double>(n)) < 1e-12);
    }
  }
}

void test_outer_product_and_rotate() {
  const Matrix<int> m = outer_product<int>(V(1, 2, 3), V(10, 20));
  SHOW(m);
  for (const int rot : {0, 90, 180, 270, -90}) SHOW(rot, rotate_ccw<int>(m, rot));
  assertx(ranges::equal(rotate_ccw<int>(rotate_ccw<int>(m, 90), -90), m));
  assertx(ranges::equal(rotate_ccw<int>(rotate_ccw<int>(m, 180), 180), m));
}

void test_convolve() {
  const Array<float> ar{1.f, 2.f, 4.f, 8.f, 16.f};
  const Array<float> kernel{.25f, .5f, .25f};
  const float bordervalue = 0.f;
  for (const Bndrule bndrule :
       {Bndrule::reflected, Bndrule::periodic, Bndrule::clamped, Bndrule::border, Bndrule::reflected101}) {
    const float* pbordervalue = bndrule == Bndrule::border ? &bordervalue : nullptr;
    const Array<float> result = convolve<float, float>(ar, kernel, bndrule, pbordervalue);
    SHOW(boundaryrule_name(bndrule), result);
  }
  {
    // Convolving a centered delta function reproduces the kernel, flipped in both dimensions.
    Matrix<float> mat(V(5, 5), 0.f);
    mat[2, 2] = 1.f;
    Matrix<float> matk(3, 3);
    for_int(y, 3) for_int(x, 3) matk[y, x] = float(1 + y * 3 + x);
    const Matrix<float> result = convolve<float, float>(mat, matk, Bndrule::border, &bordervalue);
    SHOW(result);
    // Convolution with a kernel that sums to 1 preserves a constant image under every boundary rule.
    const Matrix<float> constant(V(4, 6), 3.f);
    const Matrix<float> box(V(3, 3), 1.f / 9.f);
    for (const Bndrule bndrule : {Bndrule::reflected, Bndrule::periodic, Bndrule::clamped}) {
      assertx(max_abs_diff<float>(convolve<float, float>(constant, box, bndrule), constant) < 1e-5f);
    }
  }
}

void test_transforms() {
  {
    const Frame rot = Frame::rotation(2, TAU / 4);  // 90 degrees about the z axis.
    SHOW(linear_transform(V(1.f, 0.f), rot), linear_transform(V(0.f, 1.f), rot));
    Frame frame = rot;
    frame.p() = Point(10.f, 20.f, 0.f);
    SHOW(affine_transform(V(1.f, 0.f), frame));
    // About the center of the unit square, the rotation maps a corner to another corner and fixes the center.
    SHOW(transform_about_center(V(0.f, 0.f), rot), transform_about_center(V(.5f, .5f), rot));
  }
  {
    // Conversion between a Frame and a 4x4 matrix.
    const Frame frame = Frame::rotation(0, .3f) * Frame::translation(V(1.f, 2.f, 3.f));
    const SGrid<float, 4, 4> m = to_Matrix(frame);
    const Vec4<float> last_column(m[0, 3], m[1, 3], m[2, 3], m[3, 3]);
    SHOW(last_column);
    assertx(to_Frame(m.grid_view()) == frame);
  }
}

void test_right_justify() {
  Matrix<int> m(3, 2);
  const Array<int> values{1, -200, 1000, 3, -5, 40};
  for_int(y, 3) for_int(x, 2) m[y, x] = values[y * 2 + x];
  const Matrix<string> s = right_justify<int>(m);
  for_int(y, 3) showf("[%s] [%s]\n", s[y, 0].c_str(), s[y, 1].c_str());
}

// Compare the Euclidean distance map with the exact (brute-force) one.
void test_euclidean_distance_map() {
  const int ny = 30, nx = 40;
  const Array<Vec2<int>> seeds{V(3, 5), V(17, 2), V(25, 31), V(8, 36), V(20, 20), V(29, 0)};
  Matrix<Vec2<int>> mvec(V(ny, nx), V(10'000, 10'000));
  for (const Vec2<int>& seed : seeds) mvec[seed] = V(0, 0);
  euclidean_distance_map<int>(mvec);
  int num_inexact = 0, max_excess = 0;
  for_int(y, ny) for_int(x, nx) {
    const Vec2<int> vec = mvec[y, x];
    assertx(contains(seeds, V(y, x) + vec));  // Each vector points to a seed.
    int min_d2 = std::numeric_limits<int>::max();
    for (const Vec2<int>& seed : seeds) min_d2 = min(min_d2, square(seed[0] - y) + square(seed[1] - x));
    const int d2 = square(vec[0]) + square(vec[1]);
    assertx(d2 >= min_d2);
    if (d2 > min_d2) num_inexact++, max_excess = max(max_excess, d2 - min_d2);
  }
  SHOW(ny * nx, num_inexact, max_excess);
}

}  // namespace

int main() {
  test_invert();
  test_outer_product_and_rotate();
  test_convolve();
  test_transforms();
  test_right_justify();
  test_euclidean_distance_map();
}
