// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MatrixOp.h"

#include "libHh/Array.h"
#include "libHh/RangeOp.h"
#include "libHh/Vector4.h"
using namespace hh;

namespace {

// Matrices are equal if they have the same dimensions and elements.  (Grid has no operator==().)
template <typename T> bool same_matrix(CMatrixView<T> m1, CMatrixView<T> m2) {
  return m1.dims() == m2.dims() && ranges::equal(m1, m2);
}

// Matrix<T> is an alias of Grid<2, T>, with dimensions (ysize(), xsize()) and elements m[y, x].
void test_transpose() {
  Matrix<int> m(2, 3);
  assertx(m.ysize() == 2 && m.xsize() == 3 && m.dims() == V(2, 3) && m.size() == 6);
  for_int(y, 2) for_int(x, 3) m[y, x] = 10 * y + x;
  const Matrix<int> mt = transpose(m);
  SHOW(m);
  SHOW(mt);
  assertx(mt.dims() == V(3, 2));
  for_int(y, 2) for_int(x, 3) assertx(mt[x, y] == m[y, x]);
  assertx(same_matrix<int>(transpose(mt), m));  // Involution.
  // The transpose of a row vector is a column vector.
  const Matrix<int> mrow = {{1, 2, 3, 4}};
  const Matrix<int> mcol = transpose(mrow);
  assertx(mcol.dims() == V(4, 1) && mcol[3, 0] == 4);
  // The transpose of a matrix product is the product of the transposes in reverse order; with small integer
  // values, the float arithmetic is exact.
  Matrix<float> a(3, 4), b(4, 2);
  for_int(y, 3) for_int(x, 4) a[y, x] = float((y * 7 + x * 3) % 5 - 2);
  for_int(y, 4) for_int(x, 2) b[y, x] = float((y * 2 + x * 5) % 7 - 3);
  assertx(same_matrix<float>(transpose(mat_mul(a, b)), mat_mul(transpose(b), transpose(a))));
  // A row-vector-matrix product equals the transposed matrix times a column vector.
  const Array<float> v{1.f, -2.f, 3.f};
  assertx(mat_mul(v, a) == mat_mul(transpose(a), v));
  // A symmetric matrix equals its transpose.
  const Matrix<float> ata = mat_mul(transpose(a), a);
  assertx(same_matrix<float>(transpose(ata), ata));
  // The transpose of a view of some rows.
  const Matrix<int> mt_slice = transpose(m.slice(1, 2));
  assertx(mt_slice.dims() == V(3, 1) && mt_slice[2, 0] == 12);
  // An empty matrix.
  assertx(transpose(Matrix<int>(0, 3)).dims() == V(3, 0));
}

// Access outside the matrix using boundary rules.
void test_inside() {
  Matrix<int> m(2, 3);
  for_int(y, 2) for_int(x, 3) m[y, x] = 10 * y + x;
  const CMatrixView<int> mv = m;
  const auto bndrules = {std::pair{Bndrule::reflected, "reflected"}, std::pair{Bndrule::periodic, "periodic"},
                         std::pair{Bndrule::clamped, "clamped"}};
  for (const auto& [bndrule, name] : bndrules) {
    string s;
    for_intL(x, -4, 7) s += sform(" %d", mv.inside(-1, x, bndrule));
    showf("Row y=-1 for x=-4..6 with Bndrule::%s:%s\n", name, s.c_str());
    for_int(y, 2) for_int(x, 3) assertx(mv.inside(y, x, bndrule) == m[y, x]);
  }
  const int bordervalue = -1;
  assertx(mv.inside(0, 3, Bndrule::border, &bordervalue) == -1);
  assertx(mv.inside(-1, 0, Bndrule::border, &bordervalue) == -1);
  assertx(mv.inside(1, 2, Bndrule::border, &bordervalue) == 12);
  int y = 2, x = -1;
  assertx(!mv.map_inside(y, x, Bndrule::border));
  y = 2, x = -1;
  assertx(mv.map_inside(y, x, Bndrule::periodic) && y == 0 && x == 2);
  m.inside(5, -7, Bndrule::periodic) = 99;  // Modifiable element [1, 2].
  assertx(m[1, 2] == 99);
}

void test_reverse() {
  Matrix<int> m(3, 2);
  for_int(y, 3) for_int(x, 2) m[y, x] = 10 * y + x;
  Matrix<int> m2(m);
  m2.reverse_y();
  for_int(y, 3) for_int(x, 2) assertx(m2[y, x] == m[2 - y, x]);
  m2.reverse_x();
  for_int(y, 3) for_int(x, 2) assertx(m2[y, x] == m[2 - y, 1 - x]);  // Rotation by 180 degrees.
  assertx(same_matrix<int>(m2, rotate_ccw(m, 180)));
  // Reversing the rows of the transpose is a rotation by 90 degrees.
  Matrix<int> m3 = transpose(m);
  m3.reverse_y();
  assertx(same_matrix<int>(m3, rotate_ccw(m, 90)));
  SHOW(m3);
}

}  // namespace

int main() {
  using Vec2f = Vec2<float>;
  struct A {
    explicit A(int i = 0) : _i(i) { showf("A(%d)\n", _i); }
    ~A() { showf("~A(%d)\n", _i); }
    int _i;
  };
  {
    SHOW("Array<unique_ptr>");
    Array<unique_ptr<A>> ar(3);
    SHOW(ar.num());
    for_int(i, 3) ar[i] = make_unique<A>(i);
    SHOW("made");
    ar.push(make_unique<A>(3));
    SHOW(ar.num());
    ar.push(make_unique<A>());
    SHOW(ar.num());
    ar.shrink_to_fit();
    SHOW(ar.num());
    ar.resize(2);
    SHOW(ar.num());
    ar.shrink_to_fit();
    SHOW(ar.num());
    ar[0] = nullptr;
    SHOW("after 0 reset");
    // Array<unique_ptr<A>> ar2; ar2 = ar;  // Compile error as expected.
    ar.clear();
    SHOW(ar.num());
  }
  {
    SHOW("Matrix<unique_ptr>");
    Matrix<unique_ptr<A>> m(3, 2);
    SHOW("made");
    for_int(y, 3) for_int(x, 2) m[y, x] = make_unique<A>(y * 10 + x);
    SHOW("made some");
    m[1, 0] = make_unique<A>(66);
    SHOW("made one more");
    m.clear();
    SHOW("cleared");
  }
  {
    Array<float> array1(3, 3.f);
    array1[1] = 1.f;
    Array<float> array2(3, 5.f);
    array2[0] = 6.f;
    SHOW(array1);
    SHOW(array2);
    SHOW(array1 + array2);
    SHOW(array2 - array1);
    SHOW(array1 * array2);
    SHOW(dot(array1, array2));
    SHOW(mag2(array1));
    array2 += 10.f * array1;
    SHOW(array2);
    SHOW(min(array2));
    SHOW(max(array2));
    SHOW(sum(array2));
    SHOW(mean(array2));
  }
  {
    Matrix<Vec2f> matrix1(3, 3);
    fill(matrix1, Vec2f(1.f, BIGFLOAT));
    for_int(i, 3) fill(matrix1[i], Vec2f(1.f, 2.f));
    matrix1[1, 0] = Vec2f(4.f, 3.f);
    Matrix<Vec2f> matrix2(V(3, 3), Vec2f(4.f, 3.f));
    matrix2[2, 1] = Vec2f(5.f, 5.f);
    SHOW(matrix1);
    SHOW(matrix2);
    SHOW(matrix1 + matrix2);
  }
  {
    Matrix<float> matrix3(V(3, 3), 2.f);
    matrix3[1, 0] = 4.f;
    Matrix<float> matrix4(V(3, 3), 5.f);
    matrix4[2, 1] = 7.f;
    SHOW(matrix3);
    SHOW(matrix4);
    SHOW(matrix3 + matrix4);  // OPT:1
    SHOW(matrix3);
    SHOW(matrix4);
    for (auto f : matrix4) SHOW(f);
    SHOW(2.f * matrix3);
    SHOW(2.f * matrix3 + 3.f * matrix4);
    SHOW(mag2(matrix3));
    SHOW(mat_mul(matrix3, matrix4));  // OPT:2
  }
  {
    Matrix<float> m(4, 4);
    Array<float> v1{1.f, 2.f, 3.f, 4.f};
    Array<float> v2{8.f, 7.f, 6.f, 5.f};
    SHOW(v1);
    SHOW(v2);
    // ArrayView::operator=() is not enabled.
    // m[0] = m[2] = v1;
    // m[1] = m[3] = v2;
    m[0].assign(v1);
    m[2].assign(v1);
    m[1].assign(v2);
    m[3].assign(v2);
    SHOW(m);
    SHOW(v1);
    Array<float> v3a(v1);  // Array<float> v3a; v3a = v1;
    SHOW(v3a);
    Array<float> v3b;
    v3b = v1;
    SHOW(v3b);
    Array<float> v3(v1);
    SHOW(v3);
    Array<float> v4;
    v4 = m[0];
    SHOW(v4);
    SHOW(mat_mul(v3, m));
    v3 = mat_mul(v3, m);
    SHOW(v3);
    Array<float> v5(4);
    mat_mul(v1, m, v5);
    SHOW(v5);
    m[2, 0] = 7;
    m[3, 3] = 11;
    SHOW(m);
    SHOW(min(m));
    SHOW(max(m));
    SHOW(sum(m));
    SHOW(mean(m));
    {
      const Matrix<float> expected{
          {-1.f / 6.f, 0.f, 1.f / 6.f, 0.f},
          {-1.f / 3.f, 1.f / 6.f, -1.f / 3.f, 1.f / 6.f},
          {11.f / 18.f, 1.f / 9.f, 1.f / 6.f, -1.f / 3.f},
          {0.f, -1.f / 6.f, 0.f, 1.f / 6.f},
      };
      assertx(dist(inverse(m), expected) < 1e-6f);
    }
    assertx(dist(mat_mul(m, inverse(m)), identity_mat<float>(4)) < 1e-5f);
    {
      const Matrix<float> expected{
          {1.f, 2.f, 3.f, 4.f}, {8.f, 7.f, 6.f, 5.f}, {7.f, 2.f, 3.f, 4.f}, {8.f, 7.f, 6.f, 11.f}};
      assertx(dist(mat_mul(mat_mul(m, m), inverse(m)), expected) < 1e-4f);
    }
  }
  {
    const int n = 8;
    const Matrix<Vector4> m(V(n, n), Vector4(10.f));
    SHOW(mean(m));
    // Matrix<Vector4> mn = scale(m, 5, 5, twice(FilterBnd(Filter::get("impulse"), Bndrule::reflected)));
    Matrix<Vector4> mn = scale(m, V(5, 5), twice(FilterBnd(Filter::get("box"), Bndrule::periodic)));
    SHOW(mean(mn));
    mn = scale(mn, V(10, 10), twice(FilterBnd(Filter::get("impulse"), Bndrule::periodic)),
               implicit_cast<Vector4*>(nullptr), std::move(mn));
    SHOW(mean(mn));
  }
  {
    Matrix<int> m(V(7, 5));
    for (const size_t i : range(m.size())) m.flat(i) = int((i * 3371) % 577);
    assertx(dist2(rotate_ccw(m, 0), m) == 0);
    assertx(rotate_ccw(m, 90).dims() == V(5, 7));
    assertx(dist2(rotate_ccw(rotate_ccw(rotate_ccw(rotate_ccw(m, 90), 180), 270), 180), m) == 0);
  }
  {
    static_assert(ranges::view<CMatrixView<int>> && !ranges::view<Matrix<int>>);
  }
  test_transpose();
  test_inside();
  test_reverse();
}

// Matrix*<T> cannot be instanced because aliases for Grid*<2, T>, which however are instanced in Grid_test.cpp.
