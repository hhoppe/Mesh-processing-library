// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MatrixOp.h"

#include "libHh/Array.h"
#include "libHh/RangeOp.h"
#include "libHh/Vector4.h"
using namespace hh;

int main() {
  using Vec2f = Vec2<float>;
  struct A {
    A(int i = 0) : _i(i) { showf("A(%d)\n", _i); }
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
    // Array<unique_ptr<A>> ar2; ar2 = ar;  // compile error as expected
    ar.clear();
    SHOW(ar.num());
  }
  {
    SHOW("Matrix<unique_ptr>");
    Matrix<unique_ptr<A>> m(3, 2);
    SHOW("made");
    for_int(y, 3) for_int(x, 2) m[y][x] = make_unique<A>(y * 10 + x);
    SHOW("made some");
    m[1][0] = make_unique<A>(66);
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
    matrix1[1][0] = Vec2f(4.f, 3.f);
    Matrix<Vec2f> matrix2(V(3, 3), Vec2f(4.f, 3.f));
    matrix2[2][1] = Vec2f(5.f, 5.f);
    SHOW(matrix1);
    SHOW(matrix2);
    SHOW(matrix1 + matrix2);
  }
  {
    Matrix<float> matrix3(V(3, 3), 2.f);
    matrix3[1][0] = 4.f;
    Matrix<float> matrix4(V(3, 3), 5.f);
    matrix4[2][1] = 7.f;
    SHOW(matrix3);
    SHOW(matrix4);
    SHOW(matrix3 + matrix4);  // OPT1
    SHOW(matrix3);
    SHOW(matrix4);
    for (auto f : matrix4) SHOW(f);
    SHOW(2.f * matrix3);
    SHOW(2.f * matrix3 + 3.f * matrix4);
    SHOW(mag2(matrix3));
    SHOW(mat_mul(matrix3, matrix4));  // OPT2
  }
  {
    // Matrix4 m;
    Matrix<float> m(4, 4);
    Array<float> v1{1.f, 2.f, 3.f, 4.f};
    Array<float> v2{8.f, 7.f, 6.f, 5.f};
    if (0) {
      std::cerr << "uninitialized ";
      SHOW(m);
    }
    SHOW(v1);
    SHOW(v2);
    // I did not enable ArrayView::operator=()
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
    m[2][0] = 7;
    m[3][3] = 11;
    SHOW(m);
    SHOW(min(m));
    SHOW(max(m));
    SHOW(sum(m));
    SHOW(mean(m));
    SHOW(inverse(m));
    SHOW(mat_mul(m, inverse(m)));
    SHOW(mat_mul(mat_mul(m, m), inverse(m)));
  }
  {
    const int n = 8;
    Matrix<Vector4> m(V(n, n), Vector4(10.f));
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
    for_size_t(i, m.size()) m.flat(i) = int((i * 3371) % 577);
    assertx(dist2(rotate_ccw(m, 0), m) == 0);
    assertx(rotate_ccw(m, 90).dims() == V(5, 7));
    assertx(dist2(rotate_ccw(rotate_ccw(rotate_ccw(rotate_ccw(m, 90), 180), 270), 180), m) == 0);
  }
}

#if 0  // Matrix is a Grid, instanced in Grid_test
namespace hh {
template class CMatrixView<unsigned>;
template class CMatrixView<double>;
template class CMatrixView<const int*>;

template class MatrixView<unsigned>;
template class MatrixView<double>;
template class MatrixView<const int*>;

template class Matrix<unsigned>;
template class Matrix<double>;
template class Matrix<const int*>;

using U = unique_ptr<int>;
template <> void Matrix<U>::operator=(CMatrixView<U>) {}  // definition illegal
template class Matrix<U>;
}  // namespace hh
#endif
