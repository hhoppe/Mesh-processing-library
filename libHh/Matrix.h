// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MATRIX_H_
#define MESH_PROCESSING_LIBHH_MATRIX_H_

#include "Grid.h"

#if 0
{
  Matrix<float> mat(ysize, xsize);
  for_int(y, mat.ysize()) for_int(x, mat.xsize()) { float e = mat[y][x]; }
}
#endif

namespace hh {

// Classes to represent 2D matrices are now aliases of the general multidimensional grid classes.
// Note that these still include a few specialized member functions "For implementation of Matrix (D==2):".
template<typename T> using CMatrixView = CGridView<2,T>;
template<typename T> using MatrixView = GridView<2,T>;
template<typename T> using Matrix = Grid<2,T>;

template<typename T> Matrix<T> transpose(CMatrixView<T> m1) {
    Matrix<T> m(m1.xsize(), m1.ysize());
    for_int(y, m.ysize()) for_int(x, m.xsize()) { m[y][x] = m1[x][y]; }
    return m;
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_MATRIX_H_
