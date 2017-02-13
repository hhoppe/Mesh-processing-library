// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PRINCIPAL_H_
#define MESH_PROCESSING_LIBHH_PRINCIPAL_H_

#include "Geometry.h"
#include "Matrix.h"

namespace hh {

// Given points pa[], compute the principal component frame f and the (redundant) eigenvalues eimag.
// The frame f is guaranteed to be invertible and orthogonal, as the axis
//   will always have non-zero (albeit very small) lengths.
// The values eimag will be zero for the axes that should be zero.
// The frame f is also guaranteed to be right-handed.
void principal_components(CArrayView<Point> pa, Frame& f, Vec3<float>& eimag);

// Same but for vectors va[].  Note that the origin f.p() of frame f will therefore be thrice(0.f).
void principal_components(CArrayView<Vector> va, Frame& f, Vec3<float>& eimag);

// Given mi[m][n] (m data points of dimension n),
//   compute mo[n][n] (n orthonormal eigenvectors rows, by decreasing eigenv.) and eigenvalues eimag[n].
// Note that the mean must be subtracted out of mi[][] if desired.
// As shown in tPrincipal.cpp test_sgesvd(), this is related to computing singular value decomposition
//   using sgesdv_().
void principal_components(CMatrixView<float> mi, MatrixView<float> mo, ArrayView<float> eimag);

// Same using incremental approximate computation.
// mo.ysize() specifies the desired number of largest eigenvectors.
// Note that the mean must be subtracted out of mi[][] if desired.
void incr_principal_components(CMatrixView<float> mi, MatrixView<float> mo, ArrayView<float> eimag, int niter);

// Same using expectation maximization.
// mo.ysize() specifies the desired number of largest eigenvectors.  niter~10-50.
// Note that the mean must be subtracted out of mi[][] if desired.
bool em_principal_components(CMatrixView<float> mi, MatrixView<float> mo, ArrayView<float> eimag, int niter);

// Subtract out the mean of the rows.
void subtract_mean(MatrixView<float> mi);

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_PRINCIPAL_H_
