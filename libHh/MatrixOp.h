// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MATRIXOP_H_
#define MESH_PROCESSING_LIBHH_MATRIXOP_H_

#include "Matrix.h"
#include "GridOp.h"
#include "Geometry.h"           // Frame
#include "SGrid.h"
#include "MathOp.h"             // my_mod()

namespace hh {

// Create a view (CStridedArrayView) onto the column x of matrix mat.
template<typename T> CStridedArrayView<T> column(CMatrixView<T> mat, int x) {
    return grid_column<0>(mat, V(0, x));
}

// Create a view (StridedArrayView) onto the column x of matrix mat.
template<typename T> StridedArrayView<T> column(MatrixView<T> mat, int x) {
    return grid_column<0>(mat, V(0, x));
}

// ret: success
template<typename T> bool invert(CMatrixView<T> mi, MatrixView<T> mo) {
    static_assert(std::is_floating_point<T>::value, "");
    // mi.data()==mo.data() is OK
    const int n = mi.ysize(); assertx(n && mi.xsize()==n); assertx(same_size(mi, mo));
    Matrix<T> t(n, 2*n);
    for_int(i, n) {
        for_int(j, n) {
            t[i][j] = mi[i][j];
            t[i][n+j] = T(0);
        }
    }
    for_int(i, n) { t[i][n+i] = T(1); }
    for_int(i, n) {
        if (i<n-1) {            // swap row with largest front coefficient
            T a = abs(t[i][i]), max_a;
            int max_i = i;
            for_intL(l, i+1, n) {
                if ((max_a = abs(t[l][i]))>a) { a = max_a; max_i = l; }
            }
            if (max_i!=i) swap_ranges(t[i], t[max_i]);
        }
        if (!t[i][i]) return false;
        // Note: bug in VS2015 update 1 - x64 DebugMD due to OpenMP; if parallel, never terminates.
        cond_parallel_for_int(mi.size()*2, j, n) {
            if (j==i) continue; // must be done outside the parallel loop
            T a = -t[j][i]/t[i][i];
            for_int(k, 2*n) { t[j][k] += a*t[i][k]; }
        }
        if (1) {
            int j = i;
            T a = T(1)/t[i][i];
            for_int(k, 2*n) { t[j][k] *= a; }
        }
    }
    for_int(i, n) for_int(j, n) { mo[i][j] = t[i][n+j]; }
    return true;
}

// asserts it is invertible
template<typename T> Matrix<T> inverse(CMatrixView<T> mi) {
    Matrix<T> m(mi.dims()); assertx(invert(mi, m)); return m;
}

// Multiply matrix m1 by matrix m2 and store the result into matrix mo:  mo = m1 * m2.
template<typename T> void mat_mul(CMatrixView<T> m1, CMatrixView<T> m2, MatrixView<T> mo) {
    assertx(m1.ysize() && m1.xsize() && m1.xsize()==m2.ysize() && m2.xsize());
    cond_parallel_for_int(m1.size()*m2.xsize()*1, i, m1.ysize()) {
        // Note: for faster memory performance, it would be best to swap the j and k loops;
        //  however this would require a temporary double[m2.xsize()] buffer allocated per thread.
        for_int(j, m2.xsize()) {
            double sum = 0.; for_int(k, m1.xsize()) { sum += m1[i][k]*m2[k][j]; }
            mo[i][j] = T(sum);
        }
    }
}

// Multiply matrix m1 by matrix m2 and return the result m1 * m2.
template<typename T> Matrix<T> mat_mul(CMatrixView<T> m1, CMatrixView<T> m2) {
    Matrix<T> mo(m1.ysize(), m2.xsize()); mat_mul(m1, m2, mo); return mo;
}

// Multiply matrix m by column vector vi and store the result into column vector vo:  vo = m * vi.
template<typename T> void mat_mul(CMatrixView<T> m, CArrayView<T> vi, ArrayView<T> vo) {
    assertx(m.ysize() && m.xsize() && m.xsize()==vi.num() && m.ysize()==vo.num());
    assertx(!have_overlap(vi, vo));
    cond_parallel_for_int(m.size()*1, i, m.ysize()) {
        double sum = 0.; for_int(j, m.xsize()) { sum += m[i][j]*vi[j]; }
        vo[i] = T(sum);
    }
}

// Multiply matrix m by column vector vi and return the resulting column vector m * vi.
template<typename T> Array<T> mat_mul(CMatrixView<T> m, CArrayView<T> vi) {
    Array<T> vo(m.ysize()); mat_mul(m, vi, vo); return vo;
}

// Multiply row vector vi by matrix m and store the result into row vector vo:  vo = vi * m.
template<typename T> void mat_mul(CArrayView<T> vi, CMatrixView<T> m, ArrayView<T> vo) {
    assertx(m.ysize() && m.xsize() && m.ysize()==vi.num() && m.xsize()==vo.num());
    assertx(!have_overlap(vi, vo));
    // Note: for faster memory performance, it would be best to swap the j and i loops;
    //  however this would require a temporary double[m.xsize()] buffer allocated per thread.
    cond_parallel_for_int(m.size()*1, j, m.xsize()) {
        double sum = 0.; for_int(i, m.ysize()) { sum += vi[i]*m[i][j]; }
        vo[j] = T(sum);
    }
}

// Multiply row vector vi by matrix m and return the resulting row vector m * vi.
template<typename T> Array<T> mat_mul(CArrayView<T> vi, CMatrixView<T> m) {
    Array<T> vo(m.xsize()); mat_mul(vi, m, vo); return vo;
}

// Modify matrix mat to be the diagonal matrix whose elements are given by vector v.
template<typename T> void diag_mat(CArrayView<T> v, MatrixView<T> mat) {
    fill(mat, T(0));
    for_int(i, v.num()) { mat[i][i] = v[i]; }
}

// Return diagonal matrix whose elements are given by vector v.
template<typename T> Matrix<T> diag_mat(CArrayView<T> v) {
    Matrix<T> mat(twice(v.num())); diag_mat(v, mat); return mat;
}

// Modify matrix mat to be an identity matrix.
template<typename T> void identity_mat(MatrixView<T> mat) {
    fill(mat, T(0));
    for_int(i, min(mat.dims())) { mat[i][i] = T(1); }
}

// Return the identity matrix with dimensions dims (yx).
template<typename T> Matrix<T> identity_mat(const Vec2<int>& dims) {
    Matrix<T> mat(dims); identity_mat(mat); return mat;
}

// Return the square identity matrix with dimensions twice(n).
template<typename T> Matrix<T> identity_mat(int n) {
    return identity_mat<T>(twice(n));
}

// Convert an affine 4x3 matrix to a 4x4 Matrix.
inline SGrid<float, 4, 4> to_Matrix(const Frame& f) {
    return V(V(f[0][0], f[0][1], f[0][2], 0.f),
             V(f[1][0], f[1][1], f[1][2], 0.f),
             V(f[2][0], f[2][1], f[2][2], 0.f),
             V(f[3][0], f[3][1], f[3][2], 1.f));
}

// Convert a 4x4 Matrix to an affine 4x3 matrix.
inline Frame to_Frame(CMatrixView<float> m) {
    assertx(m.ysize()==4 && m.xsize()==4);
    Frame f;
    const float tolerance = 1e-5f;
    if (abs(m[0][3])>tolerance || abs(m[1][3])>tolerance || abs(m[2][3])>tolerance || abs(m[3][3]-1.f)>tolerance) {
        if (Warning("Frame matrix strange")) SHOW(m);
    }
    for_int(i, 4) for_int(j, 3) f[i][j] = m[i][j];
    return f;
}

// Compute a new matrix nm by transforming input matrix m according to frame,
//  where frame maps from destination pixel (y, x, 0) to source pixel (y, x, 0);
//  both have (possibly rectangular) domain [-0.5, +0.5]^2.
template<typename T> void transform(CMatrixView<T> m, const Frame& frame, const Vec2<FilterBnd>& filterbs,
                                    MatrixView<T> nm) {
    if (0 && max(mag(frame.v(0)), mag(frame.v(1)))>1.3f)
        Warning("Image transform: minification could result in aliasing"); // fails to account for unequal image sizes
    Vec2<FilterBnd> tfilterbs = filterbs;
    CMatrixView<T> mr(m);
    Matrix<T> tm;
    if (filterbs[0].filter().has_inv_convolution() || filterbs[1].filter().has_inv_convolution()) {
        tm = m;
        tfilterbs = inverse_convolution(tm, filterbs);
        mr.reinit(tm);
    }
    cond_parallel_for_int(nm.size()*500, y, nm.ysize()) for_int(x, nm.xsize()) {
        Point p((y+0.5f)/nm.ysize(), (x+0.5f)/nm.xsize(), 0.f); // y, x, 0
        bool center = true;
        if (center) p = p-Vector(0.5f, 0.5f, 0.f);
        Point tp = p*frame;     // y, x, 0
        if (center) tp = tp+Vector(0.5f, 0.5f, 0.f);
        nm[y][x] = sample_domain(mr, V(tp[0], tp[1]), tfilterbs);
    }
}

// Input: mvec[y][x].mag() is large except near seedpoints where it should indicate relative location of seedpoints.
// Output: vectors indicating relative location of nearest seedpoint.  e.g. T = int or float
template<typename T> void euclidean_distance_map(MatrixView<Vec2<T>> mvec);

// Compute the matrix which is the outer product of two vectors (ar1 is column vector, ar2 is row vector).
template<typename T> Matrix<T> outer_product(CArrayView<T> ar1, CArrayView<T> ar2) {
    Matrix<T> mat(ar1.num(), ar2.num());
    for_int(y, mat.ysize()) for_int(x, mat.xsize()) { mat[y][x] = ar1[y]*ar2[x]; }
    return mat;
}

// Convolve array ar with convolution kernel ark; result has the same size and type as ar.
template<typename T, typename TK> Array<T> convolve(CArrayView<T> ar, CArrayView<TK> ark,
                                                    Bndrule bndrule, const T* bordervalue = nullptr) {
    static_assert(std::is_floating_point<TK>::value, "Kernel array must contain float/double");
    using Precise = sum_type_t<T>;
    assertx(ark.num()%2==1);
    const int xxm = ark.num()/2;
    Array<T> nar(ar.num());
    cond_parallel_for_int(ar.num()*ark.num()*2, x, ar.num()) {
        Precise v(0);
        for_int(xx, ark.num()) { v += ark[xx]*Precise(ar.inside(x-xxm+xx, bndrule, bordervalue)); }
        nar[x] = T(v);
    }
    return nar;
}

// Convolve matrix mat with convolution kernel matk; result has the same size and type as mat.
template<typename T, typename TK> Matrix<T> convolve(CMatrixView<T> mat, CMatrixView<TK> matk,
                                                     Bndrule bndrule, const T* bordervalue = nullptr) {
    static_assert(std::is_floating_point<TK>::value, "Kernel matrix must contain float/double");
    using Precise = sum_type_t<T>;
    assertx(matk.ysize()%2==1 && matk.xsize()%2==1);
    const Vec2<int> pm = matk.dims()/2;
    Matrix<T> nmat(mat.dims());
    cond_parallel_for_int(mat.size()*matk.size()*2, y, mat.ysize()) for_int(x, mat.xsize()) {
        Precise v(0);
        for_int(yy, matk.ysize()) for_int(xx, matk.xsize()) {
            v += matk[yy][xx]*Precise(mat.inside(y-pm[0]+yy, x-pm[1]+xx, bndrule, bordervalue));
        }
        nmat[y][x] = T(v);
    }
    return nmat;
}

// Convert entries of matrix mat into right-justified strings (with equal lengths per column).
template<typename T> Matrix<string> right_justify(CMatrixView<T> mat) {
    Matrix<string> nmat(mat.dims());
    for_int(y, mat.ysize()) for_int(x, mat.xsize()) { nmat[y][x] = make_string(mat[y][x]); }
    for_int(x, nmat.xsize()) {
        int maxlen = 0; for_int(y, nmat.ysize()) { maxlen = max(maxlen, int(nmat[y][x].size())); }
        for_int(y, nmat.ysize()) {
            while (int(nmat[y][x].size())<maxlen) nmat[y][x] = " " + nmat[y][x]; // slow but easy
        }
    }
    return nmat;
}

// Rotate counter-clockwise by an angle which is a multiple of 90 degrees (e.g. -90, 0, +90, +180, +270).
template<typename T> void rotate_ccw(CMatrixView<T> mat, int rot_degrees, MatrixView<T> nmat) {
    assertx(my_mod(rot_degrees, 90)==0);
    assertx(nmat.dims() == (my_mod(rot_degrees, 180)==0 ? mat.dims() : mat.dims().rev()));
    switch (my_mod(rot_degrees, 360)) {
     bcase 0:
        nmat.assign(mat);
     bcase 90:
        for_int(y, nmat.ysize()) for_int(x, nmat.xsize()) { nmat(y, x) = mat(x, nmat.ysize()-1-y); }
     bcase 270:
        for_int(y, nmat.ysize()) for_int(x, nmat.xsize()) { nmat(y, x) = mat(nmat.xsize()-1-x, y); }
     bcase 180:
        for_int(y, nmat.ysize()) for_int(x, nmat.xsize()) { nmat(y, x) = mat(nmat.ysize()-1-y, nmat.xsize()-1-x); }
     bdefault: assertnever("");
    }
}

// Rotate counter-clockwise by an angle which is a multiple of 90 degrees (e.g. -90, 0, +90, +180, +270).
template<typename T> Matrix<T> rotate_ccw(CMatrixView<T> mat, int rot_degrees) {
    assertx(my_mod(rot_degrees, 90)==0);
    Matrix<T> nmat(my_mod(rot_degrees, 180)==0 ? mat.dims() : mat.dims().rev());
    rotate_ccw(mat, rot_degrees, nmat);
    return nmat;
}


//----------------------------------------------------------------------------

// T may be any signed arithmetic type.
template<typename T> void euclidean_distance_map(MatrixView<Vec2<T>> mvec) {
    // Compute Euclidean distance map.
    // Input: 2D vectors contain large magnitudes except at or near desired feature points.
    // Output: 2D vectors point to closest feature point.
    // T may be integer, or may be float for sub-pixel feature positions (as in Filtervideo.cpp).
    //  See original work:
    //   Per-Erik Danielsson.  Euclidean Distance Mapping.
    //    Computer Graphics and image Processing 14:227-248, 1980.
    //  And description in:
    //   F.S. Nooruddin and Greg Turk.  Simplification and repair of
    //    polygonal models using volumetric techniques.
    //    Georgia Tech TR GIT-GVU-99-37, 1999.
    // See also version in HTest.cpp function do_lloyd_relax() which supports spatially non-uniform metric.
    // Lower precision than RangeOp.h template mag2<>
    auto lmag2 = [](const Vec2<T>& v) { return square(v[0])+square(v[1]); };
    for_intL(y, 1, mvec.ysize()) {
        for_int(x, mvec.xsize()) {
            auto vt = mvec[y-1][x]+V(T(-1), T(0)); if (lmag2(vt)<lmag2(mvec[y][x])) mvec[y][x] = vt;
        }
        for_intL(x, 1, mvec.xsize()) {
            auto vt = mvec[y][x-1]+V(T(0), T(-1)); if (lmag2(vt)<lmag2(mvec[y][x])) mvec[y][x] = vt;
        }
        for (int x = mvec.xsize()-2; x>=0; --x) {
            auto vt = mvec[y][x+1]+V(T(0), T(+1)); if (lmag2(vt)<lmag2(mvec[y][x])) mvec[y][x] = vt;
        }
    }
    for (int y = mvec.ysize()-2; y>=0; --y) {
        for_int(x, mvec.xsize()) {
            auto vt = mvec[y+1][x]+V(T(+1), T(0)); if (lmag2(vt)<lmag2(mvec[y][x])) mvec[y][x] = vt;
        }
        for_intL(x, 1, mvec.xsize()) {
            auto vt = mvec[y][x-1]+V(T(0), T(-1)); if (lmag2(vt)<lmag2(mvec[y][x])) mvec[y][x] = vt;
        }
        for (int x = mvec.xsize()-2; x>=0; --x) {
            auto vt = mvec[y][x+1]+V(T(0), T(+1)); if (lmag2(vt)<lmag2(mvec[y][x])) mvec[y][x] = vt;
        }
    }
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_MATRIXOP_H_
