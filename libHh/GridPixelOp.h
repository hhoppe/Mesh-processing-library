// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "MatrixOp.h"
#include "GridOp.h"

namespace hh {

// Testing:
// foreach n (1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 40 100 1000)
// echo $n; Filterimage -create $n $n -bound c -scaleu 1 -noo
// end

// Rescale matrix of pixels to the sizes given by destination nmatrixp; views matrixp and nmatrixp must be distinct.
inline void scale_Matrix_Pixel(CMatrixView<Pixel> matrixp, const Vec2<FilterBnd>& filterbs,
                               MatrixView<Pixel> nmatrixp) {
    static const int g_test_scale_accuracy = getenv_int("IMAGE_TEST_SCALE_ACCURACY"); // also in Image.cpp
    assertx(matrixp.data()!=nmatrixp.data());
    if (matrixp.dims()==nmatrixp.dims() && !g_test_scale_accuracy &&
        filterbs[0].filter().is_interpolating() && filterbs[1].filter().is_interpolating()) {
        nmatrixp.assign(matrixp);
        return;
    }
    if (filterbs[0].filter().is_trivial_magnify() && filterbs[1].filter().is_trivial_magnify() &&
        nmatrixp.dims()==matrixp.dims()*2) {
        const int cx = matrixp.xsize();
        parallel_for_int(y, nmatrixp.ysize()) {
            // for_int(x, nx) { nmatrixp(y, x) = matrixp(y/2, x/2); }
            Pixel* __restrict an = nmatrixp[y].data();
            const Pixel* __restrict ao = matrixp[y/2].data();
            for_int(x, cx) { *an++ = *ao; *an++ = *ao++; }
        }
        return;
    }
    if (filterbs[0].filter().is_trivial_minify() && filterbs[1].filter().is_trivial_minify() && nmatrixp.size()) {
        if (nmatrixp.dims()*2==matrixp.dims()) {
            constexpr int DS = 2, DS2 = DS*DS; // square(DS);
            const int nx = nmatrixp.xsize();
            parallel_for_int(y, nmatrixp.ysize()) {
                Pixel* __restrict an = nmatrixp[y].data();
                const Pixel* __restrict ao0 = matrixp[y*DS+0].data();
                const Pixel* __restrict ao1 = matrixp[y*DS+1].data();
                for_int(x, nx) {
                    for_int(z, 4) { an[0][z] = uchar((ao0[0][z]+ao0[1][z]+ao1[0][z]+ao1[1][z]+(DS2/2))/DS2); }
                    an += 1; ao0 += DS; ao1 += DS; // OPT:DS==2
                }
            }
            return;
        }
        if (nmatrixp.dims()*4==matrixp.dims()) {
            constexpr int DS = 4, DS2 = DS*DS; // square(DS)
            const int nx = nmatrixp.xsize();
            parallel_for_int(y, nmatrixp.ysize()) {
                Pixel* __restrict an = nmatrixp[y].data();
                const Pixel* __restrict ao0 = matrixp[y*DS+0].data();
                const Pixel* __restrict ao1 = matrixp[y*DS+1].data();
                const Pixel* __restrict ao2 = matrixp[y*DS+2].data();
                const Pixel* __restrict ao3 = matrixp[y*DS+3].data();
                for_int(x, nx) {
                    for_int(z, 4) {
                        an[0][z] = uchar((ao0[0][z]+ao0[1][z]+ao0[2][z]+ao0[3][z]+
                                          ao1[0][z]+ao1[1][z]+ao1[2][z]+ao1[3][z]+
                                          ao2[0][z]+ao2[1][z]+ao2[2][z]+ao2[3][z]+
                                          ao3[0][z]+ao3[1][z]+ao3[2][z]+ao3[3][z]+
                                          (DS2/2))/DS2);
                    }
                    an += 1; ao0 += DS; ao1 += DS; ao2 += DS; ao3 += DS; // OPT:DS==4
                }
            }
            return;
        }
        Vec2<int> Dyx = matrixp.dims()/nmatrixp.dims();
        if (nmatrixp.dims()*Dyx==matrixp.dims()) {
            const int Dyx2 = Dyx[0]*Dyx[1], nx = nmatrixp.xsize();
            parallel_for_int(y, nmatrixp.ysize()) {
                for_int(x, nx) {
                    Vec4<int> sums = ntimes<4>(Dyx2/2);
                    for_int(dy, Dyx[0]) {
                        auto matrixpy = matrixp[y*Dyx[0]+dy];
                        for_int(dx, Dyx[1]) {
                            for_int(z, 4) {
                                sums[z] += matrixpy[x*Dyx[1]+dx][z]; // OPT:Dyx
                            }
                        }
                    }
                    // for_int(z, 4) { nmatrixp[y][x][z] = uchar(sums[z]/Dyx2); }
                    auto& nmatrixpyx = nmatrixp[y][x]; for_int(z, 4) { nmatrixpyx[z] = uchar(sums[z]/Dyx2); }
                }
            }
            return;
        }
    }
    Matrix<Vector4> matrix(matrixp.dims()); convert(matrixp, matrix);
    matrix = scale(matrix, nmatrixp.dims(), filterbs, std::move(matrix));
    convert(matrix, nmatrixp);
}

// Rescale 3D grid of pixels spatially (only dimensions 1 and 2) to sizes given by destination ngridp;
//  views gridp and ngridp must be distinct.
// Was previously in GridOp.cpp
inline void spatially_scale_Grid3_Pixel(CGridView<3,Pixel> gridp, const Vec2<FilterBnd>& filterbs,
                                        GridView<3,Pixel> ngridp) {
    assertx(gridp.dim(0)==ngridp.dim(0)); // only scale dimensions 1 and 2
    assertx(gridp.dim(1)>0 && gridp.dim(2)>0 && ngridp.dim(1)>0 && ngridp.dim(2)>0);
    for_int(f, gridp.dim(0)) {
        scale_Matrix_Pixel(gridp[f], filterbs, ngridp[f]);
    }
}

} // namespace hh
