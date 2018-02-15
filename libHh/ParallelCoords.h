// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PARALLELCOORDS_H_
#define MESH_PROCESSING_LIBHH_PARALLELCOORDS_H_

#include "Parallel.h"
#include "Grid.h"               // grid_index()

namespace hh {

// *** Iterate over {1, 2, D}-dimensional grids, with/without parallelism,
//    with/without function optimized for grid interior.

template<typename Func = void(int), typename FuncInterior = void(int)>
void for_1DL_interior(int x0, int xn, Func func, FuncInterior func_interior) {
    { int x = x0; if (x0<xn) func(x); }
    for_intL(x, x0+1, xn-1) func_interior(x);
    { int x = max(x0+1, xn-1); if (x<xn) func(x); }
}

template<typename Func = void(int, int)> void for_2D(int yn, int xn, Func func) {
    for_int(y, yn) for_int(x, xn) func(y, x);
}

template<typename Func = void(int, int)> void for_2DL(int y0, int yn, int x0, int xn, Func func) {
    for_intL(y, y0, yn) for_intL(x, x0, xn) func(y, x);
}

template<typename Func = void(int, int)> void parallel_for_2D(int yn, int xn, Func func) {
    parallel_for_each(range(yn), [&](const int y) { for_int(x, xn) func(y, x); });
}

template<typename Func = void(int, int)> void parallel_for_2DL(int y0, int yn, int x0, int xn, Func func) {
    parallel_for_each(range(y0, yn), [&](const int y) { for_intL(x, x0, xn) func(y, x); });
}

template<typename Func = void(int, int), typename FuncInterior = void(int, int)>
void for_2DL_interior(int y0, int yn, int x0, int xn, Func func, FuncInterior func_interior) {
    { int y = y0; if (y<yn) for_intL(x, x0, xn) func(y, x); }
    for_intL(y, y0+1, yn-1) {
        { int x = x0; if (x0<xn) func(y, x); }
        for_intL(x, x0+1, xn-1) func_interior(y, x);
        { int x = max(x0+1, xn-1); if (x<xn) func(y, x); }
    }
    { int y = max(y0+1, yn-1); if (y<yn) for_intL(x, x0, xn) func(y, x); }
}

template<typename Func = void(int, int), typename FuncInterior = void(int, int)>
void parallel_for_2DL_interior(int y0, int yn, int x0, int xn, Func func, FuncInterior func_interior) {
    { int y = y0; if (y<yn) for_intL(x, x0, xn) func(y, x); }
    parallel_for_each(range(y0+1, yn-1), [&](const int y) {
        { int x = x0; if (x0<xn) func(y, x); }
        for_intL(x, x0+1, xn-1) func_interior(y, x);
        { int x = max(x0+1, xn-1); if (x<xn) func(y, x); }
    });
    { int y = max(y0+1, yn-1); if (y<yn) for_intL(x, x0, xn) func(y, x); }
}

// *** for_coords

template<int D, typename Func = void(const Vec<int,D>&)>
void for_coordsL(Vec<int,D> uL, Vec<int,D> uU, Func func) {
    for (const auto& u : range(uL, uU)) { func(u); }
}

template<typename Func = void(const Vec1<int>&)>
void for_coordsL(Vec1<int> uL, Vec1<int> uU, Func func) {
    for_intL(x, uL[0], uU[0]) { func(V(x)); }
}

template<typename Func = void(const Vec2<int>&)>
void for_coordsL(Vec2<int> uL, Vec2<int> uU, Func func) {
    Vec2<int> yx;
    for_intL(y, uL[0], uU[0]) { yx[0] = y; for_intL(x, uL[1], uU[1]) { yx[1] = x; func(yx); } }
}

template<typename Func = void(const Vec3<int>&)>
void for_coordsL(Vec3<int> uL, Vec3<int> uU, Func func) {
    Vec3<int> u;
    for_intL(z, uL[0], uU[0]) {
        u[0] = z; for_intL(y, uL[1], uU[1]) { u[1] = y; for_intL(x, uL[2], uU[2]) { u[2] = x; func(u); } }
    }
}

template<int D, typename Func = void(const Vec<int,D>&)> void for_coords(Vec<int,D> dims, Func func) {
    for_coordsL(ntimes<D>(0), dims, func);
}

template<int D, typename Func = void(const Vec<int,D>&)>
void parallel_for_coordsL(Vec<int,D> uL, Vec<int,D> uU, Func func,
                          uint64_t est_cycles_per_elem = k_omp_many_cycles_per_elem) {
    const Vec<int,D> dims = uU-uL;
    if (est_cycles_per_elem>=k_omp_many_cycles_per_elem) {
        parallel_for_each(range(size_t(product(dims))), [&](const size_t i) {
            func(uL+grid_index_inv(dims, i)); // most parallelism but with higher overhead
        });
    } else if (product(dims)*est_cycles_per_elem>=k_omp_thresh) {
        parallel_for_each(range(uL[0], uU[0]), [&](const int r) {
            for_coordsL(uL.with(0, r), uU.with(0, r+1), func); // parallelism in first dimension
        });
    } else {
        for_coordsL(uL, uU, func); // no parallelism
    }
}

template<typename Func = void(const Vec1<int>&)>
void parallel_for_coordsL(Vec1<int> uL, Vec1<int> uU, Func func,
                          uint64_t est_cycles_per_elem = k_omp_many_cycles_per_elem) {
    int dim0 = uU[0]-uL[0];
    if (dim0*est_cycles_per_elem>=k_omp_thresh) {
        parallel_for_each(range(uL[0], uU[0]), [&](const int i) { func(V(i)); });
    } else {
        for_coordsL(uL, uU, func); // no parallelism
    }
}

template<typename Func = void(const Vec2<int>&)>
void parallel_for_coordsL(Vec2<int> uL, Vec2<int> uU, Func func,
                          uint64_t est_cycles_per_elem = k_omp_many_cycles_per_elem) {
    const Vec2<int> dims = uU-uL;
    if (est_cycles_per_elem>=k_omp_many_cycles_per_elem) {
        parallel_for_each(range(size_t(product(dims))), [&](const size_t i) {
            func(uL+grid_index_inv(dims, i)); // most parallelism but with higher overhead
        });
    } else if (dims[0]>=k_omp_min_iterations && product(dims)*est_cycles_per_elem>=k_omp_thresh) {
        parallel_for_each(range(uL[0], uU[0]), [&](const int y) {
            Vec2<int> yx; yx[0] = y;
            for_intL(x, uL[1], uU[1]) { yx[1] = x; func(yx); } // efficient parallelism in first dimension
        });
    } else if (dims[1]*est_cycles_per_elem>=k_omp_thresh) {
        for_intL(y, uL[0], uU[0]) {
            parallel_for_each(range(uL[1], uU[1]), [&](const int x) { func(V(y, x)); });
        }
    } else {
        for_coordsL(uL, uU, func); // no parallelism
    }
}

template<typename Func = void(const Vec3<int>&)>
void parallel_for_coordsL(Vec3<int> uL, Vec3<int> uU, Func func,
                          uint64_t est_cycles_per_elem = k_omp_many_cycles_per_elem) {
    const Vec3<int> dims = uU-uL;
    if (est_cycles_per_elem>=k_omp_many_cycles_per_elem) {
        parallel_for_each(range(size_t(product(dims))), [&](const size_t i) {
            func(uL+grid_index_inv(dims, i)); // most parallelism but with higher overhead
        });
    } else if (dims[0]>=k_omp_min_iterations && product(dims)*est_cycles_per_elem>=k_omp_thresh) {
        parallel_for_each(range(uL[0], uU[0]), [&](const int z) {
            Vec3<int> u; u[0] = z;
            for_intL(y, uL[1], uU[1]) { u[1] = y; for_intL(x, uL[2], uU[2]) { u[2] = x; func(u); } }
        });
    } else if (dims[1]>=k_omp_min_iterations && product(dims.tail<2>())*est_cycles_per_elem>=k_omp_thresh) {
        for_intL(z, uL[0], uU[0]) {
            parallel_for_each(range(uL[1], uU[1]), [&](const int y) {
                Vec3<int> u(z, y, 0); for_intL(x, uL[2], uU[2]) { u[2] = x; func(u); }
            });
        }
    } else if (dims[2]*est_cycles_per_elem>=k_omp_thresh) {
        for_intL(z, uL[0], uU[0]) for_intL(y, uL[1], uU[1]) {
            parallel_for_each(range(uL[2], uU[2]), [&](const int x) { func(V(z, y, x)); });
        }
    } else {
        for_coordsL(uL, uU, func); // no parallelism
    }
}

template<int D, typename Func = void(const Vec<int,D>&)>
void parallel_for_coords(Vec<int,D> dims, Func func, uint64_t est_cycles_per_elem = k_omp_many_cycles_per_elem) {
    parallel_for_coordsL(ntimes<D>(0), dims, func, est_cycles_per_elem);
}

template<int D, typename Func = void(const Vec<int,D>&)>
void parallel_tiled_for_coordsL(Vec<int,D> uL, Vec<int,D> uU, Func func,
                                uint64_t est_cycles_per_elem = k_omp_many_cycles_per_elem) {
    if (product(uU-uL)*est_cycles_per_elem>=k_omp_thresh) {
        assertnever("TODO");
    } else {
        for_coordsL(uL, uU, func);
    }
}

template<int D, typename FuncRaster = void(size_t)>
void for_coordsL_raster(Vec<int,D> dims, Vec<int,D> uL, Vec<int,D> uU, FuncRaster func_raster) {
    if (min(uU-uL)<1) return; // to allow uL[c]==uU[c]==dims[c]
    ASSERTX(uL.in_range(dims) && uU.in_range(dims+1));
    if (D==1) {
        for_intL(d0, uL[0], uU[0]) { func_raster(d0); }
    } else if (D==2) {
        for_intL(d0, uL[0], uU[0]) {
            size_t i0 = d0*dims[1];
            for_intL(d1, uL[1], uU[1]) { func_raster(i0+d1); }
        }
    } else if (D==3) {          // speed does not increase much
        for_intL(d0, uL[0], uU[0]) for_intL(d1, uL[1], uU[1]) {
            // "size_t ib = grid_index_list(dims, d0, d1, uL[2]);" fails for D!=3
            // "size_t ib = grid_index<D>(dims, {d0, d1, uL[2]});" fails for D!=3
            // size_t ib = grid_index<D>(dims, V(d0, d1, uL[2]).view()); // .view() for compilation of D!=3
            // for_int(id, uU[2]-uL[2]) { func_raster(ib+id); }
            size_t i0 = (d0*dims[1]+d1)*dims[2];
            for_intL(d2, uL[2], uU[2]) { func_raster(i0+d2); }
        }
    } else {                    // this generic case is already quite fast
        for (const auto& u : range(uL, uU)) { func_raster(grid_index(dims, u)); }
    }
}

template<int D, typename Func = void(const Vec<int,D>&), typename FuncInterior = void(size_t)>
void for_coordsL_interior(Vec<int,D> dims, Vec<int,D> uL, Vec<int,D> uU,
                          Func func, FuncInterior func_interior) {
    if (min(uU-uL)<1) return; // to allow uL[c]==uU[c]==dims[c]
    ASSERTX(uL.in_range(dims) && uU.in_range(dims+1));
    // Note: not the same traversal order as for_coordsL() !
    for_int(c, D) {
        if (uL[c]==0       && uL[c]<uU[c]) { for_coordsL(uL, uU.with(c, 1),         func); uL[c] = 1; }
        if (uU[c]==dims[c] && uL[c]<uU[c]) { for_coordsL(uL.with(c, dims[c]-1), uU, func); uU[c] = dims[c]-1; }
    }
    for_coordsL_raster(dims, uL, uU, func_interior);
}

template<int D, typename Func = void(const Vec<int,D>&), typename FuncInterior = void(size_t)>
void parallel_d0_for_coordsL_interior(Vec<int,D> dims, Vec<int,D> uL, Vec<int,D> uU,
                                      Func func, FuncInterior func_interior) {
    if (min(uU-uL)<1) return; // to allow uL[c]==uU[c]==dims[c]
    ASSERTX(uL.in_range(dims) && uU.in_range(dims+1));
    int nthreads = get_max_threads(), ny = uU[0]-uL[0], ychunk = (ny-1)/nthreads+1;
    nthreads = (ny+ychunk-1)/ychunk;
    parallel_for_each(range(nthreads), [&](const int thread) {
        for_coordsL_interior(dims,
                             uL.with(0, uL[0]+thread*ychunk),
                             uU.with(0, min(uL[0]+(thread+1)*ychunk, uU[0])),
                             func, func_interior);
    });
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_PARALLELCOORDS_H_
