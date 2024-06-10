// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_GRIDOP_H_
#define MESH_PROCESSING_LIBHH_GRIDOP_H_

#include "libHh/Filter.h"
#include "libHh/Grid.h"
#include "libHh/PArray.h"
#include "libHh/ParallelCoords.h"
#include "libHh/Pixel.h"
#include "libHh/StridedArrayView.h"
#include "libHh/Vector4.h"
#include "libHh/Vector4i.h"

#if 0
#include "libHh/Timer.h"
#define HH_GRIDOP_TIMER(name) HH_TIMER(name)
#endif

#if !defined(HH_GRIDOP_TIMER)
#define HH_GRIDOP_TIMER(name)
#endif

namespace hh {

// Create a view (CStridedArrayView) onto the column of the grid (along dimension COL_D) starting at position u0.
template <int COL_D, int D, typename T> CStridedArrayView<T> grid_column(CGridView<D, T> grid, const Vec<int, D>& u0);

// Create a view (StridedArrayView) onto the column of the grid (along dimension COL_D) starting at position u0.
template <int COL_D, int D, typename T> StridedArrayView<T> grid_column(GridView<D, T> grid, const Vec<int, D>& u0);

// Create a view (CStridedArrayView) onto the column of the grid (along dimension col_d) starting at position u0.
template <int D, typename T> CStridedArrayView<T> grid_column(CGridView<D, T> grid, int col_d, const Vec<int, D>& u0);

// Create a view (StridedArrayView) onto the column of the grid (along dimension col_d) starting at position u0.
template <int D, typename T> StridedArrayView<T> grid_column(GridView<D, T> grid, int col_d, const Vec<int, D>& u0);

// Crop the sides of a D-dimensional grid (uL/uU for lower/upper extents); any negative crop grows the grid
//  and requires the definition of boundary rules (and border value if Bndrule::border).
template <int D, typename T>
Grid<D, T> crop(CGridView<D, T> grid, const Vec<int, D>& uL, const Vec<int, D>& uU,
                Vec<Bndrule, D> bndrules = ntimes<D>(Bndrule::undefined), const T* bordervalue = nullptr);

enum class Alignment { left, center, right };

// Merge a D-dimensional grid of D-dimensional grids into a single D-dimensional grid.
//  (U must be derived from CGridView<D, T>).
template <int D, typename U, typename T = typename U::value_type>
Grid<D, T> assemble(CGridView<D, U> grids, const T& background = T{},
                    const Vec<Alignment, D>& align = ntimes<D>(Alignment::center));

// Recast a grid view as a next-higher-dimensional grid view with just a single slice in dimension zero.
template <int D, typename T> CGridView<D + 1, T> raise_grid_rank(CGridView<D, T> grid);

// Convert a grid of uint8_t pixel values into float values.
template <int D> void convert(CGridView<D, Pixel> gridu, GridView<D, Vector4> gridf);

// Convert a grid of float pixel values into uint8_t pixel values.
template <int D> void convert(CGridView<D, Vector4> gridf, GridView<D, Pixel> gridu);

// Convert a grid of uint8_t values into float values.
template <int D> void convert(CGridView<D, uint8_t> gridu, GridView<D, float> gridf);

// Convert a grid of float values into uint8_t values.
template <int D> void convert(CGridView<D, float> gridf, GridView<D, uint8_t> gridu);

// Convert a grid of Vec2<uint8_t> values into Vector4 values.
template <int D> void convert(CGridView<D, Vec2<uint8_t>> gridu, GridView<D, Vector4> gridf);

// Convert a grid of Vector4 values into Vec2<uint8_t> values.
template <int D> void convert(CGridView<D, Vector4> gridf, GridView<D, Vec2<uint8_t>> gridu);

// Rescale a grid to have new dimensions ndims (e.g., V(ny, nx) in 2D); assumes samples of old and new grids
//  lie at locations [.5 / dim(0)...(dim(0) - .5) / dim(0)]...[.5 / dim(D - 1)...(dim(D - 1) - .5) / dim(D - 1)].
template <int D, typename T>
Grid<D, T> scale(CGridView<D, T> grid, const Vec<int, D>& ndims, const Vec<FilterBnd, D>& filterbs,
                 const T* bordervalue = nullptr, Grid<D, T>&& gr = Grid<D, T>());

// Same as scale() but assumes that samples of old and new grids
//  lie at locations [0., 1. / (dim(0) - 1), ..., (dim(0) - 1) / (dim(0) - 1) == 1.] on each dimension.
template <int D, typename T>
Grid<D, T> scale_primal(CGridView<D, T> grid, const Vec<int, D>& ndims, const Vec<FilterBnd, D>& filterbs,
                        const T* bordervalue = nullptr, Grid<D, T>&& gr = Grid<D, T>());

// Rescale a grid to have new dimensions ndims, specialized to nearest sampling; works on arbitrary types T,
//  even those that are non-interpolable.
template <int D, typename T>
Grid<D, T> scale_filter_nearest(CGridView<D, T> grid, const Vec<int, D>& ndims, Grid<D, T>&& gr = Grid<D, T>());

// Apply inverse convolution filter preprocess (spline | omoms) to grid; return new filter (justspline | justomoms).
template <int D, typename T>
Vec<FilterBnd, D> inverse_convolution(GridView<D, T> grid, const Vec<FilterBnd, D>& filterbs);

// Evaluate the filter at point p of grid g, where samples lie at integers ([0, dim(0) - 1], ..., [0, dim(D - 1) - 1]).
template <int D, typename T>
T sample_grid(CGridView<D, T> g, const Vec<float, D>& p, const Vec<FilterBnd, D>& filterbs,
              const T* bordervalue = nullptr);

// Evaluate the filter at point p of grid g, where grid domain is [0, 1]^D, and thus grid samples lie at
// locations [.5 / dim(0) ... (dim(0) - .5) / dim(0)] ... [.5 / dim(D - 1) ... (dim(D - 1) - .5) / dim(D - 1)].
template <int D, typename T>
T sample_domain(CGridView<D, T> g, const Vec<float, D>& p, const Vec<FilterBnd, D>& filterbs,
                const T* bordervalue = nullptr);

// Convolve a Pixel grid along its d'th dimension with the 1D kernel (which must have odd length).
template <int D, bool parallel = true>
Grid<D, Pixel> convolve_d(CGridView<D, Pixel> grid, int d, CArrayView<float> kernel, Bndrule bndrule,
                          const Pixel* bordervalue = nullptr);

//----------------------------------------------------------------------------

template <int COL_D, int D, typename T> CStridedArrayView<T> grid_column(CGridView<D, T> grid, const Vec<int, D>& u0) {
  static_assert(COL_D >= 0 && COL_D < D);
  ASSERTX(grid.ok(u0));
  ASSERTX(u0[COL_D] == 0);
  return CStridedArrayView<T>(&grid[u0], grid.dim(COL_D), grid_stride(grid.dims(), COL_D));
}

template <int COL_D, int D, typename T> StridedArrayView<T> grid_column(GridView<D, T> grid, const Vec<int, D>& u0) {
  static_assert(COL_D >= 0 && COL_D < D);
  ASSERTX(grid.ok(u0));
  ASSERTX(u0[COL_D] == 0);
  return StridedArrayView<T>(&grid[u0], grid.dim(COL_D), grid_stride(grid.dims(), COL_D));
}

template <int D, typename T> CStridedArrayView<T> grid_column(CGridView<D, T> grid, int col_d, const Vec<int, D>& u0) {
  ASSERTX(col_d >= 0 && col_d < D);
  ASSERTX(grid.ok(u0));
  ASSERTX(u0[col_d] == 0);
  return CStridedArrayView<T>(&grid[u0], grid.dim(col_d), grid_stride(grid.dims(), col_d));
}

template <int D, typename T> StridedArrayView<T> grid_column(GridView<D, T> grid, int col_d, const Vec<int, D>& u0) {
  ASSERTX(col_d >= 0 && col_d < D);
  ASSERTX(grid.ok(u0));
  ASSERTX(u0[col_d] == 0);
  return StridedArrayView<T>(&grid[u0], grid.dim(col_d), grid_stride(grid.dims(), col_d));
}

template <int D, typename T>
Grid<D, T> crop(CGridView<D, T> grid, const Vec<int, D>& dL, const Vec<int, D>& dU, Vec<Bndrule, D> bndrules,
                const T* bordervalue) {
  for_int(i, D) {
    if (bndrules[i] == Bndrule::undefined) assertx(dL[i] >= 0 && dU[i] >= 0);  // no negative crop
    if (bndrules[i] == Bndrule::border) assertx(bordervalue);
    if (grid.size() == 0)
      assertx(bndrules[i] != Bndrule::reflected && bndrules[i] != Bndrule::periodic &&
              bndrules[i] != Bndrule::clamped && bndrules[i] != Bndrule::reflected101);
  }
  Vec<int, D> newdims = max(grid.dims() - dL - dU, ntimes<D>(0));
  if (product(newdims) == 0) {
    Warning("cropping to grid with zero volume");
    if (0) newdims = ntimes<D>(0);
  }
  Grid<D, T> newgrid(newdims);
  if (dL.in_range(grid.dims()) && dU.in_range(grid.dims())) {  // faster path: no negative crop
    if (is_zero(dL.with(0, 0)) && is_zero(dU.with(0, 0))) {    // crop only in dim0
      if (0) {
        newgrid = grid.slice(dL[0], grid.dim(0) - dU[0]);
      } else {
        parallel_for_each(
            range(newgrid.dim(0)), [&](const int d0) { newgrid[d0].assign(grid[d0 + dL[0]]); },
            product(newgrid.dims().template tail<D - 1>()));
      }
    } else {
      parallel_for_coords(
          newdims,
          [&](const Vec<int, D>& u) {
            newgrid[u] = grid[u + dL];  // OPT:crop; almost as fast as above
          },
          20);
    }
  } else {  // slower path
    parallel_for_coords(
        newdims, [&](const Vec<int, D>& u) { newgrid[u] = grid.inside(u + dL, bndrules, bordervalue); }, 20);
  }
  return newgrid;
}

template <int D, typename U, typename T>
Grid<D, T> assemble(CGridView<D, U> grids, const T& background, const Vec<Alignment, D>& align) {
  static_assert(std::is_base_of_v<CGridView<D, T>, U>);
  Vec<Array<int>, D> max_sizes;  // max size of each slice [d = 0 .. D - 1][0 .. grids.dim(d)]
  for_int(d, D) max_sizes[d].init(grids.dim(d), 0);
  for (const auto& u : range(grids.dims())) {
    for_int(d, D) max_sizes[d][u[d]] = max(max_sizes[d][u[d]], grids[u].dim(d));
  }
  Vec<Array<int>, D> locs;  // start locations on each axis; [d = 0 .. D - 1][0 .. grids.dim(d)]
  for_int(d, D) {
    int tot = 0;
    for_int(j, grids.dim(d)) {
      locs[d].push(tot);
      tot += max_sizes[d][j];
    }
    locs[d].push(tot);
  }
  Vec<int, D> dims;
  for_int(d, D) dims[d] = locs[d].last();
  bool tight_packing = true;
  for (const auto& u : range(grids.dims())) {
    for_int(d, D) {
      if (grids[u].dim(d) != max_sizes[d][u[d]]) tight_packing = false;
    }
  }
  if (0) {
    for_coords(grids.dims(), [&](const Vec<int, D>& ugrid) { SHOW(ugrid, grids[ugrid].dims()); });
    SHOW(locs, dims);
  }
  Grid<D, T> grid(dims);
  if (!tight_packing) fill(grid, background);
  parallel_for_coords(grids.dims(), [&](const Vec<int, D>& ugrid) {
    CGridView<D, T> agrid = grids[ugrid];
    Vec<int, D> offset;
    for_int(d, D) {
      int uL = locs[d][ugrid[d]];
      int uU = locs[d][ugrid[d] + 1];
      int expected = uU - uL;
      switch (align[d]) {
        case Alignment::left: offset[d] = uL; break;
        case Alignment::center: offset[d] = uL + (expected - agrid.dim(d)) / 2; break;
        case Alignment::right: offset[d] = uL + (expected - agrid.dim(d)); break;
        default: assertnever("");
      }
    }
    for (const Vec<int, D> u : range(agrid.dims())) grid[offset + u] = agrid[u];
  });
  return grid;
}

template <int D, typename T> CGridView<D + 1, T> raise_grid_rank(CGridView<D, T> grid) {
  return CGridView<D + 1, T>(grid.data(), concat(V(1), grid.dims()));
}

inline bool env_image_linear_filter() {
  static const bool value = getenv_bool("IMAGE_LINEAR_FILTER");  // expand gamma
  return value;
}

template <int D> void convert(CGridView<D, Pixel> gridu, GridView<D, Vector4> gridf) {
  HH_GRIDOP_TIMER("__convert1");
  assertx(same_size(gridu, gridf));
  parallel_for_each(
      range(gridu.size()),
      [&](const size_t i) {
        Vector4 v(gridu.flat(i));
        if (env_image_linear_filter()) v = square(v);  // assumes gamma = 2.0 rather than SRGB 2.2
        gridf.flat(i) = v;
      },
      4);
}

template <int D> void convert(CGridView<D, Vector4> gridf, GridView<D, Pixel> gridu) {
  HH_GRIDOP_TIMER("__convert2");
  assertx(same_size(gridf, gridu));
  parallel_for_each(
      range(gridf.size()),
      [&](const size_t i) {
        Vector4 v = gridf.flat(i);
        if (env_image_linear_filter()) v = sqrt(v);  // assumes gamma = 2.0 rather than SRGB 2.2
        gridu.flat(i) = v.pixel();
      },
      4);
}

template <int D> void convert(CGridView<D, uint8_t> gridu, GridView<D, float> gridf) {
  assertx(same_size(gridu, gridf));
  parallel_for_each(
      range(gridu.size()),
      [&](const size_t i) {
        float v = float(gridu.flat(i));
        if (env_image_linear_filter()) v = square(v);
        gridf.flat(i) = v;
      },
      1);
}

template <int D> void convert(CGridView<D, float> gridf, GridView<D, uint8_t> gridu) {
  assertx(same_size(gridf, gridu));
  parallel_for_each(
      range(gridf.size()),
      [&](const size_t i) {
        float v = gridf.flat(i);
        if (env_image_linear_filter()) v = sqrt(v);
        gridu.flat(i) = clamp_to_uint8(int(v));
      },
      1);
}

template <int D> void convert(CGridView<D, Vec2<uint8_t>> gridu, GridView<D, Vector4> gridf) {
  assertx(same_size(gridu, gridf));
  parallel_for_each(
      range(gridu.size()),
      [&](const size_t i) {
        const auto& uv = gridu.flat(i);
        Vector4 v(Pixel(uv[0], uv[1], 0, 0));
        if (env_image_linear_filter()) v = square(v);
        gridf.flat(i) = v;
      },
      4);
}

template <int D> void convert(CGridView<D, Vector4> gridf, GridView<D, Vec2<uint8_t>> gridu) {
  assertx(same_size(gridf, gridu));
  parallel_for_each(
      range(gridf.size()),
      [&](const size_t i) {
        Vector4 v = gridf.flat(i);
        if (env_image_linear_filter()) v = sqrt(v);
        Pixel p = v.pixel();
        gridu.flat(i) = V(p[0], p[1]);
      },
      4);
}

//----------------------------------------------------------------------------

namespace details {

// Perform the inverse convolution preprocess for a single dimension d.
template <int D, typename T> void inverse_convolution_d(GridView<D, T> grid, const FilterBnd& filterb, int d) {
  HH_GRIDOP_TIMER("__inv_convol");
  const LUfactorization& lu = filterb.lu_factorization();
  const bool lastspecial = lu.Llastrow.num() > 0;
  const Vec<int, D>& dims = grid.dims();
  // TODO: always process a coherent swath of the last dimension together for better cache performance.
  const int cx = grid.dim(d);
  if (cx == 1) return;  // inverse convolution is identity
  const size_t stride = grid_stride(dims, d);
  const Vec<int, D> rows = grid.dims().with(d, 1);
  parallel_for_coords(rows, [&](const Vec<int, D>& urow) {  // nice
    // Preprocess (inverse convolution L U x = z)
    StridedArrayView<T> rowv(&grid[urow], cx, stride);
    // Forward pass: L y = z
    if (0) rowv[0] = rowv[0];                               // just copy first element
    for_intL(x, 1, min(cx - lastspecial, lu.Llower.num()))  // special initial elements
        rowv[x] = rowv[x] - lu.Llower[x] * rowv[x - 1];
    const float cfast = lu.Llower.last();
    if (lu.Llower.num() < cx - lastspecial) {
      T vprev = rowv[lu.Llower.num() - 1];
      for_intL(x, lu.Llower.num(), cx - lastspecial) {
        rowv[x] = vprev = rowv[x] - cfast * vprev;  // OPT:ic1
      }
    }
    if (lastspecial && cx > 1) {  // last element may be dense combination
      T& lastv = rowv[cx - 1];
      lastv -= lu.LlastrowPen[min(cx - 1, lu.LlastrowPen.num() - 1)] * rowv[cx - 2];
      for_int(x, min(cx - 2, lu.Llastrow.num())) lastv -= lu.Llastrow[x] * rowv[x];
    }
    // Backward pass: U x = y
    {
      T& lastv = rowv[cx - 1];  // scale last element by special value
      lastv *= lu.UidiagLast[min(cx - 1, lu.UidiagLast.num() - 1)];
      if (lastspecial && cx > 1) {  // penultimate element may be special
        rowv[cx - 2] = lu.Uidiag[min(cx - 2, lu.Uidiag.num() - 1)] *
                       (rowv[cx - 2] - lu.UlastcolPen[min(cx - 1, lu.UlastcolPen.num() - 1)] * lastv);
      }
    }
    const float cfast1 = lu.Uidiag.last(), cfast2 = lu.Uupper;
    const int low = lu.Uidiag.num() - 1;
    if (cx - 2 - lastspecial >= low) {
      T vprev = rowv[cx - 2 - lastspecial + 1];
      for (int x = cx - 2 - lastspecial; x >= low; --x) {
        rowv[x] = vprev = cfast1 * (rowv[x] - cfast2 * vprev);  // OPT:ic2
      }
    }
    for (int x = min(lu.Uidiag.num() - 2, cx - 2 - lastspecial); x >= 0; --x) {  // special initial elements
      rowv[x] = lu.Uidiag[x] * (rowv[x] - lu.Uupper * rowv[x + 1]);
      if (lastspecial) rowv[x] -= lu.Uidiag[x] * lu.Ulastcol[x] * rowv[cx - 1];
    }
  });
}

template <int D, typename T>
Grid<D, T> evaluate_kernel_d(CGridView<D, T> grid, int d, CArrayView<int> ar_pixelindex0,
                             CMatrixView<float> mat_weights, Bndrule bndrule, const T* bordervalue) {
  HH_GRIDOP_TIMER("__evaluate");
  const Vec<int, D>& dims = grid.dims();
  int cx = dims[d];
  int nx = ar_pixelindex0.num();
  int nk = mat_weights.xsize();
  const Vec<int, D> ndims = dims.with(d, nx);
  Grid<D, T> ngrid(ndims);
  const size_t stride = grid_stride(dims, d);
  // SHOW(dims, cx, nx, nk, ndims, stride);
  // SHOW(ar_pixelindex0); SHOW(mat_weights); SHOW(grid);
  assertx(stride == grid_stride(ndims, d));  // same stride in new grid (other dims unchanged)
  int ioutmin = 0;
  while (ioutmin < nx && ar_pixelindex0[ioutmin] < 0) ++ioutmin;
  int ioutmax = nx;
  while (ioutmax > ioutmin && ar_pixelindex0[ioutmax - 1] + nk > cx) --ioutmax;
  assertx(0 <= ioutmin && ioutmin <= ioutmax && ioutmax <= nx);
  auto func = [&](const Vec<int, D>& u) {
    int x = u[d];
    T v;
    my_zero(v);
    for_int(k, nk) {
      int ii = ar_pixelindex0[x] + k;
      const T& value = map_boundaryrule_1D(ii, cx, bndrule) ? grid[u.with(d, ii)] : *bordervalue;
      v += mat_weights[x][k] * value;
    }
    ngrid[u] = v;
  };
  auto func_interior = [&](const Vec<int, D>& u) {
    int x = u[d];
    const Vec<int, D> u0 = u.with(d, ar_pixelindex0[x]);
    size_t i0 = ravel_index(dims, u0);
    T v;
    my_zero(v);
    const float* mat_weights_x = mat_weights[x].data();
    const T* grid_i0 = grid.data() + i0;
    for_int(k, nk) v += mat_weights_x[k] * grid_i0[k * stride];
    ngrid[u] = v;
  };
  // TODO: use 2D tiling approach so that both grid and mat_weights fit in cache for wide grids.
  if (ngrid.size() * 20 < k_omp_thresh) {
    for_coords(ndims, func);
  } else if (0) {
    parallel_for_coords(ndims, func);  // not so slow
  } else {
    // timing test using:
    // Filterimage ~/data/image/lake.png -tile 10 10 -scaleu 2 | imgv
    // Filterimage ~/data/image/lake.png -tile 4 4 -info `perl -e 'binmode(STDOUT); for (1..10) { print " -scaleu 2 -scaleu .5"; }'` | imgv
    for_coordsL(ntimes<D>(0).with(d, 0), ndims.with(d, ioutmin), func);
    parallel_for_coordsL(ntimes<D>(0).with(d, ioutmin), ndims.with(d, ioutmax), func_interior);
    for_coordsL(ntimes<D>(0).with(d, ioutmax), ndims.with(d, nx), func);
  }
  return ngrid;
}

// Rescale grid such that dimension d has new size nx.
template <int D, typename T>
Grid<D, T> scale_d(CGridView<D, T> grid, int d, int nx, const FilterBnd& filterb, const T* bordervalue, bool primal,
                   Grid<D, T>&& gr = Grid<D, T>()) {
  const Vec<int, D>& dims = grid.dims();
  int cx = dims[d];
  if (nx == cx && filterb.filter().is_interpolating()) {
    gr = grid;
    return std::move(gr);
  }
  const bool is_magnify = nx >= cx;
  Array<int> ar_pixelindex0;
  Matrix<float> mat_weights;
  filterb.setup_kernel_weights(cx, nx, primal, ar_pixelindex0, mat_weights);
  if (is_magnify) {  // resampling/magnification
    if (filterb.filter().has_inv_convolution()) {
      if (grid.data() != gr.data()) gr = grid;  // else directly modify temporary input buffer
      details::inverse_convolution_d(gr, filterb, d);
      return details::evaluate_kernel_d(gr, d, ar_pixelindex0, mat_weights, filterb.bndrule(), bordervalue);
    } else {
      return details::evaluate_kernel_d(grid, d, ar_pixelindex0, mat_weights, filterb.bndrule(), bordervalue);
    }
  } else {  // minification
    gr = details::evaluate_kernel_d(grid, d, ar_pixelindex0, mat_weights, filterb.bndrule(), bordervalue);
    if (filterb.filter().has_inv_convolution()) details::inverse_convolution_d(gr, filterb, d);
    return std::move(gr);
  }
}

template <int D, typename T>
Grid<D, T> scale_i(CGridView<D, T> grid, const Vec<int, D>& ndims, const Vec<FilterBnd, D>& filterbs,
                   const T* bordervalue, Grid<D, T>&& gr, bool primal) {
  HH_GRIDOP_TIMER("__scale");
  if (any_of(filterbs, [](const FilterBnd& fb) { return fb.bndrule() == Bndrule::border; })) assertx(bordervalue);
  const Vec<int, D>& dims = grid.dims();
  int npreprocess = 0;
  for_int(d, D) {
    if (filterbs[d].filter().is_preprocess()) npreprocess++;
  }
  assertx(npreprocess == 0 || npreprocess == D);
  int njustspline = 0;
  for_int(d, D) {
    if (filterbs[d].filter().name() == "justspline") njustspline++;
  }
  assertx(njustspline == 0 || njustspline == D);
  // Note: IMAGE_EXPAND_VALUE_RANGE would be 2. to get a range [-.5, 1.5].
  // The lower factor of 1.5 was found empirically to yield the best results; see ~/prevproj/2011/vtfilter/Notes.txt.
  static const float k_expand_value_range = getenv_float("IMAGE_EXPAND_VALUE_RANGE", 1.5f, true);
  float expand_value_range = npreprocess ? 1.f / assertx(k_expand_value_range) : k_expand_value_range;
  if (npreprocess) {
    assertx(ndims == dims);
    const bool no_constrained_optimization = getenv_bool("IMAGE_NO_CONSTRAINED_OPTIMIZATION");
    Grid<D, T> ogrid;
    if (!no_constrained_optimization) ogrid = grid;  // backup of original grid
    gr = grid;                                       // no-op if gr.data() == grid.data()
    inverse_convolution(gr, filterbs);
    for (T& e : gr) e = T{.5f} + (e - T{.5f}) * expand_value_range;  // shrink range
    if (!no_constrained_optimization) {
      // Slow implementation: no parallelism, no fast interior, no precomputed grid of weights.
      Vec<Bndrule, D> bndrules;
      for_int(d, D) bndrules[d] = filterbs[d].bndrule();
      for (T& e : ogrid) e = T{.5f} + (e - T{.5f}) * expand_value_range;
      for (T& e : gr) e = general_clamp(e, T{0.f}, T{1.f});
      for_int(iter, 5) {  // 10 Gauss-Seidel iterations a tiny bit better; 100 no different
        for (const auto& u : range(dims)) {
          // if (gr[u] == 0.f || gr[u] == 1.f) continue;  // constrained forever at limit, if T is scalar
          T newv = ogrid[u];
          float wcenter = 0.f;
          for (const auto& ud : range(ntimes<D>(-1), ntimes<D>(+2))) {  // [-1, 0, +1]^D
            Vec<int, D> uu = u + ud;
            assertx(gr.map_inside(uu, bndrules));
            float w = 1.f;
            for_int(d, D) w *= (ud[d] == 0 ? (4.f / 6.f) : (1.f / 6.f));
            if (uu == u) {
              wcenter += w;
            } else {
              newv -= w * gr[uu];
            }
          }
          gr[u] = general_clamp(newv / wcenter, T{0.f}, T{1.f});
        }
      }
    }
    return std::move(gr);
  }
  struct Tup {
    int dim;
    float scaling;
  };
  Array<Tup> ar;
  for_int(d, D) ar.push(Tup{d, float(ndims[d]) / assertx(dims[d])});
  // Sort dimensions for quickest domain size reduction (or slowest domain size increase).
  // However, operating on last dimension is most efficient due to memory layout.
  // Therefore, if identical downsampling on multiple dimensions, we prefer to do last dimension first.
  // And,       if identical upsampling   on multiple dimensions, we prefer to do last dimension last.
  const auto by_quickest_size_reduction = [](const Tup& t1, const Tup& t2) {
    Vec2<Tup> tups(t1, t2);
    for (Tup& t : tups) {
      if (1 && t.dim == D - 1) {
        const float adjust = 2.f;
        if (t.scaling < 1.f) t.scaling /= adjust;  // encourage earlier minification of last dimension
        if (t.scaling > 1.f) t.scaling *= adjust;  // encourage later magnification of last dimension;
      }
    }
    return tups[0].scaling < tups[1].scaling;
  };
  sort(ar, by_quickest_size_reduction);
  CGridView<D, T> gridref(grid);  // (becomes gr after first iteration)
  for (const Tup& tup : ar) {
    int d = tup.dim;  // SHOW(d);
    gr = details::scale_d(gridref, d, ndims[d], filterbs[d], bordervalue, primal, std::move(gr));
    gridref.reinit(gr);
  }
  assertx(gridref.data() == gr.data());
  if (njustspline) {
    for (T& e : gr) e = T{.5f} + (e - T{.5f}) * expand_value_range;  // expand rage
  }
  return std::move(gr);
}

}  // namespace details

template <int D, typename T>
Vec<FilterBnd, D> inverse_convolution(GridView<D, T> grid, const Vec<FilterBnd, D>& filterbs) {
  Vec<FilterBnd, D> tfilterbs = filterbs;
  for_int(d, D) {
    const string& name = filterbs[d].filter().name();
    if (0)
      void();
    else if (name == "spline")
      tfilterbs[d].set_filter(Filter::get("justspline"));
    else if (name == "omoms")
      tfilterbs[d].set_filter(Filter::get("justomoms"));
    else if (name == "preprocess")
      void();
    else
      assertnever("Unrecognized image filter '" + name + "' for inverse_convolution");
  }
  for_int(d, D) details::inverse_convolution_d(grid, filterbs[d], d);
  return tfilterbs;
}

template <int D, typename T>
Grid<D, T> scale(CGridView<D, T> grid, const Vec<int, D>& ndims, const Vec<FilterBnd, D>& filterbs,
                 const T* bordervalue, Grid<D, T>&& gr) {
  return details::scale_i(grid, ndims, filterbs, bordervalue, std::move(gr), false);
}

template <int D, typename T>
Grid<D, T> scale_primal(CGridView<D, T> grid, const Vec<int, D>& ndims, const Vec<FilterBnd, D>& filterbs,
                        const T* bordervalue, Grid<D, T>&& gr) {
  return details::scale_i(grid, ndims, filterbs, bordervalue, std::move(gr), true);
}

namespace details {

template <int D, typename T, int DD>
void scale_filter_nearest_aux(Specialize<DD>, CGridView<D, T> grid, GridView<D, T> ngrid,
                              const Vec<Array<int>, D>& maps) {
  assertx(D != 2 && D != 3);  // specialized below
  auto func = [&](const Vec<int, D>& u) {
    Vec<int, D> uu;
    for_int(d, D) uu[d] = maps[d][u[d]];
    ngrid[u] = grid[uu];
  };
  parallel_for_coords(ngrid.dims(), func, 20);
}

template <int D, typename T>
void scale_filter_nearest_aux(Specialize<1>, CGridView<D, T> grid, GridView<D, T> ngrid,
                              const Vec<Array<int>, D>& maps) {
  ASSERTX(!maps[0].num());
  const Vec<int, D> dims = grid.dims();
  const Vec<int, D> ndims = ngrid.dims();
  parallel_for_each(
      range(ndims[0]),
      [&](const int i) {
        int ii = int((i + .5f) / ndims[0] * dims[0] - 1e-4f);
        ngrid(i) = grid(ii);
      },
      3);
}

template <int D, typename T>
void scale_filter_nearest_aux(Specialize<2>, CGridView<D, T> grid, GridView<D, T> ngrid,
                              const Vec<Array<int>, D>& maps) {
  const Vec<int, D> ndims = ngrid.dims();
  parallel_for_each(
      range(ndims[0]),
      [&](const int y) {
        int yy = maps[0][y];
        for_int(x, ndims[1]) ngrid(y, x) = grid(yy, maps[1][x]);
      },
      ndims[1] * 3);
}

template <int D, typename T>
void scale_filter_nearest_aux(Specialize<3>, CGridView<D, T> grid, GridView<D, T> ngrid,
                              const Vec<Array<int>, D>& maps) {
  const Vec<int, D> ndims = ngrid.dims();
  parallel_for_each(
      range(ndims[0]),
      [&](const int z) {
        int zz = maps[0][z];
        for_int(y, ndims[1]) {
          int yy = maps[1][y];
          for_int(x, ndims[2]) ngrid(z, y, x) = grid(zz, yy, maps[2][x]);
        }
      },
      ndims[1] * ndims[2] * 3);
}

}  // namespace details

template <int D, typename T>
Grid<D, T> scale_filter_nearest(CGridView<D, T> grid, const Vec<int, D>& ndims, Grid<D, T>&& gr) {
  HH_GRIDOP_TIMER("__scale_filter_nearest");
  if (!product(ndims)) {
    gr.init(ndims);
    return std::move(gr);
  }
  if (ndims == grid.dims()) {
    gr = grid;
    return std::move(gr);
  }
  Grid<D, T> ngrid(ndims);
  Vec<Array<int>, D> maps;
  if (D > 1) {
    Vec<int, D> dims = grid.dims();
    for_int(d, D) {
      Array<int>& map = maps[d];
      map.init(ndims[d]);
      for_int(i, ndims[d]) {
        map[i] = int((i + .5f) / ndims[d] * dims[d] - 1e-4f);
        ASSERTX(map[i] >= 0 && map[i] < dims[d]);
      }
    }
  }
  details::scale_filter_nearest_aux(Specialize<D>{}, grid, ngrid, maps);
  return ngrid;
}

template <int D, typename T>
T sample_grid(CGridView<D, T> g, const Vec<float, D>& p, const Vec<FilterBnd, D>& filterbs, const T* bordervalue) {
  Vec<int, D> uL, uU;
  Vec<PArray<float, 10>, D> matw;
  for_int(d, D) {
    if (filterbs[d].bndrule() == Bndrule::border) assertx(bordervalue);
    assertx(!filterbs[d].filter().has_inv_convolution());
    KernelFunc func = assertx(filterbs[d].filter().func());
    double kernel_radius = filterbs[d].filter().radius();
    uL[d] = int(floor(p[d] - kernel_radius));
    uU[d] = int(ceil(p[d] + kernel_radius)) + 1;
    for_intL(i, uL[d], uU[d]) matw[d].push(float(func(float(i) - p[d])));
  }
  Vec<Bndrule, D> bndrules;
  for_int(d, D) bndrules[d] = filterbs[d].bndrule();
  T val;
  my_zero(val);
  double sumw = 0.;
  for (const Vec<int, D>& u : range(uL, uU)) {
    float w = 1.f;
    for_int(d, D) w *= matw[d][u[d] - uL[d]];
    sumw += w;
    val += w * g.inside(u, bndrules, bordervalue);
  }
  return val / assertx(float(sumw));
}

template <int D, typename T>
T sample_domain(CGridView<D, T> g, const Vec<float, D>& p, const Vec<FilterBnd, D>& filterbs, const T* bordervalue) {
  Vec<float, D> pp;
  for_int(d, D) pp[d] = p[d] * g.dim(d) - .5f;
  return sample_grid(g, pp, filterbs, bordervalue);
}

template <int D, bool parallel>
Grid<D, Pixel> convolve_d(CGridView<D, Pixel> grid, int d, CArrayView<float> kernel, Bndrule bndrule,
                          const Pixel* bordervalue) {
  // HH_TIMER("__convolve_d");
  if (bndrule == Bndrule::border) assertx(bordervalue);
  const Vec<int, D>& dims = grid.dims();
  const int nx = dims[d];
  const int nk = kernel.num(), r = (nk - 1) / 2;
  assertx(r * 2 + 1 == nk);  // kernel must have odd size to be symmetric about each sample
  const size_t stride = grid_stride(dims, d);
  // SHOW(dims, nx, nk, stride); SHOW(kernel);
  assertx(abs(sum(kernel) - 1.) < 1e-6);  // kernel is expected to have unit integral
  const int ishift = 16, fac = 1 << ishift, fach = 1 << (ishift - 1);
  Array<int> kerneli;
  {
    kerneli = convert<int>(kernel * float(fac) + .5f);  // (all >= 0.f so no need for floor())
    int excess = narrow_cast<int>(sum(kerneli) - fac);
    assertx(abs(excess) <= nk);  // sanity check
    kerneli[r] -= excess;        // adjust center weight to make the quantized sum correct
  }
  Grid<D, Pixel> ngrid(dims);
  int ioutmin = min(r, nx);
  int ioutmax = max(ioutmin, nx - r);
  assertx(0 <= ioutmin && ioutmin <= ioutmax && ioutmax <= nx);
  auto func = [&](const Vec<int, D>& u) {
    int x = u[d];
    Vector4i v{0};
    for_int(k, nk) {
      int ii = x - r + k;
      // NOLINTNEXTLINE(clang-analyzer-core.NullDereference)
      const Pixel& value = map_boundaryrule_1D(ii, nx, bndrule) ? grid[u.with(d, ii)] : *bordervalue;
      v += kerneli[k] * Vector4i(value);
    }
    ngrid[u] = ((v + fach) >> ishift).pixel();
  };
  auto func_interior = [&](const Vec<int, D>& u) {
    int x = u[d];
    const Vec<int, D> u0 = u.with(d, x - r);
    size_t i0 = ravel_index(dims, u0);
    Vector4i v{0};
    for_int(k, nk) v += kerneli[k] * Vector4i(grid.flat(i0 + k * stride));  // OPT:blur
    ngrid[u] = ((v + fach) >> ishift).pixel();
  };
  // TODO: use 2D tiling approach so that both grid and kernel fit in cache for wide grids.
  if (0) {
    for_coords(dims, func);
  } else {
    // timing test using:
    // Filterimage ~/data/image/lake.png -tile 10 10 -info -blur 1 -info | imgv
    // Filterimage ~/data/image/lake.png -tile 2 2 -info `perl -e 'binmode(STDOUT); for (1..10) { print " -blur 1"; }'` -info | imgv
    for_coordsL(ntimes<D>(0).with(d, 0), dims.with(d, ioutmin), func);
    if (!parallel || ngrid.size() * nk * 5 < k_omp_thresh)
      for_coordsL(ntimes<D>(0).with(d, ioutmin), dims.with(d, ioutmax), func_interior);
    else
      parallel_for_coordsL(ntimes<D>(0).with(d, ioutmin), dims.with(d, ioutmax), func_interior);
    for_coordsL(ntimes<D>(0).with(d, ioutmax), dims.with(d, nx), func);
  }
  return ngrid;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_GRIDOP_H_
