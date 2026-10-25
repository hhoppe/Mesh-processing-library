// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_GRIDOP_H_
#define MESH_PROCESSING_LIBHH_GRIDOP_H_

#include "libHh/Array.h"
#include "libHh/DigitalFilter.h"
#include "libHh/Filter.h"
#include "libHh/Grid.h"
#include "libHh/ParallelCoords.h"
#include "libHh/Pixel.h"
#include "libHh/ResampleOptions.h"
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
template <int COL_D, int D, typename T>
[[nodiscard]] CStridedArrayView<T> grid_column(CGridView<D, T> grid, const Vec<int, D>& u0);

// Create a view (StridedArrayView) onto the column of the grid (along dimension COL_D) starting at position u0.
template <int COL_D, int D, typename T>
[[nodiscard]] StridedArrayView<T> grid_column(GridView<D, T> grid, const Vec<int, D>& u0);

// Create a view (CStridedArrayView) onto the column of the grid (along dimension col_d) starting at position u0.
template <int D, typename T>
[[nodiscard]] CStridedArrayView<T> grid_column(CGridView<D, T> grid, int col_d, const Vec<int, D>& u0);

// Create a view (StridedArrayView) onto the column of the grid (along dimension col_d) starting at position u0.
template <int D, typename T>
[[nodiscard]] StridedArrayView<T> grid_column(GridView<D, T> grid, int col_d, const Vec<int, D>& u0);

// Crop the sides of a D-dimensional grid (dL/dU for lower/upper extents); any negative crop grows the grid
//  and requires the definition of boundary rules (and border value if Bndrule::border).
template <int D, typename T>
[[nodiscard]] Grid<D, T> crop(CGridView<D, T> grid, const Vec<int, D>& dL, const Vec<int, D>& dU,
                              Vec<Bndrule, D> bndrules = ntimes<D>(Bndrule::undefined),
                              const T* bordervalue = nullptr);

enum class Alignment { left, center, right };

// Merge a D-dimensional grid of D-dimensional grids into a single D-dimensional grid.
//  (U must be derived from CGridView<D, T>).
template <int D, typename U, typename T = U::value_type>
[[nodiscard]] Grid<D, T> assemble(CGridView<D, U> grids, const T& background = T{},
                                  const Vec<Alignment, D>& align = ntimes<D>(Alignment::center));

// Recast a grid view as a next-higher-dimensional grid view with just a single slice in dimension zero.
template <int D, typename T> [[nodiscard]] CGridView<D + 1, T> raise_grid_rank(CGridView<D, T> grid);

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
[[nodiscard]] Grid<D, T> scale(CGridView<D, T> grid, const Vec<int, D>& ndims, const Vec<FilterBnd, D>& filterbs,
                               const T* bordervalue = nullptr, Grid<D, T>&& gr = Grid<D, T>());

// Same as scale() but assumes that samples of old and new grids
//  lie at locations [0., 1. / (dim(0) - 1), ..., (dim(0) - 1) / (dim(0) - 1) == 1.] on each dimension.
template <int D, typename T>
[[nodiscard]] Grid<D, T> scale_primal(CGridView<D, T> grid, const Vec<int, D>& ndims,
                                      const Vec<FilterBnd, D>& filterbs, const T* bordervalue = nullptr,
                                      Grid<D, T>&& gr = Grid<D, T>());

// Rescale a grid to have new dimensions ndims, specialized to nearest sampling; works on arbitrary types T,
//  even those that are non-interpolable.
template <int D, typename T>
[[nodiscard]] Grid<D, T> scale_filter_nearest(CGridView<D, T> grid, const Vec<int, D>& ndims,
                                              Grid<D, T>&& gr = Grid<D, T>());

// Apply inverse convolution filter preprocess (spline | omoms) to grid; return new filter (justspline | justomoms).
template <int D, typename T>
Vec<FilterBnd, D> inverse_convolution(GridView<D, T> grid, const Vec<FilterBnd, D>& filterbs);

// Evaluate the filter at point p of grid g, where samples lie at integers ([0, dim(0) - 1], ..., [0, dim(D - 1) - 1]).
template <int D, typename T>
[[nodiscard]] T sample_grid(CGridView<D, T> g, const Vec<float, D>& p, const Vec<FilterBnd, D>& filterbs,
                            const T* bordervalue = nullptr);

// Evaluate the filter at point p of grid g, where grid domain is [0, 1]^D, and thus grid samples lie at
// locations [.5 / dim(0) ... (dim(0) - .5) / dim(0)] ... [.5 / dim(D - 1) ... (dim(D - 1) - .5) / dim(D - 1)].
template <int D, typename T>
[[nodiscard]] T sample_domain(CGridView<D, T> g, const Vec<float, D>& p, const Vec<FilterBnd, D>& filterbs,
                              const T* bordervalue = nullptr);

// Rescale a grid to have new dimensions ndims according to the resampling options (sample placement, kernels,
// boundary rules); cval is the constant value referenced by some boundary rules (e.g. "border").
template <int D, typename T>
[[nodiscard]] Grid<D, T> scale(CGridView<D, T> grid, const Vec<int, D>& ndims, const ResizeOptions<D>& options = {},
                               const T* cval = nullptr, Grid<D, T>&& gr = Grid<D, T>());

// Evaluate the reconstruction at point p of grid g, where samples lie at integers ([0, dim(0) - 1], ...).
template <int D, typename T>
[[nodiscard]] T sample_grid(CGridView<D, T> g, const Vec<float, D>& p, const SamplingOptions<D>& options = {},
                            const T* cval = nullptr);

// Evaluate the reconstruction at point p of grid g, whose domain is [0, 1]^D with the samples placed according to
// options.gridtype (dual: (i + .5) / dim(d); primal: i / (dim(d) - 1)).
template <int D, typename T>
[[nodiscard]] T sample_domain(CGridView<D, T> g, const Vec<float, D>& p, const SamplingOptions<D>& options = {},
                              const T* cval = nullptr);

enum class ObjectFit { contain, cover };  // As in CSS: fit the source within the destination, or cover it.

// Resize a grid to dimensions ndims with the same scaling in every dimension (preserving the aspect ratio), like
// CSS object-fit; options.scale and options.translate must be their defaults, and the boundary rule "auto" means
// "natural" here (cval, or T{} if null, outside the source domain).
template <int D, typename T>
[[nodiscard]] Grid<D, T> uniform_resize(CGridView<D, T> grid, const Vec<int, D>& ndims, ObjectFit object_fit,
                                        ResizeOptions<D> options = {}, const T* cval = nullptr);

// A grid prepared for reconstruction at arbitrary points with the kernels, boundary rules, and sample placement of
// the options: the coefficients (the grid after the digital filter when the kernel requires one) are computed once
// and the reconstruction is evaluated many times.  The grid must outlive the Sampler unless it is copied.
template <int D, typename T> class Sampler : noncopyable {
 public:
  explicit Sampler(CGridView<D, T> grid, const SamplingOptions<D>& options = {}, const T* cval = nullptr);
  Sampler(Sampler&& rhs) noexcept;
  // Reconstruct the value at point p of the unit domain [0, 1]^D.
  [[nodiscard]] T operator()(const Vec<float, D>& p) const;
  // Reconstruct the value at point p in grid coordinates (the samples lie at the integers).
  [[nodiscard]] T at_grid(const Vec<float, D>& p) const;
  // Reconstruct the value at point p of the unit domain, where `spacing` is the distance between neighboring
  // reconstruction points in each dimension (antialiasing the transition to cval of the rules that override the
  // value outside the domain) and `override_exterior` false disables those rules (see resample()).
  [[nodiscard]] T evaluate(const Vec<float, D>& p, const Vec<float, D>& spacing, bool override_exterior) const;
  [[nodiscard]] CGridView<D, T> coefficients() const { return _coefficients; }

 private:
  Grid<D, T> _owned;  // The coefficients when they differ from the grid.
  CGridView<D, T> _coefficients;
  Vec<Gridtype, D> _gridtypes;
  Vec<const Boundary*, D> _boundaries;
  Vec<KernelFunc, D> _funcs;  // The kernels evaluated after the digital filter.
  Vec<float, D> _radii;
  bool _has_cval{false};
  T _cval{};
  [[nodiscard]] T evaluate_grid(const Vec<float, D>& x, const Vec<float, D>& p, const Vec<float, D>& spacing,
                                bool override_exterior) const;
};

// Reconstruct the grid at the unit-domain points `coords`, producing a grid of the dimensions of `coords`.
template <int Dout, int D, typename T>
[[nodiscard]] Grid<Dout, T> resample(CGridView<D, T> grid, CGridView<Dout, Vec<float, D>> coords,
                                     const SamplingOptions<D>& options = {}, const T* cval = nullptr);

namespace details {

// The precomputed kernel weights of one dimension of a Resizer.
struct ResizerDim {
  const Filter* filter{nullptr};      // The resolved kernel (the prefilter when minifying).
  const Boundary* boundary{nullptr};  // The resolved boundary rule.
  Gridtype src_gridtype{Gridtype::dual}, dst_gridtype{Gridtype::dual};
  bool identity{false};  // Same size and interpolating kernel: the dimension is copied.
  bool is_minification{false};
  Array<int> index0;           // [nx] First source index of each row's window of nk dense weights.
  Matrix<float> weights;       // [nx][nk] The dense weights, used directly by the interior rows.
  int ioutmin{0}, ioutmax{0};  // The rows [ioutmin, ioutmax) are interior: their windows lie inside the grid.
  // The folded rows: the taps j in [row_begin[x], row_begin[x + 1]) of row x have source index col[j] (or -1 for
  // the constant value cval) and weight val[j], in the order of the window so that sums match the dense rows.
  Array<int> row_begin;  // [nx + 1]
  Array<int> col;
  Array<float> val;
};

}  // namespace details

// The precomputed kernel weights for resizing grids of dimensions dims to dimensions ndims according to options.
// The rows whose kernel window lies inside the grid keep dense weights; the other rows have their taps folded by the
// boundary rule into interior samples (and the constant value cval).  One Resizer serves any number of grids.
template <int D> class Resizer {
 public:
  Resizer(const Vec<int, D>& dims, const Vec<int, D>& ndims, const ResizeOptions<D>& options);
  [[nodiscard]] const Vec<int, D>& dims() const { return _dims; }
  [[nodiscard]] const Vec<int, D>& ndims() const { return _ndims; }
  [[nodiscard]] const Vec<int, D>& dim_order() const { return _dim_order; }
  [[nodiscard]] const Filter& filter(int d) const { return *_plan[d].filter; }
  [[nodiscard]] const Boundary& boundary(int d) const { return *_plan[d].boundary; }
  // Resize a grid of dimensions dims() into a grid of dimensions ndims(); cval is the constant value referenced by
  // some boundary rules (e.g. "border").
  template <typename T>
  [[nodiscard]] Grid<D, T> apply(CGridView<D, T> grid, const T* cval = nullptr, Grid<D, T>&& gr = Grid<D, T>()) const;
  // The taps of output sample i along dimension d.  For an interior row, `indices` is empty and the taps are the
  // contiguous source samples [index0, index0 + weights.num()).  Otherwise `indices` gives the source index of each
  // tap, possibly repeating an index, where -1 denotes cval (whose total weight is cval_weight).
  struct Row {
    int index0;
    CArrayView<int> indices;
    CArrayView<float> weights;
    float cval_weight;
  };
  [[nodiscard]] Row row(int d, int i) const;

 private:
  Vec<int, D> _dims, _ndims, _dim_order;
  Vec<details::ResizerDim, D> _plan;
  bool _all_preprocess{false};  // All kernels are "preprocess": only the digital filter is applied.
  bool _all_bspline3{false};    // All kernels are "bspline3": the value range is expanded afterwards.
  [[nodiscard]] Vec<FilterBnd, D> filterbnds() const;  // (Transitional bridge to the digital filter.)
};

// Convolve a Pixel grid along its d'th dimension with the 1D kernel (which must have odd length).
template <int D, bool parallel = true>
[[nodiscard]] Grid<D, Pixel> convolve_d(CGridView<D, Pixel> grid, int d, CArrayView<float> kernel, Bndrule bndrule,
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
    if (bndrules[i] == Bndrule::border) assertx(bordervalue);
    if (grid.size() == 0)
      assertx(bndrules[i] != Bndrule::reflect && bndrules[i] != Bndrule::wrap && bndrules[i] != Bndrule::clamp &&
              bndrules[i] != Bndrule::reflect_primal);
    if (bndrules[i] == Bndrule::undefined) {
      assertx(dL[i] >= 0 && dU[i] >= 0);  // No negative crop.
      // So the dimension is never accessed outside its domain, and any defined rule maps it identically.  (The slower
      // path below maps every dimension through map_boundaryrule_1d(), which rejects Bndrule::undefined.)
      bndrules[i] = Bndrule::clamp;
    }
  }
  Vec<int, D> newdims = max(grid.dims() - dL - dU, ntimes<D>(0));
  if (product(newdims) == 0) {
    Warning("cropping to grid with zero volume");
    if (0) newdims = ntimes<D>(0);
  }
  Grid<D, T> newgrid(newdims);
  if (dL.in_range(grid.dims()) && dU.in_range(grid.dims())) {  // Faster path: no negative crop.
    if (is_zero(dL.with(0, 0)) && is_zero(dU.with(0, 0))) {    // Crop only in dim0.
      if (0) {
        newgrid = grid.slice(dL[0], grid.dim(0) - dU[0]);
      } else {
        const ParallelOptions parallel_options{
            .cycles_per_elem = uint64_t(product(newgrid.dims().template tail<D - 1>())),
        };
        parallel_for(parallel_options, range(newgrid.dim(0)), [&](const int d0) {  //
          newgrid[d0].assign(grid[d0 + dL[0]]);
        });
      }
    } else {
      parallel_for_coords({.cycles_per_elem = 20}, newdims, [&](const Vec<int, D>& u) {
        newgrid[u] = grid[u + dL];  // OPT:crop; almost as fast as above
      });
    }
  } else {
    // Slower path.
    parallel_for_coords({.cycles_per_elem = 20}, newdims, [&](const Vec<int, D>& u) {  //
      newgrid[u] = grid.inside(u + dL, bndrules, bordervalue);
    });
  }
  return newgrid;
}

template <int D, typename U, typename T>
Grid<D, T> assemble(CGridView<D, U> grids, const T& background, const Vec<Alignment, D>& align) {
  static_assert(std::is_base_of_v<CGridView<D, T>, U>);
  Vec<Array<int>, D> max_sizes;  // Maximum size of each slice [d = 0 .. D - 1][0 .. grids.dim(d)].
  for_int(d, D) max_sizes[d].init(grids.dim(d), 0);
  for (const auto& u : range(grids.dims()))
    for_int(d, D) max_sizes[d][u[d]] = max(max_sizes[d][u[d]], grids[u].dim(d));
  Vec<Array<int>, D> locs;  // Start locations on each axis; [d = 0 .. D - 1][0 .. grids.dim(d)].
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
      const int uL = locs[d][ugrid[d]];
      const int uU = locs[d][ugrid[d] + 1];
      const int expected = uU - uL;
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
  static const bool value = getenv_bool("IMAGE_LINEAR_FILTER");  // Expand gamma.  Assumes gamma = 2.0 (not 2.2).
  return value;
}

template <int D> void convert(CGridView<D, Pixel> gridu, GridView<D, Vector4> gridf) {
  HH_GRIDOP_TIMER("__convert1");
  assertx(same_size(gridu, gridf));
  if (env_image_linear_filter())
    parallel_for({.cycles_per_elem = 4}, range(gridu.size()),
                 [&](const size_t i) { gridf.flat(i) = square(Vector4(gridu.flat(i))); });
  else
    parallel_for({.cycles_per_elem = 4}, range(gridu.size()),
                 [&](const size_t i) { gridf.flat(i) = Vector4(gridu.flat(i)); });
}

template <int D> void convert(CGridView<D, Vector4> gridf, GridView<D, Pixel> gridu) {
  HH_GRIDOP_TIMER("__convert2");
  assertx(same_size(gridf, gridu));
  if (env_image_linear_filter())
    parallel_for({.cycles_per_elem = 4}, range(gridf.size()),
                 [&](const size_t i) { gridu.flat(i) = sqrt(gridf.flat(i)).pixel(); });
  else
    parallel_for({.cycles_per_elem = 4}, range(gridf.size()),
                 [&](const size_t i) { gridu.flat(i) = gridf.flat(i).pixel(); });
}

template <int D> void convert(CGridView<D, uint8_t> gridu, GridView<D, float> gridf) {
  assertx(same_size(gridu, gridf));
  if (env_image_linear_filter())
    parallel_for({.cycles_per_elem = 1}, range(gridu.size()),
                 [&](const size_t i) { gridf.flat(i) = square(float(gridu.flat(i))); });
  else
    parallel_for({.cycles_per_elem = 1}, range(gridu.size()),
                 [&](const size_t i) { gridf.flat(i) = float(gridu.flat(i)); });
}

template <int D> void convert(CGridView<D, float> gridf, GridView<D, uint8_t> gridu) {
  assertx(same_size(gridf, gridu));
  if (env_image_linear_filter())
    parallel_for({.cycles_per_elem = 1}, range(gridf.size()),
                 [&](const size_t i) { gridu.flat(i) = clamp_to_uint8(int(sqrt(gridf.flat(i)))); });
  else
    parallel_for({.cycles_per_elem = 1}, range(gridf.size()),
                 [&](const size_t i) { gridu.flat(i) = clamp_to_uint8(int(gridf.flat(i))); });
}

template <int D> void convert(CGridView<D, Vec2<uint8_t>> gridu, GridView<D, Vector4> gridf) {
  assertx(same_size(gridu, gridf));
  if (env_image_linear_filter())
    parallel_for({.cycles_per_elem = 4}, range(gridu.size()), [&](const size_t i) {
      const auto& uv = gridu.flat(i);
      gridf.flat(i) = square(Vector4(Pixel(uv[0], uv[1], 0, 0)));
    });
  else
    parallel_for({.cycles_per_elem = 4}, range(gridu.size()), [&](const size_t i) {
      const auto& uv = gridu.flat(i);
      gridf.flat(i) = Vector4(Pixel(uv[0], uv[1], 0, 0));
    });
}

template <int D> void convert(CGridView<D, Vector4> gridf, GridView<D, Vec2<uint8_t>> gridu) {
  assertx(same_size(gridf, gridu));
  if (env_image_linear_filter())
    parallel_for({.cycles_per_elem = 4}, range(gridf.size()), [&](const size_t i) {
      const Pixel p = sqrt(gridf.flat(i)).pixel();
      gridu.flat(i) = V(p[0], p[1]);
    });
  else
    parallel_for({.cycles_per_elem = 4}, range(gridf.size()), [&](const size_t i) {
      const Pixel p = gridf.flat(i).pixel();
      gridu.flat(i) = V(p[0], p[1]);
    });
}

//----------------------------------------------------------------------------

namespace details {

// Apply the digital filter (inverse convolution) of a generalized kernel along dimension d, in place.
template <int D, typename T>
void inverse_convolution_d(GridView<D, T> grid, const Filter& filter, const Boundary& boundary, Gridtype gridtype,
                           int d, const T* cval) {
  HH_GRIDOP_TIMER("__inv_convol");
  const Vec<int, D>& dims = grid.dims();
  const int cx = dims[d];
  const DigitalFilter& digital_filter = DigitalFilter::get(filter, boundary, gridtype, cx);
  const size_t stride = grid_stride(dims, d);
  const Vec<int, D> rows = dims.with(d, 1);
  // TODO: Always process a coherent swath of the last dimension together for better cache performance.
  parallel_for_coords(rows, [&](const Vec<int, D>& urow) {
    digital_filter.solve(StridedArrayView<T>(&grid[urow], cx, stride), cval);
  });
}

// (Transitional:) the digital filter for a FilterBnd, on a dual grid.
template <int D, typename T> void inverse_convolution_d(GridView<D, T> grid, const FilterBnd& filterb, int d) {
  inverse_convolution_d(grid, filterb.filter(), Boundary::get(filterb.bndrule()), Gridtype::dual, d,
                        implicit_cast<const T*>(nullptr));
}

// The factor by which the "bspline3" kernel (the cubic B-spline evaluated after the "preprocess" digital filter)
// expands the value range, and by which the "preprocess" filter shrinks it.
inline float expand_value_range() {
  // Note: IMAGE_EXPAND_VALUE_RANGE would be 2. to get a range [-.5, 1.5].
  // The lower factor of 1.5 was found empirically to yield the best results; see ~/prevproj/2011/vtfilter/Notes.txt.
  static const float k_expand_value_range = getenv_float("IMAGE_EXPAND_VALUE_RANGE", 1.5f, true);
  return assertx(k_expand_value_range);
}

// The "preprocess" kernels: apply the digital filter of the cubic B-spline to the grid, shrink the value range, and
// optimize the coefficients so that the reconstruction stays within [0, 1].
template <int D, typename T>
Grid<D, T> scale_preprocess(CGridView<D, T> grid, const Vec<FilterBnd, D>& filterbs, Grid<D, T>&& gr) {
  const Vec<int, D>& dims = grid.dims();
  const float expand_value_range = 1.f / details::expand_value_range();
  const bool no_constrained_optimization = getenv_bool("IMAGE_NO_CONSTRAINED_OPTIMIZATION");
  Grid<D, T> ogrid;
  if (!no_constrained_optimization) ogrid = grid;  // A backup of the original grid.
  gr = grid;                                       // A no-op if gr.data() == grid.data().
  inverse_convolution(gr, filterbs);
  for (T& e : gr) e = T{.5f} + (e - T{.5f}) * expand_value_range;  // Shrink the range.
  if (!no_constrained_optimization) {
    // Slow implementation: no parallelism, no fast interior, no precomputed grid of weights.
    Vec<Bndrule, D> bndrules;
    for_int(d, D) bndrules[d] = filterbs[d].bndrule();
    for (T& e : ogrid) e = T{.5f} + (e - T{.5f}) * expand_value_range;
    for (T& e : gr) e = general_clamp(e, T{0.f}, T{1.f});
    for_int(iter, 5) {  // 10 Gauss-Seidel iterations are a tiny bit better; 100 are no different.
      for (const auto& u : range(dims)) {
        // if (gr[u] == 0.f || gr[u] == 1.f) continue;  // Constrained forever at limit, if T is scalar.
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

}  // namespace details

template <int D, typename T>
Vec<FilterBnd, D> inverse_convolution(GridView<D, T> grid, const Vec<FilterBnd, D>& filterbs) {
  Vec<FilterBnd, D> tfilterbs = filterbs;
  for_int(d, D) {
    const Filter& filter = filterbs[d].filter();
    if (!filter.requires_digital_filter())
      assertnever("Unrecognized image filter '" + filter.name() + "' for inverse_convolution");
    tfilterbs[d].set_filter(filter.digital_filter_kernel());
  }
  for_int(d, D) details::inverse_convolution_d(grid, filterbs[d], d);
  return tfilterbs;
}

template <int D, typename T>
Grid<D, T> scale_filter_nearest(CGridView<D, T> grid, const Vec<int, D>& ndims, Grid<D, T>&& gr) {
  HH_GRIDOP_TIMER("__scale_filter_nearest");
  if (!product(ndims)) {
    gr.init(ndims);
    return std::move(gr);
  }
  const Vec<int, D> dims = grid.dims();
  if (ndims == dims) {
    gr = grid;
    return std::move(gr);
  }
  Grid<D, T> ngrid(ndims);
  Vec<Array<int>, D> maps;
  if constexpr (D > 1) {
    for_int(d, D) {
      Array<int>& map = maps[d];
      map.init(ndims[d]);
      for_int(i, ndims[d]) {
        map[i] = int((i + .5f) / ndims[d] * dims[d] - 1e-4f);
        ASSERTX(map[i] >= 0 && map[i] < dims[d]);
      }
    }
  }
  if constexpr (D == 1) {
    ASSERTX(!maps[0].num());
    parallel_for({.cycles_per_elem = 3}, range(ndims[0]), [&](const int i) {
      const int ii = int((i + .5f) / ndims[0] * dims[0] - 1e-4f);
      ngrid[i] = grid[ii];
    });
  } else if constexpr (D == 2) {
    parallel_for({.cycles_per_elem = uint64_t(ndims[1]) * 3}, range(ndims[0]), [&](const int y) {
      const int yy = maps[0][y];
      for_int(x, ndims[1]) ngrid[y, x] = grid[yy, maps[1][x]];
    });
  } else if constexpr (D == 3) {
    parallel_for({.cycles_per_elem = uint64_t(ndims[1] * ndims[2]) * 3}, range(ndims[0]), [&](const int z) {
      const int zz = maps[0][z];
      for_int(y, ndims[1]) {
        const int yy = maps[1][y];
        for_int(x, ndims[2]) ngrid[z, y, x] = grid[zz, yy, maps[2][x]];
      }
    });
  } else {
    const auto func = [&](const Vec<int, D>& u) {
      Vec<int, D> uu;
      for_int(d, D) uu[d] = maps[d][u[d]];
      ngrid[u] = grid[uu];
    };
    parallel_for_coords({.cycles_per_elem = 20}, ndims, func);
  }
  return ngrid;
}

namespace details {

// Compute the rows of kernel weights for resizing cx samples to nx samples along one dimension.
inline void compute_resizer_dim(ResizerDim& rd, int cx, int nx, Gridtype src_gridtype, Gridtype dst_gridtype,
                                double scale, double translate) {
  const Filter& filter = *rd.filter;
  const Boundary& boundary = *rd.boundary;
  const bool src_dual = src_gridtype == Gridtype::dual, dst_dual = dst_gridtype == Gridtype::dual;
  assertx(cx >= (src_dual ? 1 : 2) && nx >= (dst_dual ? 1 : 2));
  const double scaling = (dst_dual ? nx : nx - 1.) / (src_dual ? cx : cx - 1.) * scale;
  const bool is_impulse = filter.is_impulse(), is_trapezoid = filter.is_trapezoid();
  const KernelFunc kernel_func = is_impulse || is_trapezoid ? nullptr : filter.func();
  // The trapezoid has value 1 for |x| <= 1 - radius and decreases linearly to 0 at |x| = radius (always .5 at .5).
  const double radius = is_trapezoid ? .5 + .5 * min(scaling, 1. / scaling) : filter.radius();
  const auto kernel = [&](double v) {
    return is_trapezoid ? clamp((.5 + .25 / (radius - .5)) - (.5 / (radius - .5)) * abs(v), 0., 1.) : kernel_func(v);
  };
  // (The tolerance avoids an extra zero-weight tap when the ratio is an integer up to floating-point error.)
  const int nk = is_impulse ? 1 : int(std::ceil((rd.is_minification ? radius * 2. / scaling : radius * 2.) - 1e-6));
  // The distance in the source domain between consecutive destination samples (for antialiased boundaries).
  const double spacing = (dst_dual ? 1. / nx : 1. / (nx - 1)) / scale;
  const bool nkodd = (nk % 2) > 0;
  rd.index0.init(nx);
  rd.weights.init(nx, nk);
  rd.row_begin.init(nx + 1);
  rd.col.clear();
  rd.val.clear();
  Array<bool> interior(nx);
  Array<int> indices;
  Array<float> folded;
  const auto src_position_of = [&](int x) {
    const double dst_position = dst_dual ? (x + .5) / nx : double(x) / (nx - 1);  // In the unit domain.
    return boundary.remap_coordinate((dst_position - translate) / scale);
  };
  // As in resampler, the rules that override the value outside the unit domain do so (with an antialiased
  // transition) only if some destination sample maps outside the domain; otherwise they pass the domain center.
  bool any_outside = false;
  for_int(x, nx) {
    const double src_position = src_position_of(x);
    if (src_position < 0. || src_position > 1.) any_outside = true;
  }
  for_int(x, nx) {
    const double src_position = src_position_of(x);
    const double src_index =
        src_dual ? src_position * cx - .5 : src_position * (cx - 1);  // Source sample coordinates.
    const double wf = std::floor(src_index + (nkodd ? .5 : 0.));
    const int index0 = int(wf) - ((nk - 1) / 2);
    rd.index0[x] = index0;
    double sum = 0.;
    for_int(k, nk) {
      const double w0 = double(index0 + k);
      const double v = is_impulse            ? 1.
                       : !rd.is_minification ? kernel(src_index - w0)
                                             : kernel((src_index - w0) * scaling) * scaling;
      rd.weights[x, k] = float(v);
      sum += v;
    }
    rd.weights[x] /= float(sum);
    rd.row_begin[x] = rd.col.num();
    indices.clear();
    folded.clear();
    boundary.fold(index0, rd.weights[x], cx, src_gridtype, any_outside ? src_position : .5, spacing, indices, folded);
    rd.col.push_array(indices);
    rd.val.push_array(folded);
    // The row is interior if its window lies inside the grid and the rule left its taps unchanged.
    interior[x] = index0 >= 0 && index0 + nk <= cx && indices.num() == nk;
    for_int(k, nk) {
      if (interior[x] && (indices[k] != index0 + k || folded[k] != rd.weights[x, k])) interior[x] = false;
    }
  }
  rd.row_begin[nx] = rd.col.num();
  rd.ioutmin = 0;
  while (rd.ioutmin < nx && !interior[rd.ioutmin]) rd.ioutmin++;
  rd.ioutmax = nx;
  while (rd.ioutmax > rd.ioutmin && !interior[rd.ioutmax - 1]) rd.ioutmax--;
  for_intL(x, rd.ioutmin, rd.ioutmax) {
    if (!interior[x]) {  // E.g. "tile" remaps coordinates non-monotonically: all rows use their folded taps.
      rd.ioutmin = rd.ioutmax = 0;
      break;
    }
  }
}

// Resample dimension d of the grid using the precomputed rows of weights.
template <int D, typename T>
Grid<D, T> evaluate_rows_d(CGridView<D, T> grid, int d, const ResizerDim& rd, const T* cval) {
  HH_GRIDOP_TIMER("__evaluate");
  static_assert(std::is_trivially_default_constructible_v<T>);
  const Vec<int, D>& dims = grid.dims();
  const int nx = rd.index0.num();
  const int nk = rd.weights.xsize();
  const Vec<int, D> ndims = dims.with(d, nx);
  Grid<D, T> ngrid(ndims);
  const size_t stride = grid_stride(dims, d);
  assertx(stride == grid_stride(ndims, d));  // Same stride in the new grid (other dims unchanged).
  const CArrayView<int> ar_pixelindex0 = rd.index0;
  const CMatrixView<float> mat_weights = rd.weights;
  const auto func = [&](const Vec<int, D>& u) {  // A row evaluated through its folded taps.
    const int x = u[d];
    T v{};
    for_intL(j, rd.row_begin[x], rd.row_begin[x + 1]) {
      const int i = rd.col[j];
      v += rd.val[j] * (i >= 0 ? grid[u.with(d, i)] : *cval);
    }
    ngrid[u] = v;
  };
  const auto func_interior = [&](const Vec<int, D>& u) {
    const int x = u[d];
    const Vec<int, D> u0 = u.with(d, ar_pixelindex0[x]);
    const size_t i0 = ravel_index(dims, u0);
    T v{};
    const float* mat_weights_x = mat_weights[x].data();
    const T* grid_i0 = grid.data() + i0;
    for_int(k, nk) v += mat_weights_x[k] * grid_i0[k * stride];
    ngrid[u] = v;
  };
  // TODO: Use 2D tiling approach so that both grid and mat_weights fit in cache for wide grids.
  if (ngrid.size() * 20 < k_parallel_thresh) {
    for_coords(ndims, func);
  } else {
    // Timing test using:
    // Filterimage ~/data/image/lake.png -tile 10 10 -scaleu 2 | vv
    // Filterimage ~/data/image/lake.png -tile 4 4 -info $(printf -- '-scaleu 2 -scaleu .5 %.0s' {1..10}) | vv
    for_coordsL(ntimes<D>(0).with(d, 0), ndims.with(d, rd.ioutmin), func);
    parallel_for_coordsL(ntimes<D>(0).with(d, rd.ioutmin), ndims.with(d, rd.ioutmax), func_interior);
    for_coordsL(ntimes<D>(0).with(d, rd.ioutmax), ndims.with(d, nx), func);
  }
  return ngrid;
}

// Express the filters and boundary rules of a Vec<FilterBnd, D> as resizing options (a transitional bridge).
template <int D> ResizeOptions<D> options_from_filterbnds(const Vec<FilterBnd, D>& filterbs) {
  ResizeOptions<D> options;
  for_int(d, D) {
    options.filter[d] = FilterRef(filterbs[d].filter());
    options.boundary[d] = BoundaryRef(Boundary::get(filterbs[d].bndrule()));
  }
  return options;
}

}  // namespace details

template <int D>
Resizer<D>::Resizer(const Vec<int, D>& dims, const Vec<int, D>& ndims, const ResizeOptions<D>& options)
    : _dims(dims), _ndims(ndims) {
  assertx(options.gamma->is_identity());  // (The transfer function of integer samples is applied by the callers.)
  const PerDim<Gridtype, D>& dst_gridtype = options.dst_gridtype ? *options.dst_gridtype : options.gridtype;
  int npreprocess = 0, nbspline3 = 0;
  for_int(d, D) {
    details::ResizerDim& rd = _plan[d];
    const int cx = dims[d], nx = ndims[d];
    const Gridtype src_gridtype = options.gridtype[d];
    const double src_in_samples = src_gridtype == Gridtype::dual ? cx : cx - 1.;
    const double dst_in_samples = dst_gridtype[d] == Gridtype::dual ? nx : nx - 1.;
    const double scaling = dst_in_samples / src_in_samples * options.scale[d];
    rd.is_minification = scaling < 1.;
    rd.filter =
        rd.is_minification && !options.prefilter[d]->is_auto() ? options.prefilter[d].ptr : options.filter[d].ptr;
    rd.boundary = options.boundary[d]->is_auto() ? &Boundary::get(rd.is_minification ? "clamp" : "reflect")
                                                 : options.boundary[d].ptr;
    rd.src_gridtype = src_gridtype, rd.dst_gridtype = dst_gridtype[d];
    if (rd.filter->is_preprocess()) npreprocess++;
    if (rd.filter->name() == "bspline3") nbspline3++;
    rd.identity = nx == cx && rd.filter->is_interpolating() && options.scale[d] == 1. && options.translate[d] == 0. &&
                  src_gridtype == dst_gridtype[d];
    if (rd.identity || rd.filter->is_preprocess()) continue;
    details::compute_resizer_dim(rd, cx, nx, src_gridtype, dst_gridtype[d], options.scale[d], options.translate[d]);
  }
  assertx(npreprocess == 0 || npreprocess == D);
  _all_preprocess = npreprocess == D;
  if (_all_preprocess) assertx(ndims == dims);
  assertx(nbspline3 == 0 || nbspline3 == D);
  _all_bspline3 = nbspline3 == D;
  if (options.dim_order) {
    _dim_order = *options.dim_order;
    Vec<int, D> sorted = _dim_order;
    sort(sorted);
    for_int(d, D) assertx(sorted[d] == d);  // A permutation of the dimensions.
  } else {
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
          if (t.scaling < 1.f) t.scaling /= adjust;  // Encourage earlier minification of the last dimension.
          if (t.scaling > 1.f) t.scaling *= adjust;  // Encourage later magnification of last dimension.
        }
      }
      return tups[0].scaling < tups[1].scaling;
    };
    sort(ar, by_quickest_size_reduction);
    for_int(i, D) _dim_order[i] = ar[i].dim;
  }
}

template <int D> Vec<FilterBnd, D> Resizer<D>::filterbnds() const {
  Vec<FilterBnd, D> filterbs = ntimes<D>(FilterBnd(Filter::get("box"), Bndrule::reflect));
  for_int(d, D) filterbs[d] = FilterBnd(*_plan[d].filter, parse_boundaryrule(_plan[d].boundary->name()));
  return filterbs;
}

template <int D> typename Resizer<D>::Row Resizer<D>::row(int d, int i) const {
  const details::ResizerDim& rd = _plan[d];
  assertx(!rd.identity && !rd.filter->is_preprocess());
  if (i >= rd.ioutmin && i < rd.ioutmax) return Row{rd.index0[i], CArrayView<int>(nullptr, 0), rd.weights[i], 0.f};
  const int j0 = rd.row_begin[i], j1 = rd.row_begin[i + 1];
  float cval_weight = 0.f;
  for_intL(j, j0, j1) {
    if (rd.col[j] < 0) cval_weight += rd.val[j];
  }
  return Row{rd.index0[i], rd.col.slice(j0, j1), rd.val.slice(j0, j1), cval_weight};
}

template <int D>
template <typename T>
Grid<D, T> Resizer<D>::apply(CGridView<D, T> grid, const T* cval, Grid<D, T>&& gr) const {
  HH_GRIDOP_TIMER("__scale");
  assertx(grid.dims() == _dims);
  for_int(d, D) {
    if (_plan[d].boundary->uses_cval()) assertx(cval);
  }
  if (_all_preprocess) return details::scale_preprocess(grid, filterbnds(), std::move(gr));
  CGridView<D, T> gridref(grid);  // (Becomes gr after the first dimension.)
  for (const int d : _dim_order) {
    const details::ResizerDim& rd = _plan[d];
    if (rd.identity) {
      gr = gridref;
    } else if (!rd.is_minification) {  // Resampling or magnification.
      if (rd.filter->requires_digital_filter()) {
        if (gridref.data() != gr.data()) gr = gridref;  // Else directly modify the temporary input buffer.
        details::inverse_convolution_d(gr, *rd.filter, *rd.boundary, rd.src_gridtype, d, cval);
        gr = details::evaluate_rows_d(CGridView<D, T>(gr), d, rd, cval);
      } else {
        gr = details::evaluate_rows_d(gridref, d, rd, cval);
      }
    } else {  // Minification.
      gr = details::evaluate_rows_d(gridref, d, rd, cval);
      if (rd.filter->requires_digital_filter())
        details::inverse_convolution_d(gr, *rd.filter, *rd.boundary, rd.dst_gridtype, d, cval);
    }
    gridref.reinit(gr);
  }
  assertx(gridref.data() == gr.data());
  if (_all_bspline3) {
    const float expand_value_range = details::expand_value_range();
    for (T& e : gr) e = T{.5f} + (e - T{.5f}) * expand_value_range;  // Expand the range.
  }
  return std::move(gr);
}

template <int D, typename T>
Grid<D, T> scale(CGridView<D, T> grid, const Vec<int, D>& ndims, const ResizeOptions<D>& options, const T* cval,
                 Grid<D, T>&& gr) {
  const Resizer<D> resizer(grid.dims(), ndims, options);
  return resizer.apply(grid, cval, std::move(gr));
}

template <int D, typename T>
Grid<D, T> scale(CGridView<D, T> grid, const Vec<int, D>& ndims, const Vec<FilterBnd, D>& filterbs,
                 const T* bordervalue, Grid<D, T>&& gr) {
  return scale(grid, ndims, details::options_from_filterbnds(filterbs), bordervalue, std::move(gr));
}

template <int D, typename T>
Grid<D, T> scale_primal(CGridView<D, T> grid, const Vec<int, D>& ndims, const Vec<FilterBnd, D>& filterbs,
                        const T* bordervalue, Grid<D, T>&& gr) {
  ResizeOptions<D> options = details::options_from_filterbnds(filterbs);
  options.gridtype = Gridtype::primal;
  return scale(grid, ndims, options, bordervalue, std::move(gr));
}

template <int D, typename T>
Grid<D, T> uniform_resize(CGridView<D, T> grid, const Vec<int, D>& ndims, ObjectFit object_fit,
                          ResizeOptions<D> options, const T* cval) {
  const PerDim<Gridtype, D>& dst_gridtype = options.dst_gridtype ? *options.dst_gridtype : options.gridtype;
  Vec<double, D> raw_scales;
  for_int(d, D) {
    assertx(options.scale[d] == 1. && options.translate[d] == 0.);
    const double src_in_samples = options.gridtype[d] == Gridtype::dual ? grid.dim(d) : grid.dim(d) - 1.;
    const double dst_in_samples = dst_gridtype[d] == Gridtype::dual ? ndims[d] : ndims[d] - 1.;
    raw_scales[d] = dst_in_samples / src_in_samples;
  }
  const double scale0 = object_fit == ObjectFit::contain ? min(raw_scales) : max(raw_scales);
  for_int(d, D) {
    options.scale[d] = scale0 / raw_scales[d];
    options.translate[d] = (1. - options.scale[d]) / 2.;
    if (options.boundary[d]->is_auto()) options.boundary[d] = BoundaryRef(Boundary::get("natural"));
  }
  const T default_cval{};
  return scale(grid, ndims, options, cval ? cval : &default_cval);
}

namespace details {

// Express the filters and boundary rules of a Vec<FilterBnd, D> as sampling options (a transitional bridge).
template <int D> SamplingOptions<D> sampling_options_from_filterbnds(const Vec<FilterBnd, D>& filterbs) {
  SamplingOptions<D> options;
  for_int(d, D) {
    options.filter[d] = FilterRef(filterbs[d].filter());
    options.boundary[d] = BoundaryRef(Boundary::get(filterbs[d].bndrule()));
  }
  return options;
}

}  // namespace details

template <int D, typename T>
Sampler<D, T>::Sampler(CGridView<D, T> grid, const SamplingOptions<D>& options, const T* cval) : _coefficients(grid) {
  static_assert(std::is_trivially_default_constructible_v<T>);
  assertx(options.gamma->is_identity());  // (The transfer function of integer samples is applied by the callers.)
  static const Boundary& k_reflect = Boundary::get("reflect");
  if (cval) _has_cval = true, _cval = *cval;
  bool digital = false;
  for_int(d, D) {
    const Filter& filter = *options.filter[d];
    assertx(!filter.is_impulse() && !filter.is_trapezoid());  // These kernels serve only scale().
    _gridtypes[d] = options.gridtype[d];
    assertx(grid.dim(d) >= (_gridtypes[d] == Gridtype::dual ? 1 : 2));
    _boundaries[d] = options.boundary[d]->is_auto() ? &k_reflect : options.boundary[d].ptr;
    if (_boundaries[d]->uses_cval()) assertx(cval);
    const Filter& kernel = filter.digital_filter_kernel();
    _funcs[d] = kernel.func(), _radii[d] = float(kernel.radius());
    if (filter.requires_digital_filter()) digital = true;
  }
  if (digital) {
    _owned = grid;
    for_int(d, D) {
      if (options.filter[d]->requires_digital_filter())
        details::inverse_convolution_d(GridView<D, T>(_owned), *options.filter[d], *_boundaries[d], _gridtypes[d], d,
                                       cval);
    }
    _coefficients.reinit(_owned);
  }
}

template <int D, typename T>
Sampler<D, T>::Sampler(Sampler&& rhs) noexcept
    : _owned(std::move(rhs._owned)),
      _coefficients(_owned.size() ? CGridView<D, T>(_owned) : rhs._coefficients),
      _gridtypes(rhs._gridtypes),
      _boundaries(rhs._boundaries),
      _funcs(rhs._funcs),
      _radii(rhs._radii),
      _has_cval(rhs._has_cval),
      _cval(rhs._cval) {}

template <int D, typename T> T Sampler<D, T>::operator()(const Vec<float, D>& p) const {
  Vec<float, D> x;
  for_int(d, D) {
    const int dim = _coefficients.dim(d);
    x[d] = _gridtypes[d] == Gridtype::dual ? p[d] * dim - .5f : p[d] * (dim - 1);
  }
  return evaluate_grid(x, p, ntimes<D>(0.f), true);
}

template <int D, typename T> T Sampler<D, T>::at_grid(const Vec<float, D>& x) const {
  Vec<float, D> p;
  for_int(d, D) {
    const int dim = _coefficients.dim(d);
    p[d] = _gridtypes[d] == Gridtype::dual ? (x[d] + .5f) / dim : x[d] / (dim - 1);
  }
  return evaluate_grid(x, p, ntimes<D>(0.f), true);
}

template <int D, typename T>
T Sampler<D, T>::evaluate(const Vec<float, D>& p, const Vec<float, D>& spacing, bool override_exterior) const {
  Vec<float, D> x;
  for_int(d, D) {
    const int dim = _coefficients.dim(d);
    x[d] = _gridtypes[d] == Gridtype::dual ? p[d] * dim - .5f : p[d] * (dim - 1);
  }
  return evaluate_grid(x, p, spacing, override_exterior);
}

template <int D, typename T>
T Sampler<D, T>::evaluate_grid(const Vec<float, D>& x, const Vec<float, D>& p, const Vec<float, D>& spacing,
                               bool override_exterior) const {
  // The taps of each dimension: either the dense kernel window, when it lies inside the grid and the boundary rule
  // leaves it unchanged (the common case), or the taps folded by the rule.  The scratch arrays avoid allocations.
  struct Taps {
    int index0, count;
    const float* weights;
    const int* indices;  // Null for the contiguous indices [index0, index0 + count).
  };
  Vec<Taps, D> taps;
  Vec<int, D> counts;
  static thread_local Vec<Array<float>, D> dense;
  static thread_local Vec<Array<int>, D> indices;
  static thread_local Vec<Array<float>, D> folded;
  for_int(d, D) {
    const int dim = _coefficients.dim(d);
    const float radius = _radii[d];
    const int i0 = int(std::floor(x[d] - radius)), i1 = int(std::ceil(x[d] + radius)) + 1;
    Array<float>& dense_d = dense[d];
    dense_d.init(i1 - i0);
    double sum = 0.;
    for_intL(i, i0, i1) sum += dense_d[i - i0] = float(_funcs[d](float(i) - x[d]));
    dense_d /= float(sum);
    const Boundary& boundary = *_boundaries[d];
    const bool overridden =
        override_exterior && boundary.overrides_exterior() && (p[d] < 0.f || p[d] > 1.f || spacing[d] > 0.f);
    if (i0 >= 0 && i1 <= dim && !overridden) {
      taps[d] = Taps{i0, i1 - i0, dense_d.data(), nullptr};
    } else {
      indices[d].clear(), folded[d].clear();
      boundary.fold(i0, dense_d, dim, _gridtypes[d], override_exterior ? p[d] : .5, spacing[d], indices[d], folded[d]);
      taps[d] = Taps{0, indices[d].num(), folded[d].data(), indices[d].data()};
    }
    counts[d] = taps[d].count;
  }
  T val{};
  for (const Vec<int, D>& k : range(counts)) {
    float w = 1.f;
    bool exterior = false;
    Vec<int, D> u;
    for_int(d, D) {
      const Taps& t = taps[d];
      const int i = t.indices ? t.indices[k[d]] : t.index0 + k[d];
      if (i < 0) exterior = true;
      u[d] = max(i, 0);
      w *= t.weights[k[d]];
    }
    if (w) val += w * (exterior ? _cval : _coefficients[u]);
  }
  return val;
}

template <int Dout, int D, typename T>
Grid<Dout, T> resample(CGridView<D, T> grid, CGridView<Dout, Vec<float, D>> coords, const SamplingOptions<D>& options,
                       const T* cval) {
  const Sampler<D, T> sampler(grid, options, cval);
  Grid<Dout, T> result(coords.dims());
  // As in resampler, the rules that override the value outside the unit domain apply only if some point lies
  // outside, and the transition to cval is then antialiased using the gradient of the coordinates over the
  // output grid (central differences).
  bool any_outside = false;
  for (const Vec<float, D>& p : coords) {
    for_int(d, D) {
      if (p[d] < 0.f || p[d] > 1.f) any_outside = true;
    }
  }
  const bool antialias = any_outside && min(coords.dims()) >= 2;
  parallel_for_coords({.cycles_per_elem = 500}, coords.dims(), [&](const Vec<int, Dout>& u) {
    Vec<float, D> spacing = ntimes<D>(0.f);
    if (antialias) {
      for_int(d, D) {
        float sum2 = 0.f;
        for_int(a, Dout) {
          const Vec<int, Dout> um = u.with(a, max(u[a] - 1, 0)), up = u.with(a, min(u[a] + 1, coords.dim(a) - 1));
          const float g = (coords[up][d] - coords[um][d]) / float(up[a] - um[a]);
          sum2 += g * g;
        }
        spacing[d] = std::sqrt(sum2);
      }
    }
    result[u] = sampler.evaluate(coords[u], spacing, any_outside);
  });
  return result;
}

template <int D, typename T>
T sample_grid(CGridView<D, T> g, const Vec<float, D>& p, const SamplingOptions<D>& options, const T* cval) {
  return Sampler<D, T>(g, options, cval).at_grid(p);
}

template <int D, typename T>
T sample_domain(CGridView<D, T> g, const Vec<float, D>& p, const SamplingOptions<D>& options, const T* cval) {
  return Sampler<D, T>(g, options, cval)(p);
}

template <int D, typename T>
T sample_grid(CGridView<D, T> g, const Vec<float, D>& p, const Vec<FilterBnd, D>& filterbs, const T* bordervalue) {
  return sample_grid(g, p, details::sampling_options_from_filterbnds(filterbs), bordervalue);
}

template <int D, typename T>
T sample_domain(CGridView<D, T> g, const Vec<float, D>& p, const Vec<FilterBnd, D>& filterbs, const T* bordervalue) {
  return sample_domain(g, p, details::sampling_options_from_filterbnds(filterbs), bordervalue);
}

template <int D, bool parallel>
Grid<D, Pixel> convolve_d(CGridView<D, Pixel> grid, int d, CArrayView<float> kernel, Bndrule bndrule,
                          const Pixel* bordervalue) {
  // HH_TIMER("__convolve_d");
  if (bndrule == Bndrule::border) assertx(bordervalue);
  const Vec<int, D>& dims = grid.dims();
  const int nx = dims[d];
  const int nk = kernel.num(), r = (nk - 1) / 2;
  assertx(r * 2 + 1 == nk);  // The kernel must have odd size to be symmetric about each sample.
  const size_t stride = grid_stride(dims, d);
  // SHOW(dims, nx, nk, stride); SHOW(kernel);
  assertx(abs(sum(kernel) - 1.) < 1e-6);  // The kernel is expected to have unit integral.
  const int ishift = 16, fac = 1 << ishift, fach = 1 << (ishift - 1);
  Array<int> kerneli;
  {
    kerneli = convert<int>(kernel * float(fac) + .5f);  // All >= 0.f, so there is no need for floor().
    const int excess = narrow_cast<int>(sum(kerneli) - fac);
    assertx(abs(excess) <= nk);  // A sanity check.
    kerneli[r] -= excess;        // Adjust the center weight to make the quantized sum correct.
  }
  Grid<D, Pixel> ngrid(dims);
  const int ioutmin = min(r, nx);
  const int ioutmax = max(ioutmin, nx - r);
  assertx(0 <= ioutmin && ioutmin <= ioutmax && ioutmax <= nx);
  const auto func = [&](const Vec<int, D>& u) {
    const int x = u[d];
    Vector4i v{0};
    for_int(k, nk) {
      int ii = x - r + k;
      const Pixel* pvalue = map_boundaryrule_1d(ii, nx, bndrule) ? &grid[u.with(d, ii)] : bordervalue;
      v += kerneli[k] * Vector4i(*pvalue);
    }
    ngrid[u] = ((v + fach) >> ishift).pixel();
  };
  const auto func_interior = [&](const Vec<int, D>& u) {
    const int x = u[d];
    const Vec<int, D> u0 = u.with(d, x - r);
    const size_t i0 = ravel_index(dims, u0);
    Vector4i v{0};
    for_int(k, nk) v += kerneli[k] * Vector4i(grid.flat(i0 + k * stride));  // OPT:blur
    ngrid[u] = ((v + fach) >> ishift).pixel();
  };
  // TODO: Use 2D tiling approach so that both grid and kernel fit in cache for wide grids.
  if (0) {
    for_coords(dims, func);
  } else {
    // Timing test using:
    // Filterimage ~/data/image/lake.png -tile 10 10 -info -blur 1 -info | vv
    // Filterimage ~/data/image/lake.png -tile 2 2 -info $(printf -- '-blur 1 %.0s' {1..10}) -info | vv
    for_coordsL(ntimes<D>(0).with(d, 0), dims.with(d, ioutmin), func);
    if (!parallel || ngrid.size() * nk * 5 < k_parallel_thresh)
      for_coordsL(ntimes<D>(0).with(d, ioutmin), dims.with(d, ioutmax), func_interior);
    else
      parallel_for_coordsL(ntimes<D>(0).with(d, ioutmin), dims.with(d, ioutmax), func_interior);
    for_coordsL(ntimes<D>(0).with(d, ioutmax), dims.with(d, nx), func);
  }
  return ngrid;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_GRIDOP_H_
