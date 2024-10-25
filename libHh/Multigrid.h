// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MULTIGRID_H_
#define MESH_PROCESSING_LIBHH_MULTIGRID_H_

#include "libHh/Advanced.h"  // unroll<>
#include "libHh/Grid.h"
#include "libHh/MathOp.h"
#include "libHh/ParallelCoords.h"
#include "libHh/RangeOp.h"
#include "libHh/Set.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"
#include "libHh/Vector4.h"

#if 0
{
  Vec2<int> dims{ny, nx};
  Multigrid<2, float> multigrid(dims);
  fill(multigrid.initial_estimate(), 0.f);
  multigrid.set_desired_mean(mean(value));
  setup_rhs(grid_orig, multigrid.rhs());
  if (optional)
    for (const auto& u : range(dims)) multigrid.initial_estimate()[u] = some_value;
  // Grid<2, float> grid_orig(dims); if (optional) multigrid.set_original(grid_orig);  // For convergence analysis.
  multigrid.solve();
  CGridView<2, float> grid_result = multigrid.result();
  if (1) HH_RSTAT(Sresult, grid_result);
}
#endif

namespace hh {

#if !defined(HH_MULTIGRID_TIMER)  // client can define HH_MULTIGRID_TIMER(name) as nothing to omit all the timers.
#define HH_MULTIGRID_TIMER(name) HH_STIMER(name)
#endif

// Notes:
//  see also precursor code test_Multigrid2D.cpp and test_Multigrid1D.cpp

// *** Helper functions

// Possibly contract a vector to a scalar value by taking its norm.
inline float mag_e(float e) { return e; }
inline double mag_e(double e) { return e; }
inline float mag_e(const Vector4& e) { return sqrt(mag2(e)); }

// Possibly contract a vector to a scalar value by taking its maximum coefficient value.
inline float max_e(float e) { return e; }
inline double max_e(double e) { return e; }
inline float max_e(const Vector4& e) { return max(e); }

// Specialize mean() of Grid for parallelism.
template <int D, typename T> std::enable_if_t<std::is_arithmetic_v<T>, mean_type_t<T>> mean(CGridView<D, T> g) {
  using MeanType = mean_type_t<T>;
  MeanType v;
  my_zero(v);
  intptr_t size = g.size();
  if (!size) {
    Warning("Zero-size grid");
    return v;
  }
  if (g.size() * 1 < k_parallel_thresh) {
    for (intptr_t i : range(size)) v += g.flat(i);
  } else {
    const int num_threads = get_max_threads();
    Array<MeanType> means(num_threads);
    parallel_for_chunk(range(size), num_threads, [&](const int thread_index, auto subrange) {
      MeanType v2;
      my_zero(v2);
      for (const intptr_t i : subrange) v2 += g.flat(i);
      means[thread_index] = v2;
    });
    for_int(thread_index, num_threads) v += means[thread_index];
  }
  return v * (1. / size);
}
template <int D, typename T> std::enable_if_t<std::is_arithmetic_v<T>, mean_type_t<T>> mean(GridView<D, T> g) {
  return mean(static_cast<CGridView<D, T>>(g));
}
template <int D, typename T> std::enable_if_t<std::is_arithmetic_v<T>, mean_type_t<T>> mean(const Grid<D, T>& g) {
  return mean(static_cast<CGridView<D, T>>(g));
}

// *** Specializations for Vector4

// Specialize mean() to give sufficient precision on large Grid of Vector4 -- since missing mean_type<Vector4>.
template <int D> Vector4 mean(CGridView<D, Vector4> grid) {
  Vec4<double> v;
  fill(v, 0.);
  for (const auto& e : grid) for_int(c, 4) v[c] += e[c];
  v = v / double(assertx(grid.size()));
  Vector4 vec;
  for_int(c, 4) vec[c] = float(v[c]);
  return vec;
}
template <int D> Vector4 mean(GridView<D, Vector4> grid) { return mean(static_cast<CGridView<D, Vector4>>(grid)); }
template <int D> Vector4 mean(const Grid<D, Vector4>& grid) { return mean(static_cast<CGridView<D, Vector4>>(grid)); }

// Specialize range_stat() to operate on magnitude of Vector4 elements.
template <int D> Stat range_stat(const Grid<D, Vector4>& range) {
  Stat stat;
  for (auto e : range) stat.enter(mag_e(e));
  return stat;
}

// *** Multigrid solver

// Domain boundary in each dimension can be periodic or Neumann.  Default is non-periodic on all dimensions.
template <int D> struct MultigridPeriodicNone {  // would return true if axis d were periodic
  bool operator()(int d) const {
    ASSERTX(d >= 0 && d < D);
    return false;
  }
};

template <int D> struct MultigridPeriodicAll {
  bool operator()(int d) const {
    ASSERTX(d >= 0 && d < D);
    return true;
  }
};

// Laplacian can be anisotropic, with a transformation in each dimension.  Default is isotropic (identity).
template <int D> struct MultigridMetricIsotropic {  // would modify metric value v for axis d if anisotropic
  float operator()(float v, int d) const {
    ASSERTX(d >= 0 && d < D);
    return v;
  }
};

// Solve a Poisson problem over D-dimensional domain with elements of type T using a multigrid solver,
//  optionally with Periodic boundary conditions and with an anisotropic Metric on the Laplacian.
template <int D, typename T, typename Periodic = MultigridPeriodicNone<D>,
          typename Metric = MultigridMetricIsotropic<D>>
class Multigrid : noncopyable {
  using Precise = sum_type_t<T>;

 public:
  explicit Multigrid(const Vec<int, D>& dims) : _grid_result(dims), _grid_rhs(dims) {}  // not zero'ed!
  void set_original(CGridView<D, T> grid_orig) {                                        // optional, for error analysis
    _grid_orig.reinit(grid_orig);
    assertx(same_size(_grid_orig, _grid_result));
    _mean_orig = mean(grid_orig);
    set_desired_mean(_mean_orig);
  }
  void set_desired_mean(const Precise& v) { _mean_desired = v, _have_mean_desired = true; }
  void set_num_vcycles(int v) { _num_vcycles = v; }
  GridView<D, T>& rhs() { return _grid_rhs; }                  // set right-hand-side constraints
  GridView<D, T>& initial_estimate() { return _grid_result; }  // should be set!
  void set_verbose(bool v) { _verbose = v; }
  void set_screening_weight(float v) { _screening_weight = v; }
  void solve() { run_multigrid(_grid_rhs, _grid_result); }
  void just_relax(int niter) { relax(_grid_rhs, _grid_result, niter, false); }
  CGridView<D, T> result() { return _grid_result; }  // retrieve result
 private:
  Grid<D, T> _grid_result;
  Grid<D, T> _grid_rhs;
  CGridView<D, T> _grid_orig{nullptr, ntimes<D>(0)};  // defined if have_orig()
  Precise _mean_desired{BIGFLOAT};
  bool _have_mean_desired{false};
  Precise _mean_orig{BIGFLOAT};
  int _num_vcycles{0};
  bool _verbose{false};  // include analysis of residual and error
  float _screening_weight{0.f};
  Periodic _periodic;
  Metric _metric;
  //
  static constexpr bool b_no_periodicity = std::is_same_v<Periodic, MultigridPeriodicNone<D>>;
  static constexpr bool b_default_metric = std::is_same_v<Metric, MultigridMetricIsotropic<D>>;
  static constexpr bool b_fastest = 0;
  static constexpr int k_direct_solver_resolution = 2;  // = 2, 4, 8
  static constexpr int k_num_iter_gauss_seidel = 2;     // 1, 2, 5, 10, 20, 100; note that 1 or 2 is sufficient
  static constexpr int k_default_num_vcycles = 0 ? 25 : std::is_same_v<T, double> ? 12 : 5;  // 5, 12, 16, 25, 50
  static constexpr bool k_enable_specializations = b_no_periodicity && 1;
  //
  bool have_orig() const { return _grid_orig.size() > 0; }
  static constexpr int Z = k_enable_specializations ? D : -1;  // for use in Specialize<Z>
  //
  static Vec<int, D> generate_interior_offsets(const Vec<int, D>& dims) {
    Vec<int, D> ar_interior_offsets;
    for_int(c, D) {
      int o = c == D - 1 ? 1 : assert_narrow_cast<int>(product(dims.slice(c + 1, D)));
      ar_interior_offsets[c] = o;
    }
    return ar_interior_offsets;
  }
  // Box filter on dual grid; dimensions are halved except along dimensions whose size is already 1.
  Grid<D, T> dual_downsample(CGridView<D, T> grid) { return dual_downsample_aux(Specialize<Z>{}, grid); }
  template <int DD> Grid<D, T> dual_downsample_aux(Specialize<DD>, CGridView<D, T> grid) {
    HH_MULTIGRID_TIMER("_downsample");
    const Vec<int, D> dims = grid.dims();
    assertx(max(dims) > 1);
    const Vec<int, D> ndims = (dims + 1) / 2;
    Grid<D, T> ngrid(ndims);
    // uses box filter of width 2, except filter has extent 1 along dimensions whose size is already 1.
    Vec<int, D> vrange;
    for_int(c, D) vrange[c] = dims[c] > 1 ? 2 : 1;
    assertx(product(vrange) > 1);
    float fac = 1.f / product(vrange);
    int largest_odd_size = 0, largest_other_size = 0;
    for (int d : dims) {
      if (d % 2 == 1 && d > largest_odd_size) std::swap(d, largest_odd_size);
      largest_other_size = max(largest_other_size, d);
    }
    T bordervalue;
    my_zero(bordervalue);
    // Easy inside part [0, dims / 2 - 1]:
    parallel_for_coords({uint64_t(product(vrange)) * 2}, dims / 2, [&](const Vec<int, D>& u) {
      T v;
      my_zero(v);
      for (const auto& ut : range(u * 2, u * 2 + vrange)) v += grid[ut];
      ngrid[u] = v * fac;
    });
    // The optional D leftover hyperplanes [dims / 2, ndims - 1] for odd sizes
    Vec<Bndrule, D> bndrules;
    for_int(d, D) {  // These boundary rules were chosen because they behave well empirically.
      bndrules[d] = (_periodic(d)                             ? Bndrule::border
                     : largest_odd_size >= largest_other_size ? Bndrule::border
                                                              : Bndrule::reflected);
    }
    for_int(c, D) {
      const Vec<int, D> uL = ntimes<D>(0).with(c, dims[c] / 2);
      const Vec<int, D> uU = ndims;
      for (const auto& u : range(uL, uU)) {
        T v;
        my_zero(v);
        for (const auto& ut : range(u * 2, u * 2 + vrange)) v += grid.inside(ut, bndrules, &bordervalue);
        ngrid[u] = v * fac;
      }
    }
    return ngrid;
  }
  Grid<D, T> dual_downsample_aux(Specialize<2>, CGridView<D, T> grid) {
    HH_MULTIGRID_TIMER("_downsample2");
    const Vec<int, D> dims = grid.dims();
    assertx(max(dims) > 1);
    const Vec<int, D> ndims = (dims + 1) / 2;
    Grid<D, T> ngrid(ndims);
    assertx(!_periodic(0) && !_periodic(1));  // this case is handled by the non-specialized member function above
    if (ndims[0] == dims[0]) {
      for_int(y, ndims[0]) {
        for_int(x, dims[1] / 2) ngrid[y][x] = (grid[y][x * 2 + 0] + grid[y][x * 2 + 1]) * .5f;
        if (dims[1] % 2 == 1) ngrid[y][ndims[1] - 1] = grid[y][(ndims[1] - 1) * 2] * 1.f;  // .5f or 1.f; don't care
      }
    } else if (ndims[1] == dims[1]) {
      for_int(x, ndims[1]) {
        for_int(y, dims[0] / 2) ngrid[y][x] = (grid[y * 2 + 0][x] + grid[y * 2 + 1][x]) * .5f;
        if (dims[0] % 2 == 1) ngrid[ndims[0] - 1][x] = grid[(ndims[0] - 1) * 2][x] * 1.f;  // .5f or 1.f; don't care
      }
    } else {
      parallel_for_each({uint64_t(dims[1]) * 4}, range(dims[0] / 2), [&](const int y) {
        for_int(x, dims[1] / 2) {
          ngrid[y][x] = ((grid[y * 2 + 0][x * 2 + 0] + grid[y * 2 + 0][x * 2 + 1] + grid[y * 2 + 1][x * 2 + 0] +
                          grid[y * 2 + 1][x * 2 + 1]) *
                         .25f);
        }
      });
      if (dims[0] % 2 == 1) {
        int y = ndims[0] - 1;
        float fac = dims[0] >= dims[1] ? .25f : .5f;  // border-zero or reflected
        for_int(x, dims[1] / 2) ngrid[y][x] = (grid[y * 2][x * 2 + 0] + grid[y * 2][x * 2 + 1]) * fac;
      }
      if (dims[1] % 2 == 1) {
        int x = ndims[1] - 1;
        float fac = dims[1] >= dims[0] ? .25f : .5f;  // border-zero or reflected
        for_int(y, dims[0] / 2) ngrid[y][x] = (grid[y * 2 + 0][x * 2] + grid[y * 2 + 1][x * 2]) * fac;
      }
      if (dims[0] % 2 == 1 && dims[1] % 2 == 1) {
        int y = ndims[0] - 1, x = ndims[1] - 1;
        ngrid[y][x] = grid[y * 2][x * 2] * .25f;  // .25f or 1.f; don't care
      }
    }
    return ngrid;
  }
  // Box filter on dual grid; each dimension becomes 2X, unless specified by destsize (which can be 2X-1).
  Grid<D, T> dual_upsample(CGridView<D, T> grid, const Vec<int, D>* destdims = nullptr) {
    return dual_upsample_aux(Specialize<Z>{}, grid, destdims);
  }
  template <int DD> Grid<D, T> dual_upsample_aux(Specialize<DD>, CGridView<D, T> grid, const Vec<int, D>* destdims) {
    HH_MULTIGRID_TIMER("_upsample");
    const Vec<int, D> dims = grid.dims();
    const Vec<int, D> ndims = destdims ? *destdims : dims * 2;
    if (0) SHOW(dims, ndims);
    for_int(c, D) assertx(ndims[c] == dims[c] * 2 || ndims[c] == dims[c] * 2 - 1);
    Grid<D, T> ngrid(ndims);
    // transpose of box filter
    // for (const auto& u : range(ndims)) ngrid[u] = grid[u/2];
    parallel_for_coords(ndims, [&](const Vec<int, D>& u) { ngrid[u] = grid[u / 2]; });
    return ngrid;
  }
  Grid<D, T> dual_upsample_aux(Specialize<2>, CGridView<D, T> grid, const Vec<int, D>* destdims) {
    HH_MULTIGRID_TIMER("_upsample2");
    const Vec<int, D> dims = grid.dims();
    const Vec<int, D> ndims = destdims ? *destdims : dims * 2;
    assertx(ndims[0] == dims[0] * 2 || ndims[0] == dims[0] * 2 - 1);
    assertx(ndims[1] == dims[1] * 2 || ndims[1] == dims[1] * 2 - 1);
    Grid<D, T> ngrid(ndims);
    // transpose of box filter: tensor({(1 0), (0 1)})
    parallel_for_each({uint64_t(ndims[1]) * 1}, range(ndims[0]), [&](const int y) {  //
      for_int(x, ndims[1]) ngrid[y][x] = grid[y / 2][x / 2];
    });
    return ngrid;
  }
  // Return the Laplacian weight at the given grid resolution.
  float get_wL(const Vec<int, D>& gdims) {
    float h = 1.f;
    Vec<int, D> dims = _grid_rhs.dims();
    for (;;) {
      for_int(c, D) assertx(dims[c] >= gdims[c]);
      if (dims == gdims) break;
      assertx(max(dims) > 1);
      const Vec<int, D> ndims = (dims + 1) / 2;
      h = h * 2.f;
      dims = ndims;
    }
    // 1/h^2 due to discretization of second derivative, as in Multigrid Tutorial
    float wL = square(1.f / h);
    return wL;
  }
  // Perform niter iterations of Gauss-Seidel relaxation, possibly with extra iterations near
  //   the ends of the grid for odd grid dimensions.
  void relax(CGridView<D, T> grid_rhs, GridView<D, T> grid_result, int niter, bool extra) {
    relax_aux(Specialize<Z>{}, grid_rhs, grid_result, niter, extra);
  }
  template <int DD>
  void relax_aux(Specialize<DD>, CGridView<D, T> grid_rhs, GridView<D, T> grid_result, int niter, bool extra) {
    HH_MULTIGRID_TIMER("_relax");
    assertx(same_size(grid_rhs, grid_result));
    const Vec<int, D> dims = grid_rhs.dims();
    if (product(dims) == 1) {
      Warning("relax of singleteon");
      return;
    }
    const float wL = get_wL(dims), rwLnum = 1.f / ((2.f * D * wL) + _screening_weight);
    // const bool is_finest = same_size(grid_rhs, _grid_rhs);
    // atomic<int64_t> g_nfast{0}, g_nslow{0};
    const auto func_update = [&](const Vec<int, D>& u) {  // Gauss-Seidel update of value at u
      // ++g_nslow;
      T vnei;
      my_zero(vnei);
      float vnum = _screening_weight;  // or 0.f
      const Vec<int, D> dims2 = dims;
      for_int(c, D) {
        float w = _metric(wL, c);
        bool b = _periodic(c);
        if (u[c] > 0) {
          vnei += w * grid_result[u.with(c, u[c] - 1)];
          vnum += w;
        } else if (b) {
          vnei += w * grid_result[u.with(c, dims2[c] - 1)];
          vnum += w;
        }
        if (u[c] < dims2[c] - 1) {
          vnei += w * grid_result[u.with(c, u[c] + 1)];
          vnum += w;
        } else if (b) {
          vnei += w * grid_result[u.with(c, 0)];
          vnum += w;
        }
      }
      grid_result[u] = (vnei - grid_rhs[u]) / vnum;
    };
    const Vec<int, D> ar_interior_offsets = generate_interior_offsets(dims);
    const auto func_update_interior = [&](size_t i) {
      // added "true &&" to prevent taking a reference to the constexpr
      // added "if (1)" to avoid warnings about unreachable code
      if (1) ASSERTX(true && b_default_metric);
      // ++g_nfast;
      T vnei;
      my_zero(vnei);
      if (0) {
        for (int o : ar_interior_offsets) vnei += grid_result.flat(i + o) + grid_result.flat(i - o);
      } else if (D <= 3) {
        // get rid of &vnei lambda capture which forces stack allocation on VS2013
        if (D > 0) {
          int o = ar_interior_offsets[0];
          vnei += grid_result.flat(i + o) + grid_result.flat(i - o);
        }
        if (D > 1) {
          int o = ar_interior_offsets[1];
          vnei += grid_result.flat(i + o) + grid_result.flat(i - o);
        }
        if (D > 2) {
          int o = ar_interior_offsets[2];
          vnei += grid_result.flat(i + o) + grid_result.flat(i - o);
        }
      } else {
        unroll<2 * D>([&](int j) {
          int o = ar_interior_offsets[j];
          vnei += grid_result.flat(i + o) + grid_result.flat(i - o);
        });
      }
      grid_result.flat(i) = (vnei * wL - grid_rhs.flat(i)) * rwLnum;  // OPT:relax
    };
    for_int(iter, niter) {
      if (0 || (grid_rhs.size() * 10 < k_parallel_thresh && 1)) {
        if (1 && b_default_metric) {
          for_coordsL_interior(dims, ntimes<D>(0), dims, func_update, func_update_interior);
        } else {
          for (const auto& u : range(dims)) func_update(u);
        }
      } else if (1 && D >= 2 && D <= 4) {  // even-odd parallelism on dim0; hypercolumns on dim > 0; fast
        assertx(D <= 4);                   // large D would make col_dims too small
        const int overlap = 0;             // amount to extend each side of column to obtain overlapping Gauss-Seidel
        const Vec<int, D> voverlap = ntimes<D>(overlap);
        Vec<int, D> col_dims;
        {                                                 // hypercolumn dimensions
          const int L2_cache_size = 4 * 1024 * 1024 / 8;  // conservatively assume 4 MiB shared among 8 threads
          const int num_grids = 3 + 1, fudge = 4;         // 3 rows of grid_result, grid_rhs, plus some extra
          const int col_width = int(pow(float(L2_cache_size / sizeof(T)) / (num_grids + fudge), 1.f / (D - 1.0001f)));
          col_dims = ntimes<D>(col_width).with(0, std::numeric_limits<int>::max());
        }
        // even-odd in just first dimension
        const Vec<int, D> even_odd = ntimes<D>(1).with(0, 2);  // { 2, 1, 1, ... }
        int nthreads = get_max_threads();
        col_dims[0] = max((dims[0] - 1) / (nthreads * 2) + 1, 1);
        Vec<int, D> num_col_pairs = (dims - 1) / (col_dims * even_odd) + 1;
        col_dims = ((dims - 1) / num_col_pairs + 1 - 1) / even_odd + 1;  // adjust col_dims for most uniform partition
        // SHOW(dims, col_dims, even_odd, num_col_pairs);
        const bool local_iter = true;
        for (const auto& eo : range(even_odd)) {  // { 0|1, 0, 0, ... }
          // SHOW(eo);
          const auto func_relax_column = [&](const Vec<int, D>& coli) {
            Vec<int, D> uL = general_clamp((coli * even_odd + eo + 0) * col_dims - voverlap, ntimes<D>(0), dims);
            Vec<int, D> uU = general_clamp((coli * even_odd + eo + 1) * col_dims + voverlap, ntimes<D>(0), dims);
            // { std::lock_guard<std::mutex> lock(s_mutex); SHOW(dims, uL, uU); }
            for_int(iter2, local_iter ? niter : 1) {  // implement as streaming?
              for_coordsL_interior(dims, uL, uU, func_update, func_update_interior);
            }
          };
          parallel_for_coords(num_col_pairs, func_relax_column);
        }
        if (local_iter) break;
      } else {  // parallelism across dim0 blocks; two-stage synchronization to preserve determinism
        int nthreads = get_max_threads();
        const int sync_rows = 1;  // rows per chunk to omit in first pass to avoid synchronization issues
        int dim0 = dims[0], d0chunk = max((dims[0] - 1) / nthreads + 1, sync_rows * 2);
        nthreads = (dim0 + d0chunk - 1) / d0chunk;
        parallel_for_each(range(nthreads), [&](const int thread) {  // k_parallel_thresh already tested above
          const Vec<int, D> uL = ntimes<D>(0).with(0, thread * d0chunk);
          const Vec<int, D> uU = dims.with(0, min((thread + 1) * d0chunk, dim0) - sync_rows);
          if (1 && b_default_metric) {
            // { std::lock_guard<std::mutex> lock(s_mutex); SHOW(dims, uL, uU); }
            for_coordsL_interior(dims, uL, uU, func_update, func_update_interior);
          } else {
            for (const auto& u : range(uL, uU)) func_update(u);
          }
        });
        // parallel is useful for small dimension likes video frames
        parallel_for_each(range(nthreads), [&](const int thread) {
          const int overlap = 0;  // = {1, 2} does not seem to help much over = 0.
          const Vec<int, D> uL = ntimes<D>(0).with(0, min((thread + 1) * d0chunk, dim0) - sync_rows);
          const Vec<int, D> uU = dims.with(0, min((thread + 1) * d0chunk + overlap, dim0));
          // for (const auto& u : range(uL, uU)) func_update(u);
          if (b_default_metric) {
            for_coordsL_interior(dims, uL, uU, func_update, func_update_interior);
          } else {
            for (const auto& u : range(uL, uU)) func_update(u);
          }
        });
      }
    }
    if (extra && 1) {  // perform additional relaxations near ends of dimensions with odd sizes
      // We also tried using custom relaxation kernels near end boundaries but it did not work well.
      for_int(iter, 3) {
        int extra_niter = 30, extra_size = 6;
        if (D == 3) {  // reduce computational cost on Video
          extra_niter = 5;
          extra_size = 3;
        }
        for_int(c, D) {
          if (!(dims[c] > 1 && dims[c] % 2 == 1)) continue;
          for_int(extra_iter, extra_niter) {
            const Vec<int, D> uL = ntimes<D>(0).with(c, max(dims[c] - extra_size, 0));
            const Vec<int, D> uU = dims;
            if (b_default_metric) {
              for_coordsL_interior(dims, uL, uU, func_update, func_update_interior);
            } else {
              for (const auto& u : range(uL, uU)) func_update(u);
            }
          }
        }
      }
    }
    // SHOW(g_nfast, g_nslow);
  }
  void relax_aux(Specialize<2>, CGridView<D, T> grid_rhs, GridView<D, T> grid_result, int niter, bool extra) {
    HH_MULTIGRID_TIMER("_relax2");
    assertx(same_size(grid_rhs, grid_result));
    const Vec<int, D> dims = grid_rhs.dims();
    int ny = dims[0], nx = dims[1];
    const float wL = get_wL(dims), rwL4 = 1.f / (4.f * wL + _screening_weight);
    const auto func_update = [&](int y, int x) {
      T vnei;
      my_zero(vnei);
      float w, vnum = _screening_weight;  // or 0.f
      w = _metric(wL, 0);
      if (y > 0) {
        vnei += w * grid_result[y - 1][x];
        vnum += w;
      } else if (_periodic(0)) {
        vnei += w * grid_result[ny - 1][x];
        vnum += w;
      }
      if (y < ny - 1) {
        vnei += w * grid_result[y + 1][x];
        vnum += w;
      } else if (_periodic(0)) {
        vnei += w * grid_result[0][x];
        vnum += w;
      }
      w = _metric(wL, 1);
      if (x > 0) {
        vnei += w * grid_result[y][x - 1];
        vnum += w;
      } else if (_periodic(1)) {
        vnei += w * grid_result[y][nx - 1];
        vnum += w;
      }
      if (x < nx - 1) {
        vnei += w * grid_result[y][x + 1];
        vnum += w;
      } else if (_periodic(1)) {
        vnei += w * grid_result[y][0];
        vnum += w;
      }
      grid_result[y][x] = (vnei - grid_rhs[y][x]) / vnum;
    };
    const auto func_update_interior = [&](int y, int x) {
      if (1) ASSERTX(true && b_default_metric);
      grid_result[y][x] = (((grid_result[y - 1][x + 0] + grid_result[y + 1][x + 0] + grid_result[y + 0][x - 1] +
                             grid_result[y + 0][x + 1]) *
                                wL -
                            grid_rhs[y][x]) *
                           rwL4);  // OPT:relax2
    };
    for_int(iter, niter) {
      if (0 || (grid_rhs.size() * 10 < k_parallel_thresh && 1)) {  // simple sequential version
        for_int(y, ny) for_int(x, nx) func_update(y, x);
      } else {  // two-stage row-based synchronization to preserve determinism
        int nthreads = get_max_threads();
        const int sync_rows = 1;  // rows per chunk to omit in first pass to avoid synchronization issues
        int ychunk = max((ny - 1) / nthreads + 1, sync_rows * 2);
        nthreads = (ny + ychunk - 1) / ychunk;
        parallel_for_each(range(nthreads), [&](const int thread) {
          const int y0 = thread * ychunk, yn = min((thread + 1) * ychunk, ny) - sync_rows;
          if (0) for_intL(y, y0, yn) for_int(x, nx) func_update(y, x);
          if (1 && b_default_metric) {
            for_2DL_interior(y0, yn, 0, nx, func_update, func_update_interior);
          } else {
            for_2DL(y0, yn, 0, nx, func_update);
          }
          // mingw 4096 4096: for_intL: 0.58 sec* for_2DL_interior: 0.64 sec  for_2DL: 0.83 sec
          // win   4096 4096: for_intL: 1.84 sec  for_2DL_interior: 1.19 sec* for_2DL: 1.83 sec
        });
        parallel_for_each({uint64_t(nx * size_t{sync_rows} * 10 / nthreads)}, range(nthreads), [&](const int thread) {
          const int overlap = 0;  // = {1, 2} does not seem to help much over = 0.
          int y0 = min((thread + 1) * ychunk, ny) - sync_rows, yn = min((thread + 1) * ychunk + overlap, ny);
          for_2DL(y0, yn, 0, nx, func_update);
        });
      }
      if (extra && 1) {  // perform additional relaxations near ends of dimensions with odd sizes
        const int extra_niter = 30;
        const int extra_size = 6;
        if (ny > 1 && ny % 2 == 1)
          for_int(extra_iter, extra_niter) for_2DL(max(ny - extra_size, 0), ny, 0, nx, func_update);
        if (nx > 1 && nx % 2 == 1)
          for_int(extra_iter, extra_niter) for_2DL(0, ny, max(nx - extra_size, 0), nx, func_update);
      }
    }
  }
  // Tried implementing the specialization relax_aux(Specialize<1>, ...) but it was no faster for mingw 4.8.1.
  //
  // Compute the residual of the Laplacian:  grid_rhs - Laplacian * grid_result .
  Grid<D, T> compute_residual(CGridView<D, T> grid_rhs, CGridView<D, T> grid_result) {
    return compute_residual_aux(Specialize<Z>{}, grid_rhs, grid_result);
  }
  template <int DD>
  Grid<D, T> compute_residual_aux(Specialize<DD>, CGridView<D, T> grid_rhs, CGridView<D, T> grid_result) {
    HH_MULTIGRID_TIMER("_compute_residual");
    assertx(same_size(grid_rhs, grid_result));
    const Vec<int, D> dims = grid_rhs.dims();
    const float wL = get_wL(dims);
    Grid<D, T> grid_residual(dims);
    const auto func = [&](const Vec<int, D>& u) {
      T vnei;
      my_zero(vnei);
      float vnum = _screening_weight;  // or 0.f
      for_int(c, D) {
        float w = _metric(wL, c);
        bool b = _periodic(c);
        if (u[c] > 0) {
          vnei += w * grid_result[u.with(c, u[c] - 1)];
          vnum += w;
        } else if (b) {
          vnei += w * grid_result[u.with(c, dims[c] - 1)];
          vnum += w;
        }
        if (u[c] < dims[c] - 1) {
          vnei += w * grid_result[u.with(c, u[c] + 1)];
          vnum += w;
        } else if (b) {
          vnei += w * grid_result[u.with(c, 0)];
          vnum += w;
        }
      }
      grid_residual[u] = grid_rhs[u] - (vnei - vnum * grid_result[u]);  // residual of Laplacian
    };
    const Vec<int, D> ar_interior_offsets = generate_interior_offsets(dims);
    const auto func_interior = [&](size_t i) {
      if (1) ASSERTX(true && b_default_metric);
      T vnei;
      my_zero(vnei);
      if (0) {
        for (int o : ar_interior_offsets) vnei += grid_result.flat(i + o) + grid_result.flat(i - o);
      } else {
        if (D > 0) {
          int o = ar_interior_offsets[0];
          vnei += grid_result.flat(i + o) + grid_result.flat(i - o);
        }
        if (D > 1) {
          int o = ar_interior_offsets[1];
          vnei += grid_result.flat(i + o) + grid_result.flat(i - o);
        }
        if (D > 2) {
          int o = ar_interior_offsets[2];
          vnei += grid_result.flat(i + o) + grid_result.flat(i - o);
        }
        static_assert(D <= 3);
      }
      float vnum = _screening_weight + wL * (2.f * D);
      grid_residual.flat(i) = grid_rhs.flat(i) - (wL * vnei - vnum * grid_result.flat(i));  // OPT:resid
    };
    if (0)
      for (const auto& u : range(dims)) func(u);
    if (1 && b_default_metric) {
      parallel_d0_for_coordsL_interior(dims, ntimes<D>(0), dims, func, func_interior);
    } else {
      parallel_for_coords(dims, func);
    }
    return grid_residual;
  }
  Grid<D, T> compute_residual_aux(Specialize<2>, CGridView<D, T> grid_rhs, CGridView<D, T> grid_result) {
    HH_MULTIGRID_TIMER("_compute_residual2");
    assertx(same_size(grid_rhs, grid_result));
    const Vec<int, D> dims = grid_rhs.dims();
    int ny = dims[0], nx = dims[1];
    const float wL = get_wL(dims), wL4 = (_screening_weight + wL * 4.f);
    Grid<D, T> grid_residual(dims);
    const auto func = [&](int y, int x) {
      T vnei;
      my_zero(vnei);
      float w, vnum = _screening_weight;  // or 0.f
      w = _metric(wL, 0);
      if (y > 0) {
        vnei += w * grid_result[y - 1][x];
        vnum += w;
      } else if (_periodic(0)) {
        vnei += w * grid_result[ny - 1][x];
        vnum += w;
      }
      if (y < ny - 1) {
        vnei += w * grid_result[y + 1][x];
        vnum += w;
      } else if (_periodic(0)) {
        vnei += w * grid_result[0][x];
        vnum += w;
      }
      w = _metric(wL, 1);
      if (x > 0) {
        vnei += w * grid_result[y][x - 1];
        vnum += w;
      } else if (_periodic(1)) {
        vnei += w * grid_result[y][nx - 1];
        vnum += w;
      }
      if (x < nx - 1) {
        vnei += w * grid_result[y][x + 1];
        vnum += w;
      } else if (_periodic(1)) {
        vnei += w * grid_result[y][0];
        vnum += w;
      }
      grid_residual[y][x] = grid_rhs[y][x] - (vnei - vnum * grid_result[y][x]);
    };
    const auto func_interior = [grid_result, wL, wL4, grid_rhs, &grid_residual](int y, int x) {
      if (1) ASSERTX(true && b_default_metric);
      T vnei = (grid_result[y - 1][x + 0] + grid_result[y + 1][x + 0] + grid_result[y + 0][x - 1] +
                grid_result[y + 0][x + 1]);
      grid_residual[y][x] = grid_rhs[y][x] - (wL * vnei - wL4 * grid_result[y][x]);  // OPT:resid2
    };
    dummy_use(func_interior);
    // VS2012: does not inline lambda in any case; all similar; first choice is slightly better.
    // gcc4.8.1: always produces good code (even creating func_interior() automatically).
    if (1) parallel_for_each({uint64_t(nx) * 10}, range(ny), [&](const int y) { for_int(x, nx) func(y, x); });
    if (0) parallel_for_2DL(0, ny, 0, nx, func);
    if (0) parallel_for_2DL_interior(0, ny, 0, nx, func, func_interior);
    return grid_residual;
  }
  // Creates solution with unconstrained mean.
  void run_direct_solver(CGridView<D, T> grid_rhs, GridView<D, T> grid_result) {
    // just run many iterations of Gauss-Seidel
    assertx(grid_result.size() <= 16);
    relax(grid_rhs, grid_result, 100, false);
  }
  // Print statistics at each V-cycle iteration, including error if the exact original solution is known.
  void analyze_error(string s) {
    HH_MULTIGRID_TIMER("_analyze");
    // Stat stat(s, true); stat.set_rms(); for (auto e : grid_result - _grid_orig) stat.enter(mag_e(e));
    Precise mean_result = mean(_grid_result);
    double rms_resid = mag_e(rms(compute_residual(_grid_rhs, _grid_result)));
    double rms_err =
        !have_orig() ? 0 : mag_e(rms(_grid_result - _grid_orig + static_cast<T>(_mean_orig - mean_result)));
    double max_abs_err =
        (!have_orig() ? 0
                      : max_e(max_abs_element(_grid_result - _grid_orig + static_cast<T>(_mean_orig - mean_result))));
    string smean_off = !_have_mean_desired ? "" : sform(" smean_off=%-12g", mag_e(mean_result - _mean_desired));
    showf("%9s: mean=%-12g%s rms_resid=%-12g rms_e=%-12g max_e=%g\n",  //
          s.c_str(), mag_e(mean_result), smean_off.c_str(), rms_resid, rms_err, max_abs_err);
  }
  bool coarse_enough(CGridView<D, T> grid_rhs) { return max(grid_rhs.dims()) <= k_direct_solver_resolution; }
  // Recursively perform a multigrid V-cyce.
  void rec_vcycle(CGridView<D, T> grid_rhs, GridView<D, T> grid_result) {
    assertx(same_size(grid_rhs, grid_result));
    static Set<size_t> set_debug;
    if (0 && set_debug.add(grid_rhs.size())) showf("rec_vcycle dims=%s\n", make_string(grid_rhs.dims()).c_str());
    const bool vverbose = 0;
    if (coarse_enough(grid_rhs)) {
      run_direct_solver(grid_rhs, grid_result);
      return;
    }
    double rms0 = vverbose ? mag_e(rms(compute_residual(grid_rhs, grid_result))) : 0.;
    relax(grid_rhs, grid_result, k_num_iter_gauss_seidel, false);
    Grid<D, T> grid_newrhs;
    double rms1;
    {
      Grid<D, T> grid_residual = compute_residual(grid_rhs, grid_result);
      rms1 = vverbose ? mag_e(rms(grid_residual)) : 0.;
      grid_newrhs = dual_downsample(grid_residual);
      if (0) SHOW(rms(grid_newrhs) / rms(grid_residual));
    }
    Grid<D, T> grid_newresult(grid_newrhs.dims(), T{0});
    const int num_recursions = 1;  // 1 == V-cycle, 2 == W-cycle
    for_int(k, num_recursions) rec_vcycle(grid_newrhs, grid_newresult);
    Grid<D, T> grid_correction = dual_upsample(grid_newresult, &grid_result.dims());
    {
      HH_MULTIGRID_TIMER("_add_correction");
      grid_result += grid_correction;
    }
    double rms2 = vverbose ? mag_e(rms(compute_residual(grid_rhs, grid_result))) : 0.;
    relax(grid_rhs, grid_result, k_num_iter_gauss_seidel, true);
    double rms3 = vverbose ? mag_e(rms(compute_residual(grid_rhs, grid_result))) : 0.;
    if (vverbose)
      showf(" resy=%-7d  rms0=%-12.7e rms1=%-12.7e   rms2=%-12.7e rms3=%-12.7e\n",  //
            grid_rhs.dim(0), rms0, rms1, rms2, rms3);
  }
  // Perform a sequence of multigrid V-cycles.
  void run_multigrid(CGridView<D, T> grid_rhs, GridView<D, T> grid_result) {
    HH_MULTIGRID_TIMER("multigrid");
    assertx(same_size(grid_rhs, grid_result));
    // TODO: implement a streaming multigrid algorithm (faster and less memory usage)
    if (getenv_bool("MULTIGRID_VERBOSE")) _verbose = true;
    bool is_power_of_2 = true;
    for_int(c, D) {
      if (!is_pow2(_grid_rhs.dim(c))) is_power_of_2 = false;
    }
    if (0) ASSERTX(mag_e(rms(grid_result)) == 0.);  // good starting state for numerical accuracy?
    if (!_num_vcycles) _num_vcycles = k_default_num_vcycles;
    for_int(vcycle, _num_vcycles) {
      {
        HH_MULTIGRID_TIMER("vcycle");
        rec_vcycle(grid_rhs, grid_result);
      }
      if (0) grid_result -= static_cast<T>(mean(grid_result));  // does not help reducing rms(err)
      if (!b_fastest && (1 || !is_power_of_2) && _have_mean_desired) {
        // for odd grid sizes, mean value may drift significantly due to inaccurate Galerkin condition
        grid_result += static_cast<T>(_mean_desired - mean(grid_result));
      } else {
        // staying near the original mean value of zero is generally OK
      }
      if (_verbose) analyze_error(sform("vcycle%-2d", vcycle + 1));  // relatively slow
    }
    if (_have_mean_desired) grid_result += static_cast<T>(_mean_desired - mean(grid_result));
    if (_verbose) analyze_error("Finalerr");
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MULTIGRID_H_
