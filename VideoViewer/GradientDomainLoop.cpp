// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "VideoViewer/GradientDomainLoop.h"

#include "libHh/ArrayOp.h"  // sort_unique()
#include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"       // file_exists()
#include "libHh/GridOp.h"       // scale_filter_nearest(), scale()
#include "libHh/GridPixelOp.h"  // spatially_scale_Grid3_Pixel()
#include "libHh/Image.h"
#include "libHh/Multigrid.h"
#include "libHh/Parallel.h"
#include "libHh/Queue.h"
#include "libHh/Timer.h"
using namespace hh;

// *** loop creation based on gradient-domain optimization

// TODO: solve Multigrid using streaming, and skip downsampling + relaxation away from discontinuities.

static const int verbose = 0;

namespace hh {

namespace {

inline void write_video(CGridView<3, Pixel> grid, const string& filename) {  // for debugging
  Video video;
  video = grid;
  video.attrib().framerate = 30.;
  video.attrib().bitrate = 100000000;
  video.attrib().suffix = "mp4";
  video.write_file(filename);
}

// was 1e-4f but that proved a tiny bit too weak on australiaclouds2
const float screening_weight = getenv_float("SCREENING_WEIGHT", 1e-3f, true);  // weak screening

// Determine how fast each pixel advances during the loop.
auto compute_deltatime(CMatrixView<int> mat_period, int nnf) {
  Matrix<float> mat_deltatime(mat_period.dims());
  parallel_for_coords(mat_period.dims(), [&](const Vec2<int>& yx) {
    int period = mat_period[yx];
    float deltatime = get_deltatime(period, nnf);
    if (period == 1) deltatime = 0.f;
    mat_deltatime[yx] = deltatime;
  });
  return mat_deltatime;
}

// Determine input frame for each output pixel.
auto compute_framei(Vec3<int> dims, const Matrix<float>& mat_deltatime, CMatrixView<int> mat_start,
                    CMatrixView<int> mat_period) {
  Grid<3, short> grid_framei(dims);
  int nnf = dims[0], ny = dims[1], nx = dims[2];
  Vec2<int> sdims = dims.tail<2>();
  assertx(sdims == mat_start.dims());
  assertx(sdims == mat_period.dims());
  for_int(f, nnf) {
    parallel_for_coords(V(ny, nx), [&](const Vec2<int>& yx) {
      grid_framei[f][yx] = narrow_cast<short>(get_framei(f * mat_deltatime[yx], mat_start[yx], mat_period[yx]));
    });
  }
  return grid_framei;
}

const float k_vec_zero_offset = .5f;

// Depending on parameter V4, access either all image channels (z ignored) or individual image channels (z = 0..2).
template <bool V4> struct MG_sample;
template <> struct MG_sample<true> {
  using EType = Vector4;
  static constexpr int nz = 1;
  static EType get(const Pixel& pix, int z) {
    ASSERTX(z == 0);
    return Vector4(pix);
  }
  static void put(Pixel& pix, int z, const EType& result) {
    ASSERTX(z == 0);
    pix = result.pixel();
  }
  static const float k_offset_zero;
};
const float MG_sample<true>::k_offset_zero = k_vec_zero_offset;

template <> struct MG_sample<false> {
  using EType = float;
  static constexpr int nz = 3;
  static EType get(const Pixel& pix, int z) { return to_float(pix[z]); }
  static void put(Pixel& pix, int z, const EType& result) { pix[z] = clamp_to_uint8(int(result + .5f)); }
  static const float k_offset_zero;
};
const float MG_sample<false>::k_offset_zero = k_vec_zero_offset * 255.f;

// Allows iteration over [0, num) using intervals of at most max_size elements.
// If more than one interval is required, creates intervals of uniform size (all <max_size).
class BoundedIntervals {
 public:
  BoundedIntervals(int num, int max_size) : _num(num) {
    ASSERTX(max_size > 0);
    if (_num <= 0) {
      _nintervs = 0;
      _size = -1;  // Dummy value.
      return;
    }
    _nintervs = (_num - 1) / max_size + 1;
    _size = (_num - 1) / _nintervs + 1;
  }
  bool operator*() const { return _interv < _nintervs; }
  void operator++() { ++_interv; }
  int l() const { return _interv * _size; }
  int u() const { return max((_interv + 1) * _size, _num); }

 private:
  int _num;
  int _nintervs;
  int _size;
  int _interv{0};
};

// Traverse a 2D grid using a sequence of bounded-width swaths instead of simple raster-scan order.
#define for_2D_swaths(y, ny, x, nx, xwidth)                             \
  for (BoundedIntervals HH_ID(bi)(nx, xwidth); *HH_ID(bi); ++HH_ID(bi)) \
  for_int(y, ny) for_intL(x, HH_ID(bi).l(), HH_ID(bi).u())

// Given estimated output videoloop, improve its values to better match gradients from the input video,
//  by performing relaxation only at the current (finest) resolution level.
void compute_gdloop_fast_relax(GridView<3, Pixel> videoloop, CGridView<3, Pixel> video, CMatrixView<int> mat_start,
                               CMatrixView<int> mat_period) {
  HH_TIMER("_fast_relax");
  const int onf = video.dim(0), ny = video.dim(1), nx = video.dim(2), nnf = videoloop.dim(0);
  assertx(same_size(video[0], videoloop[0]) && same_size(video[0], mat_start) && same_size(video[0], mat_period));
  const Matrix<float> mat_deltatime = compute_deltatime(mat_period, nnf);
  Matrix<short> mat_framei(ny, nx);
  Matrix<Vector4> mat_rhs(ny, nx);
  for_int(f, nnf) {  // update (f, *, *); mp0 has [f]; mt0 has [f]-[fm1]; mat_framei0 has [f]
    const int fm1 = f > 0 ? f - 1 : nnf - 1;
    const int fp1 = f < nnf - 1 ? f + 1 : 0;
    CMatrixView<float> lmat_deltatime(mat_deltatime);
    {
      HH_STIMER("_fast_relax1");  // compute the input frame mat_framei for all pixels at output frame f
      CMatrixView<int> lmat_start(mat_start);
      CMatrixView<int> lmat_period(mat_period);
      parallel_for_coords(V(ny, nx), [&](const Vec2<int>& yx) {
        mat_framei[yx] = narrow_cast<short>(get_framei(lmat_deltatime[yx] * f, lmat_start[yx], lmat_period[yx]));
      });
    }
    {
      HH_STIMER("_fast_relax2");  // compute the right-hand-side of the multigrid system for the output frame f
      auto func_stitch = [](CGridView<3, Pixel> video2, CMatrixView<Pixel> videofi0, const Vector4& pix0, int fi0,
                            int fi1, int y0, int x0, int y1, int x1, Vector4& vrhs) {
        Vector4 t = Vector4(videofi0(y1, x1)) - pix0;
        vrhs += fi0 == fi1 ? t : (t + (Vector4(video2(fi1, y1, x1)) - Vector4(video2(fi1, y0, x0)))) * .5f;
      };
      parallel_for_each(range(ny), [&](const int y) {
        for_int(x, nx) {
          int fi = mat_framei(y, x);
          CMatrixView<Pixel> videofi = video[fi];
          Vector4 pixv(videofi(y, x));
          Vector4 vrhs(-screening_weight * pixv);
          const int start = mat_start(y, x), period = mat_period(y, x);
          int fm1i = get_framei(lmat_deltatime(y, x) * fm1, start, period);
          if (fm1i <= fi) {
            vrhs += (Vector4(video(fm1i, y, x)) - pixv);
          } else {
            int ft = fm1i - period;
            if (ft >= 0) vrhs += (Vector4(video(ft, y, x)) - pixv) * .5f;
            ft = fi + period;
            if (ft < onf) vrhs += (Vector4(video(fm1i, y, x)) - Vector4(video(ft, y, x))) * .5f;
          }
          int fp1i = get_framei(lmat_deltatime(y, x) * fp1, start, period);
          if (fp1i >= fi) {  // OPT:fast_relax_simple
            vrhs += (Vector4(video(fp1i, y, x)) - pixv);
          } else {
            int ft = fp1i + period;
            if (ft < onf) vrhs += (Vector4(video(ft, y, x)) - pixv) * .5f;
            ft = fi - period;
            if (ft >= 0) vrhs += (Vector4(video(fp1i, y, x)) - Vector4(video(ft, y, x))) * .5f;
          }
          if (y > 0) func_stitch(video, videofi, pixv, fi, mat_framei(y - 1, x), y, x, y - 1, x, vrhs);
          if (y < ny - 1) func_stitch(video, videofi, pixv, fi, mat_framei(y + 1, x), y, x, y + 1, x, vrhs);
          if (x > 0) func_stitch(video, videofi, pixv, fi, mat_framei(y, x - 1), y, x, y, x - 1, vrhs);
          if (x < nx - 1) func_stitch(video, videofi, pixv, fi, mat_framei(y, x + 1), y, x, y, x + 1, vrhs);
          mat_rhs(y, x) = vrhs;
        }
      });
    }
    {
      HH_STIMER("_fast_relax3");  // perform niter Gauss-Seidel iterations on frame f using rhs computed above
      const int niter = 2;
      const float wL = 1.f, w = wL, rwL6 = 1.f / (screening_weight + 6.f * w);
      //
      MatrixView<Pixel> mat_videoloopf(videoloop[f]);
      auto func_update = [&](int y, int x) {
        float vnum = screening_weight;
        Vector4 vnei = w * (Vector4(videoloop(fp1, y, x)) + Vector4(videoloop(fm1, y, x)));
        vnum += w * 2.f;
        if (y > 0) {
          vnei += w * Vector4(mat_videoloopf(y - 1, x));
          vnum += w;
        }
        if (y + 1 < ny) {
          vnei += w * Vector4(mat_videoloopf(y + 1, x));
          vnum += w;
        }
        if (x > 0) {
          vnei += w * Vector4(mat_videoloopf(y, x - 1));
          vnum += w;
        }
        if (x + 1 < nx) {
          vnei += w * Vector4(mat_videoloopf(y, x + 1));
          vnum += w;
        }
        Vector4 result = (vnei - mat_rhs(y, x)) / vnum;
        mat_videoloopf(y, x) = result.pixel();
      };
      auto func_update_interior = [&](int y, int x) {
        Vector4 vnei = (Vector4(videoloop(fp1, y, x)) + Vector4(videoloop(fm1, y, x)) +
                        Vector4(mat_videoloopf(y - 1, x)) + Vector4(mat_videoloopf(y + 1, x)) +
                        Vector4(mat_videoloopf(y, x - 1)) + Vector4(mat_videoloopf(y, x + 1)));
        Vector4 result = (vnei - mat_rhs(y, x)) * rwL6;  // OPT:fast_relax_interior
        mat_videoloopf(y, x) = result.pixel();
      };
      for_int(iter, niter) {
        int nthreads = get_max_threads();
        const int sync_rows = 1;  // rows per chunk to omit in first pass to avoid synchronization issues
        int ychunk = max((ny - 1) / nthreads + 1, sync_rows * 2);
        nthreads = (ny + ychunk - 1) / ychunk;
        parallel_for_each(range(nthreads), [&](const int thread) {  // pass 1
          const int y0 = thread * ychunk, yn = min((thread + 1) * ychunk, ny) - sync_rows;
          if (0) for_intL(y, y0, yn) for_int(x, nx) func_update(y, x);
          for_2DL_interior(y0, yn, 0, nx, func_update, func_update_interior);
        });
        for_int(thread, nthreads) {  // pass 2
          int y0 = min((thread + 1) * ychunk, ny) - sync_rows, yn = min((thread + 1) * ychunk, ny);
          for_2DL(y0, yn, 0, nx, func_update);
        }
      }
    }
  }
}

// Given input video (and looping parameters mat_start, mat_period), compute a videoloop using multigrid.
// If have_est, perform relaxation only at the finest resolution (starting with estimate already in videoloop).
template <bool have_est, bool V4>
void compute_gdloop_aux2(CGridView<3, Pixel> video, CMatrixView<int> mat_start, CMatrixView<int> mat_period,
                         GridView<3, Pixel> videoloop, bool b_exact) {
  using MG = MG_sample<V4>;
  using EType = typename MG_sample<V4>::EType;  // "MG::EType" fails on gcc 4.7.3
  const int onf = video.dim(0), ny = video.dim(1), nx = video.dim(2), nnf = videoloop.dim(0);
  assertx(same_size(video[0], videoloop[0]) && same_size(video[0], mat_start) && same_size(video[0], mat_period));
  Vec3<int> dims(nnf, ny, nx);
  Grid<3, short> grid_framei;
  {
    Matrix<float> mat_deltatime = compute_deltatime(mat_period, nnf);
    grid_framei = compute_framei(dims, mat_deltatime, mat_start, mat_period);
  }
  for_int(z, MG::nz) {
    HH_TIMER("_mgcompute");
    // no special weighting of temporal differences vs. spatial differences
    Multigrid<3, EType, MultigridPeriodicTemporally> multigrid(dims);
    if (0) {  // simpler, slightly slower code
      HH_TIMER("__setup_rhs");
      // speedup: for each f, stream rows: extract Matrix<EType> and compute difference values on edges
      auto func_stitch = [](CGridView<3, Pixel> video2, CMatrixView<Pixel> videofi0, const EType& pix0, int fi0,
                            int fi1, int y0, int x0, int y1, int x1, int zz, EType& vrhs) {
        EType t = MG::get(videofi0(y1, x1), zz) - pix0;
        vrhs += fi0 == fi1 ? t : (t + (MG::get(video2(fi1, y1, x1), zz) - MG::get(video2(fi1, y0, x0), zz))) * .5f;
      };
      parallel_for_each(range(nnf), [&](const int f) {
        CMatrixView<short> grid_frameif = grid_framei[f];
        MatrixView<EType> mrhs = multigrid.rhs()[f];
        MatrixView<EType> mest = multigrid.initial_estimate()[f];
        const int fm1 = f > 0 ? f - 1 : nnf - 1;
        const int fp1 = f < nnf - 1 ? f + 1 : 0;
        // const int max_xwidth = 10000;  // because data fits in L2 cache, it is detrimental to break it up
        // for_2D_swaths(y, ny, x, nx, max_xwidth) {
        for_int(y, ny) for_int(x, nx) {
          int fi = grid_frameif(y, x);
          CMatrixView<Pixel> videofi = video[fi];
          EType pixv = MG::get(videofi(y, x), z);
          EType vrhs(-screening_weight * pixv);
          const int period = mat_period(y, x);
          // It is important to use grid_framei rather than deltat (e.g. brink2s loop example).
          int fm1i = grid_framei(fm1, y, x);
          if (fm1i <= fi) {
            vrhs += (MG::get(video(fm1i, y, x), z) - pixv);
          } else {
            int ft = fm1i - period;
            if (ft >= 0) vrhs += (MG::get(video(ft, y, x), z) - pixv) * .5f;
            ft = fi + period;
            if (ft < onf) vrhs += (MG::get(video(fm1i, y, x), z) - MG::get(video(ft, y, x), z)) * .5f;
          }
          int fp1i = grid_framei(fp1, y, x);
          if (fp1i >= fi) {
            vrhs += (MG::get(video(fp1i, y, x), z) - pixv);
          } else {
            int ft = fp1i + period;
            if (ft < onf) vrhs += (MG::get(video(ft, y, x), z) - pixv) * .5f;
            ft = fi - period;
            if (ft >= 0) vrhs += (MG::get(video(fp1i, y, x), z) - MG::get(video(ft, y, x), z)) * .5f;
          }
          if (y > 0) func_stitch(video, videofi, pixv, fi, grid_frameif(y - 1, x), y, x, y - 1, x, z, vrhs);
          if (y < ny - 1) func_stitch(video, videofi, pixv, fi, grid_frameif(y + 1, x), y, x, y + 1, x, z, vrhs);
          if (x > 0) func_stitch(video, videofi, pixv, fi, grid_frameif(y, x - 1), y, x, y, x - 1, z, vrhs);
          if (x < nx - 1) func_stitch(video, videofi, pixv, fi, grid_frameif(y, x + 1), y, x, y, x + 1, z, vrhs);
          mrhs(y, x) = vrhs;  // OPT:rhs_simple
          if (!have_est) mest(y, x) = pixv;
        }
      });
    } else {
      HH_TIMER("__setup_rhs2");  // surprisingly, only a little faster than the simpler version above
      parallel_for_each(range(nnf), [&](const int f) {
        CMatrixView<short> grid_frameif = grid_framei[f];
        MatrixView<EType> mrhs = multigrid.rhs()[f];
        MatrixView<EType> mest = multigrid.initial_estimate()[f];
        const int fm1 = f > 0 ? f - 1 : nnf - 1;
        const int fp1 = f < nnf - 1 ? f + 1 : 0;
        Array<EType> apix0(nx), apix1(nx), asy0(nx), asy1(nx), asx(nx + 1);
        asx[0] = asx[nx] = EType{0};
        const int xwidth = 0 ? 600 : 100000;  // breaking up into swaths is actually detrimental
        for (BoundedIntervals bi(nx, xwidth); *bi; ++bi) {
          int xl = bi.l(), xu = bi.u();
          for_intL(x, xl, min(xu + 1, nx)) apix0[x] = MG::get(video(grid_frameif(0, x), 0, x), z);
          for_intL(x, xl, xu) { asy0[x] = EType{0}; }
          for_int(y, ny) {  // update [y][x]; apix0 has [y]; asy0 has [y] - [y-1]
            int y1 = y + 1;
            if (y1 < ny) apix1[xl] = MG::get(video(grid_frameif(y1, xl), y1, xl), z);
            if (xl > 0 && xl < xu) {
              int fi1 = grid_frameif(y, xl), fi2 = grid_frameif(y, xl - 1);
              asx[xl] = (MG::get(video(fi1, y, xl), z) - MG::get(video(fi1, y, xl - 1), z) +
                         MG::get(video(fi2, y, xl), z) - MG::get(video(fi2, y, xl - 1), z)) *
                        .5f;
            }
            for_intL(x, xl, xu) {  // apix1[x] has [y1][x]; asx[xl] has [y][xl] - [y][xl - 1]
              int x1 = x + 1;
              int fi = grid_frameif(y, x);
              if (x1 < nx) {
                if (y1 < ny) apix1[x1] = MG::get(video(grid_frameif(y1, x1), y1, x1), z);
                // compute asx[x1] = [y][x1] - [y][x]
                int fi01 = grid_frameif(y, x1);
                asx[x1] =
                    (fi01 == fi
                         ? apix0[x1] - apix0[x]
                         : (apix0[x1] - MG::get(video(fi01, y, x), z) + MG::get(video(fi, y, x1), z) - apix0[x]) *
                               .5f);
              }
              if (y1 < ny) {  // compute asy1[x] = [y1][x] - [y][x]
                int fi10 = grid_frameif(y1, x);
                asy1[x] =
                    (fi10 == fi
                         ? apix1[x] - apix0[x]
                         : (apix1[x] - MG::get(video(fi10, y, x), z) + MG::get(video(fi, y1, x), z) - apix0[x]) * .5f);
              } else {
                asy1[x] = EType{0};
              }
              // update pixel [y][x] using asy0, asy1, asx, apix0
              EType pixv = apix0[x];
              EType vrhs(-screening_weight * pixv);
              const int period = mat_period(y, x);
              // It is important to use grid_framei rather than deltat (e.g. brink2s loop example).
              int fm1i = grid_framei(fm1, y, x);
              if (fm1i <= fi) {
                vrhs += (MG::get(video(fm1i, y, x), z) - pixv);
              } else {
                int ft = fm1i - period;
                if (ft >= 0) vrhs += (MG::get(video(ft, y, x), z) - pixv) * .5f;
                ft = fi + period;
                if (ft < onf) vrhs += (MG::get(video(fm1i, y, x), z) - MG::get(video(ft, y, x), z)) * .5f;
              }
              int fp1i = grid_framei(fp1, y, x);
              if (fp1i >= fi) {
                vrhs += (MG::get(video(fp1i, y, x), z) - pixv);
              } else {
                int ft = fp1i + period;
                if (ft < onf) vrhs += (MG::get(video(ft, y, x), z) - pixv) * .5f;
                ft = fi - period;
                if (ft >= 0) vrhs += (MG::get(video(fp1i, y, x), z) - MG::get(video(ft, y, x), z)) * .5f;
              }
              vrhs += asy1[x] - asy0[x] + asx[x1] - asx[x];
              mrhs(y, x) = vrhs;
              if (!have_est) mest(y, x) = pixv;  // OPT:rhs_complex
            }
            swap(apix0, apix1);
            swap(asy0, asy1);
          }
        }
      });
    }
    if (have_est) {
      HH_TIMER("_videoloop_est");
      parallel_for_each(range(nnf), [&](const int f) {
        MatrixView<EType> mest = multigrid.initial_estimate()[f];
        CMatrixView<Pixel> mest2 = videoloop[f];
        for_int(y, ny) for_int(x, nx) mest(y, x) = MG::get(mest2(y, x), z);
      });
    }
    if (0) multigrid.set_desired_mean(mean(multigrid.initial_estimate()));  // use screening_weight instead
    multigrid.set_screening_weight(screening_weight);  // small errors on brink2s loop with screening == 0
    int num_vcycles = 3;                               // 20140725 tried changing from 3 to 1, and it looks OK
    if (b_exact) num_vcycles = 10;
    if (1) multigrid.set_num_vcycles(num_vcycles);
    if (verbose >= 2) multigrid.set_verbose(true);
    const int niter = 3;
    if (have_est) {
      assertx(!b_exact);
      HH_TIMER("__just_relax");
      multigrid.just_relax(niter);
    } else {
      HH_TIMER("__solve");
      multigrid.solve();
    }
    HH_TIMER("__put");
    CGridView<3, EType> grid_result = multigrid.result();
    parallel_for_each(range(nnf), [&](const int f) {
      for_int(y, ny) for_int(x, nx) MG::put(videoloop(f, y, x), z, grid_result(f, y, x));
    });
  }
  if (0) {
    Grid<3, Pixel> ref_video(nnf, ny, nx);
    Grid<3, Pixel> diff_video(nnf, ny, nx);
    parallel_for_each(range(nnf), [&](const int f) {
      for_int(y, ny) for_int(x, nx) {
        int fi = grid_framei(f, y, x);
        ref_video(f, y, x) = video(fi, y, x);
        for_int(z, 3) {
          diff_video(f, y, x)[z] = clamp_to_uint8(128 + (videoloop(f, y, x)[z] - ref_video(f, y, x)[z]) * 5);
        }
      }
    });
    write_video(ref_video, "ref_video.wmv");
    write_video(videoloop, "videoloop.wmv");
    write_video(diff_video, "diff_video.wmv");
  }
}

Matrix<int> possibly_rescale(CMatrixView<int> mat, const Vec2<int>& sdims) {
  assertx(min(mat.dims()) > 0);
  assertx(is_zero(mat.dims() % sdims) || is_zero(sdims % mat.dims()));
  Matrix<int> nmat = mat.dims() == sdims ? Matrix<int>(mat) : scale_filter_nearest(mat, sdims);
  return nmat;
}

// Given input video (and looping parameters mat_start, mat_period), compute a videoloop using multigrid.
// Solve for offsets rather than final colors.
template <bool V4>
void solve_using_offsets_aux(CGridView<3, Pixel> video, CMatrixView<int> mat_start, CMatrixView<int> mat_period,
                             GridView<3, Pixel> video_offset) {
  using MG = MG_sample<V4>;
  using EType = typename MG_sample<V4>::EType;
  dummy_use(MG_sample<false>::k_offset_zero, MG_sample<true>::k_offset_zero);
  const int onf = video.dim(0), ny = video.dim(1), nx = video.dim(2), nnf = video_offset.dim(0);
  assertx(same_size(video[0], video_offset[0]) && same_size(video[0], mat_start) && same_size(video[0], mat_period));
  Vec3<int> dims(nnf, ny, nx);
  Grid<3, short> grid_framei;
  {
    Matrix<float> mat_deltatime = compute_deltatime(mat_period, nnf);
    grid_framei = compute_framei(dims, mat_deltatime, mat_start, mat_period);
    // HH_RSTAT(Sdeltatime, mat_deltatime);
    // HH_RSTAT(Sgrid_framei, grid_framei);
  }
  ConsoleProgress cprogress("Multigrid solver");
  for_int(z, MG::nz) {
    cprogress.update(float(z) / MG::nz);
    HH_TIMER("_mgcompute");
    // no special weighting of temporal differences vs. spatial differences
    Multigrid<3, EType, MultigridPeriodicTemporally> multigrid(dims);
    Timer timer_setup_rhs("__setup_rhs");
    if (0) {  // slower reference implementation
      auto func_stitch = [](CGridView<3, Pixel> video2, CMatrixView<Pixel> videofi0, const EType& pix0, int fi0,
                            int fi1, int y0, int x0, int y1, int x1, int zz, EType& vrhs) {
        if (fi0 == fi1) return;
        const EType& A = pix0;
        const EType& B = MG::get(videofi0(y1, x1), zz);
        const EType& C = MG::get(video2(fi1, y0, x0), zz);
        const EType& D = MG::get(video2(fi1, y1, x1), zz);
        // initial difference is D - A,  desired difference is ((B - A) / 2 + (D - C) / 2)
        vrhs += (A + B - C - D) * .5f;  // desired change
      };
      parallel_for_each(range(nnf), [&](const int f) {
        CMatrixView<short> grid_frameif = grid_framei[f];
        MatrixView<EType> mrhs = multigrid.rhs()[f];
        fill(multigrid.initial_estimate()[f], EType{0});
        const int fm1 = f > 0 ? f - 1 : nnf - 1;
        const int fp1 = f < nnf - 1 ? f + 1 : 0;
        for_int(y, ny) for_int(x, nx) {
          int fi = grid_frameif(y, x);
          CMatrixView<Pixel> videofi = video[fi];
          EType pixv = MG::get(videofi(y, x), z);
          EType vrhs(0.f);
          const int period = mat_period(y, x);
          // It is important to use grid_framei rather than deltat (e.g. brink2s loop example).
          int fm1i = grid_framei(fm1, y, x);
          if (fm1i > fi) {
            int count = 0;
            EType v(0.f);
            int ft = fm1i - period;
            if (ft >= 0) {
              count++;
              v += (MG::get(video(ft, y, x), z) - MG::get(video(fm1i, y, x), z));
            }
            ft = fi + period;
            if (ft < onf) {
              count++;
              v += (pixv - MG::get(video(ft, y, x), z));
            }
            assertx(count > 0);
            if (count == 2) v *= .5f;
            vrhs += v;
          }
          int fp1i = grid_framei(fp1, y, x);
          if (fp1i < fi) {
            int count = 0;
            EType v(0.f);
            int ft = fp1i + period;
            if (ft < onf) {
              count++;
              v += (MG::get(video(ft, y, x), z) - MG::get(video(fp1i, y, x), z));
            }
            ft = fi - period;
            if (ft >= 0) {
              count++;
              v += (pixv - MG::get(video(ft, y, x), z));
            }
            assertx(count > 0);
            if (count == 2) v *= .5f;
            vrhs += v;
          }
          if (y > 0) func_stitch(video, videofi, pixv, fi, grid_frameif(y - 1, x), y, x, y - 1, x, z, vrhs);
          if (y < ny - 1) func_stitch(video, videofi, pixv, fi, grid_frameif(y + 1, x), y, x, y + 1, x, z, vrhs);
          if (x > 0) func_stitch(video, videofi, pixv, fi, grid_frameif(y, x - 1), y, x, y, x - 1, z, vrhs);
          if (x < nx - 1) func_stitch(video, videofi, pixv, fi, grid_frameif(y, x + 1), y, x, y, x + 1, z, vrhs);
          mrhs(y, x) = vrhs;  // OPT:rhs_simple
        }
      });
    } else {  // Speed up the above by instead traversing just the discontinuities.
      // This initialization of mest is actually slower than computing the rhs!
      parallel_for_each(range(nnf), [&](const int f) {
        fill(multigrid.initial_estimate()[f], EType{0});  // OPT:fill
      });
      parallel_for_each(range(nnf), [&](const int f) { fill(multigrid.rhs()[f], EType{0}); });
      // Find temporal discontinuities.
      const int nphase = nnf % 2 ? 3 : 2;  // run in phases to avoid race condition
      for_int(iphase, nphase) {
        // (With just two phases, race condition exists on frame 0 in iphase == 0 if nnf is odd.)
        parallel_for_each(range((nnf - 1 - iphase) / nphase + 1), [&](const int hf) {
          int f0 = hf * nphase + iphase;
          int f1 = (f0 + 1) % nnf;
          CMatrixView<short> grid_frameif0 = grid_framei[f0];
          CMatrixView<short> grid_frameif1 = grid_framei[f1];
          MatrixView<EType> mrhs0 = multigrid.rhs()[f0];
          MatrixView<EType> mrhs1 = multigrid.rhs()[f1];
          for_int(y, ny) for_int(x, nx) {
            int yxi = y * nx + x;
            int fi0 = grid_frameif0.flat(yxi);
            int fi1 = grid_frameif1.flat(yxi);
            if (fi1 >= fi0) continue;
            const int period = mat_period.flat(yxi);  // OPT:rhs_temporal
            int count = 0;
            EType change(0.f);
            if (fi1 + period < onf) {
              count++;
              change += MG::get(video[fi1 + period].flat(yxi), z) - MG::get(video[fi1].flat(yxi), z);
            }
            if (fi0 - period >= 0) {
              count++;
              change += MG::get(video[fi0].flat(yxi), z) - MG::get(video[fi0 - period].flat(yxi), z);
            }
            assertx(count > 0);
            if (count == 2) change *= .5f;
            mrhs0.flat(yxi) += change;
            mrhs1.flat(yxi) -= change;
          }
        });
      }
      // Find spatial discontinuities.
      for_int(iphase, 2) {  // run in two phases to avoid race conditions
        parallel_for_each(range((ny - 1 - iphase) / 2 + 1), [&](const int yh) {
          for_int(x, nx) {
            int y = yh * 2 + iphase;
            Vec2<int> yx = V(y, x);
            for (auto yxd : {V(+1, 0), V(0, +1)}) {
              auto yxn = yx + yxd;
              if (!video[0].ok(yxn) || (mat_start[yx] == mat_start[yxn] && mat_period[yx] == mat_period[yxn]))
                continue;
              CStridedArrayView<short> ar0fi = grid_column<0>(grid_framei, concat(V(0), yx));
              CStridedArrayView<short> ar1fi = grid_column<0>(grid_framei, concat(V(0), yxn));
              CStridedArrayView<Pixel> ar0pix = grid_column<0>(video, concat(V(0), yx));
              CStridedArrayView<Pixel> ar1pix = grid_column<0>(video, concat(V(0), yxn));
              StridedArrayView<EType> ar0rhs = grid_column<0>(multigrid.rhs(), concat(V(0), yx));
              StridedArrayView<EType> ar1rhs = grid_column<0>(multigrid.rhs(), concat(V(0), yxn));
              for_int(f, nnf) {
                int fi0 = ar0fi[f];
                int fi1 = ar1fi[f];
                if (fi0 == fi1) continue;
                EType A = MG::get(ar0pix[fi0], z), B = MG::get(ar1pix[fi0], z);
                EType C = MG::get(ar0pix[fi1], z), D = MG::get(ar1pix[fi1], z);
                // initial difference is D - A,  desired difference is ((B - A) / 2 + (D - C) / 2)
                EType change = (A + B - C - D) * .5f;  // desired change; OPT:rhs_spatial
                ar0rhs[f] += change;
                ar1rhs[f] -= change;
              }
            }
          }
        });
      }
    }
    timer_setup_rhs.terminate();
    if (0) multigrid.set_desired_mean(EType{0});       // use screening_weight instead
    multigrid.set_screening_weight(screening_weight);  // small errors on brink2s loop with screening == 0
    if (1) multigrid.set_num_vcycles(1);
    if (verbose >= 2) multigrid.set_verbose(true);
    {
      HH_TIMER("__solve");
      multigrid.solve();
    }
    HH_TIMER("__put");
    CGridView<3, EType> grid_result = multigrid.result();
    parallel_for_each(range(nnf), [&](const int f) {
      for_int(y, ny) for_int(x, nx) MG::put(video_offset(f, y, x), z, grid_result(f, y, x) + MG::k_offset_zero);
    });
  }
  if (!V4) {
    const EType k_offset_zero{MG::k_offset_zero};  // to avoid warning of redundant cast below
    parallel_for_each(range(nnf), [&](const int f) {
      for_int(y, ny) for_int(x, nx) MG::put(video_offset(f, y, x), 3, k_offset_zero);
    });
  }
}

// Given input video (and looping parameters mat_start, mat_period), compute a videoloop using multigrid.
// Solve for offsets rather than final colors.
void solve_using_offsets(const Vec3<int>& odims, const string& video_filename, CGridView<3, Pixel> video,
                         CVideoNv12View video_nv12, CMatrixView<int> mat_start, CMatrixView<int> mat_period, int nnf,
                         WVideo* pwvideo, GridView<3, Pixel> videoloop, VideoNv12View videoloop_nv12, int num_loops) {
  const int onf = odims[0], ny = odims[1], nx = odims[2];
  const Vec2<int> sdims(ny, nx);
  const Vec3<int> ndims(nnf, ny, nx);
  assertx((video_filename != "") + !!video.size() + !!video_nv12.size() == 1);
  assertx(!!pwvideo + !!videoloop.size() + !!videoloop_nv12.size() == 1);
  if (video.size()) assertx(video.dims() == odims);
  if (video_nv12.size()) assertx(video_nv12.get_Y().dims() == odims);
  if (videoloop.size()) assertx(videoloop.dims() == ndims);
  if (0) {  // gather offsets and apply them here
    assertx(video.size() && videoloop.size());
    Matrix<int> mat_start_highres = possibly_rescale(mat_start, sdims);
    Matrix<int> mat_period_highres = possibly_rescale(mat_period, sdims);
    Grid<3, Pixel> video_offset(ndims);
    solve_using_offsets_aux<false>(video, mat_start_highres, mat_period_highres, video_offset);
    parallel_for_each(range(nnf), [&](const int f) {
      for_int(y, ny) for_int(x, nx) {
        videoloop(f, y, x) =
            (Vector4(videoloop(f, y, x)) + Vector4(video_offset(f, y, x)) - k_vec_zero_offset).pixel();
      }
    });
    return;
  }
  // Solve offsets at a coarser resolution, then extrapolate them.
  // const int DS = 4;       // spatial downsampling factor (good for 1920x1088 to 480x272)
  // const int DS = ny / 270;
#if 0
  int DS = 1;
  {                                                                         // spatial downsampling factor
    const int videoloops_maxh = getenv_int("VIDEOLOOPS_MAXH", 350, false);  // default value used in Loopers
    int y = ny;
    while (y > videoloops_maxh) {
      assertx(y % 2 == 0);
      y /= 2;
      DS *= 2;
    }
  }
#endif
  const int DS = odims[1] / mat_start.dim(0);
  if (verbose) showdf("Spatial downsampling factor is %d\n", DS);
  if (0) SHOW(odims, mat_start.dims(), sdims, DS);
  assertx(is_zero(sdims % DS));
  const int DT = 1;  // temporal downsampling factor
  assertx(onf % DT == 0);
  // The problem with DT > 1 is that temporal discontinuities generally occur at all frames in fine-scale video
  //  due to get_framei(), but only on even frames when upsampled from coarse-scale correction.
  Nv12 static_frame;
  if (video_filename != "") static_frame.init(sdims);
  const FilterBnd filterb(Filter::get("box"), Bndrule::reflected);
  const int hny = ny / DS, hnx = nx / DS;
  const Vec2<int> hdims(hny, hnx);
  assertx(hdims == sdims / DS);
  Matrix<int> hmat_start = possibly_rescale(mat_start, hdims);
  Matrix<int> hmat_period = possibly_rescale(mat_period, hdims);
  if (1) {  // quick optional debug
    assertx(max_abs_element(scale_filter_nearest(hmat_start, mat_start.dims()) - mat_start) == 0);
    assertx(max_abs_element(scale_filter_nearest(hmat_period, mat_period.dims()) - mat_period) == 0);
  }
  assertx(is_zero(hdims % hmat_start.dims()));
  assertx(is_zero(hdims % hmat_period.dims()));
  Grid<3, Pixel> hvideo(onf / DT, hny, hnx);
  {  // reduced (maybe "half") resolution
    HH_TIMER("__scale_down");
    assertx(DT == 1);
    if (video.size()) {
      spatially_scale_Grid3_Pixel(video, twice(filterb), nullptr, hvideo);
    } else if (video_nv12.size()) {
      for_int(f, onf) integrally_downscale_Nv12_to_Image(video_nv12[f], hvideo[f]);
    } else {
      const bool use_nv12 = true;
      RVideo rvideo(video_filename, use_nv12);  // note that attrib() should already be set in pwvideo
      assertx(rvideo.spatial_dims() == sdims);
      Nv12 frame(sdims);
      ConsoleProgress cprogress("Read and scale down");
      const bool assume_mod2_static_frames = true;
      if (assume_mod2_static_frames) {
        for_int(hy, hny) for_int(hx, hnx) {
          if (hmat_period(hy, hx) == 1) assertx(hmat_start(hy, hx) % 4 == 2);
        }
      }
      for_int(f, onf) {
        cprogress.update(float(f) / onf);
        assertx(rvideo.read(frame));
        const int DSh = DS / 2;
        assertx(DS == 1 || DSh * 2 == DS);
        integrally_downscale_Nv12_to_Image(frame, hvideo[f]);
        if (assume_mod2_static_frames && f % 4 != 2) continue;
        parallel_for_each(range(hny), [&](const int hy) {
          for_int(hx, hnx) {
            if (hmat_period(hy, hx) != 1 || hmat_start(hy, hx) != f) continue;
            for_intL(y, hy * DS, hy * DS + DS) for_intL(x, hx * DS, hx * DS + DS) {
              static_frame.get_Y()(y, x) = frame.get_Y()(y, x);
            }
            if (DS > 1) {
              for_intL(y, hy * DSh, hy * DSh + DSh) for_intL(x, hx * DSh, hx * DSh + DSh) {
                static_frame.get_UV()(y, x) = frame.get_UV()(y, x);
              }
            } else {
              if (hy % 2 + hx % 2 == 0) static_frame.get_UV()(hy / 2, hx / 2) = frame.get_UV()(hy / 2, hx / 2);
            }
          }
        });
      }
      // if (0) as_image(static_frame).write_file("image_static.png");
    }
  }
  if (0) {
    write_video(hvideo, "hvideo.mp4");
    exit(1);
  }
  if (DT > 1) {
    assertx(max(mat_start % DT) == 0);
    hmat_start /= DT;
    hmat_period = (hmat_period + (DT - 1)) / DT;  // ensure that period == 1 remains 1
  }
  Grid<3, Pixel> hvideo_offset(nnf / DT, hny, hnx);
  {  // half-resolution loop
    HH_TIMER("__solve_offsets");
    if (video_filename == "" && !getenv_bool("VIDEOLOOP_USE_LITTLE_MEMORY") &&
        available_memory() > assert_narrow_cast<size_t>(product(hvideo_offset.dims()) * sizeof(Vector4) * 6))
      solve_using_offsets_aux<true>(hvideo, hmat_start, hmat_period, hvideo_offset);  // faster but more memory
    else
      solve_using_offsets_aux<false>(hvideo, hmat_start, hmat_period, hvideo_offset);
    hvideo.clear();  // no longer needed
  }
  // Note: get the gd computation out of the timings.
  const bool disable_write = getenv_bool("VIDEOLOOP_DISABLE_WRITE");
  if (video_filename == "") {
    // In this easier case, one can randomly gather content from the input video grid.
    if (0) {  // Use box filter upsampling to directly create output video.
      HH_TIMER("__recon1");
      assertx(video.size() && videoloop.size());
      Grid<3, short> grid_framei;
      {
        HH_TIMER("___grid_framei");
        Matrix<float> mat_deltatime = compute_deltatime(mat_period, nnf);
        Matrix<int> mat_start_highres = possibly_rescale(mat_start, sdims);
        Matrix<int> mat_period_highres = possibly_rescale(mat_period, sdims);
        grid_framei = compute_framei(ndims, mat_deltatime, mat_start_highres, mat_period_highres);
      }
      parallel_for_each(range(nnf), [&](const int f) {
        for_int(y, ny) for_int(x, nx) {
          int fi = grid_framei(f, y, x);
          const Pixel& pix = video(fi, y, x);
          videoloop(f, y, x) =
              (Vector4(pix) + Vector4(hvideo_offset(f / DT, y / DS, x / DS)) - k_vec_zero_offset).pixel();
        }
      });
    } else if (video.size()) {  // faster version of above, for RGB representation
      HH_TIMER("__recon2");
      Matrix<float> hmat_deltatime = compute_deltatime(hmat_period, nnf);
      Queue<Matrix<Pixel>> queue_frames;
      Matrix<Pixel> sframe;
      if (videoloop_nv12.size()) sframe.init(sdims);
      ConsoleProgress cprogress("Video assembly");
      if (disable_write && pwvideo) {
        pwvideo->write(Grid<2, Pixel>(sdims, Pixel::white()));
      }
      for_int(iloop, num_loops) {
        for_int(f, nnf) {
          cprogress.update(float(f) / nnf);
          if (pwvideo) queue_frames.enqueue(Matrix<Pixel>{sdims});
          MatrixView<Pixel> nframe(videoloop.size()        ? videoloop[f]
                                   : videoloop_nv12.size() ? sframe
                                                           : queue_frames.rear());
          parallel_for_each(range(hny), [&](const int hy) {
            for_int(hx, hnx) {
              int fi = get_framei(f * hmat_deltatime(hy, hx), hmat_start(hy, hx), hmat_period(hy, hx));
              Vector4i offset = Vector4i(hvideo_offset(f / DT, hy, hx)) - 128;
              auto videofi = video[fi];
              auto lnframe = nframe;  // local view to help optimizer
              for_intL(y, hy * DS, hy * DS + DS) for_intL(x, hx * DS, hx * DS + DS) {
                const Pixel& pix = videofi(y, x);
                Pixel& npix = lnframe(y, x);
                npix = (Vector4i(pix) + offset).pixel();  // OPT:recon2
              }
            }
          });
          // Note: using pwvideo stream is actually slower than saving to memory and then writing to
          //  disk all at once, probably because of memory thrashing, so buffer two frames at a time.
          const int buffer_nframes = 2;
          if (pwvideo && (queue_frames.length() >= buffer_nframes || f == nnf - 1)) {  // flush
            while (!queue_frames.empty()) {
              if (disable_write) {
                queue_frames.dequeue();
              } else {
                pwvideo->write(queue_frames.dequeue());
              }
            }
          }
          if (videoloop_nv12.size()) convert_Image_to_Nv12(sframe, videoloop_nv12[f]);
        }
      }
    } else {  // Nv12 YUV representation
      assertx(video_nv12.size());
      if (DS / 2 * 2 != DS) {
        Warning("Looping parameters computed at higher resolution than UV grid in Nv12 representation");
        // Convert input video from NV12 to RGB and reinvoke this function.
        Video tvideo(odims);
        convert_VideoNv12_to_Video(video_nv12, tvideo);
        solve_using_offsets(odims, video_filename, tvideo, VideoNv12{}, mat_start, mat_period, nnf, pwvideo, videoloop,
                            videoloop_nv12, num_loops);
        return;
      }
      HH_TIMER("__recon2_nv12");
      Matrix<float> hmat_deltatime = compute_deltatime(hmat_period, nnf);
      Queue<Nv12> queue_frames;
      Nv12 sframe;
      if (videoloop.size()) sframe.init(sdims);
      ConsoleProgress cprogress("Video assembly");
      const Vector4i YUV_zero_offset(RGB_to_YUV_Vector4i(128, 128, 128));
      // SHOW(YUV_zero_offset);  // Pixel(126, 128, 128, 255)
      const int DSh = DS / 2;
      assertx(DSh * 2 == DS);
      for_int(iloop, num_loops) {
        for_int(f, nnf) {
          cprogress.update(float(f) / nnf);
          if (pwvideo) queue_frames.enqueue(Nv12{sdims});
          Nv12View nframe(videoloop_nv12.size() ? videoloop_nv12[f] : videoloop.size() ? sframe : queue_frames.rear());
          if (0) {
            parallel_for_each(range(hny), [&](const int hy) {
              for_int(hx, hnx) {
                int fi = get_framei(f * hmat_deltatime(hy, hx), hmat_start(hy, hx), hmat_period(hy, hx));
                const Pixel& poff = hvideo_offset(f / DT, hy, hx);
                Vector4i offset_YUV = RGB_to_YUV_Vector4i(poff[0], poff[1], poff[2]) - YUV_zero_offset;
                for_intL(y, hy * DS, hy * DS + DS) for_intL(x, hx * DS, hx * DS + DS) {
                  nframe.get_Y()(y, x) = clamp_to_uint8(video_nv12.get_Y()(fi, y, x) + offset_YUV[0]);
                }
                for_intL(y, hy * DSh, hy * DSh + DSh) for_intL(x, hx * DSh, hx * DSh + DSh) for_int(c, 2) {
                  nframe.get_UV()(y, x)[c] = clamp_to_uint8(video_nv12.get_UV()(fi, y, x)[c] + offset_YUV[1 + c]);
                }
              }
            });
          } else {
            parallel_for_each(range(hny), [&](const int hy) {
              auto hmat_deltatimehy = hmat_deltatime[hy];
              auto hmat_starthy = hmat_start[hy];
              auto hmat_periodhy = hmat_period[hy];
              auto hvideo_offsethy = hvideo_offset[f / DT][hy];
              auto lvideo_nv12_Y = video_nv12.get_Y();
              auto lvideo_nv12_UV = video_nv12.get_UV();
              for_int(hx, hnx) {
                int fi = get_framei(f * hmat_deltatimehy[hx], hmat_starthy[hx], hmat_periodhy[hx]);
                const Pixel& poff = hvideo_offsethy[hx];
                Vector4i offset_YUV = RGB_to_YUV_Vector4i(poff[0], poff[1], poff[2]) - YUV_zero_offset;
                auto video_nv12_Yfi = lvideo_nv12_Y[fi];
                for_intL(y, hy * DS, hy * DS + DS) {
                  auto* nframeYy = nframe.get_Y()[y].data();
                  const auto* video_nv12_Yy = video_nv12_Yfi[y].data();
                  for_intL(x, hx * DS, hx * DS + DS) {
                    nframeYy[x] = clamp_to_uint8(video_nv12_Yy[x] + offset_YUV[0]);  // OPT:nv12Y
                  }
                }
                auto video_nv12_UVfi = lvideo_nv12_UV[fi];
                for_intL(y, hy * DSh, hy * DSh + DSh) {
                  auto* nframeUVy = nframe.get_UV()[y].data();
                  const auto* video_nv12_UVy = video_nv12_UVfi[y].data();
                  for_intL(x, hx * DSh, hx * DSh + DSh) {
                    nframeUVy[x][0] = clamp_to_uint8(video_nv12_UVy[x][0] + offset_YUV[1]);
                    nframeUVy[x][1] = clamp_to_uint8(video_nv12_UVy[x][1] + offset_YUV[2]);
                  }
                }
              }
            });
          }
          const int buffer_nframes = 2;
          if (pwvideo && (queue_frames.length() >= buffer_nframes || f == nnf - 1)) {  // flush
            while (!queue_frames.empty()) {
              if (disable_write) {
                queue_frames.dequeue();
              } else {
                pwvideo->write(queue_frames.dequeue());
              }
            }
          }
          if (videoloop.size()) convert_Nv12_to_Image(sframe, videoloop[f]);
        }
      }
    }
    return;
  }
  // Multi-input-stream reconstruction (lower memory usage but more I/O)
  // Process 4K video in just 1.9 GiB max memory -- much of it is in Media Foundation video stream overhead.
  // FullHD video in 670 MB max memory.
  HH_TIMER("__recon_multistream");
  assertx(video_filename != "");
  assertx(file_requires_pipe(video_filename) || file_exists(video_filename));
  int onf_actual = max(mat_start + mat_period);
  assertx(onf_actual <= onf);
  Array<int> periods = sort_unique(hmat_period);
  if (verbose) SHOW(periods);
  const int max_num_periods = 3;  // one static period (period == 1) and up to 2 looping periods (period > 1)
  assertx(periods.num() <= max_num_periods && periods[0] == 1);
  Matrix<float> hmat_deltatime = compute_deltatime(hmat_period, nnf);
  Array<float> ar_deltatime(periods.num());
  for_int(pi, periods.num()) ar_deltatime[pi] = get_deltatime(periods[pi], nnf);
  if (verbose) SHOW(ar_deltatime);
  Array<int> ar_nstreams(periods.num());
  for_int(pi, periods.num()) ar_nstreams[pi] = (onf_actual + periods[pi] - 1) / periods[pi];
  ar_nstreams[0] = 0;  // static frame is handled separately in pre-pass
  if (verbose) SHOW(ar_nstreams);
  int totstreams = int(sum(ar_nstreams));
  Array<unique_ptr<RVideo>> prvideos(totstreams);  // video streams
  const bool use_nv12 = true;
  Array<Nv12> rvideoframes(totstreams);  // current image frame in each video stream
  for (auto& frame : rvideoframes) frame.init(sdims);
  Array<int> rvideo_fi(totstreams, std::numeric_limits<int>::max());  // current frame index in each video stream
  auto func_get_si = [&](int pi, int streami) {
    ASSERTX(pi >= 1 && ar_nstreams.ok(pi));
    ASSERTX(streami >= 0 && streami < ar_nstreams[pi]);
    int si = 0;
    for_intL(i, 1, pi) si += ar_nstreams[i];
    si += streami;
    ASSERTX(prvideos.ok(si));
    return si;
  };
  Nv12 sframe;
  if (!videoloop_nv12.size()) sframe.init(sdims);
  ConsoleProgress cprogress("Multi-stream video assembly and write");
  // TODO: use seek functionality of IMFSourceReader::SetCurrentPosition() method
  for_int(iloop, num_loops) {
    for_int(f, nnf) {
      cprogress.update(float(f) / nnf);
      if (verbose) showf("For output frame f=%d\n", f);
      // TODO: could let any read stream skip ahead if no content is used from it
      Array<int> ar_fi0(periods.num());  // input frame for first stream of each period
      for_int(pi, periods.num()) {
        if (pi == 0) {
          assertx(periods[pi] == 1);
          continue;  // static frame is handled separately
        }
        int period = periods[pi];
        int wrap = (onf_actual + period - 1) / period * period;
        int nstreams = ar_nstreams[pi];
        ar_fi0[pi] = int(f * ar_deltatime[pi] + .5f);
        // The input frames corresponding to output frame f are ar_fi0[pi] + k * period  (signed k)
        //  with si = my_mod(k, nstreams).
        for_int(streami, nstreams) {
          int si = func_get_si(pi, streami);
          int fi = my_mod(ar_fi0[pi] + streami * period, wrap);
          // SHOW(pi, streami, si, fi, period, wrap, onf_actual);
          // if fi is in range [onf_actual, wrap), the streami is not useful.
          if (fi >= onf_actual) {
            fi = -1;
            if (prvideos[si]) {
              if (verbose) showf(" closing stream\n");
              prvideos[si] = nullptr;
              rvideo_fi[si] = std::numeric_limits<int>::max();
            }
          }
          if (verbose)
            showf(" stream %d (%d of %d for period[%d]==%-3d): input %-3d->%-3d (%d)\n", si, streami, nstreams, pi,
                  period, rvideo_fi[si], fi, fi - rvideo_fi[si]);
          if (fi >= 0 && fi < rvideo_fi[si]) {
            prvideos[si] = make_unique<RVideo>(video_filename, use_nv12);  // open or re-open at beginning
            rvideo_fi[si] = -1;
          }
          while (rvideo_fi[si] < fi) {
            rvideo_fi[si]++;
            if (1 && rvideo_fi[si] < fi) {
              assertx(prvideos[si]->discard_frame());
              continue;
            }
            if (verbose >= 3) showf("  reading stream frame %d\n", rvideo_fi[si]);
            assertx(prvideos[si]->read(rvideoframes[si]));
          }
        }
      }
      Nv12View nframe(videoloop_nv12.size() ? videoloop_nv12[f] : sframe);
      const Vector4i YUV_zero_offset(RGB_to_YUV_Vector4i(128, 128, 128));
      const int DSh = DS / 2;
      assertx(DS == 1 || DSh * 2 == DS);
      parallel_for_each(range(hny), [&](const int hy) {
        for_int(hx, hnx) {
          int period = hmat_period(hy, hx);
          int pi = periods.index(period);
          ASSERTX(pi >= 0);
          int si;
          {
            if (pi == 0) {
              si = -1;  // if period == 1, access static frame
            } else {
              int fi = get_framei(f * ar_deltatime[pi], hmat_start(hy, hx), period);
              int dfi = fi - ar_fi0[pi];
              // if (!(my_mod(dfi, period) == 0)) SHOW(f, hy, hx, period, pi, fi, ar_fi0[pi], dfi);
              ASSERTX(my_mod(dfi, period) == 0);
              int nstreams = ar_nstreams[pi];
              int streami = my_mod(dfi / period, nstreams);
              si = func_get_si(pi, streami);
              // if (rvideo_fi[si] != fi) SHOW(f, hy, hx, period, fi, dfi, streami, si, rvideo_fi);
              ASSERTX(rvideo_fi[si] == fi);
            }
          }
          const Pixel& hpix = hvideo_offset(f / DT, hy, hx);
          MatrixView<uint8_t> nframeY = nframe.get_Y();
          MatrixView<Vec2<uint8_t>> nframeUV = nframe.get_UV();
          const CNv12View videofi(si < 0 ? static_frame : rvideoframes[si]);
          const Vector4i offset_YUV = RGB_to_YUV_Vector4i(hpix[0], hpix[1], hpix[2]) - YUV_zero_offset;
          for_intL(y, hy * DS, hy * DS + DS) for_intL(x, hx * DS, hx * DS + DS) {
            nframeY(y, x) = clamp_to_uint8(videofi.get_Y()(y, x) + offset_YUV[0]);  // OPT:reconY
          }
          if (DS > 1) {
            for_intL(y, hy * DSh, hy * DSh + DSh) for_intL(x, hx * DSh, hx * DSh + DSh) {
              nframeUV(y, x) = V(clamp_to_uint8(videofi.get_UV()(y, x)[0] + offset_YUV[1]),
                                 clamp_to_uint8(videofi.get_UV()(y, x)[1] + offset_YUV[2]));  // OPT:reconU
            }
          } else {
            if (hy % 2 + hx % 2 == 0)
              nframeUV(hy / 2, hx / 2) = V(clamp_to_uint8(videofi.get_UV()(hy / 2, hx / 2)[0] + offset_YUV[1]),
                                           clamp_to_uint8(videofi.get_UV()(hy / 2, hx / 2)[1] + offset_YUV[2]));
          }
        }
      });
      if (pwvideo) pwvideo->write(sframe);
      if (videoloop.size()) convert_Nv12_to_Image(sframe, videoloop[f]);
    }
  }
}

template <bool have_est>
void compute_gdloop_aux1(CGridView<3, Pixel> video, CMatrixView<int> mat_start, CMatrixView<int> mat_period,
                         GridView<3, Pixel> videoloop, bool b_exact) {
  // Multigrid<3, Vector4>(200Mpix) fits in ~12 GB memory  (== 2Mpix(FullHD) * 30fps * ~3.33sec)
  // const bool use_vector4 = getenv_bool("GDLOOP_USE_VECTOR4");
  // if (video.size() > 200 * 1000 * 1000)
  // if (use_vector4)
  if (!getenv_bool("VIDEOLOOP_USE_LITTLE_MEMORY") &&
      available_memory() > assert_narrow_cast<size_t>(product(videoloop.dims()) * sizeof(Vector4) * 6))
    // faster but more memory
    compute_gdloop_aux2<have_est, true>(video, mat_start, mat_period, videoloop, b_exact);
  else
    compute_gdloop_aux2<have_est, false>(video, mat_start, mat_period, videoloop, b_exact);
}

void show_spatial_cost(CGridView<3, Pixel> video, CMatrixView<int> mat_start, CMatrixView<int> mat_period, int nnf) {
  Grid<3, short> grid_framei;
  {
    Matrix<float> mat_deltatime = compute_deltatime(mat_period, nnf);
    grid_framei = compute_framei(V(nnf, video.dim(1), video.dim(2)), mat_deltatime, mat_start, mat_period);
  }
  const int ny = video.dim(1), nx = video.dim(2);
  Image image(V(ny, nx));
  parallel_for_each(range(ny), [&](const int y) {
    for_int(x, nx) {
      float cost = 0.f;
      for_int(f, nnf) {
        for_int(axis, 2) for_int(dir, 2) {
          int y0 = y, x0 = x, y1 = y, x1 = x;
          if (axis == 0) {
            y1 += !dir ? -1 : +1;
            if (y1 < 0 || y1 >= ny) continue;
          } else {
            x1 += !dir ? -1 : +1;
            if (x1 < 0 || x1 >= nx) continue;
          }
          int fi0 = grid_framei(f, y0, x0);
          int fi1 = grid_framei(f, y1, x1);
          for_int(c, 3) {
            cost += square(to_float(video(fi0, y0, x0)[c]) - to_float(video(fi1, y0, x0)[c]));
            cost += square(to_float(video(fi0, y1, x1)[c]) - to_float(video(fi1, y1, x1)[c]));
          }
        }
      }
      cost = max(cost, 1e-10f);
      HH_SSTAT(Scost, cost);
      const float c1 = .12f, c2 = 1.f, gamma = .5f;
      float v = pow(clamp(1.f - std::log(cost) * c1 + c2, 0.f, 1.f), gamma) * 255.f;
      HH_SSTAT(Sv, v);
      image(y, x) = Pixel::gray(clamp_to_uint8(int(v)));
    }
  });
  image.write_file("image_scost.png");
  SHOW("Done writing image_scost");
  exit(0);
}

void compute_costs(CGridView<3, Pixel> video, CGridView<3, Pixel> videoloop, CMatrixView<int> mat_start,
                   CMatrixView<int> mat_period) {
  const int nnf = videoloop.dim(0);
  Grid<3, short> grid_framei;
  {
    Matrix<float> mat_deltatime = compute_deltatime(mat_period, nnf);
    grid_framei = compute_framei(V(nnf, video.dim(1), video.dim(2)), mat_deltatime, mat_start, mat_period);
  }
  const int ny = video.dim(1), nx = video.dim(2);
  // TODO: should have \gamma (both spatial and temporal), and should have weighting of temporal vs. spatial.
  int64_t spatial_nseams = 0;
  double spatial_sum_cost = 0., spatial_sum_seam_cost = 0.;
  for_int(f, nnf) for_int(y, ny) for_int(x, nx) {
    const int y0 = y, x0 = x;
    int fi0 = grid_framei(f, y0, x0);
    for_int(axis, 2) {
      int y1 = y, x1 = x;
      if (axis == 0) {
        y1 += 1;
        if (y1 >= ny) continue;
      } else {
        x1 += 1;
        if (x1 >= nx) continue;
      }
      int fi1 = grid_framei(f, y1, x1);
      bool is_seam = fi1 != fi0;
      float cost = (mag2((Vector4(videoloop(f, y1, x1)) - Vector4(videoloop(f, y0, x0))) -
                         (Vector4(video(fi0, y1, x1)) - Vector4(video(fi0, y0, x0)))) +
                    mag2((Vector4(videoloop(f, y1, x1)) - Vector4(videoloop(f, y0, x0))) -
                         (Vector4(video(fi1, y1, x1)) - Vector4(video(fi1, y0, x0)))));
      spatial_sum_cost += cost;
      if (is_seam) {
        spatial_nseams++;
        spatial_sum_seam_cost += cost;
      }
    }
  }
  SHOW(spatial_nseams, float(spatial_nseams) / (ny * nx * 2) / nnf);
  SHOW(spatial_sum_cost, spatial_sum_seam_cost);
  // rms errors on all pixel pairs and on just on seams
  SHOW(sqrt(spatial_sum_cost / (ny * nx * 2) / nnf / 6));
  SHOW(sqrt(spatial_sum_seam_cost / spatial_nseams / 6));
  int64_t nstatic_pixels = count(mat_period, 1);
  int64_t temporal_nseams = 0;
  double temporal_sum_cost = 0., temporal_sum_seam_cost = 0.;
  for_int(f, nnf - 1) {
    for_int(y, ny) for_int(x, nx) {
      const int f0 = f, f1 = f + 1;
      if (mat_period(y, x) == 1) continue;
      int fi0 = grid_framei(f0, y, x);
      int fi1 = grid_framei(f1, y, x);
      int fid = fi1 - fi0;
      // bool is_seam = fid != 1;
      bool is_seam = abs(fid - 1) > 1;
      ASSERTX(fi1 > 0 && fi0 + 1 < nnf);
      float cost = (mag2((Vector4(videoloop(f1, y, x)) - Vector4(videoloop(f0, y, x))) -
                         (Vector4(video(fi1, y, x)) - Vector4(video(is_seam ? fi1 - 1 : fi0, y, x)))) +
                    mag2((Vector4(videoloop(f1, y, x)) - Vector4(videoloop(f0, y, x))) -
                         (Vector4(video(is_seam ? fi0 + 1 : fi1, y, x)) - Vector4(video(fi0, y, x)))));
      temporal_sum_cost += cost;
      if (is_seam) {
        temporal_nseams++;
        temporal_sum_seam_cost += cost;
      }
    }
  }
  SHOW(nstatic_pixels, float(nstatic_pixels) / (ny * nx));
  SHOW(temporal_nseams, float(temporal_nseams) / (ny * nx - nstatic_pixels));
  SHOW(temporal_sum_cost, temporal_sum_seam_cost);
  // rms errors on all pixel pairs and on just on seams
  SHOW(sqrt(temporal_sum_cost / (ny * nx) / nnf / 6));
  SHOW(sqrt(temporal_sum_seam_cost / temporal_nseams / 6));
  double rms_spatial = sqrt(spatial_sum_cost / (ny * nx * 2) / nnf / 6);
  double rms_temporal = sqrt(temporal_sum_cost / (ny * nx) / nnf / 6);
  double rms_total =
      sqrt((spatial_sum_cost + temporal_sum_cost) / (float(ny * nx * 2) * nnf * 6 + float(ny * nx) * nnf * 6));
  SHOW(rms_spatial, rms_temporal, rms_total);
}

}  // namespace

template <int dyh, int dxh> void integrally_downscale_Nv12_to_Image_aux(CNv12View nv12, MatrixView<Pixel> nmatrixp) {
  const int Dyx2 = dyh * 2 * dxh * 2, Dyxh2 = dyh * dxh;
  parallel_for_each(range(nmatrixp.ysize()), [&](const int y) {
    for_int(x, nmatrixp.xsize()) {
      int sumY = 0;
      Vec2<int> sumUV(0, 0);
      for_int(dy, dyh * 2) {
        auto mat_Yy = nv12.get_Y()[y * dyh * 2 + dy];
        for_int(dx, dxh * 2) sumY += mat_Yy[x * dxh * 2 + dx];
      }
      for_int(dy, dyh) {
        auto mat_UVy = nv12.get_UV()[y * dyh + dy];
        for_int(dx, dxh) {
          sumUV[0] += mat_UVy[x * dxh + dx][0];
          sumUV[1] += mat_UVy[x * dxh + dx][1];  // OPT:mat_UVy2
        }
      }
      sumY = (sumY + Dyx2 / 2) / Dyx2;
      sumUV = (sumUV + Dyxh2 / 2) / Dyxh2;
      nmatrixp(y, x) = YUV_to_RGB_Pixel(sumY, sumUV[0], sumUV[1]);
    }
  });
}

void integrally_downscale_Nv12_to_Image(CNv12View nv12, MatrixView<Pixel> nmatrixp) {
  assertx(nmatrixp.size());
  if (nmatrixp.dims() == nv12.get_Y().dims()) {
    // Special case because Dyx / 2 contains a zero
    convert_Nv12_to_Image(nv12, nmatrixp);
    return;
  }
  const Vec2<int> Dyx = nv12.get_Y().dims() / nmatrixp.dims();
  const Vec2<int> Dyxh = Dyx / 2;
  assertx(Dyxh * 2 == Dyx);
  assertx(nmatrixp.dims() * Dyx == nv12.get_Y().dims());
  if (0) {
  } else if (Dyx == V(2, 2)) {
    integrally_downscale_Nv12_to_Image_aux<1, 1>(nv12, nmatrixp);
  } else if (Dyx == V(4, 4)) {
    integrally_downscale_Nv12_to_Image_aux<2, 2>(nv12, nmatrixp);
  } else if (Dyx == V(8, 8)) {
    integrally_downscale_Nv12_to_Image_aux<4, 4>(nv12, nmatrixp);
  } else {
    const int Dyx2 = Dyx[0] * Dyx[1], Dyxh2 = Dyxh[0] * Dyxh[1];
    parallel_for_each(range(nmatrixp.ysize()), [&](const int y) {
      for_int(x, nmatrixp.xsize()) {
        int sumY = 0;
        Vec2<int> sumUV(0, 0);
        for_int(dy, Dyx[0]) {
          auto mat_Yy = nv12.get_Y()[y * Dyx[0] + dy];
          for_int(dx, Dyx[1]) sumY += mat_Yy[x * Dyx[1] + dx];
        }
        for_int(dy, Dyxh[0]) {
          auto mat_UVy = nv12.get_UV()[y * Dyxh[0] + dy];
          for_int(dx, Dyxh[1]) {
            sumUV[0] += mat_UVy[x * Dyxh[1] + dx][0];
            sumUV[1] += mat_UVy[x * Dyxh[1] + dx][1];  // OPT:mat_UVy
          }
        }
        sumY = (sumY + Dyx2 / 2) / Dyx2;
        sumUV = (sumUV + Dyxh2 / 2) / Dyxh2;
        nmatrixp(y, x) = YUV_to_RGB_Pixel(sumY, sumUV[0], sumUV[1]);
      }
    });
  }
}

void compute_gdloop(const Vec3<int>& videodims, const string& video_filename, CGridView<3, Pixel> video,
                    CVideoNv12View video_nv12, CMatrixView<int> mat_start, CMatrixView<int> mat_period,
                    GdLoopScheme scheme, int nnf, WVideo* pwvideo, GridView<3, Pixel> videoloop,
                    VideoNv12View videoloop_nv12, int num_loops) {
  const Vec3<int> odims = videodims;  // original (input) dimensions;
  Vec2<int> sdims = odims.tail<2>();  // spatial dimensions (for both input and loop)
  assertx(product(sdims));
  const int onf = odims[0];  // number of input frames
  assertx(onf > 0);
  assertx((video_filename != "") + !!video.size() + !!video_nv12.size() == 1);  // exactly one of 3 input types
  if (video_filename != "") assertx(file_requires_pipe(video_filename) || file_exists(video_filename));
  if (video.size()) assertx(video.dims() == odims);
  if (video_nv12.size()) assertx(video_nv12.get_Y().dims() == odims);
  assertx(mat_start.size() && mat_period.size() && mat_start.dims() == mat_period.dims());
  assertx(nnf > 0);                                                        // number of frames in new (loop) video
  const Vec3<int> ndims = concat(V(nnf), sdims);                           // dimensions of new (loop) video
  assertx(!!pwvideo + !!videoloop.size() + !!videoloop_nv12.size() == 1);  // exactly one of 3 output types
  if (pwvideo) assertx(pwvideo->spatial_dims() == sdims);
  if (videoloop.size()) assertx(videoloop.dims() == ndims);
  if (videoloop_nv12.size()) assertx(videoloop_nv12.get_Y().dims() == ndims);
  assertx(num_loops >= 1);
  if (!pwvideo) assertx(num_loops == 1);
  assertx(min(mat_period) > 0);
  for (const Vec2<int> yx : range(mat_start.dims())) {
    assertx(mat_start[yx] >= 0 && mat_start[yx] + mat_period[yx] <= onf);
    if (0) {
      // Assume room at both ends for faster, better gradient-domain blend.
      // Attempt for backward compatibility on old ~/proj/videoloops/data/ReallyFreakinAll/out/HDgiant_loop.vlp
      // However, failed for start = 0 and period = 148.
      if (!assertw(mat_start[yx] > 0)) static_cast<MatrixView<int>&>(mat_start)[yx] = 4;  // const_cast
      if (!assertw(mat_start[yx] + mat_period[yx] < onf)) static_cast<MatrixView<int>&>(mat_start)[yx] -= 4;
      if (!(mat_start[yx] > 0 && mat_start[yx] + mat_period[yx] < onf)) SHOW(yx, mat_start[yx], mat_period[yx]);
      assertx(mat_start[yx] > 0 && mat_start[yx] + mat_period[yx] < onf);
    }
  }
  if (getenv_bool("VIDEOLOOP_PRECISE")) scheme = GdLoopScheme::precise;
  if (getenv_bool("VIDEOLOOP_NO_BLEND")) scheme = GdLoopScheme::no_blend;
  if (getenv_bool("VIDEOLOOP_EXACT")) scheme = GdLoopScheme::exact;
  HH_TIMER("_gdloop");
  if (verbose) showf("Rendering looping video of %d frames.\n", nnf);
  switch (scheme) {
    case GdLoopScheme::no_blend: {
      Matrix<int> mat_start_highres = possibly_rescale(mat_start, sdims);
      Matrix<int> mat_period_highres = possibly_rescale(mat_period, sdims);
      Grid<3, short> grid_framei;
      {
        Matrix<float> mat_deltatime = compute_deltatime(mat_period_highres, nnf);
        grid_framei = compute_framei(ndims, mat_deltatime, mat_start_highres, mat_period_highres);
      }
      const int ny = sdims[0], nx = sdims[1];
      // We support all 3 * 3 possible cases of input and output types.
      if (video.size()) {  // use Image (RGB) representation
        Matrix<Pixel> sframe;
        if (!videoloop.size()) sframe.init(sdims);
        for_int(iloop, num_loops) {
          for_int(f, nnf) {
            auto frame = videoloop.size() ? videoloop[f] : sframe;
            parallel_for_each(range(ny),
                              [&](const int y) { for_int(x, nx) frame(y, x) = video(grid_framei(f, y, x), y, x); });
            if (pwvideo) pwvideo->write(sframe);
            if (videoloop_nv12.size()) convert_Image_to_Nv12(sframe, videoloop_nv12[f]);
          }
        }
      } else {  // use Nv12 (YUV) representation
        VideoNv12 tvideo;
        if (!video_nv12.size()) {
          tvideo.read_file(video_filename);
          assertx(tvideo.get_Y().dims() == odims);
        }
        CVideoNv12View ivideo = video_nv12.size() ? video_nv12 : tvideo;
        Nv12 sframe;
        if (!videoloop_nv12.size()) sframe.init(sdims);
        for_int(iloop, num_loops) {
          for_int(f, nnf) {
            auto frame = videoloop_nv12.size() ? videoloop_nv12[f] : sframe;
            parallel_for_each(range(ny), [&](const int y) {
              for_int(x, nx) {
                const int fi = grid_framei(f, y, x);
                frame.get_Y()(y, x) = ivideo.get_Y()(fi, y, x);
                const int hy = y / 2, hx = x / 2;
                frame.get_UV()(hy, hx) = ivideo.get_UV()(fi, hy, hx);  // if (hy * 2 == y && hx * 2 == x)
              }
            });
            if (pwvideo) pwvideo->write(sframe);
            if (videoloop.size()) convert_Nv12_to_Image(sframe, videoloop[f]);
          }
        }
      }
      break;
    }
    case GdLoopScheme::fast:  // new scheme: solve for offset values at coarse resolution
      solve_using_offsets(odims, video_filename, video, video_nv12, mat_start, mat_period, nnf, pwvideo, videoloop,
                          videoloop_nv12, num_loops);
      break;
    case GdLoopScheme::precise:
    case GdLoopScheme::exact: {
      // Old, slower, more precise scheme
      assertx(video.size() && videoloop.size());  // it does not implement streaming video read, write, or nv12
      Matrix<int> mat_start_highres = possibly_rescale(mat_start, sdims);
      Matrix<int> mat_period_highres = possibly_rescale(mat_period, sdims);
      const bool b_exact = scheme == GdLoopScheme::exact;
      const bool use_halfres = !b_exact;
      if (!use_halfres) {
        compute_gdloop_aux1<false>(video, mat_start_highres, mat_period_highres, videoloop, b_exact);
      } else {  // reduce resolution on two spatial dimensions by a factor two
        // "box" is fastest; previously "spline" and "triangle"
        const FilterBnd filterb(Filter::get("box"), Bndrule::reflected);
        const bool debug = false;
        Timer timer_gdloop1("_gdloop1");
        Grid<3, Pixel> hvideo(video.dim(0), ((video.dim(1) + 3) / 4) * 2,
                              ((video.dim(2) + 3) / 4) * 2);  // half-resolution
        {
          HH_TIMER("__scale_down");
          spatially_scale_Grid3_Pixel(video, twice(filterb), nullptr, hvideo);
        }
        if (debug) write_video(hvideo, "hvideo.mp4");
        const Vec2<int> hdims = hvideo.dims().tail<2>();
        Matrix<int> hmat_start = scale_filter_nearest(mat_start, hdims);    // half-resolution
        Matrix<int> hmat_period = scale_filter_nearest(mat_period, hdims);  // half-resolution
        // HH_RSTAT(Shstart, hmat_start); HH_RSTAT(Shperiod, hmat_period);
        timer_gdloop1.terminate();
        Grid<3, Pixel> hvideoloop(concat(V(videoloop.dim(0)), hdims));  // half-resolution loop
        {
          HH_TIMER("_gdloop2");
          compute_gdloop_aux1<false>(hvideo, hmat_start, hmat_period, hvideoloop, b_exact);
        }
        if (debug) write_video(hvideoloop, "hvideoloop.mp4");
        HH_TIMER("_gdloop3");
        {
          HH_TIMER("__scale_up");
          spatially_scale_Grid3_Pixel(hvideoloop, twice(filterb), nullptr, videoloop);
        }
        if (debug) write_video(videoloop, "videoloop.mp4");
        if (1) {  // visually excellent, but actual rms numbers are poor
          compute_gdloop_fast_relax(videoloop, video, mat_start_highres, mat_period_highres);
        } else if (1) {  // rms numbers are better, but still not as good as GdLoopScheme::fast
          compute_gdloop_aux1<true>(video, mat_start_highres, mat_period_highres, videoloop, b_exact);
        } else {
          // just keep low-frequency video
        }
      }
    }
    default: assertnever("");
  }
  if (getenv_bool("VIDEOLOOP_SHOW_SPATIAL_COST")) {
    assertx(video.size());
    Matrix<int> mat_start_highres = possibly_rescale(mat_start, sdims);
    Matrix<int> mat_period_highres = possibly_rescale(mat_period, sdims);
    show_spatial_cost(video, mat_start_highres, mat_period_highres, nnf);
  }
  if (getenv_bool("VIDEOLOOP_COMPUTE_COSTS")) {
    assertx(video.size() && videoloop.size());  // it does not implement streaming video read, write, or nv12
    Matrix<int> mat_start_highres = possibly_rescale(mat_start, sdims);
    Matrix<int> mat_period_highres = possibly_rescale(mat_period, sdims);
    compute_costs(video, videoloop, mat_start_highres, mat_period_highres);
  }
}

}  // namespace hh
