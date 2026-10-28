// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#include "progs/VideoViewer/LoopParameters.h"

#include "libHh/GridPixelOp.h"  // spatially_resize_Grid3_Pixel()
#include "libHh/Image.h"
#include "libHh/MathOp.h"  // int_floor_log2()
#include "libHh/Parallel.h"
#include "libHh/Stat.h"  // HH_RSTAT()
#include "libHh/Timer.h"
#include "libHh/Vector4.h"
#include "progs/VideoViewer/GradientDomainLoop.h"  // integrally_downscale_Nv12_to_Image()

#if defined(HH_HAVE_VIDEO_LOOP)
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>  // HRESULT, used by SLooperBase.h.
// From ~/git/CompPhoto/Applications/VideoLooping/LoopAPILib/LoopAPILib/src/.
#include "OptSetting.h"
#include "PipeAPI.h"
#include "mafTime.h"
#endif

namespace hh {

void compute_looping_parameters(const Vec3<int>& odims, CGridView<3, Pixel> ovideo, CVideoNv12View ovideo_nv12,
                                int nnf, bool all_labels, bool verbose, Matrix<int>& mat_start,
                                Matrix<int>& mat_period) {
  const auto [onf, ny, nx] = odims;
  // const int DT = 4, log2DT = 2; assertx((1 << log2DT) == DT);
  int DT = 4;  // Temporal downsampling factor.
  if (onf <= 50) {
    if (verbose) Warning("Few input frames, so omitting temporal downsampling");
    DT = 1;
  }
  int log2DT = int_floor_log2(DT);
  assertx((1 << log2DT) == DT);
  Grid<3, Pixel> hvideo;
  {
    HH_TIMER("loop_downsampling");
    const Vec2<int> sdims(ny, nx);
    int DS = 1;
    {                                                                         // Spatial downsampling factor.
      const int videoloops_maxh = getenv_int("VIDEOLOOPS_MAXH", 350, false);  // Default value used in Loopers.
      Vec2<int> tdims = sdims;
      while (tdims[0] > videoloops_maxh) {
        if (!is_zero((tdims / 2) % 2)) {
          SHOW("Stopping coarsening to even dims", sdims, tdims, videoloops_maxh, DS);
          break;
        }
        tdims /= 2;
        DS *= 2;
      }
    }
    if (1 && DS > 1) DS /= 2;  // Loopers::PipeAPI expects twice the spatial resolution to get good smoothing cost
    if (0) SHOW(DS);
    assertx(is_zero(V(ny, nx) % DS));
    const int hnf = onf / DT;
    if (0) assertx(onf % DT == 0);  // No, we allow arbitrary number of frames, but truncate to a multiple of DT.
    assertx(hnf > 0);
    const ResizeOptions<2> options{.filter = "box", .boundary = "reflect"};
    const Vec2<int> hdims = sdims / DS;
    assertx(hdims * DS == sdims);
    if (0) SHOW(sdims, hdims, DS);
    Grid<3, Pixel> hvideo1(concat(V(onf), hdims));
    {  // Reduced (maybe "half") resolution.
      HH_TIMER("__scale_spatially");
      if (ovideo.size()) {
        spatially_resize_Grid3_Pixel(ovideo, options, nullptr, hvideo1);
      } else if (ovideo_nv12.size()) {
        for_int(f, onf) integrally_downscale_Nv12_to_Image(ovideo_nv12[f], hvideo1[f]);
      } else {
        assertnever("");
      }
    }
    hvideo.init(concat(V(hnf), hdims));
    {
      HH_TIMER("__scale_temporally");
      parallel_for(range(hnf), [&](const int f) {
        for_int(y, hdims[0]) for_int(x, hdims[1]) {
          Vector4i v(0);
          for_int(df, DT) v += Vector4i(hvideo1[f * DT + df, y, x]);  // OPT:DT
          hvideo[f, y, x] = (v >> log2DT).pixel();
        }
      });
    }
  }
  const Vec2<int> hdims = hvideo.dims().tail<2>();
  {
    HH_TIMER("loop_optimize");
    if (0) {  // For testing.
      Warning("Testing special looping parameters");
      const int start = nnf / 3, period = nnf / 3;
      assertx(period <= nnf);
      assertx(start + period < odims[0]);
      mat_start.init(hdims, start);
      mat_period.init(hdims, period);
    } else if (1) {
#if defined(HH_HAVE_VIDEO_LOOP)
      if (getenv_bool("LOOP_STATISTICS")) SHOW(hvideo.dims());
      if (1) mafTime::Disable();
      Loopers::Config config;
      // Note that config.m_FrameWindow is used by Loopers front-end.
      // PipeAPI::Optimize() ignores m_FrameWindow and always optimizes at granularity 1.
      // Nonetheless, Optimize() sets config.m_MinPeriod = config.m_MinPeriodSrc / config.m_FrameWindow  and
      //                              config.m_MaxPeriod = config.m_MaxPeriodSrc / config.m_FrameWindow
      //  using the default config.m_FrameWindow == 4.
      if (0) {
        const float static_cost = 200.f;  // Default 50.f (it was once 200.f).
        if (0) my_setenv(Loopers::PipeAPI::env_static_cost(), sform("%g", static_cost));
        if (0) config.m_para_static = static_cost;
      }
      if (0) my_setenv(Loopers::PipeAPI::env_diagnostics(), "true");
      if (1) {
        // Unfortunately we do not have access to framerate here, so cannot work in seconds.
        // We are not using this now.
        int min_period = getenv_int("LOOP_MIN_PERIOD");  // In original frames.
        if (min_period) {
          SHOW(min_period);
          config.m_MinPeriodSrc = min_period / DT * config.m_FrameWindow;
        }
      }
      if (config.m_MinPeriodSrc / config.m_FrameWindow * DT > onf) {
        // Warning("Very short video input, so setting minimum loop period equal to its length");
        // Not really necessary given my new check in PipeAPI::PostLoadCheck().
        config.m_MinPeriodSrc = onf / DT * config.m_FrameWindow;
      }
      if (0) {
        // These parameters are used by Loopers front-end and not by Loopers::PipeAPI.
        config.m_SkipSecs = 0.f;       // Default 2.f.
        config.m_KeepSecs = BIGFLOAT;  // Default 5.f.
        // PipeAPI always uses entire input video, which is what we want here.
        // (If desired, video should have been temporally trimmed,
        //   or trim points _framein and _frameou1 should have been set.)
      }
      if (all_labels) config.m_AllLabels = true;
      Loopers::PipeAPI pipe_api;
      pipe_api.SetConfig(config);
      assertx(pipe_api.Optimize(hvideo, mat_period, mat_start));  // Multilabel graph cut.
      assertw(is_zero(hdims % mat_start.dims()));
      if (0) SHOW(pipe_api.GetConfig().m_MinPeriodSrc, pipe_api.GetConfig().m_MinPeriod);
      for (int& period : mat_period) period = period <= 1 ? 1 : period * DT;
      for (int& start : mat_start) start *= DT;
    } else {  // For debugging.
#endif
      Warning("Compiled without support for seamless video loop");
      dummy_use(all_labels);
      const int start = 10, period = 70;
      assertx(period <= nnf);
      assertx(start + period <= odims[0]);
      mat_start.init(hdims, start);
      mat_period.init(hdims, period);
    }
    if (0 || getenv_bool("OUTPUT_LOOP_PARAMETERS")) {
      Image image(mat_start.dims());  // Size may be different from hdims.
      const int K = 4;
      for (const auto& yx : range(image.dims())) {
        const uint8_t static_frame = '\0';
        const int start = mat_start[yx], period = mat_period[yx];
        if (!(start % K == 0 && (period % K == 0 || period == 1))) SHOW(yx, start, period);
        assertx(start % K == 0 && (period % K == 0 || period == 1));
        image[yx] = Pixel(static_frame, narrow_cast<uint8_t>(start / K), narrow_cast<uint8_t>(period / K), 255);
      }
      image.write_file("output_loop_parameters.vlp.png");
    }
  }
  if (1) {
    HH_TIMER("loop_rstat");
    HH_RSTAT(Speriod, mat_period);
    HH_RSTAT(Sstart, mat_start);
    // E.g.:
    // # Speriod:            (130560 )           1:88           av=72.804352      sd=31.987289
    // # Sstart:             (130560 )           4:56           av=28.371078      sd=15.893303
  }
}

}  // namespace hh
