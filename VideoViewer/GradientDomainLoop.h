#include "Hh.h"
#include "Grid.h"               // CGridView<>, GridView<>, Grid<>
#include "Pixel.h"              // Pixel
#include "Matrix.h"             // CMatrixView<>
#include "MathOp.h"             // my_mod()
#include "../libHh/Video.h"     // relative path to avoid name collision with ./Video.h in Loopers code

namespace hh {

struct MultigridPeriodicTemporally {
    bool operator()(int d) const { return d==0; } // Grid[time][y][x]; only dimension-0 (time) is periodic
};

// Determine input frame based on temporal mapping function.
inline int get_framei(float t, int start, int period) {
    return start+my_mod(static_cast<int>(t+.5f)-start, period);
}

// Fast conversion from uint8_t to float
inline float to_float(uint8_t uc) {
#if 0
    static const Vec<float,256> ar =
        [] { Vec<float,256> ar; for_int(i, 256) ar[i] = static_cast<float>(i); return ar; }();
    return ar[uc];          // is actually slower
#else
    return static_cast<float>(uc);
#endif
}

// Compute change in input frame time in new video (with nnf frames) for a pixel with given input looping period.
inline float get_deltatime(int period, int nnf) {
    const static bool b_videloop_no_temporal_scaling = getenv_bool("VIDEOLOOP_NO_TEMPORAL_SCALING");
    if (b_videloop_no_temporal_scaling) return 1.f;
    float fnloops = static_cast<float>(nnf)/period*1.000001f;
    float facshrink = (floor(fnloops)+1.f)/fnloops;
    float facstretch = fnloops/(floor(fnloops)+1e-6f);
    float deltatime = facshrink<facstretch ? facshrink : 1.f/facstretch;
    return deltatime;
}

// Shrink an Nv12 image by an integral factor (which can be one) to an Image.
void integrally_downscale_Nv12_to_Image(CNv12View nv12, MatrixView<Pixel> nmatrixp);

enum class EGDLoopScheme { no_blend, precise, fast, exact };

// Given an input video with specified dimensions videodims{num_frames, ysize, xsize},
//  provided either as
//    (1) a file (video_filename!=""),
//    (2) a preloaded video grid (video.size()>0), or
//    (3) a preloaded NV12 video grid (video_nv12.size()>0),
//  2D matrices of per-pixel looping parameters (start frames and looping periods),
//  a specified number of frames nnf for the (new) output loop,
// compute an optimized video loop using scheme,
// and either
//    (1) write it to the video write stream (pwvideo!=nullptr),
//    (2) save it to a video grid (videoloop.size()>0), or
//    (3) save it to an Nv12 video grid (videoloop_nv12.size()>0).
// If the output is to the stream *pwvideo, one may write multiple instances (num_loops>1) of the loop,
//  else num_loops must equal 1.
void compute_gdloop(const Vec3<int>& videodims,
                    const string& video_filename, CGridView<3,Pixel> video, CVideoNv12View video_nv12,
                    CMatrixView<int> mat_start, CMatrixView<int> mat_period, EGDLoopScheme scheme, int nnf,
                    WVideo* pwvideo, GridView<3,Pixel> videoloop, VideoNv12View videoloop_nv12,
                    int num_loops);

} // namespace hh
