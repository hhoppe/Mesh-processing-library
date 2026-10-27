// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#ifndef MESH_PROCESSING_VIDEOVIEWER_LOOPPARAMETERS_H_
#define MESH_PROCESSING_VIDEOVIEWER_LOOPPARAMETERS_H_

#include "libHh/Grid.h"    // CGridView<>
#include "libHh/Matrix.h"  // Matrix<>
#include "libHh/Pixel.h"   // Pixel
#include "libHh/Video.h"   // CVideoNv12View

namespace hh {

// Given an input video with dimensions odims{num_frames, ysize, xsize}, provided either as a video grid
//  (ovideo.size() > 0) or as an NV12 video grid (ovideo_nv12.size() > 0), optimize the per-pixel looping
//  parameters (start frames and looping periods) of a seamless video loop, for a (new) output loop of nnf frames.
// The parameters are computed on a downsampled video, so mat_start and mat_period may be coarser than the input;
//  compute_gdloop() in GradientDomainLoop.h rescales them when it assembles the loop.
// If all_labels, solve for all period and start labels rather than only the dominant ones (slower but higher
//  quality).
void compute_looping_parameters(const Vec3<int>& odims, CGridView<3, Pixel> ovideo, CVideoNv12View ovideo_nv12,
                                int nnf, bool all_labels, bool verbose, Matrix<int>& mat_start,
                                Matrix<int>& mat_period);

}  // namespace hh

#endif  // MESH_PROCESSING_VIDEOVIEWER_LOOPPARAMETERS_H_
