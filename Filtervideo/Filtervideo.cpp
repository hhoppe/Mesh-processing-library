// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "VideoViewer/GradientDomainLoop.h"
#include "libHh/Advanced.h"  // clone()
#include "libHh/Args.h"
#include "libHh/ArrayOp.h"  // median()
#include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/Geometry.h"
#include "libHh/GridPixelOp.h"  // spatially_scale_Grid3_Pixel()
#include "libHh/Homogeneous.h"
#include "libHh/Image.h"
#include "libHh/MathOp.h"    // smooth_step(), frac(Vec<>), floor(Vec<>)
#include "libHh/MatrixOp.h"  // euclidean_distance_map()
#include "libHh/Multigrid.h"
#include "libHh/Parallel.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Stat.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
#include "libHh/UnionFind.h"
#include "libHh/Video.h"
using namespace hh;

namespace {

Video video;  // [frame][ypos][xpos][Z=color_channel]
VideoNv12 video_nv12;

constexpr int nz = 3;  // 3 color channels; no alpha channel

bool g_not = false;
Pixel gcolor{255, 255, 255, 255};
Bndrule bndrule = Bndrule::reflected;
FilterBnd filterb{Filter::get("spline"), bndrule};
int startframe = 0;
int tradius = 0;  // was 4
bool nooutput = false;
int trunc_begin = 0;         // skip the first trunc_begin frames
int trunc_frames = INT_MAX;  // read at most trunc_frames frames

// ***

template <int D> struct MultigridMetricAnisotropic {
  float operator()(float v, int d) const { return v * _metricw[d]; }
  static Vec<float, D> _metricw;
};
template <int D> Vec<float, D> MultigridMetricAnisotropic<D>::_metricw;

// ***

inline Pixel random_color(Random& random) {
  Pixel pix;
  pix[3] = 255;
  for_int(c, 3) pix[c] = static_cast<uint8_t>(80.f + random.unif() * 150.f + .5f);
  return pix;
}

// ***

int parse_size(string s, int size, bool measure_neg_from_end) {
  assertx(s != "" && size);
  bool is_neg = remove_at_beginning(s, "-");
  assertx(s[0] != '-');
  int i;
  if (remove_at_end(s, "%")) {
    float v = Args::parse_float(s);
    i = int(v / 100.f * size + .5f);
  } else {
    i = Args::parse_int(s);
  }
  assertx(i >= 0);
  if (is_neg) {
    if (measure_neg_from_end)
      i = size - i;
    else
      i = -i;
  }
  return i;
}

int parse_nframes(string s, bool measure_neg_from_end) {
  assertx(s != "");
  bool is_neg = remove_at_beginning(s, "-");
  assertx(s[0] != '-');
  int i;
  if (remove_at_end(s, "sec") || remove_at_end(s, "s")) {
    float v = Args::parse_float(s);
    assertx(video.attrib().framerate);
    i = int(v * video.attrib().framerate + .5);
  } else if (remove_at_end(s, "%")) {
    float v = Args::parse_float(s);
    i = int(v * .01f * video.nframes() + .5f);
  } else {
    i = Args::parse_int(s);
  }
  assertx(i >= 0);
  if (is_neg) {
    if (measure_neg_from_end)
      i = assertx(video.nframes()) - i;
    else
      i = -i;
  }
  return i;
}

void read_video(const string& filename, bool use_nv12) {
  // Similar code in: Video::read_file(), VideoNv12::read_file(), and FilterVideo.cpp::read_video().
  HH_TIMER(_read_video);
  RVideo rvideo(filename, use_nv12);
  showf("Reading video %s\n", Video::diagnostic_string(rvideo.dims(), rvideo.attrib()).c_str());
  const int nfexpect = rvideo.nframes() - trunc_begin;
  const int nf_read_expect = max(min(nfexpect, trunc_frames), 0);
  assertw(nf_read_expect > 0);
  int nfread = 0;
  const int padframes = 2;  // because may read a different number of frames
  video.attrib() = rvideo.attrib();
  if (use_nv12) {
    assertx(!video.size());
    video_nv12.init(concat(V(nf_read_expect + padframes), rvideo.spatial_dims()));
  } else {
    assertx(!video_nv12.size());
    video.init(nf_read_expect + padframes, rvideo.spatial_dims());
  }
  {
    ConsoleProgress cprogress("Vread");
    for (;;) {
      if (trunc_begin + nf_read_expect) cprogress.update(float(nfread) / (trunc_begin + nf_read_expect));
      if (nfread - trunc_begin >= trunc_frames) break;
      if (nfread - trunc_begin >= nf_read_expect + padframes) break;
      int ff = max(nfread - trunc_begin, 0);
      if (use_nv12) {
        if (!rvideo.read(video_nv12[ff])) break;
      } else {
        if (!rvideo.read(video[ff])) break;
      }
      nfread++;
    }
  }
  if (video.attrib().audio.size() && (trunc_begin || trunc_frames != INT_MAX)) {
    Warning("Clearing audio");
    video.attrib().audio.clear();  // TODO
  }
  const int nftruncread = nfread - trunc_begin;
  if (nftruncread < trunc_frames && abs(nfread - trunc_begin - nfexpect) > 1) {
    SHOW(nftruncread, nfexpect);
    Warning("Video: read a different number of frames");
  }
  if (use_nv12)
    video_nv12.special_reduce_dim0(nftruncread);
  else
    video.special_reduce_dim0(nftruncread);
}

// ***

void do_nostdin(Args& args) { dummy_use(args); }

void do_create(Args& args) {
  int nframes = args.get_int(), nx = args.get_int(), ny = args.get_int();
  assertx(nframes >= 0 && nx >= 0 && ny >= 0);
  video.init(nframes, V(ny, nx));
  if (0) {
    fill(video, gcolor);  // slower
  } else {
    parallel_for_each(
        range(video.nframes()),
        [&](const int f) {
          MatrixView<Pixel> frame = video[f];
          fill(frame, gcolor);  // white by default
        },
        video.ysize() * video.xsize() * 4);
  }
  // video.set_filename() is not called, so no default file extension.
}

void do_readnv12(Args& args) {
  string filename = args.get_filename();
  read_video(filename, true);
}

void do_trunc_begin(Args& args) {
  dummy_use(args);
  assertnever("should have been caught in main()");
}

void do_trunc_frames(Args& args) {
  dummy_use(args);
  assertnever("should have been caught in main()");
}

Vec2<int> as_fit_dims;
int as_crop_vl, as_crop_vr, as_crop_vt, as_crop_vb;
int as_tnframes;

void do_as_fit(Args& args) {
  as_fit_dims[1] = args.get_int();  // nx
  as_fit_dims[0] = args.get_int();  // ny
}

void do_as_cropsides(Args& args) {
  as_crop_vl = args.get_int();
  as_crop_vr = args.get_int();
  as_crop_vt = args.get_int();
  as_crop_vb = args.get_int();
}

void do_as_tnframes(Args& args) { as_tnframes = args.get_int(); }

void apply_as_operations(Grid<3, Pixel>& vid, const Vec2<int>& yx, const Vec2<int>& gdims) {
  if (!is_zero(as_fit_dims)) {
    {
      const float vscale = min(convert<float>(as_fit_dims) / convert<float>(vid.dims().tail<2>()));
      Vec2<int> newdims = convert<int>(convert<float>(vid.dims().tail<2>()) * vscale + .5f);
      Grid<3, Pixel> newvid(concat(V(vid.dim(0)), newdims));
      spatially_scale_Grid3_Pixel(vid, twice(filterb), &gcolor, newvid);
      vid = std::move(newvid);
    }
    Vec2<int> side0 = (as_fit_dims - vid.dims().tail<2>()) / 2;
    Vec2<int> side1 = as_fit_dims - vid.dims().tail<2>() - side0;
    vid = hh::crop(vid, concat(V(0), -side0), concat(V(0), -side1), thrice(Bndrule::border), &gcolor);
  }
  if (as_crop_vl || as_crop_vr || as_crop_vt || as_crop_vb) {
    int vl = as_crop_vl, vr = as_crop_vr, vt = as_crop_vt, vb = as_crop_vb;
    if (1) {
      // Avoid negative crop (i.e. introduction of padding region) at outer boundaries of domain.
      if (yx[1] == 0) vl = max(vl, 0);
      if (yx[1] == gdims[1] - 1) vr = max(vr, 0);
      if (yx[0] == 0) vt = max(vt, 0);
      if (yx[0] == gdims[0] - 1) vb = max(vb, 0);
    }
    vid = crop(vid, V(0, vt, vl), V(0, vb, vr), thrice(Bndrule::border), &gcolor);
  }
  if (as_tnframes) {
    const int nnf = as_tnframes;
    float fac = float(nnf) / assertx(vid.dim(0));
    Grid<3, Pixel> newvid(concat(V(nnf), vid.dims().tail<2>()));
    for_int(f, nnf) {
      int of = int(f / fac);  // not +.5f !
      assertx(of >= 0 && of < vid.dim(0));
      newvid[f].assign(vid[of]);
    }
    vid = std::move(newvid);
  }
}

// Combine 2D spatial matrix of videos (with "almost" equal number of frames) into a single video.
//  (Not CMatrixView because the videos can get padded with extra frames.)
void assemble_videos(MatrixView<Video> videos) {
  int maxframes = 0;
  for (const auto& yx : range(videos.dims())) maxframes = max(maxframes, videos[yx].nframes());
  for (const auto& yx : range(videos.dims())) {
    assertx(videos[yx].ysize() == videos[yx[0]][0].ysize());
    assertx(videos[yx].xsize() == videos[0][yx[1]].xsize());
    Video& vid = videos[yx];
    const int max_pad_frames = 1;
    if (1 && vid.nframes() != maxframes && vid.nframes() && maxframes - vid.nframes() <= max_pad_frames) {
      showf("Padding video[%d][%d] with %d frames to %d frames\n", yx[0], yx[1], vid.nframes(), maxframes);
      Warning("assemble: padding shorter video with extra frames");
      const int npad = maxframes - vid.nframes();
      vid = crop(vid, V(0, 0, 0), V(-npad, 0, 0), thrice(Bndrule::clamped));
    }
    assertx(videos[yx].nframes() == videos[0][0].nframes());
  }
  CGridView<3, Video> gvideos = raise_grid_rank(videos);  // 3D grid of Videos
  if (0) {
    for (Vec3<int> ugrid : range(gvideos.dims())) SHOW(ugrid, gvideos[ugrid].dims());
  }
  video = assemble(gvideos);
}

void do_assemble(Args& args) {
  // Filtervideo ~/data/video/short.mp4 -info -disassemble 480 270 dis
  // Filtervideo -assemble 2 2 dis.{0.0,1.0,0.1,1.1}.mp4 -info >reassemble.mp4
  // Filtervideo -nostdin -color 0 0 0 -as_fit 320 320 -as_cropsides -8 -8 -8 -8 -assemble 2 2 ~/data/video/short.mp4{,,,} | vv -
  int nx = args.get_int(), ny = args.get_int();
  assertx((nx > 0 && ny >= 0) || (nx == -1 && ny == -1));
  if (ny > 0) args.ensure_at_least(nx * ny);
  Array<string> lfilenames;
  while (args.num() && args.peek_string()[0] != '-') lfilenames.push(args.get_filename());
  if (nx == -1 && ny == -1) {
    ny = int(ceil(sqrt(float(lfilenames.num()))));
    nx = ny;
  } else if (!ny) {
    ny = (lfilenames.num() - 1) / nx + 1;
  }
  Matrix<Video> videos(V(ny, nx));
  assertx(videos.size() >= lfilenames.size());
  bool prev_silent = ConsoleProgress::set_all_silent(true);
  Matrix<string> filenames(videos.dims());
  for (const auto& yx : range(filenames.dims())) {
    if (!lfilenames.num()) continue;
    string filename = lfilenames.shift();
    filenames[yx] = filename;
    if (!file_requires_pipe(filename) && !file_exists(filename)) assertnever("file '" + filename + "' not found");
  }
  parallel_for_coords(videos.dims(), [&](const Vec2<int>& yx) {
    if (filenames[yx] == "") return;
    videos[yx].read_file(filenames[yx]);
    apply_as_operations(videos[yx], yx, videos.dims());
  });  // we can assume that parallelism is justified
  if (0) {
    for (const auto& yx : range(videos.dims())) SHOW(yx, filenames[yx], videos[yx].nframes());
  }
  ConsoleProgress::set_all_silent(prev_silent);
  for (const auto& yx : range(videos.dims())) {
    if (filenames[yx] == "") {
      // Let it be OK if fewer images than nx.
      int width = yx[0] == 0 ? 0 : videos[0][yx[1]].xsize();
      assertx(yx[0] == videos.dims()[0] - 1 && yx[1] > 0);  // partial last row
      videos[yx].init(videos[0][0].nframes(), V(videos[yx[0]][0].ysize(), width));
      videos[yx].attrib() = videos[0][0].attrib();
      fill(videos[yx], gcolor);
    }
  }
  assemble_videos(videos);
  video.attrib() = videos[0][0].attrib();  // including audio
  assertw(all_of(videos, [&](const Video& v) { return v.attrib().framerate == video.attrib().framerate; }));
  const float recoding_allowance = 1.5f;
  int newbitrate = int(float(videos[0][0].attrib().bitrate) / product(videos[0][0].spatial_dims()) *
                           product(video.spatial_dims()) * recoding_allowance +
                       .5f);
  video.attrib().bitrate = newbitrate;
  showf("Assembling new video: %s\n", Video::diagnostic_string(video.dims(), video.attrib()).c_str());
}

void do_fromimages(Args& args) {
  string rootname = args.get_filename();
  assertx(contains(rootname, '%'));  // rootname.%03d.png
  int first_named_file = startframe;
  int nframes = 0;
  {
    for (;;) {
      string s = sform_nonliteral(rootname.c_str(), first_named_file + nframes);
      if (file_exists(s)) {
        nframes++;
        continue;
      }
      if (!nframes && !first_named_file) {
        first_named_file = 1;
        continue;
      }
      break;
    }
    showf("Found %d image frames (starting at %d)\n", nframes, first_named_file);
    assertx(nframes);
  }
  if (1) {  // first frame must be read sequentially before others to get dimensions
    Image image;
    image.set_silent_io_progress(true);
    image.read_file(sform_nonliteral(rootname.c_str(), first_named_file + 0));
    video.init(nframes, image.dims());
    assertx(image.zsize() >= nz);
    assertw(image.zsize() != 4);
    video[0].assign(image);
  }
  parallel_for_each(range(nframes - 1), [&](const int f1) {
    string fname = sform_nonliteral(rootname.c_str(), first_named_file + 1 + f1);
    Image image;
    image.set_silent_io_progress(true);
    image.read_file(fname);
    assertx(same_size(image, video[0]));
    assertx(image.zsize() >= nz);
    assertw(image.zsize() != 4);
    video[1 + f1].assign(image);
  });
}

void do_framerate(Args& args) {
  string s = args.get_string();
  double& framerate = video.attrib().framerate;
  const double old_framerate = framerate;
  if (remove_at_end(s, "%")) {
    framerate *= Args::parse_double(s) / 100.;
  } else {
    framerate = Args::parse_double(s);
  }
  assertx(framerate > 0.);
  if (old_framerate) video.attrib().bitrate = int(video.attrib().bitrate * (framerate / old_framerate) + .5);
}

void do_bitrate(Args& args) {
  string arg = args.get_string();
  double factor = 1.;
  if (arg != "" && arg.back() == 'k') {
    factor = 1000.;
    arg.pop_back();
  } else if (arg != "" && arg.back() == 'm') {
    factor = 1000000.;
    arg.pop_back();
  }
  double fbitrate = Args::parse_double(arg) * factor;
  assertx(fbitrate > 0);
  if (fbitrate <= 0. || abs(fbitrate - floor(fbitrate + .5)) > 1e-6)
    assertnever(sform("bitrate %g is not positive integer", fbitrate));
  video.attrib().bitrate = int(fbitrate + .5);
}

void do_bpp(Args& args) {
  float bpp = args.get_float();
  assertx(bpp > 0.f);
  video.attrib().bitrate = int(bpp * product(video.spatial_dims()) + .5f);
}

void do_to(Args& args) { video.attrib().suffix = to_lower(args.get_string()); }

void do_outfile(Args& args) {
  string filename = args.get_filename();
  video.write_file(filename);
}

void do_append(Args& args) {
  string filename = args.get_filename();
  Video video2;
  video2.read_file(filename);
  const int nf1 = video.nframes(), nf2 = video2.nframes();
  assertx(nf2 > 0);
  assertx(same_size(video[0], video2[0]));
  assertx(nf1 >= tradius * 2 && nf2 >= tradius * 2);
  assertw(!video2.attrib().audio.size());  // TODO
  int nnf = nf1 + nf2 - tradius * 2;
  showf("Appending video '%s' (%d frames) new_nframes=%d\n", filename.c_str(), nf2, nnf);
  Video nvideo(nnf, video.spatial_dims());
  nvideo.attrib() = video.attrib();
  parallel_for_each(range(nnf), [&](const int f) {
    int f2 = f - (nf1 - tradius * 2);  // frame from video2
    if (f < nf1 - tradius * 2) {
      nvideo[f].assign(video[f]);
    } else if (f >= nf1) {
      nvideo[f].assign(video2[f2]);
    } else {
      float alpha = (f2 + .5f) / (2 * tradius);
      for (const auto& yx : range(video.spatial_dims())) {
        for_int(z, nz) {
          nvideo[f][yx][z] = uint8_t((1.f - alpha) * video[f][yx][z] + alpha * video2[f2][yx][z] + .5f);
        }
      }
    }
  });
  video = std::move(nvideo);
}

void do_toimages(Args& args) {
  string rootname = args.get_filename();
  assertx(contains(rootname, '%'));                                            // rootname.%03d.png
  assertx(!file_exists(sform_nonliteral(rootname.c_str(), video.nframes())));  // last + 1 should not exist
  parallel_for_each(range(video.nframes()), [&](const int f) {
    Image image(video.spatial_dims());
    image.set_silent_io_progress(true);
    image = video[f];
    image.write_file(sform_nonliteral(rootname.c_str(), f));
  });
  nooutput = true;
}

void do_writeframe(Args& args) {
  int f = args.get_int();
  assertx(f >= 0 && f < video.nframes());
  Image image(video.spatial_dims());
  image = video[f];
  image.set_suffix("png");
  image.write_file("-");
  nooutput = true;
}

void do_info() {
  HH_TIMER(_info);
  showf("Video nf=%d w=%d h=%d fps=%g bitrate=%d suffix=%s%s\n", video.nframes(), video.xsize(), video.ysize(),
        video.attrib().framerate, video.attrib().bitrate,
        video.attrib().suffix == "" ? "unk" : video.attrib().suffix.c_str(),
        video.attrib().audio.size() ? (" (audio: " + video.attrib().audio.diagnostic_string() + ")").c_str() : "");
  Array<Stat> stat_pixels;
  for_int(z, nz) stat_pixels.push(Stat(sform("Component%d", z)));
  if (0) {
    for (const auto& fyx : range(video.dims())) {
      const Pixel& pix = video[fyx];
      for_int(z, nz) stat_pixels[z].enter(pix[z]);
    }
  } else {
    Matrix<Stat> framestats(video.nframes(), nz);
    parallel_for_each(range(video.nframes()), [&](const int f) {
      for (const auto& yx : range(video.spatial_dims())) {
        const Pixel& pix = video[f][yx];
        for_int(z, nz) framestats[f][z].enter(pix[z]);  // OPT_info
      }
    });
    for_int(f, video.nframes()) for_int(z, nz) stat_pixels[z].add(framestats[f][z]);
  }
  for_int(z, nz) showf("%s", stat_pixels[z].name_string().c_str());
}

void do_noaudio() { video.attrib().audio.clear(); }

void do_stat() {
  do_info();
  nooutput = true;
}

void do_sizes() {
  std::cout << sform("%d %d %d\n", video.nframes(), video.xsize(), video.ysize());
  nooutput = true;
}

void do_tscale(Args& args) {
  HH_TIMER(_tscale);
  float fac = args.get_float();
  assertx(fac > 0.f);
  int nnf = int(video.nframes() * fac + 0.5f);
  Video nvideo(nnf, video.spatial_dims());
  nvideo.attrib() = video.attrib();
  if (nvideo.attrib().audio.size()) {
    Warning("Clearing audio");
    nvideo.attrib().audio.clear();
  }
  showf("Time-scaling video from %d to %d frames.\n", video.nframes(), nvideo.nframes());
  parallel_for_each(range(nnf), [&](const int f) {
    int of = int(f / fac);  // not + .5f !
    assertx(of >= 0 && of < video.nframes());
    nvideo[f].assign(video[of]);
  });
  video = std::move(nvideo);
}

void do_tnframes(Args& args) {
  HH_TIMER(_tscale);
  int nnf = args.get_int();
  assertx(nnf > 0);
  if (video.nframes() == nnf) return;
  float fac = float(nnf) / assertx(video.nframes());
  Video nvideo(nnf, video.spatial_dims());
  nvideo.attrib() = video.attrib();
  if (nvideo.attrib().audio.size()) {
    Warning("Clearing audio");
    nvideo.attrib().audio.clear();
  }
  showf("Time-scaling video from %d to %d frames.\n", video.nframes(), nvideo.nframes());
  parallel_for_each(range(nnf), [&](const int f) {
    int of = int(f / fac);  // not + .5f !
    assertx(of >= 0 && of < video.nframes());
    nvideo[f].assign(video[of]);
  });
  video = std::move(nvideo);
}

void do_start(Args& args) {
  int i = parse_nframes(args.get_string(), true);
  if (i < 0) {
    Warning("Start frame < 0, so no effect");
  } else if (i >= video.nframes()) {
    video.clear();
    Warning("Start frame beyond video, so video truncated to nothing");
  } else if (i) {
    showf("Truncating %d frames from video start.\n", i);
    video = crop(video, V(i, 0, 0), V(0, 0, 0));
    if (video.attrib().audio.size()) {  // TODO
      Warning("Clearing audio");
      video.attrib().audio.clear();
    }
  }
}

void do_end(Args& args) {
  int i = parse_nframes(args.get_string(), true);
  if (i <= 0) {
    video.clear();
    Warning("End frame <= 0, so video truncated to nothing");
  } else if (i >= video.nframes()) {
    Warning("End frame beyond video, so no effect");
  } else {
    i = video.nframes() - i;
    showf("Truncating %d frames from video end.\n", i);
    if (float(video.nframes() - i) / video.nframes() < .6f) {
      // Reallocation could run out of memory for large videos.
      video = crop(video, V(0, 0, 0), V(i, 0, 0));
    } else {
      video.special_reduce_dim0(video.nframes() - i);
    }
    if (video.attrib().audio.size()) {
      Warning("Clearing audio");
      video.attrib().audio.clear();
    }
  }
}

void do_interval(Args& args) {
  int i0 = parse_nframes(args.get_string(), true);
  int i1 = parse_nframes(args.get_string(), true);
  if (i0 < 0) {
    Warning("Start frame < 0, so set to 0");
    i0 = 0;
  }
  if (i1 > video.nframes()) {
    Warning("End frame >end, so set to end");
    i1 = video.nframes();
  }
  if (i0 == i1) {
    Warning("Start frame equals end frame, so video truncated to nothing");
    video.clear();
  } else if (i0 > i1) {
    Warning("Start frame is beyond end frame, so video truncated to nothing");
    video.clear();
  } else {
    showf("Truncating to [%d, %d) == %d frames.\n", i0, i1, i1 - i0);
    int iU = video.nframes() - i1;
    video = crop(video, V(i0, 0, 0), V(iU, 0, 0));
    if (video.attrib().audio.size()) {
      Warning("Clearing audio");
      video.attrib().audio.clear();
    }
  }
}

void do_trimbeg(Args& args) {
  int nframes = parse_nframes(args.get_string(), false);
  if (video.nframes()) {
    video = crop(video, V(nframes, 0, 0), V(0, 0, 0), thrice(bndrule));
    if (video.attrib().audio.size()) {
      Warning("Clearing audio");
      video.attrib().audio.clear();
    }
  }
}

void do_trimend(Args& args) {
  int nframes = parse_nframes(args.get_string(), false);
  if (video.nframes()) {
    video = crop(video, V(0, 0, 0), V(nframes, 0, 0), thrice(bndrule));
    if (video.attrib().audio.size()) {
      Warning("Clearing audio");
      video.attrib().audio.clear();
    }
  }
}

void do_loop(Args& args) {
  HH_TIMER(_loop);
  int ninst = args.get_int();
  assertx(ninst >= 1);
  const int onf = video.nframes();
  Video nvideo(ninst * onf, video.spatial_dims());
  nvideo.attrib() = video.attrib();
  if (nvideo.attrib().audio.size()) {
    Warning("Clearing audio");
    nvideo.attrib().audio.clear();
  }
  parallel_for_each(range(nvideo.nframes()), [&](const int f) { nvideo[f].assign(video[f % onf]); });
  video = std::move(nvideo);
}

void do_mirror(Args& args) {
  HH_TIMER(_mirror);
  int ninst = args.get_int();
  assertx(ninst >= 1);
  assertw(ninst >= 2);
  const int onf = video.nframes();
  Video nvideo(ninst * onf, video.spatial_dims());
  nvideo.attrib() = video.attrib();
  if (nvideo.attrib().audio.size()) {
    Warning("Clearing audio");
    nvideo.attrib().audio.clear();
  }
  parallel_for_each(range(nvideo.nframes()), [&](const int f) {
    int inst = f / onf, fi = f % onf;
    int of = inst % 2 == 0 ? fi : onf - 1 - fi;
    nvideo[f].assign(video[of]);
  });
  video = std::move(nvideo);
}

void do_reverse() {
  int nframes = video.nframes();
  for_int(j, nframes / 2) swap_ranges(video[j], video[nframes - 1 - j]);
  if (video.attrib().audio.size()) {
    Warning("Clearing audio");
    video.attrib().audio.clear();
  }
}

void do_phaseoffset(Args& args) {
  int fbeg = parse_nframes(args.get_string(), true);
  int nframes = video.nframes();
  assertx(fbeg >= 0 && fbeg < nframes);
  if (fbeg == 0) return;
  // rotate(video, video[fbeg]);  // make fbeg the new frame 0 (using old Array<Matrix<Pixel>> representation)
  Video nvideo(nframes, video.spatial_dims());
  nvideo.attrib() = video.attrib();
  if (nvideo.attrib().audio.size()) {
    Warning("Clearing audio");
    nvideo.attrib().audio.clear();
  }
  parallel_for_each(range(nvideo.nframes()), [&](const int f) { nvideo[f].assign(video[(fbeg + f) % nframes]); });
  video = std::move(nvideo);
}

void do_tcrossfade(Args& args) {
  HH_TIMER(_tcrossfade);
  int fbeg = parse_nframes(args.get_string(), true);
  int fend = parse_nframes(args.get_string(), true);
  assertx(tradius > 0);
  int nframes = video.nframes();
  assertw(fbeg - tradius >= 0 && fend + tradius < nframes);
  assertx(fbeg + tradius - 1 < fend - tradius);
  Video tvideo(2 * tradius, video.spatial_dims());  // necessary if clamping on {fb, fe} occurs below
  parallel_for_each(range(2 * tradius), [&](const int i) {
    int fb = fbeg - tradius + i;
    int fe = fend - tradius + i;
    int fdst = fb >= fbeg ? fb : fe;
    float alpha = 1.f - .5f * (i + .5f) / tradius;  // weight given to end frame fe
    if (0) showf("alpha=%.4f fb=%-3d fe=%-3d fdst=%-3d\n", alpha, fb, fe, fdst);
    int fbc = clamp(fb, 0, nframes - 1);
    int fec = clamp(fe, 0, nframes - 1);
    if (1) {  // good code always
      for_int(y, video.ysize()) for_int(x, video.xsize()) for_int(z, nz) {
        tvideo(i, y, x)[z] = uint8_t((1.f - alpha) * video(fbc, y, x)[z] + alpha * video(fec, y, x)[z] + .5f);
      }
    } else {  // good code with gcc
      auto videofbc = video[fbc], videofec = video[fbc];
      for (const auto& yx : range(video.spatial_dims())) {
        for_int(z, nz) tvideo[i][yx][z] = uint8_t((1.f - alpha) * videofbc[yx][z] + alpha * videofec[yx][z] + .5f);
      }
    }
  });
  parallel_for_each(range(tradius), [&](const int i) { video[fbeg + i].assign(tvideo[tradius + i]); });
  parallel_for_each(range(tradius), [&](const int i) { video[fend - tradius + i].assign(tvideo[i]); });
}

// Filtervideo ~/data/video/HDbrink8hb.mp4 -makeloop 1 53 >HDbrink8hb.makeloop.mp4
// Filtervideo ~/proj/motiongraph/data/circlestar.mp4 -makeloop 1 100 >circlestar.makeloop.mp4
// Filtervideo //ivm-server2/blink/VideoLooping/NOT_Uploaded_CloudFactory/2015-12-14/Joshwe/burst_IMG_5193_stab.mp4 -makeloop 1 -1 >v.mp4
void do_makeloop(Args& args) {
  HH_TIMER(_makeloop);
  int fbeg = parse_nframes(args.get_string(), true);
  int fend = parse_nframes(args.get_string(), true);
  const int nf = fend - fbeg, ny = video.ysize(), nx = video.xsize();
  ;
  assertw(fbeg > 0 && fend < video.nframes());  // at least one frame padding at beg/end to retrieve gradient
  assertx(nf >= 3);
  Video nvideo(nf, video.spatial_dims());
  nvideo.attrib() = video.attrib();
  if (nvideo.attrib().audio.size()) {
    Warning("Clearing audio");
    nvideo.attrib().audio.clear();
  }
  showf("Rendering looping video of %d frames.\n", nf);
  ConsoleProgress cprogress;
  for_int(z, nz) {  // optimize one color channel at a time
    cprogress.update(float(z) / nz);
    if (0) {  // solve directly for colors; may require many iterations to accurately converge
#if 1
      const float wtemporal = 1.f;  // temporal weight (1.f == isotropic space and time)
      using MultigridType = Multigrid<3, float, MultigridPeriodicTemporally>;
#else
      const float wtemporal = .25f;  // give less weight to temporal differences
      using MultigridType = Multigrid<3, float, MultigridPeriodicTemporally, MultigridMetricAnisotropic<3>>;
      MultigridMetricAnisotropic<3>::_metricw = {wtemporal, 1.f, 1.f};
#endif
      MultigridType multigrid(nvideo.dims());
      {
        HH_STIMER(__setup_rhs);
        parallel_for_each(range(nf), [&](const int f) {
          for_int(y, ny) for_int(x, nx) {
            int fi = fbeg + f;
            float pixv = to_float(video(fi, y, x)[z]);
            float vrhs = 0.f;
            if (1) {
              float v = to_float(video(fi - 1, y, x)[z]) - pixv;
              if (f == 0) v = (v + to_float(video(fend - 1, y, x)[z]) - to_float(video(fend, y, x)[z])) * .5f;
              vrhs += v * wtemporal;
            }
            if (1) {
              float v = to_float(video(fi + 1, y, x)[z]) - pixv;
              if (f == nf - 1) v = (v + to_float(video(fbeg, y, x)[z]) - to_float(video(fbeg - 1, y, x)[z])) * .5f;
              vrhs += v * wtemporal;
            }
            if (y > 0) vrhs += (to_float(video(fi, y - 1, x)[z]) - pixv);
            if (y < ny - 1) vrhs += (to_float(video(fi, y + 1, x)[z]) - pixv);
            if (x > 0) vrhs += (to_float(video(fi, y, x - 1)[z]) - pixv);
            if (x < nx - 1) vrhs += (to_float(video(fi, y, x + 1)[z]) - pixv);
            multigrid.rhs()(f, y, x) = vrhs;
            multigrid.initial_estimate()(f, y, x) = pixv;
          }
        });
      }
      multigrid.set_desired_mean(mean(multigrid.initial_estimate()));
      if (1) multigrid.set_num_vcycles(10);  // was 3
      if (1) multigrid.set_verbose(true);
      multigrid.solve();
      for_int(f, nf) for_int(y, ny) for_int(x, nx) {
        nvideo(f, y, x)[z] = clamp_to_uint8(int(multigrid.result()(f, y, x) + .5f));
      }
    } else {  // solve for offsets instead of colors themselves
      const float screening_weight = getenv_float("SCREENING_WEIGHT", 1e-3f, true);  // weak screening
      using MultigridType = Multigrid<3, float, MultigridPeriodicTemporally>;
      using EType = float;
      MultigridType multigrid(nvideo.dims());
      {
        HH_STIMER(__setup_rhs);
        parallel_for_each(range(nf), [&](const int f) { fill(multigrid.initial_estimate()[f], EType{0}); });
        parallel_for_each(range(nf), [&](const int f) { fill(multigrid.rhs()[f], EType{0}); });
        GridView<3, EType> mrhs = multigrid.rhs();
        for_int(y, ny) for_int(x, nx) {
          int count = 0;
          EType change{0};
          if (fend < video.nframes()) {
            count++;
            change += video(fend, y, x)[z] - video(fbeg, y, x)[z];
          }
          if (fbeg > 0) {
            count++;
            change += video(fend - 1, y, x)[z] - video(fbeg - 1, y, x)[z];
          }
          if (count == 2) change *= .5f;
          mrhs(0, y, x) = -change;
          mrhs(nf - 1, y, x) = change;
        }
      }
      if (1) multigrid.set_screening_weight(screening_weight);
      if (1) multigrid.set_num_vcycles(3);  // was 3 then 10 then 3
      if (0) multigrid.set_verbose(true);
      multigrid.solve();
      parallel_for_each(range(nf), [&](const int f) {
        for_int(y, ny) for_int(x, nx) {
          nvideo(f, y, x)[z] =
              clamp_to_uint8(int(to_float(video(fbeg + f, y, x)[z]) + multigrid.result()(f, y, x) + .5f));
        }
      });
    }
  }
  if (0) nvideo.attrib().bitrate = video.attrib().bitrate * 2;
  if (1) nvideo.attrib().bitrate = max(video.attrib().bitrate, 20 * 1000 * 1000);
  video = std::move(nvideo);
}

// *** cropping

void do_color(Args& args) {
  for_int(i, 3) {
    int v = args.get_int();
    assertx(v >= 0 && v <= 255);
    gcolor[i] = uint8_t(v);
  }
}

void do_not() { g_not = !g_not; }

void do_getcolorfxy(Args& args) {
  int f = parse_size(args.get_string(), video.nframes(), true);
  int x = parse_size(args.get_string(), video.xsize(), true);
  int y = parse_size(args.get_string(), video.ysize(), true);
  const Vec3<int> fyx(f, y, x);
  assertx(video.ok(fyx));
  gcolor = video[fyx];
  gcolor[3] = 255;
  showf("gotcolor %d %d %d\n", gcolor[0], gcolor[1], gcolor[2]);
}

void do_boundaryrule(Args& args) {
  bndrule = parse_boundaryrule(args.get_string());
  filterb.set_bndrule(bndrule);
}

void do_cropsides(Args& args) {
  int vl = parse_size(args.get_string(), video.xsize(), false);
  int vr = parse_size(args.get_string(), video.xsize(), false);
  int vt = parse_size(args.get_string(), video.ysize(), false);
  int vb = parse_size(args.get_string(), video.ysize(), false);
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, V(0, vt, vl), V(0, vb, vr), thrice(bndrule), &gcolor);
}

void do_cropl(Args& args) {
  int v = parse_size(args.get_string(), video.xsize(), false);
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, V(0, 0, v), V(0, 0, 0), thrice(bndrule), &gcolor);
}

void do_cropr(Args& args) {
  int v = parse_size(args.get_string(), video.xsize(), false);
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, V(0, 0, 0), V(0, 0, v), thrice(bndrule), &gcolor);
}

void do_cropt(Args& args) {
  int v = parse_size(args.get_string(), video.ysize(), false);
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, V(0, v, 0), V(0, 0, 0), thrice(bndrule), &gcolor);
}

void do_cropb(Args& args) {
  int v = parse_size(args.get_string(), video.ysize(), false);
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, V(0, 0, 0), V(0, v, 0), thrice(bndrule), &gcolor);
}

void do_cropall(Args& args) {
  int v = video.ysize() == video.xsize() ? parse_size(args.get_string(), video.ysize(), false) : args.get_int();
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, V(0, v, v), V(0, v, v), thrice(bndrule), &gcolor);
}

void do_cropsquare(Args& args) {
  int x = parse_size(args.get_string(), video.xsize(), true);
  int y = parse_size(args.get_string(), video.ysize(), true);
  int s = video.ysize() == video.xsize() ? parse_size(args.get_string(), video.ysize(), false) : args.get_int();
  const int y0 = y - s / 2, x0 = x - s / 2;
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, V(0, y0, x0), V(0, video.ysize() - y0 - s, video.xsize() - x0 - s), thrice(bndrule), &gcolor);
}

void do_croprectangle(Args& args) {
  int x = parse_size(args.get_string(), video.xsize(), true);
  int y = parse_size(args.get_string(), video.ysize(), true);
  int sx = parse_size(args.get_string(), video.xsize(), false);
  int sy = parse_size(args.get_string(), video.ysize(), false);
  int y0 = y - sy / 2, x0 = x - sx / 2;
  Grid<3, Pixel>& grid = video;
  grid =
      hh::crop(grid, V(0, y0, x0), V(0, video.ysize() - y0 - sy, video.xsize() - x0 - sx), thrice(bndrule), &gcolor);
}

void do_cropcoord(Args& args) {
  int x0 = parse_size(args.get_string(), video.xsize(), true);
  int y0 = parse_size(args.get_string(), video.ysize(), true);
  int x1 = parse_size(args.get_string(), video.xsize(), true);
  int y1 = parse_size(args.get_string(), video.ysize(), true);
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, V(0, y0, x0), V(0, video.ysize() - y1, video.xsize() - x1), thrice(bndrule), &gcolor);
}

void do_croptodims(Args& args) {
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx > 0 && ny > 0);
  auto ndims = V(ny, nx);
  auto yx0 = (video.spatial_dims() - ndims) / 2;
  auto yx1 = video.spatial_dims() - ndims - yx0;
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, concat(V(0), yx0), concat(V(0), yx1), thrice(bndrule), &gcolor);
}

void do_cropmult(Args& args) {
  int fac = args.get_int();
  assertx(fac >= 1);
  auto ndims = (video.spatial_dims() + (fac - 1)) / fac * fac;
  auto yx0 = (video.spatial_dims() - ndims) / 2;
  auto yx1 = video.spatial_dims() - ndims - yx0;
  Grid<3, Pixel>& grid = video;
  grid = hh::crop(grid, concat(V(0), yx0), concat(V(0), yx1), thrice(bndrule), &gcolor);
}

// *** scale

void do_filter(Args& args) { filterb.set_filter(Filter::get(args.get_string())); }

void apply_scale(const Vec2<float>& v) {
  video.scale(v, twice(filterb), &gcolor);
  if (max(v) < 1.f) {
    video.attrib().bitrate = int(video.attrib().bitrate * pow(float(product(v)), .8f) + .5f);
    showf("Reducing bitrate after scaling: %s\n", Video::diagnostic_string(video.dims(), video.attrib()).c_str());
  }
}

void do_scaleunif(Args& args) {
  HH_TIMER(_scale);
  float s = args.get_float();
  apply_scale(twice(s));
}

void do_scalenonunif(Args& args) {
  HH_TIMER(_scale);
  float sx = args.get_float(), sy = args.get_float();
  apply_scale(V(sy, sx));
}

void do_scaletox(Args& args) {
  HH_TIMER(_scale);
  int nx = parse_size(args.get_string(), video.xsize(), false);
  assertx(nx > 0);
  float s = float(nx) / assertx(video.xsize());
  apply_scale(twice(s));
}

void do_scaletoy(Args& args) {
  HH_TIMER(_scale);
  int ny = parse_size(args.get_string(), video.ysize(), false);
  assertx(ny > 0);
  float s = float(ny) / assertx(video.ysize());
  apply_scale(twice(s));
}

void do_scaleifgtmax(Args& args) {
  HH_TIMER(_scale);
  int n = args.get_int();
  assertx(n > 0);
  int cn = max(video.spatial_dims());
  if (cn <= n) return;
  apply_scale(twice(float(n) / assertx(cn)));
}

void do_scaletodims(Args& args) {
  HH_TIMER(_scale);
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx > 0 && ny > 0);
  auto syx = convert<float>(V(ny, nx)) / convert<float>(video.spatial_dims());
  apply_scale(syx);
}

void do_scaleinside(Args& args) {
  HH_TIMER(_scale);
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx > 0 && ny > 0);
  apply_scale(twice(min(convert<float>(V(ny, nx)) / convert<float>(video.spatial_dims()))));
}

// *** misc

void do_flipvertical() {
  parallel_for_each(range(video.nframes()), [&](const int f) {
    for_int(y, video.ysize() / 2) swap_ranges(video[f][y], video[f][video.ysize() - 1 - y]);
  });
}

void do_fliphorizontal() {
  parallel_for_each(range(video.nframes()), [&](const int f) { for_int(y, video.ysize()) reverse(video[f][y]); });
}

void do_disassemble(Args& args) {
  HH_TIMER(_disassemble);
  // Filtervideo ~/data/video/short.mp4 -disassemble 480 270 dis
  // Filtervideo -assemble 2 2 dis.{0.0,1.0,0.1,1.1}.mp4 >reassemble.mp4
  int tilex = args.get_int();  // resulting tile size (x, y)
  int tiley = args.get_int();
  assertx(tilex > 0 && tiley > 0);
  assertw(video.ysize() % tiley == 0);
  assertw(video.xsize() % tilex == 0);
  const Vec2<int> tiledims(tiley, tilex), atiles = video.spatial_dims() / tiledims;
  string rootname = args.get_filename();
  string suffix = video.attrib().suffix;
  assertx(suffix != "");
  if (video.attrib().audio.size()) Warning("Duplicating audio");
  parallel_for_coords(
      atiles,
      [&](const Vec2<int>& tyx) {
        Video nvideo(video.nframes(), tiledims);
        nvideo.attrib() = video.attrib();
        for_int(f, video.nframes()) for (const auto& yx : range(tiledims)) {
          nvideo[f][yx] = video[f][tyx * tiledims + yx];
        }
        nvideo.write_file(sform_nonliteral("%s.%d.%d.%s", rootname.c_str(), tyx[1], tyx[0], suffix.c_str()));
      },
      video.nframes() * product(tiledims) * 4);
  nooutput = true;
}

void do_gridcrop(Args& args) {
  HH_TIMER(_gridcrop);
  // Filtervideo ~/data/video/short.mp4 -as_cropsides -1 -1 -1 -1 -gridcrop 3 3 20 20 >gridcrop.mp4
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx >= 2 && ny >= 2);
  int sx = args.get_int(), sy = args.get_int();
  assertx(sx >= 0 && sx <= video.xsize() && sy >= 0 && sy <= video.ysize());
  Matrix<Video> videos(ny, nx);
  parallel_for_coords(
      videos.dims(),
      [&](const Vec2<int>& yx) {
        int vl = int((video.xsize() - sx) * float(yx[1]) / (nx - 1) + .5f);
        int vt = int((video.ysize() - sy) * float(yx[0]) / (ny - 1) + .5f);
        int vr = video.xsize() - vl - sx;
        int vb = video.ysize() - vt - sy;
        if (0) SHOW(vl, vt, vr, vb);
        Grid<3, Pixel>& vid = videos[yx];
        vid = crop(video, V(0, vt, vl), V(0, vb, vr), thrice(bndrule), &gcolor);
        apply_as_operations(vid, yx, videos.dims());
      },
      video.nframes() * sy * sx * 4);
  assemble_videos(videos);
}

void do_replace(Args& args) {
  Pixel newcolor;
  for_int(i, 3) {
    int v = args.get_int();
    assertx(v >= 0 && v <= 255);
    newcolor[i] = uint8_t(v);
  }
  newcolor[3] = 255;
  int count = 0;
  for (Pixel& pix : video) {
    if (equal(pix, gcolor, nz) ^ g_not) {
      count++;
      pix = newcolor;
    }
  }
  showf("Replaced %d pixels\n", count);
}

void do_gamma(Args& args) {
  float gamma = args.get_float();
  Vec<uint8_t, 256> transf;
  for_int(i, 256) transf[i] = static_cast<uint8_t>(255.f * pow(i / 255.f, gamma) + 0.5f);
  if (1) {
    parallel_for_each(
        range(video.size()),
        [&](const size_t i) {
          for_int(z, nz) video.flat(i)[z] = transf[video.flat(i)[z]];  // fastest
        },
        10);
  } else if (0) {
    parallel_for_coords(
        video.dims(), [&](const Vec3<int>& fyx) { for_int(z, nz) video[fyx][z] = transf[video[fyx][z]]; }, 10);
  } else {
    parallel_for_each(range(video.nframes()), [&](const int f) {
      for_int(y, video.ysize()) for_int(x, video.xsize()) {
        for_int(z, nz) video(f, y, x)[z] = transf[video(f, y, x)[z]];
      }
    });
  }
}

// *** looping videos

constexpr bool use_activation = true;  // setting to "false" speeds up the loading of pjo/pjr files

struct {
  Matrix<int> mat_static;  // static frame
  Matrix<int> mat_start;   // start frame
  Matrix<int> mat_period;  // period (1 or a multiple of K)
  Matrix<float> mat_activation;
  // per-pixel data
  Matrix<float> mat_tcost;  // temporal cost
  Matrix<int> mat_iregion;  // index of independent looping region
  // per-region data
  Array<Pixel> region_color;     // color
  Array<Point> region_centroid;  // center point
} g_lp;                          // looping data structures

void possibly_rescale_loop_parameters() {
  assertx(g_lp.mat_static.ysize() > 0);
  if (g_lp.mat_static.dims() != video.spatial_dims()) {
    // std::cerr << "Scaling vlp from " << g_lp.mat_static.dims() <<  " to " << video.spatial_dims() << "\n";
    showdf("Scaling vlp from %s to %s\n", make_string(g_lp.mat_static.dims()).c_str(),
           make_string(video.spatial_dims()).c_str());
  }
  FilterBnd tfilterb(Filter::get("impulse"), Bndrule::reflected);
  auto func_scale_Matrix_int = [&](Matrix<int>& m) {
    m = scale_filter_nearest(m, video.spatial_dims(), std::move(m));
  };
  func_scale_Matrix_int(g_lp.mat_static);
  func_scale_Matrix_int(g_lp.mat_start);
  func_scale_Matrix_int(g_lp.mat_period);
  g_lp.mat_activation = scale(g_lp.mat_activation, video.spatial_dims(), twice(tfilterb),
                              implicit_cast<float*>(nullptr), std::move(g_lp.mat_activation));
}

void verify_loop_parameters() {
  if (1) {
    HH_RSTAT(Sstatic, g_lp.mat_static);
    HH_RSTAT(Sstart, g_lp.mat_start);
    HH_RSTAT(Speriod, g_lp.mat_period);
    HH_RSTAT(Sactivation, g_lp.mat_activation);
    {
      HH_STAT(Speriodd);
      for (const auto e : g_lp.mat_period) {
        if (e > 1) Speriodd.enter(e);  // dynamic
      }
    }
    {
      HH_STAT(Send);
      for_size_t(i, g_lp.mat_start.size()) Send.enter(g_lp.mat_start.flat(i) + g_lp.mat_period.flat(i));
      if (video.nframes() && int(Send.max()) > video.nframes())
        Warning("Some loop ends extend beyond the video length");
    }
  }
  if (1) {
    parallel_for_coords(
        g_lp.mat_static.dims(),
        [&](const Vec2<int>& yx) {
          int staticf = g_lp.mat_static[yx], start = g_lp.mat_start[yx], period = g_lp.mat_period[yx];
          if (video.nframes() && !(period > 0 && start + period <= video.nframes())) {
            SHOW(staticf, start, period);
            assertnever("");
          }
          if (1 && !(staticf >= start && staticf < start + period)) {
            SHOW(staticf, start, period);
            assertnever("");
          }
          if (period > 1 && period < 20) assertnever("period too short?");
        },
        20);
  }
}

void do_loadpj(Args& args) {
  HH_TIMER(_loadpj);
  string filename = args.get_filename();
  unique_ptr<RFile> pfi = make_unique<RFile>(filename);
  if ((*pfi)().peek() == 31) {  // decompress a gzip-compressed pjo/pjr file
    pfi = make_unique<RFile>("gzip -d -c <" + filename + " |");
  }
  std::istream& is = (*pfi)();
  g_lp.mat_static.init(video.spatial_dims());
  g_lp.mat_start.init(video.spatial_dims());
  g_lp.mat_period.init(video.spatial_dims());
  for (auto& e : g_lp.mat_static) is >> e;
  assertx(is);
  for (auto& e : g_lp.mat_start) is >> e;
  assertx(is);
  for (auto& e : g_lp.mat_period) is >> e;
  assertx(is);
  if (use_activation) {
    g_lp.mat_activation.init(video.spatial_dims());
    for (auto& e : g_lp.mat_activation) is >> e;  // note that parsing float is slow
    assertx(is);
    assertx(is.get() == '\t');
    assertx(is.get() == '\n');
    assertx(is.get() < 0);
    assertx(min(g_lp.mat_activation) >= 0.f);
    assertx(max(g_lp.mat_activation) <= 2.f);  // <= 1.f, or == 2.f for static
  } else {
    while (is) is.get();  // flush the remainder of the input to avoid any "dropped pipe" event
  }
  verify_loop_parameters();
}

void do_loadvlp(Args& args) {
  HH_TIMER(_loadvlp);
  string filename = args.get_filename();  // it is a png file, even if its suffix is *.vlp
  Image image;
  image.read_file(filename);
  assertx(image.zsize() >= 3);
  g_lp.mat_static.init(image.dims());
  g_lp.mat_start.init(image.dims());
  g_lp.mat_period.init(image.dims());
  g_lp.mat_activation.init(image.dims());
  const int K = 4;
  const int static_frame_offset = 2;
  for (const auto& yx : range(image.dims())) {
    g_lp.mat_static[yx] = image[yx][0] * K + static_frame_offset;
    g_lp.mat_start[yx] = image[yx][1] * K;
    g_lp.mat_period[yx] = image[yx][2] * K;
    if (0) {
      Warning("Using old period mapping");
      assertx(image[yx][2] >= 1);
      g_lp.mat_period[yx] = (image[yx][2] - 1) * K;  // we previously had pixelvalue = period / K + 1
    }
    g_lp.mat_activation[yx] = image.zsize() == 4 ? image[yx][3] / 254.f : 0.f;
    if (g_lp.mat_period[yx] == 0) g_lp.mat_period[yx] = 1;  // static pixel should have period == 1
    if (g_lp.mat_period[yx] == 1) g_lp.mat_start[yx] += static_frame_offset;
    if (1) {
      g_lp.mat_static[yx] = g_lp.mat_start[yx];  // pick any frame for now
    } else {
      // mat_static actually points into scaled/stretched looping video (including phase offset).
      // The code below likely does not reproduce Loopers computation!  not debugged.
      const int looplen = 5;  // in seconds
      const int nnf = int(looplen * video.attrib().framerate + .5);
      int period = g_lp.mat_period[yx];
      float deltatime = get_deltatime(period, nnf);
      float fnewperiod = period * deltatime;
      // HH_SSTAT(Sfnewperiod, fnewperiod);
      // int newperiod = int(fnewperiod / 4.f + .5f) * 4;
      int newperiod = int(fnewperiod + .5f);
      // HH_SSTAT(Snewperiod0, newperiod);
      int newstatic = g_lp.mat_static[yx];
      float fracstatic = float(newstatic) / newperiod;
      // HH_SSTAT(Sfracstatic, fracstatic);  // hopefully in range [0.f, 1.f)
      int start = g_lp.mat_start[yx];
      float fracstart = (start % period) / float(period);
      int istatic = start + int(frac(fracstatic - fracstart) * period * .999f);
      g_lp.mat_static[yx] = istatic;
      // Note: this is not working.
    }
  }
  verify_loop_parameters();
}

void do_savepj(Args& args) {
  HH_TIMER(_savepj);
  string filename = args.get_filename();
  assertw(contains(filename, ".pjr"));
  assertx(g_lp.mat_activation.ysize() > 0);
  WFile fi(filename);
  std::ostream& os = fi();
  const Video& v = video;
  for_int(y, v.ysize()) {
    for_int(x, v.xsize()) os << sform("%d\t", g_lp.mat_static[y][x]);
    os << "\n";
  }
  os << "\n";
  for_int(y, v.ysize()) {
    for_int(x, v.xsize()) os << sform("%d\t", g_lp.mat_start[y][x]);
    os << "\n";
  }
  os << "\n";
  for_int(y, v.ysize()) {
    for_int(x, v.xsize()) os << sform("%d\t", g_lp.mat_period[y][x]);
    os << "\n";
  }
  os << "\n";
  for_int(y, v.ysize()) {
    for_int(x, v.xsize()) os << sform("%.6f\t", g_lp.mat_activation[y][x]);
    os << "\n";
  }
  assertx(os);
}

void do_savevlp(Args& args) {
  HH_TIMER(_savevlp);
  dummy_use(args);
  assertnever("not implemented");
}

void do_savestaticloop(Args& args) {
  possibly_rescale_loop_parameters();
  string filename = args.get_filename();
  Image image(video.spatial_dims());
  for (const auto& yx : range(image.dims())) {
    int istatic = g_lp.mat_static[yx];
    bool defined = istatic >= 0 && istatic < video.nframes();
    image[yx] = defined ? video[istatic][yx] : Pixel::pink();
  }
  image.write_file(filename);
}

void do_pjcompression() {
  assertx(g_lp.mat_activation.ysize() > 0);
  Image image(g_lp.mat_start.dims());
  image.set_zsize(4);
  parallel_for_coords(
      g_lp.mat_start.dims(),
      [&](const Vec2<int>& yx) {
        int start = g_lp.mat_start[yx], period = g_lp.mat_period[yx], staticf = g_lp.mat_static[yx];
        float activation = g_lp.mat_activation[yx];
        assertx(start <= 255);
        assertx(period <= 255);
        assertx(staticf <= 255);
        uint8_t uc_activation = clamp_to_uint8(int(activation * 255.f + .5f));
        image[yx] = Pixel(uint8_t(start), uint8_t(period), uint8_t(staticf), uc_activation);
      },
      30);
  image.write_file("pjcompression.png");
}

void do_compressloop() {
  possibly_rescale_loop_parameters();
  const int nnf = video.nframes();
  const bool b_compress_magenta = getenv_bool("LOOP_COMPRESS_MAGENTA");
  const bool b_compress_hold = getenv_bool("LOOP_COMPRESS_HOLD");
  for_coords(video.spatial_dims(), [&](const Vec2<int>& yx) {
    int period = g_lp.mat_period[yx];  // period == 1 for a static pixel
    float deltatime = get_deltatime(period, nnf);
    float new_period = period / deltatime;
    if (period > 1) {
      HH_SSTAT(Snew_period, new_period);
    }
    for_int(f, video.nframes()) {
      if (b_compress_magenta) {
        if (f > new_period) video[f][yx] = Pixel(255, 0, 255);
      } else if (b_compress_hold) {
        if (f > new_period) video[f][yx] = video[int(new_period - .5f)][yx];
      } else {  // decompress
        if (f > new_period) {
          int fi = int(my_mod(float(f), new_period) + .5f);
          video[f][yx] = video[fi][yx];
        }
      }
    }
  });
}

void compute_temporal_costs() {
  possibly_rescale_loop_parameters();
  assertx(max(g_lp.mat_start) > 0);  // cannot evaluate temporal cost if video is already remapped
  g_lp.mat_tcost.init(video.spatial_dims());
  const int onf = video.nframes();
  parallel_for_coords(
      video.spatial_dims(),
      [&](const Vec2<int>& yx) {
        int start = g_lp.mat_start[yx], period = g_lp.mat_period[yx];
        float temporal_cost = float(rgb_dist2(video[(start + period) % onf][yx], video[start][yx]) +
                                    rgb_dist2(video[start + period - 1][yx], video[(start + onf - 1) % onf][yx]));
        if (period == 1) temporal_cost = 0.f;
        g_lp.mat_tcost[yx] = temporal_cost;
      },
      30);
  HH_RSTAT(Stcost, g_lp.mat_tcost);
  if (1) as_image(1.f - standardize_rms(clone(g_lp.mat_tcost)) / 3.f).write_file("image_tcost.png");
}

void compute_looping_regions() {
  HH_TIMER(_compute_looping_regions);
  possibly_rescale_loop_parameters();
  const bool small_looping_regions = getenv_bool("SMALL_LOOPING_REGIONS");  // bad; used in *.wind1
  if (small_looping_regions) assertx(use_activation && g_lp.mat_activation.ysize() > 0);
  assertx(!g_lp.region_color.num());
  {  // compute g_lp.mat_iregion : index of region at each pixel
    UnionFind<Vec2<int>> uf;
    if (0) {
      // Always use case (4) of spatial cost in [Liao et al. 2013] to predict consistency if adjacent
      //  pixels were to advance at different temporal rates.  ongoing initial exploration.
      Image image(video.spatial_dims() * 2 - 1, Pixel::white());
      for_int(y, video.ysize()) for_int(x, video.xsize()) {
        image[y * 2][x * 2] = Pixel(170, 170, 255);  // nodes are colored light-blue; edges are colored below
        for_int(axis, 2) {
          int y1 = y, x1 = x;
          if (axis == 0) {
            y1++;
            if (y1 == video.ysize()) continue;
          } else {
            x1++;
            if (x1 == video.xsize()) continue;
          }
          float scost2 = 0.f;
          for_int(edge_direction, 2) {           // two parts (at adjacent pixels x and z) of the spatial cost term
            std::swap(y, y1), std::swap(x, x1);  // isn't swap dangerous here?
            int start0 = g_lp.mat_start[y][x], period0 = g_lp.mat_period[y][x];
            int start1 = g_lp.mat_start[y1][x1], period1 = g_lp.mat_period[y1][x1];
            Vector p0sum(0.f, 0.f, 0.f), p0sum2(0.f, 0.f, 0.f);
            Vector p1sum(0.f, 0.f, 0.f), p1sum2(0.f, 0.f, 0.f);
            for_intL(f, start0, start0 + period0) for_int(c, 3) {
              p0sum[c] += to_float(video(f, y, x)[c]);
              p0sum2[c] += square(to_float(video(f, y, x)[c]));
            }
            for_intL(f, start1, start1 + period1) for_int(c, 3) {
              p1sum[c] += to_float(video(f, y, x)[c]);
              p1sum2[c] += square(to_float(video(f, y, x)[c]));  // again at same pixel [y][x]
            }
            Vector vmul = Vector(p0sum[0] * p1sum[0], p0sum[1] * p1sum[1], p0sum[2] * p1sum[2]);
            Vector vtot = (1.f / period0) * p0sum2 + (1.f / period1) * p1sum2 - (2.f / (period0 * period1)) * vmul;
            for_int(z, 3) { HH_SSTAT(Stotz, vtot[z]); }
            scost2 += vtot[0] + vtot[1] + vtot[2];
          }
          HH_SSTAT(Sscost2, scost2);
          image[y + y1][x + x1] = Pixel::gray(clamp_to_uint8(int(255.5f - scost2 * .01f)));
          bool both_static = g_lp.mat_period[y][x] == 1 && g_lp.mat_period[y1][x1] == 1;
          if (scost2 > 6000.f || both_static) uf.unify(V(y, x), V(y1, x1));  // 6000.f good?
        }
      }
      image.write_file("image_scost.png");
    } else {  // form regions based on period equality and overlapping time intervals (and optionally activation)
      assertx(max(g_lp.mat_start) > 0);  // input time intervals are lost if video is already remapped
      for (const auto& yx : range(video.spatial_dims())) {
        for_int(axis, 2) {
          Vec2<int> yx1 = yx;
          if (++yx1[axis] == video.dim(axis)) continue;
          if (g_lp.mat_period[yx] != g_lp.mat_period[yx1]) continue;
          if (g_lp.mat_period[yx] != 1) {
            if (g_lp.mat_start[yx] >= g_lp.mat_start[yx1] + g_lp.mat_period[yx1] ||
                g_lp.mat_start[yx1] >= g_lp.mat_start[yx] + g_lp.mat_period[yx])
              continue;
          }
          if (small_looping_regions && g_lp.mat_activation[yx] != g_lp.mat_activation[yx1]) continue;
          uf.unify(yx, yx1);
        }
      }
    }
    g_lp.mat_iregion.init(video.spatial_dims());
    {
      Map<Vec2<int>, int> mpii;
      for (const auto& yx : range(video.spatial_dims())) {
        const Vec2<int>& pi = uf.get_label(yx);
        bool is_new;
        int iregion = mpii.enter(pi, g_lp.region_color.num(), is_new);
        if (is_new) g_lp.region_color.add(1);  // initialized later
        g_lp.mat_iregion[yx] = iregion;
      }
    }
  }
  for (auto& pix : g_lp.region_color) pix = random_color(Random::G);
  if (getenv_bool("LOOPING_REGIONS_IMAGE")) {
    Image image(video.spatial_dims());
    for (const auto& yx : range(video.spatial_dims())) {
      image[yx] = g_lp.region_color[g_lp.mat_iregion[yx]];
    }
    image.write_file("looping_regions.png");
  }
  {
    Array<Homogeneous> arh(g_lp.region_color.num());
    for (const auto& yx : range(video.spatial_dims())) {
      arh[g_lp.mat_iregion[yx]] += Point(float(yx[0]), float(yx[1]), 0.f);
    }
    for (const Homogeneous& h : arh) {
      g_lp.region_centroid.push(to_Point(normalized(h)));
    }
  }
  if (getenv_bool("LOOPING_REGIONS_DIST_IMAGE")) {
    Image image(video.spatial_dims());
    for (const auto& yx : range(video.spatial_dims())) {
      int iregion = g_lp.mat_iregion[yx];
      Pixel pix = g_lp.region_color[iregion];
      const Point& pcentroid = g_lp.region_centroid[iregion];
      float fac = clamp(dist(Point(float(yx[0]), float(yx[1]), 0.f), pcentroid) * .1f, 0.f, 1.f);
      for_int(c, 3) pix[c] = uint8_t(pix[c] * fac);
      image[yx] = pix;
    }
    image.write_file("looping_regions_dist.png");
  }
}

struct NormalDeltaTime {
  float operator()(int, const Vec2<int>&) const {
    assertnever_ret("?");
    return BIGFLOAT;
  }
};

// Func func_dtime(int f, const Vec2<int>& yx) specifies for each output frame f and each pixel yx the
//  desired float time increment (delta time).
template <typename Func = NormalDeltaTime>
void internal_render_loops(int nnf, bool is_remap, Func func_dtime = NormalDeltaTime{}) {
  assertw(tradius > 0);
  possibly_rescale_loop_parameters();
  Video nvideo(nnf, video.spatial_dims());
  nvideo.attrib() = video.attrib();
  if (nvideo.attrib().audio.size()) {
    Warning("Clearing audio");
    nvideo.attrib().audio.clear();
  }
  showf("Rendering looping video of %d frames.\n", nnf);
  const bool pixel_adapted_trad = true;                 // perform temporal crossfading only if temporal cost is high
  Matrix<int> mat_trad(video.spatial_dims(), tradius);  // temporal crossfading radius
  bool have_func_dtime = !std::is_same<Func, NormalDeltaTime>::value;
  if (!have_func_dtime && pixel_adapted_trad) {
    compute_temporal_costs();
    parallel_for_coords(
        video.spatial_dims(),
        [&](const Vec2<int>& yx) {
          int period = g_lp.mat_period[yx];
          bool big_temporal_cost = g_lp.mat_tcost[yx] > 2.f * 3.f * square(10.f);
          int pixtradius = big_temporal_cost ? tradius : 0;  // temporal crossfading radius
          if (0) pixtradius = min(pixtradius, period / 2);
          mat_trad[yx] = pixtradius;
        },
        20);
  }
  if (have_func_dtime) {
    if (max(g_lp.mat_start) > 0) Warning("Should have run remap to obtain temporal crossfading");
  }
  // could be Array<float> indexed by iregion
  Matrix<float> mat_time;
  if (have_func_dtime) {
    mat_time.init(video.spatial_dims(), 0.f);
  }
  for_int(f, nnf) {
    parallel_for_coords(
        nvideo.spatial_dims(),
        [&](const Vec2<int>& yx) {
          const int start = g_lp.mat_start[yx], period = g_lp.mat_period[yx];
          if (is_remap && f >= period) {
            nvideo[f][yx] = false ? Pixel::pink() : nvideo[period - 1][yx];
            return;
          }
          if (have_func_dtime) {
            float deltaftime = func_dtime(f, yx);     // time change for current pixel
            float ftime = mat_time[yx] + deltaftime;  // new time at pixel
            mat_time[yx] = ftime;
            float fi = my_mod(ftime - start, float(period));
            ASSERTX(fi >= float(start) && fi < float(start) + period);
            // SHOW(ftime, fi, start, period); assertnever("");
            float flfi = floor(fi);
            float frfi = fi - flfi;  // weight for linear interpolation over time
            int fi0 = int(flfi);
            // Note: no spatial crossfading
            for_int(z, nz) {
              int fi1 = fi0 + 1 < start + period ? fi0 + 1 : start;
              // (run do_remap() as preprocess if temporal crossfading is desired)
              nvideo[f][yx][z] =
                  uint8_t((1.f - frfi) * to_float(video[fi0][yx][z]) + frfi * to_float(video[fi1][yx][z]) + 0.5f);
            }
          } else {
            int fi = get_framei(float(f), start, period);
            ASSERTX(fi >= start && fi < start + period);
            // Note: no spatial crossfading
            int pixtradius = mat_trad[yx];
            if (!pixtradius) {
              nvideo[f][yx] = video[fi][yx];
            } else {        // temporal crossfading
              int fio;      // other frame with which to blend
              float alpha;  // weight of that other frame
              if (fi - start < pixtradius) {
                fio = fi + period;
                alpha = .5f - .5f * (fi - start + .5f) / pixtradius;
              } else if (start + period - 1 - fi < pixtradius) {
                fio = fi - period;
                alpha = .5f - .5f * (start + period - 1 - fi + .5f) / pixtradius;
              } else {
                fio = fi;
                alpha = 0.f;
              }
              if (0 && sum(yx) == 0) SHOW(f, fi, fio, alpha);
              for_int(z, nz) {
                nvideo[f][yx][z] = uint8_t((1.f - alpha) * video[fi][yx][z] +
                                           alpha * video[clamp(fio, 0, video.nframes() - 1)][yx][z] + .5f);
              }
            }
          }
        },
        100);
  }
  video = std::move(nvideo);
}

void do_remap() {
  HH_TIMER(_remap);
  possibly_rescale_loop_parameters();
  if (max(g_lp.mat_start) == 0) return;  // already remapped (quite possible)
  int nnf = max(g_lp.mat_period);        // maximum period
  internal_render_loops(nnf, true);
  parallel_for_coords(
      video.spatial_dims(),
      [&](const Vec2<int>& yx) {
        g_lp.mat_static[yx] = g_lp.mat_static[yx] % g_lp.mat_period[yx];
        g_lp.mat_start[yx] = 0;
      },
      6);
}

void do_render_loops(Args& args) {
  HH_TIMER(_render_loops);
  int nnf = parse_nframes(args.get_string(), false);
  possibly_rescale_loop_parameters();
  assertw(max(g_lp.mat_start) > 0);                                 // else hopefully already temporally crossfaded
  do_remap();                                                       // for temporal crossfading
  const bool no_stretch_shrink = getenv_bool("NO_STRETCH_SHRINK");  // create original no-global-period result
  // Note that temporally scaling all pixels with the same period by the same scaling value will
  //  have zero effect on the spatial and temporal consistency.
  if (!no_stretch_shrink) {
    Matrix<float> mat_deltatime(video.spatial_dims());
    for (const auto& yx : range(video.spatial_dims())) {
      int period = g_lp.mat_period[yx];
      float deltatime = get_deltatime(period, nnf);
      HH_SSTAT(Sdeltatime, deltatime);
      mat_deltatime[yx] = deltatime;
    }
    auto func_dtime = [&](int f, const Vec2<int>& yx) -> float {
      dummy_use(f);
      return mat_deltatime[yx];
    };
    internal_render_loops(nnf, false, func_dtime);
  } else {
    internal_render_loops(nnf, false);
  }
}

void do_render_wind(Args& args) {
  HH_TIMER(_render_wind);
  int nnf = parse_nframes(args.get_string(), false);
  compute_looping_regions();
  do_remap();                                                // for temporal crossfading
  static const bool no_regions = getenv_bool("NO_REGIONS");  // show lack of phase coherence when done per-pixel
  auto func_dtime = [&](int f, const Vec2<int>& yx) -> float {
    const float ssdv = .25f;
    Point pcentroid = g_lp.region_centroid[g_lp.mat_iregion[yx]];  // (y, x, 0.f)
    if (no_regions) pcentroid = Point(float(yx[0]), float(yx[1]), 0.f);
    float deltaftime = .1f + 1.8f * gaussian(pcentroid[1] / float(video.xsize()) - f / float(nnf), ssdv);
    return deltaftime;
  };
  internal_render_loops(nnf, false, func_dtime);
}

void do_render_harmonize(Args& args) {
  HH_TIMER(_render_harmonize);
  int nnf = parse_nframes(args.get_string(), false);
  compute_looping_regions();
  do_remap();  // for temporal crossfading
  Matrix<float> mat_avgdyn(video.spatial_dims(), 0.f);
  parallel_for_coords(
      video.spatial_dims(),
      [&](const Vec2<int>& yx) {
        int start = g_lp.mat_start[yx], period = g_lp.mat_period[yx];
        for_int(ii, period == 1 ? 0 : period) {
          int fi0 = start + ii + 0;
          int fi1 = start + (ii + 1) % period;
          float d = sqrt(float(rgb_dist2(video[fi0][yx], video[fi1][yx])));
          mat_avgdyn[yx] += d / period;
        }
      },
      30);
  HH_RSTAT(Savgdyn, mat_avgdyn);
  if (1) as_image(1.f - standardize_rms(clone(mat_avgdyn)) / 3.f).write_file("image_pixel_avgdyn.png");
  Array<Stat> region_avgdyn(g_lp.region_color.num());  // for each region, build statistic of dynamism of its pixels
  HH_STAT(Ssumdyn);                                    // average dynamism of all non-static pixels
  for (const auto& yx : range(video.spatial_dims())) {
    int iregion = g_lp.mat_iregion[yx];
    region_avgdyn[iregion].enter(mat_avgdyn[yx]);
    if (mat_avgdyn[yx]) Ssumdyn.enter(mat_avgdyn[yx]);
  }
  if (1) {  // find the average dynamim for each region
    Matrix<float> mat_region_avgdyn(video.spatial_dims());
    for (const auto& yx : range(video.spatial_dims())) {
      int iregion = g_lp.mat_iregion[yx];
      mat_region_avgdyn[yx] = region_avgdyn[iregion].avg();
    }
    as_image(1.f - standardize_rms(mat_region_avgdyn) / 3.f).write_file("image_region_avgdyn.png");
  }
  Array<float> ar_deltaftime(g_lp.region_color.num());
  for_int(iregion, ar_deltaftime.num()) {
    float regiondyn = region_avgdyn[iregion].avg();
    // Rescale this region so its average per-pixel dynamism equals the average dynamism of all non-static pixels.
    float deltaftime = !regiondyn ? 1.f : Ssumdyn.avg() / (regiondyn + .00001f);
    deltaftime = clamp(deltaftime, 1.f / 3.f, 3.f);
    ar_deltaftime[iregion] = deltaftime;
  }
  HH_RSTAT(Sdeltaftime, ar_deltaftime);
  auto func_dtime = [&](int f, const Vec2<int>& yx) -> float {
    dummy_use(f);
    return ar_deltaftime[g_lp.mat_iregion[yx]];
  };
  internal_render_loops(nnf, false, func_dtime);
}

// Remap into a looping video by using gradient-domain stitching.
// e.g.: set d=~/proj/videoloops/data/test; Filtervideo $d/HDbrink8.mp4 -loadvlp $d/HDbrink8_loop.vlp -gdloop 5sec | vidv
// e.g.: cd ~/proj/videoloops/data/ReallyFreakinAll; (setenv VIDEOLOOP_PRECISE; Filtervideo -trunc_frames 215 HDgiant.mp4 -end 7sec -start -5sec -trimend -1 -loadvlp out/HDgiant_loop.vlp -gdloop 5sec -to mp4 >v1.gdloop5sec.mp4)
// e.g.: cd ~/proj/videoloops/data/ReallyFreakinAll/; (setenv VIDEOLOOP_PRECISE; Filtervideo -trunc_frames 215 HDpoolpalms.mp4 -end 7sec -start -5sec -trimend -1 -loadvlp out/HDpoolpalms_loop.vlp -gdloop 5sec -to mp4 >v.gdloop5sec.mp4)
// e.g.: cd ~/proj/videoloops/data/ReallyFreakinAll/; (setenv VIDEOLOOP_NO_BLEND; Filtervideo -trunc_frames 215 HDsquareflags3.mp4 -end 7sec -start -5sec -boundaryrule c -trimend -2 -loadvlp out/HDsquareflags3_loop.vlp -gdloop 5sec -to mp4 >v.gdloop5sec.mp4)
//  (for 4K video, 4.9 GB for input, then max of 9.8 GB; gdloop 5.3sec; write 63 sec; total 84 sec)
void do_gdloop(Args& args) {
  const int nnf = parse_nframes(args.get_string(), false);
  const Vec3<int> dims = max(video.dims(), video_nv12.get_Y().dims());
  assertx(product(dims));
  const Vec2<int> sdims = dims.tail<2>();
  Video videoloop(nnf, sdims);
  videoloop.attrib() = video.attrib();
  if (videoloop.attrib().audio.size()) {
    Warning("Clearing audio");
    videoloop.attrib().audio.clear();
  }
  VideoNv12 dummy_vnv12;
  compute_gdloop(dims, "", video, video_nv12, g_lp.mat_start, g_lp.mat_period, EGDLoopScheme::fast, videoloop.dim(0),
                 nullptr, videoloop, dummy_vnv12, 1);
  video = std::move(videoloop);
  video_nv12.clear();
  parallel_for_coords(
      g_lp.mat_static.dims(),
      [&](const Vec2<int>& yx) {
        g_lp.mat_static[yx] = g_lp.mat_static[yx] % g_lp.mat_period[yx];
        g_lp.mat_start[yx] = 0;
      },
      8);
}

// Remap into a looping video by using gradient-domain stitching.
// e.g.: set d=~/proj/videoloops/data/test; Filtervideo $d/HDbrink8.mp4 -loadvlp $d/HDbrink8_loop.vlp -gdloopfile 5sec - | vidv
// set d=~/proj/videoloops/data/test; Filtervideo $d/seacrowd.wmv -loadvlp $d/seacrowd_loop.vlp -gdloopfile 5sec v.wmv
//  (for 4K video, 4.9 GB for input, then max of 6.5 GB; total 85 sec) (198 sec with multi-stream input)
void do_gdloopfile(Args& args) {
  const int nnf = parse_nframes(args.get_string(), false);
  string loop_filename = args.get_filename();
  const Vec3<int> dims = max(video.dims(), video_nv12.get_Y().dims());
  assertx(product(dims));
  const Vec2<int> sdims = dims.tail<2>();
  const bool use_nv12 = true;
  if (video.attrib().audio.size()) {
    Warning("Clearing audio");
    video.attrib().audio.clear();
  }
  WVideo wvideo(loop_filename, sdims, video.attrib(), use_nv12);
  VideoNv12 dummy_vnv12;
  compute_gdloop(dims, "", video, video_nv12, g_lp.mat_start, g_lp.mat_period, EGDLoopScheme::fast, nnf, &wvideo,
                 Grid<3, Pixel>{}, dummy_vnv12, 1);
  nooutput = true;
}

// Remap into a looping video by using gradient-domain stitching.
// e.g.: set d=~/proj/videoloops/data/test; Filtervideo -create 0 0 0 -loadvlp $d/HDbrink8_loop.vlp -gdloopstream 150  $d/HDbrink8.mp4 v.mp4 && o v.mp4
// e.g.: set d=~/proj/videoloops/data/test; Filtervideo -create 0 0 0 -loadvlp $d/HDbrink8h_loop.vlp -gdloopstream 150  $d/HDbrink8h.mp4 v.mp4 && o v.mp4
// e.g.: set d=~/proj/videoloops/data/test f=M4Kseacrowd.wmv; Filtervideo -create 0 0 0 -loadvlp $d/${f:r}_loop.vlp -gdloopstream 150  $d/$f v.$f:e && o v.$f:e
// e.g.: set d=~/proj/videoloops/data/test f=HDbrink8h.mp4; Filtervideo -create 0 0 0 -loadvlp $d/${f:r}_loop.vlp -gdloopstream 150  $d/$f v.$f:e && o v.$f:e
//  (for 4K video, 2.1 GB max; total 135 sec now with RVideo nv12 format)
void do_gdloopstream(Args& args) {
  int nnf = parse_nframes(args.get_string(), false);
  string video_filename = args.get_filename();
  string loop_filename = args.get_filename();
  Vec3<int> odims;
  Video::Attrib attrib;
  {
    RVideo rvideo(video_filename);
    showf("Reading video %s\n", Video::diagnostic_string(rvideo.dims(), rvideo.attrib()).c_str());
    odims = rvideo.dims();  // this may underestimate or overestimate the number of frames actually in the video
    attrib = rvideo.attrib();
  }
  const bool use_nv12 = true;
  if (attrib.audio.size()) {
    Warning("Clearing audio");
    attrib.audio.clear();
  }
  WVideo wvideo(loop_filename, odims.tail<2>(), attrib, use_nv12);
  const EGDLoopScheme scheme = EGDLoopScheme::fast;
  VideoNv12 dummy_vnv12;
  compute_gdloop(odims, video_filename, Grid<3, Pixel>{}, VideoNv12{}, g_lp.mat_start, g_lp.mat_period, scheme, nnf,
                 &wvideo, Grid<3, Pixel>{}, dummy_vnv12, 1);
  nooutput = true;
}

// Analyze reconstruction error.
// e.g.: cd ~/proj/videoloops/data/test/; Filtervideo HDbrink8h.mp4 -loadvlp HDbrink8h_loop.vlp -gdlooperr 5sec
//   precise:
//    # Serr:               (235008000)         -47:38           av=0.051623918    rms=2.3937273
//   fast (solve for residual):
//    # Serr:               (235008000)         -41:44           av=-0.5756796     rms=1.457088
//     1.5 / 256 = 0.6%
// e.g.: cd ~/proj/videoloops/data/test/; Filtervideo HDbrink8.mp4 -loadvlp HDbrink8_loop.vlp -gdlooperr 5sec
void do_gdlooperr(Args& args) {
  const int nnf = parse_nframes(args.get_string(), false);
  const Vec3<int> dims = max(video.dims(), video_nv12.get_Y().dims());
  assertx(product(dims));
  const Vec2<int> sdims = dims.tail<2>();
  VideoNv12 dummy_vnv12;
  Video videoloop_exact(nnf, sdims);
  videoloop_exact.attrib() = video.attrib();
  compute_gdloop(dims, "", video, video_nv12, g_lp.mat_start, g_lp.mat_period, EGDLoopScheme::exact,
                 videoloop_exact.dim(0), nullptr, videoloop_exact, dummy_vnv12, 1);
  SHOWL;
  EGDLoopScheme scheme = 1 ? EGDLoopScheme::fast : EGDLoopScheme::precise;
  Video videoloop(nnf, sdims);
  videoloop.attrib() = video.attrib();
  compute_gdloop(dims, "", video, video_nv12, g_lp.mat_start, g_lp.mat_period, scheme, videoloop.dim(0), nullptr,
                 videoloop, dummy_vnv12, 1);
  // (Unfortunately, we are comparing data after it has already been quantized to 8-bit.)
  for_int(f, videoloop.nframes()) for (const auto& yx : range(videoloop.spatial_dims())) {
    for_int(c, 3) { HH_SSTAT_RMS(Serr, videoloop[f][yx][c] - videoloop_exact[f][yx][c]); }
  }
  nooutput = 1;
}

void do_saveloopframe(Args& args) {
  possibly_rescale_loop_parameters();
  assertx(max(g_lp.mat_start) > 0);
  const int f = parse_nframes(args.get_string(), true);
  string filename = args.get_filename();
  Image image(video.spatial_dims());
  for (const auto& yx : range(image.dims())) {
    int fi = get_framei(float(f), g_lp.mat_start[yx], g_lp.mat_period[yx]);
    HH_SSTAT(Sfi, fi);
    image[yx] = video[fi][yx];
  }
  image.write_file(filename);
  nooutput = true;
}

// ***

void process_gen(Args& args) {
  HH_TIMER(_gen);
  // see ~/proj/fiberpatterns/Notes.txt
  // Filtervideo -create 180 1024 768 -procedure gen box_y -to mp4 -framerate 30 -bitrate 10m | vidv
  assertx(video.size());
  string name = args.get_string();
  int ncolors = 3;
  if (name == "slits3" || name == "slits4" || name == "stars5") ncolors = 5;
  auto func_get_color = [&](int i) {
    static Array<Pixel> ar_colors = {
        Pixel::red(), Pixel::green(), Pixel::blue(), Pixel(255, 255, 0), Pixel(0, 255, 255),
    };
    i = i % ncolors;
    return ar_colors[i];
  };
  if (name == "cos_x") {
    float speriod = 100.f;  // was 20.f then 50.f
    float tperiod = 20.f;   // was 20.f then 60.f
    parallel_for_each(range(video.nframes()), [&](const int f) {
      for (const auto& yx : range(video.spatial_dims())) {
        float v = cos((yx[1] / speriod - f / tperiod) * TAU) * .5f + .5f;
        video[f][yx] = Pixel::gray(uint8_t(v * 255.f + .5f));
      }
    });
  } else if (name == "box_y") {
    float speriod = 100.f;  // was 20.f then 50.f
    float tperiod = 20.f;   // was 20.f then 60.f
    parallel_for_each(range(video.nframes()), [&](const int f) {
      for (const auto& yx : range(video.spatial_dims())) {
        float v = (frac(yx[0] / speriod - f / tperiod) > .5f ? 1.f : 0.f);
        video[f][yx] = Pixel::gray(uint8_t(v * 255.f + .5f));
      }
    });
  } else if (begins_with(name, "checker")) {
    float speriod = 100.f;  // was 20.f then 50.f
    float tperiod = 45.f;
    float sradius = .3f;
    if (0) {
    } else if (name == "checker1") {
      tperiod = 20.f;
    } else if (name == "checker2") {
      sradius = .3f;
    } else if (name == "checker3") {
      sradius = .3f;
    } else
      assertnever("");
    int mode;
    assertx(sscanf(name.c_str(), "checker%d", &mode) == 1);
    float motion_amplitude = speriod * 1.5f;
    parallel_for_each(range(video.nframes()), [&](const int f) {
      fill(video[f], Pixel::black());
      if (name == "checker3")
        fill(video[f], Pixel::gray(uint8_t(abs(float(f) / video.nframes() - .5) * 2.f * 255.f + .5f)));
      for (const auto& yx : range(video.spatial_dims())) {
        auto p = (convert<float>(yx) + V(sin(f / tperiod * TAU), cos(f / tperiod * TAU)) * motion_amplitude) / speriod;
        auto pf = p - floor(p);
        auto pi = convert<int>(floor(p));
        float r = float(mag(pf - V(.5f, .5f)));
        Pixel& pix = video[f][yx];
        if (mode == 1 && (pi[0] + pi[1]) % 2 == 0) pix = Pixel::white();
        if (mode == 2 && r < sradius) pix = Pixel::white();
        if (mode == 3 && r < sradius) pix = func_get_color(pi[0] + pi[1] * 37);
      }
    });
  } else if (name == "slit1_x") {
    float speriod = 40.f;
    float tperiod = 45.f;
    parallel_for_each(range(video.nframes()), [&](const int f) {
      for (const auto& yx : range(video.spatial_dims())) {
        float v = frac(float(yx[1]) / video.xsize() - f / tperiod) < speriod / video.xsize() ? 1.f : 0.f;
        video[f][yx] = Pixel::gray(uint8_t(v * 255.f + .5f));
      }
    });
  } else if (name == "slit1_y") {
    float speriod = 80.f;
    float tperiod = 45.f;
    parallel_for_each(range(video.nframes()), [&](const int f) {
      for (const auto& yx : range(video.spatial_dims())) {
        float v = frac(float(yx[0]) / video.ysize() + f / tperiod) < speriod / video.ysize() ? 1.f : 0.f;
        video[f][yx] = Pixel::gray(uint8_t(v * 255.f + .5f));
      }
    });
  } else if (begins_with(name, "slits")) {
    float nsperiods = 3.f;
    float fsperiod = .25f;
    float tperiod = 18.f;
    float rotperiod = 90.f;
    parallel_for_each(range(video.nframes()), [&](const int f) {
      for (const auto& yx : range(video.spatial_dims())) {
        float t = float(yx[0]);
        if (name == "slits4")
          t = (yx[0] - video.ysize() / 2.f) * cos(f / rotperiod * TAU) -
              (yx[1] - video.xsize() / 2.f) * sin(f / rotperiod * TAU);
        float v = t * nsperiods / video.ysize() + f / tperiod;
        Pixel& pix = video[f][yx];
        pix = Pixel::black();
        int i = int(floor(v));
        if (frac(v) < fsperiod) {
          if (name == "slits1")
            pix = Pixel::white();
          else
            pix = func_get_color(i);
        }
      }
    });
  } else if (begins_with(name, "stars")) {
    int n;           // number of stars
    float radius;    // in pixels
    float velrange;  // fraction of screen extent
    if (0) {
    } else if (name == "stars1") {
      n = 200;
      radius = 25.f;
      velrange = .03f;
    } else if (name == "stars2") {
      n = 100;
      radius = 30.f;
      velrange = .03f;
    } else if (name == "stars3") {
      n = 300;
      radius = 1e10f;
      velrange = .03f;
    } else if (name == "stars4") {
      n = 3;
      radius = 40.f;
      velrange = .05f;
    } else if (name == "stars5") {
      n = 200;
      radius = 25.f;
      velrange = .03f;
    } else
      assertnever("");
    fill(video, Pixel::black());  // default black usually
    using F2 = Vec2<float>;
    Array<F2> ar_point0(n);
    Array<F2> ar_velocity(n);
    Array<Pixel> ar_color(n, Pixel::black());
    for_int(i, n) {
      for_int(c, 2) ar_point0[i][c] = Random::G.unif();
      for_int(c, 2) ar_velocity[i][c] = (Random::G.unif() * 2.f - 1.f) * velrange;
      auto& pix = ar_color[i];
      if (0) {
        for (;;) {
          for_int(c, 3) pix[c] = static_cast<uint8_t>(20.f + Random::G.unif() * 235.f + .5f);
          if (mag(pix) > 350 && max(pix) > 150 && min(pix) < 100) break;
        }
      } else {
        pix = func_get_color(i);
      }
    }
    parallel_for_each(range(video.nframes()), [&](const int f) {
      Matrix<F2> mvec(video.spatial_dims(), twice(1e10f));
      Matrix<Pixel> mpixel(video.spatial_dims(), Pixel::pink());
      if (name == "stars4")
        fill(video[f], Pixel::gray(uint8_t(abs(float(f) / video.nframes() - .5) * 2.f * 255.f + .5f)));
      if (name == "stars5")
        fill(video[f],
             Pixel::gray(uint8_t(max(1.f - abs(frac(f * 4.f / video.nframes()) - .5f) * 4.f, 0.f) * 255.f + .5f)));
      for_int(i, n) {
        F2 p = ar_point0[i] + ar_velocity[i] * float(f);
        for_int(c, 2) p[c] = frac(p[c]);  // periodic boundaries
        F2 pp = p * F2(mvec.ysize() - 1.f, mvec.xsize() - 1.f) + .5f;
        int y0 = int(pp[0]);
        int x0 = int(pp[1]);
        for_intL(yy, y0, min(y0 + 2, mvec.ysize())) for_intL(xx, x0, min(x0 + 2, mvec.xsize())) {
          F2 vt = pp - F2(float(yy), float(xx));
          if (mag2(vt) < mag2(mvec(yy, xx))) {
            mvec(yy, xx) = vt;
            mpixel(yy, xx) = ar_color[i];
          }
        }
      }
      euclidean_distance_map(mvec);
      for (const auto& yx : range(video.spatial_dims())) {
        float v = float(mag(mvec[yx]));
        if (v > radius) continue;
        auto yxo = convert<int>(convert<float>(yx) + mvec[yx] + .5f);
        video[f][yx] = mpixel[yxo];
      }
    });
  } else if (name == "dot1") {
    float radius = 8.f;
    parallel_for_each(range(video.nframes()), [&](const int f) {
      for (const auto& yx : range(video.spatial_dims())) {
        auto pc = V((.2f + .6f * f / float(video.nframes())) * video.xsize(), .5f * video.ysize());
        float r = float(dist(convert<float>(yx), pc));
        video[f][yx] = Pixel::gray(r < radius ? 255 : 0);
      }
    });
  } else if (name == "list") {
    for (const string& s : {"cos_x", "box_y", "checker1", "checker2", "checker3", "slit1_x", "slit1_y", "slits1",
                            "slits2", "slits3", "slits4", "stars1", "stars2", "stars3", "stars4", "stars5", "dot1"})
      std::cout << s << "\n";
  } else {
    assertnever("");
  }
}

Matrix<bool> get_black_border_mask(CMatrixView<Pixel> image) {
  const Vec2<int> dims = image.dims();
  Matrix<bool> mask(dims, false);
  for_coords(dims, [&](const Vec2<int>& yx) {
    const Pixel& pixel = image[yx];
    bool is_black = true;
    for_int(c, 3) {
      if (pixel[c] >= 80) is_black = false;
    }
    mask[yx] = is_black;
  });
  // Instead, grow BFS from all 4 images sides.
  return mask;
}

void do_procedure(Args& args) {
  string name = args.get_string();
  if (0) {
  } else if (name == "rotating_grid") {
    const float degrees_per_frame = 8.f;
    const float pixels_per_grid = 9.f;
    for_int(f, video.nframes()) {
      Frame frame;
      {
        float ang = to_rad(f * degrees_per_frame);
        frame =
            (Frame::translation(V(-video.xsize() / 2.f, -video.ysize() / 2.f, 0.f)) *
             Frame::rotation(2, -to_rad(ang)) * Frame::scaling(V(1.f / pixels_per_grid, 1.f / pixels_per_grid, 1.f)));
      }
      parallel_for_coords(
          video.spatial_dims(),
          [&](const Vec2<int>& yx) {
            Point p = Point(float(yx[1]), float(yx[0]), 0.f) * frame;
            // rather inexact: box-filtering against non-rotated pixel square
            float flx = floor(p[0]);
            float fracx = p[0] - flx;
            float fly = floor(p[1]);
            float fracy = p[1] - fly;
            float bx = max(0.f, .5f - min(fracx, 1.f - fracx) * pixels_per_grid);
            float by = max(0.f, .5f - min(fracy, 1.f - fracy) * pixels_per_grid);
            float finterior = 1.f - bx - by + 3.f * bx * by;
            bool even = (int(flx) + int(fly)) % 2 == 0;
            if (!even) finterior = 1.f - finterior;
            video[f][yx] = Pixel::gray(uint8_t(finterior * 255.f + .5f));
          },
          50);
    }
  } else if (name == "slow_value_drift") {
    const float period = 60.f;     // in frames
    const float magnitude = 25.f;  // in [0..255] range
    const float sradius = 60.f;    // in pixels
    for_int(f, video.nframes()) {
      parallel_for_coords(
          video.spatial_dims(),
          [&](const Vec2<int>& yx) {
            float d = float(mag(convert<float>(yx) - convert<float>(video.spatial_dims()) * .5f));
            float vdrift = sin(f / period * TAU) * magnitude * smooth_step(clamp(1.f - d / sradius, 0.f, 1.f));
            for_int(z, nz) video[f][yx][z] = clamp_to_uint8(int(video[f][yx][z] + vdrift + .5f));
          },
          20);
    }
  } else if (name == "loop_2") {  // loop all but first and last frames
    assertx(video.nframes() > 2);
    g_lp.mat_start.init(video.spatial_dims());
    g_lp.mat_period.init(video.spatial_dims());
    g_lp.mat_static.init(video.spatial_dims());
    for (const auto& yx : range(video.spatial_dims())) {
      g_lp.mat_start[yx] = 1;
      g_lp.mat_period[yx] = video.nframes() - 2;
      g_lp.mat_static[yx] = g_lp.mat_start[yx];
    }
  } else if (name == "sanity_gaussian") {
    // Filtervideo -create 12 12 12 -procedure sanity_gaussian -noo
    for_intL(i, -20, 21) {
      float f = i / 5.f;
      showf("gauss(%10f)=%10f\n", f, gaussian(f, 1.f));
    }
    double sum = 0.;
    int n = 100000;
    float samp = 10.f;
    for_int(i, n) {
      float f = (float(i - n / 2) / (n / samp));
      sum += gaussian(f, 1.f);
    }
    SHOW(sum / (n / samp));
  } else if (name == "gen") {
    process_gen(args);
  } else if (name == "black_to_red") {
    // Combination of omp and lambda capture causes internal error in VS2015, in both Debug and Release.
    // parallel_for_each(range(video.nframes()), [&](const int f) {
    for_int(f, video.nframes()) {
      Matrix<bool> mask = get_black_border_mask(video[f]);
      for_coords(video.spatial_dims(), [&](const Vec2<int>& yx) {
        if (mask[yx]) video[f][yx] = Pixel::red();
      });
    }
  } else if (name == "black_border_copy_frame0") {
    for_int(f, video.nframes()) {
      Matrix<bool> mask = get_black_border_mask(video[f]);
      for_coords(video.spatial_dims(), [&](const Vec2<int>& yx) {
        if (mask[yx]) video[f][yx] = video[0][yx];
      });
    }
  } else if (name == "black_border_blend_frame0") {
    const Vec2<int> dims = video.spatial_dims();
    const float screening_weight = 1e-3f;
    Matrix<Vector4> grid0;
    for_int(f, video.nframes()) {
      MatrixView<Pixel> videof = video[f];
      Matrix<Vector4> gridf(dims);
      parallel_for_coords(dims, [&](const Vec2<int>& yx) { gridf[yx] = Vector4(videof[yx]); });
      Matrix<bool> mask = get_black_border_mask(videof);
      // Matrix<bool> mask(dims);
      // parallel_for_coords(dims, [&](const Vec2<int>& yx) { mask[yx] = videof[yx] == Pixel::black(); });
      SHOW(sum(mask));
      if (f == 0) {
        grid0 = gridf;
        continue;  // frame 0 is assumed to contain no black border
      }
      Multigrid<2, Vector4> multigrid(dims);
      {
        auto func_stitch = [&](int y0, int x0, int y1, int x1, Vector4& vrhs) {  // change to yx0, yx1
          if (mask[y0][x0] || mask[y1][x1])
            vrhs += grid0[y1][x1] - grid0[y0][x0];
          else
            vrhs += gridf[y1][x1] - gridf[y0][x0];
        };
        const int ny = dims[0], nx = dims[1];
        parallel_for_each(range(ny), [&](const int y) {
          for_int(x, nx) {
            Vector4 vrhs = -screening_weight * (mask[y][x] ? grid0 : gridf)[y][x];
            if (y > 0) {
              func_stitch(y, x, y - 1, x + 0, vrhs);
            }
            if (y < ny - 1) {
              func_stitch(y, x, y + 1, x + 0, vrhs);
            }
            if (x > 0) {
              func_stitch(y, x, y + 0, x - 1, vrhs);
            }
            if (x < nx - 1) {
              func_stitch(y, x, y + 0, x + 1, vrhs);
            }
            multigrid.rhs()[y][x] = vrhs;
          }
        });
      }
      Vector4 vmean(0.f);
      {
        MatrixView<Vector4> mest = multigrid.initial_estimate();
        parallel_for_coords(dims, [&](const Vec2<int>& yx) { mest[yx] = (mask[yx] ? grid0 : gridf)[yx]; });
        for_coords(dims, [&](const Vec2<int>& yx) { vmean += mest[yx]; });
        vmean /= float(product(dims));
      }
      multigrid.set_screening_weight(screening_weight);
      multigrid.set_desired_mean(vmean);
      multigrid.set_num_vcycles(3);
      if (0) multigrid.set_verbose(true);
      multigrid.solve();
      parallel_for_coords(dims, [&](const Vec2<int>& yx) { videof[yx] = multigrid.result()[yx].pixel(); });
    }
  } else if (name == "nv12tomain") {
    // Filtervideo -readnv12 ~/tmp/v.wmv -proc nv12tomain -stat
    assertx(video_nv12.nframes() && !video.nframes());
    video.init(video_nv12.get_Y().dims());
    convert_VideoNv12_to_Video(video_nv12, video);
  } else {
    args.problem("procedure '" + name + "' unrecognized");
  }
}

void do_diff(Args& args) {
  string filename = args.get_filename();
  Video video2;
  video2.read_file(filename);
  assertx(same_size(video, video2));
  parallel_for_coords(
      video.dims(),
      [&](const Vec3<int>& fyx) {
        for_int(z, nz) video[fyx][z] = clamp_to_uint8(128 + int(video[fyx][z]) - int(video2[fyx][z]));
      },
      20);
}

void do_transf(Args& args) {
  Frame frame = FrameIO::parse_frame(args.get_string());
  parallel_for_each(range(video.nframes()), [&](const int f) {
    for (Pixel& pix : video[f]) {
      Point p(0.f, 0.f, 0.f);
      for_int(z, nz) p[z] = pix[z] / 255.f;
      p *= frame;
      for_int(z, nz) pix[z] = uint8_t(clamp(p[z], 0.f, 1.f) * 255.f + .5f);
    }
  });
}

void do_noisegaussian(Args& args) {
  float sd = args.get_float();
  for (Pixel& pix : video) {
    for_int(z, nz) pix[z] = clamp_to_uint8(int(to_float(pix[z]) + Random::G.gauss() * sd + .5f));
  }
}

Vector frame_median(CMatrixView<Pixel> frame) {
  Vector vmedian;
  Array<float> ar_tmp;
  ar_tmp.reserve(assert_narrow_cast<int>(product(frame.dims())));
  for_int(z, nz) {
    ar_tmp.init(0);
    for (const Pixel& pix : frame) ar_tmp.push(pix[z]);
    vmedian[z] = float(median(ar_tmp));
  }
  return vmedian;
}

Vector frame_mean(CMatrixView<Pixel> frame) {
  Array<Stat> stat_pixels(nz);
  for (const Pixel& pix : frame) {
    for_int(z, nz) stat_pixels[z].enter(pix[z]);
  }
  return Vector(stat_pixels[0].avg(), stat_pixels[1].avg(), stat_pixels[2].avg());
}

void do_equalizemedians() {
  Array<Vector> ar_medians;
  for_int(f, video.nframes()) ar_medians.push(frame_median(video[f]));
  Vector mean_of_medians = mean(ar_medians);
  SHOW(mean_of_medians);
  for (auto& vec : ar_medians) vec -= mean_of_medians;
  for_int(f, video.nframes()) for (Pixel& pix : video[f]) {
    for_int(z, nz) pix[z] = clamp_to_uint8(int(pix[z] - ar_medians[f][z] + .5f));
  }
}

void do_equalizemeans() {
  Array<Vector> ar_means;
  for_int(f, video.nframes()) ar_means.push(frame_mean(video[f]));
  Vector mean_of_means = mean(ar_means);
  SHOW(mean_of_means);
  for (auto& vec : ar_means) vec -= mean_of_means;
  for_int(f, video.nframes()) for (Pixel& pix : video[f]) {
    for_int(z, nz) pix[z] = clamp_to_uint8(int(pix[z] - ar_means[f][z] + .5f));
  }
}

void do_frameinfo() {
  for_int(f, video.nframes()) {
    Vector median = frame_median(video[f]);
    Vector mean = frame_mean(video[f]);
    showf(" frame %-3d mean=(%6.2f %6.2f %6.2f) median=(%3g %3g %3g)\n", f, mean[0], mean[1], mean[2], median[0],
          median[1], median[2]);
  }
}

}  // namespace

int main(int argc, const char** argv) {
  my_setenv("NO_DIAGNOSTICS_IN_STDOUT", "1");
  HH_TIMER(Filtervideo);
  ParseArgs args(argc, argv);
  HH_ARGSC("", ": (Video coordinates: (x = 0, y = 0) at left, top)");
  HH_ARGSC("", ":A video is automatically read from stdin except with the following arguments:");
  HH_ARGSD(nostdin, ": do not attempt to read input video from stdin");
  HH_ARGSD(create, "nframes width height : create video (default white)");
  HH_ARGSD(readnv12, "filename : read video into NV12 grids rather than RGB grid");
  HH_ARGSD(trunc_begin, "nframes : skip the first nframes frames");
  HH_ARGSD(trunc_frames, "nframes : read no more than nframes frames");
  HH_ARGSD(as_fit, "nx ny : when assembling, scale each frame uniformly to fit into this size");
  HH_ARGSD(as_cropsides, "l r t b : when assembling, crop each frame");
  HH_ARGSD(as_tnframes, "nf : when assembling, temporally scale to specified number of frames");
  HH_ARGSD(assemble, "nx ny videos_lr_bt_order : concatenate grid of videos");
  HH_ARGSP(startframe, "i : start %d numbering at i");
  HH_ARGSD(fromimages, "rootname.%03d.png : read frame images");
  HH_ARGSC("", ":");
  HH_ARGSD(framerate, "fps : set video frame rate");
  HH_ARGSD(bitrate, "bps : set video bit rate (bits / sec)");
  HH_ARGSD(to, "suffix : set output format (mp4, wmv)");
  HH_ARGSD(bpp, "v : set video bit rate (bits / pixel)");
  HH_ARGSD(outfile, "filename : output an intermediate video");
  HH_ARGSD(append, "filename : append another video");
  HH_ARGSD(toimages, "rootname.%03d.png : output frame images");
  HH_ARGSD(writeframe, "frameindex : output selected frame (0 == first) as png image");
  HH_ARGSF(nooutput, ": do not output final video on stdout");
  HH_ARGSD(info, ": print video statistics");
  HH_ARGSD(noaudio, ": remove any audio data");
  HH_ARGSD(frameinfo, ": print per-frame statistics");
  HH_ARGSD(stat, ": equivalent to -info -nooutput");
  HH_ARGSD(sizes, ": print 'nframes width height'");
  HH_ARGSD(tscale, "fac : temporal scaling");
  HH_ARGSD(tnframes, "nframes : temporally scale to specified number of frames");
  HH_ARGSD(start, "frameindex : trim video prior  (frames or time) (eg. 30 or -1 or 2s or 10%)");
  HH_ARGSD(end, "frameindex : trim video beyond (frames or time) (eg. 30 or -1 or 2s or 10%)");
  HH_ARGSD(interval, "f1 f2 : trim video to interval [f1, f2)");
  HH_ARGSD(trimbeg, "nframes : temporally crop beginning using boundaryrule (nframes < 0 to extend)");
  HH_ARGSD(trimend, "nframes : temporally crop end using boundaryrule (nframes < 0 to extend)");
  HH_ARGSD(loop, "n : repeat n times (1=no_op)");
  HH_ARGSD(mirror, "n : repeat n times (n is even to make periodic; =2 for simple mirror)");
  HH_ARGSD(reverse, ": reverse order of frames");
  HH_ARGSD(phaseoffset, "startframe : apply circular shift to frames of loop");
  HH_ARGSP(tradius, "nframes : set temporal crossfade radius");
  HH_ARGSD(tcrossfade, "fbegin fend : temporal crossfade to create loop");
  HH_ARGSD(makeloop, "fbegin fend : gradient-domain loop");
  HH_ARGSC("", ":");
  HH_ARGSD(color, "r g b : set default color for various operations (default 255's)");
  HH_ARGSD(not, ": negate test for color selection");
  HH_ARGSD(getcolorfxy, "f x y : get color from pixel ((0, 0, 0)=(first_frame, left, top))");
  HH_ARGSD(boundaryrule, "c : reflected/periodic/clamped/border (for scaling or negative trim/crop)");
  HH_ARGSC("", ":");
  HH_ARGSD(cropsides, "l r t b : crop frames (introduce default color if negative)");
  HH_ARGSD(cropl, "l : crop frames");
  HH_ARGSD(cropr, "r : crop frames");
  HH_ARGSD(cropt, "t : crop frames");
  HH_ARGSD(cropb, "b : crop frames");
  HH_ARGSD(cropall, "i : crop all frame sides");
  HH_ARGSD(cropsquare, "x y size : crop frame square centered at (x, y)");
  HH_ARGSD(croprectangle, "x y xsize ysize : crop frame rectangle centered at (x, y)");
  HH_ARGSD(cropcoord, "x0 y0 x1 y1 : crop frame box within bounds x0<=x<x1");
  HH_ARGSD(croptodims, "x y : centered crop to obtain new dimensions");
  HH_ARGSD(cropmult, "fac : increase dimensions to be multiple of fac");
  HH_ARGSC("", ":");
  HH_ARGSD(filter, "c : imp/box/tri/quad/mitchell/keys/spline/omoms/gauss/preprocess/justspline");
  HH_ARGSD(scaleunif, "fac : zoom video (upsample and/or downsample)");
  HH_ARGSD(scalenonunif, "facx facy : zoom video");
  HH_ARGSD(scaletox, "x : uniform scale to x width");
  HH_ARGSD(scaletoy, "y : uniform scale to y height");
  HH_ARGSD(scaleifgtmax, "n : uniform scale such that max(width, height)<=n");
  HH_ARGSD(scaletodims, "x y : non-uniform scale");
  HH_ARGSD(scaleinside, "x y : uniform scale to become no larger than rectangle");
  HH_ARGSC("", ":");
  HH_ARGSD(flipvertical, ": reverse rows");
  HH_ARGSD(fliphorizontal, ": reverse columns");
  HH_ARGSD(disassemble, "tilex tiley rootname : break up into multiple video files of this size");
  HH_ARGSD(gridcrop, "nx ny sizex sizey : assemble grid of regions (with as_cropsides)");
  HH_ARGSC("", ":");
  HH_ARGSD(replace, "r g b : replace all pixels matching specified color with this color");
  HH_ARGSD(gamma, "v : gammawarp video");
  HH_ARGSC("", ":");
  HH_ARGSD(loadpj, "file.pj{o,r} : read progressive video project file");
  HH_ARGSD(loadvlp, "file.vlp : read progressive video project file");
  HH_ARGSD(savepj, "file.pj{o,r} : write progressive video project file");
  HH_ARGSD(savevlp, "file.vlp : write progressive video project file");
  HH_ARGSD(savestaticloop, "imagefile : write image for static loop");
  HH_ARGSD(pjcompression, ": analyze compression");
  HH_ARGSD(compressloop, ": compress/decompress loop");
  HH_ARGSD(remap, ": make all loops start at beginning and pad resulting video");
  HH_ARGSD(render_loops, "nframes : output a video with looping content");
  HH_ARGSD(render_wind, "nframes : introduce temporal warping to simulate wind");
  HH_ARGSD(render_harmonize, "nframes : try to make dynamism uniform");
  HH_ARGSD(gdloop, "nframes : create remapped loop with gradient-domain crossfading");
  HH_ARGSD(gdloopfile, "nframes output_file : same but stream directly to output file");
  HH_ARGSD(gdloopstream, "nframes input_file output_file : same but stream both input and output");
  HH_ARGSD(gdlooperr, "nframes : analyze solver error");
  HH_ARGSD(saveloopframe, "frameindex imagename : output specified frame of the video loop");
  HH_ARGSC("", ":");
  HH_ARGSD(procedure, "name... : apply named procedure to video");
  HH_ARGSD(diff, "video2 : compute difference 128 + video - video2");
  HH_ARGSD(transf, "'frame' : post-multiply RGB vector by a matrix (ranges [0..1])");
  HH_ARGSD(noisegaussian, "sd : introduce white Gaussian noise (in range 0..255)");
  HH_ARGSD(equalizemedians, ": shift image values at each frame to make frame medians identical");
  HH_ARGSD(equalizemeans, ": shift image values at each frame to make frame means identical");
  for (;;) {
    if (args.num() >= 2 && args.peek_string() == "-trunc_begin") {
      args.get_string();
      trunc_begin = args.get_int();
    } else if (args.num() >= 2 && args.peek_string() == "-trunc_frames") {
      args.get_string();
      trunc_frames = args.get_int();
    } else {
      break;
    }
  }
  string arg0 = args.num() ? args.peek_string() : "";
  if (!ParseArgs::special_arg(arg0) && arg0 != "-nostdin" && arg0 != "-create" && !begins_with(arg0, "-as") &&
      arg0 != "-startframe" && arg0 != "-fromimages" && arg0 != "-readnv12") {
    string filename = "-";
    if (args.num() && (arg0 == "-" || arg0[0] != '-')) filename = args.get_filename();
    read_video(filename, false);
  }
  args.parse();
  hh_clean_up();
  if (!nooutput) {
    if (video.size())
      video.write_file("-");
    else if (video_nv12.size())
      video_nv12.write_file("-", video.attrib());
  }
  return 0;
}
