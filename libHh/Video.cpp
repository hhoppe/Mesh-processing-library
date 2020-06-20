// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Video.h"

#include "libHh/GridPixelOp.h"  // spatially_scale_Grid3_Pixel()
#include "libHh/StringOp.h"     // get_path_extension(), to_lower()

namespace hh {

void Video::init(const Vec3<int>& dims) {
  base::init(dims);
  if (0) {
    // required for Nv12/YUV support, but VideoViewer uses Video as a container for arbitrary images
    assertx(xsize() % 2 == 0 && ysize() % 2 == 0);
  }
}

string Video::diagnostic_string(const Vec3<int>& dims, const Attrib& attrib) {
  string s = sform("nframes~%d %dx%d (%gfps)", dims[0], dims[2], dims[1], attrib.framerate);
  if (attrib.suffix != "") s += " (" + attrib.suffix + ")";
  int brate = attrib.bitrate;
  if (brate)
    s += (brate > 1000000 ? sform(" (%.2fMi bps)", brate / 1000000.f)
                          : brate > 1000 ? sform(" (%.2fKi bps)", brate / 1000.f) : sform(" (%d bps)", brate));
  return s;
}

void Video::scale(const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs, const Pixel* bordervalue) {
  *this = hh::scale(*this, syx, filterbs, bordervalue, std::move(*this));
}

//----------------------------------------------------------------------------

bool filename_is_video(const string& filename) {
  static const auto& k_extensions = *new Array<string>{
      "mpg", "mpeg", "mpe",  "mpv", "mp2", "mpeg2", "mp4", "mpeg4", "mov", "avi", "wmv", "flv",
      "m2v", "m4v",  "webm", "ogv", "3gp", "mts",   "gif", "vob",   "qt",  "asf", "3g2", "webm",
  };
  return k_extensions.index(to_lower(get_path_extension(filename))) >= 0;
}

string video_suffix_for_magic_byte(uchar c) {
  // see also image_suffix_for_magic_byte() and audio_suffix_for_magic_byte()
  // Documentation on prefixes for various video containers:
  //  *.mp4: "\000\000\000\030ftypmp42", "\000\000\000 ftypisom", "\000\000\000\034ftypisom"
  //  *.wmv: "0&\262u"
  //  *.avi: "RIFF"
  //  *.mov: "\000\000\000\030ftypqt   \a\t\000"
  //  *.webm: \0x1a\0x45\0xdf\0xa3
  // see https://en.wikipedia.org/wiki/List_of_file_signatures
  switch (c) {
    case 0: return "mp4";  // u'\x00'; or "mov"
    case '0': return "wmv";
    case 'R': return "avi";
    case 'G': return "gif";
    case '\x1a': return "webm";
    default: return "";
  }
}

//----------------------------------------------------------------------------

void convert_VideoNv12_to_Video(CVideoNv12View vnv12, GridView<3, Pixel> video) {
  assertx(vnv12.nframes() == video.dim(0));
  parallel_for_each(range(video.dim(0)), [&](const int f) { convert_Nv12_to_Image(vnv12[f], video[f]); });
}

void convert_Video_to_VideoNv12(CGridView<3, Pixel> video, VideoNv12View vnv12) {
  assertx(vnv12.nframes() == video.dim(0));
  parallel_for_each(range(video.dim(0)), [&](const int f) { convert_Image_to_Nv12(video[f], vnv12[f]); });
}

//----------------------------------------------------------------------------

Video scale(const Video& video, const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs, const Pixel* bordervalue,
            Video&& pnewvideo) {
  // With Grid<3, Pixel> rather than Array<Matrix<Pixel>>, we cannot directly overwrite video in-place.
  Video newvideo = &pnewvideo == &video ? Video() : std::move(pnewvideo);
  assertx(min(syx) >= 0.f);
  Vec2<int> newdims = convert<int>(convert<float>(video.spatial_dims()) * syx + .5f);
  if (video.attrib().suffix != "avi") newdims = (newdims + 1) / 2 * 2;  // make sizes be even integers
  if (!product(newdims)) {
    Warning("scaling to zero-sized frame");
    if (0) fill(newdims, 0);
  }
  newvideo.init(video.nframes(), newdims);
  newvideo.attrib() = video.attrib();
  if (!product(newdims)) return newvideo;
  spatially_scale_Grid3_Pixel(video, filterbs, bordervalue, newvideo);
  return newvideo;
}

VideoNv12 scale(const VideoNv12& video_nv12, const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs,
                const Pixel* bordervalue, VideoNv12&& pnewvideo_nv12) {
  VideoNv12 newvideo_nv12 = &pnewvideo_nv12 == &video_nv12 ? VideoNv12() : std::move(pnewvideo_nv12);
  assertx(min(syx) >= 0.f);
  const Vec2<int> sdims = video_nv12.get_Y().dims().tail<2>();
  Vec2<int> newdims = convert<int>(convert<float>(sdims) * syx + .5f);
  if (1) newdims = (newdims + 1) / 2 * 2;  // make sizes be even integers
  if (!product(newdims)) {
    Warning("scaling to zero-sized frame");
    if (0) fill(newdims, 0);
  }
  newvideo_nv12.init(concat(V(video_nv12.nframes()), newdims));
  if (product(newdims)) {
    parallel_for_each(range(newvideo_nv12.nframes()),
                      [&](const int f) { scale(video_nv12[f], filterbs, bordervalue, newvideo_nv12[f]); });
  }
  return newvideo_nv12;
}

}  // namespace hh
