// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_VIDEO_H_
#define MESH_PROCESSING_LIBHH_VIDEO_H_

#include "Grid.h"
#include "Audio.h"
#include "Filter.h"             // FilterBnd
#include "Image.h"              // Nv12View
#include "FileIO.h"             // TmpFile

#if 0
{
  Video video(nframes, V(ysize, xsize)), video2(video.dims()), video3(video.nframes() * 2, video.spatial_dims());
  Pixel& pix = video(f, y, x);    // as in Image, [y=0][x=0] is at upper-left corner of each image frame
  uint8_t c = video(f, y, x)[z];  // z=[0..2]

  // Video read/write is performed using: Windows Media Foundation (MF) or ffmpeg (FF).
}
#endif

namespace hh {

// A Video is a 3D grid of RGB pixels, plus attributes like compression type, frame rate, and bit rate.
class Video : public Grid<3,Pixel> {
    using base = Grid<3,Pixel>;
    friend void swap(Video& l, Video& r) noexcept;
 public:
    struct Attrib;
    explicit Video(const Vec3<int>& dims = V(0, 0, 0)) { init(dims); } // nframes, ysize, xsize
    explicit Video(int pnframes, const Vec2<int>& sdims) : Video(V(pnframes, sdims[0], sdims[1])) { }
    explicit Video(const Video&)                = default;
    explicit Video(const base& video)           : base(video.dims()) { base::assign(video); }
    Video(Video&& v) noexcept                   { swap(*this, v); } // =default?
    Video(base&& v) noexcept                    { swap(implicit_cast<base&>(*this), v); }
    ~Video()                                    { }
    Video& operator=(Video&& v) noexcept        { clear(); swap(*this, v); return *this; }
    void operator=(base&& v)                    { clear(); swap(implicit_cast<base&>(*this), v); }
    Video& operator=(const Video&)              = default;
    void operator=(CGridView<3,Pixel> video)    { base::assign(video); }
    void init(const Vec3<int>& dims);
    void init(int pnframes, const Vec2<int>& sdims) { init(V(pnframes, sdims[0], sdims[1])); }
    void clear()                                { init(thrice(0)); }
    int nframes() const                         { return dim(0); }
    const Vec2<int>& spatial_dims() const       { return dims().tail<2>(); }
    int ysize() const                           { return dim(1); }
    int xsize() const                           { return dim(2); }
    const Attrib& attrib() const                { return _attrib; }
    Attrib& attrib()                            { return _attrib; }
    void read_file(const string& filename); // filename may be "-" for std::cin;  may throw std::runtime_error
    void write_file(const string& filename) const; // filename may be "-" for std::cout; may throw std::runtime_error

    // Misc:
    void scale(const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs, const Pixel* bordervalue = nullptr);
    struct Attrib {
        string suffix;          // e.g. "mp4"; "" if unknown; to identify format of read_file("-") and write_file("-")
        double framerate {0.};  // frames/sec
        int bitrate {0};        // bits/sec
        Audio audio;
    };
    static string diagnostic_string(const Vec3<int>& dims, const Attrib& attrib);
 private:
    Attrib _attrib;
};

// Whether filename suffix identifies it as a video.
bool filename_is_video(const string& filename);

// Return predicted video suffix given first byte of file, or "" if unrecognized.
string video_suffix_for_magic_byte(uchar c);

// Video consisting of an 8-bit luminance grid and a 2*8-bit chroma grid at half spatial resolution.
class VideoNv12 : noncopyable {
    friend void swap(VideoNv12& l, VideoNv12& r) noexcept;
 public:
    VideoNv12()                                 = default;
    explicit VideoNv12(const Vec3<int>& dims)   { init(dims); }
    VideoNv12(VideoNv12&& vnv12)                { swap(*this, vnv12); } // =default?
    VideoNv12& operator=(VideoNv12&& v) noexcept { clear(); swap(*this, v); return *this; }
    explicit VideoNv12(Grid<3,uint8_t>&& grid_Y, Grid<3, Vec2<uint8_t>>&& grid_UV)
        : _grid_Y(std::move(grid_Y)), _grid_UV(std::move(grid_UV)) { ok(); }
    void init(const Vec3<int>& dims) {
        _grid_Y.init(dims); _grid_UV.init(dims/V(1, 2, 2)); ok();
    }
    void clear()                                { init(thrice(0)); }
    int nframes() const                         { return _grid_Y.dim(0); }
    size_t size() const                         { return _grid_Y.size(); }
    CNv12View operator[](int f) const           { return CNv12View(_grid_Y[f], _grid_UV[f]); }
    Nv12View operator[](int f)                  { return Nv12View(_grid_Y[f], _grid_UV[f]); }
    GridView<3,uint8_t> get_Y()                 { return _grid_Y; }
    CGridView<3,uint8_t> get_Y() const          { return _grid_Y; }
    GridView<3, Vec2<uint8_t>> get_UV()         { return _grid_UV; }
    CGridView<3, Vec2<uint8_t>> get_UV() const  { return _grid_UV; }
    void read_file(const string& filename, Video::Attrib* pattrib = nullptr); // may throw std::runtime_error
    void write_file(const string& filename, const Video::Attrib& attrib) const; // may throw std::runtime_error
    void special_reduce_dim0(int i)             { _grid_Y.special_reduce_dim0(i), _grid_UV.special_reduce_dim0(i); }
 private:
    Grid<3,uint8_t> _grid_Y;             // luminance
    Grid<3, Vec2<uint8_t>> _grid_UV;     // chroma at half the spatial resolution
    void ok() const                             { assertx(_grid_Y.dims()==_grid_UV.dims()*V(1, 2, 2)); }
};

// View of an 8-bit luminance grid and a 2*8-bit chroma grid at half spatial resolution.
class VideoNv12View {
 public:
    explicit VideoNv12View(GridView<3,uint8_t> grid_Y, GridView<3, Vec2<uint8_t>> grid_UV)
        : _grid_Y(grid_Y), _grid_UV(grid_UV) { assertx(_grid_Y.dims()==_grid_UV.dims()*V(1, 2, 2)); }
    VideoNv12View(VideoNv12& vnv12) : _grid_Y(vnv12.get_Y()), _grid_UV(vnv12.get_UV()) { }
    int nframes() const                         { return _grid_Y.dim(0); }
    size_t size() const                         { return _grid_Y.size(); }
    Nv12View operator[](int f)                  { return Nv12View(_grid_Y[f], _grid_UV[f]); }
    GridView<3,uint8_t> get_Y()                 { return _grid_Y; }
    CGridView<3,uint8_t> get_Y() const          { return _grid_Y; }
    GridView<3, Vec2<uint8_t>> get_UV()         { return _grid_UV; }
    CGridView<3, Vec2<uint8_t>> get_UV() const  { return _grid_UV; }
 private:
    GridView<3,uint8_t> _grid_Y;             // luminance
    GridView<3, Vec2<uint8_t>> _grid_UV;     // chroma at half the spatial resolution
};

// Constant view of an 8-bit luminance grid and a 2*8-bit chroma grid at half spatial resolution.
class CVideoNv12View {
 public:
    explicit CVideoNv12View(CGridView<3,uint8_t> grid_Y, CGridView<3, Vec2<uint8_t>> grid_UV)
        : _grid_Y(grid_Y), _grid_UV(grid_UV) { assertx(_grid_Y.dims()==_grid_UV.dims()*V(1, 2, 2)); }
    CVideoNv12View(const VideoNv12& vnv12) : _grid_Y(vnv12.get_Y()), _grid_UV(vnv12.get_UV()) { }
    int nframes() const                         { return _grid_Y.dim(0); }
    size_t size() const                         { return _grid_Y.size(); }
    CNv12View operator[](int f) const           { return CNv12View(_grid_Y[f], _grid_UV[f]); }
    CGridView<3,uint8_t> get_Y() const          { return _grid_Y; }
    CGridView<3, Vec2<uint8_t>> get_UV() const  { return _grid_UV; }
 private:
    CGridView<3,uint8_t> _grid_Y;             // luminance
    CGridView<3, Vec2<uint8_t>> _grid_UV;     // chroma at half the spatial resolution
};

void convert_VideoNv12_to_Video(CVideoNv12View vnv12, GridView<3,Pixel> video);
void convert_Video_to_VideoNv12(CGridView<3,Pixel> video, VideoNv12View vnv12);

// &video==&newvideo is OK
Video scale(const Video& video, const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs,
            const Pixel* bordervalue = nullptr, Video&& newvideo = Video());
VideoNv12 scale(const VideoNv12& video_nv12, const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs,
                const Pixel* bordervalue = nullptr, VideoNv12&& pnewvideo_nv12 = VideoNv12());

// Read a video stream one image frame at a time.  getenv_string("VIDEO_IMPLEMENTATION") may set FF or MF.
class RVideo {
 public:
    explicit RVideo(const string& filename, bool use_nv12 = false); // may throw std::runtime_error
    ~RVideo();
    const Vec3<int>& dims() const               { return _dims; } // (nframes, ysize, xsize)
    const Video::Attrib& attrib() const         { return _attrib; }
    int nframes() const                         { return _dims[0]; }
    const Vec2<int>& spatial_dims() const       { return _dims.tail<2>(); } // (ysize, xsize)
    int ysize() const                           { return _dims[1]; }
    int xsize() const                           { return _dims[2]; }
    bool read(MatrixView<Pixel> frame); // frame(ysize(), xsize()); ret: false if EOF
    bool read(Nv12View frame);          // ret: false if EOF
    bool discard_frame();               // skip the next frame; ret: success (false if EOF)
    class Implementation;
 private:
    string _filename;
    bool _use_nv12;
    Vec3<int> _dims {0, 0, 0}; // nframes, ysize, xsize
    Video::Attrib _attrib;
    unique_ptr<TmpFile> _tmpfile;
    unique_ptr<Implementation> _impl;
    friend class MF_RVideo_Implementation;
    friend class FF_RVideo_Implementation;
};

// Write a video stream one image frame at a time.  getenv_string("VIDEO_IMPLEMENTATION") may set FF or MF.
class WVideo {
 public:
    explicit WVideo(const string& filename, const Vec2<int>& spatial_dims, const Video::Attrib& attrib,
                    bool use_nv12 = false); // dims are (y, x); may throw std::runtime_error
    ~WVideo();
    const Vec2<int>& spatial_dims() const       { return _sdims; }
    int ysize() const                           { return _sdims[0]; }
    int xsize() const                           { return _sdims[1]; }
    void write(CMatrixView<Pixel> frame);
    void write(CNv12View frame);
    class Implementation;
 private:
    string _filename;
    Vec2<int> _sdims;           // ysize, xsize
    Video::Attrib _attrib;
    bool _use_nv12;
    string _pfilename;          // original name if pipe
    unique_ptr<TmpFile> _tmpfile;
    unique_ptr<Implementation> _impl;
    friend class MF_WVideo_Implementation;
    friend class FF_WVideo_Implementation;
};

//----------------------------------------------------------------------------

inline void swap(Video& l, Video& r) noexcept {
    using std::swap; swap(implicit_cast<Video::base&>(l), implicit_cast<Video::base&>(r)); swap(l._attrib, r._attrib);
}

inline void swap(VideoNv12& l, VideoNv12& r) noexcept {
    using std::swap; swap(l._grid_Y, r._grid_Y); swap(l._grid_UV, r._grid_UV);
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_VIDEO_H_
