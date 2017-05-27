// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Video.h"

// I actually get better read/write performance by directly calling Media Foundation, at least in Windows 7.
// Generally, I prefer ffmpeg for its portability and quality.

#define HH_VIDEO_HAVE_MF        // unless disabled below
#define HH_VIDEO_HAVE_FFMPEG    // always as fallback

#if !defined(_MSC_VER) || defined(__clang__) || defined(HH_NO_WINDOWS_MEDIA_FOUNDATION)
#undef HH_VIDEO_HAVE_MF
#endif

//----------------------------------------------------------------------------

#if defined(HH_VIDEO_HAVE_MF)

#include <mutex>                // std::once_flag, std::call_once()
// #undef _WIN32_WINNT
// #define _WIN32_WINNT _WIN32_WINNT_WIN7
#define WIN32_LEAN_AND_MEAN
#include <windows.h>            // required by Media Foundation
#include <VersionHelpers.h>     // IsWindows8OrGreater()
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <codecapi.h>           // CODECAPI_AVEncMPVGOPSize, etc.
// #include <wrl/client.h>             // ComPtr<>
// template<typename T> using ComPtr = Microsoft::WRL::ComPtr<T>;
HH_REFERENCE_LIB("mfplat.lib");
HH_REFERENCE_LIB("mfreadwrite.lib"); // MFCreateSourceReaderFromByteStream()
HH_REFERENCE_LIB("ole32.lib");       // PropVariantClear()
HH_REFERENCE_LIB("mfuuid.lib");      // MF_MT_DEFAULT_STRIDE
#include "windows_com.h"             // com_ptr<>

#if defined(TRY_WIC)
#include <wincodec.h>           // WIC
#endif

#endif  // defined(HH_VIDEO_HAVE_MF)


//----------------------------------------------------------------------------

#if defined(HH_VIDEO_HAVE_FFMPEG)

#include "BinaryIO.h"

#endif  // defined(HH_VIDEO_HAVE_FFMPEG)


//----------------------------------------------------------------------------

#include "GridPixelOp.h"        // spatially_scale_Grid3_Pixel()
#include "FileIO.h"
#include "Vector4.h"
#include "Timer.h"
#include "ConsoleProgress.h"
#include "StringOp.h"           // get_path_extension()
#include <cstring>              // std::memcpy()


//----------------------------------------------------------------------------

// Notes on uncompressed/lossless video formats:
// http://superuser.com/questions/347433/how-to-create-an-uncompressed-avi-from-a-series-of-1000s-of-png-images-using-ff
// Consider 1280x720p 24fps Big Buck Bunny movie:
// - uncompressed (V308 is the 8 bpc variant of V410; not supported in many viewers)  518 Mbps
//   ffmpeg ...  -c:v v308 output.mov
// - compressed but lossless (Apple QuickTime Animation which is trivial RLE scheme)  165 Mbps
//   ffmpeg ...  -c:v qtrle -pix_fmt rgb24    output.mov
// - effectively lossless  (NV12 is 4:2:0)  133 Mbps
//   ffmpeg ...  -c:v ffvhuff -pix_fmt yuv420p output.avi
// - lossless H.264 (libx264-only)  (default pix_fmt is yuv444p or rgb24?) ("-preset ultrafast" for faster encoding)
//                    29 Mbps for yuv422p, 43 Mbps with ultrafast
//   ffmpeg ...  -c:v libx264 -qp 0 -pix_fmt yuv420p -f mp4 output.mp4

// I choose "-c:v ffvhuff" because it allows a unique container suffix ("avi") that is distinct from the others.
// I could use "-c:v ffvhuff -pix_fmt yuv420p" because it is compact and translates trivially to/from Nv12Video.
// However, it is lossy like NV12.
// Instead, I prefer "-c:v ffvhuff -pix_fmt yuv444p" which is lossless.

// An alternative would be to use lossless x264, still in an avi container?


namespace hh {

static void verify_attrib(Video::Attrib& attrib) {
    if (!attrib.framerate) {
        Warning("Video write: setting framerate to 30fps");
        attrib.framerate = 30.;
    }
}

void Video::init(const Vec3<int>& dims) {
    base::init(dims);
    if (0) {
        // required for Nv12/YUV support, but VideoViewer uses Video as a container for arbitrary images
        assertx(xsize()%2==0 && ysize()%2==0);
    }
}

string Video::diagnostic_string(const Vec3<int>& dims, const Attrib& attrib) {
    string s = sform("nframes~%d %dx%d (%gfps)", dims[0], dims[2], dims[1], attrib.framerate);
    if (attrib.suffix!="") s += " (" + attrib.suffix + ")";
    int brate = attrib.bitrate;
    if (brate)
        s += (brate>1000000 ? sform(" (%.2fMi bps)", brate/1000000.f) :
              brate>1000 ? sform(" (%.2fKi bps)", brate/1000.f) :
              sform(" (%d bps)", brate));
    return s;
}

void Video::scale(const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs, const Pixel* bordervalue) {
    *this = hh::scale(*this, syx, filterbs, bordervalue, std::move(*this));
}

void Video::read_file(const string& filename) {
    // Similar code in: Video::read_file(), VideoNv12::read_file(), and FilterVideo.cpp::read_video().
    HH_TIMER(_read_video);
    clear();
    RVideo rvideo(filename);
    attrib() = rvideo.attrib();
    const int nfexpect = rvideo.nframes(); assertw(nfexpect>0);
    const int padframes = 2;    // because may read a different number of frames
    init(nfexpect+padframes, rvideo.spatial_dims());
    {
        ConsoleProgress cprogress("Vread");
        int f = 0;
        for (;;) {
            if (nfexpect) cprogress.update(float(f)/(nfexpect));
            if (f>=nframes()) break;
            if (!rvideo.read((*this)[f])) break;
            f++;
        }
        base::special_reduce_dim0(f);
    }
    const int nf = nframes();
    if (abs(nf-nfexpect)>1) { SHOW(nf, nfexpect); Warning("Video: read unexpected num frames"); }
}

void Video::write_file(const string& filename) const {
    HH_TIMER(_write_video);
    assertx(size());
    verify_attrib(const_cast<Video&>(*this).attrib()); // mutable
    string suffix = to_lower(get_path_extension(filename));
    if (suffix!="") const_cast<Video&>(*this).attrib().suffix = suffix; // mutable
    const bool use_nv12 = false;
    WVideo wvideo(filename, spatial_dims(), attrib(), use_nv12);
    ConsoleProgress cprogress("Vwrite");
    for_int(f, nframes()) {
        cprogress.update(float(f)/nframes());
        wvideo.write((*this)[f]);
    }
}


//----------------------------------------------------------------------------

bool filename_is_video(const string& filename) {
    static const Array<string> k_extensions = {
        "mpg", "mpeg", "mpe", "mpv", "mp2", "mpeg2", "mp4", "mpeg4", "mov", "avi", "wmv", "flv",
        "m2v", "m4v", "webm", "ogv", "3gp", "mts"
    };
    return k_extensions.index(to_lower(get_path_extension(filename)))>=0;
}

string video_suffix_for_magic_byte(uchar c) {
    // see also image_suffix_for_magic_byte() and audio_suffix_for_magic_byte()
    // Documentation on prefixes for various video containers:
    //  *.mp4: "\000\000\000\030ftypmp42", "\000\000\000 ftypisom", "\000\000\000\034ftypisom"
    //  *.wmv: "0&\262u"
    //  *.avi: "RIFF"
    //  *.mov: "\000\000\000\030ftypqt   \a\t\000"
    switch (c) {
     bcase 0:       return "mp4"; // u'\x00'; or "mov"
     bcase '0':     return "wmv";
     bcase 'R':     return "avi";
     bdefault:      return "";
    }
}


//----------------------------------------------------------------------------

void VideoNv12::read_file(const string& filename, Video::Attrib* pattrib) {
    // Similar code in: Video::read_file(), VideoNv12::read_file(), and FilterVideo.cpp::read_video().
    HH_TIMER(_read_video);
    clear();
    const bool use_nv12 = true;
    RVideo rvideo(filename, use_nv12);
    if (pattrib) *pattrib = rvideo.attrib();
    const int nfexpect = rvideo.nframes(); assertw(nfexpect>0);
    const int padframes = 2;    // because may read a different number of frames
    init(concat(V(nfexpect+padframes), rvideo.spatial_dims()));
    {
        ConsoleProgress cprogress("Vread");
        int f = 0;
        for (;;) {
            if (nfexpect) cprogress.update(float(f)/(nfexpect));
            if (f>=_grid_Y.dim(0)) break;
            if (!rvideo.read((*this)[f])) break;
            f++;
        }
        special_reduce_dim0(f);
    }
    const int nf = _grid_Y.dim(0);
    if (abs(nf-nfexpect)>1) { SHOW(nf, nfexpect); Warning("VideoNv12: read unexpected num frames"); }
}

void VideoNv12::write_file(const string& filename, const Video::Attrib& pattrib) const {
    HH_TIMER(_write_video);
    assertx(size());
    Video::Attrib attrib = pattrib;
    verify_attrib(attrib);
    string suffix = to_lower(get_path_extension(filename));
    if (suffix!="") attrib.suffix = suffix;
    const bool use_nv12 = true;
    WVideo wvideo(filename, _grid_Y.dims().tail<2>(), attrib, use_nv12);
    ConsoleProgress cprogress("Vwrite");
    const int nf = _grid_Y.dim(0);
    for_int(f, nf) {
        cprogress.update(float(f)/nf);
        wvideo.write((*this)[f]);
    }
}


//----------------------------------------------------------------------------

void convert_VideoNv12_to_Video(CVideoNv12View vnv12, GridView<3,Pixel> video) {
    assertx(vnv12.nframes()==video.dim(0));
    parallel_for_int(f, video.dim(0)) { convert_Nv12_to_Image(vnv12[f], video[f]); }
}

void convert_Video_to_VideoNv12(CGridView<3,Pixel> video, VideoNv12View vnv12) {
    assertx(vnv12.nframes()==video.dim(0));
    parallel_for_int(f, video.dim(0)) { convert_Image_to_Nv12(video[f], vnv12[f]); }
}


//----------------------------------------------------------------------------

Video scale(const Video& video, const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs,
            const Pixel* bordervalue, Video&& pnewvideo) {
    // With Grid<3,Pixel> rather than Array<Matrix<Pixel>>, we cannot directly overwrite video in-place.
    Video newvideo = &pnewvideo==&video ? Video() : std::move(pnewvideo);
    assertx(min(syx)>=0.f);
    Vec2<int> newdims = convert<int>(convert<float>(video.spatial_dims())*syx+.5f);
    if (video.attrib().suffix!="avi") newdims = (newdims+1)/2*2; // make sizes be even integers
    if (!product(newdims)) {
        Warning("scaling to zero-sized frame");
        if (0) fill(newdims, 0);
    }
    newvideo.init(video.nframes(), newdims); newvideo.attrib() = video.attrib();
    if (!product(newdims)) return newvideo;
    spatially_scale_Grid3_Pixel(video, filterbs, bordervalue, newvideo);
    return newvideo;
}

VideoNv12 scale(const VideoNv12& video_nv12, const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs,
                const Pixel* bordervalue, VideoNv12&& pnewvideo_nv12) {
    VideoNv12 newvideo_nv12 = &pnewvideo_nv12==&video_nv12 ? VideoNv12() : std::move(pnewvideo_nv12);
    assertx(min(syx)>=0.f);
    const Vec2<int> sdims = video_nv12.get_Y().dims().tail<2>();
    Vec2<int> newdims = convert<int>(convert<float>(sdims)*syx+.5f);
    if (1) newdims = (newdims+1)/2*2; // make sizes be even integers
    if (!product(newdims)) {
        Warning("scaling to zero-sized frame");
        if (0) fill(newdims, 0);
    }
    newvideo_nv12.init(concat(V(video_nv12.nframes()), newdims));
    if (!product(newdims)) return newvideo_nv12;
    float borderY;
    Vector4 borderUV;
    if (bordervalue) {
        uchar borderYt = RGB_to_Y(*bordervalue);
        convert(CGrid1View(borderYt), Grid1View(borderY));
        Vec2<uchar> borderUVt = V(RGB_to_U(*bordervalue), RGB_to_V(*bordervalue));
        convert(CGrid1View(borderUVt), Grid1View(borderUV));
    }
    parallel_for_int(f, newvideo_nv12.nframes()) {
        Matrix<float> mat(sdims); convert(video_nv12.get_Y()[f], mat);
        Matrix<float> mat2 = scale(mat, newdims, filterbs, bordervalue ? &borderY : nullptr);
        convert(mat2, newvideo_nv12.get_Y()[f]);
    }
    parallel_for_int(f, newvideo_nv12.nframes()) {
        Matrix<Vector4> mat(sdims/2); convert(video_nv12.get_UV()[f], mat);
        Matrix<Vector4> mat2 = scale(mat, newdims/2, filterbs, bordervalue ? &borderUV : nullptr);
        convert(mat2, newvideo_nv12.get_UV()[f]);
    }
    return newvideo_nv12;
}


//----------------------------------------------------------------------------

class RVideo::Implementation {
 public:
    Implementation(RVideo& rvideo)              : _rvideo(rvideo) { }
    virtual ~Implementation()                   { }
    virtual string name() const = 0;
    virtual bool read(MatrixView<Pixel> frame) = 0;
    virtual bool read_nv12(Nv12View frame) { // default slow path
        const Vec2<int> sdims = _rvideo.spatial_dims();
        assertx(frame.get_Y().dims()==sdims);
        Matrix<Pixel> tframe(frame.get_Y().dims());
        if (!read(tframe)) return false;
        convert_Image_to_Nv12(tframe, frame);
        return true;
    }
    virtual bool discard_frame() { // default slow path
        const Vec2<int> sdims = _rvideo.spatial_dims();
        if (_rvideo._use_nv12) {
            Nv12 nv12(sdims);
            return read_nv12(nv12);
        } else {
            Matrix<Pixel> tframe(sdims);
            return read(tframe);
        }
    }
    static unique_ptr<Implementation> make(RVideo& rvideo);
 protected:
    RVideo& _rvideo;
};

class WVideo::Implementation {
 public:
    Implementation(WVideo& wvideo)              : _wvideo(wvideo) { }
    virtual ~Implementation()                   { }
    virtual string name() const = 0;
    virtual void write(CMatrixView<Pixel> frame) = 0;
    virtual void write_nv12(CNv12View frame) { // default slow path
        assertx(product(_wvideo.spatial_dims())); assertx(frame.get_Y().dims()==_wvideo.spatial_dims());
        Matrix<Pixel> tframe(frame.get_Y().dims());
        convert_Nv12_to_Image(frame, tframe);
        write(tframe);
    }
    static unique_ptr<Implementation> make(WVideo& wvideo);
 protected:
    WVideo& _wvideo;
};

class Unsupported_RVideo_Implementation : public RVideo::Implementation {
 public:
    Unsupported_RVideo_Implementation(RVideo& rvideo)   : RVideo::Implementation(rvideo) { assertnever_ret("?"); }
    ~Unsupported_RVideo_Implementation() { }
    virtual string name() const override                { return "unsupported"; }
    bool read(MatrixView<Pixel> frame) override         { dummy_use(frame); assertnever("?"); }
};

class Unsupported_WVideo_Implementation : public WVideo::Implementation {
 public:
    Unsupported_WVideo_Implementation(WVideo& wvideo)   : WVideo::Implementation(wvideo) { assertnever_ret("?"); }
    ~Unsupported_WVideo_Implementation() { }
    virtual string name() const override                { return "unsupported"; }
    void write(CMatrixView<Pixel> frame) override       { dummy_use(frame); assertnever("?"); }
};


//----------------------------------------------------------------------------

RVideo::RVideo(const string& filename, bool use_nv12) : _filename(filename), _use_nv12(use_nv12) {
    if (file_requires_pipe(_filename)) {
        RFile fi(_filename);
        int c = fi().peek();
        if (c<0) throw std::runtime_error("Error reading video from empty pipe '" + _filename + "'");
        _attrib.suffix = video_suffix_for_magic_byte(uchar(c));
        if (_attrib.suffix=="")
            throw std::runtime_error(sform("Peeked video format (c=%d) in pipe '%s' is not recognized",
                                           c, _filename.c_str()));
        // if (_attrib.suffix=="avi") _use_nv12 = false; // must be done in caller
        _tmpfile = make_unique<TmpFile>(_attrib.suffix);
        _filename = _tmpfile->filename();
        WFile fi2(_filename);
        fi2() << fi().rdbuf();  // copy the entire input stream to the temporary file
    }
    if (!file_exists(_filename)) throw std::runtime_error("Video file '" + _filename + "' does not exist");
    _attrib.suffix = to_lower(get_path_extension(_filename));
    _impl = Implementation::make(*this);
    if (getenv_bool("VIDEO_DEBUG")) SHOW(_impl->name());
}

RVideo::~RVideo() {
    _impl.reset();
}

bool RVideo::read(MatrixView<Pixel> frame)      { assertw(!_use_nv12); return _impl->read(frame); }
bool RVideo::read(Nv12View frame)               { assertw(_use_nv12); return _impl->read_nv12(frame); }
bool RVideo::discard_frame()                    { return _impl->discard_frame(); }

WVideo::WVideo(const string& filename, const Vec2<int>& spatial_dims, const Video::Attrib& attrib, bool use_nv12)
    : _filename(filename), _sdims(spatial_dims), _attrib(attrib), _use_nv12(use_nv12), _pfilename(filename) {
    if (_attrib.suffix=="") _attrib.suffix = to_lower(get_path_extension(_filename));
    if (_attrib.suffix=="")
        throw std::runtime_error("Video '" + filename + "': no filename suffix specified for writing");
    if (file_requires_pipe(_filename)) {
        if (_filename=="-") my_setenv("NO_DIAGNOSTICS_IN_STDOUT", "1");
        _tmpfile = make_unique<TmpFile>(_attrib.suffix);
        _filename = _tmpfile->filename();
    }
    if (!_attrib.bitrate) { Warning("Setting a high video bitrate"); _attrib.bitrate = 40*1000000; }
    _impl = Implementation::make(*this);
    if (getenv_bool("VIDEO_DEBUG")) SHOW(_impl->name());
}

WVideo::~WVideo() {
    _impl.reset();
    if (_tmpfile) {
        WFile fi(_pfilename);
        RFile fi2(_filename);
        fi() << fi2().rdbuf(); // copy the entire stream
    }
}

void WVideo::write(CMatrixView<Pixel> frame)    { assertw(!_use_nv12); _impl->write(frame); }
void WVideo::write(CNv12View frame)             { assertw(_use_nv12); _impl->write_nv12(frame); }


//----------------------------------------------------------------------------
// *** Video I/O

#define AS(expr) assertx(SUCCEEDED(expr))

//----------------------------------------------------------------------------
// **** using Media Foundation
#if defined(HH_VIDEO_HAVE_MF)

// Supported Media Formats in Media Foundation
// http://msdn.microsoft.com/en-us/library/dd757927%28VS.85%29.aspx
// Containers: 3gp, asf, wma, wmv, aac, adts, avi, mp3, m4a, v4v, mov, mp4, sami, smi, wav

class Initialize_COM_MF {
 public:
    Initialize_COM_MF() {
        if (s_num_video_uses++ && 0) return;
        // default may be COINIT_MULTITHREADED, but VT code assumes COINIT_APARTMENTTHREADED
        if (0) {
            // The following could work if we could guarantee that VT initialization occurred before this call.
            HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
            assertx(SUCCEEDED(hr) || hr==RPC_E_CHANGED_MODE);
        } else {
            HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
            assertx(SUCCEEDED(hr) || hr==S_FALSE); // may equal S_FALSE if COM was previously initialized
        }
        AS(MFStartup(MF_VERSION, MFSTARTUP_FULL)); // initialize Media Foundation
    }
    ~Initialize_COM_MF() {
        if (--s_num_video_uses) return;
        if (1) return;              // there may still be other users, such as Vision Tools.
        AS(MFShutdown());
        CoUninitialize();
    }
 private:
    static std::atomic<int> s_num_video_uses; // only release MediaFoundation and COM when this reaches zero
};

std::atomic<int> Initialize_COM_MF::s_num_video_uses;

class LockIMFMediaBuffer {
 public:
    LockIMFMediaBuffer(IMFMediaBuffer* buffer) : _buffer(buffer) {
        AS(_buffer->Lock(&_pData, nullptr, nullptr));
    }
    ~LockIMFMediaBuffer()                       { assertx(_pData); AS(_buffer->Unlock()); }
    uchar* operator()() const                   { return _pData; }
 private:
    IMFMediaBuffer* _buffer;
    uchar* _pData {nullptr};
};

static void retrieve_strided_BGRA(const uchar* pData, int stride, MatrixView<Pixel> frame) {
    const int ny = frame.ysize(), nx = frame.xsize();
    uchar* pd = frame.data()->data();
    for_int(y, ny) {
        const uchar* ps = pData+y*stride;
        for_int(x, nx) {
            pd[0] = ps[2]; pd[1] = ps[1]; pd[2] = ps[0]; pd[3] = 255; // BGRA to RGBA
            pd += 4; ps += 4;
        }
    }
}

static void retrieve_strided_Nv12(const uchar* pData, int stride, int offsetUV, Nv12View nv12v) {
    const int ny = nv12v.get_Y().ysize(), nx = nv12v.get_Y().xsize();
    // This is a bottleneck, so worth optimizing.
    if (stride==nv12v.get_Y().xsize()) {
        std::memcpy(nv12v.get_Y().data(), pData, nv12v.get_Y().size());
        std::memcpy(nv12v.get_UV().data(), pData+offsetUV, nv12v.get_UV().size()*2);
    } else if (1) {
        for_int(y, ny) {
            std::memcpy(nv12v.get_Y()[y].data(), pData+y*stride, nx);
        }
        for_int(y, ny/2) {
            std::memcpy(nv12v.get_UV()[y].data(), pData+offsetUV+y*stride, nx);
        }
    } else {
        uchar* pd = nv12v.get_Y().data();
        for_int(y, ny) {
            const uchar *ps = pData+y*stride;
            for_int(x, nx) { *pd++ = *ps++; }
        }
        pd = nv12v.get_UV().data()->data();
        for_int(y, ny/2) {
            const uchar *ps = pData+offsetUV+y*stride;
            for_int(x, nx) { *pd++ = *ps++; }
        }
    }
}

class MF_RVideo_Implementation : public RVideo::Implementation {
 public:
    MF_RVideo_Implementation(RVideo& rvideo) : RVideo::Implementation(rvideo) {
        {
            com_ptr<IMFSourceResolver> pSourceResolver; AS(MFCreateSourceResolver(&pSourceResolver));
            DWORD createObjFlags = MF_RESOLUTION_BYTESTREAM |
                MF_RESOLUTION_CONTENT_DOES_NOT_HAVE_TO_MATCH_EXTENSION_OR_MIME_TYPE;
            MF_OBJECT_TYPE objectType = MF_OBJECT_INVALID;
            com_ptr<IUnknown> pSource;
            if (FAILED(pSourceResolver->CreateObjectFromURL(widen(_rvideo._filename).c_str(), createObjFlags,
                                                            nullptr, &objectType, &pSource)))
                throw std::runtime_error("Could not open video in file '" + _rvideo._filename + "'");
            assertx(objectType==MF_OBJECT_BYTESTREAM);
            AS(pSource->QueryInterface(IID_PPV_ARGS(&_pByteStream)));
        }
        {
            com_ptr<IMFAttributes> pAttributes; AS(MFCreateAttributes(&pAttributes, 2));
            AS(pAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE));
            AS(pAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE));
            if (FAILED(MFCreateSourceReaderFromByteStream(_pByteStream, pAttributes, &_pReader)))
                throw std::runtime_error("Could not open video in file '" + _rvideo._filename + "'");
        }
        double duration; {
            PROPVARIANT var; PropVariantInit(&var);
            AS(_pReader->GetPresentationAttribute(DWORD(MF_SOURCE_READER_MEDIASOURCE), MF_PD_DURATION, &var));
            assertx(var.vt==VT_UI8);
            ULONGLONG durationInHundredsOfNanoseconds = var.uhVal.QuadPart; double TIME_HNS_TO_S_FACTOR = 10000000.;
            duration = double(durationInHundredsOfNanoseconds)/TIME_HNS_TO_S_FACTOR;
            PropVariantClear(&var);
        }
        {
            _impl_nv12 = _rvideo._use_nv12;
            if (1 || !IsWindows8OrGreater()) _impl_nv12 = true; // It may be ~3.5x faster on Windows 7
            // Actually it is still 5x faster on Windows 10
        }
        {
            com_ptr<IMFMediaType> pType; AS(MFCreateMediaType(&pType));
            AS(pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
            AS(pType->SetGUID(MF_MT_SUBTYPE, _impl_nv12 ? MFVideoFormat_NV12 : MFVideoFormat_RGB32));
            if (FAILED(_pReader->SetCurrentMediaType(DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, pType))) {
                assertx(_impl_nv12);
                Warning("Read video using MFVideoFormat_NV12 failed; reverting to MFVideoFormat_RGB32");
                _impl_nv12 = false;
                AS(pType->SetGUID(MF_MT_SUBTYPE, _impl_nv12 ? MFVideoFormat_NV12 : MFVideoFormat_RGB32));
                if (FAILED(_pReader->SetCurrentMediaType(DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), nullptr, pType)))
                    throw std::runtime_error("Could not open video media type in file '" + _rvideo._filename + "'");
            }
            AS(_pReader->SetStreamSelection(DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), TRUE));
        }
        {
            com_ptr<IMFMediaType> pType;
            AS(_pReader->GetCurrentMediaType(DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), &pType));
            GUID subtype; AS(pType->GetGUID(MF_MT_SUBTYPE, &subtype));
            assertx(subtype==(_impl_nv12 ? MFVideoFormat_NV12 : MFVideoFormat_RGB32));
            UINT32 vnum = 0; UINT32 vdenom = 1; AS(MFGetAttributeRatio(pType, MF_MT_FRAME_RATE, &vnum, &vdenom));
            _rvideo._attrib.framerate = double(vnum)/double(vdenom);
            if (0) SHOW(_rvideo._attrib.framerate, duration);
            Vec3<int>& dims = _rvideo._dims;
            dims[0] = int(_rvideo._attrib.framerate*duration+.5); // set nframes
            UINT32 tx = 0; UINT32 ty = 0; AS(MFGetAttributeSize(pType, MF_MT_FRAME_SIZE, &tx, &ty));
            dims[1] = int(ty); dims[2] = int(tx); // set ysize, xsize
            _mf_ny = dims[1]; _mf_nx = dims[2];   // initial settings
            UINT32 bitrate = 0; if (!SUCCEEDED(pType->GetUINT32(MF_MT_AVG_BITRATE, &bitrate))) bitrate = 0;
            _rvideo._attrib.bitrate = int(bitrate);
            UINT32 tstride = 0; AS(pType->GetUINT32(MF_MT_DEFAULT_STRIDE, &tstride));
            _stride = tstride; assertx(_stride>0);
            if (0) SHOW(dims, _stride);
        }
    }
    ~MF_RVideo_Implementation() {
        // Note that _init_com_mf.~Initialize_COM_MF() is called after this destructor
    }
    virtual string name() const { return "MF"; }
    bool read(MatrixView<Pixel> frame) override {
        const Vec2<int> sdims = _rvideo.spatial_dims();
        assertx(frame.dims()==sdims);
        com_ptr<IMFSample> pSample = get_sample();
        if (!pSample) return false;
        const int stride = _stride;
        com_ptr<IMFMediaBuffer> pBuffer; AS(pSample->ConvertToContiguousBuffer(&pBuffer));
        {
            LockIMFMediaBuffer lock(pBuffer);
            const uchar* pData = lock();
            // The following fails on the mp4 I tried!
            // com_ptr<IMF2DBuffer> p2Dbuf; AS(pBuffer->QueryInterface(IID_PPV_ARGS(&p2Dbuf)));
            // BYTE* pData; LONG lstride; AS(p2Dbuf->Lock2D(&pData, &lstride)); {
            if (!_impl_nv12) {
                retrieve_strided_BGRA(pData, stride, frame);
            } else {
                int offset = _mf_ny*stride;
                if (stride==sdims[1]*1) {
                    CMatrixView<uchar> matY(pData, sdims);
                    CMatrixView<Vec2<uchar>> matUV(reinterpret_cast<const Vec2<uchar>*>(pData+offset), sdims/2);
                    convert_Nv12_to_Image(CNv12View(matY, matUV), frame);
                } else {
                    Nv12 nv12(sdims); retrieve_strided_Nv12(pData, stride, offset, nv12); // slow path
                    convert_Nv12_to_Image(nv12, frame);
                }
            }
            // } AS(p2Dbuf->Unlock2D());
        }
        return true;
    }
    bool read_nv12(Nv12View nv12v) override {
        const Vec2<int> sdims = _rvideo.spatial_dims();
        assertx(nv12v.get_Y().dims()==sdims);
        com_ptr<IMFSample> pSample = get_sample();
        if (!pSample) return false;
        const int stride = _stride;
        com_ptr<IMFMediaBuffer> pBuffer; AS(pSample->ConvertToContiguousBuffer(&pBuffer));
        {
            LockIMFMediaBuffer lock(pBuffer);
            const uchar* pData = lock();
            if (_impl_nv12) {
                retrieve_strided_Nv12(pData, stride, _mf_ny*stride, nv12v);
            } else {
                if (stride==sdims[1]*4) {
                    MatrixView<Pixel> mat(reinterpret_cast<Pixel*>(const_cast<uchar*>(pData)), sdims);
                    for (Pixel& pix : mat) std::swap(pix[0], pix[2]); // BGRA to RGBA
                    convert_Image_to_Nv12(mat, nv12v);
                } else {
                    Matrix<Pixel> mat(sdims); retrieve_strided_BGRA(pData, stride, mat); // slower path
                    convert_Image_to_Nv12(mat, nv12v);
                }
            }
        }
        return true;
    }
    bool discard_frame() override {
        com_ptr<IMFSample> pSample = get_sample();
        return !!pSample;
    }
    com_ptr<IMFSample> get_sample() {
        com_ptr<IMFSample> pSample;
        for (;;) {
            DWORD dwFlags = 0;
            assertx(!pSample);
            AS(_pReader->ReadSample(DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM),
                                    0, nullptr, &dwFlags, nullptr, &pSample));
            if (dwFlags & MF_SOURCE_READERF_ENDOFSTREAM) { assertx(!pSample); return {}; }
            // It is possible for real media to be 960x540 and then get "changed" to 960x544.
            // Both tx and ty may change:  Filtervideo ~/proj/videoloops/data/ReallyFreakinAll/SDBaiModelS.mp4 -noo
            //   xsize=300 ysize=540  -->  _mf_ny=544 _mf_nx=304 _stride=304
            if ((dwFlags & MF_SOURCE_READERF_CURRENTMEDIATYPECHANGED)) {
                com_ptr<IMFMediaType> pType;
                AS(_pReader->GetCurrentMediaType(DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), &pType));
                UINT32 tx = 0; UINT32 ty = 0; AS(MFGetAttributeSize(pType, MF_MT_FRAME_SIZE, &tx, &ty));
                _mf_ny = ty; _mf_nx = tx; assertx(_mf_ny>=_rvideo.ysize() && _mf_nx>=_rvideo.xsize());
                UINT32 tstride = 0; AS(pType->GetUINT32(MF_MT_DEFAULT_STRIDE, &tstride));
                _stride = tstride; assertx(_stride>0);
                // SHOW(_mf_ny, _mf_nx, _stride);
            }
            assertw(pSample);
            if (pSample) break;
        }
        return pSample;
    }
    static bool supported() { return true; }
 private:
    Initialize_COM_MF _init_com_mf; // must be declared first, to be destroyed after remaining members
    com_ptr<IMFByteStream> _pByteStream;
    com_ptr<IMFSourceReader> _pReader;
    bool _impl_nv12;
    int _mf_ny;
    int _mf_nx;
    int _stride;
};

class MF_WVideo_Implementation : public WVideo::Implementation {
 public:
    MF_WVideo_Implementation(WVideo& wvideo) : WVideo::Implementation(wvideo) {
        const auto& attrib = _wvideo._attrib;
        if (attrib.audio.size()) Warning("MF_WVideo does not currently support audio");
        _impl_nv12 = attrib.suffix=="mp4";
        if (0 && _impl_nv12) { Warning("Disabling _impl_nv12"); _impl_nv12 = false; }
        int iFramesPerSecond = int(attrib.framerate+.5);
        AS(MFFrameRateToAverageTimePerFrame(iFramesPerSecond, 1, &_rtDuration));
        GUID videoOutputFormat;
        if (0) void();
        else if (attrib.suffix=="mp4") videoOutputFormat = MFVideoFormat_H264;
        else if (attrib.suffix=="mov") videoOutputFormat = MFVideoFormat_H264;
        else if (attrib.suffix=="wmv") videoOutputFormat = MFVideoFormat_WVC1;
        // Note that Media Foundation does not support *.avi as an output format;
        //  see: https://msdn.microsoft.com/en-us/library/dd757927(VS.85).aspx
        else if (0 && attrib.suffix=="avi") videoOutputFormat = MFVideoFormat_420O; // 8bpc planar YUV 4:2:0
        else throw std::runtime_error("Video: encoder suffix '" + attrib.suffix + "' not recognized");
        // See Tutorial: Using the Sink Writer to Encode Video (Windows)
        //  http://msdn.microsoft.com/en-us/library/windows/desktop/ff819477%28v=vs.85%29.aspx
        // Color conversion bug:
        // http://social.msdn.microsoft.com/Forums/en-US/mediafoundationdevelopment/thread/8a03ac4d-34a3-493d-b27e-5147de575fb7
        // Use Color Converter DSP (http://msdn.microsoft.com/en-us/library/ff819079%28VS.85%29.aspx)
        // See "Work around for XVP color conversion problem" in
        //   ~/git/CompPhoto/ClassLibs/VisionTools/src/fileio/MFVideoSrc.h
        {
            com_ptr<IMFAttributes> pAttributes; AS(MFCreateAttributes(&pAttributes, 10));
            AS(pAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE));
            // AS(pAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, FALSE)); // no effect
            // AS(pAttributes->SetUINT32(MF_READWRITE_DISABLE_CONVERTERS, TRUE)); // causes it to fail
            AS(pAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE)); // no effect
            // AS(pAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, FALSE)); // no effect
            IMFByteStream* const k_ByteStream = nullptr;
            if (FAILED(MFCreateSinkWriterFromURL(widen(_wvideo._filename).c_str(), k_ByteStream,
                                                 nullptr/*pAttributes*/, &_pSinkWriter)))
                throw std::runtime_error("Could not write video to file '" + _wvideo._filename + "'");
        }
        {
            com_ptr<IMFMediaType> pMediaTypeOut; AS(MFCreateMediaType(&pMediaTypeOut));
            AS(pMediaTypeOut->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
            AS(pMediaTypeOut->SetGUID(MF_MT_SUBTYPE, videoOutputFormat));
            AS(pMediaTypeOut->SetUINT32(MF_MT_AVG_BITRATE, UINT32(attrib.bitrate)));
            AS(pMediaTypeOut->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
            AS(MFSetAttributeSize(pMediaTypeOut, MF_MT_FRAME_SIZE, _wvideo.xsize(), _wvideo.ysize()));
            AS(MFSetAttributeRatio(pMediaTypeOut, MF_MT_FRAME_RATE, iFramesPerSecond, 1));
            AS(MFSetAttributeRatio(pMediaTypeOut, MF_MT_PIXEL_ASPECT_RATIO, 1, 1));
            AS(_pSinkWriter->AddStream(pMediaTypeOut, &_streamIndex));
        }
        {
            com_ptr<IMFMediaType> pMediaTypeIn; AS(MFCreateMediaType(&pMediaTypeIn));
            AS(pMediaTypeIn->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
            GUID video_format = MFVideoFormat_RGB32; if (_impl_nv12) video_format = MFVideoFormat_NV12;
            AS(pMediaTypeIn->SetGUID(MF_MT_SUBTYPE, video_format));
            AS(pMediaTypeIn->SetUINT32(MF_MT_DEFAULT_STRIDE, _wvideo.xsize()*(_impl_nv12 ? 1 : 4)));
            AS(pMediaTypeIn->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
            AS(MFSetAttributeSize(pMediaTypeIn, MF_MT_FRAME_SIZE, _wvideo.xsize(), _wvideo.ysize()));
            AS(MFSetAttributeRatio(pMediaTypeIn, MF_MT_FRAME_RATE, iFramesPerSecond, 1));
            AS(MFSetAttributeRatio(pMediaTypeIn, MF_MT_PIXEL_ASPECT_RATIO, 1, 1));
            com_ptr<IMFAttributes> pAttributes; // pEncodingParameters
            // IMFAttributes* const pEncodingParameters = nullptr;
            if (0) {            // untested
                AS(MFCreateAttributes(&pAttributes, 10));
                if (0) {
                    unsigned force_keyframe_every_nframes = 20;
                    AS(pAttributes->SetUINT32(CODECAPI_AVEncMPVGOPSize, force_keyframe_every_nframes));
                }
                if (0) {
                    unsigned quality = 78;
                    AS(pAttributes->SetUINT32(CODECAPI_AVEncCommonRateControlMode,
                                              eAVEncCommonRateControlMode_Quality));
                    AS(pAttributes->SetUINT32(CODECAPI_AVEncCommonQuality, quality));
                }
            }
            AS(_pSinkWriter->SetInputMediaType(_streamIndex, pMediaTypeIn, pAttributes));
        }
        AS(_pSinkWriter->BeginWriting());
    }
    ~MF_WVideo_Implementation() {
        if (_pSinkWriter) AS(_pSinkWriter->Finalize());
        // Note that _init_com_mf.~Initialize_COM_MF() is called after this destructor
    }
    virtual string name() const { return "MF"; }
    void write(CMatrixView<Pixel> frame) override {
        const Vec2<int> sdims = _wvideo.spatial_dims();
        assertx(product(_wvideo.spatial_dims())); assertx(frame.dims()==_wvideo.spatial_dims());
#if defined(TRY_WIC)            // untested (not sure if this can work)
        assertnever("untested");
        static IWICImagingFactory* wic_factory;
        static std::once_flag flag;
        std::call_once(flag, []() {
            AS(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&wic_factory)));
        });
        assertx(wic_factory);
        com_ptr<IWICBitmap> bitmap; {
            WICPixelFormatGUID bitmap_pixel_format = GUID_WICPixelFormat32bppRGBA; // ignore undefined alpha
            unsigned stride = sdims[1]*sizeof(Pixel);
            // Note: problem for huge images; no workaround using WIC.
            unsigned buffer_size = assert_narrow_cast<unsigned>(sdims[0]*sdims[1]*sizeof(Pixel));
            uchar* buf = const_cast<uchar*>(frame.data()->data());
            AS(wic_factory->CreateBitmapFromMemory(sdims[1], sdims[0], bitmap_pixel_format,
                                                   stride, buffer_size, buf, &bitmap));
        }
        // requires Windows 8
        com_ptr<IMFMediaBuffer> pBuffer; AS(MFCreateWICBitmapBuffer(__uuidof(IWICBitmap), bitmap, &pBuffer));
#else
        DWORD cbBuffer = DWORD(product(_wvideo.spatial_dims())*(_impl_nv12 ? 1.5f : 4.f));
        com_ptr<IMFMediaBuffer> pBuffer; AS(MFCreateAlignedMemoryBuffer(cbBuffer, MF_16_BYTE_ALIGNMENT, &pBuffer));
        {
            LockIMFMediaBuffer lock(pBuffer);
            uchar* pData = lock();
            if (!_impl_nv12) {
                uchar* pd = pData;
                const uchar* ps = frame.data()->data();
                for_int(y, sdims[0]) {
                    for_int(x, sdims[1]) {
                        pd[0] = ps[2]; pd[1] = ps[1]; pd[2] = ps[0]; pd[3] = 0; // RGBA to BGRA
                        pd += 4; ps += 4;
                    }
                }
            } else {
                // Media Foundation MP4 encoding under Win7 may have poor quality -- independent of this workaround.
                MatrixView<uchar> matY(pData, _wvideo.spatial_dims());
                MatrixView<Vec2<uchar>> matUV(reinterpret_cast<Vec2<uchar>*>(pData+sdims[0]*sdims[1]),
                                              _wvideo.spatial_dims()/2);
                convert_Image_to_Nv12(frame, Nv12View(matY, matUV));
            }
        }
        AS(pBuffer->SetCurrentLength(cbBuffer));
#endif
        com_ptr<IMFSample> pSample; AS(MFCreateSample(&pSample));
        AS(pSample->AddBuffer(pBuffer));
        AS(pSample->SetSampleTime(_rtStart));
        AS(pSample->SetSampleDuration(_rtDuration));
        AS(_pSinkWriter->WriteSample(_streamIndex, pSample));
        _rtStart += _rtDuration;
    }
    void write_nv12(CNv12View nv12v) override {
        const Vec2<int> sdims = _wvideo.spatial_dims();
        assertx(product(_wvideo.spatial_dims())); assertx(nv12v.get_Y().dims()==_wvideo.spatial_dims());
        DWORD cbBuffer = DWORD(product(_wvideo.spatial_dims())*(_impl_nv12 ? 1.5f : 4.f));
        com_ptr<IMFMediaBuffer> pBuffer; AS(MFCreateAlignedMemoryBuffer(cbBuffer, MF_16_BYTE_ALIGNMENT, &pBuffer));
        {
            LockIMFMediaBuffer lock(pBuffer);
            uchar* pData = lock();
            if (_impl_nv12) {
                // Media Foundation MP4 encoding under Win7 may have poor quality; it is independent of this workaround
                MatrixView<uchar> matY(pData, _wvideo.spatial_dims());
                MatrixView<Vec2<uchar>> matUV(reinterpret_cast<Vec2<uchar>*>(pData+sdims[0]*sdims[1]),
                                              _wvideo.spatial_dims()/2);
                matY.assign(nv12v.get_Y());
                matUV.assign(nv12v.get_UV());
            } else {
                MatrixView<Pixel> mat(reinterpret_cast<Pixel*>(pData), _wvideo.spatial_dims());
                convert_Nv12_to_Image(nv12v, mat);
                for (Pixel& pix : mat) std::swap(pix[0], pix[2]); // RGBA to BGRA
            }
        }
        AS(pBuffer->SetCurrentLength(cbBuffer));
        com_ptr<IMFSample> pSample; AS(MFCreateSample(&pSample));
        AS(pSample->AddBuffer(pBuffer));
        AS(pSample->SetSampleTime(_rtStart));
        AS(pSample->SetSampleDuration(_rtDuration));
        AS(_pSinkWriter->WriteSample(_streamIndex, pSample));
        _rtStart += _rtDuration;
    }
    static bool supported() { return true; }
 private:
    Initialize_COM_MF _init_com_mf; // must be declared first, to be destroyed after remaining members
    com_ptr<IMFSinkWriter> _pSinkWriter;
    DWORD _streamIndex;
    UINT64 _rtDuration;
    LONGLONG _rtStart {0};
    bool _impl_nv12;            // otherwise, Windows 7 Media Foundation shifts colors during MP4 compression!
    // for *.wmv files, _impl_nv12 does not speed up compression -- it is always about 3x slower than MP4 in Win7.
};

#else

class MF_RVideo_Implementation : public Unsupported_RVideo_Implementation {
 public:
    MF_RVideo_Implementation(RVideo& rvideo) : Unsupported_RVideo_Implementation(rvideo) { }
    static bool supported() { return false; }
};
class MF_WVideo_Implementation : public Unsupported_WVideo_Implementation {
 public:
    MF_WVideo_Implementation(WVideo& wvideo) : Unsupported_WVideo_Implementation(wvideo) { }
    static bool supported() { return false; }
};

#endif  // defined(HH_VIDEO_HAVE_MF)


//----------------------------------------------------------------------------
// **** using ffmpeg
#if defined(HH_VIDEO_HAVE_FFMPEG)

class FF_RVideo_Implementation : public RVideo::Implementation {
 public:
    FF_RVideo_Implementation(RVideo& rvideo) : RVideo::Implementation(rvideo) {
        assertx(supported());
        // See: ~/src/_other/Repository.cpp
        // See: ffmpeg -hide_banner -pix_fmts
        const bool ldebug = getenv_bool("FF_DEBUG");
        const string& filename = _rvideo._filename;
        bool expect_audio = false;
        {                       // read header for dimensions and attributes (ignore video and audio data)
            // 2>&1 works on both Unix bash shell and Windows cmd shell: http://stackoverflow.com/questions/1420965/
            // Ideally, <nul and/or </dev/null so that "vv ~/proj/fastloops/data/assembled_all_loops_uhd.mp4" does
            //  not stop responding.
            // Instead, use -nostdin .  Yes, it works.  Moreover, now do dup2(1, 0) in VideoViewer.
            //
            // TODO: read and preserve metadata in video:  yes, likely also more robust way to read attributes below.
            // http://stackoverflow.com/a/9473239/1190077:
            //  http://www.ffmpeg.org/ffmpeg-formats.html#Metadata-1
            // ffmpeg -i input_video -f ffmetadata metadata.txt
            // ffmpeg -i input_video -i ffmetadata.txt -map_metadata 1 -codec copy output_video
            // (BTW, ffmpeg supports multiple outputs: https://trac.ffmpeg.org/wiki/Creating%20multiple%20outputs
            //   ffmpeg -i input1 -i input2 -acodec ... -vcodec ... output1 -acodec ... -vcodec ... output2 )
            // http://superuser.com/questions/349518/
            // ffmpeg -i video.mp4 -vn -acodec copy -metadata title="My Title" audio.m4a
            // ffmpeg -i ~/data/video/HDbrink8h.mp4 -f ffmetadata v.txt && cat v.txt
            // ;FFMETADATA1
            // major_brand=mp42
            // minor_version=0
            // compatible_brands=mp41isom
            // encoder=Lavf56.4.101
            // Conclusion: not useful because already given in regular stdout of ffmpeg
            // I could look at ffprobe to see if it can output more information, or use exiftool.
            // Omitting "-hide_banner" because unrecognized by older version of ffmpeg.
            // Option "-nostdin" is also unrecognized by older versions, but it can continue nonetheless.
            // Input #0, gif, from 'image4.gif':
            //   Duration: N/A, bitrate: N/A
            //     Stream #0:0: Video: gif, bgra, 960x720, 1 fps, 1 tbr, 100 tbn, 100 tbc
            //  (unfortunately, no way to know that there are 8 frames -- would have to read until EOF)
            RFile fi("ffmpeg -nostdin -i " + quote_arg_for_shell(filename) + " -vn -an 2>&1 |");
            Vec3<int> dims{0, 0, 0};
            double duration = -1., total_bitrate = -1.;
            double video_bitrate = -1., framerate = -1.;
            bool yuv444p = false;
            int nlines = 0;
            string sline;
            while (my_getline(fi(), sline, false)) {
                nlines++;
                if (ldebug) SHOW(sline);
                char vch;
                if (contains(sline, "Could not find option 'nostdin'")) {
                    Warning("Version of external program 'ffmpeg' may be too old");
                    continue;
                }
                if (contains(sline, "Duration:")) {
                    //  Duration: 00:00:05.00, start: 0.000000, bitrate: 32842 kb/s (some mp4 files)
                    //  Duration: 00:00:05.03, bitrate: 100505 kb/s
                    //  Duration: N/A, bitrate: N/A  (invalid.mp4)
                    if (contains(sline, "Duration: N/A"))
                        throw std::runtime_error("Invalid video in file '" + filename + "'");
                    {
                        int vh, vm, vs, vcs;
                        assertx(sscanf(sline.c_str(), " Duration: %d:%d:%d.%d%c",
                                       &vh, &vm, &vs, &vcs, &vch)==5 && vch==',');
                        duration = vh*3600. + vm*60. + vs + vcs*.01;
                        if (ldebug) SHOW(vh, vm, vs, vcs, duration);
                    }
                    {
                        auto i = sline.find(", bitrate:"); assertx(i!=string::npos);
                        assertx(sscanf(sline.c_str()+i, ", bitrate: %lg kb/%c", &total_bitrate, &vch)==2 && vch=='s');
                        total_bitrate *= 1000.;
                        if (ldebug) SHOW(total_bitrate);
                    }
                }
                if (contains(sline, "Stream #0:") && contains(sline, ": Video:")) {
                    // Stream #0:0(eng): Video: mpeg4 (Simple Profile) (mp4v / 0x7634706D), yuv420p, 960x540 [SAR 1:1 DAR 16:9], 32840 kb/s, 30 fps, 30 tbr, 30 tbn, 30 tbc (default)
                    // Stream #0:0: Video: msmpeg4v3 (MP43 / 0x3334504D), yuv420p, 960x540, 30 fps, 30 tbr, 1k tbn, 1k tbc
                    // Stream #0:0: Video: h264 (High) (H264 / 0x34363248), yuv420p, 960x540, 68071 kb/s, 30 fps, 30 tbr, 30 tbn, 60 tbc
                    // Stream #0:0(eng): Video: vc1 (Advanced) (WVC1 / 0x31435657), yuv420p, 3840x2160, 100000 kb/s, SAR 1:1 DAR 16:9, 30 fps, 30 tbr, 1k tbn, 1k tbc
                    // Stream #0:0(und): Video: h264 (Baseline) (avc1 / 0x31637661), yuv420p(tv, bt709), 1280x720, 10734 kb/s, 29.97 fps, 29.97 tbr, 600 tbn, 1200 tbc (default)
                    // Stream #0:0(eng): Video: h264 (High) (avc1 / 0x31637661), yuv420p(tv, bt709), 3840x2160 [SAR 1:1 DAR 16:9], 59969 kb/s, 29.97 fps, 29.97 tbr, 30k tbn, 59.94 tbc (default)
                    // Stream #0:0(eng): Video: vc1 (Advanced) (WVC1 / 0x31435657), yuv420p, 3840x2160 [SAR 1:1 DAR 16:9], 100000 kb/s, 30 tbr, 1k tbn, 60 tbc
                    // Stream #0:1(eng): Video: wmv3 (Main) (WMV3 / 0x33564D57), yuv420p, 640x480, 768 kb/s, SAR 1:1 DAR 4:3, 30 fps, 30 tbr, 1k tbn, 1k tbc
                    // Stream #0:0: Video: rawvideo, bgr24, 768x1024, 2 fps, 2 tbr, 2 tbn, 2 tbc
                    // Stream #0:0: Video: ffvhuff (FFVH / 0x48564646), yuv444p, 960x540, 155182 kb/s, 30 fps, 30 tbr, 30 tbn, 30 tbc  [*.avi]

                    for (string::size_type i = 0; ; ) {
                        i = sline.find(',', i+1); assertx(i!=string::npos);
                        if (sscanf(sline.c_str()+i, ", %dx%d", &dims[2], &dims[1])==2) break;
                    }
                    string::size_type i = sline.find(" kb/s");
                    if (i!=string::npos) {
                        if (video_bitrate>=0.) assertnever("Multiple video streams inside media container");
                        i = sline.rfind(", ", i);
                        if (i!=string::npos) {
                            assertx(sscanf(sline.c_str()+i, ", %lg kb/%c", &video_bitrate, &vch)==2 && vch=='s');
                            video_bitrate *= 1000.;
                        }
                    }
                    i = sline.find(" fps");
                    if (i!=string::npos) {
                        i = sline.rfind(", ", i); assertx(i!=string::npos);
                        assertx(sscanf(sline.c_str()+i, ", %lg fp%c", &framerate, &vch)==2 && vch=='s');
                    } else {
                        i = sline.find(" tbr"); assertx(i!=string::npos);
                        i = sline.rfind(", ", i); assertx(i!=string::npos);
                        assertx(sscanf(sline.c_str()+i, ", %lg tb%c", &framerate, &vch)==2 && vch=='r');
                    }
                    if (sline.find("yuv444p")!=string::npos) yuv444p = true;
                    if (ldebug) SHOW(dims[2], dims[1], video_bitrate, framerate, yuv444p);
                }
                if (contains(sline, "Stream #0:") && contains(sline, ": Audio:") && contains(sline, "kb/s"))
                    expect_audio = true;
            }
            if (!nlines) throw std::runtime_error("ffmpeg is unable to read video file '" + filename + "'");
            if (video_bitrate<0.) video_bitrate = total_bitrate;
            if (ldebug) SHOW(duration, total_bitrate, video_bitrate, framerate);
            if (!(duration>0. && total_bitrate>0. && video_bitrate>0. && framerate>0.))
                throw std::runtime_error("ffmpeg is unable to parse video in file '" + filename + "'");
            dims[0] = int(duration*framerate+.5);
            assertx(product(dims)>0);
            _rvideo._dims = dims;
            _rvideo._attrib.bitrate = int(video_bitrate);
            _rvideo._attrib.framerate = framerate;
            if (yuv444p && _rvideo._use_nv12) {
                // must be fixed in caller
                if (0) Warning("Switching video read away from NV12 because video encoding is yuv444p");
                if (0) _rvideo._use_nv12 = false;
                Warning("Reading NV12 from a Video encoded with yuv444p");
            }
        }
        if (expect_audio) {
            try {
                Audio& audio = _rvideo._attrib.audio;
                audio.read_file(filename);
                if (ldebug) SHOW(audio.diagnostic_string());
            }
            catch (std::runtime_error& ex) {
                SHOW("Failed to read audio within video", filename, ex.what());
            }
        }
        string pixfmt = _rvideo._use_nv12 ? "nv12" : "rgba"; // was rgb24
        // Note: -hide_banner unnecessary with "-loglevel panic" if using "-i some_input".
        string scmd = ("ffmpeg -loglevel panic -nostdin -i " + quote_arg_for_shell(filename) +
                       " -f image2pipe -pix_fmt " + pixfmt + " -vcodec rawvideo - |");
        if (ldebug) SHOW(scmd);
        _pfi = make_unique<RFile>(scmd);
    }
    ~FF_RVideo_Implementation() {
    }
    virtual string name() const override { return "FF"; }
    bool read(MatrixView<Pixel> frame) override {
        const Vec2<int> sdims = _rvideo.spatial_dims();
        assertx(frame.dims()==sdims);
        if (!_rvideo._use_nv12) {
            if (!read_binary_raw((*_pfi)(), frame.array_view())) return false;
            if (0) { for_int(i, int(product(sdims))) { frame.raster(i)[3] = 255; } }
        } else {
            _ar_tmp.init(assert_narrow_cast<int>(product(sdims)+product(sdims)/2)); // NV12
            if (!read_binary_raw((*_pfi)(), _ar_tmp)) return false;
            CMatrixView<uchar> matY(_ar_tmp.data(), sdims);
            CMatrixView<Vec2<uchar>> matUV(reinterpret_cast<const Vec2<uchar>*>(_ar_tmp.data()+product(sdims)),
                                           sdims/2);
            convert_Nv12_to_Image(CNv12View(matY, matUV), frame);
        }
        return true;
    }
    bool read_nv12(Nv12View nv12v) override {
        const Vec2<int> sdims = _rvideo.spatial_dims();
        assertx(nv12v.get_Y().dims()==sdims);
        if (_rvideo._use_nv12) {
            if (!read_binary_raw((*_pfi)(), nv12v.get_Y().array_view())) return false;
            assertx(read_binary_raw((*_pfi)(), nv12v.get_UV().array_view()));
        } else {
            _frame_rgb_tmp.init(sdims);
            if (!read(_frame_rgb_tmp)) return false;
            convert_Image_to_Nv12(_frame_rgb_tmp, nv12v);
        }
        return true;
    }
    bool discard_frame() override {
        const Vec2<int> sdims = _rvideo.spatial_dims();
        if (_rvideo._use_nv12) {
            (*_pfi)().ignore(product(sdims)+product(sdims)/2);
        } else {
            (*_pfi)().ignore(product(sdims)*4);
        }
        return !!(*_pfi)();
    }
    static bool supported() { return ffmpeg_command_exists(); }
 private:
    unique_ptr<RFile> _pfi;
    Array<uchar> _ar_tmp;
    Matrix<Pixel> _frame_rgb_tmp;
};

class FF_WVideo_Implementation : public WVideo::Implementation {
 public:
    FF_WVideo_Implementation(WVideo& wvideo) : WVideo::Implementation(wvideo) {
        const bool ldebug = getenv_bool("FF_DEBUG");
        assertx(supported());
        const string& filename = _wvideo._filename;
        Video::Attrib& attrib = _wvideo._attrib;
        const Vec2<int> sdims = _wvideo.spatial_dims();
        string sfilecontainer = ""; // default is to infer container format based on filename extension
        // ffmpeg -hide_banner -formats
        {
            assertx(filename!="-"); // WVideo::WVideo() should have created TmpFile if writing to stdout.
            // (Note that ffmpeg -f mp4 is not supported over pipe because it requires a seekable output;
            //  without -f, it actually uses avi as a default container, which doesn't match the mp4 file suffix.)
            // At some point I managed to get Win7 Windows Media Player to play some type of file by creating an avi
            //  container with a file suffix of mp4.  Not good for the long term.
            // if (attrib.suffix=="mp4") sfilecontainer = " -f avi";
            // Setting container should be unnecessary because it is derived automatically based on file suffix.
        }
        string ipixfmt = _wvideo._use_nv12 ? "nv12" : "rgba"; // was rgb24
        string str_audio = " -an"; // default no audio
        string ocodec = "";
        string opixfmt = "yuv420p";
        // ffmpeg -hide_banner -codecs | grep '^..E'
        // ffmpeg -hide_banner -encoders
        // hevc : H.265 (libx265)
        // webp : WebP (libwebp)
        // (e=mp4; ffmpeg -hide_banner -i ~/data/video/fewpalms.mp4 -y v.$e && ffmpeg -hide_banner -i v.$e -an -vn)
        // Default encoders chosen by ffmpeg based on file suffix:
        //  mp4: h264 (High) (avc1 / 0x31637661), yuv420p
        //  mov: h264 (High) (avc1 / 0x31637661), yuv420p
        //  avi: mpeg4 (Simple Profile) (FMP4 / 0x34504D46), yuv420p
        //  wmv: msmpeg4v3 (MP43 / 0x3334504D), yuv420p // (wmv2 inferior; wmv3 & vc1 encoders missing)
        // Note: Windows Media Player in Win7 cannot play back msmpeg4v3 4K video in real time!
        if (attrib.suffix=="wmv") Warning("FF_WVideo_Implementation uses msmpeg4v3 (not vc1) for *.wmv");
        if (0 && attrib.suffix=="mp4") ocodec = " -vcodec h264";
        if (0 && attrib.suffix=="mov") ocodec = " -vcodec h264";
        if (attrib.suffix=="avi") {
            ocodec = " -c:v ffvhuff";  // "effectively lossless"
            opixfmt = "yuv444p";       // lossless (not NV12 like "yuv420p")
            if (_wvideo._use_nv12) Warning("Using NV12 to write to a *.avi Video with yuv444p encoding");
        }
        Audio& audio = attrib.audio;
        if (audio.size()) {
            if (ldebug) SHOW("previously", audio.attrib().suffix);
            audio.attrib().suffix = "aac"; // or "mp3"
            // Create a temporary file containing the audio encoded as aac.
            _tmpfile_audio = make_unique<TmpFile>(audio.attrib().suffix);
            try {
                audio.write_file(_tmpfile_audio->filename());
                str_audio = " -i " + _tmpfile_audio->filename() + sform(" -ab %d", audio.attrib().bitrate);
            }
            catch (std::runtime_error& ex) {
                SHOW("Failed to write audio within video", ex.what());
            }
        }
        // Note: "-hide_banner" unnecessary with "-loglevel panic" and "-i some_input".
        string scmd = ("| ffmpeg -loglevel panic -f rawvideo -vcodec rawvideo -pix_fmt " + ipixfmt +
                       sform(" -s %dx%d -r %g", sdims[1], sdims[0], attrib.framerate) +
                       " -i -" + str_audio + sfilecontainer + ocodec + " -pix_fmt " + opixfmt +
                       sform(" -vb %d", attrib.bitrate) + " -y " + quote_arg_for_shell(filename));
        if (ldebug) SHOW(scmd);
        _pfi = make_unique<WFile>(scmd);
    }
    ~FF_WVideo_Implementation() {
    }
    virtual string name() const override { return "FF"; }
    void write(CMatrixView<Pixel> frame) override {
        const Vec2<int> sdims = _wvideo.spatial_dims();
        assertx(product(sdims)); assertx(frame.dims()==sdims);
        if (!_wvideo._use_nv12) {
            if (!write_binary_raw((*_pfi)(), frame.array_view()))
                throw std::runtime_error("FF_WVideo write frame failed");
        } else {
            _frame_nv12_tmp.init(sdims);
            convert_Image_to_Nv12(frame, _frame_nv12_tmp);
            return write_nv12(_frame_nv12_tmp);
        }
    }
    void write_nv12(CNv12View nv12v) override {
        const Vec2<int> sdims = _wvideo.spatial_dims();
        assertx(product(sdims)); assertx(nv12v.get_Y().dims()==sdims);
        if (_wvideo._use_nv12) {
            assertx(write_binary_raw((*_pfi)(), nv12v.get_Y().array_view()));
            assertx(write_binary_raw((*_pfi)(), nv12v.get_UV().array_view()));
        } else {
            _frame_rgb_tmp.init(sdims);
            convert_Nv12_to_Image(nv12v, _frame_rgb_tmp);
            write(_frame_rgb_tmp);
        }
    }
    static bool supported() { return ffmpeg_command_exists(); }
 private:
    unique_ptr<TmpFile> _tmpfile_audio; // lifespan should be longer than _pfi
    unique_ptr<WFile> _pfi;
    Nv12 _frame_nv12_tmp;
    Matrix<Pixel> _frame_rgb_tmp;
};

#endif  // defined(HH_VIDEO_HAVE_FFMPEG)


//----------------------------------------------------------------------------

unique_ptr<RVideo::Implementation> RVideo::Implementation::make(RVideo& rvideo) {
    // Notes:
    // - Win7 Media Foundation: For some mp4 files, e.g. ~/data/video/fewpalms.mp4, bitrate is read as 0.
    //   e.g.:  RVIDEO_IMPLEMENTATION=MF Filtervideo ~/data/video/fewpalms.mp4 -stat
    string implementation = getenv_string("RVIDEO_IMPLEMENTATION");
    if (implementation=="") implementation = getenv_string("VIDEO_IMPLEMENTATION");
    if (implementation=="MF") return make_unique<MF_RVideo_Implementation>(rvideo);
    if (implementation=="FF") return make_unique<FF_RVideo_Implementation>(rvideo);
    if (implementation!="") throw std::runtime_error("RVideo implementation '" + implementation + "' not recognized");
    if (FF_RVideo_Implementation::supported()) return make_unique<FF_RVideo_Implementation>(rvideo);
    if (MF_RVideo_Implementation::supported()) return make_unique<MF_RVideo_Implementation>(rvideo);
    throw std::runtime_error("Video I/O not implemented");
}

unique_ptr<WVideo::Implementation> WVideo::Implementation::make(WVideo& wvideo) {
    // Notes:
    // - Win7 Media Foundation: I can set large bitrate on mp4, but do not see increase in file size.
    //   e.g.: VIDEO_IMPLEMENTATION=MF Filtervideo ~/data/video/fewpalms.mp4 -info -bitrate 32.84m -info -to mp4 | Filtervideo -stat
    // - ffmpeg: cannot write *.wmv using VC1 codec (instead resorts to msmpeg4v3).
    string implementation = getenv_string("WVIDEO_IMPLEMENTATION");
    if (implementation=="") implementation = getenv_string("VIDEO_IMPLEMENTATION");
    if (implementation=="MF") return make_unique<MF_WVideo_Implementation>(wvideo);
    if (implementation=="FF") return make_unique<FF_WVideo_Implementation>(wvideo);
    if (implementation!="") throw std::runtime_error("WVideo implementation '" + implementation + "' not recognized");
    // Prefer ffmpeg, but revert to Windows Media Foundation (if available) for *.wmv output (using VC1 encoder).
    if (wvideo._attrib.suffix=="wmv" && MF_WVideo_Implementation::supported())
        return make_unique<MF_WVideo_Implementation>(wvideo);
    if (FF_WVideo_Implementation::supported()) return make_unique<FF_WVideo_Implementation>(wvideo);
    if (MF_WVideo_Implementation::supported()) return make_unique<MF_WVideo_Implementation>(wvideo);
    throw std::runtime_error("Video I/O not implemented");
}

} // namespace hh
