// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Video.h"

// We get better read/write performance by directly calling Media Foundation, at least in Windows 7.
// But, use of ffmpeg is convenient for its portability and quality.

#define HH_VIDEO_HAVE_MF      // unless disabled below
#define HH_VIDEO_HAVE_FFMPEG  // always as fallback

#if !defined(_MSC_VER) || defined(HH_NO_WINDOWS_MEDIA_FOUNDATION)
#undef HH_VIDEO_HAVE_MF
#endif

//----------------------------------------------------------------------------

#if defined(HH_VIDEO_HAVE_MF)

// #undef _WIN32_WINNT
// #define _WIN32_WINNT _WIN32_WINNT_WIN7
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>  // required by Media Foundation; must appear before other headers.

#pragma warning(disable : 5204 5246)  // for <mfapi.h>

#include <VersionHelpers.h>  // IsWindows8OrGreater()
#include <codecapi.h>        // CODECAPI_AVEncMPVGOPSize, etc.
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
// #include <wrl/client.h>             // ComPtr<>
// template<typename T> using ComPtr = Microsoft::WRL::ComPtr<T>;
HH_REFERENCE_LIB("mfplat.lib");
HH_REFERENCE_LIB("mfreadwrite.lib");  // MFCreateSourceReaderFromByteStream()
HH_REFERENCE_LIB("mfuuid.lib");       // MF_MT_DEFAULT_STRIDE
HH_REFERENCE_LIB("ole32.lib");        // PropVariantClear()

#include <mutex>  // once_flag, call_once()

#include "libHh/windows_com.h"  // com_ptr<>

#endif  // defined(HH_VIDEO_HAVE_MF)

//----------------------------------------------------------------------------

#if defined(HH_VIDEO_HAVE_FFMPEG)
#include "libHh/BinaryIO.h"
#endif

//----------------------------------------------------------------------------

#include <atomic>
#include <cstring>  // memcpy()

#include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"
#include "libHh/StringOp.h"  // get_path_extension(), to_lower(), starts_with(), ends_with(), contains()
#include "libHh/Timer.h"
#include "libHh/Vector4.h"

//----------------------------------------------------------------------------

// Notes on uncompressed/lossless video formats:
// https://superuser.com/questions/347433/how-to-create-an-uncompressed-avi-from-a-series-of-1000s-of-png-images-using-ff
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

// We choose "-c:v ffvhuff" because it allows a unique container suffix ("avi") that is distinct from the others.
// We could use "-c:v ffvhuff -pix_fmt yuv420p" because it is compact and translates trivially to/from Nv12Video.
// However, it is lossy like NV12.
// Instead, we prefer "-c:v ffvhuff -pix_fmt yuv444p" which is lossless.

// An alternative would be to use lossless x264, still in an avi container?

namespace hh {

namespace {

void verify_attrib(Video::Attrib& attrib) {
  if (!attrib.framerate) {
    Warning("Video write: setting framerate to 30fps");
    attrib.framerate = 30.;
  }
}

}  // namespace

void Video::read_file(const string& filename) {
  // Similar code in: Video::read_file(), VideoNv12::read_file(), and FilterVideo.cpp::read_video().
  HH_TIMER("_read_video");
  clear();
  RVideo rvideo(filename);
  attrib() = rvideo.attrib();
  const int nfexpect = rvideo.nframes();
  assertw(nfexpect > 0);
  const int padframes = 2;  // because may read a different number of frames
  init(nfexpect + padframes, rvideo.spatial_dims());
  {
    ConsoleProgress cprogress("Vread");
    int f = 0;
    for (;;) {
      if (nfexpect) cprogress.update(float(f) / (nfexpect));
      if (f >= nframes()) break;
      if (!rvideo.read((*this)[f])) break;
      f++;
    }
    base::special_reduce_dim0(f);
  }
  const int nf = nframes();
  if (abs(nf - nfexpect) > 1) {
    SHOW(nf, nfexpect);
    Warning("Video: read unexpected num frames");
  }
}

void Video::write_file(const string& filename) const {
  HH_TIMER("_write_video");
  assertx(size());
  verify_attrib(const_cast<Video&>(*this).attrib());  // mutable
  string suffix = to_lower(get_path_extension(filename));
  if (suffix != "") const_cast<Video&>(*this).attrib().suffix = suffix;  // mutable
  const bool use_nv12 = false;
  WVideo wvideo(filename, spatial_dims(), attrib(), use_nv12);
  ConsoleProgress cprogress("Vwrite");
  for_int(f, nframes()) {
    cprogress.update(float(f) / nframes());
    wvideo.write((*this)[f]);
  }
}

//----------------------------------------------------------------------------

void VideoNv12::read_file(const string& filename, Video::Attrib* pattrib) {
  // Similar code in: Video::read_file(), VideoNv12::read_file(), and FilterVideo.cpp::read_video().
  HH_TIMER("_read_video");
  clear();
  const bool use_nv12 = true;
  RVideo rvideo(filename, use_nv12);
  if (pattrib) *pattrib = rvideo.attrib();
  const int nfexpect = rvideo.nframes();
  assertw(nfexpect > 0);
  const int padframes = 2;  // because may read a different number of frames
  init(concat(V(nfexpect + padframes), rvideo.spatial_dims()));
  {
    ConsoleProgress cprogress("Vread");
    int f = 0;
    for (;;) {
      if (nfexpect) cprogress.update(float(f) / (nfexpect));
      if (f >= _grid_Y.dim(0)) break;
      if (!rvideo.read((*this)[f])) break;
      f++;
    }
    special_reduce_dim0(f);
  }
  const int nf = _grid_Y.dim(0);
  if (abs(nf - nfexpect) > 1) {
    SHOW(nf, nfexpect);
    Warning("VideoNv12: read unexpected num frames");
  }
}

void VideoNv12::write_file(const string& filename, const Video::Attrib& pattrib) const {
  HH_TIMER("_write_video");
  assertx(size());
  Video::Attrib attrib = pattrib;
  verify_attrib(attrib);
  string suffix = to_lower(get_path_extension(filename));
  if (suffix != "") attrib.suffix = suffix;
  const bool use_nv12 = true;
  WVideo wvideo(filename, _grid_Y.dims().tail<2>(), attrib, use_nv12);
  ConsoleProgress cprogress("Vwrite");
  const int nf = _grid_Y.dim(0);
  for_int(f, nf) {
    cprogress.update(float(f) / nf);
    wvideo.write((*this)[f]);
  }
}

//----------------------------------------------------------------------------

class RVideo::Implementation {
 public:
  explicit Implementation(RVideo& rvideo) : _rvideo(rvideo) {}
  virtual ~Implementation() = default;
  virtual string name() const = 0;
  virtual bool read(MatrixView<Pixel> frame) = 0;
  virtual bool read_nv12(Nv12View frame) {  // default slow path
    const Vec2<int> sdims = _rvideo.spatial_dims();
    assertx(frame.get_Y().dims() == sdims);
    Matrix<Pixel> tframe(frame.get_Y().dims());
    if (!read(tframe)) return false;
    convert_Image_to_Nv12(tframe, frame);
    return true;
  }
  virtual bool discard_frame() {  // default slow path
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
  explicit Implementation(WVideo& wvideo) : _wvideo(wvideo) {}
  virtual ~Implementation() = default;
  virtual string name() const = 0;
  virtual void write(CMatrixView<Pixel> frame) = 0;
  virtual void write_nv12(CNv12View frame) {  // default slow path
    assertx(product(_wvideo.spatial_dims()));
    assertx(frame.get_Y().dims() == _wvideo.spatial_dims());
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
  explicit Unsupported_RVideo_Implementation(RVideo& rvideo) : RVideo::Implementation(rvideo) { assertnever_ret("?"); }
  ~Unsupported_RVideo_Implementation() override = default;
  string name() const override { return "unsupported"; }
  bool read(MatrixView<Pixel> frame) override {
    dummy_use(frame);
    assertnever("?");
  }
};

class Unsupported_WVideo_Implementation : public WVideo::Implementation {
 public:
  explicit Unsupported_WVideo_Implementation(WVideo& wvideo) : WVideo::Implementation(wvideo) { assertnever_ret("?"); }
  ~Unsupported_WVideo_Implementation() override = default;
  string name() const override { return "unsupported"; }
  void write(CMatrixView<Pixel> frame) override {
    dummy_use(frame);
    assertnever("?");
  }
};

//----------------------------------------------------------------------------

RVideo::RVideo(string filename, bool use_nv12) : _filename(std::move(filename)), _use_nv12(use_nv12) {
  if (file_requires_pipe(_filename)) {
    RFile fi(_filename);
    int c = fi().peek();
    if (c < 0) throw std::runtime_error("Error reading video from empty pipe '" + _filename + "'");
    _attrib.suffix = video_suffix_for_magic_byte(uchar(c));
    if (_attrib.suffix == "")
      throw std::runtime_error(
          sform("Peeked video format (c=%d) in pipe '%s' is not recognized", c, _filename.c_str()));
    // if (_attrib.suffix == "avi") _use_nv12 = false;  // must be done in caller
    _tmpfile = make_unique<TmpFile>(_attrib.suffix, fi());
    _filename = _tmpfile->filename();
  }
  if (!file_exists(_filename)) throw std::runtime_error("Video file '" + _filename + "' does not exist");
  _attrib.suffix = to_lower(get_path_extension(_filename));
  _impl = Implementation::make(*this);
  if (getenv_bool("VIDEO_DEBUG")) SHOW(_impl->name());
}

RVideo::~RVideo() { _impl = nullptr; }

bool RVideo::read(MatrixView<Pixel> frame) {
  assertw(!_use_nv12);
  return _impl->read(frame);
}

bool RVideo::read(Nv12View frame) {
  assertw(_use_nv12);
  return _impl->read_nv12(frame);
}

bool RVideo::discard_frame() { return _impl->discard_frame(); }

WVideo::WVideo(string filename, const Vec2<int>& spatial_dims, Video::Attrib attrib, bool use_nv12)
    : _filename(std::move(filename)),
      _sdims(spatial_dims),
      _attrib(std::move(attrib)),
      _use_nv12(use_nv12),
      _pfilename(_filename) {
  if (_attrib.suffix == "") _attrib.suffix = to_lower(get_path_extension(_filename));
  if (_attrib.suffix == "")
    throw std::runtime_error("Video '" + _filename + "': no filename suffix specified for writing");
  if (file_requires_pipe(_filename)) {
    if (_filename == "-") my_setenv("NO_DIAGNOSTICS_IN_STDOUT", "1");
    _tmpfile = make_unique<TmpFile>(_attrib.suffix);
    _filename = _tmpfile->filename();
  }
  if (!_attrib.bitrate) {
    Warning("Setting a high video bitrate");
    _attrib.bitrate = 40'000'000;
  }
  _impl = Implementation::make(*this);
  if (getenv_bool("VIDEO_DEBUG")) SHOW(_impl->name());
}

WVideo::~WVideo() {
  _impl = nullptr;
  if (_tmpfile) _tmpfile->write_to(WFile{_pfilename}());
}

void WVideo::write(CMatrixView<Pixel> frame) {
  assertw(!_use_nv12);
  _impl->write(frame);
}

void WVideo::write(CNv12View frame) {
  assertw(_use_nv12);
  _impl->write_nv12(frame);
}

//----------------------------------------------------------------------------
// *** Video I/O

//----------------------------------------------------------------------------
// *** using Media Foundation
#if defined(HH_VIDEO_HAVE_MF)

// Supported Media Formats in Media Foundation
// https://learn.microsoft.com/en-us/windows/win32/medfound/supported-media-formats-in-media-foundation
// Containers: 3gp, asf, wma, wmv, aac, adts, avi, mp3, m4a, v4v, mov, mp4, sami, smi, wav

#define AS(expr) assertx(SUCCEEDED(expr))

namespace {

class Initialize_COM_MF {
 public:
  Initialize_COM_MF() {
    if (s_num_video_uses++) {
      if (0) return;
    }
    // default may be COINIT_MULTITHREADED, but VT code assumes COINIT_APARTMENTTHREADED
    if (0) {
      // The following could work if we could guarantee that VT initialization occurred before this call.
      HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
      assertx(SUCCEEDED(hr) || hr == RPC_E_CHANGED_MODE);
    } else {
      HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
      assertx(SUCCEEDED(hr) || hr == S_FALSE);  // may equal S_FALSE if COM was previously initialized
    }
    AS(MFStartup(MF_VERSION, MFSTARTUP_FULL));  // initialize Media Foundation
  }
  ~Initialize_COM_MF() {
    if (--s_num_video_uses) return;
    if (1) return;  // there may still be other users, such as Vision Tools.
    AS(MFShutdown());
    CoUninitialize();
  }

 private:
  static std::atomic<int> s_num_video_uses;  // only release MediaFoundation and COM when this reaches zero
};

std::atomic<int> Initialize_COM_MF::s_num_video_uses;

class LockIMFMediaBuffer {
 public:
  explicit LockIMFMediaBuffer(IMFMediaBuffer* buffer) : _buffer(buffer) {
    AS(_buffer->Lock(&_pData, nullptr, nullptr));
  }
  ~LockIMFMediaBuffer() {
    assertx(_pData);
    AS(_buffer->Unlock());
  }
  uint8_t* operator()() const { return _pData; }

 private:
  IMFMediaBuffer* _buffer;
  uint8_t* _pData{nullptr};
};

void retrieve_strided_BGRA(const uint8_t* pData, int stride, MatrixView<Pixel> frame) {
  const int ny = frame.ysize(), nx = frame.xsize();
  uint8_t* pd = frame.data()->data();
  for_int(y, ny) {
    const uint8_t* ps = pData + y * stride;
    for_int(x, nx) {
      // BGRA to RGBA
      pd[0] = ps[2];
      pd[1] = ps[1];
      pd[2] = ps[0];
      pd[3] = 255;
      pd += 4;
      ps += 4;
    }
  }
}

void retrieve_strided_Nv12(const uint8_t* pData, int stride, int offsetUV, Nv12View nv12v) {
  const int ny = nv12v.get_Y().ysize(), nx = nv12v.get_Y().xsize();
  // This is a bottleneck, so worth optimizing.
  if (stride == nv12v.get_Y().xsize()) {
    std::memcpy(nv12v.get_Y().data(), pData, nv12v.get_Y().size());
    std::memcpy(nv12v.get_UV().data(), pData + offsetUV, nv12v.get_UV().size() * 2);
  } else if (1) {
    for_int(y, ny) std::memcpy(nv12v.get_Y()[y].data(), pData + y * stride, nx);
    for_int(y, ny / 2) std::memcpy(nv12v.get_UV()[y].data(), pData + offsetUV + y * stride, nx);
  } else {
    uint8_t* pd = nv12v.get_Y().data();
    for_int(y, ny) {
      const uint8_t* ps = pData + y * stride;
      for ([[maybe_unused]] const int x : range(nx)) *pd++ = *ps++;
    }
    pd = nv12v.get_UV().data()->data();
    for_int(y, ny / 2) {
      const uint8_t* ps = pData + offsetUV + y * stride;
      for ([[maybe_unused]] const int x : range(nx)) *pd++ = *ps++;
    }
  }
}

}  // namespace

class Mf_RVideo_Implementation : public RVideo::Implementation {
 public:
  explicit Mf_RVideo_Implementation(RVideo& rvideo) : RVideo::Implementation(rvideo) {
    {
      com_ptr<IMFSourceResolver> pSourceResolver;
      AS(MFCreateSourceResolver(&pSourceResolver));
      DWORD createObjFlags =
          MF_RESOLUTION_BYTESTREAM | MF_RESOLUTION_CONTENT_DOES_NOT_HAVE_TO_MATCH_EXTENSION_OR_MIME_TYPE;
      MF_OBJECT_TYPE objectType = MF_OBJECT_INVALID;
      com_ptr<IUnknown> pSource;
      if (FAILED(pSourceResolver->CreateObjectFromURL(utf16_from_utf8(_rvideo._filename).c_str(), createObjFlags,
                                                      nullptr, &objectType, &pSource)))
        throw std::runtime_error("Could not open video in file '" + _rvideo._filename + "'");
      assertx(objectType == MF_OBJECT_BYTESTREAM);
      AS(pSource->QueryInterface(IID_PPV_ARGS(&_pByteStream)));
    }
    {
      com_ptr<IMFAttributes> pAttributes;
      AS(MFCreateAttributes(&pAttributes, 2));
      AS(pAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE));
      AS(pAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE));
      if (FAILED(MFCreateSourceReaderFromByteStream(_pByteStream, pAttributes, &_pReader)))
        throw std::runtime_error("Could not open video in file '" + _rvideo._filename + "'");
    }
    double duration;
    {
      PROPVARIANT var;
      PropVariantInit(&var);
      AS(_pReader->GetPresentationAttribute(DWORD(MF_SOURCE_READER_MEDIASOURCE), MF_PD_DURATION, &var));
      assertx(var.vt == VT_UI8);
      ULONGLONG durationInHundredsOfNanoseconds = var.uhVal.QuadPart;
      double TIME_HNS_TO_S_FACTOR = 10'000'000.;
      duration = double(durationInHundredsOfNanoseconds) / TIME_HNS_TO_S_FACTOR;
      PropVariantClear(&var);
    }
    {
      _impl_nv12 = _rvideo._use_nv12;
      if (1 || !IsWindows8OrGreater())
        _impl_nv12 = true;  // It may be ~3.5x faster on Windows 7
                            // Actually it is still 5x faster on Windows 10
    }
    {
      com_ptr<IMFMediaType> pType;
      AS(MFCreateMediaType(&pType));
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
      GUID subtype;
      AS(pType->GetGUID(MF_MT_SUBTYPE, &subtype));
      assertx(subtype == (_impl_nv12 ? MFVideoFormat_NV12 : MFVideoFormat_RGB32));
      UINT32 vnum = 0;
      UINT32 vdenom = 1;
      AS(MFGetAttributeRatio(pType, MF_MT_FRAME_RATE, &vnum, &vdenom));
      _rvideo._attrib.framerate = double(vnum) / double(vdenom);
      if (0) SHOW(_rvideo._attrib.framerate, duration);
      Vec3<int>& dims = _rvideo._dims;
      dims[0] = int(_rvideo._attrib.framerate * duration + .5);  // set nframes
      UINT32 tx = 0;
      UINT32 ty = 0;
      AS(MFGetAttributeSize(pType, MF_MT_FRAME_SIZE, &tx, &ty));
      dims[1] = int(ty), dims[2] = int(tx);  // set ysize, xsize
      _mf_ny = dims[1], _mf_nx = dims[2];    // initial settings
      UINT32 bitrate = 0;
      if (!SUCCEEDED(pType->GetUINT32(MF_MT_AVG_BITRATE, &bitrate))) bitrate = 0;
      _rvideo._attrib.bitrate = int(bitrate);
      UINT32 tstride = 0;
      AS(pType->GetUINT32(MF_MT_DEFAULT_STRIDE, &tstride));
      _stride = tstride;
      assertx(_stride > 0);
      if (0) SHOW(dims, _stride);
    }
  }
  ~Mf_RVideo_Implementation() override {
    // Note that _init_com_mf.~Initialize_COM_MF() is called after this destructor
  }
  string name() const override { return "mf"; }
  bool read(MatrixView<Pixel> frame) override {
    const Vec2<int> sdims = _rvideo.spatial_dims();
    assertx(frame.dims() == sdims);
    com_ptr<IMFSample> pSample = get_sample();
    if (!pSample) return false;
    const int stride = _stride;
    com_ptr<IMFMediaBuffer> pBuffer;
    AS(pSample->ConvertToContiguousBuffer(&pBuffer));
    {
      LockIMFMediaBuffer lock(pBuffer);
      const uint8_t* pData = lock();
      // The following fails in practice on an mp4.
      // com_ptr<IMF2DBuffer> p2Dbuf; AS(pBuffer->QueryInterface(IID_PPV_ARGS(&p2Dbuf)));
      // BYTE* pData; LONG lstride; AS(p2Dbuf->Lock2D(&pData, &lstride)); {
      if (!_impl_nv12) {
        retrieve_strided_BGRA(pData, stride, frame);
      } else {
        int offset = _mf_ny * stride;
        if (stride == sdims[1] * 1) {
          CMatrixView<uint8_t> matY(pData, sdims);
          CMatrixView<Vec2<uint8_t>> matUV(reinterpret_cast<const Vec2<uint8_t>*>(pData + offset), sdims / 2);
          convert_Nv12_to_Image(CNv12View(matY, matUV), frame);
        } else {  // slow path
          Nv12 nv12(sdims);
          retrieve_strided_Nv12(pData, stride, offset, nv12);
          convert_Nv12_to_Image(nv12, frame);
        }
      }
      // } AS(p2Dbuf->Unlock2D());
    }
    return true;
  }
  bool read_nv12(Nv12View nv12v) override {
    const Vec2<int> sdims = _rvideo.spatial_dims();
    assertx(nv12v.get_Y().dims() == sdims);
    com_ptr<IMFSample> pSample = get_sample();
    if (!pSample) return false;
    const int stride = _stride;
    com_ptr<IMFMediaBuffer> pBuffer;
    AS(pSample->ConvertToContiguousBuffer(&pBuffer));
    {
      LockIMFMediaBuffer lock(pBuffer);
      const uint8_t* pData = lock();
      if (_impl_nv12) {
        retrieve_strided_Nv12(pData, stride, _mf_ny * stride, nv12v);
      } else {
        if (stride == sdims[1] * 4) {
          MatrixView<Pixel> mat(reinterpret_cast<Pixel*>(const_cast<uint8_t*>(pData)), sdims);
          for (Pixel& pix : mat) std::swap(pix[0], pix[2]);  // BGRA to RGBA
          convert_Image_to_Nv12(mat, nv12v);
        } else {  // slower path
          Matrix<Pixel> mat(sdims);
          retrieve_strided_BGRA(pData, stride, mat);
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
      AS(_pReader->ReadSample(DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), 0, nullptr, &dwFlags, nullptr, &pSample));
      if (dwFlags & MF_SOURCE_READERF_ENDOFSTREAM) {
        assertx(!pSample);
        return {};
      }
      // It is possible for real media to be 960x540 and then get "changed" to 960x544.
      // Both tx and ty may change:  Filtervideo ~/proj/videoloops/data/ReallyFreakinAll/SDBaiModelS.mp4 -noo
      //   xsize=300 ysize=540  -->  _mf_ny=544 _mf_nx=304 _stride=304
      if ((dwFlags & MF_SOURCE_READERF_CURRENTMEDIATYPECHANGED)) {
        com_ptr<IMFMediaType> pType;
        AS(_pReader->GetCurrentMediaType(DWORD(MF_SOURCE_READER_FIRST_VIDEO_STREAM), &pType));
        UINT32 tx = 0;
        UINT32 ty = 0;
        AS(MFGetAttributeSize(pType, MF_MT_FRAME_SIZE, &tx, &ty));
        _mf_ny = ty;
        _mf_nx = tx;
        assertx(_mf_ny >= _rvideo.ysize() && _mf_nx >= _rvideo.xsize());
        UINT32 tstride = 0;
        AS(pType->GetUINT32(MF_MT_DEFAULT_STRIDE, &tstride));
        _stride = tstride;
        assertx(_stride > 0);
        // SHOW(_mf_ny, _mf_nx, _stride);
      }
      assertw(pSample);
      if (pSample) break;
    }
    return pSample;
  }
  static bool supported() { return true; }

 private:
  Initialize_COM_MF _init_com_mf;  // must be declared first, to be destroyed after remaining members
  com_ptr<IMFByteStream> _pByteStream;
  com_ptr<IMFSourceReader> _pReader;
  bool _impl_nv12;
  int _mf_ny;
  int _mf_nx;
  int _stride;
};

class Mf_WVideo_Implementation : public WVideo::Implementation {
 public:
  explicit Mf_WVideo_Implementation(WVideo& wvideo) : WVideo::Implementation(wvideo) {
    const auto& attrib = _wvideo._attrib;
    if (attrib.audio.size()) Warning("MF_WVideo does not currently support audio");
    _impl_nv12 = attrib.suffix == "mp4";
    if (0 && _impl_nv12) {
      Warning("Disabling _impl_nv12");
      _impl_nv12 = false;
    }
    int iFramesPerSecond = int(attrib.framerate + .5);
    AS(MFFrameRateToAverageTimePerFrame(iFramesPerSecond, 1, &_rtDuration));
    GUID videoOutputFormat;
    if (0)
      void();
    else if (attrib.suffix == "mp4")
      videoOutputFormat = MFVideoFormat_H264;
    else if (attrib.suffix == "mov")
      videoOutputFormat = MFVideoFormat_H264;
    else if (attrib.suffix == "wmv")
      videoOutputFormat = MFVideoFormat_WVC1;
    // Note that Media Foundation does not support *.avi as an output format;
    //  see: https://msdn.microsoft.com/en-us/library/dd757927(VS.85).aspx
    else if (0 && attrib.suffix == "avi")
      videoOutputFormat = MFVideoFormat_420O;  // 8bpc planar YUV 4:2:0
    else
      throw std::runtime_error("Video: encoder suffix '" + attrib.suffix + "' not recognized");
    // See Tutorial: Using the Sink Writer to Encode Video (Windows)
    //  https://learn.microsoft.com/en-us/windows/win32/medfound/tutorial--using-the-sink-writer-to-encode-video
    // Color conversion bug:
    // Use Color Converter DSP (https://learn.microsoft.com/en-us/windows/win32/medfound/colorconverter)
    // See "Work around for XVP color conversion problem" in
    //   ~/git/CompPhoto/ClassLibs/VisionTools/src/fileio/MFVideoSrc.h
    {
      com_ptr<IMFAttributes> pAttributes;
      AS(MFCreateAttributes(&pAttributes, 10));
      AS(pAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE));
      // AS(pAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, FALSE));  // no effect
      // AS(pAttributes->SetUINT32(MF_READWRITE_DISABLE_CONVERTERS, TRUE));  // causes it to fail
      AS(pAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, TRUE));  // no effect
      // AS(pAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_VIDEO_PROCESSING, FALSE));  // no effect
      IMFByteStream* const k_ByteStream = nullptr;
      if (FAILED(MFCreateSinkWriterFromURL(utf16_from_utf8(_wvideo._filename).c_str(), k_ByteStream,
                                           nullptr /*pAttributes*/, &_pSinkWriter)))
        throw std::runtime_error("Could not write video to file '" + _wvideo._filename + "'");
    }
    {
      com_ptr<IMFMediaType> pMediaTypeOut;
      AS(MFCreateMediaType(&pMediaTypeOut));
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
      com_ptr<IMFMediaType> pMediaTypeIn;
      AS(MFCreateMediaType(&pMediaTypeIn));
      AS(pMediaTypeIn->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
      GUID video_format = MFVideoFormat_RGB32;
      if (_impl_nv12) video_format = MFVideoFormat_NV12;
      AS(pMediaTypeIn->SetGUID(MF_MT_SUBTYPE, video_format));
      AS(pMediaTypeIn->SetUINT32(MF_MT_DEFAULT_STRIDE, _wvideo.xsize() * (_impl_nv12 ? 1 : 4)));
      AS(pMediaTypeIn->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
      AS(MFSetAttributeSize(pMediaTypeIn, MF_MT_FRAME_SIZE, _wvideo.xsize(), _wvideo.ysize()));
      AS(MFSetAttributeRatio(pMediaTypeIn, MF_MT_FRAME_RATE, iFramesPerSecond, 1));
      AS(MFSetAttributeRatio(pMediaTypeIn, MF_MT_PIXEL_ASPECT_RATIO, 1, 1));
      com_ptr<IMFAttributes> pAttributes;  // pEncodingParameters
      // IMFAttributes* const pEncodingParameters = nullptr;
      if (0) {  // untested
        AS(MFCreateAttributes(&pAttributes, 10));
        if (0) {
          unsigned force_keyframe_every_nframes = 20;
          AS(pAttributes->SetUINT32(CODECAPI_AVEncMPVGOPSize, force_keyframe_every_nframes));
        }
        if (0) {
          unsigned quality = 78;
          AS(pAttributes->SetUINT32(CODECAPI_AVEncCommonRateControlMode, eAVEncCommonRateControlMode_Quality));
          AS(pAttributes->SetUINT32(CODECAPI_AVEncCommonQuality, quality));
        }
      }
      AS(_pSinkWriter->SetInputMediaType(_streamIndex, pMediaTypeIn, pAttributes));
    }
    AS(_pSinkWriter->BeginWriting());
  }
  ~Mf_WVideo_Implementation() override {
    if (_pSinkWriter) AS(_pSinkWriter->Finalize());
    // Note that _init_com_mf.~Initialize_COM_MF() is called after this destructor
  }
  string name() const override { return "mf"; }
  void write(CMatrixView<Pixel> frame) override {
    const Vec2<int> sdims = _wvideo.spatial_dims();
    assertx(product(_wvideo.spatial_dims()));
    assertx(frame.dims() == _wvideo.spatial_dims());
    DWORD cbBuffer = DWORD(product(_wvideo.spatial_dims()) * (_impl_nv12 ? 1.5f : 4.f));
    com_ptr<IMFMediaBuffer> pBuffer;
    AS(MFCreateAlignedMemoryBuffer(cbBuffer, MF_16_BYTE_ALIGNMENT, &pBuffer));
    {
      LockIMFMediaBuffer lock(pBuffer);
      uint8_t* pData = lock();
      if (!_impl_nv12) {
        uint8_t* pd = pData;
        const uint8_t* ps = frame.data()->data();
        for_int(i, sdims[0] * sdims[1]) {
          // RGBA to BGRA
          pd[0] = ps[2];
          pd[1] = ps[1];
          pd[2] = ps[0];
          pd[3] = 0;
          pd += 4;
          ps += 4;
        }
      } else {
        // Media Foundation MP4 encoding under Win7 may have poor quality -- independent of this workaround.
        MatrixView<uint8_t> matY(pData, _wvideo.spatial_dims());
        MatrixView<Vec2<uint8_t>> matUV(reinterpret_cast<Vec2<uint8_t>*>(pData + sdims[0] * sdims[1]),
                                        _wvideo.spatial_dims() / 2);
        convert_Image_to_Nv12(frame, Nv12View(matY, matUV));
      }
    }
    AS(pBuffer->SetCurrentLength(cbBuffer));
    com_ptr<IMFSample> pSample;
    AS(MFCreateSample(&pSample));
    AS(pSample->AddBuffer(pBuffer));
    AS(pSample->SetSampleTime(_rtStart));
    AS(pSample->SetSampleDuration(_rtDuration));
    AS(_pSinkWriter->WriteSample(_streamIndex, pSample));
    _rtStart += _rtDuration;
  }
  void write_nv12(CNv12View nv12v) override {
    const Vec2<int> sdims = _wvideo.spatial_dims();
    assertx(product(_wvideo.spatial_dims()));
    assertx(nv12v.get_Y().dims() == _wvideo.spatial_dims());
    DWORD cbBuffer = DWORD(product(_wvideo.spatial_dims()) * (_impl_nv12 ? 1.5f : 4.f));
    com_ptr<IMFMediaBuffer> pBuffer;
    AS(MFCreateAlignedMemoryBuffer(cbBuffer, MF_16_BYTE_ALIGNMENT, &pBuffer));
    {
      LockIMFMediaBuffer lock(pBuffer);
      uint8_t* pData = lock();
      if (_impl_nv12) {
        // Media Foundation MP4 encoding under Win7 may have poor quality; it is independent of this workaround
        MatrixView<uint8_t> matY(pData, _wvideo.spatial_dims());
        MatrixView<Vec2<uint8_t>> matUV(reinterpret_cast<Vec2<uint8_t>*>(pData + sdims[0] * sdims[1]),
                                        _wvideo.spatial_dims() / 2);
        matY.assign(nv12v.get_Y());
        matUV.assign(nv12v.get_UV());
      } else {
        MatrixView<Pixel> mat(reinterpret_cast<Pixel*>(pData), _wvideo.spatial_dims());
        convert_Nv12_to_Image(nv12v, mat);
        for (Pixel& pix : mat) std::swap(pix[0], pix[2]);  // RGBA to BGRA
      }
    }
    AS(pBuffer->SetCurrentLength(cbBuffer));
    com_ptr<IMFSample> pSample;
    AS(MFCreateSample(&pSample));
    AS(pSample->AddBuffer(pBuffer));
    AS(pSample->SetSampleTime(_rtStart));
    AS(pSample->SetSampleDuration(_rtDuration));
    AS(_pSinkWriter->WriteSample(_streamIndex, pSample));
    _rtStart += _rtDuration;
  }
  static bool supported() { return true; }

 private:
  Initialize_COM_MF _init_com_mf;  // must be declared first, to be destroyed after remaining members
  com_ptr<IMFSinkWriter> _pSinkWriter;
  DWORD _streamIndex;
  UINT64 _rtDuration;
  LONGLONG _rtStart{0};
  bool _impl_nv12;  // otherwise, Windows 7 Media Foundation shifts colors during MP4 compression!
  // for *.wmv files, _impl_nv12 does not speed up compression -- it is always about 3x slower than MP4 in Win7.
};

#else

class Mf_RVideo_Implementation : public Unsupported_RVideo_Implementation {
 public:
  Mf_RVideo_Implementation(RVideo& rvideo) : Unsupported_RVideo_Implementation(rvideo) {}
  static bool supported() { return false; }
};
class Mf_WVideo_Implementation : public Unsupported_WVideo_Implementation {
 public:
  Mf_WVideo_Implementation(WVideo& wvideo) : Unsupported_WVideo_Implementation(wvideo) {}
  static bool supported() { return false; }
};

#endif  // defined(HH_VIDEO_HAVE_MF)

//----------------------------------------------------------------------------
// *** using ffmpeg
#if defined(HH_VIDEO_HAVE_FFMPEG)

class Ffmpeg_RVideo_Implementation : public RVideo::Implementation {
 public:
  explicit Ffmpeg_RVideo_Implementation(RVideo& rvideo) : RVideo::Implementation(rvideo) {
    assertx(supported());
    // See: ~/git/hh_src/_other/Repository.cpp
    // See: ffmpeg -hide_banner -pix_fmts
    const bool ldebug = getenv_bool("FFMPEG_DEBUG");
    const string& filename = _rvideo._filename;
    bool expect_audio = false;
    // This prefix is necessary to get a correct frame count in the case of *.gif generated from ffmpeg.
    string prefix = ends_with(filename, ".gif") ? " -r 60 -vsync vfr" : "";
    {  // read header for dimensions and attributes (ignore video and audio data)
      // 2>&1 works on both Unix bash shell and Windows cmd shell: https://stackoverflow.com/questions/1420965/
      // Ideally, <nul and/or </dev/null so that "vv ~/proj/fastloops/data/assembled_all_loops_uhd.mp4" does
      //  not stop responding.
      // Instead, use -nostdin .  Yes, it works.  Moreover, now do dup2(1, 0) in VideoViewer.
      //
      // TODO: Read and preserve metadata in video:  yes, likely also more robust way to read attributes below.
      // https://stackoverflow.com/a/9473239:
      //  https://www.ffmpeg.org/ffmpeg-formats.html#Metadata-1
      // ffmpeg -i input_video -f ffmetadata metadata.txt
      // ffmpeg -i input_video -i ffmetadata.txt -map_metadata 1 -codec copy output_video
      // (BTW, ffmpeg supports multiple outputs: https://trac.ffmpeg.org/wiki/Creating%20multiple%20outputs
      //   ffmpeg -i input1 -i input2 -acodec ... -vcodec ... output1 -acodec ... -vcodec ... output2 )
      // https://superuser.com/questions/349518/
      // ffmpeg -i video.mp4 -vn -acodec copy -metadata title="My Title" audio.m4a
      // ffmpeg -i ~/data/video/HDbrink8h.mp4 -f ffmetadata v.txt && cat v.txt
      // ;FFMETADATA1
      // major_brand=mp42
      // minor_version=0
      // compatible_brands=mp41isom
      // encoder=Lavf56.4.101
      // Conclusion: not useful because already given in regular stdout of ffmpeg
      // We could look at ffprobe to see if it can output more information, or use exiftool.
      // Option "-nostdin" is unrecognized by older versions, but it can continue nonetheless.
      string s = "ffmpeg -nostdin" + prefix + " -i " + quote_arg_for_shell(filename) +
                 " -acodec copy -vcodec copy -f null - 2>&1 |";
      if (ldebug) SHOW(s);
      RFile fi(s);
      Vec3<int> dims{0, 0, 0};
      double total_bitrate = 1'000'000.;  // a default value of 1Mbps
      double video_bitrate = -1.;
      double framerate = -1.;
      bool yuv444p = false;
      int nlines = 0;
      string line;
      while (my_getline(fi(), line, false)) {
        nlines++;
        if (ldebug) SHOW(line);
        char vch;
        if (contains(line, "Could not find option 'nostdin'")) {
          Warning("Version of external program 'ffmpeg' may be too old");
          continue;
        }
        {
          auto i = line.find(", bitrate:");
          if (i != string::npos && !starts_with(line.substr(i), ", bitrate: N/A")) {
            assertx(sscanf(line.c_str() + i, ", bitrate: %lg kb/%c", &total_bitrate, &vch) == 2 && vch == 's');
            total_bitrate *= 1000.;
          }
        }
        if (contains(line, "Stream #0:")) {
          if (contains(line, ": Video:") && !dims[2]) {
            for (string::size_type i = 0;;) {
              i = line.find(',', i + 1);
              assertx(i != string::npos);
              if (sscanf(line.c_str() + i, ", %dx%d", &dims[2], &dims[1]) == 2) break;
            }
            string::size_type i = line.find(" kb/s");
            if (i != string::npos) {
              i = line.rfind(", ", i);
              if (i != string::npos) {
                assertx(sscanf(line.c_str() + i, ", %lg kb/%c", &video_bitrate, &vch) == 2 && vch == 's');
                video_bitrate *= 1000.;
              }
            }
            i = line.find(" fps");
            if (i != string::npos) {
              i = line.rfind(", ", i);
              assertx(i != string::npos);
              assertx(sscanf(line.c_str() + i, ", %lg fp%c", &framerate, &vch) == 2 && vch == 's');
            } else {
              i = line.find(" tbr");
              assertx(i != string::npos);
              i = line.rfind(", ", i);
              assertx(i != string::npos);
              assertx(sscanf(line.c_str() + i, ", %lg tb%c", &framerate, &vch) == 2 && vch == 'r');
            }
            if (line.find("yuv444p") != string::npos) yuv444p = true;
            if (ldebug) SHOW(dims[2], dims[1], video_bitrate, framerate, yuv444p);
          }
          if (contains(line, ": Audio:") && contains(line, "kb/s")) expect_audio = true;
        }
        {
          string::size_type i = line.rfind("frame=");
          if (i != string::npos) assertx(sscanf(line.c_str() + i, "frame=%d%c", &dims[0], &vch) == 2 && vch == ' ');
        }
      }
      if (!nlines) throw std::runtime_error("ffmpeg is unable to read video file '" + filename + "'");
      if (!dims[0]) throw std::runtime_error("found zero frames in video file '" + filename + "'");
      if (!dims[1] || !dims[2]) throw std::runtime_error("no video stream in video file '" + filename + "'");
      if (framerate < 0.) throw std::runtime_error("no framerate in video file '" + filename + "'");
      if (ends_with(filename, ".gif") && framerate >= 21. && framerate <= 29.) framerate = 60.;
      if (video_bitrate < 0.) video_bitrate = total_bitrate;
      if (ldebug) SHOW(dims, total_bitrate, video_bitrate, framerate);
      assertx(product(dims) > 0);
      _rvideo._dims = dims;
      _rvideo._attrib.bitrate = int(video_bitrate);
      _rvideo._attrib.framerate = framerate;
      if (yuv444p && _rvideo._use_nv12) Warning("Reading NV12 from a Video encoded with yuv444p");
    }
    if (expect_audio) {
      try {
        Audio& audio = _rvideo._attrib.audio;
        audio.read_file(filename);
        if (ldebug) SHOW(audio.diagnostic_string());
      } catch (std::runtime_error& ex) {
        SHOW("Failed to read audio within video", filename, ex.what());
      }
    }
    string pixfmt = _rvideo._use_nv12 ? "nv12" : "rgba";  // was rgb24
    string command = ("ffmpeg -v panic -nostdin" + prefix + " -i " + quote_arg_for_shell(filename) +
                      " -f image2pipe -pix_fmt " + pixfmt + " -vcodec rawvideo - |");
    if (ldebug) SHOW(command);
    _pfi = make_unique<RFile>(command);
  }
  ~Ffmpeg_RVideo_Implementation() override = default;
  string name() const override { return "ffmpeg"; }
  bool read(MatrixView<Pixel> frame) override {
    const Vec2<int> sdims = _rvideo.spatial_dims();
    assertx(frame.dims() == sdims);
    if (!_rvideo._use_nv12) {
      if (!read_binary_raw((*_pfi)(), frame.array_view())) return false;
      if (0) for_int(i, int(product(sdims))) frame.flat(i)[3] = 255;
    } else {
      _ar_tmp.init(assert_narrow_cast<int>(product(sdims) + product(sdims) / 2));  // NV12
      if (!read_binary_raw((*_pfi)(), _ar_tmp)) return false;
      CMatrixView<uint8_t> matY(_ar_tmp.data(), sdims);
      CMatrixView<Vec2<uint8_t>> matUV(reinterpret_cast<const Vec2<uint8_t>*>(_ar_tmp.data() + product(sdims)),
                                       sdims / 2);
      convert_Nv12_to_Image(CNv12View(matY, matUV), frame);
    }
    return true;
  }
  bool read_nv12(Nv12View nv12v) override {
    const Vec2<int> sdims = _rvideo.spatial_dims();
    assertx(nv12v.get_Y().dims() == sdims);
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
      (*_pfi)().ignore(product(sdims) + product(sdims) / 2);
    } else {
      (*_pfi)().ignore(product(sdims) * 4);
    }
    return !!(*_pfi)();
  }
  static bool supported() { return ffmpeg_command_exists(); }

 private:
  unique_ptr<RFile> _pfi;
  Array<uint8_t> _ar_tmp;
  Matrix<Pixel> _frame_rgb_tmp;
};

class Ffmpeg_WVideo_Implementation : public WVideo::Implementation {
 public:
  explicit Ffmpeg_WVideo_Implementation(WVideo& wvideo) : WVideo::Implementation(wvideo) {
    const bool ldebug = getenv_bool("FFMPEG_DEBUG");
    assertx(supported());
    const string& filename = _wvideo._filename;
    Video::Attrib& attrib = _wvideo._attrib;
    const Vec2<int> sdims = _wvideo.spatial_dims();
    string sfilecontainer;  // default is to infer container format based on filename extension
    // ffmpeg -hide_banner -formats
    {
      assertx(filename != "-");  // WVideo::WVideo() should have created TmpFile if writing to stdout.
      // (Note that ffmpeg -f mp4 is not supported over pipe because it requires a seekable output;
      //  without -f, it actually uses avi as a default container, which doesn't match the mp4 file suffix.)
      // At some point we managed to get Win7 Windows Media Player to play some type of file by creating an avi
      //  container with a file suffix of mp4.  Not good for the long term.
      // if (attrib.suffix == "mp4") sfilecontainer = " -f avi";
      // Setting container should be unnecessary because it is derived automatically based on file suffix.
    }
    string ipixfmt = _wvideo._use_nv12 ? "nv12" : "rgba";  // was rgb24
    string str_audio = " -an";                             // default no audio
    string ocodec;
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
    if (attrib.suffix == "wmv") Warning("Ffmpeg_WVideo_Implementation uses msmpeg4v3 (not vc1) for *.wmv");
    if (0 && attrib.suffix == "mp4") ocodec = " -vcodec h264";
    if (0 && attrib.suffix == "mov") ocodec = " -vcodec h264";
    if (attrib.suffix == "avi") {
      ocodec = " -c:v ffvhuff";  // "effectively lossless"
      opixfmt = "yuv444p";       // lossless (not NV12 like "yuv420p")
      if (_wvideo._use_nv12) Warning("Using NV12 to write to a *.avi Video with yuv444p encoding");
    }
    Audio& audio = attrib.audio;
    if (audio.size()) {
      if (ldebug) SHOW("previously", audio.attrib().suffix);
      audio.attrib().suffix = "aac";  // or "mp3"
      // Create a temporary file containing the audio encoded as aac.
      _tmpfile_audio = make_unique<TmpFile>(audio.attrib().suffix);
      try {
        audio.write_file(_tmpfile_audio->filename());
        str_audio = " -i " + _tmpfile_audio->filename() + sform(" -ab %d", audio.attrib().bitrate);
      } catch (std::runtime_error& ex) {
        SHOW("Failed to write audio within video", ex.what());
      }
    }
    string command =
        ("| ffmpeg -v panic -f rawvideo -vcodec rawvideo -pix_fmt " + ipixfmt +
         sform(" -s %dx%d -r %g", sdims[1], sdims[0], attrib.framerate) + " -i -" + str_audio + sfilecontainer +
         ocodec + " -pix_fmt " + opixfmt + sform(" -vb %d", attrib.bitrate) + " -y " + quote_arg_for_shell(filename));
    if (ldebug) SHOW(command);
    _pfi = make_unique<WFile>(command);
  }
  ~Ffmpeg_WVideo_Implementation() override = default;
  string name() const override { return "ffmpeg"; }
  void write(CMatrixView<Pixel> frame) override {
    const Vec2<int> sdims = _wvideo.spatial_dims();
    assertx(product(sdims));
    assertx(frame.dims() == sdims);
    if (!_wvideo._use_nv12) {
      if (!write_binary_raw((*_pfi)(), frame.array_view()))
        throw std::runtime_error("Ffmpeg_WVideo write frame failed");
    } else {
      _frame_nv12_tmp.init(sdims);
      convert_Image_to_Nv12(frame, _frame_nv12_tmp);
      return write_nv12(_frame_nv12_tmp);
    }
  }
  void write_nv12(CNv12View nv12v) override {
    const Vec2<int> sdims = _wvideo.spatial_dims();
    assertx(product(sdims));
    assertx(nv12v.get_Y().dims() == sdims);
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
  unique_ptr<TmpFile> _tmpfile_audio;  // lifespan should be longer than _pfi
  unique_ptr<WFile> _pfi;
  Nv12 _frame_nv12_tmp;
  Matrix<Pixel> _frame_rgb_tmp;
};

#endif  // defined(HH_VIDEO_HAVE_FFMPEG)

//----------------------------------------------------------------------------

unique_ptr<RVideo::Implementation> RVideo::Implementation::make(RVideo& rvideo) {
  // Notes:
  // - Win7 Media Foundation: For some mp4 files, e.g. ~/data/video/fewpalms.mp4, bitrate is read as 0.
  //   e.g.:  RVIDEO_IMPLEMENTATION=mf Filtervideo ~/data/video/fewpalms.mp4 -stat
  string implementation = getenv_string("RVIDEO_IMPLEMENTATION");
  if (implementation == "") implementation = getenv_string("VIDEO_IMPLEMENTATION");
  if (implementation == "mf") return make_unique<Mf_RVideo_Implementation>(rvideo);
  if (implementation == "ffmpeg") return make_unique<Ffmpeg_RVideo_Implementation>(rvideo);
  if (implementation != "") throw std::runtime_error("RVideo implementation '" + implementation + "' not recognized");
  if (Ffmpeg_RVideo_Implementation::supported()) return make_unique<Ffmpeg_RVideo_Implementation>(rvideo);
  if (Mf_RVideo_Implementation::supported()) return make_unique<Mf_RVideo_Implementation>(rvideo);
  throw std::runtime_error("Video I/O not implemented");
}

unique_ptr<WVideo::Implementation> WVideo::Implementation::make(WVideo& wvideo) {
  // Notes:
  // - Win7 Media Foundation: we can set a large bitrate on mp4, but do not see increase in file size.
  //   e.g.: VIDEO_IMPLEMENTATION=mf Filtervideo ~/data/video/fewpalms.mp4 -info -bitrate 32.84m -info -to mp4 | Filtervideo -stat
  // - ffmpeg: cannot write *.wmv using VC1 codec (instead resorts to msmpeg4v3).
  string implementation = getenv_string("WVIDEO_IMPLEMENTATION");
  if (implementation == "") implementation = getenv_string("VIDEO_IMPLEMENTATION");
  if (implementation == "mf") return make_unique<Mf_WVideo_Implementation>(wvideo);
  if (implementation == "ffmpeg") return make_unique<Ffmpeg_WVideo_Implementation>(wvideo);
  if (implementation != "") throw std::runtime_error("WVideo implementation '" + implementation + "' not recognized");
  // Prefer ffmpeg, but revert to Windows Media Foundation (if available) for *.wmv output (using VC1 encoder).
  if (wvideo._attrib.suffix == "wmv" && Mf_WVideo_Implementation::supported())
    return make_unique<Mf_WVideo_Implementation>(wvideo);
  if (Ffmpeg_WVideo_Implementation::supported()) return make_unique<Ffmpeg_WVideo_Implementation>(wvideo);
  if (Mf_WVideo_Implementation::supported()) return make_unique<Mf_WVideo_Implementation>(wvideo);
  throw std::runtime_error("Video I/O not implemented");
}

}  // namespace hh
