// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Image.h"  // HH_IMAGE_HAVE_WIC

#if !defined(HH_IMAGE_HAVE_WIC)

extern void Image_wic_dummy_function_to_avoid_linkage_warnings();
void Image_wic_dummy_function_to_avoid_linkage_warnings() {}

#else

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>  // required by WIC; must appear before other headers.

#include <Shlwapi.h>      // SHCreateMemStream()
#include <wincodec.h>     // WIC; e.g., c:/Program Files (x86)/Windows Kits/10/Include/10.0.19041.0/um/wincodec.h
#include <wincodecsdk.h>  // IWICMetadataBlockReader

HH_REFERENCE_LIB("ole32.lib");     // CoInitializeEx() and CoCreateInstance()
HH_REFERENCE_LIB("oleaut32.lib");  // VARIANT; odd: required from command-line cl.exe but not from msbuild.exe
HH_REFERENCE_LIB("shlwapi.lib");   // SHCreateMemStream()

#include <cctype>  // isalnum(), toupper()

// #include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"
#include "libHh/StringOp.h"
#include "libHh/windows_com.h"  // com_ptr<>
using namespace hh;

// JPEG EXIF:
// We cannot find a way to store the metadata associated with an image into memory (e.g. Image::Attrib::exif_data).
// WIC allows traversing the multiple metadata containers and all of their elements (of different types).
// Storing all this seems hard, and reconstructing the proper hierarchy during writing may be impossible.
// WIC really expects that we keep open the input image, with an active IWICMetadataBlockReader,
//  so that the metadata can be copied using "piBlockWriter->InitializeFromBlockReader(piBlockReader);"
//  (see How-to: Re-encode a JPEG Image with Metadata:
//    https://learn.microsoft.com/en-us/windows/win32/wic/-wic-codec-jpegmetadataencoding ).
// My current approach is to store the input image filename into Image::Attrib::orig_filename.
// If the input was read from a pipe, we issue a warning about missing Exif when writing the image.

namespace hh {

namespace {

const std::wstring k_orientation_flag = L"/app1/ifd/{ushort=274}";  // EXIF tag for image orientation

#define AS(expr) assertt(SUCCEEDED(expr))

inline bool my_GlobalUnlock(HGLOBAL hGbl) { return !GlobalUnlock(hGbl) && GetLastError() == NO_ERROR; }

IWICImagingFactory* wic_factory = nullptr;

void wic_init() {
  if (wic_factory) return;
  // default may be COINIT_MULTITHREADED, but VT code assumes COINIT_APARTMENTTHREADED
  HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
  assertx(SUCCEEDED(hr) || hr == S_FALSE);  // may equal S_FALSE if COM was previously initialized
  AS(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&wic_factory)));
}

struct SuffixGuid {
  string suffix;
  const GUID* guid;
};
const Array<SuffixGuid> k_ar_suffix_container = {
    SuffixGuid{"bmp", &GUID_ContainerFormatBmp}, SuffixGuid{"png", &GUID_ContainerFormatPng},
    SuffixGuid{"jpg", &GUID_ContainerFormatJpeg}, SuffixGuid{"jpeg", &GUID_ContainerFormatJpeg},
    SuffixGuid{"tif", &GUID_ContainerFormatTiff}, SuffixGuid{"tiff", &GUID_ContainerFormatTiff},
    SuffixGuid{"gif", &GUID_ContainerFormatGif}, SuffixGuid{"wmp", &GUID_ContainerFormatWmp},
    SuffixGuid{"dds", &GUID_ContainerFormatDds}, SuffixGuid{"heif", &GUID_ContainerFormatHeif},
    // Decoder but no encoder for these:
    SuffixGuid{"adng", &GUID_ContainerFormatAdng}, SuffixGuid{"webp", &GUID_ContainerFormatWebp},
    SuffixGuid{"raw", &GUID_ContainerFormatRaw},
    // Not present: "avif" ?
};

const GUID* get_container_format(const string& suffix) {
  for (const auto& p : k_ar_suffix_container)
    if (p.suffix == suffix) return p.guid;
  return nullptr;
}

const string& get_suffix(const GUID* container_format) {
  for (const auto& p : k_ar_suffix_container)
    if (*p.guid == *container_format) return p.suffix;
  static const string& k_snull = *new string;
  return k_snull;
}

const Array<WICPixelFormatGUID> k_pixel_formats_with_alpha = {
    GUID_WICPixelFormat128bppPRGBAFloat,    GUID_WICPixelFormat128bppRGBAFloat, GUID_WICPixelFormat32bppBGRA,
    GUID_WICPixelFormat32bppPBGRA,          GUID_WICPixelFormat40bppCMYKAlpha,  GUID_WICPixelFormat64bppBGRA,
    GUID_WICPixelFormat64bppBGRAFixedPoint, GUID_WICPixelFormat64bppPRGBA,      GUID_WICPixelFormat64bppRGBA,
    GUID_WICPixelFormat64bppRGBAFixedPoint, GUID_WICPixelFormat80bppCMYKAlpha,
};

#if 0
HRESULT my_progress_callback(void* data, ULONG framenum, WICProgressOperation, double progress) {
  dummy_use(framenum);
  ConsoleProgress& cprogress = *static_cast<ConsoleProgress*>(data);
  cprogress.update(float(progress));
  return S_OK;
}
#endif

class my_HGLOBAL {
 public:
  ~my_HGLOBAL() { *this = nullptr; }
  my_HGLOBAL& operator=(HGLOBAL p) {
    if (_hMem) assertx(!GlobalFree(_hMem));
    _hMem = p;
    return *this;
  }
  operator HGLOBAL() const { return _hMem; }

 private:
  HGLOBAL _hMem{nullptr};
};

string canonical_pathname(string s) {
  s = get_canonical_path(s);
  if (starts_with(s, "/")) s = "C:" + s;
  if (s.size() > 2 && std::isalnum(s[0]) && s[1] == ':') s[0] = char(std::toupper(s[0]));
  if (s.size() < 3 || s[2] != '/') assertnever("unexpected pathname in " + s);
  return s;
}

}  // namespace

void Image::read_file_wic(const string& filename, bool bgra) {
  const string filename_suffix = to_lower(get_path_extension(filename));
  if (contains(V<string>("avif", "heif"), filename_suffix)) {
    // WIC is providing incorrect colors when decoding; maybe it is a color space issue; resort to ffmpeg.
    read_file_ffmpeg(filename, bgra);
    return;
  }
  wic_init();
  Array<uchar> buffer;
  com_ptr<IStream> input_stream;
  const WICDecodeOptions opts = WICDecodeMetadataCacheOnDemand;
  com_ptr<IWICBitmapDecoder> decoder;
  bool success;
  if (file_requires_pipe(filename)) {
    RFile fi(filename);
    std::istream& is = fi();
    std::streamsize nread = 0;
    for (; is;) {
      int chunk = 1024 * 1024;  // 1 MiB
      // Note: size of input stream cannot be >= (1ull << 32).
      buffer.resize(assert_narrow_cast<int>(int64_t{nread} + chunk));
      is.read(reinterpret_cast<char*>(buffer.data()) + nread, chunk);
      nread += is.gcount();
    }
    buffer.resize(assert_narrow_cast<int>(nread));
    input_stream.reset(assertx(SHCreateMemStream(buffer.data(), buffer.num())));
    success = SUCCEEDED(wic_factory->CreateDecoderFromStream(input_stream, nullptr, opts, &decoder));
  } else {
    if (!file_exists(filename)) throw std::runtime_error("Image file '" + filename + "' does not exist");
    success = SUCCEEDED(wic_factory->CreateDecoderFromFilename(utf16_from_utf8(filename).c_str(), nullptr,
                                                               GENERIC_READ, opts, &decoder));
  }
  if (!success) throw std::runtime_error("Could not read and decode image in file '" + filename + "'");
  {
    unsigned frame_count;
    AS(decoder->GetFrameCount(&frame_count));
    assertw(frame_count == 1);
  }
  if (0) {
    // This reveals that a *.avif file is parsed by "Microsoft HEIF Decoder" which handles many file suffixes
    //  (.heic,.heif,.hif,.avci,.heics,.heifs,.avcs,.avif,.avifs) so it does not reveal the codec used.
    com_ptr<IWICBitmapDecoderInfo> decoderInfo;
    AS(decoder->GetDecoderInfo(&decoderInfo));

    GUID codecCLSID;
    AS(decoderInfo->GetCLSID(&codecCLSID));

    UINT nameLength = 0;
    AS(decoderInfo->GetFriendlyName(0, nullptr, &nameLength));

    std::vector<WCHAR> friendlyName(nameLength);
    AS(decoderInfo->GetFriendlyName(nameLength, friendlyName.data(), &nameLength));

    string codecName = utf8_from_utf16(friendlyName.data());
    SHOW(codecName);

    UINT numFileExtensions = 0;
    AS(decoderInfo->GetFileExtensions(0, nullptr, &numFileExtensions));

    std::vector<WCHAR> fileExtensions(numFileExtensions);
    AS(decoderInfo->GetFileExtensions(numFileExtensions, fileExtensions.data(), &numFileExtensions));

    string supportedExtensions = utf8_from_utf16(fileExtensions.data());
    SHOW(supportedExtensions);
  }
  if (0) {  // For the *.avif file I tested, there was no ColorContext found.
    com_ptr<IWICColorContext> colorContext;
    unsigned actualCount;
    AS(decoder->GetColorContexts(1, &colorContext, &actualCount));
    assertw(actualCount == 1);
  }
  {
    GUID container_format;
    AS(decoder->GetContainerFormat(&container_format));
    const GUID* expected_format = get_container_format(filename_suffix);
    if (expected_format && container_format != *expected_format) {
      string container_suffix = get_suffix(&container_format);
      if (container_suffix == "") container_suffix = "beyond_compile_specs";
      SHOW(filename, filename_suffix, container_suffix);
      Warning("Image read: encoded content does not match filename suffix");
    }
    set_suffix(get_suffix(&container_format));
    if (suffix() == "heif" && !expected_format) set_suffix("avif");  // Just a guess.
    if (suffix() == "avif") Warning("WIC decoding of *.avif likely gives incorrect pixel values");
  }
  ushort orientation = 1;  // default is normal orientation (range is 1..8)
  {
    com_ptr<IWICBitmapFrameDecode> frame_decode;
    AS(decoder->GetFrame(0, &frame_decode));
    // HOW(RefCount(frame_decode));  // note: it is 2 because also referenced by decoder
    unsigned width = 0, height = 0;
    AS(frame_decode->GetSize(&width, &height));
    init(V(int(height), int(width)));
    // double dpi_X = 0., dpi_Y = 0.; AS(frame_decode->GetResolution(&dpi_X, &dpi_Y));
    WICPixelFormatGUID pixel_format;
    my_zero(pixel_format);
    AS(frame_decode->GetPixelFormat(&pixel_format));
    set_zsize(k_pixel_formats_with_alpha.contains(pixel_format) ? 4 : 3);  // ignore grayscale for now
    {
      // https://msdn.microsoft.com/en-us/library/windows/desktop/ee719904%28v=vs.85%29.aspx
      // https://github.com/Microsoft/DirectXTex/blob/master/DirectXTex/DirectXTexWIC.cpp
      com_ptr<IWICMetadataQueryReader> pQueryReader;
      if (SUCCEEDED(frame_decode->GetMetadataQueryReader(&pQueryReader))) {
        PROPVARIANT propvariant;
        PropVariantInit(&propvariant);
        if (SUCCEEDED(pQueryReader->GetMetadataByName(k_orientation_flag.c_str(), &propvariant)) &&
            assertw(propvariant.vt == VT_UI2)) {
          orientation = propvariant.uiVal;  // 1..8; 1 == normal, 6 == rotate_ccw
        }
        PropVariantClear(&propvariant);
        if (0) {
          // For HEIF containers, we should try to determine if the codec is HEIC or AV1 (AVIF).
          // I attempted to retrieve this by querying the metadata, but it is not there, and there are few docs.
          SHOWL;
          const std::wstring k_major_brand = L"/app1/ifd/exif/{ushort=16069}";
          PropVariantInit(&propvariant);
          if (SUCCEEDED(pQueryReader->GetMetadataByName(k_major_brand.c_str(), &propvariant)) &&
              assertw(propvariant.vt == VT_LPSTR)) {
            const string brand = propvariant.pszVal;
            SHOW(brand);
          }
          PropVariantClear(&propvariant);
        }
      }
    }
    com_ptr<IWICFormatConverter> converter;
    AS(wic_factory->CreateFormatConverter(&converter));
    // Pixel formats: https://learn.microsoft.com/en-us/windows/win32/wic/-wic-codec-native-pixel-formats
    //  PRGBA is premultiplied alpha (whereas RGBA is not)
    // We want to decode the incoming pixels as non-premultiplied alpha (RGBA or BGRA) so that the decoder
    //  does not do any transformation, although the values are actually (often) pre-multiplied alpha.
    // "In the absence of color space information for an image, the general rule for color space inference
    //   is that UINT RGB and grayscale formats use the standard RGB color space (sRGB), while fixed-point and
    //   floating-point RGB and grayscale formats use the extended RGB color space (scRGB). The CMYK color
    //   model uses an RWOP color space."
    WICPixelFormatGUID pixel_format2 = bgra ? GUID_WICPixelFormat32bppBGRA : GUID_WICPixelFormat32bppRGBA;
    AS(converter->Initialize(frame_decode, pixel_format2, WICBitmapDitherTypeNone,
                             nullptr,                    // specify a particular palette
                             0.f,                        // alpha threshold
                             WICBitmapPaletteTypeCustom  // or WICBitmapPaletteTypeMedianCut
                             ));
    // Optionally reinterpret the converter as a bitmap source (which it already is).
    //  com_ptr<IWICBitmapSource> bitmap_source; AS(pConverter->QueryInterface(IID_PPV_ARGS(&bitmap_source)));
    if (1) {  // verify the pixel format
      WICPixelFormatGUID pixel_format3;
      AS(converter->GetPixelFormat(&pixel_format3));
      assertx(pixel_format3 == pixel_format2);
    }
    {  // fast direct decoding into my data structure
      unsigned stride = xsize() * sizeof(Pixel);
      unsigned buffer_size = assert_narrow_cast<unsigned>(size() * sizeof(Pixel));
      // For some unknown reason, this next line fails intermittently on CONFIG=w32 for
      //  "Filterimage -assemble 2 2 root_name.{0.0,1.0,0.1,1.1}.png -diff data/lake.png -stat".
      AS(converter->CopyPixels(nullptr, stride, buffer_size, reinterpret_cast<BYTE*>(data())));
    }
    attrib().orig_filename = filename == "-" ? "-" : get_path_absolute(filename);
    attrib().orig_suffix = suffix();
  }
  if (orientation == 1) {
  } else if (orientation > 1 && orientation <= 8) {
    Warning("Adjusting for non-default image orientation");
    // https://msdn.microsoft.com/en-us/library/windows/desktop/bb760505%28v=vs.85%29.aspx
    // PHOTO_ORIENTATION_NORMAL                value = 1       text = Normal
    // PHOTO_ORIENTATION_FLIPHORIZONTAL        value = 2       text = Flip horizontal
    // PHOTO_ORIENTATION_ROTATE180             value = 3       text = Rotate 180 degrees
    // PHOTO_ORIENTATION_FLIPVERTICAL          value = 4       text = Flip vertical
    // PHOTO_ORIENTATION_TRANSPOSE             value = 5       text = Transpose
    // PHOTO_ORIENTATION_ROTATE270             value = 6       text = Rotate 270 degrees
    // PHOTO_ORIENTATION_TRANSVERSE            value = 7       text = Transverse
    // PHOTO_ORIENTATION_ROTATE90              value = 8       text = Rotate 90 degrees
    // https://beradrian.wordpress.com/2008/11/14/rotate-exif-images/
    struct S {
      int angle;
      bool fliph;
      bool flipv;
    };
    const Array<S> ar = {
        {0, false, false},  // dummy entry for value == 0
        {0, false, false},  {0, true, false},    {180, false, false}, {0, false, true},
        {270, true, false}, {270, false, false}, {90, true, false},   {90, false, false},
    };
    if (ar[orientation].angle) *this = rotate_ccw(*this, ar[orientation].angle);
    if (ar[orientation].fliph) reverse_x();
    if (ar[orientation].flipv) reverse_y();
  } else {
    Warning("Non-default image orientation not recognized");
  }
}

void Image::write_file_wic(const string& filename, bool bgra) const {
  string suf;
  // previously had: if (suffix() == "" && file_requires_pipe(filename)) suf = "bmp";
  if (suf == "") suf = to_lower(get_path_extension(filename));
  if (suf == "") suf = suffix();
  if (suf == "") throw std::runtime_error("Image '" + filename + "': no filename suffix specified for writing");
  if (contains(V<string>("avif", "heif"), suf)) {
    // WIC lacks encoding for these formats, so resort to ffmpeg.
    write_file_ffmpeg(filename, bgra);
    return;
  }
  const GUID* container_format = get_container_format(suf);
  if (!container_format && file_requires_pipe(filename)) {
    suf = "bmp";
    container_format = get_container_format(suf);
  }
  if (!container_format)
    throw std::runtime_error("Image: unrecognized suffix '" + suf + "' when writing '" + filename + "'");
  const_cast<Image&>(*this).set_suffix(suf);  // mutable
  const string& orig_filename = attrib().orig_filename;
  bool have_metadata = container_format == &GUID_ContainerFormatJpeg && orig_filename != "";
  if (have_metadata) {
    if (file_requires_pipe(orig_filename)) {
      if (attrib().orig_suffix == suffix())
        Warning("Image write: any EXIF metadata is lost because input image was read from pipe");
      have_metadata = false;
    }
  }
  wic_init();
  const bool write_through_memory =
      file_requires_pipe(filename) ||
      (have_metadata && canonical_pathname(get_path_absolute(filename)) == canonical_pathname(orig_filename));
  my_HGLOBAL hMem;  // gets defined if write_through_memory
  com_ptr<IStream> output_stream;
  if (write_through_memory) {
    // https://github.com/sumatrapdfreader/sumatrapdf/blob/master/src/utils/WinUtil.cpp
    size_t iSize = 0;
    hMem = assertx(GlobalAlloc(GMEM_MOVEABLE, iSize));
    AS(CreateStreamOnHGlobal(hMem, FALSE, &output_stream));
  } else {
    com_ptr<IWICStream> output_wic_stream;
    AS(wic_factory->CreateStream(&output_wic_stream));
    if (FAILED(output_wic_stream->InitializeFromFilename(utf16_from_utf8(filename).c_str(), GENERIC_WRITE)))
      throw std::runtime_error("Could not write image to file '" + filename + "'");
    // output_stream = output_wic_stream;  // worked but fragile
    AS(output_wic_stream->QueryInterface(IID_PPV_ARGS(&output_stream)));
  }
  assertx(output_stream);
  // CreateEncoder:
  //  https://learn.microsoft.com/en-us/windows/win32/api/wincodec/nf-wincodec-iwicimagingfactory-createencoder
  com_ptr<IWICBitmapEncoder> encoder;
  AS(wic_factory->CreateEncoder(*container_format, nullptr, &encoder));
  AS(encoder->Initialize(output_stream, WICBitmapEncoderNoCache));
#if 0
  ConsoleProgress cprogress("Iwrite");
  {
    // "MSDN is misleading on this issue, since native WIC codecs do not support this interface."
    // "This interface should be implemented on your container-level decoder class."
    com_ptr<IWICBitmapCodecProgressNotification> notification;
    AS(encoder->QueryInterface(IID_PPV_ARGS(&notification)));
    notification->RegisterProgressNotification(my_progress_callback, &cprogress, WICProgressOperationAll);
  }
#endif
  // WIC GUIDs and CLSIDs: https://learn.microsoft.com/en-us/windows/win32/wic/-wic-guids-clsids
  {
    com_ptr<IWICBitmapFrameEncode> frame_encode;
    {
      IPropertyBag2* property_bag;  // auto freed
      AS(encoder->CreateNewFrame(&frame_encode, &property_bag));
      // Encoder options: https://learn.microsoft.com/en-us/windows/win32/wic/-wic-creating-encoder
      if (container_format == &GUID_ContainerFormatJpeg) {
        int quality = getenv_int("JPG_QUALITY", 95, true);  // 0--100 (default 75)
        assertx(quality > 0 && quality <= 100);
        PROPBAG2 option;
        my_zero(option);
        option.pstrName = const_cast<wchar_t*>(L"ImageQuality");
        VARIANT variant;
        VariantInit(&variant);
        variant.vt = VT_R4;
        variant.fltVal = quality / 100.f;  // range [0., 1.]
        // The compression rate is verified to be similar to that of libjpeg for quality = 50, 95, 100.
        AS(property_bag->Write(1, &option, &variant));
        VariantClear(&variant);
      }
      AS(frame_encode->Initialize(property_bag));
    }
    com_ptr<IWICBitmapDecoder> decoder;
    com_ptr<IWICBitmapFrameDecode> frame_decode;
    com_ptr<IWICMetadataBlockReader> meta_breader;
    com_ptr<IWICMetadataBlockWriter> meta_bwriter;
    if (have_metadata) {
      const WICDecodeOptions opts = WICDecodeMetadataCacheOnDemand;
      if (!SUCCEEDED(wic_factory->CreateDecoderFromFilename(utf16_from_utf8(orig_filename).c_str(), nullptr,
                                                            GENERIC_READ, opts, &decoder))) {
        SHOW(orig_filename, filename);
        Warning("Could not decode original image metadata");
      } else {
        AS(decoder->GetFrame(0, &frame_decode));
        {
          // Some image containers may not support reading or writing metadata.
          if (0 && attrib().orig_suffix != suffix())
            Warning("Image: metadata may not carry across different containers");
          if (assertw(SUCCEEDED(frame_decode->QueryInterface(IID_PPV_ARGS(&meta_breader)))) &&
              assertw(SUCCEEDED(frame_encode->QueryInterface(IID_PPV_ARGS(&meta_bwriter))))) {
            // The following call may fail, e.g. copying a *.png to a *.jpg file.
            meta_bwriter->InitializeFromBlockReader(meta_breader);
          }
        }
        if (1) {
          // Additional metadata: make sure that orientation field is reset to normal,
          //  because we have applied any rotation to the image content itself.
          // https://learn.microsoft.com/en-us/windows/win32/wic/-wic-codec-jpegmetadataencoding
          com_ptr<IWICMetadataQueryWriter> pQueryWriter;
          if (assertw(SUCCEEDED(frame_encode->GetMetadataQueryWriter(&pQueryWriter)))) {
            PROPVARIANT propvariant;
            PropVariantInit(&propvariant);
            propvariant.vt = VT_UI2, propvariant.uiVal = 1;  // reset to default orientation
            AS(pQueryWriter->SetMetadataByName(k_orientation_flag.c_str(), &propvariant));
            PropVariantClear(&propvariant);
          }
        }
      }
    }
    AS(frame_encode->SetSize(xsize(), ysize()));
    double dpi_X = 95.986602783203125, dpi_Y = 95.986602783203125;
    AS(frame_encode->SetResolution(dpi_X, dpi_Y));
    WICPixelFormatGUID pixel_format = zsize() == 4 ? GUID_WICPixelFormat32bppBGRA : GUID_WICPixelFormat24bppBGR;
    if (zsize() == 4 && suffix() != "png" && suffix() != "tif" && suffix() != "tiff" && suffix() != "wmp") {
      Warning("Image format likely does not support alpha channel");
      if (0) pixel_format = GUID_WICPixelFormat24bppBGR;
    }
    AS(frame_encode->SetPixelFormat(&pixel_format));
    // The created "bitmap" seems to be a view on the existing data, without any copy -- nice.
    com_ptr<IWICBitmap> bitmap;
    {
      // If image lacks alpha channel, use RGBA (with non-premultiplied alpha) so that encoder can ignore
      //  the undefined alpha data, and in any case does not need to perform a division by alpha.
      WICPixelFormatGUID bitmap_pixel_format = bgra ? GUID_WICPixelFormat32bppBGRA : GUID_WICPixelFormat32bppRGBA;
      unsigned stride = xsize() * sizeof(Pixel);
      // Note: problem for huge images; no workaround using WIC.
      unsigned buffer_size = assert_narrow_cast<unsigned>(size() * sizeof(Pixel));
      uchar* buf = const_cast<Image&>(*this).data()->data();  // (not modified)
      AS(wic_factory->CreateBitmapFromMemory(xsize(), ysize(), bitmap_pixel_format, stride, buffer_size, buf,
                                             &bitmap));
    }
    AS(frame_encode->WriteSource(bitmap, nullptr));
    AS(frame_encode->Commit());
  }
  AS(encoder->Commit());
  {
    HRESULT hr = output_stream->Commit(STGC_DEFAULT);
    assertx(hr == S_OK || hr == E_NOTIMPL);
  }
  encoder.reset();  // early Release just to be safe
  if (write_through_memory) {
    WFile fi(filename);
    size_t size = assertx(GlobalSize(assertx(hMem)));
    bool success;
    void* pv = assertx(GlobalLock(hMem));
    success = !!fi().write(reinterpret_cast<char*>(pv), size);
    assertx(my_GlobalUnlock(hMem));
    // Such error may occur if filename == "-" (std::cout) is a pipe that has already been closed.
    if (!success) throw std::runtime_error("Error writing image to '" + filename + "'");
  }
}

}  // namespace hh

#endif  // defined(HH_IMAGE_HAVE_WIC)
