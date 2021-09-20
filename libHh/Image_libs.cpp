// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Image.h"  // HH_IMAGE_HAVE_LIBS

#if !defined(HH_IMAGE_HAVE_LIBS)

extern void Image_libs_dummy_function_to_avoid_linkage_warnings();
void Image_libs_dummy_function_to_avoid_linkage_warnings() {}

#else

#include <cstring>  // strlen()

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wuseless-cast"  // for (size_t) cast in two macros in jpeglib.h
#endif
extern "C" {
#include "jpeglib.h"
#include "png.h"
}

#include "libHh/Array.h"
#include "libHh/BinaryIO.h"  // read_raw(), write_raw()
#include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"
#include "libHh/NetworkOrder.h"
#include "libHh/Parallel.h"
#include "libHh/StringOp.h"  // to_lower()
using namespace hh;

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wold-style-cast"  // for (size_t) cast in two macros in jpeglib.h
#endif

HH_REFERENCE_LIB("libjpeg.lib");
HH_REFERENCE_LIB("libpng.lib");
HH_REFERENCE_LIB("libz.lib");

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic ignored "-Wstringop-overflow"  // for "for_int(z, image.zsize()) pix[z] = *p++;"
#endif

namespace hh {

namespace details {

struct ImageLibs {  // friend of Image
  static void read_rgb(Image& image, FILE* file);
  static void write_rgb(const Image& image, FILE* file);
  static void read_jpg(Image& image, FILE* file);
  static void write_jpg(const Image& image, FILE* file);
  static void read_bmp(Image& image, FILE* file);
  static void write_bmp(const Image& image, FILE* file);
  static void read_ppm(Image& image, FILE* file);
  static void write_ppm(const Image& image, FILE* file);
  static void read_png(Image& image, FILE* file);
  static void write_png(const Image& image, FILE* file);
};

struct ImageFiletype {
  const char* suffix;
  uchar magic;
  using read_type = void (*)(Image&, FILE*);
  using write_type = void (*)(const Image&, FILE*);
  read_type read_func;
  write_type write_func;
};

const Array<ImageFiletype> k_image_filetypes = {
    // Some of this information is redundant with that in image_suffix_for_magic_byte().
    {"rgb", u'\x01', ImageLibs::read_rgb, ImageLibs::write_rgb},  // also some *.bw files
    {"jpg", u'\xFF', ImageLibs::read_jpg, ImageLibs::write_jpg},  //
    {"bmp", 'B', ImageLibs::read_bmp, ImageLibs::write_bmp},      //
    {"ppm", 'P', ImageLibs::read_ppm, ImageLibs::write_ppm},      //
    {"png", u'\x89', ImageLibs::read_png, ImageLibs::write_png},  //
};

static const ImageFiletype* recognize_filetype(const string& pfilename) {
  string filename = to_lower(pfilename);
  assertx(filename != "");
  if (filename[0] == '|') return nullptr;  // pipe can take any image type
  size_t imax = 0;
  const ImageFiletype* filetype = nullptr;
  for (auto& imagefiletype : k_image_filetypes) {
    auto i = filename.rfind(string(".") + imagefiletype.suffix);  // supports rootname.bmp.gz
    if (i != string::npos && i > imax) {
      imax = i;
      filetype = &imagefiletype;
    }
  }
  return filetype;
}

// *** common

static void discard_bytes(FILE* file, int nbytes) {
  assertt(nbytes >= 0);
  if (!nbytes) return;
  Array<char> buf(nbytes);
  assertt(read_raw(file, buf));
}

inline float frac_zy(int z, int y, const Image& image) {
  return (z * image.ysize() + y) / float(image.zsize() * image.ysize());
}

// *** RGB (SGI) image

struct rgb_IMAGE {  // stuff saved on disk
  ushort imagic;
  ushort type;
  ushort dim;
  ushort xsize;
  ushort ysize;
  ushort zsize;
  uint32_t vmin;
  uint32_t vmax;
  uint32_t wastebytes;
  Vec<char, 80> name;
  uint32_t colormap;
};

constexpr int k_rgb_imagic = 0732;
constexpr int k_rgb_header_length = 512;

void ImageLibs::read_rgb(Image& image, FILE* file) {
  bool rle;
  {
    rgb_IMAGE rgbi;
    assertt(read_raw(file, ArView(rgbi)));
    for_int(i, 6) from_std((&rgbi.imagic) + i);
    for_int(i, 3) from_std((&rgbi.vmin) + i);
    assertt(rgbi.imagic == k_rgb_imagic);
    discard_bytes(file, k_rgb_header_length - sizeof(rgbi));
    assertt((rgbi.type & 0x00ff) == 1);  // one byte per component
    rle = (rgbi.type & 0xff00) != 0;
    // has (x, y, z) dimensions (!= zsize)
    assertt(rgbi.dim == 3 || (rgbi.dim == 2 && rgbi.zsize == 1));
    // assertw(!rgbi.wastebytes);
    assertt(rgbi.colormap == 0);
    image.init(V(int(rgbi.ysize), int(rgbi.xsize)));
    image.set_zsize(rgbi.zsize);
  }
  if (!product(image.dims())) return;
  if (rle) {
    int nrows = image.zsize() * image.ysize();
    Array<int32_t> rowstart(nrows);
    Array<int32_t> rowsize(nrows);
    assertt(read_raw(file, rowstart));
    for_int(i, nrows) from_std(&rowstart[i]);
    assertt(read_raw(file, rowsize));
    for_int(i, nrows) from_std(&rowsize[i]);
    if (0) {
      for_int(i, nrows) SHOW(i, rowstart[i], rowsize[i]);
    }
    // Test y-z or z-y order (ideally, sort on rowstart for all y and z).
    bool yzorder =
        image.ysize() && image.zsize() > 1 && rowstart[1 * image.ysize() + 0] < rowstart[0 * image.ysize() + 1];
    int offset = k_rgb_header_length + 2 * nrows * sizeof(rowstart[0]);
    Array<uchar> row;
    ConsoleProgress cprogress("Iread", image._silent_io_progress);
    int nlines = 0;
    int y = 0, z = 0;
    for (;;) {
      if (!image.ysize()) break;  // exception
      cprogress.update(float(nlines++) / (image.zsize() * image.ysize()));
      int nskip = rowstart[z * image.ysize() + y] - offset;
      assertw(!nskip);
      assertt(nskip >= 0);
      if (nskip) discard_bytes(file, nskip);
      row.init(rowsize[z * image.ysize() + y]);
      assertt(read_raw(file, row));
      offset = rowstart[z * image.ysize() + y] + row.num();
      int i = 0, x = 0;
      for (;;) {
        ushort pixel = row[i++];
        int count;
        count = (pixel & 0x7f);
        if (!count) break;
        if (pixel & 0x80) {
          while (count--) image[y][x++][z] = row[i++];
        } else {
          pixel = row[i++];
          while (count--) image[y][x++][z] = uchar(pixel);
        }
      }
      assertt(x == image.xsize());
      assertw(i == row.num());
      if (yzorder) {
        z++;
        if (z < image.zsize()) continue;
        z = 0;
        y++;
        if (y < image.ysize()) continue;
      } else {
        y++;
        if (y < image.ysize()) continue;
        y = 0;
        z++;
        if (z < image.zsize()) continue;
      }
      break;
    }
  } else {
    ConsoleProgress cprogress("Iread", image._silent_io_progress);
    Array<uchar> row(image.xsize());
    for_int(z, image.zsize()) {
      for_int(y, image.ysize()) {
        cprogress.update(frac_zy(z, y, image));
        assertt(read_raw(file, row));
        for_int(x, image.xsize()) image[y][x][z] = row[x];
      }
    }
  }
  if (image.zsize() == 1) {
    parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) { image[yx][2] = image[yx][1] = image[yx][0]; });
  }
  if (image.zsize() < 4) {
    for (Pixel& pix : image) pix[3] = 255;
  }
  if (1) image.reverse_y();  // because *.rgb format has image origin at lower-left
}

void ImageLibs::write_rgb(const Image& image, FILE* file) {
  ConsoleProgress cprogress("Iwrite", image._silent_io_progress);
  const bool rle = true;
  {
    rgb_IMAGE rgbi = {};
    rgbi.imagic = k_rgb_imagic;
    rgbi.type = 1;
    if (rle) rgbi.type |= 0x0100;
    rgbi.dim = 3;
    rgbi.xsize = assert_narrow_cast<ushort>(image.xsize());
    rgbi.ysize = assert_narrow_cast<ushort>(image.ysize());
    rgbi.zsize = assert_narrow_cast<ushort>(image.zsize());
    rgbi.vmin = std::numeric_limits<int>::max();
    rgbi.vmax = 0;
    rgbi.wastebytes = 0;
    const char* name = "no name";
    assertx(strlen(name) < rgbi.name.size());
    strncpy(rgbi.name.data(), name, rgbi.name.size() - 1);
    rgbi.name.last() = '\0';
    rgbi.colormap = 0;  // (NORMAL)
    for_int(y, image.ysize()) {
      cprogress.update(float(y) / image.ysize() * .333f + 0.f);
      for_int(x, image.xsize()) {
        for_int(z, image.zsize()) {
          unsigned v = image[y][x][z];
          rgbi.vmin = min(rgbi.vmin, v);
          rgbi.vmax = max(rgbi.vmax, v);
        }
      }
    }
    for_int(i, 6) to_std((&rgbi.imagic) + i);
    for_int(i, 3) to_std((&rgbi.vmin) + i);
    assertt(write_raw(file, ArView(rgbi)));
  }
  {
    Array<char> buftmp(k_rgb_header_length - sizeof(rgb_IMAGE), 0);
    assertt(write_raw(file, ArView(buftmp)));
  }
  if (!product(image.dims())) return;
  Array<uchar> row(image.xsize());
  if (rle) {
    int nrows = image.zsize() * image.ysize();
    Array<int32_t> rowstart;
    rowstart.reserve(nrows);
    Array<int32_t> rowsize;
    rowsize.reserve(nrows);
    Array<uchar> buf;
    buf.reserve(int(nrows * image.xsize() * .8f));  // rough conservative guess; will grow if insufficient
    for_int(z, image.zsize()) {
      for_int(y, image.ysize()) {
        cprogress.update(frac_zy(z, y, image) * .333f + .333f);
        rowstart.push(buf.num());
        int yy = image.ysize() - 1 - y;  // because *.rgb format has image origin at lower-left
        for_int(x, image.xsize()) row[x] = image[yy][x][z];
        for (int x = 0; x < row.num();) {
          int xs = x;
          x += 2;
          while (x < row.num() && ((row[x - 2] != row[x - 1]) || (row[x - 1] != row[x]))) x++;
          x -= 2;
          int count = x - xs;
          while (count) {
            int n = count > 126 ? 126 : count;
            count -= n;
            buf.push(narrow_cast<uchar>(0x80 | n));
            while (n--) buf.push(row[xs++]);
          }
          xs = x;
          uchar cc = row[x++];
          while (x < row.num() && row[x] == cc) x++;
          count = x - xs;
          while (count) {
            int n = count > 126 ? 126 : count;
            count -= n;
            buf.push(uchar(n));
            buf.push(cc);
          }
        }
        buf.push(0);
        rowsize.push(buf.num() - rowstart.last());
        rowstart.last() += k_rgb_header_length + 2 * nrows * sizeof(rowstart[0]);
      }
    }
    assertt(rowstart.num() == nrows && rowsize.num() == nrows);
    for (auto& v : rowstart) to_std(&v);
    for (auto& v : rowsize) to_std(&v);
    assertt(write_raw(file, rowstart));
    assertt(write_raw(file, rowsize));
    // (Note: could fail to write all if output is pipe.)
    if (0) {
      assertt(write_raw(file, buf));
    } else {
      int i = 0;
      while (i < buf.num()) {
        cprogress.update(float(i) / buf.num() * .333f + .667f);
        int ndesired = min(8192, buf.num() - i);
        assertt(write_raw(file, buf.segment(i, ndesired)));
        i += ndesired;
      }
    }
  } else {
    for_int(z, image.zsize()) {
      for_int(y, image.ysize()) {
        cprogress.update(frac_zy(z, y, image) * .667f + .333f);
        int yy = image.ysize() - 1 - y;  // because *.rgb format has image origin at lower-left
        for_int(x, image.xsize()) row[x] = image[yy][x][z];
        assertt(write_raw(file, row));
      }
    }
  }
}

// *** JPG image   (row 0 is at top of image)

static bool env_jpg_debug() {
  static const bool value = getenv_bool("JPG_DEBUG");
  return value;
}

// From jpeg-8d/install.txt:
//  You might want to tweak the RGB_xxx macros in jmorecfg.h so that the library
//  will accept or deliver color pixels in BGR sample order, not RGB; BGR order
//  is usually more convenient under Windows.  Note that this change will break
//  the sample applications cjpeg/djpeg, but the library itself works fine.

void ImageLibs::read_jpg(Image& image, FILE* file) {
#if defined(JPEG_LIBRARY_NOT_INSTALLED)
  dummy_use(image, file);
  throw std::runtime_error("Library libjpeg is not installed.  See Readme.txt file.");
#else
  // Note that it would be possible to read from an istream instead of a FILE* as described in
  //  http://stackoverflow.com/questions/6327784/how-to-use-libjpeg-to-read-a-jpeg-from-a-stdistream
  jpeg_decompress_struct cinfo;
  jpeg_error_mgr jerr;

  // Step 1: allocate and initialize JPEG decompression object:
  cinfo.err = jpeg_std_error(&jerr);
  // Intercepting warning messages ("Premature end of JPEG file") would require modifying jerr.output_message .
  // (Note that "Premature end" may appear in VideoViewer due to the prefetch reading of another image.)
  jerr.error_exit = [](j_common_ptr cinfo2) {
    char jpegLastErrorMsg[JMSG_LENGTH_MAX];
    cinfo2->err->format_message(cinfo2, jpegLastErrorMsg);
    throw std::runtime_error(string("libjpeg read error: ") + jpegLastErrorMsg);
  };
  jpeg_create_decompress(&cinfo);

  // Step 2: specify data source (eg, a file):
  jpeg_stdio_src(&cinfo, file);

  // Step 3: read file parameters with jpeg_read_header():
  jpeg_save_markers(&cinfo, JPEG_APP0 + 1, 0xFFFF);  // marker_code (APP1=Exif), length_limit (64 KiB max)
  jpeg_read_header(&cinfo, TRUE);
  // We can ignore the return value from jpeg_read_header since
  //   (a) suspension is not possible with the stdio data source, and
  //   (b) we passed TRUE to reject a tables-only JPEG file as an error.
  // See libjpeg.doc for more info.

  // Step 4: set parameters for decompression:
  // No change to any of the defaults set by jpeg_read_header().
  if (cinfo.output_components == 3) {
    assertw(cinfo.jpeg_color_space == JCS_YCbCr);
    assertw(cinfo.out_color_space == JCS_RGB);
    cinfo.out_color_space = JCS_RGB;  // just in case it is JCS_YCbCr
  }

  // Step 5: Start decompressor:
  jpeg_start_decompress(&cinfo);
  // We can ignore the return value since suspension is not possible
  // with the stdio data source.
  // We may need to do some setup of our own at this point before reading
  // the data.  After jpeg_start_decompress() we have the correct scaled
  // output image dimensions available, as well as the output colormap
  // if we asked for color quantization.
  image.init(V(int(cinfo.output_height), int(cinfo.output_width)));
  image.set_zsize(cinfo.output_components);

  // Step 6: while (scan lines remain to be read) jpeg_read_scanlines(...):
  Array<uchar> row(image.xsize() * image.zsize());
  ConsoleProgress cprogress("Iread", image._silent_io_progress);
  while (cinfo.output_scanline < cinfo.output_height) {
    cprogress.update(float(cinfo.output_scanline) / cinfo.output_height);
    int y = cinfo.output_scanline;
    // jpeg_read_scanlines expects an array of pointers to scanlines.
    // Here the array is only one element long, but you could ask for
    // more than one scanline at a time if that is more convenient.
    JSAMPROW row_pointer[1];  // pointer to JSAMPLE row[s]
    row_pointer[0] = row.data();
    assertt(jpeg_read_scanlines(&cinfo, row_pointer, 1) == 1);
    uchar* p = row.data();
    for_int(x, image.xsize()) {
      Pixel& pix = image[y][x];
      for_int(z, image.zsize()) pix[z] = *p++;
      if (image.zsize() == 1) pix[2] = pix[1] = pix[0];
      if (image.zsize() < 4) pix[3] = 255;
    }
  }

  // Step 7: Finish decompression:
  if (1) {
    for (jpeg_saved_marker_ptr marker = cinfo.marker_list; marker; marker = marker->next) {
      if (env_jpg_debug()) SHOW(int(marker->marker));
      // My camera photo has two APP1 markers, one with "Exif" and one with "http:ns.adobe.com/xap/1.0/".
      if (marker->marker == JPEG_APP0 + 1) {
        assertt(marker->data_length >= 2);
        if (marker->data[0] == 'E' && marker->data[1] == 'x') {
          if (env_jpg_debug()) SHOWL;
          image.attrib().exif_data = ArView(marker->data, marker->data_length);  // copy the data
        }
      }
    }
    if (env_jpg_debug()) {
      SHOW(image.attrib().exif_data.num());
      SHOW(convert<int>(image.attrib().exif_data.head(min(image._attrib.exif_data.num(), 2))));
    }
  }
  jpeg_finish_decompress(&cinfo);
  // We can ignore the return value since suspension is not possible with the stdio data source.

  // Step 8: Release JPEG decompression object:
  jpeg_destroy_decompress(&cinfo);
  // assertx(!fclose(infile));
  // At this point you may want to check to see whether any corrupt-data
  // warnings occurred (test whether jerr.pub.num_warnings is nonzero).
#endif  // defined(JPEG_LIBRARY_NOT_INSTALLED)
}

#if 0

struct jpeg_marker_struct {
  jpeg_saved_marker_ptr next;  // next in list, or nullptr
  UINT8 marker;                // marker code: JPEG_COM, or JPEG_APP0 + n
  unsigned original_length;    // # bytes of data in the file
  unsigned data_length;        // # bytes of data saved at data[]
  JOCTET FAR* data;            // the data contained in the marker
                               // the marker length word is not counted in data_length or original_length
};

// Control saving of COM and APPn markers into marker_list.
EXTERN(void) jpeg_save_markers JPP((j_decompress_ptr cinfo, int marker_code, unsigned length_limit));

EXTERN(void) jpeg_write_marker JPP((j_compress_ptr cinfo, int marker, const JOCTET* dataptr, unsigned datalen));

{
  // Save comments except under NONE option
  if (option != JCOPYOPT_NONE) {
    jpeg_save_markers(srcinfo, JPEG_COM, 0xFFFF);
  }
  // Save all types of APPn markers iff ALL option
  if (option == JCOPYOPT_ALL) {
    for (m = 0; m < 16; m++) jpeg_save_markers(srcinfo, JPEG_APP0 + m, 0xFFFF);
  }
  jpeg_read_header(&srcinfo, TRUE);
}

// When Exif is employed for JPEG files, the Exif data are stored in one of JPEG's defined utility
//  Application Segments, the APP1 (segment marker 0xFFE1).

// This should be called just after jpeg_start_compress() or jpeg_write_coefficients().
{
  jpeg_saved_marker_ptr marker;
  // In the current implementation, we don't actually need to examine the
  //  option flag here; we just copy everything that got saved.
  // But to avoid confusion, we do not output JFIF and Adobe APP14 markers
  //  if the encoder library already wrote one.
  for (marker = srcinfo->marker_list; marker != nullptr; marker = marker->next) {
    if (dstinfo->write_JFIF_header && marker->marker == JPEG_APP0 && marker->data_length >= 5 &&
        GETJOCTET(marker->data[0]) == 0x4A && GETJOCTET(marker->data[1]) == 0x46 &&
        GETJOCTET(marker->data[2]) == 0x49 && GETJOCTET(marker->data[3]) == 0x46 && GETJOCTET(marker->data[4]) == 0)
      continue;  // reject duplicate JFIF
    if (dstinfo->write_Adobe_marker && marker->marker == JPEG_APP0 + 14 && marker->data_length >= 5 &&
        GETJOCTET(marker->data[0]) == 0x41 && GETJOCTET(marker->data[1]) == 0x64 &&
        GETJOCTET(marker->data[2]) == 0x6F && GETJOCTET(marker->data[3]) == 0x62 && GETJOCTET(marker->data[4]) == 0x65)
      continue;  // reject duplicate Adobe
    jpeg_write_marker(dstinfo, marker->marker, marker->data, marker->data_length);
  }
}

#endif  // 0

void ImageLibs::write_jpg(const Image& image, FILE* file) {
#if defined(JPEG_LIBRARY_NOT_INSTALLED)
  dummy_use(image, file);
  throw std::runtime_error("Library libjpeg is not installed.  See Readme.txt file.");
#else
  // Note that it would be possible to write to an ostream instead of a FILE* as described in
  // https://github.com/openscenegraph/OpenSceneGraph/blob/master/src/osgPlugins/jpeg/ReaderWriterJPEG.cpp
  jpeg_compress_struct cinfo;
  jpeg_error_mgr jerr;

  // Step 1: allocate and initialize JPEG compression object:
  cinfo.err = jpeg_std_error(&jerr);
  jerr.error_exit = [](j_common_ptr cinfo2) {
    char jpegLastErrorMsg[JMSG_LENGTH_MAX];
    cinfo2->err->format_message(cinfo2, jpegLastErrorMsg);
    throw std::runtime_error(string("libjpeg write error: ") + jpegLastErrorMsg);
  };
  jpeg_create_compress(&cinfo);

  // Step 2: specify data destination (eg, a file):
  // Note: steps 2 and 3 can be done in either order.
  jpeg_stdio_dest(&cinfo, file);

  // Step 3: set parameters for compression:
  // First we supply a description of the input image.  Four fields of the cinfo struct must be filled in.
  cinfo.image_width = image.xsize();
  cinfo.image_height = image.ysize();
  cinfo.input_components = image.zsize();
  cinfo.in_color_space = (image.zsize() == 3   ? JCS_RGB
                          : image.zsize() == 1 ? JCS_GRAYSCALE
                          : image.zsize() == 4 ? JCS_UNKNOWN
                                               : (assertt(false), JCS_UNKNOWN));
  // Now use the library routine to set default compression parameters.
  // (You must set at least cinfo.in_color_space before calling this,
  // since the defaults depend on the source color space.)
  jpeg_set_defaults(&cinfo);  // quality defaults to 75
  // JFIF only supports JCS_YCbCr and JCS_GRAYSCALE, so
  //  for RGB we do automatic conversion to YCbCr (this should be the default).
  if (image.zsize() == 3) {
    assertw(cinfo.jpeg_color_space == JCS_YCbCr);
    jpeg_set_colorspace(&cinfo, JCS_YCbCr);  // this should automatically set write_JFIF_header
    assertw(cinfo.jpeg_color_space == JCS_YCbCr);
    assertw(int(cinfo.write_JFIF_header));
  }
  if (image.zsize() == 4) {
    Warning("JPEG with alpha is non-standard; color space will likely look wrong");
  }
  // Now you can set any non-default parameters you wish to.
  // Here we just illustrate the use of quality (quantization table) scaling:
  if (1) {
    int quality = getenv_int("JPG_QUALITY", 95);  // 0--100 (default 75)
    assertt(quality > 0 && quality <= 100);
    jpeg_set_quality(&cinfo, quality, TRUE);
  }

  // Step 4: Start compressor:
  // TRUE ensures that we will write a complete interchange-JPEG file.
  // Pass TRUE unless you are very sure of what you are doing.
  jpeg_start_compress(&cinfo, TRUE);
  if (image.attrib().exif_data.num()) {
    if (env_jpg_debug()) {
      SHOWL;
      SHOW(image.attrib().exif_data.num());
    }
    jpeg_write_marker(&cinfo, JPEG_APP0 + 1, image.attrib().exif_data.data(), image.attrib().exif_data.num());
  }

  // Step 5: while (scan lines remain to be written) jpeg_write_scanlines(...):
  // The standard input image format is a rectangular array of pixels, with
  // each pixel having the same number of "component" values (color channels).
  // Each pixel row is an array of JSAMPLEs (which typically are uchars).
  // If you are working with color data, then the color values for each pixel
  // must be adjacent in the row; for example, R, G, B, R, G, B, R, G, B, ... for 24-bit
  // RGB color.
  Array<uchar> row(image.xsize() * image.zsize());
  ConsoleProgress cprogress("Iwrite", image._silent_io_progress);
  while (cinfo.next_scanline < cinfo.image_height) {
    cprogress.update(float(cinfo.next_scanline) / cinfo.image_height);
    int y = cinfo.next_scanline;
    uchar* p = row.data();
    for_int(x, image.xsize()) for_int(z, image.zsize()) { *p++ = image[y][x][z]; }
    // jpeg_write_scanlines expects an array of pointers to scanlines.
    // Here the array is only one element long, but you could pass
    // more than one scanline at a time if that is more convenient.
    JSAMPROW row_pointer[1];  // pointer to JSAMPLE row[s]
    row_pointer[0] = row.data();
    assertt(jpeg_write_scanlines(&cinfo, row_pointer, 1) == 1);
  }

  // Step 6: Finish compression:
  jpeg_finish_compress(&cinfo);
  // assertx(!fclose(outfile));

  // Step 7: release JPEG compression object:
  jpeg_destroy_compress(&cinfo);
#endif  // defined(JPEG_LIBRARY_NOT_INSTALLED)
}

// *** BMP image

struct bmp_BITMAPFILEHEADER_HH {  // size 2 + 12 would be 14, but
  // if include this first entry, then sizeof() == 16 instead of 14
  // ushort bfType;
  uint32_t bfSize;
  ushort bfReserved1;
  ushort bfReserved2;
  uint32_t bfOffBits;
};
constexpr int k_size_BITMAPFILEHEADER = 14;

struct bmp_BITMAPINFOHEADER {  // size 40
  uint32_t biSize;
  int biWidth;
  int biHeight;
  ushort biPlanes;
  ushort biBitCount;
  uint32_t biCompression;
  uint32_t biSizeImage;
  int biXPelsPerMeter;
  int biYPelsPerMeter;
  uint32_t biClrUsed;
  uint32_t biClrImportant;
};

// ignoring additional fields in BITMAPV4HEADER and BITMAPV5HEADER (size > 40)

static inline bool is_gray(const Pixel& p) { return p[0] == p[1] && p[1] == p[2]; }

void ImageLibs::read_bmp(Image& image, FILE* file) {
  Vec2<char> magic;
  assertt(read_raw(file, magic.view()));
  assertt(magic[0] == 'B' && magic[1] == 'M');
  bool flip_vertical = false;
  bmp_BITMAPFILEHEADER_HH bmfh;
  assertt(read_raw(file, ArView(bmfh)));
  from_dos(&bmfh.bfSize);
  from_dos(&bmfh.bfReserved1);
  from_dos(&bmfh.bfReserved2);
  from_dos(&bmfh.bfOffBits);
  assertw(bmfh.bfReserved1 == 0 && bmfh.bfReserved2 == 0);
  assertt(bmfh.bfOffBits > 0);
  bmp_BITMAPINFOHEADER bmih;
  assertt(read_raw(file, ArView(bmih)));
  from_dos(&bmih.biSize);
  from_dos(&bmih.biWidth);
  from_dos(&bmih.biHeight);
  from_dos(&bmih.biPlanes);
  from_dos(&bmih.biBitCount);
  from_dos(&bmih.biCompression);
  from_dos(&bmih.biSizeImage);
  from_dos(&bmih.biClrUsed);
  assertt(bmih.biSize >= 40);  // ancient OS2 bmp can have size 12
  assertt(bmih.biWidth >= 0);
  if (bmih.biHeight < 0) {
    bmih.biHeight *= -1;
    flip_vertical = true;
  }
  assertt(bmih.biHeight >= 0);    // could be negative for top-down bitmap
  assertt(bmih.biPlanes == 1);    // always 1
  if (bmih.biCompression == 3) {  // BI_BITFIELDS == 3; BI_ALPHABITFIELDS == 4
    Warning("Ignoring BI_BITFIELDS color masks");
    bmih.biCompression = 0;
  } else if (bmih.biBitCount != 8) {
    assertt(bmih.biCompression == 0);  // no compression
  } else {
    assertt(bmih.biCompression <= 1);  // RLE8 on 8-bit allowed
  }
  switch (bmih.biBitCount) {
    case 24:
    case 32: assertw(!bmih.biClrUsed); break;
    case 1:
      if (!assertw(bmih.biClrUsed)) bmih.biClrUsed = 2;  // I do not remember what this is for
      assertt(bmih.biClrUsed == 2);
      break;
    case 8:
      if (!bmih.biClrUsed) bmih.biClrUsed = 256;
      break;
    default: SHOW(bmih.biBitCount); throw std::runtime_error("Such RGB image is not supported");
  }
  discard_bytes(file, bmih.biSize - sizeof(bmih));
  Array<Pixel> colormap(bmih.biClrUsed);
  bool all_gray = true;
  for_int(i, int(bmih.biClrUsed)) {
    Vec4<char> buf;
    assertt(read_raw(file, buf.view()));
    colormap[i][0] = buf[2];
    colormap[i][1] = buf[1];
    colormap[i][2] = buf[0];
    colormap[i][3] = buf[3];  // BGRA to RGBA
    if (all_gray && !is_gray(colormap[i])) all_gray = false;
  }
  if (0)
    SHOW(bmfh.bfOffBits, k_size_BITMAPFILEHEADER, bmih.biSize, bmih.biClrUsed,
         k_size_BITMAPFILEHEADER + bmih.biSize + bmih.biClrUsed * 4);
  assertt(bmfh.bfOffBits == k_size_BITMAPFILEHEADER + bmih.biSize + bmih.biClrUsed * 4);
  int ncomp = bmih.biBitCount == 32 ? 4 : 3;
  if (bmih.biBitCount <= 8 && all_gray) ncomp = 1;
  image.init(V(bmih.biHeight, bmih.biWidth));
  image.set_zsize(ncomp);
  if (!bmih.biCompression) {
    int rowsize = 4 * ((image.xsize() * bmih.biBitCount + 31) / 32);
    // while ((rowsize & 3) != 0) rowsize++;
    if (bmih.biSizeImage) {
      int expected = rowsize * image.ysize();
      int extra = int(bmih.biSizeImage) - expected;
      if (extra)
        showf("Warning: bmih.biSizeImage=%d expected=%d*%d=%d (%d extra bytes)\n", bmih.biSizeImage, rowsize,
              image.ysize(), expected, extra);
      assertt(extra >= 0);
    }
    Array<uchar> row(rowsize);
    ConsoleProgress cprogress("Iread", image._silent_io_progress);
    for_int(yi, image.ysize()) {
      cprogress.update(float(yi) / image.ysize());
      int y = !flip_vertical ? image.ysize() - 1 - yi : yi;  // because *.bmp format has image origin at lower-left
      assertt(read_raw(file, row));
      uchar* p = row.data();
      switch (bmih.biBitCount) {
        case 32:
          for_int(x, image.xsize()) {
            Pixel& pix = image[y][x];
            // convert BGRA to RGBA
            for_int(z, 3) pix[2 - z] = *p++;
            pix[3] = *p++;
          }
          break;
        case 24:
          for_int(x, image.xsize()) {
            Pixel& pix = image[y][x];
            // convert BGR to RGB
            for_int(z, 3) pix[2 - z] = *p++;
            pix[3] = 255;
          }
          break;
        case 1: {
          uchar bits8;
          dummy_init(bits8);
          for_int(x, image.xsize()) {
            if ((x & 0x7) == 0) bits8 = *p++;
            bool is_on = (bits8 >> 7) & 1;
            bits8 <<= 1;
            image[y][x] = colormap[is_on];
          }
          break;
        }
        case 8:
          for_int(x, image.xsize()) {
            int index = *p++;
            assertt(index < colormap.num());
            image[y][x] = colormap[index];
          }
          break;
        default: assertt(false);
      }
    }
    if (bmfh.bfSize) {
      unsigned size = bmfh.bfOffBits + rowsize * image.ysize();
      assertw(bmfh.bfSize == size || bmfh.bfSize == ((size + 3) / 4) * 4);
    }
  } else {
    assertt(!flip_vertical);
    assertt(bmih.biSizeImage > 0);
    int bufsize = bmih.biSizeImage;
    Array<uchar> buf(bufsize);
    assertt(read_raw(file, buf));
    int i = 0, y = 0, x = 0;
    uchar c;
    for (;;) {
      assertt(i < bufsize);
      c = buf[i++];
      if (c == 0x00) {
        assertt(i < bufsize);
        c = buf[i++];
        // showf("at %d: (0, %d)\n", i - 2, c);
        // cannot be "switch (c)" because of embedded "break"
        if (c == 0) {  // end of row
          assertw(x == image.xsize());
          x = 0;
          y++;
        } else if (c == 1) {  // end of image
          // SHOW(y, x);
          assertw(x == 0 && y == image.ysize());
          assertw(i == bufsize);
          break;
        } else if (c == 2) {  // move to (+dx, +dy)
          assertt(false);     // not implemented
        } else {              // use next c pixels literally
          int count = int(c);
          for_int(j, count) {
            assertt(i < bufsize);
            c = buf[i++];
            // showf("literal %d\n", c);
            int index = int(c);
            assertt(index < colormap.num());
            assertt(x < image.xsize() && y < image.ysize());
            image[y][x] = colormap[index];
            x++;
          }
          assertt(count % 2 == i % 2);
          if (count % 2) {  // pad to 2-byte boundary
            assertt(i < bufsize);
            c = buf[i++];
            if (0) assertw(c == 0);  // does not seem true?
          }
        }
      } else {  // repeat pixel c times
        int count = int(c);
        assertt(i < bufsize);
        c = buf[i++];
        // showf("repeat %d of pixel %d\n", count, c);
        int index = int(c);
        assertt(index < colormap.num());
        for_int(j, count) {
          if (0 && !assertw(x < image.xsize())) {
            x = 0;
            y++;
          }
          assertt(x < image.xsize() && y < image.ysize());
          image[y][x] = colormap[index];
          x++;
        }
      }
    }
    image.reverse_y();  // because *.bmp format has image origin at lower-left
  }
}

void ImageLibs::write_bmp(const Image& image, FILE* file) {
  Vec2<char> magic{'B', 'M'};
  assertt(write_raw(file, magic.view()));
  int ncomp = image.zsize();
  bmp_BITMAPFILEHEADER_HH bmfh;
  static_assert(sizeof(bmfh) == k_size_BITMAPFILEHEADER - magic.num(), "");
  int headers2size = k_size_BITMAPFILEHEADER + sizeof(bmp_BITMAPINFOHEADER);
  bmfh.bfReserved1 = 0;
  to_dos(&bmfh.bfReserved1);
  bmfh.bfReserved2 = 0;
  to_dos(&bmfh.bfReserved2);
  bmp_BITMAPINFOHEADER bmih = {};
  bmih.biSize = sizeof(bmih);
  to_dos(&bmih.biSize);
  bmih.biWidth = image.xsize();
  to_dos(&bmih.biWidth);
  bmih.biHeight = image.ysize();
  to_dos(&bmih.biHeight);
  bmih.biPlanes = 1;
  to_dos(&bmih.biPlanes);
  bmih.biBitCount = narrow_cast<ushort>(ncomp * 8);
  to_dos(&bmih.biBitCount);
  bmih.biCompression = (ncomp == 1);
  to_dos(&bmih.biCompression);
  if (ncomp == 1) {
    bmih.biClrUsed = 256;
    to_dos(&bmih.biClrUsed);
    bmfh.bfOffBits = headers2size + 256 * 4;
    to_dos(&bmfh.bfOffBits);
    // RLE encoding
    Array<uchar> buf;
    ConsoleProgress cprogress("Iwrite", image._silent_io_progress);
    for_int(y, image.ysize()) {
      cprogress.update(float(y) / image.ysize());
      int yy = image.ysize() - 1 - y;  // because *.bmp format has image origin at lower-left
      for_int(x, image.xsize()) {
        int count = 1;
        for_intL(xx, x + 1, image.xsize()) {
          if (image[yy][xx][0] != image[yy][x][0]) break;
          count++;
          if (count == 255) break;
        }
        if (count >= 2 || (image.xsize() - x) < 3) {  // repeat pixel
          buf.push(uchar(count));
          buf.push(image[yy][x][0]);
          x += count - 1;
        } else {  // sequence of literal pixels
          count = 3;
          for_intL(xx, x + 3, image.xsize()) {
            if (xx + 2 < image.xsize() && image[yy][xx + 1][0] == image[yy][xx][0] &&
                image[yy][xx + 2][0] == image[yy][xx][0])
              break;
            count++;
            if (count == 255) break;
          }
          buf.push(0);
          buf.push(uchar(count));
          for_int(i, count) buf.push(image[yy][x + i][0]);
          if (count % 2) buf.push(0);
          x += count - 1;
        }
      }
      buf.push(0), buf.push(0);  // end of row
    }
    buf.push(0), buf.push(1);  // end of image
    bmfh.bfSize = headers2size + 256 * 4 + buf.num();
    to_dos(&bmfh.bfSize);
    bmih.biSizeImage = buf.num();
    to_dos(&bmih.biSizeImage);
    assertt(write_raw(file, ArView(bmfh)));
    assertt(write_raw(file, ArView(bmih)));
    for_int(i, 256) {
      Vec4<uchar> buf2{uchar(i), uchar(i), uchar(i), uchar{255}};
      assertt(write_raw(file, buf2.view()));
    }
    assertt(write_raw(file, buf));
  } else {
    bmfh.bfOffBits = headers2size;
    to_dos(&bmfh.bfOffBits);
    int rowsize = image.xsize() * ncomp;
    while ((rowsize & 3) != 0) rowsize++;
    assertw(int64_t{rowsize} * image.ysize() < (1ull << 32));
    bmfh.bfSize = headers2size + rowsize * image.ysize();
    to_dos(&bmfh.bfSize);
    bmih.biSizeImage = rowsize * image.ysize();
    to_dos(&bmih.biSizeImage);
    assertt(write_raw(file, ArView(bmfh)));
    assertt(write_raw(file, ArView(bmih)));
    Array<uchar> row(rowsize, uchar{0});  // make sure any padding region is initialized to zero
    ConsoleProgress cprogress("Iwrite", image._silent_io_progress);
    assertt(ncomp == 3 || ncomp == 4);
    for_int(y, image.ysize()) {
      cprogress.update(float(y) / image.ysize());
      int yy = image.ysize() - 1 - y;  // because *.bmp format has image origin at lower-left
      int i = 0;
      for_int(x, image.xsize()) {
        const Pixel& pix = image[yy][x];
        // convert RGB to BGR, or RGBA to BGRA
        for_int(z, 3) row[i++] = pix[2 - z];
        if (ncomp == 4) row[i++] = pix[3];
      }
      assertt(write_raw(file, row));
    }
  }
}

// *** PPM image

void ImageLibs::read_ppm(Image& image, FILE* file) {
  Vec<char, 200> buf;
  if (!fgets(buf.data(), buf.num() - 1, file)) throw std::runtime_error("Error reading ppm image header");
  assertt(buf[0] == 'P');
  assertt(buf[1] == '6' || buf[1] == '5');
  bool is_gray = buf[1] == '5';
  // 2 == PGM text-format
  // 3 == PPM text-format
  // 5 == PGM raw-format  (8 bits/pixel)
  // 6 == PPM raw-format  (24 bits/pixel)
  for (;;) {
    assertt(fgets(buf.data(), buf.num() - 1, file));
    if (buf[0] != '#') break;
  }
  int width, height, mask;
  int numfields = sscanf(buf.data(), "%d %d %d", &width, &height, &mask);
  if (numfields == 2) {
    assertt(fgets(buf.data(), buf.num() - 1, file));
    numfields += sscanf(buf.data(), "%d", &mask);
  }
  assertt(numfields == 3);
  assertt(width >= 0 && height >= 0);
  assertw(mask == 255);
  image.init(V(height, width));
  image.set_zsize(!is_gray ? 3 : 1);
  const int rowsize = image.xsize() * image.zsize();
  Array<uchar> row(rowsize);
  ConsoleProgress cprogress("Iread", image._silent_io_progress);
  for_int(y, image.ysize()) {
    cprogress.update(float(y) / image.ysize());
    assertt(read_raw(file, row));
    uchar* p = row.data();
    for_int(x, image.xsize()) {
      Pixel& pix = image[y][x];
      for_int(z, image.zsize()) pix[z] = *p++;
      if (image.zsize() == 1) pix[2] = pix[1] = pix[0];
      if (image.zsize() < 4) pix[3] = 255;
    }
  }
}

void ImageLibs::write_ppm(const Image& image, FILE* file) {
  fprintf(file, "P6\n%d %d\n255\n", image.xsize(), image.ysize());
  if (image.zsize() == 1) Warning("Writing to ppm loses grayscale format");
  if (image.zsize() == 4) Warning("Writing to ppm loses alpha channel");
  const int ncomp = 3;
  const int rowsize = image.xsize() * ncomp;
  Array<uchar> row(rowsize);
  ConsoleProgress cprogress("Iwrite", image._silent_io_progress);
  for_int(y, image.ysize()) {
    cprogress.update(float(y) / image.ysize());
    uchar* p = row.data();
    for_int(x, image.xsize()) for_int(z, ncomp) { *p++ = image[y][x][z]; }
    assertt(write_raw(file, row));
  }
}

// *** PNG image

static void my_png_user_error_fn(png_structp png_ptr, png_const_charp error_msg) {
  dummy_use(png_ptr);
  throw std::runtime_error(string("PNG lib error: ") + error_msg);
}

static void my_png_user_warning_fn(png_structp png_ptr, png_const_charp warning_msg) {
  dummy_use(png_ptr);
  showf("PNG lib warning : %s", warning_msg);
}

void ImageLibs::read_png(Image& image, FILE* file) {
  // Note that it would be possible to read from an istream instead of a FILE* using png_set_read_fn() as
  //   described in http://www.piko3d.net/tutorials/libpng-tutorial-loading-png-files-from-streams/
  // (PNG_LIBPNG_VER_STRING, png_voidp(user_error_ptr), user_error_fn, user_warning_fn)
  png_structp png_ptr = assertt(png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr));
  png_set_error_fn(png_ptr, png_get_error_ptr(png_ptr), my_png_user_error_fn, my_png_user_warning_fn);
  png_infop info_ptr = assertt(png_create_info_struct(png_ptr));
  png_infop end_info = assertt(png_create_info_struct(png_ptr));
  png_init_io(png_ptr, file);
  // tell the library if we have already read any bytes from header
  // png_set_sig_bytes(png_ptr, 0);
  // callback to handle user chunk data
  // png_set_read_user_chunk_fn(png_ptr, user_chunk_ptr, read_chunk_callback);
  // callback used to control a progress meter
  // png_set_read_status_fn(png_ptr, read_row_callback);
  //
  if (0) {                                          // high-level read
    int png_transforms = (PNG_TRANSFORM_STRIP_16 |  // 16-bit to 8-bit
                          PNG_TRANSFORM_PACKING |   // expand 1, 2, and 4-bit
                          0);
    png_read_png(png_ptr, info_ptr, png_transforms, nullptr);
    int width = png_get_image_width(png_ptr, info_ptr);
    int height = png_get_image_height(png_ptr, info_ptr);
    int ncomp = png_get_channels(png_ptr, info_ptr);
    int bit_depth = png_get_bit_depth(png_ptr, info_ptr);
    int color_type = png_get_color_type(png_ptr, info_ptr);
    assertt(width > 0 && height > 0);
    assertt(ncomp >= 1 && ncomp <= 4);
    assertt(bit_depth == 8);
    assertt(color_type != PNG_COLOR_TYPE_PALETTE);
    image.init(V(height, width));
    image.set_zsize(ncomp);
    png_bytep* row_pointers;  // [height]
    row_pointers = png_get_rows(png_ptr, info_ptr);
    parallel_for_each(range(image.ysize()), [&](const int y) {
      uchar* buf = row_pointers[y];
      for_int(x, image.xsize()) {
        Pixel& pix = image[y][x];
        for_int(z, ncomp) pix[z] = *buf++;
        if (ncomp == 1) pix[2] = pix[1] = pix[0];
        if (ncomp < 4) pix[3] = 255;
      }
    });
  } else {  // lower-level read, directly into image.
    png_read_info(png_ptr, info_ptr);
    if (0) {
      // png_get_pHYs(png_ptr, info_ptr, &res_x, &res_y, &unit_type);
      // res_x          - pixels/unit physical resolution in x direction
      // res_y          - pixels/unit physical resolution in x direction
      // unit_type      - PNG_RESOLUTION_UNKNOWN, PNG_RESOLUTION_METER
      png_uint_32 res_x = 0, res_y = 0;
      int unit_type = 0;
      png_get_pHYs(png_ptr, info_ptr, &res_x, &res_y, &unit_type);
      SHOW(res_x, res_y, unit_type, PNG_RESOLUTION_UNKNOWN, PNG_RESOLUTION_METER);
      SHOW(png_get_x_pixels_per_meter(png_ptr, info_ptr));
      SHOW(png_get_y_pixels_per_meter(png_ptr, info_ptr));
      // common: 72dpi
      //  res_x = 2835
      //  res_y = 2835
      //  unit_type = 1
      //  PNG_RESOLUTION_UNKNOWN = 0
      //  PNG_RESOLUTION_METER = 1
      // also common  2834=~72dpi  3779=~96dpi
      // my files if undefined:
      //  res_x = 0
      //  res_y = 0
      //  unit_type = 0
      // NOTE: Word appears to read JPG as default 96dpi.
    }
    int width = png_get_image_width(png_ptr, info_ptr);
    int height = png_get_image_height(png_ptr, info_ptr);
    int ncomp = png_get_channels(png_ptr, info_ptr);
    int bit_depth = png_get_bit_depth(png_ptr, info_ptr);
    int color_type = png_get_color_type(png_ptr, info_ptr);
    assertt(width > 0 && height > 0);
    assertt(ncomp >= 1 && ncomp <= 4);
    if (bit_depth != 1 && bit_depth != 8 && bit_depth != 16) SHOW(bit_depth);
    if (bit_depth > 8) {
      assertt(bit_depth == 16);
      Warning("Reading 16-bit png image\n");
    }
    if (color_type == PNG_COLOR_TYPE_PALETTE) {
      png_set_palette_to_rgb(png_ptr);
      assertw(ncomp == 1);
      ncomp = 3;
    }
    if (0) png_set_bgr(png_ptr);  // retrieve data as BGR or BGRA
    image.init(V(height, width));
    image.set_zsize(ncomp);
    if (bit_depth == 16) png_set_strip_16(png_ptr);
    if (bit_depth < 8) png_set_packing(png_ptr);
    if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
      png_set_gray_to_rgb(png_ptr);                // always RGB
    png_set_filler(png_ptr, 0, PNG_FILLER_AFTER);  // always RGBA since Pixel expects it
    Array<png_bytep> row_pointers(image.ysize());
    for_int(y, image.ysize()) row_pointers[y] = image[y][0].data();
    png_read_image(png_ptr, row_pointers.data());
    if (bit_depth == 1) {
      Warning("correcting for bit_depth == 1");
      // apparently the "filler" expansion above expands it to 2 bytes rather than 4 bytes RGBA
      int nx = image.xsize();
      for_int(y, image.ysize()) for_int(x, nx) {
        int xx = nx - x;
        image[y][xx][0] = image[y][xx / 2][(xx % 2) * 2] ? 255 : 0;
        // image[y][x][0] = image[y][x][z] ? 255 : 0;
      }
    }
    if (image.zsize() < 4) {
      for (Pixel& pix : image) pix[3] = 255;
    }
  }
  png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
}

void ImageLibs::write_png(const Image& image, FILE* file) {
  // Note that it would be possible to write to an ostream instead of a FILE* using png_set_write_fn().
  png_structp png_ptr = assertt(png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr));
  png_set_error_fn(png_ptr, png_get_error_ptr(png_ptr), my_png_user_error_fn, my_png_user_warning_fn);
  png_infop info_ptr = assertt(png_create_info_struct(png_ptr));
  png_init_io(png_ptr, file);
  // turn off compression or set another filter
  // png_set_filter(png_ptr, 0, PNG_FILTER_NONE);
  png_set_IHDR(png_ptr, info_ptr, image.xsize(), image.ysize(), 8,
               (image.zsize() == 1   ? PNG_COLOR_TYPE_GRAY
                : image.zsize() == 3 ? PNG_COLOR_TYPE_RGB
                : image.zsize() == 4 ? PNG_COLOR_TYPE_RGB_ALPHA
                                     : (assertt(false), 0)),
               PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
  if (getenv_bool("PNG_SRGB")) {  // 20080507
    // It looks unchanged both on the screen and on the printer --> I give up on this.
    // (The intent was to make the images look less dark on the printer.)
    int srgb_intent = PNG_sRGB_INTENT_PERCEPTUAL;
    png_set_sRGB_gAMA_and_cHRM(png_ptr, info_ptr, srgb_intent);
  }
  if (1) {
    int level = getenv_int("PNG_COMPRESSION_LEVEL", 6);  //  0-9; 0=none
    assertt(level >= 0 && level <= 9);
    png_set_compression_level(png_ptr, level);
  }
  if (1) {  // 20100103 to get consistent import size into Word
    // png_set_pHYs(png_ptr, info_ptr, res_x, res_y, unit_type);
    // res_x       - pixels/unit physical resolution in x direction
    // res_y       - pixels/unit physical resolution in y direction
    // unit_type   - PNG_RESOLUTION_UNKNOWN, PNG_RESOLUTION_METER
    // 2834 pixels/meter == 71.9836 pixels/inch (dpi)
    // 2835 pixels/meter == 72.0090 pixels/inch (dpi)
    // 3779 pixels/meter == 95.9866 pixels/inch (dpi)
    png_set_pHYs(png_ptr, info_ptr, 3779, 3779, PNG_RESOLUTION_METER);
  }
  if (0) png_set_bgr(png_ptr);  // provide data as BGR or BGRA
  if (0) {                      // high-level write
    Matrix<uchar> matrix(V(image.ysize(), image.xsize() * image.zsize()));
    Array<png_bytep> row_pointers(image.ysize());
    parallel_for_each(range(image.ysize()), [&](const int y) {
      row_pointers[y] = matrix[y].data();
      uchar* buf = matrix[y].data();
      for_int(x, image.xsize()) for_int(z, image.zsize()) { *buf++ = image[y][x][z]; }
    });
    png_set_rows(png_ptr, info_ptr, row_pointers.data());
    int png_transforms = 0;
    png_write_png(png_ptr, info_ptr, png_transforms, nullptr);
  } else {  // low-level write
    png_write_info(png_ptr, info_ptr);
    // png_set_filler(png_ptr, 0, PNG_FILLER_AFTER);
    //  but no way to provide GRAY data with RGBA fill, so pack each row
    ConsoleProgress cprogress("Iwrite", image._silent_io_progress);
    Array<uchar> buffer(image.xsize() * image.zsize());
    for_int(y, image.ysize()) {
      cprogress.update(float(y) / image.ysize());
      uchar* buf = buffer.data();
      for_int(x, image.xsize()) for_int(z, image.zsize()) { *buf++ = image[y][x][z]; }
      png_bytep row_pointer = buffer.data();
      png_write_row(png_ptr, row_pointer);
    }
  }
  png_write_end(png_ptr, nullptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);
}

}  // namespace details

using namespace details;

void Image::read_file_libs(const string& filename, bool bgra) {
  RFile fi(filename);
  FILE* file = fi.cfile();
  int c = getc(file);
  if (c < 0) throw std::runtime_error("empty image file '" + filename + "'");
  assertt(c >= 0 && c <= 255);
  ungetc(c, file);
  const ImageFiletype* filetype = nullptr;
  for (auto& imagefiletype : k_image_filetypes) {
    if (c == imagefiletype.magic) {
      filetype = &imagefiletype;
      break;
    }
  }
  if (!filetype) throw std::runtime_error(sform("Unknown image format (%d) in file '%s'", c, filename.c_str()));
  set_suffix(filetype->suffix);
  const ImageFiletype* filetype2 = recognize_filetype(filename);
  if (filetype2 && filetype2 != filetype) {
    SHOW(filetype->suffix, filetype2->suffix);
    Warning("Image read: encoded content does not match filename suffix");
  }
  filetype->read_func(*this, file);
  if (bgra && zsize() >= 3) convert_rgba_bgra(*this);
}

void Image::write_file_libs(const string& filename, bool bgra) const {
  if (const ImageFiletype* filetype = recognize_filetype(filename)) {
    const_cast<Image&>(*this).set_suffix(filetype->suffix);  // mutable
  }
  if (suffix() == "") throw std::runtime_error("Image '" + filename + "': no filename suffix specified for writing");
  WFile fi(filename);
  FILE* file = fi.cfile();
  const ImageFiletype* filetype = nullptr;
  for (auto& imagefiletype : k_image_filetypes) {
    if (suffix() == imagefiletype.suffix) {
      filetype = &imagefiletype;
      break;
    }
  }
  if (!filetype)
    throw std::runtime_error("Image: unrecognized suffix '" + suffix() + "' when writing '" + filename + "'");
  if (bgra && zsize() >= 3) {
    Image timage(*this);  // make a copy because write_func() below could throw exception
    convert_rgba_bgra(timage);
    filetype->write_func(timage, file);
  } else {
    filetype->write_func(*this, file);
  }
}

}  // namespace hh

#endif  // defined(HH_IMAGE_HAVE_LIBS)
