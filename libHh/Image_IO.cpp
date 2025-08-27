// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Image.h"

#include <cstring>  // strlen()

#include "libHh/Audio.h"     // ffmpeg_command_exists()
#include "libHh/BinaryIO.h"  // read_binary_raw(), write_binary_raw()
#include "libHh/FileIO.h"    // RFile, WFile
#include "libHh/StringOp.h"

namespace hh {

// *** ffmpeg IO

void Image::read_file_ffmpeg(const string& pfilename, bool bgra) {
  string filename = pfilename;
  const bool ldebug = getenv_bool("FFMPEG_DEBUG");
  if (!ffmpeg_command_exists()) throw std::runtime_error("Cannot find ffmpeg program to read image content");
  std::optional<TmpFile> tmpfile;
  if (file_requires_pipe(filename)) {
    RFile fi(filename);
    int c = fi().peek();
    if (c < 0) throw std::runtime_error("Error reading image from empty pipe '" + filename + "'");
    attrib().suffix = image_suffix_for_magic_byte(uchar(c));
    if (attrib().suffix == "")
      throw std::runtime_error(
          sform("Peeked image format (c=%d) in pipe '%s' is not recognized", c, filename.c_str()));
    tmpfile.emplace(attrib().suffix, fi());
    filename = tmpfile->filename();
  }
  if (!file_exists(filename)) throw std::runtime_error("Image file '" + filename + "' does not exist");
  {  // read header for dimensions and attributes (ignore actual data)
    RFile fi("ffmpeg -nostdin -i " + quote_arg_for_shell(filename) + " -vn -an 2>&1 |");
    // Input #0, image2, from 'c:/hh/data/image/lake.png':
    //   Duration: 00:00:00.04, start: 0.000000, bitrate: N/A
    //     Stream #0:0: Video: png, rgb24, 256x256 [SAR 3779:3779 DAR 1:1], 25 tbr, 25 tbn, 25 tbc
    // Input #0, sgi_pipe, from 'c:/hh/data/image/chicken0265.rgb':
    //   Duration: N/A, bitrate: N/A
    //     Stream #0:0: Video: sgi, rgb24, 720x486, 25 tbr, 25 tbn, 25 tbc
    // Input #0, image2, from 'c:/hh/data/image/dancer_charts.png':
    //   Duration: 00:00:00.04, start: 0.000000, bitrate: N/A
    //     Stream #0:0: Video: png, rgba, 1024x1024, 25 tbr, 25 tbn, 25 tbc
    // Input #0, mov,mp4,m4a,3gp,3g2,mj2, from '/hh/desktop/guanglid.octaflat1.albedo.avif':
    //   Duration: N/A, start: 0.000000, bitrate: N/A
    //   Stream #0:0[0x1]: Video: av1 (libdav1d) (High) (av01 / 0x31307661), yuv444p(pc, smpte170m/bt709/iec61966-2-1), 8192x8192 [SAR 1:1 DAR 1:1], 1 fps, 1 tbr, 1 tbn (default)
    Vec2<int> dims{0, 0};
    int nimages = 0;
    string container;
    bool has_alpha = false;
    int nlines = 0;
    string line;
    while (my_getline(fi(), line, false)) {
      nlines++;
      if (ldebug) SHOW(line);
      if (contains(line, "Could not find option 'nostdin'")) {
        Warning("Version of external program 'ffmpeg' may be too old");
        continue;
      }
      if (contains(line, "Input #0, ")) nimages++;
      if (contains(line, "Stream #0:0") && contains(line, ": Video: ")) {
        string::size_type i = line.find(": Video: ");
        assertt(i != string::npos);
        i += strlen(": Video: ");
        string::size_type j1 = line.find(',', i), j2 = line.find(' ', i);
        string::size_type j = j1 == string::npos ? j2 : j2 == string::npos ? j1 : min(j1, j2);
        assertt(j != string::npos);
        container = line.substr(i, j - i);
        i = j;
        for (;;) {
          i = line.find(',', i + 1);
          if (i == string::npos) break;
          if (sscanf(line.c_str() + i, ", %dx%d", &dims[1], &dims[0]) == 2) break;
        }
        if (contains(line, ", rgba,")) has_alpha = true;
      }
    }
    if (ldebug) SHOW(nlines, nimages, dims, container, has_alpha);
    if (!nlines || nimages != 1 || !product(dims))
      throw std::runtime_error("ffmpeg is unable to read image file '" + filename + "'");
    assertt(container != "");
    init(dims);
    if (container == "mjpeg") container = "jpg";
    if (container == "mjpeg (Baseline)") container = "jpg";
    if (container == "sgi") container = "rgb";
    if (container == "av1") container = "avif";
    string suffix = to_lower(get_path_extension(filename));
    if (suffix != "" && suffix != container) {
      SHOW(suffix, container);
      Warning("Image read: encoded content does not match filename suffix");
    }
    set_suffix(container);
    set_zsize(has_alpha ? 4 : 3);
    // Reading exif is not possible using "ffmpeg -i input.jpg -f ffmetadata metadata.txt"
    // There is sufficient information in "ffprobe -show_frames input.jpg" but this requires ffprobe.
    // Another option is the separate tool "exifutil".
  }
  {
    string pix_fmt = bgra ? "bgra" : "rgba";
    string command = ("ffmpeg -v panic -nostdin -i " + quote_arg_for_shell(filename) + " -f image2pipe -pix_fmt " +
                      pix_fmt + " -vcodec rawvideo - |");
    if (ldebug) SHOW(command);
    RFile fi(command);
    if (!read_binary_raw(fi(), array_view()))
      throw std::runtime_error("Error reading pixels from image '" + filename + "'");
  }
}

void Image::write_file_ffmpeg(const string& pfilename, bool bgra) const {
  string filename = pfilename;
  const bool ldebug = getenv_bool("FFMPEG_DEBUG");
  assertx(product(dims()));
  if (suffix() == "") const_cast<Image&>(*this).set_suffix(to_lower(get_path_extension(filename)));  // mutable
  if (suffix() == "") throw std::runtime_error("Image '" + filename + "': no filename suffix specified for writing");
  if (!ffmpeg_command_exists()) throw std::runtime_error("Cannot find ffmpeg program to write image content");
  std::optional<TmpFile> tmpfile;
  if (file_requires_pipe(filename)) {
    tmpfile.emplace(suffix());
    filename = tmpfile->filename();
  }
  if (attrib().exif_data.num()) Warning("Image EXIF data lost in image_write_file_ffmpeg");
  if (zsize() == 4 && (suffix() == "bmp" || suffix() == "jpg"))
    Warning("Image format likely does not support alpha channel");
  string s_compression;
  if (suffix() == "jpg") {
    int quality = getenv_int("JPG_QUALITY", 95);  // 0--100 (default 75)
    assertt(quality > 0 && quality <= 100);
    // Nonlinear mapping; see
    // https://web.archive.org/web/20170405194543/http://www.ffmpeg-archive.org/How-to-get-JPEG-Quality-factor-td4672891.html
    int qscale = quality >= 95   ? 0
                 : quality >= 90 ? 1
                 : quality >= 80 ? 2
                 : quality >= 75 ? 3
                 : quality >= 65 ? 4
                 : quality >= 60 ? 5
                 : quality >= 50 ? 6
                 : quality >= 45 ? 7
                 : quality >= 40 ? 8
                 : quality >= 35 ? 9
                 : quality >= 30 ? 11
                 : quality >= 25 ? 13
                 : quality >= 20 ? 16
                 : quality >= 15 ? 21
                                 : 32;
    // Note: it appears that we cannot approach jpeg_set_quality() with quality > 93 or quality < 13.
    s_compression = sform(" -qscale:v %d", qscale);
  }
  if (suffix() == "png" && getenv("PNG_COMPRESSION_LEVEL")) {
    int level = getenv_int("PNG_COMPRESSION_LEVEL", 6);  // 0-9; 0=none
    assertt(level >= 0 && level <= 9);
    level = clamp(level * 17, 0, 100);  // default 6 should map to ffmpeg default 100
    // Note: it appears that we cannot approach the high compression ratio of
    //  png_set_compression_level(png_ptr, level) with level > 6.
    s_compression = sform(" -compression_level %d", level);
  }
  if (getenv("FFMPEG_CRF")) {  // Broadly applicable parameter (default value ~32).
    // FFMPEG_CRF=0 Filterimage ~/data/image/lake.png -to avif | wc -c  # Lossless.
    const int crf = getenv_int("FFMPEG_CRF");
    assertt(crf >= 0 && crf <= 63);
    s_compression = sform(" -crf %d", crf);
  }
  {
    string pix_fmt = bgra ? "bgra" : "rgba";
    string output_pix_fmt = zsize() == 4 ? "rgba" : "rgb24";
    string command =
        ("| ffmpeg -v panic -f rawvideo -vcodec rawvideo -pix_fmt " + pix_fmt + sform(" -s %dx%d", xsize(), ysize()) +
         " -i - -pix_fmt " + output_pix_fmt + s_compression + " -y " + quote_arg_for_shell(filename));
    if (ldebug) SHOW(command);
    WFile fi(command);
    if (!write_binary_raw(fi(), array_view()))
      throw std::runtime_error("Error writing pixels to image '" + filename + "'");
  }
  if (tmpfile) tmpfile->write_to(WFile{pfilename}());
}

// *** Input-Output

void Image::read_file_i(const string& filename, bool bgra) {
  {
    // Problem of reading correct JPG image orientation based on EXIF image tag:
    // e.g. vv ~/data/image/jpg/20151225_160800_exif_rotated.jpg
    // - WIC: getting orientation information seems a bit complex.
    //    Maybe: https://msdn.microsoft.com/en-us/library/windows/desktop/ee719904%28v=vs.85%29.aspx
    // - libjpeg: does not itself parse the EXIF tags.
    //    Could use libexif or https://www.sentex.net/~mwandel/jhead/ to do this.
    // - ffmpeg: its information does not show rotation.
    //    Could use separate program "exiftool" or "exif" (cygwin) or "ffprobe -show_frames" to do this.
    // Overall, it would require 3 separate efforts and more dependencies to support this.
  }
  string implementation = getenv_string("RIMAGE_IMPLEMENTATION");
  if (implementation == "") implementation = getenv_string("IMAGE_IMPLEMENTATION");
  if (implementation != "") Warning("RImage I/O implementation overridden");
  if (implementation == "wic") {
#if defined(HH_IMAGE_HAVE_WIC)
    read_file_wic(filename, bgra);
    return;
#else
    assertnever("Image_wic not enabled");
#endif
  }
  if (implementation == "libs") {
#if defined(HH_IMAGE_HAVE_LIBS)
    read_file_libs(filename, bgra);
    return;
#else
    assertnever("Image_libs not enabled");
#endif
  }
  if (implementation == "ffmpeg") {
    read_file_ffmpeg(filename, bgra);
    return;
  }
#if defined(HH_IMAGE_HAVE_WIC)
  read_file_wic(filename, bgra);
#elif defined(HH_IMAGE_HAVE_LIBS)
  read_file_libs(filename, bgra);
#else
  read_file_ffmpeg(filename, bgra);
#endif
}

void Image::write_file_i(const string& filename, bool bgra) const {
  if (filename == "-") my_setenv("NO_DIAGNOSTICS_IN_STDOUT", "1");
  string implementation = getenv_string("WIMAGE_IMPLEMENTATION");
  if (implementation == "") implementation = getenv_string("IMAGE_IMPLEMENTATION");
  if (implementation != "") Warning("WImage I/O implementation overridden");
  if (implementation == "wic") {
#if defined(HH_IMAGE_HAVE_WIC)
    write_file_wic(filename, bgra);
    return;
#else
    assertnever("Image_wic not enabled");
#endif
  }
  if (implementation == "libs") {
#if defined(HH_IMAGE_HAVE_LIBS)  // unlike WIC, allows writing image domains larger than 4 GiB
    write_file_libs(filename, bgra);
    return;
#else
    assertnever("Image_libs not enabled");
#endif
  }
  if (implementation == "ffmpeg") {
    write_file_ffmpeg(filename, bgra);
    return;
  }
#if defined(HH_IMAGE_HAVE_WIC)
  write_file_wic(filename, bgra);
#elif defined(HH_IMAGE_HAVE_LIBS)
  write_file_libs(filename, bgra);
#else
  write_file_ffmpeg(filename, bgra);
#endif
}

}  // namespace hh
