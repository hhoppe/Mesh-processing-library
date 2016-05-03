// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Image.h"

#include <cstring>              // strlen()

#include "Vector4.h"
#include "Vector4i.h"
#include "Random.h"             // for testing
#include "Stat.h"               // for testing
#include "Parallel.h"
#include "StringOp.h"
#include "GridPixelOp.h"        // scale_Matrix_Pixel()
#include "Audio.h"              // ffmpeg_command_exists()
#include "FileIO.h"             // RFile, WFile
#include "BinaryIO.h"           // read_binary_raw(), write_binary_raw()

namespace hh {

Image::Image(const Vec2<int>& pdims) {
    init(pdims);
    if (getenv_bool("IMAGE_SILENT_IO_PROGRESS")) _silent_io_progress = true;
}

void Image::init(const Vec2<int>& pdims, Pixel pix) {
    init(pdims);
    if (0) {
        fill(*this, pix);
    } else if (0) {
        parallel_for_size_t(i, size()) { raster(i) = pix; }
    } else {
        const uint32_t upix = reinterpret_cast<uint32_t&>(pix);
        uint32_t* p = reinterpret_cast<uint32_t*>(data());
        for_size_t(i, size()) { p[i] = upix; }
        // Generates "rep stosd" which is like 32-bit std::memset(), but no faster than fill() because memory-limited.
        // (Note that OpenMP parallel_for_size_t overhead would actually make this slower.)
    }
}

// *** MISC

void Image::to_bw() {
    if (zsize()==1) return;
    assertx(zsize()>=3);
    cond_parallel_for_int(size()*10, y, ysize()) for_int(x, xsize()) {
        Pixel& pix = (*this)[y][x];
        if (0) {
            // equivalent to 0.3086, 0.6094, 0.0820
            pix[0] = ((pix[0]*79+pix[1]*156+pix[2]*21) >> 8);
        } else {
            const float gamma = 2.2f;
            Vec3<float> af; for_int(z, 3) { af[z] = pow(pix[z]+0.5f, gamma); }
            float gray = af[0]*.30f+af[1]*.59f+af[2]*.11f;
            pix[0] = clamp_to_uchar(int(pow(gray, 1.f/gamma)+.5f));
        }
        pix[1] = pix[2] = pix[0]; pix[3] = 255;
    }
    set_zsize(1);
}

void Image::to_color() {
    if (zsize()>=3) return;
    assertx(zsize()==1);
    cond_parallel_for_int(size()*1, y, ysize()) for_int(x, xsize()) {
        Pixel& pix = (*this)[y][x];
        pix[1] = pix[2] = pix[0];
    }
    set_zsize(3);
}

// *** ffmpeg IO

void Image::read_file_FF(const string& pfilename, bool bgr) {
    string filename = pfilename;
    const bool ldebug = getenv_bool("FF_DEBUG");
    if (!ffmpeg_command_exists()) throw std::runtime_error("Cannot find ffmpeg program to read image content");
    unique_ptr<TmpFile> tmpfile;
    if (file_requires_pipe(filename)) {
        RFile fi(filename);
        int c = fi().peek();
        if (c<0) throw std::runtime_error("Error reading image from empty pipe '" + filename + "'");
        attrib().suffix = image_suffix_for_magic_byte(uchar(c));
        if (attrib().suffix=="")
            throw std::runtime_error(sform("Peeked image format (c=%d) in pipe '%s' is not recognized",
                                           c, filename.c_str()));
        tmpfile = make_unique<TmpFile>(attrib().suffix);
        filename = tmpfile->filename();
        WFile fi2(filename);
        fi2() << fi().rdbuf();  // copy the entire stream
    }
    if (!file_exists(filename)) throw std::runtime_error("Image file '" + filename + "' does not exist");
    {                           // read header for dimensions and attributes (ignore actual data)
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
        Vec2<int> dims{0, 0};
        int nimages = 0;
        string scontainer;
        bool has_alpha = false;
        int nlines = 0;
        string sline;
        while (my_getline(fi(), sline, false)) {
            nlines++;
            if (ldebug) SHOW(sline);
            if (contains(sline, "Could not find option 'nostdin'")) {
                Warning("Version of external program 'ffmpeg' may be too old");
                continue;
            }
            if (contains(sline, "Input #0, ")) nimages++;
            if (contains(sline, "Stream #0:0: Video: ")) {
                string::size_type i = sline.find(": Video: "); assertt(i!=string::npos);
                i += strlen(": Video: ");
                string::size_type j = sline.find(',', i); assertt(j!=string::npos);
                scontainer = sline.substr(i, j-i);
                i = j;
                for (;;) {
                    i = sline.find(',', i+1); if (i==string::npos) break;
                    if (sscanf(sline.c_str()+i, ", %dx%d", &dims[1], &dims[0])==2) break;
                }
                if (contains(sline, ", rgba,")) has_alpha = true;
            }
        }
        if (ldebug) SHOW(nlines, nimages, dims, scontainer, has_alpha);
        if (!nlines || nimages!=1 || !product(dims))
            throw std::runtime_error("ffmpeg is unable to read image file '" + filename + "'");
        assertt(scontainer!="");
        init(dims);
        if (scontainer=="mjpeg") scontainer = "jpg";
        if (scontainer=="sgi")   scontainer = "rgb";
        if (get_path_extension(filename)!="" && get_path_extension(filename)!=scontainer) {
            SHOW(get_path_extension(filename), scontainer);
            Warning("Image read: encoded content does not match filename suffix");
        }
        set_suffix(scontainer);
        set_zsize(has_alpha ? 4 : 3);
        // Reading exif is not possible using "ffmpeg -i input.jpg -f ffmetadata metadata.txt"
        // There is sufficient information in "ffprobe -show_frames input.jpg" but this requires ffprobe.
        // Another option is the separate tool "exifutil".
    }
    {
        string spixfmt = bgr ? "bgra" : "rgba";
        string scmd = ("ffmpeg -loglevel panic -nostdin -i " + quote_arg_for_shell(filename) +
                       " -f image2pipe -pix_fmt " + spixfmt + " -vcodec rawvideo - |");
        if (ldebug) SHOW(scmd);
        RFile fi(scmd);
        if (!read_binary_raw(fi(), array_view()))
            throw std::runtime_error("Error reading pixels from image '" + filename + "'");
    }
}

void Image::write_file_FF(const string& pfilename, bool bgr) const {
    string filename = pfilename;
    const bool ldebug = getenv_bool("FF_DEBUG");
    assertx(product(dims()));
    if (suffix()=="") const_cast<Image&>(*this).set_suffix(get_path_extension(filename)); // mutable
    if (suffix()=="")
        throw std::runtime_error("Image '" + filename + "': no filename suffix specified for writing");
    if (!ffmpeg_command_exists()) throw std::runtime_error("Cannot find ffmpeg program to write image content");
    unique_ptr<TmpFile> tmpfile;
    if (file_requires_pipe(filename)) {
        tmpfile = make_unique<TmpFile>(suffix());
        filename = tmpfile->filename();
    }
    if (attrib().exif_data.num()) Warning("Image EXIF data lost in image_write_file_FF");
    {
        string spixfmt = bgr ? "bgra" : "rgba";
        string sopixfmt = zsize()==4 ? "rgba" : "rgb24";
        string scmd = ("| ffmpeg -loglevel panic -f rawvideo -vcodec rawvideo -pix_fmt " + spixfmt +
                       sform(" -s %dx%d", xsize(), ysize()) +
                       " -i - -pix_fmt " + sopixfmt + " -y " + quote_arg_for_shell(filename));
        if (ldebug) SHOW(scmd);
        WFile fi(scmd);
        if (!write_binary_raw(fi(), array_view()))
            throw std::runtime_error("Error writing pixels to image '" + filename + "'");
    }
    if (tmpfile) {
        WFile fi(pfilename);
        RFile fi2(filename);
        fi() << fi2().rdbuf(); // copy the entire stream
    }
}


//----------------------------------------------------------------------------

bool filename_is_image(const string& filename) {
    static const Array<string> k_extensions = {
        "jpg", "jpeg", "png", "bmp", "rgb", "ppm", "pgm", "pbm", "tif", "tiff", "gif",
        "jxr", "hdp", "wdp", "wmp", "webp", "bpg", "jp2"
    };
    return k_extensions.index(to_lower(get_path_extension(filename)))>=0;
}

string image_suffix_for_magic_byte(uchar c) {
    // Documentation on prefixes for various image containers:
    // *.rgb: "\001"
    // *.jpg: "\377\330\377\341I\005Exif", "\377\330\377\340\000\020JFIF"
    // *.bmp: "BM"
    // *.ppm: "P6\n", "P5\r\n"
    // *.png: "\211PNG\r\n"
    switch (c) {
     bcase 1: return "rgb";     // u'\x01'
     bcase 255: return "jpg";   // u'\xFF'
     bcase 'B':     return "bmp";
     bcase 'P':     return "ppm";
     bcase 137: return "png";   // u'\x89'
     bdefault:      return "";
    }
}


//----------------------------------------------------------------------------
// *** Scaling

static const int g_test_scale_accuracy = getenv_int("IMAGE_TEST_SCALE_ACCURACY"); // also in MatrixOp.h

void Image::scale(const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs) {
    *this = hh::scale(*this, syx, filterbs, std::move(*this));
}

Image scale(const Image& image, const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs, Image&& pnewimage) {
    Image newimage = &pnewimage==&image ? Image() : std::move(pnewimage);
    assertx(min(syx)>=0.f);
    Vec2<int> newdims = convert<int>(convert<float>(image.dims())*syx+.5f);
    const int nz = image.zsize();
    if (!product(newdims)) {
        Warning("scaling to zero image");
        if (0) fill(newdims, 0);
    }
    newimage.init(newdims); newimage.attrib() = image.attrib();
    if (newdims[0]==0) return newimage;
    if (g_test_scale_accuracy) {
        Matrix<Vector4> matrix(image.dims());
        for_int(y, image.ysize()) for_int(x, image.xsize()) for_int(z, nz) {
            matrix[y][x][z] = Random::G.unif(); // matrix of noise
        }
        Matrix<Vector4> omatrix(matrix);
        for_int(i, g_test_scale_accuracy) {
            matrix = scale(matrix, matrix.dims(), filterbs, std::move(matrix));
        }
        assertx(same_size(omatrix, matrix));
        for_int(y, image.ysize()) for_int(x, image.xsize()) for_int(z, nz) {
            float diff = matrix[y][x][z]-omatrix[y][x][z];
            HH_SSTAT(Serr, diff);
        }
        exit(0);
    }
    scale_Matrix_Pixel(image, filterbs, newimage);
    return newimage;
}

// *** Input-Output

void Image::read_file_i(const string& filename, bool bgr) {
    {
        // Problem of reading correct JPG image orientation based on EXIF image tag:
        // e.g. vv ~/data/image/jpg/20151225_160800_exif_rotated.jpg
        // - WIC: getting orientation information seems a bit complex.
        //    Maybe: https://msdn.microsoft.com/en-us/library/windows/desktop/ee719904%28v=vs.85%29.aspx
        // - libjpeg: does not itself parse the EXIF tags.
        //    Could use libexif or http://www.sentex.net/~mwandel/jhead/ to do this.
        // - ffmpeg: its information does not show rotation.
        //    Could use separate program "exiftool" or "exif" (cygwin) or "ffprobe -show_frames" to do this.
        // Overall, it would require 3 separate efforts and more dependencies to support this.
    }
    string implementation = getenv_string("RIMAGE_IMPLEMENTATION");
    if (implementation=="") implementation = getenv_string("IMAGE_IMPLEMENTATION");
    if (implementation!="") Warning("RImage I/O implementation overriden");
    if (implementation=="WIC") {
#if defined(HH_IMAGE_HAVE_WIC)
        read_file_wic(filename, bgr); return;
#else
        assertnever("Image_WIC not enabled");
#endif
    }
    if (implementation=="IO") {
#if defined(HH_IMAGE_HAVE_IO)
        read_file_IO(filename, bgr); return;
#else
        assertnever("Image_IO not enabled");
#endif
    }
    if (implementation=="FF") {
        read_file_FF(filename, bgr); return;
    }
#if defined(HH_IMAGE_HAVE_WIC)
    read_file_wic(filename, bgr);
#elif defined(HH_IMAGE_HAVE_IO)
    read_file_IO(filename, bgr);
#else
    read_file_FF(filename, bgr);
#endif
}

void Image::write_file_i(const string& filename, bool bgr) const {
    if (filename=="-") my_setenv("NO_DIAGNOSTICS_IN_STDOUT", "1");
    string implementation = getenv_string("WIMAGE_IMPLEMENTATION");
    if (implementation=="") implementation = getenv_string("IMAGE_IMPLEMENTATION");
    if (implementation!="") Warning("WImage I/O implementation overriden");
    if (implementation=="WIC") {
#if defined(HH_IMAGE_HAVE_WIC)
        write_file_wic(filename, bgr); return;
#else
        assertnever("Image_WIC not enabled");
#endif
    }
    if (implementation=="IO") {
#if defined(HH_IMAGE_HAVE_IO)
        write_file_IO(filename, bgr); return;
#else
        assertnever("Image_IO not enabled");
#endif
    }
    if (implementation=="FF") {
        write_file_FF(filename, bgr); return;
    }
#if defined(HH_IMAGE_HAVE_WIC)
    write_file_wic(filename, bgr);
#elif defined(HH_IMAGE_HAVE_IO)
    write_file_IO(filename, bgr);
#else
    write_file_FF(filename, bgr);
#endif
}

// *** Conversions between YUV and RGB

void convert_Nv12_to_Image(CNv12View nv12v, MatrixView<Pixel> frame) {
    assertx(same_size(nv12v.get_Y(), frame));
    const uchar* bufY = nv12v.get_Y().data();
    const uchar* bufUV = nv12v.get_UV().data()->data();
    Pixel* bufP = frame.data();
    assertx(uintptr_t(bufP)%4==0);
    // Filtervideo ~/proj/videoloops/data/test/HDbrink8h.mp4 -stat   _read_video times for routines below:
    // 0.27 sec (no conversion), 1.00 sec, 0.70 sec, 0.55 sec, 0.48 sec, 0.34 sec
    auto clamp_4 = [](int a, int b, int c, int d) {
        return ((clamp_to_uchar(a)<<0) | (clamp_to_uchar(b)<<8) | (clamp_to_uchar(c)<<16) | (clamp_to_uchar(d)<<24));
    };
    if (0) {
        for_int(y, frame.ysize()) {
            for_int(x, frame.xsize()) {
                frame[y][x] = YUV_to_RGB_Pixel(nv12v.get_Y()[y][x],
                                               nv12v.get_UV()[y/2][x/2][0],
                                               nv12v.get_UV()[y/2][x/2][1]);
            }
        }
    } else if (0) {
        for_int(y, frame.ysize()) {
            if (y%2) bufUV -= frame.xsize(); // reuse UV row on odd lines
            for_int(x, frame.xsize()/2) {
                uchar u = bufUV[0], v = bufUV[1];
                bufP[0] = YUV_to_RGB_Pixel(bufY[0], u, v); // OPT:YUV1
                bufP[1] = YUV_to_RGB_Pixel(bufY[1], u, v);
                bufP += 2; bufY += 2; bufUV += 2;
            }
        }
    } else if (0) {
        for_int(y, frame.ysize()) {
            if (y%2) bufUV -= frame.xsize(); // reuse UV row on odd lines
            for_int(x, frame.xsize()/2) {
                int u = bufUV[0], v = bufUV[1];
                int r0 = -16*298         + 409*v + 128           - 409*128; // OPT:YUV2
                int g0 = -16*298 - 100*u - 208*v + 128 + 100*128 + 208*128;
                int b0 = -16*298 + 516*u         + 128 - 516*128;
                for_int(i, 2) {
                    int yy = bufY[i]*298;
                    *reinterpret_cast<uint32_t*>(bufP) = clamp_4((yy+r0)>>8, (yy+g0)>>8, (yy+b0)>>8, 255);
                    bufP += 1;
                }
                bufY += 2; bufUV += 2;
            }
        }
    } else if (0) {
        const int rowlen = frame.xsize();
        uint32_t* pP = reinterpret_cast<uint32_t*>(bufP);
        for_int(y, frame.ysize()/2) {
            for_int(x, frame.xsize()/2) {
                int u = bufUV[0], v = bufUV[1];
                int r0 = -16*298         + 409*v + 128           - 409*128; // OPT:YUV3
                int g0 = -16*298 - 100*u - 208*v + 128 + 100*128 + 208*128;
                int b0 = -16*298 + 516*u         + 128 - 516*128;
                int yy;
                yy = bufY[0*rowlen+0]*298; pP[0*rowlen+0] = clamp_4((yy+r0)>>8, (yy+g0)>>8, (yy+b0)>>8, 255);
                yy = bufY[0*rowlen+1]*298; pP[0*rowlen+1] = clamp_4((yy+r0)>>8, (yy+g0)>>8, (yy+b0)>>8, 255);
                yy = bufY[1*rowlen+0]*298; pP[1*rowlen+0] = clamp_4((yy+r0)>>8, (yy+g0)>>8, (yy+b0)>>8, 255);
                yy = bufY[1*rowlen+1]*298; pP[1*rowlen+1] = clamp_4((yy+r0)>>8, (yy+g0)>>8, (yy+b0)>>8, 255);
                bufY += 2; bufUV += 2; pP += 2;
            }
            bufY += rowlen; pP += rowlen;
        }
    } else if (1) {
        const int rowlen = frame.xsize();
        for_int(y, frame.ysize()/2) {
            for_int(x, frame.xsize()/2) {
                int u = bufUV[0], v = bufUV[1];
                Vector4i vi0 = (Vector4i(-16*298 + 128           - 409*128,
                                         -16*298 + 128 + 100*128 + 208*128,
                                         -16*298 + 128 - 516*128,
                                         255*256) +
                                Vector4i(0,   -100, 516, 0)*u +
                                Vector4i(409, -208,   0, 0)*v);
                const Vector4i yscale(298, 298, 298, 0);
                bufP[0*rowlen+0] = ((vi0 + yscale*bufY[0*rowlen+0]) >> 8).pixel(); // OPT:YUV4
                bufP[0*rowlen+1] = ((vi0 + yscale*bufY[0*rowlen+1]) >> 8).pixel();
                bufP[1*rowlen+0] = ((vi0 + yscale*bufY[1*rowlen+0]) >> 8).pixel();
                bufP[1*rowlen+1] = ((vi0 + yscale*bufY[1*rowlen+1]) >> 8).pixel();
                bufY += 2; bufUV += 2; bufP += 2;
            }
            bufY += rowlen; bufP += rowlen;
        }
    }
}

void convert_Nv12_to_Image_BGRA(CNv12View nv12v, MatrixView<Pixel> frame) {
    assertx(same_size(nv12v.get_Y(), frame));
    const uchar* bufY = nv12v.get_Y().data();
    const uchar* bufUV = nv12v.get_UV().data()->data();
    Pixel* bufP = frame.data();
    assertx(uintptr_t(bufP)%4==0);
    const int rowlen = frame.xsize();
    for_int(y, frame.ysize()/2) {
        for_int(x, frame.xsize()/2) {
            int u = bufUV[0], v = bufUV[1];
            Vector4i vi0 = (Vector4i(-16*298 + 128 - 516*128,
                                     -16*298 + 128 + 100*128 + 208*128,
                                     -16*298 + 128           - 409*128,
                                     255*256) +
                            Vector4i(516, -100,   0, 0)*u +
                            Vector4i(0,   -208, 409, 0)*v);
            const Vector4i yscale(298, 298, 298, 0);
            bufP[0*rowlen+0] = ((vi0 + yscale*bufY[0*rowlen+0]) >> 8).pixel(); // OPT:YUV4
            bufP[0*rowlen+1] = ((vi0 + yscale*bufY[0*rowlen+1]) >> 8).pixel();
            bufP[1*rowlen+0] = ((vi0 + yscale*bufY[1*rowlen+0]) >> 8).pixel();
            bufP[1*rowlen+1] = ((vi0 + yscale*bufY[1*rowlen+1]) >> 8).pixel();
            bufY += 2; bufUV += 2; bufP += 2;
        }
        bufY += rowlen; bufP += rowlen;
    }
}

void convert_Image_to_Nv12(CMatrixView<Pixel> frame, Nv12View nv12v) {
    assertx(same_size(nv12v.get_Y(), frame));
    uchar* __restrict bufY = nv12v.get_Y().data();
    uchar* __restrict bufUV = nv12v.get_UV().data()->data();
    assertx(uintptr_t(bufUV)%4==0); assertx(uintptr_t(bufY)%4==0);
    // I tried optimizing this, but all implementations take about the same elapsed time.
    if (0) {
        for_int(y, frame.ysize()) for_int(x, frame.xsize()) { *bufY++ = RGB_to_Y(frame[y][x]); }
        for_int(yb, frame.ysize()/2) {
            int y = yb*2;
            for_int(xb, frame.xsize()/2) {
                int x = xb*2;
                Pixel avg;
                for_int(z, 3) {
                    int sum = 0; for_int(yi, 2) for_int(xi, 2) { sum += frame[y+yi][x+xi][z]; }
                    avg[z] = uchar((sum+2)/4);
                }
                *bufUV++ = RGB_to_U(avg);
                *bufUV++ = RGB_to_V(avg);
            }
        }
    } else if (0) {
        // for_size_t(i, frame.size()) { *bufY++ = RGB_to_Y(frame.raster(i)); }
        {
            const uchar* p = frame.data()->data();
            assertx(uintptr_t(p)%4==0);
            auto func_enc_Y = [](const uchar* pp) {
                return uchar(((66*int(pp[0]) + 129*int(pp[1]) + 25*int(pp[2]) + 128) >> 8) + 16);
            };
            for_int(i, frame.ysize()*frame.xsize()/4) {
                *reinterpret_cast<uint32_t*>(bufY) = ((func_enc_Y(p+0)<<0) |
                                                      (func_enc_Y(p+4)<<8) |
                                                      (func_enc_Y(p+8)<<16) |
                                                      (func_enc_Y(p+12)<<24));
                bufY += 4; p += 16;
            }
        }
        for_int(yb, frame.ysize()/2) {
            int y = yb*2;
            for_int(xb, frame.xsize()/2) {
                int x = xb*2;
                Pixel avg;
                for_int(z, 3) {
                    int sum = 0; for_int(yi, 2) for_int(xi, 2) { sum += frame[y+yi][x+xi][z]; }
                    avg[z] = uchar((sum+2)/4);
                }
                *bufUV++ = RGB_to_U(avg);
                *bufUV++ = RGB_to_V(avg);
            }
        }
    } else if (1) {
        for_int(y, frame.ysize()/2) {
            const uchar* __restrict bufP0 = frame[y*2+0].data()->data();
            uchar* __restrict bufY0 = nv12v.get_Y()[y*2+0].data();
            const int hnx = frame.xsize()/2;
            for_int(x, hnx) {
                int r00 = bufP0[0], g00 = bufP0[1], b00 = bufP0[2];
                uchar y00 = uchar((66*r00 + 129*g00 + 25*b00 + 128+16*256) >> 8);
                int r01 = bufP0[4], g01 = bufP0[5], b01 = bufP0[6];
                r00 += r01; g00 += g01; b00 += b01;
                uchar y01 = uchar((66*r01 + 129*g01 + 25*b01 + 128+16*256) >> 8);
                const uchar* __restrict bufP1 = bufP0+hnx*8;
                int r10 = bufP1[0], g10 = bufP1[1], b10 = bufP1[2];
                r00 += r10; g00 += g10; b00 += b10;
                uchar y10 = uchar((66*r10 + 129*g10 + 25*b10 + 128+16*256) >> 8);
                int r11 = bufP1[4], g11 = bufP1[5], b11 = bufP1[6];
                r00 += r11; g00 += g11; b00 += b11;
                uchar y11 = uchar((66*r11 + 129*g11 + 25*b11 + 128+16*256) >> 8); // OPT:to_YUV
                bufY0[0] = y00;
                bufY0[1] = y01;
                bufY0[2*hnx+0] = y10;
                bufY0[2*hnx+1] = y11;
                // (more accurate than other implementations in this function)
                *bufUV++ = uchar((-38*r00 - 74*g00 + 112*b00 + 128*4 + 2*(-38-74+112) + 128*1024) >> 10); // U
                *bufUV++ = uchar((112*r00 - 94*g00 -  18*b00 + 128*4 + 2*(112-94-18)  + 128*1024) >> 10); // V
                bufP0 += 8;
                bufY0 += 2;
            }
        }
    } else if (0) {
        auto func_enc_Y = [](int r, int g, int b) { return uchar(((66*r + 129*g + 25*b + 128) >> 8) + 16); };
        for_int(y, frame.ysize()/2) {
            const uchar* bufP0 = frame[y*2+0].data()->data();
            const uchar* bufP1 = frame[y*2+1].data()->data();
            uchar* bufY0 = nv12v.get_Y()[y*2+0].data();
            uchar* bufY1 = nv12v.get_Y()[y*2+1].data();
            for_int(x, frame.xsize()/2) {
                uchar r00 = bufP0[0], g00 = bufP0[1], b00 = bufP0[2];
                uchar r01 = bufP0[4], g01 = bufP0[5], b01 = bufP0[6];
                uchar r10 = bufP1[0], g10 = bufP1[1], b10 = bufP1[2];
                uchar r11 = bufP1[4], g11 = bufP1[5], b11 = bufP1[6];
                bufP0 += 8; bufP1 += 8;
                bufY0[0] = func_enc_Y(r00, g00, b00);
                bufY0[1] = func_enc_Y(r01, g01, b01);
                bufY1[0] = func_enc_Y(r10, g10, b10);
                bufY1[1] = func_enc_Y(r11, g11, b11);
                bufY0 += 2; bufY1 += 2;
                // Pixel avg((r00+r01+r10+r11+2)/4, (g00+g01+g10+g11+2)/4, (b00+b01+b10+b11+2)/4);
                // bufUV[0] = RGB_to_U(avg);
                // bufUV[1] = RGB_to_V(avg);
                int ravg = (r00+r01+r10+r11+2)/4, gavg = (g00+g01+g10+g11+2)/4, bavg = (b00+b01+b10+b11+2)/4;
                auto enc_U = [](int r, int g, int b) { return uchar(((-38*r - 74*g + 112*b + 128) >> 8) + 128); };
                auto enc_V = [](int r, int g, int b) { return uchar(((112*r - 94*g -  18*b + 128) >> 8) + 128); };
                bufUV[0] = enc_U(ravg, gavg, bavg);
                bufUV[1] = enc_V(ravg, gavg, bavg);
                bufUV += 2;
            }
        }
    }
}

} // namespace hh
