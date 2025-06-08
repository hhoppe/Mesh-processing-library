// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Image.h"

#include "libHh/GridPixelOp.h"  // scale_Matrix_Pixel()
#include "libHh/Parallel.h"
#include "libHh/Random.h"  // for testing
#include "libHh/Stat.h"    // for testing; HH_SSTAT()
#include "libHh/StringOp.h"
#include "libHh/Vector4.h"
#include "libHh/Vector4i.h"

namespace hh {

Image::Image(const Vec2<int>& pdims) {
  init(pdims);
  if (getenv_bool("IMAGE_SILENT_IO_PROGRESS")) _silent_io_progress = true;
}

Image& Image::operator=(Image&& image) noexcept {
  clear();
  swap(*this, image);
  return *this;
}

void Image::init(const Vec2<int>& pdims, Pixel pixel) {
  init(pdims);
  if (0) {
    fill(*this, pixel);
  } else if (0) {
    parallel_for({.cycles_per_elem = 4}, range(size()), [&](const size_t i) { flat(i) = pixel; });
  } else {
    const uint32_t upix = reinterpret_cast<uint32_t&>(pixel);
    uint32_t* p = reinterpret_cast<uint32_t*>(data());
    for (const size_t i : range(size())) p[i] = upix;
    // Generates "rep stosd" which is like 32-bit std::memset(), but no faster than fill() because memory-limited.
    // (Note that parallelism overhead would actually make this slower.)
  }
}

void Image::set_zsize(int n) {
  if (!(n == 1 || n == 3 || n == 4)) assertnever("Unexpected number of channels in image: " + SSHOW(n));
  attrib().zsize = n;
}

// *** MISC

void Image::to_bw() {
  if (zsize() == 1) return;
  assertx(zsize() >= 3);
  parallel_for_coords({.cycles_per_elem = 10}, dims(), [&](const Vec2<int>& yx) {
    Pixel& pixel = (*this)[yx];
    uint8_t value;
    if (0) {
      // equivalent to 0.3086, 0.6094, 0.0820
      value = (pixel[0] * 79 + pixel[1] * 156 + pixel[2] * 21) >> 8;
    } else {
      const float gamma = 2.2f;
      Vec3<float> af;
      for_int(z, 3) af[z] = pow(pixel[z] + 0.5f, gamma);
      float gray = af[0] * .30f + af[1] * .59f + af[2] * .11f;
      value = clamp_to_uint8(int(pow(gray, 1.f / gamma) + .5f));
    }
    fill(pixel.head<3>(), value);
    pixel[3] = 255;
  });
  set_zsize(1);
}

void Image::to_color() {
  if (zsize() >= 3) return;
  assertx(zsize() == 1);
  parallel_for_coords({.cycles_per_elem = 1}, dims(), [&](const Vec2<int>& yx) {
    Pixel& pixel = (*this)[yx];
    fill(pixel.head<3>(), pixel[0]);
  });
  set_zsize(3);
}

//----------------------------------------------------------------------------

bool filename_is_image(const string& filename) {
  static const auto& k_extensions = *new Array<string>{
      "jpg", "jpeg", "png",  "bmp", "rgb", "ppm", "pgm", "pbm",  "tif",  "tiff", "jxr",  "hdp",
      "wdp", "wmp",  "webp", "bpg", "jp2", "arw", "exr", "heic", "jfif", "ico",  "avif",
  };
  return k_extensions.index(to_lower(get_path_extension(filename))) >= 0;
}

string image_suffix_for_magic_byte(uchar c) {
  // see also video_suffix_for_magic_byte() and audio_suffix_for_magic_byte()
  // also redundant information in k_image_filetypes
  // Documentation on prefixes for various image containers:
  // *.rgb: "\001"
  // *.jpg: "\377\330\377\341I\005Exif"; *.jfif: "\377\330\377\340\000\020JFIF"
  // *.bmp: "BM"
  // *.ppm: "P6\n", "P5\r\n"
  // *.png: "\211PNG\r\n"
  // *.arw: "II*\000" (Sony alpha raw)
  // *.exr: "v/1\001\002\0\0\0channels"
  // *.heic: "\000\000\000\030ftypheic" (first byte is indistinguishable from *.mp4)
  // *.ico: "\000\000\001" (first byte is indistinguishable from *.mp4)
  // *.avif: "\000\000\000\034"
  switch (c) {
    case 1: return "rgb";    // u'\x01'
    case 255: return "jpg";  // u'\xFF'
    case 'B': return "bmp";
    case 'P': return "ppm";
    case 137: return "png";  // u'\x89'
    case 'I': return "arw";
    case 'v': return "exr";  // 118
    default: return "";
  }
}

//----------------------------------------------------------------------------
// *** Scaling

static const int g_test_scale_accuracy = getenv_int("IMAGE_TEST_SCALE_ACCURACY");  // also in MatrixOp.h

void Image::scale(const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs, const Pixel* bordervalue) {
  *this = hh::scale(*this, syx, filterbs, bordervalue, std::move(*this));
}

Image scale(const Image& image, const Vec2<float>& syx, const Vec2<FilterBnd>& filterbs, const Pixel* bordervalue,
            Image&& pnewimage) {
  Image newimage = &pnewimage == &image ? Image() : std::move(pnewimage);
  assertx(min(syx) >= 0.f);
  Vec2<int> newdims = convert<int>(convert<float>(image.dims()) * syx + .5f);
  const int nz = image.zsize();
  if (!product(newdims)) {
    Warning("scaling to zero image");
    if (0) fill(newdims, 0);
  }
  newimage.init(newdims);
  newimage.attrib() = image.attrib();
  if (newdims[0] == 0) return newimage;
  if (g_test_scale_accuracy) {
    Matrix<Vector4> matrix(image.dims());
    for_int(y, image.ysize()) for_int(x, image.xsize()) for_int(z, nz) {
      matrix[y][x][z] = Random::G.unif();  // matrix of noise
    }
    Matrix<Vector4> omatrix(matrix);
    assertx(!bordervalue);
    for_int(i, g_test_scale_accuracy) {
      matrix = scale(matrix, matrix.dims(), filterbs, implicit_cast<Vector4*>(nullptr), std::move(matrix));
    }
    assertx(same_size(omatrix, matrix));
    for_int(y, image.ysize()) for_int(x, image.xsize()) for_int(z, nz) {
      float diff = matrix[y][x][z] - omatrix[y][x][z];
      HH_SSTAT(Serr, diff);
    }
    exit(0);
  }
  scale_Matrix_Pixel(image, filterbs, bordervalue, newimage);
  return newimage;
}

// *** Conversions between YUV and RGB

void convert_Nv12_to_Image(CNv12View nv12v, MatrixView<Pixel> frame) {
  assertx(same_size(nv12v.get_Y(), frame));
  const uint8_t* buf_Y = nv12v.get_Y().data();
  const uint8_t* buf_UV = nv12v.get_UV().data()->data();
  Pixel* buf_P = frame.data();
  assertx(reinterpret_cast<uintptr_t>(buf_P) % 4 == 0);
  // Filtervideo ~/proj/videoloops/data/test/HDbrink8h.mp4 -stat   _read_video times for routines below:
  // 0.27 sec (no conversion), 1.00 sec, 0.70 sec, 0.55 sec, 0.48 sec, 0.34 sec
  const auto clamp_4 = [](int a, int b, int c, int d) {
    return ((clamp_to_uint8(a) << 0) | (clamp_to_uint8(b) << 8) | (clamp_to_uint8(c) << 16) |
            (clamp_to_uint8(d) << 24));
  };
  if (0) {
    for_int(y, frame.ysize()) {
      for_int(x, frame.xsize()) {
        frame[y][x] =
            RGB_Pixel_from_YUV(nv12v.get_Y()[y][x], nv12v.get_UV()[y / 2][x / 2][0], nv12v.get_UV()[y / 2][x / 2][1]);
      }
    }
  } else if (0) {
    for_int(y, frame.ysize()) {
      if (y % 2) buf_UV -= frame.xsize();  // reuse UV row on odd lines
      for_int(x, frame.xsize() / 2) {
        uint8_t u = buf_UV[0], v = buf_UV[1];
        buf_P[0] = RGB_Pixel_from_YUV(buf_Y[0], u, v);  // OPT:YUV1
        buf_P[1] = RGB_Pixel_from_YUV(buf_Y[1], u, v);
        buf_P += 2;
        buf_Y += 2;
        buf_UV += 2;
      }
    }
  } else if (0) {
    for_int(y, frame.ysize()) {
      if (y % 2) buf_UV -= frame.xsize();  // reuse UV row on odd lines
      for_int(x, frame.xsize() / 2) {
        int u = buf_UV[0], v = buf_UV[1];
        int r0 = -16 * 298 + 409 * v + 128 - 409 * 128;  // OPT:YUV2
        int g0 = -16 * 298 - 100 * u - 208 * v + 128 + 100 * 128 + 208 * 128;
        int b0 = -16 * 298 + 516 * u + 128 - 516 * 128;
        for_int(i, 2) {
          int yy = buf_Y[i] * 298;
          *reinterpret_cast<uint32_t*>(buf_P) = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
          buf_P += 1;
        }
        buf_Y += 2;
        buf_UV += 2;
      }
    }
  } else if (0) {
    const int rowlen = frame.xsize();
    uint32_t* pP = reinterpret_cast<uint32_t*>(buf_P);
    for_int(y, frame.ysize() / 2) {
      for_int(x, frame.xsize() / 2) {
        int u = buf_UV[0], v = buf_UV[1];
        int r0 = -16 * 298 + 409 * v + 128 - 409 * 128;  // OPT:YUV3
        int g0 = -16 * 298 - 100 * u - 208 * v + 128 + 100 * 128 + 208 * 128;
        int b0 = -16 * 298 + 516 * u + 128 - 516 * 128;
        int yy;
        yy = buf_Y[0 * rowlen + 0] * 298;
        pP[0 * rowlen + 0] = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
        yy = buf_Y[0 * rowlen + 1] * 298;
        pP[0 * rowlen + 1] = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
        yy = buf_Y[1 * rowlen + 0] * 298;
        pP[1 * rowlen + 0] = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
        yy = buf_Y[1 * rowlen + 1] * 298;
        pP[1 * rowlen + 1] = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
        buf_Y += 2;
        buf_UV += 2;
        pP += 2;
      }
      buf_Y += rowlen;
      pP += rowlen;
    }
  } else if (1) {
    const int rowlen = frame.xsize();
    for_int(y, frame.ysize() / 2) {
      for_int(x, frame.xsize() / 2) {
        int u = buf_UV[0], v = buf_UV[1];
        Vector4i vi0 = (Vector4i(-16 * 298 + 128 - 409 * 128, -16 * 298 + 128 + 100 * 128 + 208 * 128,
                                 -16 * 298 + 128 - 516 * 128, 255 * 256) +
                        Vector4i(0, -100, 516, 0) * u + Vector4i(409, -208, 0, 0) * v);
        const Vector4i yscale(298, 298, 298, 0);
        buf_P[0 * rowlen + 0] = ((vi0 + yscale * buf_Y[0 * rowlen + 0]) >> 8).pixel();  // OPT:YUV4
        buf_P[0 * rowlen + 1] = ((vi0 + yscale * buf_Y[0 * rowlen + 1]) >> 8).pixel();
        buf_P[1 * rowlen + 0] = ((vi0 + yscale * buf_Y[1 * rowlen + 0]) >> 8).pixel();
        buf_P[1 * rowlen + 1] = ((vi0 + yscale * buf_Y[1 * rowlen + 1]) >> 8).pixel();
        buf_Y += 2;
        buf_UV += 2;
        buf_P += 2;
      }
      buf_Y += rowlen;
      buf_P += rowlen;
    }
  }
}

void convert_Nv12_to_Image_BGRA(CNv12View nv12v, MatrixView<Pixel> frame) {
  assertx(same_size(nv12v.get_Y(), frame));
  const uint8_t* buf_Y = nv12v.get_Y().data();
  const uint8_t* buf_UV = nv12v.get_UV().data()->data();
  Pixel* buf_P = frame.data();
  assertx(reinterpret_cast<uintptr_t>(buf_P) % 4 == 0);
  const int rowlen = frame.xsize();
  for_int(y, frame.ysize() / 2) {
    for_int(x, frame.xsize() / 2) {
      int u = buf_UV[0], v = buf_UV[1];
      Vector4i vi0 = (Vector4i(-16 * 298 + 128 - 516 * 128, -16 * 298 + 128 + 100 * 128 + 208 * 128,
                               -16 * 298 + 128 - 409 * 128, 255 * 256) +
                      Vector4i(516, -100, 0, 0) * u + Vector4i(0, -208, 409, 0) * v);
      const Vector4i yscale(298, 298, 298, 0);
      buf_P[0 * rowlen + 0] = ((vi0 + yscale * buf_Y[0 * rowlen + 0]) >> 8).pixel();  // OPT:YUV4
      buf_P[0 * rowlen + 1] = ((vi0 + yscale * buf_Y[0 * rowlen + 1]) >> 8).pixel();
      buf_P[1 * rowlen + 0] = ((vi0 + yscale * buf_Y[1 * rowlen + 0]) >> 8).pixel();
      buf_P[1 * rowlen + 1] = ((vi0 + yscale * buf_Y[1 * rowlen + 1]) >> 8).pixel();
      buf_Y += 2;
      buf_UV += 2;
      buf_P += 2;
    }
    buf_Y += rowlen;
    buf_P += rowlen;
  }
}

void convert_Image_to_Nv12(CMatrixView<Pixel> frame, Nv12View nv12v) {
  assertx(same_size(nv12v.get_Y(), frame));
  uint8_t* __restrict buf_UV = nv12v.get_UV().data()->data();
  // assertx(reinterpret_cast<uintptr_t>(nv12v.get_Y().data()) % 4 == 0);
  // assertx(reinterpret_cast<uintptr_t>(nv12v.get_UV().data()->data()) % 4 == 0);
  // Tried other optimizations too.  Note that all the implementations take about the same elapsed time.
  if (0) {
    uint8_t* __restrict buf_Y = nv12v.get_Y().data();
    for_int(y, frame.ysize()) for_int(x, frame.xsize()) { *buf_Y++ = Y_from_RGB(frame[y][x]); }
    for_int(yb, frame.ysize() / 2) {
      int y = yb * 2;
      for_int(xb, frame.xsize() / 2) {
        int x = xb * 2;
        Pixel avg;
        for_int(z, 3) {
          int sum = 0;
          for_int(yi, 2) for_int(xi, 2) sum += frame[y + yi][x + xi][z];
          avg[z] = uint8_t((sum + 2) / 4);
        }
        *buf_UV++ = U_from_RGB(avg);
        *buf_UV++ = V_from_RGB(avg);
      }
    }
  } else if (0) {
    uint8_t* __restrict buf_Y = nv12v.get_Y().data();
    // for (const size_t i : range(frame.size())) { *buf_Y++ = Y_from_RGB(frame.flat(i)); }
    {
      const uint8_t* p = frame.data()->data();
      assertx(reinterpret_cast<uintptr_t>(p) % 4 == 0);
      const auto func_enc_Y = [](const uint8_t* pp) {
        return uint8_t(((66 * int(pp[0]) + 129 * int(pp[1]) + 25 * int(pp[2]) + 128) >> 8) + 16);
      };
      for_int(i, frame.ysize() * frame.xsize() / 4) {
        *reinterpret_cast<uint32_t*>(buf_Y) = ((func_enc_Y(p + 0) << 0) | (func_enc_Y(p + 4) << 8) |
                                               (func_enc_Y(p + 8) << 16) | (func_enc_Y(p + 12) << 24));
        buf_Y += 4;
        p += 16;
      }
    }
    for_int(yb, frame.ysize() / 2) {
      int y = yb * 2;
      for_int(xb, frame.xsize() / 2) {
        int x = xb * 2;
        Pixel avg;
        for_int(z, 3) {
          int sum = 0;
          for_int(yi, 2) for_int(xi, 2) sum += frame[y + yi][x + xi][z];
          avg[z] = uint8_t((sum + 2) / 4);
        }
        *buf_UV++ = U_from_RGB(avg);
        *buf_UV++ = V_from_RGB(avg);
      }
    }
  } else if (1) {
    for_int(y, frame.ysize() / 2) {
      const uint8_t* __restrict buf_p0 = frame[y * 2 + 0].data()->data();
      uint8_t* __restrict buf_y0 = nv12v.get_Y()[y * 2 + 0].data();
      const int hnx = frame.xsize() / 2;
      for_int(x, hnx) {
        int r00 = buf_p0[0], g00 = buf_p0[1], b00 = buf_p0[2];
        uint8_t y00 = uint8_t((66 * r00 + 129 * g00 + 25 * b00 + 128 + 16 * 256) >> 8);
        int r01 = buf_p0[4], g01 = buf_p0[5], b01 = buf_p0[6];
        r00 += r01;
        g00 += g01;
        b00 += b01;
        uint8_t y01 = uint8_t((66 * r01 + 129 * g01 + 25 * b01 + 128 + 16 * 256) >> 8);
        const uint8_t* __restrict buf_p1 = buf_p0 + hnx * 8;
        int r10 = buf_p1[0], g10 = buf_p1[1], b10 = buf_p1[2];
        r00 += r10;
        g00 += g10;
        b00 += b10;
        uint8_t y10 = uint8_t((66 * r10 + 129 * g10 + 25 * b10 + 128 + 16 * 256) >> 8);
        int r11 = buf_p1[4], g11 = buf_p1[5], b11 = buf_p1[6];
        r00 += r11;
        g00 += g11;
        b00 += b11;
        uint8_t y11 = uint8_t((66 * r11 + 129 * g11 + 25 * b11 + 128 + 16 * 256) >> 8);  // OPT:to_YUV
        buf_y0[0] = y00;
        buf_y0[1] = y01;
        buf_y0[2 * hnx + 0] = y10;
        buf_y0[2 * hnx + 1] = y11;
        // (more accurate than other implementations in this function)
        *buf_UV++ =
            uint8_t((-38 * r00 - 74 * g00 + 112 * b00 + 128 * 4 + 2 * (-38 - 74 + 112) + 128 * 1024) >> 10);  // U
        *buf_UV++ =
            uint8_t((112 * r00 - 94 * g00 - 18 * b00 + 128 * 4 + 2 * (112 - 94 - 18) + 128 * 1024) >> 10);  // V
        buf_p0 += 8;
        buf_y0 += 2;
      }
    }
  } else if (0) {
    const auto func_enc_Y = [](int r, int g, int b) { return uint8_t(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16); };
    for_int(y, frame.ysize() / 2) {
      const uint8_t* buf_p0 = frame[y * 2 + 0].data()->data();
      const uint8_t* buf_p1 = frame[y * 2 + 1].data()->data();
      uint8_t* buf_y0 = nv12v.get_Y()[y * 2 + 0].data();
      uint8_t* buf_y1 = nv12v.get_Y()[y * 2 + 1].data();
      for_int(x, frame.xsize() / 2) {
        uint8_t r00 = buf_p0[0], g00 = buf_p0[1], b00 = buf_p0[2];
        uint8_t r01 = buf_p0[4], g01 = buf_p0[5], b01 = buf_p0[6];
        uint8_t r10 = buf_p1[0], g10 = buf_p1[1], b10 = buf_p1[2];
        uint8_t r11 = buf_p1[4], g11 = buf_p1[5], b11 = buf_p1[6];
        buf_p0 += 8;
        buf_p1 += 8;
        buf_y0[0] = func_enc_Y(r00, g00, b00);
        buf_y0[1] = func_enc_Y(r01, g01, b01);
        buf_y1[0] = func_enc_Y(r10, g10, b10);
        buf_y1[1] = func_enc_Y(r11, g11, b11);
        buf_y0 += 2;
        buf_y1 += 2;
        // Pixel avg((r00 + r01 + r10 + r11 + 2) / 4, (g00 + g01 + g10 + g11 + 2) / 4,
        //           (b00 + b01 + b10 + b11 + 2) / 4);
        // buf_UV[0] = U_from_RGB(avg);
        // buf_UV[1] = V_from_RGB(avg);
        int ravg = (r00 + r01 + r10 + r11 + 2) / 4, gavg = (g00 + g01 + g10 + g11 + 2) / 4,
            bavg = (b00 + b01 + b10 + b11 + 2) / 4;
        const auto enc_U = [](int r, int g, int b) {
          return uint8_t(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128);
        };
        const auto enc_V = [](int r, int g, int b) { return uint8_t(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128); };
        buf_UV[0] = enc_U(ravg, gavg, bavg);
        buf_UV[1] = enc_V(ravg, gavg, bavg);
        buf_UV += 2;
      }
    }
  }
}

void scale(CNv12View nv12, const Vec2<FilterBnd>& filterbs, const Pixel* bordervalue, Nv12View new_nv12) {
  assertx(nv12.get_Y().data() != new_nv12.get_Y().data());
  assertx(nv12.get_UV().data() != new_nv12.get_UV().data());
  if (nv12.get_Y().dims() == new_nv12.get_Y().dims()) return;
  if (!product(new_nv12.get_Y().dims())) return;
  float borderY;
  Vector4 borderUV;
  if (bordervalue) {
    uint8_t borderYt = Y_from_RGB(*bordervalue);
    convert(CGrid1View(borderYt), Grid1View(borderY));
    Vec2<uint8_t> borderUVt = V(U_from_RGB(*bordervalue), V_from_RGB(*bordervalue));
    convert(CGrid1View(borderUVt), Grid1View(borderUV));
  }
  {
    Matrix<float> mat(nv12.get_Y().dims());
    convert(nv12.get_Y(), mat);
    Matrix<float> mat2 = scale(mat, new_nv12.get_Y().dims(), filterbs, bordervalue ? &borderY : nullptr);
    convert(mat2, new_nv12.get_Y());
  }
  {
    Matrix<Vector4> mat(nv12.get_UV().dims());
    convert(nv12.get_UV(), mat);
    Matrix<Vector4> mat2 = scale(mat, new_nv12.get_UV().dims(), filterbs, bordervalue ? &borderUV : nullptr);
    convert(mat2, new_nv12.get_UV());
  }
}

}  // namespace hh
