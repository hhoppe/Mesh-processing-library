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

void Image::init(const Vec2<int>& pdims, Pixel pix) {
  init(pdims);
  if (0) {
    fill(*this, pix);
  } else if (0) {
    parallel_for_each(
        range(size()), [&](const size_t i) { flat(i) = pix; }, 4);
  } else {
    const uint32_t upix = reinterpret_cast<uint32_t&>(pix);
    uint32_t* p = reinterpret_cast<uint32_t*>(data());
    for_size_t(i, size()) p[i] = upix;
    // Generates "rep stosd" which is like 32-bit std::memset(), but no faster than fill() because memory-limited.
    // (Note that parallelism overhead would actually make this slower.)
  }
}

void Image::set_zsize(int n) {
  if (!(n == 1 || n == 3 || n == 4)) {
    SHOW(n);
    assertnever("Unexpected number of channels in image");
  }
  attrib().zsize = n;
}

// *** MISC

void Image::to_bw() {
  if (zsize() == 1) return;
  assertx(zsize() >= 3);
  parallel_for_coords(
      dims(),
      [&](const Vec2<int>& yx) {
        Pixel& pix = (*this)[yx];
        if (0) {
          // equivalent to 0.3086, 0.6094, 0.0820
          pix[0] = ((pix[0] * 79 + pix[1] * 156 + pix[2] * 21) >> 8);
        } else {
          const float gamma = 2.2f;
          Vec3<float> af;
          for_int(z, 3) af[z] = pow(pix[z] + 0.5f, gamma);
          float gray = af[0] * .30f + af[1] * .59f + af[2] * .11f;
          pix[0] = clamp_to_uint8(int(pow(gray, 1.f / gamma) + .5f));
        }
        pix[1] = pix[2] = pix[0];
        pix[3] = 255;
      },
      10);
  set_zsize(1);
}

void Image::to_color() {
  if (zsize() >= 3) return;
  assertx(zsize() == 1);
  parallel_for_coords(
      dims(),
      [&](const Vec2<int>& yx) {
        Pixel& pix = (*this)[yx];
        pix[1] = pix[2] = pix[0];
      },
      1);
  set_zsize(3);
}

//----------------------------------------------------------------------------

bool filename_is_image(const string& filename) {
  static const auto& k_extensions = *new Array<string>{
      "jpg", "jpeg", "png", "bmp", "rgb",  "ppm", "pgm", "pbm", "tif", "tiff",
      "jxr", "hdp",  "wdp", "wmp", "webp", "bpg", "jp2", "arw", "exr", "heic", "jfif",
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
  const uint8_t* bufY = nv12v.get_Y().data();
  const uint8_t* bufUV = nv12v.get_UV().data()->data();
  Pixel* bufP = frame.data();
  assertx(reinterpret_cast<uintptr_t>(bufP) % 4 == 0);
  // Filtervideo ~/proj/videoloops/data/test/HDbrink8h.mp4 -stat   _read_video times for routines below:
  // 0.27 sec (no conversion), 1.00 sec, 0.70 sec, 0.55 sec, 0.48 sec, 0.34 sec
  auto clamp_4 = [](int a, int b, int c, int d) {
    return ((clamp_to_uint8(a) << 0) | (clamp_to_uint8(b) << 8) | (clamp_to_uint8(c) << 16) |
            (clamp_to_uint8(d) << 24));
  };
  if (0) {
    for_int(y, frame.ysize()) {
      for_int(x, frame.xsize()) {
        frame[y][x] =
            YUV_to_RGB_Pixel(nv12v.get_Y()[y][x], nv12v.get_UV()[y / 2][x / 2][0], nv12v.get_UV()[y / 2][x / 2][1]);
      }
    }
  } else if (0) {
    for_int(y, frame.ysize()) {
      if (y % 2) bufUV -= frame.xsize();  // reuse UV row on odd lines
      for_int(x, frame.xsize() / 2) {
        uint8_t u = bufUV[0], v = bufUV[1];
        bufP[0] = YUV_to_RGB_Pixel(bufY[0], u, v);  // OPT:YUV1
        bufP[1] = YUV_to_RGB_Pixel(bufY[1], u, v);
        bufP += 2;
        bufY += 2;
        bufUV += 2;
      }
    }
  } else if (0) {
    for_int(y, frame.ysize()) {
      if (y % 2) bufUV -= frame.xsize();  // reuse UV row on odd lines
      for_int(x, frame.xsize() / 2) {
        int u = bufUV[0], v = bufUV[1];
        int r0 = -16 * 298 + 409 * v + 128 - 409 * 128;  // OPT:YUV2
        int g0 = -16 * 298 - 100 * u - 208 * v + 128 + 100 * 128 + 208 * 128;
        int b0 = -16 * 298 + 516 * u + 128 - 516 * 128;
        for_int(i, 2) {
          int yy = bufY[i] * 298;
          *reinterpret_cast<uint32_t*>(bufP) = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
          bufP += 1;
        }
        bufY += 2;
        bufUV += 2;
      }
    }
  } else if (0) {
    const int rowlen = frame.xsize();
    uint32_t* pP = reinterpret_cast<uint32_t*>(bufP);
    for_int(y, frame.ysize() / 2) {
      for_int(x, frame.xsize() / 2) {
        int u = bufUV[0], v = bufUV[1];
        int r0 = -16 * 298 + 409 * v + 128 - 409 * 128;  // OPT:YUV3
        int g0 = -16 * 298 - 100 * u - 208 * v + 128 + 100 * 128 + 208 * 128;
        int b0 = -16 * 298 + 516 * u + 128 - 516 * 128;
        int yy;
        yy = bufY[0 * rowlen + 0] * 298;
        pP[0 * rowlen + 0] = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
        yy = bufY[0 * rowlen + 1] * 298;
        pP[0 * rowlen + 1] = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
        yy = bufY[1 * rowlen + 0] * 298;
        pP[1 * rowlen + 0] = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
        yy = bufY[1 * rowlen + 1] * 298;
        pP[1 * rowlen + 1] = clamp_4((yy + r0) >> 8, (yy + g0) >> 8, (yy + b0) >> 8, 255);
        bufY += 2;
        bufUV += 2;
        pP += 2;
      }
      bufY += rowlen;
      pP += rowlen;
    }
  } else if (1) {
    const int rowlen = frame.xsize();
    for_int(y, frame.ysize() / 2) {
      for_int(x, frame.xsize() / 2) {
        int u = bufUV[0], v = bufUV[1];
        Vector4i vi0 = (Vector4i(-16 * 298 + 128 - 409 * 128, -16 * 298 + 128 + 100 * 128 + 208 * 128,
                                 -16 * 298 + 128 - 516 * 128, 255 * 256) +
                        Vector4i(0, -100, 516, 0) * u + Vector4i(409, -208, 0, 0) * v);
        const Vector4i yscale(298, 298, 298, 0);
        bufP[0 * rowlen + 0] = ((vi0 + yscale * bufY[0 * rowlen + 0]) >> 8).pixel();  // OPT:YUV4
        bufP[0 * rowlen + 1] = ((vi0 + yscale * bufY[0 * rowlen + 1]) >> 8).pixel();
        bufP[1 * rowlen + 0] = ((vi0 + yscale * bufY[1 * rowlen + 0]) >> 8).pixel();
        bufP[1 * rowlen + 1] = ((vi0 + yscale * bufY[1 * rowlen + 1]) >> 8).pixel();
        bufY += 2;
        bufUV += 2;
        bufP += 2;
      }
      bufY += rowlen;
      bufP += rowlen;
    }
  }
}

void convert_Nv12_to_Image_BGRA(CNv12View nv12v, MatrixView<Pixel> frame) {
  assertx(same_size(nv12v.get_Y(), frame));
  const uint8_t* bufY = nv12v.get_Y().data();
  const uint8_t* bufUV = nv12v.get_UV().data()->data();
  Pixel* bufP = frame.data();
  assertx(reinterpret_cast<uintptr_t>(bufP) % 4 == 0);
  const int rowlen = frame.xsize();
  for_int(y, frame.ysize() / 2) {
    for_int(x, frame.xsize() / 2) {
      int u = bufUV[0], v = bufUV[1];
      Vector4i vi0 = (Vector4i(-16 * 298 + 128 - 516 * 128, -16 * 298 + 128 + 100 * 128 + 208 * 128,
                               -16 * 298 + 128 - 409 * 128, 255 * 256) +
                      Vector4i(516, -100, 0, 0) * u + Vector4i(0, -208, 409, 0) * v);
      const Vector4i yscale(298, 298, 298, 0);
      bufP[0 * rowlen + 0] = ((vi0 + yscale * bufY[0 * rowlen + 0]) >> 8).pixel();  // OPT:YUV4
      bufP[0 * rowlen + 1] = ((vi0 + yscale * bufY[0 * rowlen + 1]) >> 8).pixel();
      bufP[1 * rowlen + 0] = ((vi0 + yscale * bufY[1 * rowlen + 0]) >> 8).pixel();
      bufP[1 * rowlen + 1] = ((vi0 + yscale * bufY[1 * rowlen + 1]) >> 8).pixel();
      bufY += 2;
      bufUV += 2;
      bufP += 2;
    }
    bufY += rowlen;
    bufP += rowlen;
  }
}

void convert_Image_to_Nv12(CMatrixView<Pixel> frame, Nv12View nv12v) {
  assertx(same_size(nv12v.get_Y(), frame));
  uint8_t* __restrict bufUV = nv12v.get_UV().data()->data();
  // assertx(reinterpret_cast<uintptr_t>(nv12v.get_Y().data()) % 4 == 0);
  // assertx(reinterpret_cast<uintptr_t>(nv12v.get_UV().data()->data()) % 4 == 0);
  // I tried optimizing this, but all implementations take about the same elapsed time.
  if (0) {
    uint8_t* __restrict bufY = nv12v.get_Y().data();
    for_int(y, frame.ysize()) for_int(x, frame.xsize()) { *bufY++ = RGB_to_Y(frame[y][x]); }
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
        *bufUV++ = RGB_to_U(avg);
        *bufUV++ = RGB_to_V(avg);
      }
    }
  } else if (0) {
    uint8_t* __restrict bufY = nv12v.get_Y().data();
    // for_size_t(i, frame.size()) { *bufY++ = RGB_to_Y(frame.flat(i)); }
    {
      const uint8_t* p = frame.data()->data();
      assertx(reinterpret_cast<uintptr_t>(p) % 4 == 0);
      auto func_enc_Y = [](const uint8_t* pp) {
        return uint8_t(((66 * int(pp[0]) + 129 * int(pp[1]) + 25 * int(pp[2]) + 128) >> 8) + 16);
      };
      for_int(i, frame.ysize() * frame.xsize() / 4) {
        *reinterpret_cast<uint32_t*>(bufY) = ((func_enc_Y(p + 0) << 0) | (func_enc_Y(p + 4) << 8) |
                                              (func_enc_Y(p + 8) << 16) | (func_enc_Y(p + 12) << 24));
        bufY += 4;
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
        *bufUV++ = RGB_to_U(avg);
        *bufUV++ = RGB_to_V(avg);
      }
    }
  } else if (1) {
    for_int(y, frame.ysize() / 2) {
      const uint8_t* __restrict bufP0 = frame[y * 2 + 0].data()->data();
      uint8_t* __restrict bufY0 = nv12v.get_Y()[y * 2 + 0].data();
      const int hnx = frame.xsize() / 2;
      for_int(x, hnx) {
        int r00 = bufP0[0], g00 = bufP0[1], b00 = bufP0[2];
        uint8_t y00 = uint8_t((66 * r00 + 129 * g00 + 25 * b00 + 128 + 16 * 256) >> 8);
        int r01 = bufP0[4], g01 = bufP0[5], b01 = bufP0[6];
        r00 += r01;
        g00 += g01;
        b00 += b01;
        uint8_t y01 = uint8_t((66 * r01 + 129 * g01 + 25 * b01 + 128 + 16 * 256) >> 8);
        const uint8_t* __restrict bufP1 = bufP0 + hnx * 8;
        int r10 = bufP1[0], g10 = bufP1[1], b10 = bufP1[2];
        r00 += r10;
        g00 += g10;
        b00 += b10;
        uint8_t y10 = uint8_t((66 * r10 + 129 * g10 + 25 * b10 + 128 + 16 * 256) >> 8);
        int r11 = bufP1[4], g11 = bufP1[5], b11 = bufP1[6];
        r00 += r11;
        g00 += g11;
        b00 += b11;
        uint8_t y11 = uint8_t((66 * r11 + 129 * g11 + 25 * b11 + 128 + 16 * 256) >> 8);  // OPT:to_YUV
        bufY0[0] = y00;
        bufY0[1] = y01;
        bufY0[2 * hnx + 0] = y10;
        bufY0[2 * hnx + 1] = y11;
        // (more accurate than other implementations in this function)
        *bufUV++ =
            uint8_t((-38 * r00 - 74 * g00 + 112 * b00 + 128 * 4 + 2 * (-38 - 74 + 112) + 128 * 1024) >> 10);       // U
        *bufUV++ = uint8_t((112 * r00 - 94 * g00 - 18 * b00 + 128 * 4 + 2 * (112 - 94 - 18) + 128 * 1024) >> 10);  // V
        bufP0 += 8;
        bufY0 += 2;
      }
    }
  } else if (0) {
    auto func_enc_Y = [](int r, int g, int b) { return uint8_t(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16); };
    for_int(y, frame.ysize() / 2) {
      const uint8_t* bufP0 = frame[y * 2 + 0].data()->data();
      const uint8_t* bufP1 = frame[y * 2 + 1].data()->data();
      uint8_t* bufY0 = nv12v.get_Y()[y * 2 + 0].data();
      uint8_t* bufY1 = nv12v.get_Y()[y * 2 + 1].data();
      for_int(x, frame.xsize() / 2) {
        uint8_t r00 = bufP0[0], g00 = bufP0[1], b00 = bufP0[2];
        uint8_t r01 = bufP0[4], g01 = bufP0[5], b01 = bufP0[6];
        uint8_t r10 = bufP1[0], g10 = bufP1[1], b10 = bufP1[2];
        uint8_t r11 = bufP1[4], g11 = bufP1[5], b11 = bufP1[6];
        bufP0 += 8;
        bufP1 += 8;
        bufY0[0] = func_enc_Y(r00, g00, b00);
        bufY0[1] = func_enc_Y(r01, g01, b01);
        bufY1[0] = func_enc_Y(r10, g10, b10);
        bufY1[1] = func_enc_Y(r11, g11, b11);
        bufY0 += 2;
        bufY1 += 2;
        // Pixel avg((r00 + r01 + r10 + r11 + 2) / 4, (g00 + g01 + g10 + g11 + 2) / 4,
        //           (b00 + b01 + b10 + b11 + 2) / 4);
        // bufUV[0] = RGB_to_U(avg);
        // bufUV[1] = RGB_to_V(avg);
        int ravg = (r00 + r01 + r10 + r11 + 2) / 4, gavg = (g00 + g01 + g10 + g11 + 2) / 4,
            bavg = (b00 + b01 + b10 + b11 + 2) / 4;
        auto enc_U = [](int r, int g, int b) { return uint8_t(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128); };
        auto enc_V = [](int r, int g, int b) { return uint8_t(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128); };
        bufUV[0] = enc_U(ravg, gavg, bavg);
        bufUV[1] = enc_V(ravg, gavg, bavg);
        bufUV += 2;
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
    uint8_t borderYt = RGB_to_Y(*bordervalue);
    convert(CGrid1View(borderYt), Grid1View(borderY));
    Vec2<uint8_t> borderUVt = V(RGB_to_U(*bordervalue), RGB_to_V(*bordervalue));
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
