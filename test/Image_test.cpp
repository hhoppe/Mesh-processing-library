// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Image.h"

#include "libHh/FileIO.h"  // remove_file()
#include "libHh/GridOp.h"  // crop()
#include "libHh/Random.h"
#include "libHh/Stat.h"
using namespace hh;

namespace {

// Create a small image of pseudorandom pixels.
Image random_image(const Vec2<int>& dims, int zsize, uint32_t seed) {
  Random random(seed);
  Image image(dims);
  image.set_zsize(zsize);
  image.set_silent_io_progress(true);
  for (Pixel& pixel : image) {
    for_int(z, 4) pixel[z] = uint8_t(random.get_uint64() % 256);
    if (zsize == 1) pixel[2] = pixel[1] = pixel[0];
    if (zsize < 4) pixel[3] = 255;
  }
  return image;
}

#if defined(HH_IMAGE_HAVE_WIC) || defined(HH_IMAGE_HAVE_LIBS)
// Write the image to a file, read it back, and verify that the pixels are unchanged.
void verify_file_round_trip(const Image& image, const string& filename) {
  image.write_file(filename);
  Image image2;
  image2.set_silent_io_progress(true);
  image2.read_file(filename);
  assertx(image2.dims() == image.dims());
  // A grayscale image may be read back as an RGB image.
  assertx(image2.zsize() == image.zsize() || (image.zsize() == 1 && image2.zsize() == 3));
  for (const auto& yx : range(image.dims())) assertx(equal(image2[yx], image[yx], max(image.zsize(), 3)));
  Image image3;
  image3.set_silent_io_progress(true);
  image3.read_file_bgra(filename);
  for (const auto& yx : range(image.dims())) {
    const Pixel& p = image[yx];
    assertx(rgb_equal(image3[yx], Pixel(p[2], p[1], p[0], p[3])));
  }
  assertx(remove_file(filename));
}
#endif

}  // namespace

int main() {
  {
    constexpr Pixel red{255, 0, 0, 255};
    dummy_use(red);
    constexpr Pixel green{0, 255, 0};
    dummy_use(green);
    constexpr Vec4<uint8_t> v{uint8_t{0}, uint8_t{0}, uint8_t{255}, uint8_t{255}};
    constexpr Pixel blue{v};
    dummy_use(blue);
  }
  if (1) {
    Image image(V(20, 20), Pixel(65, 66, 67, 72));
    const Bndrule bndrule = Bndrule::reflected;
    const Pixel gcolor(255, 255, 255, 255);
    {
      const Grid<2, Pixel>& grid = image;
      SHOW((image[19, 19]));
      const Grid<2, Pixel> newgrid = crop(grid, V(0, 0), V(10, 10), twice(bndrule), &gcolor);
      SHOW(newgrid.dims());
    }
  }
  {
    // Construction, attributes, and assignment.
    Image image;
    SHOW(image.dims(), image.zsize(), image.suffix());
    image.init(V(2, 3), Pixel(1, 2, 3, 4));
    SHOW(image.dims(), (image[1, 2]));
    image.set_zsize(4);
    image.set_suffix("png");
    const Image image2(image);  // The copy includes the attributes.
    SHOW(image2.zsize(), image2.suffix());
    Image image3(std::move(image));
    // NOLINTNEXTLINE(bugprone-use-after-move,clang-analyzer-cplusplus.Move): the moved-from object is left empty.
    assertx(image3.zsize() == 4 && image3.dims() == V(2, 3) && image.size() == 0);
    Matrix<Pixel> matrix(V(1, 2), Pixel::red());
    const Image image4(matrix);  // Copy of a matrix has default attributes.
    SHOW(image4.dims(), image4.zsize(), (image4[0, 1]));
    Image image5(std::move(matrix));                          // Move of a matrix transfers its allocation.
    assertx(image5.dims() == V(1, 2) && matrix.size() == 0);  // NOLINT(bugprone-use-after-move)
    image5 = image3;
    assertx(image5.dims() == V(2, 3) && image5.zsize() == 4 && image5.suffix() == "png");
    image5.clear();
    assertx(image5.dims() == V(0, 0));
    swap(image5, image3);
    assertx(image5.dims() == V(2, 3) && image3.size() == 0 && image5.zsize() == 4);
    for (const int zsize : {1, 3, 4}) {
      image5.set_zsize(zsize);
      assertx(image5.zsize() == zsize);
    }
  }
  {
    // Conversions between color and grayscale.
    Image image(V(1, 4));
    image[0, 0] = Pixel(255, 0, 0, 7);
    image[0, 1] = Pixel(0, 255, 0, 7);
    image[0, 2] = Pixel(0, 0, 255, 7);
    image[0, 3] = Pixel(10, 20, 30, 7);
    image.to_bw();
    SHOW(image.zsize());
    for_int(x, 4)
        assertx(image[0, x][0] == image[0, x][1] && image[0, x][0] == image[0, x][2] && image[0, x][3] == 255);
    Image image2(V(1, 256));  // A gray pixel maps to itself.
    for_int(x, 256) image2[0, x] = Pixel::gray(uint8_t(x));
    image2.to_bw();
    for_int(x, 256) assertx(image2[0, x] == Pixel::gray(uint8_t(x)));
    image.to_color();
    SHOW(image.zsize());
    image.to_color();  // A no-op on a color image.
    SHOW(image.zsize());
  }
  {
    // Pixel comparisons.
    const Pixel p1(10, 20, 30, 40), p2(10, 20, 30, 50), p3(13, 16, 30, 40);
    SHOW(rgb_equal(p1, p2), equal(p1, p2, 3), equal(p1, p2, 4), rgb_equal(p1, p3), rgb_dist2(p1, p3));
  }
  {
    // Filename and magic-byte recognition.
    for (const string filename : {"a.png", "dir.jpg/a.JPG", "a.jpeg", "a.avif", "a.mp4", "a", "a.png.txt"})
      SHOW(filename, filename_is_image(filename));
    string s;
    for (const uchar c : {uchar{1}, uchar{255}, uchar{'B'}, uchar{'P'}, uchar{137}, uchar{'I'}, uchar{'v'}, uchar{0}})
      s += " '" + string(image_suffix_for_magic_byte(c)) + "'";
    SHOW(s);
  }
  {
    // Conversion of matrices of various element types to images.
    {
      const Matrix<float> matrixf = {{0.f, .5f, 1.f}, {-1.f, 2.f, .25f}};
      SHOW(as_image(matrixf));
    }
    {
      // Each value rounds to the nearest level, like Vector4::pixel().
      Matrix<float> matrixf(V(1, 256));
      for_int(x, 256) matrixf[0][x] = (x + .49f) / 255.f;
      const Image imagef = as_image(matrixf);
      for_int(x, 256) assertx(imagef[0, x] == Pixel::gray(uint8_t(x)));
      for_int(x, 256) assertx(rgb_equal(imagef[0, x], Vector4(matrixf[0][x]).pixel()));
    }
    const Matrix<float> matrixf = {{.5f, .5f}};
    SHOW(as_image(matrixf)[0][1]);
    const Matrix<double> matrixd = {{.5}};
    SHOW(as_image(matrixd)[0][0]);
    const Matrix<Vector4> matrixv = {{Vector4(1.f, 0.f, .5f, 1.f), Vector4(0.f, 0.f, 0.f, 1.f)}};
    const Image imagev = as_image(matrixv);
    SHOW(imagev.zsize(), (imagev[0, 0]));
    const Matrix<Vector4> matrixv2 = {{Vector4(1.f, 0.f, .5f, .5f)}};
    SHOW(as_image(matrixv2).zsize());  // Translucent pixels yield an RGBA image.
    const Matrix<Vec3<float>> matrix3 = {{V(1.f, .5f, 0.f)}};
    SHOW(as_image(matrix3)[0][0]);
    const Matrix<Pixel> matrixp = {{Pixel(1, 2, 3, 4)}};
    SHOW(as_image(matrixp)[0][0]);
  }
  {
    // Conversions between RGB and YUV.
    const Pixel gray128 = Pixel::gray(128);
    SHOW(int{Y_from_RGB(gray128)}, int{U_from_RGB(gray128)}, int{V_from_RGB(gray128)});
    SHOW(int{Y_from_RGB(Pixel::black())}, int{Y_from_RGB(Pixel::white())});  // The limited range [16, 235].
    SHOW(YUV_Pixel_from_RGB(255, 0, 0), RGB_Pixel_from_YUV(16, 128, 128), RGB_Pixel_from_YUV(235, 128, 128));
    int max_error = 0;
    for (int r = 0; r < 256; r += 15) {
      for (int g = 0; g < 256; g += 15) {
        for (int b = 0; b < 256; b += 15) {
          const auto pixel = Pixel(uint8_t(r), uint8_t(g), uint8_t(b));
          const Pixel yuv = YUV_Pixel_from_RGB(r, g, b);
          assertx(yuv == Pixel(Y_from_RGB(pixel), U_from_RGB(pixel), V_from_RGB(pixel), 255));
          const Pixel rgb = RGB_Pixel_from_YUV(yuv[0], yuv[1], yuv[2]);
          assertx(rgb[3] == 255);
          for_int(z, 3) max_error = max(max_error, abs(rgb[z] - pixel[z]));
        }
      }
    }
    SHOW(max_error);
  }
  {
    // Conversions between RGB images and Nv12 (luminance and half-resolution chroma).
    const Image image = random_image(V(4, 6), 3, 1);
    Nv12 nv12(image.dims());
    SHOW(nv12.get_Y().dims(), nv12.get_UV().dims());
    convert_Image_to_Nv12(image, nv12);
    for (const auto& yx : range(image.dims())) assertx(nv12.get_Y()[yx] == Y_from_RGB(image[yx]));
    for (const auto& yx : range(nv12.get_UV().dims())) {
      // The chroma is computed from the average of the 2x2 block of pixels.
      Vec3<int> sum{0, 0, 0};
      for (const auto& d : range(V(2, 2))) for_int(z, 3) sum[z] += image[yx * 2 + d][z];
      const Pixel avg(uint8_t(sum[0] / 4), uint8_t(sum[1] / 4), uint8_t(sum[2] / 4));
      assertx(abs(nv12.get_UV()[yx][0] - U_from_RGB(avg)) <= 1 && abs(nv12.get_UV()[yx][1] - V_from_RGB(avg)) <= 1);
    }
    Image image2(image.dims()), image3(image.dims());
    convert_Nv12_to_Image(nv12, image2);
    convert_Nv12_to_Image_BGRA(nv12, image3);
    for (const auto& yx : range(image.dims())) {
      const Vec2<uint8_t>& uv = nv12.get_UV()[yx / 2];
      const Pixel expected = RGB_Pixel_from_YUV(nv12.get_Y()[yx], uv[0], uv[1]);
      assertx(image2[yx] == expected);
      assertx(image3[yx] == Pixel(expected[2], expected[1], expected[0], expected[3]));
    }
    // A uniform image survives the round trip with only a small error.
    const Image image4(V(4, 4), Pixel(200, 100, 50));
    Nv12 nv12b(image4.dims());
    convert_Image_to_Nv12(image4, nv12b);
    assertx(nv12b.get_UV()[1][1] == V(U_from_RGB(image4[0][0]), V_from_RGB(image4[0][0])));
    Image image5(image4.dims());
    convert_Nv12_to_Image(CNv12View(nv12b), image5);
    SHOW(image5[3][3], rgb_dist2(image5[3][3], image4[3][3]));
    for (const Pixel& pixel : image5) assertx(pixel == image5[0][0]);
    // Views onto an Nv12.
    Nv12View nv12v(nv12b);
    assertx(nv12v.get_Y().data() == nv12b.get_Y().data());
    const CNv12View cnv12v(nv12v);
    assertx(cnv12v.get_UV().data() == nv12b.get_UV().data());
    // Scaling an Nv12 image.
    Nv12 nv12c(V(8, 6));
    scale(nv12b, twice(FilterBnd(Filter::get("triangle"), Bndrule::reflected)), nullptr, nv12c);
    for (const uint8_t y : nv12c.get_Y()) assertx(abs(y - nv12b.get_Y()[0][0]) <= 1);
    for (const Vec2<uint8_t>& uv : nv12c.get_UV()) assertx(abs(uv[0] - nv12b.get_UV()[0][0][0]) <= 1);
  }
  {
    // Scaling of an image preserves a uniform color and the attributes.
    Image image(V(4, 6), Pixel(200, 100, 50, 255));
    image.set_zsize(4);
    const Image image2 = scale(image, V(2.f, .5f), twice(FilterBnd(Filter::get("triangle"), Bndrule::reflected)));
    SHOW(image2.dims(), image2.zsize());
    for (const Pixel& pixel : image2) assertx(pixel == image[0][0]);
    image.scale(V(.5f, 1.5f), twice(FilterBnd(Filter::get("box"), Bndrule::clamped)));
    SHOW(image.dims());
    for (const Pixel& pixel : image) assertx(pixel == Pixel(200, 100, 50, 255));
    // Scaling with a border value.
    const Image image3(V(2, 2), Pixel::black());
    const Pixel bordervalue = Pixel::white();
    const Image image4 =
        scale(image3, V(2.f, 2.f), twice(FilterBnd(Filter::get("triangle"), Bndrule::border)), &bordervalue);
    SHOW(image4);
  }
#if defined(HH_IMAGE_HAVE_WIC) || defined(HH_IMAGE_HAVE_LIBS)  // Without these, the image IO would require ffmpeg.
  {
    // Lossless file formats reproduce the pixels exactly.
    verify_file_round_trip(random_image(V(5, 7), 3, 2), "Image_test.png");
    verify_file_round_trip(random_image(V(3, 4), 4, 3), "Image_test.png");
    verify_file_round_trip(random_image(V(4, 3), 1, 4), "Image_test.png");
    verify_file_round_trip(random_image(V(6, 5), 3, 5), "Image_test.bmp");
    // Reading a missing file throws.
    bool threw = false;
    try {
      Image image;
      image.set_silent_io_progress(true);
      image.read_file("Image_test_nonexistent.png");
    } catch (const std::runtime_error&) {
      threw = true;
    }
    assertx(threw);
  }
#endif
}
