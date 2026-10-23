// -*- C++ -*-
#include "libHh/SphereImage.h"

using namespace hh;

namespace {

// Returns an image whose pixels all differ, so that the effect of feathering is visible.
Image create_ramp_image(int size) {
  Image image(V(size, size));
  for (const auto& yx : range(image.dims()))
    image[yx] = Pixel(uint8_t(yx[0] * 16), uint8_t(yx[1] * 16), uint8_t(yx[0] * 4 + yx[1]), 255);
  return image;
}

// Returns a weighted sum of the pixel channels, which summarizes the image.
int64_t checksum(const Image& image) {
  int64_t sum = 0;
  for (const Pixel& pixel : image) sum += int64_t{pixel[0]} + int64_t{pixel[1]} * 7 + int64_t{pixel[2]} * 13;
  return sum;
}

void show_row(const Image& image, int y) {
  string s;
  for_int(x, image.xsize()) s += " " + std::to_string(image[y, x][0]) + "," + std::to_string(image[y, x][1]);
  showf("row %d:%s\n", y, s.c_str());
}

void test_domains() {
  for (const string name : {"tetra", "octa", "octaflat", "cube", "sphere", ""})
    showf("is_sphere_image_domain(\"%s\") = %d\n", name.c_str(), is_sphere_image_domain(name));
}

void test_octaflat() {
  // Each image border becomes mirror-symmetric about its midpoint.
  const int size = 8;
  Image image = create_ramp_image(size);
  feather_sphere_image(image, "octaflat");
  for (const int border : {0, size - 1}) {
    for_int(i, size) {
      assertx(image[border, i] == image[border, size - 1 - i]);
      assertx(image[i, border] == image[size - 1 - i, border]);
    }
  }
  show_row(image, 0);
  show_row(image, 1);  // An interior row is unchanged except at its two border pixels.
  show_row(image, size - 1);
}

void test_tetra_and_cube() {
  for (const string domain : {"tetra", "cube"}) {
    Image image = create_ramp_image(8);
    const int64_t sum_before = checksum(image);
    feather_sphere_image(image, domain);
    SHOW(domain, sum_before, checksum(image));
  }
}

}  // namespace

int main() {
  test_domains();
  test_octaflat();
  test_tetra_and_cube();
}
