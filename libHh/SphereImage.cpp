// -*- C++ -*-
#include "libHh/SphereImage.h"

#include "libHh/Vector4.h"

namespace hh {

namespace {

void blend_pixels(Pixel& pixel0, Pixel& pixel1) {
  const Pixel average_pixel = interp(Vector4(pixel0), Vector4(pixel1)).pixel();
  pixel0 = pixel1 = average_pixel;
}

}  // namespace

bool is_sphere_image_domain(std::string_view domain) {
  return domain == "tetra" || domain == "octa" || domain == "octaflat" || domain == "cube";
}

void feather_sphere_image(Image& image, std::string_view domain) {
  const int ny = image.ysize(), nx = image.xsize();
  const bool expecting_periodic_texturing_mode = false;  // By default, the texturing mode is often GL_CLAMP_TO_EDGE.
  if (0) {
  } else if (domain == "tetra") {
    assertx(ny == nx && ny % 2 == 0);
    const int h = ny / 2;
    // Blend across the left and right sides of the image.
    if (!expecting_periodic_texturing_mode)
      for (const int i : range(h)) blend_pixels(image[h + i, 0], image[h + i, h * 2 - 1]);
    // Assign pixel values above the rectangle.
    for (const int i : range(h * 2)) image[h - 1, i] = image[h, h * 2 - 1 - i];
    // Blend pixel values in the lowest row of the rectangle.
    for (const int i : range(h)) blend_pixels(image[h * 2 - 1, i], image[h * 2 - 1, h * 2 - 1 - i]);

  } else if (domain == "octa" || domain == "octaflat") {
    // For each of the four image boundaries, we must establish mirror symmetry within the pixels on the boundary.
    // This leads to a narrow (1-texel width) band of zero-derivative signal across the texture boundary, but this
    // is much better than leaving a visible discontinuity.
    for (const int axis : {0, 1})
      for (const int column_index : {0, image.dim(axis) - 1}) {  // (General "column" is either row or col.)
        StridedArrayView<Pixel> single_border = grid_column(image, 1 - axis, twice(0).with(axis, column_index));
        const int num = single_border.num();
        for (const int i : range(num / 2)) blend_pixels(single_border[i], single_border[num - 1 - i]);
      }

  } else if (domain == "cube") {
    // Blend across the left and right sides of the image.
    assertx(ny == nx && ny % 4 == 0);
    const int q = ny / 4;
    if (!expecting_periodic_texturing_mode)
      for (const int i : range(q)) blend_pixels(image[q * 2 + i, 0], image[q * 2 + i, q * 4 - 1]);
    // Assign pixel values in the horizontal segment above the top of the "+" shape in the image.
    for (const int i : range(q)) image[q - 1, q + i] = image[q * 2, q * 4 - 1 - i];
    // Blend the horizontal segment at the base of the "+" (lowest image row) with the adjacent cube face.
    for (const int i : range(q)) blend_pixels(image[q * 4 - 1, q + i], image[q * 3 - 1, q * 4 - 1 - i]);  // (A).
    // Assign pixel values above the left side of the "+" shape.
    for (const int i : range(q)) image[q * 2 - 1, i] = image[q * 2 - 1 - i, q];
    // Assign pixel values below the left side of the "+" shape.
    for (const int i : range(q)) image[q * 3, i] = image[q * 4 - 1 - i, q];
    // Assign pixel values above the right side of the "+" shape.
    for (const int i : range(q)) image[q * 2 - 1, q * 2 + i] = image[q * 2 - 1 - i, q * 2 - 1];
    for (const int i : range(q)) image[q * 2 - 1, q * 3 + i] = image[q, q * 2 - 1 - i];
    // Assign pixel values below the right side of the "+" shape.
    for (const int i : range(q)) image[q * 3, q * 2 + i] = image[q * 4 - 1 - i, q * 2 - 1];
    for (const int i : range(q)) image[q * 3, q * 3 + i] = image[q * 4 - 1, q * 2 - 1 - i];  // Already blended in (A).
    // Assign pixel values left of the top part of the "+" shape.
    for (const int i : range(q)) image[q + i, q - 1] = image[q * 2, i];
    // Assign pixel values left of the bottom part of the "+" shape.
    for (const int i : range(q)) image[q * 3 + i, q - 1] = image[q * 3 - 1, q - 1 - i];
    // Assign pixel values right of the top part of the "+" shape.
    for (const int i : range(q)) image[q + i, q * 2] = image[q * 2, q * 3 - 1 - i];
    // Assign pixel values right of the bottom part of the "+" shape.
    for (const int i : range(q)) image[q * 3 + i, q * 2] = image[q * 3 - 1, q * 2 + i];
    // Assign two leftover pixels.
    image[q - 1, q - 1] = image[q * 2, q * 4 - 1];
    image[q - 1, q * 2] = image[q * 2, q * 3];
  } else {
    assertnever("Sphere image domain '" + string(domain) + "' not recognized");
  }
}

}  // namespace hh
