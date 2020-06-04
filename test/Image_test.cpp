// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Image.h"

#include "libHh/Stat.h"
using namespace hh;

int main() {
  {
    constexpr Pixel red{255, 0, 0, 255};
    (void)red;  // Avoid unused variable warning
    constexpr Pixel green{0, 255, 0};
    (void)green;  // Avoid unused variable warning
    constexpr Vec4<uint8_t> v{uint8_t{0}, uint8_t{0}, uint8_t{255}, uint8_t{255}};
    const Pixel blue{v};  // constexpr fails in VS 2015
    (void)blue;           // Avoid unused variable warning
  }
  if (1) {
    Image image(V(20, 20), Pixel(65, 66, 67, 72));
    Bndrule bndrule = Bndrule::reflected;
    Pixel gcolor(255, 255, 255, 255);
    {
      const Grid<2, Pixel>& grid = image;
      SHOW(image[19][19]);
      Grid<2, Pixel> newgrid = crop(grid, V(0, 0), V(10, 10), twice(bndrule), &gcolor);
      SHOW(newgrid.dims());
    }
  }
}
