// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Pixel.h"

#include "libHh/Array.h"
using namespace hh;

int main() {
  static_assert(sizeof(Pixel) == 4 && alignof(Pixel) == 4);
  {
    const Pixel pix(10, 20, 30, 40);
    SHOW(pix, pix.to_BGRA(), pix.to_BGRA().from_BGRA());
    SHOW(Pixel(1, 2, 3));  // The alpha defaults to 255.
    SHOW(Pixel::black(), Pixel::white(), Pixel::gray(128));
    SHOW(Pixel::red(), Pixel::green(), Pixel::blue(), Pixel::yellow(), Pixel::pink());
  }
  {
    // The specialized operator==() agrees with comparing each channel.
    const Array<Pixel> pixels{Pixel(0, 0, 0, 0), Pixel(0, 0, 0, 1), Pixel(1, 0, 0, 0), Pixel(255, 255, 255, 255),
                              Pixel(255, 255, 255, 254)};
    for (const Pixel& p1 : pixels) {
      for (const Pixel& p2 : pixels) {
        bool all_equal = true;
        for_int(c, 4) all_equal = all_equal && p1[c] == p2[c];
        assertx((p1 == p2) == all_equal);
      }
    }
    static_assert(Pixel(1, 2, 3, 4) == Pixel(1, 2, 3, 4));  // Usable at compile time.
  }
  {
    Array<Pixel> ar{Pixel(1, 2, 3, 4), Pixel(5, 6, 7, 8)};
    convert_rgba_bgra(ar);
    SHOW(ar[0], ar[1]);
    convert_bgra_rgba(ar);
    assertx(ar[0] == Pixel(1, 2, 3, 4) && ar[1] == Pixel(5, 6, 7, 8));
  }
}
