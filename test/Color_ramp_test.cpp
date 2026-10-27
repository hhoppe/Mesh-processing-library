// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#include "libHh/Color_ramp.h"

using namespace hh;

static_assert(k_color_ramp.num() == 256);
static_assert(k_color_ramp[0] == Pixel(0, 127, 0) && k_color_ramp[255] == Pixel(255, 0, 255));

int main() {
  for (const int i : {0, 1, 64, 127, 128, 129, 192, 254, 255}) SHOW(i, k_color_ramp[i]);
  // The ramp is opaque, and runs from green through yellow to magenta.
  for_int(i, 256) assertx(k_color_ramp[i][3] == 255);
  for_int(i, 128) assertx(k_color_ramp[i][2] == 0);
  for_intL(i, 128, 256) assertx(k_color_ramp[i][0] == 255);
  assertx(k_color_ramp[127] == Pixel(255, 255, 0) && k_color_ramp[128] == Pixel(255, 255, 0));
  // Each channel varies monotonically within each half of the ramp, in steps of at most 3.
  for_intL(i, 1, 256) {
    const Pixel& p0 = k_color_ramp[i - 1];
    const Pixel& p1 = k_color_ramp[i];
    for_int(c, 3) assertx(abs(int(p1[c]) - int(p0[c])) <= 3);
    if (i < 128) {
      assertx(p1[0] > p0[0] && p1[1] >= p0[1]);
    } else if (i > 128) {
      assertx(p1[1] < p0[1] && p1[2] > p0[2]);
    }
  }
}
