// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "FrameIO.h"
using namespace hh;

int main() {
  Frame f;
  int obn;
  float zoom;
  bool bin;
  const bool frame_binary = getenv_bool("FRAME_BINARY");
  for (;;) {
    if (!FrameIO::read(std::cin, f, obn, zoom, bin)) break;
    if (0) {
      SHOW_PRECISE(f);
      SHOW_PRECISE(zoom);
    }
    bin = frame_binary;
    FrameIO::write(std::cout, f, obn, zoom, bin);
  }
}
