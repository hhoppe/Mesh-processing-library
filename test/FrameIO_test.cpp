// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/FrameIO.h"
using namespace hh;

int main() {
  const bool frame_binary = getenv_bool("FRAME_BINARY");
  for (;;) {
    auto object_frame = FrameIO::read(std::cin);
    if (!object_frame) break;
    if (0) {
      SHOW_PRECISE(object_frame->frame);
      SHOW_PRECISE(object_frame->zoom);
    }
    object_frame->binary = frame_binary;
    FrameIO::write(std::cout, *object_frame);
  }
}
