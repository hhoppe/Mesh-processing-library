// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/FrameIO.h"

#include <sstream>  // std::stringstream
using namespace hh;

namespace {

bool equal(const ObjectFrame& of1, const ObjectFrame& of2) {
  return of1.frame == of2.frame && of1.obn == of2.obn && of1.zoom == of2.zoom && of1.binary == of2.binary;
}

// Self-checks, which produce no output (because stdout is the filtered stream of frames).
void self_test() {
  // The text format uses enough significant digits to represent each float exactly.
  ObjectFrame object_frame{Frame::rotation(1, .3f) * Frame::translation(V(1.f / 3.f, -2e5f, 1e-7f)), 3, .2f, false};
  const string str = FrameIO::create_string(object_frame);
  assertx(str.starts_with("F 3  ") && str.ends_with('\n'));
  assertx(FrameIO::parse_frame(str) == object_frame.frame);
  // A stream mixing text and binary records.
  std::stringstream ss;
  assertx(FrameIO::write(ss, object_frame));
  ObjectFrame object_frame2{Frame::identity(), 65535, 1.5f, true};
  assertx(FrameIO::write(ss, object_frame2));
  ObjectFrame object_frame3{FrameIO::get_not_a_frame(), 0, 0.f, false};
  assertx(FrameIO::write(ss, object_frame3));
  const auto read1 = FrameIO::read(ss), read2 = FrameIO::read(ss), read3 = FrameIO::read(ss);
  assertx(read1 && equal(*read1, object_frame));
  assertx(read2 && equal(*read2, object_frame2));
  assertx(read3 && equal(*read3, object_frame3));
  assertx(!FrameIO::read(ss));  // The end of the stream.
  // Special frames.
  assertx(FrameIO::is_not_a_frame(read3->frame) && !FrameIO::is_not_a_frame(Frame::identity()));
  assertx(FrameIO::parse_frame("F 0  1 0 0  0 1 0  0 0 1  0 0 0  0").is_ident());
}

}  // namespace

int main() {
  self_test();
  const bool frame_binary = getenv_bool("FRAME_BINARY");
  for (;;) {
    auto object_frame = FrameIO::read(std::cin);
    if (!object_frame) break;
    object_frame->binary = frame_binary;
    assertx(FrameIO::write(std::cout, *object_frame));
  }
}
