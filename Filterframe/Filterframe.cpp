// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Args.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GeomOp.h"
#include "libHh/Quaternion.h"  // pow(Frame, float)
#include "libHh/RangeOp.h"
#include "libHh/Stat.h"
using namespace hh;

namespace {

int every = 0;
int object = -1;
bool g_inverse = false;
float check_orthonormal = -1.f;  // If >= 0.f, tolerance to check.
bool b_normalize = false;
bool snap_to_axes = false;
float induce_roll = 0.f;
float lowpass = 1.f;  // Note that 1.f == no filtering.
bool add_mid_frames = false;
double delay = 0.;
int repeat = 1;
bool eof1 = false;
bool toasciit = false;  //  The name "toascii" seems to be a reserved identifier in Win32.
bool tobinary = false;

bool is_pretransf = false;
Frame cpretransf;
bool is_transf = false;
Frame ctransf;
bool statistics = false;
bool b_frame = false;

bool noinput = false;
int icount = 0;
int ocount = 0;

void do_create_euler(Args& args) {
  const float yaw = rad_from_deg(args.get_float());
  const float pitch = rad_from_deg(args.get_float());
  const float roll = rad_from_deg(args.get_float());
  const Vec3<float> angles(yaw, pitch, roll);
  const Frame frame = frame_from_euler_angles(angles, Frame::identity());
  assertx(FrameIO::write(std::cout, ObjectFrame{frame}));
  noinput = true;
}

void do_inverse() { g_inverse = true; }

void apply_induce_roll(Frame& frame) {
  static int icount1 = 0;
  static Point p0, p1, p2;
  p0 = p1;
  p1 = p2;
  p2 = frame.p();
  p2[2] = 0;  // Set elevations to zero.
  icount1++;
  if (icount1 < 3) return;
  const double v1x = double(p1[0]) - p0[0];
  const double v1y = double(p1[1]) - p0[1];
  const double v2x = double(p2[0]) - p0[0];
  const double v2y = double(p2[1]) - p0[1];
  const float sarea = float(v1y * v2x - v1x * v2y);
  const int num = 5;
  static Vec<float, num> sa;
  for_int(i, num - 1) sa[i] = sa[i + 1];
  sa[num - 1] = sarea;
  const float s = float(sum(sa)) / num;
  const float bank = s * induce_roll;
  frame = Frame::rotation(0, bank) * frame;
  HH_SSTAT(Sbank, bank);
  // Not used: it couldn't estimate curvature well enough from my gcanyon_4k2k_fly2.frame file.
}

bool process_frame(ObjectFrame& object_frame) {
  Frame& frame = object_frame.frame;
  icount++;
  if (object >= 0) object_frame.obn = object;
  if (g_inverse) assertw(frame.invert());
  if (is_pretransf) frame = cpretransf * frame;
  if (is_transf) frame = frame * ctransf;
  if (lowpass != 1.f) {
    static Frame frame_prev;
    static float zoom_prev;
    if (icount > 1) {
      frame = frame_prev * pow(~frame_prev * frame, lowpass);
      object_frame.zoom = zoom_prev * (1.f - lowpass) + object_frame.zoom * lowpass;
    }
    frame_prev = frame;
    zoom_prev = object_frame.zoom;
  }
  if (check_orthonormal >= 0.f) assertx(nearly_orthonormal(frame, check_orthonormal));
  if (b_normalize) frame = normalized_frame(frame, 1e-2f);
  if (snap_to_axes) {
    for_int(i, 3) {
      Vector& vec = frame.v(i);
      const int axis = arg_max(abs(vec));
      for_int(j, 3) vec[j] = j == axis ? sign(vec[j]) : 0.f;
    }
    frame.p() = Point(0.f, 0.f, 0.f);
    object_frame.zoom = 0.f;
  }
  if (induce_roll) apply_induce_roll(frame);
  if (add_mid_frames) {
    static Frame frame_prev;
    static float zoom_prev;
    if (icount > 1) {
      Frame frame_new = frame_prev * pow(~frame_prev * frame, .5f);
      frame_new.p() = interp(frame_prev.p(), frame.p(), .5f);
      const float zoom_new = (object_frame.zoom + zoom_prev) * .5f;
      const ObjectFrame object_frame_new{frame_new, object_frame.obn, zoom_new, object_frame.binary};
      if (!FrameIO::write(std::cout, object_frame_new)) return true;
    }
    frame_prev = frame;
    zoom_prev = object_frame.zoom;
  }
  if (toasciit) object_frame.binary = false;
  if (tobinary) object_frame.binary = true;
  if (statistics) {
    static Point origin_prev;
    if (icount > 1) HH_SSTAT(Sdisp, dist(frame.p(), origin_prev));
    origin_prev = frame.p();
    HH_SSTAT(Sxlen, mag(frame.v(0)));
    HH_SSTAT(Sylen, mag(frame.v(1)));
    HH_SSTAT(Szlen, mag(frame.v(2)));
    HH_SSTAT(Sxydot, dot(frame.v(0), frame.v(1)));
    HH_SSTAT(Syzdot, dot(frame.v(1), frame.v(2)));
    HH_SSTAT(Szxdot, dot(frame.v(2), frame.v(0)));
    return false;
  }
  for_int(irepeat, repeat) {
    if (delay) my_sleep(delay);
    if (!FrameIO::write(std::cout, object_frame)) return true;
    ocount++;
    if (b_frame) std::cout << "f 0 0 0\n";
    if (ocount == 1 && eof1) std::cout << "q 0 0 0\n";
    std::cout.flush();
  }
  return false;
}

void process_frames(std::istream& is) {
  string line;
  for (;;) {
    if (is.peek() == '#') {
      assertx(my_getline(is, line));
      std::cout << line << "\n";
      continue;
    }
    auto object_frame = FrameIO::read(is);
    if (!object_frame) break;
    process_frame(*object_frame);
  }
  if (!is.good() && !is.eof()) {
    assertnever("Read error");
  } else if (!std::cout.good()) {
    assertnever("Write error");
  } else if (!icount) {
    SHOW("warning: no frames read");
  }
}

}  // namespace

int main(int argc, const char** argv) {
  string transf;
  string pretransf;
  bool normalize = false, frame = false, stat = false;
  ParseArgs args(argc, argv);
  HH_ARGSC("A frame stream is read from stdin or filename in first arg except with the following arguments:");
  HH_ARGSD(create_euler, "yaw pitch roll : create frame from Euler angles (degrees)");
  HH_ARGSC("", ":");
  HH_ARGSP(every, "i : use only every ith element");
  HH_ARGSP(object, "obn : force object # to obn");
  HH_ARGSD(inverse, ": replace each frame by its inverse");
  HH_ARGSP(pretransf, "'frame' : pre-transform by frame");
  HH_ARGSP(transf, "'frame' : post-transform by frame");
  HH_ARGSP(check_orthonormal, "tolerance : if non-negative, assert each frame is orthonormal");
  HH_ARGSF(normalize, ": apply orthogonalization and/or normalization");
  HH_ARGSF(snap_to_axes, ": change each frame vector to the nearest axis");
  HH_ARGSP(induce_roll, "factor : for flight over terrain");
  HH_ARGSP(lowpass, "factor : contribution of new frame [0..1]");
  HH_ARGSF(add_mid_frames, ": add interpolating frame between each pair");
  HH_ARGSP(delay, "fsec : introduce delay between frames");
  HH_ARGSP(repeat, "n : repeat every frame n times");
  HH_ARGSF(frame, ": introduce 'f 0 0 0' between frames");
  HH_ARGSF(stat, ": output statistics");
  HH_ARGSF(eof1, ": introduce 'q 0 0 0' after frame 1");
  HH_ARGSF(toasciit, ": force output to be ascii text");
  HH_ARGSF(tobinary, ": force output to be binary");
  const string arg0 = args.num() ? args.peek_string() : "";
  if (ParseArgs::special_arg(arg0)) args.parse(), exit(0);
  const string filename = args.num() && (arg0 == "-" || arg0[0] != '-') ? args.get_filename() : "-";
  args.parse();
  if (noinput) exit(0);
  if (pretransf != "") {
    is_pretransf = true;
    cpretransf = FrameIO::parse_frame(pretransf);
  }
  if (transf != "") {
    is_transf = true;
    ctransf = FrameIO::parse_frame(transf);
  }
  b_normalize = normalize;
  statistics = stat;
  b_frame = frame;
  RFile fi(filename);
  process_frames(fi());
  return 0;
}
