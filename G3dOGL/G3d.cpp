// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "G3dOGL/G3d.h"

#if defined(_WIN32)
#include <io.h>  // close(), dup2()
#else
#include <unistd.h>  // close(), dup2()
#endif

#include "libHh/Args.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/RangeOp.h"
#include "libHh/StringOp.h"
using namespace hh;

namespace g3d {

ERatemode ratemode = ERatemode::move;
EFlightmode flightmode = EFlightmode::none;
int cob = 1;
bool eye_move = true;
bool object_mode = true;
float ddistance = 1.f;
bool expo = true;
bool geomorph = true;
bool globemode = false;
bool sizemode = false;
bool viewmode = false;
bool editmode = false;
bool keep_active = false;
bool auto_level = false;
bool mode_centroid = true;
int want_jump = 0;
bool auto_hither = true;
int timingtest_nframes = 0;
bool play = false;
int obview = 0;
int info = 1;
bool input = false;
bool asynchronousinput = false;
bool keep_stdin_open = false;
bool killeof = false;
bool output = false;
bool obinary = false;
float fchange = 0.f;
bool spacekill = false;
bool cur_needs_redraw = false;
bool prev_needed_redraw = false;
int button_active = 0;
Selected selected;
Frame tview = Frame::identity();
float zoom = 1.f;
bool iostat = false;
int num_input_frames = 0;
bool timestat = false;
bool terse = false;
bool tried_input = false;
string statefile;
string caption;
string keystring;
Array<string> g_aargs1;    // not including argv0
extern string g_filename;  // used in G3dOGL.cpp
string g_filename;
bool ob1_updated = false;
float anglethresh = -1.f;  // no angle threshold set
bool lod_mode = false;
float lod_level = 1.f;  // used to be default 0.f
float override_frametime = 0.f;
Point rec_point;

const FlagMask mflag_ok = Mesh::allocate_flag();

// *** object

void object::clear() {
  _def = false;
  for_int(c, 3) _stat_coord[c].zero();
  if (_mesh) {
    _mesh = nullptr;
    HB::segment_attach_mesh(_obn, nullptr);
  }
}

void object::enter_point(const Point& p) {
  _def = true;
  for_int(c, 3) _stat_coord[c].enter(p[c]);
}

void object::update_stats() {
  if (!_stat_coord[0].num()) return;
  _radius = 0.f;
  for_int(c, 3) {
    _pavg[c] = _stat_coord[c].avg();
    _radius += _stat_coord[c].num() > 1 ? _stat_coord[c].sdv() : 0.f;
    _bbox[0][c] = _stat_coord[c].min();
    _bbox[1][c] = _stat_coord[c].max();
  }
}

bool object::defined() const { return _def; }
bool object::visible() const { return _vis; }
const Frame& object::t() const { return _t; }
void object::set_vis(bool i) {
  if (_vis != i) _needs_update = true;
  _vis = i;
}
Frame& object::tm() {
  _needs_update = true;
  return _t;
}

const Point& object::center() const {
  if (_obn) assertw(_def);
  return !_def || !mode_centroid ? rec_point : _pavg;
}

const Bbox<float, 3>& object::bbox() const {
  static const Bbox k_bbempty(Point(0.f, 0.f, 0.f), Point(0.f, 0.f, 0.f));
  if (!_def) {
    Warning("Undefined bbox");
    return k_bbempty;
  }
  return _bbox;
}

float object::radius() const { return _radius; }

void object::update() {
  if (!_needs_update) return;
  _needs_update = false;
  HB::update_seg(_obn, t(), visible());
}

GMesh* object::get_mesh() {
  if (!_mesh) {
    _mesh = make_unique<GMesh>();
    _mesh->gflags().flag(g3d::mflag_ok) = true;
    HB::segment_attach_mesh(_obn, _mesh.get());
  }
  return _mesh.get();
}

// *** objects

void objects::copy(int obf, int obt) {
  assertx(legal(obf) && legal(obt));
  assertw(_ob[obf].defined());
  _ob[obt].clear();
  for_int(c, 3) _ob[obt]._stat_coord[c].add(_ob[obf]._stat_coord[c]);
  _ob[obt].update_stats();
  _ob[obt]._def = true;
}

objects g_obs;

// hook from HB
void UpdateOb1Bbox(const Bbox<float, 3>& bbox) {
  assertx(!g_obs[1].defined());
  g_obs[1].enter_point(Point(bbox[0][0], bbox[0][1], bbox[0][2]));
  g_obs[1].enter_point(Point(bbox[0][0], bbox[0][1], bbox[1][2]));
  g_obs[1].enter_point(Point(bbox[0][0], bbox[1][1], bbox[0][2]));
  g_obs[1].enter_point(Point(bbox[0][0], bbox[1][1], bbox[1][2]));
  g_obs[1].enter_point(Point(bbox[1][0], bbox[0][1], bbox[0][2]));
  g_obs[1].enter_point(Point(bbox[1][0], bbox[0][1], bbox[1][2]));
  g_obs[1].enter_point(Point(bbox[1][0], bbox[1][1], bbox[0][2]));
  g_obs[1].enter_point(Point(bbox[1][0], bbox[1][1], bbox[1][2]));
  g_obs[1].update_stats();
  g_obs.last = 1;
  HB::open_segment(1);
  HB::close_segment();
  g_obs[1].update();
  ob1_updated = true;
  assertx(g_obs[1].defined());
}

namespace {

void do_key(Args& args) { keystring += args.get_string(); }

void do_frame(Args& args) {
  std::istringstream iss(args.get_string());
  const ObjectFrame object_frame = *assertx(FrameIO::read(iss));
  UpdateFrame(object_frame);
}

bool s3dname_command_exists() { return command_exists_in_path("s3dname"); }

bool try_finding_it(const string& name) {
  if (!s3dname_command_exists()) {
    if (k_debug && 0) std::cerr << "(s3dname not found)\n";
    return false;
  }
  // Was "s3dname.bat" due to problem of launching "c:/perl/bin/perl /cygdrive/c/hh/bin/s3dname",
  //  but now solved using c:/cygwin/etc/fstab .
  // Added "<$nul" so that "G3dOGL ~/data/mesh/cat.m &" does not freeze.  (Nowadays we use "G3d ~/data/mesh".)
  // However, it still seems to freeze.  Added "close(STDIN)" inside s3dname.
  // #if 0 && defined(_WIN32)
  //   const string snul = "nul";
  // #else
  //   const string snul = "/dev/null";
  // #endif
  // bash:  0<&- or <&-  close stdin.
  // string com = "s3dname -s " + quote_arg_for_shell(name) + " <" + snul + " |";
  string com = "s3dname -s " + quote_arg_for_shell(name) + " |";
  try {
    RFile fi(com);  // may throw
    string line;
    if (!my_getline(fi(), line) || line == "") {
      if (k_debug) std::cerr << "(s3dname did not function)\n";
      return false;
    }
    statefile = line;
  } catch (const std::runtime_error& ex) {
    if (k_debug) std::cerr << string("Could not launch s3dname: ") + ex.what();
    return false;
  }
  return true;
}

void initial_statefile() {
  if (statefile != "" && file_exists(statefile)) return;
  if (statefile != "" && try_finding_it(statefile)) return;
  if (g_filename != "" && try_finding_it(g_filename)) return;
  if (statefile == "") statefile = "noname.s3d";
}

}  // namespace

void ExpandStateFilename() {
  if (contains(statefile, '/')) return;
  if (contains(statefile, '\\')) return;
  try_finding_it(statefile);
}

void UpdateFrame(const ObjectFrame& object_frame) {
  const int obn = object_frame.obn;
  assertx(g_obs.legal(obn));
  if (FrameIO::is_not_a_frame(object_frame.frame)) {
    g_obs[obn].set_vis(false);
  } else {
    g_obs[obn].tm() = object_frame.frame;
    if (!obn && object_frame.zoom) zoom = object_frame.zoom;
  }
}

}  // namespace g3d

using namespace g3d;

int main(int argc, const char** argv) {
  ensure_utf8_encoding(argc, argv);
  Array<string> aargs(argv, argv + argc);
  lod_level = getenv_float("LOD_LEVEL", lod_level);
  override_frametime = getenv_float("G3D_FRAMETIME", override_frametime);
  selected.frel = Frame::identity();
  bool hb_success = HB::init(aargs, KeyPressed, ButtonPressed, WheelTurned, Draw);
  float hither = -1.f, yonder = -1.f;
  bool eyeob = false;
  string statefilename;
  string title;
  ParseArgs args(aargs);
  HH_ARGSP(title, "name : set window title bar");
  HH_ARGSP(hither, "f : set hither plane distance");
  HH_ARGSP(yonder, "f : set yonder plane distance");
  HH_ARGSF(eyeob, ": make first object be eye object");
  HH_ARGSP(zoom, "f : set focal length");
  HH_ARGSF(iostat, ": show I/O statistics");
  HH_ARGSF(timestat, ": show timing statistics");
  HH_ARGSF(terse, ": turn diagnostics off");
  HH_ARGSF(killeof, ": exit upon reading EOF");
  HH_ARGSF(spacekill, ": exit upon space key press");
  HH_ARGSP(statefilename, "s3dname : name of .s3d file (frames)");
  HH_ARGSP(caption, "string : draw at bottom of window");
  HH_ARGSD(key, "keystring : simulate key presses");
  HH_ARGSD(frame, "'frame' : change an object's frame");
  HH_ARGSF(input, ": read stdin even if have filenames");
  HH_ARGSF(asynchronousinput, ": update between EndFrames");
  HH_ARGSP(anglethresh, "angle : sharp dihedral angle");
  args.other_args_ok();
  if (!args.parse_and_extract(g_aargs1) || !hb_success) return 0;
  g_aargs1.shift();  // ignore argv0
  if (hither >= 0) {
    auto_hither = false;
    HB::set_hither(hither);
  }
  if (yonder >= 0) HB::set_yonder(yonder);
  if (eyeob) g_obs.first = 0;
  if (g_aargs1.num() == 0) {
    if (!g_obs[1].defined()) {  // not UpdateOb1Bbox()
      input = true;
    }
  }
  if (g_aargs1.contains("-")) keep_stdin_open = true;
  if (input) {
#if !defined(_WIN32)
    assertx(set_fd_no_delay(0, true));
#endif
    HB::watch_fd0(InputArrived);
  } else if (!keep_stdin_open) {
    // Close stdin unless we need it, for Windows app started from emacs shell.
    if (0) assertx(!HH_POSIX(close)(0));
    // Close stdin, but do not leave fd0 empty in case we open another file.
    if (1) assertx(HH_POSIX(dup2)(1, 0) >= 0);
  }
  auto func_try_set_gfilename = [](string str) {
    if (g_filename != "") return;
    if (starts_with(str, "bboxtomesh ") || is_pipe(str) || is_url(str)) return;
    g_filename = str;
  };
  if (eyeob && g_aargs1.num() >= 2) func_try_set_gfilename(g_aargs1[1]);
  if (!eyeob && g_aargs1.num() >= 1) func_try_set_gfilename(g_aargs1[0]);
  if (statefilename != "") statefile = statefilename;
  initial_statefile();
  if (g_filename == "" && statefile != "") g_filename = statefile;
  if (g_filename == "") g_filename = "-";
  ReadFiles(true);
  if (input) ReadInput(true);
  // For command "PMview parasauru.pm -key Dt -st parasaur", window title ("parasaur") is poor choice;
  //  it occurs because PMview passes name through -st, which is overridden.?
  {
    string filenametail = get_path_tail(g_filename);
    HB::set_window_title(title != "" ? title : sform("G3D %.80s", filenametail.c_str()));
  }
  // always jump to good viewpoint
  if (contains(keystring, 'j') || statefile == "none") {
    // do nothing
  } else if (statefile != "noname.s3d") {
    keystring += ",";
  } else if (!contains(keystring, 'j')) {
    keystring += "j";
  }
  HB::set_current_object(cob);
  HB::open();
  if (!k_debug) {  // Faster exit, without destruction of Pool data, etc.
    exit_immediately(0);
  }
  for (int i = g_obs.first; i <= g_obs.last; i++) {
    g_obs[i].clear();
    HB::clear_segment(i);
  }
  return 0;
}
