// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_G3DOGL_G3D_H_
#define MESH_PROCESSING_G3DOGL_G3D_H_

#include "G3dOGL/HB.h"
#include "libHh/Array.h"
#include "libHh/Bbox.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/Geometry.h"
#include "libHh/Sac.h"
#include "libHh/Stat.h"

namespace g3d {
using namespace hh;

// mode of control
enum class ERatemode { move, position, step };
extern ERatemode ratemode;
enum class EFlightmode { none, fly, flight, automatic, bobble };
extern EFlightmode flightmode;
extern int cob;                 // current object to transform
extern bool eye_move;           // move eyepoint instead of object
extern bool object_mode;        // transform in observer-relative frame
extern float ddistance;         // amplitude of displacements
extern bool expo;               // exponential displacem. as function of mouse
extern bool geomorph;           // use smooth transitions in lod
extern bool globemode;          // virtual sphere interface
extern bool sizemode;           // resize object
extern bool viewmode;           // apply changes to tview frame
extern bool editmode;           // edit mesh vertices
extern bool keep_active;        // keep motion going through button release
extern bool auto_level;         // keep view level at all times
extern bool mode_centroid;      // use centroid instead of origin
extern int want_jump;           // keep doing 'j' if appending object (0, 1, 2)
extern bool auto_hither;        // set hither dist automatically
extern int timingtest_nframes;  // count down
extern bool play;               // cycle through loaded objects

constexpr int full_timingtest_nframes = 100;

// mode of display
extern int obview;  // object frame to use for view
extern int info;    // infoline (1=small, 2=verbose)

// mode of stream input
extern bool input;              // look at input (== watch_fd0 state)
extern bool asynchronousinput;  // update between EndFrames
extern bool keep_stdin_open;    // do not close stdin even if no input
extern bool killeof;            // kill g3d when EOF is read

// mode of stream output
extern bool output;   // print frame on stdout
extern bool obinary;  // output format

// state for window input
extern float fchange;            // time in seconds since last refresh
extern bool spacekill;           // kill g3d when space is hit
extern bool cur_needs_redraw;    // current window calls for a redraw
extern bool prev_needed_redraw;  // previous window called for a redraw

extern int button_active;  // 0=no, 1-3=which button
struct sselected {
  bool shift;
  Vec2<float> yxpressed;  // location button was initially pressed
  Vec2<float> yx;         // current location
  Vec2<float> yxio;       // change in location of button, misc. scales
  Vec2<float> yxfo;
  int obn;
  GMesh* mesh;
  Vertex v;
  Frame frel;  // for !object_mode, change of axis transform
};
extern sselected selected;

// viewing transforms
extern Frame tview;  // view offset
extern float zoom;   // tan(angle_of_view)

// statistics
extern bool iostat;           // show stats on I/O
extern int num_input_frames;  // # input frames read in current frame
extern bool timestat;         // show stats on frame rate

// misc
extern bool terse;              // let g3d be terse
extern bool tried_input;        // got an input event for this screen
extern string statefile;        // name of stateg3d file
extern string caption;          // string to put at bottom of window
extern string keystring;        // -key strings concatenated together
extern Array<string> g_aargs1;  // not including argv0
extern bool ob1_updated;

// mesh manipulation
extern float anglethresh;  // dihedral angle threshold
extern bool subdivmode;    // subdivision surface editing mode

// level of detail
extern bool lod_mode;
extern float lod_level;

extern float override_frametime;  // if nonzero, constant frame time

extern Point rec_point;

extern int demofly_mode;
extern float demofly_idle_time;
extern float demofly_idle_time_thresh;

class object {
 public:
  void clear();  // (re)-initialize for definition
  void enter_point(const Point& p);
  void update_stats();
  bool defined() const;
  bool visible() const;
  const Frame& t() const;
  void set_vis(bool i);
  Frame& tm();
  const Point& center() const;  // mode-dependent center in world coordinates
  const Bbox& bbox() const;
  float radius() const;
  void update();                    // update HB if necessary
  GMesh* get_mesh();                // creates if non-existent
  void override_mesh(GMesh* mesh);  // nullptr ends override
 private:
  friend class objects;
  int _obn;
  Frame _t{Frame::identity()};  // transformation to stdf
  bool _vis{true};              // is visible
  bool _def{false};             // is non-empty
  Point _pavg;                  // estimated centroid
  Bbox _bbox{Point(0.f, 0.f, 0.f), Point(0.f, 0.f, 0.f)};
  float _radius;             // estimated object radius
  bool _needs_update{true};  // wants HB update
  Vec3<Stat> _stat_coord;    // statistics on each coordinate
  unique_ptr<GMesh> _mesh;
  GMesh* _override_mesh{nullptr};
};

class objects {
 public:
  static constexpr int MAX = 2048;
  objects() { for_int(i, MAX) _ob[i]._obn = i; }
  bool legal(int obn) { return obn >= 0 && obn < MAX; }
  object& operator[](int obn) { return (assertx(legal(obn)), _ob[obn]); }
  void copy(int obf, int obt);
  int first{1};  // 0 if object 0 is defined using -eyeob
  int last{0};   // last object number defined
 private:
  Vec<object, MAX> _ob;
};
extern objects g_obs;

// G3d
void UpdateFrame(const ObjectFrame& object_frame);
void ExpandStateFilename();
void UpdateOb1Bbox(const Bbox& bbox);

// G3devent
bool KeyPressed(const string& s);
void ButtonPressed(int butnum, bool pressed, bool shift, const Vec2<float>& yxf);
void WheelTurned(float v);
void InputArrived();
void DoJump();

// G3ddraw
void Applyq(const Frame& tq);
void Draw();
void ShowInfo();
void RecomputeSharpEdges(GMesh& mesh);
void ClearSubMesh();
void Dolly(const Vec2<float>& yxq);
void update_lod();

// G3dio
void ReadFiles(bool during_init);
void ReadInput(bool during_init);
void CloseIfOpen();
void WriteOutput();

}  // namespace g3d

#endif  // MESH_PROCESSING_G3DOGL_G3D_H_
