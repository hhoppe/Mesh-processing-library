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

// Mode of control.
enum class ERatemode { move, position, step };
extern ERatemode ratemode;
enum class EFlightmode { none, fly, flight, automatic, bobble };
extern EFlightmode flightmode;
extern int cob;                 // Current object to transform.
extern bool eye_move;           // Move the eyepoint instead of the object.
extern bool object_mode;        // Transform in the observer-relative frame.
extern float ddistance;         // Amplitude of displacements.
extern bool expo;               // Exponential displacement as a function of mouse.
extern bool geomorph;           // Use smooth transitions in lod.
extern bool globemode;          // Virtual sphere interface.
extern bool sizemode;           // Resize the object.
extern bool viewmode;           // Apply changes to the tview frame.
extern bool editmode;           // Edit mesh vertices.
extern bool keep_active;        // Keep motion going through button release.
extern bool auto_level;         // Keep the view level at all times.
extern bool mode_centroid;      // Use the centroid instead of the origin.
extern int want_jump;           // Keep doing 'j' if appending an object (0, 1, 2).
extern bool auto_hither;        // Set the hither distance automatically.
extern int timingtest_nframes;  // Count down.
extern bool play;               // Cycle through loaded objects.

constexpr int full_timingtest_nframes = 100;

// Mode of display.
extern int obview;  // Object frame to use for the view.
extern int info;    // Infoline (1 = small, 2 = verbose).

// Mode of stream input.
extern bool input;              // Look at input (== watch_fd0 state).
extern bool asynchronousinput;  // Update between EndFrames.
extern bool keep_stdin_open;    // Do not close stdin even if no input.
extern bool killeof;            // Kill g3d when EOF is read.

// Mode of stream output.
extern bool output;   // Print the frame on stdout.
extern bool obinary;  // Output format.

// State for window input.
extern float fchange;            // Time in seconds since the last refresh.
extern bool spacekill;           // Kill g3d when space is hit.
extern bool cur_needs_redraw;    // The current window calls for a redraw.
extern bool prev_needed_redraw;  // The previous window called for a redraw.

struct SelectedVertex {
  int obn;
  GMesh* mesh;
  Vertex v;
};

struct SelectedEdge {
  int obn;
  GMesh* mesh;
  Edge e;
  Point inter;
};

struct SelectedFace {
  int obn;
  GMesh* mesh;
  Face f;
};

extern int button_active;  // 0 = no, 1-3 = which button.
struct Selected {
  bool shift;
  Vec2<float> yxpressed;  // Location where the button was initially pressed.
  Vec2<float> yx;         // Current location.
  Vec2<float> yxio;       // Change in location of the button, miscellaneous scales.
  Vec2<float> yxfo;
  std::optional<SelectedVertex> selected_vertex;
  Frame frel;  // For !object_mode, change of axis transform.
};
extern Selected selected;

// Viewing transforms.
extern Frame tview;  // View offset.
extern float zoom;   // Equals tan(angle_of_view).

// Statistics.
extern bool iostat;           // Show stats on I/O.
extern int num_input_frames;  // # input frames read in current frame
extern bool timestat;         // Show stats on frame rate.

// Miscellaneous.
extern bool terse;              // Let g3d be terse.
extern bool tried_input;        // Got an input event for this screen.
extern string statefile;        // Name of the stateg3d file.
extern string caption;          // String to put at the bottom of the window.
extern string keystring;        // The -key strings concatenated together.
extern Array<string> g_aargs1;  // Not including argv0.
extern bool ob1_updated;

// Mesh manipulation.
extern float anglethresh;  // Dihedral angle threshold.

// Level of detail.
extern bool lod_mode;
extern float lod_level;

extern float override_frametime;  // If nonzero, a constant frame time.

extern Point rec_point;

class object {
 public:
  void clear();  // Re-initialize for definition.
  void enter_point(const Point& p);
  void update_stats();
  [[nodiscard]] bool defined() const;
  [[nodiscard]] bool visible() const;
  [[nodiscard]] const Frame& t() const;
  void set_vis(bool i);
  [[nodiscard]] Frame& tm();
  [[nodiscard]] const Point& center() const;  // Mode-dependent center in world coordinates.
  [[nodiscard]] const Bbox<float, 3>& bbox() const;
  [[nodiscard]] float radius() const;
  void update();      // Update HB if necessary.
  GMesh* get_mesh();  // Creates it if non-existent.
 private:
  friend class objects;
  int _obn;
  Frame _t{Frame::identity()};  // Transformation to stdf.
  bool _vis{true};              // Is it visible?
  bool _def{false};             // Is it non-empty?
  Point _pavg;                  // Estimated centroid.
  Bbox<float, 3> _bbox{Point(0.f, 0.f, 0.f), Point(0.f, 0.f, 0.f)};
  float _radius;             // Estimated object radius.
  bool _needs_update{true};  // Wants an HB update.
  Vec3<Stat> _stat_coord;    // Statistics on each coordinate.
  unique_ptr<GMesh> _mesh;
};

class objects {
 public:
  static constexpr int MAX = 2048;
  objects() { for_int(i, MAX) _ob[i]._obn = i; }
  [[nodiscard]] bool legal(int obn) { return obn >= 0 && obn < MAX; }
  [[nodiscard]] object& operator[](int obn) { return assertx(legal(obn)), _ob[obn]; }
  void copy(int obf, int obt);
  int first{1};  // 0 if object 0 is defined using -eyeob.
  int last{0};   // Last object number defined.
 private:
  Vec<object, MAX> _ob;
};
extern objects g_obs;

// G3d
void UpdateFrame(const ObjectFrame& object_frame);
void ExpandStateFilename();
void UpdateOb1Bbox(const Bbox<float, 3>& bbox);

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
void Dolly(const Vec2<float>& yxq);
void update_lod();

// G3dio
void ReadFiles(bool during_init);
void ReadInput(bool during_init);
void CloseIfOpen();
void WriteOutput();

}  // namespace g3d

#endif  // MESH_PROCESSING_G3DOGL_G3D_H_
