// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#ifndef MESH_PROCESSING_G3DOGL_HB_H_
#define MESH_PROCESSING_G3DOGL_HB_H_

#include "libHh/Array.h"
#include "libHh/Flags.h"
#include "libHh/Geometry.h"

namespace g3d { extern const hh::FlagMask mflag_ok; }

namespace hh {

class A3dElem;
class GMesh;
class PMesh;
struct PMeshInfo;
struct PlyMesh;
class RFile;

namespace HB {

// Screen coordinate system is (y = 0, x = 0) top left to (y = 1, x = 1) bottom right.
// Hither and yonder planes may be 0.f (disabled).

// Returns success.
bool init(Array<string>& aargs,
          bool (*pfkeyp)(const string& s),  // Returns true if handled.
          void (*pfbutp)(int butnum, bool pressed, bool shift, const Vec2<float>& yx), void (*pfwheel)(float v),
          void (*pfdraw)());

// Call after init() and before open():
void set_window_title(string s);
void before_window(void (*pfbefore)());  // Call before open().
void open();

// Call after init():
void watch_fd0(void (*pfinpu)());

// Call after open():
void quit();  // The user requests open() to return.
void redraw_later();
void redraw_now();
Vec2<int> get_extents();
std::optional<Vec2<float>> get_pointer();  // (0, 0)=(top, left).
void set_camera(const Frame& p_real_t, float p_real_zoom, const Frame& p_view_t, float p_view_zoom);
float get_hither();
float get_yonder();
void set_hither(float h);
void set_yonder(float y);
void set_current_object(int obn);  // A hook for lighting specifics.
void update_seg(int segn, const Frame& frame, bool vis);
void draw_space();
bool special_keypress(char ch);  // Returns true if recognized.
string show_info();              // Info line state string.
// Like key_press(), but a key name may have modifiers, as in "<C-l>"; see HwBase::simulate_key_press().
void simulate_key_press(const string& s);

struct VdcResult {
  float zs;
  std::optional<Vec2<float>> xys;  // Encodes: in_front ? (xs, ys) : {}.
};
VdcResult vdc_from_world(const Point& pi);
Vector view_ray(const Vec2<float>& yx);  // Camera-frame direction (with x = 1) through the window location yx.
// Whether surface_point() is available, i.e., whether the renderer keeps a depth buffer.
[[nodiscard]] bool has_depth_buffer();
// The world location of the surface visible at window location yx in the last frame drawn (read from the depth
//  buffer), or {} if no surface is visible there.
[[nodiscard]] std::optional<Point> surface_point(const Vec2<float>& yx);

void draw_segment(const Vec2<float>& yx1, const Vec2<float>& yx2);
Vec2<int> get_font_dims();  // Height, width.
void draw_text(const Vec2<float>& yx, const string& s);
void draw_row_col_text(const Vec2<int>& yx, const string& s);
void clear_segment(int segn);
void open_segment(int segn);
void segment_add_object(const A3dElem& el);
void segment_add_pmesh(unique_ptr<PMesh> pmesh);  // A progressive mesh, shown at a level of detail.
// A progressive mesh file, whose vsplits may be read lazily (as its level of detail increases); returns its header.
PMeshInfo segment_add_pmesh_file(unique_ptr<RFile> file);
void segment_add_ply(unique_ptr<PlyMesh> ply_mesh);  // A polygon mesh or point set read from a PLY file.
void close_segment();
void segment_attach_mesh(int segn, GMesh* mesh);
void make_segment_link(int oldsegn, int newsegn);
void segment_morph_mesh(int segn, float finterp);
void reload_textures();
void flush();
void beep();
int id();
void* escape(int code, void* data);

}  // namespace HB

}  // namespace hh

#endif  // MESH_PROCESSING_G3DOGL_HB_H_
