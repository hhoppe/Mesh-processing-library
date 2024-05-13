// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_G3DOGL_HB_H_
#define MESH_PROCESSING_G3DOGL_HB_H_

#include <optional>

#include "libHh/Array.h"
#include "libHh/Flags.h"
#include "libHh/Geometry.h"

namespace g3d {
extern const hh::FlagMask mflag_ok;
extern const hh::FlagMask vflag_ok;
extern const hh::FlagMask fflag_ok;
}  // namespace g3d

namespace hh {

class A3dElem;
class GMesh;

namespace HB {

// Screen coordinate system is (y = 0, x = 0) top left to (y = 1, x = 1) bottom right.
// Hither and yonder planes may be 0.f (disabled).

bool init(Array<string>& aargs,
          bool (*pfkeyp)(const string& s),  // ret: handled
          void (*pfbutp)(int butnum, bool pressed, bool shift, const Vec2<float>& yx), void (*pfwheel)(float v),
          void (*pfdraw)());  // ret: success

// call after init() and before open():
void set_window_title(string s);
void open();

// call after init():
void watch_fd0(void (*pfinpu)());

// call after open():
void quit();  // user requests open() to return
void redraw_later();
void redraw_now();
Vec2<int> get_extents();
bool get_pointer(Vec2<float>& yxf);  // ret: false if no info
void set_camera(const Frame& p_real_t, float p_real_zoom, const Frame& p_view_t, float p_view_zoom);
float get_hither();
float get_yonder();
void set_hither(float h);
void set_yonder(float y);
void set_current_object(int obn);  // hook for lighting specific.
void update_seg(int segn, const Frame& f, bool vis);
void draw_space();
bool special_keypress(char ch);                                              // ret: recognized
string show_info();                                                          // info line state string
std::pair<float, std::optional<Vec2<float>>> world_to_vdc(const Point& pi);  // [zs, pi_in_front ? (xs, ys) : {}]
void draw_segment(const Vec2<float>& yx1, const Vec2<float>& yx2);
Vec2<int> get_font_dims();  // height, width
void draw_text(const Vec2<float>& yx, const string& s);
void draw_row_col_text(const Vec2<int>& yx, const string& s);
void clear_segment(int segn);
void open_segment(int segn);
void segment_add_object(const A3dElem& el);
void close_segment();
void segment_attach_mesh(int segn, GMesh* mesh);
void make_segment_link(int oldsegn, int newsegn);
void segment_morph_mesh(int segn, float finterp);
void reload_textures();
void flush();
void beep();
int id();
void* escape(void* code, void* data);

}  // namespace HB

}  // namespace hh

#endif  // MESH_PROCESSING_G3DOGL_HB_H_
