// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHWIN_HWBASE_H_
#define MESH_PROCESSING_LIBHWIN_HWBASE_H_

#if defined(GL_VERSION)  // OpenGL
#if defined(_WIN32)
#include "GL/glext.h"  // possibly use local file because Windows does not come with it.
#else                  // Unix
// #define GL_GLEXT_PROTOTYPES 1 // would obviate need for USE_GL_EXT() except to test presence of extension
#include <GL/glext.h>
#include <GL/glx.h>  // glXGetProcAddress()
#endif
#endif  // defined(GL_VERSION)

#include "Array.h"
#include "FileIO.h"  // file_exists()
#include "Image.h"
#include "RangeOp.h"
#include "StringOp.h"
#include "Vec.h"
#include "Vector4.h"

namespace hh {

// Base class for abstracting the windowing interface (X Windows and Win32).
// Window coordinates have origin at upper left.  Floating-point coordinates have pixels at half integers.
class HWbase : noncopyable {
 public:
  HWbase() {}
  virtual ~HWbase() {}
  bool init(Array<string>& aargs) { return init_aux(aargs); }  // ret: success

  // Callbacks:
  virtual bool key_press(string s) = 0;                                          // ret: handled
  virtual void button_press(int butnum, bool pressed, const Vec2<int>& yx) = 0;  // 1=L, 2=M, 3=R, 4=B, 5=F
  virtual void wheel_turn(float v) { dummy_use(v); }                             // abs(v) == 1.f for one click turn
  virtual void draw_window(const Vec2<int>& dims) = 0;  // must late-call clear_window() before drawing
  virtual void drag_and_drop(CArrayView<string> filenames) { dummy_use(filenames); }
  virtual void input_received() {}

  // call after init() but before open():
  void set_default_background(string s) { assertx(_state == EState::init), _default_background = std::move(s); }
  void set_default_foreground(string s) { assertx(_state == EState::init), _default_foreground = std::move(s); }
  void set_default_geometry(string s) { assertx(_state == EState::init), _default_geometry = std::move(s); }
  virtual void set_double_buffering(bool newstate) = 0;  // (also allowed after open() in HWX without HH_OGLX)

  // call anytime after init():
  virtual void set_window_title(string s) = 0;  // set text for window title bar
  virtual void beep() = 0;
  void watch_fd0(bool b) { assertx(_state != EState::uninit), _watch_fd0 = b; }
  void redraw_later();
  void redraw_now();
  void quit() { _update = EUpdate::quit; }
  virtual void open() = 0;

  // call after open():
  virtual bool suggests_stop() = 0;             // ret: HW requests program to stop drawing
  virtual bool get_pointer(Vec2<int>& yx) = 0;  // ret success
  enum class EModifier { shift, control, alt };
  virtual bool get_key_modifier(EModifier modifier) = 0;
  virtual void set_color_to_foreground() = 0;
  virtual void set_color(const Pixel& pix) = 0;
  Vec2<int> get_font_dims() { return _font_dims; }
  Vec2<int> get_window_dims() { return _win_dims; }
  enum class EStyle { regular, shadowed, boxed };
  void draw_text(const Vec2<int>& yx, const string& s, EStyle style = EStyle::regular,
                 const Pixel& back_color = Pixel::black(), bool wrap = true);
  virtual void fill_polygon(CArrayView<Vec2<float>> points) = 0;  // (y, x) order; polygon assumed convex
  void fill_rectangle(const Vec2<float>& top_left, const Vec2<float>& bot_right);  // (y, x) coordinates
  void draw_segment(const Vec2<float>& yx1, const Vec2<float>& yx2);               // coordinates in pixel units
  void draw_point(const Vec2<float>& yx);
  Vec2<int> window_position_yx() const { return _win_pos; }
  virtual Vec2<int> get_max_window_dims() = 0;          // (ny, nx)
  virtual void resize_window(const Vec2<int>& yx) = 0;  // resize to new width and height
  virtual bool is_fullscreen() = 0;
  virtual void make_fullscreen(bool b) = 0;  // make the window fullscreen if b
  virtual void grab_focus() {}

  // call within draw_window():
  virtual void clear_window() = 0;
  void process_keystring(string& keystring);  // makes calls to key_press() and clears keystring

  // query user for values:
  bool query(const Vec2<int>& yx, string prompt, string& buffer);  // ret: true with <enter>, false with <esc>
  bool query(const Vec2<int>& yx, string prompt, float& f);
  bool query(const Vec2<int>& yx, string prompt, int& i);
  virtual Array<string> query_open_filenames(const string& hint_filename);
  virtual string query_save_filename(const string& hint_filename, bool force = false);  // ret: "" if canceled
  bool within_query() const { return _within_query; }

  // buffering:
  virtual void hard_flush() = 0;          // synchronize screen
  virtual void begin_draw_visible() = 0;  // force update to visible buffer
  virtual void end_draw_visible() = 0;
  virtual void wake_up() {}  // called from an asynchronous client thread to force redraw

  // clipboard:
  virtual bool copy_image_to_clipboard(const Image&) {
    Warning("clipboard not implemented");
    return false;
  }
  virtual bool copy_clipboard_to_image(Image&) {
    Warning("clipboard not implemented");
    return false;
  }

 private:
  friend class HW;  // grant access to HW but not to any DerivedHW
  string _default_background{"white"};
  string _default_foreground{"black"};
  string _default_geometry{"200x200+100+0"};
  string _argv0;
  string _window_title{"noname"};
  bool _watch_fd0{false};
  enum class EState { uninit, init, open } _state{EState::uninit};
  enum class EUpdate { nothing, quit, redrawlater, redrawnow } _update{EUpdate::nothing};
  bool _async{false};
  bool _gotevent{false};
  Array<Vec2<Vec2<float>>> _ar_seg;  // {{y1, x1}, {y2, x2}}
  Array<Vec2<float>> _ar_point;      // {y, x}
  Vec2<int> _win_dims{0, 0};         // y, x : size of client area
  Vec2<int> _win_pos{0, 0};          // y, x : upper-left position of client area
  bool _exposed{false};
  bool _first_draw{true};
  bool _is_glx_dbuf{false};
  bool _is_keyintr{false};
  string _hwkey;
  float _hwdelay{2.f};
  bool _query{false};
  Vec2<int> _query_yx;
  string _query_prompt;
  string _query_buffer;
  bool _query_success;
  bool _within_query;
  int _hwdebug{0};
  string _user_geometry;
  string _backcolor;
  string _forecolor;
  string _offscreen;                    // name of output file to render to, without any visible window
  Pixel _color_foreground{0, 0, 0, 0};  // 24-bit foreground/drawing color
  Pixel _color_background{0, 0, 0, 0};  // 24-bit background/clear color
  bool _bigfont{false};
  Vec2<int> _font_dims{0, 0};  // height, width
  int _listbase_font{-1};

  virtual bool init_aux(Array<string>& aargs) = 0;
  virtual void start_hwkey() = 0;
  virtual void end_hwkey() = 0;
  virtual void flush_seg() = 0;
  virtual void flush_point() = 0;
  virtual bool loop() = 0;  // ret: should quit
  virtual void draw_text_internal(const Vec2<int>& yx, const string& s) = 0;

  void soft_flush();    // flush output buffers
  void soft_discard();  // discard buffered segments and points
  void query_keypress(string s);
  void handle_keyintr();
#if defined(GL_VERSION)
  void clear_window_ogl();
  void draw_text_ogl(const Vec2<int>& yx, const string& s);
  void fill_polygon_ogl(CArrayView<Vec2<float>> points);
  void flush_seg_ogl();
  void flush_point_ogl();
#endif
};

// Return Pixel given string of the form "#RRGGBB" or "#RRGGBBAA" in hexadecimal.
Pixel parse_color(const string& scolor);

//----------------------------------------------------------------------------

inline void HWbase::redraw_later() {
  if (0) assertx(_state != EState::uninit);  // could occur in a background thread after quit()
  if (_update == EUpdate::nothing) _update = EUpdate::redrawlater;
}

inline void HWbase::redraw_now() {
  assertx(_state != EState::uninit);
  if (_update == EUpdate::nothing || _update == EUpdate::redrawlater) _update = EUpdate::redrawnow;
}

inline void HWbase::draw_text(const Vec2<int>& yx, const string& s, EStyle style, const Pixel& back_color, bool wrap) {
  if (!s.size()) return;
  // use uchar{127} to render all non-ascii characters.
  auto func_is_nonascii = [](const string& ss) {
    for_size_t(i, ss.size()) {
      uchar ch = static_cast<uchar>(ss[i]);
      if (!(ch >= 32 && ch <= 127)) return true;
    }
    return false;
  };
  if (func_is_nonascii(s)) {
    Warning("Non-ASCII characters present in string will not be displayed properly in window");
    string s2;
    for_size_t(i, s.size()) {
      uchar ch = static_cast<uchar>(s[i]);
      s2 += static_cast<uchar>(ch >= 32 && ch <= 126 ? s[i] : 127);
    }
    return draw_text(yx, s2, style, back_color, wrap);
  }
  if (wrap) {
    assertx(_font_dims[1] > 0);
    assertx(_win_dims[1] > 0);
    if (yx[1] + narrow_cast<int>(s.size()) * _font_dims[1] > _win_dims[1]) {
      int nch = (_win_dims[1] - yx[1]) / _font_dims[1];
      if (nch > 0) {
        draw_text(yx, s.substr(0, nch), style, back_color, wrap);
        assertx(nch < narrow_cast<int>(s.size()));
        draw_text(V(yx[0] + _font_dims[0], 0), s.substr(nch), style, back_color, wrap);
      }
      return;
    }
  }
  if (style == EStyle::shadowed) {
    set_color(back_color);
    for_intL(xd, -1, 2) for_intL(yd, -1, 2) {
      if (xd || yd) draw_text_internal(yx + V(yd, xd), s);
    }
    set_color_to_foreground();
  } else if (style == EStyle::boxed) {
    set_color(back_color);
    fill_rectangle(convert<float>(yx + V(4, -2)),
                   convert<float>(yx + _font_dims * V(1, narrow_cast<int>(s.size())) + V(4, 2)));
    set_color_to_foreground();
  }
  draw_text_internal(yx, s);
}

inline void HWbase::fill_rectangle(const Vec2<float>& top_left, const Vec2<float>& bot_right) {
  auto top_right = V(top_left[0], bot_right[1]);
  auto bot_left = V(bot_right[0], top_left[1]);
  fill_polygon(V(top_left, bot_left, bot_right, top_right));
}

inline void HWbase::draw_segment(const Vec2<float>& yx1, const Vec2<float>& yx2) {
  ASSERTX(_state == EState::open);
  _ar_seg.push(V(yx1, yx2));
  if (_ar_seg.num() >= 256) flush_seg();
}

inline void HWbase::draw_point(const Vec2<float>& yx) {
  ASSERTX(_state == EState::open);
  _ar_point.push(yx);
  if (_ar_point.num() >= 1024) flush_point();
}

// Only reads keys from _hwkey
inline void HWbase::handle_keyintr() {
  if (!_is_keyintr) return;
  _is_keyintr = false;
  bool skip = false;
  int is_group = 0;
  for (;;) {
    const char ch = _hwkey[0];
    _hwkey.erase(0, 1);
    string press = string(1, ch);
    if (ch == '<') {
      auto j = _hwkey.find('>');
      if (j == string::npos) assertnever("No matching '>' in string '" + _hwkey + "'");
      press = ch + _hwkey.substr(0, j + 1);
      _hwkey = _hwkey.substr(j + 1);
    } else if (ch == '\\' && _hwkey != "") {
      if (_hwkey[0] == 'n') {
        _hwkey.erase(0, 1);
        press = string(1, 13);
      } else if (_hwkey[0] == 'c') {
        quit();
        break;
      } else if (_hwkey[0] == '1') {
        _hwkey.erase(0, 1);
        skip = true;
      } else if (_hwkey[0] >= '2' && _hwkey[0] <= '9') {
        _hwkey[0] -= 1;
        _hwkey.insert(0, "\\");  // put the symbol back
        skip = true;
      } else {
        press = string(1, _hwkey[0]);
        _hwkey.erase(0, 1);
      }
    } else if (ch == '(' && _hwkey != "") {
      is_group = 1;
      continue;
    } else if (ch == ')') {
      is_group = 2;
    }
    if (_hwkey == "") end_hwkey();
    if (skip) break;
    if (is_group == 2) break;
    key_press(press);
    if (!is_group) break;
  }
}

inline void HWbase::soft_flush() {
  assertx(_state == EState::open);
  flush_seg();
  flush_point();
}

inline void HWbase::soft_discard() {
  assertx(_state == EState::open);
  _ar_seg.init(0);
  _ar_point.init(0);
}

inline void HWbase::process_keystring(string& keystring) {
  for (string::size_type i = 0;;) {
    char ch = keystring[i];
    if (!ch) {
      keystring = "";
      return;
    }
    if (ch == '<') {
      auto j = keystring.find('>', i + 1);
      if (j == string::npos) assertnever("No matching '>' in string '" + keystring.substr(i) + "'");
      key_press(keystring.substr(i, j + 1 - i));
      i = j + 1;
    } else {
      key_press(string(1, ch));
      i++;
    }
  }
}

inline void HWbase::query_keypress(string s) {
  if (s == "<left>") s = "\b";
  char ch = s[0];
  if (ch == '\b') {                              // <backspace>/C-h key (== uchar{8} == 'H' - 64)
    if (get_key_modifier(EModifier::control)) {  // C-<backspace> deletes word
      if (_query_buffer != "" && _query_buffer.back() == '/') _query_buffer.pop_back();
      while (_query_buffer != "" && _query_buffer.back() == ' ') _query_buffer.pop_back();
      while (_query_buffer != "" && !contains(" /", _query_buffer.back())) _query_buffer.pop_back();
    } else {
      if (_query_buffer != "") _query_buffer.pop_back();
    }
    redraw_later();
  } else if (ch == '\033') {  // <esc> key (== uchar{27})
    _query = false;
    redraw_later();
  } else if (ch == 'U' - 64) {  // C-u deletes entire line
    _query_buffer = "";
    redraw_later();
  } else if (ch == '\r') {  // <enter>/<ret> key (== uchar{13} == 'M' - 64)
    _query = false;
    _query_success = true;
    redraw_later();
  } else if (s.size() > 1 || !(static_cast<uchar>(ch) >= 32 && static_cast<uchar>(ch) <= 127)) {
    beep();
  } else {
    _query_buffer += ch;
    redraw_later();
  }
}

inline bool HWbase::query(const Vec2<int>& yx, string prompt, string& buffer) {
  assertx(_state == EState::open);
  _query = true;
  _query_yx = yx;
  _query_prompt = std::move(prompt);
  _query_buffer = buffer;
  _query_success = false;
  _within_query = true;
  redraw_later();
  while (_query) {
    if (loop()) break;
  }
  _within_query = false;
  buffer = _query_buffer;
  return _query_success;
}

inline bool HWbase::query(const Vec2<int>& yx, string prompt, float& f) {
  string s = sform("%g", f);
  bool success = query(yx, std::move(prompt), s);
  if (success) f = static_cast<float>(atof(s.c_str()));
  return success;
}

inline bool HWbase::query(const Vec2<int>& yx, string prompt, int& i) {
  string s = sform("%d", i);
  bool success = query(yx, std::move(prompt), s);
  if (success) i = to_int(s);
  return success;
}

inline Array<string> HWbase::query_open_filenames(const string& hint_filename) {
  string filename = hint_filename;
  if (query(V(20, 10), "Open file:", filename)) return {std::move(filename)};
  return {};
}

inline string HWbase::query_save_filename(const string& hint_filename, bool force) {
  string filename = hint_filename;
  if (query(V(20, 10), "Save file:", filename)) {
    bool ok = force || !file_exists(filename);
    // TODO: verify that file can be written
    if (!ok) {
      string s = "yes";
      if (query(V(20, 10), "File exists!  Overwrite (or <esc>) ? ", s) && s == "yes") ok = true;
    }
    return ok ? filename : "";
  }
  return "";
}

// Convert a hexadecimal digit to an integer (0..15).
inline uint8_t parse_hexa_nibble(char ch) {
  if (ch >= '0' && ch <= '9') {  // or std::isdigit()
    return narrow_cast<uint8_t>(ch - '0');
  } else if (ch >= 'A' && ch <= 'F') {
    return narrow_cast<uint8_t>(10 + (ch - 'A'));
  } else if (ch >= 'a' && ch <= 'f') {
    return narrow_cast<uint8_t>(10 + (ch - 'a'));
  } else {
    assertnever(sform("Character '%c' is not hexadecimal (int{ch}=%d)", ch, int{ch}));
  }
}

// Convert a nibble (pair of hexadecimal digits) to an integer (0..255).
inline uint8_t parse_hexa_nibble_pair(const char* sbyte) {
  return parse_hexa_nibble(sbyte[0]) * 16u + parse_hexa_nibble(sbyte[1]);
}

inline Pixel parse_color(const string& scolor) {
  assertx(scolor != "");
  const char* s = scolor.c_str();
  if (scolor[0] == '#' && scolor.size() == 7) {
    return Pixel(parse_hexa_nibble_pair(s + 1), parse_hexa_nibble_pair(s + 3), parse_hexa_nibble_pair(s + 5));
  } else if (scolor[0] == '#' && scolor.size() == 9) {
    return Pixel(parse_hexa_nibble_pair(s + 1), parse_hexa_nibble_pair(s + 3), parse_hexa_nibble_pair(s + 5),
                 parse_hexa_nibble_pair(s + 7));
  } else if (scolor == "white") {
    return parse_color("#FFFFFF");
  } else if (scolor == "black") {
    return parse_color("#000000");
  } else if (scolor == "yellow") {
    return parse_color("#FFFF40");
  } else if (scolor == "red") {
    return parse_color("#FF0000");
  } else if (scolor == "green") {
    return parse_color("#00FF00");
  } else if (scolor == "blue") {
    return parse_color("#0000FF");
  } else if (scolor == "hhblue") {
    return parse_color("#5987B3");
  } else if (1 && to_int(s) > 0) {
    Warning("Old int32-style color specification; now use #RRGGBBAA");
    unsigned u = unsigned(to_int(s));
    return Pixel((u >> 0) & 255, (u >> 8) & 255, (u >> 16) & 255, (u >> 24) & 255);
  } else {
    assertnever("Cannot parse color '" + scolor + "'");
  }
}

//----------------------------------------------------------------------------

#if defined(GL_VERSION)  // OpenGL

// See:
// http://open.gl/textures  really nice!
// http://www.clockworkcoders.com/oglsl/tutorial8.htm
// http://docs.gl/
// http://www.opengl-tutorial.org/beginners-tutorials/tutorial-2-the-first-triangle/   shader program

// c:/Program Files (x86)/Windows Kits/8.1/Include/um/gl/GL.h
// ~/src/G3dOGL/glext.h

// Check if there are any outstanding OpenGL errors and if so report them.
// For debugging, sprinkle:  ASSERTX(!gl_report_errors());
bool gl_report_errors();  // ret: true if errors (only call after init() and before open() returns)

// Return a string containing all the supported OpenGL extensions.
const string& gl_extensions_string();

#define USE_GL_EXT_MAYBE_AUX(func, type, GetProc) \
  using Type##func = type;                        \
  static Type##func func;                         \
  static bool is_init_##func;                     \
  if (!is_init_##func) {                          \
    is_init_##func = true;                        \
    func = Type##func(GetProc);                   \
  }                                               \
  HH_EAT_SEMICOLON

#if defined(_WIN32)
#define USE_GL_EXT_MAYBE(func, type) USE_GL_EXT_MAYBE_AUX(func, type, wglGetProcAddress(#func))
#elif 1  // Unix / X windows
#define USE_GL_EXT_MAYBE(func, type) \
  USE_GL_EXT_MAYBE_AUX(func, type, glXGetProcAddress(reinterpret_cast<const GLubyte*>(#func)))
#else  // Mac OSX
#define USE_GL_EXT_MAYBE(func, type) USE_GL_EXT_MAYBE_AUX(func, type, NSGLGetProcAddress(#func))
#endif

// See ~/src/G3dOGL/glext.h
#define USE_GL_EXT(func, type)  \
  USE_GL_EXT_MAYBE(func, type); \
  assertx(func)

inline bool gl_report_errors() {
  GLenum v = glGetError();
  if (!v) return false;  // GL_NO_ERROR == 0
  for_int(i, 10) {
    string s;
    {
      // GL_INVALID_FRAMEBUFFER_OPERATION is absent in early OpenGL
      switch (v) {
        case GL_INVALID_ENUM: s = "GL_INVALID_ENUM"; break;
        case GL_INVALID_VALUE: s = "GL_INVALID_VALUE"; break;
        case GL_INVALID_OPERATION: s = "GL_INVALID_OPERATION"; break;
        case GL_STACK_OVERFLOW: s = "GL_STACK_OVERFLOW"; break;
        case GL_STACK_UNDERFLOW: s = "GL_STACK_UNDERFLOW"; break;
        case GL_OUT_OF_MEMORY: s = "GL_OUT_OF_MEMORY"; break;
        default: s = "unknown error";
      }
    }
    showf("OpenGL error: %s\n", s.c_str());
    v = glGetError();
    if (!v) return true;
  }
  assertnever("Likely an infinite loop; perhaps OpenGL device context is already shutdown");
}

inline const string& gl_extensions_string() {
  // e.g. see http://www.geeks3d.com/20100722/tips-how-to-get-the-list-of-the-opengl-extensions-with-a-core-profile/
  // Remote Desktop (even on Win10):
  //  glGetString(GL_VERSION) = 1.1.0
  //  gl_extensions_string() = GL_WIN_swap_hint GL_EXT_bgra GL_EXT_paletted_texture
  static string s_string;
  static bool is_init = false;
  if (!is_init) {
#if 0
    int n;
    glGetIntegerv(GL_NUM_EXTENSIONS, &n);
    for_int(i, n) s_string += string(glGetStringi(GL_EXTENSIONS, i)) + " ";  // glGetStringi() only in GL 3.0
    is_init = true;
#else
    const char* s = reinterpret_cast<const char*>(glGetString(GL_EXTENSIONS));
    if (!s) {
      if (1) Warning("glGetString(GL_EXTENSIONS) failed once");
      void(glGetError());
    } else {
      s_string = s;
      s_string += " ";
      is_init = true;
    }
#endif
  }
  return s_string;
}

inline void HWbase::clear_window_ogl() {
  glViewport(0, 0, _win_dims[1], _win_dims[0]);
  if (_is_glx_dbuf) glDrawBuffer(GL_BACK);
  if (_first_draw) {
    _first_draw = false;
    glDrawBuffer(GL_FRONT_AND_BACK);
  }
  {
    Vector4 v(_color_background);
    glClearColor(v[0], v[1], v[2], 1.f);
  }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (_is_glx_dbuf)
    glDrawBuffer(GL_BACK);
  else
    glDrawBuffer(GL_FRONT);
  set_color_to_foreground();
}

inline void HWbase::draw_text_ogl(const Vec2<int>& yx, const string& s) {
  glListBase(_listbase_font);
  USE_GL_EXT_MAYBE(glWindowPos2i, PFNGLWINDOWPOS2IPROC);  // not supported on Remote Desktop
  if (glWindowPos2i) {
    glWindowPos2i(yx[1],
                  _win_dims[0] - yx[0] - _font_dims[0]);  // reverse y; not clip-tested, so raster position valid
    glCallLists(narrow_cast<int>(s.size()), GL_UNSIGNED_BYTE, reinterpret_cast<const uchar*>(s.c_str()));
  } else {
    glRasterPos2i(yx[1], yx[0] + _font_dims[0]);  // clipped, so raster position may be invalid
    glCallLists(narrow_cast<int>(s.size()), GL_UNSIGNED_BYTE, reinterpret_cast<const uchar*>(s.c_str()));
  }
}

inline void HWbase::fill_polygon_ogl(CArrayView<Vec2<float>> points) {
  glBegin(points.num() == 3 ? GL_TRIANGLES : points.num() == 4 ? GL_QUADS : GL_POLYGON);
  for (const Vec2<float>& p : points) {
    glVertex2f(p[1], p[0]);
  }
  glEnd();
}

inline void HWbase::flush_seg_ogl() {
  if (!_ar_seg.num()) return;
  glBegin(GL_LINES);
  for (const auto& s : _ar_seg) {
    // glVertex2fv(s[0].rev().data());
    // glVertex2fv(s[1].rev().data());
    glVertex2f(s[0][1], s[0][0]);
    glVertex2f(s[1][1], s[1][0]);
  }
  glEnd();
  _ar_seg.init(0);
}

inline void HWbase::flush_point_ogl() {
  if (!_ar_point.num()) return;
  glBegin(GL_POINTS);
  for (const auto& p : _ar_point) {
    // glVertex2fv(p.rev().data());
    glVertex2f(p[1], p[0]);
  }
  glEnd();
  _ar_point.init(0);
}

#endif  // defined(GL_VERSION)

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHWIN_HWBASE_H_
