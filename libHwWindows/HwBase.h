// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHWWINDOWS_HWBASE_H_
#define MESH_PROCESSING_LIBHWWINDOWS_HWBASE_H_

#include <atomic>

#if defined(GL_VERSION)  // OpenGL.
#if defined(_WIN32)
#include "GL/glext.h"  // Possibly use local file because Windows does not come with it.
#else                  // Unix.
// #define GL_GLEXT_PROTOTYPES 1 // would obviate need for USE_GL_EXT() except to test presence of extension
#include <GL/glext.h>
#include <GL/glx.h>  // glXGetProcAddress()
#endif
#endif  // defined(GL_VERSION)

#include "libHh/Array.h"
#include "libHh/FileIO.h"  // file_exists()
#include "libHh/Image.h"
#include "libHh/RangeOp.h"
#include "libHh/StringOp.h"
#include "libHh/Vec.h"
#include "libHh/Vector4.h"

namespace hh {

// Base class for abstracting the windowing interface (X Windows and Win32).
// Window coordinates have origin at upper left.  Floating-point coordinates have pixels at half integers.
class HwBase : noncopyable {
 public:
  HwBase() = default;
  bool init(Array<string>& aargs) { return init_aux(aargs); }  // Returns success.

  // Callbacks:
  virtual bool key_press(string s) = 0;                                          // Returns true if handled.
  virtual void button_press(int butnum, bool pressed, const Vec2<int>& yx) = 0;  // 1 = L, 2 = M, 3 = R, 4 = B, 5 = F.
  virtual void wheel_turn(float v) { dummy_use(v); }                             // abs(v) == 1.f for one click turn.
  virtual void draw_window(const Vec2<int>& dims) = 0;  // Must late-call clear_window() before drawing.
  virtual void drag_and_drop(CArrayView<string> filenames) { dummy_use(filenames); }
  virtual void input_received() {}

  // Call after init() but before open():
  void set_default_background(string s) { assertx(_state == EState::init), _default_background = std::move(s); }
  void set_default_foreground(string s) { assertx(_state == EState::init), _default_foreground = std::move(s); }
  void set_default_geometry(string s) { assertx(_state == EState::init), _default_geometry = std::move(s); }
  virtual void set_double_buffering(bool newstate) = 0;  // (also allowed after open() in libHwX without HH_OGLX)

  // Call anytime after init():
  virtual void set_window_title(string s) = 0;  // Set text for window title bar.
  virtual void beep() = 0;
  void watch_fd0(bool b) { assertx(_state != EState::uninit), _watch_fd0 = b; }
  void redraw_later();
  void redraw_now();
  void quit() { _update = EUpdate::quit; }
  virtual void open() = 0;

  // Call after open():
  virtual bool suggests_stop() = 0;  // Returns true if Hw requests the program to stop drawing.
  virtual std::optional<Vec2<int>> get_pointer() = 0;
  enum class EModifier { shift, control, alt };
  virtual bool get_key_modifier(EModifier modifier) = 0;
  virtual void set_color_to_foreground() = 0;
  virtual void set_color(const Pixel& pixel) = 0;
  Vec2<int> get_font_dims() { return _font_dims; }
  Vec2<int> get_window_dims() { return _win_dims; }
  enum class EStyle { regular, shadowed, boxed };
  void draw_text(const Vec2<int>& yx, const string& s, EStyle style = EStyle::regular,
                 const Pixel& back_color = Pixel::black(), bool wrap = true);
  virtual void fill_polygon(CArrayView<Vec2<float>> points) = 0;  // (y, x) order; polygon assumed convex
  void fill_rectangle(const Vec2<float>& top_left, const Vec2<float>& bot_right);  // (y, x) coordinates
  void draw_segment(const Vec2<float>& yx1, const Vec2<float>& yx2);               // Coordinates in pixel units.
  void draw_point(const Vec2<float>& yx);
  [[nodiscard]] Vec2<int> window_position_yx() const { return _win_pos; }
  [[nodiscard]] virtual Vec2<int> get_max_window_dims() = 0;  // (ny, nx)
  virtual void resize_window(const Vec2<int>& yx) = 0;        // Resize to new width and height.
  [[nodiscard]] virtual bool is_fullscreen() = 0;
  virtual void make_fullscreen(bool b) = 0;  // Make the window fullscreen if b.
  virtual void grab_focus() {}

  // Call within draw_window():
  virtual void clear_window() = 0;
  void process_keystring(string& keystring);  // Makes calls to key_press() and clears keystring.

  // Query the user for values; returns true with <enter>, false with <esc>.
  [[nodiscard]] bool query(const Vec2<int>& yx, string prompt, string& buffer);
  [[nodiscard]] bool query(const Vec2<int>& yx, string prompt, float& f);
  [[nodiscard]] bool query(const Vec2<int>& yx, string prompt, int& i);
  [[nodiscard]] virtual Array<string> query_open_filenames(const string& hint_filename);
  [[nodiscard]] virtual string query_save_filename(const string& hint_filename, bool force = false);  // "" if canceled
  [[nodiscard]] bool within_query() const { return _within_query; }
  [[nodiscard]] bool is_hidden() const { return _hidden; }              // -hidden (also implied by -offscreen)
  [[nodiscard]] bool is_offscreen() const { return _offscreen != ""; }  // -offscreen

  // Buffering:
  virtual void hard_flush() = 0;          // Synchronize screen.
  virtual void begin_draw_visible() = 0;  // Force update to visible buffer.
  virtual void end_draw_visible() = 0;
  virtual void wake_up() {}  // Called from an asynchronous client thread to force redraw.

  // Clipboard:
  virtual bool copy_image_to_clipboard(const Image&) {
    Warning("clipboard not implemented");
    return false;
  }
  [[nodiscard]] virtual std::optional<Image> copy_clipboard_to_image() {
    Warning("clipboard not implemented");
    return {};
  }

 protected:
  virtual ~HwBase() = default;

 private:
  friend class Hw;  // Grant access to Hw but not to any DerivedHw.
  string _default_background{"white"};
  string _default_foreground{"black"};
  string _default_geometry{"200x200+100+0"};
  string _argv0;
  string _window_title{"noname"};
  bool _watch_fd0{false};
  enum class EState { uninit, init, open } _state{EState::uninit};
  enum class EUpdate { nothing, quit, redrawlater, redrawnow };
  std::atomic<EUpdate> _update{EUpdate::nothing};
  bool _async{false};
  bool _gotevent{false};
  Array<Vec2<Vec2<float>>> _ar_seg;  // {{y1, x1}, {y2, x2}}
  Array<Vec2<float>> _ar_point;      // {y, x}
  Vec2<int> _win_dims{0, 0};         // Size of the client area (y, x).
  Vec2<int> _win_pos{0, 0};          // Upper-left position of the client area (y, x).
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
  string _offscreen;  // Name of output file to render to, without any visible window.

  bool _hidden{false};                               // Render into framebuffer objects instead of a visible window.
  int _hidden_samples{0};                            // Multisamples in the hidden framebuffer (0 for none).
  Vec2<int> _hidden_dims{0, 0};                      // Size of the allocated hidden framebuffers.
  Vec2<unsigned> _hidden_framebuffers{0u, 0u};       // Names of the drawn and resolved framebuffers.
  Vec3<unsigned> _hidden_renderbuffers{0u, 0u, 0u};  // Names of the drawn color, drawn depth, and resolved color.

  bool _nosetforeground{false};
  Pixel _color_foreground{0, 0, 0, 0};  // 24-bit foreground/drawing color.
  Pixel _color_background{0, 0, 0, 0};  // 24-bit background/clear color.
  bool _bigfont{false};
  Vec2<int> _font_dims{0, 0};  // Height, width.
  int _listbase_font{-1};

  virtual bool init_aux(Array<string>& aargs) = 0;
  virtual void start_hwkey() = 0;
  virtual void end_hwkey() = 0;
  virtual void flush_seg() = 0;
  virtual void flush_point() = 0;
  virtual bool loop() = 0;  // Returns true if it should quit.
  virtual void draw_text_internal(const Vec2<int>& yx, const string& s) = 0;

  void soft_flush();    // Flush output buffers.
  void soft_discard();  // Discard buffered segments and points.
  void query_keypress(string s);
  void handle_keyintr();
#if defined(GL_VERSION)
  void clear_window_ogl();
  void draw_text_ogl(const Vec2<int>& yx, const string& s);
  void fill_polygon_ogl(CArrayView<Vec2<float>> points);
  void flush_seg_ogl();
  void flush_point_ogl();
  void hidden_bind_framebuffer();
  void hidden_resolve_framebuffer();
  void hidden_write_image(const string& filename);
#endif
};

// Return Pixel given string of the form "#RRGGBB" or "#RRGGBBAA" in hexadecimal.
Pixel parse_color(const string& scolor);

//----------------------------------------------------------------------------

inline void HwBase::redraw_later() {
  if (0) assertx(_state != EState::uninit);  // Could occur in a background thread after quit().
  // Atomic check-then-set; may run on a background thread.
  EUpdate expected = EUpdate::nothing;
  _update.compare_exchange_strong(expected, EUpdate::redrawlater);
}

inline void HwBase::redraw_now() {
  assertx(_state != EState::uninit);
  if (_update == EUpdate::nothing || _update == EUpdate::redrawlater) _update = EUpdate::redrawnow;
}

inline void HwBase::draw_text(const Vec2<int>& yx, const string& s, EStyle style, const Pixel& back_color, bool wrap) {
  if (!s.size()) return;
  // Use uchar{127} to render all non-ascii characters.
  const auto func_is_nonascii = [](const string& ss) {
    return ranges::any_of(ss, [](char ch) { return !(uchar(ch) >= 32 && uchar(ch) <= 127); });
  };
  if (func_is_nonascii(s)) {
    Warning("Non-ASCII characters present in string will not be displayed properly in window");
    string s2;
    for (const size_t i : range(s.size())) {
      const uchar ch = uchar(s[i]);
      s2 += uchar(ch >= 32 && ch <= 126 ? s[i] : 127);
    }
    return draw_text(yx, s2, style, back_color, wrap);
  }
  if (wrap) {
    assertx(_font_dims[1] > 0);
    assertx(_win_dims[1] > 0);
    if (yx[1] + narrow_cast<int>(s.size()) * _font_dims[1] > _win_dims[1]) {
      const int nch = (_win_dims[1] - yx[1]) / _font_dims[1];
      if (nch > 0) {
        draw_text(yx, s.substr(0, nch), style, back_color, wrap);
        assertx(nch < narrow_cast<int>(s.size()));
        draw_text(V(yx[0] + _font_dims[0], 0), s.substr(nch), style, back_color, wrap);
      }
      return;
    }
  }
  switch (style) {
    case EStyle::regular: break;
    case EStyle::shadowed:
      set_color(back_color);
      for_intL(xd, -1, 2) for_intL(yd, -1, 2) {
        if (xd || yd) draw_text_internal(yx + V(yd, xd), s);
      }
      set_color_to_foreground();
      break;
    case EStyle::boxed:
      set_color(back_color);
      fill_rectangle(convert<float>(yx + V(4, -2)),
                     convert<float>(yx + _font_dims * V(1, narrow_cast<int>(s.size())) + V(4, 2)));
      set_color_to_foreground();
      break;
    default: assertnever("");
  }
  draw_text_internal(yx, s);
}

inline void HwBase::fill_rectangle(const Vec2<float>& top_left, const Vec2<float>& bot_right) {
  auto top_right = V(top_left[0], bot_right[1]);
  auto bot_left = V(bot_right[0], top_left[1]);
  fill_polygon(V(top_left, bot_left, bot_right, top_right));
}

inline void HwBase::draw_segment(const Vec2<float>& yx1, const Vec2<float>& yx2) {
  ASSERTX(_state == EState::open);
  _ar_seg.push(V(yx1, yx2));
  if (_ar_seg.num() >= 256) flush_seg();
}

inline void HwBase::draw_point(const Vec2<float>& yx) {
  ASSERTX(_state == EState::open);
  _ar_point.push(yx);
  if (_ar_point.num() >= 1024) flush_point();
}

// Only reads keys from _hwkey.
inline void HwBase::handle_keyintr() {
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
        _hwkey.insert(0, "\\");  // Put the symbol back.
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

inline void HwBase::soft_flush() {
  assertx(_state == EState::open);
  flush_seg();
  flush_point();
}

inline void HwBase::soft_discard() {
  assertx(_state == EState::open);
  _ar_seg.init(0);
  _ar_point.init(0);
}

inline void HwBase::process_keystring(string& keystring) {
  for (string::size_type i = 0;;) {
    const char ch = keystring[i];
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

inline void HwBase::query_keypress(string s) {
  if (s == "<left>") s = "\b";
  const char ch = s[0];
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
  } else if (s.size() > 1 || !(uchar(ch) >= 32 && uchar(ch) <= 127)) {
    beep();
  } else {
    _query_buffer += ch;
    redraw_later();
  }
}

inline bool HwBase::query(const Vec2<int>& yx, string prompt, string& buffer) {
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

inline bool HwBase::query(const Vec2<int>& yx, string prompt, float& f) {
  string s = sform("%g", f);
  const bool success = query(yx, std::move(prompt), s);
  if (success) f = to_float(s);
  return success;
}

inline bool HwBase::query(const Vec2<int>& yx, string prompt, int& i) {
  string s = sform("%d", i);
  const bool success = query(yx, std::move(prompt), s);
  if (success) i = to_int(s);
  return success;
}

inline Array<string> HwBase::query_open_filenames(const string& hint_filename) {
  string filename = hint_filename;
  if (query(V(20, 10), "Open file:", filename)) return {std::move(filename)};
  return {};
}

inline string HwBase::query_save_filename(const string& hint_filename, bool force) {
  string filename = hint_filename;
  if (query(V(20, 10), "Save file:", filename)) {
    bool ok = force || !file_exists(filename);
    // TODO: Verify that the file can be written.
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
  if (ch >= '0' && ch <= '9') {  // Or std::isdigit().
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
  } else if (1 && to_uint(s) > 0) {
    Warning("Old int32-style color specification; now use #RRGGBBAA");
    const unsigned u = to_uint(s);
    return Pixel((u >> 0) & 255, (u >> 8) & 255, (u >> 16) & 255, (u >> 24) & 255);
  } else {
    assertnever("Cannot parse color '" + scolor + "'");
  }
}

//----------------------------------------------------------------------------

#if defined(GL_VERSION)  // OpenGL.

// See:
// https://open.gl/textures  really nice!
// https://docs.gl/
// https://www.opengl-tutorial.org/beginners-tutorials/tutorial-2-the-first-triangle/   shader program

// c:/Program Files (x86)/Windows Kits/8.1/Include/um/gl/GL.h
// ~/git/mesh_processing/G3dOGL/glext.h

// Check if there are any outstanding OpenGL errors and if so report them.
// For debugging, sprinkle:  ASSERTX(!gl_report_errors());
bool gl_report_errors();  // Returns true if errors (only call after init() and before open() returns).

// Return a string containing all the supported OpenGL extensions.
const string& gl_extensions_string();

// GCC warns on casts between incompatible function pointer types, which is inherent to the wglGetProcAddress()
// pattern; the alternative of casting through `void*` is less portable.
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic ignored "-Wcast-function-type"  // We do not use push/pop as it is too awkward within a macro.
#endif

#define USE_GL_EXT_MAYBE_AUX(func, type, GetProc) \
  using Type##func = type;                        \
  static Type##func func;                         \
  static bool is_init_##func;                     \
  if (!is_init_##func) {                          \
    is_init_##func = true;                        \
    func = reinterpret_cast<Type##func>(GetProc); \
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

// See ~/git/mesh_processing/G3dOGL/glext.h.
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
  // E.g. see https://www.geeks3d.com/20100722/tips-how-to-get-the-list-of-the-opengl-extensions-with-a-core-profile/
  // Remote Desktop (even on Win10):
  //  glGetString(GL_VERSION) = 1.1.0
  //  gl_extensions_string() = GL_WIN_swap_hint GL_EXT_bgra GL_EXT_paletted_texture
  static string s_string;
  static bool is_init = false;
  if (!is_init) {
#if 0
    int n;
    glGetIntegerv(GL_NUM_EXTENSIONS, &n);
    for_int(i, n) s_string += string(glGetStringi(GL_EXTENSIONS, i)) + " ";  // glGetStringi() only in GL 3.0.
    is_init = true;
#else
    const char* s = reinterpret_cast<const char*>(glGetString(GL_EXTENSIONS));
    if (!s) {
      if (1) Warning("glGetString(GL_EXTENSIONS) failed once");
      glGetError();
    } else {
      s_string = s;
      s_string += ' ';
      is_init = true;
    }
#endif
  }
  return s_string;
}

inline void HwBase::clear_window_ogl() {
  glViewport(0, 0, _win_dims[1], _win_dims[0]);
  if (_hidden) {
    hidden_bind_framebuffer();  // A framebuffer object has no front or back buffer to select.
  } else {
    if (_is_glx_dbuf) glDrawBuffer(GL_BACK);
    if (_first_draw) {
      _first_draw = false;
      glDrawBuffer(GL_FRONT_AND_BACK);
    }
  }
  {
    const Vector4 v(_color_background.with(3, 255));
    glClearColor(v[0], v[1], v[2], v[3]);
  }
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (!_hidden) {
    if (_is_glx_dbuf)
      glDrawBuffer(GL_BACK);
    else
      glDrawBuffer(GL_FRONT);
  }
  set_color_to_foreground();
}

// Bind the framebuffer object that replaces the window when _hidden, first (re)allocating it to match _win_dims.
inline void HwBase::hidden_bind_framebuffer() {
  USE_GL_EXT(glBindFramebuffer, PFNGLBINDFRAMEBUFFERPROC);
  if (_win_dims != _hidden_dims) {
    USE_GL_EXT(glGenFramebuffers, PFNGLGENFRAMEBUFFERSPROC);
    USE_GL_EXT(glDeleteFramebuffers, PFNGLDELETEFRAMEBUFFERSPROC);
    USE_GL_EXT(glGenRenderbuffers, PFNGLGENRENDERBUFFERSPROC);
    USE_GL_EXT(glDeleteRenderbuffers, PFNGLDELETERENDERBUFFERSPROC);
    USE_GL_EXT(glBindRenderbuffer, PFNGLBINDRENDERBUFFERPROC);
    USE_GL_EXT(glRenderbufferStorageMultisample, PFNGLRENDERBUFFERSTORAGEMULTISAMPLEPROC);
    USE_GL_EXT(glFramebufferRenderbuffer, PFNGLFRAMEBUFFERRENDERBUFFERPROC);
    USE_GL_EXT(glCheckFramebufferStatus, PFNGLCHECKFRAMEBUFFERSTATUSPROC);
    if (_hidden_dims != twice(0)) {
      glDeleteFramebuffers(2, _hidden_framebuffers.data());
      glDeleteRenderbuffers(3, _hidden_renderbuffers.data());
    }
    _hidden_dims = _win_dims;
    glGenFramebuffers(2, _hidden_framebuffers.data());
    glGenRenderbuffers(3, _hidden_renderbuffers.data());
    const auto storage = [&](unsigned renderbuffer, int samples, GLenum format) {
      glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer);
      glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, format, _win_dims[1], _win_dims[0]);
    };
    // The drawn framebuffer is multisampled like the window would be; the resolved one is for reading pixels.
    storage(_hidden_renderbuffers[0], _hidden_samples, GL_RGBA8);
    storage(_hidden_renderbuffers[1], _hidden_samples, GL_DEPTH_COMPONENT24);
    storage(_hidden_renderbuffers[2], 0, GL_RGBA8);
    glBindFramebuffer(GL_FRAMEBUFFER, _hidden_framebuffers[1]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _hidden_renderbuffers[2]);
    assertx(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
    glBindFramebuffer(GL_FRAMEBUFFER, _hidden_framebuffers[0]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, _hidden_renderbuffers[0]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, _hidden_renderbuffers[1]);
    assertx(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, _hidden_framebuffers[0]);
}

// Resolve the drawn hidden framebuffer so that subsequent pixel reads (e.g. glReadPixels()) see the rendering.
inline void HwBase::hidden_resolve_framebuffer() {
  if (_hidden_dims == twice(0)) return;  // Nothing has been drawn yet.
  USE_GL_EXT(glBindFramebuffer, PFNGLBINDFRAMEBUFFERPROC);
  USE_GL_EXT(glBlitFramebuffer, PFNGLBLITFRAMEBUFFERPROC);
  const int w = _hidden_dims[1], h = _hidden_dims[0];
  glBindFramebuffer(GL_READ_FRAMEBUFFER, _hidden_framebuffers[0]);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _hidden_framebuffers[1]);
  glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _hidden_framebuffers[0]);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, _hidden_framebuffers[1]);
}

// Save the rendering in the hidden framebuffer to an image file (as done for the -offscreen option).
inline void HwBase::hidden_write_image(const string& filename) {
  assertx(_hidden_dims == _win_dims);
  hidden_resolve_framebuffer();
  Image image(_win_dims);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, _win_dims[1], _win_dims[0], GL_RGBA, GL_UNSIGNED_BYTE, image.data());
  assertx(!gl_report_errors());
  image.reverse_y();  // Because OpenGL has image origin at lower-left.
  image.write_file(filename);
}

inline void HwBase::draw_text_ogl(const Vec2<int>& yx, const string& s) {
  glListBase(_listbase_font);
  USE_GL_EXT_MAYBE(glWindowPos2i, PFNGLWINDOWPOS2IPROC);  // Not supported on Remote Desktop.
  if (assertw(glWindowPos2i)) {
    const int x = yx[1], y = _win_dims[0] - yx[0] - _font_dims[0];
    glWindowPos2i(x, y);  // Reverse y; not clip-tested, so raster position valid.
    glCallLists(narrow_cast<int>(s.size()), GL_UNSIGNED_BYTE, reinterpret_cast<const uchar*>(s.c_str()));
  } else {
    const int x = yx[1], y = yx[0] + _font_dims[0];
    glRasterPos2i(x, y);  // Clipped, so raster position may be invalid.
    glCallLists(narrow_cast<int>(s.size()), GL_UNSIGNED_BYTE, reinterpret_cast<const uchar*>(s.c_str()));
  }
}

inline void HwBase::fill_polygon_ogl(CArrayView<Vec2<float>> points) {
  glBegin(points.num() == 3 ? GL_TRIANGLES : points.num() == 4 ? GL_QUADS : GL_POLYGON);
  for (const Vec2<float>& p : points) glVertex2f(p[1], p[0]);
  glEnd();
}

inline void HwBase::flush_seg_ogl() {
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

inline void HwBase::flush_point_ogl() {
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

#endif  // MESH_PROCESSING_LIBHWWINDOWS_HWBASE_H_
