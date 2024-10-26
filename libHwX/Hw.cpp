// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Hw.h"

#include <sys/select.h>  // fd_set, select()

extern "C" {
#include <X11/Xatom.h>  // XA_STRING
#include <X11/Xos.h>
#include <X11/cursorfont.h>
#include <X11/keysym.h>
}

#include <cerrno>
#include <csignal>  // signal()
#include <cstring>  // strlen(), strerror()
#include <ctime>    // setitimer(), struct itimerval, struct timeval

#include "Hw.xbm"
#include "libHh/Args.h"
#include "libHh/Image.h"
#include "libHh/MathOp.h"  // is_pow2()

#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wold-style-cast"  // for DefaultScreen() etc.
#endif

namespace hh {

namespace {
string from_nullptr_or_cstring(const char* s) { return !s ? "" : s; }
const char* to_nullptr_or_cstring(const string& s) { return s == "" ? nullptr : s.c_str(); }
}  // namespace

void set_keyintr(Hw& hw) { hw.set_keyintr(); }

const string k_font_name = "6x13";

static Hw* g_hw;  // assumes single-window model; fine.

Hw::Hw() {
#if defined(HH_OGLX)
  _oglx = true;
#endif
}

bool Hw::init_aux(Array<string>& aargs) {
  assertx(_state == EState::uninit);
  _state = EState::init;
  string display_name = "";
  bool iconic = false;
  bool minimize = false;
  ParseArgs args(aargs, "Hw");
  args.p("-disp[lay", display_name, "name : set DISPLAY");
  args.p("-geom[etry", _user_geometry, "geom : set window geometry (e.g. 640x480)");
  args.f("-icon[ic", iconic, ": set initial window state");
  args.p("-back[ground", _backcolor, "color : set backcolor (e.g. white or #5987B3)");
  args.p("-fore[ground", _forecolor, "color : set forecolor (e.g. black)");
  args.p("-hwdebug", _hwdebug, "val : set debug level");
  args.p("-hwkey", _hwkey, "string : simulate keys");
  args.p("-hwdelay", _hwdelay, "secs : delay between keys");
  args.f("-bigfont", _bigfont, ": use larger font");
  args.f("-minimize", minimize, ": same as -iconic");
  args.f("-maximize", _maximize, ": set initial window state");
  args.f("-fullscreen", _fullscreen, ": set initial window state");
  args.p("-offscreen", _offscreen, "imagefilename : save image");
  args.f("-nosetforeground", _nosetforeground, ": do not ask that window receive focus");
  args.other_args_ok();
  args.other_options_ok();
  args.disallow_prefixes();
  if (!args.parse_and_extract(aargs)) {
    _state = EState::uninit;
    return false;
  }
  assertx(aargs.num());
  _argv0 = aargs[0];
  if (minimize) iconic = true;
  if (_offscreen != "") iconic = true;  // less distracting; ideally window would be invisible
  g_hw = this;
  //
  _pwmhints = assertx(XAllocWMHints());  // never freed using XFree()
  // _pwmhints->flags = 0;  // unnecessary
  if (iconic) {
    _pwmhints->initial_state = IconicState;
    _pwmhints->flags |= StateHint;
  }
  if (!(_display = XOpenDisplay(to_nullptr_or_cstring(display_name)))) {
    showf("Hw: cannot connect to X server '%s'\n", XDisplayName(to_nullptr_or_cstring(display_name)));
    _state = EState::uninit;
    return false;
  }
  _screen = DefaultScreen(_display);
  _depth = DisplayPlanes(_display, _screen);
  if (_hwdebug) SHOW(_screen, _depth, XDisplayCells(_display, _screen));
  _cmap = DefaultColormap(_display, _screen);
  //
  set_window_title(_argv0);
  set_double_buffering(true);
  return true;
}

static int my_io_error_handler(Display* display) {
  dummy_use(display);
  // This handler is called when the window close button is pressed, so there is nothing unusual.
  // SHOW("X server connection is lost");
  exit_immediately(0);  // IE error handler is unfortunately required to exit
  // Ideally, CYGWIN would send a DestroyWindow event when user clicks "X" button, but it does not seem to.
  return 0;
}

void Hw::get_color(string color_name, unsigned long& pixel, Pixel& color) {
  if (color_name == "hhblue") color_name = "#5987B3";
  XColor xcolor;
  if (!XParseColor(_display, _cmap, color_name.c_str(), &xcolor)) assertnever("color '" + color_name + "' not found");
  assertx(XAllocColor(_display, _cmap, &xcolor));
  pixel = xcolor.pixel;
  // The red, green, blue color fields in XColor are in range [0, 65535].
  color = Pixel(xcolor.red / 256, xcolor.green / 256, xcolor.blue / 256);
}

void Hw::open() {
  assertx(_state == EState::init);
  _state = EState::open;
  if (_hwdebug) SHOW("hw: open");
  Visual* visual = DefaultVisual(_display, _screen);
  unsigned border_width = 1;
#if defined(HH_OGLX)
  GLXContext glcx;
  dummy_init(glcx);
  if (_oglx) {
    // static const int multisample = getenv_int("GLX_MULTISAMPLE");
    _multisample = getenv_int("MULTISAMPLE", 4, true);
    XVisualInfo* visinfo;
    for (;;) {
      Array<int> attributelist = {
          GLX_RGBA,  // (for TrueColor and DirectColor instead of PseudoColor; there is no "RGB")
          GLX_RED_SIZE,
          8,  // could be just "1"
          GLX_DEPTH_SIZE,
          24,  // could be just "1"; 24 is arbitrary
          // Found necessary for Chrome Remote Desktop and "//third_party/mesa:GL",
          //  else ALPHA_SIZE == 0 and somehow the rendered window is mostly transparent.
          GLX_ALPHA_SIZE,
          8,
      };
      if (_hwdebug) SHOW("hw: open", _is_glx_dbuf);
      if (_is_glx_dbuf) attributelist.push(GLX_DOUBLEBUFFER);
      if (_multisample > 1) {
        // Warning("Turning on GLX_SAMPLES_SGIS");
        assertw(_multisample == 4 || _multisample == 8 || _multisample == 16);
        attributelist.push_array(V(GLX_SAMPLES_SGIS, _multisample));
        // then becomes enabled by default.
        // Note: inf_reality balrog has _multisample<=8.
        // Note: may want to disable multisampling manually before drawing anti-aliased lines.  Actually,
        //  anti-aliased lines look rather poor on inf_reality (too thick); we prefer multisampled aliased lines.
        // Note: do we need glEnable(GL_POLYGON_SMOOTH)? No, it seems that it is an old way of anti-aliasing
        //  using alpha planes.
      }
      attributelist.push(None);
      // Note: on my Octane, this chooses VisualId 0x37: `glxinfo`:
      //     visual  x  bf lv rg d st  r  g  b a  ax dp st accum buffs  ms
      //   id dep cl sp sz l  ci b ro sz sz sz sz bf th cl  r  g  b  a ns b
      //  0x37 24 tc  . 36  . r  y  . 12 12 12  .  . 24  8 16 16 16 16  . .
      // which has 12bits/channel (even though only 4 were requested); not bad.
      // Note: on cygwin, it selects: VisualId 396 (0x18c)
      //     visual  x   bf lv rg d st  colorbuffer  sr ax dp st accumbuffer  ms  cav
      //   id dep cl sp  sz l  ci b ro  r  g  b  a F gb bf th cl  r  g  b  a ns b eat
      // 0x18c 24 tc  0  32  0 r  y .   8  8  8  8 .  .  0 24  0 16 16 16 16  0 0 None
      visinfo = glXChooseVisual(_display, _screen, attributelist.data());
      if (visinfo) break;
      if (_multisample > 1) {
        if (_hwdebug) Warning("Downgrading to MULTISAMPLE=1");
        _multisample = 1;
        continue;
      }
      assertnever("Could not successfully call glXChooseVisual()");
    }
    _depth = visinfo->depth;  // (number of bits in RGBA; unrelated to GLX_DEPTH_SIZE)
    _screen = visinfo->screen;
    visual = visinfo->visual;
    if (_hwdebug) {
      SHOW(_depth, _screen, visinfo->visualid, visinfo->bits_per_rgb);
#define T(attrib)                                              \
  {                                                            \
    int value;                                                 \
    assertx(!glXGetConfig(_display, visinfo, attrib, &value)); \
    SHOW(#attrib, value);                                      \
  }                                                            \
  HH_EAT_SEMICOLON
      T(GLX_USE_GL);
      T(GLX_BUFFER_SIZE);
      T(GLX_LEVEL);
      T(GLX_RGBA);
      T(GLX_DOUBLEBUFFER);
      T(GLX_STEREO);
      T(GLX_RED_SIZE);
      T(GLX_ALPHA_SIZE);
      T(GLX_DEPTH_SIZE);
      T(GLX_STENCIL_SIZE);
      T(GLX_ACCUM_RED_SIZE);
      T(GLX_ACCUM_ALPHA_SIZE);
#undef T
    }
    // If glXCreateContext fails when launching client over "ssh -Y", it may be because X server
    //  was launched without "+iglx" option.
    GLXContext share_list = nullptr;  // no shared display lists
    Bool direct = 1;                  // direct connection to graphics system if possible
    // Note: "It may not be possible to render to a GLX pixmap with a direct rendering context"
    glcx = assertx(glXCreateContext(_display, visinfo, share_list, direct));
    if (_hwdebug) SHOW(glXIsDirect(_display, glcx));
    _cmap = XCreateColormap(_display, RootWindow(_display, _screen), visual, AllocNone);
    border_width = 0;
    // ? XSetErrorHandler(0);
  }
#endif  // defined(HH_OGLX)
  {
    if (_backcolor == "") _backcolor = from_nullptr_or_cstring(XGetDefault(_display, _argv0.c_str(), "background"));
    if (_backcolor == "") _backcolor = _default_background;
    if (_forecolor == "") _forecolor = from_nullptr_or_cstring(XGetDefault(_display, _argv0.c_str(), "foreground"));
    if (_forecolor == "") _forecolor = _default_foreground;
    get_color(_backcolor, _pixel_background, _color_background);
    get_color(_forecolor, _pixel_foreground, _color_foreground);
  }
  Pixel dummy_color_border;
  get_color("black", _pixel_border, dummy_color_border);
  if (_user_geometry == "")
    _user_geometry = from_nullptr_or_cstring(XGetDefault(_display, _argv0.c_str(), "geometry"));
  XFontStruct* font_info;
  if (!(font_info = XLoadQueryFont(_display, k_font_name.c_str()))) {
    showf("Hw: cannot open font '%s'\n", k_font_name.c_str());
    return;
  }
  XSizeHints* pxsh = assertx(XAllocSizeHints());  // never freed using XFree()
  {
    pxsh->flags = PPosition | PSize | PMinSize;
    pxsh->x = 0, pxsh->y = 0;  // (the x, y, width, and height members are now obsolete)
    pxsh->width = _win_dims[1];
    pxsh->height = _win_dims[0];
    pxsh->min_width = 0;
    pxsh->min_height = 0;
  }
  int bitmask =
      XWMGeometry(_display, _screen, to_nullptr_or_cstring(_user_geometry), to_nullptr_or_cstring(_default_geometry),
                  1, pxsh, &pxsh->x, &pxsh->y, &pxsh->width, &pxsh->height, &pxsh->win_gravity);
#if defined(__CYGWIN__)
  // Adjustment for window borders (ideally, only if relative to upper and left boundaries).
  pxsh->x += 3;
  pxsh->y += 22;
#endif
  if (bitmask & (XValue | YValue)) pxsh->flags |= USPosition;
  if (bitmask & (WidthValue | HeightValue)) pxsh->flags |= USSize;
  _win_dims = V(pxsh->height, pxsh->width);
  {
    XSetWindowAttributes swa = {};
    swa.colormap = _cmap;
    swa.border_pixel = _pixel_border;
    swa.background_pixel = _pixel_background;
    _win = XCreateWindow(_display, RootWindow(_display, _screen), pxsh->x, pxsh->y, pxsh->width, pxsh->height,
                         border_width, _depth, InputOutput, visual, CWColormap | CWBorderPixel | CWBackPixel, &swa);
  }
  Pixmap icon_pixmap = assertx(XCreateBitmapFromData(_display, _win, hw_bits, hw_width, hw_height));
  _pwmhints->flags |= IconPixmapHint;
  _pwmhints->icon_pixmap = icon_pixmap;
  XClassHint* pxclasshint = assertx(XAllocClassHint());  // never freed using XFree()
  pxclasshint->res_name = const_cast<char*>(_argv0.c_str());
  pxclasshint->res_class = const_cast<char*>("Hw");
  Vec2<const char*> largv = {_argv0.c_str(), nullptr};
  const int largc = 1;
  string icon_name = _argv0;
  XmbSetWMProperties(_display, _win, _window_title.c_str(), icon_name.c_str(), const_cast<char**>(largv.data()), largc,
                     pxsh, _pwmhints, pxclasshint);
  XStoreName(_display, _win, _window_title.c_str());  // necessary to set window title in CYGWIN
  Cursor cursor = XCreateFontCursor(_display, XC_crosshair);
  XDefineCursor(_display, _win, cursor);
  XSelectInput(_display, _win,
               ExposureMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask | StructureNotifyMask);
  if (0)
    XSelectInput(_display, _win,
                 ExposureMask | KeyPressMask | ButtonPressMask | ButtonReleaseMask | StructureNotifyMask |
                     EnterWindowMask | LeaveWindowMask | FocusChangeMask | PropertyChangeMask | VisibilityChangeMask |
                     SubstructureNotifyMask);
  if (_oglx) {
    // should not use X font because of double-buffering problem.
  } else {
    _gc = XCreateGC(_display, _win, 0UL, &_gcvalues);
    XSetFont(_display, _gc, font_info->fid);
    // avoid generating NoExpose events when doing XCopyArea()
    XSetGraphicsExposures(_display, _gc, 0);
    XSetBackground(_display, _gc, _pixel_background);
    XSetForeground(_display, _gc, _pixel_foreground);
  }
  if (0) {  // This fails With Cinnamon: it prevents window close and we don't receive any ClientMessage event.
    _wmDeleteMessage = XInternAtom(_display, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(_display, _win, &_wmDeleteMessage, 1);
  }
  // if (_offscreen == "")
  XMapWindow(_display, _win);  // window must be mapped to obtain an image
  if (_maximize) {
    _maximize = false;
    // (Note that make_fullscreen(true) is different from maximize.)
    XEvent event;
    event.type = ClientMessage;
    event.xclient.display = _display;
    event.xclient.window = _win;
    event.xclient.message_type = XInternAtom(_display, "_NET_WM_STATE", False);
    event.xclient.format = 32;
    event.xclient.data.l[0] = 1;  // _NET_WM_STATE_ADD
    event.xclient.data.l[1] = XInternAtom(_display, "_NET_WM_STATE_MAXIMIZED_HORZ", False);
    event.xclient.data.l[2] = XInternAtom(_display, "_NET_WM_STATE_MAXIMIZED_VERT", False);
    assertx(XSendEvent(_display, XDefaultRootWindow(_display), False, SubstructureNotifyMask, &event));
  }
  if (_fullscreen) {
    _fullscreen = false;
    make_fullscreen(true);
  }
  if (_oglx) {
#if defined(HH_OGLX)
    glXMakeCurrent(_display, _win, glcx);
    if (_hwdebug) SHOW(glGetString(GL_VERSION));
    if (_hwdebug) SHOW(gl_extensions_string());
    {
      const string regular_font_name = (
          // "9x15bold"
          // iris13 (close to IrisGL default font)
          // "-*-iris-medium-r-normal--*-130-*-*-m-*-iso8859-1" // latest on SGI
          // "-*-courier-*-r-*-*-*-100-*-*-*-*-*-*" // works on CYGWIN; has width=9
          // "-*-courier-*-r-*-*-*-190-*-*-*-*-*-*" // looks like width16 on SGI
          // "fixed"
          "9x15bold");
      const string big_font_name = (
          // "12x24" // yuck
          "-misc-fixed-medium-r-normal--20-200-75-75-c-100-iso8859-1"  // not all that large
      );
      string font_name = _bigfont ? big_font_name : regular_font_name;
      XFontStruct* font_info2;
      font_info2 = XLoadQueryFont(_display, font_name.c_str());
      if (!font_info2) {
        showf("XLoadQueryFont failed on font '%s', so reverting to font 'fixed'\n", font_name.c_str());
        font_name = "fixed";  // == "6x13"
        font_info2 = assertx(XLoadQueryFont(_display, font_name.c_str()));
      }
      Font id = font_info2->fid;
      int first = font_info2->min_char_or_byte2;
      int last = font_info2->max_char_or_byte2;
      if (0) SHOW(first, last);  // first=0 last=255
      _listbase_font = assertx(glGenLists(last + 1));
      assertx(!gl_report_errors());
      glXUseXFont(id, first, last - first + 1, _listbase_font + first);
      {
        GLenum v = glGetError();
        if (v) {
          assertx(v == GL_OUT_OF_MEMORY);  // Unexpected behavior under WSL after glXUseXFont().
          assertx(!glGetError());
        }
      }
      assertx(!gl_report_errors());
      if (font_name == "fixed") {
        _font_dims = V(13 + 2, 6);
      } else if (!_bigfont) {
        _font_dims = V(15 + 3, 9);  // (to be compatible with IrisGL)
      } else {
        _font_dims = V(38, 16);  // 16x38 from CourierBold20 of IrisGL fm.
        _font_dims = V(28, 16);  // reduce line-spacing
      }
      if (_hwdebug) SHOW(_font_dims);
    }
    if (_multisample > 1) {  // likely unnecessary because the default value for GL_MULTISAMPLE is GL_TRUE
      glEnable(GL_MULTISAMPLE);
      assertw(!gl_report_errors());
      if (_hwdebug) {
        int sample_buffers;
        glGetIntegerv(GL_SAMPLE_BUFFERS, &sample_buffers);
        SHOW(sample_buffers);
        int samples;
        glGetIntegerv(GL_SAMPLES, &samples);
        SHOW(samples);
      }
    }
#endif
  }
  _draw = _win;
  // TODO: Implement HwBase::drag_and_drop().
  int force_first_draws = 0;
#if defined(__CYGWIN__)
  force_first_draws = 3;  // For some unknown reason, first couple draws are corrupt on __CYGWIN__
#endif
  if (1) {
    // Avoid error message upon pressing close window button, e.g.:
    //  XIO:  fatal IO error 11 (Resource temporarily unavailable) on X server ":0"
    XSetIOErrorHandler(my_io_error_handler);
  }
  if (_offscreen == "") {
    for (int i = 0;;) {
      if (loop()) break;
      if (++i <= force_first_draws) redraw_later();
    }
  } else {
    assertw(_oglx);
#if defined(HH_OGLX)
    _exposed = true;
    // Due to interaction with window events or double-buffering, first draw_it() produces only background color.
    draw_it();
    draw_it();
    // glFlush();
    glFinish();
    Image image(_win_dims);
    if (1) {
      Vec2<int> tdims = image.dims();
      for_int(c, 2) {
        while (!is_pow2(tdims[c])) tdims[c]++;
      }
      Image timage(tdims);
      GLenum internal_format = GL_RGBA8;
      glTexImage2D(GL_TEXTURE_2D, 0, internal_format, tdims[1], tdims[0], 0, GL_RGBA, GL_UNSIGNED_BYTE, timage.data());
      int get_w;
      glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &get_w);
      assertx(get_w == tdims[1]);
      glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, _win_dims[1], _win_dims[0]);
      glFinish();
      glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, timage.data());
      image = crop(timage, twice(0), tdims - _win_dims);
    }
    image.reverse_y();  // because OpenGL has image origin at lower-left
    image.write_file(_offscreen);
#endif
  }
#if defined(HH_OGLX)
  assertw(!gl_report_errors());
#endif
  _state = EState::uninit;
  // Cleanup
  XCloseDisplay(_display);
}

bool Hw::loop() {
  for (;;) {
    // handle all possible types of events
    if (_update == EUpdate::quit) return true;
    handle_events();
    if (_update == EUpdate::quit) return true;
    handle_keyintr();
    if (_update == EUpdate::quit) return true;
    if (!got_event() && !_is_keyintr) {
      // no more events, so possibly blocked
      if (_update != EUpdate::nothing) break;
      // Later found: https://stackoverflow.com/questions/8592292/how-to-quit-the-blocking-of-xlibs-xnextevent
      fd_set fdr;
      int fd = ConnectionNumber(_display);
      FD_ZERO(&fdr);
      FD_SET(fd, &fdr);
      if (_watch_fd0) FD_SET(0, &fdr);
      struct timeval tv;
      tv.tv_sec = 0;
      tv.tv_usec = 30'000;  // .03sec; introduced to support wake_up()
      struct timeval* ptv = 1 ? &tv : nullptr;
      if (select(fd + 1, &fdr, implicit_cast<fd_set*>(nullptr), implicit_cast<fd_set*>(nullptr), ptv) == -1) {
        if (errno == EBADF) {
          assertx(_watch_fd0);
          _watch_fd0 = false;
          continue;
        }
        if (errno == EINTR) continue;
        std::cerr << "Hw: error in select(): " << std::strerror(errno) << "\n";
        return true;
      }
      if (_watch_fd0 && FD_ISSET(0, &fdr)) input_received();
    }
  }
  // No more events but redraw has been requested
  draw_it();
  return false;
}

void Hw::handle_events() {
  while (got_event() && _update != EUpdate::quit) {
    handle_event();
  }
}

void Hw::handle_event() {
  _gotevent = false;
  if (_hwdebug) SHOW(_event.type);
  switch (_event.type) {
    case MapNotify:
      if (_hwdebug) SHOW("MapNotify");
      if (!_exposed) {
        _exposed = true;
        if (_hwdebug) SHOW("window now exposed");
        if (_hwkey != "") start_hwkey();
      }
      break;
    case UnmapNotify:
      if (_hwdebug) SHOW("UnmapNotify");
      // Possibly we could somehow force the redraws to stop.
      break;
    case Expose:
      if (_hwdebug) SHOW("Expose", _exposed);
      if (!_exposed) return;
      while (XCheckTypedEvent(_display, Expose, &_event)) {
      }
      redraw_now();
      break;
    case ConfigureNotify: {
      _win_pos = V(_event.xconfigure.y, _event.xconfigure.x);
      _win_dims = V(_event.xconfigure.height, _event.xconfigure.width);
      bool send_event = _event.xconfigure.send_event;
      if (_hwdebug) SHOW("configure", _win_pos, _win_dims, send_event, _exposed, _oglx);
      if (send_event) return;  // WM has moved window
      if (!_exposed && _oglx) return;
      redraw_now();
      soft_discard();
      if (_oglx) {
#if defined(HH_OGLX)
        glViewport(0, 0, _win_dims[1], _win_dims[0]);
        {
          Vector4 v(_color_background);
          glClearColor(v[0], v[1], v[2], 1.f);
        }
        glDrawBuffer(GL_FRONT_AND_BACK);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (_is_glx_dbuf) glDrawBuffer(GL_BACK);
        set_color_to_foreground();
#endif
      } else if (_is_pixbuf) {
        bool draw_was_bbuf = _draw == _bbuf;
        XFreePixmap(_display, _bbuf);
        _bbuf = XCreatePixmap(_display, _win, _win_dims[1], _win_dims[0], _depth);
        if (draw_was_bbuf) _draw = _bbuf;
        XClearWindow(_display, _win);
      } else {
        XClearWindow(_display, _win);
      }
      while (XCheckTypedEvent(_display, Expose, &_event)) {
      }
      break;
    }
    case KeyPress:
      if (_hwdebug) SHOW("KeyPress");
      handle_key();
      break;
    case ButtonPress:
    case ButtonRelease: {
      if (_hwdebug) SHOW("Button");
      int butnum = _event.xbutton.button;
      const bool pressed = _event.type == ButtonPress;
      const bool shift = (_event.xbutton.state & ShiftMask) != 0;
      dummy_use(shift);
      if (butnum == 4 || butnum == 5) {
        if (pressed) wheel_turn(butnum == 4 ? +1.f : -1.f);
      } else {
        if (butnum == 8) butnum = 4;  // Back button
        if (butnum == 9) butnum = 5;  // Forward button
        button_press(butnum, pressed, V(_event.xbutton.y, _event.xbutton.x));
      }
      break;
    }
    case MappingNotify:
      if (_hwdebug) SHOW("MappingNotify");
      XRefreshKeyboardMapping(reinterpret_cast<XMappingEvent*>(&_event));
      break;
    // case ClientMessage:
    //    SHOWL;
    //    if (_event.xclient.data.l[0] == _wmDeleteMessage) {
    //        SHOWL;
    //        // Receive message from window manager before it kills the window.
    //        _update = EUpdate::quit;
    //    }
    //    break;
    default:
      if (_hwdebug) SHOW("ignoring unknown event");
  }
}

bool Hw::got_event() {
  // If connection to X server is lost, XCheckMaskEvent generates an IO error here.
  if (!_gotevent && XCheckMaskEvent(_display, 0xfffffff, &_event)) _gotevent = true;
  return _gotevent;
}

bool Hw::suggests_stop() {
  assertx(_state == EState::open);
  if (got_event() && _update != EUpdate::quit) handle_events();
  if (_is_keyintr) handle_keyintr();
  return _update == EUpdate::quit || _update == EUpdate::redrawnow;
}

static void handle_alarm(int) {
  signal(SIGALRM, handle_alarm);  // for ATT unix
  set_keyintr(*g_hw);
}

void Hw::start_hwkey() {
  struct itimerval ti;
  struct timeval tv;
  signal(SIGALRM, handle_alarm);
  int64_t usec = int(_hwdelay * 1'000'000.f + .5f);
  tv.tv_sec = int(usec / 1'000'000);
  tv.tv_usec = int(usec % 1'000'000);
  ti.it_value = tv;
  ti.it_interval = tv;
  if (setitimer(ITIMER_REAL, &ti, implicit_cast<struct itimerval*>(nullptr)))
    std::cerr << "Hw: setitimer: " << std::strerror(errno) << "\n";
}

void Hw::end_hwkey() {
  struct itimerval ti;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  ti.it_value = tv;
  ti.it_interval = tv;
  if (setitimer(ITIMER_REAL, &ti, implicit_cast<struct itimerval*>(nullptr)))
    std::cerr << "Hw: setitimer2: " << std::strerror(errno) << "\n";
  // SIGALRM not reset to SIG_DFL just to be sure
  // signal(SIGALRM, SIG_DFL);
}

void Hw::handle_key() {
  string s;
  {
    Vec<char, 20> buf;
    KeySym keysym;
    int nchar = XLookupString(reinterpret_cast<XKeyEvent*>(&_event), buf.data(), buf.num() - 1, &keysym,
                              implicit_cast<XComposeStatus*>(nullptr));
    assertx(buf.ok(nchar));
    buf[nchar] = 0;
    s = buf.data();
    if (keysym >= XK_F1 && keysym <= XK_F12) {
      s = sform("<f%d>", int(keysym - XK_F1 + 1));
    } else if (keysym >= XK_0 && keysym <= XK_9) {  // so that control-2, .. control-9 are interpreted correctly
      assertx(XK_0 == '0');
      s = sform("%c", narrow_cast<char>(keysym));
    } else {
      switch (keysym) {
        case XK_Left: s = "<left>"; break;
        case XK_Right: s = "<right>"; break;
        case XK_Up: s = "<up>"; break;
        case XK_Down: s = "<down>"; break;
        case XK_Home: s = "<home>"; break;
        case XK_End: s = "<end>"; break;
        case XK_Prior: s = "<prior>"; break;
        case XK_Next: s = "<next>"; break;
        case XK_Insert: s = "<insert>"; break;
        case XK_Delete: s = "<delete>"; break;
        default: void();
      }
    }
    if (_hwdebug) SHOW(keysym, s, s.size(), convert<int>(convert<uchar>(ArView<char>(s.data(), s.size()))));
  }
  // (keysym>=XK_space && keysym<=XK_asciitilde)
  if (s == "") return;  // e.g. 'shift' key
  if (_query) {
    query_keypress(s);
  } else if (key_press(s)) {
    // client has handled key press
  } else if (s == "\033") {  // <esc> key (== uchar{27})
    quit();
  } else {
    beep();
  }
}

void Hw::draw_it() {
  _update = EUpdate::nothing;
  if (!_exposed) return;  // user events before X expose event
  if (_hwdebug) SHOW("draw_it", _win_dims);
  if (_need_toggle_buffering) {
    _need_toggle_buffering = false;
    toggle_buffering();
  }
  _async = true;
  if (_query) {
    clear_window();
    draw_text(_query_yx, _query_prompt + _query_buffer + "_");
  } else {
    // 2014-12-13
    glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  // for non-premultiplied alpha
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);  // 2017-02-23; source is assumed to have premultiplied alpha
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();  // glLoadMatrixf(to_Matrix(Frame::identity()).data());
    if (0) {
      glOrtho(-.5, _win_dims[1] - .5, _win_dims[0] - .5, -.5, -1., 1.);  // multiplies GL_PROJECTION; reverse y
    } else {
      // 2015-01-08 pixels are now centered at half-integers
      glOrtho(0., _win_dims[1] - 0., _win_dims[0] - 0., 0., -1., 1.);  // multiplies GL_PROJECTION; reverse y
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    draw_window(_win_dims);
  }
  _async = false;
  if (_update == EUpdate::quit) return;
  if (_update == EUpdate::nothing || _update == EUpdate::redrawlater) {
    soft_flush();
  } else {
    soft_discard();
  }
  if (_oglx) {
#if defined(HH_OGLX)
    if (_update == EUpdate::nothing || _update == EUpdate::redrawlater) {
      const bool vista = true;
      if (!vista) glDrawBuffer(GL_FRONT);
      if (_is_glx_dbuf) {
        if (_hwdebug) SHOW("glxSwapBuffers1");
        if (0) {
          if (_hwdebug) SHOW("glxSwapBuffers1b");
          glFinish();
          if (_hwdebug) SHOW("glxSwapBuffers1c");
        }
        if (0) {
          SHOW("before_swap");
          sleep(1);
        }
        if (0) glFinish();  // does not help
        // 2015-02-04 cygwin new bug: Swapping the buffers causes the window to blink to background.
        glXSwapBuffers(_display, _win);
        // 2015-07-27 cygwin: "VideoViewer ~/data/image/lake.png" causes segmentation fault in line above.
        if (0) {
          SHOW("after_swap");
          sleep(1);
        }
        if (_hwdebug) SHOW("glxSwapBuffers2");
      } else {
        hard_flush();
      }
    }
#endif
  } else if (_is_pixbuf) {
    if (_update == EUpdate::nothing || _update == EUpdate::redrawlater)
      XCopyArea(_display, _bbuf, _win, _gc, 0, 0, _win_dims[1], _win_dims[0], 0, 0);
    _draw = _win;
  } else {
    hard_flush();
  }
#if 1 || defined(__CYGWIN__)
  // This synchronization was necessary with cygwin to avoid having GLX build a buffer of many frames of rendering.
  // 2016-03-17 this also became necessary with iglx over ssh under Unix,
  //   e.g. ~/distrib/bin/unix/G3dOGL ~/distrib/demos/data/fandisk.orig.m -key J
  if (1) XSync(_display, 0);
#endif
}

void Hw::clear_window() {
  soft_discard();
  if (_oglx) {
#if defined(HH_OGLX)
    clear_window_ogl();
#endif
  } else {
    XSetForeground(_display, _gc, _pixel_background);
    if (_is_pixbuf) _draw = _bbuf;
    XFillRectangle(_display, _draw, _gc, 0, 0, _win_dims[1], _win_dims[0]);
    XSetForeground(_display, _gc, _pixel_foreground);
  }
}

void Hw::toggle_buffering() {
  assertx(!_async && _state == EState::open && !_oglx);
  if (!_is_pixbuf) {
    allocate_buf();
  } else {
    deallocate_buf();
  }
}

void Hw::allocate_buf() {
  assertx(!_is_pixbuf && !_oglx);
  if (_hwdebug) SHOW("allocating double buffer", _depth);
  _bbuf = XCreatePixmap(_display, _win, _win_dims[1], _win_dims[0], _depth);
  _is_pixbuf = true;
}

void Hw::deallocate_buf() {
  assertx(_is_pixbuf);
  if (_hwdebug) SHOW("deallocating double buffer");
  XFreePixmap(_display, _bbuf);
  _draw = _win;
  _is_pixbuf = false;
}

void Hw::beep() {
  assertx(_state != EState::uninit);
  XBell(_display, 0);
}

void Hw::set_double_buffering(bool newstate) {
  if (_offscreen != "") newstate = false;
  if (_oglx) {
    if (!assertw(_state == EState::init)) return;
    _is_glx_dbuf = newstate;
    // if !_is_glx_dbuf, do not call glXSwapBuffers(), glDrawBuffer(GL_BACK)
    if (_hwdebug) SHOW(_is_glx_dbuf);
    return;
  }
  bool ostate = _is_pixbuf ^ _need_toggle_buffering;
  if (newstate != ostate) {
    if (_async || _state != EState::open) {
      assertx(!_need_toggle_buffering);
      _need_toggle_buffering = true;
    } else {
      toggle_buffering();
    }
  }
}

bool Hw::get_pointer(Vec2<int>& yx) {
  assertx(_state == EState::open);
  int rx, ry;
  unsigned mask;
  Window rw, cw;
  return XQueryPointer(_display, _win, &rw, &cw, &rx, &ry, &yx[1], &yx[0], &mask);
}

bool Hw::get_key_modifier(EModifier modifier) {
  Vec<char, 32> keys;
  XQueryKeymap(_display, keys.data());
  const auto key_pressed = [&](KeySym keysym) {
    int keycode = XKeysymToKeycode(_display, keysym);
    return !!(keys[keycode / 8] & (1 << (keycode % 8)));
  };
  switch (modifier) {
    case EModifier::shift: return key_pressed(XK_Shift_L) || key_pressed(XK_Shift_R);
    case EModifier::control: return key_pressed(XK_Control_L) || key_pressed(XK_Control_R);
    case EModifier::alt: return key_pressed(XK_Alt_L) || key_pressed(XK_Alt_R);
    default: assertnever("");
  }
}

void Hw::set_color_to_foreground() {
  assertx(_state == EState::open);
  if (_oglx) {
#if defined(HH_OGLX)
    glColor3ubv(_color_foreground.data());
#endif
  } else {
    XSetForeground(_display, _gc, _pixel_foreground);
  }
}

void Hw::set_color(const Pixel& pix) {
  assertx(_state == EState::open);
  if (_oglx) {
#if defined(HH_OGLX)
    glColor4ubv(pix.data());
#endif
  } else {
    XColor xcolor;
    xcolor.red = pix[0] * 256;
    xcolor.green = pix[1] * 256;
    xcolor.blue = pix[2] * 256;
    xcolor.flags = DoRed | DoGreen | DoBlue;
    XAllocColor(_display, _cmap, &xcolor);
    XSetForeground(_display, _gc, xcolor.pixel);
  }
}

void Hw::draw_text_internal(const Vec2<int>& yx, const string& s) {
  assertx(_state == EState::open);
  if (_oglx) {
    draw_text_ogl(yx, s);
  } else {
    XDrawImageString(_display, _draw, _gc, yx[1], yx[0], s.c_str(), narrow_cast<int>(s.size()));
  }
}

void Hw::fill_polygon(CArrayView<Vec2<float>> points) {
  assertx(_state == EState::open);
  if (_oglx) {
    fill_polygon_ogl(points);
  } else {
    Array<XPoint> xpoints(points.num());
    for_int(i, points.num()) {
      xpoints[i].x = int(floor(points[i][1]));
      xpoints[i].y = int(floor(points[i][0]));
    }
    // shape is: Convex, Nonconvex, or Complex
    XFillPolygon(_display, _draw, _gc, xpoints.data(), xpoints.num(), Convex, CoordModeOrigin);
  }
}

void Hw::set_window_title(string ps) {
  assertx(_state == EState::init || _state == EState::open);
  _window_title = std::move(ps);
  if (_state == EState::open) {
    if (1) XStoreName(_display, _win, _window_title.c_str());
    if (0) {
      XTextProperty prop;
      if (0) {
        prop.value = reinterpret_cast<uchar*>(const_cast<char*>(_window_title.c_str()));
        prop.encoding = XA_STRING;
        prop.format = 8;
        prop.nitems = strlen((char*)prop.value);
      } else {
        Vec1<char*> ar(const_cast<char*>(_window_title.c_str()));
        XStringListToTextProperty(ar.data(), ar.num(), &prop);
      }
      XSetWMName(_display, _win, &prop);
    }
    if (0) {
      // https://stackoverflow.com/questions/23273292/how-to-change-net-wm-name-x-library
      XChangeProperty(_display, _win, XInternAtom(_display, "_NET_WM_NAME", False),
                      XInternAtom(_display, "UTF8_STRING", False), 8, PropModeReplace,
                      reinterpret_cast<uchar*>(const_cast<char*>(_window_title.c_str())), _window_title.size());
    }
    XFlush(_display);
  }
}

Vec2<int> Hw::get_max_window_dims() {
  XWindowAttributes attribs;
  assertx(XGetWindowAttributes(_display, DefaultRootWindow(_display), &attribs));
  // cygwin: ignores taskbar and returns V(1600, 2560).
  // SHOW(attribs.x, attribs.y, attribs.height, attribs.width);
  // SHOW(DisplayWidth(_display, _screen), DisplayHeight(_display, _screen));  // same
  // Vec2<int> window_borders = V(55, 0);  // taskbar at bottom in Ubuntu Cinnamon
  Vec2<int> window_borders = V(55, 0);  // taskbar at bottom in Debian Linux Cinnamon (Window borders Albatross)
  return V(attribs.height, attribs.width) - window_borders;
}

void Hw::resize_window(const Vec2<int>& yx) {
  assertx(_state == EState::open);
  if (_hwdebug) SHOW("Hw resize", yx);
  XResizeWindow(_display, _win, yx[1], yx[0]);
  // XMoveResizeWindow(_display, _win, new_left, new_top, yx[1], yx[0])
}

bool Hw::is_fullscreen() { return _is_fullscreen; }

void Hw::make_fullscreen(bool b) {
  if (b == _is_fullscreen) return;
  _is_fullscreen = b;
#if defined(__cygwin__) || defined(__APPLE__)
  const bool use_change_property = true;
#else
  const bool use_change_property = false;  // Linux
#endif
  if (use_change_property) {
    // https://stackoverflow.com/questions/9083273/x11-fullscreen-window-opengl
    // https://specifications.freedesktop.org/wm-spec/1.3/ar01s05.html
    // Under cygwin, it seems to detect _NET_WM_STATE_FULLSCREEN only the first time it is set.
    // On Mac, it doesn't seem to do anything.
    static Vec2<int> bu_dims;
    if (b) {
      bu_dims = _win_dims;
      Vec1<Atom> atoms = {XInternAtom(_display, "_NET_WM_STATE_FULLSCREEN", False)};
      XChangeProperty(_display, _win, XInternAtom(_display, "_NET_WM_STATE", False), XA_ATOM, 32, PropModeReplace,
                      reinterpret_cast<uchar*>(atoms.data()), atoms.num());
      // The second time this property is set with cygwin, the window size does not change.
    } else {
      Vec0<Atom> atoms = {};  // was { None };
      XChangeProperty(_display, _win, XInternAtom(_display, "_NET_WM_STATE", False), XA_ATOM, 32, PropModeReplace,
                      reinterpret_cast<uchar*>(atoms.data()), atoms.num());
      // cygwin ignores the fact that the property is gone above?
      resize_window(bu_dims);
      bu_dims = twice(0);
    }
  } else {
    // https://mail.gnome.org/archives/metacity-devel-list/2010-February/msg00000.html
    // This does not work under cygwin.
    // On Mac, it makes window fullscreen but leads to OpenGL error and shows black until later refresh
    //  the first time fullscreen is invoked.  Now on Sierra 10.12.4 XQuartz 2.7.11 it hangs badly.
    XEvent event;
    event.type = ClientMessage;
    event.xclient.display = _display;
    event.xclient.window = _win;
    // event.xclient.serial        = 0;
    // event.xclient.send_event    = True;
    event.xclient.message_type = XInternAtom(_display, "_NET_WM_STATE", False);
    event.xclient.format = 32;
    event.xclient.data.l[0] = b ? 1 : 0;  // 0 == unset, 1 == set, 2 == toggle
    event.xclient.data.l[1] = XInternAtom(_display, "_NET_WM_STATE_FULLSCREEN", False);
    event.xclient.data.l[2] = 0;
    long mask = SubstructureRedirectMask | SubstructureNotifyMask;
    assertx(XSendEvent(_display, XDefaultRootWindow(_display), False, mask, &event));
    XFlush(_display);
  }
}

void Hw::flush_seg() {
  if (!_ar_seg.num()) return;
  if (_oglx) {
#if defined(HH_OGLX)
    flush_seg_ogl();
#endif
  } else {
    Array<XSegment> ar_xseg(_ar_seg.num());
    for_int(i, _ar_seg.num()) {
      ar_xseg[i].x1 = int(floor(_ar_seg[i][0][1]));
      ar_xseg[i].y1 = int(floor(_ar_seg[i][0][0]));
      ar_xseg[i].x2 = int(floor(_ar_seg[i][1][1]));
      ar_xseg[i].y2 = int(floor(_ar_seg[i][1][0]));
    }
    XDrawSegments(_display, _draw, _gc, ar_xseg.data(), ar_xseg.num());
    _ar_seg.init(0);
  }
}

void Hw::flush_point() {
  if (!_ar_point.num()) return;
  if (_oglx) {
#if defined(HH_OGLX)
    flush_point_ogl();
#endif
  } else {
    Array<XPoint> ar_xpoint(_ar_point.num());
    for_int(i, _ar_point.num()) {
      ar_xpoint[i].x = int(floor(_ar_point[i][1]));
      ar_xpoint[i].y = int(floor(_ar_point[i][0]));
    }
    XDrawPoints(_display, _draw, _gc, ar_xpoint.data(), ar_xpoint.num(), CoordModeOrigin);
    _ar_point.init(0);
  }
}

void Hw::hard_flush() {
  assertx(_state == EState::open);
  soft_flush();
  if (_oglx) {
#if defined(HH_OGLX)
    glXWaitGL();
#endif
  }
  XFlush(_display);
}

void Hw::begin_draw_visible() {
  assertx(_state == EState::open);
  soft_flush();
  if (_oglx) {
#if defined(HH_OGLX)
    glDrawBuffer(GL_FRONT);
#endif
  } else if (_is_pixbuf) {
    _draw = _win;
  }
}

void Hw::end_draw_visible() {
  assertx(_state == EState::open);
  soft_flush();
  if (_oglx) {
#if defined(HH_OGLX)
    if (_is_glx_dbuf) glDrawBuffer(GL_BACK);
#endif
  } else if (_is_pixbuf) {
    _draw = _bbuf;
  }
}

void Hw::wake_up() {
  // Warning("untested");
  // https://stackoverflow.com/questions/10785491/how-to-allow-a-worker-thread-to-updata-an-x11-window
  // https://ubuntuforums.org/archive/index.php/t-570702.html
  // https://stackoverflow.com/questions/8592292/how-to-quit-the-blocking-of-xlibs-xnextevent
  if (!_exposed) return;
  if (0) {
    // "Xlib is not threadsafe and calling its functions from more than one thread may be dangerous!"
    // "For some reason it works stably if the two threads both call XOpenDisplay and use separate
    //   display* structures."
    XEvent event = {};
    event.type = Expose;
    event.xexpose.window = _win;
    XSendEvent(_display, _win, False, ExposureMask, &event);
    XFlush(_display);
  }
  // Instead, rely on timeout within select() call in Hw::loop().
  redraw_later();
}

}  // namespace hh
