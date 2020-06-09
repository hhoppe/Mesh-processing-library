// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "HW.h"

#if defined(_MSC_VER) && (NTDDI_VERSION >= NTDDI_WINBLUE)
#include <ShellScalingApi.h>  // SetProcessDpiAwareness(), if (NTDDI_VERSION >= NTDDI_WINBLUE)
#endif
#include <commdlg.h>   // GetOpenFileNameW()
#include <shellapi.h>  // DragAcceptFiles()

HH_REFERENCE_LIB("opengl32.lib");
HH_REFERENCE_LIB("glu32.lib");
HH_REFERENCE_LIB("winmm.lib");     // timeEndPeriod, etc.
HH_REFERENCE_LIB("comdlg32.lib");  // GetOpenFilenameW(), GetSaveFileNameW()
HH_REFERENCE_LIB("shcore.lib");    // SetProcessDpiAwareness()

#include <cstring>  // std::memset()
#include <mutex>    // std::once_flag, std::call_once()

#include "libHh/Args.h"
#include "libHh/Array.h"
#include "libHh/Image.h"
#include "libHh/MathOp.h"
#include "libHh/StringOp.h"

// Notes:
//  http://msdn.microsoft.com/en-us/library/windows/desktop/ms633575%28v=vs.85%29.aspx Using Window Classes
//  http://msdn.microsoft.com/en-us/library/windows/desktop/ms632595%28v=vs.85%29.aspx Windows

namespace hh {

// Determine if an application (in this case always a CONSOLE app) was started from a console window
//  (as opposed to launched from some UI).
// http://stackoverflow.com/a/14550262
namespace details {
static int fwbp_pid;
static int fwbp_count;
static BOOL CALLBACK find_win_by_procid(HWND hwnd, LPARAM lp) {
  dummy_use(lp);
  int pid;
  GetWindowThreadProcessId(hwnd, LPDWORD(&pid));
  if (pid == fwbp_pid) fwbp_count++;
  return TRUE;
}
static bool win_started_from_console_aux() {
  static bool result;
  static std::once_flag flag;
  std::call_once(flag, [] {
    // the result seems to be only correct the first time it is run
    result = false;
    fwbp_pid = GetCurrentProcessId();
    if (fwbp_pid) {
      fwbp_count = 0;
      EnumWindows(WNDENUMPROC(find_win_by_procid), 0L);
      result = fwbp_count == 0;
    }
  });
  return result;
}
}  // namespace details
static bool win_started_from_console() { return details::win_started_from_console_aux(); }

const HWND k_bogus_hwnd = HWND(intptr_t{-7});  // clang: reinterpret_cast<HWND>() is not allowed in a constexpr

extern HANDLE g_buf_event_data_available;  // from Buffer.cpp

// Solutions in http://stackoverflow.com/questions/117792/best-method-for-storing-this-pointer-for-use-in-wndproc
//  seems too complicated.  I can assume a single-window model.
static HW* pHW;

bool HW::init_aux(Array<string>& aargs) {
  assertx(_state == EState::uninit);
  _state = EState::init;
  bool minimize = false;
  ParseArgs args(aargs, "HW");
  args.p("-geom[etry", _user_geometry, "geom : set window geometry (e.g. 640x480)");
  args.f("-icon[ic", _iconic, ": set initial window state");
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
  args.other_args_ok();
  args.other_options_ok();
  args.disallow_prefixes();
  if (!args.parse_and_extract(aargs)) {
    _state = EState::uninit;
    return false;
  }
  assertx(aargs.num());
  _argv0 = aargs[0];
  if (minimize) _iconic = true;
  pHW = this;
  //
  _hInstance = GetModuleHandle(nullptr);
  if (0) {
    // Hopefully avoid foreground process priority boost during G3dcmp by making all windows have
    //  higher process priority.
    // Problem 1: SetPriorityClass has parameter ABOVE_NORMAL_PRIORITY_CLASS which is not found in winbase.h
    // For now, try HIGH_PRIORITY_CLASS.
    // Test:
    //   G3dcmp -win1 "-key iij____J" hover.m{,} -key i
    // That does not solve problem.  I notice the following:
    // If the left window is selected using either Alt-TAB or by clicking
    //   into it, it updates much faster than the right window.
    // However, if I just left-click on the title bar, or if I select the left window by clicking its title bar,
    //  that bad behavior does not occur.  Bizarre.
    // The behavior is the same at both NORMAL and HIGH priority classes.
    SHOWL;
    assertx(SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS));
  }
  if (0) {
    // This has not effect, for either NORMAL or HIGH priority classes.
    SHOWL;
    const bool disable = true;
    assertx(SetProcessPriorityBoost(GetCurrentProcess(), disable));
  }
  //
  if (1) {
#if defined(_MSC_VER) && (NTDDI_VERSION >= NTDDI_WINBLUE)
    // This program is taking care of DPI issues; Windows dwm should never rescale the window contents.
    // (default is PROCESS_DPI_UNAWARE; intermediate is PROCESS_SYSTEM_DPI_AWARE.)
    // assertx(SetProcessDpiAwareness(PROCESS_PER_MONITOR_DPI_AWARE) == S_OK);
    // E_ACCESSDENIED if already set by prior call or in *.exe manifest.
    void(SetProcessDpiAwareness(PROCESS_PER_MONITOR_DPI_AWARE));
#else
    // assertx(SetProcessDPIAware());  // this older API is available; should be in user32.lib but not found
#endif
  }
  set_window_title(_argv0);
  set_double_buffering(true);
  if (!win_started_from_console()) _extra_console_visible = true;
#if 1  // || !defined(HH_DEBUG)
  // I don't know why, but if I launch from a shortcut as "minimized", the program terminates unless
  //  "ShowWindow(GetConsoleWindow(), SW_HIDE);" is called as done below.
  //
  // See discussion at
  //  http://stackoverflow.com/questions/493536/can-one-executable-be-both-a-console-and-gui-application
  //  http://blogs.msdn.com/b/oldnewthing/archive/2009/01/01/9259142.aspx
  if (!getenv_bool("SHOW_CONSOLE_WINDOW") && _extra_console_visible && !IsDebuggerPresent()) {
    // http://www.cplusplus.com/forum/beginner/12001/
    // Benefits:
    //  - no change to compilation (i.e., omit: -SUBSYSTEM:windows -ENTRY:mainCRTStartup).
    //  - subprocesses (RFile / WFile / my_system) still have access to console so behave properly.
    // Drawbacks:
    //  - Flashes console window briefly before it disappears;
    //     To counter this, one can create a shortcut that starts minimized.
    _extra_console_visible = false;
    ShowWindow(GetConsoleWindow(), SW_HIDE);
  }
#endif
  return true;
}

void HW::open() {
  assertx(_state == EState::init);
  _state = EState::open;
  // Create a window
  {
    if (_backcolor == "") _backcolor = _default_background;
    if (_forecolor == "") _forecolor = _default_foreground;
    _color_background = parse_color(_backcolor);
    _color_foreground = parse_color(_forecolor);
  }
  Vec2<int> yxpos;
  {
    string s = _user_geometry;
    if (s == "") s = _default_geometry;
    if (sscanf(s.c_str(), "%dx%d+%d+%d", &_win_dims[1], &_win_dims[0], &yxpos[1], &yxpos[0]) == 4) {
    } else if (sscanf(s.c_str(), "%dx%d", &_win_dims[1], &_win_dims[0]) == 2) {
      yxpos = twice(-1);
    } else {
      Warning("Could not parse window geometry string");
      yxpos = twice(-1);
      _win_dims = twice(500);
    }
  }
  ogl_create_window(yxpos);
  if (_offscreen == "") {
    for (;;) {
      if (loop()) break;
    }
  } else {
    _exposed = true;
    // Due to interaction with window events or double-buffering, first draw_it() produces only background color.
    draw_it();
    draw_it();
    // glFlush();
    glFinish();
    // my_sleep(0.2);
    Image image(_win_dims);
    if (!_pbuffer) {
      assertx(!_is_glx_dbuf);
      uint8_t* p = static_cast<uint8_t*>(_bitmap_data);
      for_int(y, image.ysize()) {
        for_int(x, image.xsize()) {
          for_int(z, 3) image[y][x][2 - z] = *p++;  // BGR to RGB
        }
        while ((reinterpret_cast<uintptr_t>(p) & 3) != 0) p++;
      }
    } else {
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
  }
  _state = EState::uninit;
  // Cleanup/un-initialization
  if (_hwdebug) SHOW("HW::open Cleanup");
  assertw(!gl_report_errors());
  assertx(wglMakeCurrent(_hRenderDC, nullptr));
  assertx(wglDeleteContext(_hRC));
  if (_offscreen != "" && _pbuffer) {
    // I did not bother putting these in.
    // wglReleasePbufferDCARB(hbuf, _hRenderDC);
    // wglDestroyPbufferARB(hbuf);
  }
  if (_hRenderDC != _hDC) assertx(DeleteDC(_hRenderDC));
  if (_hDC) assertx(ReleaseDC(_hwnd, _hDC));
  assertx(DestroyWindow(_hwnd));
}

bool HW::loop() {
  for (;;) {
    // handle all possible types of events
    if (_update == EUpdate::quit) return true;
    handle_events();  // WM_ MESSAGES
    if (_update == EUpdate::quit) return true;
    handle_keyintr();  // non-interactive key input
    if (_update == EUpdate::quit) return true;
    if (!got_event() && !_is_keyintr) {
      // no more events, so possibly blocked
      if (_update != EUpdate::nothing) break;
      // We've really got nothing to do, so let's pause to avoid CPU usage when nothing is happening.
      if (_hwkey != "") {
        // it would be best if callbackSimuKey() would post a message so we could wait in the MsgWait.
        const double delay_no_activity = 0.001;  // how long to sleep (in seconds) when there's no activity
        my_sleep(delay_no_activity);
        continue;
      }
      if (_watch_fd0) {
        DWORD ret =
            MsgWaitForMultipleObjectsEx(1, &g_buf_event_data_available, INFINITE, QS_ALLEVENTS, MWMO_INPUTAVAILABLE);
        assertx(ret != WAIT_FAILED);
        if (ret == WAIT_OBJECT_0) input_received();
      } else {
        assertx(WaitMessage());
      }
    }
  }
  // No more events but redraw has been requested
  // *Might* need to check if '_exposed' is true, and only do draw_it() if is true (and else, do a my_sleep() here).
  draw_it();
  // This improves load-balancing a bit in G3dcmp (see also discussion on SetPriorityClass)
  //  and doesn't seem to hurt performance too much for single window.
  if (1) SleepEx(0, TRUE);  // 0 milliseconds; relinquish time slice
  return false;
}

void HW::handle_events() {
  while (got_event() && _update != EUpdate::quit) {
    handle_event();
  }
}

void HW::handle_event() {
  _gotevent = false;
  TranslateMessage(&_msg);
  DispatchMessage(&_msg);
  // Won't return from here until the window proc finishes processing message -- so ok.
}

// This wrapper to avoid the problem where can't access 'HW' class members from a non-class function -- yuck!
LRESULT CALLBACK wndProc_wrapper(HWND hwnd, UINT iMsg, WPARAM wParam, LPARAM lParam) {
  if (0) SHOW(hwnd, iMsg, wParam, lParam);
  if (pHW->_hwnd == k_bogus_hwnd) pHW->_hwnd = hwnd;
  assertx(hwnd == pHW->_hwnd);
  return pHW->wndProc(iMsg, wParam, lParam);
}

// Windows 7 Touch Input:
//  http://msdn.microsoft.com/en-us/library/windows/desktop/dd371581%28v=vs.85%29.aspx RegisterTouchWindow
//  http://msdn.microsoft.com/en-us/library/windows/desktop/dd317341%28v=vs.85%29.aspx WM_TOUCH message
//  http://msdn.microsoft.com/en-us/library/windows/desktop/dd744775%28v=vs.85%29.aspx Detecting and Tracking
//  http://msdn.microsoft.com/en-us/library/windows/desktop/dd317334%28v=vs.85%29.aspx TOUCHINPUT structure
//  http://msdn.microsoft.com/en-us/library/windows/desktop/dd562199%28v=vs.85%29.aspx samples
//  http://msdn.microsoft.com/en-us/magazine/ee336016.aspx overview

LRESULT HW::wndProc(UINT iMsg, WPARAM wParam, LPARAM lParam) {
  if (_hwdebug) SHOW("wndProc", iMsg);
  static PAINTSTRUCT ps;  // always zero
  switch (iMsg) {
    case WM_ERASEBKGND:
      // http://www.opengl.org/pipeline/article/vol003_7/
      // Handle the application window's WM_ERASEBKGND by returning non-zero in the message handler
      // (this will avoid GDI clearing the OpenGL windows background).
      // HH: I found this unnecessary so far.
      return 1;  // introduced 20141203
    case WM_CREATE:
      // On window creation, send a WM_SHOWWINDOW msg,
      //  since that msg doesn't get sent when ShowWindow() is called, in certain situations.
      if (_hwdebug) SHOW("WM_CREATE");
      assertx(PostMessage(_hwnd, WM_SHOWWINDOW, TRUE, 0));
      DragAcceptFiles(_hwnd, TRUE);
      return 0;
    case WM_SHOWWINDOW: {
      BOOL fShow = BOOL(wParam);
      if (_hwdebug) SHOW("WM_SHOWWINDOW", fShow);
      if (fShow) {
        // Window is about to be SHOWN (like X-windows MapNotify)
        if (!_exposed) {
          _exposed = true;
          if (_hwdebug) SHOW("window now exposed");
          if (_hwkey != "") start_hwkey();
        }
      } else {
        // Window is about to be HIDDEN (like X-windows UnmapNotify)
        // I could perhaps somehow force the redraws to stop.
      }
      return 0;
    }
    case WM_PAINT: {
      // (Part of) window requires repaint (like X-windows Expose)
      if (_hwdebug) SHOW("WM_PAINT");
      if (!_exposed) assertnever("");
      // Remove from message queue all other repaint messages.
      // Implicitly "validates" whole window to remove WM_PAINT msgs.
      // Win32 note: only 1 WM_PAINT message will be in queue at a time.
      // To support several OpenGL contexts, see http://stackoverflow.com/questions/2842319/swapbuffers-causes-redraw
      assertx(BeginPaint(_hwnd, &ps));
      assertx(EndPaint(_hwnd, &ps));
      // Redraw the screen
      redraw_now();
      return 0;
    }
    case WM_SIZE: {
      // Window size has changed (like X-windows ConfigureNotify).
      // Might want to do some tricks (for efficiency only) to see if we _really_ need to resize the window, like:
      //  if (wParam == SIZE_MAXHIDE || wParam == SIZE_MAXHIDE)...
      // Update value of '_exposed' flag
      if (wParam == SIZE_MAXIMIZED || wParam == SIZE_RESTORED) {
        _exposed = true;
      } else if (wParam == SIZE_MINIMIZED) {
        _exposed = false;
      }
      // This is size of _client_ area.  Do we need to subtract window border, scrollbars, etc?  No.
      _win_dims = convert<int>(V(HIWORD(lParam), LOWORD(lParam)));  // new (height, width) of client area
      if (_hwdebug) SHOW("WM_SIZE", _win_dims);
      if (!_exposed) return 0;
      soft_discard();
      glViewport(0, 0, _win_dims[1], _win_dims[0]);
      if (1) {
        // Remove from message queue all other repaint messages.
        // Implicitly "validates" whole window to remove WM_PAINT msgs.
        // Win32 note: only 1 WM_PAINT message will be in queue at a time.
        assertx(BeginPaint(_hwnd, &ps));
        assertx(EndPaint(_hwnd, &ps));
      }
      // The problem in windows is that there is no way to avoid seeing a flash of background color right
      //  after window resizing (most noticeable in Remote Desktop).  Good discussion at
      //  http://stackoverflow.com/questions/9786218/drawing-in-window-while-resizing-leaves-unpainted-border
      if (0) {
        // This clears the new window completely, which makes the problem somewhat worse.
        {
          Vector4 v(_color_background);
          glClearColor(v[0], v[1], v[2], 1.f);
        }
        glDrawBuffer(GL_FRONT_AND_BACK);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (_is_glx_dbuf) glDrawBuffer(GL_BACK);
        set_color_to_foreground();
      }
      if (0 && gl_extensions_string() != "") {  // only if OpenGL is already initialized.
        // This draws directly to the front buffer.  However, the window is already painted white.
        glDrawBuffer(GL_FRONT);
        bool bu_is_glx_dbuf = _is_glx_dbuf;
        _is_glx_dbuf = false;
        draw_it();
        _is_glx_dbuf = bu_is_glx_dbuf;
      } else {
        redraw_now();  // redraw it later; works just as well
      }
      return 0;
    }
    case WM_MOVE: {
      _win_pos = convert<int>(V(HIWORD(lParam), LOWORD(lParam)));  // y, x
      if (_hwdebug) SHOW("WM_MOVE", _win_pos);
      return 0;
    }
    case WM_KEYDOWN:
    case WM_CHAR:
      if (_hwdebug) SHOW("WM_KEYDOWN/WM_CHAR", wParam);
      handle_key(iMsg, wParam);
      return 0;
    case WM_LBUTTONDOWN:
    case WM_LBUTTONUP:
    case WM_MBUTTONDOWN:
    case WM_MBUTTONUP:
    /**/ case WM_RBUTTONDOWN:
    case WM_RBUTTONUP:
    case WM_XBUTTONDOWN:
    case WM_XBUTTONUP: {
      // cursor relative to upper-left of client area
      const Vec2<int> yx = convert<int>(V(HIWORD(lParam), LOWORD(lParam)));
      const bool shift = (wParam & MK_SHIFT) != 0;
      dummy_use(shift);
      int butnum;
      switch (iMsg) {
        case WM_LBUTTONDOWN:
        case WM_LBUTTONUP: butnum = 1; break;
        case WM_MBUTTONDOWN:
        case WM_MBUTTONUP: butnum = 2; break;
        case WM_RBUTTONDOWN:
        case WM_RBUTTONUP: butnum = 3; break;
        case WM_XBUTTONDOWN:
        case WM_XBUTTONUP: butnum = 3 + HIWORD(wParam); break;  // thus 4 or 5
        default: assertnever("");
      }
      bool pressed;
      switch (iMsg) {
        case WM_LBUTTONDOWN:
        case WM_MBUTTONDOWN:
        case WM_RBUTTONDOWN:
        case WM_XBUTTONDOWN: pressed = true; break;
        case WM_LBUTTONUP:
        case WM_MBUTTONUP:
        case WM_RBUTTONUP:
        case WM_XBUTTONUP: pressed = false; break;
        default: assertnever("");
      }
      if (_hwdebug) SHOW("WM_BUTTON", butnum, pressed, yx);
      if (pressed)
        SetCapture(_hwnd);
      else
        assertw(ReleaseCapture());
      button_press(butnum, pressed, yx);
      return 0;
    }
    case WM_MOUSEWHEEL: {
      bool shift = (wParam & MK_SHIFT) != 0;
      dummy_use(shift);
      int wheel_motion = HIWORD(wParam);
      if (wheel_motion >= (1 << 15)) wheel_motion = wheel_motion - (1 << 16);
      if (_hwdebug) SHOW("WM_MOUSEWHEEL", wheel_motion);
      wheel_turn(wheel_motion / 120.f);  // wheel_motion == 120 for one click turn
      return 0;
    }
    case WM_DROPFILES: {
      // See http://www.codeproject.com/Articles/840/How-to-Implement-Drag-and-Drop-Between-Your-Progra
      Array<string> filenames;
      {
        HDROP hdrop = HDROP(wParam);
        // POINT pt; DragQueryPoint(hdrop, &pt); SHOW(pt.x, pt.y);
        int cFiles = int(DragQueryFileW(hdrop, unsigned(-1), nullptr, 0));
        filenames.init(cFiles);
        for_int(i, cFiles) {
          Vec<wchar_t, 2000> filename;
          assertx(DragQueryFileW(hdrop, i, filename.data(), filename.num() - 1));
          filenames[i] = get_canonical_path(narrow(filename.data()));
          // TextOutW(hdc, pt.x, pt.y, filename.data(), lstrlenW(filename)); pt.y += 20;
        }
        DragFinish(hdrop);
      }
      if (_hwdebug) SHOW("WM_DROPFILES", filenames);
      drag_and_drop(filenames);
      grab_focus();
      return 0;
    }
    case WM_CLOSE:
      if (_hwdebug) SHOW("WM_CLOSE");
      quit();
      return 0;
    case WM_DESTROY:
      if (_hwdebug) SHOW("WM_DESTROY");
      DragAcceptFiles(_hwnd, FALSE);
      break;
    case WM_GETMINMAXINFO: {
      // Try to override the smallest width to < 104 interior pixels.
      //  (GetSystemMetrics(SM_CXMIN)=112 - 2 * 4 == 104).
      // The fix below seems to allow small geometry from command-line,
      //   but not when resizing window (min width 100 on Windows 7).  I can live with this.
      MINMAXINFO* p = reinterpret_cast<MINMAXINFO*>(lParam);
      p->ptMinTrackSize.x = 20;  // was previously 50
      p->ptMinTrackSize.y = 20;  // no effect
      if (_hwdebug) SHOW("WM_GETMINMAXINFO");
      return 0;
    }
      // case WM_DPICHANGED:
      // do nothing currently
      // break;
    default:
      void();
      // unrecognized message, pass through
  }  // end switch
  // If got here, didn't handle the message, so pass on to default handler.
  return DefWindowProcW(_hwnd, iMsg, wParam, lParam);
}

bool HW::got_event() {
  // If there's a pending event, retrieve it and set flag
  //  (_unless_ we already have an event; in that case, don't fetch another one).
  if (!_gotevent && PeekMessage(&_msg, nullptr, 0, 0, PM_REMOVE)) _gotevent = true;
  return _gotevent;
}

bool HW::suggests_stop() {
  assertx(_state == EState::open);
  if (got_event() && _update != EUpdate::quit) handle_events();
  if (_is_keyintr) handle_keyintr();
  return _update == EUpdate::quit || _update == EUpdate::redrawnow;
}

// Warning: Can't call any system-defined functions inside a multimedia-timer callback,
//  except for PostMessage, OutputDebugString, certain 'timeXXXX' functions, and certain 'midiOutXXXX' functions.
//  (See 'TimeProc' documentation for full details.)
// Only called by start_hwkey()
void CALLBACK callbackSimuKey(UINT id, UINT /*unused*/, DWORD_PTR userData, DWORD_PTR /*unused*/,
                              DWORD_PTR /*unused*/) {
  dummy_use(id);
  // Set flag, meaning that a "new" keypress (simulated) has occurred.
  // (Basically just "opens the gates", so msg loop will process next simulated key now.
  //  Gives periodic/delayed keypresses this way, instead of all being processed at once -- more like real input).
  reinterpret_cast<HW*>(userData)->set_keyintr();
}

void HW::start_hwkey() {
  // *** TURN-ON timer (periodic) ***
  UINT timer_delay;    // How long b/w timer events (mSec)
  TIMECAPS time_caps;  // timer capabilities
  // Check what resolution, etc the timer supports
  assertx(timeGetDevCaps(&time_caps, sizeof(time_caps)) == TIMERR_NOERROR);
  // Clamp desired timer resolution to allowed range
  _sk_timerResolution = clamp(_sk_timerResolution, time_caps.wPeriodMin, time_caps.wPeriodMax);
  // Set our req'd timer resolution
  assertx(timeBeginPeriod(_sk_timerResolution) == TIMERR_NOERROR);
  // Start/create the timer
  _hwdelay = max(_hwdelay, .01f);
  timer_delay = int(_hwdelay * 1000.f + .5f);
  _sk_timerID = timeSetEvent(timer_delay, _sk_timerResolution, callbackSimuKey, reinterpret_cast<DWORD_PTR>(this),
                             TIME_PERIODIC);
  assertx(_sk_timerID);
}

void HW::end_hwkey() {
  // *** TURN-OFF timer (periodic) ***
  if (_sk_timerID) {
    assertx(timeKillEvent(_sk_timerID) == TIMERR_NOERROR);
    _sk_timerID = 0;
  }
  if (_sk_timerResolution) {
    assertx(timeEndPeriod(_sk_timerResolution) == TIMERR_NOERROR);
    _sk_timerResolution = 0;
  }
}

void HW::handle_key(int why_called, WPARAM key_data) {
  string s;
  // SHOW(why_called, why_called == WM_KEYDOWN, int(key_data));
  if (why_called == WM_KEYDOWN) {
    int virt_key = int(key_data);
    if (virt_key >= VK_F1 && virt_key <= VK_F12) {
      s = sform("<f%d>", virt_key - VK_F1 + 1);
    } else {
      switch (virt_key) {
        case VK_LEFT: s = "<left>"; break;
        case VK_RIGHT: s = "<right>"; break;
        case VK_UP: s = "<up>"; break;
        case VK_DOWN: s = "<down>"; break;
        case VK_HOME: s = "<home>"; break;
        case VK_END: s = "<end>"; break;
        case VK_PRIOR: s = "<prior>"; break;
        case VK_NEXT: s = "<next>"; break;
        case VK_INSERT: s = "<insert>"; break;
        case VK_DELETE: s = "<delete>"; break;
        default:
          if (get_key_modifier(EModifier::control) && virt_key >= '0' && virt_key <= '9') {
            // control-numbers (C-0 .. C-9) do not produce any subsequent WM_CHAR message, so handle now.
            char ch = assert_narrow_cast<char>(virt_key);
            s = ch;
          } else {
            return;  // we don't care about this key, or it will be handled later (by WM_CHAR)
          }
      }
    }
  } else {
    // we assume we're handling a WM_CHAR message
    char ch = assert_narrow_cast<char>(key_data);
    if (ch == 127) ch = '\b';  // C-<backspace> should just be <backspace> for now
    s = ch;
  }
  assertx(s != "");
  if (1) {
    if (_query) {
      query_keypress(s);
    } else if (key_press(s)) {
      // client has handled key press
    } else if (s == "\033") {  // <esc> key ( == uchar{27})
      quit();
    } else if (s == "~") {  // toggle console window
      if (!win_started_from_console()) {
        _extra_console_visible = !_extra_console_visible;
        const int show_flag = 0 ? SW_SHOWNORMAL : SW_SHOWNOACTIVATE;
        ShowWindow(GetConsoleWindow(), _extra_console_visible ? show_flag : SW_HIDE);
      }
    } else {
      beep();
    }
  }
}

void HW::draw_it() {
  _update = EUpdate::nothing;
  if (!_exposed) return;
  if (_hwdebug) SHOW("draw_it", _win_dims);
  _async = true;
  if (_query) {
    clear_window();
    draw_text(_query_yx, _query_prompt + _query_buffer + "_");
  } else {
    // 2014-12-13
    glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);   // for non-premultiplied alpha
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
    if (_is_glx_dbuf) {
      // glFlush();
      // assertx(SwapBuffers(_hDC));
      assertx(SwapBuffers(_hRenderDC));
    } else {
      glFlush();
    }
  } else {
    soft_discard();
  }
}

void HW::clear_window() {
  soft_discard();
  clear_window_ogl();
}

void HW::beep() {
  assertx(_state != EState::uninit);
  Beep(600, 100);
}

void HW::set_double_buffering(bool newstate) {
  if (_offscreen != "") newstate = true;  // else multisampling is not supported (bizarre)
  if (!assertw(_state == EState::init)) return;
  _is_glx_dbuf = newstate;
  // if !_is_glx_dbuf, do not call glXSwapBuffers(), glDrawBuffer(GL_BACK)
  if (_hwdebug) SHOW(_is_glx_dbuf);
}

bool HW::get_pointer(Vec2<int>& yx) {
  assertx(_state == EState::open);
  POINT pt_screen, pt_client;
  // Get position of mouse cursor (relative to screen, not window).
  if (!GetCursorPos(&pt_screen)) return false;
  // Convert coords so that they're relative to window's client area.
  pt_client = pt_screen;
  assertx(ScreenToClient(_hwnd, &pt_client));
  // Assign values to the [OUT] arguments.
  yx = convert<int>(V(pt_client.y, pt_client.x));
  // Return True if mouse pointer is inside window, or False if isn't
  //  (to be consistent with X-windows -- recall that X-windows only activates window if pointer is on it).
  RECT window_rect;
  assertx(GetWindowRect(_hwnd, &window_rect));
  // return PtInRect(&window_rect, pt_screen);
  return true;
}

bool HW::get_key_modifier(EModifier modifier) {
  int virt_key;
  switch (modifier) {
    case EModifier::shift: virt_key = VK_SHIFT; break;
    case EModifier::control: virt_key = VK_CONTROL; break;
    case EModifier::alt: virt_key = VK_MENU; break;
    default: assertnever("");
  }
  short v = GetKeyState(virt_key);
  return !!(v & (1 << 15));  // high-order bit indicates key down, low-order bit is odd "toggle" state for caps_lock
}

void HW::set_color_to_foreground() {
  assertx(_state == EState::open);
  glColor3ubv(_color_foreground.data());
}

void HW::set_color(const Pixel& pix) {
  assertx(_state == EState::open);
  glColor4ubv(pix.data());
}

void HW::draw_text_internal(const Vec2<int>& yx, const string& s) {
  assertx(_state == EState::open);
  draw_text_ogl(yx, s);
}

void HW::fill_polygon(CArrayView<Vec2<float>> points) {
  assertx(_state == EState::open);
  fill_polygon_ogl(points);
}

void HW::set_window_title(string ps) {
  assertx(_state == EState::init || _state == EState::open);
  _window_title = std::move(ps);
  // std::wcerr << widen(_window_title) << widen("\n");
  if (_state == EState::open) {
    ASSERTX(IsWindowUnicode(_hwnd));
    assertx(SetWindowTextW(_hwnd, widen(_window_title).c_str()));
    // Note that this gets converted to a WM_SETTEXT message;
    //  see http://stackoverflow.com/questions/9410681/setwindowtextw-in-an-ansi-project
  }
}

Vec2<int> HW::get_max_window_dims() {
  RECT wa_rect;
  assertx(SystemParametersInfo(SPI_GETWORKAREA, 0, &wa_rect, 0));
  // The above does subtract the taskbar area.
  // SHOW(wa_rect.top, wa_rect.bottom, wa_rect.left, wa_rect.right);
  //
  // Use Win->Up_arrow to maximize window height, then use '@' key to compare:
  //  get_max_window_dims() = [1573, 2464]
  //  g_win_dims=[1568, 2360]
  Vec2<int> top = V(16, 0);  // Windows 7
  top = V(19, 0);            // Windows 10
  top = V(24, 0);            // Windows 10 update1511
  auto window_borders =
      top + V(2 * (3 + 1 + 0), 2 * (3 + 1 + 0));  // iBorderWidth default 1, +iPaddedBorderWidth (def 4, set 0)
  return V(int(wa_rect.bottom - wa_rect.top), int(wa_rect.right - wa_rect.left)) - window_borders;
}

void HW::resize_window(const Vec2<int>& yx) {
  assertx(_state == EState::open);
  // http://stackoverflow.com/questions/5609486/in-c-windows-api-resize-window-during-runtime
  // http://stackoverflow.com/questions/692742/how-do-you-programmatically-resize-and-move-windows-with-the-windows-api
  RECT wa_rect;
  assertx(SystemParametersInfo(SPI_GETWORKAREA, 0, &wa_rect, 0));
  RECT rect = {0, 0, yx[1], yx[0]};  // left, top, right, bottom
  assertx(AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS, 0));
  int width = rect.right - rect.left;
  int height = rect.bottom - rect.top;
  // or consider MoveWindow()
  // http://msdn.microsoft.com/en-us/library/windows/desktop/ms633545%28v=vs.85%29.aspx
  int new_top = 0, new_left = 0;  // unused unless force_move
  bool force_move = false;
  if (_win_pos[0] + height > wa_rect.bottom || _win_pos[1] + width > wa_rect.right) {
    new_top = wa_rect.top;
    new_left = wa_rect.left;
    force_move = true;
  }
  // SHOW(new_top, new_left);
  if (_hwdebug) SHOW("resize", yx, width, height);
  // SWP_NOCOPYBITS?  SWP_DEFERERASE?  SWP_SHOWWINDOW?  SWP_NOREDRAW?  remove SWP_NOACTIVATE?
  assertx(SetWindowPos(_hwnd, nullptr, new_left, new_top, width, height,
                       (force_move ? 0 : SWP_NOMOVE) | SWP_NOZORDER | SWP_NOACTIVATE));
  // Note: Although the initially created window can have arbitrary tiny size,
  //  using SetWindwPos() to resize the window forces a minimum width on the window
  //  (approximately 102 pixels) (for the buttons above it).
}

bool HW::is_fullscreen() {
  DWORD style = GetWindowLong(_hwnd, GWL_STYLE);
  return !(style & WS_OVERLAPPEDWINDOW);
}

void HW::make_fullscreen(bool b) {
  // http://blogs.msdn.com/b/oldnewthing/archive/2010/04/12/9994016.aspx
  if (b == is_fullscreen()) return;
  static WINDOWPLACEMENT g_wp_prev;
  g_wp_prev.length = sizeof(g_wp_prev);
  DWORD style = GetWindowLong(_hwnd, GWL_STYLE);
  if (b) {  // go full screen
    MONITORINFO mi;
    mi.cbSize = sizeof(mi);
    if (GetWindowPlacement(_hwnd, &g_wp_prev) &&
        GetMonitorInfo(MonitorFromWindow(_hwnd, MONITOR_DEFAULTTOPRIMARY), &mi)) {
      SetWindowLong(_hwnd, GWL_STYLE, style & ~WS_OVERLAPPEDWINDOW);
      SetWindowPos(_hwnd, HWND_TOP, mi.rcMonitor.left, mi.rcMonitor.top, mi.rcMonitor.right - mi.rcMonitor.left,
                   mi.rcMonitor.bottom - mi.rcMonitor.top, SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
    }
  } else {  // come back from full screen
    SetWindowLong(_hwnd, GWL_STYLE, style | WS_OVERLAPPEDWINDOW);
    SetWindowPlacement(_hwnd, &g_wp_prev);
    SetWindowPos(_hwnd, nullptr, 0, 0, 0, 0,
                 SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
  }
}

void HW::grab_focus() {
  // assertx(SetFocus(_hwnd));
  // assertw(SetActiveWindow(_hwnd));
  assertw(SetForegroundWindow(_hwnd));
}

void HW::flush_seg() { flush_seg_ogl(); }

void HW::flush_point() { flush_point_ogl(); }

void HW::hard_flush() {
  assertx(_state == EState::open);
  soft_flush();
  glFinish();
}

#define VIDEO_EXTS                                                     \
  L"*.mpg;*.mpeg;*.mpe;*.mpv;*.mp2;*.mpeg2;*.mp4;*.mpeg4;*.mov;*.avi;" \
  L"*.wmv;*.flv;*.m2v;*.m4v;*.webm;*.ogv;*.3gp;*.mts;*.gif"
#define IMAGE_EXTS                                                  \
  L"*.jpg;*.jpeg;*.png;*.bmp;*.rgb;*.ppm;*.pgm;*.pbm;*.tif;*.tiff;" \
  L"*.jxr;*.hdp;*.wdp;*.wmp;*.webp;*.bpg;*.jp2"

Array<string> HW::query_open_filenames(const string& hint_filename) {
  // http://www.winprog.org/tutorial/app_two.html
  std::wstring whint_filename = widen(hint_filename);
  std::wstring whint_directory = widen(replace_all(get_path_head(hint_filename), "/", "\\"));
  std::wstring whint_tail = widen(get_path_tail(hint_filename));
  Array<wchar_t> buffer(64000);
  assertx(whint_tail.size() < buffer.size());
  wcsncpy(buffer.data(), whint_tail.c_str(), buffer.size());
  OPENFILENAMEW ofn;
  {
    std::memset(&ofn, 0, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = _hwnd;
    ofn.lpstrFilter = (L"Video and Images\0" VIDEO_EXTS L";" IMAGE_EXTS L"\0"
                           L"Video Files\0" VIDEO_EXTS L"\0"
                           L"Image Files\0" IMAGE_EXTS L"\0"
                           L"All Files (*.*)\0*.*\0");
    ofn.lpstrFile = buffer.data();
    ofn.nMaxFile = buffer.num();
    ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY | OFN_ALLOWMULTISELECT;
    // ofn.lpstrDefExt = L"txt";
    ofn.lpstrTitle = L"Open File";
    ofn.lpstrInitialDir = whint_directory.c_str();
    if (0) SHOW(narrow(ofn.lpstrInitialDir), narrow(ofn.lpstrFile));
  }
  if (!GetOpenFileNameW(&ofn)) return {};
  // If the OFN_ALLOWMULTISELECT flag is set and the user selects multiple files, the buffer contains
  //  the current directory followed by the file names of the selected files, separated by null characters.
  // If the user selects only one file, the lpstrFile string does not have a separator between the path and
  //  file name.
  Array<string> filenames;
  if (ofn.nFileOffset == 0 || buffer[ofn.nFileOffset - 1]) {  // single file
    filenames.reserve(1);
    filenames.push(get_canonical_path(narrow(buffer.data())));
  } else {  // multiple files
    string directory = get_canonical_path(narrow(buffer.data()));
    for (wchar_t* p = &buffer[int(directory.size() + 1)]; *p;) {
      if (0) SHOW(directory, narrow(p));
      filenames.push(directory + "/" + narrow(p));
      p += wcslen(p) + 1;
    }
  }
  if (0) SHOW(filenames);
  return filenames;
}

string HW::query_save_filename(const string& hint_filename, bool force) {
  // http://msdn.microsoft.com/en-us/library/windows/desktop/ms646928%28v=vs.85%29.aspx
  std::wstring whint_filename = widen(hint_filename);
  std::wstring whint_directory = widen(replace_all(get_path_head(hint_filename), "/", "\\"));
  std::wstring whint_tail = widen(get_path_tail(hint_filename));
  std::wstring whint_extension = widen(get_path_extension(hint_filename));
  Array<wchar_t> buffer(64000);
  assertx(whint_tail.size() < buffer.size());
  wcsncpy(buffer.data(), whint_tail.c_str(), buffer.size());
  OPENFILENAMEW ofn;
  {
    std::memset(&ofn, 0, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = _hwnd;
    ofn.lpstrFilter = (L"All Files (*.*)\0*.*\0");
    ofn.lpstrFile = buffer.data();
    ofn.nMaxFile = buffer.num();
    ofn.Flags = OFN_EXPLORER | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY | (force ? 0 : OFN_OVERWRITEPROMPT);
    ofn.lpstrDefExt = whint_extension.c_str();
    ofn.lpstrTitle = L"Save File";
    ofn.lpstrInitialDir = whint_directory.c_str();
  }
  if (!GetSaveFileNameW(&ofn)) return "";
  assertx(ofn.nFileOffset == 0 || buffer[ofn.nFileOffset - 1]);  // single file
  return get_canonical_path(narrow(buffer.data()));
}

void HW::begin_draw_visible() {
  assertx(_state == EState::open);
  soft_flush();
  glDrawBuffer(GL_FRONT);
}

void HW::end_draw_visible() {
  assertx(_state == EState::open);
  soft_flush();
  if (_is_glx_dbuf) glDrawBuffer(GL_BACK);
}

void HW::wake_up() {
  WPARAM wParam = 0;
  LPARAM lParam = 0;
  if (_exposed) assertw(PostMessage(_hwnd, WM_USER + 0, wParam, lParam));
}

void HW::set_pixel_format(bool fake_first) {
  // http://www.opengl.org/pipeline/article/vol003_7/
  // Windows Vista introduces the new pixelformat flag PFD_SUPPORT_COMPOSITION (defined in the Driver
  //  Development Kit's wingdi.h as 0x00008000).
  // Creating an OpenGL context for a pixelformat without this flag will disable composition for the duration
  //  of the process which created the context.  The flag is mutually exclusive with PFD_SUPPORT_GDI.
#if !defined(PFD_SUPPORT_COMPOSITION)  // e.g. clang
  const unsigned PFD_SUPPORT_COMPOSITION = 0x00008000;
#endif
  PIXELFORMATDESCRIPTOR pfd;
  pfd.nSize = sizeof(pfd);
  pfd.nVersion = 1;
  pfd.dwFlags = PFD_SUPPORT_OPENGL;
  if (fake_first || _offscreen == "") {
    pfd.dwFlags |= PFD_DRAW_TO_WINDOW;
    if (_is_glx_dbuf) pfd.dwFlags |= PFD_DOUBLEBUFFER;
    pfd.dwFlags |= PFD_SUPPORT_COMPOSITION;
  } else if (!_pbuffer) {
    pfd.dwFlags |= PFD_DRAW_TO_BITMAP;
  } else {
    // WGL_DRAW_TO_PBUFFER not recognized by ChoosePixelFormat.
    // We will use wglChoosePixelFormatARB() instead later.
  }
  pfd.iPixelType = PFD_TYPE_RGBA;             // COLORINDEX or RGBA
  pfd.cColorBits = BYTE(_scr_bpp);            // Bits per pixel used for color
  pfd.cAlphaBits = 0;                         // >= 0
  pfd.cAccumBits = 0;                         // >= 0
  pfd.cDepthBits = BYTE(_scr_zbufbits);       // >= 0
  pfd.cStencilBits = BYTE(_scr_stencilbits);  // >= 0
  pfd.cAuxBuffers = 0;                        // >= 0
  pfd.iLayerType = PFD_MAIN_PLANE;            // MAIN or OVERLAY or UNDERLAY
  if (_hwdebug) {
    SHOW("Requested:");
    const PIXELFORMATDESCRIPTOR& p = pfd;
    SHOW(p.dwFlags, unsigned(p.cColorBits), unsigned(p.cAlphaBits), unsigned(p.cAccumBits));
    SHOW(unsigned(p.cDepthBits), unsigned(p.cStencilBits), unsigned(p.cAuxBuffers), unsigned(p.iLayerType));
  }
  int iPixelFormat = assertx(ChoosePixelFormat(_hRenderDC, &pfd));
  // note: pfd is not modified by ChoosePixelFormat.
  if (1) {
    PIXELFORMATDESCRIPTOR pfd2;
    assertx(DescribePixelFormat(_hRenderDC, iPixelFormat, sizeof(pfd2), &pfd2));
    if (_hwdebug) {
      SHOW("Obtained:");
      const PIXELFORMATDESCRIPTOR& p = pfd2;
      SHOW(p.dwFlags, unsigned(p.cColorBits), unsigned(p.cAlphaBits), unsigned(p.cAccumBits));
      SHOW(unsigned(p.cDepthBits), unsigned(p.cStencilBits), unsigned(p.cAuxBuffers), unsigned(p.iLayerType));
    }
    assertx(PFD_SWAP_COPY == 1024);  // dwFlags == 1061 == 1024 + 32 + 4 + 1
    if (!fake_first) {
      unsigned desired = pfd.dwFlags & ~PFD_SUPPORT_COMPOSITION;
      assertw((pfd2.dwFlags & desired) == desired);
      assertw(pfd2.iPixelType == pfd.iPixelType);
      assertw(pfd2.cColorBits >= pfd.cColorBits);
      assertw(pfd2.cAlphaBits >= pfd.cAlphaBits);
      assertw(pfd2.cAccumBits >= pfd.cAccumBits);
      assertw(pfd2.cDepthBits >= pfd.cDepthBits);
      assertw(pfd2.cStencilBits >= pfd.cStencilBits);
      assertw(pfd2.cAuxBuffers >= pfd.cAuxBuffers);
      assertw(pfd2.iLayerType == pfd.iLayerType);
    }
  }
  // Set the pixelFormat (NOTE: Can only do this once per window!)
  assertx(SetPixelFormat(_hRenderDC, iPixelFormat, &pfd));
}

void HW::ogl_create_window(const Vec2<int>& yxpos) {
  WNDCLASSEXW wnd_class;
  // Setup parameters of window class
  wnd_class.cbSize = sizeof(wnd_class);
  wnd_class.style = 0;
  wnd_class.lpfnWndProc = wndProc_wrapper;
  wnd_class.cbClsExtra = 0;
  wnd_class.cbWndExtra = 0;
  wnd_class.hInstance = _hInstance;
  wnd_class.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
  wnd_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
  // hbrBackground was (HBRUSH) GetStockObject(BLACK_BRUSH),  then was nullptr
  wnd_class.hbrBackground = HBRUSH(COLOR_WINDOW + 1);
  wnd_class.lpszMenuName = nullptr;
  std::wstring wargv0 = widen(_argv0);  // must live until the RegisterClassExW() call
  wnd_class.lpszClassName = wargv0.c_str();
  wnd_class.hIconSm = LoadIcon(nullptr, IDI_APPLICATION);
  // wnd_class.hIconSm      = LoadImage(_hInstance, MAKEINTRESOURCE(5), IMAGE_ICON, GetSystemMetrics(SM_CXSMICON),
  //                                    GetSystemMetrics(SM_CYSMICON), LR_DEFAULTCOLOR);
  assertx(RegisterClassExW(&wnd_class));
  RECT rect = {0, 0, _win_dims[1], _win_dims[0]};  // left, top, right, bottom
  assertx(AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS, 0));
  //
  string wgl_extensions;
  _scr_stencilbits = getenv_int("STENCIL_BITS");  // dynamically updated by my_setenv() in G3dOGL.cpp
  {
    _hwnd = k_bogus_hwnd;
    int style = WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS;
    _hwnd = CreateWindowW(widen(_argv0).c_str(), widen(_window_title).c_str(), style, 0, 0, rect.right - rect.left,
                          rect.bottom - rect.top, nullptr, nullptr, _hInstance, nullptr);
    if (!_hwnd) SHOW(GetLastError());
    assertx(_hwnd);
    _hDC = assertx(GetDC(_hwnd));
    _hRenderDC = _hDC;
    // No wglGetProcAddress() is valid before wglMakeCurrent() is called!
    { set_pixel_format(true); }
    _hRC = assertx(wglCreateContext(_hRenderDC));
    assertx(wglMakeCurrent(_hRenderDC, _hRC));
  }
  USE_GL_EXT_MAYBE(wglGetExtensionsStringARB, const char*(WINAPI*)(HDC hdc));
  USE_GL_EXT_MAYBE(wglChoosePixelFormatARB,
                   BOOL(WINAPI*)(HDC hdc, const int* piAttribIList, const FLOAT* pfAttribFList, UINT nMaxFormats,
                                 int* piFormats, UINT* nNumFormats));
  DECLARE_HANDLE(HPBUFFERARB);
  USE_GL_EXT_MAYBE(wglCreatePbufferARB,
                   HPBUFFERARB(WINAPI*)(HDC hDC, int iPixelFormat, int iWidth, int iHeight, const int* piAttribList));
  USE_GL_EXT_MAYBE(wglGetPbufferDCARB, HDC(WINAPI*)(HPBUFFERARB hPbuffer));
  USE_GL_EXT_MAYBE(wglSwapIntervalEXT, BOOL(WINAPI*)(int interval));
  {
    if (wglGetExtensionsStringARB) {
      if (_hwdebug) SHOW("getting wgl_extensions");
      wgl_extensions = assertx(wglGetExtensionsStringARB(_hRenderDC));
    }
    assertx(wglMakeCurrent(_hRenderDC, nullptr));
    assertx(wglDeleteContext(_hRC));
    _hRC = nullptr;
    _hRenderDC = nullptr;
    assertx(ReleaseDC(_hwnd, _hDC));
    _hDC = nullptr;
    assertx(DestroyWindow(_hwnd));
    _hwnd = nullptr;
  }
  if (_hwdebug) SHOW(wgl_extensions);
  if (_offscreen != "") {
    if (wglCreatePbufferARB) {
      _pbuffer = true;
      if (getenv_bool("NO_PBUFFER")) _pbuffer = false;
    }
    if (!_pbuffer) {
      Warning("Using DRAW_TO_BITMAP instead of pbuffer");
      _is_glx_dbuf = false;  // is it too late to do this?
    }
    if (_hwdebug) SHOW(_pbuffer);
  }
  // _multisample == 5 is a bit nicer, but blurs out the text fonts, as does _multisample == 3.
  // So _multisample == 4 is my preferred default.
  _multisample = getenv_int("MULTISAMPLE", 4, true);
  // 0 = none, 2 = 2samples, 3 = 2 + 5tap(Qunicux), 4 = 4 samples, 5 = 4 + 9tap
  if (_multisample == 1) _multisample = 0;
  if (0 && !contains(wgl_extensions, "WGL_ARB_multisample")) _multisample = 0;
  if (_multisample && !contains(wgl_extensions, "WGL_ARB_multisample")) {
    if (wgl_extensions == "") {
      // likely Remote Desktop session.
    } else {
      Warning("Multisample is not available");
    }
    _multisample = 0;
  }
  if (_hwdebug) SHOW(_multisample);
  // Note: yxpos == twice(-1) if unspecified.
  // Note: wndProc_wrapper is called during CreateWindow!
  _hwnd = k_bogus_hwnd;
  int style = WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS;
  if (_iconic) style |= WS_ICONIC;
  if (_maximize) style |= WS_MAXIMIZE;
  _hwnd = CreateWindowW(widen(_argv0).c_str(),         // window class name
                        widen(_window_title).c_str(),  // window title bar
                        style,
                        yxpos[1] < 0 ? CW_USEDEFAULT : yxpos[1],  // initial x position
                        yxpos[0] < 0 ? CW_USEDEFAULT : yxpos[0],  // initial y position
                        // size of entire window; not necessarily client area
                        rect.right - rect.left,  // initial width
                        rect.bottom - rect.top,  // initial height
                        nullptr,                 // parent window handle
                        nullptr,                 // window menu handle
                        _hInstance,              // program instance handle
                        nullptr);                // creation parameters
  // Make sure we were able to create the window.
  assertx(_hwnd);
  // Get a DC for the window (for convenience; let's only do it once)
  _hDC = assertx(GetDC(_hwnd));
  _hRenderDC = _hDC;
  if (_offscreen == "") {
    // Display the window
    ShowWindow(_hwnd, (_iconic ? SW_SHOWMINIMIZED : _maximize ? SW_MAXIMIZE : SW_SHOWDEFAULT));
    assertx(UpdateWindow(_hwnd));
    // Grabbing focus often fails because of focus rules (which is nice).
    SetForegroundWindow(_hwnd);
    if (!_iconic) assertx(SetFocus(_hwnd));
  }
  //
  if (_fullscreen) {
    _fullscreen = false;
    make_fullscreen(true);
  }
  if (_offscreen != "" && !_pbuffer) {
    assertx(!_is_glx_dbuf);
    BITMAPINFOHEADER bmih;
    std::memset(&bmih, 0, sizeof(bmih));
    bmih.biSize = sizeof(bmih);
    bmih.biWidth = _win_dims[1];
    bmih.biHeight = _win_dims[0];
    bmih.biPlanes = 1;
    bmih.biBitCount = WORD(_scr_bpp);
    bmih.biCompression = BI_RGB;  // no compression
    _bitmap = assertx(
        CreateDIBSection(_hDC, reinterpret_cast<BITMAPINFO*>(&bmih), DIB_RGB_COLORS, &_bitmap_data, nullptr, 0));
    // Create a window-compatible memory DC and select DIBSection into it.
    _hRenderDC = assertx(CreateCompatibleDC(_hDC));  // was called hMemDC
    assertx(SelectObject(_hRenderDC, _bitmap));
  }
  if (_pbuffer || _multisample) {
    // _hRC = assertx(wglCreateContext(_hRenderDC));
    // assertx(wglMakeCurrent(_hRenderDC, _hRC));
    //
    int orig_multisample = _multisample;
    int iPixelFormat;
    for (;;) {
      const unsigned WGL_DRAW_TO_WINDOW_ARB = 0x2001;
      const unsigned WGL_ACCELERATION_ARB = 0x2003;
      const unsigned WGL_SUPPORT_OPENGL_ARB = 0x2010;
      const unsigned WGL_DOUBLE_BUFFER_ARB = 0x2011;
      const unsigned WGL_COLOR_BITS_ARB = 0x2014;
      const unsigned WGL_DEPTH_BITS_ARB = 0x2022;
      const unsigned WGL_STENCIL_BITS_ARB = 0x2023;
      const unsigned WGL_DRAW_TO_PBUFFER_ARB = 0x202D;
      const unsigned WGL_SAMPLE_BUFFERS_ARB = 0x2041;  // 0 or 1
      const unsigned WGL_SAMPLES_ARB = 0x2042;         // number of samples
      const unsigned WGL_FULL_ACCELERATION_ARB = 0x2027;
      const unsigned WGL_RED_BITS_ARB = 0x2015;
      const unsigned WGL_GREEN_BITS_ARB = 0x2017;
      const unsigned WGL_BLUE_BITS_ARB = 0x2019;
      const unsigned WGL_ALPHA_BITS_ARB = 0x201B;
      //
      Array<int> iattribl = {int(WGL_DRAW_TO_WINDOW_ARB), 1, int(WGL_ACCELERATION_ARB), int(WGL_FULL_ACCELERATION_ARB),
                             int(WGL_SUPPORT_OPENGL_ARB), 1,
                             // for unknown reason, disabling double-buffering for _offscreen prevents _multisample > 0
                             int(WGL_DOUBLE_BUFFER_ARB), (_is_glx_dbuf || _offscreen != "" ? 1 : 0),
                             int(WGL_COLOR_BITS_ARB), _scr_bpp, int(WGL_DEPTH_BITS_ARB), _scr_zbufbits,
                             int(WGL_STENCIL_BITS_ARB), _scr_stencilbits, int(WGL_RED_BITS_ARB), 8,
                             int(WGL_GREEN_BITS_ARB), 8, int(WGL_BLUE_BITS_ARB), 8, int(WGL_ALPHA_BITS_ARB), 8};
      if (_pbuffer) iattribl.push_array(V(int(WGL_DRAW_TO_PBUFFER_ARB), 1));
      if (_multisample) {
        int nsamples = _multisample == 3 ? 2 : _multisample == 5 ? 4 : _multisample;
        iattribl.push_array(V(int(WGL_SAMPLE_BUFFERS_ARB), 1, int(WGL_SAMPLES_ARB), nsamples));
      }
      iattribl.push_array(V(0, 0));
      Array<float> fattribl = {0.f, 0.f};
      Vec<int, 1000> iFormats;
      unsigned numFormats;
      assertx(wglChoosePixelFormatARB);
      assertx(wglChoosePixelFormatARB(_hRenderDC, iattribl.data(), fattribl.data(), unsigned(iFormats.num()),
                                      iFormats.data(), &numFormats));
      if (_hwdebug) showf("wglChoosePixelFormatARB found %d [0]==%d\n", numFormats, numFormats > 0 ? iFormats[0] : -1);
      if (numFormats > 0) {
        iPixelFormat = iFormats[0];
        // if (numFormats == 16) iPixelFormat = iFormats[9];
        if (_hwdebug) SHOW(iPixelFormat);
        break;
      } else if (_multisample > 4) {
        _multisample = 4;
      } else if (_multisample > 2) {
        _multisample = 2;
      } else if (_multisample >= 1) {
        _multisample = 0;
      } else if (_scr_bpp == 24) {
        showf("Lowering color_bits_per_pixel from 24 to 16\n");
        _scr_bpp = 16;
      } else if (_scr_zbufbits == 24) {
        showf("Lowering depth_bits_per_pixel from 24 to 16\n");
        _scr_zbufbits = 16;
      } else {
        showf("Is the framebuffer 32bit_color?\n");
        showf("Do display settings support 24bit_depth?\n");
        showf("Does this graphics card support 3D graphics?\n");
        assertnever("wglChoosePixelFormatARB failed");
      }
    }
    if (_multisample != orig_multisample) {
      showf("HW: had to downgrade to multisample=%d\n", _multisample);
    }
    //
    // assertx(wglMakeCurrent(_hRenderDC, 0));  // release
    // assertx(wglDeleteContext(_hRC)); _hRC = 0;
    //
    if (_pbuffer) {
      const Array<int> attr = {0};
      assertx(wglCreatePbufferARB);
      HPBUFFERARB hbuf = wglCreatePbufferARB(_hRenderDC, iPixelFormat, _win_dims[1], _win_dims[0], attr.data());
      assertx(hbuf);
      assertx(wglGetPbufferDCARB);
      _hRenderDC = assertx(wglGetPbufferDCARB(hbuf));
    } else {
      // Set the pixelFormat
      PIXELFORMATDESCRIPTOR pfd;
      std::memset(&pfd, 0, sizeof(pfd));
      pfd.nSize = sizeof(pfd);
      pfd.nVersion = 1;
      assertx(SetPixelFormat(_hRenderDC, iPixelFormat, &pfd));
      if (_hwdebug) SHOWL;
    }
  } else {
    set_pixel_format(false);
  }
  // Create all desired rendering contexts.
  //  We'll create all our OpenGL rendering contexts (GLRCs) from inside this
  //    thread (whatever it may be), so that all are associated with the same
  //    device and have the same pixel format. But later, we'll need to call
  //    wglMakeCurrent() from each thread that wants to use it.
  //  IMPT NOTE(S): -must have a GLRC for each thread that wants to use GL
  //                  calls (even if they're not graphics calls, like getting
  //                  values of a matrix!)
  //               -a GLRC can only be used by one thread (ie each thread must have its own)
  //               -BUT you can associate multiple GLRCs with the same device context
  //  ALSO: a GL rendering context basically contains all the GL machine
  //    "state".  That's why GL calls won't do anything unless you have a
  //    GL rendering context (for a particular thread).
  // Create the *main* rendering context
  _hRC = assertx(wglCreateContext(_hRenderDC));
  assertx(wglMakeCurrent(_hRenderDC, _hRC));
  if (_multisample) {
    glEnable(GL_MULTISAMPLE);
    assertx(!gl_report_errors());
    if (_hwdebug) {
      int sample_buffers;
      glGetIntegerv(GL_SAMPLE_BUFFERS, &sample_buffers);
      SHOW(sample_buffers);
      int samples;
      glGetIntegerv(GL_SAMPLES, &samples);
      SHOW(samples);
    }
  }
  // *** SETUP "OTHER" OPENGL-RELATED STUFF (FONT USAGE, ETC) ***
  if (_hwdebug) SHOW(glGetString(GL_VERSION));
  if (_hwdebug) SHOW(gl_extensions_string());
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  if (1) {
    // const int height =_bigfont ? -20 : -15;
    const int height = -MulDiv((_bigfont ? 15 : 11), GetDeviceCaps(_hDC, LOGPIXELSY), 72);
    // SHOW(GetDeviceCaps(_hDC, LOGPIXELSY), height);
    // GetDeviceCaps(_hDC, LOGPIXELSY)=96, height=-15
    LOGFONTW lf;
    std::memset(&lf, 0, sizeof(lf));
    lf.lfHeight = height;
    lf.lfWidth = 0;  // default aspect ratio
    lf.lfEscapement = 0;
    lf.lfOrientation = 0;
    lf.lfWeight = _bigfont ? FW_BOLD : FW_NORMAL;
    lf.lfItalic = 0 ? 1 : 0;
    lf.lfUnderline = 0 ? 1 : 0;
    lf.lfStrikeOut = 0 ? 1 : 0;
    lf.lfCharSet = DEFAULT_CHARSET;
    lf.lfOutPrecision = OUT_OUTLINE_PRECIS;  // or OUT_TT_ONLY_PRECIS
    lf.lfClipPrecision = 0;
    lf.lfQuality = 0;
    lf.lfPitchAndFamily = FIXED_PITCH | FF_SCRIPT;
    // const char* face_name = "Courier New";
    // assertx(strlen(face_name)<LF_FACESIZE);
    // strncpy(lf.lfFaceName, face_name, LF_FACESIZE-1)); lf.lfFaceName[LF_FACESIZE-1] = '\0';
    const wchar_t* face_name = L"Courier New";
    assertx(wcslen(face_name) < LF_FACESIZE);
    wcsncpy(lf.lfFaceName, face_name, LF_FACESIZE - 1);
    lf.lfFaceName[LF_FACESIZE - 1] = wchar_t{0};
    HFONT handle_font = assertx(CreateFontIndirectW(&lf));
    assertx(SelectObject(_hRenderDC, handle_font));
  } else {
    assertx(SelectObject(_hRenderDC, GetStockObject(ANSI_FIXED_FONT)));
  }
  {
    TEXTMETRIC tm;
    assertx(GetTextMetrics(_hRenderDC, &tm));
    // assertw(tm.tmPitchAndFamily&TMPF_FIXED_PITCH);  // somehow false
    assertw(tm.tmPitchAndFamily & TMPF_TRUETYPE);
    _font_dims = convert<int>(V(tm.tmHeight, tm.tmAveCharWidth));
    if (_hwdebug) SHOW(_font_dims);
    // -15 -> width=9 height=17
    // -20 -> width=12 height=23
  }
  // Create the bitmap display lists
  //  We are making images of glyphs 0 thru 255, and the display list numbering starts at 1000 (arbitrary choice).
  _listbase_font = 1000;  // arbitrary value
  assertw(wglUseFontBitmaps(_hRenderDC, 0, 255, _listbase_font));
  // wglUseFontBitmapsW(_hRenderDC, 0, 65535, _listbase_font);  // would generate bitmaps for all Unicode (large)
  // done later: glListBase(_listbase_font);
  if (getenv("SWAP_INTERVAL")) {
    // Can be set to zero to go faster than monitor refresh.
    // Default is usually 1.
    Warning("Setting swap_interval");
    int interval = getenv_int("SWAP_INTERVAL");
    assertw(wglSwapIntervalEXT);
    if (wglSwapIntervalEXT) assertw(wglSwapIntervalEXT(interval));
  }
  if (_multisample && contains(gl_extensions_string(), "GL_NV_multisample_filter_hint")) {
    int val = _multisample == 3 || _multisample == 5 ? GL_NICEST : GL_FASTEST;
    glHint(GL_MULTISAMPLE_FILTER_HINT_NV, val);
  }
}

// Clipboard
//  debug using ~/bin/sys/FreeClipViewer.exe

namespace {

struct bmp_BITMAPINFOHEADER {  // size 40
  uint32_t biSize;
  int biWidth;
  int biHeight;
  ushort biPlanes;
  ushort biBitCount;
  uint32_t biCompression;
  uint32_t biSizeImage;
  int biXPelsPerMeter;
  int biYPelsPerMeter;
  uint32_t biClrUsed;
  uint32_t biClrImportant;
};

}  // namespace

bool HW::copy_image_to_clipboard(const Image& image) {
  if (!assertw(image.size())) return false;
  int ncomp = image.zsize() == 4 ? 4 : 3;
  int rowsize = image.xsize() * ncomp;
  while ((rowsize & 3) != 0) rowsize++;
  int size = sizeof(bmp_BITMAPINFOHEADER) + rowsize * image.ysize();
  HANDLE hGlobal = assertx(GlobalAlloc(GHND | GMEM_SHARE, size));
  {
    uint8_t* buf = static_cast<uint8_t*>(assertx(GlobalLock(hGlobal)));
    {
      bmp_BITMAPINFOHEADER bmih;
      static_assert(sizeof(bmih) == 40, "");
      std::memset(&bmih, 0, sizeof(bmih));
      bmih.biSize = sizeof(bmih);
      bmih.biWidth = image.xsize();
      bmih.biHeight = image.ysize();
      bmih.biPlanes = 1;
      bmih.biBitCount = narrow_cast<ushort>(ncomp * 8);
      bmih.biCompression = 0;
      bmih.biSizeImage = rowsize * image.ysize();
      *reinterpret_cast<bmp_BITMAPINFOHEADER*>(buf) = bmih;
      uint8_t* p = buf + sizeof(bmih);
      for_int(y, image.ysize()) {
        int yy = image.ysize() - 1 - y;  // because bmp has image origin at lower-left
        for_int(x, image.xsize()) {
          const Pixel& pix = image[yy][x];
          // RGBA to BGRA
          *p++ = pix[2];
          *p++ = pix[1];
          *p++ = pix[0];
          if (ncomp == 4) *p++ = pix[3];
        }
        while (reinterpret_cast<uintptr_t>(p) & 3) *p++ = uint8_t{0};
      }
      assertx(int(p - buf) == size);
    }
    assertx(!GlobalUnlock(hGlobal));
  }
  bool ok = true;
  unsigned enc_format = CF_DIB;
  assertx(OpenClipboard(nullptr));
  {  // no window handle, but OK.
    assertx(EmptyClipboard());
    if (!assertw(SetClipboardData(enc_format, hGlobal))) ok = false;  // data may be too big
  }
  assertx(CloseClipboard());
  // Note: should not call GlobalFree(hGlobal) since ownership has been transferred to system.
  return ok;
}

bool HW::copy_clipboard_to_image(Image& image) {
  if (IsClipboardFormatAvailable(CF_BITMAP)) {
    Warning("ignoring CF_BITMAP for now");
  }
  if (IsClipboardFormatAvailable(CF_DIB)) {
    if (!assertw(OpenClipboard(nullptr))) return false;
    HANDLE hGlobal = assertx(GetClipboardData(CF_DIB));
    size_t size = assertx(GlobalSize(hGlobal));
    assertx(size >= sizeof(bmp_BITMAPINFOHEADER));
    {
      uint8_t* buf = static_cast<uint8_t*>(assertx(GlobalLock(hGlobal)));
      {
        bmp_BITMAPINFOHEADER& bmih = *reinterpret_cast<bmp_BITMAPINFOHEADER*>(buf);
        assertx(bmih.biSize == 40);
        image.init(V(bmih.biHeight, bmih.biWidth));
        assertx(bmih.biBitCount == 8 || bmih.biBitCount == 24 || bmih.biBitCount == 32);
        const int ncomp = bmih.biBitCount / 8;
        image.set_zsize(ncomp);
        uint8_t* p = buf + bmih.biSize;
        for_int(y, image.ysize()) {
          int yy = image.ysize() - 1 - y;  // flip vertically
          for_int(x, image.xsize()) {
            Pixel& pix = image[yy][x];
            // BGRA to RGBA
            pix[2] = *p++;
            pix[1] = *p++;
            pix[0] = *p++;
            if (ncomp == 4)
              pix[3] = *p++;
            else
              pix[3] = 255;
          }
          while (reinterpret_cast<uintptr_t>(p) & 3) p++;
        }
        if (0) SHOW(size, p - buf, image.dims(), bmih.biBitCount, image.zsize());
        // e.g. fails with size=837180 p-buf=837168 image.dims()=[389, 538] bmih.biBitCount=32 image.zsize()=4
        if (0) assertx(narrow_cast<size_t>(p - buf) == size);
        if (1) assertw(abs((p - buf) - static_cast<ptrdiff_t>(size)) < 32);
      }
      GlobalUnlock(hGlobal);  // decrement reference count; nonzero because still owned by clipboard
    }
    assertx(CloseClipboard());
    return true;
  }
  return false;
}

}  // namespace hh
