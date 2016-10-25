// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once

extern "C" {
// #define WIN32_LEAN_AND_MEAN // not possible
#define Polygon Win32_Polygon   // avoid name collision on symbol Polygon
#include <windows.h>            // required by OpenGL
#undef Polygon
#undef small                    // <windows.h> defines 'small':  "#define small char" in ./shared/rpcndr.h
#include <GL/gl.h>              // glBegin() etc. for users of this class
#include <GL/glu.h>
}

#include "HWbase.h"

namespace hh {

// Win32 implementation of abstract windowing interface.
class HW : public HWbase {
 public:
    HW();
// call anytime after init() before open()
    void set_double_buffering(bool newstate) override;
    void open() override;
// call anytime after init()
    void set_window_title(string s) override;
    void beep() override;
// call after open()
    bool suggests_stop() override;
    bool get_pointer(Vec2<int>& yx) override;
    bool get_key_modifier(EModifier modifier) override;
    void set_color_to_foreground() override;
    void set_color(const Pixel& pix) override;
    void fill_polygon(CArrayView<Vec2<float>> points) override;
    Vec2<int> get_max_window_dims() override;
    void resize_window(const Vec2<int>& yx) override;
    void make_fullscreen(bool b) override;
    void grab_focus() override;
// call within draw_window()
    void clear_window() override;
    void hard_flush() override;
// call outside draw_window()
    Array<string> query_open_filenames(const string& hint_filename) override;
    string query_save_filename(const string& hint_filename, bool force) override;
    void begin_draw_visible() override;
    void end_draw_visible() override;
    void wake_up() override;
// clipboard
    bool copy_image_to_clipboard(const Image& image) override;
    bool copy_clipboard_to_image(Image& image) override;
 private:
    MMRESULT _sk_timerID {0};
    UINT _sk_timerResolution {0}; // Max tolerable error in timer delay (mSec)
    bool _iconic {false};
    bool _maximize {false};
    int _scr_bpp {24};
    int _scr_zbufbits {24};     // was previously 16
    int _scr_stencilbits {0};
    int _multisample {0};
    bool _extra_console_visible {false};
    bool _pbuffer;              // using an OpenGL PBuffer
    HINSTANCE _hInstance;
    HWND _hwnd;
    HDC _hDC;
    HDC _hRenderDC;
    HBITMAP _bitmap;
    void* _bitmap_data;
    HGLRC _hRC;
    MSG _msg;                   // the current window-message

    bool init_aux(Array<string>& aargs) override;
    void start_hwkey() override;
    void end_hwkey() override;
    void flush_seg() override;
    void flush_point() override;
    bool loop() override;
    void draw_text_internal(const Vec2<int>& yx, const string& s) override;
    
    bool got_event();           // get event, ret: have an event ready
    void handle_events();
    void handle_event();
    void handle_key(int why_called, WPARAM key_data);
    void draw_it();
    
    void ogl_create_window(const Vec2<int>& yx);
    void set_pixel_format(bool fake_first);
    LRESULT wndProc(UINT iMsg, WPARAM wParam, LPARAM lParam);
    void set_keyintr() { _is_keyintr = true; }
    friend void CALLBACK callbackSimuKey(UINT id, UINT, DWORD_PTR userData, DWORD_PTR, DWORD_PTR);
    friend LRESULT CALLBACK wndProc_wrapper(HWND hWnd, UINT iMsg, WPARAM wParam, LPARAM lParam);
};

} // namespace hh
