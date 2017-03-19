// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHWX_HW_H_
#define MESH_PROCESSING_LIBHWX_HW_H_

#define HH_OGLX

extern "C" {
#include <X11/Xlib.h>
#include <X11/Xutil.h>          // struct XWMHints
#if defined(HH_OGLX)
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif
}

#include "../libHWin/HWbase.h"

namespace hh {

// X Windows implementation of abstract windowing interface.
class HW : public HWbase {
 public:
    HW();
// call after init() before open()
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
    bool is_fullscreen() override;
    void make_fullscreen(bool b) override;
// call within draw_window()
    void clear_window() override;
    void hard_flush() override;
// call outside draw_window()
    void begin_draw_visible() override;
    void end_draw_visible() override;
    void wake_up() override;
 private:
    bool _maximize {false};             // command-line desire
    bool _fullscreen {false};           // command-line desire
    bool _is_fullscreen {false};
    int _multisample {0};
    unsigned long _pixel_background; // XWindows pixel id
    unsigned long _pixel_foreground;
    unsigned long _pixel_border;
    bool _need_toggle_buffering {false};
    bool _is_pixbuf {false};
    bool _oglx {false};
    XWMHints* _pwmhints {nullptr};
    Display* _display {nullptr};
    Window _win;
    GC _gc;
    XGCValues _gcvalues;
    Colormap _cmap;
    XEvent _event;
    Pixmap _draw;
    Pixmap _bbuf;
    int _screen;
    int _depth;
    Atom _wmDeleteMessage;

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
    void handle_key();
    void draw_it();
    
    void toggle_buffering();
    void allocate_buf();
    void deallocate_buf();
    void get_color(string colorname, unsigned long& pixel, Pixel& color);
    void set_keyintr() { _is_keyintr = true; }
    friend void set_keyintr(HW* hwobject);
};

} // namespace hh

#endif // MESH_PROCESSING_LIBHWX_HW_H_
