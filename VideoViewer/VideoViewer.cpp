// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <atomic>
#include <thread>
#include <functional>           // function<>
#if defined(_WIN32)
#include <io.h>                 // close(), dup2()
#endif

#include "HW.h"
#include "Args.h"
#include "Image.h"
#include "Video.h"
#include "Geometry.h"
#include "MathOp.h"             // is_pow2()
#include "FileIO.h"             // my_sh()
#include "StringOp.h"           // get_path_tail()
#include "SGrid.h"
#include "Timer.h"
#include "Locks.h"              // <mutex>
#include "ArrayOp.h"            // median()
#include "Stat.h"               // HH_RSTAT()
#include "Bbox.h"
#include "Principal.h"          // principal_components()
#include "Polygon.h"            // intersect_poly_poly()
#include "GridPixelOp.h"        // spatially_scale_Grid_Pixel()
#include "Color_ramp.h"         // k_color_ramp
#include "BinarySearch.h"       // discrete_binary_search()
#include "GridOp.h"             // crop()
#include "Map.h"

#include "GradientDomainLoop.h"

#if !defined(HH_NO_VIDEO_LOOP)
// From ~/git/CompPhoto/Applications/VideoLooping/LoopAPILib/LoopAPILib/src/
#include "OptSetting.h"
#include "PipeAPI.h"
#include "mafTime.h"
#endif

#define HH_HAVE_BACKGROUND_THREAD
#if defined(__clang__)
#undef HH_HAVE_BACKGROUND_THREAD
#endif

using namespace hh;

// TODO:
// - for long videos, do not allocate memory and develop some RVideo seek() function.
// - possibly associate g_view per object?  it is useful as it is now too.
// - unsharp mask:  affine extrapolation of original and Gaussian-blurred image:
//    radius, magnitude (0% is blurred, 100% is original), threshold (skip pixels for which difference is too small)
// - clone tool with gradient-domain blending: draw destination region,
//    then interactively select position/offset of source content

namespace {

const bool k_prefer_nv12 = 1;   // read videos using NV12 format (if even dimensions) to save memory and improve speed
const bool k_use_bgra = 1;      // read images using BGRA (rather than RGBA) channel order to improve speed
const Pixel k_background_color = Pixel::black();
const Array<double> k_speeds = { // video playback speed factors
    0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,  1.,  1.1, 1.2, 1.3, 1.4, 1.5, 2., 3., 4., 5., 10.
};
const float k_key_zoom_fac = 2.f;                 // spatial zoom factor for each UI keypress
const float k_wheel_zoom_fac = sqrt(2.f);         // spatial zoom factor for each UI mouse wheel step
const Vec2<int> k_default_window_dims{576, 1024}; // was {480, 640}
const double k_before_start = -1e9; // large negative number within "int" limits, to play video from first frame
const bool k_force_refresh = true;  // parameter constant to force upload of texture to GPU
const bool k_no_text_wrap = false;  // parameter constant to disable wrapping of text in app_draw_text
const int k_usual_tex_padding_width = 8; // >1 for better memory alignment (to be safe) and better mipmap
const double k_loop_duration = 5.;       // output loop length in seconds

class DerivedHW : public HW {
 public:
    DerivedHW() { }
    bool key_press(string s) override;
    void button_press(int butnum, bool pressed, const Vec2<int>& yx) override;
    void wheel_turn(float v) override;
    void draw_window(const Vec2<int>& dims) override;
    void drag_and_drop(CArrayView<string> filenames) override;
};

DerivedHW hw;

struct Object {
    // Create a video object.
    Object(Video&& video, VideoNv12&& video_nv12, unique_ptr<RVideo> prvideo, string filename, bool unsaved = true)
        : _dims(video.size() ? video.dims() : video_nv12.get_Y().dims()), _is_image(false),
          _video(std::move(video)), _video_nv12(std::move(video_nv12)), _prvideo(std::move(prvideo)),
          _nframes_loaded(_prvideo ? 0 : _dims[0]), _filename(std::move(filename)), _orig_filename(_filename),
          _frameou1(_dims[0]), _unsaved(unsaved),
          _file_modification_time(get_path_modification_time(_filename)) {
        ok();
    }
    // Create either a video or image, depending on prev_ob.
    Object(const Object& prev_ob, Video&& video, VideoNv12&& video_nv12, string filename)
        : _dims(video.size() ? video.dims() : video_nv12.get_Y().dims()), _is_image(prev_ob._is_image),
          _video(std::move(video)), _video_nv12(std::move(video_nv12)), _prvideo(nullptr),
          _nframes_loaded(_dims[0]), _filename(std::move(filename)), _orig_filename(prev_ob._orig_filename),
          _frameou1(_dims[0]), _unsaved(true), _file_modification_time(prev_ob._file_modification_time) {
        assertx(!prev_ob._prvideo); assertx(prev_ob._nframes_loaded==prev_ob._dims[0]);
        _image_is_bgra = prev_ob._image_is_bgra;
        if (_dims[0]==prev_ob._dims[0]) {
            _framenum = prev_ob._framenum;
            _framein = prev_ob._framein;
            _frameou1 = prev_ob._frameou1;
        }
        _video.attrib() = prev_ob._video.attrib();
        _image_attrib = prev_ob._image_attrib;
        ok();
    }
    // Create an image object.
    Object(Image&& image, string filename, bool bgra, bool unsaved = true)
        : _dims(concat(V(1), image.dims())), _is_image(true),
          _video(),
          _nframes_loaded(1), _filename(std::move(filename)), _orig_filename(_filename),
          _frameou1(_dims[0]), _unsaved(unsaved),
          _file_modification_time(get_path_modification_time(_filename)) {
        _image_is_bgra = bgra;
        _image_attrib = image.attrib();
        _video = increase_grid_rank(std::move(image));
        // _video[0].assign(image);
        ok();
    }
    size_t size() const                         { return narrow_cast<size_t>(product(_dims)); }
    int nframes() const                         { return _dims[0]; }
    bool is_image() const                       { return _is_image; }
    const Vec2<int>& spatial_dims() const       { return _dims.tail<2>(); }
    string stype() const                        { return is_image() ? "image" : "video"; }
    Vec3<int> _dims;            // _dims[0] is the number of frames, which is always 1 for an image
    bool _is_image;
    Video _video;               // if is_image(), contains a single frame which is the image
    VideoNv12 _video_nv12;
    Image::Attrib _image_attrib; // if is_image()
    unique_ptr<RVideo> _prvideo;
    std::atomic<int> _nframes_loaded;
    string _filename;           // always absolute path, using "/" as directory separator
    string _orig_filename;
    bool _image_is_bgra {false};
    std::atomic<bool> _locked_by_background_thread {false};
    int _framenum {int(k_before_start)}; // only used when switching objects
    int _framein {0};                    // IN trim frame
    int _frameou1;                       // OUT trim frame plus 1
    bool _unsaved;
    uint64_t _file_modification_time;
    void ok() const {
        if (!file_requires_pipe(_filename) && !is_path_absolute(_filename)) assertnever("? " + _filename);
        assertx(!!_video.size()^!!_video_nv12.size());
        if (_is_image) assertx(_dims[0]==1 && !_video_nv12.size());
    }
};

Array<unique_ptr<Object>>& g_obs = *new Array<unique_ptr<Object>>; // never deleted so background thread is safe
std::mutex g_mutex_obs;
std::atomic<int> g_cob{-1};     // currently selected object (0..g_obs.num()-1), or -1 if g_obs.num()==0
Object& getob(int cob) { assertx(g_obs.ok(cob)); return *g_obs[cob]; } // (bounds-check even in Release)
Object& getob() { return getob(g_cob); }
int getobnum() { return g_obs.num(); }

std::atomic<bool> g_request_loop {false};  // request to generate seamless loop
bool g_request_loop_synchronously {false}; // if set, wait until seamless loop is ready
bool g_high_quality_loop {false};          // solve for all period+start labels rather than dominant ones
bool g_working_on_loop_creation {false};   // set by background thread
unique_ptr<Object> g_videoloop_ready_obj;  // created by background thread
unique_ptr<Object> g_vlp_ready_obj;        // created by background thread
double g_initial_time = 0.;                // requested initial time in video (in seconds)
double g_frametime = k_before_start; // continous time in units of frame; <0. means show first frame next
std::atomic<int> g_framenum{-1};     // clamp(int(floor(g_frametime)), 0, getob()._nframes_loaded-1) or -1
Vec2<int> g_frame_dims;              // spatial dimensions in pixels of current video or image object
bool g_frame_has_transparency;       // true if png image with some partially transparent pixel(s)
bool g_refresh_texture = false;      // image has changed since uploaded as texture
struct Message { string s; double time; };
Array<Message> g_messages;
std::mutex g_mutex_messages;

string g_argv0;                 // pathname used to invoke this program
string g_keystring;             // accumulates command-line simulated keystrokes
int g_verbose = 0;              // 0==normal, 1==some_output, 2==lots_of_output
bool g_playing = true;          // video is playing (else paused)
enum class ELooping { off, one, all, mirror };
const ELooping k_default_looping = ELooping::one; // default is to loop the current video object infinitely
ELooping g_looping = k_default_looping;
bool g_mirror_state_forward = true; // true if currently advancing forward in mirror loop
enum class ESort { name, date };
const ESort k_default_sort = ESort::name;
ESort g_sort = k_default_sort;
double g_speed = 1.;            // play speed factor relative to real time; usually k_speeds.index(g_speed)>=0
Pixel g_through_color;          // color visible through partially transparent pixels
bool g_checker;                 // use moving black-white checker as alternative to g_through_color
enum class EFit { isotropic, anisotropic };
const EFit k_default_fit = EFit::isotropic; // do not distort aspect ratio to fit image/video to window
EFit g_fit = k_default_fit;
bool g_fit_view_to_window = true; // always adjust g_view to fit window (using g_fit), else g_view is unconstrained
enum class EKernel { linear, keys, lanczos6, lanczos10, nearest, last };
Array<string> k_kernel_string { "linear", "keys", "lanczos6", "lanczos10", "nearest" };
const EKernel k_default_kernel = EKernel::keys; // 20150521: was EKernel::linear; could be EKernel::lanczos6
EKernel g_kernel = k_default_kernel;            // desired image reconstruction (sampling) filter
EKernel g_render_kernel;                        // filter in use
const bool k_default_info = true;
bool g_show_info = k_default_info; // show information in window (including timeline)
bool g_other_show_info = false;    // show information in non-curent window config (e.g. fullscreen if not)
bool g_show_exif = false;          // show additional information, included content metadata
bool g_show_help = false;          // show overlaid help text
Vec2<int> g_win_dims;              // window dimensions (y, x)
Vec2<int> g_desired_dims {-1, -1}; // if waiting for window resize
Vec2<int> g_tex_dims;              // allocated texture buffer dimensions (y, x)
Vec2<int> g_tex_active_dims;       // dimension of region of texture that is currently used for media content
Vec2<int> g_prev_win_pos;          // to detect window boundary motions
Vec2<int> g_prev_win_dims;         // to detect window boundary motions
Pixel g_text_shadow_color = Pixel::black();
int g_background_padding_width = 0; // padding surrounding all 4 sides of texture if no edge_clamp support
bool g_generated_mipmap = false;    // whether the mipmap has been generated for the current texture
Frame g_view = Frame::identity();   // map from image/video pixel coordinates to window pixel coordinates
bool g_prompted_for_delete = false; // have warned user about deleting a file
string g_dest_dir;                  // for object move or copy

bool g_use_sliders = false;
float g_brightness = 1.f;
float g_contrast = 1.f;
float g_gamma = 1.f;
float g_saturation = 1.f;

struct Slider { string name; float* pval; };
const Vec<Slider,4> g_sliders = {
    Slider{"brightness",        &g_brightness},
    Slider{"contrast",          &g_contrast},
    Slider{"gamma",             &g_gamma},
    Slider{"saturation",        &g_saturation},
};

struct S_Selected {             // during drag operation
    int button_active {0};      // 1..3 if mouse button currently pressed, else 0
    bool shift_was_pressed;
    bool control_was_pressed;
    Vec2<int> yx_pressed;       // location where button was initially pressed
    Vec2<int> yx_last;          // previous location during a drag operation
    int axis_constraint;        // 0 or 1, or -1 if none
    bool on_timeline;           // button was pressed on timeline
} g_selected;

struct S_LoopingParameters {
    Matrix<int> mat_static;     // static frame
    Matrix<int> mat_start;      // start frame
    Matrix<int> mat_period;     // period (1 or a multiple of K==4)
    Matrix<float> mat_activation;
    std::atomic<bool> is_loaded {false};
} g_lp;                         // looping data structures

struct S_Timeline {
    const float hpad = .03f;            // horizontal padding on left and right sides
    const float top = .96f, bot = .97f; // top and bottom coordinates of timeline
    const float left = hpad, right = 1.f-hpad, width = 1.f-2*hpad;
    const float fudge = .03f;             // extra leeway distance to detect clicks on timeline
    const Pixel cur_color {255, 200, 50}; // color for current frame handle (light orange)
    const float min_w = .008f;       // minimum width of handle for current frame (in case video is very long)
    const float trim_extrah = .003f; // extra height of trim line above and below regular timeline
    const Pixel trim_color = Pixel::blue();                                          // color of trim line
    float get_wcur(int nframes) const { return max(width/assertx(nframes), min_w); } // width of current frame handle
    bool is_on_timeline(const Vec2<float>& yx) const {
        return yx[0]>=top-fudge && yx[0]<=bot+fudge && yx[1]>=left-fudge && yx[1]<=right+fudge;
    }
} g_timeline;

struct PrefetchImage {
    string filename;
    uint64_t file_modification_time {0}; // 0==load_never_attempted
    unique_ptr<Image> pimage;            // nullptr could indicate a load error; bgra format iff k_use_bgra
};
SArray<PrefetchImage,2> g_prefetch_image; // {0==next, 1==prev}
std::mutex g_mutex_prefetch;

bool filename_is_media(const string& s) {
    return filename_is_image(s) || filename_is_video(s);
}

class DirMediaFilenames {
 public:
    DirMediaFilenames() { }
    CArrayView<string> get(const string& filename) {
        assertx(!file_requires_pipe(filename));
        string directory = get_path_head(filename); assertx(directory!="");
        string filename_tail = get_path_tail(filename);
        bool is_new; S& s = _map.enter(directory, S{}, is_new);
        double cur_time = get_precise_time();
        const double k_max_time_before_refresh = 5.; // in seconds
        bool file_found = s.filenames.index(filename_tail)>=0;
        if (cur_time>s.time_updated+k_max_time_before_refresh || !file_found) {
            s.time_updated = cur_time;
            s.filenames = sort_dir(directory, get_files_in_directory(directory));
            auto func_not_media = [](const string& s2){ return !filename_is_media(s2); };
            s.filenames.erase(std::remove_if(s.filenames.begin(), s.filenames.end(), func_not_media),
                              s.filenames.end());
        }
        return s.filenames;
    }
    void invalidate() {
        // for (S& s : s_map.values()) { s.time_updated = 0.; }
        _map.clear();
    }
 private:
    struct S {
        double time_updated {0.};
        Array<string> filenames;
    };
    Map<string, S> _map;        // directory -> filenames_in_that_directory
    static Array<string> sort_dir(const string& directory, Array<string> filenames) {
        switch (g_sort) {
         bcase ESort::name:
            sort(filenames);
         bcase ESort::date:
            sort(filenames, [&](const string& s1, const string& s2) {
                return (get_path_modification_time(directory + '/' + s1) <
                        get_path_modification_time(directory + '/' + s2));
            });
         bdefault: assertnever("");
        }
        return filenames;
    }
} g_dir_media_filenames;

CArrayView<string> get_directory_media_filenames(const string& filename) {
    return g_dir_media_filenames.get(filename);
}

// Forward declaration
void background_work(bool asynchronous);

bool is_fullscreen() {
    return hw.is_fullscreen();
}

bool view_has_rotation() {
    return g_view[0][1] || g_view[1][0];
}

float get_brightness_term() {
    return g_brightness-1.f;
}

float get_contrast_fac() {
    return g_contrast;
}

float get_saturation_fac() {
    return g_saturation;
}

// Given window pixel coordinates (pixel centers on half integers), return coordinates of rendered image texel,
//  where win_yx==twice(0.f) is upper-left corner of window.
Vec2<int> get_image_yx(const Vec2<float>& win_yx) {
    return convert<int>(floor((Point(concat(win_yx, V(0.f))) * inverse(g_view)).head<2>()));
}

// Given image texel coordinates (texel centers on half integers), return window pixel coordinates of the texel center,
//  where image_yx==twice(0.f) is upper-left corner of texture image.
Vec2<int> get_win_yx(const Vec2<float>& image_yx) {
    return convert<int>(floor((Point(concat(image_yx, V(0.f))) * g_view).head<2>()));
}

template<typename T> Vec2<Vec2<T>> bbox_minmax(CArrayView<Vec2<T>> ar) {
    return V(transitive_min(ar), transitive_max(ar));
}

template<typename T> Vec4<Vec2<T>> minmax_corners(const Vec2<Vec2<T>>& minmax) {
    const Vec2<T>& vmin = minmax[0];
    const Vec2<T>& vmax = minmax[1];
    return V(V(vmin[0], vmin[1]), V(vmax[0], vmin[1]), V(vmax[0], vmax[1]), V(vmin[0], vmax[1]));
}

bool is_ccw(const Vec3<Vec2<float>>& pts) {
    return (pts[1][0]-pts[0][0])*(pts[2][1]-pts[0][1]) > (pts[2][0]-pts[0][0])*(pts[1][1]-pts[0][1]);
}

bool is_convex_ccw(CArrayView<Vec2<float>> poly) {
    for_int(i, poly.num()) {
        int i1 = (i+1)%poly.num();
        int i2 = (i+2)%poly.num();
        if (!is_ccw(V(poly[i], poly[i1], poly[i2]))) return false;
    }
    return true;
}

bool intersect_poly_poly_2D(CArrayView<Vec2<float>> poly1, CArrayView<Vec2<float>> poly2) {
    ASSERTX(is_convex_ccw(poly1) && is_convex_ccw(poly2)); // assume polygons are convex and oriented ccw
    // Look for any separating line using any edge of either poly1 or poly2.
    for_int(i, poly1.num()) {
        int i1 = (i+1)%poly1.num();
        int ninside = 0; for_int(j, poly2.num()) { ninside += is_ccw(V(poly1[i], poly1[i1], poly2[j])); }
        if (!ninside) return false;
    }
    for_int(i, poly2.num()) {
        int i1 = (i+1)%poly2.num();
        int ninside = 0; for_int(j, poly1.num()) { ninside += is_ccw(V(poly2[i], poly2[i1], poly1[j])); }
        if (!ninside) return false;
    }
    return true;
}

bool image_is_not_visible() {
    assertx(g_cob>=0);
    return !intersect_poly_poly_2D(minmax_corners(V(twice(0.f), convert<float>(g_win_dims))),
                                   map(minmax_corners(V(twice(0.f), convert<float>(g_frame_dims))),
                                       [](const Vec2<float>& p) { return convert<float>(get_win_yx(p)); }));
}

// Add a modifier (e.g. "_crop") to the end of the root part of a filename.
string append_to_filename(const string& filename, const string& smodif) {
    if (file_requires_pipe(filename)) return filename;
    if (filename=="") return {};
    string sroot = get_path_root(filename), sext = get_path_extension(filename);
    if (0 && ends_with(sroot, smodif)) return filename; // possibly only add if not already present
    return sroot + smodif + (sext!="" ? ("." + sext) : "");
}

bool is_unlocked(const Object& ob) {
    while (ob._locked_by_background_thread) {
        if (g_request_loop) return false; // Waiting for background thread to finish could take several seconds
        my_sleep(0.);                     // Wait for background thread to finish reading a frame
    }
    return true;
}

void verify_saved(const Object& ob) {
    string s = ob.stype() + " '" + ob._filename + "'";
    if (ob._unsaved) throw s + " is unsaved";
    if (file_requires_pipe(ob._filename)) throw s + " is loaded from a pipe rather than a file";
    if (!file_exists(ob._filename)) throw "file '" + ob._filename + "' is no longer found";
}

// Must be called with lock on g_mutex_obs.  Is there any way to assert this (e.g. in Debug build)?
Object& check_object() {
    if (g_cob<0) throw string("no image or video is opened");
    Object& ob = getob();
    if (!is_unlocked(ob)) throw ob.stype() + " is locked due to background processing";
    return ob;
}

Object& check_loaded_object() {
    Object& ob = check_object();
    if (ob._nframes_loaded!=ob.nframes()) throw string("video is not finished loading");
    return ob;
}

Object& check_loaded_image() {
    Object& ob = check_loaded_object();
    if (!ob.is_image()) throw string("operation expects an image (not a video)");
    return ob;
}

Object& check_loaded_video() {
    Object& ob = check_loaded_object();
    if (ob.is_image()) throw string("operation expects a video (not an image)");
    return ob;
}

Object& check_loaded_saved_object() {
    Object& ob = check_loaded_object();
    verify_saved(ob);
    return ob;
}

Object& check_saved_object() {
    Object& ob = check_object();
    verify_saved(ob);
    return ob;
}

void check_all_objects() {
    for (auto& pob : g_obs) {
        if (!is_unlocked(*pob)) throw pob->stype() + " " + pob->_filename + "is locked due to background processing";
    }
}

// No lock on g_mutex_obs.
Object& verify_video() {
    if (g_cob<0) throw string("no video is opened");
    Object& ob = getob();
    if (ob.is_image()) throw string("operation expects a video (not an image)");
    return ob;
}

unique_ptr<Object> object_reading_video(string filename) {
    try {
        if (!assertw(file_requires_pipe(filename) || filename_is_video(filename))) SHOW("not video?", filename);
        bool use_nv12 = k_prefer_nv12 && !ends_with(filename, ".avi");
        unique_ptr<RVideo> prvideo = make_unique<RVideo>(filename, use_nv12); // may throw
        Vec3<int> dims = prvideo->dims(); // should allocate extra padframes?
        if (use_nv12 && !is_zero(dims.tail<2>()%2)) {
            use_nv12 = false;
            prvideo.reset();
            prvideo = make_unique<RVideo>(filename, use_nv12); // may throw
        }
        Video video(!use_nv12 ? dims : thrice(0));
        VideoNv12 video_nv12(use_nv12 ? dims : thrice(0));
        video.attrib() = prvideo->attrib();
        const bool unsaved = false;
        return make_unique<Object>(std::move(video), std::move(video_nv12), std::move(prvideo),
                                   std::move(filename), unsaved);
    }
    catch (std::bad_alloc&) {
        throw std::runtime_error("insufficient memory to load video");
    }
}

void read_image(Image& image, const string& filename) {
    if (k_use_bgra) {
        image.read_file_bgra(filename); // may throw
    } else {
        image.read_file(filename); // may throw
    }
}

unique_ptr<Object> object_reading_image(string filename) {
    if (!assertw(file_requires_pipe(filename) || filename_is_image(filename))) SHOW("not image?", filename);
    Image image;
    HH_CTIMER(_read_image, g_verbose>=1);
    // about 0.20sec for 5472x3648 using WIC; 0.40sec using Image_IO libjpeg; 0.08sec using Pixel::gray
    bool bgra = false, unsaved = false;
    if (0) {                    // test the response time without any loading delay
        static uchar uc = 40; uc = narrow_cast<uchar>(40 + my_mod(uc+40, 180)); // not thread-safe
        HH_TIMER(_read_init);
        image.init(V(3648, 5472), Pixel::gray(uc));
    } else if (PrefetchImage* pp = find_if(g_prefetch_image, [&](const PrefetchImage& p) {
        return p.filename==filename && p.file_modification_time==get_path_modification_time(filename) && p.pimage;
    })) {
        std::lock_guard<std::mutex> lg(g_mutex_prefetch);
        PrefetchImage& p = *pp;
        assertx(p.filename==filename);
        if (g_verbose>=1) SHOW("accessing prefetch", pp-g_prefetch_image.data(), filename);
        p.filename = "";
        p.file_modification_time = 0;
        image = std::move(*p.pimage.get());
        p.pimage.reset();
        bgra = k_use_bgra;
    } else {
        read_image(image, filename); // may throw
        bgra = k_use_bgra;
    }
    return make_unique<Object>(std::move(image), std::move(filename), bgra, unsaved);
}

unique_ptr<Object> object_reading_file(string filename) { // may throw
    if (filename_is_image(filename)) {
        return object_reading_image(filename);
    } else {
        return object_reading_video(filename);
    }
}

string next_image_in_directory(const string& filename, int increment) {
    assertx(!file_requires_pipe(filename));
    assertx(abs(increment)==1);
    if (!file_exists(filename)) return "";
    string directory = get_path_head(filename);
    string filename_tail = get_path_tail(filename);
    CArrayView<string> filenames = get_directory_media_filenames(filename);
    int i0 = filenames.index(filename_tail);
    if (!assertw(i0>=0)) return "";
    for (int i = i0; ; ) {
        i = my_mod(i+increment, filenames.num());
        if (i==i0) return "";
        if (filename_is_image(filenames[i])) return directory + "/" + filenames[i];
    }
}

void app_draw_text(const Vec2<int>& yx, const string& s, bool wrap = true) {
    if (s=="") return;
    if (1) {
        const uchar alpha_transparency = 150;
        hw.draw_text(yx, s, HW::EStyle::boxed, g_text_shadow_color.with(3, alpha_transparency), wrap);
    } else {
        hw.draw_text(yx, s, HW::EStyle::shadowed, g_text_shadow_color, wrap);
    }
}

// Show a message on the window for a given duration.
void message(const string& s, double duration = 1.5) {
    std::lock_guard<std::mutex> lg(g_mutex_messages);
    if (1) g_messages.clear();  // immediately erase any old messages
    g_messages.push(Message{s, get_precise_time()+duration});
    hw.redraw_later();
    hw.wake_up();               // in case this is called from a background thread
}

// Show a message immediately in the foreground (visible) window buffer.
void immediate_message(const string& s) {
    const int font_height = hw.get_font_dims()[0];
    hw.begin_draw_visible();
    app_draw_text(V(2*(font_height+4), 6), s);
    hw.end_draw_visible();
    hw.hard_flush();
    hw.redraw_later();
}

Vec2<int> determine_default_window_dims(Vec2<int> frame_dims) {
    if (!product(frame_dims)) frame_dims = k_default_window_dims;
    auto max_dims = hw.get_max_window_dims();
    if (1) max_dims -= 2;       // allow grabbing of bottom and right boundaries to crop
    if (0) SHOW(frame_dims, max_dims);
    if (max(frame_dims, max_dims)==max_dims) return frame_dims;
    Vec2<int> ndims = max_dims;
    Vec2<float> arzoom = convert<float>(max_dims) / convert<float>(frame_dims);
    int cmax = arzoom[0]>arzoom[1] ? 0 : 1;
    ndims[cmax] = int(frame_dims[cmax]*arzoom[1-cmax]+.5f);
    if (0) SHOW(max_dims, arzoom, cmax, ndims);
    return ndims;
}

// Identify rectangle [yxL, yxU) of image texels that are fully visible.
void fully_visible_image_rectangle(Vec2<int>& yxL, Vec2<int>& yxU) {
    assertx(!view_has_rotation());
    const float eps = .02f;     // sufficient fudge even at high magnification (was .01f)
    yxL = max(get_image_yx(twice(-eps))+1, twice(0));
    yxU = min(get_image_yx(convert<float>(g_win_dims)+eps), getob().spatial_dims());
}

bool image_is_fully_visible() {
    assertx(!view_has_rotation());
    Vec2<int> yxL, yxU; fully_visible_image_rectangle(yxL, yxU);
    return is_zero(yxL) && yxU==getob().spatial_dims();
}

void app_set_window_title() {
    string s = "VideoViewer";
    if (g_cob>=0) {
        string filename = getob()._filename;
        string sname = file_requires_pipe(filename) ? filename : get_path_tail(filename);
        s = sname + (getob()._unsaved ? " (unsaved)" : "") +  " - " + s;
        if (!g_fit_view_to_window && !view_has_rotation() && !image_is_fully_visible()) {
            Vec2<int> yxL, yxU; fully_visible_image_rectangle(yxL, yxU);
            s += sform(" (%d, %d)-(%d, %d)  (W=%d, H=%d)",
                       yxL[1], yxL[0], yxU[1]-1, yxU[0]-1, yxU[1]-yxL[1], yxU[0]-yxL[0]);
        }
    }
    hw.set_window_title(s);
}

void set_view(const Frame& view) {
    g_view = view;
}

// Select the frame in object indexed cob at continuous time frametime.
void set_video_frame(int cob, double frametime, bool force_refresh = false) {
    assertx(getob(cob).size());
    g_frametime = frametime;
    int nframenum = getob(cob)._nframes_loaded ? clamp(int(floor(g_frametime)), 0, getob(cob)._nframes_loaded-1) : -1;
    if (cob==g_cob && nframenum==g_framenum && !force_refresh) return;
    if (cob!=g_cob) {
        g_cob = cob;
    }
    Object& o = getob();
    g_framenum = nframenum;
    o._framenum = g_framenum;
    g_frame_dims = o.spatial_dims();
    g_frame_has_transparency = 1 && o.is_image() && o._image_attrib.zsize==4 &&
        find_if(o._video[0], [](const Pixel& pix) { return pix[3]!=255; });
    if (0 && o.is_image()) g_playing = false;
    g_refresh_texture = true;
    if (1 && o.is_image() && !file_requires_pipe(o._filename)) {
        // Try prefetch of adjacent images in the same directory.
        for (int increment : {1, -1}) {
            string filename = next_image_in_directory(o._filename, increment);
            if (filename=="") continue;
            int i = (1-increment)/2; // 0==next, 1==prev
            std::lock_guard<std::mutex> lg(g_mutex_prefetch);
            if (none_of(g_prefetch_image, [&](const PrefetchImage& p){ return p.filename==filename; })) {
                if (g_verbose>=1) SHOW("requesting_prefetch", i, filename);
                g_prefetch_image[i].filename = filename;
                g_prefetch_image[i].file_modification_time = 0; // zero means request prefetch
                g_prefetch_image[i].pimage.reset();
            }
        }
    }
    hw.redraw_later();
}

void set_video_frame(int cob, int frame, bool force_refresh = false) {
    set_video_frame(cob, double(frame), force_refresh);
}

// Insert new object right after current object (or as first object) and select it.
void add_object(unique_ptr<Object> pob) {
    g_obs.insert(g_cob+1, 1);
    g_obs[g_cob+1] = std::move(pob);
    set_video_frame(g_cob+1, g_framenum);
}

Pixel get_frame_pix(const Vec2<int>& yx) {
    assertx(g_cob>=0 && g_framenum>=0);
    if (getob()._video_nv12.size()) {
        CNv12View nv12v = getob()._video_nv12[g_framenum];
        uchar y = nv12v.get_Y()[yx];
        const Vec2<uchar>& uv = nv12v.get_UV()[yx/2];
        return YUV_to_RGB_Pixel(y, uv[0], uv[1]);
    } else {
        Pixel pix = getob()._video[g_framenum][yx];
        const bool bgra = getob().is_image() && getob()._image_is_bgra;
        if (bgra) std::swap(pix[0], pix[2]);
        return pix;
    }
}

// When making a window larger after having made it smaller, round the dimensions back towards the original.
Vec2<int> round_dims(Vec2<int> dims, Vec2<int> orig_dims) {
    for_int(c, 2) {
        for (;;) {
            if (dims[c]==orig_dims[c]-1) { assertx(dims[c]%2==0); dims[c]++; break; }
            if (dims[c]>=orig_dims[c]) break;
            assertx(orig_dims[c]);
            orig_dims[c] /= 2;
        }
    }
    return dims;
}

// Use SVD to extract the min/max zoom values from Matrix g_view.
Vec2<float> get_zooms() {
    SGrid<float, 2, 2> mo;
    Vec2<float> eimag;
    principal_components(SGrid<float, 2, 2>{{g_view[0][0], g_view[0][1]}, {g_view[1][0], g_view[1][1]}}.view(),
                         mo, eimag);
    eimag *= sqrt(float(eimag.num()));
    return eimag;
}

string get_szoom() {
    Vec2<float> arzoom = get_zooms();
    if (g_fit==EFit::isotropic) arzoom = twice(min(arzoom));
    const float vzoom = sqrt(float(product(arzoom))); // geometric mean
    const bool isotropic = arzoom[0]==arzoom[1];      // was abs(arzoom[1]/arzoom[0]-1.f)<.01f;
    return sform("%s%d%%", (isotropic ? "" : "~"), int(vzoom*100.f+.5f));
}

// Select filter for spatial resampling operations.
const Filter& get_resampling_filter() {
    return (g_kernel==EKernel::nearest ?   Filter::get("impulse") :
            g_kernel==EKernel::linear ?    Filter::get("triangle") :
            g_kernel==EKernel::keys ?      Filter::get("spline") : // higher quality than "keys"
            g_kernel==EKernel::lanczos6  ? Filter::get("lanczos6") :
            g_kernel==EKernel::lanczos10 ? Filter::get("lanczos10") :
            (assertnever_ret(""), Filter::get("impulse")));
}


// Change zooom while holding window dimensions constant and keeping same content at cursor.
void perform_zoom_at_cursor(float fac_zoom, Vec2<int> yx) {
    if (fac_zoom==1.f) return;
    g_fit_view_to_window = false;
    Vec3<float> pcenter = concat(convert<float>(yx)+.5f, V(0.f));
    set_view(g_view * Frame::translation(-pcenter) * Frame::scaling(thrice(fac_zoom)) * Frame::translation(pcenter));
}

// Resize the window and reset the view.
void reset_window(const Vec2<int>& ndims) {
    hw.resize_window(ndims);
    g_desired_dims = ndims;
    g_fit_view_to_window = true;
}

// Change zooom of window; if fullscreen, zoom about window center, else resize window dimensions.
void perform_window_zoom(float fac_zoom) {
    g_win_dims = hw.get_window_dims();           // for -key "=="
    if (is_fullscreen() || !g_fit_view_to_window) { // zoom the view relative to window center
        g_fit_view_to_window = false;
        Vec3<float> pc = concat(convert<float>(g_win_dims), V(0.f))/2.f; // no .5f adjustment
        set_view(g_view * Frame::translation(-pc) * Frame::scaling(thrice(fac_zoom)) * Frame::translation(pc));
        if (!is_fullscreen() && product(g_frame_dims) && !view_has_rotation()) {
            // tighten window if entire image can be made visible
            assertx(var(get_zooms())<1e-10f); // zoom must be isotropic if !g_fit_view_to_window
            const float zoom = get_zooms()[0];
            Vec2<int> ndims = convert<int>(convert<float>(g_frame_dims)*twice(zoom)+.5f);
            if (0) SHOW(zoom, ndims, g_win_dims);
            if (max(ndims, g_win_dims)==g_win_dims) {
                if (0) SHOW(ndims);
                reset_window(ndims);
                set_view(Frame::scaling(thrice(zoom))); // remove translation
            }
        }
        message("Zoom set to: " + get_szoom());
    } else {                    // zoom by resizing the window
        Vec2<int> cur_dims = g_win_dims;
        if (g_desired_dims[0]>=0) cur_dims = g_desired_dims;
        Vec2<int> ndims = convert<int>(convert<float>(cur_dims)*fac_zoom);
        {
            Vec3<float> pc0 = concat(convert<float>(cur_dims), V(0.f))/2.f;
            Vec3<float> pc1 = concat(convert<float>(ndims), V(0.f))/2.f;
            set_view(g_view * Frame::translation(-pc0) * Frame::scaling(thrice(fac_zoom)) * Frame::translation(pc1));
        }
        if (max(ndims-hw.get_max_window_dims())>0) {  // window would be too large for screen
            ndims = hw.get_max_window_dims();
            g_fit_view_to_window = false;
            message("Zooming window");
            hw.resize_window(ndims);
            g_desired_dims = ndims;
        } else if (product(g_frame_dims)) {
            ndims = round_dims(ndims, g_frame_dims);
            if (g_fit==EFit::isotropic) { // tighten the window dimensions for isotropic fit
                Vec2<float> arzoom = convert<float>(ndims) / convert<float>(g_frame_dims);
                int cmax = arzoom[0]>arzoom[1] ? 0 : 1;
                ndims[cmax] = int(g_frame_dims[cmax]*arzoom[1-cmax]+.5f);
            }
            if (product(ndims)) {
                message("Zooming window");
                reset_window(ndims);
            }
        }
    }
}

// Rotate view.
void perform_window_rotation(float vrotate) {
    if (vrotate==0.f) return;
    g_fit_view_to_window = false;
    Vec3<float> pcenter = concat(convert<float>(g_win_dims), V(0.f))/2.f;
    set_view(g_view * Frame::translation(-pcenter) * Frame::rotation(2, vrotate) * Frame::translation(pcenter));
}

// Change 3-way looping state and display message.
void set_looping(ELooping looping) {
    g_looping = looping;
    message(string() + "Looping set to: " + (g_looping==ELooping::off ? "off" :
                                             g_looping==ELooping::one ? "one" :
                                             g_looping==ELooping::all ? "all" : "mirror"));
}

// Set speed and display message.
void set_speed(double speed) {
    g_speed = speed;
    message(sform("Speed set to: %gx%s", g_speed, (g_speed!=1. ? "    (press <1> to revert to default)" : "")));
}

void set_fullscreen(bool v) {
    if (v!=is_fullscreen()) std::swap(g_show_info, g_other_show_info);
    hw.make_fullscreen(v);
}

// Open current object in default external program.
void view_externally() {
    const string& filename = getob()._filename;
    if (!file_exists(filename)) throw "file '" + filename + "' does not exist";
    if (file_requires_pipe(filename)) throw "file '" + filename + "' is a pipe";
    message("Externally opening " +  filename, 5.);
    // This works best in Windows, even with Unicode filenames.
    if (!my_spawn(V<string>("cmd", "/s/c", "start \"dummy_window_title\" \"" + filename + "\""), true)) {
        if (g_verbose) SHOW("spawned using cmd");
        return;                 // success
    }
    const Array<string> programs = {
        "cygstart",             // Cygwin
        "xdg-open",             // Linux
        "open",                 // Mac
    };
    const Array<string> image_programs = {
        "irfan",
        "c:/Program Files/IrfanView/i_view64.exe", // difficulty: only works with backslash-delimited arg pathnames
        "c:/Program Files (x86)/IrfanView/i_view32.exe", // same problem
        "eog",                                           // Gnome image viewer
        "display",                                       // ImageMagick
    };
    const Array<string> video_programs = {
        "vlc",
        "c:/Program Files/VideoLAN/VLC/vlc.exe",
        "c:/Program Files (x86)/VideoLAN/VLC/vlc.exe",
        "totem",                        // default program for "Videos" on Gnome
    };
    const bool is_image = getob().is_image();
    for (const string& program : concat(programs, is_image ? image_programs : video_programs)) {
        string tfilename = filename;
        if (contains(program, "irfan") || contains(program, "i_view"))
            tfilename = replace_all(tfilename, "/", "\\");
        // Unfortunately, quoting misbehaves and irfanview sees: ""\\"hh"\\"data"\\"image"\\"lake.png"
        if (0) SHOW(program, tfilename);
        if (!my_spawn(V(program, tfilename), true)) {
            if (g_verbose) SHOW("spawned using", program);
            return;             // success
        }
    }
    throw "unable to externally open " + filename;
}

// Rotate counter-clockwise by an angle of -270, -180, -90, 0, +90, +180, or +270 degrees.
template<typename T> Grid<3,T> rotate_ccw(CGridView<3,T> grid, int rot_degrees) {
    assertx(my_mod(rot_degrees, 90)==0);
    const Vec2<int> sdims = grid.dims().template tail<2>();
    const Vec2<int> nsdims = my_mod(rot_degrees, 180)==0 ? sdims : sdims.rev();
    Grid<3,T> ngrid(concat(V(grid.dim(0)), nsdims));
    parallel_for_int(i, grid.dim(0)) { rotate_ccw(grid[i], rot_degrees, ngrid[i]); }
    return ngrid;
}

// Replace current object with previous/next file in object's directory.  Ret: success.
bool replace_with_other_object_in_directory(int increment) {
    assertx(abs(increment)==1 || abs(increment)==INT_MAX);
    const Object& ob0 = check_saved_object();
    string filename0 = ob0._filename;
    string directory = get_path_head(filename0);
    string filename0_tail = get_path_tail(filename0);
    CArrayView<string> filenames = get_directory_media_filenames(filename0);
    if (!filenames.num()) { message("No media files in directory"); return false; }
    int i0 = filenames.index(filename0_tail); // i0 is -1 if g_cob<0 or if ob0 is somehow an unsaved object.
    if (g_verbose>=1) SHOW(directory, filename0_tail, i0);
    bool skip_first_advance = false;
    if (i0<0) { skip_first_advance = true; i0 = 0; } // cannot find file, so select first file in directory
    if (increment==-INT_MAX) { increment = +1; skip_first_advance = true; i0 = 0; }
    if (increment==+INT_MAX) { increment = -1; skip_first_advance = true; i0 = filenames.num()-1; }
    string smess;
    double mess_time = 4.;
    for (int i = i0; ; ) {
        if (skip_first_advance) {
            skip_first_advance = false;
        } else {
            i = my_mod(i+increment, filenames.num());
            if (i==i0) {
                message(smess + "No other video/image in " + directory, mess_time);
                return false;
            }
        }
        const string& filename_tail = filenames[i];
        string filename = directory + "/" + filename_tail;
        if (!filename_is_media(filename_tail) || !file_exists(filename)) // it may have been very recently deleted
            continue;
        try {
            unique_ptr<Object> pob = object_reading_file(filename); // may throw
            if (getob().is_image()) { // save current image into prefetch buffer
                Object& ob = getob();
                Image image = reduce_grid_rank(std::move(ob._video));
                image.attrib() = ob._image_attrib;
                if (!ob._image_is_bgra && k_use_bgra) convert_rgba_bgra(image);
                if (ob._image_is_bgra && !k_use_bgra) convert_bgra_rgba(image);
                int ii = increment==1 ? 1 : 0;
                if (g_verbose>=1) SHOW("saving_to_prefetch", ii, ob._filename);
                std::lock_guard<std::mutex> lg(g_mutex_prefetch);
                g_prefetch_image[ii].filename = ob._filename;
                g_prefetch_image[ii].file_modification_time = ob._file_modification_time;
                g_prefetch_image[ii].pimage = make_unique<Image>(std::move(image));
            }
            g_obs[g_cob] = std::move(pob);
            if (0) smess += "Loaded " + getob().stype() + " " + filename_tail;
            message(smess, mess_time); // may be "", in which case we clear messages
            set_video_frame(g_cob, k_before_start, k_force_refresh);
            return true;
        }
        catch (std::runtime_error& ex) {
            smess += "(Error opening " + filename_tail + " : " + ex.what() + ") ";
            mess_time = 10;
        }
    }
}

Array<string> get_image_sequence(const string& filename) {
    assertx(!file_requires_pipe(filename));
    string rootname = get_path_root(filename);
    string extension = get_path_extension(filename);
    string::size_type i = rootname.find_last_not_of("0123456789");
    i = i==string::npos ? 0 : i+1; // point to first digit
    string base = rootname.substr(0, i);
    int ndigits = narrow_cast<int>(rootname.size()-i);
    if (!ndigits) return {};
    int val = to_int(&rootname[i]);
    Array<string> ar;
    for (;;) {
        string s = base + sform("%0*d", ndigits, val) + "." + extension;
        if (!file_exists(s)) break;
        ar.push(s);
        val++;
    }
    return ar;
}

void reset_sliders() {
    for (const Slider& slider : g_sliders) { *slider.pval = 1.f; }
}

void initiate_loop_request() {
    Object& ob = getob();
    bool no_trim_region = !ob._framein && ob._frameou1==ob.nframes();
    const double framerate = ob._video.attrib().framerate;
    const double vtime = ob.nframes()/framerate; // video length in seconds
    const double max_keep = 5.;                  // maximum number of seconds to keep
    const double max_skip = 2.;                  // maximum number of seconds to skip at front
    const double extra = .1;    // 150 frames at 29.97 frames/sec is not really that much more than 5 seconds
    if (!g_lp.is_loaded && no_trim_region && vtime>max_keep+extra) {
        message("Input video is long and untrimmed, so automatically setting a trim region", 20.);
        ob._framein = max(int(min(vtime-max_keep, max_skip)*framerate), 1);
        ob._frameou1 = min(ob._framein+int(max_keep*framerate+.5), ob.nframes());
        SHOW("Setting trim region:", framerate, vtime, ob.nframes(), ob._framein, ob._frameou1);
    }
    g_request_loop = true;
}

void unload_current_object() {
    g_obs.erase(g_cob, 1);
    if (getobnum()) {
        int cob = min(int(g_cob), getobnum()-1);
        set_video_frame(cob, k_before_start, k_force_refresh);
    } else {
        g_cob = -1;
        if (min(g_win_dims, k_default_window_dims)!=k_default_window_dims)
            reset_window(k_default_window_dims);
        hw.redraw_later();
    }
}

template<typename R> Array<float> to_luminance(const R& range) {
    static_assert(std::is_same<iterator_t<R>, Pixel>::value, "");
    Array<float> ar; ar.reserve(int(distance(range)));
    for (const Pixel& pix : range) { ar.push(RGB_to_Y(pix)); }
    return ar;
}

// Remove the white boundary around an uncropped scanned image.
Matrix<Pixel> compute_wcrop(Matrix<Pixel> image) {
    // Test using: VideoViewer -key 'W<enter>i' ~/data/image/uncropped_photos/scan0182.jpg
    const bool ldebug = false;
    if (ldebug) SHOW(image.dims());
    if (1) {
        // Remove all-white borders on all sides
        const float luminance_thresh = 210.f;
        SGrid<int, 2, 2> borderw; fill(borderw, 0); // crop widths; [L==0|U==1][axis]
        for_int(axis, 2) {
            for_int(side, 2) {
                const int col_d = 1-axis; // axis==0 -> column;  axis==1 -> row
                const int wmax = image.dim(axis), len = image.dim(col_d); dummy_use(len);
                int w = 0;
                for(; w<wmax; w++) {
                    int j = side==0 ? w : wmax-1-w;
                    auto range = grid_column(image, col_d, twice(0).with(axis, j));
                    if (rankf_element(to_luminance(range), .1)<luminance_thresh) break;
                }
                borderw[side][axis] = w;
            }
        }
        if (ldebug) SHOW(borderw);
        if (!is_zero(borderw)) image = crop(image, borderw[0], borderw[1]);
    }
    if (1) {
        SGrid<int, 2, 2> borderw; fill(borderw, 0); // crop widths; [L==0|U==1][axis]
        if (ldebug) SHOW(image.dims());
        // Remove mostly white borders on all sides
        const float luminance_thresh = 220.f;
        for_int(axis, 2) {
            for_int(side, 2) {
                const int col_d = 1-axis; // axis==0 -> column;  axis==1 -> row
                const int wmax = image.dim(axis), len = image.dim(col_d);
                auto outermost_column = grid_column(image, col_d, twice(0).with(axis, side==0 ? 0 : wmax-1));
                float outermost_luminance = float(median(to_luminance(outermost_column)));
                if (outermost_luminance<luminance_thresh) continue; // outermost column is not white enough
                Array<int> ar_w(len);
                for_int(i, len) {
                    Vec2<int> u; u[col_d] = i;
                    int w = 0;
                    for (; w<wmax; w++) {
                        u[axis] = side==0 ? w : wmax-1-w;
                        if (RGB_to_Y(image[u])<luminance_thresh) break;
                    }
                    ar_w[i] = w;
                }
                // int w = rankf_element(ar_w, .95);
                int w = rankf_element(ar_w, .70);
                if (ldebug) { SHOW(axis, side, wmax, len, w); if (0) SHOW(ar_w); }
                borderw[side][axis] = w;
            }
        }
        if (ldebug) SHOW(borderw);
        if (!is_zero(borderw)) image = crop(image, borderw[0], borderw[1]);
    }
    if (1) {
        SGrid<int, 2, 2> borderw; fill(borderw, 0); // crop widths; [L==0|U==1][axis]
        if (ldebug) SHOW(image.dims());
        // Examine abnormal change in luminace on outermost columns.
        for_int(axis, 2) {
            for_int(side, 2) {
                const int col_d = 1-axis; // axis==0 -> column;  axis==1 -> row
                const int wmax = image.dim(axis), len = image.dim(col_d); dummy_use(len);
                Array<float> ar_luminance;
                for_int(w, min(30, wmax)) {
                    int j = side==0 ? w : wmax-1-w;
                    auto range = grid_column(image, col_d, twice(0).with(axis, j));
                    ar_luminance.push(float(mean(to_luminance(range))));
                }
                if (ldebug) SHOW(ar_luminance);
                for_int(w, ar_luminance.num()) {
                    if (ar_luminance[w]<ar_luminance.last()*1.1f) {
                        borderw[side][axis] = w;
                        break;
                    }
                }
            }
        }
        if (ldebug) SHOW(borderw);
        if (!is_zero(borderw)) image = crop(image, borderw[0], borderw[1]);
    }
    return image;
}


bool DerivedHW::key_press(string skey) {
    // HH_TIMER(key_press);
    bool recognized = true;
    static string prev_skey1, prev_skey2;  prev_skey2 = prev_skey1; prev_skey1 = skey;
    auto func_switch_ob = [&](int obi) {
        set_video_frame(obi, float(getob(obi)._framenum));
        message("Switched to " + getob().stype() + " " + get_path_tail(getob()._filename));
    };
    bool is_shift =   get_key_modifier(HW::EModifier::shift);
    bool is_control = get_key_modifier(HW::EModifier::control);
    // bool is_alt =     get_key_modifier(HW::EModifier::alt);
    if (0) SHOW(skey, int(skey[0]), is_shift, is_control);
    if (g_cob>=0 && getob().is_image()) {
        if (skey=="<left>")  skey = "<prior>";
        if (skey=="<right>") skey = "<next>";
    }
    if ((skey=="/" || skey=="\x1F") && is_control) skey = "?"; // control-/ is also help
    int keycode = skey[0];
    try {
        if (0) {
        } else if (skey=="<f1>") { // help
            return key_press("?");
        } else if (skey=="<f2>") { // rename file
            std::lock_guard<std::mutex> lg(g_mutex_obs);
            Object& ob = check_loaded_saved_object();
            string old_filename = ob._filename;
            string old_type = ob.stype();
            string new_filename = old_filename;
            if (!query(V(20, 10), "Rename " + old_type + " to (or <esc>): ", new_filename)) throw string();
            if (new_filename==old_filename) throw string("source and destination are identical");
            if (file_exists(new_filename)) throw "file " + new_filename + " already exists";
            bool success = !rename(old_filename.c_str(), new_filename.c_str()); // like command mv(1)
            if (!success) throw "could not rename " + old_filename + " to " + new_filename;
            message("Renamed " + old_type + " to " + new_filename);
            ob._filename = new_filename;
            ob._orig_filename = new_filename;
            g_dir_media_filenames.invalidate();
        } else if (skey=="<f5>") { // reload from file
            std::lock_guard<std::mutex> lg(g_mutex_obs);
            Object& ob0 = check_object();
            string filename = ob0._filename;
            Vec2<int> osdims = ob0.spatial_dims();
            for (;;) {
                if (file_requires_pipe(filename) || file_exists(filename)) break;
                string sroot = get_path_root(filename);
                auto i = sroot.rfind('_');
                if (i==string::npos) throw "cannot find file for " + ob0._filename;
                filename = sroot.substr(0, i) + "." + get_path_extension(filename);
            }
            try {
                g_obs[g_cob] = ob0._is_image ? object_reading_image(filename) : object_reading_video(filename);
            }
            catch (std::runtime_error& ex) {
                throw "while re-reading " + getob().stype() + " from " + filename + " : " + ex.what();
            }
            Object& ob = getob();
            message("Reloading " + ob.stype() + " " + get_path_tail(filename));
            set_video_frame(g_cob, k_before_start, k_force_refresh);
            ob._filename = filename;
            if (ob.spatial_dims()!=osdims) reset_window(determine_default_window_dims(g_frame_dims));
        } else if (skey=="<f7>") { // move file
            std::lock_guard<std::mutex> lg(g_mutex_obs);
            check_loaded_saved_object();
            string old_filename = getob()._filename;
            string old_type = getob().stype();
            if (g_dest_dir=="") g_dest_dir = get_path_head(old_filename);
            if (!query(V(20, 10), "Move " + old_type + " to directory (or <esc>): ", g_dest_dir)) throw string();
            if (g_dest_dir==get_path_head(old_filename)) throw string("source and destination are identical");
            if (!directory_exists(g_dest_dir)) throw string("destination directory does not exist");
            string new_filename = g_dest_dir + "/" + get_path_tail(old_filename);
            if (file_exists(new_filename)) throw "file " + new_filename + " already exists";
            bool next_loaded = replace_with_other_object_in_directory(+1);
            bool success = !rename(old_filename.c_str(), new_filename.c_str()); // like command mv(1)
            // SHOW(old_type, old_filename, new_filename, next_loaded, success);
            if (!success) {
                if (next_loaded) replace_with_other_object_in_directory(-1);
                throw "could not move " + old_filename + " to " + new_filename;
            }
            if (next_loaded) {
                message("Moved " + old_type + " to " + new_filename + " and loaded next object");
            } else {
                message("Moved " + old_type + " to " + new_filename + "; no other files in directory");
                unload_current_object();
            }
            g_dir_media_filenames.invalidate();
        } else if (skey=="<f8>") { // copy file
            std::lock_guard<std::mutex> lg(g_mutex_obs);
            const Object& ob = check_saved_object();
            string old_filename = ob._filename;
            string old_type = ob.stype();
            if (g_dest_dir=="") g_dest_dir = get_path_head(old_filename);
            if (!query(V(20, 10), "Copy " + old_type + " to directory (or <esc>): ", g_dest_dir)) throw string();
            if (g_dest_dir==get_path_head(old_filename)) throw string("source and destination are identical");
            if (!directory_exists(g_dest_dir)) throw string("destination directory does not exist");
            string new_filename = g_dest_dir + "/" + get_path_tail(old_filename);
            if (file_exists(new_filename)) {
                string s = "yes";
                if (!query(V(20, 10), "OK to overwrite " + new_filename + ": ", s) || s!="yes") throw string();
            }
            try {               // like command "cp"
                {
                    RFile fi(old_filename);
                    WFile fi2(new_filename);
                    fi2() << fi().rdbuf(); // copy the entire file
                }
                assertw(set_path_modification_time(new_filename, get_path_modification_time(old_filename)));
            }
            catch (const std::runtime_error& ex) {
                throw "could not copy " + old_filename + " to " + new_filename + " : " + ex.what();
            }
            message("Copied " + old_type + " to " + new_filename);
        } else if (keycode=='L'-64 && !is_shift) { // C-l is unbound
            beep();
        } else if (keycode=='R'-64 && !is_shift) { // C-r is unbound
            beep();
        } else if (skey=="<f11>" || skey=="<enter>") { // fullscreen <enter>/<ret>
            return key_press("\r");
        } else if (skey=="<left>") { // select frame-1
            if (g_cob<0) throw string("no loaded objects");
            g_playing = false;
            double dframetime = is_shift ? 10. : 1.;
            double nframetime = min(int(floor(g_frametime)), getob()._nframes_loaded-1)-dframetime;
            int obi = g_cob;
            if (nframetime<0.) {
                if (g_looping==ELooping::all) obi = g_cob>0 ? g_cob-1 : getobnum()-1;
                if ((g_looping==ELooping::one || g_looping==ELooping::all) &&
                    getob(obi)._nframes_loaded==getob(obi).nframes()) nframetime = getob(obi).nframes()-1.;
            }
            set_video_frame(obi, nframetime);
        } else if (skey=="<right>") { // select frame+1
            if (g_cob<0) throw string("no loaded objects");
            g_playing = false;
            double dframetime = is_shift ? 10. : 1.;
            double nframetime = max(int(floor(g_frametime)), 0)+dframetime;
            int obi = g_cob;
            if (nframetime>=getob().nframes()) {
                if (g_looping==ELooping::all) obi = g_cob<getobnum()-1 ? g_cob+1 : 0;
                if (g_looping==ELooping::one || g_looping==ELooping::all) nframetime = k_before_start;
            }
            set_video_frame(obi, nframetime);
        } else if (skey=="<home>") { // select first directory file or first video frame
            if (g_cob<0) throw string("no loaded objects");
            if (getob()._is_image || is_control) { // load first object in directory
                std::lock_guard<std::mutex> lg(g_mutex_obs);
                if (!replace_with_other_object_in_directory(-INT_MAX)) beep();
            } else {            // jump to first frame in video
                g_playing = false;
                set_video_frame(g_cob, k_before_start);
            }
        } else if (skey=="<end>") { // select last directory file or last video frame
            if (g_cob<0) throw string("no loaded objects");
            if (getob()._is_image || is_control) { // load last object in directory
                std::lock_guard<std::mutex> lg(g_mutex_obs);
                if (!replace_with_other_object_in_directory(+INT_MAX)) beep();
            } else {            // jump to last frame in video
                g_playing = false;
                set_video_frame(g_cob, getob().nframes()-1.);
            }
        } else if (skey=="<prior>") { // previous object in directory
            std::lock_guard<std::mutex> lg(g_mutex_obs);
            if (!replace_with_other_object_in_directory(-1)) beep();
        } else if (skey=="<next>") { // next object in directory
            std::lock_guard<std::mutex> lg(g_mutex_obs);
            if (!replace_with_other_object_in_directory(+1)) beep();
        } else if (skey=="<delete>") { // delete file
            std::lock_guard<std::mutex> lg(g_mutex_obs);
            check_saved_object();
            string old_filename = getob()._filename;
            string old_type = getob().stype();
            if (!g_prompted_for_delete) {
                string s = "yes";
                if (!query(V(20, 10), "OK to delete " + old_type + " '" + old_filename + "': ", s) || s!="yes")
                    throw string();
                g_prompted_for_delete = true;
            }
            bool next_loaded = replace_with_other_object_in_directory(+1);
            if (!recycle_path(old_filename)) {
                if (next_loaded) replace_with_other_object_in_directory(-1);
                throw "could not delete " + old_filename;
            }
            if (next_loaded) {
                message("Deleted current " + old_type + " and loaded next " + getob().stype());
            } else {
                message("Deleted current " + old_type + "; no other files in directory");
                unload_current_object();
            }
            g_dir_media_filenames.invalidate();
        } else if (skey=="<esc>") { // exit, from -key "<esc>" or -hwdelay 2 -hwkey '<enter><esc>'; see also '\033'
            quit();
        } else if (skey.size()==1 && keycode>='1' && keycode<='9' && is_control) { // C-1 ... C-9: select object
            int ob = keycode-'1';
            if (ob<getobnum()) {
                func_switch_ob(ob);
            } else beep();
        } else if (skey.size()!=1) {
            recognized = false;
        } else {
            switch (keycode) {
// Object/play controls
             bcase 'r': {       // reset sliders or reset all parameters
                 if (g_use_sliders) {
                     reset_sliders();
                     // message("Reset sliders");
                     redraw_later();
                 } else {
                     if (0) g_playing = true;
                     if (0 && g_cob>=0) set_video_frame(0, k_before_start);
                     g_looping = k_default_looping;
                     g_mirror_state_forward = true;
                     set_fullscreen(false); // OK to subsequently call reset_window() below before a draw_window()?
                     g_speed = 1.;
                     g_fit = k_default_fit;
                     g_fit_view_to_window = true; // also set by reset_window()
                     g_kernel = k_default_kernel;
                     g_show_info = k_default_info;
                     g_other_show_info = false;
                     g_show_exif = false;
                     if (0) g_show_help = false;
                     for_int(obi, getobnum()) {
                         Object& ob = getob(obi);
                         ob._framein = 0;
                         ob._frameou1 = ob._dims[0];
                     }
                     reset_window(determine_default_window_dims(g_frame_dims));
                     message("All parameters reset to defaults", 5.);
                 }
             }
             bcase ' ': {       // run (toggle play/pause video)
                 if (g_cob>=0 && !g_playing) {
                     if (g_frametime>=getob()._nframes_loaded-1. ||
                         g_frametime>=getob()._frameou1-1.)
                         set_video_frame(g_cob, getob()._framein ? getob()._framein-.001 : k_before_start);
                 }
                 g_playing = !g_playing;
                 if (g_playing) redraw_later();
             }
             bcase 'l': {       // loop one
                 set_looping(g_looping==ELooping::one ? ELooping::off : ELooping::one);
             }
             bcase 'a': {       // loop all
                 set_looping(g_looping==ELooping::all ? ELooping::off : ELooping::all);
             }
             bcase 'm': {       // loop mirror
                 set_looping(g_looping==ELooping::mirror ? ELooping::off : ELooping::mirror);
                 g_mirror_state_forward = true;
             }
             bcase '[': {       // slow down video by 2x
                 set_speed(g_speed*.5);
             }
             bcase ']': {       // speed up video by 2x
                 set_speed(g_speed*2.);
             }
             bcase '{': {       // slow down video among preselected speeds
                 if (0) set_speed(k_speeds.inside(k_speeds.index(g_speed)-1, Bndrule::clamped));
                 int index = discrete_binary_search(concat(k_speeds, V(std::numeric_limits<double>::max())),
                                                    0, k_speeds.num(),
                                                    clamp(g_speed*.999999f, k_speeds[0], k_speeds.last()));
                 set_speed(k_speeds.inside(index+0, Bndrule::clamped));
             }
             bcase '}': {       // speed up video among preselected speeds
                 if (0) set_speed(k_speeds.inside(k_speeds.index(g_speed)+1, Bndrule::clamped));
                 int index = discrete_binary_search(concat(k_speeds, V(std::numeric_limits<double>::max())),
                                                    0, k_speeds.num(),
                                                    clamp(g_speed, k_speeds[0], k_speeds.last()));
                 set_speed(k_speeds.inside(index+1, Bndrule::clamped));
             }
             bcase '1': ocase '\\': { // 1x speed
                 set_speed(1.);
             }
             bcase '2': {       // 2x speed
                 set_speed(2.);
             }
             bcase '5': {       // .5x speed
                 set_speed(.5);
             }
             bcase '\t': {       // <tab> == C-i (== uchar{9} == 'I'-64),   previous/next object
                 if (is_shift) { // previous object
                     return key_press("p");
                 } else {       // next object
                     return key_press("n");
                 }
             }
             bcase 'p': {       // previous object
                 if (g_cob>0) func_switch_ob(g_cob-1);
                 else if (g_cob>=0 && g_looping==ELooping::all) func_switch_ob(getobnum()-1);
                 else beep();
             }
             bcase 'n': {       // next object
                 if (g_cob<getobnum()-1) func_switch_ob(g_cob+1);
                 else if (g_cob>=0 && g_looping==ELooping::all) func_switch_ob(0);
                 else beep();
             }
             bcase 'P': {       // first object
                 if (g_cob>=0) func_switch_ob(0);
                 else beep();
             }
             bcase 'N': {       // last object
                 if (g_cob>=0) func_switch_ob(getobnum()-1);
                 else beep();
             }
             bcase 'x': {       // exchange object with previous one
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 check_object();
                 if (g_cob<1) throw string("no prior object to exchange with");
                 std::swap(g_obs[g_cob], g_obs[g_cob-1]);
                 g_cob--;
                 if (0) set_video_frame(g_cob, g_framenum); // would force unnecessary texture refresh
                 message("Moved object earlier than " + g_obs[g_cob+1]->_filename);
             }
             bcase 'X': {       // exchange object with next one
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 check_object();
                 if (g_cob==getobnum()-1) throw string("no next object to exchange with");
                 std::swap(g_obs[g_cob], g_obs[g_cob+1]);
                 g_cob++;
                 if (0) set_video_frame(g_cob, g_framenum); // would force unnecessary texture refresh
                 message("Moved object later than " + g_obs[g_cob-1]->_filename);
             }
             bcase 's': {       // change directory sort type
                 switch (g_sort) {
                  bcase ESort::name:
                     g_sort = ESort::date; message("Directory sort for <pgdn>,<pgup> set to 'date'", 5.);
                     g_dir_media_filenames.invalidate();
                  bcase ESort::date:
                     g_sort = ESort::name; message("Directory sort for <pgdn>,<pgup> set to 'name'", 5.);
                     g_dir_media_filenames.invalidate();
                  bdefault: assertnever("");
                 }
             }
// Window controls
             bcase 'f': {       // fit anisotropically
                 if (!g_fit_view_to_window) {
                     g_fit_view_to_window = true;
                     message("View scaling set to fit window");
                 } else if (g_fit==EFit::isotropic) {
                     g_fit = EFit::anisotropic;
                     message("View scaling set to anisotropic");
                 } else if (g_fit==EFit::anisotropic) {
                     g_fit = EFit::isotropic;
                     message("View scaling set to isotropic");
                 } else assertnever("");
             }
             bcase 'w': {       // window fit
                 g_fit_view_to_window = !g_fit_view_to_window;
                 message(g_fit_view_to_window ?
                         "Zoom set to adjust to window" :
                         "Zoom set to be independent of window");
             }
             bcase '0': {       // 100% zoom
                 const Vec2<int> dims = product(g_frame_dims) ? g_frame_dims : k_default_window_dims;
                 if (is_fullscreen()) {
                     g_fit_view_to_window = false;
                     set_view(Frame::translation(concat(convert<float>(g_win_dims-dims), V(0.f))/2.f));
                     if (g_win_dims==dims)
                         message("Zoom set to 100%");
                     else
                         message("Zoom set to 100%; press <f> to see entire frame", 5.);
                 } else {
                     Vec2<int> owin_dims = g_win_dims;
                     Vec2<int> win_dims = determine_default_window_dims(dims);
                     // SHOW(dims, win_dims, owin_dims, get_max_window_dims());
                     reset_window(win_dims); // (does not immediately update g_win_dims)
                     if (win_dims==dims) {
                         message("Zoom set to 100%");
                     } else if (win_dims!=owin_dims) {
                         message("Zoom set for best screen fit; press <0> again for 100% zoom", 5.);
                     } else {
                         g_fit_view_to_window = false;
                         set_view(Frame::translation(concat(convert<float>(win_dims-dims), V(0.f))/2.f));
                         message("Zoom set to 100%; press <f> to see entire frame", 5.);
                     }
                 }
             }
             bcase '=': ocase '+': { // increase window size or zoom
                 perform_window_zoom(k_key_zoom_fac);
             }
             bcase '-': {       // decrease window size or zoom
                 perform_window_zoom(1.f/k_key_zoom_fac);
             }
             bcase '\r':                // <enter>/<ret>/C-M key (== uchar{13} == 'M'-64),  toggle fullscreen
             ocase '\n':                // G3d -key $'\n'
             {
                 g_fit_view_to_window = true;
                 set_fullscreen(!is_fullscreen());
             }
             bcase 'k': {       // rotate among reconstruction kernels
                 g_kernel = EKernel(my_mod(int(g_kernel)+1, int(EKernel::last)));
                 message(string() + "Reconstruction kernel set to: " + k_kernel_string[int(g_kernel)]);
             }
             bcase 'K': {       // rotate among reconstruction kernels
                 g_kernel = EKernel(my_mod(int(g_kernel)-1, int(EKernel::last)));
                 message(string() + "Reconstruction kernel set to: " + k_kernel_string[int(g_kernel)]);
             }
// Other
             bcase 'O'-64: {    // C-o: open an existing video/image file
                 string cur_filename = (g_cob>=0 && !file_requires_pipe(getob()._filename) ? getob()._filename :
                                        get_current_directory() + '/');
                 Array<string> filenames = query_open_filenames(cur_filename);
                 int first_cob_loaded = -1;
                 string smess;
                 for (const string& pfilename : filenames) {
                     string filename = get_path_absolute(pfilename);
                     if (!file_exists(filename)) {
                         smess += " (File '" + filename + "' not found)";
                         continue;
                     }
                     std::lock_guard<std::mutex> lg(g_mutex_obs);
                     try {
                         add_object(object_reading_file(filename)); // may throw
                         set_video_frame(g_cob, k_before_start);    // set to first frame
                         if (first_cob_loaded<0) first_cob_loaded = g_cob;
                     }
                     catch (std::runtime_error& ex) {
                         smess += " (Error reading file " + filename + " : " + ex.what() + ")";
                     }
                 }
                 if (first_cob_loaded>=0) {
                     set_video_frame(first_cob_loaded, k_before_start);
                     reset_window(determine_default_window_dims(g_frame_dims));
                 }
                 if (smess!="") throw smess;
             }
             bcase 'S'-64: {    // C-s: save video/image to file;  C-S-s: overwrite original file
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 Object& ob = check_loaded_object();
                 if (g_use_sliders) throw string("close sliders first before saving file");
                 string cur_filename = ob._filename;
                 if (is_shift && ob._orig_filename!="") cur_filename = ob._orig_filename;
                 if (is_shift && ob._file_modification_time && file_exists(cur_filename) &&
                     ob._file_modification_time!=get_path_modification_time(cur_filename)) {
                     ob._file_modification_time = 0; // succeed if try again
                     throw "file '" + cur_filename + "' has been modified externally";
                 }
                 bool force = is_shift;
                 string filename = query_save_filename(cur_filename, force);
                 if (filename=="") throw string("");
                 immediate_message("Writing to file " + filename + " ...");
                 try {
                     uint64_t time = 0;
                     if (is_shift) time = get_path_modification_time(filename);
                     if (ob.is_image()) {
                         Image image = reduce_grid_rank(std::move(ob._video));
                         image.attrib() = ob._image_attrib;
                         if (ob._image_is_bgra) {
                             image.write_file_bgra(filename);
                         } else {
                             image.write_file(filename);
                         }
                         Grid<2,Pixel> grid = std::move(image);
                         ob._video = increase_grid_rank(std::move(grid));
                     } else if (ob._video.size()) {
                         ob._video.write_file(filename);
                     } else if (ob._video_nv12.size()) {
                         ob._video_nv12.write_file(filename, ob._video.attrib());
                     } else assertnever("");
                     message("Done writing '" + get_path_tail(filename) + "'", 4.);
                     ob._unsaved = false;
                     ob._filename = filename;
                     ob._orig_filename = filename;
                     if (time) assertw(set_path_modification_time(filename, time));
                 }
                 catch (const std::runtime_error& ex) {
                     throw "while writing file " + filename + " : " + ex.what();
                 }
             }
             bcase 'v': {       // view externally (using default "start" association)
                 if (g_cob<0) throw string("no loaded objects");
                 view_externally();
             }
             bcase 'D'-64: {    // C-d: unload current image/video from viewer
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 Object& ob = check_object();
                 message("Unloaded " + ob.stype() + " " + get_path_tail(ob._filename), 4.);
                 unload_current_object();
             }
             bcase 'K'-64: {    // C-k: unload all objects except current one
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 check_object();
                 check_all_objects();
                 g_obs.erase(0, g_cob);
                 g_obs.erase(1, g_obs.num()-1);
                 g_cob = 0;
                 if (0) set_video_frame(g_cob, g_framenum); // would force unnecessary texture refresh
                 message("Unloaded all but current object", 4.);
             }
             bcase 'N'-64: {    // C-n: open new VideoViewer window on same file
                 if (g_cob<0) throw string("no loaded object");
                 string filename = getob()._filename;
                 if (filename=="") throw getob().stype() + " has no filename";
                 if (my_spawn(V<string>(g_argv0, filename), false))
                     throw "failed to create new VideoViewer window on '" + filename + "'";
                 if (g_verbose) SHOW("spawned new window", g_argv0, filename);
             }
             bcase 'C': {       // crop to view (and resample content if view includes a rotation)
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_object();
                 if (view_has_rotation()) {            // view includes a rotation
                     assertx(var(get_zooms())<1e-10f); // zoom must be isotropic if rotation is present
                     const Vec2<int> osdims = ob.spatial_dims();
                     Vec2<int> nsdims = convert<int>(convert<float>(g_win_dims)/get_zooms()[0]+.5f);
                     if (!ob.is_image()) nsdims = nsdims/4*4; // video should have dims that are multiples of 4
                     auto scale_win = concat(convert<float>(g_win_dims), V(1.f));
                     auto scale_frame = concat(convert<float>(g_frame_dims), V(1.f));
                     // frame maps from/to [-0.5, +0.5]^2 whereas g_view maps from/to pixel/texel coordinates
                     Frame frame = Frame::translation(thrice(.5f)) * Frame::scaling(scale_win) * ~g_view *
                         ~Frame::scaling(scale_frame) * ~Frame::translation(thrice(.5f));
                     const FilterBnd filterb(get_resampling_filter(), Bndrule::reflected);
                     immediate_message("Resampling rotated object...");
                     Video nvideo;
                     VideoNv12 nvideo_nv12;
                     if (ob._video.size()) {
                         nvideo.init(concat(V(ob.nframes()), nsdims));
                         parallel_for_int(f, ob.nframes()) {
                             Matrix<Vector4> matv(osdims); convert(ob._video[f], matv);
                             Matrix<Vector4> nmatv(nsdims); transform(matv, frame, twice(filterb), nmatv);
                             convert(nmatv, nvideo[f]);
                         }
                     } else {
                         nvideo_nv12.init(concat(V(ob.nframes()), nsdims));
                         parallel_for_int(f, ob.nframes()) {
                             Matrix<float> matv(osdims); convert(ob._video_nv12.get_Y()[f], matv);
                             Matrix<float> nmatv(nsdims); transform(matv, frame, twice(filterb), nmatv);
                             convert(nmatv, nvideo_nv12.get_Y()[f]);
                         }
                         parallel_for_int(f, ob.nframes()) {
                             Matrix<Vector4> matv(osdims/2); convert(ob._video_nv12.get_UV()[f], matv);
                             Matrix<Vector4> nmatv(nsdims/2); transform(matv, frame, twice(filterb), nmatv);
                             convert(nmatv, nvideo_nv12.get_UV()[f]);
                         }
                     }
                     add_object(make_unique<Object>(ob, std::move(nvideo), std::move(nvideo_nv12),
                                                    append_to_filename(ob._filename, "_resampled")));
                     g_fit_view_to_window = true;
                     message("Resampled " + ob.stype());
                     // if (nsdims!=osdims) reset_window(determine_default_window_dims(g_frame_dims));
                 } else {       // no rotation, so crop without resampling
                     Vec2<int> yxL, yxU; fully_visible_image_rectangle(yxL, yxU);
                     if (!ob.is_image()) {
                         // Video should have dims that are multiples of 4.
                         // Also, crop offsets should be multiples of 2 for VideoNV12 UV access.
                         yxL = (yxL+0)/4*4;
                         yxU = (yxU+3)/4*4;
                     }
                     const Vec2<int> nsdims = yxU-yxL;
                     if (0) SHOW("crop", ob.spatial_dims(), yxL, yxU, nsdims);
                     if (image_is_fully_visible())
                         throw string("cropping has no effect since entire image is visible");
                     if (!product(nsdims))
                         throw string("cropping to this view would result in zero-sized image");
                     immediate_message("Applying spatial cropping...");
                     Video nvideo;
                     VideoNv12 nvideo_nv12;
                     const Vec2<int> cL = yxL, cU = ob.spatial_dims()-yxU;
                     if (ob._video.size()) { // includes the case of ob.is_image()
                         nvideo = crop(ob._video, concat(V(0), cL), concat(V(0), cU));
                     } else {
                         nvideo_nv12 =
                             VideoNv12(crop(ob._video_nv12.get_Y(), concat(V(0), cL), concat(V(0), cU)),
                                       crop(ob._video_nv12.get_UV(), concat(V(0), cL)/2, concat(V(0), cU)/2));
                     }
                     add_object(make_unique<Object>(ob, std::move(nvideo), std::move(nvideo_nv12),
                                                    append_to_filename(ob._filename, "_crop")));
                     g_fit_view_to_window = true;
                     message("Cropped " + ob.stype());
                 }
             }
             bcase 'W': {
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_image();
                 assertx(ob._video.size());
                 Matrix<Pixel> nimage = compute_wcrop(Matrix<Pixel>(ob._video[0]));
                 if (!nimage.size()) throw string("resulting image would be empty");
                 Video nvideo = increase_grid_rank(std::move(nimage));
                 add_object(make_unique<Object>(ob, std::move(nvideo), VideoNv12{},
                                                append_to_filename(ob._filename, "_wcrop")));
                 g_fit_view_to_window = true;
                 message("Cropped the white borders from image");
             }
             bcase 'S': {       // scale (resample content to current view resolution)
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_object();
                 if (max_abs_element(V(g_view[0][1], g_view[1][0]))>0.f) throw ob.stype() + " is rotated";
                 Vec2<int> ndims;
                 if (abs(g_view[0][0]-g_view[1][1])>1e-6f) { // anisotropic window fit
                     ndims = convert<int>(convert<float>(g_frame_dims)*V(g_view[0][0], g_view[1][1]));
                 } else {
                     string s = sform("%g", g_view[0][0]);
                     if (!query(V(20, 10), "Scale by spatial factor: ", s)) throw string("");
                     if (!Args::check_float(s)) throw string("spatial factor is not a float");
                     float fac = Args::parse_float(s);
                     ndims = convert<int>(convert<float>(g_frame_dims)*twice(fac));
                 }
                 if (!ob.is_image() && ob._video.attrib().suffix!="avi") {
                     ndims = (ndims+2)/4*4; // video should have dims that are multiples of 4
                 } else {
                     if (0) ndims = (ndims+1)/2*2; // no need to round images to even sizes
                 }
                 Vec2<float> syx = convert<float>(ndims)/convert<float>(g_frame_dims);
                 const auto filterbs = twice(FilterBnd(get_resampling_filter(), Bndrule::reflected));
                 immediate_message("Spatially rescaling object...");
                 Video nvideo;
                 VideoNv12 nvideo_nv12;
                 if (ob._video.size()) { // includes the case of ob.is_image()
                     nvideo = scale(ob._video, syx, filterbs);
                 } else {
                     nvideo_nv12 = scale(ob._video_nv12, syx, filterbs);
                 }
                 add_object(make_unique<Object>(ob, std::move(nvideo), std::move(nvideo_nv12),
                                                append_to_filename(ob._filename, "_scaled")));
                 reset_window(determine_default_window_dims(g_frame_dims));
                 message("Rescaled " + ob.stype());
             }
             bcase 'A': {       // select window aspect ratio
                 string s;
                 if (!query(V(20, 10), "Window aspect ratio (e.g. 1.5 or 16:9): ", s)) throw string("");
                 float ratio = -1.f;
                 float v1, v2;
                 if (sscanf(s.c_str(), "%g:%g", &v1, &v2)==2) {
                     if (v1>0.f && v2>0.f) ratio = v1/v2;
                 } else if (sscanf(s.c_str(), "%g", &v1)==1) {
                     if (v1>0.f) ratio = v1;
                 }
                 if (ratio<=0.f) throw string("invalid aspect ratio");
                 set_fullscreen(false); // OK to subsequently call resize_window() below before a draw_window()?
                 const int nlarge = 1000000;
                 Vec2<int> ndims = determine_default_window_dims(V(nlarge, int(nlarge*ratio+.5f)));
                 resize_window(ndims);
                 g_fit_view_to_window = false;
                 Frame view; {
                     Vec2<float> arzoom = convert<float>(ndims) / convert<float>(g_frame_dims);
                     const int cmax = arzoom[0]>arzoom[1] ? 0 : 1;
                     view = Frame::scaling(concat(twice(arzoom[cmax]), V(1.f)));
                     view[3][1-cmax] = (ndims[1-cmax]-g_frame_dims[1-cmax]*arzoom[cmax])/2.f;
                     // align window edge with pixel edge using fmod()
                     view[3][1-cmax] = view[3][1-cmax] - fmod(view[3][1-cmax], arzoom[cmax]);
                 }
                 set_view(view);
                 g_prev_win_dims = ndims; // do not look to translate image
             }
             bcase 'L'-64: ocase 'R'-64: { // C-S-l, C-S-r: rotate content 90-degrees left (ccw) or right (clw)
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const int rot_degrees = keycode=='L'-64 ? 90 : -90;
                 Object& ob = check_loaded_object();
                 if (ob._video.size()) { // includes the case of ob.is_image()
                     ob._video = rotate_ccw(ob._video, rot_degrees);
                 } else {
                     ob._video_nv12 = VideoNv12(rotate_ccw(ob._video_nv12.get_Y(), rot_degrees),
                                                rotate_ccw(ob._video_nv12.get_UV(), rot_degrees));
                 }
                 reverse(ob._dims.tail<2>());
                 {
                     string sroot = get_path_root(ob._filename), sext = get_path_extension(ob._filename);
                     const Array<string> ar { "_ccw", "_rot180", "_clw", "" };
                     int cur = -1; for_int(i, ar.num()) { if (ends_with(sroot, ar[i])) { cur = i; break; } }
                     assertx(cur>=0);
                     assertx(remove_at_end(sroot, ar[cur]));
                     ob._filename = sroot + ar[my_mod(cur+(rot_degrees/90), 4)] + "." + sext;
                     ob._unsaved = cur!=2 || !file_exists(ob._filename);
                 }
                 if (!is_fullscreen()) {
                     reset_window(determine_default_window_dims(ob.spatial_dims()));
                     // if (g_fit_view_to_window) reset_window(g_win_dims.rev());
                 }
                 g_fit_view_to_window = true;
                 set_video_frame(g_cob, g_framenum, k_force_refresh);
                 message("Rotated " + ob.stype());
             }
             bcase 'V': {       // convert sequence of images (starting from current) to a video
                 // vv ~/prevproj/2016/motiongraph/Other/20150720/Morphs/Dancer-MSECIELAB10000/Atlas-F*.png -key 'V'
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_image();
                 const int ibeg = g_cob;
                 int n = 0;
                 for_intL(i, g_cob, getobnum()) {
                     if (!getob(i).is_image() || getob(i).spatial_dims()!=ob.spatial_dims()) break;
                     n++;
                 }
                 if (n<2) throw string("must have at least one next same-size image to create a video");
                 string filename = get_path_root(ob._filename) + ".mp4";
                 Video video(n, ob.spatial_dims());
                 parallel_for_int(f, n) {
                     video[f].assign(g_obs[ibeg+f]->_video[0]);
                     if (g_obs[ibeg+f]->_image_is_bgra) convert_bgra_rgba(video[f]);
                 }
                 VideoNv12 video_nv12;
                 const bool use_nv12 = k_prefer_nv12 && is_zero(video.spatial_dims()%2);
                 if (use_nv12) {
                     video_nv12.init(video.dims());
                     convert_Video_to_VideoNv12(video, video_nv12);
                     video.clear();
                 }
                 const int new_cur_frame = 0;
                 set_video_frame(ibeg+n-1, g_framenum); // select the last image so we append video after it
                 add_object(make_unique<Object>(std::move(video), std::move(video_nv12),
                                                nullptr, std::move(filename)));
                 set_video_frame(g_cob, new_cur_frame);
                 g_obs.erase(g_cob-n, n); // unload the n images
                 g_cob -= n;
                 message(sform("Concatenated %d images to create this video", n), 6.);
             }
             bcase '#': {       // convert image sequence (incrementing current image name) to a video
                 // VideoViewer ~/proj/motiongraph/Other/20150720/Morphs/Dancer-MSECIELAB10000/Atlas-F00001.png -key '#'
                 // vv d:/Other/2015_06_12_HuguesH_Take2/Output_V1/Frames/view.F00001.png -key '#'
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_image();
                 if (file_requires_pipe(ob._filename)) throw string("image is loaded from a pipe");
                 string filename = get_path_root(ob._filename) + ".mp4";
                 Array<string> filenames = get_image_sequence(ob._filename);
                 int nframes = filenames.num();
                 if (nframes<2) throw string("cannot find an image sequence with >=2 frames");
                 immediate_message(sform("Reading %d image frames...", nframes));
                 const Vec3<int> dims = concat(V(nframes), ob.spatial_dims());
                 const bool use_nv12 = k_prefer_nv12 && is_zero(dims.tail<2>()%2);
                 Video nvideo(!use_nv12 ? dims : thrice(0));
                 VideoNv12 nvideo_nv12(use_nv12 ? dims : thrice(0));
                 std::atomic<bool> ok{true};
                 parallel_for_int(f, nframes) {
                     if (!ok) continue;
                     Image image; image.read_file(filenames[f]); // not bgra
                     if (image.dims()!=dims.tail<2>()) { ok = false; continue; }
                     if (!use_nv12) {
                         nvideo[f].assign(image);
                     } else {
                         convert_Image_to_Nv12(image, nvideo_nv12[f]);
                     }
                 }
                 if (!ok) throw string("images have differing dimensions");
                 add_object(make_unique<Object>(std::move(nvideo), std::move(nvideo_nv12),
                                                nullptr, std::move(filename)));
                 message("Created video from image sequence -- set <F>ramerate and <B>itrate", 6.);
             }
             bcase 'd': {       // open directory containing current object
                 string s = get_current_directory();
                 if (g_cob>=0 && directory_exists(get_path_head(getob()._filename)))
                     s = get_path_head(getob()._filename);
                 if (my_sh(V<string>("start", s)) &&
                     my_sh(V<string>("cygstart", s)))
                     throw "Could not launch directory window on " + s;
             }
             bcase '<': ocase ',': { // set IN frame
                 Object& ob = verify_video();
                 if (g_framenum<0) throw string("no current frame");
                 ob._framein = g_framenum;
                 if (ob._frameou1<=ob._framein) ob._frameou1 = ob.nframes();
                 message("Beginning frame of trim is now set");
             }
             bcase '>': ocase '.': { // set OUT frame
                 Object& ob = verify_video();
                 if (g_framenum<0) throw string("no current frame");
                 ob._frameou1 = g_framenum+1;
                 if (ob._framein>=ob._frameou1) ob._framein = 0;
                 message("End frame of trim is now set");
             }
             bcase 'u': {       // unmark (remove IN and OUT frames)
                 Object& ob = verify_video();
                 ob._framein = 0;
                 ob._frameou1 = ob.nframes();
                 message("Unmarked beginning and end trim frames");
             }
             bcase 'T': {       // temporal exterior video trim (shift-t)
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_video();
                 if (ob._framein==0 && ob._frameou1==ob.nframes())
                     throw string("to trim video, set beg and end frames using '<' and '>' keys.");
                 Video nvideo;
                 VideoNv12 nvideo_nv12;
                 const int trimbeg = ob._framein;
                 const int trimend = ob.nframes()-ob._frameou1;
                 assertx(trimbeg>=0 && trimend>=0);
                 if (ob._video.size()) {
                     nvideo = crop(ob._video, V(trimbeg, 0, 0), V(trimend, 0, 0));
                 } else if (ob._video_nv12.size()) {
                     nvideo_nv12 = VideoNv12(crop(ob._video_nv12.get_Y(),  V(trimbeg, 0, 0), V(trimend, 0, 0)),
                                             crop(ob._video_nv12.get_UV(), V(trimbeg, 0, 0), V(trimend, 0, 0)));
                 } else assertnever("");
                 const int new_cur_frame = g_framenum-trimbeg;
                 unique_ptr<Object> newob = make_unique<Object>(ob, std::move(nvideo), std::move(nvideo_nv12),
                                                                append_to_filename(ob._filename, "_trim"));
                 newob->_video.attrib().audio.clear(); // TODO
                 add_object(std::move(newob));
                 set_video_frame(g_cob, new_cur_frame);
                 message("Trimmed video");
             }
             bcase 'T'-64: {    // temporal interior video cut (control-shift-t)
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 if (!is_shift) { beep(); throw string(""); }
                 const Object& ob = check_loaded_video();
                 if (ob._framein==0 && ob._frameou1==ob.nframes())
                     throw string("to cut video interior, set beg and end frames using '<' and '>' keys.");
                 Video nvideo;
                 VideoNv12 nvideo_nv12;
                 const int ncut = ob._frameou1-ob._framein;
                 const Vec3<int> ndims = ob._dims - V(ncut, 0, 0);
                 if (ob._video.size()) {
                     nvideo.init(ndims);
                     nvideo.slice(0, ob._framein).assign(ob._video.slice(0, ob._framein));
                     nvideo.slice(ob._framein, ndims[0]).assign(ob._video.slice(ob._frameou1, ob._dims[0]));
                 } else if (ob._video_nv12.size()) {
                     nvideo_nv12.init(ndims);
                     auto oYY = ob._video_nv12.get_Y();  auto nYY = nvideo_nv12.get_Y();
                     auto oUV = ob._video_nv12.get_UV(); auto nUV = nvideo_nv12.get_UV();
                     nYY.slice(0, ob._framein).assign(oYY.slice(0, ob._framein));
                     nUV.slice(0, ob._framein).assign(oUV.slice(0, ob._framein));
                     nYY.slice(ob._framein, ndims[0]).assign(oYY.slice(ob._frameou1, ob._dims[0]));
                     nUV.slice(ob._framein, ndims[0]).assign(oUV.slice(ob._frameou1, ob._dims[0]));
                 } else assertnever("");
                 const int new_cur_frame = (g_framenum<ob._framein ? int(g_framenum) :
                                            g_framenum>=ob._frameou1 ? g_framenum-ncut :
                                            ob._framein);
                 unique_ptr<Object> newob = make_unique<Object>(ob, std::move(nvideo), std::move(nvideo_nv12),
                                                                append_to_filename(ob._filename, "_cut"));
                 newob->_video.attrib().audio.clear(); // TODO
                 add_object(std::move(newob));
                 set_video_frame(g_cob, new_cur_frame);
                 message("Cut video");
             }
             bcase '|': { // split current video into two, where current frame becomes first frame of second part
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_video();
                 const int nf = ob.nframes();
                 if (g_framenum<1) throw string("splitting requires selecting frame number >=1");
                 Video nvideo1, nvideo2;
                 VideoNv12 nvideo1_nv12, nvideo2_nv12;
                 if (ob._video.size()) {
                     nvideo1 = crop(ob._video, V(0, 0, 0), V(nf-g_framenum, 0, 0));
                     nvideo2 = crop(ob._video, V(int(g_framenum), 0, 0), V(0, 0, 0));
                 } else if (ob._video_nv12.size()) {
                     nvideo1_nv12 = VideoNv12(crop(ob._video_nv12.get_Y(),  V(0, 0, 0), V(nf-g_framenum, 0, 0)),
                                              crop(ob._video_nv12.get_UV(), V(0, 0, 0), V(nf-g_framenum, 0, 0)));
                     nvideo2_nv12 = VideoNv12(crop(ob._video_nv12.get_Y(),  V(int(g_framenum), 0, 0), V(0, 0, 0)),
                                              crop(ob._video_nv12.get_UV(), V(int(g_framenum), 0, 0), V(0, 0, 0)));
                 } else assertnever("");
                 unique_ptr<Object> newob1 = make_unique<Object>(ob, std::move(nvideo1), std::move(nvideo1_nv12),
                                                                 append_to_filename(ob._filename, "_split1"));
                 newob1->_video.attrib().audio.clear(); // TODO
                 add_object(std::move(newob1));
                 unique_ptr<Object> newob2 = make_unique<Object>(ob, std::move(nvideo2), std::move(nvideo2_nv12),
                                                                 append_to_filename(ob._filename, "_split2"));
                 newob2->_video.attrib().audio.clear(); // TODO
                 add_object(std::move(newob2));
                 set_video_frame(g_cob, 0.); // select the first frame (of the second video)
                 g_obs.erase(g_cob-2, 1);    // unload the old video
                 g_cob--;
                 message("Here is the second part of the split video; use <&> to undo.", 6.);
             }
             bcase '&': {       // create new video by merging (concatenating) current video with previous one
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob2 = check_loaded_video();
                 if (g_cob==0) throw string("no previous video object to append to");
                 const Object& ob1 = *g_obs[g_cob-1];
                 if (ob1.is_image()) throw string("previous object is not a video");
                 if (ob1.spatial_dims()!=ob2.spatial_dims())
                     throw string("previous and current video have different spatial dimensions");
                 assertx(!!ob1._video.size()==!!ob2._video.size());
                 Video nvideo;
                 VideoNv12 nvideo_nv12;
                 const Vec3<int> ndims = concat(V(ob1.nframes()+ob2.nframes()), ob1.spatial_dims());
                 if (ob1._video.size()) {
                     nvideo.init(ndims);
                     nvideo.slice(0, ob1.nframes()).assign(ob1._video);
                     nvideo.slice(ob1.nframes(), ndims[0]).assign(ob2._video);
                 } else if (ob1._video_nv12.size()) {
                     nvideo_nv12.init(ndims);
                     nvideo_nv12.get_Y().slice(0, ob1.nframes()).assign(ob1._video_nv12.get_Y());
                     nvideo_nv12.get_Y().slice(ob1.nframes(), ndims[0]).assign(ob2._video_nv12.get_Y());
                     nvideo_nv12.get_UV().slice(0, ob1.nframes()).assign(ob1._video_nv12.get_UV());
                     nvideo_nv12.get_UV().slice(ob1.nframes(), ndims[0]).assign(ob2._video_nv12.get_UV());
                 } else assertnever("");
                 const int new_cur_frame = ob1.nframes()+g_framenum;
                 const Object& ob_attrib = ob1._video.attrib().framerate ? ob1 : ob2;
                 unique_ptr<Object> newob = make_unique<Object>(ob_attrib, std::move(nvideo), std::move(nvideo_nv12),
                                                                append_to_filename(ob1._filename, "_merged"));
                 newob->_video.attrib().audio.clear(); // TODO
                 add_object(std::move(newob));
                 set_video_frame(g_cob, new_cur_frame);
                 g_obs.erase(g_cob-2, 2); // unload both the old videos
                 g_cob -= 2;
                 string stmp = get_path_root(ob2._filename);
                 if (remove_at_end(stmp, "_mirror") && stmp==get_path_root(ob1._filename)) {
                     getob()._filename = append_to_filename(ob1._filename, "_mirrorloop");
                     message("Here is the resulting mirror loop; use '|' to undo.", 6.);
                     if (g_looping==ELooping::mirror) g_looping = ELooping::one;
                 } else {
                     message("Here is the merged video; use <|> to undo.", 6.);
                 }
             }
             bcase 'M': {       // mirror: reverse the frames of a video
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_video();
                 Video nvideo;
                 VideoNv12 nvideo_nv12;
                 if (ob._video.size()) {
                     nvideo.init(ob._dims);
                     parallel_for_int(f, ob.nframes()) { nvideo[f].assign(ob._video[ob.nframes()-1-f]); }
                 } else if (ob._video_nv12.size()) {
                     nvideo_nv12.init(ob._dims);
                     parallel_for_int(f, ob.nframes()) {
                         nvideo_nv12.get_Y()[f].assign(ob._video_nv12.get_Y()[ob.nframes()-1-f]);
                         nvideo_nv12.get_UV()[f].assign(ob._video_nv12.get_UV()[ob.nframes()-1-f]);
                     }
                 } else assertnever("");
                 const int new_cur_frame = g_framenum;
                 unique_ptr<Object> newob = make_unique<Object>(ob, std::move(nvideo), std::move(nvideo_nv12),
                                                                append_to_filename(ob._filename, "_mirror"));
                 newob->_video.attrib().audio.clear(); // TODO
                 add_object(std::move(newob));
                 set_video_frame(g_cob, new_cur_frame);
                 message("Here is the time-mirrored video; use <&> to append to the original video.", 6.);
             }
             bcase 'R': {       // resample temporal rate
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_video();
                 string s;
                 if (!query(V(20, 10), "Resample by temporal factor (e.g. .5 reduces #frames by half): ", s))
                     throw string("");
                 if (!Args::check_double(s)) throw string("temporal factor is not a float");
                 double fac = Args::parse_double(s);
                 if (fac<=0.) throw string("factor must be positive");
                 if (fac==1.) throw string("");
                 const int nnf = int(ob.nframes()*fac+0.5);
                 const int new_cur_frame = int(g_framenum/fac);
                 Vec3<int> ndims = concat(V(nnf), ob.spatial_dims());
                 immediate_message("Temporally rescampling the video...");
                 Video nvideo;
                 VideoNv12 nvideo_nv12;
                 if (ob._video.size()) {
                     nvideo.init(ndims);
                     parallel_for_int(f, nnf) {
                         int of = int(f/fac);    // not +.5f !
                         nvideo[f].assign(ob._video[of]);
                     }
                 } else {
                     nvideo_nv12.init(ndims);
                     parallel_for_int(f, nnf) {
                         int of = int(f/fac);    // not +.5f !
                         nvideo_nv12.get_Y()[f].assign(ob._video_nv12.get_Y()[of]);
                         nvideo_nv12.get_UV()[f].assign(ob._video_nv12.get_UV()[of]);
                     }
                 }
                 unique_ptr<Object> newob = make_unique<Object>(ob, std::move(nvideo), std::move(nvideo_nv12),
                                                                append_to_filename(ob._filename, "_rate"));
                 newob->_video.attrib().audio.clear(); // TODO
                 add_object(std::move(newob));
                 set_video_frame(g_cob, new_cur_frame);
                 message("Here is the time-scaled video.", 6.);
             }
             bcase 'b': {       // brightness controls
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 redraw_later();
                 if (!g_use_sliders) {
                     g_use_sliders = true;
                 } else if (all_of(g_sliders, [](const Slider& slider) { return *slider.pval==1.f; })) {
                     g_use_sliders = false;
                     reset_sliders();
                 } else {
                     const Object& ob = check_loaded_object();
                     g_use_sliders = false;
                     Video nvideo;
                     VideoNv12 nvideo_nv12;
                     immediate_message("Applying brightness/color changes to object...");
                     const float brightness_term = get_brightness_term();
                     const float contrast_fac = get_contrast_fac();
                     const float saturation_fac = get_saturation_fac();
                     if (ob._video.size()) {
                         nvideo.init(ob._video.dims());
                         const bool bgra = ob.is_image() && ob._image_is_bgra;
                         parallel_for_size_t(i, ob._video.size()) {
                             Pixel pix = ob._video.raster(i);
                             if (bgra) std::swap(pix[0], pix[2]);
                             Pixel yuv = RGB_to_YUV_Pixel(pix[0], pix[1], pix[2]);
                             float y = yuv[0]/255.f;
                             y = pow(y, g_gamma);
                             y *= contrast_fac;
                             y += brightness_term;
                             yuv[0] = clamp_to_uchar(int(y*255.f+.5f));
                             for_intL(c, 1, 3) { yuv[c] = clamp_to_uchar(int(128.5f+(yuv[c]-128.f)*saturation_fac)); }
                             pix = YUV_to_RGB_Pixel(yuv[0], yuv[1], yuv[2]);
                             if (bgra) std::swap(pix[0], pix[2]);
                             nvideo.raster(i) = pix;
                         }
                     } else {
                         nvideo_nv12.init(ob._video_nv12.get_Y().dims());
                         parallel_for_size_t(i, ob._video_nv12.get_Y().size()) {
                             float y = ob._video_nv12.get_Y().raster(i)/255.f;
                             y = pow(y, g_gamma);
                             y *= contrast_fac;
                             y += brightness_term;
                             nvideo_nv12.get_Y().raster(i) = clamp_to_uchar(int(y*255.f+.5f));
                         }
                         parallel_for_size_t(i, ob._video_nv12.get_UV().size()) for_int(c, 2) {
                             nvideo_nv12.get_UV().raster(i)[c] =
                                 clamp_to_uchar(int(128.5f+(ob._video_nv12.get_UV().raster(i)[c]-128.f)*
                                                    saturation_fac));
                         }
                     }
                     string s;
                     for (const Slider& slider: g_sliders) {
                         if (*slider.pval!=1.f) s += "_" + slider.name + sform("%g", *slider.pval);
                     }
                     add_object(make_unique<Object>(ob, std::move(nvideo), std::move(nvideo_nv12),
                                                    append_to_filename(ob._filename, s)));
                     reset_sliders();
                     message("Created modified " + ob.stype());
                 }
             }
             bcase 'L': {       // create an unoptimized loop (synchronously)
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_object();
                 if (ob.nframes()<4) throw string("too few video frames");
                 const Vec2<int> hdims = ob.spatial_dims()/2; // in case YUV representation is used
                 g_lp.mat_start.init(hdims, 1);
                 g_lp.mat_period.init(hdims, ob.nframes()-2);
                 g_lp.is_loaded = true;
                 message("Waiting for gradient-domain loop creation");
                 g_request_loop_synchronously = true;
                 initiate_loop_request();
             }
             bcase 'g': {       // generate optimized seamless loop
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_object();
                 if (ob.nframes()<4) throw string("too few video frames");
                 g_request_loop_synchronously = false;
                 initiate_loop_request();
             }
             bcase 'G'-64: {    // generate high-quality optimized seamless loop
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_object();
                 if (ob.nframes()<4) throw string("too few video frames");
                 g_request_loop_synchronously = false;
                 g_high_quality_loop = true;
                 initiate_loop_request();
             }
             bcase 'G': {       // generate optimized seamless loop synchronously
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_object();
                 if (ob.nframes()<4) throw string("too few video frames");
                 message("Waiting for seamless loop creation");
                 g_request_loop_synchronously = true;
                 initiate_loop_request();
             }
             bcase 'c': {       // clone
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_object();
                 Video nvideo(ob._video);
                 VideoNv12 nvideo_nv12(Grid<3,uchar>(ob._video_nv12.get_Y()),
                                       Grid<3, Vec2<uchar>>(ob._video_nv12.get_UV()));
                 add_object(make_unique<Object>(ob, std::move(nvideo), std::move(nvideo_nv12), ob._filename));
                 getob()._unsaved = ob._unsaved;
                 message("This is the cloned " + ob.stype());
             }
             bcase 'I': {       // copy current frame as a new image object
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 const Object& ob = check_loaded_video();
                 if (g_framenum<0) throw string("no current video frame");
                 Image image(ob.spatial_dims());
                 bool bgra = false;
                 if (ob._video_nv12.size()) {
                     convert_Nv12_to_Image_BGRA(ob._video_nv12[g_framenum], image);
                     bgra = true;
                 } else {
                     image = ob._video[g_framenum];
                     if (ob.is_image()) {
                         bgra = ob._image_is_bgra;
                         image.attrib() = ob._image_attrib;
                     }
                 }
                 string filename = append_to_filename(ob._filename, sform("_frame%d", g_framenum+0));
                 filename = get_path_root(filename) + ".png";
                 g_obs.push(make_unique<Object>(std::move(image), filename, bgra));
                 message("Saved current frame as new image");
             }
             bcase 'F': {       // set framerate
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 Object& ob = check_loaded_video();
                 string s = sform("%g", ob._video.attrib().framerate);
                 if (!query(V(20, 10), "Framerate (fps) for video: ", s)) throw string("");
                 if (!Args::check_double(s)) throw string("framerate not a float");
                 ob._video.attrib().framerate = Args::parse_double(s);
                 redraw_later();
             }
             bcase 'B': {       // set bitrate
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 Object& ob = check_loaded_video();
                 double bitrate = double(ob._video.attrib().bitrate);
                 string s = bitrate>=1000000. ? sform("%gm", bitrate/1000000.) : sform("%gk", bitrate/1000.);
                 if (!query(V(20, 10), "Bitrate (bps) for video: ", s)) throw string("");
                 double factor = 1.;
                 if (s!="" && s.back()=='k') {
                     factor = 1000.; s.pop_back();
                 } else if (s!="" && s.back()=='m') {
                     factor = 1000000.; s.pop_back();
                 }
                 if (!Args::check_double(s)) throw string("cannot parse bitrate");
                 bitrate = Args::parse_double(s)*factor;
                 if (bitrate<=0. || abs(bitrate-floor(bitrate+.5))>1e-6)
                     throw string("bitrate is not positive integer");
                 ob._video.attrib().bitrate = int(bitrate+.5);
                 redraw_later();
             }
             bcase 'C'-64: {    // C-c: copy image or frame to clipboard
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 Object& ob = check_object();
                 if (g_framenum<0) throw string("no current video frame");
                 Image image(ob.spatial_dims());
                 bool bgra = false;
                 if (ob._video_nv12.size()) {
                     convert_Nv12_to_Image(ob._video_nv12[g_framenum], image);
                 } else {
                     image = ob._video[g_framenum];
                     if (ob.is_image()) {
                         bgra = ob._image_is_bgra;
                         image.attrib() = ob._image_attrib;
                     }
                 }
                 if (bgra) convert_bgra_rgba(image);
                 if (!copy_image_to_clipboard(image)) throw string("could not copy image/frame to clipboard");
             }
             bcase 'V'-64: {    // C-v: paste clipboard image as new object
                 std::lock_guard<std::mutex> lg(g_mutex_obs);
                 Image image;
                 if (!copy_clipboard_to_image(image)) throw string("could not copy an image from clipboard");
                 const bool bgra = false; const bool unsaved = true;
                 string filename = get_current_directory() + "/v1.png";
                 g_obs.push(make_unique<Object>(std::move(image), filename, bgra, unsaved));
                 set_video_frame(getobnum()-1, k_before_start);
                 reset_window(determine_default_window_dims(g_frame_dims));
             }
             bcase 'i': {       // info
                 g_show_info = !g_show_info;
                 redraw_later();
             }
             bcase 'e': {       // exif
                 g_show_exif = !g_show_exif;
                 redraw_later();
             }
             bcase 'H': {       // checker
                 g_checker = !g_checker;
                 redraw_later();
             }
             bcase 'h': ocase '?': { // help
                 g_show_help = !g_show_help;
                 redraw_later();
             }
             bcase '@': {       // redraw window (for testing) and output some diagnostics
                 if (0) {
                     string s;
                     for_intL(i, 1, 256) s += narrow_cast<char>(i);
                     message("Chars:" + s, 1000.);
                 }
                 if (1) {
                     SHOW(g_win_dims, g_tex_dims, g_tex_active_dims);
                     SHOW(get_max_window_dims());
                     SHOWP(g_view);
                     if (g_cob>=0) {
                         const Object& ob = getob();
                         SHOW(g_cob, ob._filename);
                         SHOW(ob._dims, ob._nframes_loaded, ob._framenum, ob._framein, ob._frameou1);
                         SHOW(ob._unsaved, ob._file_modification_time);
                         if (ob.is_image()) SHOW(ob._image_is_bgra, ob._image_attrib.zsize);
                     }
                 }
                 if (1) g_refresh_texture = true;
                 redraw_later();
             }
             bcase '/': {       // no-op operation, for "-key /"
                 void();
             }
             bcase '\033': {    // exit; <esc> key (== uchar{27}); see also "<esc>"
                 if (0 && getobnum()>1 && prev_skey2!="\033") {
                     message("More than one file is open, press <esc> again to confirm quit", 10.);
                 } else {
                     quit();
                 }
             }
             bdefault:          // unrecognized key
                recognized = false;
            }
        }
    }
    catch (string s) {
        if (s!="") { message("Error: " + s, 8.); beep(); }
    }
    return recognized;
}

bool timeline_shown() {
    return g_show_info && g_cob>=0 && !getob().is_image() && getob().nframes()>1 && g_win_dims[0]>=100;
}

// Select new frame based on user selection on timeline at window location x.
void act_timeline(const Vec2<int>& yx) {
    assertw(timeline_shown());
    const auto& g = g_timeline;
    const float wcur = g.get_wcur(getob().nframes());
    const float xf = float(yx[1])/g_win_dims[1];
    float f = (xf-g.left)/max(g.width-wcur, 1e-6f)*(getob().nframes()-1);
    set_video_frame(g_cob, f);
    hw.redraw_later();
    g_playing = false;
}

void DerivedHW::button_press(int butnum, bool pressed, const Vec2<int>& pyx) {
    Vec2<int> yx = pyx;
    if (g_verbose>=1) SHOW(butnum, pressed, yx);
    if (getobnum()==0) return;
    if (pressed) {
        if (butnum<=3) {
            g_selected.button_active = butnum;
            g_selected.shift_was_pressed = get_key_modifier(HW::EModifier::shift);
            g_selected.control_was_pressed = get_key_modifier(HW::EModifier::control);
            g_selected.yx_pressed = yx;
            g_selected.yx_last = yx;
            g_selected.axis_constraint = -1;
            g_selected.on_timeline = false;
        }
        switch (butnum) {
         bcase 1: {
             if (timeline_shown() && g_timeline.is_on_timeline(convert<float>(yx)/convert<float>(g_win_dims))) {
                 // start drag along timeline
                 g_selected.on_timeline = true;
                 act_timeline(yx);
             } else if (g_use_sliders) {
                 redraw_later();         // start drag for slider
             } else if (g_framenum>=0) { // start drag, showing pixel information under cursor
                 Vec2<int> prev_yxi(twice(-1));
                 for (;;) {
                     auto yxi = get_image_yx(convert<float>(yx)+.5f);
                     if (prev_yxi!=yxi) {
                         string s = sform("XY:(%d, %d) - ", yxi[1], yxi[0]);
                         if (!yxi.in_range(g_frame_dims)) {
                             s += "outside image bounds";
                         } else {
                             Pixel pix = get_frame_pix(yxi);
                             Vec4<int> p = convert<int>(pix);
                             s += sform("RGB:(%d, %d, %d), HTML:(#%02X%02X%02X)", p[0], p[1], p[2], p[0], p[1], p[2]);
                         }
                         if (!is_fullscreen()) {
                             set_window_title(s);
                         } else {
                             immediate_message(s + "     ");
                         }
                         prev_yxi = yxi;
                     }
                     if (suggests_stop() || !get_pointer(yx) || g_selected.button_active!=1) break;
                 }
                 g_selected.button_active = 0;
                 app_set_window_title();
             } else beep();
         }
         bcase 2: {
             redraw_later();    // start drag for zoom or rotate
         }
         bcase 3: {
             redraw_later();    // start drag for pan
         }
         bcase 4: {             // back button
             if (g_cob>0) set_video_frame(g_cob-1, k_before_start);
             else if (g_cob>=0 && g_looping==ELooping::all) set_video_frame(getobnum()-1, k_before_start);
             else beep();
         }
         bcase 5: {             // forward button
             if (g_cob<getobnum()-1) set_video_frame(g_cob+1, k_before_start);
             else if (g_cob>=0 && g_looping==ELooping::all) set_video_frame(0, k_before_start);
             else beep();
         }
         bdefault:
            beep();
        }
    } else {
        g_selected.button_active = 0;
    }
}

void DerivedHW::wheel_turn(float v) {
    Vec2<int> yx;
    if (get_pointer(yx)) {
        float fac_zoom = pow(k_wheel_zoom_fac, v);
        perform_zoom_at_cursor(fac_zoom, yx);
        const Vec2<float> arzoom = get_zooms();
        if (arzoom[0]==arzoom[1]) { // adjust zoom factor to be integer multiple/divisor if sufficiently close
            const float zoom = arzoom[0];
            float new_zoom = 0.f;
            const float eps = .01f;
            if (zoom>.9f && abs(zoom-floor(zoom+.5f))<eps) new_zoom = floor(zoom+.5f);
            else if (zoom<.9f && abs(1.f/zoom-floor(1.f/zoom+.5f))<eps) new_zoom = 1.f/floor(1.f/zoom+.5f);
            if (new_zoom) perform_zoom_at_cursor(new_zoom/zoom, yx);
        }
        redraw_later();
    }
}

void advance_frame() {
    double time_since_last_frame, steady_time_since_last_frame; { // steady is set to negative if unreliable
        static double last_frame_time = 0.;
        static Vec<double,8> ar_last_frame_times;
        double vtime = get_precise_time();
        if (!last_frame_time) { last_frame_time = vtime; fill(ar_last_frame_times, 10000.); }
        time_since_last_frame = vtime-last_frame_time;
        rotate(ar_last_frame_times, ar_last_frame_times.last()); // shift towards rear
        ar_last_frame_times[0] = time_since_last_frame;
        steady_time_since_last_frame = (sqrt(var(ar_last_frame_times))>.008 ? -1 :
                                        1 ? median(ar_last_frame_times) : mean(ar_last_frame_times));
        assertw(time_since_last_frame>=0.);
        time_since_last_frame = clamp(time_since_last_frame, 0., 3./60.);
        last_frame_time = vtime;
    }
    if (g_cob<0) return;
    if (!g_playing && getob()._nframes_loaded) return;
    if (getob().is_image()) return;
    const auto& ob = getob();
    assertx(ob.size());
    const double video_framerate = ob._video.attrib().framerate ? ob._video.attrib().framerate : 30.;
    if (g_frametime<0. || !ob._nframes_loaded) {
        g_frametime = 0.;
        while (!ob._nframes_loaded)
            my_sleep(0.);
    } else {
        int old_framenum = g_framenum;
        int direction = g_looping==ELooping::mirror && !g_mirror_state_forward ? -1 : +1;
        if (0) {
            g_frametime += direction*1.*g_speed; // always 60 fps; ignore video_framerate and g_speed
        } else if (steady_time_since_last_frame>=0. &&
                   steady_time_since_last_frame<1./40. &&
                   steady_time_since_last_frame*video_framerate*g_speed>=.92) {
            g_frametime += direction*steady_time_since_last_frame*video_framerate*g_speed;
        } else {
            // For slow playback (less than monitor refresh), sleep until it is time to show a new frame.
            for (;;) {
                double time_change; {
                    static double prev_time = 0.;
                    double vtime = get_precise_time();
                    if (!prev_time) prev_time = vtime;
                    time_change = vtime-prev_time;
                    assertw(time_change>=0.);
                    // time_change = clamp(time_change, .001, .5+1./30.);
                    const double max_time_change = 1./20.; // was .5+1./30.;
                    time_change = clamp(time_change, 0., max_time_change);
                    prev_time = vtime;
                }
                g_frametime += direction*time_change*video_framerate*g_speed;
                if (int(floor(g_frametime))!=old_framenum) break;
                if (g_messages.num()) break;
                if (0) my_sleep(.002); // could be more accurate
                my_sleep(0.);          // just give up time slice
            }
        }
        bool centered = false;
        {
            // I notice a slight drift: the frame sync of the GPU/monitor is slightly slower than 60 fps.
            // Therefore if I advance based on real time, I occasionally have to skip forward a frame.
            // To counter this, I create a slight bias to moving g_frametime towards the center of the
            //  current video frame interval, if we are advancing frames at the same rate as monitor refresh.
            if (1 && abs(steady_time_since_last_frame-1./(video_framerate*g_speed))<.005) {
                centered = true;
                const double adjustment_rate = .01; // 1% seems sufficient
                double adjustment = -signz(g_frametime-floor(g_frametime)-.5)*adjustment_rate;
                g_frametime += adjustment;
            }
        }
        const int framediff = my_mod(direction*(int(floor(g_frametime))-old_framenum), ob.nframes());
        if (g_verbose>=2 || (0 && abs(framediff)!=1 && g_speed==1.))
            showf("frametime=%-11.5f framediff=%-2d dtime=%-9.5f steady=%-9.5f"
                  " nloaded=%-3d  centered=%d\n",
                  g_frametime, framediff, time_since_last_frame, steady_time_since_last_frame,
                  ob._nframes_loaded+0, centered);
        if (int(floor(g_frametime))>=ob._nframes_loaded && ob._nframes_loaded<ob.nframes()) {
            if (g_verbose>=2) SHOW("wait", g_frametime, ob._nframes_loaded, ob.nframes());
            // We have advanced past the number of loaded frames, so wait until at least some forward progress.
            while (old_framenum==ob._nframes_loaded-1) {
                if (0) my_sleep(.005);
                my_sleep(0.);
            }
            assertx(old_framenum<ob._nframes_loaded-1);
            g_frametime = ob._nframes_loaded-1.; // reset the desired time
        } else if (g_looping==ELooping::mirror) {
            if (g_frametime>=ob._frameou1) {
                g_mirror_state_forward = false;
                g_frametime = ob._frameou1-1-1e-6f;
            } else if (g_frametime<ob._framein) {
                g_mirror_state_forward = true;
                g_frametime = ob._framein;
            }
        } else if (g_frametime>=ob._frameou1) {
            if (g_looping==ELooping::one) {
                g_frametime -= (ob._frameou1-ob._framein); // restart current video
            } else if (g_looping==ELooping::all) {
                int obi = g_cob>=getobnum()-1 ? 0 : g_cob+1;
                set_video_frame(obi, getob(obi)._framein ? getob(obi)._framein-.001 : k_before_start);
            } else {
                g_frametime = ob._frameou1-1; // freeze on last frame
                g_playing = false;
            }
        }
    }
}

static bool supports_non_power_of_two_textures;
static bool supports_pbuffer;
static bool supports_BGRA;
static bool supports_texture_edge_clamp;
static bool supports_filter_anisotropic;

static void upload_sub_texture(int level, const Vec2<int>& offset, const Vec2<int>& dims,
                               std::function<GLenum(MatrixView<Pixel>)> func_copy) {
    if (!supports_pbuffer) {
        Matrix<Pixel> frame(dims);
        GLenum frame_format = func_copy(frame);
        glTexSubImage2D(GL_TEXTURE_2D, level, offset[1], offset[0], dims[1], dims[0],
                        frame_format, GL_UNSIGNED_BYTE, assertx(frame.data()));
    } else {
        // Using pixel buffer objects; see http://www.mathematik.uni-dortmund.de/~goeddeke/gpgpu/tutorial3.html
        // Also https://www.opengl.org/registry/specs/ARB/pixel_buffer_object.txt
        //  and http://www.nvidia.com/object/fast_texture_transfers.html
        USE_GL_EXT(glGenBuffers, PFNGLGENBUFFERSPROC);
        USE_GL_EXT(glBindBuffer, PFNGLBINDBUFFERPROC);
        USE_GL_EXT(glBufferData, PFNGLBUFFERDATAPROC);
        USE_GL_EXT(glMapBuffer, PFNGLMAPBUFFERPROC);
        USE_GL_EXT(glUnmapBuffer, PFNGLUNMAPBUFFERPROC);
        const int nbuf = 1;
        static unsigned io_buf[nbuf];
        static bool is_init2 = false; if (!is_init2) {
            is_init2 = true;
            glGenBuffers(nbuf, io_buf);
        }
        glEnable(GL_TEXTURE_2D); // to be safe on AMD; see https://www.opengl.org/wiki/Common_Mistakes
        {
            // Required for AMD; see https://www.opengl.org/wiki/Common_Mistakes
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            // else texture is incomplete and get error below on glTexSubImage2D()
        }
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, io_buf[0]);
        // GL_STATIC_DRAW (uploaded once); GL_DYNAMIC_DRAW (used several times); GL_STREAM_DRAW (used once)
        glBufferData(GL_PIXEL_UNPACK_BUFFER, narrow_cast<size_t>(product(dims)*sizeof(Pixel)),
                     nullptr, GL_STREAM_DRAW);
        void* ioMem = assertx(glMapBuffer(GL_PIXEL_UNPACK_BUFFER, GL_WRITE_ONLY));
        MatrixView<Pixel> frame(reinterpret_cast<Pixel*>(ioMem), dims); // texture buffer
        GLenum frame_format = func_copy(frame);
        assertx(glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER));
        assertx(!gl_report_errors());
        if (0) SHOW(level, offset, dims, frame_format);
        glTexSubImage2D(GL_TEXTURE_2D, level, offset[1], offset[0], dims[1], dims[0],
                        frame_format, GL_UNSIGNED_BYTE, implicit_cast<char*>(nullptr));
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    }
}

void upload_image_to_texture() {
    HH_CTIMER(_upload_image, g_verbose>=1);
    assertx(product(g_frame_dims));
    assertx(g_cob>=0 && g_framenum>=0);
    static int max_texture_size;
    static bool is_init1 = false; if (!is_init1) {
        is_init1 = true;
        if (g_verbose>=2) SHOW(gl_extensions_string());
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture_size);
        // USE_GL_EXT_MAYBE(glMapBuffer, PFNGLMAPBUFFERPROC);
        // supports_pbuffer = !!glMapBuffer; // returns address on cygwin yet is not implemented
        supports_non_power_of_two_textures = contains(gl_extensions_string(), "GL_ARB_texture_non_power_of_two");
        supports_pbuffer = contains(gl_extensions_string(), "GL_ARB_pixel_buffer_object");
        supports_BGRA = contains(gl_extensions_string(), "GL_EXT_bgra");
        supports_texture_edge_clamp = contains(gl_extensions_string(), "GL_EXT_texture_edge_clamp");
        supports_filter_anisotropic = contains(gl_extensions_string(), "GL_EXT_texture_filter_anisotropic");
        if (0) {                // for testing
            supports_non_power_of_two_textures = false; supports_pbuffer = false; supports_BGRA  = false;
            supports_texture_edge_clamp = false; supports_filter_anisotropic = false;
        }
        if (g_verbose>=1) SHOW(max_texture_size, supports_non_power_of_two_textures, supports_pbuffer, supports_BGRA,
                               supports_texture_edge_clamp, supports_filter_anisotropic);
        // On Windows Remote Desktop: non_power_of_two_textures=0 pbuffer=0 BGRA=1 texture_edge_clamp=0 anisotropic=0
        if (!supports_texture_edge_clamp) g_background_padding_width = k_usual_tex_padding_width;
        unsigned texname0; glGenTextures(1, &texname0);
        glBindTexture(GL_TEXTURE_2D, texname0);
    }
    g_tex_active_dims = g_frame_dims;
    Vec2<int> desired_dims = g_frame_dims+g_background_padding_width*2;
    if (!supports_non_power_of_two_textures) for_int(c, 2) { while (!is_pow2(desired_dims[c])) desired_dims[c]++; }
    bool tex_size_exceeded = max(desired_dims)>max_texture_size;
    if (tex_size_exceeded) {
        if (g_verbose) SHOW(g_frame_dims, desired_dims, max_texture_size);
        message(sform("Size exceeds supported texture dimension %d", max_texture_size), 5.);
        const int k_fallback_size = 512;
        assertx(k_fallback_size<=max_texture_size);
        desired_dims = twice(k_fallback_size);
        g_tex_active_dims = desired_dims-g_background_padding_width*2;
    }
    const int level = 0;
    const GLenum format = supports_BGRA ? GL_BGRA : GL_RGBA;
    static Vec2<int> prev_tex_active_dims;
    bool best_boundaries = false; // slower image perusal but precise filtering of right and bottom boundaries
    if (max(g_tex_dims)<=2048) best_boundaries = true; // if we haven't seen any big images so far
    if (!getob().is_image()) best_boundaries = true; // tighten boundaries for better video playback performance
    if (best_boundaries && desired_dims!=g_tex_dims) g_tex_dims = twice(0); // force use of tight dimensions
    if (max(desired_dims, g_tex_dims)!=g_tex_dims) {
        if (g_verbose>=1) SHOW(g_frame_dims, desired_dims, g_tex_dims, max(desired_dims, g_tex_dims));
        g_tex_dims = max(desired_dims, g_tex_dims);
        // glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // default 4 is good
        const bool fill_all = false;
        Pixel color = k_background_color; if (supports_BGRA) color = color.to_BGRA();
        {
            const GLenum internal_format = GL_RGBA8;
            const int border = 0;
            if (g_verbose>=1) SHOW("glTexImage2D", g_tex_dims, border, format);
            glTexImage2D(GL_TEXTURE_2D, level, internal_format, g_tex_dims[1], g_tex_dims[0], border,
                         format, GL_UNSIGNED_BYTE, fill_all ? Matrix<Pixel>(g_tex_dims, color).data() : nullptr);
            assertx(!gl_report_errors());
        }
        prev_tex_active_dims = g_tex_active_dims;
        if (!fill_all) {        // Fill first two of the four gutters.
            if (g_background_padding_width) {
                for_int(c, 2) {
                    Vec2<int> offset = twice(0);
                    Vec2<int> dims = g_tex_dims.with(c, g_background_padding_width);
                    upload_sub_texture(level, offset, dims, [&](MatrixView<Pixel> frame) {
                        fill(frame, color); return format;
                    });
                }
            }
            prev_tex_active_dims = twice(-1); // force update of remaining two gutters
        }
    }
    if (g_tex_active_dims!=prev_tex_active_dims) {
        prev_tex_active_dims = g_tex_active_dims;
        // Note that two of the four gutters (nearest origin) are either always empty (zero padding) or
        //  have constant black color.  Update the two remaining gutters.
        for_int(c, 2) {
            if (g_tex_active_dims[c]==g_tex_dims[c]) continue; // tight fit, no need for gutter
            Vec2<int> offset = twice(0).with(c, g_background_padding_width+g_tex_active_dims[c]);
            int w = min(g_tex_dims[c]-offset[c], k_usual_tex_padding_width); assertx(w>0);
            Vec2<int> dims = (g_tex_active_dims+twice(g_background_padding_width*2)).with(c, w);
            assertx(max(dims, g_tex_dims)==g_tex_dims);
            Pixel color = k_background_color; if (supports_BGRA) color = color.to_BGRA();
            if (g_verbose>=1) SHOW("padding", c, offset, dims);
            upload_sub_texture(level, offset, dims, [&](MatrixView<Pixel> frame) {
                fill(frame, color); return format;
            });
        }
    }
    if (tex_size_exceeded) {    // fill the texture interior with pink color (denoting undefined content)
        Vec2<int> offset = twice(g_background_padding_width);
        Pixel color = Pixel::pink(); if (supports_BGRA) color = color.to_BGRA();
        const Vec2<int> dims = g_tex_active_dims;
        if (g_verbose) SHOW("pink", offset, dims, g_tex_dims);
        upload_sub_texture(level, offset, dims, [&](MatrixView<Pixel> frame) {
            fill(frame, color); return format;
        });
    } else {                    // upload the texture data
        Vec2<int> offset = twice(g_background_padding_width);
        upload_sub_texture(level, offset, g_frame_dims, [&](MatrixView<Pixel> frame) {
            GLenum frame_format = format;
            if (getob()._video_nv12.size()) {
                if (supports_BGRA) convert_Nv12_to_Image_BGRA(getob()._video_nv12[g_framenum], frame);
                else               convert_Nv12_to_Image     (getob()._video_nv12[g_framenum], frame);
            } else {
                // frame.assign(getob()._video[g_framenum]);
                CMatrixView<Pixel> vframe(getob()._video[g_framenum]);
                std::copy(vframe.begin(), vframe.end(), frame.data());
                if (!getob().is_image() || !getob()._image_is_bgra) {
                    frame_format = GL_RGBA;
                } else if (!supports_BGRA) {
                    convert_bgra_rgba(frame);
                    frame_format = GL_RGBA;
                }
            }
            return frame_format;
        });
    }
    g_generated_mipmap = false;
    {
        // GL_DECAL is the same as GL_REPLACE except it does the "right thing" on texture with alphas.
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    }
    if (supports_filter_anisotropic) {
        const float anisotropy = 8.f;
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, V(anisotropy).data());
    }
    assertx(!gl_report_errors());
}

// cygwin: OpenGL GLSL 3.00 is not supported. Supported versions are: 1.10, 1.20, 1.30, 1.00 ES, and 3.00 ES
// https://github.com/mattdesl/lwjgl-basics/wiki/GLSL-Versions
// https://en.wikipedia.org/wiki/OpenGL_Shading_Language
// http://www.opengl.org/registry/doc/GLSLangSpec.4.50.pdf
// https://www.opengl.org/wiki/Sampler_%28GLSL%29
// http://emblemparade.net/blog/which-versions-of-opengl-es-should-you-target/
//  "OpenGL ES GLSL 300 es" is mostly equivalent to "OpenGL GLSL 330".
//  "This is great, because you'll be able to share shader code between them."

#if defined(__CYGWIN__)
static const string glsl_shader_version = "#version 300 es\n"; // works everywhere
// static const string glsl_shader_version = "#version 130\n"; // last non-es version supported by cygwin; also works
#elif defined(__APPLE__)
// "GLX/X11 is limited to OpenGL 2.1 on OSX" (legacy context)
// "Apparently X11 doesn't support OpenGL higher than 2.1 on OS X.  As such I suggest you switch to GLFW."
// http://www.geeks3d.com/20121109/overview-of-opengl-support-on-os-x/
static const string glsl_shader_version = "#version 120\n"
    "#error Mac OSX XQuartz only supports OpenGL 2.1, which is insufficient for this program (VideoViewer).\n";
// For Mac OS X 10.5 - XQuartz 2.7.8: "2.1 INTEL-10.6.33"
// Note: work around this by subsequently setting "use_modern_opengl = false".
#else
static const string glsl_shader_version = "#version 330\n"; // not supported on cygwin
#endif

static const string vertex_shader = glsl_shader_version + (
#include "vertex_shader.glsl"
    );


static const string fragment_shader = glsl_shader_version + (
#include "fragment_shader.glsl"
    );


void render_image() {
    // HH_TIMER(_render_image);
    glEnable(GL_TEXTURE_2D);    // may need to come before glGenerateMipmap on old AMD drivers
    if (1) {
        USE_GL_EXT_MAYBE(glGenerateMipmap, PFNGLGENERATEMIPMAPPROC);
        const float min_zoom = min(get_zooms());
        const bool mipmap_enabled = true;
        if (mipmap_enabled && min_zoom<1.f && glGenerateMipmap && !g_generated_mipmap) {
            // Ideally, for highest quality filtering modes, I should manually construct the coarser mipmap levels.
            // test on:  VideoViewer ~/data/video/M4Kseacrowd.mp4 -key -
            glGenerateMipmap(GL_TEXTURE_2D); // not supported on Windows Remote Desktop
            g_generated_mipmap = true;
            if (0) SHOW("generated mimap");
        }
    }
    {
        // Settings for !use_modern_opengl
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                        (g_render_kernel==EKernel::nearest ? GL_NEAREST :
                         g_generated_mipmap ? GL_LINEAR_MIPMAP_LINEAR :
                         GL_LINEAR));
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                        g_render_kernel==EKernel::nearest ? GL_NEAREST : GL_LINEAR);
    }
    {
        unsigned wrap_mode = (!supports_texture_edge_clamp ? GL_CLAMP : // Windows 7 Remote Desktop
                              GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap_mode);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap_mode);
    }
    const auto yxi = V(V(0, 0), V(1, 0), V(1, 1), V(0, 1));
#if 1 && defined(__CYGWIN__)
    const bool use_modern_opengl = false;
    // Otherwise I get a segmentation fault in glxSwapBuffers(); I don't know why.
    // make CONFIG=cygwin -C ~/src -j8 VideoViewer && ~/src/bin/cygwin/VideoViewer -hwdebug 1 ~/data/image/lake.png
#else
    // "//third_party/GL" is currently "2.1 Mesa 10.1.1", which only supports GLSL 1.10 and 1.20.
    // Mac OS is currently "2.1" which is insufficient.
    const bool use_modern_opengl = 1 && assertx(glGetString(GL_VERSION))[0]>='3';
#endif
    if (use_modern_opengl) {
        // Note: this is not portable across Windows Remote Desktop under Windows 7 (which has GL_VERSION 1.1).
        // See [Gortler book] page 47, page 252.
        // See http://www.3dgraphicsfoundations.com/code.html
        // See http://www.cg.tuwien.ac.at/courses/CG23/slides/tutorials/CG2LU_OpenGL_3.x_Introduction_pt_1.pdf
        // See http://www.cg.tuwien.ac.at/courses/CG23/slides/tutorials/CG2LU_OpenGL_3.x_Textures_and_Objects_pt_2.pdf
        USE_GL_EXT(glCreateShader, PFNGLCREATESHADERPROC);
        USE_GL_EXT(glShaderSource, PFNGLSHADERSOURCEPROC);
        USE_GL_EXT(glCompileShader, PFNGLCOMPILESHADERPROC);
        USE_GL_EXT(glGetShaderiv, PFNGLGETSHADERIVPROC);
        USE_GL_EXT(glGetShaderInfoLog, PFNGLGETSHADERINFOLOGPROC);
        USE_GL_EXT(glCreateProgram, PFNGLCREATEPROGRAMPROC);
        USE_GL_EXT(glAttachShader, PFNGLATTACHSHADERPROC);
        USE_GL_EXT(glBindFragDataLocation, PFNGLBINDFRAGDATALOCATIONPROC);
        USE_GL_EXT(glLinkProgram, PFNGLLINKPROGRAMPROC);
        USE_GL_EXT(glGetProgramiv, PFNGLGETPROGRAMIVPROC);
        USE_GL_EXT(glGetProgramInfoLog, PFNGLGETPROGRAMINFOLOGPROC);
        USE_GL_EXT(glValidateProgram, PFNGLVALIDATEPROGRAMPROC);
        USE_GL_EXT(glIsProgram, PFNGLISPROGRAMPROC);
        USE_GL_EXT(glGenBuffers, PFNGLGENBUFFERSPROC);
        USE_GL_EXT(glGetAttribLocation, PFNGLGETATTRIBLOCATIONPROC);
        USE_GL_EXT(glBindBuffer, PFNGLBINDBUFFERPROC);
        USE_GL_EXT(glBufferData, PFNGLBUFFERDATAPROC);
        USE_GL_EXT(glVertexAttribPointer, PFNGLVERTEXATTRIBPOINTERPROC);
        USE_GL_EXT(glEnableVertexAttribArray, PFNGLENABLEVERTEXATTRIBARRAYPROC);
        USE_GL_EXT(glDisableVertexAttribArray, PFNGLDISABLEVERTEXATTRIBARRAYPROC);
        USE_GL_EXT(glUseProgram, PFNGLUSEPROGRAMPROC);
        USE_GL_EXT(glGetUniformLocation, PFNGLGETUNIFORMLOCATIONPROC);
        USE_GL_EXT(glUniformMatrix4fv, PFNGLUNIFORMMATRIX4FVPROC);
        USE_GL_EXT(glUniform4fv, PFNGLUNIFORM4FVPROC);
        USE_GL_EXT(glUniform1i, PFNGLUNIFORM1IPROC);
        USE_GL_EXT(glUniform1f, PFNGLUNIFORM1FPROC);
        USE_GL_EXT(glUniform2fv, PFNGLUNIFORM2FVPROC);
        static GLuint vertex_shader_id;
        static GLuint fragment_shader_id;
        static GLuint program_id;
        static int h_vertex;
        static int h_tex;
        static int h_kernel_id;
        static int h_brightness_term;
        static int h_contrast_fac;
        static int h_gamma;
        static int h_saturation_fac;
        static int h_checker_offset;
        static int h_through_color;
        static GLuint buf_p;
        static GLuint buf_i;
        static bool is_init = false;
        if (!is_init) {
            is_init = true;
            {
                auto func_get_line = [](string slines, int line) -> string {
                    string::size_type i = 0;
                    for_int(count, line-1) { i = slines.find('\n', i); if (i==string::npos) return ""; i++; }
                    string::size_type j = slines.find('\n', i);
                    return slines.substr(i, j-i+1);
                };
                auto func_err = [&](string shadertype, string shader, string serr) {
                    showf("OpenGL %s compilation error: %s", shadertype.c_str(), serr.data());
                    int line;
                    if (sscanf(serr.c_str(), "0(%d)", &line)==1) {
                        string sline = func_get_line(shader, line);
                        if (sline!="") showf("%s", sline.c_str());
                    }
                    _exit(1);
                };
                auto func_compile_shader = [&](GLuint shader_id, string shader_string, GLenum shaderType) {
                    glShaderSource(shader_id, 1,
                                   // const_cast from "const char*const*" necessary on cygwin32 gcc4.7
                                   const_cast<const char**>(ArView(shader_string.c_str()).data()),
                                   ArView(GLint(shader_string.size())).data());
                    glCompileShader(shader_id);
                    GLint params; glGetShaderiv(shader_id, GL_COMPILE_STATUS, &params);
                    if (params!=GL_TRUE) {
                        GLint slen; glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &slen);
                        Array<char> str(slen, '\0');
                        glGetShaderInfoLog(shader_id, str.num(), nullptr, str.data());
                        string shader_type = shaderType==GL_VERTEX_SHADER ? "vertex-shader" : "fragment-shader";
                        func_err(shader_type, shader_string, str.data());
                    }
                };
                vertex_shader_id = assertx(glCreateShader(GL_VERTEX_SHADER));
                func_compile_shader(vertex_shader_id, vertex_shader, GL_VERTEX_SHADER);
                //
                fragment_shader_id = assertx(glCreateShader(GL_FRAGMENT_SHADER));
                func_compile_shader(fragment_shader_id, fragment_shader, GL_FRAGMENT_SHADER);
                //
                program_id = assertx(glCreateProgram());
                glAttachShader(program_id, vertex_shader_id);
                glAttachShader(program_id, fragment_shader_id);
                glBindFragDataLocation(program_id, 0, "frag_color");
                glLinkProgram(program_id); {
                    GLint params; glGetProgramiv(program_id, GL_LINK_STATUS, &params);
                    if (params!=GL_TRUE) {
                        GLint slen; glGetProgramiv(program_id, GL_INFO_LOG_LENGTH, &slen);
                        Array<char> str(slen, '\0');
                        glGetProgramInfoLog(program_id, str.num(), nullptr, str.data());
                        showf("OpenGL program linking error: %s", str.data()); _exit(1);
                    }
                }
                if (1) {
                    if (g_verbose>=1) SHOW("about to validate shader program");
                    glValidateProgram(program_id);
                    GLint params; glGetProgramiv(program_id, GL_VALIDATE_STATUS, &params); assertx(params==GL_TRUE);
                    if (g_verbose>=1) SHOW("validated");
                }
                assertx(glIsProgram(program_id));
            }
            h_vertex = glGetAttribLocation(program_id, "vertex"); assertx(h_vertex>=0);
            h_tex = glGetUniformLocation(program_id, "tex"); assertx(h_tex>=0);
            h_kernel_id = glGetUniformLocation(program_id, "kernel_id"); assertx(h_kernel_id>=0);
            h_brightness_term = glGetUniformLocation(program_id, "brightness_term"); assertx(h_brightness_term>=0);
            h_contrast_fac = glGetUniformLocation(program_id, "contrast_fac"); assertx(h_contrast_fac>=0);
            h_gamma = glGetUniformLocation(program_id, "gamma"); assertx(h_gamma>=0);
            h_saturation_fac = glGetUniformLocation(program_id, "saturation_fac"); assertx(h_saturation_fac>=0);
            h_checker_offset = glGetUniformLocation(program_id, "checker_offset"); assertx(h_checker_offset>=0);
            h_through_color = glGetUniformLocation(program_id, "through_color"); assertx(h_through_color>=0);
            glGenBuffers(1, &buf_p);
            glGenBuffers(1, &buf_i);
            assertx(!gl_report_errors());
        }
        {
            glUseProgram(program_id); assertx(!gl_report_errors());
            const int texture_unit = 0; // glActiveTexture(GL_TEXTURE0+texture_unit); glBindTexture(GL_TEXTURE_2D, ..);
            glUniform1i(h_tex, texture_unit);
            glUniform1i(h_kernel_id, int(g_render_kernel));
            glUniform1f(h_brightness_term, get_brightness_term());
            glUniform1f(h_contrast_fac, get_contrast_fac());
            glUniform1f(h_gamma, g_gamma);
            glUniform1f(h_saturation_fac, get_saturation_fac());
            {
                static int s_frame_num = 0;
                s_frame_num++;
                const float motion_radius = 5.f;
                const double angular_velocity = .1;
                const float ang = float(my_mod(double(s_frame_num)*angular_velocity, D_TAU));
                glUniform2fv(h_checker_offset, 1, (motion_radius*V(cos(ang), sin(ang))).data());
                glUniform4fv(h_through_color, 1, (g_checker ? V(-1.f, 0.f, 0.f, 0.f).data() :
                                                  Vector4(g_through_color).data()));
            }
            if (0) {
                int h_somematrix = glGetUniformLocation(program_id, "somematrix");
                glUniformMatrix4fv(h_somematrix, 1, GL_FALSE, SGrid<float, 4, 4>{}.data());
            }
            Array<Vec4<float>> ar_vertex; // (xy, uv)
            for_int(i, 4) {
                Point pyx = Point(concat(convert<float>(yxi[i]*g_frame_dims), V(0.f))) * g_view;
                pyx = pyx/concat(convert<float>(g_win_dims), V(1.f))*2.f-1.f; // without glOrtho(), range is [-1, +1]
                pyx[0] = -pyx[0];                                             // flip Y
                Vec2<float> uv = (convert<float>(g_background_padding_width+yxi[i]*g_tex_active_dims)/
                                  convert<float>(g_tex_dims));
                ar_vertex.push(concat(pyx.head<2>().rev(), uv.rev()));
            }
            if (0) ar_vertex = V(V(-1.f, -1.f, 0.f, 0.f), V(+1.f, -1.f, 1.f, 0.f),
                                 V(+1.f, +1.f, 1.f, 1.f), V(-1.f, +1.f, 0.f, 1.f));
            if (0) SHOW(ar_vertex);
            if (1) {
                glBindBuffer(GL_ARRAY_BUFFER, buf_p);
                glBufferData(GL_ARRAY_BUFFER, ar_vertex.num()*sizeof(ar_vertex[0]), ar_vertex.data()->data(),
                             GL_DYNAMIC_DRAW);
                glVertexAttribPointer(h_vertex, ar_vertex[0].num(), GL_FLOAT, GL_FALSE, 0, nullptr);
                glEnableVertexAttribArray(h_vertex);
                if (1) {
                    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
                } else {
                    glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_INT, V(0, 1, 2, 3).data());
                }
                glDisableVertexAttribArray(h_vertex);
                glBindBuffer(GL_ARRAY_BUFFER, 0);
            } else if (0) {
                glEnableVertexAttribArray(h_vertex);
                glVertexAttribPointer(h_vertex, ar_vertex[0].num(), GL_FLOAT, GL_FALSE, 0, ar_vertex.data());
                glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
                glDisableVertexAttribArray(h_vertex);
            } else if (0) {
                glEnableVertexAttribArray(h_vertex);
                glVertexAttribPointer(h_vertex, ar_vertex[0].num(), GL_FLOAT, GL_FALSE, 0, ar_vertex.data());
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buf_i);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, 4*sizeof(int), V(0, 1, 2, 3).data(), GL_DYNAMIC_DRAW);
                glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_INT, nullptr);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
                glDisableVertexAttribArray(h_vertex);
            }
            glUseProgram(0);
        }
    } else {
        {
            hw.set_color(g_through_color);
            glBegin(GL_QUADS);
            for_int(i, 4) {
                Vec2<float> uv = (convert<float>(g_background_padding_width+yxi[i]*g_tex_active_dims)/
                                  convert<float>(g_tex_dims));
                glTexCoord2fv(uv.rev().data());
                Point pyx = Point(concat(convert<float>(yxi[i]*g_frame_dims), V(0.f))) * g_view;
                glVertex2fv(pyx.head<2>().rev().data());
                if (0) SHOW(uv, pyx);
            }
            glEnd();
        }
    }
    if (1) {
        glDisable(GL_TEXTURE_2D);
        if (g_show_info) {      // show each boundary of frame unless it is outside window
            hw.set_color_to_foreground();
            if (1) {
                glBegin(GL_LINES);
                SGrid<Point, 2, 2> gridp;
                for (const auto& u : range(gridp.dims())) {
                    gridp[u] = Point(concat(convert<float>(u*g_frame_dims), V(0.f))) * g_view;
                    gridp[u].head<2>() += convert<float>(u*2-1)*1.f; // move points outwards by 1 window pixel
                }
                for_int(c, 2) {
                    for_int(idir, 2) {
                        Vec2<int> u0 = twice(idir).with(1-c, 0);
                        Vec2<int> u1 = twice(idir).with(1-c, 1);
                        if (!view_has_rotation() &&
                            ((idir==0 && gridp[u0][c]<0.f) ||
                             (idir==1 && gridp[u0][c]>g_win_dims[c]-1.f))) continue;
                        glVertex2fv(gridp[u0].head<2>().rev().data());
                        glVertex2fv(gridp[u1].head<2>().rev().data());
                    }
                }
                glEnd();
            } else {
                glBegin(GL_LINE_LOOP);
                for_int(i, 4) {
                    Point pyx = Point(concat(convert<float>(yxi[i]*g_frame_dims), V(0.f))) * g_view;
                    pyx.head<2>() += convert<float>(yxi[i]*2-1)*1.f; // move lines outwards by 1 window pixel
                    glVertex2fv(pyx.head<2>().rev().data());
                }
                glEnd();
            }
        }
    }
}

void DerivedHW::draw_window(const Vec2<int>& dims) {
    // HH_TIMER(draw_window);
    static bool first = true;
    if (first) {
        first = false;
        if (is_fullscreen()) std::swap(g_show_info, g_other_show_info);
    }
    g_win_dims = dims;
    g_desired_dims = twice(-1);         // reset
    if (!assertw(!gl_report_errors())) {
        if (0) { redraw_later(); return; } // failed attempt to refresh window after <enter>fullscreen on Mac
    }
#if !defined(HH_HAVE_BACKGROUND_THREAD)
    background_work(false);
#endif
    if (1) {               // adjust the view if the window moved due to being resized, to facilitate cropping
        Vec2<int> win_pos = window_position_yx();
        if (g_verbose>=2) SHOW("draw", g_win_dims, win_pos, g_cob, getobnum(), product(g_frame_dims),
                               g_framenum, g_refresh_texture);
        if (is_zero(g_prev_win_dims)) {
            if (1) {
                // Make sure the entire window is visible.
                g_prev_win_pos = win_pos;
                g_prev_win_dims = g_win_dims;
                resize_window(dims);
                redraw_later();
                return;
            }
        } else if (g_win_dims!=g_prev_win_dims && !g_fit_view_to_window) {
            if (0) SHOW(g_prev_win_pos, win_pos, g_prev_win_pos-win_pos);
            set_view(g_view * Frame::translation(concat(convert<float>(g_prev_win_pos-win_pos), V(0.f))));
        }
        g_prev_win_pos = win_pos;
        g_prev_win_dims = g_win_dims;
    }
    //
    if (0 && g_cob<0 && getobnum()) set_video_frame(0, k_before_start);
    if ((g_cob<0 || g_initial_time>0.) && getobnum()) {
        Object& o = getob(0);
        double desired_frametime = o.is_image() ? 0. : g_initial_time*o._video.attrib().framerate;
        set_video_frame(0, desired_frametime);
        if (g_framenum<int(floor(desired_frametime)) && o._nframes_loaded<o.nframes()) {
            // show blank window while waiting to seek to requested initial frame
            my_sleep(.1);
            redraw_later();
            return;
        } else {
            g_initial_time = 0.; // reset this setting
        }
    }
    process_keystring(g_keystring);
    if (!product(g_win_dims)) return;
    if (g_request_loop && g_request_loop_synchronously && g_working_on_loop_creation) {
        while (!g_videoloop_ready_obj) my_sleep(.1);
    }
    if (g_videoloop_ready_obj) { // background thread done creating seamless loop
        std::lock_guard<std::mutex> lg(g_mutex_obs);
        if (g_vlp_ready_obj) {
            add_object(std::move(g_vlp_ready_obj)); // insert right after current video
        }
        {
            add_object(std::move(g_videoloop_ready_obj)); // insert right after current video (or as first video)
            set_video_frame(g_cob, k_before_start);       // set to first frame
        }
    }
    if (g_selected.button_active && g_cob>=0) { // drag operation using one of the three mouse buttons.
        assertx(g_selected.button_active<=3);
        Vec2<int> yx;
        if (!get_pointer(yx)) g_selected.button_active = 0;
        bool shift_pressed   = get_key_modifier(HW::EModifier::shift);
        bool alt_pressed     = get_key_modifier(HW::EModifier::alt);
        switch (g_selected.button_active) {
         bcase 1: {             // either sliders or timeline
             if (g_selected.on_timeline) {
                 act_timeline(yx);
             } else if (g_use_sliders) {
                 int i = int(g_selected.yx_pressed[1]/float(g_win_dims[1])*g_sliders.num()*.9999f);
                 assertx(i>=0 && i<g_sliders.num());
                 float dval;
                 if (!g_selected.control_was_pressed) { // pointer position determines value
                     dval = exp((yx[0]-g_selected.yx_last[0])/float(-g_win_dims[0])*.90f); // was .60f
                 } else {       // pointer position determines rate of change
                     dval = exp((yx[0]-g_selected.yx_pressed[0])/float(-g_win_dims[0])*.02f);
                 }
                 if (alt_pressed) dval = pow(dval, .2f);
                 *g_sliders[i].pval *= dval;
                 redraw_later();
             }
         }
         bcase 2: {                               // zoom or rotate
             if (!g_selected.shift_was_pressed) { // zoom
                 float fac_zoom;
                 if (!g_selected.control_was_pressed) { // pointer position determines value
                     fac_zoom = exp((yx[0]-g_selected.yx_last[0])*.005f);
                 } else {       // pointer position determines rate of change
                     fac_zoom = exp((yx[0]-g_selected.yx_pressed[0])*.005f*.02f);
                 }
                 if (alt_pressed) fac_zoom = pow(fac_zoom, .2f);
                 perform_zoom_at_cursor(fac_zoom, g_selected.yx_pressed);
             } else {           // rotate
                 float vrotate = -(yx[0]-g_selected.yx_pressed[0])*.0001f;
                 if (alt_pressed) vrotate *= .1f;
                 perform_window_rotation(vrotate);
             }
             redraw_later();
         }
         bcase 3: {             // pan
             Vec2<float> yxd;
             if (!g_selected.control_was_pressed) { // pointer position determines value
                 yxd = convert<float>(yx-g_selected.yx_last);
             } else {           // pointer position determines rate of change
                 yxd = convert<float>(yx-g_selected.yx_pressed)*.02f;
             }
             if (alt_pressed) yxd *= .1f;
             if (shift_pressed) {
                 if (g_selected.axis_constraint<0 && g_selected.yx_last==g_selected.yx_pressed && mag2(yxd)) {
                     max_index(abs(yxd), &g_selected.axis_constraint);
                     if (g_verbose>=3) SHOW(abs(yxd), g_selected.axis_constraint);
                 }
                 if (g_selected.axis_constraint>=0)
                     yxd[1-g_selected.axis_constraint] = 0.f; // zero-out the contribution of least-moved dimension
             }
             Vec2<bool> b_constrain_inside = twice(false);
             if (!shift_pressed && !view_has_rotation()) {
                 // For each axis, restrict motion to image boundaries if we see the entire image on that axis.
                 for_int(c, 2) {
                     Vec2<int> winc;
                     for_int(idir, 2) {
                         const float eps = .001f; // was .01f
                         Vec2<float> ar = twice(0.f).with(c, idir==0 ? eps : g_frame_dims[c]-eps);
                         winc[idir] = get_win_yx(ar)[c];
                     }
                     if (0) SHOW(c, winc);
                     b_constrain_inside[c] = winc[0]<=0 && winc[1]>=g_win_dims[c]-1;
                 }
                 if (0) SHOW(b_constrain_inside);
             }
             Frame old_view = g_view;
             if (!is_zero(yxd)) {
                 set_view(g_view * Frame::translation(concat(yxd, V(0.f))));
                 for_int(c, 2) {
                     if (b_constrain_inside[c]) {
                         if (0) {
                             SHOW(c, g_view[3][c], g_win_dims[c], g_frame_dims[c], g_view[c][c],
                                  g_win_dims[c]-g_frame_dims[c]*g_view[c][c]);
                             SHOWP(g_view[c][c]); SHOWP(g_view[3][c]);
                             SHOWP(float(g_win_dims[c]-double(g_frame_dims[c])*g_view[c][c]));
                         }
                         g_view[3][c] = clamp(g_view[3][c],
                                              float(g_win_dims[c]-double(g_frame_dims[c])*g_view[c][c]), 0.f);
                         if (0) SHOWP(g_view[3][c]);
                     }
                 }
                 if (g_view!=old_view) g_fit_view_to_window = false;
             }
             redraw_later();
         }
         bdefault: assertnever("");
        }
        g_selected.yx_last = yx;
    }
    advance_frame();
    if (g_cob>=0) set_video_frame(g_cob, g_frametime);
    if (g_playing && g_cob>=0 && !getob().is_image()) redraw_later();
    int nframes_loaded = 0;
    if (g_cob>=0 && g_fit_view_to_window) {
        assertx(product(g_frame_dims));
        Vec2<float> arzoom = convert<float>(g_win_dims) / convert<float>(g_frame_dims);
        set_view(Frame::scaling(V(arzoom[0], arzoom[1], 1.f)));
        if (g_fit==EFit::isotropic) {
            int cmax = arzoom[0]>arzoom[1] ? 0 : 1;
            g_view[cmax][cmax] = arzoom[1-cmax];
            g_view[3][cmax] = (g_win_dims[cmax]-g_frame_dims[cmax]*arzoom[1-cmax])/2.f;
        }
    }
    const float min_zoom = min(get_zooms());
    const float k_min_zoom_for_2x2rgba = 45.f;
    const float k_min_zoom_for_4x1rgba_yx = 100.f;
    const bool show_2x2rgba = min_zoom>k_min_zoom_for_2x2rgba;
    const bool show_4x1rgba_yx = min_zoom>k_min_zoom_for_4x1rgba_yx;
    g_render_kernel = show_2x2rgba ? EKernel::nearest : g_kernel;
    clear_window();             // not needed if we overwrite framebuffer completely; even so, recommended.
    if (g_cob>=0) {             // render the current image
        nframes_loaded = getob()._nframes_loaded; // capture atomic variable just once
        if (g_refresh_texture) { upload_image_to_texture(); g_refresh_texture = false; }
        render_image();
    }
    app_set_window_title();
    if (g_cob>=0 && g_framenum>=0 && min_zoom>k_min_zoom_for_2x2rgba && !g_show_help) {
        // show RGBA values and coordinates as text
        const Vec2<int> fdims = get_font_dims();
        const float eps = .01f;
        const Vec2<Vec2<int>> yxminmax =
            bbox_minmax(map(minmax_corners(V(twice(-eps), convert<float>(g_win_dims)+eps)), &get_image_yx).view());
        for (Vec2<int> tex_yx : range(yxminmax[0], yxminmax[1]+1)) {
            if (!tex_yx.in_range(g_frame_dims)) continue;
            const Pixel pix = get_frame_pix(tex_yx);
            const Vec2<int> win_yx = get_win_yx(convert<float>(tex_yx)+.5f);
            if (show_4x1rgba_yx) {
                const auto yxsep = V(10, min_zoom<120.f ? 6 : min_zoom<140.f ? 7 : 8);
                for_int(c, 4) {
                    auto yx = win_yx + V(-fdims[0]-yxsep[0]/2, (c-2)*(fdims[1]*2+yxsep[1])+yxsep[1]/2) + V(-3, -1);
                    app_draw_text(yx, sform("%02X", pix[c]), k_no_text_wrap);
                }
                string s = sform("%d,%d", tex_yx[1], tex_yx[0]);
                auto yx = win_yx + V(yxsep[0]/2, -fdims[1]*narrow_cast<int>(s.size())/2) + V(-3, -1);
                app_draw_text(yx, s, k_no_text_wrap);
            } else {
                for_int(c, 4) {
                    auto yxsep = V(4, min_zoom<54.f ? 4 : min_zoom<70.f ? 5 : 6);
                    auto dyx = V(c/2, c%2);
                    auto yx = win_yx + (dyx-1)*(V(1, 2)*fdims+yxsep) + yxsep/2 + V(-3, -1);
                    string s = sform("%02X", pix[c]);
                    app_draw_text(yx, s, k_no_text_wrap);
                }
            }
        }
    }
    set_color_to_foreground();
    if (timeline_shown()) {     // show timeline
        set_color(Pixel::gray(30));
        const auto& g = g_timeline; {
            auto win_yx = convert<float>(g_win_dims);
            fill_rectangle(V(g.top, g.left)*win_yx, V(g.bot, g.right)*win_yx);
            set_color(Pixel::gray(128));
            // right boundary of set of loaded frames:
            const float rloaded = g.left+g.width*float(nframes_loaded)/getob().nframes();
            fill_rectangle(V(g.top, g.left)*win_yx, V(g.bot, rloaded)*win_yx);
            if (getob()._framein>0 || getob()._frameou1<getob().nframes()) {
                const float wcur = g.get_wcur(getob().nframes());
                const float l = g.left+(g.width-wcur)*float(getob()._framein)/(getob().nframes()-1);
                const float r = g.left+(g.width-wcur)*float(getob()._frameou1-1)/(getob().nframes()-1)+wcur;
                set_color(g.trim_color);
                fill_rectangle(V(g.top-g.trim_extrah, l)*win_yx, V(g.bot+g.trim_extrah, r)*win_yx);
            }
            if (g_framenum>=0) {
                const float wcur = g.get_wcur(getob().nframes());
                const float l = g.left+(g.width-wcur)*float(g_framenum)/max(getob().nframes()-1, 1);
                const float r = l + wcur;
                set_color(g.cur_color);
                fill_rectangle(V(g.bot, l)*win_yx, V(g.top, r)*win_yx);
            }
        }
        set_color_to_foreground();
    }
    if (g_show_info && g_cob>=0 && g_win_dims[1]>=100) { // show status line at top of window
        string s; {
            const auto& ob = getob();
            assertx(ob.size());
            string sobcount; {
                if (getobnum()>1) sobcount = sform("(%d of %d)  ", g_cob+1, getobnum());
            }
            string sdir; {
                const string& filename = getob()._filename;
                if (!file_requires_pipe(filename) && file_exists(filename)) {
                    CArrayView<string> filenames = get_directory_media_filenames(filename);
                    int i0 = filenames.index(get_path_tail(filename));
                    if (i0>=0 && filenames.num()>1)
                        sdir = sform("[%d/%d]  ", i0+1, filenames.num());
                }
            }
            string sframe;      // +speed+framerate for video
            if (!ob.is_image()) {
                const double video_framerate = ob._video.attrib().framerate ? ob._video.attrib().framerate : 30.;
                double dtime = g_framenum/video_framerate;
                int nmin = narrow_cast<int>(static_cast<uint64_t>(dtime)/60);
                int nsec = narrow_cast<int>(static_cast<uint64_t>(dtime)%60);
                int nmsec = int((dtime-floor(dtime))*1000.);
                string stime = sform("%d:%02d.%03d", nmin, nsec, nmsec);
                const int iframerate = int(video_framerate+.5);
                sframe = sform("%03d/%03d %s  %gx %dfps  ",
                               int(g_framenum), ob.nframes(), stime.c_str(), g_speed, iframerate);
            }
            string smiddle = get_szoom() + sform(" %dx%d ", g_frame_dims[1], g_frame_dims[0]);
            string smodes; {
                smodes = (ob.is_image() ? "" :
                          sform(" [%s] ", (g_looping==ELooping::all ? "a" : g_looping==ELooping::one ? "l":
                                           g_looping==ELooping::mirror ? "m" : " ")));
            }
            string sname; {
                if (is_fullscreen()) sname = " " + getob()._filename;
            }
            string sbitrate; {
                const int bitrate = ob._video.attrib().bitrate;
                sbitrate = (ob.is_image() ? "" :
                            bitrate>1000000 ? sform(" %.2fMbps", bitrate/1000000.f) :
                            bitrate>1000 ? sform(" %.2fKbps", bitrate/1000.f) :
                            bitrate>0 ? sform(" %dbps", bitrate) : "");
            }
            string sloaded; {
                int nremaining = 0;
                for_int(obi, getobnum()) { nremaining += getob(obi).nframes()-getob(obi)._nframes_loaded; }
                if (0 && nremaining) { sloaded = sform(" (%d frames to read)", nremaining); redraw_later(); }
            }
            s = sobcount + sdir + sframe + smiddle + smodes + sname + sbitrate + sloaded;
        }
        const int left = 6;
        s = s.substr(0, (g_win_dims[1]-left)/get_font_dims()[1]);
        app_draw_text(V(0, left), s, k_no_text_wrap);
    }
    const int font_height = get_font_dims()[0];
    if (g_show_help) {          // show overlaid help text
        Array<string> ar = {
            "<h>,<?>,<f1>: Toggle this help   (S=shift, C=control)",
            "Mouse:  <Left>select   <Mid>zoom   <S-Mid>rotate   <Right>pan",
            "Keys:  <r>eset_all",
            " <=>enlarge   <->shrink   <0>100%   <enter>fullscreen   <f>it_stretch   <w>indow_fit   <A>spectratio",
            " <k>ernel_filter   <b>rightness_controls",
            " <n>ext_object   <p>rev   <C-1>,<P>first   <N>last   <c>lone   <x>exchange_prev   <X>exchange_next",
            " <C-d>unload_object   <C-k>eep_only_cur   <C-c>copy   <C-v>paste",
            " <pgdn>next_file   <pgup>prev_file   <s>ort_order   <C-o>pen   <C-s>ave   <C-S-s>overwrite",
            " <f2>rename   <f5>reload   <f7>move   <f8>copy   <C-n>ew_window   <d>irectory",
            " <C>rop_to_view   <S>cale_using_view   <C-S-l>,<C-S-r>rotate   <W>hite_crop",
            " <v>iew_externally   <i>nfo   <e>xif   <H>checker   <~>console   <esc>quit",
            "Video:",
            " <spc>play/pause   <l>oop   <a>ll_loop   <m>irror_loop",
            " <left>frame-1   <right>frame+1   <home>first   <end>last",
            " <[>slower   <]>faster   <1>normal   <2>twice   <5>half   <F>ramerate   <B>itrate",
            " <,>mark_beg   <.>mark_end   <u>nmark   <T>rim   <C-S-t>cut   <|>split    <&>merge   <M>irror",
            " <R>esample_temporally   <g>en_seamless_loop   <C-g>high-quality_loop   <L>oop",
            " <I>mage_from_frame   <V>ideo_from_images   <#>from_image_files%03d",
        };
        for_int(i, ar.num())
            app_draw_text(V((4+i)*(font_height+4), 6), ar[i], k_no_text_wrap);
    } else if (g_show_exif) {
        Array<string> ar;
        if (g_cob>=0) {
            const auto& ob = getob();
            ar.push("Filter kernel: " + k_kernel_string[int(g_kernel)]);
            ar.push("File: " + ob._filename);
            ar.push(string() + "Status: " + (ob._unsaved ? "unsaved" : "saved"));
            string lower_filename = to_lower(ob._filename);
            if (1 && (ends_with(lower_filename, ".jpg") || ends_with(lower_filename, ".jpeg")) &&
                command_exists_in_path("exif")) {
                RFile fi("exif '" + ob._filename + "' 2>&1 |");
                // EXIF tags in 'c:/hh/desktop/christmas_tmp/20151225_103357.jpg' ('Intel' byte order):
                // --------------------+----------------------------------------------------------
                // Tag                 |Value
                // --------------------+----------------------------------------------------------
                // Image Width         |5312
                // ...
                // --------------------+----------------------------------------------------------
                // EXIF data contains a thumbnail (22890 bytes).
                ar.push("EXIF:");
                string sline;
                while (my_getline(fi(), sline)) {
                    if (contains(sline, "-----") || sline=="" ||
                        begins_with(sline, "Tag   ") || begins_with(sline, "EXIF tags")) continue;
                    if (!begins_with(sline, "EXIF tags")) sline = "  " + sline;
                    ar.push(sline);
                }
            }
            if (1 && ob._video.attrib().audio.size()) {
                ar.push("Audio: " + ob._video.attrib().audio.diagnostic_string());
            }
        }
        for_int(i, ar.num())
            app_draw_text(V((4+i)*(font_height+4), 6), ar[i], k_no_text_wrap);
    }
    if (g_use_sliders) {
        app_draw_text(V((2+1)*(font_height+4), 6), "Brightness sliders (<b>ake/exit   <r>eset   <control>vary)");
        for_int(i, g_sliders.num()) {
            float x = (i+.2f)/g_sliders.num(); int xi = int(x*g_win_dims[1]);
            app_draw_text(V((2+2)*(font_height+4), xi), g_sliders[i].name);
        }
        for_int(i, g_sliders.num()) {
            float x = (i+.2f)/g_sliders.num(); int xi = int(x*g_win_dims[1]);
            app_draw_text(V((2+3)*(font_height+4), xi+10), sform("%g", *g_sliders[i].pval));
        }
    }
    if (g_messages.num()) {     // show any messages
        {
            std::lock_guard<std::mutex> lg(g_mutex_messages);
            for_int(i, g_messages.num()) {
                app_draw_text(V((2+i)*(font_height+4), 6), g_messages[i].s);
            }
            for (int i = 0; ; ) {
                if (i>=g_messages.num()) break;
                if (g_messages[i].time<get_precise_time()) g_messages.erase(i, 1);
                else i++;
            }
        }
        redraw_later();
    } else if (!g_show_info) {
    } else if (g_cob<0) {
        app_draw_text(V((2+0)*(font_height+4), 6), "To open content, drag a file here or use key <C-o> or <d>");
        app_draw_text(V((2+1)*(font_height+4), 6), " (press key <h> for help)");
    } else if (!nframes_loaded) {
        app_draw_text(V((2+0)*(font_height+4), 6), "Waiting for first video frame...");
        redraw_later();
    } else if (image_is_not_visible()) {
        app_draw_text(V((2+0)*(font_height+4), 6), "Lost track of the image?  <f> to reset the view");
    } else if (g_selected.button_active==2) {
        app_draw_text(V((2+0)*(font_height+4), 6),
                      "Drag up/down to zoom  (<shift>:rotate)"
                      " (<f> to reset view, <h> for help)");
    } else if (g_selected.button_active==3) {
        app_draw_text(V((2+0)*(font_height+4), 6),
                      "Drag to pan  (<shift>:constraints) (<f> to reset view, <h> for help)");
    } else {
#if !defined(HH_NO_VIDEO_LOOP)
        if (0 && getobnum()==1 && g_cob>=0 && !getob().is_image() && !getenv_bool("HH_NO_LOOP_MESSAGE"))
            app_draw_text(V((2+0)*(font_height+4), 6), "To generate a loop, press the <g> key");
#endif
    }
    assertx(!gl_report_errors());
    if (g_frame_has_transparency) redraw_later();
}

// For video (either Video or VideoNv12) in object ob, with specified trim frames, compute looping parameters
//  and save these in g_lp.
void compute_looping_parameters(const Vec3<int>& odims, CGridView<3,Pixel> ovideo, CVideoNv12View ovideo_nv12,
                                int nnf) {
    const int onf = odims[0], ny = odims[1], nx = odims[2];
    // const int DT = 4, log2DT = 2; assertx((1<<log2DT)==DT);
    int DT = 4;                 // temporal downsampling factor
    if (onf<=50) {
        if (g_verbose) Warning("Few input frames, so omitting temporal downsampling");
        DT = 1;
    }
    int log2DT = int_floor_log2(DT); assertx((1<<log2DT)==DT);
    Grid<3,Pixel> hvideo; {
        HH_TIMER(loop_downsampling);
        const Vec2<int> sdims(ny, nx);
        int DS = 1; {           // spatial downsampling factor
            const int videoloops_maxh = getenv_int("VIDEOLOOPS_MAXH", 350, false); // default value used in Loopers
            Vec2<int> tdims = sdims;
            while (tdims[0]>videoloops_maxh) {
                if (!is_zero((tdims/2)%2)) {
                    SHOW("Stopping coarsening to even dims", sdims, tdims, videoloops_maxh, DS); break;
                }
                tdims /= 2;
                DS *= 2;
            }
        }
        if (1 && DS>1) DS /= 2; // Loopers::PipeAPI expects twice the spatial resolution to get good smoothing cost
        if (0) SHOW(DS);
        assertx(is_zero(V(ny, nx)%DS));
        const int hnf = onf/DT;
        if (0) assertx(onf%DT==0); // No, we allow arbitrary number of frames, but truncate to a multiple of DT.
        assertx(hnf>0);
        const FilterBnd filterb(Filter::get("box"), Bndrule::reflected);
        const Vec2<int> hdims = sdims/DS; assertx(hdims*DS==sdims);
        if (0) SHOW(sdims, hdims, DS);
        Grid<3,Pixel> hvideo1(concat(V(onf), hdims)); { // reduced (maybe "half") resolution
            HH_TIMER(__scale_spatially);
            if (ovideo.size()) {
                spatially_scale_Grid3_Pixel(ovideo, twice(filterb), nullptr, hvideo1);
            } else if (ovideo_nv12.size()) {
                for_int(f, onf) {
                    integrally_downscale_Nv12_to_Image(ovideo_nv12[f], hvideo1[f]);
                }
            } else assertnever("");
        }
        hvideo.init(concat(V(hnf), hdims));
        {
            HH_TIMER(__scale_temporally);
            parallel_for_int(f, hnf) {
                for_int(y, hdims[0]) for_int(x, hdims[1]) {
                    Vector4i v(0);
                    for_int(df, DT) { v += Vector4i(hvideo1(f*DT+df, y, x)); } // OPT:DT
                    hvideo(f, y, x) = (v>>log2DT).pixel();
                }
            }
        }
    }
    const Vec2<int> hdims = hvideo.dims().tail<2>();
    {
        HH_TIMER(loop_optimize);
        if (0) {                // for testing
            Warning("Testing special looping parameters");
            const int start = nnf/3, period = nnf/3;
            assertx(period<=nnf); assertx(start+period<odims[0]);
            g_lp.mat_start.init(hdims, start);
            g_lp.mat_period.init(hdims, period);
        } else if (1) {
#if !defined(HH_NO_VIDEO_LOOP)
            if (getenv_bool("LOOP_STATISTICS")) SHOW(hvideo.dims());
            if (1) mafTime::Disable();
            Loopers::Config config;
            // Note that config.m_FrameWindow is used by Loopers front-end.
            // PipeAPI::Optimize() ignores m_FrameWindow and always optimizes at granularity 1.
            // Nonetheless, Optimize() sets config.m_MinPeriod = config.m_MinPeriodSrc / config.m_FrameWindow  and
            //                              config.m_MaxPeriod = config.m_MaxPeriodSrc / config.m_FrameWindow
            //  using the default config.m_FrameWindow==4.
            if (0) {
                const float static_cost = 200.f; // default 50.f (it was once 200.f)
                if (0) my_setenv(Loopers::PipeAPI::env_static_cost(), sform("%g", static_cost));
                if (0) config.m_para_static = static_cost;
            }
            if (0) my_setenv(Loopers::PipeAPI::env_diagnostics(), "true");
            if (1) {
                // Unfortunately we do not have access to framerate here, so cannot work in seconds.
                // I am not using this now.
                int min_period = getenv_int("LOOP_MIN_PERIOD"); // in original frames
                if (min_period) { SHOW(min_period); config.m_MinPeriodSrc = min_period/DT*config.m_FrameWindow; }
            }
            if (config.m_MinPeriodSrc/config.m_FrameWindow*DT > onf) {
                // Warning("Very short video input, so setting minimum loop period equal to its length");
                // Not really necessary given my new check in PipeAPI::PostLoadCheck().
                config.m_MinPeriodSrc = onf/DT*config.m_FrameWindow;
            }
            if (0) {
                // These parameters are used by Loopers front-end and not by Loopers::PipeAPI.
                config.m_SkipSecs = 0.f;      // default 2.f
                config.m_KeepSecs = BIGFLOAT; // default 5.f
                // PipeAPI always uses entire input video, which is what we want here.
                // (If desired, video should have been temporally trimmed,
                //   or trim points _framein and _frameou1 should have been set.)
            }
            if (g_high_quality_loop) {
                g_high_quality_loop = false;
                config.m_AllLabels = true;
            }
            Loopers::PipeAPI pipe_api;
            pipe_api.SetConfig(config);
            assertx(pipe_api.Optimize(hvideo, g_lp.mat_period, g_lp.mat_start)); // multilabel graph cut
            assertw(is_zero(hdims%g_lp.mat_start.dims()));
            if (0) SHOW(pipe_api.GetConfig().m_MinPeriodSrc, pipe_api.GetConfig().m_MinPeriod);
            for (int& period : g_lp.mat_period) { period = period<=1 ? 1 : period*DT; }
            for (int& start : g_lp.mat_start) { start *= DT; }
        } else {                // for debugging
#endif
            Warning("Compiled without support for seamless video loop");
            const int start = 10, period = 70;
            assertx(period<=nnf); assertx(start+period<=odims[0]);
            g_lp.mat_start.init(hdims, start);
            g_lp.mat_period.init(hdims, period);
        }
        if (0 || getenv_bool("OUTPUT_LOOP_PARAMETERS")) {
            Image image(g_lp.mat_start.dims()); // size may be different from hdims
            const int K = 4;
            for (const auto& yx : range(image.dims())) {
                uchar static_frame = '\0';
                int start = g_lp.mat_start[yx], period = g_lp.mat_period[yx];
                if (!(start%K==0 && (period%K==0 || period==1))) SHOW(yx, start, period);
                assertx(start%K==0 && (period%K==0 || period==1));
                image[yx] = Pixel(static_frame, narrow_cast<uchar>(start/K), narrow_cast<uchar>(period/K), 255);
            }
            image.write_file("output_loop_parameters.vlp.png");
        }
    }
    if (1) {
        HH_TIMER(loop_rstat);
        HH_RSTAT(Speriod, g_lp.mat_period);
        HH_RSTAT(Sstart, g_lp.mat_start);
        // e.g.:
        // # Speriod:            (130560 )           1:88           av=72.804352      sd=31.987289
        // # Sstart:             (130560 )           4:56           av=28.371078      sd=15.893303
    }
}

// In a detached thread, asynchronously read video(s) and possibly compute a seamless video loop.
void background_work(bool asynchronous) {
#if defined(_WIN32)
    if (1) {                    // Lower priority of the current (background) thread.
        assertx(SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL));
    }
#else
    // see http://stackoverflow.com/questions/10876342/equivalent-of-setthreadpriority-on-linux-pthreads
#endif
    if (0) my_sleep(2.);        // test delay in video read
    for (;;) {
        {               // Identify a video with frames not yet loaded, prioritizing the current video object.
            Object* pob = nullptr;
            bool is_cob = false; // pob is current object
            {
                std::lock_guard<std::mutex> lg(g_mutex_obs);
                if (g_cob<0) {
                } else if (getob()._prvideo) {
                    pob = &getob(); is_cob = true;
                } else {
                    for_int(obi, getobnum()) {
                        if (getob(obi)._prvideo) { pob = &getob(obi); break; }
                    }
                }
                if (pob) pob->_locked_by_background_thread = true;
            }
            if (pob) {
                auto& ob = *pob;
                assertx(ob._prvideo);
                if (ob._nframes_loaded<ob.nframes()) {
                    bool success = (ob._video.size() ?
                                    ob._prvideo->read(ob._video[ob._nframes_loaded]) :
                                    ob._prvideo->read(ob._video_nv12[ob._nframes_loaded]));
                    if (g_verbose>=2) SHOW("background", is_cob, g_framenum, ob._nframes_loaded, success);
                    if (success) {
                        ob._nframes_loaded++;
                    } else {
                        if (0) SHOW("Premature video end: failed to load video frame", ob._nframes_loaded);
                        // Hopefully reducing the number of frames does not confuse the foregound thread anywhere.
                        if (ob._video.size()) {
                            ob._video.special_reduce_dim0(ob._nframes_loaded);
                        } else {
                            ob._video_nv12.special_reduce_dim0(ob._nframes_loaded);
                        }
                        ob._dims[0] = ob._nframes_loaded;
                        ob._frameou1 = min(ob._frameou1, ob.nframes());
                    }
                    if (1) hw.redraw_later();
                    hw.wake_up();
                    if (asynchronous && g_playing && (!is_cob || ob._nframes_loaded>g_framenum+5)) {
                        if (g_framenum<0) {
                            if (g_framenum<0) my_sleep(.01); // help main thread get started
                        } else {
                            my_sleep(0.); // help main thread catch up if it can
                        }
                    }
                }
                if (ob._nframes_loaded==ob.nframes()) ob._prvideo.reset(); // free up read stream
                ob._locked_by_background_thread = false;
                continue;
            }
        }
        {            // Presently there is no more video content to be loaded.  Possibly create seamless loop.
            Object* pob = nullptr;
            {
                std::lock_guard<std::mutex> lg(g_mutex_obs);
                if (g_cob>=0 && !g_videoloop_ready_obj && !getob().is_image() &&
                    (g_request_loop || (g_lp.is_loaded && getobnum()==1))) {
                    pob = &getob();
                    pob->_locked_by_background_thread = true;
                    g_working_on_loop_creation = true;
                }
            }
            if (pob) {
                auto& ob = *pob;
                HH_TIMER(loop);
                const Vec3<int> odims = ob._dims.with(0, ob._frameou1-ob._framein);
                CGridView<3,Pixel> ovideo(ob._video.size() ? ob._video.slice(ob._framein, ob._frameou1) : ob._video);
                CVideoNv12View ovideo_nv12(ob._video_nv12.size() ?
                                           CVideoNv12View(ob._video_nv12.get_Y().slice(ob._framein, ob._frameou1),
                                                          ob._video_nv12.get_UV().slice(ob._framein, ob._frameou1)) :
                                           ob._video_nv12);
                // frames in seamless video loop; was 150
                const int nframes = int(k_loop_duration*ob._video.attrib().framerate+.5);
                const Vec3<int> ndims = concat(V(nframes), ob.spatial_dims());
                const int use_nv12 = !ovideo.size();
                //----------------------------------------------------------------------
                message("Computing looping parameters for seamless video loop...", 1e6);
                if (!g_lp.is_loaded) {
                    // computes g_lp.mat_start, g_lp.mat_period
                    compute_looping_parameters(odims, ovideo, ovideo_nv12, nframes);
                }
                //----------------------------------------------------------------------
                message("Assembling seamless video loop...", 1e6);
                Video videoloop(!use_nv12 ? ndims : thrice(0));
                VideoNv12 videoloop_nv12(use_nv12 ? ndims : thrice(0));
                {
                    HH_TIMER(loop_gdloop);
                    EGDLoopScheme scheme = 1 ? EGDLoopScheme::fast : EGDLoopScheme::precise;
                    compute_gdloop(odims, "", ovideo, ovideo_nv12,
                                   g_lp.mat_start, g_lp.mat_period, scheme, ndims[0],
                                   nullptr, videoloop, videoloop_nv12, 1);
                }
                //----------------------------------------------------------------------
                message("Here is the resulting seamless video loop (press <p><p> to select original video)", 5.);
                if (1) {
                    string filename = get_path_root(getob()._filename) + "_vlp.png";
                    Image image_vlp(g_lp.mat_start.dims());
                    // see also Filterimage.cpp vlp_mask_to_color
                    const int num_input_frames = odims[0];
                    parallel_for_coords(image_vlp.dims(), [&](const Vec2<int>& yx) {
                        int start = g_lp.mat_start[yx];
                        int period = g_lp.mat_period[yx];
                        float fstart = float(start)/(num_input_frames-period-1);
                        float fperiod = float(period)/(num_input_frames-1);
                        Pixel pix = k_color_ramp[clamp_to_uchar(int(fperiod*255.f+.5f))];
                        for_int(c, 3) { pix[c] = clamp_to_uchar(int(pix[c]*(.4f+.6f*fstart))); }
                        const bool have_mask = false;
                        const bool is_masked = false;
                        if (period==1) pix = Pixel::gray(have_mask ? 180 : 230);
                        if (is_masked) pix = Pixel::white();
                        image_vlp[yx] = pix;
                    });
                    const bool bgra = false;
                    g_vlp_ready_obj = make_unique<Object>(std::move(image_vlp), std::move(filename), bgra);
                }
                {
                    string filename = append_to_filename(getob()._filename, "_loop");
                    g_videoloop_ready_obj = make_unique<Object>(ob, std::move(videoloop), std::move(videoloop_nv12),
                                                                filename);
                    g_videoloop_ready_obj->_video.attrib().audio.clear(); // open research problem
                }
                g_lp.is_loaded = false; // reset
                ob._locked_by_background_thread = false;
                g_working_on_loop_creation = false;
                g_request_loop = false;
                if (1) hw.redraw_later();
                hw.wake_up();
                // Note: for batch loop creation, it is best to use do_batch_create_loop().
                if (getenv_bool("VIDEOLOOP_EXIT")) { HH_TIMER_END(loop); SHOW("exiting"); exit(0); }
                continue;
            }
        }
        {                       // Try image prefetching
            int i = -1;
            string filename;
            {
                std::lock_guard<std::mutex> lg(g_mutex_prefetch);
                for_int(ii, 2) {
                    if (g_prefetch_image[ii].filename!="" && g_prefetch_image[ii].file_modification_time==0) {
                        assertx(!g_prefetch_image[ii].pimage);
                        i = ii; filename = g_prefetch_image[i].filename;
                        break;
                    }
                }
            }
            if (i>=0) {
                bool ok = true;
                uint64_t file_modification_time = get_path_modification_time(filename);
                if (!file_modification_time) file_modification_time = 1; // a small nonzero value
                Image image;
                try {
                    HH_CTIMER(_background_read_image, g_verbose>=1);
                    read_image(image, filename);
                }
                catch (std::runtime_error&) {
                    ok = false;
                }
                if (g_verbose>=1) SHOW("prefetched", i, filename, file_modification_time, ok);
                {
                    std::lock_guard<std::mutex> lg(g_mutex_prefetch);
                    if (g_prefetch_image[i].filename==filename) {
                        g_prefetch_image[i].file_modification_time = file_modification_time;
                        g_prefetch_image[i].pimage = ok ? make_unique<Image>(std::move(image)) : nullptr;
                    }
                }
            }
        }
        if (!asynchronous) break;
        my_sleep(.1); // nothing to do, so wait
        if (0) break; // never let thread terminate, so that it can still handle new files from drag-and-drop.
    }
    // Asynchronous thread would terminate after returning from this function.
}

// Add a video object.
void do_video(Args& args) {
    string filename = args.get_filename();
    if (!file_requires_pipe(filename)) filename = get_path_absolute(filename);
    std::lock_guard<std::mutex> lg(g_mutex_obs);
    g_obs.push(object_reading_video(filename)); // may throw
}

// Add an image object.
void do_image(Args& args) {
    string filename = args.get_filename();
    if (!file_requires_pipe(filename)) filename = get_path_absolute(filename);
    std::lock_guard<std::mutex> lg(g_mutex_obs);
    g_obs.push(object_reading_image(filename)); // may throw
}

// Add either an image or video.
void do_stdin(Args& args) {
    // Let do_video() or do_image() parse the "-" argument.
    int c = std::cin.peek();
    if (c<0) assertnever("Empty stdin");
    string video_suffix = video_suffix_for_magic_byte(uchar(c));
    string image_suffix = image_suffix_for_magic_byte(uchar(c));
    assertx(video_suffix=="" || image_suffix=="");
    if (video_suffix=="" && image_suffix=="") Warning("Unrecognized magic byte; assuming stdin is video");
    if (image_suffix=="")
        do_video(args);
    else
        do_image(args);
    
}

// Read a *.vlp file containing video looping parameters, to create a seamless video loop.
void do_vlp(Args& args) {
    string filename = args.get_filename(); // it is a png file, even if its suffix is *.vlp
    Image image;
    try {
        image.read_file(filename);
    }
    catch (const std::runtime_error& ex) {
        SHOW("error reading vlp image", filename, ex.what());
        return;
    }
    assertx(image.zsize()>=3);
    g_lp.mat_static.init(image.dims());
    g_lp.mat_start.init(image.dims());
    g_lp.mat_period.init(image.dims());
    g_lp.mat_activation.init(image.dims());
    const int K = 4;
    const int static_frame_offset = 2;
    for (const auto& yx : range(image.dims())) {
        g_lp.mat_static[yx] = image[yx][0]*K + static_frame_offset;
        g_lp.mat_start[yx] = image[yx][1]*K;
        g_lp.mat_period[yx] = image[yx][2]*K;
        g_lp.mat_activation[yx] = image.zsize()==4 ? image[yx][3]/254.f : 0.f;
        if (g_lp.mat_period[yx]==0) g_lp.mat_period[yx] = 1; // static pixel should have period==1
        if (g_lp.mat_period[yx]==1) g_lp.mat_start[yx] += static_frame_offset;
    }
    g_lp.is_loaded = true;
}

static void crop_spatial_dimensions_to_multiple(VideoNv12& onv12, int k) {
    assertx(k%2==0);            // should be a multiple of 2 for UV representation
    // Workaround for Cygwin gcc 5.4.0 compiler bug in demos/create_videoloop_palmtrees.sh:
    //  (cd ~/src/demos; ~/src/bin/cygwin/VideoViewer -batch_create_loop data/palmtrees_small.mp4 v.mp4)
    // Vec2<int> odims = onv12.get_Y().dims().tail<2>();
    Vec3<int> tmpdims = onv12.get_Y().dims();
    Vec2<int> odims = tmpdims.tail<2>();
    Vec2<int> ndims = odims/k*k;
    if (ndims==odims) return;
    Warning("Cropping spatial dimensions of video to more even size");
    onv12 = VideoNv12(crop(onv12.get_Y(),  thrice(0), concat(V(0), odims-ndims)),
                      crop(onv12.get_UV(), thrice(0), concat(V(0), (odims-ndims)/2)));
}

// Time the creation of a video loop (without the overhead of opening and updating the window).
//  e.g.:  VideoViewer -batch_create_loop ~/proj/fastloops/data/maf_rsig00/SDstreetlight/SDstreetlight_orig.mp4 ""
void do_batch_create_loop(Args& args) {
    string input_filename = args.get_filename();
    string output_filename = args.get_string();
    if (output_filename!="") {
        Args::check_filename(output_filename);
        output_filename = get_canonical_path(output_filename);
    }
    VideoNv12 ovideo_nv12;
    Video::Attrib attrib;
    ovideo_nv12.read_file(input_filename, &attrib);
    attrib.audio.clear();
    SHOW("original video", ovideo_nv12.get_Y().dims());
    { const int orig_num_frames = ovideo_nv12.get_Y().dim(0); assertx(orig_num_frames>2); }
    crop_spatial_dimensions_to_multiple(ovideo_nv12, 8);
    if (ovideo_nv12.get_Y().dim(0)<20) {
        // For short input video, repeat the first and last frames to enable looping to include these end frames.
        ovideo_nv12 = VideoNv12(crop(ovideo_nv12.get_Y(),  V(-1, 0, 0), V(-1, 0, 0), thrice(Bndrule::clamped)),
                                crop(ovideo_nv12.get_UV(), V(-1, 0, 0), V(-1, 0, 0), thrice(Bndrule::clamped)));
    }
    {
        HH_TIMER(loop);
        // frames in seamless video loop; was 150
        int nframes = int(k_loop_duration*attrib.framerate+.5);
        nframes = getenv_int("LOOP_NFRAMES", nframes, g_verbose>=1);
        const Vec3<int> odims = ovideo_nv12.get_Y().dims();
        const Vec3<int> ndims = concat(V(nframes), ovideo_nv12.get_Y().dims().tail<2>());
        {
            HH_TIMER(loop_optimize);
            compute_looping_parameters(odims, Video(), ovideo_nv12, nframes);
        }
        EGDLoopScheme scheme = 1 ? EGDLoopScheme::fast : EGDLoopScheme::precise;
        Video ovideo;
        VideoNv12 videoloop_nv12;
        Video videoloop;
        if (scheme==EGDLoopScheme::fast) {
            videoloop_nv12.init(ndims);
        } else {
            ovideo.init(ovideo_nv12.get_Y().dims()); convert_VideoNv12_to_Video(ovideo_nv12, ovideo);
            ovideo_nv12.clear();
            videoloop.init(ndims); videoloop.attrib() = attrib;
        }
        {
            HH_TIMER(loop_gdloop);
            compute_gdloop(odims, "", ovideo, ovideo_nv12,
                           g_lp.mat_start, g_lp.mat_period, scheme, ndims[0],
                           nullptr, videoloop, videoloop_nv12, 1);
        }
        HH_TIMER_END(loop);
        if (output_filename!="") {
            if (videoloop_nv12.get_Y().size()) videoloop_nv12.write_file(output_filename, attrib);
            else videoloop.write_file(output_filename);
        }
    }
    exit(0);
}

// Create a procedural video containing a moving vertical stripe.
//  e.g.:  VideoViewer -stripe 320 640 480 60
void do_stripe(Args& args) {
    const int nframes = args.get_int();
    const int xsize = args.get_int();
    const int ysize = args.get_int();
    const double framerate = args.get_double();
    const Vec3<int> dims = V(nframes, ysize, xsize);
    const Pixel back_color = Pixel::white();
    const Pixel stripe_color = Pixel::black();
    // SHOW(int(RGB_to_Y(back_color)), int(RGB_to_U(back_color)), int(RGB_to_V(back_color)));
    // SHOW(int(RGB_to_Y(stripe_color)), int(RGB_to_U(stripe_color)), int(RGB_to_V(stripe_color)));
    Video video;
    VideoNv12 video_nv12;
    const bool use_nv12 = k_prefer_nv12 && is_zero(dims.tail<2>()%2);
    if (!use_nv12) {
        video.init(dims); fill(video, back_color);
        for_int(f, nframes) {
            const int stripe_width = 2;
            int x0 = int(float(f)/max(nframes-1, 1)*(xsize-stripe_width)+.5f);
            assertx(x0+stripe_width<=xsize);
            for_int(dx, stripe_width) fill(column(video[f], x0+dx), Pixel::black());
        }
    } else {
        video_nv12.init(dims); fill(video_nv12.get_Y(), RGB_to_Y(back_color));
        fill(video_nv12.get_UV(), twice(uchar{128})); // both Pixel::black() and Pixel::white() have this UV
        for_int(f, nframes) {
            const int stripe_width = 2;
            int x0 = int(float(f)/max(nframes-1, 1)*(xsize-stripe_width)+.5f);
            assertx(x0+stripe_width<=xsize);
            for_int(dx, stripe_width) fill(column(video_nv12[f].get_Y(), x0+dx), RGB_to_Y(stripe_color));
        }
    }
    video.attrib().framerate = framerate;
    {
        std::lock_guard<std::mutex> lg(g_mutex_obs);
        const bool unsaved = true;
        g_obs.push(make_unique<Object>(std::move(video), std::move(video_nv12), nullptr,
                                       get_current_directory() + "/stripe_video.mp4", unsaved));
    }
}

// Create a procedural image containing a "zonal plate".
//  e.g.:  VideoViewer -zonal 1024 1024
void do_zonal(Args& args) {
    const int xsize = args.get_int();
    const int ysize = args.get_int();
    Image image(V(ysize, xsize));
    if (0) {
        const float scale = 25.f/float(mean(image.dims()));
        parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) {
            image[yx] = Pixel::gray(uchar((cos(square((ysize-1-yx[0])*scale)+square(yx[1]*scale))*.499f+.5f)*256.f));
        });
    } else {
        // from resample/supersampling/zonal.cpp
        const float scale = TAU/float(sum(image.dims()));
        parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) {
            image[yx] = Pixel::gray(uchar((cos((square(ysize-1-yx[0])+square(yx[1]))*scale)*.45f+.5f)*256.f));
        });
    }
    {
        std::lock_guard<std::mutex> lg(g_mutex_obs);
        const bool bgra = false; const bool unsaved = true;
        g_obs.push(make_unique<Object>(std::move(image), get_current_directory() + "/zonal_plate.png", bgra, unsaved));
    }
}

void DerivedHW::drag_and_drop(CArrayView<string> filenames) {
    if (g_verbose>=1) SHOW(filenames);
    if (filenames.num()==1 && ends_with(filenames[0], ".vlp")) {
        { Args args{filenames[0]}; do_vlp(args); }
    } else {
        int nread = 0;
        for (auto& filename : filenames) {
            try {
                Args args{filename};
                if (filename_is_image(filename)) {
                    do_image(args);
                } else {
                    do_video(args);
                }
                nread++;
            }
            catch (std::runtime_error& ex) {
                message("Error loading file " + filename + " : " + ex.what(), 10.);
            }
        }
        if (nread) set_video_frame(getobnum()-nread, k_before_start);
        if (g_cob>=0 && (1 || g_cob==0)) { // resize window appropriately
            reset_window(determine_default_window_dims(g_frame_dims));
        }
    }
}

// Accumulate a set of keystrokes to simulate after opening the window.
void do_key(Args& args) {
    g_keystring += args.get_string();
}

void test() {
    if (0) {
        auto poly = V(V(0.f, 0.f), V(1.f, 0.f), V(0.f, 1.f));
        for_int(i, 10) SHOW(i, intersect_poly_poly_2D(poly, poly+V(i*.2f, 0.f)));
    }
    if (0) {
        auto poly1 = minmax_corners(V(twice(0.f), twice(1.f)));
        auto poly2 = V(V(0.f, 0.f), V(1.f, 0.f), V(0.f, 1.f));
        SHOW(poly1, poly2);
        for_int(i, 10) SHOW(i, intersect_poly_poly_2D(poly1, poly2+V(-1.f+i*.1f, -.5f)));
    }
}

} // namespace

/// Run as:
///   VideoViewer
///   VideoViewer ~/data/video/HDbrink8h{.mp4,_loop.vlp}
///   VideoViewer ~/data/video/{HDbrink8h.mp4,fewpalms.mp4}
///   VideoViewer ~/data/image/lake.png
int main(int argc, const char** argv) {
    ensure_utf8_encoding(argc, argv);
    Array<string> aargs(argv, argv+argc);
    g_argv0 = get_canonical_path(aargs[0]);
    if (!contains(aargs, "-")) {
        if (0) assertx(!close(0)); // close stdin unless we need it, for Windows app started from emacs shell
        if (1) assertx(dup2(1, 0)>=0); // close stdin, but do not leave fd0 empty in case we open another file
    }
    bool b_help = aargs.num()==2 && ParseArgs::special_arg(aargs[1]);
    if (0) { test(); return 0; }
    bool hw_success = hw.init(aargs);
    double speed = 1.;
    double time = 0.;
    string through_color = "#FF9696";   // pink
    bool checker = true;
    ParseArgs args(aargs);
    ARGSD(video,                "file : load input video");
    args.p("*.mp4", do_video,   ": load input video");
    args.p("*.wmv", do_video,   ": load input video");
    args.p("*.avi", do_video,   ": load input video");
    args.p("*.mov", do_video,   ": load input video");
    args.p("*.MP4", do_video,   ": load input video");
    args.p("*.WMV", do_video,   ": load input video");
    args.p("*.AVI", do_video,   ": load input video");
    args.p("*.MOV", do_video,   ": load input video");
    args.p("-",     do_stdin,   ": load input image or video");
    ARGSD(image,                "file : load input image");
    args.p("*.jpg",  do_image,  ": load input image");
    args.p("*.jpeg", do_image,  ": load input image");
    args.p("*.png",  do_image,  ": load input image");
    args.p("*.bmp",  do_image,  ": load input image");
    args.p("*.JPG",  do_image,  ": load input image");
    args.p("*.JPEG", do_image,  ": load input image");
    args.p("*.PNG",  do_image,  ": load input image");
    args.p("*.BMP",  do_image,  ": load input image");
    args.p("*.arw",  do_image,  ": load input image (requires IMAGE_IMPLEMENTATION=FF)");
    args.p("*.exr",  do_image,  ": load input image (requires IMAGE_IMPLEMENTATION=FF)");
    ARGSD(vlp,                  "file.vlp : load looping parameters");
    args.p("*.vlp", do_vlp,     ": load looping parameters");
    ARGSD(batch_create_loop,    "input_video.mp4 output_loop.mp4 : create loop without opening any window");
    ARGSD(stripe,               "nf nx ny rate: create procedual video (e.g. 150 640 480 60)");
    ARGSD(zonal,                "nx ny : create zonal-plate procedural image (e.g. 1024 1024)");
    ARGSD(key,                  "keystring : simulate key presses (e.g. -key 'a2<enter>kk')");
    ARGSP(speed,                "fac : set fractional speedup factor");
    ARGSP(time,                 "sec : set initial frame time");
    ARGSP(through_color,        "#RRGGBB : for partially transparent images");
    ARGSP(checker,              "b : for partially transparent images");
    args.p("-verbose", g_verbose, "b : debug verbosity (0=none, 1=some, 2=more");
    try {
        if (!args.parse() || !hw_success) return 1;
    }
    catch (const std::runtime_error& ex) {
        showf("Error while parsing command-line arguments: %s\n", ex.what());
        return 1;
    }
    if (b_help) return 0;
    assertx(speed>0.); g_speed = speed;
    assertx(time>=0.); if (time) g_initial_time = time;
    g_through_color = parse_color(through_color);
    g_checker = checker;
    assertx(g_cob<getobnum());  // g_cob is initialized to -1, so always true
#if defined(HH_HAVE_BACKGROUND_THREAD)
    {
        // Launch asynchronous background thread
        std::thread th{background_work, true};
        th.detach();            // the detached thread never terminates
#if defined(_WIN32)
        // Raise priority of the current (foreground) thread (which renders video into window).
        if (0) assertx(SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST)); // omitting seems fine
        if (0) assertx(SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS));
        if (0) assertx(SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS));
#endif
    }
#endif
    hw.set_window_title("VideoViewer");
    Vec2<int> win_dims = (getobnum() ? determine_default_window_dims(getob(0).spatial_dims()) :
                          k_default_window_dims); // window geometry when no video is yet loaded
    hw.set_default_geometry(sform("%dx%d+130+0", win_dims[1], win_dims[0]));
    hw.set_default_background("black"); // not used since OpenGL rendering completely covers it
    if (1) {
        hw.set_default_foreground("yellow"); g_text_shadow_color = Pixel::black();
    } else {
        hw.set_default_foreground("black");  g_text_shadow_color = Pixel::white(); // does not look as nice
    }
    hw.open();
    return 0;
}
