// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#define DEF_PM
#define DEF_SR
#define DEF_SC
#define DEF_PLY

#include "G3dOGL/HB.h"
#include "G3dOGL/SCGeomorph.h"         // DEF_SC
#include "G3dOGL/SimplicialComplex.h"  // DEF_SC
#include "G3dOGL/SplitRecord.h"        // DEF_SC
#include "G3dOGL/normalmapping.h"
#include "HW.h"
#include "libHh/A3dStream.h"
#include "libHh/Args.h"
#include "libHh/Array.h"
#include "libHh/BinaryIO.h"
#include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"
#include "libHh/GMesh.h"
#include "libHh/Image.h"
#include "libHh/MathOp.h"
#include "libHh/MeshOp.h"        // Vnors
#include "libHh/NetworkOrder.h"  // DEF_PLY
#include "libHh/PMesh.h"         // DEF_PM
#include "libHh/Polygon.h"
#include "libHh/SGrid.h"
#include "libHh/SRMesh.h"  // DEF_SR
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
#include "libHh/Video.h"

using namespace hh;

namespace hh {
extern bool g_is_ati;  // used in PMesh_ogl.cpp
bool g_is_ati = false;
}  // namespace hh

extern float ambient;  // used in G3devent.cpp
float ambient;

#if defined(_WIN32) && !defined(_WIN64)
// Problem: with __MINGW32__, restarting GL_TRIANGLES seems to require reinitializing some color state.
//   Same problem with Win32 build.  e.g.:
// FilterPM ~/src/demos/data/cessna.pm -nf 3000 -geom_nf 5000 | ~/src/bin/win32/G3dOGL -st ~/src/demos/data/cessna.s3d -key ,S
// ~/src/bin/win32/G3dOGL ~/src/demos/data/standingblob.geomorphs -key PDeS -lightambient .5 -thickboundary 1
constexpr bool force_color_update = true;
#else
constexpr bool force_color_update = false;
#endif

#define ALLOW_LOD  // Comment this out to minimize space usage

namespace g3d {

extern string statefile;   // to modify statefile name
extern float fchange;      // to get constant speed sliders
extern bool input;         // to detect EOF for -picture
extern bool output;        // to output lod changes
extern Frame tview;        // to let SR access view matrix
extern int info;           // for SR diagnostics
extern string g_filename;  // to override filename for ob1
extern float lod_level;    // in movie/video
extern void UpdateOb1Bbox(const Bbox& bbox);
extern void update_lod();
extern bool keep_stdin_open;
extern float override_frametime;

}  // namespace g3d

namespace {

struct Slider {
  string name;
  float* val;
};

constexpr int k_max_object = 2048;  // should be >= objects::MAX

// const string k_default_geometry = "600x600+0+0";
// const string k_default_geometry = "1000x1000+0+0";
const string k_default_geometry = "1000x1000+150+0";
constexpr float k_default_hither = .1f;
constexpr float k_default_yonder = BIGFLOAT;
constexpr bool lod_use_nvertices = true;  // lod input/output uses #vertices, not slider

struct DerivedHW : HW {
  bool key_press(string s) override;
  void button_press(int butnum, bool pressed, const Vec2<int>& yx) override;
  void wheel_turn(float v) override;
  void draw_window(const Vec2<int>& dims) override;
  void input_received() override {
    if (_finpu) _finpu();
  }
  void (*_finpu)(){nullptr};
};

DerivedHW hw;

bool quickmode;  // draw quick segments
int quicki;      // draw every quicki'th segment
bool butquick;   // do quickmode when button is pressed
bool mdepthcue;
bool antialiasing;
int thickboundary = 2;
int thicksharp = 2;
int thicknormal = 1;
int thicka3d = 1;
float thickpoint = 2.f;  // was 1.f
bool nice_rendering;
bool perspec = true;
bool uvtopos;
bool slidermode;
bool dbuffer;
bool picture;
bool inpicture;
unique_ptr<ConsoleProgress> movie_cprogress;
int movie_nframes;
string movie_rootname;
bool movie_desire_video;
unique_ptr<Video> movie_video;
int movie_frame;
bool cullbackedges = true;
bool outside_frustum;
float frustum_frac = 1.4f;
float hither;
float yonder;
float lightsource;
Vec3<float> backfacec;
float fdisplacepolygon = 1.f;
float flinewidth = 1.f;
float sphereradius = 0.f;  // was .005
Array<string> texturemaps;
bool texturenormal;
bool texture_lit;
float texturescale = 0.f;  // 0 == no wrapping
bool worldlight;
int anisotropy = 1;
bool noinfo;
bool g_twosided = getenv_bool("G3D_TWOSIDED");        // no green ever
const int g_twolights = getenv_int("G3D_TWOLIGHTS");  // 0, 1, 2

bool use_dl = true;
Set<int> svalid_dl;
bool defining_dl = false;
bool was_using_dl = false;

float real_zoom;
float view_zoom;
Frame tpos;
Frame tcam;
bool (*fkeyp)(const string& s);
void (*fbutp)(int butnum, bool pressed, bool shift, const Vec2<float>& yx);
void (*fwheel)(float v);
void (*fdraw)();

// string imagefilename="| Filterimage -to jpg | clip";
// string imagefilename="| Filterimage -to jpg >~/tmp/v.jpg";
// string imagefilename="| Filterimage -to png >c:/hh/tmp/v.png";
// string imagefilename = "| Filterimage -to png | csh.exe -c write_to_desktop";
string imagefilename = "| Filterimage -to png | csh -c write_to_desktop";
bool is_window;      // window is open and drawable
Vec2<int> win_dims;  // window dimensions
Vec2<float> tzp;
float edgeoffset;
Frame tcami;
Frame feyetomodel;
// Frame fpostomodel;
int cob;
bool lquickmode;  // is quickmode active now?
bool lshading;
bool lsmooth;
bool ledges;
bool lcullface;
int button_active;
int num_concave;
bool textures_loaded;
bool texture_active;
int strip_lines;  // 0..2
string edgecolor = "black";
Pixel pix_edgecolor;
string sharpedgecolor = "blue";  // was "#FFFF40"
Pixel pix_sharpedgecolor;
string bndedgecolor = "blue";  // was "#FFFF40"
Pixel pix_bndedgecolor;
Vec2<float> yx_pointer_old;
const Frame k_eye_to_gleye{Vector(0.f, 0.f, -1.f), Vector(-1.f, 0.f, 0.f), Vector(0.f, 1.f, 0.f),
                           Point(0.f, 0.f, 0.f)};
bool ogl_normalize;
constexpr float k_one_slider_left_thresh = 0.2f;  // left area of screen for 1 slider
const Vector k_lightdir_eye0{-1.f, -.6f, .3f};
const Vector k_lightdir_eye1{+1.f, +.6f, -.3f};

struct Color {
  Pixel d;
  Pixel s;
  float g;
};

struct VertexLOD {
  Point Opos;
  Point Npos;
  Vector Onor;
  Vector Nnor;
  Pixel Od;
  Pixel Nd;
  UV Ouv;
  UV Nuv;
};

struct CornerLOD {
  Vector Onor;
  Vector Nnor;
  Pixel Od;
  Pixel Nd;
};

#if defined(ALLOW_LOD)
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, VertexLOD, v_lod);
HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, CornerLOD, c_lod);
#else
VertexLOD& v_lod(Vertex) { assertnever(""); }
CornerLOD& c_lod(Corner) { assertnever(""); }
#endif

HH_SAC_ALLOCATE_FUNC(Mesh::MFace, Vector, f_pnor);

const FlagMask mflag_f_colors = Mesh::allocate_flag();
const FlagMask fflag_color = Mesh::allocate_Face_flag();
HH_SAC_ALLOCATE_FUNC(Mesh::MFace, Pixel, f_color);

const FlagMask mflag_v_colors = Mesh::allocate_flag();
const FlagMask vflag_color = Mesh::allocate_Vertex_flag();
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Pixel, v_color);

const FlagMask mflag_c_colors = Mesh::allocate_flag();
const FlagMask vflag_c_color = Mesh::allocate_Vertex_flag();
HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, Pixel, c_color);

const FlagMask vflag_unique_nors = Mesh::allocate_Vertex_flag();  // v_lod.{Onor,Nnor}
HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, Vector, c_nor);

const FlagMask mflag_uv = Mesh::allocate_flag();
HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, UV, c_uv);

const Pixel k_color_invalid{0xCD, 0xAB, 0xFF, 0x00};
bool lmcad;    // lmcolor() state
Pixel curcol;  // current color (for lines and points)
Color matcol;  // material color (for polygons)

// default color for polygons  (was (.9f, .6f, .4f) before 20020117
// const A3dVertexColor k_default_color{A3dColor(.8f, .5f, .4f), A3dColor(.5f, .5f, .5f), A3dColor(4.f, 0.f, 0.f)};
const A3dVertexColor k_default_color{A3dColor(.6f, .6f, .6f), A3dColor(.5f, .5f, .5f), A3dColor(4.f, 0.f, 0.f)};

// default color for polylines and points  (was 1.f, 1.f, 1.f,  0.f, 0.f, 0.f,  1.f, 0.f, 0.f)
const A3dVertexColor k_default_poly_color{A3dColor(0.f, 0.f, 0.f), A3dColor(0.f, 0.f, 0.f), A3dColor(1.f, 0.f, 0.f)};

// default color for mesh polygons (now defined in meshcold, meshcols, meshcolp, meshca)
// const A3dVertexColor MESHCOL{A3dColor(.8f, .5f, .4f), A3dColor(.5f, .5f, .5f), A3dColor(4.f, 0.f, 0.f)};
Color meshcolor;
Color cuspcolor;

struct Node : noncopyable {
  virtual ~Node() = default;
  enum class EType { polygon, line, point };
  EType _type;  // faster than virtual function or dynamic_cast() or hh::dynamic_exact_cast(), and avoids RTTI
 protected:
  explicit Node(EType type) : _type(type) {}
};

struct NodePolygon : Node {
  NodePolygon() : Node(Node::EType::polygon) {}
  Polygon poly;
  Vector pnor;
  Array<Vector> vnors;
  Array<Color> colors;
};

struct NodeLine : Node {
  NodeLine() : Node(Node::EType::line) {}
  Array<Point> pa;
  Array<Pixel> colors;
};

struct NodePoint : Node {
  NodePoint() : Node(Node::EType::point) {}
  Point p;
  Pixel color;
};

struct Texture {
  GLuint texname;
};

Array<Texture> g_textures;

class GXobject {
 public:
  ~GXobject() { assertx(!_opened); }
  void open(bool todraw) {
    assertx(!_opened);
    _opened = true;
    s_idraw = todraw;
  }
  void add(const A3dElem& el);
  void close();
  void morph(float finterp);
  CArrayView<unique_ptr<Node>> traverse() const {
    assertx(!_opened);
    return _arn;
  }
  GMesh* pmesh{nullptr};

 private:
  bool _opened{false};
  Array<unique_ptr<Node>> _arn;
  static bool s_idraw;  // draw elements as they are added
  void append(unique_ptr<Node> n);
};

class GXobjects {
 public:
  GXobjects();
  Vec<Frame, k_max_object> t;
  Vec<bool, k_max_object> vis;
  Vec<bool, k_max_object> cullface;      // backface culling (polygon normal)
  Vec<bool, k_max_object> reverse_cull;  // reverse direction of culling
  Vec<bool, k_max_object> shading;       // shade the polygons
  Vec<bool, k_max_object> smooth;        // compute normals at vertices
  Vec<bool, k_max_object> edges;         // show polygon edges
  int min_segn() const { return _imin; }
  int max_segn() const { return _imax; }
  void clear(int segn);
  void open(int segn);
  void add(const A3dElem& el) {
    assertx(_segn != -1);
    _ob[_segn]->add(el);
  }
  void close() {
    assertx(_segn != -1);
    _ob[_segn]->close();
    _segn = -1;
  }
  void make_link(int oldsegn, int newsegn);
  bool defined(int segn) const { return !!obp(segn); }  // recursion on links not permitted
  GXobject& operator[](int i) { return *assertx(obp(i)); }

 private:
  int _imin{k_max_object};
  int _imax{0};
  // if _link[i], is a link to GXobject _link[i]
  Vec<int, k_max_object> _link;
  Vec<unique_ptr<GXobject>, k_max_object> _ob;
  int _segn{-1};
  GXobject* obp(int i) const;
};

bool GXobject::s_idraw;

GXobjects g_xobs;

// *** PM stuff (progressive mesh)

bool pm_mode;
#if defined(DEF_PM)
void read_pm(const string& filename);
void draw_pm();
void pm_wrap_draw(bool show);
void pm_set_lod(float lod);
bool pm_key_press(char ch);
#endif

// *** SR stuff (selective refinement)

bool sr_mode;
#if defined(DEF_SR)
void read_sr(const string& filename);
void draw_sr();
void sr_wrap_draw(bool show);
void sr_pre_space();
bool sr_key_press(char ch);
float sr_screen_thresh = 0.005f;
float sr_regulatenf = 0.f;  // could be int except for sliders
// Observation: sr_gain should be set approximately equal to sr_fracvtrav.
// The fact that they should be proportional is intuitive: the number of frames necessary to visit
//  all active vertices is  1.f / sr_fractrav; therefore the regulator factor should be modified
//  as pow(factor, sr_fractrav) in order to obtain the same affect.
// The fact that they have the same scale is likely luck and platform-dependent.
float sr_fracvtrav = 1.f;
float sr_gain = 1.f;
int sr_gtime = 32;
#endif

// *** SC stuff

bool sc_mode;
bool psc_mode;
bool sc_gm_mode;
bool hpix;
#if defined(DEF_SC)
void read_sc(const string& filename);
void read_psc(const string& filename);
void draw_sc();
void psc_wrap_draw(bool show);
void psc_set_lod(float lod);
// geomorph
void read_sc_gm(const string& filename);
void sc_gm_wrap_draw(bool show);
void sc_gm_set_lod(float lod);
void draw_sc_gm(const SimplicialComplex& kmesh);
Vec<SCGeomorph, 20> Gmorphs;  // has to be here because of draw_all
int sc_gm_morph = 0;
int sc_gm_num = -1;
bool psc_key_press(char ch);
#endif

// *** PLY stuff

bool ply_mode;
#if defined(DEF_PLY)
void read_ply(const string& filename);
void draw_ply();
#endif

// *** lighting

Pixel pack_color(const A3dColor& col) { return Vector4(col[0], col[1], col[2], 1.f).pixel(); }

// unlit colors

void initialize_unlit() {
  if (!lmcad) return;
  lmcad = false;
  glDisable(GL_LIGHTING);
  glDisable(GL_COLOR_MATERIAL);
  assertx(curcol == k_color_invalid);
  matcol.d = k_color_invalid;
  matcol.s = k_color_invalid;
  matcol.g = -1;
}

void update_cur_color2(const Pixel& col) {
  assertx(!lmcad);
  curcol = col;
  glColor4ubv(col.data());
}

inline void update_cur_color(const Pixel& col) {
  if (col != curcol || force_color_update) update_cur_color2(col);
}

// lit colors

void initialize_lit() {
  if (lmcad) return;
  lmcad = true;
  glEnable(GL_LIGHTING);
  glColorMaterial(g_twosided ? GL_FRONT_AND_BACK : GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
  assertx(matcol.d == k_color_invalid);
  assertx(matcol.s == k_color_invalid);
  assertx(matcol.g == -1);
  curcol = k_color_invalid;
}

inline void fast_update_mat_diffuse(const Pixel& cd) {
  matcol.d = cd;
  glColor4ubv(matcol.d.data());
}

void update_mat_color2(const Color& col) {
  assertx(lmcad);
  if (col.g != matcol.g) {
    matcol.g = col.g;
    float shininess = min(matcol.g, 128.f);  // OpenGL limit is 128.
    glMaterialf(g_twosided ? GL_FRONT_AND_BACK : GL_FRONT, GL_SHININESS, shininess);
  }
  if (col.s != matcol.s) {
    matcol.s = col.s;
    glMaterialfv(g_twosided ? GL_FRONT_AND_BACK : GL_FRONT, GL_SPECULAR, Vector4(matcol.s).data());
  }
  if (col.d != matcol.d) fast_update_mat_diffuse(col.d);
  curcol = k_color_invalid;
}

inline void update_mat_color(const Color& col) {
  if (col.d != matcol.d || col.s != matcol.s || col.g != matcol.g) update_mat_color2(col);
}

inline void maybe_update_mat_diffuse(const Pixel& cd) {
  if (cd != matcol.d || force_color_update) fast_update_mat_diffuse(cd);
}

// *** normal mapping

NormalMapping* pnormalmap = nullptr;

bool normalmap_init() {
  if (pnormalmap) return true;
  pnormalmap = NormalMapping::get();
  if (!pnormalmap) return false;
  pnormalmap->init();
  if (pnormalmap->name() == "dot3") {
    Warning("Resorting to 'dot3' for normal-mapping");
    if (1) {  // make material brighter
      uint8_t meshca = meshcolor.d[3];
      A3dColor material;
      for_int(c, 3) material[c] = meshcolor.d[c] / 255.f;
      float maxv = assertx(max(material));
      material /= maxv;
      meshcolor.d = pack_color(material);
      meshcolor.d[3] = meshca;
    }
  }
  if (0) SHOW(pnormalmap->name());
  return true;
}

void normalmap_setlight(const Vector& lightdirmodel, const Vector& eyedirmodel, float lambient) {
  assertx(pnormalmap)->set_parameters(lightdirmodel, eyedirmodel, lambient, lightsource, meshcolor.s);
}

void normalmap_activate() { assertx(pnormalmap)->activate(); }

void normalmap_deactivate() { assertx(pnormalmap)->deactivate(); }

// ***

bool bigfont() { return HB::get_font_dims()[1] > 9; }

void invalidate_dls() { svalid_dl.clear(); }

inline Vector interp_normal(const Vector& n1, const Vector& n2, float f1, float f2) {
  float n1x = n1[0], n1y = n1[1], n1z = n1[2];
  float n2x = n2[0], n2y = n2[1], n2z = n2[2];
  float nx, ny, nz;
  if (n1x == n2x && n1y == n2y && n1z == n2z) {
    nx = n1x;
    ny = n1y;
    nz = n1z;
  } else {
    nx = f1 * n1[0] + f2 * n2[0];
    ny = f1 * n1[1] + f2 * n2[1];
    nz = f1 * n1[2] + f2 * n2[2];
    float denom = sqrt(nx * nx + ny * ny + nz * nz);
    if (denom) {
      nx /= denom;
      ny /= denom;
      nz /= denom;
    }
  }
  return Vector(nx, ny, nz);
}

inline Pixel interp_color(const Pixel& c1, const Pixel& c2, int f1, int f2) {
  ASSERTX(f1 + f2 == 256);
  return ((Vector4i(c1) * f1 + Vector4i(c2) * f2 + 128) >> 8).pixel();
}

// ***

void do_movie(Args& args) {
  picture = true;
  if (!g3d::override_frametime) g3d::override_frametime = 1.f / 60.f;  // 60fps
  movie_nframes = args.get_int();
  assertx(movie_nframes >= 2);
  movie_rootname = args.get_filename();
  movie_cprogress = make_unique<ConsoleProgress>();
  movie_desire_video = false;
  if (0) use_dl = false;
}

// G3dOGL -video 360 movie.mp4 -noinfo 1 -geom 800x800 -key iiJDm ~/data/mesh/cat.m
void do_video(Args& args) {
  picture = true;
  if (!g3d::override_frametime) g3d::override_frametime = 1.f / 60.f;  // 60fps
  movie_nframes = args.get_int();
  assertx(movie_nframes >= 2);
  movie_rootname = args.get_filename();
  movie_cprogress = make_unique<ConsoleProgress>();
  movie_desire_video = true;
  if (0) use_dl = false;
}

void do_texturemap(Args& args) {
  const string filename = args.get_filename();
  texturemaps.push(filename);
}

// *** HW callback functions

bool DerivedHW::key_press(string s) { return fkeyp(s); }

void DerivedHW::button_press(int butnum, bool pressed, const Vec2<int>& yx) {
  Vec2<float> yxf = convert<float>(yx) / convert<float>(win_dims);
  if (pressed) yx_pointer_old = yxf;
  bool shift = get_key_modifier(EModifier::shift);
  bool in_slider = (slidermode && !(pm_mode && yx_pointer_old[1] >= k_one_slider_left_thresh) &&
                    !(psc_mode && yx_pointer_old[1] >= k_one_slider_left_thresh) &&
                    !(sc_gm_mode && yx_pointer_old[1] >= k_one_slider_left_thresh));
  if (pressed) {
    was_using_dl = use_dl;
    if (in_slider) {
      hw.redraw_later();
      use_dl = false;
    } else {
      fbutp(butnum, pressed, shift, yxf);
    }
    if (butnum <= 3) button_active = butnum;
  } else {
    if (in_slider) {
      use_dl = was_using_dl;
      was_using_dl = false;
      if (g3d::output) {
        std::cout << "lod "
                  << "-1\n";
        std::cout.flush();
      }
    } else {
      fbutp(butnum, pressed, shift, yxf);
    }
    button_active = 0;
  }
  if (butquick) {
    invalidate_dls();
    hw.redraw_now();
  }
}

void DerivedHW::wheel_turn(float v) { fwheel(v); }

void DerivedHW::draw_window(const Vec2<int>& dims) {
  is_window = true;
  win_dims = dims;
  fdraw();
}

int g_pthick;

void reset_thickness() { g_pthick = -1; }

void set_thickness2(int vthick) {
  g_pthick = vthick;
  if (vthick > 1) {
    if (antialiasing) glDisable(GL_LINE_SMOOTH);
    glLineWidth(float(g_pthick));
  } else {
    if (antialiasing) glEnable(GL_LINE_SMOOTH);
    glLineWidth(float(max(g_pthick, 1)));
  }
}

inline void set_thickness(int vthick) {
  if (vthick != g_pthick) set_thickness2(vthick);
}

void space_init() {
  if (yonder <= hither * 1.0001f) yonder = hither * 1.0001f;
  lquickmode = quickmode || (butquick && button_active);
  tzp = twice(.5f) / (view_zoom * convert<float>(win_dims) / float(min(win_dims)));
  const float cedgeoffset = 4.2e-3f;
  edgeoffset = cedgeoffset * view_zoom * fdisplacepolygon;
  dummy_use(edgeoffset);
}

// Find largest texture max_yx with specified internal format and aspect
Vec2<int> find_max_texture(GLenum internal_format, const Vec2<int>& yx_aspect, int border, int max_mipmap_level) {
  Vec2<int> max_yx = twice(0);
  assertx(!gl_report_errors());
  Vec2<int> yx;
  int level_offset;
  {
    if (yx_aspect[1] > yx_aspect[0]) {
      yx = V(1, yx_aspect[1] / yx_aspect[0]);
      level_offset = int_floor_log2(yx[1]);
    } else {
      yx = V(yx_aspect[0] / yx_aspect[1], 1);
      level_offset = int_floor_log2(yx[0]);
    }
  }
  for (int level = 0;;) {
    if (level) {
      // First check to see if can allocate non-mipmap of same size
      Vec2<int> yxt = (yx * (1 << level)) + 2 * border;
      glTexImage2D(GL_PROXY_TEXTURE_2D, 0, internal_format, yxt[1], yxt[0], border, GL_RGBA, GL_UNSIGNED_BYTE,
                   nullptr);
      if (glGetError() != GL_NO_ERROR) break;
      int w;
      glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &w);
      if (glGetError() != GL_NO_ERROR) break;
      if (!w) break;
    }
    {
      Vec2<int> yxt = yx + 2 * border;
      glTexImage2D(GL_PROXY_TEXTURE_2D, level, internal_format, yxt[1], yxt[0], border, GL_RGBA, GL_UNSIGNED_BYTE,
                   nullptr);
    }
    if (glGetError() != GL_NO_ERROR) break;
    int w;
    glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, level, GL_TEXTURE_WIDTH, &w);
    if (glGetError() != GL_NO_ERROR) break;
    static const bool debug = getenv_bool("TEXTURE_MAX_DEBUG");
    if (debug) SHOW("try", level, internal_format, yx, border, w);
    if (!w) break;  // proxy allocation failed
    if (level >= 16) {
      // Bug on squeal (Impact).
      SHOW("level > 16");
      break;
    }
    max_yx = (yx * (1 << level)) + 2 * border;
    if (max_mipmap_level == 0) {
      // try the next larger image size at level zero
      yx *= 2;
    } else {
      // try same image size at next higher mipmap level
      level++;
      if (level + level_offset > max_mipmap_level) break;
    }
  }
  if (product(max_yx)) {
    // Apply the successful one again in case the caller wants to get
    //  information on internal storage format (r/g/b/a).
    glTexImage2D(GL_PROXY_TEXTURE_2D, 0, internal_format, max_yx[1], max_yx[0], border, GL_RGBA, GL_UNSIGNED_BYTE,
                 nullptr);
    int w;
    glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &w);
    assertx(w);  // else could set: max_yx = twice(0);
  }
  assertx(!gl_report_errors());
  return max_yx;
}

struct glt_format {
  GLenum format;
  string name;
};

const Array<glt_format> k_glt_formats = {
#define E(f) \
  { f, #f }
    E(GL_COMPRESSED_RGB),
    E(GL_RGB16),
    E(GL_RGBA16),
    E(GL_LUMINANCE16),
    E(GL_RGB10),
    E(GL_RGB10_A2),
    E(GL_RGB),
    E(GL_RGB8),
    E(GL_RGBA8),
    E(GL_LUMINANCE),
    E(GL_LUMINANCE_ALPHA),
    E(GL_RGB5),
    E(GL_RGB5_A1),
    E(GL_RGB4),
    E(GL_RGBA4),
    E(GL_LUMINANCE4),
    E(GL_RGBA_FLOAT32_ATI),
    E(GL_RGB_FLOAT32_ATI),
    E(GL_RGBA_FLOAT16_ATI),
    E(GL_RGB_FLOAT16_ATI),
#undef E
};

// On PC/GeForce3Quadro3 2001-10-17
// GL_MAX_TEXTURE_SIZE: 4096
// GL_RGB16             1:1 (4096x4096) [8,8,8,0] mipmap
// GL_RGB10             1:1 (4096x4096) [8,8,8,0] mipmap
// GL_RGB               1:1 (4096x4096) [8,8,8,0] mipmap
// GL_RGBA16            1:1 (4096x4096) [8,8,8,8] mipmap
// GL_RGB10_A2          1:1 (4096x4096) [8,8,8,8] mipmap
// GL_RGBA8             1:1 (4096x4096) [8,8,8,8] mipmap
// GL_LUMINANCE         1:1 (4096x4096) [0,0,0,0] mipmap
// GL_LUMINANCE4        1:1 (4096x4096) [0,0,0,0] mipmap
// GL_RGBA16            1:1 (4096x4096) [0,0,0,0] mipmap
// GL_LUMINANCE16       1:1 (4096x4096) [0,0,0,0] mipmap
// GL_LUMINANCE_ALPHA   1:1 (4096x4096) [0,0,0,8] mipmap
// GL_RGB5              1:1 (4096x4096) [5,6,5,0] mipmap
// GL_RGB4              1:1 (4096x4096) [5,6,5,0] mipmap
// GL_RGB5_A1           1:1 (4096x4096) [5,5,5,1] mipmap
// GL_RGBA4             1:1 (4096x4096) [4,4,4,4] mipmap
// GL_RGB_FLOAT32_ATI   1:1 (   0x   0) [0,0,0,0] mipmap
// GL_RGB_FLOAT16_ATI   1:1 (   0x   0) [0,0,0,0] mipmap

// On PC/GeForce4Ti4200 2003-07-22
// GL_MAX_TEXTURE_SIZE: 4096
// GL_COMPRESSED_RGB    1:1 (4096x4096) [5,5,5,1] comp mipmap
// GL_RGB16             1:1 (4096x4096) [8,8,8,0] mipmap
// GL_RGBA16            1:1 (4096x4096) [8,8,8,8] mipmap
// GL_LUMINANCE16       1:1 (4096x4096) [0,0,0,0] mipmap
// GL_RGB10             1:1 (4096x4096) [8,8,8,0] mipmap
// GL_RGB10_A2          1:1 (4096x4096) [8,8,8,8] mipmap
// GL_RGB               1:1 (4096x4096) [8,8,8,0] mipmap
// GL_RGB8              1:1 (4096x4096) [8,8,8,0] mipmap
// GL_RGBA8             1:1 (4096x4096) [8,8,8,8] mipmap
// GL_LUMINANCE         1:1 (4096x4096) [0,0,0,0] mipmap
// GL_LUMINANCE_ALPHA   1:1 (4096x4096) [0,0,0,8] mipmap
// GL_RGB5              1:1 (4096x4096) [5,6,5,0] mipmap
// GL_RGB5_A1           1:1 (4096x4096) [5,5,5,1] mipmap
// GL_RGB4              1:1 (4096x4096) [5,6,5,0] mipmap
// GL_RGBA4             1:1 (4096x4096) [4,4,4,4] mipmap
// GL_LUMINANCE4        1:1 (4096x4096) [0,0,0,0] mipmap
// GL_RGB_FLOAT32_ATI   1:1 (   0x   0) [0,0,0,0] mipmap
// GL_RGB_FLOAT16_ATI   1:1 (   0x   0) [0,0,0,0] mipmap

// On PC/Radeon9700Pro 2003-07-21
// GL_MAX_TEXTURE_SIZE: 2048
// GL_COMPRESSED_RGB    1:1 (2048x2048) [5,6,5,0] mipmap
// GL_RGB16             1:1 (2048x2048) [16,16,16,16] mipmap
// GL_LUMINANCE16       1:1 (2048x2048) [0,0,0,0] mipmap
// GL_RGB10             1:1 (2048x2048) [10,10,10,2] mipmap
// GL_RGB               1:1 (2048x2048) [5,6,5,0] mipmap
// GL_RGB8              1:1 (2048x2048) [8,8,8,8] mipmap
// GL_RGBA8             1:1 (2048x2048) [4,4,4,4] mipmap
// GL_LUMINANCE         1:1 (2048x2048) [0,0,0,0] mipmap
// GL_LUMINANCE_ALPHA   1:1 (2048x2048) [0,0,0,8] mipmap
// GL_RGB5              1:1 (2048x2048) [5,6,5,0] mipmap
// GL_RGB5_A1           1:1 (2048x2048) [5,5,5,1] mipmap
// GL_RGB4              1:1 (2048x2048) [5,6,5,0] mipmap
// GL_RGBA4             1:1 (2048x2048) [4,4,4,4] mipmap
// GL_LUMINANCE4        1:1 (2048x2048) [0,0,0,0] mipmap
// GL_RGB_FLOAT32_ATI   1:1 (2048x2048) [32,32,32,32] mipmap
// GL_RGB_FLOAT16_ATI   1:1 (2048x2048) [16,16,16,16] mipmap

// On PC/RemoteDesktop (GeF4ti4200 over Radeon9700Pro) 2003-07-21
// On PC/RemoteDesktop (Radeon9700Pro over Quadro3) 2003-07-21
// GL_MAX_TEXTURE_SIZE: 1024
// GL_COMPRESSED_RGB    1:1 (   0x   0) [0,0,0,0] mipmap
// GL_RGB16             1:1 (1024x1024) [24,24,24,0] mipmap
// GL_RGBA16            1:1 (1024x1024) [24,24,24,24] mipmap
// GL_LUMINANCE16       1:1 (1024x1024) [0,0,0,0] mipmap
// GL_RGB10             1:1 (1024x1024) [24,24,24,0] mipmap
// GL_RGB10_A2          1:1 (1024x1024) [24,24,24,24] mipmap
// GL_RGB               1:1 (1024x1024) [8,8,8,0] mipmap
// GL_RGB8              1:1 (1024x1024) [8,8,8,0] mipmap
// GL_RGBA8             1:1 (1024x1024) [8,8,8,8] mipmap
// GL_LUMINANCE         1:1 (1024x1024) [0,0,0,0] mipmap
// GL_LUMINANCE_ALPHA   1:1 (1024x1024) [0,0,0,24] mipmap
// GL_RGB5              1:1 (1024x1024) [8,8,8,0] mipmap
// GL_RGB5_A1           1:1 (1024x1024) [8,8,8,8] mipmap
// GL_RGB4              1:1 (1024x1024) [8,8,8,0] mipmap
// GL_RGBA4             1:1 (1024x1024) [8,8,8,8] mipmap
// GL_LUMINANCE4        1:1 (1024x1024) [0,0,0,0] mipmap
// GL_RGB_FLOAT32_ATI   1:1 (   0x   0) [0,0,0,0] mipmap
// GL_RGB_FLOAT16_ATI   1:1 (   0x   0) [0,0,0,0] mipmap

void display_texture_size_info() {
  const bool detailed = true;
  assertx(!gl_report_errors());
  if (1) {
    GLint max_texture_size;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture_size);
    showf("GL_MAX_TEXTURE_SIZE: %d\n", max_texture_size);
  }
  for (const auto& glt_format : k_glt_formats) {
    for_intL(mm, 1 - detailed, 2) {
      for_intL(aspx, 1, 1 + 3 * detailed + 1) {
        if (aspx == 3) continue;  // always fails to allocate
        if (aspx == 4) continue;  // always same size as aspx == 2
        Vec2<int> aspyx(1, aspx);
        int border = 0;
        Vec2<int> max_yx = find_max_texture(glt_format.format, aspyx, border, (mm ? INT_MAX : 0));
        int r = 0, g = 0, b = 0, a = 0, comp = 0;
        glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_RED_SIZE, &r);
        glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_GREEN_SIZE, &g);
        glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_BLUE_SIZE, &b);
        glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_ALPHA_SIZE, &a);
        if (1) glGetTexLevelParameteriv(GL_PROXY_TEXTURE_2D, 0, GL_TEXTURE_COMPRESSED, &comp);
        glGetError();  // ignore any error
        showf("%-20s %d:%d (%4dx%4d) [%d,%d,%d,%d]%s%s\n", glt_format.name.c_str(), aspyx[1], aspyx[0], max_yx[1],
              max_yx[0], r, g, b, a, (comp ? " comp" : ""), (mm ? " mipmap" : ""));
      }
    }
    if (detailed) showf("\n");
  }
}

void set_anisotropy() {
  static const bool debug = getenv_bool("OPENGL_DEBUG");
  if (!contains(gl_extensions_string(), "GL_EXT_texture_filter_anisotropic")) {
    if (debug) Warning("No anisotropic extension");
    anisotropy = 1;
    return;
  }
  {
    float max_anisotropy;
    glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_anisotropy);
    if (debug) SHOW(max_anisotropy);
    if (anisotropy > max_anisotropy) anisotropy = int(max_anisotropy);
  }
  {
    Vec1<float> ani;
    ani[0] = float(anisotropy);
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, ani.data());
  }
}

void update_anisotropy() {
  float max_anisotropy;
  { glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_anisotropy); }
  anisotropy *= 2;
  if (anisotropy > max_anisotropy) anisotropy = 1;
  set_anisotropy();
  SHOW(anisotropy);
}

void set_light_ambient(float lambient) {
  Vec4<float> a{lambient, lambient, lambient, 1.f};
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, a.data());
}

void load_texturemaps() {
  static const bool debug = getenv_bool("TEXTURE_DEBUG");
  if (debug) display_texture_size_info();
  // HH_TIMER(_load_texturemap);
  textures_loaded = true;
  if (!texturemaps.num()) {
    string name = g3d::g_filename;
    if (name == "") {
      Warning("No name for texture map");
      texture_active = false;
      return;
    }
    remove_at_end(name, ".gz");
    remove_at_end(name, ".pm");
    remove_at_end(name, ".s3d");
    remove_at_end(name, ".m");
    remove_at_end(name, ".obj");
    if (contains(name, ".nf")) name.erase(name.find(".nf"));
    name = replace_all(name, "Mesh-", "Atlas-");  // Kent data
    const Array<string> exts = {"nor.bmp", "nor.jpg", "nor.ppm", "nor.rgb", "nor.png",
                                "bmp",     "jpg",     "ppm",     "rgb",     "png"};
    string s;
    for (const auto& ext : exts) {
      s = name + "." + ext;
      if (file_exists(s)) break;
      s.clear();
    }
    if (s == "") {
      Warning("Texture map file not found");
      texture_active = false;
      return;
    }
    if (contains(s, ".nor")) texturenormal = true;
    texturemaps.push(s);
  }
  assertx(!g_textures.num());
  g_textures.init(texturemaps.num());
  for_int(i, texturemaps.num()) {
    if (debug) SHOW("defining texture", i);
    glGenTextures(1, &g_textures[i].texname);
    glBindTexture(GL_TEXTURE_2D, g_textures[i].texname);
    Image itexture;
    const string filename = texturemaps[i];
    itexture.read_file(filename);
    if (1) itexture.reverse_y();  // because glTexImage2D() has image origin at lower-left
    int orig_xsize = itexture.xsize(), orig_ysize = itexture.ysize();
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    // On ATI, default GL_RGB is in fact GL_RGB5.
    GLenum internal_format = GL_RGBA8;  // was GL_RGB8
    bool define_mipmap = true;
    if (getenv_bool("NO_MIPMAP")) {
      showdf("NO_MIPMAP\n");
      define_mipmap = false;
    }
    if (itexture.zsize() == 4) internal_format = GL_RGBA8;
    if (1) {
      int max_texture_size;
      glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture_size);
      if (max(itexture.dims()) > max_texture_size) {
        showf("texture too large (maxsize=%d), so downsampling\n", max_texture_size);
        itexture.scale(twice(min(twice(float(max_texture_size)) / convert<float>(itexture.dims()))),
                       twice(FilterBnd(Filter::get("triangle"), Bndrule::reflected)));
      }
    }
    // Use modern approach with glGenerateMipmap().
    {
      glEnable(GL_TEXTURE_2D);  // may need to come before glGenerateMipmap on old AMD drivers
      const int level = 0, border = 0;
      glTexImage2D(GL_TEXTURE_2D, level, internal_format, itexture.xsize(), itexture.ysize(), border, GL_RGBA,
                   GL_UNSIGNED_BYTE, itexture.data());
      USE_GL_EXT_MAYBE(glGenerateMipmap, PFNGLGENERATEMIPMAPPROC);
      if (glGenerateMipmap && define_mipmap) {
        glGenerateMipmap(GL_TEXTURE_2D);  // not supported on Remote Desktop
      }
    }
    {
      int w, h, r, g, b, a;
      glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &w);
      glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &h);
      glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_RED_SIZE, &r);
      glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_GREEN_SIZE, &g);
      glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_BLUE_SIZE, &b);
      glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_ALPHA_SIZE, &a);
      showf("Loaded texture '%s' (%dx%d) [%d,%d,%d,%d]\n", filename.c_str(), w, h, r, g, b, a);
    }
    {
      // GL_REPLACE is the same as GL_DECAL except it does the "right thing" on texture with alphas.
      glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
      texture_lit = false;  // compatible with old
      if (getenv_bool("G3D_TEXTURE_LIT")) texture_lit = true;
      if (texturenormal) texture_lit = false;
      if (texture_lit) {
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        // and must later set material color to white.
        if (1) {
          // more expensive: apply specular highlight post texture.
          //  (default: GL_SINGLE_COLOR)
          glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR);
          // remember to set "-meshcols 0 0 0" if you don't want the white specular lights.
        }
      }
      {
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
        glTexGenfv(GL_S, GL_OBJECT_PLANE, V((orig_ysize > orig_xsize ? 2.f : 1.f), 0.f, 0.f, 0.f).data());
      }
      {
        glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
        glTexGenfv(GL_T, GL_OBJECT_PLANE, V(0.f, (orig_ysize < orig_xsize ? 2.f : 1.f), 0.f, 0.f).data());
      }
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);  // default
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);  // default
      glEnable(GL_ALPHA_TEST);
      glAlphaFunc(GL_GREATER, 0.0);  // discard fragments with zero alpha if partially transparent texture
      if (getenv_bool("G3D_TEX_CLAMP")) {
        Warning("G3D_TEX_CLAMP now obsolete; enabled by -texturescale 0");
      }
      if (!texturescale) {
        // showf("Setting texture clamp mode\n");
        unsigned wrap_mode = GL_CLAMP_TO_EDGE;
        if (!contains(gl_extensions_string(), "GL_EXT_texture_edge_clamp")) {
          // Warning("No texture_edge_clamp extension!");  // it could be due to Remote Desktop, or Apple
          wrap_mode = GL_CLAMP;  // (obsolete; uses border texels; seems to work though)
        }
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap_mode);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap_mode);
      }
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                      define_mipmap ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);  // default is NEAREST_MIPMAP_LINEAR
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);      // default is GL_LINEAR
      if (getenv_bool("G3D_TEX_NEAREST"))
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);  // show pixels blocks.
    }
    if (texturescale && texturescale != 1.f) {
      // Matrix4 m1; m1.ident();
      SGrid<float, 4, 4> m1;
      for_int(y, 4) for_int(x, 4) m1[y][x] = x == y ? 1.f : 0.f;
      m1[0][0] = texturescale;
      m1[1][1] = texturescale;
      glMatrixMode(GL_TEXTURE);
      glLoadMatrixf(m1.data());
      glMatrixMode(GL_MODELVIEW);
    }
    //
    anisotropy = INT_MAX;
    if (getenv_bool("NO_ANISO")) {
      showdf("NO_ANISO\n");
      anisotropy = 1;
    }
    anisotropy = getenv_int("ANISOTROPY", anisotropy);
    set_anisotropy();
    //
    if (texturenormal) {
      if (!normalmap_init()) {
        Warning("Normal mapping unsupported -> texturenormal=false");
        texturenormal = false;
      }
    }
    //
    bool texture_elev = getenv_bool("TEXTURE_ELEV");
    if (texture_elev && !contains(gl_extensions_string(), "GL_ARB_multitexture")) {
      Warning("GL_ARB_multitexture unsupported -> texture_elev=false");
      texture_elev = false;
    }
    if (texture_elev) {
      showf("Defining texture_elev\n");
      USE_GL_EXT(glActiveTexture, PFNGLACTIVETEXTUREPROC);
      GLint max_texture_units;
      glGetIntegerv(GL_MAX_TEXTURE_UNITS, &max_texture_units);
      if (0) SHOW(max_texture_units);
      assertx(max_texture_units >= 2);
      glActiveTexture(GL_TEXTURE1);
      glEnable(GL_TEXTURE_1D);
      GLuint texname1;
      glGenTextures(1, &texname1);
      glBindTexture(GL_TEXTURE_1D, texname1);
      glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
      GLenum internal_format2 = GL_RGBA8;
      if (0) {
        const int n = 128;
        int level = 0, nl = n;
        for (;;) {
          Image etexture(V(1, nl));
          for_int(x, nl) {
            uint8_t v = 255;
            if (level == 1) {
              if (x < 2)
                v = 0;
              else if (x % (nl / 4) < 1)
                v = 0;
            } else if (level == 2) {
              if (x < 1)
                v = 0;
              else if (x % (nl / 4) < 1)
                v = 128;
            } else if (level == 3) {
              if (x < 1) v = 128;
            }
            etexture[0][x] = Pixel::gray(v);
          }
          int border = 0;
          glTexImage1D(GL_TEXTURE_1D, level, internal_format2, etexture.xsize(), border, GL_RGBA, GL_UNSIGNED_BYTE,
                       etexture.data());
          if (etexture.xsize() == 1) break;
          level++;
          nl /= 2;
        }
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
        glTexGenfv(GL_S, GL_OBJECT_PLANE, V(0.f, 0.f, 1000.f, 0.f).data());
        glEnable(GL_TEXTURE_GEN_S);
      } else {
        Image etexture;
        etexture.read_file("ramp1.png");
        const int level = 0, border = 0;
        glTexImage1D(GL_TEXTURE_1D, level, internal_format2, etexture.xsize(), border, GL_RGBA, GL_UNSIGNED_BYTE,
                     etexture.data());
        USE_GL_EXT_MAYBE(glGenerateMipmap, PFNGLGENERATEMIPMAPPROC);
        if (glGenerateMipmap) {
          glGenerateMipmap(GL_TEXTURE_1D);  // not supported on Remote Desktop
        }
        // To map it to elevation (in meters) use
        //   z = 10 x - 7995
        // where z is the elevation and x is the horizontal index of the
        // texture, with the leftmost pixel being centered at x = 0.
        //
        // Horizontal spacing is 10m -> 16385 samples span 163'840m
        //  x = 799.5 + z * 0.10
        //  x' in [0, 1]:  x' = x / 1600  [shift by 0.5?]
        //  z' in world_units:  z' = z / 163840
        //  1600 * x' = 799.5 + z' * 163840 * 0.10
        //  x' = 0.5 + z' * 10.24
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
        glTexGenfv(GL_S, GL_OBJECT_PLANE, V(0.f, 0.f, 10.24f, 0.5f).data());
        glEnable(GL_TEXTURE_GEN_S);
      }
      {
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      }
      glActiveTexture(GL_TEXTURE0);
    }
  }
}

void enable_polygon_offset() {
  // (1.f, 1.f) are good default parameters
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.f, fdisplacepolygon);
}

void disable_polygon_offset() { glDisable(GL_POLYGON_OFFSET_FILL); }

// must be followed by setup_ob()!
void gl_init() {
  {  // matrices
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float a = 1.f / min(win_dims);
    if (perspec) {
      glFrustum(-win_dims[1] * a * view_zoom * hither, win_dims[1] * a * view_zoom * hither,
                -win_dims[0] * a * view_zoom * hither, win_dims[0] * a * view_zoom * hither, hither, yonder);
    } else {
      glOrtho(-win_dims[1] * a * view_zoom, win_dims[1] * a * view_zoom, -win_dims[0] * a * view_zoom,
              win_dims[0] * a * view_zoom, hither, yonder);
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
  }
  {  // materials
    if (!g_twosided) {
      glMaterialfv(GL_BACK, GL_AMBIENT, V(0.f, 0.f, 0.f, 1.f).data());
      glMaterialfv(GL_BACK, GL_DIFFUSE, V(.15f, .15f, .15f, 1.f).data());
      glMaterialfv(GL_BACK, GL_EMISSION, concat(backfacec, V(1.f)).data());
    }
    lmcad = false;
    curcol = k_color_invalid;
    matcol.d = k_color_invalid;
    matcol.s = k_color_invalid;
    matcol.g = -1;  // phong == -1 is different
  }
  {  // lighting
    if (1) {
      Vector lightdireyegl = k_lightdir_eye0 * k_eye_to_gleye;
      glLightfv(GL_LIGHT0, GL_POSITION, concat(lightdireyegl, V(0.f)).data());  // directional; current matrix
      // ambient color of light is (0, 0, 0, 1) by default  (!= scene_ambient)
      Vec4<float> color = concat(thrice(lightsource), V(1.f));
      glLightfv(GL_LIGHT0, GL_DIFFUSE, color.data());
      glLightfv(GL_LIGHT0, GL_SPECULAR, color.data());
      glEnable(GL_LIGHT0);
    }
    if (g_twolights) {
      Vector lightdireyegl = k_lightdir_eye1 * k_eye_to_gleye;
      glLightfv(GL_LIGHT1, GL_POSITION, concat(lightdireyegl, V(0.f)).data());
      Vec4<float> color = concat(thrice(lightsource), V(1.f));
      if (g_twolights > 1) {
        // make rear light blue
        color[0] *= 0.5f;
        color[1] *= 0.7f;
        color[2] *= 1.5f;
      }
      glLightfv(GL_LIGHT1, GL_DIFFUSE, color.data());
      glLightfv(GL_LIGHT1, GL_SPECULAR, color.data());
      glEnable(GL_LIGHT1);
    }
  }
  {  // misc
    glPointSize(thickpoint);
    glEnable(GL_DEPTH_TEST);
    if (antialiasing) {
      glEnable(GL_POINT_SMOOTH);
      // glDisable(GL_BLEND);  // only enable where drawing lines/points
      // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    } else {
      glDisable(GL_POINT_SMOOTH);
      // glDisable(GL_BLEND);
    }
    // GL_NICEST looks worse for lines in SRview of scaled bunny
    //  and makes it run much slower.  GL_FASTEST is always fast and good.
    glHint(GL_LINE_SMOOTH_HINT, nice_rendering ? GL_NICEST : GL_FASTEST);
    glHint(GL_POINT_SMOOTH_HINT, nice_rendering ? GL_NICEST : GL_FASTEST);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, nice_rendering ? GL_NICEST : GL_FASTEST);
    glHint(GL_FOG_HINT, nice_rendering ? GL_NICEST : GL_FASTEST);
    if (nice_rendering || ogl_normalize)
      glEnable(GL_NORMALIZE);
    else
      glDisable(GL_NORMALIZE);
    if (mdepthcue) {
      glEnable(GL_FOG);
      glFogi(GL_FOG_MODE, GL_EXP);   // GL_EXP is default
      glFogf(GL_FOG_DENSITY, 1.0f);  // 1.0f is default
      glFogf(GL_FOG_START, 0.0f);    // 0.0f is default
      glFogf(GL_FOG_END, 1.0f);      // 1.0f is default
      Vec4<float> fogcolor;
      glGetFloatv(GL_COLOR_CLEAR_VALUE, fogcolor.data());
      glFogfv(GL_FOG_COLOR, fogcolor.data());
    }
    reset_thickness();
    set_thickness(thicka3d);
    if (num_concave) Warning("Have concave polygons");
  }
  if (texture_active && !textures_loaded) load_texturemaps();
  static const bool depthfunc_lequal = getenv_bool("DEPTHFUNC_LEQUAL");
  if (depthfunc_lequal) glDepthFunc(GL_LEQUAL);
}

void gl_fixup() {
  if (1 || texture_active) {
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_TEXTURE_GEN_S);
    glDisable(GL_TEXTURE_GEN_T);
  }
  glPointSize(1.f);
  set_thickness(1);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // glOrtho(-.5, win_dims[1]-.5, win_dims[0]-.5, -.5, -1., 1.);  // reverse y
  glOrtho(0., win_dims[1] - 0., win_dims[0] - 0., 0., -1., 1.);  // reverse y
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  // glDisable(GL_BLEND);
  // glBlendFunc(GL_ONE, GL_ZERO);
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_POINT_SMOOTH);
  glDisable(GL_FOG);
  glDisable(GL_CULL_FACE);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  disable_polygon_offset();
  glShadeModel(GL_FLAT);
  hw.set_color_to_foreground();  // undo color changes
  if (0) {
    // Try disabling multisample to improve text font
    // - it does not improve text rendering
    // - it would have to be re-enabled in gl_init();
    // The problem for text is in the MULTISAMPLE=(3, 5) settings only
    //  (the NVIDIA extension which looks at neighboring subsamples).
    // -> set MULTISAMPLE=4 by default.
    glDisable(GL_MULTISAMPLE);
  }
}

bool setup_ob(int i) {
  // tcur = g_xobs.t[i]*tcami;
  // my +x -> GL -z
  // my +y -> GL -x
  // my +z -> GL +y
  Frame fmodeltoworld = g_xobs.t[i];
  static const bool g3d_radar = getenv_bool("G3D_RADAR");
  if (!i && g3d_radar) {
    // scale view_object so if frustum it looks good (for radar view).
    fmodeltoworld.v(1) *= real_zoom * float(win_dims[1]) / min(win_dims);
    fmodeltoworld.v(2) *= real_zoom * float(win_dims[0]) / min(win_dims);
  }
  Frame fworldtomodel = ~fmodeltoworld;
  Frame fmodeltoeye = fmodeltoworld * tcami;
  Frame fmodeltoeyegl = fmodeltoeye * k_eye_to_gleye;
  {
    SGrid<float, 4, 4> m1 = to_Matrix(fmodeltoeyegl);
    glLoadMatrixf(m1.data());
  }
  // GL looks at orthonormality of f automatically (see nmode())
  feyetomodel = tcam * fworldtomodel;
  // Note: fpostomodel = inverse(fmodeltoworld*tposi);
  // fpostomodel = tpos*fworldtomodel;
  // set object attributes
  lshading = g_xobs.shading[i];
  lsmooth = g_xobs.smooth[i];
  ledges = g_xobs.edges[i];
  if (!lshading && ledges) lsmooth = false;  // cheaper
  bool twoside = !g_xobs.cullface[i] || g_xobs.reverse_cull[i];
  float lambient = ambient;
  if (i == 0 && worldlight) lambient = 1.f;
  set_light_ambient(lambient);
  {
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, nice_rendering ? 1 : 0);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, twoside);
  }
  if (worldlight) {
    // rebind using current model-view matrix
    if (1) {
      Vector lightdireyegl = k_lightdir_eye0 * k_eye_to_gleye;
      glLightfv(GL_LIGHT0, GL_POSITION, concat(lightdireyegl, V(0.f)).data());  // directional
    }
    if (g_twolights) {
      Vector lightdireyegl = k_lightdir_eye1 * k_eye_to_gleye;
      glLightfv(GL_LIGHT1, GL_POSITION, concat(lightdireyegl, V(0.f)).data());  // directional
    }
  }
  {
    lcullface = g_xobs.cullface[i];
    if (g_xobs.cullface[i]) {
      glEnable(GL_CULL_FACE);
      glCullFace(!g_xobs.reverse_cull[i] ? GL_BACK : GL_FRONT);
    } else {
      glDisable(GL_CULL_FACE);
    }
  }
  glPolygonMode(GL_FRONT_AND_BACK, lshading ? GL_FILL : GL_LINE);
  static const bool all_textured = getenv_bool("G3D_ALL_TEXTURED");
  const int texture_id = i - 1;
  if (texture_active && texture_id >= 0 && (texture_id < g_textures.num() || all_textured)) {
    glBindTexture(GL_TEXTURE_2D, g_textures[texture_id].texname);
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_TEXTURE_GEN_S);
    glEnable(GL_TEXTURE_GEN_T);
    if (texturenormal) {
      Vector lightdirmodel = k_lightdir_eye0 * ~fmodeltoeye;
      assertw(lightdirmodel.normalize());
      Vector eyedirmodel = Vector(-1.f, 0.f, 0.f) * ~fmodeltoeye;
      assertw(eyedirmodel.normalize());
      normalmap_setlight(lightdirmodel, eyedirmodel, lambient);
    }
  }
  if (ledges || strip_lines) enable_polygon_offset();
  return true;
}

void draw_list(CArrayView<unique_ptr<Node>> arn) {
  glShadeModel(lsmooth ? GL_SMOOTH : GL_FLAT);
  int ii = lquickmode ? quicki : 0;
  const int buffer_ntriangles = g_is_ati ? INT_MAX : 32;
  const int buffer_nedges = g_is_ati ? INT_MAX : 128;
  const int buffer_npoints = g_is_ati ? INT_MAX : 128;
  for_int(i, arn.num()) {  // note: index i is also incremented within loop
    if (ii) {
      if (!--ii)
        ii = quicki;
      else
        continue;
    }
    const Node* un = arn[i].get();
    switch (un->_type) {
      case Node::EType::polygon: {
        const auto* n = down_cast<const NodePolygon*>(un);
        if (ledges && !lshading) continue;
        initialize_lit();
        // OpenGL seems to have problem culling polygons, even if planar and convex.
        // Would probably have to tessellate to get around that bug.
        if (n->poly.num() == 3)
          glBegin(GL_TRIANGLES);
        else
          glBegin(GL_POLYGON);
        for (int j = 0;;) {
          bool csmooth = lsmooth && n->vnors.num();
          bool hascolors = n->colors.num() > 1;
          if (!hascolors) {
            update_mat_color(n->colors[0]);
            if (!csmooth) glNormal3fv(n->pnor.data());
            for_int(vi, n->poly.num()) {
              if (csmooth) glNormal3fv(n->vnors[vi].data());
              glVertex3fv(n->poly[vi].data());
            }
          } else {
            for_int(vi, n->poly.num()) {
              update_mat_color(n->colors[vi]);
              glNormal3fv(csmooth ? n->vnors[vi].data() : n->pnor.data());
              glVertex3fv(n->poly[vi].data());
            }
          }
          if (n->poly.num() != 3) break;
          if (i + 1 == arn.num()) break;
          n = down_cast<const NodePolygon*>(arn[i + 1].get());  // Node* but not necessarily NodePolygon*
          if (n->_type != Node::EType::polygon || n->poly.num() != 3 || ++j >= buffer_ntriangles) break;
          i++;
        }
        glEnd();
        break;
      }
      case Node::EType::line: {
        const auto* n = down_cast<const NodeLine*>(un);
        initialize_unlit();
        if (n->pa.num() == 2) {
          glBegin(GL_LINES);
          for (int j = 0;;) {
            for_int(vi, n->pa.num()) {
              update_cur_color(n->colors[vi]);
              glVertex3fv(n->pa[vi].data());
            }
            if (i + 1 == arn.num()) break;
            n = down_cast<const NodeLine*>(arn[i + 1].get());  // Node* but not necessarily NodeLine*
            if (n->_type != Node::EType::line || ++j >= buffer_nedges || n->pa.num() != 2) break;
            i++;
          }
          glEnd();
        } else {
          glBegin(GL_LINE_STRIP);
          for_int(vi, n->pa.num()) {
            update_cur_color(n->colors[vi]);
            glVertex3fv(n->pa[vi].data());
          }
          glEnd();
        }
        break;
      }
      case Node::EType::point: {
        const auto* n = down_cast<const NodePoint*>(un);
        initialize_unlit();
        glBegin(GL_POINTS);
        for (int j = 0;;) {
          update_cur_color(n->color);
          glVertex3fv(n->p.data());
          if (i + 1 == arn.num()) break;
          n = down_cast<const NodePoint*>(arn[i + 1].get());  // Node* but not necessarily NodePoint*
          if (n->_type != Node::EType::point || ++j >= buffer_npoints) break;
          i++;
        }
        glEnd();
        break;
      }
      default: assertnever("");
    }
  }
  if (ledges) {
    glShadeModel(GL_FLAT);
    initialize_unlit();
    update_cur_color(pix_edgecolor);
    glBegin(GL_LINES);
    int j = 0;
    for_int(i, arn.num()) {
      if (ii) {
        if (!--ii)
          ii = quicki;
        else
          continue;
      }
      const Node* un = arn[i].get();
      if (un->_type == Node::EType::polygon) {
        const auto* n = down_cast<const NodePolygon*>(un);
        const Polygon& poly = n->poly;
        j += poly.num();
        if (j > buffer_nedges) {
          glEnd();
          glBegin(GL_LINES);
        }
        glVertex3fv(poly[0].data());
        for_intL(vi, 1, poly.num()) {
          glVertex3fv(poly[vi].data());
          glVertex3fv(poly[vi].data());
        }
        glVertex3fv(poly[0].data());
      }
    }
    glEnd();
  }
}

Map<const GMesh*, Array<Face>> map_mfa;

void mesh_init(GMesh& mesh) {
  static Set<const GMesh*> have_vnors, have_fnors;
  Set<Vertex> vredo;
  Set<Face> fredo;
  Polygon poly;
  bool meshmodified = !mesh.gflags().flag(g3d::mflag_ok).set(true);
  if (meshmodified) {
    // Could free up mesh strings after use; useful?
    // For future: mesh.gflags().flag(mflag_f_colors) = false;
    // Clear the NON-current mode
    if (!lsmooth) have_vnors.remove(&mesh);
    if (lsmooth) have_fnors.remove(&mesh);
    map_mfa.remove(&mesh);  // remove may return empty array
    for (Vertex v : mesh.vertices()) {
      if (mesh.flags(v).flag(g3d::vflag_ok).set(true)) continue;
      vredo.add(v);
      for (Vertex w : mesh.vertices(v)) vredo.add(w);
      for (Face f : mesh.faces(v)) fredo.add(f);
    }
    for (Vertex v : vredo) {
      VertexLOD& vlod = v_lod(v);
      if (parse_key_vec(mesh.get_string(v), "Opos", vlod.Opos)) {
        vlod.Npos = mesh.point(v);
      }
      A3dColor co;
      if (parse_key_vec(mesh.get_string(v), "rgb", co)) {
        mesh.gflags().flag(mflag_v_colors) = true;
        mesh.flags(v).flag(vflag_color) = true;
        v_color(v) = pack_color(co);
        vlod.Nd = v_color(v);
        if (parse_key_vec(mesh.get_string(v), "Orgb", co)) {
          vlod.Od = pack_color(co);
        } else {
          vlod.Od = pack_color(A3dColor(1.f, 1.f, 0.f));  // yellow
        }
      }
      bool has_c_color = false;
      for (Corner c : mesh.corners(v)) {
        if (GMesh::string_has_key(mesh.get_string(c), "rgb")) has_c_color = true;
      }
      if (has_c_color) {
        if (mesh.flags(v).flag(vflag_color).set(false)) Warning("Found both vertex and corner rgb");
        mesh.gflags().flag(mflag_c_colors) = true;
        mesh.flags(v).flag(vflag_c_color) = true;
        for (Corner c : mesh.corners(v)) {
          if (!parse_key_vec(mesh.get_string(c), "rgb", co)) {
            Warning("corner missing color");
            co = A3dColor(0.f, 0.f, 0.f);
          }
          c_color(c) = pack_color(co);
          c_lod(c).Nd = c_color(c);
          if (parse_key_vec(mesh.get_string(c), "Orgb", co)) {
            c_lod(c).Od = pack_color(co);
          } else {
            c_lod(c).Od = pack_color(A3dColor(1.f, .8f, .5f));  // orange
          }
        }
      }
      if (1) {
        UV vuv(BIGFLOAT, BIGFLOAT);
        if (parse_key_vec(mesh.get_string(v), "uv", vuv)) {
          // is now stored in vuv
          mesh.gflags().flag(mflag_uv) = true;
        }
        for (Corner c : mesh.corners(v)) {
          UV& uv = c_uv(c);
          if (parse_key_vec(mesh.get_string(c), "uv", uv)) {
            // is now stored in uv
            mesh.gflags().flag(mflag_uv) = true;
          } else {
            uv = vuv;
          }
        }
      }
    }
    for (Face f : mesh.faces()) {
      if (!mesh.flags(f).flag(g3d::fflag_ok).set(true)) fredo.add(f);
    }
    for (Face f : fredo) {
      if (!mesh.is_triangle(f)) {
        mesh.polygon(f, poly);
        if (!poly.is_convex() && !num_concave++) Warning("Have concave polygons in mesh");
      }
      A3dColor co;
      if (parse_key_vec(mesh.get_string(f), "rgb", co)) {
        mesh.gflags().flag(mflag_f_colors) = true;
        mesh.flags(f).flag(fflag_color) = true;
        f_color(f) = pack_color(co);
      }
    }
  }
  if (lsmooth) {
    if (have_vnors.add(&mesh))
      for (Vertex v : mesh.vertices()) vredo.add(v);
    for (Vertex v : vredo) {
      Vnors vnors;
      vnors.compute(mesh, v);
      bool uniquenors = true;
      int num = 0;
      Vector gOnor, gNnor;
      dummy_init(gOnor, gNnor);
      for (Corner c : mesh.corners(v)) {
        c_nor(c) = vnors.get_nor(mesh.corner_face(c));
        Vector onor;
        if (mesh.parse_corner_key_vec(c, "Onormal", onor)) {
          CornerLOD& clod = c_lod(c);
          clod.Nnor = c_nor(c);
          clod.Onor = onor;
          if (!num++) {
            gOnor = clod.Onor;
            gNnor = clod.Nnor;
          } else {
            if (gOnor != clod.Onor) uniquenors = false;
            if (gNnor != clod.Nnor) uniquenors = false;
          }
        } else {
          if (num) {
            Warning("missing Onormal");
            uniquenors = false;
          }
        }
      }
      // HH_SSTAT(Suniquenors, uniquenors);
      if (uniquenors) {
        mesh.flags(v).flag(vflag_unique_nors) = true;
        VertexLOD& vlod = v_lod(v);
        vlod.Onor = gOnor;
        vlod.Nnor = gNnor;
      }
    }
  }
  if (1) {
    for (Vertex v : vredo) {
      if (parse_key_vec(mesh.get_string(v), "Ouv", v_lod(v).Ouv)) {
        assertx(parse_key_vec(mesh.get_string(v), "uv", v_lod(v).Nuv));
        assertx(mesh.gflags().flag(mflag_uv));
      }
    }
  }
  if (!lsmooth || ((ledges || strip_lines) && cullbackedges && lcullface)) {
    if (have_fnors.add(&mesh))
      for (Face f : mesh.faces()) fredo.add(f);
    for (Face f : fredo) {
      mesh.flags(f).flag(g3d::fflag_ok) = true;
      mesh.polygon(f, poly);
      f_pnor(f) = poly.get_normal();
    }
  }
  if (0 && strip_lines && !map_mfa.contains(&mesh)) {
    Array<Face> fa;
    for (Face f : mesh.ordered_faces()) fa.push(f);
    map_mfa.enter(&mesh, std::move(fa));
  }
  if (strip_lines) {
    bool is_new;
    Array<Face>& fa = map_mfa.enter(&mesh, Array<Face>(), is_new);
    if (is_new) {
      for (Face f : mesh.ordered_faces()) fa.push(f);
    }
  }
}

// inline Pixel interp_color(const Pixel& col1, const Pixel& col2, float alpha) {
//     assertx(alpha >= 0.f && alpha <= 1.f);
//     int ialpha1 = int(alpha * 256.f), ialpha2 = 256 - ialpha1;
//     return interp_color(col1, col2, ialpha1, ialpha2);
// }

inline void setup_face(const GMesh& mesh, Face f) {
  if (texture_active && !texture_lit) {
  } else {
    if (!lsmooth) glNormal3fv(f_pnor(f).data());
    if (mesh.flags(f).flag(fflag_color)) {
      maybe_update_mat_diffuse(f_color(f));
    } else if (mesh.gflags().flag(mflag_f_colors)) {
      Warning("face without color");
      maybe_update_mat_diffuse(meshcolor.d);
    }
  }
}

inline void render_corner(const GMesh& mesh, Corner c) {
  Vertex v = mesh.corner_vertex(c);
  if (texture_active) {
    if (mesh.gflags().flag(mflag_uv)) {
      UV uv = c_uv(c);
      if (uv[0] != BIGFLOAT) {
        glTexCoord2fv(uv.data());
      } else {
        Warning("No uv at corner");
      }
    } else {
      // use GL_TEXTURE_GEN
    }
  }
  if (texture_active && !texture_lit) {
  } else {
    if (mesh.flags(v).flag(vflag_c_color)) {
      maybe_update_mat_diffuse(c_color(c));
    } else if (mesh.flags(v).flag(vflag_color)) {
      maybe_update_mat_diffuse(v_color(v));
    } else if (!mesh.flags(mesh.corner_face(c)).flag(fflag_color)) {
      // this may be optional.
      maybe_update_mat_diffuse(meshcolor.d);
    }
    if (lsmooth) glNormal3fv(c_nor(c).data());
  }
  glVertex3fv(mesh.point(v).data());
}

inline void render_face(const GMesh& mesh, Face f) {
  setup_face(mesh, f);
  for (Corner c : mesh.corners(f)) render_corner(mesh, c);
}

inline bool same_corner_attrib(const GMesh& mesh, Vertex v1, Corner c1a, Corner c1b) {
  if (texture_active && mesh.gflags().flag(mflag_uv) && c_uv(c1a) != c_uv(c1b)) return false;
  if (mesh.flags(v1).flag(vflag_c_color) && c_color(c1a) != c_color(c1b)) return false;
  if (lsmooth && c_nor(c1a) != c_nor(c1b)) return false;
  return true;
}

inline bool edge_continuous(const GMesh& mesh, Edge e) {
  {
    Vertex v1 = mesh.vertex1(e);
    if (!same_corner_attrib(mesh, v1, mesh.ccw_corner(v1, e), mesh.clw_corner(v1, e))) return false;
  }
  {
    Vertex v2 = mesh.vertex2(e);
    if (!same_corner_attrib(mesh, v2, mesh.ccw_corner(v2, e), mesh.clw_corner(v2, e))) return false;
  }
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  ASSERTX(f2);
  if (lsmooth && (mesh.flags(f1).flag(fflag_color) != mesh.flags(f2).flag(fflag_color) ||
                  (mesh.flags(f1).flag(fflag_color) && f_color(f1) != f_color(f2))))
    return false;
  return true;
}

void draw_mesh(GMesh& mesh) {
  mesh_init(mesh);
  const int buffer_ntriangles = g_is_ati ? INT_MAX : 32;
  const int buffer_nquads = g_is_ati ? INT_MAX : 32;
  const int buffer_nedges = g_is_ati ? INT_MAX : 128;
  if (!ledges || lshading) {
    bool has_f_color = mesh.gflags().flag(mflag_f_colors);
    bool has_v_color = mesh.gflags().flag(mflag_v_colors);
    bool has_c_color = mesh.gflags().flag(mflag_c_colors);
    // bool has_only_v_color = has_v_color && !has_c_color;
    bool has_either_color = has_v_color || has_c_color;
    assertw(!(has_f_color && (has_v_color || has_c_color)));
    bool smooth_shade_model = lsmooth || has_either_color;
    glShadeModel(smooth_shade_model ? GL_SMOOTH : GL_FLAT);
    bool cannot_strip = !lsmooth && smooth_shade_model;
    initialize_lit();
    update_mat_color(meshcolor);
    if (texture_active) {
      if (mesh.gflags().flag(mflag_uv)) {
        glDisable(GL_TEXTURE_GEN_S);
        glDisable(GL_TEXTURE_GEN_T);
      }
      if (!texture_lit) {
        glShadeModel(GL_FLAT);
        initialize_unlit();
        update_cur_color(meshcolor.d);
        set_light_ambient(0.f);
        if (texturenormal) normalmap_activate();
      }
      if (0) {  // debug: peak at primary colors
        glDisable(GL_TEXTURE_2D);
        if (texturenormal) normalmap_deactivate();
      }
    }
    int ii = lquickmode ? quicki : 0;
    // On ATI, display_list creation is unacceptably slow for
    //  many triangle strip primitives.
    // It is reasonably fast if given large buffers of GL_TRIANGLES.
    // Rendering performance is good except on meshes with vertex colors,
    //  where presumably the mesh connectivity is not reconstructed.
    // On NVIDIA, display_list creation is fast for small buffers
    //  of GL_TRIANGLE_STRIP.
    //
    // Set this variable for overlapped faces (lapped textures)
    //  to ensure all faces are rendered in same order and
    //  with same vertex order.
    static const bool strict_mesh_order = getenv_bool("STRICT_MESH_ORDER");
    if (strict_mesh_order || (defining_dl && g_is_ati)) {
      int ntriangles = 0;
      int nquads = 0;
      for (Face f : mesh.ordered_faces()) {
        if (ii) {
          if (!--ii)
            ii = quicki;
          else
            continue;
        }
        int nv = mesh.num_vertices(f);
        // It appears that on some platforms, use of GL_QUADS results in GL_INVALID_OPERATION even
        // though the rendering succeeds.  Instead we use GL_POLYGON to be safe.
        const bool replace_quads_by_polygons = true;
        if (replace_quads_by_polygons && nv == 4) nv = -1;
        switch (nv) {
          case 3:
            if (nquads) {
              glEnd();
              nquads = 0;
            }
            if (ntriangles == buffer_ntriangles) {
              glEnd();
              ntriangles = 0;
            }
            if (!ntriangles) glBegin(GL_TRIANGLES);
            ntriangles++;
            break;
          case 4:
            if (ntriangles) {
              glEnd();
              ntriangles = 0;
            }
            if (nquads == buffer_nquads) {
              glEnd();
              nquads = 0;
            }
            glBegin(GL_QUADS);
            nquads++;
            break;
          default:
            if (ntriangles) {
              glEnd();
              ntriangles = 0;
            }
            if (nquads) {
              glEnd();
              nquads = 0;
            }
            glBegin(GL_POLYGON);
        }
        render_face(mesh, f);
        if (!ntriangles && !nquads) glEnd();  // GL_POLYGON
      }
      if (ntriangles) glEnd();  // GL_TRIANGLES
      if (nquads) glEnd();      // GL_QUADS
    } else if (defining_dl && !lquickmode && !cannot_strip) {
      // Invest time to form triangle strips.
      HH_STATNP(Sstriplen);
      if (getenv_bool("STRIPS_DEBUG")) Sstriplen.set_print(true);
      // toggle visited flag (to avoid a pass to clear all the flags)
      static const FlagMask mflag_fvisited = Mesh::allocate_flag();
      static const FlagMask fflag_visited = Mesh::allocate_Face_flag();
      bool vis_new_state = !mesh.gflags().flag(mflag_fvisited);
      mesh.gflags().flag(mflag_fvisited) = vis_new_state;
      if (k_debug) {
        for (Face f : mesh.faces()) assertx(mesh.flags(f).flag(fflag_visited) != vis_new_state);
      }
      Array<Vertex> va;
      for (Face f : mesh.faces()) {
        if (mesh.flags(f).flag(fflag_visited) == vis_new_state) continue;
        mesh.flags(f).flag(fflag_visited) = vis_new_state;
        if (!mesh.is_triangle(f)) {
          glBegin(GL_POLYGON);
          render_face(mesh, f);
          glEnd();
          continue;
        }
        // Start a triangle strip at face f
        glBegin(GL_TRIANGLE_STRIP);
        int striplen = 1;
        int vi = 0;
        mesh.get_vertices(f, va);
        for_int(i, 3) {
          Face fn = mesh.opp_face(va[i], f);
          if (!fn || !mesh.is_triangle(fn) || mesh.flags(fn).flag(fflag_visited) == vis_new_state ||
              !edge_continuous(mesh, mesh.opp_edge(va[i], f)))
            continue;
          vi = i;
        }
        setup_face(mesh, f);
        for_int(i, 3) {
          Vertex v = va[mod3(vi + i)];
          render_corner(mesh, mesh.corner(v, f));
        }
        int pivot = 1;
        Face ff = f;
        for (;;) {
          Face fn = mesh.opp_face(va[vi], ff);
          if (!fn || !mesh.is_triangle(fn) || mesh.flags(fn).flag(fflag_visited) == vis_new_state ||
              !edge_continuous(mesh, mesh.opp_edge(va[vi], ff)))
            break;
          static const int max_striplen = getenv_int("MAX_STRIPLEN");
          if (striplen == max_striplen) break;
          ff = fn;
          mesh.flags(ff).flag(fflag_visited) = vis_new_state;
          striplen++;
          Vertex vwaspiv = va[mod3(vi + pivot)];
          mesh.get_vertices(ff, va);
          vi = va.index(vwaspiv);
          ASSERTX(vi >= 0);
          if (texture_active && !texture_lit) {
          } else {
            if (!lsmooth) glNormal3fv(f_pnor(ff).data());
            if (mesh.flags(ff).flag(fflag_color)) maybe_update_mat_diffuse(f_color(ff));
          }
          Vertex v = va[mod3(vi + pivot)];
          render_corner(mesh, mesh.corner(v, ff));
          pivot = 3 - pivot;
        }
        glEnd();
        Sstriplen.enter(striplen);
      }
    } else {
      int ntriangles = 0;
      for (Face f : mesh.faces()) {
        // quicki not ideal here since it can beat against the
        //  number of entries per list in the id_face Map.
        if (ii) {
          if (!--ii)
            ii = quicki;
          else
            continue;
        }
        if (mesh.is_triangle(f)) {
          if (ntriangles == buffer_ntriangles) {
            glEnd();
            ntriangles = 0;
          }
          if (!ntriangles) {
            if (0) update_mat_color(meshcolor);  // would obviate force_color_update just for this case
            glBegin(GL_TRIANGLES);
          }
          ntriangles++;
        } else {
          if (ntriangles) {
            glEnd();
            ntriangles = 0;
          }
          glBegin(GL_POLYGON);
        }
        render_face(mesh, f);
        if (!ntriangles) glEnd();  // GL_POLYGON
      }
      if (ntriangles) glEnd();  // GL_TRIANGLES
    }
  }
  if (texture_active) {
    glDisable(GL_TEXTURE_2D);
    set_light_ambient(ambient);
    if (texturenormal) normalmap_deactivate();
  }
  if (ledges) {
    glShadeModel(GL_FLAT);
    initialize_unlit();
    int ii = lquickmode ? quicki : 0;
    int nedges = 0;
    for (Edge e : mesh.edges()) {
      if (ii) {
        if (!--ii)
          ii = quicki;
        else
          continue;
      }
      if (cullbackedges && lcullface && !uvtopos && !defining_dl) {
        const Point& p = mesh.point(mesh.vertex1(e));
        Vector vtoe = p - feyetomodel.p();
        bool cull = true;
        for (Face f : mesh.faces(e)) {
          if (dot(vtoe, f_pnor(f)) < 0) {
            cull = false;
            break;
          }
        }
        if (cull) continue;
      }
      int ithick;
      if (mesh.is_boundary(e)) {
        ithick = thickboundary;
        if (ithick) update_cur_color(pix_bndedgecolor);
      } else if (mesh.flags(e).flag(GMesh::eflag_sharp)) {
        ithick = thicksharp;
        if (ithick) update_cur_color(pix_sharpedgecolor);
      } else {
        ithick = thicknormal;
        if (ithick) update_cur_color(pix_edgecolor);
      }
      if (!ithick) continue;
      if (ithick != g_pthick && nedges) {
        glEnd();
        nedges = 0;  // change of state
      }
      set_thickness(ithick);
      if (nedges == buffer_nedges) {
        glEnd();
        nedges = 0;
      }
      if (!nedges) glBegin(GL_LINES);
      nedges++;
      if (uvtopos && mesh.gflags().flag(mflag_uv)) {
        UV uv11 = c_uv(mesh.corner(mesh.vertex1(e), mesh.face1(e)));
        UV uv21 = c_uv(mesh.corner(mesh.vertex2(e), mesh.face1(e)));
        if (1) {
          Point p11(uv11[0], uv11[1], 0.f);
          Point p21(uv21[0], uv21[1], 0.f);
          glVertex3fv(p11.data());
          glVertex3fv(p21.data());
        }
        if (mesh.face2(e)) {
          UV uv12 = c_uv(mesh.corner(mesh.vertex1(e), mesh.face2(e)));
          UV uv22 = c_uv(mesh.corner(mesh.vertex2(e), mesh.face2(e)));
          if (uv11 != uv12) {
            Point p12(uv12[0], uv12[1], 0.f);
            Point p22(uv22[0], uv22[1], 0.f);
            glVertex3fv(p12.data());
            glVertex3fv(p22.data());
          }
        }
      } else {
        glVertex3fv(mesh.point(mesh.vertex1(e)).data());
        glVertex3fv(mesh.point(mesh.vertex2(e)).data());
      }
      if (!nedges) glEnd();  // GL_LINES
    }
    if (nedges) glEnd();  // GL_LINES
    if (sphereradius) {
      static Map<GMesh*, float> mesh_radii;
      bool is_new;
      float& mesh_radius = mesh_radii.enter(&mesh, 0.f, is_new);
      if (is_new) {
        Bbox bbox;
        for (Vertex v : mesh.vertices()) bbox.union_with(mesh.point(v));
        mesh_radius = bbox.max_side();
        assertw(mesh_radius > 0.f);
      }
      glShadeModel(GL_SMOOTH);
      initialize_lit();
      update_mat_color(cuspcolor);
#if !defined(HH_NO_GLU)
      unique_ptr<GLUquadricObj, decltype(&gluDeleteQuadric)> quadric{gluNewQuadric(), gluDeleteQuadric};
      for (Vertex v : mesh.vertices()) {
        if (!mesh.flags(v).flag(GMesh::vflag_cusp)) continue;
        glPushMatrix();
        {
          const Point& p = mesh.point(v);
          glTranslatef(p[0], p[1], p[2]);
          const int nslices = 8, nstacks = 8;
          gluSphere(quadric.get(), sphereradius * mesh_radius, nslices, nstacks);
        }
        glPopMatrix();
      }
#endif
    }
    set_thickness(thicka3d);
  }
  if (strip_lines && !lquickmode) {
    bool strip_always_connect = strip_lines >= 2;
    glShadeModel(GL_FLAT);
    initialize_unlit();
    update_cur_color(pix_sharpedgecolor);
    set_thickness(thicksharp);
    Array<Vertex> va;
    Point p;
    Point pp;
    Face fp = nullptr;
    bool stripstarted = false;
    const Array<Face>& fa = map_mfa.get(&mesh);
    for (Face f : fa) {
      mesh.get_vertices(f, va);
      p = interp(mesh.point(va[0]), mesh.point(va[1]), mesh.point(va[2]));
      if (1) p = interp(feyetomodel.p(), p, edgeoffset);
      int afound = 0;
      if (1) {
        for (Face ff : mesh.faces(f)) {
          if (ff == fp) {
            afound = 1;
            break;
          }
        }
      } else {  // for cover faces in Edge Processing project.
        for (Vertex v : mesh.vertices(f)) {
          for (Face ff : mesh.faces(v)) {
            if (ff == fp) {
              afound = 1;
              break;
            }
          }
        }
      }
      if (strip_always_connect && fp && !afound) afound = 2;
      if (afound && cullbackedges && lcullface) {
        if (dot(p - feyetomodel.p(), f_pnor(f)) > 0)
          afound = 0;
        else if (dot(pp - feyetomodel.p(), f_pnor(fp)) > 0)
          afound = 0;
      }
      if (afound) {
        if (!stripstarted) {
          stripstarted = true;
          glBegin(GL_LINE_STRIP);
          glVertex3fv(pp.data());
        }
        // const Pixel pix_lightblue(0xA0, 0xA0, 0xFF);
        // const Pixel pix_lightblue(0x90, 0x90, 0xFF);
        // const Pixel pix_lightblue(0xC0, 0xC0, 0xFF);
        const Pixel pix_lightblue = Pixel::white();
        update_cur_color(afound == 1 ? pix_sharpedgecolor : pix_lightblue);
        // cannot change linewidth here in middle of line_strip.
        glVertex3fv(p.data());
      } else {
        if (stripstarted) {
          stripstarted = false;
          glEnd();
        }
      }
      fp = f;
      pp = p;
    }
    if (stripstarted) {
      // stripstarted = false;
      glEnd();
    }
    set_thickness(thicka3d);
  }
}

void draw_all() {
  static bool is_init = false;
  if (!is_init) {  // make sure all loaded to avoid later stalls
    is_init = true;
    for (int i = g_xobs.min_segn(); i <= g_xobs.max_segn(); i++) {
      if (!setup_ob(i)) continue;
      if (g_xobs[i].pmesh) mesh_init(*g_xobs[i].pmesh);
    }
  }
  static const bool no_dl = getenv_bool("G3D_NO_DL");
  for (int i = g_xobs.min_segn(); i <= g_xobs.max_segn(); i++) {
    if (hw.suggests_stop() && !inpicture) break;
    if (!g_xobs.defined(i) || !g_xobs.vis[i]) continue;
    if (!setup_ob(i)) continue;
    //
    // DL Tests 20020528: frames / sec (before use_dl -> after use_dl)
    //  dragonf 200K faces: (-geom 500x500)
    //   .m polygons        3.3 -> 5.8
    //   .m ntriangles=inf  3.7 -> 20.3 ** and long dl compile!
    //   .m ntriangles=128  3.7 -> 26.7
    //   .m ntriangles=32   3.7 -> 26.8
    //   .m strips~4 (NEW)      -> 45.5 !
    //   .pm (strips)       5.9 -> 42.5
    //   .pm ntriangles=inf 5.4 -> 9.9  (and long dl compile)
    //   .pm ntriangles=128 5.4 -> 26.7
    //   .tmpm shaded       6.7 -> 15.4
    //   .tmpm textured     6.4 -> 53.3
    //
    // If the mesh has been modified since cached, turn off all caching.
    if (use_dl && svalid_dl.contains(i) && g_xobs[i].pmesh && !g_xobs[i].pmesh->gflags().flag(g3d::mflag_ok)) {
      svalid_dl.remove(i);
      static const bool g3d_force_dl1 = getenv_bool("G3D_FORCE_DL1");
      if (!g3d_force_dl1) use_dl = false;
    }
    if (no_dl) use_dl = false;
    static Map<int, int> mdlnum;
    if (use_dl) {
      bool is_new;
      int& dlnum = mdlnum.enter(i, 0, is_new);
      if (is_new) dlnum = assertx(glGenLists(1));
      if (!svalid_dl.contains(i)) {
        defining_dl = true;
        // glNewList(dlnum, GL_COMPILE_AND_EXECUTE);  // bad(slow)!
        glNewList(dlnum, GL_COMPILE);
        initialize_lit(), initialize_unlit();  // force state record
      } else {
        glCallList(dlnum);
        continue;
      }
    }
    //
    if (i == 1 && pm_mode) {
#if defined(DEF_PM)
      draw_pm();
#endif
    } else if (i == 1 && sr_mode) {
#if defined(DEF_SR)
      draw_sr();
#endif
    } else if (i == 1 && (sc_mode || psc_mode)) {
#if defined(DEF_SC)
      draw_sc();
#endif
    } else if (i == 1 && sc_gm_mode) {
#if defined(DEF_SC)
      draw_sc_gm(Gmorphs[sc_gm_morph].getK());
#endif
    } else if (i == 1 && ply_mode) {
#if defined(DEF_PLY)
      draw_ply();
#endif
    } else {
      if (g_xobs[i].pmesh) draw_mesh(*g_xobs[i].pmesh);
      draw_list(g_xobs[i].traverse());
    }
    if (g_xobs[i].pmesh) g_xobs[i].pmesh->gflags().flag(g3d::mflag_ok) = true;  // if sc_mode, psc_mode, etc.
    //
    if (defining_dl) {
      defining_dl = false;
      static const bool debug_dl = getenv_bool("DL_DEBUG");
      if (debug_dl) {
        HH_TIMER(_dl_create);
        glEndList();
      } else {
        glEndList();
      }
      assertx(svalid_dl.add(i));
      glCallList(mdlnum.get(i));
    }
  }
}

void wrap_draw(bool show) {
  if (pm_mode) {
#if defined(DEF_PM)
    pm_wrap_draw(show);
#endif
    return;
  }
  if (sr_mode) {
#if defined(DEF_SR)
    sr_wrap_draw(show);
#endif
    return;
  }
  if (psc_mode) {
#if defined(DEF_SC)
    psc_wrap_draw(show);
#endif
    return;
  }
  if (sc_gm_mode) {
#if defined(DEF_SC)
    sc_gm_wrap_draw(show);
#endif
    return;
  }
  if (!slidermode) return;
  Array<Slider> sliders = {
      Slider{"ambient", &ambient},
      Slider{"lightsource", &lightsource},
      Slider{"fdisplacepolygon", &fdisplacepolygon},
      Slider{"flinewidth", &flinewidth},
  };
  if (!show) {
    if (button_active) {
      Vec2<float> yxf;
      assertx(HB::get_pointer(yxf));
      float dval = exp((yxf[0] - yx_pointer_old[0]) * -1.5f * g3d::fchange);
      int i = int(yx_pointer_old[1] * sliders.num() * .9999f);
      *sliders[i].val *= dval;
      hw.redraw_later();
    }
  } else {
    for_int(i, sliders.num()) {
      float x = (i + .2f) / sliders.num();
      HB::draw_text(V(.12f, x), sliders[i].name);
      HB::draw_text(V(.17f, x), sform("%g", *sliders[i].val));
    }
  }
}

void process_print() {
  hw.hard_flush();
  if (!movie_nframes) SHOW("starting image print...");
  Image image(win_dims);
  const int nypix = win_dims[0], nxpix = win_dims[1];
  // IMAGE* image = iopen(imagefilename.c_str(), "w", RLE(1), 3, nxpix, nypix, 3);
  // pixmode(); default 32 bit size fine, row order unimportant
  // drawmode(); NORMALDRAW is fine
  glPushAttrib(GL_PIXEL_MODE_BIT);
  {                            // save GL_READ_BUFFER
    if (0) {                   // 20160907 disabled because we do want most-recent rendering which is in backbuffer
      glReadBuffer(GL_FRONT);  // default is GL_BACK if doublebuffered
    }
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    // glPixelStorei(GL_PACK_ROW_LENGTH, nxpix*3);
    if (nxpix < 44) {
      // Generally slow for RemoteDesktop, but must use this to counter OpenGL bug if nxpix<44.
      for_int(y, nypix) glReadPixels(0, y, nxpix, 1, GL_RGBA, GL_UNSIGNED_BYTE, image[y].data());
    } else {
      glReadPixels(0, 0, nxpix, nypix, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
    }
  }
  glPopAttrib();
  if (1) image.reverse_y();  // because glReadPixels() has image origin at lower-left
  if (movie_desire_video) {
    if (!movie_video) {
      assertx(!movie_frame);
      movie_video = make_unique<Video>(concat(V(movie_nframes), image.dims()));
      movie_video->attrib().suffix = "mp4";                                           // useful default
      movie_video->attrib().framerate = 60;                                           // useful default
      movie_video->attrib().bitrate = max(1000000, int(product(image.dims()) * 10));  // ~20Mbps at FullHD
    }
    (*movie_video)[movie_frame].assign(image);
  } else {
    string name = imagefilename;
    if (movie_nframes) name = sform("%s.%03d.bmp", movie_rootname.c_str(), movie_frame);
    if (name[0] == '|') image.set_suffix("bmp");
    if (!movie_nframes) SHOW(name);
    if (movie_nframes) image.set_silent_io_progress(true);
    image.write_file(name);
    if (!movie_nframes) SHOW("...image printed");
  }
}

void toggle_attribute(Vec<bool, k_max_object>& attrib) {
  bool allvis = true;
  for (int i = g_xobs.min_segn(); i <= g_xobs.max_segn(); i++) {
    if (g_xobs.defined(i) && !g_xobs.vis[i]) allvis = false;
  }
  for_int(i, k_max_object) {
    if (!allvis && (!g_xobs.defined(i) || !g_xobs.vis[i])) continue;
    bool& val = attrib[i];
    val = !val;
  }
}

void all_reset() {
  for_int(i, k_max_object) {
    if (!g_xobs.defined(i)) continue;
    g_xobs.cullface[i] = true;
    g_xobs.reverse_cull[i] = false;
    g_xobs.shading[i] = true;
    g_xobs.smooth[i] = true;
    g_xobs.edges[i] = false;
  }
  mdepthcue = false;
  antialiasing = true;
  nice_rendering = false;
  perspec = true;
  uvtopos = false;
  slidermode = false;
  outside_frustum = false;
  butquick = false;
  quickmode = false;
  strip_lines = 0;
  quicki = 4;
  fdisplacepolygon = 1.f;
  flinewidth = 1.f;
  frustum_frac = 1.4f;
  texture_active = false;
  if (0) {
    ambient = .30f;
    lightsource = .65f;
    anisotropy = INT_MAX;
    set_anisotropy();
  }
}

// *** GXobject

Color create_mat_color(const A3dVertexColor& vc) {
  Color col;
  col.d = pack_color(vc.d);
  col.s = pack_color(vc.s);
  col.g = vc.g[0];
  return col;
}

bool operator==(const A3dVertexColor& c1, const A3dVertexColor& c2) {
  return c1.d == c2.d && c1.s == c2.s && c1.g == c2.g;
}

bool operator!=(const A3dVertexColor& c1, const A3dVertexColor& c2) { return !(c1 == c2); }

void GXobject::add(const A3dElem& el) {
  assertx(_opened);
  assertx(el.num());
  switch (el.type()) {
    case A3dElem::EType::polygon: {
      auto n = make_unique<NodePolygon>();
      bool hasvnors = false, hascolors = false;
      for_int(i, el.num()) {
        if (!is_zero(el[i].n)) hasvnors = true;
        if (i && el[i].c != el[0].c) hascolors = true;
      }
      n->poly.init(el.num());
      if (hasvnors) n->vnors.init(el.num());
      n->colors.init(hascolors ? el.num() : 1);
      for_int(i, el.num()) {
        n->poly[i] = el[i].p;
        if (hasvnors) n->vnors[i] = el[i].n;
        if (!i || hascolors) {
          const A3dVertexColor* c = el[i].c.g[0] ? &el[i].c : &k_default_color;
          n->colors[i] = create_mat_color(*c);
        }
      }
      n->pnor = n->poly.get_normal();
      if (!n->poly.is_convex() && !num_concave++ && s_idraw) Warning("Have concave polygons in a3d");
      append(std::move(n));
      break;
    }
    case A3dElem::EType::polyline: {
      if (0 && el.num() == 2 && el[0].p == el[1].p) {
        // because zero-length lines don't show up under GL
        // but, confuses OpenGL display list optimizer
        // And, why treat zero-length lines differently from 1e-10f lines?
        auto n = make_unique<NodePoint>();
        n->p = el[0].p;
        n->color = pack_color(el[0].c.g[0] ? el[0].c.d : k_default_poly_color.d);
        append(std::move(n));
        return;
      }
      auto n = make_unique<NodeLine>();
      n->pa.init(el.num());
      n->colors.init(el.num());
      for_int(i, el.num()) {
        n->pa[i] = el[i].p;
        n->colors[i] = pack_color(el[i].c.g[0] ? el[i].c.d : k_default_poly_color.d);
      }
      // Problem is that line is still usually invisible, or too visible.
      if (0 && el.num() == 2 && el[0].p == el[1].p) n->pa[1] += Vector(1e-4f, 0.f, 0.f);
      append(std::move(n));
      break;
    }
    case A3dElem::EType::point: {
      auto n = make_unique<NodePoint>();
      n->p = el[0].p;
      n->color = pack_color(el[0].c.g[0] ? el[0].c.d : k_default_poly_color.d);
      append(std::move(n));
      break;
    }
    default: assertnever(string() + "unknown type '" + narrow_cast<char>(el.type()) + "'");
  }
}

void GXobject::append(unique_ptr<Node> n) {
  _arn.push(std::move(n));
  if (s_idraw) {
    assertx(is_window);
    draw_list(ArView(_arn.last()));
  }
}

void GXobject::close() {
  assertx(_opened);
  _opened = false;
  if (s_idraw) {
    gl_fixup();
    hw.end_draw_visible();
  }
}

void GXobject::morph(float finterp) {  // finterp == 1.f is new,   finterp == 0.f is old
  use_dl = false;
  GMesh& mesh = *pmesh;
  mesh_init(mesh);
  bool has_v_color = mesh.gflags().flag(mflag_v_colors);
  bool has_c_color = mesh.gflags().flag(mflag_c_colors);
  bool has_only_v_color = has_v_color && !has_c_color;
  bool has_either_color = has_v_color || has_c_color;
  float f1 = finterp, f2 = 1.f - f1;
  int if1 = int(finterp * 256.f), if2 = 256 - if1;
  for (Vertex v : mesh.vertices()) {
    const VertexLOD& vlod = v_lod(v);
    { mesh.set_point(v, interp(vlod.Npos, vlod.Opos, f1)); }
    if (has_only_v_color) {
      v_color(v) = interp_color(vlod.Nd, vlod.Od, if1, if2);
    } else if (has_either_color) {
      if (mesh.flags(v).flag(vflag_color)) {
        v_color(v) = interp_color(vlod.Nd, vlod.Od, if1, if2);
      } else if (mesh.flags(v).flag(vflag_c_color)) {
        for (Corner c : mesh.corners(v)) {
          const CornerLOD& clod = c_lod(c);
          c_color(c) = interp_color(clod.Nd, clod.Od, if1, if2);
        }
      }
    }
    if (lsmooth) {
      if (mesh.flags(v).flag(vflag_unique_nors)) {
        Vector nnor = interp_normal(vlod.Nnor, vlod.Onor, f1, f2);
        for (Corner c : mesh.corners(v)) c_nor(c) = nnor;
      } else {
        for (Corner c : mesh.corners(v)) {
          const CornerLOD& clod = c_lod(c);
          c_nor(c) = interp_normal(clod.Nnor, clod.Onor, f1, f2);
        }
      }
    }
    if (texture_active) {
      UV uv;
      uv[0] = f1 * vlod.Nuv[0] + f2 * vlod.Ouv[0];
      uv[1] = f1 * vlod.Nuv[1] + f2 * vlod.Ouv[1];
      for (Corner c : mesh.corners(v)) c_uv(c) = uv;
    }
  }
  if (!lsmooth || (ledges && cullbackedges && lcullface)) {
    if (0) {
      Polygon poly;
      for (Face f : mesh.faces()) {
        mesh.polygon(f, poly);
        f_pnor(f) = poly.get_normal();
      }
    } else {
      Warning("GL_NORMALIZE");
      glEnable(GL_NORMALIZE);  // not reset anywhere I hope.
      ogl_normalize = true;
      for (Face f : mesh.faces()) {
        Corner c1;
        dummy_init(c1);
        for (Corner c : mesh.corners(f)) {
          c1 = c;
          if (1) break;
        }
        Corner c2 = mesh.ccw_face_corner(c1);
        Corner c3 = mesh.ccw_face_corner(c2);
        if (!assertw(mesh.ccw_face_corner(c3) == c1)) continue;
        const Point& p1 = mesh.point(mesh.corner_vertex(c1));
        const Point& p2 = mesh.point(mesh.corner_vertex(c2));
        const Point& p3 = mesh.point(mesh.corner_vertex(c3));
        Vector& v = f_pnor(f);
        // from cross()
        float p1x = p1[0], p1y = p1[1], p1z = p1[2];
        float v1x = p2[0] - p1x, v1y = p2[1] - p1y, v1z = p2[2] - p1z;
        float v2x = p3[0] - p1x, v2y = p3[1] - p1y, v2z = p3[2] - p1z;
        float vx = v1y * v2z - v1z * v2y;
        float vy = v1z * v2x - v1x * v2z;
        float vz = v1x * v2y - v1y * v2x;
        v[0] = vx;
        v[1] = vy;
        v[2] = vz;
      }
    }
  }
}

// *** GXobjects

GXobjects::GXobjects() {
  for_int(i, k_max_object) {
    _link[i] = 0;
    t[i] = Frame::identity();
    vis[i] = false;
    cullface[i] = true;
    reverse_cull[i] = false;
    shading[i] = true;
    smooth[i] = true;
    edges[i] = false;
  }
}

void GXobjects::clear(int segn) {
  assertx(_segn == -1);
  assertx(_link.ok(segn));
  _link[segn] = 0;
  _ob[segn] = nullptr;
  svalid_dl.remove(segn);
  // I could update _imin, _imax here
}

void GXobjects::open(int segn) {
  assertx(_segn == -1);
  _segn = segn;
  assertx(_link.ok(segn));
  _link[_segn] = 0;
  if (!_ob[_segn]) {  // create GXobject
    _imin = min(_imin, _segn);
    _imax = max(_imax, _segn);
    _ob[_segn] = make_unique<GXobject>();
  }
  if (is_window) {
    hw.begin_draw_visible();
    gl_init();
    setup_ob(_segn);
  }
  _ob[_segn]->open(is_window);
}

void GXobjects::make_link(int oldsegn, int newsegn) {
  assertx(_segn == -1);
  assertx(_link.ok(oldsegn));
  assertx(_link.ok(newsegn));
  assertx(oldsegn != newsegn);
  clear(newsegn);
  _link[newsegn] = oldsegn;
  _imin = min(_imin, newsegn);
  _imax = max(_imax, newsegn);
}

GXobject* GXobjects::obp(int i) const {
  assertx(_link.ok(i));
  if (_ob[i]) return _ob[i].get();
  if (_link[i]) return _ob[_link[i]].get();
  return nullptr;
}

}  // namespace

// *** HB function calls

bool HB::init(Array<string>& aargs, bool (*pfkeyp)(const string& s),
              void (*pfbutp)(int butnum, bool pressed, bool shift, const Vec2<float>& yx), void (*pfwheel)(float v),
              void (*pfdraw)()) {
  fkeyp = assertx(pfkeyp);
  fbutp = assertx(pfbutp);
  fwheel = assertx(pfwheel);
  fdraw = assertx(pfdraw);
  quicki = 4;
  mdepthcue = false;
  antialiasing = true;
  nice_rendering = false;
  perspec = true;
  slidermode = false;
  hither = k_default_hither;
  yonder = k_default_yonder;
  // had ambient=.25 lightsource=.75 on SGI
  // had ambient=.60 lightsource=.75 on PC
  // changed to high-contrast default on 20020117
  //  (I liked the contrast of the old normal-map images.)
  // ambient = .25f; lightsource = .85f;  // before 20120502
  ambient = .30f;      // was .60f
  lightsource = .65f;  // was .75f then .72f
  backfacec = {.15f, .5f, .15f};
  dbuffer = true;
  Vec3<float> spherecolor = {.2f, 1.f, .2f};  // was {1.f, .2f, .2f}
  Vec3<float> meshcold = {.6f, .6f, .6f};     // was {.8f, .5f, .4f} before 20120502
  Vec3<float> meshcols = {.5f, .5f, .5f};
  Vec3<float> meshcolp = {4.f, 0.f, 0.f};  // 4 to match normal-map; was 5 before 20020117
  Vec1<float> meshcola = {1.f};
  string pm_filename;
  string sc_filename;
  string sc_gm_filename;
  string psc_filename;
  string sr_filename;
  string ply_filename;
  bool hw_success = hw.init(aargs);
  ParseArgs args(aargs, "HB_GL");
  HH_ARGSP(edgecolor, "#RRGGBB : set 'De' color");
  HH_ARGSP(sharpedgecolor, "#RRGGBB : ''");
  HH_ARGSP(bndedgecolor, "#RRGGBB : ''");
  HH_ARGSP(cullbackedges, "b : cull backfacing 'De' edges");
  HH_ARGSF(outside_frustum, ": view larger than frustum");
  HH_ARGSP(frustum_frac, "f : set fraction larger if outside");
  args.p("-lighta[mbient]", ambient, "f : set ambient intensity (0-1)");
  args.p("-lights[ource]", lightsource, "f : set light intensity (0-1)");
  args.p("-bfacec[olor]", backfacec, "r g b : set backfacing color");
  args.p("-imagen[ame]", imagefilename, "image_filename : set 'DP' name");
  HH_ARGSP(dbuffer, "val : set double buffering");
  HH_ARGSF(picture, ": output picture and end");
  HH_ARGSD(movie, "nframes rootname : create movie as set of rootname.%03d.bmp files");
  HH_ARGSD(video, "nframes name.mp4 : create movie (360 frames for one 'J' rotation)");
  args.p("-thickb[oundary]", thickboundary, "i : width of bnd edges");
  args.p("-thicks[harp]", thicksharp, "i : width of sharp edges");
  args.p("-thickn[ormal]", thicknormal, "i : width of edges");
  args.p("-thicka[3d]", thicka3d, "i : width of a3d edges");
  args.p("-thickp[oint]", thickpoint, "f : size of a3d points");
  HH_ARGSP(sphereradius, "f : tagged vertices radius");
  HH_ARGSP(spherecolor, "r g b : tagged vertices color");
  HH_ARGSP(meshcold, "r g b : mesh color diffuse");
  HH_ARGSP(meshcols, "r g b : mesh color specular");
  HH_ARGSP(meshcolp, "c 0 0 : mesh color phong coef");
  HH_ARGSP(meshcola, "a : mesh color alpha");
  HH_ARGSD(texturemap, "image_filename : load texture map");
  HH_ARGSP(texturenormal, "bool : normal (not bump) map");
  HH_ARGSP(texturescale, "fac : scale texture coordinates (0=no_wrap)");
  args.p("-pm_mode", pm_filename, "file.pm : progressive mesh mode");
  args.p("-sr_mode", sr_filename, "file.pm : selective refinement");
  args.p("-sc_mode", sc_filename, "file.sc : simplicial complex mode");
  args.p("-psc_mode", psc_filename, "file.psc : sc unify stream");
  args.p("-sc_gm_mode", sc_gm_filename, "file.scg : sc geomorph");
  args.p("-ply_mode", ply_filename, "file.ply : read ply file");
  HH_ARGSF(hpix, ": do standard work for profiling");
  HH_ARGSF(worldlight, ": bind light to world coordinates");
  HH_ARGSP(fdisplacepolygon, "f : change OpenGL setting");
#if defined(DEF_SR)
  HH_ARGSP(sr_screen_thresh, "t : fraction window radius");
  HH_ARGSP(sr_regulatenf, "nfaces : regulate # faces");
  HH_ARGSP(sr_fracvtrav, "f : fraction active verts to traverse");
  HH_ARGSP(sr_gain, "val : set regulator gain");
  HH_ARGSP(sr_gtime, "val : # frames for geomorph refinement");
#endif
  HH_ARGSP(noinfo, "bool : do not show info or sliders");
  args.other_args_ok();
  args.other_options_ok();
  args.disallow_prefixes();
  if (!args.parse_and_extract(aargs) || !hw_success) return false;
  pix_edgecolor = parse_color(edgecolor);
  pix_sharpedgecolor = parse_color(sharpedgecolor);
  pix_bndedgecolor = parse_color(bndedgecolor);
  bool cuspbright = spherecolor[0] + spherecolor[1] + spherecolor[2] > 2.f;
  cuspcolor = create_mat_color(A3dVertexColor(A3dColor(spherecolor[0], spherecolor[1], spherecolor[2]),
                                              A3dColor(spherecolor[0], spherecolor[1], spherecolor[2]),
                                              A3dColor((cuspbright ? 1.f : 7.f), 0.f, 0.f)));
  meshcolor = create_mat_color(A3dVertexColor(A3dColor(meshcold[0], meshcold[1], meshcold[2]),
                                              A3dColor(meshcols[0], meshcols[1], meshcols[2]),
                                              A3dColor(meshcolp[0], meshcolp[1], meshcolp[2])));
  int mesha = int(meshcola[0] * 255.f + .5f);
  assertx(mesha >= 0 && mesha <= 255);
  meshcolor.d[3] = uint8_t(mesha);
  if (picture) nice_rendering = true;
  if (pm_filename != "") {
#if defined(DEF_PM)
    pm_mode = true;
    read_pm(pm_filename);
#else
    assertnever("not enabled");
#endif
  }
  if (sr_filename != "") {
#if defined(DEF_SR)
    sr_mode = true;
    use_dl = false;
    read_sr(sr_filename);
#else
    assertnever("not enabled");
#endif
  }
  if (sc_filename != "") {
#if defined(DEF_SC)
    sc_mode = true;
    read_sc(sc_filename);
#else
    assertnever("not enabled");
#endif
  }
  if (psc_filename != "") {
#if defined(DEF_SC)
    assertx(!sc_mode);
    read_psc(psc_filename);
    psc_mode = true;
#else
    assertnever("not enabled");
#endif
  }
  if (sc_gm_filename != "") {
#if defined(DEF_SC)
    assertx(!sc_mode && !psc_mode);
    read_sc_gm(sc_gm_filename);
    sc_gm_mode = true;
    use_dl = false;
#else
    assertnever("not enabled");
#endif
  }
  if (ply_filename != "") {
#if defined(DEF_SC)
    read_ply(ply_filename);
    ply_mode = true;
#else
    assertnever("not enabled");
#endif
  }
  dummy_use(lod_use_nvertices);
  return true;
}

void HB::set_window_title(string s) { hw.set_window_title(std::move(s)); }

void HB::open() {
  // hw.set_default_background("#5987B3");  // .35 .53 .70; "hhblue" == 89, 135, 179
  hw.set_default_background("white");
  hw.set_default_foreground("black");
  hw.set_default_geometry(k_default_geometry);
  hw.set_double_buffering(dbuffer);
  hw.open();
}

void HB::watch_fd0(void (*pfinpu)()) {
  hw._finpu = pfinpu;
  hw.watch_fd0(!!hw._finpu);
}

void HB::quit() { hw.quit(); }

void HB::redraw_later() { hw.redraw_later(); }

void HB::redraw_now() { hw.redraw_now(); }

Vec2<int> HB::get_extents() { return win_dims; }

bool HB::get_pointer(Vec2<float>& yxf) {
  Vec2<int> yx;
  if (!hw.get_pointer(yx)) return false;
  yxf = convert<float>(yx) / convert<float>(win_dims);
  if (0) SHOW(yxf);
  return true;
}

void HB::set_camera(const Frame& p_real_t, float p_real_zoom, const Frame& p_view_t, float p_view_zoom) {
  tpos = p_real_t;
  tcam = p_view_t;
  real_zoom = p_real_zoom;
  view_zoom = p_view_zoom;
  if (uvtopos) {
    tpos = Frame(Vector(0.f, 0.f, -1.f), Vector(0.f, 1.f, 0.f), Vector(1.f, 0.f, 0.f), Point(0.5f, 0.5f, 2.5f));
    real_zoom = 0.2f;
    tcam = tpos;
    view_zoom = real_zoom;
    hither = 0.1f;
  }
  tcami = inverse(tcam);
  if (outside_frustum) view_zoom *= frustum_frac;
}

float HB::get_hither() { return hither; }

float HB::get_yonder() { return yonder; }

void HB::set_hither(float h) {
  if (uvtopos) {
    hither = 0.1f;
    return;
  }
  hither = h;
  if (!hither) hither = k_default_hither;
}

void HB::set_yonder(float y) {
  yonder = y;
  if (!yonder) yonder = k_default_yonder;
}

void HB::set_current_object(int obn) {
  // lighting feature ?
  cob = obn;
}

void HB::update_seg(int segn, const Frame& f, bool vis) {
  assertx(segn >= 0 && segn < k_max_object);
  g_xobs.t[segn] = f;
  g_xobs.vis[segn] = vis;
}

void HB::draw_space() {
  static bool is_init = false;
  if (!is_init) {
    is_init = true;
    string extensions = gl_extensions_string();
    if (getenv_bool("OPENGL_DEBUG")) SHOW(extensions);
    g_is_ati = contains(extensions, "GL_ATI_envmap_bumpmap");
    if (getenv_bool("OPENGL_DEBUG")) SHOW(g_is_ati);
  }
  space_init();
  if (movie_nframes) {
    g3d::lod_level = float(movie_frame) / (movie_nframes - 1.f);
    if (1) {
      // disabled 20110114 for terrain which became blank since no LOD
      // re-enabled 20120306 to create movie from LOD slider; added check "if (!lod_mode)" in G3ddraw.cpp
      g3d::update_lod();
    }
    // HB::segment_morph_mesh(1, g3d::lod_level);
  }
#if defined(DEF_SR)
  // I checked: I believe there are no graphics calls before this since the last swapbuffer.
  if (sr_mode) sr_pre_space();
#endif
  // I noticed a 12% slowdown with gcanyon_4k2k_fly2.frames when I
  //  placed the ClearWindow() before SRMesh::adapt_refinement() !
  hw.clear_window();  // computation done before this!
  wrap_draw(false);
  if (picture && !g3d::input) {
    if (!dbuffer) {
      inpicture = true;
    } else {
      // must generate some dummy frames
      static int count = 0;
      // SHOW(count);
      HB::redraw_later();
      if (count++ == 2) inpicture = true;
    }
  }
  gl_init();
  draw_all();
  gl_fixup();
  if (outside_frustum) {
    float xrad = .5f / frustum_frac;
    float x1 = .5f - xrad, x2 = .5f + xrad;
    float yrad = .5f / frustum_frac;
    float y1 = .5f - yrad, y2 = .5f + yrad;
    HB::draw_segment(V(y1, x1), V(y1, x2));
    HB::draw_segment(V(y2, x1), V(y2, x2));
    HB::draw_segment(V(y1, x1), V(y2, x1));
    HB::draw_segment(V(y1, x2), V(y2, x2));
    if (thicknormal > 1) {
      Vec2<float> yxd = twice(1.f) / convert<float>(win_dims);
      x1 -= yxd[1];
      x2 += yxd[1];
      y1 -= yxd[0];
      y2 += yxd[0];
      HB::draw_segment(V(y1, x1), V(y1, x2));
      HB::draw_segment(V(y2, x1), V(y2, x2));
      HB::draw_segment(V(y1, x1), V(y2, x1));
      HB::draw_segment(V(y1, x2), V(y2, x2));
    }
  }
  if (inpicture) {
    process_print();
    if (movie_nframes) {
      if (movie_frame < movie_nframes - 1) {
        movie_frame++;
        movie_cprogress->update(float(movie_frame) / (movie_nframes - 1));
      } else {
        movie_cprogress = nullptr;
        inpicture = false;
        if (movie_video) {
          movie_video->write_file(movie_rootname);
          movie_video = nullptr;
        }
        HB::quit();
      }
    } else {
      inpicture = false;
      HB::quit();
    }
  }
  if (noinfo) {
    g3d::info = false;
  } else {
    wrap_draw(true);
  }
  assertw(!gl_report_errors());
}

bool HB::special_keypress(char ch) {
#if defined(DEF_SR)
  if (sr_mode && sr_key_press(ch)) return true;
#endif
#if defined(DEF_PM)
  if (pm_mode && pm_key_press(ch)) return true;
#endif
#if defined(DEF_SC)
  if (psc_mode && psc_key_press(ch)) return true;
#endif
  switch (ch) {
    case 'b':
      toggle_attribute(g_xobs.cullface);
      invalidate_dls();
      hw.redraw_now();
      break;
    case 'r':
      toggle_attribute(g_xobs.reverse_cull);
      invalidate_dls();
      hw.redraw_now();
      break;
    case 's':
      toggle_attribute(g_xobs.shading);
      invalidate_dls();
      hw.redraw_now();
      break;
    case 'm':
      toggle_attribute(g_xobs.smooth);
      invalidate_dls();
      hw.redraw_now();
      break;
    case 'e':
      toggle_attribute(g_xobs.edges);
      invalidate_dls();
      hw.redraw_now();
      break;
    case 'u':
      mdepthcue = !mdepthcue;
      hw.redraw_now();
      break;
    case 'a':
      antialiasing = !antialiasing;
      invalidate_dls();
      hw.redraw_now();
      break;
    case 'n':
      nice_rendering = !nice_rendering;
      invalidate_dls();
      hw.redraw_now();
      break;
    case 'p':
      perspec = !perspec;
      hw.redraw_now();
      break;
    case 'S':
      slidermode = !slidermode;
      if (slidermode) use_dl = false;
      hw.redraw_now();
      break;
    case 'o':
      outside_frustum = !outside_frustum;
      hw.redraw_now();
      break;
    case 'Q':
      butquick = !butquick;
      quickmode = false;
      invalidate_dls();
      hw.redraw_now();
      break;
    case 'q':
      quickmode = !quickmode;
      butquick = false;
      invalidate_dls();
      hw.redraw_now();
      break;
    case 't':
      texture_active = !texture_active;
      invalidate_dls();
      hw.redraw_now();
      break;
    case 'y':
      update_anisotropy();
      hw.redraw_now();
      break;
    case 'U':
      uvtopos = !uvtopos;
      special_keypress('s');
      special_keypress('e');
      invalidate_dls();
      hw.redraw_now();
      break;
    case 'C':
      use_dl = !use_dl;
      invalidate_dls();
      was_using_dl = false;
      hw.redraw_now();
      break;
    case 'T':
      strip_lines = (strip_lines + 1) % 3;  // 0..2
      invalidate_dls();
      hw.redraw_now();
      break;
    case ']':
      quicki *= 2;
      invalidate_dls();
      if (quickmode || butquick) hw.redraw_now();
      break;
    case '[':
      quicki /= 2;
      if (!quicki) quicki = 1;
      invalidate_dls();
      if (quickmode || butquick) hw.redraw_now();
      break;
    case 'E': {
      static int bu_thicknormal = 1;
      if (thicknormal) {
        bu_thicknormal = thicknormal;
        thicknormal = 0;
      } else {
        thicknormal = bu_thicknormal;
      }
      invalidate_dls();
      hw.redraw_now();
      break;
    }
    case '/': {
      string s = g3d::statefile;
      if (hw.query(V(30, 2), "Stateg3d:", s)) g3d::statefile = s;
      break;
    }
    case 'R': {
      string s = imagefilename;
      if (hw.query(V(30, 2), "imagefilename:", s)) imagefilename = s;
      break;
    }
    case 'P': process_print(); break;
    case 'Z':
      all_reset();
      invalidate_dls();
      hw.redraw_now();
      break;
    case '\r':  // <enter>/<ret> key (== uchar{13} == 'M' - 64)
    case '\n':  // G3d -key $'\n'
      static bool g_fullscreen;
      g_fullscreen = !g_fullscreen;
      hw.make_fullscreen(g_fullscreen);
      break;
    case '?': {
      const string s = &R"(
Device commands (prefixed by 'D'):
   Per object:
<b>ackfacecull  <r>eversecull  <s>hading  s<m>ooth_shading  <e>dges
   Global:
depthc<u>e  <a>ntialiasing  <n>ice_rendering  <p>erspective  <S>liders
<q>uickmode  button<Q>uick   <[>, <]>:change_quicki
</>statefile  set<R>enderedimage  <P>rint_image  <cntrl-C>quit
)"[1];
      std::cerr << s;
      break;
    }
    default: return false;
  }
  return true;
}

string HB::show_info() {
  int obn = cob;
  return sform("[GL %c%c%c%c%c%c%c%c%c%c%c%c%c]", g_xobs.cullface[obn] ? 'b' : ' ',
               g_xobs.reverse_cull[obn] ? 'r' : ' ', g_xobs.shading[obn] ? 's' : ' ', g_xobs.smooth[obn] ? 'm' : ' ',
               g_xobs.edges[obn] ? 'e' : ' ', mdepthcue ? 'u' : ' ', antialiasing ? 'a' : ' ',
               nice_rendering ? 'n' : ' ', perspec ? 'p' : ' ', slidermode ? 'S' : ' ', texture_active ? 't' : ' ',
               use_dl ? 'C' : ' ', quickmode ? 'q' : butquick ? 'Q' : ' ');
}

bool HB::world_to_vdc(const Point& pi, float& xo, float& yo, float& zo) {
  Point p = pi * tcami;
  zo = p[0];
  if (p[0] < hither) return false;
  xo = .5f - p[1] / p[0] * tzp[1];
  yo = .5f - p[2] / p[0] * tzp[0];
  return true;
}

void HB::draw_segment(const Vec2<float>& yx1, const Vec2<float>& yx2) {
  hw.draw_segment(yx1 * convert<float>(win_dims), yx2 * convert<float>(win_dims));
}

Vec2<int> HB::get_font_dims() { return hw.get_font_dims(); }

void HB::draw_text(const Vec2<float>& yx, const string& s) {
  hw.draw_text(convert<int>(floor(yx * convert<float>(win_dims))), s);
}

void HB::draw_row_col_text(const Vec2<int>& yx, const string& s) {
  auto fdims = HB::get_font_dims();
  hw.draw_text(V((yx[0] < 0 ? win_dims[0] + yx[0] * fdims[0] : yx[0] * fdims[0]),
                 (yx[1] == INT_MAX ? (win_dims[1] - narrow_cast<int>(s.size()) * fdims[1]) / 2
                                   : yx[1] < 0 ? win_dims[1] + yx[1] * fdims[1] - 2 : yx[1] * fdims[1] + 2)),
               s);
}

void HB::clear_segment(int segn) { g_xobs.clear(segn); }

void HB::open_segment(int segn) { g_xobs.open(segn); }

void HB::segment_add_object(const A3dElem& el) { g_xobs.add(el); }

void HB::close_segment() { g_xobs.close(); }

void HB::segment_attach_mesh(int segn, GMesh* pmesh) {
  if (!assertw(g_xobs.defined(segn))) return;
  g_xobs[segn].pmesh = pmesh;
}

void HB::make_segment_link(int oldsegn, int newsegn) { g_xobs.make_link(oldsegn, newsegn); }

void HB::segment_morph_mesh(int segn, float finterp) {
  lshading = g_xobs.shading[segn];
  lsmooth = g_xobs.smooth[segn];
  ledges = g_xobs.edges[segn];
  lcullface = g_xobs.cullface[segn];
  if (!lshading && ledges) lsmooth = false;  // cheaper
  g_xobs[segn].morph(finterp);
}

void HB::reload_textures() {
  for_int(i, g_textures.num()) glDeleteTextures(1, &g_textures[i].texname);
  g_textures.clear();
  textures_loaded = false;
}

void HB::flush() { hw.hard_flush(); }

void HB::beep() { hw.beep(); }

int HB::id() { return 2000; }

void* HB::escape(void* code, void* data) {
  int icode = narrow_cast<int>(reinterpret_cast<intptr_t>(code));
  float fdata = *static_cast<float*>(data);
  dummy_use(fdata);
  switch (icode) {
    case 1: {
      if (fdata < 0.f) {
        if (g3d::output) {
          std::cout << "lod "
                    << "-1\n";
          std::cout.flush();
        }
        if (!use_dl) use_dl = was_using_dl;
      } else {
        if (use_dl) {
          was_using_dl = use_dl;
          use_dl = false;
        }
#if defined(DEF_PM)
        if (pm_mode) pm_set_lod(fdata);
#endif
#if defined(DEF_SC)
        if (psc_mode) psc_set_lod(fdata);
        if (sc_gm_mode) sc_gm_set_lod(fdata);
#endif
      }
      break;
    }
    case 2:
#if defined(DEF_SR)
      if (sr_mode) sr_screen_thresh = fdata;
#endif
      break;
    default: assertnever("");
  }
  return nullptr;
}

namespace {

// *** PM mode

#if defined(DEF_PM)

// variables
PMesh pmesh;
unique_ptr<RFile> g_fi;
unique_ptr<PMeshRStream> pmrs;
unique_ptr<PMeshIter> pmi;
float pm_lod_level = getenv_float("PM_LOD_LEVEL", 0.f);

void read_pm(const string& filename) {
  // HH_TIMER(_read_pm);
  // RFile fi(filename);
  if (filename == "-") g3d::keep_stdin_open = true;
  g_fi = make_unique<RFile>(filename);
  pmrs = make_unique<PMeshRStream>((*g_fi)(), &pmesh);
  pmi = make_unique<PMeshIter>(*pmrs);
  if (0 && pm_lod_level == 0.f) pm_lod_level = 1.f;
  pmi->goto_nvertices(pmrs->base_mesh()._vertices.num() + int(pmrs->_info._tot_nvsplits * pm_lod_level + .5f));
  slidermode = true;
  g3d::UpdateOb1Bbox(pmrs->_info._full_bbox);
  if (g3d::g_filename == "") g3d::g_filename = filename;
}

void pm_update_lod() {
  float flevel = min(pm_lod_level, 1.f);
  int nv0 = pmesh._base_mesh._vertices.num();
  int nvsplits = pmesh._info._tot_nvsplits;
  int nv = nv0 + int((nvsplits + 1) * flevel * .999999f);
  pmi->goto_nvertices(nv);
  invalidate_dls();
  if (g3d::output) {
    float val = lod_use_nvertices ? pmi->_vertices.num() : pm_lod_level;
    std::cout << "lod " << val << '\n';
    std::cout.flush();
  }
}

void pm_wrap_draw(bool show) {
  if (!slidermode) return;
  if (!show) {
    if (button_active && yx_pointer_old[1] < k_one_slider_left_thresh) {
      Vec2<float> yxf;
      assertx(HB::get_pointer(yxf));
      float oldval = pm_lod_level;
      switch (button_active) {
        case 1: {
          pm_lod_level = 1.1f - (yxf[0]) * 1.2f;
          break;
        }
        case 2: {
          float a = (yxf[0] - 0.5f) * -3.f;
          float a2 = pow(abs(a), 4.f) * sign(a) * g3d::fchange * 1.f;
          pm_lod_level += a2;
          break;
        }
        case 3: {
          const float factor = 5.f;
          float val = 1.1f - (yxf[0]) * 1.2f;
          val = max(val, 0.f);
          pm_lod_level = (exp(factor * val) - 1.0f) / (exp(factor) - 1.0f);
          break;
        }
        default: assertnever("");
      }
      pm_lod_level = clamp(pm_lod_level, 0.f, 1.f);
      if (pm_lod_level != oldval) pm_update_lod();
      hw.redraw_later();
    }
  } else {
    int lmargin = !bigfont() ? 6 : 3;
    if (win_dims[1] < 400 && !bigfont()) lmargin -= 3;
    HB::draw_row_col_text(V(1, lmargin), "LOD");
    HB::draw_row_col_text(V(2, lmargin), sform(" %.3f", pm_lod_level));
    HB::draw_row_col_text(V(1, lmargin + 8), "#Faces");
    HB::draw_row_col_text(V(2, lmargin + 8), sform(" %d", pmi->_faces.num()));
    // HB::draw_row_col_text(V(4, 21), "#Verts");
    // HB::draw_row_col_text(V(5, 22), sform("%d", pmi->_vertices.num()));
    float yline = .004f, xleft = .05f;
    {  // current level
      float lod = clamp(pm_lod_level, 0.f, 1.f);
      float x1 = xleft + .01f, x2 = xleft + .05f;
      float y1 = (1.1f - lod) / 1.2f - .002f;
      float y2 = y1 + yline;
      HB::draw_segment(V(y1, x1), V(y1, x2));
      HB::draw_segment(V(y2, x1), V(y2, x2));
      HB::draw_segment(V(y1, x1), V(y2, x1));
      HB::draw_segment(V(y1, x2), V(y2, x2));
    }
    {  // slider
      float y1 = (1.1f - 0) / 1.2f + .004f, y2 = (1.1f - 1) / 1.2f - .004f, yd = .01f;
      float x1 = xleft + .02f, x2 = xleft + .04f, xm = xleft + .03f;
      HB::draw_segment(V(y1 + yd, x1), V(y1, xm));
      HB::draw_segment(V(y1 + yd, x2), V(y1, xm));
      HB::draw_segment(V(y1 + yd, x1), V(y1 + yd, x2));
      HB::draw_segment(V(y2 - yd, x1), V(y2, xm));
      HB::draw_segment(V(y2 - yd, x2), V(y2, xm));
      HB::draw_segment(V(y2 - yd, x1), V(y2 - yd, x2));
      HB::draw_segment(V(y1, xm), V(y2, xm));
    }
  }
}

void pm_set_lod(float lod) {
  if (lod_use_nvertices) {
    int nv = int(lod);
    int nv0 = pmesh._base_mesh._vertices.num();
    int nvsplits = pmesh._info._tot_nvsplits;
    if (!assertw(nvsplits)) return;
    pm_lod_level = (float(nv) - nv0) / nvsplits;
  } else {
    pm_lod_level = lod;
  }
  if (!assertw(pm_lod_level >= 0.f)) pm_lod_level = 0.f;
  if (!assertw(pm_lod_level <= 1.f)) pm_lod_level = 1.f;
  pm_update_lod();
  hw.redraw_later();
}

void draw_pm() {
  AWMesh& wmesh = *pmi;
  if (!ledges || lshading) {
    // Options lsmooth, lquickmode not handled.
    if (!pmesh._info._has_rgb) assertw(lsmooth);
    glShadeModel(lsmooth || pmesh._info._has_rgb ? GL_SMOOTH : GL_FLAT);
    initialize_lit();
    update_mat_color(meshcolor);
    if (texture_active) {
      if (!texture_lit) {
        glShadeModel(GL_FLAT);
        initialize_unlit();
        update_cur_color(meshcolor.d);
        set_light_ambient(0.f);
        if (texturenormal) normalmap_activate();
      }
    }
    // static const int pm_strips = getenv_int("PM_STRIPS");
    // 20020528: it now always seems faster with strips and it greatly helps when Display Lists are enabled.
    static const bool pm_strips = !getenv_bool("PM_NOSTRIPS");
    if (!pm_strips || pmesh._info._has_rgb) {
      if (0) Warning("PMesh: no strips for rendering (may be slower)");
      wmesh.ogl_render_faces_individually(pmesh._info, texture_active);
    } else if (defining_dl && g_is_ati) {
      // Much faster to create display list, since fewer primitives.
      wmesh.ogl_render_faces_individually(pmesh._info, texture_active);
    } else {
      wmesh.ogl_render_faces_strips(pmesh._info, texture_active);
    }
    initialize_unlit(), initialize_lit();  // re-synchronize
  }
  if (texture_active) {
    glDisable(GL_TEXTURE_2D);
    set_light_ambient(ambient);
    if (texturenormal) normalmap_deactivate();
  }
  if (ledges && !texture_active) {
    // Options cullbackedges, lquickmode not handled.
    glShadeModel(GL_FLAT);
    initialize_unlit();
    set_thickness(thicknormal);
    update_cur_color(pix_edgecolor);
    wmesh.ogl_render_edges();
    set_thickness(thicka3d);
  }
}

bool pm_key_press(char ch) {
  switch (ch) {
    case 'c':
      pm_lod_level = 0.f;
      pm_update_lod();
      if (g3d::output) {
        std::cout << "lod "
                  << "-1\n";
        std::cout.flush();
      }
      hw.redraw_now();
      break;
    case 'a':
      pm_lod_level = 1.f;
      pm_update_lod();
      if (g3d::output) {
        std::cout << "lod "
                  << "-1\n";
        std::cout.flush();
      }
      hw.redraw_now();
      break;
    default: return false;
  }
  return true;
}

#endif  // defined(DEF_PM)

// *** SR mode

#if defined(DEF_SR)

SRMesh srmesh;
int sr_ntstrips;
int sr_ncachemiss;
bool sr_freeze;
bool sr_radar;
bool sr_striplines;
double sr_refinement_time;
constexpr float sr_hither_radar = 1.0f;
const string sr_stats_filename = getenv_string("G3D_SR_STATS");
float sr_radar_old_hither;
bool sr_morph_active;

void read_sr(const string& filename) {
  HH_TIMER(_read_sr);
  RFile fi(filename);
  for (string sline; fi().peek() == '#';) assertx(my_getline(fi(), sline));
  assertx(fi().peek() == 'P' || fi().peek() == 'S');
  bool srm_input = fi().peek() == 'S';
  if (!srm_input) {
    PMeshRStream local_pmrs(fi());
    // Note that there is no memory-resident PMesh; great!
    srmesh.read_pm(local_pmrs);
  } else {
    srmesh.read_srm(fi());
  }
  g3d::UpdateOb1Bbox(srmesh.get_bbox());
  if (g3d::g_filename == "") g3d::g_filename = filename;
}

void sr_regulator() {
  float goal_nfaces = sr_regulatenf;
  if (!goal_nfaces) return;
  int cur_nfaces = srmesh.num_active_faces();
  const float minpixtol = 0.5f;
  if (!product(win_dims)) {  // window is iconified on _WIN32
                             // don't modify sr_screen_thresh
  } else {
    float min_screen_thresh = 2.0f * minpixtol / min(win_dims);
    if (abs(cur_nfaces - goal_nfaces) > 10) sr_screen_thresh *= pow(cur_nfaces / goal_nfaces, sr_gain);
    sr_screen_thresh = max(sr_screen_thresh, min_screen_thresh);
  }
  float pixtol = sr_screen_thresh * min(win_dims) * .5f;
  static int count;
  if (++count > 50) {  // ignore the transient behavior in the first 50 frames
    HH_SSTAT(Stau, pixtol);
  }
}

void sr_adapt_refinement() {
  // HH_ATIMER(_sr_adapt_refinement);
  // Timer timer;
  // Note: called before setup_ob(), so we must form the parameters we need here ourselves!
  {
    SRViewParams vp;
    vp.set_frame(tpos * inverse(g_xobs.t[1]));
    if (!product(win_dims)) {  // window is iconified on _WIN32
      vp.set_zooms(twice(1.f));
    } else {
      vp.set_zooms(real_zoom * convert<float>(win_dims) / float(min(win_dims)));
    }
    vp.set_screen_thresh(sr_screen_thresh);
    float true_hither = (!sr_radar ? hither : hither < 1.f ? hither : sr_radar_old_hither);
    vp.set_hither(true_hither);
    vp.set_yonder(yonder == k_default_yonder ? -1.f : yonder);
    srmesh.set_view_params(vp);
  }
  {
    // const float upper_limit = 1.1f;
    // int max_active_faces = sr_regulatenf ? sr_regulatenf * upper_limit : INT_MAX;
    int nvtrav = sr_fracvtrav >= 1.f ? INT_MAX : int(srmesh.num_active_vertices() * sr_fracvtrav);
    srmesh.adapt_refinement(nvtrav);
  }
  // timer.stop();
  // sr_refinement_time = timer.real();
  sr_refinement_time = 0.;
  // if (sr_morph_active) hw.redraw_later();
  // NOTE: if sr_fractrav<1.f, should somehow detect if the entire active
  //  vertex list has been traversed over several frames without
  //  any changes!
  if (srmesh.is_still_morphing() || srmesh.is_still_adapting()) hw.redraw_later();
}

unique_ptr<WFile> sr_stats_file;

void sr_pre_space() {
  if (sr_stats_filename != "") {
    // Output stats on previously drawn frame.
    if (!sr_stats_file) {
      sr_stats_file = make_unique<WFile>(sr_stats_filename);
    } else {
      // We have a previous frame.
      std::ostream& os = (*sr_stats_file)();
      os << sform("%6.4f %6.4f %6d %7.5f %4.2f\n", sr_refinement_time, g3d::fchange, srmesh.num_active_faces(),
                  sr_screen_thresh, float(sr_ncachemiss) / srmesh.num_active_faces());
    }
  }
  {
    static const bool g3d_sr_stats = getenv_bool("G3D_SR_STATS2");
    if (g3d_sr_stats) {
      HH_SSTAT(Ssr_vgr, float(srmesh.num_vertices_refine_morphing()) / srmesh.num_active_vertices());
      HH_SSTAT(Ssr_vgc, float(srmesh.num_vertices_coarsen_morphing()) / srmesh.num_active_vertices());
    }
  }
  {
    static const int g3d_dump_frame = getenv_int("G3D_DUMP_FRAME");
    static int frame;
    if (g3d_dump_frame && frame++ == g3d_dump_frame) {
      SHOW("dumping v.m and my_sleep(10.)");
      GMesh gmesh;
      srmesh.extract_gmesh(gmesh);
      WFile fi("v.m");
      gmesh.write(fi());
      my_sleep(10.);
    }
  }
  if (!sr_freeze) {
    sr_regulator();
    sr_adapt_refinement();
  }
}

void draw_sr() {
  // Options lsmooth, lquickmode not handled.
  if (sr_striplines) enable_polygon_offset();
  sr_ntstrips = 0;
  sr_ncachemiss = 0;
  static const bool sr_no_strips = getenv_bool("SR_NO_STRIPS");
  static const bool sr_use_tvc = getenv_bool("SR_USE_TVC");
  if (!ledges || lshading) {
    assertw(lsmooth);
    glShadeModel(lsmooth ? GL_SMOOTH : GL_FLAT);
    initialize_lit();
    update_mat_color(meshcolor);
    if (texture_active) {
      if (!texture_lit) {
        glShadeModel(GL_FLAT);
        initialize_unlit();  // disable lighting
        update_cur_color(meshcolor.d);
        set_light_ambient(0.f);
        if (texturenormal) normalmap_activate();
      }
    }
    if (sr_no_strips) {
      Warning("SRMesh: not using strips for rendering");
      srmesh.ogl_render_faces_individually(texture_active && !texture_lit);
    } else if (!sr_use_tvc) {
      srmesh.ogl_render_faces_strips(texture_active && !texture_lit);
    } else {
      srmesh.ogl_render_faces_tvc(texture_active && !texture_lit);
    }
    initialize_unlit(), initialize_lit();  // re-synchronize
  }
  if (texture_active) {
    glDisable(GL_TEXTURE_2D);
    set_light_ambient(ambient);
    if (texturenormal) normalmap_deactivate();
  }
  if (sr_striplines) {
    assertw(!texture_active);
    glShadeModel(GL_FLAT);
    initialize_unlit();
    update_cur_color(pix_sharpedgecolor);
    set_thickness(thicksharp);
    if (!sr_use_tvc) {
      sr_ntstrips = srmesh.ogl_render_striplines();
    } else {
      sr_ncachemiss = srmesh.ogl_render_tvclines();
    }
  }
  if (ledges && !texture_active) {
    // Options cullbackedges, lquickmode not handled.
    glShadeModel(GL_FLAT);
    initialize_unlit();
    update_cur_color(pix_edgecolor);
    set_thickness(thicknormal);
    srmesh.ogl_render_edges();
    set_thickness(thicka3d);
  }
  static const bool show_radii = getenv_bool("SHOW_RADII");
  if (show_radii) {
    glShadeModel(GL_FLAT);
    initialize_unlit();
    update_cur_color(pix_edgecolor);
    srmesh.ogl_show_radii();
  }
  static const int show_residuals = getenv_int("SHOW_RESIDUALS");  // 0, 1, 2
  if (show_residuals) {
    glShadeModel(GL_FLAT);
    initialize_unlit();
    update_cur_color(pix_edgecolor);
    set_thickness(thicknormal);
    srmesh.ogl_show_residuals(show_residuals >= 2);
    set_thickness(thicka3d);
  }
}

void sr_wrap_draw(bool show) {
  if (sr_radar) {
    static float sr_bu_hither;
    static float sr_bu_yonder;
    if (!show) {
      sr_bu_hither = hither;
      hither = sr_hither_radar;  // to avoid shimmering
      sr_bu_yonder = yonder;
      yonder = k_default_yonder;  // to be able to see in radar view
    } else {
      hither = sr_bu_hither;
      yonder = sr_bu_yonder;
    }
  }
  if (show && g3d::info) {
    int nf = srmesh.num_active_faces();
    float pixtol = sr_screen_thresh * min(win_dims) * .5f;
    if (outside_frustum) pixtol /= frustum_frac;
    bool morph = sr_morph_active;
    string s1 = sform("faces=%-5d", nf);
    string s2 = !bigfont() || !morph ? sform(" pixtol=%4.2f", pixtol) : sform(" pix=%4.2f", pixtol);
    string s3 =
        (!morph ? ""
                : !bigfont() ? sform(" morph%02d vgr=%04.1f%% vgc=%04.1f%%", sr_gtime,
                                     srmesh.num_vertices_refine_morphing() * 100.f / srmesh.num_active_vertices(),
                                     srmesh.num_vertices_coarsen_morphing() * 100.f / srmesh.num_active_vertices())
                             : sform("vgr=%04.1f%% vgc=%04.1f%%",
                                     srmesh.num_vertices_refine_morphing() * 100.f / srmesh.num_active_vertices(),
                                     srmesh.num_vertices_coarsen_morphing() * 100.f / srmesh.num_active_vertices()));
    string s4 =
        (sr_ntstrips ? sform(" f/strip=%4.1f", float(nf) / sr_ntstrips)
                     : sr_ncachemiss ? sform(" v/t=%4.2f", float(sr_ncachemiss) / srmesh.num_active_faces()) : "");
    if (!bigfont()) {
      HB::draw_row_col_text(V(1, 4), s1 + s2 + s3 + s4);
    } else {
      if (!morph) {
        s3 = s2.substr(1);
        s2 = "";
      }
      HB::draw_row_col_text(V(1, 0), s1 + s2);
      HB::draw_row_col_text(V(2, 0), s3);
      HB::draw_row_col_text(V(3, 0), s4);
    }
  }
  if (slidermode) {
    Array<Slider> sliders;
    if (sr_screen_thresh) sliders.push(Slider{"screen_thresh", &sr_screen_thresh});
    if (sr_regulatenf) sliders.push(Slider{"regulatenf", &sr_regulatenf});
    if (sr_gain) sliders.push(Slider{"gain", &sr_gain});
    if (sr_fracvtrav) sliders.push(Slider{"fracvtrav", &sr_fracvtrav});
    if (outside_frustum) sliders.push(Slider{"frustum_frac", &frustum_frac});
    if (!show) {
      if (button_active) {
        Vec2<float> yxf;
        assertx(HB::get_pointer(yxf));
        float dval = exp((yxf[0] - yx_pointer_old[0]) * -1.5f * g3d::fchange);
        int i = int(yx_pointer_old[1] * sliders.num() * .9999f);
        *sliders[i].val *= dval;
        hw.redraw_later();
      }
    } else {
      if (!bigfont() || g3d::info == 2) {
        for_int(i, sliders.num()) {
          float x = (i + .2f) / sliders.num();
          HB::draw_text(V(.12f, x), sliders[i].name);
          HB::draw_text(V(.17f, x), sform("%g", *sliders[i].val));
        }
      } else {
        HB::draw_text(V(.22f, .05f), "MODIFY PARAMS");
      }
    }
  }
  if (!show && g3d::output) {
    static float old_screen_thresh;
    if (sr_screen_thresh != old_screen_thresh) {
      old_screen_thresh = sr_screen_thresh;
      std::cout << "screen_thresh " << sr_screen_thresh << '\n';
      std::cout.flush();
    }
  }
}

void sr_update_morph_times() {
  int refine_time = sr_morph_active ? sr_gtime : 0;
  int coarsen_time = sr_morph_active ? sr_gtime / 2 : 0;
  if (coarsen_time == 1) coarsen_time = 2;
  srmesh.set_refine_morph_time(refine_time);
  srmesh.set_coarsen_morph_time(coarsen_time);
  if (0) showf("sr_morph %s (%d)\n", (sr_morph_active ? "active" : "inactive"), sr_gtime);
}

bool sr_key_press(char ch) {
  switch (ch) {
    case 'f':
      sr_freeze = !sr_freeze;
      hw.redraw_now();
      break;
    case 'a':
      srmesh.fully_refine();
      sr_freeze = true;
      hw.redraw_now();
      break;
    case 'c':
      srmesh.fully_coarsen();
      sr_freeze = true;
      hw.redraw_now();
      break;
    case 'g':
      sr_morph_active = !sr_morph_active;
      sr_update_morph_times();
      hw.redraw_now();
      break;
    case 'G':
      if (sr_gtime == 256)
        sr_gtime = 2;
      else
        sr_gtime *= 2;
      sr_update_morph_times();
      hw.redraw_now();
      break;
    case 'T': {
      static bool striplines_old_edges;
      static bool striplines_old_shading;
      static bool striplines_old_textureactive;
      sr_striplines = !sr_striplines;
      if (sr_striplines) {
        striplines_old_edges = g_xobs.edges[1];
        striplines_old_shading = g_xobs.shading[1];
        striplines_old_textureactive = texture_active;
        g_xobs.edges[1] = true;
        g_xobs.shading[1] = false;
        texture_active = false;
      } else {
        g_xobs.edges[1] = striplines_old_edges;
        g_xobs.shading[1] = striplines_old_shading;
        texture_active = striplines_old_textureactive;
      }
      hw.redraw_now();
      break;
    }
    case 'R':
      if (!sr_regulatenf) {
        sr_regulatenf = float(srmesh.num_active_faces());
      } else {
        sr_regulatenf = 0.f;
      }
      hw.redraw_now();
      break;
    case 'r': {
      Frame sr_tview;
      if (0)
        sr_tview = Frame(Vector(0.43588989f, 0.f, -0.9f), Vector(0.f, 1.f, 0.f), Vector(0.9f, 0.f, 0.43588989f),
                         Point(-0.65f, 0.f, 2.48f));
      const float vv = 0.25f;
      if (1)
        sr_tview = Frame(Vector(vv, 0.f, -sqrt(1.f - square(vv))), Vector(0.f, 1.f, 0.f),
                         Vector(sqrt(1.f - square(vv)), 0.f, vv), Point(-0.25f, 0.f, 2.8f));
      if (getenv_bool("G3D_RADAR_LOWER_SKEWED_RIGHT"))
        sr_tview = Frame(Vector(0.249389f, 0.0699849f, -0.965879f), Vector(-0.0175022f, 0.997555f, 0.0677609f),
                         Vector(0.968246f, 6.13302e-06f, 0.25f), Point(-0.0890237f, 0.021173f, 1.53995f));
      // float diam = srmesh._bbox.max_side();
      // diam == 0.823263 for teapot20.b.pm (for SIGGRAPH 97 images)
      // sr_tview = Frame::scaling(thrice(1.f / diam)) * sr_tview;  doesn't work.
      //
      static Frame radar_old_tview;
      static bool radar_old_edges;
      static bool radar_old_textureactive;
      static bool radar_old_outside_frustum;
      //
      sr_radar = !sr_radar;
      if (sr_radar) {
        radar_old_tview = g3d::tview;
        radar_old_edges = g_xobs.edges[1];
        sr_radar_old_hither = hither;
        radar_old_textureactive = texture_active;
        radar_old_outside_frustum = outside_frustum;
        g3d::tview = sr_tview;
        g_xobs.edges[1] = true;
        // hither = sr_hither_radar;
        texture_active = false;
        if (g_xobs.min_segn() == 0 && g_xobs.defined(0)) g_xobs.vis[0] = true;
        outside_frustum = false;
      } else {
        g3d::tview = radar_old_tview;
        g_xobs.edges[1] = radar_old_edges;
        hither = sr_radar_old_hither;
        texture_active = radar_old_textureactive;
        if (g_xobs.min_segn() == 0 && g_xobs.defined(0)) g_xobs.vis[0] = false;
        outside_frustum = radar_old_outside_frustum;
      }
      hw.redraw_now();
      break;
    }
    case 'O':
      // Traverse all data structures to force into memory.
      srmesh.ok();
      break;
    case 'K':  // was 13 <enter>
      if (!sr_freeze) {
        assertx(sr_key_press('c'));
      } else {
        assertx(sr_key_press('f'));
      }
      return true;
    default: return false;
  }
  return true;
}

#endif  // defined(DEF_SR)

// *** SC mode

#if defined(DEF_SC)

class Cylinder {
 public:
  explicit Cylinder(int depth);
  void draw();
  void draw(const Point& p1, const Point& p2, float r);

 private:
  Array<Point> _v;   // verts
  Array<Vector> _n;  // normals
  // endcaps
  Point bc;  // bottom center
  Point tc;  // top center
  Vector tn;
  Vector bn;
};

// variables
struct NormalRecord {
  int fid;
  Vector fct_nor;
  Vec3<Vector> corner_nor;
};

Array<Array<NormalRecord>*> psc_unify_normal_list;
Array<Array<NormalRecord>*> psc_split_normal_list;
Array<Array<AreaData>*> psc_unify_area_list;
Array<SplitRecord> psc_lod_list;  // HH: last entry is null!
int psc_lod_num = 0;
float psc_lod_level = getenv_float("PSC_LOD_LEVEL", 0.f);
SimplicialComplex Kmesh;
Array<Vector> fct_pnor;
Array<Vector> corner_pnor;
Array<Pixel> s_color;
Array<int> s_norgroup;
Set<Simplex> psc_principal_edges;
Set<Simplex> psc_principal_verts;

int psc_orphan_nedges = 0;
int psc_orphan_nverts = 0;
Cylinder cyl{2};

// geomorph
float sc_gm_lod_level = 0.f;

void psc_update_lod();

// Determine vertex normal by averaging normals of adjacent faces belonging to the same normal group.
void vertSmoothNormal(Simplex vs, Simplex corner_fct, Vector& avg_norm) {
  Vec3<Simplex> verts;
  int ngroup = s_norgroup[corner_fct->getVAttribute()];
  corner_fct->vertices(verts.data());
  // find order of vs in verts
  int i_vs = -1;
  for_int(i, 3) {
    if (vs == verts[i]) i_vs = i;
  }
  assertx(i_vs >= 0);
  avg_norm = fct_pnor[corner_fct->getId()];
  // go around in one direction averaging normals
  // of adjacent facets with same normal group
  // the orientation is given by ordering of va and vb
  Simplex va = verts[i_vs];
  Simplex vb = verts[mod3(i_vs + 1)];
  Simplex e = va->edgeTo(vb);
  assertx(e);
  Simplex fct = corner_fct;
  bool done = false;
  while (e->isManifold()) {
    // find other facet around e
    ForSCSimplexParent(e, f) {
      if (f != fct) {
        fct = f;
        break;
      }
    }
    EndFor;
    // if the new fct is the corner_fct made a full circle
    // nothing left to do
    if (fct == corner_fct) {
      done = true;
      break;
    }
    // if new facet does not have same smoothing group
    if (s_norgroup[fct->getVAttribute()] != ngroup) break;
    Vec3<Simplex> verts2;
    fct->vertices(verts2.data());
    // find va among verts2
    int i_va = -1;
    for_int(i, 3) {
      if (va == verts2[i]) i_va = i;
    }
    assertx(i_va >= 0);
    if (verts2[mod3(i_va + 1)] == vb) {
      // va still before vb
      // inconsistent with previous fct
      // flip normal
      avg_norm -= fct_pnor[fct->getId()];
      vb = verts2[mod3(i_va + 2)];
    } else {
      // va after vb in the order
      // consistent with previous fct
      avg_norm += fct_pnor[fct->getId()];
      vb = verts2[mod3(i_va + 1)];
    }
    e = va->edgeTo(vb);
    assertx(e);
  }
  if (!done) {
    // go around in other direction averaging normals
    // of adjacent facets with same normal group
    // the orientation is given by ordering of va and vb
    va = verts[mod3(i_vs + 2)];
    vb = verts[i_vs];
    e = va->edgeTo(vb);
    assertx(e);
    fct = corner_fct;
    while (e->isManifold()) {
      // find other facet around e
      ForSCSimplexParent(e, f) {
        if (f != fct) {
          fct = f;
          break;
        }
      }
      EndFor;
      // cannot make a full circle this way
      assertx(fct != corner_fct);
      // if new facet does not have same smoothing group
      if (s_norgroup[fct->getVAttribute()] != ngroup) break;
      // Vec3<Simplex> verts;
      fct->vertices(verts.data());
      // find va among verts
      int i_va = -1;
      for_int(i, 3) {
        if (va == verts[i]) i_va = i;
      }
      assertx(i_va >= 0);
      if (verts[mod3(i_va + 1)] == vb) {
        // va still before vb
        // inconsistent with previous fct
        // flip normal
        avg_norm -= fct_pnor[fct->getId()];
        va = verts[mod3(i_va + 2)];
      } else {
        // va after vb in the order
        // consistent with previous fct
        avg_norm += fct_pnor[fct->getId()];
        va = verts[mod3(i_va + 1)];
      }
      e = va->edgeTo(vb);
      assertx(e);
    }
  }
  avg_norm.normalize();
}

void read_sc(const string& filename) {
  HH_TIMER(__read_sc);
  RFile fin(filename);
  Kmesh.read(fin());
  fct_pnor.init(Kmesh.getMaxId(2));
  // 3 verts per each face
  // will use face and vert id for indexing
  corner_pnor.init(Kmesh.getMaxId(2) * 3);
  s_color.init(Kmesh.materialNum());
  s_norgroup.init(Kmesh.materialNum());
  string str;
  for_int(attrid, Kmesh.materialNum()) {
    const char* s = Kmesh.getMaterial(attrid);
    A3dColor co;
    s_color[attrid] = parse_key_vec(s, "rgb", co) ? pack_color(co) : meshcolor.d;
    s_norgroup[attrid] = to_int(assertx(GMesh::string_key(str, s, "norgroup")));
  }
  ForSCSimplex(Kmesh, 2, s2) {
    Vec3<Simplex> v;
    s2->vertices(v.data());
    fct_pnor[s2->getId()] = ok_normalized(cross(v[0]->getPosition(), v[1]->getPosition(), v[2]->getPosition()));
  }
  EndFor;
  ForSCSimplex(Kmesh, 2, s2) {
    Vec3<Simplex> verts;
    s2->vertices(verts.data());
    for_int(i, 3) vertSmoothNormal(verts[i], s2, corner_pnor[3 * s2->getId() + i]);
  }
  EndFor;
  // initialize principal verts and edges set
  ForSCSimplex(Kmesh, 0, v) {
    if (v->isPrincipal()) psc_principal_verts.enter(v);
  }
  EndFor;
  ForSCSimplex(Kmesh, 1, e) {
    if (e->isPrincipal()) psc_principal_edges.enter(e);
  }
  EndFor;
  toggle_attribute(g_xobs.cullface);
  // no bbox information yet.
  // g3d::UpdateOb1Bbox(...);
  if (g3d::g_filename == "") g3d::g_filename = filename;
  g_twosided = true;
}

void read_psc(const string& filename) {
  HH_TIMER(__read_psc);
  RFile fin(filename);
  // read in base point
  Kmesh.read(fin());
  assertx(Kmesh.num(0) == 1 && Kmesh.num(1) == 0 && Kmesh.num(2) == 0);
  // initialize principal verts and edges set
  ForSCSimplex(Kmesh, 0, v) { psc_principal_verts.enter(v); }
  EndFor;
  s_color.init(Kmesh.materialNum());
  s_norgroup.init(Kmesh.materialNum());
  string str;
  for_int(attrid, Kmesh.materialNum()) {
    const char* s = Kmesh.getMaterial(attrid);
    A3dColor co;
    s_color[attrid] = parse_key_vec(s, "rgb", co) ? pack_color(co) : meshcolor.d;
    s_norgroup[attrid] = to_int(assertx(GMesh::string_key(str, s, "norgroup")));
  }
  int last = psc_lod_list.add(1);
  psc_unify_normal_list.push(nullptr);
  psc_split_normal_list.push(nullptr);
  psc_unify_area_list.push(nullptr);
  while (!psc_lod_list[last].read(fin())) {
    last = psc_lod_list.add(1);
    psc_unify_normal_list.push(nullptr);
    psc_split_normal_list.push(nullptr);
    psc_unify_area_list.push(nullptr);
  }
  // initialize area of the base vertex
  Array<AreaData>& aar = *new Array<AreaData>;
  psc_unify_area_list[0] = &aar;
  AreaData ar;
  ar.dim = 0;
  ar.id = 1;
  ar.area = Kmesh.getSimplex(0, 1)->getArea();
  aar.push(ar);
  if (hpix) {
    for_int(times, 10) {
      {
        HH_ATIMER(vsplit);
        for_int(i, psc_lod_list.num() - 1) {
          SplitRecord& vsplit = psc_lod_list[i];
          vsplit.applySplit(Kmesh);
        }
      }
      {
        HH_ATIMER(vunify);
        for (int i = psc_lod_list.num() - 2; i >= 0; --i) {
          SplitRecord& vsplit = psc_lod_list[i];
          vsplit.applyUnify(Kmesh);
        }
      }
    }
    return;
  }
  toggle_attribute(g_xobs.cullface);
  if (psc_lod_level != 0.f) psc_update_lod();
  slidermode = true;
  // no bbox information yet.
  // g3d::UpdateOb1Bbox(...);
  if (g3d::g_filename == "") g3d::g_filename = filename;
  g_twosided = true;
}

void psc_update_lod() {
  int new_lod_num = int((psc_lod_list.num() - 1) * psc_lod_level * .999999f);
  // SHOW(psc_lod_level, new_lod_num, psc_lod_num);
  // go down in complexity
  if (new_lod_num < psc_lod_num) {
    Simplex vs, vt;
    for (int i = psc_lod_num - 1; i >= new_lod_num; --i) {
      SplitRecord& vsplit = psc_lod_list[i];
      vs = assertx(Kmesh.getSimplex(0, vsplit.getVs()));
      vt = assertx(Kmesh.getSimplex(0, vsplit.getVt()));
      // remove former principal simplices
      if (vs->isPrincipal()) {
        psc_principal_verts.remove(vs);
      } else {
        ForSCSimplexParent(vs, e) {
          if (!e->isPrincipal()) continue;
          psc_principal_edges.remove(e);
        }
        EndFor;
      }
      if (vt->isPrincipal()) {
        psc_principal_verts.remove(vt);
      } else {
        ForSCSimplexParent(vt, e) {
          if (!e->isPrincipal()) continue;
          psc_principal_edges.remove(e);
        }
        EndFor;
      }
      // do unify
      vsplit.applyUnify(Kmesh);
      // add new principal simplices
      if (vs->isPrincipal()) {
        psc_principal_verts.enter(vs);
      } else {
        ForSCSimplexParent(vs, e) {
          if (!e->isPrincipal()) continue;
          psc_principal_edges.enter(e);
        }
        EndFor;
      }
      if (psc_unify_normal_list[i]) {
        // if normals cached use them
        for (NormalRecord& nr : *psc_unify_normal_list[i]) {
          fct_pnor.access(nr.fid);
          fct_pnor[nr.fid] = nr.fct_nor;
          corner_pnor.access(3 * nr.fid + 2);
          corner_pnor[3 * nr.fid] = nr.corner_nor[0];
          corner_pnor[3 * nr.fid + 1] = nr.corner_nor[1];
          corner_pnor[3 * nr.fid + 2] = nr.corner_nor[2];
        }
      } else {
        // otherwise, compute and cache
        int cnt = 0;
        ForSCVertexFace(vs, f) {
          dummy_use(f);
          cnt++;
        }
        EndFor;
        Array<NormalRecord>& anr = *new Array<NormalRecord>;
        psc_unify_normal_list[i] = &anr;
        anr.reserve(cnt);
        Vec3<Simplex> verts;
        NormalRecord nr;
        // calculate facet normals
        ForSCVertexFace(vs, s2) {
          assertx(s2->getDim() == 2);
          nr.fid = s2->getId();
          s2->vertices(verts.data());
          nr.fct_nor = ok_normalized(cross(verts[0]->getPosition(), verts[1]->getPosition(), verts[2]->getPosition()));
          fct_pnor.access(nr.fid);
          fct_pnor[nr.fid] = nr.fct_nor;
          anr.push(nr);
        }
        EndFor;
        // note: to be completely correct we should also recalculate corners just oustide
        // starbar of vs, but we don't bother.
        // revisit facets in same order and calculate corner normals
        int ii = 0;
        ForSCVertexFace(vs, s2) {
          Vec3<Simplex> verts2;
          s2->vertices(verts2.data());
          corner_pnor.access(3 * anr[ii].fid + 2);
          for_int(v, 3) {
            vertSmoothNormal(verts2[v], s2, anr[ii].corner_nor[v]);
            corner_pnor[3 * anr[ii].fid + v] = anr[ii].corner_nor[v];
          }
          ii++;
        }
        EndFor;
      }
      assertx(psc_unify_area_list[i]);
      for (AreaData& ar : *psc_unify_area_list[i]) {
        Simplex s = Kmesh.getSimplex(ar.dim, ar.id);
        assertx(Kmesh.valid(s) && s->isPrincipal());
        s->setArea(ar.area);
      }
    }
  }
  // go up in complexity
  if (new_lod_num > psc_lod_num) {
    Simplex vs, vt;
    for (int i = psc_lod_num; i < new_lod_num; i++) {
      SplitRecord& vsplit = psc_lod_list[i];
      vs = assertx(Kmesh.getSimplex(0, vsplit.getVs()));
      // remove former principal simplices
      if (vs->isPrincipal()) {
        psc_principal_verts.remove(vs);
      } else {
        ForSCSimplexParent(vs, e) { psc_principal_edges.remove(e); }
        EndFor;
      }
      // do split
      vsplit.applySplit(Kmesh);
      // add new principal simplices
      vt = assertx(Kmesh.getSimplex(0, vsplit.getVt()));
      if (vs->isPrincipal()) {
        psc_principal_verts.enter(vs);
      } else {
        ForSCSimplexParent(vs, e) {
          if (!e->isPrincipal()) continue;
          psc_principal_edges.enter(e);
        }
        EndFor;
      }
      if (vt->isPrincipal()) {
        psc_principal_verts.enter(vt);
      } else {
        Simplex vsvt = vs->edgeTo(vt);
        ForSCSimplexParent(vt, e) {
          if (e == vsvt || !e->isPrincipal()) continue;
          psc_principal_edges.enter(e);
        }
        EndFor;
      }
      if (psc_split_normal_list[i]) {
        // if normals cached use them
        for (const NormalRecord& nr : *psc_split_normal_list[i]) {
          fct_pnor.access(nr.fid);
          fct_pnor[nr.fid] = nr.fct_nor;
          corner_pnor.access(3 * nr.fid + 2);
          corner_pnor[3 * nr.fid] = nr.corner_nor[0];
          corner_pnor[3 * nr.fid + 1] = nr.corner_nor[1];
          corner_pnor[3 * nr.fid + 2] = nr.corner_nor[2];
        }
      } else {
        int cnt = 0;
        ForSCVertexFace(vs, f) {
          dummy_use(f);
          cnt++;
        }
        EndFor;
        ForSCVertexFace(vt, f) {
          dummy_use(f);
          cnt++;
        }
        EndFor;
        Array<NormalRecord>& anr = *new Array<NormalRecord>;
        psc_split_normal_list[i] = &anr;
        anr.reserve(cnt);
        NormalRecord nr;
        Vec3<Simplex> verts;
        // calculate facet normals
        ForSCVertexFace(vs, s2) {
          assertx(s2->getDim() == 2);
          s2->vertices(verts.data());
          nr.fid = s2->getId();
          nr.fct_nor = ok_normalized(cross(verts[0]->getPosition(), verts[1]->getPosition(), verts[2]->getPosition()));
          fct_pnor.access(nr.fid);
          fct_pnor[nr.fid] = nr.fct_nor;
          anr.push(nr);
        }
        EndFor;
        ForSCVertexFace(vt, s2) {
          assertx(s2->getDim() == 2);
          s2->vertices(verts.data());
          nr.fid = s2->getId();
          nr.fct_nor = ok_normalized(cross(verts[0]->getPosition(), verts[1]->getPosition(), verts[2]->getPosition()));
          fct_pnor.access(nr.fid);
          fct_pnor[nr.fid] = nr.fct_nor;
          anr.push(nr);
        }
        EndFor;
        // revisit facets in same order and calculate corner normals
        int ii = 0;
        ForSCVertexFace(vs, s2) {
          Vec3<Simplex> verts2;
          s2->vertices(verts2.data());
          corner_pnor.access(3 * anr[ii].fid + 2);
          for_int(v, 3) {
            vertSmoothNormal(verts2[v], s2, anr[ii].corner_nor[v]);
            corner_pnor[3 * anr[ii].fid + v] = anr[ii].corner_nor[v];
          }
          ii++;
        }
        EndFor;
        ForSCVertexFace(vt, s2) {
          Vec3<Simplex> verts2;
          s2->vertices(verts2.data());
          corner_pnor.access(3 * anr[ii].fid + 2);
          for_int(v, 3) {
            vertSmoothNormal(verts2[v], s2, anr[ii].corner_nor[v]);
            corner_pnor[3 * anr[ii].fid + v] = anr[ii].corner_nor[v];
          }
          ii++;
        }
        EndFor;
      }
      if (!psc_unify_area_list[i + 1]) {
        Array<AreaData>& aar = *new Array<AreaData>;
        psc_unify_area_list[i + 1] = &aar;
        const Array<AreaData>& tmp = vsplit.getAreas();
        aar.reserve(tmp.num());
        AreaData ar;
        for_int(ii, tmp.num()) {
          Simplex s = Kmesh.getSimplex(tmp[ii].dim, tmp[ii].id);
          assertx(s && s->isPrincipal());
          ar.dim = tmp[ii].dim;
          ar.id = tmp[ii].id;
          ar.area = tmp[ii].area;
          aar.push(ar);
        }
      }
    }
  }
  psc_lod_num = new_lod_num;
  invalidate_dls();
  if (g3d::output) {
    float val = lod_use_nvertices ? psc_lod_num + 1 : psc_lod_level;
    std::cout << "lod " << val << '\n';
    std::cout.flush();
  }
}

void psc_wrap_draw(bool show) {
  if (!slidermode) return;
  if (!show) {
    if (button_active && yx_pointer_old[1] < k_one_slider_left_thresh) {
      float oldval = psc_lod_level;
      Vec2<float> yxf;
      assertx(HB::get_pointer(yxf));
      switch (button_active) {
        case 1: {
          psc_lod_level = 1.1f - (yxf[0]) * 1.2f;
          break;
        }
        case 2: {
          float a = ((yxf[0]) - 0.5f) * -3.f;
          float a2 = pow(abs(a), 4.f) * sign(a) * g3d::fchange * 1.f;
          psc_lod_level += a2;
          break;
        }
        case 3: {
          const float factor = 5.f;
          float val = 1.1f - (yxf[0]) * 1.2f;
          val = max(val, 0.f);
          psc_lod_level = (exp(factor * val) - 1.0f) / (exp(factor) - 1.0f);
          break;
        }
        default: assertnever("");
      }
      psc_lod_level = clamp(psc_lod_level, 0.f, 1.f);
      if (psc_lod_level != oldval) psc_update_lod();
      hw.redraw_later();
    }
  } else {
    int nfaces = Kmesh.num(2);
    int lmargin = !bigfont() ? 7 : 3;
    if (win_dims[1] < 400 && !bigfont()) lmargin -= 3;
    static const bool psc2win = getenv_bool("G3D_PSC2WIN");
    static const bool psc2win1 = getenv_bool("G3D_PSC2WIN1");
    psc_orphan_nedges = psc_principal_edges.num();
    psc_orphan_nverts = psc_principal_verts.num();
    if (psc2win) {
      HB::draw_row_col_text(V(1, lmargin - 1), "#2s");
      HB::draw_row_col_text(V(1, lmargin + 3), sform("%d", nfaces));
      HB::draw_row_col_text(V(2, lmargin - 1), "#1s");
      HB::draw_row_col_text(V(2, lmargin + 3), sform("%d", psc_orphan_nedges));
      HB::draw_row_col_text(V(3, lmargin - 1), "#0s");
      HB::draw_row_col_text(V(3, lmargin + 3), sform("%d", psc_orphan_nverts));
    } else if (psc2win1) {
      HB::draw_row_col_text(V(1, lmargin), "#2s");
      HB::draw_row_col_text(V(1, lmargin + 4), sform("%d", nfaces));
      HB::draw_row_col_text(V(2, lmargin), "#1s");
      HB::draw_row_col_text(V(2, lmargin + 4), sform("%d", psc_orphan_nedges));
      HB::draw_row_col_text(V(3, lmargin), "#0s");
      HB::draw_row_col_text(V(3, lmargin + 4), sform("%d", psc_orphan_nverts));
    } else {
      HB::draw_row_col_text(V(1, lmargin), "LOD");
      HB::draw_row_col_text(V(1, lmargin + 4), sform("%d", psc_lod_num));
      HB::draw_row_col_text(V(3, lmargin), "#2s");
      HB::draw_row_col_text(V(3, lmargin + 4), sform("%d", nfaces));
      HB::draw_row_col_text(V(4, lmargin), "#1s");
      HB::draw_row_col_text(V(4, lmargin + 4), sform("%d", psc_orphan_nedges));
      HB::draw_row_col_text(V(5, lmargin), "#0s");
      HB::draw_row_col_text(V(5, lmargin + 4), sform("%d", psc_orphan_nverts));
    }
    float yline = .004f, xleft = .05f;
    {  // current level
      float lod = clamp(psc_lod_level, 0.f, 1.f);
      float x1 = xleft + .01f, x2 = xleft + .05f;
      float y1 = (1.1f - lod) / 1.2f - .002f;
      float y2 = y1 + yline;
      HB::draw_segment(V(y1, x1), V(y1, x2));
      HB::draw_segment(V(y2, x1), V(y2, x2));
      HB::draw_segment(V(y1, x1), V(y2, x1));
      HB::draw_segment(V(y1, x2), V(y2, x2));
    }
    {  // slider
      float y1 = (1.1f - 0) / 1.2f + .004f, y2 = (1.1f - 1) / 1.2f - .004f, yd = .01f;
      float x1 = xleft + .02f, x2 = xleft + .04f, xm = xleft + .03f;
      HB::draw_segment(V(y1 + yd, x1), V(y1, xm));
      HB::draw_segment(V(y1 + yd, x2), V(y1, xm));
      HB::draw_segment(V(y1 + yd, x1), V(y1 + yd, x2));
      HB::draw_segment(V(y2 - yd, x1), V(y2, xm));
      HB::draw_segment(V(y2 - yd, x2), V(y2, xm));
      HB::draw_segment(V(y2 - yd, x1), V(y2 - yd, x2));
      HB::draw_segment(V(y1, xm), V(y2, xm));
    }
  }
}

void psc_set_lod(float lod) {
  if (lod_use_nvertices) {
    int nv = int(lod);
    if (!assertw(psc_lod_list.num() > 1)) return;
    psc_lod_level = (float(nv) - 1) / (psc_lod_list.num() - 1);
  } else {
    psc_lod_level = lod;
  }
  if (!assertw(psc_lod_level >= 0.f)) psc_lod_level = 0.f;
  if (!assertw(psc_lod_level <= 1.f)) psc_lod_level = 1.f;
  psc_update_lod();
  hw.redraw_later();
}

// Determine projected area of a point x.
inline float projected_area(float area, const Point& x) {
  const float t1 = .5f * min(win_dims);
  // find projected area
  return area * t1 / (view_zoom * dist(x, tcam.p()));
}

void draw_point(const Point& vp, float area) {
  dummy_use(vp, area);
#if !defined(HH_NO_GLU)
  float sphererad = sqrt(area / (TAU * 2));
  unique_ptr<GLUquadricObj, decltype(&gluDeleteQuadric)> quadric{gluNewQuadric(), gluDeleteQuadric};
  glPushMatrix();
  {
    glTranslatef(vp[0], vp[1], vp[2]);
    float prad = projected_area(sphererad, vp);
    int complexity;
    if (prad < 10) {
      complexity = 3;
    } else if (prad < 40) {
      complexity = 8;
    } else {
      complexity = 20;
    }
    gluSphere(quadric.get(), sphererad, complexity, complexity);
  }
  glPopMatrix();
#endif
}

void draw_sc() {
  if (defining_dl) Warning("Display lists should be off ('DC') if zooming in/out");
  glShadeModel(GL_SMOOTH);
  initialize_lit();
  update_mat_color(meshcolor);
  if (!ledges || lshading) {
    // Options lsmooth, lquickmode not handled.
    assertw(lsmooth);
    glBegin(GL_TRIANGLES);
    int i = 0;
    ForSCSimplex(Kmesh, 2, s2) {
      if ((++i & 0x7F) == 0 && i) {
        glEnd();
        glBegin(GL_TRIANGLES);
      }
      maybe_update_mat_diffuse(s_color[s2->getVAttribute()]);
      Simplex v0, v1, v2;
      Vec3<Simplex> verts;
      s2->vertices(verts.data());
      v0 = verts[0];
      v1 = verts[1];
      v2 = verts[2];
      const Point& p0 = v0->getPosition();
      const Point& p1 = v1->getPosition();
      const Point& p2 = v2->getPosition();
      const Vector& n0 = corner_pnor[3 * s2->getId()];
      const Vector& n1 = corner_pnor[3 * s2->getId() + 1];
      const Vector& n2 = corner_pnor[3 * s2->getId() + 2];
      glNormal3fv(n0.data());
      glVertex3fv(p0.data());
      glNormal3fv(n1.data());
      glVertex3fv(p1.data());
      glNormal3fv(n2.data());
      glVertex3fv(p2.data());
    }
    EndFor;
    glEnd();
    // draw all 1-simplices with no parents
    psc_orphan_nedges = 0;
    glPushAttrib(GL_TRANSFORM_BIT);
    {  // save GL_NORMALIZE
      glEnable(GL_NORMALIZE);
      for (Simplex s1 : psc_principal_edges) {
        assertx(s1->isPrincipal());
        psc_orphan_nedges++;
        const Point& vj = s1->getChild(0)->getPosition();
        const Point& vk = s1->getChild(1)->getPosition();
        float area = s1->getArea();
        assertx(area >= 0.f);
        // radius of a cylinder from vj to vk with same area
        float rad;
        float height = dist(vj, vk);
        rad = area / (TAU * height);
        // cylinder center
        const Point& center = interp(vj, vk);
        if (height < 2.f * rad) {
          // draw as point
          maybe_update_mat_diffuse(s_color[s1->getVAttribute()]);
          draw_point(center, area);
          continue;
        }
        // find projected area
        float prad = projected_area(rad, center);
        // skip lines of disregardable thickness
        if (prad < 0.2f) continue;
        int thickness = int(prad);
        if (thickness < 5) {
          // draw a line
          initialize_unlit();
          update_cur_color(s_color[s1->getVAttribute()]);
          set_thickness(thickness);
          glBegin(GL_LINES);
          glVertex3fv(vj.data());
          glVertex3fv(vk.data());
          glEnd();
          initialize_lit();
          update_mat_color(meshcolor);
        } else {
          // draw as cylinder
          maybe_update_mat_diffuse(s_color[s1->getVAttribute()]);
          cyl.draw(vj, vk, rad);
        }
      }
    }
    glPopAttrib();
    // draw all 0-simplices with no parents
    psc_orphan_nverts = 0;
    for (Simplex s0 : psc_principal_verts) {
      assertx(s0->isPrincipal());
      psc_orphan_nverts++;
      maybe_update_mat_diffuse(s_color[s0->getVAttribute()]);
      float area = s0->getArea();
      assertx(area >= 0.f);
      draw_point(s0->getPosition(), area);
    }
  }
  if (ledges) {
    // Options cullbackedges, lquickmode not handled.
    glShadeModel(GL_FLAT);
    initialize_unlit();
    set_thickness(thicknormal);
    glBegin(GL_LINES);
    ForSCSimplex(Kmesh, 1, s) {
      update_cur_color(s->hasColor() ? s_color[s->getVAttribute()] : pix_edgecolor);
      Point p0 = s->getChild(0)->getPosition();
      Point p1 = s->getChild(1)->getPosition();
      glVertex3fv(p0.data());
      glVertex3fv(p1.data());
    }
    EndFor;
    glEnd();
    set_thickness(thicka3d);
  }
}

// SCGeomorph stuff
bool grab_sc_gm(std::istream& is, std::stringstream& grab_stream) {
  for (string sline; my_getline(is, sline);) {
    if (begins_with(sline, "[SC Geomorph]")) return true;
    grab_stream << sline << "\n";
  }
  return false;
}

void read_sc_gm(const string& filename) {
  HH_TIMER(__read_sc_gm);
  RFile fin(filename);
  // swallow stuff before first delim
  {
    std::stringstream dummy;
    grab_sc_gm(fin(), dummy);
  }
  // read in geomorphs
  {
    bool ok = true;
    while (ok) {
      std::stringstream iss;
      if (!grab_sc_gm(fin(), iss)) ok = false;
      sc_gm_num++;
      Gmorphs[sc_gm_num].read(iss);
    }
  }
  sc_gm_num++;
  assertx(sc_gm_num > 0);
  const SimplicialComplex& Kmesh2 = Gmorphs[sc_gm_num - 1].getK();
  // 3 verts per each face
  // will use face and vert id for indexing
  corner_pnor.init(Kmesh2.getMaxId(2) * 3);
  s_color.init(Kmesh2.materialNum());
  s_norgroup.init(Kmesh2.materialNum());
  for_int(attrid, Kmesh2.materialNum()) {
    const char* s = Kmesh2.getMaterial(attrid);
    A3dColor co;
    s_color[attrid] = parse_key_vec(s, "rgb", co) ? pack_color(co) : meshcolor.d;
  }
  Gmorphs[sc_gm_morph].update(sc_gm_lod_level, corner_pnor);
  toggle_attribute(g_xobs.cullface);
  slidermode = true;
  // no bbox information yet.
  // g3d::UpdateOb1Bbox(...);
  if (g3d::g_filename == "") g3d::g_filename = filename;
  g_twosided = true;
}

void sc_gm_update_lod() {
  float step = 1.f / sc_gm_num;
  sc_gm_morph = int((sc_gm_lod_level / step) - 1e-6);
  float bot = sc_gm_morph * step;
  float top = (sc_gm_morph + 1) * step;
  float alpha = (sc_gm_lod_level - bot) / (top - bot);
  Gmorphs[sc_gm_morph].update(alpha, corner_pnor);
}

void sc_gm_wrap_draw(bool show) {
  if (!slidermode) return;
  if (!show) {
    if (button_active && yx_pointer_old[1] < k_one_slider_left_thresh) {
      float oldval = sc_gm_lod_level;
      Vec2<float> yxf;
      assertx(HB::get_pointer(yxf));
      switch (button_active) {
        case 1: {
          sc_gm_lod_level = 1.1f - (yxf[0]) * 1.2f;
          break;
        }
        case 2: {
          float a = ((yxf[0]) - 0.5f) * -3.f;
          float a2 = pow(abs(a), 4.f) * sign(a) * g3d::fchange * 1.f;
          sc_gm_lod_level += a2;
          break;
        }
        case 3: break;
        default: assertnever("");
      }
      sc_gm_lod_level = clamp(sc_gm_lod_level, 0.f, 1.f);
      if (sc_gm_lod_level != oldval) {
        sc_gm_update_lod();
      }
      hw.redraw_later();
    }
  } else {
    int nfaces = Gmorphs[sc_gm_morph].getK().num(2);
    int lmargin = !bigfont() ? 7 : 3;
    if (win_dims[1] < 400 && !bigfont()) lmargin -= 3;
    HB::draw_row_col_text(V(1, lmargin), "GM#");
    HB::draw_row_col_text(V(1, lmargin + 4), sform("%d", sc_gm_morph));
    HB::draw_row_col_text(V(3, lmargin), "#2s");
    HB::draw_row_col_text(V(3, lmargin + 4), sform("%d", nfaces));
    HB::draw_row_col_text(V(4, lmargin), "#1s");
    HB::draw_row_col_text(V(4, lmargin + 4), sform("%d", psc_orphan_nedges));
    HB::draw_row_col_text(V(5, lmargin), "#0s");
    HB::draw_row_col_text(V(5, lmargin + 4), sform("%d", psc_orphan_nverts));
    float yline = .004f, xleft = .05f;
    {  // current level
      float lod = clamp(sc_gm_lod_level, 0.f, 1.f);
      float x1 = xleft + .01f, x2 = xleft + .05f;
      float y1 = (1.1f - lod) / 1.2f - .002f;
      float y2 = y1 + yline;
      HB::draw_segment(V(y1, x1), V(y1, x2));
      HB::draw_segment(V(y2, x1), V(y2, x2));
      HB::draw_segment(V(y1, x1), V(y2, x1));
      HB::draw_segment(V(y1, x2), V(y2, x2));
    }
    {  // slider
      float y1 = (1.1f - 0) / 1.2f + .004f, y2 = (1.1f - 1) / 1.2f - .004f, yd = .01f;
      float x1 = xleft + .02f, x2 = xleft + .04f, xm = xleft + .03f;
      HB::draw_segment(V(y1 + yd, x1), V(y1, xm));
      HB::draw_segment(V(y1 + yd, x2), V(y1, xm));
      HB::draw_segment(V(y1 + yd, x1), V(y1 + yd, x2));
      HB::draw_segment(V(y2 - yd, x1), V(y2, xm));
      HB::draw_segment(V(y2 - yd, x2), V(y2, xm));
      HB::draw_segment(V(y2 - yd, x1), V(y2 - yd, x2));
      HB::draw_segment(V(y1, xm), V(y2, xm));
    }
  }
}

void sc_gm_set_lod(float lod) {
  sc_gm_lod_level = lod;
  if (!assertw(sc_gm_lod_level >= 0.f)) sc_gm_lod_level = 0.f;
  if (!assertw(sc_gm_lod_level <= 1.f)) sc_gm_lod_level = 1.f;
  sc_gm_update_lod();
  hw.redraw_later();
}

void draw_sc_gm(const SimplicialComplex& kmesh) {
  if (defining_dl) Warning("Display lists should be off ('DC') if zooming in/out");
  glShadeModel(GL_SMOOTH);
  initialize_lit();
  update_mat_color(meshcolor);
  if (!ledges || lshading) {
    // Options lsmooth, lquickmode not handled.
    assertw(lsmooth);
    glBegin(GL_TRIANGLES);
    int i = 0;
    ForSCSimplex(kmesh, 2, s2) {
      if ((++i & 0x7F) == 0 && i) {
        glEnd();
        glBegin(GL_TRIANGLES);
      }
      maybe_update_mat_diffuse(s_color[s2->getVAttribute()]);
      Simplex v0, v1, v2;
      Vec3<Simplex> verts;
      s2->vertices(verts.data());
      v0 = verts[0];
      v1 = verts[1];
      v2 = verts[2];
      const Point& p0 = v0->getPosition();
      const Point& p1 = v1->getPosition();
      const Point& p2 = v2->getPosition();
      const Vector& n0 = corner_pnor[3 * s2->getId()];
      const Vector& n1 = corner_pnor[3 * s2->getId() + 1];
      const Vector& n2 = corner_pnor[3 * s2->getId() + 2];
      glNormal3fv(n0.data());
      glVertex3fv(p0.data());
      glNormal3fv(n1.data());
      glVertex3fv(p1.data());
      glNormal3fv(n2.data());
      glVertex3fv(p2.data());
    }
    EndFor;
    glEnd();
    // draw all 1-simplices with no parents
    psc_orphan_nedges = 0;
    glPushAttrib(GL_TRANSFORM_BIT);
    {  // save GL_NORMALIZE
      glEnable(GL_NORMALIZE);
      ForSCSimplex(kmesh, 1, s1) {
        if (s1->getArea() < 1e-3f) continue;
        psc_orphan_nedges++;
        const Point& vj = s1->getChild(0)->getPosition();
        const Point& vk = s1->getChild(1)->getPosition();
        float area = s1->getArea();
        assertx(area >= 0.f);
        // radius of a cylinder from vj to vk with same area
        float rad;
        float height = dist(vj, vk);
        rad = area / (TAU * height);
        // cylinder center
        const Point& center = interp(vj, vk);
        if (height < 2.f * rad) {
          // draw as point
          maybe_update_mat_diffuse(s_color[s1->getVAttribute()]);
          draw_point(center, area);
          continue;
        }
        // find projected area
        float prad = projected_area(rad, center);
        // skip lines of disregardable thickness
        if (prad < 0.2) continue;
        int thickness = int(prad);
        if (thickness < 5) {
          // draw a line
          initialize_unlit();
          update_cur_color(s_color[s1->getVAttribute()]);
          set_thickness(thickness);
          glBegin(GL_LINES);
          glVertex3fv(vj.data());
          glVertex3fv(vk.data());
          glEnd();
          initialize_lit();
          update_mat_color(meshcolor);
        } else {
          // draw as cylinder
          maybe_update_mat_diffuse(s_color[s1->getVAttribute()]);
          cyl.draw(vj, vk, rad);
        }
      }
      EndFor;
    }
    glPopAttrib();
    // draw all 0-simplices with no parents
    psc_orphan_nverts = 0;
    ForSCSimplex(kmesh, 0, s0) {
      if (s0->getArea() < 1e-3f) continue;
      psc_orphan_nverts++;
      maybe_update_mat_diffuse(s_color[s0->getVAttribute()]);
      float area = s0->getArea();
      assertx(area >= 0.f);
      if (area < 1e-3f) continue;
      draw_point(s0->getPosition(), area);
    }
    EndFor;
  }
  if (ledges) {
    // Options cullbackedges, lquickmode not handled.
    glShadeModel(GL_FLAT);
    initialize_unlit();
    set_thickness(thicknormal);
    glBegin(GL_LINES);
    ForSCSimplex(kmesh, 1, s) {
      update_cur_color(s->hasColor() ? s_color[s->getVAttribute()] : pix_edgecolor);
      Point p0 = s->getChild(0)->getPosition();
      Point p1 = s->getChild(1)->getPosition();
      glVertex3fv(p0.data());
      glVertex3fv(p1.data());
    }
    EndFor;
    glEnd();
    set_thickness(thicka3d);
  }
}

// *** Cylinder

Cylinder::Cylinder(int depth) {
  Vec<float, 64> verts;
  // clip maximum depth
  depth = clamp(depth, 1, 3);
  int nv = 4 * (1 << (depth - 1));
  int num = 2 * (nv + 1);
  _v.init(num);
  _n.init(num);
  float dth = TAU / nv;
  float th = .5f * dth;
  int i;
  for (i = 0; i < nv; i++) {
    verts[2 * i] = cos(th);
    verts[2 * i + 1] = sin(th);
    th += dth;
  }
  int ii = 0;
  // verts
  for (i = 0; i <= nv; i++) {
    _v[ii][0] = _v[ii + 1][0] = verts[2 * (i % nv)];
    _v[ii][1] = _v[ii + 1][1] = verts[2 * (i % nv) + 1];
    _v[ii][2] = 1.f;
    _v[ii + 1][2] = 0.f;
    ii += 2;
  }
  assertx(ii == num);
  // normals
  for (i = 0; i < num; i += 2) {
    Vector an(0.f, 0.f, 0.f);
    an += ok_normalized(cross(_v[i], _v[i + 1], _v[(i + 2) % num]));
    an += ok_normalized(cross(_v[(i + 2) % num], _v[(i + 3) % num], _v[(i + 4) % num]));
    an = ok_normalized(an);
    _n[(i + 2) % num] = an;
    _n[(i + 3) % num] = an;
  }
  // endcaps
  tc[0] = bc[0] = 0.f;
  tc[1] = bc[1] = 0.f;
  tc[2] = 1.f;
  bc[2] = 0.f;
  tn = ok_normalized(cross(tc, _v[0], _v[2]));
  bn = ok_normalized(cross(bc, _v[1], _v[3]));
}

void Cylinder::draw() {
  // optimize: disable double-side lighting.
  glBegin(GL_QUAD_STRIP);
  for_int(i, _v.num()) {
    glNormal3fv(_n[i].data());
    glVertex3fv(_v[i].data());
  }
  glEnd();
  //
  glBegin(GL_TRIANGLE_FAN);
  glNormal3fv(tn.data());
  glVertex3fv(tc.data());
  for (int i = 0; i < _v.num(); i += 2) glVertex3fv(_v[i].data());
  glEnd();
  //
  glBegin(GL_TRIANGLE_FAN);
  glNormal3fv(bn.data());
  glVertex3fv(bc.data());
  for (int i = _v.num() - 1; i >= 0; i -= 2) glVertex3fv(_v[i].data());
  glEnd();
}

void Cylinder::draw(const Point& p1, const Point& p2, float r) {
  float D = dist(p1, p2);
  Vector u = (p2 - p1) / D;
  float d = sqrt(u[0] * u[0] + u[1] * u[1]);
  // Matrix4 m;
  SGrid<float, 4, 4> m = {
      {-u[1] / d, u[0] / d, 0.f, 0.f},
      {-(u[0] * u[2]) / d, -(u[1] * u[2]) / d, d, 0.f},
      {u[0], u[1], u[2], 0.f},
      {0.f, 0.f, 0.f, 1.f},
  };
  glPushMatrix();
  {
    glTranslatef(p1[0], p1[1], p1[2]);
    glMultMatrixf(m.data());
    glScalef(r, r, D);
    draw();
  }
  glPopMatrix();
}

bool psc_key_press(char ch) {
  switch (ch) {
    case 'c':
      psc_lod_level = 0.f;
      psc_update_lod();
      hw.redraw_now();
      break;
    case 'a':
      psc_lod_level = 1.f;
      psc_update_lod();
      hw.redraw_now();
      break;
    default: return false;
  }
  return true;
}

#endif  // defined(DEF_SC)

// *** PLY mode

#if defined(DEF_PLY)

// struct PlyIndices { int vi[3]; };
// using PlyIndices = std::array<int, 3>;
using PlyIndices = PArray<int, 4>;
Array<Point> ply_vpos;
Array<Vector> ply_vnor;
Array<Pixel> ply_vrgb;
Array<PlyIndices> ply_findices;

// ply
// format binary_big_endian 1.0
// element vertex 255909
// property float x
// property float y
// property float z
// property float confidence
// element face 441455
// property list uchar int vertex_indices
// end_header
//
// property list uchar uint vertex_indices  # Blender

void read_ply(const string& filename) {
  // HH_TIMER(_read_ply);
  RFile fi(filename);
  {
    string sline;
    assertx(my_getline(fi(), sline));
    assertx(sline == "ply");
  }
  bool binary = false;
  bool bigendian = false;
  int len_nfv = 1;  // default uchar
  int state = 0;    // 0=undef, 1=vert, 2=face
  int vnpos = 0, vnnor = 0, vnrgb = 0, vnother = 0, vnotherb = 0, flist = 0, fnother = 0, fnotherb = 0;
  for (string sline;;) {
    assertx(my_getline(fi(), sline));
    if (0) {
    } else if (begins_with(sline, "comment ")) {
      assertw(state == 0);
    } else if (sline == "format ascii 1.0") {
      assertx(state == 0);
      binary = false;
      bigendian = false;
    } else if (sline == "format binary_big_endian 1.0") {
      assertx(state == 0);
      binary = true;
      bigendian = true;
    } else if (sline == "format binary_little_endian 1.0") {
      assertx(state == 0);
      binary = true;
      bigendian = false;
    } else if (begins_with(sline, "element vertex ")) {
      assertx(state == 0);
      int nv = 0;
      assertx(sscanf(sline.c_str(), "element vertex %d", &nv) == 1);
      assertx(nv > 0);
      ply_vpos.init(nv);
      state = 1;
    } else if (begins_with(sline, "element face ")) {
      assertx(state == 1);
      int nf = 0;
      assertx(sscanf(sline.c_str(), "element face %d", &nf) == 1);
      assertw(nf > 0);
      ply_findices.init(nf);
      state = 2;
    } else if (sline == "property float x") {
      assertx(state == 1);
      vnpos++;
      assertx(vnnor + vnrgb + vnother == 0);
    } else if (sline == "property float y") {
      assertx(state == 1);
      vnpos++;
      assertx(vnnor + vnrgb + vnother == 0);
    } else if (sline == "property float z") {
      assertx(state == 1);
      vnpos++;
      assertx(vnnor + vnrgb + vnother == 0);
    } else if (sline == "property float nx") {
      assertx(state == 1);
      vnnor++;
      assertx(vnrgb + vnother == 0);
    } else if (sline == "property float ny") {
      assertx(state == 1);
      vnnor++;
      assertx(vnrgb + vnother == 0);
    } else if (sline == "property float nz") {
      assertx(state == 1);
      vnnor++;
      assertx(vnrgb + vnother == 0);
    } else if (sline == "property uchar red") {
      assertx(state == 1);
      vnrgb++;
      assertx(vnother == 0);
    } else if (sline == "property uchar green") {
      assertx(state == 1);
      vnrgb++;
      assertx(vnother == 0);
    } else if (sline == "property uchar blue") {
      assertx(state == 1);
      vnrgb++;
      assertx(vnother == 0);
    } else if (sline == "property list uchar int vertex_indices") {
      assertx(state == 2);
      flist++;
      assertx(fnother == 0);
    } else if (sline == "property list uchar uint vertex_indices") {
      assertx(state == 2);
      flist++;
      assertx(fnother == 0);
    } else if (sline == "property list int int vertex_indices") {
      assertx(state == 2);
      flist++;
      assertx(fnother == 0);
      len_nfv = 4;
    } else if (begins_with(sline, "property float")) {
      if (state == 1)
        vnotherb += 4;
      else if (state == 2)
        fnotherb += 4;
      else
        assertnever("");
      if (state == 1)
        vnother++;
      else if (state == 2)
        fnother++;
      else
        assertnever("");
    } else if (begins_with(sline, "property int")) {
      if (state == 1)
        vnotherb += 4;
      else if (state == 2)
        fnotherb += 4;
      else
        assertnever("");
      if (state == 1)
        vnother++;
      else if (state == 2)
        fnother++;
      else
        assertnever("");
    } else if (begins_with(sline, "property uchar")) {
      if (state == 1)
        vnotherb += 1;
      else if (state == 2)
        fnotherb += 1;
      else
        assertnever("");
      if (state == 1)
        vnother++;
      else if (state == 2)
        fnother++;
      else
        assertnever("");
    } else if (sline == "end_header") {
      break;
    } else {
      assertnever("ply field unknown in '" + sline + "'");
    }
  }
  assertx(vnpos == 3);
  assertx(vnnor == 0 || vnnor == 3);
  assertx(vnrgb == 0 || vnrgb == 3 || vnrgb == 4);
  // assertw(flist == 1);  // there may be just vertices
  if (vnnor) ply_vnor.init(ply_vpos.num());
  if (vnrgb) ply_vrgb.init(ply_vpos.num(), Pixel::gray(0));
  if (binary) {
    for_int(i, ply_vpos.num()) {
      assertx(read_binary_raw(fi(), ply_vpos[i].view()));
      for_int(c, vnpos) {
        if (bigendian)
          from_std(&ply_vpos[i][c]);
        else
          from_dos(&ply_vpos[i][c]);
      }
      if (vnnor) assertx(read_binary_raw(fi(), ply_vnor[i].view()));
      for_int(c, vnnor) {
        if (bigendian)
          from_std(&ply_vnor[i][c]);
        else
          from_dos(&ply_vnor[i][c]);
      }
      for_int(c, vnrgb) assertx(read_binary_raw(fi(), ArView(ply_vrgb[i][c])));
      for_int(c, vnotherb) {
        uchar dummy;
        assertx(read_binary_raw(fi(), ArView(dummy)));
      }
    }
    for_int(i, ply_findices.num()) {
      int ni;
      if (len_nfv == 1) {
        uchar vi = 0;
        assertx(read_binary_raw(fi(), ArView(vi)));
        ni = vi;
      } else if (len_nfv == 4) {
        int32_t vi = 0;
        assertx(read_binary_raw(fi(), ArView(vi)));
        ni = vi;
      } else {
        assertnever("");
      }
      ply_findices[i].init(ni);
      for_int(j, ni) {
        assertx(read_binary_raw(fi(), ArView(ply_findices[i][j])));
        if (bigendian)
          from_std(&ply_findices[i][j]);
        else
          from_dos(&ply_findices[i][j]);
        assertx(ply_findices[i][j] < ply_vpos.num());
      }
      for_int(c, fnotherb) {
        uchar dummy;
        assertx(read_binary_raw(fi(), ArView(dummy)));
      }
    }
  } else {
    for_int(i, ply_vpos.num()) {
      for_int(c, vnpos) assertx(fi() >> ply_vpos[i][c]);
      for_int(c, vnnor) assertx(fi() >> ply_vnor[i][c]);
      for_int(c, vnrgb) {
        int v;
        assertx(fi() >> v);
        ply_vrgb[i][c] = narrow_cast<uchar>(v);
      }
      for_int(c, vnother) {
        float dummy;
        assertx(fi() >> dummy);
      }
    }
    // SHOW(ply_vpos.last());
    for_int(i, ply_findices.num()) {
      int ni = 0;
      assertx(fi() >> ni);
      ply_findices[i].init(ni);
      for_int(j, ni) {
        assertx(fi() >> ply_findices[i][j]);
        assertx(ply_findices[i][j] < ply_vpos.num());
      }
      for_int(c, fnother) {
        float dummy;
        assertx(fi() >> dummy);
      }
    }
  }
  showf("G3d: (1) File:%s v=%d f=%d\n", filename.c_str(), ply_vpos.num(), ply_findices.num());
  Bbox bbox;
  for_int(i, ply_vpos.num()) bbox.union_with(ply_vpos[i]);
  g3d::UpdateOb1Bbox(bbox);
  if (g3d::g_filename == "") g3d::g_filename = filename;
}

void draw_ply() {
  if (lsmooth && !ply_vnor.num()) {
    ply_vnor.init(ply_vpos.num(), Vector(0.f, 0.f, 0.f));
    for_int(i, ply_findices.num()) {
      Polygon poly;
      for_int(j, ply_findices[i].num()) {
        int vi = ply_findices[i][j];
        poly.push(ply_vpos[vi]);
      }
      Vector vnor = poly.get_normal();
      for_int(j, ply_findices[i].num()) {
        int vi = ply_findices[i][j];
        ply_vnor[vi] += vnor;
      }
    }
    for (Vector& vnor : ply_vnor) assertw(vnor.normalize());
  }
  // Options lsmooth, lquickmode not handled.
  if (ply_findices.num() > 5000000 && defining_dl)
    Warning("Graphics card may not have sufficient memory for display list");
  bool has_rgb = false;
  if (!ledges || lshading) {
    glShadeModel(lsmooth || has_rgb ? GL_SMOOTH : GL_FLAT);
    initialize_lit();
    update_mat_color(meshcolor);
    if (texture_active) {
      if (!texture_lit) {
        glShadeModel(GL_FLAT);
        initialize_unlit();
        update_cur_color(meshcolor.d);
        set_light_ambient(0.f);
        if (texturenormal) normalmap_activate();
      }
    }
    const int buffer_ntriangles = g_is_ati ? INT_MAX : 32;
    int ntriangles = 0;
    for_int(i, ply_findices.num()) {
      if (ply_findices[i].num() == 3) {
        if (ntriangles == buffer_ntriangles) {
          glEnd();
          ntriangles = 0;
        }
        if (!ntriangles) glBegin(GL_TRIANGLES);
        ntriangles++;
      } else {
        if (ntriangles) {
          glEnd();
          ntriangles = 0;
        }
        glBegin(GL_POLYGON);
      }
      if (!lsmooth) {
        Vector fnormal = ok_normalized(
            cross(ply_vpos[ply_findices[i][0]], ply_vpos[ply_findices[i][1]], ply_vpos[ply_findices[i][2]]));
        glNormal3fv(fnormal.data());
      }
      for_int(j, ply_findices[i].num()) {
        int vi = ply_findices[i][j];
        if (lsmooth && ply_vnor.num()) glNormal3fv(ply_vnor[vi].data());
        if (ply_vrgb.num()) glColor4ubv(ply_vrgb[vi].data());
        glVertex3fv(ply_vpos[vi].data());
      }
      if (!ntriangles) glEnd();  // GL_POLYGON
    }
    if (ntriangles) glEnd();    // GL_TRIANGLES
    if (!ply_findices.num()) {  // draw points
      initialize_unlit();
      glBegin(GL_POINTS);
      for_int(i, ply_vpos.num()) {
        if (ply_vnor.num()) glNormal3fv(ply_vnor[i].data());
        if (ply_vrgb.num()) glColor4ubv(ply_vrgb[i].data());
        glVertex3fv(ply_vpos[i].data());
      }
      glEnd();
    }
    initialize_unlit(), initialize_lit();  // re-synchronize
  }
  if (texture_active) {
    glDisable(GL_TEXTURE_2D);
    set_light_ambient(ambient);
    if (texturenormal) normalmap_deactivate();
  }
  if (ledges && !texture_active) {
    glShadeModel(GL_FLAT);
    initialize_unlit();
    set_thickness(thicknormal);
    update_cur_color(pix_edgecolor);
    glBegin(GL_LINES);
    for_int(i, ply_findices.num()) {
      int n = ply_findices[i].num();
      for_int(j, n) {
        // draws most edges twice :-(
        glVertex3fv(ply_vpos[ply_findices[i][(j + 0) % n]].data());
        glVertex3fv(ply_vpos[ply_findices[i][(j + 1) % n]].data());
      }
    }
    glEnd();
    set_thickness(thicka3d);
  }
}

#endif  // defined(DEF_PLY)

}  // namespace
