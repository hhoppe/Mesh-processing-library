// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/A3dStream.h"
#include "libHh/Args.h"
#include "libHh/BinaryIO.h"
#include "libHh/Color_ramp.h"  // k_color_ramp
#include "libHh/ConsoleProgress.h"
#include "libHh/Contour.h"
#include "libHh/Encoding.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/GridOp.h"
#include "libHh/GridPixelOp.h"  // scale_Matrix_Pixel()
#include "libHh/Image.h"
#include "libHh/Lls.h"
#include "libHh/MathOp.h"  // smooth_step(), floor(Vec<>)
#include "libHh/Matrix.h"
#include "libHh/MatrixOp.h"  // euclidean_distance_map()
#include "libHh/Multigrid.h"
#include "libHh/Parallel.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Stat.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
#include "libHh/Vector4.h"
#include "libHh/Video.h"  // RVideo, WVideo
using namespace hh;

namespace {

Image image;  // [Y][X][0 .. 3]

ParseArgs* g_parseargs = nullptr;
bool elevation = false;
bool rg_elev = false;
float offsetzaxis = 0.f;
float scalezaxis = 1.f;
bool removekinks = false;
int blocks = 0;
int step = 1;
int bxnum = 0;
int bynum = 0;
bool quads = false;
bool strip_order = false;
bool best_diagonal = false;
bool toggle_order = false;
bool g_not = false;
bool fixedbnd = false;
float wconformal = 0.f;
float gscale = 1.f;
int g_niter = 1;
bool use_lab = true;
bool nooutput = false;
Pixel gcolor{255, 255, 255, 255};
float tolerance = 0.f;
Vec2<Bndrule> g_bndrules = twice(Bndrule::reflected);
Vec2<FilterBnd> g_filterbs =
    V(FilterBnd(Filter::get("spline"), g_bndrules[0]), FilterBnd(Filter::get("spline"), g_bndrules[1]));

constexpr Bndrule k_reflected = Bndrule::reflected;
constexpr Vec2<Bndrule> k_reflected2 = twice(Bndrule::reflected);

int parse_size(string s, int size, bool measure_neg_from_end) {
  assertx(s != "" && size);
  bool is_neg = remove_at_beginning(s, "-");
  assertx(s[0] != '-');
  int i;
  if (remove_at_end(s, "%")) {
    float v = Args::parse_float(s);
    i = int(v / 100.f * size + .5f);
  } else {
    i = Args::parse_int(s);
  }
  assertx(i >= 0);
  if (is_neg) {
    if (measure_neg_from_end)
      i = size - i;
    else
      i = -i;
  }
  return i;
}

// ***

void do_nostdin(Args& args) { dummy_use(args); }

void do_create(Args& args) {
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx > 0 && ny > 0);
  image.init(V(ny, nx), gcolor);  // white by default
                                  // image.set_suffix() is left unspecified
}

// Remove quantization kinks.

inline float get_pixel_value(const Vec2<int>& yx) {
  return !rg_elev ? image[yx][0] : image[yx][0] * 256.f + image[yx][1];
}

// returns 0.f .. 255.f  or 0.f .. 16383.f
float get_filtered_pixel_value(const Vec2<int>& yxc) {
  assertx(image.zsize() == (1 + 2 * (rg_elev ? 1 : 0)));
  float valc = get_pixel_value(yxc);
  if (!removekinks) return valc;
  float sum = valc;
  int count = 1;
  const int upper_range = 10;
  for (int r = 1;; r++) {
    bool stop = false;
    float psum = 0.f;
    int pcount = 0;
    for (int sign : {-1, 1}) {
      int x = yxc[1] + sign * r;
      if (x < 0 || x >= image.xsize()) continue;
      for_intL(y, max(yxc[0] - (r - 0), 0), min(yxc[0] + (r - 0), image.ysize())) {
        float val = get_pixel_value(V(y, x));
        if (val > valc + 1.f || val < valc - 1.f) {
          stop = true;
          break;
        } else {
          psum += val;
          pcount++;
        }
      }
    }
    for (int sign : {-1, 1}) {
      int y = yxc[0] + sign * r;
      if (y < 0 || y >= image.ysize()) continue;
      for_intL(x, max(yxc[1] - (r - 1), 0), min(yxc[1] + (r - 1), image.xsize())) {
        float val = get_pixel_value(V(y, x));
        if (val > valc + 1.f || val < valc - 1.f) {
          stop = true;
          break;
        } else {
          psum += val;
          pcount++;
        }
      }
    }
    if (stop || r == upper_range) {
      // HH_SSTAT(Srange, r);
      break;
    }
    sum += psum;
    count += pcount;
  }
  return sum / count;
}

Vec2<int> beg_yx;
Vec2<float> scale_yx;

inline void assign_vertex(GMesh& mesh, MatrixView<Vertex> verts, const Vec2<int>& yx) {
  Vertex v = mesh.create_vertex();
  assertx(!verts[yx - beg_yx]);
  verts[yx - beg_yx] = v;
  if (elevation) {
    float elev = get_filtered_pixel_value(yx) * scalezaxis + offsetzaxis;
    mesh.set_point(v, Point(concat((convert<float>(yx) * scale_yx).rev(), V(elev))));
  } else {
    Point p = concat((convert<float>(yx) * scale_yx).rev(), V(0.f));
    Vec2<int> yx2 = yx;
    const bool flip_vertical = true;
    if (flip_vertical) {
      p[1] = 1.f - p[1];
      yx2[0] = image.ysize() - 1 - yx[0];
    }
    mesh.set_point(v, p);
    const float scalergb = 1.f / 255.f;
    Vector vrgb = Vector(image[yx2][0], image[yx2][1], image[yx2][2]) * scalergb;
    string str;
    mesh.update_string(v, "rgb", csform(str, "(%.3f %.3f %.3f)", vrgb[0], vrgb[1], vrgb[2]));
  }
}

void do_tomesh(Args& args) {
  ParseArgs& pargs = static_cast<ParseArgs&>(args);
  showff("%s", pargs.header().c_str());
  //
  assertx(image.zsize() == (elevation && !rg_elev ? 1 : 3));
  beg_yx = twice(0);
  Vec2<int> end_yx1 = image.dims();
  if (blocks) {
    beg_yx = (V(bynum, bxnum) + 0) * blocks;
    end_yx1 = (V(bynum, bxnum) + 1) * blocks + 1;
  }
  GMesh mesh;
  Matrix<Vertex> verts;  // [Y][X]
  const int s = step;
  verts.init(end_yx1 - beg_yx);
  const int begy = beg_yx[0], begx = beg_yx[1];
  const int endy1 = end_yx1[0], endx1 = end_yx1[1];
  for (int y = 0; y < endy1 - begy; y += s) {
    for (int x = 0; x < endx1 - begx; x += s) verts[y][x] = nullptr;
  }
  const bool uniform_scaling = true;
  if (!uniform_scaling) {
    scale_yx = 1.f / convert<float>(image.dims() - 1);
  } else {
    scale_yx = twice(1.f / (max(image.dims()) - 1));
  }
  if (blocks) {
    for (int x = begx; x < endx1; x += s) assign_vertex(mesh, verts, V(begy, x));
    for (int x = begx; x < endx1; x += s) assign_vertex(mesh, verts, V(endy1 - s, x));
    for (int y = begy + s; y < endy1 - s; y += s) assign_vertex(mesh, verts, V(y, begx));
    for (int y = begy + s; y < endy1 - s; y += s) assign_vertex(mesh, verts, V(y, endx1 - s));
    for (int y = begy + s; y < endy1 - s; y += s) {
      for (int x = begx + s; x < endx1 - s; x += s) assign_vertex(mesh, verts, V(y, x));
    }
    for (int y = 0; y < endy1 - begy; y += s) {
      for (int x = 0; x < endx1 - begx; x += s) assertx(verts[y][x]);
    }
  } else {
    for (int y = begy; y < endy1; y += s) {
      for (int x = begx; x < endx1; x += s) assign_vertex(mesh, verts, V(y, x));
    }
  }
  bool yeven = false;
  for (int y = 0; y < endy1 - begy - s; y += s) {
    yeven = !yeven;
    for (int x = 0; x < endx1 - begx - s; x += s) {
      CMatrixView<Vertex> v(verts);
      if (quads) {
        mesh.create_face(V(v[y][x], v[y][x + s], v[y + s][x + s], v[y + s][x]));
      } else if (strip_order) {
        mesh.create_face(v[y + s][x], v[y][x], v[y + s][x + s]);
        mesh.create_face(v[y][x], v[y][x + s], v[y + s][x + s]);
      } else if (toggle_order) {
        if (yeven) {
          mesh.create_face(v[y + s][x], v[y][x], v[y + s][x + s]);
          mesh.create_face(v[y][x], v[y][x + s], v[y + s][x + s]);
        } else {
          mesh.create_face(v[y][x], v[y][x + s], v[y + s][x]);
          mesh.create_face(v[y + s][x], v[y][x + s], v[y + s][x + s]);
        }
      } else if (best_diagonal) {
        if (dist2(mesh.point(v[y + s][x]), mesh.point(v[y][x + s])) >
            dist2(mesh.point(v[y][x]), mesh.point(v[y + s][x + s]))) {
          mesh.create_face(v[y + s][x], v[y][x], v[y + s][x + s]);
          mesh.create_face(v[y][x], v[y][x + s], v[y + s][x + s]);
        } else {
          mesh.create_face(v[y][x], v[y][x + s], v[y + s][x]);
          mesh.create_face(v[y + s][x], v[y][x + s], v[y + s][x + s]);
        }
      } else {
        mesh.create_face(v[y][x], v[y + s][x + s], v[y + s][x]);
        mesh.create_face(v[y][x], v[y][x + s], v[y + s][x + s]);
      }
    }
  }
  if (blocks) {
    string str;
    for (Face f : mesh.faces()) mesh.set_string(f, csform(str, "block=\"x%dy%d\"", bxnum, bynum));
  }
  mesh.write(std::cout);
  std::cout.flush();
  nooutput = true;
  if (0) exit(0);  // avoid GMesh::~GMesh for speed
}

void do_tofloats(Args& args) {
  string filename = args.get_filename();
  WFile fi(filename);
  auto begyx = V(0, 0), endyx1 = image.dims();
  if (blocks) {
    begyx = V(bynum, bxnum) * blocks;
    endyx1 = begyx + blocks + 1;
  }
  auto numyx = (endyx1 - begyx - 1) / step + 1;
  showdf("Writing floats file of size (%dx%d)\n", numyx[1], numyx[0]);
  assertx(write_binary_std(fi(), convert<float>(numyx.rev()).const_view()));
  for (const auto& iyx : range(numyx)) {
    auto yx = begyx + iyx * step;
    float elev = get_filtered_pixel_value(yx) * scalezaxis + offsetzaxis;
    assertx(write_binary_std(fi(), ArView(elev)));
  }
  nooutput = true;
}

void do_tofmp(Args& args) {
  string filename = args.get_filename();
  if (image.zsize() > 1) Warning("Only channel 0 (red) will be used to write fmp");
  WFile fi(filename);
  const int chw = 3;
  Array<float> buf(image.xsize() * chw);
  assertx(min(image.dims()) >= 2);
  for_int(y, image.ysize()) {
    float* p = buf.data();
    for_int(x, image.xsize()) {
      *p++ = float(x) / (image.xsize() - 1.f);
      *p++ = float(y) / (image.ysize() - 1.f);
      float v = float(image[y][x][0]) / 255.f * scalezaxis + offsetzaxis;
      *p++ = v;
    }
    assertx(write_binary_raw(fi(), buf));
  }
  fi().flush();
  assertx(fi());
  nooutput = true;
}

void do_to(Args& args) { image.set_suffix(args.get_string()); }

void do_outfile(Args& args) {
  string filename = args.get_filename();
  image.write_file(filename);
}

void do_info() {
  showf("Image w=%d h=%d z=%d format=%s\n",  //
        image.xsize(), image.ysize(), image.zsize(), image.suffix() == "" ? "unk" : image.suffix().c_str());
  Array<Stat> stat_pixels;
  for_int(z, image.zsize()) stat_pixels.push(Stat(sform("Component%d", z)));
  int na0 = 0, na255 = 0;
  for (const auto& yx : range(image.dims())) {
    for_int(z, image.zsize()) stat_pixels[z].enter(image[yx][z]);
    if (image.zsize() == 4) {
      na0 += image[yx][3] == 0;
      na255 += image[yx][3] == 255;
    }
  }
  for_int(z, image.zsize()) showf("%s", stat_pixels[z].name_string().c_str());
  if (image.zsize() == 4) {
    showf("Alpha: #0=%d(%.3f%%)  #255=%d(%.3f%%)\n",  //
          na0, float(na0) / image.size() * 100.f, na255, float(na255) / image.size() * 100.f);
  }
  if (0) {
    Stat stat("ch0");
    stat.set_rms();
    for (const auto& yx : range(image.dims())) stat.enter(image[yx][0]);
    showf("%s", stat.name_string().c_str());
    SHOW(stat.rms());
    const float maxv = 255.f;
    const float rmsv = stat.rms();
    float psnr = 20.f * std::log10(maxv / rmsv);
    SHOW(psnr);
  }
}

void do_stat() {
  do_info();
  nooutput = true;
}

void do_sizes() {
  std::cout << sform("%d %d\n", image.xsize(), image.ysize());
  nooutput = true;
}

// *** cropping

void do_color(Args& args) {
  for_int(i, 4) {
    int v = args.get_int();
    assertx(v >= 0 && v <= 255);
    gcolor[i] = narrow_cast<uint8_t>(v);
  }
}

void do_not() { g_not = !g_not; }

void do_getcolorxy(Args& args) {
  int x = parse_size(args.get_string(), image.xsize(), true);
  int y = parse_size(args.get_string(), image.ysize(), true);
  const Vec2<int> yx(y, x);
  assertx(image.ok(yx));
  gcolor = image[yx];
  if (image.zsize() < 4) gcolor[3] = 255;
  showf("gotcolor %d %d %d %d\n", gcolor[0], gcolor[1], gcolor[2], gcolor[3]);
}

void do_boundaryrule(Args& args) {
  const Bndrule bndrule = parse_boundaryrule(args.get_string());
  g_bndrules = twice(bndrule);
  for_int(i, 2) g_filterbs[i].set_bndrule(bndrule);
}

void do_hboundaryrule(Args& args) {
  const Bndrule bndrule = parse_boundaryrule(args.get_string());
  g_bndrules[1] = bndrule;
  g_filterbs[1].set_bndrule(bndrule);
}

void do_vboundaryrule(Args& args) {
  const Bndrule bndrule = parse_boundaryrule(args.get_string());
  g_bndrules[0] = bndrule;
  g_filterbs[0].set_bndrule(bndrule);
}

void do_cropsides(Args& args) {
  int vl = parse_size(args.get_string(), image.xsize(), false);
  int vr = parse_size(args.get_string(), image.xsize(), false);
  int vt = parse_size(args.get_string(), image.ysize(), false);
  int vb = parse_size(args.get_string(), image.ysize(), false);
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, V(vt, vl), V(vb, vr), g_bndrules, &gcolor);
}

void do_cropl(Args& args) {
  int v = parse_size(args.get_string(), image.xsize(), false);
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, V(0, v), V(0, 0), g_bndrules, &gcolor);
}

void do_cropr(Args& args) {
  int v = parse_size(args.get_string(), image.xsize(), false);
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, V(0, 0), V(0, v), g_bndrules, &gcolor);
}

void do_cropt(Args& args) {
  int v = parse_size(args.get_string(), image.ysize(), false);
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, V(v, 0), V(0, 0), g_bndrules, &gcolor);
}

void do_cropb(Args& args) {
  int v = parse_size(args.get_string(), image.ysize(), false);
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, V(0, 0), V(v, 0), g_bndrules, &gcolor);
}

void do_cropall(Args& args) {
  string s = args.get_string();
  Vec2<int> sides = V(parse_size(s, image.ysize(), false), parse_size(s, image.xsize(), false));
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, sides, sides, g_bndrules, &gcolor);
}

void do_cropsquare(Args& args) {
  int x = parse_size(args.get_string(), image.xsize(), true);
  int y = parse_size(args.get_string(), image.ysize(), true);
  int s = image.ysize() == image.xsize() ? parse_size(args.get_string(), image.ysize(), false) : args.get_int();
  const Vec2<int> p(y, x), p0 = p - (s / 2);
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, p0, image.dims() - p0 - s, g_bndrules, &gcolor);
}

void do_croprectangle(Args& args) {
  int x = parse_size(args.get_string(), image.xsize(), true);
  int y = parse_size(args.get_string(), image.ysize(), true);
  int sx = parse_size(args.get_string(), image.xsize(), false);
  int sy = parse_size(args.get_string(), image.ysize(), false);
  const Vec2<int> p(y, x), s(sy, sx), p0 = p - s / 2;
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, p0, image.dims() - p0 - s, g_bndrules, &gcolor);
}

void do_cropcoord(Args& args) {
  int x0 = parse_size(args.get_string(), image.xsize(), true);
  int y0 = parse_size(args.get_string(), image.ysize(), true);
  int x1 = parse_size(args.get_string(), image.xsize(), true);
  int y1 = parse_size(args.get_string(), image.ysize(), true);
  const Vec2<int> p0(y0, x0), p1(y1, x1);
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, p0, image.dims() - p1, g_bndrules, &gcolor);
}

void do_croptodims(Args& args) {
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx > 0 && ny > 0);
  auto ndims = V(ny, nx);
  auto yx0 = (image.dims() - ndims) / 2;
  auto yx1 = image.dims() - ndims - yx0;
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, yx0, yx1, g_bndrules, &gcolor);
}

void do_cropmatte() {
  const int nz = image.zsize();
  int l = 0, r = 0, t = 0, b = 0;
  for (; l < image.xsize(); l++) {
    int x = l;
    bool ok = true;
    for_int(y, image.ysize()) {
      if (!equal(image[y][x], gcolor, nz)) {
        ok = false;
        break;
      }
    }
    if (!ok) break;
  }
  for (; r < image.xsize() - l; r++) {
    int x = image.xsize() - 1 - r;
    bool ok = true;
    for_int(y, image.ysize()) {
      if (!equal(image[y][x], gcolor, nz)) {
        ok = false;
        break;
      }
    }
    if (!ok) break;
  }
  for (; t < image.ysize(); t++) {
    int y = t;
    bool ok = true;
    for_int(x, image.xsize()) {
      if (!equal(image[y][x], gcolor, nz)) {
        ok = false;
        break;
      }
    }
    if (!ok) break;
  }
  for (; b < image.ysize() - t; b++) {
    int y = image.ysize() - 1 - b;
    bool ok = true;
    for_int(x, image.xsize()) {
      if (!equal(image[y][x], gcolor, nz)) {
        ok = false;
        break;
      }
    }
    if (!ok) break;
  }
  Grid<2, Pixel>& grid = image;
  grid = crop(grid, V(t, l), V(b, r), g_bndrules, &gcolor);
}

void do_overlayimage(Args& args) {
  int x0 = parse_size(args.get_string(), image.xsize(), true);
  int y0 = parse_size(args.get_string(), image.ysize(), true);
  const Vec2<int> yx0(y0, x0);
  string filename = args.get_string();
  Image image2(filename);
  assertx(image.ok(yx0 + image2.dims() - 1));
  for (const auto& yx : range(image2.dims())) image[yx0 + yx] = image2[yx];
}

void do_drawrectangle(Args& args) {
  int x0 = parse_size(args.get_string(), image.xsize(), true);
  int y0 = parse_size(args.get_string(), image.ysize(), true);
  int x1 = parse_size(args.get_string(), image.xsize(), true);
  int y1 = parse_size(args.get_string(), image.ysize(), true);
  const Vec2<int> p0(y0, x0), p1(y1, x1);
  for_coordsL(p0, p1, [&](const Vec2<int>& yx) { image[yx] = gcolor; });
}

// *** scale

void do_filter(Args& args) {
  const Filter& filter = Filter::get(args.get_string());
  for_int(i, 2) g_filterbs[i].set_filter(filter);
}

void do_hfilter(Args& args) {
  const Filter& filter = Filter::get(args.get_string());
  g_filterbs[1].set_filter(filter);
}

void do_vfilter(Args& args) {
  const Filter& filter = Filter::get(args.get_string());
  g_filterbs[0].set_filter(filter);
}

void do_scaleunif(Args& args) {
  HH_TIMER("_scale");
  float s = args.get_float();
  image.scale(twice(s), g_filterbs, &gcolor);
}

void do_scalenonunif(Args& args) {
  HH_TIMER("_scale");
  float sx = args.get_float(), sy = args.get_float();
  image.scale(V(sy, sx), g_filterbs, &gcolor);
}

void do_scaletox(Args& args) {
  HH_TIMER("_scale");
  int nx = parse_size(args.get_string(), image.xsize(), false);
  assertx(nx > 0);
  float s = float(nx) / assertx(image.xsize());
  image.scale(twice(s), g_filterbs, &gcolor);
}

void do_scaletoy(Args& args) {
  HH_TIMER("_scale");
  int ny = parse_size(args.get_string(), image.ysize(), false);
  assertx(ny > 0);
  float s = float(ny) / assertx(image.ysize());
  image.scale(twice(s), g_filterbs, &gcolor);
}

void do_scaletodims(Args& args) {
  HH_TIMER("_scale");
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx > 0 && ny > 0);
  auto syx = convert<float>(V(ny, nx)) / convert<float>(image.dims());
  image.scale(syx, g_filterbs, &gcolor);
}

void do_scaleinside(Args& args) {
  HH_TIMER("_scale");
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx > 0 && ny > 0);
  image.scale(twice(min(convert<float>(V(ny, nx)) / convert<float>(image.dims()))), g_filterbs, &gcolor);
}

void do_scalehalf2n1() {
  HH_TIMER("_scale");
  if (image.xsize() == 1 && image.ysize() == 1) Warning("Image already 1x1; no effect");
  const Vec2<int> newdims = (image.dims() - 1) / 2 + 1;
  assertx((newdims - 1) * 2 + 1 == image.dims());
  Image timage(image);
  image.init(newdims);
  for (const auto& yx : range(image.dims())) image[yx] = timage[yx * 2];
}

// *** tops (to postscript)

int column_ps;

inline void output_hex_nibble(uint8_t c) { std::cout.put("0123456789abcdef"[c]); }

inline void output_hex_byte(uint8_t c) {
  // std::cout << sform("%02x", c);
  output_hex_nibble(c >> 4);
  output_hex_nibble(c & 0xF);
  column_ps += 2;
  if (column_ps >= 72) {
    std::cout << "\n";
    column_ps = 0;
  }
}

// Filterimage ~/data/image/lake.png -tops | gv
// Filterimage ~/data/image/lake.png -to jpg -tops | gv
void do_tops() {
  assertx(image.zsize() == 1 || image.zsize() == 3);
  const int cy = image.ysize(), cx = image.xsize(), cz = image.zsize();
  const bool is_jpg = image.suffix() == "jpg";
  // 6inch square centered on page at 4.25, 5.5in
  int x1, x2, y1, y2;
  {
    float aspectx = cx > cy ? 1.f : float(cx) / cy;
    float aspecty = cy > cx ? 1.f : float(cy) / cx;
    const float radius = 3.f;  // 3 inches
    x1 = int(4.25f * 72.f - aspectx * radius * 72.f + .5f);
    x2 = int(4.25f * 72.f + aspectx * radius * 72.f + .5f);
    y1 = int(5.50f * 72.f - aspecty * radius * 72.f + .5f);
    y2 = int(5.50f * 72.f + aspecty * radius * 72.f + .5f);
  }
  std::cout << "%!PS-Adobe-2.0 EPSF-1.2\n";
  std::cout << "%%BoundingBox: " << sform("%d %d %d %d\n", x1, y1, x2, y2);
  std::cout << "%%Pages: 1\n";
  std::cout << "%%Orientation: Portrait\n";
  std::cout << "%%Creator: Filterimage -tops\n";
  std::cout << "%%CreationDate: " << get_current_datetime() << "\n";
  std::cout << "%%LanguageLevel: 1\n";
  std::cout << "%%EndComments\n";
  if (1) {
    std::cout << "%%BeginPreview: 16 16 1 16\n";
    std::cout << "% FFFF\n% 8001\n% 8001\n% 8001\n";
    std::cout << "% 8001\n% 8001\n% 8001\n% 8001\n";
    std::cout << "% 8001\n% 8001\n% 8001\n% 8001\n";
    std::cout << "% 8001\n% 8001\n% 8001\n% FFFF\n";
    std::cout << "%%EndPreview\n";
  }
  std::cout << "%%EndProlog\n";
  std::cout << "%%Page: 1 1\n";
  std::cout << "gsave\n";
  std::cout << sform("%d %d translate\n", x1, y1);
  std::cout << sform("%d %d scale\n", x2 - x1, y2 - y1);
  if (1 && is_jpg) {
    std::cout << sform("/DeviceRGB setcolorspace\n");
    std::cout << sform("<<\n");
    std::cout << sform(" /ImageType 1\n");
    std::cout << sform(" /Width %d\n", cx);
    std::cout << sform(" /Height %d\n", cy);
    std::cout << sform(" /BitsPerComponent 8\n");
    std::cout << sform(" /Decode [0 1 0 1 0 1]\n");
    // std::cout << sform(" /ImageMatrix [%d 0 0 %d 0 0]\n", cx, cy);
    std::cout << sform(" /ImageMatrix [%d 0 0 %d 0 %d]\n", cx, -cy, cy);
    // could use /ASCII85Decode instead
    std::cout << sform(" /DataSource currentfile /ASCIIHexDecode filter\n");
    std::cout << sform("   <</Relax 1>> /DCTDecode filter\n");
    std::cout << sform(">>\n");
    std::cout << sform("image\n");
    std::cout.flush();
    {
      // Write the image to a temporary JPG file and read its raw bytes.
      TmpFile tmpfile("jpg");
      string filename = tmpfile.filename();
      image.write_file(filename);
      RFile fi(filename);
      for (;;) {
        uint8_t byte;
        if (!read_binary_raw(fi(), ArView(byte))) break;
        output_hex_byte(byte);
      }
    }
    if (column_ps) std::cout << "\n";
  } else {
    std::cout << sform("/hhpicstr %d string def\n", cx * cz);
    std::cout << sform("%d %d 8\n", cx, cy);
    // std::cout << sform("  [%d 0 0 %d 0 0]\n", cx, cy);
    std::cout << sform("  [%d 0 0 %d 0 %d]\n", cx, -cy, cy);
    std::cout << sform("  { currentfile hhpicstr readhexstring pop }\n");
    std::cout << sform("  false %d\n", cz);
    std::cout << sform("  colorimage\n");
    column_ps = 0;
    for (const auto& yx : range(image.dims())) for_int(z, cz) output_hex_byte(image[yx][z]);  // ? test bw image
    if (column_ps) std::cout << "\n";
  }
  std::cout << "% % end of image data\n";
  std::cout << "grestore\n";
  std::cout << "showpage\n";
  std::cout << "%%EOF\n";
  nooutput = true;
}

// *** misc

void do_flipvertical() { image.reverse_y(); }

void do_fliphorizontal() { image.reverse_x(); }

void do_rot180() {
  // == -flipvertical -fliphorizontal
  image = rotate_ccw(image, 180);
}

// frame maps from destination pixel (y, x) to source pixel (y, x);
//  both have (possibly rectangular) domain [-0.5, +0.5]^2
void apply_frame(const Frame& frame) {
  HH_STIMER("_apply_frame");
  Vector4 vgcolor;
  convert(CGrid1View(gcolor), Grid1View(vgcolor));
  Matrix<Vector4> matv(image.dims());
  convert(image, matv);
  Matrix<Vector4> nmatv(image.dims());
  transform(matv, frame, g_filterbs, nmatv, &vgcolor);
  convert(nmatv, image);
}

// Compare 2X scaling using transform and built-in scaling
// Filterimage ~/data/image/lake.png -filter o -boundaryrule r -cropall -50% -gtransf "F 0  0.5 0 0  0 0.5 0  0 0 1  0 0 0  0" >v1.png
// Filterimage ~/data/image/lake.png -filter o -scaleu 2.0 >v2.png
// Filterimage v1.png -compare v2.png
//
// e.g. 10 degree rotation (on anisometric rectangle!):  Filterimage ~/data/image/lake.png -filter o -gtransf "F 0  0.9848 -0.1736 0  0.1736 0.9848 0  0 0 1  0 0 0  0" | imgv
// e.g. add 3X scale:  Filterimage ~/data/image/lake.png -boundaryrule r -cropall -100% -filter o -gtransf "F 0  0.32827 -0.05788 0  0.05788 0.32827 0  0 0 1  0 0 0  0" | imgv
//  ca cos(10/45*atan2(1, 1)), sin(10/45*atan2(1, 1))
void do_gtransf(Args& args) {
  Frame frame = FrameIO::parse_frame(args.get_string());
  apply_frame(frame);
}

// e.g. 10 degree   :  Filterimage ~/data/image/lake.png -filter o -boundaryrule r -rotate 10 | imgv
// 36-times repeated:  Filterimage ~/data/image/lake.png -filter o -boundaryrule r `perl -e 'binmode(STDOUT); for (1..36) { print "-rotate 10 "; }'` | imgv
// large image      :  time Filterimage ~/data/image/rampart1.jpg -filter o -boundaryrule r `perl -e 'binmode(STDOUT); for (1..1) { print "-rotate 10 "; }'` >v.jpg && imgv v.jpg
// Filterimage ~/data/image/lake.png -filt k `perl -e 'binmode(STDOUT); for (1..20) { print "-rotate 18 "; }'` | imgv
void do_rotate(Args& args) {
  float ang = args.get_float();
  int iang = int(ang);
  if (ang == iang && my_mod(iang, 90) == 0 && abs(iang) <= 270) {
    image = rotate_ccw(image, iang);
  } else {
    Frame f_rot = Frame::rotation(2, -to_rad(ang));  // backward map from new image to old image (but y, x reversed)
    Frame frame;
    if (0) {  // anisometric rotation if image is rectangle rather than square!
      frame = f_rot;
    } else {  // rigid rotation
      float vmin = float(min(image.dims()));
      Frame fscaling = Frame::scaling(V(image.ysize() / vmin, image.xsize() / vmin, 1.f));
      frame = fscaling * f_rot * ~fscaling;
    }
    apply_frame(frame);
  }
}

void do_permutecolors() {
  // drawback: considers each channel separately, so if red is all zero, it remains that way.
  //   perl -e 'binmode(STDOUT); for (0..255) { printf "%03d\n", ($_ * 53) % 256; }'
  //   factors 53
  Vec<uint8_t, 256> lookup;
  for_int(i, 256) lookup[i] = (i * 53) % 256;
  parallel_for_coords(
      image.dims(), [&](const Vec2<int>& yx) { for_int(z, image.zsize()) image[yx][z] = lookup[image[yx][z]]; }, 10);
}

void do_randomizeRGB() {
  parallel_for_coords(
      image.dims(),
      [&](const Vec2<int>& yx) {
        unsigned v =
            unsigned(image[yx][0] * (53 + 113 * 256 + 43 * 65536) + image[yx][1] * (97 + 89 * 256 + 107 * 65536) +
                     image[yx][2] * (11 + 61 * 256 + 47 * 65536)) %
            (1 << 24);
        image[yx] = Pixel(narrow_cast<uint8_t>((v >> 0) & 255), narrow_cast<uint8_t>((v >> 8) & 255),
                          narrow_cast<uint8_t>((v >> 16) & 255));
      },
      20);
}

void do_noisegaussian(Args& args) {
  float sd = args.get_float();
  for (const auto& yx : range(image.dims())) {
    for_int(z, image.zsize()) {
      image[yx][z] = clamp_to_uint8(int(float(image[yx][z]) + Random::G.gauss() * sd + .5f));
    }
  }
}

void do_blur(Args& args) {
  // e.g.: Filterimage ~/data/image/lake.png -blur 1 | imgv
  // Filterimage ~/data/image/rampart1.jpg -info -blur 1 -info | imgv
  //  old  (_blur:                   8.74  x8.0      8.82)
  //  new  (_blur:                   0.34  x6.5      0.37)
  // Filterimage ~/data/image/rampart1.jpg -info -blur 2 -info | imgv
  //  old  (_blur:                  32.00  x8.0     32.83)
  //  new  (_blur:                   0.50  x7.2      0.54)
  HH_TIMER("_blur");
  const float sdv_pixels = args.get_float();  // 1sdv in pixels
  const float nsvd = 2.5f;
  const int r = int(nsvd * sdv_pixels + .5f);  // window radius
  Array<float> ar_gauss(2 * r + 1);            // cached Gaussian weights
  for_int(i, 2 * r + 1) ar_gauss[i] = gaussian(float(i) - r, sdv_pixels);
  // SHOW(ar_gauss);
  if (0) {
    {  // normalize the 1D weights such that 2D tensor sums to 1 over discrete 2D window
      double sum = 0.;
      for_intL(dy, -r, r + 1) for_intL(dx, -r, r + 1) sum += ar_gauss[dy + r] * ar_gauss[dx + r];
      for_int(i, 2 * r + 1) {
        ar_gauss[i] /= sqrt(float(sum));
        if (0) SHOW(ar_gauss[i]);
      }
    }
    Grid<2, Vector4> grid(image.dims());
    parallel_for_coords(
        image.dims(), [&](const Vec2<int>& yx) { grid[yx] = Vector4(image[yx]); }, 6);
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          Vector4 vec{};
          for_coordsL(twice(-r), twice(r + 1), [&](const Vec2<int>& dyx) {
            vec += ar_gauss[dyx[0] + r] * ar_gauss[dyx[1] + r] * grid.inside(yx + dyx, g_bndrules);
          });
          image[yx] = vec.pixel();
        },
        1000);
  } else {
    ar_gauss /= float(sum(ar_gauss));
    // SHOW(ar_gauss); SHOW(sum(ar_gauss));
    for_int(d, 2) image = convolve_d(image, d, ar_gauss, Bndrule::reflected);
  }
}

Vec2<int> as_fit_dims;
int as_crop_vl, as_crop_vr, as_crop_vt, as_crop_vb;

void do_as_fit(Args& args) {
  as_fit_dims[1] = args.get_int();  // nx
  as_fit_dims[0] = args.get_int();  // ny
}

void do_as_cropsides(Args& args) {
  as_crop_vl = args.get_int();
  as_crop_vr = args.get_int();
  as_crop_vt = args.get_int();
  as_crop_vb = args.get_int();
}

void apply_assemble_operations(Grid<2, Pixel>& im, const Vec2<int>& yx, const Vec2<int>& gdims) {
  if (!is_zero(as_fit_dims)) {
    {
      const float vscale = min(convert<float>(as_fit_dims) / convert<float>(im.dims()));
      Vec2<int> newdims = convert<int>(convert<float>(im.dims()) * vscale + .5f);
      Grid<2, Pixel> newim(newdims);
      scale_Matrix_Pixel(im, g_filterbs, &gcolor, newim);
      im = std::move(newim);
    }
    Vec2<int> side0 = (as_fit_dims - im.dims()) / 2;
    Vec2<int> side1 = as_fit_dims - im.dims() - side0;
    im = crop(im, -side0, -side1, twice(Bndrule::border), &gcolor);
  }
  if (as_crop_vl || as_crop_vr || as_crop_vt || as_crop_vb) {
    int vl = as_crop_vl, vr = as_crop_vr, vt = as_crop_vt, vb = as_crop_vb;
    if (1) {
      // Avoid negative crop (i.e. introduction of padding region) at outer boundaries of domain.
      if (yx[1] == 0) vl = max(vl, 0);
      if (yx[1] == gdims[1] - 1) vr = max(vr, 0);
      if (yx[0] == 0) vt = max(vt, 0);
      if (yx[0] == gdims[0] - 1) vb = max(vb, 0);
    }
    im = crop(im, V(vt, vl), V(vb, vr), twice(Bndrule::border), &gcolor);
  }
}

void assemble_images(CMatrixView<Image> images) {
  for (const auto& yx : range(images.dims())) assertx(images[yx].zsize() == images[0][0].zsize());
  image.init(V(0, 0));
  image.set_zsize(images[0][0].zsize());  // attributes copied outside this function
  image = assemble(images, gcolor);
}

void do_assemble(Args& args) {
  // Filterimage ~/data/image/lake.png -disassemble 128 128 rootname
  // Filterimage -assemble 2 2 rootname.{0.0,1.0,0.1,1.1}.png >lake.same.png
  // Filterimage -as_fit 480 320 -as_cropsides -8 -8 -8 -8 -assemble 2 2 ~/data/image/lake.png{,,,} | vv -
  // (cd $HOMEPATH/Dropbox/Pictures/2014/india; Filterimage -nostdin -color 0 0 0 255 -as_fit 640 640 -as_cropsides -4 -4 -4 -4 -assemble -1 -1 2*.jpg | vv -)
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx >= -1 && ny >= -1);
  if (nx > 0 && ny > 0) args.ensure_at_least(nx * ny);
  Array<string> lfilenames;
  while (args.num() && args.peek_string()[0] != '-') lfilenames.push(args.get_filename());
  if (nx <= 0 && ny <= 0) {
    nx = ny = max(int(ceil(sqrt(float(lfilenames.num())))), 1);
  } else if (nx <= 0) {
    nx = (lfilenames.num() - 1) / ny + 1;
  } else if (ny <= 0) {
    ny = (lfilenames.num() - 1) / nx + 1;
  }
  Matrix<Image> images(V(ny, nx));
  assertx(images.size() >= lfilenames.size());
  Matrix<string> filenames(images.dims());
  for (const auto& yx : range(filenames.dims())) {
    if (!lfilenames.num()) continue;
    string filename = lfilenames.shift();
    filenames[yx] = filename;
    if (!file_requires_pipe(filename) && !file_exists(filename)) assertnever("file '" + filename + "' not found");
  }
  parallel_for_coords(images.dims(), [&](const Vec2<int>& yx) {
    if (filenames[yx] == "") return;
    if (images.size() >= 10) images[yx].set_silent_io_progress(true);
    images[yx].read_file(filenames[yx]);
    apply_assemble_operations(images[yx], yx, images.dims());
  });  // we can assume that parallelism is justified
  assemble_images(images);
  image.attrib() = images[0][0].attrib();
}

void do_fromtxt(Args& args) {
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx > 0 && ny > 0);
  int nch = args.get_int();
  assertx(nch == 1 || nch == 3 || nch == 4);
  string filename = args.get_filename();
  image.init(V(ny, nx));
  image.set_zsize(nch);
  RFile fi(filename);
  Array<float> aval;
  aval.reserve(ny * nx * nch);
  for (const auto& yx : range(image.dims())) {
    for_int(c, nch) {
      dummy_use(yx);
      float val;
      assertx(fi() >> val);
      aval.push(val);
    }
  }
  if (1) {
    float dummy_val;
    fi() >> dummy_val;
    assertw(!fi());
  }
  if (0) SHOW(aval[0], aval.last());
  const float* pval = aval.data();
  for (const auto& yx : range(image.dims())) {
    for_int(c, nch) {
      float val = *pval++;
      image[yx][c] = uint8_t(clamp(val, 0.f, 1.f) * 255.f + .5f);
    }
  }
}

void do_invideo(Args& args) {
  string filename = args.get_filename();
  RVideo rvideo(filename);
  showf("Reading video %s\n", Video::diagnostic_string(rvideo.dims(), rvideo.attrib()).c_str());
  unique_ptr<WVideo> pwvideo;  // not defined now because we do not yet know the frame size
  ConsoleProgress cprogress("Invideo");
  image.init(rvideo.spatial_dims());
  for (int i = 0;; i++) {
    if (rvideo.nframes()) cprogress.update(float(i) / rvideo.nframes());
    if (!rvideo.read(image)) break;
    ParseArgs parseargs{Array<string>{}};
    parseargs.copy_parse(*g_parseargs);
    parseargs.parse();
    if (!pwvideo) pwvideo = make_unique<WVideo>("-", image.dims(), rvideo.attrib());
    pwvideo->write(image);
  }
  while (args.num()) args.get_string();  // discard already parsed arguments
  nooutput = true;
}

void do_gridcrop(Args& args) {
  // Filterimage ~/data/image/lake.png -as_cropsides -1 -1 -1 -1 -gridcrop 3 3 20 20 | imgv
  int nx = args.get_int(), ny = args.get_int();
  assertx(nx >= 2 && ny >= 2);
  int sx = args.get_int(), sy = args.get_int();
  assertx(sx >= 0 && sx <= image.xsize() && sy >= 0 && sy <= image.ysize());
  Matrix<Image> images(V(ny, nx));
  parallel_for_coords(
      images.dims(),
      [&](const Vec2<int>& yx) {
        int vl = int((image.xsize() - sx) * float(yx[1]) / (images.dims()[1] - 1) + .5f);
        int vt = int((image.ysize() - sy) * float(yx[0]) / (images.dims()[0] - 1) + .5f);
        int vr = image.xsize() - vl - sx;
        int vb = image.ysize() - vt - sy;
        if (0) showf("vl=%d vt=%d  vr=%d vb=%d\n", vl, vt, vr, vb);
        Grid<2, Pixel>& im = images[yx];
        im = crop(image, V(vt, vl), V(vb, vr), g_bndrules, &gcolor);
        apply_assemble_operations(im, yx, images.dims());
      },
      sy * sx * 4);
  assemble_images(images);
}

void do_disassemble(Args& args) {
  // Filterimage ~/data/image/lake.png -disassemble 128 128 rootname
  // Filterimage -assemble 2 2 rootname.{0.0,1.0,0.1,1.1}.png >lake.same.png
  int tilex = args.get_int();
  assertx(tilex > 0);
  int tiley = args.get_int();
  assertx(tiley > 0);
  const Vec2<int> tiledims(tiley, tilex), atiles = image.dims() / tiledims;
  assertx(atiles * tiledims == image.dims());
  string rootname = args.get_filename();
  string suffix = image.suffix();
  assertx(suffix != "");
  parallel_for_coords(atiles, [&](const Vec2<int>& yx) {
    Image nimage(tiledims);
    nimage.attrib() = image.attrib();
    for (const auto& yxd : range(tiledims)) nimage[yxd] = image[yx * tiledims + yxd];
    nimage.set_silent_io_progress(true);
    nimage.write_file(sform("%s.%d.%d.%s", rootname.c_str(), yx[1], yx[0], suffix.c_str()));
  });
  nooutput = true;
}

void do_tile(Args& args) {
  const int nx = args.get_int(), ny = args.get_int();
  assertx(nx > 0 && ny > 0);
  const Vec2<int> atiles(ny, nx);
  Image timage(image);
  image.init(timage.dims() * atiles);
  parallel_for_coords(
      atiles,
      [&](const Vec2<int>& yx) {
        for (const auto& yxd : range(timage.dims())) image[yx * timage.dims() + yxd] = timage[yxd];
      },
      product(timage.dims()) * 4);
}

void do_replace(Args& args) {
  Pixel newcolor;
  for_int(i, 4) {
    int v = args.get_int();
    assertx(v >= 0 && v <= 255);
    newcolor[i] = narrow_cast<uint8_t>(v);
  }
  int count = 0;
  for (const auto& yx : range(image.dims())) {
    bool is_match = tolerance ? dist2(image[yx], gcolor) <= tolerance : equal(image[yx], gcolor, image.zsize());
    if (is_match ^ g_not) {
      count++;
      image[yx] = newcolor;
    }
  }
  showf("Replaced %d pixels\n", count);
}

void do_gamma(Args& args) {
  float gamma = args.get_float();
  Vec<uint8_t, 256> transf;
  for_int(i, 256) transf[i] = uint8_t(clamp(pow(i / 255.f, gamma), 0.f, 1.f) * 255.f + .5f);
  parallel_for_coords(
      image.dims(), [&](const Vec2<int>& yx) { for_int(z, image.zsize()) image[yx][z] = transf[image[yx][z]]; }, 10);
}

void do_tobw() { image.to_bw(); }

void do_tocolor() { image.to_color(); }

void do_hue() {
  assertx(image.zsize() >= 3);
  // https://en.wikipedia.org/wiki/Grayscale
  // To convert any color to a grayscale representation of its luminance, first one must obtain the values of
  // its red, green, and blue (RGB) primaries in linear intensity encoding, by gamma expansion. Then, add together
  // 30% of the red value, 59% of the green value, and 11% of the blue value (these weights depend on the exact
  // choice of the RGB primaries, but are typical). The formula (11*R + 16*G + 5*B) /32 is also popular since it
  // can be efficiently implemented using only integer operations.
  const float gamma = 2.2f;
  parallel_for_coords(
      image.dims(),
      [&](const Vec2<int>& yx) {
        Vec3<float> af;
        for_int(z, 3) af[z] = pow(image[yx][z] + 0.5f, gamma);
        float gray = af[0] * .30f + af[1] * .59f + af[2] * .11f;
        for_int(z, 3) af[z] = af[z] * pow(128.f, gamma) / gray;
        for_int(z, 3) image[yx][z] = clamp_to_uint8(int(pow(af[z], 1.f / gamma) + .5f));
      },
      50);
}

void do_noalpha() {
  if (image.zsize() == 4) {
    // Assume alpha is an independent channel; RGB not premultiplied by alpha.
    parallel_for_coords(
        image.dims(), [&](const Vec2<int>& yx) { image[yx][3] = 255; }, 30);
  } else if (image.zsize() == 1) {
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          image[yx][1] = image[yx][0];
          image[yx][2] = image[yx][0];
          image[yx][3] = 255;
        },
        3);
  }
  image.set_zsize(3);
}

void do_undoalpha() {
  if (image.zsize() == 4) {
    // Composite any transparent color above current gcolor.  Assume premultiplied alpha.
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          for_int(z, 3) {
            // C_new = C_f * alpha_f + C_b * alpha_b * (1 - alpha_f)
            image[yx][z] = clamp_to_uint8(int(image[yx][z] + (1.f - image[yx][3] / 255.f) * gcolor[z]));
          }
        },
        30);
  }
  do_noalpha();
}

void do_readalpha(Args& args) {
  string filename = args.get_filename();
  if (image.zsize() != 4) {
    assertx(image.zsize() == 3);
    image.set_zsize(4);
  }
  Image ialpha(filename);
  assertx(same_size(ialpha, image));
  parallel_for_coords(
      image.dims(),
      [&](const Vec2<int>& yx) {
        image[yx][3] = ialpha[yx][0];
        for_int(c, 3) image[yx][c] = uint8_t(float(image[yx][c]) * image[yx][3] / 255.f + .5f);  // premultiplied alpha
      },
      10);
}

void do_setalpha(Args& args) {
  int av = args.get_int();
  assertx(av >= 0 && av <= 255);
  if (image.zsize() != 4) {
    assertx(image.zsize() == 3);
    image.set_zsize(4);
  }
  parallel_for_coords(
      image.dims(), [&](const Vec2<int>& yx) { image[yx][3] = uint8_t(av); }, 1);
}

void do_getalpha() {
  assertx(image.zsize() == 4);
  parallel_for_coords(
      image.dims(), [&](const Vec2<int>& yx) { image[yx] = Pixel::gray(image[yx][3]); }, 2);
  image.set_zsize(3);
}

void do_matchtoalpha() {
  if (image.zsize() != 4) {
    assertx(image.zsize() == 3);
    image.set_zsize(4);
  }
  parallel_for_coords(
      image.dims(), [&](const Vec2<int>& yx) { image[yx][3] = image[yx] == gcolor ? 255 : 0; }, 2);
}

// *** quadpullpush

template <int D> int64_t count_undef(CGridView<D, Pixel> grid) {
  return count_if(grid, [](const Pixel& pix) {
    assertx(pix[3] == 0 || pix[3] == 255);
    return pix[3] == 0;
  });
}

template <int D> void quad_pullpush(GridView<D, Pixel> grid) {
  if (!grid.size()) return;
  if (min(grid.dims()) == 1) {
    assertx(count_undef(grid) == 0);
    return;
  }
  Grid<D, Pixel> hgrid((grid.dims() + 1) / 2);
  for (const auto& hu : range(hgrid.dims())) {
    Vec3<int> accum(0, 0, 0);
    int num_def = 0;
    for (const auto& ud : range(ntimes<D>(2))) {
      Vec<int, D> u = hu * 2 + ud;
      if (!grid.ok(u)) continue;
      const Pixel& pix = grid[u];
      if (pix[3]) {
        for_int(c, 3) accum[c] += pix[c];
        num_def++;
      }
    }
    if (num_def) {
      for_int(c, 3) hgrid[hu][c] = uint8_t(accum[c] / num_def);
      hgrid[hu][3] = 255;
    } else {
      hgrid[hu][3] = 0;
    }
  }
  quad_pullpush(hgrid);  // recursive call
  ASSERTX(count_undef(hgrid) == 0);
  for (const auto& hu : range(hgrid.dims())) {
    for (const auto& ud : range(ntimes<D>(2))) {
      Vec<int, D> u = hu * 2 + ud;
      if (!grid.ok(u)) continue;
      Pixel& pix = grid[u];
      if (pix[3]) continue;
      pix = hgrid[hu];
      ASSERTX(pix[3] != 0);  // now defined
    }
  }
}

// Filterimage ~/data/image/texture256.input.png -color 255 0 0 255 -info -quadpullpush -info | imgv
void do_quadpullpush() {
  assertx(image.zsize() == 3);  // no alpha
  for (const auto& yx : range(image.dims())) {
    bool is_undef = rgb_equal(image[yx], gcolor) ^ g_not;
    image[yx][3] = is_undef ? 0 : 255;
  }
  int64_t num_undef = count_undef(image);
  showf("quadpullpush: filling in %d undefined pixels\n", assert_narrow_cast<int>(num_undef));
  quad_pullpush(image);
  assertx(count_undef(image) == 0);  // optional
}

// *** softpullpush

// Given a container of Vector4, undo the premultiplication of the alpha ([3]) channel.
template <typename Container> auto undo_alpha(Container&& c) {
  return map(c, [](const Vector4& v) { return v / (v[3] + 1e-7f); });
}

// Given a container of Vector4, extract just the alpha channel as a new container of float.
template <typename Container> auto just_alpha(Container&& c) {
  return map(c, [](const Vector4& v) { return v[3]; });
}

// Pull-push from [Lumigraph 1996] but with several modifications:
// - dual-subdivision structure between resolution levels
// - use of Vector4: [3] == w weight, and [0..2] == w * x premultiplied color
// - could introduce arbitrary filtering kernels
template <int D> void new_pullpush(GridView<D, Vector4> grid) {
  if (!grid.size()) return;
  if (0) {
    int64_t num_undef = count_if(grid, [](const Vector4& v) { return v[3] == 0.f; });
    int64_t num_def = count_if(grid, [](const Vector4& v) { return v[3] == 1.f; });
    int64_t n = grid.size();
    SHOW(n, num_def, num_undef, n - num_def - num_undef);
  }
  const bool primal = false;
  if (grid.size() == 1) {
    assertx(grid[ntimes<D>(0)][3] > 0.f);  // at least one defined value in grid
    return;
  }
  const Vector4 vzero(0.f);
  const Vec<int, D> dims = grid.dims();
  const Vec<int, D> hdims = (dims + 1) / 2;
  Grid<D, Vector4> hgrid;  // half-resolution grid
  {                        // pull step
    // clamp weights prior to downscaling
    for (const auto& u : range(dims)) grid[u] *= 1.f / max(grid[u][3], 1.f);  // make weight at most 1.f
    if (!primal) {
      // - This dual structure is better than original lumigraph pull-push when used over power-of-two grids
      //    because it doesn't shift the content off the right/bottom boundaries at coarse levels.
      // - It is important to use Bndrule::border to avoid introducing excessive confidence at coarse levels.
      // - Filter "triangle" is nicer overall than original lumigraph although it introduces a bit of ringing.
      // Expand to even dimensions.
      Grid<D, Vector4> grid2 = crop(grid, ntimes<D>(0), -dims % 2, ntimes<D>(Bndrule::border), &vzero);
      // Using filter "triangle" (with weights (1 / 4, 3 / 4, 3 / 4, 1 / 4) / 2) is dual to
      //  the upscaling "triangle" kernel (with alternating weights (3 / 4, 1 / 4) and (1 / 4, 3 / 4)).
      // It introduces a little bit of undesirable ringing.  "box" has no ringing but inferior.
      const auto downscaling_kernel = ntimes<D>(FilterBnd(Filter::get("triangle"), Bndrule::border));
      hgrid = scale(grid2, hdims, downscaling_kernel, &vzero);  // spatially downscale
      const float adjust_weight = pow(2.f, float(D));           // somewhat arbitrary
      for (auto& e : hgrid) e *= adjust_weight;
    } else {
      // - Somehow the coarse-scale content is getting shifted right/bottom unlike in the original Lumigraph
      //   implementation; the cause is unknown.
      // Expand to odd dimensions.
      Grid<D, Vector4> grid2 =
          crop(grid, ntimes<D>(0), -(ntimes<D>(1) - dims % 2), ntimes<D>(Bndrule::border), &vzero);
      assertx(grid2.dims() % 2 == ntimes<D>(1));
      const auto downscaling_kernel = ntimes<D>(FilterBnd(Filter::get("triangle"), Bndrule::border));
      hgrid = scale_primal(grid2, (grid2.dims() - 1) / 2 + 1, downscaling_kernel, &vzero);  // spatially downscale
      if (0) SHOW(grid.dims(), grid2.dims(), hgrid.dims(), hdims, hgrid.dims() - hdims);
      if (hgrid.size() >= grid.size() || 0)
        hgrid = crop(hgrid, ntimes<D>(0), hgrid.dims() - hdims);  // optionally crop extra
      const float adjust_weight = .5f * pow(2.f, float(D));       // somewhat arbitrary
      for (auto& e : hgrid) e *= adjust_weight;
    }
  }
#if 1  // Only for D == 2.
  if (0) as_image(hgrid).write_file(sform("hgrid.%03dx%03d.png", hgrid.xsize(), hgrid.ysize()));
  if (0) as_image(undo_alpha(hgrid)).write_file(sform("hgridx.%03dx%03d.png", hgrid.xsize(), hgrid.ysize()));
  if (0) as_image(just_alpha(hgrid)).write_file(sform("hgridw.%03dx%03d.png", hgrid.xsize(), hgrid.ysize()));
#endif
  new_pullpush(hgrid);
  {  // push step
    // clamp weights prior to upscaling
    for (const auto& u : range(hdims)) hgrid[u] *= 1.f / max(hgrid[u][3], 1.f);  // make weight at most 1.f
    Grid<D, Vector4> gridu;
    if (!primal) {
      const auto upscaling_kernel = ntimes<D>(FilterBnd(Filter::get("triangle"), Bndrule::border));  // bilinear
      // This bilinear prolongation has weights (3/4, 1/4) and (1/4, 3/4) on alternating entries.
      gridu = scale(hgrid, hdims * 2, upscaling_kernel, &vzero);  // spatially upscale
      gridu = crop(gridu, ntimes<D>(0), dims % 2);                // crop any extra expanded dimensions
    } else {
      const auto upscaling_kernel = ntimes<D>(FilterBnd(Filter::get("triangle"), Bndrule::border));  // bilinear
      // expand if original dimension was even
      Grid<D, Vector4> grid2 =
          crop(hgrid, ntimes<D>(0), -(ntimes<D>(1) - dims % 2), ntimes<D>(Bndrule::border), &vzero);
      gridu = scale(grid2, (grid2.dims() - 1) * 2 + 1, upscaling_kernel, &vzero);  // spatially upscale
      if (0) SHOW(hgrid.dims(), grid2.dims(), gridu.dims(), grid.dims(), gridu.dims() - grid.dims());
      gridu = crop(gridu, ntimes<D>(0), gridu.dims() - grid.dims());  // crop any extra
    }
    assertx(gridu.dims() == grid.dims());
    for (const auto& u : range(dims)) {
      // composite grid over gridu == upsample(hgrid)
      // C_new = C_f + (1 - alpha_f) * C_b
      const float w = grid[u][3];
      grid[u] = grid[u] + (1.f - w) * gridu[u];
    }
  }
#if 1  // Only for D == 2.
  if (0) as_image(grid).write_file(sform("grid.%03dx%03d.png", grid.xsize(), grid.ysize()));
  if (0) as_image(undo_alpha(grid)).write_file(sform("gridx.%03dx%03d.png", grid.xsize(), grid.ysize()));
  if (0) as_image(just_alpha(grid)).write_file(sform("gridw.%03dx%03d.png", grid.xsize(), grid.ysize()));
#endif
}

// Filterimage ~/data/image/texture256.input.png -color 255 0 0 255 -info -softpullpush -info >v.png
void do_softpullpush() {
  Matrix<Vector4> mat(image.dims());
  for (const auto& yx : range(image.dims())) {
    bool is_undef = rgb_equal(image[yx], gcolor) ^ g_not;
    mat[yx] = is_undef ? Vector4(0.f) : Vector4(image[yx]);
    // HH_SSTAT(Sa, mat[yx][3]);
  }
  new_pullpush(mat);
  for (const auto& yx : range(image.dims())) {
    assertx(mat[yx][3] > 0.f);
    image[yx] = (mat[yx] / mat[yx][3]).pixel();
  }
}

// *** voronoidilate

void do_voronoidilate() {
  // Filterimage ~/git/mesh_processing/demos/data/texture.input.png -filter i -scaleu 10 -color 255 0 0 255 -voronoidilate -noo
  // Using Vec2<float>:  win: 0.74  gcc: 0.23
  // Using Vec2<int>  :  win: 0.27  gcc: 0.19
  HH_TIMER("_voronoi");
  if (image.suffix() == "jpg") assertnever("euclidean_distance_map not useful on jpg image");
  Matrix<Vec2<int>> mvec(image.dims(), image.dims());
  int num_undef = 0;
  for (const auto& yx : range(image.dims())) {
    bool is_undef = rgb_equal(image[yx], gcolor) ^ g_not;
    if (!is_undef) mvec[yx] = V(0, 0);
    num_undef += is_undef;
  }
  showf("voronoidilate: filling in %d undefined pixels\n", num_undef);
  euclidean_distance_map(mvec);
  // Fill in unfilled pixels
  for (const auto& yx : range(image.dims())) {
    if (mag2(mvec[yx]) == 0) continue;
    assertx(mvec[yx][0] < image.ysize());  // no more large (undefined) values
    HH_SSTAT(Svoronoidist, mag<float>(mvec[yx]));
    image[yx] = image[yx + mvec[yx]];
  }
}

void do_featureoffsets() {
  if (image.suffix() == "jpg") assertnever("euclidean_distance_map not useful on jpg image");
  Matrix<Vec2<int>> mvec(image.dims(), image.dims());  // (mvec is initialized with large values image.dims())
  for (const auto& yx : range(image.dims())) {
    bool is_undef = rgb_equal(image[yx], gcolor) ^ g_not;
    if (!is_undef) mvec[yx] = V(0, 0);
  }
  euclidean_distance_map(mvec);
  for (const auto& yx : range(image.dims())) {
    assertx(mvec[yx][0] < image.ysize());  // no more large (undefined) values
    HH_SSTAT(Svoronoidist, mag<float>(mvec[yx]));
    if (1) {
      for_int(c, 2) {
        int i = mvec[yx][c];
        if (1) {  // linear
          i = 128 + i * 12;
        } else {  // arctan
          float f = float(i);
          // 128 + 70 * ArcTan[f * 1]
          f = 128.f + 70.f * std::atan2(f, 1.f);
          i = int(f + .5f);
        }
        assertw(i >= 0 && i <= 255);
        image[yx][c] = uint8_t(i);
      }
    } else {  // scalar arctan
      float f = mag<float>(mvec[yx]);
      f = 0.f + 150.f * std::atan2(f, 1.f);
      int i = int(f + .5f);
      assertx(i >= 0 && i <= 255);
      image[yx][0] = uint8_t(i);
      image[yx][1] = 0;
    }
    image[yx][2] = 0;
  }
}

void do_normalizenor() {
  assertx(image.zsize() == 3);  // no alpha
  for (const auto& yx : range(image.dims())) {
    Pixel& pix = image[yx];
    Vector v;
    for_int(c, 3) v[c] = float(pix[c]) / 255.f * 2.f - 1.f;
    assertw(v.normalize());
    for_int(c, 3) pix[c] = uint8_t((v[c] + 1.f) / 2.f * 255.f + .5f);
  }
}

void do_shadenor(Args& args) {
  Vector lightdir;
  for_int(c, 3) lightdir[c] = args.get_float();
  assertx(image.zsize() == 3);  // no alpha
  const bool twolights = getenv_bool("G3D_TWOLIGHTS");
  for (const auto& yx : range(image.dims())) {
    Pixel& pix = image[yx];
    Vector vnor;
    for_int(c, 3) vnor[c] = float(pix[c]) / 255.f * 2.f - 1.f;
    if (vnor == Vector(-1.f, -1.f, -1.f)) {
      Warning("not shading black pixel");
    } else {
      float vdot = dot(vnor, lightdir);
      if (twolights && vdot < 0.f) vdot = -vdot;
      vdot = clamp(vdot, .05f, 1.f);
      for_int(c, 3) pix[c] = uint8_t(vdot * 255.f + .5f);
    }
  }
}

void do_shadefancy(Args& args) {
  Frame frame = FrameIO::parse_frame(args.get_string());
  assertx(image.zsize() == 3);  // no alpha
  Array<Vector> ld;             // direction
  Array<Vector> lc;             // color
  Array<float> li;              // intensity
  Array<float> ls;              // specularity
  // G3dOGL: const Vector lightdireye(-1.f, -.6f, .3f);
  ld.push(Vector(-1.f, -.6f, +.3f));
  lc.push(Vector(1.f, 1.f, 1.f));
  li.push(.82f);
  ls.push(1.5f);
  ld.push(Vector(+.8f, +.4f, +.0f));
  lc.push(Vector(.6f, .6f, .3f));
  li.push(1.2f);
  ls.push(1.5f);
  ld.push(Vector(-.1f, +.9f, +.0f));
  lc.push(Vector(.9f, .1f, .1f));
  li.push(0.6f);
  ls.push(4.f);
  ld.push(Vector(-.1f, -.9f, +.0f));
  lc.push(Vector(.9f, .1f, .1f));
  li.push(0.6f);
  ls.push(4.f);
  ld.push(Vector(+.1f, +.0f, +.9f));
  lc.push(Vector(.6f, .5f, .1f));
  li.push(0.6f);
  ls.push(4.f);
  for_int(i, ld.num()) {
    ld[i].normalize();
    ld[i] *= frame;
  }
  for (const auto& yx : range(image.dims())) {
    Pixel& pix = image[yx];
    Vector vnor;
    for_int(c, 3) vnor[c] = float(pix[c]) / 255.f * 2.f - 1.f;
    Vector vcol{};
    for_int(i, ld.num()) {
      float vdot = dot(vnor, ld[i]);
      vdot = clamp(vdot, 0.f, 1.f);
      vdot = pow(vdot, ls[i]);
      vcol += vdot * li[i] * lc[i];
    }
    for_int(c, 3) pix[c] = uint8_t(clamp(vcol[c], 0.f, 1.f) * 255.f + .5f);
  }
}

void do_cycle(Args& args) {
  int ncycle = args.get_int();
  assertx(ncycle > 0);
  // const int low = 40, high = 190, offset = low, vrange = high - low;
  const int low = 0, high = 255, offset = low, vrange = high - low;
  // use "-cycle 5" to see all the detail.
  int term = vrange / ncycle;
  for (const auto& yx : range(image.dims())) {
    for_int(z, image.zsize()) image[yx][z] = uint8_t(offset + (image[yx][z] % ncycle) * term);
  }
}

void do_transf(Args& args) {
  Frame frame = FrameIO::parse_frame(args.get_string());
  parallel_for_coords(
      image.dims(),
      [&](const Vec2<int>& yx) {
        Point p{};
        for_int(z, image.zsize()) p[z] = image[yx][z] / 255.f;
        p *= frame;
        for_int(z, image.zsize()) image[yx][z] = uint8_t(clamp(p[z], 0.f, 1.f) * 255.f + .5f);
      },
      30);
}

void do_composite(Args& args) {
  string opname = args.get_string();
  float weight = args.get_float();
  string bfilename = args.get_filename();  // background image
  Array<string> ops;
#define E(x)    \
  ops.push(#x); \
  const int Op_##x = ops.num() - 1
  E(blend);  // weight
  E(special);
  E(shlomo);  // weight
  E(min);
  E(max);
  E(sub);
  E(unblend);
#undef E
  int op = ops.index(opname);
  if (op < 0) assertnever("composite operation '" + opname + "' unrecognized");
  Image bimage(bfilename);
  assertx(same_size(bimage, image));
  assertx(bimage.zsize() == image.zsize());
  for (const auto& yx : range(image.dims())) {
    for_int(z, image.zsize()) {
      float vf = image[yx][z] / 255.f;
      float vb = bimage[yx][z] / 255.f;
      float vr;
      // cannot use "switch (op)" because Op_* are not compile-time constants.
      if (0) {
      } else if (op == Op_blend) {
        vr = vf * weight + vb * (1 - weight);
      } else if (op == Op_special) {
        int fred = image[yx][2] < 200;
        int bblue = bimage[yx][0] < 50;
        vr = fred && !bblue ? vf : vb;
      } else if (op == Op_shlomo) {
        // Shlomo formula:
        // f * lambda - f * lambda * m + b + lambda * m - lambda * b;
        //  m is mask from blue channel f_blue
        float lambda = weight;
        float vm = image[yx][2] / 255.f;
        vr = vf * lambda - vf * lambda * vm + vb + lambda * vm - lambda * vb;
      } else if (op == Op_min) {
        vr = min(vf, vb);
      } else if (op == Op_max) {
        vr = max(vf, vb);
      } else if (op == Op_sub) {
        vr = clamp(vf - vb, 0.f, 1.f);
      } else if (op == Op_unblend) {
        vr = max(vf, vb);
      } else {
        assertnever("");
      }
      image[yx][z] = uint8_t(clamp(vr, 0.f, 1.f) * 255.f + .5f);
    }
  }
}

void do_genpattern(Args& args) {
  string pattern = args.get_string();
  for (const auto& yx : range(image.dims())) {
    const int y = yx[0], x = yx[1];
    const char* p = pattern.c_str();  // inefficient re-parsing but OK.
    float v;
    if (*p == 'B') {  // examine bilinear reconstruction for PL image
      p++;
      float xf = float(x) / (image.xsize() - 1.f);
      float yf = float(y) / (image.ysize() - 1.f);
      switch (*p++) {
        case 'd': {  // diagonal, black at (0, 0), (1, 1)
          const float vA = 0.f, vB = 1.f, vC = 0.f, vD = 1.f;
          v = (1 - yf) * ((1 - xf) * vA + xf * vB) + (yf) * ((1 - xf) * vD + xf * vC);
          break;
        }
        case '3': {  // only one white corner
          const float vA = 1.f, vB = 0.f, vC = 0.f, vD = 0.f;
          v = (1 - yf) * ((1 - xf) * vA + xf * vB) + (yf) * ((1 - xf) * vD + xf * vC);
          break;
        }
        default: assertnever("");
      }
    } else {
      float r;
      switch (*p++) {
        case 'x': r = float(x) / (image.xsize() - 1.f); break;
        case 'y': r = float(y) / (image.ysize() - 1.f); break;
        case 'd':  // diagonal
          r = float(x + y) / (image.xsize() - 1.f + image.ysize() - 1.f);
          break;
        case '2': {  // 20-degree diagonal
          const float ang = 20.f * (TAU / 360);
          auto vrot = V(std::cos(ang), std::sin(ang));
          r = dot(V(y, x), vrot) / dot(convert<float>(image.dims() - 1), vrot);
          break;
        }
        case 'r': {                                                   // radius
          float xf = (float(x) / (image.xsize() - 1.f)) * 2.f - 1.f;  // -1 .. 1
          float yf = (float(y) / (image.ysize() - 1.f)) * 2.f - 1.f;
          r = min(sqrt(xf * xf + yf * yf), 1.f);
          break;
        }
        default: assertnever("error parsing genpattern name");
      }
      while (*p == 'h') {  // hat
        p++;
        r = r <= .5f ? r * 2.f : 1.f - (r - .5f) * 2.f;
      }
      switch (*p++) {
        case 's':  // step
          v = r < .5f ? 0.f : 1.f;
          break;
        case 'l':  // linear
          v = r;
          break;
        case 'q':  // some quadratic
          v = square(r);
          break;
        case 'c':  // some cubic
          // cubic is symmetric and has zero 1st derivatives at 0 and 1
          v = -2.f * r * r * r + 3.f * r * r;
          break;
        default: assertnever("error parsing genpattern name2");
      }
    }
    assertw(v >= 0.f && v <= 1.f);
    for_int(z, image.zsize()) image[yx][z] = uint8_t(clamp(v, 0.f, 1.f) * 255.f + .5f);
  }
}

// Project out the n * n - 1 low-frequency modes of an image (defined as tensor-product cosine basis) (ignore DC term).
// It works for a cropped image too, defined using the image alpha channel.
//  The difficulty is that the cropped basis is not orthogonal or normalized.
//  This is handled by a clever low-memory orthogonal projection.
void do_homogenize(Args& args) {
  HH_TIMER("_homogenize");
  // e.g.:  Filterimage ~/data/image/lake.png -scalen 1 .5 -homogenize 4 | imgv
  //        Filterimage ~/prevproj/2010/spherestitch/Other/mattu_lowfreq/test2.png -homogenize 4 | imgv
  int n = args.get_int();
  assertx(n >= 1 || n == -1);
  bool bilinear = false;
  if (n == -1) {
    bilinear = true;
    n = 2;
  }
  // Using linear (n == -1) rather than cos[0 .. TAU / 2] (n == 1) makes little difference.
  // Only invbb code below would correctly account for the fact that linear is not orthogonal to other cos modes.
  assertx(image.size());
  Vec2<Matrix<double>> table;
  for_int(c, 2) {
    table[c].init(V(n, image.dim(c)));
    for_int(k, n) for_int(x, image.dim(c)) {
      table[c][k][x] = (k == 0                 ? 1.0
                        : (k == 1 && bilinear) ? (x + 0.5 - image.dim(c) / 2.)
                                               : std::cos((x + 0.5) * k * (D_TAU / 2) / image.dim(c)));
    }
  }
  if (image.zsize() < 4) {  // version without alpha-channel cropping
    for_int(c, 2) for_int(k, n) normalize(table[c][k]);
    // The rms of 1, where x is in the range [0, 1], is 1 (whereas sdv is 0).
    // The rms of cos(k * x * TAU / 2) is 1 / sqrt(2) if k > 0,   or 1 if k == 0.
    // The rms of x is 1 / sqrt(3)
    // Mathematica: Sqrt[Integrate[Cos[k * x * Pi]^2, {x, 0, 1}, Assumptions -> {Integer[k]}]]
    for_int(z, image.zsize()) {
      Matrix<double> ar(V(n, n), 0.);
      // TODO: parallelize by allocating Matrix ar per-thread, and then summing them.
      for (const auto& yx : range(image.dims())) {
        for_int(ky, n) for_int(kx, n) ar[ky][kx] += table[0][ky][yx[0]] * table[1][kx][yx[1]] * image[yx][z];
      }
      ar[0][0] = 0.0;  // do not project out the DC term
      parallel_for_coords(
          image.dims(),
          [&](const Vec2<int>& yx) {
            double oldval = image[yx][z];
            double fitval = 0.;
            for_int(ky, n) for_int(kx, n) fitval += ar[ky][kx] * table[0][ky][yx[0]] * table[1][kx][yx[1]];
            double newval = oldval - fitval;
            image[yx][z] = clamp_to_uint8(int(newval + .5));
          },
          n * n * 4);
    }
  } else {  // version with alpha-channel cropping
    const bool modify_unselected_too = getenv_bool("MODIFY_UNSELECTED_TOO");
    for (const auto& yx : range(image.dims())) {
      assertx(image[yx][3] == 0 || image[yx][3] == 255);  // no fractional alpha values
    }
    // Derivation:
    //  We have  v = B * x  where rows of B are non-orthogonal,  bb = B * B^T
    //  Assume B^T = Q * R,  where columns of Q are orthogonal.
    //   then B = R^T * Q^T, and B * B^T = R^T * R (since Q^T * Q = I).
    //  Therefore we can obtain R as Cholesky factor of B * B^T (i.e. bb).
    //  Since B * x = R^T * Q^T * x, then R^-T * B * x = Q^T * x .
    // However, what we really want is the following.
    //  We want x' = x - Q * Q^T * x
    //             = x - B^T * R^-1 * R^-T * B * x
    //             = x - B^T * (R^T * R)^-1 * B * x
    //             = x - B^T * (B * B^T)^-1 * B * x
    //              (which does not simplify because B is non-square, like (A^T * A)^-1 * A^T * b.
    // In fact, see https://en.wikipedia.org/wiki/Projection_operator (orthogonal projection).
    // My key insight is that we can perform orthogonal projection without ever forming the matrix A!
    // So memory complexity is O(m + n * n), where m is the size of the image x and n is the number of basis.
    const int n2 = n * n;
    // Compute the inner products of the cropped basis functions.
    Matrix<double> bb(V(n2, n2), 0.);  // B * B^T
    {
      // TODO: parallelize by allocating bb (and arw) per-thread, and then summing the bb.
      Array<double> arw(n2);  // column of matrix B
      for (const auto& yx : range(image.dims())) {
        if (!image[yx][3]) continue;
        for_int(ky, n) for_int(kx, n) arw[ky * n + kx] = table[0][ky][yx[0]] * table[1][kx][yx[1]];
        for_int(i, n2) for_int(j, i + 1) bb[i][j] += arw[i] * arw[j];  // sum into lower triangular matrix
      }
    }
    for_int(i, n2) for_intL(j, i + 1, n2) bb[i][j] = bb[j][i];  // complete the symmetric matrix
    Matrix<double> invbb = inverse(bb);                         // (B * B^T)^-1
    for_int(z, image.zsize()) {
      Array<double> bx(n2, 0.);  // B * x
      // TODO: parallelize by allocating bx per-thread, and then summing them.
      for (const auto& yx : range(image.dims())) {
        if (!image[yx][3]) continue;
        double v = image[yx][z];
        for_int(ky, n) for_int(kx, n) bx[ky * n + kx] += table[0][ky][yx[0]] * table[1][kx][yx[1]] * v;
      }
      Array<double> arn = mat_mul(invbb, bx);  // (B * B^T)^-1 * B * x
      double dc = bx[0] / bb[0][0];            // DC term
      parallel_for_coords(
          image.dims(),
          [&](const Vec2<int>& yx) {
            if (!image[yx][3] && !modify_unselected_too) return;
            double v = image[yx][z];
            for_int(ky, n) for_int(kx, n) v -= table[0][ky][yx[0]] * table[1][kx][yx[1]] * arn[ky * n + kx];
            v += dc;  // reintroduce DC term
            image[yx][z] = clamp_to_uint8(int(v + .5));
          },
          n * n * 3);
    }
  }
}

inline Vector4 to_YIQ(const Vector4& pix) {
  return Vector4(dot(pix, Vector4(+0.299f, +0.587f, +0.114f, 0.f)), dot(pix, Vector4(+0.596f, -0.274f, -0.322f, 0.f)),
                 dot(pix, Vector4(+0.211f, -0.523f, +0.312f, 0.f)), 255.f);
}

inline Vector4 to_RGB(const Vector4& yiq) {
  return Vector4(dot(yiq, Vector4(+1.000f, +0.956f, +0.621f, 0.f)), dot(yiq, Vector4(+1.000f, -0.272f, -0.647f, 0.f)),
                 dot(yiq, Vector4(+1.000f, -1.106f, +1.703f, 0.f)), 255.f);
}

void do_superresolution(Args& args) {
  // Filterimage ~/data/image/misc/test1.png -superreso 4 | imgv
  float fac = args.get_float();
  assertx(fac > 1.f && fac <= 32.f);
  assertx(image.zsize() == 3);  // 3-channel RGB image
  const auto fb_bilinear2 = twice(FilterBnd(Filter::get("triangle"), Bndrule::reflected));
  // Precompute pixel luminance values (in range [0.f, 255.f]).
  Matrix<float> mlum(image.dims());
  for (const auto& yx : range(image.dims())) mlum[yx] = to_YIQ(to_Vector4_raw(image[yx]))[0];
  // Apply ordinary magnification (using any chosen scaling filter).
  Image oimage = std::move(image);
  image = scale(oimage, twice(fac), g_filterbs, &gcolor);
  // Now add super-res adaptive sharpening.
  parallel_for_coords(
      image.dims(),
      [&](const Vec2<int>& yx) {
        auto fyx = (convert<float>(yx) + .5f) * convert<float>(oimage.dims()) / convert<float>(image.dims()) - .5f;
        // Get regularly magnified pixel.
        Vector4 pix = to_Vector4_raw(image[yx]);
        // Find continuous neighborhood min/max statistics on luminance.
        float lmin = BIGFLOAT, lmax = -BIGFLOAT;
        Vec2<int> iyx = convert<int>(floor(fyx));
        // Get target sample itself.
        float slum = to_YIQ(pix)[0];  // for bilinear, =bilinear(mlum, fy, fx);
        auto func_minmax = [&](float v) {
          if (v < lmin) lmin = v;
          if (v > lmax) lmax = v;
        };
        func_minmax(slum);
        // The next calls are highly unoptimized!  Ideally, the bounds checking should happen only once,
        //  and source samples should be re-used between calls.
        // 4 exact nearest pixels
        func_minmax(mlum.inside(iyx + V(0, 0), k_reflected2));
        func_minmax(mlum.inside(iyx + V(0, 1), k_reflected2));
        func_minmax(mlum.inside(iyx + V(1, 0), k_reflected2));
        func_minmax(mlum.inside(iyx + V(1, 1), k_reflected2));
        // 4 bilinear samples
        func_minmax(sample_grid(mlum, fyx + V(-1.f, -1.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(-1.f, +1.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(+1.f, -1.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(+1.f, +1.f), fb_bilinear2));
        // 8 linear samples (naively use expensive bilinear function)
        func_minmax(sample_grid(mlum, fyx + V(-1.f, +0.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(-1.f, +1.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(+1.f, +0.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(+1.f, +1.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(+0.f, -1.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(+1.f, -1.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(+0.f, +1.f), fb_bilinear2));
        func_minmax(sample_grid(mlum, fyx + V(+1.f, +1.f), fb_bilinear2));
        float delta = lmax - lmin;
        const float threshold = .1f * 255.f;  // 1e-20f is sharpest
        if (delta >= threshold) {
          float lmid = (lmin + lmax) * .5f;
          const float C = 1.0f * 4.f;
          Vector4 yiq = to_YIQ(pix);
          yiq[0] += C * (slum - lmin) * (lmax - slum) * (slum - lmid) / (delta * delta);
          pix = general_clamp(to_RGB(yiq), Vector4(0.f), Vector4(255.99f));
          image[yx] = pix.raw_pixel();
        }
      },
      1000);
}

void do_istoroidal() {
  // Note: only considers C0 continuity rather than higher-order.
  assertx(min(image.dims()) >= 6);
  const float e = 2.f;
  Vec2<float> zaconsec, zarandom;
  for_int(dir, 2) {
    CMatrixView<Pixel> omatrix = image;
    Matrix<Pixel> matrix;
    if (dir == 0) {  // Rows
      matrix = omatrix;
    } else {  // Columns
      matrix = transpose(omatrix);
    }
    Stat stat_int, stat_bnd, stat_rnd;
    auto func_diff_rows = [&](int i1, int i0) {
      double diff = 0.;
      for_int(ix, matrix.xsize()) {
        float err2 = 0.f;
        for_int(z, image.zsize()) {
          uint8_t v1 = matrix[i1][ix][z];
          uint8_t v0a = matrix[i0][ix][z];
          uint8_t v0b = matrix[i0][clamp(ix - 1, 0, matrix.xsize() - 1)][z];
          uint8_t v0c = matrix[i0][clamp(ix + 1, 0, matrix.xsize() - 1)][z];
          uint8_t v0min = min({v0a, v0b, v0c});
          uint8_t v0max = max({v0a, v0b, v0c});
          int d = v1 < v0min ? v0min - v1 : v1 > v0max ? v1 - v0max : 0;
          err2 += square(d);
        }
        float err = sqrt(err2);
        diff += pow(err, e);
      }
      diff = pow(diff / matrix.xsize(), 1. / e);
      return float(diff);
    };
    for_int(i0, matrix.ysize()) {
      int i1 = (i0 + 1) % matrix.ysize();
      float diff = func_diff_rows(i1, i0);
      if (i1 != 0)
        stat_int.enter(diff);
      else
        stat_bnd.enter(diff);
    }
    for_intL(i1, 1, matrix.ysize() - 2) {  // compare last row against all but first, penultimate, or last
      int i0 = matrix.ysize() - 1;
      float diff = func_diff_rows(i1, i0);
      stat_rnd.enter(diff);
    }
    if (0) SHOW(stat_int, stat_bnd, stat_rnd);
    zaconsec[dir] = (stat_bnd.avg() - stat_int.avg()) / stat_int.sdv();
    zarandom[dir] = (stat_bnd.avg() - stat_rnd.avg()) / stat_rnd.sdv();
  }
  float zconsec = max(zaconsec[0], zaconsec[1]);
  float zrandom = max(zarandom[0], zarandom[1]);
  dummy_use(zrandom);
  if (0) showf("zconsec=%7.3f    zrandom=%7.3f\n", zconsec, zrandom);
  if (1) {
    std::cout << sform("%.2f\n", zconsec);
    std::cout.flush();
  }
  nooutput = true;
}

void do_gdtoroidal() {
  // e.g.: Filterimage ~/git/hh_src/test/multigrid/rampart256.png -gdtoroidal -tile 2 2 | imgv
  assertx(image.zsize() == 3);
  Grid<2, Vector4> grid_orig(image.dims());
  parallel_for_coords(
      image.dims(), [&](const Vec2<int>& yx) { grid_orig[yx] = Vector4(image[yx]); }, 6);
  Multigrid<2, Vector4, MultigridPeriodicAll<2>> multigrid(image.dims());
  if (0) {  // solve for color values
    multigrid.initial_estimate().assign(grid_orig);
    // multigrid.set_original(grid_orig);
    multigrid.set_desired_mean(mean(grid_orig));
    // To create a toroidal image, we compute the image Laplacian using reflected boundary conditions
    //  and find the image whose periodic-boundary Laplacian matches it.
    const Vec2<Bndrule> bndrules = twice(Bndrule::reflected);
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          Vector4 vrhs{};
          vrhs += grid_orig.inside(yx + V(-1, 0), bndrules) - grid_orig[yx];
          vrhs += grid_orig.inside(yx + V(+1, 0), bndrules) - grid_orig[yx];
          vrhs += grid_orig.inside(yx + V(0, -1), bndrules) - grid_orig[yx];
          vrhs += grid_orig.inside(yx + V(0, +1), bndrules) - grid_orig[yx];
          multigrid.rhs()[yx] = vrhs;
        },
        50);
    if (0) multigrid.set_verbose(true);
    multigrid.solve();
    auto grid_result = multigrid.result();
    parallel_for_coords(
        image.dims(), [&](const Vec2<int>& yx) { image[yx] = grid_result[yx].pixel(); }, 6);
  } else {  // instead solve for color value offsets
    fill(multigrid.initial_estimate(), Vector4(0.f));
    multigrid.set_desired_mean(Vector4(0.f));
    // To create a toroidal image, we compute the sparse change in Laplacian across the periodic boundaries.
    const Vec2<Bndrule> bndrules = twice(Bndrule::periodic);
    fill(multigrid.rhs(), Vector4(0.f));
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {  // (highly inefficient implementation)
          Vector4 vrhs{};
          for (auto yxd : {V(-1, 0), V(+1, 0), V(0, -1), V(0, +1)}) {
            auto yxn = yx + yxd;
            if (grid_orig.ok(yxn)) continue;  // only consider neighbors across the periodic boundaries
            assertx(grid_orig.map_inside(yxn, bndrules));
            vrhs += (grid_orig[yx] - grid_orig[yxn]);
          }
          multigrid.rhs()[yx] = vrhs;
        },
        50);
    if (0) multigrid.set_verbose(true);
    multigrid.solve();
    auto grid_result = multigrid.result();
    parallel_for_coords(
        image.dims(), [&](const Vec2<int>& yx) { image[yx] = (grid_orig[yx] + grid_result[yx]).pixel(); }, 6);
  }
}

void do_gradientsharpen(Args& args) {
  // e.g.: Filterimage ~/git/hh_src/test/multigrid/rampart256.png -gradientsharpen 1.5 | imgv
  float gradient_sharpening = args.get_float();
  float screening_weight = 1.f;
  assertx(image.zsize() == 3);
  Grid<2, Vector4> grid_orig(image.dims());
  parallel_for_coords(
      image.dims(), [&](const Vec2<int>& yx) { grid_orig[yx] = Vector4(image[yx]); }, 6);
  Multigrid<2, Vector4> multigrid(image.dims());
  multigrid.initial_estimate().assign(grid_orig);
  multigrid.set_desired_mean(mean(grid_orig));
  parallel_for_coords(
      image.dims(),
      [&](const Vec2<int>& yx) {
        const int ny = grid_orig.dim(0), nx = grid_orig.dim(1);
        Vector4 vrhs = -screening_weight * grid_orig[yx];
        if (yx[0] > 0) vrhs += (grid_orig[yx + V(-1, 0)] - grid_orig[yx]) * gradient_sharpening;
        if (yx[0] < ny - 1) vrhs += (grid_orig[yx + V(+1, 0)] - grid_orig[yx]) * gradient_sharpening;
        if (yx[1] > 0) vrhs += (grid_orig[yx + V(0, -1)] - grid_orig[yx]) * gradient_sharpening;
        if (yx[1] < nx - 1) vrhs += (grid_orig[yx + V(0, +1)] - grid_orig[yx]) * gradient_sharpening;
        multigrid.rhs()[yx] = vrhs;
      },
      30);
  if (0) multigrid.set_verbose(true);
  multigrid.set_screening_weight(screening_weight);
  multigrid.solve();
  auto grid_result = multigrid.result();
  parallel_for_coords(
      image.dims(), [&](const Vec2<int>& yx) { image[yx] = grid_result[yx].pixel(); }, 6);
}

void do_gdfill() {
  // e.g.: Filterimage ~/data/image/lake.png -setalpha 255 -color 0 0 0 0 -drawrect 30% 30% -30% -30% -gdfill | imgv
  // also:  Filterimage ~/data/image/misc/lake.masked.png -gdfill | imgv  (see ~/proj/skype/Notes.txt)
  HH_TIMER("_gdfill");
  float screening_weight = 1e-5f;  // 0.f is fine too; 1e-4f has visible difference
  assertx(image.zsize() == 4);
  auto masked = [&](const Vec2<int>& yx) { return image[yx][3] < 255; };
  Vector4 vmean{};
  {
    for_coords(image.dims(), [&](const Vec2<int>& yx) {  // sequential due to reduction
      if (!masked(yx)) vmean += Vector4(image[yx]);
    });
    vmean /= vmean[3];  // SHOW(vmean);
  }
  Grid<2, Vector4> grid_orig(image.dims());
  parallel_for_coords(
      image.dims(), [&](const Vec2<int>& yx) { grid_orig[yx] = masked(yx) ? vmean : Vector4(image[yx]); }, 10);
  Multigrid<2, Vector4> multigrid(image.dims());
  if (0) {  // solve for color values
    multigrid.initial_estimate().assign(grid_orig);
    multigrid.set_desired_mean(vmean);
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          const int ny = grid_orig.dim(0), nx = grid_orig.dim(1);
          Vector4 vrhs = -screening_weight * grid_orig[yx];
          if (!masked(yx)) {
            if (yx[0] > 0) {
              auto p = yx + V(-1, 0);
              if (!masked(p)) vrhs += (grid_orig[p] - grid_orig[yx]);
            }
            if (yx[0] < ny - 1) {
              auto p = yx + V(+1, 0);
              if (!masked(p)) vrhs += (grid_orig[p] - grid_orig[yx]);
            }
            if (yx[1] > 0) {
              auto p = yx + V(0, -1);
              if (!masked(p)) vrhs += (grid_orig[p] - grid_orig[yx]);
            }
            if (yx[1] < nx - 1) {
              auto p = yx + V(0, +1);
              if (!masked(p)) vrhs += (grid_orig[p] - grid_orig[yx]);
            }
          }
          multigrid.rhs()[yx] = vrhs;
        },
        50);
    if (1) multigrid.set_verbose(true);
    multigrid.set_screening_weight(screening_weight);
    multigrid.set_num_vcycles(1);  // default 5, then was 2
    multigrid.solve();
    auto grid_result = multigrid.result();
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          image[yx] = grid_result[yx].pixel();  // alpha mask is overwritten with 1
        },
        10);
  } else {  // instead solve for offsets; rhs becomes sparse; so does residual; TODO: perhaps avoid storing residual
    fill(multigrid.initial_estimate(), Vector4(0.f));
    multigrid.set_desired_mean(Vector4(0.f));
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          Vector4 vrhs{};
          for (auto yxd : {V(-1, 0), V(+1, 0), V(0, -1), V(0, +1)}) {
            auto yxn = yx + yxd;
            if (!grid_orig.ok(yxn)) continue;
            if (masked(yx) != masked(yxn)) vrhs += (grid_orig[yx] - grid_orig[yxn]);
          }
          multigrid.rhs()[yx] = vrhs;
        },
        100);
    if (1) multigrid.set_verbose(true);
    multigrid.set_screening_weight(screening_weight);
    multigrid.set_num_vcycles(1);
    multigrid.solve();
    auto grid_result = multigrid.result();
    for_coords(grid_result.dims(), [&](const Vec2<int>& yx) { HH_SSTAT(Sgrid_result0, grid_result[yx][0]); });
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          image[yx] = (grid_orig[yx] + grid_result[yx]).pixel();  // alpha mask is overwritten with 1
        },
        10);
  }
  image.set_zsize(3);  // remove alpha channel
}

void output_contour(int gn, float contour_value) {
  if (!gn) gn = max(image.dims());
  WSA3dStream wcontour(std::cout);
  // Pixels are at locations [ 0.5 / imagesize, (imagesize - .5) / imagesize ], consistent with sample_domain()
  Matrix<float> matrix(image.dims());
  for (const auto& yx : range(image.dims())) matrix[yx] = float(image[yx][0]);  // red channel

  Vec2<FilterBnd> filterbs = g_filterbs;
  if (filterbs[0].filter().has_inv_convolution() || filterbs[1].filter().has_inv_convolution())
    filterbs = inverse_convolution(matrix, filterbs);
  auto func_eval = [&](const Vec2<float>& p) -> float { return sample_domain(matrix, p, filterbs) - contour_value; };
  A3dElem el;
  auto func_contour = [&](CArrayView<Vec2<float>> poly) {
    el.init(A3dElem::EType::polyline);
    for_int(i, poly.num()) {
      Point p(poly[i][1], poly[i][0], 0.f);
      el.push(A3dVertex(p, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red())));
    }
    wcontour.write(el);
  };
  Contour2D contour(gn, func_eval, func_contour);
  contour.set_vertex_tolerance(.0001f);
  for (const auto& yx : range(twice(gn - 1))) {                   // visit all contour cells
    contour.march_from((convert<float>(yx) + 1.f) * (1.f / gn));  // center of contour cell
  }
  nooutput = true;
}

void do_contour(Args& args) {
  // e.g.:  Filterimage ~/data/image/lake.png -contour 256 | G3d -st imageup
  // e.g.:  CONTOUR_VERTEX_TOL=0 Filterimage ~/data/image/lake.png -contour 8 | G3d -st imageup
  // e.g.:  Filterimage ~/data/image/lake.png -scaletox 16 -contour 64 | Filtera3d -joinlines | G3d -st imageup
  int gn = args.get_int();
  output_contour(gn, 127.5f);
}

void do_mcontours(Args& args) {
  // e.g.: Filterimage ~/data/image/lake.png -scaletox 32 -mcontours 256 10 | Filtera3d -joinlines | G3d -st imageup
  int gn = args.get_int();
  int ncontours = args.get_int();
  for_int(i, ncontours) {
    // float contour_value = 255.f * (i + .5f) / ncontours;  // contours lie at center of uniform value intervals
    float contour_value = 255.f * i / (ncontours + 1.f);  // contours delineate uniform partition of pixel values
    output_contour(gn, contour_value);
  }
}

void do_niter(Args& args) { g_niter = args.get_int(); }

void do_poisson() {
  bool conformal = wconformal > 0.f;
  const bool conf_L = true;
  // Incorporate both gradient-matching and conformality (LSCM) constraints.
  assertx(min(image.dims()) >= 2);
  const int nconstraints = !fixedbnd ? 2 : image.xsize() * 2 + image.ysize() * 2;
  auto func_default_pos = [&](const Vec2<int>& yx) {
    int maxn = max(image.dims());
    return Point(((maxn - image.dim(1)) / 2.f + yx[1]) / (maxn - 1.f),
                 ((maxn - image.dim(0)) / 2.f + yx[0]) / (maxn - 1.f), 0.f);
  };
  float vmean = 0.f;
  for (const auto& yx : range(image.dims())) vmean += float(image[yx][0]);
  vmean /= float(image.size());
  Matrix<Point> matp(image.dims());
  for (const auto& yx : range(image.dims())) matp[yx] = func_default_pos(yx);
  Matrix<float> matw(image.dims() - 1, 1.f);
  for_int(iter, g_niter) {
    // Solve a sparse linear-least-squares system;
    //  parameters are number of rows (constraints), number of unknowns, and dimensionality of unknowns.
    Timer timer("_lls");
    const int ny = image.ysize(), nx = image.xsize();
    SparseLls lls(
        2 * (ny * (nx - 1) + nx * (ny - 1)) + conformal * 2 * (ny - 2 + conf_L) * (nx - 2 + conf_L) + nconstraints,
        ny * nx * 2, 1);
    lls.set_verbose(0);
    // lls.set_tolerance(1e-12f);  // default 1e-10f
    int row = 0;
    // Approach related to [Praun et al 2000] (Lapped Textures):
    //  Create warped parameterization by aligning local frame.
    // Discretization similar to that in [Perez et al 2003] (Poisson Image Editing):
    //  For each edge between two samples, desire difference of samples to
    //  equal the component of the gradient in that edge direction, where
    //  the gradient is taken to be the mean of gradients at the two samples.
    {
      const float scale = 1.f / vmean / max(nx, ny) * gscale;
      const float smallw = 0.f;  // also used 1e-3f
      for_int(x, nx) for_int(y, ny - 1) {
        // Desire matx[y + 1][x] - matx[yx] == { (image[yx][1] + image[y + 1][x][1]) / 2 , 0 }
        float w = 1.f;
        float v = (image[y + 0][x][0] + image[y + 1][x][0]) / 2.f * scale;
        if (!v) {
          Warning("Interpreting zero input differences as unconstrained differences");
          w = smallw;
        }
        lls.enter_a_rc(row, ((y + 0) * nx + (x)) * 2 + 0, -w);
        lls.enter_a_rc(row, ((y + 1) * nx + (x)) * 2 + 0, +w);
        lls.enter_b_rc(row, 0, 0.f);
        row++;
        lls.enter_a_rc(row, ((y + 0) * nx + (x)) * 2 + 1, -w);
        lls.enter_a_rc(row, ((y + 1) * nx + (x)) * 2 + 1, +w);
        lls.enter_b_rc(row, 0, w * v);
        row++;
      }
      for_int(y, ny) for_int(x, nx - 1) {
        // Desire matx[y][x + 1] - matx[yx] == { (image[yx][0] + image[y][x + 1][0]) / 2 , 0 }
        float w = 1.f;
        float v = (image[y][x + 0][0] + image[y][x + 1][0]) / 2.f * scale;
        if (!v) {
          Warning("Interpreting zero input differences as unconstrained differences");
          w = smallw;
        }
        lls.enter_a_rc(row, (y * nx + (x + 0)) * 2 + 0, -w);
        lls.enter_a_rc(row, (y * nx + (x + 1)) * 2 + 0, +w);
        lls.enter_b_rc(row, 0, w * v);
        row++;
        lls.enter_a_rc(row, (y * nx + (x + 0)) * 2 + 1, -w);
        lls.enter_a_rc(row, (y * nx + (x + 1)) * 2 + 1, +w);
        lls.enter_b_rc(row, 0, 0.f);
        row++;
      }
    }
    if (conformal) {
      if (conf_L) {
        // L-shape: penalize wconformal * mag2(rot90(mat[y][x + 1] - mat[yx]) - (mat[y + 1][x] - mat[yx]))
        float sqrtw = my_sqrt(wconformal);
        if (iter > 0) {
          Matrix<float> matconf(ny - 1, nx - 1);
          double a = 0.;
          HH_STAT(stat);
          for (const auto& yx : range(image.dims() - 1)) {
            matconf[yx] = (square(matp[yx + V(0, 1)][0] - matp[yx][0] + matp[yx][1] - matp[yx + V(1, 0)][1]) +
                           square(matp[yx + V(0, 1)][1] - matp[yx][1] - matp[yx][0] + matp[yx + V(1, 0)][0]));
            a += std::log(matconf[yx]);
            stat.enter(matconf[yx]);
          }
          float gmean = std::exp(float(a) / ny / nx);
          SHOW(gmean);
          gmean = stat.avg();  // !
          for (const auto& yx : range(image.dims() - 1)) {
            float mag2 = (square(matp[yx + V(0, 1)][0] - matp[yx][0] + matp[yx][1] - matp[yx + V(1, 0)][1]) +
                          square(matp[yx + V(0, 1)][1] - matp[yx][1] - matp[yx][0] + matp[yx + V(1, 0)][0]));
            if (mag2 > gmean) matw[yx] *= pow(mag2 / gmean, .25f);
          }
        }
        for_int(y, ny - 1) for_int(x, nx - 1) {
          float lw = matw[y][x] * sqrtw;
          lls.enter_a_rc(row, ((y + 0) * nx + (x + 0)) * 2 + 0, -lw);
          lls.enter_a_rc(row, ((y + 0) * nx + (x + 1)) * 2 + 0, +lw);
          lls.enter_a_rc(row, ((y + 1) * nx + (x + 0)) * 2 + 1, -lw);
          lls.enter_a_rc(row, ((y + 0) * nx + (x + 0)) * 2 + 1, +lw);
          lls.enter_b_rc(row, 0, 0.f);
          row++;
          lls.enter_a_rc(row, ((y + 0) * nx + (x + 0)) * 2 + 1, -lw);
          lls.enter_a_rc(row, ((y + 0) * nx + (x + 1)) * 2 + 1, +lw);
          lls.enter_a_rc(row, ((y + 1) * nx + (x + 0)) * 2 + 0, +lw);
          lls.enter_a_rc(row, ((y + 0) * nx + (x + 0)) * 2 + 0, -lw);
          lls.enter_b_rc(row, 0, 0.f);
          row++;
        }
      } else {
        // +-shape: penalize wconformal * mag2(rot90(mat[y][x + 1] - mat[y][x - 1]) - (mat[y + 1][x] - mat[y - 1][x]))
        // compared to L-shape, this looks no better, or sometimes ever slightly worse.
        float sqrtw = my_sqrt(wconformal) * 0.5f;
        for_intL(y, 1, ny - 1) for_intL(x, 1, nx - 1) {
          lls.enter_a_rc(row, ((y + 0) * nx + (x - 1)) * 2 + 0, -sqrtw);
          lls.enter_a_rc(row, ((y + 0) * nx + (x + 1)) * 2 + 0, +sqrtw);
          lls.enter_a_rc(row, ((y + 1) * nx + (x + 0)) * 2 + 1, -sqrtw);
          lls.enter_a_rc(row, ((y - 1) * nx + (x + 0)) * 2 + 1, +sqrtw);
          lls.enter_b_rc(row, 0, 0.f);
          row++;
          lls.enter_a_rc(row, ((y + 0) * nx + (x - 1)) * 2 + 1, -sqrtw);
          lls.enter_a_rc(row, ((y + 0) * nx + (x + 1)) * 2 + 1, +sqrtw);
          lls.enter_a_rc(row, ((y + 1) * nx + (x + 0)) * 2 + 0, +sqrtw);
          lls.enter_a_rc(row, ((y - 1) * nx + (x + 0)) * 2 + 0, -sqrtw);
          lls.enter_b_rc(row, 0, 0.f);
          row++;
        }
      }
    }
    if (nconstraints == 2) {
      // Fix middle point at (.5f, .5f)
      lls.enter_a_rc(row, ((ny / 2) * nx + (nx / 2)) * 2 + 0, 1.f);
      lls.enter_b_rc(row, 0, .5f);
      row++;
      lls.enter_a_rc(row, ((ny / 2) * nx + (nx / 2)) * 2 + 1, 1.f);
      lls.enter_b_rc(row, 0, .5f);
      row++;
    } else if (nconstraints == nx * 2 + ny * 2) {
      const float bigf = 1e5f;
      for_int(y, 2) for_int(x, nx) {
        int nym1 = y ? ny - 1 : 0;
        lls.enter_a_rc(row, (nym1 * nx + (x)) * 2 + 1, bigf);
        Point p = func_default_pos(V(nym1, x));
        lls.enter_b_rc(row, 0, bigf * p[1]);
        row++;
      }
      for_int(x, 2) for_int(y, ny) {
        int nxm1 = x ? nx - 1 : 0;
        lls.enter_a_rc(row, (y * nx + nxm1) * 2 + 0, bigf);
        Point p = func_default_pos(V(y, nxm1));
        lls.enter_b_rc(row, 0, bigf * p[0]);
        row++;
      }
    } else {
      assertnever("");
    }
    assertx(row == lls.num_rows());
    for_int(y, ny) for_int(x, nx) {
      lls.enter_xest_rc((y * nx + x) * 2 + 0, 0, matp[y][x][0]);
      lls.enter_xest_rc((y * nx + x) * 2 + 1, 0, matp[y][x][1]);
    }
    assertx(lls.solve());
    timer.terminate();  // "_lls"
    for_int(y, ny) for_int(x, nx) {
      matp[y][x][0] = lls.get_x_rc((y * nx + x) * 2 + 0, 0);
      matp[y][x][1] = lls.get_x_rc((y * nx + x) * 2 + 1, 0);
    }
  }
  {
    GMesh mesh;
    Matrix<Vertex> matv(image.dims());
    string str;
    for (const auto& yx : range(image.dims())) {
      Vertex v = mesh.create_vertex();
      matv[yx] = v;
      mesh.set_point(v, matp[yx]);
      UV uv((convert<float>(yx) / convert<float>(image.dims() - 1)).rev());
      mesh.update_string(v, "uv", csform_vec(str, uv));
      if (1) {
        mesh.update_string(v, "Ouv", csform_vec(str, uv));
        Point opos = func_default_pos(yx);
        mesh.update_string(v, "Opos", csform_vec(str, opos));
      }
    }
    for (const auto& yx : range(image.dims() - 1)) {
      mesh.create_face(V(matv[yx + V(0, 0)], matv[yx + V(0, 1)], matv[yx + V(1, 1)], matv[yx + V(1, 0)]));
    }
    // WFile fi("mesh.m"); mesh.write(fi());
    hh_clean_up();
    mesh.write(std::cout);
    std::cout.flush();
    nooutput = true;
  }
}

void do_procedure(Args& args) {
  string name = args.get_string();
  if (0) {
  } else if (name == "pqtotoast") {
    assertx(image.size() && image.xsize() == 4 * image.ysize());
    Image timage(image);
    image.init(twice(timage.ysize()));
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          image[yx] = timage[yx[0] < timage.ysize() ? yx : image.dims() - 1 - yx + V(0, timage.xsize() / 2)];
        },
        2);
  } else if (name == "premultiply_alpha") {
    assertx(image.zsize() == 4);
    parallel_for_coords(
        image.dims(),
        [&](const Vec2<int>& yx) {
          auto alpha = image[yx][3];
          for_int(c, 3) image[yx][c] = uint8_t(float(image[yx][c]) * alpha / 255.f + .5f);
        },
        10);
  } else if (name == "interleavecols") {
    Image image1(image);
    Image image2(args.get_filename());
    assertx(same_size(image1, image2));
    for (const auto& yx : range(image.dims())) image[yx] = yx[1] % 2 == 0 ? image1[yx] : image2[yx];
  } else if (name == "stereo_purple") {
    const int n = 800;
    const int nobj = 8;
    const int r = 25;
    const int rg = 180;
    Image image1(V(n, n), Pixel::white());
    Image image2(V(n, n), Pixel::white());
    for (const auto& oyx : range(twice(nobj))) {
      for (const auto& yx : range(oyx * 100 + 50 - r, oyx * 100 + 50 + r)) {
        int kind = (oyx[0] * nobj + oyx[1]) % 3;
        switch (kind) {
          case 0:
            image1[yx] = Pixel(rg, 0, rg);
            image2[yx] = Pixel(rg, 0, rg);
            break;
          case 1:
            image1[yx] = Pixel::red();
            image2[yx] = Pixel::blue();
            break;
          case 2:
            image1[yx] = Pixel::blue();
            image2[yx] = Pixel::red();
            break;
          default: assertnever("");
        }
      }
    }
    Matrix<Image> images(1, 2);
    images[0][0] = image1;
    images[0][1] = image2;
    assemble_images(images);
  } else if (name == "benchmark") {
    HH_TIMER("_overall");
    const float s = 1.01f;
    for_int(i, 10) {
      // HH_TIMER("__scale");
      image.scale(twice(s), g_filterbs, &gcolor);
    }
  } else if (name == "mark_dark") {
    const int thresh = args.get_int();
    if (image.zsize() < 4) {
      assertx(image.zsize() == 3);
      image.set_zsize(4);
    }
    for (const auto& yx : range(image.dims())) {
      int lum = image[yx][0] + image[yx][1] + image[yx][2];
      image[yx][3] = lum > thresh ? 0 : 255;
    }
  } else if (name == "test1") {
    // ~/proj/morph/data/quadmesh/Notes.txt
    // Filterimage image1.png -procedure test1 image2.png >mesh.m
    assertx(min(image.dims()) >= 2);
    string filename2 = args.get_filename();
    Image image2(filename2);
    assertx(same_size(image, image2));
    Matrix<Vector4> image2v(image.dims());
    convert(image2, image2v);
    Vec2<FilterBnd> filterbs = g_filterbs;
    if (filterbs[0].filter().has_inv_convolution() || filterbs[1].filter().has_inv_convolution())
      filterbs = inverse_convolution(image2v, filterbs);
    GMesh mesh;
    Matrix<Vertex> matv(image.dims());
    string str;
    for (const auto& yx : range(image.dims())) {
      Vertex v = mesh.create_vertex();
      matv[yx] = v;
      Point p(yx[1] / (image.dim(1) - 1.f), yx[0] / (image.dim(0) - 1.f), 0.f);
      Point pcenter(0.5f, 0.5f, 0.f);
      Vector vr = p - pcenter;
      float r = mag(vr);
      float a = std::atan2(vr[1], vr[0]);
      // Point np = (p - pcenter) * Frame::rotation(2, to_rad(20.f)) + pcenter;
      a += to_rad(20.f - abs(r - 0.5f) * 20.f);
      r = (.6f + smooth_step(r / .707f) * .5f) * r;
      Point np = pcenter + r * Vector(std::cos(a), std::sin(a), 0.f);
      mesh.set_point(v, np);
      mesh.update_string(v, "Opos", csform_vec(str, p));
      Vector vrgb = Vector(image[yx][0], image[yx][1], image[yx][2]) / 255.f;
      // Vector nvrgb = Vector(image2[yx][0], image2[yx][1], image2[yx][2]) / 255.f;
      Vector4 vec4 = sample_domain(image2v, V(np[1], np[0]), filterbs);
      vec4 = general_clamp(vec4, Vector4(0.f), Vector4(1.f));
      Vector nvrgb(vec4[0], vec4[1], vec4[2]);
      mesh.update_string(v, "rgb", csform_vec(str, nvrgb));
      mesh.update_string(v, "Orgb", csform_vec(str, vrgb));
    }
    for (const auto& yx : range(image.dims() - 1)) {
      mesh.create_face(V(matv[yx], matv[yx + V(0, 1)], matv[yx + V(1, 1)], matv[yx + V(1, 0)]));
    }
    hh_clean_up();
    mesh.write(std::cout);
    std::cout.flush();
    nooutput = true;
  } else if (name == "checkers4") {
    // Filterimage -create 128 128 -proc checkers4 | imgv
    const int n = image.ysize() / 2;
    assertx(image.dims() == twice(n * 2));
    for (const auto& yxi : range(twice(2))) {
      for (const auto& yx : range(twice(n))) {
        const int beat = 8;
        const uint8_t greyv = 60;
        Pixel pix(greyv, greyv, greyv, 255);
        bool is_on = ((yx[1] / beat) + (yx[0] / beat)) % 2 == 1;
        if (is_on) {
          if (0)
            void();
          else if (yxi == V(0, 0))
            pix = Pixel(255, 130, 130);
          else if (yxi == V(0, 1))
            pix = Pixel(40, 255, 40);
          else if (yxi == V(1, 0))
            pix = Pixel(150, 150, 255);
          else if (yxi == V(1, 1))
            pix = Pixel(255, 40, 255);
          else
            assertnever("");
        }
        image[yxi * n + yx] = pix;
      }
    }
  } else if (name == "gradchecker") {
    // Filterimage -create 512 512 -proc gradchecker 5 -to png | imgv
    int gridn = args.get_int();
    assertx(gridn >= 1);
    int size = max(image.dims());
    const Pixel pixel_gray(235, 235, 235, 255);
    for (const auto& yx : range(image.dims())) {
      const Vec2<float> yxf = (convert<float>(yx) + .5f) / float(size);
      const Vec2<int> yxi = convert<int>(yxf * float(gridn));
      const bool is_on = sum(yxi) % 2 == 1;
      Pixel pix = !is_on ? pixel_gray : Pixel(uint8_t(yxf[0] * 255.f + .5f), uint8_t((1.f - yxf[0]) * 255.f + .5f), 0);
      image[yx] = pix;
    }
  } else if (name == "fix_agarwala") {
    // streaming multigrid: fix Aseem Agarwala labels file
    // Filterimage labels.png -proc fix_agarwala >labels.fixed.png
    for (const auto& yx : range(image.dims())) {
      int l = image[yx][0];
      uint8_t v;
      uint8_t a = 255;
      if (l == 255) {
        v = 255;  // v = 0; a = 0;
      } else if (l % 20 == 0) {
        v = uint8_t(l / 20);
      } else if ((l + 256) % 20 == 0) {
        v = uint8_t((l + 256) / 20);
      } else {
        SHOW(l);
        assertnever("");
      }
      for_int(z, 3) image[yx][z] = v;
      image[yx][3] = a;
    }
  } else if (name == "red_green_ramp") {
    // Filterimage -create 512 512 -procedure red_green_ramp -to png >~/data/image/ramp_red_green.png
    for (const auto& yx : range(image.dims())) {
      image[yx].head<2>() = convert<uint8_t>(convert<float>(yx) / (convert<float>(image.dims()) - 1.f) * 255.f + .5f);
      image[yx][2] = 0;
      image[yx][3] = 255;
    }
  } else if (name == "test_videoloop_mask_dilation") {
    // cd ~/proj/videoloops/data/test_bfs_mask_expansion
    // Filterimage TDpoolpalms_temp_opt0_3.png -procedure test_videoloop_mask_dilation
    assertx(image.size() > 0);
    Image image_periods("TDpoolpalms_labels_opt0_3_period.png");  // period0: 128, 128, 255; 480x272
    // Image image_periods("periods.png");  // period0: 0, 0, 0 but 480x270
    assertx(image_periods.dims() == image.dims());
    //
    const uint8_t cost_threshold = 3;  // let the mask consist of the pixels whose temporal costs is >= threshold
    const int radius2_threshold = 4;   // dilation squared radius
    const int radius2_max = 20;        // squared radius of maximum bread-first-search expansion
    const Pixel period_zero(128, 128, 255, 255);
    {
      Image image_initial_mask(image.dims(), Pixel::white());
      parallel_for_coords(
          image.dims(),
          [&](const Vec2<int>& yx) {
            if (image[yx][0] >= cost_threshold) image_initial_mask[yx] = Pixel::black();
          },
          8);
      image_initial_mask.write_file("mask_initial.png");
    }
    if (1) {  // check for pixels that are static (period0) but have nonzero temporal cost
      for (const auto& yx : range(image.dims())) {
        if (image_periods[yx] == period_zero) {
          HH_SSTAT(Sp0tcost, image[yx][0]);
        }
        // # Sp0tcost:           (79313  )          0:255         av=8.1941299      sd=35.532955
      }
    }
    Matrix<Vec2<int>> mvec(image.dims());
    {  // compute vectors to closest pixel in the mask
      const auto undefined_dist = mvec.dims();
      parallel_for_coords(
          image.dims(),
          [&](const Vec2<int>& yx) { mvec[yx] = image[yx][0] >= cost_threshold ? V(0, 0) : undefined_dist; }, 8);
      euclidean_distance_map(mvec);
    }
    {  // dilate the mask by a radius that is the square-root of radius2_threshold
      Image image_dilated(image.dims());
      parallel_for_coords(
          image.dims(),
          [&](const Vec2<int>& yx) {
            bool in_mask = mag2(mvec[yx]) <= radius2_threshold;
            image_dilated[yx] = Pixel::gray(in_mask ? 0 : 255);
          },
          8);
      image_dilated.write_file("mask_dilated.png");
    }
    Matrix<bool> visited(image.dims(), false);
    {  // Grow mask to pixels with the same period.
      Queue<Vec2<int>> queue;
      for_coords(image.dims(), [&](const Vec2<int>& yx) {
        if (image[yx][0] >= cost_threshold) {
          queue.enqueue(yx);
          visited[yx] = true;
        }
      });
      while (!queue.empty()) {
        Vec2<int> yx = queue.dequeue();
        if (1 && image_periods[yx] == period_zero) continue;  // do not grow from static pixels (period0)
        for (const auto& yxd : {V(+1, 0), V(-1, 0), V(0, +1), V(0, -1)}) {
          Vec2<int> yxn = yx + yxd;
          if (!image.ok(yxn)) continue;                      // out-of-bounds
          if (0 && mag2(mvec[yxn]) > radius2_max) continue;  // optional: not expand beyond this squared radius
          if (!visited[yxn] && image_periods[yxn] == image_periods[yx]) {  // compare RGB values of period
            ASSERTX(image_periods[yxn] != period_zero);
            queue.enqueue(yxn);
            visited[yxn] = true;
          }
        }
      }
    }
    {
      Image image_bfs(image.dims());
      parallel_for_coords(
          image.dims(), [&](const Vec2<int>& yx) { image_bfs[yx] = Pixel::gray(visited[yx] ? 0 : 255); }, 8);
      image_bfs.write_file("mask_bfs.png");
    }
    {  // Let the mask be a dilation unioned with the bfs result but limited to a maximum radius
      Image image_final(image.dims());
      parallel_for_coords(
          image.dims(),
          [&](const Vec2<int>& yx) {
            int radius2 = narrow_cast<int>(mag2(mvec[yx]));
            bool in_mask = (radius2 <= radius2_threshold || (visited[yx] && radius2 <= radius2_max));
            image_final[yx] = Pixel::gray(in_mask ? 0 : 255);
          },
          8);
      image_final.write_file("mask_final.png");
    }
    nooutput = true;
  } else if (name == "assemble_thumbnails") {
    // ls $HOMEPATH/Dropbox/Pictures/2014/arboretum/*.jpg | arg Filterimage -create 1 1 -as_cropsides -1 -1 -1 -1 -procedure assemble_thumbnails -stdin 128 -to png >v.png
    // for y in "$(cygpath -u "$HOMEPATH")"/Dropbox/Pictures/2*; do yy="$(tname "$y")"; echo $yy; for i in "$y"/**/*.jpg; do echo "$i"; done | arg Filterimage -create 1 1 -as_cropsides -1 -1 -1 -1 -procedure assemble_thumbnails -stdin 128 -to png >$yy.thumbs.png; done
    // for i in 2*.thumbs.png; do Filterimage $i -scaleu .25 -to jpg >${i/thumbs.png/thumbs2.jpg}; done
    // for i in "$(cygpath -u "$HOMEPATH")"/Dropbox/Pictures/2*/**/*.jpg; do echo "$i"; done | arg Filterimage -create 1 1 -as_cropsides -1 -1 -1 -1 -procedure assemble_thumbnails -stdin 128 -to png >all.thumbs.png
    // i=all.thumbs.png; JPG_QUALITY=10 Filterimage $i -to jpg >${i/thumbs.png/thumbs1.jpg}
    // i=all.thumbs.png; Filterimage $i -scaleu .125 -to jpg >${i/thumbs.png/thumbs3.jpg}
    image.clear();
    string name_image_list = args.get_filename();
    int size = args.get_int();
    Array<string> imagenames;
    {
      RFile fi(name_image_list);
      string str;
      while (my_getline(fi(), str)) {
        if (!file_exists(str)) assertnever("image file '" + str + "' does not exist");
        imagenames.push(str);
      }
      assertx(imagenames.num() >= 1);
    }
    Array<Image> ar_thumbnails(imagenames.num());
    {
      ConsoleProgress cprogress;
      std::atomic<int> count{0};
      parallel_for_each(range(imagenames.num()), [&](const int i) {
        cprogress.update(float(count++) / imagenames.num());
        Image& limage = ar_thumbnails[i];
        limage.set_silent_io_progress(true);
        limage.read_file(imagenames[i]);
        // Downscale to maximum size.
        limage.scale(twice(float(size) / assertx(max(limage.dims()))), g_filterbs, &gcolor);
        // Fill to square.
        Grid<2, Pixel>& grid = limage;
        int vt = -(size - limage.ysize()) / 2, vb = -(size - limage.ysize() + vt);
        int vl = -(size - limage.xsize()) / 2, vr = -(size - limage.xsize() + vl);
        grid = crop(grid, V(vt, vl), V(vb, vr), twice(Bndrule::border), &gcolor);
      });
    }
    Matrix<Image> grid_thumbnails;
    {
      int ncol = int(sqrt(imagenames.num() - 1)) + 1;  // number of columns
      grid_thumbnails.init(V((imagenames.num() - 1) / ncol + 1, ncol));
      int i = 0;
      for_coords(grid_thumbnails.dims(), [&](const Vec2<int>& yx) {
        grid_thumbnails[yx] = (ar_thumbnails.ok(i) ? std::move(ar_thumbnails[i++]) : Image(twice(size), gcolor));
        apply_assemble_operations(grid_thumbnails[yx], yx, grid_thumbnails.dims());
      });
    }
    assemble_images(grid_thumbnails);
  } else if (name == "teddy1") {
    GMesh mesh;
    Matrix<Vertex> matv(image.dims());
    string str;
    image.read_file("test2_depth.png");
    for_coords(image.dims(), [&](const Vec2<int>& yx) {
      Vertex v = mesh.create_vertex();
      matv[yx] = v;
      const Pixel& pixel = image[yx];
      assertx(pixel[0] == 128);
      assertx(pixel[3] == 255);
      int depth = pixel[1] * 256 + pixel[2];  // g * 256 + b
      HH_SSTAT(Sdepth, depth);
      depth = min(depth, 1000);
      Vec2<float> yxf = (convert<float>(yx) + .5f) / max(convert<float>(image.dims()));
      float fac = 5e-4f;
      mesh.set_point(v, Point(yxf[1], yxf[0], depth * fac));
      UV uv(yx[1] / (image.dim(1) - 1.f), 1.f - yx[0] / (image.dim(0) - 1.f));
      // = (convert<float>(yx) / convert<float>(image.dims() - 1)).rev();
      mesh.update_string(v, "uv", csform_vec(str, uv));
    });
    for_coords(image.dims() - 1, [&](const Vec2<int>& yx) {
      if (0) {
        mesh.create_face(V(matv[yx], matv[yx + V(0, 1)], matv[yx + V(1, 1)], matv[yx + V(1, 0)]));
      } else {
        mesh.create_face(matv[yx], matv[yx + V(1, 1)], matv[yx + V(0, 1)]);
        mesh.create_face(matv[yx], matv[yx + V(1, 0)], matv[yx + V(1, 1)]);
      }
    });
    WFile fi("mesh.m");
    mesh.write(fi());
  } else if (name == "vlp_to_color_ramps") {
    // Filterimage ~/proj/videoloops/data/ReallyFreakinAll/out/HDdunravenpass1_loop.vlp -proc vlp_to_color_ramps v -noo
    // creates v.static.png v.start.png v.period.png v.activation.png
    string rootname = args.get_filename();
    assertx(image.size() > 0);
    const Vec<string, 4> channel_name{"static", "start", "period", "activation"};
    int est_num_input_frames = 0;
    for_coords(image.dims(), [&](const Vec2<int>& yx) {
      int staticframe = image[yx][0], start = image[yx][1], period = image[yx][2];
      int endframe = period <= 1 ? start : start + period - 1;
      est_num_input_frames = max({est_num_input_frames, staticframe + 1, endframe + 1});
    });
    assertx(est_num_input_frames);
    SHOW(est_num_input_frames);
    for_int(z, 4) {
      Image image2(image.dims());
      parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) {
        bool is_static = (z == 2 || z == 3) && image[yx][2] == 0;
        uint8_t val = image[yx][z];
        if (z < 3) val = clamp_to_uint8(int(val * 255.f / (est_num_input_frames - 1) + .5f));
        image2[yx] = is_static ? Pixel::gray(230) : k_color_ramp[val];
      });
      image2.write_file(rootname + "." + channel_name[z] + ".png");
    }
  } else if (name == "vlp_mask_to_color") {
    // Filterimage ~/proj/fastloops/data/test/HDmorningsteam1_vlp_level0_opt0.png -proc vlp_mask_to_color ~/proj/fastloops/data/test/HDmorningsteam1_mask_level0_opt0.png >v.png
    assertx(image.size() > 0);
    string mask_name = args.get_filename();
    bool have_mask = mask_name != "none";
    Image image_mask;
    if (have_mask) {
      image_mask.read_file(mask_name);
      assertx(image_mask.dims() == image.dims());
    }
    int est_num_input_frames = 0;
    for_coords(image.dims(), [&](const Vec2<int>& yx) {
      int staticframe = image[yx][0], start = image[yx][1], period = image[yx][2];
      int endframe = period <= 1 ? start : start + period - 1;
      est_num_input_frames = max({est_num_input_frames, staticframe + 1, endframe + 1});
    });
    assertx(est_num_input_frames);  // SHOW(est_num_input_frames);
    Image timage(image);
    parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) {
      bool is_masked = have_mask && image_mask[yx] == Pixel::gray(255);
      bool is_static = timage[yx][2] == 0;
      int start = timage[yx][1];
      int period = timage[yx][2];
      float fstart = float(start) / (est_num_input_frames - period - 1);
      float fperiod = float(period) / (est_num_input_frames - 1);
      Pixel pix = k_color_ramp[clamp_to_uint8(int(fperiod * 255.f + .5f))];
      for_int(c, 3) pix[c] = clamp_to_uint8(int(pix[c] * (.4f + .6f * fstart)));
      if (is_static) pix = Pixel::gray(have_mask ? 180 : 230);  // there are so few, make them more prominent
      if (is_masked) pix = Pixel::white();
      image[yx] = pix;
    });
  } else if (name == "entropy") {
    // Filterimage -create 256 20 -proc color_ramp -to png >~/data/image/color_ramp.png
    assertx(image.size() > 0);
    Encoding<Vec2<int>> encoding;
    for_coords(image.dims(), [&](const Vec2<int>& yx) {
      int start = image[yx][1];
      int period = image[yx][2];
      Vec2<int> period_start{period, start};
      if (0 || period > 1) encoding.add(period_start, 1.f);
    });
    std::cout << encoding.norm_entropy() << "\n";
    nooutput = true;
  } else if (name == "color_ramp") {
    Grid<1, Vector4> color_ramp(V(k_color_ramp.num()));
    convert(CGridView<1, Pixel>(k_color_ramp), color_ramp);
    Vector4 vgcolor;
    convert(CGrid1View(gcolor), Grid1View(vgcolor));
    for_coords(image.dims(), [&](const Vec2<int>& yx) {
      image[yx] = sample_domain(color_ramp, V((yx[1] + .5f) / image.xsize()), g_filterbs.head<1>(), &vgcolor).pixel();
    });
  } else if (name == "compress_yuv420") {
    Nv12 nv12(image.dims());
    convert_Image_to_Nv12(image, nv12);
    convert_Nv12_to_Image(nv12, image);
  } else {
    args.problem("procedure '" + name + "' unrecognized");
  }
}

void do_diff(Args& args) {
  string filename = args.get_filename();
  Image image2(filename);
  assertx(same_size(image, image2) && image.zsize() == image2.zsize());
  const int nz = image.zsize();
  parallel_for_coords(
      image.dims(),
      [&](const Vec2<int>& yx) {
        for_int(z, nz) image[yx][z] = clamp_to_uint8(128 + int(image[yx][z]) - int(image2[yx][z]));
      },
      20);
}

void do_maxdiff(Args& args) {
  float thresh = args.get_float();
  string filename = args.get_filename();
  Image image2(filename);
  assertx(same_size(image, image2) && image.zsize() == image2.zsize());
  const int nz = image.zsize();
  int maxdiff = 0;
  for (const auto& yx : range(image.dims())) {
    for_int(z, nz) maxdiff = max(maxdiff, abs(int(image[yx][z]) - int(image2[yx][z])));
  }
  if (maxdiff > thresh) {
    SHOW(maxdiff, thresh);
    assertnever("maxdiff threshold exceeded");
  }
}

void do_maxrmsdiff(Args& args) {
  float thresh = args.get_float();
  string filename = args.get_filename();
  Image image2(filename);
  assertx(same_size(image, image2) && image.zsize() == image2.zsize());
  const int nz = image.zsize();
  Stat stat;
  for (const auto& yx : range(image.dims())) {
    for_int(z, nz) stat.enter(int(image[yx][z]) - int(image2[yx][z]));
  }
  if (0 || getenv_int("SHOW_MAX_RMS_DIFF")) SHOW(stat.rms());
  if (stat.rms() > thresh) {
    SHOW(stat.rms(), thresh);
    assertnever("rms threshold exceeded");
  }
}

void do_compare(Args& args) {
  string filename = args.get_filename();
  Image image2(filename);
  const Image& image1 = image;
  assertx(same_size(image1, image2) && image1.zsize() == image2.zsize());
  const int r = 5;                    // window radius
  double sigma = 2.5;                 // number of standard deviations at edge of window
  Array<double> ar_gauss(2 * r + 1);  // cached Gaussian weights
  for_int(i, 2 * r + 1) ar_gauss[i] = gaussian(double(i) - r, r / sigma);
  {  // normalize the 1D weights such that 2D tensor sums to 1 over discrete 2D window
    double sum = 0.;
    for_intL(dy, -r, r + 1) for_intL(dx, -r, r + 1) sum += ar_gauss[dy + r] * ar_gauss[dx + r];
    for_int(i, 2 * r + 1) {
      ar_gauss[i] /= sqrt(sum);
      if (0) SHOW(ar_gauss[i]);
    }
  }
  if (0) {
    double sum = 0.;
    double var = 0.;
    for_intL(dy, -r, r + 1) for_intL(dx, -r, r + 1) {
      sum += ar_gauss[dy + r] * ar_gauss[dx + r];
      var += ar_gauss[dy + r] * ar_gauss[dx + r] * (square(dy) + square(dx));
    }
    assertx(abs(sum - 1.) < 1e-6);
    showf("Effective spatial standard deviation of windowed Gaussian is %f\n", sqrt(var));
  }
  int allmax = 0;
  for (const auto& yx : range(image.dims())) {
    for_int(z, image.zsize()) allmax = max(allmax, abs(image1[yx][z] - image2[yx][z]));
  }
  Array<double> ar_err2(image.zsize()), ar_mssim(image.zsize());
  parallel_for_each(range(image.zsize()), [&](const int z) {
    double err2 = 0.;
    double mssim = 0.;
    for_int(y, image1.ysize()) for_int(x, image1.xsize()) {
      const Vec2<int> yx(y, x);
      double err = image1[yx][z] - image2[yx][z];
      err2 += square(err);
      double mean1 = 0.;
      double mean2 = 0.;
      for (const auto& yxd : range(twice(-r), twice(r + 1))) {
        double w = ar_gauss[yxd[0] + r] * ar_gauss[yxd[1] + r];
        mean1 += image1.inside(yx + yxd, k_reflected2)[z] * w;
        mean2 += image2.inside(yx + yxd, k_reflected2)[z] * w;
      }
      double var1 = 0.;
      double var2 = 0.;
      double corr = 0.;
      for (const auto& yxd : range(twice(-r), twice(r + 1))) {
        double w = ar_gauss[yxd[0] + r] * ar_gauss[yxd[1] + r];
        double v1 = image1.inside(yx + yxd, k_reflected2)[z];
        double v2 = image2.inside(yx + yxd, k_reflected2)[z];
        var1 += square(v1 - mean1) * w;
        var2 += square(v2 - mean2) * w;
        corr += (v1 - mean1) * (v2 - mean2) * w;
      }
      float scorr = float(corr);
      // float s = float((corr + c3) / (sdv1 * sdv2 + c3));
      const float c1 = square(0.01f * 255.0f);
      const float c2 = square(0.03f * 255.0f);
      float ssim = float((2.f * mean1 * mean2 + c1) * (2.f * scorr + c2) /
                         ((square(mean1) + square(mean2) + c1) * (var1 + var2 + c2)));
      // HH_SSTAT(Sssim, ssim);
      mssim += ssim;
    }
    ar_err2[z] = err2 / image1.size();
    ar_mssim[z] = mssim / image1.size();
  });
  double allerr2 = 0., allmssim = 0.;
  for_int(z, image.zsize()) {
    double err2 = ar_err2[z];
    double mssim = ar_mssim[z];
    const double psnr = 20. * std::log10(255. / (my_sqrt(err2) + 1e-10));
    showf("channel%d: RMSE[0,255]=%f PSNR=%f MSSIM[0,1]=%f\n", z, my_sqrt(err2), psnr, mssim);
    allerr2 += err2;
    allmssim += mssim;
  }
  allerr2 /= image.zsize();
  allmssim /= image.zsize();
  const double psnr = 20. * std::log10(255. / (my_sqrt(allerr2) + 1e-10));
  showf("all: RMSE=%f PSNR=%f MAXE=%d MSSIM=%f\n", allerr2, psnr, allmax, allmssim);
  nooutput = true;
}

// *** Pyramid  (see also Pyramid.cpp)

// Color-space conversion of a single pixel (each channel in range [0.f, 255.f]).
inline Vector4 RGB_to_LAB(const Vector4& pv) {
  Vector4 v = pv;
  const float eps = 0.008856f;
  const float k = 903.3f;
  for_int(i, 3) {
    if (v[i] <= 0.04045f)
      v[i] /= 12.92f;
    else
      v[i] = pow((v[i] + 0.055f) / 1.055f, 2.4f);
  }
  float X = 0.412424f * v[0] + 0.357579f * v[1] + 0.180464f * v[2];
  float Y = 0.212656f * v[0] + 0.715158f * v[1] + 0.0721856f * v[2];
  float Z = 0.0193324f * v[0] + 0.119193f * v[1] + 0.950444f * v[2];
  v[0] = X / 0.95047f;
  v[1] = Y / 1.0f;
  v[2] = Z / 1.08883f;
  for_int(i, 3) {
    if (v[i] > eps)
      v[i] = pow(v[i], 1 / 3.f);
    else
      v[i] = (k * v[i] + 16.f) / 116.f;
  }
  X = (116.f * v[1]) - 16.f;
  Y = 500.f * (v[0] - v[1]);
  Z = 200.f * (v[1] - v[2]);
  return Vector4(V(X, Y, Z, 0.f));
}

// Color-space conversion of a single pixel (each channel in range [0.f, 255.f]).
inline Vector4 LAB_to_RGB(const Vector4& pv) {
  Vector4 v = pv;
  float Y = (v[0] + 16.f) / 116.f;
  float X = (v[1] / 500.f) + Y;
  float Z = Y - (v[2] / 200.f);
  v[0] = X;
  v[1] = Y;
  v[2] = Z;
  for_int(i, 3) {
    float pf = pow(v[i], 3.f);
    if (pf > 0.008856f)
      v[i] = pf;
    else
      v[i] = (v[i] - 16.f / 116.f) / 7.787f;
  }
  X = v[0] * 0.95047f;
  Y = v[1] * 1.0f;
  Z = v[2] * 1.08883f;
  v[0] = X * 3.24071f + Y * -1.53726f + Z * -0.498571f;
  v[1] = X * -0.969258f + Y * 1.87599f + Z * 0.0415557f;
  v[2] = X * 0.0556352f + Y * -0.203996f + Z * 1.05707f;
  v[3] = 255.f;
  for_int(i, 3) {
    if (v[i] > 0.0031308f)
      v[i] = 1.055f * (pow(v[i], (1.0f / 2.4f))) - 0.055f;
    else
      v[i] *= 12.92f;
    if (1) {  // HH
      v[i] = clamp(v[i], 0.f, 255.999f);
    }
  }
  return v;
}

// Color-space conversion of an image.
auto convert_to_LAB(CMatrixView<Vector4> mat_RGB) {
  Matrix<Vector4> mat_LAB(mat_RGB.dims());
  parallel_for_coords(
      mat_RGB.dims(), [&](const Vec2<int>& yx) { mat_LAB[yx] = RGB_to_LAB(mat_RGB[yx]); }, 50);
  return mat_LAB;
}

// Color-space conversion of an image.
auto convert_to_RGB(CMatrixView<Vector4> mat_LAB) {
  Matrix<Vector4> mat_RGB(mat_LAB.dims());
  parallel_for_coords(
      mat_LAB.dims(), [&](const Vec2<int>& yx) { mat_RGB[yx] = LAB_to_RGB(mat_LAB[yx]); }, 50);
  return mat_RGB;
}

auto convert_image_mat(CMatrixView<Pixel> im) {
  Matrix<Vector4> mat(im.dims());
  parallel_for_coords(
      im.dims(),
      [&](const Vec2<int>& yx) {
        mat[yx] = to_Vector4_raw(im[yx].data());  // range [0.f, 255.f]
      },
      5);
  return mat;
}

auto convert_mat_image(CMatrixView<Vector4> mat) {
  Matrix<Pixel> im(mat.dims());
  parallel_for_coords(
      mat.dims(),
      [&](const Vec2<int>& yx) { im[yx] = general_clamp(mat[yx], Vector4(0.f), Vector4(255.99f)).raw_pixel(); }, 5);
  return im;
}

// Downsample by one level, creating coarser image from finer image.
auto downsample_image(CMatrixView<Vector4> mat_F) {
  assertx(mat_F.dims() % 2 == V(0, 0));
  Matrix<Vector4> mat_C(mat_F.dims() / 2);
  // downsampling weights: (-3 -9 29 111 111 29 -9 -3) / 256
  const int kn = 8;
  Array<float> fkernel(kn);
  fkernel[0] = fkernel[7] = -3.f / 256.f;
  fkernel[1] = fkernel[6] = -9.f / 256.f;
  fkernel[2] = fkernel[5] = 29.f / 256.f;
  fkernel[3] = fkernel[4] = 111.f / 256.f;
  if (0) {  // box filter
    fill(fkernel, 0.f);
    fkernel[3] = fkernel[4] = 1.f;
  }
  //
  if (0) {  // slow implementation
    parallel_for_coords(
        mat_C.dims(),
        [&](const Vec2<int>& yx) {
          const Vec2<int> yxf0 = yx * 2 - kn / 2 + 1;
          Vector4 sum{};
          for_int(iy, kn) for_int(ix, kn) {
            sum += (fkernel[iy] * fkernel[ix]) * mat_F.inside(yxf0 + V(iy, ix), k_reflected2);
          }
          mat_C[yx] = sum;
        },
        kn * kn * 10);
  } else {  // faster: first downsample horizontally, then vertically
    // Possible optimization: lift boundary testing outside of loops.
    Matrix<Vector4> mtmp(mat_F.dims() / V(1, 2));  // non-square
    parallel_for_coords(
        mtmp.dims(),
        [&](const Vec2<int>& yx) {
          int xf0 = yx[1] * 2 - kn / 2 + 1;
          Vector4 sum{};
          for_int(ix, kn) sum += fkernel[ix] * mat_F.inside(V(yx[0], xf0 + ix), k_reflected2);
          mtmp[yx] = sum;
        },
        kn * 8);
    parallel_for_coords(
        mat_C.dims(),
        [&](const Vec2<int>& yx) {
          int yf0 = yx[0] * 2 - kn / 2 + 1;
          Vector4 sum{};
          for_int(iy, kn) sum += fkernel[iy] * mtmp.inside(V(yf0 + iy, yx[1]), k_reflected2);
          mat_C[yx] = sum;
        },
        kn * 8);
  }
  return mat_C;
}

// Upsample by one level, creating finer image from coarser image.
auto upsample_image(CMatrixView<Vector4> mat_C) {
  Matrix<Vector4> mat_F(mat_C.dims() * 2);
  // upsampling weights: (-9 111 29 -3) / 128  and (-3 29 111 -9) / 128  on alternating pixels
  const int kn = 4;
  Array<float> fkernel(kn);
  fkernel[0] = -9.f / 128.f;
  fkernel[1] = 111.f / 128.f;
  fkernel[2] = 29.f / 128.f;
  fkernel[3] = -3.f / 128.f;
  if (0) {  // box filter
    fill(fkernel, 0.f);
    fkernel[1] = 1.f;
  }
  Vec2<Array<float>> fkernels;
  fkernels[0] = fkernel;
  fkernels[1] = reverse(clone(fkernel));
  // Possible optimization: lift boundary testing outside of loops.
  parallel_for_coords(
      mat_F.dims(),
      [&](const Vec2<int>& yx) {
        const Vec2<int> yxc = (yx + 1) / 2, yxodd = (yx + 1) - (yxc * 2), yxc0 = yxc - kn / 2;
        Vector4 sum{};
        for_int(iy, kn) for_int(ix, kn) {
          sum += (fkernels[yxodd[0]][iy] * fkernels[yxodd[1]][ix]) * mat_C.inside(yxc0 + V(iy, ix), k_reflected2);
        }
        mat_F[yx] = sum;
      },
      kn * kn * 10);
  return mat_F;
}

// Precompute Gaussian window weights (in 1D).
Array<float> compute_window_weights() {
  // const int window_radius = 10;  // for 21^2 window
  static const int window_radius = getenv_int("XFER_RADIUS", 10, true);
  const int window_diam = window_radius * 2 + 1;
  // const float window_sdv = 4.0f;  // standard deviation of Gaussian, in pixels.
  const float window_sdv = window_radius / 2.5f;
  Array<float> fwindow(window_diam);
  for_int(x, fwindow.num()) fwindow[x] = gaussian(float(x) - window_radius, window_sdv);
  float vsum = float(sum(fwindow));
  if (0) SHOW(vsum);
  fwindow /= vsum;  // normalize the 1D window weights
  if (0) SHOW(fwindow);
  return fwindow;
}

// Combine detail structure from mat_s and color from mat_c to form new image mat_out.
// Also save the intermediate z-scores into mat_zscore for visualization.
void structure_transfer_zscore(CMatrixView<Vector4> mat_s0, CMatrixView<Vector4>& mat_c0, Matrix<Vector4>& mat_out,
                               Matrix<Vector4>& mat_zscore) {
  assertx(same_size(mat_s0, mat_c0));
  Array<float> fwindow = compute_window_weights();
  const int window_diam = fwindow.num();
  const int window_radius = (window_diam - 1) / 2;
  // Convert both the color image and the structure image from RGB space to LAB space.
  assertw(use_lab);
  Matrix<Vector4> mat_s(use_lab ? convert_to_LAB(mat_s0) : mat_s0);
  Matrix<Vector4> mat_c(use_lab ? convert_to_LAB(mat_c0) : mat_c0);
  static const float zscore_scale = getenv_float("ZSCORE_SCALE", 1.f, true);
  mat_out.init(mat_s.dims());
  mat_zscore.init(mat_s.dims());
  Vector4 minsvar(0.f);  // min structural variance; > 0.f to avoid negative sqrt and division by zero.
  if (1) {               // to remove blocking noise in jpg-compressed image
    if (!use_lab) minsvar = Vector4(square(5.f));
    // LAB luminance has wider range
    if (use_lab) minsvar = Vector4(V(square(200.f), square(40.f), square(40.f), square(40.f)));
  }
  const bool optimized = true;
  parallel_for_each(
      range(mat_s.ysize()),
      [&](const int y) {
        Array<Vector4> fscolsum(mat_s.xsize()), fscolsum2(mat_s.xsize());
        Array<Vector4> fccolsum(mat_c.xsize()), fccolsum2(mat_c.xsize());
        if (optimized) {
          // HH_ATIMER("___rows");
          fill(fscolsum, Vector4{});
          fill(fscolsum2, Vector4{});
          fill(fccolsum, Vector4{});
          fill(fccolsum2, Vector4{});
          for_int(iy, window_diam) {
            float w = fwindow[iy];
            int yy = y - window_radius + iy;
            assertx(map_boundaryrule_1D(yy, mat_s.ysize(), k_reflected));
            for_int(x, mat_s.xsize()) {
              Vector4 sv = mat_s[yy][x];
              fscolsum[x] += w * sv;
              fscolsum2[x] += w * square(sv);
              Vector4 cv = mat_c[yy][x];
              fccolsum[x] += w * cv;
              fccolsum2[x] += w * square(cv);
            }
          }
        }
        // HH_ATIMER("___rest");
        for_int(x, mat_s.xsize()) {
          Vector4 ssum{}, ssum2{};  // structure sum and sum squared
          Vector4 csum{}, csum2{};  // color sum and sum squared
          if (!optimized) {
            // Gather window statistics in structure image (downsampled fine image).
            for_int(iy, window_diam) for_int(ix, window_diam) {
              Vector4 v = mat_s.inside(y - window_radius + iy, x - window_radius + ix, k_reflected);  // pixel val
              float w = fwindow[iy] * fwindow[ix];                                                    // weight
              ssum += w * v;
              ssum2 += w * square(v);
            }
            // if (0) { HH_SSTAT(Ssmean, smean[0]); HH_SSTAT(Sssdv, ssdv[0]); }  // note: LAB have broader range.
            // Gather window statistics in color image (coarse image).
            for_int(iy, window_diam) for_int(ix, window_diam) {
              Vector4 v = mat_c.inside(y - window_radius + iy, x - window_radius + ix, k_reflected);  // pixel val
              float w = fwindow[iy] * fwindow[ix];                                                    // weight
              csum += w * v;
              csum2 += w * square(v);
            }
          } else {
            for_int(ix, window_diam) {
              float w = fwindow[ix];
              int xx = x - window_radius + ix;
              bool b = map_boundaryrule_1D(xx, mat_s.xsize(), k_reflected);
              ASSERTX(b);
              ssum += w * fscolsum[xx];
              ssum2 += w * fscolsum2[xx];
              csum += w * fccolsum[xx];
              csum2 += w * fccolsum2[xx];
            }
          }
          Vector4 smean = ssum;
          Vector4 ssdv = sqrt(max(ssum2 - square(ssum), minsvar));
          Vector4 cmean = csum;
          Vector4 csdv = sqrt(max(csum2 - square(csum), Vector4(0.f)));
          Vector4 zscore = (mat_s[y][x] - smean) / ssdv;
          mat_out[y][x] = cmean + zscore * csdv * zscore_scale;
          if (use_lab) zscore = Vector4(zscore[0]);  // Z score based on luminance only
          mat_zscore[y][x] = zscore * (255.0f / 6.0f);
        }
      },
      mat_s.xsize() * 2 * window_radius * 20);
  if (use_lab) mat_out = convert_to_RGB(mat_out);
}

struct VXY {
  float v, w;
};
bool operator<(const VXY& a, const VXY& b) { return a.v < b.v; }

// Same but use rank rather than z-score.
void structure_transfer_rank(CMatrixView<Vector4> mat_s0, CMatrixView<Vector4>& mat_c0, Matrix<Vector4>& mat_out,
                             Matrix<Vector4>& mat_zscore) {
  assertx(same_size(mat_s0, mat_c0));
  Array<float> fwindow = compute_window_weights();
  const int window_diam = fwindow.num();
  const int window_radius = (window_diam - 1) / 2;
  // Convert both the color image and the structure image from RGB space to LAB space.
  Matrix<Vector4> mat_s(use_lab ? convert_to_LAB(mat_s0) : mat_s0);
  Matrix<Vector4> mat_c(use_lab ? convert_to_LAB(mat_c0) : mat_c0);
  Array<VXY> ar(square(window_diam));
  mat_out.init(mat_s.dims());
  mat_zscore.init(mat_s.dims());
  assertw(use_lab);
  const int NCH = 3;
  for_int(ch, NCH) {
    parallel_for_coords(
        mat_s.dims(),
        [&](const Vec2<int>& yx) {
          ar.init(0);                      // (initially unsorted) pdf of color image window
          float scenterv = mat_s[yx][ch];  // value of center pixel in structure image
          float scenterrank = 0.f;         // center pixel rank in structure image
          for (const auto& iyx : range(twice(window_diam))) {
            float w = fwindow[iyx[0]] * fwindow[iyx[1]];
            float sv = mat_s.inside(yx - window_radius + iyx, k_reflected2)[ch];
            float cv = mat_c.inside(yx - window_radius + iyx, k_reflected2)[ch];
            if (sv < scenterv) scenterrank += w;
            VXY vxy;
            vxy.v = cv;
            vxy.w = w;
            ar.push(vxy);
          }
          sort(ar);
          float f = scenterrank;
          float val = ar[square(window_diam) - 1].v;
          for_int(idx, square(window_diam)) {
            f -= ar[idx].w;
            if (f <= 0.f) {
              val = ar[idx].v;
              break;
            }
          }
          mat_out[yx][ch] = val;
          mat_zscore[yx][ch] = ch == 0 ? scenterrank * 255.f : mat_zscore[yx][0];
        },
        square(window_diam) * 20);
  }
  if (use_lab) mat_out = convert_to_RGB(mat_out);
}

void structure_transfer(CMatrixView<Vector4> mat_s, CMatrixView<Vector4>& mat_c, Matrix<Vector4>& mat_out,
                        Matrix<Vector4>& mat_zscore) {
  HH_TIMER("__structure_transfer");
  if (getenv_bool("USE_RANK_TRANSFER")) {  // results in grain artifacts
    structure_transfer_rank(mat_s, mat_c, mat_out, mat_zscore);
  } else if (getenv_bool("USE_NO_TRANSFER")) {  // results in ghosting
    mat_out = mat_c;
    fill(mat_zscore, Vector4(0.f));
  } else {  // z-score transfer is best
    structure_transfer_zscore(mat_s, mat_c, mat_out, mat_zscore);
  }
}

void output_image(CMatrixView<Vector4> mat, const string& filename) {
  Image nimage = convert_mat_image(mat);
  nimage.write_file(filename);
}

// Given the coarse-scale image (already loaded) and the fine-scale image (specified as argument),
//  construct smooth visual transition.
void do_pyramid(Args& args) {
  // e.g.  (cd ~/tmp; cp -p ~/data/image/misc/city.input.{13,17}.jpg .; Filterimage city.input.13.jpg -pyramid city.input.17.jpg; ls -al)
  string ffilename = args.get_filename();  // argument is fine-scale image
  HH_TIMER("_pyramid");
  string rootname = ffilename;
  assertx(contains(rootname, '.'));
  rootname.erase(rootname.find('.'));  // unlike get_path_root(), remove multiple extensions
  Image& imagec = image;
  Image imagef(ffilename);
  int sizeratio = imagef.ysize() / imagec.ysize();
  assertx(imagef.dims() == imagec.dims() * sizeratio);
  assertx(is_pow2(sizeratio));
  int ld = int_floor_log2(sizeratio);  // (octave) level difference
  const int lbase = 10;                // offset just for easily sortable file numbering
  int lf = lbase + ld;
  int lc = lbase + 0;
  Array<Matrix<Vector4>> mat_gaussianf(lf + 1);  // Gaussian pyramid of fine-scale image; never re-allocated
  // Convert the fine-scale image, and enter it into the appropriate level of the Gaussian pyramid.
  mat_gaussianf[lf] = convert_image_mat(imagef);
  // Create the Gaussian image pyramid of the fine-scale image.
  if (ld > 0) {
    HH_TIMER("__pyramid_downsample");
    for (int l = lf - 1; l >= lc; --l) {
      mat_gaussianf[l] = downsample_image(mat_gaussianf[l + 1]);
    }
  }
  if (ld > 0) output_image(mat_gaussianf[lc], rootname + ".down.png");
  // Convert the coarse-scale image.
  Matrix<Vector4> mat_c = convert_image_mat(imagec);
  // Perform structure transfer, combining detail of the downsampled fine image and color of the coarse image.
  Matrix<Vector4> mat_xfer, mat_zscore;
  structure_transfer(mat_gaussianf[lc], mat_c, mat_xfer, mat_zscore);
  if (getenv_bool("OUTPUT_ZSCORE")) output_image(mat_zscore, rootname + ".Z.png");
  // Downsample the structure-transferred image and output.
  if (ld > 0) {
    Matrix<Vector4> mat_tmp1 = downsample_image(mat_xfer);
    Matrix<Vector4> mat_tmp2 = downsample_image(mat_tmp1);
    output_image(mat_tmp2, sform("%s.out.%02d.png", rootname.c_str(), lbase - 2));
    output_image(mat_tmp1, sform("%s.out.%02d.png", rootname.c_str(), lbase - 1));
    output_image(mat_xfer, sform("%s.out.%02d.png", rootname.c_str(), lbase + 0));
  } else {
    output_image(mat_xfer, sform("%s.xfer.png", rootname.c_str()));
  }
  // Iteratively upsample and blend the difference image.
  const int iskipfinest = 1;                 // since will be unmodified
  if (getenv_bool("USE_LINEAR_BLENDING")) {  // results in blurring
    for_intL(l, lc + 1, lf - iskipfinest + 1) {
      Matrix<Vector4> mat_tmp = upsample_image(mat_xfer);
      float alpha = (float(lf) - float(l)) / (float(lf) - float(lc));
      parallel_for_coords(
          mat_tmp.dims(),
          [&](const Vec2<int>& yx) { mat_gaussianf[l][yx] += (mat_tmp[yx] - mat_gaussianf[l][yx]) * alpha; }, 4);
      if (l < lf - iskipfinest) mat_xfer = mat_tmp;
    }
  } else {  // Clipped Laplacian blending is best
    // Compute the coarse-scale difference image.
    Matrix<Vector4> mat_diff(imagec.dims());
    parallel_for_coords(
        imagec.dims(), [&](const Vec2<int>& yx) { mat_diff[yx] = mat_xfer[yx] - mat_gaussianf[lc][yx]; }, 2);
    HH_TIMER("__upsample_blend");
    for_intL(l, lc + 1, lf - iskipfinest + 1) {
      Matrix<Vector4> mat_tmp = upsample_image(mat_diff);
      float alpha = (float(lf) - float(l)) / (float(lf) - float(lc));
      parallel_for_coords(
          mat_tmp.dims(), [&](const Vec2<int>& yx) { mat_gaussianf[l][yx] += mat_tmp[yx] * alpha; }, 2);
      if (l < lf - iskipfinest) mat_diff = mat_tmp;
    }
  }
  for_intL(l, lc + 1, lf - iskipfinest + 1) {
    output_image(mat_gaussianf[l], sform("%s.out.%02d.png", rootname.c_str(), lbase + (l - lc)));
  }
  nooutput = true;
}

// Given the color image (already loaded) and the structure image (specified as argument),
//  perform structure transfer.
void do_structuretransfer(Args& args) {
  // Filterimage ~/data/image/misc/city.input.13.jpg -structuretransfer ~/data/image/misc/city.down.png | imgv
  string sfilename = args.get_filename();  // argument is structure image
  Image& cimage = image;
  Image simage(sfilename);
  Matrix<Vector4> mat_c = convert_image_mat(cimage);
  Matrix<Vector4> mat_s = convert_image_mat(simage);
  Matrix<Vector4> mat_xfer, mat_zscore;
  structure_transfer(mat_s, mat_c, mat_xfer, mat_zscore);
  if (getenv_bool("OUTPUT_ZSCORE")) output_image(mat_zscore, "zscore.png");
  image = convert_mat_image(mat_xfer);
}

// (cd ~/proj/morph/data/quadmesh; Filterimage image2.png -filter k -boundaryrule r -resamplemesh 128_mesh2.m | G3d -lighta 1 -lights 0 -st imagenew)
void do_resamplemesh(Args& args) {
  string mfile = args.get_filename();
  assertx(min(image.dims()) >= 2);
  GMesh mesh{RFile(mfile)()};
  Matrix<Vector4> imagev(image.dims());
  convert(image, imagev);
  Vec2<FilterBnd> filterbs = g_filterbs;
  if (filterbs[0].filter().has_inv_convolution() || filterbs[1].filter().has_inv_convolution())
    filterbs = inverse_convolution(imagev, filterbs);
  string str;
  for (Vertex v : mesh.vertices()) {
    Point p = mesh.point(v);  // in range [0, 1]
    Vector4 vec4 = sample_domain(imagev, V(p[1], p[0]), filterbs);
    vec4 = general_clamp(vec4, Vector4(0.f), Vector4(1.f));
    mesh.update_string(v, "rgb", csform_vec(str, ArView(vec4.data(), 3)));
  }
  hh_clean_up();
  mesh.write(std::cout);
  std::cout.flush();
  nooutput = true;
}

}  // namespace

int main(int argc, const char** argv) {
  my_setenv("NO_DIAGNOSTICS_IN_STDOUT", "1");
  ParseArgs args(argc, argv);
  HH_ARGSC("(Image coordinates: (x = 0, y = 0) at (left, top).)");
  HH_ARGSC("An image is read from stdin or first arg except with the following arguments:");
  HH_ARGSD(nostdin, ": do not attempt to read input image from stdin");
  HH_ARGSD(create, "width height : create white image");
  HH_ARGSD(as_fit, "nx ny : when assembling, scale each image uniformly to fit into this size");
  HH_ARGSD(as_cropsides, "l r t b : when assembling, crop each image");
  HH_ARGSD(assemble, "nx ny images_lr_tb_order : concatenate grid of images");
  HH_ARGSD(fromtxt, "nx ny nch file.txt : read values in range [0., 1.]");
  HH_ARGSD(invideo, "videofile : process each video frame, writing to a new video");
  HH_ARGSC("", ":");
  HH_ARGSD(to, "suffix : set output format (jpg, png, bmp, ppm, rgb, tif, wmp)");
  HH_ARGSD(outfile, "filename : output an intermediate image");
  HH_ARGSF(nooutput, ": do not output final image on stdout");
  HH_ARGSD(info, ": print image statistics");
  HH_ARGSD(stat, ": equivalent to '-info -nooutput'");
  HH_ARGSD(sizes, ": print 'width height'");
  HH_ARGSD(tops, ": convert to encapsulated postscript");
  HH_ARGSC("", ":");
  HH_ARGSD(color, "r g b a : set color for various operations (default 255's)");
  HH_ARGSD(not, ": negate test for color selection");
  HH_ARGSD(getcolorxy, "x y : get color from pixel ((x = 0, y = 0) = (left, top))");
  HH_ARGSD(boundaryrule, "c : reflected/periodic/clamped/border");
  HH_ARGSD(hboundaryrule, "c : set just horizontal boundary rule");
  HH_ARGSD(vboundaryrule, "c : set just vertical boundary rule");
  HH_ARGSC("", ":");
  HH_ARGSD(cropsides, "l r t b : crop image (introduce specified color if negative)");
  HH_ARGSD(cropl, "l : crop image");
  HH_ARGSD(cropr, "r : crop image");
  HH_ARGSD(cropt, "t : crop image");
  HH_ARGSD(cropb, "b : crop image");
  HH_ARGSD(cropall, "i : crop all sides");
  HH_ARGSD(cropsquare, "x y size : crop square centered at (x, y)");
  HH_ARGSD(croprectangle, "x y xsize ysize : crop rectangle centered at (x, y)");
  HH_ARGSD(cropcoord, "x0 y0 x1 y1 : crop box within bounds x0<=x<x1");
  HH_ARGSD(croptodims, "x y : centered crop to obtain new dimensions");
  HH_ARGSD(cropmatte, ": crop all sides matching the set color");
  HH_ARGSC("", ":");
  HH_ARGSD(filter, "c : imp/box/tri/quad/mitchell/keys/spline/omoms/gauss/preprocess/justspline");
  HH_ARGSD(hfilter, "c : set just horizontal filter");
  HH_ARGSD(vfilter, "c : set just vertical filter");
  HH_ARGSD(scaleunif, "fac : zoom image (upsample and/or downsample)");
  HH_ARGSD(scalenonunif, "facx facy : zoom image");
  HH_ARGSD(scaletox, "x : uniform scale to x width");
  HH_ARGSD(scaletoy, "y : uniform scale to y height");
  HH_ARGSD(scaletodims, "x y : non-uniform scale");
  HH_ARGSD(scaleinside, "x y : uniform scale to become no larger than rectangle");
  HH_ARGSD(scalehalf2n1, ": subsample 4**n + 1 -> 2**n + 1 on each axis");
  HH_ARGSC("", ":");
  HH_ARGSD(flipvertical, ": reverse rows");
  HH_ARGSD(fliphorizontal, ": reverse columns");
  HH_ARGSD(rot180, ": rotate by 180 degrees");
  HH_ARGSD(rotate, "ang : rotate ccw by ang degrees");
  HH_ARGSD(gtransf, "'frame' : geometrically transform image using (y, x, 0) 3D-coordinates)");
  HH_ARGSD(tile, "nx ny : grid repeat");
  HH_ARGSD(disassemble, "tilex tiley rootname : break up into multiple image files of this size");
  HH_ARGSD(gridcrop, "nx ny sizex sizey : assemble grid of regions (with as_cropsides)");
  HH_ARGSC("", ":");
  HH_ARGSD(overlayimage, "xl yt image : place image above current one");
  HH_ARGSD(drawrectangle, "x0 y0 x1 y1 : replace specified region by current color");
  HH_ARGSP(tolerance, "f : max Euclidean distance in '-replace'.");
  HH_ARGSD(replace, "r g b a : replace all pixels matching specified color with this color");
  HH_ARGSD(gamma, "v : gammawarp image");
  HH_ARGSD(tobw, ": convert to grayscale");
  HH_ARGSD(tocolor, ": convert to color");
  HH_ARGSD(hue, ": just keep hue");
  HH_ARGSD(noalpha, ": remove an independent alpha channel");
  HH_ARGSD(undoalpha, ": remove alpha channel and adjust RGB values to undo premultiplication");
  HH_ARGSD(readalpha, "image_alpha : set alpha channel from Red channel of this image");
  HH_ARGSD(setalpha, "alpha : set alpha channel (0..255)");
  HH_ARGSD(getalpha, ": move alpha channel to RGB color channels");
  HH_ARGSD(matchtoalpha, ": set alpha channel based on whether pixel matches color");
  HH_ARGSC("", ":");
  HH_ARGSD(quadpullpush, ": replace matching color based on quadtree");
  HH_ARGSD(softpullpush, ": more fancy, based on bicubic filter");
  HH_ARGSD(voronoidilate, ": replace all matching pixels with closest non-matching color");
  HH_ARGSD(featureoffsets, ": replace with vector to closest nonmatching");
  HH_ARGSD(normalizenor, ": for normal map, normalize |(r, g, b)| to 1");
  HH_ARGSD(shadenor, "dx dy dz : shade normal map given light_dir");
  HH_ARGSD(shadefancy, "'frame' : shade using multiple lights");
  HH_ARGSD(cycle, "n : cycle through colors");
  HH_ARGSD(transf, "'frame' : post-multiply RGB vector by a matrix (ranges [0..1])");
  HH_ARGSD(genpattern, "pat : create procedural pattern ([xydr]h+[slqc])");
  HH_ARGSD(permutecolors, ": randomize the unique colors");
  HH_ARGSD(randomizeRGB, ": randomize the unique colors, across channels too");
  HH_ARGSD(noisegaussian, "sd : introduce white Gaussian noise (in range 0..255)");
  HH_ARGSD(blur, "r : apply Gaussian blurring with 1sdv = r pixels (e.g. 1.)");
  HH_ARGSC("", ":");
  HH_ARGSD(composite, "op_name float back_image : blend");
  HH_ARGSD(homogenize, "periods : remove low frequencies (typically 4) (-1 is bilinear)");
  HH_ARGSD(superresolution, "fac : magnify smartly");
  HH_ARGSD(istoroidal, ": set exit_state > 0 if image is nontoroidal");
  HH_ARGSD(gdtoroidal, ": use gradient-domain optimization to make toroidal");
  HH_ARGSD(gradientsharpen, "fac : apply gradient-based sharpening/attenuation (e.g. 1.5 or 0.7)");
  HH_ARGSD(gdfill, ": apply Poisson diffusion to alpha-masked regions");
  HH_ARGSD(contour, "gridres : output A3d contour at red-channel isovalue 127.5 (!gridres -> max(dims))");
  HH_ARGSD(mcontours, "gridres ncontours : output multiple contour curves");
  HH_ARGSC("", ":");
  HH_ARGSF(fixedbnd, ": rectangular boundary for poisson");
  HH_ARGSP(wconformal, "w : weight on conformality constraint (e.g. 1.f)");
  HH_ARGSP(gscale, "fac : scale on input image gradient (typically used only if unconstrained regions)");
  HH_ARGSD(niter, "n : number of iterations of linear Poisson");
  HH_ARGSD(poisson, ": output poisson warping mesh");
  HH_ARGSD(procedure, "name... : performed named operation");
  HH_ARGSD(diff, "image2 : compute difference 128 + image - image2");
  HH_ARGSD(maxdiff, "fthresh image2 : crash if image value differs by more than threshold (range 0..255)");
  HH_ARGSD(maxrmsdiff, "fthresh image2 : crash if rms image diff exceeds threshold (range 0..255)");
  HH_ARGSD(compare, "image2 : compute difference statistics");
  HH_ARGSD(resamplemesh, "mesh.m : resample image colors onto warped mesh");
  HH_ARGSC("", ":");
  HH_ARGSP(use_lab, "bool : use LAB rather than RGB");
  HH_ARGSD(pyramid, "imagef : structure transfer + CLB");
  HH_ARGSD(structuretransfer, "structureimg : inject detail");
  HH_ARGSC("", ":");
  HH_ARGSF(elevation, ": input is elevation map (instead of color)");
  HH_ARGSF(rg_elev, ": elevation is 16bit (R, G channels)");
  HH_ARGSP(scalezaxis, "fac : for elevation, scale pixel values");
  HH_ARGSP(offsetzaxis, "off : for elevation, offset after scaling");
  HH_ARGSF(removekinks, ":  smooth out quantized pixel values");
  HH_ARGSP(blocks, "s : block size");
  HH_ARGSP(step, "s : step size");
  HH_ARGSP(bxnum, "x : block number");
  HH_ARGSP(bynum, "y : block number");
  HH_ARGSF(quads, ": generate quads instead of triangles");
  HH_ARGSF(strip_order, ": different order for lru cache size 3");
  HH_ARGSF(best_diagonal, ": use shorter diagonal");
  HH_ARGSF(toggle_order, ": toggle triangle diagonal on each row");
  HH_ARGSD(tomesh, ": output mesh on stdout");
  HH_ARGSD(tofloats, "f.floats : output file of binary elevations");
  HH_ARGSD(tofmp, "f.fmp : output (X, Y, Z) binary floating-point");
  string arg0 = args.num() ? args.peek_string() : "";
  if (!ParseArgs::special_arg(arg0) && arg0 != "-nostdin" && arg0 != "-create" && !begins_with(arg0, "-as") &&
      arg0 != "-fromtxt" && arg0 != "-invideo") {
    string filename = "-";
    if (args.num() && (arg0 == "-" || arg0[0] != '-')) filename = args.get_filename();
    image.read_file(filename);
  }
  g_parseargs = &args;
  args.parse();
  hh_clean_up();
  if (!nooutput) image.write_file("-");
  return 0;
}
