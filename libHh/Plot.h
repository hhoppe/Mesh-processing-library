// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PLOT_H_
#define MESH_PROCESSING_LIBHH_PLOT_H_

#include "libHh/Geometry.h"

namespace hh {

// Output of a 2D line drawing (lines and points) as vector graphics, in Postscript or SVG.
class Plot : noncopyable {
 public:
  virtual ~Plot() = default;
  // All parameters x, y are in the range [0, 1], with (x = 0, y = 0) at the lower-left corner.
  virtual void line(float x1, float y1, float x2, float y2) = 0;
  virtual void point(float x, float y) = 0;
  virtual void edge_width(float w) = 0;  // Relative to the default width 1.
  virtual void comment(const string& s) = 0;

 protected:
  // Clip the segment to the square [-1, 1]^2, returning false if nothing of it remains.
  static bool clip_line(float& x1, float& y1, float& x2, float& y2) {
    const bool c1x = abs(x1) > 1.f;
    const bool c2x = abs(x2) > 1.f;
    if (c1x && c2x && x1 * x2 > 0.f) return false;
    bool c1y = abs(y1) > 1.f;
    bool c2y = abs(y2) > 1.f;
    if (c1y && c2y && y1 * y2 > 0.f) return false;
    if (c1x || c2x || c1y || c2y) {
      const float m = x1 == x2 ? 1e6f : y1 == y2 ? 1e-6f : (y2 - y1) / (x2 - x1);
      float a;
      if (c1x) {
        a = sign(x1);
        y1 = (a - x1) * m + y1;
        x1 = a;
      }
      if (c2x) {
        a = sign(x2);
        y2 = (a - x2) * m + y2;
        x2 = a;
      }
      c1y = abs(y1) > 1.f;
      c2y = abs(y2) > 1.f;
      if (c1y && c2y && y1 * y2 > 0.f) return false;
      if (c1y) {
        a = sign(y1);
        x1 = (a - y1) / m + x1;
        y1 = a;
      }
      if (abs(x1) > 1.f) return false;
      if (c2y) {
        a = sign(y2);
        x2 = (a - y2) / m + x2;
        y2 = a;
      }
      if (abs(x2) > 1.f) return false;
    }
    return true;
  }
};

// Output Postscript graphics with lines and points.
class PostscriptPlot : public Plot {
 public:
  explicit PostscriptPlot(std::ostream& os, int nxpix, int nypix) : _os(os), _nxpix(nxpix), _nypix(nypix) { init(); }
  ~PostscriptPlot() override { clear(); }
  // All parameters x, y are in range [0, 1] in postscript coordinate system: (x = 0, y = 0) at left bottom.
  // Note that this is different from hps.c which had y reversed.
  void line(float x1p, float y1p, float x2p, float y2p) override { line_i(x1p, y1p, x2p, y2p); }
  void point(float xp, float yp) override { point_i(xp, yp); }
  void edge_width(float w) override {
    if (w == _curw) return;
    _curw = w;
    set_state(EState::undef);
    set_state(EState::line);
    _os << sform("wline%s setlinewidth\n", _curw == 1.f ? "" : sform(" %g mul", _curw).c_str());
  }
  void comment(const string& s) override { set_state(EState::undef), _os << "% " << s << "\n"; }

 private:
  static constexpr int k_max = 15000;
  std::ostream& _os;
  int _nxpix;
  int _nypix;
  enum class EState { undef, point, line } _state{EState::undef};  // State for the line width.
  int _bbx0{std::numeric_limits<int>::max()};
  int _bbx1{std::numeric_limits<int>::min()};
  int _bby0{std::numeric_limits<int>::max()};
  int _bby1{std::numeric_limits<int>::min()};
  float _curw{0.f};  // The current line width.
  Frame _ctm;
  int _opx, _opy;  // The old pen position if LINE.

  void init() {
    const bool landscape = _nxpix > _nypix;
    // This format actually conforms to PS-Adobe-3.0.
    _os << "%!PS-Adobe-2.0 EPSF-1.2\n";
    _os << "%%Orientation: " << (landscape ? "Landscape\n" : "Portrait\n");
    _os << "%%DocumentFonts:\n";
    _os << "%%Creator: hps\n";
    _os << "%%CreationDate: " << get_current_datetime() << "\n";
    _os << "%%LanguageLevel: 1\n";
    _os << "%%Pages: 1\n";
    // (10.5 - 7.8) / 2 * 72 + 15 == 112 + 7.8 * 72 == 674
    if (_nxpix == _nypix)
      _os << "%%BoundingBox: 20 112 582 674\n";
    else  // Not optimal?
      _os << "%%BoundingBox: 20 15 582 771\n";
    _os << "%%EndComments\n";
    _os << "%%BeginPreview: 16 16 1 16\n";
    _os << "% FFFF\n% 8001\n% 8001\n% 8001\n";
    _os << "% 8001\n% 8001\n% 8001\n% 8001\n";
    _os << "% 8001\n% 8001\n% 8001\n% 8001\n";
    _os << "% 8001\n% 8001\n% 8001\n% FFFF\n";
    _os << "%%EndPreview\n";
    // %%BeginDefaults, %%EndDefaults
    _os << "%%BeginProlog\n";
    _os << "%%EndProlog\n";
    // %%BeginSetup, %%EndSetup
    _os << "%%Page: 1 1\n";
    _os << "%%BeginPageSetup\n";
    _os << "15 dict begin\n/pagelevel save def\n";
    _os << "% Setup CTM\n";
    constexpr float k_px = 7.8f;
    constexpr float k_py = 10.5f;
    const float rpx = !landscape ? k_px : k_py;
    const float rpy = !landscape ? k_py : k_px;
    Frame frame =
        (rpy / rpx > float(_nypix) / _nxpix
             ? (Frame::translation(V(0.f, (rpy / rpx * _nxpix / _nypix - 1.f) * k_max, 0.f)) *
                Frame::scaling(V(72.f * rpx / (k_max * 2), 72.f * rpx / (k_max * 2.f) * _nypix / _nxpix, 1.f)))
             : (Frame::translation(V((rpx / rpy * _nypix / _nxpix - 1.f) * k_max, 0.f, 0.f)) *
                Frame::scaling(V(72.f * rpy / (k_max * 2) * _nxpix / _nypix, 72.f * rpy / (k_max * 2), 1.f))));
    if (!landscape) {
      frame = frame * Frame::translation(V(20.f, 15.f, 0.f));
    } else {
      frame = frame * Frame::translation(V(15.f, 20.f, 0.f)) * Frame::translation(V(0.f, -8.5f * 72.f, 0.f)) *
              Frame::rotation(2, TAU / 4);
    }
    _os << sform("[%g %g  %g %g  %g %g] concat\n",  //
                 frame[0, 0], frame[0, 1], frame[1, 0], frame[1, 1], frame.p()[0], frame.p()[1]);
    _ctm = frame;
    _os << "%%EndPageSetup\n";
    _os << "% Created from class PostscriptPlot in libHh\n";
    _os << "% Initialize procedures\n";
    _os << "/m { moveto } bind def\n";
    _os << "/l { lineto } bind def\n";
    _os << "/s { stroke } bind def\n";
    // _os << "/p { 2 copy m l s } bind def\n";
    _os << "/p { wpoint 0.5 mul  0 360 arc  fill } bind def\n";
    _os << "\n% Setup variables\n";
    _os << "/wborder 20 def\n";
    _os << "/wline 20 def\n";
    _os << "/wpoint 80 def\n";
    _os << "/drawborder false def\n";
    _os << "\n% Begin drawing\n";
    _os << "newpath\n1 setlinecap\n1 setlinejoin\n";
    _os << "drawborder {\n  wborder setlinewidth\n";
    _os << sform("  0 0 moveto 0 %d lineto %d %d lineto %d 0 lineto \n", k_max * 2, k_max * 2, k_max * 2, k_max * 2);
    _os << "  closepath stroke\n";
    _os << "} if\n\nnewpath\n";
  }
  void clear() {
    set_state(EState::undef);
    _os << "\npagelevel restore\nend\nshowpage\n\n%%Trailer\n";
    if (_bbx0 >= _bbx1) {
      _bbx0 = 20;
      _bbx1 = 582;
    }
    if (_bby0 >= _bby1) {
      _bby0 = 112;
      _bby1 = 674;
    }
    _os << "%TightBoundingBox: " << sform("%d %d  %d %d\n", _bbx0, _bby0, _bbx1, _bby1);
  }
  // Here, xp, yp are in the range 0..1, converted internally to x, y in the range -1 to 1.
  void line_i(float x1p, float y1p, float x2p, float y2p) {
    float x1 = x1p * 2.f - 1.f, y1 = y1p * 2.f - 1.f;
    float x2 = x2p * 2.f - 1.f, y2 = y2p * 2.f - 1.f;
    if (!clip_line(x1, y1, x2, y2)) return;
    int px1, py1;
    convert(x1, y1, px1, py1);
    int px2, py2;
    convert(x2, y2, px2, py2);
    if (_state == EState::line && _opx == px1 && _opy == py1) {
      _os << px2 << " " << py2 << " l\n";
      _opx = px2;
      _opy = py2;
      return;
    } else if (_state == EState::line && _opx == px2 && _opy == py2) {
      _os << px1 << " " << py1 << " l\n";
      _opx = px1;
      _opy = py1;
      return;
    }
    if (_state == EState::line)
      _os << "s\n";
    else
      set_state(EState::line);
    _os << px1 << " " << py1 << " m\n";
    _os << px2 << " " << py2 << " l\n";
    _opx = px2;
    _opy = py2;
  }
  void point_i(float xp, float yp) {
    const float x = xp * 2.f - 1.f, y = yp * 2.f - 1.f;
    if (abs(x) > 1.f || abs(y) > 1.f) return;
    int px, py;
    convert(x, y, px, py);
    set_state(EState::point);
    _os << px << " " << py << " p\n";
  }
  void convert(float x, float y, int& px, int& py) {
    px = k_max + int(std::floor(k_max * x + .5f));
    py = k_max + int(std::floor(k_max * y + .5f));
    Point p = Point(float(px), float(py), 0.f) * _ctm;
    const int ix = int(p[0] + .5f), iy = int(p[1] + .5f);
    _bbx0 = min(_bbx0, ix);
    _bbx1 = max(_bbx1, ix);
    _bby0 = min(_bby0, iy);
    _bby1 = max(_bby1, iy);
  }
  void set_state(EState state) {
    if (_state == state) return;
    if (_state == EState::line) _os << "s\n";
    _state = state;
    if (_state == EState::line) _os << "wline setlinewidth\n";
    if (_state == EState::point) _os << "wpoint setlinewidth\n";
    if (_state == EState::line) _opx = -1;
  }
};

// Output SVG (Scalable Vector Graphics) with lines and points.
class SvgPlot : public Plot {
 public:
  // The image has nxpix * nypix pixel units; lines have width 1 (times edge_width()) and points diameter 4.
  explicit SvgPlot(std::ostream& os, int nxpix, int nypix) : _os(os), _nxpix(nxpix), _nypix(nypix) { init(); }
  ~SvgPlot() override { clear(); }
  // All parameters x, y are in the range [0, 1], with (x = 0, y = 0) at the lower-left corner.
  void line(float x1p, float y1p, float x2p, float y2p) override {
    float x1 = x1p * 2.f - 1.f, y1 = y1p * 2.f - 1.f;
    float x2 = x2p * 2.f - 1.f, y2 = y2p * 2.f - 1.f;
    if (!clip_line(x1, y1, x2, y2)) return;
    const Vec2<float> p1 = convert(x1, y1), p2 = convert(x2, y2);
    // Continue the current polyline if the segment starts (or ends) at its last vertex.
    if (_in_path && p1 == _last) {
      _os << " L " << coords(p2);
      _last = p2;
    } else if (_in_path && p2 == _last) {
      _os << " L " << coords(p1);
      _last = p1;
    } else {
      end_path();
      _os << "<path" << (_curw == 1.f ? string() : sform(" stroke-width=\"%g\"", _wline * _curw)) << " d=\"M "
          << coords(p1) << " L " << coords(p2);
      _in_path = true;
      _last = p2;
    }
  }
  void point(float xp, float yp) override {
    const float x = xp * 2.f - 1.f, y = yp * 2.f - 1.f;
    if (abs(x) > 1.f || abs(y) > 1.f) return;
    end_path();
    const Vec2<float> p = convert(x, y);
    _os << sform("<circle cx=\"%.2f\" cy=\"%.2f\" r=\"%g\" fill=\"black\" stroke=\"none\"/>\n", p[0], p[1],
                 _wpoint / 2);
  }
  void edge_width(float w) override {
    if (w == _curw) return;
    end_path();
    _curw = w;
  }
  void comment(const string& s) override { end_path(), _os << "<!-- " << s << " -->\n"; }

 private:
  std::ostream& _os;
  int _nxpix;
  int _nypix;
  static constexpr float _wline = 1.f;   // The default line width, in pixel units.
  static constexpr float _wpoint = 4.f;  // The diameter of points, in pixel units.
  float _curw{1.f};                      // The current line width, relative to _wline.
  bool _in_path{false};
  Vec2<float> _last;  // The last vertex of the current polyline if _in_path.

  void init() {
    _os << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    _os << sform("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"%d\" height=\"%d\" viewBox=\"0 0 %d %d\">\n",
                 _nxpix, _nypix, _nxpix, _nypix);
    _os << "<!-- Created from class SvgPlot in libHh -->\n";
    _os << sform("<rect width=\"%d\" height=\"%d\" fill=\"white\"/>\n", _nxpix, _nypix);
    _os << sform(
        "<g fill=\"none\" stroke=\"black\" stroke-width=\"%g\" stroke-linecap=\"round\""
        " stroke-linejoin=\"round\">\n",
        _wline);
  }
  void clear() {
    end_path();
    _os << "</g>\n</svg>\n";
  }
  // Convert from the range [-1, 1]^2 to pixel units, with y pointing down.
  Vec2<float> convert(float x, float y) const { return V((x + 1.f) * .5f * _nxpix, (1.f - y) * .5f * _nypix); }
  static string coords(const Vec2<float>& p) { return sform("%.2f %.2f", p[0], p[1]); }
  void end_path() {
    if (!_in_path) return;
    _os << "\"/>\n";
    _in_path = false;
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_PLOT_H_
