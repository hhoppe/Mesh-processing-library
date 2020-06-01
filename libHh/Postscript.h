// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_POSTSCRIPT_H_
#define MESH_PROCESSING_LIBHH_POSTSCRIPT_H_

#include "Geometry.h"

namespace hh {

// Output Postscript graphics with lines and points.
class Postscript : noncopyable {
 public:
    explicit Postscript(std::ostream& os, int nxpix, int nypix) : _os(os), _nxpix(nxpix), _nypix(nypix) { init(); }
    ~Postscript()                               { clear(); }
    // All parameters x, y are in range [0, 1] in postscript coordinate system: (x=0, y=0) at left bottom.
    // Note that this is different from hps.c which had y reversed.
    void line(float x1p, float y1p, float x2p, float y2p)       { line_i(x1p, y1p, x2p, y2p); }
    void point(float xp, float yp)                              { point_i(xp, yp); }
    void edge_width(float w) {
        if (w==_curw) return;
        _curw = w;
        set_state(EState::undef); set_state(EState::line);
        _os << sform("wline%s setlinewidth\n", _curw==1.f ? "" : sform(" %g mul", _curw).c_str());
    }
    void flush_write(const string& s)           { set_state(EState::undef), _os << s; } // include linefeeds in s
 private:
    static constexpr int k_max = 15000;
    std::ostream& _os;
    int _nxpix;
    int _nypix;
    enum class EState { undef, point, line } _state {EState::undef}; // state for line width
    int _bbx0 {+INT_MAX};
    int _bbx1 {-INT_MAX};
    int _bby0 {+INT_MAX};
    int _bby1 {-INT_MAX};
    float _curw {0.f};          // current line width
    Frame _ctm;
    int _opx, _opy;             // old pen position if LINE
    //
    void init() {
        bool landscape = _nxpix>_nypix;
        // this format actually conforms to PS-Adobe-3.0
        _os << "%!PS-Adobe-2.0 EPSF-1.2\n";
        _os << "%%Orientation: " << (landscape ? "Landscape\n" : "Portrait\n");
        _os << "%%DocumentFonts:\n";
        _os << "%%Creator: hps\n";
        _os << "%%CreationDate: " << get_current_datetime() << "\n";
        _os << "%%LanguageLevel: 1\n";
        _os << "%%Pages: 1\n";
        // (10.5-7.8)/2*72+15==112 +7.8*72==674
        if (_nxpix==_nypix)
            _os << "%%BoundingBox: 20 112 582 674\n";
        else                        // not optimal?
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
        float rpx = !landscape ? k_px : k_py;
        float rpy = !landscape ? k_py : k_px;
        Frame f = (rpy/rpx>static_cast<float>(_nypix)/_nxpix
                   ? (Frame::translation(V(0.f, (rpy/rpx*_nxpix/_nypix-1.f)*k_max, 0.f))*
                      Frame::scaling(V(72.f*rpx/(k_max*2), 72.f*rpx/(k_max*2.f)*_nypix/_nxpix, 1.f)))
                   : (Frame::translation(V((rpx/rpy*_nypix/_nxpix-1.f)*k_max, 0.f, 0.f))*
                      Frame::scaling(V(72.f*rpy/(k_max*2)*_nxpix/_nypix, 72.f*rpy/(k_max*2), 1.f))));
        if (!landscape) {
            f = f*Frame::translation(V(20.f, 15.f, 0.f));
        } else {
            f = f*Frame::translation(V(15.f, 20.f, 0.f))*Frame::translation(V(0.f, -8.5f*72.f, 0.f))*
                Frame::rotation(2, TAU/4);
        }
        _os << sform("[%g %g  %g %g  %g %g] concat\n", f[0][0], f[0][1], f[1][0], f[1][1], f.p()[0], f.p()[1]);
        _ctm = f;
        _os << "%%EndPageSetup\n";
        _os << "% hps.c created from Postscript.h in libHh\n";
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
        _os << sform("  0 0 moveto 0 %d lineto %d %d lineto %d 0 lineto \n", k_max*2, k_max*2, k_max*2, k_max*2);
        _os << "  closepath stroke\n";
        _os << "} if\n\nnewpath\n";
    }
    void clear() {
        set_state(EState::undef);
        _os << "\npagelevel restore\nend\nshowpage\n\n%%Trailer\n";
        if (_bbx0>=_bbx1) { _bbx0 = 20;  _bbx1 = 582; }
        if (_bby0>=_bby1) { _bby0 = 112; _bby1 = 674; }
        _os << "%TightBoundingBox: " << sform("%d %d  %d %d\n", _bbx0, _bby0, _bbx1, _bby1);
    }
    // xp, yp in range 0..1; converted internally to x, y in range -1 to 1
    void line_i(float x1p, float y1p, float x2p, float y2p) {
        float x1 = x1p*2.f-1.f, y1 = y1p*2.f-1.f;
        float x2 = x2p*2.f-1.f, y2 = y2p*2.f-1.f;
        bool c1x = abs(x1)>1.f;
        bool c2x = abs(x2)>1.f;
        if (c1x && c2x && x1*x2>0.f) return;
        bool c1y = abs(y1)>1.f;
        bool c2y = abs(y2)>1.f;
        if (c1y && c2y && y1*y2>0.f) return;
        if (c1x || c2x || c1y || c2y) {
            float m = x1==x2 ? 1e+6f : y1==y2 ? 1e-6f : (y2-y1)/(x2-x1);
            float a;
            if (c1x) { a = sign(x1); y1 = (a-x1)*m+y1; x1 = a; }
            if (c2x) { a = sign(x2); y2 = (a-x2)*m+y2; x2 = a; }
            c1y = abs(y1)>1.f;
            c2y = abs(y2)>1.f;
            if (c1y && c2y && y1*y2>0.f) return;
            if (c1y) { a = sign(y1); x1 = (a-y1)*m+x1; y1 = a; }
            if (abs(x1)>1.f) return;
            if (c2y) { a = sign(y2); x2 = (a-y2)*m+x2; y2 = a; }
            if (abs(x2)>1.f) return;
        }
        int px1, py1; convert(x1, y1, px1, py1);
        int px2, py2; convert(x2, y2, px2, py2);
        if (_state==EState::line && _opx==px1 && _opy==py1) {
            _os << px2 << " " << py2 << " l\n";
            _opx = px2; _opy = py2;
            return;
        } else if (_state==EState::line && _opx==px2 && _opy==py2) {
            _os << px1 << " " << py1 << " l\n";
            _opx = px1; _opy = py1;
            return;
        }
        if (_state==EState::line)
            _os << "s\n";
        else
            set_state(EState::line);
        _os << px1 << " " << py1 << " m\n";
        _os << px2 << " " << py2 << " l\n";
        _opx = px2; _opy = py2;
    }
    void point_i(float xp, float yp) {
        float x = xp*2.f-1.f, y = yp*2.f-1.f;
        if (abs(x)>1.f || abs(y)>1.f) return;
        int px, py; convert(x, y, px, py);
        set_state(EState::point);
        _os << px << " " << py << " p\n";
    }
    void convert(float x, float y, int& px, int& py) {
        px = k_max+static_cast<int>(k_max*x+.5f);
        py = k_max+static_cast<int>(k_max*y+.5f);
        Point p = Point(static_cast<float>(px), static_cast<float>(py), 0.f)*_ctm;
        int ix = static_cast<int>(p[0]+.5f), iy = static_cast<int>(p[1]+.5f);
        _bbx0 = min(_bbx0, ix); _bbx1 = max(_bbx1, ix);
        _bby0 = min(_bby0, iy); _bby1 = max(_bby1, iy);
    }
    void set_state(EState state) {
        if (_state==state) return;
        if (_state==EState::line) _os << "s\n";
        _state = state;
        if (_state==EState::line) _os << "wline setlinewidth\n";
        if (_state==EState::point) _os << "wpoint setlinewidth\n";
        if (_state==EState::line) _opx = -1;
    }
};

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_POSTSCRIPT_H_
