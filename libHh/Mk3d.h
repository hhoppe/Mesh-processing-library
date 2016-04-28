// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "A3dStream.h"
#include "Stack.h"

namespace hh {

// Create a stream of polygons and polylines on a WA3dStream, with nested geometric transforms and color transforms.
class Mk3d : noncopyable {
 public:
    explicit Mk3d(WA3dStream& pos);
    ~Mk3d();
    WA3dStream& oa3d()                          { return _os; }
    void push()                                 { _stack_frame.push(_ctm); _stack_framei.push(_ctmi); }
    void pop()                                  {_ctm = _stack_frame.pop(); _ctmi = _stack_framei.pop();}
    template<typename Func> void save(Func func) { push(); func(); pop(); }
    void translate(float x, float y, float z)   { translate(Vector(x, y, z)); }
    void translate(const Vector& v)             { apply(Frame::translation(v)); }
    enum { Xaxis = 0, Yaxis = 1, Zaxis = 2 }; // (Mk3d::Xaxis is easier than Mk3d::Axis::x)
    void rotate(int axis, float angle)          { apply(Frame::rotation(axis, angle)); }
    void scale(float x, float y, float z);
    void scale(float v)                         { scale(v, v, v); }
    void apply(const Frame& t)                  { _ctm = t*_ctm; _ctmi = inverse(_ctm); }
    Point transform(const Point& point)         { return point*_ctm; }
    Vector transform(const Vector& normal)      { return _ctmi*normal; }
    void push_color()                           { _stack_color.push(_cc); }
    void pop_color()                            { assertx(!_stack_color.empty()); _cc = _stack_color.pop(); }
    template<typename Func> void save_color(Func func) { push_color(); func(); pop_color(); }
    void diffuse(float r, float g, float b)     { _cc.d = A3dColor(r, g, b); }
    void diffuse(const A3dColor& col)           { _cc.d = col; }
    void specular(float r, float g, float b)    { _cc.s = A3dColor(r, g, b); }
    void specular(const A3dColor& col)          { _cc.s = col; }
    void phong(float p)                         { _cc.g = A3dColor(p, 0.f, 0.f); }
    void color(const A3dVertexColor& color)     { _cc = color; }
    void scale_color(float sr, float sg, float sb);
    void point(float x, float y, float z)       { point(Point(x, y, z)); }
    void point(const Point& p)                  { _el.push(A3dVertex(transform(p), Vector(0.f, 0.f, 0.f), _cc)); }
    void normal(float x, float y, float z)      { normal(Vector(x, y, z)); }
    void normal(const Vector& normal);
    void begin_force_polyline(bool b)           { _stack_force_polyline.push(_force_polyline); _force_polyline = b; }
    void end_force_polyline()                   { _force_polyline = _stack_force_polyline.pop(); }
    void begin_force_flip(bool b)               { _stack_force_flip.push(_force_flip); _force_flip = b; }
    void end_force_flip()                       { _force_flip = _stack_force_flip.pop(); }
    void end_polygon();
    void end_2polygon();        // two-sided polygon
    void end_polyline();
    void end_point();
 private:
    WA3dStream& _os;
    Stack<Frame> _stack_frame;
    Stack<Frame> _stack_framei;
    Frame _ctm {Frame::identity()};
    Frame _ctmi {Frame::identity()};
    Stack<A3dVertexColor> _stack_color;
    A3dVertexColor _cc {A3dColor(1.f, .16f, 0.f), A3dColor(1.f, .5f, .3f), A3dColor(3.f, 0.f, 0.f)};
    Stack<bool> _stack_force_polyline;
    Stack<bool> _stack_force_flip;
    bool _force_polyline {false};
    bool _force_flip {false};
    A3dElem _el;
    int _max_vertices {0};
    int _total_vertices {0};
    int _total_polygons {0};
    int _total_polylines {0};
    int _total_points {0};
    void output_poly();
    void flip_poly();
};

struct MkSave {                 // save and restore the current geometric transform
    MkSave(Mk3d& mk)                            : _mk(mk) { _mk.push(); }
    ~MkSave()                                   { _mk.pop(); }
    Mk3d& _mk;
};

struct MkSaveColor {            // save and restore the current color transform
    MkSaveColor(Mk3d& mk)                       : _mk(mk) { _mk.push_color(); }
    ~MkSaveColor()                              { _mk.pop_color(); }
    Mk3d& _mk;
};

#define mk_save hh::MkSave HH_UNIQUE_ID(mksave)(mk)
#define mk_save_color hh::MkSaveColor HH_UNIQUE_ID(mksavecolor)(mk)

} // namespace hh
