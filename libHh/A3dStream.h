// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Geometry.h"
#include "PArray.h"
#include "Pixel.h"

namespace hh {

// A3dStream is a stream of geometric primitives (points, polylines, polygons) and
//  directives (comments, endobject, endframe, endfile, editobject).
// Each geometric primitive contains an array of vertices (1 for point, >=2 for polyline and >=3 for polygon),
//  where each vertex has position, normal, and color.
// Color consists of diffuse, specular, and Phong components.
// An A3dStream can be read/written from a std::stream using RSA3dStream/WSA3dStream or
//  from a Buffer using BufferedA3dStream.h .
// In either case, the format can be text or binary.

struct A3dColor : Vec3<float> {
    A3dColor()                                  = default;
    A3dColor(float r, float g, float b)         : Vec3<float>(r, g, b) { }
    A3dColor(Vec3<float> a)                     : Vec3<float>(a) { }
};

struct A3dVertexColor {
    A3dVertexColor()                            = default;
    A3dVertexColor(A3dColor pd)                 : d(pd), s(1.f, 1.f, 1.f), g(1.f, 0.f, 0.f) { }
    A3dVertexColor(const Pixel& pix)    : A3dVertexColor(A3dColor(pix[0]/255.f, pix[1]/255.f, pix[2]/255.f)) { }
    A3dVertexColor(A3dColor pd, A3dColor ps, A3dColor pg) : d(pd), s(ps), g(pg) { }
    A3dColor d;                 // diffuse
    A3dColor s;                 // specular
    A3dColor g;                 // g[0] is Phong coefficient; other channels unused
};

struct A3dVertex {
    A3dVertex()                                 = default;
    A3dVertex(Point pp, Vector pn, A3dVertexColor pc) : p(pp), n(pn), c(pc) { }
    Point p;
    Vector n;
    A3dVertexColor c;
};

class Polygon;

// I thought about making this a base class for different types of elements,
//  but thought it would be too inefficient to allocate/deallocate every time.
class A3dElem {
 public:
    enum class EType {
        polygon = 'P', polyline = 'L', point = 'p',
        comment = '#', endobject = 'o', endframe = 'f', endfile = 'q', editobject = 'O'
    };
    // also reserved: 'v', 'E', 'n', 'd', 's', 'g'
    A3dElem()                                           = default;
    explicit A3dElem(EType type, bool binary = false, int nv = 0) { init(type, binary, nv); } // allocates AND init()
    // both A3dElem(..nv) and init() allocate and initialize for nv
    void init(EType type, bool binary = false, int nv = 0);
    void update(EType type, bool binary = false); // polygon<>polyline<>point
    EType type() const                                   { return _type; }
    void set_binary(bool b)                             { _binary = b; }
    bool binary() const                                 { return _binary; }
    static bool status_type(EType type)                 { char ch = char(type); return ch=='d' || ch=='s' || ch=='g'; }
    static bool command_type(EType type)                { return command_type_i(type); }
// for EType::polygon || EType::polyline || EType::point:
    int num() const                                     { return _v.num(); }
    size_t size() const                                 { return _v.size(); }
    void push(const A3dVertex& vertex)                  { push_i(vertex); }
    A3dVertex& operator[](int i)                        { return _v[i]; }
    const A3dVertex& operator[](int i) const            { return _v[i]; }
// for EType::polygon
    Vector pnormal() const;     // may be degenerate (zero)!
    void get_polygon(Polygon& poly) const;
// for TComment:
    void set_comment(string str); // str may start with ' '
    const string& comment() const;
// for command_type():
    Vec3<float>& f()                                    { assertx(command_type(_type)); return _f; }
    const Vec3<float>& f() const                        { assertx(command_type(_type)); return _f; }
 private:
    EType _type {EType::polygon};
    bool _binary {false};
    PArray<A3dVertex,8> _v;     // for EType::polygon, EType::polyline, EType::point
    string _comment;            // for EType::comment
    Vec3<float> _f;             // for comamnd_type()
    static bool command_type_i(EType type) {
        return type==EType::endobject || type==EType::endframe || type==EType::endfile || type==EType::editobject;
    }
    void push_i(const A3dVertex& vertex) {
        assertx(_type==EType::polygon || _type==EType::polyline || _type==EType::point);
        if (_type==EType::point) assertx(!num());
        _v.push(vertex);
    }
    friend std::ostream& operator<<(std::ostream& os, const A3dElem& el) {
        os << "A3dElem{ type='" << assert_narrow_cast<char>(el.type()) << "' binary=" << el.binary();
        switch (el.type()) {
         bcase A3dElem::EType::polygon: ocase A3dElem::EType::polyline: ocase A3dElem::EType::point:
            if (el.type()==A3dElem::EType::polygon)
                os << " pnormal=" << el.pnormal();
            os << " num=" << el.num() << " {\n";
            for_int(i, el.num()) {
                os << "  [" << i << "] = {p=" << el[i].p << ", d=" << el[i].c.d << ", s=" <<
                    el[i].c.s << ", g=" << el[i].c.g << ", n=" << el[i].n << "}\n";;
            }
         bcase A3dElem::EType::comment:
            os << " comment='" << el.comment() << "'";
         bdefault:
            os << " f=" << el.f();
        }
        return os << "}\n";
    }
};

class A3dStream : noncopyable {
 protected:
    virtual ~A3dStream()                        { } // not =default because gcc "looser throw specifier" in derived
    A3dVertexColor curcolor() const             { return _curcol; }
    void set_current_color(char ctype, const Vec3<float>& f);
    static constexpr int k_binary_code = 3;
 protected:
    A3dVertexColor _curcol {A3dColor(0.f, 0.f, 0.f), A3dColor(0.f, 0.f, 0.f), A3dColor(0.f, 0.f, 0.f)};
};


class RA3dStream : public A3dStream {
 public:
    void read(A3dElem& el);
 protected:
    RA3dStream()                                = default;
    virtual bool read_line(bool& binary, char& type, Vec3<float>& f, string& comment) = 0; // ret success
};

class RSA3dStream : public RA3dStream { // Read from stream
 public:
    explicit RSA3dStream(std::istream& pis)     : _is(pis) { }
    std::istream& is()                          { return _is; }
 protected:
    bool read_line(bool& binary, char& type, Vec3<float>& f, string& comment) override;
 private:
    std::istream& _is;
};


class WA3dStream : public A3dStream {
 public:
    void write(const A3dElem& el);
    void write_comment(const string& str); // can contain newlines
    void write_end_object(bool binary = false, float f0 = 1.f, float f1 = 1.f);
    void write_clear_object(bool binary = false, float f0 = 1.f, float f1 = 0.f);
    void write_end_frame(bool binary = false);
    virtual void flush() = 0;
 protected:
    WA3dStream()                                = default;
    bool _oldformat;            // old a3d format
    virtual void output(bool binary, char type, const Vec3<float>& f) = 0;
    virtual void output_comment(const string& s) = 0;
    virtual void blank_line() = 0;
 private:
    bool _first {true};         // first write
    bool _force_choice_binary;  // force choice one way or the other
    bool _choice_binary;        // which way is forced
    bool _pblank {false};       // previous element left a blank line
    void write_old_format(const A3dElem& el);
};

class WSA3dStream : public WA3dStream { // Write to stream
 public:
    explicit WSA3dStream(std::ostream& pos)     : _os(pos) { }
    ~WSA3dStream()                              { flush(); }
    void flush() override                       { _os.flush(); }
    std::ostream& os()                          { return _os; }
 protected:
    void output(bool binary, char type, const Vec3<float>& f) override;
    void output_comment(const string& s) override;
    void blank_line() override;
 private:
    std::ostream& _os;
};


//----------------------------------------------------------------------------
//
// NAME
//   A3d - new ascii 3D file format
//
// DESCRIPTION
//   The format is a list of records.  Each record contains a character code
//   and three floating point numbers.
//   Here is a description:
//
//    # Comments begin with a '#' in the first column.
//
//    # Geometric primitives
//    #  polygon
//    P 0 0 0
//    v x1 y1 z1
//    v x2 y2 z2
//    ...
//    E 0 0 0
//    #  polyline
//    L 0 0 0
//    v x1 y1 z1
//    v x2 y2 z2
//    ...
//    E 0 0 0
//    #  point
//    p x y z
//
//    # State information
//    #  diffuse color  (r, g, b between 0. and 1.)
//    d r g b
//    #  specular color
//    s r g b
//    #  phong factor
//    g phong 0 0
//    #  normal  (should be normalized)
//    n vx vy vz
//    # The color state is permanent
//    #   (except the colors are reset to zero at End{Object,Frame,File})
//    # The normal should precede a (v)ertex or a (p)oint and applies only to
//    #  that vertex/point.  (lack of normal is equivalent to zero normal)
//
//    # Special commands (for advanced use)
//    #  EndObject: ends current object and selects a new object number
//    #  (works like seek(2): int_code=0 -> beg, int_code=1 -> relative, ...)
//    #  (ex.: 'o 1 1 0' select next object, 'o 0 3 0' select object 3)
//    o int_code int_disp 0
//    #  EndFrame (used by viewer to determine when to update screen)
//    f 0 0 0
//    #  EndFile (the remainder of the stream is no longer a3d input)
//    q 0 0 0
//    #  EditObject: clears an object (same format as EndObject)
//    O int_code int_disp 0
//    # Geometric primitives are always appended to the current object.
//    # Use EndObject to select a different object, and EditObject to clear an
//    # object before replacing its definition.

} // namespace hh
