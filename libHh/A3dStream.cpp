// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/A3dStream.h"

#include "libHh/BinaryIO.h"
#include "libHh/NetworkOrder.h"
#include "libHh/Polygon.h"
#include "libHh/RangeOp.h"
#include "libHh/StringOp.h"
#include "libHh/Vec.h"

namespace hh {

namespace {

const A3dColor k_color_undefined{-1.f, 0.f, 0.f};

struct a3d_binary_buf {
  Vec2<char> magic;
  ushort utype;
  Vec3<float> f;
};

}  // namespace

// *** A3dElem

void A3dElem::init(EType type, bool binary, int nv) {
  _type = type;
  _binary = binary;
  switch (_type) {
    case EType::polygon:
    case EType::polyline:
    case EType::point: _v.init(nv); break;
    case EType::comment: _comment = ""; break;
    case EType::editobject:
    case EType::endfile:
    case EType::endframe:
    case EType::endobject: fill(_f, 0.f); break;
    default: assertnever("");
  }
}

void A3dElem::update(EType type, bool binary) {
  assertx(_type == EType::polygon || _type == EType::polyline || _type == EType::point);
  _type = type;
  assertx(_type == EType::polygon || _type == EType::polyline || _type == EType::point);
  _binary = binary;
  if (_type == EType::point) assertx(num() <= 1);
}

void A3dElem::set_comment(string str) {
  assertx(_type == EType::comment);
  _comment = std::move(str);
}

const string& A3dElem::comment() const {
  assertx(_type == EType::comment);
  return _comment;
}

Vector A3dElem::pnormal() const {
  Vector vt{};
  assertx(_type == EType::polygon && num() >= 3);
  for_intL(i, 1, num() - 1) vt += cross(_v[0].p, _v[i].p, _v[i + 1].p);
  vt.normalize();
  return vt;
}

void A3dElem::get_polygon(Polygon& poly) const {
  assertx(_type == EType::polygon);
  poly.init(num());
  for_int(i, num()) poly[i] = (*this)[i].p;
}

// *** RA3dStream

void RA3dStream::read(A3dElem& el) {
  Vector normal(0.f, 0.f, 0.f);
  string commentstr;
  for (;;) {
    char ctype;
    bool binary;
    Vec3<float> f;
    bool i = read_line(binary, ctype, f, commentstr);
    if (!i) {
      binary = false;
      ctype = char(A3dElem::EType::endfile);
      fill(f, 0.f);
    }
    A3dElem::EType type = A3dElem::EType(ctype);
    if (ctype == 'n') {
      assertx(is_zero(normal));
      normal = Vector(f);
      continue;
    } else if (A3dElem::status_type(type)) {
      set_current_color(ctype, f);
      continue;
    }
    el.init(type, binary);
    if (type == A3dElem::EType::comment) {
      assertx(is_zero(normal));
      el.set_comment(commentstr);
      return;
    } else if (A3dElem::command_type(type)) {
      assertx(is_zero(normal));
      el.f() = f;
      return;
    } else if (type == A3dElem::EType::point) {
      el.push(A3dVertex(Point(f[0], f[1], f[2]), normal, _curcol));
      normal = Vector(0.f, 0.f, 0.f);
      return;
    } else if (type == A3dElem::EType::polygon || type == A3dElem::EType::polyline) {
      break;
    } else {
      assertnever(string("RA3dStream: type '") + ctype + "' unknown");
    }
  }
  assertx(is_zero(normal));
  int nv = 0;
  for (;;) {
    char ctype;
    bool binary;
    Vec3<float> f;
    bool i = read_line(binary, ctype, f, commentstr);
    if (!i) assertnever("RA3dStream: EOF within poly");
    A3dElem::EType type = A3dElem::EType(ctype);
    if (ctype == 'E') {
      break;
    } else if (ctype == 'n') {
      assertx(is_zero(normal));
      normal = Vector(f);
    } else if (ctype == 'v') {
      el.push(A3dVertex(Point(f), normal, _curcol));
      normal = Vector(0.f, 0.f, 0.f);
      nv++;
    } else if (A3dElem::status_type(type)) {
      set_current_color(ctype, f);
    } else {
      assertnever(string("RA3dStream: type '") + ctype + "' unexpected within polygon");
    }
  }
  assertx(is_zero(normal));
  switch (el.type()) {
    case A3dElem::EType::polygon: assertx(nv >= 3); break;
    case A3dElem::EType::polyline: assertx(nv >= 2); break;
    default: assertnever("");
  }
}

void RA3dStream::set_current_color(char ctype, const Vec3<float>& f) {
  switch (ctype) {
    case 'd': _curcol.d = A3dColor(f); break;
    case 's': _curcol.s = A3dColor(f); break;
    case 'g': _curcol.g = A3dColor(f); break;
    default: assertnever("");
  }
}

// *** RSA3dStream

bool RSA3dStream::read_line(bool& binary, char& ctype, Vec3<float>& f, string& comment) {
  // _is >> std::ws;  // commented 2012-12-11
  char ch;
  if (_is.peek() == '\n') _is.get(ch);  // there may be a blank line between elements
  int vpeek = _is.peek();
  if (vpeek < 0) return false;
  assertx(_is);
  ch = static_cast<char>(vpeek);
  binary = ch == k_a3d_binary_code;
  if (binary) {
    a3d_binary_buf buf;
    assertx(read_binary_raw(_is, ArView(buf)));
    assertx(buf.magic[1] == 0);
    from_std(&buf.utype);
    ctype = narrow_cast<char>(buf.utype);
    for_int(i, 3) {
      from_std(&buf.f[i]);
      f[i] = buf.f[i];
    }
    return true;
  }
  const string possible_types = "PLp#ofqOvEndsg";
  if (!contains(possible_types, ch)) assertnever(sform("read error on character '%c' (int %d)", ch, int(ch)));
  ctype = ch;
  _is.ignore();
  if (A3dElem::EType(ctype) == A3dElem::EType::comment) {
    assertx(my_getline(_is, comment));
  } else {
    assertx(_is.peek() == ' ');
    if (1) {
      string line;
      assertx(my_getline(_is, line));
      const char* s = line.c_str();
      for_int(c, 3) f[c] = float_from_chars(s);
      assert_no_more_chars(s);
    } else {
      assertx(_is >> f[0] >> f[1] >> f[2]);
      _is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // likely needed
      assertx(_is);
    }
  }
  return true;
}

// *** WA3dStream

void WA3dStream::write(const A3dElem& el) {
  bool binary = el.binary();
  A3dElem::EType type = el.type();
  if (_first) {
    _first = false;
    _curcol.d = _curcol.s = _curcol.g = k_color_undefined;
    _force_choice_binary = !!getenv("A3D_BINARY");  // dynamically updated by my_setenv() in Filtera3d
    if (_force_choice_binary) _choice_binary = getenv_bool("A3D_BINARY");
    write_comment(" Created by WA3dStream on " + get_current_datetime());
  }
  if (_force_choice_binary) binary = _choice_binary;
  if (type == A3dElem::EType::polygon || type == A3dElem::EType::polyline || type == A3dElem::EType::point) {
    // common case, skip for now
  } else if (A3dElem::command_type(type)) {
    if (!binary && !_pblank) blank_line();
    output(binary, static_cast<char>(type), el.f());
    if (!binary) {
      blank_line();
      _pblank = true;
    }
    _curcol.d = _curcol.s = _curcol.g = k_color_undefined;
    flush();
    return;
  } else if (type == A3dElem::EType::comment) {
    output_comment(el.comment());
    _pblank = false;
    return;
  } else {
    assertnever(sform("Unrecognized type '%c'", static_cast<char>(type)));
  }
  if (type == A3dElem::EType::polygon)
    assertx(el.num() >= 3);
  else if (type == A3dElem::EType::polyline)
    assertx(el.num() >= 2);
  else if (type == A3dElem::EType::point)
    assertx(el.num() == 1);
  if (type == A3dElem::EType::polygon || type == A3dElem::EType::polyline) {
    if (!binary && !_pblank) blank_line();
    output(binary, static_cast<char>(type), V(0.f, 0.f, 0.f));
  }
  for_int(i, el.num()) {
    if (_curcol.d != el[i].c.d) {
      _curcol.d = el[i].c.d;
      output(binary, 'd', _curcol.d);
    }
    if (_curcol.s != el[i].c.s) {
      _curcol.s = el[i].c.s;
      output(binary, 's', _curcol.s);
    }
    if (_curcol.g != el[i].c.g) {
      _curcol.g = el[i].c.g;
      output(binary, 'g', _curcol.g);
    }
    if (!is_zero(el[i].n)) {
      Vector nl = el[i].n;
      if (mag2(nl) > .99f) {
        for_int(c, 3) {
          if (nl[c] && abs(nl[c]) < 1e-5f) nl[c] = 0.f;
        }
      }
      output(binary, 'n', nl);
    }
    output(binary, type == A3dElem::EType::point ? 'p' : 'v', el[i].p);
  }
  _pblank = false;
  if (type == A3dElem::EType::polygon || type == A3dElem::EType::polyline) {
    output(binary, 'E', V(0.f, 0.f, 0.f));
    if (!binary) {
      blank_line();
      _pblank = true;
    }
  }
}

void WA3dStream::write_comment(const string& str) {
  A3dElem el(A3dElem::EType::comment);
  if (!contains(str, '\n')) {
    el.set_comment(str);
    write(el);
    return;
  }
  string s = str;
  for (;;) {
    auto i = s.find('\n');
    string s2 = s.substr(0, i);
    el.set_comment(s2);
    write(el);
    if (i == string::npos) break;
    s.erase(0, i + 1);
  }
}

void WA3dStream::write_end_object(bool binary, float f0, float f1) {
  A3dElem el(A3dElem::EType::endobject, binary);
  el.f() = V(f0, f1, 0.f);
  write(el);
}

void WA3dStream::write_clear_object(bool binary, float f0, float f1) {
  A3dElem el(A3dElem::EType::editobject, binary);
  el.f() = V(f0, f1, 0.f);
  write(el);
}

void WA3dStream::write_end_frame(bool binary) {
  A3dElem el(A3dElem::EType::endframe, binary);
  el.f() = V(0.f, 0.f, 0.f);
  write(el);
}

// *** WSA3dStream

void WSA3dStream::output(bool binary, char ctype, const Vec3<float>& f) {
  if (binary) {
    a3d_binary_buf buf;
    buf.magic[0] = k_a3d_binary_code;
    buf.magic[1] = 0;
    buf.utype = narrow_cast<ushort>(ctype);
    to_std(&buf.utype);
    for_int(c, 3) {
      buf.f[c] = f[c];
      to_std(&buf.f[c]);
    }
    write_binary_raw(_os, ArView(buf));
  } else {
    // precision determined by that in WFile::WFile()
    _os << ctype << " " << f[0] << " " << f[1] << " " << f[2] << '\n';
  }
  if (!_os) {
    showf("Write failed, maybe due to broken pipe.\n");
    exit_immediately(0);
  }
}

void WSA3dStream::output_comment(const string& s) { assertx(_os << "#" << s << "\n"); }

void WSA3dStream::blank_line() { assertx(_os << '\n'); }

}  // namespace hh
