// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GMesh.h"

#include <cctype>    // isalnum()
#include <charconv>  // to_chars()
#include <cstring>   // strncmp(), strlen(), memmove(), etc.

#include "libHh/A3dStream.h"
#include "libHh/Array.h"
#include "libHh/Polygon.h"
#include "libHh/Set.h"

namespace hh {

const FlagMask GMesh::vflag_cusp = Mesh::allocate_Vertex_flag();
const FlagMask GMesh::eflag_sharp = Mesh::allocate_Edge_flag();

void swap(GMesh& l, GMesh& r) noexcept {
  using std::swap;
  swap(implicit_cast<Mesh&>(l), implicit_cast<Mesh&>(r));
  swap(l._os, r._os);
}

void GMesh::copy(const GMesh& m) {
  Mesh::copy(m);
  for (Vertex v : m.vertices()) {
    Vertex vn = id_vertex(m.vertex_id(v));
    set_string(vn, m.get_string(v));
    set_point(vn, m.point(v));
  }
  for (Face f : m.faces()) {
    Face fn = id_face(m.face_id(f));
    set_string(fn, m.get_string(f));
    for (Corner c : m.corners(f)) {
      if (!m.get_string(c)) continue;
      Vertex v = m.corner_vertex(c);
      Vertex vn = id_vertex(m.vertex_id(v));
      Corner cn = corner(vn, fn);
      set_string(cn, m.get_string(c));
    }
  }
  for (Edge e : m.edges()) {
    if (!m.get_string(e)) continue;
    Edge en = edge(id_vertex(m.vertex_id(m.vertex1(e))), id_vertex(m.vertex_id(m.vertex2(e))));
    set_string(en, m.get_string(e));
  }
}

void GMesh::merge(const GMesh& mo, Map<Vertex, Vertex>* pmvvn) {
  unique_ptr<Map<Vertex, Vertex>> tmvvn = !pmvvn ? make_unique<Map<Vertex, Vertex>>() : nullptr;
  Map<Vertex, Vertex>& mvvn = pmvvn ? *pmvvn : *tmvvn;
  for (Vertex vo : mo.ordered_vertices()) {
    Vertex vn = create_vertex();
    mvvn.enter(vo, vn);
    flags(vn) = mo.flags(vo);
    set_string(vn, mo.get_string(vo));
    set_point(vn, mo.point(vo));
  }
  Array<Vertex> van;
  for (Face fo : mo.ordered_faces()) {
    van.init(0);
    for (Vertex vo : mo.vertices(fo)) van.push(mvvn.get(vo));
    Face fn = create_face(van);
    flags(fn) = mo.flags(fo);
    set_string(fn, mo.get_string(fo));
    for (Corner co : mo.corners(fo)) {
      if (!mo.get_string(co)) continue;
      Vertex vo = mo.corner_vertex(co);
      Vertex vn = mvvn.get(vo);
      Corner cn = corner(vn, fn);
      set_string(cn, mo.get_string(co));
    }
  }
  for (Edge eo : mo.edges()) {
    Edge en = edge(mvvn.get(mo.vertex1(eo)), mvvn.get(mo.vertex2(eo)));
    flags(en) = mo.flags(eo);
    set_string(en, mo.get_string(eo));
  }
}

void GMesh::set_point(Vertex v, const Point& pp) {
  v->_point = pp;
  if (_os) *_os << sform("MVertex %d  %g %g %g\n", vertex_id(v), pp[0], pp[1], pp[2]);
}

void GMesh::polygon(Face f, Polygon& poly) const {
  poly.init(0);
  for (Vertex v : vertices(f)) poly.push(point(v));
}

float GMesh::length2(Edge e) const { return dist2(point(vertex1(e)), point(vertex2(e))); }

float GMesh::length(Edge e) const { return sqrt(length2(e)); }

float GMesh::area(Face f) const {
  if (is_triangle(f)) {
    return sqrt(area2(triangle_points(f)));
  } else {
    Polygon& poly = _tmp_poly;
    polygon(f, poly);
    return poly.get_area();
  }
}

void GMesh::transform(const Frame& frame) {
  for (Vertex v : vertices()) set_point(v, point(v) * frame);
}

// STRING

namespace {

inline int str_key_nchars(const char* s) {
  const char* p = s;
  for (;;) {
    const char ch = *p;
    if (ch == ' ' || ch == '=' || ch == 0) break;
    p++;
  }
  return narrow_cast<int>(p - s);
}

inline const char* str_last_non_space(const char* s) {
  const char* p = s;
  for (;;) {
    const char ch = *p;
    if (ch == ' ' || ch == 0) break;
    p++;
  }
  return p - 1;
}

// This may be identical to strchr().  Possibly faster when inlined.
inline const char* str_chr(const char* s, char ch) {
  for (const char* p = s;;) {
    char chp = *p;
    if (chp == ch) return p;
    if (!chp) return nullptr;
    p++;
  }
}

void parse_aux(const char* s, ArrayView<float> ar) {
  assertx(*s++ == '(');
  for_int(c, ar.num()) {
    if (c) assertx(*s++ == ' ');
    ar[c] = float_from_chars(s);
  }
  assertx(*s++ == ')');
}

}  // namespace

bool StringKeyIter::next(const char*& kb, int& kl, const char*& vb, int& vl) {
  if (!_s || !*_s) return false;
  int nch = str_key_nchars(_s);
  if (!assertw(nch)) {
    SHOW(_str, _s);
    return false;
  }
  kb = _s, kl = nch;
  if (_s[nch] != '=') {
    assertx(_s[nch] == 0 || _s[nch] == ' ');
    vb = kb + kl, vl = 0;  // null string ""
    _s += nch;
    if (_s[0] == ' ') _s++;
    return true;
  }
  char ch = _s[nch + 1];
  const char* send;
  if (ch == '(') {
    send = str_chr(_s + nch + 2, ')');
    if (!send) assertnever("No matching ')' " + SSHOW(_s, _s + nch + 2));
  } else if (ch == '"') {
    send = str_chr(_s + nch + 2, '"');
    if (!send) assertnever("No matching '\"' " + SSHOW(_s, _s + nch + 2));
  } else if (std::isalnum(ch)) {
    send = str_last_non_space(_s + nch + 2);
  } else {
    if (Warning("Cannot parse StringKey value")) SHOW(_str, _s + nch + 1);
    return false;
  }
  vb = _s + nch + 1, vl = narrow_cast<int>(send - vb + 1);
  _s = send + 1;
  if (_s[0] == ' ') _s++;
  return true;
}

bool GMesh::string_has_key(const char* ss, const char* key) {
  if (!ss) return false;
  int keyl = int(strlen(key));
  bool found = false;
  for_cstring_key_value_ptr(ss, [&](const char* kb, int kl, const char* vb, int vl) {
    dummy_use(vb, vl);
    if (kl == keyl && !strncmp(kb, key, kl)) {
      found = true;
      return true;
    }
    return false;
  });
  return found;
}

const char* GMesh::string_key(string& str, const char* ss, const char* key) {
  int keyl = int(strlen(key));
  const char* sfound = nullptr;
  for_cstring_key_value_ptr(ss, [&](const char* kb, int kl, const char* vb, int vl) {
    if (kl == keyl && !strncmp(kb, key, kl)) {
      str.assign(vb, vl);
      sfound = str.c_str();
      return true;
    }
    return false;
  });
  return sfound;
}

bool parse_key_vec(const char* ss, const char* key, ArrayView<float> ar) {
  assertx(key && ar.num() >= 1);
  if (!ss) return false;
  string str;
  const char* s = GMesh::string_key(str, ss, key);
  if (!s) return false;
  parse_aux(s, ar);
  return true;
}

bool GMesh::parse_corner_key_vec(Corner c, const char* key, ArrayView<float> ar) const {
  assertx(c && key && ar.num() >= 1);
  string str;
  const char* s = corner_key(str, c, key);
  if (!s) return false;
  parse_aux(s, ar);
  return true;
}

const char* GMesh::corner_key(string& str, Corner c, const char* key) const {
  bool b1 = string_has_key(get_string(c), key);
  bool b2 = string_has_key(get_string(corner_vertex(c)), key);
  if (!b1 && !b2) return nullptr;
  if (b1 && b2) Warning("Have both vertex and corner info");
  if (!b1) return string_key(str, get_string(corner_vertex(c)), key);
  return string_key(str, get_string(c), key);
}

string GMesh::string_update(const string& s, const char* key, const char* val) {
  // inefficient (seldom used)
  unique_ptr<char[]> ss = s != "" ? make_unique_c_string(s.c_str()) : nullptr;
  update_string_ptr(ss, key, val);
  string snew = ss ? ss.get() : "";
  return snew;
}

void GMesh::update_string_ptr(unique_ptr<char[]>& ss, const char* key, const char* val) {
  assertx(key && *key);
  HH_ASSUME(key);
  size_t keyl = strlen(key);
  size_t vall;
  dummy_init(vall);
  if (val) vall = strlen(val);
  const char* sso = ss.get();  // may be nullptr
  int nkeys2 = 0;              // may equal 2 if > 2 keys
  const char* fkb = nullptr;
  int fvl = 0;
  const char* frb = nullptr;  // remainder of string after matching key
  for_cstring_key_value_ptr(sso, [&](const char* kb, int kl, const char* vb, int vl) {
    nkeys2++;
    if (fkb && !frb) {  // found remainder of string; parsed enough.
      frb = kb;
      if (!k_debug) return true;
    }
    bool found = static_cast<size_t>(kl) == keyl && !strncmp(kb, key, kl);
    if (found) {
      if (fkb) assertnever("dup key: " + SSHOW(sso, kb, kl, vb, vl, key, val));
      fkb = kb;
      fvl = vl;
    }
    return false;
  });
  if (!fkb && !val) return;  // no change
  size_t ssol;
  dummy_init(ssol);
  if (sso) ssol = strlen(sso);
  size_t newl = sso ? ssol : 0;
  if (fkb) newl -= (nkeys2 > 1) + keyl + (fvl > 0) + fvl;
  if (val) newl += (nkeys2 - (fkb ? 1 : 0) > 0) + keyl + (*val != 0) + vall;
  if (0) {
    SHOW(sso, key, val, fkb, frb, ssol, newl, nkeys2, keyl, fvl, fkb);
    SHOW(val && *val != 0);
    SHOW(vall);
  }
  unique_ptr<char[]> arnew;
  char* s0;
  if (newl == 0) {  // new string is null (""), so clear it
    ss = nullptr;
    return;
  } else if (sso && newl <= ssol) {  // new string fits, so copy in-place
    s0 = ss.get();                   // "char*" whereas sso is "const char*"
  } else {                           // string needs to grow
    arnew = make_unique<char[]>(newl + 1);
    s0 = arnew.get();
  }
  char* s = s0;
  ASSERTX(s);
  if (fkb) {
    ASSERTX(sso);  // logic implies it
    if (fkb > sso) {
      if (s != sso) std::memcpy(s, sso, fkb - sso - 1);  // does not write '\0'
      s += fkb - sso - 1;
    }
  } else {
    if (sso) {
      ASSERTX(s != sso);
      std::memcpy(s, sso, ssol);
      s += ssol;
    }
  }
  if (val) {
    if (s > s0) *s++ = ' ';
    std::memmove(s, key, keyl);
    s += keyl;
    if (*val) {
      *s++ = '=';
      std::memmove(s, val, vall);
      s += vall;
    }
  }
  if (frb) {
    if (s > s0) *s++ = ' ';
    size_t frbl = strlen(frb);  // frb may be partially overwritten by next std::memmove()
    if (s != frb) std::memmove(s, frb, frbl);
    s += frbl;
  }
  *s = '\0';
  if (s != s0 + newl) assertnever(SSHOW(sso, s0, s - s0, newl));
  if (arnew) ss = std::move(arnew);
}

void GMesh::update_string(Vertex v, const char* key, const char* val) { update_string_ptr(v->_string, key, val); }

void GMesh::update_string(Face f, const char* key, const char* val) { update_string_ptr(f->_string, key, val); }

void GMesh::update_string(Edge e, const char* key, const char* val) { update_string_ptr(e->_string, key, val); }

void GMesh::update_string(Corner c, const char* key, const char* val) { update_string_ptr(c->_string, key, val); }

// I/O

void GMesh::read(std::istream& is) {
  for (string line; my_getline(is, line);) read_line(const_cast<char*>(line.c_str()));
  if (debug() >= 1) ok();
}

// Get the string within the braces.  Note the side-effect on `s`!  This function is copied elsewhere too.
static const char* get_sinfo(const char* s_const) {
  char* s = const_cast<char*>(s_const);
  while (std::isspace(*s)) s++;
  if (!*s) return nullptr;
  if (*s != '{') assertnever("Unexpected character (not '{') at start of '" + string(s) + "'");
  char* s2 = strchr(s + 1, '}');
  if (!s2) assertnever("No matching '}' in '" + string(s) + "'");
  *s++ = 0;
  *s2 = 0;
  return s;
}

void GMesh::read_line(char* sline) {
  if (sline[0] == '#') return;
  switch (sline[0]) {
    case 'V':
      if (const char* s = after_prefix(sline, "Vertex ")) {
        const int vi = int_from_chars(s);
        Point p;
        for_int(c, 3) p[c] = float_from_chars(s);
        const char* sinfo = get_sinfo(s);
        Vertex v = create_vertex_private(vi);
        set_point(v, p);
        if (sinfo) {
          set_string(v, sinfo);
          if (string_has_key(sinfo, "cusp")) flags(v).flag(vflag_cusp) = true;
        }
        return;
      }
      if (const char* s = after_prefix(sline, "Vspl ")) {
        const int vi = int_from_chars(s), vs1i = int_from_chars(s), vs2i = int_from_chars(s), vni = int_from_chars(s);
        assert_no_more_chars(s);
        split_vertex(id_vertex(vi), (vs1i ? id_vertex(vs1i) : nullptr), (vs2i ? id_vertex(vs2i) : nullptr), vni);
        return;
      }
      if (const char* s = after_prefix(sline, "Vmerge ")) {
        const int vi1 = int_from_chars(s), vi2 = int_from_chars(s);
        assert_no_more_chars(s);
        merge_vertices(id_vertex(vi1), id_vertex(vi2));
        return;
      }
      break;
    case 'F':
      if (const char* s = after_prefix(sline, "Face ")) {
        const int fi = int_from_chars(s);
        PArray<Vertex, 6> va;
        for (;;) {
          while (std::isspace(*s)) s++;
          if (!*s || *s == '{') break;
          const int vi = int_from_chars(s);
          Vertex v = id_retrieve_vertex(vi);
          if (!v) assertnever("Vertex in '" + string(sline) + "' does not exist");
          va.push(v);
        }
        if (!assertw(va.num() >= 3)) return;
        if (!assertw(legal_create_face(va))) {  // Profiler shows as costly, but not really -- just cache-priming.
          if (0) SHOW(sline, va);
          return;
        }
        Face f = fi ? create_face_private(fi, va) : create_face(va);
        if (const char* sinfo = get_sinfo(s)) set_string(f, sinfo);
        return;
      }
      break;
    case 'C':
      if (const char* s = after_prefix(sline, "Corner ")) {
        const int vi = int_from_chars(s), fi = int_from_chars(s);
        Vertex v = id_retrieve_vertex(vi);
        Face f = id_retrieve_face(fi);
        const char* sinfo = assertx(get_sinfo(s));
        if (!v) {
          Warning("Corner vertex does not exist");
        } else if (!f) {
          Warning("Corner face does not exist");
        } else {
          set_string(corner(v, f), sinfo);
        }
        return;
      }
      if (const char* s = after_prefix(sline, "CVertex ")) {
        create_vertex_private(to_int(s));
        return;
      }
      break;
    case 'E':
      if (const char* s = after_prefix(sline, "Edge ")) {
        const int vi1 = int_from_chars(s), vi2 = int_from_chars(s);
        const char* sinfo = get_sinfo(s);
        Edge e = query_edge(id_vertex(vi1), id_vertex(vi2));
        if (!e)
          Warning("GMesh::read_line(): Did not find edge in mesh");
        else if (sinfo) {
          set_string(e, sinfo);
          flags(e).flag(eflag_sharp) = string_has_key(sinfo, "sharp");
        }
        return;
      }
      if (const char* s = after_prefix(sline, "Ecol ")) {
        const int vi1 = int_from_chars(s), vi2 = int_from_chars(s);
        assert_no_more_chars(s);
        // collapse_edge(ordered_edge(id_vertex(vi1), id_vertex(vi2)));
        Vertex vs = id_vertex(vi1), vt = id_vertex(vi2);
        collapse_edge_vertex(edge(vs, vt), vs);
        return;
      }
      if (const char* s = after_prefix(sline, "Eswa ")) {
        const int vi1 = int_from_chars(s), vi2 = int_from_chars(s);
        assert_no_more_chars(s);
        assertx(swap_edge(ordered_edge(id_vertex(vi1), id_vertex(vi2))));
        return;
      }
      if (const char* s = after_prefix(sline, "Espl ")) {
        const int vi1 = int_from_chars(s), vi2 = int_from_chars(s), vi3 = int_from_chars(s);
        assert_no_more_chars(s);
        split_edge(ordered_edge(id_vertex(vi1), id_vertex(vi2)), vi3);
        return;
      }
      break;
    case 'M':
      if (const char* s = after_prefix(sline, "MVertex ")) {
        const int vi = int_from_chars(s);
        Point p;
        for_int(c, 3) p[c] = float_from_chars(s);
        const char* sinfo = get_sinfo(s);
        Vertex v = id_vertex(vi);
        set_point(v, p);
        if (sinfo) set_string(v, sinfo);
        return;
      }
      break;
    case 'D':
      if (const char* s = after_prefix(sline, "DVertex ")) {
        destroy_vertex(id_vertex(to_int(s)));
        return;
      }
      if (const char* s = after_prefix(sline, "DFace ")) {
        destroy_face(id_face(to_int(s)));
        return;
      }
      break;
  }
  if (Warning("GMesh::read: cannot parse line")) SHOW(sline);
}

static inline int strprefix(const char* s, const char* p) {
  for (;;) {
    if (!*p) return true;
    if (*s != *p) return false;
    s++;
    p++;
  }
}

bool GMesh::recognize_line(const char* s) {
  static const char* prefixes[] = {
      "Vertex ",  "Face ",  "Corner ", "Edge ", "MVertex ", "CVertex ",
      "DVertex ", "DFace ", "Ecol ",   "Eswa ", "Espl ",    "Vspl",
  };
  for (const char* prefix : prefixes)
    if (strprefix(s, prefix)) return true;
  return false;
}

void GMesh::write(std::ostream& os) const {
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ < 11
  // https://en.cppreference.com/w/cpp/compiler_support/17
  // GCC libstdc++ lacks float support for elementary string conversions prior to Version 11.
  for (Vertex v : ordered_vertices()) {
    const Point& p = point(v);
    os << "Vertex " << vertex_id(v) << "  " << p[0] << " " << p[1] << " " << p[2];
    const char* sinfo = get_string(v);
    if (sinfo) os << " {" << sinfo << "}";
    assertx(os << "\n");
  }

  Array<Face> ar_faces{ordered_faces()};
  for (Face f : ar_faces) {
    os << "Face " << face_id(f) << " ";
    for (Vertex v : vertices(f)) os << " " << vertex_id(v);
    const char* sinfo = get_string(f);
    if (sinfo) os << " {" << sinfo << "}";
    assertx(os << "\n");
  }

  for (Edge e : edges()) {
    const char* sinfo = get_string(e);
    if (sinfo)
      assertx(os << "Edge " << vertex_id(vertex1(e)) << " " << vertex_id(vertex2(e)) << " {" << sinfo << "}\n");
  }

  for (Face f : ar_faces) {
    for (Corner c : corners(f)) {
      const char* sinfo = get_string(c);
      if (sinfo)
        assertx(os << "Corner " << vertex_id(corner_vertex(c)) << " " << face_id(f) << " {" << sinfo << "}\n");
    }
  }

#else
  constexpr int capacity = 500;
  char buffer[capacity];
  char* const end = buffer + capacity;

  strcpy(buffer, "Vertex ");
  char* beg = buffer + strlen(buffer);
  for (Vertex v : ordered_vertices()) {
    char* s = beg;
    s = std::to_chars(s, end, vertex_id(v)).ptr;
    *s++ = ' ';
    const Point& p = point(v);
    for_int(c, 3) {
      *s++ = ' ';
      constexpr int precision = 6;  // Default for printf("%g").
      s = std::to_chars(s, end, p[c], std::chars_format::general, precision).ptr;
    }
    if (const char* sinfo = get_string(v)) {
      *s++ = ' ';
      *s++ = '{';
      assertx(s + strlen(sinfo) + 4 < end);
      while (*sinfo) *s++ = *sinfo++;
      *s++ = '}';
    }
    *s++ = '\n';
    *s = '\0';
    assertx(os << buffer);
  }

  Array<Face> ar_faces{ordered_faces()};
  strcpy(buffer, "Face ");
  beg = buffer + strlen(buffer);
  for (Face f : ar_faces) {
    char* s = beg;
    s = std::to_chars(s, end, face_id(f)).ptr;
    *s++ = ' ';
    for (Vertex v : vertices(f)) {
      *s++ = ' ';
      s = std::to_chars(s, end, vertex_id(v)).ptr;
      assertx(s < end - 4);
    }
    if (const char* sinfo = get_string(f)) {
      *s++ = ' ';
      *s++ = '{';
      assertx(s + strlen(sinfo) + 4 < end);
      while (*sinfo) *s++ = *sinfo++;
      *s++ = '}';
    }
    *s++ = '\n';
    *s = '\0';
    assertx(os << buffer);
  }

  strcpy(buffer, "Edge ");
  beg = buffer + strlen(buffer);
  for (Edge e : edges()) {
    const char* sinfo = get_string(e);
    if (!sinfo) continue;
    char* s = beg;
    s = std::to_chars(s, end, vertex_id(vertex1(e))).ptr;
    *s++ = ' ';
    s = std::to_chars(s, end, vertex_id(vertex2(e))).ptr;
    *s++ = ' ';
    *s++ = '{';
    assertx(s + strlen(sinfo) + 4 < end);
    while (*sinfo) *s++ = *sinfo++;
    *s++ = '}';
    *s++ = '\n';
    *s = '\0';
    assertx(os << buffer);
  }

  strcpy(buffer, "Corner ");
  beg = buffer + strlen(buffer);
  for (Face f : ar_faces) {
    for (Corner c : corners(f)) {
      const char* sinfo = get_string(c);
      if (!sinfo) continue;
      char* s = beg;
      s = std::to_chars(s, end, vertex_id(corner_vertex(c))).ptr;
      *s++ = ' ';
      s = std::to_chars(s, end, face_id(f)).ptr;
      *s++ = ' ';
      *s++ = '{';
      assertx(s + strlen(sinfo) + 4 < end);
      while (*sinfo) *s++ = *sinfo++;
      *s++ = '}';
      *s++ = '\n';
      *s = '\0';
      assertx(os << buffer);
    }
  }
#endif
  os.flush();
}

void GMesh::write(WA3dStream& oa3d, const A3dVertexColor& col) const {
  A3dElem el;
  for (Face f : ordered_faces()) write_face(oa3d, el, col, f);
  oa3d.flush();
}

void GMesh::write_face(WA3dStream& oa3d, A3dElem& el, const A3dVertexColor& col, Face f) const {
  el.init(A3dElem::EType::polygon);
  A3dVertexColor fcol = col;
  A3dColor fcold = col.d;
  parse_key_vec(get_string(f), "rgb", fcold);  // else unmodified
  for (Corner c : corners(f)) {
    Vertex v = corner_vertex(c);
    Vector nor(0.f, 0.f, 0.f);
    fcol.d = fcold;
    parse_corner_key_vec(c, "normal", nor);
    parse_corner_key_vec(c, "rgb", fcol.d);
    el.push(A3dVertex(point(v), nor, fcol));
  }
  oa3d.write(el);
}

std::ostream* GMesh::record_changes(std::ostream* pos) {
  std::ostream* oos = _os;
  _os = pos;
  return oos;
}

Vertex GMesh::create_vertex_private(int id) {
  if (_os) *_os << "CVertex " << id << '\n';
  return Mesh::create_vertex_private(id);
}

// Override Mesh members
void GMesh::destroy_vertex(Vertex v) {
  if (_os) *_os << "DVertex " << vertex_id(v) << '\n';
  Mesh::destroy_vertex(v);
}

Face GMesh::create_face_private(int id, CArrayView<Vertex> va) {
  Face f = Mesh::create_face_private(id, va);
  if (_os) {
    *_os << "Face " << id << ' ';
    for_int(i, va.num()) { *_os << ' ' << vertex_id(va[i]); }
    *_os << '\n';
  }
  return f;
}

void GMesh::destroy_face(Face f) {
  if (_os) *_os << "DFace " << face_id(f) << '\n';
  Mesh::destroy_face(f);
}

void GMesh::collapse_edge_vertex(Edge e, Vertex vs) {
  if (debug() >= 1) valid(e);
  std::ostream* tos = _os;
  _os = nullptr;
  Vertex vt = opp_vertex(vs, e);
  int ids = vertex_id(vs), idt = vertex_id(vt);
  // Compute geometry for unified vertex (vs)
  int isbs = is_boundary(vs), isbt = is_boundary(vt), sumb = isbs + isbt;
  Point p = sumb == 0 || sumb == 2 ? Point(interp(point(vs), point(vt))) : isbs ? point(vs) : point(vt);
  Set<Vertex> vsharp;
  for (Edge ee : edges(vt))
    if (ee != e && flags(ee).flag(eflag_sharp)) vsharp.enter(opp_vertex(vt, ee));
  Mesh::collapse_edge_vertex(e, vs);  // vs is kept
  // e = nullptr;  // now undefined
  set_point(vs, p);  // (_os == nullptr)
  for (Vertex v : vsharp) {
    Edge ee = edge(vs, v);
    if (!is_boundary(ee)) flags(ee).flag(eflag_sharp) = true;
  }
  for (Edge ee : edges(vs)) {
    // test (vs, vo1), (vs, vo2) to remove eflag_sharp if now boundary
    if (is_boundary(ee) && flags(ee).flag(eflag_sharp)) flags(ee).flag(eflag_sharp) = false;
  }
  if (tos) {
    _os = tos;
    *_os << "Ecol " << ids << ' ' << idt << '\n';
  }
}

void GMesh::collapse_edge(Edge e) { collapse_edge_vertex(e, vertex1(e)); }

Vertex GMesh::split_edge(Edge e, int id) {
  if (debug() >= 1) valid(e);
  std::ostream* tos = _os;
  _os = nullptr;
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Face f1 = face1(e), f2 = face2(e);
  Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e);
  bool flag_e = flags(e).flag(eflag_sharp);
  auto fstring1 = make_unique_c_string(get_string(f1));  // often nullptr
  auto fstring2 = f2 ? make_unique_c_string(get_string(f2)) : nullptr;
  Vertex vn = Mesh::split_edge(e, id);
  flags(edge(v1, vn)).flag(eflag_sharp) = flag_e;
  flags(edge(v2, vn)).flag(eflag_sharp) = flag_e;
  if (fstring1)
    for (Face f : faces(edge(vn, vo1))) set_string(f, fstring1.get());
  if (fstring2)
    for (Face f : faces(edge(vn, vo2))) set_string(f, fstring2.get());
  set_point(vn, interp(point(v1), point(v2)));  // (_os == nullptr)
  if (tos) {
    _os = tos;
    *_os << "Espl " << vertex_id(v1) << ' ' << vertex_id(v2) << ' ' << vertex_id(vn) << '\n';
  }
  return vn;
}

Edge GMesh::swap_edge(Edge e) {
  if (debug() >= 1) valid(e);
  std::ostream* tos = _os;
  _os = nullptr;
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Face f1 = face1(e), f2 = face2(e);
  unique_ptr<char[]> fstring;
  if (get_string(f1) && get_string(f2) && !strcmp(get_string(f1), get_string(f2)))
    fstring = make_unique_c_string(get_string(f1));
  Edge ne = Mesh::swap_edge(e);
  if (fstring)
    for (Face f : faces(ne)) set_string(f, fstring.get());
  if (tos) {
    _os = tos;
    *_os << "Eswa " << vertex_id(v1) << ' ' << vertex_id(v2) << '\n';
  }
  return ne;
}

Vertex GMesh::split_vertex(Vertex v1, Vertex vs1, Vertex vs2, int v2i) {
  std::ostream* tos = _os;
  _os = nullptr;
  Vertex vn = Mesh::split_vertex(v1, vs1, vs2, v2i);
  if (Edge e = vs1 ? query_edge(v1, vs1) : nullptr) {
    set_string(e, nullptr);
    flags(e) = 0;
  }
  if (Edge e = vs2 ? query_edge(v1, vs2) : nullptr) {
    set_string(e, nullptr);
    flags(e) = 0;
  }
  if (tos) {
    _os = tos;
    *_os << "Vspl " << vertex_id(v1) << ' ' << (vs1 ? vertex_id(vs1) : 0) << ' ' << (vs2 ? vertex_id(vs2) : 0) << ' '
         << v2i << '\n';
  }
  return vn;
}

void GMesh::merge_vertices(Vertex vs, Vertex vt) {
  if (_os) *_os << "Vmerge " << vertex_id(vs) << ' ' << vertex_id(vt) << '\n';
  Mesh::merge_vertices(vs, vt);
}

Vertex GMesh::center_split_face(Face f) {
  Polygon poly;
  polygon(f, poly);
  auto fstring = make_unique_c_string(get_string(f));  // often nullptr
  Map<Vertex, unique_ptr<char[]>> mvs;
  for (Corner c : corners(f)) {
    Vertex v = corner_vertex(c);
    if (get_string(c)) mvs.enter(v, extract_string(c));
  }
  Vector snor{};
  bool have_nor = true;
  Vector scol{};
  bool have_col = true;
  UV suv{};
  bool have_uv = true;
  for (Corner c : corners(f)) {
    if (Vector nor; parse_corner_key_vec(c, "normal", nor))
      snor += nor;
    else
      have_nor = false;
    if (Vector col; parse_corner_key_vec(c, "rgb", col))
      scol += col;
    else
      have_col = false;
    if (UV uv; parse_corner_key_vec(c, "uv", uv))
      suv += uv;
    else
      have_uv = false;
  }
  snor /= float(poly.num());
  scol /= float(poly.num());
  suv /= float(poly.num());
  Vertex vn = Mesh::center_split_face(f);
  set_point(vn, mean(poly));
  string str;
  if (have_nor) update_string(vn, "normal", csform_vec(str, snor));
  if (have_col) update_string(vn, "rgb", csform_vec(str, scol));
  if (have_uv) update_string(vn, "uv", csform_vec(str, suv));
  if (fstring)
    for (Face fn : faces(vn)) set_string(fn, fstring.get());
  if (mvs.num()) {
    for (Face fn : faces(vn)) {
      for (Corner cc : corners(fn)) {
        Vertex vv = corner_vertex(cc);
        if (vv == vn) continue;
        if (mvs.contains(vv)) set_string(cc, std::move(mvs.get(vv)));  // may be nullptr
      }
    }
  }
  return vn;
}

Edge GMesh::split_face(Face f, Vertex v1, Vertex v2) {
  auto fstring = make_unique_c_string(get_string(f));  // often nullptr
  Map<Vertex, unique_ptr<char[]>> mvs;
  for (Corner c : corners(f)) {
    Vertex v = corner_vertex(c);
    if (get_string(c)) mvs.enter(v, make_unique_c_string(get_string(c)));
  }
  Edge en = Mesh::split_face(f, v1, v2);
  // f = nullptr;  // now undefined
  if (fstring)
    for (Face ff : faces(en)) set_string(ff, fstring.get());
  if (mvs.num()) {
    for (Face ff : faces(en)) {
      for (Corner cc : corners(ff)) {
        Vertex vv = corner_vertex(cc);
        set_string(cc, mvs.retrieve(vv).get());  // could be nullptr
      }
    }
  }
  return en;
}

Face GMesh::coalesce_faces(Edge e) {
  Face f1 = face1(e), f2 = face2(e);
  unique_ptr<char[]> fstring;
  if (get_string(f1) && get_string(f2) && !strcmp(get_string(f1), get_string(f2)))
    fstring = make_unique_c_string(get_string(f1));
  Face fn = Mesh::coalesce_faces(e);
  set_string(fn, fstring.get());
  return fn;
}

Vertex GMesh::insert_vertex_on_edge(Edge e) {
  // lose strings
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Vertex vn = Mesh::insert_vertex_on_edge(e);
  set_point(vn, interp(point(v1), point(v2)));
  return vn;
}

Edge GMesh::remove_vertex_between_edges(Vertex vr) {
  // lose strings
  return Mesh::remove_vertex_between_edges(vr);
}

Array<Vertex> GMesh::fix_vertex(Vertex v) {
  Array<Vertex> new_vertices = Mesh::fix_vertex(v);
  for (Vertex vnew : new_vertices) {
    set_string(vnew, get_string(v));
    set_point(vnew, point(v));
  }
  return new_vertices;
}

}  // namespace hh
