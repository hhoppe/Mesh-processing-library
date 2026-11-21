// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#include "libHh/GMesh.h"

#include <cctype>   // isalnum()
#include <cstring>  // strncmp(), strlen(), memmove(), etc.
#include <sstream>  // std::istringstream

#include "libHh/A3dStream.h"
#include "libHh/Array.h"
#include "libHh/LineWriter.h"
#include "libHh/Map.h"
#include "libHh/Ply.h"
#include "libHh/Polygon.h"
#include "libHh/Set.h"

namespace hh {

const FlagMask GMesh::vflag_cusp = Mesh::allocate_Vertex_flag();
const FlagMask GMesh::eflag_sharp = Mesh::allocate_Edge_flag();

void swap(GMesh& l, GMesh& r) noexcept {
  ranges::swap(implicit_cast<Mesh&>(l), implicit_cast<Mesh&>(r));
  ranges::swap(l._os, r._os);
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
  auto tmvvn = make_optional_if<Map<Vertex, Vertex>>(!pmvvn);
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
  if (_os) {
    LineWriter w;
    w << "MVertex " << vertex_id(v) << ' ';
    for_int(c, 3) w << ' ' << pp[c];
    w.write_line(*_os);
  }
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
    Polygon poly;
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
    const char chp = *p;
    if (chp == ch) return p;
    if (!chp) return nullptr;
    p++;
  }
}

void parse_aux(const char* s, ArrayView<float> ar) {
  assertx(*s++ == '(');
  for_int(c, ar.num()) {
    if (c && *s++ != ' ') assertnever(string("got: ") + s);
    ar[c] = float_from_chars(s);
  }
  assertx(*s++ == ')');
}

}  // namespace

bool StringKeyIter::next(const char*& kb, int& kl, const char*& vb, int& vl) {
  if (!_s || !*_s) return false;
  const int nch = str_key_nchars(_s);
  if (!assertw(nch)) {
    SHOW(_str, _s);
    return false;
  }
  kb = _s, kl = nch;
  if (_s[nch] != '=') {
    assertx(_s[nch] == 0 || _s[nch] == ' ');
    vb = kb + kl, vl = 0;  // A null string "".
    _s += nch;
    if (_s[0] == ' ') _s++;
    return true;
  }
  const char ch = _s[nch + 1];
  const char* send;
  if (ch == '(') {
    send = str_chr(_s + nch + 2, ')');
    if (!send) assertnever("No matching ')' " + SSHOW(_s, _s + nch + 2));
  } else if (ch == '"') {
    send = str_chr(_s + nch + 2, '"');
    if (!send) assertnever("No matching '\"' " + SSHOW(_s, _s + nch + 2));
  } else if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '-') {
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
  assertx(key && ar.num());
  if (!ss) return false;
  string str;
  const char* s = GMesh::string_key(str, ss, key);
  if (!s) return false;
  parse_aux(s, ar);
  return true;
}

bool GMesh::parse_corner_key_vec(Corner c, const char* key, ArrayView<float> ar) const {
  assertx(c && key && ar.num());
  string str;
  const char* s = corner_key(str, c, key);
  if (!s) return false;
  parse_aux(s, ar);
  return true;
}

const char* GMesh::corner_key(string& str, Corner c, const char* key) const {
  const bool b1 = string_has_key(get_string(c), key);
  const bool b2 = string_has_key(get_string(corner_vertex(c)), key);
  if (!b1 && !b2) return nullptr;
  if (b1 && b2) Warning("Have both vertex and corner info");
  if (!b1) return string_key(str, get_string(corner_vertex(c)), key);
  return string_key(str, get_string(c), key);
}

string GMesh::string_update(const string& s, const char* key, const char* val) {
  // Inefficient (seldom used).
  unique_ptr<char[]> ss = s != "" ? make_unique_c_string(s.c_str()) : nullptr;
  update_string_ptr(ss, key, val);
  string s_new = ss ? ss.get() : "";
  return s_new;
}

void GMesh::update_string_ptr(unique_ptr<char[]>& ss, const char* key, const char* val) {
  assertx(key && *key);
  HH_ASSUME(key);
  size_t keyl = strlen(key);
  size_t vall;
  dummy_init(vall);
  if (val) vall = strlen(val);
  const char* sso = ss.get();  // May be nullptr.
  int nkeys2 = 0;              // May equal 2 if > 2 keys.
  const char* fkb = nullptr;
  int fvl = 0;
  const char* frb = nullptr;  // Remainder of the string after the matching key.
  for_cstring_key_value_ptr(sso, [&](const char* kb, int kl, const char* vb, int vl) {
    nkeys2++;
    if (fkb && !frb) {  // Found remainder of string; parsed enough.
      frb = kb;
      if (!k_debug) return true;
    }
    const bool found = size_t(kl) == keyl && !strncmp(kb, key, kl);
    if (found) {
      if (fkb) assertnever("dup key: " + SSHOW(sso, kb, kl, vb, vl, key, val));
      fkb = kb;
      fvl = vl;
    }
    return false;
  });
  if (!fkb && !val) return;  // No change.
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
  if (newl == 0) {  // The new string is null (""), so clear it.
    ss = nullptr;
    return;
  } else if (sso && newl <= ssol) {  // The new string fits, so copy in-place.
    s0 = ss.get();                   // A "char*" whereas sso is "const char*".
  } else {                           // The string needs to grow.
    arnew = make_unique<char[]>(newl + 1);
    s0 = arnew.get();
  }
  char* s = s0;
  ASSERTX(s);
  if (fkb) {
    ASSERTX(sso);  // Logic implies it.
    if (fkb > sso) {
      if (s != sso) std::memcpy(s, sso, fkb - sso - 1);  // Does not write '\0'.
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
    const size_t frbl = strlen(frb);  // Here, frb may be partially overwritten by the next std::memmove().
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

namespace {

// Returns true if the line (the first one that is neither blank nor a comment) belongs to a Wavefront OBJ file
// rather than a mesh file.
bool is_obj_line(std::string_view line) {
  if (line.starts_with("#MRGB ")) return true;  // ZBrush vertex colors, which precede the vertices.
  const size_t end = line.find_first_of(" \t");
  const std::string_view token = line.substr(0, end);
  if (token == "o") {  // Unless it is the object line "o 1 1 0" of an a3d/mesh stream.
    std::istringstream iss{string(line.substr(1))};
    int i1, i2, i3;
    string rest;
    if (iss >> i1 >> i2 >> i3 && !(iss >> rest)) return false;
  }
  for (const char* keyword : {"v", "vt", "vn", "vp", "f", "l", "g", "s", "o", "usemtl", "mtllib"})
    if (token == keyword) return true;
  return false;
}

// Skips whitespace and returns true if a value follows.
bool more_values(const char*& s) {
  while (*s == ' ' || *s == '\t' || *s == '\r') s++;
  return *s != '\0';
}

}  // namespace

void GMesh::read(std::istream& is) {
  if (is.peek() == 'p') {  // A mesh file never starts with 'p', whereas a PLY file starts with the line "ply".
    init_from_ply(read_ply(is));
  } else {
    bool seen_data = false;
    // DOS line endings are common in OBJ files, so they draw a warning only after the format is known to be mesh.
    for (string line; my_getline(is, line, seen_data);) {
      if (!seen_data) {
        const size_t i = line.find_first_not_of(" \t\r");
        if (i == string::npos || (line[i] == '#' && !line.starts_with("#MRGB "))) continue;
        seen_data = true;
        if (is_obj_line(line)) {
          read_obj(is, line);
          break;
        }
      }
      read_line(line.data());
    }
  }
  if (debug() >= 1) ok();
}

// *** GMeshInput

GMeshInput::GMeshInput(std::istream& is) : _is(is) {
  if (_is.peek() == 'p') {
    _ply_header_lines = read_ply_header(_is);
    for (const string& comment : ply_header_comments(_ply_header_lines)) showff("|%s\n", comment.c_str());
  } else {
    for (string line; _is.peek() == '#';) {
      assertx(my_getline(_is, line, false));  // The comments may precede an OBJ file, whose DOS line endings are ok.
      if (line.size() > 1) showff("|%s\n", line.substr(line[1] == ' ' ? 2 : 1).c_str());
    }
  }
}

void GMeshInput::read(GMesh& mesh) {
  if (_ply_header_lines.num()) {
    mesh.init_from_ply(read_ply_body(_is, _ply_header_lines));
  } else {
    mesh.read(_is);
  }
}

PlyMesh GMesh::to_ply() const {
  // The kind of a vertex or corner key: a flag (without value), a numeric vector of some dimension, or invalid.
  struct KeyInfo {
    int dim{-1};    // 0 for a flag; -2 if invalid (non-numeric value or inconsistent dimensions); -1 if not yet seen.
    int offset{0};  // Offset of its values within a wedge record.
  };
  Map<string, KeyInfo> key_infos;
  // The few distinct keys are found by a linear search, faster than hashing each key of each string.  (The values of
  // key_infos have stable addresses.)
  Array<std::pair<string, KeyInfo*>> key_cache;
  const auto find_key_info = [&](const char* kb, int kl) -> KeyInfo& {
    const std::string_view key(kb, kl);
    for (const auto& [name, pinfo] : key_cache)
      if (name == key) return *pinfo;
    bool is_new;
    KeyInfo& info = key_infos.enter(string(key), KeyInfo{}, is_new);
    if (key_cache.num() < 32) key_cache.push({string(key), &info});
    return info;
  };
  Array<float> values;
  // Parse the value of a key: returns its dimension (0 if no value), or -2 if it is not numeric.
  const auto parse_value = [&](const char* vb, int vl) -> int {
    values.init(0);
    if (vl == 0) return 0;
    const char* s = vb;
    const char* const send = vb + vl;
    const bool is_vector = vl >= 2 && vb[0] == '(' && send[-1] == ')';
    const char* const vend = is_vector ? send - 1 : send;
    if (is_vector) s++;
    for (;;) {  // Parse in place, without copying the value.
      while (s < vend && (*s == ' ' || *s == '\t')) s++;
      if (s == vend) break;
      float value;
      if (!real_from_chars(s, vend, value)) return -2;
      values.push(value);
    }
    if (values.empty() || (!is_vector && values.num() != 1)) return -2;
    return values.num();
  };
  const auto classify = [&](const char* str) {
    for_cstring_key_value_ptr(str, [&](const char* kb, int kl, const char* vb, int vl) {
      const int dim = parse_value(vb, vl);
      KeyInfo& info = find_key_info(kb, kl);
      if (info.dim == -1) {
        info.dim = dim;
      } else if (info.dim != dim) {
        info.dim = -2;
      }
      return false;
    });
  };
  for (Vertex v : vertices()) classify(get_string(v));
  for (Face f : faces())
    for (Corner c : corners(f)) classify(get_string(c));
  // The keys stored in the wedge records, in sorted order, and the dropped keys.
  Array<string> record_keys, dropped_keys;
  {
    Array<string> keys(key_infos.keys());
    sort(keys);
    for (const string& key : keys) {
      const int dim = key_infos.get(key).dim;
      if (key == "wid" && dim == 1) continue;
      (dim >= 0 ? record_keys : dropped_keys).push(key);
    }
  }
  int record_size = 0;
  for (const string& key : record_keys) {
    KeyInfo& info = key_infos.get(key);
    info.offset = record_size;
    record_size += max(info.dim, 1);
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  // Overlay the values of the keys of the string onto the record (NaN for absent values; 0 or 1 for flags).
  const auto add_to_record = [&](const char* str, ArrayView<float> record) {
    for_cstring_key_value_ptr(str, [&](const char* kb, int kl, const char* vb, int vl) {
      const KeyInfo& info = find_key_info(kb, kl);
      if (info.dim < 0 || std::string_view(kb, kl) == "wid") return false;
      assertx(parse_value(vb, vl) == info.dim);
      if (info.dim == 0) {
        record[info.offset] = 1.f;
      } else {
        for_int(k, info.dim) record[info.offset + k] = values[k];
      }
      return false;
    });
  };
  const auto empty_record = [&] {
    Array<float> record(record_size, nan);
    for (const string& key : record_keys)
      if (key_infos.get(key).dim == 0) record[key_infos.get(key).offset] = 0.f;
    return record;
  };
  const auto wid_of = [&](const char* str) {
    string str2;
    const char* s = string_key(str2, str, "wid");
    return s ? to_int(s) : 0;
  };
  // The vertices in id order.
  Array<Vertex> va(ordered_vertices());
  const int nv = va.num();
  Array<int> index_of_vertex_id(nv ? vertex_id(va.last()) + 1 : 0, -1);
  for_int(i, nv) index_of_vertex_id[vertex_id(va[i])] = i;
  const auto vertex_index = [&](Vertex v) { return index_of_vertex_id[vertex_id(v)]; };
  // Whether two records are identical, including their NaN values.
  const auto same_records = [](CArrayView<float> record1, CArrayView<float> record2) {
    return std::memcmp(record1.data(), record2.data(), record1.num() * sizeof(float)) == 0;
  };
  Array<Array<float>> vertex_records(nv);
  for_int(i, nv) {
    vertex_records[i] = empty_record();
    add_to_record(get_string(va[i]), vertex_records[i]);
  }
  // The wedges: each one has a vertex and a record; each face corner refers to a wedge.
  struct Wedge {
    int vertex;
    int order;  // Its wid, or else its order of creation.
    Array<float> record;
  };
  Array<Wedge> wedges;
  Array<Face> fa(ordered_faces());
  Array<int> corner_wedges;  // Concatenated over the faces in id order.
  Array<int> face_start{0};
  for (Face f : fa) {
    for (Vertex v : vertices(f)) dummy_use(v), corner_wedges.push(-1);
    face_start.push(corner_wedges.num());
  }
  // Try the wid keys: each wid must have one vertex and one record.
  bool use_wids = key_infos.contains("wid") && key_infos.get("wid").dim == 1;
  if (use_wids) {
    Map<int, int> wedge_of_wid;
    for_int(fi, fa.num()) {
      Face f = fa[fi];
      int j = face_start[fi];
      for (Corner c : corners(f)) {
        const Vertex v = corner_vertex(c);
        const int vi = vertex_index(v);
        int wid = wid_of(get_string(c));
        if (!wid) wid = wid_of(get_string(v));
        if (!wid) {
          use_wids = false;
          break;
        }
        Array<float> record(vertex_records[vi]);
        add_to_record(get_string(c), record);
        bool is_new;
        const int w = wedge_of_wid.enter(wid, wedges.num(), is_new);
        if (is_new) {
          wedges.push(Wedge{vi, wid, std::move(record)});
        } else if (wedges[w].vertex != vi || !same_records(wedges[w].record, record)) {
          use_wids = false;
          break;
        }
        corner_wedges[j++] = w;
      }
      if (!use_wids) break;
    }
    if (!use_wids) {
      Warning("GMesh::to_ply(): inconsistent wid keys are ignored");
      wedges.init(0);
    }
  }
  if (!use_wids) {
    // Each vertex without corner strings has a single wedge; the corners of the other vertices are grouped by their
    // records.
    Array<bool> has_corner_strings(nv, false);
    for (Face f : fa)
      for (Corner c : corners(f))
        if (get_string(c)) has_corner_strings[vertex_index(corner_vertex(c))] = true;
    Array<int> vertex_wedge(nv, -1);
    // The wedges of each vertex with corner strings, as a linked list: the first one and the next of each.
    Array<int> first_wedge(nv, -1), next_wedge;
    for_int(fi, fa.num()) {
      Face f = fa[fi];
      int j = face_start[fi];
      for (Corner c : corners(f)) {
        const Vertex v = corner_vertex(c);
        const int vi = vertex_index(v);
        const char* cstr = get_string(c);
        if (!has_corner_strings[vi]) {
          if (vertex_wedge[vi] < 0) {
            vertex_wedge[vi] = wedges.num();
            wedges.push(Wedge{vi, wedges.num(), Array<float>(vertex_records[vi])});
          }
          corner_wedges[j++] = vertex_wedge[vi];
          continue;
        }
        Array<float> record(vertex_records[vi]);
        add_to_record(cstr, record);
        int w = first_wedge[vi];
        while (w >= 0 && !same_records(wedges[w].record, record)) w = next_wedge[w];
        if (w < 0) {
          w = wedges.num();
          wedges.push(Wedge{vi, w, std::move(record)});
          next_wedge.resize(w + 1);
          next_wedge[w] = first_wedge[vi];
          first_wedge[vi] = w;
        }
        corner_wedges[j++] = w;
      }
    }
  }
  // Isolated vertices get a wedge with their vertex record.
  {
    Array<bool> has_wedge(nv, false);
    for (const Wedge& wedge : wedges) has_wedge[wedge.vertex] = true;
    for_int(vi, nv) {
      if (has_wedge[vi]) continue;
      const int order = use_wids ? std::numeric_limits<int>::max() : wedges.num();
      wedges.push(Wedge{vi, order, Array<float>(vertex_records[vi])});
    }
  }
  // Order the wedges by wid, or else by vertex and then order of creation.
  Array<int> order(range(wedges.num()));
  sort(order, [&](int w1, int w2) {
    const Wedge &wedge1 = wedges[w1], &wedge2 = wedges[w2];
    if (!use_wids && wedge1.vertex != wedge2.vertex) return wedge1.vertex < wedge2.vertex;
    if (wedge1.order != wedge2.order) return wedge1.order < wedge2.order;
    return w1 < w2;
  });
  Array<int> new_index(wedges.num());
  for_int(i, order.num()) new_index[order[i]] = i;
  const int nw = wedges.num();
  PlyMesh ply;
  ply.points.init(nw);
  for_int(i, nw) ply.points[i] = point(va[wedges[order[i]].vertex]);
  // The standard properties, for keys present (with the right dimension) on all wedges.
  const auto all_present = [&](const char* key, int dim) {
    if (!key_infos.contains(key) || key_infos.get(key).dim != dim) return false;
    const KeyInfo& info = key_infos.get(key);
    return ranges::none_of(wedges, [&](const Wedge& wedge) { return std::isnan(wedge.record[info.offset]); });
  };
  Set<string> standard_keys;
  if (all_present("normal", 3)) {
    const int offset = key_infos.get("normal").offset;
    ply.normals.init(nw);
    for_int(i, nw) for_int(c, 3) ply.normals[i][c] = wedges[order[i]].record[offset + c];
    standard_keys.enter("normal");
  }
  if (all_present("rgb", 3)) {
    const int offset = key_infos.get("rgb").offset;
    ply.colors.init(nw);
    for_int(i, nw) {
      for_int(c, 3) ply.colors[i][c] = uint8_t(clamp(wedges[order[i]].record[offset + c], 0.f, 1.f) * 255.f + .5f);
      ply.colors[i][3] = 255;
    }
    standard_keys.enter("rgb");
  }
  if (all_present("uv", 2)) {
    const int offset = key_infos.get("uv").offset;
    ply.uvs.init(nw);
    for_int(i, nw) for_int(c, 2) ply.uvs[i][c] = wedges[order[i]].record[offset + c];
    standard_keys.enter("uv");
  }
  for (const string& key : record_keys) {
    if (standard_keys.contains(key)) continue;
    const KeyInfo& info = key_infos.get(key);
    PlyMesh::WedgeKey wedge_key{key, info.dim, Array<float>(nw * max(info.dim, 1))};
    const int stride = max(info.dim, 1);
    for_int(i, nw) { for_int(k, stride) wedge_key.values[i * stride + k] = wedges[order[i]].record[info.offset + k]; }
    ply.wedge_keys.push(std::move(wedge_key));
  }
  // The wedge_vertex element, unless each vertex has a single wedge in vertex order.
  bool identity = nw == nv;
  for_int(i, nw) identity = identity && wedges[order[i]].vertex == i;
  if (!identity) {
    ply.wedge_vertex.init(nw);
    for_int(i, nw) ply.wedge_vertex[i] = wedges[order[i]].vertex;
  }
  // The ids of the vertices and faces, unless they are consecutive.
  if (!ranges::all_of(range(nv), [&](int i) { return vertex_id(va[i]) == i + 1; }))
    ply.vertex_ids = transformed(va, [&](Vertex v) { return vertex_id(v); });
  if (!ranges::all_of(range(fa.num()), [&](int i) { return face_id(fa[i]) == i + 1; }))
    ply.face_ids = transformed(fa, [&](Face f) { return face_id(f); });
  // The faces and their materials.
  ply.face_vertices.init(corner_wedges.num());
  for_int(j, corner_wedges.num()) ply.face_vertices[j] = new_index[corner_wedges[j]];
  ply.face_start = face_start;
  {
    // Use the matid keys as material indices if they are consistent, else the order of first appearance.
    bool use_matids = true;
    Map<int, string> string_of_matid;
    for (Face f : fa) {
      const char* str = get_string(f);
      string str2;
      const char* smatid = str ? string_key(str2, str, "matid") : nullptr;
      const int matid = smatid ? to_int(smatid) : -1;
      if (matid < 0 || matid >= 65536) {
        use_matids = false;
        break;
      }
      bool is_new;
      const string& s2 = string_of_matid.enter(matid, str, is_new);
      if (s2 != str) {
        use_matids = false;
        break;
      }
    }
    Map<string, int> matid_of_string;
    ply.face_matids.init(fa.num());
    for_int(fi, fa.num()) {
      const char* str = get_string(fa[fi]);
      const string s2 = str ? str : "";
      int matid;
      if (use_matids) {
        string str2;
        matid = to_int(string_key(str2, str, "matid"));
      } else {
        bool is_new;
        matid = matid_of_string.enter(s2, matid_of_string.num(), is_new);
        if (matid >= 65536) assertnever("GMesh::to_ply(): more than 65536 distinct face strings");
      }
      if (matid >= ply.materials.num()) ply.materials.resize(matid + 1);
      ply.materials[matid] = s2;
      ply.face_matids[fi] = matid;
    }
    if (ranges::all_of(ply.face_matids, [](int matid) { return matid == 0; })) ply.face_matids.init(0);
    if (ply.materials.num() == 1 && ply.materials[0] == "") ply.materials.init(0);
  }
  // Report what is lost.
  const auto join = [](CArrayView<string> strings) {
    string result;
    for (const string& str : strings) result += (result == "" ? "" : " ") + str;
    return result;
  };
  int num_edge_strings = 0;
  Set<string> edge_keys;
  for (Edge e : edges()) {
    if (!get_string(e)) continue;
    num_edge_strings++;
    for_cstring_key_value_ptr(get_string(e), [&](const char* kb, int kl, const char*, int) {
      edge_keys.add(string(kb, kl));
      return false;
    });
  }
  if (num_edge_strings) {
    Array<string> keys(edge_keys);
    sort(keys);
    Warning("GMesh::to_ply(): edge strings are dropped");
    showdf("GMesh::to_ply(): dropping the strings of %d edges (keys: %s)\n", num_edge_strings, join(keys).c_str());
  }
  if (dropped_keys.num()) {
    Warning("GMesh::to_ply(): non-numeric vertex/corner keys are dropped");
    showdf("GMesh::to_ply(): dropping the keys: %s\n", join(dropped_keys).c_str());
  }
  ply.add_comment_lines(diagnostics_transcript());
  return ply;
}

void GMesh::read_obj(std::istream& is, const string& first_line) {
  assertx(empty());
  constexpr int k_no_smoothing = 0, k_default_smoothing = 1;  // Smoothing is "on" by default.
  Map<string, int> smoothing_ids;
  smoothing_ids.enter("off", k_no_smoothing);
  smoothing_ids.enter("1", k_default_smoothing);
  int smoothing = k_default_smoothing;
  string groups = "default", material = "nomaterial";
  Map<string, int> material_counts;
  Array<Point> ar_p;
  Array<Uv> ar_vt;
  Array<Vector> ar_vn;
  Array<Vector> ar_vrgb;
  Array<int> face_smoothing;  // Indexed by face id - 1.
  int num_created_vertices = 0, num_illegal = 0, num_lines = 0, num_ignored = 0;
  string str;
  Array<Vertex> va;
  Array<Vec3<int>> corners;  // For each face vertex, the 1-based indices of v, vt (or 0), and vn (or 0).
  const auto fix_index = [&](int i, int n, const string& line) {
    if (i < 0) i = n + 1 + i;  // A negative index is relative to the end.
    if (i < 1 || i > n) assertnever("obj: index out of range in line '" + line + "'");
    return i;
  };
  const auto process = [&](const string& line) {
    const char* s0 = line.c_str();
    if (const char* s = after_prefix(s0, "v ")) {
      Point p;
      for_int(c, 3) p[c] = float_from_chars(s);
      ar_p.push(p);
      if (!more_values(s)) return;
      if (*s == '#') {  // The color format of Phil Dench's ply2obj: "#rrggbb".
        s++;
        Vector rgb;
        for_int(c, 3) {
          const string hex = string(s, 6).substr(size_t{2} * c, 2);
          rgb[c] = float(std::stoi(hex, nullptr, 16)) / 255.f;
        }
        ar_vrgb.push(rgb);
        return;
      }
      Array<float> values;
      while (more_values(s)) values.push(float_from_chars(s));
      if (values.num() == 3 || values.num() == 4) {  // Colors "r g b", possibly after the weight w.
        ar_vrgb.push(Vector(values[values.num() - 3], values[values.num() - 2], values[values.num() - 1]));
      } else if (values.num() != 1) {  // A lone value is the weight w, which is ignored.
        assertnever("obj: cannot parse line '" + line + "'");
      }
    } else if (const char* svt = after_prefix(s0, "vt ")) {
      Uv uv;
      for_int(c, 2) uv[c] = float_from_chars(svt);  // An optional third coordinate is ignored.
      ar_vt.push(uv);
    } else if (const char* svn = after_prefix(s0, "vn ")) {
      Vector nor;
      for_int(c, 3) nor[c] = float_from_chars(svn);
      ar_vn.push(nor);
    } else if (const char* sf = after_prefix(s0, "f ")) {
      // Create the vertices defined so far, with any attributes that are given per vertex.
      for (; num_created_vertices < ar_p.num(); num_created_vertices++) {
        const int i = num_created_vertices;
        Vertex v = create_vertex();
        set_point(v, ar_p[i]);
        string sinfo;
        if (ar_vt.num() == ar_p.num()) sinfo += sform(" uv=%s", csform_vec(str, ar_vt[i]));
        if (ar_vn.num() == ar_p.num()) sinfo += sform(" normal=%s", csform_vec(str, ar_vn[i]));
        if (ar_vrgb.num() == ar_p.num()) sinfo += sform(" rgb=%s", csform_vec(str, ar_vrgb[i]));
        if (sinfo != "") set_string(v, sinfo.c_str() + 1);
      }
      corners.init(0);
      while (more_values(sf)) {
        Vec3<int> corner{0, 0, 0};
        corner[0] = fix_index(int_from_chars(sf), ar_p.num(), line);
        if (*sf == '/') {
          sf++;
          if (*sf != '/' && *sf != ' ' && *sf != '\t' && *sf)
            corner[1] = fix_index(int_from_chars(sf), ar_vt.num(), line);
          if (*sf == '/') {
            sf++;
            if (*sf != ' ' && *sf != '\t' && *sf) corner[2] = fix_index(int_from_chars(sf), ar_vn.num(), line);
          }
        }
        corners.push(corner);
      }
      const int fid = face_smoothing.num() + 1;  // Faces are numbered as in the former script obj_to_mesh.
      face_smoothing.push(smoothing);
      material_counts[material]++;
      va.init(0);
      for (const Vec3<int>& corner : corners) va.push(id_vertex(corner[0]));
      if (va.num() < 3 || !legal_create_face(va)) {
        num_illegal++;
        return;
      }
      Face f = create_face_private(fid, va);
      set_string(f, sform(R"(mat="%s" groups="%s")", material.c_str(), groups.c_str()).c_str());
      for_int(j, corners.num()) {
        const Vec3<int>& corner = corners[j];
        string sinfo;
        if (corner[1]) {
          if (ar_vt.num() != ar_p.num()) {
            sinfo += sform(" uv=%s", csform_vec(str, ar_vt[corner[1] - 1]));
          } else if (corner[1] != corner[0]) {
            assertnever("obj: per-vertex texture coordinate index differs in line '" + line + "'");
          }
        }
        if (corner[2]) {
          if (ar_vn.num() != ar_p.num()) {
            sinfo += sform(" normal=%s", csform_vec(str, ar_vn[corner[2] - 1]));
          } else if (corner[2] != corner[0]) {
            assertnever("obj: per-vertex normal index differs in line '" + line + "'");
          }
        }
        if (sinfo != "") set_string(this->corner(va[j], f), sinfo.c_str() + 1);
      }
    } else if (line == "s" || line.starts_with("s ")) {
      string name = line.size() > 2 ? line.substr(2) : "1";
      while (name != "" && (name.back() == ' ' || name.back() == '\t' || name.back() == '\r')) name.pop_back();
      if (name == "" || name == "0") name = name == "0" ? "off" : "1";
      bool is_new;
      int& id = smoothing_ids.enter(name, smoothing_ids.num(), is_new);
      smoothing = id;
    } else if (line == "g" || line.starts_with("g ")) {
      groups = line.size() > 2 ? line.substr(2) : "default";
      while (groups != "" && (groups.back() == ' ' || groups.back() == '\r')) groups.pop_back();
      if (groups == "") groups = "default";
    } else if (const char* smtl = after_prefix(s0, "usemtl ")) {
      material = smtl;
      while (material != "" && (material.back() == ' ' || material.back() == '\r')) material.pop_back();
    } else if (const char* smrgb = after_prefix(s0, "#MRGB ")) {  // ZBrush colors: MMRRGGBB per vertex (MM is a mask).
      for (; std::isxdigit(static_cast<unsigned char>(smrgb[0])) && strlen(smrgb) >= 8; smrgb += 8) {
        Vector rgb;
        for_int(c, 3) rgb[c] = float(std::stoi(string(smrgb + 2, 6).substr(size_t{2} * c, 2), nullptr, 16)) / 255.f;
        ar_vrgb.push(rgb);
      }
    } else {
      const size_t i = line.find_first_not_of(" \t\r");
      if (i == string::npos || line[i] == '#') return;  // A blank line or a comment.
      if (recognize_line(line)) assertnever("obj: unexpected mesh line '" + line + "'");
      // Lines "mtllib" (whose material colors are not read), "o", "l", "vp", etc. are ignored.
      num_ignored++;
    }
  };
  process(first_line);
  for (string line; my_getline(is, line, false);) {
    process(line);
    num_lines++;
  }
  dummy_use(num_lines);
  showdf("obj: vertices v=%d vt=%d vn=%d vrgb=%d, faces=%d\n", ar_p.num(), ar_vt.num(), ar_vn.num(), ar_vrgb.num(),
         face_smoothing.num());
  if (num_ignored) showdf("obj: ignored %d lines (e.g., mtllib, o, l)\n", num_ignored);
  if (num_illegal) {
    Warning("obj: skipping illegal faces");
    showdf("(obj: skipped %d illegal faces)\n", num_illegal);
  }
  Array<string> material_names(material_counts.keys());
  sort(material_names);  // (The Map order is unspecified.)
  for (const string& mat : material_names) showdf("obj: material \"%s\" %d\n", mat.c_str(), material_counts.get(mat));
  // Without normals, the edges between faces of different smoothing groups (or with smoothing off) are sharp.
  if (ar_vn.empty() && ranges::any_of(face_smoothing, [](int sg) { return sg != k_default_smoothing; })) {
    int num_sharp = 0;
    for (Edge e : edges()) {
      Face f1 = face1(e), f2 = face2(e);
      if (!f2) continue;
      const int sg1 = face_smoothing[face_id(f1) - 1], sg2 = face_smoothing[face_id(f2) - 1];
      if (sg1 != k_no_smoothing && sg1 == sg2) continue;
      update_string(e, "sharp", "");
      flags(e).flag(eflag_sharp) = true;
      num_sharp++;
    }
    showdf("obj: %d sharp edges\n", num_sharp);
  }
}

GMesh GMesh::from_ply(const PlyMesh& ply) {
  GMesh mesh;
  mesh.init_from_ply(ply);
  return mesh;
}

void GMesh::init_from_ply(const PlyMesh& ply) {
  assertx(empty());
  const int nw = ply.points.num();
  const bool has_wid = ply.wedge_vertex.num() > 0;
  const int nv = ply.num_mesh_vertices();
  Array<int> num_wedges(nv, 0), first_wedge(nv, -1);
  for_int(w, nw) {
    const int v = has_wid ? ply.wedge_vertex[w] : w;
    if (num_wedges[v]++ == 0) {
      first_wedge[v] = w;
    } else if (ply.points[w] != ply.points[first_wedge[v]]) {
      assertnever(sform("ply: the wedges %d and %d of vertex %d have different positions", first_wedge[v], w, v));
    }
  }
  for_int(v, nv) if (!num_wedges[v]) assertnever(sform("ply: vertex %d has no wedges", v));
  // The wedges of each vertex, to find the keys whose values are the same on all the wedges of a vertex.
  Array<int> wedge_start(nv + 1, 0), vertex_wedges(nw);
  for_int(v, nv) wedge_start[v + 1] = wedge_start[v] + num_wedges[v];
  {
    Array<int> next(wedge_start.head(nv));
    for_int(w, nw) vertex_wedges[next[has_wid ? ply.wedge_vertex[w] : w]++] = w;
  }
  const auto is_uniform = [&](const PlyMesh::WedgeKey& wedge_key, int v) {
    const int stride = max(wedge_key.dim, 1);
    const CArrayView<float> values0 = wedge_key.values.segment(vertex_wedges[wedge_start[v]] * stride, stride);
    for (int i = wedge_start[v] + 1; i < wedge_start[v + 1]; i++) {
      const CArrayView<float> values = wedge_key.values.segment(vertex_wedges[i] * stride, stride);
      if (std::memcmp(values0.data(), values.data(), stride * sizeof(float)) != 0) return false;
    }
    return true;
  };
  // The attribute string of wedge w of vertex v: all its attributes for a vertex with a single wedge; else, for the
  // vertex string, the keys of "wedge_<key>" elements that are the same on all its wedges (e.g., a flag such as
  // "global"), and for the corner strings, the other attributes.
  enum class EPart { all, vertex, corner };
  string str;
  const auto wedge_string = [&](int w, EPart part, int v) -> const string& {
    LineWriter lw;
    const auto key = [&](const char* name) {
      if (lw.view() != "") lw << ' ';
      lw << name;
    };
    const auto vec = [&](const char* name, CArrayView<float> values) {
      key(name);
      lw << "=(";
      for_int(c, values.num()) {
        if (c) lw << ' ';
        lw << values[c];
      }
      lw << ')';
    };
    if (part != EPart::vertex) {
      if (has_wid) {
        key("wid=");
        lw << w + 1;
      }
      if (ply.normals.num()) vec("normal", ply.normals[w]);
      if (ply.colors.num()) {
        key(sform("rgb=(%.3f %.3f %.3f)", ply.colors[w][0] / 255.f, ply.colors[w][1] / 255.f, ply.colors[w][2] / 255.f)
                .c_str());
      }
      if (ply.uvs.num()) vec("uv", ply.uvs[w]);
    }
    for (const PlyMesh::WedgeKey& wedge_key : ply.wedge_keys) {
      if (part != EPart::all && is_uniform(wedge_key, v) != (part == EPart::vertex)) continue;
      if (wedge_key.dim == 0) {
        if (wedge_key.values[w]) key(wedge_key.name.c_str());
        continue;
      }
      const CArrayView<float> values = wedge_key.values.slice(w * wedge_key.dim, (w + 1) * wedge_key.dim);
      if (ranges::all_of(values, [](float value) { return std::isnan(value); })) continue;  // An absent attribute.
      if (wedge_key.dim == 1) {
        key(wedge_key.name.c_str());
        lw << '=' << values[0];
      } else {
        vec(wedge_key.name.c_str(), values);
      }
    }
    str = lw.view();
    return str;
  };
  const auto mesh_vertex_id = [&](int v) { return ply.vertex_ids.num() ? ply.vertex_ids[v] : v + 1; };
  for_int(v, nv) {
    Vertex gv = create_vertex_private(mesh_vertex_id(v));
    set_point(gv, ply.points[first_wedge[v]]);
    const string& s = wedge_string(first_wedge[v], num_wedges[v] == 1 ? EPart::all : EPart::vertex, v);
    if (s != "") {
      set_string(gv, s.c_str());
      if (string_has_key(s.c_str(), "cusp")) flags(gv).flag(vflag_cusp) = true;
    }
  }
  Array<Vertex> va;
  int num_illegal = 0;
  for_int(f, ply.num_faces()) {
    const CArrayView<int> wedges = ply.face(f);
    va.init(0);
    for (const int w : wedges) va.push(id_vertex(mesh_vertex_id(has_wid ? ply.wedge_vertex[w] : w)));
    if (va.num() < 3 || !legal_create_face(va)) {
      num_illegal++;
      continue;
    }
    Face gf = create_face_private(ply.face_ids.num() ? ply.face_ids[f] : f + 1, va);
    const int matid = ply.face_matid(f);
    if (matid < ply.materials.num() && ply.materials[matid] != "") set_string(gf, ply.materials[matid].c_str());
    for_int(j, wedges.num()) {
      const int w = wedges[j];
      Corner c = corner(va[j], gf);
      if (const int v = has_wid ? ply.wedge_vertex[w] : w; num_wedges[v] > 1) {
        const string& s = wedge_string(w, EPart::corner, v);
        if (s != "") set_string(c, s.c_str());
      }
      if (ply.corner_uvs.num()) {
        string str2;
        update_string(c, "uv", csform_vec(str2, ply.corner_uvs[ply.face_start[f] + j]));
      }
    }
  }
  if (num_illegal) {
    Warning("ply: skipping illegal faces");
    showdf("(ply: skipped %d illegal faces)\n", num_illegal);
  }
}

// Get the string within the braces.  Note the side-effect on the argument `s_const`!
// This function is copied elsewhere too.
static const char* get_sinfo(const char* s_const) {
  char* s = const_cast<char*>(s_const);
  while (std::isspace(static_cast<unsigned char>(*s))) s++;
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
        InlinedArray<Vertex, 6> va;
        for (;;) {
          while (std::isspace(static_cast<unsigned char>(*s))) s++;
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
        if (!e) {
          Warning("GMesh::read_line(): Did not find edge in mesh");
        } else if (sinfo) {
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
    default: break;
  }
  if (Warning("GMesh::read: cannot parse line")) SHOW(sline);
}

bool GMesh::recognize_line(std::string_view line) {
  static constexpr auto k_prefixes =
      to_Vec<std::string_view>({"Vertex ", "Face ", "Corner ", "Edge ", "MVertex ", "CVertex ", "DVertex ", "DFace ",
                                "Ecol ", "Eswa ", "Espl ", "Vspl ", "Vmerge "});
  return ranges::any_of(k_prefixes, [&](std::string_view prefix) { return line.starts_with(prefix); });
}

void GMesh::write(std::ostream& os) const {
  LineWriter w;
  for (Vertex v : ordered_vertices()) {
    w << "Vertex " << vertex_id(v) << ' ';
    const Point& p = point(v);
    for_int(c, 3) w << ' ' << p[c];
    if (const char* sinfo = get_string(v)) w << " {" << sinfo << '}';
    w.write_line(os);
  }

  const Array ar_faces(ordered_faces());
  for (Face f : ar_faces) {
    w << "Face " << face_id(f) << ' ';
    for (Vertex v : vertices(f)) w << ' ' << vertex_id(v);
    if (const char* sinfo = get_string(f)) w << " {" << sinfo << '}';
    w.write_line(os);
  }

  for (Edge e : edges()) {
    const char* sinfo = get_string(e);
    if (!sinfo) continue;
    w << "Edge " << vertex_id(vertex1(e)) << ' ' << vertex_id(vertex2(e)) << " {" << sinfo << '}';
    w.write_line(os);
  }

  for (Face f : ar_faces) {
    for (Corner c : corners(f)) {
      const char* sinfo = get_string(c);
      if (!sinfo) continue;
      w << "Corner " << vertex_id(corner_vertex(c)) << ' ' << face_id(f) << " {" << sinfo << '}';
      w.write_line(os);
    }
  }
  assertx(os.flush());
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
  parse_key_vec(get_string(f), "rgb", fcold);  // Else unmodified.
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

// Override Mesh members.
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
  const int ids = vertex_id(vs), idt = vertex_id(vt);
  // Compute the geometry for the unified vertex (vs).
  const int isbs = is_boundary(vs), isbt = is_boundary(vt), sumb = isbs + isbt;
  const Point p = sumb == 0 || sumb == 2 ? Point(interp(point(vs), point(vt))) : isbs ? point(vs) : point(vt);
  Set<Vertex> vsharp;
  for (Edge ee : edges(vt))
    if (ee != e && flags(ee).flag(eflag_sharp)) vsharp.enter(opp_vertex(vt, ee));
  Mesh::collapse_edge_vertex(e, vs);  // Here, vs is kept.
  // e = nullptr;  // Now undefined.
  set_point(vs, p);  // (_os == nullptr)
  for (Vertex v : vsharp) {
    Edge ee = edge(vs, v);
    if (!is_boundary(ee)) flags(ee).flag(eflag_sharp) = true;
  }
  for (Edge ee : edges(vs)) {
    // Test (vs, vo1), (vs, vo2) to remove eflag_sharp if now boundary.
    if (is_boundary(ee) && flags(ee).flag(eflag_sharp)) flags(ee).flag(eflag_sharp) = false;
  }
  if (tos) {
    _os = tos;
    *_os << "Ecol " << ids << ' ' << idt << '\n';
  }
}

void GMesh::collapse_edge_vertex_saving_attribs(Edge e, Vertex vs) {
  Vertex v2 = opp_vertex(vs, e);
  string str;
  const bool v_has_corner_normals =
      ranges::any_of(corners(vs), [&](Corner c) { return GMesh::string_has_key(get_string(c), "normal"); });
  const bool v_has_corner_uvs =
      ranges::any_of(corners(vs), [&](Corner c) { return GMesh::string_has_key(get_string(c), "uv"); });
  Map<Face, string> face_normal;
  Map<Face, string> face_uv;
  if (v_has_corner_normals)
    for (Corner c : corners(v2))
      if (Face f = corner_face(c); f != face1(e) && f != face2(e))
        if (const char* s = corner_key(str, c, "normal")) face_normal.enter(f, s);
  if (v_has_corner_uvs)
    for (Corner c : corners(v2))
      if (Face f = corner_face(c); f != face1(e) && f != face2(e))
        if (const char* s = corner_key(str, c, "uv")) face_uv.enter(f, s);
  collapse_edge_vertex(e, vs);  // Vertex v2 is destroyed.
  if (v_has_corner_normals) {
    for (Corner c : corners(vs))
      if (const string* s = face_normal.find_ptr(corner_face(c))) update_string(c, "normal", s->c_str());
  } else {
    for (Corner c : corners(vs)) update_string(c, "normal", nullptr);
  }
  if (v_has_corner_uvs) {
    for (Corner c : corners(vs))
      if (const string* s = face_uv.find_ptr(corner_face(c))) update_string(c, "uv", s->c_str());
  } else {
    for (Corner c : corners(vs)) update_string(c, "uv", nullptr);
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
  const bool flag_e = flags(e).flag(eflag_sharp);
  auto fstring1 = make_unique_c_string(get_string(f1));  // Often nullptr.
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
         << vertex_id(vn) << '\n';
  }
  return vn;
}

void GMesh::merge_vertices(Vertex vs, Vertex vt) {
  if (_os) *_os << "Vmerge " << vertex_id(vs) << ' ' << vertex_id(vt) << '\n';
  Mesh::merge_vertices(vs, vt);
}

Vertex GMesh::center_split_face(Face f) {
  auto fstring = make_unique_c_string(get_string(f));  // Often nullptr.
  Map<Vertex, string> mvs;
  for (Corner c : corners(f)) {
    Vertex v = corner_vertex(c);
    if (const char* s = get_string(c)) mvs.enter(v, s);
  }
  Polygon poly;
  polygon(f, poly);
  Vector sum_normal{};
  bool have_normal = true;
  Vector sum_rgb{};
  bool have_rgb = true;
  Uv sum_uv{};
  bool have_uv = true;
  for (Corner c : corners(f)) {
    if (Vector normal; parse_corner_key_vec(c, "normal", normal))
      sum_normal += normal;
    else
      have_normal = false;
    if (Vector rgb; parse_corner_key_vec(c, "rgb", rgb))
      sum_rgb += rgb;
    else
      have_rgb = false;
    if (Uv uv; parse_corner_key_vec(c, "uv", uv))
      sum_uv += uv;
    else
      have_uv = false;
  }
  Vertex vn = Mesh::center_split_face(f);
  set_point(vn, mean(poly));
  string str;
  if (have_normal) {
    sum_normal = normalized(sum_normal);
    if (1) sum_normal = poly.get_normal();  // More useful.
    update_string(vn, "normal", csform_vec(str, sum_normal));
  }
  if (have_rgb) update_string(vn, "rgb", csform_vec(str, sum_rgb / float(poly.num())));
  if (have_uv) update_string(vn, "uv", csform_vec(str, sum_uv / float(poly.num())));
  if (fstring)
    for (Face fn : faces(vn)) set_string(fn, fstring.get());
  if (mvs.num()) {
    for (Face fn : faces(vn)) {
      for (Corner cc : corners(fn)) {
        Vertex vv = corner_vertex(cc);
        if (vv == vn) continue;
        // Each original corner string should get assigned to two different corners.
        if (const string* s = mvs.find_ptr(vv)) set_string(cc, s->c_str());
      }
    }
  }
  return vn;
}

Edge GMesh::split_face(Face f, Vertex v1, Vertex v2) {
  auto fstring = make_unique_c_string(get_string(f));  // Often nullptr.
  Map<Vertex, unique_ptr<char[]>> mvs;
  for (Corner c : corners(f)) {
    Vertex v = corner_vertex(c);
    if (get_string(c)) mvs.enter(v, make_unique_c_string(get_string(c)));
  }
  Edge en = Mesh::split_face(f, v1, v2);
  // f = nullptr;  // Now undefined.
  if (fstring)
    for (Face ff : faces(en)) set_string(ff, fstring.get());
  if (mvs.num()) {
    for (Face ff : faces(en)) {
      for (Corner cc : corners(ff)) {
        Vertex vv = corner_vertex(cc);
        set_string(cc, mvs.retrieve(vv).get());  // Could be nullptr.
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
  // Lose strings.
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Vertex vn = Mesh::insert_vertex_on_edge(e);
  set_point(vn, interp(point(v1), point(v2)));
  return vn;
}

Edge GMesh::remove_vertex_between_edges(Vertex vr) {
  // Lose strings.
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

void GMesh::show_keys(Vertex v) const {
  SHOW(v, get_string(v));
  for (Corner c : corners(v)) SHOW(c, get_string(c));
}

}  // namespace hh
