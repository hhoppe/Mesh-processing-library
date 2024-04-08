// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_GMESH_H_
#define MESH_PROCESSING_LIBHH_GMESH_H_

#include <cstring>  // std::memcpy()

#include "libHh/Mesh.h"
#include "libHh/Polygon.h"

#if 0
{
  Mesh mesh;
  Vertex v1 = mesh.create_vertex();
  mesh.set_point(v1, Point(1.f, 2.f, 3.f));
  Vertex v2 = mesh.create_vertex();
  mesh.set_point(v2, Point(1.f, 4.f, 5.f));
  Vertex v3 = mesh.create_vertex();
  mesh.set_point(v3, Point(1.f, 6.f, 7.f));
  Face f1 = mesh.create_face(v1, v2, v3);
  for (Face f : mesh.ordered_faces()) {
    for (Vertex v : mesh.vertices(f)) process(f, mesh.point(v));
  }
}
#endif

namespace hh {

// *** See documentation on MESH FILE FORMAT at the end of this file.

class WA3dStream;
class A3dElem;
struct A3dVertexColor;

// Corner data is currently not handled

// A Mesh with geometric structure (Point at each Vertex) and with strings at each mesh element.
class GMesh : public Mesh {
 public:
  GMesh() = default;
  GMesh(GMesh&& m) noexcept { swap(*this, m); }  // = default?
  explicit GMesh(std::istream& is);
  ~GMesh() override = default;
  GMesh& operator=(GMesh&& m) noexcept {
    clear();
    swap(*this, m);
    return *this;
  }

  // ** Extend functionality:
  void copy(const GMesh& m);  // carries flags (but not sac fields), hence not named operator=().
  void merge(const GMesh& mo, Map<Vertex, Vertex>* mvvn = nullptr);
  void destroy_vertex(Vertex v) override;
  void destroy_face(Face f) override;
  // do appropriate actions with geometry, eflag_sharp, and face strings
  void collapse_edge_vertex(Edge e, Vertex vs) override;
  void collapse_edge(Edge e) override;
  Vertex split_edge(Edge e, int id = 0) override;
  Edge swap_edge(Edge e) override;
  //
  Vertex split_vertex(Vertex v1, Vertex vs1, Vertex vs2, int v2i) override;
  void merge_vertices(Vertex vs, Vertex vt) override;
  Vertex center_split_face(Face f) override;
  Edge split_face(Face f, Vertex v1, Vertex v2) override;
  Face coalesce_faces(Edge e) override;
  Vertex insert_vertex_on_edge(Edge e) override;
  Edge remove_vertex_between_edges(Vertex vr) override;
  Array<Vertex> fix_vertex(Vertex v) override;

  // ** Geometry:
  const Point& point(Vertex v) const { return v->_point; }
  void set_point(Vertex v, const Point& p);
  void polygon(Face f, Polygon& poly) const;
  Vec3<Point> triangle_points(Face f) const;
  float length2(Edge e) const;
  float length(Edge e) const;
  float area(Face f) const;
  void transform(const Frame& frame);

  // ** Strings:
  const char* get_string(Vertex v) const { return v->_string.get(); }
  const char* get_string(Face f) const { return f->_string.get(); }
  const char* get_string(Edge e) const { return e->_string.get(); }
  const char* get_string(Corner c) const { return c->_string.get(); }
  unique_ptr<char[]> extract_string(Vertex v) { return std::move(v->_string); }
  unique_ptr<char[]> extract_string(Face f) { return std::move(f->_string); }
  unique_ptr<char[]> extract_string(Edge e) { return std::move(e->_string); }
  unique_ptr<char[]> extract_string(Corner c) { return std::move(c->_string); }
  static bool string_has_key(const char* ss, const char* key);
  static const char* string_key(string& str, const char* ss, const char* key);
  const char* corner_key(string& str, Corner c, const char* key) const;             // Corner | Vertex
  bool parse_corner_key_vec(Corner c, const char* key, ArrayView<float> ar) const;  // Corner | Vertex
  // copies string
  void set_string(Vertex v, const char* s) { v->_string = make_unique_c_string(s); }
  void set_string(Face f, const char* s) { f->_string = make_unique_c_string(s); }
  void set_string(Edge e, const char* s) { e->_string = make_unique_c_string(s); }
  void set_string(Corner c, const char* s) { c->_string = make_unique_c_string(s); }
  void set_string(Vertex v, unique_ptr<char[]> s) { v->_string = std::move(s); }
  void set_string(Face f, unique_ptr<char[]> s) { f->_string = std::move(s); }
  void set_string(Edge e, unique_ptr<char[]> s) { e->_string = std::move(s); }
  void set_string(Corner c, unique_ptr<char[]> s) { c->_string = std::move(s); }
  static string string_update(const string& s, const char* key, const char* val);
  void update_string(Vertex v, const char* key, const char* val);
  void update_string(Face f, const char* key, const char* val);
  void update_string(Edge e, const char* key, const char* val);
  void update_string(Corner c, const char* key, const char* val);
  static void update_string_ptr(unique_ptr<char[]>& ss, const char* key, const char* val);

  // ** Standard I/O for my meshes (see also GMesh(std::istream&)) (see format below):
  void read_line(char* s);      // no '\n' required
  static bool recognize_line(const char* s);
  void write(std::ostream& os) const;
  void write(WA3dStream& oa3d, const A3dVertexColor& col) const;
  void write_face(WA3dStream& oa3d, A3dElem& el, const A3dVertexColor& col, Face f) const;
  std::ostream* record_changes(std::ostream* pos);  // pos may be nullptr, ret old

  // ** Flag bits:
  // Predefined {Vertex, Face, Edge} flag bits; vflag_cusp and eflag_sharp are parsed when reading a mesh.
  static const FlagMask vflag_cusp;   // "cusp" on Vertex
  static const FlagMask eflag_sharp;  // "sharp" on Edge

  // ** Discouraged:
  Vertex create_vertex_private(int id) override;
  Face create_face_private(int id, CArrayView<Vertex> va) override;  // or die

  // ** Misc:
  friend void swap(GMesh& l, GMesh& r) noexcept;

 private:
  std::ostream* _os{nullptr};  // for record_changes
  mutable Polygon _tmp_poly;
};

// Format a vector string "(%g ... %g)" with ar.num() == 1..4
const char* csform_vec(string& str, CArrayView<float> ar);

// Parse a vector from a {key=value}+ string
bool parse_key_vec(const char* ss, const char* key, ArrayView<float> ar);

// I/O Mesh Format (Vertices and Faces must fit on one line)
//   (vertex numbers begin with 1)
//   Vertex vi  x y z [{other_info}]
//   Face fi  vi1 vi2 ... vin [{other_info}]
//   MVertex vi newx newy newz
//   Ecol v1 v2
//   Eswa v1 v2
//   Espl v1 v2 vnew
// Example:
//   Vertex 1  1.5e2 0 1.5 {normal=(0,1,0)}
//   Vertex 2  0 1.5 0
//   Face 1  1 2 3
//   Face 2  2 3 4 5 {color=red, phong=2}
//  fi may be zero, in which case a number is assigned

class StringKeyIter {
 public:
  explicit StringKeyIter(const char* str) : _str(str), _s(str) {}
  bool next(const char*& kb, int& kl, const char*& vb, int& vl);

 private:
  const char* _str;
  const char* _s;
  // Default operator=() and copy_constructor are safe.
};

// Iterate through an info string, calling func for each attribute with the char* and length of its key and value.
template <typename Func = bool(const char* keystr, int keylen, const char* valstr, int vallen)>
void for_cstring_key_value_ptr(const char* str, Func func) {
  StringKeyIter ski(str);
  const char* keystr;
  int keylen;
  const char* valstr;
  int vallen;
  while (ski.next(keystr, keylen, valstr, vallen)) {
    if (func(keystr, keylen, valstr, vallen)) break;
  }
}

// Iterate through an info string, calling func for each attribute after setting arrays key and val appropriately.
template <typename Func = void()>
void for_cstring_key_value(const char* str, Array<char>& key, Array<char>& val, Func func) {
  for_cstring_key_value_ptr(str, [&](const char* keystr, int keylen, const char* valstr, int vallen) {
    key.init(keylen + 1);
    std::memcpy(key.data(), keystr, keylen);
    key[keylen] = 0;
    val.init(vallen + 1);
    std::memcpy(val.data(), valstr, vallen);
    val[vallen] = 0;
    func();
    return false;
  });
}

inline Vec3<Point> GMesh::triangle_points(Face f) const {
  Vec3<Point> pa;
  HEdge he = herep(f), he0 = he;
  pa[0] = he->_vert->_point, he = he->_next;
  pa[1] = he->_vert->_point, he = he->_next;
  pa[2] = he->_vert->_point, he = he->_next;
  assertx(he == he0);  // is_triangle()
  return pa;
}

//----------------------------------------------------------------------------

// MESH FILE FORMAT
//
// Set of one-line records.  Each record contains either:
//
// # a comment with '#' in first column
// Vertex vi x y z [{other_info}]
// Face fi vi1 vi2 vi3 [{other_info}]
// Corner vi fi {other_info}
// Edge vi1 vi2 {other_info}
//
// The values vi and fi are vertex and face indices respectively, starting at 1.
//
// other_info: a string containing a list of attributes separated by exactly
//  one space; each attribute has the form:
//  - attrib=value
//  - attrib="some string value"
//  - attrib=(some string value)
//  - attrib                (its value is "")
//
// As mentioned in the PM paper, scalar attributes are associated with the mesh
//  vertices, or with the corners adjacent to a vertex.
// (If all corner attributes about a vertex are the same, the attributes can be
//  moved to the vertex.)
// Also, discrete attributes are associated with the faces of the mesh.
//
// My simplification procedure recognizes the following attributes on meshes:
//  - normal=(x y z)    normals (on vertices or corners)
//  - uv=(u v)          texture coordinates (on vertices or corners)
//  - rgb=(r g b)       color values (on vertices, corners, or on faces)
//  - wid=id            wedge_identifier (on vertices or corners)
//  - any set of attributes on faces (matid=d mat="material name", etc.)
//  - no attributes on edges (actually, it supports "sharp" edges on input).
//
// The wedge identifiers need not be present in the input mesh.
// They are created by the simplification procedure, and identify those corners
// about a vertex which share common attributes.  These wedge identifiers could
// be used for more efficient storage.
//
// In addition, meshes which are in fact geomorphs contain "ancestor"
// attributes (as they are called in the paper) to enable smooth visual
// interpolation.  These attributes are:
//
//  - Opos=(x y z)      the old position of the vertex
//  - Onormal=(x y z)   the old normal (on vertex or corner)
//  - Ouv=(u v)         the old texture coordinates (on vertex or corner)
//  - Orgb=(r g b)      the old color values (on vertex or corner)
//
// Example:
//
// # Some random mesh.
// Vertex 1  1.1e0 0 1.5 {normal=(0 1 0) uv=(0 0)}
// Vertex 2  0 1.5 0 {normal=(1 0 0) uv=(0.5 0.5)}
// Vertex 3  1 1.5 0
// Vertex 4  1 1.5 1 {normal=(1 0 0) uv=(0.5 0.5)}
// Face 1  1 2 3 {mat="red_brick17" rgb=(1 0 0)}
// Face 2  3 2 4 {mat="grey_cement" rgb=(.5 .5 .5)}
// Corner 3 1 {normal=(1 0 0) uv=(0.5 0.5)}
// Corner 3 2 {normal=(0 1 0) uv=(0 0.5)}
//
// (For exact specifications, refer to GMesh.cpp)

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_GMESH_H_
