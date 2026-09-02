// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MESH_H_
#define MESH_PROCESSING_LIBHH_MESH_H_

#include "libHh/Array.h"
#include "libHh/Flags.h"
#include "libHh/Geometry.h"  // because of Point, too bad.
#include "libHh/Map.h"
#include "libHh/PArray.h"
#include "libHh/Pool.h"
#include "libHh/Sac.h"

#if 0
{
  Mesh mesh;
  Vertex v1 = mesh.create_vertex(), v2 = mesh.create_vertex(), v3 = mesh.create_vertex();
  Face f1 = mesh.create_face(v1, v2, v3);
  for (Face f : mesh.ordered_faces())
    for (Vertex v : mesh.vertices(f)) process(f, v);
}
#endif

namespace hh {

class Random;

// Mesh: a set of Vertices, Faces, and Edges and their topological relations.
// Properties:
//   (1a) - vertices appear at most once per face.
//   (1b) - an oriented edge (2 consecutive vertices of a face) appears in at most one face; hence mesh is orientable.
// A Mesh must always satisfy (1a) and (1b); such a mesh is called "legal".
//
//   (2a) - vertices are nice (contain at most 1 (possibly partial) face ring).
//   (2b) - faces are nice: (a, b, c) implies no (a, c, b).
// A Mesh is "nice" if it also satisfies (2a) and (2b) -- basically, if it is everywhere a 2D manifold.
//
//   (3a) - all faces are triangular.
// A Mesh is a "nice triangular mesh" if in addition it satisfies (3a).
//
// MVertex allocates space for Point, which is used later in GMesh.
// MVertex, MFace, MEdge, MHEdge allocate space for string, also used in GMesh.

class Mesh : noncopyable {
 public:  // for use by Sac
  struct MHEdge;
  struct MVertex;
  struct MFace;
  using MCorner = MHEdge;
  struct MEdge;

  using HEdge = MHEdge*;
  using Vertex = MVertex*;
  using Face = MFace*;
  using Corner = HEdge;
  using Edge = MEdge*;
  friend void swap(Mesh& l, Mesh& r) noexcept;

 private:
  struct Edges_range;
  struct OrderedVertices_range;
  struct OrderedFaces_range;
  struct VV_range;
  struct VE_range;
  struct FC_range;
  struct WV_range;
  struct WE_range;
  struct WC_range;

 public:
  Mesh();
  Mesh(Mesh&& m) noexcept { swap(*this, m); }
  virtual ~Mesh() { clear(); }
  Mesh& operator=(Mesh&& m) noexcept { return clear(), swap(*this, m), *this; }
  void clear();
  void copy(const Mesh& m);  // not a GMesh!  carries flags (but not sac fields), hence not named operator=().

  // ** Raw manipulation functions, may lead to non-nice Meshes:
  // always legal
  Vertex create_vertex() { return create_vertex_private(_vertexnum); }
  // die if degree(v) > 0
  virtual void destroy_vertex(Vertex v);
  // ret false if duplicate vertices or if existing edge
  [[nodiscard]] bool legal_create_face(CArrayView<Vertex> va) const;
  // die if !legal_create_face()
  Face create_face(CArrayView<Vertex> va) { return create_face_private(_facenum, va); }
  Face create_face(Vertex v1, Vertex v2, Vertex v3) { return create_face(V(v1, v2, v3)); }
  // always legal
  virtual void destroy_face(Face f);

  // ** Vertex:
  [[nodiscard]] bool is_nice(Vertex v) const;
  [[nodiscard]] int degree(Vertex v) const;             // == number of adjacent vertices/edges
  [[nodiscard]] int num_boundaries(Vertex v) const;     // 0/1 for a nice vertex
  [[nodiscard]] bool is_boundary(Vertex v) const;       // is_nice(v), degree(v) > 0
  [[nodiscard]] Edge opp_edge(Vertex v, Face f) const;  // is_triangle(f); slow
  [[nodiscard]] Vertex opp_vertex(Vertex v, Edge e) const;
  // all most_clw and most_ccw assert is_nice(v)
  // move about vertices adjacent to a vertex
  [[nodiscard]] Vertex most_clw_vertex(Vertex v) const;          // if !bnd, ret any; may return nullptr
  [[nodiscard]] Vertex most_ccw_vertex(Vertex v) const;          // if !bnd, ret any; may return nullptr
  [[nodiscard]] Vertex clw_vertex(Vertex v, Vertex vext) const;  // slow; may return nullptr
  [[nodiscard]] Vertex ccw_vertex(Vertex v, Vertex vext) const;  // slow; may return nullptr
  // move about faces adjacent to a vertex
  [[nodiscard]] Face most_clw_face(Vertex v) const;     // if !bnd, ret any; may return nullptr
  [[nodiscard]] Face most_ccw_face(Vertex v) const;     // if !bnd, ret any; may return nullptr
  [[nodiscard]] Face clw_face(Vertex v, Face f) const;  // slow; may return nullptr
  [[nodiscard]] Face ccw_face(Vertex v, Face f) const;  // slow; may return nullptr
  // move about edges adjacent to a vertex
  [[nodiscard]] Edge most_clw_edge(Vertex v) const;     // if !bnd, ret any; may return nullptr
  [[nodiscard]] Edge most_ccw_edge(Vertex v) const;     // if !bnd, ret any; may return nullptr
  [[nodiscard]] Edge clw_edge(Vertex v, Edge e) const;  // may return nullptr
  [[nodiscard]] Edge ccw_edge(Vertex v, Edge e) const;  // may return nullptr
  // get face relative to vertex
  [[nodiscard]] Face ccw_face(Vertex v, Edge e) const;  // may return nullptr
  [[nodiscard]] Face clw_face(Vertex v, Edge e) const;  // may return nullptr

  // ** Face:
  [[nodiscard]] bool is_nice(Face f) const;
  [[nodiscard]] int num_vertices(Face f) const;
  [[nodiscard]] bool is_triangle(Face f) const {
    HEdge he = herep(f);
    return he->_next->_next->_next == he;
  }
  [[nodiscard]] bool is_boundary(Face f) const;         // == has a boundary vertex
  [[nodiscard]] Face opp_face(Face f, Edge e) const;    // ret nullptr if is_boundary(e)
  [[nodiscard]] Face opp_face(Vertex v, Face f) const;  // is_triangle(f); ret nullptr if none
  // ccw order
  void get_vertices(Face f, Array<Vertex>& va) const;
  [[nodiscard]] Vec3<Vertex> triangle_vertices(Face f) const;
  [[nodiscard]] Vec3<Corner> triangle_corners(Face f) const;
  [[nodiscard]] Vertex vertex(Face f, int i) const;  // die if i >= num_vertices(f)
  // move about a face
  [[nodiscard]] Edge clw_edge(Face f, Edge e) const { return hedge_from_ef(e, f)->_prev->_edge; }
  [[nodiscard]] Edge ccw_edge(Face f, Edge e) const { return hedge_from_ef(e, f)->_next->_edge; }
  [[nodiscard]] Vertex clw_vertex(Face f, Vertex v) const { return get_hedge(v, f)->_prev->_vert; }  // slow
  [[nodiscard]] Vertex ccw_vertex(Face f, Vertex v) const { return get_hedge(v, f)->_next->_vert; }  // slow
  [[nodiscard]] Edge clw_edge(Face f, Vertex v) const { return get_hedge(v, f)->_edge; }             // slow
  [[nodiscard]] Edge ccw_edge(Face f, Vertex v) const { return get_hedge(v, f)->_next->_edge; }      // slow

  // ** Edge:
  [[nodiscard]] bool is_boundary(Edge e) const { return !herep(e)->_sym; }
  [[nodiscard]] Vertex vertex1(Edge e) const { return herep(e)->_prev->_vert; }
  [[nodiscard]] Vertex vertex2(Edge e) const { return herep(e)->_vert; }
  [[nodiscard]] Vertex vertex(Edge e, int i) const {
    return ASSERTX(i == 0 || i == 1), i == 0 ? vertex1(e) : vertex2(e);
  }
  [[nodiscard]] Face face1(Edge e) const { return herep(e)->_face; }
  [[nodiscard]] Face face2(Edge e) const {
    HEdge he = herep(e);
    return he->_sym ? he->_sym->_face : nullptr;
  }
  [[nodiscard]] Face face(Edge e, int i) const { return ASSERTX(i == 0 || i == 1), i == 0 ? face1(e) : face2(e); }
  // i == 0 or i == 1; ret nullptr if i == 1 && is_boundary(e)
  [[nodiscard]] Vertex side_vertex1(Edge e) const { return opp_vertex(e, face1(e)); }  // is_triangle(face1())
  [[nodiscard]] Vertex side_vertex2(Edge e) const { return face2(e) ? opp_vertex(e, face2(e)) : nullptr; }
  [[nodiscard]] Vertex side_vertex(Edge e, int i) const {
    return ASSERTX(i == 0 || i == 1), !i ? side_vertex1(e) : side_vertex2(e);
  }
  [[nodiscard]] Vertex opp_vertex(Edge e, Face f) const;                                 // is_triangle(f)
  [[nodiscard]] Edge opp_boundary(Edge e, Vertex v) const;                               // is_boundary(e)
  [[nodiscard]] Edge clw_boundary(Edge e) const { return opp_boundary(e, vertex2(e)); }  // is_boundary(e)
  [[nodiscard]] Edge ccw_boundary(Edge e) const { return opp_boundary(e, vertex1(e)); }  // is_boundary(e)
  [[nodiscard]] Vertex vertex_between_edges(Edge e1, Edge e2) const;

  // ** Corner:
  [[nodiscard]] Corner corner(Vertex v, Face f) const { return get_hedge(v, f); }
  [[nodiscard]] Vertex corner_vertex(Corner c) const { return c->_vert; }
  [[nodiscard]] Face corner_face(Corner c) const { return c->_face; }
  [[nodiscard]] Corner ccw_corner(Corner c) const { return c->_sym ? c->_sym->_prev : nullptr; }  // around vertex
  [[nodiscard]] Corner clw_corner(Corner c) const { return c->_next->_sym; }                      // around vertex
  [[nodiscard]] Corner ccw_face_corner(Corner c) const { return c->_next; }                       // around face
  [[nodiscard]] Corner clw_face_corner(Corner c) const { return c->_prev; }                       // around face
  [[nodiscard]] Corner ccw_corner(Vertex v, Edge e) const {
    HEdge he = hedge_from_ev1(e, v);
    return he ? he->_prev : nullptr;
  }
  [[nodiscard]] Corner clw_corner(Vertex v, Edge e) const { return hedge_from_ev2(e, v); }
  [[nodiscard]] Edge ccw_face_edge(Corner c) const { return c->_next->_edge; }
  [[nodiscard]] Edge clw_face_edge(Corner c) const { return c->_edge; }  // (fastest)

  // ** Other associations:
  // obtain edge from vertices
  [[nodiscard]] Edge query_edge(Vertex v, Vertex w) const;
  [[nodiscard]] Edge edge(Vertex v, Vertex w) const { return assertx(query_edge(v, w)); }
  [[nodiscard]] Edge ordered_edge(Vertex v1, Vertex v2) const;  // asserts it exists, oriented
  // get face from 2 consecutive vertices (ccw order)
  [[nodiscard]] Face face(Vertex v, Vertex vccw) const { return clw_face(vccw, edge(v, vccw)); }  // may return nullptr

  // ** Counting routines (fast):
  [[nodiscard]] bool empty() const { return !num_vertices(); }
  [[nodiscard]] int num_vertices() const { return _id2vertex.num(); }
  [[nodiscard]] int num_faces() const { return _id2face.num(); }
  [[nodiscard]] int num_edges() const { return _nedges; }

  // ** Random access (fast), assert there exist at least one:
  [[nodiscard]] Vertex random_vertex(Random& r) const;
  [[nodiscard]] Face random_face(Random& r) const;
  [[nodiscard]] Edge random_edge(Random& r) const;  // unbiased for a closed triangle mesh

  // ** Flags:
  [[nodiscard]] static FlagMask allocate_flag() {
    static int s_counter;
    return Flags::allocate(s_counter);
  }
  [[nodiscard]] static FlagMask allocate_Vertex_flag() {
    static int s_counter;
    return Flags::allocate(s_counter);
  }
  [[nodiscard]] static FlagMask allocate_Face_flag() {
    static int s_counter;
    return Flags::allocate(s_counter);
  }
  [[nodiscard]] static FlagMask allocate_Edge_flag() {
    static int s_counter;
    return Flags::allocate(s_counter);
  }
  [[nodiscard]] auto& gflags(this auto&& self) { return self._flags; }
  [[nodiscard]] Flags& flags(Vertex v) { return v->_flags; }
  [[nodiscard]] const Flags& flags(Vertex v) const { return v->_flags; }
  [[nodiscard]] Flags& flags(Face f) { return f->_flags; }
  [[nodiscard]] const Flags& flags(Face f) const { return f->_flags; }
  [[nodiscard]] Flags& flags(Edge e) { return e->_flags; }
  [[nodiscard]] const Flags& flags(Edge e) const { return e->_flags; }

  // ** Triangular mesh operations (die if not triangular!):
  // would collapse be legal?
  [[nodiscard]] bool legal_edge_collapse(Edge e) const;
  // would collapse preserve a nice mesh?
  [[nodiscard]] bool nice_edge_collapse(Edge e) const;
  // would edge swap be legal?  (legal implies nice here)
  [[nodiscard]] bool legal_edge_swap(Edge e) const;

  virtual void collapse_edge_vertex(Edge e, Vertex vs);
  // die if !legal_edge_collapse(e)
  // remove f1, [f2], v2, (v2, {*})
  // add (v1, {**})  where {**} = {*}-{v1, vo1, vo2}
  virtual void collapse_edge(Edge e);
  // split_edge(e) always legal
  // remove f1, [f2], (v1, v2)
  // add 2/4 faces, vnew, (vnew, v1), (vnew, v2), (vnew, vo1), [(vnew, vo2)]
  virtual Vertex split_edge(Edge e, int vid = 0);
  // die if !legal_edge_swap(e)
  // remove f1, f2, (v1, v2)
  // add 2 faces, (vo1, vo2)
  virtual Edge swap_edge(Edge e);

  // ** More mesh operations:
  // vs2 can be nullptr, returns v2, leaves hole
  virtual Vertex split_vertex(Vertex v1, Vertex vs1, Vertex vs2, int v2i);
  // replace (vt, {*}) by (vs, {*})
  [[nodiscard]] bool legal_vertex_merge(Vertex vs, Vertex vt);
  virtual void merge_vertices(Vertex vs, Vertex vt);  // die if !legal
  // introduce one center vertex and triangulate face
  virtual Vertex center_split_face(Face f);  // always legal
  // introduce an edge to split face on (v1, v2)
  virtual Edge split_face(Face f, Vertex v1, Vertex v2);
  // remove the consecutive set of edges separating two faces
  //  (may destroy some vertices if > 1 edge shared by two faces)
  [[nodiscard]] bool legal_coalesce_faces(Edge e);
  virtual Face coalesce_faces(Edge e);  // die if !legal
  virtual Vertex insert_vertex_on_edge(Edge e);
  virtual Edge remove_vertex_between_edges(Vertex vr);
  // Separate the vertex into multiple vertices if it is adjacent to disconnected face rings; return new vertices.
  virtual Array<Vertex> fix_vertex(Vertex v);

  // ** Mesh:
  [[nodiscard]] Vertex id_vertex(int i) const { return _id2vertex.get(i); }
  [[nodiscard]] int vertex_id(Vertex v) const { return v->_id; }
  [[nodiscard]] Face id_face(int i) const { return _id2face.get(i); }
  [[nodiscard]] int face_id(Face f) const { return f->_id; }
  [[nodiscard]] Vertex id_retrieve_vertex(int i) const { return _id2vertex.retrieve(i); }
  [[nodiscard]] Face id_retrieve_face(int i) const { return _id2face.retrieve(i); }
  [[nodiscard]] bool is_nice() const;
  void renumber();  // renumber vertices and faces

  // ** Misc:
  void ok() const;             // die if problem
  void valid(Vertex v) const;  // die if invalid
  void valid(Face f) const;    // die if invalid
  void valid(Edge e) const;    // die if invalid
  void valid(Corner c) const;  // die if invalid

  // ** Iterators; can crash if continued after any change in the Mesh:
  // These mesh iterators do not define an order.
  [[nodiscard]] auto vertices() const { return _id2vertex.values(); }
  [[nodiscard]] auto faces() const { return _id2face.values(); }
  [[nodiscard]] Edges_range edges() const { return Edges_range(*this); }
  // These mesh iterators sort by id numbers.
  [[nodiscard]] OrderedVertices_range ordered_vertices() const { return OrderedVertices_range(*this); }
  [[nodiscard]] OrderedFaces_range ordered_faces() const { return OrderedFaces_range(*this); }

  // These vertex iterators do not specify order, and work correctly even on non-nice vertices.
  [[nodiscard]] VV_range vertices(Vertex v) const { return VV_range(*this, v); }
  [[nodiscard]] auto faces(Vertex v) const;
  [[nodiscard]] VE_range edges(Vertex v) const { return VE_range(*this, v); }
  [[nodiscard]] auto corners(Vertex v) const;

  // These vertex iterators go CCW, but require nice vertices.
  [[nodiscard]] WV_range ccw_vertices(Vertex v) const { return WV_range(*this, v); }
  [[nodiscard]] auto ccw_faces(Vertex v) const;
  [[nodiscard]] WE_range ccw_edges(Vertex v) const { return WE_range(*this, v); }
  [[nodiscard]] WC_range ccw_corners(Vertex v) const { return WC_range(*this, v); }

  // Face iterators all go CCW
  [[nodiscard]] auto vertices(Face f) const;
  [[nodiscard]] auto faces(Face f) const;
  [[nodiscard]] auto edges(Face f) const;
  [[nodiscard]] FC_range corners(Face f) const { return FC_range(*this, f); }

  // Edge iterators do not define an order.
  [[nodiscard]] Vec2<Vertex> vertices(Edge e) const;
  [[nodiscard]] PArray<Face, 2> faces(Edge e) const;

 private:
  friend class GMesh;

  // Mesh Iter

  struct Edges_iterator {
    using type = Edges_iterator;
    using iterator_concept = std::forward_iterator_tag;
    using value_type = Edge;
    using difference_type = std::ptrdiff_t;
    explicit Edges_iterator(const Mesh& m) {
      _vcur = m._id2vertex.values().begin();
      _vend = m._id2vertex.values().end();
      next();
    }
    Edges_iterator() = default;
    // _hcur uniquely identifies the position, and is null exactly when exhausted.
    [[nodiscard]] bool operator==(const type& rhs) const { return _hcur == rhs._hcur; }
    [[nodiscard]] bool operator==(std::default_sentinel_t) const { return !_hcur; }
    [[nodiscard]] Edge operator*() const { return ASSERTX(_hcur != _hend), (*_hcur)->_edge; }
    type& operator++() {
      ASSERTX(_hcur);
      ++_hcur;
      next();
      return *this;
    }
    type operator++(int) { return postfix_increment(*this); }
    //
    CArrayView<HEdge>::iterator _hcur{nullptr}, _hend{nullptr};  // _hcur points at current element.
    Map<int, Vertex>::cvalues_iterator _vcur, _vend;             // _vcur points one vertex ahead.
    void next() {
      for (;;) {
        if (_hcur != _hend) {
          if ((*_hcur)->_edge->_herep != *_hcur) {  // Skip over any non-representative half-edge.
            ++_hcur;
            continue;
          }
          return;
        }
        if (_vcur == _vend) break;
        auto& range = (*_vcur)->_arhe;
        ++_vcur;
        _hcur = range.begin();
        _hend = range.end();
      }
      _hcur = _hend = nullptr;  // No element found.
    }
  };

  struct Edges_range : ranges::view_interface<Edges_range> {
    explicit Edges_range(const Mesh& m) : _m(&m) {}
    [[nodiscard]] Edges_iterator begin() const { return Edges_iterator(*_m); }
    [[nodiscard]] std::default_sentinel_t end() const { return {}; }
    [[nodiscard]] int size() const { return _m->num_edges(); }
    const Mesh* _m;
  };

  struct OrderedVertices_range {  // Do not inherit from view_interface because not O(1) copyable.
    explicit OrderedVertices_range(const Mesh& mesh);
    [[nodiscard]] auto begin(this auto&& self) { return self._vertices.begin(); }
    [[nodiscard]] auto end(this auto&& self) { return self._vertices.end(); }
    [[nodiscard]] int size() const { return _vertices.num(); }

   private:
    Array<Vertex> _vertices;
  };

  struct OrderedFaces_range {  // Do not inherit from view_interface because not O(1) copyable.
    explicit OrderedFaces_range(const Mesh& mesh);
    [[nodiscard]] auto begin(this auto&& self) { return self._faces.begin(); }
    [[nodiscard]] auto end(this auto&& self) { return self._faces.end(); }
    [[nodiscard]] int size() const { return _faces.num(); }

   private:
    Array<Face> _faces;
  };

  // Vertex Iter

  struct VV_sentinel {
    CArrayView<HEdge>::iterator _end{};
  };

  struct VV_iterator {
    using type = VV_iterator;
    using iterator_concept = std::forward_iterator_tag;
    using value_type = Vertex;
    using difference_type = std::ptrdiff_t;
    VV_iterator(CArrayView<HEdge>::iterator it) : _it(it) {}
    VV_iterator() = default;
    [[nodiscard]] bool operator==(const type& rhs) const { return _it == rhs._it && _extrav == rhs._extrav; }
    [[nodiscard]] bool operator==(VV_sentinel s) const { return !_extrav && _it == s._end; }
    [[nodiscard]] Vertex operator*() const { return _extrav ? _extrav : (*_it)->_vert; }
    type& operator++() {
      if (_extrav) return _extrav = nullptr, *this;
      if (!(*_it)->_prev->_sym) _extrav = (*_it)->_prev->_prev->_vert;
      return ++_it, *this;
    }
    type operator++(int) { return postfix_increment(*this); }
    CArrayView<HEdge>::iterator _it{};
    Vertex _extrav{nullptr};  // Boundary vertex to yield before advancing past *_it.
  };

  struct VV_range : ranges::view_interface<VV_range> {
    VV_range(const Mesh&, Vertex v) : _v(v) {}
    [[nodiscard]] VV_iterator begin() const { return VV_iterator(_v->_arhe.begin()); }
    [[nodiscard]] VV_sentinel end() const { return VV_sentinel{_v->_arhe.end()}; }
    // Note that size() is not trivially computable (due to one extra vertex for each boundary component).
    Vertex _v;
  };

  struct VE_sentinel {
    CArrayView<HEdge>::iterator _end{};
  };

  struct VE_iterator {
    using type = VE_iterator;
    using iterator_concept = std::forward_iterator_tag;
    using value_type = Edge;
    using difference_type = std::ptrdiff_t;
    VE_iterator(CArrayView<HEdge>::iterator it) : _it(it) {}
    VE_iterator() = default;
    [[nodiscard]] bool operator==(const type& rhs) const { return _it == rhs._it && _extrae == rhs._extrae; }
    [[nodiscard]] bool operator==(VE_sentinel s) const { return !_extrae && _it == s._end; }
    [[nodiscard]] Edge operator*() const { return _extrae ? _extrae : (*_it)->_edge; }
    type& operator++() {
      if (_extrae) return _extrae = nullptr, *this;
      if (!(*_it)->_prev->_sym) _extrae = (*_it)->_prev->_edge;
      return ++_it, *this;
    }
    type operator++(int) { return postfix_increment(*this); }
    CArrayView<HEdge>::iterator _it{};
    Edge _extrae{nullptr};  // Boundary edge to yield before advancing past *_it.
  };

  struct VE_range : ranges::view_interface<VE_range> {
    VE_range(const Mesh&, Vertex v) : _v(v) {}
    [[nodiscard]] VE_iterator begin() const { return VE_iterator(_v->_arhe.begin()); }
    [[nodiscard]] VE_sentinel end() const { return VE_sentinel{_v->_arhe.end()}; }
    // Note that size() is not trivially computable (due to one extra edge for each boundary component).
    Vertex _v;
  };

  // Face Iter

  struct FC_sentinel {
    HEdge _herep{};
  };

  struct FC_iterator {
    using type = FC_iterator;
    using iterator_concept = std::forward_iterator_tag;
    using value_type = Corner;
    using difference_type = std::ptrdiff_t;
    explicit FC_iterator(HEdge he) : _it(he), _beg(true) {}
    FC_iterator() = default;
    [[nodiscard]] bool operator==(const type& rhs) const { return _it == rhs._it && _beg == rhs._beg; }
    // The hedges form a cycle, so only the initial position distinguishes begin() from end().
    [[nodiscard]] bool operator==(FC_sentinel s) const { return !_beg && _it == s._herep; }
    [[nodiscard]] Corner operator*() const { return _it; }
    type& operator++() { return _beg = false, _it = _it->_next, *this; }
    type operator++(int) { return postfix_increment(*this); }
    HEdge _it{};
    bool _beg{};  // True only for begin(), to distinguish it from the end of the cycle.
  };

  struct FC_range : ranges::view_interface<FC_range> {
    FC_range(const Mesh& m, Face f) : _herep(m.herep(f)) {}
    [[nodiscard]] FC_iterator begin() const { return FC_iterator(_herep); }
    [[nodiscard]] FC_sentinel end() const { return FC_sentinel{_herep}; }
    // Note that size() is not trivially computable.
    HEdge _herep;
  };

  // Edge Iter

  struct EV_range : Vec2<Vertex> {  // always 2 vertices
    EV_range(const Mesh& m, Edge e) {
      HEdge he = m.herep(e);
      (*this)[0] = he->_vert;
      (*this)[1] = he->_prev->_vert;
    }
  };

  struct EF_range : PArray<Face, 2> {  // 1 or 2 faces
    EF_range(const Mesh& m, Edge e) {
      HEdge he = m.herep(e);
      push(he->_face);
      if (he->_sym) push(he->_sym->_face);
    }
  };

  // Ccw iterators around a vertex, requiring the vertex to be nice.

  struct WV_range : PArray<Vertex, 10> {  // ccw Vertex Iter; one extra Vertex if v is on a boundary.
    WV_range(const Mesh& m, Vertex v) {
      for (HEdge he : m.ccw_corners(v)) {
        push(he->_next->_vert);
        if (!he->_sym) push(he->_prev->_vert);
      }
    }
  };

  struct WE_range : PArray<Edge, 10> {  // ccw Edge Iter; one extra Edge if v is on a boundary.
    WE_range(const Mesh& m, Vertex v) {
      for (HEdge he : m.ccw_corners(v)) {
        push(he->_next->_edge);
        if (!he->_sym) push(he->_edge);
      }
    }
  };

  struct WC_sentinel {
    HEdge _hef{};  // The most-clw hedge, at which a closed ring terminates.
  };

  struct WC_iterator {  // ccw Corner iterator; requires a nice Vertex.
    using type = WC_iterator;
    using iterator_concept = std::forward_iterator_tag;
    using value_type = Corner;
    using difference_type = std::ptrdiff_t;
    WC_iterator(const Mesh& m, HEdge he) : _m(&m), _it(he), _beg(true) {}
    WC_iterator() = default;
    [[nodiscard]] bool operator==(const type& rhs) const { return _it == rhs._it && _beg == rhs._beg; }
    // An open ring ends at a null hedge; a closed ring returns to the initial one.
    [[nodiscard]] bool operator==(WC_sentinel s) const { return !_it || (!_beg && _it == s._hef); }
    [[nodiscard]] Corner operator*() const { return _it; }
    type& operator++() { return _beg = false, _it = _m->ccw_hedge(_it), *this; }
    type operator++(int) { return postfix_increment(*this); }
    const Mesh* _m{nullptr};
    HEdge _it{};
    bool _beg{};  // True only for begin(), to distinguish it from the end of a closed ring.
  };

  struct WC_range : ranges::view_interface<WC_range> {
    WC_range(const Mesh& m, Vertex v) : _m(&m), _hef(m.most_clw_hedge(v)) {}
    [[nodiscard]] WC_iterator begin() const { return WC_iterator(*_m, _hef); }
    [[nodiscard]] WC_sentinel end() const { return WC_sentinel{_hef}; }
    // Note that size() is not trivially computable.
    const Mesh* _m;
    HEdge _hef;  // May be nullptr for an isolated Vertex.
  };

 public:  // should be private but uses Pool
  struct MEdge {
    HEdge _herep;
    Flags _flags;
    unique_ptr<char[]> _string;
    MEdge(HEdge herep) : _herep(herep) {}
    HH_MAKE_POOLED_SAC(Mesh::MEdge);  // must be last entry of struct!
    friend std::ostream& operator<<(std::ostream& os, Edge e);
  };

  struct MVertex {
    PArray<HEdge, 8> _arhe;  // hedges he such that he->_prev->_vert == this
    int _id;
    Flags _flags;
    unique_ptr<char[]> _string;
    Point _point;
    MVertex(int id) : _id(id) {}
    HH_MAKE_POOLED_SAC(Mesh::MVertex);  // must be last entry of struct!
    friend std::ostream& operator<<(std::ostream& os, Vertex v);
  };

  struct MFace {
    HEdge _herep;
    int _id;
    Flags _flags;
    unique_ptr<char[]> _string;
    MFace(int id) : _id(id) {}
    HH_MAKE_POOLED_SAC(MFace);  // must be last entry of struct!
    friend std::ostream& operator<<(std::ostream& os, Face f);
  };

  struct MHEdge {
    HEdge _prev;   // previous HEdge in ring around face
    HEdge _next;   // next HEdge in ring around face
    HEdge _sym;    // pointer to symmetric HEdge (or 0)
    Vertex _vert;  // Vertex to which this HEdge is pointing
    Face _face;    // Face on which this HEdge belongs
    Edge _edge;    // Edge to which this HEdge belongs
    unique_ptr<char[]> _string;
    MHEdge() = default;
    HH_MAKE_POOLED_SAC(MHEdge);  // must be last entry of struct!
    friend std::ostream& operator<<(std::ostream& os, HEdge he);
  };

 public:                                                            // Discouraged:
  virtual Vertex create_vertex_private(int id);                     // die if id is already used
  virtual Face create_face_private(int id, CArrayView<Vertex> va);  // die if id is already used
  void vertex_renumber_id_private(Vertex v, int newid);
  void face_renumber_id_private(Face f, int newid);

 protected:
  [[nodiscard]] static int debug();  // 0=no, 1=min, 2=max
 private:
  Flags _flags;
  Map<int, Vertex> _id2vertex;  // also acts as set of vertices
  Map<int, Face> _id2face;      // also acts as set of faces
  int _vertexnum{1};            // id to assign to next new vertex
  int _facenum{1};              // id to assign to next new face
  int _nedges{0};

  [[nodiscard]] HEdge most_clw_hedge(Vertex v) const;                        // is_nice(v), may return nullptr
  [[nodiscard]] HEdge most_ccw_hedge(Vertex v) const;                        // is_nice(v), may return nullptr
  [[nodiscard]] HEdge clw_hedge(HEdge he) const { return he->_next->_sym; }  // may return nullptr
  [[nodiscard]] HEdge ccw_hedge(HEdge he) const { return he->_sym ? he->_sym->_prev : nullptr; }
  [[nodiscard]] HEdge herep(Vertex v) const { return !v->_arhe.num() ? nullptr : v->_arhe[0]->_prev; }
  [[nodiscard]] HEdge herep(Face f) const { return f->_herep; }
  [[nodiscard]] HEdge herep(Edge e) const { return e->_herep; }
  [[nodiscard]] bool is_boundary(HEdge he) const { return !he->_sym; }
  [[nodiscard]] HEdge hedge_from_ev1(Edge e, Vertex v) const {  // may return nullptr
    if (vertex1(e) == v) return herep(e);
    if (vertex2(e) == v) return herep(e)->_sym;
    assertnever("Vertex not on Edge");
  }
  [[nodiscard]] HEdge hedge_from_ev2(Edge e, Vertex v) const {  // may return nullptr
    if (vertex1(e) == v) return herep(e)->_sym;
    if (vertex2(e) == v) return herep(e);
    assertnever("Vertex not on Edge");
  }
  [[nodiscard]] HEdge hedge_from_ef(Edge e, Face f) const {  // may return nullptr
    if (face1(e) == f) return herep(e);
    if (face2(e) == f) return herep(e)->_sym;
    assertnever("Face not adjacent to edge");
  }
  [[nodiscard]] HEdge get_hedge(Vertex v, Face f) const;  // slow; on f pointing to v
  [[nodiscard]] HEdge query_hedge(Vertex v1, Vertex v2) const;
  void enter_hedge(HEdge he, Vertex v1);
  void remove_hedge(HEdge he, Vertex v1);
  void create_bogus_hedges(ArrayView<HEdge> ar_he);
  void remove_bogus_hedges(CArrayView<HEdge> ar_he);
  [[nodiscard]] Array<Vertex> gather_edge_coalesce_vertices(Edge e) const;
};

using Vertex = Mesh::Vertex;
using Face = Mesh::Face;
using Corner = Mesh::Corner;
using Edge = Mesh::Edge;

inline std::ostream& operator<<(std::ostream& os, Vertex v) {
  if (!v) return os << "Vertex{null}";
  return os << sform("Vertex{%d}", v->_id);
}

inline std::ostream& operator<<(std::ostream& os, Face f) {
  if (!f) return os << "Face{null}";
  os << sform("Face{%d}=[", f->_id);
  Mesh::HEdge herep = f->_herep;
  for (Mesh::HEdge he = herep;;) {
    os << sform("%d", he->_vert->_id);
    he = he->_next;
    if (!he || he == herep) break;
    os << ' ';
  }
  return os << ']';
}

inline std::ostream& operator<<(std::ostream& os, Edge e) {
  if (!e) return os << "Edge{null}";
  return os << sform("Edge{%d, %d}", e->_herep->_prev->_vert->_id, e->_herep->_vert->_id);  // vertex1(e), vertex2(e).
}

inline std::ostream& operator<<(std::ostream& os, Corner he) {
  if (!he) return os << "Corner{null}";
  return os << sform("Corner{v=%d, f=%d}", he->_vert->_id, he->_face->_id);  // corner_vertex(c), corner_face(c).
}

//----------------------------------------------------------------------------

HH_INITIALIZE_POOL_NESTED(Mesh::MVertex, MeshMVertex);
HH_INITIALIZE_POOL_NESTED(Mesh::MFace, MeshFace);
HH_INITIALIZE_POOL_NESTED(Mesh::MEdge, MeshMEdge);
HH_INITIALIZE_POOL_NESTED(Mesh::MHEdge, MeshMHEdge);

inline auto Mesh::faces(Vertex v) const {
  return v->_arhe | views::transform([](HEdge he) { return he->_face; });
}

inline auto Mesh::corners(Vertex v) const {
  return v->_arhe | views::transform([](HEdge he) { return he->_prev; });
}

inline auto Mesh::ccw_faces(Vertex v) const {
  return ccw_corners(v) | views::transform([](Corner c) { return c->_face; });
}

inline auto Mesh::vertices(Face f) const {
  return corners(f) | views::transform([](Corner c) { return c->_vert; });
}

inline auto Mesh::faces(Face f) const {
  return corners(f) | views::filter([](Corner c) { return !!c->_sym; }) |
    views::transform([](Corner c) { return c->_sym->_face; });
}

inline auto Mesh::edges(Face f) const {
  return corners(f) | views::transform([](Corner c) { return c->_edge; });
}

inline Vec2<Vertex> Mesh::vertices(Edge e) const {
  HEdge he = herep(e);
  return V(he->_vert, he->_prev->_vert);
}

inline PArray<Face, 2> Mesh::faces(Edge e) const {
  const HEdge he = herep(e);
  return he->_sym ? PArray<Face, 2>{he->_face, he->_sym->_face} : PArray<Face, 2>{he->_face};
}

inline Vec3<Vertex> Mesh::triangle_vertices(Face f) const {
  Vec3<Vertex> va;
  HEdge he = herep(f), he0 = he;
  va[0] = he->_vert, he = he->_next;
  va[1] = he->_vert, he = he->_next;
  va[2] = he->_vert, he = he->_next;
  assertx(he == he0);  // is_triangle()
  return va;
}

inline Vec3<Corner> Mesh::triangle_corners(Face f) const {
  Vec3<Corner> ca;
  HEdge he = herep(f);
  ca[0] = he, he = he->_next;
  ca[1] = he, he = he->_next;
  ca[2] = he, he = he->_next;
  assertx(he == ca[0]);  // is_triangle()
  return ca;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MESH_H_
