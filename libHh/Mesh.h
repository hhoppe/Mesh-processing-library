// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MESH_H_
#define MESH_PROCESSING_LIBHH_MESH_H_

#include "Map.h"
#include "Geometry.h"           // because of Point, too bad.
#include "Array.h"
#include "PArray.h"
#include "Pool.h"
#include "Sac.h"
#include "Flags.h"

#if 0
{
    Mesh mesh;
    Vertex v1 = mesh.create_vertex(), v2 = mesh.create_vertex(), v3 = mesh.create_vertex();
    Face f1 = mesh.create_face(v1, v2, v3);
    for (Face f : mesh.ordered_faces()) { for (Vertex v : mesh.vertices(f)) { process(f, v); } }
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
 public:
    struct MHEdge; struct MVertex; struct MFace; using MCorner = MHEdge; struct MEdge; // private
    using HEdge = MHEdge*; using Vertex = MVertex*; using Face = MFace*; using Corner = HEdge; using Edge = MEdge*;
    friend void swap(Mesh& l, Mesh& r) noexcept;
 private:
    using Vertices_range = Map<int,Vertex>::cvalues_range;
    using Faces_range = Map<int,Face>::cvalues_range;
    struct Edges_range; struct OrderedVertices_range; struct OrderedFaces_range;
    struct VV_range; struct VF_range; struct VE_range; struct VC_range;
    struct FV_range; struct FF_range; struct FE_range; struct FC_range;
    struct EV_range; struct EF_range;
    struct WV_range; struct WF_range; struct WE_range; struct WC_range;
 public:
    Mesh();
    Mesh(Mesh&& m) noexcept                     { swap(*this, m); } // =default?
    virtual ~Mesh()                             { clear(); }
    Mesh& operator=(Mesh&& m) noexcept          { clear(); swap(*this, m); return *this; } // =default?
    void clear();
    void copy(const Mesh& m); // not a GMesh!  carries flags (but not sac fields), hence not named operator=().
// Raw manipulation functions, may lead to non-nice Meshes.
    // always legal
    Vertex create_vertex()                      { return create_vertex_private(_vertexnum); }
    // die if degree(v)>0
    virtual void destroy_vertex(Vertex v);
    // ret false if duplicate vertices or if existing edge
    bool legal_create_face(CArrayView<Vertex> va) const;
    // die if !legal_create_face()
    Face create_face(CArrayView<Vertex> va)     { return create_face_private(_facenum, va); }
    Face create_face(Vertex v1, Vertex v2, Vertex v3) { return create_face(V(v1, v2, v3)); }
    // always legal
    virtual void destroy_face(Face f);
// Vertex
    bool is_nice(Vertex v) const;
    int degree(Vertex v) const;            // == number of adjacent vertices/edges
    int num_boundaries(Vertex v) const;    // 0/1 for a nice vertex
    bool is_boundary(Vertex v) const;      // is_nice(v), degree(v)>0
    Edge opp_edge(Vertex v, Face f) const; // is_triangle(f); slow
    Vertex opp_vertex(Vertex v, Edge e) const;
    // all most_clw and most_ccw assert is_nice(v)
    // move about vertices adjacent to a vertex
    Vertex most_clw_vertex(Vertex v) const;         // if !bnd, ret any; may return nullptr
    Vertex most_ccw_vertex(Vertex v) const;         // if !bnd, ret any; may return nullptr
    Vertex clw_vertex(Vertex v, Vertex vext) const; // slow; may return nullptr
    Vertex ccw_vertex(Vertex v, Vertex vext) const; // slow; may return nullptr
    // move about faces adjacent to a vertex
    Face most_clw_face(Vertex v) const;    // if !bnd, ret any; may return nullptr
    Face most_ccw_face(Vertex v) const;    // if !bnd, ret any; may return nullptr
    Face clw_face(Vertex v, Face f) const; // slow; may return nullptr
    Face ccw_face(Vertex v, Face f) const; // slow; may return nullptr
    // move about edges adjacent to a vertex
    Edge most_clw_edge(Vertex v) const;    // if !bnd, ret any; may return nullptr
    Edge most_ccw_edge(Vertex v) const;    // if !bnd, ret any; may return nullptr
    Edge clw_edge(Vertex v, Edge e) const; // may return nullptr
    Edge ccw_edge(Vertex v, Edge e) const; // may return nullptr
    // get face relative to vertex
    Face ccw_face(Vertex v, Edge e) const; // may return nullptr
    Face clw_face(Vertex v, Edge e) const; // may return nullptr
// Face
    bool is_nice(Face f) const;
    int num_vertices(Face f) const;
    bool is_triangle(Face f) const              { HEdge he = herep(f); return he->next->next->next==he; }
    bool is_boundary(Face f) const;        // == has a boundary vertex
    Face opp_face(Face f, Edge e) const;   // ret nullptr if is_boundary(e)
    Face opp_face(Vertex v, Face f) const; // is_triangle(f); ret nullptr if none
    // ccw order
    void get_vertices(Face f, Array<Vertex>& va) const;
    void triangle_vertices(Face f, Vec3<Vertex>& va) const;
    Vertex vertex(Face f, int i) const; // die if i>=num_vertices(f)
    Array<Corner> get_corners(Face f, Array<Corner>&& ca = Array<Corner>()) const;
    // move about a face
    Edge clw_edge(Face f, Edge e) const         { return hedge_from_ef(e, f)->prev->edge; }
    Edge ccw_edge(Face f, Edge e) const         { return hedge_from_ef(e, f)->next->edge; }
    Vertex clw_vertex(Face f, Vertex v) const   { return get_hedge(v, f)->prev->vert; } // slow
    Vertex ccw_vertex(Face f, Vertex v) const   { return get_hedge(v, f)->next->vert; } // slow
    Edge clw_edge(Face f, Vertex v) const       { return get_hedge(v, f)->edge; }       // slow
    Edge ccw_edge(Face f, Vertex v) const       { return get_hedge(v, f)->next->edge; } // slow
// Edge
    bool is_boundary(Edge e) const              { return !herep(e)->sym; }
    Vertex vertex1(Edge e) const                { return herep(e)->prev->vert; }
    Vertex vertex2(Edge e) const                { return herep(e)->vert; }
    Vertex vertex(Edge e, int i) const          { ASSERTX(i==0 || i==1); return i==0 ? vertex1(e) : vertex2(e); }
    Face face1(Edge e) const                    { return herep(e)->face; }
    Face face2(Edge e) const                    { HEdge he = herep(e); return he->sym ? he->sym->face : nullptr; }
    Face face(Edge e, int i) const              { ASSERTX(i==0 || i==1); return i==0 ? face1(e) : face2(e); }
// i==0 or i==1; ret nullptr if i==1 && is_boundary(e)
    Vertex side_vertex1(Edge e) const           { return opp_vertex(e, face1(e)); } // is_triangle(face1())
    Vertex side_vertex2(Edge e) const           { return face2(e) ? opp_vertex(e, face2(e)) : nullptr; }
    Vertex side_vertex(Edge e, int i) const { ASSERTX(i==0 || i==1); return !i ? side_vertex1(e) : side_vertex2(e); }
    Vertex opp_vertex(Edge e, Face f) const;   // is_triangle(f)
    Edge opp_boundary(Edge e, Vertex v) const; // is_boundary(e)
    Edge clw_boundary(Edge e) const             { return opp_boundary(e, vertex2(e)); } // is_boundary(e)
    Edge ccw_boundary(Edge e) const             { return opp_boundary(e, vertex1(e)); } // is_boundary(e)
    Vertex vertex_between_edges(Edge e1, Edge e2);
// Corner
    Corner corner(Vertex v, Face f) const       { return get_hedge(v, f); }
    Vertex corner_vertex(Corner c) const        { return c->vert; }
    Face corner_face(Corner c) const            { return c->face; }
    Corner ccw_corner(Corner c) const           { return c->sym ? c->sym->prev : nullptr; } // around vertex
    Corner clw_corner(Corner c) const           { return c->next->sym; }                    // around vertex
    Corner ccw_face_corner(Corner c) const      { return c->next; }                         // around face
    Corner clw_face_corner(Corner c) const      { return c->prev; }                         // around face
    Corner ccw_corner(Vertex v, Edge e) const   { HEdge he = hedge_from_ev1(e, v); return he ? he->prev : nullptr; }
    Corner clw_corner(Vertex v, Edge e) const   { return hedge_from_ev2(e, v); }
    Edge ccw_face_edge(Corner c) const          { return c->next->edge; }
    Edge clw_face_edge(Corner c) const          { return c->edge; } // (fastest)
// Other associations
    // obtain edge from vertices
    Edge query_edge(Vertex v, Vertex w) const;
    Edge edge(Vertex v, Vertex w) const         { return assertx(query_edge(v, w)); }
    Edge ordered_edge(Vertex v1, Vertex v2) const; // asserts it exists, oriented
    // get face from 2 consecutive vertices (ccw order)
    Face face(Vertex v, Vertex vccw) const      { return clw_face(vccw, edge(v, vccw)); } // may return nullptr
// Counting routines (fast)
    int num_vertices() const                    { return _id2vertex.num(); }
    int num_faces() const                       { return _id2face.num(); }
    int num_edges() const                       { return _nedges; }
// Random access (fast), assert there exist at least one
    Vertex random_vertex(Random& r) const;
    Face random_face(Random& r) const;
    Edge random_edge(Random& r) const; // unbiased for a closed triangle mesh
// Flags
    static FlagMask allocate_flag()             { static int s_counter; return Flags::allocate(s_counter); }
    static FlagMask allocate_Vertex_flag()      { static int s_counter; return Flags::allocate(s_counter); }
    static FlagMask allocate_Face_flag()        { static int s_counter; return Flags::allocate(s_counter); }
    static FlagMask allocate_Edge_flag()        { static int s_counter; return Flags::allocate(s_counter); }
    Flags& gflags()                             { return _flags; }
    const Flags& gflags() const                 { return _flags; }
    Flags& flags(Vertex v)                      { return v->_flags; }
    const Flags& flags(Vertex v) const          { return v->_flags; }
    Flags& flags(Face f)                        { return f->_flags; }
    const Flags& flags(Face f) const            { return f->_flags; }
    Flags& flags(Edge e)                        { return e->_flags; }
    const Flags& flags(Edge e) const            { return e->_flags; }
// Triangular mesh operations (die if not triangular!)
    // would collapse be legal?
    bool legal_edge_collapse(Edge e) const;
    // would collapse preserve a nice mesh?
    bool nice_edge_collapse(Edge e) const;
    // would edge swap be legal?  (legal implies nice here)
    bool legal_edge_swap(Edge e) const;
    //
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
// More mesh operations
    // vs2 can be nullptr, returns v2, leaves hole
    virtual Vertex split_vertex(Vertex v1, Vertex vs1, Vertex vs2, int v2i);
    // replace (vt, {*}) by (vs, {*})
    bool legal_vertex_merge(Vertex vs, Vertex vt);
    virtual void merge_vertices(Vertex vs, Vertex vt); // die if !legal
    // introduce one center vertex and triangulate face
    virtual Vertex center_split_face(Face f); // always legal
    // introduce an edge to split face on (v1, v2)
    virtual Edge split_face(Face f, Vertex v1, Vertex v2);
    // remove the consecutive set of edges separating two faces
    //  (may destroy some vertices if >1 edge shared by two faces)
    bool legal_coalesce_faces(Edge e);
    virtual Face coalesce_faces(Edge e); // die if !legal
    virtual Vertex insert_vertex_on_edge(Edge e);
    virtual Edge remove_vertex_between_edges(Vertex vr);
// Mesh
    Vertex id_vertex(int i) const               { return _id2vertex.get(i); }
    int vertex_id(Vertex v) const               { return v->id; }
    Face id_face(int i) const                   { return _id2face.get(i); }
    int face_id(Face f) const                   { return f->id; }
    Vertex id_retrieve_vertex(int i) const      { return _id2vertex.retrieve(i); }
    Face id_retrieve_face(int i) const          { return _id2face.retrieve(i); }
    bool is_nice() const;
    void renumber();            // renumber vertices and faces
// Misc
    void ok() const;            // die if problem
    bool valid(Vertex v) const; // die if invalid
    bool valid(Face f) const;   // die if invalid
    bool valid(Edge e) const;   // die if invalid
    bool valid(Corner c) const; // die if invalid
// Iterators; can crash if continued after any change in the Mesh.
    // These mesh iterators do not define an order.
    Vertices_range vertices() const             { return _id2vertex.values(); }
    Faces_range faces() const                   { return _id2face.values(); }
    Edges_range edges() const                   { return Edges_range(*this); }
    // These mesh iterators sort by id numbers.
    OrderedVertices_range ordered_vertices() const { return OrderedVertices_range(*this); }
    OrderedFaces_range ordered_faces() const { return OrderedFaces_range(*this); }
    // These vertex iterators do not specify order, and work correctly even on non-nice vertices.
    VV_range vertices(Vertex v) const           { return VV_range(*this, v); }
    VF_range faces(Vertex v) const              { return VF_range(*this, v); }
    VE_range edges(Vertex v) const              { return VE_range(*this, v); }
    VC_range corners(Vertex v) const            { return VC_range(*this, v); }
    // These vertex iterators go CCW, but require nice vertices.
    WV_range ccw_vertices(Vertex v) const       { return WV_range(*this, v); }
    WF_range ccw_faces(Vertex v) const          { return WF_range(*this, v); }
    WE_range ccw_edges(Vertex v) const          { return WE_range(*this, v); }
    WC_range ccw_corners(Vertex v) const        { return WC_range(*this, v); }
    // Face iterators all go CCW
    FV_range vertices(Face f) const             { return FV_range(*this, f); }
    FF_range faces(Face f) const                { return FF_range(*this, f); }
    FE_range edges(Face f) const                { return FE_range(*this, f); }
    FC_range corners(Face f) const              { return FC_range(*this, f); }
    // Edge iterators do not define an order.
    EV_range vertices(Edge e) const             { return EV_range(*this, e); }
    EF_range faces(Edge e) const                { return EF_range(*this, e); }
 private:
    friend class GMesh;
    // Mesh Iter
    class Edges_iterator : public std::iterator<std::forward_iterator_tag,Edge> {
     public:
        Edges_iterator(const Mesh& m, bool beg) {
            _vend = m._id2vertex.values().end();
            _vcur = beg ? m._id2vertex.values().begin() : _vend;
            next();
        }
        bool operator!=(const Edges_iterator& rhs) const { return _hcur!=rhs._hcur || _vcur!=rhs._vcur; }
        Edge operator*() const                  { ASSERTX(_hcur!=_hend); return (*_hcur)->edge; }
        Edges_iterator& operator++()            { ASSERTX(_hcur!=_hend); ++_hcur; next(); return *this; }
     private:
        CArrayView<HEdge>::iterator _hcur {nullptr} , _hend {nullptr}; // _hcur points at current element
        Map<int,Vertex>::cvalues_iterator _vcur, _vend;                // _vcur points one vertex ahead
        void next() {
            for (;;) {
                if (_hcur!=_hend) {
                    if ((*_hcur)->edge->herep!=*_hcur) { ++_hcur; continue; }
                    return;
                }
                if (_vcur==_vend) break;
                auto& range = (*_vcur)->_arhe; ++_vcur; _hcur = range.begin(); _hend = range.end();
            }
            _hcur = _hend = nullptr;    // no element found
        }
    };
    struct Edges_range {
        Edges_range(const Mesh& m)              : _m(m) { }
        Edges_iterator begin() const            { return Edges_iterator(_m, true); }
        Edges_iterator end() const              { return Edges_iterator(_m, false); }
        const Mesh& _m;
    };
    struct OrderedVertices_range {
        using Container = Array<Vertex>;
        OrderedVertices_range(const Mesh& mesh);
        Container::iterator begin()             { return _vertices.begin(); }
        Container::iterator end()               { return _vertices.end(); }
        Container _vertices;
    };
    struct OrderedFaces_range {
        using Container = Array<Face>;
        OrderedFaces_range(const Mesh& mesh);
        Container::iterator begin()             { return _faces.begin(); }
        Container::iterator end()               { return _faces.end(); }
        Container _faces;
    };
    // Vertex Iter
    class VV_iterator : public std::iterator<std::forward_iterator_tag,Vertex> {
     public:
        VV_iterator(CArrayView<HEdge>::iterator it) : _it(it) { }
        bool operator!=(const VV_iterator& rhs) const { ASSERTX(!rhs._extrav); return _extrav || _it!=rhs._it; }
        Vertex operator*() const                { return _extrav ? _extrav : (*_it)->vert; }
        VV_iterator& operator++() {
            if (_extrav) { _extrav = nullptr; return *this; }
            if (!(*_it)->prev->sym) _extrav = (*_it)->prev->prev->vert;
            ++_it;
            return *this;
        }
     private:
        CArrayView<HEdge>::iterator _it;
        Vertex _extrav {nullptr};
    };
    struct VV_range {
        VV_range(const Mesh&, Vertex v)         : _ar(v->_arhe) { }
        VV_iterator begin() const               { return VV_iterator(_ar.begin()); }
        VV_iterator end() const                 { return VV_iterator(_ar.end()); }
        CArrayView<HEdge> _ar;
    };
    class VF_iterator : public std::iterator<std::forward_iterator_tag,Face> {
     public:
        VF_iterator(CArrayView<HEdge>::iterator it) : _it(it) { }
        bool operator!=(const VF_iterator& rhs) const { return _it!=rhs._it; }
        Face operator*() const                  { return (*_it)->face; }
        VF_iterator& operator++()               { ++_it; return *this; }
     private:
        CArrayView<HEdge>::iterator _it;
    };
    struct VF_range {
        VF_range(const Mesh&, Vertex v)         : _ar(v->_arhe) { }
        VF_iterator begin() const               { return VF_iterator(_ar.begin()); }
        VF_iterator end() const                 { return VF_iterator(_ar.end()); }
        CArrayView<HEdge> _ar;
    };
    class VE_iterator : public std::iterator<std::forward_iterator_tag,Edge> {
     public:
        VE_iterator(CArrayView<HEdge>::iterator it) : _it(it) { }
        bool operator!=(const VE_iterator& rhs) const { ASSERTX(!rhs._extrae); return _extrae || _it!=rhs._it; }
        Edge operator*() const                  { return _extrae ? _extrae : (*_it)->edge; }
        VE_iterator& operator++() {
            if (_extrae) { _extrae = nullptr; return *this; }
            if (!(*_it)->prev->sym) _extrae = (*_it)->prev->edge;
            ++_it;
            return *this;
        }
     private:
        CArrayView<HEdge>::iterator _it;
        Edge _extrae {nullptr};
    };
    struct VE_range {
        VE_range(const Mesh&, Vertex v)         : _ar(v->_arhe) { }
        VE_iterator begin() const               { return VE_iterator(_ar.begin()); }
        VE_iterator end() const                 { return VE_iterator(_ar.end()); }
        CArrayView<HEdge> _ar;
    };
    class VC_iterator : public std::iterator<std::forward_iterator_tag,Corner> {
     public:
        VC_iterator(CArrayView<HEdge>::iterator it) : _it(it) { }
        bool operator!=(const VC_iterator& rhs) const { return _it!=rhs._it; }
        Corner operator*() const                { return (*_it)->prev; }
        VC_iterator& operator++()               { ++_it; return *this; }
     private:
        CArrayView<HEdge>::iterator _it;
    };
    struct VC_range {
        VC_range(const Mesh&, Vertex v)         : _ar(v->_arhe) { }
        VC_iterator begin() const               { return VC_iterator(_ar.begin()); }
        VC_iterator end() const                 { return VC_iterator(_ar.end()); }
        CArrayView<HEdge> _ar;
    };
    // Face Iter
    class FV_iterator : public std::iterator<std::forward_iterator_tag,Vertex> {
     public:
        FV_iterator(HEdge he, bool beg)         : _it(he), _beg(beg) { }
        bool operator!=(const FV_iterator& rhs) const { ASSERTX(!rhs._beg); return _beg || _it!=rhs._it; }
        Vertex operator*() const                { return _it->vert; }
        FV_iterator& operator++()               { _beg = false; _it = _it->next; return *this; }
     private:
        HEdge _it;
        bool _beg;
    };
    struct FV_range {
        FV_range(const Mesh& m, Face f)         : _herep(m.herep(f)) { }
        FV_iterator begin() const               { return FV_iterator(_herep, true); }
        FV_iterator end() const                 { return FV_iterator(_herep, false); }
        HEdge _herep;
    };
    class FF_iterator : public std::iterator<std::forward_iterator_tag,Face> {
     public:
        FF_iterator(HEdge he, bool beg) : _it(he), _beg(beg) {
            for (;;) {
                if (_it->sym) break;
                _it = _it->next;
                if (_it==he) { _beg = false; break; }
            }
        }
        bool operator!=(const FF_iterator& rhs) const { ASSERTX(!rhs._beg); return _beg || _it!=rhs._it; }
        Face operator*() const                  { return _it->sym->face; }
        FF_iterator& operator++() {
            HEdge bu = _beg ? nullptr : _it;
            _beg = false;
            for (;;) {
                _it = _it->next;
                if (_it->sym) break;
                ASSERTX(_it!=bu);
            }
            return *this;
        }
     private:
        HEdge _it;
        bool _beg;
    };
    struct FF_range {
        FF_range(const Mesh& m, Face f)         : _herep(m.herep(f)) { }
        FF_iterator begin() const               { return FF_iterator(_herep, true); }
        FF_iterator end() const                 { return FF_iterator(_herep, false); }
        HEdge _herep;
    };
    class FE_iterator : public std::iterator<std::forward_iterator_tag,Edge> {
     public:
        FE_iterator(HEdge he, bool beg)         : _it(he), _beg(beg) { }
        bool operator!=(const FE_iterator& rhs) const { ASSERTX(!rhs._beg); return _beg || _it!=rhs._it; }
        Edge operator*() const                  { return _it->edge; }
        FE_iterator& operator++()               { _beg = false; _it = _it->next; return *this; }
     private:
        HEdge _it;
        bool _beg;
    };
    struct FE_range {
        FE_range(const Mesh& m, Face f)         : _herep(m.herep(f)) { }
        FE_iterator begin() const               { return FE_iterator(_herep, true); }
        FE_iterator end() const                 { return FE_iterator(_herep, false); }
        HEdge _herep;
    };
    class FC_iterator : public std::iterator<std::forward_iterator_tag,Corner> {
     public:
        FC_iterator(HEdge he, bool beg)         : _it(he), _beg(beg) { }
        bool operator!=(const FC_iterator& rhs) const { ASSERTX(!rhs._beg); return _beg || _it!=rhs._it; }
        Corner operator*() const                { return _it; }
        FC_iterator& operator++()               { _beg = false; _it = _it->next; return *this; }
     private:
        HEdge _it;
        bool _beg;
    };
    struct FC_range {
        FC_range(const Mesh& m, Face f)         : _herep(m.herep(f)) { }
        FC_iterator begin() const               { return FC_iterator(_herep, true); }
        FC_iterator end() const                 { return FC_iterator(_herep, false); }
        HEdge _herep;
    };
    // Edge Iter
    struct EV_range : Vec2<Vertex> { // always 2 vertices
        EV_range(const Mesh& m, Edge e) {
            HEdge he = m.herep(e);
            (*this)[0] = he->vert;
            (*this)[1] = he->prev->vert;
        }
    };
    struct EF_range : PArray<Face,2> { // 0 or 1 face
        EF_range(const Mesh& m, Edge e) {
            HEdge he = m.herep(e);
            push(he->face);
            if (he->sym) push(he->sym->face);
        }
    };
    // ccw Vertex Iter
    struct WV_range : PArray<Vertex,10> {
        WV_range(const Mesh& m, Vertex v) {
            HEdge he = m.most_clw_hedge(v), hef = he; // return HEdges pointing to v
            while (he) {
                push(he->next->vert);
                if (!he->sym) push(he->prev->vert);
                he = m.ccw_hedge(he);
                if (he==hef) break;
            }
        }
    };
    struct WF_range : PArray<Face,10> {
        WF_range(const Mesh& m, Vertex v) {
            HEdge he = m.most_clw_hedge(v), hef = he;
            while (he) {
                push(he->face);
                he = m.ccw_hedge(he);
                if (he==hef) break;
            }
        }
    };
    struct WE_range : PArray<Edge,10> {
        WE_range(const Mesh& m, Vertex v) {
            HEdge he = m.most_clw_hedge(v), hef = he;
            while (he) {
                push(he->next->edge);
                if (!he->sym) push(he->edge);
                he = m.ccw_hedge(he);
                if (he==hef) break;
            }
        }
    };
    struct WC_range : PArray<Corner,10> {
        WC_range(const Mesh& m, Vertex v) {
            HEdge he = m.most_clw_hedge(v), hef = he;
            while (he) {
                push(he);
                he = m.ccw_hedge(he);
                if (he==hef) break;
            }
        }
    };
 public:                        // should be private but uses Pool
    struct MEdge {
        HEdge herep;
        Flags _flags;
        unique_ptr<char[]> _string;
        MEdge(HEdge pherep)                     : herep(pherep) { }
        HH_MAKE_POOLED_SAC(Mesh::MEdge); // must be last entry of class!
    };
    struct MVertex {
        PArray<HEdge,8> _arhe;  // hedges he such that he->prev->vert==this
        int id;
        Flags _flags;
        unique_ptr<char[]> _string;
        Point point;
        MVertex(int pid)                        : id(pid) { }
        HH_MAKE_POOLED_SAC(Mesh::MVertex); // must be last entry of class!
    };
    struct MFace {
        HEdge herep;
        int id;
        Flags _flags;
        unique_ptr<char[]> _string;
        MFace(int pid)                          : id(pid) { }
        HH_MAKE_POOLED_SAC(MFace);  // must be last entry of class!
    };
    struct MHEdge {
        HEdge prev;             // previous HEdge in ring around face
        HEdge next;             // next HEdge in ring around face
        HEdge sym;              // pointer to symmetric HEdge (or 0)
        Vertex vert;            // Vertex to which this HEdge is pointing
        Face face;              // Face on which this HEdge belongs
        Edge edge;              // Edge to which this HEdge belongs
        unique_ptr<char[]> _string;
        MHEdge()                                = default;
        HH_MAKE_POOLED_SAC(MHEdge); // must be last entry of class!
    };
 public:                                                       // Discouraged:
    virtual Vertex create_vertex_private(int id);              // die if id is already used
    virtual Face create_face_private(int id, CArrayView<Vertex> va); // die if id is already used
    void vertex_renumber_id_private(Vertex v, int newid);
    void face_renumber_id_private(Face f, int newid);
 protected:
    static const int sdebug;    // 0=no, 1=min, 2=max
 private:
    Flags _flags;
    Map<int,Vertex> _id2vertex; // also acts as set of vertices
    Map<int,Face> _id2face;     // also acts as set of faces
    int _vertexnum {1};         // id to assign to next new vertex
    int _facenum {1};           // id to assign to next new face
    int _nedges {0};
    //
    HEdge most_clw_hedge(Vertex v) const; // is_nice(v), may return nullptr
    HEdge most_ccw_hedge(Vertex v) const; // is_nice(v), may return nullptr
    HEdge clw_hedge(HEdge he) const             { return he->next->sym; } // may return nullptr
    HEdge ccw_hedge(HEdge he) const             { return he->sym ? he->sym->prev : nullptr; }
    HEdge herep(Vertex v) const                 { return !v->_arhe.num() ? nullptr : v->_arhe[0]->prev; }
    HEdge herep(Face f) const                   { return f->herep; }
    HEdge herep(Edge e) const                   { return e->herep; }
    bool is_boundary(HEdge he) const            { return !he->sym; }
    HEdge hedge_from_ev1(Edge e, Vertex v) const { // may return nullptr
        if (vertex1(e)==v) return herep(e);
        if (vertex2(e)==v) return herep(e)->sym;
        assertnever("Vertex not on Edge");
    }
    HEdge hedge_from_ev2(Edge e, Vertex v) const { // may return nullptr
        if (vertex1(e)==v) return herep(e)->sym;
        if (vertex2(e)==v) return herep(e);
        assertnever("Vertex not on Edge");
    }
    HEdge hedge_from_ef(Edge e, Face f) const { // may return nullptr
        if (face1(e)==f) return herep(e);
        if (face2(e)==f) return herep(e)->sym;
        assertnever("Face not adjacent to edge");
    }
    HEdge get_hedge(Vertex v, Face f) const;      // slow; on f pointing to v
    HEdge query_hedge(Vertex v1, Vertex v2) const;
    void enter_hedge(HEdge he, Vertex v1);
    void remove_hedge(HEdge he, Vertex v1);
    void create_bogus_hedges(ArrayView<HEdge> ar_he);
    void remove_bogus_hedges(CArrayView<HEdge> ar_he);
    Array<Vertex> gather_edge_coalesce_vertices(Edge e) const;
};

using Vertex = Mesh::Vertex;
using Face = Mesh::Face;
using Corner = Mesh::Corner;
using Edge = Mesh::Edge;


//----------------------------------------------------------------------------

HH_INITIALIZE_POOL_NESTED(Mesh::MVertex, MeshMVertex);
HH_INITIALIZE_POOL_NESTED(Mesh::MFace, MeshFace);
HH_INITIALIZE_POOL_NESTED(Mesh::MEdge, MeshMEdge);
HH_INITIALIZE_POOL_NESTED(Mesh::MHEdge, MeshMHEdge);

inline void swap(Mesh& l, Mesh& r) noexcept {
    using std::swap; swap(l._flags, r._flags); swap(l._id2vertex, r._id2vertex); swap(l._id2face, r._id2face);
    swap(l._vertexnum, r._vertexnum); swap(l._facenum, r._facenum); swap(l._nedges, r._nedges);
}

inline void Mesh::triangle_vertices(Face f, Vec3<Vertex>& va) const {
    HEdge he = herep(f), he0 = he;
    va[0] = he->vert; he = he->next;
    va[1] = he->vert; he = he->next;
    va[2] = he->vert; he = he->next;
    assertx(he==he0);           // is_triangle()
}

HH_SAC_DECLARATION(Mesh::MVertex);
HH_SAC_DECLARATION(Mesh::MFace);
HH_SAC_DECLARATION(Mesh::MEdge);
HH_SAC_DECLARATION(Mesh::MHEdge);

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_MESH_H_
