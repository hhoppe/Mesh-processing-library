// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Mesh.h"

#include "libHh/Array.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // sort()
#include "libHh/Set.h"
#include "libHh/Stack.h"

namespace hh {

HH_ALLOCATE_POOL(Mesh::MVertex);
HH_ALLOCATE_POOL(Mesh::MFace);
HH_ALLOCATE_POOL(Mesh::MEdge);
HH_ALLOCATE_POOL(Mesh::MHEdge);

// *** MISC

// Analysis of memory requirement:
//
//  MVertex: (7+6*2)*4  *1v/v   == 76 bytes/vertex
//  MFace:   4*4        *2f/v   == 32 bytes/vertex
//  MEdge:   3*4        *3e/v   == 36 bytes/vertex
//  MHEdge:  7*4        *6h/v   == 168 bytes/vertex
//  id2vertex: 16       *1v/v   == 16 bytes/vertex
//  id2face:   16       *2f/v   == 32 bytes/vertex
//
//  Total:                      == 360 bytes/vertex (+ string_info{v,f,e,w})
//
//  example: mojave_sq600 (360k vertices): 130 MBytes
//
// (Compare with WMesh: 68 bytes/vertex;  AWMesh:  92 bytes/vertex
//  no attribs:  WMesh: 40 bytes/vertex;  AWMesh:  64 bytes/vertex)

// *** DRAWING

// Note: changes to mesh go through enter_hedge() and remove_hedge() !

/*
//            v2
//       /    ^    \
//   vo1 f1 he|  f2  vo2
//       \    |    /
//            v1            if (is_boundary(he)), f2 == 0, vo2 == 0
*/

//  Vertex v1 -> stack of HEdges (v1, ?)

// *** Mesh

int Mesh::debug() {
  static const int value = getenv_int("MESH_DEBUG");  // 0, 1, or 3
  return value;
}

Mesh::Mesh() { assertw(debug() == 0); }

Mesh& Mesh::operator=(Mesh&& m) noexcept {
  clear();
  swap(*this, m);
  return *this;
}

void Mesh::clear() {
  if (debug() >= 1) ok();
  while (num_faces()) {
    destroy_face(_id2face.get_one_value());
  }
  while (num_vertices()) {
    destroy_vertex(_id2vertex.get_one_value());
  }
  _vertexnum = 1;
  _facenum = 1;
  _nedges = 0;
}

void Mesh::copy(const Mesh& m) {
  clear();
  _flags = m._flags;
  for (Vertex v : m.vertices()) {
    Vertex vn = create_vertex_private(m.vertex_id(v));
    vn->_flags = v->_flags;
  }
  Array<Vertex> va;
  for (Face f : m.faces()) {
    m.get_vertices(f, va);
    for_int(i, va.num()) va[i] = id_vertex(m.vertex_id(va[i]));
    Face fn = create_face_private(m.face_id(f), va);
    fn->_flags = f->_flags;
  }
  for (Edge e : m.edges()) {
    if (!e->_flags) continue;
    Edge en = ordered_edge(id_vertex(m.vertex_id(m.vertex1(e))), id_vertex(m.vertex_id(m.vertex2(e))));
    en->_flags = e->_flags;
  }
}

// *** Raw manipulation

Vertex Mesh::create_vertex_private(int id) {
  assertx(id >= 1);
  Vertex v = new MVertex(id);
  // v->point undefined
  _id2vertex.enter(id, v);
  _vertexnum = max(_vertexnum, id + 1);
  return v;
}

void Mesh::destroy_vertex(Vertex v) {
  assertx(!herep(v));
  assertx(_id2vertex.remove(v->_id));
  if (0 && _vertexnum - 1 == v->_id) --_vertexnum;  // intermittent reuse of vertex id might be unsafe
  delete v;
}

bool Mesh::legal_create_face(CArrayView<Vertex> va) const {
  assertx(va.num() >= 3);
  if (debug() >= 1) {
    for (Vertex v : va) valid(v);
  }
  if (va.num() == 3) {  // cheap check
    if (va[0] == va[1] || va[1] == va[2] || va[0] == va[2]) return false;
  } else {
    Set<Vertex> setv;
    for (Vertex v : va) {
      if (!setv.add(v)) return false;
    }
  }
  Vertex vo = va[va.num() - 1];
  for_int(i, va.num()) {
    if (query_hedge(vo, va[i])) return false;
    vo = va[i];
  }
  return true;
}

Face Mesh::create_face_private(int id, CArrayView<Vertex> va) {
  assertx(id >= 1);
  assertx(va.num() >= 3);
  if (debug() >= 1) assertx(legal_create_face(va));
  Face f = new MFace(id);
  // f->herep defined below
  _id2face.enter(id, f);
  HEdge hep = nullptr;
  int nv = va.num();
  for_int(i, nv) {
    Vertex v2 = va[i + 1 == nv ? 0 : i + 1];
    HEdge he = new MHEdge;
    he->_prev = hep;
    // he->_next is set below
    he->_vert = v2;
    enter_hedge(he, va[i]);
    // he->_sym and he->_edge were set in enter_hedge
    he->_face = f;
    hep = he;
  }
  assertx(hep);  // HH_ASSUME(hep);
  HEdge helast = hep;
  for (;;) {
    HEdge hepp = hep->_prev;
    if (!hepp) break;
    hepp->_next = hep;
    hep = hepp;
  }
  hep->_prev = helast;
  helast->_next = hep;
  f->_herep = helast;  // such that f->herep->_vert == va[0]
  _facenum = max(_facenum, id + 1);
  if (debug() >= 3) ok();
  return f;
}

void Mesh::destroy_face(Face f) {
  {
    HEdge he = assertx(herep(f)), hef = he;
    Vertex v1 = he->_prev->_vert;
    for (;;) {
      HEdge hen = he->_next;
      Vertex v1n = he->_vert;
      remove_hedge(he, v1);
      delete he;
      he = hen;
      v1 = v1n;
      if (he == hef) break;
    }
  }
  assertx(_id2face.remove(f->_id));
  if (0 && _facenum - 1 == f->_id) --_facenum;  // intermittent reuse of face id might be unsafe
  delete f;
  if (debug() >= 3) ok();
}

// *** Vertex

bool Mesh::is_nice(Vertex v) const {
  // return num_boundaries(v) < 2;  // ignores 2 interior vertices touching
  HEdge her = herep(v);
  if (!her) return true;
  int nhe = 0;
  HEdge he = her;
  for (;;) {
    nhe++;
    he = clw_hedge(he);
    if (!he || he == her) break;
  }
  if (he != her) {
    for (he = her; (he = ccw_hedge(he)) != nullptr;) nhe++;
  }
  return nhe == v->_arhe.num();
}

int Mesh::degree(Vertex v) const {
  int n = 0;
  for (HEdge he : v->_arhe) n += is_boundary(he) ? 2 : 1;
  return n;
}

int Mesh::num_boundaries(Vertex v) const {
  int n = 0;
  for (HEdge he : v->_arhe) {
    if (is_boundary(he)) n++;
  }
  return n;
}

bool Mesh::is_boundary(Vertex v) const {
  if (!herep(v))
    assertnever("Did not expect isolated vertex -- perhaps clean the input using \"Filtermesh -rmcomp 0\"");
  int nb = num_boundaries(v);
  if (nb >= 2)  // !is_nice(v)
    assertnever("Did not expect non-nice vertex -- perhaps clean the input using \"Filtermesh -fixvertices\"");
  return nb > 0;
}

Edge Mesh::opp_edge(Vertex v, Face f) const {
  assertx(is_triangle(f));
  return get_hedge(v, f)->_prev->_edge;
}

Vertex Mesh::opp_vertex(Vertex v, Edge e) const {
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  if (v == v1) return v2;
  if (v == v2) return v1;
  assertnever("Vertex not on Edge");
}

Vertex Mesh::most_clw_vertex(Vertex v) const {
  HEdge he = most_clw_hedge(v);
  return he ? he->_next->_vert : nullptr;
}

Vertex Mesh::most_ccw_vertex(Vertex v) const {
  HEdge he = most_ccw_hedge(v);
  return he ? he->_prev->_vert : nullptr;
}

Vertex Mesh::clw_vertex(Vertex v, Vertex vext) const {
  HEdge he = query_hedge(vext, v);
  if (he) return he->_next->_vert;
  if (debug() >= 1) assertx(query_hedge(v, vext));
  return nullptr;
}

Vertex Mesh::ccw_vertex(Vertex v, Vertex vext) const {
  HEdge he = query_hedge(v, vext);
  if (he) return he->_prev->_prev->_vert;
  if (debug() >= 1) assertx(query_hedge(vext, v));
  return nullptr;
}

Face Mesh::most_clw_face(Vertex v) const {
  HEdge he = most_clw_hedge(v);
  return he ? he->_face : nullptr;
}

Face Mesh::most_ccw_face(Vertex v) const {
  HEdge he = most_ccw_hedge(v);
  return he ? he->_face : nullptr;
}

Face Mesh::clw_face(Vertex v, Face f) const { return opp_face(f, ccw_edge(f, v)); }

Face Mesh::ccw_face(Vertex v, Face f) const { return opp_face(f, clw_edge(f, v)); }

Edge Mesh::most_clw_edge(Vertex v) const {
  HEdge he = most_clw_hedge(v);
  return he ? he->_next->_edge : nullptr;
}

Edge Mesh::most_ccw_edge(Vertex v) const {
  HEdge he = most_ccw_hedge(v);
  return he ? he->_edge : nullptr;
}

Edge Mesh::clw_edge(Vertex v, Edge e) const {
  HEdge he = hedge_from_ev2(e, v);
  return he ? he->_next->_edge : nullptr;
}

Edge Mesh::ccw_edge(Vertex v, Edge e) const {
  HEdge he = hedge_from_ev1(e, v);
  return he ? he->_prev->_edge : nullptr;
}

Face Mesh::ccw_face(Vertex v, Edge e) const {
  if (vertex1(e) == v) return face1(e);
  if (vertex2(e) == v) return face2(e);
  assertnever("Vertex not on edge");
}

Face Mesh::clw_face(Vertex v, Edge e) const {
  if (vertex1(e) == v) return face2(e);
  if (vertex2(e) == v) return face1(e);
  assertnever("Vertex not on edge");
}

// *** Face

bool Mesh::is_nice(Face f) const {
  // non-nice iff all its neighbors are the same
  Face fp = nullptr;
  int n = 0;
  for (Face ff : faces(f)) {
    if (!n++) {
      fp = ff;
    } else if (ff != fp) {
      fp = nullptr;
      break;
    }
  }
  return n <= 1 || !fp;
}

int Mesh::num_vertices(Face f) const {
  int n = 0;
  for (HEdge he : corners(f)) {
    dummy_use(he);
    n++;
  }
  return n;
}

bool Mesh::is_boundary(Face f) const {
  for (Vertex v : vertices(f)) {
    if (is_boundary(v)) return true;
  }
  return false;
}

Face Mesh::opp_face(Face f, Edge e) const {
  Face f1 = face1(e), f2 = face2(e);
  if (f == f1) return f2;
  if (f == f2) return f1;
  assertnever("Face not adjacent to Edge");
}

Face Mesh::opp_face(Vertex v, Face f) const { return opp_face(f, opp_edge(v, f)); }

void Mesh::get_vertices(Face f, Array<Vertex>& va) const {
  va.init(0);
  for (Vertex v : vertices(f)) va.push(v);
}

Vertex Mesh::vertex(Face f, int i) const {
  assertx(i >= 0);
  int j = 0;
  for (Vertex v : vertices(f)) {
    if (j++ == i) return v;
  }
  SHOW(i, num_vertices(f));
  assertnever("Face has too few vertices");
}

// *** Edge

Vertex Mesh::opp_vertex(Edge e, Face f) const {
  HEdge he = hedge_from_ef(e, f);
  if (he->_next->_next->_next != he) assertnever("mesh face is not triangle");  // cheaper than is_triangle(f)
  return he->_next->_vert;
}

Edge Mesh::opp_boundary(Edge e, Vertex v) const {
  HEdge he = herep(e);
  assertx(is_boundary(he));
  if (he->_vert == v) {
    while (HEdge hen = clw_hedge(he)) he = hen;
    return he->_next->_edge;
  }
  if (he->_prev->_vert == v) {
    he = he->_prev;
    while (HEdge hen = ccw_hedge(he)) he = hen;
    return he->_edge;
  }
  assertnever("Vertex not on Edge");
}

Vertex Mesh::vertex_between_edges(Edge e1, Edge e2) const {
  if (vertex1(e1) == vertex1(e2) || vertex1(e1) == vertex2(e2)) return vertex1(e1);
  if (vertex2(e1) == vertex1(e2) || vertex2(e1) == vertex2(e2)) return vertex2(e1);
  assertnever("edges not adjacent");
}

// *** Other associations

Edge Mesh::query_edge(Vertex v, Vertex w) const {
  HEdge he = query_hedge(v, w);
  if (he) return he->_edge;
  he = query_hedge(w, v);
  if (he) return he->_edge;
  return nullptr;
}

Edge Mesh::ordered_edge(Vertex v1, Vertex v2) const {
  HEdge he = assertx(query_hedge(v1, v2));
  Edge e = he->_edge;
  assertx(e->_herep == he);
  return e;
}

// *** Counting routines

Vertex Mesh::random_vertex(Random& r) const {
  assertx(num_vertices());
  return _id2vertex.get_random_value(r);
}

Face Mesh::random_face(Random& r) const {
  assertx(num_faces());
  return _id2face.get_random_value(r);
}

Edge Mesh::random_edge(Random& r) const {
  Face f = random_face(r);
  int vi = r.get_unsigned(num_vertices(f));
  HEdge hef = nullptr;
  for (HEdge he : corners(f)) {
    if (!vi--) {
      hef = he;
      break;
    }
  }
  return assertx(hef)->_edge;
}

// *** Mesh operations

bool Mesh::legal_edge_collapse(Edge e) const {
  if (debug() >= 1) valid(e);
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e);  // vo2 may be nullptr
  // Check that substituting v2 to v1 will not duplicate an edge in any
  // faces adjacent to v2 (besides f1 and f2).
  // (case of Face vertices being duplicated cannot happen here
  //  since only f1 and f2 can have both v1 and v2)
  for (Vertex v : vertices(v2)) {
    if (v == v1 || v == vo1 || v == vo2) continue;
    if (query_edge(v, v1)) return false;
  }
  return true;
}

bool Mesh::nice_edge_collapse(Edge e) const {
  if (debug() >= 1) valid(e);
  if (!legal_edge_collapse(e)) return false;
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Face f1 = face1(e), f2 = face2(e);
  assertx(is_triangle(f1));
  if (f2) assertx(is_triangle(f2));
  if (debug() >= 1) assertx(is_nice(v1) && is_nice(v2));
  // Requirements:
  // * 1 - If v1 and v2 are both boundary, (v1, v2) is a boundary edge
  if (!is_boundary(e) && is_boundary(v1) && is_boundary(v2)) return false;
  // * 2 - For all vertices adjacent to both v1 and v2, exists a face
  Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e);
  Set<Vertex> set;
  for (Vertex v : vertices(v1)) {
    if (v != vo1 && v != vo2) set.enter(v);
  }
  for (Vertex v : vertices(v2)) {
    if (v != vo1 && v != vo2 && !set.add(v)) return false;
  }
  // * 3 - two small base cases: single face and tetrahedron
  if (set.num() == 2 && is_boundary(e)) return false;                        // single face
  if (set.num() == 2 && !is_boundary(v1) && !is_boundary(v2)) return false;  // tetrahedron
  return true;
}

bool Mesh::legal_edge_swap(Edge e) const {
  if (debug() >= 1) valid(e);
  if (is_boundary(e)) return false;
  // illegal if cross edge already exists (as in tetrahedron)
  if (query_edge(side_vertex1(e), side_vertex2(e))) return false;
  return true;
}

void Mesh::collapse_edge_vertex(Edge e, Vertex vs) {
  assertx(legal_edge_collapse(e));
  HEdge he1 = hedge_from_ev1(e, vs);
  HEdge he2 = hedge_from_ev2(e, vs);
  Vertex vt = he1 ? he1->_vert : assertx(he2)->_prev->_vert;
  assertx(vt == opp_vertex(vs, e));  // optional
  // Create bogus hedges if boundaries
  Array<HEdge> ar_he;
  if (he1) ar_he.push(he1->_prev);
  if (he2) ar_he.push(he2->_next);
  create_bogus_hedges(ar_he);
  if (he1) {
    assertx(is_triangle(he1->_face));
    destroy_face(he1->_face);
  }
  if (he2) {
    assertx(is_triangle(he2->_face));
    destroy_face(he2->_face);
  }
  // Change remaining faces around vt to have vs instead
  for (HEdge he : Array<Corner>(corners(vt))) {
    // ends up deleting and recreating MEdge structures, which is great.
    remove_hedge(he, he->_prev->_vert);
    remove_hedge(he->_next, he->_vert);
    he->_vert = vs;
    enter_hedge(he, he->_prev->_vert);
    enter_hedge(he->_next, he->_vert);
  }
  // Destroy vertex vt
  destroy_vertex(vt);
  remove_bogus_hedges(ar_he);
}

void Mesh::collapse_edge(Edge e) { collapse_edge_vertex(e, vertex1(e)); }

Vertex Mesh::split_edge(Edge e, int id) {
  if (debug() >= 1) valid(e);
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Face f1 = face1(e), f2 = face2(e);                    // f2 could be nullptr
  Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e);  // implies triangles
  // Create bogus hedges if boundaries
  Array<HEdge> ar_he;
  for (Face f : faces(e)) {
    for (HEdge he : corners(f)) {
      if (he->_edge != e) ar_he.push(he);
    }
  }
  create_bogus_hedges(ar_he);  // note: temporarily causes mesh.ok() to fail
  // Destroy faces
  destroy_face(f1);
  if (f2) destroy_face(f2);
  // Create new vertex
  Vertex vn = id ? create_vertex_private(id) : create_vertex();
  // Create new faces
  create_face(vn, v2, vo1);
  create_face(vn, vo1, v1);
  if (vo2) {
    create_face(vn, v1, vo2);
    create_face(vn, vo2, v2);
  }
  remove_bogus_hedges(ar_he);
  return vn;
}

Edge Mesh::swap_edge(Edge e) {
  assertx(legal_edge_swap(e));
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Face f1 = face1(e), f2 = face2(e);
  Vertex vo1 = side_vertex1(e), vo2 = side_vertex2(e);  // implies triangles
  // Create bogus hedges if boundaries
  Array<HEdge> ar_he;
  for (Face f : faces(e)) {
    for (HEdge he : corners(f)) ar_he.push(he);
  }
  create_bogus_hedges(ar_he);
  // Destroy faces
  destroy_face(f1);
  destroy_face(f2);
  // Create new faces; may die if illegal
  create_face(v1, vo2, vo1);
  create_face(v2, vo1, vo2);
  remove_bogus_hedges(ar_he);
  return edge(vo1, vo2);
}

Vertex Mesh::split_vertex(Vertex v1, Vertex vs1, Vertex vs2, int v2i) {
  Vertex v2 = v2i ? create_vertex_private(v2i) : create_vertex();
  Stack<Corner> stackc;  // faces clw of vs1 and ccw of vs2
  Corner c = nullptr;
  if (vs1 && !c) {
    c = clw_corner(v1, edge(v1, vs1));
    for (; c; c = clw_corner(c)) {
      stackc.push(c);
      if (vs2 && corner_vertex(ccw_face_corner(c)) == vs2) break;
    }
  }
  if (vs2 && !c) {
    c = ccw_corner(v1, edge(v1, vs2));
    for (; c; c = ccw_corner(c)) {
      stackc.push(c);
      if (vs1 && corner_vertex(clw_face_corner(c)) == vs1) assertnever("");
    }
  }
  for (Corner cc : stackc) {
    // ends up deleting and recreating MEdge structures, which is great.
    HEdge he = cc;
    remove_hedge(he, he->_prev->_vert);
    remove_hedge(he->_next, he->_vert);
    he->_vert = v2;
    enter_hedge(he, he->_prev->_vert);
    enter_hedge(he->_next, he->_vert);
  }
  // info on edges around v1 still valid; edges on v2 all new
  return v2;
}

bool Mesh::legal_vertex_merge(Vertex vs, Vertex vt) {
  for (HEdge he : corners(vt)) {
    if (query_hedge(he->_prev->_vert, vs)) return false;
    if (query_hedge(vs, he->_next->_vert)) return false;
  }
  return true;
}

void Mesh::merge_vertices(Vertex vs, Vertex vt) {
  assertx(legal_vertex_merge(vs, vt));
  // We cannot introduce bogus hedges, because we intend to merge boundary edges together.
  // Change faces around vt to instead use vs.
  for (HEdge he : Array<Corner>(corners(vt))) {
    ASSERTX(he->_vert == vt);
    remove_hedge(he, he->_prev->_vert);
    remove_hedge(he->_next, he->_vert);
    he->_vert = vs;
    enter_hedge(he, he->_prev->_vert);
    enter_hedge(he->_next, he->_vert);
  }
  destroy_vertex(vt);
}

Vertex Mesh::center_split_face(Face f) {
  if (debug() >= 1) valid(f);
  Array<Vertex> va;
  get_vertices(f, va);
  // Create bogus hedges if boundaries
  Array<HEdge> ar_he(corners(f));
  create_bogus_hedges(ar_he);
  // Destroy face
  destroy_face(f);
  // Create new vertex and faces
  Vertex vn = create_vertex();
  for_int(i, va.num()) create_face(va[i], va[(i + 1) % va.num()], vn);
  remove_bogus_hedges(ar_he);
  return vn;
}

Edge Mesh::split_face(Face f, Vertex v1, Vertex v2) {
  if (debug() >= 1) valid(v1), valid(v2);
  assertx(!query_edge(v1, v2));
  Array<Vertex> va1, va2;
  for (Vertex v = v1;;) {
    va1.push(v);
    if (v == v2) break;
    v = ccw_vertex(f, v);
  }
  for (Vertex v = v2;;) {
    va2.push(v);
    if (v == v1) break;
    v = ccw_vertex(f, v);
  }
  // Create bogus hedges if boundaries
  Array<HEdge> ar_he(corners(f));
  create_bogus_hedges(ar_he);
  // Destroy face
  destroy_face(f);
  // Create new faces
  if (debug() >= 1) assertx(legal_create_face(va1) && legal_create_face(va2));
  create_face(va1);
  create_face(va2);
  remove_bogus_hedges(ar_he);
  return edge(v1, v2);
}

Array<Vertex> Mesh::gather_edge_coalesce_vertices(Edge e) const {
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Face f1 = face1(e), f2 = face2(e);
  Array<Vertex> va;
  if (1) {
    Array<Vertex> va1;
    get_vertices(f1, va1);  // slow but threadsafe
    Array<Vertex> va2;
    get_vertices(f2, va2);
    int nv1 = va1.num(), nv2 = va2.num();
    int i1 = 0;
    // Find one vertex common to both faces (v1)
    for (; i1 < nv1; i1++)
      if (va1[i1] == v1) break;
    int i2 = 0;
    for (; i2 < nv2; i2++)
      if (va2[i2] == v1) break;
    assertx(i1 < nv1 && i2 < nv2);
    // Find most clw vertex on face1 common to both
    while (va1[(i1 - 1 + nv1) % nv1] == va2[(i2 + 1) % nv2]) {
      i1 = (i1 - 1 + nv1) % nv1;
      i2 = (i2 + 1) % nv2;
    }
    // Let ic be the number of vertices common to both faces
    int ic = 1;
    for (;; ic++)
      if (va1[(i1 + ic) % nv1] != va2[(i2 - ic + nv2) % nv2]) break;
    for_intL(i, ic - 1, nv1) va.push(va1[(i1 + i) % nv1]);
    for_int(i, nv2 - ic + 1) va.push(va2[(i2 + i) % nv2]);
    dummy_use(v2);
  } else {
    // this version only works for a single edge between 2 faces
    for (Vertex v = v2;;) {
      va.push(v);
      v = ccw_vertex(f1, v);
      if (v == v1) break;
    }
    for (Vertex v = v1;;) {
      va.push(v);
      v = ccw_vertex(f2, v);
      if (v == v2) break;
    }
  }
  return va;
}

bool Mesh::legal_coalesce_faces(Edge e) {
  if (is_boundary(e)) {
    Warning("Boundary edge ignored");
    return false;
  }
  Array<Vertex> va = gather_edge_coalesce_vertices(e);
  {  // check for duplicate vertices
    Set<Vertex> setv;
    for (Vertex v : va) {
      if (!setv.add(v)) return false;
    }
  }
  {  // check that we get a "nice" face
    Face f1 = face1(e), f2 = face2(e);
    Face fp = nullptr;
    int i = 0;
    for (Face f : faces(e)) {
      for (Edge ee : edges(f)) {
        if (is_boundary(ee)) {
          fp = nullptr;
          break;
        }
        Face ff = opp_face(f, ee);
        if (ff == f1 || ff == f2) continue;
        if (!i++) {
          fp = ff;
        } else if (ff != fp) {
          fp = nullptr;
          break;
        }
      }
    }
    if (fp) {
      Warning("coalesce -> non-nice face");
      return false;
    }
  }
  return true;
}

Face Mesh::coalesce_faces(Edge e) {
  if (debug() >= 1) assertx(legal_coalesce_faces(e));
  Face f1 = face1(e), f2 = face2(e);
  Array<Vertex> va = gather_edge_coalesce_vertices(e);
  // See if any vertices can be deleted
  Set<Vertex> vbefore;
  for (Vertex v : vertices(f1)) vbefore.enter(v);
  for (Vertex v : vertices(f2)) vbefore.add(v);
  // Create bogus hedges if boundaries
  Array<HEdge> ar_he;
  for (Face f : faces(e)) {
    for (HEdge he : corners(f)) ar_he.push(he);
  }
  create_bogus_hedges(ar_he);
  // Destroy faces
  destroy_face(f1);
  destroy_face(f2);
  // Create new face
  if (debug() >= 1) assertx(legal_create_face(va));
  Face fn = create_face(va);
  remove_bogus_hedges(ar_he);
  // Delete any isolated vertices
  for (Vertex v : va) vbefore.remove(v);
  for (Vertex v : vbefore) destroy_vertex(v);
  if (debug() >= 3) ok();
  return fn;
}

Vertex Mesh::insert_vertex_on_edge(Edge e) {
  // If > 1 edge shared between f1 and f2, other shared edges will be
  //  destroyed and recreated -> will lose attributes.
  if (debug() >= 1) valid(e);
  // Create bogus hedges if boundaries
  Array<HEdge> ar_he;
  for (Face f : faces(e)) {
    for (HEdge he : corners(f)) ar_he.push(he);
  }
  create_bogus_hedges(ar_he);
  Vertex v1 = vertex1(e), v2 = vertex2(e);
  Face f1 = face1(e), f2 = face2(e);
  Array<Vertex> va1, va2;
  for (Vertex v = v2;;) {
    va1.push(v);
    v = ccw_vertex(f1, v);
    if (v == v2) break;
  }
  if (f2) {
    for (Vertex v = v1;;) {
      va2.push(v);
      v = ccw_vertex(f2, v);
      if (v == v1) break;
    }
  }
  destroy_face(f1);
  if (f2) destroy_face(f2);
  Vertex vn = create_vertex();
  va1.push(vn);
  f1 = create_face(va1);
  dummy_use(f1);
  if (f2) {
    va2.push(vn);
    f2 = create_face(va2);
    dummy_use(f2);
  }
  remove_bogus_hedges(ar_he);
  return vn;
}

Edge Mesh::remove_vertex_between_edges(Vertex vr) {
  Array<Face> fa(ccw_faces(vr));
  // Create bogus hedges if boundaries
  Array<HEdge> ar_he;
  for (Face f : fa) {
    for (HEdge he : corners(f)) ar_he.push(he);
  }
  create_bogus_hedges(ar_he);
  Vec2<Array<Vertex>> va;
  for_int(i, fa.num()) {
    for (Vertex v = vr;;) {
      v = ccw_vertex(fa[i], v);
      if (v == vr) break;
      va[i].push(v);
    }
  }
  for_int(i, fa.num()) destroy_face(fa[i]);
  destroy_vertex(vr);
  for_int(i, fa.num()) fa[i] = create_face(va[i]);
  remove_bogus_hedges(ar_he);
  return edge(va[0][0], va[0].last());
}

Array<Vertex> Mesh::fix_vertex(Vertex v) {
  Array<Vertex> new_vertices;
  PArray<HEdge, 8> hedges;
  for (HEdge he : v->_arhe) hedges.push(he->_prev);
  for (HEdge he : hedges) assertx(he->_vert == v);
  int num_he_processed = 0;
  PArray<HEdge, 8> component;
  for (HEdge herep : hedges) {
    if (herep->_vert != v) continue;  // half-edge already moved to a new vertex
    component.init(0);
    {
      HEdge he = herep;
      for (;;) {
        assertx(he->_vert == v);
        component.push(he);
        he = clw_hedge(he);
        if (!he || he == herep) break;
      }
      if (!he) {
        for (he = herep;;) {
          he = ccw_hedge(he);
          if (!he) break;
          assertx(he != herep && he->_vert == v);
          component.push(he);
        }
      }
    }
    if (num_he_processed + component.num() == hedges.num()) break;  // do not fix last component
    Vertex vnew = create_vertex();
    new_vertices.push(vnew);
    flags(vnew) = flags(v);  // it is reasonable to copy all vertex attributes
    for (HEdge he : component) {
      assertx(he->_vert == v);
      remove_hedge(he, he->_prev->_vert);
      remove_hedge(he->_next, he->_vert);
      he->_vert = vnew;
      enter_hedge(he, he->_prev->_vert);
      enter_hedge(he->_next, he->_vert);
    }
    num_he_processed += component.num();
  }
  ASSERTX(is_nice(v));
  return new_vertices;
}

bool Mesh::is_nice() const {
  for (Vertex v : vertices()) {
    if (!is_nice(v)) return false;
  }
  for (Face f : faces()) {
    if (!is_nice(f)) return false;
  }
  return true;
}

void Mesh::renumber() {
  int id = 1;
  // self-change to mesh is OK since ordered iterator creates copy of verts.
  for (Vertex v : ordered_vertices()) {
    ASSERTX(v->_id >= id);
    if (v->_id != id) {
      assertx(_id2vertex.remove(v->_id) == v);
      v->_id = id;
      _id2vertex.enter(id, v);
    }
    id++;
  }
  id = 1;
  // self-change to mesh is OK since ordered iterator creates copy of faces.
  for (Face f : ordered_faces()) {
    ASSERTX(f->_id >= id);
    if (f->_id != id) {
      assertx(_id2face.remove(f->_id) == f);
      f->_id = id;
      _id2face.enter(id, f);
    }
    id++;
  }
}

void Mesh::vertex_renumber_id_private(Vertex v, int newid) {
  if (v->_id == newid) return;
  assertx(_id2vertex.remove(v->_id) == v);
  v->_id = newid;
  _id2vertex.enter(newid, v);
  _vertexnum = max(_vertexnum, newid + 1);
}

void Mesh::face_renumber_id_private(Face f, int newid) {
  if (f->_id == newid) return;
  assertx(_id2face.remove(f->_id) == f);
  f->_id = newid;
  _id2face.enter(newid, f);
  _facenum = max(_facenum, newid + 1);
}

void Mesh::ok() const {
  // Check consistency of id2x (one way)
  for_map_key_value(_id2vertex, [&](int id, Vertex v) {
    valid(v);
    assertx(v->_id == id);
  });
  for_map_key_value(_id2face, [&](int id, Face f) {
    valid(f);
    assertx(f->_id == id);
  });
  // Look over Vertices
  Set<HEdge> sethe;
  for (Vertex v1 : vertices()) {
    assertx(_id2vertex.get(v1->_id) == v1);
    Set<Vertex> set;
    for (HEdge he : v1->_arhe) {
      // Check that HEdges are valid
      valid(he);
      assertx(he->_prev->_vert == v1);
      Vertex v2 = he->_vert;
      // Check that v2's are not duplicated
      assertx(set.add(v2));
      // Check that sym matches that in _arhe
      assertx(query_hedge(v2, v1) == he->_sym);
      // Check that HEdge sym is valid
      if (he->_sym) {
        valid(he->_sym);
        assertx(he->_sym->_edge == he->_edge);
        assertx(he->_sym->_sym == he);
        assertx(he->_sym->_vert == he->_prev->_vert);
        assertx(he->_sym->_prev->_vert == he->_vert);
      }
      // Check that hedges are unique, and one-to-one with ste (1)
      assertx(sethe.add(he));
      // Check that Faces reachable from Edges are valid
      valid(he->_face);
      // Check that each HEdge appears in its face
      bool found = false;
      for (HEdge hee : corners(he->_face)) {
        if (hee == he) {
          found = true;
          break;
        }
      }
      assertx(found);
      // Check that Edge is valid
      valid(he->_edge);
    }
  }
  // Look over Faces
  for (Face f : faces()) {
    assertx(_id2face.get(f->_id) == f);
    // Check that Face has valid erep
    assertx(herep(f)->_face == f);
    valid(herep(f)->_edge);
    Set<Vertex> set;
    for (Vertex v : vertices(f)) {
      // Check Face contains no duplicate Vertices
      assertx(set.add(v));
      // Check Vertices in face are valid
      valid(v);
    }
    assertx(set.num() >= 3);
    // Check Edges in Faces are one-to-one with _arhe (2)
    for (HEdge he : corners(f)) {
      assertx(he->_face == f);
      assertx(sethe.remove(he));
    }
  }
  // Check Edges in Faces are one-to-one with _arhe (3)
  assertx(sethe.empty());
  // Check _nedges
  {
    int i = 0;
    for (Edge e : edges()) {
      dummy_use(e);
      i++;
    }
    assertx(_nedges == i);
  }
}

bool Mesh::valid(Vertex v) const {
  assertx(v && v == _id2vertex.get(vertex_id(v)));
  return true;
}

bool Mesh::valid(Face f) const {
  assertx(f && f == _id2face.get(face_id(f)));
  return true;
}

bool Mesh::valid(Edge e) const {
  assertx(e);
  valid(vertex1(e));
  valid(vertex2(e));
  valid(face1(e));
  assertx(ordered_edge(vertex1(e), vertex2(e)) == e);
  return true;
}

bool Mesh::valid(Corner c) const {
  HEdge he = c;  // they are equivalent
  assertx(he && he->_next && he->_prev && he->_face && he->_vert);
  HH_ASSUME(he);
  Edge e = assertx(he->_edge);
  assertx(hedge_from_ev2(e, he->_vert) == he);
  return true;
}

// *** Iterators

Mesh::OrderedVertices_range::OrderedVertices_range(const Mesh& mesh) {
  _vertices.reserve(mesh.num_vertices());
  for (Vertex v : mesh.vertices()) _vertices.push(v);
  sort(_vertices, [&mesh](Vertex v1, Vertex v2) { return mesh.vertex_id(v1) < mesh.vertex_id(v2); });
}

Mesh::OrderedFaces_range::OrderedFaces_range(const Mesh& mesh) {
  _faces.reserve(mesh.num_faces());
  for (Face f : mesh.faces()) _faces.push(f);
  sort(_faces, [&mesh](Face f1, Face f2) { return mesh.face_id(f1) < mesh.face_id(f2); });
}

// *** Mesh protected

Mesh::HEdge Mesh::most_clw_hedge(Vertex v) const {
  assertx(is_nice(v));
  HEdge he = herep(v);
  if (!he) return nullptr;
  for (HEdge hef = he;;) {
    HEdge hen = clw_hedge(he);
    if (!hen || hen == hef) break;
    he = hen;
  }
  return he;
}

Mesh::HEdge Mesh::most_ccw_hedge(Vertex v) const {
  assertx(is_nice(v));
  HEdge he = herep(v);
  if (!he) return nullptr;
  for (HEdge hef = he;;) {
    HEdge hen = ccw_hedge(he);
    if (!hen || hen == hef) break;
    he = hen;
  }
  return he;
}

Mesh::HEdge Mesh::get_hedge(Vertex v, Face f) const {
  for (Corner he : corners(v)) {
    if (he->_face == f) {
      return he;
    }
  }
  assertnever("Face not adjacent to Vertex");
}

Mesh::HEdge Mesh::query_hedge(Vertex v1, Vertex v2) const {
  for (HEdge he : v1->_arhe) {
    if (he->_vert == v2) return he;
  }
  return nullptr;
}

// Must have:           vert
// Defines:             sym, edge
// Must define elsewhere: next, prev, face, edge
void Mesh::enter_hedge(HEdge he, Vertex v1) {
  Vertex v2 = he->_vert;
  if (debug() >= 1) {
    valid(v1);
    valid(v2);
    assertx(!query_hedge(v1, v2) && !v1->_arhe.contains(he));
  }
  v1->_arhe.push(he);
  HEdge hes = he->_sym = query_hedge(v2, v1);
  if (hes) {
    assertx(!hes->_sym);
    hes->_sym = he;
    Edge e = hes->_edge;
    he->_edge = e;
    if (he->_vert->_id > hes->_vert->_id) e->_herep = he;
  } else {
    _nedges++;
    Edge e = new MEdge(he);
    he->_edge = e;
  }
}

// Must have:           vert, sym, edge
// Must do later:       delete he
// Note: must be careful: could have he == herep(he->_face) ! :
//   destroy_face() : deletes face -> ok.
//   substitute_face_vertex() : reintroduces edge -> ok.
void Mesh::remove_hedge(HEdge he, Vertex v1) {
  Vertex v2 = he->_vert;
  if (debug() >= 1) {
    valid(v1);
    valid(v2);
    assertx(he == query_hedge(v1, v2));
    assertx(he->_sym == query_hedge(v2, v1));
  }
  Edge e = he->_edge;
  HEdge hes = he->_sym;
  if (hes) {
    hes->_sym = nullptr;
    if (e->_herep == he) e->_herep = hes;
  } else {
    --_nedges;
    // e->herep = nullptr;     // optional
    delete e;
  }
  // he->_edge = nullptr;         // optional
  assertx(v1->_arhe.remove_unordered(he));  // slow, shucks
}

void Mesh::create_bogus_hedges(ArrayView<HEdge> ar_he) {
  for_int(i, ar_he.num()) {
    HEdge& he = ar_he[i];
    if (is_boundary(he)) {
      HEdge heo = he;
      Vertex v1 = heo->_vert;
      he = new MHEdge;
      HH_ASSUME(heo->_prev);
      he->_vert = heo->_prev->_vert;
      he->_prev = nullptr;  // note: temporarily causes mesh.ok() to fail
      he->_next = nullptr;
      he->_face = reinterpret_cast<Face>(v1);  // temporary overload
      enter_hedge(he, v1);
    } else {
      he = nullptr;
    }
  }
}

void Mesh::remove_bogus_hedges(CArrayView<HEdge> ar_he) {
  for (HEdge he : ar_he) {
    if (!he) continue;
    Vertex v1 = reinterpret_cast<Vertex>(he->_face);  // temporary overload
    remove_hedge(he, v1);
    delete he;
  }
}

}  // namespace hh
