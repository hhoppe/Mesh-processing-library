// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Mesh.h"

#include "libHh/Array.h"
#include "libHh/Random.h"
using namespace hh;

namespace {

void show_mesh(const Mesh& mesh) {
  showf("Mesh {\n  Vertices (%d) {\n", mesh.num_vertices());
  for (Vertex v : mesh.ordered_vertices()) showf("    %d\n", mesh.vertex_id(v));
  showf("  } EndVertices\n  Edges (%d)\n  Faces (%d) {\n", mesh.num_edges(), mesh.num_faces());
  for (Face f : mesh.ordered_faces()) {
    showf("    Face %d {", mesh.face_id(f));
    for (Vertex v : mesh.vertices(f)) showf(" %d", mesh.vertex_id(v));
    showf(" }\n");
  }
  SHOW("  } EndFaces\n} EndMesh");
}

// Creates nv vertices and the faces given as lists of vertex indices; returns the vertices.
Array<Vertex> create_mesh(Mesh& mesh, int nv, CArrayView<Array<int>> faces) {
  Array<Vertex> va;
  for_int(i, nv) va.push(mesh.create_vertex());
  for (const Array<int>& face : faces) {
    Array<Vertex> fva;
    for (const int i : face) fva.push(va[i]);
    assertx(mesh.legal_create_face(fva));
    mesh.create_face(fva);
  }
  return va;
}

int euler_characteristic(const Mesh& mesh) { return mesh.num_vertices() - mesh.num_edges() + mesh.num_faces(); }

// Prints the faces of the mesh, ordered by face id.
void show_faces(const Mesh& mesh) {
  for (Face f : mesh.ordered_faces()) std::cout << " " << f;
  std::cout << "\n";
}

// Verifies the consistency of the corner, edge, and face adjacencies of a mesh with nice vertices.
void verify_adjacencies(const Mesh& mesh) {
  mesh.ok();
  int sum_degrees = 0, num_corners = 0;
  for (Vertex v : mesh.vertices()) {
    assertx(mesh.is_nice(v));
    sum_degrees += mesh.degree(v);
    assertx(ranges::distance(mesh.vertices(v)) == mesh.degree(v));
    assertx(ranges::distance(mesh.edges(v)) == mesh.degree(v));
    const int nfaces = narrow_cast<int>(ranges::distance(mesh.faces(v)));
    assertx(ranges::distance(mesh.ccw_faces(v)) == nfaces);
    assertx(ranges::distance(mesh.corners(v)) == nfaces);
    assertx(mesh.num_boundaries(v) == (mesh.degree(v) - nfaces));  // A nice vertex has at most one boundary.
    assertx(ranges::distance(mesh.ccw_vertices(v)) == mesh.degree(v));
    assertx(ranges::distance(mesh.ccw_edges(v)) == mesh.degree(v));
    for (Corner c : mesh.ccw_corners(v)) {
      assertx(mesh.corner_vertex(c) == v && mesh.corner(v, mesh.corner_face(c)) == c);
      if (Corner c2 = mesh.ccw_corner(c)) assertx(mesh.clw_corner(c2) == c);
      num_corners++;
    }
    // Successive ccw vertices are linked by the ccw faces.
    if (mesh.degree(v)) {
      Vertex vccw = mesh.most_ccw_vertex(v);
      assertx(!mesh.ccw_vertex(v, vccw) == mesh.is_boundary(v));
      Vertex vclw = mesh.most_clw_vertex(v);
      assertx(!mesh.clw_vertex(v, vclw) == mesh.is_boundary(v));
    }
  }
  assertx(sum_degrees == 2 * mesh.num_edges());
  int sum_face_vertices = 0;
  for (Face f : mesh.faces()) {
    const int nv = mesh.num_vertices(f);
    sum_face_vertices += nv;
    assertx(mesh.is_triangle(f) == (nv == 3));
    int i = 0;
    for (Corner c : mesh.corners(f)) {
      assertx(mesh.corner_face(c) == f && mesh.corner_vertex(c) == mesh.vertex(f, i));
      assertx(mesh.clw_face_corner(mesh.ccw_face_corner(c)) == c);
      assertx(mesh.clw_face_edge(c) == mesh.edge(mesh.vertex(f, (i + nv - 1) % nv), mesh.vertex(f, i)));
      assertx(mesh.ccw_face_edge(c) == mesh.edge(mesh.vertex(f, i), mesh.vertex(f, (i + 1) % nv)));
      i++;
    }
    assertx(i == nv);
    bool has_boundary_vertex = false;
    for (Vertex v : mesh.vertices(f)) has_boundary_vertex |= mesh.is_boundary(v);
    assertx(mesh.is_boundary(f) == has_boundary_vertex);
    for (Edge e : mesh.edges(f)) {
      assertx(mesh.opp_face(f, e) == (mesh.face1(e) == f ? mesh.face2(e) : mesh.face1(e)));
      assertx(mesh.ccw_edge(f, mesh.clw_edge(f, e)) == e);
    }
    if (mesh.is_triangle(f)) {
      const Vec3<Vertex> tv = mesh.triangle_vertices(f);
      const Vec3<Corner> tc = mesh.triangle_corners(f);
      for_int(j, 3) {
        assertx(mesh.corner_vertex(tc[j]) == tv[j] && tv[j] == mesh.vertex(f, j));
        const Edge eopp = mesh.opp_edge(tv[j], f);
        assertx(mesh.opp_vertex(eopp, f) == tv[j]);
        assertx(mesh.opp_face(tv[j], f) == mesh.opp_face(f, eopp));
      }
    }
  }
  assertx(sum_face_vertices == num_corners);
  int num_edges = 0;
  for (Edge e : mesh.edges()) {
    num_edges++;
    const Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
    assertx(mesh.edge(v1, v2) == e && mesh.edge(v2, v1) == e);
    assertx(mesh.vertex(e, 0) == v1 && mesh.vertex(e, 1) == v2);
    assertx(mesh.vertices(e) == V(v1, v2) || mesh.vertices(e) == V(v2, v1));  // The order is unspecified.
    assertx(mesh.face(e, 0) == mesh.face1(e) && mesh.face(e, 1) == mesh.face2(e));
    assertx(mesh.faces(e).num() == (mesh.is_boundary(e) ? 1 : 2));
    // The face1 of an edge contains the oriented edge (v1, v2).
    assertx(mesh.face(v1, v2) == mesh.face1(e));
    assertx(mesh.ccw_face(v1, e) == mesh.face1(e) && mesh.clw_face(v2, e) == mesh.face1(e));
    if (mesh.is_boundary(e)) {
      assertx(mesh.ordered_edge(v1, v2) == e);
      assertx(mesh.is_boundary(v1) && mesh.is_boundary(v2));
      // The boundary edges form closed loops.
      assertx(mesh.ccw_boundary(mesh.clw_boundary(e)) == e);
      assertx(mesh.vertex_between_edges(e, mesh.clw_boundary(e)) == v2);
    } else {
      assertx(mesh.face(v2, v1) == mesh.face2(e));
    }
    if (mesh.is_triangle(mesh.face1(e))) assertx(mesh.side_vertex(e, 0) == mesh.opp_vertex(e, mesh.face1(e)));
  }
  assertx(num_edges == mesh.num_edges() && ranges::distance(mesh.edges()) == num_edges);
}

// A closed tetrahedron.
void test_tetrahedron() {
  Mesh mesh;
  const Array<Vertex> va =
      create_mesh(mesh, 4, V(Array<int>{0, 1, 2}, Array<int>{0, 2, 3}, Array<int>{0, 3, 1}, Array<int>{1, 3, 2}));
  verify_adjacencies(mesh);
  assertx(mesh.is_nice());
  SHOW(mesh.num_vertices(), mesh.num_edges(), mesh.num_faces(), euler_characteristic(mesh));
  for (Vertex v : mesh.vertices()) assertx(mesh.degree(v) == 3 && !mesh.is_boundary(v) && mesh.is_nice(v));
  for (Edge e : mesh.edges()) {
    assertx(!mesh.is_boundary(e));
    // Collapsing an edge is legal but would leave two faces on the same three vertices (not nice), and swapping it
    // would duplicate an existing edge.
    assertx(mesh.legal_edge_collapse(e) && !mesh.nice_edge_collapse(e) && !mesh.legal_edge_swap(e));
  }
  for (Face f : mesh.faces()) assertx(!mesh.is_boundary(f) && mesh.is_nice(f));
  show_faces(mesh);
  SHOW(mesh.edge(va[0], va[1]), mesh.corner(va[2], mesh.face(va[0], va[1])));
  SHOW(mesh.query_edge(va[0], va[0]));
  // Copy, including the flags.
  const FlagMask vflag = Mesh::allocate_Vertex_flag(), fflag = Mesh::allocate_Face_flag();
  const FlagMask eflag = Mesh::allocate_Edge_flag();
  mesh.flags(va[2]).flag(vflag) = true;
  mesh.flags(mesh.face(va[0], va[1])).flag(fflag) = true;
  mesh.flags(mesh.edge(va[1], va[3])).flag(eflag) = true;
  Mesh mesh2;
  mesh2.copy(mesh);
  verify_adjacencies(mesh2);
  for (Vertex v : mesh.vertices()) {
    const Vertex v2 = mesh2.id_vertex(mesh.vertex_id(v));
    assertx(mesh2.flags(v2).flag(vflag) == (v == va[2]));
  }
  for (Face f : mesh.faces()) {
    const Face f2 = mesh2.id_face(mesh.face_id(f));
    Array<int> ids, ids2;
    for (Vertex v : mesh.vertices(f)) ids.push(mesh.vertex_id(v));
    for (Vertex v : mesh2.vertices(f2)) ids2.push(mesh2.vertex_id(v));
    assertx(ids == ids2);
    assertx(mesh2.flags(f2).flag(fflag) == mesh.flags(f).flag(fflag));
  }
  for (Edge e : mesh.edges()) {
    const Edge e2 =
        mesh2.edge(mesh2.id_vertex(mesh.vertex_id(mesh.vertex1(e))), mesh2.id_vertex(mesh.vertex_id(mesh.vertex2(e))));
    assertx(mesh2.flags(e2).flag(eflag) == mesh.flags(e).flag(eflag));
  }
  // Move construction and move assignment.
  Mesh mesh3(std::move(mesh2));
  // NOLINTNEXTLINE(bugprone-use-after-move): the moved-from object is left empty.
  assertx(mesh2.empty() && mesh2.num_faces() == 0 && mesh3.num_faces() == 4);
  Mesh mesh4;
  mesh4.create_vertex();
  mesh4 = std::move(mesh3);
  // NOLINTNEXTLINE(bugprone-use-after-move): the moved-from object is left empty.
  assertx(mesh4.num_vertices() == 4 && mesh3.empty());
  verify_adjacencies(mesh4);
  // Random access returns elements of the mesh.
  Random random(1);
  for_int(i, 10) {
    assertx(mesh.id_vertex(mesh.vertex_id(mesh.random_vertex(random))));
    assertx(mesh.id_face(mesh.face_id(mesh.random_face(random))));
    const Edge e = mesh.random_edge(random);
    assertx(mesh.edge(mesh.vertex1(e), mesh.vertex2(e)) == e);
  }
  mesh.clear();
  assertx(mesh.empty() && !mesh.num_edges());
}

// A hexagonal fan: an interior vertex 0 surrounded by the boundary vertices 1..6.
void create_fan(Mesh& mesh, Array<Vertex>& va) {
  Array<Array<int>> faces;
  for_int(i, 6) faces.push(Array<int>{0, 1 + i, 1 + (i + 1) % 6});
  va = create_mesh(mesh, 7, faces);
}

// Edit operations on a triangle mesh, which preserve the Euler characteristic of the disk.
void test_fan() {
  Mesh mesh;
  Array<Vertex> va;
  create_fan(mesh, va);
  verify_adjacencies(mesh);
  assertx(mesh.is_nice());
  SHOW(mesh.num_vertices(), mesh.num_edges(), mesh.num_faces(), euler_characteristic(mesh));
  assertx(!mesh.is_boundary(va[0]) && mesh.degree(va[0]) == 6);
  assertx(mesh.is_boundary(va[1]) && mesh.degree(va[1]) == 3 && mesh.num_boundaries(va[1]) == 1);
  // The ccw orderings about the center vertex and about a boundary vertex.
  const auto show_ids = [&](auto&& range) {
    for (Vertex v : range) std::cout << " " << mesh.vertex_id(v);
    std::cout << "\n";
  };
  show_ids(mesh.ccw_vertices(va[1]));  // From most clw to most ccw.
  SHOW(mesh.most_clw_vertex(va[1]), mesh.most_ccw_vertex(va[1]));
  SHOW(mesh.most_clw_face(va[1]), mesh.most_ccw_face(va[1]));
  SHOW(mesh.most_clw_edge(va[1]), mesh.most_ccw_edge(va[1]));
  assertx(mesh.ccw_face(va[1], mesh.most_clw_face(va[1])) == mesh.most_ccw_face(va[1]));
  assertx(mesh.clw_face(va[1], mesh.most_ccw_face(va[1])) == mesh.most_clw_face(va[1]));
  assertx(!mesh.ccw_face(va[1], mesh.most_ccw_face(va[1])));
  assertx(mesh.ccw_vertex(va[0], va[1]) == va[2] && mesh.clw_vertex(va[0], va[1]) == va[6]);
  assertx(mesh.ccw_face(va[0], mesh.face(va[0], va[1])) == mesh.face(va[0], va[2]));
  // Walk the boundary loop.
  {
    const Edge e0 = mesh.edge(va[1], va[2]);
    int count = 0;
    for (Edge e = e0;;) {
      count++;
      e = mesh.clw_boundary(e);
      if (e == e0) break;
    }
    SHOW(count);
  }
  // Swap an interior edge.
  {
    const Edge e = mesh.edge(va[0], va[1]);
    assertx(mesh.legal_edge_swap(e) && mesh.legal_edge_collapse(e) && mesh.nice_edge_collapse(e));
    const Edge enew = mesh.swap_edge(e);
    assertx(mesh.vertices(enew) == V(va[6], va[2]) || mesh.vertices(enew) == V(va[2], va[6]));
    verify_adjacencies(mesh);
    SHOW(mesh.degree(va[0]), mesh.degree(va[1]), mesh.degree(va[2]));
    assertx(!mesh.query_edge(va[0], va[1]));
    show_faces(mesh);
  }
  // Split an interior edge and a boundary edge.
  {
    const Vertex v = mesh.split_edge(mesh.edge(va[0], va[3]));
    verify_adjacencies(mesh);
    assertx(mesh.degree(v) == 4 && !mesh.is_boundary(v));
    const Vertex v2 = mesh.split_edge(mesh.edge(va[3], va[4]), 100);
    verify_adjacencies(mesh);
    assertx(mesh.vertex_id(v2) == 100 && mesh.degree(v2) == 3 && mesh.is_boundary(v2));
    SHOW(mesh.num_vertices(), mesh.num_edges(), mesh.num_faces(), euler_characteristic(mesh));
  }
  // Collapse an interior edge, then a boundary edge.
  {
    const Edge e = mesh.edge(va[0], va[4]);
    assertx(mesh.nice_edge_collapse(e));
    mesh.collapse_edge_vertex(e, va[4]);   // Keep va[4], at the boundary.
    assertx(!mesh.id_retrieve_vertex(1));  // Vertex va[0] is gone.
    verify_adjacencies(mesh);
    const Edge e2 = mesh.edge(va[5], va[6]);
    assertx(mesh.is_boundary(e2) && mesh.nice_edge_collapse(e2));
    mesh.collapse_edge(e2);
    verify_adjacencies(mesh);
    SHOW(mesh.num_vertices(), mesh.num_edges(), mesh.num_faces(), euler_characteristic(mesh));
    show_faces(mesh);
  }
  // Split a face about a new center vertex.
  {
    const Face f = mesh.ordered_faces().begin()[0];
    const Vertex v = mesh.center_split_face(f);
    verify_adjacencies(mesh);
    assertx(mesh.degree(v) == 3 && ranges::distance(mesh.faces(v)) == 3);
    SHOW(mesh.num_vertices(), mesh.num_edges(), mesh.num_faces(), euler_characteristic(mesh));
  }
  // Renumber the vertices and faces to consecutive ids.
  mesh.renumber();
  verify_adjacencies(mesh);
  {
    int i = 1;
    for (Vertex v : mesh.ordered_vertices()) assertx(mesh.vertex_id(v) == i++);
    i = 1;
    for (Face f : mesh.ordered_faces()) assertx(mesh.face_id(f) == i++);
    assertx(mesh.id_retrieve_vertex(mesh.num_vertices()) && !mesh.id_retrieve_vertex(mesh.num_vertices() + 1));
    assertx(mesh.id_retrieve_face(mesh.num_faces()) && !mesh.id_retrieve_face(mesh.num_faces() + 1));
    show_faces(mesh);
  }
  // Destroy all faces and then all vertices.
  for (Face f : Array<Face>(mesh.faces())) mesh.destroy_face(f);
  assertx(!mesh.num_faces() && !mesh.num_edges());
  for (Vertex v : Array<Vertex>(mesh.vertices())) {
    assertx(!mesh.degree(v));
    mesh.destroy_vertex(v);
  }
  assertx(mesh.empty());
}

// Operations on general polygonal faces.
void test_polygons() {
  Mesh mesh;
  // Two quadrilaterals sharing the edge (1, 2).
  const Array<Vertex> va = create_mesh(mesh, 6, V(Array<int>{0, 1, 2, 3}, Array<int>{1, 4, 5, 2}));
  verify_adjacencies(mesh);
  // Split a quadrilateral into two triangles.
  const Face fquad = mesh.face(va[0], va[1]);
  assertx(mesh.num_vertices(fquad) == 4 && !mesh.is_triangle(fquad));
  const Edge e = mesh.split_face(fquad, va[1], va[3]);
  verify_adjacencies(mesh);
  SHOW(mesh.num_faces(), mesh.num_edges());
  show_faces(mesh);
  // Coalesce them back.
  assertx(mesh.legal_coalesce_faces(e));
  const Face f = mesh.coalesce_faces(e);
  verify_adjacencies(mesh);
  SHOW(f, mesh.num_faces(), mesh.num_edges());
  // Insert a vertex on the interior edge, then remove it.
  const Vertex vnew = mesh.insert_vertex_on_edge(mesh.edge(va[1], va[2]));
  verify_adjacencies(mesh);
  // The two pentagons now share two edges, so they are not nice faces.
  assertx(!mesh.is_nice());
  SHOW(mesh.face(va[1], vnew), mesh.face(vnew, va[1]), mesh.degree(vnew));
  const Edge e2 = mesh.remove_vertex_between_edges(vnew);
  verify_adjacencies(mesh);
  assertx(mesh.vertices(e2) == V(va[1], va[2]) || mesh.vertices(e2) == V(va[2], va[1]));
  SHOW(mesh.num_vertices(), mesh.num_edges(), mesh.num_faces());
  show_faces(mesh);
  // Coalesce the two quadrilaterals into a hexagon.
  const Face fhex = mesh.coalesce_faces(mesh.edge(va[1], va[2]));
  verify_adjacencies(mesh);
  SHOW(fhex, mesh.num_vertices(fhex), mesh.num_edges());
  // Two faces sharing two consecutive edges, so coalescing them removes the vertex between those edges.
  Mesh mesh2;
  const Array<Vertex> vb = create_mesh(mesh2, 5, V(Array<int>{0, 1, 2, 3}, Array<int>{3, 2, 1, 4}));
  verify_adjacencies(mesh2);
  assertx(mesh2.degree(vb[2]) == 2);
  assertx(mesh2.legal_coalesce_faces(mesh2.edge(vb[1], vb[2])));
  const Face fc = mesh2.coalesce_faces(mesh2.edge(vb[1], vb[2]));
  verify_adjacencies(mesh2);
  SHOW(fc, mesh2.num_vertices());
  {
    // Insert a vertex on a boundary edge, then remove it; the other edges of the modified faces are preserved.
    Mesh mesh3;
    const Array<Vertex> vc = create_mesh(mesh3, 4, V(Array<int>{0, 1, 2}, Array<int>{0, 2, 3}));
    const Edge e12 = mesh3.edge(vc[1], vc[2]), e02 = mesh3.edge(vc[0], vc[2]);
    const Vertex vn = mesh3.insert_vertex_on_edge(mesh3.edge(vc[0], vc[1]));
    verify_adjacencies(mesh3);
    assertx(mesh3.is_boundary(vn) && mesh3.degree(vn) == 2 && mesh3.num_vertices(mesh3.face(vc[0], vn)) == 4);
    assertx(mesh3.edge(vc[1], vc[2]) == e12 && mesh3.edge(vc[0], vc[2]) == e02);
    const Edge e3 = mesh3.remove_vertex_between_edges(vn);
    verify_adjacencies(mesh3);
    assertx(mesh3.is_boundary(e3) && mesh3.query_edge(vc[0], vc[1]) == e3);
    assertx(mesh3.edge(vc[1], vc[2]) == e12 && mesh3.edge(vc[0], vc[2]) == e02);
    SHOW(mesh3.num_vertices(), mesh3.num_edges(), mesh3.num_faces());
  }
  {
    // Remove a vertex from the boundary of a single quadrilateral, leaving a triangle.
    Mesh mesh4;
    const Array<Vertex> vd = create_mesh(mesh4, 4, V(Array<int>{0, 1, 2, 3}));
    assertx(!mesh4.legal_vertex_merge(vd[0], vd[2]));  // Opposite vertices of the same face.
    const Edge e23 = mesh4.edge(vd[2], vd[3]), e30 = mesh4.edge(vd[3], vd[0]);
    const Edge e4 = mesh4.remove_vertex_between_edges(vd[1]);
    verify_adjacencies(mesh4);
    assertx(mesh4.is_boundary(e4) && mesh4.query_edge(vd[2], vd[0]) == e4);
    assertx(mesh4.edge(vd[2], vd[3]) == e23 && mesh4.edge(vd[3], vd[0]) == e30);
    SHOW(mesh4.num_vertices(), mesh4.num_edges(), mesh4.num_faces());
    show_faces(mesh4);
  }
}

// Vertex merging, splitting, and fixing of non-nice vertices.
void test_vertex_operations() {
  {
    // Two separate triangles (0, 1, 2) and (3, 4, 5), where vertices 4 and 5 coincide with 2 and 1.
    Mesh mesh;
    const Array<Vertex> va = create_mesh(mesh, 6, V(Array<int>{0, 1, 2}, Array<int>{3, 4, 5}));
    // Merging two vertices of the same face (or a vertex with itself) would create a face with a duplicate vertex.
    assertx(!mesh.legal_vertex_merge(va[0], va[2]) && !mesh.legal_vertex_merge(va[2], va[0]));
    assertx(!mesh.legal_vertex_merge(va[0], va[0]));
    assertx(mesh.legal_vertex_merge(va[2], va[4]));
    mesh.merge_vertices(va[2], va[4]);
    // The shared vertex 2 now has two separate partial rings, so it is not nice.
    assertx(!mesh.is_nice(va[2]) && !mesh.is_nice() && mesh.num_boundaries(va[2]) == 2);
    // Now the faces are (0, 1, 2) and (3, 2, 5), so merging vertex 5 into 0 would duplicate the oriented edge (2, 0).
    assertx(!mesh.legal_vertex_merge(va[0], va[5]));
    assertx(mesh.legal_vertex_merge(va[1], va[5]));
    mesh.merge_vertices(va[1], va[5]);
    verify_adjacencies(mesh);
    SHOW(mesh.num_vertices(), mesh.num_edges(), mesh.num_faces());
    show_faces(mesh);
    assertx(!mesh.is_boundary(mesh.edge(va[1], va[2])));
  }
  {
    // A bowtie: two triangles sharing only vertex 0, which is therefore not nice.
    Mesh mesh;
    const Array<Vertex> va = create_mesh(mesh, 5, V(Array<int>{0, 1, 2}, Array<int>{0, 3, 4}));
    mesh.ok();
    assertx(!mesh.is_nice(va[0]) && mesh.is_nice(va[1]) && !mesh.is_nice());
    SHOW(mesh.num_boundaries(va[0]), mesh.degree(va[0]));
    const Array<Vertex> new_vertices = mesh.fix_vertex(va[0]);
    assertx(new_vertices.num() == 1);
    verify_adjacencies(mesh);
    assertx(mesh.degree(va[0]) == 2 && mesh.degree(new_vertices[0]) == 2);
    SHOW(mesh.num_vertices(), mesh.num_edges());
    // Fixing a nice vertex does nothing.
    assertx(mesh.fix_vertex(va[1]).num() == 0);
  }
  {
    // Faces (0, 1, 2) and (0, 2, 1) form a legal but not nice mesh.
    Mesh mesh;
    const Array<Vertex> va = create_mesh(mesh, 3, V(Array<int>{0, 1, 2}));
    assertx(mesh.legal_create_face(V(va[0], va[2], va[1])));
    assertx(!mesh.legal_create_face(V(va[0], va[1], va[2])));  // The oriented edges already exist.
    assertx(!mesh.legal_create_face(V(va[0], va[0], va[1])));  // A duplicate vertex.
    const Face f = mesh.create_face(va[0], va[2], va[1]);
    mesh.ok();
    assertx(!mesh.is_nice(f) && !mesh.is_nice());
  }
  {
    // Split the center vertex of the fan into two vertices, moving to the new vertex the faces clw of vs1 = va[1]
    // and ccw of vs2 = va[4].  This leaves a hole, so that vertices vs1 and vs2 are not nice.
    Mesh mesh;
    Array<Vertex> va;
    create_fan(mesh, va);
    const Vertex v2 = mesh.split_vertex(va[0], va[1], va[4], 0);
    mesh.ok();
    assertx(!mesh.is_nice(va[1]) && !mesh.is_nice(va[4]) && mesh.is_nice(va[0]) && mesh.is_nice(v2));
    SHOW(mesh.degree(va[0]), mesh.degree(v2), mesh.num_faces(), mesh.num_edges());
    show_faces(mesh);
    // Fill the hole with two faces, as in a progressive mesh vertex split.
    mesh.create_face(va[1], va[0], v2);
    mesh.create_face(va[0], va[4], v2);
    verify_adjacencies(mesh);
    assertx(mesh.is_nice());
    SHOW(mesh.num_vertices(), mesh.num_edges(), mesh.num_faces(), euler_characteristic(mesh));
  }
}

}  // namespace

int main() {
  Mesh mesh;
  Vertex v1 = mesh.create_vertex();
  Vertex v2 = mesh.create_vertex();
  Vertex v3 = mesh.create_vertex();
  Vertex v4 = mesh.create_vertex();
  show_mesh(mesh);

  Face f0, f1;
  f0 = mesh.create_face(V(v1, v2, v3));
  dummy_use(f0);
  show_mesh(mesh);
  assertx(!mesh.legal_create_face(V(v1, v2, v4)));
  f1 = mesh.create_face(v2, v1, v4);
  dummy_use(f1);
  show_mesh(mesh);
  mesh.ok();
  //       2
  //   3   |   4
  //       1
  assertx(mesh.degree(v1) == 3);
  assertx(mesh.degree(v3) == 2);

  Vertex v5;
  {
    mesh.ok();
    // (1, 2, 3), (2, 1, 4)
    Edge e = mesh.edge(v1, v2);
    for (Face f : mesh.faces(e)) SHOW(mesh.face_id(f));
    SHOW("done");
    mesh.ok();
    v5 = mesh.split_edge(e);
    assertx(v5);
    // (5, 2, 3), (5, 3, 1), (5, 1, 4), (5, 4, 2)
  }
  //       2
  //   3   5   4
  //       1
  show_mesh(mesh);
  assertx(mesh.degree(v3) == 3);
  assertx(mesh.degree(v5) == 4);
  assertx(mesh.is_nice(v1));
  assertx(mesh.is_boundary(v2));
  assertx(!mesh.is_boundary(v5));
  assertx(mesh.num_vertices() == 5);
  assertx(mesh.num_faces() == 4);
  assertx(mesh.num_edges() == 8);
  {
    Array<Vertex> va(mesh.ccw_vertices(v5));
    assertx(va.num() == 4);
    SHOW("vertices: 3 1 4 2");
    for_int(i, 4) SHOW(mesh.vertex_id(va[i]));
  }
  {
    Array<Face> fa(mesh.ccw_faces(v2));
    assertx(fa.num() == 2);
    Face pf1 = mesh.face(v5, v2 /*, v3*/);
    Face pf2 = mesh.face(v5, v4 /*, v2*/);
    assertx(fa[0] == pf1 || fa[0] == pf2);
    assertx(fa[1] == pf1 || fa[1] == pf2);
  }
  assertx(mesh.opp_edge(v1, mesh.face(v1, v5 /*, v3*/)) == mesh.edge(v3, v5));
  assertx(mesh.most_clw_vertex(v4) == v2);
  assertx(mesh.most_ccw_vertex(v4) == v1);
  assertx(mesh.clw_vertex(v5, v1) == v3);
  assertx(mesh.ccw_vertex(v5, v1) == v4);
  assertx(!mesh.clw_vertex(v3, v1));
  assertx(!mesh.ccw_vertex(v3, v2));
  assertx(mesh.most_clw_edge(v3) == mesh.edge(v3, v1));
  assertx(mesh.most_ccw_edge(v3) == mesh.edge(v3, v2));
  assertx(mesh.clw_edge(v3, mesh.edge(v3, v5)) == mesh.edge(v3, v1));
  assertx(mesh.ccw_edge(v3, mesh.edge(v3, v5)) == mesh.edge(v2, v3));
  Face f531 = mesh.face(v5, v3 /*, v1*/);
  {
    Face f = f531;
    assertx(mesh.num_vertices(f) == 3);
    assertx(mesh.is_triangle(f));
    assertx(mesh.is_boundary(f));
    SHOW("face: 5 3 1");
    Array<Vertex> va;
    mesh.get_vertices(f, va);
    assertx(va.num() == 3);
    for_int(i, 3) SHOW(mesh.vertex_id(va[i]));
    for_int(i, 3) assertx(va[i] == mesh.vertex(f, i));
    Face fo = mesh.opp_face(f, mesh.edge(v1, v5));
    assertx(fo == mesh.face(v1, v4 /*, v5*/));
    SHOW("face vertex: 5 3 1");
    for (Vertex v : mesh.vertices(f)) SHOW(mesh.vertex_id(v));
    SHOW("face face (ccw, by face id): 5 3");
    for (Face ff : mesh.faces(f)) SHOW(mesh.face_id(ff));
    SHOW("face edge: (1, 5), (3, 5), (3, 1)");
    for (Edge e : mesh.edges(f))
      showf("edge (%d, %d)\n", mesh.vertex_id(mesh.vertex1(e)), mesh.vertex_id(mesh.vertex2(e)));
  }
  {
    mesh.ok();
    Vertex v = v3;
    SHOW("vertex vertex (in unspecified order): 1 5 2");
    for (Vertex vv : mesh.vertices(v)) SHOW(mesh.vertex_id(vv));
    SHOW("vertex face (in unspecified order, by face id): 4 3");
    for (Face f : mesh.faces(v)) SHOW(mesh.face_id(f));
    SHOW("vertex edge (in unspecified order): (3, 1), (3, 5), (2, 3)");
    for (Edge e : mesh.edges(v))
      showf("edge (%d, %d)\n", mesh.vertex_id(mesh.vertex1(e)), mesh.vertex_id(mesh.vertex2(e)));
  }
  {
    Edge e = mesh.edge(v1, v5);
    assertx(mesh.vertex1(e) == v1 || mesh.vertex1(e) == v5);
    assertx(mesh.vertex2(e) == v1 || mesh.vertex2(e) == v5);
    {
      Face f = mesh.face1(e);
      assertx(f == f531 || f == mesh.face(v1, v4 /*, v5*/));
    }
    {
      Face f = mesh.face2(e);
      assertx(f == f531 || f == mesh.face(v1, v4 /*, v5*/));
    }
    {
      Vertex v = mesh.side_vertex1(e);
      assertx(v == v3 || v == v4);
    }
    {
      Vertex v = mesh.side_vertex2(e);
      assertx(v == v3 || v == v4);
    }
    assertx(mesh.opp_vertex(e, mesh.face1(e)) == mesh.side_vertex1(e));
    assertx(mesh.opp_vertex(e, mesh.face2(e)) == mesh.side_vertex2(e));
    assertx(mesh.ccw_edge(f531, e) == mesh.edge(v5, v3));
    assertx(mesh.clw_edge(f531, e) == mesh.edge(v1, v3));
    e = mesh.edge(v1, v3);
    assertx(mesh.opp_boundary(e, v1) == mesh.edge(v1, v4));
    assertx(mesh.opp_boundary(e, v3) == mesh.edge(v2, v3));
  }
  {  // All four faces should have exactly two neighbors.
    for (Face f : mesh.faces()) {
      int count = 0;
      for (Face ff : mesh.faces(f)) {
        dummy_use(ff);
        count++;
      }
      assertx(count == 2);
    }
  }
  assertx(mesh.clw_edge(f531, v5) == mesh.edge(v1, v5));
  {
    // (5, 3, 1), (5, 2, 3), (5, 4, 2), (5, 1, 4)
    Edge e = mesh.edge(v1, v3);
    assertx(mesh.vertex1(e) == v3);  // Vertex kept.
    mesh.collapse_edge(e);
    // (5, 2, 3), (5, 4, 2), (5, 3, 4)
  }
  show_mesh(mesh);
  {
    Edge e = mesh.edge(v3, v5);
    mesh.collapse_edge(e);
    // (2, 3, 4)
  }
  show_mesh(mesh);
  {
    for (Edge e : mesh.edges()) assertx(!mesh.nice_edge_collapse(e));
  }
  {
    Array<Vertex> va;
    for_int(i, 3) va.push(mesh.create_vertex());
    Face f = mesh.create_face(va);
    for (Face ff : mesh.faces(f)) {
      dummy_use(ff);
      if (1) assertnever("");
    }
  }
  {
    static_assert(ranges::viewable_range<decltype(std::declval<const Mesh&>().vertices())>);
    static_assert(ranges::viewable_range<decltype(std::declval<const Mesh&>().vertices(Vertex{}))>);
    static_assert(ranges::viewable_range<decltype(std::declval<const Mesh&>().ccw_vertices(Vertex{}))>);
    static_assert(!ranges::view<decltype(std::declval<const Mesh&>().ordered_vertices())>);
  }
  test_tetrahedron();
  test_fan();
  test_polygons();
  test_vertex_operations();
  SHOW("all ok");
}
