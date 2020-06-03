// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Mesh.h"

#include "Array.h"
using namespace hh;

namespace {

void show_mesh(const Mesh& mesh) {
  showf("Mesh {\n  Vertices (%d) {\n", mesh.num_vertices());
  for (Vertex v : mesh.ordered_vertices()) {
    showf("    %d\n", mesh.vertex_id(v));
  }
  showf("  } EndVertices\n  Edges (%d)\n  Faces (%d) {\n", mesh.num_edges(), mesh.num_faces());
  for (Face f : mesh.ordered_faces()) {
    showf("    Face %d {", mesh.face_id(f));
    for (Vertex v : mesh.vertices(f)) {
      showf(" %d", mesh.vertex_id(v));
    }
    showf(" }\n");
  }
  SHOW("  } EndFaces\n} EndMesh");
}

}  // namespace

int main() {
  Mesh mesh;
  Vertex v1 = mesh.create_vertex();
  Vertex v2 = mesh.create_vertex();
  Vertex v3 = mesh.create_vertex();
  Vertex v4 = mesh.create_vertex();
  show_mesh(mesh);
  //
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
  //
  Vertex v5;
  {
    mesh.ok();
    // (1, 2, 3), (2, 1, 4)
    Edge e = mesh.edge(v1, v2);
    for (Face f : mesh.faces(e)) {
      SHOW(mesh.face_id(f));
    }
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
    Array<Vertex> va;
    for (Vertex v : mesh.ccw_vertices(v5)) {
      va.push(v);
    }
    assertx(va.num() == 4);
    SHOW("vertices: 3 1 4 2");
    for_int(i, 4) SHOW(mesh.vertex_id(va[i]));
  }
  {
    Array<Face> fa;
    for (Face f : mesh.ccw_faces(v2)) {
      fa.push(f);
    }
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
    for (Vertex v : mesh.vertices(f)) {
      SHOW(mesh.vertex_id(v));
    }
    SHOW("face face: 4, 2");
    for (Face ff : mesh.faces(f)) {
      SHOW(mesh.face_id(ff));
    }
    SHOW("face edge: (1, 5), (3, 5), (3, 1)");
    for (Edge e : mesh.edges(f)) {
      showf("edge (%d, %d)\n", mesh.vertex_id(mesh.vertex1(e)), mesh.vertex_id(mesh.vertex2(e)));
    }
  }
  {
    mesh.ok();
    Vertex v = v3;
    SHOW("vertex vertex: 5 2 1");
    for (Vertex vv : mesh.vertices(v)) {
      SHOW(mesh.vertex_id(vv));
    }
    SHOW("vertex face: 2, 3");
    for (Face f : mesh.faces(v)) {
      SHOW(mesh.face_id(f));
    }
    SHOW("vertex edge: (3, 5), (2, 3), (3, 1)");
    for (Edge e : mesh.edges(v)) {
      showf("edge (%d, %d)\n", mesh.vertex_id(mesh.vertex1(e)), mesh.vertex_id(mesh.vertex2(e)));
    }
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
  {  // all four faces should have exactly two neighbors
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
    assertx(mesh.vertex1(e) == v3);  // vertex kept
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
    for (Edge e : mesh.edges()) {
      assertx(!mesh.nice_edge_collapse(e));
    }
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
  SHOW("all ok");
}
