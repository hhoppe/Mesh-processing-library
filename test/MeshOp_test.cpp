// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshOp.h"

#include "libHh/GMesh.h"
#include "libHh/MathOp.h"  // TAU
using namespace hh;

namespace {

// A mesh built from points and faces (lists of point indices), keeping the vertices in creation order.
struct TestMesh {
  GMesh mesh;
  Array<Vertex> va;

  TestMesh(CArrayView<Point> points, CArrayView<Array<int>> faces) {
    for (const Point& p : points) {
      const Vertex v = mesh.create_vertex();
      mesh.set_point(v, p);
      va.push(v);
    }
    for (const Array<int>& face : faces) {
      Array<Vertex> fva;
      for (const int i : face) fva.push(va[i]);
      assertx(mesh.create_face(fva));
    }
  }
  [[nodiscard]] Edge edge(int i, int j) const { return mesh.edge(va[i], va[j]); }
};

float rounded(float v) { return round_fraction_digits(v, 1e4f); }

// A regular tetrahedron, with outward-facing (counterclockwise) faces.
TestMesh tetrahedron() {
  return TestMesh(V(Point(1.f, 1.f, 1.f), Point(1.f, -1.f, -1.f), Point(-1.f, 1.f, -1.f), Point(-1.f, -1.f, 1.f)),
                  V(Array<int>{0, 1, 2}, Array<int>{0, 2, 3}, Array<int>{0, 3, 1}, Array<int>{1, 3, 2}));
}

// A unit cube with 6 quadrilateral faces; vertex index i has coordinates (i & 1, (i >> 1) & 1, (i >> 2) & 1).
TestMesh cube() {
  Array<Point> points;
  for_int(i, 8) points.push(Point(float(i & 1), float((i >> 1) & 1), float((i >> 2) & 1)));
  return TestMesh(points, V(Array<int>{0, 2, 3, 1}, Array<int>{4, 5, 7, 6}, Array<int>{0, 4, 6, 2},
                            Array<int>{1, 3, 7, 5}, Array<int>{0, 1, 5, 4}, Array<int>{2, 6, 7, 3}));
}

// A flat n x n grid of unit squares in the plane z = 0, each split into two triangles.
TestMesh grid_patch(int n) {
  Array<Point> points;
  for_int(y, n + 1) for_int(x, n + 1) points.push(Point(float(x), float(y), 0.f));
  Array<Array<int>> faces;
  const auto index = [&](int y, int x) { return y * (n + 1) + x; };
  for_int(y, n) for_int(x, n) {
    faces.push(Array<int>{index(y, x), index(y, x + 1), index(y + 1, x + 1)});
    faces.push(Array<int>{index(y, x), index(y + 1, x + 1), index(y + 1, x)});
  }
  return TestMesh(points, faces);
}

// A triangulated torus, obtained by identifying the opposite sides of an nu x nv grid.
TestMesh torus(int nu, int nv) {
  Array<Point> points;
  for_int(j, nv) for_int(i, nu) {
    const float u = TAU * float(i) / float(nu), v = TAU * float(j) / float(nv);
    points.push(Point((2.f + std::cos(v)) * std::cos(u), (2.f + std::cos(v)) * std::sin(u), std::sin(v)));
  }
  Array<Array<int>> faces;
  const auto index = [&](int i, int j) { return (j % nv) * nu + (i % nu); };
  for_int(j, nv) for_int(i, nu) {
    faces.push(Array<int>{index(i, j), index(i + 1, j), index(i + 1, j + 1)});
    faces.push(Array<int>{index(i, j), index(i + 1, j + 1), index(i, j + 1)});
  }
  return TestMesh(points, faces);
}

void test_topology() {
  {
    const TestMesh tm = tetrahedron();
    SHOW(mesh_genus_string(tm.mesh), mesh_genus(tm.mesh));
  }
  {
    const TestMesh tm = torus(5, 4);
    SHOW(mesh_genus_string(tm.mesh), mesh_genus(tm.mesh));
  }
  {
    const TestMesh tm = grid_patch(3);
    SHOW(mesh_genus_string(tm.mesh));
    const Edge e = tm.edge(0, 1);
    assertx(tm.mesh.is_boundary(e));
    const Queue<Edge> boundary = gather_boundary(tm.mesh, e);
    SHOW(boundary.length());
    const Stat stat = mesh_stat_boundaries(tm.mesh);
    SHOW(stat.inum(), stat.avg());
  }
  {
    // Two connected components: a tetrahedron and a lone triangle, which shares no vertex with it.
    TestMesh tm = tetrahedron();
    Array<Vertex> va;
    for_int(i, 3) {
      va.push(tm.mesh.create_vertex());
      tm.mesh.set_point(va.last(), Point(5.f + float(i), float(i == 2), 0.f));
    }
    const Face f = tm.mesh.create_face(va);
    SHOW(mesh_genus_string(tm.mesh));
    const Array<Set<Face>> components = gather_components(tm.mesh);
    SHOW(components.num(), components[0].num(), components[1].num());  // In order of increasing size.
    assertx(gather_component(tm.mesh, f).num() == 1);
    assertx(gather_component_v(tm.mesh, f).num() == 1);
    const Stat stat = mesh_stat_components(tm.mesh);
    SHOW(stat.inum(), stat.min(), stat.max());
  }
  {
    // Removing a face of a tetrahedron leaves a triangular hole, which mesh_remove_boundary() fills again.
    TestMesh tm = tetrahedron();
    tm.mesh.destroy_face(tm.mesh.face(tm.va[1], tm.va[3]));
    SHOW(mesh_genus_string(tm.mesh));
    const Edge e = tm.edge(1, 3);
    assertx(tm.mesh.is_boundary(e));
    const Set<Face> new_faces = mesh_remove_boundary(tm.mesh, e);
    SHOW(new_faces.num(), mesh_genus_string(tm.mesh));
  }
}

void test_geometry() {
  {
    const TestMesh tm = tetrahedron();
    const Edge e = tm.edge(0, 1);
    // For a regular tetrahedron, the exterior dihedral angle has cosine -1/3.
    SHOW(rounded(edge_dihedral_angle_cos(tm.mesh, e)), rounded(edge_signed_dihedral_angle(tm.mesh, e)));
    SHOW(rounded(vertex_solid_angle(tm.mesh, tm.va[0])));  // acos(23 / 27) == 0.5513 steradians.
  }
  {
    TestMesh tm = cube();
    SHOW(rounded(edge_dihedral_angle_cos(tm.mesh, tm.edge(0, 1))));  // A right angle.
    Array<Face> faces;
    for (Face f : tm.mesh.faces()) faces.push(f);
    for (Face f : faces) assertx(triangulate_face(tm.mesh, f));
    SHOW(tm.mesh.num_faces(), tm.mesh.num_edges(), mesh_genus_string(tm.mesh));
    for (Edge e : tm.mesh.edges()) {
      const float cosine = rounded(edge_dihedral_angle_cos(tm.mesh, e));
      assertx(cosine == 0.f || cosine == 1.f);  // Either a cube edge or a diagonal within a face.
    }
    SHOW(rounded(vertex_solid_angle(tm.mesh, tm.va[7])));  // A cube corner subtends TAU / 4 steradians.
    const Vnors vnors(tm.mesh, tm.va[7], Vnors::EType::angle);
    assertx(vnors.is_unique());
    SHOW(vnors.unique_nor());  // Angle-weighting is symmetric at the corner: (1, 1, 1) / sqrt(3).
  }
  {
    const TestMesh tm = grid_patch(3);
    const Vertex v = tm.va[5];  // The interior vertex at (1, 1).
    for (const auto nortype : {Vnors::EType::angle, Vnors::EType::sum, Vnors::EType::area, Vnors::EType::sloan}) {
      const Vnors vnors(tm.mesh, v, nortype);
      assertx(vnors.is_unique() && dist(vnors.unique_nor(), Vector(0.f, 0.f, 1.f)) < 1e-6f);
    }
    // Project a point above the patch onto it, starting from a nearby face.
    Face pf = *tm.mesh.faces(v).begin();
    Bary bary;
    Point clp;
    const float d2 = project_point_neighborhood(tm.mesh, Point(1.3f, 1.6f, .7f), pf, bary, clp, false);
    SHOW(rounded(d2), clp);
    // Collapsing an interior edge of a flat patch changes no volume and has no quadric error.
    const Edge e = tm.edge(5, 10);
    SHOW(rounded(collapse_edge_volume_criterion(tm.mesh, e)), rounded(collapse_edge_qem_criterion(tm.mesh, e)));
  }
}

void test_retriangulate() {
  for_int(icriterion, 2) {
    // A thin rhombus split along its long diagonal; both criteria prefer the short diagonal.
    TestMesh tm(V(Point(-2.f, 0.f, 0.f), Point(0.f, -1.f, 0.f), Point(2.f, 0.f, 0.f), Point(0.f, 1.f, 0.f)),
                V(Array<int>{0, 1, 2}, Array<int>{0, 2, 3}));
    const EDGEF criterion = icriterion == 0 ? circum_radius_swap_criterion : diagonal_distance_swap_criterion;
    const int num_swaps = retriangulate_all(tm.mesh, .9f, criterion);
    SHOW(icriterion, num_swaps, !!tm.mesh.query_edge(tm.va[1], tm.va[3]), !!tm.mesh.query_edge(tm.va[0], tm.va[2]));
    assertx(retriangulate_all(tm.mesh, .9f, criterion) == 0);  // Now stable.
  }
}

void test_split_valence() {
  // A wheel: a center vertex surrounded by 12 triangles.
  const int n = 12;
  Array<Point> points{Point(0.f, 0.f, 0.f)};
  for_int(i, n) {
    const float a = TAU * float(i) / float(n);
    points.push(Point(std::cos(a), std::sin(a), 0.f));
  }
  Array<Array<int>> faces;
  for_int(i, n) faces.push(Array<int>{0, 1 + i, 1 + (i + 1) % n});
  TestMesh tm(points, faces);
  SHOW(tm.mesh.degree(tm.va[0]));
  split_valence(tm.mesh, 8);  // (The maximum valence must exceed 6.)
  int max_degree = 0;
  for (Vertex v : tm.mesh.vertices()) max_degree = max(max_degree, tm.mesh.degree(v));
  SHOW(tm.mesh.num_vertices(), tm.mesh.num_faces(), max_degree);
  assertx(max_degree < 8);
}

}  // namespace

int main() {
  test_topology();
  test_geometry();
  test_retriangulate();
  test_split_valence();
}
