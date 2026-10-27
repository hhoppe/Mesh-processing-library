// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#include "libHh/SubMesh.h"

#include <sstream>  // std::istringstream

#include "libHh/Matrix.h"

using namespace hh;

namespace {

float rounded(float v) { return round_fraction_digits(v, 1e4f); }

GMesh mesh_from_string(const string& str) {
  GMesh mesh;
  std::istringstream iss(str);
  mesh.read(iss);
  mesh.ok();
  return mesh;
}

// A regular tetrahedron.
const char* const k_mesh_tetra = R"(Vertex 1  1 1 1
Vertex 2  1 -1 -1
Vertex 3  -1 1 -1
Vertex 4  -1 -1 1
Face 1  1 2 3
Face 2  1 3 4
Face 3  1 4 2
Face 4  2 4 3
)";

// A torus of 4x4 quads (major radius 2, minor radius 1), whose vertices all have valence 4, as the quad masks require.
GMesh torus_mesh() {
  GMesh mesh;
  Matrix<Vertex> vertices(V(4, 4));
  const auto cs = V(V(1.f, 0.f), V(0.f, 1.f), V(-1.f, 0.f), V(0.f, -1.f));  // Cosine and sine of i * TAU / 4.
  for_int(i, 4) for_int(j, 4) {
    vertices[i, j] = mesh.create_vertex();
    const float r = 2.f + cs[j][0];
    mesh.set_point(vertices[i, j], Point(r * cs[i][0], r * cs[i][1], cs[j][1]));
  }
  for_int(i, 4) for_int(j, 4) {
    const int i1 = (i + 1) % 4, j1 = (j + 1) % 4;
    (void)mesh.create_face(V(vertices[i, j], vertices[i1, j], vertices[i1, j1], vertices[i, j1]));
  }
  mesh.ok();
  return mesh;
}

void set_all_variable(GMesh& mesh) {
  for (Vertex v : mesh.vertices()) mesh.flags(v).flag(SubMesh::vflag_variable) = true;
}

// Show one line per vertex of mesh(): the vertices of orig_mesh() by id, then the new vertices in sorted order.
// (The ids of new vertices depend on the iteration order of Mesh::edges(), which differs across standard libraries.)
void show_lines(const SubMesh& smesh, const auto& func_line) {
  const GMesh& mesh = smesh.mesh();
  Array<string> new_lines;
  for (Vertex v : mesh.ordered_vertices()) {
    const string line = func_line(v);
    if (smesh.orig_mesh().id_retrieve_vertex(mesh.vertex_id(v))) {
      showf("v%-3d %s\n", mesh.vertex_id(v), line.c_str());
    } else {
      new_lines.push(line);
    }
  }
  sort(new_lines);
  for (const string& line : new_lines) showf("new  %s\n", line.c_str());
}

void show_points(const SubMesh& smesh) {
  show_lines(smesh, [&](Vertex v) {
    const Point p = transformed(smesh.mesh().point(v), rounded);
    return sform("(%g %g %g)", p[0], p[1], p[2]);
  });
}

// Show the combination of each vertex of mesh() in terms of orig_mesh(), with weights sorted by vertex id.
void show_combinations(const SubMesh& smesh) {
  show_lines(smesh, [&](Vertex v) {
    const Combvh& comb = smesh.combination(v);
    assertx(comb.is_combination());
    Array<std::pair<int, float>> weights;
    for_combination(comb.c, [&](Vertex vo, float val) { weights.push({smesh.orig_mesh().vertex_id(vo), val}); });
    sort(weights);
    string line = sform("h=%g:", rounded(comb.h[3]));
    for (const auto& [id, val] : weights) line += sform("  %d:%g", id, rounded(val));
    return line;
  });
}

// Check that each face of mesh() maps to a face of orig_mesh() and back.
void verify_face_maps(const SubMesh& smesh) {
  const GMesh& mesh = smesh.mesh();
  for (Face f : mesh.faces()) {
    Face of;
    int index;
    smesh.orig_face_index(f, of, index);
    assertx(of == smesh.orig_face(f));
    assertx(smesh.get_face(of, index) == f);
  }
}

}  // namespace

int main() {
  {
    // One subdivision of a tetrahedron: 1-to-4 splits followed by Loop averaging.
    GMesh omesh = mesh_from_string(k_mesh_tetra);
    set_all_variable(omesh);
    SubMesh smesh(omesh);
    smesh.subdivide();
    SHOW(smesh.mesh().num_vertices(), smesh.mesh().num_faces());
    smesh.mesh().ok();
    show_combinations(smesh);
    smesh.update_vertex_positions();
    show_points(smesh);
    verify_face_maps(smesh);
    for (Face of : omesh.faces()) {
      int num = 0;
      for (Face f : smesh.mesh().faces()) num += smesh.orig_face(f) == of;
      assertx(num == 4);
    }
    // The subdivision masks are affine-invariant: translating the original mesh translates the result.
    const Vector offset(1.f, 2.f, 3.f);
    Map<Vertex, Point> old_points;
    for (Vertex v : smesh.mesh().vertices()) old_points.enter(v, smesh.mesh().point(v));
    for (Vertex v : omesh.vertices()) omesh.set_point(v, omesh.point(v) + offset);
    smesh.update_vertex_positions();
    for (Vertex v : smesh.mesh().vertices()) assertx(dist(smesh.mesh().point(v), old_points.get(v) + offset) < 1e-5f);
    // The combination of an original vertex composed with the subdivision gives a combination of original vertices.
    Combvh comb;
    comb.c[smesh.mesh().id_vertex(1)] = .5f;
    comb.c[smesh.mesh().id_vertex(2)] = .5f;
    const Combvh comb2 = smesh.compose_c_mvcvh(comb);
    assertx(comb2.is_combination());
    SHOW(transformed(comb2.evaluate(omesh), rounded));
  }
  {
    // Two subdivisions followed by projection to the limit surface, versus repeated subdivide().
    GMesh omesh = mesh_from_string(k_mesh_tetra);
    set_all_variable(omesh);
    SubMesh smesh(omesh);
    smesh.subdivide_n(2, 1);
    SubMesh smesh2(omesh);
    smesh2.subdivide();
    smesh2.subdivide();
    SHOW(smesh.mesh().num_vertices(), smesh.mesh().num_faces());
    assertx(smesh2.mesh().num_vertices() == smesh.mesh().num_vertices());
    smesh.update_vertex_positions();
    smesh2.update_vertex_positions();
    // The limit positions of the original vertices lie inside the tetrahedron, closer than the subdivided ones.
    for_intL(id, 1, 5) {
      const Point& p = smesh.mesh().point(smesh.mesh().id_vertex(id));
      const Point& p2 = smesh2.mesh().point(smesh2.mesh().id_vertex(id));
      SHOW(id, transformed(p, rounded), transformed(p2, rounded));
      assertx(mag(p) < mag(p2) && mag(p2) < mag(omesh.point(omesh.id_vertex(id))));
    }
    verify_face_maps(smesh);
  }
  {
    // A vertex that is not variable contributes its position through the homogeneous term.
    GMesh omesh = mesh_from_string(k_mesh_tetra);
    set_all_variable(omesh);
    omesh.flags(omesh.id_vertex(1)).flag(SubMesh::vflag_variable) = false;
    SubMesh smesh(omesh);
    smesh.subdivide();
    show_combinations(smesh);
    smesh.update_vertex_positions();
    // Moving the fixed vertex in the original mesh does not move the subdivided mesh.
    Map<Vertex, Point> old_points;
    for (Vertex v : smesh.mesh().vertices()) old_points.enter(v, smesh.mesh().point(v));
    omesh.set_point(omesh.id_vertex(1), Point(5.f, 5.f, 5.f));
    smesh.update_vertex_positions();
    for (Vertex v : smesh.mesh().vertices()) assertx(smesh.mesh().point(v) == old_points.get(v));
  }
  {
    // A quad mesh is subdivided with tensor-product B-spline masks and then triangulated at the face centroids.
    GMesh omesh = torus_mesh();
    set_all_variable(omesh);
    SubMesh smesh(omesh);
    smesh.subdivide_n(1, 0);
    smesh.mesh().ok();
    SHOW(smesh.mesh().num_vertices(), smesh.mesh().num_faces());
    for (Face f : smesh.mesh().faces()) assertx(smesh.mesh().is_triangle(f));
    verify_face_maps(smesh);
    smesh.update_vertex_positions();
    for_intL(id, 1, 5) SHOW(id, transformed(smesh.mesh().point(smesh.mesh().id_vertex(id)), rounded));
    // The result keeps the symmetry of the torus about the origin.
    Vector sum(0.f, 0.f, 0.f);
    for (Vertex v : smesh.mesh().vertices()) sum += smesh.mesh().point(v) - Point(0.f, 0.f, 0.f);
    assertx(mag(sum) < 1e-4f);
  }
}
