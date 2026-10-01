// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshSearch.h"

#include "libHh/Bbox.h"
#include "libHh/Matrix.h"
#include "libHh/MeshOp.h"
#include "libHh/Random.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

// Linear-scan reference for the squared distance from p to the mesh.
float brute_force_dist2(const GMesh& mesh, const Point& p) {
  float min_d2 = BIGFLOAT;
  for (Face f : mesh.faces()) min_d2 = min(min_d2, project_point_triangle(p, mesh.triangle_points(f)).d2);
  return min_d2;
}

// A nonplanar n x n grid over the unit square, with a bump in z.
GMesh bumpy_grid(int n) {
  GMesh mesh;
  Matrix<Vertex> matv(n, n);
  for_int(y, n) for_int(x, n) {
    matv[y, x] = mesh.create_vertex();
    const float fx = float(x) / float(n - 1), fy = float(y) / float(n - 1);
    mesh.set_point(matv[y, x], Point(fx, fy, .3f * std::sin(fx * 3.f) * std::cos(fy * 2.f)));
  }
  for_int(y, n - 1) for_int(x, n - 1) {
    mesh.create_face(matv[y, x], matv[y + 1, x + 1], matv[y + 1, x]);
    mesh.create_face(matv[y, x], matv[y, x + 1], matv[y + 1, x + 1]);
  }
  return mesh;
}

void test_brute_force() {
  const GMesh mesh = bumpy_grid(9);
  SHOW(mesh_genus_string(mesh));
  const MeshSearch mesh_search(mesh, {});
  // The query points must lie within the bounding box of the mesh, which MeshSearch maps to the unit cube.
  Bbox<float, 3> bbox;
  for (Vertex v : mesh.vertices()) bbox.union_with(mesh.point(v));
  Random random(1);
  for_int(i, 200) {
    Point p;
    for_int(c, 3) p[c] = bbox[0][c] + random.unif() * (bbox[1][c] - bbox[0][c]);
    const auto [f, bary, clp, d2] = mesh_search.search(p, nullptr);
    assertx(f && bary.is_convex() && abs(bary[0] + bary[1] + bary[2] - 1.f) < 1e-5f);
    assertx(dist(interp(mesh.triangle_points(f), bary), clp) < 1e-5f && abs(dist2(p, clp) - d2) < 1e-5f);
    assertx(abs(d2 - brute_force_dist2(mesh, p)) < 1e-6f);
  }
  // With allow_local_project, a sequence of nearby points near the surface is found by walking from the hint face.
  const MeshSearch mesh_search2(mesh, {.allow_local_project = true});
  Face hint_f = nullptr;
  int num_same_face = 0;
  for_int(i, 100) {
    const float t = float(i) / 99.f;
    const float fx = .05f + .9f * t, fy = .5f + .4f * std::sin(t * 5.f);
    const Point p(fx, fy, .3f * std::sin(fx * 3.f) * std::cos(fy * 2.f) + 1e-4f);
    const auto [f, bary, clp, d2] = mesh_search2.search(p, hint_f);
    assertx(f && abs(d2 - brute_force_dist2(mesh, p)) < 1e-6f);
    num_same_face += f == hint_f;
    hint_f = f;
  }
  assertx(num_same_face > 10);
}

// An octahedron with vertices on the unit sphere.
GMesh octahedron() {
  GMesh mesh;
  const Vec<Point, 6> points{Point(1.f, 0.f, 0.f),  Point(-1.f, 0.f, 0.f), Point(0.f, 1.f, 0.f),
                             Point(0.f, -1.f, 0.f), Point(0.f, 0.f, 1.f),  Point(0.f, 0.f, -1.f)};
  Array<Vertex> va;
  for (const Point& p : points) {
    va.push(mesh.create_vertex());
    mesh.set_point(va.last(), p);
  }
  for (const int iz : {4, 5}) {
    const Vec4<int> ring = iz == 4 ? V(0, 2, 1, 3) : V(0, 3, 1, 2);
    for_int(i, 4) mesh.create_face(va[ring[i]], va[ring[(i + 1) % 4]], va[iz]);
  }
  mesh.ok();
  return mesh;
}

void test_search_on_sphere() {
  const GMesh mesh = octahedron();
  assertx(mesh.is_nice());
  SHOW(mesh_genus_string(mesh));
  const MeshSearch mesh_search(mesh, {});
  Random random(2);
  Face hint_f = nullptr;
  for_int(i, 100) {
    Point p;
    for_int(c, 3) p[c] = random.unif() * 2.f - 1.f;
    if (!p.normalize()) continue;
    const auto [f, bary] = mesh_search.search_on_sphere(p, hint_f);
    hint_f = f;
    assertx(f && min(bary) >= 0.f && abs(bary[0] + bary[1] + bary[2] - 1.f) < 1e-5f);
    // The face is the octant containing p, and the barycentric coordinates are those of the gnomonic projection.
    for (Vertex v : mesh.vertices(f)) assertx(dot(mesh.point(v), p) >= 0.f);
    assertx(dist(normalized(interp(mesh.triangle_points(f), bary)), p) < 1e-5f);
  }
}

}  // namespace

int main() {
  my_setenv("SHOW_STATS", "-2");  // Due to variations in Spatial construction.
  my_setenv("SHOW_TIMES", "-1");
  {
    GMesh mesh;
    Vertex v1 = mesh.create_vertex();
    mesh.set_point(v1, Point(0.f, 0.f, 0.f));
    Vertex v2 = mesh.create_vertex();
    mesh.set_point(v2, Point(10.f, 0.f, 0.f));
    Vertex v3 = mesh.create_vertex();
    mesh.set_point(v3, Point(10.f, 10.f, 0.f));
    Vertex v4 = mesh.create_vertex();
    mesh.set_point(v4, Point(0.f, 10.f, 0.f));
    Face f1 = mesh.create_face(v1, v2, v3);
    SHOW(mesh.face_id(f1));
    Face f2 = mesh.create_face(v1, v3, v4);
    SHOW(mesh.face_id(f2));
    const MeshSearch mesh_search(mesh, {});
    Face hint_f = nullptr;
    {
      const auto [f, bary, clp, d2] = mesh_search.search(Point(5.f, 2.f, 0.f), hint_f);
      SHOW(mesh.face_id(f), bary, clp);
      assertx(d2 < 1e-12f);
    }
    {
      const auto [f, bary, clp, d2] = mesh_search.search(Point(5.f, 9.f, 0.1f), hint_f);
      SHOW(mesh.face_id(f), bary, clp, d2);
    }
  }
  {
    GMesh mesh;
    const int n = 5;
    Matrix<Vertex> matv(n, n);
    for_int(y, n) for_int(x, n) {
      matv[y, x] = mesh.create_vertex();
      mesh.set_point(matv[y, x], Point(x / (n - 1.f), y / (n - 1.f), 0.f));
    }
    for_int(y, n - 1) for_int(x, n - 1) {
      mesh.create_face(matv[y, x], matv[y + 1, x], matv[y + 1, x + 1]);
      mesh.create_face(matv[y, x], matv[y + 1, x + 1], matv[y, x + 1]);
    }
    SHOW(mesh_genus_string(mesh));
    const MeshSearch mesh_search(mesh, {.allow_local_project = true});
    Face hint_f = nullptr;
    for_int(i, 8) {
      Point p;
      for_int(c, 3) p[c] = Random::G.unif();
      p[2] *= 1e-7f;  // Was 1e-4f.
      SHOW(p);
      const auto [f, bary, clp, d2] = mesh_search.search(p, hint_f);
      hint_f = f;
      SHOW(mesh.face_id(f), bary, clp, d2);
    }
  }
  test_brute_force();
  test_search_on_sphere();
}
