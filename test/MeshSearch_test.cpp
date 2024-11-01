// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshSearch.h"

#include "libHh/Matrix.h"
#include "libHh/MeshOp.h"
#include "libHh/Timer.h"
using namespace hh;

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
    const MeshSearch mesh_search(mesh, {false});
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
      matv[y][x] = mesh.create_vertex();
      mesh.set_point(matv[y][x], Point(x / (n - 1.f), y / (n - 1.f), 0.f));
    }
    for_int(y, n - 1) for_int(x, n - 1) {
      mesh.create_face(matv[y][x], matv[y + 1][x], matv[y + 1][x + 1]);
      mesh.create_face(matv[y][x], matv[y + 1][x + 1], matv[y][x + 1]);
    }
    SHOW(mesh_genus_string(mesh));
    const MeshSearch mesh_search(mesh, {true});
    Face hint_f = nullptr;
    for_int(i, 8) {
      Point p;
      for_int(c, 3) p[c] = Random::G.unif();
      p[2] *= 1e-7f;  // was 1e-4f
      SHOW(p);
      const auto [f, bary, clp, d2] = mesh_search.search(p, hint_f);
      hint_f = f;
      SHOW(mesh.face_id(f), bary, clp, d2);
    }
  }
}
