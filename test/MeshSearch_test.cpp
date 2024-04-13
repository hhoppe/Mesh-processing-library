// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/MeshSearch.h"

#include "libHh/Matrix.h"
#include "libHh/MeshOp.h"
#include "libHh/Timer.h"
using namespace hh;

int main() {
  my_setenv("SHOW_STATS", "-2");
  Timer::set_show_times(-1);
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
    bool allow_local_project = false;
    MeshSearch msearch(&mesh, allow_local_project);
    Face hintf = nullptr;
    {
      Bary bary;
      Point clp;
      float d2;
      Face f = msearch.search(Point(5.f, 2.f, 0.f), hintf, bary, clp, d2);
      hintf = f;
      SHOW(mesh.face_id(f), bary, clp);
      assertx(d2 < 1e-12f);
    }
    {
      Bary bary;
      Point clp;
      float d2;
      Face f = msearch.search(Point(5.f, 9.f, 0.1f), hintf, bary, clp, d2);
      hintf = f;
      SHOW(mesh.face_id(f), bary, clp, d2);
    }
    dummy_use(hintf);
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
    bool allow_local_project = true;
    MeshSearch msearch(&mesh, allow_local_project);
    Face hintf = nullptr;
    for_int(i, 8) {
      Point p;
      for_int(c, 3) p[c] = Random::G.unif();
      p[2] *= 1e-7f;  // was 1e-4f
      SHOW(p);
      Bary bary;
      Point clp;
      float d2;
      Face f = msearch.search(p, hintf, bary, clp, d2);
      hintf = f;
      SHOW(mesh.face_id(f), bary, clp, d2);
    }
  }
}
