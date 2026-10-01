// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Contour.h"

#include <map>

#include "libHh/A3dStream.h"
#include "libHh/FileIO.h"
#include "libHh/MathOp.h"
using namespace hh;

namespace {

// *** Contour2d

void test2d() {
  struct feval2d {
    float operator()(const Vec2<float>& p) const {
      float f = float(dist(p, V(.4f, .4f)) - .25);
      if (dist2(p, V(.3f, .6f)) < square(.3)) f = k_Contour_undefined;
      return f;
    }
  };
  const auto func_polylinetoa3d = [](CArrayView<Vec2<float>> poly, A3dElem& el) {
    el.init(A3dElem::EType::polyline);
    for_int(i, poly.num()) {
      el.push(A3dVertex(Point(0.f, poly[i][0], poly[i][1]), Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red())));
    }
  };
  const int gn = 20;
  WFile fcontour("Contour_test.2D");
  WSA3dStream wcontour(fcontour());
  WFile fborder("Contour_test.2Dborder");
  WSA3dStream wborder(fborder());
  A3dElem el;
  const auto func_contour = [&](CArrayView<Vec2<float>> poly) {
    func_polylinetoa3d(poly, el);
    wcontour.write(el);
  };
  const auto func_border = [&](CArrayView<Vec2<float>> poly) {
    func_polylinetoa3d(poly, el);
    wborder.write(el);
  };
  Contour2d contour(gn, feval2d(), func_contour, func_border);
  contour.march_near(V(.64f, .39f));
  // contour.march_from(V(.64f, .39f));
}

// *** Contour3d

struct feval3d {
  float operator()(const Vec3<float>& p) const {
    // Compute at double-precision to avoid numerical differences between different CONFIG.
    const Vec3<double> pd = convert<double>(p);
    float f = float((dist(pd, V(.2, .3, .3)) - .15) * (dist(pd, V(.6, .65, .7)) - .35));
    if (dist2(pd, V(.53, .53, .53)) < square(.15)) f = k_Contour_undefined;
    return f;
  }
};

void test3d() {
  const int gn = 10;
  WFile fcontour("Contour_test.3D");
  WSA3dStream wcontour(fcontour());
  WFile fborder("Contour_test.3Dborder");
  WSA3dStream wborder(fborder());
  const auto func_polygontoa3d = [](CArrayView<Vec3<float>> poly, A3dElem& el) {
    el.init(A3dElem::EType::polygon);
    for_int(i, poly.num()) el.push(A3dVertex(poly[i], Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red())));
  };
  A3dElem el;
  const auto func_contour = [&](CArrayView<Vec3<float>> poly) {
    func_polygontoa3d(poly, el);
    wcontour.write(el);
  };
  const auto func_border = [&](CArrayView<Vec3<float>> poly) {
    func_polygontoa3d(poly, el);
    wborder.write(el);
  };
  Contour3d contour(gn, func_contour, feval3d(), func_border);
  const int nc1 = contour.march_from(Point(.35f, .3f, .3f));
  const int nc2 = contour.march_from(Point(.25f, .65f, .7f));
  const int nc3 = contour.march_from(Point(.95f, .65f, .7f));
  const int nc4 = contour.march_from(Point(.8f, .2f, .1f));
  const int nc5 = contour.march_from(Point(.8f, .2f, .1f));
  SHOW(nc1, nc2, nc3, nc4, nc5);
}

void testmesh() {
  GMesh mesh;
  {
    Contour3dMesh<feval3d> contour(10, &mesh);
    if (0) contour.big_mesh_faces();
    contour.set_vertex_tolerance(1e-4f);
    const int nc1 = contour.march_from(Point(.35f, .3f, .3f));
    const int nc2 = contour.march_from(Point(.25f, .65f, .7f));
    SHOW(nc1, nc2);
  }
  for (Vertex v : mesh.vertices()) {
    Point p = mesh.point(v);
    round_elements(p, 1e4f);
    mesh.set_point(v, p);
  }
  WFile fmesh("Contour_test.m");
  mesh.write(fmesh());
}

struct fmonkey {
  float operator()(const Point& p) const {
    // Monkey saddle, z = x^3 - 3 y^2 x.
    const float s = 4.f;
    const Point pp = (p * 2.f - 1.f) * s;
    const float x = pp[0], y = pp[1], z = pp[2];
    const float f = z - pow(x, 3.f) + 3.f * y * y * x;
    return f;
  }
};

void do_monkey() {
  GMesh mesh;
  {
    Contour3dMesh<fmonkey> contour(50, &mesh);
    contour.set_vertex_tolerance(1e-5f);
    contour.march_near(Point(.5f, .5f, .5f));
  }
  mesh.write(std::cout);
}

void do_densemonkey() {
  GMesh mesh;
  {
    Contour3dMesh<fmonkey> contour(500, &mesh);
    contour.set_vertex_tolerance(1e-5f);
    contour.march_near(Point(.5f, .5f, .5f));
  }
  mesh.write(std::cout);
}

void do_sphere() {
  static constexpr float k_radius = .4f;
  const auto func_sphere = [](const Vec3<float>& p) {
    const float r = dist(p, V(.5f, .5f, .5f));
    if (0) {
      return square(r) - square(k_radius);
    } else if (0) {
      return r - k_radius;
    } else {
      return pow(r, 6.f) - pow(k_radius, 6.f);
    }
  };
  GMesh mesh;
  {
    Contour3dMesh contour(128, &mesh, func_sphere);
    contour.set_vertex_tolerance(1 ? 1e-5f : 0);
    contour.march_near(Point(.5f + k_radius, .5f, .5f));
  }
  mesh.write(std::cout);
}

// *** Self-checking tests on shapes with known geometry.

constexpr Vec3<double> k_sphere_center{.52, .47, .5};
constexpr double k_sphere_radius = .31;  // Chosen so that no grid vertex lies exactly on the sphere.

// Signed distance to the sphere, computed at double-precision for portability.
float sphere_distance(const Vec3<float>& p) {
  return float(dist(convert<double>(p), k_sphere_center) - k_sphere_radius);
}

// Return the maximum deviation of the points from the sphere.
float max_sphere_error(CArrayView<Vec3<float>> points) {
  float max_error = 0.f;
  for (const Vec3<float>& p : points) max_error = max(max_error, abs(sphere_distance(p)));
  return max_error;
}

// Contour the sphere into a mesh and verify that the mesh is a closed, consistently oriented genus-0 surface.
void test_mesh_sphere(int gn, float vertex_tol, bool big_mesh_faces) {
  GMesh mesh;
  int nc;
  {
    Contour3dMesh contour(gn, &mesh, sphere_distance);
    contour.set_ostream(nullptr);
    contour.set_vertex_tolerance(vertex_tol);
    if (big_mesh_faces) contour.big_mesh_faces();
    nc = contour.march_near(convert<float>(k_sphere_center + V(k_sphere_radius, 0., 0.)));
  }
  const int nv = mesh.num_vertices(), ne = mesh.num_edges(), nf = mesh.num_faces();
  SHOW(gn, vertex_tol > 0.f, big_mesh_faces, nc, nv, ne, nf, nv - ne + nf);
  assertx(nv - ne + nf == 2);  // The Euler characteristic of a sphere.
  assertx(mesh.is_nice());
  for (Edge e : mesh.edges()) assertx(!mesh.is_boundary(e));
  // The vertices lie on grid edges, except for those introduced by center_split_face() in faces with >= 6 sides.
  Array<Vec3<float>> points;
  int ncenter = 0;
  for (Vertex v : mesh.vertices()) {
    const Vec3<float>& p = mesh.point(v);
    int ngrid = 0;
    for_int(c, 3) ngrid += abs(p[c] * gn - std::round(p[c] * gn)) < 1e-4f;
    assertx(ngrid <= 2);  // Never a vertex of the grid itself.
    if (ngrid == 2) {
      points.push(p);
    } else {
      ncenter++;
      assertx(!big_mesh_faces);
    }
  }
  SHOW(ncenter);
  const float max_error = max_sphere_error(points);
  assertx(max_error < (vertex_tol ? 1e-4f : .02f));
  // Each face is oriented with its normal pointing outward, and the total area approximates that of the sphere.
  double area = 0.;
  for (Face f : mesh.faces()) {
    if (!big_mesh_faces) assertx(mesh.is_triangle(f));
    Array<Vertex> va;
    mesh.get_vertices(f, va);
    Vec3<float> centroid{};
    for (Vertex v : va) centroid += mesh.point(v) / float(va.num());
    for_intL(i, 1, va.num() - 1) {
      const Vec3<float> normal = cross(mesh.point(va[0]), mesh.point(va[i]), mesh.point(va[i + 1]));
      area += mag(normal) * .5;
      assertx(dot(convert<double>(normal), convert<double>(centroid) - k_sphere_center) > 0.);
    }
  }
  // The fan triangulation of a nonplanar big face underestimates its area.
  const double expected_area = 2. * D_TAU * square(k_sphere_radius);
  assertx(abs(area / expected_area - 1.) < (big_mesh_faces ? .06 : .03));
}

// Contour the sphere into a stream of triangles and verify their geometry and orientation.
void test_stream_sphere(int gn, float vertex_tol) {
  int ntriangles = 0;
  double area = 0.;
  float max_error = 0.f;
  const auto func_contour = [&](CArrayView<Vec3<float>> poly) {
    assertx(poly.num() == 3);
    ntriangles++;
    max_error = max(max_error, max_sphere_error(poly));
    const Vec3<float> normal = cross(poly[0], poly[1], poly[2]);
    area += mag(normal) * .5;
    const Vec3<float> centroid = (poly[0] + poly[1] + poly[2]) / 3.f;
    // The normal points toward positive values, i.e., outward; allow for nearly degenerate triangles.
    if (mag(normal) > 1e-6f) assertx(dot(convert<double>(normal), convert<double>(centroid) - k_sphere_center) > 0.);
  };
  int nc;
  {
    Contour3d contour(gn, func_contour, sphere_distance);
    contour.set_ostream(nullptr);
    contour.set_vertex_tolerance(vertex_tol);
    nc = contour.march_from(convert<float>(k_sphere_center + V(0., k_sphere_radius, 0.)));
  }
  SHOW(gn, vertex_tol > 0.f, nc, ntriangles);
  assertx(max_error < (vertex_tol ? 1e-4f : .02f));
  const double expected_area = 2. * D_TAU * square(k_sphere_radius);
  assertx(abs(area / expected_area - 1.) < .03);
}

// The return values of march_from(), and a partial surface bounded by an undefined region.
void test_march_and_border() {
  {
    int ntriangles = 0;
    const auto func_contour = [&](CArrayView<Vec3<float>>) { ntriangles++; };
    Contour3d contour(10, func_contour, sphere_distance);
    contour.set_ostream(nullptr);
    const int nc1 = contour.march_from(V(.01f, .01f, .01f));  // A cube far from the surface.
    const int nc2 = contour.march_from(V(.01f, .01f, .01f));  // Revisiting the same cube.
    const int nc3 = contour.march_from(convert<float>(k_sphere_center + V(0., 0., k_sphere_radius)));
    const int nc4 = contour.march_from(convert<float>(k_sphere_center - V(0., 0., k_sphere_radius)));
    SHOW(nc1, nc2, nc3 > 1, nc4);
    assertx(nc1 == 1 && nc2 == 0 && nc3 > 1 && nc4 == 0 && ntriangles > 0);
  }
  {
    // A sphere whose part with x < .5 is undefined yields a mesh with the topology of a disk.
    const auto func_eval = [](const Vec3<float>& p) { return p[0] < .5f ? k_Contour_undefined : sphere_distance(p); };
    GMesh mesh;
    {
      Contour3dMesh contour(12, &mesh, func_eval);
      contour.set_ostream(nullptr);
      contour.march_near(convert<float>(k_sphere_center + V(k_sphere_radius, 0., 0.)));
    }
    int nboundary = 0;
    for (Edge e : mesh.edges()) nboundary += mesh.is_boundary(e);
    const int euler = mesh.num_vertices() - mesh.num_edges() + mesh.num_faces();
    SHOW(nboundary > 0, euler);
    assertx(mesh.is_nice() && nboundary > 0 && euler == 1);
  }
  {
    // The border polygons are faces of grid cubes.
    const int gn = 8;
    const auto func_eval = [](const Vec3<float>& p) { return p[0] < .5f ? k_Contour_undefined : sphere_distance(p); };
    int nborder = 0;
    const auto func_border = [&](CArrayView<Vec3<float>> poly) {
      assertx(poly.num() == 4);
      nborder++;
      for (const Vec3<float>& p : poly) for_int(c, 3) assertx(abs(p[c] * gn - std::round(p[c] * gn)) < 1e-5f);
      // The four corners span a unit square in exactly two of the axes.
      const Vec3<float> extent =
          max(max(poly[0], poly[1]), max(poly[2], poly[3])) - min(min(poly[0], poly[1]), min(poly[2], poly[3]));
      int nspan = 0;
      for_int(c, 3) nspan += extent[c] > .5f / gn;
      assertx(nspan == 2);
    };
    const auto func_contour = [](CArrayView<Vec3<float>>) {};
    Contour3d contour(gn, func_contour, func_eval, func_border);
    contour.set_ostream(nullptr);
    contour.march_near(convert<float>(k_sphere_center + V(k_sphere_radius, 0., 0.)));
    assertx(nborder > 0);
  }
}

// Contour a circle in 2D and verify that the polyline segments form a closed, consistently oriented curve.
void test_circle_2d(int gn, float vertex_tol) {
  const Vec2<double> center{.5, .45};
  const double radius = .3;
  const auto func_eval = [&](const Vec2<float>& p) { return float(dist(convert<double>(p), center) - radius); };
  Array<Vec2<Vec2<float>>> segments;
  const auto func_contour = [&](CArrayView<Vec2<float>> poly) {
    assertx(poly.num() == 2);
    segments.push(V(poly[0], poly[1]));
  };
  {
    Contour2d contour(gn, func_eval, func_contour);
    contour.set_ostream(nullptr);
    contour.set_vertex_tolerance(vertex_tol);
    contour.march_near(convert<float>(center + V(radius, 0.)));
  }
  std::map<Vec2<float>, int> nstarts, nends;
  double length = 0.;
  float max_error = 0.f;
  int npositive = 0;
  for (const auto& [p0, p1] : segments) {
    nstarts[p0]++;
    nends[p1]++;
    length += dist(p0, p1);
    for (const Vec2<float>& p : {p0, p1}) max_error = max(max_error, abs(func_eval(p)));
    npositive += cross(convert<double>(p0) - center, convert<double>(p1) - center) > 0.;
  }
  SHOW(gn, vertex_tol > 0.f, segments.num(), npositive);
  // Every vertex starts exactly one segment and ends exactly one segment.
  assertx(nstarts.size() == size_t(segments.num()) && nends.size() == size_t(segments.num()));
  for (const auto& [p, n] : nstarts) assertx(n == 1 && nends.contains(p) && nends[p] == 1);
  assertx(npositive == 0 || npositive == segments.num());  // A consistent orientation.
  assertx(max_error < (vertex_tol ? 1e-4f : .01f));
  assertx(abs(length / (D_TAU * radius) - 1.) < .01);
}

}  // namespace

int main() {
  if (0) {
  } else if (getenv_bool("PARTIAL_SPHERE")) {
    const auto func_eval = [](const Point& p) {
      return p[0] < .3f ? k_Contour_undefined : dist(p, Point(.5f, .5f, .5f)) - .4f;
    };
    GMesh mesh;
    {
      Contour3dMesh contour(50, &mesh, func_eval);  // Or 6.
      contour.march_near(Point(.9f, .5f, .5f));
    }
    mesh.write(std::cout);
  } else if (getenv_bool("SPHERE")) {
    do_sphere();
  } else if (getenv_bool("MONKEY")) {
    do_monkey();
  } else if (getenv_bool("DENSE_MONKEY")) {
    do_densemonkey();
  } else {
    testmesh();
    test2d();
    test3d();
    test_mesh_sphere(12, 0.f, false);
    test_mesh_sphere(12, 1e-5f, false);
    test_mesh_sphere(9, 0.f, true);
    test_stream_sphere(12, 0.f);
    test_stream_sphere(12, 1e-5f);
    test_march_and_border();
    test_circle_2d(30, 0.f);
    test_circle_2d(30, 1e-5f);
  }
}
