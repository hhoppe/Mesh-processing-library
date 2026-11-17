// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/Ply.h"

#include <cmath>    // std::isnan()
#include <sstream>  // std::istringstream, std::ostringstream

#include "libHh/GMesh.h"
#include "libHh/PMesh.h"
using namespace hh;

namespace {

// Returns true if the two arrays have the same values, where NaN equals NaN.
bool same_floats(CArrayView<float> a1, CArrayView<float> a2) {
  if (a1.num() != a2.num()) return false;
  for_int(i, a1.num()) {
    if (std::isnan(a1[i]) ? !std::isnan(a2[i]) : a1[i] != a2[i]) return false;
  }
  return true;
}

bool same_meshes(const PlyMesh& m1, const PlyMesh& m2) {
  if (m1.wedge_keys.num() != m2.wedge_keys.num()) return false;
  for_int(i, m1.wedge_keys.num()) {
    const PlyMesh::WedgeKey &k1 = m1.wedge_keys[i], &k2 = m2.wedge_keys[i];
    if (k1.name != k2.name || k1.dim != k2.dim || !same_floats(k1.values, k2.values)) return false;
  }
  return m1.points == m2.points && m1.normals == m2.normals && m1.colors == m2.colors && m1.uvs == m2.uvs &&
         m1.face_vertices == m2.face_vertices && m1.face_start == m2.face_start && m1.corner_uvs == m2.corner_uvs &&
         m1.wedge_vertex == m2.wedge_vertex && m1.face_matids == m2.face_matids && m1.vertex_ids == m2.vertex_ids &&
         m1.face_ids == m2.face_ids && m1.materials == m2.materials && m1.comments == m2.comments;
}

void show_summary(const PlyMesh& mesh) {
  showf(
      "points=%d normals=%d colors=%d uvs=%d faces=%d corner_uvs=%d wedge_vertex=%d face_matids=%d materials=%d"
      " wedge_keys=%d comments=%d mesh_vertices=%d\n",
      mesh.points.num(), mesh.normals.num(), mesh.colors.num(), mesh.uvs.num(), mesh.num_faces(),
      mesh.corner_uvs.num(), mesh.wedge_vertex.num(), mesh.face_matids.num(), mesh.materials.num(),
      mesh.wedge_keys.num(), mesh.comments.num(), mesh.num_mesh_vertices());
}

PlyMesh round_trip(const PlyMesh& mesh, bool binary) {
  std::ostringstream oss;
  write_ply(oss, mesh, binary);
  std::istringstream iss(oss.str());
  PlyMesh mesh2 = read_ply(iss);
  assertx(iss.peek() == std::char_traits<char>::eof());  // The reader consumed exactly the PLY data.
  return mesh2;
}

void test_round_trip() {
  // Two triangles and a quad over 5 wedges of 4 vertices (vertex 1 has 2 wedges), with all the optional data.
  PlyMesh mesh;
  mesh.points = {Point(0.f, 0.f, 0.f), Point(1.f, 0.f, 0.f), Point(1.f, 0.f, 0.f), Point(0.f, 1.f, 1e-7f),
                 Point(0.1f, 0.2f, 0.3f)};
  mesh.normals = {Vector(0.f, 0.f, 1.f), Vector(0.f, 0.f, 1.f), Vector(0.f, 1.f, 0.f), Vector(0.6f, 0.8f, 0.f),
                  Vector(0.f, 0.f, -1.f)};
  mesh.colors = {Pixel(255, 0, 0), Pixel(0, 255, 0), Pixel(0, 0, 255), Pixel(1, 2, 3), Pixel(250, 128, 7)};
  mesh.uvs = {Uv(0.f, 0.f), Uv(1.f, 0.f), Uv(0.5f, 0.25f), Uv(1.f / 3.f, 2.f / 3.f), Uv(0.f, 1.f)};
  mesh.face_vertices = {0, 1, 3, 2, 4, 3, 0, 3, 4, 1};
  mesh.face_start = {0, 3, 6, 10};
  mesh.wedge_vertex = {0, 1, 1, 2, 3};
  mesh.face_matids = {0, 2, 1};
  mesh.vertex_ids = {1, 3, 4, 7};
  mesh.face_ids = {2, 5, 9};
  mesh.materials = {"matid=0", "matid=1 rgb=(0.5 0.25 0.125)", ""};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  mesh.wedge_keys.push(PlyMesh::WedgeKey{"Ouv", 2, {0.f, 0.f, 0.5f, 0.125f, nan, nan, 1.f, 1.f, 0.25f, 0.75f}});
  mesh.wedge_keys.push(PlyMesh::WedgeKey{"global", 0, {1.f, 0.f, 0.f, 1.f, 0.f}});
  mesh.comments = {"Created by Ply_test", ""};
  show_summary(mesh);
  for (const bool binary : {true, false}) {
    const PlyMesh mesh2 = round_trip(mesh, binary);
    showf("binary=%d same=%d\n", binary, same_meshes(mesh, mesh2));
  }
  {  // Corner uvs and a color alpha, without the trailing elements.
    PlyMesh mesh3;
    mesh3.points = mesh.points;
    mesh3.face_vertices = mesh.face_vertices;
    mesh3.face_start = mesh.face_start;
    for_int(i, mesh3.face_vertices.num()) mesh3.corner_uvs.push(Uv(float(i), 0.5f));
    mesh3.colors = mesh.colors;
    mesh3.colors[2][3] = 100;
    for (const bool binary : {true, false}) {
      const PlyMesh mesh4 = round_trip(mesh3, binary);
      showf("corner_uvs binary=%d same=%d\n", binary, same_meshes(mesh3, mesh4));
    }
  }
  {
    std::ostringstream oss;
    write_ply(oss, mesh, false);
    showf("%s", oss.str().c_str());
    // Read the PLY file as a GMesh, whose vertex 1 (with two wedges) gets corner strings.
    std::istringstream iss(oss.str());
    GMesh gmesh;
    gmesh.read(iss);
    std::ostringstream oss2;
    gmesh.write(oss2);
    showf("%s", oss2.str().c_str());
  }
}

void test_tristrips() {
  const string str =
      "ply\nformat ascii 1.0\nelement vertex 5\nproperty float x\nproperty float y\nproperty float z\n"
      "element tristrips 1\nproperty list int int vertex_indices\nend_header\n"
      "0 0 0\n1 0 0\n0 1 0\n1 1 0\n2 1 0\n"
      "8 0 1 2 3 -1 2 3 4\n";
  std::istringstream iss(str);
  const PlyMesh mesh = read_ply(iss);
  for_int(f, mesh.num_faces()) {
    showf("tristrips face %d:", f);
    for (const int v : mesh.face(f)) showf(" %d", v);
    showf("\n");
  }
}

void test_wmesh() {
  // A square of two triangles whose diagonal is a uv seam: vertex 0 and vertex 2 each have two wedges.
  WMesh wmesh;
  PMeshInfo pminfo;
  pminfo._has_uv = true;
  wmesh._materials.set(0, "matid=0");
  wmesh._materials.set(1, "matid=1");
  for (const Point& p : {Point(0.f, 0.f, 0.f), Point(1.f, 0.f, 0.f), Point(1.f, 1.f, 0.f), Point(0.f, 1.f, 0.f)})
    wmesh._vertices.push(PmVertex{PmVertexAttrib{p}});
  const auto add_wedge = [&](int v, Uv uv) {
    wmesh._wedges.push(PmWedge{v, PmWedgeAttrib{Vector(0.f, 0.f, 1.f), A3dColor(0.f, 0.f, 0.f), uv}});
  };
  add_wedge(0, Uv(0.f, 0.f));
  add_wedge(1, Uv(1.f, 0.f));
  add_wedge(2, Uv(1.f, 1.f));
  add_wedge(3, Uv(0.f, 1.f));
  add_wedge(0, Uv(0.5f, 0.f));
  add_wedge(2, Uv(0.5f, 1.f));
  wmesh._faces.push(PmFace{V(0, 1, 2), PmFaceAttrib{0}});
  wmesh._faces.push(PmFace{V(4, 5, 3), PmFaceAttrib{1}});
  wmesh.ok();
  std::ostringstream oss;
  wmesh.write_ply(oss, pminfo, true);
  std::istringstream iss(oss.str());
  const PlyMesh mesh = read_ply(iss);
  show_summary(mesh);
  showf("wedge_vertex:");
  for (const int v : mesh.wedge_vertex) showf(" %d", v);
  showf("\nface_matids: %d %d  materials: '%s' '%s'\n", mesh.face_matids[0], mesh.face_matids[1],
        mesh.materials[0].c_str(), mesh.materials[1].c_str());
  bool same_attribs = true;
  for_int(w, wmesh._wedges.num()) {
    const PmWedge& wedge = wmesh._wedges[w];
    same_attribs = same_attribs && mesh.points[w] == wmesh._vertices[wedge.vertex].attrib.point &&
                   mesh.normals[w] == wedge.attrib.normal && mesh.uvs[w] == wedge.attrib.uv &&
                   mesh.wedge_vertex[w] == wedge.vertex;
  }
  SHOW(same_attribs);
}

void test_gmesh_to_ply() {
  // A square of two triangles whose vertex 1 has two wedges (a normal discontinuity), with a flag key, partial keys,
  // and face strings with matid keys.  (The dropping of other keys and of edge strings is reported by a Warning(),
  // whose line number would make this output fragile.)
  const string str =
      "Vertex 1  0 0 0 {global}\n"
      "Vertex 2  1 0 0 {normal=(0 0 1) sph=(1 0 0)}\n"
      "Vertex 4  1 1 0 {normal=(0 0 1)}\n"
      "Vertex 5  0 1 0 {normal=(0 0 1) Ouv=(0.25 0.5)}\n"
      "Face 1  1 2 4 {matid=3 rgb=(1 0 0)}\n"
      "Face 2  1 4 5 {matid=1}\n"
      "Corner 1 1 {normal=(0 0 1)}\n"
      "Corner 1 2 {normal=(0 1 0)}\n";
  std::istringstream iss(str);
  GMesh gmesh;
  gmesh.read(iss);
  const PlyMesh mesh = gmesh.to_ply();
  std::ostringstream oss;
  write_ply(oss, mesh, false);
  showf("%s", oss.str().c_str());
  std::istringstream iss2(oss.str());
  GMesh gmesh2;
  gmesh2.read(iss2);
  std::ostringstream oss2;
  gmesh2.write(oss2);
  showf("%s", oss2.str().c_str());
}

}  // namespace

int main() {
  test_round_trip();
  test_tristrips();
  test_wmesh();
  test_gmesh_to_ply();
}
