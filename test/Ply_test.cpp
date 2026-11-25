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

bool same_keys(CArrayView<PlyMesh::Key> keys1, CArrayView<PlyMesh::Key> keys2) {
  if (keys1.num() != keys2.num()) return false;
  for_int(i, keys1.num()) {
    const PlyMesh::Key &k1 = keys1[i], &k2 = keys2[i];
    if (k1.name != k2.name || k1.dim != k2.dim || !same_floats(k1.values, k2.values)) return false;
  }
  return true;
}

bool same_meshes(const PlyMesh& m1, const PlyMesh& m2) {
  if (!same_keys(m1.wedge_keys, m2.wedge_keys) || !same_keys(m1.edge_keys, m2.edge_keys) ||
      !same_keys(m1.face_keys, m2.face_keys))
    return false;
  return m1.edge_vertices == m2.edge_vertices && m1.face_colors == m2.face_colors && m1.points == m2.points &&
         m1.normals == m2.normals && m1.colors == m2.colors && m1.uvs == m2.uvs &&
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
  my_setenv("PLY_BINARY", binary ? "1" : "0");
  write_ply(oss, mesh);
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
  mesh.wedge_keys.push(PlyMesh::Key{"Ouv", 2, {0.f, 0.f, 0.5f, 0.125f, nan, nan, 1.f, 1.f, 0.25f, 0.75f}});
  mesh.wedge_keys.push(PlyMesh::Key{"global", 0, {1.f, 0.f, 0.f, 1.f, 0.f}});
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
    my_setenv("PLY_BINARY", "0");
    write_ply(oss, mesh);
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
  my_setenv("PLY_BINARY", "1");
  wmesh.write_ply(oss, pminfo);
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
  // and face strings with matid keys.  (The dropping of other keys is reported by a Warning(), whose line number would
  // make this output fragile.)
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
  my_setenv("PLY_BINARY", "0");
  write_ply(oss, mesh);
  showf("%s", oss.str().c_str());
  std::istringstream iss2(oss.str());
  GMesh gmesh2;
  gmesh2.read(iss2);
  std::ostringstream oss2;
  gmesh2.write(oss2);
  showf("%s", oss2.str().c_str());
}

void test_edge_face_keys() {
  // Two triangles with two listed edges, per-edge keys (a flag and a 2-float vector), a per-face key, and face colors.
  PlyMesh mesh;
  mesh.points = {Point(0.f, 0.f, 0.f), Point(1.f, 0.f, 0.f), Point(1.f, 1.f, 0.f), Point(0.f, 1.f, 0.f)};
  mesh.face_vertices = {0, 1, 2, 0, 2, 3};
  mesh.face_start = {0, 3, 6};
  mesh.edge_vertices = {V(0, 2), V(1, 2)};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  mesh.edge_keys.push(PlyMesh::Key{"sharp", 0, {1.f, 0.f}});
  mesh.edge_keys.push(PlyMesh::Key{"filletr", 2, {nan, nan, 0.5f, 0.25f}});
  mesh.face_keys.push(PlyMesh::Key{"oldfid", 1, {7.f, nan}});
  mesh.face_colors = {Pixel(255, 0, 0), Pixel(1, 2, 3)};
  mesh.face_matids = Array<int>{0, 1};
  mesh.materials = {"matid=0", "matid=1 mat=\"x y\""};
  for (const bool binary : {true, false}) {
    const PlyMesh mesh2 = round_trip(mesh, binary);
    showf("edge_face_keys binary=%d same=%d\n", binary, same_meshes(mesh, mesh2));
  }
  std::ostringstream oss;
  my_setenv("PLY_BINARY", "0");
  write_ply(oss, mesh);
  showf("%s", oss.str().c_str());
  for_int(f, mesh.num_faces()) showf("face_string(%d)='%s'\n", f, mesh.face_string(f).c_str());
  for_int(i, mesh.edge_vertices.num()) showf("edge_string(%d)='%s'\n", i, mesh.edge_string(i).c_str());
  {  // Float face colors with alpha, as written by other programs.
    const string str =
        "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\n"
        "element face 1\nproperty list uchar int vertex_indices\nproperty float red\nproperty float green\n"
        "property float blue\nproperty float alpha\nend_header\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2 1 0.5 0 0.25\n";
    std::istringstream iss(str);
    const PlyMesh mesh3 = read_ply(iss);
    const Pixel& pixel = mesh3.face_colors[0];
    showf("float face color: %d %d %d %d  face_string='%s'\n", pixel[0], pixel[1], pixel[2], pixel[3],
          mesh3.face_string(0).c_str());
  }
}

// Returns the mesh read from the PLY file written for gmesh.
GMesh gmesh_round_trip(const GMesh& gmesh, bool binary) {
  std::ostringstream oss;
  my_setenv("PLY_BINARY", binary ? "1" : "0");
  write_ply(oss, gmesh.to_ply());
  std::istringstream iss(oss.str());
  GMesh gmesh2;
  gmesh2.read(iss);
  return gmesh2;
}

// Returns the ascii PLY header written for gmesh, without its comments.
string ply_header(const GMesh& gmesh) {
  std::ostringstream oss;
  my_setenv("PLY_BINARY", "0");
  write_ply(oss, gmesh.to_ply());
  std::istringstream iss(oss.str());
  string result;
  for (const string& line : read_ply_header(iss))
    if (!line.starts_with("comment")) result += line + "\n";
  return result;
}

// Returns the attributes of the string ("key" or "key=value"), sorted.
Array<string> sorted_attributes(const char* str) {
  Array<string> attributes;
  if (str) {
    for_cstring_key_value_ptr(str, [&](const char* kb, int kl, const char* vb, int vl) {
      attributes.push(string(kb, vl ? vb + vl - kb : kl));
      return false;
    });
  }
  sort(attributes);
  return attributes;
}

// Returns the number of faces and edges whose strings differ (up to the order of their attributes) between the two
// meshes, which have the same vertices.
int num_string_differences(const GMesh& gmesh1, const GMesh& gmesh2) {
  int num = 0;
  for (Face f1 : gmesh1.faces()) {
    const Face f2 = gmesh2.id_face(gmesh1.face_id(f1));
    if (sorted_attributes(gmesh1.get_string(f1)) != sorted_attributes(gmesh2.get_string(f2))) num++;
  }
  for (Edge e1 : gmesh1.edges()) {
    const Edge e2 = gmesh2.edge(gmesh2.id_vertex(gmesh1.vertex_id(gmesh1.vertex1(e1))),
                                gmesh2.id_vertex(gmesh1.vertex_id(gmesh1.vertex2(e1))));
    if (sorted_attributes(gmesh1.get_string(e1)) != sorted_attributes(gmesh2.get_string(e2))) num++;
    if (gmesh1.flags(e1).flag(GMesh::eflag_sharp) != gmesh2.flags(e2).flag(GMesh::eflag_sharp)) num++;
  }
  return num;
}

// Returns a mesh of separate triangles with the given face strings.
GMesh separate_triangles(CArrayView<string> face_strings) {
  GMesh gmesh;
  for_int(i, face_strings.num()) {
    Vec3<Vertex> va;
    for_int(k, 3) {
      va[k] = gmesh.create_vertex();
      gmesh.set_point(va[k], Point(float(i), float(k == 1), float(k == 2)));
    }
    gmesh.set_string(gmesh.create_face(va), face_strings[i].c_str());
  }
  return gmesh;
}

void test_gmesh_strings() {
  {  // Material mode, with edge strings: a flag only, a 2-float vector only, and both.
    const string str =
        "Vertex 1  0 0 0\nVertex 2  1 0 0\nVertex 3  1 1 0\nVertex 4  0 1 0\nVertex 5  2 0 0\n"
        "Face 1  1 2 3 {matid=0 mat=\"a b\"}\nFace 2  1 3 4 {matid=1 oldfid=7}\nFace 3  2 5 3\n"
        "Edge 1 3 {sharp}\nEdge 2 3 {filletr=(0.5 0.25)}\nEdge 3 4 {sharp filletr=(1 2)}\n";
    std::istringstream iss(str);
    GMesh gmesh;
    gmesh.read(iss);
    showf("%s", ply_header(gmesh).c_str());
    for (const bool binary : {true, false}) {
      const GMesh gmesh2 = gmesh_round_trip(gmesh, binary);
      showf("material mode binary=%d differences=%d\n", binary, num_string_differences(gmesh, gmesh2));
      if (binary) {
        std::ostringstream oss;
        gmesh2.write(oss);
        showf("%s", oss.str().c_str());
      }
    }
  }
  // Split mode: more than 1000 distinct face strings, on separate triangles.
  const auto face_strings = [](const auto& face_string) -> Array<string> {
    Array<string> strings;
    for_int(i, 1200) strings.push(face_string(i));
    return strings;
  };
  {  // With an rgb on every face (written as standard face colors, hence with values k / 255).
    const GMesh gmesh = separate_triangles(face_strings([](int i) {
      return sform("matid=%d mat=\"m%d\" oldfid=%d rgb=(%.3f %.3f %.3f)%s", i % 3, i % 3, i + 1, (i % 256) / 255.f,
                   (i * 7 % 256) / 255.f, (i * 13 % 256) / 255.f, i % 5 ? "" : " hole");
    }));
    showf("%s", ply_header(gmesh).c_str());
    for (const bool binary : {true, false}) {
      const GMesh gmesh2 = gmesh_round_trip(gmesh, binary);
      showf("split mode binary=%d differences=%d\n", binary, num_string_differences(gmesh, gmesh2));
      if (binary) showf("face 1: '%s'\n", gmesh2.get_string(gmesh2.id_face(1)));
    }
  }
  {  // With an rgb on some faces (written as an element face_rgb).
    const GMesh gmesh = separate_triangles(face_strings([](int i) {
      return sform("oldfid=%d%s", i + 1, i % 2 ? "" : sform(" rgb=(%g 0.5 1)", (i % 4) * 0.25f).c_str());
    }));
    showf("%s", ply_header(gmesh).c_str());
    for (const bool binary : {true, false}) {
      const GMesh gmesh2 = gmesh_round_trip(gmesh, binary);
      showf("split mode partial rgb binary=%d differences=%d\n", binary, num_string_differences(gmesh, gmesh2));
    }
  }
}

}  // namespace

int main() {
  test_round_trip();
  test_tristrips();
  test_wmesh();
  test_gmesh_to_ply();
  test_edge_face_keys();
  test_gmesh_strings();
}
