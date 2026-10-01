// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GMesh.h"

#include <sstream>  // std::istringstream, std::ostringstream

#include "libHh/FileIO.h"
using namespace hh;

namespace {

int sum_destruct = 0;

struct Struct1 {
  explicit Struct1(int i = 0) : _i(i) { SHOW(i); }
  ~Struct1() {
    if (0) SHOW(_i);
    sum_destruct += _i;
    _i = std::numeric_limits<int>::max();
  }
  int _i;
};
using upStruct1 = unique_ptr<Struct1>;
HH_SACABLE(upStruct1);
HH_SAC_ALLOCATE_CD_FUNC(Mesh::MFace, upStruct1, f_pstruct1);

void show_mesh(GMesh& mesh) {
  mesh.ok();
  showf("Mesh {\n  Vertices (%d) {\n", mesh.num_vertices());
  string str;
  for (Vertex v : mesh.ordered_vertices()) {
    const Point& p = mesh.point(v);
    showf("    %d : %s\n", mesh.vertex_id(v), csform_vec(str, p));
  }
  showf("  } EndVertices\n  Edges (%d)\n  Faces (%d) {\n", mesh.num_edges(), mesh.num_faces());
  for (Face f : mesh.ordered_faces()) {
    showf("    Face %d {", mesh.face_id(f));
    for (Vertex v : mesh.vertices(f)) showf(" %d", mesh.vertex_id(v));
    showf(" }\n");
  }
  SHOW("  } EndFaces\n} EndMesh");
}

GMesh mesh_from_string(const string& str) {
  GMesh mesh;
  std::istringstream iss(str);
  mesh.read(iss);
  mesh.ok();
  return mesh;
}

string string_from_mesh(const GMesh& mesh) {
  std::ostringstream oss;
  mesh.write(oss);
  return oss.str();
}

// A square in the plane z == 0 with an interior center vertex 5, as four triangles and with mesh attributes.
// (A mesh written with more than one Edge string would list them in an unspecified order.)
const char* const k_mesh_fan = R"(# A comment line.
Vertex 1  0 0 0 {cusp normal=(0 0 1)}
Vertex 2  2 0 0 {normal=(0 0 1)}
Vertex 3  2 2 0
Vertex 4  0 2 0
Vertex 5  1 1 0 {uv=(.5 .5)}
Face 1  1 2 5 {mat=1}
Face 2  2 3 5 {mat=1}
Face 3  3 4 5 {mat=2}
Face 4  4 1 5 {mat=2}
Edge 2 5 {sharp}
Corner 3 2 {normal=(0 0 1) uv=(1 1)}
Corner 3 3 {normal=(0 0 -1)}
)";

void test_io() {
  GMesh mesh = mesh_from_string(k_mesh_fan);
  const string str = string_from_mesh(mesh);
  std::cout << str;
  // Reading the written mesh gives the same mesh.
  assertx(string_from_mesh(mesh_from_string(str)) == str);
  // Flags parsed from the strings.
  for (Vertex v : mesh.vertices()) assertx(mesh.flags(v).flag(GMesh::vflag_cusp) == (mesh.vertex_id(v) == 1));
  for (Edge e : mesh.edges()) {
    const bool expected = mesh.edge(mesh.id_vertex(2), mesh.id_vertex(5)) == e;
    assertx(mesh.flags(e).flag(GMesh::eflag_sharp) == expected);
    assertx(bool(mesh.get_string(e)) == expected);
  }
  // Recognized lines.
  assertx(GMesh::recognize_line("Vertex 1  0 0 0") && GMesh::recognize_line("Ecol 1 2"));
  assertx(!GMesh::recognize_line("# Vertex 1") && !GMesh::recognize_line("Vertex") && !GMesh::recognize_line(""));
  // A face index of zero is assigned automatically.
  GMesh mesh2 = mesh_from_string("Vertex 1  0 0 0\nVertex 2  1 0 0\nVertex 3  0 1 0\nFace 0  1 2 3\n");
  SHOW(mesh2.num_faces(), mesh2.face_id(mesh2.ordered_faces().begin()[0]));
  // Copy, move, and swap.
  GMesh mesh3;
  mesh3.copy(mesh);
  assertx(string_from_mesh(mesh3) == str);
  GMesh mesh4(std::move(mesh3));
  // NOLINTNEXTLINE(bugprone-use-after-move): the moved-from object is left empty.
  assertx(mesh3.empty() && string_from_mesh(mesh4) == str);
  swap(mesh2, mesh4);
  assertx(string_from_mesh(mesh2) == str && mesh4.num_faces() == 1);
  mesh4 = std::move(mesh2);
  // NOLINTNEXTLINE(bugprone-use-after-move): the moved-from object is left empty.
  assertx(mesh2.empty() && string_from_mesh(mesh4) == str);
  // Merging a mesh into another one renumbers the merged vertices.
  Map<Vertex, Vertex> mvvn;
  GMesh mesh5 = mesh_from_string("Vertex 1  0 0 5\nVertex 2  1 0 5\nVertex 3  0 1 5\nFace 1  1 2 3 {mat=3}\n");
  mesh5.merge(mesh, &mvvn);
  mesh5.ok();
  assertx(mvvn.num() == mesh.num_vertices());
  for (Vertex v : mesh.vertices()) assertx(mesh5.point(mvvn.get(v)) == mesh.point(v));
  std::cout << string_from_mesh(mesh5);
}

void test_geometry() {
  GMesh mesh = mesh_from_string(k_mesh_fan);
  const Face f1 = mesh.id_face(1);
  SHOW(mesh.triangle_points(f1), mesh.area(f1));
  Polygon poly;
  mesh.polygon(f1, poly);
  assertx(poly.num() == 3 && poly[0] == mesh.point(mesh.id_vertex(1)));
  const Edge e = mesh.edge(mesh.id_vertex(1), mesh.id_vertex(2));
  SHOW(mesh.length2(e), mesh.length(e));
  float sum_area = 0.f;
  for (Face f : mesh.faces()) sum_area += mesh.area(f);
  SHOW(sum_area);
  // A quadrilateral face.
  GMesh mesh2 =
      mesh_from_string("Vertex 1  0 0 0\nVertex 2  3 0 0\nVertex 3  3 2 0\nVertex 4  0 2 0\nFace 1  1 2 3 4\n");
  SHOW(mesh2.area(mesh2.id_face(1)));
  mesh2.transform(Frame::scaling(V(2.f, 1.f, 1.f)) * Frame::translation(V(1.f, 1.f, 1.f)));
  SHOW(mesh2.point(mesh2.id_vertex(3)), mesh2.area(mesh2.id_face(1)));
}

void test_strings() {
  GMesh mesh = mesh_from_string(k_mesh_fan);
  const Vertex v1 = mesh.id_vertex(1), v3 = mesh.id_vertex(3), v5 = mesh.id_vertex(5);
  const Face f2 = mesh.id_face(2), f3 = mesh.id_face(3);
  string str;
  SHOW(mesh.get_string(v1), mesh.get_string(f2));
  assertx(!mesh.get_string(v3));
  assertx(GMesh::string_has_key(mesh.get_string(v1), "cusp") && !GMesh::string_has_key(mesh.get_string(v1), "cus"));
  assertx(!GMesh::string_has_key(nullptr, "cusp"));
  SHOW(GMesh::string_key(str, mesh.get_string(v1), "normal"));
  SHOW(GMesh::string_key(str, mesh.get_string(v1), "cusp"));  // A key without a value has value "".
  assertx(!GMesh::string_key(str, mesh.get_string(v1), "uv"));
  // A corner key falls back on the vertex string.
  const Corner c32 = mesh.corner(v3, f2), c33 = mesh.corner(v3, f3), c52 = mesh.corner(v5, f2);
  SHOW(mesh.get_string(c32), mesh.corner_key(str, c32, "uv"), mesh.corner_key(str, c52, "uv"));
  assertx(!mesh.corner_key(str, c33, "uv"));
  Vector nor;
  assertx(mesh.parse_corner_key_vec(c33, "normal", nor));
  SHOW(nor);
  assertx(!mesh.parse_corner_key_vec(mesh.corner(mesh.id_vertex(4), f3), "normal", nor));
  Uv uv;
  assertx(mesh.parse_corner_key_vec(c52, "uv", uv));
  SHOW(uv);
  assertx(parse_key_vec("a=1 normal=(1.5 -2 3e2) b", "normal", nor));
  SHOW(nor);
  assertx(!parse_key_vec("a=1 b", "normal", nor) && !parse_key_vec(nullptr, "normal", nor));
  SHOW(csform_vec(str, V(1.5f, -2.f, 1e-7f)), csform_vec(str, Vec2<float>(1.f / 3.f, 1e10f)));
  // Iterate over the attributes of a string.
  Array<char> key, val;
  for_cstring_key_value(R"(a=1 b c=(2 3) d="x y" e=-4)", key, val,
                        [&] { showf("%s='%s'\n", key.data(), val.data()); });
  // Update, set, and extract strings.
  mesh.update_string(v3, "rgb", "(1 0 0)");
  mesh.update_string(v1, "cusp", nullptr);
  mesh.update_string(f2, "mat", "5");
  mesh.update_string(c32, "uv", nullptr);
  mesh.update_string(mesh.edge(v1, v5), "crease", "");
  SHOW(mesh.get_string(v3), mesh.get_string(v1), mesh.get_string(f2), mesh.get_string(c32));
  SHOW(mesh.get_string(mesh.edge(v1, v5)));
  mesh.update_string(c32, "normal", nullptr);
  assertx(!mesh.get_string(c32));  // An empty string is cleared.
  unique_ptr<char[]> s = mesh.extract_string(f2);
  assertx(!mesh.get_string(f2) && s && string(s.get()) == "mat=5");
  mesh.set_string(f3, std::move(s));
  mesh.set_string(v5, "");
  SHOW(mesh.get_string(f3), mesh.get_string(v5));
  mesh.set_string(v5, nullptr);
  assertx(!mesh.get_string(v5));
}

// Mesh operations update the geometry and strings, and their recorded changes can be replayed.
void test_operations() {
  GMesh mesh = mesh_from_string(k_mesh_fan);
  GMesh original;
  original.copy(mesh);
  std::ostringstream oss;
  assertx(!mesh.record_changes(&oss));
  const auto vertex = [&](int i) { return mesh.id_vertex(i); };
  // Splitting an edge places the new vertex at its midpoint, and keeps the face strings and sharp edge flag.
  const Vertex v6 = mesh.split_edge(mesh.edge(vertex(2), vertex(5)), 6);
  mesh.ok();
  SHOW(mesh.point(v6));
  assertx(mesh.flags(mesh.edge(vertex(2), v6)).flag(GMesh::eflag_sharp));
  for (Face f : mesh.faces(v6)) assertx(string(mesh.get_string(f)) == "mat=1");
  // Swapping an edge between faces with identical strings keeps those strings.
  const Edge e = mesh.swap_edge(mesh.edge(vertex(4), vertex(5)));
  mesh.ok();
  for (Face f : mesh.faces(e)) SHOW(f, mesh.get_string(f));
  // Collapsing an interior edge places the remaining vertex at the midpoint.
  mesh.collapse_edge_vertex(mesh.edge(vertex(5), v6), vertex(5));
  mesh.ok();
  SHOW(mesh.point(vertex(5)));
  // Collapsing an edge between an interior and a boundary vertex places the vertex on the boundary.
  mesh.collapse_edge_vertex(mesh.edge(vertex(5), vertex(1)), vertex(5));
  mesh.ok();
  SHOW(mesh.point(vertex(5)), mesh.num_vertices(), mesh.num_faces());
  // Splitting a face about its center.
  const Face f = mesh.ordered_faces().begin()[0];
  const Vertex vc = mesh.center_split_face(f);
  mesh.ok();
  SHOW(mesh.point(vc), mesh.get_string(vc));
  std::cout << string_from_mesh(mesh);
  mesh.record_changes(nullptr);
  const string changes = oss.str();
  std::cout << changes;
  // Replaying the recorded changes on the original mesh gives the same mesh.
  std::istringstream iss(changes);
  for (string line; my_getline(iss, line);) {
    assertx(GMesh::recognize_line(line));
    original.read_line(line.data());
  }
  original.ok();
  // (The recorded points are written with only 6 significant digits.)
  for (Vertex v : mesh.vertices())
    assertx(dist(original.point(original.id_vertex(mesh.vertex_id(v))), mesh.point(v)) < 1e-5f);
  for (Face ff : mesh.faces()) {
    Array<int> ids, ids2;
    for (Vertex v : mesh.vertices(ff)) ids.push(mesh.vertex_id(v));
    for (Vertex v : original.vertices(original.id_face(mesh.face_id(ff)))) ids2.push(original.vertex_id(v));
    assertx(ids == ids2);
  }
}

void test_more_operations() {
  {
    // Swapping an edge between faces with different strings drops those strings.
    GMesh mesh0 = mesh_from_string(k_mesh_fan);
    const Edge e0 = mesh0.swap_edge(mesh0.edge(mesh0.id_vertex(3), mesh0.id_vertex(5)));
    mesh0.ok();
    for (Face f : mesh0.faces(e0)) SHOW(f, mesh0.get_string(f));
  }
  {
    // Coalescing faces with identical strings keeps the string; inserting a vertex places it at the edge midpoint.
    GMesh mesh = mesh_from_string(k_mesh_fan);
    const Face fn = mesh.coalesce_faces(mesh.edge(mesh.id_vertex(4), mesh.id_vertex(5)));
    mesh.ok();
    SHOW(fn, mesh.get_string(fn));
    const Vertex vn = mesh.insert_vertex_on_edge(mesh.edge(mesh.id_vertex(2), mesh.id_vertex(5)));
    mesh.ok();
    SHOW(mesh.point(vn));
    const Face fn2 = mesh.coalesce_faces(mesh.edge(mesh.id_vertex(1), mesh.id_vertex(5)));
    mesh.ok();
    SHOW(fn2, mesh.get_string(fn2));  // The strings differ, so the string is dropped.
    // Splitting a face keeps its string.
    GMesh mesh2 = mesh_from_string(
        "Vertex 1  0 0 0\nVertex 2  1 0 0\nVertex 3  1 1 0\nVertex 4  0 1 0\nFace 1  1 2 3 4 {mat=7}\n");
    const Edge e = mesh2.split_face(mesh2.id_face(1), mesh2.id_vertex(1), mesh2.id_vertex(3));
    mesh2.ok();
    for (Face f : mesh2.faces(e)) SHOW(f, mesh2.get_string(f));
  }
  {
    // Fixing a non-nice vertex copies its point and string to the new vertex.
    GMesh mesh = mesh_from_string(
        "Vertex 1  0 0 0 {a}\nVertex 2  1 0 0\nVertex 3  0 1 0\nVertex 4  -1 0 0\nVertex 5  0 -1 0\n"
        "Face 1  1 2 3\nFace 2  1 4 5\n");
    const Array<Vertex> new_vertices = mesh.fix_vertex(mesh.id_vertex(1));
    mesh.ok();
    assertx(new_vertices.num() == 1);
    SHOW(mesh.point(new_vertices[0]), mesh.get_string(new_vertices[0]));
    assertx(mesh.is_nice());
  }
}

}  // namespace

int main() {
  {
    const char* s1 = "sharp normal=(.1 .2 .3) groups=\"tuv=3\" uv=(1 2) tag";
    const char* s2 = "normal=(.4 .5 .6) groups=\"tuv=3\" sharp tag uv=(3 4)";
    const char* s3 = "uv=(1 2)";
    assertx(!GMesh::string_has_key(s1, "rgb"));
    SHOW(GMesh::string_update(s1, "sharp", nullptr));
    SHOW(GMesh::string_update(s1, "sharp", ""));
    SHOW(GMesh::string_update(s1, "sharp", "(1 2)"));
    SHOW(GMesh::string_update(s1, "sharp", "\"true\""));
    SHOW(GMesh::string_update(s1, "normal", nullptr));
    SHOW(GMesh::string_update(s1, "normal", ""));
    SHOW(GMesh::string_update(s1, "normal", "(0.0 0.1 0.2)"));
    SHOW(GMesh::string_update(s1, "groups", nullptr));
    SHOW(GMesh::string_update(s1, "groups", ""));
    SHOW(GMesh::string_update(s1, "groups", "\"group\""));
    SHOW(GMesh::string_update(s1, "uv", nullptr));
    SHOW(GMesh::string_update(s1, "uv", ""));
    SHOW(GMesh::string_update(s1, "uv", "()"));
    SHOW(GMesh::string_update(s1, "tag", nullptr));
    SHOW(GMesh::string_update(s1, "tag", ""));
    SHOW(GMesh::string_update(s1, "tag", "(hello)"));
    SHOW(GMesh::string_update(s1, "new", nullptr));
    SHOW(GMesh::string_update(s1, "new", ""));
    SHOW(GMesh::string_update(s1, "new", "(1)"));
    SHOW(GMesh::string_update(s2, "normal", nullptr));
    SHOW(GMesh::string_update(s2, "normal", "(0.0 0.1 0.2)"));
    SHOW(GMesh::string_update(s2, "uv", nullptr));
    SHOW(GMesh::string_update(s2, "uv", ""));
    SHOW(GMesh::string_update(s2, "uv", "(5 6)"));
    SHOW(GMesh::string_update(s2, "sharp", nullptr));
    SHOW(GMesh::string_update(s2, "tag", nullptr));
    SHOW(GMesh::string_update(s2, "sharp", ""));
    SHOW(GMesh::string_update(s3, "uv", "(3 4)"));
    SHOW(GMesh::string_update(s3, "uv", ""));
    assertx(GMesh::string_update(s3, "uv", nullptr) == "");  // Not == nullptr.
    SHOW(GMesh::string_update(s3, "sharp", ""));
  }
  {
    GMesh mesh;
    mesh.read(RFile("-")());
    show_mesh(mesh);
    SHOW("original");
    mesh.write(std::cout);
    SHOW("renumbered");
    mesh.renumber();
    mesh.write(std::cout);
    for (Face f : mesh.ordered_faces()) f_pstruct1(f) = make_unique<Struct1>(mesh.face_id(f));
    mesh.clear();
    // The mesh faces are destroyed in an unspecified order, so only the sum of the destroyed ids is shown.
    SHOW(sum_destruct);
  }
  test_io();
  test_geometry();
  test_strings();
  test_operations();
  test_more_operations();
}
