// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GMesh.h"
using namespace hh;

namespace {

int sum_destruct = 0;

struct Struct1 {
  Struct1(int i = 0) : _i(i) { SHOW(i); }
  ~Struct1() {
    if (0) SHOW(_i);
    sum_destruct += _i;
    _i = INT_MAX;
  }
  int _i;
};
using upStruct1 = unique_ptr<Struct1>;
HH_SACABLE(upStruct1);
HH_SAC_ALLOCATE_CD_FUNC(Mesh::MFace, upStruct1, f_pstruct1);

void showmesh(GMesh& mesh) {
  mesh.ok();
  showf("Mesh {\n  Vertices (%d) {\n", mesh.num_vertices());
  for (Vertex v : mesh.ordered_vertices()) {
    const Point& p = mesh.point(v);
    showf("    %d : (%g %g %g)\n", mesh.vertex_id(v), p[0], p[1], p[2]);
  }
  showf("  } EndVertices\n  Edges (%d)\n  Faces (%d) {\n", mesh.num_edges(), mesh.num_faces());
  for (Face f : mesh.ordered_faces()) {
    showf("    Face %d {", mesh.face_id(f));
    for (Vertex v : mesh.vertices(f)) showf(" %d", mesh.vertex_id(v));
    showf(" }\n");
  }
  SHOW("  } EndFaces\n} EndMesh");
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
    assertx(GMesh::string_update(s3, "uv", nullptr) == "");  // not == nullptr
    SHOW(GMesh::string_update(s3, "sharp", ""));
  }
  {
    GMesh mesh;
    mesh.read(std::cin);
    showmesh(mesh);
    SHOW("original");
    mesh.write(std::cout);
    SHOW("renumbered");
    mesh.renumber();
    mesh.write(std::cout);
    for (Face f : mesh.ordered_faces()) f_pstruct1(f) = make_unique<Struct1>(mesh.face_id(f));
    mesh.clear();
    // The mesh faces are destroyed in a non-sorted order.
    SHOW(sum_destruct);
  }
}
