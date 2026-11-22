// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/PointsInput.h"

#include <sstream>  // std::istringstream, std::ostringstream

#include "libHh/A3dStream.h"
#include "libHh/Ply.h"
using namespace hh;

namespace {

// Read the points of the input, echoing its provenance, and show them, all on std::cout as for the echo.  (An input
// that starts with 'p' but is neither a PLY file nor an A3d stream, or that contains faces, polygons, or polylines,
// aborts the program.)
void show_points(const string& name, const string& input) {
  std::cout << name << ":\n";
  std::istringstream iss(input);
  PointsInput points_input(iss);
  points_input.echo_provenance();
  const PointSet point_set = points_input.read();
  std::cout << sform(" %d points, %s\n", point_set.points.num(),
                     point_set.normals.num() ? "with normals" : "without normals");
  assertx(point_set.normals.empty() || point_set.normals.num() == point_set.points.num());
  for_int(i, point_set.points.num()) {
    std::cout << sform(" point %d: ", i) << point_set.points[i];
    if (point_set.normals.num()) std::cout << "  normal " << point_set.normals[i];
    std::cout << "\n";
  }
}

string binary_a3d() {
  std::ostringstream oss;
  {
    WSA3dStream oa3d(oss);
    A3dElem el(A3dElem::EType::point, true);
    el.push(A3dVertex(Point(1.f, 2.f, 3.f), Vector(0.f, 0.f, 1.f), A3dVertexColor(A3dColor(1.f, 1.f, 1.f))));
    oa3d.write(el);
    el.init(A3dElem::EType::point, true);
    el.push(A3dVertex(Point(4.f, 5.f, 6.f), Vector(0.f, 0.f, 0.f), A3dVertexColor(A3dColor(1.f, 1.f, 1.f))));
    oa3d.write(el);
  }
  return oss.str();
}

string binary_ply() {
  PlyMesh ply;
  ply.points = {Point(1.f, 2.f, 3.f), Point(4.f, 5.f, 6.f), Point(7.f, 8.f, 9.f)};
  ply.normals = {Vector(1.f, 0.f, 0.f), Vector(0.f, 1.f, 0.f), Vector(0.f, 0.f, 1.f)};
  ply.comments = {"Binary point set"};
  std::ostringstream oss;
  write_ply(oss, ply);
  return oss.str();
}

}  // namespace

int main() {
  show_points("A3d with comments and a normal",
              "# Provenance line 1.\n#Provenance line 2.\nn 0 0 1\np 1 2 3\np 4 5 6\n");
  show_points("A3d starting with a point", "p 1 2 3\np 4 5 6\n");
  show_points("A3d starting with a normal", "n 0 1 0\np 1 2 3\n");
  show_points("A3d starting with a color, with an end of frame", "d 1 0 0\np 1 2 3\nf 0 0 0\np 4 5 6\n");
  show_points("A3d binary", binary_a3d());
  show_points("PLY ascii",
              "ply\nformat ascii 1.0\ncomment Ascii point set\nelement vertex 2\nproperty float x\n"
              "property float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\n"
              "element face 0\nproperty list uchar int vertex_indices\nend_header\n1 2 3 0 0 1\n4 5 6 0 1 0\n");
  show_points("PLY binary", binary_ply());
  show_points("Empty input", "");
}
