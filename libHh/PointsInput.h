// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_POINTSINPUT_H_
#define MESH_PROCESSING_LIBHH_POINTSINPUT_H_

#include "libHh/Array.h"
#include "libHh/Geometry.h"

namespace hh {

// A set of points, with their normals if present.
struct PointSet {
  Array<Point> points;
  Array<Vector> normals;  // Parallel to points (zero where absent), or empty if no point has a normal.
};

// The points input of a program: either an A3d stream of points (e.g., a *.pts file), or a PLY file whose vertices
// (with their normals nx, ny, nz if present) are the points.  It is read in
// two steps so that a program can echo the provenance of the input before writing its own header:
//   PointsInput input(fi());   // Read the provenance: the '#' lines of an A3d stream, or the comments of a PLY file.
//   input.echo_provenance();   // (Optional) Echo it with showff(), each line with the prefix '|'.
//   showff("%s", args.header().c_str());
//   const PointSet point_set = input.read();
// A PLY file is recognized by its first two characters "pl" (a text A3d stream may start with "p " and a binary one
// starts with the byte 3).  The input must contain only points: a PLY file with faces, or an A3d stream with polygons
// or polylines (e.g., a mesh given by mistake), is an error.
class PointsInput {
 public:
  explicit PointsInput(std::istream& is);
  void echo_provenance() const;
  [[nodiscard]] PointSet read();

 private:
  std::istream& _is;
  Array<string> _provenance;
  Array<string> _ply_header_lines;  // Nonempty if the input is a PLY file.
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_POINTSINPUT_H_
