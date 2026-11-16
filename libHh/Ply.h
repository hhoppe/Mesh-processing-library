// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_PLY_H_
#define MESH_PROCESSING_LIBHH_PLY_H_

#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/Pixel.h"

namespace hh {

// A polygon mesh (or a point set if it has no faces) read from a PLY file in the format ascii,
// binary_little_endian, or binary_big_endian.
struct PlyMesh {
  Array<Point> points;
  Array<Vector> normals;     // Per vertex (nx, ny, nz), or empty.
  Array<Pixel> colors;       // Per vertex (red, green, blue[, alpha]; float colors scaled by 255), or empty.
  Array<Uv> uvs;             // Per vertex (s, t; or u, v; or texture_u, texture_v), or empty.
  Array<int> face_vertices;  // Concatenated vertex indices of all faces.
  Array<int> face_start;     // Face f is face_vertices[face_start[f] .. face_start[f + 1]); size nf + 1.
  Array<Uv> corner_uvs;      // Parallel to face_vertices (face list "texcoord"), or empty.
  [[nodiscard]] int num_faces() const { return max(face_start.num() - 1, 0); }
  [[nodiscard]] CArrayView<int> face(int f) const { return face_vertices.slice(face_start[f], face_start[f + 1]); }
};

// Read the header lines of a PLY file, from "ply" through "end_header", without any trailing whitespace or '\r'.
[[nodiscard]] Array<string> read_ply_header(std::istream& is);

// Read the body of a PLY file given its header lines.  The properties may have any order and type, a face list
// named "vertex_indices" (or "vertex_index") may have any count type, and other properties and elements are
// skipped (and reported in one line).  Dies with a descriptive message on malformed data.  The stream is read
// exactly up to the end of the PLY data, so other data may follow it.
[[nodiscard]] PlyMesh read_ply_body(std::istream& is, CArrayView<string> header_lines);

// Read a PLY file (its header and body).
[[nodiscard]] PlyMesh read_ply(std::istream& is);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_PLY_H_
