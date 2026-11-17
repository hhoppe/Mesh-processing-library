// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_PLY_H_
#define MESH_PROCESSING_LIBHH_PLY_H_

#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/Pixel.h"

namespace hh {

// A polygon mesh (or a point set if it has no faces) read from or written to a PLY file in the format ascii,
// binary_little_endian, or binary_big_endian.
//
// Each PLY vertex may be a wedge of a mesh vertex: a vertex together with the attributes (normal, color, uv) that it
// has in some of its adjacent faces.  The optional trailing elements below, which other PLY readers can ignore,
// identify the mesh vertices and carry further per-wedge and per-face data:
// - "wedge_vertex" (property int "vertex"): the 0-based mesh vertex of each PLY vertex;
// - "face_material" (property ushort "matid"): the material of each face, whose string is given by a header line
//   "comment material <matid> <string>";
// - "vertex_id" (property int "id", one per mesh vertex) and "face_id" (property int "id"): the 1-based ids of the
//   mesh vertices and faces (as in a .m file), if these are not consecutive;
// - "wedge_<key>" (properties float "c0", "c1", ...): the numeric vector value of attribute <key> of each PLY vertex,
//   with NaN for an absent value; or (property uchar "flag"): whether each PLY vertex has the valueless attribute
//   <key>.
struct PlyMesh {
  // An attribute of the PLY vertices, stored in a trailing element "wedge_<name>".
  struct WedgeKey {
    string name;
    int dim;              // Number of components; 0 for a valueless flag.
    Array<float> values;  // dim values per PLY vertex (NaN if absent); for a flag, one value 0.f or 1.f.
  };
  Array<Point> points;
  Array<Vector> normals;     // Per vertex (nx, ny, nz), or empty.
  Array<Pixel> colors;       // Per vertex (red, green, blue[, alpha]; float colors scaled by 255), or empty.
  Array<Uv> uvs;             // Per vertex (s, t; or u, v; or texture_u, texture_v), or empty.
  Array<int> face_vertices;  // Concatenated vertex indices of all faces.
  Array<int> face_start;     // Face f is face_vertices[face_start[f] .. face_start[f + 1]); size nf + 1.
  Array<Uv> corner_uvs;      // Parallel to face_vertices (face list "texcoord"), or empty.
  Array<int> wedge_vertex;   // Per vertex, its 0-based mesh vertex; empty if each PLY vertex is a mesh vertex.
  Array<int> face_matids;    // Per face, its material; empty if all faces have material 0.
  Array<int> vertex_ids;     // Per mesh vertex, its 1-based id; empty if the ids are 1, 2, ....
  Array<int> face_ids;       // Per face, its 1-based id; empty if the ids are 1, 2, ....
  Array<string> materials;   // Material strings, indexed by matid (possibly empty).
  Array<WedgeKey> wedge_keys;
  Array<string> comments;  // The header comments other than the materials (without the leading "comment ").
  [[nodiscard]] int num_faces() const { return max(face_start.num() - 1, 0); }
  [[nodiscard]] CArrayView<int> face(int f) const { return face_vertices.slice(face_start[f], face_start[f + 1]); }
  [[nodiscard]] int face_matid(int f) const { return face_matids.num() ? face_matids[f] : 0; }
  // Returns the number of mesh vertices, given wedge_vertex.
  [[nodiscard]] int num_mesh_vertices() const;
};

// Read the header lines of a PLY file, from "ply" through "end_header", without any trailing whitespace or '\r'.
[[nodiscard]] Array<string> read_ply_header(std::istream& is);

// Returns the provenance lines in the header of a PLY file: its comments (other than empty ones and the materials)
// and its "obj_info" lines (with that keyword), for a reader to echo with a '|' prefix.
[[nodiscard]] Array<string> ply_header_comments(CArrayView<string> header_lines);

// Read the body of a PLY file given its header lines.  The properties may have any order and type, a face list
// named "vertex_indices" (or "vertex_index") may have any count type, triangle strips (element "tristrips", whose
// list "vertex_indices" separates the strips by -1) are converted to faces, and other properties and elements are
// skipped (and reported in one line).  Dies with a descriptive message on malformed data.  The stream is read
// exactly up to the end of the PLY data, so other data may follow it.
[[nodiscard]] PlyMesh read_ply_body(std::istream& is, CArrayView<string> header_lines);

// Read a PLY file (its header and body).
[[nodiscard]] PlyMesh read_ply(std::istream& is);

// Returns the diagnostics written so far (see diagnostics_transcript()), as lines for the comments of a PLY file,
// so that the file records its provenance: the inherited header lines (prefixed by '|') and those of this program.
[[nodiscard]] Array<string> provenance_comments();

// Write a PLY file, in the format binary_big_endian (network order) or ascii, including the trailing elements above
// for the nonempty arrays.
void write_ply(std::ostream& os, const PlyMesh& mesh, bool binary);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_PLY_H_
