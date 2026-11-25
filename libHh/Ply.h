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
// has in some of its adjacent faces.  The face element may have the standard color properties "red", "green", "blue"
// (and "alpha"), as uchar or float.  The optional element "edge" (properties int "vertex1", "vertex2") lists edges
// by their PLY vertices.  The optional trailing elements below, which other PLY readers can ignore, identify the
// mesh vertices and carry further per-wedge, per-edge, and per-face data:
// - "wedge_vertex" (property int "vertex"): the 0-based mesh vertex of each PLY vertex;
// - "face_material" (property ushort "matid"): the material of each face, whose string is given by a header line
//   "comment material <matid> <string>";
// - "vertex_id" (property int "id", one per mesh vertex) and "face_id" (property int "id"): the 1-based ids of the
//   mesh vertices and faces (as in a .m file), if these are not consecutive;
// - "wedge_<key>", "edge_<key>", and "face_<key>", with one instance per PLY vertex, edge, or face: either
//   (properties float "c0", "c1", ...) the numeric vector value of attribute <key>, with NaN in all components for
//   an absent value, or (property uchar "flag") whether the instance has the valueless attribute <key>.
struct PlyMesh {
  // An attribute of the PLY vertices, edges, or faces, stored in a trailing element "wedge_<name>", "edge_<name>", or
  // "face_<name>".
  struct Key {
    string name;
    int dim;              // Number of components; 0 for a valueless flag.
    Array<float> values;  // dim values per instance (NaN if absent); for a flag, one value 0.f or 1.f.
    // Append to s the attribute of instance i as "name", "name=value", or "name=(value value ...)", preceded by a space
    // if s is nonempty; append nothing if the instance lacks the attribute.
    void append_to(string& s, int i) const;
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
  Array<Key> wedge_keys;
  Array<Vec2<int>> edge_vertices;  // Per edge (element "edge"), its two PLY vertices.
  Array<Key> edge_keys;            // Per-edge attributes, parallel to edge_vertices.
  Array<Key> face_keys;            // Per-face attributes.
  Array<Pixel> face_colors;        // Per face (red, green, blue[, alpha]; float colors scaled by 255), or empty.
  Array<string> comments;          // The header comments other than the materials (without the leading "comment ").
  // Append each line of text (e.g., diagnostics_transcript(), to record the provenance) as a comment.
  void add_comment_lines(std::string_view text);
  [[nodiscard]] int num_faces() const { return max(face_start.num() - 1, 0); }
  [[nodiscard]] CArrayView<int> face(int f) const { return face_vertices.slice(face_start[f], face_start[f + 1]); }
  [[nodiscard]] int face_matid(int f) const { return face_matids.num() ? face_matids[f] : 0; }
  // Returns the number of mesh vertices, given wedge_vertex.
  [[nodiscard]] int num_mesh_vertices() const;
  // Returns the string of face f: its material string, followed by its attributes of face_keys and its color
  // face_colors[f] (as "rgb=(r g b)" with 3 decimals).
  [[nodiscard]] string face_string(int f) const;
  // Returns the string of edge i: its attributes of edge_keys.
  [[nodiscard]] string edge_string(int i) const;
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

// Write a PLY file, in the format binary_big_endian (network order) or else, if the environment variable PLY_BINARY
// is 0 (read at each call), ascii, including the trailing elements above for the nonempty arrays.  A comment that
// would read as a material definition is indented by a space.
void write_ply(std::ostream& os, const PlyMesh& mesh);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_PLY_H_
