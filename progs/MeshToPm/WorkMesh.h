// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_PROGS_MESHTOPM_WORKMESH_H_
#define MESH_PROCESSING_PROGS_MESHTOPM_WORKMESH_H_

#include "libHh/Array.h"
#include "libHh/Bbox.h"
#include "libHh/PMesh.h"

namespace hh {

// An edge, identified by the face f and the index j of the face corner opposite the edge, i.e., the edge between
// the face corners mod3(j + 1) and mod3(j + 2), across which lies the face _fnei[f].faces[j].
struct EdgeHandle {
  int f;
  int j;
};

// A face corner: the corner j of face f.  It is invalid (f < 0) to denote an absent corner.
struct MCorner {
  int f{-1};
  int j{0};
  [[nodiscard]] explicit operator bool() const { return f >= 0; }
  [[nodiscard]] bool operator==(const MCorner&) const = default;
};

// The mesh being simplified: an AWMesh in which removed faces and wedges are tombstoned rather than deleted, plus
// per-vertex information.
class WorkMesh : public AWMesh {
 public:
  static constexpr uchar k_vflag_removed = 1;
  static constexpr uchar k_vflag_global = 2;
  static constexpr int k_face_removed = -2;  // Value of _faces[f].wedges[0] for a removed face.

  Array<int> _someface;            // For each vertex, some live adjacent face; -1 if the vertex is removed.
  Array<uchar> _vflags;            // For each vertex, a combination of k_vflag_*.
  Array<int> _vertex_original_id;  // For each vertex, its 1-based id in the .m file (or ordinal + 1 in a .ply file).
  Array<int> _face_original_id;    // For each face, its 1-based id in the .m file (or ordinal + 1 in a .ply file).
  bool _has_rgb{false};
  bool _has_uv{false};

  [[nodiscard]] bool face_alive(int f) const { return _faces[f].wedges[0] != k_face_removed; }
  [[nodiscard]] bool vertex_alive(int v) const { return !(_vflags[v] & k_vflag_removed); }
  [[nodiscard]] int vertex(int f, int j) const { return _wedges[_faces[f].wedges[j]].vertex; }
  [[nodiscard]] int num_live_faces() const;
  [[nodiscard]] int num_live_vertices() const;
  [[nodiscard]] Bbox<float, 3> bbox() const;  // Over the live vertices.
  [[nodiscard]] bool is_boundary_vertex(int v) const { return is_boundary(v, _someface[v]); }
  [[nodiscard]] int degree(int v) const;  // Number of adjacent edges.
  // Corners, with the rotations of Mesh: ccw and clw around a vertex (invalid across a boundary), or around a face.
  [[nodiscard]] MCorner corner(int v, int f) const { return MCorner{f, get_jvf(v, f)}; }
  [[nodiscard]] int corner_vertex(MCorner c) const { return vertex(c.f, c.j); }
  [[nodiscard]] int corner_wedge(MCorner c) const { return _faces[c.f].wedges[c.j]; }
  [[nodiscard]] MCorner ccw_corner(MCorner c) const { return adjacent_corner(c, mod3(c.j + 1)); }
  [[nodiscard]] MCorner clw_corner(MCorner c) const { return adjacent_corner(c, mod3(c.j + 2)); }
  [[nodiscard]] static MCorner ccw_face_corner(MCorner c) { return MCorner{c.f, mod3(c.j + 1)}; }
  [[nodiscard]] static MCorner clw_face_corner(MCorner c) { return MCorner{c.f, mod3(c.j + 2)}; }
  // Returns the face across the edge (a, b) from face f, or -1 if f < 0 or the edge is a boundary.
  [[nodiscard]] int face_across(int f, int a, int b) const;
  // Returns true if the vertex v has more than one wedge.
  [[nodiscard]] bool vertex_has_multiple_wedges(int v) const;

 private:
  // The corner of the same vertex in the face across the edge opposite corner jopp of c.f.
  [[nodiscard]] MCorner adjacent_corner(MCorner c, int jopp) const {
    const int fn = _fnei[c.f].faces[jopp];
    return fn == k_undefined ? MCorner{} : corner(corner_vertex(c), fn);
  }
};

// Fill _fnei and _someface from _faces (whose wedges must already refer to vertices).  Die on a non-manifold edge,
// on an inconsistently oriented pair of faces, or on a vertex whose faces do not form a single fan.  Vertices
// referenced by no face are marked removed, with a warning.
void build_adjacency(WorkMesh& mesh);

// Write the live mesh in the .m format, as MeshSimplify's write_mesh() does.
void write_mesh_m(std::ostream& os, const WorkMesh& mesh);

// Returns the live part of the mesh as a compact AWMesh: vertices ordered by original id, wedge i being the first
// wedge of vertex i (for i < num_vertices) and the additional wedges following, and faces in their current order.
[[nodiscard]] AWMesh compact_awmesh(const WorkMesh& mesh);

// Returns a PMeshInfo describing the live mesh as the full mesh of a progressive mesh (with no vsplits).
[[nodiscard]] PMeshInfo full_mesh_info(const WorkMesh& mesh);

}  // namespace hh

#endif  // MESH_PROCESSING_PROGS_MESHTOPM_WORKMESH_H_
