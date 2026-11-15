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
  static constexpr uchar k_vflag_boundary = 4;  // Set by build_adjacency() and maintained by collapses.
  static constexpr int k_face_removed = -2;     // Value of _faces[f].wedges[0] for a removed face.

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
  [[nodiscard]] bool is_boundary_vertex(int v) const { return _vflags[v] & k_vflag_boundary; }
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

  // As AWMesh::ccw_faces() and ccw_vertices(), but without the clw walk to find the start for an interior vertex.
  struct WVF_sentinel {
    int _stopf;
  };
  struct WVF_iterator {
    using iterator_concept = std::forward_iterator_tag;
    using value_type = int;
    using difference_type = std::ptrdiff_t;
    const WorkMesh* _mesh{};
    int _v{}, _ff{k_undefined};
    bool _beg{};
    [[nodiscard]] bool operator==(const WVF_iterator& rhs) const { return _ff == rhs._ff && _beg == rhs._beg; }
    [[nodiscard]] bool operator==(WVF_sentinel s) const { return !_beg && _ff == s._stopf; }
    [[nodiscard]] int operator*() const noexcept { return _ff; }
    WVF_iterator& operator++() {
      _beg = false;
      _ff = _mesh->_fnei[_ff].faces[mod3(_mesh->get_jvf(_v, _ff) + 1)];
      return *this;
    }
    WVF_iterator operator++(int) {
      auto t = *this;
      ++*this;
      return t;
    }
  };
  struct WVF_range : std::ranges::view_interface<WVF_range> {
    const WorkMesh* _mesh;
    int _v, _startf, _stopf;
    [[nodiscard]] WVF_iterator begin() const noexcept { return WVF_iterator{_mesh, _v, _startf, true}; }
    [[nodiscard]] WVF_sentinel end() const noexcept { return WVF_sentinel{_stopf}; }
  };
  [[nodiscard]] int start_face(int v, int f) const {
    if (!(_vflags[v] & k_vflag_boundary)) return f;
    int ff = f, lastf;
    do {
      lastf = ff;
      ff = _fnei[ff].faces[mod3(get_jvf(v, ff) + 2)];
    } while (ff != k_undefined && ff != f);
    return lastf;
  }
  [[nodiscard]] WVF_range ccw_faces(int v, int f) const {
    const bool bnd = _vflags[v] & k_vflag_boundary;
    return WVF_range{{}, this, v, start_face(v, f), bnd ? k_undefined : f};
  }
  struct WVV_iterator {
    using iterator_concept = std::forward_iterator_tag;
    using value_type = std::pair<int, int>;
    using difference_type = std::ptrdiff_t;
    WVV_iterator() = default;
    WVV_iterator(const WorkMesh& mesh, int v, int ff) : _mesh(&mesh), _v(v), _ff(ff) {
      _j = mesh.get_jvf(v, ff);
      _vv = mesh._wedges[mesh._faces[ff].wedges[mod3(_j + 1)]].vertex;
      _stopv = _vv;
      _nextv = mesh._wedges[mesh._faces[ff].wedges[mod3(_j + 2)]].vertex;
    }
    [[nodiscard]] bool operator==(const WVV_iterator& rhs) const { return _vv == rhs._vv && _ff == rhs._ff; }
    [[nodiscard]] bool operator==(std::default_sentinel_t) const { return _vv == k_undefined; }
    [[nodiscard]] std::pair<int, int> operator*() const noexcept { return std::pair{_vv, _ff}; }
    WVV_iterator& operator++() {
      _vv = _nextv;
      const int lastf = _ff;
      _ff = _mesh->_fnei[_ff].faces[mod3(_j + 1)];
      if (_ff == k_undefined) {
        _nextv = k_undefined;
        _ff = lastf;
      } else {
        _j = _mesh->get_jvf(_v, _ff);
        _nextv = _mesh->_wedges[_mesh->_faces[_ff].wedges[mod3(_j + 2)]].vertex;
        if (_nextv == _stopv) _nextv = k_undefined;
      }
      return *this;
    }
    WVV_iterator operator++(int) {
      auto t = *this;
      ++*this;
      return t;
    }
    const WorkMesh* _mesh{};
    int _v{}, _ff{k_undefined}, _j{}, _vv{k_undefined}, _nextv{k_undefined}, _stopv{k_undefined};
  };
  struct WVV_range : std::ranges::view_interface<WVV_range> {
    const WorkMesh* _mesh;
    int _v, _startf;
    [[nodiscard]] WVV_iterator begin() const noexcept { return WVV_iterator(*_mesh, _v, _startf); }
    [[nodiscard]] std::default_sentinel_t end() const noexcept { return {}; }
  };
  [[nodiscard]] WVV_range ccw_vertices(int v, int f) const { return WVV_range{{}, this, v, start_face(v, f)}; }

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
