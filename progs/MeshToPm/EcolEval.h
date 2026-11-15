// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_PROGS_MESHTOPM_ECOLEVAL_H_
#define MESH_PROCESSING_PROGS_MESHTOPM_ECOLEVAL_H_

#include "libHh/Array.h"
#include "progs/MeshToPm/WorkMesh.h"

namespace hh {

// Parameters of the simplification, with the defaults of MeshSimplify.
struct EcolParams {
  float colfac{.1f};              // Weight of color in the quadric (fraction of the diameter).
  float norfac{.02f};             // Weight of normals in the quadric (fraction of the diameter).
  float bndfac{0.f};              // Encourage boundary edge removal.
  float trishapeafac{0.f};        // Penalize the aspect ratio of triangles times area.
  int maxvalence{32};             // Maximum vertex valence resulting from a collapse.
  int strict_sharp{0};            // Disallow changes in the topology of discontinuity curves (0, 1, 2).
  int min_chart_bnd_vertices{0};  // With strict_sharp 2, keep at least this number of boundary vertices per chart.
  bool dihallow{false};           // Penalize but allow bad dihedral angles.
  bool keepglobalv{false};        // Do not remove vertices tagged global.
  // Set by the program with min_chart_bnd_vertices: the number of boundary vertices of each chart (material).
  const Array<int>* chart_num_boundary_vertices{nullptr};
  float diam{0.f};  // Set by the program: the largest side of the bounding box of the original mesh.
};

// A legal half-edge collapse: the vertex vt merges into vs, which keeps its position.
struct EcolConfig {
  int ii{2};                           // As in Vsplit; always 2 (the position of vs is kept).
  int vs{-1}, vt{-1}, vl{-1}, vr{-1};  // Here vr == -1 on a boundary edge.
  int fl{-1}, fr{-1};                  // fl = (vs, vt, vl); fr = (vs, vr, vt), or -1 on a boundary edge.
  // The new wedge of each corner (3 * f + j) of the faces adjacent to vs or vt other than fl and fr, as assigned by
  // MeshSimplify's replace_wi(); it is a wedge of vs or of vt.
  InlinedArray<std::pair<int, int>, 24> corner_wedges;
  // The new attributes of the wedges that the corners of corner_wedges refer to.
  InlinedArray<std::pair<int, PmWedgeAttrib>, 4> wedge_attribs;
};

struct EcolResult {
  float cost{BIGFLOAT};  // BIGFLOAT if the collapse is illegal.
  EcolConfig config;     // Defined if cost < BIGFLOAT.
};

// Evaluate the collapse of edge e, as MeshSimplify's try_ecol() does with "-minqem -vsgeom".
[[nodiscard]] EcolResult evaluate_ecol(const WorkMesh& mesh, EdgeHandle e, const EcolParams& params);

// The hh99 quadric of the face with corner vectors p0, p1, p2 (in R^n, n = 3, 6, or 9: a position followed by
// attributes), evaluated at x, as Qem<double, n>::set_distance_hh99() and evaluate() compute it: the squared distance
// of the position from the face plane plus, for each attribute, the squared difference from its linear interpolation
// over the plane.  Computed in double, and unweighted.
[[nodiscard]] double hh99_face_distance2(CArrayView<float> p0, CArrayView<float> p1, CArrayView<float> p2,
                                         CArrayView<float> x);

// Returns true if edge e is a boundary edge, separates different materials, or has a wedge discontinuity.
[[nodiscard]] bool edge_sharp(const WorkMesh& mesh, EdgeHandle e);

// Returns the charts (materials) whose boundary contains vertex v, i.e., all its adjacent charts if it is adjacent to
// two or more charts or if it lies on the mesh boundary.
[[nodiscard]] InlinedArray<int, 8> charts_with_boundary_vertex(const WorkMesh& mesh, int v);

// Returns true if face f and its neighbors (-1 if absent) have a legal configuration of materials, as in
// MeshSimplify's strict_mat_neighbors().
[[nodiscard]] bool strict_mat_neighbors(const WorkMesh& mesh, int f, const Vec3<int>& fnei);

// Returns the number of sharp edges adjacent to vertex v.
[[nodiscard]] int vertex_num_sharp_edges(const WorkMesh& mesh, int v);

}  // namespace hh

#endif  // MESH_PROCESSING_PROGS_MESHTOPM_ECOLEVAL_H_
