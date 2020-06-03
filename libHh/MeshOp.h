// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MESHOP_H_
#define MESH_PROCESSING_LIBHH_MESHOP_H_

#include "GMesh.h"
#include "Map.h"
#include "Polygon.h"
#include "Queue.h"
#include "Set.h"
#include "Stat.h"

namespace hh {

// *** Misc

// Given a mesh boundary edge, follow the boundary to construct a closed loop of edges.
Queue<Edge> gather_boundary(const Mesh& mesh, Edge e);

// Given a mesh face, find all faces conected through Face-Edge-Face connections.
Set<Face> gather_component(const Mesh& mesh, Face f);

// Given a mesh face, find all faces connected through vertices (at least as large as gather_component).
Set<Face> gather_component_v(const Mesh& mesh, Face f);

// Return statistics on number of edges in each gather_boundary() loop.
Stat mesh_stat_boundaries(const Mesh& mesh);

// Return statistics on number of faces in each gather_component() group.
Stat mesh_stat_components(const Mesh& mesh);

// Return genus value (accounting for number of components and boundaries)
float mesh_genus(const Mesh& mesh);

// Return string giving basic topological characteristics of mesh.
string mesh_genus_string(const Mesh& mesh);

// For faces with > 3 sides, find a good triangulation of the vertices.
// Return: success (may fail if some edges already exist).
bool triangulate_face(GMesh& mesh, Face f);

// ret: cosf of signed angle away from "flattness" (== exterior angle)
// range -1..1  (or -2 if a triangle is degenerate)
// For non-triangles, looks at average of immediate neighbors on either side.
float edge_dihedral_angle_cos(const GMesh& mesh, Edge e);

// ret: -10.f if degenerate
float edge_signed_dihedral_angle(const GMesh& mesh, Edge e);

// Must be a nice interior vertex.
float vertex_solid_angle(const GMesh& mesh, Vertex v);

// Return a criterion given an edge e that is low if the edge should be collapsed.
// Use the product of the edge length with the smallest inscribed
// radius of the two adjacent faces (its dimension is area).
float collapse_edge_inscribed_criterion(const GMesh& mesh, Edge e);

// Return change in volume, but penalize bad dihedral angles.
float collapse_edge_volume_criterion(const GMesh& mesh, Edge e);

// Return memoryless QEM and penalize bad dihedral angles.
float collapse_edge_qem_criterion(const GMesh& mesh, Edge e);

// Fill the one or more holes associated with the boundary loop containing erep.  Return the new multi-sided faces.
Set<Face> mesh_remove_boundary(Mesh& mesh, Edge erep);

// *** Retriangulate

using EDGEF = bool (*)(const GMesh& m, Edge e);

// For all Mesh Edge e,
//  if dihedral angle cosf of faces both before and after is > mincos, and if (fdoswap(e)) then
//    call fdel(e), swap the edge, and call fadd(newedge).
// Consider all affect edges again.
// Return number of edges swapped.
int retriangulate_all(GMesh& mesh, float mincos, EDGEF fdoswap, EDGEF fdel = nullptr, EDGEF fadd = nullptr);

// Consider only Edge e and recursively, all affected edges.  (e cannot be boundary edge!)
// Return number of edges swapped.
int retriangulate_from_edge(GMesh& mesh, Edge e, float mincos, EDGEF fdoswap, EDGEF fdel = nullptr,
                            EDGEF fadd = nullptr);

// Consider swapping Edge e. (e cannot be boundary edge!)
// Return number of edges swapped (0 or 1).
int retriangulate_one_edge(GMesh& mesh, Edge e, float mincos, EDGEF fdoswap, EDGEF fdel = nullptr,
                           EDGEF fadd = nullptr);

// Two EDGEF functions to determine which edges to swap in retriangulate functions above.
bool circum_radius_swap_criterion(const GMesh& mesh, Edge e);
bool diagonal_distance_swap_criterion(const GMesh& mesh, Edge e);

// *** Normal estimation

// Compute the normal(s) of the corners of faces around vertex v; output into vnors.
// Normals are obtained as follows:
//  EType::unspecified: look at environment variables to pick scheme
//  EType::angle: sum normals of adjacent faces, weighted by subtended angle
//  EType::sum: sum normals of adjacent faces
//  EType::area: sum normals of adjacent faces, weighted by area of faces
//  EType::sloan: sum normals of adjacent faces, weighted by inverse squared area
//  EType::subdiv: from the limit surface of the SIG94 piecewise smooth scheme
// If string(v) contains normal information, use that instead.
class Vnors {
 public:
  Vnors() = default;
  Vnors(Vnors&& v) : _mfnor(std::move(v._mfnor)), _nor(v._nor) {}  // = default
  enum class EType { unspecified, angle, sum, area, sloan, subdiv };
  void compute(const GMesh& mesh, Vertex v, EType nortype = EType::unspecified);
  bool is_unique() const { return !_mfnor; }
  const Vector& unique_nor() const { return (ASSERTX(is_unique()), _nor); }
  const Vector& face_nor(Face f) const { return (ASSERTX(!is_unique()), _mfnor->get(f)); }
  const Vector& get_nor(Face f) const { return _mfnor ? _mfnor->get(f) : _nor; }  // in any case
  void clear();

 private:
  unique_ptr<Map<Face, Vector>> _mfnor;  // if !_mfnor, common normal is stored in _nor
  Vector _nor;
  Polygon _tmp_poly;
};

// *** Projection onto mesh

// If fast != 0 and point p projects within interior of face and edges of face are not sharp,
//   do not consider neighboring faces.
float project_point_neighb(const GMesh& mesh, const Point& p, Face& pf, Bary& ret_bary, Point& ret_clp, bool fast);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MESHOP_H_
