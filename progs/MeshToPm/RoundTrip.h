// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_PROGS_MESHTOPM_ROUNDTRIP_H_
#define MESH_PROCESSING_PROGS_MESHTOPM_ROUNDTRIP_H_

#include "libHh/PMesh.h"
#include "progs/MeshToPm/WorkMesh.h"

namespace hh {

// A copy of the full mesh of a progressive mesh, identified by original vertex ids, for verifying its round trip.
struct RoundTripReference {
  Array<int> original_ids;  // For each vertex.
  Array<Point> points;      // For each vertex.
  struct Face {
    Vec3<int> ids;  // Original vertex ids, rotated so that the smallest is first.
    Vec3<PmWedgeAttrib> attribs;
    Vec3<int> wedges;  // Wedge indices (in the reference mesh, or in the refined progressive mesh).
    int matid;
  };
  Array<Face> faces;  // Sorted by ids.
  Materials materials;
  int num_wedges{0};  // Number of distinct wedges.
};

[[nodiscard]] RoundTripReference make_round_trip_reference(const WorkMesh& mesh);

// Refine the progressive mesh to full resolution and verify that it reproduces the reference: the same faces,
// materials, and wedge partition, and the same positions and wedge attributes within a rounding tolerance.  Prints
// a summary line starting with "roundtrip: ok", or dies.
void check_round_trip(const PMesh& pmesh, const RoundTripReference& ref, CArrayView<int> pm_original_ids);

}  // namespace hh

#endif  // MESH_PROCESSING_PROGS_MESHTOPM_ROUNDTRIP_H_
