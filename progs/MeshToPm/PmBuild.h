// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_PROGS_MESHTOPM_PMBUILD_H_
#define MESH_PROCESSING_PROGS_MESHTOPM_PMBUILD_H_

#include "libHh/PMesh.h"
#include "progs/MeshToPm/EcolEval.h"
#include "progs/MeshToPm/WorkMesh.h"

namespace hh {

// Records the vsplits that undo the edge collapses, once the recording has started (at the first '-simplify').
struct PmRecorder {
  bool started{false};              // The full mesh of the progressive mesh has been determined.
  PMeshInfo info;                   // Describes the full mesh (set when started).
  int num_faces_at_start{0};        // Number of faces of the full mesh.
  int num_faces_removed{0};         // Number of faces removed by the recorded collapses so far.
  Array<Vsplit> vsplits;            // In collapse order; flclw is an AWMesh face index until finalize_pm().
  Array<int> flclw_corner;          // For each vsplit, the AWMesh corner index of vs in face flclw.
  Array<int> removed_original_ids;  // For each recorded collapse, the original id of vt.
};

// Start the recording with the current live mesh as the full mesh.
void start_recording(const WorkMesh& mesh, PmRecorder& recorder);

// Perform the collapse.  If the recording has started, append the vsplit that undoes it.
void collapse(WorkMesh& mesh, const EcolConfig& config, PmRecorder& recorder);

// Fill the (empty) progressive mesh: its base mesh is the live mesh, and its vsplits are the recorded ones (which
// are moved from the recorder).
void finalize_pm(const WorkMesh& mesh, PmRecorder& recorder, PMesh& pmesh);

// Returns the original ids of the vertices of the progressive mesh, in its vertex order: those of the base mesh,
// then those created by the vsplits.
[[nodiscard]] Array<int> pm_original_ids(const WorkMesh& mesh, const PmRecorder& recorder);

}  // namespace hh

#endif  // MESH_PROCESSING_PROGS_MESHTOPM_PMBUILD_H_
