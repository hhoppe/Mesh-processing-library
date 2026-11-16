// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_PROGS_MESHTOPM_PMBUILD_H_
#define MESH_PROCESSING_PROGS_MESHTOPM_PMBUILD_H_

#include "libHh/PMesh.h"
#include "progs/MeshToPm/EcolEval.h"
#include "progs/MeshToPm/WorkMesh.h"

namespace hh {

// A vsplit in compact form: for a half-edge collapse, vad_small and the residuals are zero, and its wedge attribute
// deltas are stored contiguously in PmRecorder::wads.
struct CompactVsplit {
  int flclw;    // An AWMesh face index until finalize_pm(), then the index in the progressive mesh.
  int wad_end;  // The end of its deltas in PmRecorder::wads (they start at the wad_end of the previous record).
  Vector vad_large;
  short vlr_offset1;
  ushort code;  // Its VSINDEX bits hold the AWMesh corner of vs in face flclw until finalize_pm().
  ushort fl_matid, fr_matid;
};

// Records the vsplits that undo the edge collapses, once the recording has started (at the first '-simplify').
struct PmRecorder {
  bool started{false};                       // The full mesh of the progressive mesh has been determined.
  PMeshInfo info;                            // Describes the full mesh (set when started).
  int num_faces_at_start{0};                 // Number of faces of the full mesh.
  int num_faces_removed{0};                  // Number of faces removed by the recorded collapses so far.
  Array<CompactVsplit> vsplits;              // In collapse order (the reverse of the progressive mesh order).
  Array<PmWedgeAttribD> wads;                // The wedge attribute deltas of all the vsplits.
  Array<int> removed_original_ids;           // For each recorded collapse, the original id of vt.
  [[nodiscard]] Vsplit vsplit(int i) const;  // Returns the full form of vsplits[i].
};

// Start the recording with the current live mesh as the full mesh.
void start_recording(const WorkMesh& mesh, PmRecorder& recorder);

// Perform the collapse.  If the recording has started, append the vsplit that undoes it.
void collapse(WorkMesh& mesh, const EcolConfig& config, PmRecorder& recorder);

// Finish the recording: set the base mesh (the live mesh), the information, and the flclw and vs_index of the
// recorded vsplits.
void finalize_pm(const WorkMesh& mesh, PmRecorder& recorder, AWMesh& base_mesh, PMeshInfo& info);

// Write the progressive mesh, generating its vsplits from the (finalized) recorder.
void write_pm(std::ostream& os, const AWMesh& base_mesh, const PMeshInfo& info, const PmRecorder& recorder);

// Fill the (empty) progressive mesh from the (finalized) recorder.
void make_pmesh(const AWMesh& base_mesh, const PMeshInfo& info, const PmRecorder& recorder, PMesh& pmesh);

// Returns the original ids of the vertices of the progressive mesh, in its vertex order: those of the base mesh,
// then those created by the vsplits.
[[nodiscard]] Array<int> pm_original_ids(const WorkMesh& mesh, const PmRecorder& recorder);

}  // namespace hh

#endif  // MESH_PROCESSING_PROGS_MESHTOPM_PMBUILD_H_
