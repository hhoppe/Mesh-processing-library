// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_PROGS_MESHTOPM_MESHINPUT_H_
#define MESH_PROCESSING_PROGS_MESHTOPM_MESHINPUT_H_

#include "progs/MeshToPm/WorkMesh.h"

namespace hh {

// Read a mesh from a .m or .ply file (possibly compressed as .gz; "-" denotes stdin, in .m format) into the empty
// mesh, including adjacency, normals, wedges, and materials.
void read_mesh(const string& filename, WorkMesh& mesh);

}  // namespace hh

#endif  // MESH_PROCESSING_PROGS_MESHTOPM_MESHINPUT_H_
