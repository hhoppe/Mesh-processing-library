// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_PROGS_MESHTOPM_MESHINPUT_H_
#define MESH_PROCESSING_PROGS_MESHTOPM_MESHINPUT_H_

#include "progs/MeshToPm/WorkMesh.h"

namespace hh {

// Read the rest of a mesh file, after GMeshInput has read its header (the leading '#' lines of a .m file, or the
// given lines through "end_header" of a PLY file), into the empty mesh, including adjacency, normals, wedges, and
// materials.
void read_mesh(std::istream& is, CArrayView<string> ply_header_lines, WorkMesh& mesh);

}  // namespace hh

#endif  // MESH_PROCESSING_PROGS_MESHTOPM_MESHINPUT_H_
