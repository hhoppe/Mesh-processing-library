// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_PROGS_MESHTOPM_MESHINPUT_H_
#define MESH_PROCESSING_PROGS_MESHTOPM_MESHINPUT_H_

#include "progs/MeshToPm/WorkMesh.h"

namespace hh {

// The header of a mesh file.
struct MeshFileHeader {
  bool is_ply{false};       // The file is a PLY file (detected by its first line "ply"), else a .m file.
  Array<string> ply_lines;  // For a PLY file, its header lines, from "ply" through "end_header".
};

// Read the header of a mesh file (the leading '#' lines of a .m file, or the lines through "end_header" of a PLY
// file), and echo it as comments as Filtermesh does.
[[nodiscard]] MeshFileHeader read_mesh_header(std::istream& is);

// Read the rest of the mesh file into the empty mesh, including adjacency, normals, wedges, and materials.
void read_mesh(std::istream& is, const MeshFileHeader& header, WorkMesh& mesh);

}  // namespace hh

#endif  // MESH_PROCESSING_PROGS_MESHTOPM_MESHINPUT_H_
