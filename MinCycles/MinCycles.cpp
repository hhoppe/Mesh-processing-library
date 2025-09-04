// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "MinCycles/CloseMinCycles.h"
#include "libHh/Args.h"
#include "libHh/FileIO.h"
#include "libHh/MeshOp.h"
#include "libHh/Timer.h"
using namespace hh;

int main(int argc, const char** argv) {
  CloseMinCycles::Options options;
  bool nooutput = false;
  ParseArgs args(argc, argv);
  HH_ARGSC("A mesh is read from stdin or first arg.  Subsequent options are:");
  HH_ARGSC(HH_ARGS_INDENT "Criteria for stopping topological simplification:");
  HH_ARGSP_O(max_cycle_length, "len : when smallest cycle exceeds specified length");
  HH_ARGSP_O(max_cycle_nedges, "n : when smallest cycle has > n edges");
  HH_ARGSP_O(num_cycles, "n : after removing this number of cycles");
  HH_ARGSP_O(genus, "g : when mesh genus <= g");
  HH_ARGSC(HH_ARGS_INDENT "Other topological simplification parameters:");
  HH_ARGSP_O(frac_cycle_length, "frac>=1. : allow cycles to have length greater than minimal");
  HH_ARGSP_O(mark_edges_sharp, "bool : mark loops of edges using 'sharp' key string");
  HH_ARGSP_O(mark_faces_filled, "bool : mark rings of new faces using 'filled' key string");
  HH_ARGSP_O(mark_min_num_edges, "n : only mark 'sharp'/'filled' if cycle has >= n edges");
  HH_ARGSP_O(frac_offset, "float : Interpenetrate closed cycles by a fraction of bbox");
  HH_ARGSC("", ":");
  HH_ARGSF(nooutput, ": do not write mesh at program end");
  const string arg0 = args.num() ? args.peek_string() : "";
  if (ParseArgs::special_arg(arg0)) args.parse(), exit(0);
  const string filename = args.num() && (arg0 == "-" || arg0[0] != '-') ? args.get_filename() : "-";
  RFile fi(filename);
  for (string line; fi().peek() == '#';) {
    assertx(my_getline(fi(), line));
    if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
  }
  showff("%s", args.header().c_str());
  args.parse();
  GMesh mesh;
  {
    HH_TIMER("MinCycles");
    {
      HH_TIMER("_readmesh");
      mesh.read(fi());
    }
    CloseMinCycles close_min_cycles(mesh, options);
    close_min_cycles.compute();
  }
  showdf("%s\n", mesh_genus_string(mesh).c_str());
  hh_clean_up();
  if (!nooutput) {
    HH_TIMER("_writemesh");
    mesh.write(std::cout);
  }
  if (!k_debug) exit_immediately(0);  // Skip ~GMesh().
  return 0;
}
