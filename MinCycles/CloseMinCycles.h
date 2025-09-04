// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_MINCYCLES_CLOSEMINCYCLES_H_
#define MESH_PROCESSING_MINCYCLES_CLOSEMINCYCLES_H_

#include "libHh/Array.h"
#include "libHh/GMesh.h"
#include "libHh/Map.h"

namespace hh {

// Simplify the topology of a mesh by iteratively locating and closing minimal non-separating cycles.
// Options specify when to stop the topological simplification, and how precise "minimal" must be.
class CloseMinCycles {
 public:
  struct Options {
    float max_cycle_length{BIGFLOAT};                       // Maximum Euclidean length of found cycles.
    int max_cycle_nedges{std::numeric_limits<int>::max()};  // Maximum number of edges of found cycles.
    int num_cycles{std::numeric_limits<int>::max()};        // Maximum number of found cycles.
    int genus{0};                                           // Minimum genus of topologically simplified mesh.
    float frac_cycle_length{1.f};  // By default, find exact minimal cycles (> 1.f means approximate).
    bool mark_edges_sharp{true};   // Set "sharp" on closed edge cycles.
    bool mark_faces_filled{true};  // Set "filled" and "handle"/"tunnel" on faces, "filledcenter" on vertex.
    int mark_min_num_edges{0};     // Mark "sharp" and "filled" only if cycle has a minimum number of edges.
    float frac_offset = 0.f;       // If nonzero, interpenetrate the closed cycles by a fraction of the object bbox.
  };

  CloseMinCycles(GMesh& mesh, Options options);
  void compute();

 private:
  GMesh& _mesh;
  const Options _options;
  int _current_genus{std::numeric_limits<int>::max()};
  int _total_handles{0};
  int _total_tunnels{0};
  float _offset_magnitude{0.f};
  Flag e_joined(Edge e);
  void flood_reinitialize(Vertex vseed);
  Array<Vertex> close_cycle(const CArrayView<Vertex> vertex_loop);
  bool would_be_nonseparating_cycle(Edge e12, bool exact);
  std::optional<int> look_for_cycle(Vertex v1, Vertex v2, bool process, float verify_dist);  // Ret: num_edges.
  struct MinCycleResult {
    float search_radius;
    Vertex farthest_vertex;
    int num_edges;
  };
  std::optional<MinCycleResult> min_cycle_from_vertex(Vertex vseed, bool process);
  void find_cycles();
};

}  // namespace hh

#endif  // MESH_PROCESSING_MINCYCLES_CLOSEMINCYCLES_H_
