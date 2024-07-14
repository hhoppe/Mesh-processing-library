// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef SPHEREPARAM_SPHEREMAPPER_H_
#define SPHEREPARAM_SPHEREMAPPER_H_

#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/PMesh.h"

namespace hh {

// Compute a stretch-minimizing spherical parameterization of a triangle mesh,
// i.e., an assignment of spherical coordinates `sph` to the mesh vertices that minimizes a stretch metric over
// the map from the spherical triangles to the surface mesh triangles.  See `compute()`.
class SphereMapper {
 public:
  struct Options {
    int verbose{1};                   // 0=quiet; 1=default; 2=more.
    int effort{2};                    // Level (0..5) of thoroughness in optimization (slower but more accurate).
    bool visualize{false};            // Launch a piped process to visualize progress of spherical parameterization.
    bool fix_base{false};             // Never update the sph values of the base mesh vertices.
    bool optimize_inverse{false};     // Minimize inverse stretch (from sph to surface) instead of regular stretch.
    float conformal_weight{1e-4f};    // Small amount of inverse stretch for regularization.
    float hole_weight{1e-5f};         // Weight for faces marked with "hole" string.
    bool respect_sharp_edges{false};  // Optimize only tangentially along wedge discontinuities.
    bool flatten_to_x0{false};        // Override surface coordinate x = 0 (for flat octahedron domain surface).

    // The level of CPU parallelism can be overridden by setting OMP_NUM_THREADS (e.g. 1 or 24).

    // In addition, we can fine-tune the optimization parameters using the following environment variables:
    HH_IGNORE(spheremapper_optim_line_search_iter);  // Max number of iteration in Brent 1D line search.
    HH_IGNORE(spheremapper_optim_vsplit_vt_iter);    // Optimization iterations on new vsplit vertex vt.
    HH_IGNORE(spheremapper_optim_vsplit_nei_iter);   // Optimization iterations on vsplit neighbor vertices.
    HH_IGNORE(spheremapper_optim_global_iter);       // Num passes over all vertices in each global optimization.
    HH_IGNORE(spheremapper_optim_movetol);           // In global optimization, ignore vertex if mag(last_move) < tol.
    HH_IGNORE(spheremapper_optim_nv_ratio);          // Series factor on #vertices at which to globally optimize.
    // e.g.: spheremapper_optim_movetol=.02 SphereParam ...
  };

  // The surface mesh is represented as a progressive mesh.  The progressive mesh iterator `pmi` must be
  // initialized to the base mesh of the PM sequence.
  SphereMapper(PMeshIter& pmi, Options options);
  ~SphereMapper();

  // Print a summary of the optimization parameters associated with the requested `options.effort`.
  void show_parameters() const;

  // Given an initial spherical parameterization `base_sphmap` (array of 3D sphere points) of the base mesh vertices,
  // perform a coarse-to-fine optimization to determine the spherical parameterizations of all vertices of the
  // fully refined mesh in the PM sequence (returned as a constant view on an array of 3D sphere points).
  // As a side effect, the progressive mesh iterator `pmi` is advanced to the full-resolution mesh.
  CArrayView<Point> compute(CArrayView<Point> base_sphmap);

  // Print out stretch statistics on the computed spherical parameterization.
  void show_total_stretch();

  // Determine a 3D rotation matrix that, when applied to the spherical coordinates, brings them into
  // (approximate) alignment with the vertex normals of the surface mesh.
  Frame frame_aligning_sphmap_to_surface_normals() const;

 private:
  class Implementation;
  unique_ptr<Implementation> _impl;
};

}  // namespace hh

#endif  // SPHEREPARAM_SPHEREMAPPER_H_
