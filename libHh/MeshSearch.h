// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MESHSEARCH_H_
#define MESH_PROCESSING_LIBHH_MESHSEARCH_H_

#include <optional>

#include "libHh/Bbox.h"
#include "libHh/TriangleFaceSpatial.h"

#if 0
{
  const MeshSearch mesh_search(mesh, {true});
  Face hint_f = nullptr;
  const auto [f, bary, clp, d2] = mesh_search.search(p, hint_f);
}
#endif

namespace hh {

// Construct a spatial data structure from a mesh, to enable fast closest-point queries from arbitrary points.
// Optionally, tries to speed up the search by caching the result of the previous search and incrementally
// walking over the mesh from that prior result.
class MeshSearch {
 public:
  struct Options {
    bool allow_local_project{false};
    bool allow_internal_boundaries{false};
    bool allow_off_surface{false};
    std::optional<Bbox<float, 3>> bbox;
  };
  explicit MeshSearch(const GMesh& mesh, Options options);
  ~MeshSearch();

  struct Result {
    Face f;
    Bary bary;
    Point clp;
    float d2;
  };
  Result search(const Point& p, Face hint_f) const;

  struct ResultOnSphere {
    Face f;
    Bary bary;
  };
  ResultOnSphere search_on_sphere(const Point& p, Face hint_f, const Point* final_p = nullptr) const;

  const GMesh& mesh() const { return _mesh; }

 private:
  const GMesh& _mesh;
  Options _options;
  Array<TriangleFace> _trianglefaces;
  unique_ptr<TriangleFaceSpatial> _spatial;
  Frame _xform;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_MESHSEARCH_H_
