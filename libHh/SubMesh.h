// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_SUBMESH_H_
#define MESH_PROCESSING_LIBHH_SUBMESH_H_

#include "libHh/Array.h"
#include "libHh/Combination.h"
#include "libHh/GMesh.h"
#include "libHh/Homogeneous.h"

namespace hh {

// Weighted combination of vertices and a homogeneous coordinate.
struct Combvh {
  Homogeneous h;  // placed first because may need 16-alignment if Vector4
  Combination<Vertex> c;
  bool is_combination() const;
  Point evaluate(const GMesh& mesh) const;
  HH_POOL_ALLOCATION(Combvh);
};

HH_INITIALIZE_POOL(Combvh);

// We previously used unique_ptr<Combvh> rather than Combvh because of alignment problems due to __m128 in Combvh::h
// See ~/git/hh_src/test/native/unordered_map_of_m128.cpp

class Mvcvh : public Map<Vertex, Combvh> {
 public:
  bool is_convolution() const;               // check combination is affine
  Combvh compose_c(const Combvh& ci) const;  // co = ci * this
  // compose two maps to produce one, die unless mconv.is_convolution()
  void compose(const Mvcvh& mconv);  // this = mconv * this
};

// Subdivide a mesh and maintain relationships between subdivided mesh and original base mesh.
class SubMesh {
 public:
  explicit SubMesh(GMesh& mesh);
  void clear();
  static const FlagMask vflag_variable;  // MVertex flag bit
  // mesh() may be modified if no more SubMesh operations will be done.
  GMesh& mesh() { return _m; }
  const GMesh& mesh() const { return _m; }
  GMesh& orig_mesh() { return _omesh; }
  const GMesh& orig_mesh() const { return _omesh; }

  // subdivide (makes use of refine(), create_conv(), convolve_self(), ...):
  void subdivide(float cosang = 1.f);
  void subdivide_n(int nsubdiv, int limit, float cosang = 1.f, bool triang = true);

  // combinations:
  // get a combination (expressing v of mesh() in terms of orig_mesh())
  const Combvh& combination(Vertex v) const;
  // Compose c1 with _cmvcvh to get combination in terms of orig. verts.
  Combvh compose_c_mvcvh(const Combvh& ci) const;

  // Update vertex positions on mesh() according to its mask:
  void update_vertex_position(Vertex v);
  void update_vertex_positions();

  // misc:
  void mask_parameters(bool ps222, float pweighta) {
    _s222 = ps222;
    _weighta = pweighta;
  }

  // omesh to and from mesh:
  Face orig_face(Face f) const;
  void orig_face_index(Face fi, Face& of, int& pindex) const;
  Face get_face(Face of, int index) const;

  // split and compute splitting masks:
  void refine(Mvcvh& mconv);  // 1to4 split at edge midpoints
  // refine near creases, and refine edges with cosdihedral <cosang
  void selectively_refine(Mvcvh& mconv, float cosang);

  // compute averaging masks:
  using FVMASK = void (SubMesh::*)(Vertex v, Combvh& comb) const;
  void create_conv(Mvcvh& mconv, FVMASK f);  // use a subdivision mask

  // the masks:
  void averaging_mask(Vertex v, Combvh& comb) const;
  void limit_mask(Vertex v, Combvh& comb) const;

  // triangulate:
  void triangulate_quads(Mvcvh& mconv);  // 1to4 split at centroids

  // apply a convolution:
  void convolve_self(const Mvcvh& mconv);  // _cmvcvh = mconv * _cmvcvh

  // debug:
  void show_mvcvh(const Mvcvh& mvcvh) const;
  void show_cmvcvh() const;

 private:
  GMesh& _omesh;
  GMesh _m;
  Mvcvh _cmvcvh;                  // maps vertices of _m to Combvh of _omesh
  Map<Face, Face> _mforigf;       // face of _m -> face of _omesh
  Map<Face, int> _mfindex;        // face of _m -> index within origf
  Map<Face, Array<Face>> _mofif;  // face of _omesh -> (int -> face of _m)
  bool _allvvar;                  // if all vertices are vflag_variable
  bool _isquad;
  //
  bool _s222{false};
  float _weighta{0.f};
  bool _selrefine{false};  // no longer used
  //
  bool sharp(Edge e) const;
  int nume(Vertex v) const;
  int num_sharp_edges(Vertex v) const;
  Edge opp_sharp_edge(Vertex v, Edge e) const;
  Vertex opp_sharp_vertex(Vertex v, Vertex v2) const;
  void subdivide_aux(float cosang, Mvcvh* mconv);
  void crease_averaging_mask(Vertex v, Combvh& comb) const;
  bool extraordinary_crease_vertex(Vertex v) const;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SUBMESH_H_
