// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_G3DOGL_SCGEOMORPH_H_
#define MESH_PROCESSING_G3DOGL_SCGEOMORPH_H_

#include "G3dOGL/SimplicialComplex.h"
#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/Map.h"

namespace hh {

class ScGeomorph : noncopyable {
 public:
  void clear();
  void read(std::istream& is);
  void update(float alpha, ArrayView<Vector> corner_nors);
  SimplicialComplex& getK() { return K; }

 private:
  void vertSmoothNormal(Simplex vs, Simplex corner_fct, Vector& avg_norm, bool skip_degenerate = false);
  int degenerate(const Vec3<Simplex>& verts);

  SimplicialComplex K;

  // Positions.
  Array<Point> vold;
  Array<Point> vnew;

  // Areas.
  Map<Simplex, float> anew;
  Map<Simplex, float> aold;

  // Materials.
  Map<Simplex, int> mold;

  // Normals.
  Array<Vector> nold;
  Array<Vector> nnew;

  Array<Vector> fct_pnor;
  Array<int> s_norgroup;
};

inline void ScGeomorph::clear() {
  K.clear();
  vold.clear();
  vnew.clear();

  nnew.clear();
  nold.clear();

  fct_pnor.clear();
  s_norgroup.clear();

  anew.clear();
  aold.clear();
  mold.clear();
}

}  // namespace hh

#endif  // MESH_PROCESSING_G3DOGL_SCGEOMORPH_H_
