// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_G3DOGL_SCGEOMORPH_H_
#define MESH_PROCESSING_G3DOGL_SCGEOMORPH_H_

#include "SimplicialComplex.h"
#include "Map.h"
#include "Geometry.h"
#include "Array.h"

namespace hh {

class SCGeomorph : noncopyable {
 public:
    void clear();
    void read(std::istream& is);
    void update(float alpha, Array<Vector>& corner_nors);
    SimplicialComplex& getK() { return K; }
    
 private:
    void vertSmoothNormal(Simplex vs, Simplex corner_fct, Vector& avg_norm,
                          bool skip_degenerate = false);
    int degenerate(Simplex verts[3]);
    
    SimplicialComplex K;

    // positions
    Array<Point> vold;
    Array<Point> vnew;

    // areas
    Map<Simplex,float> anew;
    Map<Simplex,float> aold;

    // materials
    Map<Simplex,int> mold;

    // normals
    Array<Vector> nold;
    Array<Vector> nnew;

    Array<Vector> fct_pnor;
    Array<int> s_norgroup;
};

inline void SCGeomorph::clear() {
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

} // namespace hh

#endif // MESH_PROCESSING_G3DOGL_SCGEOMORPH_H_
