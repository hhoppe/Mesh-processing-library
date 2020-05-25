// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_MKLIB_H_
#define MESH_PROCESSING_LIBHH_MKLIB_H_

#include "Mk3d.h"

namespace hh {

// Create a stream of geometric shapes on a WA3dStream.
class Mklib : noncopyable {
 public:
    explicit Mklib(Mk3d& mk3d)                  : mk(mk3d) { }
    ~Mklib()                                    { assertw(_stack_smooth.empty()); }
    bool smooth()                               { return _smooth; }
    void begin_smooth(bool b)                   { _stack_smooth.push(_smooth); _smooth = b; }
    void end_smooth()                           { assertx(!_stack_smooth.empty()); _smooth = _stack_smooth.pop(); }
    void squareO();             // unit square centered at origin, facing +x axis
    void squareXY();            // unit square between (0, 0, 0) and (1, 1, 0), facing +z
    void squareU();             // square above origin, in xy plane, facing +z axis
    void cubeO();               // unit cube centered at origin
    void cubeXYZ();             // unit cube between (0, 0, 0) and (1, 1, 1)
    void cubeU();               // cube with center of bottom face at origin
    void polygonO(int n);       // regular polygon, radius 1, normal to x axis, vertex on y axis
    void polygonU(int n);       // polygon facing +z axis, vertex on +x axis
    void ringU(int n, float h, float r0, float r1, float a0, float a1);
    void flat_ringU(int n, float h, float r0, float r1); // ring with angles such that it is flat (not smooth)
    void poly_hole(int n, float r1);                     // annulus (bounded by two concentric circles)
    void volume_ringU(int n, float r1);                  // a "discrete torus" with rectangular cross section
    void tubeU(int n);                    // height 1, radius 1 open in +z axis, vertex on +x axis
    void cylinderU(int n);                // cylinder==tube with closed ends
    void capU(int n);                     // height 1, radius 1, bottom at origin, peak at (1, 0, 0)
    void coneU(int n);                    // cone==cap with closed bottom
    void sphere(int nlat, int nlong);     // radius 1, #latitudes(>=2), #longitudes(>=3)
    void hemisphere(int nlat, int nlogn); // radius 1, #latitudes(>=2), #longitudes(>=2)
    void tetra();                         // centered at centroid, edge=1 height=sqrt(2/3)
    void tetraU();                        // bottom face centroid at origin, top at (0, 0, sqrt(2/3))
    // transformation on object: (-.5, -.5, -.5)<>(.5, .5, .5) with primary axis +x, secondary axis +y ("O")
    //                        -->  (-.5, -.5, 0)<>(.5, .5, 1)  with primary axis +z, secondary axis +x ("U")
    template<typename Func = void(int)> void OtoU(Func func, int n) {
        MkSave mk_save(mk); mk.rotate(1, TAU/4); mk.rotate(0, -TAU/4); mk.translate(.5f, 0, 0); func(n);
    }
    // radius 1 along +x axis, calls func with +x axis normal to circle;  s = sin(TAU/2/n) h = cos(TAU/2/n);
    //  scaled to touch at (0, -s, 0) & (0, +s, 0) and center of circle at (-h, 0, 0).
    template<typename Func = void(int)> void circle_of(Func func, int n) {
        float a = TAU/n, h = cos(a*.5f);
        MkSave mk_save(mk); mk.rotate(1, TAU/4); mk.rotate(2, TAU/4); mk.rotate(2, a*.5f);
        for_int(i, n) {
            { MkSave mk_save2(mk); mk.translate(h, 0, 0); func(i); }
            mk.rotate(2, a);
        }
    }
    template<typename Func = void(int)> void circle_ofU(Func func, int n) {
        float a = TAU/n, h = cos(a*.5f);
        MkSave mk_save(mk); mk.rotate(2, a*.5f);
        for_int(i, n) {
            { MkSave mk_save2(mk); mk.translate(h, 0, 0); func(i); }
            mk.rotate(2, a);
        }
    }
    // radius 1 along +z axis, calls func with +x axis normal to circle;
    //  not scaled -> center of circle @(-1, 0, 0).
    template<typename Func = void(int)> void radius_ofU(Func func, int n) {
        float a = TAU/n;
        MkSave mk_save(mk);
        for_int(i, n) {
            { MkSave mk_save2(mk); mk.translate(1, 0, 0); func(i); }
            mk.rotate(2, a);
        }
    }
    // height h, 2 radii and normal angles w/respect XY plane
 public:
    Mk3d& mk;
 private:
    bool _smooth {true};
    Stack<bool> _stack_smooth;
    void gsphere(int nlat, int nlong, bool hemi);
};

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_MKLIB_H_
