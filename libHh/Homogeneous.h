// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HOMOGENEOUS_H_
#define MESH_PROCESSING_LIBHH_HOMOGENEOUS_H_

#include "Geometry.h"

namespace hh {

// A Homogeneous vector is a 4D entity, with h[3]==0.f for a Vector, and h[3]==1.f for a Point.
struct Homogeneous : Vec4<float> {
    Homogeneous()                                               : Homogeneous(0.f, 0.f, 0.f, 0.f) { }
    constexpr Homogeneous(float x, float y, float z, float w)   : Vec4<float>(x, y, z, w) { }
    constexpr Homogeneous(const Point& p)                       : Homogeneous(p[0], p[1], p[2], 1.f) { }
    constexpr Homogeneous(const Vector& v)                      : Homogeneous(v[0], v[1], v[2], 0.f) { }
    constexpr Homogeneous(Vec4<float> h)                        : Vec4<float>(std::move(h)) { }
};

Point to_Point(const Homogeneous& h);
Vector to_Vector(const Homogeneous& h);
inline Homogeneous& operator+=(Homogeneous& h, const Point& p) { h += Homogeneous(p); return h; }
inline Homogeneous normalized(const Homogeneous& h)            { return h/assertx(h[3]); }

//----------------------------------------------------------------------------

inline Point to_Point(const Homogeneous& h) {
    const float tolerance = 1e-5f;
    if (abs(h[3]-1.f)>=tolerance) assertnever(make_string(h) + " not a Point");
    return Point(h[0], h[1], h[2]);
}

inline Vector to_Vector(const Homogeneous& h) {
    const float tolerance = 1e-5f;
    if (abs(h[3])>=tolerance) assertnever(make_string(h) + " not a Vector");
    return Vector(h[0], h[1], h[2]);
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_HOMOGENEOUS_H_
