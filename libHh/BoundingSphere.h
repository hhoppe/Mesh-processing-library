// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BOUNDINGSPHERE_H_
#define MESH_PROCESSING_LIBHH_BOUNDINGSPHERE_H_

#include "Geometry.h"

namespace hh {

struct BoundingSphere {
    Point point;
    float radius;
    friend std::ostream& operator<<(std::ostream& os, const BoundingSphere& bsphere) {
        return os << "BoundingSphere(point=" << bsphere.point << ", radius=" << bsphere.radius << ")";
    }
};

inline BoundingSphere bsphere_union(const BoundingSphere& bsphere1, const BoundingSphere& bsphere2) {
    float d = dist(bsphere1.point, bsphere2.point);
    if (bsphere1.radius>=d+bsphere2.radius) {
        return bsphere1;
    } else if (bsphere2.radius>=d+bsphere1.radius) {
        return bsphere2;
    } else {
        // (from above, d obviously cannot be zero)
        float newradius = (bsphere1.radius+bsphere2.radius+d)*0.5f;
        // point==interp(bsphere1.point, bsphere2.point, b1) where r1+(1-b1)*d==r2+(b1)*d
        // therefore b1 = ((r1-r2)/d+1)/2
        float b1 = ((bsphere1.radius-bsphere2.radius)/d+1.f)*0.5f;
        ASSERTX(b1>=0.f && b1<=1.f);
        BoundingSphere bsphere;
        bsphere.point = interp(bsphere1.point, bsphere2.point, b1);
        bsphere.radius = newradius;
        return bsphere;
    }
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_BOUNDINGSPHERE_H_
