// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Geometry.h"
#include "RangeOp.h"

namespace hh {

// Axis-aligned bounding box in 3D.
struct Bbox : Vec2<Point> {
    Bbox()                                      = default;
    Bbox(const Point& pmin, const Point& pmax)  : Vec2<Point>(pmin, pmax) { }
    Bbox(Vec2<Point> bb)                        : Vec2<Point>(std::move(bb)) { }
    void clear()                        { auto& self = *this; self[0] = thrice(+big()); self[1] = thrice(-big()); }
    void infinite()                     { auto& self = *this; self[0] = thrice(-big()); self[1] = thrice(+big()); }
    void union_with(const Bbox& bb) {
        auto& self = *this;
        for_int(c, 3) {
            if (bb[0][c]<self[0][c]) self[0][c] = bb[0][c];
            if (bb[1][c]>self[1][c]) self[1][c] = bb[1][c];
        }
    }
    void union_with(const Point& pp) {
        auto& self = *this;
        for_int(c, 3) {
            float v = pp[c];
            if (v<self[0][c]) self[0][c] = v;
            if (v>self[1][c]) self[1][c] = v;
        }
    }
    void intersect(const Bbox& bb) {
        auto& self = *this;
        for_int(c, 3) {
            if (bb[0][c]>self[0][c]) self[0][c] = bb[0][c];
            if (bb[1][c]<self[1][c]) self[1][c] = bb[1][c];
        }
    }
    bool inside(const Bbox& bb) const {
        auto& self = *this;
        for_int(c, 3) {
            if (self[0][c]<bb[0][c]) return false;
            if (self[1][c]>bb[1][c]) return false;
        }
        return true;
    }
    bool overlap(const Bbox& bb) const {
        auto& self = *this;
        for_int(c, 3) {
            if (bb[0][c]>self[1][c] || bb[1][c]<self[0][c]) return false;
        }
        return true;
    }
    // uniform scaling into unit cube, centered on x & y, rest at z=0
    Frame get_frame_to_cube() const {
        auto& self = *this;
        Vector di = self[1]-self[0];
        float maxdi = max(di);
        assertx(maxdi);
        Vector center;
        for_int(c, 3) { center[c] = (1.f-di[c]/maxdi)*.5f; }
        center[2] = 0.f;            // objects lie at bottom of cube
        return Frame::translation(-to_Vector(self[0])) * Frame::scaling(thrice(1.f/maxdi)) *
            Frame::translation(center);
    }
    Frame get_frame_to_small_cube(float cubesize = .8f) const {
        Frame f = get_frame_to_cube();
        float bnd = (1.f-cubesize)/2.f;
        f = f*Frame::scaling(thrice(cubesize))*Frame::translation(thrice(bnd));
        for_int(i, 3) {
            if (abs(f[i][i]-1.f)<.05f) f[i][i] = 1.f;
            if (abs(f[3][i])<.05f) f[3][i] = 0.f;
        }
        return f;
    }
    float max_side() const {
        auto& self = *this;
        return max({self[1][0]-self[0][0], self[1][1]-self[0][1], self[1][2]-self[0][2]});
    }
    void transform(const Frame& frame) {
        auto& self = *this;
        Bbox bb;
        bb.clear();
        for_int(i0, 2) for_int(i1, 2) for_int(i2, 2) {
            Point corner(i0 ? self[1][0] : self[0][0], i1 ? self[1][1] : self[0][1], i2 ? self[1][2] : self[0][2]);
            bb.union_with(corner*frame);
        }
        self = bb;
    }
    friend std::ostream& operator<<(std::ostream& os, const Bbox& bb) {
        return os << "Bbox{" << bb[0] << ", " << bb[1] << "}";
    }
 private:
    static float big()                          { return BIGFLOAT; }
};

} // namespace hh
