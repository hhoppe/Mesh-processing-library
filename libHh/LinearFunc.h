// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Geometry.h"

namespace hh {

// Linear function in 3D, represented as a vector and scalar offset.
struct LinearFunc {
    Vector v;
    float offset {0.f};
    LinearFunc()                                = default;
    CONSTEXPR LinearFunc(Vector pv, const Point& pp) : v(std::move(pv)), offset(-dot(to_Vector(pp), v)) { }
    CONSTEXPR float eval(const Point& p) const  { return dot(to_Vector(p), v)+offset; }
    void add(const LinearFunc& lf)              { v += lf.v; offset += lf.offset; }
    friend std::ostream& operator<<(std::ostream& os, const LinearFunc& lf) {
        return os << "LinearFunc(v=" << lf.v << ", offset=" << lf.offset << ")";
    }
};

} // namespace hh
