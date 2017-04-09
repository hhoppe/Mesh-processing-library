// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Homogeneous.h"
using namespace hh;

int main() {
    {
        Point p(1.f, 2.f, 3.f), q(8.f, 7.f, 6.f);
        Vector v(1.f, 2.f, 3.f), w(1.f, 0.f, 1.f), x(0.f, 1.f, 0.f), y = x;
        SHOW(p, q, v, w, y);
        SHOW(Homogeneous(p)+Homogeneous(q));
        SHOW(to_Point((p+p*2.f+p)/4.f));
        SHOW(to_Point((p+p)/2.f));
        SHOW(to_Point((p*2.f)/2.f));
        SHOW(to_Point((p+q)/2.f));
    }
    {
        const Homogeneous h1(1.f, 2.f, 3.f, 4.f); dummy_use(h1);
        const Homogeneous h2(h1); dummy_use(h2);
        const Vec4<float> v(0.f, 0.f, 0.f, 0.f); dummy_use(v);
#if defined(_MSC_VER) && _MSC_VER<1910
#else
        const Homogeneous h3 = v; dummy_use(h3);
        const Homogeneous h4 = V(0.f, 0.f, 0.f, 0.f); dummy_use(h4);
#endif
    }
}
