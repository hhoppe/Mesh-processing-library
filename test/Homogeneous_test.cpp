// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Homogeneous.h"
using namespace hh;

int main() {
  {
    const Point p(1.f, 2.f, 3.f), q(8.f, 7.f, 6.f);
    const Vector v(1.f, 2.f, 3.f), w(1.f, 0.f, 1.f), x(0.f, 1.f, 0.f), y = x;
    SHOW(p, q, v, w, y);
    SHOW(Homogeneous(p) + Homogeneous(q));
    SHOW(to_Point((Homogeneous(p) + Homogeneous(p) * 2.f + Homogeneous(p)) / 4.f));
    SHOW(to_Point((Homogeneous(p) + Homogeneous(p)) / 2.f));
    SHOW(to_Point((Homogeneous(p) * 2.f) / 2.f));
    SHOW(to_Point((Homogeneous(p) + Homogeneous(q)) / 2.f));
  }
  {
    const Homogeneous h1(1.f, 2.f, 3.f, 4.f);
    const Homogeneous h2(h1);
    assertx(h2 == V(1.f, 2.f, 3.f, 4.f));
    const Vec4<float> v(5.f, 6.f, 7.f, 8.f);
    const Homogeneous h3 = v;  // Implicit conversion from Vec4<float>.
    assertx(h3[2] == 7.f && h3[3] == 8.f);
    const Homogeneous h4 = V(0.f, 0.f, 0.f, 0.f);
    assertx(h4 == Homogeneous());  // The default constructor gives the zero vector.
    SHOW(normalized(h1));
    assertx(normalized(Homogeneous(2.f, 4.f, 6.f, 2.f)) == V(1.f, 2.f, 3.f, 1.f));
  }
  {
    // A Point has h[3] == 1.f, and a Vector has h[3] == 0.f.
    const Point p(1.f, 2.f, 3.f), q(8.f, 7.f, 6.f);
    const Vector v(1.f, -2.f, .5f);
    assertx(Homogeneous(p)[3] == 1.f && Homogeneous(v)[3] == 0.f);
    assertx(to_Point(Homogeneous(p)) == p && to_Vector(Homogeneous(v)) == v);
    // The difference of two points is a vector; the sum of a point and a vector is a point.
    SHOW(to_Vector(Homogeneous(q) - Homogeneous(p)));
    assertx(to_Vector(Homogeneous(q) - Homogeneous(p)) == q - p);
    assertx(to_Point(Homogeneous(p) + Homogeneous(v)) == p + v);
    // An affine combination (with weights summing to 1) of points is a point.
    SHOW(to_Point(Homogeneous(p) * .25f + Homogeneous(q) * .75f));
    // A sum of points has h[3] equal to the number of points, and normalizes to their centroid.
    Homogeneous h;
    for (const Point& pp : {p, q, Point(0.f, 0.f, 0.f), Point(-1.f, 3.f, 3.f)}) h += pp;
    assertx(h[3] == 4.f);
    SHOW(to_Point(normalized(h)));
    // The conversions allow a small tolerance on h[3].
    assertx(to_Point(Homogeneous(1.f, 2.f, 3.f, 1.f + 4e-6f)) == p);
    assertx(to_Vector(Homogeneous(1.f, -2.f, .5f, -4e-6f)) == v);
  }
}
