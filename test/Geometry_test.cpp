// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Geometry.h"

#include "libHh/MatrixOp.h"
#include "libHh/RangeOp.h"
#include "libHh/SGrid.h"
using namespace hh;

namespace {

template <typename Range, typename = enable_if_range_t<Range>> Range truncate_small_floats(Range&& range) {
  using T = range_value_t<Range>;
  static_assert(std::is_floating_point_v<T>, "range must contain elements of type float/double");
  for (auto& e : range)
    if (abs(e) < 1e-6f) e = 0.f;
  return std::forward<Range>(range);
}

}  // namespace

int main() {
  {
    Point p(1.f, 2.f, 3.f), q(8.f, 7.f, 6.f);
    Vector v(1.f, 2.f, 3.f), w(1.f, 0.f, 1.f), x(0.f, 1.f, 0.f), y = x;
    SHOW(p, q, v, w, x, y);
    p += 2.f * w - x;
    SHOW(p);
    x += (q - p) * 3.f + w;
    SHOW(x);
    p = Point(1.f, 2.f, 3.f);
    SHOW(p);
    q = Point(8.f, 7.f, 6.f);
    SHOW(q);
  }
  {
    Point o(5.f, 4.f, 3.f);
    Vector v1(0.f, 0.f, 2.f), v2(3.f, 0.f, 0.f), v3(0.f, 1.f, 1.f);
    Point p(2.f, 3.f, 4.f), q;
    Frame f(v1, v2, v3, o);
    Frame f2 = Frame::identity();
    SHOW(f2);
    f2 = f;
    Frame f3 = f2;
    SHOW(f);
    q = p * f;
    SHOW(f[1][0]);
    SHOW(f.p()[1]);
    SHOW(f.p());
    SHOW(q);
    // SHOW(~f2);
    Frame f2inv = ~f2;
    truncate_small_floats(f2inv);
    SHOW(f2inv);
    SHOW(f2 * f2);
    SHOW(q * inverse(f2));
    Frame ff = f3 * f * ~f2 * Frame::identity() * ~Frame::identity();
    truncate_small_floats(ff);
    SHOW(ff);
    Frame zero = Frame::scaling(thrice(0.f));
    SHOW(zero);
    SHOW(Frame::identity());
    SHOW(dot(v1, v2));
    SHOW(dot(v1, v3));
  }
  {
    Frame f(Vector(0.f, 0.f, 2.f), Vector(3.f, 0.f, 0.f), Vector(0.f, 1.f, 1.f), Point(5.f, 4.f, 3.f));
    SHOW(f);
    SGrid<float, 4, 4> hf = to_Matrix(f);
    SHOW(hf);
    hf[1][3] = 4.f;
    hf[3][3] = 0.f;
    SHOW(hf);
    // Point p(2.f, 3.f, 4.f);
    Array<float> p{2.f, 3.f, 4.f, 1.f};
    SHOW(p);
    SHOW(mat_mul(p, hf.view()));
  }
  {
    constexpr Vector v1(1.f, 2.f, 3.f), v2(4.f, 5.f, 3.f);
    constexpr float d = dot(v1, v2);
    SHOW(d);
    const float m = mag(v1);
    SHOW(m);
    constexpr Vector vcross = cross(v1, v2);
    SHOW(vcross);
    const Frame frame(v1, v1, v2, Point(10.f, 10.f, 10.f));
    const Point origin = frame.p();
    SHOW(origin);
  }
  {
    constexpr auto vdeg = to_deg(D_TAU / 5);
    SHOW(vdeg);
  }
  {
    Point p(1.f, 2.f, 3.f), q(8.f, 7.f, 6.f);
    SHOW(dist2(p, q));
    SHOW(dist(p, q));
    SHOW(dist2(p, (p + q) / 2.f));
    SHOW(dist(p, (p + q) / 2.f));
    SHOW(dist(p + (q - p), p));
    SHOW(dist2((p + q) / 2.f, (p + q) / 2.f));
    SHOW(mag2(p - q));
    SHOW(mag2(p - p));
  }
}
