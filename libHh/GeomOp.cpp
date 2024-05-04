// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GeomOp.h"

#include "libHh/MathOp.h"

namespace hh {

// *** Radii

// Deduced from book: Coxeter "Geometry".
float circum_radius(const Point& p0, const Point& p1, const Point& p2) {
  double a = dist<>(p0, p1), b = dist<>(p1, p2), c = dist<>(p2, p0);  // use double dist<>() rather than float dist()
  double s = (a + b + c) * .5;
  double d2 = s * (s - a) * (s - b) * (s - c);
  if (d2 <= 0.) {
    Warning("circum_radius degenerate");
    return 1e10f;
  }
  return float(a * b * c * .25 / my_sqrt(d2));
}

// From Coxeter "Intro to Geometry, Second ed.", page 12, equation  1.531
float inscribed_radius(const Point& p0, const Point& p1, const Point& p2) {
  // r = d / s
  // d = sqrt(s * (s - a) * (s - b) * (s -c))
  // s = (a + b + c) / 2
  double a = dist<>(p0, p1), b = dist<>(p1, p2), c = dist<>(p2, p0);
  double s = (a + b + c) * .5;
  double d2 = s * (s - a) * (s - b) * (s - c);
  if (d2 <= 0.) {
    Warning("inscribed_radius degenerate");
    return 0.f;
  }
  return float(sqrt(d2) / s);
}

// Should normalize to be 1.f for an equilateral triangle?
float aspect_ratio(const Point& p0, const Point& p1, const Point& p2) {
  double a = dist<double>(p0, p1), b = dist<double>(p1, p2), c = dist<double>(p2, p0);
  double s = (a + b + c) * .5;
  double d2 = s * (s - a) * (s - b) * (s - c);
  if (d2 <= 0.) {
    // Warning("aspect_ratio degenerate");
    return 1e10f;
  }
  return float(a * b * c * .25 * s / d2);
}

// *** Misc

float dihedral_angle_cos(const Point& p1, const Point& p2, const Point& po1, const Point& po2) {
#if 0
  Vector ves1 = cross(p1, p2, po1);
  Vector ves2 = cross(p1, po2, p2);
  if (!ves1.normalize() || !ves2.normalize()) return -2.f;
  float d = dot(ves1, ves2);
  if (d < -1.f) d = -1.f;
  if (d > +1.f) d = +1.f;
  return d;
#endif
  float ves1x, ves1y, ves1z;
  {
    float v1x = p2[0] - p1[0], v1y = p2[1] - p1[1], v1z = p2[2] - p1[2];
    float v2x = po1[0] - p1[0], v2y = po1[1] - p1[1], v2z = po1[2] - p1[2];
    ves1x = v1y * v2z - v1z * v2y;
    ves1y = v1z * v2x - v1x * v2z;
    ves1z = v1x * v2y - v1y * v2x;
    float sum2 = ves1x * ves1x + ves1y * ves1y + ves1z * ves1z;
    if (!sum2) return -2.f;
    float fac = 1.f / sqrt(sum2);
    ves1x *= fac;
    ves1y *= fac;
    ves1z *= fac;
  }
  float ves2x, ves2y, ves2z;
  {
    float v1x = po2[0] - p1[0], v1y = po2[1] - p1[1], v1z = po2[2] - p1[2];
    float v2x = p2[0] - p1[0], v2y = p2[1] - p1[1], v2z = p2[2] - p1[2];
    ves2x = v1y * v2z - v1z * v2y;
    ves2y = v1z * v2x - v1x * v2z;
    ves2z = v1x * v2y - v1y * v2x;
    float sum2 = ves2x * ves2x + ves2y * ves2y + ves2z * ves2z;
    if (!sum2) return -2.f;
    float fac = 1.f / sqrt(sum2);
    ves2x *= fac;
    ves2y *= fac;
    ves2z *= fac;
  }
  float d = ves1x * ves2x + ves1y * ves2y + ves1z * ves2z;
  if (d < -1.f) d = -1.f;
  if (d > +1.f) d = +1.f;
  return d;
}

float signed_dihedral_angle(const Point& p1, const Point& p2, const Point& po1, const Point& po2) {
  Vector ves1 = cross(p1, p2, po1);
  Vector ves2 = cross(p1, po2, p2);
  // no need to normalize since we obtain both fcos and fsin
  float fcos = dot(ves1, ves2);
  Vector vcross = cross(ves1, ves2);
  float fsin = mag(vcross);
  if (dot(vcross, p2 - p1) < 0.f) fsin = -fsin;
  if (!fsin && !fcos) return -10.f;
  return std::atan2(fsin, fcos);
}

float solid_angle(const Point& p, CArrayView<Point> pa) {
  // solid angle: fraction area covered on sphere centered about p.  maximum = TAU * 2.
  // idea: Gauss-Bonnet theorem:
  // integral of curvature + line integral + exterior angles = TAU
  // integral of curvature on unit sphere is equal to area
  // line integral along geodesics (great circles) is zero
  // So, solid angle = TAU - sum of exterior angles on unit sphere
  //
  // NOTE: This is imprecise for small triangles, due to TAU - TAU, but changing computation to double fixes that.
  //
  // Alternative definitions (only valid for ang < TAU / 4):
  // sin(ang / 2) = sqrt(sin(s) * sin(s - a) * sin(s - b) * sin(s - c)) / (2 * cos(a / 2) * cos(b / 2) * cos(c / 2))
  //  where  s = (a + b + c) / 2   a = arclen(BC) on sphere, ...
  //
  // tan(ang / 4) = sqrt(tan(s / 2) tan((s - a) / 2) tan((s - b) / 2) tan((s - c) / 2))
  //
  const int np = pa.num();
  double sum_ang = 0.;
  if (0) {
    for_int(i, np) {
      int ip = (i - 1 + np) % np, in = (i + 1) % np;
      Vector top = pa[i] - p;
      if (!assertw(top.normalize())) continue;
      Vector v1 = pa[i] - pa[ip];
      v1 -= top * dot(v1, top);
      if (!assertw(v1.normalize())) continue;
      Vector v2 = pa[in] - pa[i];
      v2 -= top * dot(v2, top);
      if (!assertw(v2.normalize())) continue;
      float vcos = dot(v1, v2);
      float vsin = dot(cross(v1, v2), top);
      float ang = std::atan2(vsin, vcos);
      sum_ang += ang;
    }
  } else {
    // Can correctly handle duplicate points in loop
    Point pp = pa[0];
    Vector vp;
    dummy_init(vp);
    Vector topp = vp;
    bool have_prior = false;
    for (int i = 1;; i++) {
      Point pc = pa[i % np];
      Vector vc = pc - pp;
      if (is_zero(vc)) {
        Warning("is_zero(vc)");
        if (!have_prior && i >= np) {
          Warning("Degenerate solid angle");
          sum_ang = D_TAU;
          break;
        }
        continue;
      }
      Vector top = pc - p;
      if (!assertw(top.normalize())) return 0.f;
      if (have_prior) {
        // project onto sphere tangent vectors at topp
        Vector v1 = project_orthogonally(vp, topp);
        if (!assertw(v1.normalize())) return 0.f;
        Vector v2 = project_orthogonally(vc, topp);
        if (!assertw(v2.normalize())) return 0.f;
        float vcos = dot(v1, v2);
        float vsin = dot(cross(v1, v2), topp);
        double ang = std::atan2(vsin, vcos);
        // Ambiguity between -TAU / 2 and +TAU / 2 does matter here!
        if (0) SHOW(ang, ang + D_TAU / 2);
        if (ang < -D_TAU / 2 + 1e-6) {
          Warning("Near degen angle");
          ang = +D_TAU / 2;
        }
        sum_ang += ang;
      }
      if (i >= np + 1) break;
      pp = pc;
      have_prior = true;
      vp = vc;
      topp = top;
    }
  }
  float solid_ang = float(D_TAU - sum_ang);
  if (0 && solid_ang > TAU * 2 - 1e-6f) solid_ang = 0.f;
  if (0) solid_ang = clamp(solid_ang, 0.f, TAU * 2);
  return solid_ang;
}

float angle_cos(const Point& p1, const Point& p2, const Point& p3) {
  Vector v1 = p2 - p1;
  Vector v2 = p3 - p2;
  if (!v1.normalize() || !v2.normalize()) return -2;
  float d = dot(v1, v2);
  if (d < -1.f) d = -1.f;
  if (d > +1.f) d = +1.f;
  return d;
}

// *** Frames and Euler angles

namespace {

template <typename T> T my_atan2(T y, T x) { return !y && !x ? T{0} : std::atan2(y, x); }

}  // namespace

// See https://en.wikipedia.org/wiki/Euler_angles
//  Tait-Bryan angles / Nautical angles / Cardan angles : sometimes called Euler angles, not "proper Euler angles"
//
//  I use (z, -y', x''):
//   angle 0 is measured wrt  z  axis (alpha) (yaw)   (heading)
//   angle 1 is measured wrt -y' axis (beta)  (pitch) (elevation)
//   angle 2 is measured wrt x'' axis (phi)   (roll)  (bank)
//
//  This is similar to case 5 of Tait-Bryan angles (z, y', x'') except that y' is negated.
//
//  It is also similar to aircraft principal axes: (-z (down), -y' (right), x'' (front)) except that z is negated,
//   so my yaw/heading is reversed wrt compass directions.
//
// To extract angles, look at tostdf:
//      cos(b)cos(a)  cos(b)sin(a)  -sin(b)
//      ?             ?             sin(p)cos(b)
//      ?             ?             cos(p)cos(b)

Vec3<float> frame_to_euler_angles(const Frame& f) {
  Vec3<float> ang;
  ang[0] = my_atan2(f[0][1], f[0][0]);
  ang[1] = my_atan2(-f[0][2], std::hypot(f[0][0], f[0][1]));
  ang[2] = my_atan2(f[1][2] / sqrt(square(f[1][0]) + square(f[1][1]) + square(f[1][2])),
                    f[2][2] / sqrt(square(f[2][0]) + square(f[2][1]) + square(f[2][2])));
  return ang;
}

void euler_angles_to_frame(const Vec3<float>& ang, Frame& f) {
  Frame fr = Frame::identity();  // note: not modifying f but local temporary (to preserve f.p())
  for_int(c, 3) fr[c][c] = mag(f.v(c));
  for_int(c, 3) fr = fr * Frame::rotation(c, ang[2 - c]);  // world Z yaw, then world Y pitch, then world X roll
  for_int(c, 3) f[c] = fr[c];
}

void frame_aim_at(Frame& f, const Vector& v) {
  Vec3<float> ang;
  ang[0] = my_atan2(v[1], v[0]);
  ang[1] = my_atan2(-v[2], std::hypot(v[0], v[1]));
  ang[2] = 0.f;
  euler_angles_to_frame(ang, f);
}

Frame make_level(const Frame& f) {
  Frame fnew = f;
  static const bool world_zxy = getenv_bool("WORLD_ZXY");  // z forward, -x left, -y up
  const Frame from_zxy =
      Frame(Vector(0.f, 0.f, 1.f), Vector(-1.f, 0.f, 0.f), Vector(0.f, -1.f, 0.f), Point(0.f, 0.f, 0.f));
  if (world_zxy) fnew *= ~from_zxy;
  Vec3<float> ang = frame_to_euler_angles(fnew);
  ang[2] = 0.f;
  euler_angles_to_frame(ang, fnew);
  if (world_zxy) {
    fnew *= from_zxy;
    fnew.p() = f.p();
  }
  return fnew;
}

Frame make_horiz(const Frame& f) {
  Frame fnew = f;
  static const bool world_zxy = getenv_bool("WORLD_ZXY");  // z forward, -x left, -y up
  const Frame from_zxy =
      Frame(Vector(0.f, 0.f, 1.f), Vector(-1.f, 0.f, 0.f), Vector(0.f, -1.f, 0.f), Point(0.f, 0.f, 0.f));
  if (world_zxy) fnew *= ~from_zxy;
  Vec3<float> ang = frame_to_euler_angles(fnew);
  ang[1] = 0.f;
  ang[2] = 0.f;
  euler_angles_to_frame(ang, fnew);
  if (world_zxy) {
    fnew *= from_zxy;
    fnew.p() = f.p();
  }
  return fnew;
}

void widen_triangle(ArrayView<Point> poly, float eps) {
  assertx(poly.num() == 3);
  Point p0 = interp(poly[0], poly[1], poly[2], 1.f + eps, -eps * 0.5f);
  Point p1 = interp(poly[0], poly[1], poly[2], -eps * 0.5f, 1.f + eps);
  Point p2 = interp(poly[0], poly[1], poly[2], -eps * 0.5f, -eps * 0.5f);
  poly[0] = p0;
  poly[1] = p1;
  poly[2] = p2;
}

float signed_volume(const Point& p1, const Point& p2, const Point& p3, const Point& p4) {
  // Formula derived from the scalar triple product of vectors.
  if (0) {
    return dot(p2 - p1, cross(p3 - p1, p4 - p1)) / 6.f;
  } else {
    return dot(p2 - p1, cross(p1, p3, p4)) / 6.f;
  }
}

}  // namespace hh
