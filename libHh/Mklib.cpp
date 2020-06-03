// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Mklib.h"

namespace hh {

#define mk_save hh::MkSave HH_UNIQUE_ID(mksave)(mk)
#define mk_save_color hh::MkSaveColor HH_UNIQUE_ID(mksavecolor)(mk)

void Mklib::squareO() {
  mk.point(0, -.5f, -.5f);
  mk.point(0, +.5f, -.5f);
  mk.point(0, +.5f, +.5f);
  mk.point(0, -.5f, +.5f);
  mk.end_polygon();
}

void Mklib::squareXY() {
  mk_save;
  mk.translate(.5f, .5f, 0);
  mk.rotate(1, TAU / 4);
  squareO();
}

void Mklib::squareU() {
  mk_save;
  mk.rotate(1, -TAU / 4);
  squareO();
}

void Mklib::cubeO() {
  {
    mk_save;
    mk.translate(+.0f, +.0f, -.5f);
    mk.rotate(1, TAU / 4);
    squareO();
  }
  {
    mk_save;
    mk.translate(+.0f, +.0f, +.5f);
    mk.rotate(1, -TAU / 4);
    squareO();
  }
  {
    mk_save;
    mk.translate(-.5f, +.0f, +.0f);
    mk.rotate(2, TAU / 2);
    squareO();
  }
  {
    mk_save;
    mk.translate(+.5f, +.0f, +.0f);
    mk.rotate(2, 0);
    squareO();
  }
  {
    mk_save;
    mk.translate(+.0f, -.5f, +.0f);
    mk.rotate(2, -TAU / 4);
    squareO();
  }
  {
    mk_save;
    mk.translate(+.0f, +.5f, +.0f);
    mk.rotate(2, TAU / 4);
    squareO();
  }
}

void Mklib::cubeXYZ() {
  mk_save;
  mk.translate(.5f, .5f, .5f);
  cubeO();
}

void Mklib::cubeU() {
  mk_save;
  mk.translate(0, 0, .5f);
  cubeO();
}

void Mklib::polygonO(int n) {
  for_int(i, n) {
    float a = i * TAU / n;
    mk.point(0, cos(a), sin(a));
  }
  mk.end_polygon();
}

void Mklib::polygonU(int n) {
  mk_save;
  mk.rotate(1, -TAU / 4);
  mk.rotate(0, -TAU / 4);
  polygonO(n);
}

void Mklib::ringU(int n, float h, float r0, float r1, float a0, float a1) {
  assertx((r0 > 0 || r1 > 0) && r0 >= 0 && r1 >= 0);
  mk_save;
  if (r0 <= 0) {
    mk.translate(0, 0, h);
    mk.rotate(0, TAU / 2);
    ringU(n, h, r1, r0, -a1, -a0);
  } else {
    float ft0 = r1 / r0, ft3 = h / r0;
    float ft4 = sin((TAU / 2) / n), ft5 = cos((TAU / 2) / n);
    float ft1 = tan(a0), ft2 = tan(a1);
    mk.scale(r0);
    auto func_wedge0 = [&](int) {
      mk.point(0, ft4, 0);
      if (smooth()) mk.normal(ft5, ft4, ft1);
      mk.point((ft0 - 1) * ft5, ft4 * ft0, ft3);
      if (ft0 > 1e-6f) {
        if (smooth()) mk.normal(ft5, ft4, ft2);
        mk.point((ft0 - 1) * ft5, -ft4 * ft0, ft3);
        if (smooth()) mk.normal(ft5, -ft4, ft2);
      } else {
        if (smooth()) mk.normal(1, 0, ft2);
      }
      mk.point(0, -ft4, 0);
      if (smooth()) mk.normal(ft5, -ft4, ft1);
      mk.end_polygon();
    };
    circle_ofU(func_wedge0, n);
  }
}

void Mklib::flat_ringU(int n, float h, float r0, float r1) {
  assertx(r0 * h || r0 * r0 - r1);
  float a0 = atan2(r0 * h, r0 * r0 - r1);
  ringU(n, h, r0, r1, a0, a0);
}

void Mklib::poly_hole(int n, float r1) {
  auto func_poly0 = [&](int) {
    float s = sin((TAU / 2) / n), h = cos((TAU / 2) / n);
    mk.point(0, s, 0);
    mk.point(-h * (1 - r1), +s * r1, 0);
    mk.point(-h * (1 - r1), -s * r1, 0);
    mk.point(0, -s, 0);
    mk.end_polygon();
  };
  circle_ofU(func_poly0, n);
}

void Mklib::volume_ringU(int n, float r1) {
  {
    mk_save;
    mk.rotate(0, TAU / 2);
    poly_hole(n, r1);
  }
  {
    mk_save;
    mk.translate(0, 0, 1);
    poly_hole(n, r1);
  }
  {
    mk_save;
    tubeU(n);
  }
  {
    mk_save;
    mk.scale(r1, r1, 1);
    mk.begin_force_flip(true);
    { tubeU(n); }
    mk.end_force_flip();
  }
}

void Mklib::tubeU(int n) { ringU(n, 1, 1, 1, 0, 0); }

void Mklib::cylinderU(int n) {
  tubeU(n);
  {
    mk_save;
    mk.rotate(0, TAU / 2);
    polygonU(n);
  }
  {
    mk_save;
    mk.translate(0, 0, 1);
    polygonU(n);
  }
}

void Mklib::capU(int n) { ringU(n, 1, 1, 0, TAU / 8, TAU / 8); }

void Mklib::coneU(int n) {
  capU(n);
  {
    mk_save;
    mk.rotate(0, TAU / 2);
    polygonU(n);
  }
}

void Mklib::sphere(int nlat, int nlong) { gsphere(nlat, nlong, false); }

void Mklib::hemisphere(int nlat, int nlong) { gsphere(nlat * 2, nlong, true); }

void Mklib::gsphere(int nlat, int nlong, bool hemi) {
  assertx(nlat >= 2 && nlong >= 3);
  for_int(i, nlat) {
    if (hemi && i < nlat / 2) continue;
    float a1 = (-.5f + float(i) / nlat) * (TAU / 2);
    float a2 = (-.5f + float(i + 1) / nlat) * (TAU / 2);
    float c1 = abs(cos(a1)), c2 = abs(cos(a2));
    float s1 = sin(a1), s2 = sin(a2);
    if (i == 0) {
      a1 = -.249999f * TAU;
      c1 = 0.f;
    } else if (i == nlat - 1) {
      a2 = .249999f * TAU;
      c2 = 0.f;
    }
    {
      mk_save;
      mk.translate(0, 0, s1);
      ringU(nlong, s2 - s1, c1, c2, a1, a2);
    }
  }
}

void Mklib::tetra() {
  float xp = 1.f / sqrt(3.f), xn = xp / -2.f, yp = .5f, yn = -.5f, zp = 1.5f / sqrt(6.f), zn = zp / -3.f;
  mk.point(xp, 0, zn);
  mk.point(xn, yn, zn);
  mk.point(xn, yp, zn);
  mk.end_polygon();
  mk.point(xp, 0, zn);
  mk.point(0, 0, zp);
  mk.point(xn, yn, zn);
  mk.end_polygon();
  mk.point(xn, yn, zn);
  mk.point(0, 0, zp);
  mk.point(xn, yp, zn);
  mk.end_polygon();
  mk.point(xn, yp, zn);
  mk.point(0, 0, zp);
  mk.point(xp, 0, zn);
  mk.end_polygon();
}

void Mklib::tetraU() {
  mk_save;
  mk.translate(.5f / sqrt(6.f), 0, 0);
  tetra();
}

}  // namespace hh
