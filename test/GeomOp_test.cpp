// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GeomOp.h"

using namespace hh;

int main() {
  {
    // The axis points map exactly between lon-lat and sph.
    SHOW(sph_from_lonlat(Uv(0.f, 0.f)));
    SHOW(sph_from_lonlat(Uv(.3f, 1.f)));
    SHOW(sph_from_lonlat(Uv(0.f, .5f)));
    SHOW(sph_from_lonlat(Uv(.25f, .5f)));
    SHOW(sph_from_lonlat(Uv(.5f, .5f)));
    SHOW(sph_from_lonlat(Uv(.75f, .5f)));
    SHOW(lonlat_from_sph(Point(0.f, 0.f, -1.f)));
    SHOW(lonlat_from_sph(Point(0.f, 0.f, 1.f)));
    SHOW(lonlat_from_sph(Point(-1.f, 0.f, 0.f)));
    SHOW(lonlat_from_sph(Point(0.f, -1.f, 0.f)));
    SHOW(lonlat_from_sph(Point(1.f, 0.f, 0.f)));
  }
  {
    // Round trip lonlat -> sph -> lonlat, including latitudes near the poles, where sph_from_lonlat() once snapped a
    // coordinate to +-1 without the others, and lonlat_from_sph() once lost precision in std::asin().
    double max_mag_error = 0.;
    float max_lat_error = 0.f, max_lon_error = 0.f;
    const int n = 400;
    for_int(i, n + 1) {
      for_int(j, n + 1) {
        const float lat = j == 1 ? .0004f : j == n - 1 ? .9996f : float(j) / n;
        const Uv lonlat(float(i) / n, lat);
        const Point sph = sph_from_lonlat(lonlat);
        max_mag_error = max(max_mag_error, abs(mag<double>(sph) - 1.));
        const Uv lonlat2 = lonlat_from_sph(sph);
        max_lat_error = max(max_lat_error, abs(lonlat2[1] - lonlat[1]));
        // The longitude is undefined at the poles and ill-conditioned near them, and it wraps at the prime meridian.
        if (i > 0 && i < n)
          max_lon_error = max(max_lon_error, abs(lonlat2[0] - lonlat[0]) * std::sin(lat * (TAU / 2)));
      }
    }
    SHOW(max_mag_error < 4e-7);
    SHOW(max_lat_error < 1e-6f);
    SHOW(max_lon_error < 1e-6f);
    // A latitude within 1.4e-3 radians of a pole used to return as the pole itself.
    SHOW(abs(lonlat_from_sph(sph_from_lonlat(Uv(.3f, .0004f)))[1] - .0004f) < 1e-6f);
  }
  {
    // Round trip sph -> lonlat -> sph, for points at geometrically spaced angles from each pole.  The error is
    // dominated by the snapping of a latitude within 1e-6 of 0 or 1, i.e. within 3.1e-6 radians of a pole.
    float max_error = 0.f;
    for_int(i, 1000) {
      const double angle = 1e-7 * std::pow(1e7, i / 999.), lon = i * .618;
      for (const double sign_z : {-1., 1.}) {
        const Point sph(float(std::sin(angle) * std::cos(lon)), float(std::sin(angle) * std::sin(lon)),
                        float(sign_z * std::cos(angle)));
        max_error = max(max_error, dist(sph_from_lonlat(lonlat_from_sph(sph)), sph));
      }
    }
    SHOW(max_error < 1e-5f);
  }
}
