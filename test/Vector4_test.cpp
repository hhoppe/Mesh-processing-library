// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Vector4.h"

// #include <iomanip>  // setprecision()

#include "libHh/Array.h"
#include "libHh/Vec.h"
using namespace hh;

// Test Neon syntax:
// modify make/Makefile_defs_clang to override cxxall, then
// make CONFIG=clang -C ~/git/mesh_processing/test Vector4_test.o
//  (It checks C++ syntax, then crashes with "ARM does not support Windows COFF format".)

static void to_norm(const Vector4& v) {
  SHOW(v);
  Pixel pix = v.pixel();
  Vec4<int> ar;
  for_int(c, 4) ar[c] = pix[c];
  SHOW(ar);
}

int main() {
  {
      // Setting the precision has no effect on SHOW() because it now uses a temporary std::ostringstream .
      // std::cerr << std::setprecision(4) << std::setiosflags(std::ios::fixed);
      // std::cerr.precision(4); std::cerr.setf(std::ios::fixed);
  } {
    Vector4 v1(1.f, 2.f, 3.f, 4.f), v2(8.f, 7.f, 6.f, 5.f);
    SHOW(v1);
    SHOW(v2);
    SHOW(v1 + v2);
    SHOW(v1 - v2);
    SHOW(v1 - v2 - v1 + v2 * 2.f);
    SHOW(dot(v1, v2));
    SHOW(mag2(v2));
    SHOW(dist2(v1, v2));
    SHOW(sum(v2));
    SHOW(min(v1, v2));
    SHOW(max(v1, v2));
    alignas(16) float ar1[4];
    v1.store_aligned(ar1);
    SHOW(ar1[0], ar1[3]);
    Vector4 v3;
    v3.load_aligned(ar1);
    SHOW(v3);
    Vector4 v4(v3);
    SHOW(v4);
    v4 += v2;
    SHOW(v4);
    SHOW(sizeof(v1));
    Vector4 va[2];
    SHOW(reinterpret_cast<uint8_t*>(&va[1]) - reinterpret_cast<uint8_t*>(&va[0]));
  }
  {
    Vec<uint8_t, 8> ar{uint8_t{23}, uint8_t{37}, uint8_t{45}, uint8_t{255},
                       uint8_t{12}, uint8_t{31}, uint8_t{37}, uint8_t{0}};
    SHOW(to_Vector4_raw(ar.data()));
    SHOW(to_Vector4_raw(&ar[4]));
    SHOW(to_Vector4_norm(ar.data()));
    SHOW(to_Vector4_norm(&ar[4]));
    Vec4<uint8_t> ar2;
    Vector4 v1 = to_Vector4_raw(ar.data());
    // v1.raw_to_byte4(ar2.data());
    ar2 = v1.raw_pixel();
    SHOW(int(ar2[0]), int(ar2[1]), int(ar2[2]), int(ar2[3]));
    v1 = to_Vector4_norm(ar.data());
    // v1.norm_to_byte4(ar2.data());
    ar2 = v1.pixel();
    SHOW(int(ar2[0]), int(ar2[1]), int(ar2[2]), int(ar2[3]));
  }
  {
    to_norm(Vector4(0.f, 0.49f / 255.f, 0.51f / 255.f, 1.51f / 255.f));
    to_norm(Vector4(-10.f, .5f, 1e4f, 0.f));
    to_norm(Vector4(23.f, 37.f, 45.f, 255.f) / 255.f);
    to_norm(Vector4(0.f, 1.f, 2.f, 3.f) / 255.f);
    to_norm(Vector4(100.f, 101.f, 102.f, 103.f) / 255.f);
  }
  if (0) {  // huge numbers fail the conversion to int32_t
    to_norm(Vector4(2147483583.f, 2147483584.f, 2147483647.f, BIGFLOAT) / 255.f);
    to_norm(Vector4(-2147483580.f, -2147483582.f, -2147483647.f, -BIGFLOAT) / 255.f);
  }
#if 0
  {
    // Fails: static_assert(std::is_trivially_copyable_v<Vector4>);
#if defined(HH_VECTOR4_SSE)
    static_assert(std::is_trivially_copyable_v<__m128>);  // true
#endif
  }
#endif
}
