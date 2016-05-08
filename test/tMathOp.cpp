// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "MathOp.h"
using namespace hh;


// (float)0., -0.      0x00000000
// (float)1            0x3f800000
// (float)3            0x40400000
// (float)-zero        0x80000000  (-1/1e20/1e20)
// (float)-1           0xbf800000
// (float)-3           0xc0400000
// (float)1.#INF       0x7f800000  (+1/0)   (C++11: INFINITY)
// (float)-1.#INF      0xff800000  (-1/0)
// (float)-1.#IND      0xffc00000  (acos(2), 0/0)  (std::isnan())
//                     0x00400000 always forced on by hardware for any nanf (x86)

// Create an infinite float value.
inline float create_infinityf() {
    if (0) {
        // return INFINITY;            // C++11; warning: overflow in constant arithmetic
    } else if (1) {
        return std::numeric_limits<float>::infinity(); // also C++11
    } else {
        union { float f; uint32_t ui; } u; u.ui = 0x7f800000; return u.f;
    }
}

// Create a not-a-number float value which encodes integer i (0..4194303 or 22 bits).
inline float create_nanf(unsigned i = 0) {
    if (0 && i==0) return NAN;  // C++11, equivalent to std::numeric_limits<float>::quiet_NaN()
    ASSERTXX((i&0xffc00000)==0);
    // Could in principle retrieve 0x80000000 (sign) bit from i and use it, but forget it.
    union { float f; uint32_t ui; } u; u.ui = 0x7fc00000|(i&0x003fffff); return u.f;
}

// Retrieve the integer value encoded in the not-a-number value f.
inline unsigned nanf_value(float f) {
    union { float f; uint32_t ui; } u; u.f = f; ASSERTXX(std::isnan(u.f)); return u.ui&0x003fffff;
}

int main() {
    {
        assertx(INFINITY==HUGE_VALF);
        // assertx(std::numeric_limits<float>::infinity()==HUGE_VALF); // warning: overflow in constant arithmetic
        // assertx(std::numeric_limits<double>::infinity()==HUGE_VAL);
    }
    {
        const Array<float> ar = { 100.f, 102.f, 103.f, 110.f, 100.f, 90.f, 80.f, 71.f };
        int n = 20;
        for_int(i, n) {
            float x = i/(n-1.f);    // x in [0, 1]
            const int degree = 3;
            float v = eval_uniform_bspline(ar, degree, x);
            showf("x=%7.4f   v=%6.3f\n", x, v);
        }
    }
    if (0) {
        float g_float_zero = g_unoptimized_zero ? 1.f : 0.f;
        auto func_show_float = [](float a) {
            union { float f; uint32_t ui; } u; u.f = a;
            showf("(float)%-15.9g 0x%08x  F%d I%d N%d%s\n",
                  a, u.ui, std::isfinite(a), std::isinf(a), std::isnan(a),
                  std::isnan(a) ? sform(" nanfv%08x", nanf_value(a)).c_str() : "");
        };
        float a;
        a = +0.f; func_show_float(a);
        a = +1.f; func_show_float(a);
        a = +3.f; func_show_float(a);
        a = -0.f; func_show_float(a);
        a = -1.f/1e30f/1e30f; func_show_float(a); // -0 in win; 0 in mingw and cygwin
        a = -1.f; func_show_float(a);
        a = -3.f; func_show_float(a);
        a = +1.f/g_float_zero; func_show_float(a);
        a = -1.f/g_float_zero; func_show_float(a);
        a = acos(2.f); func_show_float(a);
        a = acos(-2.f); func_show_float(a);
        a = 0.f/g_float_zero; func_show_float(a); // IND in win; 0 in mingw and cygwin
        a = create_infinityf(); func_show_float(a);
        a = -create_infinityf(); func_show_float(a);
        a = create_nanf(); func_show_float(a);
        a = create_nanf(0x00000000); func_show_float(a);
        a = create_nanf(0x00000001); func_show_float(a);
        a = create_nanf(0x00000002); func_show_float(a);
        a = create_nanf(0x003fffff); func_show_float(a);
        SHOW(std::isfinite(0.f/g_float_zero));
        // Note: with gcc (mingw, mingw32, cygwin), isinf() and isnan() always report false, and
        //  isfinite() always reports true,
        //  This is due to the compilation flag "-ffast-math".
        //  It might work with "-fno-finite-math-only" but then some optimizations might be disabled.
        // win:
        //   (float)0               0x00000000  F1 I0 N0
        //   (float)1               0x3f800000  F1 I0 N0
        //   (float)3               0x40400000  F1 I0 N0
        //   (float)-0              0x80000000  F1 I0 N0
        //   (float)-0              0x80000000  F1 I0 N0
        //   (float)-1              0xbf800000  F1 I0 N0
        //   (float)-3              0xc0400000  F1 I0 N0
        //   (float)inf             0x7f800000  F0 I1 N0
        //   (float)-inf            0xff800000  F0 I1 N0
        //   (float)-nan(ind)       0xffc00000  F0 I0 N1 nanfv00000000
        //   (float)-nan(ind)       0xffc00000  F0 I0 N1 nanfv00000000
        //   (float)-nan(ind)       0xffc00000  F0 I0 N1 nanfv00000000
        //   (float)inf             0x7f800000  F0 I1 N0
        //   (float)-inf            0xff800000  F0 I1 N0
        //   (float)nan             0x7fc00000  F0 I0 N1 nanfv00000000
        //   (float)nan             0x7fc00000  F0 I0 N1 nanfv00000000
        //   (float)nan             0x7fc00001  F0 I0 N1 nanfv00000001
        //   (float)nan             0x7fc00002  F0 I0 N1 nanfv00000002
        //   (float)nan             0x7fffffff  F0 I0 N1 nanfv003fffff
        // clang:
        //   (float)0               0x00000000  F1 I0 N0
        //   (float)1               0x3f800000  F1 I0 N0
        //   (float)3               0x40400000  F1 I0 N0
        //   (float)-0              0x80000000  F1 I0 N0
        //   (float)-0              0x80000000  F1 I0 N0
        //   (float)-1              0xbf800000  F1 I0 N0
        //   (float)-3              0xc0400000  F1 I0 N0
        //   (float)inf             0x7f800000  F0 I1 N0
        //   (float)-inf            0xff800000  F0 I1 N0
        //   (float)nan             0xffc00000  F0 I0 N1 nanfv00000000
        //   (float)nan             0xffc00000  F0 I0 N1 nanfv00000000
        //   (float)nan             0xffc00000  F0 I0 N1 nanfv00000000
        //   (float)inf             0x7f800000  F0 I1 N0
        //   (float)-inf            0xff800000  F0 I1 N0
        //   (float)nan             0x7fc00000  F0 I0 N1 nanfv00000000
        //   (float)nan             0x7fc00000  F0 I0 N1 nanfv00000000
        //   (float)nan             0x7fc00001  F0 I0 N1 nanfv00000001
        //   (float)nan             0x7fc00002  F0 I0 N1 nanfv00000002
        //   (float)nan             0x7fffffff  F0 I0 N1 nanfv003fffff
        // mingw and cygwin (with -ffast-math):
        //   (float)0               0x00000000  F1 I0 N0
        //   (float)1               0x3f800000  F1 I0 N0
        //   (float)3               0x40400000  F1 I0 N0
        //   (float)-0              0x80000000  F1 I0 N0
        //   (float)-0              0x80000000  F1 I0 N0
        //   (float)-1              0xbf800000  F1 I0 N0
        //   (float)-3              0xc0400000  F1 I0 N0
        //   (float)inf             0x7f800000  F1 I0 N0
        //   (float)-inf            0xff800000  F1 I0 N0
        //   (float)nan             0xffc00000  F1 I0 N0 // 0x7fc00000 in cygwin
        //   (float)nan             0xffc00000  F1 I0 N0 // 0x7fc00000 in cygwin
        //   (float)nan             0xffc00000  F1 I0 N0
        //   (float)inf             0x7f800000  F1 I0 N0
        //   (float)-inf            0xff800000  F1 I0 N0
        //   (float)nan             0x7fc00000  F1 I0 N0
        //   (float)nan             0x7fc00000  F1 I0 N0
        //   (float)nan             0x7fc00001  F1 I0 N0
        //   (float)nan             0x7fc00002  F1 I0 N0
        //   (float)nan             0x7fffffff  F1 I0 N0
    }
    {
        const int vm4mod7 = my_mod(-4, 7); SHOW(vm4mod7);
        constexpr double vmid = smooth_step(.5); SHOW(vmid);
        constexpr double vfurther = smooth_step(2./3.); SHOW(vfurther);
        const float vfrac = frac(TAU); SHOW(vfrac);
        const float vgauss1 = gaussian(1.f); SHOW(vgauss1);
        const double vmyacos = my_acos(-1.0001); SHOW(vmyacos);
        constexpr bool is_pow2_16 = is_pow2(16); SHOW(is_pow2_16);
        constexpr bool is_pow2_17 = is_pow2(17); SHOW(is_pow2_17);
    }
}
