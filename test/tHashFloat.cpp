// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "HashFloat.h"

#include <iomanip>              // std::setprecision()

#include "RangeOp.h"            // sum()
#include "ArrayOp.h"            // concat()
using namespace hh;

namespace {

void try_it(HashFloat& hf, float f) {
    float f2 = hf.enter(f);
    showf("enter %14.8f -> %14.8f\n", f, f2);
}

void test(int nignore, float small) {
    SHOW(nignore);
    SHOW(small);
    HashFloat hf(nignore, small);
    {
        try_it(hf, 1.0000000f);
        try_it(hf, 1.0200000f);
        try_it(hf, 1.0020000f);
        try_it(hf, 1.0002000f);
        try_it(hf, 1.0000200f);
        try_it(hf, 1.0000112f);
        try_it(hf, 1.0000066f);
        try_it(hf, 1.0000073f);
        try_it(hf, 1.0000057f);
        try_it(hf, 1.0000069f);
        try_it(hf, 1.0000082f);
        try_it(hf, 1.0000062f);
        try_it(hf, 1.0000034f);
        try_it(hf, 1.0000036f);
        try_it(hf, 1.0000038f);
        try_it(hf, 1.0000032f);
        try_it(hf, 1.0000037f);
        try_it(hf, 1.0000033f);
        try_it(hf, 1.0000020f);
        try_it(hf, 1.0000015f);
        try_it(hf, 1.0000010f);
        try_it(hf, 1.0000006f);
        try_it(hf, 1.0000004f);
        try_it(hf, 1.0000002f);
        try_it(hf, 1.0000001f);
        try_it(hf, 0.99999999f);
        try_it(hf, 0.9999999f);
        try_it(hf, 0.9999999f);
        try_it(hf, 0.9999990f);
        try_it(hf, 0.9999900f);
        try_it(hf, 0.9999000f);
        try_it(hf, 0.9990000f);
    }
    {
        for_int(i, 20) { try_it(hf, i*1e-5f); }
    }
    if (0) {
        try_it(hf, 0.0222916f);
        try_it(hf, 0.0222923f);
    }
}

template<typename T> T roundtrip(T v, int digits = -1) {
    std::stringstream ss;
    if (digits>=0) {
        if (0) {
            assertx(ss << std::setprecision(digits));
            // std::setprecision(std::numeric_limits<T>::digits10);      // 6 for float; 15 for double
            // std::setprecision(std::numeric_limits<T>::max_digits10);  // 9 for float; 17 for double
        }
        auto old_precision = ss.precision(digits);
        assertx(old_precision==6);
    }
    assertx(ss << v);
    T v2;
    if (1) {
        static_assert(std::is_same<T, float>::value, "");
        assertx(sscanf(ss.str().c_str(), "%f", &v2));
    } else {
        assertx(ss >> v2);
    }
    return v2;
}

uint32_t as_uint(float f) { union { uint32_t ui; float f; } u; u.f = f; return u.ui; }

void test_io() {
    const float eps = std::numeric_limits<float>::epsilon();
    const Array<float> ar { -3.f, -2.5f, -2.f, -1.5f, -1.f, -.5f, 0.f, .5f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f };
    const Array<float> ar1eps = ar*eps+1.f;
    // 7 digits of precision are insufficient, in either sform("%.7g") or ostream << setprecision(7).
    // 8 digits are mostly sufficient -- e.g. not true of numbers between 1000 and 1024
    // 1023.9932861328125f from http://randomascii.wordpress.com/2012/02/11/they-sure-look-equal/
    // printf("%1.8e\n", d);  // Round-trippable float, always with an exponent
    // printf("%.9g\n", d);   // Round-trippable float, shortest possible
    // printf("%1.16e\n", d); // Round-trippable double, always with an exponent
    // printf("%.17g\n", d);  // Round-trippable double, shortest possible
    for (float f : concat(ar1eps, ArView(1023.9932861328125f), ArView(1023.9933471679687f),
                          V(0.2288884f, 0.228888392f, 0.228888407f).view())) {
        float f2 = roundtrip(f, 9); // was 7; in principle 9 is required!  (17 is sufficient for double)
        // %a (hexadecimal float) is not supported in mingw which uses old MS CRT
        showf("f=%-10.7g %-10.8g %-11.9g %x  f2=%-10.7g %-10.8g %-11.9g %x  f==f2=%d\n",
              f, f, f, as_uint(f), f2, f2, f2, as_uint(f2), f==f2);
    }
    // 0.2288884f in tFrameIO.inp:
    //  VC12 writes it as 0.228888392; this seems incorrect -- it is different as shown below
    //  gcc  writes it as 0.228888407
    //win:
    // f=0.2288884  0.22888841 0.228888407 3e6a61b9  f2=0.2288884  0.22888841 0.228888407 3e6a61b9  f==f2=1
    // f=0.2288884  0.22888839 0.228888392 3e6a61b8  f2=0.2288884  0.22888839 0.228888392 3e6a61b8  f==f2=1
    // f=0.2288884  0.22888841 0.228888407 3e6a61b9  f2=0.2288884  0.22888841 0.228888407 3e6a61b9  f==f2=1
    //mingw:
    // f=0.2288884  0.22888841 0.228888407 3e6a61b9  f2=0.2288884  0.22888841 0.228888407 3e6a61b9  f==f2=1
    // f=0.2288884  0.22888839 0.228888392 3e6a61b8  f2=0.2288884  0.22888839 0.228888392 3e6a61b8  f==f2=1
    // f=0.2288884  0.22888841 0.228888407 3e6a61b9  f2=0.2288884  0.22888841 0.228888407 3e6a61b9  f==f2=1

}

} // namespace

int main() {
    {
        test(8, 1e-4f);
        test(4, 1e-6f);
        test(0, 0.f);
    }
    test_io();
}
