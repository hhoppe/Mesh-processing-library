// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Array.h"
#include "RangeOp.h"
#include "Advanced.h"           // clone()
using namespace hh;

int main() {
    {
        Array<uchar> ar1 = {4, 200, 254, 3, 7, 2};
        Array<uchar> ar2 = {4, 0, 0, 3, 7, 2};
        SHOW(mag2(ar1));
        SHOW(square(rms(ar1))*ar1.num());
        SHOW(mag(ar2));
        SHOW(sqrt(var(ar1)));
        SHOW(dist2(ar1, ar2));
        SHOW(dist(ar1, ar2));
        SHOW(dot(ar1, ar2));
        SHOW(int(min(ar1)));
        SHOW(int(max(ar1)));
        // SHOW(int(max_abs_element(ar1))); // abs(uchar) gives compilation warning in clang (-Wabsolute-value)
        SHOW(sum(ar1));
        SHOW(sum(ar2));
        SHOW(product(ar1));
        SHOW(product(ar2));
        SHOW(compare(ar1, ar2));
        SHOW(compare(ar2, ar1));
        SHOW(compare(ar1, ar1));
        SHOW(compare(ar2, ar2));
        SHOW(ar1==ar2);
        SHOW(ar2==ar1);
        SHOW(ar1==ar1);
        SHOW(ar2==ar2);
        SHOW(is_zero(ar1));
        SHOW(is_zero(ar1-ar1));
        SHOW(count(ar2, 0));
        SHOW(count(ar2, 3));
        SHOW(count(ar2, 99));
        // SHOW(count_if(ar2, [](uchar uc){ return uc>5; })); // compilation error (lambda used in decltype of SHOW)
        auto func_gt5 = [](uchar uc){ return uc>5; };
        SHOW(count_if(ar2, func_gt5));
    }
    {
        Array<float> ar1 = {2.7f, -3.3f, 5.1f, -6.2f, 0.f};
        Array<float> ar2 = {2.7f, -3.3f, 5.2f, -6.2f, 0.f};
        swap_ranges(ar1, ar2); SHOW(ar1); swap_ranges(ar1, ar2);
        SHOW(normalize(clone(ar1)));
        SHOW(mag(normalize(clone(ar1))));
        SHOW(sort(clone(ar1)));
        SHOW(reverse(sort(clone(ar1))));
        SHOW(max_abs_element(ar1));
        SHOW(sum(ar1));
        SHOW(compare(ar1, ar2));
        SHOW(compare(ar2, ar1));
        SHOW(compare(ar1, ar1));
        SHOW(compare(ar2, ar2));
        SHOW(ar1==ar2);
        SHOW(ar2==ar1);
        SHOW(ar1==ar1);
        SHOW(ar2==ar2);
        const float tol = .3f;
        SHOW(compare(ar1, ar2, tol));
        SHOW(compare(ar2, ar1, tol));
        SHOW(compare(ar1, ar1, tol));
        SHOW(compare(ar2, ar2, tol));
    }
    {
        int ar[] = {10, 11, 12, 13, 14, 15}; // test C-array
        SHOW(mean(ar));
    }
    {
        SHOW(details::has_begin<Array<float>>::value ? 1 : 0);
        SHOW(details::has_begin<std::fstream>::value ? 1 : 0);
        struct S { int _a; };
        // "!!" necessary in gcc 4.8.2 debug,
        //  else linker error "ld: tRangeOp.cpp: undefined reference to `hh::details::has_begin<main::S>::value'"
        SHOW(!!details::has_begin<S>::value);
    }
    {
        // This should fail to compile.
        // S s; SHOW(mean(s));
    }
    {
        SHOW(type_name<mean_type_t<float>>());
        SHOW(type_name<mean_type_t<double>>());
        SHOW(type_name<mean_type_t<char>>());
        SHOW(type_name<mean_type_t<uchar>>());
        SHOW(type_name<mean_type_t<short>>());
        SHOW(type_name<mean_type_t<ushort>>());
        SHOW(type_name<mean_type_t<int>>());
        SHOW(type_name<mean_type_t<unsigned>>());
        SHOW(type_name<mean_type_t<char*>>());
    }
    {
        Array<float> ar1 = {2.7f, -3.3f, 5.1f, -6.2f, 0.f};
        { auto ar = clone(ar1); rotate(ar, ar[2]); SHOW(ar); }
        { auto ar = clone(ar1); std::rotate(ar.begin(), &ar[2], ar.end()); SHOW(ar); }
    }
}
