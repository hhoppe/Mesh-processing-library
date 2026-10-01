// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/HashFloat.h"

#include <bit>      // bit_cast().
#include <iomanip>  // setprecision()

#include "libHh/RangeOp.h"  // sum(), concatenate()
using namespace hh;

namespace {

void try_it(HashFloat& hf, float f) {
  const float f2 = hf.enter(f);
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
    for_int(i, 20) try_it(hf, i * 1e-5f);  //
  }
  if (0) {
    try_it(hf, 0.0222916f);
    try_it(hf, 0.0222923f);
  }
}

// With the default nignorebits = 8, the buckets of values in [1.f, 2.f) each span 256 ulps, i.e. a width of 2^-15.
// Function enter() adopts the representative of the value's bucket if any, or else of a nearby bucket.
void test_buckets() {
  const float w = std::ldexp(1.f, -15);                  // The bucket width.
  const auto at = [&](float k) { return 1.f + k * w; };  // A value (exact) lying in bucket floor(k), for k >= 0.
  {  // The equivalence classes depend on the order in which values are entered.
    HashFloat hf;
    assertx(hf.enter(at(.25f)) == at(.25f));    // Bucket 0 gets a new representative.
    assertx(hf.enter(at(3.25f)) == at(3.25f));  // Bucket 3 is too far from bucket 0.
    assertx(hf.enter(at(1.25f)) == at(.25f));   // Bucket 1 inherits from bucket 0.
    assertx(hf.enter(at(2.25f)) == at(.25f));   // Bucket 2 inherits from bucket 1, although it is next to bucket 3.
    assertx(hf.enter(at(3.75f)) == at(3.25f));
    assertx(hf.enter(at(.75f)) == at(.25f));
  }
  {  // Negative values behave symmetrically, and are distinct from positive values.
    HashFloat hf;
    assertx(hf.enter(-at(.25f)) == -at(.25f));
    assertx(hf.enter(-at(1.25f)) == -at(.25f));
    assertx(hf.enter(-at(3.25f)) == -at(3.25f));
    assertx(hf.enter(at(1.25f)) == at(1.25f));
  }
  {  // Values with magnitude at most `small` map to zero, as do the values in buckets next to them.
    HashFloat hf(8, 1e-4f);
    assertx(hf.enter(-5e-5f) == 0.f);
    assertx(hf.enter(1e-4f) == 0.f);
    assertx(hf.enter(-1.00001e-4f) == 0.f);
    assertx(hf.enter(2e-4f) == 2e-4f);
    assertx(hf.enter(0.f) == 0.f);
  }
  {  // A pre-pass with pre_consider() unifies the representatives of buckets 0 and 2 through bucket 1.
    HashFloat hf;
    hf.pre_consider(at(.25f));
    hf.pre_consider(at(2.25f));  // Bucket 2 gets its own representative.
    hf.pre_consider(at(1.25f));  // Its neighbors have different representatives, so they are unified (a warning).
    hf.pre_consider(at(3.25f));  // Bucket 3 inherits from bucket 2 (the lower neighbor).
    for (const float k : {.25f, .75f, 1.25f, 2.25f, 2.75f, 3.25f, 3.75f}) assertx(hf.enter(at(k)) == at(1.25f));
    assertx(hf.enter(at(5.25f)) == at(5.25f));
  }
  {
    HashFloat hf;
    hf.pre_consider(at(5.25f));
    hf.pre_consider(at(4.25f));  // Bucket 4 inherits from bucket 5 (the upper neighbor).
    hf.pre_consider(at(5.75f));  // Bucket 5 already has a representative.
    hf.pre_consider(0.f);        // The small values have the representative zero.
    for (const float k : {4.25f, 4.75f, 5.25f, 5.75f}) assertx(hf.enter(at(k)) == at(5.25f));
    assertx(hf.enter(at(2.25f)) == at(2.25f));
    assertx(hf.enter(5e-5f) == 0.f);
  }
}

template <typename T> T roundtrip(T v, int digits = -1) {
  std::stringstream ss;
  if (digits >= 0) {
    if (0) {
      assertx(ss << std::setprecision(digits));
      // std::setprecision(std::numeric_limits<T>::digits10);      // 6 for float; 15 for double.
      // std::setprecision(std::numeric_limits<T>::max_digits10);  // 9 for float; 17 for double.
    }
    auto old_precision = ss.precision(digits);
    assertx(old_precision == 6);
  }
  assertx(ss << v);
  T v2;
  if (1) {
    static_assert(std::is_same_v<T, float>);
    assertx(sscanf(ss.str().c_str(), "%f", &v2));
  } else {
    assertx(ss >> v2);
  }
  return v2;
}

uint32_t as_uint(float f) { return std::bit_cast<uint32_t>(f); }

void test_io() {
  const float eps = std::numeric_limits<float>::epsilon();
  const Array<float> ar{-3.f, -2.5f, -2.f, -1.5f, -1.f, -.5f, 0.f, .5f, 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f};
  const Array<float> ar1eps = ar * eps + 1.f;
  // 7 digits of precision are insufficient, in either sform("%.7g") or ostream << setprecision(7).
  // 8 digits are mostly sufficient; e.g. not true of numbers between 1000 and 1024.
  // 1023.9932861328125f from https://randomascii.wordpress.com/2012/02/11/they-sure-look-equal/
  // printf("%1.8e\n", d);   // Round-trippable float, always with an exponent.
  // printf("%.9g\n", d);    // Round-trippable float, shortest possible.
  // printf("%1.16e\n", d);  // Round-trippable double, always with an exponent.
  // printf("%.17g\n", d);   // Round-trippable double, shortest possible.
  for (const float f : concatenate(ar1eps, V(1023.9932861328125f), V(1023.9933471679687f),
                                   V(0.2288884f, 0.228888392f, 0.228888407f))) {
    const float f2 = roundtrip(f, 9);  // Was 7; in principle 9 is required!  (17 is sufficient for double.)
    // %a (hexadecimal float) is not supported in mingw which uses old MS CRT
    showf("f=%-10.7g %-10.8g %-11.9g %x  f2=%-10.7g %-10.8g %-11.9g %x  f==f2=%d\n",  //
          f, f, f, as_uint(f), f2, f2, f2, as_uint(f2), f == f2);
  }
  // 0.2288884f in FrameIO_test.inp:
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

}  // namespace

int main() {
  {
    test(8, 1e-4f);
    test(4, 1e-6f);
    test(0, 0.f);
  }
  test_buckets();
  test_io();
}
