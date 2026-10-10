// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Random.h"

#include <random>

namespace hh {

Random Random::G;  // Initialized with a default seed.

class Random::Implementation {
  using Engine = std::mt19937;  // A standard mersenne_twister_engine, implementation-independent!
  Engine _engine;

 public:
  void seed(uint32_t seedv) {
    _engine.seed(seedv ^ Engine::default_seed);  // My default zero seed should map to the engine's default_seed.
  }
  uint32_t operator()() { return _engine(); }
  static constexpr uint32_t k_expected_first_value = 3'499'211'612;
};

int Random::g_init() {
  // This must run after construction of Random::G.
  assertx(G._impl);  // Ensure that G is initialized.
  const int seedv = getenv_int("SEED_RANDOM");
  if (seedv) {
    if (0) {
      // Note: the Warnings class is not yet initialized.
      Warning("SEED_RANDOM used");
    }
    assertx((*G._impl)() == Implementation::k_expected_first_value);  // Ensure that G has never been used.
    G.seed(seedv);
  }
  return 0;
}
int Random::_g_init = Random::g_init();

Random::Random(uint32_t seedv) { seed(seedv); }

Random::~Random() = default;  // Must be defined after "class Implementation".

void Random::seed(uint32_t seedv) {
  if (!_impl) _impl = make_unique<Implementation>();
  _impl->seed(seedv);
}

template <> uint32_t Random::get_int<4>() { return (*_impl)(); }

template <> uint64_t Random::get_int<8>() {
  const uint64_t v1 = get_int<4>();
  const uint64_t v2 = get_int<4>();
  return v1 | (v2 << 32);
}

// https://stackoverflow.com/questions/11603818/why-is-there-ambiguity-between-uint32-t-and-uint64-t-when-using-size-t-on-mac-os
//  Mac: using uint32_t = unsigned int; using uint64_t = unsigned long long; using size_t = unsigned long;
unsigned Random::get_unsigned() { return get_int<sizeof(unsigned)>(); }
uint64_t Random::get_uint64() { return get_int<sizeof(uint64_t)>(); }
size_t Random::get_size_t() { return get_int<sizeof(size_t)>(); }

Random::result_type Random::operator()() { return get_int<sizeof(result_type)>(); }

unsigned Random::get_unsigned(unsigned ub) {
  ASSERTX(ub);
  // Lemire's method (D. Lemire, "Fast random integer generation in an interval", ACM TOMACS 2019): the high 32 bits of
  // v * ub are a value in [0, ub - 1].  Each value arises from either floor(2^32 / ub) or that plus one values of v;
  // rejecting the v whose low 32 bits fall below 2^32 % ub leaves exactly floor(2^32 / ub) for each, so the result is
  // unbiased.  It usually needs no division: the remainder is computed only when the low bits are below ub.  It is as
  // fast as masking for a power-of-two ub, and faster than the earlier rejection method (draws below the largest
  // multiple of ub, then v % ub) except for ub above 2^31.
  uint64_t m = uint64_t{get_unsigned()} * ub;
  if (unsigned(m) < ub) {
    const unsigned threshold = (0u - ub) % ub;  // Equals 2^32 % ub.
    while (unsigned(m) < threshold) m = uint64_t{get_unsigned()} * ub;
  }
  return unsigned(m >> 32);
}

// The result approximates the nearest float (or double) to the uniform real (i + 1/2) / 2^32 (or (i + 1/2) / 2^64),
// for a random integer i.  It uses all the random bits, so its resolution is fine near zero (the smallest value is
// 2^-33 or 2^-65), which matters for transforms such as -log(u).  Its mean is within about 1e-10 of 1/2.
// The conversion of the integer to floating-point rounds the largest values (those within 2^7 of 2^32, or within 2^10
// of 2^64) up to the power of two, so the result is clamped to the largest representable value below 1.
// Alternatives considered:
// - float((i >> 8) | 1) * 2^-24, the midpoints of 2^23 equal bins: no rounding or clamp, and exactly symmetric with a
//   mean of exactly 1/2, but only 2^23 equally spaced values, so the smallest value is 2^-24.
// - float(i >> 8) * 2^-24: 2^24 equally spaced values, but it includes 0 and its mean is 1/2 - 2^-25.
// - A scale factor of (1 - 2^-24) * 2^-32 instead of the clamp: it also avoids 1, but changes nearly every value by
//   one ulp and lowers the mean by about 4e-8.
template <> float Random::get_unif<float>() {
  static const float unif_factor = pow(2.f, -32.f);
  constexpr float max_unif = 1.f - std::numeric_limits<float>::epsilon() / 2.f;  // Largest float below 1.f.
  return std::min(get_int<4>() * unif_factor + .5f * unif_factor, max_unif);
}

template <> double Random::get_unif<double>() {
  static const double unif_factor = pow(2., -64.);
  constexpr double max_unif = 1. - std::numeric_limits<double>::epsilon() / 2.;  // Largest double below 1.
  return std::min(get_int<8>() * unif_factor + .5 * unif_factor, max_unif);
}

float Random::unif() { return get_unif<float>(); }
double Random::dunif() { return get_unif<double>(); }

template <typename T> T Random::get_gauss() requires std::floating_point<T> {
  static_assert(std::is_floating_point_v<T>);
  // See experiments in test/opt/test_random.cpp (the Box-Muller transform is best).
  if (0) {
    const int k_ngauss = 10;  // Number of uniform randoms to obtain a Gaussian.
    const double gauss_factor = sqrt(12.) / sqrt(double(k_ngauss));
    double acc = 0.;
    for_int(i, k_ngauss) acc += get_unif<T>();
    return T((acc - k_ngauss * .5) * gauss_factor);
  } else if (0) {  // Unfortunately, implementation-dependent.
    static std::normal_distribution<T> distrib(T{0}, T{1});
    return distrib(*this);
  } else if (1) {
    // Generate two samples of N(0, 1) from two samples of U[0, 1] using the Box-Muller transformation.
    T s, v1, v2;
    do {
      v1 = T{2} * get_unif<T>() - T{1};
      v2 = T{2} * get_unif<T>() - T{1};
      s = v1 * v1 + v2 * v2;
    } while (s >= T{1} || s == T{0});
    // The 2D point (v1, v2) lies inside the unit-radius circle.
    T a = sqrt(T{-2} * std::log(s) / s);
    return a * v1;
    // (there is an additional random sample number: a * v2)
  }
}

float Random::gauss() { return get_gauss<float>(); }
double Random::dgauss() { return get_gauss<double>(); }

void Random::discard(uint64_t count) {
  while (count--) get_int<4>();
}

}  // namespace hh
