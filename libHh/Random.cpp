// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Random.h"

#include <random>

namespace hh {

Random Random::G;  // initialized with a default seed

class Random::Implementation {
  using Engine = std::mt19937;  // a standard mersenne_twister_engine, implementation-independent!
  Engine _engine;

 public:
  void seed(uint32_t seedv) {
    _engine.seed(seedv ^ Engine::default_seed);  // my default zero seed should map to engine's default_seed
  }
  uint32_t operator()() { return _engine(); }
  static constexpr uint32_t k_expected_first_value = 3499211612;
};

int Random::g_init() {
  // This must run after construction of Random::G.
  assertx(G._impl);  // ensure that G is initialized
  const int seedv = getenv_int("SEED_RANDOM");
  if (seedv) {
    if (0) {
      // Note: Warnings class is not yet initialized
      Warning("SEED_RANDOM used");
    }
    assertx((*G._impl)() == Implementation::k_expected_first_value);  // ensure that G has never been used
    G.seed(seedv);
  }
  return 0;
}
int Random::_g_init = Random::g_init();

Random::Random(uint32_t seedv) { seed(seedv); }

Random::~Random() {
  // must be defined after "class Implementation"
}

void Random::seed(uint32_t seedv) {
  if (!_impl) _impl = make_unique<Implementation>();
  _impl->seed(seedv);
}

template <> inline uint32_t Random::get_int<4>() { return (*_impl)(); }

template <> inline uint64_t Random::get_int<8>() {
  uint64_t v = get_int<4>();
  return v | (static_cast<uint64_t>(get_int<4>()) << 32);
}

// http://stackoverflow.com/questions/11603818/why-is-there-ambiguity-between-uint32-t-and-uint64-t-when-using-size-t-on-mac-os
//  Mac: using uint32_t = unsigned int; using uint64_t = unsigned long long; using size_t = unsigned long;
unsigned Random::get_unsigned() { return get_int<sizeof(unsigned)>(); }
uint64_t Random::get_uint64() { return get_int<sizeof(uint64_t)>(); }
size_t Random::get_size_t() { return get_int<sizeof(size_t)>(); }

Random::result_type Random::operator()() { return get_int<sizeof(result_type)>(); }

unsigned Random::get_unsigned(unsigned ub) {
  ASSERTX(ub);
  if (0) {  // unfortunately, implementation-dependent
    std::uniform_int_distribution<unsigned> distrib(0, ub - 1);
    return distrib(*this);
  } else {
    const bool ub_is_pow2 = (ub & (ub - 1)) == 0;
    if (ub_is_pow2) return get_unsigned() & (ub - 1);                   // fast case: no need for loop or remainder
    const unsigned nspans = std::numeric_limits<unsigned>::max() / ub;  // number of whole spans of length ub
    const unsigned maxv = nspans * ub;
    for (;;) {
      unsigned v = get_unsigned();
      if (v >= maxv) continue;
      return v % ub;
    }
  }
}

template <> inline float Random::get_unif<float>() {
  const float unif_factor = pow(2.f, -32.f);
  return get_int<4>() * unif_factor + .5f * unif_factor;
}

template <> inline double Random::get_unif<double>() {
  const double unif_factor = pow(2., -64.);
  return get_int<8>() * unif_factor + .5 * unif_factor;
}

float Random::unif() { return get_unif<float>(); }
double Random::dunif() { return get_unif<double>(); }

template <typename T> T Random::get_gauss() {
  // See experiments in test/opt/test_random.cpp   (Box-Muller transform is best)
  if (0) {
    const int k_ngauss = 10;  // number of uniform randoms to obtain Gaussian
    const double gauss_factor = sqrt(12.) / sqrt(double(k_ngauss));
    double acc = 0.;
    for_int(i, k_ngauss) acc += get_unif<T>();
    return static_cast<T>((acc - k_ngauss * .5) * gauss_factor);
  } else if (0) {  // unfortunately, implementation-dependent
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
    // (there is an additional random sample number: a*v2)
  }
}

float Random::gauss() { return get_gauss<float>(); }
double Random::dgauss() { return get_gauss<double>(); }

void Random::discard(uint64_t count) {
  while (count--) get_int<4>();
}

}  // namespace hh
