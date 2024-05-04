// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_RANDOM_H_
#define MESH_PROCESSING_LIBHH_RANDOM_H_

#include "libHh/Vec.h"

namespace hh {

// getenv_int("SEED_RANDOM") --> overrides default seed 0 for Random::G

// Platform-independent deterministic random number generator.
// (This is to overcome the fact that the mappings from the raw std::*_engine() to
//   most distributions, including floating-point types, is implementation-dependent.)
class Random : noncopyable {
 public:
  static Random G;  // can be uninitialized before main()!  any shared Random is not thread-safe.
  explicit Random(uint32_t seedv = 0);
  ~Random();
  void seed(uint32_t seedv);
  unsigned get_unsigned();  // [0, std::numeric_limits<unsigned>::max()]
  uint64_t get_uint64();    // [0, std::numeric_limits<uint64_t>::max()]
  size_t get_size_t();      // range depends on size_t
  unsigned get_unsigned(unsigned ub);  // [0, ub - 1]
  float unif();                        // [0.f, 1.f)
  double dunif();                      // [0., 1.)
  float gauss();                       // avg = 0.f, sdv = 1.f
  double dgauss();                     // avg = 0.,  sdv = 1.
  void discard(uint64_t count);
  // http://en.cppreference.com/w/cpp/concept/UniformRandomNumberGenerator (std::*_engine classes):
  using result_type = uint32_t;
  result_type operator()();
  static_assert(std::is_integral<result_type>::value, "");
  static constexpr result_type min() { return std::numeric_limits<result_type>::min(); }
  static constexpr result_type max() { return std::numeric_limits<result_type>::max(); }
  static constexpr result_type default_seed = 0;

 private:
  class Implementation;
  unique_ptr<Implementation> _impl;
  template <size_t size> std::conditional_t<size == 4, uint32_t, uint64_t> get_int();  // size == 4 or size == 8
  template <typename T> T get_unif();                                                  // float and double
  template <typename T> T get_gauss();                                                 // float and double
  static int g_init();
  static int _g_init;
};

// Randomly shuffle the elements in an array view.
template <typename T> void shuffle(ArrayView<T> ar, Random& r);

//----------------------------------------------------------------------------

template <typename T> void shuffle(ArrayView<T> ar, Random& r) {
  // std::default_random_engine e; std::shuffle(ar.begin(), ar.end(), e);  // implementation-dependent
  // std::mt19937_64 e;            std::shuffle(ar.begin(), ar.end(), e);  // implementation-dependent
  const int n = ar.num();
  for_int(i, n - 1) {
    int j = i + r.get_unsigned(n - i);
    if (j != i) {
      using std::swap;
      swap(ar[i], ar[j]);
    }
  }
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_RANDOM_H_
