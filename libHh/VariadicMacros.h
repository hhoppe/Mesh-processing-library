// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_VARIADICMACROS_H_
#define MESH_PROCESSING_LIBHH_VARIADICMACROS_H_

// This file is included from Hh.h

// Return number of arguments.
#define HH_NUM_ARGS(...) HH_NUM_ARGS_((__VA_ARGS__, HH_NUM_ARGS_RSEQ_N()))
#define HH_NUM_ARGS_(tuple) HH_NUM_ARGS_B tuple
#define HH_NUM_ARGS_RSEQ_N() 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0
#define HH_NUM_ARGS_B(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, N, ...) N
#define HH_COMMA_SEQ_N() 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0
// https://stackoverflow.com/questions/9183993/msvc-variadic-macro-expansion (VC treats __VA_ARGS__ as single token)
// https://stackoverflow.com/questions/11317474/macro-to-count-number-of-arguments   (finding comma)

// Return 1 if there is more than one argument (i.e. has a comma), else 0.
#define HH_GT1_ARGS(...) HH_NUM_ARGS_((__VA_ARGS__, HH_COMMA_SEQ_N()))

// This will let macros expand before concatenating them.
#define HH_PRIMITIVE_CAT(tuple) HH_PRIMITIVE_CAT_ tuple
#define HH_PRIMITIVE_CAT_(x, y) HH_CAT(x, y)

// Call a MAPPING macro on each argument passed in, and separate results by a REDUCTION.
// Example:
// #define F_EACH(x) unsigned(x)
// #define F(...) HH_MAP_REDUCE((F_EACH, +, __VA_ARGS__))
#define HH_MAP_REDUCE(tuple) HH_MAP_REDUCE_ tuple
#define HH_MAP_REDUCE_(macro, reduction, ...) \
  HH_PRIMITIVE_CAT((HH_MAP_REDUCE_, HH_NUM_ARGS(__VA_ARGS__)))(macro, reduction, __VA_ARGS__)
#define HH_MAP_REDUCE_1(m, r, x1) m(x1)
#define HH_MAP_REDUCE_2(m, r, x1, x2) m(x1) r m(x2)
#define HH_MAP_REDUCE_3(m, r, x1, x2, x3) \
  m(x1) r m(x2)                           \
  r m(x3)
#define HH_MAP_REDUCE_4(m, r, x1, x2, x3, x4) \
  m(x1) r m(x2)                               \
  r m(x3)                                     \
  r m(x4)
#define HH_MAP_REDUCE_5(m, r, x1, x2, x3, x4, x5) \
  m(x1) r m(x2)                                   \
  r m(x3)                                         \
  r m(x4)                                         \
  r m(x5)
#define HH_MAP_REDUCE_6(m, r, x1, x2, x3, x4, x5, x6) \
  m(x1) r m(x2)                                       \
  r m(x3)                                             \
  r m(x4)                                             \
  r m(x5)                                             \
  r m(x6)
#define HH_MAP_REDUCE_7(m, r, x1, x2, x3, x4, x5, x6, x7) \
  m(x1) r m(x2)                                           \
  r m(x3)                                                 \
  r m(x4)                                                 \
  r m(x5)                                                 \
  r m(x6)                                                 \
  r m(x7)
#define HH_MAP_REDUCE_8(m, r, x1, x2, x3, x4, x5, x6, x7, x8) \
  m(x1) r m(x2)                                               \
  r m(x3)                                                     \
  r m(x4)                                                     \
  r m(x5)                                                     \
  r m(x6)                                                     \
  r m(x7)                                                     \
  r m(x8)
#define HH_MAP_REDUCE_9(m, r, x1, x2, x3, x4, x5, x6, x7, x8, x9) \
  m(x1) r m(x2)                                                   \
  r m(x3)                                                         \
  r m(x4)                                                         \
  r m(x5)                                                         \
  r m(x6)                                                         \
  r m(x7)                                                         \
  r m(x8)                                                         \
  r m(x9)
#define HH_MAP_REDUCE_10(m, r, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10) \
  m(x1) r m(x2)                                                         \
  r m(x3)                                                               \
  r m(x4)                                                               \
  r m(x5)                                                               \
  r m(x6)                                                               \
  r m(x7)                                                               \
  r m(x8)                                                               \
  r m(x9)                                                               \
  r m(x10)
#define HH_MAP_REDUCE_11(m, r, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11) \
  m(x1) r m(x2)                                                              \
  r m(x3)                                                                    \
  r m(x4)                                                                    \
  r m(x5)                                                                    \
  r m(x6)                                                                    \
  r m(x7)                                                                    \
  r m(x8)                                                                    \
  r m(x9)                                                                    \
  r m(x10)                                                                   \
  r m(x11)
#define HH_MAP_REDUCE_12(m, r, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12) \
  m(x1) r m(x2)                                                                   \
  r m(x3)                                                                         \
  r m(x4)                                                                         \
  r m(x5)                                                                         \
  r m(x6)                                                                         \
  r m(x7)                                                                         \
  r m(x8)                                                                         \
  r m(x9)                                                                         \
  r m(x10)                                                                        \
  r m(x11)                                                                        \
  r m(x12)

// Call a MACRO on each argument passed in (i.e. special case of HH_MAP_REDUCE where reduction is ",").
// Example:
// #define F_EACH(x) unsigned(x)
// #define F(...) HH_APPLY((F_EACH, __VA_ARGS__))
#define HH_APPLY(tuple) HH_APPLY_ tuple
#define HH_APPLY_(macro, ...) HH_PRIMITIVE_CAT((HH_APPLY_, HH_NUM_ARGS(__VA_ARGS__)))(macro, __VA_ARGS__)
#define HH_APPLY_1(m, x1) m(x1)
#define HH_APPLY_2(m, x1, x2) m(x1), m(x2)
#define HH_APPLY_3(m, x1, x2, x3) m(x1), m(x2), m(x3)
#define HH_APPLY_4(m, x1, x2, x3, x4) m(x1), m(x2), m(x3), m(x4)
#define HH_APPLY_5(m, x1, x2, x3, x4, x5) m(x1), m(x2), m(x3), m(x4), m(x5)
#define HH_APPLY_6(m, x1, x2, x3, x4, x5, x6) m(x1), m(x2), m(x3), m(x4), m(x5), m(x6)
#define HH_APPLY_7(m, x1, x2, x3, x4, x5, x6, x7) m(x1), m(x2), m(x3), m(x4), m(x5), m(x6), m(x7)
#define HH_APPLY_8(m, x1, x2, x3, x4, x5, x6, x7, x8) m(x1), m(x2), m(x3), m(x4), m(x5), m(x6), m(x7), m(x8)
#define HH_APPLY_9(m, x1, x2, x3, x4, x5, x6, x7, x8, x9) m(x1), m(x2), m(x3), m(x4), m(x5), m(x6), m(x7), m(x8), m(x9)
#define HH_APPLY_10(m, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10) \
  m(x1), m(x2), m(x3), m(x4), m(x5), m(x6), m(x7), m(x8), m(x9), m(x10)
#define HH_APPLY_11(m, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11) \
  m(x1), m(x2), m(x3), m(x4), m(x5), m(x6), m(x7), m(x8), m(x9), m(x10), m(x11)
#define HH_APPLY_12(m, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12) \
  m(x1), m(x2), m(x3), m(x4), m(x5), m(x6), m(x7), m(x8), m(x9), m(x10), m(x11), m(x12)

#endif  // MESH_PROCESSING_LIBHH_VARIADICMACROS_H_
