// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HASHFLOAT_H_
#define MESH_PROCESSING_LIBHH_HASHFLOAT_H_

#include "libHh/Map.h"

namespace hh {

// Override parameters using getenv_int("HASHFLOAT_NIGNOREBITS") and getenv_float("HASHFLOAT_SMALL").

// Robustly hash similar floating-point values into quantized buckets, with special bucket for small values.
// The resulting equivalence classes are dependent on the order that values are inserted.
// This drawback may be reduced by using an initial pass of pre_consider() values.
class HashFloat : noncopyable {
 public:
  explicit HashFloat(int nignorebits = 8, float small = 1e-4f);
  [[nodiscard]] float enter(float f);  // Returns the filtered value.
  void pre_consider(float f);          // A more robust pre-pass.
 private:
  Map<uint32_t, float> _m;  // Encoded float bucket -> float representative.
  int _nignorebits;         // Number of least significant bits to ignore in the floating-point representation.
  float _small;             // Numbers with absolute value < _small are grouped at 0.
  float _factor;            // Used to access the previous and next buckets.
  float _recip;             // 1 / _factor
  [[nodiscard]] uint32_t encode(float f) const;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_HASHFLOAT_H_
