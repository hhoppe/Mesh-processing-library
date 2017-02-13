// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_HASHFLOAT_H_
#define MESH_PROCESSING_LIBHH_HASHFLOAT_H_

#include "Map.h"

namespace hh {

// Override parameters using getenv_int("HASHFLOAT_NIGNOREBITS") and getenv_float("HASHFLOAT_SMALL").

// Robustly hash similar floating-point values into quantized buckets, with special bucket for small values.
// The resulting equivalence classes are dependent on the order that values are inserted.
// This drawback may be reduced by using an initial pass of pre_consider() values.
class HashFloat : noncopyable {
 public:
    explicit HashFloat(int nignorebits = 8, float small = 1e-4f);
    float enter(float f);       // ret: filtered value
    void pre_consider(float f); // more robust pre-pass
 private:
    Map<uint32_t,float> _m;     // encoded float bucket -> float rep
    int _nignorebits;           // num of least significant bits to ignore in floating-poing representation
    float _small;               // numbers with abs<small are grouped at 0
    float _factor;              // used to access prev and next buckets
    float _ractor;              // 1/_factor
    uint32_t encode(float v) const;
};

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_HASHFLOAT_H_
