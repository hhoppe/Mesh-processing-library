// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "HashFloat.h"

namespace hh {

// Idea:
// Each bucket will contain a unique representative floating point number.
// When entering a number into an empty bucket, its neighbors are tested first; if one of its neighbors
//  already has a FP number, the new bucket inherits this number.
// -> equivalence relation is correct.
// Small numbers (abs(x)<threshold) are placed in a special bucket that is
// adjacent to numbers that are almost small.

// TODO: Exploit the fact that "Adjacent floats (of the same sign) have adjacent integer representations".
//  See http://randomascii.wordpress.com/2012/01/23/stupid-float-tricks-2/

namespace {

constexpr uint32_t k_small_key = 1;
constexpr float k_small_val = 1e-30f;

inline float compute_factor(int n) { return 1.f+pow(.5f, 23.f-n)*.49999f; }

inline uint32_t float_bits_to_unsigned(const float& f) {
    union { uint32_t ui; float f; } u; u.f = f; return u.ui;
}

} // namespace

HashFloat::HashFloat(int nignorebits, float small) : _nignorebits(nignorebits), _small(small) {
    // use float to detect override with "0"
    _nignorebits = getenv_int("HASHFLOAT_NIGNOREBITS", _nignorebits, true);
    assertx(_nignorebits>=0 && _nignorebits<=22);
    _small = getenv_float("HASHFLOAT_SMALL", _small, true);
    _factor = compute_factor(_nignorebits);
    _ractor = 1.f/_factor;
}

// Return a key that encodes the bucket in which the value lies.
inline uint32_t HashFloat::encode(float f) const {
    if (abs(f)<=_small) {
        return k_small_key;
    } else {
        uint32_t u = float_bits_to_unsigned(f);
        return assertx(u>>_nignorebits);
    }
}

float HashFloat::enter(float f) {
#if 1
    bool foundexact = false;
    uint32_t bucketn = encode(f);
    float r = _m.retrieve(bucketn); // retrieve closest float
    if (r) foundexact = true;
    if (0) {
        SHOW(encode(f/_factor/_factor));
        SHOW(encode(f/_factor));
        SHOW(encode(f));
        SHOW(encode(f*_factor));
        SHOW(encode(f*_factor*_factor));
    }
    if (!r) r = _m.retrieve(encode(f*_factor));
    if (!r) r = _m.retrieve(encode(f/_factor));
    if (!r) r = _m.retrieve(encode(f*_factor*_factor));
    if (!r) r = _m.retrieve(encode(f/_factor/_factor));
    if (r) {                    // found
        // if found in adjacent cell, propagate close value here
        if (!foundexact) _m.enter(bucketn, r);
        if (r==k_small_val) r = 0.f;
        return r;
    }
    float fe = f;
    if (bucketn==k_small_key) { fe = k_small_val; f = 0.f; }
    _m.enter(bucketn, fe);
    return f;
#else
    if (0 && getenv_bool("HASHFLOAT_DEBUG")) {
#define DEBUG_SHOW(x) showf(#x ## ": %.9g -> %d -> %.9g\n", x, encode(x), _m.retrieve(encode(x)))
        DEBUG_SHOW(f*_ractor*_ractor);
        DEBUG_SHOW(f*_ractor);
        DEBUG_SHOW(f);
        DEBUG_SHOW(f*_factor);
        DEBUG_SHOW(f*_factor*_factor);
    }
    uint32_t bc = encode(f);
    float r = _m.retrieve(bc);  // retrieve closest float
    uint32_t bp = encode(f*_factor); if (bp==bc) bp = encode(f*_factor*_factor);
    uint32_t bm = encode(f*_ractor); if (bm==bc) bm = encode(f*_ractor*_ractor);
    float rp = _m.retrieve(bp);
    float rm = _m.retrieve(bm);
    if (r) {
        if (rp && abs(f-rp)<abs(f-r)) r = rp;
        if (rm && abs(f-rm)<abs(f-r)) r = rm;
        return r==k_small_val ? 0 : r;
    }
    if (rp && rm) {
        Warning("HashFloat: Possible problem; try narrower buckets");
        r = abs(f-rp)<abs(f-rm) ? rp : rm;
    } else if (rp) {
        r = rp;
    } else if (rm) {
        r = rm;
    }
    if (r) {                    // found
        // if found in adjacent cell, propagate close value here
        _m.enter(bc, r);
        return r==k_small_val ? 0 : r;
    }
    float fe = f;
    if (bc==k_small_key) { fe = k_small_val; f = 0.f; }
    _m.enter(bc, fe);
    return f;
#endif
}

void HashFloat::pre_consider(float f) {
    uint32_t bc = encode(f);
    float r = _m.retrieve(bc);  // retrieve closest float
    if (r) return;
    uint32_t bp = encode(f*_factor); if (bp==bc) bp = encode(f*_factor*_factor);
    uint32_t bm = encode(f*_ractor); if (bm==bc) bm = encode(f*_ractor*_ractor);
    float rp = _m.retrieve(bp);
    float rm = _m.retrieve(bm);
    if (rp && rm) {
        Warning("HashFloat: Performing unification");
        assertx(bc!=k_small_key);  // I have not thought of that case yet.
        _m.enter(bc, f);
        for (float ff = f; ; ) {
            ff *= _factor; uint32_t b = encode(ff); assertx(b!=k_small_key);
            if (!_m.retrieve(b)) break;
            _m.replace(b, f);
        }
        for (float ff = f; ; ) {
            ff *= _ractor; uint32_t b = encode(ff); assertx(b!=k_small_key);
            if (!_m.retrieve(b)) break;
            _m.replace(b, f);
        }
    } else if (rp) {
        _m.enter(bc, rp);
    } else if (rm) {
        _m.enter(bc, rm);
    } else {
        _m.enter(bc, bc==k_small_key ? k_small_val : f);
    }
}

} // namespace hh
