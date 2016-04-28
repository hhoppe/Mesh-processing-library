// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Hh.h"

namespace hh {

// Univ is my universal fundamental type, essentially a union of: { T*, int, unsigned, float } .
// It is not used much anymore.

class SUniv; using Univ = SUniv*;

static_assert(sizeof(Univ)>=sizeof(int), "");
static_assert(sizeof(Univ)>=sizeof(float), "");

// Templated helper class for conversion to/from Univ.
template<typename T> struct Conv;

template<typename T> struct Conv<T*> {
    static Univ e(T* v) { return Univ(v); }                 // encode
    static T* d(Univ v) { return reinterpret_cast<T*>(v); } // decode
};

template<> struct Conv<int> {
    static Univ e(int v) { return Univ(intptr_t(v)); }
    static int d(Univ v) { return narrow_cast<int>(intptr_t(v)); }
};

template<> struct Conv<unsigned> {
    static Univ e(unsigned v) { return Univ(uintptr_t(v)); }
    static unsigned d(Univ v) { return narrow_cast<unsigned>(uintptr_t(v)); }
};

template<> struct Conv<float> {
    static Univ e(float v) { union { uint32_t ui; float f; } u; u.f = v; return Conv<unsigned>::e(u.ui); }
    static float d(Univ v) { union { uint32_t ui; float f; } u; u.ui = Conv<unsigned>::d(v); return u.f; }
};

} // namespace hh
