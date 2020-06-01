// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_FLAGS_H_
#define MESH_PROCESSING_LIBHH_FLAGS_H_

#include "MathOp.h"             // is_pow2()

namespace hh {

class Flag;                     // forward reference

// Mask identifying an allocated flag within a Flags structure.
using FlagMask = unsigned;

// A structure containing a set (~32-64) of binary flags.
class Flags {
    using type = FlagMask;
    static constexpr int max_nflags = std::numeric_limits<type>::digits;
 public:
    Flags()                                     { }
    void operator=(type flags)                  { _flags = flags; }
    operator type() const                       { return _flags; }
    type set(type flags)                        { return std::exchange(_flags, flags); } // return previous
    Flag flag(FlagMask fmask);
    bool flag(FlagMask fmask) const             { return (_flags&fmask)!=0; }
    friend void swap(Flags& l, Flags& r) noexcept { std::swap(l._flags, r._flags); }
    static FlagMask allocate(int& counter)      { return (assertx(counter<max_nflags), 1u<<(counter++)); }
 private:
    type _flags {0};
    friend Flag;
};

// A reference to a binary flag in a Flags structure.
class Flag {
 public:
    void operator=(bool bset)                   { if (bset) _flags._flags |= _fmask; else _flags._flags &= ~_fmask; }
    void operator=(const Flag& flag)            { *this = bool(flag); } // for unambiguity in gcc 4.8.2 SurfEd.cpp
    Flag(const Flag&)                           = default;              // hence needed for clang
    operator bool() const                       { return (_flags._flags&_fmask)!=0; }
    bool set(bool bset)                         { bool t = *this; *this = bset; return t; } // return previous
 private:
    Flag(Flags& flags, FlagMask fmask)          : _flags(flags), _fmask(fmask) { ASSERTX(is_pow2(_fmask)); }
    Flags& _flags;
    const FlagMask _fmask;
    friend Flags;
};

//----------------------------------------------------------------------------

inline Flag Flags::flag(FlagMask fmask) { return Flag(*this, fmask); }

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_FLAGS_H_
