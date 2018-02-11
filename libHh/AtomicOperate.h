// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ATOMICOPERATE_H_
#define MESH_PROCESSING_LIBHH_ATOMICOPERATE_H_

#include "Hh.h"

#if 0
{
    // atomic operation
    Array<float> ar(100, 1.f);
    parallel_for_int(iter, 1000) { int i = func(iter); atomic_operate(ar[i], [](float v) { return v*.5f+.33f; }); }
}
#endif

namespace hh {
template<typename T, typename Func = T(T)> void atomic_operate(volatile T* ptr, Func func);
} // namespace hh


//----------------------------------------------------------------------------

namespace hh {

#if defined(__GNUC__) || defined(__clang__)

#define sync_val_compare_and_swap __sync_val_compare_and_swap

#else

inline char sync_val_compare_and_swap(volatile char* ptr, char old_value, char new_value) {
    return _InterlockedCompareExchange8(ptr, new_value, old_value);
}
inline short sync_val_compare_and_swap(volatile short* ptr, short old_value, short new_value) {
    return _InterlockedCompareExchange16(ptr, new_value, old_value);
}
inline int32_t sync_val_compare_and_swap(volatile int32_t* ptr, int32_t old_value, int32_t new_value) {
    return _InterlockedCompareExchange(reinterpret_cast<volatile long*>(ptr), new_value, old_value);
}
inline int64_t sync_val_compare_and_swap(volatile int64_t* ptr, int64_t old_value, int64_t new_value) {
    return _InterlockedCompareExchange64(ptr, new_value, old_value);
}

#endif

namespace details {
// Avoid GCC warning "dereferencing type-punned pointer will break strict-aliasing rules" for T=float or double.
template<typename T, typename S> T* force_pointer_cast(S* s) {
    static_assert(sizeof(T)==sizeof(S), "");
    return reinterpret_cast<T*>(s);
}
} // namespace details

// Atomically apply func (often a lambda function) to the value stored in *ptr (which can be floating-point).
template<typename T, typename Func> void atomic_operate(volatile T* ptr, Func func) {
    static_assert(sizeof(T)==1 || sizeof(T)==2 || sizeof(T)==4 || sizeof(T)==8, "");
    using Tint = std::conditional_t<sizeof(T)==1, char,
                                    std::conditional_t<sizeof(T)==2, short,
                                                       std::conditional_t<sizeof(T)==4, int32_t,
                                                                          int64_t>>>;
    T value = *ptr;
    Tint& inew_value = *(details::force_pointer_cast<Tint>(&value));
    Tint iold_value;
    // Note: there is http://en.cppreference.com/w/cpp/atomic/atomic_compare_exchange
    //  but it only operates on std::atomic<T> types.  Maybe there is a good reason.
    do {
        iold_value = inew_value;
        value = func(value);
        // Note that second and third arguments are swapped compared to _InterlockedCompareExchange*().
        inew_value = sync_val_compare_and_swap(reinterpret_cast<volatile Tint*>(ptr), iold_value, inew_value);
    } while (inew_value!=iold_value);
}

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_ATOMICOPERATE_H_
