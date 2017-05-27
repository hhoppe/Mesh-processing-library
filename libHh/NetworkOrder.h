// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_NETWORKORDER_H_
#define MESH_PROCESSING_LIBHH_NETWORKORDER_H_

#include "Hh.h"

namespace hh {

// Convert between native byte ordering and network byte ordering.
// 
// Notes:
// - Network order is Big Endian (MSB first).  I use that for most of my binary files.
//
// - Intel_x86 and VAX are Little Endian
// - RISC is mostly Big Endian.  ARM is both.  All ARM versions of Windows run Little Endian

// The Internet Protocol defines big-endian as the standard network byte order used for all numeric values in
//  the packet headers and for many higher level protocols and file formats that are designed for use over IP.

// Big Endian is natural for dates/times (2014-12-22 12:34:56).
// I also use it for grid access (matrix[y][x]==matrix[yx]; matrix.dims()==V(matrix.ysize(), matrix.xsize())) and
//   for screen coordinates (const Vec2<int>& yx).

#if defined(__GNUC__)

inline uint64_t swap_8bytes(uint64_t v) { return __builtin_bswap64(v); }
inline uint32_t swap_4bytes(uint32_t v) { return __builtin_bswap32(v); }
#if __GNUC__*100+__GNUC_MINOR__>=408
inline uint16_t swap_2bytes(uint16_t v) { return __builtin_bswap16(v); }
#else
inline uint16_t swap_2bytes(uint16_t v) { return static_cast<uint16_t>((v>>8) | (v<<8)); }
#endif

#elif defined(_MSC_VER)

#include <cstdlib>
inline uint64_t swap_8bytes(uint64_t v) { return _byteswap_uint64(v); }
inline uint32_t swap_4bytes(uint32_t v) { return _byteswap_ulong(v); }
inline uint16_t swap_2bytes(uint16_t v) { return _byteswap_ushort(v); }
// also #include "immintrin.h": int _bswap(int); int64_t _bswap64(int64_t);

#else

inline uint64_t swap_8bytes(uint64_t v) {
    return (((v)                    >> 56) |
            ((v&0x00FF000000000000) >> 40) |
            ((v&0x0000FF0000000000) >> 24) |
            ((v&0x000000FF00000000) >>  8) |
            ((v&0x00000000FF000000) <<  8) |
            ((v&0x0000000000FF0000) << 24) |
            ((v&0x000000000000FF00) << 40) |
            ((v)                    << 56));
}
inline uint32_t swap_4bytes(uint32_t v) {
    return (((v)            >> 24) |
            ((v&0x00FF0000) >>  8) |
            ((v&0x0000FF00) <<  8) |
            ((v)            << 24));
}
inline uint16_t swap_2bytes(uint16_t v) {
    return ((v >> 8) |
            (v << 8));
}

#endif


// See http://stackoverflow.com/questions/2100331/c-macro-definition-to-determine-big-endian-or-little-endian-machine
// #define HH_LITTLE_ENDIAN 0x41424344UL
// #define HH_BIG_ENDIAN    0x44434241UL
// #define HH_ENDIAN_ORDER  ('ABCD')  // GCC: error: multi-character character constant [-Werror=multichar]
// #define HH_IS_BIG_ENDIAN ENDIAN_ORDER==BIG_ENDIAN
// #define HH_IS_BIG_ENDIAN (*(uint16_t *)"\0\xff" < 0x100) // Error
// no satisfactory portable solution for #if preprocessor macro

// #define HH_IS_BIG_ENDIAN (1 != *(unsigned char *)&(const int){1})
#define HH_IS_BIG_ENDIAN (*reinterpret_cast<const uint16_t*>("\0\xff") < 0x100)
// It does not seem possible to determine endianness in a constexpr:
//  http://stackoverflow.com/questions/1583791/constexpr-and-endianness

template<typename T> void my_swap_bytes(T* p) {
    static_assert(sizeof(T)==8 || sizeof(T)==4 || sizeof(T)==2, "");
    // The "union" are required for gcc 4.8.1; otherwise it changes value in memory but not in register
    //  -- see tNetworkOrder.cpp
    // I first tried making "T* p" be volatile, but that hit another compiler bug in FrameIO.cpp
    //  exposed in tFrameIO.cpp (the zoom variable is incorrect) and tA3dStream.cpp .
    if (sizeof(T)==8) {
        union { uint64_t ui; T t; } u; u.t = *p; u.ui = swap_8bytes(u.ui); *p = u.t;
    }
    if (sizeof(T)==4) {
        union { uint32_t ui; T t; } u; u.t = *p; u.ui = swap_4bytes(u.ui); *p = u.t;
    }
    if (sizeof(T)==2) {
        union { uint16_t ui; T t; } u; u.t = *p; u.ui = swap_2bytes(u.ui); *p = u.t;
    }
}

static const bool k_is_big_endian = HH_IS_BIG_ENDIAN;

// Convert from native to network order.
template<typename T> void to_std(T* p) {
    if (!k_is_big_endian) my_swap_bytes(p);
}

// Convert from network order to native order.
template<typename T> void from_std(T* p) { to_std(p); }

// Convert from to native order to DOS order.
template<typename T> void to_dos(T* p) {
    if (k_is_big_endian) my_swap_bytes(p);
}

// Convert from DOS order to native order.
template<typename T> void from_dos(T* p) { to_dos(p); }

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_NETWORKORDER_H_
