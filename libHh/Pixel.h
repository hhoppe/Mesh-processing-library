// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Vec.h"

namespace hh {

// A pixel using unsigned char for each of four channels: red, green, blue, alpha.
// It is most often used in conjunction with class Image.
struct Pixel : Vec4<uchar> {
    Pixel()                                             = default;
    CONSTEXPR Pixel(uchar r, uchar g, uchar b, uchar a) : Vec4<uchar>(r, g, b, a) { }
    CONSTEXPR2 Pixel(uchar r, uchar g, uchar b)         : Pixel(r, g, b, 255) { }
    CONSTEXPR Pixel(Vec4<uchar> p)                      : Vec4<uchar>(std::move(p)) { }
    Pixel to_BGRA() const                       { return Pixel((*this)[2], (*this)[1], (*this)[0], (*this)[3]); }
    Pixel from_BGRA() const                     { return Pixel((*this)[2], (*this)[1], (*this)[0], (*this)[3]); }
    static CONSTEXPR2 Pixel gray(uchar v)               { return Pixel(v, v, v); }
    static CONSTEXPR2 Pixel white()                     { return Pixel::gray(255); }
    static CONSTEXPR2 Pixel black()                     { return Pixel::gray(0); }
    static CONSTEXPR2 Pixel red()                       { return Pixel(255, 0, 0); }
    static CONSTEXPR2 Pixel green()                     { return Pixel(0, 255, 0); }
    static CONSTEXPR2 Pixel blue()                      { return Pixel(0, 0, 255); }
    static CONSTEXPR2 Pixel pink()                      { return Pixel(255, 150, 150); }
    static CONSTEXPR2 Pixel yellow()                    { return Pixel(255, 255, 0); }
    friend std::ostream& operator<<(std::ostream& os, const Pixel& p) { // otherwise prints uchars
        return os << "Pixel(" << int(p[0]) << ", " << int(p[1]) << ", " << int(p[2]) << ", " << int(p[3]) << ")";
    }
};

template<typename R, typename = enable_if_range_t<R> > void swap_rgb_bgr(R&& range) {
    static_assert(std::is_same<iterator_t<R>, Pixel>::value, "");
    for (Pixel& p : range) std::swap(p[0], p[2]);
}


//----------------------------------------------------------------------------

// Faster, specialized version of Vec<>::operator==().
inline bool operator==(const Pixel& pix1, const Pixel& pix2) {
    return *reinterpret_cast<const uint32_t*>(&pix1) == *reinterpret_cast<const uint32_t*>(&pix2);
}

// Faster, specialized version of Vec<>::operator!=().
inline bool operator!=(const Pixel& pix1, const Pixel& pix2) { return !(pix1==pix2); }

} // namespace hh
