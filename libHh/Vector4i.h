// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_VECTOR4I_H_
#define MESH_PROCESSING_LIBHH_VECTOR4I_H_

#include "Vector4.h"

// SSE4.1: _mm_cvtepu8_epi32(), _mm_mullo_epi32()

namespace hh {

// Abstraction of a 4-int vector, hopefully accelerated by vectorized CPU instructions.
class Vector4i {
  using type = Vector4i;

 public:
  Vector4i() = default;  // could be: { fill(0); }
  explicit Vector4i(int j) { fill(j); }
  int& operator[](int i) { return (HH_CHECK_BOUNDS(i, 4), _c[i]); }
  const int& operator[](int i) const { return (HH_CHECK_BOUNDS(i, 4), _c[i]); }
  Vector4i with(int i, int j) const {
    HH_CHECK_BOUNDS(i, 4);
    Vector4i v = *this;
    v[i] = j;
    return v;
  }
  friend Vector4i operator-(const Vector4i& l) { return Vector4i(0) - l; }
  using value_type = int;
  using iterator = const int*;
  using const_iterator = const int*;
  const int* begin() const { return _c; }
  const int* end() const { return _c + 4; }
  int* data() { return _c; }
  const int* data() const { return _c; }
  friend std::ostream& operator<<(std::ostream& os, const Vector4i& v) {
    return os << "Vector4i(" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << ")";
  }
  static bool ok(int i) { return i >= 0 && i < 4; }
#if defined(HH_VECTOR4_SSE)
  Vector4i(const Vector4i& v) : _r(v._r) {}
  Vector4i(int x, int y, int z, int w) { _r = _mm_set_epi32(w, z, y, x); }  // note reverse ordering
  explicit Vector4i(const Pixel& pix) {
#if defined(HH_NO_SSE41)
    for_int(c, 4) _c[c] = pix[c];
#else
    __m128i in = _mm_cvtsi32_si128(reinterpret_cast<const int&>(pix));
    // same: __m128i in = _mm_castps_si128(_mm_load_ss(reinterpret_cast<const float*>(pix.data())));
    _r = _mm_cvtepu8_epi32(in);          // expand 4 unsigned 8-bit to 4 unsigned 32-bit (SSE4.1)
#endif
  }
  Pixel pixel() const {
    Pixel pix;
    __m128i t2 = _mm_packs_epi32(_r, _r);   // 8 signed 32-bit -> 8 signed 16-bit (saturation)
    __m128i t3 = _mm_packus_epi16(t2, t2);  // 16 signed 16-bit -> 16 unsigned 8-bit (saturation)
    reinterpret_cast<int&>(pix) = _mm_cvtsi128_si32(t3);
    // worse: _mm_store_ss(reinterpret_cast<float*>(pix.data()), _mm_castsi128_ps(t3));
    return pix;
  }
  void load_unaligned(const int* pSrc) { _r = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pSrc)); }
  void store_unaligned(int* pDst) const { _mm_storeu_si128(reinterpret_cast<__m128i*>(pDst), _r); }
  void load_aligned(const int* pSrc) { _r = _mm_load_si128(reinterpret_cast<const __m128i*>(pSrc)); }
  void store_aligned(int* pDst) const { _mm_store_si128(reinterpret_cast<__m128i*>(pDst), _r); }
  friend Vector4i operator+(const Vector4i& l, const Vector4i& r) { return _mm_add_epi32(l._r, r._r); }
  friend Vector4i operator-(const Vector4i& l, const Vector4i& r) { return _mm_sub_epi32(l._r, r._r); }
  friend Vector4i operator*(const Vector4i& l, const Vector4i& r) {
#if defined(HH_NO_SSE41)
    return Vector4i(l[0] * r[0], l[1] * r[1], l[2] * r[2], l[3] * r[3]);
#else
    return _mm_mullo_epi32(l._r, r._r);  // SSE4.1
#endif
  }
  friend Vector4i operator+(const Vector4i& v, int i) { return _mm_add_epi32(v._r, _mm_set1_epi32(i)); }
  friend Vector4i operator-(const Vector4i& v, int i) { return _mm_sub_epi32(v._r, _mm_set1_epi32(i)); }
  friend Vector4i operator*(const Vector4i& v, int i) {
#if defined(HH_NO_SSE41)
    return Vector4i(v[0] * i, v[1] * i, v[2] * i, v[3] * i);
#else
    return _mm_mullo_epi32(v._r, _mm_set1_epi32(i));
#endif
  }
  Vector4i& operator=(const Vector4i& r) {
    _r = r._r;
    return *this;
  }
  void fill(int v) { _r = _mm_set1_epi32(v); }
  friend Vector4i min(const Vector4i& l, const Vector4i& r) { return _mm_min_epi32(l._r, r._r); }
  friend Vector4i max(const Vector4i& l, const Vector4i& r) { return _mm_max_epi32(l._r, r._r); }
  friend Vector4i operator&(const Vector4i& l, const Vector4i& r) { return _mm_and_si128(l._r, r._r); }
  friend Vector4i operator|(const Vector4i& l, const Vector4i& r) { return _mm_or_si128(l._r, r._r); }
  friend Vector4i operator^(const Vector4i& l, const Vector4i& r) { return _mm_xor_si128(l._r, r._r); }
  friend Vector4i operator<<(const Vector4i& l, int n) { return _mm_slli_epi32(l._r, n); }
  friend Vector4i operator>>(const Vector4i& l, int n) { return _mm_srai_epi32(l._r, n); }
  friend Vector4i abs(const Vector4i& l) { return _mm_abs_epi32(l._r); }
#if !(defined(_M_X64) || defined(__x86_64))
  // "new type[size]" does not create aligned storage -- problem for Vector4i in 32-bit model
  static void* operator new(size_t s) { return aligned_malloc(s, alignof(type)); }
  static void operator delete(void* p, size_t) { aligned_free(p); }
  static void* operator new[](size_t s) { return aligned_malloc(s, alignof(type)); }
  static void operator delete[](void* p, size_t) { aligned_free(p); }
#endif
 private:
  Vector4i(__m128i v) : _r(v) {}
  Vector4i& operator=(const __m128i& r) {
    _r = r;
    return *this;
  }
  union {
    __m128i _r;
    int _c[4];
  };
#elif defined(HH_VECTOR4_NEON)
  // TODO: implement these as Neon intrinsics.
  Vector4i(const Vector4i& v) {
    for_int(c, 4) _c[c] = v._c[c];
  }
  Vector4i(int x, int y, int z, int w) { _c[0] = x, _c[1] = y, _c[2] = z, _c[3] = w; }
  explicit Vector4i(const Pixel& pix) {
    for_int(c, 4) _c[c] = pix[c];
  }
  Pixel pixel() const {
    Pixel v;
    for_int(c, 4) v[c] = clamp_to_uint8(_c[c]);
    return v;
  }
  void load_unaligned(const int* pSrc) {
    for_int(c, 4) _c[c] = pSrc[c];
  }
  void store_unaligned(int* pDst) const {
    for_int(c, 4) pDst[c] = _c[c];
  }
  void load_aligned(const int* pSrc) {
    for_int(c, 4) _c[c] = pSrc[c];
  }
  void store_aligned(int* pDst) const {
    for_int(c, 4) pDst[c] = _c[c];
  }
  friend Vector4i operator+(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] + r[0], l[1] + r[1], l[2] + r[2], l[3] + r[3]);
  }
  friend Vector4i operator-(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] - r[0], l[1] - r[1], l[2] - r[2], l[3] - r[3]);
  }
  friend Vector4i operator*(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] * r[0], l[1] * r[1], l[2] * r[2], l[3] * r[3]);
  }
  friend Vector4i operator+(const Vector4i& v, int i) { return v + Vector4i(i); }
  friend Vector4i operator-(const Vector4i& v, int i) { return v - Vector4i(i); }
  friend Vector4i operator*(const Vector4i& v, int i) { return v * Vector4i(i); }  // vmulq_n_s32(v._r, i);
  Vector4i& operator=(const Vector4i& r) {
    for_int(c, 4) _c[c] = r._c[c];
    return *this;
  }
  void fill(int v) {
    for_int(c, 4) _c[c] = v;
  }
  friend Vector4i min(const Vector4i& l, const Vector4i& r) {
    return Vector4i(min(l[0], r[0]), min(l[1], r[1]), min(l[2], r[2]), min(l[3], r[3]));
  }
  friend Vector4i max(const Vector4i& l, const Vector4i& r) {
    return Vector4i(max(l[0], r[0]), max(l[1], r[1]), max(l[2], r[2]), max(l[3], r[3]));
  }
  friend Vector4i operator&(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] & r[0], l[1] & r[1], l[2] & r[2], l[3] & r[3]);
  }
  friend Vector4i operator|(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] | r[0], l[1] | r[1], l[2] | r[2], l[3] | r[3]);
  }
  friend Vector4i operator^(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] ^ r[0], l[1] ^ r[1], l[2] ^ r[2], l[3] ^ r[3]);
  }
  friend Vector4i operator<<(const Vector4i& l, int n) { return Vector4i(l[0] << n, l[1] << n, l[2] << n, l[3] << n); }
  friend Vector4i operator>>(const Vector4i& l, int n) { return Vector4i(l[0] >> n, l[1] >> n, l[2] >> n, l[3] >> n); }
  friend Vector4i abs(const Vector4i& l) { return Vector4i(abs(l[0]), abs(l[1]), abs(l[2]), abs(l[3])); }

 private:
  Vector4i(int32x4_t v) : _r(v) {}
  Vector4i& operator=(const int32x4_t& r) {
    _r = r;
    return *this;
  }
  union {
    int32x4_t _r;
    int _c[4];
  };

#else   // neither defined(HH_VECTOR4_SSE) nor defined(HH_VECTOR4_NEON)
  Vector4i(const Vector4i& v) {
    for_int(c, 4) _c[c] = v._c[c];
  }
  Vector4i(int x, int y, int z, int w) { _c[0] = x, _c[1] = y, _c[2] = z, _c[3] = w; }
  explicit Vector4i(const Pixel& pix) {
    for_int(c, 4) _c[c] = pix[c];
  }
  Pixel pixel() const {
    Pixel v;
    for_int(c, 4) v[c] = clamp_to_uint8(_c[c]);
    return v;
  }
  void load_unaligned(const int* pSrc) {
    for_int(c, 4) _c[c] = pSrc[c];
  }
  void store_unaligned(int* pDst) const {
    for_int(c, 4) pDst[c] = _c[c];
  }
  void load_aligned(const int* pSrc) {
    for_int(c, 4) _c[c] = pSrc[c];
  }
  void store_aligned(int* pDst) const {
    for_int(c, 4) pDst[c] = _c[c];
  }
  friend Vector4i operator+(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] + r[0], l[1] + r[1], l[2] + r[2], l[3] + r[3]);
  }
  friend Vector4i operator-(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] - r[0], l[1] - r[1], l[2] - r[2], l[3] - r[3]);
  }
  friend Vector4i operator*(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] * r[0], l[1] * r[1], l[2] * r[2], l[3] * r[3]);
  }
  friend Vector4i operator+(const Vector4i& v, int i) { return v + Vector4i(i); }
  friend Vector4i operator-(const Vector4i& v, int i) { return v - Vector4i(i); }
  friend Vector4i operator*(const Vector4i& v, int i) { return v * Vector4i(i); }
  Vector4i& operator=(const Vector4i& r) {
    for_int(c, 4) _c[c] = r._c[c];
    return *this;
  }
  void fill(int v) {
    for_int(c, 4) _c[c] = v;
  }
  friend Vector4i min(const Vector4i& l, const Vector4i& r) {
    return Vector4i(min(l[0], r[0]), min(l[1], r[1]), min(l[2], r[2]), min(l[3], r[3]));
  }
  friend Vector4i max(const Vector4i& l, const Vector4i& r) {
    return Vector4i(max(l[0], r[0]), max(l[1], r[1]), max(l[2], r[2]), max(l[3], r[3]));
  }
  friend Vector4i operator&(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] & r[0], l[1] & r[1], l[2] & r[2], l[3] & r[3]);
  }
  friend Vector4i operator|(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] | r[0], l[1] | r[1], l[2] | r[2], l[3] | r[3]);
  }
  friend Vector4i operator^(const Vector4i& l, const Vector4i& r) {
    return Vector4i(l[0] ^ r[0], l[1] ^ r[1], l[2] ^ r[2], l[3] ^ r[3]);
  }
  friend Vector4i operator<<(const Vector4i& l, int n) { return Vector4i(l[0] << n, l[1] << n, l[2] << n, l[3] << n); }
  friend Vector4i operator>>(const Vector4i& l, int n) { return Vector4i(l[0] >> n, l[1] >> n, l[2] >> n, l[3] >> n); }
  friend Vector4i abs(const Vector4i& l) { return Vector4i(abs(l[0]), abs(l[1]), abs(l[2]), abs(l[3])); }

 private:
  int _c[4];
#endif  // defined(HH_VECTOR4_SSE) or defined(HH_VECTOR4_NEON)
};

Vector4i operator+(const Vector4i& l, const Vector4i& r);
Vector4i operator-(const Vector4i& l, const Vector4i& r);
Vector4i operator*(const Vector4i& l, const Vector4i& r);
Vector4i operator+(const Vector4i& v, int i);
Vector4i operator-(const Vector4i& v, int i);
Vector4i operator*(const Vector4i& v, int i);
Vector4i operator&(const Vector4i& l, const Vector4i& r);
Vector4i operator|(const Vector4i& l, const Vector4i& r);
Vector4i operator^(const Vector4i& l, const Vector4i& r);
Vector4i operator<<(const Vector4i& l, int n);
Vector4i operator>>(const Vector4i& l, int n);
inline Vector4i operator*(int i, const Vector4i& v) { return v * i; }
inline Vector4i& operator+=(Vector4i& l, const Vector4i& r) { return l = l + r; }
inline Vector4i& operator-=(Vector4i& l, const Vector4i& r) { return l = l - r; }
inline Vector4i& operator*=(Vector4i& l, const Vector4i& r) { return l = l * r; }
inline Vector4i& operator+=(Vector4i& l, int i) { return l = l + i; }
inline Vector4i& operator-=(Vector4i& l, int i) { return l = l - i; }
inline Vector4i& operator*=(Vector4i& l, int i) { return l = l * i; }
template <> inline void my_zero(Vector4i& v) { v = Vector4i(0); }

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_VECTOR4I_H_
