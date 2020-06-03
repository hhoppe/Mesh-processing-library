// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_VECTOR4_H_
#define MESH_PROCESSING_LIBHH_VECTOR4_H_

#if 0  // Note: sandbox/server machines like msr-sandbox-031.redmond.corp.microsoft.com lack this feature.
#define HH_NO_SSE41
#endif

#if 0 && defined(HH_DEBUG)  // debug version would be more portable, but ~/src/progtest/Filterimage.ou has differences
#define HH_NO_SSE41
#endif

#if defined(HH_NO_VECTOR4_VECTORIZATION)
// If so, do not enable vectorization.
#elif defined(_M_ARM) && _M_ARM_FP >= 40 || \
    defined(__ARM_NEON__)  // from win and clang respectively; maybe __aarch64__
#define HH_VECTOR4_NEON
#elif _M_IX86_FP >= 2 || defined(_M_X64) || defined(__SSE2__)  // (_M_IX86_FP is undefined for x64)
#define HH_VECTOR4_SSE
#if (defined(__GNUC__) || defined(__clang__)) && !defined(__SSE4_1__)
#define HH_NO_SSE41
#endif
#elif defined(__powerpc__) || defined(__ppc__)  // maybe __powerpc64__
#else
#pragma message("warning: untested")
#endif
// http://gcc.gnu.org/onlinedocs/gcc-4.7.0/gcc/Vector-Extensions.html
// typedef int v4si __attribute__((vector_size (16)));

#if defined(HH_VECTOR4_SSE)
#include <emmintrin.h>  // __m128i
#include <smmintrin.h>  // _mm_dp_ps(), _mm_cvtepu8_epi32(), etc
#include <xmmintrin.h>  // __m128, _mm_add_ps(), etc
#elif defined(HH_VECTOR4_NEON)
#include <arm_neon.h>
#endif

#include "Pixel.h"

namespace hh {

class Vector4;
Vector4 to_Vector4_norm(const uint8_t p[4]);  // converts each uint8_t [0, 255] to [0.f, 1.f]; used in Vector4(Pixel)
Vector4 to_Vector4_raw(const uint8_t p[4]);   // converts each uint8_t [0, 255] to [0.f, 255.f]

// Abstraction of a 4-float vector, hopefully accelerated by vectorized CPU instructions.
// See also class F32vec4 in <fvec.h> in Microsoft Visual Studio, provided by Intel:
//  https://software.intel.com/sites/products/documentation/studio/composer/en-us/2011Update/compiler_c/cref_cls/common/cppref_class_cpp_simd.htm
// And also https://github.com/scoopr/vectorial/blob/master/include/vectorial/simd4f_sse.h
//  and https://github.com/scoopr/vectorial/blob/master/include/vectorial/simd4f_neon.h
// See ~/git/CompPhoto/ClassLibs/VisionTools/src/common/SSEonNeon.h
// http://gcc.gnu.org/onlinedocs/gcc-4.8.1/gcc/ARM-NEON-Intrinsics.html#ARM-NEON-Intrinsics
class Vector4 {
  using type = Vector4;

 public:
  Vector4() = default;  // was: { fill(0.f); }
  explicit Vector4(float v) { fill(v); }
  explicit Vector4(const Pixel& pix) { *this = to_Vector4_norm(pix.data()); }
  explicit Vector4(const Vec4<float>& a) { load_unaligned(a.data()); }
  float& operator[](int i) { return (HH_CHECK_BOUNDS(i, 4), _c[i]); }
  const float& operator[](int i) const { return (HH_CHECK_BOUNDS(i, 4), _c[i]); }
  Vector4 with(int i, float f) const {
    HH_CHECK_BOUNDS(i, 4);
    Vector4 v = *this;
    v[i] = f;
    return v;
  }
  void raw_to_byte4(uint8_t p[4]) const;   // maps from [0.f, 255.999f] to uint8 using truncation, without clamping
  void norm_to_byte4(uint8_t p[4]) const;  // maps from [0.f, 1.f]     to uint8 using rounding and clamping
  Pixel raw_pixel() const {
    Pixel pixel;
    raw_to_byte4(pixel.data());
    return pixel;
  }
  Pixel pixel() const {
    Pixel pixel;
    norm_to_byte4(pixel.data());
    return pixel;
  }
  friend float mag2(const Vector4& v) { return dot(v, v); }
  friend float dist2(const Vector4& l, const Vector4& r) { return mag2(l - r); }
  friend float sum(const Vector4& v) { return dot(v, Vector4(1.f)); }
  friend Vector4 operator-(const Vector4& l) { return Vector4(0.f) - l; }
  using value_type = float;
  using iterator = const float*;
  using const_iterator = const float*;
  const float* begin() const { return _c; }
  const float* end() const { return _c + 4; }
  float* data() { return _c; }
  const float* data() const { return _c; }
  friend std::ostream& operator<<(std::ostream& os, const Vector4& v) {
    return os << "Vector4(" << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << ")";
  }
  static bool ok(int i) { return i >= 0 && i < 4; }
  friend Vector4 to_Vector4_raw(const uint8_t p[4]);
#if !(defined(_M_X64) || defined(__x86_64))
  // "new type[size]" does not create aligned storage -- problem for Vector4 in 32-bit model
  static void* operator new(size_t s) { return aligned_malloc(s, alignof(type)); }
  static void operator delete(void* p, size_t) { aligned_free(p); }
  static void* operator new[](size_t s) { return aligned_malloc(s, alignof(type)); }
  static void operator delete[](void* p, size_t) { aligned_free(p); }
#endif
#if defined(HH_VECTOR4_SSE)
  Vector4(const Vector4& v) : _r(v._r) {}
  Vector4(float x, float y, float z, float w) { _r = _mm_set_ps(w, z, y, x); }  // note reverse ordering
  void load_unaligned(const float* pSrc) { _r = _mm_loadu_ps(pSrc); }
  void store_unaligned(float* pDst) const { _mm_storeu_ps(pDst, _r); }
  void load_aligned(const float* pSrc) { _r = _mm_load_ps(pSrc); }   // VT: vld1q_f32_ex(pSrc, 128);
  void store_aligned(float* pDst) const { _mm_store_ps(pDst, _r); }  // VT: vst1q_f32_ex(pDst, _r, 128);
  friend Vector4 operator+(const Vector4& l, const Vector4& r) { return _mm_add_ps(l._r, r._r); }
  friend Vector4 operator-(const Vector4& l, const Vector4& r) { return _mm_sub_ps(l._r, r._r); }
  friend Vector4 operator*(const Vector4& l, const Vector4& r) { return _mm_mul_ps(l._r, r._r); }
  friend Vector4 operator/(const Vector4& l, const Vector4& r) { return _mm_div_ps(l._r, r._r); }
  friend Vector4 operator+(const Vector4& v, float f) { return _mm_add_ps(v._r, _mm_set_ps1(f)); }
  friend Vector4 operator-(const Vector4& v, float f) { return _mm_sub_ps(v._r, _mm_set_ps1(f)); }
  friend Vector4 operator*(const Vector4& v, float f) { return _mm_mul_ps(v._r, _mm_set_ps1(f)); }
  friend Vector4 operator/(const Vector4& v, float f) { return _mm_div_ps(v._r, _mm_set_ps1(f)); }
  Vector4& operator=(const Vector4& r) {
    _r = r._r;
    return *this;
  }
  void fill(float v) { _r = _mm_set_ps1(v); }  // all components set to same value
  friend Vector4 min(const Vector4& l, const Vector4& r) { return _mm_min_ps(l._r, r._r); }  // component-wise min
  friend Vector4 max(const Vector4& l, const Vector4& r) { return _mm_max_ps(l._r, r._r); }  // component-wise max
  friend float dot(const Vector4& v1, const Vector4& v2) {
#if defined(HH_NO_SSE41)
    Vector4 v = v1 * v2;
    return v[0] + v[1] + v[2] + v[3];
#else
    __m128 r = _mm_dp_ps(v1._r, v2._r, 0xFF);  // SSE4.1 DPPS instruction (dot product)
    // extracts the lower order floating point value from the parameter
    return _mm_cvtss_f32(r);
    // return r.m128_f32[0];  // less portable
    // float f; _mm_store_ss(&f, r); return f;  // requires stack store and load on VC11
    // http://stackoverflow.com/questions/6996764/fastest-way-to-do-horizontal-float-vector-sum-on-x86
#endif
  }
  friend Vector4 sqrt(const Vector4& v) {  // For use in RangeOp.h mag(), rms(), dist()
    // http://stackoverflow.com/questions/1528727/why-is-sse-scalar-sqrtx-slower-than-rsqrtx-x
    return _mm_sqrt_ps(v._r);
  }
  friend Vector4 abs(const Vector4& v) {  // For use in max_abs_element()
    // http://stackoverflow.com/questions/3361132/flipping-sign-on-packed-sse-floats
    static const __m128 k_sign_mask = _mm_set1_ps(-0.f);  // -0.f = 1 << 31, _mm_set1_ps() not constexpr
    // static const __m128 k_sign_mask = _mm_castsi128_ps(_mm_set1_epi32(0x80000000));  // not constexpr
    return _mm_andnot_ps(k_sign_mask, v._r);
  }
  // friend Vector4 madd(const Vector4& v1, const Vector4& v2, const Vector& v3) {  // v1 * v2 + v3
  //     return _mm_macc_ps(v1._r, v2._r, v3._r); } // AVX FMA4 XMM instruction vfmaddps
  //     // Intel plans to implement FMA3 in processors using its Haswell microarchitecture, due in 2013
  // }
 private:
  Vector4(__m128 v) : _r(v) {}
  Vector4& operator=(const __m128& r) {
    _r = r;
    return *this;
  }
  // Unfortunately, presence of union leads to !std::is_trivially_copyable<Vector4>::value .
  // I could remove union and use more complex accessor function as in
  //  http://stackoverflow.com/questions/12624466/get-member-of-m128-by-index
  // or in <fvec.h>.
  // However, this seems too complicated for little gain.
  union {
    __m128 _r;  // This could be made public to allow mixing of above overloads and full set of SSE ops.
    float _c[4];
  };
#elif defined(HH_VECTOR4_NEON)
  Vector4(const Vector4& v) : _r(v._r) {}
  Vector4(float x, float y, float z, float w) {
    float r[4] = {x, y, z, w};
    _r = vld1q_f32(r);
  }
  void load_unaligned(const float* pSrc) {  // other instruction?
    for_int(c, 4) _c[c] = pSrc[c];
  }
  void store_unaligned(float* pDst) const {  // other instruction?
    for_int(c, 4) pDst[c] = _c[c];
  }
  void load_aligned(const float* pSrc) { _r = vld1q_f32(pSrc); }
  void store_aligned(float* pDst) const { vst1q_f32(pDst, _r); }
  friend Vector4 operator+(const Vector4& l, const Vector4& r) { return vaddq_f32(l._r, r._r); }
  friend Vector4 operator-(const Vector4& l, const Vector4& r) { return vsubq_f32(l._r, r._r); }
  friend Vector4 operator*(const Vector4& l, const Vector4& r) { return vmulq_f32(l._r, r._r); }
  friend Vector4 operator/(const Vector4& l, const Vector4& r) { return vmulq_f32(l._r, recip(r._r)._r); }
  friend Vector4 operator+(const Vector4& v, float f) { return vaddq_f32(v._r, vdupq_n_f32(f)); }
  friend Vector4 operator-(const Vector4& v, float f) { return vsubq_f32(v._r, vdupq_n_f32(f)); }
  friend Vector4 operator*(const Vector4& v, float f) { return vmulq_n_f32(v._r, f); }
  friend Vector4 operator/(const Vector4& v, float f) { return vmulq_n_f32(v._r, 1.f / f); }
  Vector4& operator=(const Vector4& r) {
    _r = r._r;
    return *this;
  }
  void fill(float v) { _r = vdupq_n_f32(v); }  // all components set to same value
  friend Vector4 min(const Vector4& l, const Vector4& r) { return vminq_f32(l._r, r._r); }  // component-wise min
  friend Vector4 max(const Vector4& l, const Vector4& r) { return vmaxq_f32(l._r, r._r); }  // component-wise max
  friend float dot(const Vector4& v1, const Vector4& v2) {
    Vector4 v = v1 * v2;
    return v[0] + v[1] + v[2] + v[3];
  }
  friend Vector4 sqrt(const Vector4& v) {  // For use in RangeOp.h mag(), rms(), dist()
    // return vrecpeq_f32(vrsqrteq_f32(v));  // very approximate
    // Maybe use vrsqrteq_f32 and vrsqrtsq_f32 as in http://rcl-rs-vvg.blogspot.com/2010/08/simd-etudes.html
    //  but unclear.
    using std::sqrt;
    return Vector4(sqrt(v[0]), sqrt(v[1]), sqrt(v[2]), sqrt(v[3]));
  }
  friend Vector4 abs(const Vector4& v) { return vabsq_f32(v._r); }

 private:
  Vector4(float32x4_t v) : _r(v) {}
  Vector4& operator=(const float32x4_t& r) {
    _r = r;
    return *this;
  }
  union {
    float32x4_t _r;  // This could be made public to allow mixing of above overloads and full set of Neon ops.
    float _c[4];
  };
  static Vector4 recip(const Vector4& v) {
    float32x4_t estimate = vrecpeq_f32(v._r);  // get estimate, then apply two Newton-Raphson steps
    estimate = vmulq_f32(vrecpsq_f32(v._r, estimate), estimate);
    estimate = vmulq_f32(vrecpsq_f32(v._r, estimate), estimate);
    return estimate;
  }
#else   // neither defined(HH_VECTOR4_SSE) nor defined(HH_VECTOR4_NEON)
  Vector4(const Vector4& v) {
    for_int(c, 4) _c[c] = v._c[c];
  }
  Vector4(float x, float y, float z, float w) { _c[0] = x, _c[1] = y, _c[2] = z, _c[3] = w; }
  void load_unaligned(const float* pSrc) {
    for_int(c, 4) _c[c] = pSrc[c];
  }
  void store_unaligned(float* pDst) const {
    for_int(c, 4) pDst[c] = _c[c];
  }
  void load_aligned(const float* pSrc) {
    for_int(c, 4) _c[c] = pSrc[c];
  }
  void store_aligned(float* pDst) const {
    for_int(c, 4) pDst[c] = _c[c];
  }
  friend Vector4 operator+(const Vector4& l, const Vector4& r) {
    return Vector4(l[0] + r[0], l[1] + r[1], l[2] + r[2], l[3] + r[3]);
  }
  friend Vector4 operator-(const Vector4& l, const Vector4& r) {
    return Vector4(l[0] - r[0], l[1] - r[1], l[2] - r[2], l[3] - r[3]);
  }
  friend Vector4 operator*(const Vector4& l, const Vector4& r) {
    return Vector4(l[0] * r[0], l[1] * r[1], l[2] * r[2], l[3] * r[3]);
  }
  friend Vector4 operator/(const Vector4& l, const Vector4& r) {
    return Vector4(l[0] / r[0], l[1] / r[1], l[2] / r[2], l[3] / r[3]);
  }
  friend Vector4 operator+(const Vector4& v, float f) { return v + Vector4(f); }
  friend Vector4 operator-(const Vector4& v, float f) { return v - Vector4(f); }
  friend Vector4 operator*(const Vector4& v, float f) { return v * Vector4(f); }
  friend Vector4 operator/(const Vector4& v, float f) { return v * (1.f / f); }
  Vector4& operator=(const Vector4& r) {
    for_int(c, 4) _c[c] = r._c[c];
    return *this;
  }
  void fill(float v) {
    for_int(c, 4) _c[c] = v;
  }
  friend Vector4 min(const Vector4& l, const Vector4& r) {
    return Vector4(min(l[0], r[0]), min(l[1], r[1]), min(l[2], r[2]), min(l[3], r[3]));
  }
  friend Vector4 max(const Vector4& l, const Vector4& r) {
    return Vector4(max(l[0], r[0]), max(l[1], r[1]), max(l[2], r[2]), max(l[3], r[3]));
  }
  friend float dot(const Vector4& v1, const Vector4& v2) {
    Vector4 v = v1 * v2;
    return v[0] + v[1] + v[2] + v[3];
  }

  friend Vector4 sqrt(const Vector4& v) {
    using std::sqrt;
    return Vector4(sqrt(v[0]), sqrt(v[1]), sqrt(v[2]), sqrt(v[3]));
  }
  friend Vector4 abs(const Vector4& v) {
    using std::abs;
    return Vector4(abs(v[0]), abs(v[1]), abs(v[2]), abs(v[3]));
  }
  // friend Vector4 madd(const Vector4& v1, const Vector4& v2, const Vector& v3) { return v1 * v2 + v3; }
 private:
  alignas(16)  // since no __mm128 element to induce 16-byte alignment
      float _c[4];
#endif  // defined(HH_VECTOR4_SSE) or defined(HH_VECTOR4_NEON)
};

inline Vector4 to_Vector4_norm(const uint8_t p[4]) { return to_Vector4_raw(p) * (1.f / 255.f); }
inline Vector4 to_Vector4_raw(const Pixel& pixel) { return to_Vector4_raw(pixel.data()); }
float mag2(const Vector4& v);
float dist2(const Vector4& l, const Vector4& r);
float sum(const Vector4& v);
float dot(const Vector4& l, const Vector4& r);
Vector4 operator+(const Vector4& l, const Vector4& r);
Vector4 operator-(const Vector4& l, const Vector4& r);
Vector4 operator*(const Vector4& l, const Vector4& r);
Vector4 operator/(const Vector4& l, const Vector4& r);
Vector4 operator+(const Vector4& v, float f);
Vector4 operator-(const Vector4& v, float f);
Vector4 operator*(const Vector4& v, float f);
Vector4 operator/(const Vector4& v, float f);
inline Vector4 operator*(float f, const Vector4& v) { return v * f; }
inline Vector4& operator+=(Vector4& l, const Vector4& r) { return l = l + r; }
inline Vector4& operator-=(Vector4& l, const Vector4& r) { return l = l - r; }
inline Vector4& operator*=(Vector4& l, const Vector4& r) { return l = l * r; }
inline Vector4& operator/=(Vector4& l, const Vector4& r) { return l = l / r; }
inline Vector4& operator+=(Vector4& l, float f) { return l = l + f; }
inline Vector4& operator-=(Vector4& l, float f) { return l = l - f; }
inline Vector4& operator*=(Vector4& l, float f) { return l = l * f; }
inline Vector4& operator/=(Vector4& l, float f) { return l = l / f; }
template <> inline void my_zero(Vector4& v) { v = Vector4(0.f); }
inline Vector4 interp(const Vector4& v1, const Vector4& v2, float f1 = .5f) { return f1 * v1 + (1.f - f1) * v2; }

//----------------------------------------------------------------------------

#if defined(HH_VECTOR4_SSE) && !defined(HH_NO_SSE41)

// _mm_packs_epi32   : pack the  8 signed 32-bit integers from a and b into signed 16-bit and saturate.
// _mm_packus_epi16  : pack the 16 signed 16-bit from a and b into 8-bit unsigned and saturate.
// _mm_packs_epi16   : pack the 16 signed 16-bit integers from a and b into 8-bit integers and saturate.
// _mm_cvtepu8_epi32 : expand 4 unsigned 8-bit to 4 unsigned 32-bit (SSE4.1)

inline Vector4 to_Vector4_raw(const uint8_t p[4]) {
  // avoids a shuffle, unlike _mm_set1_epi32(*reinterpret_cast<const int*>(p))
  __m128i in = _mm_castps_si128(_mm_load_ss(reinterpret_cast<const float*>(p)));
  __m128i t1 = _mm_cvtepu8_epi32(in);  // expand 4 unsigned 8-bit to 4 unsigned 32-bit (SSE4.1)
  __m128 t2 = _mm_cvtepi32_ps(t1);     // convert four signed 32-bit to floats.
  return t2;
}
inline void Vector4::raw_to_byte4(uint8_t p[4]) const {
  for_int(c, 4) ASSERTX(_c[c] >= 0.f && _c[c] < 255.999f);
  __m128i t1 = _mm_cvttps_epi32(_r);      // 4 float -> 4 signed 32-bit int (truncation)  (or cvtps for rounding)
  __m128i t2 = _mm_packs_epi32(t1, t1);   // 8 signed 32-bit -> 8 signed 16-bit (saturation)
  __m128i t3 = _mm_packus_epi16(t2, t2);  // 16 signed 16-bit -> 16 unsigned 8-bit (saturation)
  _mm_store_ss(reinterpret_cast<float*>(p), _mm_castsi128_ps(t3));
}
inline void Vector4::norm_to_byte4(uint8_t p[4]) const {
  Vector4 t = *this * 255.f;
  for_int(c, 4) ASSERTX(t[c] <= 2147480000.f);  // see Vector4_test.h
  __m128i t1 = _mm_cvtps_epi32(t._r);     // 4 float -> 4 signed 32-bit int (rounding)  (or cvttps for truncation)
  __m128i t2 = _mm_packs_epi32(t1, t1);   // 8 signed 32-bit -> 8 signed 16-bit (saturation)
  __m128i t3 = _mm_packus_epi16(t2, t2);  // 16 signed 16-bit -> 16 unsigned 8-bit (saturation)
  _mm_store_ss(reinterpret_cast<float*>(p), _mm_castsi128_ps(t3));
}

#elif defined(HH_VECTOR4_NEON)

inline Vector4 to_Vector4_raw(const uint8_t p[4]) {
  // See http://stackoverflow.com/a/14506159/1190077
  //  vmovl.u8       q3, d2    // Expand to 16-bit
  //  vmovl.u16      q10, d6   // Expand to 32-bit
  //  vmovl.u16      q11, d7
  //  vcvt.f32.u32   q10, q10  // Convert to float
  //  vcvt.f32.u32   q11, q11
  // uint8x8x2_t a = vld2_u8(p);  // uint8x8x2_t vld2_u8(uint8_t const * ptr);  // VLD2.8 {d0, d1}, [r0]
  // uint8x8_t a0 = a.val[0];
  // This initialization of variable "a" may be optional.
  uint32x2_t a = vcreate_u32(0);  // uint32x2_t vcreate_u32(uint64_t a);  // VMOV d0, r0, r0
  // uint32x2_t vld1_lane_u32(const uint32_t* ptr, uint32x2_t vec, __constrange(0, 1) int lane);  // VLD1.32
  a = vld1_lane_u32(reinterpret_cast<const uint32_t*>(p), a, 0);
  uint8x8_t a0 = vreinterpret_u8_u32(a);  // uint8x8_t vreinterpret_u8_u32(uint32x2_t);
  uint16x8_t b = vmovl_u8(a0);            // uint16x8_t vmovl_u8(uint8x8_t a);  // VMOVL.U8 q0, d0
  uint16x4_t c = vget_low_u16(b);         // uint16x4_t vget_low_u16(uint16x8_t a);  // VMOV d0, d0
  uint32x4_t d = vmovl_u16(c);            // uint32x4_t vmovl_u16(uint16x4_t a);  // VMOVL.U16 q0, d0
  float32x4_t f = vcvtq_f32_u32(d);       // float32x4_t vcvtq_f32_u32(uint32x4_t a);  // VCVT.F32.U32 q0, q0
  return f;
}
inline void Vector4::raw_to_byte4(uint8_t p[4]) const {
  for_int(c, 4) ASSERTX(_c[c] >= 0.f && _c[c] < 255.999f);
  uint32x4_t a = vcvtq_u32_f32(_r);       // uint32x4_t vcvt_u32_f32(float32x4_t a);  // VCVT.U32.F32 q0, q0 // truncate
  uint16x4_t b = vqmovn_u32(a);           // uint16x4_t vqmovn_u32(uint32x4_t a);  // VQMOVN.I32 d0, q0 // saturation
  uint16x8_t c = vcombine_u16(b, b);      // uint16x8_t vcombine_u16(uint16x4_t low, uint16x4_t high);
  uint8x8_t d = vqmovn_u16(c);            // uint8x8_t vqmovn_u16(uint16x8_t a);  // VQMOVN.I16 d0, q0
  uint32x2_t e = vreinterpret_u32_u8(d);  // uint32x2_t vreinterpret_u32_u8 (uint8x8_t)
  vst1_lane_u32(reinterpret_cast<uint32_t*>(p), e, 0);  // vst1_lane_u32 (uint32_t*, uint32x2_t, int lane)
}
inline void Vector4::norm_to_byte4(uint8_t p[4]) const {
  Vector4 t = *this * 255.f;
  for_int(c, 4) ASSERTX(t[c] <= 2147480000.f);  // see Vector4_test.h
  uint32x4_t a = vcvtq_u32_f32(t._r);     // uint32x4_t vcvtq_u32_f32(float32x4_t a);  // VCVT.U32.F32 q0, q0 // round
  uint16x4_t b = vqmovn_u32(a);           // uint16x4_t vqmovn_u32(uint32x4_t a);  // VQMOVN.I32 d0, q0 // saturation
  uint16x8_t c = vcombine_u16(b, b);      // uint16x8_t vcombine_u16(uint16x4_t low, uint16x4_t high);
  uint8x8_t d = vqmovn_u16(c);            // uint8x8_t vqmovn_u16(uint16x8_t a);  // VQMOVN.I16 d0, q0 // saturation
  uint32x2_t e = vreinterpret_u32_u8(d);  // uint32x2_t vreinterpret_u32_u8 (uint8x8_t)
  vst1_lane_u32(reinterpret_cast<uint32_t*>(p), e, 0);  // vst1_lane_u32 (uint32_t*, uint32x2_t, int lane)
}

#else  // neither SSE nor NEON

inline Vector4 to_Vector4_raw(const uint8_t p[4]) { return Vector4(p[0], p[1], p[2], p[3]); }
inline void Vector4::raw_to_byte4(uint8_t p[4]) const {
  for_int(c, 4) {
    ASSERTX(_c[c] >= 0.f && _c[c] < 255.999f);
    p[c] = static_cast<uint8_t>(_c[c]);
  }
}
inline void Vector4::norm_to_byte4(uint8_t p[4]) const {
  for_int(c, 4) p[c] = static_cast<uint8_t>(clamp(_c[c], 0.f, 1.f) * 255.f + .5f);
}

#endif  // defined(HH_VECTOR4_SSE) or defined(HH_VECTOR4_NEON)

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_VECTOR4_H_
