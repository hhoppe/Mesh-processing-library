// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_VECTORF_H_
#define MESH_PROCESSING_LIBHH_VECTORF_H_

#include "libHh/Advanced.h"  // unroll<>, unroll_max<>
#include "libHh/Array.h"
#include "libHh/Vec.h"
#include "libHh/Vector4.h"

namespace hh {

// Vector of n float values, with arithmetic accelerated by Vector4 class.
template <int n> class VectorF : Vec<Vector4, n / 4>, Vec<float, n % 4> {
  using type = VectorF;
  // We derive from the two Vec classes to benefit from "empty base class optimization".
  static constexpr int m = n / 4;
  static constexpr int p = n % 4;
  static constexpr int max_unroll = 4;

 public:
  VectorF() = default;  // Was: { fill(0.f); }.
  explicit VectorF(float v) { fill(v); }
  VectorF(const VectorF<n>&) = default;
  [[nodiscard]] constexpr int num() const { return n; }
  [[nodiscard]] constexpr size_t size() const { return n; }
  [[nodiscard]] auto& operator[](this auto&& self, int i) { return HH_CHECK_BOUNDS(i, n), self.data()[i]; }
  [[nodiscard]] static bool ok(int i) { return i >= 0 && i < n; }
  void load_unaligned(const float* pSrc) {
    for_int(j, m) for_int(c, 4) a()[j][c] = *pSrc++;
    for_int(k, p) b()[k] = *pSrc++;
  }
  void store_unaligned(float* pDst) const {
    for_int(j, m) for_int(c, 4) { *pDst++ = a()[j][c]; }
    for_int(k, p) { *pDst++ = b()[k]; }
  }
  void load_aligned(const float* pSrc) {  // Must be 16-aligned.
    for_int(j, m) {
      a()[j].load_aligned(pSrc);
      pSrc += 4;
    }
    for_int(k, p) b()[k] = *pSrc++;
  }
  void store_aligned(float* pDst) const {  // Must be 16-aligned.
    for_int(j, m) {
      a()[j].store_aligned(pDst);
      pDst += 4;
    }
    for_int(k, p) { *pDst++ = b()[k]; }
  }
  using value_type = float;
  using iterator = float*;
  using const_iterator = const float*;
  [[nodiscard]] auto begin(this auto&& self) { return self.data(); }
  [[nodiscard]] auto end(this auto&& self) { return self.data() + n; }
  [[nodiscard]] float* data() { return reinterpret_cast<float*>(this); }
  [[nodiscard]] const float* data() const { return reinterpret_cast<const float*>(this); }
  void zero() { fill(0.f); }
  void fill(float v) {
    local_unroll<m>([&](int j) { a()[j] = Vector4(v); });
    for_int(k, p) b()[k] = v;
  }
  [[nodiscard]] friend VectorF<n> min(const VectorF<n>& l, const VectorF<n>& r) {
    VectorF<n> v;
    local_unroll<m>([&](int j) { v.a()[j] = min(l.a()[j], r.a()[j]); });
    for_int(k, p) v.b()[k] = min(l.b()[k], r.b()[k]);
    return v;
  }
  [[nodiscard]] friend VectorF<n> max(const VectorF<n>& l, const VectorF<n>& r) {
    VectorF<n> v;
    local_unroll<m>([&](int j) { v.a()[j] = max(l.a()[j], r.a()[j]); });
    for_int(k, p) v.b()[k] = max(l.b()[k], r.b()[k]);
    return v;
  }
  VectorF<n>& operator=(const VectorF<n>& v) {
    local_unroll<m>([&](int j) { a()[j] = v.a()[j]; });
    for_int(k, p) b()[k] = v.b()[k];
    return *this;
  }
  [[nodiscard]] friend VectorF<n> operator+(const VectorF<n>& l, const VectorF<n>& r) {
    VectorF<n> v;
    local_unroll<m>([&](int j) { v.a()[j] = l.a()[j] + r.a()[j]; });
    for_int(k, p) v.b()[k] = l.b()[k] + r.b()[k];
    return v;
  }
  [[nodiscard]] friend VectorF<n> operator-(const VectorF<n>& l, const VectorF<n>& r) {
    VectorF<n> v;
    local_unroll<m>([&](int j) { v.a()[j] = l.a()[j] - r.a()[j]; });
    for_int(k, p) v.b()[k] = l.b()[k] - r.b()[k];
    return v;
  }
  [[nodiscard]] friend VectorF<n> operator*(const VectorF<n>& l, const VectorF<n>& r) {
    VectorF<n> v;
    local_unroll<m>([&](int j) { v.a()[j] = l.a()[j] * r.a()[j]; });
    for_int(k, p) v.b()[k] = l.b()[k] * r.b()[k];
    return v;
  }
  [[nodiscard]] friend VectorF<n> operator/(const VectorF<n>& l, const VectorF<n>& r) {
    VectorF<n> v;
    local_unroll<m>([&](int j) { v.a()[j] = l.a()[j] / r.a()[j]; });
    for_int(k, p) v.b()[k] = l.b()[k] / r.b()[k];
    return v;
  }
  [[nodiscard]] friend VectorF<n> operator*(const VectorF<n>& l, float f) {
    VectorF<n> v;
    local_unroll<m>([&](int j) { v.a()[j] = l.a()[j] * f; });
    for_int(k, p) v.b()[k] = l.b()[k] * f;
    return v;
  }
  [[nodiscard]] friend VectorF<n> operator/(const VectorF<n>& v, float f) { return v * (1.f / f); }
  [[nodiscard]] friend float dot(const VectorF<n>& l, const VectorF<n>& r) {
    float sum1;
    if constexpr (!m) {
      sum1 = 0.f;
    } else {
      sum1 = dot(l.a()[0], r.a()[0]);
      local_unroll<m - 1>([&](int j) { sum1 += dot(l.a()[j + 1], r.a()[j + 1]); });
    }
    for_int(k, p) sum1 += l.b()[k] * r.b()[k];
    return sum1;
  }
  [[nodiscard]] friend float mag2(const VectorF<n>& v) {
    float sum2;
    if constexpr (!m) {
      sum2 = 0.f;
    } else {
      sum2 = mag2(v.a()[0]);
      local_unroll<m - 1>([&](int j) { sum2 += mag2(v.a()[j + 1]); });
    }
    for_int(k, p) sum2 += square(v.b()[k]);
    return sum2;
  }
  [[nodiscard]] friend float dist2(const VectorF<n>& l, const VectorF<n>& r) {
    float sum2;
    if constexpr (!m) {
      sum2 = 0.f;
    } else {
      sum2 = dist2(l.a()[0], r.a()[0]);
      local_unroll<m - 1>([&](int j) { sum2 += dist2(l.a()[j + 1], r.a()[j + 1]); });
    }
    for_int(k, p) sum2 += square(l.b()[k] - r.b()[k]);
    return sum2;
  }
  [[nodiscard]] friend float sum(const VectorF<n>& v) {
    float sum1;
    if constexpr (!m) {
      sum1 = 0.f;
    } else {
      Vector4 vsum1 = v.a()[0];
      local_unroll<m - 1>([&](int j) { vsum1 += v.a()[j + 1]; });
      sum1 = sum(vsum1);
    }
    for_int(k, p) sum1 += v.b()[k];
    return sum1;
  }

 private:
  auto a(this auto&& self) noexcept { return self.Vec<Vector4, n / 4>::data(); }
  auto b(this auto&& self) noexcept { return self.Vec<float, n % 4>::data(); }
  template <int count, typename Func> static void local_unroll(Func func) { unroll_max<count, max_unroll>(func); }
};

template <int n> std::ostream& operator<<(std::ostream& os, const VectorF<n>& v);
template <int n> VectorF<n> operator+(const VectorF<n>& l, const VectorF<n>& r);
template <int n> VectorF<n> operator-(const VectorF<n>& l, const VectorF<n>& r);
template <int n> VectorF<n> operator*(const VectorF<n>& l, const VectorF<n>& r);
template <int n> VectorF<n> operator/(const VectorF<n>& l, const VectorF<n>& r);
template <int n> VectorF<n> operator*(const VectorF<n>& v, float f);
template <int n> VectorF<n> operator/(const VectorF<n>& v, float f);
template <int n> VectorF<n>& operator+=(VectorF<n>& l, const VectorF<n>& r) { return l = l + r; }
template <int n> VectorF<n>& operator-=(VectorF<n>& l, const VectorF<n>& r) { return l = l - r; }
template <int n> VectorF<n>& operator*=(VectorF<n>& l, const VectorF<n>& r) { return l = l * r; }
template <int n> VectorF<n>& operator/=(VectorF<n>& l, const VectorF<n>& r) { return l = l / r; }
template <int n> VectorF<n>& operator+=(VectorF<n>& l, float f) { return l = l + f; }
template <int n> VectorF<n>& operator-=(VectorF<n>& l, float f) { return l = l - f; }
template <int n> VectorF<n>& operator*=(VectorF<n>& l, float f) { return l = l * f; }
template <int n> VectorF<n>& operator/=(VectorF<n>& l, float f) { return l = l / f; }
template <int n> float dot(const VectorF<n>& l, const VectorF<n>& r);
template <int n> float mag2(const VectorF<n>& v);
template <int n> float dist2(const VectorF<n>& l, const VectorF<n>& r);
template <int n> float sum(const VectorF<n>& v);
template <int n> VectorF<n> operator*(float f, const VectorF<n>& v) { return v * f; }

//----------------------------------------------------------------------------

template <int n> std::ostream& operator<<(std::ostream& os, const VectorF<n>& v) {
  os << "VectorF<" << n << ">(";
  for_int(i, n) os << (i ? ", " : "") << v[i];
  return os << ")";
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_VECTORF_H_
