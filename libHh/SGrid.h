// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_SGRID_H_
#define MESH_PROCESSING_LIBHH_SGRID_H_

#include "libHh/Grid.h"

namespace hh {

template <typename T, int d0, int... od> class SGrid;

namespace details {
template <int d0, int... od> struct SGrid_vol : std::integral_constant<size_t, d0 * SGrid_vol<od...>::value> {};
template <int d0> struct SGrid_vol<d0> : std::integral_constant<size_t, d0> {};
template <typename T, int d0, int... od> struct SGrid_sslice;
template <typename T, int d0, int d1, int... od> struct SGrid_sslice<T, d0, d1, od...> {
  using type = Vec<typename SGrid_sslice<T, d1, od...>::type, d1>;
};
template <typename T, int d0, int d1> struct SGrid_sslice<T, d0, d1> { using type = Vec<T, d1>; };
template <typename T, int d0> struct SGrid_sslice<T, d0> { using type = T; };
}  // namespace details

// Self-contained fixed-size multidimensional grid with elements of type T.
// A small benefit compared to Vec<Vec<...>> is the introduction of dims(), data(), flat(), view().
template <typename T, int d0, int... od>
class SGrid : public Vec<typename details::SGrid_sslice<T, d0, od...>::type, d0> {
  static constexpr int D = 1 + sizeof...(od);
  static constexpr size_t vol = details::SGrid_vol<d0, od...>::value;
  using type = SGrid<T, d0, od...>;
  using slice = typename details::SGrid_sslice<T, d0, od...>::type;  // Slice as Vec.
  using base = Vec<slice, d0>;
  using initializer_type = details::nested_initializer_list_t<D, T>;
  using nested_retrieve = details::nested_list_retrieve<D, T>;

 public:
  SGrid() = default;
  SGrid(const type&) = default;
  SGrid(initializer_type l) { *this = l; }  // Not constexpr, instead use = V(V(), V(), ...).
  constexpr explicit SGrid(const base& g) : base(g) {}
  constexpr SGrid(base&& g) : base(std::move(g)) {}
  SGrid(CGridView<D, T> g) { *this = g; }
  type& operator=(const type& g) = default;
  type& operator=(initializer_type l) {
    nested_retrieve()(*this, l);
    return *this;
  }
  type& operator=(CGridView<D, T> g) {
    assign(g);
    return *this;
  }
  constexpr int ndim() const { return D; }
  constexpr Vec<int, D> dims() const { return Vec<int, D>(d0, od...); }
  constexpr int dim(int c) const { return dims()[c]; }
  constexpr size_t size() const { return vol; }
  T& operator[](const Vec<int, D>& u) { return flat(ravel_index(dims(), u)); }
  const T& operator[](const Vec<int, D>& u) const { return flat(ravel_index(dims(), u)); }
  slice& operator[](int r) { return (ASSERTXX(check(r)), b()[r]); }
  const slice& operator[](int r) const { return (ASSERTXX(check(r)), b()[r]); }
  T& flat(size_t i) { return (ASSERTXX(i < vol), data()[i]); }
  const T& flat(size_t i) const { return (ASSERTXX(i < vol), data()[i]); }
  bool operator==(const type& p) const;
  bool operator!=(const type& p) const { return !(*this == p); }
  static type all(const T& e) {
    type g;
    for_size_t(i, vol) g.flat(i) = e;
    return g;
  }
  operator GridView<D, T>() { return view(); }
  operator CGridView<D, T>() const { return view(); }
  GridView<D, T> view() { return GridView<D, T>(data(), dims()); }
  CGridView<D, T> view() const { return CGridView<D, T>(data(), dims()); }
  CGridView<D, T> const_view() const { return CGridView<D, T>(data(), dims()); }
  ArrayView<T> array_view() { return ArrayView<T>(data(), narrow_cast<int>(size())); }
  CArrayView<T> array_view() const { return CArrayView<T>(data(), narrow_cast<int>(size())); }
  CArrayView<T> const_array_view() const { return CArrayView<T>(data(), narrow_cast<int>(size())); }
  template <int s> SGrid<T, s, od...>& segment(int i) {
    return (ASSERTXX(check(i, s)), *reinterpret_cast<SGrid<T, s, od...>*>(p(i)));
  }
  template <int s> const SGrid<T, s, od...>& segment(int i) const {
    return (ASSERTXX(check(i, s)), *reinterpret_cast<const SGrid<T, s, od...>*>(p(i)));
  }
  using value_type = T;
  using iterator = T*;
  using const_iterator = const T*;
  T* begin() { return data(); }
  const T* begin() const { return data(); }
  T* end() { return data() + vol; }
  const T* end() const { return data() + vol; }
  T* data() { return reinterpret_cast<T*>(b().data()); }
  const T* data() const { return reinterpret_cast<const T*>(b().data()); }

 private:
  base& b() { return *this; }
  const base& b() const { return *this; }
  slice* p(int i) { return b().data() + i; }
  const slice* p(int i) const { return b().data() + i; }
  bool check(int r) const {
    if (r >= 0 && r < d0) return true;
    SHOW(r, dims());
    return false;
  }
  bool check(int i, int s) const {
    if (i >= 0 && s >= 0 && i + s <= d0) return true;
    SHOW(i, s, dims());
    return false;
  }
  void assign(CGridView<D, T> g) {
    ASSERTX(dims() == g.dims());
    for_size_t(i, vol) flat(i) = g.flat(i);
  }
};

// Given container c, evaluate func() on each element (possibly changing the element type) and return new container.
template <typename Func, typename T, int d0, int... od> auto map(const SGrid<T, d0, od...>& c, Func func) {
  SGrid<decltype(func(std::declval<T>())), d0, od...> nc;
  for_size_t(i, c.size()) nc.flat(i) = func(c.flat(i));
  return nc;
}

//----------------------------------------------------------------------------

template <typename T, int d0, int... od> bool SGrid<T, d0, od...>::operator==(const type& p) const {
  for_size_t(i, vol) {
    if (flat(i) != p.flat(i)) return false;
  }
  return true;
}

template <typename T, int d0, int... od> std::ostream& operator<<(std::ostream& os, const SGrid<T, d0, od...>& g) {
  const int D = 1 + sizeof...(od);
  return os << CGridView<D, T>(g);
}
template <typename T, int d0, int... od> HH_DECLARE_OSTREAM_EOL(SGrid<T, d0, od...>);

//----------------------------------------------------------------------------

// Set of functions common to Vec.h, SGrid.h, Array.h, Grid.h
// Note that RangeOp.h functions are valid here: mag2(), mag(), dist2(), dist(), dot(), is_zero(), compare().
#define TT template <typename T, int d0, int... od>
#define G SGrid<T, d0, od...>
#define F for_size_t(i, g1.size())
// clang-format off

TT G operator+(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) + g2.flat(i); } return g; }
TT G operator-(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) - g2.flat(i); } return g; }
TT G operator*(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) * g2.flat(i); } return g; }
TT G operator/(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) / g2.flat(i); } return g; }
TT G operator%(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) % g2.flat(i); } return g; }

TT G operator+(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) + v; } return g; }
TT G operator-(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) - v; } return g; }
TT G operator*(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) * v; } return g; }
TT G operator/(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) / v; } return g; }
TT G operator%(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) % v; } return g; }

TT G operator+(T v, const G& g1) { G g; F { g.flat(i) = v + g1.flat(i); } return g; }
TT G operator-(T v, const G& g1) { G g; F { g.flat(i) = v - g1.flat(i); } return g; }
TT G operator*(T v, const G& g1) { G g; F { g.flat(i) = v * g1.flat(i); } return g; }
TT G operator/(T v, const G& g1) { G g; F { g.flat(i) = v / g1.flat(i); } return g; }
TT G operator%(T v, const G& g1) { G g; F { g.flat(i) = v % g1.flat(i); } return g; }

TT G& operator+=(G& g1, const G& g2) { F { g1.flat(i) += g2.flat(i); } return g1; }
TT G& operator-=(G& g1, const G& g2) { F { g1.flat(i) -= g2.flat(i); } return g1; }
TT G& operator*=(G& g1, const G& g2) { F { g1.flat(i) *= g2.flat(i); } return g1; }
TT G& operator/=(G& g1, const G& g2) { F { g1.flat(i) /= g2.flat(i); } return g1; }
TT G& operator%=(G& g1, const G& g2) { F { g1.flat(i) %= g2.flat(i); } return g1; }

TT G& operator+=(G& g1, const T& v) { F { g1.flat(i) += v; } return g1; }
TT G& operator-=(G& g1, const T& v) { F { g1.flat(i) -= v; } return g1; }
TT G& operator*=(G& g1, const T& v) { F { g1.flat(i) *= v; } return g1; }
TT G& operator/=(G& g1, const T& v) { F { g1.flat(i) /= v; } return g1; }
TT G& operator%=(G& g1, const T& v) { F { g1.flat(i) %= v; } return g1; }

TT G operator-(const G& g1) { G g; F { g.flat(i) = -g1.flat(i); } return g; }

TT G min(const G& g1, const G& g2) { G g; F { g.flat(i) = min(g1.flat(i), g2.flat(i)); } return g; }
TT G max(const G& g1, const G& g2) { G g; F { g.flat(i) = max(g1.flat(i), g2.flat(i)); } return g; }

TT G interp(const G& g1, const G& g2, float f1 = 0.5f) {
  G g; F { g.flat(i) = f1 * g1.flat(i) + (1.f - f1) * g2.flat(i); } return g;
}
TT G interp(const G& g1, const G& g2, const G& g3, float f1 = 1.f / 3.f, float f2 = 1.f / 3.f) {
  G g; F { g.flat(i) = f1 * g1.flat(i) + f2 * g2.flat(i) + (1.f - f1 - f2) * g3.flat(i); } return g;
}
TT G interp(const G& g1, const G& g2, const G& g3, const Vec3<float>& bary) {
  // Vec3<float> == Bary;   May have bary[0] + bary[1] + bary[2] != 1.f.
  G g; F { g.flat(i) = bary[0] * g1.flat(i) + bary[1] * g2.flat(i) + bary[2] * g3.flat(i); } return g;
}

// clang-format on
#undef F
#undef G
#undef TT

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SGRID_H_
