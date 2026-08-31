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
template <typename T, int d0, int d1> struct SGrid_sslice<T, d0, d1> {
  using type = Vec<T, d1>;
};
template <typename T, int d0> struct SGrid_sslice<T, d0> {
  using type = T;
};
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

 public:
  SGrid() = default;
  SGrid(const type&) = default;
  SGrid(initializer_type l) requires Copyable<T> { *this = l; }  // Not constexpr, instead use = V(V(), V(), ...).
  constexpr explicit SGrid(const base& g) requires Copyable<T> : base(g) {}
  constexpr SGrid(base&& g) : base(std::move(g)) {}
  SGrid(CGridView<D, T> g) requires Copyable<T> { *this = g; }
  type& operator=(const type& g) = default;
  type& operator=(initializer_type l) requires Copyable<T> {
    details::nested_list_retrieve<D, T>(this->view(), l);
    return *this;
  }
  type& operator=(CGridView<D, T> g) requires Copyable<T> {
    assign(g);
    return *this;
  }
  [[nodiscard]] static constexpr int ndim() { return D; }
  [[nodiscard]] static constexpr Vec<int, D> dims() { return Vec<int, D>(d0, od...); }
  [[nodiscard]] static constexpr int dim(int c) { return dims()[c]; }
  [[nodiscard]] static constexpr size_t size() { return vol; }
  [[nodiscard]] T& operator[](const Vec<int, D>& u) { return flat(ravel_index(dims(), u)); }
  [[nodiscard]] const T& operator[](const Vec<int, D>& u) const { return flat(ravel_index(dims(), u)); }
  template <std::integral... A> [[HH_GNU_PURE]] [[nodiscard]] constexpr auto& operator[](A... dd) {
    static_assert(sizeof...(A) >= 1 && sizeof...(A) <= D);
    return subscript(b(), dd...);
  }
  template <std::integral... A> [[HH_GNU_PURE]] [[nodiscard]] constexpr auto& operator[](A... dd) const {
    static_assert(sizeof...(A) >= 1 && sizeof...(A) <= D);
    return subscript(b(), dd...);
  }
  [[nodiscard]] T& flat(size_t i) { return ASSERTXX(i < vol), data()[i]; }
  [[nodiscard]] const T& flat(size_t i) const { return ASSERTXX(i < vol), data()[i]; }
  [[nodiscard]] bool operator==(const type& p) const;
  [[nodiscard]] static type all(const T& e) requires Copyable<T> {
    type g;
    for (const size_t i : range(vol)) g.flat(i) = e;
    return g;
  }
  [[nodiscard]] operator GridView<D, T>() { return view(); }
  [[nodiscard]] operator CGridView<D, T>() const { return view(); }
  [[nodiscard]] GridView<D, T> view() { return GridView<D, T>(data(), dims()); }
  [[nodiscard]] CGridView<D, T> view() const { return CGridView<D, T>(data(), dims()); }
  [[nodiscard]] CGridView<D, T> const_view() const { return CGridView<D, T>(data(), dims()); }
  [[nodiscard]] ArrayView<T> array_view() { return ArrayView<T>(data(), narrow_cast<int>(size())); }
  [[nodiscard]] CArrayView<T> array_view() const { return CArrayView<T>(data(), narrow_cast<int>(size())); }
  [[nodiscard]] CArrayView<T> const_array_view() const { return CArrayView<T>(data(), narrow_cast<int>(size())); }
  template <int s> [[nodiscard]] SGrid<T, s, od...>& segment(int i) {
    return ASSERTXX(check(i, s)), *reinterpret_cast<SGrid<T, s, od...>*>(p(i));
  }
  template <int s> [[nodiscard]] const SGrid<T, s, od...>& segment(int i) const {
    return ASSERTXX(check(i, s)), *reinterpret_cast<const SGrid<T, s, od...>*>(p(i));
  }
  using value_type = T;
  using iterator = T*;
  using const_iterator = const T*;
  [[nodiscard]] T* begin() { return data(); }
  [[nodiscard]] const T* begin() const { return data(); }
  [[nodiscard]] T* end() { return data() + vol; }
  [[nodiscard]] const T* end() const { return data() + vol; }
  [[nodiscard]] T* data() { return reinterpret_cast<T*>(b().data()); }
  [[nodiscard]] const T* data() const { return reinterpret_cast<const T*>(b().data()); }

 private:
  constexpr base& b() { return *this; }
  constexpr const base& b() const { return *this; }
  slice* p(int i) { return b().data() + i; }
  const slice* p(int i) const { return b().data() + i; }
  // Successively index into the nested Vec structure; each index is bounds-checked by Vec::operator[].
  template <typename... A> static constexpr auto& subscript(auto& v, int d, A... dd) {
    if constexpr (sizeof...(dd) == 0)
      return v[d];
    else
      return subscript(v[d], dd...);
  }
  constexpr bool check(int i, int s) const {
    if (i >= 0 && s >= 0 && i + s <= d0) return true;
    if !consteval {
      SHOW(i, s, dims());
    }
    return false;
  }
  void assign(CGridView<D, T> g) requires Copyable<T> {
    ASSERTX(dims() == g.dims());
    for (const size_t i : range(vol)) flat(i) = g.flat(i);
  }
};

// Given container c, evaluate func() on each element (possibly changing the element type) and return new container.
template <typename Func, typename T, int d0, int... od>
[[nodiscard]] constexpr auto transformed(const SGrid<T, d0, od...>& c, Func func) {
  SGrid<decltype(func(std::declval<T>())), d0, od...> nc;
  for (const size_t i : range(c.size())) nc.flat(i) = func(c.flat(i));
  return nc;
}

//----------------------------------------------------------------------------

template <typename T, int d0, int... od> [[nodiscard]] bool SGrid<T, d0, od...>::operator==(const type& p) const {
  for (const size_t i : range(vol))
    if (flat(i) != p.flat(i)) return false;
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
#define TTC TT [[nodiscard]] constexpr
#define TTA TT constexpr
#define G SGrid<T, d0, od...>
#define F for (const size_t i : range(g1.size()))
// clang-format off

TTC G operator+(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) + g2.flat(i); } return g; }
TTC G operator-(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) - g2.flat(i); } return g; }
TTC G operator*(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) * g2.flat(i); } return g; }
TTC G operator/(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) / g2.flat(i); } return g; }
TTC G operator%(const G& g1, const G& g2) { G g; F { g.flat(i) = g1.flat(i) % g2.flat(i); } return g; }

TTC G operator+(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) + v; } return g; }
TTC G operator-(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) - v; } return g; }
TTC G operator*(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) * v; } return g; }
TTC G operator/(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) / v; } return g; }
TTC G operator%(const G& g1, T v) { G g; F { g.flat(i) = g1.flat(i) % v; } return g; }

TTC G operator+(T v, const G& g1) { G g; F { g.flat(i) = v + g1.flat(i); } return g; }
TTC G operator-(T v, const G& g1) { G g; F { g.flat(i) = v - g1.flat(i); } return g; }
TTC G operator*(T v, const G& g1) { G g; F { g.flat(i) = v * g1.flat(i); } return g; }
TTC G operator/(T v, const G& g1) { G g; F { g.flat(i) = v / g1.flat(i); } return g; }
TTC G operator%(T v, const G& g1) { G g; F { g.flat(i) = v % g1.flat(i); } return g; }

TTA G& operator+=(G& g1, const G& g2) { F { g1.flat(i) += g2.flat(i); } return g1; }
TTA G& operator-=(G& g1, const G& g2) { F { g1.flat(i) -= g2.flat(i); } return g1; }
TTA G& operator*=(G& g1, const G& g2) { F { g1.flat(i) *= g2.flat(i); } return g1; }
TTA G& operator/=(G& g1, const G& g2) { F { g1.flat(i) /= g2.flat(i); } return g1; }
TTA G& operator%=(G& g1, const G& g2) { F { g1.flat(i) %= g2.flat(i); } return g1; }

TTA G& operator+=(G& g1, const T& v) { F { g1.flat(i) += v; } return g1; }
TTA G& operator-=(G& g1, const T& v) { F { g1.flat(i) -= v; } return g1; }
TTA G& operator*=(G& g1, const T& v) { F { g1.flat(i) *= v; } return g1; }
TTA G& operator/=(G& g1, const T& v) { F { g1.flat(i) /= v; } return g1; }
TTA G& operator%=(G& g1, const T& v) { F { g1.flat(i) %= v; } return g1; }

TTC G operator-(const G& g1) { G g; F { g.flat(i) = -g1.flat(i); } return g; }

TTC G min(const G& g1, const G& g2) { G g; F { g.flat(i) = min(g1.flat(i), g2.flat(i)); } return g; }
TTC G max(const G& g1, const G& g2) { G g; F { g.flat(i) = max(g1.flat(i), g2.flat(i)); } return g; }

TTC G interp(const G& g1, const G& g2, float f1 = 0.5f) {
  G g; F { g.flat(i) = f1 * g1.flat(i) + (1.f - f1) * g2.flat(i); } return g;
}
TTC G interp(const G& g1, const G& g2, const G& g3, float f1, float f2) {
  G g; F { g.flat(i) = f1 * g1.flat(i) + f2 * g2.flat(i) + (1.f - f1 - f2) * g3.flat(i); } return g;
}
TTC G interp(const G& g1, const G& g2, const G& g3) { return interp(g1, g2, g3, 1.f / 3.f, 1.f / 3.f); }
TTC G interp(const G& g1, const G& g2, const G& g3, const Vec3<float>& bary) {
  // Vec3<float> == Bary;   May have sum(bary) != 1.f.
  G g; F { g.flat(i) = bary[0] * g1.flat(i) + bary[1] * g2.flat(i) + bary[2] * g3.flat(i); } return g;
}

// clang-format on
#undef F
#undef G
#undef TTA
#undef TTC
#undef TT

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SGRID_H_
