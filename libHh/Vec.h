// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_VEC_H_
#define MESH_PROCESSING_LIBHH_VEC_H_

#include "libHh/Advanced.h"  // my_hash(), hash_combine()
#include "libHh/Array.h"     // ArrayView<>, CArrayView<>

namespace hh {

namespace details {
template <typename T, int n> struct VecBase;
template <int D> class Vec_range;
template <int D> class VecL_range;
}  // namespace details

// Allocated fixed-size 1D array with n elements of type T.
// Like std::array<T, n>, but with constructors and "empty base class optimization" support for n == 0.
template <typename T, int n> class Vec : details::VecBase<T, n> {
  static_assert(n >= 0);
  using type = Vec<T, n>;
  using base = details::VecBase<T, n>;

 public:
  Vec() = default;
  // Include arg0 to disambiguate from default constructor.  arg0 may be either const l-value or r-value reference.
  template <typename... Args>
  constexpr Vec(const T& arg0, Args&&... args1) noexcept requires Copyable<T>
      : base{{arg0, std::forward<Args>(args1)...}} {
    static_assert(sizeof...(args1) + 1 == n, "#args");
  }
  template <typename... Args>
  constexpr Vec(T&& arg0, Args&&... args1) noexcept : base{{std::move(arg0), std::forward<Args>(args1)...}} {
    static_assert(sizeof...(args1) + 1 == n, "#args");
  }

  Vec(CArrayView<T> ar) requires Copyable<T> { assign(ar); }
  // To allow class to be trivial, and to allow generation of implicit move constructor and assignment,
  //  it is safest to not include any copy-constructor, not even a default one.
  [[HH_GNU_PURE]] [[nodiscard]] constexpr int num() const { return n; }
  [[nodiscard]] constexpr size_t size() const { return static_cast<size_t>(n); }
  [[HH_GNU_PURE]] [[nodiscard]] constexpr auto& operator[](this auto&& self, int i) {
    return HH_CHECK_BOUNDS(i, n), as_vec(self).data()[i];
  }
  [[nodiscard]] constexpr auto& last(this auto&& self) { return self[n - 1]; }
  [[nodiscard]] constexpr bool ok(int i) const { return i >= 0 && i < n; }
  constexpr void assign(CArrayView<T> ar) requires Copyable<T> {
    ASSERTXX(ar.num() == n);
    std::copy(ar.data(), ar.data() + n, data());
  }
  [[nodiscard]] constexpr type rev() const requires Copyable<T> { return rev_aux(std::make_index_sequence<n>()); }
  [[nodiscard]] constexpr bool in_range(const type& dims) const requires std::integral<T> {  // [c] < uU[c] for all c.
    return in_range(type::all(T{}), dims);
  }
  // uL[c] <= [c] < uU[c] for all c.
  [[nodiscard]] constexpr bool in_range(const type& uL, const type& uU) const requires std::integral<T>;
  [[nodiscard]] constexpr type with(int i, T e) const& requires Copyable<T> {
    type ar(*this);
    ar[i] = std::move(e);
    return ar;
  }
  [[nodiscard]] constexpr type with(int i, T e) && {
    operator[](i) = std::move(e);
    return std::move(*this);
  }
  [[nodiscard]] constexpr bool operator==(const type& rhs) const {
    for_int(i, n) if (data()[i] != rhs[i]) return false;
    return true;
  }
  // Enable lexicographic ordering, e.g. to use Vec as a key in std::map or std::set.
  [[nodiscard]] friend constexpr auto operator<=>(const type&, const type&) = default;
  [[nodiscard]] constexpr operator ArrayView<T>() { return view(); }
  [[nodiscard]] constexpr operator CArrayView<T>() const { return view(); }
  [[nodiscard]] constexpr auto view(this auto&& self) { return array_view_t<decltype(self.data())>(self.data(), n); }
  [[nodiscard]] constexpr CArrayView<T> const_view() const { return CArrayView<T>(data(), n); }
  [[nodiscard]] constexpr auto& vec(this auto&& self) { return as_vec(self); }

  // The segment(), head(), and tail() functions returning Vec<T, s>& reinterpret the s elements starting at index i
  // as a Vec<T, s>.  This relies on Vec<T, n> being a standard-layout type whose only storage is a T[n] at offset 0,
  // so that the two are layout-compatible.  It is not strictly conforming (no Vec<T, s> object is ever created at
  // that address), which is also why these functions cannot be constexpr; the alternative of returning a copy would
  // lose the ability to write through the result, e.g. frame.head<3>() = ...
  // V(1, 2, 3).head<2>() == V(1, 2).
  template <int i, int s> [[nodiscard]] auto& segment(this auto&& self) {  // V(1, 2, 3, 4).segment<1, 2>() == V(2, 3).
    static_assert(i >= 0 && s >= 0 && i + s <= n);
    return *reinterpret_cast<copy_const_t<decltype(self), Vec<T, s>>*>(self.data() + i);
  }
  template <int s> [[nodiscard]] auto& segment(this auto&& self, int i) {  // V(1, 2, 3, 4).segment<2>(1) == V(2, 3).
    static_assert(s >= 0 && s <= n);
    ASSERTXX(as_vec(self).check(i, s));
    return *reinterpret_cast<copy_const_t<decltype(self), Vec<T, s>>*>(self.data() + i);
  }
  [[nodiscard]] constexpr auto segment(this auto&& self, int i, int s) {
    return ASSERTXX(as_vec(self).check(i, s)), array_view_t<decltype(self.data())>(self.data() + i, s);
  }
  template <int s> [[nodiscard]] auto& head(this auto&& self) { return as_vec(self).template segment<0, s>(); }
  [[nodiscard]] constexpr auto head(this auto&& self, int s) { return as_vec(self).segment(0, s); }
  // V(1, 2, 3).tail<2>() == V(2, 3).
  template <int s> [[nodiscard]] auto& tail(this auto&& self) { return as_vec(self).template segment<n - s, s>(); }
  [[nodiscard]] constexpr auto tail(this auto&& self, int s) { return as_vec(self).segment(n - s, s); }
  [[nodiscard]] constexpr auto slice(this auto&& self, int ib, int ie) { return as_vec(self).segment(ib, ie - ib); }

  template <typename U> [[nodiscard]] constexpr Vec<U, n> cast() const {
    return transformed(*this, [](const auto& e) { return static_cast<U>(e); });
  }
  using value_type = T;
  using iterator = T*;
  using const_iterator = const T*;
  [[nodiscard]] constexpr T* begin() { return data(); }
  [[nodiscard]] constexpr const T* begin() const { return data(); }
  [[nodiscard]] constexpr T* end() { return data() + n; }
  [[nodiscard]] constexpr const T* end() const { return data() + n; }
  [[nodiscard]] constexpr T* data() {
    if constexpr (n > 0)
      return this->_a;
    else
      return nullptr;
  }
  [[nodiscard]] constexpr const T* data() const {
    if constexpr (n > 0)
      return this->_a;
    else
      return nullptr;
  }
  [[nodiscard]] static constexpr type all(const T& e) requires Copyable<T> {
    return all_aux(e, std::make_index_sequence<n>());
  }
  static constexpr int Num = n;
  // E.g.: const auto vec = Vec<float, 3>::create([](int i) { return i * .5f; }).
  template <typename Func> requires std::is_invocable_r_v<T, Func, int>
  [[nodiscard]] static constexpr type create(Func func) {
    return [&]<size_t... Is>(std::index_sequence<Is...>) {
      return type(func(int{Is})...);
    }(std::make_index_sequence<n>());
  }
  constexpr bool normalize() requires std::floating_point<T> {
    auto sum2 = mag2(*this);
    if (!sum2) return false;
    *this *= 1.f / sqrt(sum2);
    return true;
  }
  // Enable structured bindings.
  template <std::size_t Index> [[nodiscard]] constexpr auto&& get(this auto&& self) {
    static_assert(Index < std::size_t{n});
    return std::forward_like<decltype(self)>(self[int{Index}]);
  }
  // Defining friend functions in-class is convenient but unfortunately _MSC_VER attempts to instantiate
  // all these and this fails if "T - T" or "sqrt(T)" are undefined.
  // [[nodiscard]] friend constexpr T mag2(const Vec<T, n>& vec) { return dot(vec, vec); }

  using owns_elements = void;  // Marker for hh::clone().

 private:
  // C++ requires empty classes to have nonzero size to ensure object identity.
  // Therefore, even with an empty struct, it is necessary that sizeof(Vec<T, 0>) > 0.
  // However, using the "empty base class optimization", a class derived from Vec<T, 0> has zero space overhead.

  [[nodiscard]] constexpr bool check(int i, int s) const {
    if (i >= 0 && s >= 0 && i + s <= n) return true;
    if !consteval {
      SHOW(i, s, n);
    }
    return false;
  }
  // Deducing `this` deduces the *derived* class, so an unqualified member call on `self` can be hijacked by a
  // same-named member declared there (e.g. SGrid::segment() hides Vec::segment()).  Member calls that must resolve
  // to Vec's own members therefore go through as_vec().  Unqualified lookup of as_vec() finds this class member, so
  // ADL is suppressed and the call is never itself hijacked.
  template <typename Self> [[nodiscard]] static constexpr auto& as_vec(Self&& self) {
    return static_cast<copy_const_t<Self, type>&>(self);
  }
  template <size_t... Is> constexpr type rev_aux(std::index_sequence<Is...>) const requires Copyable<T> {
    return type(data()[n - 1 - Is]...);
  }
  template <size_t... Is> static constexpr type all_aux(const T& e, std::index_sequence<Is...>) requires Copyable<T> {
    return type((void(Is), e)...);
  }
  // Default operator=() and copy_constructor are safe.
};

template <typename T> using Vec0 = Vec<T, 0>;
template <typename T> using Vec1 = Vec<T, 1>;
template <typename T> using Vec2 = Vec<T, 2>;
template <typename T> using Vec3 = Vec<T, 3>;
template <typename T> using Vec4 = Vec<T, 4>;

// Construct a Vec from an immediate list of elements, inferring the element type (unless it is explicitly specified)
// and the array size automatically.
template <typename T = void, typename Arg0, typename... Args>
[[nodiscard]] constexpr auto V(Arg0&& arg0, Args&&... args) {
  using Elem = std::conditional_t<std::is_void_v<T>, std::decay_t<Arg0>, T>;
  return Vec<Elem, 1 + sizeof...(Args)>(std::forward<Arg0>(arg0), std::forward<Args>(args)...);
}

// Construct a zero-length Vec.
template <typename T> [[nodiscard]] constexpr Vec<T, 0> V() { return Vec<T, 0>(); }

// Construct a Vec from a braced list, inferring the size (like std::to_array).
template <typename T, size_t n> [[nodiscard]] constexpr Vec<T, int(n)> to_Vec(T (&&a)[n]) {
  return [&]<size_t... i>(std::index_sequence<i...>) {
    return Vec<T, int(n)>(std::move(a[i])...);
  }(std::make_index_sequence<n>());
}

// Construct an Vec with two identical elements, e.g. twice(v) == V(v, v).
template <typename T> [[nodiscard]] constexpr Vec2<T> twice(const T& v) { return {v, v}; }

// Construct an Vec with three identical elements, e.g. thrice(v) == V(v, v, v).
template <typename T> [[nodiscard]] constexpr Vec3<T> thrice(const T& v) { return {v, v, v}; }

// Construct an Vec with identical elements, e.g. ntimes<4>(.5f) == V(.5f, .5f, .5f, .5f).
template <int n, typename T> [[nodiscard]] constexpr Vec<T, n> ntimes(const T& v) { return Vec<T, n>::all(v); }

// Given container c, evaluate func() on each element (possibly changing the element type) and return new container.
template <typename T, int n, typename Func> [[nodiscard]] constexpr auto transformed(const Vec<T, n>& c, Func func) {
  using ResultType = std::decay_t<std::invoke_result_t<Func, const T&>>;
  return Vec<ResultType, n>::create([&](int i) { return func(c[i]); });
}

// Range of coordinates: Vec<int, D>: 0 <= [0] < uU[0], 0 <= [1] < uU[1], ..., 0 <= [D - 1] < uU[D - 1].
//  e.g.: for (const auto& p : range(grid.dims())) grid[p] = func(p);
template <int D> [[nodiscard]] details::Vec_range<D> range(const Vec<int, D>& uU);

// Range of coordinates: Vec<int, D>: uL[0] <= [0] < uU[0], ..., uL[D - 1] <= [D - 1] < uU[D - 1].
template <int D> [[nodiscard]] details::VecL_range<D> range(const Vec<int, D>& uL, const Vec<int, D>& uU);

// Concatenate several Vec's to create single Vec, e.g. concat(V(1, 2), V(3), V(4, 5)) == V(1, 2, 3, 4, 5).
template <typename T, int n1, int n2, typename... A>
[[nodiscard]] constexpr Vec<T, (n1 + n2 + (A::Num + ... + 0))> concat(const Vec<T, n1>& a1, const Vec<T, n2>& a2,
                                                                      const A&... arr) {
  return concat(
      [&]<size_t... Is1, size_t... Is2>(std::index_sequence<Is1...>, std::index_sequence<Is2...>) {
        return Vec<T, n1 + n2>(a1[int{Is1}]..., a2[int{Is2}]...);
      }(std::make_index_sequence<n1>(), std::make_index_sequence<n2>()),
      arr...);
}
template <typename T, int n1> [[nodiscard]] constexpr Vec<T, n1> concat(const Vec<T, n1>& a1) { return a1; }

//----------------------------------------------------------------------------

template <Numeric T, int n> [[nodiscard]] constexpr T dot(const Vec<T, n>& v1, const Vec<T, n>& v2) {
  T sum{};
  for_int(i, n) sum += v1[i] * v2[i];
  return sum;
}

template <Numeric T, int n> [[nodiscard]] constexpr T mag2(const Vec<T, n>& vec) { return dot(vec, vec); }
template <std::floating_point T, int n> [[nodiscard]] constexpr T mag(const Vec<T, n>& vec) { return sqrt(mag2(vec)); }

template <Numeric T, int n> [[nodiscard]] constexpr T dist2(const Vec<T, n>& v1, const Vec<T, n>& v2) {
  return mag2(v1 - v2);
}
template <std::floating_point T, int n> [[nodiscard]] constexpr T dist(const Vec<T, n>& v1, const Vec<T, n>& v2) {
  return sqrt(dist2(v1, v2));
}

template <std::floating_point T, int n> [[nodiscard]] constexpr Vec<T, n> normalized(Vec<T, n> vec) {
  assertx(vec.normalize());
  return vec;
}
template <std::floating_point T, int n> [[nodiscard]] constexpr Vec<T, n> ok_normalized(Vec<T, n> vec) {
  vec.normalize();
  return vec;
}
template <std::floating_point T, int n> [[nodiscard]] constexpr Vec<T, n> fast_normalized(const Vec<T, n>& vec) {
  return vec / mag(vec);
}

template <std::floating_point T, int n>
[[nodiscard]] constexpr bool is_unit(const Vec<T, n>& vec, float tolerance = 1e-4f) {
  return abs(mag2(vec) - 1.f) <= tolerance;
}

template <std::floating_point T> [[nodiscard]] constexpr T snap_coordinate(T value) {
  const T eps = 1e-6f;
  if (abs(value - 0.f) < eps) return 0.f;
  if (abs(value - 1.f) < eps) return 1.f;
  if (abs(value + 1.f) < eps) return -1.f;
  return value;
}

template <std::floating_point T, int n> [[nodiscard]] constexpr Vec<T, n> snap_coordinates(Vec<T, n> vec) {
  return transformed(vec, [](const T& e) { return snap_coordinate(e); });
}

//----------------------------------------------------------------------------

namespace details {

// Storage for Vec.  The n == 0 specialization is a truly empty class -- not merely one with a [[no_unique_address]]
// empty member, which compilers do not agree is eligible for the empty base optimization -- so that Vec<T, 0>, and
// anything derived from it such as SGrid<T, 0, ...>, occupies no space in a derived class.
template <typename T, int n> struct VecBase {
  T _a[n];
  friend auto operator<=>(const VecBase&, const VecBase&) = default;
};
template <typename T> struct VecBase<T, 0> {
  friend auto operator<=>(const VecBase&, const VecBase&) = default;
};

}  // namespace details

//----------------------------------------------------------------------------

// There is a way to unify Vec_range and VecL_range ??

// ?? The coordinate iterators are not C++20 iterators, so range(grid.dims()) can't feed std::ranges at
// all. difference_type = void fails std::weakly_incrementable, there's no operator==, no post-increment, and
// no default construction. Fixing this unlocks range(dims) | views::filter(...), ranges::for_each,
// and parallel algorithms. The idiomatic shape also removes the fake end iterator:
//  using difference_type = std::ptrdiff_t;
//  bool operator==(std::default_sentinel_t) const { return _u[0] >= _uU[0]; }
// That also removes the current constraint - undocumented, and enforced only by an ASSERTXX - that iterators
// may only be compared against end().

namespace details {

// Iterator for traversing coordinates: 0 <= [0] < uU[0], 0 <= [1] < uU[1], ..., 0 <= [D - 1] < uU[D - 1].
template <int D> class Vec_iterator {
  using type = Vec_iterator<D>;

 public:
  // TODO: satisfy the std::ranges concept by defining a proper sentinel type.
  using iterator_category = std::forward_iterator_tag;
  using value_type = Vec<int, D>;
  using difference_type = void;
  Vec_iterator(const Vec<int, D>& u, const Vec<int, D>& uU) : _u(u), _uU(uU) {}
  Vec_iterator(const type& iter) = default;
  Vec_iterator() = default;
  bool operator!=(const type& rhs) const {
    dummy_use(rhs);
    ASSERTXX(rhs._uU == _uU);
    ASSERTXX(rhs._u[0] == _uU[0]);
    return _u[0] < _uU[0];  // Quick check against usual end().
  }
  const Vec<int, D>& operator*() const { return ASSERTXX(_u[0] < _uU[0]), _u; }
  type& operator++() {
    static_assert(D > 0);
    ASSERTXX(_u[0] < _uU[0]);
    if constexpr (D == 1) {
      _u[0]++;
      return *this;
    } else if constexpr (D == 2) {  // Else VC12 does not unroll this tiny loop.
      if (++_u[1] < _uU[1]) return *this;
      _u[1] = 0;
      ++_u[0];
      return *this;
    } else {
      int c = D - 1;  // Here to avoid warning about loop condition in VC14 code analysis.
      for (; c > 0; --c) {
        if (++_u[c] < _uU[c]) return *this;
        _u[c] = 0;
      }
      _u[0]++;
      return *this;
    }
  }
  type operator++(int) { return postfix_increment(*this); }

 private:
  Vec<int, D> _u{}, _uU{};
};

// Range of coordinates 0 <= [0] < uU[0], 0 <= [1] < uU[1], ..., 0 <= [D - 1] < uU[D - 1].
template <int D> class Vec_range {
 public:
  Vec_range(const Vec<int, D>& uU) : _uU(uU) {}
  Vec_iterator<D> begin() const { return Vec_iterator<D>(ntimes<D>(0), _uU); }
  Vec_iterator<D> end() const { return Vec_iterator<D>(_uU, _uU); }
  size_t size() const {
    size_t product = 1;
    for_int(c, D) product *= _uU[c];
    return product;
  }

 private:
  Vec<int, D> _uU;
};

// Iterator for traversing coordinates: uL[0] <= [0] < uU[0], ..., uL[D - 1] <= [D - 1] < uU[D - 1].
template <int D> class VecL_iterator {
  using type = VecL_iterator<D>;

 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type = Vec<int, D>;
  using difference_type = void;
  VecL_iterator(const Vec<int, D>& uL, const Vec<int, D>& uU) : _u(uL), _uL(uL), _uU(uU) {}
  VecL_iterator(const type& iter) = default;
  VecL_iterator() = default;
  bool operator!=(const type& rhs) const {
    dummy_use(rhs);
    ASSERTXX(rhs._uU == _uU);
    ASSERTXX(rhs._u[0] == _uU[0]);
    return _u[0] < _uU[0];  // Quick check against usual end().
  }
  const Vec<int, D>& operator*() const { return ASSERTXX(_u[0] < _uU[0]), _u; }
  type& operator++() {
    ASSERTXX(_u[0] < _uU[0]);
    for (int c = D - 1; c > 0; --c) {
      _u[c]++;
      if (_u[c] < _uU[c]) return *this;
      _u[c] = _uL[c];
    }
    _u[0]++;
    return *this;
  }
  type operator++(int) { return postfix_increment(*this); }

 private:
  Vec<int, D> _u{}, _uL{}, _uU{};
};

// Range of coordinates uL[0] <= [0] < uU[0], ..., uL[D - 1] <= [D - 1] < uU[D - 1].
template <int D> class VecL_range {
 public:
  VecL_range(const Vec<int, D>& uL, const Vec<int, D>& uU) : _uL(uL), _uU(uU) {}
  VecL_iterator<D> begin() const { return VecL_iterator<D>(_uL, _uU); }
  VecL_iterator<D> end() const { return VecL_iterator<D>(_uU, _uU); }
  size_t size() const {
    size_t product = 1;
    for_int(c, D) product *= _uU[c] - _uL[c];
    return product;
  }

 private:
  Vec<int, D> _uL, _uU;
};

}  // namespace details

template <int D> details::Vec_range<D> range(const Vec<int, D>& uU) {
  for_int(c, D) {
    if (uU[c] <= 0) return details::Vec_range<D>(ntimes<D>(0));
  }
  return details::Vec_range<D>(uU);
}

template <int D> details::VecL_range<D> range(const Vec<int, D>& uL, const Vec<int, D>& uU) {
  for_int(c, D) {
    if (uU[c] <= uL[c]) return details::VecL_range<D>(uU, uU);
  }
  return details::VecL_range<D>(uL, uU);
}

//----------------------------------------------------------------------------

template <typename T, int n>
constexpr bool Vec<T, n>::in_range(const Vec<T, n>& uL, const Vec<T, n>& uU) const requires std::integral<T> {
  for_int(c, n) {
    if ((*this)[c] < uL[c] || (*this)[c] >= uU[c]) return false;
  }
  return true;
}

template <typename T, int n> std::ostream& operator<<(std::ostream& os, const Vec<T, n>& ar) {
  if (has_ostream_eol_v<T>) {
    os << "Vec<" << type_name<T>() << "," << n << "> {\n";
    for_int(i, n) os << "  " << ar[i];
    return os << "}\n";
  } else {
    os << "[";
    for_int(i, n) os << (i ? ", " : "") << ar[i];
    return os << "]";
  }
}

template <typename T, int n> inline constexpr bool has_ostream_eol_aux_v<Vec<T, n>> = has_ostream_eol_v<T>;

//----------------------------------------------------------------------------

// These Vec operations may be more efficient than similar ArrayView operations because
//  (1) n is known, and
//  (2) there is no heap allocation.

#if 0
// We could define the following:
template <typename T, typename T2, int n> auto operator+(const Vec<T, n>& g1, const Vec<T2, n>& g2) {
  // using ReturnType = std::common_type_t<T, T2>;
  using ReturnType = std::decay_t<decltype(std::declval<T>() + std::declval<T2>())>;
  Vec<ReturnType, n> ar;
  for_int(i, n) ar[i] = g1[i] + g2[i];
  return ar;
}
#endif

// Set of functions common to Vec.h, SGrid.h, Array.h, Grid.h
// Note that RangeOp.h functions are valid here: mag2(), mag(), dist2(), dist(), dot(), is_zero(), compare().
// See also floor(), ceil(), abs() generalized to Vec<> in MathOp.h.
#define TT template <typename T, int n>
#define TTC TT [[nodiscard]] constexpr
#define TTA TT constexpr
#define G Vec<T, n>
#define F for_int(i, n)
// clang-format off

TTC G operator+(const G& g1, const G& g2) { G g; F { g[i] = g1[i] + g2[i]; } return g; }
TTC G operator-(const G& g1, const G& g2) { G g; F { g[i] = g1[i] - g2[i]; } return g; }
TTC G operator*(const G& g1, const G& g2) { G g; F { g[i] = g1[i] * g2[i]; } return g; }
TTC G operator/(const G& g1, const G& g2) { G g; F { g[i] = g1[i] / g2[i]; } return g; }
TTC G operator%(const G& g1, const G& g2) { G g; F { g[i] = g1[i] % g2[i]; } return g; }

TTC G operator+(const G& g1, T v) { G g; F { g[i] = g1[i] + v; } return g; }
TTC G operator-(const G& g1, T v) { G g; F { g[i] = g1[i] - v; } return g; }
TTC G operator*(const G& g1, T v) { G g; F { g[i] = g1[i] * v; } return g; }
TTC G operator/(const G& g1, T v) { G g; F { g[i] = g1[i] / v; } return g; }
TTC G operator%(const G& g1, T v) { G g; F { g[i] = g1[i] % v; } return g; }

TTC G operator+(T v, const G& g1) { G g; F { g[i] = v + g1[i]; } return g; }
TTC G operator-(T v, const G& g1) { G g; F { g[i] = v - g1[i]; } return g; }
TTC G operator*(T v, const G& g1) { G g; F { g[i] = v * g1[i]; } return g; }
TTC G operator/(T v, const G& g1) { G g; F { g[i] = v / g1[i]; } return g; }
TTC G operator%(T v, const G& g1) { G g; F { g[i] = v % g1[i]; } return g; }

TTA G& operator+=(G& g1, const G& g2) { F { g1[i] += g2[i]; } return g1; }
TTA G& operator-=(G& g1, const G& g2) { F { g1[i] -= g2[i]; } return g1; }
TTA G& operator*=(G& g1, const G& g2) { F { g1[i] *= g2[i]; } return g1; }
TTA G& operator/=(G& g1, const G& g2) { F { g1[i] /= g2[i]; } return g1; }
TTA G& operator%=(G& g1, const G& g2) { F { g1[i] %= g2[i]; } return g1; }

TTA G& operator+=(G& g1, const T& v) { F { g1[i] += v; } return g1; }
TTA G& operator-=(G& g1, const T& v) { F { g1[i] -= v; } return g1; }
TTA G& operator*=(G& g1, const T& v) { F { g1[i] *= v; } return g1; }
TTA G& operator/=(G& g1, const T& v) { F { g1[i] /= v; } return g1; }
TTA G& operator%=(G& g1, const T& v) { F { g1[i] %= v; } return g1; }

TTC G operator-(const G& g1) { G g; F { g[i] = -g1[i]; } return g; }

TTC G min(const G& g1, const G& g2) { G g; F { g[i] = min(g1[i], g2[i]); } return g; }
TTC G max(const G& g1, const G& g2) { G g; F { g[i] = max(g1[i], g2[i]); } return g; }

TTC G clamp(const G& g1, T vmin, T vmax) { G g; F { g[i] = clamp(g1[i], vmin, vmax); } return g; }

TTC G interp(const G& g1, const G& g2, float f1 = 0.5f) {
  G g; F { g[i] = f1 * g1[i] + (1.f - f1) * g2[i]; } return g;
}
TTC G interp(const G& g1, const G& g2, const G& g3, float f1, float f2) {
  G g; F { g[i] = f1 * g1[i] + f2 * g2[i] + (1.f - f1 - f2) * g3[i]; } return g;
}
TTC G interp(const G& g1, const G& g2, const G& g3) { return interp(g1, g2, g3, 1.f / 3.f, 1.f / 3.f); }
TTC G interp(const G& g1, const G& g2, const G& g3, const Vec3<float>& bary) {
  // Vec3<float> == Bary;  may have sum(bary) != 1.f
  G g; F { g[i] = bary[0] * g1[i] + bary[1] * g2[i] + bary[2] * g3[i]; } return g;
}

// clang-format on
#undef F
#undef G
#undef TTA
#undef TTC
#undef TT

template <typename Class>
concept DerivedFromVec = std::is_base_of_v<Vec<typename Class::value_type, Class::Num>, Class>;

template <typename T> T interp(const Vec3<T>& triple, float f1, float f2) {
  return interp(triple[0], triple[1], triple[2], f1, f2);
}
template <typename T> T interp(const Vec3<T>& triple) {  //
  return interp(triple[0], triple[1], triple[2]);
}
// DerivedFromVec to disambiguate from "Point interp(Point, Point)".
template <DerivedFromVec SomeVec> SomeVec interp(const Vec3<SomeVec>& triple, const Vec3<float>& bary) {
  return interp(triple[0], triple[1], triple[2], bary);
}

// Template deduction guides:
template <typename T, typename... Args> Vec(T, Args...) -> Vec<T, 1 + sizeof...(Args)>;

}  // namespace hh

//----------------------------------------------------------------------------

// Enable structured bindings.
template <typename T, int n> struct std::tuple_size<hh::Vec<T, n>> : std::integral_constant<std::size_t, n> {};
template <std::size_t Index, typename T, int n> struct std::tuple_element<Index, hh::Vec<T, n>> {
  using type = T;
};

//----------------------------------------------------------------------------

template <typename T, int n> struct std::hash<hh::Vec<T, n>> {
  [[nodiscard]] size_t operator()(const hh::Vec<T, n>& ar) const {
    size_t h = 0;
    for_int(i, n) h = hh::hash_combine(h, ar[i]);
    return h;
  }
};

#endif  // MESH_PROCESSING_LIBHH_VEC_H_
