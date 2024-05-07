// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_VEC_H_
#define MESH_PROCESSING_LIBHH_VEC_H_

#include "libHh/Array.h"  // ArrayView<>, CArrayView<>

namespace hh {

namespace details {
template <typename T, int n> struct Vec_base;
template <int D> class Vec_range;
template <int D> class VecL_range;
}  // namespace details

// Allocated fixed-size 1D array with n elements of type T.
// Like std::array<T, n>, but with constructors and "empty base class optimization" support for n == 0.
template <typename T, int n> class Vec : details::Vec_base<T, n> {
  static_assert(n >= 0, "");
  using type = Vec<T, n>;
  using base = details::Vec_base<T, n>;

 public:
  Vec() = default;
  Vec(CArrayView<T> ar) { assign(ar); }
  // Include arg0 to disambiguate from default constructor.  arg0 may be either const l-value or r-value reference.
  template <typename... Args>
  constexpr Vec(const T& arg0, Args&&... args1) noexcept : base(nullptr, arg0, std::forward<Args>(args1)...) {}
  template <typename... Args>
  constexpr Vec(T&& arg0, Args&&... args1) noexcept : base(nullptr, std::move(arg0), std::forward<Args>(args1)...) {}
  // To allow class to be trivial, and to allow generation of implicit move constructor and assignment,
  //  it is safest to not include any copy-constructor, not even a default one.
  constexpr int num() const { return n; }
  constexpr size_t size() const { return n; }
  T& operator[](int i) { return (HH_CHECK_BOUNDS(i, n), base::operator[](i)); }
  constexpr const T& operator[](int i) const { return (HH_CHECK_BOUNDS(i, n), base::operator[](i)); }
  T& last() { return (*this)[n - 1]; }
  const T& last() const { return (*this)[n - 1]; }
  bool ok(int i) const { return i >= 0 && i < n; }
  void assign(CArrayView<T> ar) { assign_i(ar); }
  constexpr type rev() const { return rev_aux(std::make_index_sequence<n>()); }
  bool in_range(const type& dims) const { return in_range(type::all(T{}), dims); }
  bool in_range(const type& uL, const type& uU) const;  // true if uL[c] <= [c] < uU[c] for all c in [0, n - 1]
  // type with(int i, const T& e) const& { type ar(*this); ar[i] = e; return ar; }
  type with(int i, T e) const& {
    type ar(*this);
    ar[i] = std::move(e);
    return ar;
  }
  type with(int i, T e) && {
    operator[](i) = std::move(e);
    return *this;
  }
  bool operator==(const type& p) const {
    for_int(i, n) {
      if (!(a()[i] == p[i])) return false;
    }
    return true;
  }
  bool operator!=(const type& p) const { return !(*this == p); }
  operator ArrayView<T>() { return view(); }
  operator CArrayView<T>() const { return view(); }
  ArrayView<T> view() { return ArrayView<T>(a(), n); }
  CArrayView<T> view() const { return CArrayView<T>(a(), n); }
  template <int s> Vec<T, s>& head() { return segment<s>(0); }  // V(1, 2, 3).head<2>() == V(1, 2)
  template <int s> const Vec<T, s>& head() const { return segment<s>(0); }
  ArrayView<T> head(int s) { return segment(0, s); }
  CArrayView<T> head(int s) const { return segment(0, s); }
  template <int s> Vec<T, s>& tail() { return segment<s>(n - s); }  // V(1, 2, 3).tail<2>() == V(2, 3)
  template <int s> const Vec<T, s>& tail() const { return segment<s>(n - s); }
  ArrayView<T> tail(int s) { return segment(n - s, s); }
  CArrayView<T> tail(int s) const { return segment(n - s, s); }
  template <int i, int s> Vec<T, s>& segment() {  // V(1, 2, 3, 4).segment<2, 1> == V(2, 3)
    static_assert(i >= 0 && s >= 0 && i + s <= n, "");
    return *reinterpret_cast<Vec<T, s>*>(a() + i);
  }
  template <int i, int s> const Vec<T, s>& segment() const {
    static_assert(i >= 0 && s >= 0 && i + s <= n, "");
    return *reinterpret_cast<const Vec<T, s>*>(a() + i);
  }
  template <int s> Vec<T, s>& segment(int i) {  // V(1, 2, 3, 4).segment<2>(1) == V(2, 3)
    static_assert(s >= 0 && s <= n, "");
    ASSERTXX(check(i, s));
    return *reinterpret_cast<Vec<T, s>*>(a() + i);
  }
  template <int s> const Vec<T, s>& segment(int i) const {
    static_assert(s >= 0 && s <= n, "");
    ASSERTXX(check(i, s));
    return *reinterpret_cast<const Vec<T, s>*>(a() + i);
  }
  ArrayView<T> segment(int i, int s) { return (ASSERTXX(check(i, s)), ArrayView<T>(a() + i, s)); }
  CArrayView<T> segment(int i, int s) const { return (ASSERTXX(check(i, s)), CArrayView<T>(a() + i, s)); }
  ArrayView<T> slice(int ib, int ie) { return segment(ib, ie - ib); }
  CArrayView<T> slice(int ib, int ie) const { return segment(ib, ie - ib); }
  template <typename U> Vec<U, n> cast() const {
    Vec<U, n> v;
    for_int(i, n) v[i] = static_cast<U>((*this)[i]);
    return v;
  }
  using value_type = T;
  using iterator = T*;
  using const_iterator = const T*;
  T* begin() { return a(); }
  const T* begin() const { return a(); }
  T* end() { return a() + n; }
  const T* end() const { return a() + n; }
  T* data() { return a(); }
  const T* data() const { return a(); }
  static constexpr type all(const T& e) { return all_aux(e, std::make_index_sequence<n>()); }
  static constexpr int Num = n;
  template <typename Func = T(int)> static type create(Func func) {
    return create_aux(func, std::make_index_sequence<n>());
  }
  // (std::is_floating_point_v<> requires C++17.)
  template <typename U = T> std::enable_if_t<std::is_floating_point<U>::value, bool> normalize() {
    auto sum2 = mag2(*this);
    if (!sum2) return false;
    *this *= 1.f / sqrt(sum2);
    return true;
  }
  // Defining friend functions in-class is convenient but unfortunately _MSC_VER (VS 2019) attempts to instantiate
  // all these and this fails if "T - T" or "sqrt(T)" are undefined.
  // friend T mag2(const type& vec) { return dot(vec, vec); }
  // friend T mag(const type& vec) { return sqrt(mag2(vec)); }

 private:
  // C++ requires empty classes to have nonzero size to ensure object identity.
  // Therefore, even with an empty struct, it is necessary that sizeof(Vec<T, 0>) > 0.
  // However, using the "empty base class optimization", a class derived from Vec<T, 0> has zero space overhead.
  void assign_i(CArrayView<T> ar) {
    // ASSERTX(ar.num() == n); for_int(i, n) a()[i] = ar[i];
    ASSERTX(ar.num() == n);
    std::copy(ar.data(), ar.data() + n, a());
  }
  bool check(int i, int s) const {
    if (i >= 0 && s >= 0 && i + s <= n) return true;
    SHOW(i, s, n);
    return false;
  }
  template <size_t... Is> constexpr type rev_aux(std::index_sequence<Is...>) const {
    return type(base::operator[](n - 1 - Is)...);
  }
  template <size_t... Is> static constexpr type all_aux(const T& e, std::index_sequence<Is...>) {
    return type((void(Is), e)...);
  }
  template <typename Func = T(int), size_t... Is> static type create_aux(Func func, std::index_sequence<Is...>) {
    return type(func(Is)...);
  }
  using base::a;
  // Default operator=() and copy_constructor are safe.
};

template <typename T, int n> using SArray = Vec<T, n>;  // backwards compatibility
template <typename T> using Vec0 = Vec<T, 0>;
template <typename T> using Vec1 = Vec<T, 1>;
template <typename T> using Vec2 = Vec<T, 2>;
template <typename T> using Vec3 = Vec<T, 3>;
template <typename T> using Vec4 = Vec<T, 4>;

// Construct an Vec from an immediate list of elements, inferring the element type and array size automatically.
template <typename T, typename... Ts> constexpr Vec<std::decay_t<T>, (1 + sizeof...(Ts))> V(const T& t, Ts&&... ts) {
  return Vec<std::decay_t<T>, 1 + sizeof...(ts)>(t, std::forward<Ts>(ts)...);
}

// Construct an Vec from an immediate list of elements, inferring the element type and array size automatically.
template <typename T, typename... Ts> constexpr Vec<std::decay_t<T>, (1 + sizeof...(Ts))> V(T&& t, Ts&&... ts) {
  return Vec<std::decay_t<T>, 1 + sizeof...(ts)>(std::move(t), std::forward<Ts>(ts)...);
}

// Construct a zero-length Vec.
template <typename T> constexpr Vec<T, 0> V() { return Vec<T, 0>(); }

// Construct an Vec with two identical elements, e.g. twice(v) == V(v, v).
template <typename T> constexpr Vec2<T> twice(const T& v) { return {v, v}; }

// Construct an Vec with three identical elements, e.g. thrice(v) == V(v, v, v).
template <typename T> constexpr Vec3<T> thrice(const T& v) { return {v, v, v}; }

// Construct an Vec with identical elements, e.g. ntimes<4>(.5f) == V(.5f, .5f, .5f, .5f).
template <int n, typename T> constexpr Vec<T, n> ntimes(const T& v) { return Vec<T, n>::all(v); }

// Given container c, evaluate func() on each element (possibly changing the element type) and return new container.
template <typename T, int n, typename Func> auto map(const Vec<T, n>& c, Func func) {
  Vec<decltype(func(std::declval<T>())), n> nc;
  for_int(i, n) nc[i] = func(c[i]);
  return nc;
}

// Range of coordinates: Vec<int, D>: 0 <= [0] < uU[0], 0 <= [1] < uU[1], ..., 0 <= [D - 1] < uU[D - 1].
//  e.g.: for (const auto& p : range(grid.dims())) grid[p] = func(p);
template <int D> details::Vec_range<D> range(const Vec<int, D>& uU);

// Range of coordinates: Vec<int, D>: uL[0] <= [0] < uU[0], ..., uL[D - 1] <= [D - 1] < uU[D - 1].
template <int D> details::VecL_range<D> range(const Vec<int, D>& uL, const Vec<int, D>& uU);

namespace details {
template <typename T, typename... A> struct concat_n { static constexpr int value = T::Num + concat_n<A...>::value; };
template <typename T> struct concat_n<T> { static constexpr int value = T::Num; };
template <typename T, int n1, int n2, size_t... Is>
constexpr Vec<T, (n1 + n2)> concat_aux(const Vec<T, n1>& a1, const Vec<T, n2>& a2, std::index_sequence<Is...>) {
  return Vec<T, n1 + n2>((narrow_cast<int>(Is) < n1 ? a1[narrow_cast<int>(Is)] : a2[narrow_cast<int>(Is) - n1])...);
}
}  // namespace details

// Concatenate several Vec's to create single Vec, e.g. concat(V(1, 2), V(3), V(4, 5)) == V(1, 2, 3, 4, 5).
template <typename T, int n1, int n2, typename... A>
constexpr Vec<T, (n1 + details::concat_n<Vec<T, n2>, A...>::value)> concat(const Vec<T, n1>& a1, const Vec<T, n2>& a2,
                                                                           A... arr) {
  return concat(details::concat_aux(a1, a2, std::make_index_sequence<n1 + n2>()), arr...);
}
template <typename T, int n1> constexpr Vec<T, n1> concat(const Vec<T, n1>& a1) { return a1; }

//----------------------------------------------------------------------------

// For Numeric T:

template <typename T, int n> T dot(const Vec<T, n>& v1, const Vec<T, n>& v2) {
  T sum{};
  for_int(i, n) sum += v1[i] * v2[i];
  return sum;
}
template <typename T, int n> T mag2(const Vec<T, n>& vec) { return dot(vec, vec); }
template <typename T, int n> T mag(const Vec<T, n>& vec) { return sqrt(mag2(vec)); }
template <typename T, int n> T dist2(const Vec<T, n>& v1, const Vec<T, n>& v2) { return mag2(v1 - v2); }
template <typename T, int n> T dist(const Vec<T, n>& v1, const Vec<T, n>& v2) { return sqrt(dist2(v1, v2)); }
template <typename T, int n> Vec<T, n> normalized(Vec<T, n> vec) {
  assertx(vec.normalize());
  return vec;
}
template <typename T, int n> Vec<T, n> ok_normalized(Vec<T, n> vec) {
  vec.normalize();
  return vec;
}
template <typename T, int n> Vec<T, n> fast_normalized(const Vec<T, n>& vec) { return vec / mag(vec); }
template <typename T, int n> bool is_unit(const Vec<T, n>& vec, float tolerance = 1e-4f) {
  return abs(mag2(vec) - 1.f) <= tolerance;
}
template <typename T> constexpr Vec<T, 3> cross(const Vec<T, 3>& v1, const Vec<T, 3>& v2) {
  return Vec<T, 3>(v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]);
}
template <typename T> constexpr T cross(const Vec<T, 2>& v1, const Vec<T, 2>& v2) {
  return v1[0] * v2[1] - v2[0] * v1[1];
}
// More robust than acos(dot()) for small angles!
template <typename T> T angle_between_unit_vectors(const Vec<T, 3>& v1, const Vec<T, 3>& v2) {
  ASSERTXX(is_unit(v1) && is_unit(v2));
  T vdot = dot(v1, v2);
  const float thresh = 0.9475f;  // Empirically from Python determine_crossover_for_acos_angle_approximation().
  if (vdot > +thresh) {
    return std::asin(mag(cross(v1, v2)));
  } else if (vdot < -thresh) {
    return T(D_TAU / 2) - std::asin(mag(cross(v1, v2)));
  } else {
    return std::acos(vdot);
  }
}
template <typename T> T angle_between_unit_vectors(const Vec<T, 2>& v1, const Vec<T, 2>& v2) {
  ASSERTXX(is_unit(v1) && is_unit(v2));
  T vdot = dot(v1, v2);
  const float thresh = 0.9475f;
  if (vdot > +thresh) {
    return std::asin(cross(v1, v2));
  } else if (vdot < -thresh) {
    return T(D_TAU / 2) - std::asin(cross(v1, v2));
  } else {
    return std::acos(vdot);
  }
}

//----------------------------------------------------------------------------

namespace details {

template <typename T, int n> struct Vec_base {  // allocates a member variable only if n > 0.
  Vec_base() = default;
  template <typename... Args> constexpr Vec_base(void*, Args&&... args) noexcept : _a{std::forward<Args>(args)...} {
    static_assert(sizeof...(args) == n, "#args");
  }
  T _a[n];
  T* a() noexcept { return &_a[0]; }
  const T* a() const noexcept { return &_a[0]; }
  T& operator[](int i) { return _a[i]; }
  constexpr const T& operator[](int i) const { return _a[i]; }  // operator[] needed for constexpr
};

template <typename T> struct Vec_base<T, 0> {
  Vec_base() = default;
  template <typename... Args> Vec_base(void*, Args&&... args) noexcept = delete;
  T* a() noexcept { return nullptr; }
  const T* a() const noexcept { return nullptr; }
  T& operator[](int) { return *implicit_cast<T*>(nullptr); }
  constexpr const T& operator[](int) const { return *implicit_cast<T*>(nullptr); }
  // No member variable at all.
};

}  // namespace details

//----------------------------------------------------------------------------

namespace details {

// Iterator for traversing coordinates: 0 <= [0] < uU[0], 0 <= [1] < uU[1], ..., 0 <= [D - 1] < uU[D - 1].
template <int D> class Vec_iterator {
  using type = Vec_iterator<D>;

 public:
  using iterator_category = std::forward_iterator_tag;
  using value_type = Vec<int, D>;
  using difference_type = void;
  using pointer = value_type*;
  using reference = value_type&;
  Vec_iterator(const Vec<int, D>& u, const Vec<int, D>& uU) : _u(u), _uU(uU) {}
  Vec_iterator(const type& iter) = default;
  bool operator!=(const type& rhs) const {
    dummy_use(rhs);
    ASSERTXX(rhs._uU == _uU);
    ASSERTXX(rhs._u[0] == _uU[0]);
    return _u[0] < _uU[0];  // quick check against usual end()
  }
  const Vec<int, D>& operator*() const { return (ASSERTX(_u[0] < _uU[0]), _u); }
  type& operator++() {
    static_assert(D > 0, "");
    ASSERTXX(_u[0] < _uU[0]);
    if (D == 1) {
      _u[0]++;
      return *this;
    } else if (D == 2) {  // else VC12 does not unroll this tiny loop
      if (++_u[1] < _uU[1]) return *this;
      _u[1] = 0;
      ++_u[0];
      return *this;
    } else {
      int c = D - 1;  // here to avoid warning about loop condition in VC14 code analysis
      for (; c > 0; --c) {
        if (++_u[c] < _uU[c]) return *this;
        _u[c] = 0;
      }
      _u[0]++;
      return *this;
    }
  }

 private:
  Vec<int, D> _u, _uU;
};

// Range of coordinates 0 <= [0] < uU[0], 0 <= [1] < uU[1], ..., 0 <= [D - 1] < uU[D - 1].
template <int D> class Vec_range {
 public:
  Vec_range(const Vec<int, D>& uU) : _uU(uU) {}
  Vec_iterator<D> begin() const { return Vec_iterator<D>(ntimes<D>(0), _uU); }
  Vec_iterator<D> end() const { return Vec_iterator<D>(_uU, _uU); }

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
  using pointer = value_type*;
  using reference = value_type&;
  VecL_iterator(const Vec<int, D>& uL, const Vec<int, D>& uU) : _u(uL), _uL(uL), _uU(uU) {}
  VecL_iterator(const type& iter) = default;
  bool operator!=(const type& rhs) const {
    ASSERTX(rhs._uU == _uU);
    ASSERTX(rhs._u[0] == _uU[0]);
    return _u[0] < _uU[0];  // quick check against usual end()
  }
  const Vec<int, D>& operator*() const { return (ASSERTX(_u[0] < _uU[0]), _u); }
  type& operator++() {
    ASSERTX(_u[0] < _uU[0]);
    for (int c = D - 1; c > 0; --c) {
      _u[c]++;
      if (_u[c] < _uU[c]) return *this;
      _u[c] = _uL[c];
    }
    _u[0]++;
    return *this;
  }

 private:
  Vec<int, D> _u, _uL, _uU;
};

// Range of coordinates uL[0] <= [0] < uU[0], ..., uL[D - 1] <= [D - 1] < uU[D - 1].
template <int D> class VecL_range {
 public:
  VecL_range(const Vec<int, D>& uL, const Vec<int, D>& uU) : _uL(uL), _uU(uU) {}
  VecL_iterator<D> begin() const { return VecL_iterator<D>(_uL, _uU); }
  VecL_iterator<D> end() const { return VecL_iterator<D>(_uU, _uU); }

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

// Backwards compatibility; deprecated.
template <int D> details::Vec_range<D> coords(const Vec<int, D>& uU) { return range(uU); }
// Backwards compatibility; deprecated.
template <int D> details::VecL_range<D> coordsL(const Vec<int, D>& uL, const Vec<int, D>& uU) { return range(uL, uU); }

//----------------------------------------------------------------------------

template <typename T, int n> bool Vec<T, n>::in_range(const Vec<T, n>& uL, const Vec<T, n>& uU) const {
  for_int(c, n) {
    if ((*this)[c] < uL[c] || (*this)[c] >= uU[c]) return false;
  }
  return true;
}

template <typename T, int n> std::ostream& operator<<(std::ostream& os, const Vec<T, n>& ar) {
  if (has_ostream_eol<T>()) {
    os << "Vec<" << type_name<T>() << "," << n << "> {\n";
    for_int(i, n) os << "  " << ar[i];
    return os << "}\n";
  } else {
    os << "[";
    for_int(i, n) os << (i ? ", " : "") << ar[i];
    return os << "]";
  }
}
// Unlike would-be HH_DECLARE_OSTREAM_EOL(Vec<T, n>), it considers has_ostream_eol<T>.
template <typename T, int n> struct has_ostream_eol_aux<Vec<T, n>> {
  static constexpr bool value() { return has_ostream_eol<T>(); }
};

//----------------------------------------------------------------------------

// These Vec operations may be more efficient than similar ArrayView operations because
//  (1) n is known, and
//  (2) there is no heap allocation.

#if 0
// We could define the following:
template <typename T, typename T2, int n, typename RT = Vec<std::common_type_t<T, T2>>>
Vec<RT, n> operator+(const Vec<T, n>& a1, const Vec<T2, n>& a2) {
  Vec<RT, n> ar;
  for_int(i, n) ar[i] = a1[i] + a2[i];
  return ar;
}
//  (Stroustrup book had dubious value_type:  =Vec<Common_type<Value_type<T>, Value_type<T2>>, n> )
#endif

// Set of functions common to Vec.h, SGrid.h, Array.h, Grid.h
// Note that RangeOp.h functions are valid here: mag2(), mag(), dist2(), dist(), dot(), is_zero(), compare().
// See also floor(), ceil(), abs() generalized to Vec<> in MathOp.h
#define TT template <typename T, int n>
#define G Vec<T, n>
#define F for_int(i, n)
// clang-format off

TT G operator+(const G& g1, const G& g2) { G g; F { g[i] = g1[i] + g2[i]; } return g; }
TT G operator-(const G& g1, const G& g2) { G g; F { g[i] = g1[i] - g2[i]; } return g; }
TT G operator*(const G& g1, const G& g2) { G g; F { g[i] = g1[i] * g2[i]; } return g; }
TT G operator/(const G& g1, const G& g2) { G g; F { g[i] = g1[i] / g2[i]; } return g; }
TT G operator%(const G& g1, const G& g2) { G g; F { g[i] = g1[i] % g2[i]; } return g; }

TT G operator+(const G& g1, T v) { G g; F { g[i] = g1[i] + v; } return g; }
TT G operator-(const G& g1, T v) { G g; F { g[i] = g1[i] - v; } return g; }
TT G operator*(const G& g1, T v) { G g; F { g[i] = g1[i] * v; } return g; }
TT G operator/(const G& g1, T v) { G g; F { g[i] = g1[i] / v; } return g; }
TT G operator%(const G& g1, T v) { G g; F { g[i] = g1[i] % v; } return g; }

TT G operator+(T v, const G& g1) { G g; F { g[i] = v + g1[i]; } return g; }
TT G operator-(T v, const G& g1) { G g; F { g[i] = v - g1[i]; } return g; }
TT G operator*(T v, const G& g1) { G g; F { g[i] = v * g1[i]; } return g; }
TT G operator/(T v, const G& g1) { G g; F { g[i] = v / g1[i]; } return g; }
TT G operator%(T v, const G& g1) { G g; F { g[i] = v % g1[i]; } return g; }

TT G& operator+=(G& g1, const G& g2) { F { g1[i] += g2[i]; } return g1; }
TT G& operator-=(G& g1, const G& g2) { F { g1[i] -= g2[i]; } return g1; }
TT G& operator*=(G& g1, const G& g2) { F { g1[i] *= g2[i]; } return g1; }
TT G& operator/=(G& g1, const G& g2) { F { g1[i] /= g2[i]; } return g1; }
TT G& operator%=(G& g1, const G& g2) { F { g1[i] %= g2[i]; } return g1; }

TT G& operator+=(G& g1, const T& v) { F { g1[i] += v; } return g1; }
TT G& operator-=(G& g1, const T& v) { F { g1[i] -= v; } return g1; }
TT G& operator*=(G& g1, const T& v) { F { g1[i] *= v; } return g1; }
TT G& operator/=(G& g1, const T& v) { F { g1[i] /= v; } return g1; }
TT G& operator%=(G& g1, const T& v) { F { g1[i] %= v; } return g1; }

TT G operator-(const G& g1) { G g; F { g[i] = -g1[i]; } return g; }

TT G min(const G& g1, const G& g2) { G g; F { g[i] = min(g1[i], g2[i]); } return g; }
TT G max(const G& g1, const G& g2) { G g; F { g[i] = max(g1[i], g2[i]); } return g; }

TT G interp(const G& g1, const G& g2, float f1 = 0.5f) {
  G g; F { g[i] = f1 * g1[i] + (1.f - f1) * g2[i]; } return g;
}
TT G interp(const G& g1, const G& g2, const G& g3, float f1 = 1.f / 3.f, float f2 = 1.f / 3.f) {
  G g; F { g[i] = f1 * g1[i] + f2 * g2[i] + (1.f - f1 - f2) * g3[i]; } return g;
}
TT G interp(const G& g1, const G& g2, const G& g3, const Vec3<float>& bary) {
  // Vec3<float> == Bary;  may have bary[0] + bary[1] + bary[2] != 1.f
  G g; F { g[i] = bary[0] * g1[i] + bary[1] * g2[i] + bary[2] * g3[i]; } return g;
}

// clang-format on
#undef F
#undef G
#undef TT

}  // namespace hh

//----------------------------------------------------------------------------

namespace std {
template <typename T, int n> struct hash<hh::Vec<T, n>> {
  size_t operator()(const hh::Vec<T, n>& ar) const {
    size_t h = hash<T>()(ar[0]);
    for_intL(i, 1, n) h = h * 2039 + hash<T>()(ar[i]);
    return h;
  }
};
}  // namespace std

#endif  // MESH_PROCESSING_LIBHH_VEC_H_
