// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_VEC_H_
#define MESH_PROCESSING_LIBHH_VEC_H_

#include "libHh/Advanced.h"  // my_hash(), hash_combine()
#include "libHh/Array.h"     // ArrayView<>, CArrayView<>

namespace hh {

// Defined in Grid.h, which includes this header; only the declarations are needed here, because the return types
//  and bodies of grid_view() are required complete only at instantiation -- necessarily in a translation unit
//  that has already included Grid.h.
template <int D, typename T> class CGridView;
template <int D, typename T> class GridView;

namespace details {
template <typename T, int n> struct VecBase;
template <int D, typename T> struct SGridLeaf;
}  // namespace details

// INVARIANT: is_vec_v must never change observable behavior.  It only gates the presence of the extra grid members
//  below; it must not alter size(), num(), begin(), end(), value_type, operator<<, has_ostream_eol, or anything
//  else.  A Vec<T, n> is a range of exactly n elements of type T at every nesting level, without exception.
//  The reason is that a nested Vec is not necessarily a grid: Bbox derives from Vec2<Vec<T, dim>> and is a pair of
//  corners, Vec3<Vec2<float>> is a triangle, and Vec3<Vec3<Precision>> is three points.  The type cannot tell those
//  apart from SGrid<float, 3, 2>, so the number of dimensions D is always supplied by the caller at the use site,
//  never inferred.  Violating this invariant would silently reinterpret those types as grids.

// True only for an exact Vec specialization, never for a class (Point, Vector, Pixel) derived from Vec.
// Note that this is not what bounds the flattening depth -- details::SGridLeaf below does that, since a partial
//  specialization on Vec<T, n> does not match a derived class, so sgrid_leaf_t<3, SGrid<Point, 2, 2>> is ill-formed
//  rather than silently reinterpreting Point as three floats.
// The exactness matters instead because is_vec_v gates member and operator overloads: were it true for Point,
//  Vec<Point, 3> would acquire an initializer_list<Point> constructor that hijacks list-initialization from the
//  variadic constructor (losing its static_assert on arity), and Vec<Point, 3> * 2.f would silently become legal.
template <typename T> inline constexpr bool is_vec_v = false;
template <typename T>
concept IsVec = is_vec_v<T>;

// The element type obtained by peeling D levels of Vec nesting from T.
template <int D, typename T> using sgrid_leaf_t = typename details::SGridLeaf<D, T>::type;

// The grid view type (const or mutable) corresponding to an element pointer type Ptr; the analogue of array_view_t.
template <int D, typename Ptr>
using grid_view_t = std::conditional_t<std::is_const_v<std::remove_pointer_t<Ptr>>,
                                       CGridView<D, std::remove_cv_t<std::remove_pointer_t<Ptr>>>,
                                       GridView<D, std::remove_pointer_t<Ptr>>>;

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
  // Nested brace initialization, e.g. SGrid<int, 2, 3>{{1, 2, 3}, {4, 5, 6}}; a braced-init-list is a non-deduced
  //  context, so the variadic constructor above cannot accept one.  Enabled only for nested Vec, both to leave
  //  Vec<int, 3>{1, 2, 3} on the variadic constructor and because initializer_list wins list-initialization.
  constexpr Vec(std::initializer_list<T> l) requires IsVec<T> && Copyable<T> {
    ASSERTX(narrow_cast<int>(l.size()) == n);
    T* p = data();
    for (const T& e : l) *p++ = e;
  }
  // To allow class to be trivial, and to allow generation of implicit move constructor and assignment,
  //  it is safest to not include any copy-constructor, not even a default one.
  [[HH_GNU_PURE]] [[nodiscard]] constexpr int num() const { return n; }
  [[nodiscard]] constexpr size_t size() const { return static_cast<size_t>(n); }
  [[HH_GNU_PURE]] [[nodiscard]] constexpr auto& operator[](this auto&& self, int i) {
    return HH_CHECK_BOUNDS(i, n), as_vec(self).data()[i];
  }
  // Subscript by a coordinate, e.g. grid[V(i, j, k)]; recurses into the nested Vec.
  template <int D>
  [[nodiscard]] constexpr auto& operator[](this auto&& self, const Vec<int, D>& u) requires(D == 1 || IsVec<T>) {
    if constexpr (D == 1)
      return self[u[0]];
    else
      return self[u[0]][u.template tail<D - 1>()];
  }
  // Multidimensional subscript, e.g. frame[2, 0]; recurses into the nested Vec.
  template <std::integral... A>
  [[HH_GNU_PURE]] [[nodiscard]] constexpr auto& operator[](this auto&& self, int i, A... dd)
      requires(IsVec<T> && sizeof...(A) >= 1) {
    return self[i][dd...];
  }
  [[nodiscard]] constexpr auto& last(this auto&& self) { return self[n - 1]; }
  [[nodiscard]] constexpr bool ok(int i) const { return i >= 0 && i < n; }
  constexpr void assign(CArrayView<T> ar) requires Copyable<T> {
    ASSERTXX(ar.num() == n);
    ranges::copy(ar, data());
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
  // Grid interface, present only when the element is itself a Vec.  The number of dimensions D is always stated
  //  explicitly, because a nested Vec need not be a grid, e.g. Vec3<Vec2<float>> is a triangle.
  template <int D = 2> [[nodiscard]] static constexpr Vec<int, D> grid_dims() requires(D == 1 || IsVec<T>) {
    if constexpr (D == 1)
      return Vec<int, 1>(n);
    else
      return concat(Vec<int, 1>(n), T::template grid_dims<D - 1>());
  }
  template <int D = 2> [[nodiscard]] auto grid_view(this auto&& self) requires(D == 1 || IsVec<T>) {
    using Leaf = std::remove_reference_t<decltype(*self.data())>;  // Either const or mutable.
    using LeafPtr = std::conditional_t<std::is_const_v<Leaf>, const sgrid_leaf_t<D, type>*, sgrid_leaf_t<D, type>*>;
    return grid_view_t<D, LeafPtr>(reinterpret_cast<LeafPtr>(self.data()), grid_dims<D>());
  }
  template <int D = 2>
  [[nodiscard]] CGridView<D, sgrid_leaf_t<D, type>> const_grid_view() const requires(D == 1 || IsVec<T>) {
    return CGridView<D, sgrid_leaf_t<D, type>>(reinterpret_cast<const sgrid_leaf_t<D, type>*>(data()), grid_dims<D>());
  }
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
  // Deducing this deduces the derived class (Point, Frame, Bbox, ...), so member access on self would be
  // looked up there.  Instead, as_vec() pins self to the exact Vec base so that a member added to a derived class
  // cannot alter Vec's own behavior.
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

template <typename T, int n> inline constexpr bool is_vec_v<Vec<T, n>> = true;

namespace details {
template <typename T> struct SGridLeaf<0, T> {
  using type = T;
};
template <int D, typename T, int n> requires(D > 0) struct SGridLeaf<D, Vec<T, n>> {
  using type = typename SGridLeaf<D - 1, T>::type;
};

template <typename T, int... ds> struct SGrid_type;
template <typename T, int d0> struct SGrid_type<T, d0> {
  using type = Vec<T, d0>;
};
template <typename T, int d0, int... od> struct SGrid_type<T, d0, od...> {
  using type = Vec<typename SGrid_type<T, od...>::type, d0>;
};
template <int D, bool has_lower_bound> class Vec_range;
}  // namespace details

// Readable spelling for a nested Vec: SGrid<T, n0, n1, n2> is Vec<Vec<Vec<T, n2>, n1>, n0>.
// It is a pure alias: no behavior differs between the two spellings.
template <typename T, int... ds> using SGrid = typename details::SGrid_type<T, ds...>::type;

// Range of coordinates: Vec<int, D>: 0 <= [0] < uU[0], 0 <= [1] < uU[1], ..., 0 <= [D - 1] < uU[D - 1].
//  e.g.: for (const auto& p : range(grid.dims())) grid[p] = func(p);
template <int D> [[nodiscard]] constexpr details::Vec_range<D, false> range(const Vec<int, D>& uU);

// Range of coordinates: Vec<int, D>: uL[0] <= [0] < uU[0], ..., uL[D - 1] <= [D - 1] < uU[D - 1].
template <int D>
[[nodiscard]] constexpr details::Vec_range<D, true> range(const Vec<int, D>& uL, const Vec<int, D>& uU);

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

namespace details {

// Lower bound of a Vec_range and of its iterators.  When the bound is known to be zero, this specialization is
// empty, so that (as an empty base class) it occupies no space and lets the compiler replace the reset step in
// Vec_iterator::operator++() by a constant.
template <int D, bool has_lower_bound> struct Vec_lower_bound {
  constexpr Vec_lower_bound() = default;
  constexpr explicit Vec_lower_bound(const Vec<int, D>& uL) : _uL(uL) {}
  [[nodiscard]] constexpr int lower(int c) const { return _uL[c]; }
  [[nodiscard]] constexpr const Vec<int, D>& lower() const { return _uL; }
  Vec<int, D> _uL{};
};
template <int D> struct Vec_lower_bound<D, false> {
  [[nodiscard]] static constexpr int lower(int) { return 0; }
  [[nodiscard]] static constexpr Vec<int, D> lower() { return ntimes<D>(0); }
};

// Iterator over the coordinates uL[0] <= [0] < uU[0], ..., uL[D - 1] <= [D - 1] < uU[D - 1], in lexicographic
// order (the last coordinate varies fastest).  It stores the bounds rather than pointing to the Vec_range, so it
// remains valid after the range is destroyed, which lets Vec_range be a borrowed_range.
template <int D, bool has_lower_bound> class Vec_iterator : private Vec_lower_bound<D, has_lower_bound> {
  static_assert(D > 0);
  using type = Vec_iterator<D, has_lower_bound>;
  using base = Vec_lower_bound<D, has_lower_bound>;

 public:
  using iterator_concept = std::forward_iterator_tag;
  using value_type = Vec<int, D>;
  using difference_type = std::ptrdiff_t;
  constexpr Vec_iterator() = default;
  // Note `base(lower_bound)` invokes the implicit copy constructor of base, which is a no-op if !has_lower_bound.
  constexpr Vec_iterator(const base& lower_bound, const Vec<int, D>& uU)
      : base(lower_bound), _u(lower_bound.lower()), _uU(uU) {}
  [[nodiscard]] constexpr const Vec<int, D>& operator*() const { return ASSERTXX(_u[0] < _uU[0]), _u; }
  constexpr type& operator++() {
    ASSERTXX(_u[0] < _uU[0]);
    for (int c = D - 1; c > 0; --c) {  // The loop is unrolled because D is a compile-time constant.
      if (++_u[c] < _uU[c]) return *this;
      _u[c] = this->lower(c);
    }
    ++_u[0];
    return *this;
  }
  type operator++(int) { return postfix_increment(*this); }
  // Testing the first coordinate suffices because Vec_range empties it if any other dimension is empty.
  [[nodiscard]] constexpr bool operator==(std::default_sentinel_t) const { return _u[0] >= _uU[0]; }
  // Two iterators of the same range are equal when at the same coordinate; the end state is unique.
  [[nodiscard]] constexpr bool operator==(const type& iter) const { return _u == iter._u; }

 private:
  Vec<int, D> _u{}, _uU{};
};

// Range of the coordinates uL[0] <= [0] < uU[0], ..., uL[D - 1] <= [D - 1] < uU[D - 1], in lexicographic order.
// It is a view, so it can be composed with std::ranges adaptors, e.g. range(dims) | views::filter(func).
template <int D, bool has_lower_bound>
class Vec_range : public ranges::view_interface<Vec_range<D, has_lower_bound>>,
                  private Vec_lower_bound<D, has_lower_bound> {
  static_assert(D > 0);
  using base = Vec_lower_bound<D, has_lower_bound>;

 public:
  using iterator = Vec_iterator<D, has_lower_bound>;
  constexpr Vec_range() = default;
  constexpr explicit Vec_range(const Vec<int, D>& uU) requires(!has_lower_bound) : _uU(uU) { empty_if_any_empty(); }
  constexpr Vec_range(const Vec<int, D>& uL, const Vec<int, D>& uU) requires has_lower_bound : base(uL), _uU(uU) {
    empty_if_any_empty();
  }
  [[nodiscard]] constexpr iterator begin() const { return iterator(static_cast<const base&>(*this), _uU); }
  [[nodiscard]] constexpr std::default_sentinel_t end() const { return std::default_sentinel; }
  [[nodiscard]] constexpr size_t size() const {
    size_t product = 1;
    for_int(c, D) product *= size_t(max(0, _uU[c] - this->lower(c)));
    return product;
  }

 private:
  Vec<int, D> _uU{};
  // Because the iteration only tests the first coordinate, an empty dimension must empty the first one as well.
  constexpr void empty_if_any_empty() {
    for_intL(c, 1, D) {
      if (_uU[c] <= this->lower(c)) {
        _uU[0] = this->lower(0);
        break;
      }
    }
  }
};

}  // namespace details

template <int D> constexpr details::Vec_range<D, false> range(const Vec<int, D>& uU) {
  return details::Vec_range<D, false>(uU);
}

template <int D> constexpr details::Vec_range<D, true> range(const Vec<int, D>& uL, const Vec<int, D>& uU) {
  return details::Vec_range<D, true>(uL, uU);
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

template <typename T> inline constexpr int vec_depth_v = 0;
template <typename T, int n> inline constexpr int vec_depth_v<Vec<T, n>> = 1 + vec_depth_v<T>;

//----------------------------------------------------------------------------

// These Vec operations may be more efficient than similar ArrayView operations because
//  (1) n is known, and
//  (2) there is no heap allocation.

#if 0
// We could define the following:
#define TT2 template <typename T1, int n, typename T2>
TT2 [[nodiscard]] constexpr auto operator OP(const Vec<T1, n>& g1, const Vec<T2, n>& g2)
    -> Vec<std::common_type_t<T1, T2>, n> requires requires(T1 a, T2 b) { a OP b; } {
  Vec<std::common_type_t<T1, T2>, n> ar;
  for_int(i, n) ar[i] = g1[i] OP g2[i];
  return ar;
}
// However, `SGrid<int, 2, 2> + Vec2<int>` would then silently add to columns rather than rows!
// Instead, this constrained generalization would be safer:
TT2 [[nodiscard]] constexpr auto operator OP(const Vec<T1, n>& g1, const Vec<T2, n>& g2)
    -> Vec<std::common_type_t<T1, T2>, n>
    requires(vec_depth_v<T1> == vec_depth_v<T2> && requires(T1 a, T2 b) { a OP b; }) {}
#endif

// Set of functions common to Array.h, Grid.h, Vec.h.
// Note that RangeOp.h functions are valid here: mag2(), mag(), dist2(), dist(), dot(), is_zero(), compare().
// See also floor(), ceil(), abs() generalized to Vec<> in MathOp.h.
#define TT template <typename T, int n>
#define TTC TT [[nodiscard]] constexpr
#define TTA TT constexpr
#define G Vec<T, n>
#define F for_int(i, n)
// Scalar operations.  U is an independent parameter (rather than reusing T) for two reasons: it lets the operation
//  recurse past the immediate element type, so that SGrid<float, 4, 3> * 2.f reaches the leaves; and the
//  requires-expression keeps the overload SFINAE-friendly, so an inapplicable U fails overload resolution rather
//  than erroring inside the body.
#define TTU template <typename T, int n, typename U>
// U must be less deeply nested than G, so that only one of the two directions applies; and U must not be a class
//  merely derived from Vec (Point, Pixel, ...), because G would then be deduced as that class's Vec base and the
//  scalar form would hijack Point + Point from the element-wise form.  The test is on is_class_v rather than
//  DerivedFromVec because the latter needs U complete, and GCC then reports a changed satisfaction value when the
//  constraint is first evaluated inside U's own definition (e.g. Frame).
#define HH_VEC_LESS_NESTED (vec_depth_v<U> < vec_depth_v<G> && (IsVec<U> || !std::is_class_v<U>))
// clang-format off

#define HH_OPERATIONS(OP) \
  TTU [[nodiscard]] constexpr G operator OP(const G& g1, const U& v)       \
      requires(HH_VEC_LESS_NESTED && requires(T t, U u) { t OP u; }) {     \
    G g; F { g[i] = g1[i] OP v; } return g;                                \
  }                                                                        \
  TTU [[nodiscard]] constexpr G operator OP(const U& v, const G& g1)       \
      requires(HH_VEC_LESS_NESTED && requires(T t, U u) { u OP t; }) {     \
    G g; F { g[i] = v OP g1[i]; } return g;                                \
  }                                                                        \
  TTU constexpr G& operator OP##=(G& g1, const U& v)                       \
      requires(HH_VEC_LESS_NESTED && requires(T t, U u) { t OP##= u; }) {  \
    F { g1[i] OP##= v; } return g1;                                        \
  }                                                                        \
  TTC G operator OP(const G& g1, const G& g2) { G g; F { g[i] = g1[i] OP g2[i]; } return g; } \
  TTA G& operator OP##=(G& g1, const G& g2) { F { g1[i] OP##= g2[i]; } return g1; } \
  HH_EAT_SEMICOLON

HH_OPERATIONS(+); HH_OPERATIONS(-); HH_OPERATIONS(*); HH_OPERATIONS(/); HH_OPERATIONS(%);

TTC G operator-(const G& g1) { G g; F { g[i] = -g1[i]; } return g; }

TTC G min(const G& g1, const G& g2) { G g; F { g[i] = min(g1[i], g2[i]); } return g; }
TTC G max(const G& g1, const G& g2) { G g; F { g[i] = max(g1[i], g2[i]); } return g; }

TTU [[nodiscard]] constexpr G clamp(const G& g1, const U& vmin, const U& vmax)
    requires(HH_VEC_LESS_NESTED && requires(T t, U u) { clamp(t, u, u); }) {
  G g; F { g[i] = clamp(g1[i], vmin, vmax); } return g;
}

TTC G interp(const G& g1, const G& g2, float f1 = 0.5f) requires(!IsVec<T>) {
  G g; F { g[i] = static_cast<T>(f1 * g1[i] + (1.f - f1) * g2[i]); } return g;
}
TTC G interp(const G& g1, const G& g2, const G& g3, float f1, float f2) requires(!IsVec<T>) {
  G g; F { g[i] = static_cast<T>(f1 * g1[i] + f2 * g2[i] + (1.f - f1 - f2) * g3[i]); } return g;
}
TTC G interp(const G& g1, const G& g2, const G& g3, const Vec3<float>& bary) requires(!IsVec<T>) {
  // Vec3<float> == Bary;  may have sum(bary) != 1.f
  G g; F { g[i] = static_cast<T>(bary[0] * g1[i] + bary[1] * g2[i] + bary[2] * g3[i]); } return g;
}

// For a nested Vec, recurse rather than multiply by the float weights here, so that the leaf arithmetic is
//  float * T even when T is integral; the flat loop in the former SGrid.h did the same.
TTC G interp(const G& g1, const G& g2, float f1 = 0.5f) requires(IsVec<T>) {
  G g; F { g[i] = interp(g1[i], g2[i], f1); } return g;
}
TTC G interp(const G& g1, const G& g2, const G& g3, float f1, float f2) requires(IsVec<T>) {
  G g; F { g[i] = interp(g1[i], g2[i], g3[i], f1, f2); } return g;
}
TTC G interp(const G& g1, const G& g2, const G& g3, const Vec3<float>& bary) requires(IsVec<T>) {
  G g; F { g[i] = interp(g1[i], g2[i], g3[i], bary); } return g;
}

TTC G interp(const G& g1, const G& g2, const G& g3) { return interp(g1, g2, g3, 1.f / 3.f, 1.f / 3.f); }

// clang-format on
#undef HH_OPERATIONS
#undef HH_VEC_LESS_NESTED
#undef TTU
#undef F
#undef G
#undef TTA
#undef TTC
#undef TT

template <typename Class>
concept DerivedFromVec = std::is_base_of_v<Vec<typename Class::value_type, Class::Num>, Class>;

template <typename T> [[nodiscard]] T interp(const Vec3<T>& triple, float f1, float f2) {
  return interp(triple[0], triple[1], triple[2], f1, f2);
}
template <typename T> [[nodiscard]] T interp(const Vec3<T>& triple) {  //
  return interp(triple[0], triple[1], triple[2]);
}
// DerivedFromVec to disambiguate from "Point interp(Point, Point)".
template <DerivedFromVec SomeVec> [[nodiscard]] SomeVec interp(const Vec3<SomeVec>& triple, const Vec3<float>& bary) {
  return interp(triple[0], triple[1], triple[2], bary);
}

// Template deduction guides:
template <typename T, typename... Args> Vec(T, Args...) -> Vec<T, 1 + sizeof...(Args)>;

}  // namespace hh

//----------------------------------------------------------------------------

template <int D, bool has_lower_bound>
inline constexpr bool std::ranges::enable_borrowed_range<hh::details::Vec_range<D, has_lower_bound>> = true;

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
