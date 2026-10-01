// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Vec.h"

#include <type_traits>
#include <vector>

#include "libHh/Advanced.h"
#include "libHh/Array.h"
#include "libHh/RangeOp.h"
#include "libHh/Set.h"
using namespace hh;

namespace {

template <typename T, int N, size_t... Is>
constexpr Vec<T, (N - 1)> V_rest_aux(const Vec<T, N>& u, std::index_sequence<Is...> /*unused*/) {
  return Vec<T, N - 1>(u[Is + 1]...);
}

template <typename T, int N> constexpr Vec<T, (N - 1)> V_rest(const Vec<T, N>& u) {
  return V_rest_aux(u, std::make_index_sequence<N - 1>());
}

template <int n, typename T, int N, size_t... Is>
constexpr Vec<T, n> V_segment_aux(const Vec<T, N>& u, int i, std::index_sequence<Is...> /*unused*/) {
  return Vec<T, n>(u[Is + i]...);
}

template <int n, typename T, int N> constexpr Vec<T, n> V_segment(const Vec<T, N>& u, int i) {
  return V_segment_aux<n, T, N>(u, i, std::make_index_sequence<n>());
}

template <int N> constexpr size_t V_dot_slow(const Vec<int, N>& u1, const Vec<int, N>& u2) {
  if constexpr (N == 1)
    return u1[0] * u2[0];
  else
    return u1[0] * u2[0] + V_dot_slow(V_rest(u1), V_rest(u2));
}

template <typename T> constexpr T list_sum(const T& t0) { return t0; }
template <typename T, typename... U> constexpr T list_sum(const T& t0, U&&... ts) { return t0 + list_sum(ts...); }

template <typename T, int N, size_t... Is>
constexpr T V_dot_aux(const Vec<T, N>& u1, const Vec<T, N>& u2, std::index_sequence<Is...> /*unused*/) {
  return list_sum(u1[Is] * u2[Is]...);
}

template <typename T, int N> constexpr T V_dot(const Vec<T, N>& u1, const Vec<T, N>& u2) {
  return V_dot_aux(u1, u2, std::make_index_sequence<N>());
}

namespace details {

template <int N, size_t... Is> constexpr Vec<int, N> V_iota_aux(std::index_sequence<Is...> /*unused*/) {
  return Vec<int, N>(int(Is)...);
}

template <int N, size_t... Is> constexpr Vec<int, N> V_rev_iota_aux(std::index_sequence<Is...> /*unused*/) {
  return Vec<int, N>(int(N - 1 - Is)...);
}

}  // namespace details

template <int N> constexpr Vec<int, N> V_iota() { return details::V_iota_aux<N>(std::make_index_sequence<N>()); }

template <int N> constexpr Vec<int, N> V_rev_iota() {
  return details::V_rev_iota_aux<N>(std::make_index_sequence<N>());
}

}  // namespace

int main() {
  {
    Vec3<int> a1;
    fill(a1, 1);
    Vec3<int> a2{2, 1, 3};
    SHOW(a1);
    for (const auto& u : range(a1)) SHOW(u);
    SHOW(a2);
    for (const auto& u : range(a2)) SHOW(u);
    const Vec3<int> a3{};
    assertx(is_zero(a3));
    Vec3<int> a4(1, 2, 3);
    a4 = Vec3<int>{};
    assertx(is_zero(a4));
  }
  struct S : Vec2<int> {
    // void f() const { SHOW(*this); }
  };
  struct S2 {
    int v[2];
  };
  struct S3 {
    S3() = default;
    S3(int a, int b) {
      v[0] = a;
      v[1] = b;
    }
    int v[2];
  };
  struct S4 {
    float* p;
  };
  {
    S3 dummy1, dummy2(1, 2);
    dummy_use(dummy1, dummy2);
  }
  if (1) {
    {
      constexpr Vec<int, 5> ar1 = concat(V(1), V(2), V(3, 4, 5));
      SHOW(ar1);
    }
    {
      constexpr Vec<int, 5> ar2 = concat(V(1, 2), V(3, 4, 5));
      SHOW(ar2);
    }
    {
      constexpr Vec<int, 5> ar3 = concat(V(1, 2), V(3, 4), V(5));
      SHOW(ar3);
    }
    {
      constexpr Vec<int, 5> ar4 = concat(V(1, 2, 3, 4, 5));
      SHOW(ar4);
    }
  }
  if (1) {
    {
      constexpr Vec<int, 2> ar = V(4, 5);
      constexpr int t5 = ar[1];
      SHOW(t5);
    }
    {
      constexpr Vec<int, 3> rest654 = V_rest(V(7, 6, 5, 4));
      SHOW(rest654);
    }
    {
      constexpr Vec<int, 2> segment54 = V_segment<2>(V(7, 6, 5, 4, 3), 2);
      SHOW(segment54);
    }
    {
      constexpr size_t dotslow = V_dot_slow(V(7, 5), V(3, 1));
      SHOW(dotslow);
    }
    {
      constexpr size_t dot = V_dot(V(7, 5), V(3, 1));
      SHOW(dot);
    }
    {
      constexpr Vec<int, 5> iota5 = V_iota<5>();
      SHOW(iota5);
    }
    {
      constexpr Vec<int, 5> reviota5 = V_rev_iota<5>();
      SHOW(reviota5);
    }
  }
  {
    const Vec<char, 2> magic{'B', 'M'};
    static_assert(sizeof(magic) == 2);
    const Vec<uchar, 4> buf2{uchar{0}, uchar{0}, uchar{0}, uchar{0}};
    static_assert(sizeof(buf2) == 4);
  }
  {
    Vec2<float> p(1.f, 2.f), q(8.f, 7.f), r;
    SHOW(p);
    SHOW(q);
    SHOW(p - q);
    SHOW(p + q);
    SHOW(dot(p, q));
    SHOW(p * q);
    SHOW(mag(p));
    SHOW(2.f * p + 3.f * q);
    r = p + q;
    SHOW(r);
    {
      ranges::swap(p, q);
      SHOW(p);
      SHOW(q);
    }
  }
  {
    Vec<int, 3> a1;
    fill(a1, 1);
    Vec<int, 3> a2 = {2, 1, 3};
    SHOW(product(a1), sum(a1));
    SHOW(product(a2), sum(a2));
    SHOW(a1 == ntimes<3>(1));
    SHOW(a1 == ntimes<3>(2));
    SHOW(a2 == ntimes<3>(1));
    SHOW(a2.rev());
    SHOW(a2[2]);
  }
  {
    // Can a class derived from Vec be automatically converted to a CArrayView argument? yes.
    const auto func = [](CArrayView<int> a) {
      assertx(a.num() == 2);
      SHOW(a[1]);
    };
    S s;
    s[0] = 11;
    s[1] = 12;
    func(s);
  }
  {
    Vec3<int> a1(3, 4, 5);
    SHOW(a1);
#if 0
    // The expression a1 / 3.f compiles (with -Wconversion warnings), but its type is Vec3<int>, with each element
    // truncated, so this initialization fails.
    Vec3<float> a2 = a1 / 3.f;
#endif
    SHOW(a1.cast<float>() / 3.f);  // The correct form.
  }
  {
    Vec2<Vec2<int>> pp{V(3, 4), Vec2<int>{5, 6}};  // Test both ways.
    SHOW(pp);
    SHOW((pp + V(10, 20)));
    SHOW((Vec2<int>(10, 20) - pp));
    SHOW(-pp);
  }
  {
    static_assert(std::is_standard_layout_v<Vec3<int>> == true);
    static_assert(std::is_trivially_default_constructible_v<Vec3<int>> == true);
    static_assert(std::is_trivially_copyable_v<Vec3<int>> == true);
    static_assert(std::is_trivially_copyable_v<S> == true);
    static_assert(std::is_trivially_copyable_v<S3> == true);
    static_assert(std::is_trivially_copyable_v<S4> == true);
    static_assert(std::is_trivially_default_constructible_v<Vec3<int>> == true);
    static_assert(std::is_trivially_default_constructible_v<S> == true);
    static_assert(std::is_trivially_default_constructible_v<S3> == true);
    static_assert(std::is_standard_layout_v<Vec3<int>> == true);
    static_assert(std::is_standard_layout_v<S> == true);
    static_assert(std::is_standard_layout_v<S3> == true);
    static_assert(std::is_standard_layout_v<S2> == true);
    static_assert(std::is_trivially_default_constructible_v<S2>);
    static_assert(std::is_trivially_copyable_v<S2>);
    static_assert(std::is_standard_layout_v<S4> == true);
    static_assert(std::is_trivially_default_constructible_v<S4>);
    static_assert(std::is_trivially_copyable_v<S4>);
  }
  {
    Set<size_t> set;
    for_int(i, 100) for_int(j, 100) {
      const size_t h = my_hash(V(i, j));
      // SHOW(i, j, h);
      assertx(set.add(h));  // All 10'000 are unique.
    }
  }
  {
    constexpr auto ar1 = V(4, 5, 6);
    constexpr int v5 = ar1[1];
    SHOW(v5);
    constexpr auto triple6a = Vec3<float>::all(6);
    SHOW(triple6a, type_name(triple6a));
    constexpr auto triple6b = ntimes<3>(6);
    SHOW(triple6b, type_name(triple6b));
    constexpr auto triple6c = ntimes<3>(6.f);
    SHOW(triple6c, type_name(triple6c));
    constexpr auto triple7 = thrice(7);
    SHOW(triple7);
    constexpr auto ntimes7 = ntimes<3>(7);
    SHOW(ntimes7);
    const auto vonly3is2 = ntimes<5>(0).with(3, 2);
    SHOW(vonly3is2);
  }
  {
    Vec<int, 3> ar(1, 2, 3);
    ar = Vec<int, 3>{};
    SHOW(ar);
  }
  {
    auto ar = V(1, 2, 3, 4, 5);
    SHOW(ar.segment<3>(2));
    SHOW(ar.head<3>());
    SHOW(ar.tail<3>());
  }
  {
    const auto ar0 = Vec<int, 5>::create([](int i) { return i * 5 + 3; });
    SHOW(ar0);
    auto ar1 = V(1, 2, 3, 4, 5);
    const auto ar2 = transformed(ar1, [](int e) { return e * 10; });
    SHOW(ar2);
  }
  {
    struct P {
      P() { SHOW("P::P()"); }
      ~P() { SHOW("P::~P()"); }
      P(const P& /*unused*/) { SHOW("P::P(const P&)"); }
      P(P&& /*unused*/) noexcept { SHOW("P::P(P&&)"); }
      P& operator=(const P& /*unused*/) {
        SHOW("P::operator=(const P&)");
        return *this;
      }
      // P& operator=(P&&) { SHOW("P::operator=(P&&)"); return *this; }
    };
    static_assert(std::is_trivially_default_constructible_v<P> == false);
    static_assert(std::is_trivially_default_constructible_v<Vec1<P>> == false);
    static_assert(std::is_trivially_copyable_v<P> == false);
    static_assert(std::is_trivially_copyable_v<Vec1<P>> == false);
    Vec1<P> s1;
    const Vec1<P> s2 = s1;
    const Vec1<P> s3 = std::move(s1);
  }
  {
    using SU2 = Vec2<unique_ptr<int>>;
    SU2 s1 = V(make_unique<int>(1), make_unique<int>(2));
    assertx(*s1[0] == 1);
    assertx(*s1[1] == 2);
    SU2 s2 = std::move(s1);
    assertx(*s2[0] == 1);
    assertx(*s2[1] == 2);
    assertx(!s1[0]);  // NOLINT(bugprone-use-after-move, hicpp-invalid-access-moved, clang-analyzer-cplusplus.Move)
    assertx(!s1[1]);
  }
  {
    Vec<int, 0> v0;
    dummy_use(v0);
    SHOW(v0);
    SHOW(V<int>());
  }
  {
    Vec3 v3(1, 2, 3);
    SHOW(v3);
  }
  {
    Vec v4(1, 2, 3, 4);
    SHOW(v4);
  }
  {
    struct D0 : Vec<int, 0> {
      int a;
    };
    struct SG0 : Vec<int, 0> {};
    struct D0b : SG0 {
      int a;
    };  // Transitive, as for SGrid<T, 0, ...>.
    struct Du0 : Vec<uint8_t, 0> {
      uint8_t a;
    };  // Sharpest: a lost byte has no padding to hide in.
    static_assert(sizeof(D0) == sizeof(int));
    static_assert(sizeof(D0b) == sizeof(int));
    static_assert(sizeof(Du0) == 1);
    static_assert(sizeof(Vec<float, 3>) == 3 * sizeof(float));
    static_assert(std::is_trivially_copyable_v<Vec<float, 3>>);
    static_assert(std::is_standard_layout_v<Vec<float, 3>>);
  }
  {
    // The coordinate ranges are proper C++20 views over which the std::ranges adaptors and algorithms compose.
    // The iterators are input, not forward: operator*() returns a reference to the coordinate held within the
    // iterator, so the reference is invalidated by the next operator++(); see the discussion in Vec.h.
    using Iter2 = ranges::iterator_t<decltype(range(Vec2<int>{}))>;
    static_assert(std::input_iterator<Iter2> && !std::forward_iterator<Iter2>);
    // The legacy category must be input too, else std::iterator_traits would deduce forward_iterator_tag from
    // the lvalue-reference return type and the std:: algorithms would assume the multi-pass guarantee.
    static_assert(std::is_same_v<std::iterator_traits<Iter2>::iterator_category, std::input_iterator_tag>);
    static_assert(std::sentinel_for<std::default_sentinel_t, Iter2>);
    static_assert(ranges::input_range<decltype(range(Vec3<int>{}))>);
    static_assert(!ranges::forward_range<decltype(range(Vec3<int>{}))>);
    static_assert(ranges::view<decltype(range(Vec3<int>{}))>);
    static_assert(ranges::viewable_range<decltype(range(Vec3<int>{}))>);
    static_assert(ranges::borrowed_range<decltype(range(Vec3<int>{}))>);
    static_assert(ranges::sized_range<decltype(range(Vec3<int>{}))>);
    static_assert(ranges::input_range<decltype(range(Vec3<int>{}, Vec3<int>{}))>);
    static_assert(!ranges::forward_range<decltype(range(Vec3<int>{}, Vec3<int>{}))>);
    static_assert(ranges::view<decltype(range(Vec3<int>{}, Vec3<int>{}))>);
    static_assert(ranges::borrowed_range<decltype(range(Vec3<int>{}, Vec3<int>{}))>);
    static_assert(ranges::sized_range<decltype(range(Vec3<int>{}, Vec3<int>{}))>);
    static_assert(std::is_same_v<ranges::range_value_t<decltype(range(Vec2<int>{}))>, Vec2<int>>);
    static_assert(std::is_same_v<ranges::range_reference_t<decltype(range(Vec2<int>{}))>, const Vec2<int>&>);
    static_assert(std::is_trivially_copyable_v<decltype(range(Vec2<int>{}))>);
    // The lower bound occupies no space when it is known to be zero.
    static_assert(sizeof(ranges::iterator_t<decltype(range(Vec3<int>{}))>) == 2 * sizeof(Vec3<int>));
    static_assert(sizeof(ranges::iterator_t<decltype(range(Vec3<int>{}, Vec3<int>{}))>) == 3 * sizeof(Vec3<int>));
    // The ranges are usable in constant expressions, including the collapse of any empty range.
    static_assert(range(V(2, 3)).size() == 6);
    static_assert(range(V(1, 2), V(4, 7)).size() == 15);
    static_assert(range(V(2, 0, 3)).size() == 0 && range(V(2, 0, 3)).empty());
    static_assert(range(V(1, 2), V(4, 2)).size() == 0 && range(V(1, 2), V(4, 2)).empty());
    static_assert(!range(V(2, 3)).empty());
    // Being an input_range does not lose size(): view_interface::empty() and operator bool require
    // sized_range || forward_range (LWG 3715), so both survive, and ranges::distance() stays O(1).  Of the
    // view_interface members, only front() is forfeited (back() and operator[] were never available anyway).
    static_assert(bool(range(V(2, 3))) && !bool(range(V(3, 0))));
    static_assert(bool(range(V(1, 2), V(4, 7))) && !bool(range(V(1, 2), V(4, 2))));
  }
  {
    for (const auto& u : range(V(1, 2), V(3, 4))) SHOW(u);
    for (const auto& u : range(V(2, 1, 2), V(4, 2, 3))) SHOW(u);
    SHOW(range(V(2, 3)).size(), range(V(1, 2), V(4, 7)).size());
    // An empty extent in any dimension makes the whole range empty.
    for (const auto& u : range(V(3, 0))) SHOW(u);
    for (const auto& u : range(V(1, 2), V(4, 2))) SHOW(u);
    SHOW(range(V(3, 0)).empty(), range(V(1, 2), V(4, 2)).empty());
    SHOW(ranges::distance(range(V(3, 4))), ranges::distance(range(V(1, 1), V(3, 4))));
  }
  {
    // Although single-pass, the iterators still feed the std::ranges algorithms and adaptors.
    SHOW(*ranges::find(range(V(3, 4)), V(1, 2)));
    SHOW(contains(range(V(3, 4)), V(1, 3)));
    SHOW(contains(range(V(3, 4)), V(1, 4)));
    SHOW(ranges::count_if(range(V(3, 4)), [](const Vec2<int>& u) { return u[0] == u[1]; }));
    ranges::for_each(range(V(2, 2)), [](const Vec2<int>& u) { SHOW(u); });
    const auto is_diagonal = [](const Vec2<int>& u) { return u[0] == u[1]; };
    SHOW(range(V(1, 1), V(4, 4)) | views::filter(is_diagonal) | ranges::to<Array<Vec2<int>>>());
    SHOW(Array(range(V(2, 3)) | views::transform([](const Vec2<int>& u) { return u[0] * 10 + u[1]; })));
    SHOW(Array(range(V(3, 3)) | views::drop(4) | views::take(2)));
    // Because the ranges are borrowed_range, an iterator into a temporary range stays valid.
    const auto iter = ranges::find(range(V(3, 4)), V(2, 1));
    SHOW(*iter);
  }
  {
    // The reference returned by operator*() aliases the coordinate held in the iterator, so it follows the
    // increments; a caller that must retain a coordinate across an increment has to copy it.
    auto iter2 = ranges::begin(range(V(2, 2)));
    const Vec2<int>& u_ref = *iter2;
    const Vec2<int> u_copy = *iter2;
    ++iter2;
    SHOW(u_ref, u_copy);
    // Conversions needing only the element count are unaffected because the ranges remain sized_range; Array's
    // range constructor keeps its exact-allocation path, which selects on forward_range || sized_range.
    SHOW(ranges::distance(range(V(3, 4))), range(V(3, 4)).size());
    SHOW(Array(range(V(2, 3))));
    SHOW(range(V(1, 2), V(3, 4)) | ranges::to<Array<Vec2<int>>>());
  }
  {
    // Element access, and the views of the elements.
    Vec<int, 4> v(1, 2, 3, 4);
    static_assert(Vec<int, 4>::Num == 4);
    assertx(v.num() == 4 && v.size() == 4);
    assertx(v.last() == 4 && v.ok(0) && v.ok(3) && !v.ok(4) && !v.ok(-1));
    v.last() = 40;
    static_assert(std::is_same_v<decltype(v.view()), ArrayView<int>>);
    static_assert(std::is_same_v<decltype(std::as_const(v).view()), CArrayView<int>>);
    static_assert(std::is_same_v<decltype(v.const_view()), CArrayView<int>>);
    static_assert(std::is_same_v<decltype(v.head(2)), ArrayView<int>>);
    static_assert(std::is_same_v<decltype(std::as_const(v).tail(2)), CArrayView<int>>);
    static_assert(std::is_same_v<decltype(v.head<2>()), Vec2<int>&>);
    static_assert(std::is_same_v<decltype(std::as_const(v).head<2>()), const Vec2<int>&>);
    static_assert(std::is_same_v<decltype(std::as_const(v)[0]), const int&>);
    assertx(v.head(2) == V(1, 2).view() && v.tail(1) == V(40).view());
    assertx(v.segment(1, 2) == V(2, 3).view() && v.slice(1, 3) == V(2, 3).view() && v.head(0).num() == 0);
    assertx((v.segment<1, 2>() == V(2, 3)) && v.segment<2>(2) == V(3, 40));
    assertx(&v.tail<3>()[0] == &v[1]);
    v.head<2>() = V(10, 20);  // Writes through the reinterpreted reference.
    v.segment<1>(2) = V(30);
    v.tail<1>()[0] += 1;
    assertx(v == V(10, 20, 30, 41));
    v.assign(V(5, 6, 7, 8));
    assertx(v == V(5, 6, 7, 8));
    const Vec3<int> w = v.head(3);  // Construction from a view of the same size.
    assertx(w == V(5, 6, 7));
    v.view().tail(2).assign(V(0, 0));
    assertx(v == V(5, 6, 0, 0));
    assertx(&v.vec() == &v);
  }
  {
    // The comparison is lexicographic.
    assertx(V(1, 2) < V(1, 3) && V(1, 3) < V(2, 0) && V(2, 0) > V(1, 9) && V(1, 2) <= V(1, 2));
    assertx((V(1, 2) <=> V(1, 2)) == 0 && V(1, 2) != V(2, 1));
    static_assert(V(1, 2, 3) < V(1, 2, 4));
    Array<Vec2<int>> ar{V(2, 1), V(1, 3), V(1, 2), V(0, 5)};
    sort(ar);
    SHOW(ar);
  }
  {
    // Structured bindings.
    static_assert(std::tuple_size_v<Vec3<int>> == 3);
    static_assert(std::is_same_v<std::tuple_element_t<1, Vec3<float>>, float>);
    const auto [a, b, c] = V(1, 2, 3);
    SHOW(a, b, c);
    Vec2<int> u(1, 2);
    auto& [x, y] = u;
    x = 5;
    y++;
    assertx(u == V(5, 3));
    auto [p, q] = V(make_unique<int>(7), make_unique<int>(8));  // Moves from the temporary.
    assertx(*p == 7 && *q == 8);
  }
  {
    // The functions with(), rev(), cast(), and in_range().
    const auto v = V(1, 2, 3);
    assertx(v.with(0, 9) == V(9, 2, 3) && v == V(1, 2, 3));
    assertx(V(1, 2, 3).with(2, 0) == V(1, 2, 0));  // The rvalue overload.
    const auto vu = V(make_unique<int>(1), make_unique<int>(2)).with(1, make_unique<int>(3));
    assertx(*vu[0] == 1 && *vu[1] == 3);
    assertx(v.rev() == V(3, 2, 1) && V(5).rev() == V(5));
    const auto vf = v.cast<float>();
    static_assert(std::is_same_v<decltype(vf), const Vec3<float>>);
    assertx(vf == V(1.f, 2.f, 3.f));
    assertx(V(1.7f, -1.7f).cast<int>() == V(1, -1));  // The conversion truncates.
    assertx(V(1, 2).in_range(V(2, 3)) && !V(2, 2).in_range(V(2, 3)) && !V(-1, 0).in_range(V(2, 3)));
    assertx(V(1, 2).in_range(V(1, 2), V(2, 3)) && !V(0, 2).in_range(V(1, 2), V(2, 3)));
    assertx(!V(1, 3).in_range(V(1, 2), V(2, 3)));
    static_assert(V(1, 2).in_range(V(2, 3)));
  }
  {
    // The construction helpers.
    static_assert(twice(5) == V(5, 5) && thrice(6) == V(6, 6, 6) && ntimes<4>(7) == V(7, 7, 7, 7));
    static_assert(std::is_same_v<decltype(V<double>(1.f, 2.)), Vec2<double>>);  // An explicit element type.
    static_assert(std::is_same_v<decltype(V(1.f, 2.f)), Vec2<float>>);
    constexpr auto vt = to_Vec({1, 2, 3});
    static_assert(std::is_same_v<decltype(vt), const Vec3<int>> && vt == V(1, 2, 3));
    const auto vp = to_Vec({make_unique<int>(4), make_unique<int>(5)});
    assertx(*vp[1] == 5);
    static_assert(concat(V(1, 2), V<int>(), V(3)) == V(1, 2, 3));
    static_assert(Vec<int, 3>::create([](int i) { return i * i; }) == V(0, 1, 4));
    static_assert(Vec3<int>::all(2) == V(2, 2, 2));
    constexpr Vec<int, 0> v0;
    static_assert(v0.num() == 0 && v0.begin() == v0.end());
    SHOW(concat(V(1, 2), V(3)), type_name<decltype(concat(V(1, 2), V(3)))>());
  }
  {
    // Element-wise and scalar arithmetic.
    const auto a = V(6, 7, 8), b = V(1, 2, 3);
    assertx(a + b == V(7, 9, 11) && a - b == V(5, 5, 5) && a * b == V(6, 14, 24));
    assertx(a / b == V(6, 3, 2) && a % b == V(0, 1, 2) && -b == V(-1, -2, -3));
    assertx(a + 1 == V(7, 8, 9) && 10 - a == V(4, 3, 2) && 2 * a == V(12, 14, 16));
    assertx(a / 2 == V(3, 3, 4) && 24 / a == V(4, 3, 3) && a % 3 == V(0, 1, 2));
    Vec3<int> c = a;
    c += b, c -= 1, c *= V(1, 2, 3), c /= 2, c %= 5;
    assertx(c == V(3, 3, 0));  // (((6, 8, 10) * (1, 2, 3)) / 2) % 5.
    c += 2, c *= 3, c -= V(0, 1, 2), c /= 3;
    assertx(c == V(5, 4, 1));
    assertx(min(a, V(7, 7, 7)) == V(6, 7, 7) && max(a, V(7, 7, 7)) == V(7, 7, 8));
    assertx(clamp(V(-5, 3, 12), 0, 10) == V(0, 3, 10));
    assertx(dot(a, b) == 44 && mag2(b) == 14 && dist2(a, b) == 75);
    static_assert(dot(V(1, 2), V(3, 4)) == 11);
  }
  {
    // Floating-point functions, with arguments that give exact results.
    assertx(mag(V(3.f, 4.f)) == 5.f && dist(V(1.f, 1.f), V(4.f, 5.f)) == 5.f && mag(V(0., 2.)) == 2.);
    const Vec2<float> n = normalized(V(3.f, 4.f));
    assertx(std::abs(n[0] - .6f) < 1e-6f && std::abs(n[1] - .8f) < 1e-6f && is_unit(n));
    assertx(!is_unit(V(1.f, 1.f)) && is_unit(V(0.f, 0.f, 1.f)));
    const Vec2<float> nf = fast_normalized(V(0.f, 2.f));
    assertx(nf == V(0.f, 1.f));
    Vec2<float> z{};
    assertx(!z.normalize() && z == V(0.f, 0.f));  // A zero vector cannot be normalized and is unchanged.
    assertx(ok_normalized(V(0.f, 0.f)) == V(0.f, 0.f));
    assertx(snap_coordinates(V(1e-7f, .9999999f, -1.0000001f, .5f)) == V(0.f, 1.f, -1.f, .5f));
    assertx(snap_coordinate(-2e-7) == 0. && snap_coordinate(.1) == .1);
    assertx(interp(V(0.f, 4.f), V(8.f, 12.f)) == V(4.f, 8.f));
    assertx(interp(V(0.f, 4.f), V(8.f, 12.f), .25f) == V(6.f, 10.f));
    assertx(interp(V(0.f, 4.f), V(8.f, 12.f), V(16.f, 20.f), .5f, .25f) == V(6.f, 10.f));
    assertx(interp(V(0.f, 4.f), V(8.f, 12.f), V(16.f, 20.f), V(.5f, .25f, .25f)) == V(6.f, 10.f));
    assertx(interp(V(0.f, 4.f), V(8.f, 12.f), V(16.f, 20.f), V(1.f, 1.f, 0.f)) == V(8.f, 16.f));  // Unnormalized.
    assertx(interp(V(V(0.f, 4.f), V(8.f, 12.f), V(16.f, 20.f)), .5f, .25f) == V(6.f, 10.f));      // A triple.
    assertx(interp(V(0, 10), V(10, 20)) == V(5, 15));     // Integer elements, interpolated in float.
    SHOW(interp(V(0.f, 3.f), V(3.f, 6.f), V(6.f, 9.f)));  // Equal weights of one third.
  }
  {
    // A nested Vec, as an SGrid.
    using G = SGrid<int, 2, 3>;
    static_assert(std::is_same_v<G, Vec2<Vec3<int>>>);
    static_assert(vec_depth_v<int> == 0 && vec_depth_v<Vec3<int>> == 1 && vec_depth_v<G> == 2);
    static_assert(is_vec_v<G> && is_vec_v<Vec3<int>> && !is_vec_v<int> && !is_vec_v<S>);
    static_assert(std::is_same_v<sgrid_leaf_t<1, G>, Vec3<int>> && std::is_same_v<sgrid_leaf_t<2, G>, int>);
    static_assert(G::grid_dims() == V(2, 3) && G::grid_dims<1>() == V(2));
    static_assert(SGrid<int, 4, 3, 2>::grid_dims<3>() == V(4, 3, 2));
    static_assert(sizeof(G) == 6 * sizeof(int));
    G g{{1, 2, 3}, {4, 5, 6}};  // Nested brace initialization.
    assertx((g[1, 2] == 6 && g[V(1, 2)] == 6 && g[1][2] == 6));
    g[0, 1] = 20;
    assertx(g[V(0, 1)] == 20);
    g[V(0, 1)] = 2;
    SHOW(g * 10);                  // A scalar operation reaches the leaves.
    SHOW(g + V(100, 200, 300));    // A less nested Vec applies to each row.
    SHOW(interp(g, g * 3, .25f));  // The interpolation recurses to the leaves.
    int count = 0;
    for (const auto& u : range(G::grid_dims())) assertx(g[u] == ++count);
    assertx(count == 6);
  }
  {
    // Equal Vecs have equal hashes, so a Vec can be used as a key.
    assertx(std::hash<Vec2<int>>{}(V(1, 2)) == std::hash<Vec2<int>>{}(V(1, 2)));
    assertx(my_hash(V(1, 2)) != my_hash(V(2, 1)));
    Set<Vec2<int>> set;
    for_int(i, 3) for_int(j, 3) set.enter(V(i, j));
    assertx(set.num() == 9 && set.contains(V(2, 1)) && !set.contains(V(3, 0)));
  }
}

template class hh::Vec<int, 4>;
template class hh::Vec<double, 1>;
template class hh::Vec<float, 2>;
template class hh::Vec<ushort, 3>;
template class hh::Vec<unique_ptr<int>, 2>;
template class hh::Vec<void*, 3>;
