// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/RangeOp.h"

#include <list>
#include <vector>

#include "libHh/Array.h"
#include "libHh/Map.h"
#include "libHh/Mesh.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

// A single-pass range over the integers [0, n) that counts how often it is advanced.
class CountingCursor {
 public:
  using Iterator = CursorIterator<CountingCursor>;
  explicit CountingCursor(int n) : _n(n) {}
  Iterator begin() noexcept { return Iterator(*this); }
  [[nodiscard]] std::default_sentinel_t end() const noexcept { return {}; }
  [[nodiscard]] bool empty() const { return _i >= _n; }
  [[nodiscard]] int num_advances() const { return _num_advances; }

 private:
  int _i{0};
  int _n;
  int _num_advances{0};
  [[nodiscard]] int current() const { return _i; }
  void advance() { _i++, _num_advances++; }
  friend CursorIterator<CountingCursor>;
};

template <typename R> Array<int> to_int_array(R&& range) {
  Array<int> result;
  for (const int e : range) result.push(e);
  return result;
}

}  // namespace

int main() {
  {
    const Array<uchar> ar1 = {4, 200, 254, 3, 7, 2};
    Array<uchar> ar2 = {4, 0, 0, 3, 7, 2};
    SHOW(mag2(ar1));
    SHOW(square(rms(ar1)) * ar1.num());
    SHOW(mag(ar2));
    SHOW(sqrt(var(ar1)));
    SHOW(dist2(ar1, ar2));
    SHOW(dist(ar1, ar2));
    SHOW(dot(ar1, ar2));
    SHOW(int(min(ar1)));
    SHOW(int(max(ar1)));
    // Clang: warning: taking the absolute value of unsigned type has no effect [-Wabsolute-value].
    // SHOW(int(max_abs_element(ar1)));
    SHOW(sum(ar1));
    SHOW(sum(ar2));
    SHOW(product(ar1));
    SHOW(product(ar2));
    SHOW(compare(ar1, ar2));
    SHOW(compare(ar2, ar1));
    SHOW(compare(ar1, ar1));
    SHOW(compare(ar2, ar2));
    SHOW(ar1 == ar2);
    SHOW(ar2 == ar1);
    SHOW(ar1 == ar1);
    SHOW(ar2 == ar2);
    SHOW(is_zero(ar1));
    SHOW(is_zero(ar1 - ar1));
    SHOW(ranges::count(ar2, 0));
    SHOW(ranges::count(ar2, 3));
    SHOW(ranges::count(ar2, 99));
    const auto func_gt5 = [](uchar uc) { return uc > 5; };
    SHOW(ranges::count_if(ar2, func_gt5));
  }
  {
    Array<float> ar1 = {2.7f, -3.3f, 5.1f, -6.2f, 0.f};
    Array<float> ar2 = {2.7f, -3.3f, 5.2f, -6.2f, 0.f};
    swap_elements(ar1, ar2);
    SHOW(ar1);
    swap_elements(ar1, ar2);
    SHOW(normalize(clone(ar1)));
    SHOW(mag(normalize(clone(ar1))));
    SHOW(sort(clone(ar1)));
    SHOW(sorted(ar1));
    SHOW(reverse(sort(clone(ar1))));
    SHOW(reverse(sorted(ar1)));
    SHOW(max_abs_element(ar1));
    SHOW(sum(ar1));
    SHOW(compare(ar1, ar2));
    SHOW(compare(ar2, ar1));
    SHOW(compare(ar1, ar1));
    SHOW(compare(ar2, ar2));
    SHOW(ar1 == ar2);
    SHOW(ar2 == ar1);
    SHOW(ar1 == ar1);
    SHOW(ar2 == ar2);
    const float tol = .3f;
    SHOW(compare(ar1, ar2, tol));
    SHOW(compare(ar2, ar1, tol));
    SHOW(compare(ar1, ar1, tol));
    SHOW(compare(ar2, ar2, tol));
  }
  {
    const int ar[] = {10, 11, 12, 13, 14, 15};  // Test C-array.
    SHOW(mean(ar));
  }
  {
    SHOW(ranges::range<Array<float>> ? 1 : 0);
    SHOW(ranges::range<std::fstream> ? 1 : 0);
    struct S {
      int _a;
    };
    SHOW(ranges::range<S>);
  }
  if (0) {
    // This should fail to compile.
    // S s; SHOW(mean(s));
  }
  {
    SHOW(type_name<mean_type_t<float>>());
    SHOW(type_name<mean_type_t<double>>());
    SHOW(type_name<mean_type_t<char>>());
    SHOW(type_name<mean_type_t<uchar>>());
    SHOW(type_name<mean_type_t<short>>());
    SHOW(type_name<mean_type_t<ushort>>());
    SHOW(type_name<mean_type_t<int>>());
    SHOW(type_name<mean_type_t<unsigned>>());
    SHOW(type_name<mean_type_t<char*>>());
  }
  {
    const Array<float> ar1 = {2.7f, -3.3f, 5.1f, -6.2f, 0.f};
    {
      auto ar = clone(ar1);
      auto& ar2 = rotate(ar, ar.begin() + 2);
      SHOW(ar);
      assertx(&ar2 == &ar);
    }
    {
      auto ar = clone(ar1);
      auto prev_left_range = ranges::rotate(ar, ar.begin() + 2);
      SHOW(ar);
      SHOW(Array(prev_left_range));
    }
  }
  {
    Array<int> ar2 = {6, 4, 2};
    for (int i : views::transform(ar2, [](int j) { return j * j; })) SHOW(i);
  }
  {
    for (int i : views::transform(std::vector<int>{10, 11}, [](int j) { return j * j; })) SHOW(i);
  }
  {
    Array<int> result;
    const Array<int> ar1{3, 4, 5};
    for (const int i : concatenate(V(1, 2), ar1)) result.push(i);
    int c_array[1] = {6};
    for (const int i : concatenate(c_array, InlinedArray<int, 2>{7, 8, 9})) result.push(i);
    for (const int i : concatenate(std::vector<int>{10, 11}, std::list<int>{12, 13})) result.push(i);
    std::vector<int> vector{14, 15};
    std::list<int> list{16, 17};
    for (const int i : concatenate(vector, list)) result.push(i);
    SHOW(result);
  }
  {
    const Array ar1{3, 4, 5, 6};
    const Array ar2(ar1 | views::filter([](int i) { return i != 3 && i != 6; }));
    SHOW(ar2);
  }
  {
    assertx(index(range(10), 3) == 3);
    assertx(index(V(3, 5, 7), 5) == 1);
  }
  {
    Array<int> indices;
    Array<char> chars;
    for (const auto [i, ch] : enumerate(string("ABC"))) {
      indices.push(int(i));
      chars.push(ch);
    }
    SHOW(indices);
    SHOW(chars);
  }
  {
    // Exercise the substitute for std::views::enumerate on all platforms, even where the standard one exists.
    constexpr details::EnumerateFallback enumerate_fallback;
    Array<int> ar{5, 6, 7};
    for (auto [i, e] : ar | enumerate_fallback) e += int(i) * 10;  // The elements are references.
    SHOW(ar);
    using Index = std::remove_cvref_t<decltype(std::get<0>(*ranges::begin(ar | enumerate_fallback)))>;
    static_assert(std::is_same_v<Index, ranges::range_difference_t<Array<int>>>);  // As in the standard.
    const auto to_array = [](auto&& range) -> Array<std::pair<int, char>> {
      Array<std::pair<int, char>> result;
      for (const auto [i, ch] : range) result.push({int(i), ch});
      return result;
    };
    const Array<std::pair<int, char>> pairs = to_array(enumerate_fallback(string("ABC")));
    SHOW(pairs);
    assertx(to_array(enumerate(string("ABC"))) == pairs);  // The standard one, where it exists.
  }
  {
    static_assert(ranges::range<Array<float>>);
    static_assert(std::is_same_v<range_value_t<Array<float>>, float>);
    static_assert(!ranges::range<std::pair<float, float>>);
    static_assert(ranges::sized_range<Array<float>>);
    static_assert(ranges::random_access_range<Array<float>>);
    static_assert(ranges::sized_range<InlinedArray<int, 3>>);
    static_assert(ranges::random_access_range<InlinedArray<int, 3>>);
    const Map<int, float> map;
    static_assert(ranges::sized_range<decltype(map.keys())>);
    static_assert(!ranges::random_access_range<decltype(map.keys())>);
    Mesh mesh;
    static_assert(ranges::sized_range<decltype(mesh.vertices())>);
    static_assert(!ranges::random_access_range<decltype(mesh.vertices())>);
    static_assert(ranges::sized_range<decltype(mesh.ordered_vertices())>);
    static_assert(ranges::random_access_range<decltype(mesh.ordered_vertices())>);
    static_assert(ranges::sized_range<decltype(mesh.faces())>);
    static_assert(!ranges::random_access_range<decltype(mesh.faces())>);
    static_assert(ranges::sized_range<decltype(mesh.ordered_faces())>);
    static_assert(ranges::random_access_range<decltype(mesh.ordered_faces())>);
    static_assert(ranges::sized_range<decltype(mesh.edges())>);
    Vertex v = mesh.create_vertex();
    static_assert(ranges::sized_range<decltype(mesh.faces(v))>);
    static_assert(ranges::random_access_range<decltype(mesh.faces(v))>);
    static_assert(!ranges::sized_range<decltype(mesh.vertices(v))>);
    static_assert(!ranges::random_access_range<decltype(mesh.vertices(v))>);
    Edge e = nullptr;
    static_assert(ranges::sized_range<decltype(mesh.vertices(e))>);
    static_assert(ranges::random_access_range<decltype(mesh.vertices(e))>);
    dummy_use(map, v, e);
  }
  {
    SHOW(mean(range(20)));
    SHOW(mean(range(20) | views::filter([](auto e) { return e != 10; })));
  }
  {
    SHOW(contains(range(20), 13));
    SHOW(contains(range(20) | views::filter([](auto e) { return e != 10; }), 13));
    SHOW(contains(range(20) | views::filter([](auto e) { return e != 10; }), 10));
  }
  {
    // iterable() passes a const-iterable range through by reference, and copies a view that is not.
    const Array<int> ar{1, 2};
    static_assert(std::is_same_v<decltype(iterable(ar)), const Array<int>&>);
    using View = decltype(ar | views::filter([](int i) { return i > 1; }));
    static_assert(!std::is_reference_v<decltype(iterable(std::declval<const View&>()))>);
    assertx(&iterable(ar) == &ar);
  }
  {
    // find_index() and index() on random-access and other ranges.
    const Array<int> ar{5, 7, 5, 9};
    assertx(find_index(ar, 5) == 0 && find_index(ar, 9) == 3 && !find_index(ar, 6));
    assertx(!find_index(Array<int>{}, 5));
    const std::list<int> list{5, 7, 5, 9};
    assertx(find_index(list, 7) == 1 && !find_index(list, 6));
    assertx(index(list, 9) == 3);
    assertx(find_index(range(10) | views::filter([](int i) { return i % 2 == 1; }), 7) == 3);
    assertx(contains(list, 9) && !contains(list, 8) && !contains(Array<int>{}, 0));
  }
  {
    // find_if_ptr() returns the address of the first matching element, through which it may be modified.
    Array<int> ar{1, 4, 6, 8};
    int* p = find_if_ptr(ar, [](int i) { return i % 2 == 0; });
    assertx(p == &ar[1]);
    *p = 40;
    assertx(ar[1] == 40);
    assertx(!find_if_ptr(ar, [](int i) { return i > 100; }));
    const std::list<int> list{3, 5};
    const int* p2 = find_if_ptr(list, [](int i) { return i == 5; });
    assertx(p2 && *p2 == 5);
  }
  {
    // min(), max(), arg_min(), and arg_max() return the first occurrence among ties, optionally with a comparator.
    const Array<int> ar{3, -7, 9, -7, 9, 2};
    SHOW(min(ar), max(ar), arg_min(ar), arg_max(ar));
    const auto abs_less = [](int a, int b) { return abs(a) < abs(b); };
    SHOW(min(ar, abs_less), max(ar, abs_less), arg_min(ar, abs_less), arg_max(ar, abs_less));
    SHOW(min(ar, std::greater<>()), arg_min(ar, std::greater<>()));
    SHOW(max_abs_element(ar), max_abs_element(V(-2.5f)));
    assertx(min(V(4)) == 4 && arg_max(V(4)) == 0);
    assertx(min(range(5, 9) | views::filter([](int i) { return i != 5; })) == 6);
  }
  {
    // fill(), reverse(), rotate(), and sort() return the range for chaining.
    Array<int> ar(5);
    assertx(&fill(ar, 3) == &ar && ar == V(3, 3, 3, 3, 3).view());
    Array<int> ar2{1, 2, 3, 4, 5};
    SHOW(reverse(rotate(ar2, ar2.begin() + 1)));
    SHOW(sort(ar2, std::greater<>()));
    const Array<int> ar3 = sorted(ar2);
    assertx(ar3 == V(1, 2, 3, 4, 5).view() && ar2 == V(5, 4, 3, 2, 1).view());
    assertx(sort(Array<int>{}).num() == 0 && reverse(Array<int>{7}) == V(7).view());
  }
  {
    // swap_elements() on sized ranges and on a range that is not sized.
    Array<int> ar1{1, 2, 3}, ar2{4, 5, 6};
    swap_elements(ar1, ar2);
    assertx(ar1 == V(4, 5, 6).view() && ar2 == V(1, 2, 3).view());
    std::list<int> list{7, 8, 9};
    swap_elements(ar1, list | views::filter([](int) { return true; }));
    assertx(ar1 == V(7, 8, 9).view() && list == std::list<int>({4, 5, 6}));
  }
  {
    // The sums use a wider accumulator type, unless a type is specified.
    const Array<int> ar{std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
    SHOW(sum(ar), type_name(sum(ar)));
    SHOW(sum<double>(V(1, 2)), type_name(sum<double>(V(1, 2))));
    SHOW(sum(Array<float>{}), sum(Array<int>{}));
    SHOW(product(V(100'000, 100'000)), product(V(2.5f)), product(V(uchar{200}, uchar{200})));
    SHOW(mean(Array<uchar>{1, 2}), type_name(mean(Array<uchar>{1, 2})));
    SHOW(mean<float>(V(1, 2)), type_name(mean<float>(V(1, 2))));
    SHOW(mag2(Array<int>{}), mag2(V(3, 4)), mag(V(3, 4)), mag(V(3.f, 4.f)));
  }
  {
    // Compare var() and rms() against a reference computation.
    const Array<float> ar{2.f, 4.f, 4.f, 4.f, 5.f, 5.f, 7.f, 9.f};
    double s1 = 0., s2 = 0.;
    for (const float e : ar) s1 += e, s2 += square(double(e));
    const double n = ar.num();
    assertx(abs(var(ar) - (s2 - s1 * s1 / n) / (n - 1.)) < 1e-12);
    assertx(abs(rms(ar) - std::sqrt(s2 / n)) < 1e-12);
    assertx(abs(mean(ar) - s1 / n) < 1e-12);
    SHOW(mean(ar), var(ar), rms(ar));
  }
  {
    // is_unit(), normalize(), and round_elements().
    SHOW(is_unit(V(.6f, .8f)), is_unit(V(1.f, 1.f)), is_unit(V(1.001f), 1e-2f));
    SHOW(normalize(V(3.f, 0.f, 4.f)));
    SHOW(round_elements(V(1.234567f, -2.5f, 0.000004f)), round_elements(V(1.26f, -1.26f), 10.f));
  }
  {
    // dist2(), dist(), and dot() on ranges of floats and of mixed range types.
    const Array<float> ar1{1.f, 2.f, 3.f};
    const Vec3<float> ar2{4.f, 6.f, 3.f};
    SHOW(dist2(ar1, ar2), dist(ar1, ar2), dot(ar1, ar2));
    SHOW(dist2(Array<int>{}, Array<int>{}), dot(Array<float>{}, Array<float>{}));
  }
  {
    // compare() with a tolerance distinguishes differences larger than the tolerance.
    SHOW(compare(V(1.f, 2.f), V(1.f, 2.5f), .3f), compare(V(1.f, 2.5f), V(1.f, 2.f), .3f));
    SHOW(compare(V(1.f, 2.f), V(1.2f, 9.f), .3f));
    SHOW(compare(Array<int>{}, Array<int>{}));
  }
  {
    // convert() casts each element, truncating toward zero for floating-point to integer.
    SHOW(convert<float>(V(1, 2)), convert<int>(V(1.7f, -1.7f)));
    SHOW(convert<int>(Array<float>{2.9f, -0.5f}));
  }
  {
    // concatenate() of empty ranges, of more than two ranges, and with mutable references.
    assertx(to_int_array(concatenate(Array<int>{}, Array<int>{})).num() == 0);
    assertx(to_int_array(concatenate(Array<int>{}, V(1), Array<int>{}, V(2, 3))) == V(1, 2, 3).view());
    const auto c = concatenate(V(1, 2), V(3), V(4, 5, 6));
    SHOW(c.size(), ranges::distance(c), sum(c));
    static_assert(ranges::forward_range<decltype(c)>);
    Array<int> ar1{1, 2};
    std::vector<int> vec{3};
    for (int& e : concatenate(ar1, vec)) e *= 10;
    assertx(ar1 == V(10, 20).view() && vec == std::vector<int>{30});
  }
  {
    // truncate() yields at most count elements.
    const Array<int> ar{1, 2, 3, 4};
    assertx(to_int_array(ar | truncate(2)) == V(1, 2).view());
    assertx(to_int_array(ar | truncate(0)).num() == 0);
    assertx(to_int_array(ar | truncate(10)) == ar);
    assertx(to_int_array(Array<int>{} | truncate(3)).num() == 0);
    assertx(to_int_array(range(100) | truncate(3)) == V(0, 1, 2).view());
    // Unlike views::take(), truncate() does not advance a single-pass source beyond the last yielded element.
    CountingCursor cursor1(10);
    assertx(to_int_array(cursor1 | truncate(3)) == V(0, 1, 2).view());
    CountingCursor cursor2(10);
    assertx(to_int_array(cursor2 | views::take(3)) == V(0, 1, 2).view());
    SHOW(cursor1.num_advances(), cursor2.num_advances());
    CountingCursor cursor3(2);
    assertx(to_int_array(cursor3 | truncate(5)) == V(0, 1).view());
  }
}
