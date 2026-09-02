// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/RangeOp.h"

#include <list>
#include <vector>

#include "libHh/Array.h"
#include "libHh/Map.h"
#include "libHh/Mesh.h"
#include "libHh/PArray.h"
#include "libHh/Vec.h"
using namespace hh;

int main() {
  {
    Array<uchar> ar1 = {4, 200, 254, 3, 7, 2};
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
    // clang: warning: taking the absolute value of unsigned type has no effect [-Wabsolute-value].
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
    int ar[] = {10, 11, 12, 13, 14, 15};  // test C-array
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
    Array<float> ar1 = {2.7f, -3.3f, 5.1f, -6.2f, 0.f};
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
    for (int i : concatenate(V(1, 2), ar1)) result.push(i);
    int c_array[1] = {6};
    for (int i : concatenate(c_array, PArray<int, 2>{7, 8, 9})) result.push(i);
    for (int i : concatenate(std::vector<int>{10, 11}, std::list<int>{12, 13})) result.push(i);
    std::vector<int> vector{14, 15};
    std::list<int> list{16, 17};
    for (int i : concatenate(vector, list)) result.push(i);
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
    for (const auto [i, ch] : views::enumerate(string("ABC"))) {
      indices.push(int(i));
      chars.push(ch);
    }
    SHOW(indices);
    SHOW(chars);
  }
  {
    static_assert(ranges::range<Array<float>>);
    static_assert(std::is_same_v<range_value_t<Array<float>>, float>);
    static_assert(!ranges::range<std::pair<float, float>>);
    static_assert(ranges::sized_range<Array<float>>);
    static_assert(ranges::random_access_range<Array<float>>);
    static_assert(ranges::sized_range<PArray<int, 3>>);
    static_assert(ranges::random_access_range<PArray<int, 3>>);
    Map<int, float> map;
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
}
