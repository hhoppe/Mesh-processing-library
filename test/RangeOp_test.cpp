// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/RangeOp.h"

#include <list>
#include <vector>

#include "libHh/Advanced.h"  // clone()
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
    // SHOW(int(max_abs_element(ar1)));  // abs(uchar) gives compilation warning in clang (-Wabsolute-value)
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
    SHOW(count(ar2, 0));
    SHOW(count(ar2, 3));
    SHOW(count(ar2, 99));
    // SHOW(count_if(ar2, [](uchar uc) { return uc > 5; }));  // Error (lambda used in decltype of SHOW); needs C++20.
    const auto func_gt5 = [](uchar uc) { return uc > 5; };
    SHOW(count_if(ar2, func_gt5));
  }
  {
    Array<float> ar1 = {2.7f, -3.3f, 5.1f, -6.2f, 0.f};
    Array<float> ar2 = {2.7f, -3.3f, 5.2f, -6.2f, 0.f};
    swap_ranges(ar1, ar2);
    SHOW(ar1);
    swap_ranges(ar1, ar2);
    SHOW(normalize(clone(ar1)));
    SHOW(mag(normalize(clone(ar1))));
    SHOW(sort(clone(ar1)));
    SHOW(reverse(sort(clone(ar1))));
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
    SHOW(is_range_v<Array<float>> ? 1 : 0);
    SHOW(is_range_v<std::fstream> ? 1 : 0);
    struct S {
      int _a;
    };
    SHOW(is_range_v<S>);
  }
  {
      // This should fail to compile.
      // S s; SHOW(mean(s));
  } {
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
      rotate(ar, ar[2]);
      SHOW(ar);
    }
    {
      auto ar = clone(ar1);
      std::rotate(ar.begin(), &ar[2], ar.end());
      SHOW(ar);
    }
  }
  {
    Array<int> ar2 = {6, 4, 2};
    for (int i : transform(ar2, [](int j) { return j * j; })) SHOW(i);
  }
  {
    for (int i : transform(std::vector<int>{10, 11}, [](int j) { return j * j; })) SHOW(i);
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
    assertx(index(range(10), 3) == 3);
    assertx(index(V(3, 5, 7), 5) == 1);
  }
  {
    Array<int> indices;
    Array<char> chars;
    for (const auto& [i, ch] : enumerate(string("ABC"))) {
      indices.push(int(i));
      chars.push(ch);
    }
    SHOW(indices);
    SHOW(chars);
  }
  {
    static_assert(is_range_v<Array<float>>);
    static_assert(std::is_same_v<range_value_t<Array<float>>, float>);
    static_assert(!is_range_v<std::pair<float, float>>);
    static_assert(range_has_size_v<Array<float>>);
    static_assert(random_access_range_v<Array<float>>);
    static_assert(range_has_size_v<PArray<int, 3>>);
    static_assert(random_access_range_v<PArray<int, 3>>);
    Map<int, float> map;
    static_assert(range_has_size_v<decltype(map.keys())>);
    static_assert(!random_access_range_v<decltype(map.keys())>);
    Mesh mesh;
    static_assert(range_has_size_v<decltype(mesh.vertices())>);
    static_assert(!random_access_range_v<decltype(mesh.vertices())>);
    static_assert(range_has_size_v<decltype(mesh.ordered_vertices())>);
    static_assert(random_access_range_v<decltype(mesh.ordered_vertices())>);
    static_assert(range_has_size_v<decltype(mesh.faces())>);
    static_assert(!random_access_range_v<decltype(mesh.faces())>);
    static_assert(range_has_size_v<decltype(mesh.ordered_faces())>);
    static_assert(random_access_range_v<decltype(mesh.ordered_faces())>);
    static_assert(range_has_size_v<decltype(mesh.edges())>);
    Vertex v = mesh.create_vertex();
    static_assert(range_has_size_v<decltype(mesh.faces(v))>);
    static_assert(!random_access_range_v<decltype(mesh.faces(v))>);
    static_assert(!range_has_size_v<decltype(mesh.vertices(v))>);
    static_assert(!random_access_range_v<decltype(mesh.vertices(v))>);
    Edge e = nullptr;
    static_assert(range_has_size_v<decltype(mesh.vertices(e))>);
    static_assert(random_access_range_v<decltype(mesh.vertices(e))>);
    dummy_use(map, v, e);
  }
}
