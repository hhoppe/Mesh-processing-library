// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt

#include "libHh/Geometry.h"
#include "libHh/Grid.h"
#include "libHh/RangeOp.h"
#include "libHh/Vec.h"
using namespace hh;

int main() {
  {
    SGrid<int, 2, 3, 4> grid;
    SHOW(grid.grid_dims<3>());
    SHOW(grid.grid_dims<3>()[2]);
    {
      constexpr auto grid_dims = grid.grid_dims<3>();  // NOLINT(readability-static-accessed-through-instance)
      SHOW(grid_dims);
      constexpr auto dim2 = grid_dims[2];
      SHOW(dim2);
    }
    SHOW(grid.size());
    SHOW(grid.const_grid_view<1>().size());
    SHOW(grid.const_grid_view<2>().size());
    SHOW(grid.const_grid_view<3>().size());
    fill(grid.grid_view<3>(), 0);
    grid[1][0][3] = 103;
    grid[0, 2, 3] = 23;
    SHOW(grid.const_grid_view<3>().flat(11));
    SHOW(grid);
    SHOW(grid[1]);
    SHOW(grid[1][0]);
    SHOW((grid[1, 0]));
    SHOW(grid[1][0][3]);
    SHOW((grid[1, 0, 3]));
    SHOW((grid[V(1, 0, 3)]));
    fill(grid[1][2], 12);
    SHOW(grid);
    SHOW(grid == grid);
    for (const size_t i : range(grid.const_grid_view<3>().size())) grid.grid_view<3>().flat(i) = int(i);
    SHOW(grid);
  }
  {
    SGrid<int, 3, 2> grid32 = V(V(20, 21), V(22, 23), V(24, 25));
    SHOW(grid32);
    SHOW(grid32.const_grid_view());
    SHOW(grid32.segment<2>(0).const_grid_view());  // SGrid<int, 2, 2>
    SHOW(grid32.segment<2>(1).const_grid_view());  // SGrid<int, 2, 2>
    SHOW(grid32.segment<1>(1).const_grid_view());  // SGrid<int, 1, 2>
    SGrid<int, 3, 2> grid32b;
    grid32b = V(V(10, 11), V(12, 13), V(14, 15));
    {
      ranges::swap(grid32, grid32b);
    }
    SHOW(grid32.const_grid_view());
    SHOW(grid32b.const_grid_view());
    grid32 = grid32b;
    SHOW(grid32.const_grid_view());
    grid32 = V(V(0, 1), V(2, 3), V(4, 5));
    SHOW(grid32.const_grid_view());
    SGrid<int, 3, 2> gridn(grid32.const_view());
    SHOW(gridn.const_grid_view());
    fill(gridn.grid_view(), 0);
    gridn.grid_view().assign(grid32.const_grid_view());
    SHOW(gridn.const_grid_view());
  }
  {
    constexpr auto grid32f = V(V(20, 21), V(22, 23), V(24, 25));
    SHOW(grid32f.const_grid_view());
    constexpr int v11 = grid32f[1][1];
    static_assert(v11 == 23);
  }
  {
    constexpr SGrid<int, 3, 2> grid32g{V(V(20, 21), V(22, 23), V(24, 25))};
    SHOW(grid32g.const_grid_view());
  }
  {
    constexpr SGrid<int, 3, 2> grid32h{{10, 11}, {12, 13}, {14, 15}};
    SHOW(grid32h.const_grid_view());
  }
  {
    constexpr SGrid<int, 3, 2> grid32i{{20, 21}, {22, 23}, {24, 25}};
    SHOW(grid32i.const_grid_view());
    constexpr int v11 = grid32i[1][1];
    static_assert(v11 == 23);
  }
  {
    SHOW(sizeof(Vec<int, 0>));
    // SHOW(sizeof(SGrid<int>));  // Too few template arguments.
    SHOW(sizeof(SGrid<int, 0>));
    SHOW(sizeof(SGrid<int, 1>));
    SHOW(sizeof(SGrid<int, 2>));
    SHOW(sizeof(SGrid<int, 3>));
    SHOW(sizeof(SGrid<int, 0, 0>));
    SHOW(sizeof(SGrid<int, 0, 1>));
    SHOW(sizeof(SGrid<int, 1, 0>));
    SHOW(sizeof(SGrid<int, 1, 1>));
    SHOW(sizeof(SGrid<int, 1, 2>));
    SHOW(sizeof(SGrid<int, 2, 1>));
    SHOW(sizeof(SGrid<int, 2, 2>));
    SHOW(sizeof(SGrid<int, 1, 1, 1>));
    SHOW(sizeof(SGrid<int, 2, 1, 1>));
    SHOW(sizeof(SGrid<int, 1, 2, 1>));
    SHOW(sizeof(SGrid<int, 1, 1, 2>));
    SHOW(sizeof(SGrid<int, 3, 1, 1>));
    SHOW(sizeof(SGrid<int, 1, 3, 1>));
    SHOW(sizeof(SGrid<int, 1, 1, 3>));
    {
      struct Si0 : SGrid<int, 0> {
        int a;
      };
      SHOW(sizeof(Si0));
    }
    {
      struct Si10 : SGrid<int, 1, 0> {
        int a;
      };
      SHOW(sizeof(Si10));
    }
    {
      struct Si01 : SGrid<int, 0, 1> {
        int a;
      };
      SHOW(sizeof(Si01));
    }
    {
      struct Si11 : SGrid<int, 1, 1> {
        int a;
      };
      SHOW(sizeof(Si11));
    }
    SHOW(sizeof(Vec<uint8_t, 0>));
    SHOW(sizeof(SGrid<uint8_t, 0>));
    SHOW(sizeof(SGrid<uint8_t, 1>));
    SHOW(sizeof(SGrid<uint8_t, 2>));
    SHOW(sizeof(SGrid<uint8_t, 3>));
    SHOW(sizeof(SGrid<uint8_t, 0, 0>));
    SHOW(sizeof(SGrid<uint8_t, 0, 1>));
    SHOW(sizeof(SGrid<uint8_t, 1, 0>));
    SHOW(sizeof(SGrid<uint8_t, 1, 1>));
    SHOW(sizeof(SGrid<uint8_t, 1, 2>));
    SHOW(sizeof(SGrid<uint8_t, 2, 1>));
    SHOW(sizeof(SGrid<uint8_t, 2, 2>));
    SHOW(sizeof(SGrid<uint8_t, 1, 1, 1>));
    SHOW(sizeof(SGrid<uint8_t, 2, 1, 1>));
    SHOW(sizeof(SGrid<uint8_t, 1, 2, 1>));
    SHOW(sizeof(SGrid<uint8_t, 1, 1, 2>));
    SHOW(sizeof(SGrid<uint8_t, 3, 1, 1>));
    SHOW(sizeof(SGrid<uint8_t, 1, 3, 1>));
    SHOW(sizeof(SGrid<uint8_t, 1, 1, 3>));
    {
      struct Su0 : SGrid<uint8_t, 0> {
        uint8_t a;
      };
      SHOW(sizeof(Su0));
    }
    {
      struct Su10 : SGrid<uint8_t, 1, 0> {
        uint8_t a;
      };
      SHOW(sizeof(Su10));
    }
    {
      struct Su01 : SGrid<uint8_t, 0, 1> {
        uint8_t a;
      };
      SHOW(sizeof(Su01));
    }
    {
      struct Su11 : SGrid<uint8_t, 1, 1> {
        uint8_t a;
      };
      SHOW(sizeof(Su11));
    }
  }
  {
#if 0
    SGrid<float> grid_no_dimensions;  // It fails to compile, which is correct.
#endif
#if 0
    using Over = sgrid_leaf_t<3, SGrid<Point, 2, 2>>;  // It fails to compile, which is correct.
#endif
  }
  {
    // SGrid is an alias, so the two spellings are the same type.
    static_assert(std::is_same_v<SGrid<float, 3>, Vec<float, 3>>);
    static_assert(std::is_same_v<SGrid<float, 2, 3>, Vec<Vec<float, 3>, 2>>);
    static_assert(std::is_same_v<SGrid<float, 2, 3, 4>, Vec<Vec<Vec<float, 4>, 3>, 2>>);
    static_assert(std::is_same_v<sgrid_leaf_t<3, SGrid<float, 2, 3, 4>>, float>);
    static_assert(std::is_same_v<sgrid_leaf_t<2, SGrid<float, 2, 3, 4>>, Vec<float, 4>>);
    static_assert(std::is_same_v<sgrid_leaf_t<1, SGrid<float, 2, 3, 4>>, Vec<Vec<float, 4>, 3>>);
    // The nesting stops at the declared element type; Point derives from Vec but is not one.
    static_assert(is_vec_v<Vec<float, 3>> && !is_vec_v<Point> && !is_vec_v<float>);
    static_assert(std::is_same_v<sgrid_leaf_t<2, SGrid<Point, 3, 3>>, Point>);
    // A Vec is a range over its own elements at every level, whatever the nesting.
    static_assert(SGrid<float, 2, 3, 4>().size() == 2);
    static_assert(std::is_same_v<SGrid<float, 2, 3>::value_type, Vec<float, 3>>);
    // Triviality and the empty-base optimization are unaffected.
    static_assert(std::is_standard_layout_v<SGrid<float, 2, 3>>);
    static_assert(std::is_trivially_copyable_v<SGrid<float, 2, 3>>);
    static_assert(std::is_trivially_default_constructible_v<SGrid<float, 2, 3>>);
  }
  {
    // Arithmetic recurses and remains usable in constant evaluation.
    constexpr SGrid<int, 2, 3> ga = {{1, 2, 3}, {4, 5, 6}};
    constexpr SGrid<int, 2, 3> gb = ga + ga;
    static_assert(gb[1][2] == 12);
    constexpr SGrid<int, 2, 3> gc = ga * 2;  // Leaf scalar, recursing past the element type.
    static_assert(gc[1][1] == 10);
    constexpr SGrid<int, 2, 3> gd = 10 - ga;
    static_assert(gd[0, 0] == 9);
    SHOW(ga, gb, gc, gd);
    SHOW(min(ga, gc), max(ga, gc), interp(ga, gc));
    SGrid<int, 2, 3> ge = ga;
    ge *= 3;
    SHOW(ge);
    SHOW(transformed(ga, [](const Vec<int, 3>& v) { return v.rev(); }));
  }
  {
    // Arithmetic recurses and remains usable in constant evaluation.
    constexpr SGrid<float, 2, 3> ga = {{1.f, 2.f, 3.f}, {4.f, 5.f, 6.f}};
    constexpr SGrid<float, 2, 3> gb = ga + ga;
    static_assert(gb[1][2] == 12.f);
    constexpr SGrid<float, 2, 3> gc = ga * 2.f;  // Leaf scalar, recursing past the element type.
    static_assert(gc[1][1] == 10.f);
    constexpr SGrid<float, 2, 3> gd = 10.f - ga;
    static_assert(gd[0, 0] == 9.f);
    SHOW(ga, gb, gc, gd);
    SHOW(min(ga, gc), max(ga, gc), interp(ga, gc));
    SGrid<float, 2, 3> ge = ga;
    ge *= 3.f;
    SHOW(ge);
    SHOW(transformed(ga, [](const Vec<float, 3>& v) { return v.rev(); }));
  }
  {
    // The grid interface is opt-in, with the number of dimensions stated at the use site.
    SGrid<float, 2, 3, 4> g;
    static_assert(g.grid_dims<3>() == V(2, 3, 4));  // NOLINT(readability-static-accessed-through-instance)
    static_assert(g.grid_dims<2>() == V(2, 3));     // NOLINT(readability-static-accessed-through-instance)
    static_assert(g.grid_dims<1>() == V(2));        // NOLINT(readability-static-accessed-through-instance)
    fill(g.grid_view<3>(), 1.f);
    g[1, 2, 3] = 7.f;
    SHOW(g.grid_view<3>().size(), sum(g.grid_view<3>()));
    SHOW(g.grid_view<2>().dims(), g.grid_view<2>().size());
    SHOW(g.size(), g.num(), g[1].size());
    // A nested Vec that is not a grid keeps its Vec behavior; only the caller decides otherwise.
    const Vec3<Vec2<float>> triangle = {{0.f, 0.f}, {1.f, 0.f}, {0.f, 1.f}};
    SHOW(triangle, triangle.size(), triangle[1]);
    SHOW(triangle.grid_dims<2>());
  }
  {
    const auto g = V(V(1, 2), V(3, 4), V(5, 6));
    SHOW(g);
    SHOW(clamp(g, 2, 5));
  }
  {
    // The flat grid view enumerates the leaves in raster order, and writes through it reach the nested Vec.
    SGrid<int, 2, 3, 4> grid;
    for_int(i, 2) for_int(j, 3) for_int(k, 4) grid[i][j][k] = i * 100 + j * 10 + k;
    int count = 0;
    for (const auto& u : range(V(2, 3, 4))) {
      assertx(grid.grid_view<3>().flat(count) == u[0] * 100 + u[1] * 10 + u[2]);
      const int i = u[0], j = u[1], k = u[2];
      assertx(&grid[u] == &grid.grid_view<3>()[u] && &grid[u] == &grid[i, j, k]);
      count++;
    }
    grid.grid_view<3>()[1, 2, 3] = -1;
    assertx(grid[1][2][3] == -1);
    grid.grid_view<2>()[0, 1][2] = -2;  // A rank-2 view has leaves of type Vec<int, 4>.
    assertx(grid[0][1][2] == -2);
    assertx(grid.grid_view<1>()[1][0][0] == 100);
    // Subscripting by a partial coordinate yields the nested Vec.
    static_assert(std::is_same_v<decltype(grid[V(1)]), Vec<Vec<int, 4>, 3>&>);
    static_assert(std::is_same_v<decltype(grid[V(1, 2)]), Vec<int, 4>&>);
    static_assert(std::is_same_v<decltype(grid[V(1, 2, 3)]), int&>);
    assertx(&grid[V(1, 2)] == &grid[1][2] && &grid[V(1)] == &grid[1]);
    // The view types reflect the constness of the object and the requested rank.
    const SGrid<int, 2, 3, 4>& cgrid = grid;
    static_assert(std::is_same_v<decltype(grid.grid_view<3>()), GridView<3, int>>);
    static_assert(std::is_same_v<decltype(cgrid.grid_view<3>()), CGridView<3, int>>);
    static_assert(std::is_same_v<decltype(grid.grid_view<2>()), GridView<2, Vec<int, 4>>>);
    static_assert(std::is_same_v<decltype(grid.grid_view<1>()), GridView<1, Vec<Vec<int, 4>, 3>>>);
    static_assert(std::is_same_v<decltype(grid.const_grid_view<3>()), CGridView<3, int>>);
    static_assert(std::is_same_v<decltype(cgrid[1, 2, 3]), const int&>);
    assertx(cgrid.grid_view<3>().data() == &grid[0][0][0]);
    assertx(cgrid.grid_view<3>().dims() == V(2, 3, 4) && grid.grid_view<2>().dims() == V(2, 3));
  }
  {
    // Grids with a zero-size dimension occupy no elements.
    SGrid<int, 2, 0> grid20;
    static_assert(SGrid<int, 2, 0>::grid_dims<2>() == V(2, 0));
    SHOW(grid20.grid_view<2>().size(), grid20.grid_view<2>().dims(), grid20.size(), grid20[1].size());
    const SGrid<int, 0, 3> grid03{};
    SHOW(grid03.grid_view<2>().size(), grid03.size());
  }
  {
    // A 1-wide grid and a 1D grid.
    SGrid<float, 1, 3> grid13 = {{1.f, 2.f, 3.f}};
    static_assert(SGrid<float, 1, 3>::grid_dims<2>() == V(1, 3));
    SHOW(grid13.grid_view<2>());
    Vec3<int> vec = V(4, 5, 6);
    static_assert(Vec3<int>::grid_dims<1>() == V(3));
    vec.grid_view<1>()[2] = 7;
    SHOW(vec.grid_view<1>());
    SHOW(vec[V(2)]);
  }
  {
    // Lexicographic comparison, equality, and hashing recurse into the nested Vec.
    constexpr SGrid<int, 2, 2> ga = {{1, 2}, {3, 4}};
    constexpr SGrid<int, 2, 2> gb = {{1, 2}, {3, 5}};
    constexpr SGrid<int, 2, 2> gc = {{1, 3}, {0, 0}};
    static_assert(ga < gb && gb < gc && ga != gb && ga == SGrid<int, 2, 2>{{1, 2}, {3, 4}});
    assertx(std::hash<SGrid<int, 2, 2>>()(ga) == std::hash<SGrid<int, 2, 2>>()(SGrid<int, 2, 2>{{1, 2}, {3, 4}}));
    // Structured bindings operate on the outermost level.
    const auto& [row0, row1] = ga;
    SHOW(row0, row1);
  }
  {
    // The remaining elementwise operations recurse to the leaves.
    constexpr SGrid<int, 2, 3> ga = {{1, 2, 3}, {4, 5, 6}};
    constexpr SGrid<int, 2, 3> gb = {{2, 2, 2}, {3, 3, 3}};
    SHOW(-ga, ga % gb, ga % 4, ga / gb, ga * gb);
    SGrid<int, 2, 3> gc = ga;
    gc -= gb;
    gc %= 3;
    gc += 10;
    SHOW(gc);
    constexpr SGrid<float, 2, 2> fa = {{0.f, 1.f}, {2.f, 3.f}};
    constexpr SGrid<float, 2, 2> fb = {{4.f, 5.f}, {6.f, 7.f}};
    constexpr SGrid<float, 2, 2> fc = {{8.f, 9.f}, {10.f, 11.f}};
    SHOW(interp(fa, fb, .25f), interp(fa, fb, fc), interp(fa, fb, fc, V(.5f, .25f, .25f)));
    static_assert(interp(fa, fb, .25f)[1][1] == 6.f);
    // An integral grid interpolates through float at the leaves, truncating the result.
    constexpr SGrid<int, 1, 2> ia = {{0, 10}}, ib = {{10, 20}};
    SHOW(interp(ia, ib, .25f));
  }
}

template class hh::Vec<double*, 1>;
template class hh::Vec<float, 10>;
template class hh::Vec<hh::Vec<hh::Vec<int, 3>, 3>, 2>;
template class hh::Vec<hh::Vec<hh::Vec<unique_ptr<int>, 2>, 1>, 8>;
