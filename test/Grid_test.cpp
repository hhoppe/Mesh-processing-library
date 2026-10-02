// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Grid.h"

#include "libHh/RangeOp.h"
using namespace hh;

namespace {

// Return true if the two grids have the same dimensions and the same elements.
bool equal_grids(const auto& g1, const auto& g2) { return g1.dims() == g2.dims() && ranges::equal(g1, g2); }

// Brute-force reference for map_boundaryrule_1d(): returns false if i lies outside a Bndrule::border domain.
bool ref_map_boundaryrule(int& i, int n, Bndrule bndrule) {
  const auto mod = [](int a, int b) { return ((a % b) + b) % b; };
  switch (bndrule) {
    case Bndrule::reflected: {
      const int m = mod(i, 2 * n);
      i = m < n ? m : 2 * n - 1 - m;
      return true;
    }
    case Bndrule::periodic: i = mod(i, n); return true;
    case Bndrule::clamped: i = clamp(i, 0, n - 1); return true;
    case Bndrule::border: return i >= 0 && i < n;
    case Bndrule::reflected101: {
      if (n == 1) {
        i = 0;
        return true;
      }
      const int m = mod(i, 2 * n - 2);
      i = m < n ? m : 2 * n - 2 - m;
      return true;
    }
    default: assertnever("");
  }
}

constexpr auto k_bndrules =
    V(Bndrule::reflected, Bndrule::periodic, Bndrule::clamped, Bndrule::border, Bndrule::reflected101);

// Verify all the inside() and map_inside() variants of a 2D grid against the brute-force reference.
void test_inside_2d(const Vec2<int>& dims) {
  Grid<2, int> grid(dims);
  for (const auto& yx : range(dims)) grid[yx] = yx[0] * 100 + yx[1];
  const int bordervalue = -1;
  for (const Bndrule bndrule : k_bndrules) {
    for_intL(y, -2 * dims[0] - 3, 3 * dims[0] + 3) for_intL(x, -2 * dims[1] - 3, 3 * dims[1] + 3) {
      int yy = y, xx = x;
      const bool expect_inside =
          ref_map_boundaryrule(yy, dims[0], bndrule) && ref_map_boundaryrule(xx, dims[1], bndrule);
      const int expected = expect_inside ? grid[yy, xx] : bordervalue;
      {
        int y2 = y, x2 = x;
        assertx(grid.map_inside(y2, x2, bndrule) == expect_inside);
        if (expect_inside) assertx(y2 == yy && x2 == xx);
      }
      {
        Vec2<int> yx = V(y, x);
        assertx(grid.map_inside(yx, twice(bndrule)) == expect_inside);
        if (expect_inside) assertx(yx == V(yy, xx));
      }
      assertx(grid.inside(y, x, bndrule, &bordervalue) == expected);
      assertx(grid.inside(V(y, x), twice(bndrule), &bordervalue) == expected);
      if (bndrule != Bndrule::border) {
        assertx(grid.inside(y, x, bndrule) == expected);
        assertx(grid.inside(V(y, x), twice(bndrule)) == expected);
      }
      assertx(grid.ok(V(y, x)) == (y >= 0 && y < dims[0] && x >= 0 && x < dims[1]));
    }
  }
}

}  // namespace

int main() {
  {
    Grid<3, float> grid(3, 4, 2);
    fill(grid, 2.f);
    grid[V(0, 0, 1)] = 3.f;
    grid[V(1, 0, 0)] = 4.f;
    grid[2, 3, 0] = 5.f;
    SHOW(grid.dims());
    SHOW(grid.size());
    SHOW(grid_stride(grid.dims(), 0));
    SHOW(grid_stride(grid.dims(), 1));
    SHOW(grid_stride(grid.dims(), 2));
    for (const size_t i : range(grid.size())) SHOW(grid.flat(i));
    for (const auto& u : range(grid.dims())) SHOW(u, grid[u]);
    SHOW((grid[0, 0, 1]));
    SHOW((grid[1, 0, 0]));
    SHOW((grid[2, 3, 0]));
    SHOW(grid);
    for (const auto& u : range(V(3, 4, 2), V(5, 5, 6))) SHOW(u);
    SHOW(1);
    for (const auto& u : range(V(16, 0), V(16, 16))) SHOW(u);
    SHOW(2);
    for (const auto& u : range(V(0, 16), V(16, 16))) SHOW(u);
    SHOW(3);
    for (const auto& u : range(V(0, 7), V(1, 8))) SHOW(u);
    SHOW(4);
    for (const auto& u : range(V(0, 7), V(1, 7))) SHOW(u);
    SHOW(5);
    for (const auto& u : range(V(0, 7), V(0, 7))) SHOW(u);
  }
  {
    Grid<2, int> grid({256, 8}, 2);
    SHOW(grid.dims());
    for_int(y, grid.dim(0)) for_int(x, grid.dim(1)) assertx(grid[y, x] == 2);
  }
  {
    Grid<2, int> grid{{1, 2, 3}, {4, 5, 6}};
    SHOW(grid);
    grid = {{1, 2}, {3, 4}, {5, 6}, {7, 8}};
    SHOW(grid);
    SHOW((Grid<1, int>{1, 2, 3}));
    SHOW((Grid<3, int>{{{1, 2, 3}, {4, 5, 6}}}));
    SHOW((Grid<3, int>{{{1, 2, 3}, {4, 5, 6}}, {{1, 2, 3}, {4, 5, 6}}}));
  }
  if (0) {
    // Grid<3, float> grid(3, 4.f, 2); SHOW(grid);  // Correctly fails to compile.
  }
  {
    const Grid<1, int> grid1(256);
    SHOW(ravel_index_list(grid1.dims(), 7));
    SHOW(unravel_index(grid1.dims(), ravel_index_list(grid1.dims(), 7)));
    const Grid<2, int> grid2(100, 1000);
    SHOW(ravel_index_list(grid2.dims(), 3, 7));
    SHOW(unravel_index(grid2.dims(), ravel_index_list(grid2.dims(), 3, 7)));
    const Grid<3, int> grid3(V(10, 100, 1000));
    SHOW(ravel_index_list(grid3.dims(), 3, 4, 5));
    SHOW(unravel_index(grid3.dims(), ravel_index_list(grid3.dims(), 3, 4, 5)));
    const Grid<4, int> grid4(4, 10, 100, 1000);
    SHOW(ravel_index_list(grid4.dims(), 3, 4, 5, 6));
    SHOW(unravel_index(grid4.dims(), ravel_index_list(grid4.dims(), 3, 4, 5, 6)));
  }
  {
    SHOW((has_ostream_eol_v<Grid<2, int>>));
    SHOW((has_ostream_eol_v<Vec<int, 5>>));
    constexpr bool b = has_ostream_eol_v<Vec<int, 5>>;
    SHOW(b);
  }
  {
    SHOW(ravel_index(V(7, 5), V(2, 1)));
    SHOW(ravel_index(V(7, 5), V(2, 2)));
    SHOW(ravel_index(V(7, 5), V(3, 1)));
    SHOW(ravel_index(V(3, 4, 5, 6), V(1, 0, 0, 0)));
    SHOW(ravel_index(V(3, 4, 5, 6), V(1, 1, 1, 1)));
    {
      constexpr size_t gi = ravel_index(V(7, 5), V(3, 1));
      SHOW(gi);
    }
    {
      constexpr size_t gilist = ravel_index_list(V(7, 5), 3, 1);
      SHOW(gilist);
    }
  }
  {
    Grid<3, int> grid(thrice(3));
    for (const size_t i : range(grid.size())) grid.flat(i) = int(i);
    SHOW(grid[0]);
    SHOW(grid[0][0]);
    SHOW(grid[0][0][0]);
    // SHOW(grid[V(0, 0)], grid[V(0, 0)][0]);  // Would require new Grid<> member functions.
    SHOW(grid[0][V(0, 0)]);
  }
  {
    Grid<3, Vec2<int>> grid(thrice(3));
    for (const size_t i : range(grid.size())) grid.flat(i) = V(int(i * 10), int(i * 10 + 1));
    SHOW(grid[0]);
    SHOW(grid[0][0]);
    SHOW(grid[0][0][0]);
    // SHOW(grid[V(0, 0)]);
    SHOW(grid[0][V(0, 0)]);
    SHOW(grid[0][V(0, 0)][0]);
  }
  {
    {
      Grid<2, int> grid{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}};
      SHOW((grid));
      // SHOW((grid[]));  // Internal compiler error on _MSC_VER.
      SHOW((grid[V<int>()]));
      SHOW((grid[1]));
      SHOW((grid[V(1)]));
      SHOW((grid[1, 2]));
      SHOW((grid[1][2]));
      SHOW((grid[V(1, 2)]));
    }
    {
      Grid<4, int> grid{{{{1, 2}, {3, 4}}, {{5, 6}, {7, 8}}}, {{{11, 12}, {13, 14}}, {{15, 16}, {17, 18}}}};
      SHOW((grid));
      SHOW((grid[1]));
      SHOW((grid[V(1)]));
      SHOW((grid[1, 1]));
      SHOW((grid[1][1]));
      SHOW((grid[V(1, 1)]));

      SHOW((grid[1, 1, 1]));
      SHOW((grid[V(1, 1, 1)]));
      SHOW((grid[1][1][1]));
      SHOW((grid[1][1, 1]));
      SHOW((grid[1, 1][1]));
      SHOW((grid[V(1)][V(1)][V(1)]));
      SHOW((grid[V(1, 1)][1]));
      SHOW((grid[1][V(1, 1)]));
      SHOW((grid[1, 1][V(1)]));
      SHOW((grid[V(1)][1, 1]));

      SHOW((grid[V(1, 1, 1, 1)]));
      SHOW((grid[V<int>()][V(1, 1, 1, 1)]));
      SHOW((grid[1, 1, 1, 1]));
      SHOW((grid[1][V(1, 1)][1]));
      SHOW((grid[V(1, 1, 1)][1]));
      SHOW((grid[1][V(1, 1, 1)]));
      SHOW((grid[V(1, 1)][V(1, 1)]));
    }
  }
  {
    static_assert(ranges::view<CGridView<2, int>> && ranges::view<GridView<2, int>>);
    static_assert(ranges::borrowed_range<CGridView<2, int>> && ranges::borrowed_range<GridView<2, int>>);
    static_assert(ranges::viewable_range<CGridView<2, int>>);  // The rvalue pipes.
    static_assert(!ranges::view<Grid<2, int>> && !ranges::borrowed_range<Grid<2, int>>);
  }
  {
    // Elementwise intent must go through assign(); reseating through reinit() or an explicit std::move().
    static_assert(!std::is_assignable_v<GridView<2, int>&, Grid<2, int>&>);      // gv = grid;
    static_assert(!std::is_assignable_v<GridView<2, int>&, GridView<2, int>&>);  // gv = other_gv;
    static_assert(!std::is_assignable_v<ArrayView<int>, ArrayView<int>>);        // grid[0] = grid[1];
    static_assert(std::is_assignable_v<GridView<2, int>&, GridView<2, int>&&>);  // The one legal form.
  }
  {
    // These must NOT compile (elementwise intent must go through assign(), reseating through reinit()):
    Grid<2, int> grid;
    GridView<2, int> gv = grid, other_gv = grid;
    // gv = grid;
    // gv = other_gv;
    // grid[0] = grid[1];
    dummy_use(grid, gv, other_gv);
  }
  {
    const Grid grid = grid_from_flat(V(2, 3), range(6));
    SHOW(Array(CGridView<2, int>(grid) | views::transform([](int v) { return v + 100; })));
    SHOW(grid);
    SHOW(Array(grid[1] | views::transform([](int v) { return v + 100; })));
  }
  {
    // Grids with zero-size dimensions.
    Grid<2, int> grid0;
    assertx(grid0.dims() == V(0, 0) && grid0.size() == 0 && grid0.data() == nullptr && grid0.begin() == grid0.end());
    const Grid<2, int> grid05(0, 5);
    SHOW(grid05.dims(), grid05.size(), grid05.dim(1));
    SHOW(grid05);
    const Grid<3, int> grid203(2, 0, 3);
    SHOW(grid203.dims(), grid203.size(), grid203[1].dims(), grid203.slice(1, 2).dims());
    assertx(grid203.data() == nullptr);
    SHOW((Grid<1, int>(0)));
    Grid<2, int> grid(V(2, 3), 7);
    grid.init(V(4, 0));
    assertx(grid.size() == 0 && grid.data() == nullptr);
    grid.init(V(2, 2), 3);
    assertx(sum(grid) == 12);
    grid.clear();
    assertx(grid.dims() == V(0, 0) && grid.size() == 0);
  }
  {
    // Reinitializing to a different shape with the same volume reuses the allocation.
    Grid<2, int> grid(2, 6);
    const int* const p = grid.data();
    grid.init(V(3, 4));
    assertx(grid.data() == p && grid.dims() == V(3, 4));
    grid.init(V(12, 1));
    assertx(grid.data() == p);
  }
  {
    // Copy, move, and swap.
    Grid<2, int> grid1 = grid_from_flat(V(2, 3), range(6));
    const Grid<2, int> grid2(grid1);
    assertx(equal_grids(grid1, grid2) && grid2.data() != grid1.data());
    const int* const p1 = grid1.data();
    Grid<2, int> grid3(std::move(grid1));
    assertx(grid3.data() == p1 && grid3.dims() == V(2, 3));
    assertx(grid1.size() == 0 && grid1.dims() == V(0, 0));  // NOLINT(bugprone-use-after-move)
    Grid<2, int> grid4(V(1, 1), 9);
    swap(grid3, grid4);
    assertx(grid4.data() == p1 && grid3.dims() == V(1, 1) && grid3[0, 0] == 9);
    grid4 = Grid<2, int>(V(4, 4), 1);
    assertx(grid4.dims() == V(4, 4) && sum(grid4) == 16);
    grid4 = grid2;  // Copy assignment reshapes.
    assertx(equal_grids(grid4, grid2));
    grid4 = CGridView<2, int>(grid3);  // Assignment from a view.
    assertx(equal_grids(grid4, grid3));
    const Grid grid5{CGridView<2, int>(grid2)};  // The deduction guide.
    static_assert(std::is_same_v<decltype(grid5), const Grid<2, int>>);
    assertx(equal_grids(grid5, grid2));
  }
  {
    // Boundary rules, compared against a brute-force reference, including 1-wide dimensions.
    test_inside_2d(V(3, 4));
    test_inside_2d(V(1, 5));
    test_inside_2d(V(2, 1));
    test_inside_2d(V(1, 1));
    // A readable summary of the 1D mappings on a domain of size 4.
    for (const Bndrule bndrule : k_bndrules) {
      string s;
      for_intL(i, -6, 10) {
        int ii = i;
        s += map_boundaryrule_1d(ii, 4, bndrule) ? sform(" %d", ii) : string(" .");
      }
      showf("%-12s:%s\n", string(boundaryrule_name(bndrule)).c_str(), s.c_str());
    }
    // Distinct boundary rules on each dimension of a 3D grid.
    Grid<3, int> grid(V(2, 3, 4));
    for (const auto& u : range(grid.dims())) grid[u] = u[0] * 100 + u[1] * 10 + u[2];
    const auto bndrules = V(Bndrule::periodic, Bndrule::clamped, Bndrule::reflected);
    const int bordervalue = -1;
    SHOW(grid.inside(V(-1, -1, -1), bndrules), grid.inside(V(2, 3, 4), bndrules), grid.inside(V(5, 7, 9), bndrules));
    const auto bndrules2 = V(Bndrule::border, Bndrule::reflected101, Bndrule::periodic);
    SHOW(grid.inside(V(1, -2, 6), bndrules2, &bordervalue), grid.inside(V(2, 0, 0), bndrules2, &bordervalue));
  }
  {
    // Raveling of all coordinates matches the raster order of range(dims), including 1-wide dimensions.
    const Vec<int, 4> dims = V(3, 1, 4, 2);
    size_t i = 0;
    for (const auto& u : range(dims)) {
      assertx(ravel_index(dims, u) == i);
      assertx(ravel_index_list(dims, u[0], u[1], u[2], u[3]) == i);
      assertx(unravel_index(dims, i) == u);
      i++;
    }
    assertx(i == product_dims<4>(dims.data()));
    SHOW(grid_stride(dims, 0), grid_stride(dims, 1), grid_stride(dims, 2), grid_stride(dims, 3));
    static_assert(product_dims<0>(nullptr) == 1);
  }
  {
    // The constness of subscripts and views.
    Grid<3, int> grid(V(2, 3, 4));
    const Grid<3, int>& cgrid = grid;
    static_assert(std::is_same_v<decltype(grid[0]), GridView<2, int>>);
    static_assert(std::is_same_v<decltype(cgrid[0]), CGridView<2, int>>);
    static_assert(std::is_same_v<decltype(grid[0, 1]), ArrayView<int>>);
    static_assert(std::is_same_v<decltype(cgrid[0, 1]), CArrayView<int>>);
    static_assert(std::is_same_v<decltype(grid[0, 1, 2]), int&>);
    static_assert(std::is_same_v<decltype(cgrid[0, 1, 2]), const int&>);
    static_assert(std::is_same_v<decltype(grid.slice(0, 1)), GridView<3, int>>);
    static_assert(std::is_same_v<decltype(cgrid.slice(0, 1)), CGridView<3, int>>);
    static_assert(std::is_same_v<decltype(grid.array_view()), ArrayView<int>>);
    static_assert(std::is_same_v<decltype(cgrid.array_view()), CArrayView<int>>);
    static_assert(std::is_same_v<decltype(grid.flat(0)), int&>);
    static_assert(std::is_same_v<decltype(cgrid.flat(0)), const int&>);
  }
  {
    // Slices and flat views.
    Grid<3, int> grid(V(4, 2, 3));
    for (const size_t i : range(grid.size())) grid.flat(i) = int(i);
    const auto slice = grid.slice(1, 3);
    SHOW(slice.dims(), (slice[0, 0, 0]), (slice[1, 1, 2]));
    assertx(grid.slice(4, 4).size() == 0 && grid.slice(0, 4).data() == grid.data());
    grid.slice(2, 3)[0, 1, 1] = -1;  // Write through a mutable slice.
    assertx(grid[2, 1, 1] == -1);
    int count = 0;
    for (const auto slice2 : grid.slices()) {
      static_assert(std::is_same_v<std::remove_cvref_t<decltype(slice2)>, GridView<2, int>>);
      assertx(slice2.data() == grid[count].data() && slice2.dims() == V(2, 3));
      count++;
    }
    assertx(count == 4);
    const Grid<2, int> grid32(V(3, 2), 5);
    for (const auto row : grid32.slices()) assertx(row.num() == 2 && row[1] == 5);
    const auto array_view = grid.array_view();
    assertx(array_view.num() == 24 && array_view.data() == grid.data() && array_view[23] == 23);
    const Grid<1, int> grid1 = {4, 5, 6};
    const CGridView<1, int> view1(grid1.array_view());
    assertx(view1.dims() == V(3) && view1[2] == 6);
  }
  {
    // Reversal of rows and columns, for both even and odd sizes.
    for (const Vec2<int> dims : {V(3, 4), V(4, 3), V(1, 5), V(5, 1), V(0, 2)}) {
      const Grid<2, int> grid = grid_from_flat(dims, range(dims[0] * dims[1]));
      Grid<2, int> grid_y(grid), grid_x(grid);
      grid_y.reverse_y();
      grid_x.reverse_x();
      for (const auto& yx : range(dims)) {
        const int y = yx[0], x = yx[1];
        assertx(grid_y[yx] == grid[dims[0] - 1 - y, x]);
        assertx(grid_x[yx] == grid[y, dims[1] - 1 - x]);
      }
    }
    Grid<2, int> grid = grid_from_flat(V(2, 3), range(6));
    grid.reverse_y();
    grid.reverse_x();
    SHOW(grid);
  }
  {
    // Elementwise assign() and the reseating of views.
    const Grid<2, int> grid1 = grid_from_flat(V(2, 3), range(6));
    Grid<2, int> grid2(V(2, 3), 0);
    GridView<2, int> view = grid2;
    view.assign(grid1);
    assertx(equal_grids(grid2, grid1));
    view.assign(grid2);  // Self-assignment is a no-op.
    assertx(equal_grids(grid2, grid1));
    Grid<2, int> grid3(V(4, 4), 1);
    view.reinit(grid3);
    assertx(view.data() == grid3.data() && view.dims() == V(4, 4));
    CGridView<2, int> cview = grid1;
    cview.reinit(grid3);
    assertx(cview.data() == grid3.data());
    assertx(same_size(grid1, grid2) && !same_size(grid1, grid3));
  }
  {
    // Views onto single elements.
    int value = 5;
    Grid1View(value)[0] = 6;
    assertx(value == 6);
    const CGridView<1, int> cview = CGrid1View(value);
    assertx(cview.dims() == V(1) && cview.data() == &value);
  }
  {
    // Changes of rank, which transfer the allocation.
    Grid<2, int> grid2 = grid_from_flat(V(3, 2), range(6));
    const int* const p = grid2.data();
    Grid<3, int> grid3 = increase_grid_rank(std::move(grid2));
    assertx(grid3.dims() == V(1, 3, 2) && grid3.data() == p && grid3[0, 2, 1] == 5);
    assertx(grid2.size() == 0 && grid2.data() == nullptr);  // NOLINT(bugprone-use-after-move)
    Grid<2, int> grid2b = reduce_grid_rank(std::move(grid3));
    assertx(grid2b.dims() == V(3, 2) && grid2b.data() == p);
    assertx(grid3.size() == 0 && grid3.data() == nullptr);  // NOLINT(bugprone-use-after-move)
    Grid<1, int> grid1 = reduce_grid_rank(Grid<2, int>(V(1, 4), 2));
    assertx(grid1.dims() == V(4) && sum(grid1) == 8);
    grid2b.special_reduce_dim0(2);  // Retains the allocation but shrinks the first dimension.
    assertx(grid2b.dims() == V(2, 2) && grid2b[1, 1] == 3);
    grid2b.special_reduce_dim0(0);  // Frees the allocation (else a leak, which LeakSanitizer would report).
    assertx(grid2b.dims() == V(0, 2) && grid2b.data() == nullptr);
    grid2b.init(V(2, 3), 4);
    assertx(sum(grid2b) == 24);
    Grid<2, int> grid2c(V(3, 0));
    grid2c.special_reduce_dim0(1);
    grid2c.special_reduce_dim0(0);
    assertx(grid2c.dims() == V(0, 0) && grid2c.data() == nullptr);
  }
  {
    // Elementwise arithmetic, compared against the per-element results.
    const Grid<2, int> grid1 = grid_from_flat(V(2, 3), range(1, 7));
    const Grid<2, int> grid2(V(2, 3), 4);
    const auto verify = [&](CGridView<2, int> result, auto func) {
      assertx(result.dims() == grid1.dims());
      for_int(i, 6) assertx(result.flat(i) == func(grid1.flat(i), grid2.flat(i)));
    };
    verify(grid1 + grid2, [](int a, int b) { return a + b; });
    verify(grid1 - grid2, [](int a, int b) { return a - b; });
    verify(grid1 * grid2, [](int a, int b) { return a * b; });
    verify(grid1 / grid2, [](int a, int b) { return a / b; });
    verify(grid1 % grid2, [](int a, int b) { return a % b; });
    verify(grid1 * 3, [](int a, int) { return a * 3; });
    verify(10 - grid1, [](int a, int) { return 10 - a; });
    verify(grid1 % 4, [](int a, int) { return a % 4; });
    Grid<2, int> grid3(grid1);
    grid3 += grid2;
    grid3 *= 2;
    grid3 -= 1;
    grid3 /= grid2;
    verify(grid3, [](int a, int b) { return ((a + b) * 2 - 1) / b; });
    SHOW(grid3);
    const Grid<2, float> gridf = transformed(grid1, [](int v) { return v * .5f; });
    static_assert(std::is_same_v<decltype(gridf), const Grid<2, float>>);
    SHOW(gridf);
  }
}

template class hh::CGridView<1, unsigned>;
template class hh::CGridView<2, double>;
template class hh::CGridView<2, unique_ptr<int>>;
template class hh::CGridView<3, const int*>;
template class hh::CGridView<4, unique_ptr<int>>;

template class hh::GridView<1, unsigned>;
template class hh::GridView<2, double>;
template class hh::GridView<2, unique_ptr<int>>;
template class hh::GridView<3, const int*>;
template class hh::GridView<4, unique_ptr<int>>;

template class hh::Grid<1, unsigned>;
template class hh::Grid<2, double>;
template class hh::Grid<2, unique_ptr<int>>;
template class hh::Grid<3, const int*>;
template class hh::Grid<4, unique_ptr<int>>;
