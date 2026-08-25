// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Grid.h"

#include "libHh/RangeOp.h"
using namespace hh;

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
    // Grid<3, float> grid(3, 4.f, 2); SHOW(grid);  // correctly fails to compile
  }
  {
    Grid<1, int> grid1(256);
    SHOW(ravel_index_list(grid1.dims(), 7));
    SHOW(unravel_index(grid1.dims(), ravel_index_list(grid1.dims(), 7)));
    Grid<2, int> grid2(100, 1000);
    SHOW(ravel_index_list(grid2.dims(), 3, 7));
    SHOW(unravel_index(grid2.dims(), ravel_index_list(grid2.dims(), 3, 7)));
    Grid<3, int> grid3(V(10, 100, 1000));
    SHOW(ravel_index_list(grid3.dims(), 3, 4, 5));
    SHOW(unravel_index(grid3.dims(), ravel_index_list(grid3.dims(), 3, 4, 5)));
    Grid<4, int> grid4(4, 10, 100, 1000);
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
    // SHOW(grid[V(0, 0)], grid[V(0, 0)][0]);  // would require new Grid<> member functions
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
      SHOW((grid[1][V(1, 1)]));

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
