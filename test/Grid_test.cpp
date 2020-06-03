// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Grid.h"

#include "RangeOp.h"
using namespace hh;

int main() {
  {
    Grid<3, float> grid(3, 4, 2);
    fill(grid, 2.f);
    grid[V(0, 0, 1)] = 3.f;
    grid[V(1, 0, 0)] = 4.f;
    grid(2, 3, 0) = 5.f;
    SHOW(grid.dims());
    SHOW(grid.size());
    SHOW(grid_stride(grid.dims(), 0));
    SHOW(grid_stride(grid.dims(), 1));
    SHOW(grid_stride(grid.dims(), 2));
    for_size_t(i, grid.size()) SHOW(grid.flat(i));
    for (const auto& u : range(grid.dims())) SHOW(u, grid[u]);
    SHOW(grid(0, 0, 1));
    SHOW(grid(1, 0, 0));
    SHOW(grid(2, 3, 0));
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
    for_int(y, grid.dim(0)) for_int(x, grid.dim(1)) assertx(grid[y][x] == 2);
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
  {
      // Grid<3, float> grid(3, 4.f, 2); SHOW(grid);  // correctly fails to compile
  } {
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
    SHOW((has_ostream_eol<Grid<2, int>>()));
    SHOW((has_ostream_eol<Vec<int, 5>>()));
    constexpr bool b = has_ostream_eol<Vec<int, 5>>();
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
    for_size_t(i, grid.size()) grid.flat(i) = int(i);
    SHOW(grid[0]);
    SHOW(grid[0][0]);
    SHOW(grid[0][0][0]);
    // SHOW(grid[V(0, 0)], grid[V(0, 0)][0]);  // would require new Grid<> member functions
    SHOW(grid[0][V(0, 0)]);
  }
  {
    Grid<3, Vec2<int>> grid(thrice(3));
    for_size_t(i, grid.size()) grid.flat(i) = V(int(i * 10), int(i * 10 + 1));
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
      SHOW(grid);
      SHOW(grid[V<int>()]);
      SHOW(grid[1]);
      SHOW(grid[V(1)]);
      SHOW(grid[1][2]);
      SHOW(grid(1, 2));
      SHOW(grid[V(1, 2)]);
    }
    {
      Grid<4, int> grid{{{{1, 2}, {3, 4}}, {{5, 6}, {7, 8}}}, {{{11, 12}, {13, 14}}, {{15, 16}, {17, 18}}}};
      SHOW(grid);
      SHOW(grid[1]);
      SHOW(grid[V(1)]);
      SHOW(grid[V(1, 1)]);
      SHOW(grid[V(1, 1, 1)]);
      SHOW(grid[V(1, 1, 1, 1)]);
      SHOW(grid[V<int>()][V(1, 1, 1, 1)]);
      SHOW(grid(1, 1, 1, 1));
      SHOW(grid[1][V(1, 1)][1]);
      SHOW(grid[V(1, 1, 1)][1]);
      SHOW(grid[1][V(1, 1, 1)]);
      SHOW(grid[V(1, 1)][V(1, 1)]);
    }
  }
}

template class hh::Grid<2, int>;
// Cannot fully instantiate D != 2 because of Matrix-specialized member functions.
// template class hh::Grid<4, unsigned>;
// template class hh::Grid<3, float*>;
