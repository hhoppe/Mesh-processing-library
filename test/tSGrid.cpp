// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "SGrid.h"
#include "RangeOp.h"
using namespace hh;

int main() {
    {
        SGrid<int, 2, 3, 4> grid;
        SHOW(grid.dims());
        SHOW(grid.dim(2));
        {
            constexpr auto grid_dims = grid.dims(); SHOW(grid_dims);
            constexpr auto dim2 = grid_dims[2]; SHOW(dim2);
        }
        SHOW(grid.size());
        fill(grid, 0);
        grid[1][0][3] = 103;
        grid[0][2][3] = 23;
        SHOW(grid.raster(11));
        SHOW(grid);
        SHOW(grid[1]);
        SHOW(grid[1][0]);
        SHOW(grid[1][0][3]);
        SHOW((grid[{1, 0, 3}]));
        fill(grid[1][2], 12);
        SHOW(grid);
        SHOW(grid==grid);
        for_size_t(i, grid.size()) { grid.raster(i) = int(i); }
        SHOW(grid);
    }
    {
        SGrid<int, 3, 2> grid32 = { {20, 21}, {22, 23}, {24, 25} };
        SHOW(grid32);
        SHOW(grid32.segment<2>(0)); // SGrid<int, 2, 2>
        SHOW(grid32.segment<2>(1)); // SGrid<int, 2, 2>
        SHOW(grid32.segment<1>(1)); // SGrid<int, 1, 2>
        SGrid<int, 3, 2> grid32b;
        grid32b = { {10, 11}, {12, 13}, {14, 15} };
        { using std::swap; swap(grid32, grid32b); }
        SHOW(grid32);
        SHOW(grid32b);
        grid32 = grid32b;
        SHOW(grid32);
        grid32 = V(V(0, 1), V(2, 3), V(4, 5));
        SHOW(grid32);
        SGrid<int, 3, 2> gridn(grid32.view());
        SHOW(gridn);
        fill(gridn, 0);
        gridn = grid32.view();
        SHOW(gridn);
    }
    {
        CONSTEXPR2 SGrid<int, 3, 2> grid32(V(V(20, 21), V(22, 23), V(24, 25)));
        SHOW(grid32);
    }
    {
        CONSTEXPR2 SGrid<int, 3, 2> grid32 { V(V(20, 21), V(22, 23), V(24, 25)) };
        SHOW(grid32);
    }
    {
        SHOW(sizeof(Vec<int,0>));
        // SHOW(sizeof(SGrid<int>)); // too few template arguments
        SHOW(sizeof(SGrid<int,0>));
        SHOW(sizeof(SGrid<int,1>));
        SHOW(sizeof(SGrid<int,2>));
        SHOW(sizeof(SGrid<int,3>));
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
        { struct S : SGrid<int,0>     { int a; }; SHOW(sizeof(S)); } // SGrid takes zero space
        { struct S : SGrid<int, 1, 0> { int a; }; SHOW(sizeof(S)); }
        { struct S : SGrid<int, 0, 1> { int a; }; SHOW(sizeof(S)); } // SGrid takes zero space
        { struct S : SGrid<int, 1, 1> { int a; }; SHOW(sizeof(S)); }
        SHOW(sizeof(Vec<uchar,0>));
        SHOW(sizeof(SGrid<uchar,0>));
        SHOW(sizeof(SGrid<uchar,1>));
        SHOW(sizeof(SGrid<uchar,2>));
        SHOW(sizeof(SGrid<uchar,3>));
        SHOW(sizeof(SGrid<uchar, 0, 0>));
        SHOW(sizeof(SGrid<uchar, 0, 1>));
        SHOW(sizeof(SGrid<uchar, 1, 0>));
        SHOW(sizeof(SGrid<uchar, 1, 1>));
        SHOW(sizeof(SGrid<uchar, 1, 2>));
        SHOW(sizeof(SGrid<uchar, 2, 1>));
        SHOW(sizeof(SGrid<uchar, 2, 2>));
        SHOW(sizeof(SGrid<uchar, 1, 1, 1>));
        SHOW(sizeof(SGrid<uchar, 2, 1, 1>));
        SHOW(sizeof(SGrid<uchar, 1, 2, 1>));
        SHOW(sizeof(SGrid<uchar, 1, 1, 2>));
        SHOW(sizeof(SGrid<uchar, 3, 1, 1>));
        SHOW(sizeof(SGrid<uchar, 1, 3, 1>));
        SHOW(sizeof(SGrid<uchar, 1, 1, 3>));
        { struct S : SGrid<uchar,0>     { uchar a; }; SHOW(sizeof(S)); } // SGrid takes zero space
        { struct S : SGrid<uchar, 1, 0> { uchar a; }; SHOW(sizeof(S)); }
        { struct S : SGrid<uchar, 0, 1> { uchar a; }; SHOW(sizeof(S)); } // SGrid takes zero space
        { struct S : SGrid<uchar, 1, 1> { uchar a; }; SHOW(sizeof(S)); }
    }
    {
#if 0
        SGrid<float> grid_no_dimensions; // it fails to compile, which is correct
#endif
    }
}

template class hh::SGrid<int, 2, 3, 3>;
