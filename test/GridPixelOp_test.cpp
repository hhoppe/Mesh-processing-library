// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GridPixelOp.h"

using namespace hh;

namespace {

bool equal_grids(const auto& g1, const auto& g2) { return g1.dims() == g2.dims() && ranges::equal(g1, g2); }

void show_matrix(const string& name, CMatrixView<Pixel> matrix) {
  showf("%s (%dx%d):\n", name.c_str(), matrix.ysize(), matrix.xsize());
  for_int(y, matrix.ysize()) {
    showf(" ");
    for_int(x, matrix.xsize()) {
      const Pixel& pix = matrix[y, x];
      showf(" (%d %d %d %d)", pix[0], pix[1], pix[2], pix[3]);
    }
    showf("\n");
  }
}

Vec2<FilterBnd> filterbs(const string& filter_name) {
  return twice(FilterBnd(Filter::get(filter_name), Bndrule::reflected));
}

Matrix<Pixel> scaled(CMatrixView<Pixel> matrix, const Vec2<int>& dims, const string& filter_name) {
  Matrix<Pixel> nmatrix(dims);
  scale_Matrix_Pixel(matrix, filterbs(filter_name), nullptr, nmatrix);
  return nmatrix;
}

}  // namespace

int main() {
  // Pixel values are multiples of 4, so that the filtered values used below are integers.
  Matrix<Pixel> matrix(V(4, 4));
  for_int(y, 4) for_int(x, 4) {
    matrix[y, x] = Pixel(narrow_cast<uint8_t>(y * 64 + x * 16), narrow_cast<uint8_t>(x * 8), 200, 252);
  }
  show_matrix("matrix", matrix);
  {
    // Same size with an interpolating filter: a copy.
    assertx(equal_grids(scaled(matrix, V(4, 4), "triangle"), matrix));
  }
  {
    // Trivial magnification by 2 (box and impulse) replicates each pixel.
    const Matrix<Pixel> nmatrix = scaled(matrix, V(8, 8), "box");
    for_int(y, 8) for_int(x, 8) assertx((nmatrix[y, x] == matrix[y / 2, x / 2]));
    assertx(equal_grids(scaled(matrix, V(8, 8), "impulse"), nmatrix));
  }
  {
    // Trivial minification by 2 and by 4 averages blocks with rounding.
    const Matrix<Pixel> nmatrix2 = scaled(matrix, V(2, 2), "box");
    show_matrix("box 2x2", nmatrix2);
    for_int(y, 2) for_int(x, 2) for_int(z, 4) {
      int sum = 0;
      for_int(dy, 2) for_int(dx, 2) sum += matrix[y * 2 + dy, x * 2 + dx][z];
      assertx(nmatrix2[y, x][z] == (sum + 2) / 4);
    }
    const Matrix<Pixel> nmatrix1 = scaled(matrix, V(1, 1), "box");
    show_matrix("box 1x1", nmatrix1);
  }
  {
    // Trivial minification by other integer factors (here 3 and 2) uses the general block average.
    Matrix<Pixel> matrix2(V(6, 4));
    for_int(y, 6) for_int(x, 4) matrix2[y, x] = Pixel(narrow_cast<uint8_t>(y * 40 + x), 0, 255, 255);
    const Matrix<Pixel> nmatrix = scaled(matrix2, V(2, 2), "box");
    show_matrix("box 6x4 -> 2x2", nmatrix);
    for_int(y, 2) for_int(x, 2) {
      int sum = 0;
      for_int(dy, 3) for_int(dx, 2) sum += matrix2[y * 3 + dy, x * 2 + dx][0];
      assertx(nmatrix[y, x][0] == (sum + 3) / 6);
    }
  }
  {
    // A nontrivial filter goes through floating-point resampling; a constant image stays constant.
    show_matrix("triangle 2x2", scaled(matrix, V(2, 2), "triangle"));
    show_matrix("triangle 2x8", scaled(matrix, V(2, 8), "triangle"));
    Matrix<Pixel> constant(V(3, 5), Pixel(10, 20, 30, 40));
    for (const auto& dims : {V(3, 5), V(6, 10), V(7, 2)}) {
      for (const char* filter_name : {"triangle", "mitchell"}) {
        const Matrix<Pixel> nmatrix = scaled(constant, dims, filter_name);
        for (const Pixel& pix : nmatrix) assertx(pix == Pixel(10, 20, 30, 40));
      }
    }
  }
  {
    // A 3D grid is scaled frame by frame.
    Grid<3, Pixel> grid(V(2, 4, 4));
    grid[0].assign(matrix);
    for_int(y, 4) for_int(x, 4) grid[1][y, x] = Pixel::gray(narrow_cast<uint8_t>(x * 60));
    Grid<3, Pixel> ngrid(V(2, 2, 8));
    spatially_scale_Grid3_Pixel(grid, filterbs("triangle"), nullptr, ngrid);
    for_int(f, 2) assertx(equal_grids(ngrid[f], scaled(grid[f], V(2, 8), "triangle")));
    show_matrix("frame 1", ngrid[1]);
  }
}
