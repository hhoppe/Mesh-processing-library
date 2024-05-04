// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GridOp.h"

#include "libHh/Filter.h"
#include "libHh/MatrixOp.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/SGrid.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"
using namespace hh;

template <int D> void test(const Vec<int, D>& dims, const Vec<int, D>& ndims) {
  Array<const Filter*> filters;  // not: "gaussian", "preprocess", "justspline", "justomoms"
  for (string s : {"impulse", "box", "triangle", "quadratic", "mitchell", "keys", "spline", "omoms"}) {
    filters.push(&Filter::get(s));
  }
  {  // inverse convolution is partition-of-unity
    Grid<D, float> grid(dims, 1.f);
    inverse_convolution(grid, ntimes<D>(FilterBnd(Filter::get("spline"), Bndrule::reflected)));
    // SHOW(grid);
    assertx(abs(min(grid) - 1.f) < 1e-6f);
    assertx(abs(max(grid) - 1.f) < 1e-6f);
  }
  {  // inverse convolution has unit integral
    Grid<D, float> grid(dims, 0.f);
    Vec<int, D> p;
    for_int(c, D) p[c] = min(c, dims[c] - 1);  // e.g. V(0, 1, 2) for D == 3
    grid[p] = 1.f;
    // SHOW(grid);
    inverse_convolution(grid, ntimes<D>(FilterBnd(Filter::get("spline"), Bndrule::reflected)));
    // SHOW(grid);
    assertx(abs(sum(grid) - 1.f) < 1e-6f);
    // SHOW(Stat(grid));
  }
  {  // rescaling of unity-valued grid reproduces unity-valued grid
    Grid<D, float> grid(dims, 1.f);
    // not: Bndrule::border
    for (Bndrule bndrule : {Bndrule::reflected, Bndrule::periodic, Bndrule::clamped}) {
      for (const Filter* pfilter : filters) {
        const Filter& filter = *pfilter;
        if (filter.has_inv_convolution() && !(bndrule == Bndrule::reflected || bndrule == Bndrule::periodic)) continue;
        Grid<D, float> gridn = scale(grid, ndims, ntimes<D>(FilterBnd(filter, bndrule)));
        // SHOW(Stat(gridn));
        assertx(abs(min(gridn) - 1.f) < 1e-6f);
        assertx(abs(max(gridn) - 1.f) < 1e-6f);
      }
    }
  }
  {  // random samples of unity field all reproduce unity
    for (Bndrule bndrule : {Bndrule::reflected, Bndrule::periodic}) {
      for (string filtername : {"spline", "omoms"}) {
        const Filter& filter = Filter::get(filtername);
        Grid<D, float> grid(dims, 1.f);
        Vec<FilterBnd, D> nfilterbs = inverse_convolution(grid, ntimes<D>(FilterBnd(filter, bndrule)));
        // SHOW(Stat(grid));
        for_int(i, 10) {
          Vec<float, D> p;
          for_int(d, D) p[d] = Random::G.unif();
          assertx(abs(sample_domain(grid, p, nfilterbs) - 1.f) < 1e-6f);
        }
      }
    }
  }
  if (product(dims) <= 30) {  // interpolating filters do interpolate
    Grid<D, float> ogrid(dims);
    for (float& e : ogrid) e = Random::G.unif();
    if (0) SHOW(ogrid);
    for (const Filter* pfilter : filters) {
      const Filter& ofilter = *pfilter;
      if (ofilter.is_impulse()) continue;         // cannot be used in sample_domain()
      if (!ofilter.is_interpolating()) continue;  // (skip mitchell)
      for (Bndrule bndrule : {Bndrule::reflected, Bndrule::periodic, Bndrule::clamped, Bndrule::reflected101}) {
        Grid<D, float> grid(ogrid);
        auto filterbs = ntimes<D>(FilterBnd(ofilter, bndrule));
        if (0) SHOW(filterbs[0].filter().name(), ofilter.name());
        if (ofilter.has_inv_convolution()) {
          if (!(bndrule == Bndrule::reflected || bndrule == Bndrule::periodic)) continue;
          filterbs = inverse_convolution(grid, filterbs);
        }
        for (const Vec<int, D> u : range(ogrid.dims())) {
          Vec<float, D> p;
          for_int(d, D) p[d] = (u[d] + .5f) / ogrid.dim(d);
          float oval = ogrid[u];
          float nval = sample_domain(grid, p, filterbs);
          // SHOW(u, p, oval, nval);
          assertx(abs(nval - oval) < 1e-6f);
        }
      }
    }
  }
}

int main() {
  Timer::set_show_times(-1);
  if (0) {
    Matrix<float> mat = {{1.f, 1.f, 2.f}, {1.f, 1.f, 1.f}, {1.f, 10.f, 1.f}};
    SHOW(mat);
    SHOW(scale_primal(mat, V(2, 2), twice(FilterBnd(Filter::get("triangle"), Bndrule::reflected))));
    SHOW(scale_primal(mat, V(5, 5), twice(FilterBnd(Filter::get("triangle"), Bndrule::reflected))));
    SHOW(scale_primal(scale_primal(mat, V(5, 5), twice(FilterBnd(Filter::get("triangle"), Bndrule::reflected))),
                      V(3, 3), twice(FilterBnd(Filter::get("triangle"), Bndrule::reflected))));
    exit(0);
  }
  {  // most basic experiment
    const int D = 2;
    Grid<D, float> grid(V(2, 2), 1.f);
    Grid<D, float> gridn = scale(grid, V(3, 3), ntimes<D>(FilterBnd(Filter::get("impulse"), Bndrule::reflected)));
    SHOW(Stat(gridn));
    assertx(abs(min(gridn) - 1.f) < 1e-5f);
    assertx(abs(max(gridn) - 1.f) < 1e-5f);
  }
  {
    SHOW("begin test");
    test(V(9), V(3));
    test(V(2), V(3));
    test(V(1), V(3));
    test(V(9, 5, 3), V(4, 5, 7));
    test(V(3, 9, 5, 4), V(2, 7, 5, 2));
    test(V(2, 1, 5, 3), V(1, 2, 1, 4));
    SHOW("end test");
  }
  {  // scaling of Grid<2, T> matches scaling of Matrix<2, T>; no longer applicable
    int cy = 13, cx = 17;
    int ny = 16, nx = 11;
    Matrix<float> mat(cy, cx);
    for (float& e : mat) e = Random::G.unif();
    FilterBnd filterb(Filter::get("spline"), Bndrule::reflected);
    // Matrix<float> matn = scale(mat, ny, nx, {filterb, filterb});
    // SHOW(Stat(matn));
    Grid<2, float> gridn = scale(mat, V(ny, nx), twice(filterb));
    // SHOW(Stat(gridn));
    // assertx(dist(matn, gridn)<1e-5f);
  }
  {
    Grid<2, int> grid(V(20, 20), 5);
    assertx(sum(grid) == 2000);
    Grid<2, int> newgrid = crop(grid, V(0, 0), V(10, 10));
    assertx(newgrid.dims() == V(10, 10));
    assertx(sum(newgrid) == 500);
  }
  {
    Grid<2, Pixel> grid(V(20, 20), Pixel(65, 66, 67, 72));
    assertx(grid[19][19] == Pixel(65, 66, 67, 72));
    Bndrule bndrule = Bndrule::reflected;
    Pixel gcolor(255, 255, 255, 255);
    grid = crop(grid, V(0, 0), V(10, 10), twice(bndrule), &gcolor);
    assertx(grid.dims() == V(10, 10));
    assertx(grid(9, 9) == Pixel(65, 66, 67, 72));
  }
  if (1) {
    string name = "hamming6";
    const Filter& filter = Filter::get(name);
    KernelFunc func = filter.func();
    double radius = filter.radius();
    const int n = 30;
    for_int(i, n) {
      double x = ((double(i) / n) - .5) * radius * 1.1 + 1e-14;
      showf("func(%12f)=%12f\n", x, func(x));
    }
  }
  if (1) {
    for (string name : {"box", "triangle", "quadratic", "mitchell", "keys", "spline", "omoms", "gaussian", "lanczos6",
                        "lanczos10", "hamming6"}) {
      const Filter& filter = Filter::get(name);
      SHOW(filter.name());
      assertx(filter.name() == name);
      KernelFunc func = filter.func();
      double radius = filter.radius();
      bool is_partition_of_unity = filter.is_partition_of_unity();
      SHOW(name);
      const int n = 100'000;
      {
        double sum = 0., xo = 0.;
        for_int(i, n) {
          double x = ((i + .5) / n - .5) * 2. * radius * (1 - 1e-10);
          sum += func(x);
          if (i) assertx(abs(x - xo) < .01);  // verify that the function is continuous
          xo = x;
        }
        sum = sum / double(n) * (2. * radius);
        SHOW(sum);
        assertx(abs(sum - 1.) < (filter.is_unit_integral() ? 1e-5 : .01));
      }
      {
        Stat stat;
        for_int(i, n) {
          double x = (i + .5) / n - .5;
          double sum = 0.;
          for_intL(j, -10, 10 + 1) {
            double xx = x + double(j);
            if (abs(xx) <= radius) sum += func(xx);
          }
          stat.enter(float(sum));
          assertw(abs(sum - 1.) < (is_partition_of_unity ? 1e-5 : .07));  // gaussian needs .07
        }
        if (1) SHOW(stat);
      }
    }
  }
  if (1) {
    SHOW(Filter::get("spline").func()(0.));
    SHOW(Filter::get("spline").func()(1.));
    SHOW(Filter::get("spline").func()(2.));
    SHOW(Filter::get("omoms").func()(0.));
    SHOW(Filter::get("omoms").func()(1.));
    SHOW(Filter::get("omoms").func()(2.));
  }
  if (1) {
    // 1D
    SGrid<int, 5> grid1 = V(1, 2, 3, 4, 5);
    SHOW(scale_filter_nearest(grid1.view(), V(1)));
    SHOW(scale_filter_nearest(grid1.view(), V(2)));
    SHOW(scale_filter_nearest(grid1.view(), V(3)));
    SHOW(scale_filter_nearest(grid1.view(), V(4)));
    SHOW(scale_filter_nearest(grid1.view(), V(5)));
    SHOW(scale_filter_nearest(grid1.view(), V(6)));
    SHOW(scale_filter_nearest(grid1.view(), V(10)));
    SHOW(scale_filter_nearest(grid1.view(), V(11)));
    // 2D
    SGrid<int, 4, 5> grid2 = V(V(1, 2, 3, 4, 5), V(6, 7, 8, 9, 10), V(11, 12, 13, 14, 15), V(16, 17, 18, 19, 20));
    SHOW(scale_filter_nearest(grid2.view(), V(2, 3)));
    SHOW(scale_filter_nearest(grid2.view(), V(1, 8)));
    SHOW(scale_filter_nearest(grid2.view(), V(4, 10)));
    // 3D
    SGrid<int, 2, 2, 5> grid3 =
        V(V(V(1, 2, 3, 4, 5), V(11, 12, 13, 14, 15)), V(V(101, 102, 103, 104, 105), V(111, 112, 113, 114, 115)));
    SHOW(scale_filter_nearest(grid3.view(), V(2, 1, 5)));
    SHOW(scale_filter_nearest(grid3.view(), V(1, 2, 7)));
    SHOW(scale_filter_nearest(grid3.view(), V(1, 1, 3)));
    SHOW(scale_filter_nearest(grid3.view(), V(2, 2, 3)));
    // 4D
    SGrid<int, 2, 2, 2, 2> grid4 =
        V(V(V(V(1, 2), V(3, 4)), V(V(5, 6), V(7, 8))), V(V(V(11, 12), V(13, 14)), V(V(15, 16), V(17, 18))));
    SHOW(scale_filter_nearest(grid4.view(), V(2, 2, 2, 2)));
    SHOW(scale_filter_nearest(grid4.view(), V(2, 1, 2, 2)));
    SHOW(scale_filter_nearest(grid4.view(), V(2, 4, 2, 2)));
    SHOW(scale_filter_nearest(grid4.view(), V(1, 2, 5, 2)));
    SHOW(scale_filter_nearest(grid4.view(), V(6, 2, 2, 2)));
    SHOW(scale_filter_nearest(grid4.view(), V(3, 2, 2, 5)));
    SHOW(grid_column<0>(grid4.view(), V(0, 0, 0, 0)));
    SHOW(grid_column<0>(grid4.view(), V(0, 0, 0, 1)));
    SHOW(grid_column<0>(grid4.view(), V(0, 1, 1, 1)));
    SHOW(grid_column<1>(grid4.view(), V(0, 0, 0, 0)));
    SHOW(grid_column<1>(grid4.view(), V(0, 0, 0, 1)));
    SHOW(grid_column<1>(grid4.view(), V(1, 0, 1, 1)));
    SHOW(grid_column<3>(grid4.view(), V(1, 1, 1, 0)));
  }
  if (1) {
    Grid<2, Pixel> grid(V(20, 20), Pixel(65, 66, 67, 72));
    CGridView<3, Pixel> view = raise_grid_rank(grid);
    assertx(view.dims() == V(1, 20, 20));
    assertx(equal(view[0], grid));
  }
}
