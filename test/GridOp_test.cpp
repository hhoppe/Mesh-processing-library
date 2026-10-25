// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/GridOp.h"

#include "libHh/Boundary.h"
#include "libHh/Filter.h"
#include "libHh/MatrixOp.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

template <int D> void test(const Vec<int, D>& dims, const Vec<int, D>& ndims) {
  // Not: "gaussian", "preprocess", "bspline3", "justomoms".
  const auto filters =
      transformed(V("impulse", "box", "triangle", "quadratic", "mitchell", "cubic", "cardinal3", "omoms3"),
                  [](const std::string_view s) { return &Filter::get(s); });
  {  // Inverse convolution is partition-of-unity.
    Grid<D, float> grid(dims, 1.f);
    inverse_convolution(grid, ntimes<D>(FilterBnd(Filter::get("spline"), Bndrule::reflect)));
    // SHOW(grid);
    assertx(abs(min(grid) - 1.f) < 1e-6f);
    assertx(abs(max(grid) - 1.f) < 1e-6f);
  }
  {  // Inverse convolution has unit integral.
    Grid<D, float> grid(dims, 0.f);
    Vec<int, D> p;
    for_int(c, D) p[c] = min(c, dims[c] - 1);  // E.g. V(0, 1, 2) for D == 3
    grid[p] = 1.f;
    // SHOW(grid);
    inverse_convolution(grid, ntimes<D>(FilterBnd(Filter::get("spline"), Bndrule::reflect)));
    // SHOW(grid);
    assertx(abs(sum(grid) - 1.f) < 1e-6f);
    // SHOW(Stat(grid));
  }
  {  // Rescaling of unity-valued grid reproduces unity-valued grid.
    const Grid<D, float> grid(dims, 1.f);
    // Not: Bndrule::border.
    for (const Bndrule bndrule : {Bndrule::reflect, Bndrule::wrap, Bndrule::clamp}) {
      for (const Filter* pfilter : filters) {
        const Filter& filter = *pfilter;
        if (filter.has_inv_convolution() && !(bndrule == Bndrule::reflect || bndrule == Bndrule::wrap)) continue;
        const Grid<D, float> gridn = scale(grid, ndims, ntimes<D>(FilterBnd(filter, bndrule)));
        // SHOW(Stat(gridn));
        assertx(abs(min(gridn) - 1.f) < 1e-6f);
        assertx(abs(max(gridn) - 1.f) < 1e-6f);
      }
    }
  }
  {  // Random samples of unity field all reproduce unity.
    for (const Bndrule bndrule : {Bndrule::reflect, Bndrule::wrap}) {
      for (const string filter_name : {"spline", "omoms"}) {
        const Filter& filter = Filter::get(filter_name);
        Grid<D, float> grid(dims, 1.f);
        const Vec<FilterBnd, D> nfilterbs = inverse_convolution(grid, ntimes<D>(FilterBnd(filter, bndrule)));
        // SHOW(Stat(grid));
        for_int(i, 10) {
          Vec<float, D> p;
          for_int(d, D) p[d] = Random::G.unif();
          assertx(abs(sample_domain(grid, p, nfilterbs) - 1.f) < 1e-6f);
        }
      }
    }
  }
  if (product(dims) <= 30) {  // Interpolating filters do interpolate.
    Grid<D, float> ogrid(dims);
    for (float& e : ogrid) e = Random::G.unif();
    if (0) SHOW(ogrid);
    for (const Filter* pfilter : filters) {
      const Filter& ofilter = *pfilter;
      if (ofilter.is_impulse()) continue;         // Cannot be used in sample_domain().
      if (!ofilter.is_interpolating()) continue;  // (skip mitchell)
      // The boundary rules and grid types (reflect on a primal grid is the former reflected101).
      for (const auto& [boundary, gridtype] :
           {std::pair{"reflect", Gridtype::dual}, std::pair{"wrap", Gridtype::dual},
            std::pair{"clamp", Gridtype::dual}, std::pair{"reflect", Gridtype::primal},
            std::pair{"wrap", Gridtype::primal}}) {
        if (gridtype == Gridtype::primal && min(ogrid.dims()) < 2) continue;
        const Sampler<D, float> sampler(ogrid, {.gridtype = gridtype, .filter = ofilter, .boundary = boundary});
        for (const Vec<int, D> u : range(ogrid.dims())) {
          // On a primal grid, "wrap" replaces the last sample by the first one.
          if (gridtype == Gridtype::primal && string(boundary) == "wrap" && max(u - ogrid.dims() + 1) == 0) continue;
          Vec<float, D> p;
          for_int(d, D) p[d] =
              gridtype == Gridtype::dual ? (u[d] + .5f) / ogrid.dim(d) : float(u[d]) / (ogrid.dim(d) - 1);
          const float oval = ogrid[u];
          const float nval = sampler(p);
          if (!(abs(nval - oval) < 1e-6f)) assertnever(SSHOW(ofilter.name(), boundary, gridtype, u, p, oval, nval));
        }
      }
    }
  }
}

}  // namespace

int main() {
  Timer::set_show_times(-1);
  if (0) {
    Matrix<float> mat = {{1.f, 1.f, 2.f}, {1.f, 1.f, 1.f}, {1.f, 10.f, 1.f}};
    SHOW(mat);
    SHOW(scale_primal(mat, V(2, 2), twice(FilterBnd(Filter::get("triangle"), Bndrule::reflect))));
    SHOW(scale_primal(mat, V(5, 5), twice(FilterBnd(Filter::get("triangle"), Bndrule::reflect))));
    SHOW(scale_primal(scale_primal(mat, V(5, 5), twice(FilterBnd(Filter::get("triangle"), Bndrule::reflect))), V(3, 3),
                      twice(FilterBnd(Filter::get("triangle"), Bndrule::reflect))));
    exit(0);
  }
  {  // Most basic experiment.
    const int D = 2;
    const Grid<D, float> grid(V(2, 2), 1.f);
    const Grid<D, float> gridn = scale(grid, V(3, 3), ntimes<D>(FilterBnd(Filter::get("impulse"), Bndrule::reflect)));
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
  {  // Magnification by a non-preprocessed filter matches pointwise evaluation by sample_domain().
    Random random(17);
    Matrix<float> mat(5, 4);
    for (float& e : mat) e = random.unif();
    for (const auto& [filter_name, bndrule] :
         {std::pair{"triangle", Bndrule::reflect}, std::pair{"keys", Bndrule::clamp},
          std::pair{"mitchell", Bndrule::wrap}}) {
      const auto filterbs = twice(FilterBnd(Filter::get(filter_name), bndrule));
      for (const Vec2<int> ndims : {V(5, 4), V(9, 11), V(5, 13)}) {
        const Grid<2, float> gridn = scale(mat, ndims, filterbs);
        assertx(gridn.dims() == ndims);
        for (const auto& yx : range(ndims)) {
          const Vec2<float> p((yx[0] + .5f) / ndims[0], (yx[1] + .5f) / ndims[1]);
          const float expected = sample_domain(mat, p, filterbs);
          if (!(abs(gridn[yx] - expected) < 1e-5f)) assertnever(SSHOW(filter_name, ndims, yx, gridn[yx], expected));
        }
      }
    }
  }
  {
    const Grid<2, int> grid(V(20, 20), 5);
    assertx(sum(grid) == 2000);
    const Grid<2, int> newgrid = crop(grid, V(0, 0), V(10, 10));
    assertx(newgrid.dims() == V(10, 10));
    assertx(sum(newgrid) == 500);
  }
  {
    Grid<2, Pixel> grid(V(20, 20), Pixel(65, 66, 67, 72));
    assertx(grid[19, 19] == Pixel(65, 66, 67, 72));
    const Bndrule bndrule = Bndrule::reflect;
    const Pixel gcolor(255, 255, 255, 255);
    grid = crop(grid, V(0, 0), V(10, 10), twice(bndrule), &gcolor);
    assertx(grid.dims() == V(10, 10));
    assertx(grid[9, 9] == Pixel(65, 66, 67, 72));
  }
  {  // Old names and single-letter shortcuts remain aliases of the resampler names.
    for (const auto& [alias, name] :
         {std::pair{"keys", "cubic"}, std::pair{"spline", "cardinal3"}, std::pair{"omoms", "omoms3"},
          std::pair{"justspline", "bspline3"}, std::pair{"lanczos6", "lanczos3"}, std::pair{"hamming6", "hamming3"},
          std::pair{"k", "cubic"}, std::pair{"s", "cardinal3"}, std::pair{"o", "omoms3"}, std::pair{"l", "lanczos3"},
          std::pair{"h", "hamming3"}, std::pair{"i", "impulse"}, std::pair{"b", "box"}, std::pair{"t", "triangle"},
          std::pair{"q", "quadratic"}, std::pair{"m", "mitchell"}, std::pair{"g", "gaussian"},
          std::pair{"p", "preprocess"}})
      assertx(Filter::get(alias).name() == name);
    assertx(&Filter::get("cardinal3").digital_filter_kernel() == &Filter::get("bspline3"));
    assertx(&Filter::get("omoms3").digital_filter_kernel() == &Filter::get("justomoms"));
    assertx(&Filter::get("cubic").digital_filter_kernel() == &Filter::get("cubic"));
    assertx(Filter::get("cardinal5").requires_digital_filter() && !Filter::get("cubic").requires_digital_filter());
    for (const string name : {"sharpcubic", "cardinal5", "omoms5", "lanczos10", "kaiser3", "bspline5", "justomoms5"})
      SHOW(name, Filter::get(name).radius(), Filter::get(name).func()(0.), Filter::get(name).func()(.5));
    SHOW(Filter::get("hamming3").func()(.5));  // The Hamming window uses a0 = 25 / 46, as in resampler.
  }
  {  // Boundary rules fold the exterior taps of a weight row into interior samples.
    assertx(Boundary::get("reflected").name() == "reflect" && Boundary::get("r").name() == "reflect");
    assertx(Boundary::get(Bndrule::wrap).name() == "wrap" && Boundary::get("periodic").name() == "wrap");
    assertx(Boundary::get("border").uses_cval() && !Boundary::get("clamp").uses_cval());
    assertx(Boundary::get("auto").is_auto() && !Boundary::get("reflect").is_auto());
    for (const string name : {"reflect", "wrap", "clamp", "border"}) {
      for (const Gridtype gridtype : {Gridtype::dual, Gridtype::primal}) {
        Array<int> indices;
        Array<float> folded;
        Boundary::get(name).fold(-2, Array<float>{.1f, .2f, .3f, .4f}, 3, gridtype, .5, 0., indices, folded);
        SHOW(name, gridtype, indices, folded);  // Taps at indices -2, -1, 0, 1 of a grid of 3 samples.
      }
    }
  }
  if (1) {
    const string name = "hamming3";
    const Filter& filter = Filter::get(name);
    KernelFunc func = filter.func();
    const double radius = filter.radius();
    const int n = 30;
    for_int(i, n) {
      const double x = ((double(i) / n) - .5) * radius * 1.1 + 1e-14;
      showf("func(%12f)=%12f\n", x, func(x));
    }
  }
  if (1) {
    for (string name : {"box", "triangle", "quadratic", "mitchell", "cubic", "sharpcubic", "cardinal3", "cardinal5",
                        "omoms3", "omoms5", "gaussian", "lanczos3", "lanczos5", "lanczos10", "hamming3", "kaiser3"}) {
      const Filter& filter = Filter::get(name);
      SHOW(filter.name());
      assertx(filter.name() == name);
      KernelFunc func = filter.func();
      const double radius = filter.radius();
      const bool is_partition_of_unity = filter.is_partition_of_unity();
      SHOW(name);
      const int n = 100'000;
      {
        double sum = 0., xo = 0.;
        for_int(i, n) {
          const double x = ((i + .5) / n - .5) * 2. * radius * (1 - 1e-10);
          sum += func(x);
          if (i) assertx(abs(x - xo) < .01);  // Verify that the function is continuous.
          xo = x;
        }
        sum = sum / double(n) * (2. * radius);
        SHOW(sum);
        assertx(abs(sum - 1.) < (filter.is_unit_integral() ? 1e-5 : .01));
      }
      {
        Stat stat;
        for_int(i, n) {
          const double x = (i + .5) / n - .5;
          double sum = 0.;
          for_intL(j, -10, 10 + 1) {
            const double xx = x + double(j);
            if (abs(xx) <= radius) sum += func(xx);
          }
          // For same Stat sd results between CONFIG=win (debug) and others (release).
          const float float_sum = float(sum);
          stat.enter(float_sum);
          assertw(abs(sum - 1.) < (is_partition_of_unity ? 1e-5 : .07));  // Gaussian needs .07.
        }
        if (1) SHOW(stat);
      }
    }
  }
  if (1) {
    SHOW(Filter::get("cardinal3").func()(0.));
    SHOW(Filter::get("cardinal3").func()(1.));
    SHOW(Filter::get("cardinal3").func()(2.));
    SHOW(Filter::get("omoms3").func()(0.));
    SHOW(Filter::get("omoms3").func()(1.));
    SHOW(Filter::get("omoms3").func()(2.));
  }
  if (1) {
    // 1D
    constexpr SGrid<int, 5> grid1 = V(1, 2, 3, 4, 5);
    SHOW(scale_filter_nearest(grid1.grid_view<1>(), V(1)));
    SHOW(scale_filter_nearest(grid1.grid_view<1>(), V(2)));
    SHOW(scale_filter_nearest(grid1.grid_view<1>(), V(3)));
    SHOW(scale_filter_nearest(grid1.grid_view<1>(), V(4)));
    SHOW(scale_filter_nearest(grid1.grid_view<1>(), V(5)));
    SHOW(scale_filter_nearest(grid1.grid_view<1>(), V(6)));
    SHOW(scale_filter_nearest(grid1.grid_view<1>(), V(10)));
    SHOW(scale_filter_nearest(grid1.grid_view<1>(), V(11)));
    // 2D
    constexpr SGrid<int, 4, 5> grid2 =
        V(V(1, 2, 3, 4, 5), V(6, 7, 8, 9, 10), V(11, 12, 13, 14, 15), V(16, 17, 18, 19, 20));
    SHOW(scale_filter_nearest(grid2.grid_view<2>(), V(2, 3)));
    SHOW(scale_filter_nearest(grid2.grid_view<2>(), V(1, 8)));
    SHOW(scale_filter_nearest(grid2.grid_view<2>(), V(4, 10)));
    // 3D
    constexpr SGrid<int, 2, 2, 5> grid3 = {{{1, 2, 3, 4, 5}, {11, 12, 13, 14, 15}},
                                           {{101, 102, 103, 104, 105}, {111, 112, 113, 114, 115}}};
    SHOW(scale_filter_nearest(grid3.grid_view<3>(), V(2, 1, 5)));
    SHOW(scale_filter_nearest(grid3.grid_view<3>(), V(1, 2, 7)));
    SHOW(scale_filter_nearest(grid3.grid_view<3>(), V(1, 1, 3)));
    SHOW(scale_filter_nearest(grid3.grid_view<3>(), V(2, 2, 3)));
    // 4D
    constexpr SGrid<int, 2, 2, 2, 2> grid4 =
        V(V(V(V(1, 2), V(3, 4)), V(V(5, 6), V(7, 8))), V(V(V(11, 12), V(13, 14)), V(V(15, 16), V(17, 18))));
    SHOW(scale_filter_nearest(grid4.grid_view<4>(), V(2, 2, 2, 2)));
    SHOW(scale_filter_nearest(grid4.grid_view<4>(), V(2, 1, 2, 2)));
    SHOW(scale_filter_nearest(grid4.grid_view<4>(), V(2, 4, 2, 2)));
    SHOW(scale_filter_nearest(grid4.grid_view<4>(), V(1, 2, 5, 2)));
    SHOW(scale_filter_nearest(grid4.grid_view<4>(), V(6, 2, 2, 2)));
    SHOW(scale_filter_nearest(grid4.grid_view<4>(), V(3, 2, 2, 5)));
    SHOW(grid_column<0>(grid4.grid_view<4>(), V(0, 0, 0, 0)));
    SHOW(grid_column<0>(grid4.grid_view<4>(), V(0, 0, 0, 1)));
    SHOW(grid_column<0>(grid4.grid_view<4>(), V(0, 1, 1, 1)));
    SHOW(grid_column<1>(grid4.grid_view<4>(), V(0, 0, 0, 0)));
    SHOW(grid_column<1>(grid4.grid_view<4>(), V(0, 0, 0, 1)));
    SHOW(grid_column<1>(grid4.grid_view<4>(), V(1, 0, 1, 1)));
    SHOW(grid_column<3>(grid4.grid_view<4>(), V(1, 1, 1, 0)));
  }
  if (1) {
    const Grid<2, Pixel> grid(V(20, 20), Pixel(65, 66, 67, 72));
    CGridView<3, Pixel> view = raise_grid_rank(grid);
    assertx(view.dims() == V(1, 20, 20));
    assertx(view.data() == grid.data());
    assertx(ranges::equal(view[0], grid));
  }
  {  // The runtime-dimension grid_column() matches the compile-time one, and the mutable column writes through.
    Grid<3, int> grid(V(2, 3, 4));
    for (const size_t i : range(grid.size())) grid.flat(i) = int(i);
    for (const auto& u : range(grid.dims())) {
      for_int(d, 3) {
        if (u[d] != 0) continue;
        const CStridedArrayView<int> column = grid_column(CGridView<3, int>(grid), d, u);
        assertx(column.num() == grid.dim(d));
        for_int(i, column.num()) assertx(column[i] == grid[u.with(d, i)]);
      }
      if (u[1] == 0) assertx(ranges::equal(grid_column<1>(CGridView<3, int>(grid), u), grid_column(grid, 1, u)));
    }
    StridedArrayView<int> column = grid_column<0>(GridView<3, int>(grid), V(0, 2, 3));
    for (int& e : column) e = -e;
    SHOW(grid_column(CGridView<3, int>(grid), 0, V(0, 2, 3)));
    grid_column(GridView<3, int>(grid), 2, V(1, 1, 0))[3] = 99;
    assertx((grid[1, 1, 3]) == 99);
  }
  {  // Cropping, including negative crops that grow the grid using each boundary rule.
    const Grid<2, int> grid = grid_from_flat(V(3, 4), range(12));
    const int bordervalue = -1;
    for (const Bndrule bndrule :
         {Bndrule::reflect, Bndrule::wrap, Bndrule::clamp, Bndrule::border, Bndrule::reflect_primal}) {
      for (const auto& dLU : range(ntimes<4>(-3), ntimes<4>(2))) {
        const Vec2<int> dL = V(dLU[0], dLU[1]), dU = V(dLU[2], dLU[3]);
        const Grid<2, int> newgrid = crop(grid, dL, dU, twice(bndrule), &bordervalue);
        assertx(newgrid.dims() == grid.dims() - dL - dU);
        for (const auto& yx : range(newgrid.dims())) {
          Vec2<int> yx2 = yx + dL;
          const int expected = grid.map_inside(yx2, twice(bndrule)) ? grid[yx2] : bordervalue;
          assertx(newgrid[yx] == expected);
        }
      }
    }
    SHOW(crop(grid, V(1, 1), V(0, 2)));  // The fast path, cropping more than dim0.
    SHOW(crop(grid, V(1, 0), V(1, 0)));  // The fastest path, cropping just dim0.
    SHOW(crop(grid, V(-1, -2), V(-1, -2), twice(Bndrule::reflect)));
    SHOW(crop(grid, V(0, -1), V(0, -1), V(Bndrule::clamp, Bndrule::border), &bordervalue));
    // A Bndrule::undefined dimension (not cropped negatively) combined with a negative crop in another dimension.
    SHOW(crop(grid, V(0, -1), V(0, -1), V(Bndrule::undefined, Bndrule::border), &bordervalue));
    SHOW(crop(grid, V(1, -1), V(0, 0), V(Bndrule::undefined, Bndrule::wrap)));
    for (const auto& dLU : range(V(0, -3, 0, -3), V(2, 2, 2, 2))) {
      const Vec2<int> dL = V(dLU[0], dLU[1]), dU = V(dLU[2], dLU[3]);
      const Grid<2, int> newgrid = crop(grid, dL, dU, V(Bndrule::undefined, Bndrule::reflect_primal));
      assertx(newgrid.dims() == grid.dims() - dL - dU);
      for (const auto& yx : range(newgrid.dims())) {
        int x = yx[1] + dL[1];
        assertx(map_boundaryrule_1d(x, grid.dim(1), Bndrule::reflect_primal));
        assertx(newgrid[yx] == grid[yx[0] + dL[0], x]);
      }
    }
  }
  {  // Assembly of a grid of grids with differing sizes and each alignment.
    Grid<2, Grid<2, int>> grids(V(2, 2));
    grids[0, 0] = Grid<2, int>(V(1, 2), 1);
    grids[0, 1] = Grid<2, int>(V(2, 1), 2);
    grids[1, 0] = Grid<2, int>(V(3, 3), 3);
    grids[1, 1] = Grid<2, int>(V(1, 1), 4);
    for (const Alignment alignment : {Alignment::left, Alignment::center, Alignment::right}) {
      const Grid<2, int> grid = assemble(grids, 0, twice(alignment));
      SHOW(grid);
    }
    SHOW(assemble(grids, 0, V(Alignment::right, Alignment::left)));
    Grid<1, Grid<1, int>> grids1(V(2));
    grids1[0] = Grid<1, int>{1, 2};
    grids1[1] = Grid<1, int>{3, 4, 5};
    SHOW(assemble(grids1));  // Tight packing.
  }
  {  // Conversions between integer and floating-point grids round-trip exactly.
    Grid<1, Pixel> gridu(V(256));
    for_int(i, 256) gridu[i] = Pixel(uint8_t(i), uint8_t(255 - i), uint8_t(i / 2), uint8_t(i * 7));
    Grid<1, Vector4> gridf(gridu.dims());
    convert(gridu, gridf);
    assertx(gridf[0][0] == 0.f && gridf[255][0] == 1.f && gridf[255][1] == 0.f);
    Grid<1, Pixel> gridu2(gridu.dims());
    convert(gridf, gridu2);
    assertx(ranges::equal(gridu, gridu2));
    Grid<2, uint8_t> gridb(V(16, 16));
    for (const size_t i : range(gridb.size())) gridb.flat(i) = uint8_t(i);
    Grid<2, float> gridbf(gridb.dims());
    convert(gridb, gridbf);
    assertx(gridbf[15, 15] == 255.f);
    Grid<2, uint8_t> gridb2(gridb.dims());
    convert(gridbf, gridb2);
    assertx(ranges::equal(gridb, gridb2));
    Grid<1, Vec2<uint8_t>> griduv(V(256));
    for_int(i, 256) griduv[i] = V(uint8_t(i), uint8_t(255 - i));
    Grid<1, Vector4> griduvf(griduv.dims());
    convert(griduv, griduvf);
    Grid<1, Vec2<uint8_t>> griduv2(griduv.dims());
    convert(griduvf, griduv2);
    assertx(ranges::equal(griduv, griduv2));
  }
  {  // Primal scaling with the (interpolating) triangle filter reproduces a linear ramp.
    const Grid<1, float> ramp = grid_from_flat(V(5), range(5) | views::transform([](int i) { return i / 4.f; }));
    const Grid<1, float> ramp2 = scale_primal(ramp, V(9), V(FilterBnd(Filter::get("triangle"), Bndrule::clamp)));
    for_int(i, 9) assertx(abs(ramp2[i] - i / 8.f) < 1e-6f);
    const Grid<2, float> grid(V(3, 3), 2.f);
    const Grid<2, float> grid2 =
        scale_primal(grid, V(5, 7), twice(FilterBnd(Filter::get("spline"), Bndrule::reflect)));
    for (const float e : grid2) assertx(abs(e - 2.f) < 1e-5f);
  }
  {  // Nearest-filter scaling to the same dimensions copies, and to a zero-size grid yields an empty grid.
    const Grid<2, int> grid = grid_from_flat(V(2, 3), range(6));
    assertx(ranges::equal(scale_filter_nearest(grid, V(2, 3)), grid));
    const Grid<2, int> grid0 = scale_filter_nearest(grid, V(0, 3));
    assertx(grid0.dims() == V(0, 3) && grid0.size() == 0);
  }
  {  // Pointwise evaluation with the triangle filter is linear interpolation, with boundary rules outside.
    const Grid<1, float> grid = {0.f, 10.f, 20.f, 40.f};
    const float bordervalue = 100.f;
    const auto filterb_clamp = V(FilterBnd(Filter::get("triangle"), Bndrule::clamp));
    const auto filterb_border = V(FilterBnd(Filter::get("triangle"), Bndrule::border));
    const auto filterb_wrap = V(FilterBnd(Filter::get("triangle"), Bndrule::wrap));
    SHOW(sample_grid(grid, V(0.f), filterb_clamp), sample_grid(grid, V(1.5f), filterb_clamp),
         sample_grid(grid, V(2.25f), filterb_clamp), sample_grid(grid, V(-1.f), filterb_clamp),
         sample_grid(grid, V(4.f), filterb_clamp));
    SHOW(sample_grid(grid, V(-.5f), filterb_border, &bordervalue), sample_grid(grid, V(3.5f), filterb_wrap));
    SHOW(sample_domain(grid, V(.5f), filterb_clamp), sample_domain(grid, V(.125f), filterb_clamp));
    const Grid<2, float> grid2 = {{0.f, 1.f}, {2.f, 3.f}};
    SHOW(sample_grid(grid2, V(.5f, .5f), twice(filterb_clamp[0])),
         sample_grid(grid2, V(1.f, .25f), twice(filterb_clamp[0])));
  }
  {  // Convolution of a pixel grid along each dimension, compared against floating-point evaluation.
    Random random(5);
    Grid<2, Pixel> grid(V(5, 7));
    for (Pixel& pixel : grid) for_int(z, 4) pixel[z] = uint8_t(random.get_uint64() % 256);
    const Pixel bordervalue(255, 0, 0, 255);
    for (const Bndrule bndrule :
         {Bndrule::reflect, Bndrule::wrap, Bndrule::clamp, Bndrule::border, Bndrule::reflect_primal}) {
      for (const Array<float>& kernel :
           {Array<float>{1.f}, Array<float>{.25f, .5f, .25f}, Array<float>{.1f, .2f, .4f, .2f, .1f},
            Array<float>{.0625f, .0625f, .0625f, .625f, .0625f, .0625f, .0625f}}) {
        const int r = kernel.num() / 2;
        for_int(d, 2) {
          const Grid<2, Pixel> newgrid = convolve_d(grid, d, kernel, bndrule, &bordervalue);
          const Grid<2, Pixel> newgrid2 = convolve_d<2, false>(grid, d, kernel, bndrule, &bordervalue);
          assertx(ranges::equal(newgrid, newgrid2));
          for (const auto& yx : range(grid.dims())) {
            Vec4<float> v{};
            for_int(k, kernel.num()) {
              Vec2<int> yx2 = yx.with(d, yx[d] - r + k);
              const Pixel& pixel = grid.map_inside(yx2, twice(bndrule)) ? grid[yx2] : bordervalue;
              for_int(z, 4) v[z] += kernel[k] * pixel[z];
            }
            for_int(z, 4) assertx(abs(newgrid[yx][z] - v[z]) <= .51f);
          }
        }
      }
    }
    const Grid<2, Pixel> grid1 = convolve_d(grid, 1, Array<float>{1.f}, Bndrule::reflect);
    assertx(ranges::equal(grid1, grid));
    SHOW(convolve_d(grid, 1, Array<float>{.25f, .5f, .25f}, Bndrule::clamp)[2]);
  }
  {  // Inverse convolution returns the corresponding non-preprocessing filters.
    Grid<2, float> grid(V(4, 5), 1.f);
    const auto filterbs = inverse_convolution(
        grid, V(FilterBnd(Filter::get("spline"), Bndrule::reflect), FilterBnd(Filter::get("omoms"), Bndrule::wrap)));
    SHOW(filterbs);
    // An inverse convolution along a dimension of size 1 is the identity.
    Grid<2, float> grid1(V(1, 1), 5.f);
    inverse_convolution(grid1, twice(FilterBnd(Filter::get("spline"), Bndrule::reflect)));
    assertx((grid1[0, 0]) == 5.f);
    // The inverse convolution along dimension 0 commutes with the extraction of a column.
    Grid<2, float> grid2(V(6, 2));
    for_int(y, 6) grid2[y, 0] = grid2[y, 1] = float(y * y);
    Grid<1, float> column = grid_from_flat(V(6), grid_column<0>(CGridView<2, float>(grid2), V(0, 0)));
    inverse_convolution(grid2, V(FilterBnd(Filter::get("spline"), Bndrule::reflect),
                                 FilterBnd(Filter::get("spline"), Bndrule::reflect)));
    inverse_convolution(column, V(FilterBnd(Filter::get("spline"), Bndrule::reflect)));
    for_int(y, 6) {
      const float tolerance = 1e-5f * (1.f + abs(column[y]));
      assertx(abs(grid2[y, 0] - column[y]) < tolerance && abs(grid2[y, 1] - column[y]) < tolerance);
    }
  }
}
