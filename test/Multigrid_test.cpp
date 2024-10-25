// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#define HH_MULTIGRID_TIMER(name)  // override before "Multigrid.h"
#include "libHh/Multigrid.h"

#include "libHh/Args.h"
#include "libHh/Image.h"
#include "libHh/ParallelCoords.h"
#include "libHh/Random.h"
#include "libHh/Stat.h"
#include "libHh/Vector4.h"
using namespace hh;

namespace {

// Notes:
//  see also test_Multigrid2D.cpp and test_Multigrid1D.cpp

// Read an image into a grid.  Used reference grid_orig rather than return value for template matching.
template <int D, typename T> void read_image(Image& image, Grid<D, T>& grid_orig) {
  static_assert(D == 2, "image only valid in 2D");
  image.to_bw();
  grid_orig.init(image.dims());
  parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) { grid_orig[yx] = T{image[yx][0] / 255.f}; });
  if (0) HH_RSTAT(Sorig, grid_orig);
}

template <> inline void read_image(Image& image, Grid<2, Vector4>& grid_orig) {
  assertx(image.zsize() == 3);
  grid_orig.init(image.dims());
  parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) { grid_orig[yx] = Vector4(image[yx]); });
}

// Compute the Laplacian of the given original grid.
template <int D, typename T, typename Periodic, typename Metric>
void setup_rhs(CGridView<D, T> grid_orig, Multigrid<D, T, Periodic, Metric>& multigrid) {
  // HH_TIMER("_setup_rhs");
  GridView<D, T> grid_rhs = multigrid.rhs();
  assertx(same_size(grid_orig, grid_rhs));
  const Vec<int, D> dims = grid_rhs.dims();
  Periodic periodic;
  parallel_for_coords({D * 25}, dims, [&](const Vec<int, D>& u) {
    T vrhs;
    my_zero(vrhs);
    for_int(c, D) {
      if (u[c] > 0) {
        vrhs += grid_orig[u.with(c, u[c] - 1)] - grid_orig[u];
      } else if (periodic(c)) {
        vrhs += grid_orig[u.with(c, dims[c] - 1)] - grid_orig[u];
      }
      if (u[c] < dims[c] - 1) {
        vrhs += grid_orig[u.with(c, u[c] + 1)] - grid_orig[u];
      } else if (periodic(c)) {
        vrhs += grid_orig[u.with(c, 0)] - grid_orig[u];
      }
    }
    grid_rhs[u] = vrhs;
  });
}

template <typename T>
void setup_rhs2(CGridView<2, T> grid_orig, GridView<2, T> grid_rhs, float gradient_sharpening = 1.f,
                float screening_weight = 0.f) {
  // HH_TIMER("_setup_rhs2");
  assertx(same_size(grid_orig, grid_rhs));
  int ny = grid_orig.dim(0), nx = grid_orig.dim(1);
  parallel_for_each({uint64_t(nx * 10)}, range(ny), [&](const int y) {
    for_int(x, nx) {
      T vrhs = -screening_weight * grid_orig[y][x];
      if (y > 0) vrhs += (grid_orig[y - 1][x] - grid_orig[y][x]) * gradient_sharpening;
      if (y < ny - 1) vrhs += (grid_orig[y + 1][x] - grid_orig[y][x]) * gradient_sharpening;
      if (x > 0) vrhs += (grid_orig[y][x - 1] - grid_orig[y][x]) * gradient_sharpening;
      if (x < nx - 1) vrhs += (grid_orig[y][x + 1] - grid_orig[y][x]) * gradient_sharpening;
      grid_rhs[y][x] = vrhs;
    }
  });
}

// Perform a test of reconstructing a grid of random numbers from their Laplacian.
template <int D> void test_random(Args& args) {
  Vec<int, D> dims;
  for_int(c, D) dims[c] = args.get_int();
  // using FType = float;
  using FType = double;
  // using FType = Vector4;  // must disable HH_RSTAT calls below
  Grid<D, FType> grid_orig(dims);
  for (auto& e : grid_orig) e = FType{Random::G.unif()};
  Multigrid<D, FType> multigrid(dims);
  fill(multigrid.initial_estimate(), FType{0});
  if (1) {
    multigrid.set_original(grid_orig);
  } else {
    multigrid.set_desired_mean(mean(grid_orig));
  }
  setup_rhs(grid_orig, multigrid);
  {
    CGridView<D, FType> grid_rhs = multigrid.rhs();
    HH_RSTAT_RMS(Srhs, grid_rhs);
    HH_RSTAT(S0, standardize_rms(Grid<D, FType>(grid_rhs)));
    HH_RSTAT(Sstandardized, FType{.5f} + standardize_rms(Grid<D, FType>(grid_rhs)) * FType{.15f});
  }
  multigrid.solve();
  CGridView<D, FType> grid_result = multigrid.result();
  if (1) HH_RSTAT(Sresult, grid_result);
  HH_RSTAT(Serr, grid_result - grid_orig);
  // as_image(grid_result).write_file("image_result.bmp");
  // as_image(FType{.5f} + standardize_rms(grid_result - grid_orig) * FType{.1f}).write_file("image_err.bmp");
}

// Reconstruct a grid of random numbers from their Laplacian.
template <int D, typename T, typename Periodic = MultigridPeriodicNone<D>>
void test(GridView<D, T> grid_orig, Periodic /*unused*/ = Periodic{}) {
  const Vec<int, D> dims = grid_orig.dims();
  SHOW(dims);
  for (auto& e : grid_orig) e = T{Random::G.unif()};
  Multigrid<D, T, Periodic> multigrid(dims);
  fill(multigrid.initial_estimate(), T{0});
  multigrid.set_desired_mean(mean(grid_orig));
  if (1) multigrid.set_original(grid_orig);
  setup_rhs(grid_orig, multigrid);
  if (0 && max(dims) >= 2047) multigrid.set_num_vcycles(15);
  if (1) multigrid.set_num_vcycles(20);
  multigrid.solve();
  CGridView<D, T> grid_result = multigrid.result();
  auto result_rms_err = max_e(rms(grid_result - grid_orig));
  bool has_an_odd_dim = any_of(dims, [](int i) { return i > 1 && i % 2; });
  const float fudge = has_an_odd_dim ? 2.f : 1.f;
  const float tolerance = std::is_same_v<decltype(max_e(T{})), float> ? 1e-7f : 1e-16f;
  auto expected_rms_err = max(dims) * 10.f * fudge * tolerance;
  if (0) SHOW(expected_rms_err, result_rms_err);
  if (result_rms_err >= expected_rms_err) {
    SHOW(expected_rms_err, result_rms_err);
    if (0) assertnever("");
  }
}

struct MultigridPeriodicDim0 {
  bool operator()(int d) const { return d == 0; }  // only dimension-0 is periodic
};

}  // namespace

int main(int argc, const char** argv) {
  // To disable parallelism: OMP_NUM_THREADS=1
  ParseArgs args(argc, argv);
  const bool standard_test = 1;
  if (standard_test && !args.num()) {  // verify multigrid convergence with different grid dimensions
    if (1) {
      test(Grid<1, float>(1025));
      test(Grid<1, float>(257));
      test(Grid<2, double>(33, 33));
      test(Grid<2, double>(129, 3));
      test(Grid<3, Vector4>(8, 16, 8));
      test(Grid<3, float>(32, 8, 4), MultigridPeriodicDim0());
    } else {
      test(Grid<1, float>(2049));
      test(Grid<1, float>(511));
      test(Grid<2, double>(33, 33));
      test(Grid<2, double>(129, 3));
      test(Grid<3, Vector4>(16, 16, 8));
      test(Grid<3, float>(64, 8, 4), MultigridPeriodicDim0());
    }
    return 0;
  }
  if (standard_test && !args.num()) {  // verify multigrid convergence with different grid dimensions
    test(Grid<1, float>(1024));
    test(Grid<1, float>(4096));
    test(Grid<1, float>(4095));
    test(Grid<1, float>(4097));
    test(Grid<1, float>(511));
    test(Grid<2, double>(64, 64));
    test(Grid<2, double>(8, 256));
    test(Grid<2, double>(256, 8));
    test(Grid<2, double>(65, 65));
    test(Grid<2, double>(5, 127));
    test(Grid<2, double>(129, 3));
    test(Grid<3, Vector4>(16, 16, 16));
    test(Grid<3, Vector4>(15, 32, 8));
    test(Grid<3, Vector4>(1, 16, 128));
    test(Grid<3, float>(64, 8, 4), MultigridPeriodicDim0());
    return 0;
  }
  if (0) {  // debug the iterators with normal and interior functions
    const auto func_norm = [&](int y, int x) { showf("(%2d, %2d) norm\n", y, x); };
    const auto func_interior = [&](int y, int x) { showf("(%2d, %2d) interior\n", y, x); };
    SHOW("beg");
    for_2DL_interior(0, 4, 0, 4, func_norm, func_interior);
    SHOW("beg");
    for_2DL_interior(0, 2, 0, 4, func_norm, func_interior);
    SHOW("beg");
    for_2DL_interior(0, 1, 0, 4, func_norm, func_interior);
    SHOW("beg");
    for_2DL_interior(0, 0, 0, 4, func_norm, func_interior);
    SHOW("beg");
    for_2DL_interior(0, 5, 3, 3, func_norm, func_interior);
    SHOW("beg");
    for_2DL_interior(0, 5, 3, 4, func_norm, func_interior);
    SHOW("beg");
    for_2DL_interior(0, 5, 3, 5, func_norm, func_interior);
    SHOW("beg");
    for_2DL_interior(0, 5, 3, 6, func_norm, func_interior);
    return 0;
  }
  if (0) {  // debug the iterators with normal and interior functions
    const auto func_norm = [&](const Vec2<int>& u) { showf("%s norm\n", make_string(u).c_str()); };
    const auto func_interior = [&](size_t i) {
      Vec2<int> u{int(i / 5), int(i % 5)};
      showf("%s interior\n", make_string(u).c_str());
    };
    SHOW("beg");
    for_coordsL_interior(V(5, 5), V(0, 0), V(5, 5), func_norm, func_interior);
    SHOW("beg");
    for_coordsL_interior(V(5, 5), V(2, 0), V(4, 5), func_norm, func_interior);
    SHOW("beg");
    for_coordsL_interior(V(5, 5), V(0, 0), V(2, 4), func_norm, func_interior);
    return 0;
  }
  if (1) {  // reconstruct noise grid from its Laplacian
    // test_multigrid 256 256 256
    // test_multigrid 4096 4096
    // test_Multigrid 1048576
    if (0) {
    } else if (args.num() == 1) {
      test_random<1>(args);
      return 0;
    } else if (args.num() == 2) {
      test_random<2>(args);
      return 0;
    } else if (args.num() == 3) {
      test_random<3>(args);
      return 0;
    } else if (args.num() > 0) {
      assertnever("");
    }
  }
  if (0) {  // gradient sharpening
    float gradient_sharpening, screening_weight;
    gradient_sharpening = 10.f, screening_weight = 1e8f;  // original
    gradient_sharpening = 1.f, screening_weight = 0.f;    // original
    gradient_sharpening = 3.f, screening_weight = 10.f;   // good gradient amplification
    gradient_sharpening = .7f, screening_weight = 1.f;    // good gradient attenuation
    gradient_sharpening = 1.5f, screening_weight = 1.f;   // good gradient amplification
    string image_name;
    image_name = "multigrid/rampart256.png";
    image_name = "multigrid/rampart2048.png";
    Image image(image_name);
    Grid<2, Vector4> grid_orig;
    read_image(image, grid_orig);
    const Vec2<int> dims = grid_orig.dims();
    Multigrid<2, Vector4> multigrid(dims);
    multigrid.initial_estimate().assign(grid_orig);
    multigrid.set_desired_mean(mean(grid_orig));
    multigrid.set_screening_weight(screening_weight);
    setup_rhs2(grid_orig, multigrid.rhs(), gradient_sharpening, screening_weight);
    if (1) {
      CGridView<2, Vector4> grid_rhs = multigrid.rhs();
      as_image(Vector4(.5f) + standardize_rms(Grid<2, Vector4>(grid_rhs)) * Vector4(.15f)).write_file("image_rhs.bmp");
    }
    multigrid.set_verbose(true);
    multigrid.solve();
    CGridView<2, Vector4> grid_result = multigrid.result();
    as_image(grid_result).write_file("image_result.bmp");
  }
  if (0) {  // reconstruct image from its Laplacian
    string image_name;
    image_name = "multigrid/lake16.png";
    image_name = "multigrid/lake256.png";
    image_name = "multigrid/rampart237.png";
    image_name = "multigrid/rampart255.png";
    image_name = "multigrid/rampart256.png";
    image_name = "multigrid/rampart256r.png";
    image_name = "multigrid/rampart257r.png";
    image_name = "multigrid/rampart2048x64.png";
    image_name = "multigrid/rampart2048x48.png";
    image_name = "multigrid/rampart257.png";
    image_name = "multigrid/rampart2048.png";
    Image image(image_name);
    Grid<2, Vector4> grid_orig;
    read_image(image, grid_orig);
    const Vec2<int> dims = grid_orig.dims();
    Multigrid<2, Vector4> multigrid(dims);
    fill(multigrid.initial_estimate(), Vector4(0.f));
    multigrid.set_original(grid_orig);
    setup_rhs2(grid_orig, multigrid.rhs());
    if (1) {
      CGridView<2, Vector4> grid_rhs = multigrid.rhs();
      as_image(Vector4(.5f) + standardize_rms(Grid<2, Vector4>(grid_rhs)) * Vector4(.15f)).write_file("image_rhs.bmp");
    }
    multigrid.solve();
    CGridView<2, Vector4> grid_result = multigrid.result();
    as_image(grid_result).write_file("image_result.bmp");
    if (1)
      as_image(Vector4(.5f) + standardize_rms(grid_result - grid_orig) * Vector4(.1f)).write_file("image_err.bmp");
  }
  if (1) {  // stitch two images together
    string image_name0, image_name1;
    image_name0 = "multigrid/rampart237.png";
    image_name1 = "multigrid/prismatic237.png";
    image_name0 = "multigrid/rampart2048.png";
    image_name1 = "multigrid/prismatic2048.png";
    Vec3<Grid<2, Vector4>> grids;
    Image image0(image_name0);
    read_image(image0, grids[0]);
    Image image1(image_name1);
    read_image(image1, grids[1]);
    assertx(same_size(grids[0], grids[1]));
    const Vec2<int> dims = grids[0].dims();
    grids[2].init(dims, mean(grids[0]));
    Grid<2, int> grid_labels(dims);
    parallel_for_coords(grid_labels.dims(), [&](const Vec2<int>& yx) {
      if (0) grid_labels[yx] = yx[1] < int(grids[0].dim(0) * .4f) ? 0 : 1;
      if (1) grid_labels[yx] = yx[1] * .7f / grids[0].dim(1) + yx[0] * .2f / grids[0].dim(0) < 0.45f ? 0 : 1;
      if (1 && square(yx[0] - 500) + square(yx[1] - 500) < square(300)) grid_labels[yx] = 2;
    });
    float screening_weight;  // 2048: 1e-5f good; 1e-6f weak; 1e-4f strong; 3e-5 good too; 1e-7f or 0.f none
    // 256: 1e-4f good
    screening_weight = product(dims) >= square(1024) ? 1e-5f : 1e-4f;
    Image image_result(dims);
    for_int(c, 3) {
      Multigrid<2, float> multigrid(dims);
      double mean_orig;
      {
        // HH_TIMER("_setup_stitch");
        const auto func_stitch = [&](int y0, int x0, int y1, int x1, double& vrhs) {
          int label0 = grid_labels[y0][x0];
          int label1 = grid_labels[y1][x1];
          if (1) vrhs += (grids[label0][y1][x1][c] - grids[label0][y0][x0][c]) * .5f;
          if (1) vrhs += (grids[label1][y1][x1][c] - grids[label1][y0][x0][c]) * .5f;  // necessary!
        };
        parallel_for_each(range(dims[0]), [&](const int y) {
          for_int(x, dims[1]) {
            int label = grid_labels[y][x];
            double vrhs = -screening_weight * grids[label][y][x][c];
            if (y > 0) func_stitch(y, x, y - 1, x + 0, vrhs);
            if (y < dims[0] - 1) func_stitch(y, x, y + 1, x + 0, vrhs);
            if (x > 0) func_stitch(y, x, y + 0, x - 1, vrhs);
            if (x < dims[1] - 1) func_stitch(y, x, y + 0, x + 1, vrhs);
            multigrid.rhs()[y][x] = float(vrhs);
          }
        });
        Array<double> sums(dims[0], 0.);
        parallel_for_each(range(dims[0]), [&](const int y) {
          double sum = 0.f;
          for_int(x, dims[1]) sum += grids[grid_labels[y][x]][y][x][c];
          sums[y] = sum;
        });
        mean_orig = sum(sums) / grid_labels.size();
      }
      multigrid.set_desired_mean(mean_orig);
      fill(multigrid.initial_estimate(), 0.f);
      multigrid.set_screening_weight(screening_weight);
      multigrid.solve();
      for_int(y, dims[0]) for_int(x, dims[1]) {
        image_result[y][x][c] = uint8_t(clamp(multigrid.result()[y][x], 0.f, 1.f) * 255.f + .5f);
      }
    }
    image_result.write_file("image_result.bmp");
  }
  return 0;
}

template class hh::Multigrid<2, Vector4>;
// Cannot instantiate classes with D != 2 because specializations would fail.  SFINAE cannot help.
// template class hh::Multigrid<3, float>;
// template class hh::Multigrid<1, double>;
