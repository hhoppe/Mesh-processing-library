// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/Gamma.h"
#include "libHh/GridOp.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/ResampleOptions.h"

#include "Resample_parity.inc"  // Defines k_parity_cases.
using namespace hh;

namespace {

// The source grids and points of the parity cases, defined as in resampler_parity.py.
Grid<2, double> parity_source_2d() {
  Grid<2, double> grid(V(4, 5));
  for (const Vec2<int> yx : range(grid.dims())) grid[yx] = double(yx[0] * yx[0] + 3 * yx[1] - yx[1] * yx[0]) / 16.;
  return grid;
}

Grid<1, Vec2<float>> parity_coords_line() {
  Grid<1, Vec2<float>> coords(V(8));
  for_int(i, 8) coords[i] = V(-.25f, -.125f) + float(i) * .1875f;
  return coords;
}

Grid<2, Vec2<float>> parity_coords_grid() {
  Grid<2, Vec2<float>> coords(V(3, 4));
  for_int(i, 3) for_int(j, 4) {
    coords[i, j] = V(-.25f + .375f * float(i) + .125f * float(j), -.125f + .0625f * float(i) + .3125f * float(j));
  }
  return coords;
}

template <typename T> Grid<1, T> parity_integer_source(const Vec<int, 5>& values) {
  Grid<1, T> grid(V(5));
  for_int(i, 5) grid[i] = T(values[i]);
  return grid;
}

// The parameter of each dimension, from a word "a" (the same in every dimension) or "a,b".
Vec2<string> parity_per_dim(const string& word) {
  const size_t comma = word.find(',');
  return comma == string::npos ? V(word, word) : V(word.substr(0, comma), word.substr(comma + 1));
}

// Compare a result of libHh with that of the Python package resampler, for a case of Resample_parity.inc (generated
// by resampler_parity.py): "<function> <parameters> = <values>".
void check_parity(const string& spec) {
  std::istringstream iss(spec);
  string kind;
  iss >> kind;
  const double cval = -1.;
  Array<double> result;
  const auto collect = [&](const auto& grid) {
    for (const auto& v : grid) result.push(double(v));
  };
  if (kind == "resize") {
    string boundary, filter, gridtype, dst_gridtype;
    double scale_, translate;
    int src_size, dst_size;
    iss >> boundary >> filter >> gridtype >> dst_gridtype >> scale_ >> translate >> src_size >> dst_size;
    Grid<1, double> src(V(src_size));
    for_int(i, src_size) src[i] = double(i * i) / 25.;
    const ResizeOptions<1> options{.gridtype = parse_gridtype(gridtype),
                                   .filter = filter,
                                   .boundary = boundary,
                                   .dst_gridtype = parse_gridtype(dst_gridtype),
                                   .scale = scale_,
                                   .translate = translate};
    collect(resize(src, V(dst_size), options, &cval));
  } else if (kind == "resample_line" || kind == "resample_grid") {
    string boundary, filter, gridtype;
    iss >> boundary >> filter >> gridtype;
    const Vec2<string> gridtypes = parity_per_dim(gridtype);
    const SamplingOptions<2> options{.gridtype = V(parse_gridtype(gridtypes[0]), parse_gridtype(gridtypes[1])),
                                     .filter = parity_per_dim(filter),
                                     .boundary = parity_per_dim(boundary)};
    if (kind == "resample_line") {
      collect(resample(parity_source_2d(), parity_coords_line(), options, &cval));
    } else {
      collect(resample(parity_source_2d(), parity_coords_grid(), options, &cval));
    }
  } else if (kind == "uniform_resize") {
    string object_fit, filter, gridtype;
    int ny, nx;
    iss >> object_fit >> filter >> gridtype >> ny >> nx;
    assertx(object_fit == "contain" || object_fit == "cover");
    const ObjectFit fit = object_fit == "contain" ? ObjectFit::contain : ObjectFit::cover;
    const ResizeOptions<2> options{.gridtype = parse_gridtype(gridtype), .filter = filter};
    collect(uniform_resize(parity_source_2d(), V(ny, nx), fit, options, &cval));
  } else if (kind == "gamma") {
    string dtype, gamma, filter;
    int dst_size;
    iss >> dtype >> gamma >> filter >> dst_size;
    const ResizeOptions<1> options{.filter = filter, .gamma = gamma};
    if (dtype == "uint8") {
      collect(resize(parity_integer_source<uint8_t>(V(0, 30, 200, 255, 90)), V(dst_size), options));
    } else {
      assertx(dtype == "uint16");
      collect(resize(parity_integer_source<uint16_t>(V(0, 9000, 50000, 65535, 20000)), V(dst_size), options));
    }
  } else {
    assertnever("Unknown parity case: " + spec);
  }
  string equals;
  Array<double> expected;
  if (iss >> equals && equals == "=") {
    for (double e; iss >> e;) expected.push(e);
  }
  if (!iss.eof() || expected.num() != result.num()) assertnever("Malformed parity case: " + spec);
  for_int(i, result.num()) {
    if (!(abs(result[i] - expected[i]) < 1e-5)) assertnever(SSHOW(spec, i, result[i], expected[i]));
  }
}

}  // namespace

int main() {
  {  // Gamma transfer functions.
    assertx(Gamma::get("power2").decode(.5f) == .25f);
    assertx(Gamma::get("identity").decode(.3f) == .3f && Gamma::get("identity").encode(.3f) == .3f);
    SHOW(Gamma::get("srgb").decode(.5f), Gamma::get("srgb").encode(.25f));
    SHOW(Gamma::get("power22").decode(.5f), Gamma::get("power22").encode(.25f));
    for (const string name : {"identity", "power2", "power22", "srgb"}) {
      const Gamma& gamma = Gamma::get(name);
      assertx(gamma.name() == name);
      for_int(i, 256) {
        const float x = i / 255.f;
        assertx(abs(gamma.encode(gamma.decode(x)) - x) < 1e-6f);
      }
      assertx(gamma.decode(0.f) == 0.f && abs(gamma.decode(1.f) - 1.f) < 1e-6f);
      const CArrayView<float> table = gamma.decode_table_uint8();
      assertx(table.num() == 256 && table[0] == 0.f && abs(table[255] - 1.f) < 1e-6f);
      assertx(table[128] == gamma.decode(128 / 255.f));
    }
    assertx(Gamma::get("power2").decode(.5) == .25);  // The double overload.
  }
  {  // Options structs: per-dimension fields broadcast from one value or take a Vec, and parse names at construction.
    const PerDim<FilterRef, 2> filters = "cubic";
    assertx(filters[0]->name() == "cubic" && filters[1]->name() == "cubic");
    const string filter_name = "lanczos3";  // A name held in a string also converts.
    assertx(ResizeOptions<1>{.filter = filter_name}.filter[0]->name() == "lanczos3");
    const ResizeOptions<2> options{.filter = V("cubic", "triangle"), .boundary = "clamp", .scale = .5};
    assertx(options.filter[1]->name() == "triangle" && options.boundary[0]->name() == "clamp");
    assertx(options.prefilter[0]->is_auto() && options.gridtype[1] == Gridtype::dual && options.scale[1] == .5);
    assertx(options.gamma->is_identity() && !options.dst_gridtype && !options.dim_order && options.translate[0] == 0.);
    const SamplingOptions<2> sampling = options.sampling();
    assertx(sampling.filter[1]->name() == "triangle" && sampling.boundary[1]->name() == "clamp");
    assertx(SamplingOptions<3>{}.filter[2]->name() == "cubic" && SamplingOptions<3>{}.boundary[0]->is_auto());
    const Filter& my_filter = Filter::get("mitchell");
    assertx(ResizeOptions<1>{.filter = my_filter}.filter[0].ptr == &my_filter);
    SHOW(options.filter, options.boundary, options.gamma);
  }
  {  // Primal grids and per-dimension kernels and rules.
    Grid<1, float> ramp(V(5));
    for_int(i, 5) ramp[i] = float(i * i);
    SHOW(
        resize(ramp, V(9), ResizeOptions<1>{.gridtype = Gridtype::primal, .filter = "triangle", .boundary = "clamp"}));
    // On a primal grid, "reflect" is the 101 reflection (the boundary sample is not repeated), as in resampler.
    SHOW(resize(ramp, V(9), ResizeOptions<1>{.gridtype = Gridtype::primal, .filter = "cubic", .boundary = "reflect"}));
    Random random(17);
    Grid<3, float> grid(V(4, 5, 3));
    for (float& e : grid) e = random.unif();
    const ResizeOptions<3> options{.filter = V("cubic", "triangle", "box"), .boundary = V("reflect", "wrap", "clamp")};
    for (const Vec3<int> ndims : {V(7, 2, 6), V(4, 9, 1), V(2, 2, 2)})
      SHOW(ndims, round_fraction_digits(float(sum(resize(grid, ndims, options))), 100.f));
  }
  {  // A single output sample gets a hard cutoff to cval (no antialiased transition), as in resampler.
    Grid<1, float> ramp(V(5));
    for_int(i, 5) ramp[i] = float(i);
    const float cval = -7.f;
    const ResizeOptions<1> options{.filter = "triangle", .boundary = "constant", .translate = .8};
    assertx(resize(ramp, V(1), options, &cval)[0] == cval);  // The sample at .5 maps to -.3, outside the domain.
  }
  {  // Resizer: rows of precomputed weights, with the boundary taps folded into interior samples.
    const Resizer<1> resizer(V(3), V(7), {.filter = "cubic", .boundary = "wrap"});
    for_int(i, 7) {
      const Resizer<1>::Row row = resizer.row(0, i);
      SHOW(i, row.index0, row.indices, row.weights, row.cval_weight);
      assertx(row.indices.num() == 4 && row.weights.num() == 4);  // The window of 4 taps is never interior.
      assertx(abs(sum(row.weights) - 1.f) < 1e-6f);
    }
    Grid<1, float> grid(V(3));
    grid[0] = 1.f, grid[1] = 2.f, grid[2] = 4.f;
    const Grid<1, float> result = resizer.apply(grid);
    assertx(ranges::equal(result, resize(grid, V(7), ResizeOptions<1>{.filter = "cubic", .boundary = "wrap"})));
    SHOW(result);
    const Grid<1, float> ones(V(3), 1.f);  // The resizer applies to any grid of the same dimensions.
    assertx(abs(max(resizer.apply(ones)) - 1.f) < 1e-6f && abs(min(resizer.apply(ones)) - 1.f) < 1e-6f);
    const Resizer<1> resizer2(V(3), V(7), {.filter = "triangle", .boundary = "border"});
    assertx(resizer2.row(0, 0).cval_weight > 0.f && resizer2.row(0, 3).cval_weight == 0.f);
    const float cval = 10.f;
    SHOW(resizer2.apply(grid, &cval));
    assertx(resizer2.apply(grid, &cval)[0] > 3.f);  // The border value (10) shows at the ends: .286 * 10 + .714 * 1.
    // The rules that alter only the boundary rows ("natural" here) keep the dense weights of the interior rows.
    const Resizer<1> resizer3(V(100), V(201), {.filter = "cubic", .boundary = "natural"});
    assertx(resizer3.row(0, 100).indices.num() == 0 && resizer3.row(0, 0).indices.num() > 0);
  }
  {  // Sampler: point reconstruction with any kernel (after its digital filter) and boundary rule.
    Random random(31);
    Matrix<float> mat(5, 4);
    for (float& e : mat) e = random.unif();
    const float cval = .25f;
    for (const auto& [filter_name, boundary_name] :
         {std::pair{"triangle", "reflect"}, std::pair{"cubic", "clamp"}, std::pair{"cardinal3", "reflect"},
          std::pair{"cardinal3", "border"}, std::pair{"omoms5", "natural"}, std::pair{"lanczos3", "wrap"},
          std::pair{"cubic", "linear"}}) {
      const SamplingOptions<2> options{.filter = filter_name, .boundary = boundary_name};
      const Sampler<2, float> sampler(mat, options, &cval);
      for (const Vec2<int> u : range(mat.dims())) {  // Interpolating kernels reproduce the samples.
        const Vec2<float> p((u[0] + .5f) / 5.f, (u[1] + .5f) / 4.f);
        assertx(abs(sampler(p) - mat[u]) < 1e-5f);
        assertx(abs(sampler.at_grid(convert<float>(u)) - mat[u]) < 1e-5f);
      }
      const Vec2<int> ndims(9, 11);  // Magnification by resize() matches point sampling.
      const Grid<2, float> gridn =
          resize(mat, ndims, ResizeOptions<2>{.filter = filter_name, .boundary = boundary_name}, &cval);
      for (const Vec2<int> yx : range(ndims)) {
        const Vec2<float> p((yx[0] + .5f) / ndims[0], (yx[1] + .5f) / ndims[1]);
        if (!(abs(gridn[yx] - sampler(p)) < 1e-5f))
          assertnever(SSHOW(filter_name, boundary_name, yx, gridn[yx], sampler(p)));
      }
      if (!options.filter[0]
               ->requires_digital_filter())  // (sample_domain() asserts that no digital filter is needed.)
        assertx(sample_domain(mat, V(.3f, .6f), options, &cval) == sampler(V(.3f, .6f)));
    }
    {  // A primal grid: the samples lie at i / (n - 1).
      const Sampler<2, float> sampler(mat, {.gridtype = Gridtype::primal, .filter = "cubic", .boundary = "reflect"});
      for (const Vec2<int> u : range(mat.dims())) assertx(abs(sampler(V(u[0] / 4.f, u[1] / 3.f)) - mat[u]) < 1e-5f);
    }
    {  // The "tile" rule maps the point into the unit domain before reconstruction.
      const Sampler<2, float> sampler(mat, {.filter = "cubic", .boundary = "tile"});
      assertx(abs(sampler(V(1.1f, -.4f)) - sampler(V(.1f, .6f))) < 1e-5f);
    }
    {  // A point outside the domain gets cval with "constant", and the reflected value with "reflect".
      const Sampler<2, float> constant(mat, {.filter = "cubic", .boundary = "constant"}, &cval);
      assertx(constant(V(-.2f, .5f)) == cval && constant(V(.5f, 1.1f)) == cval);
      const Sampler<2, float> reflect(mat, {.filter = "cubic", .boundary = "reflect"});
      assertx(abs(reflect(V(-.2f, .5f)) - reflect(V(.2f, .5f))) < 1e-6f);
    }
  }
  {  // resample(): reconstruction at a grid of unit-domain coordinates.
    Random random(5);
    Grid<2, Vector4> grid(V(5, 7));
    for (Vector4& e : grid) {  // (Sequenced, because the evaluation order of arguments varies across compilers.)
      const float r = random.unif(), g = random.unif(), b = random.unif();
      e = Vector4(r, g, b, 1.f);
    }
    Grid<2, Vec2<float>> coords(grid.dims());  // The identity: the dual sample positions.
    for (const Vec2<int> u : range(grid.dims())) coords[u] = (convert<float>(u) + .5f) / convert<float>(grid.dims());
    const Grid<2, Vector4> same = resample(grid, coords, {.filter = "lanczos3"});
    for (const Vec2<int> u : range(grid.dims())) assertx(dist(same[u], grid[u]) < 1e-5f);
    Grid<1, Vec2<float>> line(V(6));  // A line of points crossing the boundary x = 1 of the domain.
    for_int(i, 6) line[i] = V(.5f, .9f + i * .05f);
    const Vector4 cval(0.f, 0.f, 0.f, 0.f);
    const Grid<1, Vector4> along = resample(grid, line, {.filter = "cubic", .boundary = "constant"}, &cval);
    SHOW(along);
    // The transition to cval (alpha 1, 1, .5, 0, 0, 0) is antialiased and monotone.
    for_int(i, 5) assertx(along[i][3] >= along[i + 1][3] - 1e-6f);
    assertx(abs(along[0][3] - 1.f) < 1e-6f && abs(along[2][3] - .5f) < 1e-6f && along[5][3] == 0.f);
  }
  {  // Integer sample types are decoded with the gamma, resampled, and encoded with rounding and clamping.
    Grid<1, uint8_t> g8(V(4));
    g8[0] = 0, g8[1] = 100, g8[2] = 200, g8[3] = 255;
    const Grid<1, uint8_t> r8 = resize(g8, V(7), ResizeOptions<1>{.filter = "triangle"});
    SHOW(convert<int>(r8));
    Grid<1, float> f(V(4));
    for_int(i, 4) f[i] = g8[i] / 255.f;
    const Grid<1, float> rf = resize(f, V(7), ResizeOptions<1>{.filter = "triangle"});
    for_int(i, 7) assertx(r8[i] == uint8_t(clamp(rf[i], 0.f, 1.f) * 255.f + .5f));
    Grid<1, uint8_t> two(V(2));
    two[0] = 0, two[1] = 255;  // The midpoint is 128 in the encoded space but sqrt(.5) * 255 = 180 with power2.
    assertx(resize(two, V(3), ResizeOptions<1>{.filter = "triangle"})[1] == 128);
    assertx(resize(two, V(3), ResizeOptions<1>{.filter = "triangle", .gamma = "power2"})[1] == 180);
    Grid<1, uint16_t> g16(V(4));  // The overshoot of the cubic kernel saturates at the maximum value.
    g16[0] = 0, g16[1] = 30000, g16[2] = 65535, g16[3] = 65535;
    const Grid<1, uint16_t> r16 = resize(g16, V(9), ResizeOptions<1>{.filter = "cubic"});
    SHOW(r16);
    assertx(max(r16) == 65535 && r16[0] == 0);
    Grid<1, uint32_t> g32(V(4));
    g32[0] = 0, g32[1] = 1'000'000'000u, g32[2] = 4294967295u, g32[3] = 4294967295u;
    const Grid<1, uint32_t> r32 = resize(g32, V(9), ResizeOptions<1>{.filter = "cubic", .gamma = "power2"});
    SHOW(r32);
    // (The float32 kernel weights limit the precision of 32-bit samples to about 2^-24 relative.)
    assertx(max(r32) == 4294967295u && r32[0] == 0 && r32[8] > 4294967295u - 512u);
    Grid<1, Vec3<uint16_t>> gv(V(3));
    gv[0] = convert<uint16_t>(V(0, 100, 65535)), gv[1] = convert<uint16_t>(V(1000, 200, 0));
    gv[2] = convert<uint16_t>(V(2000, 300, 65535));
    SHOW(resize(gv, V(5), ResizeOptions<1>{.filter = "triangle", .gamma = "srgb"}));
    Matrix<Pixel> pix(V(4, 4), Pixel(255, 255, 255, 255));  // The left half is white and the right half black.
    for (const Vec2<int> u : range(pix.dims()))
      if (u[1] >= 2) pix[u] = Pixel(0, 0, 0, 255);
    const Sampler<2, Pixel> sampler(pix, {.filter = "cubic"});
    assertx(sampler(V(.5f, .3125f)) == Pixel(255, 255, 255, 255));  // Overshoot above 255 is clamped.
    assertx(sampler(V(.5f, .6875f)) == Pixel(0, 0, 0, 255));        // Undershoot below 0 is clamped.
    const Sampler<2, Pixel> sampler2(pix, {.filter = "triangle", .gamma = "power2"});
    SHOW(sampler2(V(.5f, .5f)));  // The blend of white and black in linear space, encoded back: 180.
    assertx(sampler2(V(.5f, .5f))[0] == 180);
    // An identity resize copies the samples exactly, whatever the gamma.
    assertx(ranges::equal(resize(pix, pix.dims(), ResizeOptions<2>{.filter = "cubic", .gamma = "srgb"}), pix));
    const Grid<2, Pixel> near = resize(pix, V(7, 5), ResizeOptions<2>{.filter = "impulse"});  // An exact gather.
    for (const Vec2<int> u : range(near.dims())) {
      Vec2<int> src;
      for_int(d, 2) src[d] = int(std::floor((u[d] + .5) / near.dim(d) * pix.dim(d)));
      assertx(near[u] == pix[src]);
    }
  }
  {  // The accuracy of repeated resizing (64 -> 65 -> 64) of a smooth field with the cardinal cubic B-spline.
    Matrix<Vector4> matrix(V(64, 64));
    for (const Vec2<int> u : range(matrix.dims()))
      matrix[u] = Vector4(std::sin(u[0] * .3f) * std::cos(u[1] * .2f) * .5f + .5f);
    Matrix<Vector4> omatrix(matrix.dims());
    omatrix.assign(matrix);
    const ResizeOptions<2> options{.filter = "cardinal3", .boundary = "reflect"};
    for_int(i, 10) {
      matrix = resize(matrix, V(65, 65), options, implicit_cast<const Vector4*>(nullptr), std::move(matrix));
      matrix = resize(matrix, V(64, 64), options, implicit_cast<const Vector4*>(nullptr), std::move(matrix));
    }
    float max_error = 0.f;
    double sum2 = 0.;
    for (const Vec2<int> u : range(matrix.dims())) {
      for_int(c, 4) {
        const float e = abs(matrix[u][c] - omatrix[u][c]);
        max_error = max(max_error, e), sum2 += double(e) * e;
      }
    }
    const float rms_error = float(std::sqrt(sum2 / (matrix.size() * 4)));
    SHOW(round_fraction_digits(max_error, 100.f), round_fraction_digits(rms_error, 100.f));
    assertx(max_error < .02f && rms_error < .005f);
  }
  {  // Parity with the Python package resampler.
    for (const std::string_view parity_case : k_parity_cases) check_parity(string(parity_case));
    // Every output sample maps outside the source domain: the result is cval, not NaN.
    check_parity("resize natural cubic dual dual 0.5 2 3 4 = -1 -1 -1 -1");
  }
  {  // The digital filter of a cardinal spline on a primal grid of 2 samples with "wrap" (period 1) is well-posed.
    const Grid<1, double> two(V(2), 1.);
    const ResizeOptions<1> options{.gridtype = Gridtype::primal, .filter = "cardinal3", .boundary = "wrap"};
    const Grid<1, double> result = resize(two, V(5), options);
    SHOW(result);
    for (const double v : result) assertx(abs(v - 1.) < 1e-6);
  }
  {  // uniform_resize() preserves the aspect ratio (the doctests of resampler.uniform_resize).
    const Grid<2, double> ones2(V(2, 2), 1.);
    SHOW(uniform_resize(ones2, V(2, 4), ObjectFit::contain, {.filter = "trapezoid"}));
    const Grid<2, double> ones4(V(4, 8), 1.);
    SHOW(uniform_resize(ones4, V(2, 7), ObjectFit::contain, {.filter = "trapezoid"}));
    Grid<2, double> a(V(2, 3));
    for_int(i, 6) a.flat(i) = double(i);
    const Grid<2, double> covered = uniform_resize(a, V(2, 2), ObjectFit::cover, {.filter = "trapezoid"});
    SHOW(covered);
    assertx(abs(covered[0, 0] - .5) < 1e-6 && abs(covered[1, 1] - 4.5) < 1e-6);
  }
}
