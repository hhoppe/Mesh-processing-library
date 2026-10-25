// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Gamma.h"
#include "libHh/GridOp.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/ResampleOptions.h"
using namespace hh;

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
  {  // The option-based scale() and sample_domain() agree with the FilterBnd-based ones.
    Random random(17);
    Matrix<float> mat(5, 4);
    for (float& e : mat) e = random.unif();
    const float cval = .25f;
    for (const auto& [filter_name, boundary_name] :
         {std::pair{"triangle", "reflect"}, std::pair{"cubic", "clamp"}, std::pair{"mitchell", "wrap"},
          std::pair{"cardinal3", "reflect"}, std::pair{"box", "border"}, std::pair{"impulse", "clamp"}}) {
      const auto filterbs = twice(FilterBnd(Filter::get(filter_name), parse_boundaryrule(boundary_name)));
      const ResizeOptions<2> options{.filter = filter_name, .boundary = boundary_name};
      for (const Vec2<int> ndims : {V(5, 4), V(9, 11), V(5, 13), V(3, 2)}) {
        const Grid<2, float> expected = scale(mat, ndims, filterbs, &cval);
        const Grid<2, float> result = scale(mat, ndims, options, &cval);
        assertx(ranges::equal(result, expected));
      }
      if (Filter::get(filter_name).requires_digital_filter() || Filter::get(filter_name).is_impulse()) continue;
      for_int(i, 5) {
        const Vec2<float> p(random.unif() * 1.4f - .2f, random.unif() * 1.4f - .2f);
        assertx(sample_domain(mat, p, filterbs, &cval) == sample_domain(mat, p, options.sampling(), &cval));
        const Vec2<float> pg(p[0] * 5.f, p[1] * 4.f);
        assertx(sample_grid(mat, pg, filterbs, &cval) == sample_grid(mat, pg, options.sampling(), &cval));
      }
    }
    {  // A primal grid.
      Grid<1, float> ramp(V(5));
      for_int(i, 5) ramp[i] = float(i * i);
      const Grid<1, float> expected = scale_primal(ramp, V(9), V(FilterBnd(Filter::get("triangle"), Bndrule::clamp)));
      const ResizeOptions<1> options{.gridtype = Gridtype::primal, .filter = "triangle", .boundary = "clamp"};
      assertx(ranges::equal(scale(ramp, V(9), options), expected));
      SHOW(scale(ramp, V(9), options));
    }
  }
}
