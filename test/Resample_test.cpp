// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Gamma.h"
#include "libHh/RangeOp.h"
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
}
