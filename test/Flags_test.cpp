// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Flags.h"
using namespace hh;

int main() {
  int counter = 0;
  const FlagMask fa = Flags::allocate(counter), fb = Flags::allocate(counter), fc = Flags::allocate(counter);
  SHOW(counter, fa, fb, fc);
  Flags flags;
  assertx(FlagMask(flags) == 0);
  assertx(!flags.flag(fa));
  flags.flag(fa) = true;
  flags.flag(fc) = true;
  SHOW(FlagMask(flags), bool(flags.flag(fa)), bool(flags.flag(fb)), bool(flags.flag(fc)));
  // Each set() returns the previous value.  (Separate statements ensure the order of evaluation.)
  const bool previous1 = flags.flag(fb).set(true);
  const bool previous2 = flags.flag(fb).set(false);
  SHOW(previous1, previous2);
  flags.flag(fb) = flags.flag(fa);  // Assigning from another flag copies its value.
  assertx(flags.flag(fb));
  flags.flag(fa) = false;
  assertx(!flags.flag(fa) && flags.flag(fb) && flags.flag(fc));
  const FlagMask previous = flags.set(fa);  // Returns the previous value of all the flags.
  SHOW(previous, FlagMask(flags));
  {
    const Flags& cflags = flags;
    assertx(cflags.flag(fa) && !cflags.flag(fb) && !cflags.flag(fc));
  }
  Flags flags2;
  flags2 = fb | fc;
  swap(flags, flags2);
  SHOW(FlagMask(flags), FlagMask(flags2));
  // All the flags of the mask type can be allocated.
  while (counter < std::numeric_limits<FlagMask>::digits) dummy_use(Flags::allocate(counter));
  SHOW(counter);
}
