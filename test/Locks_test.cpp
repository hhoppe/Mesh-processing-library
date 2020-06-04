// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Locks.h"

#include "libHh/Parallel.h"
#include "libHh/RangeOp.h"  // sum()
#include "libHh/Stat.h"
#include "libHh/Vec.h"

#include <future>
#include <vector>

using namespace hh;

// Occasionally, this program stops responding, especially in CONFIG=mingw32.

int main() {
  if (1) {
    int process_id = -1;
    int counter = 0;
    const int nprocess = 4;
    const int niter = 10;
    Vec<int, nprocess> ar_int;
    auto func_process = [&](int i) {
      int vsum = 0;
      for_int(iter, 10) {
        my_sleep(.0001);
        HH_LOCK {
          assertx(process_id < 0);
          process_id = i;
          my_sleep(.0001);
          process_id = -1;
          vsum += counter;
          counter++;
        }
        my_sleep(.00001 * ((i * 97 + iter) % 7));  // delay some variable amount
      }
      ar_int[i] = vsum;
      return vsum;
    };
    std::vector<std::future<int>> results;
    for_int(i, nprocess) results.push_back(std::async(func_process, i));  // launch asynchronous tasks
    int vsum = 0;
    for_int(i, nprocess) vsum += results[i].get();  // wait for them in the original order
    if (0) {
      SHOW(counter, niter * nprocess, vsum, (niter * nprocess) * (niter * nprocess + 1) / 2);
      SHOW(Stat(ar_int));
    }
    assertx(counter == niter * nprocess);
    assertx(vsum == (niter * nprocess) * (niter * nprocess - 1) / 2);
  }
}
