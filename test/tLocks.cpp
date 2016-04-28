// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Parallel.h"
#include "Locks.h"
#include "Vec.h"
#include "RangeOp.h"            // sum()
#include "Stat.h"

#include <vector>
#include <future>

using namespace hh;

// Occasionally, this program stops responding, especially in CONFIG=mingw32.

int main() {
#if !defined(__clang__)        // mingw target used by my version of clang does not support <mutex> / <future>
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
                    assertx(process_id<0);
                    process_id = i;
                    my_sleep(.0001);
                    process_id = -1;
                    vsum += counter;
                    counter++;
                }
                my_sleep(.00001*((i*97+iter)%7)); // delay some variable amount
            }
            ar_int[i] = vsum;
            return vsum;
        };
        std::vector<std::future<int>> results;
        for_int(i, nprocess) results.push_back(std::async(func_process, i)); // launch asynchronous tasks
        int vsum = 0;
        for_int(i, nprocess) vsum += results[i].get(); // wait for them in the original order
        if (0) {
            SHOW(counter, niter*nprocess, vsum, (niter*nprocess)*(niter*nprocess+1)/2);
            SHOW(Stat(ar_int));
        }
        assertx(counter==niter*nprocess);
        assertx(vsum==(niter*nprocess)*(niter*nprocess-1)/2);
    }
#endif
}
