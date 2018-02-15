// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "AtomicOperate.h"
#include "Parallel.h"
#include "Array.h"
#include "Timer.h"

using namespace hh;

int main() {
#if 1
    {
        const int num = 100;
        const int niter = 1*1000*1000;
        auto func_hash = [=](int i) { return narrow_cast<int>((int64_t{i}*97)%num); };
        auto func_oper = [](float v) { return v*2.f+1.f; };
        Array<float> ar1(num, 1.f); {
            // HH_TIMER(_ar1);
            parallel_for_each(range(niter), [&](const int iter) {
                atomic_operate(&ar1[func_hash(iter)], func_oper);
            });
        }
        Array<float> ar2(num, 1.f); {
            // HH_TIMER(_ar2);
            for_int(iter, niter) {
                int i = func_hash(iter);
                ar2[i] = func_oper(ar2[i]);
            }
        }
        assertx(ar1==ar2);
    }
    {
        const int num = 100;
        const int niter = 1*1000*1000;
        auto func_hash = [=](int i) { return narrow_cast<int>((int64_t{i}*97)%num); };
        auto func_oper = [](uchar v) { return uchar((v*97+31)%256); };
        Array<uchar> ar1(num, uchar{1}); {
            // HH_TIMER(_ar1);
            parallel_for_each(range(niter), [&](const int iter) {
                atomic_operate(&ar1[func_hash(iter)], func_oper);
            });
        }
        Array<uchar> ar2(num, uchar{1}); {
            // HH_TIMER(_ar2);
            for_int(iter, niter) {
                int i = func_hash(iter);
                ar2[i] = func_oper(ar2[i]);
            }
        }
        assertx(ar1==ar2);
    }
#endif
}
