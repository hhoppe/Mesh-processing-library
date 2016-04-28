// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <cstdio>               // printf()
#include <cstdlib>              // malloc(), free()

#include "Pool.h"
using namespace hh;

#if 0
void operator delete(void* p /* , size_t */) noexcept {
    // p may equal nullptr
    // printf() may call new()/delete() !   printf("global delete - correct\n");
    if (0) std::cerr << "global delete - correct\n";
    dummy_use(p);
    // free(p);
}

void* operator new(size_t s)
{
    // printf() may call new()/delete() !   printf("global new(%d) - correct\n", s);
    // std::cerr and stderr may yet be uninitialized!
    return malloc(s);
}
#endif

namespace {

int icount;

class A {
 public:
    A()                                 { printf("A::A()\n"); e = ++icount; }
    ~A()                                { printf("A::~A(%d)\n", e); }
    static void* operator new(size_t s) {
        long long ls = s;
        printf("A::new(%lld)\n", ls);
        return malloc(s);
    }
    static void operator delete(void* p , size_t s) {
        assertx(p);
        long long ls = s;
        printf("A::delete(%lld)\n", ls);
        free(p);
    }
    int e;
};

} // namespace

int main() {
    {
        A* pa = new A[10];
        delete[] pa;            // should call global "delete"!
        pa = nullptr;
        delete[] pa;            // should call nothing!
    }
    {
        SHOW("make_unique");
        auto pa = make_unique<A>();
    }
}
