// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Pool.h"

#include <cstdlib>  // malloc(), free()

using namespace hh;

namespace {

int icount;

class A {
 public:
  A() {
    std::cout << "A::A()\n";
    _e = ++icount;
  }
  ~A() { std::cout << "A::~A(" << _e << ")\n"; }
  static void* operator new(size_t s) {
    std::cout << "A::new(" << s << ")\n";
    return malloc(s);
  }
  static void operator delete(void* p, size_t s) {
    assertx(p);
    std::cout << "A::delete(" << s << ")\n";
    free(p);
  }
  int _e;
};

}  // namespace

int main() {
  {
    A* pa = new A[10];
    delete[] pa;  // should call global "delete"!
    pa = nullptr;
    delete[] pa;  // should call nothing!
  }
  {
    SHOW("make_unique");
    auto pa = make_unique<A>();
  }
}
