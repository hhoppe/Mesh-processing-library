// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Pool.h"

#include <cstdlib>  // malloc(), free()

#include "libHh/Array.h"
#include "libHh/RangeOp.h"
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

// The pooled classes lie outside the anonymous namespace, like those in the library, so that the unused placement
// operator new() declared by HH_POOL_ALLOCATION() does not cause a warning.

// A class whose instances are allocated from a Pool.
struct P {  // NOLINT(misc-use-internal-linkage)
  explicit P(int i) : _i(i), _d(i * .5) {}
  int _i;
  double _d;
  HH_POOL_ALLOCATION(P);
};
HH_INITIALIZE_POOL(P);
HH_ALLOCATE_POOL(P);

// A pooled class with an alignment larger than that of its members.
struct alignas(32) Q {  // NOLINT(misc-use-internal-linkage)
  char _c{'q'};
  HH_POOL_ALLOCATION(Q);
};
HH_INITIALIZE_POOL(Q);
HH_ALLOCATE_POOL(Q);

namespace {

// A Pool used directly, sized by its first alloc_size() call; it must have static storage duration, because
// construct() expects its members to be zero-initialized.
Pool g_pool;

bool is_aligned(const void* p, size_t alignment) { return reinterpret_cast<uintptr_t>(p) % alignment == 0; }

}  // namespace

int main() {
  {
    A* pa = new A[10];
    delete[] pa;  // Should call global "delete"!
    pa = nullptr;
    delete[] pa;  // Should call nothing!
  }
  {
    SHOW("make_unique");
    auto pa = make_unique<A>();  // NOLINT(clang-analyzer-unix.Malloc)
  }
  {
    static_assert(!std::is_copy_constructible_v<Pool> && !std::is_copy_assignable_v<Pool>);
    // Allocate enough elements to span several chunks of the pool.
    constexpr int n = 3000;
    Array<P*> ps(n);
    for_int(i, n) ps[i] = new P(i);
    for_int(i, n) assertx(ps[i]->_i == i && ps[i]->_d == i * .5 && is_aligned(ps[i], alignof(P)));
    // The elements are distinct and do not overlap.
    Array<uintptr_t> addresses(n);
    for_int(i, n) addresses[i] = reinterpret_cast<uintptr_t>(ps[i]);
    sort(addresses);
    for_intL(i, 1, n) assertx(addresses[i] - addresses[i - 1] >= sizeof(P));
    // The free list is last-in first-out, so a freed element is reused by the next allocation.
    const uintptr_t freed = reinterpret_cast<uintptr_t>(ps[1234]);
    delete ps[1234];
    ps[1234] = new P(-1);
    assertx(reinterpret_cast<uintptr_t>(ps[1234]) == freed && ps[1234]->_i == -1);
    for (P* p : ps) delete p;         // Return all elements, so that the pool reports no outstanding elements at exit.
    delete static_cast<P*>(nullptr);  // Deleting a null pointer is a no-op.
    // A unique_ptr also uses the class-specific operator new and operator delete.
    const auto up = make_unique<P>(7);
    SHOW(up->_i, up->_d);
  }
  {
    // An array of pooled objects uses the global allocator.
    P* pa = new P[3]{P(1), P(2), P(3)};
    SHOW(pa[2]._i);
    delete[] pa;
  }
  {
    // The pool respects an alignment larger than that of the members.
    static_assert(alignof(Q) == 32 && sizeof(Q) == 32);
    Array<Q*> qs(100);
    for (Q*& q : qs) q = new Q;
    for (Q* q : qs) assertx(is_aligned(q, 32) && q->_c == 'q');
    for (Q* q : qs) delete q;
    alignas(Q) uint8_t buffer[sizeof(Q)];
    Q* q = new (buffer) Q;
    assertx(q->_c == 'q');
    q->~Q();
  }
  if (0) {
    // KNOWN_BUG: an array of an over-aligned pooled class is misaligned, because the operator new[] defined by
    // HH_POOL_ALLOCATION() calls ::operator new(size_t), which ignores the alignment.
    Q* qa = new Q[2];
    assertx(is_aligned(qa, 32));
    delete[] qa;
  }
  {
    // Direct use of a Pool whose element size is set by the first alloc_size() call, as in HH_MAKE_POOLED_SAC().
    g_pool.construct("g_pool", 0, 0);
    constexpr int n = 2000;
    constexpr size_t size = 40;
    Array<uint8_t*> elems(n);
    for_int(i, n) {
      elems[i] = static_cast<uint8_t*>(g_pool.alloc_size(16, size));
      assertx(is_aligned(elems[i], 16));
      std::fill_n(elems[i], size, uint8_t(i));
    }
    for_int(i, n) assertx(std::all_of(elems[i], elems[i] + size, [&](uint8_t v) { return v == uint8_t(i); }));
    for (uint8_t* elem : elems) g_pool.free_size(elem, size);
    g_pool.free_size(nullptr, size);  // Freeing a null pointer is a no-op.
    g_pool.destroy();                 // It prints nothing because no elements are outstanding.
  }
}  // NOLINT(clang-analyzer-unix.Malloc)
