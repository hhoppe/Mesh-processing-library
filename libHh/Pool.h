// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_POOL_H_
#define MESH_PROCESSING_LIBHH_POOL_H_

#include "libHh/Hh.h"

#if 0
{
  // *.h
  class Polygon {
    ...;
    HH_POOL_ALLOCATION(Polygon);
  };
  HH_INITIALIZE_POOL(Polygon);

  // *.cpp
  HH_ALLOCATE_POOL(Polygon);
}
#endif

namespace hh {

// See also Sac.h which defines HH_MAKE_POOLED_SAC()!

#define HH_POOL_ALLOCATION(T) \
  HH_POOL_ALLOCATION_1(T)     \
  HH_POOL_ALLOCATION_2(T)

// Reason to check size_t in new and delete: class may be derived and may not provide its own new/delete operators.
#define HH_POOL_ALLOCATION_1(T)                                       \
  static void* operator new(size_t s) {                               \
    ASSERTX(s == sizeof(T));                                          \
    return pool.alloc();                                              \
  }                                                                   \
  static void* operator new(size_t, void* p) { return p; }            \
  static void operator delete(void* p, size_t s) {                    \
    ASSERTX(s == sizeof(T));                                          \
    pool.free(p);                                                     \
  }                                                                   \
  static void* operator new[](size_t s) { return ::operator new(s); } \
  static void operator delete[](void* p, size_t) { ::operator delete(p); }

#define HH_POOL_ALLOCATION_2(T)                                \
  static hh::Pool pool;                                        \
  struct PoolInit {                                            \
    PoolInit() {                                               \
      if (!count++) pool.construct(#T, sizeof(T), alignof(T)); \
    }                                                          \
    ~PoolInit() {                                              \
      if (!--count) pool.destroy();                            \
    }                                                          \
    static int count;                                          \
  }

#define HH_POOL_ALLOCATION_3(T)               \
  static hh::Pool pool;                       \
  struct PoolInit {                           \
    PoolInit() {                              \
      if (!count++) pool.construct(#T, 0, 0); \
    }                                         \
    ~PoolInit() {                             \
      if (!--count) pool.destroy();           \
    }                                         \
    static int count;                         \
  }

#define HH_INITIALIZE_POOL(T) HH_INITIALIZE_POOL_NESTED(T, T)

#define HH_INITIALIZE_POOL_NESTED(T, name) static T::PoolInit pool_init_##name

#define HH_ALLOCATE_POOL(T) \
  hh::Pool T::pool;         \
  int T::PoolInit::count

#if defined(__clang__) || defined(__GNUC__)
#define HH_ATTRIBUTE_NO_SANITIZE_ADDRESS __attribute__((no_sanitize_address))
#else
#define HH_ATTRIBUTE_NO_SANITIZE_ADDRESS
#endif

//----------------------------------------------------------------------------

// Custom memory allocation pool for a class of objects.
class Pool : noncopyable {
 public:
  Pool() {
    // this constructor must be a no-op as it may be called after construct() has been called!
  }
  ~Pool() {
    // do nothing here, wait for other destruction means
  }
  HH_ATTRIBUTE_NO_SANITIZE_ADDRESS void construct(const char* name, unsigned esize, int ealign) {
    if (1) {
      // initialized to zero by static initialization
      assertx(!_name && !_esize && !_ealign && !_h && !_nalloc && !_chunkh);
    }
    _name = assertx(name);
    _esize = esize;
    _ealign = ealign;
    _h = nullptr;
    _nalloc = 0;
    _chunkh = nullptr;
    // static variable sdebug may not yet be initialized.
    if (getenv_int("POOL_DEBUG") >= 2) showf("Pool %-20s: construct (size=%2d, align=%2d)\n", _name, _esize, _ealign);
    if (_esize) init();
  }
  void destroy() {
    assertx(_name);
    int n = 0;
    for (Link* p = _h; p; p = p->next) n++;
    if (sdebug >= 2 || (sdebug && _nalloc) || n != _nalloc)
      showf("Pool %-20s: (size %2d) %6d/%-6d elements outstanding%s\n",  //
            _name, _esize, _nalloc - n, _nalloc, (n != _nalloc ? " **" : ""));
    if (n != _nalloc) return;
    for (Chunk* chunk = _chunkh; chunk;) {
      uint8_t* p = reinterpret_cast<uint8_t*>(chunk);
      chunk = chunk->next;
      aligned_free(p - _offset);
    }
    _name = nullptr;
    _esize = 0;
    _h = nullptr;
    _nalloc = 0;
    _chunkh = nullptr;
  }
  // allocate based on static size of class
  void* alloc() {
    if (!_h) grow();
    Link* p = _h;
    _h = p->next;
    return p;
  }
  void free(void* pp) {
    if (!pp) return;
    Link* p = static_cast<Link*>(pp);
    p->next = _h;
    _h = p;
  }
  // allocate based on size of first alloc_size() call
  void* alloc_size(int align, size_t s64) {
    int s = narrow_cast<int>(s64);
    if (!_h) grow_size(s, align);
    Link* p = _h;
    _h = p->next;
    return p;
  }
  void free_size(void* pp, size_t s) {
    dummy_use(s);
    // Pool::free(pp);
    if (!pp) return;
    Link* p = static_cast<Link*>(pp);
    p->next = _h;
    _h = p;
  }

 private:
  static constexpr int k_pagesize = 16 * 1024;  // could refer to getpagesize();
  static constexpr int k_malloc_overhead = 64;  // high just to be safe, multiple of 16; was 32
  static constexpr int k_chunksize = k_pagesize - k_malloc_overhead;
  const int sdebug = getenv_int("POOL_DEBUG");  // 0, 1, 2, or 3; may be uninitialized in constructor() and init()
  struct Link {
    Link* next;
  };
  struct Chunk {
    Chunk* next;
  };
  unsigned _esize;
  int _ealign;
  const char* _name;  // not "string" because construct() may be called before constructor!
  Link* _h;
  Chunk* _chunkh;
  int _nalloc;
  int _offset;

  HH_ATTRIBUTE_NO_SANITIZE_ADDRESS void init() {
    // make allocated size a multiple of sizeof(Link)!
    _esize = ((_esize + sizeof(Link) - 1) / sizeof(Link)) * sizeof(Link);
    assertx(_esize >= sizeof(Link) && (_esize % sizeof(Link)) == 0);
    // make allocated size a multiple of _ealign
    _esize = ((_esize + _ealign - 1) / _ealign) * _ealign;
    assertx(_esize >= unsigned(_ealign) && (_esize % _ealign) == 0);
    _offset = k_chunksize - sizeof(Chunk);
    // static variable sdebug may not yet be initialized.
    if (getenv_int("POOL_DEBUG") >= 2)
      showf("Pool %-20s: _esize=%d _ealign=%d _offset=%d\n", _name, _esize, _ealign, _offset);
  }
  void grow() {
    assertx(!_h);
    assertx(_esize);
    assertx((_esize % _ealign) == 0);
    const int nelem = _offset / _esize;
    assertx(nelem > 0);
    if (sdebug >= 3) showf("Pool %-20s: allocating new chunk\n", _name);
    char* p = static_cast<char*>(assertx(aligned_malloc(_ealign, k_chunksize)));
    Chunk* chunk = reinterpret_cast<Chunk*>(p + _offset);
    chunk->next = _chunkh;
    _chunkh = chunk;
    _h = reinterpret_cast<Link*>(p);
    assertx((reinterpret_cast<uintptr_t>(p) % _ealign) == 0);
    char* l = p + (nelem - 1) * _esize;
    for (; p < l; p += _esize) reinterpret_cast<Link*>(p)->next = reinterpret_cast<Link*>(p + _esize);
    reinterpret_cast<Link*>(p)->next = nullptr;
    _nalloc += nelem;
  }
  void grow_size(int size, int align) {
    assertx(!_h);
    if (!_esize) {
      _esize = unsigned(size);
      _ealign = align;
      init();
    }
    grow();
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_POOL_H_
