// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_POOL_H_
#define MESH_PROCESSING_LIBHH_POOL_H_

#include "libHh/Hh.h"

#if HH_HAS_ASAN
#include <sanitizer/asan_interface.h>
#endif

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
// An array is allocated by the global allocator, with the alignment of T, which may exceed the default alignment.
#define HH_POOL_ALLOCATION_1(T)                                                                       \
  static void* operator new(size_t s) {                                                               \
    ASSERTX(s == sizeof(T));                                                                          \
    return pool.alloc();                                                                              \
  }                                                                                                   \
  static void* operator new(size_t, void* p) { return p; }                                            \
  static void operator delete(void* p, size_t s) {                                                    \
    ASSERTX(s == sizeof(T));                                                                          \
    pool.free(p);                                                                                     \
  }                                                                                                   \
  static void* operator new[](size_t s) { return ::operator new[](s, std::align_val_t{alignof(T)}); } \
  static void operator delete[](void* p, size_t) { ::operator delete[](p, std::align_val_t{alignof(T)}); }

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
  Pool() = default;   // This constructor must be a no-op as it may be called after construct() has been called!
  ~Pool() = default;  // Do nothing here; wait for other destruction means.
  HH_ATTRIBUTE_NO_SANITIZE_ADDRESS void construct(const char* name, unsigned esize, int ealign) {
    if (1) {
      // Initialized to zero by static initialization.
      assertx(!_name && !_esize && !_ealign && !_h && !_unused && !_unused_end && !_nalloc && !_nused && !_chunkh &&
              !_reuse);
    }
    _name = assertx(name);
    _esize = esize;
    _ealign = ealign;
    _h = nullptr;
    _unused = nullptr;
    _unused_end = nullptr;
    _nalloc = 0;
    _nused = 0;
    _chunkh = nullptr;
    _reuse = nullptr;
    // The static variable sdebug may not yet be initialized.
    if (getenv_int("POOL_DEBUG") >= 2) showf("Pool %-20s: construct (size=%2u, align=%2d)\n", _name, _esize, _ealign);
    if (_esize) init();
  }
  void destroy() {
    assertx(_name);
    if (sdebug >= 2 || (sdebug && _nalloc) || _nused)
      showf("Pool %-20s: (size %2u) %6d/%-6d elements outstanding%s\n",  //
            _name, _esize, _nused, _nalloc, (_nused ? " **" : ""));
    if (_nused) return;
    for (Chunk* chunk = _chunkh; chunk;) {
      Chunk* next = chunk->next;
      unpoison(chunk, k_chunksize);  // Clear any user poisoning before returning the memory.
      aligned_free(chunk);
      chunk = next;
    }
    _name = nullptr;
    _esize = 0;
    _ealign = 0;
    _h = nullptr;
    _unused = nullptr;
    _unused_end = nullptr;
    _nalloc = 0;
    _nused = 0;
    _chunkh = nullptr;
    _reuse = nullptr;
    _offset = 0;
  }
  // Allocate based on the static size of the class.
  [[nodiscard]] void* alloc() {
    Link* p = _h;
    if (p) {
      _h = p->next;
    } else {  // Take the next never-used element of the current chunk.
      if (_unused == _unused_end) grow();
      p = reinterpret_cast<Link*>(_unused);
      _unused += _esize;
    }
    _nused++;
    unpoison(p, _esize);
    return p;
  }
  void free(void* pp) {
    if (!pp) return;
    Link* p = static_cast<Link*>(pp);
    // The analyzer conflates the pointer and size parameters of the class-level sized operator delete(),
    // yielding a bogus pointer value of sizeof(T).
    // NOLINTNEXTLINE(clang-analyzer-core.FixedAddressDereference,clang-analyzer-optin.core.FixedAddressDereference)
    p->next = _h;
    _h = p;
    poison(p, _esize);
    if (!--_nused) reset();
  }
  // Allocate based on the size of the first alloc_size() call.
  [[nodiscard]] void* alloc_size(int align, size_t s64) {
    const int s = narrow_cast<int>(s64);
    if (!_h && _unused == _unused_end) grow_size(s, align);
    return alloc();
  }
  void free_size(void* pp, size_t s) {
    // Pool::free(pp);
    if (!pp) return;
    Link* p = static_cast<Link*>(pp);
    p->next = _h;
    _h = p;
    poison(p, s);
    if (!--_nused) reset();
  }

 private:
  // In debug builds, overwrite a freed element (beyond its free-list link) with a pattern, so that a later access
  // to the destroyed element fails loudly rather than silently reading its stale contents.  For instance, the
  // std::hash of a mesh element reads its id (see Mesh.h), so a hash container that still holds a destroyed
  // element as a key would otherwise keep working until the pool reuses the memory.  The pattern makes such an id
  // a large negative value and such a pointer a non-canonical address.
  // Under AddressSanitizer, which cannot otherwise see that a pool element is freed, the same region is also
  // marked as poisoned, so that any access to it is reported immediately with a stack trace.
  static void poison(void* p, size_t size) {
    std::byte* const after_link = static_cast<std::byte*>(p) + sizeof(Link);
    if constexpr (k_debug) std::fill_n(after_link, size - sizeof(Link), std::byte{0xDB});
#if HH_HAS_ASAN
    ASAN_POISON_MEMORY_REGION(after_link, size - sizeof(Link));
#endif
    dummy_use(after_link, size);
  }
  static void unpoison(void* p, size_t size) {
#if HH_HAS_ASAN
    ASAN_UNPOISON_MEMORY_REGION(p, size);
#endif
    dummy_use(p, size);
  }
  // Large chunks keep a pool's elements contiguous in memory; going from 16 KiB to 256 KiB chunks made Filtermesh on a
  // 1M-face mesh 0-3% faster (WSL clang++ 21.1 -O3, and Windows MSVC 14.44 -O2).
  // A chunk's elements are handed out only as needed, so its untouched pages cost no physical memory.
  static constexpr int k_heap_block_size = 256 * 1024;
  // Bytes left for the allocator's own bookkeeping so that a chunk occupies no more than k_heap_block_size bytes.
  // glibc serves such a large block with mmap() (until its dynamic threshold rises), adding a 16-byte header and
  // rounding up to whole pages; the Windows _aligned_malloc() adds alignment + 8 bytes to a 16-byte heap header.
  static constexpr int k_malloc_overhead = 64;
  static constexpr int k_chunksize = k_heap_block_size - k_malloc_overhead;
  const int sdebug = getenv_int("POOL_DEBUG");  // 0, 1, 2, or 3; may be uninitialized in constructor() and init().
  struct Link {
    Link* next;
  };
  struct Chunk {
    Chunk* next;
  };
  unsigned _esize;
  int _ealign;
  const char* _name;     // Not "string" because construct() may be called before constructor!
  Link* _h;              // Free list of released elements.
  uint8_t* _unused;      // Start of the never-used elements in the current chunk.
  uint8_t* _unused_end;  // End of those elements.
  Chunk* _chunkh;        // All chunks, most recently allocated first.
  Chunk* _reuse;         // After a reset(), the next chunk of _chunkh whose elements are handed out again.
  int _nalloc;           // Number of elements in all chunks.
  int _nused;            // Number of elements currently allocated.
  int _offset;           // Offset of the first element within a chunk, after the Chunk link.

  HH_ATTRIBUTE_NO_SANITIZE_ADDRESS void init() {
    // Make the allocated size a multiple of sizeof(Link)!
    _esize = ((_esize + sizeof(Link) - 1) / sizeof(Link)) * sizeof(Link);
    assertx(_esize >= sizeof(Link) && (_esize % sizeof(Link)) == 0);
    // Make the allocated size a multiple of _ealign.
    _esize = ((_esize + _ealign - 1) / _ealign) * _ealign;
    assertx(_esize >= unsigned(_ealign) && (_esize % _ealign) == 0);
    _offset = ((int(sizeof(Chunk)) + _ealign - 1) / _ealign) * _ealign;
    // The static variable sdebug may not yet be initialized.
    if (getenv_int("POOL_DEBUG") >= 2)
      showf("Pool %-20s: _esize=%u _ealign=%d _offset=%d\n", _name, _esize, _ealign, _offset);
  }
  void grow() {
    assertx(!_h && _unused == _unused_end);
    assertx(_esize);
    assertx((_esize % _ealign) == 0);
    const int nelem = (k_chunksize - _offset) / _esize;
    assertx(nelem > 0);
    if (_reuse) {
      Chunk* chunk = _reuse;
      _reuse = chunk->next;
      mark_unused(chunk);
      return;
    }
    if (sdebug >= 3) showf("Pool %-20s: allocating new chunk\n", _name);
    void* p = assertx(aligned_malloc(_ealign, k_chunksize));
    assertx((reinterpret_cast<uintptr_t>(p) % _ealign) == 0);
    Chunk* chunk = static_cast<Chunk*>(p);
    chunk->next = _chunkh;
    _chunkh = chunk;
    _nalloc += nelem;
    mark_unused(chunk);
  }
  // Make alloc() hand out the elements of this chunk, in address order, by setting the never-used range
  // [_unused, _unused_end) to all of them.
  void mark_unused(Chunk* chunk) {
    _unused = reinterpret_cast<uint8_t*>(chunk) + _offset;
    _unused_end = _unused + size_t((k_chunksize - _offset) / _esize) * _esize;
#if HH_HAS_ASAN
    ASAN_POISON_MEMORY_REGION(_unused, _unused_end - _unused);  // Report any access to a not-yet-allocated element.
#endif
  }
  // When no element remains allocated, discard the free list, whose order has become scattered, and hand out the
  // elements of the existing chunks again in address order, which gives later allocations better memory locality.
  void reset() {
    _h = nullptr;
    _reuse = _chunkh->next;
    mark_unused(_chunkh);
  }
  void grow_size(int size, int align) {
    assertx(!_h && _unused == _unused_end);
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
