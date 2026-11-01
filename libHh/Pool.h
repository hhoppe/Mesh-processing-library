// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_POOL_H_
#define MESH_PROCESSING_LIBHH_POOL_H_

#include <array>
#include <atomic>
#include <mutex>
#include <thread>  // this_thread::yield()
#include <vector>

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

  // Creating and destroying pooled objects (e.g., mesh elements) in several threads.
  {
    const Pool::ThreadSafe pool_thread_safe;  // In the main thread, before the parallel work.
    parallel_for(range(n), [&](const int i) {
      GMesh mesh;  // Its vertices, faces, and edges are allocated from (and returned to) the pools.
      ...;
    });
  }  // The pools leave their thread-safe mode.
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

// Custom memory allocation pool for the objects of a class.  Such a class (e.g., Polygon above, or the mesh elements
// Mesh::MVertex, Mesh::MFace, ...) declares a static Pool using HH_POOL_ALLOCATION(), whose operator new and
// operator delete hand out elements of large chunks and keep a free list of the released elements.
//
// By default, a Pool is not thread-safe.  To create and destroy pooled objects concurrently in several threads, a
// client constructs a Pool::ThreadSafe object (e.g., on the stack) in the main thread before the parallel section,
// such as a parallel_for(), and destroys it after the section (or keeps it across several sections).  While any
// ThreadSafe object exists, every Pool (i.e., the pool of every pooled class) is in a thread-safe mode, in which
// each thread allocates from and frees into its own cache of elements, one per Pool.  A cache exchanges batches of
// elements with its Pool under a lock, which is rarely needed (in Subdivfit, about once per 10'000 allocations).
// When the last ThreadSafe object is destroyed, at which time no other thread may be using a Pool, all the caches
// return their elements to their Pools.
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
    _slot = s_num_pools.fetch_add(1);
    assertx(_slot < k_max_pools);
    s_pools[_slot] = this;
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
  // Constructed by a client around code that creates and destroys pooled objects in several threads (see the
  // overview above).  Outside the thread-safe mode, the only overhead in alloc() and free() is the test of a global
  // atomic counter.
  class ThreadSafe : noncopyable {
   public:
    ThreadSafe() { s_num_thread_safe.fetch_add(1); }
    ~ThreadSafe() {
      if (s_num_thread_safe.fetch_sub(1) == 1) flush_thread_caches();
    }
  };
  // Allocate based on the static size of the class.
  [[nodiscard]] void* alloc() {
    if (!thread_safe()) return alloc_aux();
    Cache& cache = thread_caches().caches[_slot];
    if (!cache.head) {
      const Lock lock(_lock);
      for_int(i, k_cache_batch) cache_push(cache, static_cast<Link*>(alloc_aux()));
    }
    return cache_pop(cache);
  }
  void free(void* pp) {
    if (!thread_safe()) return free_aux(pp);
    if (pp) cache_free(static_cast<Link*>(pp));
  }
  // Allocate based on the size of the first alloc_size() call.
  [[nodiscard]] void* alloc_size(int align, size_t s64) {
    if (!thread_safe()) return alloc_size_aux(align, s64);
    Cache& cache = thread_caches().caches[_slot];
    if (!cache.head) {
      const Lock lock(_lock);
      cache_push(cache, static_cast<Link*>(alloc_size_aux(align, s64)));
      for_int(i, k_cache_batch - 1) cache_push(cache, static_cast<Link*>(alloc_aux()));
    }
    return cache_pop(cache);
  }
  void free_size(void* pp, size_t s) {
    if (!thread_safe()) return free_size_aux(pp, s);
    if (pp) cache_free(static_cast<Link*>(pp));
  }

 private:
  struct Link {
    Link* next;
  };
  static constexpr int k_max_pools = 64;  // Maximum number of pools (one per pooled class).
  // A cache exchanges batches of elements with its pool, and it holds up to about k_cache_max elements, so that a
  // thread that repeatedly creates and destroys a temporary mesh usually reuses its own elements.
  static constexpr int k_cache_batch = 256;
  static constexpr int k_cache_max = 1 << 16;
  struct Cache {
    Link* head;
    int num;
  };
  // The caches of a thread (one per pool).  As they are thread_local, they are also registered, so that the main
  // thread can empty the caches of all threads when the last ThreadSafe object is destroyed (after the parallel work).
  struct ThreadCaches {
    ThreadCaches() {
      const std::scoped_lock lock(registry().mutex);
      registry().all.push_back(this);
    }
    ~ThreadCaches() {
      const std::scoped_lock lock(registry().mutex);
      std::erase(registry().all, this);
    }
    std::array<Cache, k_max_pools> caches{};
  };
  struct Registry {
    std::mutex mutex;
    std::vector<ThreadCaches*> all;  // The caches of all threads, in no particular order (used as a set).
  };
  static Registry& registry() {
    static Registry& value = *new Registry;  // Never destroyed, as threads may exit during static destruction.
    return value;
  }
  static ThreadCaches& thread_caches() {
    thread_local ThreadCaches value;
    return value;
  }
  static void flush_thread_caches() {
    const std::scoped_lock lock(registry().mutex);
    for (ThreadCaches* thread_caches : registry().all) {
      for_int(slot, k_max_pools) {
        Cache& cache = thread_caches->caches[slot];
        while (Link* p = cache.head) {
          cache.head = p->next;
          Pool& pool = *s_pools[slot];
          unpoison(p, pool._esize);  // Because free_aux() poisons the element again.
          pool.free_aux(p);
        }
        cache.num = 0;
      }
    }
  }
  // Add an element to a cache, as an unused element (like those of the free list).
  void cache_push(Cache& cache, Link* p) const {
    p->next = cache.head;
    cache.head = p;
    cache.num++;
    poison(p, _esize);
  }
  void* cache_pop(Cache& cache) const {
    Link* p = cache.head;
    cache.head = p->next;
    cache.num--;
    unpoison(p, _esize);
    return p;
  }
  void cache_free(Link* p) {
    Cache& cache = thread_caches().caches[_slot];
    cache_push(cache, p);
    if (cache.num < k_cache_max + k_cache_batch) return;
    const Lock lock(_lock);
    for_int(i, k_cache_batch) {
      Link* p2 = cache.head;
      cache.head = p2->next;
      cache.num--;
      unpoison(p2, _esize);  // Because free_aux() poisons the element again.
      free_aux(p2);
    }
  }
  static inline std::atomic<int> s_num_thread_safe{0};
  static inline std::atomic<int> s_num_pools{0};
  static inline std::array<Pool*, k_max_pools> s_pools{};
  static bool thread_safe() { return s_num_thread_safe.load(std::memory_order_relaxed) != 0; }
  // A spin lock, as the critical sections are short; it yields the processor if the lock stays busy.
  class Lock : noncopyable {
   public:
    explicit Lock(std::atomic_flag& flag) : _flag(flag) {
      for (int count = 0; _flag.test_and_set(std::memory_order_acquire);)
        while (_flag.test(std::memory_order_relaxed))
          if (++count > 100) std::this_thread::yield();
    }
    ~Lock() { _flag.clear(std::memory_order_release); }

   private:
    std::atomic_flag& _flag;
  };
  std::atomic_flag _lock;  // Clear upon (constant) initialization.
  int _slot;               // Index of the pool in s_pools and in the caches of each thread.
  [[nodiscard]] void* alloc_aux() {
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
  void free_aux(void* pp) {
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
  [[nodiscard]] void* alloc_size_aux(int align, size_t s64) {
    const int s = narrow_cast<int>(s64);
    if (!_h && _unused == _unused_end) grow_size(s, align);
    return alloc_aux();
  }
  void free_size_aux(void* pp, size_t s) {
    // Pool::free(pp);
    if (!pp) return;
    Link* p = static_cast<Link*>(pp);
    p->next = _h;
    _h = p;
    poison(p, s);
    if (!--_nused) reset();
  }
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
