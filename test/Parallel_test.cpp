// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Parallel.h"

#include <latch>
#include <list>
#include <thread>

#include "libHh/RangeOp.h"
#include "libHh/Set.h"
#include "libHh/Vec.h"
using namespace hh;

int main() {
  assertx(get_max_threads() >= 1);
  {
    const Array array(range(1000));
    SHOW(sum(array));
  }
  {
    Array array(range(1000));
    parallel_for(array, [&](int& i) { i += 5; });
    SHOW(sum(array));
  }
  {
    Array array(range(1000));
    parallel_for({.cycles_per_elem = 1}, array, [&](int& i) { i += 5; });
    SHOW(sum(array));
  }
  {
    Array array(range(1000));
    const int num_threads = get_max_threads();
    Array<int> sums(num_threads);
    parallel_for_chunk(array, num_threads, [&](const int thread_index, auto subrange) {  //
      sums[thread_index] = sum<int>(subrange);
    });
    int result = sum<int>(sums);
    SHOW(result);
  }
  {
    const int num_threads = get_max_threads();
    Array<std::thread::id> thread_ids(num_threads);
    std::atomic<int64_t> count{0};
    std::latch latch(num_threads);
    parallel_for_chunk(range(1000), num_threads, [&](const int thread_index, auto subrange) {
      if (0) SHOW(thread_index, std::this_thread::get_id(), *subrange.begin(), *(subrange.end() - 1));
      count += ranges::distance(subrange);
      thread_ids[thread_index] = std::this_thread::get_id();
      latch.arrive_and_wait();  // Prevent any thread from completing its chunk and claiming a second chunk.
    });
    SHOW(count);
    const Set<std::thread::id> unique_ids(thread_ids);
    if (0) SHOW(num_threads, unique_ids.num(), unique_ids);
    assertx(unique_ids.num() == num_threads);  // Parallelism actually occurred (if num_threads > 1).
  }
  {
    // An empty range invokes nothing.
    int num_calls = 0;
    parallel_for(Array<int>{}, [&](int) { num_calls++; });
    parallel_for(range(0), [&](int) { num_calls++; });
    assertx(num_calls == 0);
  }
  {
    // Each element is visited exactly once, for sizes smaller and larger than the number of threads.
    for (const int n : {1, 2, 3, 7, 64, 1000, 1001}) {
      Array<int> visits(n, 0);
      parallel_for(visits, [&](int& visit) { visit++; });
      assertx(ranges::all_of(visits, [](int visit) { return visit == 1; }));
      Array<int> visits2(n, 0);
      parallel_for(range(n), [&](const int i) { visits2[i]++; });
      assertx(visits2 == visits);
    }
  }
  {
    // The chunks are consecutive subranges of nearly equal size, possibly empty if there are more chunks than
    // elements, and each chunk is processed exactly once.
    for (const int n : {0, 1, 3, 10, 1000}) {
      for (const int num_chunks : {1, 2, 3, 5, 16}) {
        const auto r = range(n);
        Array<Vec2<int>> chunks(num_chunks, V(-1, -1));
        parallel_for_chunk(r, num_chunks, [&](const int thread_index, auto subrange) {
          assertx(chunks[thread_index] == V(-1, -1));
          chunks[thread_index] = V(int(subrange.begin() - r.begin()), int(subrange.end() - r.begin()));
        });
        if (n == 0) {
          // An empty range is processed sequentially as a single empty chunk.
          assertx(chunks[0] == V(0, 0));
          for_intL(i, 1, num_chunks) assertx(chunks[i] == V(-1, -1));
          continue;
        }
        const int chunk_size = (n + num_chunks - 1) / num_chunks;
        int expected_begin = 0;
        for (const Vec2<int>& chunk : chunks) {
          assertx(chunk == V(expected_begin, min(expected_begin + chunk_size, n)));
          expected_begin = chunk[1];
        }
        assertx(expected_begin == n);
      }
    }
  }
  {
    // A loop estimated to be cheap is run sequentially as a single chunk in the calling thread.
    const std::thread::id main_id = std::this_thread::get_id();
    int num_calls = 0;
    parallel_for_chunk({.cycles_per_elem = 1}, range(1000), 4, [&](const int thread_index, auto subrange) {
      num_calls++;
      assertx(thread_index == 0 && ranges::distance(subrange) == 1000);
      assertx(std::this_thread::get_id() == main_id);
    });
    assertx(num_calls == 1);
    parallel_for({.cycles_per_elem = 1}, range(10), [&](int) { assertx(std::this_thread::get_id() == main_id); });
  }
  {
    // A sized forward range that is not random-access, here modified in place.
    std::list<int> list;
    for_int(i, 100) list.push_back(i);
    parallel_for(list, [&](int& e) { e *= 2; });
    SHOW(sum(list));
    Array<int> sums(3, 0);
    parallel_for_chunk(list, 3,
                       [&](const int thread_index, auto subrange) { sums[thread_index] = sum<int>(subrange); });
    SHOW(sums);
    // The overload without thread_index uses get_max_threads() chunks.
    std::atomic<int> total{0};
    parallel_for_chunk(list, [&](auto subrange) { total += sum<int>(subrange); });
    assertx(total == 9900);
  }
  {
    // A nested parallel loop runs serially within its outer iteration (without a data race on the state of the
    // thread pool, as checked by ThreadSanitizer).
    std::atomic<int> count{0};
    parallel_for(range(64), [&](int) {
      const std::thread::id id = std::this_thread::get_id();
      parallel_for(range(64), [&](int) {
        assertx(std::this_thread::get_id() == id);
        count++;
      });
    });
    assertx(count == 64 * 64);
  }
}
