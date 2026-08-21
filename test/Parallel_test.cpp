// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Parallel.h"

#include <latch>
#include <thread>

#include "libHh/RangeOp.h"
#include "libHh/Set.h"
using namespace hh;

int main() {
  {
    Array array(range(1000));
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
      latch.arrive_and_wait();  // Prevent any thread from complete its chunk and claiming a second chunk.
    });
    SHOW(count);
    Set<std::thread::id> unique_ids(thread_ids);
    if (0) SHOW(num_threads, unique_ids.num(), unique_ids);
    assertx(unique_ids.num() >= 2);  // Parallelism actually occurred.
    assertx(unique_ids.num() == num_threads);
  }
}
