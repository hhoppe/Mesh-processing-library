// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PARALLEL_H_
#define MESH_PROCESSING_LIBHH_PARALLEL_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "libHh/Hh.h"

#if 0
{
  // To disable parallelism, set: OMP_NUM_THREADS=1  (admittedly a confusing variable because we do not use OpenMP)
  parallel_for(range(n), [&](const int i) { func(i); });
}
#endif

namespace hh {

inline int get_max_threads() {
  constexpr int k_uninitialized = -99;
  static int s_value = k_uninitialized;
  if (s_value == k_uninitialized) {
    int default_value = max(int(std::thread::hardware_concurrency()), 1);
    s_value = getenv_int("OMP_NUM_THREADS", default_value);
  }
  return s_value;
}

constexpr uint64_t k_parallel_thresh = 100'000;  // Number of instruction cycles above which to parallelize a loop.
constexpr uint64_t k_parallel_many_cycles_per_elem = 400;  // Num of instr. cycles for larger-overhead parallelism.
constexpr int k_parallel_min_iterations = 8;               // Sufficient number of loop iterations for parallelism.

namespace details {

// Launches a set of threads whose number matches the hardware parallelism, and terminates these threads upon
// destruction.  The member function execute(num_tasks, task_function) allows parallel execution of an indexed task,
// i.e. calling task_function(0), ..., task_function(num_tasks-1) and waiting for all these calls to finish.
// The function execute() can be called successively on different tasks with low overhead as it reuses the
// same set of threads.
class ThreadPoolIndexedTask : noncopyable {
 public:
  using Task = std::function<void(int)>;
  ThreadPoolIndexedTask() {
    const int num_threads = get_max_threads();
    _threads.reserve(num_threads);
    for_int(i, num_threads) _threads.emplace_back(&ThreadPoolIndexedTask::worker_main, this);
  }
  ~ThreadPoolIndexedTask() {
    {
      std::unique_lock<std::mutex> lock(_mutex);
      assertx(_running);
      assertx(!_num_remaining_tasks);
      assertx(_task_index == _num_tasks);
      _running = false;
      _task_index = 0;
      _num_tasks = 1;
      _condition_variable_worker.notify_all();
    }
    for (auto& thread : _threads) thread.join();
  }
  int num_threads() const { return int(_threads.size()); }
  bool already_active() const { return _num_remaining_tasks != 0; }  // Detect nested execution.
  void execute(int num_tasks, const Task& task_function) {
    if (already_active()) {
      Warning("Nested execution of ThreadPoolIndexedTask is run serially");
      for_int(i, num_tasks) task_function(i);
    } else {
      std::unique_lock<std::mutex> lock(_mutex);
      _task_function = task_function;
      _num_tasks = num_tasks;
      _num_remaining_tasks = num_tasks;
      _task_index = 0;
      _condition_variable_worker.notify_all();
      _condition_variable_main.wait(lock, [this] { return !_num_remaining_tasks; });
    }
  }
  static ThreadPoolIndexedTask& default_threadpool() {
    static unique_ptr<ThreadPoolIndexedTask> thread_pool;
    // This is safe because thread_pool is nullptr only in the main thread before any other thread is launched.
    if (!thread_pool) thread_pool = make_unique<ThreadPoolIndexedTask>();
    return *thread_pool;
  }

 private:
  std::mutex _mutex;
  bool _running = true;
  std::vector<std::thread> _threads;
  Task _task_function;
  int _num_tasks = 0;
  int _num_remaining_tasks = 0;
  int _task_index = 0;
  std::condition_variable _condition_variable_worker;
  std::condition_variable _condition_variable_main;

  void worker_main() {
    std::unique_lock<std::mutex> lock(_mutex);
    // Consider: https://stackoverflow.com/questions/233127/how-can-i-propagate-exceptions-between-threads .
    // However, rethrowing the exception in the main thread loses the stack state, so not useful for debugging.
    for (;;) {
      _condition_variable_worker.wait(lock, [this] { return _task_index < _num_tasks; });
      if (!_running) break;
      while (_task_index < _num_tasks) {
        int i = _task_index++;
        lock.unlock();
        _task_function(i);
        lock.lock();
        assertx(_num_remaining_tasks > 0);
        if (!--_num_remaining_tasks) _condition_variable_main.notify_all();
      }
    }
  }
};

template <typename Iterator> class Subrange {
 public:
  Subrange(Iterator begin_, Iterator end_) : _begin(begin_), _end(end_) {}
  auto begin() { return _begin; }
  auto end() { return _end; }

 private:
  Iterator _begin, _end;
};

}  // namespace details

struct ParallelOptions {
  // Estimated number of CPU cycles required to process each element in the parallel loop.
  // If the total number of cycles across all range elements is estimated to be lower than `k_parallel_thresh`,
  // the loop is executed sequentially to avoid the overhead of synchronizing parallel threads.
  // With the default value, the loop is always executed in parallel.
  uint64_t cycles_per_elem = k_parallel_thresh;
};

// Divide `range` into `num_threads` chunks (i.e., subranges) and call `process_chunk(thread_index, subrange)` in
// parallel over the different chunks using a cached thread pool.  The range must support begin/end functions
// returning random-access iterators.
// Parallelism is disabled if the estimated cost `options.cycles_per_elem * size(range)` is less than some
// internal threshold, or if we are already executing (nested) within another parallel_for_*() loop.
// Exceptions within process_chunk() cause program termination as they are not caught.  One drawback over OpenMP
// is that if an exception or abort occurs within process_chunk(), the stack trace will not include the functions
// that called parallel_for_chunk() because these lie in the stack frames of a different thread.
// Environment variable OMP_NUM_THREADS overrides the default parallelism (even though OpenMP is not used).
template <typename Range, typename ProcessChunk>
void parallel_for_chunk(const ParallelOptions& options, const Range& range, int num_threads,
                        const ProcessChunk& process_chunk) {
  if (num_threads < 1) assertnever(SSHOW(num_threads));
  using std::begin, std::end, std::size;
  const auto begin_range = begin(range);
  const auto end_range = end(range);
  // const auto num_elements = end_range - begin_range;
  const auto num_elements = size(range);  // Note that num_elements be larger than size_t (e.g., uint64_t on win32).
  using Iterator = decltype(begin_range);
  const uint64_t total_num_cycles = num_elements * options.cycles_per_elem;
  const bool desire_parallelism = num_threads > 1 && total_num_cycles >= k_parallel_thresh;
  details::ThreadPoolIndexedTask* const thread_pool =
      desire_parallelism ? &details::ThreadPoolIndexedTask::default_threadpool() : nullptr;
  if (!thread_pool || thread_pool->already_active()) {
    // Process the entire range as a single chunk.
    const int thread_index = 0;
    details::Subrange<Iterator> subrange(begin_range, end_range);
    process_chunk(thread_index, subrange);
  } else {
    // Process the chunks in parallel.
    const auto chunk_size = (num_elements + num_threads - 1) / num_threads;
    const auto func = [begin_range, num_elements, chunk_size, &process_chunk](int thread_index) {
      Iterator begin_chunk = begin_range + std::min(thread_index * chunk_size, num_elements);
      Iterator end_chunk = begin_range + std::min((thread_index + 1) * chunk_size, num_elements);
      details::Subrange<Iterator> subrange(begin_chunk, end_chunk);
      process_chunk(thread_index, subrange);
    };
    thread_pool->execute(num_threads, func);
  }
}

// See previous function.
template <typename Range, typename ProcessChunk>
void parallel_for_chunk(const Range& range, int num_threads, const ProcessChunk& process_chunk) {
  parallel_for_chunk({}, range, num_threads, process_chunk);
}

// See previous function.
template <typename Range, typename ProcessChunk>
void parallel_for_chunk(const Range& range, const ProcessChunk& process_chunk) {
  parallel_for_chunk(range, get_max_threads(), [&](int thread_index, auto subrange) {
    dummy_use(thread_index);
    process_chunk(subrange);
  });
}

// Evaluates process_element(element) for each element in range by parallelizing across chunks of elements using
// a cached thread pool.  The range must support begin/end functions returning random-access iterators.
// Parallelism is disabled if the estimated cost (options.cycles_per_elem * size(range)) is less than some
// internal threshold, or if we are already executing (nested) within another parallel_for_*() loop.
// Exceptions within process_chunk() cause program termination as they are not caught.  One drawback over OpenMP
// is that if an exception or abort occurs within process_chunk(), the stack trace will not include the functions
// that called parallel_for_chunk() because these lie in the stack frames of a different thread.
// Environment variable OMP_NUM_THREADS overrides the default parallelism (even though OpenMP is not used).
template <typename Range, typename ProcessElement>
void parallel_for(const ParallelOptions& options, const Range& range, const ProcessElement& process_element) {
  using std::size;
  const auto num_elements = size(range);  // Could be size_t or larger (e.g., uint64_t on win32).
  if (!num_elements) return;
  using NumElements = decltype(num_elements);
  const int max_num_threads = get_max_threads();
  const int num_threads = int(std::min(NumElements(max_num_threads), num_elements));
  const auto process_chunk = [&](const int thread_index, auto subrange) {
    dummy_use(thread_index);
    for (auto& element : subrange) process_element(element);
  };
  parallel_for_chunk(options, range, num_threads, process_chunk);
}

// See previous function.
template <typename Range, typename ProcessElement>
void parallel_for(const Range& range, const ProcessElement& process_element) {
  parallel_for({}, range, process_element);
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_PARALLEL_H_
