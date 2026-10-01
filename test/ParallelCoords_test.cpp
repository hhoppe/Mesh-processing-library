// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/ParallelCoords.h"

#include <atomic>
#include <vector>

#include "libHh/Array.h"
using namespace hh;

namespace {

// Per-element visit counters, which may be incremented concurrently.
class Counts {
 public:
  explicit Counts(size_t n) : _counts(n) {}
  void add(size_t i) { _counts[i].fetch_add(1, std::memory_order_relaxed); }
  [[nodiscard]] int operator[](size_t i) const { return _counts[i].load(); }

 private:
  std::vector<std::atomic<int>> _counts;
};

// Is the coordinate u on the outer boundary of a grid with dimensions dims?
template <int D> bool on_boundary(const Vec<int, D>& dims, const Vec<int, D>& u) {
  for_int(c, D) {
    if (u[c] == 0 || u[c] == dims[c] - 1) return true;
  }
  return false;
}

// Each element is visited once, by func() if on the boundary of [x0, xn) and otherwise by func_interior().
void test_1d_interior() {
  for_int(x0, 3) for_int(n, 7) {
    const int xn = x0 + n;
    Array<int> kinds(n, 0);
    for_1dL_interior(x0, xn, [&](const int x) { kinds[x - x0] += 1; }, [&](const int x) { kinds[x - x0] += 10; });
    for_int(i, n) assertx(kinds[i] == (i == 0 || i == n - 1 ? 1 : 10));
  }
}

void test_2d_interior() {
  for_int(y0, 2) for_int(x0, 2) for_int(ny, 5) for_int(nx, 5) {
    const int yn = y0 + ny, xn = x0 + nx;
    const auto expected = [&](const int y, const int x) {
      return y == y0 || y == yn - 1 || x == x0 || x == xn - 1 ? 1 : 10;
    };
    for_int(parallel, 2) {
      Counts counts(size_t(ny * nx) * 11);  // Separate counters for func() and func_interior().
      const auto index = [&](const int y, const int x) { return size_t(y - y0) * size_t(nx) + size_t(x - x0); };
      const auto func = [&](const int y, const int x) { counts.add(index(y, x) * 11); };
      const auto func_interior = [&](const int y, const int x) { counts.add(index(y, x) * 11 + 10); };
      if (parallel)
        parallel_for_2dL_interior(y0, yn, x0, xn, func, func_interior);
      else
        for_2dL_interior(y0, yn, x0, xn, func, func_interior);
      for_intL(y, y0, yn) for_intL(x, x0, xn) {
        const int kind = counts[index(y, x) * 11] + counts[index(y, x) * 11 + 10] * 10;
        assertx(kind == expected(y, x));
      }
    }
  }
}

void test_2d_order() {
  string s;
  for_2d(2, 3, [&](const int y, const int x) { s += sform(" %d%d", y, x); });
  SHOW(s);
  s.clear();
  for_2dL(1, 3, 2, 4, [&](const int y, const int x) { s += sform(" %d%d", y, x); });
  SHOW(s);
  s.clear();
  for_coordsL(V(1, 0, 2), V(3, 2, 4), [&](const Vec3<int>& u) { s += sform(" %d%d%d", u[0], u[1], u[2]); });
  SHOW(s);
  s.clear();
  for_coords(V(2, 2), [&](const Vec2<int>& u) { s += sform(" %d%d", u[0], u[1]); });
  SHOW(s);
}

void test_parallel_2d() {
  const int ny = 37, nx = 23;
  Counts counts(ny * nx);
  parallel_for_2d(ny, nx, [&](const int y, const int x) { counts.add(y * nx + x); });
  for_int(i, ny * nx) assertx(counts[i] == 1);
  Counts counts2(ny * nx);
  parallel_for_2dL(3, ny, 5, nx, [&](const int y, const int x) { counts2.add(y * nx + x); });
  for_int(y, ny) for_int(x, nx) assertx(counts2[y * nx + x] == (y >= 3 && x >= 5 ? 1 : 0));
}

// Every coordinate in [uL, uU) is visited exactly once, for options that select each of the parallelization branches.
template <int D> void test_parallel_coords(const Vec<int, D>& uL, const Vec<int, D>& uU) {
  const Vec<int, D> dims = uU;
  for (const uint64_t cycles_per_elem : {uint64_t{1}, uint64_t{200}, k_parallel_thresh}) {
    Counts counts(product(dims));
    parallel_for_coordsL(ParallelOptions{.cycles_per_elem = cycles_per_elem}, uL, uU,
                         [&](const Vec<int, D>& u) { counts.add(ravel_index(dims, u)); });
    for (const Vec<int, D>& u : range(dims)) assertx(counts[ravel_index(dims, u)] == (u.in_range(uL, uU) ? 1 : 0));
  }
}

template <int D> void test_raster(const Vec<int, D>& dims, const Vec<int, D>& uL, const Vec<int, D>& uU) {
  Array<size_t> indices, expected;
  for_coordsL_raster(dims, uL, uU, [&](const size_t i) { indices.push(i); });
  for (const Vec<int, D>& u : range(uL, uU)) expected.push(ravel_index(dims, u));
  assertx(indices == expected);
}

// Each coordinate in [uL, uU) is visited once, by func() if on the boundary of the grid, else by func_interior().
template <int D> void test_interior(const Vec<int, D>& dims, const Vec<int, D>& uL, const Vec<int, D>& uU) {
  for_int(parallel, 2) {
    const size_t n = product(dims);
    Counts counts_func(n), counts_interior(n);
    const auto func = [&](const Vec<int, D>& u) { counts_func.add(ravel_index(dims, u)); };
    const auto func_interior = [&](const size_t i) { counts_interior.add(i); };
    if (parallel)
      parallel_d0_for_coordsL_interior(dims, uL, uU, func, func_interior);
    else
      for_coordsL_interior(dims, uL, uU, func, func_interior);
    for (const Vec<int, D>& u : range(dims)) {
      const size_t i = ravel_index(dims, u);
      const bool inside = u.in_range(uL, uU);
      assertx(counts_func[i] == (inside && on_boundary(dims, u) ? 1 : 0));
      assertx(counts_interior[i] == (inside && !on_boundary(dims, u) ? 1 : 0));
    }
  }
}

}  // namespace

int main() {
  test_1d_interior();
  test_2d_interior();
  test_2d_order();
  test_parallel_2d();
  {
    test_parallel_coords(V(0), V(700));
    test_parallel_coords(V(5), V(700));
    test_parallel_coords(V(0, 0), V(16, 40));
    test_parallel_coords(V(0, 0), V(4, 600));
    test_parallel_coords(V(1, 2), V(16, 40));
    test_parallel_coords(V(0, 0, 0), V(16, 10, 10));
    test_parallel_coords(V(0, 0, 0), V(3, 16, 40));
    test_parallel_coords(V(0, 0, 0), V(2, 3, 600));
    test_parallel_coords(V(1, 0, 3), V(3, 16, 40));
    test_parallel_coords(V(0, 0, 0, 0), V(8, 4, 4, 4));
    test_parallel_coords(V(2, 1, 0, 1), V(8, 4, 4, 4));
  }
  {
    test_raster(V(5), V(1), V(4));
    test_raster(V(5, 6), V(1, 0), V(4, 6));
    test_raster(V(5, 6, 7), V(1, 0, 2), V(4, 6, 5));
    test_raster(V(3, 4, 5, 6), V(1, 1, 0, 2), V(3, 3, 5, 4));
    test_raster(V(5, 6), V(5, 0), V(5, 6));  // Empty.
  }
  {
    test_interior(V(6, 7), V(0, 0), V(6, 7));  // The whole grid.
    test_interior(V(6, 7), V(0, 2), V(3, 7));  // Touching some of the grid boundary.
    test_interior(V(6, 7), V(1, 1), V(5, 6));  // Entirely interior.
    test_interior(V(6, 7), V(6, 7), V(6, 7));  // Empty, at the far corner.
    test_interior(V(1, 7), V(0, 0), V(1, 7));  // A single row.
    test_interior(V(2, 2), V(0, 0), V(2, 2));  // No interior.
    test_interior(V(5, 6, 7), V(0, 0, 0), V(5, 6, 7));
    test_interior(V(5, 6, 7), V(2, 0, 3), V(5, 4, 7));
    test_interior(V(30, 4, 5), V(0, 0, 0), V(30, 4, 5));  // Enough rows to divide among the threads.
  }
  showf("done\n");
}
