// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_TALLY_H_
#define MESH_PROCESSING_LIBHH_TALLY_H_

#include "libHh/Array.h"
#include "libHh/RangeOp.h"
#include "libHh/Stat.h"

namespace hh {

// Counts of the occurrences of each value in a range of nonnegative integers.
class Tally {
 public:
  template <ranges::forward_range R> requires std::convertible_to<ranges::range_reference_t<R>, int>
  explicit Tally(const R& range_) {
    auto&& range = iterable(range_);
    if (ranges::empty(range)) return;
    _counts.init(int(max(range)) + 1, 0);
    for (const int value : range) {
      assertx(value >= 0);
      _counts[value]++;
    }
  }
  [[nodiscard]] int num() const { return _counts.num(); }  // One more than the largest value.
  [[nodiscard]] int operator[](int value) const {          // Zero for a value beyond the largest.
    ASSERTX(value >= 0);
    return value < num() ? _counts[value] : 0;
  }
  [[nodiscard]] int64_t total() const { return sum<int64_t>(_counts); }
  void show() const {
    showff("Tally[0..%d] (tot=%lld):{\n", num() - 1, possible_cast<long long>(total()));
    for_int(i, num()) {
      if (_counts[i]) showff("  [%3d]: %d\n", i, _counts[i]);
    }
    showff("}\n");
  }
  [[nodiscard]] Stat stat() const {  // Statistics of the values, weighted by their counts.
    Stat result;
    for_int(i, num()) {
      if (_counts[i]) result.enter_multiple(float(i), _counts[i]);
    }
    return result;
  }

 private:
  Array<int> _counts;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_TALLY_H_
