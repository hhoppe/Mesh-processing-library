// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_IDTABLE_H_
#define MESH_PROCESSING_LIBHH_IDTABLE_H_

#include <bit>  // std::bit_ceil(), std::countr_zero()

#include "libHh/Array.h"
#include "libHh/Random.h"

#if 0
{
  IdTable<Polygon> table;  // Maps an int id (any value, including 0 or negative) to a non-null Polygon*.
  table.enter(id, new Polygon);
  Polygon* p = table.get(id);                 // The id must be present.
  Polygon* q = table.retrieve(id2);           // Returns nullptr if id2 is absent.
  for (Polygon* p : table.values()) func(p);  // In insertion order.
  delete table.remove(id);
}
#endif

namespace hh {

// Hash table from int ids to non-null pointers, organized like Python's compact dict: the entries are stored
// contiguously in insertion order, and a separate index of slots maps each id to its entry by open addressing
// (linear probing).  Compared with Map<int, T*> (std::unordered_map), it allocates no node per element, so
// iteration scans contiguous memory even after many insertions and removals (e.g., for the vertices and faces of a
// Mesh), memory is proportional to the number of elements however sparse the ids are, and the iteration order
// (insertion order) is the same on all platforms.  Insertion order also matters for speed: the elements of a Mesh
// are allocated in their creation order, so visiting them in insertion order traverses memory coherently, whereas
// visiting them in hash order was measured to be 2-8x slower.  Removing an element leaves a hole among the entries,
// and the entries are compacted (preserving their order) once holes form half of them.  Each group of
// k_block consecutive ids maps to a block of k_block consecutive slots (one cache line), so that lookups of nearby
// ids (e.g., the vertices of the faces read from a mesh file) share cache lines, and the blocks are scattered by a
// multiplicative (Fibonacci) hash of the group: an identity hash would map runs of consecutive ids to runs of
// consecutive slots, whose clusters make linear probing degenerate.
template <typename T> class IdTable : noncopyable {
  struct Entry {
    int id;
    T* p;  // A removed entry (a hole) has p == nullptr.
  };
  struct Slot {
    int id{0};
    int index{-1};  // Index into _entries; an empty slot has index == -1.
  };

 public:
  IdTable() = default;
  IdTable(IdTable&& t) noexcept { swap(*this, t); }
  IdTable& operator=(IdTable&& t) noexcept { return swap(*this, t), *this; }
  friend void swap(IdTable& l, IdTable& r) noexcept {
    ranges::swap(l._entries, r._entries);
    ranges::swap(l._slots, r._slots);
    ranges::swap(l._num, r._num);
    ranges::swap(l._shift, r._shift);
  }
  void clear() { *this = IdTable(); }
  void enter(int id, T* p) {  // The id must be new.
    ASSERTX(p);
    if (2 * (_num + 1) > _slots.num()) rebuild(max(k_min_capacity, 2 * _slots.num()));
    int i = home(id);
    for (; _slots[i].index >= 0; i = next(i)) ASSERTX(_slots[i].id != id);
    _slots[i] = {id, _entries.num()};
    _entries.push({id, p});
    _num++;
  }
  [[nodiscard]] bool contains(int id) const { return find(id) >= 0; }
  [[nodiscard]] T* retrieve(int id) const {  // Returns nullptr if id is absent.
    const int i = find(id);
    return i >= 0 ? _entries[_slots[i].index].p : nullptr;
  }
  [[nodiscard]] T* get(int id) const {  // The id must be present.
    const int i = find(id);
    ASSERTXX(i >= 0);
    return _entries[_slots[i].index].p;
  }
  T* remove(int id) {  // Returns nullptr if id is absent.
    int i = find(id);
    if (i < 0) return nullptr;
    Entry& entry = _entries[_slots[i].index];
    T* const p = entry.p;
    entry.p = nullptr;
    // Backward-shift deletion: move later slots of the probe sequence into the hole, so that no tombstone is
    // needed.  The slot j may move to the hole at i only if its home slot does not lie cyclically within (i, j].
    const int mask = _slots.num() - 1;
    for (int j = next(i); _slots[j].index >= 0; j = next(j)) {
      if (((j - home(_slots[j].id)) & mask) >= ((j - i) & mask)) {
        _slots[i] = _slots[j];
        i = j;
      }
    }
    _slots[i] = {};
    _num--;
    while (_entries.num() && !_entries.last().p) _entries.sub(1);  // Drop trailing holes (no index refers to them).
    // Compact the entries once holes form half of them, which bounds the cost of iteration and the expected
    // number of attempts in get_random(); also shrink the slots if they are sparsely used.
    if ((2 * _num < _entries.num() && _entries.num() > k_min_capacity) ||
        (16 * _num < _slots.num() && _slots.num() > k_min_capacity))
      rebuild(max(k_min_capacity, 2 * narrow_cast<int>(std::bit_ceil(unsigned(max(_num, 1))))));
    return p;
  }
  [[nodiscard]] int num() const { return _num; }
  [[nodiscard]] bool empty() const { return !_num; }
  [[nodiscard]] T* get_random(Random& random) const {  // Each element is equally likely.
    assertx(_num);
    for (;;) {
      T* const p = _entries[narrow_cast<int>(random.get_size_t() % size_t(_entries.num()))].p;
      if (p) return p;
    }
  }

  struct iterator {
    using type = iterator;
    using iterator_concept = std::forward_iterator_tag;
    using iterator_category = std::forward_iterator_tag;
    using value_type = T*;
    using difference_type = std::ptrdiff_t;
    using reference = T*;
    iterator() = default;
    iterator(const Entry* entry, const Entry* end) : _entry(entry), _end(end) { skip_holes(); }
    [[nodiscard]] bool operator==(const type& rhs) const { return _entry == rhs._entry; }
    [[nodiscard]] T* operator*() const { return ASSERTXX(_entry != _end), _entry->p; }
    type& operator++() {
      ASSERTXX(_entry != _end);
      ++_entry;
      skip_holes();
      return *this;
    }
    type operator++(int) { return postfix_increment(*this); }

   private:
    const Entry* _entry{nullptr};
    const Entry* _end{nullptr};
    void skip_holes() {
      while (_entry != _end && !_entry->p) ++_entry;
    }
  };

  struct Values_range : ranges::view_interface<Values_range> {
    explicit Values_range(const IdTable& t) : _t(&t) {}
    [[nodiscard]] iterator begin() const { return iterator(entries_begin(), entries_end()); }
    [[nodiscard]] iterator end() const { return iterator(entries_end(), entries_end()); }
    [[nodiscard]] int size() const { return _t->_num; }

   private:
    const IdTable* _t;
    [[nodiscard]] const Entry* entries_begin() const { return _t->_entries.data(); }
    [[nodiscard]] const Entry* entries_end() const { return _t->_entries.data() + _t->_entries.num(); }
  };
  [[nodiscard]] Values_range values() const { return Values_range(*this); }  // In insertion order.

 private:
  static constexpr int k_log2_block = 3;
  static constexpr int k_block = 1 << k_log2_block;  // Slots per block; 8 slots of 8 bytes form a cache line.
  static constexpr int k_min_capacity = 16;
  static_assert(k_min_capacity >= k_block);
  Array<Entry> _entries;  // In insertion order, including holes; the last entry is never a hole.
  Array<Slot> _slots;     // Capacity is zero or a power of two, at least twice _num.
  int _num{0};            // Number of elements, i.e., of entries that are not holes.
  int _shift{64};         // 64 - log2(_slots.num() / k_block).
  [[nodiscard]] int home(int id) const {
    const uint64_t group = uint32_t(id) >> k_log2_block;
    const int block = int((group * uint64_t{0x9E3779B97F4A7C15}) >> _shift);
    return (block << k_log2_block) | (id & (k_block - 1));
  }
  [[nodiscard]] int next(int i) const { return (i + 1) & (_slots.num() - 1); }
  [[nodiscard]] int find(int id) const {  // Returns the slot index, or -1 if id is absent.
    if (!_num) return -1;
    for (int i = home(id);; i = next(i)) {
      const Slot& slot = _slots[i];
      if (slot.index < 0) return -1;
      if (slot.id == id) return i;
    }
  }
  void rebuild(int capacity) {  // Remove the holes from the entries, and rebuild the slots with the capacity.
    if (_entries.num() != _num) {
      Array<Entry> entries;
      entries.reserve(_num);
      for (const Entry& entry : _entries)
        if (entry.p) entries.push(entry);
      _entries = std::move(entries);
    }
    _slots = Array<Slot>(capacity, Slot{});
    _shift = 64 - std::countr_zero(unsigned(capacity)) + k_log2_block;
    for_int(index, _entries.num()) {
      int i = home(_entries[index].id);
      while (_slots[i].index >= 0) i = next(i);
      _slots[i] = {_entries[index].id, index};
    }
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_IDTABLE_H_
