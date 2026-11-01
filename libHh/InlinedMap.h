// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_INLINEDMAP_H_
#define MESH_PROCESSING_LIBHH_INLINEDMAP_H_

#include "libHh/Array.h"

#if 0
{
  InlinedMap<Vertex, float, 8> map;
  map[v] += 1.f;
  for (auto& [v, value] : map) SHOW(mesh.vertex_id(v), value);
}
#endif

namespace hh {

// A map intended for few entries (up to a few dozen), stored as an array of key-value pairs that is searched linearly,
// and that holds up to inline_capacity entries without heap allocation.  Its iteration order is the order of
// insertion, except that remove() moves the last entry into the place of the removed one.
template <typename Key, typename Value, int inline_capacity> class InlinedMap {
 public:
  using value_type = std::pair<Key, Value>;
  void clear() { _entries.clear(); }
  void enter(Key key, Value value) {  // Key must be new!
    ASSERTX(!find(key));
    _entries.push(value_type{std::move(key), std::move(value)});
  }
  [[nodiscard]] bool contains(const Key& key) const { return find(key) != nullptr; }
  // Returns the value of the key if present, else a default value.
  [[nodiscard]] Value retrieve(const Key& key) const requires Copyable<Value> {
    const value_type* p = find(key);
    return p ? p->second : Value{};
  }
  [[nodiscard]] auto& get(this auto&& self, const Key& key) { return assertx(self.find(key))->second; }
  const Value& operator[](const Key& key) const { return get(key); }  // Key must be present.
  Value& operator[](const Key& key) requires Copyable<Key> {
    if (value_type* p = find(key)) return p->second;
    _entries.push(value_type{key, Value{}});
    return _entries.last().second;
  }
  Value remove(const Key& key) {
    value_type* p = assertx(find(key));
    Value value = std::move(p->second);
    if (p != &_entries.last()) *p = std::move(_entries.last());
    (void)_entries.pop();
    return value;
  }
  [[nodiscard]] int num() const { return _entries.num(); }
  [[nodiscard]] bool empty() const { return _entries.num() == 0; }
  [[nodiscard]] auto begin(this auto&& self) { return self._entries.begin(); }
  [[nodiscard]] auto end(this auto&& self) { return self._entries.end(); }
  [[nodiscard]] auto keys() const { return views::keys(_entries); }
  [[nodiscard]] auto values(this auto&& self) { return views::values(self._entries); }

 private:
  InlinedArray<value_type, inline_capacity> _entries;

  [[nodiscard]] auto* find(this auto&& self, const Key& key) {
    for (auto& entry : self._entries)
      if (entry.first == key) return &entry;
    return static_cast<decltype(&self._entries[0])>(nullptr);
  }
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_INLINEDMAP_H_
