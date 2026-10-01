// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/VertexCache.h"

#include <deque>

#include "libHh/Set.h"
using namespace hh;

namespace {

// A straightforward reference model of the cache: the cached vertices, with the most recent one at the front.
class ModelCache {
 public:
  ModelCache(VertexCache::EType type, int cs) : _type(type), _cs(cs) {}
  bool access_hits(int vi) {
    const auto it = ranges::find(_list, vi);
    if (it != _list.end()) {
      if (_type == VertexCache::EType::lru) {
        _list.erase(it);
        _list.push_front(vi);
      }
      return true;
    }
    _list.push_front(vi);
    if (int(_list.size()) > _cs) _list.pop_back();
    return false;
  }
  [[nodiscard]] int location(int vi) const {
    const auto it = ranges::find(_list, vi);
    return it == _list.end() ? -1 : int(it - _list.begin());
  }
  [[nodiscard]] Array<int> contents() const { return sort(Array<int>(_list)); }

 private:
  VertexCache::EType _type;
  int _cs;
  std::deque<int> _list;
};

// The cached vertices, in sorted order (because the iteration order is undefined).
[[nodiscard]] Array<int> cache_contents(const VertexCache& vc) {
  Array<int> ar;
  Set<int> set;
  const auto iter = vc.make_iterator();
  for (;;) {
    const int vi = iter->next();
    if (!vi) break;
    assertx(set.add(vi));  // Each vertex appears at most once.
    ar.push(vi);
  }
  return sort(std::move(ar));
}

// Compare the cache with the model after each access in a pseudorandom sequence of vertex references.
void test_against_model(VertexCache::EType type, int nverts, int cs) {
  const int nverts1 = nverts + 1;  // Vertex ids begin at 1.
  const auto vc = VertexCache::make(type, nverts1, cs);
  assertx(vc->type() == type);
  ModelCache model(type, cs);
  uint32_t state = 1;
  int num_hits = 0;
  for_int(i, 2000) {
    state = state * 1'664'525u + 1'013'904'223u;
    const int vi = 1 + int((state >> 16) % unsigned(nverts));
    const bool hit = vc->access_hits(vi);
    assertx(hit == model.access_hits(vi));
    num_hits += hit;
    for_intL(vj, 1, nverts1) {
      const int loc = model.location(vj);
      assertx(vc->contains(vj) == (loc >= 0));
      assertx(vc->location(vj) == loc);
      assertx(vc->location_alt(vj) == (loc >= 0 ? loc : cs));
    }
    assertx(cache_contents(*vc) == model.contents());
  }
  SHOW(VertexCache::type_string(type), nverts, cs, num_hits);
}

void test_copy_and_init(VertexCache::EType type) {
  const int nverts1 = 10, cs = 4;
  const auto vc1 = VertexCache::make(type, nverts1, cs);
  for (const int vi : {1, 2, 3, 2, 5, 6}) dummy_use(vc1->access_hits(vi));
  const auto vc2 = VertexCache::make(type, nverts1, cs);
  dummy_use(vc2->access_hits(9));
  vc2->copy(*vc1);
  for_intL(vi, 1, nverts1) assertx(vc2->location(vi) == vc1->location(vi));
  SHOW(VertexCache::type_string(type), *vc1, *vc2);
  vc1->init(nverts1, cs);  // Reinitialization empties the cache.
  for_intL(vi, 1, nverts1) assertx(!vc1->contains(vi));
  SHOW(*vc1, vc1->access_hits(5), *vc1);
  SHOW(*vc2);  // The copy is independent.
}

}  // namespace

int main() {
  for (const auto type : {VertexCache::EType::fifo, VertexCache::EType::lru}) {
    for (const int cs : {1, 2, 5, 16}) {
      for (const int nverts : {1, 3, 20}) test_against_model(type, nverts, cs);
    }
    test_copy_and_init(type);
  }
}
