// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/IdTable.h"

#include <climits>  // INT_MIN, INT_MAX
#include <map>

#include "libHh/Array.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // sort()
using namespace hh;

namespace {

struct Element {
  int id;
  int seq{0};  // Insertion sequence number.
};

// Verify that the table holds exactly the entries of the model, through lookups and through iteration.
void verify(const IdTable<Element>& table, const std::map<int, Element*>& model) {
  assertx(table.num() == int(model.size()));
  assertx(table.empty() == model.empty());
  assertx(table.values().size() == table.num());
  Array<int> ids;
  int last_seq = -1;
  for (Element* p : table.values()) {
    ids.push(p->id);
    assertx(p->seq > last_seq);  // Iteration follows insertion order.
    last_seq = p->seq;
  }
  Array<int> model_ids;
  for (const auto& [id, p] : model) {
    model_ids.push(id);
    assertx(table.contains(id));
    assertx(table.get(id) == p);
    assertx(table.retrieve(id) == p);
  }
  assertx(sort(ids) == model_ids);
}

// Random insertions and removals compared with a std::map, for ids drawn from a few distributions.
void test_random_operations(const char* name, auto draw_id) {
  Random random(1);
  Array<unique_ptr<Element>> storage;
  IdTable<Element> table;
  std::map<int, Element*> model;
  int max_num = 0;
  for_int(iter, 60'000) {
    // Grow during the first half, then shrink until the table is almost empty.
    const bool grow = iter < 30'000 ? random.get_unsigned(4) != 0 : random.get_unsigned(5) == 0;
    const int id = draw_id(random);
    auto it = model.find(id);
    if (grow && it == model.end()) {
      storage.push(make_unique<Element>(id, storage.num()));
      table.enter(id, storage.last().get());
      model.emplace(id, storage.last().get());
    } else if (!grow && !model.empty()) {
      if (it == model.end()) {  // Remove an existing element, near the drawn id.
        it = model.lower_bound(id);
        if (it == model.end()) it = model.begin();
      }
      const int id2 = it->first;
      Element* const p = it->second;
      model.erase(it);
      assertx(table.remove(id2) == p);
      assertx(!table.remove(id2));
      assertx(!table.contains(id2) && !table.retrieve(id2));
    }
    max_num = max(max_num, table.num());
    if (iter % 1'000 == 0) verify(table, model);
  }
  verify(table, model);
  SHOW(name, max_num, table.num());
  table.clear();
  assertx(table.empty() && !table.contains(model.empty() ? 1 : model.begin()->first));
  assertx(ranges::distance(table.values()) == 0);
}

void test_special_ids() {
  Array<unique_ptr<Element>> storage;
  IdTable<Element> table;
  const Array<int> ids = {0, -1, 1, INT_MIN, INT_MAX, 2'000'000'000, -2'000'000'000};
  for (const int id : ids) {
    storage.push(make_unique<Element>(id));
    table.enter(id, storage.last().get());
  }
  for_int(i, ids.num()) assertx(table.get(ids[i]) == storage[i].get());
  assertx(!table.contains(2) && !table.retrieve(-2));
  assertx(table.remove(0) == storage[0].get());
  assertx(!table.contains(0) && table.contains(-1) && table.contains(INT_MIN));
  Array<int> iterated_ids;
  for (Element* p : table.values()) iterated_ids.push(p->id);
  SHOW(iterated_ids);
}

void test_empty() {
  IdTable<Element> table;
  assertx(table.empty() && !table.num() && !table.contains(0) && !table.retrieve(7) && !table.remove(7));
  assertx(table.values().begin() == table.values().end());
  IdTable<Element> table2 = std::move(table);
  assertx(table2.empty());
}

// Many consecutive ids: a hash that clustered them would make each operation take time linear in their number.
void test_consecutive_ids() {
  const int n = 1'000'000;
  Array<Element> storage(n);
  IdTable<Element> table;
  for_int(i, n) {
    storage[i].id = i + 1;
    table.enter(i + 1, &storage[i]);
  }
  for_int(i, n) assertx(table.get(i + 1) == &storage[i]);
  for_int(i, n) {  // Remove all ids except the multiples of 100, leaving sparse ids.
    if ((i + 1) % 100) assertx(table.remove(i + 1) == &storage[i]);
  }
  int count = 0;
  for (Element* p : table.values()) count += p->id % 100 == 0;
  SHOW(table.num(), count);
}

// Every element is equally likely to be drawn by get_random(), also after many removals.
void test_random_sampling() {
  const int n = 1'000;
  Array<Element> storage(n);
  IdTable<Element> table;
  for_int(i, n) {
    storage[i].id = i * 7;
    table.enter(i * 7, &storage[i]);
  }
  for_int(i, n) {
    if (i % 10) (void)table.remove(i * 7);
  }
  Random random(2);
  Array<int> counts(n, 0);
  const int num_samples = 100'000;
  for_int(i, num_samples) counts[narrow_cast<int>(table.get_random(random) - storage.data())]++;
  int min_count = INT_MAX, max_count = 0;
  for_int(i, n) {
    if (i % 10) {
      assertx(!counts[i]);
    } else {
      min_count = min(min_count, counts[i]);
      max_count = max(max_count, counts[i]);
    }
  }
  // Each of the 100 elements expects 1'000 samples, with a standard deviation of about 31.
  assertx(min_count > 850 && max_count < 1'150);
  SHOW(table.num(), "uniform");
}

}  // namespace

int main() {
  test_empty();
  test_special_ids();
  test_random_operations("dense", [](Random& r) { return 1 + int(r.get_unsigned(5'000)); });
  test_random_operations("sparse", [](Random& r) { return int(r.get_unsigned()); });
  test_random_operations("strided", [](Random& r) { return int(r.get_unsigned(5'000)) * 1'024; });
  test_consecutive_ids();
  test_random_sampling();
}

template class hh::IdTable<Element>;
