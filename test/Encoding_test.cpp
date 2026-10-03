// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Encoding.h"

#include "libHh/Random.h"
using namespace hh;

namespace {

// A reference model of MoveToFront::enter(): it returns the prior index of the element (or the list length if it is
// new), and moves the element to the front of the list.
int move_to_front_model(Array<int>& list, int e) {
  const int i = find_index(list, e).value_or(list.num());
  if (i == list.num()) list.push(e);
  for (int j = i; j > 0; --j) std::swap(list[j], list[j - 1]);
  return i;
}

}  // namespace

int main() {
  {
    MoveToFront<int> move;
    SHOW(move.enter(4));
    SHOW(move.enter(5));
    SHOW(move.enter(6));
    SHOW(move.enter(4));
    SHOW(move.enter(4));
    SHOW(move.enter(5));
    SHOW(move.enter(4));
  }
  {
    Encoding<int> enc;
    enc.add(10, .5f);
    enc.add(11, .125f);
    enc.add(12, .125f);
    enc.add(13, .125f);
    enc.add(12, .125f);
    enc.print();
    SHOW(enc.huffman_cost());
    SHOW(enc.entropy());
    SHOW(enc.norm_entropy());
    enc.print_top_entries("enc", 2, [](const int& i) { return std::to_string(i); });
  }
  {
    // With only negative values, the sign encoding that follows a positive sign remains empty and costs no bits.
    DeltaEncoding de;
    de.enter_coords(V(-3.f, -5.f));
    assertx(de.total_entropy() == 4.f);
  }
  {
    DeltaEncoding de;
    de.enter_sign(0);
    de.enter_bits(3);
    de.enter_sign(0);
    de.enter_bits(4);
    de.enter_sign(1);
    de.enter_bits(5);
    de.enter_sign(1);
    de.enter_bits(4);
    de.enter_sign(1);
    de.enter_bits(3);
    int total_bits = de.analyze("de");
    SHOW(total_bits);
    SHOW(DeltaEncoding::val_bits(13.f));
    SHOW(DeltaEncoding::val_bits(3.15f));
    SHOW(DeltaEncoding::val_bits(3.1f));
    SHOW(DeltaEncoding::val_bits(3.f));
    SHOW(DeltaEncoding::val_bits(2.5f));
    SHOW(DeltaEncoding::val_bits(2.2f));
    SHOW(DeltaEncoding::val_bits(2.0f));
    SHOW(DeltaEncoding::val_bits(1.5f));
    SHOW(DeltaEncoding::val_bits(1.0f));
    SHOW(DeltaEncoding::val_sign(13.3f));
    SHOW(DeltaEncoding::val_sign(-13.3f));
    SHOW(de.total_entropy());
    assertx(total_bits == int(std::ceil(de.total_entropy())));
  }
  {
    // After entering 1, 2, 3 (giving the list [3, 2, 1]), entering 2 gives [2, 3, 1], so that enter(3) returns 1.
    MoveToFront<int> move;
    SHOW(move.enter(1), move.enter(2), move.enter(3), move.enter(2), move.enter(3));
  }
  {
    // A random sequence of elements, compared against a reference model.
    MoveToFront<int> move;
    Array<int> list;
    Random random{7};
    for_int(iter, 500) {
      const int e = int(random.get_unsigned(12));
      assertx(move.enter(e) == move_to_front_model(list, e));
    }
    assertx(list.num() == 12);
  }
  {
    MoveToFront<string> move;
    SHOW(move.enter("a"), move.enter("b"));
    SHOW(move.enter("a"), move.enter("a"));
  }
  {
    // An empty distribution has zero entropy (without a warning).
    const Encoding<int> enc;
    assertx(enc.entropy() == 0.f && enc.norm_entropy() == 0.f);
  }
  {
    // A single event has zero entropy.
    Encoding<int> enc;
    enc.add(3, 2.f);
    SHOW(enc.entropy(), enc.norm_entropy());
  }
  {
    // Unnormalized counts: entropy() is in total bits, and norm_entropy() is in bits per unit of probability.
    Encoding<int> enc;
    enc.add(1, 2.f);
    enc.add(2, 1.f);
    enc.add(3, 1.f);
    // The Huffman code lengths are 1, 2, 2 bits, which match the entropy because the frequencies are dyadic.
    SHOW(enc.huffman_cost(), enc.entropy(), enc.norm_entropy());
    assertx(enc.huffman_cost() == 6.f && enc.entropy() == 6.f && enc.norm_entropy() == 1.5f);
  }
  {
    // With non-dyadic frequencies, the Huffman cost exceeds the entropy.
    Encoding<string> enc;
    for (const char* s : {"x", "y", "z", "x", "y", "z"}) enc.add(s, 1.f);
    enc.print();
    SHOW(enc.huffman_cost());  // The code lengths are 1, 2, and 2 bits for 2 occurrences of each symbol.
    const float expected_entropy = float(6. * std::log2(3.));
    assertx(std::abs(enc.entropy() - expected_entropy) < 1e-5f);
    assertx(std::abs(enc.norm_entropy() - expected_entropy / 6.f) < 1e-6f);
    SHOW(std::round(enc.entropy() * 1e4f) / 1e4f);
  }
  {
    // Huffman cost of a uniform distribution over 8 events, which needs exactly 3 bits per event.
    Encoding<int> enc;
    for_int(i, 8) enc.add(i, .125f);
    SHOW(enc.huffman_cost(), enc.entropy());
  }
  {
    // The val_bits() table documented in Encoding.h, for values 1 through 20, and for negative values.
    const Vec<int, 20> expected{1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4};
    for_int(i, 20) {
      const float v = float(i + 1);
      assertx(DeltaEncoding::val_bits(v) == expected[i] && DeltaEncoding::val_bits(-v) == expected[i]);
    }
    SHOW(DeltaEncoding::val_bits(0.f), DeltaEncoding::val_bits(0.99f), DeltaEncoding::val_bits(-0.5f));
    SHOW(DeltaEncoding::val_bits(255.f), DeltaEncoding::val_bits(256.f));
    SHOW(DeltaEncoding::val_sign(0.f), DeltaEncoding::val_sign(-0.f), DeltaEncoding::val_sign(-1e-6f));
  }
  {
    // enter_coords() encodes each coordinate separately, with a sign only for a nonzero number of bits.
    DeltaEncoding de;
    // (The last coordinate gives a sign that follows a positive sign; otherwise, total_entropy() would evaluate
    // entropy() on an empty Encoding, which warns.)
    de.enter_coords(V(0.5f, -3.f, 7.f, 2.f));
    SHOW(de.analyze(""));  // An empty name prints nothing.
    de.enter_coords(V(0.f, 0.f, 0.f));
    SHOW(de.analyze("coords"));
  }
  {
    // enter_vector() encodes the maximum number of bits once for the whole vector, and then all its signs.
    DeltaEncoding de;
    de.enter_vector(V(0.5f, -3.f, 7.f));
    de.enter_vector(V(0.f, 0.f, 0.f));
    de.enter_vector(V(1.f, -1.f, 2.f));
    SHOW(de.analyze("vector"));
  }
}

template class hh::MoveToFront<int>;
template class hh::MoveToFront<int*>;

template class hh::Encoding<int>;
template class hh::Encoding<int*>;
