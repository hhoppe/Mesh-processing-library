// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Encoding.h"
using namespace hh;

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
        enc.print_top_entries("enc", 2, [](const int& i) { return sform("%d", i); });
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
    }
}

namespace hh {
template class MoveToFront<int>;
template class MoveToFront<int*>;

template class Encoding<int>;
template class Encoding<int*>;
} // namespace hh
