// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "UnionFind.h"
#include "Vec.h"
using namespace hh;

int main() {
    {
        // using PairInt = std::pair<int,int>;
        using PairInt = Vec2<int>;
        UnionFind<PairInt> uf;
        SHOW(uf.get_label(PairInt(1, 7)));
        SHOW(uf.unify(PairInt(1, 1), PairInt(3, 2)));
        SHOW(uf.get_label(PairInt(3, 2)));
        SHOW(uf.get_label(PairInt(1, 1)));
        SHOW(uf.get_label(PairInt(1, 7)));
        SHOW(uf.unify(PairInt(3, 2), PairInt(5, 4)));
        SHOW(uf.unify(PairInt(7, 6), PairInt(1, 1)));
        SHOW(uf.unify(PairInt(1, 1), PairInt(3, 2)));
        SHOW(uf.get_label(PairInt(3, 2)));
        SHOW(uf.get_label(PairInt(1, 1)));
        SHOW(uf.get_label(PairInt(1, 7)));
        SHOW(uf.get_label(PairInt(7, 6)));
        SHOW(uf.get_label(PairInt(5, 4)));
    }
    {
        UnionFind<int> uf;
        SHOW(uf.unify( 1, 2));
        SHOW(uf.unify(11, 12));
        SHOW(uf.unify( 1, 3));
        SHOW(uf.unify( 1, 4));
        SHOW(uf.unify( 4, 5));
        SHOW(uf.unify( 5, 6));
        SHOW(uf.unify( 7, 4));
        SHOW(uf.unify(11, 13));
        SHOW(uf.unify(13, 14));
        SHOW(uf.unify(15, 13));
        SHOW(uf.unify(16, 11));
        SHOW(uf.unify( 0, 1));
        SHOW(uf.unify(20, 0));
        SHOW(uf.unify(19, 0));
        SHOW(uf.unify(19, 20));
        SHOW(uf.unify(16, 12));
        SHOW(uf.unify( 5, 7));
        SHOW(uf.unify(12, 3));
        SHOW(uf.unify(12, 14));
        SHOW(uf.unify(13, 7));
        SHOW(uf.unify( 1, 2));
    }
    {
        UnionFind<int> uf;
        SHOW(uf.get_label(5));
        uf.unify(1, 2);
        SHOW(uf.get_label(1));
        SHOW(uf.get_label(2));
        uf.promote(1);
        SHOW(uf.get_label(1));
        SHOW(uf.get_label(2));
        uf.promote(2);
        SHOW(uf.get_label(1));
        SHOW(uf.get_label(2));

        uf.unify(3, 4);
        SHOW(uf.get_label(3));
        SHOW(uf.get_label(4));
        uf.unify(1, 3);
        SHOW(uf.get_label(1));
        SHOW(uf.get_label(2));
        SHOW(uf.get_label(3));
        SHOW(uf.get_label(4));
        uf.promote(3);
        SHOW(uf.get_label(1));
        SHOW(uf.get_label(2));
        SHOW(uf.get_label(3));
        SHOW(uf.get_label(4));
        uf.promote(1);
        SHOW(uf.get_label(1));
        SHOW(uf.get_label(2));
        SHOW(uf.get_label(3));
        SHOW(uf.get_label(4));

        UnionFind<int> uf2;
        SHOW(uf2.get_label(1));
        SHOW(uf2.get_label(2));
        SHOW(uf2.get_label(3));
        SHOW(uf2.get_label(4));
        SHOW(uf2.get_label(5));
        uf2 = uf;
        SHOW(uf2.get_label(1));
        SHOW(uf2.get_label(2));
        SHOW(uf2.get_label(3));
        SHOW(uf2.get_label(4));
        SHOW(uf2.get_label(5));
    }
}

template class hh::UnionFind<unsigned>;
