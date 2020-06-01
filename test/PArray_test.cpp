// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "PArray.h"
using namespace hh;

int main() {
    struct ST {
        ST(int i)                                   : _i(i) { showf("ST(%d)\n", _i); }
        ~ST()                                       { showf("~ST(%d)\n", _i); }
        int _i;
    };
    auto func_construct_array = [](int i0, int n) { // -> PArray<unique_ptr<ST>, 2>
        PArray<unique_ptr<ST>, 2> ar;
        for_int(i, n) { ar.push(make_unique<ST>(i0+i)); }
        return ar;
    };
    {
        SHOW("beg");
        PArray<unique_ptr<ST>, 2> ar;
        ar.push(make_unique<ST>(4));
        SHOW("end");
    }
    {
        SHOW("beg");
        PArray<unique_ptr<ST>, 2> ar;
        ar.push(make_unique<ST>(4));
        ar.push(make_unique<ST>(5));
        SHOW("end");
    }
    {
        SHOW("beg");
        PArray<unique_ptr<ST>, 2> ar;
        ar.push(make_unique<ST>(4));
        ar.push(make_unique<ST>(5));
        ar.push(make_unique<ST>(6));
        for (auto& e : ar) { SHOW(e->_i); }
        SHOW("end");
    }
    {
        SHOW("beg");
        PArray<unique_ptr<ST>, 2> ar;
        for_int(i, 20) { ar.push(make_unique<ST>(i)); }
        SHOW("end");
    }
    {
        SHOW("beg");
        PArray<unique_ptr<ST>, 2> ar(func_construct_array(100, 2));
        SHOW("end");
    }
    {
        SHOW("beg");
        PArray<unique_ptr<ST>, 2> ar;
        ar = func_construct_array(500, 2);
        SHOW(ar[0]->_i);
        ar = func_construct_array(600, 3);
        SHOW("end");
    }
    {
        SHOW("beg");
        auto ar = func_construct_array(100, 3);
        SHOW("mid");
        ar = func_construct_array(200, 2);
        SHOW("end");
    }
    {
        SHOW("beg");
        auto ar = func_construct_array(100, 3);
        SHOW("mid");
        ar = func_construct_array(200, 3);
        SHOW("end");
    }
    {
        PArray<int,3> ar1;
        SHOW(ar1);
        ar1.push(7);
        ar1.push(6);
        ar1.push(5);
        SHOW(ar1);
        ar1.push(4);
        ar1.push(3);
        SHOW(ar1);
        auto func = [](int v) { return v*1.5f; };
        SHOW(map(ar1, func));
        PArray<int,3> ar2;
        ar2.push(11);
        ar2.push(12);
        SHOW(ar2);
        swap(ar1, ar2); SHOW("after swap"); SHOW(ar1); SHOW(ar2);
        swap(ar1, ar2); SHOW("after swap back"); SHOW(ar1); SHOW(ar2);
        ar2.push(13);
        ar2.push(14);
        ar2.push(15);
        SHOW(ar2);
        swap(ar1, ar2); SHOW("after swap"); SHOW(ar1); SHOW(ar2);
        swap(ar1, ar2); SHOW("after swap back"); SHOW(ar1); SHOW(ar2);
        ar1.erase(0, 3);
        SHOW(ar1);
        ar2.erase(0, 3);
        SHOW(ar2);
        swap(ar1, ar2); SHOW("after swap"); SHOW(ar1); SHOW(ar2);
        swap(ar1, ar2); SHOW("after swap back"); SHOW(ar1); SHOW(ar2);
    }
}

namespace hh {
template class PArray<unsigned,4>;
template class PArray<double,4>;
template class PArray<const int*, 4>;

using U = unique_ptr<int>;
template<> PArray<U,4>::PArray(const PArray<U,4>&) : ArrayView() { }       // non-&& definition illegal
template<> PArray<U,4>::PArray(CArrayView<U>) : ArrayView() { }            // non-&& definition illegal
template<> PArray<U,4>::PArray(std::initializer_list<U>) : ArrayView() { } // definition illegal
template<> PArray<U,4>& PArray<U,4>::operator=(const PArray<U,4>&) { return *this; } // non-&& definition illegal
template<> PArray<U,4>& PArray<U,4>::operator=(CArrayView<U>) { return *this; } // non-&& definition illegal
template<> void PArray<U,4>::push(const U&) { }                                 // non-&& definition illegal
template<> void PArray<U,4>::push(CArrayView<U>) { }                            // non-&& definition illegal
template<> void PArray<U,4>::unshift(const U&) { }                              // non-&& definition illegal
template<> void PArray<U,4>::unshift(CArrayView<U>) { }                         // non-&& definition illegal
template class PArray<U,4>;
} // namespace hh
