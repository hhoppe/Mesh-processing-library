// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/InlinedArray.h"
using namespace hh;

int main() {
  struct S {
    explicit S(int i) : _i(i) { showf("S(%d)\n", _i); }
    ~S() { showf("~S(%d)\n", _i); }
    int _i;
  };
  const auto func_construct_array = [](int i0, int n) {  // -> InlinedArray<unique_ptr<S>, 2>
    InlinedArray<unique_ptr<S>, 2> ar;
    for_int(i, n) ar.push(make_unique<S>(i0 + i));
    return ar;
  };
  {
    SHOW("beg");
    InlinedArray<unique_ptr<S>, 2> ar;
    ar.push(make_unique<S>(4));
    SHOW("end");
  }
  {
    SHOW("beg");
    InlinedArray<unique_ptr<S>, 2> ar;
    ar.push(make_unique<S>(4));
    ar.push(make_unique<S>(5));
    SHOW("end");
  }
  {
    SHOW("beg");
    InlinedArray<unique_ptr<S>, 2> ar;
    ar.push(make_unique<S>(4));
    ar.push(make_unique<S>(5));
    ar.push(make_unique<S>(6));
    for (auto& e : ar) SHOW(e->_i);
    SHOW("end");
  }
  {
    SHOW("beg");
    InlinedArray<unique_ptr<S>, 2> ar;
    for_int(i, 20) ar.push(make_unique<S>(i));
    SHOW("end");
  }
  {
    SHOW("beg");
    const InlinedArray<unique_ptr<S>, 2> ar(func_construct_array(100, 2));
    SHOW("end");
  }
  {
    SHOW("beg");
    InlinedArray<unique_ptr<S>, 2> ar;
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
    InlinedArray<int, 3> ar1;
    SHOW(ar1);
    ar1.push(7);
    ar1.push(6);
    ar1.push(5);
    SHOW(ar1);
    ar1.push(4);
    ar1.push(3);
    SHOW(ar1);
    const auto func = [](int v) { return v * 1.5f; };
    SHOW(transformed(ar1, func));
    InlinedArray<int, 3> ar2;
    ar2.push(11);
    ar2.push(12);
    SHOW(ar2);
    ranges::swap(ar1, ar2);
    SHOW("after swap");
    SHOW(ar1);
    SHOW(ar2);
    swap(ar1, ar2);
    SHOW("after swap back");
    SHOW(ar1);
    SHOW(ar2);
    ar2.push(13);
    ar2.push(14);
    ar2.push(15);
    SHOW(ar2);
    swap(ar1, ar2);
    SHOW("after swap");
    SHOW(ar1);
    SHOW(ar2);
    ranges::swap(ar1, ar2);
    SHOW("after swap back");
    SHOW(ar1);
    SHOW(ar2);
    ar1.erase(0, 3);
    SHOW(ar1);
    ar2.erase(0, 3);
    SHOW(ar2);
    swap(ar1, ar2);
    SHOW("after swap");
    SHOW(ar1);
    SHOW(ar2);
    swap(ar1, ar2);
    SHOW("after swap back");
    SHOW(ar1);
    SHOW(ar2);
  }
  {
    InlinedArray<int, 2> ar1{1};
    SHOW(ar1);
    InlinedArray<int, 2> ar2{1, 2, 3};
    SHOW(ar2);
  }
}

template class hh::InlinedArray<unsigned, 4>;
template class hh::InlinedArray<double, 10>;
template class hh::InlinedArray<const int*, 100>;
template class hh::InlinedArray<unique_ptr<int>, 2>;
