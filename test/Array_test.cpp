// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Array.h"

#include <print>  // std::println.
#include <vector>

#include "libHh/ArrayOp.h"
#include "libHh/RangeOp.h"
#include "libHh/Vec.h"
using namespace hh;

int main() {
  struct S {
    explicit S(int i) : _i(i) { showf("S(%d)\n", _i); }
    ~S() { showf("~S(%d)\n", _i); }
    int _i;
  };
  const auto func_make_array = [](int i0, int n) {  // -> Array<unique_ptr<S>>
    Array<unique_ptr<S>> ar;
    for_int(i, n) ar.push(make_unique<S>(i0 + i));
    return ar;
  };
  {
    SHOW("beg 4");
    Array<unique_ptr<S>> ar;
    ar.push(make_unique<S>(4));
    SHOW("end");
  }
  {
    SHOW("beg 4 5");
    Array<unique_ptr<S>> ar;
    ar.push(make_unique<S>(4));
    ar.push(make_unique<S>(5));
    SHOW("end");
  }
  {
    SHOW("beg 4 5 6");
    Array<unique_ptr<S>> ar;
    ar.push(make_unique<S>(4));
    ar.push(make_unique<S>(5));
    ar.push(make_unique<S>(6));
    for (auto& e : ar) SHOW(e->_i);
    SHOW("end");
  }
  {
    SHOW("beg 20");
    Array<unique_ptr<S>> ar;
    for_int(i, 20) ar.push(make_unique<S>(i));
    SHOW("end");
  }
  {
    SHOW("beg 100, 2");
    auto ar = func_make_array(100, 2);
    SHOW("end");
  }
  {
    SHOW("beg 500");
    Array<unique_ptr<S>> ar;
    ar = func_make_array(500, 2);
    SHOW(ar[0]->_i);
    SHOW("beg 600");
    ar = func_make_array(600, 3);
    SHOW("end");
  }
  {
    SHOW("beg 100, 3");
    auto ar = func_make_array(100, 3);
    SHOW("beg 200, 2");
    ar = func_make_array(200, 2);
    SHOW("end");
  }
  {
    SHOW("beg 100, 3");
    auto ar = func_make_array(100, 3);
    SHOW("beg 200, 3");
    ar = func_make_array(200, 3);
    SHOW("end");
  }
  {
    SHOW(CArrayView<int>({1, 2, 3, 4}));
    Array<int> ar{1, 2, 3, 4};
    SHOW(sum(ar));
    const Array<int>& ar2 = ar;
    SHOW(sum(ar2));
    SHOW(min(ar2));
    SHOW(sum(ar2));
    SHOW(mean(ar2));
  }
  {
    Array<uchar> ar = {'a', 'd'};
    SHOW(sum(ar));
    SHOW(min(ar));
    SHOW(mean(ar));
    SHOW(mag2(ar));
    SHOW(mag(ar));
    SHOW(rms(ar));
  }
  {
    int a[5] = {10, 11, 12, 13, 14};  // test C-array
    SHOW(CArrayView<int>(a));
    SHOW(CArrayView(a));
    CArrayView<int> ar(a);
    SHOW(var(ar));
    SHOW(sqrt(var(ar)));
    SHOW(rms(ar - 12));  // rms() and var() have slightly different denominators
    SHOW(reverse(ArrayView(a)));
    SHOW(CArrayView(a));
    SHOW(sort(ArrayView(a)));
    SHOW(CArrayView(a));
    reverse(a);
    SHOW(CArrayView(a));
    fill(a, 16);
    SHOW(CArrayView(a));
  }
  {
    Vec3 a(10, 11, 12);
    SHOW(a);
    SHOW(CArrayView<int>(a));
    SHOW(a.view());
    CArrayView<int> ar(a);
    SHOW(var(ar));
    SHOW(sqrt(var(ar)));
    SHOW(rms(ar - 12));
  }
  {
    SHOW(sort_unique(V(10, 13, 12, 13, 9, 12, 15, 10)));
    SHOW(median(V(10, 13, 12, 13, 9, 12, 15, 10)));
    SHOW(median(V(8, 7, 6, 5, 4, 9, 10)));
    SHOW(median_two(V(10, 13, 12, 13, 9, 12, 15, 10)));
    SHOW(mean(median_two(V(10, 13, 12, 13, 9, 12, 15, 10))));
    SHOW(median_two(V(8, 7, 6, 5, 4, 9, 10)));
    SHOW(mean(median_two(V(8, 7, 6, 5, 4, 9, 10))));
  }
  {
    Array<int> ar{8, 7, 6, 5, 4, 9, 10};
    for_int(i, ar.num()) SHOW(i, rank_element(ar, i));
    for (double rankf : {0., .1, .2, .3, .4, .5, .6, .7, .8, .9, 1.}) SHOW(rankf, rankf_element(ar, rankf));
  }
  {
    Array ar1{1, 2};
    SHOW(ar1 == V(1, 1 + 1).view());
    SHOW(ar1 == V(1, 2, 3).view());
    SHOW(ar1 == V(1, 3).view());
  }
  if (0) {
    // Array<int> ar(5);
    // ArrayView<int> arv(ar); arv = ar;        // is illegal as expected
  }
  if (0) {
    Array<int> ar(2, -1);
    SHOW(ar[2]);  // out-of-bounds error
  }
  {
    using Array3 = Vec3<Array<int>>;
    Array3 ar;
    ar[0].push(1);
    SHOW(ar);
    // Not allowed because Array is not copy-constructible.
    // Array3 ar2(ar);
    // SHOW(ar2);
  }
  {
    Array ar(std::vector{1, 2});
    SHOW(ar);
  }
  {
    std::vector vec{1, 3};
    Array ar(ranges::subrange(vec.begin(), vec.end()));
    SHOW(ar);
  }
  {
    std::vector vec{1, 2, 3};
    fill(ArrayView(vec.data(), 2), 10);
    SHOW(Array(vec));
  }
  {
    // (1) Move-only elements from a source that is not an Array<T>, so push_array(type&&) does not apply.
    std::vector<std::unique_ptr<int>> src;
    for (int i : range(3)) src.push_back(std::make_unique<int>(i));
    Array<std::unique_ptr<int>> all;
    all.push_array(src | std::views::as_rvalue);
    printf("(1) all=%d,%d,%d   src nulled=%d%d%d\n", *all[0], *all[1], *all[2], !src[0], !src[1], !src[2]);

    // (2) Move only part of an Array, avoiding string copies.
    Array<std::string> words{"alpha", "beta", "gamma", "delta"};
    Array<std::string> tail;
    tail.push_array(ranges::subrange(words.tail(2)) | std::views::as_rvalue);
    printf("(2) tail=%s,%s   words[2..3]='%s','%s' (emptied)\n",  //
           tail[0].c_str(), tail[1].c_str(), words[2].c_str(), words[3].c_str());

    // (3) Move a filtered subset; std::move() cannot express this at all.
    Array<std::string> pool{"keep_a", "drop", "keep_b"};
    Array<std::string> kept;
    kept.push_array(pool | std::views::filter([](const std::string& s) { return s.starts_with("keep"); }) |
                    std::views::as_rvalue);
    printf("(3) kept=%s,%s   pool[0]='%s' (emptied), pool[1]='%s' (untouched)\n",  //
           kept[0].c_str(), kept[1].c_str(), pool[0].c_str(), pool[1].c_str());

    // (4) Without as_rvalue, the same call copies and the source is intact.
    Array<std::string> copied;
    copied.push_array(ranges::subrange(words.head(2)));
    printf("(4) copied=%s,%s   words[0]='%s' (intact)\n", copied[0].c_str(), copied[1].c_str(), words[0].c_str());
  }
  {
    // (1) Move-only elements from a source that is not an Array<T>, so push_array(type&&) does not apply.
    std::vector<std::unique_ptr<int>> src;
    for (int i : range(3)) src.push_back(std::make_unique<int>(i));
    Array<std::unique_ptr<int>> all;
    all.push_array(src | std::views::as_rvalue);
    std::println("(1) all={},{},{}   src nulled={:d}{:d}{:d}", *all[0], *all[1], *all[2], !src[0], !src[1], !src[2]);

    // (2) Move only part of an Array, avoiding string copies.
    Array<std::string> words{"alpha", "beta", "gamma", "delta"};
    Array<std::string> tail;
    tail.push_array(ranges::subrange(words.tail(2)) | std::views::as_rvalue);
    std::println("(2) tail={},{}   words[2..3]='{}','{}' (emptied)", tail[0], tail[1], words[2], words[3]);

    // (3) Move a filtered subset; std::move() cannot express this at all.
    Array<std::string> pool{"keep_a", "drop", "keep_b"};
    Array<std::string> kept;
    kept.push_array(pool | std::views::filter([](const std::string& s) { return s.starts_with("keep"); }) |
                    std::views::as_rvalue);
    std::println("(3) kept={},{}   pool[0]='{}' (emptied), pool[1]='{}' (untouched)",  //
                 kept[0], kept[1], pool[0], pool[1]);

    // (4) Without as_rvalue, the same call copies and the source is intact.
    Array<std::string> copied;
    copied.push_array(ranges::subrange(words.head(2)));
    std::println("(4) copied={},{}   words[0]='{}' (intact)", copied[0], copied[1], words[0]);
  }
}

template class hh::CArrayView<unsigned>;
template class hh::CArrayView<double>;
template class hh::CArrayView<const int*>;
template class hh::CArrayView<unique_ptr<int>>;

template class hh::ArrayView<unsigned>;
template class hh::ArrayView<double>;
template class hh::ArrayView<const int*>;
template class hh::ArrayView<unique_ptr<int>>;

template class hh::Array<unsigned>;
template class hh::Array<double>;
template class hh::Array<const int*>;
template class hh::Array<unique_ptr<int>>;
