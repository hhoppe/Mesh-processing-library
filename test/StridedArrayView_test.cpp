// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/StridedArrayView.h"

#include "libHh/Array.h"
#include "libHh/GridOp.h"  // grid_column()
#include "libHh/Matrix.h"
#include "libHh/RangeOp.h"
using namespace hh;

int main() {
  {
    Array<int> ar(200, 0);
    assertx(sum(ar) == 0);
    StridedArrayView<int> ar10(ar.data(), 5, 10);
    assertx(&ar10[3] == &ar[30]);
    for (int& e : ar10) e = 7;
    assertx(ar[0] == 7);
    assertx(ar[10] == 7);
    assertx(ar[20] == 7);
    assertx(ar[30] == 7);
    assertx(ar[40] == 7);
    assertx(ar[50] == 0);
    assertx(sum(ar) == 5 * 7);
    const CStridedArrayView<int> ar10b(ar10);
    assertx(sum(ar10b) == 5 * 7);
    assertx(ar10b.num() == 5);
    CStridedArrayView<int> ar10c(ar.data(), 5, 10);
    assertx(&ar10c[3] == &ar[30]);
    for (const int e : ar10c) assertx(e == 7);
  }
  {
    // Non-owning views that can be piped and are safe to return iterators from.
    static_assert(ranges::view<CStridedArrayView<int>> && ranges::view<StridedArrayView<int>>);
    static_assert(ranges::borrowed_range<CStridedArrayView<int>> && ranges::borrowed_range<StridedArrayView<int>>);
    static_assert(ranges::viewable_range<CStridedArrayView<int>>);  // The rvalue pipes.
    static_assert(ranges::random_access_range<CStridedArrayView<int>> && ranges::sized_range<CStridedArrayView<int>>);
    static_assert(
        std::same_as<ranges::borrowed_iterator_t<CStridedArrayView<int>>, ranges::iterator_t<CStridedArrayView<int>>>);
    // Elementwise intent must be explicit; reseating requires an rvalue and an lvalue object.
    static_assert(!std::is_assignable_v<StridedArrayView<int>&, StridedArrayView<int>&>);
    static_assert(!std::is_assignable_v<StridedArrayView<int>, StridedArrayView<int>>);
    static_assert(std::is_assignable_v<StridedArrayView<int>&, StridedArrayView<int>&&>);
  }
  {
    Matrix<int> matrix(V(3, 4));
    for_int(y, 3) for_int(x, 4) matrix[y, x] = y * 10 + x;
    SHOW(Array(grid_column(matrix, 0, V(0, 1)) | views::transform([](int v) { return v * 2; })));
    SHOW(sum(grid_column(matrix, 1, V(2, 0))));  // Strided view consumed directly as an rvalue.  20 + 21 + 22 + 23.
  }
  {
    // An iterator converts to a const_iterator but not the reverse, and the two interoperate.
    using It = StridedArrayView<int>::iterator;
    using CIt = CStridedArrayView<int>::iterator;
    static_assert(std::same_as<CIt, StridedArrayView<int>::const_iterator>);
    static_assert(std::convertible_to<It, CIt> && !std::convertible_to<CIt, It>);
    Array<int> ar(20);
    for_int(i, 20) ar[i] = i;
    StridedArrayView<int> v(ar.data() + 1, 5, 4);  // 1, 5, 9, 13, 17.
    const It it = v.begin();
    const CIt cit = it;               // The converting constructor reads the private members of It.
    assertx(cit == it && it == cit);  // The second comparison uses the reversed candidate.
    assertx(cit - it == 0);  // Mixed-type subtraction works only in this direction (operator- is not rewritten).
    assertx(cit + v.num() == v.end());
    assertx(*(cit + 2) == v[2] && cit < v.end());
    *it = -1;  // The mutable iterator still writes through.
    assertx(*cit == -1 && ar[1] == -1);
  }
  {
    // The accessors, and the output operator.
    Array<int> ar(12);
    for_int(i, ar.num()) ar[i] = i;
    StridedArrayView<int> v(ar.data() + 2, 4, 3);  // 2, 5, 8, 11.
    assertx(v.num() == 4 && v.size() == 4 && v.stride() == 3);
    assertx(v.data() == ar.data() + 2);
    assertx(v.ok(0) && v.ok(3) && !v.ok(4) && !v.ok(-1));
    assertx(&v.last() == &ar[11]);
    v.last() = 110;  // The last element is modifiable.
    SHOW(v);
    const CStridedArrayView<int> cv = v;  // Conversion to the base class.
    assertx(&cv[1] == &ar[5] && cv.last() == 110);
    // A const StridedArrayView, like a CStridedArrayView, yields const elements.
    const StridedArrayView<int> v_const = v;
    static_assert(std::is_same_v<decltype(v[0]), int&>);
    static_assert(std::is_same_v<decltype(v_const[0]), const int&>);
    static_assert(std::is_same_v<decltype(cv[0]), const int&>);
    static_assert(std::is_same_v<decltype(*v_const.begin()), const int&>);
    static_assert(std::is_same_v<decltype(v.last()), int&> && std::is_same_v<decltype(cv.last()), const int&>);
    static_assert(!std::is_constructible_v<StridedArrayView<int>, CStridedArrayView<int>>);
  }
  {
    // A stride of one is equivalent to an ArrayView, and an empty view has no elements.
    Array<int> ar{3, 1, 2};
    const CStridedArrayView<int> v1(ar.data(), ar.num(), 1);
    assertx(ranges::equal(v1, ar));
    const CStridedArrayView<int> v0(ar.data(), 0, 5);
    assertx(v0.num() == 0 && v0.begin() == v0.end() && ranges::empty(v0));
  }
  {
    // The random-access iterator operations, compared with the indices of the underlying array.
    Array<int> ar(30);
    for_int(i, ar.num()) ar[i] = i;
    const CStridedArrayView<int> v(ar.data() + 1, 6, 5);  // 1, 6, 11, 16, 21, 26.
    using CIt = CStridedArrayView<int>::iterator;
    static_assert(std::random_access_iterator<CIt>);
    CIt it = v.begin();
    assertx(*it == 1 && it[3] == 16 && *(it + 5) == 26 && *(2 + it) == 11);
    it += 4;
    assertx(*it == 21 && it - v.begin() == 4 && v.begin() - it == -4 && v.end() - it == 2);
    it -= 3;
    assertx(*it == 6 && *(it - 1) == 1);
    assertx(*it++ == 6 && *it == 11 && *it-- == 11 && *it == 6);
    assertx(*++it == 11 && *--it == 6);
    assertx(v.begin() < it && v.begin() <= it && it >= v.begin() && v.end() > it && !(it < v.begin()));
    assertx(ranges::distance(v) == 6 && v.end() - v.begin() == v.num());
    assertx(*ranges::find(v, 16) == 16 && ranges::find(v, 17) == v.end());  // 17 lies between the strided elements.
    SHOW(Array(v | views::reverse));
    SHOW(Array(v | views::drop(2) | views::stride(2)));
  }
  {
    // Algorithms that modify the elements through the view touch only the strided elements.
    Array<int> ar{50, 0, 40, 0, 30, 0, 20, 0, 10, 0};
    StridedArrayView<int> v(ar.data(), 5, 2);
    ranges::sort(v);
    SHOW(ar);
    ranges::reverse(v);
    assertx(ar == V(50, 0, 40, 0, 30, 0, 20, 0, 10, 0).view());
    ranges::fill(StridedArrayView<int>(ar.data() + 1, 5, 2), 7);
    assertx(ar == V(50, 7, 40, 7, 30, 7, 20, 7, 10, 7).view());
  }
}

template class hh::CStridedArrayView<unsigned>;
template class hh::CStridedArrayView<unique_ptr<int>>;

template class hh::StridedArrayView<unsigned>;
template class hh::StridedArrayView<unique_ptr<int>>;
