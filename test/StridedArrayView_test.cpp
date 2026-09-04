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
    for (int e : ar10c) assertx(e == 7);
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
    It it = v.begin();
    CIt cit = it;                     // The converting constructor reads the private members of It.
    assertx(cit == it && it == cit);  // The second comparison uses the reversed candidate.
    assertx(cit - it == 0);  // Mixed-type subtraction works only in this direction (operator- is not rewritten).
    assertx(cit + v.num() == v.end());
    assertx(*(cit + 2) == v[2] && cit < v.end());
    *it = -1;  // The mutable iterator still writes through.
    assertx(*cit == -1 && ar[1] == -1);
  }
}

template class hh::CStridedArrayView<unsigned>;
template class hh::CStridedArrayView<unique_ptr<int>>;

template class hh::StridedArrayView<unsigned>;
template class hh::StridedArrayView<unique_ptr<int>>;
