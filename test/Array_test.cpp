// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Array.h"

#include <print>  // std::println.
#include <vector>

#include "libHh/ArrayOp.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Vec.h"
using namespace hh;

namespace {

void test_inlined_array() {
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
  {
    static_assert(std::is_same_v<InlinedArray<int, 4>, GeneralArray<int, 4>>);
    static_assert(std::is_same_v<Array<int>, GeneralArray<int, 0>>);
    // Moves from the built-in storage and from the heap, and changes of capacity across its boundary.
    InlinedArray<int, 3> ar1{1, 2};
    InlinedArray<int, 3> ar2{3, 4, 5, 6};
    InlinedArray<int, 3> ar3 = std::move(ar1);
    // NOLINTNEXTLINE(bugprone-use-after-move, clang-analyzer-cplusplus.Move): it checks the moved-from state.
    SHOW(ar3, ar1.num(), ar3.capacity());
    InlinedArray<int, 3> ar4 = std::move(ar2);
    // NOLINTNEXTLINE(bugprone-use-after-move, clang-analyzer-cplusplus.Move): it checks the moved-from state.
    SHOW(ar4, ar2.num(), ar4.capacity());
    ar4.resize(2);
    ar4.shrink_to_fit();  // Moves the elements back into the built-in storage.
    SHOW(ar4, ar4.capacity());
    ar4.reserve(10);
    SHOW(ar4, ar4.capacity());
    ar4.clear();
    SHOW(ar4.num(), ar4.capacity());
  }
}

// Applies the same random sequence of operations to a GeneralArray and to a std::vector as a reference model.
template <int inline_capacity> void verify_against_vector() {
  GeneralArray<int, inline_capacity> ar;
  std::vector<int> vec;
  Random random{17 + inline_capacity};
  const auto rand_int = [&](int ub) { return int(random.get_unsigned(unsigned(ub))); };  // Range [0, ub - 1].
  int max_capacity = 0;
  for_int(iter, 4000) {
    const int n = ar.num();
    const int value = rand_int(10);
    const int k = rand_int(4);
    switch (rand_int(18)) {
      case 0:
      case 1: ar.push(value), vec.push_back(value); break;
      case 2:
        if (n) assertx(ar.pop() == vec.back()), vec.pop_back();
        break;
      case 3: {
        const int i = rand_int(n + 1);
        ar.insert(i, k);
        vec.insert(vec.begin() + i, k, 0);
        for_int(j, k) {
          const int ij = i + j;
          ar[ij] = vec[size_t(ij)] = value + j;
        }
        break;
      }
      case 4: {
        const int i = rand_int(n + 1), k2 = min(k, n - i);
        ar.erase(i, k2);
        vec.erase(vec.begin() + i, vec.begin() + i + k2);
        break;
      }
      case 5: {
        const int i = rand_int(n + 1), k2 = min(k, n - i);
        ar.erase(ar.data() + i, ar.data() + i + k2);
        vec.erase(vec.begin() + i, vec.begin() + i + k2);
        break;
      }
      case 6: {
        const auto it = ranges::find(vec, value);
        assertx(ar.remove_ordered(value) == (it != vec.end()));
        if (it != vec.end()) vec.erase(it);
        break;
      }
      case 7: {
        const auto it = ranges::find(vec, value);
        assertx(ar.remove_unordered(value) == (it != vec.end()));
        if (it != vec.end()) *it = vec.back(), vec.pop_back();  // The last element fills the hole.
        break;
      }
      case 8:
        if (n) assertx(ar.shift() == vec.front()), vec.erase(vec.begin());
        break;
      case 9: ar.unshift(value), vec.insert(vec.begin(), value); break;
      case 10: {
        const int m = rand_int(n + 6);
        ar.resize(m);
        vec.resize(size_t(m));
        for_intL(i, n, m) ar[i] = vec[size_t(i)] = value;  // The new elements are unspecified.
        break;
      }
      case 11: {
        const int i = rand_int(n + 4);
        ar.access(i);
        if (i >= n) vec.resize(size_t(i) + 1);
        for_intL(j, n, i + 1) ar[j] = vec[size_t(j)] = value;  // The new elements are unspecified.
        assertx(ar.num() == max(n, i + 1));
        break;
      }
      case 12: {
        assertx(ar.add(k) == n);
        const int new_n = n + k;
        vec.resize(size_t(new_n), value);
        for_intL(i, n, n + k) ar[i] = value;  // The new elements are unspecified.
        break;
      }
      case 13: {
        const int k2 = min(k, n);
        ar.sub(k2);
        vec.resize(size_t(n - k2));
        break;
      }
      case 14: {
        const int k2 = min(k, n);
        const Array<int> popped = ar.pop(k2);
        assertx(ranges::equal(popped, vec | views::drop(n - k2)));
        vec.resize(size_t(n - k2));
        break;
      }
      case 15: {
        const int k2 = min(k, n);
        const Array<int> shifted = ar.shift(k2);
        assertx(ranges::equal(shifted, vec | views::take(k2)));
        vec.erase(vec.begin(), vec.begin() + k2);
        break;
      }
      case 16: {
        const int s = rand_int(30);
        const int old_capacity = ar.capacity();
        ar.reserve(s);
        assertx(ar.capacity() == max(old_capacity, s));
        break;
      }
      case 17:
        if (k == 0) {
          ar.shrink_to_fit();
          assertx(ar.capacity() == max(ar.num(), inline_capacity));
        } else if (k == 1) {
          ar.clear(), vec.clear();
          assertx(ar.capacity() == inline_capacity);
        } else if (k == 2) {
          const int m = rand_int(8);
          ar.init(m, value), vec.assign(size_t(m), value);
        } else {
          ar.push_array(V(value, value + 1).view());
          vec.insert(vec.end(), {value, value + 1});
        }
        break;
      default: assertnever("");
    }
    assertx(ar.num() == int(vec.size()) && ranges::equal(ar, vec));
    assertx(ar.capacity() >= max(ar.num(), inline_capacity));
    max_capacity = max(max_capacity, ar.capacity());
  }
  assertx(max_capacity > inline_capacity);  // The test exercised the heap storage.
}

void test_views() {
  Array<int> ar{0, 1, 2, 3, 4, 5};
  // Subviews of a mutable array are mutable, and those of a const array are const.
  static_assert(std::is_same_v<decltype(ar.head(1)), ArrayView<int>>);
  static_assert(std::is_same_v<decltype(std::as_const(ar).head(1)), CArrayView<int>>);
  static_assert(std::is_same_v<decltype(ar.segment(1, 2)), ArrayView<int>>);
  static_assert(std::is_same_v<decltype(CArrayView<int>(ar).tail(1)), CArrayView<int>>);
  static_assert(std::is_same_v<decltype(std::as_const(ar).last()), const int&>);
  static_assert(std::is_same_v<decltype(*std::as_const(ar).begin()), const int&>);
  // A const array or a CArrayView never yields a modifiable view.
  static_assert(!std::is_constructible_v<ArrayView<int>, const Array<int>&>);
  static_assert(!std::is_constructible_v<ArrayView<int>, CArrayView<int>>);
  static_assert(!std::is_constructible_v<ArrayView<int>, const ArrayView<int>&>);
  static_assert(std::is_convertible_v<Array<int>&, ArrayView<int>>);
  static_assert(std::is_convertible_v<const Array<int>&, CArrayView<int>>);
  // A view is reseated only from an rvalue into an lvalue; `arview = array` and `matrix[0] = matrix[1]` are
  // ill-formed, so that copying elements must be explicit through assign().
  static_assert(!std::is_assignable_v<ArrayView<int>&, Array<int>&>);
  static_assert(!std::is_assignable_v<ArrayView<int>&, ArrayView<int>&>);
  static_assert(!std::is_assignable_v<ArrayView<int>, ArrayView<int>>);
  static_assert(std::is_assignable_v<ArrayView<int>&, ArrayView<int>>);
  static_assert(!std::is_assignable_v<CArrayView<int>&, const CArrayView<int>&>);
  // The copy constructor of an Array is explicit, and absent for move-only elements.
  static_assert(std::is_constructible_v<Array<int>, const Array<int>&>);
  static_assert(!std::is_convertible_v<const Array<int>&, Array<int>>);
  static_assert(!std::is_constructible_v<Array<unique_ptr<int>>, const Array<unique_ptr<int>>&>);
  static_assert(std::is_nothrow_move_constructible_v<Array<int>> && std::is_nothrow_move_assignable_v<Array<int>>);
  // The deduction guides.
  static_assert(std::is_same_v<decltype(CArrayView(std::as_const(ar).data(), 2)), CArrayView<int>>);
  static_assert(std::is_same_v<decltype(ArrayView(ar.data(), 2)), ArrayView<int>>);
  static_assert(std::is_same_v<decltype(Array(std::vector<float>{})), Array<float>>);

  assertx(ar.head(2) == V(0, 1).view() && ar.tail(2) == V(4, 5).view());
  assertx(ar.segment(1, 3) == V(1, 2, 3).view() && ar.slice(2, 4) == V(2, 3).view());
  assertx(ar.head(0).num() == 0 && ar.tail(0).num() == 0 && ar.segment(6, 0).num() == 0);
  assertx(ar.tail(0).data() == ar.data() + 6);
  assertx(ar.head(4).tail(2) == V(2, 3).view());  // Subviews compose.
  assertx(ar.last() == 5 && &ar.last() == &ar[5]);
  ar.head(2).last() = 10;  // Writes through the view.
  assertx(ar[1] == 10);
  assertx(ar.ok(0) && ar.ok(5) && !ar.ok(-1) && !ar.ok(6));
  assertx(ar.ok(&ar[3]) && !ar.ok(ar.data() + 6));
  assertx(ar.size() == 6 && same_size(ar, V(1, 2, 3, 4, 5, 6).view()) && !same_size(ar, ar.head(2)));
  // Overlap of views.
  assertx(!have_overlap(ar.head(3), ar.tail(3)));
  assertx(have_overlap(ar.head(4), ar.tail(3)) && have_overlap(ar.tail(3), ar.head(4)));
  assertx(have_overlap(ar.segment(1, 4), ar.segment(2, 1)));
  assertx(!have_overlap(ar.head(0), ar.tail(6)));
  // ArView() views a single element.
  int x = 3;
  ArView(x)[0] = 4;
  assertx(x == 4 && ArView(x).num() == 1);
  // assign() copies elements into a view of the same size.
  ar.head(3).assign(V(7, 8, 9));
  assertx(ar == V(7, 8, 9, 3, 4, 5).view());
  ar.tail(2).assign(ar.tail(2));  // Assigning a view to itself is a no-op.
  assertx(ar == V(7, 8, 9, 3, 4, 5).view());
  // reinit() reseats a view.
  ArrayView<int> view = ar.head(2);
  view.reinit(ar.tail(3));
  assertx(view.num() == 3 && view.data() == ar.data() + 3);
  CArrayView<int> cview = ar.head(1);
  cview.reinit(ar.segment(2, 2));
  assertx(cview == V(9, 3).view());
  // Equality compares sizes and then elements.
  assertx(ar.head(0) == CArrayView<int>(nullptr, 0));
  assertx(ar.head(2) != ar.head(3) && ar.head(2) != ar.tail(2));
}

void test_boundary_rules() {
  for (const Bndrule bndrule :
       {Bndrule::reflected, Bndrule::periodic, Bndrule::clamped, Bndrule::border, Bndrule::reflected101}) {
    const std::string_view name = boundaryrule_name(bndrule);
    assertx(parse_boundaryrule(name) == bndrule);
    assertx(parse_boundaryrule(name.substr(0, 1)) == bndrule);  // The first letter suffices.
    SHOW(bndrule);
  }
  SHOW(Bndrule::undefined);
  // A reference model of the boundary rules, for n >= 1; it returns -1 for a border index outside the domain.
  const auto model = [](int i, int n, Bndrule bndrule) {
    const auto mod = [](int a, int b) { return ((a % b) + b) % b; };
    switch (bndrule) {
      case Bndrule::reflected: {
        const int j = mod(i, 2 * n);
        return j < n ? j : 2 * n - 1 - j;
      }
      case Bndrule::periodic: return mod(i, n);
      case Bndrule::clamped: return clamp(i, 0, n - 1);
      case Bndrule::border: return i >= 0 && i < n ? i : -1;
      case Bndrule::reflected101: {
        if (n == 1) return 0;
        const int j = mod(i, 2 * n - 2);
        return j < n ? j : 2 * n - 2 - j;
      }
      default: assertnever("");
    }
  };
  for (const Bndrule bndrule :
       {Bndrule::reflected, Bndrule::periodic, Bndrule::clamped, Bndrule::border, Bndrule::reflected101}) {
    for (const int n : {1, 2, 3, 5}) {
      for_intL(i0, -3 * n - 2, 4 * n + 2) {
        int i = i0;
        const bool inside = map_boundaryrule_1d(i, n, bndrule);
        const int expected = model(i0, n, bndrule);
        assertx(inside == (expected >= 0));
        if (inside) assertx(i == expected);
      }
    }
    string s;
    for_intL(i, -5, 9) {
      int i2 = i;
      s += map_boundaryrule_1d(i2, 4, bndrule) ? string(1, char('0' + i2)) : "-";
    }
    SHOW(bndrule, s);  // The mapped indices for i in [-5, 8] with n == 4.
  }
  // The functions are constexpr.
  static_assert(parse_boundaryrule("periodic") == Bndrule::periodic);
  static_assert([] {
    int i = -1;
    return map_boundaryrule_1d(i, 4, Bndrule::reflected101) && i == 1;
  }());
  // CArrayView::inside() accesses the array according to a boundary rule.
  const Array<int> ar{10, 11, 12};
  int i = 4;
  assertx(ar.map_inside(i, Bndrule::periodic) && i == 1);
  i = 5;
  assertx(!ar.map_inside(i, Bndrule::border));
  assertx(ar.inside(-1, Bndrule::reflected) == 10 && ar.inside(3, Bndrule::reflected) == 12);
  assertx(ar.inside(-1, Bndrule::reflected101) == 11 && ar.inside(3, Bndrule::reflected101) == 11);
  assertx(ar.inside(-1, Bndrule::periodic) == 12 && ar.inside(7, Bndrule::clamped) == 12);
  const int bordervalue = 99;
  assertx(ar.inside(-1, Bndrule::border, &bordervalue) == 99 && ar.inside(2, Bndrule::border, &bordervalue) == 12);
  assertx(&ar.inside(1, Bndrule::border, &bordervalue) == &ar[1]);
  Array<int> ar2{1, 2};
  ar2.inside(-1, Bndrule::clamped) = 5;  // A mutable array gives a mutable element.
  assertx(ar2[0] == 5);
}

void test_operations() {
  const Array<int> a{6, 7, 8}, b{1, 2, 3};
  assertx(a + b == V(7, 9, 11).view() && a - b == V(5, 5, 5).view() && a * b == V(6, 14, 24).view());
  assertx(a / b == V(6, 3, 2).view() && a % b == V(0, 1, 2).view() && -b == V(-1, -2, -3).view());
  assertx(a + 1 == V(7, 8, 9).view() && 10 - a == V(4, 3, 2).view() && a * 2 == V(12, 14, 16).view());
  assertx(a / 2 == V(3, 3, 4).view() && 24 / a == V(4, 3, 3).view() && a % 3 == V(0, 1, 2).view());
  assertx(min(a, V(7, 7, 7).view()) == V(6, 7, 7).view() && max(a, V(7, 7, 7).view()) == V(7, 7, 8).view());
  Array<int> c(a);
  c += b;  // The compound operators modify a view in place.
  assertx(c == V(7, 9, 11).view());
  c.head(2) -= 1;
  assertx(c == V(6, 8, 11).view());
  c *= 2, c /= V(2, 4, 11).view();
  assertx(c == V(6, 4, 2).view());
  c %= 4;
  assertx(c == V(2, 0, 2).view());
  // The binary operators allocate their result, also for empty arrays.
  assertx((Array<int>{} + Array<int>{}).num() == 0);
  // Interpolation, with weights that give exact results.
  const Array<float> f1{0.f, 4.f}, f2{8.f, 12.f}, f3{16.f, 20.f};
  assertx(interp(f1, f2) == V(4.f, 8.f).view());
  assertx(interp(f1, f2, .25f) == V(6.f, 10.f).view());
  assertx(interp(f1, f2, f3, .5f, .25f) == V(6.f, 10.f).view());
  SHOW(interp(f1, f2, f3));  // Equal weights of one third.
  // transformed() may change the element type.
  const auto halves = transformed(CArrayView<int>(a), [](int v) { return v * .5f; });
  static_assert(std::is_same_v<decltype(halves), const Array<float>>);
  assertx(halves == V(3.f, 3.5f, 4.f).view());
  const InlinedArray<int, 4> ia{1, 2};
  const auto ia2 = transformed(ia, [](int v) { return v > 1; });
  static_assert(std::is_same_v<decltype(ia2), const InlinedArray<bool, 4>>);
  assertx(ia2 == V(false, true).view());
}

void test_construction() {
  {
    const Array<int> ar(3, 7);
    assertx(ar == V(7, 7, 7).view() && ar.capacity() == 3);
    const Array<int> empty(0);
    assertx(empty.num() == 0 && empty.data() == nullptr && empty.begin() == empty.end());
    const Array<int> ar2(ar);  // NOLINT(performance-unnecessary-copy-initialization): the copy constructor is tested.
    assertx(ar2 == ar && ar2.data() != ar.data());
    Array<int> ar3;
    ar3 = ar.head(2);  // Assignment from a view.
    assertx(ar3 == V(7, 7).view());
    Array<int>& ref3 = ar3;
    ar3 = ref3;  // Self-assignment leaves the array unchanged.
    assertx(ar3 == V(7, 7).view());
    ar3.init(2);  // Retains the storage without reallocation when it is large enough.
    assertx(ar3.num() == 2 && ar3.capacity() == 2);
    ar3.init(5, 1);
    assertx(ar3 == V(1, 1, 1, 1, 1).view());
  }
  {
    // A move leaves the source empty.
    Array<int> ar1{1, 2, 3};
    const int* data = ar1.data();
    Array<int> ar2 = std::move(ar1);
    assertx(ar2.data() == data && ar2.num() == 3);
    // NOLINTNEXTLINE(bugprone-use-after-move, clang-analyzer-cplusplus.Move): it checks the moved-from state.
    assertx(ar1.num() == 0 && ar1.capacity() == 0 && ar1.data() == nullptr);
    ar1 = std::move(ar2);
    assertx(ar1.data() == data);
    // NOLINTNEXTLINE(bugprone-use-after-move, clang-analyzer-cplusplus.Move): it checks the moved-from state.
    assertx(ar2.num() == 0);
    Array<int> ar3{4};
    swap(ar1, ar3);
    assertx(ar1 == V(4).view() && ar3.data() == data);
  }
  {
    // The capacity grows geometrically, so that a sequence of n pushes performs O(log(n)) reallocations.
    Array<int> ar;
    int nrealloc = 0;
    for_int(i, 1000) {
      const int capacity = ar.capacity();
      ar.push(i);
      if (ar.capacity() != capacity) nrealloc++;
    }
    assertx(nrealloc <= 20 && ar.capacity() >= 1000);
    ar.shrink_to_fit();
    assertx(ar.capacity() == 1000 && ar[999] == 999);
    ar.clear();
    assertx(ar.num() == 0 && ar.capacity() == 0);
  }
  {
    // An array of move-only elements.
    Array<unique_ptr<int>> ar;
    for_int(i, 5) ar.push(make_unique<int>(i));
    ar.insert(1, 1);  // The element at the insertion point is left in a moved-from state, here nullptr.
    assertx(!ar[1] && *ar[2] == 1);
    ar[1] = make_unique<int>(10);
    ar.erase(3, 1);  // Removes 2.
    const unique_ptr<int> first = ar.shift();
    assertx(*first == 0);
    const unique_ptr<int> last = ar.pop();
    assertx(*last == 4);
    const Array<unique_ptr<int>> both = ar.pop(2);
    assertx(*both[0] == 1 && *both[1] == 3 && ar.num() == 1 && *ar[0] == 10);
    ar.push_array(Array<unique_ptr<int>>(2));  // The rvalue overload moves the elements.
    assertx(ar.num() == 3 && !ar[1] && !ar[2]);
    ar.access(3);
    assertx(ar.num() == 4);
    assertx(ar.remove_unordered(nullptr) && ar.num() == 3 && *ar[0] == 10);
  }
  {
    // Arrays of strings, whose elements are moved when the storage grows or the elements shift.
    Array<string> ar{"b", "c"};
    ar.unshift("a");
    for_int(i, 10) ar.push(string(20, char('d' + i)));  // Long strings, beyond the small-string optimization.
    assertx(ar[0] == "a" && ar[2] == "c" && ar.last() == string(20, 'm'));
    assertx(ar.remove_ordered("b") && !ar.remove_ordered("z"));
    assertx(ar[1] == "c" && ar.num() == 12);
  }
}

}  // namespace

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
    const Array<int> ar{1, 2, 3, 4};
    SHOW(sum(ar));
    const Array<int>& ar2 = ar;
    SHOW(sum(ar2));
    SHOW(min(ar2));
    SHOW(max(ar2));
    SHOW(mean(ar2));
  }
  {
    const Array<uchar> ar = {'a', 'd'};
    SHOW(sum(ar));
    SHOW(min(ar));
    SHOW(mean(ar));
    SHOW(mag2(ar));
    SHOW(mag(ar));
    SHOW(rms(ar));
  }
  {
    int a[5] = {10, 11, 12, 13, 14};  // Test C-array.
    SHOW(CArrayView<int>(a));
    SHOW(CArrayView(a));
    CArrayView<int> ar(a);
    SHOW(var(ar));
    SHOW(sqrt(var(ar)));
    SHOW(rms(ar - 12));  // Note that rms() and var() have slightly different denominators.
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
    const Array<int> ar{8, 7, 6, 5, 4, 9, 10};
    for_int(i, ar.num()) SHOW(i, rank_element(ar, i));
    for (const double rankf : {0., .1, .2, .3, .4, .5, .6, .7, .8, .9, 1.}) SHOW(rankf, rankf_element(ar, rankf));
  }
  {
    const Array ar1{1, 2};
    SHOW(ar1 == V(1, 1 + 1).view());
    SHOW(ar1 == V(1, 2, 3).view());
    SHOW(ar1 == V(1, 3).view());
  }
  if (0) {
    Array<int> ar(2, -1);
    SHOW(ar[2]);  // Out-of-bounds error.
  }
  {
    using Array3 = Vec3<Array<int>>;
    Array3 ar;
    ar[0].push(1);
    SHOW(ar);
    // Not portable: Array's copy constructor is explicit, and GCC (unlike Clang) does not accept it for the
    // direct-initialization of the array elements in the implicit copy constructor of Vec.
    // Array3 ar2(ar);
    // SHOW(ar2);
  }
  {
    Array ar(std::vector{1, 2});
    SHOW(ar);
  }
  {
    std::vector vec{1, 3};
    SHOW(Array(ranges::subrange(vec.begin(), vec.end())));
    SHOW(Array(ranges::subrange(vec)));
  }
  {
    std::vector vec{1, 2, 3};
    fill(ArrayView(vec.data(), 2), 10);
    SHOW(Array(vec));
  }
  test_inlined_array();  // Before the blocks that print to stdout, whose output order varies by platform.
  verify_against_vector<0>();
  verify_against_vector<3>();
  test_views();
  test_boundary_rules();
  test_operations();
  test_construction();
  {
    // (1) Move-only elements from a source that is not an Array<T>, so push_array(type&&) does not apply.
    std::vector<std::unique_ptr<int>> src;
    for (const int i : range(3)) src.push_back(std::make_unique<int>(i));
    Array<std::unique_ptr<int>> all;
    all.push_array(src | views::as_rvalue);
    printf("(1) all=%d,%d,%d   src nulled=%d%d%d\n", *all[0], *all[1], *all[2], !src[0], !src[1], !src[2]);

    // (2) Move only part of an Array, avoiding string copies.
    Array<std::string> words{"alpha", "beta", "gamma", "delta"};
    Array<std::string> tail;
    tail.push_array(ranges::subrange(words.tail(2)) | views::as_rvalue);
    printf("(2) tail=%s,%s   words[2..3]='%s','%s' (emptied)\n",  //
           tail[0].c_str(), tail[1].c_str(), words[2].c_str(), words[3].c_str());

    // (3) Move a filtered subset; std::move() cannot express this at all.
    Array<std::string> pool{"keep_a", "drop", "keep_b"};
    Array<std::string> kept;
    kept.push_array(pool | views::filter([](const std::string& s) { return s.starts_with("keep"); }) |
                    views::as_rvalue);
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
    for (const int i : range(3)) src.push_back(std::make_unique<int>(i));
    Array<std::unique_ptr<int>> all;
    all.push_array(src | views::as_rvalue);
    std::println("(1) all={},{},{}   src nulled={:d}{:d}{:d}", *all[0], *all[1], *all[2], !src[0], !src[1], !src[2]);

    // (2) Move only part of an Array, avoiding string copies.
    Array<std::string> words{"alpha", "beta", "gamma", "delta"};
    Array<std::string> tail;
    tail.push_array(ranges::subrange(words.tail(2)) | views::as_rvalue);
    std::println("(2) tail={},{}   words[2..3]='{}','{}' (emptied)", tail[0], tail[1], words[2], words[3]);

    // (3) Move a filtered subset; std::move() cannot express this at all.
    Array<std::string> pool{"keep_a", "drop", "keep_b"};
    Array<std::string> kept;
    kept.push_array(pool | views::filter([](const std::string& s) { return s.starts_with("keep"); }) |
                    views::as_rvalue);
    std::println("(3) kept={},{}   pool[0]='{}' (emptied), pool[1]='{}' (untouched)",  //
                 kept[0], kept[1], pool[0], pool[1]);

    // (4) Without as_rvalue, the same call copies and the source is intact.
    Array<std::string> copied;
    copied.push_array(ranges::subrange(words.head(2)));
    std::println("(4) copied={},{}   words[0]='{}' (intact)", copied[0], copied[1], words[0]);
  }
  {
    static_assert(ranges::view<CArrayView<int>> && ranges::view<ArrayView<int>>);
    static_assert(ranges::borrowed_range<CArrayView<int>> && ranges::borrowed_range<ArrayView<int>>);
    static_assert(!ranges::view<Array<int>> && !ranges::borrowed_range<Array<int>>);
    static_assert(!ranges::view<InlinedArray<int, 4>> && !ranges::borrowed_range<InlinedArray<int, 4>>);
    // Algorithms on a temporary view return a usable iterator; on an owning container, ranges::dangling.
    static_assert(std::same_as<ranges::borrowed_iterator_t<CArrayView<int>>, const int*>);
    static_assert(std::same_as<ranges::borrowed_iterator_t<Array<int>>, ranges::dangling>);
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

template class hh::GeneralArray<unsigned>;
template class hh::GeneralArray<double>;
template class hh::GeneralArray<const int*>;
template class hh::GeneralArray<unique_ptr<int>>;

template class hh::GeneralArray<unsigned, 4>;
template class hh::GeneralArray<double, 10>;
template class hh::GeneralArray<const int*, 100>;
template class hh::GeneralArray<unique_ptr<int>, 2>;
