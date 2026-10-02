// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Kdtree.h"

#include "libHh/Random.h"
#include "libHh/Set.h"
using namespace hh;

namespace {

// Returns true if the two boxes overlap, using the same strict inequalities as Kdtree::search().
template <int D>
bool boxes_overlap(const Vec<float, D>& a0, const Vec<float, D>& a1, const Vec<float, D>& b0,
                   const Vec<float, D>& b1) {
  for_int(c, D) {
    if (a0[c] >= b1[c] || a1[c] <= b0[c]) return false;
  }
  return true;
}

// Random boxes (mostly small, some extending outside the unit cube), searched with random query boxes, compared with
// a brute-force search.  With allow_duplication(), an element may be reported more than once.
template <int D> void test_random_searches(int maxlevel, float fsize) {
  using KD = Kdtree<int, D>;
  using VecD = Vec<float, D>;
  Random random(1);
  const auto random_box = [&](VecD& b0, VecD& b1) {  // (Exact scale factors avoid any dependence on FMA.)
    for_int(c, D) {
      const float max_size = random.get_unsigned(4) ? .0625f : .5f;
      const float size = random.unif() * max_size;
      b0[c] = random.unif() - .1f;
      b1[c] = b0[c] + size;
    }
  };
  KD kd(maxlevel);
  if (fsize) kd.allow_duplication(fsize);
  const int n = 300;
  Array<VecD> ar_bb0(n), ar_bb1(n);
  for_int(i, n) {
    random_box(ar_bb0[i], ar_bb1[i]);
    kd.enter(i, ar_bb0[i], ar_bb1[i]);
  }
  int num_found = 0, num_reports = 0;
  for_int(iquery, 300) {
    VecD q0, q1;
    random_box(q0, q1);
    Array<int> found;
    const auto func = [&](const int& id, VecD& bb0, VecD& bb1, KD::CBloc floc) {
      assertx(bb0 == q0 && bb1 == q1);
      found.push(id);
      if (found.num() == 1) {  // A nested search starting from location floc also finds this element.
        bool found_again = false;
        const auto func2 = [&](const int& id2, VecD&, VecD&, KD::CBloc) {
          found_again |= id2 == id;
          return KD::ECallbackReturn::nothing;
        };
        VecD bb0_copy = bb0, bb1_copy = bb1;
        assertx(!kd.search(bb0_copy, bb1_copy, func2, floc));
        assertx(found_again);
      }
      return KD::ECallbackReturn::nothing;
    };
    assertx(!kd.search(q0, q1, func));
    Array<int> expected;
    for_int(i, n) {
      if (boxes_overlap(q0, q1, ar_bb0[i], ar_bb1[i])) expected.push(i);
    }
    sort(found);
    Array<int> distinct;
    for (const int id : found) {
      if (!distinct.num() || distinct.last() != id) distinct.push(id);
    }
    if (!fsize) assertx(distinct.num() == found.num());  // Without duplication, each element is reported once.
    assertx(distinct == expected);
    num_found += distinct.num();
    num_reports += found.num();
  }
  SHOW(D, maxlevel, fsize, num_found, num_reports);
}

// Query boxes whose coordinates are multiples of 1/32, so that they often lie exactly on splitting planes, and which
// often have zero extent along some axes (e.g., the point queries of HiddenLineRemoval), compared with a brute-force
// search.
template <int D> void test_degenerate_searches(int maxlevel, float fsize) {
  using KD = Kdtree<int, D>;
  using VecD = Vec<float, D>;
  Random random(2);
  KD kd(maxlevel);
  if (fsize) kd.allow_duplication(fsize);
  const int n = 300;
  Array<VecD> ar_bb0(n), ar_bb1(n);
  for_int(i, n) {
    for_int(c, D) {  // (Separate statements give a defined order of the random calls.)
      const float max_size = random.get_unsigned(4) ? .0625f : .5f;
      ar_bb0[i][c] = random.unif();
      ar_bb1[i][c] = ar_bb0[i][c] + random.unif() * max_size;
    }
    kd.enter(i, ar_bb0[i], ar_bb1[i]);
  }
  int num_found = 0;
  for_int(iquery, 1000) {
    VecD q0, q1;
    for_int(c, D) {
      q0[c] = float(random.get_unsigned(33)) / 32.f;
      q1[c] = random.get_unsigned(2) ? q0[c] : std::min(q0[c] + float(random.get_unsigned(4)) / 32.f, 1.f);
    }
    Set<int> found;
    assertx(!kd.search(q0, q1, [&](const int& id, VecD&, VecD&, KD::CBloc) {
      found.add(id);
      return KD::ECallbackReturn::nothing;
    }));
    int num_expected = 0;
    for_int(i, n) {
      if (boxes_overlap(q0, q1, ar_bb0[i], ar_bb1[i])) {
        assertx(found.contains(i));
        num_expected++;
      }
    }
    assertx(found.num() == num_expected);
    num_found += num_expected;
  }
  SHOW(D, maxlevel, fsize, num_found);
}

void test_callback_returns() {
  using KD = Kdtree<int, 2>;
  KD kd;
  {
    Vec2<float> bb0(0.f, 0.f), bb1(1.f, 1.f);
    int ncalls = 0;
    assertx(!kd.search(bb0, bb1, [&](const int&, Vec2<float>&, Vec2<float>&, KD::CBloc) {
      ncalls++;
      return KD::ECallbackReturn::nothing;
    }));
    assertx(ncalls == 0);  // Searching an empty tree finds nothing.
  }
  for_int(i, 10) kd.enter(i, V(i * .1f, 0.f), V(i * .1f + .05f, 1.f));  // Ten thin vertical strips.
  {  // Returning `stop` ends the search, which then returns true.
    Vec2<float> bb0(0.f, .4f), bb1(1.f, .6f);
    int ncalls = 0;
    assertx(kd.search(bb0, bb1, [&](const int&, Vec2<float>&, Vec2<float>&, KD::CBloc) {
      ncalls++;
      return KD::ECallbackReturn::stop;
    }));
    assertx(ncalls == 1);
  }
  {  // Returning `bbshrunk` after shrinking the query box limits the subsequent results to the smaller box.
    Vec2<float> bb0(0.f, .4f), bb1(1.f, .6f);
    Array<int> found;
    assertx(!kd.search(bb0, bb1, [&](const int& id, Vec2<float>& b0, Vec2<float>& b1, KD::CBloc) {
      assertx(b0[0] < id * .1f + .05f && b1[0] > id * .1f);  // The strip overlaps the current query box.
      found.push(id);
      // Shrink the box to exclude this strip and those beyond it.
      if (id < 5)
        b0[0] = std::max(b0[0], id * .1f + .05f);
      else
        b1[0] = std::min(b1[0], id * .1f);
      return KD::ECallbackReturn::bbshrunk;
    }));
    SHOW(found);
    sort(found);
    for_int(i, found.num() - 1) assertx(found[i] < found[i + 1]);  // No strip is reported twice.
  }
  kd.clear();
  {
    Vec2<float> bb0(0.f, 0.f), bb1(1.f, 1.f);
    int ncalls = 0;
    assertx(!kd.search(bb0, bb1, [&](const int&, Vec2<float>&, Vec2<float>&, KD::CBloc) {
      ncalls++;
      return KD::ECallbackReturn::nothing;
    }));
    assertx(ncalls == 0);  // A cleared tree is empty.
  }
  kd.enter(3, V(.2f, .2f), V(.3f, .3f));
  kd.print();
}

}  // namespace

int main() {
  {
    struct cbf {
      Kdtree<int, 1>::ECallbackReturn operator()(const int& id, Vec<float, 1>& bb0, Vec<float, 1>& bb1,
                                                 Kdtree<int, 1>::CBloc floc) const {
        dummy_use(bb0, bb1, floc);
        showf("found index %d\n", id);
        return Kdtree<int, 1>::ECallbackReturn::nothing;
      }
    };
    Kdtree<int, 1> kd(3);
    using BB = SGrid<float, 2, 1>;
    BB v;
    v = BB{{0.100f}, {0.200f}};
    kd.enter(1, v[0], v[1]);
    kd.print();
    SHOW("");
    v = BB{{0.700f}, {0.800f}};
    kd.enter(2, v[0], v[1]);
    kd.print();
    SHOW("");
    v = BB{{0.550f}, {0.900f}};
    kd.enter(3, v[0], v[1]);
    kd.print();
    SHOW("");
    v = BB{{0.450f}, {0.520f}};
    kd.enter(4, v[0], v[1]);
    kd.print();
    SHOW("");
    v = BB{{0.251f}, {0.252f}};
    kd.enter(5, v[0], v[1]);
    kd.print();
    SHOW("");
    v = BB{{0.150f}, {0.300f}};
    kd.search(v[0], v[1], cbf());
    SHOW("");
    v = BB{{0.400f}, {0.700f}};
    kd.search(v[0], v[1], cbf());
    SHOW("");
    v = BB{{0.900f}, {0.910f}};
    kd.search(v[0], v[1], cbf());
    SHOW("");
  }
  {
    Kdtree<int, 2> kd(5);
    using BB = SGrid<float, 2, 2>;
    BB v;
    v = BB{{0.100f, 0.510f}, {0.200f, 0.520f}};
    kd.enter(6, v[0], v[1]);
    kd.print();
    SHOW("");
    v = BB{{0.626f, 0.100f}, {0.627f, 0.900f}};
    kd.enter(7, v[0], v[1]);
    kd.print();
    SHOW("");
    v = BB{{0.100f, 0.626f}, {0.900f, 0.627f}};
    kd.enter(8, v[0], v[1]);
    kd.print();
    SHOW("");
    v = BB{{0.400f, 0.326f}, {0.430f, 0.327f}};
    kd.enter(9, v[0], v[1]);
    kd.print();
    SHOW("");
  }
  {
    using U = unique_ptr<int>;
    Kdtree<U, 2> kd;
    using BB = SGrid<float, 2, 2>;
    BB v;
    v = BB{{0.100f, 0.510f}, {0.200f, 0.520f}};
    kd.enter(make_unique<int>(), v[0], v[1]);
  }
  test_random_searches<1>(8, 0.f);
  test_random_searches<2>(8, 0.f);
  test_random_searches<2>(5, 1.f);
  test_random_searches<3>(6, 0.f);
  test_random_searches<3>(6, .5f);
  test_callback_returns();
  {
    // With allow_duplication(), a point query lying exactly on a splitting plane finds the element that straddles the
    // plane, although that element is stored only in the two child subtrees.
    using KD = Kdtree<int, 1>;
    KD kd;
    kd.allow_duplication(1.f);
    kd.enter(1, Vec1<float>(.4f), Vec1<float>(.6f));
    for (const float x : {.45f, .5f, .55f}) {
      Vec1<float> bb0(x), bb1(x);
      int nfound = 0;
      kd.search(bb0, bb1, [&](const int&, Vec1<float>&, Vec1<float>&, KD::CBloc) {
        nfound++;
        return KD::ECallbackReturn::nothing;
      });
      assertx(nfound == 1);
    }
  }
  test_degenerate_searches<1>(8, 0.f);
  test_degenerate_searches<1>(8, 1.f);
  test_degenerate_searches<2>(8, 0.f);
  test_degenerate_searches<2>(8, 1.f);
  test_degenerate_searches<3>(6, .5f);
}

template class hh::Kdtree<unsigned, 1>;
template class hh::Kdtree<float, 2>;
template class hh::Kdtree<double, 3>;
template class hh::Kdtree<unique_ptr<int>, 2>;
