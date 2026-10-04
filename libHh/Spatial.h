// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_SPATIAL_H_
#define MESH_PROCESSING_LIBHH_SPATIAL_H_

#include "libHh/Array.h"
#include "libHh/Bbox.h"
#include "libHh/Geometry.h"
#include "libHh/Map.h"
#include "libHh/PriorityQueue.h"
#include "libHh/Queue.h"
#include "libHh/Set.h"
#include "libHh/Stat.h"
#include "libHh/Univ.h"
#include "libHh/Vec.h"

namespace hh {

namespace details {
class BaseSpatialSearch;
}

// Set of the elements already entered into the priority queue of a spatial search.  Its built-in storage avoids
// heap allocation in most searches, which visit only a few elements.
using SpatialVisitedSet = InlinedSet<Univ, 8>;

// Priority queue of the elements of a spatial search, ordered by distance.  Its built-in storage likewise avoids heap
// allocation in most searches.
using SpatialPriorityQueue = PriorityQueue<Univ, 20>;

// Spatial data structure for efficient queries like "closest_elements" or "find_elements_intersecting_ray".
class Spatial : noncopyable {  // An abstract class.
 public:
  static constexpr int k_max_gn = 1023;  // 10 bits per coordinate.
  explicit Spatial(int gn) : _gn(gn), _gni(1.f / assertx(gn)) { assertx(_gn > 0 && _gn <= k_max_gn); }
  virtual ~Spatial() = default;
  virtual void clear() = 0;

 protected:
  friend details::BaseSpatialSearch;
  const int _gn;     // The grid size.
  const float _gni;  // 1.f / _gn

  using Ind = Vec3<int>;
  [[nodiscard]] bool inbounds(int i) const { return i >= 0 && i < _gn; }
  [[nodiscard]] bool indices_inbounds(const Ind& ci) const {
    return inbounds(ci[0]) && inbounds(ci[1]) && inbounds(ci[2]);
  }
  [[nodiscard]] int index_from_float(float fd) const;
  [[nodiscard]] float float_from_index(int i) const { return i * _gni; }
  [[nodiscard]] Ind indices_from_point(const Point& p) const {
    Ind ci;
    for_int(c, 3) ci[c] = index_from_float(p[c]);
    return ci;
  }
  [[nodiscard]] Point point_from_indices(const Ind& ci) const {
    Point p;
    for_int(c, 3) p[c] = float_from_index(ci[c]);
    return p;
  }
  [[nodiscard]] Bbox<float, 3> bbox_of_indices(const Ind& ci) const;
  [[nodiscard]] int encode(const Ind& ci) const { return (ci[0] << 20) | (ci[1] << 10) | ci[2]; }  // k_max_gn implied.
  [[nodiscard]] Ind decode(int en) const;

  // For BaseSpatialSearch:
  // Add elements from cell ci to priority queue with priority equal to distance from pcenter squared.
  // May use set to avoid duplication.
  virtual void add_cell(const Ind& ci, SpatialPriorityQueue& pq, const Point& pcenter,
                        SpatialVisitedSet& set) const = 0;

  // Refine the distance estimate of the first entry in pq (optional).
  virtual void pq_refine(SpatialPriorityQueue& pq, const Point& pcenter) const { dummy_use(pq, pcenter); }

  virtual Univ pq_id(Univ pqe) const = 0;  // Given a pq entry, return the id.
};

namespace details {

class BasePointSpatial : public Spatial {
 public:
  explicit BasePointSpatial(int gn) : Spatial(gn) {}
  ~BasePointSpatial() override { BasePointSpatial::clear(); }
  void clear() override;
  // Die unless id != 0.
  void enter(Univ id, const Point* pp);   // Note: pp is not copied; no ownership is taken.
  void remove(Univ id, const Point* pp);  // Must exist, else die.
  void shrink_to_fit();                   // Often just fragments memory.

 private:
  void add_cell(const Ind& ci, SpatialPriorityQueue& pq, const Point& pcenter, SpatialVisitedSet& set) const override;
  [[nodiscard]] Univ pq_id(Univ pqe) const override;
  struct Node {
    Univ id;
    const Point* p;
  };
  Map<int, Array<Node>> _map;  // Encoded cube index -> Array.
};

}  // namespace details

// Spatial data structure for point elements.
template <typename T> class PointSpatial : public details::BasePointSpatial {
 public:
  explicit PointSpatial(int gn) : BasePointSpatial(gn) {}
  void enter(T id, const Point* pp) { BasePointSpatial::enter(Conv<T>::e(id), pp); }
  void remove(T id, const Point* pp) { BasePointSpatial::remove(Conv<T>::e(id), pp); }
};

// Spatial data structure for point elements indexed by an integer.
class IPointSpatial : public Spatial {
 public:
  explicit IPointSpatial(int gridn, CArrayView<Point> arp);
  ~IPointSpatial() override { clear(); }
  void clear() override;

 private:
  void add_cell(const Ind& ci, SpatialPriorityQueue& pq, const Point& pcenter, SpatialVisitedSet& set) const override;
  [[nodiscard]] Univ pq_id(Univ pqe) const override;

  const Point* _pp;
  Map<int, Array<int>> _map;  // Encoded cube index -> Array of point indices.
};

// Spatial data structure for more general objects.
template <typename Approx2 = float(const Point& p, Univ id), typename Exact2 = float(const Point& p, Univ id)>
class ObjectSpatial : public Spatial {
 public:
  explicit ObjectSpatial(int gn) : Spatial(gn) {}
  ~ObjectSpatial() override { ObjectSpatial::clear(); }
  void clear() override {
    for (auto& cell : _map.values()) HH_SSTAT(Sospcelln, cell.num());
  }
  // Die unless id != 0.
  // Enter an object that comes with a containment function: the function returns true if the object lies
  // within a given bounding box.  A starting point is also given.
  template <typename Func = bool(const Bbox<float, 3>&)> void enter(Univ id, const Point& startp, Func fcontains);

  // Find the objects that could possibly intersect the segment (p1, p2), calling ftest(id) on each one once.
  // The function ftest returns the parametric position f of the object's intersection along the segment, i.e. the
  // point p1 + f * (p2 - p1) with 0 <= f <= 1, or BIGFLOAT if the object does not intersect the segment.
  // The objects are not tested in the exact order of intersection!
  // However, the procedure stops only after calling ftest with all the objects that could intersect the segment at
  // a position smaller than the minimum f returned so far.
  template <typename Func = float(Univ)> void search_segment(const Point& p1, const Point& p2, Func ftest) const;

 private:
  Map<int, Array<Univ>> _map;  // Encoded cube index -> vector.

  void add_cell(const Ind& ci, SpatialPriorityQueue& pq, const Point& pcenter, SpatialVisitedSet& set) const override;
  void pq_refine(SpatialPriorityQueue& pq, const Point& pcenter) const override;
  [[nodiscard]] Univ pq_id(Univ pqe) const override { return pqe; }
};

namespace details {

class BaseSpatialSearch : noncopyable {
 public:
  // The pmaxdis is only a request; you may get objects that lie farther.
  explicit BaseSpatialSearch(const Spatial* pspatial, const Point& p, float maxdis = 10.f);
  ~BaseSpatialSearch();
  struct Result {
    Univ id;
    float d2;  // Squared distance.
  };

  // Also the customization point for ranges::empty(), whose begin() == end() fallback requires a forward range.
  [[nodiscard]] bool empty() const noexcept { return _done; }

 protected:
  void advance();  // Set _result to the next closest element, or set _done.
  [[nodiscard]] const Result& current() const { return _result; }

 private:
  friend Spatial;
  Result _result{};
  bool _done{false};
  using Ind = Vec3<int>;
  const Spatial& _spatial;
  const Point _pcenter;
  float _maxdis;
  SpatialPriorityQueue _pq;    // The pq of entries by distance.
  Vec2<Ind> _ssi;              // Search space indices (extents).
  float _disbv2{0.f};          // Distance to the search space boundary.
  int _axis{-1};               // Axis to expand next.
  int _dir{-1};                // Direction in which to expand next (0, 1).
  SpatialVisitedSet _setevis;  // May be used by add_cell().
  int _ncellsv{0};
  int _nelemsv{0};

  void get_closest_next_cell();
  void expand_search_space();
  void consider(const Ind& ci);
};

}  // namespace details

// Search for nearest element(s) from a given query point.
template <typename T> class SpatialSearch : public details::BaseSpatialSearch {
 public:
  SpatialSearch(const Spatial* pspatial, const Point& pp, float pmaxdis = 10.f)
      : BaseSpatialSearch(pspatial, pp, pmaxdis) {}
  struct Result {
    T id;
    float d2;  // Squared distance.
  };
  // Single-pass iteration in order of increasing distance: "for (const auto [id, d2] : ss) ...".
  // The bindings are by value because operator*() returns a prvalue Result (the Univ id is converted).
  using Iterator = CursorIterator<SpatialSearch>;
  [[nodiscard]] Iterator begin() noexcept { return Iterator(*this); }
  [[nodiscard]] std::default_sentinel_t end() const noexcept { return {}; }

 private:
  friend CursorIterator<SpatialSearch>;
  [[nodiscard]] Result current() const {
    const auto& [id, d2] = BaseSpatialSearch::current();
    return {Conv<T>::d(id), d2};
  }
};

//----------------------------------------------------------------------------

inline int Spatial::index_from_float(float fd) const {
  float f = fd;
  if (f < 0.f) {
    ASSERTX(f > -.01f);
    f = 0.f;
  }
  if (f >= .99999f) {
    ASSERTX(f < 1.01f);
    f = .99999f;
  }
  return int(f * _gn);
}

inline Bbox<float, 3> Spatial::bbox_of_indices(const Ind& ci) const {
  const Point bb0 = point_from_indices(ci);
  const float eps = 1e-7f;
  return Bbox{bb0 - eps, bb0 + thrice(_gni + eps)};
}

inline Spatial::Ind Spatial::decode(int en) const {
  Ind ci;
  // Note: k_max_gn implied here.
  ci[2] = en & ((1 << 10) - 1);
  en = en >> 10;
  ci[1] = en & ((1 << 10) - 1);
  en = en >> 10;
  ci[0] = en;
  return ci;
}

template <typename Approx2, typename Exact2>
void ObjectSpatial<Approx2, Exact2>::add_cell(const Ind& ci, SpatialPriorityQueue& pq, const Point& pcenter,
                                              SpatialVisitedSet& set) const {
  const int en = encode(ci);
  const auto* cell = _map.find_ptr(en);
  if (!cell) return;
  Approx2 approx2;
  for (Univ e : *cell) {
    if (!set.add(e)) continue;
    pq.enter(e, approx2(pcenter, e));
  }
}

template <typename Approx2, typename Exact2>
void ObjectSpatial<Approx2, Exact2>::pq_refine(SpatialPriorityQueue& pq, const Point& pcenter) const {
  Univ id = pq.min();
  const float oldv = pq.min_priority();
  Exact2 exact2;
  const float newv = exact2(pcenter, id);
  if (newv == oldv) return;
  if (newv < oldv - 1e-12f && Warning("newv < oldv")) SHOW(oldv, newv);
  assertx(pq.remove_min() == id);
  pq.enter(id, newv);
}

template <typename Approx2, typename Exact2>
template <typename Func>
void ObjectSpatial<Approx2, Exact2>::enter(Univ id, const Point& startp, Func fcontains) {
  Set<int> set;
  Queue<int> queue;
  int ncubes = 0;
  Ind ci = indices_from_point(startp);
  assertx(indices_inbounds(ci));
  const int enf = encode(ci);
  set.enter(enf);
  queue.enqueue(enf);
  while (!queue.empty()) {
    const int en = queue.dequeue();
    ci = decode(en);
    const Bbox bbox = bbox_of_indices(ci);
    const bool in_cell = fcontains(bbox);
    if (en == enf) assertx(in_cell);
    if (!in_cell) continue;
    _map[en].push(id);
    ncubes++;
    Vec2<Ind> bi;
    for_int(c, 3) {
      bi[0][c] = max(ci[c] - 1, 0);
      bi[1][c] = min(ci[c] + 1, _gn - 1);
    }
    for (const Ind& cit : range(bi[0], bi[1] + 1)) {
      const int enc = encode(cit);
      if (set.add(enc)) queue.enqueue(enc);
    }
  }
  HH_SSTAT(Sospobcells, ncubes);
}

template <typename Approx2, typename Exact2>
template <typename Func>
void ObjectSpatial<Approx2, Exact2>::search_segment(const Point& p1, const Point& p2, Func ftest) const {
  static_assert(std::is_same_v<std::invoke_result_t<Func&, Univ>, float>);
  Set<Univ> set;
  float fmin = BIGFLOAT;  // The minimum parametric position of an intersection found so far.
  for_int(c, 3) {
    assertx(p1[c] >= 0.f && p1[c] <= 1.f);
    assertx(p2[c] >= 0.f && p2[c] <= 1.f);
  }
  const float maxe = max_abs_element(p2 - p1);
  const int ni = index_from_float(maxe) + 2;  // Add 2 there just to be safe.
  Ind pci = indices_from_point(p1);
  int pen = -1;
  for (int i = 0;; i++) {
    // Compute each sample directly (exactly p2 when i == ni), as accumulating steps would drift from p2.
    const Point p = interp(p1, p2, float(ni - i) / float(ni));
    Ind cci = indices_from_point(p);
    ASSERTX(indices_inbounds(cci));
    Vec2<Ind> bi;
    for_int(c, 3) {
      bi[0][c] = min(cci[c], pci[c]);
      bi[1][c] = max(cci[c], pci[c]);
    }
    for (const Ind& cit : range(bi[0], bi[1] + 1)) {
      const int en = encode(cit);
      if (en == pen) continue;
      const auto* cell = _map.find_ptr(en);
      if (!cell) continue;
      for (Univ e : *cell)
        if (set.add(e)) fmin = min(fmin, ftest(e));
    }
    if (i == ni) break;
    if (fmin != BIGFLOAT) {
      // The cells visited so far contain the segment up to its exit from the cell cci (which contains p), so any
      // object not yet tested can only intersect the segment beyond that parametric position.
      float fexit = BIGFLOAT;
      for_int(c, 3) {
        const float d = p2[c] - p1[c];
        if (d) fexit = min(fexit, (float_from_index(cci[c] + (d > 0.f ? 1 : 0)) - p1[c]) / d);
      }
      if (fmin <= fexit) break;
    }
    pci = cci;
    pen = encode(pci);
  }
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SPATIAL_H_
