// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Spatial.h"

namespace hh {

// Given 10000 random data points uniformly sampled over the unit cube,
// find the closest 10 neighbors:
// spatialtest -gn ? -pn 10000
// average time
//      ?       10000           1000
//      1       1.1745          .14967
//      2       .25267          .05333
//      3       .11967          .03883
//      4       .07450          .03217
//      5       .05583          .02917
//      10      .03266          .02517
//      20      .02733          .02433
//      30      .02500          .02833
//      40      .02500          .03550
//      50      .02550          .04733
//      70      .03067          .08950
//      100     .04383          .20933
//      200     .18033          1.4403
// optimal #cells/point:
//              .1-30           .06-40
//
// for 100'000 data points, gn=40, time is .03100 (still very good)
//                          gn=50, time is .02583

// *** BPointSpatial

void BPointSpatial::clear() {
  for (auto& cell : _map.values()) HH_SSTAT(Spspcelln, cell.num());
  _map.clear();
}

void BPointSpatial::enter(Univ id, const Point* pp) {
  Ind ci = point_to_indices(*pp);
  assertx(indices_inbounds(ci));
  int en = encode(ci);
  _map[en].push(Node{id, pp});  // First create empty Array<Node> if not present.
}

void BPointSpatial::remove(Univ id, const Point* pp) {
  Ind ci = point_to_indices(*pp);
  assertx(indices_inbounds(ci));
  int en = encode(ci);
  Array<Node>& ar = _map.get(en);
  int ind = -1;
  for_int(i, ar.num()) {
    if (ar[i].id == id) {
      assertx(ind < 0);
      ind = i;
    }
  }
  assertx(ind >= 0);
  ar.erase(ind, 1);
  if (!ar.num()) _map.remove(en);
}

void BPointSpatial::shrink_to_fit() {
  for (auto& cell : _map.values()) cell.shrink_to_fit();
}

void BPointSpatial::add_cell(const Ind& ci, Pqueue<Univ>& pq, const Point& pcenter, Set<Univ>& /*set*/) const {
  // SHOW("add_cell", ci);
  int en = encode(ci);
  bool present;
  const auto& cell = _map.retrieve(en, present);
  if (!present) return;
  for (const Node& e : cell) {
    // SHOW("enter", *e.p, dist2(pcenter, *e.p));
    pq.enter(Conv<const Node*>::e(&e), dist2(pcenter, *e.p));
  }
}

Univ BPointSpatial::pq_id(Univ pqe) const {
  const Node* e = Conv<const Node*>::d(pqe);
  return e->id;
}

// *** IPointSpatial

IPointSpatial::IPointSpatial(int gn, CArrayView<Point> arp) : Spatial(gn), _pp(arp.data()) {
  for_int(i, arp.num()) {
    Ind ci = point_to_indices(arp[i]);
    assertx(indices_inbounds(ci));
    int en = encode(ci);
    _map[en].push(i);  //  First create empty Array<int> if not present.
  }
  if (0)
    for (auto& cell : _map.values()) cell.shrink_to_fit();
}

void IPointSpatial::clear() {
  for (auto& cell : _map.values()) HH_SSTAT(Spspcelln, cell.num());
  _map.clear();
}

void IPointSpatial::add_cell(const Ind& ci, Pqueue<Univ>& pq, const Point& pcenter, Set<Univ>& /*set*/) const {
  int en = encode(ci);
  bool present;
  const auto& cell = _map.retrieve(en, present);
  if (!present) return;
  for (int i : cell) pq.enter(Conv<int>::e(i), dist2(pcenter, _pp[i]));
}

Univ IPointSpatial::pq_id(Univ pqe) const { return pqe; }

// *** SpatialSearch

BSpatialSearch::BSpatialSearch(const Spatial* sp, const Point& p, float maxdis)
    : _sp(*assertx(sp)), _pcenter(p), _maxdis(maxdis) {
  // SHOW("search", p, maxdis);
  Ind ci = _sp.point_to_indices(_pcenter);
  assertx(_sp.indices_inbounds(ci));
  for_int(i, 2) for_int(c, 3) _ssi[i][c] = ci[c];
  consider(ci);
  get_closest_next_cell();
}

BSpatialSearch::~BSpatialSearch() {
  HH_SSTAT(Sssncellsv, _ncellsv);
  HH_SSTAT(Sssnelemsv, _nelemsv);
}

bool BSpatialSearch::done() {
  for (;;) {
    if (!_pq.empty()) return false;
    if (_disbv2 >= square(_maxdis)) return true;
    expand_search_space();
  }
}

Univ BSpatialSearch::next(float* pdis2) {
  Univ u;
  for (;;) {
    if (_pq.empty()) assertx(!done());  // Refill _pq.
    float dis2 = _pq.min_priority();
    if (dis2 > _disbv2) {
      expand_search_space();
      continue;
    }
    u = _pq.min();
    _sp.pq_refine(_pq, _pcenter);
    if (_pq.min() != u || _pq.min_priority() != dis2) continue;
    if (pdis2) *pdis2 = _pq.min_priority();
    u = _pq.remove_min();
    break;
  }
  return _sp.pq_id(u);
}

void BSpatialSearch::consider(const Ind& ci) {
  // SHOW("consider", ci);
  _ncellsv++;
  int n = _pq.num();
  _sp.add_cell(ci, _pq, _pcenter, _setevis);
  _nelemsv += _pq.num() - n;
}

void BSpatialSearch::get_closest_next_cell() {
  float mindis = 1e10f;
  for_int(c, 3) {
    if (_ssi[0][c] > 0) {
      float a = _pcenter[c] - _sp.index_to_float(_ssi[0][c]);
      assertx(a >= 0.f);
      if (a < mindis) {
        mindis = a;
        _axis = c;
        _dir = 0;
      }
    }
    if (_ssi[1][c] < _sp._gn - 1) {
      float a = _sp.index_to_float(_ssi[1][c] + 1) - _pcenter[c];
      assertx(a >= 0.f);
      if (a < mindis) {
        mindis = a;
        _axis = c;
        _dir = 1;
      }
    }
  }
  // The value mindis may be large if all of space has been searched.
  _disbv2 = square(mindis);
}

void BSpatialSearch::expand_search_space() {
  ASSERTX(_axis >= 0 && _axis < 3 && _dir >= 0 && _dir <= 1);
  // SHOW("expand", _axis, _dir, _ssi);
  _ssi[_dir][_axis] += _dir ? 1 : -1;
  Vec2<Ind> bi = _ssi;
  bi[0][_axis] = bi[1][_axis] = _ssi[_dir][_axis];
  // Consider the layer whose axis's value is _ssi[_dir][_axis].
  for (const Ind& cit : range(bi[0], bi[1] + 1)) consider(cit);
  get_closest_next_cell();
}

}  // namespace hh
