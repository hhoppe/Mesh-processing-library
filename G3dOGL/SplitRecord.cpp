// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "G3dOGL/SplitRecord.h"

namespace hh {

void SplitRecord::write(std::ostream& os) const {
  os << _vsid << " " << _vtid << "\n";
  for_int(i, _outcome.num() - 1) os << _outcome[i];
  os << "\n";
  os << _pos_bit << " " << _deltap[0] << " " << _deltap[1] << " " << _deltap[2] << "\n";
  for_int(i, _material.num()) {
    os << _material[i].dim << " " << _material[i].id << " " << _material[i].matid << "\n";
  }
  for_int(i, _area.num()) os << _area[i].dim << " " << _area[i].id << " " << _area[i].area << "\n";
}

bool SplitRecord::read(std::istream& is) {
  // get vs and vt
  if (!(is >> _vsid)) return true;
  assertx(is >> _vtid);

  assertx(_vsid > 0 && _vtid > 0);
  assertx(is.get() == '\n');

  // get the code
  int c;
  while ((c = is.get()) != '\n') {
    assertx(c >= 0);
    _outcome.push(c - '0');
  }
  _outcome.push(-1);

  // get the position bit
  assertx(is >> _pos_bit);
  assertx(is >> _deltap[0] >> _deltap[1] >> _deltap[2]);

  // get the material ids
  int dim, id, matid;
  assertx(is >> dim);
  while (dim != -1) {
    assertx(is >> id >> matid);
    int index = _material.add(1);
    _material[index].dim = dim;
    _material[index].id = id;
    _material[index].matid = matid;

    assertx(is >> dim);
  }

  // get the partition area
  float area;
  assertx(is >> dim);
  while (dim != -1) {
    assertx(is >> id >> area);
    int index = _area.add(1);
    _area[index].dim = dim;
    _area[index].id = id;
    _area[index].area = area;

    assertx(is >> dim);
  }

  return false;
}

void SplitRecord::applySplit(SimplicialComplex& K) {
  Simplex vs = assertx(K.getSimplex(0, _vsid));
  Simplex vt = nullptr;

  Pqueue<Simplex> pq[MAX_DIM + 1];
  ForScSimplexStar(vs, spx) { pq[spx->getDim()].enter_unsorted(spx, float(spx->getId())); }
  EndFor;

  if (0) {
    SimplicialComplex sb;
    K.starbar(vs, sb);
    sb.write(std::cout);
    std::cout.flush();
  }

  // 0-simps
  int dim = 0;
  pq[dim].sort();
  while (!pq[dim].empty()) {
    assertx(pq[dim].remove_min() == vs);
    int outcome = getNextOutcome();

    // debug
    assertx(outcome == 9);
    outcome = getNextOutcome();

    vt = K.createSimplex(0);
    const Point& vsp = vs->getPosition();
    const Point& dp = _deltap;
    // always set vt's position
    vt->setPosition(Point(dp[0] + vsp[0], dp[1] + vsp[1], dp[2] + vsp[2]));

    // if vs is at a former midpoint need to update vs as well
    if (_pos_bit == 0)  // Midpoint.
      vs->setPosition(Point(vsp[0] - dp[0], vsp[1] - dp[1], vsp[2] - dp[2]));

    if (outcome == SplitRecord::V_NOEDGE && vs->isPrincipal()) {
    }
    if (outcome == SplitRecord::V_EDGE) {  // create edge between vs and vt

      Simplex e = K.createSimplex(1);
      e->setChild(0, vs);
      e->setChild(1, vt);
      vs->addParent(e);
      vt->addParent(e);
    }
  }

  assertx(vt);

  // 1-simps
  dim = 1;
  pq[dim].sort();
  while (!pq[dim].empty()) {
    Simplex e = pq[dim].remove_min();
    int outcome = getNextOutcome();

    // debug
    assertx(outcome == 8);
    outcome = getNextOutcome();

    if (outcome == SplitRecord::E_VT) {  // map e from vs to vt
      vs->removeParent(e);
      vt->addParent(e);
      if (e->getChild(0) == vs) {
        e->setChild(0, vt);
      } else if (e->getChild(1) == vs) {
        e->setChild(1, vt);
      } else {
        assertnever("");
      }
    } else if (outcome == SplitRecord::E_VSVT || outcome == SplitRecord::E_F) {  // create simplex to vt
      Simplex splite = K.createSimplex(1);
      Simplex oppv = e->opp_vertex(vs);
      splite->setChild(0, oppv);
      splite->setChild(1, vt);
      oppv->addParent(splite);
      vt->addParent(splite);

      if (outcome == SplitRecord::E_F) {
        Simplex f = K.createSimplex(2);
        f->setChild(0, e);
        e->addParent(f);
        f->setChild(1, splite);
        splite->addParent(f);
        Simplex vsvt_edge = vs->edgeTo(vt);
        f->setChild(2, vsvt_edge);
        vsvt_edge->addParent(f);
      }
    }
  }

  // 2-simps
  dim = 2;
  pq[dim].sort();
  while (!pq[dim].empty()) {
    Simplex f = pq[dim].remove_min();
    int outcome = getNextOutcome();
    // debug
    assertx(outcome == 7);
    outcome = getNextOutcome();
    if (outcome == SplitRecord::F_VT) {  // map from vs to vt
      ForScSimplexChildIndex(f, e, ei) {
        // vertex opposite to vs
        Simplex voppvs = e->opp_vertex(vs);
        // if such vertex exists, ie if e is not opp_edge(vs)
        if (voppvs) {
          // map e to edge adjacent to vt
          e->removeParent(f);
          Simplex newe = assertx(voppvs->edgeTo(vt));
          f->setChild(ei, newe);
          newe->addParent(f);
        }
      }
      EndFor;
    } else if (outcome == SplitRecord::F_VSVT) {
      Simplex splitf = K.createSimplex(2);
      Simplex verts[3];
      f->vertices(verts);

      for_int(i, 3) {
        Simplex v1 = verts[i] == vs ? vt : verts[i];
        Simplex v2 = verts[mod3(i + 1)] == vs ? vt : verts[mod3(i + 1)];
        Simplex e = assertx(v1->edgeTo(v2));

        splitf->setChild(i, e);
        e->addParent(splitf);
      }
    }
  }

  for_int(i, _material.num()) {
    Simplex s = K.getSimplex(_material[i].dim, _material[i].id);
    assertx(s && s->isPrincipal());
    s->setVAttribute(_material[i].matid);
  }

  for_int(i, _area.num()) {
    Simplex s = K.getSimplex(_area[i].dim, _area[i].id);
    assertx(s && s->isPrincipal());
    s->setArea(_area[i].area);
  }

  reset();
}

void SplitRecord::applyGMSplit(SimplicialComplex& K) {
  Simplex vs = assertx(K.getSimplex(0, _vsid));
  Simplex vt = nullptr;

  Pqueue<Simplex> pq[MAX_DIM + 1];
  ForScSimplexStar(vs, spx) { pq[spx->getDim()].enter_unsorted(spx, float(spx->getId())); }
  EndFor;

  if (0) {
    SimplicialComplex sb;
    K.starbar(vs, sb);
    sb.write(std::cout);
    std::cout.flush();
  }

  new_facets.clear();

  // 0-simps
  int dim = 0;
  pq[dim].sort();
  while (!pq[dim].empty()) {
    assertx(pq[dim].remove_min() == vs);
    int outcome = getNextOutcome();

    // debug
    assertx(outcome == 9);
    outcome = getNextOutcome();

    vt = K.createSimplex(0);
    const Point& vsp = vs->getPosition();
    const Point& dp = _deltap;
    // always set vt's position
    vt->setPosition(Point(dp[0] + vsp[0], dp[1] + vsp[1], dp[2] + vsp[2]));

    // if vs is at a former midpoint need to update vs as well
    if (_pos_bit == 0)  // Midpoint.
      vs->setPosition(Point(vsp[0] - dp[0], vsp[1] - dp[1], vsp[2] - dp[2]));

    if (outcome == SplitRecord::V_NOEDGE && vs->isPrincipal()) {
    }
    if (outcome == SplitRecord::V_EDGE) {  // create edge between vs and vt
      Simplex e = K.createSimplex(1);
      e->setChild(0, vs);
      e->setChild(1, vt);
      vs->addParent(e);
      vt->addParent(e);
    }
  }

  assertx(vt);

  // 1-simps
  dim = 1;
  pq[dim].sort();
  while (!pq[dim].empty()) {
    Simplex e = pq[dim].remove_min();
    int outcome = getNextOutcome();

    // debug
    assertx(outcome == 8);
    outcome = getNextOutcome();

    if (outcome == SplitRecord::E_VT) {  // map e from vs to vt
      vs->removeParent(e);
      vt->addParent(e);
      if (e->getChild(0) == vs) {
        e->setChild(0, vt);
      } else if (e->getChild(1) == vs) {
        e->setChild(1, vt);
      } else {
        assertnever("");
      }
    } else if (outcome == SplitRecord::E_VSVT || outcome == SplitRecord::E_F) {  // create simplex to vt
      Simplex splite = K.createSimplex(1);
      Simplex oppv = e->opp_vertex(vs);
      splite->setChild(0, oppv);
      splite->setChild(1, vt);
      oppv->addParent(splite);
      vt->addParent(splite);

      if (outcome == SplitRecord::E_F) {
        Simplex f = K.createSimplex(2);
        new_facets.push_back(f);
        f->setChild(0, e);
        e->addParent(f);
        f->setChild(1, splite);
        splite->addParent(f);
        Simplex vsvt_edge = vs->edgeTo(vt);
        f->setChild(2, vsvt_edge);
        vsvt_edge->addParent(f);
      }
    }
  }

  // 2-simps
  dim = 2;
  pq[dim].sort();
  while (!pq[dim].empty()) {
    Simplex f = pq[dim].remove_min();
    int outcome = getNextOutcome();
    // debug
    assertx(outcome == 7);
    outcome = getNextOutcome();
    if (outcome == SplitRecord::F_VT) {  // map from vs to vt
      ForScSimplexChildIndex(f, e, ei) {
        // vertex opposite to vs
        Simplex voppvs = e->opp_vertex(vs);
        // if such vertex exists, ie if e is not opp_edge(vs)
        if (voppvs) {
          // map e to edge adjacent to vt
          e->removeParent(f);
          Simplex newe = assertx(voppvs->edgeTo(vt));
          f->setChild(ei, newe);
          newe->addParent(f);
        }
      }
      EndFor;
    } else if (outcome == SplitRecord::F_VSVT) {
      Simplex splitf = K.createSimplex(2);
      new_facets.push_back(splitf);
      Simplex verts[3];
      f->vertices(verts);

      for_int(i, 3) {
        Simplex v1 = verts[i] == vs ? vt : verts[i];
        Simplex v2 = verts[mod3(i + 1)] == vs ? vt : verts[mod3(i + 1)];
        Simplex e = assertx(v1->edgeTo(v2));

        splitf->setChild(i, e);
        e->addParent(splitf);
      }
    }
  }

  for_int(i, _material.num()) {
    Simplex s = K.getSimplex(_material[i].dim, _material[i].id);
    assertx(s && s->isPrincipal());
    s->setVAttribute(_material[i].matid);
  }

  for_int(i, _area.num()) {
    Simplex s = K.getSimplex(_area[i].dim, _area[i].id);
    assertx(s && s->isPrincipal());
    s->setArea(_area[i].area);
  }

  reset();
}

void SplitRecord::applyCmpSplit(SimplicialComplex& K) {
  Simplex vs = assertx(K.getSimplex(0, _vsid));
  Simplex vt = nullptr;

  Pqueue<Simplex> pq[MAX_DIM + 1];
  ForScSimplexStar(vs, spx) { pq[spx->getDim()].enter_unsorted(spx, float(spx->getId())); }
  EndFor;

  if (0) {
    SimplicialComplex sb;
    K.starbar(vs, sb);
    sb.write(std::cout);
    std::cout.flush();
  }

  new_simplices.clear();

  // 0-simps
  int dim = 0;
  pq[dim].sort();
  while (!pq[dim].empty()) {
    assertx(pq[dim].remove_min() == vs);
    int outcome = getNextOutcome();

    // debug
    assertx(outcome == 9);
    outcome = getNextOutcome();

    vt = K.createSimplex(0);
    new_simplices.push_back(vt);
    const Point& vsp = vs->getPosition();
    const Point& dp = _deltap;
    // always set vt's position
    vt->setPosition(Point(dp[0] + vsp[0], dp[1] + vsp[1], dp[2] + vsp[2]));

    // if vs is at a former midpoint need to update vs as well
    if (_pos_bit == 0)  // Midpoint.
      vs->setPosition(Point(vsp[0] - dp[0], vsp[1] - dp[1], vsp[2] - dp[2]));

    if (outcome == SplitRecord::V_NOEDGE && vs->isPrincipal()) {
    }
    if (outcome == SplitRecord::V_EDGE) {  // create edge between vs and vt

      Simplex e = K.createSimplex(1);
      new_simplices.push_back(e);
      e->setChild(0, vs);
      e->setChild(1, vt);
      vs->addParent(e);
      vt->addParent(e);
    }
  }

  assertx(vt);

  // 1-simps
  dim = 1;
  pq[dim].sort();
  while (!pq[dim].empty()) {
    Simplex e = pq[dim].remove_min();
    int outcome = getNextOutcome();

    // debug
    assertx(outcome == 8);
    outcome = getNextOutcome();

    if (outcome == SplitRecord::E_VT) {  // map e from vs to vt
      vs->removeParent(e);
      vt->addParent(e);
      if (e->getChild(0) == vs) {
        e->setChild(0, vt);
      } else if (e->getChild(1) == vs) {
        e->setChild(1, vt);
      } else {
        assertnever("");
      }
    } else if (outcome == SplitRecord::E_VSVT || outcome == SplitRecord::E_F) {  // create simplex to vt
      Simplex splite = K.createSimplex(1);
      new_simplices.push_back(splite);
      Simplex oppv = e->opp_vertex(vs);
      splite->setChild(0, oppv);
      splite->setChild(1, vt);
      oppv->addParent(splite);
      vt->addParent(splite);

      if (outcome == SplitRecord::E_F) {
        Simplex f = K.createSimplex(2);
        new_simplices.push_back(f);
        f->setChild(0, e);
        e->addParent(f);
        f->setChild(1, splite);
        splite->addParent(f);
        Simplex vsvt_edge = vs->edgeTo(vt);
        f->setChild(2, vsvt_edge);
        vsvt_edge->addParent(f);
      }
    }
  }

  // 2-simps
  dim = 2;
  pq[dim].sort();
  while (!pq[dim].empty()) {
    Simplex f = pq[dim].remove_min();
    int outcome = getNextOutcome();
    // debug
    assertx(outcome == 7);
    outcome = getNextOutcome();
    if (outcome == SplitRecord::F_VT) {  // map from vs to vt
      ForScSimplexChildIndex(f, e, ei) {
        // vertex opposite to vs
        Simplex voppvs = e->opp_vertex(vs);
        // if such vertex exists, ie if e is not opp_edge(vs)
        if (voppvs) {
          // map e to edge adjacent to vt
          e->removeParent(f);
          Simplex newe = assertx(voppvs->edgeTo(vt));
          f->setChild(ei, newe);
          newe->addParent(f);
        }
      }
      EndFor;
    } else if (outcome == SplitRecord::F_VSVT) {
      Simplex splitf = K.createSimplex(2);
      new_simplices.push_back(splitf);
      Simplex verts[3];
      f->vertices(verts);

      for_int(i, 3) {
        Simplex v1 = verts[i] == vs ? vt : verts[i];
        Simplex v2 = verts[mod3(i + 1)] == vs ? vt : verts[mod3(i + 1)];
        Simplex e = assertx(v1->edgeTo(v2));

        splitf->setChild(i, e);
        e->addParent(splitf);
      }
    }
  }

  for_int(i, _material.num()) {
    Simplex s = K.getSimplex(_material[i].dim, _material[i].id);
    assertx(s && s->isPrincipal());
    s->setVAttribute(_material[i].matid);
  }

  for_int(i, _area.num()) {
    Simplex s = K.getSimplex(_area[i].dim, _area[i].id);
    assertx(s && s->isPrincipal());
    s->setArea(_area[i].area);
  }

  reset();
}

void SplitRecord::applyUnify(SimplicialComplex& K) const {
  Simplex vs = assertx(K.getSimplex(0, _vsid));
  Simplex vt = assertx(K.getSimplex(0, _vtid));

  Point new_pos = vs->getPosition();
  if (_pos_bit == 0) new_pos = interp(vs->getPosition(), vt->getPosition());

  K.unify(vs, vt);
  vs->setPosition(new_pos);
}

}  // namespace hh
