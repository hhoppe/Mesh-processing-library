// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "G3dOGL/ScGeomorph.h"

#include "libHh/GMesh.h"
using namespace hh;

// Determine whether triangle defined by verts is degenerate.
int ScGeomorph::degenerate(Simplex v[3]) {
  int i, j;
  int cnt = 0;
  for (i = 0; i < 2; i++) {
    for (j = i + 1; j < 3; j++) {
      if (vold[v[i]->getId()] == vold[v[j]->getId()]) cnt++;
    }
  }
  assertx(cnt != 2);
  return cnt;
}

// Determine vertex normal by averaging normals of adjacent faces belonging to the same normal group.
void ScGeomorph::vertSmoothNormal(Simplex vs, Simplex corner_fct, Vector& avg_norm, bool skip_degenerate) {
  Simplex verts[3];

  int ngroup = s_norgroup[corner_fct->getVAttribute()];
  corner_fct->vertices(verts);

  int i_vs = index(verts, vs);

  avg_norm = fct_pnor[corner_fct->getId()];

  if (skip_degenerate && degenerate(verts) == 3) {
    skip_degenerate = false;
  }

  // go around in one direction averaging normals
  // of adjacent facets with same normal group
  // the orientation is given by ordering of va and vb

  Simplex va = verts[i_vs];
  Simplex vb = verts[mod3(i_vs + 1)];

  if (skip_degenerate) {
    if (vold[va->getId()] == vold[vb->getId()]) {
      assertx(degenerate(verts));
      i_vs = mod3(i_vs + 1);
      vb = verts[mod3(i_vs + 1)];
      va = verts[i_vs];
    }
  }

  Simplex e = va->edgeTo(vb);
  assertx(e);
  Simplex fct = corner_fct;
  bool done = false;
  while (e->isManifold()) {
    // find other facet around e
    ForScSimplexParent(e, f) {
      if (f != fct) {
        fct = f;
        break;
      }
    }
    EndFor;

    // if the new fct is the corner_fct made a full circle
    // nothing left to do
    if (fct == corner_fct) {
      done = true;
      break;
    }

    // Simplex verts[3];
    fct->vertices(verts);

    // skip degenerate facets
    if (skip_degenerate && degenerate(verts)) {
      if (vold[va->getId()] == vold[vb->getId()]) {
        Simplex tmp = vb;
        for_int(i, 3) {
          if (verts[i] != va && verts[i] != vb) {
            vb = verts[i];
            break;
          }
        }
        va = tmp;
      } else {
        // if va is degenerate one change it
        bool change = false;
        for_int(i, 3) {
          if (verts[i] == va) continue;

          if (vold[va->getId()] == vold[verts[i]->getId()]) {
            va = verts[i];
            change = true;
            break;
          }
        }

        // if vb is degenerate one change it
        for_int(i, 3) {
          if (verts[i] == vb) continue;

          if (vold[vb->getId()] == vold[verts[i]->getId()]) {
            vb = verts[i];
            assertx(!change);
            break;
          }
        }
      }
    } else {  // regular facet
      // if new facet does not have same smoothing group
      if (s_norgroup[fct->getVAttribute()] != ngroup) break;

      const int i_va = index(verts, va);
      if (verts[mod3(i_va + 1)] == vb) {
        // va still before vb
        // inconsistent with previous fct
        // flip normal
        avg_norm -= fct_pnor[fct->getId()];
        vb = verts[mod3(i_va + 2)];
      } else {
        // va after vb in the order
        // consistent with previous fct
        avg_norm += fct_pnor[fct->getId()];
        vb = verts[mod3(i_va + 1)];
      }
    }

    e = va->edgeTo(vb);
    assertx(e);
  }

  if (!done) {
    // go around in other direction averaging normals
    // of adjacent facets with same normal group
    // the orientation is given by ordering of va and vb
    va = verts[mod3(i_vs + 2)];
    vb = verts[i_vs];

    if (skip_degenerate) {
      if (vold[va->getId()] == vold[vb->getId()]) {
        i_vs = mod3(i_vs + 2);
        va = verts[mod3(i_vs + 2)];
        vb = verts[i_vs];
      }
    }

    e = va->edgeTo(vb);
    assertx(e);
    fct = corner_fct;

    while (e->isManifold()) {
      // find other facet around e
      ForScSimplexParent(e, f) {
        if (f != fct) {
          fct = f;
          break;
        }
      }
      EndFor;

      // cannot make a full circle this way
      assertx(fct != corner_fct);

      // Simplex verts[3];
      fct->vertices(verts);

      // skip degenerate facets
      if (skip_degenerate && degenerate(verts)) {
        if (vold[va->getId()] == vold[vb->getId()]) {
          Simplex tmp = va;
          for_int(i, 3) {
            if (verts[i] != va && verts[i] != vb) {
              va = verts[i];
              break;
            }
          }
          vb = tmp;
        } else {
          // if va is degenerate one change it
          bool change = false;
          for_int(i, 3) {
            if (verts[i] == va) continue;

            if (vold[va->getId()] == vold[verts[i]->getId()]) {
              va = verts[i];
              change = true;
              break;
            }
          }

          // if vb is degenerate one change it
          for_int(i, 3) {
            if (verts[i] == vb) continue;

            if (vold[vb->getId()] == vold[verts[i]->getId()]) {
              vb = verts[i];
              assertx(!change);
              break;
            }
          }
        }
      } else {
        // if new facet does not have same smoothing group
        if (s_norgroup[fct->getVAttribute()] != ngroup) break;

        const int i_va = index(verts, va);
        if (verts[mod3(i_va + 1)] == vb) {
          // va still before vb
          // inconsistent with previous fct
          // flip normal
          avg_norm -= fct_pnor[fct->getId()];
          va = verts[mod3(i_va + 2)];
        } else {
          // va after vb in the order
          // consistent with previous fct
          avg_norm += fct_pnor[fct->getId()];
          va = verts[mod3(i_va + 1)];
        }
      }

      e = va->edgeTo(vb);
      assertx(e);
    }
  }

  avg_norm.normalize();
}

void ScGeomorph::read(std::istream& is) {
  K.read(is);

  vold.init(K.getMaxId(0));
  vnew.init(K.getMaxId(0));
  fct_pnor.init(K.getMaxId(2));
  // 3 verts per each face
  // will use face and vert id for indexing
  nold.init(K.getMaxId(2) * 3);
  nnew.init(K.getMaxId(2) * 3);
  s_norgroup.init(K.materialNum());

  string str;
  for_int(attrid, K.materialNum()) {
    const char* s = K.getMaterial(attrid);
    s_norgroup[attrid] = to_int(assertx(GMesh::string_key(str, s, "norgroup")));
  }

  // verts
  ForScSimplex(K, 0, v) {
    // new pos
    vnew[v->getId()] = v->getPosition();

    // old pos
    Point& opos = vold[v->getId()];
    assertx(parse_key_vec(v->get_string(), "Opos", opos));

    // old area
    const char* soa = GMesh::string_key(str, v->get_string(), "Oarea");
    if (soa) {
      float area;
      assertx(sscanf(soa, "%g", &area) == 1);
      aold.enter(v, area);

      // old material
      int attrid = to_int(assertx(GMesh::string_key(str, v->get_string(), "Omat")));
      mold.enter(v, attrid);
    }

    // new area
    if (v->isPrincipal()) {
      if (0) {
        const char* sna = assertx(GMesh::string_key(str, v->get_string(), "Narea"));
        float area;
        assertx(sscanf(sna, "%g", &area) == 1);
        anew.enter(v, area);
      } else {
        anew.enter(v, v->getArea());
      }
    }
  }
  EndFor;

  // edges
  ForScSimplex(K, 1, e) {
    // old area
    const char* soa = GMesh::string_key(str, e->get_string(), "Oarea");
    if (soa) {
      float area;
      assertx(sscanf(soa, "%g", &area) == 1);
      aold.enter(e, area);

      // old material
      int attrid = to_int(assertx(GMesh::string_key(str, e->get_string(), "Omat")));
      mold.enter(e, attrid);
    }
    // new area
    if (e->isPrincipal()) {
      if (0) {
        const char* sna = assertx(GMesh::string_key(str, e->get_string(), "Narea"));
        float area;
        assertx(sscanf(sna, "%g", &area) == 1);
        anew.enter(e, area);
      } else {
        anew.enter(e, e->getArea());
      }
    }
  }
  EndFor;

  // facets for corner normals
  ForScSimplex(K, 2, f) {
    Simplex v[3];
    f->vertices(v);

    for_int(i, 3) {
      Vector& on = nold[3 * f->getId() + i];
      assertx(parse_key_vec(f->get_string(), csform(str, "Onorm%d", v[i]->getId()), on));
    }
  }
  EndFor;

  // corner normals
  // new normals
  ForScSimplex(K, 2, f) {
    Simplex v[3];
    f->vertices(v);
    fct_pnor[f->getId()] = ok_normalized(cross(v[0]->getPosition(), v[1]->getPosition(), v[2]->getPosition()));
  }
  EndFor;

  ForScSimplex(K, 2, f) {
    Simplex verts[3];
    f->vertices(verts);
    for_int(i, 3) vertSmoothNormal(verts[i], f, nnew[3 * f->getId() + i]);
  }
  EndFor;
}

// Interpolate between old and new using alpha parameter.
void ScGeomorph::update(float alpha, ArrayView<Vector> corner_nors) {  // alpha == 1.f is new;  alpha == 0.f is old
  // verts
  ForScSimplex(K, 0, v) { v->setPosition(interp(vnew[v->getId()], vold[v->getId()], alpha)); }
  EndFor;

  // area
  for (auto& [s, narea] : anew) {
    assertx(s->isPrincipal());
    float oarea = 0.f;
    if (aold.contains(s)) oarea = aold.get(s);
    s->setArea(alpha * narea + (1.f - alpha) * oarea);
  }

  for (auto& [s, oarea] : aold) {
    // handled by anew
    if (!s->isPrincipal()) {
      s->setArea((1.f - alpha) * oarea);
      s->setVAttribute(mold.get(s));
    }
  }

  // normals
  assertx(corner_nors.num() >= nnew.num());
  for_int(i, nnew.num()) {
    const Vector& on = nold[i];
    // if (on[0] == -2.f && on[1] == -2.f && on[2] == -2.f) {
    //     corner_nors[i] = nnew[i];
    // } else {
    //     corner_nors[i] = on;
    // }

    if (on[0] == -2.f && on[1] == -2.f && on[2] == -2.f) {
      corner_nors[i] = nnew[i];
    } else {
      corner_nors[i] = ok_normalized(alpha * nnew[i] + (1.f - alpha) * nold[i]);
    }
  }
}
