// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "G3dOGL/SimplicialComplex.h"

#include "libHh/RangeOp.h"  // compare()
#include "libHh/Set.h"
#include "libHh/Stack.h"  // also vec_contains()
#include "libHh/StringOp.h"

namespace hh {

namespace {

constexpr float k_tolerance = 1e-6f;     // scalar attribute equality tolerance
constexpr float k_undefined = BIGFLOAT;  // undefined scalar attributes
HH_STAT(Sarea_dropped);
HH_STAT(Sarea_moved);

}  // namespace

// *** ISimplex

HH_ALLOCATE_POOL(ISimplex);

Array<Simplex> ISimplex::all_faces() const {
  Array<Simplex> faces;
  Queue<Simplex> queue;
  queue.enqueue(const_cast<Simplex>(this));
  while (!queue.empty()) {
    Simplex s = queue.dequeue();
    faces.push(s);
    if (s->getDim() != 0) {
      for (Simplex c : s->children())
        if (!queue.contains(c)) queue.enqueue(c);
    }
  }
  return faces;
}

PArray<Simplex, 20> ISimplex::get_star() const {
  // Buggy: only valid for simplicial complex with DIM <= 3.
  PArray<Simplex, 20> simplices;
  Simplex s = const_cast<Simplex>(this);
  simplices.push(s);
  for (Simplex ss : s->getParents()) simplices.push(ss);
  if (s->getDim() == 0) {
    int index = simplices.num();
    // for each edge
    for_intL(i, 1, index) {
      // add in faces
      for (Simplex f : simplices[i]->getParents()) {
        bool found = false;
        // only, if not already there
        for_intL(j, index, simplices.num()) {
          if (f == simplices[j]) {
            found = true;
            break;
          }
        }
        if (!found) simplices.push(f);
      }
    }
  }
  return simplices;
}

PArray<Simplex, 20> ISimplex::faces_of_vertex() const {
  PArray<Simplex, 20> simplices;
  Simplex s = const_cast<Simplex>(this);
  assertx(s->getDim() == 0);
  for (Simplex e : s->getParents())
    for (Simplex f : e->getParents())
      if (!simplices.contains(f)) simplices.push(f);
  return simplices;
}

void ISimplex::polygon(Polygon& poly) const {
  assertx(_dim == 2);
  Simplex s0[2];
  Simplex s1;
  poly.init(0);
  s0[0] = getChild(0)->getChild(0);
  poly.push(s0[0]->getPosition());
  s0[1] = getChild(0)->getChild(1);
  poly.push(s0[1]->getPosition());
  s1 = getChild(1);
  const int child_index = s1->getChild(0) != s0[0] && s1->getChild(0) != s0[1] ? 0 : 1;
  poly.push(s1->getChild(child_index)->getPosition());
  return;
}

// *** SimplicialComplex

void SimplicialComplex::clear() {
  for_int(i, MAX_DIM + 1) {
    for (Simplex s : this->simplices_dim(i)) delete s;
    _simplices[i].clear();
    _free_sid[i] = 1;
  }
}

// Make *this SC a copy of the orig.
void SimplicialComplex::copy(const SimplicialComplex& orig) {
  clear();

  for_int(i, MAX_DIM + 1) {
    for (Simplex s : orig.simplices_dim(i)) {
      Simplex news = createSimplex(s->getDim(), s->getId());
      for (auto [ci, c] : enumerate<int>(s->children())) {
        Simplex this_child = getSimplex(c->getDim(), c->getId());
        news->setChild(ci, this_child);
        this_child->addParent(news);
      }
      if (s->getDim() == 0) news->setPosition(s->getPosition());

      news->_flags = s->_flags;
      news->_area = s->_area;
      news->setVAttribute(s->getVAttribute());
    }
  }

  _material_strings = orig._material_strings;

  for_int(i, MAX_DIM + 1) assertx(num(i) == orig.num(i));
}

// Check as much as possible if SimplicialComplex is ok.
void SimplicialComplex::ok() const {
  HH_ATIMER("__ok");
  for_int(i, MAX_DIM + 1) {
    for (Simplex si : this->simplices_dim(i)) {
      assertx(si->getDim() <= MAX_DIM && si->getDim() >= 0);
      for (Simplex c : si->children()) {
        if (!valid(c)) std::cerr << "Simplex " << si->getDim() << " " << si->getId() << "has invalid child.\n";

        if (!vec_contains(c->_parent, si))
          std::cerr << "Simplex " << c->getDim() << " " << c->getId() << "does not know for a parent (" << si->getDim()
                    << " " << si->getId() << ") of which it is a child.\n";
      }

      for (Simplex p : si->getParents()) {
        if (!valid(p)) std::cerr << "Simplex " << si->getDim() << " " << si->getId() << "has invalid parent.\n";

        bool found = false;
        for (Simplex pc : p->children())
          if (pc == si) found = true;

        if (!found) {
          std::cerr << "Simplex " << p->getDim() << " " << p->getId() << " does not know for a child (" << si->getDim()
                    << " " << si->getId() << ") of which it is a parent.\n";
        }
      }
      // check for duplicates
      int num_identical = 0;
      for (Simplex p1 : si->getParents()) {
        for (Simplex p2 : si->getParents()) {
          if (p1 == p2) {
            num_identical++;
            continue;
          }
          if (equal(p1, p2))
            std::cerr << "Simplex " << si->getDim() << " " << si->getId() << " has duplicate parents.\n";
        }
      }
      if (num_identical != narrow_cast<int>(si->_parent.size()))
        std::cerr << "Simplex " << si->getDim() << " " << si->getId() << " has duplicate parents.\n";
    }
  }
}

// Return simplicial complex representing starbar of simplex s.
// id's of simplices in resulting SC are also meaningful in *this.
void SimplicialComplex::starbar(Simplex s, SimplicialComplex& res) const {
  assertx(s->getDim() == 0);

  res.clear();
  // commented primarily for unify record reasons
  // res._material_strings = _material_strings;

  // note: it cycles through lower dimension parents first
  for (Simplex curr : s->get_star()) {
    Simplex news;

    // create a copy of current simplex in resulting simplicial complex
    news = assertx(res.createSimplex(curr->getDim(), curr->getId()));
    if (news->getDim() == 0) news->setPosition(s->getPosition());
    news->setVAttribute(curr->getVAttribute());
    news->_flags = curr->_flags;
    news->_area = curr->_area;

    for (auto [ci, c] : enumerate<int>(curr->children())) {
      Simplex res_child = res.getSimplex(c->getDim(), c->getId());
      // note some children might not be ancestors of s
      if (!res_child) {
        // if so, first create them
        res_child = res.createSimplex(c->getDim(), c->getId());
        if (c->getDim() == 0) res_child->setPosition(c->getPosition());
        res_child->setVAttribute(c->getVAttribute());
        res_child->_flags = c->_flags;
        res_child->_area = c->_area;

        // update child pointers (all must exist)
        for (auto [cci, cc] : enumerate<int>(c->children())) {
          Simplex res_childchild = res.getSimplex(cc->getDim(), cc->getId());
          assertx(res_childchild);  // all must exist
          res_child->setChild(cci, res_childchild);
          res_childchild->addParent(res_child);
        }
      }
      news->setChild(ci, res_child);
      res_child->addParent(news);
    }
  }
}

// Perform union of two simplicial complex where id's are meaningful within this SC.
void SimplicialComplex::scUnion(const SimplicialComplex& s1, const SimplicialComplex& s2,
                                SimplicialComplex& res) const {
  res.copy(s1);

  for_int(i, MAX_DIM + 1) {
    for (Simplex s2_s : s2.simplices_dim(i)) {
      Simplex res_news = res.getSimplex(s2_s->getDim(), s2_s->getId());

      // create it if it doesn't exist in res
      if (!res_news) {
        res_news = res.createSimplex(s2_s->getDim(), s2_s->getId());
        if (res_news->getDim() == 0) res_news->setPosition(s2_s->getPosition());
        res_news->setVAttribute(s2_s->getVAttribute());
        res_news->_flags = s2_s->_flags;
        res_news->_area = s2_s->_area;
        // update its links
        for (auto [s2_ci, s2_c] : enumerate<int>(s2_s->children())) {
          Simplex res_child = res.getSimplex(s2_c->getDim(), s2_c->getId());
          assertx(res_child);  // all children must exist

          res_news->setChild(s2_ci, res_child);

          // update p
          res_child->addParent(res_news);
        }
      }
    }
  }
}

// Given a simplex s, return with res containing all simplices adjacent to s.
void SimplicialComplex::star(Simplex s, Array<Simplex>& res) const {
  res.init(0);
  // res.push(s);
  for (Simplex curr : s->get_star()) res.push(curr);
}

// Removes simplex and all of its ancestors from SC.
void SimplicialComplex::destroySimplex(Simplex s, int area_test) {
  assertx(valid(s));
  if (area_test) assertx(s->getArea() == 0.f);

  // remove all references from its children
  for (Simplex c : s->children())
    if (c) vec_remove_ordered(c->_parent, s);

  Stack<Simplex> todel;
  // find all parents to be removed
  for (Simplex p : s->getParents())
    if (p) todel.push(p);

  // destroy parents!
  while (!todel.empty()) {
    Simplex del = todel.pop();
    destroySimplex(del);
  }

  _free_sid[s->getDim()] -= 1;
  _simplices[s->getDim()].remove(s->getId());
  delete s;
}

// Lisp-like equal comparison: two simplices equal if their children are equal.
// Note eq => equal but NOT equal => eq   (more expensive than eq).
bool SimplicialComplex::equal(Simplex s1, Simplex s2) const {
  if (s1->getDim() != s2->getDim()) return false;

  if (s1 == s2 || s1->getId() == s2->getId()) return true;

  int num_equal = 0;
  for (Simplex c1 : s1->children())
    for (Simplex c2 : s2->children())
      if (equal(c1, c2)) num_equal++;

  assertx(num_equal <= s1->getDim() + 1);

  if (num_equal == s1->getDim() + 1) return true;

  return false;
}

// Lisp like eq comparison: two simplices eq if they are identical or their children are identical eq.
// Note: children must be distinct for this to work i.e. no degenerate simplices allowed.
// Note: no duplicate simplices allowed.
bool SimplicialComplex::eq1simp(Simplex s1, Simplex s2) const {
  assertx(s1->getDim() == 1);
  assertx(s2->getDim() == 1);
  Simplex s1v1 = s1->getChild(0);
  Simplex s1v2 = s1->getChild(1);
  Simplex s2v1 = s2->getChild(0);
  Simplex s2v2 = s2->getChild(1);

  return (s1v1 == s2v1 && s1v2 == s2v2) || (s1v1 == s2v2 && s1v2 == s2v1);
}

bool SimplicialComplex::eq2simp(Simplex s1, Simplex s2) const {
  assertx(s1->getDim() == 2);
  assertx(s2->getDim() == 2);

  Simplex s1verts[3];
  Simplex s2verts[3];
  s1->vertices(s1verts);
  s2->vertices(s2verts);

  return ((s1verts[0] == s2verts[0] && ((s1verts[1] == s2verts[1] && s1verts[2] == s2verts[2]) ||
                                        (s1verts[1] == s2verts[2] && s1verts[2] == s2verts[1]))) ||
          (s1verts[0] == s2verts[1] && ((s1verts[1] == s2verts[2] && s1verts[2] == s2verts[0]) ||
                                        (s1verts[1] == s2verts[0] && s1verts[2] == s2verts[2]))) ||
          (s1verts[0] == s2verts[2] && ((s1verts[1] == s2verts[0] && s1verts[2] == s2verts[1]) ||
                                        (s1verts[1] == s2verts[1] && s1verts[2] == s2verts[0]))));
}

void SimplicialComplex::unify(Simplex vs, Simplex vt, int propagate_area) {
  assertx(vs->getDim() == 0 && vt->getDim() == 0);

  Simplex both = vs->edgeTo(vt);
  Stack<Simplex> check_principal;

  // propagate material
  if (both) {
    // new principal edges
    for (Simplex s : both->get_star()) {
      if (s == both) continue;

      assertx(s->getDim() == 2);
      for (Simplex c : s->children())
        if (c->getParents().size() == 1) c->setVAttribute(s->getVAttribute());
    }

    // new principal verts
    if (vs->getParents().size() == 1) vs->setVAttribute(both->getVAttribute());
  }

  float cmp_area = 0.f;

  if (propagate_area) {
    for (Simplex s : vs->get_star()) {
      if (s->isPrincipal()) {
        assertx(s->getArea() > 0.f);
      } else {
        assertx(s->getArea() == 0.f);
      }
    }

    for (Simplex s : vt->get_star()) {
      if (s->isPrincipal()) {
        assertx(s->getArea() > 0.f);
      } else {
        assertx(s->getArea() == 0.f);
      }
    }

    if (both) {
      // distribute area
      for (Simplex spx : both->get_star()) {
        // consider only principal simplices
        if (!spx->isPrincipal()) continue;

        bool area_given = false;
        bool drop_area = false;

        // give it's area to manifold adjacent component
        for (Simplex c : spx->children()) {
          if (c == both) continue;

          if (c->isManifold()) {
            Simplex spx_adj = nullptr;
            // find adjacent component
            for (Simplex p : c->getParents()) {
              if (p != spx) {
                spx_adj = p;
                break;
              }
            }
            assertx(spx_adj && spx_adj->isPrincipal());

            Sarea_moved.enter(spx->getArea());
            Sarea_dropped.enter(spx->getArea());
            spx_adj->setArea(spx_adj->getArea() + spx->getArea());
            spx->setArea(0.f);
            area_given = true;
            break;
          }

          // if all children are boundary the ancestor will be a
          // a principal simplex and we can give its area away
          // to the ancestor
          if (!c->is_boundary()) {
            // otherwise drop the area
            drop_area = true;
            Sarea_dropped.enter(spx->getArea());
            spx->setArea(0.f);
          }
        }

        // if area should not be dropped and is not given away
        if (!drop_area && !area_given) {
          assertx(spx->getDim() == 1 || spx->getDim() == 2);

          Simplex spx_adj = nullptr;
          if (spx->getDim() == 2) {
            // facet
            spx_adj = spx->opp_edge(vt);
          }

          if (spx->getDim() == 1) {
            // vert
            spx_adj = vs;
          }

          assertx(spx_adj);
          check_principal.push(spx_adj);

          Sarea_dropped.enter(spx->getArea());
          Sarea_moved.enter(spx->getArea());
          spx_adj->setArea(spx_adj->getArea() + spx->getArea());
          spx->setArea(0.f);
        }
      }
    }

    if (vt->isPrincipal()) {
      cmp_area = vt->getArea();
      Sarea_dropped.enter(cmp_area);
      vt->setArea(0.f);
    } else if (vs->isPrincipal()) {
      cmp_area = vs->getArea();
      Sarea_dropped.enter(cmp_area);
      vs->setArea(0.f);
    }
  }

  if (both) destroySimplex(both, propagate_area);
  // both = nullptr;  // now undefined

  // remap all references of vt to vs in simplices adjacent to vt.
  std::vector<Simplex> worklist[MAX_DIM + 1];
  for (Simplex s : vs->get_star())
    if (s != vs) worklist[s->getDim()].push_back(s);

  Stack<Simplex> affected_spx[MAX_DIM + 2];
  replace(vt, vs, affected_spx[1]);

  // remove vt
  vt->_parent.clear();
  destroySimplex(vt, propagate_area);
  // vt = nullptr;  // now undefined

  assertx(Sarea_moved.sum() <= Sarea_dropped.sum());

  // remove duplicate simplices
  int dim = 1;
  while (!affected_spx[dim].empty()) {
    Simplex vs_new_ancestor = affected_spx[dim].pop();
    for (Simplex vs_ancestor : worklist[dim]) {  // was ForStack which went in reverse order
      assertx(vs_ancestor->getDim() == vs_new_ancestor->getDim());
      assertx(vs_new_ancestor != vs_ancestor);

      if (eq1simp(vs_new_ancestor, vs_ancestor)) {
        // remove duplicate
        if (propagate_area) {
          if (vs_new_ancestor->isPrincipal() && vs_ancestor->isPrincipal()) {
            Sarea_dropped.enter(vs_new_ancestor->getArea());
            Sarea_moved.enter(vs_new_ancestor->getArea());
            vs_ancestor->setArea(vs_ancestor->getArea() + vs_new_ancestor->getArea());
            vs_new_ancestor->setArea(0.f);
          } else {
            if (vs_new_ancestor->getArea() > 0.f) {
              assertx(vs_new_ancestor->isPrincipal());
              Sarea_dropped.enter(vs_new_ancestor->getArea());
              vs_new_ancestor->setArea(0.f);
            }
            if (vs_ancestor->getArea() > 0.f) {
              assertx(vs_ancestor->isPrincipal());
              Sarea_dropped.enter(vs_new_ancestor->getArea());
              vs_ancestor->setArea(0.f);
            }
          }
        }

        replace(vs_new_ancestor, vs_ancestor, affected_spx[dim + 1]);
        if (dim + 1 == 3) assertx(affected_spx[3].empty());
        vs_new_ancestor->_parent.clear();
        destroySimplex(vs_new_ancestor, propagate_area);
        break;
      }
    }
  }

  dim = 2;
  while (!affected_spx[dim].empty()) {
    Simplex vs_new_ancestor = affected_spx[dim].pop();
    for (Simplex vs_ancestor : worklist[dim]) {  // was ForStack which went in reverse order
      assertx(vs_ancestor->getDim() == vs_new_ancestor->getDim());
      assertx(vs_new_ancestor != vs_ancestor);

      if (eq2simp(vs_new_ancestor, vs_ancestor)) {
        replace(vs_new_ancestor, vs_ancestor, affected_spx[dim + 1]);
        if (dim + 1 == 3) assertx(affected_spx[3].empty());

        // remove duplicate
        if (propagate_area) {
          Sarea_dropped.enter(vs_new_ancestor->getArea());
          Sarea_moved.enter(vs_new_ancestor->getArea());
          vs_ancestor->setArea(vs_ancestor->getArea() + vs_new_ancestor->getArea());
          vs_new_ancestor->setArea(0.f);
        }

        vs_new_ancestor->_parent.clear();
        destroySimplex(vs_new_ancestor, propagate_area);
        break;
      }
    }
  }

  // distribute vt area if any
  if (propagate_area && cmp_area != 0.f) {
    for (Simplex s : vs->get_star())
      if (s->isPrincipal()) {
        Sarea_moved.enter(cmp_area);
        s->setArea(s->getArea() + cmp_area);
        break;
      }
  }

  if (propagate_area) {
    assertx(Sarea_moved.sum() <= Sarea_dropped.sum());
    while (!check_principal.empty()) {
      Simplex s = check_principal.pop();
      assertx(s->isPrincipal());
    }
  }
}

void SimplicialComplex::replace(Simplex src, Simplex tgt, Stack<Simplex>& affected_parents) {
  // remove references from children
  for (auto [ci, c] : enumerate<int>(src->children())) {
    if (!c) continue;
    src->_child[ci] = nullptr;
    vec_remove_ordered(c->_parent, src);
  }

  // replace references from parents
  // and add reference to parent from tgt
  for (Simplex p : src->getParents()) {
    if (!p) continue;
    for (auto [ci, c] : enumerate<int>(p->children()))
      if (c == src) p->setChild(ci, tgt);

    if (!affected_parents.contains(p)) affected_parents.push(p);
    tgt->addParent(p);
  }
}

void SimplicialComplex::write(std::ostream& os) const {
  // dump materials

  if (_material_strings.num()) {
    os << "[Attributes]\n";
    for_int(attrid, _material_strings.num()) os << _material_strings[attrid] << "\n";
    os << "[EndAttributes]\n";
  }

  // dump simplicial complex
  for_int(dim, MAX_DIM + 1) {
    for (Simplex s : this->ordered_simplices_dim(dim)) {
      os << "Simplex " << dim << " " << s->getId() << "  ";
      if (dim == 0) {
        Point pos = s->getPosition();
        os << " " << pos[0] << " " << pos[1] << " " << pos[2];
      } else {  // dim != 0
        // iterate over children
        for (Simplex c : s->children()) os << " " << c->getId();
      }

      // print vattributes
      string out;
      if (s->get_string()) {
        if (!out.empty()) out += " ";
        out += s->get_string();
      }

      if (s->isPrincipal()) {
        assertx(s->getVAttribute() != -1);
        if (!out.empty()) out += " ";
        out += sform("attrid=%d", s->getVAttribute());
      }

      if (s->isPrincipal()) {
        if (!out.empty()) out += " ";
        out += sform("area=%g", s->getArea());
      }

      if (!out.empty()) os << "  {" << out << "}";
      assertx(os << "\n");
    }
  }
}

void SimplicialComplex::read(std::istream& is) {
  string line;
  auto parse_line = &SimplicialComplex::readLine;
  for (;;) {
    if (!my_getline(is, line)) break;
    if (line == "") continue;
    if (line == "#") break;        // done parsing simplex, before vsplit records
    if (line[0] == '#') continue;  // skip comment
    // if attribute change state and read next line
    if (starts_with(line, "[Attributes]")) {
      parse_line = &SimplicialComplex::attrReadLine;
      continue;
    }

    if (starts_with(line, "[EndAttributes]")) {
      parse_line = &SimplicialComplex::readLine;
      continue;
    }
    (this->*parse_line)(line.c_str());
  }
}

void SimplicialComplex::readLine(const char* str) {
  char* sline = const_cast<char*>(str);
  if (sline[0] == '#') return;
  char* va_field = strchr(sline, '{');
  if (va_field) {
    *va_field++ = 0;
    char* s = strchr(va_field, '}');
    if (!s) {
      if (Warning("No matching '}'")) SHOW(sline, va_field);
      va_field = nullptr;
    } else {
      *s = 0;
    }
  }
  if (const char* s = after_prefix(sline, "Simplex ")) {
    const int dim = int_from_chars(s), sid = int_from_chars(s);
    Simplex sd = assertx(createSimplex(dim, sid));
    // read and update children pointers
    if (dim == 0) {
      // read position
      Point pos;
      for_int(i, 3) pos[i] = float_from_chars(s);
      sd->setPosition(pos);
    } else {  // dim != 0
      // read connectivity
      for_int(i, dim + 1) {
        const int child = int_from_chars(s);
        Simplex spxChild = assertx(getSimplex(dim - 1, child));
        sd->setChild(i, spxChild);
      }
      // Update children's parent pointers
      for (Simplex c : sd->children()) c->addParent(sd);
    }
    while (std::isspace(*s)) s++;
    assert_no_more_chars(s);

    // read in vattributes
    sd->set_string(va_field);

    string str2;
    const char* attrid = GMesh::string_key(str2, va_field, "attrid");
    if (attrid) sd->setVAttribute(to_int(attrid));

    const char* area = GMesh::string_key(str2, va_field, "area");
    if (area) sd->setArea(to_float(area));
    return;
  }
  if (const char* s = after_prefix(sline, "Unify ")) {
    const int vi1 = int_from_chars(s), vi2 = int_from_chars(s);
    assert_no_more_chars(s);
    unify(getSimplex(0, vi1), getSimplex(0, vi2));
    return;
  }
  assertnever("Unrecognized line '" + string(sline) + "'");
}

void SimplicialComplex::attrReadLine(const char* str) {
  if (str[0] == '#') return;

  _material_strings.push(str);
}

// Construct skeleteon from this SC containing only simplices of dimension <= dim.
void SimplicialComplex::skeleton(int dim) {
  if (dim + 1 > MAX_DIM) return;
  for (Simplex s : Array<Simplex>{this->simplices_dim(dim + 1)}) destroySimplex(s);
}

// Read triangulation produced by quick hull and extend this SC with such triangulation.
void SimplicialComplex::readQHull(std::istream& is) {
  int n;
  assertx(is >> n);
  is >> std::ws;

  assertx(num(1) == 0);

  // verts might not be numbered in order
  Map<int, Simplex> id2vtx;
  int verts;
  {
    int i = 0;
    for (Simplex v : this->ordered_simplices_dim(0)) {
      id2vtx.enter(i, v);
      i++;
    }
    verts = num(0);
    assertx(i == verts);
  }

  // for all facets for triangulation
  string line;
  for_int(i, n) {
    if (!my_getline(is, line)) break;
    Vec4<int> v;
    const char* s = line.c_str();
    for_int(k, 4) v[k] = int_from_chars(s);
    assert_no_more_chars(s);
    for_int(j, 3) {
      if (v[j] >= verts) continue;

      for_intL(k, j + 1, 4) {
        if (v[k] >= verts) continue;

        Simplex vs = id2vtx.get(v[j]);
        Simplex vt = id2vtx.get(v[k]);
        assertx(vs && vt);

        // if there isn't already an edge, create one
        if (!vs->edgeTo(vt)) {
          Simplex e = createSimplex(1);
          assertx(e);

          e->setVAttribute(1);
          e->setChild(0, vs);
          e->setChild(1, vt);
          vs->addParent(e);
          vt->addParent(e);
        }
      }
    }
  }
}

#if 0
void SimplicialComplex::attrReadLine(char* sline) {
  if (sline[0] == '#') return;
  if (const char* s = after_prefix(sline, "Simplex ")) {
    const int dim = int_from_chars(s), sid = int_from_chars(s);
    // update position for 0-simplices
    if (dim == 0) {
      Point pos;
      for_int(i, 3) pos[i] = float_from_chars(s);
      getSimplex(dim, sid)->setPosition(pos);
    }
    // read and update color for 0,1,2-simp
    if (*s) {
      Point rgb;
      for_int(i, 3) rgb[i] = float_from_chars(s);
      getSimplex(dim, sid)->setColor(rgb);
    }
    assert_no_more_chars();
  }
}
#endif

int SimplicialComplex::compare_normal(const GMesh& mesh, Corner c1, Corner c2) {
  // if nothing to compare
  assertx(c1 && c2);
  Vector n1, n2;
  assertx(parse_key_vec(mesh.get_string(c1), "normal", n1));
  assertx(parse_key_vec(mesh.get_string(c2), "normal", n2));
  return compare(n1, n2, k_tolerance);
}

void SimplicialComplex::readGMesh(std::istream& is) {
  GMesh mesh;
  mesh.read(is);
  Simplex s0, s1, s2;
  Map<Vertex, Simplex> v2s0;
  Map<Edge, Simplex> e2s1;

  string str;
  for (Vertex v : mesh.vertices()) {
    s0 = createSimplex(0, mesh.vertex_id(v));
    v2s0.enter(v, s0);
    // no children
    // parent updated later when its created
    s0->setPosition(mesh.point(v));

    // compute normals
    Vnors vnors(mesh, v);

    for (Corner c : mesh.corners(v)) {
      Vector nor = vnors.get_nor(mesh.corner_face(c));

      // Normalize normals if necessary.
      assertx(nor[0] != k_undefined);  // normals always present
      // Always renormalize normal.
      if (!nor.normalize()) {
        Warning("Normal is zero, setting arbitrarily to (1, 0, 0)");
        nor = Vector(1.f, 0.f, 0.f);
      }

      mesh.update_string(c, "normal", csform_vec(str, nor));
    }
  }

  for (Edge e : mesh.edges()) {
    s1 = createSimplex(1);
    e2s1.enter(e, s1);
    // update children
    s1->setChild(0, v2s0.get(mesh.vertex1(e)));
    s1->setChild(1, v2s0.get(mesh.vertex2(e)));

    // update parent of the children
    s1->getChild(0)->addParent(s1);
    s1->getChild(1)->addParent(s1);
  }

  Map<Face, Simplex> mfs;
  for (Face f : mesh.faces()) {
    s2 = createSimplex(2, mesh.face_id(f));
    mfs.enter(f, s2);
    // update children
    int ind = 0;
    for (Edge e : mesh.edges(f)) {
      assertx(ind < 3);
      s2->setChild(ind, e2s1.get(e));
      ind++;
    }

    // update parent of the children
    for (Simplex c : s2->children()) c->addParent(s2);
  }

  {
    // If attrid keys present in input file, use them, else add new ones after the maximum found.
    // This is useful if output of simplification is re-simplified.
    // See identical code in MeshSimplify.cpp
    Set<string> hashstring;
    Map<Simplex, const string*> mssrep;
    for (Face f : mesh.faces()) {
      assertx(mesh.is_triangle(f));
      if (!mesh.get_string(f)) mesh.set_string(f, "");
      bool is_new;
      const string& srep = hashstring.enter(mesh.get_string(f), is_new);
      mssrep.enter(mfs.get(f), &srep);
    }
    Map<const string*, int> msrepattrid;
    for (const string& srep : hashstring) {
      const char* smat = GMesh::string_key(str, srep.c_str(), "attrid");
      if (!smat) continue;
      int attrid = to_int(smat);
      assertx(attrid >= 0);
      _material_strings.access(attrid);
      assertx(_material_strings[attrid] == "");  // no duplicate attrid's
      _material_strings[attrid] = srep;
      msrepattrid.enter(&srep, attrid);
    }
    showdf("Found %d materials with existing attrid (%d empty)\n",  //
           msrepattrid.num(), _material_strings.num() - msrepattrid.num());
    int nfirst = msrepattrid.num();
    // string str;
    for (const string& srep : hashstring) {
      if (GMesh::string_has_key(srep.c_str(), "attrid")) continue;  // handled above
      int attrid = _material_strings.add(1);
      _material_strings[attrid] = GMesh::string_update(srep, "attrid", csform(str, "%d", attrid));
      msrepattrid.enter(&srep, attrid);
    }
    showdf("Found %d materials without existing attrid\n", _material_strings.num() - nfirst);
    showdf("nmaterials=%d\n", _material_strings.num());
    for (Simplex s3 : this->simplices_dim(2)) s3->setVAttribute(msrepattrid.get(mssrep.get(s3)));
  }
}

// Allocates new simplex of dimension dim and inserts into simplicial complex.
// Note: children and parents if any must be specified later.
Simplex SimplicialComplex::createSimplex(int dim) {
  int sid = _free_sid[dim]++;
  Simplex s = new ISimplex(dim, sid);
  // _child = nullptr's; _parent = empty

  _simplices[dim].enter(sid, s);

  return s;
}

// Same as createSimplex(dim) but use the given id instead of next available one.
Simplex SimplicialComplex::createSimplex(int dim, int id) {
  int sid = id;
  Simplex s = new ISimplex(dim, id);
  // _child = nullptr's; _parent = empty

  _simplices[dim].enter(sid, s);

  _free_sid[dim] = id >= _free_sid[dim] ? id + 1 : _free_sid[dim];

  return s;
}

}  // namespace hh
