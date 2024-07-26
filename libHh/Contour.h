// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_CONTOUR_H_
#define MESH_PROCESSING_LIBHH_CONTOUR_H_

#include "libHh/GMesh.h"
#include "libHh/MeshOp.h"  // triangulate_face()
#include "libHh/PArray.h"
#include "libHh/Queue.h"
#include "libHh/SGrid.h"
#include "libHh/Set.h"
#include "libHh/Stat.h"

#if 0
{
  const auto func_eval = [](const Vec3<float>& p) {
    return p[0] < .3f ? k_Contour_undefined : dist(p, Point(.6f, .6f, .6f)) - .4f;
  };
  if (1) {
    GMesh mesh;
    Contour3DMesh contour(50, &mesh, func_eval);
    contour.march_near(Point(.9f, .6f, .6f));
    mesh.write(std::cout);
  } else {
    struct func_contour {
      void operator()(CArrayView<Vec3<float>>){...};
    };
    const auto func_border = [](CArrayView<Vec3<float>>) { ... };
    Contour3D contour(50, func_eval, func_contour(), func_border);
    contour.march_from(Point(.9f, .6f, .6f));
  }
}
#endif

namespace hh {

// Contour2D/Contour3DMesh/Contour3D compute a piecewise linear approximation to the zeroset of a scalar function:
//   - surface triangle mesh in the unit cube   (Contour3DMesh)
//   - surface triangle stream in the unit cube (Contour3D)
//   - curve polyline stream in the unit square (Contour2D)

// TODO: improving efficiency/generality:
// - use 64-bit encoding to allow larger grid sizes.
// - perhaps distinguish  Set<unsigned> cubes_visited and  Map<unsigned, Node>  cube_vertices? and edge_vertices too?
// - somehow remove _en from Node?
// - somehow remove mapsucc
// - Contour2D: directly extract joined polylines; no need to check degen

constexpr float k_Contour_undefined = 1e31f;  // represents undefined distance, to introduce surface boundaries

// Protected content in this class just factors functions common to Contour2D, Contour3DMesh, and Contour3D.
template <int D, typename VertexData = Vec0<int>> class ContourBase {
 public:
  void set_ostream(std::ostream* os) { _os = os; }  // for summary text output; may be set to nullptr
  void set_vertex_tolerance(float tol) {            // if nonzero, do binary search; tol is absolute distance in domain
    _vertex_tol = tol;
    _vertex_tol = getenv_float("CONTOUR_VERTEX_TOL", _vertex_tol, true);  // override
  }

 protected:
  static constexpr float k_not_yet_evaled = BIGFLOAT;
  using DPoint = Vec<float, D>;  // domain point
  using IPoint = Vec<int, D>;    // grid point
  static_assert(D == 2 || D == 3);
  static constexpr int k_max_gn = D == 3 ? 1024 : 65536;  // bits/coordinate == 10 for 3D, 16 for 2D (max 32 bits)
  explicit ContourBase(int gn) : _gn(gn), _gni(1.f / gn) {
    assertx(_gn > 0);
    assertx(_gn < k_max_gn);  // must leave room for [0 ... _gn] inclusive
    set_vertex_tolerance(_vertex_tol);
  }
  ~ContourBase() {
    assertx(_queue.empty());
    if (_os) {
      *_os << sform("%sMarch:\n", g_comment_prefix_string);
      *_os << sform("%svisited %d cubes (%d were undefined, %d contained nothing)\n",  //
                    g_comment_prefix_string, _ncvisited, _ncundef, _ncnothing);
      *_os << sform("%sevaluated %d vertices (%d were zero, %d were undefined)\n",  //
                    g_comment_prefix_string, _nvevaled, _nvzero, _nvundef);
      *_os << sform("%sencountered %d tough edges\n", g_comment_prefix_string, _nedegen);
    }
  }
  int _gn;
  float _gni;  // 1.f / _gn
  std::ostream* _os{&std::cerr};
  float _vertex_tol{0.f};  // note: 0.f is special: infinite tolerance
  // Model: the domain [0.f, 1.f] ^ D is partitioned into _gn ^ D cubes.
  // These cubes are indexed by nodes with indices [0, _gn - 1].
  // The cube vertices are indexed by nodes with indices [0, _gn].  See get_point().
  // So there are no "+ .5f" roundings anywhere in the code.
  struct Node : VertexData {
    explicit Node(unsigned pen) : _en(pen) {}
    enum class ECubestate { nothing, queued, visited };
    unsigned _en;                                // encoded vertex index
    ECubestate _cubestate{ECubestate::nothing};  // cube info
    float _val{k_not_yet_evaled};                // vertex value
    DPoint _p;                                   // vertex point position in grid
                                                 // Note that for 3D, base class contains Vec3<Vertex> _verts.
  };
  struct hash_Node {
    size_t operator()(const Node& n) const { return n._en; }
  };
  struct equal_Node {
    bool operator()(const Node& n1, const Node& n2) const { return n1._en == n2._en; }
  };
  Set<Node, hash_Node, equal_Node> _m;
  // (std::unordered_set<> : References and pointers to key stored in the container are only
  //   invalidated by erasing that element.  So it's OK to keep pointers to Node* even as more are added.)
  Queue<unsigned> _queue;  // cubes queued to be visited
  int _ncvisited{0};
  int _ncundef{0};
  int _ncnothing{0};
  int _nvevaled{0};
  int _nvzero{0};
  int _nvundef{0};
  int _nedegen{0};
  Array<DPoint> _tmp_poly;
  //
  bool cube_inbounds(const IPoint& ci) const { return ci.in_range(ntimes<D>(_gn)); }
  DPoint get_point(const IPoint& ci) const {
    // Note: less strict than cube_inbounds() because ci[c] == _gn is OK for a vertex.
    // DPoint dp; for_int(c, D) { ASSERTX(ci[c] >= 0 && ci[c] <= _gn); dp[c] = min(ci[c] * _gni, 1.f); }
    DPoint dp;
    for_int(c, D) {
      ASSERTX(ci[c] >= 0 && ci[c] <= _gn);
      dp[c] = ci[c] < _gn ? ci[c] * _gni : 1.f;
    }
    return dp;
  }
  template <bool avoid_degen, typename Eval = float(const DPoint&)>
  DPoint compute_point(const DPoint& pp, const DPoint& pn, float vp, float vn, Eval& eval) {
    DPoint pm;
    float fm;
    if (!_vertex_tol) {
      fm = vp / (vp - vn);
      pm = interp(pn, pp, fm);
    } else {
      float v0 = vp, v1 = vn;
      DPoint p0 = pp, p1 = pn;
      float f0 = 0.f, f1 = 1.f;
      int neval = 0;
      for (;;) {
        ASSERTX(v0 >= 0.f && v1 < 0.f && f0 < f1);
        float b1 = v0 / (v0 - v1);
        b1 = clamp(b1, .05f, .95f);  // guarantee quick convergence
        fm = f0 * (1.f - b1) + f1 * b1;
        pm = interp(p1, p0, b1);
        float vm = eval(pm);
        neval++;
        if (neval > 20) break;
        if (vm < 0.f) {
          f1 = fm;
          p1 = pm;
          v1 = vm;
        } else {
          f0 = fm;
          p0 = pm;
          v0 = vm;
        }
        if (dist2(p0, p1) <= square(_vertex_tol)) break;
      }
      HH_SSTAT(SContneval, neval);
    }
    if (avoid_degen) {
      // const float fs = _gn > 500 ? .05f : _gn > 100 ? .01f : .001f;
      const float fs = 2e-5f * _gn;  // sufficient precision for HashFloat with default nignorebits == 8
      if (fm < fs) {
        _nedegen++;
        pm = interp(pn, pp, fs);
      } else if (fm > 1.f - fs) {
        _nedegen++;
        pm = interp(pp, pn, fs);
      }
    }
    return pm;
  }
};

// *** Contour3D

struct Contour3D_NoBorder {  // special type to indicate that no border output is desired
  float operator()(CArrayView<Vec3<float>>) const {
    assertnever_ret("");
    return 0.f;
  }
};

struct VertexData3DMesh {
  Vec3<Vertex> _verts{ntimes<3>(implicit_cast<Vertex>(nullptr))};
};

template <typename VertexData = Vec0<int>,
          typename Derived = void,  // for contour_cube()
          typename Eval = float(const Vec3<float>&), typename Border = Contour3D_NoBorder>
class Contour3DBase : public ContourBase<3, VertexData> {
 protected:
  static constexpr int D = 3;
  using base = ContourBase<D, VertexData>;
  using base::_gn;
  using base::_m;
  using base::_ncnothing;
  using base::_ncundef;
  using base::_ncvisited;
  using base::_nvevaled;
  using base::_nvundef;
  using base::_nvzero;
  using base::_queue;
  using base::_tmp_poly;
  using base::cube_inbounds;
  using base::get_point;
  using base::k_max_gn;
  using typename base::DPoint;
  using typename base::IPoint;
  using typename base::Node;
  Derived& derived() { return *down_cast<Derived*>(this); }
  const Derived& derived() const { return *down_cast<const Derived*>(this); }

 public:
  explicit Contour3DBase(int gn, Eval eval, Border border) : base(gn), _eval(eval), _border(border) {}
  ~Contour3DBase() {}
  // ret number of new cubes visited: 0 = revisit_cube, 1 = no_surf, > 1 = new
  int march_from(const DPoint& startp) { return march_from_i(startp); }
  // call march_from() on all cells near startp; ret num new cubes visited
  int march_near(const DPoint& startp) { return march_near_i(startp); }

 protected:
  Eval _eval;
  Border _border;
  static constexpr bool b_no_border = std::is_same_v<Border, Contour3D_NoBorder>;
  using Node222 = SGrid<Node*, 2, 2, 2>;
  using base::k_not_yet_evaled;
  //
  unsigned encode(const IPoint& ci) const {
    static_assert(k_max_gn <= 1024);
    return (((unsigned(ci[0]) << 10) | unsigned(ci[1])) << 10) | unsigned(ci[2]);
  }
  IPoint decode(unsigned en) const {
    static_assert(k_max_gn <= 1024);
    return IPoint(narrow_cast<int>(en >> 20), narrow_cast<int>((en >> 10) & ((1u << 10) - 1)),
                  narrow_cast<int>(en & ((1u << 10) - 1)));
  }
  int march_from_i(const DPoint& startp) {
    for_int(d, D) ASSERTX(startp[d] >= 0.f && startp[d] <= 1.f);
    IPoint cc;
    for_int(d, D) cc[d] = min(int(startp[d] * _gn), _gn - 1);
    return march_from_aux(cc);
  }
  int march_near_i(const DPoint& startp) {
    for_int(d, D) ASSERTX(startp[d] >= 0.f && startp[d] <= 1.f);
    IPoint cc;
    for_int(d, D) cc[d] = min(int(startp[d] * _gn), _gn - 1);
    int ret = 0;
    IPoint ci;
    for_intL(i, -1, 2) {
      ci[0] = cc[0] + i;
      if (ci[0] < 0 || ci[0] >= _gn) continue;
      for_intL(j, -1, 2) {
        ci[1] = cc[1] + j;
        if (ci[1] < 0 || ci[1] >= _gn) continue;
        for_intL(k, -1, 2) {
          ci[2] = cc[2] + k;
          if (ci[2] < 0 || ci[2] >= _gn) continue;
          ret += march_from_aux(ci);
        }
      }
    }
    return ret;
  }
  int march_from_aux(const IPoint& cc) {
    int oncvisited = _ncvisited;
    {
      unsigned en = encode(cc);
      bool is_new;
      Node* n = const_cast<Node*>(&_m.enter(Node(en), is_new));  // un-const OK if not modify n->_en
      if (n->_cubestate == Node::ECubestate::visited) return 0;
      ASSERTX(n->_cubestate == Node::ECubestate::nothing);
      _queue.enqueue(en);
      n->_cubestate = Node::ECubestate::queued;
    }
    while (!_queue.empty()) {
      unsigned en = _queue.dequeue();
      consider_cube(en);
    }
    int cncvisited = _ncvisited - oncvisited;
    if (cncvisited == 1) _ncnothing++;
    return cncvisited;
  }
  void consider_cube(unsigned encube) {
    _ncvisited++;
    IPoint cc = decode(encube);
    Node222 na;
    bool cundef = false;
    for_int(i, 2) for_int(j, 2) for_int(k, 2) {
      IPoint cd(i, j, k);
      IPoint ci = cc + cd;
      unsigned en = encode(ci);
      bool is_new;
      Node* n = const_cast<Node*>(&_m.enter(Node(en), is_new));
      na[i][j][k] = n;
      if (n->_val == k_not_yet_evaled) {
        n->_p = get_point(ci);
        n->_val = _eval(n->_p);
        _nvevaled++;
        if (!n->_val) _nvzero++;
        if (n->_val == k_Contour_undefined) _nvundef++;
      }
      if (n->_val == k_Contour_undefined) cundef = true;
    }
    Node* n = na[0][0][0];
    ASSERTX(n->_cubestate == Node::ECubestate::queued);
    n->_cubestate = Node::ECubestate::visited;
    if (cundef) {
      _ncundef++;
    } else {
      derived().contour_cube(cc, na);
    }
    for_int(d, D) for_int(i, 2) {  // push neighbors
      int d1 = (d + 1) % D, d2 = (d + 2) % D;
      IPoint cd;
      cd[d] = i;
      float vmin = BIGFLOAT, vmax = -BIGFLOAT;
      for (cd[d1] = 0; cd[d1] < 2; cd[d1]++) {
        for (cd[d2] = 0; cd[d2] < 2; cd[d2]++) {
          float v = na[cd[0]][cd[1]][cd[2]]->_val;
          ASSERTX(v != k_not_yet_evaled);
          if (v < vmin) vmin = v;
          if (v > vmax) vmax = v;
        }
      }
      cd[d] = i ? 1 : -1;
      cd[d1] = cd[d2] = 0;
      IPoint ci = cc + cd;  // indices of node for neighboring cube;
      // note: vmin < 0 since 0 is arbitrarily taken to be positive
      if (vmax != k_Contour_undefined && vmin < 0 && vmax >= 0 && cube_inbounds(ci)) {
        unsigned en = encode(ci);
        bool is_new;
        Node* n2 = const_cast<Node*>(&_m.enter(Node(en), is_new));
        if (n2->_cubestate == Node::ECubestate::nothing) {
          n2->_cubestate = Node::ECubestate::queued;
          _queue.enqueue(en);
        }
      } else if (!b_no_border) {  // output boundary
        cd[d] = i;
        auto& poly = _tmp_poly;
        poly.init(0);
        for (cd[d1] = 0; cd[d1] < 2; cd[d1]++) {
          int sw = cd[d] ^ cd[d1];  // 0 or 1
          for (cd[d2] = sw; cd[d2] == 0 || cd[d2] == 1; cd[d2] += (sw ? -1 : 1)) poly.push(get_point(cc + cd));
        }
        _border(poly);
      }
    }
  }
};

template <typename Eval = float(const Vec3<float>&), typename Border = Contour3D_NoBorder>
class Contour3DMesh : public Contour3DBase<VertexData3DMesh, Contour3DMesh<Eval, Border>, Eval, Border> {
  using base = Contour3DBase<VertexData3DMesh, Contour3DMesh<Eval, Border>, Eval, Border>;

 public:
  explicit Contour3DMesh(int gn, GMesh* pmesh, Eval eval = Eval(), Border border = Border())
      : base(gn, eval, border), _pmesh(pmesh) {
    assertx(_pmesh);
  }
  void big_mesh_faces() { _big_mesh_faces = true; }

 private:
  // Need to friend base class for callback access to contour_cube().
  friend base;
  using base::_eval;
  using base::compute_point;
  using base::D;
  using base::decode;
  using typename base::IPoint;
  using typename base::Node;
  using typename base::Node222;
  GMesh* _pmesh;
  bool _big_mesh_faces{false};
  static int mod4(int j) { return (ASSERTX(j >= 0), j & 0x3); }
  void contour_cube(const IPoint& cc, const Node222& na) {
    // Based on Wyvill et al.
    dummy_use(cc);
    Map<Vertex, Vertex> mapsucc;
    for_int(d, D) for_int(v, 2) {  // examine each of 6 cube faces
      Vec4<Node*> naf;
      {
        int d1 = (d + 1) % D, d2 = (d + 2) % D;
        IPoint cd;
        cd[d] = v;
        int i = 0;
        // Gather 4 cube vertices in a consistent order
        for (cd[d1] = 0; cd[d1] < 2; cd[d1]++) {
          int sw = cd[d] ^ cd[d1];  // 0 or 1
          for (cd[d2] = sw; cd[d2] == 0 || cd[d2] == 1; cd[d2] += (sw ? -1 : 1)) naf[i++] = na[cd[0]][cd[1]][cd[2]];
        }
      }
      int nneg = 0;
      double sumval = 0.;
      for_int(i, 4) {
        float val = naf[i]->_val;
        if (val < 0) nneg++;
        sumval += val;  // If pedantic, could sort the vals before summing.
      }
      for_int(i, 4) {
        int i1 = mod4(i + 1), i2 = mod4(i + 2), i3 = mod4(i + 3);
        if (!(naf[i]->_val < 0 && naf[i1]->_val >= 0)) continue;
        // have start of edge
        ASSERTX(nneg >= 1 && nneg <= 3);
        int ie;  // end of edge
        if (nneg == 1) {
          ie = i3;
        } else if (nneg == 3) {
          ie = i1;
        } else if (naf[i2]->_val >= 0) {
          ie = i2;
        } else if (sumval < 0) {
          ie = i1;
        } else {
          ie = i3;
        }
        Vertex v1 = get_vertex_onedge(naf[i1], naf[i]);
        Vertex v2 = get_vertex_onedge(naf[ie], naf[mod4(ie + 1)]);
        mapsucc.enter(v2, v1);  // to get face order correct
      }
    }
    Vec<Vertex, 12> va;
    while (!mapsucc.empty()) {
      Vertex vf = nullptr;
      int minvi = std::numeric_limits<int>::max();  // find min to be portable
      for (Vertex v : mapsucc.keys()) {
        int vi = _pmesh->vertex_id(v);
        if (vi < minvi) {
          minvi = vi;
          vf = v;
        }
      }
      int nv = 0;
      for (Vertex v = vf;;) {
        va[nv++] = v;
        v = assertx(mapsucc.remove(v));
        if (v == vf) break;
      }
      Face f = _pmesh->create_face(CArrayView<Vertex>(va.data(), nv));
      if (nv > 3 && !_big_mesh_faces) {
        // If 6 or more edges, may have 2 edges on same cube face, then must introduce new vertex to be safe.
        if (nv >= 6)
          _pmesh->center_split_face(f);
        else
          assertx(triangulate_face(*_pmesh, f));
      }
    }
  }
  Vertex get_vertex_onedge(Node* n1, Node* n2) {
    bool is_new;
    Vertex* pv;
    {
      IPoint cc1 = decode(n1->_en);
      IPoint cc2 = decode(n2->_en);
      int d = -1;
      for_int(c, D) {
        if (cc1[c] != cc2[c]) {
          ASSERTX(d < 0);
          d = c;
        }
      }
      ASSERTX(d >= 0);
      ASSERTX(abs(cc1[d] - cc2[d]) == 1);
      Node* n = (cc1[d] < cc2[d]) ? n1 : n2;
      pv = &n->_verts[d];
      is_new = !*pv;
    }
    Vertex& v = *pv;
    if (is_new) {
      v = _pmesh->create_vertex();
      _pmesh->set_point(v, this->template compute_point<false>(n1->_p, n2->_p, n1->_val, n2->_val, _eval));
    }
    return v;
  }
};

template <typename Eval = float(const Vec3<float>&), typename Contour = float(CArrayView<Vec3<float>>),
          typename Border = Contour3D_NoBorder>
class Contour3D : public Contour3DBase<Vec0<int>, Contour3D<Eval, Contour, Border>, Eval, Border> {
  using base = Contour3DBase<Vec0<int>, Contour3D<Eval, Contour, Border>, Eval, Border>;

 public:
  explicit Contour3D(int gn, Contour contour = Contour(), Eval eval = Eval(), Border border = Border())
      : base(gn, eval, border), _contour(contour) {}

 private:
  // Need to friend base class for callback access to contour_cube().
  // friend base;  // somehow insufficient on mingw and clang (whereas somehow sufficient in Contour3DMesh)
  template <typename, typename, typename, typename> friend class Contour3DBase;
  Contour _contour;
  using base::_eval;
  using base::_tmp_poly;
  using base::compute_point;
  using typename base::DPoint;
  using typename base::IPoint;
  using typename base::Node;
  using typename base::Node222;
  void contour_cube(const IPoint& cc, const Node222& na) {
    dummy_use(cc);
    // do Kuhn 6-to-1 triangulation of cube
    contour_tetrahedron(V(na[0][0][0], na[0][0][1], na[1][0][1], na[0][1][0]));
    contour_tetrahedron(V(na[0][0][0], na[1][0][1], na[1][0][0], na[0][1][0]));
    contour_tetrahedron(V(na[1][0][1], na[1][1][0], na[1][0][0], na[0][1][0]));
    contour_tetrahedron(V(na[0][1][0], na[0][1][1], na[0][0][1], na[1][0][1]));
    contour_tetrahedron(V(na[1][1][1], na[0][1][1], na[0][1][0], na[1][0][1]));
    contour_tetrahedron(V(na[1][1][1], na[0][1][0], na[1][1][0], na[1][0][1]));
  }
  void contour_tetrahedron(Vec4<Node*> n4) {
    int nposi = 0;
    for_int(i, 4) {
      if (n4[i]->_val >= 0) nposi++;
    }
    if (nposi == 0 || nposi == 4) return;
    for (int i = 0, j = 3; i < j;) {
      if (n4[i]->_val >= 0) {
        i++;
        continue;
      }
      if (n4[j]->_val < 0) {
        --j;
        continue;
      }
      std::swap(n4[i], n4[j]);
      i++;
      --j;
    }
    switch (nposi) {
      case 1: output_triangle(V(V(n4[0], n4[1]), V(n4[0], n4[2]), V(n4[0], n4[3]))); break;
      case 2:
        output_triangle(V(V(n4[0], n4[2]), V(n4[0], n4[3]), V(n4[1], n4[3])));
        output_triangle(V(V(n4[0], n4[2]), V(n4[1], n4[3]), V(n4[1], n4[2])));
        break;
      case 3: output_triangle(V(V(n4[0], n4[3]), V(n4[1], n4[3]), V(n4[2], n4[3]))); break;
      default: assertnever("");
    }
  }
  void output_triangle(const SGrid<Node*, 3, 2>& n3) {
    auto& poly = _tmp_poly;
    poly.init(3);
    for_int(i, 3) {
      Node* np = n3[i][0];
      Node* nn = n3[i][1];
      poly[i] = this->template compute_point<true>(np->_p, nn->_p, np->_val, nn->_val, _eval);
    }
    Vector normal = cross(poly[0], poly[1], poly[2]);
    // swap might be unnecessary if we carefully swapped above?
    if (dot(normal, n3[0][0]->_p - n3[0][1]->_p) < 0.f) std::swap(poly[0], poly[1]);
    _contour(poly);
  }
};

// *** Contour2D

struct Contour2D_NoBorder {  // special type to indicate that no border output is desired
  float operator()(CArrayView<Vec2<float>>) const {
    if (1) assertnever("");
    return 0.f;
  }
};

template <typename Eval = float(const Vec2<float>&), typename Contour = void(CArrayView<Vec2<float>>),
          typename Border = Contour2D_NoBorder>
class Contour2D : public ContourBase<2> {
  static constexpr int D = 2;
  using base = ContourBase<D>;

 public:
  explicit Contour2D(int gn, Eval eval = Eval(), Contour contour = Contour(), Border border = Border())
      : base(gn), _eval(eval), _contour(contour), _border(border) {}
  ~Contour2D() {}
  // ret number of new cubes visited: 0=revisit_cube, 1=no_surface, >1=new
  int march_from(const DPoint& startp) { return march_from_i(startp); }
  // call march_from() on all cells near startp; ret num new cubes visited
  int march_near(const DPoint& startp) { return march_near_i(startp); }

 private:
  Eval _eval;
  Contour _contour;
  Border _border;
  static constexpr bool b_no_border = std::is_same_v<Border, Contour2D_NoBorder>;
  using Node22 = SGrid<Node*, 2, 2>;
  using base::k_not_yet_evaled;
  //
  unsigned encode(const IPoint& ci) const {
    static_assert(k_max_gn <= 65536);
    return (unsigned(ci[0]) << 16) | unsigned(ci[1]);
  }
  IPoint decode(unsigned en) const {
    static_assert(k_max_gn <= 65536);
    return IPoint(narrow_cast<int>(en >> 16), narrow_cast<int>(en & ((1u << 16) - 1)));
  }
  int march_from_i(const DPoint& startp) {
    for_int(d, D) ASSERTX(startp[d] >= 0.f && startp[d] <= 1.f);
    IPoint cc;
    for_int(d, D) cc[d] = min(int(startp[d] * _gn), _gn - 1);
    return march_from_aux(cc);
  }
  int march_near_i(const DPoint& startp) {
    for_int(d, D) ASSERTX(startp[d] >= 0.f && startp[d] <= 1.f);
    IPoint cc;
    for_int(d, D) cc[d] = min(int(startp[d] * _gn), _gn - 1);
    int ret = 0;
    IPoint ci;
    for_intL(i, -1, 2) {
      ci[0] = cc[0] + i;
      if (ci[0] < 0 || ci[0] >= _gn) continue;
      for_intL(j, -1, 2) {
        ci[1] = cc[1] + j;
        if (ci[1] < 0 || ci[1] >= _gn) continue;
        ret += march_from_aux(ci);
      }
    }
    return ret;
  }
  int march_from_aux(const IPoint& cc) {
    int oncvisited = _ncvisited;
    {
      unsigned en = encode(cc);
      bool is_new;
      Node* n = const_cast<Node*>(&_m.enter(Node(en), is_new));
      if (n->_cubestate == Node::ECubestate::visited) return 0;
      ASSERTX(n->_cubestate == Node::ECubestate::nothing);
      _queue.enqueue(en);
      n->_cubestate = Node::ECubestate::queued;
    }
    while (!_queue.empty()) {
      unsigned en = _queue.dequeue();
      consider_square(en);
    }
    int cncvisited = _ncvisited - oncvisited;
    if (cncvisited == 1) _ncnothing++;
    return cncvisited;
  }
  void consider_square(unsigned encube) {
    _ncvisited++;
    IPoint cc = decode(encube);
    Node22 na;
    {
      bool cundef = false;
      IPoint cd;
      for (cd[0] = 0; cd[0] < 2; cd[0]++) {
        for (cd[1] = 0; cd[1] < 2; cd[1]++) {
          IPoint ci = cc + cd;
          unsigned en = encode(ci);
          bool is_new;
          Node* n = const_cast<Node*>(&_m.enter(Node(en), is_new));
          na[cd[0]][cd[1]] = n;
          if (n->_val == k_not_yet_evaled) {
            n->_p = get_point(ci);
            n->_val = _eval(n->_p);
            _nvevaled++;
            if (!n->_val) _nvzero++;
            if (n->_val == k_Contour_undefined) _nvundef++;
          }
          if (n->_val == k_Contour_undefined) cundef = true;
        }
      }
      Node* n = na[0][0];
      ASSERTX(n->_cubestate == Node::ECubestate::queued);
      n->_cubestate = Node::ECubestate::visited;
      if (cundef) {
        _ncundef++;
      } else {
        contour_square(na);
      }
    }
    for_int(d, D) for_int(i, 2) {  // push neighbors
      int d1 = (d + 1) % D;
      IPoint cd;
      cd[d] = i;
      float vmin = BIGFLOAT, vmax = -BIGFLOAT;
      for (cd[d1] = 0; cd[d1] < 2; cd[d1]++) {
        float v = na[cd[0]][cd[1]]->_val;
        ASSERTX(v != k_not_yet_evaled);
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
      }
      cd[d] = i ? 1 : -1;
      cd[d1] = 0;
      IPoint ci = cc + cd;  // indices of node for neighboring cube;
      // note: vmin < 0 since 0 is arbitrarily taken to be positive
      if (vmax != k_Contour_undefined && vmin < 0 && vmax >= 0 && cube_inbounds(ci)) {
        unsigned en = encode(ci);
        bool is_new;
        Node* n = const_cast<Node*>(&_m.enter(Node(en), is_new));
        if (n->_cubestate == Node::ECubestate::nothing) {
          n->_cubestate = Node::ECubestate::queued;
          _queue.enqueue(en);
        }
      } else if (!b_no_border) {  // output boundary
        cd[d] = i;
        auto& poly = _tmp_poly;
        poly.init(0);
        for (cd[d1] = 0; cd[d1] < 2; cd[d1]++) poly.push(get_point(cc + cd));
        _border(poly);
      }
    }
  }
  void contour_square(const Node22& na) {
    contour_triangle(V(na[0][0], na[1][1], na[0][1]));
    contour_triangle(V(na[0][0], na[1][0], na[1][1]));
  }
  void contour_triangle(Vec3<Node*> n3) {
    int nposi = 0;
    for_int(i, 3) {
      if (n3[i]->_val >= 0) nposi++;
    }
    if (nposi == 0 || nposi == 3) return;
    for (int i = 0, j = 2; i < j;) {
      if (n3[i]->_val >= 0) {
        i++;
        continue;
      }
      if (n3[j]->_val < 0) {
        --j;
        continue;
      }
      std::swap(n3[i], n3[j]);
      i++;
      --j;
    }
    switch (nposi) {
      case 1: output_line(V(V(n3[0], n3[1]), V(n3[0], n3[2]))); break;
      case 2: output_line(V(V(n3[0], n3[2]), V(n3[1], n3[2]))); break;
      default: assertnever("");
    }
  }
  void output_line(const Node22& n2) {
    auto& poly = _tmp_poly;
    poly.init(2);
    for_int(i, 2) {
      Node* np = n2[i][0];
      Node* nn = n2[i][1];
      poly[i] = compute_point<true>(np->_p, nn->_p, np->_val, nn->_val, _eval);
    }
    Vec2<float> v = poly[1] - poly[0];
    Vec2<float> normal(-v[1], v[0]);  // 90 degree rotation
    if (dot(normal, n2[0][0]->_p - n2[0][1]->_p) < 0.) std::swap(poly[0], poly[1]);
    _contour(poly);
  }
};

// Template deduction guides:

template <typename Eval, typename Border>
Contour3DMesh(int gn, GMesh* pmesh, Eval eval, Border border) -> Contour3DMesh<Eval, Border>;

template <typename Eval, typename Border>
Contour3D(int gn, GMesh* pmesh, Eval eval, Border border) -> Contour3D<Eval, Border>;

template <typename Eval, typename Contour, typename Border>
Contour2D(int gn, Eval eval, Contour contour, Border border) -> Contour2D<Eval, Contour, Border>;

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_CONTOUR_H_
