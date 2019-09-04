// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "SubMesh.h"

#include "Homogeneous.h"
#include "MeshOp.h"             // edge_dihedral_angle_cos()
#include "Set.h"
#include "Queue.h"
#include "RangeOp.h"            // is_zero()

namespace hh {

// On 1994-04-18, eliminated capacity to get higher order splines on creases.
// This did not seem to make sense in conjunction with extraordinary crease vertices.
// Also, this greatly reducing bookkeeping (msharpvv disappears)

// _mofif and _mfindex work by assuming that two faces--whose vertices are in
// the same order and have the same ids--will be subdivided in the same order
// regardless of the mesh they belong to.
// This is true because of Mesh::vertices and because of Mesh::ordered_faces().

namespace {

struct subvertexinfo {
    int nume;
    int numsharpe;
};
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, subvertexinfo, vinfo);

bool debug() {
    static const bool value = getenv_bool("SUBMESH_DEBUG");
    return value;
}

// *** helper

// translate Vertex from one Mesh to another Mesh
inline Vertex trvmm(Vertex v, const Mesh& mf, const Mesh& mt) {
    return mt.id_vertex(mf.vertex_id(v));
}

// translate Face from one Mesh to another Mesh
inline Face trfmm(Face f, const Mesh& mf, const Mesh& mt) {
    return mt.id_face(mf.face_id(f));
}

// translate Edge from one Mesh to another Mesh
// inline Edge tremm(Edge e, const Mesh& mf, const Mesh& mt) {
//     return mt.edge(trvmm(mf.vertex1(e), mf, mt), trvmm(mf.vertex2(e), mf, mt));
// }

} // namespace

const FlagMask SubMesh::vflag_variable = Mesh::allocate_Vertex_flag();

// *** Combvh

HH_ALLOCATE_POOL(Combvh);

bool Combvh::is_combination() const {
    return abs(c.sum()+h[3]-1.f)<1e-6f;
}

Point Combvh::evaluate(const GMesh& mesh) const {
    Homogeneous th(h);
    for_combination(c, [&](Vertex vv, float val) {
        th += val*Homogeneous(mesh.point(vv));
    });
    return to_Point(th);
}

// *** Mvcvh

bool Mvcvh::is_convolution() const {
    for (const Combvh& comb : values()) {
        if (!is_zero(comb.h)) return false;
    }
    return true;
}

// co=ci*this
Combvh Mvcvh::compose_c(const Combvh& ci) const {
    Combvh co;
    co.h = ci.h;
    for_combination(ci.c, [&](Vertex v, float val) {
        bool present; const Combvh& comb = retrieve(v, present);
        if (!present) {
            // missing entry -> assume identity map
            co.c[v] += val;
        } else {
            co.h += comb.h*val;
            if (comb.c.num()) {
                for_combination(comb.c, [&](Vertex v2, float val2) {
                    co.c[v2] += val2*val;
                });
            }
        }
    });
    return co;
}

// this=mconv*this
void Mvcvh::compose(const Mvcvh& mconv) {
    Mvcvh nthis;
    for_map_key_value(mconv, [&](Vertex v, const Combvh& comb) {
        assertx(is_zero(comb.h));
        if (!comb.c.num()) {
            // identity assumed, keep unchanged
        } else if (comb.c.num()==1) {
            Warning("Waste of space");
            assertw(comb.c[v]==1.f);
        } else {
            Combvh ncomb = compose_c(comb);
            if (debug()) assertx(ncomb.is_combination());
            nthis.enter(v, std::move(ncomb));
        }
    });
    // swap(*this, nthis) is wrong because nthis only has entries for changed vertices.
    while (!nthis.empty()) {
        Vertex v = nthis.get_one_key();
        (*this)[v] = nthis.remove(v);
    }
}

// *** SubMesh

// weight of center vertex in position vertex mask, Loop scheme
// It is the weight that appears in the subdivision matrix,
// not the weight used when splitting+averaging.
static inline float subdiv_a(int n) {
    return n==6 ? .625f : 3.f/8.f+square((3.f+2.f*cos(TAU/n))/8.f);
}
// 3, 0.4375   4, 0.515625   5, 0.579534   6, 0.625   7, 0.656826   1000, 0.765619

SubMesh::SubMesh(GMesh& pmesh) : _omesh(pmesh) {
    _m.copy(_omesh);            // vertex ids will match, flags copied
    {
        bool have_quads = false, have_tris = false;
        for (Face f : _m.faces()) {
            int nv = _m.num_vertices(f); assertx(nv<=4);
            if (nv==3) { have_tris = true; } else { have_quads = true; }
        }
        assertx(have_tris ^ have_quads);
        _isquad = have_quads;
    }
    for (Face f : _m.faces()) {
        _mforigf.enter(f, trfmm(f, _m, _omesh));
        _mfindex.enter(f, 0);
    }
    for (Face f : _omesh.faces()) {
        bool is_new; Array<Face>& ar = _mofif.enter(f, Array<Face>(), is_new); assertx(is_new);
        ar.push(trfmm(f, _omesh, _m));
    }
    for (Vertex v : _m.vertices()) {
        int nsharpe = 0; for (Edge e : _m.edges(v)) { if (sharp(e)) nsharpe++; }
        vinfo(v).nume = _m.degree(v);
        vinfo(v).numsharpe = nsharpe;
    }
    _allvvar = true;
    for (Vertex v : _m.vertices()) {
        Combvh comb;
        if (_isquad && _m.degree(v)==2) {
            // degree-2 vertex must be centroid of its neighbors.
            for (Vertex vo : _m.vertices(v)) {
                comb.c[trvmm(vo, _m, _omesh)] = .25f;
                Vertex vs = assertx(_m.clw_vertex(vo, v));
                comb.c[trvmm(vs, _m, _omesh)] = .25f;
            }
        } else if (_m.flags(v).flag(vflag_variable)) {
            comb.c[trvmm(v, _m, _omesh)] = 1.f;
        } else {
            comb.h = _omesh.point(trvmm(v, _m, _omesh));
            _allvvar = false;
        }
        _cmvcvh.enter(v, std::move(comb));
    }
}

void SubMesh::clear() {
    _m.clear();
    _cmvcvh.clear();
    _mforigf.clear();
    _mfindex.clear();
    _mofif.clear();
}

// *** subdivide

void SubMesh::subdivide(float cosang) {
    if (_allvvar) {
        Mvcvh mconv;
        subdivide_aux(cosang, &mconv);
        convolve_self(mconv);
    } else {
        subdivide_aux(cosang, nullptr);
    }
}

void SubMesh::subdivide_n(int nsubdiv, int limit, float cosang, bool triang) {
    // whichever is faster
    if (_allvvar) {
        Mvcvh mconv;
        for_int(i, nsubdiv) {
            Mvcvh mconv1;
            subdivide_aux(cosang, &mconv1);
            mconv.compose(mconv1);
        }
        if (limit) {
            Mvcvh mconv1;
            create_conv(mconv1, &SubMesh::limit_mask);
            mconv.compose(mconv1);
        }
        convolve_self(mconv);
    } else {
        for_int(i, nsubdiv) {
            subdivide_aux(cosang, nullptr);
        }
        if (limit) {
            Mvcvh mconv;
            create_conv(mconv, &SubMesh::limit_mask);
            convolve_self(mconv);
        }
    }
    if (_isquad && triang) {
        Mvcvh mconv;
        triangulate_quads(mconv);
        convolve_self(mconv);
    }
}

void SubMesh::subdivide_aux(float cosang, Mvcvh* pmconv) {
    {
        unique_ptr<Mvcvh> tmconv = !pmconv ? make_unique<Mvcvh>() : nullptr;
        Mvcvh& mconv = pmconv ? *pmconv : *tmconv;
        if (cosang<.99999f) {
            // udpate geometry so that EdgeDihedralCos makes sense
            update_vertex_positions();
            _selrefine = true;
            selectively_refine(mconv, cosang);
        } else {
            refine(mconv);
        }
        if (!pmconv) convolve_self(mconv);
    }
    Mvcvh mconv2;
    create_conv(mconv2, &SubMesh::averaging_mask);
    if (!pmconv) convolve_self(mconv2);
    else pmconv->compose(mconv2);
    _selrefine = false;
}

// *** compute convolutions

void SubMesh::refine(Mvcvh& mconv) {
    // Save current mesh objects for later iteration
    // Array<Vertex> arv; for (Vertex v : _m.vertices()) { arv += v; }
    Array<Face> arf; for (Face f : _m.ordered_faces()) { arf.push(f); }
    Array<Edge> are; for (Edge e : _m.edges()) { are.push(e); }
    Map<Edge,Vertex> menewv;
    // Create new vertices and make them midpoints of old edges
    for (Edge e : are) {        // was ForStack which went in reverse order
        Vertex v = _m.create_vertex(); menewv.enter(e, v);
        vinfo(v).nume = (_isquad
                         ? (_m.is_boundary(e) ? 3 : 4)
                         : (_m.is_boundary(e) ? 4 : 6));
        vinfo(v).numsharpe = sharp(e) ? 2 : 0;
        Combvh comb;
        comb.c[_m.vertex1(e)] = .5f;
        comb.c[_m.vertex2(e)] = .5f;
        mconv.enter(v, std::move(comb));
    }
    Map<Face,Vertex> mfnewv;
    if (_isquad) {
        for (Face f : arf) {    // was ForStack which went in reverse order
            Vertex v = _m.create_vertex(); mfnewv.enter(f, v);
            vinfo(v).nume = 4; vinfo(v).numsharpe = 0;
            Combvh comb;
            for (Vertex vv : _m.vertices(f)) { comb.c[vv] = .25f; }
            mconv.enter(v, std::move(comb));
        }
    }
    // Create new triangulation in situ with old one.
    if (_isquad) {
        const bool has_uv = getenv_bool("HAS_UV");
        const bool has_imagen = getenv_bool("HAS_IMAGEN");
        int nn = 0;
        if (has_imagen) {
            nn = int(sqrt(float(_m.num_faces())));
            assertx(square(nn)==_m.num_faces());
        }
        string str;
        Array<Vertex> va;
        for (Face f : arf) {    // was ForStack which went in reverse order
            _m.get_vertices(f, va); assertx(va.num()==4);
            Vec4<Vertex> vs;
            for_int(i, 4) {
                vs[i] = menewv.get(_m.edge(va[i], va[(i+1)%4]));
            }
            Vertex vc = mfnewv.get(f);
            Face forig = _mforigf.get(f);
            Array<Face>& ar = _mofif.get(forig);
            for_int(i, 4) {
                Face fn = _m.create_face(V(vc, vs[(i+3)%4], va[i], vs[i]));
                _mforigf.enter(fn, forig);
                _mfindex.enter(fn, ar.num());
                ar.push(fn);
            }
            if (has_uv) {
                Vec4<UV> uva;
                Face* fna = &ar[unsigned(ar.num())-4]; // unsigned to avoid -Werror=strict-overflow
                for_int(i, 4) {
                    Corner c = _m.corner(va[i], f);
                    UV& uv = uva[i];
                    assertx(_m.parse_corner_key_vec(c, "uv", uv));
                    c = _m.corner(va[i], fna[i]);
                    _m.update_string(c, "uv", csform_vec(str, uv));
                }
                for_int(i, 4) {
                    UV uv = interp(uva[i], uva[(i+1)%4]);
                    for_int(j, 2) {
                        Corner cc = _m.corner(vs[i], fna[(i+j)%4]);
                        _m.update_string(cc, "uv", csform_vec(str, uv));
                    }
                }
                UV uv = interp(interp(uva[0], uva[1]), interp(uva[2], uva[3]));
                _m.update_string(vc, "uv", csform_vec(str, uv));
            }
            if (has_imagen) {
                Vec4<int> ina, ins;
                Face* fna = &ar[unsigned(ar.num())-4]; // unsigned to avoid -Werror=strict-overflow
                for_int(i, 4) {
                    Corner c = _m.corner(va[i], f);
                    int inv = assertx(to_int(assertx(_m.corner_key(str, c, "imagen"))))-1;
                    int x = inv/(nn+1), y = inv%(nn+1);
                    assertx(x<=nn);
                    ina[i] = (x*2)*(nn*2+1)+(y*2)+1;
                    c = _m.corner(va[i], fna[i]);
                    _m.update_string(c, "imagen", csform(str, "%d", ina[i]));
                }
                for_int(i, 4) {
                    ins[i] = (ina[i]+ina[(i+1)%4])/2;
                    for_int(j, 2) {
                        _m.update_string(_m.corner(vs[i], fna[(i+j)%4]), "imagen", csform(str, "%d", ins[i]));
                    }
                }
                int inc = (ins[0]+ins[2])/2;
                for (Corner c : _m.corners(vc)) {
                    _m.update_string(c, "imagen", csform(str, "%d", inc));
                }
            }
        }
        if (has_imagen) {
            for (Vertex v : _m.vertices()) { _m.update_string(v, "imagen", nullptr); }
        }
    } else {
        for (Face f : arf) {    // was ForStack which went in reverse order
            Vec3<Vertex> va; _m.triangle_vertices(f, va);
            Vec3<Vertex> vs;
            for_int(i, 3) {
                vs[i] = menewv.get(_m.edge(va[i], va[mod3(i+1)]));
            }
            Face forig = _mforigf.get(f);
            Array<Face>& ar = _mofif.get(forig);
            for_int(i, 3) {
                Face fn = _m.create_face(va[i], vs[i], vs[mod3(i+2)]);
                _mforigf.enter(fn, forig);
                _mfindex.enter(fn, ar.num());
                ar.push(fn);
            }
            Face fn = _m.create_face(vs[0], vs[1], vs[2]);
            _mforigf.enter(fn, forig);
            _mfindex.enter(fn, ar.num());
            ar.push(fn);
        }
    }
    // Update sharp edges
    for (Edge e : are) {
        Vertex vnew = menewv.get(e);
        if (!_m.flags(e)) continue;
        _m.flags(_m.edge(vnew, _m.vertex1(e))) = _m.flags(e);
        _m.flags(_m.edge(vnew, _m.vertex2(e))) = _m.flags(e);
    }
    // Remove old triangulation
    for (Face f : arf) {
        _m.destroy_face(f);
        assertx(_mforigf.remove(f));
        _mfindex.remove(f);     // can be index 0
    }
}

// *** selectively_refine

void SubMesh::selectively_refine(Mvcvh& mconv, float cosang) {
    // e.g.: Filtermesh ~/data/mesh/cat.m -angle 40 -mark | Subdivfit -mf - -selective 170 -nsub 2 -outn >v.m
    //       Subdivfit -mf ~/data/mesh/cat.m -selective 40 -nsub 2 -outn >v
    // See also Filtermesh.cpp:do_silsubdiv()
    assertx(!_isquad);
    // _mforigf, _mfindex, and _mofif are not supported with this scheme!
    Array<Face> arf; for (Face f : _m.faces()) { arf.push(f); }
    // Determine which edges will be subdivided
    Set<Edge> subde;            // edges to subdivide
    for (Edge e : _m.edges()) {
        if (sharp(e) || edge_dihedral_angle_cos(_m, e)<=cosang)
            subde.enter(e);
    }
    Queue<Face> queuef;
    for (Face f : arf) { queuef.enqueue(f); }
    while (!queuef.empty()) {
        Face f = queuef.dequeue();
        int nnew = 0;
        for (Edge e : _m.edges(f)) {
            if (subde.contains(e)) nnew++;
        }
        if (nnew!=2) continue;  // ok, no propagating changes
        for (Edge e : _m.edges(f)) {
            if (!subde.add(e)) continue;
            Face f2 = _m.opp_face(f, e);
            if (f2) queuef.enqueue(f2);
        }
    }
    // Introduce new vertices at midpoints
    struct Snvf { Vertex vnew; Flags eflags; };
    struct Svv {
        Svv(Vertex v1, Vertex v2) : _v1(v1), _v2(v2) { if (_v2<_v1) std::swap(_v1, _v2); }
        Vertex _v1, _v2;
    };
    struct hash_Svv {
        hash_Svv(const GMesh* mesh)             : _mesh(*assertx(mesh)) { }
        size_t operator()(const Svv& s) const {
            return _mesh.vertex_id(s._v1)+intptr_t{_mesh.vertex_id(s._v2)}*761;
        }
        const GMesh& _mesh;
    };
    struct equal_Svv {          // : std::equal_to<Svv>
        bool operator()(const Svv& s1, const Svv& s2) const { return s1._v1==s2._v1 && s1._v2==s2._v2; }
    };
    hash_Svv hashf(&_m);
    Map<Svv, Snvf, hash_Svv, equal_Svv> mvvnewv(hashf);
    for (Edge e : subde) {
        Vertex v = _m.create_vertex();
        vinfo(v).nume = _m.is_boundary(e) ? 4 : 6;
        vinfo(v).numsharpe = sharp(e) ? 2 : 0;
        mvvnewv.enter(Svv(_m.vertex1(e), _m.vertex2(e)), Snvf{v, _m.flags(e)});
        Combvh comb;
        comb.c[_m.vertex1(e)] = .5f;
        comb.c[_m.vertex2(e)] = .5f;
        mconv.enter(v, std::move(comb));
    }
    // Subdivide faces, destroys validity of flags(e)
    for (Face f : arf) {
        int nnew = 0, i0 = -1;
        Vec3<Vertex> va; _m.triangle_vertices(f, va);
        Vec3<Vertex> vs;
        for_int(i, 3) {
            Edge e = _m.edge(va[i], va[mod3(i+1)]);
            bool present; const Snvf& nvf = mvvnewv.retrieve(Svv(_m.vertex1(e), _m.vertex2(e)), present);
            vs[i] = present ? nvf.vnew : nullptr;
            if (vs[i]) { nnew++; i0 = i; }
        }
        if (!nnew) continue;
        assertx(nnew==1 || nnew==3);
        _m.destroy_face(f);
        if (nnew==1) {
            int i1 = mod3(i0+1), i2 = mod3(i0+2);
            _m.create_face(va[i0], vs[i0], va[i2]);
            _m.create_face(vs[i0], va[i1], va[i2]);
        } else {
            for_int(i, 3) { _m.create_face(va[i], vs[i], vs[mod3(i+2)]); }
            _m.create_face(vs[0], vs[1], vs[2]);
        }
    }
    for_map_key_value(mvvnewv, [&](const Svv& vv, const Snvf& nvf) {
        if (nvf.eflags) {
            _m.flags(_m.edge(nvf.vnew, vv._v1)) = nvf.eflags;
            _m.flags(_m.edge(nvf.vnew, vv._v2)) = nvf.eflags;
        }
    });
    if (1) {                    // need to fix up some values
        for (Vertex v : _m.vertices()) {
            int nsharpe = 0; for (Edge e : _m.edges(v)) { if (sharp(e)) nsharpe++; }
            vinfo(v).nume = _m.degree(v);
            vinfo(v).numsharpe = nsharpe;
        }
    }
}

void SubMesh::create_conv(Mvcvh& mconv, FVMASK fsubdivision) {
    assertx(mconv.empty());
    for (Vertex v : _m.vertices()) {
        Combvh comb;
        (this->*fsubdivision)(v, comb);
        assertx(is_zero(comb.h));
        if (comb.c.empty()) continue;
        if (debug()) assertx(comb.is_combination());
        mconv.enter(v, std::move(comb));
    }
}

// *** averaging masks and limit masks

namespace {

// Central weight for the subdivision masks at a cone, as function of n

constexpr auto k_table_cone_subd =
    V(0.f, 0.f, 0.f, .625f, .75f, .827254f, .875f, .905872f, .926777f, .941511f, .952254f);

constexpr auto k_table_cone_limit =
    V(0.f, 0.f, 0.f, .5f, .6f, .684624f, .75f, .799356f, .836637f, .865074f, .887058f);

} // namespace

void SubMesh::averaging_mask(Vertex v, Combvh& comb) const {
    int ne = nume(v), nesharp = num_sharp_edges(v);
    if (_isquad) {
        assertx(!nesharp); assertx(ne==2 || ne==4);
        ASSERTX(ne==_m.degree(v));
        // (1, 2, 1) x (1, 2, 1)
        comb.c[v] = 4.f/16.f;
        for_int(nweight, 1+(ne==2)) {
            for (Vertex vv : _m.vertices(v)) {
                comb.c[vv] += 2.f/16.f;
                Vertex vo = assertx(_m.clw_vertex(vv, v));
                comb.c[vo] += 1.f/16.f;
            }
        }
        ASSERTX(comb.is_combination());
        return;
    }
    float wa;                                  // central weight
    if (_m.flags(v).flag(GMesh::vflag_cusp)) { // cusp vertex
        if (nesharp>=2) return;     // becomes corner vertex
        int nuse = ne;
        if (!assertw(nuse<k_table_cone_subd.num())) nuse = k_table_cone_subd.num()-1;
        wa = k_table_cone_subd[nuse]*2-1;
        if (_weighta) wa = _weighta*2-1;
    } else if (nesharp<=1) {    // interior or dart vertex
        // normal case, quartic bspline surface
        // was wa=subdiv_a(ne); after refinement, wa=subdiv_a(ne)*2-1
        // _s222 : mask was n/3 --- 1 (n times); after refinement, .25
        wa = _s222 ? .25f : subdiv_a(ne)*2-1;
        if (_weighta && ne!=6) wa = _weighta*2-1;
    } else if (nesharp==2) {    // on crease
        crease_averaging_mask(v, comb);
        return;
    } else if (nesharp>=3) {    // corner vertex held constant
        return;
    } else assertnever("");
    float wc = (1-wa)/ne;
    if (wa==1) return;
    if (wa) comb.c[v] = wa;
    if (wc) for (Vertex vv : _m.vertices(v)) { comb.c[vv] = wc; }
}

// Subdivision algorithm:
// - if dart-ord, dart-dart, dart-ecv, or dart-corner -> smooth mask
// - if ecv-ord or corner-ord -> special 3-5 mask
// - all other cases
//     (ord-ord, ecv-ecv, corner-corner, ecv-corner) -> crease mask
void SubMesh::crease_averaging_mask(Vertex v, Combvh& comb) const {
    assertx(!_isquad);
    int adj_dart_vertices = 0, adj_corner_vertices = 0, adj_ec_vertices = 0, svi = 0;
    PArray<Vertex,10> va;
    for (Edge e : _m.edges(v)) {
        if (!sharp(e)) continue;
        Vertex vo = _m.opp_vertex(v, e);
        va.push(vo);
        int nesharp = num_sharp_edges(vo);
        if (nesharp==0) {
            assertnever("");    // even if _selrefine, should not occur.
        } else if (nesharp==1) {
            adj_dart_vertices++;
        } else if (nesharp>=3) {
            adj_corner_vertices++; svi = va.num()-1;
        } else if (extraordinary_crease_vertex(vo)) {
            adj_ec_vertices++; svi = va.num()-1;
        }
    }
    int adj_special = adj_dart_vertices+adj_corner_vertices+adj_ec_vertices;
    assertx(adj_special<=2);
    if (adj_dart_vertices>=1) {
        // adjacent to single dart vertex, do smooth mask
        int ne = nume(v);
        float a = _s222 ? .25f : subdiv_a(ne)*2.f-1.f;
        float wa = a, wc = (1.f-wa)/ne;
        comb.c[v] = wa;
        for (Vertex vv : _m.vertices(v)) { comb.c[vv] = wc; }
        ASSERTX(comb.is_combination());
        return;
    }
    static const bool no_special_crease = getenv_bool("SUBMESH_NO_SPECIAL_CREASE");
    if (adj_special!=1 || no_special_crease) {
        // (1, 2, 1) mask gives cubic spline
        comb.c[v] = .5f; comb.c[va[0]] = .25f; comb.c[va[1]] = .25f;
        return;
    }
    if (adj_ec_vertices==1 || adj_corner_vertices==1) {
        // adjacent to single extraordinary crease vertex or corner
        comb.c[v] = .75f; comb.c[va[1-svi]] = .25f;
        return;
    }
    assertnever("");
}

bool SubMesh::extraordinary_crease_vertex(Vertex v) const {
    assertx(!_isquad); assertx(num_sharp_edges(v)==2);
    if (_m.is_boundary(v)) return nume(v)!=4;
    if (nume(v)!=6) return true;
    int nside = 0, sharpef = 0;
    for (Vertex vv : _m.ccw_vertices(v)) {
        if (sharpef==1) nside++;
        if (sharp(_m.edge(v, vv))) sharpef++;
    }
    assertx(sharpef==2);
    return nside!=3;
}

void SubMesh::limit_mask(Vertex v, Combvh& comb) const {
    int ne = nume(v), nesharp = num_sharp_edges(v);
    if (_isquad) {
        assertx(!nesharp); assertx(ne==2 || ne==4);
        ASSERTX(ne==_m.degree(v));
        // (1, 4, 1) x (1, 4, 1)
        comb.c[v] = 16.f/36.f;
        for_int(nweight, 1+(ne==2)) {
            for (Vertex vv : _m.vertices(v)) {
                comb.c[vv] += 4.f/36.f;
                Vertex vo = assertx(_m.clw_vertex(vv, v));
                comb.c[vo] += 1.f/36.f;
            }
        }
        ASSERTX(comb.is_combination());
        return;
    }
    if (_m.flags(v).flag(GMesh::vflag_cusp)) { // cusp
        if (nesharp>=2) return;     // becomes corner vertex, held constant
        int nuse = ne;
        if (!assertw(nuse<k_table_cone_limit.num())) nuse = k_table_cone_limit.num()-1;
        float wa = k_table_cone_limit[nuse];
        assertw(!_weighta);
        float wc = (1-wa)/ne;
        comb.c[v] = wa;
        for (Vertex vv : _m.vertices(v)) { comb.c[vv] = wc; }
        return;
    }
    if (nesharp>=3) return;     // corner vertex held constant
    if (nesharp<=1) {           // interior or dart vertex
        if (_weighta && ne!=6) { Warning("to do?"); return; }
        // was wa = 3/(3+8*a1); wc = 8*a1/(3+8*a1)/ne; wb = wc;
        // since there is no refinement here, should be the same
        // _s222 : mask is n --- 1 (n times)
        float a = _s222 ? .5f : subdiv_a(ne);
        float wa = 3/(11-8*a), wc = (1-wa)/ne;
        if (wa) comb.c[v] = wa;
        if (wc) for (Vertex vv : _m.vertices(v)) { comb.c[vv] = wc; }
    } else if (nesharp==2) {    // bspline curve
        // for cubic, 1-4-1 mask
        float wa = 4.f/6.f, wb = 1.f/6.f, wc = 0.f;
        if (extraordinary_crease_vertex(v)) { wa = 3.f/5.f; wb = 1.f/5.f; }
        comb.c[v] = wa;
        for (Edge e : _m.edges(v)) {
            float w = sharp(e) ? wb : wc;
            if (w) comb.c[_m.opp_vertex(v, e)] = w;
        }
    } else assertnever("");
}

void SubMesh::triangulate_quads(Mvcvh& mconv) {
    assertx(_isquad);
    Array<Face> arf; for (Face f : _m.ordered_faces()) { arf.push(f); }
    for (Edge e : _m.edges()) { assertx(!_m.flags(e)); }
    if (0) {
        for (Vertex v : _m.vertices()) {
            Combvh comb;
            comb.c[v] = 1.f;
            mconv.enter(v, std::move(comb));
        }
    }
    for (Face f : arf) {        // was ForStack which went in reverse order
        Face forig = _mforigf.remove(f);
        _mfindex.remove(f);     // can be index 0
        Vertex v = _m.center_split_face(f); f = nullptr;
        vinfo(v).nume = 4; vinfo(v).numsharpe = 0;
        {
            Combvh comb;
            for (Vertex vv : _m.vertices(v)) { comb.c[vv] = .25f; }
            mconv.enter(v, std::move(comb));
        }
        Array<Face>& ar = _mofif.get(forig);
        for (Face fn : _m.faces(v)) {
            _mforigf.enter(fn, forig);
            _mfindex.enter(fn, ar.num());
            ar.push(fn);
        }
    }
    // Sharp edges are lost.
}

// *** misc

void SubMesh::convolve_self(const Mvcvh& mconv) {
    _cmvcvh.compose(mconv);
}

const Combvh& SubMesh::combination(Vertex v) const {
    return _cmvcvh.get(v);
}

Combvh SubMesh::compose_c_mvcvh(const Combvh& ci) const {
    return _cmvcvh.compose_c(ci);
}

void SubMesh::update_vertex_position(Vertex v) {
    Combvh& comb = _cmvcvh.get(v);
    _m.set_point(v, comb.evaluate(_omesh));
}

void SubMesh::update_vertex_positions() {
    for (Vertex v : _m.vertices()) { update_vertex_position(v); }
}

Face SubMesh::orig_face(Face f) const {
    return _mforigf.get(f);
}

void SubMesh::orig_face_index(Face fi, Face& of, int& pindex) const {
    of = _mforigf.get(fi);
    pindex = _mfindex.get(fi);
}

Face SubMesh::get_face(Face of, int index) const {
    const Array<Face>& ar = _mofif.get(of);
    return ar[index];
}

// *** debug

void SubMesh::show_mvcvh(const Mvcvh& mvcvh) const {
    showf("Mvcvh = {\n");
    for_map_key_value(mvcvh, [&](Vertex v, const Combvh& comb) {
        showf(" vertex %d {\n", _m.vertex_id(v));
        showf("  h[3]=%g\n", comb.h[3]);
        for_combination(comb.c, [&](Vertex vv, float val) {
            showf("  v%-4d  %g\n", _m.vertex_id(vv), val);
        });
        showf(" }\n");
    });
    showf("}\n");
}

void SubMesh::show_cmvcvh() const {
    show_mvcvh(_cmvcvh);
}

// *** helper

bool SubMesh::sharp(Edge e) const {
    return _m.is_boundary(e) || _m.flags(e).flag(GMesh::eflag_sharp);
}

int SubMesh::nume(Vertex v) const {
    return vinfo(v).nume;
}

int SubMesh::num_sharp_edges(Vertex v) const {
    return vinfo(v).numsharpe;
}

Edge SubMesh::opp_sharp_edge(Vertex v, Edge ee) const {
    Edge er = nullptr;
    for (Edge e : _m.edges(v)) {
        if (!sharp(e)) continue;
        if (e==ee) { ee = nullptr; } else { assertx(!er); er = e; }
    }
    assertx(!ee && er);
    return er;
}

Vertex SubMesh::opp_sharp_vertex(Vertex v, Vertex v2) const {
    return _m.opp_vertex(v, opp_sharp_edge(v, _m.edge(v, v2)));
}

} // namespace hh
