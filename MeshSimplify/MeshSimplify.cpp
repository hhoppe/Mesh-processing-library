// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <charconv>  // to_chars(), chars_format

#include "MeshSimplify/BQem.h"
#include "libHh/A3dStream.h"  // A3dColor
#include "libHh/Args.h"
#include "libHh/Array.h"
#include "libHh/Bbox.h"
#include "libHh/BinaryIO.h"
#include "libHh/BinarySearch.h"
#include "libHh/BoundingSphere.h"
#include "libHh/ConsoleProgress.h"
#include "libHh/Facedistance.h"  // project_point_triangle2(), project_point_seg2()
#include "libHh/FileIO.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"  // dihedral_angle_cos()
#include "libHh/LinearFunc.h"
#include "libHh/Map.h"
#include "libHh/MathOp.h"
#include "libHh/Matrix.h"
#include "libHh/MeshOp.h"  // Vnors
#include "libHh/Parallel.h"
#include "libHh/Polygon.h"
#include "libHh/Pqueue.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/SGrid.h"
#include "libHh/Set.h"
#include "libHh/Timer.h"
using namespace hh;

#if defined(HH_HAVE_SIMPLEX)
#include "recipes.h"
#endif

namespace {

// If disabled, use "-minqem" instead of point sampling.  (usually defined)
#define ENABLE_FACEPTS

// If disabled, avoid meshes with color to avoid fptcolor.  (usually not defined) (but needed for demos mandrill)
#define ENABLE_FPTCOLOR

// If disabled, use "-minqem" or "-norfac 0" to avoid ftpnor.  (usually defined)
#define ENABLE_FPTNOR

// If disabled, use "-minqem" or "-neptfac 0" to avoid edge points.  (usually defined)
#define ENABLE_EDGEPTS

// If disabled, do not use "-qemcache" (cache quadric error metric on faces).  (usually defined)
#define ENABLE_QEMCACHE

// If disabled, uses single-precision QEM floats. (usually defined)
#define QEM_DOUBLE

//  (usually undefined)
// #define ENABLE_TVC

// ***

#define SSTATV2(Svar, v)                             \
  do {                                               \
    static Stat HH_ID(Svar)(#Svar, verb >= 2, true); \
    if (verb >= 2) HH_ID(Svar).enter(v);             \
  } while (false)

// *** MISC

// Analysis of memory requirement:
//
//  #fpts == 3* #vertices (if random face sampling + vertices)
//  #epts == ?1.5 * #vertices (based on cessna etc...)
//
//  GMesh: 360 bytes/vertex
//
//  Sacs:
//   f_setpts: (4+5+3.0/2*2)*4 *2f/v == 96 bytes/vertex
//   e_setpts: (4+0+1.5/3*2)*4 *3e/v == 60 bytes/vertex
//   f_matid:  (1)*4           *2f/v == 8 bytes/vertex
//   c_wedge_id: (1)*4         *6c/v == 24 bytes/vertex
//
//  Array<WedgeInfo> : 8*4     *1w/v == 32 bytes/vertex
//
//  pqecost: 5*4              *3e/v == 60 bytes/vertex
//
//  fptinfo: (13)*4          *3.0p/v == 156 bytes/vertex
//  eptinfo: (5)*4           *1.5p/v == 30 bytes/vertex
//
//  Total so far: 826 bytes/vertex
//
//  ?qem: (10)*4               *1q/v ==   40 bytes/vertex
//
// Practical result: simplification of gcanyon_sq513:
//  (with no random face sampling and no epts)
//  get size=14108 rss=12000  (tested using su -c "squeeze -m 15")
//  uses about 192 MiB  -> 765 bytes/vertex

/* HPIX
// set o=fandisk.m r=fandisk.test; hpix MeshSimplify $o -numpts 10000 \
// -nf 4000 -simp -outm $r.nf4000.m.gz -nf 2000 -simp -outm $r.nf2000.m.gz \
// -nf 1000 -simp -outm $r.nf1000.m.gz -nf 400 -simp -outm $r.nf400.m.gz \
// -nf 200 -simp -outm $r.nf200.m.gz -nf 100 -simp -outm $r.nf100.m.gz \
// -nooutput > $r.simp.ou
*/
// Date: Mon Feb 12 15:30:24 PST 1996
//     cycles %cycles  cumu%    instrs  cycles   calls     cycles procedure
//                                      /inst               /call
// 11137545181  30.1%  30.1% 6055440361    1.8    51659     215597 project_fpts(const NewMeshNei&,const Point&,Param&)
// 6027550578  16.3%  46.4% 2583226130    2.3 15171951        397 project_point_triangle2(const Point&,const Point&,const Point&,const Point&,Bary&,Point&)
// 2344203287   6.3%  52.7% 1195859417    2.0  9109779        257 projecth(const Point&,const Point&,const Point&,const Point&,Bary&,Point&,const Point&,float,float,float)
// 1550061548   4.2%  56.9%  946647852    1.6    51659      30006 fit_geom(const NewMeshNei&,const Param&,float,Point&)
// 1517734548   4.1%  61.0%  914471669    1.7    77801      19508 gather_nn(MEdge*,NewMeshNei&)
// 1085097886   2.9%  63.9%  598116639    1.8     6423     168939 reproject_locally(const NewMeshNei&)
// 1028811173   2.8%  66.7%  483863549    2.1  2780370        370 dihedral_angle_cos(const Point&,const Point&,const Point&,const Point&)
//  844822024   2.3%  69.0%  258140104    3.3  5866823        144 __sqrt
//  632873419   1.7%  70.7%  231134422    2.7   402354       1573 BHPqueue::adjust(int,int,int)
//  593372951   1.6%  72.3%  295282680    2.0  2472386        240 project_point_seg2(const Point&,const Point&,const Point&,float*)
//  575498969   1.6%  73.8%  439056127    1.3  3478247        165 _lmalloc
//  567826149   1.5%  75.4%  264785199    2.1  8257721         69 Mesh::query_hedge(MVertex*,MVertex*) const
//  542184958   1.5%  76.8%  399972510    1.4  8888278         61 hedge_scalar_bnd(MVertex*,MEdge*)
//  441385895   1.2%  78.0%  227603595    1.9  3478106        127 _lfree

// HPIXPC for terrain simplification (gcanyon_sq200):
//  simplification rate: 184.38 faces / sec
//  (MeshSimplify:          429.54)
//  429633 4.3e+02s R10000 R10010 195.0MHz   1      1.0ms     2(bytes)
// samples   time(%)      cumu time(%)     procedure (dso:file)
//   29631    30s(  6.9)   30s(  6.9) BHPqueue::adjust
//   22902    23s(  5.3)   53s( 12.2) evaluate_terrain_resid
//   16214    16s(  3.8)   69s( 16.0) Mesh::query_hedge
//   15445    15s(  3.6)   84s( 19.6)       _lmalloc
//   15353    15s(  3.6) 1e+02s( 23.2)    BMap::clear
//   11312    11s(  2.6) 1.1e+02s( 25.8) hedge_scalar_bnd
//   11171    11s(  2.6) 1.2e+02s( 28.4)        _doprnt
//   10816    11s(  2.5) 1.3e+02s( 30.9)       optimize
//    9912   9.9s(  2.3) 1.4e+02s( 33.2)         _times
//    9715   9.7s(  2.3) 1.5e+02s( 35.5) BHPqueue::retrieve
//    9583   9.6s(  2.2) 1.6e+02s( 37.7)      gather_nn
//    7788   7.8s(  1.8) 1.7e+02s( 39.5)       try_ecol
//    7334   7.3s(  1.7) 1.8e+02s( 41.2) VertexEdgeIter::next
//    7326   7.3s(  1.7) 1.8e+02s( 42.9)         _lfree
//    7130   7.1s(  1.7) 1.9e+02s( 44.6)     edge_sharp
//    7129   7.1s(  1.7) 2e+02s( 46.3) Mesh::clw_corner
//    7100   7.1s(  1.7) 2.1e+02s( 47.9)   BMap::resize
//    6703   6.7s(  1.6) 2.1e+02s( 49.5)    _BSD_getime
// --> evaluate_terrain_resid should be bottleneck, so speedup of nearly 20 should be possible -> 4000 faces / sec!

// Notes:
// - springs seem somewhat useful; they cannot be removed entirely.
//    (e.g. in key.np10000.crep1e-5.spr0.m , the ring starts folding inwards, and there are several bad folds)

// Gather a ring of vertices around vertex v.
auto gather_vertex_ring(const GMesh& mesh, Vertex v) {
  Array<Vertex> va;
  for (Vertex w = mesh.most_clw_vertex(v);;) {
    va.push(w);
    w = mesh.ccw_vertex(v, w);
    if (w == va[0]) {
      va.push(w);
      break;
    }
    if (!w) break;
  }
  return va;
}

// Return smallest dihedral value in would-be mesh neighborhood around newp.
float min_local_dihedral(const GMesh& mesh, CArrayView<Vertex> va, const Point& newp) {
  int nw = va.num();
  assertx(nw > 1);
  bool open = va[0] != va.last();
  float min_dih = 10.f;
  // Remember: !open -> va[0] is repeated at end of va!
  for_intL(i, 1, nw - open) {
    int im1 = i - 1, ip1 = i + 1;
    if (ip1 == nw) ip1 = 1;
    float dih;
    if (0) {
      dih = signed_dihedral_angle(newp, mesh.point(va[i]), mesh.point(va[ip1]), mesh.point(va[im1]));
      if (1) dih = dih < -8.f ? -2.f : std::cos(dih);
    } else {
      dih = dihedral_angle_cos(newp, mesh.point(va[i]), mesh.point(va[ip1]), mesh.point(va[im1]));
    }
    min_dih = min(min_dih, dih);
  }
  return min_dih;
}

// Return smallest dihedral value in mesh neighborhood around v.
float min_dihedral_about_vertex(const GMesh& mesh, Vertex v) {
  Array<Vertex> va = gather_vertex_ring(mesh, v);
  return min_local_dihedral(mesh, va, mesh.point(v));
}

struct fptinfo {
  Point p;
  Face cmf;
  float dist2;
#if defined(ENABLE_FPTNOR)
  Vector& ptnor() { return _ptnor; }
  const Vector& ptnor() const { return _ptnor; }
  float& nordist2() { return _nordist2; }
  const float& nordist2() const { return _nordist2; }

 private:
  Vector _ptnor;    // k_undefined if undefined
  float _nordist2;  // always 0.f if ptnor undefined
 public:
#else
  Vector& ptnor() const { assertnever(""); }
  float& nordist2() const { assertnever(""); }
#endif
#if defined(ENABLE_FPTCOLOR)
  A3dColor& ptcol() { return _ptcol; }
  const A3dColor& ptcol() const { return _ptcol; }
  float& coldist2() { return _coldist2; }
  const float& coldist2() const { return _coldist2; }

 private:
  A3dColor _ptcol;  // k_undefined if undefined
  float _coldist2;  // always 0.f if ptcol undefined
 public:
#else
  A3dColor& ptcol() const {
    assertnever_ret("");
    static A3dColor t;
    return t;
  }
  float& coldist2() const {
    assertnever_ret("");
    static float t;
    return t;
  }
#endif
};
Array<fptinfo> fpts;  // set $X$ of points sampled on faces

struct eptinfo {
  Point p;
  Edge cme;  // nullptr if retired
  float dist2;
};
Array<eptinfo> epts;  // set $X_{disc}$ sampled on sharp edges

struct struct_f_setpts {
  ~struct_f_setpts() { ASSERTX(setpts.empty()); }
  Set<fptinfo*> setpts;
};
#if defined(ENABLE_FACEPTS)
HH_SACABLE(struct_f_setpts);
HH_SAC_ALLOCATE_CD_FUNC(Mesh::MFace, struct_f_setpts, func_f_setpts);
inline Set<fptinfo*>& f_setpts(Face f) { return func_f_setpts(f).setpts; }
#else
Set<fptinfo*>& f_setpts(Face) {
  static Set<fptinfo*> t;
  return t;
}
#endif

struct struct_e_setpts {
  ~struct_e_setpts() { ASSERTX(setpts.empty()); }
  Set<eptinfo*> setpts;
};
#if defined(ENABLE_EDGEPTS)
HH_SACABLE(struct_e_setpts);
HH_SAC_ALLOCATE_CD_FUNC(Mesh::MEdge, struct_e_setpts, func_e_setpts);
inline Set<eptinfo*>& e_setpts(Edge e) { return func_e_setpts(e).setpts; }
#else
Set<eptinfo*>& e_setpts(Edge) {
  static Set<eptinfo*> t;
  return t;
}
#endif

// ***

#if defined(QEM_DOUBLE)
using L_QEM_T = double;
#else
using L_QEM_T = float;
#endif
using BQemT = BQem<L_QEM_T>;
constexpr int k_qemsmax = 9;

using upBQemT = unique_ptr<BQemT>;
#if defined(ENABLE_QEMCACHE)
HH_SACABLE(upBQemT);
HH_SAC_ALLOCATE_CD_FUNC(Mesh::MFace, upBQemT, f_qem_p);
#else
upBQemT& f_qem_p(Face) {
  assertnever_ret("");
  static upBQemT t;
  return t;
}
#endif
BQemT& f_qem(Face f) { return *f_qem_p(f); }

// ***

Array<string> material_strings;

// Encodes the discrete material identifier of each face.
// Used in determining if discrete face attributes give rise to a sharp edge.
HH_SAC_ALLOCATE_FUNC(Mesh::MFace, int, f_matid);

struct WedgeInfo {
  A3dColor col;
  Vector nor;
  UV uv;
};

Array<WedgeInfo> gwinfo;  // indexed by c_wedge_id, gwinfo[0] not used!

Array<unique_ptr<BQemT>> gwq;  // empty() if !minqem || qemlocal.  indexed by c_wedge_id

HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, int, c_wedge_id);  // wedge id's of mesh corners

inline WedgeInfo& c_winfo(Corner c) { return gwinfo[c_wedge_id(c)]; }

struct P2WedgeInfo {
  Vec2<const WedgeInfo*> wi;
};

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, int, v_desn);  // number of descendants
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, int, v_desh);  // height of descendant tree

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, BoundingSphere, v_bsphere);

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, bool, v_global);  // vertex is feature

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Point, v_sph);  // sphericalparam

// Information relating to the mesh neighborhood following a speculative edge collapse.
// Note that this information is independent of orientation of edge (v1, v2).
struct NewMeshNei : noncopyable {
  ~NewMeshNei() {
    for (BQemT* qemp : ar_wq) delete qemp;
  }
  Array<Vertex> va;                // CCW, va[0] repeated if closed (== #faces + 1)
  Array<int> ar_vdisc;             // sharp edges, indices into va[]
  Array<Vec3<Corner>> ar_corners;  // 3 corners for each face (third is center)
  //
  Array<P2WedgeInfo> ar_p2wi;  // winfo of outside ar_corners
  //
  Array<int> ar_nwid;     // for each new corner, new fake wedge id
  Array<int> ar_rwid_v1;  // for each nwid, would-be real wid on v1
  Array<int> ar_rwid_v2;  // for each nwid, would-be real wid on v2
  //
  Array<fptinfo*> ar_fpts;  // face points projecting onto neighborhood
  //
  Array<eptinfo*> ar_epts;       // edge points (excluding eptretire)
  Array<eptinfo*> ar_eptretire;  // edge points to retire
  Array<int> ar_eptv;            // for ar_epts[], index in va of sharp edge
  //
  Array<BQemT*> ar_wq;  // qem for each nwid (new'ed); not unique_ptr<BQemT> because
                        //    DQem<T>::compute_minp*() recasts arg type from BQem<T>::compute_minp*()
};

// Parameterization of face points on would-be neighborhood.
struct Param {
  Array<int> ar_mini;   // closest face, index into ar_corners
  Array<Bary> ar_bary;  // coordinates on face (using ar_corners)
};

constexpr float k_bad_dih = 1e28f;

class LHPqueue : public HPqueue<Edge> {
  using base = HPqueue<Edge>;

 public:
  void clear() {
    _tot = 0.;
    _ntot = 0;
    base::clear();
  }
  void enter(Edge e, float pri) {
    if (pri < k_bad_dih) {
      _tot += pri;
      _ntot++;
    }
    base::enter(e, pri);
  }
  Edge remove_min() {
    float opri = min_priority();
    if (opri < k_bad_dih) {
      _tot -= opri;
      --_ntot;
    }
    return base::remove_min();
  }
  void enter_unsorted(Edge e, float pri) {
    if (pri < k_bad_dih) {
      _tot += pri;
      _ntot++;
    }
    base::enter_unsorted(e, pri);
  }
  float remove(Edge e) {
    float opri = base::remove(e);
    if (opri >= 0.f && opri < k_bad_dih) {
      _tot -= opri;
      --_ntot;
    }
    return opri;
  }
  float update(Edge e, float pri) {
    float opri = base::retrieve(e);
    if (opri < 0.f) return opri;
    if (opri >= 0.f && opri < k_bad_dih) {
      _tot -= opri;
      --_ntot;
    }
    if (pri < k_bad_dih) {
      _tot += pri;
      _ntot++;
    }
    return base::update(e, pri);
  }
  double total_priority() const { return _tot; }
  int total_num() const { return _ntot; }

 private:
  double _tot{0.};
  int _ntot{0};
};

int numpts = -1;              // # random sample pts (in addition to verts)
int nfaces = 0;               // simplify to this # of faces
int nvertices = 0;            // simplify to this # of vertices
bool nooutput = false;        // do not write final mesh out
int verb = 1;                 // verbosity level
int affectpq = 2;             // after ecol, quantity of cost updates
bool miniiall = false;        // always consider all 3 optim. starting pts
bool minii1 = false;          // consider only minii == 1 starting point
bool nominii1 = false;        // consider only minii == {0, 2} starting points
bool minii2 = false;          // constrain minii == 2 in output file
bool no_simp_bnd = false;     // never involve boundary vertices
bool keepuvcorners = false;   // for geometry image corners
bool keepglobalv = false;     // keep vertices tagged global
bool sphericalparam = false;  // keep valid spherical param (using vert sph)
bool attrib_project = false;  // project vunified to update wedge attribs
int strict_sharp = 0;         // disallow disc. curve topo type changes {0, 1, 2}
float colfac = .1f;           // color rms (0..sqrt(3)) to dist/diam rms
float norfac = .02f;          // normal error (0..2) to dist/diam rms
float desdfac = 0.f;          // penalize diff max #desc to balance tree
float desnfac = 0.f;          // penalize sum #desc to balance tree
float bspherefac = 0.f;       // penalize hierarchical bounding spheres
float edgepathfac = 0.f;      // penalize edge path error
float trishapepow = 1.f;      // exponent on aspect ratio
float trishapefac = 0.f;      // penalize aspect ratio of triangles
float trishapeafac = 0.f;     // penalize aspect ratio times area
float bndfac = 0.f;           // encourage boundary edge removal
bool tvcpqa = false;          // use tvcfac*priority_queue_average_cost
float tvcfac = 0.f;           // encourage transparent vertex caching
bool tvcowid = false;         // compare tvc wedges in original mesh
float neptfac = -1.f;         // scale number of sharp edge samples
float gspring = 0.f;          // force spring constant to this setting
float gmindih = -1.f / 3.f;   // minimum dihedral angle (== tetrahedron angle)
bool rnor001 = false;         // compute residuals wrt Vector(0.f, 0.f, 1.f)
bool relerror = false;        // measure relative instead of new error (OLD)
bool maxerr = false;          // use maximum error of point samples
bool terrain = false;         // max error wrt grid
string ter_grid;              // terrain grid file
bool gridushorts = false;
Vec2<int> grid_dims{0, 0};  // dimensions of grid if original mesh is grid
float gridzscale = 1.f;
bool minqem = false;          // use quadric error metric
bool minaps = false;          // use APS
bool minrandom = false;       // random sequence
bool qemgh98 = false;         // use QEM from G&H98 paper (vs. my own)
bool qemlocal = true;         // memoryless QEM
bool qemvolume = true;        // linearly constrained to preserve volume
bool qemweight = true;        // scale QEM with face_area / edge_len^2
bool qemcache = true;         // cache qem at faces (for qemlocal); faster execution but uses more memory.
bool no_fit_geom = false;     // do not optimize geometry (e.g. mandrill)
bool fit_colors = true;       // optimize colors (otherwise, still use in metric)
bool fit_normals = false;     // optimize normals (otherwise, still use in metric)
bool jittervertices = false;  // when sampling vertex points, jitter
bool innerhull = false;       // construct inner hull instead of outer
bool hull = false;            // build progressive hull
bool minvolume = false;       // minimize volume (no point samples)
bool minarea = false;         // minimize area (no point samples)
bool minedgelength = false;   // pick shortest edge
bool minvdist = false;        // minimize dist(vunified, previous_mesh)
float mresid = 0.f;           // stop simplification if residual exceeded
int maxvalence = 32;          // maximum vertex valence as result of ecol
bool poszfacenormal = false;  // prevent edge collapses that would create face normals with negative z
bool dihallow = false;        // penalize but allow bad dihedral angles
int invertexorder = 0;        // remove vertices in reverse order (2=fix_edges)
bool wedge_materials = true;  // material boundaries imply wedge boundaries; introduced for DirectX 1996-07-25
constexpr bool use_parallelism = true;

// failed attempt at signed_dihedral_angle():
//  const float gmindih = -to_rad(109.471);

constexpr float k_jitter_bary_max = 0.5f;

constexpr float k_tol = 1e-6f;           // scalar attribute equality tolerance
constexpr float k_bad_cost = BIGFLOAT;   // illegal cost (very high)
constexpr float k_undefined = BIGFLOAT;  // undefined scalar attributes

GMesh mesh;                    // current mesh
Bbox<float, 3> gbbox;          // bbox of original mesh
float gdiam;                   // diameter of original mesh
float gcolc;                   // constant in front of color error term
float gnorc;                   // constant in front of normal error term
int g_necols;                  // number of edge collapses
unique_ptr<WFile> wfile_prog;  // PM stream output (may be nullptr)
bool have_ccolors = false;     // have color scalar attributes
constexpr bool have_cnormals = true;
float offset_cost;  // offset zero in pqe cost
const bool sdebug = getenv_bool("MESHSIMPLIFY_DEBUG");
Frame grid_frame;        // position of grid in world
float inv_grid_frame00;  // 1.f / grid_frame[0][0]
float inv_grid_frame11;  // 1.f / grid_frame[1][1]
bool has_wad2 = false;
float sqrt_neptfac;
int qems;  // size of QEM as supported in make_qem()

LHPqueue pqecost;  // conservative estimate of cost of ecol

Matrix<float> g_gridf;   // if terrain, grid of height values
Matrix<ushort> g_gridu;  // if -gridushorts

// ***

#if defined(ENABLE_TVC)
HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, int, c_tvc_owid);  // original wedge id of mesh corner
#else
int& c_tvc_owid(Corner c) {
  dummy_use(c);
  assertnever_ret("");
  static int t;
  return t;
}
#endif

constexpr int tvc_fudge = 0;  // since not real FIFO simulation
constexpr int tvc_realcachesize = 16;
constexpr int tvc_cachesize = tvc_realcachesize - tvc_fudge;
// index 0 is most recently added; empty entries have {wid = -1, v = 0}.
struct CacheEntry {
  CacheEntry() = default;
  explicit CacheEntry(Corner c) {
    assertx(c);
    wid = c_wedge_id(c);
    v = mesh.corner_vertex(c);
    owid = c_tvc_owid(c);
  }
  int wid;
  Vertex v;
  int owid;
};
Array<CacheEntry> tvc_cache{tvc_cachesize};
constexpr float tvc_max_improvement = 6.f;
constexpr int tvc_maxcare = 0;  // could be 3
int tvc_ncachemiss = 0;

// ***

// Result of a possible edge collapse.
enum EResult { R_success, R_dih, R_illegal, R_NUM };

// Forward definition
void get_tvc_cost_edir(Edge e, float& tvccost, bool& edir);

// *** Functions

// Return: two faces have same discrete attributes.
//  Currently based on equality of Face strings.
bool same_discrete(Face f1, Face f2) { return f_matid(f1) == f_matid(f2); }

// Return: exists discrete attribute discontinuity across edge e.
bool edge_discrete_bnd(Edge e) {
  ASSERTX(!mesh.is_boundary(e));
  return !same_discrete(mesh.face1(e), mesh.face2(e));
}

// Return: two corners have different scalar attributes.
bool same_scalar(Corner c1, Corner c2) { return c_wedge_id(c1) == c_wedge_id(c2); }

// Return: exists scalar attribute discontinuity at edge e near vertex v.
bool hedge_scalar_bnd(Vertex v, Edge e) {
  ASSERTX(!mesh.is_boundary(e));
  return !same_scalar(mesh.ccw_corner(v, e), mesh.clw_corner(v, e));
}

// Return: exists scalar attribute discontinuity across edge.
bool edge_scalar_bnd(Edge e) {
  ASSERTX(!mesh.is_boundary(e));
  if (0) return hedge_scalar_bnd(mesh.vertex1(e), e) || hedge_scalar_bnd(mesh.vertex2(e), e);
  Mesh::HEdge he = e->_herep, he2 = he->_sym;
  return !same_scalar(he, he2->_prev) || !same_scalar(he->_prev, he2);
}

// Return: edge is sharp.
bool edge_sharp(Edge e) { return mesh.is_boundary(e) || edge_discrete_bnd(e) || edge_scalar_bnd(e); }

// Return: number of incident sharp edges on vertex v.
int vertex_num_sharpe(Vertex v) {
  int nsharpe = 0;
  for (Edge e : mesh.edges(v))
    if (edge_sharp(e)) nsharpe++;
  return nsharpe;
}

// Return: vertex v has more than one wedge.
bool vertex_has_hedge_scalar_bnd(Vertex v) {
  for (Edge e : mesh.edges(v))
    if (mesh.is_boundary(e) || hedge_scalar_bnd(v, e)) return true;
  return false;
}

inline void get_grid_point(const Point& p, Vec2<int>& pyx, float& pz) {
  pyx[0] = int((p[1] - grid_frame.p()[1]) * inv_grid_frame11 + .5f);
  pyx[1] = int((p[0] - grid_frame.p()[0]) * inv_grid_frame00 + .5f);
  pz = p[2];
}

inline float get_grid_value(const Vec2<int>& yx) { return gridzscale * (!gridushorts ? g_gridf[yx] : g_gridu[yx]); }

// Given dihedral values before and after edge collapse, is it illegal?
bool bad_dihedral(float dihb, float diha) { return diha < gmindih && diha < dihb; }

constexpr bool use_traditional_dih = true;  // Preferable sometimes, e.g. shark.
struct Dihedral {
  float dihb{0.f};
  Array<Vector> ar_dirs;
};

Dihedral enter_dihedral(Edge e, const NewMeshNei& nn) {
  Dihedral dih;
  if (use_traditional_dih) {
    dih.dihb = min(min_dihedral_about_vertex(mesh, mesh.vertex1(e)), min_dihedral_about_vertex(mesh, mesh.vertex2(e)));
  } else {
    for_int(i, nn.ar_corners.num()) {
      dih.ar_dirs.push(cross(mesh.point(mesh.corner_vertex(nn.ar_corners[i][0])),
                             mesh.point(mesh.corner_vertex(nn.ar_corners[i][1])),
                             mesh.point(mesh.corner_vertex(nn.ar_corners[i][2]))));
    }
  }
  return dih;
}

float dihedral_penalty(const Dihedral& dih, const NewMeshNei& nn, const Point& newp) {
  bool bad = false;
  if (use_traditional_dih) {
    float diha = min_local_dihedral(mesh, nn.va, newp);
    assertx(std::isfinite(diha));
    bad = bad_dihedral(dih.dihb, diha);
  } else {
    CArrayView<Vector> dirs = dih.ar_dirs;
    assertx(dirs.num() == nn.ar_corners.num());
    for_int(i, nn.ar_corners.num()) {
      Vector dir = cross(mesh.point(mesh.corner_vertex(nn.ar_corners[i][0])),
                         mesh.point(mesh.corner_vertex(nn.ar_corners[i][1])), newp);
      if (dot(dir, dih.ar_dirs[i]) <= 0) {
        bad = true;
        break;
      }
    }
  }
  float penalty = dihallow ? k_bad_dih : BIGFLOAT;
  return bad ? penalty : 0;
}

// Given a corner, find its new wedge_id in nn.
int find_nwid(const NewMeshNei& nn, Corner c) {
  assertx(c);
  for_int(i, nn.ar_corners.num()) {
    if (nn.ar_corners[i][2] == c) return nn.ar_nwid[i];
  }
  assertnever("");
}

// Given a corner, find its new wedge_id in nn.
int retrieve_nwid(const NewMeshNei& nn, Corner c) {
  assertx(c);
  for_int(i, nn.ar_corners.num()) {
    if (nn.ar_corners[i][2] == c) return nn.ar_nwid[i];
  }
  return -1;
}

unique_ptr<BQemT> make_qem() {
  switch (qems) {
    case 3: return make_unique<DQem<L_QEM_T, 3>>();
    case 6: return make_unique<DQem<L_QEM_T, 6>>();
    case 9: return make_unique<DQem<L_QEM_T, 9>>();
    default: assertnever("");
  }
}

void create_qem_vector(const Point& po, const WedgeInfo& wi, ArrayView<float> pp) {
  int i = 0;
  for_int(c, 3) pp[i++] = po[c];
  if (colfac) {
    const float cfac = gdiam * colfac;
    for_int(c, 3) pp[i++] = wi.col[c] * cfac;
  }
  if (norfac) {
    const float nfac = gdiam * norfac;
    for_int(c, 3) pp[i++] = wi.nor[c] * nfac;
  }
  assertx(i == qems);
}

void extract_qem_vector(const float* pp, Point& po, WedgeInfo& wi) {
  const float* p = pp;
  for_int(c, 3) po[c] = *p++;
  if (colfac) {
    const float cfac = gdiam * colfac;
    for_int(c, 3) wi.col[c] = *p++ / cfac;
  }
  if (norfac) {
    const float nfac = gdiam * norfac;
    for_int(c, 3) wi.nor[c] = *p++ / nfac;
  }
  assertx(p - pp == qems);
}

void corner_qem_vector(Corner c, ArrayView<float> pp) {
  create_qem_vector(mesh.point(mesh.corner_vertex(c)), c_winfo(c), pp);
}

// Without frac_diam, using qemweight, the rssa numbers can become so small that translating them by
// offset_cost makes them lose all precision.
constexpr float frac_diam = 1e-3f;

void get_face_qem(Face f, BQemT& qem) {
  Vec3<Vertex> va = mesh.triangle_vertices(f);
  SGrid<float, 3, k_qemsmax> pa;
  for_int(i, 3) corner_qem_vector(mesh.corner(va[i], f), pa[i]);
  if (qemgh98) {
    qem.set_distance_gh98(pa[0].data(), pa[1].data(), pa[2].data());
  } else {
    qem.set_distance_hh99(pa[0].data(), pa[1].data(), pa[2].data());
  }
  if (qemweight) {
    float facearea = mag(cross(mesh.point(va[0]), mesh.point(va[1]), mesh.point(va[2])));
    // Remember that QEM itself is squared on scale.  If weigh faces by area, then cost is O(scale^4).
    facearea *= 1.f / square(frac_diam * gdiam);
    qem.scale(facearea);
  }
}

void get_sharp_edge_qem(Edge e, BQemT& qem) {
  Vector nor{};  // average normal of adjacent 1 or 2 faces.
  for (Face f : mesh.faces(e)) {
    Vec3<Vertex> va = mesh.triangle_vertices(f);
    Vector fnor = cross(mesh.point(va[0]), mesh.point(va[1]), mesh.point(va[2]));
    assertw(fnor.normalize());
    nor += fnor;
  }
  {
    Vector evec = (mesh.point(mesh.vertex2(e)) - mesh.point(mesh.vertex1(e)));
    nor = cross(nor, evec);
  }
  assertw(nor.normalize());  // is_zero(nor) is OK below.
  // Length squared does not seem like the right thing to do here.  Instead, should perhaps normalize geometry to
  // unit cube and use simply edge length.
  // Note: scaling nor by length is equivalent to scaling qem by length^2.
  if (qemweight) nor *= (mesh.length(e) / (frac_diam * gdiam));
  // Weight qem on sharp edges by a factor of neptfac.
  nor *= sqrt_neptfac;
  float d = -pvdot(mesh.point(mesh.vertex1(e)), nor);
  Vec<float, k_qemsmax> enor;
  for_int(i, 3) enor[i] = nor[i];
  for_intL(i, 3, qems) enor[i] = 0.f;
  qem.set_d2_from_plane(enor.data(), d);
}

void init_qem() {
  HH_TIMER("_init_qem");
  assertx(minqem);
  if (qemlocal) {
    if (qemcache) {
      parallel_for_each(Array<Face>{mesh.faces()}, [&](Face f) {
        f_qem_p(f) = make_qem();
        get_face_qem(f, f_qem(f));
      });
    }
  } else {
    assertx(!gwq.num());
    gwq.init(gwinfo.num());
    for_int(i, gwq.num()) {
      gwq[i] = make_qem();
      gwq[i]->set_zero();
    }
    {
      auto up_qem = make_qem();
      BQemT& qem = *up_qem;
      for (Face f : mesh.faces()) {
        get_face_qem(f, qem);
        for (Corner c : mesh.corners(f)) gwq[c_wedge_id(c)]->add(qem);
      }
    }
    // Add perpendicular constraints along sharp edges
    if (neptfac) {
      if (sizeof(L_QEM_T) == sizeof(float)) assertw(neptfac <= 30.f);
      auto up_qem = make_qem();
      BQemT& qem = *up_qem;
      for (Edge e : mesh.edges()) {
        if (!edge_sharp(e)) continue;
        get_sharp_edge_qem(e, qem);
        if (!mesh.is_boundary(e)) qem.scale(0.5f);  // since now added to wedges on both sides
        for (Vertex v : mesh.vertices(e)) {
          for (Face f : mesh.faces(e)) {
            Corner c = mesh.corner(v, f);
            gwq[c_wedge_id(c)]->add(qem);
          }
        }
      }
    }
    if (1) {  // Verify QEM's are initially zero
      HH_TIMER("_qem_verify0");
      for (Vertex v : mesh.vertices()) {
        for (Corner c : mesh.corners(v)) {
          Vec<float, k_qemsmax> p;
          corner_qem_vector(c, p);
          float qv = gwq[c_wedge_id(c)]->evaluate(p.data());
          SSTATV2(Sinitqvc, qv);
        }
      }
    }
  }
}

void gather_nn_qem(Edge e, NewMeshNei& nn) {
  assertx(minqem);
  Vertex v2 = mesh.vertex2(e);
  int nw = nn.ar_rwid_v1.num();
  assertx(nw);
  nn.ar_wq.init(nw);
  for_int(i, nw) {
    nn.ar_wq[i] = make_qem().release();  // (Qem not zero'ed) (deleted in ~NewMeshNei())
  }
  if (qemlocal) {
    for_int(i, nw) nn.ar_wq[i]->set_zero();
    auto up_ql = make_qem();
    BQemT& ql = *up_ql;
    // First consider all faces besides f1 & f2
    for_int(fi, nn.ar_corners.num()) {
      const int nwid = nn.ar_nwid[fi];
      Corner c = nn.ar_corners[fi][2];
      Face f = mesh.corner_face(c);
      if (qemcache) {
        nn.ar_wq[nwid]->add(f_qem(f));
      } else {
        get_face_qem(f, ql);
        nn.ar_wq[nwid]->add(ql);
      }
    }
    // Now consider f1 & f2.  Try all neighboring corners with same wedge id to see if any survive.
    for (Face f : mesh.faces(e)) {
      const int nwid = [&] {
        for (Vertex v : mesh.vertices(e)) {
          Corner ci = mesh.corner(v, f);
          assertx(retrieve_nwid(nn, ci) < 0);
          int wid = c_wedge_id(ci);
          for (Corner c = ci;;) {  // try ccw
            c = mesh.ccw_corner(c);
            assertx(c != ci);
            if (!c || c_wedge_id(c) != wid) break;
            int nwid2 = retrieve_nwid(nn, c);
            if (nwid2 >= 0) return nwid2;
          }
          for (Corner c = ci;;) {  // try clw
            c = mesh.clw_corner(c);
            assertx(c != ci);
            if (!c || c_wedge_id(c) != wid) break;
            int nwid2 = retrieve_nwid(nn, c);
            if (nwid2 >= 0) return nwid2;
          }
        }
        return -1;
      }();
      if (nwid < 0) {
        if (verb >= 2) Warning("Qem: lose a wedge");
        continue;
      }
      if (qemcache) {
        nn.ar_wq[nwid]->add(f_qem(f));
      } else {
        get_face_qem(f, ql);
        nn.ar_wq[nwid]->add(ql);
      }
    }
    if (neptfac) {
      for (Vertex v : mesh.vertices(e)) {
        for (Edge ee : mesh.edges(v)) {
          if (v == v2 && ee == e) continue;
          if (!edge_sharp(ee)) continue;
          get_sharp_edge_qem(ee, ql);
          // Assign this quadric to any one wedge; does not matter.
          nn.ar_wq[0]->add(ql);
        }
      }
    }
  } else {
    for_int(i, nw) {
      int rwid1 = nn.ar_rwid_v1[i];
      int rwid2 = nn.ar_rwid_v2[i];
      nn.ar_wq[i]->copy(*gwq[rwid1]);
      if (rwid2 != rwid1) nn.ar_wq[i]->add(*gwq[rwid2]);
    }
  }
}

// Given that would-be vertex v1 would have 2 incident sharp edges, and one of them is (v1, nn.va[vi]),
// return the index of the other sharp edge.
int other_creasevi(const NewMeshNei& nn, int vi) {
  assertx(nn.ar_vdisc.num() == 2);
  return (vi == nn.ar_vdisc[0] ? nn.ar_vdisc[1] : vi == nn.ar_vdisc[1] ? nn.ar_vdisc[0] : (assertnever(""), 0));
}

// Interpolate two arrays of floating point values, taking into account that either one could contain
// k_undefined values.
template <int n> Vec<float, n> interp_floats(const Vec<float, n>& ar1, const Vec<float, n>& ar2, int ii) {
  assertx(ii >= 0 && ii <= 2);
  if ((ar1[0] == k_undefined && ii > 0) || (ar2[0] == k_undefined && ii < 2)) {
    return ntimes<n>(k_undefined);
  } else {
    return interp(ar1, ar2, ii * .5f);
  }
}

// Interpolate two wedge scalar attributes.  ii == 0: v2, ii == 2: v1, ii == 1: midp
WedgeInfo interp_wi(const WedgeInfo& wi1, const WedgeInfo& wi2, int ii) {
  WedgeInfo wio{interp_floats(wi1.col, wi2.col, ii), interp_floats(wi1.nor, wi2.nor, ii),
                interp_floats(wi1.uv, wi2.uv, ii)};
  // Interpolated normal may be zero.  Will test for this later.
  if (wio.nor[0] != k_undefined) wio.nor.normalize();
  return wio;
}

// Are two wedge attributes different?
int compare_wi(const WedgeInfo& wi1, const WedgeInfo& wi2) {
  return compare(wi1.col, wi2.col, k_tol) || compare(wi1.nor, wi2.nor, k_tol) || compare(wi1.uv, wi2.uv, k_tol);
}

// Create vertex and corner strings representing wedge info on vertex v.
// Possibly force all info onto corners even if vertex has unique wedge.
void create_vertex_corner_strings(Vertex v, string& str, bool force_on_corners = false) {
  int num = 0, g_wid;
  dummy_init(g_wid);
  bool same_wid = true;
  for (Corner c : mesh.corners(v)) {
    int wid = c_wedge_id(c);
    if (!num++) {
      g_wid = wid;
    } else {
      if (g_wid != wid) same_wid = false;
    }
  }
  assertx(num);
  if (!force_on_corners && same_wid) {
    int wid = g_wid;
    mesh.update_string(v, "wid", csform(str, "%d", wid));
    const WedgeInfo& wi = gwinfo[wid];
    const A3dColor& col = wi.col;
    if (col[0] != k_undefined) mesh.update_string(v, "rgb", csform_vec(str, col));
    const Vector& nor = wi.nor;
    if (nor[0] != k_undefined) mesh.update_string(v, "normal", csform_vec(str, nor));
    const UV& uv = wi.uv;
    if (uv[0] != k_undefined) mesh.update_string(v, "uv", csform_vec(str, uv));
  } else {
    for (Corner c : mesh.corners(v)) {
      int wid = c_wedge_id(c);
      const WedgeInfo& wi = gwinfo[wid];
      mesh.update_string(c, "wid", csform(str, "%d", wid));
      const A3dColor& col = wi.col;
      if (col[0] != k_undefined) mesh.update_string(c, "rgb", csform_vec(str, col));
      const Vector& nor = wi.nor;
      if (nor[0] != k_undefined) mesh.update_string(c, "normal", csform_vec(str, nor));
      const UV& uv = wi.uv;
      if (uv[0] != k_undefined) mesh.update_string(c, "uv", csform_vec(str, uv));
    }
  }
}

// Clear vertex and corner strings of vertex v.
void clear_vertex_corner_strings(Vertex v) {
  mesh.set_string(v, nullptr);
  for (Corner c : mesh.corners(v)) mesh.set_string(c, nullptr);
}

void create_face_string(Face f) { mesh.set_string(f, material_strings[f_matid(f)].c_str()); }

void clear_face_string(Face f) { mesh.set_string(f, nullptr); }

// Clear all mesh strings except on faces.
void clear_mesh_strings() {
  parallel_for_each(Array<Vertex>{mesh.vertices()}, [&](Vertex v) { clear_vertex_corner_strings(v); });
  parallel_for_each(Array<Edge>{mesh.edges()}, [&](Edge e) { mesh.update_string(e, "sharp", nullptr); });
  // Encoded by f_matid and material_strings.
  parallel_for_each(Array<Face>{mesh.faces()}, [&](Face f) { clear_face_string(f); });
}

// Write current mesh.
void write_mesh(std::ostream& os) {
  string str;
  for (Vertex v : mesh.vertices()) create_vertex_corner_strings(v, str);
  for (Vertex v : mesh.vertices())
    if (v_global(v)) mesh.update_string(v, "global", "");
  for (Face f : mesh.faces()) create_face_string(f);
  if (sphericalparam) {
    for (Vertex v : mesh.vertices()) {
      const Point& sph = v_sph(v);
      assertx(is_unit(sph));
      mesh.update_string(v, "sph", csform_vec(str, sph));
    }
    if (1) {  // sanity check
      for (Face f : mesh.faces()) {
        Vec3<Vertex> va = mesh.triangle_vertices(f);
        Vec3<Point> poly;
        for_int(i, 3) poly[i] = v_sph(va[i]);
        assertx(spherical_triangle_area(poly) < TAU);
      }
    }
  }
  mesh.write(os);
  assertx(os);
  clear_mesh_strings();
}

// Parse wedge attributes at corner c, using computed normals in vnors if corner doesn't have an explicit normal.
WedgeInfo construct_wi(Corner c, const Vnors& vnors) {
  WedgeInfo wi;
  A3dColor& col = wi.col;
  fill(col, k_undefined);
  mesh.parse_corner_key_vec(c, "rgb", col);
  Vector& nor = wi.nor;
  nor = vnors.get_nor(mesh.corner_face(c));
  // Normalize normals if necessary.
  assertx(nor[0] != k_undefined);  // normals always present
  if (0 && abs(mag2(nor) - 1.f) > 1e-4f) {
    Warning("Renormalizing original normal");
    assertw(nor.normalize());
  }
  // Always renormalize normal.
  if (!nor.normalize()) {
    Warning("Normal is zero, setting arbitrarily to (1.f, 0.f, 0.f)");
    nor = Vector(1.f, 0.f, 0.f);
  }
  // disabled next line 2001-08-23
  if (0 && rnor001) nor = Vector(0.f, 0.f, 1.f);
  UV& uv = wi.uv;
  fill(uv, k_undefined);
  mesh.parse_corner_key_vec(c, "uv", uv);
  return wi;
}

void parse_mesh_material_identifiers() {
  // If matid keys present in input file, use them, else add new ones after the maximum found.
  // This is useful if the output of simplification is re-simplified.
  Array<Face> ar_faces{mesh.faces()};
  Set<string> unique_strings;
  {
    const int num_threads = get_max_threads();
    Array<Set<string>> chunk_unique_strings(num_threads);
    parallel_for_chunk(ar_faces, num_threads, [&](const int thread_index, auto subrange) {
      Set<string>& unique_strings = chunk_unique_strings[thread_index];
      for (Face f : subrange) {
        assertx(mesh.is_triangle(f));
        if (!mesh.get_string(f)) mesh.set_string(f, "");
        unique_strings.add(mesh.get_string(f));
      }
    });
    for (Set<string>& set : chunk_unique_strings) unique_strings.merge(set);
  }
  Map<string, int> matid_of_string;
  for (const string& face_str : unique_strings) {
    string str;
    if (const char* smat = GMesh::string_key(str, face_str.c_str(), "matid")) {
      int matid = to_int(smat);
      assertx(matid >= 0);
      material_strings.access(matid);
      assertx(material_strings[matid] == "");  // No duplicate matid.
      material_strings[matid] = face_str;
      matid_of_string.enter(face_str, matid);
    }
  }
  const int nexistingmatidempty = material_strings.num() - matid_of_string.num();
  int nfirst = matid_of_string.num();
  for (const string& face_str : unique_strings) {
    if (GMesh::string_has_key(face_str.c_str(), "matid")) continue;  // Already handled above.
    const int matid = material_strings.add(1);
    string str;
    material_strings[matid] = GMesh::string_update(face_str, "matid", csform(str, "%d", matid));
    matid_of_string.enter(face_str, matid);
  }
  showdf("Materials=%d: %d with matid (%d unused), %d without matid\n",  //
         material_strings.num(), nfirst, nexistingmatidempty, material_strings.num() - nfirst);
  showff("nmaterials=%d\n", material_strings.num());
  for_int(i, material_strings.num()) showff("%s\n", material_strings[i].c_str());
  parallel_for_each(ar_faces, [&](Face f) { f_matid(f) = matid_of_string.get(mesh.get_string(f)); });
}

void parse_mesh_wedge_identifiers() {
  Array<Vertex> ar_vertices{mesh.vertices()};
  // If wid keys present in input file, use them, else add new ones after the maximum found.
  // This is useful if output of simplification is re-simplified.
  const int num_threads = get_max_threads();
  Array<int> chunk_max_vid(num_threads, 1), chunk_nwidfound(num_threads, 0), chunk_maxwidfound(num_threads, 0);
  parallel_for_chunk(ar_vertices, num_threads, [&](const int thread_index, auto subrange) {
    string str;
    int& max_vid = chunk_max_vid[thread_index];
    int& nwidfound = chunk_nwidfound[thread_index];
    int& maxwidfound = chunk_maxwidfound[thread_index];
    for (Vertex v : subrange) {
      max_vid = max(max_vid, mesh.vertex_id(v));
      for (Corner c : mesh.corners(v)) {
        const char* swid = mesh.corner_key(str, c, "wid");
        if (!swid) continue;
        nwidfound++;
        int wid = to_int(swid);
        assertx(wid > 0);
        if (wid > maxwidfound) maxwidfound = wid;
      }
    }
  });
  const int max_vid = max(chunk_max_vid), nwidfound = sum<int>(chunk_nwidfound), maxwidfound = max(chunk_maxwidfound);
  showff("Found %d/%d existing wid's\n", nwidfound, maxwidfound);
  assertx(!nwidfound || nwidfound == mesh.num_faces() * 3);
  // Initial gwinfo based on wid; more entries are added later if !nwidfound and vertices have multiple wedges.
  gwinfo.init(1 + (nwidfound ? maxwidfound : max_vid));  // Skip gwinfo[0] (wid start at 1).
  std::mutex mutex;
  Array<int> chunk_nccolors(num_threads, 0);
  parallel_for_chunk(ar_vertices, num_threads, [&](const int thread_index, auto subrange) {
    string str;
    int& nccolors = chunk_nccolors[thread_index];
    for (Vertex v : subrange) {
      // Vnors will get normals from vertex and corner strings if present.
      // Remove normals which are explicitly zero.
      for (;;) {
        Vector nor;
        if (!parse_key_vec(mesh.get_string(v), "normal", nor)) break;
        if (!is_zero(nor)) break;
        Warning("Removing explicit zero normal from vertex");
        mesh.update_string(v, "normal", nullptr);
        break;
      }
      for (Corner c : mesh.corners(v)) {
        Vector nor;
        if (!parse_key_vec(mesh.get_string(c), "normal", nor)) continue;
        if (!is_zero(nor)) continue;
        Warning("Removing explicit zero normal from corner");
        mesh.update_string(c, "normal", nullptr);
      }
      const Vnors vnors(mesh, v);  // const??
      Set<Corner> setcvis;
      for (Corner crep : mesh.corners(v)) {
        if (!setcvis.add(crep)) continue;
        const WedgeInfo wi = construct_wi(crep, vnors);
        int wid;
        if (nwidfound) {
          // Array gwinfo is never resized, so locking is unnecessary.
          wid = assertx(to_int(mesh.corner_key(str, crep, "wid")));
          gwinfo[wid] = wi;
        } else {
          std::lock_guard<std::mutex> lock(mutex);
          if (setcvis.num() == 1) {
            wid = mesh.vertex_id(v);
          } else {
            wid = gwinfo.add(1);
          }
          gwinfo[wid] = wi;
        }
        c_wedge_id(crep) = wid;
        int matid = f_matid(mesh.corner_face(crep));
        for_int(dir, 2) {  // two directions (CCW, CLW)
          Corner c = crep;
          for (;;) {
            c = dir ? mesh.clw_corner(c) : mesh.ccw_corner(c);
            if (!c || c == crep) break;
            WedgeInfo wi2 = construct_wi(c, vnors);
            bool diff = ((wedge_materials && f_matid(mesh.corner_face(c)) != matid) || compare_wi(wi, wi2));
            if (nwidfound && sdebug) {
              int wid2 = assertx(to_int(mesh.corner_key(str, c, "wid")));
              assertx(diff == (wid != wid2));
            }
            if (diff) break;
            // assertx(setcvis.add(c));
            if (!setcvis.add(c)) {
              // Very rare case when attributes are very close:
              //  illustration: 0 1 2 2 2 with k_tol == 1;
              //  first set: (0, 1), second set (2, 2, 2, error:1)
              break;
            }
            c_wedge_id(c) = wid;
          }
          if (c == crep) {
            assertx(dir == 0);
            break;
          }
        }
      }
      for (Corner c : mesh.corners(v))
        if (c_winfo(c).col[0] != k_undefined) nccolors++;
    }
  });
  const int nccolors = sum<int>(chunk_nccolors);
  if (nccolors) {
    have_ccolors = true;
    assertx(nccolors == mesh.num_faces() * 3);
  }
}

// Parse the original mesh.
void parse_mesh() {
  // Notes:
  //  Sharp edges are based on either:
  //  - mesh boundaries
  //  - discrete attribute boundaries (material differences)
  //  - scalar attribute boundaries (wedge_id differences)
  //  where:
  //  - material boundary is currently based on face strings.
  //  - scalar attributes: either normals, color, or uv differences
  assertx(mesh.num_faces());
  assertx(mesh.is_nice());
  // Encode material identifiers (f_matid(f))
  parse_mesh_material_identifiers();
  // Encode wedge id's (c_wedge_id(c))
  parse_mesh_wedge_identifiers();
  // Clear edge flags now that normals have been computed.
  for (Edge e : mesh.edges()) mesh.flags(e) = 0;
  for (Vertex v : mesh.vertices()) {
    v_global(v) = false;
    if (GMesh::string_has_key(mesh.get_string(v), "global")) v_global(v) = true;
  }
  if (sphericalparam) {
    for (Vertex v : mesh.vertices()) {
      Point& sph = v_sph(v);
      assertx(parse_key_vec(mesh.get_string(v), "sph", sph));
      assertx(is_unit(sph));
    }
    for (Face f : mesh.faces()) {
      Vec3<Vertex> va = mesh.triangle_vertices(f);
      Vec3<Point> poly;
      for_int(i, 3) poly[i] = v_sph(va[i]);
      assertx(spherical_triangle_area(poly) < TAU);
    }
  }
  // Clear out strings.
  clear_mesh_strings();
  if (sdebug) {
    for (Vertex v : mesh.vertices()) {
      for (Corner c : mesh.corners(v)) {
        int wid = c_wedge_id(c);
        assertx(gwinfo.ok(wid));
      }
    }
  }
  // Initialize descendant information.
  for (Vertex v : mesh.vertices()) {
    v_desn(v) = 1;  // each vertex has one descendant (itself)
    v_desh(v) = 1;  // height of tree
  }
  // Construct bounding spheres
  if (bspherefac) {
    for (Vertex v : mesh.vertices()) {
      const Bbox bbox{transform(concatenate(V(v), mesh.vertices(v)), [&](Vertex vv) { return mesh.point(vv); })};
      const Point point = interp(bbox[0], bbox[1]);
      float max_d2 = dist2(mesh.point(v), point);
      for (Vertex vv : mesh.vertices(v)) {
        const float d2 = dist2(mesh.point(vv), point);
        if (d2 > max_d2) max_d2 = d2;
      }
      v_bsphere(v).point = point;
      v_bsphere(v).radius = sqrt(max_d2);
    }
  }
}

// Begin recording ecol records onto PM stream.
void do_progressive(Args& args) {
  string filename = args.get_filename();
  if (wfile_prog) {
    Warning("Second '-prog' ignored");
  } else {
    if (filename != "") wfile_prog = make_unique<WFile>(filename);
  }
}

void do_vsgeom() {
  no_fit_geom = true;
  minii2 = true;
}

void do_gridx(Args& args) { grid_dims[1] = args.get_int(); }

void do_gridy(Args& args) { grid_dims[0] = args.get_int(); }

// Update the face onto which a face point projects.
void point_change_face(fptinfo* pfpt, Face newf) {
  Face oldf = pfpt->cmf;
  if (newf == oldf) return;
  if (oldf) assertx(f_setpts(oldf).remove(pfpt));
  pfpt->cmf = newf;
  if (newf) f_setpts(newf).enter(pfpt);
}

// Update the edge onto which an edge point projects.
void point_change_edge(eptinfo* pept, Edge newe) {
  Edge olde = pept->cme;
  if (newe == olde) return;
  if (olde) assertx(e_setpts(olde).remove(pept));
  pept->cme = newe;
  if (newe) e_setpts(newe).enter(pept);
}

// Sample a face point of face f, possibly sampling color.
// Must later do point_change_face()!
void add_face_point(Face f, Bary bary, bool define_scalars) {
  fpts.add(1);
  fptinfo& fpt = fpts.last();
  fpt.cmf = f;
  Polygon poly;
  mesh.polygon(f, poly);
  fpt.p = interp(poly[0], poly[1], poly[2], bary);
  fpt.dist2 = 0.f;
  if (have_ccolors) {
    if (define_scalars) {
      Vec3<Corner> ca = mesh.triangle_corners(f);
      fpt.ptcol() = interp(c_winfo(ca[0]).col, c_winfo(ca[1]).col, c_winfo(ca[2]).col, bary);
    } else {
      fpt.ptcol() = A3dColor(k_undefined, k_undefined, k_undefined);
    }
    fpt.coldist2() = 0.f;
  }
  if (have_cnormals && norfac) {
    if (define_scalars) {
      Vec3<Corner> ca = mesh.triangle_corners(f);
      fpt.ptnor() = interp(c_winfo(ca[0]).nor, c_winfo(ca[1]).nor, c_winfo(ca[2]).nor, bary);
      assertw(fpt.ptnor().normalize());
    } else {
      fpt.ptnor() = Vector(k_undefined, k_undefined, k_undefined);
    }
    fpt.nordist2() = 0.f;
  }
}

// Sample an edge point on edge e.  Must later do point_change_edge()!
void add_edge_point(Edge e, float bary) {
  epts.add(1);
  eptinfo& ept = epts.last();
  ept.cme = e;
  ept.p = interp(mesh.point(mesh.vertex1(e)), mesh.point(mesh.vertex2(e)), bary);
  ept.dist2 = 0.f;
}

// Write out the complexity of the current mesh and statistics on fit errors.
void analyze_mesh(const char* s) {
  int nv = mesh.num_vertices(), nf = mesh.num_faces(), ne = mesh.num_edges();
  std::atomic<int> nshae = 0, nbnde = 0, ndise = 0, nscae = 0;
  parallel_for_each(Array<Edge>{mesh.edges()}, [&](Edge e) {
    if (edge_sharp(e)) nshae++;
    if (mesh.is_boundary(e)) {
      nbnde++;
    } else {
      if (edge_discrete_bnd(e)) ndise++;
      if (edge_scalar_bnd(e)) nscae++;
    }
  });
  showff("%-12s: v=%d f=%d e=%d (sha=%d: bnd=%d dis=%d sca=%d)\n",  //
         s, nv, nf, ne, int(nshae), int(nbnde), int(ndise), int(nscae));
  if (minqem || minaps) return;
  {
    float efdis = 0.f;
    for (const fptinfo& fpt : fpts) efdis += fpt.dist2;
    float fdrms = my_sqrt(efdis / max(fpts.num(), 1));
    float dmax2 = 0.f;
    for (const fptinfo& fpt : fpts) dmax2 = max(dmax2, fpt.dist2);
    float fdmax = my_sqrt(dmax2);
    showff(" fdist(%d): rms=%g (%.3f%%)  max=%g (%.3f%% of bbox)\n",  //
           fpts.num(), fdrms, fdrms / gdiam * 100, fdmax, fdmax / gdiam * 100);
  }
  {
    float eedis = 0.f;
    for (const eptinfo& ept : epts) eedis += ept.dist2;
    float edrms = my_sqrt(eedis / max(epts.num(), 1));
    float dmax2 = 0.f;
    for (const eptinfo& ept : epts) dmax2 = max(dmax2, ept.dist2);
    float edmax = my_sqrt(dmax2);
    showff(" edist(%d): rms=%g (%.3f%%)  max=%g (%.3f%% of bbox)\n",  //
           epts.num(), edrms, edrms / gdiam * 100, edmax, edmax / gdiam * 100);
  }
  if (have_ccolors) {
    // Compute color errors (Euclidean distance in RGB unit cube)
    int nptcol = 0;
    float efdis = 0.f, dmax2 = 0.f;
    for (const fptinfo& fpt : fpts) {
      if (fpt.ptcol()[0] == k_undefined) continue;
      nptcol++;
      efdis += fpt.coldist2();
      dmax2 = max(dmax2, fpt.coldist2());
    }
    float fdrms = my_sqrt(efdis / max(nptcol, 1));
    float fdmax = my_sqrt(dmax2);
    showff(" fcoldist(%d): rms=%g max=%g\n", nptcol, fdrms, fdmax);
  }
  if (have_cnormals && norfac) {
    // Compute normal errors (Euclidean distance)
    int nptnor = 0;
    float efdis = 0.f, dmax2 = 0.f;
    for (const fptinfo& fpt : fpts) {
      if (fpt.ptnor()[0] == k_undefined) continue;
      nptnor++;
      efdis += fpt.nordist2();
      dmax2 = max(dmax2, fpt.nordist2());
    }
    float fdrms = my_sqrt(efdis / max(nptnor, 1));
    float fdmax = my_sqrt(dmax2);
    showff(" fnordist(%d): rms=%g max=%g\n", nptnor, fdrms, fdmax);
  }
}

// Sample the original mesh to obtain the set of face points and edge points.
void sample_pts() {
  HH_TIMER("_sample_pts");
  if (terrain || minarea || minvolume || minedgelength || minvdist || minqem || minaps || minrandom) {
    assertx(!fpts.num());
    assertx(!numpts);
    showff("No points sampled.\n");
    return;
  }
#if !defined(ENABLE_FACEPTS)
  assertnever("ENABLE_FACEPTS was not set during compile");
#endif
  float mesh_area;
  {
    double sum_area = 0.;
    for (Face f : mesh.faces()) sum_area += mesh.area(f);
    mesh_area = float(sum_area);
  }
  if (numpts == -1) {
    numpts = mesh.num_faces();
    showff("Setting number of random points to %d\n", numpts);
  }
  {  // reserve space to prevent over-allocation
    assertx(!fpts.num());
    fpts.init(numpts + mesh.num_vertices());
    fpts.init(0);
  }
  // Random sampling.
  if (numpts) {
    Array<Face> fface;    // Face of this index (nf)
    Array<float> fcarea;  // cumulative area (nf + 1)
    {
      double sumarea = 0.;  // for accuracy
      for (Face f : mesh.faces()) {
        float area = mesh.area(f);
        fface.push(f);
        fcarea.push(float(sumarea));
        sumarea += area;
      }
      for_int(i, fface.num()) fcarea[i] /= mesh_area;
      fcarea.push(1.00001f);
    }
    for_int(i, numpts) {
      int fi = discrete_binary_search(fcarea, 0, fface.num(), Random::G.unif());
      Face f = fface[fi];
      float a = Random::G.unif(), b = Random::G.unif();
      if (a + b > 1.f) {
        a = 1.f - a;
        b = 1.f - b;
      }
      Bary bary(a, b, 1.f - a - b);
      add_face_point(f, bary, true);
    }
    showff("Created %d random points\n", numpts);
  }
  // Vertex sampling.
  {
    Array<Vertex> va;
    Array<Face> fa;  // for jittering
    for (Vertex v : mesh.vertices()) {
      bool define_scalars = !vertex_has_hedge_scalar_bnd(v);
      Face f;
      if (jittervertices) {
        fa.init(0);
        for (Face ff : mesh.faces(v)) fa.push(ff);
        f = fa[Random::G.get_unsigned(fa.num())];
      } else {
        f = assertx(mesh.most_clw_face(v));
      }
      mesh.get_vertices(f, va);
      assertx(va.num() == 3);
      Bary bary(0.f, 0.f, 0.f);
      for_int(i, 3) {
        if (va[i] != v) continue;
        bary[i] = 1.f;
        if (jittervertices) {
          float a = Random::G.unif(), b = Random::G.unif();
          if (a + b > 1.f) {
            a = 1.f - a;
            b = 1.f - b;
          }
          a *= k_jitter_bary_max;
          b *= k_jitter_bary_max;
          bary[i] = 1.f - a - b;
          bary[mod3(i + 1)] = a;
          bary[mod3(i + 2)] = b;
        }
      }
      assertx(abs(bary[0] + bary[1] + bary[2] - 1.f) < 1e-6f);
      add_face_point(f, bary, define_scalars);
    }
    showff("Created %d points at vertices\n", mesh.num_vertices());
  }
  // Assign points to sets.
  for_int(i, fpts.num()) {
    fptinfo& fpt = fpts[i];
    Face f = fpt.cmf;
    fpt.cmf = nullptr;
    point_change_face(&fpt, f);
  }
  // Edge sampling.
  if (neptfac) {
    const bool edge_regular_sampling = true;
    const bool edge_random_sampling = false;
    float mesh_elen;
    Array<Edge> eedge;   // Edge of this index
    Array<float> eclen;  // cumulative length
    {
      double sumlen = 0.;
      for (Edge e : mesh.edges()) {
        if (!edge_sharp(e)) continue;
        float len = mesh.length(e);
        eedge.push(e);
        eclen.push(float(sumlen));
        sumlen += len;
      }
      mesh_elen = float(sumlen);
      for_int(i, eedge.num()) eclen[i] /= mesh_elen;
      eclen.push(1.00001f);
    }
    {  // for a square patch, perimeter_ratio is 4
      float perimeter_ratio = mesh_elen / (my_sqrt(mesh_area));
      showff("perimeter_ratio=%g\n", perimeter_ratio);
    }
    float sampling_density = (numpts + mesh.num_vertices()) / mesh_area;
    int np = int(my_sqrt(sampling_density) * mesh_elen * neptfac + .5f);
    showff("mesh_area=%g mesh_elen=%g enp=%d\n", mesh_area, mesh_elen, np);
    if (!np) {
      Warning("No points sampled on sharp edges");
    } else if (edge_regular_sampling) {
#if !defined(ENABLE_EDGEPTS)
      assertnever("ENABLE_EDGEPTS was not set during compile");
#endif
      // Duplicate points at vertices, but that doesn't matter.
      const float adjustment = 1.f;
      const float factor = 1.f / mesh_elen * np * adjustment;
      for (Edge e : mesh.edges()) {
        if (!edge_sharp(e)) continue;
        int level = int(mesh.length(e) * factor + 1.f);
        for_int(i, level + 1) {
          float a = i / float(level);
          add_edge_point(e, a);
        }
      }
    } else if (edge_random_sampling) {
      for_int(i, np) {
        int ei = discrete_binary_search(eclen, 0, eedge.num(), Random::G.unif());
        Edge e = eedge[ei];
        add_edge_point(e, Random::G.unif());
      }
    } else {
      assertnever("");
    }
    showff("Created %d points on sharp edges\n", epts.num());
    // Assign points to sets.
    for_int(i, epts.num()) {
      eptinfo& ept = epts[i];
      Edge e = ept.cme;
      ept.cme = nullptr;
      point_change_edge(&ept, e);
    }
  }
}

void do_tvcreinit() {
  for (Face f : mesh.faces())
    for (Corner c : mesh.corners(f)) c_tvc_owid(c) = c_wedge_id(c);
  // Note: in the current Mesh implementation, Mesh::collapse_edge() preserves attributes of Corners,
  // so c_tvc_owid(c) is propagated correctly through simplification.
  for_int(i, tvc_cache.num()) tvc_cache[i].owid = tvc_cache[i].wid;
}

// Initialize global state.
void perhaps_initialize() {
  if (gdiam) return;
  if (neptfac < 0.f) neptfac = minqem ? 1.f : 4.f;
  sqrt_neptfac = sqrt(neptfac);
  if (terrain) {
    // do_vsgeom();
    numpts = 0;
    norfac = 0.f;
    rnor001 = true;
  }
  if (minarea || minvolume || minedgelength || minvdist || minqem || minaps || minrandom) numpts = 0;
  {
    HH_TIMER("_parsemesh");
    parse_mesh();
  }
  {
    gbbox = Bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
    gdiam = assertx(gbbox.max_side());
    gdiam = getenv_float("GDIAM", gdiam, true);
    gcolc = square(gdiam * colfac);
    if (!have_ccolors) {
      colfac = 0.f;
      gcolc = 0.f;
    }
    gnorc = square(gdiam * norfac);
    if (!have_cnormals) {
      norfac = 0.f;
      gnorc = 0.f;
    }
    offset_cost = square(gdiam * 1e-5f);  // was square(gdiam*1e-2f)
    // For DEBUG, offset_cost could cause rssa/cost to lose precision.
    if (0) offset_cost = 0.f;
  }
  if (!have_ccolors || !gcolc) fit_colors = false;
  if (!have_cnormals || !gnorc) fit_normals = false;
  showff("PM: bounding box %g %g %g  %g %g %g\n",  //
         gbbox[0][0], gbbox[0][1], gbbox[0][2], gbbox[1][0], gbbox[1][1], gbbox[1][2]);
  showdf("parameters: diam=%g fitcol=%d colfac=%g fitnor=%d norfac=%g\n",  //
         gdiam, fit_colors, colfac, fit_normals, norfac);
  if ((fit_colors || fit_normals) || attrib_project) {
    has_wad2 = true;
    showff("PM: has_wad2\n");
  }
  sample_pts();
  analyze_mesh("INITIAL");
  assertx(!pqecost.num());
  bool boundaries_first;
  dummy_init(boundaries_first);
  assertx(is_zero(grid_dims) || product(grid_dims));
  for (;;) {
    if (ter_grid != "" || product(grid_dims)) break;
    int max_vid = 0;
    for (Vertex v : mesh.vertices()) max_vid = max(max_vid, mesh.vertex_id(v));
    if (max_vid != mesh.num_vertices()) break;
    Point p11 = gbbox[0];
    {
      int gridx = 1;
      for (; gridx <= mesh.num_vertices(); gridx++) {
        const Point& p = mesh.point(mesh.id_vertex(gridx));
        if (p[1] != p11[1]) break;
      }
      --gridx;
      if (!gridx) break;
      grid_dims = V(mesh.num_vertices() / gridx, gridx);
    }
    if (min(grid_dims) < 4 || product(grid_dims) != mesh.num_vertices()) {
      grid_dims = twice(0);
      break;
    }
    {
      Point pn1 = mesh.point(mesh.id_vertex(grid_dims[1]));
      Point p1n, pnn;
      if (mesh.point(mesh.id_vertex(2 * grid_dims[1] + 2))[0] == p11[0]) {
        boundaries_first = true;
        p1n = mesh.point(mesh.id_vertex(grid_dims[1] + 1));
        pnn = mesh.point(mesh.id_vertex(2 * grid_dims[1]));
      } else {
        boundaries_first = false;
        p1n = mesh.point(mesh.id_vertex((grid_dims[0] - 1) * grid_dims[1] + 1));
        pnn = mesh.point(mesh.id_vertex((grid_dims[0] - 1) * grid_dims[1] + grid_dims[1]));
      }
      if (p11[0] != p1n[0] || pn1[0] != pnn[0] || p11[1] != pn1[1] || p1n[1] != pnn[1]) {
        grid_dims = twice(0);
        break;
      }
      assertx(p11[0] == gbbox[0][0]);
      assertx(p11[1] == gbbox[0][1]);
      assertx(pn1[0] == gbbox[1][0]);
      assertx(pn1[1] == gbbox[0][1]);
      assertx(p1n[0] == gbbox[0][0]);
      assertx(p1n[1] == gbbox[1][1]);
      assertx(pnn[0] == gbbox[1][0]);
      assertx(pnn[1] == gbbox[1][1]);
    }
    showdf("Original mesh is likely grid of %dx%d\n", grid_dims[1], grid_dims[0]);
    showdf("Grid boundaries_first=%d\n", boundaries_first);
    // Assertions.
    {  // verify that diagonals run the right way
      // Look for vertex at grid[1][1].
      Vertex v = mesh.id_vertex(boundaries_first ? 2 * grid_dims[1] + 2 * (grid_dims[0] - 2) + 1 : grid_dims[1] + 2);
      assertx(mesh.edge(mesh.id_vertex(1), v));
    }
    break;
  }
  if (terrain) {
    assertx(no_fit_geom);
    assertx(minii2);
    if (ter_grid != "") {
      RFile fi(ter_grid);
      if (!gridushorts) {
        if (!product(grid_dims)) {
          float fx;
          assertx(read_binary_std(fi(), ArView(fx)));
          grid_dims[1] = int(fx);
          float fy;
          assertx(read_binary_std(fi(), ArView(fy)));
          grid_dims[0] = int(fy);
        }
        showdf("Reading terrain grid (%dx%d)\n", grid_dims[1], grid_dims[0]);
        assertx(min(grid_dims) >= 4);
        g_gridf.init(grid_dims);
        for_int(y, grid_dims[0]) assertx(read_binary_std(fi(), g_gridf[y]));
        HH_RSTAT(Sgrid, g_gridf);
      } else {
        assertx(product(grid_dims));
        showdf("Reading terrain grid (%dx%d)\n", grid_dims[1], grid_dims[0]);
        assertx(min(grid_dims) >= 4);
        g_gridu.init(grid_dims);
        static_assert(sizeof(ushort) == 2);
        for_int(y, grid_dims[0]) assertx(read_binary_std(fi(), g_gridu[y]));
      }
    } else {
      assertx(product(grid_dims));
      g_gridf.init(grid_dims);
      int i = 1;
      if (boundaries_first) {
        for_int(x, grid_dims[1]) g_gridf[0][x] = mesh.point(mesh.id_vertex(i++))[2];
        for_int(x, grid_dims[1]) g_gridf[grid_dims[0] - 1][x] = mesh.point(mesh.id_vertex(i++))[2];
        for_intL(y, 1, grid_dims[0] - 1) g_gridf[y][0] = mesh.point(mesh.id_vertex(i++))[2];
        for_intL(y, 1, grid_dims[0] - 1) g_gridf[y][grid_dims[1] - 1] = mesh.point(mesh.id_vertex(i++))[2];
        for_intL(y, 1, grid_dims[0] - 1) {
          for_intL(x, 1, grid_dims[1] - 1) g_gridf[y][x] = mesh.point(mesh.id_vertex(i++))[2];
        }
      } else {
        for_int(y, grid_dims[0]) for_int(x, grid_dims[1]) g_gridf[y][x] = mesh.point(mesh.id_vertex(i++))[2];
      }
      assertx(i == mesh.num_vertices() + 1);
    }
  }
  if (product(grid_dims) && (g_gridf.ysize() || g_gridu.ysize())) {
    grid_frame = Frame(Vector((gbbox[1][0] - gbbox[0][0]) / (grid_dims[1] - 1.f), 0.f, 0.f),
                       Vector(0.f, (gbbox[1][1] - gbbox[0][1]) / (grid_dims[0] - 1.f), 0.f), Vector(0.f, 0.f, 1.f),
                       Point(gbbox[0][0], gbbox[0][1], 0.f));
    inv_grid_frame00 = 1.f / grid_frame[0][0];
    inv_grid_frame11 = 1.f / grid_frame[1][1];
    if (1) {
      for (Vertex v : mesh.vertices()) {
        Vec2<int> npyx;
        float npz;
        get_grid_point(mesh.point(v), npyx, npz);
        float gv = get_grid_value(npyx);
        if (square(npz - gv) > square(1e-6f)) {
          if (0) showf("(%d, %d, %f) grid=%f\n", npyx[1], npyx[0], npz, gv);
          HH_SSTAT(Skinkerr, npz - gv);
          Warning("Grid floats do not agree; could be due to removekinks on cropped image");
        }
      }
    }
  }
  qems = 0;
  if (minqem) {
    qems += 3;  // geometry
    if (norfac) qems += 3;
    if (colfac) qems += 3;
    showdf("Qems=%d\n", qems);
  }
  if (minqem) init_qem();
  if (tvcfac) {
    for_int(i, tvc_cache.num()) {
      tvc_cache[i].wid = -1;
      tvc_cache[i].v = nullptr;
      tvc_cache[i].owid = -1;
    }
    do_tvcreinit();
  }
}

// End of program.
void wrap_up() {
  showff("PM: necols=%d\n", g_necols);
  analyze_mesh("FINAL");
  for (fptinfo& fpt : fpts) {
    ASSERTX(mesh.valid(fpt.cmf));
    point_change_face(&fpt, nullptr);
  }
  int nretired = 0;
  for (eptinfo& ept : epts) {
    if (!ept.cme) {
      nretired++;
      continue;
    }
    ASSERTX(mesh.valid(ept.cme));
    point_change_edge(&ept, nullptr);
  }
  showff("number of retired edge points: %d/%d\n", nretired, epts.num());
}

// Compute a local spring constant for the would-be neighborhood nn.
float compute_spring(const NewMeshNei& nn) {
  // #corners == #faces
  int nf = nn.ar_corners.num(), np;
  if (0) {
    // old way: poor: biases simplification at boundary.
    np = nn.ar_fpts.num() + nn.ar_epts.num();
  } else {
    np = nn.ar_fpts.num();
  }
  // spring as a function of #points and #faces?
  // From looking at many examples of Meshfit, had roughly #f / #p == 7--40 before spring was set below 1e-2 .
  // Let frac = np / nf
  //  spring = frac<4 ? 1e-2 : frac<8 ? 1e-4 : 1e-8;
  float spring = (np < nf * 4 ? 1e-2f : np < nf * 8 ? 1e-4f : 1e-8f);
  // Variable spring constants tends to produce patches of large faces, which gives poor behavior for selective
  // refinement.  So now 1997-08-27 we use constant springs.
  // if (1) spring = 1e-8f;
  // No, that doesn't give good results for meshes >sq100. We go back to higher spring constants.
  if (gspring != -1) spring = gspring;
  if (spring > 0.f) SSTATV2(Slogspring, log10f(spring));
  return spring;
}

// *** ULls
// Solve a linear least squares problem involving a single variable in which the problem decomposes into nd
//  independent univariate LLS problems.
// The system is Ux=b,
//  the design matrix U is a column vector with m rows
//  (the m constraints coming from either the springs or the projected points).
// The unknown x is simply a scalar (actually, one scalar for each dimension).
// The vector b is also a column vector with m rows.
// To solve it using (UtU)x=Utb, note that UtU is simply the norm of U,
//  and Utb is simply the dot product of U and b.
// To do this efficiently, UtU and Utb can be accumulated for all nd coordinates simultaneously,
//  while traversing U and b row-by-row.
// To compute the rss (||Ux-b||^2), Werner observed that rss= ||b||^2-||Ux||^2 = ||b||^2 - x^2*||U||^2.
class ULls : noncopyable {
 public:
  ULls(float* sol, int nd);
  // Constraint between point pdata and the point on triangle (p1, p2, p)
  //  with barycentric coordinates (param1, param2, 1-param1-param2).
  void enter_fprojection(const Vec3<float>& pdata, const Vec3<float>& p1, const Vec3<float>& p2, float param1,
                         float param2);
  // Constraint between point pdata and the point on edge (p, p1)
  //  with barycentric coordinates (1-param1, param1).
  void enter_eprojection(const Point& pdata, const Point& p1, float param1);
  void enter_spring(const Point& pother, float sqrt_spring);
  void solve(double& rss1);  // updates point pp!
 private:
  float* _sol;
  int _nd;
  Array<double> _vUtU, _vUtb, _btb;
};

ULls::ULls(float* sol, int nd) : _sol(sol), _nd(nd), _vUtU(nd), _vUtb(nd), _btb(nd) {
  for_int(c, _nd) _vUtU[c] = _vUtb[c] = _btb[c] = 0.;
}

inline void ULls::enter_fprojection(const Vec3<float>& pdata, const Vec3<float>& p1, const Vec3<float>& p2,
                                    float param1, float param2) {
  double pa1 = param1, pa2 = param2, u = 1. - pa1 - pa2;
  for_int(c, _nd) {
    double b = pdata[c] - pa1 * p1[c] - pa2 * p2[c];
    _vUtU[c] += u * u;
    _vUtb[c] += u * b;
    _btb[c] += b * b;
  }
}

inline void ULls::enter_eprojection(const Point& pdata, const Point& p1, float param1) {
  double pa1 = param1, u = 1. - pa1;
  for_int(c, _nd) {
    double b = pdata[c] - pa1 * p1[c];
    _vUtU[c] += u * u;
    _vUtb[c] += u * b;
    _btb[c] += b * b;
  }
}

inline void ULls::enter_spring(const Point& pother, float sqrt_spring) {
  for_int(c, _nd) {
    double u = sqrt_spring;
    double b = double(pother[c]) * sqrt_spring;
    _vUtU[c] += u * u;
    _vUtb[c] += u * b;
    _btb[c] += b * b;
  }
}

void ULls::solve(double& prss1) {
  double rss1 = 0.;
  for_int(c, _nd) {
    double newv = _vUtU[c] ? _vUtb[c] / _vUtU[c] : (Warning("ULls ill-conditioned"), _sol[c]);
    _sol[c] = float(newv);
    double a = _btb[c] - _vUtU[c] * square(newv);
    assertw(a > -1e-8);
    if (a > 0) rss1 += a;
  }
  prss1 = rss1;
}

// ***

void gather_nn_1(Edge e, NewMeshNei& nn) {
  // modified gather_edge_ring()
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);  // f2 could be nullptr
  // current Mesh implementation boundary Edge direction
  if (mesh.is_boundary(e)) assertx(mesh.most_ccw_vertex(v2) == v1);
  Vertex cv = (mesh.is_boundary(e) ? v2 :  // current vertex for rotation
                   !mesh.is_boundary(v1) ? v2
               : !mesh.is_boundary(v2)   ? v1
                                         : (assertnever_ret(""), implicit_cast<Vertex>(nullptr)));
  Vertex ov = mesh.opp_vertex(cv, e);  // other vertex of rotation (v1 or v2)
  Vertex w = assertx(mesh.most_clw_vertex(cv));
  if (!mesh.is_boundary(cv)) {
    // Find starting point such that all edges ee are visited!
    while (w == ov || mesh.clw_vertex(cv, w) == ov) w = assertx(mesh.clw_vertex(cv, w));
  }
  for (;;) {
    // Add vertex, face, and corner to ring.
    nn.va.push(w);
    {
      Vertex v = w;
      Face f = mesh.ccw_face(cv, mesh.edge(cv, v));
      if (f && (f == f1 || f == f2)) f = mesh.ccw_face(ov, mesh.edge(ov, v));
      if (f) {
        assertx(f != f1 && f != f2);
        // nn.ar_faces += f;
        Corner c0 = mesh.corner(v, f);
        Corner c1 = mesh.ccw_face_corner(c0);
        Corner c2 = mesh.clw_face_corner(c0);  // the inside corner
        nn.ar_corners.push(Vec3<Corner>(c0, c1, c2));
        P2WedgeInfo p2wi;
        p2wi.wi[0] = &c_winfo(c0);
        p2wi.wi[1] = &c_winfo(c1);
        nn.ar_p2wi.push(p2wi);
      }
    }
    Vertex w2 = mesh.ccw_vertex(cv, w);
    if (w2 == ov) {
      ov = cv;
      cv = w2;
      w2 = mesh.ccw_vertex(cv, w);
    }
    w = w2;
    if (w == nn.va[0]) {
      nn.va.push(w);
      break;
    }
    if (!w) break;
  }
  if (sdebug) {
    assertx((!mesh.is_boundary(v1) && !mesh.is_boundary(v2)) == (nn.va[0] == nn.va.last()));
    assertx(nn.ar_corners.num() == nn.va.num() - 1);
    int nfexpect =
        mesh.degree(v1) - mesh.is_boundary(v1) + mesh.degree(v2) - mesh.is_boundary(v2) - 2 * (1 + (f2 != nullptr));
    if (nfexpect != nn.ar_corners.num())
      assertnever(SSHOW(mesh.degree(v1), mesh.is_boundary(v1), mesh.degree(v2), mesh.is_boundary(v2), f2 != nullptr,
                        nn.ar_corners.num()));
  }
}

bool gather_nn_valid(Edge e) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);  // f2 could be nullptr
  Vertex vs = v1, vt = v2;
  Face fl = f1;
  Corner cvsfl = mesh.corner(vs, fl), cvtfl = mesh.ccw_face_corner(cvsfl);
  Corner cvsflo = mesh.ccw_corner(cvsfl);
  Corner cvtflo = mesh.clw_corner(cvtfl);
  Corner cvsfr = !f2 ? nullptr : mesh.clw_corner(cvsfl);
  Corner cvtfr = !f2 ? nullptr : mesh.ccw_corner(cvtfl);
  Corner cvsfro = !f2 ? nullptr : mesh.clw_corner(cvsfr);
  Corner cvtfro = !f2 ? nullptr : mesh.ccw_corner(cvtfr);
  bool thru_sl = cvsflo && c_wedge_id(cvsfl) == c_wedge_id(cvsflo);
  bool thru_sr = cvsfro && c_wedge_id(cvsfr) == c_wedge_id(cvsfro);
  bool thru_tl = cvtflo && c_wedge_id(cvtfl) == c_wedge_id(cvtflo);
  bool thru_tr = cvtfro && c_wedge_id(cvtfr) == c_wedge_id(cvtfro);
  bool vs_has_wedge_bnd = vertex_has_hedge_scalar_bnd(vs);
  bool vt_has_wedge_bnd = vertex_has_hedge_scalar_bnd(vt);
  if (thru_sl && thru_sr && c_wedge_id(cvsfl) == c_wedge_id(cvsfr) && vs_has_wedge_bnd && vt_has_wedge_bnd) {
    if (verb >= 2) Warning("Edge collapse would fragment wedge around vs");
    return false;
  }
  if (thru_tl && thru_tr && c_wedge_id(cvtfl) == c_wedge_id(cvtfr) && vt_has_wedge_bnd && vs_has_wedge_bnd) {
    if (verb >= 2) Warning("Edge collapse would fragment wedge around vt");
    return false;
  }
  if ((thru_sl && thru_sr && c_wedge_id(cvsfl) == c_wedge_id(cvsfr) && c_wedge_id(cvtfl) != c_wedge_id(cvtfr)) ||
      (thru_tl && thru_tr && c_wedge_id(cvtfl) == c_wedge_id(cvtfr) && c_wedge_id(cvsfl) != c_wedge_id(cvsfr))) {
    // Problem is that there are two different attributes that
    //  resulting wedge could take on -> problem in encoding.
    if (verb >= 2) Warning("Edge collapse would squash dart");
    return false;
  }
  // These are older more specific checks.
  if (thru_sl && thru_sr && c_wedge_id(cvsfl) != c_wedge_id(cvsfr) && thru_tl && thru_tr &&
      c_wedge_id(cvtfl) == c_wedge_id(cvtfr)) {
    if (verb >= 2) Warning("Edge collapse would squash sharp half-edge at vs");
    assertnever("");
  }
  if (thru_tl && thru_tr && c_wedge_id(cvtfl) != c_wedge_id(cvtfr) && thru_sl && thru_sr &&
      c_wedge_id(cvsfl) == c_wedge_id(cvsfr)) {
    if (verb >= 2) Warning("Edge collapse would squash sharp half-edge at vt");
    assertnever("");
  }
  return true;
}

bool gather_nn_2(Edge e, NewMeshNei& nn) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  int nf = nn.ar_corners.num();
  bool closed = nn.va[0] == nn.va.last();
  assertx(!nn.ar_nwid.num());
  for_int(i, nf) nn.ar_nwid.push(-1);
  for_int(irep, nf) {
    if (nn.ar_nwid[irep] >= 0) continue;
    Corner crep = nn.ar_corners[irep][2];
    int wid = nn.ar_rwid_v1.num();
    nn.ar_nwid[irep] = wid;
    int rwid_v1 = c_wedge_id(crep);
    int rwid_v2 = c_wedge_id(crep);
    for_int(dir, 2) {  // two directions (CCW, CLW)
      int i = irep;
      Corner pc = crep;  // previous corner
      for (;;) {
        if (!dir) {
          if (++i >= nf) {
            if (closed) {
              i = 0;
            } else {
              Corner c = mesh.ccw_corner(pc);
              if (c && c_wedge_id(c) == c_wedge_id(pc)) {
                assertx(mesh.corner_face(c) == mesh.face1(e) || mesh.corner_face(c) == mesh.face2(e));
                Corner c2 = mesh.clw_face_corner(c);
                Vertex vo = mesh.corner_vertex(c2);
                if (vo == v1)
                  rwid_v1 = c_wedge_id(c2);
                else if (vo == v2)
                  rwid_v2 = c_wedge_id(c2);
                else
                  assertnever("");
              }
              break;
            }
          }
        } else if (dir) {
          if (--i < 0) {
            if (closed) {
              i = nf - 1;
            } else {
              Corner c = mesh.clw_corner(pc);
              if (c && c_wedge_id(c) == c_wedge_id(pc)) {
                assertx(mesh.corner_face(c) == mesh.face1(e) || mesh.corner_face(c) == mesh.face2(e));
                Corner c2 = mesh.ccw_face_corner(c);
                Vertex vo = mesh.corner_vertex(c2);
                if (vo == v1)
                  rwid_v1 = c_wedge_id(c2);
                else if (vo == v2)
                  rwid_v2 = c_wedge_id(c2);
                else
                  assertnever("");
              }
              break;
            }
          }
        }
        int cnwid = nn.ar_nwid[i];
        // if (cnwid >= 0) break;
        Corner c = nn.ar_corners[i][2];
        Corner c1 = !dir ? mesh.clw_corner(c) : mesh.ccw_corner(c);
        Corner pc1 = !dir ? mesh.ccw_corner(pc) : mesh.clw_corner(pc);
        assertx(c1 && pc1);
        assertx((c1 == pc) == (pc1 == c));
        if (c1 == pc) {  // two faces adjacent on same vertex
          bool same = c_wedge_id(c) == c_wedge_id(pc);
          if (cnwid >= 0 && same != (wid == cnwid)) {
            // Equivalence of wid's across edges above v2
            //  (or below v1) would be violated.
            assertnever("Edge collapse would cause problem1");
          }
          // if (cnwid >= 0) break;
          if (!same) break;
          if (cnwid >= 0) {  // 2009-04-15
            assertx(cnwid == wid);
            break;
          }
        } else {  // two faces across from {f1|f2}
          // if (cnwid >= 0) break;
          if (!dir) assertx(mesh.ccw_face_corner(c1) == pc1);
          if (dir) assertx(mesh.clw_face_corner(c1) == pc1);
          bool same = c_wedge_id(pc) == c_wedge_id(pc1) && c_wedge_id(c) == c_wedge_id(c1);
          if (!same) {
            if (1 && c_wedge_id(pc) == c_wedge_id(pc1)) {  // 2009-04-15
              assertx(mesh.corner_face(c1) == mesh.face1(e) || mesh.corner_face(c1) == mesh.face2(e));
              Vertex vo = mesh.corner_vertex(c1);
              if (vo == v1)
                rwid_v1 = c_wedge_id(c1);
              else if (vo == v2)
                rwid_v2 = c_wedge_id(c1);
              else
                assertnever("");
            }
            break;
          }
          if (cnwid >= 0) {  // 2009-04-15
            assertx(cnwid == wid);
            break;
          }
        }
        nn.ar_nwid[i] = wid;
        if (mesh.corner_vertex(c) == v1) rwid_v1 = c_wedge_id(c);
        if (mesh.corner_vertex(c) == v2) rwid_v2 = c_wedge_id(c);
        pc = c;
      }
    }
    for (int r : nn.ar_rwid_v1) {
      // This check is necessary due to new code for rwid_v1 above, else Filterprog would crash on grotto.
      // if (!assertw(r != rwid_v1 && r != rwid_v2)) return false;
      if (!(r != rwid_v1 && r != rwid_v2)) return false;
    }
    for (int r : nn.ar_rwid_v2) {
      // Same here.
      // if (!assertw(r != rwid_v1 && r != rwid_v2)) return false;
      if (!(r != rwid_v1 && r != rwid_v2)) return false;
    }
    nn.ar_rwid_v1.push(rwid_v1);
    nn.ar_rwid_v2.push(rwid_v2);
  }
  if (has_wad2 && wfile_prog) {
    assertx(nn.ar_rwid_v1.num() == 1);
    assertx(nn.ar_rwid_v2.num() == 1);
  }
  if (sdebug) for_int(i, nn.ar_corners.num()) assertx(nn.ar_rwid_v1.ok(nn.ar_nwid[i]));
  return true;
}

// To consider the edge collapse of e, gather information about the neighborhood that would result.
bool gather_nn(Edge e, NewMeshNei& nn) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);  // f2 could be nullptr
  // Gather va, ar_corners.
  gather_nn_1(e, nn);
  // Test validity with respect to wedges.
  if (!gather_nn_valid(e)) return false;
  // Gather ar_nwid, ar_rwid.
  if (!gather_nn_2(e, nn)) return false;
  if (minqem) return true;            // 2024-05-30
  Vec2<bool> eoretire{false, false};  // (v1, vo{1,2}) no longer sharp
  bool eretire{false};                // (v1, v2) was sharp and no adjacent crease
  // Gather eoretire, eretire.
  {
    int nc = nn.ar_corners.num();
    bool closed = nn.va[0] == nn.va.last();
    Vertex vo1 = mesh.side_vertex1(e), vo2 = mesh.side_vertex2(e);
    for_int(i, nc - !closed) {
      // Consider edge between this corner and next one.
      int j = i < nc - 1 ? i + 1 : 0;
      Corner c0 = nn.ar_corners[i][1];
      Corner cc0 = nn.ar_corners[i][2];
      Vertex vc0 = mesh.corner_vertex(cc0);
      Corner c1 = nn.ar_corners[j][0];
      Corner cc1 = nn.ar_corners[j][2];
      Vertex vc1 = mesh.corner_vertex(cc1);
      assertx(mesh.corner_vertex(c0) == mesh.corner_vertex(c1));
      assertx(mesh.corner_face(cc0) != mesh.corner_face(cc1));
      Vertex vo = mesh.corner_vertex(c0);
      bool expect_sharp = edge_sharp(mesh.edge(vc0, vo)) || (vc1 != vc0 && edge_sharp(mesh.edge(vc1, vo)));
      if (!expect_sharp) continue;  // no problem
      bool will_be_sharp = (!same_discrete(mesh.corner_face(cc0), mesh.corner_face(cc1)) || !same_scalar(c0, c1) ||
                            nn.ar_nwid[i] != nn.ar_nwid[j]);
      if (will_be_sharp) continue;  // no problem
      // Problem: edge will in fact retire.
      if (vo == vo1) {
        eoretire[0] = true;
      } else if (vo == vo2) {
        eoretire[1] = true;
      } else {
        // If allowed this, then it would be difficult to encode in vsplit records of PM representation.
        // 1996-04-17 is this still true?
        if (verb >= 2) Warning("Edge collapse would retire external edge");
        return false;
      }
    }
    if (eoretire[0] || eoretire[1]) SSTATV2(Seoretire, (eoretire[0] ? 1 : 0) + (eoretire[1] ? 1 : 0));
  }
  {
    // Retire edge e iff neither v1 nor v2 is a crease vertex
    //  (where a crease vertex has exactly 1 non-retiring sharp edge).
    if (edge_sharp(e)) {
      int dnse = 2 + (eoretire[0] ? 1 : 0) + (eoretire[1] ? 1 : 0);  // desired # sharp edges
      int v1nse = vertex_num_sharpe(v1), v2nse = vertex_num_sharpe(v2);
      if (v1nse != dnse && v2nse != dnse) eretire = true;
    }
    if (eretire) SSTATV2(Seretire, eretire);
  }
  // Gather ar_vdisc, ar_epts, ar_eptv, ar_eptretire.
  {
    Vertex vo1 = mesh.side_vertex1(e), vo2 = mesh.side_vertex2(e);
    int vcreasei = -1;  // crease continuation; 1 of possibly 2 edges
    Vertex cv = (mesh.is_boundary(e)     ? v2
                 : !mesh.is_boundary(v1) ? v2
                 : !mesh.is_boundary(v2) ? v1
                                         : (assertnever_ret(""), implicit_cast<Vertex>(nullptr)));
    Vertex ov = mesh.opp_vertex(cv, e);  // other vertex of rotation (v1 or v2)
    Vertex w = assertx(mesh.most_clw_vertex(cv));
    if (!mesh.is_boundary(cv)) {
      // Find starting point such that all edges ee are visited!
      while (w == ov || mesh.clw_vertex(cv, w) == ov) w = assertx(mesh.clw_vertex(cv, w));
    }
    Vertex wf = w;
    int nva = 0;
    for (;;) {
      nva++;
      for_int(i, 2) {  // run this at most 2 times (usually once)
        Edge ee = mesh.edge(cv, w);
        assertx(ee != e);
        if (!edge_sharp(ee)) {
          assertx(e_setpts(ee).empty());
        } else {
          if ((w == vo1 && eoretire[0]) || (w == vo2 && eoretire[1])) {
            for (eptinfo* pept : e_setpts(ee)) nn.ar_eptretire.push(pept);
          } else {
            int vi = nva - 1;
            for (eptinfo* pept : e_setpts(ee)) {
              nn.ar_epts.push(pept);
              nn.ar_eptv.push(vi);
            }
            if (!nn.ar_vdisc.contains(vi)) nn.ar_vdisc.push(vi);
            int dnse = 2 + eoretire[0] + eoretire[1];
            if (edge_sharp(e) && vertex_num_sharpe(cv) == dnse) vcreasei = vi;  // could be set twice (ok)
          }
        }
        Vertex w2 = mesh.ccw_vertex(cv, w);
        if (w2 == ov) {
          assertx(!i);
          ov = cv;
          cv = w2;
          continue;
        }
        w = w2;
        break;
      }
      if (w == wf || !w) break;
    }
    if (!edge_sharp(e)) {
      assertx(e_setpts(e).empty());
    } else if (eretire) {
      for (eptinfo* pept : e_setpts(e)) nn.ar_eptretire.push(pept);
    } else {
      assertx(vcreasei >= 0);
      for (eptinfo* pept : e_setpts(e)) {
        nn.ar_epts.push(pept);
        nn.ar_eptv.push(vcreasei);
      }
    }
  }
  // Gather ar_fpts.
  for (Face f : mesh.faces(v1)) {
    for (fptinfo* pfpt : f_setpts(f)) nn.ar_fpts.push(pfpt);
    if (f == f1 || f == f2) continue;
  }
  for (Face f : mesh.faces(v2)) {
    if (f == f1 || f == f2) continue;
    for (fptinfo* pfpt : f_setpts(f)) nn.ar_fpts.push(pfpt);
  }
  return true;
}

// Given that optimization starts at position ii between v1 and v2,
//  update scalar wedge attributes around new would-be vertex.
void update_initial_wi(Edge e, const NewMeshNei& nn, int ii, Array<WedgeInfo>& ar_wi) {
  {
    ar_wi.init(0);
    for_int(i, nn.ar_rwid_v1.num()) {
      // It doesn't matter here if ar_rwid_v2 would be chosen since it
      //  is interpolated as need be below.
      int rwid = nn.ar_rwid_v1[i];
      ar_wi.push(gwinfo[rwid]);
    }
  }
  // In the case that half_edges (vs, vt) and (vs, vl) are sharp (and possibly half_edge (vs, vr) is sharp) and
  // vertex vt is smooth, then problem occurs since resulting wedge could be assigned two different attribute values.
  // This should have been prevented earlier.
  //
  const int not_used = std::numeric_limits<int>::max();
  //
  Corner cv1f1 = mesh.corner(mesh.vertex1(e), mesh.face1(e));
  Corner cv2f1 = mesh.ccw_face_corner(cv1f1);
  Corner cv1f1o = mesh.ccw_corner(cv1f1);
  Corner cv2f1o = mesh.clw_corner(cv2f1);
  int nwidv1f1o = !cv1f1o ? not_used : find_nwid(nn, cv1f1o);
  int nwidv2f1o = !cv2f1o ? not_used : find_nwid(nn, cv2f1o);
  bool cv1f1smooth = cv1f1o && c_wedge_id(cv1f1o) == c_wedge_id(cv1f1);
  bool cv2f1smooth = cv2f1o && c_wedge_id(cv2f1o) == c_wedge_id(cv2f1);
  //
  Corner cv1f2 = !mesh.face2(e) ? nullptr : mesh.corner(mesh.vertex1(e), mesh.face2(e));
  Corner cv2f2 = !cv1f2 ? nullptr : mesh.clw_face_corner(cv1f2);
  Corner cv1f2o = !cv1f2 ? nullptr : mesh.clw_corner(cv1f2);
  Corner cv2f2o = !cv1f2 ? nullptr : mesh.ccw_corner(cv2f2);
  int nwidv1f2o = !cv1f2o ? not_used : find_nwid(nn, cv1f2o);
  int nwidv2f2o = !cv2f2o ? not_used : find_nwid(nn, cv2f2o);
  bool cv1f2smooth = cv1f2o && c_wedge_id(cv1f2o) == c_wedge_id(cv1f2);
  bool cv2f2smooth = cv2f2o && c_wedge_id(cv2f2o) == c_wedge_id(cv2f2);
  //
  assertx(!((cv2f1smooth && cv2f2smooth && nwidv2f1o == nwidv2f2o && c_wedge_id(cv1f1) != c_wedge_id(cv1f2)) ||
            (cv1f1smooth && cv1f2smooth && nwidv1f1o == nwidv1f2o && c_wedge_id(cv2f1) != c_wedge_id(cv2f2))));
  //
  if (cv1f1smooth && cv2f1smooth) assertx(nwidv1f1o == nwidv2f1o);
  if (cv1f1smooth)
    ar_wi[nwidv1f1o] = interp_wi(c_winfo(cv1f1), c_winfo(cv2f1), ii);
  else if (cv2f1smooth)
    ar_wi[nwidv2f1o] = interp_wi(c_winfo(cv1f1), c_winfo(cv2f1), ii);
  //
  if (cv1f2smooth && cv2f2smooth) assertx(nwidv1f2o == nwidv2f2o);
  if (cv1f2smooth)
    ar_wi[nwidv1f2o] = interp_wi(c_winfo(cv1f2), c_winfo(cv2f2), ii);
  else if (cv2f2smooth)
    ar_wi[nwidv2f2o] = interp_wi(c_winfo(cv1f2), c_winfo(cv2f2), ii);
}

// Commit the optimized scalar wedge attributes to the new mesh.
void replace_wi(const NewMeshNei& nn, CArrayView<WedgeInfo> ar_wi, CArrayView<int> ar_rwid) {
  assertx(ar_wi.num() == ar_rwid.num());  // optional
  for_int(i, nn.ar_corners.num()) {
    int rwid = ar_rwid[nn.ar_nwid[i]];
    c_wedge_id(nn.ar_corners[i][2]) = rwid;
  }
  for_int(i, ar_wi.num()) {
    int rwid = ar_rwid[i];
    gwinfo[rwid] = ar_wi[i];
  }
}

// Project the points onto the faces described by nn and newp, and store the results as param.
void project_fpts(const NewMeshNei& nn, const Point& newp, Param& param) {
  int nf = nn.ar_corners.num();
  assertx(nf);  // at least one face
  int np = nn.ar_fpts.num();
  assertw(np);
  Array<Bbox<float, 3>> ar_bbox(nf);
  for_int(i, nf) ar_bbox[i] = Bbox{V(newp, mesh.point(nn.va[i]), mesh.point(nn.va[i + 1]))};
  param.ar_mini.init(np);
  param.ar_bary.init(np);
  Array<float> ar_d2(nf);
  int nproj = 0;
  for_int(pi, np) {
    const fptinfo& fpt = *nn.ar_fpts[pi];
    const Point& p = fpt.p;
    // about 1.9 cycles/instr
    // For X3.m, had nf=7 np=50
    // -> expect 47*7*50*1.9 == ~31255 out of 81500 cycles (38% of local_fit)
    //  observed it is about 40% of cycles in local_fit.
    for_int(i, nf) {
      // 37--46 instr/loop (+ delays) when '-O'
      ar_d2[i] = square(lb_dist_point_bbox(p, ar_bbox[i]));
    }
    float min_d2 = BIGFLOAT;
    int min_i = -1;
    Bary min_bary;
    dummy_init(min_bary);
    for (;;) {
      // For X3.m, had nproj = 2.16 -> (2.16 + 1) * 50 * 7 * 9 * 1.9 == ~18900 (23%)
      int tmin_i = arg_min(ar_d2);
      float tmin_d2 = ar_d2[tmin_i];
      if (tmin_d2 >= min_d2) break;
      assertx(tmin_d2 != BIGFLOAT);
      ar_d2[tmin_i] = BIGFLOAT;
      nproj++;
      Bary bary;
      Point dummy_clp;
      float d2 =
          project_point_triangle2(p, mesh.point(nn.va[tmin_i]), mesh.point(nn.va[tmin_i + 1]), newp, bary, dummy_clp);
      if (d2 < min_d2) {
        min_d2 = d2;
        min_i = tmin_i;
        min_bary = bary;
      }
    }
    // Found closest face min_i and corresponding min_bary.
    assertx(min_i >= 0);
    param.ar_mini[pi] = min_i;
    param.ar_bary[pi] = min_bary;
  }
  if (np) SSTATV2(Snproj, float(nproj) / np);
}

// Optimize the position newp of v1 given fixed parameterizations of the face points (param).
// Return the resulting geometric energy (before reprojection -> overestimate).
double fit_geom(const NewMeshNei& nn, const Param& param, float spring, Point& newp) {
  assertx(nn.ar_corners.num());
  SSTATV2(Sfit_nf, nn.ar_corners.num());
  SSTATV2(Sfit_nfp, nn.ar_fpts.num());
  SSTATV2(Sfit_nep, nn.ar_epts.num());
  ULls ulls(newp.data(), 3);
  for_int(pi, nn.ar_fpts.num()) {
    const fptinfo& fpt = *nn.ar_fpts[pi];
    int mini = param.ar_mini[pi];
    const Bary& bary = param.ar_bary[pi];
    // 50 * (21 + 42 * 3) == ~14000 (17%)
    ulls.enter_fprojection(fpt.p, mesh.point(nn.va[mini]), mesh.point(nn.va[mini + 1]), bary[0], bary[1]);
  }
  for_int(i, nn.ar_epts.num()) {
    const eptinfo& ept = *nn.ar_epts[i];
    const Point& p = ept.p;
    int mini = nn.ar_eptv[i];
    float min_bary;
    float min_d2 = project_point_seg2(p, mesh.point(nn.va[mini]), newp, &min_bary);
    if (nn.ar_vdisc.num() == 2) {
      int ovi = other_creasevi(nn, mini);
      float bary;
      float d2 = project_point_seg2(p, mesh.point(nn.va[ovi]), newp, &bary);
      if (d2 < min_d2) {
        // min_d2 = d2;
        mini = ovi;
        min_bary = bary;
      }
    }
    ulls.enter_eprojection(p, mesh.point(nn.va[mini]), min_bary);
  }
  if (spring) {
    float sqrt_spring = sqrt(spring);
    bool closed = nn.va[0] == nn.va.last();
    for_int(i, nn.va.num() - closed) ulls.enter_spring(mesh.point(nn.va[i]), sqrt_spring);
  }
  double rss1;
  ulls.solve(rss1);
  return rss1;
}

// Evaluate the geometric energy with v1 position at newp.
double evaluate_geom(const NewMeshNei& nn, const Param& param, float spring, const Point& newp) {
  double rss1 = 0.;
  int nw = nn.va.num();
  bool closed = nn.va[0] == nn.va.last();
  for_int(pi, nn.ar_fpts.num()) {
    const fptinfo& fpt = *nn.ar_fpts[pi];
    int mini = param.ar_mini[pi];
    const Bary& bary = param.ar_bary[pi];
    rss1 += dist2(fpt.p, interp(mesh.point(nn.va[mini]), mesh.point(nn.va[mini + 1]), newp, bary));
  }
  for_int(i, nn.ar_epts.num()) {
    const eptinfo& ept = *nn.ar_epts[i];
    int mini = nn.ar_eptv[i];
    float min_d2 = project_point_seg2(ept.p, mesh.point(nn.va[mini]), newp);
    if (nn.ar_vdisc.num() == 2) {
      int ovi = other_creasevi(nn, mini);
      float d2 = project_point_seg2(ept.p, mesh.point(nn.va[ovi]), newp);
      if (d2 < min_d2) min_d2 = d2;
    }
    rss1 += min_d2;
  }
  if (spring) for_int(i, nw - closed) rss1 += spring * dist2(mesh.point(nn.va[i]), newp);
  return rss1;
}

double evaluate_maxerr(const NewMeshNei& nn, const Param& param, const Point& newp) {
  float rss1 = 0.;
  for_int(pi, nn.ar_fpts.num()) {
    const fptinfo& fpt = *nn.ar_fpts[pi];
    int mini = param.ar_mini[pi];
    const Bary& bary = param.ar_bary[pi];
    rss1 = max(rss1, dist2(fpt.p, interp(mesh.point(nn.va[mini]), mesh.point(nn.va[mini + 1]), newp, bary)));
  }
  for_int(i, nn.ar_epts.num()) {
    const eptinfo& ept = *nn.ar_epts[i];
    int mini = nn.ar_eptv[i];
    float min_d2 = project_point_seg2(ept.p, mesh.point(nn.va[mini]), newp);
    if (nn.ar_vdisc.num() == 2) {
      int ovi = other_creasevi(nn, mini);
      float d2 = project_point_seg2(ept.p, mesh.point(nn.va[ovi]), newp);
      if (d2 < min_d2) min_d2 = d2;
    }
    rss1 = max(rss1, min_d2);
  }
  return rss1;
}

double evaluate_terrain_resid(const NewMeshNei& nn, const Point& newp) {
  using std::swap;
  const float epsilon = .05f;  // epsilon on integer grid coordinates
  // still safe for puget.16k with single precision
  assertx(epsilon / float(max(grid_dims)) > 1e-6f);
  double max_d2;
  Vec2<int> npyx;
  float npz;
  get_grid_point(newp, npyx, npz);
  const bool lverb = false;
  // const bool lverb = mesh.num_faces() < 240 && mesh.num_faces() > 238;
  // const bool lverb = mesh.num_faces() < 8;
  // Consider the newp itself.
  {
    float d2 = square(npz - get_grid_value(npyx));
    max_d2 = d2;
    if (lverb) showf(" centerpoint (%d, %d)=%g grid=%g\n", npyx[1], npyx[0], npz, get_grid_value(npyx));
    // should be on gridpoint, so zero error
    assertx(d2 < square(1e-5f));
  }
  // Consider the grid points strictly within the triangles, and verify that triangles are not flipped or degenerate.
  int nf = nn.ar_corners.num();
  for_int(fi, nf) {
    int p0x, p0y;
    float p0z;
    p0x = npyx[1];
    p0y = npyx[0];
    p0z = npz;
    Vec2<int> p1yx;
    float p1z;
    get_grid_point(mesh.point(nn.va[fi + 0]), p1yx, p1z);
    Vec2<int> p2yx;
    float p2z;
    get_grid_point(mesh.point(nn.va[fi + 1]), p2yx, p2z);
    int p1y = p1yx[0], p1x = p1yx[1], p2y = p2yx[0], p2x = p2yx[1];
    if ((p1x - p0x) * (p2y - p0y) <= (p1y - p0y) * (p2x - p0x)) {
      // Warning("face would flip or degenerate");
      return k_bad_cost;
    }
    ASSERTX(!((p0x == p1x && p1x == p2x) || (p0y == p1y && p1y == p2y)));
    if (p1y > p2y) {
      swap(p1x, p2x);
      swap(p1y, p2y);
      swap(p1z, p2z);
    }
    if (p0y > p2y) {
      swap(p0x, p2x);
      swap(p0y, p2y);
      swap(p0z, p2z);
    }
    if (p0y > p1y) {
      swap(p0x, p1x);
      swap(p0y, p1y);
      swap(p0z, p1z);
    }
    ASSERTX(p2y >= p1y && p1y >= p0y && p2y > p0y);
    if (lverb)
      showf(" triangle (%d, %d, %g) (%d, %d, %g) (%d, %d, %g)\n", p2x, p2y, p2z, p1x, p1y, p1z, p0x, p0y, p0z);
    if (p2y - p0y < 2) continue;  // no interior grid points
    float p20i = 1.f / (p2y - p0y);
    float cxrinc = (p0x - p2x) * p20i;
    float czrinc = (p0z - p2z) * p20i;
    // upper triangle, plus middle horizontal segment!
    if (p2y > p1y + 0) {
      float xrinc = cxrinc;
      float zrinc = czrinc;
      float p21i = 1.f / (p2y - p1y);
      float xlinc = (p1x - p2x) * p21i;
      float zlinc = (p1z - p2z) * p21i;
      if (xlinc > xrinc) {
        swap(xlinc, xrinc);
        swap(zlinc, zrinc);
      }
      bool extra = (p1y > p0y);  // include middle horizontal segment
      for_intL(i, 1, p2y - p1y + extra) {
        int y = p2y - i;
        float xl = p2x + i * xlinc;
        float xr = p2x + i * xrinc;
        float zl = p2z + i * zlinc;
        float zr = p2z + i * zrinc;
        ASSERTX(xr >= xl);
        if (lverb) showf("  upper %d: (%g, %g) z=[%g, %g]\n", y, xl, xr, zl, zr);
        float slope = (zr - zl) / (xr - xl);
        for_intL(x, int(xl + 1.f + epsilon), int(xr - epsilon) + 1) {
          float z = zl + (x - xl) * slope;
          float d2 = square(z - get_grid_value(V(y, x)));
          if (lverb) showf("   (%d, %d)=%g grid=%g d2=%g\n", x, y, z, get_grid_value(V(y, x)), d2);
          if (d2 > max_d2) max_d2 = d2;
        }
      }
    }
    // lower triangle
    if (p1y > p0y + 1) {
      float xrinc = -cxrinc;
      float zrinc = -czrinc;
      float p10i = 1.f / (p1y - p0y);
      float xlinc = (p1x - p0x) * p10i;
      float zlinc = (p1z - p0z) * p10i;
      if (xlinc > xrinc) {
        swap(xlinc, xrinc);
        swap(zlinc, zrinc);
      }
      for_intL(i, 1, p1y - p0y) {
        int y = p0y + i;
        float xl = p0x + i * xlinc;
        float xr = p0x + i * xrinc;
        float zl = p0z + i * zlinc;
        float zr = p0z + i * zrinc;
        ASSERTX(xr >= xl);
        if (lverb) showf("  lower %d: (%g, %g) z=[%g, %g]\n", y, xl, xr, zl, zr);
        float slope = (zr - zl) / (xr - xl);
        for_intL(x, int(xl + 1.f + epsilon), int(xr - epsilon) + 1) {
          float z = zl + (x - xl) * slope;
          float d2 = square(z - get_grid_value(V(y, x)));
          if (lverb) showf("   (%d, %d)=%g grid=%g d2=%g\n", x, y, z, get_grid_value(V(y, x)), d2);
          if (d2 > max_d2) max_d2 = d2;
        }
      }
    }
  }
  // Consider grid crossings strictly within edges of star(vertex).
  bool closed = nn.va[0] == nn.va.last();
  for_int(ei, nn.va.num() - closed) {
    int p0x, p0y;
    float p0z;
    p0x = npyx[1];
    p0y = npyx[0];
    p0z = npz;
    Vec2<int> p1yx;
    float p1z;
    get_grid_point(mesh.point(nn.va[ei]), p1yx, p1z);
    int p1y = p1yx[0], p1x = p1yx[1];
    if (lverb) showf(" edge (%d, %d, %g) (%d, %d, %g)\n", p0x, p0y, p0z, p1x, p1y, p1z);
    // vertical lines
    if (p1x < p0x) {
      swap(p0x, p1x);
      swap(p0y, p1y);
      swap(p0z, p1z);
    }
    if (p1x > p0x + 1) {
      float xdi = 1.f / (p1x - p0x);
      float yinc = (p1y - p0y) * xdi;
      float zinc = (p1z - p0z) * xdi;
      for_intL(i, 1, p1x - p0x) {
        int x = p0x + i;
        float yf = p0y + i * yinc;
        float z = p0z + i * zinc;
        int y = int(yf);
        float yr = yf - y;
        ASSERTX(y < grid_dims[0] - 1 || yr < epsilon);
        float zg = (yr < epsilon ? get_grid_value(V(y, x))
                                 : ((1.f - yr) * get_grid_value(V(y, x)) + yr * get_grid_value(V(y + 1, x))));
        float d2 = square(z - zg);
        if (d2 > max_d2) max_d2 = d2;
        if (lverb) showf("  vedge(%d, %g)=%g grid=%g d2=%g\n", x, yf, z, zg, d2);
      }
    }
    // horizontal lines
    if (p1y < p0y) {
      swap(p0x, p1x);
      swap(p0y, p1y);
      swap(p0z, p1z);
    }
    if (p1y > p0y + 1) {
      float ydi = 1.f / (p1y - p0y);
      float xinc = (p1x - p0x) * ydi;
      float zinc = (p1z - p0z) * ydi;
      for_intL(i, 1, p1y - p0y) {
        int y = p0y + i;
        float xf = p0x + i * xinc;
        float z = p0z + i * zinc;
        int x = int(xf);
        float xr = xf - x;
        ASSERTX(x < grid_dims[1] - 1 || xr < epsilon);
        float zg = (xr < epsilon ? get_grid_value(V(y, x))
                                 : ((1.f - xr) * get_grid_value(V(y, x)) + xr * get_grid_value(V(y, x + 1))));
        float d2 = square(z - zg);
        if (d2 > max_d2) max_d2 = d2;
        if (lverb) showf("  hedge(%g, %d)=%g grid=%g d2=%g\n", xf, y, z, zg, d2);
      }
    }
    // diagonal lines
    int p0l = p0y - p0x;
    int p1l = p1y - p1x;
    if (p1l < p0l) {
      swap(p0x, p1x);
      swap(p0y, p1y);
      swap(p0z, p1z);
      swap(p0l, p1l);
    }
    if (p0l < p1l + 1) {
      float ldi = 1.f / (p1l - p0l);
      float xinc = (p1x - p0x) * ldi;
      float yinc = (p1y - p0y) * ldi;
      float zinc = (p1z - p0z) * ldi;
      for_intL(i, 1, p1l - p0l) {
        float xf = p0x + i * xinc;
        float yf = p0y + i * yinc;
        float z = p0z + i * zinc;
        int x = int(xf);
        int y = int(yf);
        float xr = xf - x;
        float yr = yf - y;
        if (yr > xr + .5f) {
          ASSERTX(yr > xr + 1.f - epsilon);  // numerical imprecision
          y++;
          // yr = 0.f;
        } else if (xr > yr + .5f) {
          ASSERTX(xr > yr + 1.f - epsilon);  // numerical imprecision
          x++;
          xr = 0.f;
        }
        ASSERTX((x < grid_dims[1] - 1 && y < grid_dims[0] - 1) || xr < epsilon);
        float zg = (xr < epsilon ? get_grid_value(V(y, x))
                                 : ((1.f - xr) * get_grid_value(V(y, x)) + xr * get_grid_value(V(y + 1, x + 1))));
        float d2 = square(z - zg);
        if (d2 > max_d2) max_d2 = d2;
        if (lverb) showf("  dedge(%g, %g)=%g grid=%g d2=%g\n", xf, yf, z, zg, d2);
      }
    }
  }
  return max_d2;
}

// Optimize the scalar attributes of wedges around v1 given fixed parameterizations of the face points (param).
// Return the resulting scalar energy.
double fit_color(const NewMeshNei& nn, const Param& param, ArrayView<WedgeInfo> ar_wi) {
  assertx(ar_wi.num() == nn.ar_rwid_v1.num());
  // SSTATV2(Srwid, ar_wi.num());
  Array<unique_ptr<ULls>> ar_ulls;
  for_int(i, ar_wi.num()) {
    A3dColor& newcolor = ar_wi[i].col;
    ar_ulls.push(make_unique<ULls>(newcolor.data(), 3));
  }
  for_int(pi, nn.ar_fpts.num()) {
    const fptinfo& fpt = *nn.ar_fpts[pi];
    if (fpt.ptcol()[0] == k_undefined) continue;
    int mini = param.ar_mini[pi];
    int nwid = nn.ar_nwid[mini];
    ASSERTX(nn.ar_rwid_v1.ok(nwid));
    const Bary& bary = param.ar_bary[pi];
    const P2WedgeInfo& p2wi = nn.ar_p2wi[mini];
    // ignore gcolc factor for now
    ar_ulls[nwid]->enter_fprojection(fpt.ptcol(), p2wi.wi[0]->col, p2wi.wi[1]->col, bary[0], bary[1]);
  }
  double rss1 = 0.;
  int cliprgb = 0;
  for_int(i, ar_wi.num()) {
    double rssp1 = 0.;
    // May have 0 points projecting -> ill-defined -> keeps same value.
    ar_ulls[i]->solve(rssp1);
    rss1 += rssp1;
    A3dColor& newcolor = ar_wi[i].col;
    for_int(c, 3) {
      if (newcolor[c] < 0.f) {
        newcolor[c] = 0.f;
        cliprgb++;
      } else if (newcolor[c] > 1.f) {
        newcolor[c] = 1.f;
        cliprgb++;
      }
    }
  }
  SSTATV2(Scliprgb, cliprgb);
  if (cliprgb) {  // have some clipped rgb values
    rss1 = 0.;
    for_int(pi, nn.ar_fpts.num()) {
      const fptinfo& fpt = *nn.ar_fpts[pi];
      if (fpt.ptcol()[0] == k_undefined) continue;
      int mini = param.ar_mini[pi];
      int nwid = nn.ar_nwid[mini];
      const Bary& bary = param.ar_bary[pi];
      const P2WedgeInfo& p2wi = nn.ar_p2wi[mini];
      // ignore gcolc factor for now
      rss1 += dist2(fpt.ptcol(), interp(p2wi.wi[0]->col, p2wi.wi[1]->col, ar_wi[nwid].col, bary));
    }
  }
  return rss1 * gcolc;
}

// Evaluate the scalar energy with wedge attributes around v1 set to ar_wi.
double evaluate_color(const NewMeshNei& nn, const Param& param, CArrayView<WedgeInfo> ar_wi) {
  assertx(ar_wi.num() == nn.ar_rwid_v1.num());
  double rss1 = 0.;
  for_int(pi, nn.ar_fpts.num()) {
    const fptinfo& fpt = *nn.ar_fpts[pi];
    if (fpt.ptcol()[0] == k_undefined) continue;
    int mini = param.ar_mini[pi];
    int nwid = nn.ar_nwid[mini];
    const Bary& bary = param.ar_bary[pi];
    const P2WedgeInfo& p2wi = nn.ar_p2wi[mini];
    // ignore gcolc factor for now
    rss1 += dist2(fpt.ptcol(), interp(p2wi.wi[0]->col, p2wi.wi[1]->col, ar_wi[nwid].col, bary));
  }
  return rss1 * gcolc;
}

// Evaluate the scalar energy with wedge attributes around v1 set to ar_wi.
double evaluate_normal(const NewMeshNei& nn, const Param& param, CArrayView<WedgeInfo> ar_wi) {
  assertx(ar_wi.num() == nn.ar_rwid_v1.num());
  double rss1 = 0.;
  for_int(pi, nn.ar_fpts.num()) {
    const fptinfo& fpt = *nn.ar_fpts[pi];
    if (fpt.ptnor()[0] == k_undefined) continue;
    int mini = param.ar_mini[pi];
    int nwid = nn.ar_nwid[mini];
    const Bary& bary = param.ar_bary[pi];
    const P2WedgeInfo& p2wi = nn.ar_p2wi[mini];
    // ignore gnorc factor for now
    rss1 += dist2(fpt.ptnor(), ok_normalized(interp(p2wi.wi[0]->nor, p2wi.wi[1]->nor, ar_wi[nwid].nor, bary)));
  }
  return rss1 * gnorc;
}

void compute_residual(CArrayView<Vector> ar_resid, CArrayView<float> ar_normaldist2, const Vector& vnormal,
                      float& uni_error, float& dir_error) {
  if (!ar_resid.num()) {
    if (numpts) assertw(ar_resid.num());
    uni_error = 0.f;
    dir_error = 0.f;
    return;
  }
  const float accept_factor = 2.f;
  int npnormal = 0;  // number of possible normal projections
  int nnormal = 0;
  float bb_x = 0.f, bb_y = 0.f;
  for_int(i, ar_resid.num()) {
    float m2 = mag2(ar_resid[i]);
    float y, x;
    if (ar_normaldist2[i] != BIGFLOAT) npnormal++;
    if ((rnor001 && ar_normaldist2[i] != BIGFLOAT) || ar_normaldist2[i] < square(accept_factor) * m2) {
      nnormal++;
      m2 = ar_normaldist2[i];
      y = my_sqrt(m2);
      x = 0.f;
    } else {
      y = abs(dot(ar_resid[i], vnormal));
      x = my_sqrt(m2 - square(y));
    }
    if (x > bb_x) bb_x = x;
    if (y > bb_y) bb_y = y;
  }
  SSTATV2(Sres_npnor, float(npnormal) / ar_resid.num());
  SSTATV2(Sres_nnor, float(nnormal) / ar_resid.num());
  if (!bb_x) {
    uni_error = 0.f;
    dir_error = bb_y;
  } else {
    float ratio_bb = !bb_x ? 1000.f : !bb_y ? 0.f : bb_y / bb_x;
    if (ratio_bb <= 1.001f) ratio_bb = 1.001f;
    // (when ratio_bb<=1.f, only the uniform error component matters.)
    float vtan = 1.f / my_sqrt(square(ratio_bb) - 1.f);
    float max_u = 0.f;
    for_int(i, ar_resid.num()) {
      float m2 = mag2(ar_resid[i]);
      float y, x;
      if ((rnor001 && ar_normaldist2[i] != BIGFLOAT) || ar_normaldist2[i] < square(accept_factor) * m2) {
        m2 = ar_normaldist2[i];
        y = my_sqrt(m2);
        x = 0.f;
      } else {
        y = abs(dot(ar_resid[i], vnormal));
        x = my_sqrt(m2 - square(y));
      }
      if (!x) {
        if (y > max_u * ratio_bb) max_u = y / ratio_bb;
        // yes, checked equivalence with below
      } else {
        float ratio_yx = y / x;
        if (ratio_yx <= vtan) {
          // bounded by uniform component
          float m = my_sqrt(m2);
          if (m > max_u) max_u = m;
          // yes, checked equivalence with below at ratio_yx == vtan
        } else {
          // bounded by directional component
          float desired_d = y + x / vtan;
          if (desired_d > max_u * ratio_bb) max_u = desired_d / ratio_bb;
        }
      }
    }
    uni_error = max_u;
    dir_error = max_u * ratio_bb;
    SSTATV2(Sres_ufrac, 1.f / ratio_bb);
  }
  SSTATV2(Sres_uni, uni_error);
  SSTATV2(Sres_dir, dir_error);
}

// After ecol is committed, reproject points onto new mesh neighborhood and update global structures.
void reproject_locally(const NewMeshNei& nn, float& uni_error, float& dir_error) {
  uni_error = 0.f;
  dir_error = 0.f;
  bool handle_residuals = !!wfile_prog;
  Vector vnormal{};
  {
    // Compute average vertex normal (as will be done in SrMesh if the vertex doesn't have a unique normal).
    for_int(i, nn.ar_corners.num()) {
      Corner c = nn.ar_corners[i][2];
      int wid = c_wedge_id(c);
      const Vector& nor = gwinfo[wid].nor;
      assertx(nor[0] != k_undefined);
      vnormal += nor;
    }
    assertw(vnormal.normalize());
  }
  if (rnor001) vnormal = Vector(0.f, 0.f, 1.f);
  Array<Vector> ar_resid;
  Array<float> ar_normaldist2;
  {
    int totpoints = nn.ar_fpts.num() + nn.ar_epts.num();
    ar_resid.reserve(totpoints);
    ar_normaldist2.reserve(totpoints);
  }
  // Current problem with rnor001:
  //  Both the distance evaluation in fit_geom() and evaluate_geom() and the reprojection here do not use
  //  the (0, 0, 1) vector direction.
  //  That mis-reprojection is why we sometimes see a uniform error component here.
  {
    int nf = nn.ar_corners.num();
    Array<Bbox<float, 3>> ar_bbox(nf);
    for_int(i, nf) {
      Face f = mesh.corner_face(nn.ar_corners[i][2]);
      ar_bbox[i] = Bbox{transform(mesh.vertices(f), [&](Vertex v) { return mesh.point(v); })};
    }
    Polygon poly;
    Array<float> ar_d2(nf);
    for (fptinfo* pfpt : nn.ar_fpts) {
      fptinfo& fpt = *pfpt;
      const Point& p = fpt.p;
      for_int(i, nf) ar_d2[i] = square(lb_dist_point_bbox(p, ar_bbox[i]));
      float min_d2 = BIGFLOAT;
      Face min_f = nullptr;
      Bary min_bary{};
      for (;;) {
        int tmin_i = arg_min(ar_d2);
        float tmin_d2 = ar_d2[tmin_i];
        if (tmin_d2 >= min_d2) break;
        assertx(tmin_d2 != BIGFLOAT);
        ar_d2[tmin_i] = BIGFLOAT;
        Face f = mesh.corner_face(nn.ar_corners[tmin_i][2]);
        mesh.polygon(f, poly);
        Bary bary;
        Point dummy_clp;
        float d2 = project_point_triangle2(p, poly[0], poly[1], poly[2], bary, dummy_clp);
        if (d2 < min_d2) {
          min_d2 = d2;
          min_f = f;
          min_bary = bary;
        }
      }
      assertx(min_f);
      fpt.dist2 = min_d2;
      if (have_ccolors && fpt.ptcol()[0] != k_undefined) {
        Vec3<Corner> ca = mesh.triangle_corners(min_f);
        fpt.coldist2() =
            dist2(interp(c_winfo(ca[0]).col, c_winfo(ca[1]).col, c_winfo(ca[2]).col, min_bary[0], min_bary[1]),
                  Point(fpt.ptcol()));
      }
      if (have_cnormals && norfac && fpt.ptnor()[0] != k_undefined) {
        Vec3<Corner> ca = mesh.triangle_corners(min_f);
        Vector nor = ok_normalized(
            interp(c_winfo(ca[0]).nor, c_winfo(ca[1]).nor, c_winfo(ca[2]).nor, min_bary[0], min_bary[1]));
        fpt.nordist2() = mag2(nor - fpt.ptnor());
      }
      point_change_face(&fpt, min_f);
      if (handle_residuals) {
        mesh.polygon(min_f, poly);
        ar_resid.push((p - interp(poly[0], poly[1], poly[2], min_bary)));
        Point pint;
        float d2 = BIGFLOAT;
        if (poly.intersect_line(p, vnormal, pint)) {
          d2 = dist2(p, pint);
        } else {
          for_int(i, nf) {
            Face f = mesh.corner_face(nn.ar_corners[i][2]);
            mesh.polygon(f, poly);
            float eps = 1e-3f;
            // We used to have eps=1e-5f, but this would result in Sres_npnor av=.95 for gcanyon_sq200
            // (it should be 1.0 except for the last edge collapse which changes terrain to a triangle).
            // With eps=1e-3f, Sres_npnor av=.9999 (excellent).
            // It must have been a numerical problem in intersect_line().
            widen_triangle(poly, eps);
            if (poly.intersect_line(p, vnormal, pint)) {
              d2 = dist2(p, pint);
              break;
            }
          }
        }
        ar_normaldist2.push(d2);
      }
    }
  }
  {
    Vertex v1 = mesh.corner_vertex(nn.ar_corners[0][2]);
    const Point& newp = mesh.point(v1);
    for_int(i, nn.ar_epts.num()) {
      eptinfo& ept = *nn.ar_epts[i];
      const Point& p = ept.p;
      int mini = nn.ar_eptv[i];
      float min_d2 = project_point_seg2(p, mesh.point(nn.va[mini]), newp);
      if (nn.ar_vdisc.num() == 2) {
        int ovi = other_creasevi(nn, mini);
        float d2 = project_point_seg2(p, mesh.point(nn.va[ovi]), newp);
        if (d2 < min_d2) {
          min_d2 = d2;
          mini = ovi;
        }
      }
      Edge e = mesh.edge(v1, nn.va[mini]);
      ept.dist2 = min_d2;
      // ept.clp = interp(mesh.point(nn.va[mini]), newp, min_bary);
      // if (mesh.vertex2(e) != v1) min_bary = 1.f - min_bary;
      point_change_edge(&ept, e);
      if (handle_residuals) {
        float cba;
        project_point_seg2(p, mesh.point(nn.va[mini]), newp, &cba);
        ar_resid.push((p - interp(mesh.point(nn.va[mini]), newp, cba)));
        ar_normaldist2.push(BIGFLOAT);
      }
    }
  }
  if (handle_residuals) compute_residual(ar_resid, ar_normaldist2, vnormal, uni_error, dir_error);
}

// ret: success
bool compute_hull_point(Edge e, const NewMeshNei& nn, Point& newpoint) {
  if (nn.va.num() == 2) {
    Warning("hull: ignoring boundary 'corner' vertex");
    return false;
  }
  // Set up a linear programming problem.
  // Recall that all variables are expected to be >= 0, annoying!
  //  workaround: translate all variables into (x, y, z) > 0 octant
  // simplx: epsilon is not scale-invariant -> scale data as well.
  // p' = p * scale + translate .   p' in range [border, border + size]
  float scale;
  Vector translate;
  const float transf_border = 10.f;  // allow vertex to go anywhere
  // const float transf_border = .25f;  // severely constrain the vertex
  // const float transf_border = 1.f;
  const float transf_size = 1.f;
  {
    const Bbox bbox{transform(mesh.vertices(e), [&](Vertex v) { return mesh.point(v); })};
    if (!bbox.max_side()) {
      Warning("Empty bbox");
      return false;
    }
    scale = transf_size / bbox.max_side();
    translate = (Vector(transf_border, transf_border, transf_border) + (Point(0.f, 0.f, 0.f) - bbox[0]) * scale);
  }
  // Gather constraints.
  Array<LinearFunc> ar_lf;
  {
    Vertex v2 = mesh.vertex2(e);
    Face f1 = mesh.face1(e), f2 = mesh.face2(e);
    Polygon poly;
    for (Vertex v : mesh.vertices(e)) {
      for (Face f : mesh.faces(v)) {
        if (v == v2 && (f == f1 || f == f2)) continue;
        mesh.polygon(f, poly);
        Vector normal = poly.get_normal_dir();
        // Normalize normal for numerical precision in simplx.
        if (!normal.normalize()) {
          Warning("Ignoring degenerate face");
          continue;
        }
        if (innerhull) normal = -normal;
        Point pold = poly[0];
        Point pnew = to_Point(to_Vector(pold) * scale) + translate;
        if (1) {
          for_int(c, 3) {
            if (pnew[c] <= transf_border - 1e-4f || pnew[c] >= transf_border + transf_size + 1e-4f) assertnever("");
          }
        }
        ar_lf.push(LinearFunc(normal, pnew));
      }
    }
  }
  if (1) {
    // See if (v1 + v2) / 2 satisfies the constraints.
    // (Useful in the case of complete planarity.)
    Point pcand = interp(mesh.point(mesh.vertex1(e)), mesh.point(mesh.vertex2(e)));
    Point pcandnew = to_Point(to_Vector(pcand) * scale) + translate;
    const bool ok = all_of(ar_lf, [&](const LinearFunc& lf) { return lf.eval(pcandnew) >= -1e-5f; });
    if (ok) {
      Warning("Using midpoint as hull point");
      newpoint = pcand;
      return true;
    }
  }
  // Gather objective function.
  Vector link_normal;
  {
    Polygon poly;
    bool closed = nn.va[0] == nn.va.last();
    if (!closed) Warning("compute_hull not designed for boundaries");
    for_int(i, nn.va.num() - closed) poly.push(mesh.point(nn.va[i]));
    link_normal = poly.get_normal_dir();
    // Normalize for numerical precision in simplx.
    if (!link_normal.normalize()) {
      Warning("Zero volume gradient; skipping");
      return false;
    }
    if (innerhull) link_normal = -link_normal;
  }
#if defined(HH_HAVE_SIMPLEX)
  // Count number of constraints which are of form lfunc(x, y, z) >= d >= 0
  int n_m1 = 0;
  for_int(i, ar_lf.num()) {
    if (ar_lf[i].offset > 0) n_m1++;
  }
  // Use Numerical Recipes code (converted to double precision)
  int n = 3;                                       // number of variables (x, y, z)
  int m1 = n_m1, m2 = ar_lf.num() - n_m1, m3 = 0;  // '<=', '>=', '==' constraints
  int m = m1 + m2 + m3;                            // total number of constraints
  double** a = dmatrix(1, m + 2, 1, n + 1);
  int* iposv = ivector(1, m);
  int* izrov = ivector(1, n);
  int icase;
  // Maximize the linear functional: -x*link_normal[0]-y*link_normal[1]...
  a[1][1] = 0.f;  // always
  a[1][2] = -link_normal[0];
  a[1][3] = -link_normal[1];
  a[1][4] = -link_normal[2];
  // Subject to the constraint: lf.eval(Point(x, y, z)) >= 0
  //  equivalently, lf[0] * x + lf[1] * y + lf[2] * z >= -lf[3]
  //  or possibly, -lf[0] * x - lf[1] * y - lf[2] * z <= lf[3]
  {
    int nr = 0;
    for_int(i, ar_lf.num()) {
      const LinearFunc& lf = ar_lf[i];
      if (!(lf.offset > 0)) continue;
      a[2 + nr][1] = lf.offset;
      a[2 + nr][2] = lf.v[0];
      a[2 + nr][3] = lf.v[1];
      a[2 + nr][4] = lf.v[2];
      nr++;
    }
    assertx(nr == m1);
    for_int(i, ar_lf.num()) {
      const LinearFunc& lf = ar_lf[i];
      if (lf.offset > 0) continue;
      a[2 + nr][1] = -lf.offset;
      a[2 + nr][2] = -lf.v[0];
      a[2 + nr][3] = -lf.v[1];
      a[2 + nr][4] = -lf.v[2];
      nr++;
    }
    assertx(nr == m);
  }
  {
    // We could instead use the Raymond Seidel (or Raimund Seidel) fast linear programming scheme, copied from
    // Seth Teller into rseidel_lp.tar.gz, then into linprog.tar.gz .
    simplx(a, m, n, m1, m2, m3, &icase, izrov, iposv);
  }
  bool ret = false;
  if (icase == -2) {
    Warning("simplx: hhiter exceeded");
  } else if (icase == -1) {
    Warning("simplx: no solution satisfies constraints");
  } else if (icase < 0) {
    assertnever("");
  } else if (icase > 0) {
    Warning("simplx: objective function is unbounded");
  } else {
    // A finite solution is found.
    Point pnew(0.f, 0.f, 0.f);  // must be initialized to zero's.
    for_intL(i, 1, m + 1) {
      assertx(iposv[i] >= 1);
      if (iposv[i] <= n) pnew[iposv[i] - 1] = float(a[1 + i][1]);
    }
    if (1) {
      for_int(i, ar_lf.num()) {
        if (ar_lf[i].eval(pnew) < -1e-4f) {
          Warning("Simplx linear inequality not satisfied");
          SSTATV2(Ssimplxbad, ar_lf[i].eval(pnew));
        }
      }
    }
    bool good = true;
    for_int(c, 3) {
      assertw(pnew[c] >= -1e-4f);
      if (pnew[c] < .01f) {
        Warning("hull point outside border_min");
        good = false;
        break;
      }
      if (pnew[c] > transf_border * 2 + transf_size) {
        Warning("hull point outside border_max");
        good = false;
        break;
      }
    }
    if (good) {
      newpoint = to_Point(to_Vector(pnew - translate) * (1.f / scale));
      ret = true;
    }
  }
  free_dmatrix(a, 1, m + 2, 1, n + 1);
  free_ivector(iposv, 1, m);
  free_ivector(izrov, 1, n);
  return ret;
#else   // defined(HH_HAVE_SIMPLEX)
  assertnever("Linear programming (simplex) is unavailable");
#endif  // defined(HH_HAVE_SIMPLEX)
}

float compute_volume_before(Edge e) {
  float vol = 0.f;
  Vertex v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  Polygon poly;
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (v == v2 && (f == f1 || f == f2)) continue;
      mesh.polygon(f, poly);
      vol += dot(cross(to_Vector(poly[0]), to_Vector(poly[1])), to_Vector(poly[2]));
    }
  }
  return vol;
}

float compute_volume_after(const NewMeshNei& nn, const Point& newp) {
  float vol = 0.f;
  for_int(i, nn.va.num() - 1) {
    vol += dot(cross(to_Vector(mesh.point(nn.va[i])), to_Vector(mesh.point(nn.va[i + 1]))), to_Vector(newp));
  }
  return vol;
}

float compute_area_before(Edge e) {
  float area = 0.f;
  Vertex v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (v == v2 && (f == f1 || f == f2)) continue;
      area += mesh.area(f);
    }
  }
  return area;
}

float compute_area_after(const NewMeshNei& nn, const Point& newp) {
  float area = 0.f;
  for_int(i, nn.va.num() - 1) area += my_sqrt(area2(mesh.point(nn.va[i]), mesh.point(nn.va[i + 1]), newp));
  return area;
}

bool is_grid_corner(Vertex v) {
  Vec2<int> pyx;
  float pz;
  get_grid_point(mesh.point(v), pyx, pz);
  return (pyx[0] == 0 || pyx[0] == grid_dims[0] - 1) && (pyx[1] == 0 || pyx[1] == grid_dims[1] - 1);
}

bool is_uv_corner(Corner c) {
  const UV& uv = c_winfo(c).uv;
  const float eps = 1e-6f;
  return ((abs(uv[0] - 0.f) < eps || abs(uv[0] - 1.f) < eps) && (abs(uv[1] - 0.f) < eps || abs(uv[1] - 1.f) < eps));
}

int estimate_ii(Vertex v1, Vertex v2, const Point& newp) {
  const Point& p1 = mesh.point(v1);
  const Point& p2 = mesh.point(v2);
  Vector vp = (p2 - p1);
  float vpm2 = mag2(vp);
  if (!vpm2) return 1;
  float a = dot(newp - p1, vp) / vpm2;
  const float fudge = 0.2f;
  return a < fudge ? 2 : a > 1.f - fudge ? 0 : 1;
}

void project_vnew_nn(Edge e, const Point& newp, Face& pcf, Bary& pbary, Point& pclp) {
  Vertex v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);  // f2 could be nullptr
  Polygon poly;
  float min_d2 = BIGFLOAT;
  pcf = f1;  // dummy assignment for clang-tidy
  for (Vertex v : mesh.vertices(e)) {
    for (Face f : mesh.faces(v)) {
      if (v == v2 && (f == f1 || f == f2)) continue;
      mesh.polygon(f, poly);
      Bary bary;
      Point clp;
      float d2 = project_point_triangle2(newp, poly[0], poly[1], poly[2], bary, clp);
      if (d2 < min_d2) {
        min_d2 = d2;
        pcf = f;
        pbary = bary;
        pclp = clp;
      }
    }
  }
}

template <int n>
Vec<float, n> interp_3way(const Vec<float, n>& ar1, const Vec<float, n>& ar2, const Vec<float, n>& ar3,
                          const Bary& bary) {
  if (ar1[0] == k_undefined) return ntimes<n>(k_undefined);
  return interp(ar1, ar2, ar3, bary);
}

void perform_attrib_project(Edge e, const Point& newp, ArrayView<WedgeInfo> ar_wi) {
  assertx(ar_wi.num() == 1);
  Face cf;
  Bary bary;
  Point clp;
  project_vnew_nn(e, newp, cf, bary, clp);
  Vec3<Vertex> va = mesh.triangle_vertices(cf);
  const WedgeInfo& wi1 = c_winfo(mesh.corner(va[0], cf));
  const WedgeInfo& wi2 = c_winfo(mesh.corner(va[1], cf));
  const WedgeInfo& wi3 = c_winfo(mesh.corner(va[2], cf));
  WedgeInfo& wi = ar_wi[0];
  wi.col = interp_3way(wi1.col, wi2.col, wi3.col, bary);
  wi.nor = interp_3way(wi1.nor, wi2.nor, wi3.nor, bary);
  wi.uv = interp_3way(wi1.uv, wi2.uv, wi3.uv, bary);
  // Interpolated normal may be zero.  Will test for this later.
  if (wi.nor[0] != k_undefined) wi.nor.normalize();
}

// Note that ar_ce may contain duplicate wid or owid !
Array<CacheEntry> tvc_get_e_cacheentry(Edge e, bool edir) {
  Corner csl = mesh.corner(mesh.vertex1(e), mesh.face1(e));
  Corner ctl = mesh.ccw_face_corner(csl);
  Corner cll = mesh.ccw_face_corner(ctl);
  ASSERTX(mesh.corner_vertex(cll) == mesh.side_vertex1(e));
  Corner csr = mesh.clw_corner(csl), ctr = nullptr, crr = nullptr;
  if (csr) {
    ctr = mesh.clw_face_corner(csr);
    crr = mesh.clw_face_corner(ctr);
    ASSERTX(mesh.corner_vertex(crr) == mesh.side_vertex2(e));
  }
  if (edir) {
    std::swap(cll, crr);
    std::swap(csl, ctr);
    std::swap(ctl, csr);
  }
  const bool add_duplicates = true;
  Array<CacheEntry> ar_ce;
  if (1) {
    // Current PM face vertex order.
    if (csl) ar_ce.push_array(V(CacheEntry(csl), CacheEntry(ctl), CacheEntry(cll)));
    if (csr) {
      if (add_duplicates || !csl || (!tvcowid && c_wedge_id(csr) != ar_ce[0].wid) ||
          (tvcowid && c_tvc_owid(csr) != ar_ce[0].owid))
        ar_ce.push(CacheEntry(csr));
      if (1) ar_ce.push(CacheEntry(crr));
      if (add_duplicates || !csl || (!tvcowid && c_wedge_id(ctr) != ar_ce[1].wid) ||
          (tvcowid && c_tvc_owid(ctr) != ar_ce[1].owid))
        ar_ce.push(CacheEntry(ctr));
    }
  } else {
    // Would-be possibly better order for PM face order
    assertnever("");
    // Process in order: cll, csl, ctl,  ctr, csr, crr
  }
  if (!add_duplicates) {
    for_int(i, ar_ce.num()) {
      for_intL(j, i + 1, ar_ce.num()) {
        if (!tvcowid) ASSERTX(ar_ce[i].wid != ar_ce[j].wid);
        ASSERTX(ar_ce[i].owid != ar_ce[j].owid);
      }
    }
  }
  return ar_ce;
}

void tvc_print_cache() {
  std::cerr << "cache={\n";
  for (const CacheEntry& ce : tvc_cache)
    std::cerr << sform(" (wid=%d, v=%d, owid=%d)\n", ce.wid, (ce.v ? mesh.vertex_id(ce.v) : 0), ce.owid);
  std::cerr << "}\n";
}

void tvc_update_cache(Edge e) {
  // Insert wedges referred to by {fl, fr} into tvc_cache.
  // Final PM order:  { facel , facer , cached_vertices } , so insert them in reverse order.
  Array<CacheEntry> ar_ce;
  {
    float tvccost;
    bool edir;
    get_tvc_cost_edir(e, tvccost, edir);
    ar_ce = tvc_get_e_cacheentry(e, edir);
  }
  const bool print_cache = false;
  if (print_cache) SHOWL;
  for_int(ii, ar_ce.num()) {
    const CacheEntry& ce = ar_ce[ar_ce.num() - 1 - ii];
    // Insert ce into tvc_cache.
    CacheEntry cetmp = ce;
    bool found = false;
    for_int(j, tvc_cache.num()) {
      std::swap(tvc_cache[j], cetmp);
      if (cetmp.wid == (!tvcowid ? ce.wid : ce.owid)) {
        found = true;
        break;
      }
    }
    tvc_ncachemiss += !found;
    if (print_cache) tvc_print_cache();
  }
}

bool try_ecol_legal(Edge e, Vertex v1, Vertex v2, int v1nse, int v2nse) {
  if (no_simp_bnd && (mesh.is_boundary(v1) || mesh.is_boundary(v2))) return false;
  if (maxvalence && mesh.degree(v1) + mesh.degree(v2) - (mesh.is_boundary(e) ? 3 : 4) > maxvalence) return false;
  if (strict_sharp) {
    // Also see additional restrictions on min_ii below.
    if (!edge_sharp(e)) {
      if (v1nse >= 1 && v2nse >= 1) {
        if (verb >= 2) Warning("Edge collapse would offend sharp edges (a)");
        return false;
      }
    } else {
      if (v1nse >= 3 && v2nse >= 3) {
        if (verb >= 2) Warning("Edge collapse would offend sharp edges (b)");
        return false;
      }
      for (Face f : mesh.faces(e)) {
        Vertex vs = mesh.opp_vertex(e, f);
        if (edge_sharp(mesh.edge(v1, vs)) && edge_sharp(mesh.edge(v2, vs))) {
          if (verb >= 2) Warning("Edge collapse would offend sharp edges (c)");
          return false;
        }
      }
      if ((v1nse == 1 && v2nse != 2) || (v2nse == 1 && v1nse != 2)) {
        if (verb >= 2) Warning("Edge collapse would cause dart to vanish");
        return false;
      }
    }
  }
  return true;
}

bool intersect_lines(Point p1, Point p2, Point p3, Point p4, float& ia, float& ib) {
  p1[2] = p2[2] = p3[2] = p4[2] = 1.f;
  Point pi = to_Point(cross(cross(to_Vector(p1), to_Vector(p2)), cross(to_Vector(p3), to_Vector(p4))));
  if (!pi[2]) return false;
  pi[0] /= pi[2];
  pi[1] /= pi[2];
  pi[2] = 0;
  ia = 1.f - dot(p2 - p1, pi - p1) / dot(p2 - p1, p2 - p1);
  ib = 1.f - dot(p4 - p3, pi - p3) / dot(p4 - p3, p4 - p3);
  return ia >= 0.f && ia <= 1.f && ib >= 0.f && ib <= 1.f;
}

inline Point c_uv(Corner c) { return Point(c_winfo(c).uv[0], c_winfo(c).uv[1], 0.f); }

void check_ccw(Vertex v) {
  Vec3<Point> p;
  for (Face f : mesh.faces(v)) {
    int i = 0;
    for (Vertex vv : mesh.vertices(f)) p[i++] = c_uv(mesh.corner(vv, f));
    assertw(cross(p[0], p[1], p[2])[2] >= 0.f);
  }
}

bool valid_aps(Edge e, Vertex v1, Vertex v2) {
  Vec3<Point> pa;
  for (Corner c : mesh.corners(v1)) {
    Face f = mesh.corner_face(c);
    Corner cs1 = mesh.ccw_face_corner(c);
    Corner cs2 = mesh.clw_face_corner(c);
    Vertex vs1 = mesh.corner_vertex(cs1);
    Vertex vs2 = mesh.corner_vertex(cs2);
    if (vs1 == v2 || vs2 == v2) continue;
    Face ft;
    if (f_matid(f) == f_matid(mesh.face1(e))) {
      ft = mesh.face1(e);
    } else {
      assertx(mesh.face2(e) && f_matid(f) == f_matid(mesh.face2(e)));
      ft = mesh.face2(e);
    }
    pa[0] = c_uv(mesh.corner(v2, ft));
    pa[1] = c_uv(cs1);
    pa[2] = c_uv(cs2);
    if (my_sqrt(area2(pa)) == 0.f) {
      // This shouldn't be necessary because "strict_sharp 2" also takes care of it
      Warning("Not valid APS - Area");
      return false;
    }
    Vector vv = cross(pa[0], pa[1], pa[2]);
    if (vv[2] < 0) {
      Warning("Not valid APS - Cross");
      return false;
    }
  }
  return true;
}

int inside_triangle(const Point& tc, const Vec3<Point>& cs) {
  return (cross(cs[1] - cs[0], tc - cs[0])[2] >= -.000001f && cross(cs[2] - cs[1], tc - cs[1])[2] >= -.000001f &&
          cross(cs[0] - cs[2], tc - cs[2])[2] >= -.000001f);
}

float aps_dist2_v1(Edge e, Vertex v1, Vertex v2) {
  float max_mag2 = -1.f;
  for (Face f : mesh.faces(v1)) {
    Face fout;
    if (f_matid(f) == f_matid(mesh.face1(e))) {
      fout = mesh.face1(e);
    } else {
      assertx(mesh.face2(e) && f_matid(f) == f_matid(mesh.face2(e)));
      fout = mesh.face2(e);
    }
    Vec3<Point> pt, pv;
    int i = 0;
    for (Vertex v : mesh.vertices(f)) {
      if (v == v1) {
        pv[i] = mesh.point(v2);
        pt[i] = c_uv(mesh.corner(v2, fout));
      } else {
        pv[i] = mesh.point(v);
        pt[i] = c_uv(mesh.corner(v, f));
      }
      i++;
    }
    if (inside_triangle(c_uv(mesh.corner(v1, f)), pt)) {
      Bary bary;
      Point clp;
      project_point_triangle2(c_uv(mesh.corner(v1, f)), pt[0], pt[1], pt[2], bary, clp);
      Point pe = interp(pv[0], pv[1], pv[2], bary);
      max_mag2 = max(max_mag2, dist2(pe, mesh.point(v1)));
    }
  }
  if (max_mag2 == -1.f) Warning("Vertex is not inside any new face in texture plane");
  return max_mag2;
}

// return: is_legal
bool strict_mat_neighbors(Face fc, Face fnei0, Face fnei1, Face fnei2) {
  assertx(fc);
  int matfc = f_matid(fc);
  int matf0 = fnei0 ? f_matid(fnei0) : -1;
  int matf1 = fnei1 ? f_matid(fnei1) : -1;
  int matf2 = fnei2 ? f_matid(fnei2) : -1;
  // If all neighbors different, face is either single chart or part of one adjacent chart but at a corner,
  // so all is OK.
  if (matf0 != matf1 && matf0 != matf2 && matf1 != matf2) return true;
  // If all neighbors same, face is not along any boundary, and is OK.
  if (matf0 == matf1 && matf0 == matf2 && matf1 == matf2) {
    if (matf0 == -1) {
      // OK to have single chart surrounded by surface boundaries
    } else {
      // cannot have isolated triangle chart without 3 corners.
      assertx(matfc == matf0);
    }
    return true;
  }
  // two neighbors are the same, and one is different.
  // then center face must have same material as the two same neighbors.
  if (matf0 == matf1) return matfc == matf0;
  if (matf0 == matf2) return matfc == matf0;
  if (matf1 == matf2) return matfc == matf1;
  assertnever("");
}

double evaluate_aps(Edge e, int ii) {
  Vertex v1 = (ii == 2) ? mesh.vertex2(e) : mesh.vertex1(e);
  Vertex v2 = (ii == 2) ? mesh.vertex1(e) : mesh.vertex2(e);
  // v1 is vertex removed
  if (!valid_aps(e, v1, v2)) return -1.;
  double max_mag2 = aps_dist2_v1(e, v1, v2);
  if (max_mag2 == -1.) return -1.;
  for (Vertex v : mesh.vertices(v1)) {
    if (v == v2) continue;
    {
      Edge ee = mesh.edge(v1, v);
      if (!mesh.face2(ee) || f_matid(mesh.face1(ee)) != f_matid(mesh.face2(ee))) continue;
    }
    Point p = c_uv(mesh.corner(v, mesh.face(v, v1)));
    Point p1 = c_uv(mesh.corner(v1, mesh.face(v, v1)));
    Point pv = mesh.point(v);
    for (Vertex vv : mesh.vertices(v1)) {
      if (vv == v2) continue;
      {
        Edge ee = mesh.edge(v1, vv);
        if (!mesh.face2(ee) || f_matid(mesh.face1(ee)) != f_matid(mesh.face2(ee))) continue;
      }
      Point p2;
      if (f_matid(mesh.face(v2, v1)) != f_matid(mesh.face(vv, v1))) {
        assertx(f_matid(mesh.face(v1, v2)) == f_matid(mesh.face(vv, v1)));
        p2 = c_uv(mesh.corner(v2, mesh.face(v1, v2)));
      } else {
        p2 = c_uv(mesh.corner(v2, mesh.face(v2, v1)));
      }
      Point pp = c_uv(mesh.corner(vv, mesh.face(vv, v1)));
      float ia, ib;
      if (intersect_lines(p1, p, p2, pp, ia, ib)) {
        Vector vec = (interp(mesh.point(v1), pv, ia) - interp(mesh.point(v2), mesh.point(vv), ib));
        max_mag2 = max(max_mag2, double(mag2(vec)));
      }
    }
  }
  return sqrt(max_mag2);
}

float trishape_quality(const Point& p0, const Point& p1, const Point& p2) {
  return pow(aspect_ratio(p0, p1, p2), trishapepow);
}

void write_mvertex(std::ostream& os, Vertex v, const char* sinfo) {
  const Point& p = mesh.point(v);
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ < 11
  // https://en.cppreference.com/w/cpp/compiler_support/17
  // GCC libstdc++ lacks float support for elementary string conversions prior to Version 11.
  os << "MVertex " << mesh.vertex_id(v) << "  " << p[0] << " " << p[1] << " " << p[2];
  if (sinfo) os << " {" << sinfo << "}";
  os << "\n";
#else
  constexpr int capacity = 80;
  char buffer[capacity];
  char* const end = buffer + capacity;
  strcpy(buffer, "MVertex ");
  char* s = buffer + 8;
  s = std::to_chars(s, end, mesh.vertex_id(v)).ptr;
  *s++ = ' ';
  for_int(c, 3) {
    *s++ = ' ';
    constexpr int precision = 6;  // Default for printf("%g").
    s = std::to_chars(s, end, p[c], std::chars_format::general, precision).ptr;
  }
  if (sinfo) {
    *s++ = ' ';
    *s++ = '{';
    assertx(s + strlen(sinfo) + 4 < end);
    while (*sinfo) *s++ = *sinfo++;
    *s++ = '}';
  }
  *s++ = '\n';
  *s = '\0';
  os << buffer;
#endif
}

void write_corner(std::ostream& os, Vertex v, Corner c) {
  const char* sinfo = mesh.get_string(c);
  assertx(sinfo);
  if (0) {
    os << "Corner " << mesh.vertex_id(v) << " " << mesh.face_id(mesh.corner_face(c)) << " {" << sinfo << "}\n";
  } else {
    constexpr int capacity = 160;
    char buffer[capacity];
    char* const end = buffer + capacity;
    strcpy(buffer, "Corner ");
    char* s = buffer + 7;
    s = std::to_chars(s, end, mesh.vertex_id(v)).ptr;
    *s++ = ' ';
    s = std::to_chars(s, end, mesh.face_id(mesh.corner_face(c))).ptr;
    *s++ = ' ';
    *s++ = '{';
    assertx(s + strlen(sinfo) + 4 < end);
    while (*sinfo) *s++ = *sinfo++;
    *s++ = '}';
    *s++ = '\n';
    *s = '\0';
    os << buffer;
  }
}

struct EcolResult {
  EResult result;
  float cost{k_bad_cost};
  int min_ii{-1};
  Vertex vs{nullptr};
};

// Consider the edge collapse of edge e.
EcolResult try_ecol(Edge e, bool commit) {
  EcolResult ecol_result;
  if (commit)
    ASSERTX(mesh.nice_edge_collapse(e));
  else if (!mesh.nice_edge_collapse(e))
    return {R_illegal};
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);                    // f2 could be nullptr
  Vertex vo1 = mesh.side_vertex1(e), vo2 = mesh.side_vertex2(e);  // vo2 could be nullptr
  int v1nse = vertex_num_sharpe(v1), v2nse = vertex_num_sharpe(v2);
  if (!try_ecol_legal(e, v1, v2, v1nse, v2nse)) return {R_illegal};
  NewMeshNei nn;
  if (!gather_nn(e, nn)) return {R_illegal};
  Dihedral dih;
  if (!terrain) dih = enter_dihedral(e, nn);
  BoundingSphere new_bsphere;
  dummy_init(new_bsphere);
  if (bspherefac) new_bsphere = bsphere_union(v_bsphere(v1), v_bsphere(v2));
  float spring = compute_spring(nn);
  double rssf;
  if (!relerror) {
    rssf = 0.;
  } else if (minvolume) {
    rssf = compute_volume_before(e);
    if (hull && innerhull) rssf = -rssf;
  } else if (minarea) {
    rssf = compute_area_before(e);
  } else if (minedgelength) {
    rssf = 0.;
  } else if (minvdist || terrain || minqem || minaps || minrandom) {
    assertnever("never use relerror");
  } else {
    assertx(!maxerr);
    double rssfg = 0., rssfc = 0., rssfn = 0.;
    for (fptinfo* pfpt : nn.ar_fpts) rssfg += pfpt->dist2;
    for (eptinfo* pept : nn.ar_epts) rssfg += pept->dist2;
    if (spring) {
      for (Vertex v : mesh.vertices(e)) {
        for (Edge ee : mesh.edges(v)) {
          if (v == v2 && ee == e) continue;  // count e exactly once
          rssfg += spring * mesh.length2(ee);
        }
      }
    }
    if (have_ccolors && gcolc) {
      for (fptinfo* pfpt : nn.ar_fpts) rssfc += pfpt->coldist2();
      rssfc *= gcolc;
    }
    if (have_cnormals && gnorc) {
      for (fptinfo* pfpt : nn.ar_fpts) rssfn += pfpt->nordist2();
      rssfn *= gnorc;
    }
    rssf = rssfg + rssfc + rssfn;
  }
  {
    float rssfpenalty = 0.f;
    for (eptinfo* pept : nn.ar_eptretire) {
      dummy_use(pept);
      // do not create benefit! pept->dist2 could be > length2(e)!
      //  not: rssfg += pept->dist2;
      rssfpenalty += mesh.length2(e);  // penalty for retirement
    }
    if (desdfac) rssfpenalty += square(gdiam * desdfac) * abs(v_desh(v1) - v_desh(v2));
    if (desnfac) rssfpenalty += square(gdiam * desnfac) * (v_desn(v1) + v_desn(v2));
    rssf -= rssfpenalty;
  }
  if (minqem) gather_nn_qem(e, nn);
  // Consider 3 optimizations with different starting locations.
  double min_rssa = BIGFLOAT;
  int min_ii = -1;
  Point min_p;
  Array<WedgeInfo> min_ar_wi;
  float min_dir_error;
  dummy_init(min_dir_error);
  bool have_fakeii = (minqem && !no_fit_geom) || hull;
  for_int(fakeii, 3 + have_fakeii) {
    int ii = fakeii;
    bool strict_ii = false;
    if (fakeii == 3) {
      ii = 1;
      if (!edge_sharp(e)) {
        if (v1nse >= 1 && v2nse >= 1) {
        } else if (v1nse >= 1) {
          ii = 2;
          strict_ii = true;
        } else if (v2nse >= 1) {
          ii = 0;
          strict_ii = true;
        }
      } else {
        if (v1nse >= 3 && v2nse >= 3) {
        } else if (v1nse >= 3) {
          ii = 2;
          strict_ii = true;
        } else if (v2nse >= 3) {
          ii = 0;
          strict_ii = true;
        }
      }
    }
    if (have_fakeii && fakeii < 3 && minqem && qemvolume) continue;
    if (hull && fakeii < 3) continue;
    if (minii1 && ii != 1) continue;
    if (nominii1 && ii == 1) continue;
    // Added these tests to speed up search by not considering cases that lead to crease and corner migration.
    if (minii2) {
      if (ii == 1) continue;
      if (mesh.is_boundary(e)) {
        if (ii != 2) continue;
        if (terrain) {
          if (is_grid_corner(v2)) continue;
        } else {
          // new 1998-01-05 for SR_PREDICT_MATID; or should we use clw_edge? double check.
          // 2004-03-01: removed to allow simplification of planck400k to 2 triangles with 4 vertices tagged global.
          // 2004-07-13: reintroduced because otherwise ctfparam can crash.
          if (1 && mesh.is_boundary(mesh.ccw_edge(f1, e))) continue;
        }
      } else {
        if (mesh.is_boundary(v1) && ii != 2) continue;
        if (mesh.is_boundary(v2) && ii != 0) continue;
      }
    }
    if (keepglobalv && ii != 2 && v_global(v1)) {
      if (0) Warning("Keeping global vertex");
      continue;
    }
    if (keepglobalv && ii != 0 && v_global(v2)) {
      if (0) Warning("Keeping global vertex");
      continue;
    }
    if (invertexorder) {
      if (0) {
      } else if (mesh.vertex_id(v1) == mesh.num_vertices()) {
        if (ii != 0) continue;
      } else if (mesh.vertex_id(v2) == mesh.num_vertices()) {
        if (ii != 2) continue;
      } else {
        assertnever("");
      }
    }
    if (!edge_sharp(e)) {
      if (v1nse >= 1 && v2nse >= 1) {
        assertx(!strict_sharp);
        // Try all 3 ii's but penalize later.
      } else if (v1nse >= 1) {
        if (ii < 2 && !minii1) continue;
      } else if (v2nse >= 1) {
        if (ii > 0 && !minii1) continue;
      } else if (minqem) {
        // do consider all 3 ii's and pick best one.
      } else {
        if (ii != 1 && !miniiall && !nominii1 && !minii2) continue;
      }
    } else {
      if (v1nse >= 3 && v2nse >= 3) {
        assertx(!strict_sharp);
        // Try all 3 ii's but penalize later.
      } else if (v1nse >= 3) {
        if (ii < 2 && !minii1) continue;
      } else if (v2nse >= 3) {
        if (ii > 0 && !minii1) continue;
      } else {
        if (0 && !mesh.is_boundary(e)) {
          if (ii != 1 && !miniiall && !nominii1 && !minii2) continue;
        }
        // Try all 3 ii's in order to preserve discontinuity
        //  curve corners (not just boundary curve corners).
      }
    }
    if (strict_sharp >= 2) {  // chart-compliant
      assertx(ii != 1);       // 0 or 2
      Vertex tv1 = v1, tv2 = v2;
      if (ii == 0) std::swap(tv1, tv2);
      // tv1 is kept, tv2 is lost
      const bool ok = [&] {
        for (Face f : mesh.faces(e)) {
          Vertex vv = mesh.opp_vertex(e, f);  // side_vertex1|2
          Face fc = mesh.opp_face(f, mesh.edge(tv2, vv));
          // if (v2, vv) is boundary, ecol should be disallowed
          assertx(fc);
          assertx(f_matid(fc) == f_matid(f));
          Vertex vvv = mesh.opp_vertex(mesh.edge(tv2, vv), fc);
          // note: Face fc is (tv2, vv, vvv)
          Face fnei0 = mesh.opp_face(fc, mesh.edge(tv2, vvv));
          Face fnei1 = mesh.opp_face(fc, mesh.edge(vv, vvv));
          Face fnei2 = mesh.opp_face(f, mesh.edge(tv1, vv));
          if (!strict_mat_neighbors(fc, fnei0, fnei1, fnei2)) return false;  // From lambda.
        }
        // Gather materials adjacent to v1
        Set<int> v1mats;
        for (Face f : mesh.faces(tv1)) v1mats.add(f_matid(f));
        // Check whether any other vertex around v1 is on the same boundary as v1
        for (Vertex v : mesh.vertices(tv2)) {
          if (v == tv1 || v == mesh.side_vertex1(e) || v == mesh.side_vertex2(e)) continue;
          Edge ee = mesh.edge(tv2, v);
          int imat = f_matid(mesh.face1(ee));
          if (mesh.face2(ee) && f_matid(mesh.face2(ee)) == imat) {
            for (Face f : mesh.faces(v))
              if (f_matid(f) != imat && v1mats.contains(f_matid(f))) return false;  // From lambda.
          }
        }
        return true;  // From lambda.
      }();
      if (!ok) continue;
    }
    if (keepuvcorners) {
      if (is_uv_corner(mesh.corner(v1, f1)) && ii != 2) continue;
      if (is_uv_corner(mesh.corner(v2, f1)) && ii != 0) continue;
    }
    if (sphericalparam) {
      assertx(minii2);
      // See if new faces are all valid on sphere.
      const bool ok = [&]() {
        Polygon poly(3);
        Vertex tv1 = ii == 2 ? v1 : v2;
        for_int(i, nn.va.num() - 1) {
          poly[0] = v_sph(nn.va[i]);
          poly[1] = v_sph(nn.va[i + 1]);
          poly[2] = v_sph(tv1);
          if (spherical_triangle_area(poly) >= TAU) return false;
        }
        return true;
      }();
      if (!ok) continue;
    }
    Point newp = interp(mesh.point(v1), mesh.point(v2), ii * .5f);
    Array<WedgeInfo> ar_wi;  // indexed by nn.ar_nwid
    update_initial_wi(e, nn, ii, ar_wi);
    if (hull) {
      if (!compute_hull_point(e, nn, newp)) continue;
      // Estimate ii from v1, v2, newp
      if (!strict_ii) ii = estimate_ii(v1, v2, newp);
      SSTATV2(Sestii, ii);
      update_initial_wi(e, nn, ii, ar_wi);
      if (attrib_project) perform_attrib_project(e, newp, ar_wi);
    }
    float dihpenalty = 0.f;
    if ((!terrain && fakeii < 3) || hull) {
      dihpenalty = dihedral_penalty(dih, nn, newp);
      if (dihpenalty == BIGFLOAT) continue;
    }
    if (poszfacenormal) {
      const bool ok = [&]() {
        Polygon poly(3);
        for_int(i, nn.va.num() - 1) {
          poly[0] = mesh.point(nn.va[i]);
          poly[1] = mesh.point(nn.va[i + 1]);
          poly[2] = newp;
          Vector normaldir = poly.get_normal_dir();
          if (normaldir[2] < 0.f) return false;
        }
        return true;
      }();
      if (!ok) continue;
    }
    float dir_error = 0.f;
    double rssa;
    if (minvolume) {
      rssa = compute_volume_after(nn, newp);
      if (hull && innerhull) rssa = -rssa;
    } else if (minarea) {
      rssa = compute_area_after(nn, newp);
      // Truncate this to ignore increases in area
      // if (rssa<rssf) rssa = rssf;
    } else if (minedgelength) {
      rssa = mesh.length2(e);
    } else if (minvdist) {
      Warning("minvdist was found to be a bad idea");
      Face cf;
      Bary bary;
      Point clp;
      project_vnew_nn(e, newp, cf, bary, clp);
      rssa = dist2(newp, clp);
    } else if (terrain) {
      assertw(!gcolc && !gnorc && !spring);
      rssa = evaluate_terrain_resid(nn, newp);
      if (rssa == k_bad_cost) continue;
      dir_error = float(my_sqrt(rssa));
      if (mresid && dir_error > mresid) continue;
    } else if (minqem) {
      LinearFunc lfvol(Vector(0.f, 0.f, 0.f), Point(0.f, 0.f, 0.f));
      bool lfvol_ok = true;
      if (fakeii == 3 && qemvolume) {
        Polygon poly;
        for (Vertex v : mesh.vertices(e)) {
          for (Face f : mesh.faces(v)) {
            if (v == v2 && (f == f1 || f == f2)) continue;
            mesh.polygon(f, poly);
            Vector vc = cross(poly[0], poly[1], poly[2]);
            LinearFunc lf1(vc, poly[0]);
            lfvol.add(lf1);
          }
        }
        {
          float sum2 = mag2(lfvol.v);
          if (sum2 < square(gdiam * 1e-6f)) {  // was 1e-4f
            lfvol_ok = false;
          } else {
            float fac = 1.f / sqrt(sum2);
            lfvol.v *= fac;
            lfvol.offset *= fac;
          }
          SSTATV2(Slfvol_ok, lfvol_ok);
        }
      }
      int nw = ar_wi.num();
      Matrix<float> minp(nw, k_qemsmax);
      for_int(i, nw) create_qem_vector(newp, ar_wi[i], minp[i]);
      if (fakeii == 3) {
        // If colfac && !fit_colors, or if norfac && !fit_normals, should compute minimum subject to those constraints.
        const float small_constr_cweight = 1e-3f;
        float cweight = small_constr_cweight;
        // if (qemweight) cweight *= square(gdiam*.05f);
        auto up_qbu0 = make_qem();
        BQemT& qbu0 = *up_qbu0;
        qbu0.copy(*nn.ar_wq[0]);
        int ncwi = 0;
        for (;; ncwi++) {
          bool success;
          if (qemvolume && lfvol_ok) {
            Vec<float, k_qemsmax + 1> constr;
            for_int(c, k_qemsmax) constr[c] = 0.f;
            for_int(c, 3) constr[c] = lfvol.v[c];
            constr[qems] = lfvol.offset;
            // auto constrv = ArView(constr, qems + 1);
            if (1 && nw == 1) {
              success = nn.ar_wq[0]->fast_minp_constr_lf(minp[0].data(), constr.data());
              // 2005-10-19 sometimes never returns success if qemvolume == 1 and mesh has boundaries.
            } else {
              success = nn.ar_wq[0]->ar_compute_minp_constr_lf(nn.ar_wq, minp, constr.data());
            }
          } else {
            success = nn.ar_wq[0]->ar_compute_minp(nn.ar_wq, minp);
          }
          if (success) break;
          auto up_qpd2 = make_qem();
          BQemT& qpd2 = *up_qpd2;
          qpd2.set_d2_from_point(minp[0].data());
          qpd2.scale(cweight);
          nn.ar_wq[0]->add(qpd2);
          cweight *= 10.f;
          if (ncwi >= 20) {
            Warning("Assuming current attributes are just fine");
            break;
          }
        }
        if (ncwi) {
          SSTATV2(Sncwi, ncwi);
          nn.ar_wq[0]->copy(qbu0);
        }
        {
          WedgeInfo dummy_wi;
          extract_qem_vector(minp[0].data(), newp, dummy_wi);
          dihpenalty = dihedral_penalty(dih, nn, newp);
          if (dihpenalty == BIGFLOAT) continue;
          if (!strict_ii) ii = estimate_ii(v1, v2, newp);
          SSTATV2(Sestii, ii);
          update_initial_wi(e, nn, ii, ar_wi);
          if (attrib_project) perform_attrib_project(e, newp, ar_wi);
        }
        if (fit_colors || fit_normals) {
          for_int(i, nw) {
            Point dummy_p;
            WedgeInfo wi;
            extract_qem_vector(minp[i].data(), dummy_p, wi);  // just want wi
            assertx(dummy_p == newp);
            if (fit_colors) {
              for_int(c, 3) {
                float v = wi.col[c];
                if (v < 0.f) {
                  SSTATV2(Snewcs, v);
                  v = 0.f;
                }
                if (v > 1.f) {
                  SSTATV2(Snewcl, v);
                  v = 1.f;
                }
                ar_wi[i].col[c] = v;
              }
            }
            if (fit_normals) {
              wi.nor.normalize();  // will be checked later
              ar_wi[i].nor = wi.nor;
            }
          }
        }
        for_int(i, nw) create_qem_vector(newp, ar_wi[i], minp[i]);
      }  // End of "if (fakeii == 3)".
      {
        double v = 0.;
        for_int(i, nw) v += nn.ar_wq[i]->evaluate(minp[i].data());
        rssa = max(v, 0.);
      }
    } else if (minaps) {
      assertx(ii == 0 || ii == 2);
      // ii == 2 means new vertex equals mesh.vertex1(e)
      rssa = evaluate_aps(e, ii);
      if (rssa == -1.) continue;
    } else if (minrandom) {
      rssa = Random::G.unif();
    } else {
      double rssag = 0., rssac = 0., rssan = 0., rssapenalty = 0.;
      if (!edge_sharp(e) && v1nse >= 1 && v2nse >= 1) {
        rssapenalty += mesh.length2(e) * nn.ar_epts.num();
      } else if (edge_sharp(e) && v1nse >= 3 && v2nse >= 3) {
        rssapenalty += mesh.length2(e) * nn.ar_epts.num();
      }
      Param param;
      project_fpts(nn, newp, param);
      const bool optimize_geom = !no_fit_geom && !hull && !(keepglobalv && (v_global(v1) || v_global(v2)));
      if (optimize_geom) {
        rssag = fit_geom(nn, param, spring, newp);
        dihpenalty = dihedral_penalty(dih, nn, newp);
        if (dihpenalty == BIGFLOAT) continue;
        if (attrib_project) perform_attrib_project(e, newp, ar_wi);
        if (fit_colors) project_fpts(nn, newp, param);
      } else {
        rssag = evaluate_geom(nn, param, spring, newp);
      }
      if (maxerr) rssag = evaluate_maxerr(nn, param, newp);
      if (have_ccolors && fit_colors) {
        rssac = fit_color(nn, param, ar_wi);
      } else if (have_ccolors && gcolc) {
        Warning("Not tested");
        rssac = evaluate_color(nn, param, ar_wi);
      }
      if (have_cnormals && gnorc) rssan = evaluate_normal(nn, param, ar_wi);
      rssa = rssag + rssac + rssan + rssapenalty;
    }
    if (bspherefac) {
      float new_radius = dist(newp, new_bsphere.point) + new_bsphere.radius;
      rssa += square(bspherefac * new_radius);
    }
    if (edgepathfac) {
      assertx(ii == 0 || ii == 2);
      // if ii == 0, vertex kept is v2
      // if ii == 2, vertex kept is v1
      for (Vertex vv : mesh.vertices(ii == 0 ? v1 : v2)) {
        if (vv == v1 || vv == v2 || vv == vo1 || vv == vo2) continue;
        // loop over vertices vv adjacent to v1|v2
        float area = sqrt(area2(mesh.point(v1), mesh.point(v2), mesh.point(vv)));
        rssa += edgepathfac * area;
      }
    }
    if (trishapefac) {
      for_int(i, nn.va.num() - 1) {
        const Point& p0 = mesh.point(nn.va[i]);
        const Point& p1 = mesh.point(nn.va[i + 1]);
        const Point& p2 = newp;
        rssa += trishapefac * trishape_quality(p0, p1, p2) * square(gdiam);
      }
    }
    if (trishapeafac) {
      for_int(i, nn.va.num() - 1) {
        const Point& p0 = mesh.point(nn.va[i]);
        const Point& p1 = mesh.point(nn.va[i + 1]);
        const Point& p2 = newp;
        float triarea = sqrt(area2(p0, p1, p2));
        rssa += trishapeafac * trishape_quality(p0, p1, p2) * triarea;
      }
    }
    if (bndfac) {
      assertx(!rssf);
      if (mesh.is_boundary(e)) {
        rssa /= bndfac;
        if (dihpenalty) {
          rssa *= sqrt(bndfac);
          dihpenalty = 0.f;
        }
      } else {
        if (maxvalence && ((vo1 && mesh.degree(vo1) > maxvalence / 2) || (vo2 && mesh.degree(vo2) > maxvalence / 2)))
          rssa /= bndfac;
      }
    }
    assertx(rssa < k_bad_dih * .1f);
    rssa += dihpenalty;
    assertx(std::isfinite(float(rssa)));
    if (any_of(ar_wi, [&](const WedgeInfo& wi) { return is_zero(wi.nor); })) {
      Warning("Edge collapse would introduce zero normal");
      continue;
    }
    if (rssa < min_rssa) {
      min_rssa = rssa;
      min_ii = ii;
      min_p = newp;
      min_ar_wi.init(0);
      for (const WedgeInfo& wi : ar_wi) min_ar_wi.push(wi);
      min_dir_error = dir_error;
    }
  }
  if (min_ii < 0) return {R_dih};  // No dihedrally admissible configuration.
  float raw_cost;
  raw_cost = float(min_rssa - rssf);
  ecol_result.cost = raw_cost + offset_cost;
  if (raw_cost < 0) SSTATV2(Snegcost, raw_cost);
  const float smallcost = 1e-20f;
  if (ecol_result.cost < smallcost) ecol_result.cost = smallcost;
  if (invertexorder == 2) {
    // Of the legal edge collapses, select the one whose opposite vertex has smallest id.
    Vertex vo = mesh.opp_vertex(mesh.id_vertex(mesh.num_vertices()), e);
    int voi = mesh.vertex_id(vo);
    ecol_result.cost = float(voi);
  }
  ecol_result.result = R_success;
  if (!commit) return ecol_result;

  // ALL SYSTEMS GO.
  if (verb >= 3) SHOW("ecol:", rssf, min_rssa, raw_cost);
  if (wfile_prog) g_necols++;
  int new_desn = v_desn(v1) + v_desn(v2);
  int new_desh = max(v_desh(v1), v_desh(v2)) + 1;
  if (desdfac || desnfac) {
    SSTATV2(Sdesn, new_desn);
    SSTATV2(Sdesh, new_desh);
    SSTATV2(Sdesd, abs(v_desn(v1) - v_desn(v2)));
  }
  if (!strict_sharp) {
    if (!edge_sharp(e)) {
      if (v1nse >= 1 && v2nse >= 1)
        if (verb >= 2) Warning("Edge collapse offends sharp edges (a)");
    } else {
      if (v1nse >= 3 && v2nse >= 3)
        if (verb >= 2) Warning("Edge collapse offends sharp edges (b)");
      for (Face f : mesh.faces(e)) {
        Vertex vs = mesh.opp_vertex(e, f);
        if (edge_sharp(mesh.edge(v1, vs)) && edge_sharp(mesh.edge(v2, vs)))
          if (verb >= 2) Warning("Edge collapse offends sharp edges (c)");
      }
      if ((v1nse == 1 && v2nse != 2) || (v2nse == 1 && v1nse != 2))
        if (verb >= 2) Warning("Edge collapse causes dart to vanish");
    }
  }
  // Consider swapping the orientation of the edge.
  bool bswap = false;
  // Old test: encouraged min_ii == 2 if possible:
  // if (f2 && min_ii == 0) bswap = true;
  // New test: encourage short CLW rotation from vl to vr:
  if (f2) {
    Vertex vs = mesh.vertex1(e), vt = mesh.vertex2(e);
    Vertex vl = mesh.side_vertex1(e), vr = mesh.side_vertex2(e);
    int dir = -1;  // 0 == CCW, 1 == CLW
    int jmin = std::numeric_limits<int>::max();
    Vec<bool, 2> ar_ok = {false, false};
    for_int(i, 2) {
      if (minii2) {
        if (min_ii == 0 && i == 1) continue;  // force swap
        if (min_ii == 2 && i == 0) continue;  // disallow swap
      }
      Vertex v = vl;
      int j = 0;
      for (;;) {
        j++;
        v = !i ? mesh.ccw_vertex(vs, v) : mesh.clw_vertex(vt, v);
        assertx(v != vl);
        if (!v || v == vr) break;
      }
      if (v != vr) continue;
      ar_ok[i] = true;
      if (j <= jmin) {
        jmin = j;
        dir = i;
      }
    }
    assertx(dir >= 0 && dir <= 1);
    assertx(jmin >= 1);
    if (dir == 0) bswap = true;
    if (tvcfac) {
      float tvccost;
      bool edir;
      get_tvc_cost_edir(e, tvccost, edir);
      assertx(ar_ok[edir ? 0 : 1]);
      bswap = edir;
      SSTATV2(Sswap, bswap);
    }
  }
  // bswap = false; if (f2 && min_ii == 0) bswap = true;
  Vertex vs = !bswap ? v1 : v2;
  Vertex vt = !bswap ? v2 : v1;
  Vertex vl = !bswap ? vo1 : vo2;
  Vertex vr = !bswap ? vo2 : vo1;
  Face fl = !bswap ? f1 : f2;
  Face fr = !bswap ? f2 : f1;
  SSTATV2(Sminii0, min_ii);
  if (bswap) min_ii = 2 - min_ii;
  if (minii2) assertx(min_ii == 2);
  SSTATV2(Sminii1, min_ii);
  ecol_result.min_ii = min_ii;
  ecol_result.vs = vs;
  CArrayView<int> ar_rwid = !bswap ? nn.ar_rwid_v1 : nn.ar_rwid_v2;
  // v1 = v2 = nullptr; vo1 = vo2 = nullptr; f1 = f2 = nullptr;  // now undefined
  if (!minqem) {
    for (eptinfo* pept : nn.ar_epts) point_change_edge(pept, nullptr);
    for (eptinfo* pept : nn.ar_eptretire) {
      point_change_edge(pept, nullptr);
      pept->dist2 = 0.f;
    }
    for (Vertex v : mesh.vertices(e))
      for (Edge ee : mesh.edges(v)) assertx(e_setpts(ee).empty());
    for (fptinfo* pfpt : nn.ar_fpts) point_change_face(pfpt, nullptr);
    for (Face f : mesh.faces(e)) assertx(f_setpts(f).empty());
  }
  if (wfile_prog) {
    std::ostream& os = (*wfile_prog)();
    os << "# Beg REcol\n";
    // adapted from write_mesh() and GMesh::write():
    string str;
    for_int(i, 2) {
      Face f = !i ? fl : fr;
      if (!f) {
        assertx(i == 1);
        continue;
      }
      Vertex vo = mesh.opp_vertex(e, f);
      create_vertex_corner_strings(vo, str, true);
      Corner c = mesh.corner(vo, f);
      const char* sinfo = mesh.get_string(c);
      if (sinfo)
        os << "Corner " << mesh.vertex_id(vo) << " " << mesh.face_id(mesh.corner_face(c)) << " {" << sinfo << "}\n";
      clear_vertex_corner_strings(vo);
    }
    for_int(i, 2) {
      Vertex v = !i ? vs : vt;
      create_vertex_corner_strings(v, str, true);
      write_mvertex(os, v, nullptr);
      for (Corner c : mesh.corners(v)) write_corner(os, v, c);
      clear_vertex_corner_strings(v);
    }
    for_int(i, 2) {
      Face f = !i ? fl : fr;
      if (!f) {
        assertx(i == 1);
        continue;
      }
      create_face_string(f);
      os << "Face " << mesh.face_id(f) << " ";
      for (Vertex v : mesh.vertices(f)) os << " " << mesh.vertex_id(v);
      const char* sinfo = mesh.get_string(f);
      if (sinfo) os << " {" << sinfo << "}";
      os << "\n";
      clear_face_string(f);
    }
    os << sform("REcol %d  %d %d  %d %d  %d %d  %d\n",  //
                mesh.vertex_id(vs), mesh.vertex_id(vs), mesh.vertex_id(vt), (vl ? mesh.vertex_id(vl) : 0),
                (vr ? mesh.vertex_id(vr) : 0), mesh.face_id(fl), (fr ? mesh.face_id(fr) : 0), min_ii);
  }
  if (minqem) {
    if (qemlocal) {
      if (qemcache) {
        // for (Face f : mesh.faces(e)) { delete f_qem_p(f); f_qem_p(f) = nullptr; }
        // remaining affected faces are updated after edge collapse
      }
    } else {
      for_int(i, ar_rwid.num()) {
        int rwid = ar_rwid[i];
        assertx(rwid == (!bswap ? nn.ar_rwid_v1[i] : nn.ar_rwid_v2[i]));
        int orwid = !bswap ? nn.ar_rwid_v2[i] : nn.ar_rwid_v1[i];
        if (orwid != rwid) gwq[orwid] = nullptr;
      }
      // remaining affected gwq are updated after edge collapse
    }
  }
  if (tvcfac) tvc_update_cache(e);
  // ***DO IT
  mesh.collapse_edge_vertex(e, vs);  // vs kept
  mesh.set_point(vs, min_p);
  replace_wi(nn, min_ar_wi, ar_rwid);
  v_desn(vs) = new_desn;
  v_desh(vs) = new_desh;
  if (bspherefac) v_bsphere(vs) = new_bsphere;
  if (minqem) {
    if (qemlocal) {
      if (qemcache)
        for (Face f : mesh.faces(vs)) get_face_qem(f, f_qem(f));
    } else {
      for_int(i, ar_rwid.num()) gwq[ar_rwid[i]]->copy(*nn.ar_wq[i]);
    }
  }
  // sanity checks
  if (minaps) check_ccw(vs);
  if (k_debug) {
    {
      for_int(i, nn.ar_corners.num()) {
        Corner c = nn.ar_corners[i][2];
        assertx(mesh.corner_vertex(c) == vs);
        mesh.valid(c);
        assertx(nn.ar_corners[i][0] == mesh.ccw_face_corner(c));
        assertx(nn.ar_corners[i][1] == mesh.clw_face_corner(c));
        assertx(mesh.corner_vertex(nn.ar_corners[i][0]) == nn.va[i]);
        assertx(mesh.corner_vertex(nn.ar_corners[i][1]) == nn.va[i + 1]);
      }
      assertx(nn.ar_corners.num() == mesh.degree(vs) - mesh.is_boundary(vs));
    }
    {
      Set<Vertex> setv;
      bool closed = nn.va[0] == nn.va.last();
      for_int(i, nn.va.num() - closed) {
        Vertex v = nn.va[i];
        assertx(setv.add(v));
        assertx(mesh.query_edge(vs, v));
      }
      assertx(nn.va.num() - closed == mesh.degree(vs));
    }
    {
      bool closed = nn.va[0] == nn.va.last();
      for_int(i, nn.va.num() - closed) {
        bool found = nn.ar_vdisc.contains(i);
        Edge ee = mesh.edge(vs, nn.va[i]);
        assertx(found == edge_sharp(ee));
      }
    }
  }
  float uni_error, dir_error;
  if (terrain) {
    uni_error = 0.f;
    dir_error = min_dir_error;
  } else {
    reproject_locally(nn, uni_error, dir_error);
  }
  if (wfile_prog) {
    std::ostream& os = (*wfile_prog)();
    // Faces are gone.
    string str;
    create_vertex_corner_strings(vs, str, true);
    write_mvertex(os, vs, mesh.get_string(vs));
    for (Corner c : mesh.corners(vs)) write_corner(os, vs, c);
    os << sform("# Residuals %g %g\n", uni_error, dir_error);
    os << sform("# End REcol\n");
    assertx(os);
    clear_vertex_corner_strings(vs);
  }
  if (k_debug) {
    for (fptinfo* pfpt : nn.ar_fpts) mesh.valid(assertx(pfpt->cmf));
    for (eptinfo* pept : nn.ar_epts) {
      if (!pept->cme) continue;
      mesh.valid(assertx(pept->cme));
    }
  }
  if (strict_sharp >= 2) {
    // Check that there are no degenerate faces.
    // A face is degenerate if it has all 3 vertices on chart boundary,
    //  or equivalently that two of its neighbors have the same matid
    //   but that matid is different from its current matid.
    for (Face f : mesh.faces(vs)) {
      Vec3<Face> fn;
      int i = 0;
      if (0) {
        for (Face ff : mesh.faces(f)) fn[i++] = ff;
      } else {
        for (Vertex v : mesh.vertices(f)) fn[i++] = mesh.opp_face(v, f);  // fn[i] can be nullptr
      }
      // Can occur if at corner of terrain grid && !terrain
      assertw(strict_mat_neighbors(f, fn[0], fn[1], fn[2]));
    }
    for (Edge ee : mesh.edges(vs)) {
      int imat = f_matid(mesh.face1(ee));
      if (!mesh.face2(ee) || f_matid(mesh.face2(ee)) != imat) continue;
      Vertex vv1 = mesh.vertex1(ee);
      Vertex vv2 = mesh.vertex2(ee);
      Set<int> v1mats;
      for (Face f : mesh.faces(vv1))
        if (f_matid(f) != imat) v1mats.add(f_matid(f));
      for (Face f : mesh.faces(vv2)) assertx(!v1mats.contains(f_matid(f)));
    }
  }
  return ecol_result;
}

float get_tvc_cost(Edge e, bool edir) {
  if (!edir) {
    if (!mesh.is_boundary(e) && mesh.is_boundary(mesh.vertex2(e))) return BIGFLOAT;  // illegal in this direction
  } else {
    if (mesh.is_boundary(e) || mesh.is_boundary(mesh.vertex1(e))) return BIGFLOAT;  // illegal in this direction
  }
  Array<CacheEntry> ar_ce = tvc_get_e_cacheentry(e, edir);
  int nincache = 0;
  // Simulate LRU cache (even though ideally it should be FIFO).
  // Final PM order:  { facel , facer , cached_vertices } ,
  // so access the face corners in reverse order, looking into cached_vertices, accounting for cache misses so far.
  for_int(ii, ar_ce.num()) {
    const CacheEntry& ce = ar_ce[ar_ce.num() - 1 - ii];
    int wi = ce.wid, owi = ce.owid;
    int nfound = 0;
    for_int(j, tvc_cache.num() - (ii - nincache)) {
      const CacheEntry& ce2 = tvc_cache[j];
      if (!tvcowid)
        nfound += ce2.wid == wi;
      else
        nfound += ce2.owid == owi;
    }
    assertx(nfound <= 1);
    nincache += nfound;
  }
  SSTATV2(Snincache, nincache);
  if (tvc_maxcare) {
    // Heuristic: ignore the fact that more than maxcare are in cache.
    if (nincache >= tvc_maxcare) nincache = tvc_maxcare;
    SSTATV2(Snincache1, nincache);
  }
  assertx(nincache <= tvc_max_improvement);
  float cost;
  if (tvcpqa) {
    float avg_cost = !pqecost.total_num() ? 0.f : float(pqecost.total_priority() / pqecost.total_num());
    cost = -nincache * avg_cost * tvcfac;
  } else {
    // Scale by squared object size to achieve scale invariance.
    cost = -nincache * square(gdiam * tvcfac);
  }
  return cost;
}

void get_tvc_cost_edir(Edge e, float& tvccost, bool& edir) {
  float tvccost0 = get_tvc_cost(e, false);
  float tvccost1 = get_tvc_cost(e, true);
  if (tvccost0 == tvccost1) {
    tvccost = tvccost0;
    edir = false;
    if (1) edir = ((mesh.vertex_id(mesh.vertex1(e)) + mesh.vertex_id(mesh.vertex2(e))) & 1) != 0;
  } else if (tvccost0 < tvccost1) {
    tvccost = tvccost0;
    edir = false;
  } else {
    tvccost = tvccost1;
    edir = true;
  }
}

// Status: edefault is the best edge in pqecost, with cost costdefault.
// See if there are any better candidate edges when taking into account tvc_cache.
// If there is a better candidate, return it as edefault.
void consider_tvc(Edge& edefault, float costdefault) {
  Edge ebest = nullptr;
  float costbest = costdefault;
  Set<Edge> sete;
  for (const CacheEntry& ce : tvc_cache) {
    int wid = ce.wid, owid = ce.owid;
    assertx((wid < 0) == (owid < 0));
    if (wid < 0) continue;
    Vertex v = ce.v;
    assertx(mesh.valid(v));
    for (Corner c : mesh.corners(v)) {
      if ((!tvcowid && c_wedge_id(c) == wid) || (tvcowid && c_tvc_owid(c) == owid)) {
        Face f = mesh.corner_face(c);
        for (Edge e : mesh.edges(f)) sete.add(e);
      }
    }
  }
  SSTATV2(Stvcsete, sete.num());
  for (Edge e : sete) {
    float cost = pqecost.retrieve(e);
    ASSERTX(cost >= 0.f);
    // (cost could be k_bad_cost)
    float max_improvement =
        (tvc_max_improvement *
         (tvcpqa ? (!pqecost.total_num() ? 0.f : float(pqecost.total_priority() / pqecost.total_num()))
                 : square(gdiam * tvcfac)));
    if (cost - max_improvement > costbest) continue;
    float tvc_cost;
    bool edir;
    get_tvc_cost_edir(e, tvc_cost, edir);
    float adjusted_cost = cost + tvc_cost;
    if (adjusted_cost > costbest) continue;
    assertx(affectpq >= 3);
    costbest = adjusted_cost;
    ebest = e;
  }
  if (!ebest || ebest == edefault) return;
  edefault = ebest;
  costdefault = pqecost.retrieve(edefault);
  assertx(costdefault >= 0.f && costdefault != k_bad_cost);
}

float fractional_progress(int orig_nfaces, int orig_nvertices) {
  if (nfaces) return (orig_nfaces - float(mesh.num_faces())) / max(1.f, orig_nfaces - float(nfaces));
  return (orig_nvertices - float(mesh.num_vertices())) / max(1.f, orig_nvertices - float(nvertices));
}

// Simplify the mesh until it has <= nfaces or <= nvertices.
void parallel_optimize() {
  HH_STIMER("_parallel_opt");
  const int orig_nfaces = mesh.num_faces(), orig_nvertices = mesh.num_vertices();
  ConsoleProgress cprogress;
  for (;;) {
    if (verb == 1) cprogress.update(fractional_progress(orig_nfaces, orig_nvertices));
    if (mesh.num_faces() <= nfaces || mesh.num_vertices() <= nvertices) break;

    struct EdgeCost {
      Edge e;
      float cost;
    };
    Array<EdgeCost> ar_edgecost{transform(mesh.edges(), [&](Edge e) { return EdgeCost{e, 0.f}; })};

    {
      HH_STIMER("__opt_cost");
      parallel_for_each(range(ar_edgecost.num()), [&](int index) {
        Edge e = ar_edgecost[index].e;
        const EcolResult ecol_result = try_ecol(e, false);
        ar_edgecost[index].cost = ecol_result.cost;
      });
    }
    {
      HH_STIMER("__opt_sort");
      const auto by_increasing_cost = [&](auto& ec1, auto& ec2) { return ec1.cost < ec2.cost; };
      sort(ar_edgecost, by_increasing_cost);
    }

    HH_STIMER("__opt_ecols");
    Set<Edge> invalidated_edges;
    const float k_fraction_edges = 0.15f;
    int num_edges_considered = 0, num_edges_collapsed = 0;
    for_int(index, ar_edgecost.num()) {
      if (num_edges_collapsed > 0 && index > int(k_fraction_edges * ar_edgecost.num())) break;
      const auto& [e, cost] = ar_edgecost[index];
      if (cost == k_bad_cost) break;
      if (mesh.num_faces() <= nfaces || mesh.num_vertices() <= nvertices) break;
      num_edges_considered++;
      if (invalidated_edges.contains(e)) continue;
      // COMMIT.
      num_edges_collapsed++;
      Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
      Vec2<Set<Edge>> invalidations_v1_v2;
      for_int(j, 2) {
        Vertex v = V(v1, v2)[j];
        for (Vertex vv : mesh.vertices(v))
          for (Edge ee : mesh.edges(vv)) invalidations_v1_v2[j].add(ee);
      }
      mesh.valid(e);
      const EcolResult ecol_result = try_ecol(e, true);
      // Note: Edge e is now undefined.
      assertx(ecol_result.result == R_success);
      if (minqem) {
        assertx(ecol_result.min_ii == 2);
        assertx(minii2 && no_fit_geom);
      }
      // assertx(ecol_result.cost == cost);
      if (float err = abs(ecol_result.cost - cost); err > 1e-6f && err / cost > 1e-4f && 0)
        assertnever(SSHOW(err, cost));
      const int j_vt = 1 - hh::index(V(v1, v2), ecol_result.vs);
      // for (const int j : {0, 1})  // Achieves strict "assertx(ecol_result.cost == cost)".
      Array<int> ar_j;
      if (minqem && minii2 && no_fit_geom) {
        ar_j = {j_vt};
      } else {
        ar_j = {0, 1};
      }
      for (const int j : ar_j) {
        // for (Edge ee : invalidations_v1_v2[j]) invalidated_edges.add(ee);
        invalidated_edges.merge(invalidations_v1_v2[j]);
      }
    }
    if (verb >= 2)
      showdf("Sweep: %8d edges, %8d considered, %8d collapsed\n",  //
             ar_edgecost.num(), num_edges_considered, num_edges_collapsed);
    if (num_edges_collapsed == 0) break;
  }
  nfaces = 0, nvertices = 0;  // Default for next '-simplify'.
}

// Simplify the mesh until it has <=nfaces or <=nvertices.
void optimize() {
  if (minii2 && minqem) assertx(no_fit_geom);
  if (strict_sharp == 2) {
    for (Face f : mesh.faces()) {
      Vec3<Face> fn;
      int i = 0;
      for (Vertex v : mesh.vertices(f)) fn[i++] = mesh.opp_face(v, f);  // fn[i] can be nullptr
      if (strict_mat_neighbors(f, fn[0], fn[1], fn[2])) continue;
      if (int(!fn[0]) + int(!fn[1]) + int(!fn[2]) >= 2) {
        // Can occur if at corner of terrain grid && !terrain
        Warning("Valence 2 vertex on boundary will be degenerate");
      } else {
        assertnever("!strict_mat_neighbors prior to optimize()");
      }
    }
  }
  if (use_parallelism) {
    const bool qem_compatible = minii2;  // The only Qem method used is evaluate() which is thread-safe.
    const bool can_use_parallelism = !invertexorder && !tvcfac && (!minqem || qem_compatible);
    if (can_use_parallelism) {
      parallel_optimize();
      return;
    }
  }
  ConsoleProgress cprogress;
  cprogress.update(0.f);
  if (!pqecost.num() && !invertexorder) {
    pqecost.reserve(mesh.num_edges());
    // showf("Entering %d edges into priority queue\n", mesh.num_edges());
    ConsoleProgress cprogress2;
    // Add some randomness to the way edges are selected.
    Array<Edge> ar(mesh.edges());
    shuffle(ar, Random::G);
    for (Edge e : ar) {
      if (verb >= 1) cprogress2.update(float(pqecost.num()) / mesh.num_edges());
      ASSERTX(!pqecost.contains(e));
      const EcolResult ecol_result = try_ecol(e, false);
      pqecost.enter_unsorted(e, ecol_result.cost);
      if (verb >= 3) showdf("adding edge with cost=%g\n", ecol_result.cost - offset_cost);
    }
    pqecost.sort();
  }
  // showf("Begin simplification\n");
  const int orig_nfaces = mesh.num_faces(), orig_nvertices = mesh.num_vertices();
  int ntested = 0, nsuccess = 0, onf = mesh.num_faces(), neval = 0, nnotbest = 0;
  for (;;) {
    if (!invertexorder) assertx(pqecost.num() == mesh.num_edges());
    if (0 && ntested % 50 == 0 && pqecost.total_num()) {
      double otot = pqecost.total_priority();
      int ontot = pqecost.total_num();
      double tot = 0.;
      int ntot = 0;
      for (Edge e : mesh.edges()) {
        float pri = pqecost.retrieve(e);
        assertx(pri >= 0.f);
        if (pri < k_bad_dih) {
          tot += pri;
          ntot++;
        }
      }
      SHOW(otot, ontot, tot, ntot);
      assertx(abs(tot / otot - 1.) < 1e-3);
    }
    if (verb >= 1) cprogress.update(fractional_progress(orig_nfaces, orig_nvertices));
    if (verb >= 4 && ntested % 1000 == 0) {
      showf("it %5d, pq%5d/%-5d nf=%d (%3d) eval=%-2d notbest=%-2d\n",  //
            ntested, pqecost.num(), mesh.num_edges(), mesh.num_faces(), mesh.num_faces() - onf, neval, nnotbest);
      onf = mesh.num_faces();
      neval = 0;
      nnotbest = 0;
    }
    if (mesh.num_faces() <= nfaces || mesh.num_vertices() <= nvertices) {
      cprogress.clear();
      showff("Stop. Number of faces reached.\n");
      break;
    }
    // Pull what we think is the best edge.
    if (invertexorder) {
      pqecost.clear();
      int idtoremove = mesh.num_vertices();
      // If this fails, then we should have renumbered original mesh.
      Vertex vtoremove = assertx(mesh.id_retrieve_vertex(idtoremove));
      for (Edge e : mesh.edges(vtoremove)) {
        const EcolResult ecol_result = try_ecol(e, false);
        pqecost.enter(e, ecol_result.cost);
        neval++;
      }
    }
    float expect_cost = pqecost.min_priority();
    if (expect_cost == k_bad_cost) {
      cprogress.clear();
      showff("Stop. No more good edge collapses.\n");
      {
        assertx(pqecost.total_num() == 0);
        assertw(abs(pqecost.total_priority()) < 1e-6);
      }
      break;
    }
    Edge e = pqecost.remove_min();
    mesh.valid(e);
    float nexte_cost = pqecost.min_priority();
    const float a_factor = 1.0f;
    float thresh_cost = expect_cost + (nexte_cost - expect_cost) * a_factor + 1e-20f;
    ntested++;
    {
      const auto& [result, cost, unused_min_ii, unused_vs] = try_ecol(e, false);
      if (verb >= 4)
        showdf("op, expect=%e got=%e thresh=%e %s\n",  //
               expect_cost, cost, thresh_cost,
               (result == R_success ? sform(" success (%.2f)", cost / thresh_cost).c_str() : ""));
      if (cost > thresh_cost || cost == k_bad_cost) {
        if (cost != k_bad_cost) {
          if (0) showf("** expect %e, got %e (next is %e)\n", expect_cost, cost, nexte_cost);
          SSTATV2(Snotbest, expect_cost - cost);
          nnotbest++;
        }
        if (invertexorder) assertnever("");
        pqecost.enter(e, cost);
        continue;
      }
      assertx(result == R_success);
      if (tvcfac) {
        pqecost.enter(e, cost);
        // Think things over in view of TVC; may modify e and cost!
        consider_tvc(e, cost);
        pqecost.remove(e);
      }
    }
    // READY TO COMMIT.
    if (!invertexorder) {
      for (Vertex v : mesh.vertices(e))
        for (Edge ee : mesh.edges(v))
          if (ee != e) assertx(pqecost.remove(ee) >= 0);
    }
    // COMMIT.
    Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
    const auto& [result, unused_cost, unused_min_ii, vs] = try_ecol(e, true);
    assertx(result == R_success);
    if (invertexorder) assertx(mesh.vertex_id(vs) <= mesh.num_vertices());
    // e = nullptr;  // now undefined
    nsuccess++;
    if (tvcfac) {
      Vertex vo = v1 == vs ? v2 : v1;
      for_int(i, tvc_cache.num()) {
        if (tvc_cache[i].v == vo) tvc_cache[i].v = vs;
        // The wedge tvc_cache[i].wid may no longer be present in mesh, but that's OK;
        //  it only slows things down a little bit in consider_tvc().
      }
    }
    // Enter replacement edges.
    Set<Edge> seterecompute;
    if (!invertexorder) {
      for (Edge ee : mesh.edges(vs)) {
        pqecost.enter(ee, k_bad_cost);
        seterecompute.enter(ee);
      }
      assertx(affectpq >= 2);  // affectpq == 1 no longer supported.
      for (Face f : mesh.faces(vs)) {
        Edge ee = mesh.opp_edge(vs, f);
        float cost1 = pqecost.retrieve(ee);
        if ((affectpq >= 1 && cost1 == k_bad_cost) || affectpq >= 3) assertx(seterecompute.add(ee));
      }
      for (Vertex v : mesh.vertices(vs)) {
        for (Edge ee : mesh.edges(v)) {
          float cost1 = pqecost.retrieve(ee);
          if ((affectpq >= 1 && cost1 == k_bad_cost) || affectpq >= 3) seterecompute.add(ee);
        }
      }
    }
    SSTATV2(Serecompute, seterecompute.num());
    for (Edge ee : seterecompute) {
      const EcolResult ecol_result = try_ecol(ee, false);
      pqecost.update(ee, ecol_result.cost);
      neval++;
    }
  }
  cprogress.clear();
  if (verb >= 2)
    showdf("it %5d, pq%5d/%-5d nf=%d (%3d) eval=%-2d notbest=%-2d\n",  //
           ntested, pqecost.num(), mesh.num_edges(), mesh.num_faces(), mesh.num_faces() - onf, neval, nnotbest);
  if (verb >= 2)
    showdf("Finished optimization.  |pq|=%d/%dtot  min=%g\n",  //
           pqecost.num(), mesh.num_edges(), (pqecost.empty() ? 0 : pqecost.min_priority()));
  if (verb >= 2)
    showdf("Operations successful: %d/%d  (%.1f%%)\n", nsuccess, ntested, float(nsuccess) / max(ntested, 1) * 100.f);
  if (tvcfac) showdf("Number of cache misses: %d\n", tvc_ncachemiss);
  nfaces = 0, nvertices = 0;  // default for next '-simplify'
}

// Simplify the mesh until it has <=nfaces or <=nvertices.
void do_simplify() {
  HH_TIMER("_simplify");
  perhaps_initialize();
  static bool pm_was_output = false;
  if (wfile_prog && !pm_was_output) {
    pm_was_output = true;
    int num_wedges;
    {
      Set<int> set_wid;
      for (Face f : mesh.faces())
        for (Corner c : mesh.corners(f)) set_wid.add(c_wedge_id(c));
      num_wedges = set_wid.num();
    }
    showdf("PM: nvertices=%d nwedges=%d nfaces=%d\n", mesh.num_vertices(), num_wedges, mesh.num_faces());
  }
  optimize();
  showdf("Simplified mesh: nvertices=%d nfaces=%d\n", mesh.num_vertices(), mesh.num_faces());
  if (0) analyze_mesh("Simplified");
}

// Recompute the priority queue of edge costs,
//   e.g. for   -mresid 1e-6f -simp  -mresid 1e-4f -rebuildpq -prog x -simp
void do_rebuildpq() {
  assertx(pqecost.num() == mesh.num_edges());
  for (Edge e : mesh.edges()) assertw(pqecost.remove(e) == k_bad_cost);
  assertx(!pqecost.num());
}

void do_verb(Args& args) {
  verb = args.get_int();
  // if (verb < 2 && !Timer::show_times()) Timer::set_show_times(-1);
}

// Output the current mesh.
void do_outmesh(Args& args) {
  perhaps_initialize();
  WFile fi(args.get_filename());
  std::ostream& os = fi();
  write_mesh(os);
  assertx(os);
}

// Print the error residuals for the face point projections onto the mesh.
void do_fpclp(Args& args) {
  WFile fi(args.get_filename());
  WSA3dStream oa3d(fi());
  perhaps_initialize();
  A3dElem el;
  Polygon poly;
  for (const fptinfo& fpt : fpts) {
    if (!assertw(fpt.cmf)) continue;
    mesh.polygon(fpt.cmf, poly);
    Bary dummy_bary;
    Point clp;
    project_point_triangle2(fpt.p, poly[0], poly[1], poly[2], dummy_bary, clp);
    el.init(A3dElem::EType::polyline);
    el.push(A3dVertex(fpt.p, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red())));
    el.push(A3dVertex(clp, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::red())));
    oa3d.write_comment(sform("Face %d", mesh.face_id(fpt.cmf)));
    oa3d.write(el);
  }
  oa3d.flush();
}

// Print the error residuals for the edge point projections onto the mesh.
void do_epclp(Args& args) {
  WFile fi(args.get_filename());
  WSA3dStream oa3d(fi());
  perhaps_initialize();
  A3dElem el;
  for (const eptinfo& ept : epts) {
    if (!ept.cme) continue;
    float bary;
    project_point_seg2(ept.p, mesh.point(mesh.vertex1(ept.cme)), mesh.point(mesh.vertex2(ept.cme)), &bary);
    Point clp = interp(mesh.point(mesh.vertex1(ept.cme)), mesh.point(mesh.vertex2(ept.cme)), bary);
    el.init(A3dElem::EType::polyline);
    el.push(A3dVertex(ept.p, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::green())));
    el.push(A3dVertex(clp, Vector(0.f, 0.f, 0.f), A3dVertexColor(Pixel::green())));
    oa3d.write_comment(
        sform("Edge %d %d", mesh.vertex_id(mesh.vertex1(ept.cme)), mesh.vertex_id(mesh.vertex2(ept.cme))));
    oa3d.write(el);
  }
  oa3d.flush();
}

void do_removesharp() {
  for (Edge e : mesh.edges()) {
    mesh.set_string(e, nullptr);
    mesh.flags(e) = 0;
  }
  for (Face f : mesh.faces())
    for (Corner c : mesh.corners(f)) mesh.set_string(c, nullptr);
}

void do_removeinfo() {
  for (Vertex v : mesh.vertices()) {
    mesh.set_string(v, nullptr);
    mesh.flags(v) = 0;
  }
  for (Face f : mesh.faces()) {
    mesh.set_string(f, nullptr);
    mesh.flags(f) = 0;
  }
  do_removesharp();
}

}  // namespace

int main(int argc, const char** argv) {
  // Suggested args:
  //  MeshSimplify mesh.m -numpts 40000 -nf 20000 -simp -prog mesh.prog -nf 500 -simp >mesh.base.m
  // where
  //  40000 is the number of points to sample on the surface
  //  20000 is the maximum number of faces to keep in the PM representation
  //  500 is the number of faces of the base mesh in the PM rep
  //
  ParseArgs args(argc, argv);
  HH_ARGSC("A mesh is read from stdin or first arg.  Subsequent options are:");
  HH_ARGSP(numpts, "n : set number of random pts to sample");
  HH_ARGSP(nfaces, "n : number of faces desired");
  HH_ARGSP(nvertices, "n : number of vertices desired");
  HH_ARGSD(progressive, "file.prog : record reversible ecol/vsplit");
  HH_ARGSD(simplify, ": apply mesh simplification schedule");
  HH_ARGSC("", ":");
  HH_ARGSP(colfac, "f : weight of color (fraction of diam.)");
  HH_ARGSP(norfac, "f : weight of normals (fraction of diam.)");
  HH_ARGSP(desdfac, "f : weight on diff in height of descendants");
  HH_ARGSP(desnfac, "f : weight on sum of # descendants");
  HH_ARGSP(bspherefac, "f : weight on hier. bounding sphere radius");
  HH_ARGSP(edgepathfac, "f : weight on edge path error");
  HH_ARGSP(trishapepow, "f : exponent on aspect ratio");
  HH_ARGSP(trishapefac, "f : weight on tri aspect ratio");
  HH_ARGSP(trishapeafac, "f : weight on tri aspect ratio times area");
  HH_ARGSP(bndfac, "f : weight to encourage boundary simpl. (1e10)");
  HH_ARGSP(tvcfac, "f : weight on tvc coherence");
  HH_ARGSP(tvcpqa, "bool : tvcfac * pqecost_average_cost");
  HH_ARGSP(tvcowid, "bool : compare tvc wedges in original mesh");
  HH_ARGSD(tvcreinit, ": reinitialize owid and vertex cache");
  HH_ARGSF(dihallow, ": allow bad dihedral as last resort");
  HH_ARGSP(invertexorder, "i : remove vertices in rev. order, 2=fix edges");
  HH_ARGSC("", ":");
  HH_ARGSP(strict_sharp, "i : preserve topology of discont. curves");
  HH_ARGSF(no_fit_geom, ": do not optimize over geometry");
  HH_ARGSP(fit_colors, "bool : modify colors (wad2)");
  HH_ARGSP(fit_normals, "bool : modify normals (wad2)");
  HH_ARGSF(nooutput, ": don't print final mesh on stdout");
  HH_ARGSC("", ":");
  HH_ARGSD(outmesh, "file.m : output current mesh to file");
  HH_ARGSD(fpclp, "file.pclp : print proj. onto mesh (lines)");
  HH_ARGSD(epclp, "file.eclp : print proj. onto mesh (lines)");
  HH_ARGSD(rebuildpq, ": reevaluate all edges");
  HH_ARGSD(verb, "i : verbosity level (1=avg, 2=more, 3=lots)");
  HH_ARGSP(neptfac, "f : fac # samples on sharp edges (def 1or4)");
  HH_ARGSP(affectpq, "i :  (1=little, 2=avg, 3=all)");
  HH_ARGSD(vsgeom, ": for SR, '-minii2 -no_fit_geom'");
  HH_ARGSF(miniiall, ": search 3 new vertex pos");
  HH_ARGSF(minii1, ": force minii == 1 always");
  HH_ARGSF(nominii1, ": force minii != 1 always");
  HH_ARGSF(minii2, ": force minii == 2 in output always");
  HH_ARGSF(no_simp_bnd, ": do not involve boundary vertices");
  HH_ARGSF(keepuvcorners, ": do not simplify vertices at uv=(0, 0), (1, 0), ...");
  HH_ARGSP(keepglobalv, "bool : do not simplify vertices tagged global");
  HH_ARGSP(sphericalparam, "bool : keep valid spherical parameterization");
  HH_ARGSF(attrib_project, ": project vunified to update wedge attribs");
  HH_ARGSP(gspring, "spr : force spring setting (-1 adaptive)");
  HH_ARGSP(gmindih, "dihcos : set min acceptable dihedral angle");
  HH_ARGSF(rnor001, ": compute residuals wrt Vector(0.f, 0.f, 1.f)");
  HH_ARGSF(relerror, ": measure err'-err instead of err' (OLD)");
  HH_ARGSF(maxerr, ": use L^infty norm instead of L^2");
  HH_ARGSP(ter_grid, "file : use as terrain grid");
  HH_ARGSD(gridx, "int : size of grid (or look in ter_grid)");
  HH_ARGSD(gridy, "int : size of grid (or look in ter_grid)");
  HH_ARGSF(gridushorts, ": expect 'ushort' instead of 'float'");
  HH_ARGSP(gridzscale, "fac : scale grid height values");
  HH_ARGSF(terrain, ": measure max error wrt original grid");
  HH_ARGSF(jittervertices, ": jitter samples points off vertices");
  HH_ARGSP(innerhull, "bool : inner instead of outer hull");
  HH_ARGSF(hull, ": build progressive hull");
  HH_ARGSF(minvolume, ": minimize volume (no point samples)");
  HH_ARGSF(minarea, ": minimize area (no point samples)");
  HH_ARGSF(minedgelength, ": always pick shortest edge");
  HH_ARGSF(minvdist, ": minimize dist(vunified, previous_mesh)");
  HH_ARGSF(minqem, ": use quadric error metric");
  HH_ARGSF(minaps, ": use appearance-preserving simplification");
  HH_ARGSF(minrandom, ": use random simplification");
  HH_ARGSP(qemgh98, "bool : use scalar QEM from G&H98 (vs. HH99)");
  HH_ARGSP(qemlocal, "bool : use memoryless QEM");
  HH_ARGSP(qemvolume, "bool : constrain to preserve vol");
  HH_ARGSP(qemweight, "bool : weight qem by area/length");
  HH_ARGSP(qemcache, "bool : cache Q on faces for qemlocal; faster but more mem.");
  HH_ARGSD(removesharp, ": delete attributes at edges and corners");
  HH_ARGSD(removeinfo, ": delete all attributes");
  HH_ARGSP(wedge_materials, "bool : material boundaries imply wedge bnds");
  HH_ARGSP(mresid, "maxresidual : then stop simplification");
  HH_ARGSP(maxvalence, "val : prevent ecols creating verts >valence");
  HH_ARGSF(poszfacenormal, ": prevent facenormal from having nor_z < 0");
  {
    Args targs{"1"};
    do_verb(targs);
  }
  string arg0 = args.num() ? args.peek_string() : "";
  if (ParseArgs::special_arg(arg0)) args.parse(), exit(0);
  string filename = "-";
  if (args.num() && (arg0 == "-" || arg0[0] != '-')) filename = args.get_filename();
  RFile fi(filename);
  for (string line; fi().peek() == '#';) {
    assertx(my_getline(fi(), line));
    if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
  }
  showdf("%s", args.header().c_str());
  Timer timer("MeshSimplify", Timer::EMode::always);
  {
    HH_TIMER("_readmesh");
    mesh.read(fi());
  }
  const int orig_nf = mesh.num_faces();
  args.parse();
  perhaps_initialize();
  wrap_up();
  timer.stop();
  showdf("Rate: %.0f faces/sec\n", (orig_nf - mesh.num_faces()) / max(timer.real(), 0.001));
  timer.start();
  timer.terminate();
  hh_clean_up();
  if (!nooutput) write_mesh(std::cout);
  wfile_prog = nullptr;
  gwinfo.clear();
  if (!k_debug) exit_immediately(0);
  return 0;
}
