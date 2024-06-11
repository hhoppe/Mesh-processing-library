// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/SrMesh.h"

#include "libHh/BinaryIO.h"
#include "libHh/BoundingSphere.h"
#include "libHh/FileIO.h"
#include "libHh/GMesh.h"
#include "libHh/MathOp.h"
#include "libHh/NetworkOrder.h"  // from_std()
#include "libHh/PMesh.h"
#include "libHh/Set.h"
#include "libHh/Stack.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"
#if defined(ANALYZE_PM_COMPRESSION_WITH_XIA_VARSHNEY)
#include "libHh/Encoding.h"
#include "libHh/STree.h"
#endif

namespace hh {

namespace {

// Instead of allowing a variable number of view frustum clipping planes (from 0 to 6),
//  always use 3 (left, right, and bottom).
// In Vis98 flythrough at 60fps: change is very minor:
// Stau:       (8713   )        0.5:8.28283     av=3.45052     sd=1.33553
// Stau:       (8781   )        0.5:9.23064     av=3.42375     sd=1.33939
// So it should be more efficient to use VIEWPARAMS_LRB
// #define VIEWPARAMS_LRB

// Set in SrMesh.h
#if !defined(SR_NOR001)
constexpr bool b_nor001 = false;
#else
constexpr bool b_nor001 = true;
const Vector SrVertexGeometry::vnormal{0.f, 0.f, 1.f};
const float SrVsplit::uni_error_mag2 = 0.f;
const float SrVsplit::sin2alpha = 0.f;
#endif

}  // namespace

// *** MISC

// Analysis of memory requirement:
//
// SIGGRAPH97:
//  SrVertex    2n      19*4    == 152n bytes
//  SrFace      2n      9*4     == 72n  bytes
//  Total:                      == 224n bytes
//
//  example: mojave_sq600 (360k vertices): 80 MBytes
//
// (Compare with GMesh: 312n bytes + 28n bytes attribs)
// (Compare with WMesh: 68n bytes;  AWMesh:  92n bytes)
//
// Obvious compression scheme:
//  (SrVsplit, no fl field, no materials)
//  SrVertex    2n      10*4    == 80n  bytes
//  SrVsplit    n       8*4     == 32n  bytes
//  SrFace      2n      8*4     == 64n  bytes
//  Total:                      == 176n bytes
//
// Same + quantization (8bit on refine_info, 16bit on point/normal)
//  SrVertex    2n      7*4     == 56n  bytes
//  SrVsplit    n       4*4+8   == 20n  bytes
//  SrFace      2n      8*4     == 64n  bytes
//  Total:                      == 140n bytes
//
// NEW (implemented)
//  SrVertex    2n      3*4     == 24n  bytes
//  SrFace      2n      1*4     == 8n   bytes
//  SrVsplit    n       6*4+9*4 == 60n  bytes
//  SrAVertex   m       10*4    == 40m  bytes
//  SrAFace     2m      9*4     == 72m  bytes
//  SrAVertexM  g       4+12*4  == 52g  bytes
//  Total:                      == 92n + 112m + 52g bytes
//
// PAPER: same if defined(SR_NOR001) (no normals, no uni_error, no sin2alpha)
//           and defined(SR_PREDICT_MATID)
//  SrVertex    2n      3*4     == 24n  bytes
//  SrFace      2n      1*4     == 8n   bytes
//  SrVsplit    n       3*4+6*4 == 36n  bytes
//  SrAVertex   m       7*4     == 28m  bytes
//  SrAFace     2m      9*4     == 72m  bytes
//  SrAVertexM  g       4+6*4   == 28g  bytes
//  Total:                      == 68n + 100m + 28g bytes
//
// Same + short for SrAVertex* and SrAFace*
//  SrVertex    2n      10      == 20n  bytes
//  SrFace      2n      1*2     == 4n   bytes
//  SrVsplit    n       3*4+6*4 == 36n  bytes
//  SrAVertex   m       4+5*4   == 24m  bytes
//  SrAFace     2m      9*2     == 36m  bytes
//  SrAVertexM  g       4+6*4   == 28g  bytes
//  Total:                         60n + 60m + 28g bytes
//
// Same + quantization (16bit on static point, 8bit on dir_error and radius)
//  SrVertex    2n      10      == 20n  bytes
//  SrFace      2n      1*2     == 4n   bytes
//  SrVsplit    n   3*2+4*4+2*1 == 24n  bytes
//  SrAVertex   m       4+5*4   == 24m  bytes
//  SrAFace     2m      9*2     == 36m  bytes
//  SrAVertexM  g       4+6*4   == 28g  bytes
//  Total:                      == 48n + 60m + 28g bytes
//
// my PMesh:
//  Vertex:3*4 Wedge:6*4 Face: 2*6*4 Vsplit:(3+6+6)*4   Total: 60n + 84m bytes
// my PMesh (compressed, no wedges)
//  Vertex:6*2 Face: 2*6*4 Vsplit:(2+3)*4               Total: 20n + 60m bytes
// D3DIM PMesh:
//  Vertex:1.2*36 Face:2*14 Vsplit:64                   Total: 64n + 64m bytes
// D3DIM Mesh:
//  Vertex:32 Face: 2*6                                 Total: 44n bytes

// Observations:
// Note that 'array_field-_array' requires an integer division, which is very slow on R10K (~40 cycle stall).
// We managed to eliminate these in all critical sections (using "int SrVertex::vspli" instead of
//  "SrVsplit* SrVertex::vspl").

// New profiling (canyon_4k2k_fly2.frames):
//     25% adapt_refinement()
//     24% render_faces_ogl_texture1()
//     11% qrefine()
//      4% update_vmorphs()
//      2% apply_vspl()
//      2% qcoarsen()
//      2% apply_ecol()
//     27% OpenGL calls (12% __glMgrim_Vertex3fv)

// Timing:
//  NO_REGULATION=1 NO_AMORTIZATION=1 demo_gcanyon -timestat -st gcanyon_4k2k_perf.s3d -geom 500x150 -sr_screen_thresh .00145 -key DS
//   just display (little rasterization): 712'000 faces / sec
//   with adapt_refinement:               460'000 faces / sec

// TODO:
// - vertex geometries could be defined procedurally (eg. image, noise) to reduce space.
// - reduce space: use shorts for SrVertex::avertex and SrFace::aface and set maximum number of SrAVertex and
//    SrAFace to 64 Ki (problem: fully_refine() is currently required during read_pm() ).
// - possibly: separate linked list for vmorph'ing SrAVertex nodes to speed up update_vmorphs(), or better yet,
//     update_vmorphs when creating DrawIndexPrimitive
// - let screen-space tolerance vary with distance of surface from eye.

// Notes on software view frustum culling (SR_SW_CULLING):
// - Evaluating visibility is more difficult for leaf vertices, since they don't have radius_neg values, so for them
//    we just assign vsa->visible = true.  We could also refer to the bounding spheres of their parent vertices
//   (leaf_bsphere_use_parent in RCS).  We don't know which gives better performance.
// - The test for face_invisible() must be "!vis(v0) && !vis(v1) && !vis(v2)" instead of
//    "!vis(v0) || !vis(v1) || !vis(v2)" because the positions of the vertices v_l and v_r are unknown due to the
//    relaxed refinement constraints./  Very unfortunate since otherwise would easily know that some vertices
//    need not be entered into the DrawPrimitive() call (transform + lighting).
// - Even with this && test, some faces sometimes drop out erroneously when geomorphs are turned on in demo_gcanyon.
// - Performance: seems to improve.  for the demos in Video.Notes, we see an improvement from nf=10000 to nf=11300
//    for 36 fps flythrough.
// - Conclusion: for now, leave on since small perf improvement with only very few visual errors.
//    How to handle DrawPrimitive() in future?

// *** SrViewParams

void SrViewParams::set_frame(const Frame& frame) { _frame = frame; }

void SrViewParams::set_zooms(const Vec2<float>& zoomyx) { _zoomyx = zoomyx; }

void SrViewParams::activate_lr_planes(bool flag) { _activate_lr_planes = flag; }

void SrViewParams::activate_bottom_plane(bool flag) { _activate_bottom_plane = flag; }

void SrViewParams::activate_top_plane(bool flag) { _activate_top_plane = flag; }

void SrViewParams::set_hither(float hither) { _hither = hither; }

void SrViewParams::set_yonder(float yonder) { _yonder = yonder; }

void SrViewParams::set_screen_thresh(float screen_thresh) { _screen_thresh = screen_thresh; }

bool SrViewParams::ok() const { return _frame[0][0] != BIGFLOAT && min(_zoomyx) > 0.f; }

// *** SrMesh

HH_ALLOCATE_POOL(SrAVertex);

HH_ALLOCATE_POOL(SrAFacePair);

HH_ALLOCATE_POOL(SrVertexMorph);

SrFace SrMesh::_isolated_face;    // initialized in SrMesh::SrMesh()
SrAFace SrMesh::_isolated_aface;  // initialized in SrMesh::SrMesh()

inline SrAFace* SrMesh::rotate_clw(SrAFace* f, SrAVertex* v) const {
  // Could implement this functionality using fnei comparisons.
  return f->fnei[get_vf_j2(v, f)];
}

inline SrAFace* SrMesh::rotate_ccw(SrAFace* f, SrAVertex* v) const {
  // Could implement this functionality using fnei comparisons.
  return f->fnei[get_vf_j1(v, f)];
}

inline const SrVertex* SrMesh::get_vt(int vspli) const {
  ASSERTX(_vsplits.ok(vspli));
  return _quick_first_vt + vspli * 2;
}

inline SrVertex* SrMesh::get_vt(int vspli) {
  ASSERTX(_vsplits.ok(vspli));
  return _quick_first_vt + vspli * 2;
}

inline const SrFace* SrMesh::get_fl(int vspli) const {
  ASSERTX(_vsplits.ok(vspli));
  return _quick_first_fl + vspli * 2;
}

inline SrFace* SrMesh::get_fl(int vspli) {
  ASSERTX(_vsplits.ok(vspli));
  return _quick_first_fl + vspli * 2;
}

inline int SrMesh::get_vspli(const SrFace* fl) const {
  ASSERTX(_faces.ok(fl));
  return int((fl - _quick_first_fl) / 2);  // slow but seldom
}

inline bool SrMesh::is_active_f(const SrFace* f) const { return f->aface != &_isolated_aface; }

inline bool SrMesh::has_been_created(const SrVertex* v) const {
  return !v->parent || is_active_f(get_fl(v->parent->vspli));
}

inline bool SrMesh::has_been_split(const SrVertex* v) const {
  return is_splitable(v) && is_active_f(get_fl(v->vspli));
}

inline bool SrMesh::is_active_v(const SrVertex* v) const {
  ASSERTX((v->avertex != nullptr) == (has_been_created(v) && !has_been_split(v)));
  return v->avertex != nullptr;
}

inline bool SrMesh::creates_2faces(const SrVsplit* vspl) const { return vspl->fn[3] != &_isolated_face; }

inline const SrVertexGeometry* SrMesh::refined_vg(const SrAVertex* va) const {
  const SrVertexGeometry* vg = &va->vgeom;
  if (va->vmorph) vg = &va->vmorph->vgrefined;
  return vg;
}

inline void SrMesh::finish_vmorph(SrAVertex* va) {
  va->vgeom = va->vmorph->vgrefined;
  delete va->vmorph;
  va->vmorph = nullptr;
}

SrMesh::SrMesh() {
  // Note: _view_params & _refp are set in set_initial_view_params().
  // Initialize static objects.
  _isolated_aface.matid = k_illegal_matid;
  for_int(j, 3) _isolated_aface.fnei[j] = &_isolated_aface;
  _isolated_face.aface = &_isolated_aface;
}

SrMesh::~SrMesh() {
  if (1) {
    fully_coarsen();
    {  // to avoid ~EList(): not empty
      for_int(vi, _base_vertices.num()) _vertices[vi].avertex->activev.unlink();
      assertx(_active_vertices.empty());
      for_int(fi, _base_faces.num()) _faces[fi].aface->activef.unlink();
      assertx(_active_faces.empty());
    }
  } else {
    // Quick destruction involves deleting all SrAVertex and SrAFacePair which are not in base mesh.
    // Traverse active vertices:
    // - delete any active vertex if its upwards branch towards the base mesh includes a right-child relation.
    // - to delete faces, uniquely identify internal nodes (forest above active vertices) by
    //    considering nodes accessible with only right-child relations.
    EListNode* ndelim = _active_vertices.delim();
    for (EListNode* n = ndelim->next(); n != ndelim;) {
      SrAVertex* va = HH_ELIST_OUTER(SrAVertex, activev, n);
      n = n->next();  // advance here before node gets deleted
      bool branch_has_left_child = false;
      bool branch_has_right_child = false;
      for (SrVertex* v = va->vertex;;) {
        SrVertex* vp = v->parent;
        if (!vp) break;
        ASSERTX(is_splitable(vp));
        int vspli = vp->vspli;
        bool is_left_child = get_vt(vspli) == v;
        if (!is_left_child) {
          branch_has_right_child = true;
          if (branch_has_left_child) break;
          SrFace* f = get_fl(vspli);
          ASSERTX(is_active_f(f));
          SrAFace* fa = f->aface;
          SrAFacePair* fpair = reinterpret_cast<SrAFacePair*>(fa);
          delete fpair;
        } else {
          branch_has_left_child = true;
          if (branch_has_right_child) break;
        }
        v = vp;
      }
      if (branch_has_right_child) {
        delete va->vmorph;
        delete va;
      } else {
        ASSERTX(!va->vmorph);
      }
    }
  }
  // optimize: instead, could associate the two Pools (SrAVertex, SrAFacePair) with this object,
  //  and simply deallocate the pools.
}

bool SrMesh::vspl_legal(const SrVertex* vs) const {
  // Warning: this function has been adapted inline in force_vsplit()
  //  for maximum efficiency.
  int vspli = vs->vspli;
  {
    // Preconditions.
    ASSERTX(is_active_v(vs) && is_splitable(vs));
    // Sanity checks.
    ASSERTX(!is_active_v(get_vt(vspli) + 0));
    ASSERTX(!is_active_v(get_vt(vspli) + 1));
    ASSERTX(!is_active_f(get_fl(vspli) + 0));
    ASSERTX(!is_active_f(get_fl(vspli) + 1));
  }
  const SrVsplit* vspl = &_vsplits[vspli];
  if ((!is_active_f(vspl->fn[0]) && vspl->fn[0] != &_isolated_face) || !is_active_f(vspl->fn[1]) ||
      (!is_active_f(vspl->fn[2]) && vspl->fn[2] != &_isolated_face) ||
      (!is_active_f(vspl->fn[3]) && vspl->fn[3] != &_isolated_face))
    return false;
  return true;
}

bool SrMesh::ecol_legal(const SrVertex* vt) const {
  // Warning: this function has been adapted inline in adapt_refinement()
  //  for maximum efficiency.
  const SrVertex* vs = vt->parent;
  int vspli = vs->vspli;
  {
    // Preconditions.
    ASSERTX(vs);
    ASSERTX(is_active_v(vt));
    ASSERTX(get_vt(vspli) == vt);  // vt is the left child of its parent!
    // Sanity checks.
    ASSERTX(is_splitable(vs));
    ASSERTX(!is_active_v(vs));
    ASSERTX(is_active_f(get_fl(vspli) + 0));
    ASSERTX(!creates_2faces(&_vsplits[vspli]) || is_active_f(get_fl(vspli) + 1));
  }
  if (!is_active_v(vt + 1)) return false;
  const SrFace* fl = get_fl(vspli);
  const SrAFace* fla = fl->aface;
  const SrVsplit* vspl = &_vsplits[vspli];
  if (fla->fnei[1] != vspl->fn[0]->aface || fla->fnei[0] != vspl->fn[1]->aface) return false;
  const SrFace* fr = fl + 1;
  const SrAFace* fra = fr->aface;
  ASSERTX(creates_2faces(vspl) ||
          (!is_active_f(fr) && vspl->fn[2] == &_isolated_face && vspl->fn[3] == &_isolated_face));
  if (fra->fnei[2] != vspl->fn[2]->aface || fra->fnei[0] != vspl->fn[3]->aface) return false;
  {
    ASSERTX(fla->vertices[0] == (vt + 0)->avertex);
    ASSERTX(fla->vertices[1] == (vt + 1)->avertex);
    ASSERTX(!creates_2faces(vspl) || (fra->vertices[0] == (vt + 0)->avertex && fra->vertices[2] == (vt + 1)->avertex &&
                                      fla->fnei[2] == fra && fra->fnei[1] == fla));
    ASSERTX(creates_2faces(vspl) || fla->fnei[2] == &_isolated_aface);
  }
  return true;
}

void SrMesh::display_hierarchy_height() const {
  Array<int> ar_height(_vertices.num());
  int max_height = 0;
  for_int(vi, _vertices.num()) {
    const SrVertex* vs = &_vertices[vi];
    const SrVertex* vp = vs->parent;
    int height = !vp ? 1 : ar_height[narrow_cast<int>(vp - _vertices.data())] + 1;
    ar_height[vi] = height;
    if (k_debug)
      if (vp) assertx(vp < vs);
    if (height > max_height) max_height = height;
  }
  showdf("vertex hierarchy height=%d\n", max_height);
}

void SrMesh::read_pm(PMeshRStream& pmrs) {
  HH_TIMER("__read_pm");
  assertx(!_base_vertices.num());
  assertx(!_refine_morph_time && !_coarsen_morph_time);
  assertw(!pmrs._info._has_rgb);
  assertw(!pmrs._info._has_uv);
  const AWMesh& bmesh = pmrs.base_mesh();
  // Get info about fully detailed mesh and number of vsplits.
  int full_nvertices = pmrs._info._full_nvertices;
  int full_nfaces = pmrs._info._full_nfaces;
  int tot_nvsplits = pmrs._info._tot_nvsplits;
  assertx(full_nvertices && full_nfaces && tot_nvsplits);
  // Pre-allocate arrays.
  {
    assertx(full_nvertices == bmesh._vertices.num() + tot_nvsplits);
    _vertices.init(bmesh._vertices.num() + 2 * tot_nvsplits);
    _faces.init(bmesh._faces.num() + 2 * tot_nvsplits);  // != full_nfaces!
    _vsplits.init(tot_nvsplits);
    _base_vertices.init(bmesh._vertices.num());
    _base_faces.init(bmesh._faces.num());
    _quick_first_vt = &_vertices[bmesh._vertices.num()];
    _quick_first_fl = &_faces[bmesh._faces.num()];
  }
  // Copy materials.
  _materials = bmesh._materials;
  Timer timer("___read_convert");
  Array<SrVertexGeometry> vgeoms(_vertices.num());  // all vertex geometries
  // Process the base mesh.
  {
    assertx(bmesh._vertices.num() == bmesh._wedges.num());
    for_int(wi, bmesh._wedges.num()) {
      int vi = wi;
      assertx(bmesh._wedges[wi].vertex == vi);
      SrAVertex* va = &_base_vertices[vi];
      _vertices[vi].avertex = va;
      _vertices[vi].parent = nullptr;
      _vertices[vi].vspli = -1;
      va->activev.link_before(_active_vertices.delim());
      va->vertex = &_vertices[vi];
      va->vgeom.point = bmesh._vertices[vi].attrib.point;
      va->vgeom.vnormal = bmesh._wedges[vi].attrib.normal;
      if (b_nor001) assertx(Vector(0.f, 0.f, 1.f) == va->vgeom.vnormal);
      // va->vmorph = nullptr;
      va->visible = false;
      va->cached_time = 0;
      vgeoms[vi] = va->vgeom;
    }
    for_int(fi, bmesh._faces.num()) {
      SrAFace* fa = &_base_faces[fi];
      _faces[fi].aface = fa;
      fa->activef.link_before(_active_faces.delim());
      fa->matid = bmesh._faces[fi].attrib.matid;
      assertx(_materials.ok(fa->matid));
    }
    for_int(fi, bmesh._faces.num()) {
      SrAFace* fa = _faces[fi].aface;
      for_int(j, 3) {
        int pm_wi = bmesh._faces[fi].wedges[j];
        int pm_vi = bmesh._wedges[pm_wi].vertex;
        fa->vertices[j] = _vertices[pm_vi].avertex;
        int pm_fnei = bmesh._fnei[fi].faces[j];
        fa->fnei[j] = pm_fnei < 0 ? &_isolated_aface : _faces[pm_fnei].aface;
      }
    }
    _num_active_vertices = _base_vertices.num();
    _num_active_faces = _base_faces.num();
  }
  // Process vsplit records.
  Array<int> f_pm2sr(full_nfaces);  // PM faces index -> SR face index
  f_pm2sr.init(0);
  for_int(fi, _base_faces.num()) f_pm2sr.push(fi);
  //
  Array<short> f_matid(full_nfaces);  // temporary back-up
  f_matid.init(0);
  for_int(fi, _base_faces.num()) {
    f_matid.push(narrow_cast<short>(_faces[fi].aface->matid));
    _faces[fi].aface->matid = fi;  // in the next section, matid is temporarily used to store face indices?
  }
  //
  for_int(vspli, tot_nvsplits) {
    const Vsplit& pm_vspl = *assertx(pmrs.next_vsplit());
    const int code = pm_vspl.code;
    int vs_index = (code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
    int ii = (code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
    assertw(ii == 2);
    SrVsplit* vspl = &_vsplits[vspli];
    SrAVertex* vsa;
    int flclwi = f_pm2sr[pm_vspl.flclw];
    assertx(pm_vspl.vlr_offset1 > 0);  // flclw non-existent is now illegal
    SrFace* flclw = &_faces[flclwi];
    ASSERTX(is_active_f(flclw));
    vsa = flclw->aface->vertices[vs_index];
    vspl->fn[1] = flclw;
    {
      SrAFace* fa1 = rotate_ccw(flclw->aface, vsa);
      vspl->fn[0] = fa1 == &_isolated_aface ? &_isolated_face : &_faces[fa1->matid];
      if (fa1 != &_isolated_aface) assertx(is_active_f(vspl->fn[0]));
    }
    if (pm_vspl.vlr_offset1 == 1) {
      vspl->fn[2] = &_isolated_face;
      vspl->fn[3] = &_isolated_face;
    } else {
      SrAFace* fa = flclw->aface;
      for_int(count, pm_vspl.vlr_offset1 - 2) {
        fa = rotate_clw(fa, vsa);
        ASSERTX(fa != &_isolated_aface && fa != flclw->aface);
      }
      vspl->fn[3] = &_faces[fa->matid];
      fa = rotate_clw(fa, vsa);
      vspl->fn[2] = fa == &_isolated_aface ? &_isolated_face : &_faces[fa->matid];
      if (fa != &_isolated_aface) assertx(is_active_f(vspl->fn[2]));
    }
    SrVertex* vs = vsa->vertex;
    vs->vspli = vspli;
    SrVertex* vt = get_vt(vspli);
    for_int(i, 2) {
      (vt + i)->avertex = nullptr;
      (vt + i)->parent = vs;
      (vt + i)->vspli = -1;
    }
    SrFace* fl = get_fl(vspli);
    for_int(i, 2) { (fl + i)->aface = &_isolated_aface; }
    bool cr2faces = creates_2faces(vspl);
#if !defined(SR_NO_VSGEOM)
    vspl->vu_vgeom.point = vsa->vgeom.point + pm_vspl.vad_large.dpoint;
    assertx(is_zero(pm_vspl.vad_small.dpoint));
    assertx(pm_vspl.ar_wad.num() == 1);
    vspl->vu_vgeom.vnormal = vsa->vgeom.vnormal + pm_vspl.ar_wad[0].dnormal;
    if (b_nor001) assertx(is_zero(pm_vspl.ar_wad[0].dnormal));
    vgeoms[_base_vertices.num() + 2 * vspli + 0] = vgeoms[narrow_cast<int>(vs - _vertices.data())];
    vgeoms[_base_vertices.num() + 2 * vspli + 1] = vspl->vu_vgeom;
#else
    vspl->vs_vgeom = vsa->vgeom;
    vspl->vt_vgeom = vsa->vgeom;
    vspl->vu_vgeom = vsa->vgeom;
    if (ii == 1) {
      Point midp = vsa->vgeom.point + pm_vspl.vad_small.dpoint;
      vspl->vt_vgeom.point = midp - pm_vspl.vad_large.dpoint;
      vspl->vu_vgeom.point = midp + pm_vspl.vad_large.dpoint;
    } else if (ii == 2) {
      vspl->vt_vgeom.point = vsa->vgeom.point + pm_vspl.vad_small.dpoint;
      vspl->vu_vgeom.point = vsa->vgeom.point + pm_vspl.vad_large.dpoint;
    } else {
      vspl->vt_vgeom.point = vsa->vgeom.point + pm_vspl.vad_large.dpoint;
      vspl->vu_vgeom.point = vsa->vgeom.point + pm_vspl.vad_small.dpoint;
    }
    assertx(is_zero(pm_vspl.ar_wad[0].dnormal));
    vgeoms[_base_vertices.num() + 2 * vspli + 0] = vspl->vt_vgeom;
    vgeoms[_base_vertices.num() + 2 * vspli + 1] = vspl->vu_vgeom;
#endif  // !defined(SR_NO_VSGEOM)
#if !defined(SR_PREDICT_MATID)
    vspl->fl_matid =
        narrow_cast<short>((code & Vsplit::FLN_MASK) ? pm_vspl.fl_matid : f_matid[vspl->fn[1]->aface->matid]);
    vspl->fr_matid = narrow_cast<short>(!cr2faces                   ? -1
                                        : (code & Vsplit::FRN_MASK) ? pm_vspl.fr_matid
                                                                    : f_matid[vspl->fn[3]->aface->matid]);
#else
    assertx(!(code & Vsplit::FLN_MASK));
    assertx(!cr2faces || !(code & Vsplit::FRN_MASK));
#endif  // !defined(SR_PREDICT_MATID)
    vspl->uni_error_mag2 = square(pm_vspl.resid_uni);
    vspl->dir_error_mag2 = square(pm_vspl.resid_dir);
    if (b_nor001 && vspl->uni_error_mag2) {
      Warning("uni_error_mag2 > 0 with b_nor001");
      vspl->dir_error_mag2 = max(vspl->uni_error_mag2, vspl->dir_error_mag2);
      vspl->uni_error_mag2 = 0.f;
    }
    {
      EListNode* n = _active_vertices.delim();
      apply_vspl(vs, n);  // apply vsplit on SrMesh
    }
    for_int(i, 2) {
      int matid = (fl + i)->aface->matid;
      if (!i || cr2faces) {
        // 2012-12-12 This section appears to be broken, because aface matid are used to store face_id's,
        //  and these same matid are used to predict matid for newly introduced faces in apply_vspl().
        // In summary, support of > 1 material seems broken.
        if (0) assertw(_materials.ok(matid));
      }
      // 2012-12-12 added mask (_isolated_aface.matid == k_illegal_matid or bad matid)
      f_matid.push(static_cast<short>(matid & 0xFFFF));
      if (!i || cr2faces) (fl + i)->aface->matid = _base_faces.num() + 2 * vspli + i;
      if (!i || cr2faces) f_pm2sr.push(_base_faces.num() + 2 * vspli + i);
    }
  }
  assertx(f_pm2sr.num() == num_active_faces());
  for_int(fi, full_nfaces) {
    if (!is_active_f(&_faces[fi])) continue;  // !creates_2faces()
    _faces[fi].aface->matid = f_matid[fi];    // restore matid
  }
  f_pm2sr.clear();  // cleanup to shrink memory usage
  f_matid.clear();  // cleanup to shrink memory usage
  assertx(!pmrs.peek_next_vsplit());
  timer.terminate();  // "___read_convert"
  if (k_debug) ok();
  // Compute radii of influence of vertex splits.
  compute_bspheres(vgeoms);
  // Construct hierarchy of bounds on surface normals; similar approach.
  if (b_nor001) {
    for_int(vspli, _vsplits.num()) _vsplits[vspli].sin2alpha = 0.f;
  } else {
    compute_nspheres(vgeoms);
  }
  // Record bbox of model.
  assertx(_vertices.num() == vgeoms.num());
  _bbox = Bbox{transform(vgeoms, [](auto v) { return v.point; })};
  display_hierarchy_height();
  if (k_debug) ok();
  {
    // Coarsen now so that performance statistics don't get skewed.
    // Also we usually never care about the fully detailed mesh.
    HH_TIMER("___read_coarsen");
    fully_coarsen();
  }
  if (k_debug) ok();
  set_initial_view_params();
#if defined(ANALYZE_PM_COMPRESSION_WITH_XIA_VARSHNEY)
  static const bool g_reorder = getenv_bool("SR_REORDER");
  if (g_reorder) {
    refine_in_best_dflclw_order();
    fully_coarsen();
  }
#endif
}

void SrMesh::compute_bspheres(CArrayView<SrVertexGeometry> vgeoms) {
  HH_TIMER("___compute_bspheres");
  // spheres bounding positions over surface.
  Array<BoundingSphere> ar_bsphere(_vertices.num());
  // Compute bound(star(v)) of vertices in fully refined mesh.
  {
    // based on shirman-abi-ezzi93.
    Array<Bbox<float, 3>> ar_bbox(_vertices.num());
    for_int(vi, _vertices.num()) {
      if (!is_active_v(&_vertices[vi])) continue;
      ar_bbox[vi][0] = ar_bbox[vi][1] = _vertices[vi].avertex->vgeom.point;
    }
    for_int(fi, _faces.num()) {
      SrAFace* fa = _faces[fi].aface;
      if (fa == &_isolated_aface) continue;  // !creates_2faces()
      Vec3<int> ar_vi;
      for_int(j, 3) {
        // division in here may be slow.
        ar_vi[j] = narrow_cast<int>(fa->vertices[j]->vertex - _vertices.data());
      }
      for_int(j, 3) {
        int j0 = j, j1 = mod3(j0 + 1);
        SrAVertex* va0 = fa->vertices[j0];
        SrAVertex* va1 = fa->vertices[j1];
        int vi0 = ar_vi[j0];
        int vi1 = ar_vi[j1];
        ar_bbox[vi0].union_with(va1->vgeom.point);
        ar_bbox[vi1].union_with(va0->vgeom.point);
      }
    }
    // Let center of bounding sphere equal the center of bbox.
    for_int(vi, _vertices.num()) {
      if (!is_active_v(&_vertices[vi])) continue;
      ar_bsphere[vi].point = interp(ar_bbox[vi][0], ar_bbox[vi][1]);
      ar_bsphere[vi].radius = dist2(_vertices[vi].avertex->vgeom.point, ar_bsphere[vi].point);
    }
    for_int(fi, _faces.num()) {
      SrAFace* fa = _faces[fi].aface;
      if (fa == &_isolated_aface) continue;  // !creates_2faces()
      Vec3<int> ar_vi;
      for_int(j, 3) {
        // division in here may be slow.
        ar_vi[j] = narrow_cast<int>(fa->vertices[j]->vertex - _vertices.data());
      }
      for_int(j, 3) {
        int j0 = j, j1 = mod3(j0 + 1);
        SrAVertex* va0 = fa->vertices[j0];
        SrAVertex* va1 = fa->vertices[j1];
        int vi0 = ar_vi[j0];
        int vi1 = ar_vi[j1];
        float d2;
        d2 = dist2(va1->vgeom.point, ar_bsphere[vi0].point);
        if (d2 > ar_bsphere[vi0].radius) ar_bsphere[vi0].radius = d2;
        d2 = dist2(va0->vgeom.point, ar_bsphere[vi1].point);
        if (d2 > ar_bsphere[vi1].radius) ar_bsphere[vi1].radius = d2;
      }
    }
    // Unsquare the radii.
    for_int(vi, _vertices.num()) {
      if (is_active_v(&_vertices[vi])) ar_bsphere[vi].radius = sqrt(ar_bsphere[vi].radius);
    }
  }
  // Iterate back over vsplits to assign bounding spheres to
  //  interior vertices.
  for (int vspli = _vsplits.num() - 1; vspli >= 0; --vspli) {
    SrVertex* vt = get_vt(vspli);
    SrVertex* vs = vt->parent;
    int vti = narrow_cast<int>(vt - _vertices.data());
    int vsi = narrow_cast<int>(vs - _vertices.data());
    ASSERTX(ar_bsphere[vti + 0].radius >= 0.f);
    ASSERTX(ar_bsphere[vti + 1].radius >= 0.f);
    ar_bsphere[vsi] = bsphere_union(ar_bsphere[vti + 0], ar_bsphere[vti + 1]);
  }
  // From computed bounds, compute appropriate radii for vsplits.
  static const bool sr_no_frustum_test = getenv_bool("SR_NO_FRUSTUM_TEST");
  assertw(!sr_no_frustum_test);
  for_int(vi, _vertices.num()) {
    SrVertex* vs = &_vertices[vi];
    if (!is_splitable(vs)) continue;
    SrVsplit* vspl = &_vsplits[vs->vspli];
    vspl->radius_neg = -(dist(vgeoms[vi].point, ar_bsphere[vi].point) + ar_bsphere[vi].radius);
    if (sr_no_frustum_test) vspl->radius_neg = -BIGFLOAT;
  }
}

void SrMesh::compute_nspheres(CArrayView<SrVertexGeometry> vgeoms) {
  HH_TIMER("___compute_nspheres");
  // spheres bounding normals over surface.
  Array<BoundingSphere> ar_nsphere(_vertices.num());
  // Compute bound(star(v)) of vertices in fully refined mesh.
  {
    Array<Bbox<float, 3>> ar_bbox(_vertices.num());
    Array<Vector> ar_fnormal(_faces.num());  // cache face normals
    for_int(fi, _faces.num()) {
      SrAFace* fa = _faces[fi].aface;
      if (fa == &_isolated_aface) continue;  // !creates_2faces()
      const Point& p0 = fa->vertices[0]->vgeom.point;
      const Point& p1 = fa->vertices[1]->vgeom.point;
      const Point& p2 = fa->vertices[2]->vgeom.point;
      Vector fnormal = cross(p0, p1, p2);
      assertw(fnormal.normalize());
      ar_fnormal[fi] = fnormal;
      for_int(j, 3) {
        SrVertex* v = fa->vertices[j]->vertex;
        int vi = narrow_cast<int>(v - _vertices.data());
        ar_bbox[vi].union_with(ar_fnormal[fi]);
      }
    }
    // Let center of bounding sphere equal the center of bbox.
    for_int(vi, _vertices.num()) {
      if (!is_active_v(&_vertices[vi])) continue;
      ar_nsphere[vi].point = interp(ar_bbox[vi][0], ar_bbox[vi][1]);
      ar_nsphere[vi].radius = 0.f;
    }
    for_int(fi, _faces.num()) {
      SrAFace* fa = _faces[fi].aface;
      if (fa == &_isolated_aface) continue;  // !creates_2faces()
      for_int(j, 3) {
        SrVertex* v = fa->vertices[j]->vertex;
        int vi = narrow_cast<int>(v - _vertices.data());
        float d2 = dist2(Point(ar_fnormal[fi]), ar_nsphere[vi].point);
        if (d2 > ar_nsphere[vi].radius) ar_nsphere[vi].radius = d2;
      }
    }
    // Unsquare the radii.
    for_int(vi, _vertices.num()) {
      if (is_active_v(&_vertices[vi])) ar_nsphere[vi].radius = sqrt(ar_nsphere[vi].radius);
    }
  }
  // Iterate back over vsplits to assign bounding spheres to
  //  interior vertices.
  for (int vspli = _vsplits.num() - 1; vspli >= 0; --vspli) {
    SrVertex* vt = get_vt(vspli);
    SrVertex* vs = vt->parent;
    int vti = narrow_cast<int>(vt - _vertices.data());
    int vsi = narrow_cast<int>(vs - _vertices.data());
    ASSERTX(ar_nsphere[vti + 0].radius >= 0.f);
    ASSERTX(ar_nsphere[vti + 1].radius >= 0.f);
    ar_nsphere[vsi] = bsphere_union(ar_nsphere[vti + 0], ar_nsphere[vti + 1]);
  }
  // From computed normal bounds, compute sin2alpha for vsplits.
  static const bool sr_no_normal_test = getenv_bool("SR_NO_NORMAL_TEST");
  assertw(!sr_no_normal_test);
  for_int(vi, _vertices.num()) {
    SrVertex* vs = &_vertices[vi];
    if (!is_splitable(vs)) continue;
    SrVsplit* vspl = &_vsplits[vs->vspli];
    Vector dir_b = to_Vector(ar_nsphere[vi].point);
    Vector nor_b = dir_b;
    assertw(nor_b.normalize());
    float r_b = ar_nsphere[vi].radius;
    // cos(theta) == (b ^ 2 + c ^ 2 - a ^ 2) / (2 * b * c)
    float v = (mag2(dir_b) + 1.f - square(r_b)) / (2.f * mag(dir_b));
    if (v < -1.f) v = -1.f;  // (triangle inequality failed, ok)
    float alpha_b = my_acos(v);
    // float alpha_p = my_acos(dot(nor_b, vgeoms[vi]. vnormal));
    float alpha_p = angle_between_unit_vectors(nor_b, vgeoms[vi].vnormal);
    float alpha = alpha_b + alpha_p;
    if (sr_no_normal_test) alpha = TAU / 4;
    if (alpha >= TAU / 4) alpha = TAU / 4;  // then backface cone is empty.
    HH_SSTAT(Salpha, alpha);
    vspl->sin2alpha = square(std::sin(alpha));
  }
}

void SrMesh::write_srm(std::ostream& os) const {
  assertx(num_active_faces() == _base_faces.num());  // else fully_coarsen();
  // Write out sizes.
  os << "SRM\n";
  os << "base_nvertices=" << _base_vertices.num() << " base_nfaces=" << _base_faces.num()
     << " nvsplits=" << _vsplits.num() << '\n';
  // Write out bounding box.
  os << "bbox " << _bbox[0][0] << ' ' << _bbox[0][1] << ' ' << _bbox[0][2] << "  " << _bbox[1][0] << ' ' << _bbox[1][1]
     << ' ' << _bbox[1][2] << '\n';
  // Write out materials.
  _materials.write(os);
  // Write out base mesh.
  for_int(vi, _base_vertices.num()) {
    assertx(_vertices[vi].avertex == &_base_vertices[vi]);
    const SrVertexGeometry* vg = &_vertices[vi].avertex->vgeom;
    write_binary_std(os, vg->point.view());
    write_binary_std(os, vg->vnormal.view());
  }
  for_int(fi, _base_faces.num()) {
    const SrAFace* fa = &_base_faces[fi];
    for_int(j, 3) write_binary_std(os, ArView(narrow_cast<int>(fa->vertices[j] - _base_vertices.data())));
    for_int(j, 3) {
      unsigned fni = fa->fnei[j] != &_isolated_aface ? narrow_cast<int>(fa->fnei[j] - _base_faces.data()) + 1 : 0;
      write_binary_std(os, ArView(fni));
    }
    write_binary_std(os, ArView(unsigned(fa->matid) & ~k_Face_visited_mask));
  }
  // Write out vsplits.
  for_int(vspli, _vsplits.num()) {
    const SrVsplit* vspl = &_vsplits[vspli];
    const SrVertex* vs = assertx(get_vt(vspli)->parent);
    write_binary_std(os, ArView(narrow_cast<int>(vs - _vertices.data())));
    for_int(j, 4) {
      unsigned fni = vspl->fn[j] != &_isolated_face ? narrow_cast<int>(vspl->fn[j] - _faces.data()) + 1 : 0;
      write_binary_std(os, ArView(fni));
    }
#if !defined(SR_PREDICT_MATID)
    write_binary_std(os, ArView(narrow_cast<ushort>(unsigned(vspl->fl_matid) & ~k_Face_visited_mask)));
    write_binary_std(os, ArView(narrow_cast<ushort>(unsigned(vspl->fr_matid) & ~k_Face_visited_mask)));
#else
    write_binary_std(os, ArView(ushort{0}));
    write_binary_std(os, ArView(ushort{0}));
#endif
    write_binary_std(os, ArView(vspl->uni_error_mag2));
    write_binary_std(os, ArView(vspl->dir_error_mag2));
    write_binary_std(os, ArView(vspl->radius_neg));
    write_binary_std(os, ArView(vspl->sin2alpha));
    write_binary_std(os, vspl->vu_vgeom.point.view());
    write_binary_std(os, vspl->vu_vgeom.vnormal.view());
  }
}

void SrMesh::read_srm(std::istream& is) {
  HH_TIMER("__read_srm");
  assertx(!_base_vertices.num());
  assertx(!_refine_morph_time && !_coarsen_morph_time);  // just to be safe
  // Read past comments.
  for (string line;;) {
    assertx(my_getline(is, line));
    if (line == "" || line[0] == '#') continue;
    assertx(line == "SRM");
    break;
  }
  // Read in sizes.
  {
    string line;
    assertx(my_getline(is, line));
    const char* s = line.c_str();
    int bnv, bnf, nvspl;
    s = assertx(after_prefix(s, "base_nvertices=")), bnv = int_from_chars(s);
    s = assertx(after_prefix(s, " base_nfaces=")), bnf = int_from_chars(s);
    s = assertx(after_prefix(s, " nvsplits=")), nvspl = int_from_chars(s);
    assert_no_more_chars(s);
    _vertices.init(bnv + 2 * nvspl);
    _faces.init(bnf + 2 * nvspl);
    _vsplits.init(nvspl);
    _base_vertices.init(bnv);
    _base_faces.init(bnf);
    _quick_first_vt = &_vertices[bnv];
    _quick_first_fl = &_faces[bnf];
  }
  // Read in bounding box.
  {
    string line;
    assertx(my_getline(is, line));
    const char* s = assertx(after_prefix(line.c_str(), "bbox "));
    for_int(i, 2) for_int(j, 3) _bbox[i][j] = float_from_chars(s);
    assert_no_more_chars(s);
  }
  // Read in materials.
  _materials.read(is);
  // Read in base mesh.
  for_int(vi, _base_vertices.num()) {
    SrAVertex* va = &_base_vertices[vi];
    _vertices[vi].avertex = va;
    _vertices[vi].parent = nullptr;
    _vertices[vi].vspli = -1;
    va->activev.link_before(_active_vertices.delim());
    va->vertex = &_vertices[vi];
    // va->vmorph = nullptr;
    va->visible = false;
    va->cached_time = 0;
    assertx(read_binary_std(is, va->vgeom.point.view()));
    assertx(read_binary_std(is, va->vgeom.vnormal.view()));
    if (b_nor001) assertx(Vector(0.f, 0.f, 1.f) == va->vgeom.vnormal);
  }
  for_int(fi, _base_faces.num()) {
    SrAFace* fa = &_base_faces[fi];
    _faces[fi].aface = fa;
    fa->activef.link_before(_active_faces.delim());
    Vec3<int> vi;
    assertx(read_binary_std(is, vi.view()));
    Vec3<int> fni;
    assertx(read_binary_std(is, fni.view()));
    int matid;
    assertx(read_binary_std(is, ArView(matid)));
    for_int(j, 3) fa->vertices[j] = &_base_vertices[vi[j]];
    for_int(j, 3) fa->fnei[j] = fni[j] ? &_base_faces[fni[j] - 1] : &_isolated_aface;
    fa->matid = matid;
  }
  _num_active_vertices = _base_vertices.num();
  _num_active_faces = _base_faces.num();
  // Read in vsplits.
  for_int(vspli, _vsplits.num()) {
    SrVsplit* vspl = &_vsplits[vspli];
    struct srm_vsplit_binary_buf {
      int vsi;
      Vec4<int> fni;
      ushort fl_matid;
      ushort fr_matid;
      float uni_error_mag2;
      float dir_error_mag2;
      float radius_neg;
      float sin2alpha;
      Vec3<float> p;
      Vec3<float> n;
    };
    srm_vsplit_binary_buf buf;
    assertx(read_binary_raw(is, ArView(buf)));
    from_std(&buf.vsi);
    int vsi = buf.vsi;
    SrVertex* vs = &_vertices[vsi];
    vs->vspli = vspli;
    for_int(j, 4) {
      from_std(&buf.fni[j]);
      int fni = buf.fni[j];
      vspl->fn[j] = fni ? &_faces[fni - 1] : &_isolated_face;
    }
    from_std(&buf.fl_matid);
    short fl_matid = buf.fl_matid;
    from_std(&buf.fr_matid);
    short fr_matid = buf.fr_matid;
#if !defined(SR_PREDICT_MATID)
    vspl->fl_matid = fl_matid;
    vspl->fr_matid = fr_matid;
#else
    if (fl_matid || fr_matid) {
      static int warning_matid;
      if (!warning_matid++) Warning("{fl,fr}_matid ignored; may be ok");
    }
#endif
    from_std(&buf.uni_error_mag2);
    vspl->uni_error_mag2 = buf.uni_error_mag2;
    if (b_nor001) assertx(!vspl->uni_error_mag2);
    from_std(&buf.dir_error_mag2);
    vspl->dir_error_mag2 = buf.dir_error_mag2;
    from_std(&buf.radius_neg);
    vspl->radius_neg = buf.radius_neg;
    from_std(&buf.sin2alpha);
    vspl->sin2alpha = buf.sin2alpha;
    // if (b_nor001) assertx(!vspl->sin2alpha);
    if (b_nor001 && vspl->sin2alpha) {
      static int warning_sin2alpha;
      if (!warning_sin2alpha++) Warning("Ignoring sin2alpha values");
      vspl->sin2alpha = 0.f;
    }
    Point& p = vspl->vu_vgeom.point;
    Vector& n = vspl->vu_vgeom.vnormal;
    for_int(c, 3) {
      from_std(&buf.p[c]);
      p[c] = buf.p[c];
    }
    for_int(c, 3) {
      from_std(&buf.n[c]);
      n[c] = buf.n[c];
    }
    if (b_nor001) assertx(Vector(0.f, 0.f, 1.f) == n);
    SrVertex* vt = get_vt(vspli);
    for_int(i, 2) {
      (vt + i)->avertex = nullptr;
      (vt + i)->parent = vs;
      (vt + i)->vspli = -1;
    }
    SrFace* fl = get_fl(vspli);
    for_int(i, 2) { (fl + i)->aface = &_isolated_aface; }
  }
  display_hierarchy_height();
  if (k_debug) ok();
  // Note: we are fully coarsened now.  It's the natural state.
  set_initial_view_params();
}

// Reject vsplit if surface orientation is away.
//   Return 0
//    if (dot(normalized(p - eye), vnormal) > sin(alpha))
//    if (dot(p - eye, vnormal) > 0 && square(dot(p - eye, vnormal)) > mag2(p - eye) * square(sin(alpha))
// Reject vsplit if residual smaller than screen threshold.
//   Note: using Euclidean distance (pem2 below) is better in some ways because it removes more faces on sides and
//     behind viewer, but using linear functional tzdist is more accurate for approximating residual size under
//     perspective projection.
//   As pointed out in Lindstrom-etal96, Euclidean distance tends to coarsen objects away from the center of
//     projection, especially with a wide field of view.
//   So overall the Euclidean distance is preferred.
//   Refine based on uniform and directional residual error.
//   Assume dir_error = vspl->dir_error_mag * vs->vnormal:
//   Return 0
//    if mag(cross(normalized(p - eye), dir_error)) < thresh * zoom * mag(p - eye)
//    if mag2(cross(normalized(p - eye), dir_error)) < tz2 * mag2(p - eye)
//    if dir_error_mag2 * mag2(cross(normalized(p - eye), vnormal)) < tz2 * mag2(p -eye)
//      [ mag2(cross(unitv1, unitv2)) == (1 - square(dot(unitv1, unitv2))) ]
//    if dir_error_mag2 * (1 - square(dot(normalized(p - eye), vnormal))) < ...
//    if dir_error_mag2 * (mag2(p - eye)-square(dot(p - eye, vnormal))) < tz2 * mag4(p - eye)
//  unoptimized:
//   const Point& eye = _refp._eye;
//   float rhs1 = _refp._tz2 * mag2(p - eye);
//   if (vspl->uni_error_mag2 < rhs1 &&
//       vspl->dir_error_mag 2 *(mag2(p - eye)-square(dot(p - eye, vs->vnormal))) < rhs1 * mag2(p - eye)) return false;
// Reject vsplit if no effect inside view frustum.
//   for_int(i, _refp._nplanes) {
//       if (_refp._planes[i].eval(p) < rneg) return false;
//   }

inline bool SrMesh::is_visible(const SrVertexGeometry* vg, const SrVsplit* vspl) const {
  const Point& p = vg->point;
#if !defined(SR_NOR001)
  const Point& e = _refp._eye;
  const Point pe = p - e;
  const Vector& vnor = vg->vnormal;
  float vdot = pe[0] * vnor[0] + pe[1] * vnor[1] + pe[2] * vnor[2];
  float pem2 = pe[0] * pe[0] + pe[1] * pe[1] + pe[2] * pe[2];
  if (vdot > 0.f && square(vdot) > pem2 * vspl->sin2alpha) return false;
#endif
  float rneg = vspl->radius_neg;
  for_int(i, _refp._nplanes) {
    if (_refp._planes[i].eval(p) < rneg) return false;
  }
  return true;
}

inline bool SrMesh::big_error(const SrVertexGeometry* vg, const SrVsplit* vspl) const {
#if !defined(SR_NOR001)
  // The original SIGGRAPH 97 definition.
  const Point& p = vg->point;
  float px = p[0], py = p[1], pz = p[2];
  const Point& e = _refp._eye;
  float pex = px - e[0], pey = py - e[1], pez = pz - e[2];
  float pem2 = pex * pex + pey * pey + pez * pez;
  float rhs1 = _refp._tz2 * pem2;
  const Vector& vnor = vg->vnormal;
  float vdot = pex * vnor[0] + pey * vnor[1] + pez * vnor[2];
  return vspl->uni_error_mag2 >= rhs1 || vspl->dir_error_mag2 * (pem2 - square(vdot)) >= rhs1 * pem2;
#elif 0
  // SIGGRAPH 97 definition specialized to a height field.
  const Point& p = vg->point;
  float px = p[0], py = p[1], pz = p[2];
  const Point& e = _refp._eye;
  float pex = px - e[0], pey = py - e[1], pez = pz - e[2];
  float pez2 = pez * pez;
  float pem2 = pex * pex + pey * pey + pez2;
  float rhs1 = _refp._tz2 * pem2;
  return vspl->dir_error_mag2 * (pem2 - pez2) >= rhs1 * pem2;
#elif 0
  // Remove the term that checks the z direction of the view direction.
  const Point& p = vg->point;
  float px = p[0], py = p[1], pz = p[2];
  const Point& e = _refp._eye;
  float pex = px - e[0], pey = py - e[1], pez = pz - e[2];
  float pem2 = pex * pex + pey * pey + pez * pez;
  float rhs1 = _refp._tz2 * pem2;
  return vspl->dir_error_mag2 >= rhs1;
#else
  // Try linear functional instead of Euclidean (z direction still ignored).
  // This allows eyepoint anticipation for correct geomorphs.
  float lf = _refp._eyedir.eval(vg->point);
  return vspl->dir_error_mag2 >= _refp._tz2 * square(lf);
#endif
}

// ***

bool SrMesh::qrefine(const SrVertex* vs) const {
  ASSERTX(is_splitable(vs) && is_active_v(vs));
  const SrVertexGeometry* vg = refined_vg(vs->avertex);
  const SrVsplit* vspl = &_vsplits[vs->vspli];
  return is_visible(vg, vspl) && big_error(vg, vspl);
}

bool SrMesh::qcoarsen(const SrVertex* vt) const {
  ASSERTX(is_active_v(vt));
  ASSERTX(get_vt(vt->parent->vspli) == vt);  // left child of its parent!
  SrVertex* vs = vt->parent;
  const SrVsplit* vspl = &_vsplits[vs->vspli];
  const SrVertexGeometry* vg = refined_vg(vt->avertex);
  return !(is_visible(vg, vspl) && big_error(vg, vspl));
}

void SrMesh::apply_vspl(SrVertex* vs, EListNode*& pn) {
  ASSERTX(vspl_legal(vs));
  SrAVertex* vta = vs->avertex;  // vsa becomes vta!
  if (vta->vmorph && vta->vmorph->coarsening) abort_coarsen_morphing(vta->vertex);
  ASSERTX(!(vta->vmorph && vta->vmorph->coarsening));
  SrAVertex* vua = new SrAVertex;
  SrAFacePair* fpair = new SrAFacePair;
  SrAFace* fla = &fpair->pair[0];
  SrAFace* fra = &fpair->pair[1];
  int vspli = vs->vspli;
  SrVsplit* vspl = &_vsplits[vspli];
  SrVertex* vt = get_vt(vspli);
  SrVertex* vu = vt + 1;
  vs->avertex = nullptr;
  vt->avertex = vta;
  vu->avertex = vua;
  vta->vertex = vt;
  vua->vertex = vu;
  vua->visible = vta->visible;
  vua->cached_time = 0;
  if (vua->visible && _refine_morph_time) {
    vua->vmorph = new SrVertexMorph();
    SrVertexMorph* vm = vua->vmorph;
    vm->coarsening = false;
    int time = _refine_morph_time;
    vm->time = static_cast<short>(time - 1);
    float frac = 1.f / time;
    // vm->vgrefined = vspl->vu_vgeom;
    // vua->vgeom = vta->vgeom;
    float* lp = vua->vgeom.point.data();
    const float* cp = vta->vgeom.point.data();
    const float* ogp = vspl->vu_vgeom.point.data();
    float* gp = vm->vgrefined.point.data();
    float* ip = vm->vginc.point.data();
    // for_int(c, 6 - 3 * b_nor001) ip[c] = (gp[c] - cp[c]) * frac;
    // faster code because possible aliasing is avoided:
    float cp0, cp1, cp2, cp3, cp4, cp5;
    if (1) {
      cp0 = cp[0];
      cp1 = cp[1];
      cp2 = cp[2];
    }
    if (!b_nor001) {
      cp3 = cp[3];
      cp4 = cp[4];
      cp5 = cp[5];
    }
    if (1) {
      lp[0] = cp0;
      lp[1] = cp1;
      lp[2] = cp2;
    }
    if (!b_nor001) {
      lp[3] = cp3;
      lp[4] = cp4;
      lp[5] = cp5;
    }
    float gp0, gp1, gp2, gp3, gp4, gp5;
    if (1) {
      gp0 = ogp[0];
      gp1 = ogp[1];
      gp2 = ogp[2];
    }
    if (!b_nor001) {
      gp3 = ogp[3];
      gp4 = ogp[4];
      gp5 = ogp[5];
    }
    if (1) {
      gp[0] = gp0;
      gp[1] = gp1;
      gp[2] = gp2;
    }
    if (!b_nor001) {
      gp[3] = gp3;
      gp[4] = gp4;
      gp[5] = gp5;
    }
    if (1) {
      ip[0] = (gp0 - cp0) * frac;
      ip[1] = (gp1 - cp1) * frac;
      ip[2] = (gp2 - cp2) * frac;
    }
    if (!b_nor001) {
      ip[3] = (gp3 - cp3) * frac;
      ip[4] = (gp4 - cp4) * frac;
      ip[5] = (gp5 - cp5) * frac;
    }
  } else {
#if defined(SR_NO_VSGEOM)
    vta->vgeom = vspl->vt_vgeom;
#endif
    vua->vgeom = vspl->vu_vgeom;
    delete vua->vmorph;
    vua->vmorph = nullptr;
  }
  SrFace* fl = get_fl(vspli);
  SrFace* fr = fl + 1;
  fl->aface = fla;
  fr->aface = fra;
  SrAFace* flclw = vspl->fn[1]->aface;
  SrAFace* frccw = vspl->fn[3]->aface;
  SrAVertex* vla;
  SrAVertex* vra = nullptr;
  {
    SrAFace* fa = flclw;
    int j0 = get_vf_j0(vta, fa);
    fa->fnei[mod3(j0 + 1)] = fla;  // fa == flclw
    vla = fa->vertices[mod3(j0 + 2)];
    fa->vertices[j0] = vua;
    for (;;) {
      if (fa != frccw) {
        fa = fa->fnei[mod3(j0 + 2)];
        if (fa == &_isolated_aface) break;
        j0 = get_vf_j0(vta, fa);
        fa->vertices[j0] = vua;
        continue;
      }
      fa->fnei[mod3(j0 + 2)] = fra;  // fa == frccw
      vra = fa->vertices[mod3(j0 + 1)];
      break;
    }
  }
  SrAFace* flccw = vspl->fn[0]->aface;
  SrAFace* frclw = vspl->fn[2]->aface;
  ASSERTX(flccw == &_isolated_aface || flccw->fnei[get_vf_j2(vta, flccw)] == flclw);
  ASSERTX(frclw == &_isolated_aface || frclw->fnei[get_vf_j1(vta, frclw)] == frccw);
  if (flccw != &_isolated_aface) flccw->fnei[get_vf_j2(vta, flccw)] = fla;
  if (frclw != &_isolated_aface) frclw->fnei[get_vf_j1(vta, frclw)] = fra;
  fla->vertices[0] = vta;
  fla->vertices[1] = vua;
  fla->vertices[2] = vla;
  fla->fnei[0] = flclw;
  fla->fnei[1] = flccw;
  fla->fnei[2] = fra;
  fra->vertices[0] = vta;
  fra->vertices[1] = vra;  // undefined if frccw == &_isolated_aface (ok)
  fra->vertices[2] = vua;
  fra->fnei[0] = frccw;
  fra->fnei[1] = fla;
  fra->fnei[2] = frclw;
  fla->activef.link_before(_active_faces.delim());
#if !defined(SR_PREDICT_MATID)
  fla->matid = vspl->fl_matid | _cur_frame_mask;
#else
  fla->matid = flclw->matid;
#endif
  if (frccw != &_isolated_aface) {
    fra->activef.link_before(_active_faces.delim());
#if !defined(SR_PREDICT_MATID)
    fra->matid = vspl->fr_matid | _cur_frame_mask;
#else
    fra->matid = frccw->matid;
#endif
    ASSERTX(vra);
    if (vra->vmorph && vra->vmorph->coarsening) perhaps_abort_coarsen_morphing(vra->vertex);
  } else {
    fr->aface = &_isolated_aface;     // correction
    fla->fnei[2] = &_isolated_aface;  // correction
    --_num_active_faces;              // anticipating correction
  }
  if (vla->vmorph && vla->vmorph->coarsening) perhaps_abort_coarsen_morphing(vla->vertex);
  EListNode* n = pn;
  if (is_splitable(vu)) {
    vua->activev.link_after(n);
  } else {
    vua->activev.link_before(n);
  }
  if (is_splitable(vt)) {
    if (n != &vta->activev) {
      vta->activev.relink_after(n);
    }
  } else {
    if (n == &vta->activev) pn = n->next();
  }
  _num_active_vertices++;
  _num_active_faces += 2;
}

void SrMesh::apply_ecol(SrVertex* vs, EListNode*& pn) {
  ASSERTX(ecol_legal(get_vt(vs->vspli)));
  int vspli = vs->vspli;
  SrVsplit* vspl = &_vsplits[vspli];
  SrFace* fl = get_fl(vspli);
  (fl + 0)->aface->activef.unlink();
  SrVertex* vt = get_vt(vspli);
  SrAVertex* vsa = (vt + 0)->avertex;  // vta becomes vsa!
  SrAVertex* vua = (vt + 1)->avertex;
  vua->activev.unlink();
  ASSERTX(!(vsa->vmorph && vsa->vmorph->coarsening));
  ASSERTX(!(vua->vmorph && vua->vmorph->coarsening));
  // vua may in fact be refine-morphing if apply_ecol() is called directly
  //  from adapt_refinement() without coarsen-morphing.
  vs->avertex = vsa;
  (vt + 0)->avertex = nullptr;
  (vt + 1)->avertex = nullptr;
  vsa->vertex = vs;
  SrAFace* flccw = vspl->fn[0]->aface;
  SrAFace* flclw = vspl->fn[1]->aface;
  if (flccw != &_isolated_aface) get_fnei(flccw, (fl + 0)->aface) = flclw;
  SrAFace* frclw = vspl->fn[2]->aface;
  SrAFace* frccw = vspl->fn[3]->aface;
  if (frclw != &_isolated_aface) get_fnei(frclw, (fl + 1)->aface) = frccw;
#if defined(SR_NO_VSGEOM)
  vsa->vgeom = vspl->vs_vgeom;
#endif
  {
    SrAFace* fa = flclw;
    int j0 = get_vf_j0(vua, fa);
    fa->fnei[mod3(j0 + 1)] = flccw;
    fa->vertices[j0] = vsa;
    for (;;) {
      if (fa != frccw) {
        fa = fa->fnei[mod3(j0 + 2)];
        if (fa == &_isolated_aface) break;
        j0 = get_vf_j0(vua, fa);
        fa->vertices[j0] = vsa;
        continue;
      }
      fa->fnei[mod3(j0 + 2)] = frclw;
      break;
    }
  }
  delete vua->vmorph;
  delete vua;
  EListNode* n = pn;
  {
    SrVertex* vlp = (fl + 0)->aface->vertices[2]->vertex->parent;
    if (vlp) {
      SrVertex* vlpt = get_vt(vlp->vspli);
      SrAVertex* vlpta = (vlpt + 0)->avertex;
      if (vlpta && (vlpt + 1)->avertex) {
        if (n != &vlpta->activev) {
          vlpta->activev.relink_after(n);
        }
      }
    }
  }
  if (frccw != &_isolated_aface) {
    (fl + 1)->aface->activef.unlink();
    --_num_active_faces;
    SrVertex* vrp = (fl + 1)->aface->vertices[1]->vertex->parent;
    if (vrp) {
      SrVertex* vrpt = get_vt(vrp->vspli);
      SrAVertex* vrpta = (vrpt + 0)->avertex;
      if (vrpta && (vrpt + 1)->avertex) {
        if (n != &vrpta->activev) {
          vrpta->activev.relink_after(n);
        }
      }
    }
  }
  {
    SrVertex* vsp = vs->parent;
    if (!vsp) {
      if (n == &vsa->activev) pn = n->next();
    } else {
      SrVertex* vspt = get_vt(vsp->vspli);  // vs's sibling (or itself)
      SrAVertex* vspta = (vspt + 0)->avertex;
      if (vspta && (vspt + 1)->avertex) {
        // Reconsider vspt + 0 if not next node.
        if (n != &vspta->activev) {
          vspta->activev.relink_after(n);
        }
      } else {
        // Skip past vsa if current node.
        if (n == &vsa->activev) pn = n->next();
      }
    }
  }
  SrAFacePair* fpair = reinterpret_cast<SrAFacePair*>(fl->aface);
  delete fpair;
  (fl + 0)->aface = &_isolated_aface;
  (fl + 1)->aface = &_isolated_aface;
  --_num_active_vertices;
  --_num_active_faces;
}

GMesh SrMesh::extract_gmesh() const {
  GMesh gmesh;
  string str;
  for (SrAVertex* va : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    int vi = narrow_cast<int>(va->vertex - _vertices.data());
    Vertex gv = gmesh.create_vertex_private(vi + 1);
    gmesh.set_point(gv, va->vgeom.point);
    const Vector& nor = va->vgeom.vnormal;
    gmesh.update_string(gv, "normal", csform_vec(str, nor));
  }
  Array<Vertex> gvaa;
  // Should not use HH_ELIST_RANGE(_active_faces, SrAFace, fa) because we
  //  would not get reproducible face id's (no SrAFace* -> SrFace* info).
  for_int(fi, _faces.num()) {
    SrAFace* fa = _faces[fi].aface;
    if (fa == &_isolated_aface) continue;  // inactive or !creates_2faces()
    gvaa.init(0);
    for_int(j, 3) {
      int vi = narrow_cast<int>(fa->vertices[j]->vertex - _vertices.data());
      gvaa.push(gmesh.id_vertex(vi + 1));
    }
    Face gf = gmesh.create_face_private(fi + 1, gvaa);
    dummy_use(gf);
    gmesh.set_string(gf, _materials.get(unsigned(fa->matid) & ~k_Face_visited_mask).c_str());
  }
  return gmesh;
}

void SrMesh::ok() const {
  Set<const SrAVertex*> setva;
  {
    for (SrAVertex* va : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
      assertx(_vertices.ok(va->vertex));
      assertx(va->vertex->avertex == va);
      assertx(setva.add(va));
    }
    assertx(setva.num() == num_active_vertices());
    int numv = 0;
    for_int(vi, _vertices.num()) {
      const SrVertex* v = &_vertices[vi];
      const SrAVertex* va = v->avertex;
      if (!va) continue;
      numv++;
      assertx(setva.contains(va));
    }
    assertx(numv == num_active_vertices());
  }
  Set<const SrAFace*> setfa;
  {
    for (SrAFace* fa : HH_ELIST_RANGE(_active_faces, SrAFace, activef)) {
      assertx(fa != &_isolated_aface);
      assertx(setfa.add(fa));
    }
    assertx(setfa.num() == num_active_faces());
    int numf = 0;
    for_int(fi, _faces.num()) {
      const SrFace* f = &_faces[fi];
      const SrAFace* fa = f->aface;
      if (fa == &_isolated_aface) continue;
      numf++;
      assertx(setfa.contains(fa));
    }
    assertx(numf == num_active_faces());
  }
  {
    for (SrAFace* fa : HH_ELIST_RANGE(_active_faces, SrAFace, activef)) {
      for_int(j, 3) {
        const SrAVertex* va = fa->vertices[j];
        assertx(setva.contains(va));
        const SrAFace* fna = fa->fnei[mod3(j + 2)];
        if (fna != &_isolated_aface) {
          assertx(setfa.contains(fna));
          assertx(fna->fnei[get_vf_j1(va, fna)] == fa);
        }
      }
    }
  }
  for_int(vi, _vertices.num()) {
    const SrVertex* v = &_vertices[vi];
    if (is_active_v(v)) {
      for (SrVertex* vp = v->parent; vp; vp = vp->parent) {
        assertx(!is_active_v(vp));
        assertx(has_been_split(vp));
        const SrVsplit* vpspl = &_vsplits[vp->vspli];
        for_int(i, 4) {
          if (vpspl->fn[i] != &_isolated_face) assertx(is_active_f(vpspl->fn[i]));
        }
      }
      if (is_splitable(v)) {
        int vspli = v->vspli;
        assertx(!is_active_v(get_vt(vspli) + 0));
        assertx(!is_active_v(get_vt(vspli) + 1));
        assertx(!is_active_f(get_fl(vspli) + 0));
        assertx(!is_active_f(get_fl(vspli) + 1));
      }
    }
  }
  assertx(!is_active_f(&_isolated_face));
  assertx(_isolated_aface.matid == k_illegal_matid);
  assertx(_isolated_aface.fnei[0] == &_isolated_aface);
  assertx(_isolated_aface.fnei[1] == &_isolated_aface);
  assertx(_isolated_aface.fnei[2] == &_isolated_aface);
}

void SrMesh::set_view_params(const SrViewParams& vp) {
  assertx(vp.ok());
  bool activate_lr_planes = vp._activate_lr_planes;
  bool activate_bottom_plane = vp._activate_bottom_plane;
  bool activate_top_plane = vp._activate_top_plane;
  float hither = vp._hither;
  float yonder = vp._yonder;
  _view_params = vp;
  // Now create new _refp:
  const Frame& frame = vp._frame;
  // Note: frame.p() == Point(0.f, 0.f, 0.f) * frame
  _refp._nplanes = 0;
  if (activate_lr_planes) {
    // Left
    _refp._planes[_refp._nplanes++] =
        LinearFunc(ok_normalized(cross(frame.p(), Point(1.f, +vp._zoomyx[1], -1.f) * frame,
                                       Point(1.f, +vp._zoomyx[1], +1.f) * frame)),
                   frame.p());
    // Right
    _refp._planes[_refp._nplanes++] =
        LinearFunc(ok_normalized(cross(frame.p(), Point(1.f, -vp._zoomyx[1], +1.f) * frame,
                                       Point(1.f, -vp._zoomyx[1], -1.f) * frame)),
                   frame.p());
  }
  if (activate_bottom_plane) {
    // Bottom
    _refp._planes[_refp._nplanes++] =
        LinearFunc(ok_normalized(cross(frame.p(), Point(1.f, -1.f, -vp._zoomyx[0]) * frame,
                                       Point(1.f, +1.f, -vp._zoomyx[0]) * frame)),
                   frame.p());
  }
  if (activate_top_plane) {
    // Top
    _refp._planes[_refp._nplanes++] =
        LinearFunc(ok_normalized(cross(frame.p(), Point(1.f, +1.f, +vp._zoomyx[0]) * frame,
                                       Point(1.f, -1.f, +vp._zoomyx[0]) * frame)),
                   frame.p());
  }
  if (hither >= 0.f) {
    // Front (hither)
    _refp._planes[_refp._nplanes++] = LinearFunc(frame.v(0), frame.p() + hither * frame.v(0));
  }
  if (yonder >= 0.f) {
    // Rear (yonder)
    _refp._planes[_refp._nplanes++] = LinearFunc(-frame.v(0), frame.p() + yonder * frame.v(0));
  }
  Point old_eye = _refp._eye;
  Point cur_eye = frame.p();
  _refp._eye = cur_eye;
  if (old_eye[0] == BIGFLOAT) old_eye = cur_eye;
  _refp._tz2 = square(vp._screen_thresh * min(vp._zoomyx));
  Point fut_eye = interp(cur_eye, old_eye, 1.f + float(_refine_morph_time));  // extrapolate
  _refp._eyedir = LinearFunc(frame.v(0), cur_eye);
  // SHOW(_refp._eyedir.eval(fut_eye));
  static const bool sr_geomorph_anticipation = getenv_bool("SR_GEOMORPH_ANTICIPATION");
  if (sr_geomorph_anticipation && _refp._eyedir.eval(fut_eye) > 0.f) _refp._eyedir = LinearFunc(frame.v(0), fut_eye);
}

void SrMesh::set_initial_view_params() {
  assertx(num_active_faces() == _base_faces.num());  // else fully_coarsen();
  _refp._eye = Point(BIGFLOAT, 0.f, 0.f);
}

void SrMesh::fully_refine() {
  int bu_rmt = _refine_morph_time;
  set_refine_morph_time(0);
  int bu_cmt = _coarsen_morph_time;
  set_coarsen_morph_time(0);
  // Full refinement by looping over ordered vsplits.
  for (int vi = _base_vertices.num(); vi < _vertices.num(); vi += 2) {
    SrVertex* vt = &_vertices[vi];
    SrVertex* vs = vt->parent;
    if (!is_active_v(vs)) continue;
    {
      EListNode* n = _active_vertices.delim();
      apply_vspl(vs, n);
    }
    if (k_debug && 0) ok();
  }
  set_refine_morph_time(bu_rmt);
  set_coarsen_morph_time(bu_cmt);
}

void SrMesh::fully_coarsen() {
  int bu_rmt = _refine_morph_time;
  set_refine_morph_time(0);
  int bu_cmt = _coarsen_morph_time;
  set_coarsen_morph_time(0);
  // Full coarsening by looping over ordered ecols.
  for (int vi = _vertices.num() - 2; vi >= _base_vertices.num(); vi -= 2) {
    SrVertex* vt = &_vertices[vi];
    if (!is_active_v(vt)) continue;
    SrVertex* vs = vt->parent;
    {
      EListNode* n = &vt->avertex->activev;
      apply_ecol(vs, n);
    }
    if (k_debug && 0) ok();
  }
  set_refine_morph_time(bu_rmt);
  set_coarsen_morph_time(bu_cmt);
}

void SrMesh::verify_optimality() const {
  HH_ATIMER("____verify_optimality");
  for (SrAVertex* vsa : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    SrVertex* vs = vsa->vertex;
    if (!is_splitable(vs)) continue;
    if (qrefine(vs)) Warning("** should refine");
  }
  for (SrAVertex* vta : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    SrVertex* vt = vta->vertex;
    SrVertex* vs = vt->parent;
    if (!vs) continue;
    if (get_vt(vs->vspli) != vt) continue;
    if (!ecol_legal(vt)) continue;
    if (qcoarsen(vt)) {
      if (!(vta->vmorph && vta->vmorph->coarsening)) Warning("** should coarsen");
    } else {
      if (vta->vmorph && vta->vmorph->coarsening) Warning("** should not be coarsen-morphing");
    }
  }
}

void SrMesh::force_vsplit(SrVertex* vsf, EListNode*& n) {
  Stack<SrVertex*> stack_split;
  stack_split.push(vsf);
  while (!stack_split.empty()) {
    SrVertex* vs = stack_split.top();
    if (has_been_split(vs)) {
      stack_split.pop();
    } else if (!has_been_created(vs)) {
      SrVertex* vp = vs->parent;
      ASSERTX(vp);
      stack_split.push(vp);
    } else {
      SrVsplit* vspl = &_vsplits[vs->vspli];
      for_int(i, 4) {
        SrFace* fn = vspl->fn[i];
        if (!is_active_f(fn) && fn != &_isolated_face) stack_split.push(get_vt(get_vspli(fn))->parent);
      }
      if (stack_split.top() == vs) {
        stack_split.pop();
        ASSERTX((vs == vsf) == stack_split.empty());
        apply_vspl(vs, n);
      }
    }
  }
}

// Given:   0001110101000
// Returns: 0000000001000
static inline unsigned lsb_mask(unsigned size) {
  int count = 0;
  while ((size & 1) == 0) {
    count++;
    size >>= 1;
  }
  return 1u << count;
}

void SrMesh::adapt_refinement(int pnvtraverse) {
  // too slow. HH_ATIMER("____adapt_ref_f");
  _ar_tobevisible.init(0);
  uintptr_t left_child_mask = lsb_mask(sizeof(SrVertex));
  uintptr_t left_child_result = reinterpret_cast<uintptr_t>(_quick_first_vt) & left_child_mask;
  int nvtraverse = pnvtraverse;
  bool is_modified = false;
  EListNode* ndelim = _active_vertices.delim();
  EListNode* n = ndelim->next();
  for (;;) {
    if (n == ndelim) break;
    SrAVertex* vsa = HH_ELIST_OUTER(SrAVertex, activev, n);
    SrVertex* vs = vsa->vertex;
    if (!nvtraverse--) break;
    n = n->next();
    const SrVertexGeometry* rvg = refined_vg(vsa);
    bool new_vis = false;
    if (is_splitable(vs)) {
      const SrVsplit* cvspl = &_vsplits[vs->vspli];
      new_vis = is_visible(rvg, cvspl);
      if (!new_vis) {
        vsa->visible = false;
      } else {
        if (big_error(rvg, cvspl)) {
          is_modified = true;
          // Variable tn to help SGI compiler assign n to register.
          EListNode* tn = n->prev();
          force_vsplit(vs, tn);
          n = tn;
          continue;
        }
        if (!vsa->visible) _ar_tobevisible.push(vs);
      }
    } else {
#if defined(SR_SW_CULLING)
      vsa->visible = true;
#else
      // new_vis = false;
      // vsa->visible does not matter.
#endif
    }
    SrVertex* vsp = vs->parent;
    // if (!vsp || get_vt(vsp->vspli) != vs || !ecol_legal(vs)) continue;
    if (!vsp) continue;
    if ((reinterpret_cast<uintptr_t>(vs) & left_child_mask) != left_child_result) continue;
    ASSERTX(!(vsa->vmorph && vsa->vmorph->coarsening) || ecol_legal(vs));
    if (!is_active_v(vs + 1)) continue;
    // Now considering coarsening a left child with an active sibling.
    int pvspli = vsp->vspli;
    const SrFace* fl = get_fl(pvspli);
    const SrAFace* fla = fl->aface;
    const SrVsplit* pvspl = &_vsplits[pvspli];
    if (fla->fnei[1] != pvspl->fn[0]->aface || fla->fnei[0] != pvspl->fn[1]->aface) continue;
    const SrFace* fr = fl + 1;
    const SrAFace* fra = fr->aface;
    if (fra->fnei[2] != pvspl->fn[2]->aface || fra->fnei[0] != pvspl->fn[3]->aface) continue;
    // Simpler version for below that avoids geomorph coarsening:
    // if (qcoarsen(vs)) {
    //     EListNode* tn = n->prev(); apply_ecol(vsp, tn); n = tn;
    // }
    SrVertexMorph* vm = vsa->vmorph;
#if defined(SR_NO_VSGEOM)
    rvg = &pvspl->vs_vgeom;
    new_vis = false;
#endif
    if (!new_vis && !is_visible(rvg, pvspl)) {  // instant. coarsening
      is_modified = true;
      if (vm && vm->coarsening) {
        finish_vmorph(vsa);
        SrAVertex* vua = (vs + 1)->avertex;
        ASSERTX(vua->vmorph && vua->vmorph->coarsening);
        finish_vmorph(vua);
      }
      EListNode* tn = n->prev();
      apply_ecol(vsp, tn);
      n = tn;
    } else if (big_error(rvg, pvspl)) {  // no need to coarsen
      if (vm && vm->coarsening) abort_coarsen_morphing(vs);
    } else {  // geomorph coarsening
      is_modified = true;
      if (vm && vm->coarsening) {
        if (vm->time) continue;  // still coarsen-morphing
        finish_vmorph(vsa);
        SrAVertex* vua = (vs + 1)->avertex;
        ASSERTX(vua->vmorph && vua->vmorph->coarsening && vua->vmorph->time == 0);
        finish_vmorph(vua);
        EListNode* tn = n->prev();
        apply_ecol(vsp, tn);
        n = tn;
      } else if (!_coarsen_morph_time) {
        EListNode* tn = n->prev();
        apply_ecol(vsp, tn);
        n = tn;
      } else {
        start_coarsen_morphing(vs);
      }
    }
  }
#if defined(SR_SW_CULLING)
  {
    EListNode* n_bu = n;
    for (;;) {
      if (n == ndelim) break;
      SrAVertex* vsa = HH_ELIST_OUTER(SrAVertex, activev, n);
      SrVertex* vs = vsa->vertex;
      n = n->next();
      if (vsa->visible) continue;
      const SrVertexGeometry* rvg = refined_vg(vsa);
      if (is_splitable(vs)) {
        const SrVsplit* cvspl = &_vsplits[vs->vspli];
        if (is_visible(rvg, cvspl)) vsa->visible = true;
      } else {
        vsa->visible = true;
      }
    }
    n = n_bu;
  }
#endif
  _was_modified = is_modified;
  bool visited_whole_list = n == ndelim;
  dummy_use(visited_whole_list);
  for (SrVertex* vs : _ar_tobevisible) {
    if (vs->avertex) vs->avertex->visible = true;
  }
  // Move beginning of list right before next node.
  if (n != ndelim) ndelim->relink_before(n);
  if (k_debug && visited_whole_list) verify_optimality();
  // Note to D3D developers:
  // It would be great to do the update_vmorphs as part of the traversal
  //  of vertices to construct the vertex buffer for DrawPrimitive!
  update_vmorphs();
  if (k_debug && 1) ok();
}

void SrMesh::start_coarsen_morphing(SrVertex* vt) {
  ASSERTX(ecol_legal(vt));
  const SrVertexGeometry* coarsened_vg = refined_vg(vt->avertex);
  for_int(i, 2) {
    SrAVertex* va = (vt + i)->avertex;
    SrVertexMorph* vm = va->vmorph;
    if (!vm) {
      va->vmorph = new SrVertexMorph();
      vm = va->vmorph;
      vm->vgrefined = va->vgeom;
      // if i == 0, left child is not going to morph, so may want to
      //  do something more efficient than adding zero's every frame.
    } else {
      ASSERTX(!vm->coarsening);
    }
    vm->coarsening = true;
    int time = _coarsen_morph_time;
    vm->time = static_cast<short>(time - 1);
    float frac = 1.f / time;
    vm->vginc.point = to_Point((coarsened_vg->point - va->vgeom.point) * frac);
    if (!b_nor001) vm->vginc.vnormal = (coarsened_vg->vnormal - va->vgeom.vnormal) * frac;
    // const float* cp = va->vgeom.point.data();
    // const float* gp = coarsened_vg->point.data();
    // float* ip = vm->vginc.point.data();
    // for_int(c, 6 - 3 * b_nor001) ip[c] = (gp[c] - cp[c]) * frac;
  }
}

void SrMesh::abort_coarsen_morphing(SrVertex* vc) {
  // Vertex vc may be either left child or right child.
  SrVertex* vp = vc->parent;
  SrVertex* vt = get_vt(vp->vspli);
  for_int(i, 2) {
    SrVertex* vti = vt + i;
    SrAVertex* va = vti->avertex;
    SrVertexMorph* vm = va->vmorph;
    ASSERTX(vm && vm->coarsening);
    // Now set the vertex to refine-morph to its desired position.
    vm->coarsening = false;
    int time = _refine_morph_time;
    vm->time = static_cast<short>(time - 1);
    float frac = 1.f / time;
    vm->vginc.point = to_Point((vm->vgrefined.point - va->vgeom.point) * frac);
    if (!b_nor001) vm->vginc.vnormal = (vm->vgrefined.vnormal - va->vgeom.vnormal) * frac;
    // const float* cp = va->vgeom.point.data();
    // const float* gp = vm->vgrefined.point.data();
    // float* ip = vm->vginc.point.data();
    // for_int(c, 6 - 3 * b_nor001) ip[c] = (gp[c] - cp[c]) * frac;
  }
}

void SrMesh::perhaps_abort_coarsen_morphing(SrVertex* vc) {
  SrVertex* vp = vc->parent;
  SrVertex* vt = get_vt(vp->vspli);
  if (!ecol_legal(vt)) abort_coarsen_morphing(vc);
}

void SrMesh::set_refine_morph_time(int refine_morph_time) {
  assertx(!refine_morph_time || refine_morph_time > 1);
  if (_refine_morph_time && !refine_morph_time) {
    // Snap refine-morphing vertices to their final positions.
    for (SrAVertex* va : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
      if (!va->vmorph) continue;
      if (va->vmorph->coarsening) continue;
      // Snap vertices forward to their (new) refined positions.
      finish_vmorph(va);
    }
  }
  _refine_morph_time = refine_morph_time;
  assertx(_refine_morph_time <= 32767);
}

void SrMesh::set_coarsen_morph_time(int coarsen_morph_time) {
  assertx(!coarsen_morph_time || coarsen_morph_time > 1);
  if (_coarsen_morph_time && !coarsen_morph_time) {
    // Perform ecol's for coarsen-morphing vertices.
    for (SrAVertex* va : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
      if (!va->vmorph) continue;
      if (!va->vmorph->coarsening) continue;
      // Snap vertices backward to their (original) refined positions.
      finish_vmorph(va);
    }
  }
  _coarsen_morph_time = coarsen_morph_time;
  assertx(_coarsen_morph_time <= 32767);
}

void SrMesh::update_vmorphs() {
  _num_vertices_refine_morphing = 0;
  _num_vertices_coarsen_morphing = 0;
  if (!_refine_morph_time && !_coarsen_morph_time) return;
  int num_vertices_refine_morphing = 0;
  int num_vertices_coarsen_morphing = 0;
  EListNode* ndelim = _active_vertices.delim();
  for (EListNode* n = ndelim->next();;) {
    if (n == ndelim) break;
    SrAVertex* va = HH_ELIST_OUTER(SrAVertex, activev, n);
    n = n->next();
    SrVertexMorph* vm = va->vmorph;
    if (!vm) continue;
    if (vm->time) {
      vm->time -= 1;
      num_vertices_refine_morphing += !vm->coarsening;
      num_vertices_coarsen_morphing += vm->coarsening;
      float* cp = va->vgeom.point.data();
      const float* ip = vm->vginc.point.data();
      // for_int(c, 6 - 3 * b_nor001) cp[c] += ip[c];
      // faster code because possible aliasing is avoided:
      float cp0, cp1, cp2, cp3, cp4, cp5;
      if (1) {
        cp0 = cp[0];
        cp1 = cp[1];
        cp2 = cp[2];
      }
      if (!b_nor001) {
        cp3 = cp[3];
        cp4 = cp[4];
        cp5 = cp[5];
      }
      float ip0, ip1, ip2, ip3, ip4, ip5;
      if (1) {
        ip0 = ip[0];
        ip1 = ip[1];
        ip2 = ip[2];
      }
      if (!b_nor001) {
        ip3 = ip[3];
        ip4 = ip[4];
        ip5 = ip[5];
      }
      if (1) {
        cp[0] = cp0 + ip0;
        cp[1] = cp1 + ip1;
        cp[2] = cp2 + ip2;
      }
      if (!b_nor001) {
        cp[3] = cp3 + ip3;
        cp[4] = cp4 + ip4;
        cp[5] = cp5 + ip5;
      }
    } else {
      ASSERTX(vm->time == 0);
      // Vertex could be coarsen-morphing and not caught in
      //  adapt_refinement if pnvtraverse < std::numeric_limits<int>::max().
      if (!vm->coarsening) finish_vmorph(va);
    }
  }
  _num_vertices_refine_morphing = num_vertices_refine_morphing;
  _num_vertices_coarsen_morphing = num_vertices_coarsen_morphing;
}

bool SrMesh::is_still_morphing() const { return _num_vertices_refine_morphing || _num_vertices_coarsen_morphing; }

bool SrMesh::is_still_adapting() const { return _was_modified; }

int SrMesh::num_vertices_refine_morphing() const { return _num_vertices_refine_morphing; }

int SrMesh::num_vertices_coarsen_morphing() const { return _num_vertices_coarsen_morphing; }

bool SrMesh::verify_all_faces_visited() const {
  for (SrAFace* f : HH_ELIST_RANGE(_active_faces, SrAFace, activef)) {
    assertx((unsigned(f->matid) & k_Face_visited_mask) == _cur_frame_mask);
  }
  return true;
}

bool SrMesh::verify_all_vertices_uncached() const {
  for (SrAVertex* v : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    assertx(v->cached_time < _cache_time);
  }
  return true;
}

// *** Geomorphs

void SrMesh::construct_geomorph(SrGeomorphInfo& geoinfo) {
  assertx(!_refine_morph_time && !_coarsen_morph_time);
  Map<SrVertex*, SrVertexGeometry> m_v_vg;
  // Record the geometry of the current set of active vertices.
  for (SrAVertex* va : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    m_v_vg.enter(va->vertex, va->vgeom);
  }
  // Apply vspl's.
  {
    HH_ATIMER("__geo_vspls");
    EListNode* ndelim = _active_vertices.delim();
    for (EListNode* n = ndelim->next(); n != ndelim; n = n->next()) {
      SrAVertex* vsa = HH_ELIST_OUTER(SrAVertex, activev, n);
      SrVertex* vs = vsa->vertex;
      if (!is_splitable(vs) || !qrefine(vs)) continue;
      force_vsplit(vs, n);
      n = n->prev();
    }
  }
  // Compute geoinfo._ancestors[0]
  for (SrAVertex* va : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    SrVertex* v = va->vertex;
    SrVertex* vv = v;
    while (!m_v_vg.contains(vv)) vv = assertx(vv->parent);
    if (vv != v) geoinfo._ancestors[0].enter(v, m_v_vg.get(vv));
  }
  m_v_vg.clear();
  // Apply ecol's and record them in a sequence.
  Array<SrVertex*> seq_ecols;
  {
    HH_ATIMER("__geo_ecols");
    EListNode* ndelim = _active_vertices.delim();
    for (EListNode* n = ndelim->next(); n != ndelim; n = n->next()) {
      SrAVertex* vsa = HH_ELIST_OUTER(SrAVertex, activev, n);
      SrVertex* vs = vsa->vertex;
      SrVertex* vsp = vs->parent;
      if (!vsp || get_vt(vsp->vspli) != vs || !ecol_legal(vs)) continue;
      if (!qcoarsen(vs)) continue;
      seq_ecols.push(vsp);
      apply_ecol(vsp, n);
      n = n->prev();
    }
  }
  // Record the geometry of the current set of active vertices.
  for (SrAVertex* va : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    m_v_vg.enter(va->vertex, va->vgeom);
  }
  // Undo ecol's by traversing sequence backwards.
  for (int i = seq_ecols.num() - 1; i >= 0; --i) {
    SrVertex* vs = seq_ecols[i];
    assertx(vspl_legal(vs));
    {
      EListNode* n = _active_vertices.delim();
      apply_vspl(vs, n);
    }
  }
  // Compute geoinfo._ancestors[1]
  for (SrAVertex* va : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    SrVertex* v = va->vertex;
    SrVertex* vv = v;
    while (!m_v_vg.contains(vv)) vv = assertx(vv->parent);
    if (vv != v) geoinfo._ancestors[1].enter(v, m_v_vg.get(vv));
  }
}

GMesh SrMesh::extract_gmesh(const SrGeomorphInfo& geoinfo) const {
  GMesh gmesh;
  string str;
  for (SrAVertex* va : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    SrVertex* v = va->vertex;
    int vi = narrow_cast<int>(v - _vertices.data());
    Vertex gv = gmesh.id_vertex(vi + 1);
    {
      bool present;
      const SrVertexGeometry* vg = &geoinfo._ancestors[0].retrieve(v, present);
      if (!present) vg = &va->vgeom;
      const Point& p = vg->point;
      gmesh.update_string(gv, "Opos", csform_vec(str, p));
      const Vector& n = vg->vnormal;
      gmesh.update_string(gv, "Onormal", csform_vec(str, n));
    }
    {
      bool present;
      const SrVertexGeometry* vg = &geoinfo._ancestors[1].retrieve(v, present);
      if (!present) vg = &va->vgeom;
      const Point& p = vg->point;
      gmesh.set_point(gv, p);
      const Vector& n = vg->vnormal;
      gmesh.update_string(gv, "normal", csform_vec(str, n));
    }
  }
  return gmesh;
}

// ***

#if defined(ANALYZE_PM_COMPRESSION_WITH_XIA_VARSHNEY)

// Analyze the compression of a PM file (like "FilterPM -reorder_vspl -compression")
//  except that the vspl dependencies are computed using the VDPM framework instead of the xia-varshney framework.

static inline int wrap_dflclw(int dflclw, int nfaces) {
  if (dflclw < -int((nfaces - 1) / 2)) dflclw += nfaces;
  if (dflclw > int(nfaces / 2)) dflclw -= nfaces;
  return dflclw;
}

int SrMesh::get_iflclw(SrVertex* vs) const {
  assertx(is_splitable(vs));
  const SrVsplit* vspl = &_vsplits[vs->vspli];
  const SrFace* flclw = vspl->fn[1];
  return narrow_cast<int>(flclw - _faces.data());
}

void SrMesh::refine_in_best_dflclw_order() {
  struct Scvspl {
    SrVertex* vs;
    int iflclw1;  // iflclw + 1 to reserve 0 for undefined Scvspl()
  };
  struct less_Scvspl {
    static int compare_Scvspl(const Scvspl& s1, const Scvspl& s2) {
      int i = s1.iflclw1 - s2.iflclw1;
      if (i) return i;
      assertx(s1.vs != s2.vs);
      return s1.vs < s2.vs ? -1 : 1;
    }
    bool operator()(const Scvspl& s1, const Scvspl& s2) const { return compare_Scvspl(s1, s2) < 0; }
  };
  STree<Scvspl, less_Scvspl> stcvspl;  // current legal vsplits
  int ncand = 0;                       // number in stcvspl
  for (SrAVertex* vsa : HH_ELIST_RANGE(_active_vertices, SrAVertex, activev)) {
    SrVertex* vs = vsa->vertex;
    if (!is_splitable(vs) || !vspl_legal(vs)) continue;
    Scvspl n;
    n.vs = vs;
    n.iflclw1 = get_iflclw(vs) + 1;
    assertx(stcvspl.enter(n));
    ncand++;
  }
  Array<SrVertex*> pncands;  // possible new candidates
  DeltaEncoding de_dflclw;
  Scvspl nlast;
  nlast.iflclw1 = 0;
  dummy_init(nlast.vs);
  while (!stcvspl.empty()) {
    HH_SSTAT(Sfpcand, _num_active_faces / float(ncand));
    Scvspl nmin;
    nmin = stcvspl.succ_eq(nlast);
    if (!nmin.iflclw1) nmin = stcvspl.min();
    SrVertex* vs = nmin.vs;
    SrVsplit* vspl = &_vsplits[vs->vspli];
    int dflclw = nmin.iflclw1 - nlast.iflclw1;
    nlast.iflclw1 = nmin.iflclw1;
    dflclw = wrap_dflclw(dflclw, _num_active_faces);
    if (0) showf("dflclw=%4d   ncand=%4d  nf=%5d\n", dflclw, ncand, _num_active_faces);
    HH_SSTAT(Sdflclw, dflclw);
    {
      float fdflclw = float(dflclw);
      de_dflclw.enter_vector(V(fdflclw));
    }
    assertx(stcvspl.remove(nmin));
    --ncand;
    {
      pncands.init(0);
      pncands.push(get_vt(vs->vspli) + 0);  // vt
      pncands.push(get_vt(vs->vspli) + 1);  // vu
      SrVertex* vl;
      SrVertex* vr = nullptr;
      SrAFace* fa = vspl->fn[1]->aface;
      vl = fa->vertices[get_vf_j2(vs->avertex, fa)]->vertex;
      fa = vspl->fn[3]->aface;
      if (fa != &_isolated_aface) vr = fa->vertices[get_vf_j1(vs->avertex, fa)]->vertex;
      pncands.push(vl);
      if (vr) pncands.push(vr);
    }
    {
      EListNode* n = _active_vertices.delim();
      apply_vspl(vs, n);
    }
    for (SrVertex* v : pncands) {
      if (!is_splitable(v) || !vspl_legal(v)) continue;
      Scvspl nt;
      nt.vs = v;
      nt.iflclw1 = get_iflclw(v) + 1;
      if (stcvspl.enter(nt)) ncand++;
    }
  }
  assertx(_num_active_vertices == _base_vertices.num() + _vsplits.num());
  de_dflclw.analyze("de_dflclw");
}

#endif  // defined(ANALYZE_PM_COMPRESSION_WITH_XIA_VARSHNEY)

}  // namespace hh
