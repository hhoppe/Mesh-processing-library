// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/SRMesh.h"

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

// Set in SRMesh.h
#if !defined(SR_NOR001)
constexpr bool b_nor001 = false;
#else
constexpr bool b_nor001 = true;
const Vector SRVertexGeometry::vnormal{0.f, 0.f, 1.f};
const float SRVsplit::uni_error_mag2 = 0.f;
const float SRVsplit::sin2alpha = 0.f;
#endif

}  // namespace

// *** MISC

// Analysis of memory requirement:
//
// SIGGRAPH97:
//  SRVertex    2n      19*4    == 152n bytes
//  SRFace      2n      9*4     == 72n  bytes
//  Total:                      == 224n bytes
//
//  example: mojave_sq600 (360K vertices): 80 MBytes
//
// (Compare with GMesh: 312n bytes + 28n bytes attribs)
// (Compare with WMesh: 68n bytes;  AWMesh:  92n bytes)
//
// Obvious compression scheme:
//  (SRVsplit, no fl field, no materials)
//  SRVertex    2n      10*4    == 80n  bytes
//  SRVsplit    n       8*4     == 32n  bytes
//  SRFace      2n      8*4     == 64n  bytes
//  Total:                      == 176n bytes
//
// Same + quantization (8bit on refine_info, 16bit on point/normal)
//  SRVertex    2n      7*4     == 56n  bytes
//  SRVsplit    n       4*4+8   == 20n  bytes
//  SRFace      2n      8*4     == 64n  bytes
//  Total:                      == 140n bytes
//
// NEW (implemented)
//  SRVertex    2n      3*4     == 24n  bytes
//  SRFace      2n      1*4     == 8n   bytes
//  SRVsplit    n       6*4+9*4 == 60n  bytes
//  SRAVertex   m       10*4    == 40m  bytes
//  SRAFace     2m      9*4     == 72m  bytes
//  SRAVertexM  g       4+12*4  == 52g  bytes
//  Total:                      == 92n + 112m + 52g bytes
//
// PAPER: same if defined(SR_NOR001) (no normals, no uni_error, no sin2alpha)
//           and defined(SR_PREDICT_MATID)
//  SRVertex    2n      3*4     == 24n  bytes
//  SRFace      2n      1*4     == 8n   bytes
//  SRVsplit    n       3*4+6*4 == 36n  bytes
//  SRAVertex   m       7*4     == 28m  bytes
//  SRAFace     2m      9*4     == 72m  bytes
//  SRAVertexM  g       4+6*4   == 28g  bytes
//  Total:                      == 68n + 100m + 28g bytes
//
// Same + short for SRAVertex* and SRAFace*
//  SRVertex    2n      10      == 20n  bytes
//  SRFace      2n      1*2     == 4n   bytes
//  SRVsplit    n       3*4+6*4 == 36n  bytes
//  SRAVertex   m       4+5*4   == 24m  bytes
//  SRAFace     2m      9*2     == 36m  bytes
//  SRAVertexM  g       4+6*4   == 28g  bytes
//  Total:                         60n + 60m + 28g bytes
//
// Same + quantization (16bit on static point, 8bit on dir_error and radius)
//  SRVertex    2n      10      == 20n  bytes
//  SRFace      2n      1*2     == 4n   bytes
//  SRVsplit    n   3*2+4*4+2*1 == 24n  bytes
//  SRAVertex   m       4+5*4   == 24m  bytes
//  SRAFace     2m      9*2     == 36m  bytes
//  SRAVertexM  g       4+6*4   == 28g  bytes
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
// Note that 'array_field-_array' requires an integer division,
//  which is very slow on R10K (~40 cycle stall).  I've managed to eliminate
//  these in all critical sections
//  (using "int SRVertex::vspli" instead of "SRVsplit* SRVertex::vspl")

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
//  (setenv NO_REGULATION; setenv NO_AMORTIZATION; demo_gcanyon -timestat -st gcanyon_4k2k_perf.s3d -geom 500x150 -sr_screen_thresh .00145 -key DS)
//   just display (little rasterization): 712000 faces / sec
//   with adapt_refinement:               460000 faces / sec

// TODO:
// - vertex geometries could be defined procedurally (eg. image, noise)
//    to reduce space.
// - reduce space: use shorts for SRVertex::avertex and SRFace::aface and
//    set maximum number of SRAVertex and SRAFace to 64Ki (problem:
//    fully_refine() is currently required during read_pm() ).
// - possibly: separate linked list for vmorph'ing SRAVertex nodes to speed
//     up update_vmorphs(), or better yet,
//     update_vmorphs when creating DrawIndexPrimitive
// - let screen-space tolerance vary with distance of surface from eye.

// Notes on software view frustum culling (SR_SW_CULLING):
// - Evaluating visibility is more difficult for leaf vertices, since
//  they don't have radius_neg values, so for them I just assign
//  vsa->visible = true.  I could also refer to the bounding
//  spheres of their parent vertices (leaf_bsphere_use_parent in RCS).
//  I don't know which gives better performance.
// - The test for face_invisible() must be "!vis(v0) && !vis(v1) && !vis(v2)"
//  instead of "!vis(v0) || !vis(v1) || !vis(v2)" because the positions of the
//  vertices v_l and v_r are unknonwn due to the relaxed refinement constraints.
//  Very unfortunate since otherwise would easily know that some vertices
//  need not be entered into the DrawPrimitive() call (transform + lighting).
// - Even with this && test, some faces sometimes drop out erroneously
//  when geomorphs are turned on in demo_gcanyon.
// - Performance: seems to improve.  for the demos in Video.Notes,
//  I see an improvement from nf=10000 to nf=11300 for 36 fps flythrough.
// - Conclusion: for now, leave on since small perf improvement with only
//  very few visual errors.  How to handle DrawPrimitive() in future?

// *** SRViewParams

void SRViewParams::set_frame(const Frame& frame) { _frame = frame; }

void SRViewParams::set_zooms(const Vec2<float>& zoomyx) { _zoomyx = zoomyx; }

void SRViewParams::activate_lr_planes(bool flag) { _activate_lr_planes = flag; }

void SRViewParams::activate_bottom_plane(bool flag) { _activate_bottom_plane = flag; }

void SRViewParams::activate_top_plane(bool flag) { _activate_top_plane = flag; }

void SRViewParams::set_hither(float hither) { _hither = hither; }

void SRViewParams::set_yonder(float yonder) { _yonder = yonder; }

void SRViewParams::set_screen_thresh(float screen_thresh) { _screen_thresh = screen_thresh; }

bool SRViewParams::ok() const { return _frame[0][0] != BIGFLOAT && min(_zoomyx) > 0.f; }

// *** SRMesh

HH_ALLOCATE_POOL(SRAVertex);

HH_ALLOCATE_POOL(SRAFacePair);

HH_ALLOCATE_POOL(SRVertexMorph);

SRFace SRMesh::_isolated_face;    // initialized in SRMesh::SRMesh()
SRAFace SRMesh::_isolated_aface;  // initialized in SRMesh::SRMesh()

inline SRAFace* SRMesh::rotate_clw(SRAFace* f, SRAVertex* v) const {
  // Could implement this functionality using fnei comparisons.
  return f->fnei[get_vf_j2(v, f)];
}

inline SRAFace* SRMesh::rotate_ccw(SRAFace* f, SRAVertex* v) const {
  // Could implement this functionality using fnei comparisons.
  return f->fnei[get_vf_j1(v, f)];
}

inline const SRVertex* SRMesh::get_vt(int vspli) const {
  ASSERTX(_vsplits.ok(vspli));
  return _quick_first_vt + vspli * 2;
}

inline SRVertex* SRMesh::get_vt(int vspli) {
  ASSERTX(_vsplits.ok(vspli));
  return _quick_first_vt + vspli * 2;
}

inline const SRFace* SRMesh::get_fl(int vspli) const {
  ASSERTX(_vsplits.ok(vspli));
  return _quick_first_fl + vspli * 2;
}

inline SRFace* SRMesh::get_fl(int vspli) {
  ASSERTX(_vsplits.ok(vspli));
  return _quick_first_fl + vspli * 2;
}

inline int SRMesh::get_vspli(const SRFace* fl) const {
  ASSERTX(_faces.ok(fl));
  return int((fl - _quick_first_fl) / 2);  // slow but seldom
}

inline bool SRMesh::is_active_f(const SRFace* f) const { return f->aface != &_isolated_aface; }

inline bool SRMesh::has_been_created(const SRVertex* v) const {
  return !v->parent || is_active_f(get_fl(v->parent->vspli));
}

inline bool SRMesh::has_been_split(const SRVertex* v) const {
  return is_splitable(v) && is_active_f(get_fl(v->vspli));
}

inline bool SRMesh::is_active_v(const SRVertex* v) const {
  ASSERTX((v->avertex != nullptr) == (has_been_created(v) && !has_been_split(v)));
  return v->avertex != nullptr;
}

inline bool SRMesh::creates_2faces(const SRVsplit* vspl) const { return vspl->fn[3] != &_isolated_face; }

inline const SRVertexGeometry* SRMesh::refined_vg(const SRAVertex* va) const {
  const SRVertexGeometry* vg = &va->vgeom;
  if (va->vmorph) vg = &va->vmorph->vgrefined;
  return vg;
}

inline void SRMesh::finish_vmorph(SRAVertex* va) {
  va->vgeom = va->vmorph->vgrefined;
  va->vmorph = nullptr;
}

SRMesh::SRMesh() {
  // Note: _view_params & _refp are set in set_initial_view_params().
  // Initialize static objects.
  _isolated_aface.matid = k_illegal_matid;
  for_int(j, 3) _isolated_aface.fnei[j] = &_isolated_aface;
  _isolated_face.aface = &_isolated_aface;
}

SRMesh::~SRMesh() {
  if (1) {
    fully_coarsen();
    {  // to avoid ~EList(): not empty
      for_int(vi, _base_vertices.num()) _vertices[vi].avertex->activev.unlink();
      assertx(_active_vertices.empty());
      for_int(fi, _base_faces.num()) _faces[fi].aface->activef.unlink();
      assertx(_active_faces.empty());
    }
  } else {
    // Quick destruction involves deleting all SRAVertex and SRAFacePair which are not in base mesh.
    // Traverse active vertices:
    // - delete any active vertex if its upwards branch towards the base mesh includes a right-child relation.
    // - to delete faces, uniquely identify internal nodes (forest above active vertices) by
    //    considering nodes accessible with only right-child relations.
    EListNode* ndelim = _active_vertices.delim();
    for (EListNode* n = ndelim->next(); n != ndelim;) {
      SRAVertex* va = EListOuter(SRAVertex, activev, n);
      n = n->next();  // advance here before node gets deleted
      bool branch_has_left_child = false;
      bool branch_has_right_child = false;
      for (SRVertex* v = va->vertex;;) {
        SRVertex* vp = v->parent;
        if (!vp) break;
        ASSERTX(is_splitable(vp));
        int vspli = vp->vspli;
        bool is_left_child = get_vt(vspli) == v;
        if (!is_left_child) {
          branch_has_right_child = true;
          if (branch_has_left_child) break;
          SRFace* f = get_fl(vspli);
          ASSERTX(is_active_f(f));
          SRAFace* fa = f->aface;
          SRAFacePair* fpair = reinterpret_cast<SRAFacePair*>(fa);
          delete fpair;
        } else {
          branch_has_left_child = true;
          if (branch_has_right_child) break;
        }
        v = vp;
      }
      if (branch_has_right_child) {
        // va->vmorph is deleted automatically
        delete va;
      } else {
        ASSERTX(!va->vmorph);
      }
    }
  }
  // optimize: instead, could associate the two Pools (SRAVertex, SRAFacePair) with this object,
  //  and simply deallocate the pools.
}

bool SRMesh::vspl_legal(const SRVertex* vs) const {
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
  const SRVsplit* vspl = &_vsplits[vspli];
  if ((!is_active_f(vspl->fn[0]) && vspl->fn[0] != &_isolated_face) || !is_active_f(vspl->fn[1]) ||
      (!is_active_f(vspl->fn[2]) && vspl->fn[2] != &_isolated_face) ||
      (!is_active_f(vspl->fn[3]) && vspl->fn[3] != &_isolated_face))
    return false;
  return true;
}

bool SRMesh::ecol_legal(const SRVertex* vt) const {
  // Warning: this function has been adapted inline in adapt_refinement()
  //  for maximum efficiency.
  const SRVertex* vs = vt->parent;
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
  const SRFace* fl = get_fl(vspli);
  const SRAFace* fla = fl->aface;
  const SRVsplit* vspl = &_vsplits[vspli];
  if (fla->fnei[1] != vspl->fn[0]->aface || fla->fnei[0] != vspl->fn[1]->aface) return false;
  const SRFace* fr = fl + 1;
  const SRAFace* fra = fr->aface;
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

void SRMesh::display_hierarchy_height() const {
  Array<int> ar_height(_vertices.num());
  int max_height = 0;
  for_int(vi, _vertices.num()) {
    const SRVertex* vs = &_vertices[vi];
    const SRVertex* vp = vs->parent;
    int height = !vp ? 1 : ar_height[narrow_cast<int>(vp - _vertices.data())] + 1;
    ar_height[vi] = height;
    if (k_debug)
      if (vp) assertx(vp < vs);
    if (height > max_height) max_height = height;
  }
  showdf("vertex hierarchy height=%d\n", max_height);
}

void SRMesh::read_pm(PMeshRStream& pmrs) {
  HH_TIMER(__read_pm);
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
  HH_TIMER(___read_convert);
  Array<SRVertexGeometry> vgeoms(_vertices.num());  // all vertex geometries
  // Process the base mesh.
  {
    assertx(bmesh._vertices.num() == bmesh._wedges.num());
    for_int(wi, bmesh._wedges.num()) {
      int vi = wi;
      assertx(bmesh._wedges[wi].vertex == vi);
      SRAVertex* va = &_base_vertices[vi];
      _vertices[vi].avertex = va;
      _vertices[vi].parent = nullptr;
      _vertices[vi].vspli = -1;
      va->activev.link_before(_active_vertices.delim());
      va->vertex = &_vertices[vi];
      va->vgeom.point = bmesh._vertices[vi].attrib.point;
      va->vgeom.vnormal = bmesh._wedges[vi].attrib.normal;
      if (b_nor001) assertx(Vector(0.f, 0.f, 1.f) == va->vgeom.vnormal);
      va->vmorph = nullptr;
      va->visible = false;
      va->cached_time = 0;
      vgeoms[vi] = va->vgeom;
    }
    for_int(fi, bmesh._faces.num()) {
      SRAFace* fa = &_base_faces[fi];
      _faces[fi].aface = fa;
      fa->activef.link_before(_active_faces.delim());
      fa->matid = bmesh._faces[fi].attrib.matid;
      assertx(_materials.ok(fa->matid));
    }
    for_int(fi, bmesh._faces.num()) {
      SRAFace* fa = _faces[fi].aface;
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
    SRVsplit* vspl = &_vsplits[vspli];
    SRAVertex* vsa;
    int flclwi = f_pm2sr[pm_vspl.flclw];
    assertx(pm_vspl.vlr_offset1 > 0);  // flclw non-existent is now illegal
    SRFace* flclw = &_faces[flclwi];
    ASSERTX(is_active_f(flclw));
    vsa = flclw->aface->vertices[vs_index];
    vspl->fn[1] = flclw;
    {
      SRAFace* fa1 = rotate_ccw(flclw->aface, vsa);
      vspl->fn[0] = fa1 == &_isolated_aface ? &_isolated_face : &_faces[fa1->matid];
      if (fa1 != &_isolated_aface) assertx(is_active_f(vspl->fn[0]));
    }
    if (pm_vspl.vlr_offset1 == 1) {
      vspl->fn[2] = &_isolated_face;
      vspl->fn[3] = &_isolated_face;
    } else {
      SRAFace* fa = flclw->aface;
      for_int(count, pm_vspl.vlr_offset1 - 2) {
        fa = rotate_clw(fa, vsa);
        ASSERTX(fa != &_isolated_aface && fa != flclw->aface);
      }
      vspl->fn[3] = &_faces[fa->matid];
      fa = rotate_clw(fa, vsa);
      vspl->fn[2] = fa == &_isolated_aface ? &_isolated_face : &_faces[fa->matid];
      if (fa != &_isolated_aface) assertx(is_active_f(vspl->fn[2]));
    }
    SRVertex* vs = vsa->vertex;
    vs->vspli = vspli;
    SRVertex* vt = get_vt(vspli);
    for_int(i, 2) {
      (vt + i)->avertex = nullptr;
      (vt + i)->parent = vs;
      (vt + i)->vspli = -1;
    }
    SRFace* fl = get_fl(vspli);
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
    vspl->fr_matid = narrow_cast<short>(
        !cr2faces ? -1 : (code & Vsplit::FRN_MASK) ? pm_vspl.fr_matid : f_matid[vspl->fn[3]->aface->matid]);
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
      apply_vspl(vs, n);  // apply vsplit on SRMesh
    }
    for_int(i, 2) {
      int matid = (fl + i)->aface->matid;
      if (!i || cr2faces) {
        // 20121212 This section appears to be broken, because aface matid are used to store face_id's,
        //  and these same matid are used to predict matid for newly introduced faces in apply_vspl().
        // In summary, support of > 1 material seems broken.
        if (0) assertw(_materials.ok(matid));
      }
      // 20121212 added mask (_isolated_aface.matid == k_illegal_matid or bad matid)
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
  HH_TIMER_END(___read_convert);
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
  _bbox.clear();
  for_int(vi, _vertices.num()) _bbox.union_with(vgeoms[vi].point);
  display_hierarchy_height();
  if (k_debug) ok();
  {
    // Coarsen now so that performance statistics don't get skewed.
    // Also we usually never care about the fully detailed mesh.
    HH_TIMER(___read_coarsen);
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

void SRMesh::compute_bspheres(CArrayView<SRVertexGeometry> vgeoms) {
  HH_TIMER(___compute_bspheres);
  // spheres bounding positions over surface.
  Array<BoundingSphere> ar_bsphere(_vertices.num());
  // Compute bound(star(v)) of vertices in fully refined mesh.
  {
    // based on shirman-abi-ezzi93.
    Array<Bbox> ar_bbox(_vertices.num());
    for_int(vi, _vertices.num()) {
      if (!is_active_v(&_vertices[vi])) continue;
      ar_bbox[vi][0] = ar_bbox[vi][1] = _vertices[vi].avertex->vgeom.point;
    }
    for_int(fi, _faces.num()) {
      SRAFace* fa = _faces[fi].aface;
      if (fa == &_isolated_aface) continue;  // !creates_2faces()
      Vec3<int> ar_vi;
      for_int(j, 3) {
        // division in here may be slow.
        ar_vi[j] = narrow_cast<int>(fa->vertices[j]->vertex - _vertices.data());
      }
      for_int(j, 3) {
        int j0 = j, j1 = mod3(j0 + 1);
        SRAVertex* va0 = fa->vertices[j0];
        SRAVertex* va1 = fa->vertices[j1];
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
      SRAFace* fa = _faces[fi].aface;
      if (fa == &_isolated_aface) continue;  // !creates_2faces()
      Vec3<int> ar_vi;
      for_int(j, 3) {
        // division in here may be slow.
        ar_vi[j] = narrow_cast<int>(fa->vertices[j]->vertex - _vertices.data());
      }
      for_int(j, 3) {
        int j0 = j, j1 = mod3(j0 + 1);
        SRAVertex* va0 = fa->vertices[j0];
        SRAVertex* va1 = fa->vertices[j1];
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
    SRVertex* vt = get_vt(vspli);
    SRVertex* vs = vt->parent;
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
    SRVertex* vs = &_vertices[vi];
    if (!is_splitable(vs)) continue;
    SRVsplit* vspl = &_vsplits[vs->vspli];
    vspl->radius_neg = -(dist(vgeoms[vi].point, ar_bsphere[vi].point) + ar_bsphere[vi].radius);
    if (sr_no_frustum_test) vspl->radius_neg = -BIGFLOAT;
  }
}

void SRMesh::compute_nspheres(CArrayView<SRVertexGeometry> vgeoms) {
  HH_TIMER(___compute_nspheres);
  // spheres bounding normals over surface.
  Array<BoundingSphere> ar_nsphere(_vertices.num());
  // Compute bound(star(v)) of vertices in fully refined mesh.
  {
    Array<Bbox> ar_bbox(_vertices.num());
    Array<Vector> ar_fnormal(_faces.num());  // cache face normals
    for_int(fi, _faces.num()) {
      SRAFace* fa = _faces[fi].aface;
      if (fa == &_isolated_aface) continue;  // !creates_2faces()
      const Point& p0 = fa->vertices[0]->vgeom.point;
      const Point& p1 = fa->vertices[1]->vgeom.point;
      const Point& p2 = fa->vertices[2]->vgeom.point;
      Vector fnormal = cross(p0, p1, p2);
      assertw(fnormal.normalize());
      ar_fnormal[fi] = fnormal;
      for_int(j, 3) {
        SRVertex* v = fa->vertices[j]->vertex;
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
      SRAFace* fa = _faces[fi].aface;
      if (fa == &_isolated_aface) continue;  // !creates_2faces()
      for_int(j, 3) {
        SRVertex* v = fa->vertices[j]->vertex;
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
    SRVertex* vt = get_vt(vspli);
    SRVertex* vs = vt->parent;
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
    SRVertex* vs = &_vertices[vi];
    if (!is_splitable(vs)) continue;
    SRVsplit* vspl = &_vsplits[vs->vspli];
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
    vspl->sin2alpha = square(sin(alpha));
  }
}

void SRMesh::write_srm(std::ostream& os) const {
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
    const SRVertexGeometry* vg = &_vertices[vi].avertex->vgeom;
    write_binary_std(os, vg->point.view());
    write_binary_std(os, vg->vnormal.view());
  }
  for_int(fi, _base_faces.num()) {
    const SRAFace* fa = &_base_faces[fi];
    for_int(j, 3) write_binary_std(os, ArView(narrow_cast<int>(fa->vertices[j] - _base_vertices.data())));
    for_int(j, 3) {
      unsigned fni = fa->fnei[j] != &_isolated_aface ? narrow_cast<int>(fa->fnei[j] - _base_faces.data()) + 1 : 0;
      write_binary_std(os, ArView(fni));
    }
    write_binary_std(os, ArView(unsigned(fa->matid & ~k_Face_visited_mask)));
  }
  // Write out vsplits.
  for_int(vspli, _vsplits.num()) {
    const SRVsplit* vspl = &_vsplits[vspli];
    const SRVertex* vs = assertx(get_vt(vspli)->parent);
    write_binary_std(os, ArView(narrow_cast<int>(vs - _vertices.data())));
    for_int(j, 4) {
      unsigned fni = vspl->fn[j] != &_isolated_face ? narrow_cast<int>(vspl->fn[j] - _faces.data()) + 1 : 0;
      write_binary_std(os, ArView(fni));
    }
#if !defined(SR_PREDICT_MATID)
    write_binary_std(os, ArView(narrow_cast<ushort>(vspl->fl_matid & ~k_Face_visited_mask)));
    write_binary_std(os, ArView(narrow_cast<ushort>(vspl->fr_matid & ~k_Face_visited_mask)));
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

void SRMesh::read_srm(std::istream& is) {
  HH_TIMER(__read_srm);
  assertx(!_base_vertices.num());
  assertx(!_refine_morph_time && !_coarsen_morph_time);  // just to be safe
  // Read past comments.
  for (string sline;;) {
    assertx(my_getline(is, sline));
    if (sline == "" || sline[0] == '#') continue;
    assertx(sline == "SRM");
    break;
  }
  // Read in sizes.
  {
    int bnv, bnf, nvspl;
    string sline;
    assertx(my_getline(is, sline));
    assertx(sscanf(sline.c_str(), "base_nvertices=%d base_nfaces=%d nvsplits=%d", &bnv, &bnf, &nvspl) == 3);
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
    string sline;
    assertx(my_getline(is, sline));
    assertx(sscanf(sline.c_str(), "bbox %g %g %g  %g %g %g", &_bbox[0][0], &_bbox[0][1], &_bbox[0][2], &_bbox[1][0],
                   &_bbox[1][1], &_bbox[1][2]) == 6);
  }
  // Read in materials.
  _materials.read(is);
  // Read in base mesh.
  for_int(vi, _base_vertices.num()) {
    SRAVertex* va = &_base_vertices[vi];
    _vertices[vi].avertex = va;
    _vertices[vi].parent = nullptr;
    _vertices[vi].vspli = -1;
    va->activev.link_before(_active_vertices.delim());
    va->vertex = &_vertices[vi];
    va->vmorph = nullptr;
    va->visible = false;
    va->cached_time = 0;
    assertx(read_binary_std(is, va->vgeom.point.view()));
    assertx(read_binary_std(is, va->vgeom.vnormal.view()));
    if (b_nor001) assertx(Vector(0.f, 0.f, 1.f) == va->vgeom.vnormal);
  }
  for_int(fi, _base_faces.num()) {
    SRAFace* fa = &_base_faces[fi];
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
    SRVsplit* vspl = &_vsplits[vspli];
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
    SRVertex* vs = &_vertices[vsi];
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
    SRVertex* vt = get_vt(vspli);
    for_int(i, 2) {
      (vt + i)->avertex = nullptr;
      (vt + i)->parent = vs;
      (vt + i)->vspli = -1;
    }
    SRFace* fl = get_fl(vspli);
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
//   Note: using Euclidean distance (pem2 below) is better in some ways
//    because it removes more faces on sides and behind viewer,
//   but using linear functional tzdist is more accurate for
//    approximating residual size under perspective projection.
//   As pointed out in Lindstrom-etal96, Euclidean distance tends
//    to coarsen objects away from the center of projection, especially
//    with a wide field of view.
//   So overall I prefer Euclidean distance.
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

inline bool SRMesh::is_visible(const SRVertexGeometry* vg, const SRVsplit* vspl) const {
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

inline bool SRMesh::big_error(const SRVertexGeometry* vg, const SRVsplit* vspl) const {
#if !defined(SR_NOR001)
  // original SIGGRAPH 97 definition
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
  // SIGGRAPH 97 definition specialized to height field
  const Point& p = vg->point;
  float px = p[0], py = p[1], pz = p[2];
  const Point& e = _refp._eye;
  float pex = px - e[0], pey = py - e[1], pez = pz - e[2];
  float pez2 = pez * pez;
  float pem2 = pex * pex + pey * pey + pez2;
  float rhs1 = _refp._tz2 * pem2;
  return vspl->dir_error_mag2 * (pem2 - pez2) >= rhs1 * pem2;
#elif 0
  // remove term that checks z direction of view direction
  const Point& p = vg->point;
  float px = p[0], py = p[1], pz = p[2];
  const Point& e = _refp._eye;
  float pex = px - e[0], pey = py - e[1], pez = pz - e[2];
  float pem2 = pex * pex + pey * pey + pez * pez;
  float rhs1 = _refp._tz2 * pem2;
  return vspl->dir_error_mag2 >= rhs1;
#elif 1
  // try linear functional instead of Euclidean (z direction still ignored)
  // this allows eyepoint anticipation for correct geomorphs.
  float lf = _refp._eyedir.eval(vg->point);
  return vspl->dir_error_mag2 >= _refp._tz2 * square(lf);
#else
  BUG;
#endif
}

// ***

bool SRMesh::qrefine(const SRVertex* vs) const {
  ASSERTX(is_splitable(vs) && is_active_v(vs));
  const SRVertexGeometry* vg = refined_vg(vs->avertex);
  const SRVsplit* vspl = &_vsplits[vs->vspli];
  return is_visible(vg, vspl) && big_error(vg, vspl);
}

bool SRMesh::qcoarsen(const SRVertex* vt) const {
  ASSERTX(is_active_v(vt));
  ASSERTX(get_vt(vt->parent->vspli) == vt);  // left child of its parent!
  SRVertex* vs = vt->parent;
  const SRVsplit* vspl = &_vsplits[vs->vspli];
  const SRVertexGeometry* vg = refined_vg(vt->avertex);
  return !(is_visible(vg, vspl) && big_error(vg, vspl));
}

void SRMesh::apply_vspl(SRVertex* vs, EListNode*& pn) {
  ASSERTX(vspl_legal(vs));
  SRAVertex* vta = vs->avertex;  // vsa becomes vta!
  if (vta->vmorph && vta->vmorph->coarsening) abort_coarsen_morphing(vta->vertex);
  ASSERTX(!(vta->vmorph && vta->vmorph->coarsening));
  SRAVertex* vua = new SRAVertex;
  SRAFacePair* fpair = new SRAFacePair;
  SRAFace* fla = &fpair->pair[0];
  SRAFace* fra = &fpair->pair[1];
  int vspli = vs->vspli;
  SRVsplit* vspl = &_vsplits[vspli];
  SRVertex* vt = get_vt(vspli);
  SRVertex* vu = vt + 1;
  vs->avertex = nullptr;
  vt->avertex = vta;
  vu->avertex = vua;
  vta->vertex = vt;
  vua->vertex = vu;
  vua->visible = vta->visible;
  vua->cached_time = 0;
  if (vua->visible && _refine_morph_time) {
    vua->vmorph = make_unique<SRVertexMorph>();
    SRVertexMorph* vm = vua->vmorph.get();
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
    vua->vmorph = nullptr;
  }
  SRFace* fl = get_fl(vspli);
  SRFace* fr = fl + 1;
  fl->aface = fla;
  fr->aface = fra;
  SRAFace* flclw = vspl->fn[1]->aface;
  SRAFace* frccw = vspl->fn[3]->aface;
  SRAVertex* vla;
  SRAVertex* vra = nullptr;
  {
    SRAFace* fa = flclw;
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
  SRAFace* flccw = vspl->fn[0]->aface;
  SRAFace* frclw = vspl->fn[2]->aface;
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
    if (n == &vta->activev) pn = n = n->next();
  }
  _num_active_vertices++;
  _num_active_faces += 2;
}

void SRMesh::apply_ecol(SRVertex* vs, EListNode*& pn) {
  ASSERTX(ecol_legal(get_vt(vs->vspli)));
  int vspli = vs->vspli;
  SRVsplit* vspl = &_vsplits[vspli];
  SRFace* fl = get_fl(vspli);
  (fl + 0)->aface->activef.unlink();
  SRVertex* vt = get_vt(vspli);
  SRAVertex* vsa = (vt + 0)->avertex;  // vta becomes vsa!
  SRAVertex* vua = (vt + 1)->avertex;
  vua->activev.unlink();
  ASSERTX(!(vsa->vmorph && vsa->vmorph->coarsening));
  ASSERTX(!(vua->vmorph && vua->vmorph->coarsening));
  // vua may in fact be refine-morphing if apply_ecol() is called directly
  //  from adapt_refinement() without coarsen-morphing.
  vs->avertex = vsa;
  (vt + 0)->avertex = nullptr;
  (vt + 1)->avertex = nullptr;
  vsa->vertex = vs;
  SRAFace* flccw = vspl->fn[0]->aface;
  SRAFace* flclw = vspl->fn[1]->aface;
  if (flccw != &_isolated_aface) get_fnei(flccw, (fl + 0)->aface) = flclw;
  SRAFace* frclw = vspl->fn[2]->aface;
  SRAFace* frccw = vspl->fn[3]->aface;
  if (frclw != &_isolated_aface) get_fnei(frclw, (fl + 1)->aface) = frccw;
#if defined(SR_NO_VSGEOM)
  vsa->vgeom = vspl->vs_vgeom;
#endif
  {
    SRAFace* fa = flclw;
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
  delete vua;
  EListNode* n = pn;
  {
    SRVertex* vlp = (fl + 0)->aface->vertices[2]->vertex->parent;
    if (vlp) {
      SRVertex* vlpt = get_vt(vlp->vspli);
      SRAVertex* vlpta = (vlpt + 0)->avertex;
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
    SRVertex* vrp = (fl + 1)->aface->vertices[1]->vertex->parent;
    if (vrp) {
      SRVertex* vrpt = get_vt(vrp->vspli);
      SRAVertex* vrpta = (vrpt + 0)->avertex;
      if (vrpta && (vrpt + 1)->avertex) {
        if (n != &vrpta->activev) {
          vrpta->activev.relink_after(n);
        }
      }
    }
  }
  {
    SRVertex* vsp = vs->parent;
    if (!vsp) {
      if (n == &vsa->activev) pn = n = n->next();
    } else {
      SRVertex* vspt = get_vt(vsp->vspli);  // vs's sibling (or itself)
      SRAVertex* vspta = (vspt + 0)->avertex;
      if (vspta && (vspt + 1)->avertex) {
        // Reconsider vspt + 0 if not next node.
        if (n != &vspta->activev) {
          vspta->activev.relink_after(n);
        }
      } else {
        // Skip past vsa if current node.
        if (n == &vsa->activev) pn = n = n->next();
      }
    }
  }
  SRAFacePair* fpair = reinterpret_cast<SRAFacePair*>(fl->aface);
  delete fpair;
  (fl + 0)->aface = &_isolated_aface;
  (fl + 1)->aface = &_isolated_aface;
  --_num_active_vertices;
  --_num_active_faces;
}

void SRMesh::extract_gmesh(GMesh& gmesh) const {
  assertx(!gmesh.num_vertices());
  string str;
  for (SRAVertex* va : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    int vi = narrow_cast<int>(va->vertex - _vertices.data());
    Vertex gv = gmesh.create_vertex_private(vi + 1);
    gmesh.set_point(gv, va->vgeom.point);
    const Vector& nor = va->vgeom.vnormal;
    gmesh.update_string(gv, "normal", csform_vec(str, nor));
  }
  Array<Vertex> gvaa;
  // Should not use EList_outer_range(_active_faces, SRAFace, fa) because we
  //  would not get reproducible face id's (no SRAFace* -> SRFace* info).
  for_int(fi, _faces.num()) {
    SRAFace* fa = _faces[fi].aface;
    if (fa == &_isolated_aface) continue;  // inactive or !creates_2faces()
    gvaa.init(0);
    for_int(j, 3) {
      int vi = narrow_cast<int>(fa->vertices[j]->vertex - _vertices.data());
      gvaa.push(gmesh.id_vertex(vi + 1));
    }
    Face gf = gmesh.create_face_private(fi + 1, gvaa);
    dummy_use(gf);
    gmesh.set_string(gf, _materials.get(fa->matid & ~k_Face_visited_mask).c_str());
  }
}

void SRMesh::ok() const {
  Set<const SRAVertex*> setva;
  {
    for (SRAVertex* va : EList_outer_range(_active_vertices, SRAVertex, activev)) {
      assertx(_vertices.ok(va->vertex));
      assertx(va->vertex->avertex == va);
      assertx(setva.add(va));
    }
    assertx(setva.num() == num_active_vertices());
    int numv = 0;
    for_int(vi, _vertices.num()) {
      const SRVertex* v = &_vertices[vi];
      const SRAVertex* va = v->avertex;
      if (!va) continue;
      numv++;
      assertx(setva.contains(va));
    }
    assertx(numv == num_active_vertices());
  }
  Set<const SRAFace*> setfa;
  {
    for (SRAFace* fa : EList_outer_range(_active_faces, SRAFace, activef)) {
      assertx(fa != &_isolated_aface);
      assertx(setfa.add(fa));
    }
    assertx(setfa.num() == num_active_faces());
    int numf = 0;
    for_int(fi, _faces.num()) {
      const SRFace* f = &_faces[fi];
      const SRAFace* fa = f->aface;
      if (fa == &_isolated_aface) continue;
      numf++;
      assertx(setfa.contains(fa));
    }
    assertx(numf == num_active_faces());
  }
  {
    for (SRAFace* fa : EList_outer_range(_active_faces, SRAFace, activef)) {
      for_int(j, 3) {
        const SRAVertex* va = fa->vertices[j];
        assertx(setva.contains(va));
        const SRAFace* fna = fa->fnei[mod3(j + 2)];
        if (fna != &_isolated_aface) {
          assertx(setfa.contains(fna));
          assertx(fna->fnei[get_vf_j1(va, fna)] == fa);
        }
      }
    }
  }
  for_int(vi, _vertices.num()) {
    const SRVertex* v = &_vertices[vi];
    if (is_active_v(v)) {
      for (SRVertex* vp = v->parent; vp; vp = vp->parent) {
        assertx(!is_active_v(vp));
        assertx(has_been_split(vp));
        const SRVsplit* vpspl = &_vsplits[vp->vspli];
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

void SRMesh::set_view_params(const SRViewParams& vp) {
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

void SRMesh::set_initial_view_params() {
  assertx(num_active_faces() == _base_faces.num());  // else fully_coarsen();
  _refp._eye = Point(BIGFLOAT, 0.f, 0.f);
}

void SRMesh::fully_refine() {
  int bu_rmt = _refine_morph_time;
  set_refine_morph_time(0);
  int bu_cmt = _coarsen_morph_time;
  set_coarsen_morph_time(0);
  // Full refinement by looping over ordered vsplits.
  for (int vi = _base_vertices.num(); vi < _vertices.num(); vi += 2) {
    SRVertex* vt = &_vertices[vi];
    SRVertex* vs = vt->parent;
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

void SRMesh::fully_coarsen() {
  int bu_rmt = _refine_morph_time;
  set_refine_morph_time(0);
  int bu_cmt = _coarsen_morph_time;
  set_coarsen_morph_time(0);
  // Full coarsening by looping over ordered ecols.
  for (int vi = _vertices.num() - 2; vi >= _base_vertices.num(); vi -= 2) {
    SRVertex* vt = &_vertices[vi];
    if (!is_active_v(vt)) continue;
    SRVertex* vs = vt->parent;
    {
      EListNode* n = &vt->avertex->activev;
      apply_ecol(vs, n);
    }
    if (k_debug && 0) ok();
  }
  set_refine_morph_time(bu_rmt);
  set_coarsen_morph_time(bu_cmt);
}

void SRMesh::verify_optimality() const {
  HH_ATIMER(____verify_optimality);
  for (SRAVertex* vsa : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    SRVertex* vs = vsa->vertex;
    if (!is_splitable(vs)) continue;
    if (qrefine(vs)) Warning("** should refine");
  }
  for (SRAVertex* vta : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    SRVertex* vt = vta->vertex;
    SRVertex* vs = vt->parent;
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

void SRMesh::force_vsplit(SRVertex* vsf, EListNode*& n) {
  Stack<SRVertex*> stack_split;
  stack_split.push(vsf);
  while (!stack_split.empty()) {
    SRVertex* vs = stack_split.top();
    if (has_been_split(vs)) {
      stack_split.pop();
    } else if (!has_been_created(vs)) {
      SRVertex* vp = vs->parent;
      ASSERTX(vp);
      stack_split.push(vp);
    } else {
      SRVsplit* vspl = &_vsplits[vs->vspli];
      for_int(i, 4) {
        SRFace* fn = vspl->fn[i];
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

void SRMesh::adapt_refinement(int pnvtraverse) {
  // too slow. HH_ATIMER(____adapt_ref_f);
  _ar_tobevisible.init(0);
  uintptr_t left_child_mask = lsb_mask(sizeof(SRVertex));
  uintptr_t left_child_result = reinterpret_cast<uintptr_t>(_quick_first_vt) & left_child_mask;
  int nvtraverse = pnvtraverse;
  bool is_modified = false;
  EListNode* ndelim = _active_vertices.delim();
  EListNode* n = ndelim->next();
  for (;;) {
    if (n == ndelim) break;
    SRAVertex* vsa = EListOuter(SRAVertex, activev, n);
    SRVertex* vs = vsa->vertex;
    if (!nvtraverse--) break;
    n = n->next();
    const SRVertexGeometry* rvg = refined_vg(vsa);
    bool new_vis = false;
    if (is_splitable(vs)) {
      const SRVsplit* cvspl = &_vsplits[vs->vspli];
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
    SRVertex* vsp = vs->parent;
    // if (!vsp || get_vt(vsp->vspli) != vs || !ecol_legal(vs)) continue;
    if (!vsp) continue;
    if ((reinterpret_cast<uintptr_t>(vs) & left_child_mask) != left_child_result) continue;
    ASSERTX(!(vsa->vmorph && vsa->vmorph->coarsening) || ecol_legal(vs));
    if (!is_active_v(vs + 1)) continue;
    // Now considering coarsening a left child with an active sibling.
    int pvspli = vsp->vspli;
    const SRFace* fl = get_fl(pvspli);
    const SRAFace* fla = fl->aface;
    const SRVsplit* pvspl = &_vsplits[pvspli];
    if (fla->fnei[1] != pvspl->fn[0]->aface || fla->fnei[0] != pvspl->fn[1]->aface) continue;
    const SRFace* fr = fl + 1;
    const SRAFace* fra = fr->aface;
    if (fra->fnei[2] != pvspl->fn[2]->aface || fra->fnei[0] != pvspl->fn[3]->aface) continue;
    // Simpler version for below that avoids geomorph coarsening:
    // if (qcoarsen(vs)) {
    //     EListNode* tn = n->prev(); apply_ecol(vsp, tn); n = tn;
    // }
    SRVertexMorph* vm = vsa->vmorph.get();
#if defined(SR_NO_VSGEOM)
    rvg = &pvspl->vs_vgeom;
    new_vis = false;
#endif
    if (!new_vis && !is_visible(rvg, pvspl)) {  // instant. coarsening
      is_modified = true;
      if (vm && vm->coarsening) {
        finish_vmorph(vsa);
        SRAVertex* vua = (vs + 1)->avertex;
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
        SRAVertex* vua = (vs + 1)->avertex;
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
      SRAVertex* vsa = EListOuter(SRAVertex, activev, n);
      SRVertex* vs = vsa->vertex;
      n = n->next();
      if (vsa->visible) continue;
      const SRVertexGeometry* rvg = refined_vg(vsa);
      if (is_splitable(vs)) {
        const SRVsplit* cvspl = &_vsplits[vs->vspli];
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
  for (SRVertex* vs : _ar_tobevisible) {
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

void SRMesh::start_coarsen_morphing(SRVertex* vt) {
  ASSERTX(ecol_legal(vt));
  const SRVertexGeometry* coarsened_vg = refined_vg(vt->avertex);
  for_int(i, 2) {
    SRAVertex* va = (vt + i)->avertex;
    SRVertexMorph* vm = va->vmorph.get();
    if (!vm) {
      va->vmorph = make_unique<SRVertexMorph>();
      vm = va->vmorph.get();
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

void SRMesh::abort_coarsen_morphing(SRVertex* vc) {
  // Vertex vc may be either left child or right child.
  SRVertex* vp = vc->parent;
  SRVertex* vt = get_vt(vp->vspli);
  for_int(i, 2) {
    SRVertex* vti = vt + i;
    SRAVertex* va = vti->avertex;
    SRVertexMorph* vm = va->vmorph.get();
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

void SRMesh::perhaps_abort_coarsen_morphing(SRVertex* vc) {
  SRVertex* vp = vc->parent;
  SRVertex* vt = get_vt(vp->vspli);
  if (!ecol_legal(vt)) abort_coarsen_morphing(vc);
}

void SRMesh::set_refine_morph_time(int refine_morph_time) {
  assertx(!refine_morph_time || refine_morph_time > 1);
  if (_refine_morph_time && !refine_morph_time) {
    // Snap refine-morphing vertices to their final positions.
    for (SRAVertex* va : EList_outer_range(_active_vertices, SRAVertex, activev)) {
      if (!va->vmorph) continue;
      if (va->vmorph->coarsening) continue;
      // Snap vertices forward to their (new) refined positions.
      finish_vmorph(va);
    }
  }
  _refine_morph_time = refine_morph_time;
  assertx(_refine_morph_time <= 32767);
}

void SRMesh::set_coarsen_morph_time(int coarsen_morph_time) {
  assertx(!coarsen_morph_time || coarsen_morph_time > 1);
  if (_coarsen_morph_time && !coarsen_morph_time) {
    // Perform ecol's for coarsen-morphing vertices.
    for (SRAVertex* va : EList_outer_range(_active_vertices, SRAVertex, activev)) {
      if (!va->vmorph) continue;
      if (!va->vmorph->coarsening) continue;
      // Snap vertices backward to their (original) refined positions.
      finish_vmorph(va);
    }
  }
  _coarsen_morph_time = coarsen_morph_time;
  assertx(_coarsen_morph_time <= 32767);
}

void SRMesh::update_vmorphs() {
  _num_vertices_refine_morphing = 0;
  _num_vertices_coarsen_morphing = 0;
  if (!_refine_morph_time && !_coarsen_morph_time) return;
  int num_vertices_refine_morphing = 0;
  int num_vertices_coarsen_morphing = 0;
  EListNode* ndelim = _active_vertices.delim();
  for (EListNode* n = ndelim->next();;) {
    if (n == ndelim) break;
    SRAVertex* va = EListOuter(SRAVertex, activev, n);
    n = n->next();
    SRVertexMorph* vm = va->vmorph.get();
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
      //  adapt_refinement if pnvtraverse<INT_MAX.
      if (!vm->coarsening) finish_vmorph(va);
    }
  }
  _num_vertices_refine_morphing = num_vertices_refine_morphing;
  _num_vertices_coarsen_morphing = num_vertices_coarsen_morphing;
}

bool SRMesh::is_still_morphing() const { return _num_vertices_refine_morphing || _num_vertices_coarsen_morphing; }

bool SRMesh::is_still_adapting() const { return _was_modified; }

int SRMesh::num_vertices_refine_morphing() const { return _num_vertices_refine_morphing; }

int SRMesh::num_vertices_coarsen_morphing() const { return _num_vertices_coarsen_morphing; }

bool SRMesh::verify_all_faces_visited() const {
  for (SRAFace* f : EList_outer_range(_active_faces, SRAFace, activef)) {
    assertx((f->matid & k_Face_visited_mask) == _cur_frame_mask);
  }
  return true;
}

bool SRMesh::verify_all_vertices_uncached() const {
  for (SRAVertex* v : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    assertx(v->cached_time < _cache_time);
  }
  return true;
}

// *** Geomorphs

void SRMesh::construct_geomorph(SRGeomorphInfo& geoinfo) {
  assertx(!_refine_morph_time && !_coarsen_morph_time);
  Map<SRVertex*, SRVertexGeometry> m_v_vg;
  // Record the geometry of the current set of active vertices.
  for (SRAVertex* va : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    m_v_vg.enter(va->vertex, va->vgeom);
  }
  // Apply vspl's.
  {
    HH_ATIMER(__geo_vspls);
    EListNode* ndelim = _active_vertices.delim();
    for (EListNode* n = ndelim->next(); n != ndelim; n = n->next()) {
      SRAVertex* vsa = EListOuter(SRAVertex, activev, n);
      SRVertex* vs = vsa->vertex;
      if (!is_splitable(vs) || !qrefine(vs)) continue;
      force_vsplit(vs, n);
      n = n->prev();
    }
  }
  // Compute geoinfo._ancestors[0]
  for (SRAVertex* va : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    SRVertex* v = va->vertex;
    SRVertex* vv = v;
    while (!m_v_vg.contains(vv)) vv = assertx(vv->parent);
    if (vv != v) geoinfo._ancestors[0].enter(v, m_v_vg.get(vv));
  }
  m_v_vg.clear();
  // Apply ecol's and record them in a sequence.
  Array<SRVertex*> seq_ecols;
  {
    HH_ATIMER(__geo_ecols);
    EListNode* ndelim = _active_vertices.delim();
    for (EListNode* n = ndelim->next(); n != ndelim; n = n->next()) {
      SRAVertex* vsa = EListOuter(SRAVertex, activev, n);
      SRVertex* vs = vsa->vertex;
      SRVertex* vsp = vs->parent;
      if (!vsp || get_vt(vsp->vspli) != vs || !ecol_legal(vs)) continue;
      if (!qcoarsen(vs)) continue;
      seq_ecols.push(vsp);
      apply_ecol(vsp, n);
      n = n->prev();
    }
  }
  // Record the geometry of the current set of active vertices.
  for (SRAVertex* va : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    m_v_vg.enter(va->vertex, va->vgeom);
  }
  // Undo ecol's by traversing sequence backwards.
  for (int i = seq_ecols.num() - 1; i >= 0; --i) {
    SRVertex* vs = seq_ecols[i];
    assertx(vspl_legal(vs));
    {
      EListNode* n = _active_vertices.delim();
      apply_vspl(vs, n);
    }
  }
  // Compute geoinfo._ancestors[1]
  for (SRAVertex* va : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    SRVertex* v = va->vertex;
    SRVertex* vv = v;
    while (!m_v_vg.contains(vv)) vv = assertx(vv->parent);
    if (vv != v) geoinfo._ancestors[1].enter(v, m_v_vg.get(vv));
  }
}

void SRMesh::extract_gmesh(GMesh& gmesh, const SRGeomorphInfo& geoinfo) const {
  extract_gmesh(gmesh);
  string str;
  for (SRAVertex* va : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    SRVertex* v = va->vertex;
    int vi = narrow_cast<int>(v - _vertices.data());
    Vertex gv = gmesh.id_vertex(vi + 1);
    {
      bool present;
      const SRVertexGeometry* vg = &geoinfo._ancestors[0].retrieve(v, present);
      if (!present) vg = &va->vgeom;
      const Point& p = vg->point;
      gmesh.update_string(gv, "Opos", csform_vec(str, p));
      const Vector& n = vg->vnormal;
      gmesh.update_string(gv, "Onormal", csform_vec(str, n));
    }
    {
      bool present;
      const SRVertexGeometry* vg = &geoinfo._ancestors[1].retrieve(v, present);
      if (!present) vg = &va->vgeom;
      const Point& p = vg->point;
      gmesh.set_point(gv, p);
      const Vector& n = vg->vnormal;
      gmesh.update_string(gv, "normal", csform_vec(str, n));
    }
  }
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

int SRMesh::get_iflclw(SRVertex* vs) const {
  assertx(is_splitable(vs));
  const SRVsplit* vspl = &_vsplits[vs->vspli];
  const SRFace* flclw = vspl->fn[1];
  return narrow_cast<int>(flclw - _faces.data());
}

void SRMesh::refine_in_best_dflclw_order() {
  struct Scvspl {
    SRVertex* vs;
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
  for (SRAVertex* vsa : EList_outer_range(_active_vertices, SRAVertex, activev)) {
    SRVertex* vs = vsa->vertex;
    if (!is_splitable(vs) || !vspl_legal(vs)) continue;
    Scvspl n;
    n.vs = vs;
    n.iflclw1 = get_iflclw(vs) + 1;
    assertx(stcvspl.enter(n));
    ncand++;
  }
  Array<SRVertex*> pncands;  // possible new candidates
  DeltaEncoding de_dflclw;
  Scvspl nlast;
  nlast.iflclw1 = 0;
  dummy_init(nlast.vs);
  while (!stcvspl.empty()) {
    HH_SSTAT(Sfpcand, _num_active_faces / float(ncand));
    Scvspl nmin;
    nmin = stcvspl.succ_eq(nlast);
    if (!nmin.iflclw1) nmin = stcvspl.min();
    SRVertex* vs = nmin.vs;
    SRVsplit* vspl = &_vsplits[vs->vspli];
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
      SRVertex* vl;
      SRVertex* vr = nullptr;
      SRAFace* fa = vspl->fn[1]->aface;
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
    for (SRVertex* v : pncands) {
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
