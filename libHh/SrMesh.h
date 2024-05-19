// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_SRMESH_H_
#define MESH_PROCESSING_LIBHH_SRMESH_H_

#include "libHh/Bbox.h"
#include "libHh/EList.h"
#include "libHh/LinearFunc.h"
#include "libHh/Map.h"
#include "libHh/Materials.h"
#include "libHh/Pixel.h"
#include "libHh/Pool.h"

#if 0
{
  SrMesh srmesh;
  srmesh.read_srm(std::cin);  // Or read_pm().
  srmesh.set_refine_morph_time(32);
  srmesh.set_coarsen_morph_time(16);
  for (;;) {
    SrViewParams vp;
    srmesh.set_view_params(vp);
    if (0) {
      SrGeomorphInfo geoinfo;
      srmesh.construct_geomorph(geoinfo);
      GMesh gmesh = srmesh.extract_gmesh(geoinfo);
    }
    srmesh.adapt_refinement();
    srmesh.ogl_render_faces_tvc(false);  // See all ogl_render_*().
    if (0) {
      GMesh gmesh = srmesh.extract_gmesh();
    }
    if (!(srmesh.is_still_morphing() || srmesh.is_still_adapting())) pause_for_input();
  }
}
#endif

namespace hh {

class PMeshRStream;
class GMesh;
struct SrAVertex;
struct SrAFace;

// True if input *.pm file was not created with '-minii2 -no_fit_geom'
// #define SR_NO_VSGEOM

// True if all normals are Vector(0.f, 0.f, 1.f), so no need to store them.
// #define SR_NOR001

// True if all matid in Vsplits are predicted correctly from fn[1] and fn[3].
#define SR_PREDICT_MATID

// Do not send render faces if provably outside frustum (see notes in .cpp)
// #define SR_SW_CULLING

// Almost never defined.
// #define ANALYZE_PM_COMPRESSION_WITH_XIA_VARSHNEY

#if defined(SR_NO_VSGEOM)
#undef SR_NOR001
#undef SR_PREDICT_MATID
#endif

struct SrVertexGeometry {
  Point point;
#if !defined(SR_NOR001)
  Vector vnormal;
#else
  static const Vector vnormal;        // always (0.f, 0.f, 1.f)
#endif
};

struct SrVertexMorph {
  bool coarsening;             // == !refining
  short time;                  // time - 1 left to achieve goal (always >= 0)
  SrVertexGeometry vgrefined;  // right child refined geometry
  SrVertexGeometry vginc;      // increment to add during morph
  HH_POOL_ALLOCATION(SrVertexMorph);
};
HH_INITIALIZE_POOL(SrVertexMorph);

struct SrVertex {
  SrAVertex* avertex;  // nullptr if not active
  SrVertex* parent;    // nullptr if in M^0
  int vspli;           // -1 if in M^n
};

struct SrAVertex {    // active vertex
  EListNode activev;  // at offset 0 for fast pointer conversion
  SrVertex* vertex;
  SrVertexGeometry vgeom;
  SrVertexMorph* vmorph{nullptr};  // avoid unique_ptr<> because we need standard layout for offsetof() in EListNode.
  bool visible;                    // was vertex visible when last traversed?
  int cached_time;                 // for transparent vertex caching
  HH_POOL_ALLOCATION(SrAVertex);
};
HH_INITIALIZE_POOL(SrAVertex);

struct SrFace {
  SrAFace* aface;  // nullptr if not active, == &_isolated_aface if !fr
};

struct SrAFace {      // active face
  EListNode activef;  // at offset 0 for fast pointer conversion
  int matid;          // see notes below.  up here for cache line
  Vec3<SrAVertex*> vertices;
  Vec3<SrAFace*> fnei;  // &_isolated_aface if no neighbor
                        // Notes on matid:
                        //  - a high bit is used for k_Face_visited_mask in SrMesh::*_render_*
                        //  - it is used as temporary 'int' storage in SrMesh::read_pm()
};

struct SrAFacePair {
  // Note that these two faces generally become unconnected through refinement.
  Vec2<SrAFace> pair;
  HH_POOL_ALLOCATION(SrAFacePair);
};
HH_INITIALIZE_POOL(SrAFacePair);

struct SrVsplit {
  SrVertexGeometry vu_vgeom;
#if defined(SR_NO_VSGEOM)
  SrVertexGeometry vs_vgeom, vt_vgeom;
#endif
  // fn[0..3] == {flccw, flclw, frclw, frccw}
  Vec4<SrFace*> fn;  // == &_inactive_face if no face expected
#if !defined(SR_PREDICT_MATID)
  short fl_matid;
  short fr_matid;
#endif
// Refinement criteria information
#if !defined(SR_NOR001)
  float uni_error_mag2;  // magnitude2 of uniform residual error
#else
  static const float uni_error_mag2;  // always 0.f
#endif
  float dir_error_mag2;  // magnitude2 of directional residual error
  float radius_neg;      // max radius of influence (negated)
#if !defined(SR_NOR001)
  float sin2alpha;  // backface region, square(sin(alpha))
#else
  static const float sin2alpha;       // always 0.f
#endif
};

struct SrRefineParams {
  // Frame _framei;            // to compute screen-space projections.
  int _nplanes;                // number of view frustum planes
  Vec<LinearFunc, 6> _planes;  // view frustum planes
  Point _eye;                  // eyepoint in world coordinates (frame.p())
  float _tz2;                  // screen-space tolerance zoomed and squared
  LinearFunc _eyedir;          // linear func along view direction
};

class SrGeomorphInfo : noncopyable {
 private:
  friend class SrMesh;
  Vec2<Map<SrVertex*, SrVertexGeometry>> _ancestors;
};

class SrViewParams {
 public:
  // frame.p() = eyepoint; frame.v(0) = view_direction; frame.v(1) = left_dir;
  //  frame.v(2) = top_dir   (frame must be orthonormal!)
  void set_frame(const Frame& frame);  // no default
  // zoomyx[1] is tan(angle) where angle is between frame.v(0) and left edge of viewport
  // zoomyx[0] is tan(angle) where angle is between frame.v(0) and top edge of viewport
  void set_zooms(const Vec2<float>& zoomyx);  // no default
  // enable left/right view frustum planes
  void activate_lr_planes(bool flag);  // default true
  // enable bottom view frustum plane
  void activate_bottom_plane(bool flag);  // default true
  // enable top view frustum plane
  void activate_top_plane(bool flag);  // default true
  // hither distance (along frame.v(0) from frame.p()).  set < 0.f to disable
  void set_hither(float hither);  // default -1.f
  // yonder distance (along frame.v(0) from frame.p()).  set < 0.f to disable
  void set_yonder(float yonder);  // default -1.f
  // screen-space tolerance is screen_thresh * min_window_diameter * 0.5f
  // in other words, screen-space tolerance in pixels is
  //  screen_thresh*min(nxpixels, nypixels) * 0.5f
  void set_screen_thresh(float screen_thresh);  // default 0.f
 private:
  Frame _frame{Vector(BIGFLOAT, 0.f, 0.f), Vector(0.f, 0.f, 0.f), Vector(0.f, 0.f, 0.f), Point(0.f, 0.f, 0.f)};
  Vec2<float> _zoomyx{twice(0.f)};
  bool _activate_lr_planes{true};
  bool _activate_bottom_plane{true};
  bool _activate_top_plane{true};
  float _hither{-1.f};
  float _yonder{-1.f};
  float _screen_thresh{0.f};
  friend class SrMesh;
  bool ok() const;
};

// Selectively refinable progressive mesh.
class SrMesh {
 public:
  SrMesh();
  ~SrMesh();
  // SrMesh must be empty prior to read_*().  SrMesh is left coarsened.
  void read_pm(PMeshRStream& pmrs);
  void read_srm(std::istream& is);
  void write_srm(std::ostream& os) const;  // must be fully_coarsened.
  void fully_refine();
  void fully_coarsen();
  void set_refine_morph_time(int refine_morph_time);    // 0 = disable
  void set_coarsen_morph_time(int coarsen_morph_time);  // 0 = disable
  void set_view_params(const SrViewParams& vp);
  void adapt_refinement(int nvtraverse = std::numeric_limits<int>::max());
  bool is_still_morphing() const;
  bool is_still_adapting() const;
  int num_vertices_refine_morphing() const;
  int num_vertices_coarsen_morphing() const;
  void construct_geomorph(SrGeomorphInfo& geoinfo);
  GMesh extract_gmesh() const;
  GMesh extract_gmesh(const SrGeomorphInfo& geoinfo) const;
  int num_active_vertices() const { return _num_active_vertices; }
  int num_active_faces() const { return _num_active_faces; }
  void ok() const;
  const Bbox<float, 3>& get_bbox() const { return _bbox; }

  // Interface: Rendering using OpenGL:
  void ogl_render_faces_individually(bool unlit_texture);
  void ogl_render_faces_strips(bool unlit_texture);
  int ogl_render_striplines();  // return number of strips
  void ogl_render_edges();
  void ogl_show_radii();
  void ogl_show_residuals(bool uniform_too);
  void ogl_render_faces_tvc(bool unlit_texture);
  int ogl_render_tvclines();  // return number of cache misses

 private:
  Bbox<float, 3> _bbox;
  Materials _materials;
  Array<SrVertex> _vertices;
  Array<SrFace> _faces;
  Array<SrVsplit> _vsplits;
  Array<SrAVertex> _base_vertices;
  Array<SrAFace> _base_faces;
  EList _active_vertices;
  EList _active_faces;
  int _num_active_vertices{-1};
  int _num_active_faces{-1};
  SrVertex* _quick_first_vt;
  SrFace* _quick_first_fl;
  SrVertexGeometry* _quick_first_vg;
  int _refine_morph_time{0};
  int _coarsen_morph_time{0};
  int _num_vertices_refine_morphing{0};
  int _num_vertices_coarsen_morphing{0};
  bool _was_modified{false};
  int _cache_time{1};

  // Temporary structs:
  Array<SrVertex*> _ar_tobevisible;

  // Static structs:
  // Properties: aface == &_isolated_aface
  static SrFace _isolated_face;  // fn[*] when no expected neighbor
  // Properties: matid == illegal, fnei[*] == &_isolated_aface
  static SrAFace _isolated_aface;  // fr->activef when !creates_2faces

  // Refinement parameters:
  SrViewParams _view_params;
  SrRefineParams _refp;

  // Rendering: common code and data:
  static constexpr unsigned k_Face_visited_mask = 1 << 30;  // high bit of matid
  static constexpr int k_illegal_matid = -1;
  unsigned _cur_frame_mask{0};  // 0 or k_Face_visited_mask

  // Rendering: accessor functions:
  // main ones
  const Point& get_point(const SrAVertex* va) const { return va->vgeom.point; }
  const Vector& get_normal(const SrAVertex* va) const { return va->vgeom.vnormal; }
  // auxiliary ones
  bool splitable(const SrAVertex* va) const { return is_splitable(va->vertex); }
  float get_uni_error_mag2(const SrAVertex* va) const { return _vsplits[va->vertex->vspli].uni_error_mag2; }
  float get_dir_error_mag2(const SrAVertex* va) const { return _vsplits[va->vertex->vspli].dir_error_mag2; }
  float get_radiusneg(const SrAVertex* va) const { return _vsplits[va->vertex->vspli].radius_neg; }
  float get_sin2alpha(const SrAVertex* va) const { return _vsplits[va->vertex->vspli].sin2alpha; }

  // Rendering using OpenGL:
  Array<Pixel> _ogl_mat_byte_rgba;  // size is _materials.num()
  void ogl_process_materials();
  void draw_vertex(const SrAVertex* v, bool use_texture) const;
  template <bool use_texture> void ogl_render_faces_strips_aux();

  int get_vf_j0(const SrAVertex* v, const SrAFace* f) const;
  int get_vf_j1(const SrAVertex* v, const SrAFace* f) const;
  int get_vf_j2(const SrAVertex* v, const SrAFace* f) const;
  SrAFace*& get_fnei(SrAFace* f, SrAFace* fn) const;
  SrAFace* rotate_clw(SrAFace* f, SrAVertex* v) const;
  SrAFace* rotate_ccw(SrAFace* f, SrAVertex* v) const;
  const SrVertex* get_vt(int vspli) const;
  SrVertex* get_vt(int vspli);
  const SrFace* get_fl(int vspli) const;
  SrFace* get_fl(int vspli);
  int get_vspli(const SrFace* fl) const;  // slow, requires integer divide
  bool is_splitable(const SrVertex* v) const { return v->vspli >= 0; }
  bool has_been_created(const SrVertex* v) const;
  bool has_been_split(const SrVertex* v) const;
  bool is_active_f(const SrFace* f) const;
  bool is_active_v(const SrVertex* v) const;
  bool creates_2faces(const SrVsplit* vspl) const;
  const SrVertexGeometry* refined_vg(const SrAVertex* va) const;
  bool vspl_legal(const SrVertex* vs) const;
  bool ecol_legal(const SrVertex* vt) const;  // vt left child of its parent!
  void compute_bspheres(CArrayView<SrVertexGeometry> vgeoms);
  void compute_nspheres(CArrayView<SrVertexGeometry> vgeoms);
  bool is_visible(const SrVertexGeometry* vg, const SrVsplit* vspl) const;
  bool big_error(const SrVertexGeometry* vg, const SrVsplit* vspl) const;
  bool qrefine(const SrVertex* vs) const;
  bool qcoarsen(const SrVertex* vt) const;
  void apply_vspl(SrVertex* vs, EListNode*& pn);
  void apply_ecol(SrVertex* vs, EListNode*& pn);
  void set_initial_view_params();
  void force_vsplit(SrVertex* vsf, EListNode*& n);
  void finish_vmorph(SrAVertex* va);
  void start_coarsen_morphing(SrVertex* vt);
  void abort_coarsen_morphing(SrVertex* vc);
  void perhaps_abort_coarsen_morphing(SrVertex* vc);
  void verify_optimality() const;
  void display_hierarchy_height() const;
  void update_vmorphs();
  bool verify_all_faces_visited() const;
  bool verify_all_vertices_uncached() const;
#if defined(ANALYZE_PM_COMPRESSION_WITH_XIA_VARSHNEY)
  int get_iflclw(SrVertex* vs) const;
  void refine_in_best_dflclw_order();
#endif
};

//----------------------------------------------------------------------------

inline int SrMesh::get_vf_j0(const SrAVertex* v, const SrAFace* f) const {
  ASSERTX(f->vertices[0] == v || f->vertices[1] == v || f->vertices[2] == v);
  return (f->vertices[1] == v) + (f->vertices[2] == v) * 2;
}

inline int SrMesh::get_vf_j1(const SrAVertex* v, const SrAFace* f) const {
  ASSERTX(f->vertices[0] == v || f->vertices[1] == v || f->vertices[2] == v);
  return (f->vertices[0] == v) + (f->vertices[1] == v) * 2;
}

inline int SrMesh::get_vf_j2(const SrAVertex* v, const SrAFace* f) const {
  ASSERTX(f->vertices[0] == v || f->vertices[1] == v || f->vertices[2] == v);
  return (f->vertices[2] == v) + (f->vertices[0] == v) * 2;
}

inline SrAFace*& SrMesh::get_fnei(SrAFace* f, SrAFace* fn) const {
  ASSERTX(f->fnei[0] == fn || f->fnei[1] == fn || f->fnei[2] == fn);
  return f->fnei[(f->fnei[1] == fn) + (f->fnei[2] == fn) * 2];
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SRMESH_H_
