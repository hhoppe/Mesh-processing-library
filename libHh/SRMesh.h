// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Bbox.h"
#include "EList.h"
#include "LinearFunc.h"
#include "Pool.h"
#include "Map.h"
#include "Materials.h"
#include "Pixel.h"

#if 0
// Selectively refined mesh usage:
{
    SRMesh srmesh; srmesh.read_srm(std::cin); // or read_pm()
    srmesh.set_refine_morph_time(32);
    srmesh.set_coarsen_morph_time(16);
    for (;;) {
        SRViewParams vp; srmesh.set_view_params(vp);
        if (0) {
            SRGeomorphInfo geoinfo; srmesh.construct_geomorph(geoinfo);
            GMesh gmesh; extract_gmesh(gmesh, geoinfo);
        }
        srmesh.adapt_refinement();
        srmesh.ogl_render_faces_tvc(false); // ogl_render_*();
        if (0) { GMesh gmesh; srmesh.extract_gmesh(gmesh); }
        if (!(srmesh.is_still_morphing() || srmesh.is_still_adapting()))
            pause_for_input();
    }
}
#endif

namespace hh {

class PMeshRStream; class GMesh; struct SRAVertex; struct SRAFace;

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

struct SRVertexGeometry {
    Point point;
#if !defined(SR_NOR001)
    Vector vnormal;
#else
    static const Vector vnormal; // always (0.f, 0.f, 1.f)
#endif
};

struct SRVertexMorph {
    bool coarsening;            // ==!refining
    short time;                 // time-1 left to achieve goal (always >=0)
    SRVertexGeometry vgrefined; // right child refined geometry
    SRVertexGeometry vginc;     // increment to add during morph
    HH_POOL_ALLOCATION(SRVertexMorph);
};
HH_INITIALIZE_POOL(SRVertexMorph);

struct SRVertex {
    SRAVertex* avertex;         // nullptr if not active
    SRVertex* parent;           // nullptr if in M^0
    int vspli;                  // -1 if in M^n
};

struct SRAVertex {              // active vertex
    EListNode activev;          // at offset 0 for fast pointer conversion
    SRVertex* vertex;
    SRVertexGeometry vgeom;
    unique_ptr<SRVertexMorph> vmorph; // nullptr if no morph
    bool visible;                     // was vertex visible when last traversed?
    int cached_time;                  // for transparent vertex caching
    HH_POOL_ALLOCATION(SRAVertex);
};
HH_INITIALIZE_POOL(SRAVertex);

struct SRFace {
    SRAFace* aface;             // nullptr if not active, ==&_isolated_aface if !fr
};

struct SRAFace {                // active face
    EListNode activef;          // at offset 0 for fast pointer conversion
    int matid;                  // see notes below.  up here for cache line
    Vec3<SRAVertex*> vertices;
    Vec3<SRAFace*> fnei;        // &_isolated_aface if no neighbor
    // Notes on matid:
    //  - a high bit is used for k_Face_visited_mask in SRMesh::*_render_*
    //  - it is used as temporary 'int' storage in SRMesh::read_pm()
};

struct SRAFacePair {
    // Note that these two faces generally become unconnected thru refinement.
    Vec2<SRAFace> pair;
    HH_POOL_ALLOCATION(SRAFacePair);
};
HH_INITIALIZE_POOL(SRAFacePair);

struct SRVsplit {
    SRVertexGeometry vu_vgeom;
#if defined(SR_NO_VSGEOM)
    SRVertexGeometry vs_vgeom, vt_vgeom;
#endif
    // fn[0..3]=={flccw, flclw, frclw, frccw}
    Vec4<SRFace*> fn;           // ==&_inactive_face if no face expected
#if !defined(SR_PREDICT_MATID)
    short fl_matid;
    short fr_matid;
#endif
// Refinement criteria information
#if !defined(SR_NOR001)
    float uni_error_mag2;       // magnitude2 of uniform residual error
#else
    static const float uni_error_mag2; // always 0.f; constexpr in C++11
#endif
    float dir_error_mag2;       // magnitude2 of directional residual error
    float radius_neg;           // max radius of influence (negated)
#if !defined(SR_NOR001)
    float sin2alpha;            // backface region, square(sin(alpha))
#else
    static const float sin2alpha; // always 0.f; constexpr in C++11
#endif
};

struct SRRefineParams {
    // Frame _framei;           // to compute screen-space projections.
    int _nplanes;                 // number of view frustum planes
    Vec<LinearFunc,6> _planes;    // view frustum planes
    Point _eye;                   // eyepoint in world coordinates (frame.p())
    float _tz2;                   // screen-space tolerance zoomed and squared
    LinearFunc _eyedir;           // linear func along view direction
};

class SRGeomorphInfo : noncopyable {
 private:
    friend class SRMesh;
    Vec2<Map<SRVertex*,SRVertexGeometry>> _ancestors;
};

class SRViewParams {
 public:
    // frame.p() = eyepoint; frame.v(0) = view_direction; frame.v(1) = left_dir;
    //  frame.v(2) = top_dir   (frame must be orthonormal!)
    void set_frame(const Frame& frame); // no default
    // zoomyx[1] is tan(angle) where angle is between frame.v(0) and left edge of viewport
    // zoomyx[0] is tan(angle) where angle is between frame.v(0) and top edge of viewport
    void set_zooms(const Vec2<float>& zoomyx); // no default
    // enable left/right view frustum planes
    void activate_lr_planes(bool flag);    // default true
    // enable bottom view frustum plane
    void activate_bottom_plane(bool flag); // default true
    // enable top view frustum plane
    void activate_top_plane(bool flag); // default true
    // hither distance (along frame.v(0) from frame.p()).  set <0.f to disable
    void set_hither(float hither); // default -1.f
    // yonder distance (along frame.v(0) from frame.p()).  set <0.f to disable
    void set_yonder(float yonder); // default -1.f
    // screen-space tolerance is screen_thresh*min_window_diameter*0.5f
    // in other words, screen-space tolerance in pixels is
    //  screen_thresh*min(nxpixels, nypixels)*0.5f
    void set_screen_thresh(float screen_thresh); // default 0.f
 private:
    Frame _frame {Vector(BIGFLOAT,0.f,0.f), Vector(0.f,0.f,0.f), Vector(0.f,0.f,0.f), Point(0.f,0.f,0.f)}; // bad num
    Vec2<float> _zoomyx {twice(0.f)};
    bool _activate_lr_planes {true};
    bool _activate_bottom_plane {true};
    bool _activate_top_plane {true};
    float _hither {-1.f};
    float _yonder {-1.f};
    float _screen_thresh {0.f};
    friend class SRMesh;
    bool ok() const;
};

// Selectively refinable progressive mesh.
class SRMesh {
 public:
    SRMesh();
    ~SRMesh();
    // SRMesh must be empty prior to read_*().  SRMesh is left coarsened.
    void read_pm(PMeshRStream& pmrs);
    void read_srm(std::istream& is);
    void write_srm(std::ostream& os) const; // must be fully_coarsened.
    void fully_refine();
    void fully_coarsen();
    void set_refine_morph_time(int refine_morph_time);   // 0 = disable
    void set_coarsen_morph_time(int coarsen_morph_time); // 0 = disable
    void set_view_params(const SRViewParams& vp);
    void adapt_refinement(int nvtraverse = INT_MAX);
    bool is_still_morphing() const;
    bool is_still_adapting() const;
    int num_vertices_refine_morphing() const;
    int num_vertices_coarsen_morphing() const;
    void construct_geomorph(SRGeomorphInfo& geoinfo);
    void extract_gmesh(GMesh& gmesh) const;
    void extract_gmesh(GMesh& gmesh, const SRGeomorphInfo& geoinfo) const;
    int num_active_vertices() const             { return _num_active_vertices; }
    int num_active_faces() const                { return _num_active_faces; }
    void ok() const;
    const Bbox& get_bbox() const { return _bbox; }
// Interface: Rendering using OpenGL
    void ogl_render_faces_individually(bool unlit_texture);
    void ogl_render_faces_strips(bool unlit_texture);
    int ogl_render_striplines(); // return number of strips
    void ogl_render_edges();
    void ogl_show_radii();
    void ogl_show_residuals(bool uniform_too);
    void ogl_render_faces_tvc(bool unlit_texture);
    int ogl_render_tvclines();  // return number of cache misses
 private:
    Bbox _bbox;
    Materials _materials;
    Array<SRVertex> _vertices;
    Array<SRFace> _faces;
    Array<SRVsplit> _vsplits;
    Array<SRAVertex> _base_vertices;
    Array<SRAFace> _base_faces;
    EList _active_vertices;
    EList _active_faces;
    int _num_active_vertices {-1};
    int _num_active_faces {-1};
    SRVertex* _quick_first_vt;
    SRFace* _quick_first_fl;
    SRVertexGeometry* _quick_first_vg;
    int _refine_morph_time {0};
    int _coarsen_morph_time {0};
    int _num_vertices_refine_morphing {0};
    int _num_vertices_coarsen_morphing {0};
    bool _was_modified {false};
    int _cache_time {1};
// Temporary structs
    Array<SRVertex*> _ar_tobevisible;
// Static structs
    // Properties: aface==&_isolated_aface
    static SRFace _isolated_face;   // fn[*] when no expected neighbor
    // Properties: matid==illegal, fnei[*]==&_isolated_aface
    static SRAFace _isolated_aface; // fr->activef when !creates_2faces
// Refinement parameters
    SRViewParams _view_params;
    SRRefineParams _refp;
// Rendering: common code and data
    static constexpr int k_Face_visited_mask = 1<<30; // high bit of matid
    static constexpr int k_illegal_matid = -1;
    int _cur_frame_mask {0};    // 0 or k_Face_visited_mask
// Rendering: accessor functions
    // main ones
    const Point& get_point(const SRAVertex* va) const   { return va->vgeom.point; }
    const Vector& get_normal(const SRAVertex* va) const { return va->vgeom.vnormal; }
    // auxiliary ones
    bool splitable(const SRAVertex* va) const           { return is_splitable(va->vertex); }
    float get_uni_error_mag2(const SRAVertex* va) const { return _vsplits[va->vertex->vspli].uni_error_mag2; }
    float get_dir_error_mag2(const SRAVertex* va) const { return _vsplits[va->vertex->vspli].dir_error_mag2; }
    float get_radiusneg(const SRAVertex* va) const      { return _vsplits[va->vertex->vspli].radius_neg; }
    float get_sin2alpha(const SRAVertex* va) const      { return _vsplits[va->vertex->vspli].sin2alpha; }
// Rendering using OpenGL
    Array<Pixel> _ogl_mat_byte_rgba; // size is _materials.num()
    void ogl_process_materials();
    void draw_vertex(const SRAVertex* v, bool use_texture) const;
    template<bool use_texture> void ogl_render_faces_strips_aux();
 private:
    int get_vf_j0(const SRAVertex* v, const SRAFace* f) const;
    int get_vf_j1(const SRAVertex* v, const SRAFace* f) const;
    int get_vf_j2(const SRAVertex* v, const SRAFace* f) const;
    SRAFace*& get_fnei(SRAFace* f, SRAFace* fn) const;
    SRAFace* rotate_clw(SRAFace* f, SRAVertex* v) const;
    SRAFace* rotate_ccw(SRAFace* f, SRAVertex* v) const;
    const SRVertex* get_vt(int vspli) const;
    SRVertex* get_vt(int vspli);
    const SRFace* get_fl(int vspli) const;
    SRFace* get_fl(int vspli);
    int get_vspli(const SRFace* fl) const; // slow, requires integer divide
    bool is_splitable(const SRVertex* v) const  { return v->vspli>=0; }
    bool has_been_created(const SRVertex* v) const;
    bool has_been_split(const SRVertex* v) const;
    bool is_active_f(const SRFace* f) const;
    bool is_active_v(const SRVertex* v) const;
    bool creates_2faces(const SRVsplit* vspl) const;
    const SRVertexGeometry* refined_vg(const SRAVertex* va) const;
    bool vspl_legal(const SRVertex* vs) const;
    bool ecol_legal(const SRVertex* vt) const; // vt left child of its parent!
    void compute_bspheres(CArrayView<SRVertexGeometry> vgeoms);
    void compute_nspheres(CArrayView<SRVertexGeometry> vgeoms);
    bool is_visible(const SRVertexGeometry* vg, const SRVsplit* vspl) const;
    bool big_error(const SRVertexGeometry* vg, const SRVsplit* vspl) const;
    bool qrefine(const SRVertex* vs) const;
    bool qcoarsen(const SRVertex* vt) const;
    void apply_vspl(SRVertex* vs, EListNode*& pn);
    void apply_ecol(SRVertex* vs, EListNode*& pn);
    void set_initial_view_params();
    void force_vsplit(SRVertex* vsf, EListNode*& n);
    void finish_vmorph(SRAVertex* va);
    void start_coarsen_morphing(SRVertex* vt);
    void abort_coarsen_morphing(SRVertex* vc);
    void perhaps_abort_coarsen_morphing(SRVertex* vc);
    void verify_optimality() const;
    void display_hierarchy_height() const;
    void update_vmorphs();
    bool verify_all_faces_visited() const;
    bool verify_all_vertices_uncached() const;
#if defined(ANALYZE_PM_COMPRESSION_WITH_XIA_VARSHNEY)
    int get_iflclw(SRVertex* vs) const;
    void refine_in_best_dflclw_order();
#endif
};


//----------------------------------------------------------------------------

inline int SRMesh::get_vf_j0(const SRAVertex* v, const SRAFace* f) const {
    ASSERTX(f->vertices[0]==v || f->vertices[1]==v || f->vertices[2]==v);
    return (f->vertices[1]==v)+(f->vertices[2]==v)*2;
}

inline int SRMesh::get_vf_j1(const SRAVertex* v, const SRAFace* f) const {
    ASSERTX(f->vertices[0]==v || f->vertices[1]==v || f->vertices[2]==v);
    return (f->vertices[0]==v)+(f->vertices[1]==v)*2;
}

inline int SRMesh::get_vf_j2(const SRAVertex* v, const SRAFace* f) const {
    ASSERTX(f->vertices[0]==v || f->vertices[1]==v || f->vertices[2]==v);
    return (f->vertices[2]==v)+(f->vertices[0]==v)*2;
}

inline SRAFace*& SRMesh::get_fnei(SRAFace* f, SRAFace* fn) const {
    ASSERTX(f->fnei[0]==fn || f->fnei[1]==fn || f->fnei[2]==fn);
    return f->fnei[(f->fnei[1]==fn)+(f->fnei[2]==fn)*2];
}

} // namespace hh
