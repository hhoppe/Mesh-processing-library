// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PMESH_H_
#define MESH_PROCESSING_LIBHH_PMESH_H_

#include "Geometry.h"           // Point, Vector
#include "A3dStream.h"          // A3dColor
#include "Array.h"
#include "Bbox.h"
#include "Materials.h"
#include "Pixel.h"

namespace hh {

class GMesh; class Ancestry; class PMeshIter; struct PMeshInfo;

// Vertex attributes.
struct PMVertexAttrib {
    Point point;
};

// Vertex attribute deltas.
struct PMVertexAttribD {
    Vector dpoint;
};

void interp(PMVertexAttrib& a, const PMVertexAttrib& a1, const PMVertexAttrib& a2, float f1);
void add(PMVertexAttrib& a, const PMVertexAttrib& a1, const PMVertexAttribD& ad);
void sub(PMVertexAttrib& a, const PMVertexAttrib& a1, const PMVertexAttribD& ad);
void diff(PMVertexAttribD& ad, const PMVertexAttrib& a1, const PMVertexAttrib& a2);
int compare(const PMVertexAttrib& a1, const PMVertexAttrib& a2);
int compare(const PMVertexAttrib& a1, const PMVertexAttrib& a2, float tol);

// Wedge attributes.
struct PMWedgeAttrib {
    Vector normal;
    A3dColor rgb;
    UV uv;
};

// Wedge attribute deltas.
struct PMWedgeAttribD {
    Vector dnormal;
    A3dColor drgb;
    UV duv;
};

void interp(PMWedgeAttrib& a, const PMWedgeAttrib& a1, const PMWedgeAttrib& a2, float f1);
void add(PMWedgeAttrib& a, const PMWedgeAttrib& a1, const PMWedgeAttribD& ad);
void sub_noreflect(PMWedgeAttrib& a, const PMWedgeAttrib& abase, const PMWedgeAttribD& ad);
void sub_reflect(PMWedgeAttrib& a, const PMWedgeAttrib& abase, const PMWedgeAttribD& ad);
void add_zero(PMWedgeAttrib& a, const PMWedgeAttribD& ad);
void diff(PMWedgeAttribD& ad, const PMWedgeAttrib& a1, const PMWedgeAttrib& a2);
int compare(const PMWedgeAttrib& a1, const PMWedgeAttrib& a2);
int compare(const PMWedgeAttrib& a1, const PMWedgeAttrib& a2, float tol);

// Face attribute is discrete: an integer material identifier that indexes into the _materials array.
struct PMFaceAttrib {
    int matid;
    // Note: a high bit is used for k_Face_visited_mask in SRMesh::ogl_render_faces_strips().
};

struct PMVertex {
    PMVertexAttrib attrib;
};

struct PMWedge {
    int vertex;
    PMWedgeAttrib attrib;
};

struct PMFace {
    Vec3<int> wedges;
    PMFaceAttrib attrib;
};

// optimize: use pointers in PMFace::wedges and W_Edge::vertex
// However, this requires arrays that don't reallocate.
// So either reserve, or use versatile Win32 memory allocation.

// Wedge mesh: faces -> wedges -> vertices
struct WMesh {
    void read(std::istream& is, const PMeshInfo& pminfo); // must be empty
    void write(std::ostream& os, const PMeshInfo& pminfo) const;
    void extract_gmesh(GMesh& gmesh, const PMeshInfo& pminfo) const;
    void ok() const;
    Materials _materials;
    Array<PMVertex> _vertices;
    Array<PMWedge> _wedges;
    Array<PMFace> _faces;
};

// Vertex split record.
// Records the information necessary to split a vertex of the mesh, to add to the mesh 1 new vertex and 1/2 new faces.
struct Vsplit {
    void read(std::istream& is, const PMeshInfo& pminfo);
    void write(std::ostream& os, const PMeshInfo& pminfo) const;
    void ok() const;
    bool adds_two_faces() const;
    // This format provides these limits:
    // - maximum number of faces: 1ull<<32
    // - maximum vertex valence:  1u<<16
    // - maximum number of materials: 1u<<16
// Encoding of vertices vs, vl, vr.
    // Face flclw is the face just CLW of vl from vs.
    //  vs is the vs_index\'th vertex of flclw
    //  (vl is the (vs_index+2)%3\'th vertex of face flclw)
    //  vr is the (vlr_offset1-1)\'th vertex when rotating CLW about vs from vl
    // Special cases:
    // - vlr_offset1==1 : no_vr and no_fr
    // - vlr_offest1==0: no flclw! vspl.flclw is actually flccw.
    int flclw;                  // 0..(mesh.num_faces()-1)
    short vlr_offset1;          // 0..(max_vertex_valence) (prob<valence/2)
    ushort code;                // (vs_index(2), ii(2), ws(3), wt(3), wl(2), wr(2), fl_matid>=0(1), fr_matid>=0(1))
    enum EMaskBase : unsigned {
        B_STMASK = 0x0007,
        B_LSAME = 0x0001,
        B_RSAME = 0x0002,
        B_CSAME = 0x0004,
        //
        B_LRMASK = 0x0003,
        B_ABOVE = 0x0000,
        B_BELOW = 0x0001,
        B_NEW  = 0x0002,                // must be on separate bit.
    };
    enum EMask : unsigned {
        VSINDEX_SHIFT = 0,
        VSINDEX_MASK = (0x0003<<VSINDEX_SHIFT),
        //
        II_SHIFT = 2,
        II_MASK = (0x0003<<II_SHIFT),
        //
        S_SHIFT = 4,
        S_MASK =  (B_STMASK<<S_SHIFT),
        S_LSAME = (B_LSAME <<S_SHIFT),
        S_RSAME = (B_RSAME <<S_SHIFT),
        S_CSAME = (B_CSAME <<S_SHIFT),
        //
        T_SHIFT = 7,
        T_MASK =  (B_STMASK<<T_SHIFT),
        T_LSAME = (B_LSAME <<T_SHIFT),
        T_RSAME = (B_RSAME <<T_SHIFT),
        T_CSAME = (B_CSAME <<T_SHIFT),
        //
        L_SHIFT = 10,
        L_MASK =  (B_LRMASK<<L_SHIFT),
        L_ABOVE = (B_ABOVE <<L_SHIFT),
        L_BELOW = (B_BELOW <<L_SHIFT),
        L_NEW  =  (B_NEW   <<L_SHIFT),
        //
        R_SHIFT = 12,
        R_MASK =  (B_LRMASK<<R_SHIFT),
        R_ABOVE = (B_ABOVE <<R_SHIFT),
        R_BELOW = (B_BELOW <<R_SHIFT),
        R_NEW  =  (B_NEW   <<R_SHIFT),
        //
        FLN_SHIFT = 14,
        FLN_MASK = (1u<<FLN_SHIFT),
        //
        FRN_SHIFT = 15,
        FRN_MASK = (1u<<FRN_SHIFT),
    };
    // *** Documentation:
    // vs_index: 0..2: index of vs within flace flclw
    // ii: 0..2: ==alpha(1.0, 0.5, 0.0)
    //   ii=2: a=0.0 (old_vs=~new_vs)
    //   ii=1: a=0.5
    //   ii=0: a=1.0 (old_vs=~new_vt)
    // Inside wedges
    //  {S,T}{LSAME}: if exists outside left wedge and if same
    //  {S,T}{RSAME}: if exists outside right wedge and if same
    //  {S,T}{CSAME}: if inside left and right wedges are same
    //  (when no_vr, {S,T}RSAME==1, {S,T}CSAME==0)
    // Outside wedges
    //  (when no_vr, RABOVE==1)
    // New face material identifiers
    //  {L,R}NF: if 1, face matids not predicted correctly using ii,
    //     so included in f{l,r}_matid
    //  (when no_vr, RNF==0 obviously)
    //
    // *** Probabilities:
    //  vs_index: 0..2 (prob. uniform)
    //  ii: ii==2 prob. low/med   (med if 'MeshSimplify -nominii1')
    //      ii==0 prob. low/med
    //      ii==1 prob. high/zero (zero if 'MeshSimplify -monminii1')
    //  {S,T}LSAME: prob. high
    //  {S,T}RSAME: prob. high
    //  {S,T}CSAME: prob. low
    //  {L,R}ABOVE: prob. high
    //  {L,R}BELOW: prob. low
    //  {L,R}NEW:   prob. low
    // Note: wl, wr, ws, wt are correlated since scalar half-edge
    //  discontinuities usually match up at both ends of edges.
    // -> do entropy coding on (ii, wl, wr, ws, wt) symbol as a whole.
// Face attribute values (usually predicted correctly)
    // these are defined only if {L,R}NF respectively
    //  otherwise for now they are set to 0
    ushort fl_matid;
    ushort fr_matid;
// Vertex attribute deltas
    // for ii==2: vad_large=new_vt-old_vs, vad_small=new_vs-old_vs
    // for ii==0: vad_large=new_vs-old_vs, vad_small=new_vt-old_vs
    // for ii==1: vad_large=new_vt-new_i,  vad_small=new_i-old_vs
    //    where new_i=interp(new_vt, new_vs)
    PMVertexAttribD vad_large;
    PMVertexAttribD vad_small;  // is zero if "MeshSimplify -nofitgeom"
// Wedge attribute deltas (size 1--6)
    Array<PMWedgeAttribD> ar_wad;
    // Order: [(wvtfl, wvsfl), [(wvtfr, wvsfr)], wvlfl, [wvrfr]]
// Residual information
    float resid_uni;
    float resid_dir;
    int expected_wad_num(const PMeshInfo& pminfo) const;
};

// For each face, what are its 3 neighbors?
struct PMFaceNeighbors {
    Vec3<int> faces;            // faces[i] is across edge opposite of wedges[i].  <0 if no neighbor.
};

// Wedge mesh augmented with adjacency information.
// Specifically, dual graph encoding face-face adjacency using PMFaceNeighbors.
class AWMesh : public WMesh {
 public:
    void read(std::istream& is, const PMeshInfo& pminfo); // must be empty
    void write(std::ostream& os, const PMeshInfo& pminfo) const;
    void ok() const;
    void rotate_ccw(int v, int& w, int& f) const;
    void rotate_clw(int v, int& w, int& f) const;
// Rendering: common code and data
    static constexpr int k_Face_visited_mask = 1<<30; // high bit of matid
    int _cur_frame_mask {0};                          // 0 or k_Face_visited_mask
// Rendering using OpenGL
    void ogl_render_faces_individually(const PMeshInfo& pminfo, int usetexture);
    void ogl_render_faces_strips(const PMeshInfo& pminfo, int usetexture);
    void ogl_render_edges();
// Rendering using Direct3D
    // to be defined
 public:
    Array<PMFaceNeighbors> _fnei; // must be same size as _faces!
 public:
    int get_jvf(int v, int f) const; // get index of vertex v in face f
    int get_wvf(int v, int f) const;
    int most_clw_face(int v, int f); // negative if v is interior vertex
    int most_ccw_face(int v, int f); // negative if v is interior vertex
    bool is_boundary(int v, int f);
    bool gather_faces(int v, int f, Array<int>& faces); // ret: is_boundary
 private:
    void construct_adjacency();
    void apply_vsplit_ancestry(Ancestry* ancestry, int vs, bool isr, int onumwedges, int code,
                               int wvlfl, int wvrfr, int wvsfl, int wvsfr, int wvtfl, int wvtfr);
// Rendering using OpenGL
    Array<Pixel> _ogl_mat_byte_rgba; // size is _materials.num()
    void ogl_process_materials();
// Rendering using Direct3D
    // to be defined
 protected:
    void apply_vsplit(const Vsplit& vspl, const PMeshInfo& pminfo, Ancestry* ancestry = nullptr);
    void undo_vsplit(const Vsplit& vspl, const PMeshInfo& pminfo);
    // Default operator=() and copy_constructor are safe.
 public:                        // hidden
    void apply_vsplit_private(const Vsplit& vspl, const PMeshInfo& pminfo, Ancestry* ancestry = nullptr);
};

struct PMeshInfo {
    int _read_version;
    bool _has_rgb;
    bool _has_uv;
    bool _has_resid;
    bool _has_wad2;
    int _tot_nvsplits;
    int _full_nvertices;
    int _full_nwedges;
    int _full_nfaces;
    Bbox _full_bbox;
};

// Progressive mesh:
//  contains a base mesh and a sequence of vertex split records.
class PMesh : noncopyable {
 public:
    PMesh();
    // non-progressive read
    void read(std::istream& is); // die unless empty
    void write(std::ostream& os) const;
    void truncate_beyond(PMeshIter& pmi); // remove all vsplits beyond iterator
    void truncate_prior(PMeshIter& pmi);  // advance base mesh
 public:
    friend class PMeshRStream;
    AWMesh _base_mesh;
    Array<Vsplit> _vsplits;
    PMeshInfo _info;
 private:
    static PMeshInfo read_header(std::istream& is);
    static bool at_trailer(std::istream& is);
    // const AWMesh& base_mesh const { return _base_mesh; }
};

// Progressive mesh stream
// Can be either:
//  - read from an existing PMesh, or
//  - read from an input streamm, or
//  - read from an input stream and archived to a PMesh
class PMeshRStream : noncopyable {
 public:
    explicit PMeshRStream(const PMesh& pm);
    explicit PMeshRStream(std::istream& is, PMesh* ppm_construct = nullptr);
    ~PMeshRStream();
    void read_base_mesh(AWMesh* bmesh = nullptr); // always call this first!
    const AWMesh& base_mesh();
    bool is_reversible() const { return !!_pm; }
    const Vsplit* next_vsplit();
    const Vsplit* prev_vsplit();      // die if !is_reversible()
    const Vsplit* peek_next_vsplit(); // peek without using it
    PMeshInfo _info;
 private:
    friend PMeshIter;
    friend PMesh;               // for PMesh::truncate_*()
    std::istream* _is;          // may be nullptr
    PMesh* _pm;                 // may be nullptr
    int _vspliti {-1};          // def if _pm, next to read from _pm->_vsplits; -1 before base_mesh is read
    Vsplit _tmp_vspl;           // def if !_pm
    bool _vspl_ready {false};   // def if !_pm, true if _vspl is only peeked
    AWMesh _lbase_mesh;         // used to store basemesh if !_pm
};

// Progressive mesh iterator (is a AWMesh!)
class PMeshIter : public AWMesh {
 public:
    explicit PMeshIter(PMeshRStream& pmrs);
    bool next()                                 { return next_ancestry(nullptr); } // ret: success
    bool prev();                // ret: success; die if !_pmrs.is_reversible()
    bool goto_nvertices(int nv)                 { return goto_nvertices_ancestry(nv, nullptr); } // ret: success
    bool goto_nfaces(int nf)                    { return goto_nfaces_ancestry(nf, nullptr); } // within +-1, favor 0/-1
    PMeshRStream& rstream()                     { return _pmrs; }
    const PMeshRStream& rstream() const         { return _pmrs; }
 private:
    friend class Geomorph;
    friend PMesh;               // for PMesh::truncate_*()
    PMeshRStream& _pmrs;
    bool next_ancestry(Ancestry* ancestry);
    bool goto_nvertices_ancestry(int nvertices, Ancestry* ancestry);
    bool goto_nfaces_ancestry(int nfaces, Ancestry* ancestry);
    // Default operator=() is disabled due to reference; default copy_constructor is safe.
};

// Records vertex and wedge ancestry during PM traversal in order to construct geomorphs.
class Ancestry {
 public:
    Array<PMVertexAttrib> _vancestry;
    Array<PMWedgeAttrib> _wancestry;
};

// Geomorph endstates: pair of attributes for a changing vertex.
struct PMVertexAttribG {
    int vertex;
    Vec2<PMVertexAttrib> attribs;
};

// Geomorph endstates: pair of attributes for a changing wedge.
struct PMWedgeAttribG {
    int wedge;
    Vec2<PMWedgeAttrib> attribs;
};

// A geomorph is a mesh which is able to smoothly transition between two endstates.
// It is a mesh together with a list of vertices and wedges that change and their endstates.
// It can be evaluated over a continuous blend parameter 0<=alpha<=1.
// It is created by applying a sequence of vsplits to a PM iterator.
class Geomorph : public WMesh {
 public:
// Construction
    // Create a geomorph from pmi's current mesh to the mesh obtained after applying n vsplits to pmi.
    // Note side-effect on pmi!
    // Ret: was_able_to_go_all_the_way; die if !empty
    bool construct_next(PMeshIter& pmi, int nvsplits);
    // Same up to nvertices
    // Ret: success
    bool construct_goto_nvertices(PMeshIter& pmi, int nvertices);
    // Same up to nfaces (or nfaces-1)
    // Ret: success
    bool construct_goto_nfaces(PMeshIter& pmi, int nfaces);
// Evaluation
    // Modify each vertex and wedge attributes of this mesh by linearly
    //  interpolating between attribs[0] and attribs[1].
    // Could optimize this by:
    //  (1) creating a list of only the changing attributes  (done)
    //  (2) precomputing (attribs[1]-attribs[0])
    //  (3) renormalizing only normals that vary significantly
    void evaluate(float alpha); // 0<=alpha<=1 (0==coarse)
 private:
    friend class SGeomorph;
    Array<PMVertexAttribG> _vgattribs;
    Array<PMWedgeAttribG> _wgattribs;
    enum class EWant { vsplits, nvertices, nfaces };
    bool construct(PMeshIter& pmi, EWant want, int num);
    // Default operator=() and copy_constructor are safe.
};

// A simple mesh is one in which wedges are converted to vertices:  faces -> vertices
// This means that geometry information (vertex positions) may be duplicated.
// The PM representation should not operate on such meshes directly because vsplits would tear the surface
//   (unless complicated constraints were introduced to keep duplicated vertices together which is essentially what
//    WMesh does.)

// Simple vertex attributes.
struct PMSVertexAttrib {
    PMVertexAttrib v;
    PMWedgeAttrib w;
};

void interp(PMSVertexAttrib& a, const PMSVertexAttrib& a1, const PMSVertexAttrib& a2, float f1);

struct PMSVertex {
    PMSVertexAttrib attrib;
};

struct PMSFace {
    Vec3<int> vertices;
    PMFaceAttrib attrib;
};

struct PMSVertexAttribG {
    int vertex;
    Vec2<PMSVertexAttrib> attribs;
};
#if defined(__GNUC__)           // else makes VS2015 Intellisense unhappy
static_assert(std::is_pod<PMSVertexAttribG>::value, "");
#endif

// Simple mesh: faces -> vertices.
// Split wedges into independent vertices.
class SMesh {
 public:
    explicit SMesh(const WMesh& wmesh);
    void extract_gmesh(GMesh& gmesh, int has_rgb, int has_uv) const;
 public:
    Materials _materials;
    Array<PMSVertex> _vertices;
    Array<PMSFace> _faces;
    // Default operator=() and copy_constructor are safe.
};

// Simple geomorph.
// Efficiency trade-off:
// - a simple mesh is quicker to render since there is one less level of indirection (no wedges).
// - but, it may be more expensive to geomorph since duplicated vertex positions must be interpolated separately.
class SGeomorph : public SMesh {
 public:
    explicit SGeomorph(const Geomorph& geomorph);
    void evaluate(float alpha); // 0<=alpha<=1 (0==coarse)
    void extract_gmesh(GMesh& gmesh, int has_rgb, int has_uv) const;
 private:
    Array<PMSVertexAttribG> _vgattribs;
    // Default operator=() and copy_constructor are safe.
};


//----------------------------------------------------------------------------

inline bool Vsplit::adds_two_faces() const {
    return vlr_offset1>1;
}

#if defined(HH_DEBUG)
inline int AWMesh::get_jvf(int v, int f) const {
    ASSERTX(_vertices.ok(v));
    ASSERTX(_faces.ok(f));
    for_int(j, 3) {
        if (_wedges[_faces[f].wedges[j]].vertex==v) return j;
    }
    assertnever("");
}
inline int AWMesh::get_wvf(int v, int f) const {
    return _faces[f].wedges[get_jvf(v, f)];
}
#else
inline int AWMesh::get_jvf(int v, int f) const {
//     return (_wedges[_faces[f].wedges[0]].vertex==v ? 0 :
//             _wedges[_faces[f].wedges[1]].vertex==v ? 1 :
//             2);
    return ((_wedges[_faces[f].wedges[1]].vertex==v)+
            (_wedges[_faces[f].wedges[2]].vertex==v)*2);
}
inline int AWMesh::get_wvf(int v, int f) const {
    int w0 = _faces[f].wedges[0]; if (_wedges[w0].vertex==v) return w0;
    int w1 = _faces[f].wedges[1]; if (_wedges[w1].vertex==v) return w1;
    int w2 = _faces[f].wedges[2]; return w2;
}
#endif  // defined(HH_DEBUG)

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_PMESH_H_
