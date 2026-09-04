// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_PMESH_H_
#define MESH_PROCESSING_LIBHH_PMESH_H_

#include "libHh/A3dStream.h"  // A3dColor
#include "libHh/Array.h"
#include "libHh/Bbox.h"
#include "libHh/GMesh.h"
#include "libHh/Geometry.h"  // Point, Vector
#include "libHh/Materials.h"
#include "libHh/Pixel.h"

namespace hh {

class Ancestry;
class PMeshIter;
struct PMeshInfo;

// Vertex attributes.
struct PmVertexAttrib {
  Point point;
};

// Vertex attribute deltas.
struct PmVertexAttribD {
  Vector dpoint;
};

void interp(PmVertexAttrib& a, const PmVertexAttrib& a1, const PmVertexAttrib& a2, float frac1);
void add(PmVertexAttrib& a, const PmVertexAttrib& a1, const PmVertexAttribD& ad);
void sub(PmVertexAttrib& a, const PmVertexAttrib& a1, const PmVertexAttribD& ad);
void diff(PmVertexAttribD& ad, const PmVertexAttrib& a1, const PmVertexAttrib& a2);
std::partial_ordering compare(const PmVertexAttrib& a1, const PmVertexAttrib& a2);
std::weak_ordering compare(const PmVertexAttrib& a1, const PmVertexAttrib& a2, float tol);

// Wedge attributes.
struct PmWedgeAttrib {
  Vector normal;
  A3dColor rgb;
  Uv uv;
};

// Wedge attribute deltas.
struct PmWedgeAttribD {
  Vector dnormal;
  A3dColor drgb;
  Uv duv;
};

void interp(PmWedgeAttrib& a, const PmWedgeAttrib& a1, const PmWedgeAttrib& a2, float frac1);
void add(PmWedgeAttrib& a, const PmWedgeAttrib& a1, const PmWedgeAttribD& ad);
void sub_noreflect(PmWedgeAttrib& a, const PmWedgeAttrib& abase, const PmWedgeAttribD& ad);
void sub_reflect(PmWedgeAttrib& a, const PmWedgeAttrib& abase, const PmWedgeAttribD& ad);
void add_zero(PmWedgeAttrib& a, const PmWedgeAttribD& ad);
void diff(PmWedgeAttribD& ad, const PmWedgeAttrib& a1, const PmWedgeAttrib& a2);
std::partial_ordering compare(const PmWedgeAttrib& a1, const PmWedgeAttrib& a2);
std::weak_ordering compare(const PmWedgeAttrib& a1, const PmWedgeAttrib& a2, float tol);

// Face attribute is discrete: an integer material identifier that indexes into the _materials array.
struct PmFaceAttrib {
  int matid;
  // Note: a high bit is used for k_Face_visited_mask in SrMesh::ogl_render_faces_strips().
};

struct PmVertex {
  PmVertexAttrib attrib;
};

struct PmWedge {
  int vertex;
  PmWedgeAttrib attrib;
};

struct PmFace {
  Vec3<int> wedges;
  PmFaceAttrib attrib;
};

// optimize: use pointers in PmFace::wedges and W_Edge::vertex
// However, this requires arrays that don't reallocate.
// So either reserve, or use versatile Win32 memory allocation.

// Wedge mesh: faces -> wedges -> vertices
class WMesh {
 public:
  void read(std::istream& is, const PMeshInfo& pminfo);  // must be empty
  void write(std::ostream& os, const PMeshInfo& pminfo) const;
  void write_ply(std::ostream& os, const PMeshInfo& pminfo, bool binary) const;
  [[nodiscard]] GMesh extract_gmesh(const PMeshInfo& pminfo) const;
  void ok() const;
  [[nodiscard]] Vec3<int> face_vertices(int f) const;
  [[nodiscard]] Vec3<Point> face_points(int f) const;
  [[nodiscard]] int get_jvf(int v, int f) const;  // get index of vertex v in face f
  [[nodiscard]] int get_wvf(int v, int f) const;
  [[nodiscard]] Array<int> gather_someface() const;  // Returns mapping: vertex index -> index of some adjacent face.

  Materials _materials;
  Array<PmVertex> _vertices;
  Array<PmWedge> _wedges;
  Array<PmFace> _faces;
};

// Vertex split record.  Records the information necessary to split a vertex of the mesh, to add to the mesh 1 new
// vertex and 1 or 2 new faces.
struct Vsplit {
  void read(std::istream& is, const PMeshInfo& pminfo);
  void write(std::ostream& os, const PMeshInfo& pminfo) const;
  void ok() const;
  [[nodiscard]] bool adds_two_faces() const;
  // This format provides these limits:
  // - maximum number of faces: 1ull << 32
  // - maximum vertex valence:  1u << 16
  // - maximum number of materials: 1u << 16

  // ** Encoding of vertices vs, vl, vr:
  // Face flclw is the face just CLW of vl from vs.
  //  vs is the vs_index\'th vertex of flclw
  //  vl is the ((vs_index + 2) % 3)'th vertex of face flclw
  //  vr is the (vlr_offset1 - 1)'th vertex when rotating CLW about vs from vl
  // Special cases:
  // - vlr_offset1 == 1 : no_vr and no_fr
  // - vlr_offest1 == 0 : no flclw! vspl.flclw is actually flccw.
  int flclw;          // 0 .. (mesh.num_faces() - 1)
  short vlr_offset1;  // 0 .. (max_vertex_valence) (prob < valence / 2)
  ushort code;        // (vs_index (2), ii (2), ws (3), wt (3), wl (2), wr (2), fl_matid >= 0 (1), fr_matid >= 0 (1))
  enum EMaskBase : unsigned {
    B_STMASK = 0x0007,
    B_LSAME = 0x0001,
    B_RSAME = 0x0002,
    B_CSAME = 0x0004,

    B_LRMASK = 0x0003,
    B_ABOVE = 0x0000,
    B_BELOW = 0x0001,
    B_NEW = 0x0002,  // must be on separate bit.
  };
  enum EMask : unsigned {
    VSINDEX_SHIFT = 0,
    VSINDEX_MASK = 0x0003 << VSINDEX_SHIFT,

    II_SHIFT = 2,
    II_MASK = 0x0003 << II_SHIFT,

    S_SHIFT = 4,
    S_MASK = B_STMASK << S_SHIFT,
    S_LSAME = B_LSAME << S_SHIFT,
    S_RSAME = B_RSAME << S_SHIFT,
    S_CSAME = B_CSAME << S_SHIFT,

    T_SHIFT = 7,
    T_MASK = B_STMASK << T_SHIFT,
    T_LSAME = B_LSAME << T_SHIFT,
    T_RSAME = B_RSAME << T_SHIFT,
    T_CSAME = B_CSAME << T_SHIFT,

    L_SHIFT = 10,
    L_MASK = B_LRMASK << L_SHIFT,
    L_ABOVE = B_ABOVE << L_SHIFT,
    L_BELOW = B_BELOW << L_SHIFT,
    L_NEW = B_NEW << L_SHIFT,

    R_SHIFT = 12,
    R_MASK = B_LRMASK << R_SHIFT,
    R_ABOVE = B_ABOVE << R_SHIFT,
    R_BELOW = B_BELOW << R_SHIFT,
    R_NEW = B_NEW << R_SHIFT,

    FLN_SHIFT = 14,
    FLN_MASK = 1u << FLN_SHIFT,

    FRN_SHIFT = 15,
    FRN_MASK = 1u << FRN_SHIFT,
  };
  // *** Documentation:
  // vs_index: 0..2: index of vs within flace flclw
  // ii: 0..2: == alpha(1.0, 0.5, 0.0)
  //   ii = 2: a = 0.0 (old_vs = ~new_vs)
  //   ii = 1: a = 0.5
  //   ii = 0: a = 1.0 (old_vs = ~new_vt)
  // Inside wedges
  //  {S,T}{LSAME}: if exists outside left wedge and if same
  //  {S,T}{RSAME}: if exists outside right wedge and if same
  //  {S,T}{CSAME}: if inside left and right wedges are same
  //  (when no_vr, {S,T}RSAME == 1, {S,T}CSAME == 0)
  // Outside wedges
  //  (when no_vr, RABOVE == 1)
  // New face material identifiers
  //  {L,R}NF: if 1, face matids not predicted correctly using ii,
  //     so included in f{l,r}_matid
  //  (when no_vr, RNF == 0 obviously)
  //
  // *** Probabilities:
  //  vs_index: 0..2 (prob. uniform)
  //  ii: ii == 2 prob. low/med   (med if 'MeshSimplify -nominii1')
  //      ii == 0 prob. low/med
  //      ii == 1 prob. high/zero (zero if 'MeshSimplify -monminii1')
  //  {S,T}LSAME: prob. high
  //  {S,T}RSAME: prob. high
  //  {S,T}CSAME: prob. low
  //  {L,R}ABOVE: prob. high
  //  {L,R}BELOW: prob. low
  //  {L,R}NEW:   prob. low
  // Note: wl, wr, ws, wt are correlated since scalar half-edge
  //  discontinuities usually match up at both ends of edges.
  // -> do entropy coding on (ii, wl, wr, ws, wt) symbol as a whole.

  // ** Face attribute values (usually predicted correctly)
  // these are defined only if {L,R}NF respectively
  //  otherwise for now they are set to 0
  ushort fl_matid;
  ushort fr_matid;

  // ** Vertex attribute deltas
  // for ii == 2: vad_large = new_vt - old_vs, vad_small = new_vs - old_vs
  // for ii == 0: vad_large = new_vs - old_vs, vad_small = new_vt - old_vs
  // for ii == 1: vad_large = new_vt - new_i,  vad_small = new_i - old_vs
  //    where new_i=interp(new_vt, new_vs)
  PmVertexAttribD vad_large;
  PmVertexAttribD vad_small;  // is zero if "MeshSimplify -nofitgeom"

  // ** Wedge attribute deltas (size 1--6)
  Array<PmWedgeAttribD> ar_wad;
  // Order: [(wvtfl, wvsfl), [(wvtfr, wvsfr)], wvlfl, [wvrfr]]

  // ** Residual information:
  float resid_uni;
  float resid_dir;
  [[nodiscard]] int expected_wad_num(const PMeshInfo& pminfo) const;
};

// For each face, what are its 3 neighbors?
struct PmFaceNeighbors {
  Vec3<int> faces;  // faces[i] is across edge opposite of wedges[i].  < 0 if no neighbor.
};

// Wedge mesh augmented with adjacency information.
// Specifically, dual graph encoding face-face adjacency using PmFaceNeighbors.
class AWMesh : public WMesh {
 private:
  struct VF_range;
  struct VV_range;

 public:
  void read(std::istream& is, const PMeshInfo& pminfo);  // must be empty
  void write(std::ostream& os, const PMeshInfo& pminfo) const;
  void ok() const;

  // Rendering: common code and data
  static constexpr int k_Face_visited_mask = 1 << 30;  // high bit of matid
  int _cur_frame_mask{0};                              // 0 or k_Face_visited_mask

  // Rendering using OpenGL
  void ogl_render_faces_individually(const PMeshInfo& pminfo, int use_texture);
  void ogl_render_faces_strips(const PMeshInfo& pminfo, int use_texture);
  void ogl_render_edges();

  Array<PmFaceNeighbors> _fnei;  // must be same size as _faces!

  // Sentinel for an absent face or wedge; every negative entry in _fnei equals this value.
  static constexpr int k_undefined = k_debug ? std::numeric_limits<int>::min() : -1;

  [[nodiscard]] int most_clw_face(int v, int f) const;  // negative if v is interior vertex
  [[nodiscard]] int most_ccw_face(int v, int f) const;  // negative if v is interior vertex
  [[nodiscard]] bool is_boundary(int v, int f) const;
  [[nodiscard]] VF_range ccw_faces(int v, int f) const { return VF_range(*this, v, f); }
  [[nodiscard]] VV_range ccw_vertices(int v, int f) const { return VV_range(*this, v, f); }  // range over [vv, ff].
  // Split the edge between _faces[f].wedges[j] and _faces[f].wedges[mod3(j + 1)] with interp(v1, v2, frac1).
  void split_edge(int f, int j, float frac1);

 private:
  void construct_adjacency();
  void apply_vsplit_ancestry(Ancestry* ancestry, int vs, bool isr, int onumwedges, int code, int wvlfl, int wvrfr,
                             int wvsfl, int wvsfr, int wvtfl, int wvtfr);
  // Rendering using OpenGL
  Array<Pixel> _ogl_mat_byte_rgba;  // size is _materials.num()
  void ogl_process_materials();

  struct VF_sentinel {
    int _stopf{k_undefined};  // The face closing the ccw cycle, or k_undefined if v lies on a boundary.
  };

  struct VF_iterator {
    using type = VF_iterator;
    using iterator_concept = std::forward_iterator_tag;
    using value_type = int;
    using difference_type = std::ptrdiff_t;
    VF_iterator(const AWMesh& mesh, int v, int ff) : _mesh(&mesh), _v(v), _ff(ff), _beg(true) {}
    VF_iterator() = default;
    [[nodiscard]] bool operator==(const type& rhs) const { return _ff == rhs._ff && _beg == rhs._beg; }
    // The faces form a cycle, so only the initial position distinguishes begin() from the end of an interior cycle.
    [[nodiscard]] bool operator==(VF_sentinel s) const { return !_beg && _ff == s._stopf; }
    [[nodiscard]] int operator*() const noexcept { return ASSERTXX(_ff >= 0), _ff; }
    type& operator++() {
      _beg = false;
      _ff = _mesh->_fnei[_ff].faces[mod3(_mesh->get_jvf(_v, _ff) + 1)];  // Go ccw.
      return *this;
    }
    type operator++(int) { return postfix_increment(*this); }
    const AWMesh* _mesh{};
    int _v{};
    int _ff{k_undefined};
    bool _beg{};  // True only for begin(), to distinguish it from the end of the cycle.
  };

  struct VF_range : ranges::view_interface<VF_range> {
    VF_range(const AWMesh& mesh, int v, int f) : _mesh(&mesh), _v(v) {
      int ff = f, lastf;
      do {
        lastf = ff;
        ff = mesh._fnei[ff].faces[mod3(mesh.get_jvf(v, ff) + 2)];  // Go clw.
      } while (ff != k_undefined && ff != f);
      const bool is_boundary = ff == k_undefined;
      _startf = is_boundary ? lastf : f;       // Start at the most clw face if v lies on a boundary.
      _stopf = is_boundary ? k_undefined : f;  // On a boundary, the ccw walk instead falls off the edge.
    }
    [[nodiscard]] VF_iterator begin() const noexcept { return VF_iterator(*_mesh, _v, _startf); }
    [[nodiscard]] VF_sentinel end() const noexcept { return VF_sentinel{_stopf}; }
    // Note that size() is not trivially computable.
    const AWMesh* _mesh;
    int _v, _startf, _stopf;
  };

  struct VV_iterator {
    using type = VV_iterator;
    using iterator_concept = std::forward_iterator_tag;
    using value_type = std::pair<int, int>;
    using difference_type = std::ptrdiff_t;
    VV_iterator(const AWMesh& mesh, int v, int ff) : _mesh(&mesh), _v(v), _ff(ff) {
      _j = mesh.get_jvf(v, ff);
      _vv = mesh._wedges[mesh._faces[ff].wedges[mod3(_j + 1)]].vertex;
      _stopv = _vv;
      _nextv = mesh._wedges[mesh._faces[ff].wedges[mod3(_j + 2)]].vertex;
    }
    VV_iterator() = default;
    [[nodiscard]] bool operator==(const type& rhs) const { return _vv == rhs._vv && _ff == rhs._ff; }
    // The lookahead _nextv is set to k_undefined once the last neighbor has been reached.
    [[nodiscard]] bool operator==(std::default_sentinel_t) const { return _vv == k_undefined; }
    [[nodiscard]] std::pair<int, int> operator*() const noexcept {
      return ASSERTXX(_vv != k_undefined), std::pair{_vv, _ff};
    }
    type& operator++() {
      _vv = _nextv;
      const int lastf = _ff;
      _ff = _mesh->_fnei[_ff].faces[mod3(_j + 1)];  // Go ccw.
      if (_ff == k_undefined) {
        _nextv = k_undefined;
        _ff = lastf;
      } else {
        _j = _mesh->get_jvf(_v, _ff);
        _nextv = _mesh->_wedges[_mesh->_faces[_ff].wedges[mod3(_j + 2)]].vertex;
        if (_nextv == _stopv) _nextv = k_undefined;  // The cycle has closed.
      }
      return *this;
    }
    type operator++(int) { return postfix_increment(*this); }
    const AWMesh* _mesh{};
    int _v{}, _ff{k_undefined}, _j{}, _vv{k_undefined}, _nextv{k_undefined}, _stopv{k_undefined};
  };

  struct VV_range : ranges::view_interface<VV_range> {
    VV_range(const AWMesh& mesh, int v, int f) : _mesh(&mesh), _v(v) {
      int ff = f, lastf;
      do {
        lastf = ff;
        ff = mesh._fnei[ff].faces[mod3(mesh.get_jvf(v, ff) + 2)];  // Go clw.
      } while (ff != k_undefined && ff != f);
      _startf = ff == k_undefined ? lastf : f;  // Start at the most clw face if v lies on a boundary.
    }
    [[nodiscard]] VV_iterator begin() const noexcept { return VV_iterator(*_mesh, _v, _startf); }
    [[nodiscard]] std::default_sentinel_t end() const noexcept { return {}; }
    // Note that size() is not trivially computable.
    const AWMesh* _mesh;
    int _v, _startf;
  };

 protected:
  void apply_vsplit(const Vsplit& vspl, const PMeshInfo& pminfo, Ancestry* ancestry = nullptr);
  void undo_vsplit(const Vsplit& vspl, const PMeshInfo& pminfo);
  // Default operator=() and copy_constructor are safe.
 public:  // hidden
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
  Bbox<float, 3> _full_bbox;
};

// Progressive mesh:
//  contains a base mesh and a sequence of vertex split records.
class PMesh : noncopyable {
 public:
  PMesh();
  PMesh(AWMesh&& awmesh, const PMeshInfo& pminfo);
  // non-progressive read
  void read(std::istream& is);  // die unless empty
  void write(std::ostream& os) const;
  void truncate_beyond(PMeshIter& pmi);  // remove all vsplits beyond iterator
  void truncate_prior(PMeshIter& pmi);   // advance base mesh
 public:
  friend class PMeshRStream;
  AWMesh _base_mesh;
  Array<Vsplit> _vsplits;
  PMeshInfo _info;

 private:
  static constexpr uchar k_magic_first_byte = 0xFF;  // (Network-order first byte (MSB) of int flclw is <= 127.)
  [[nodiscard]] static PMeshInfo read_header(std::istream& is);
  [[nodiscard]] static bool at_trailer(std::istream& is) { return is.peek() == k_magic_first_byte; }
  // const AWMesh& base_mesh const { return _base_mesh; }
};

// Progressive mesh stream
// Can be either:
//  - read from an existing PMesh, or
//  - read from an input stream, or
//  - read from an input stream and archived to a PMesh
class PMeshRStream : noncopyable {
 public:
  explicit PMeshRStream(const PMesh& pm);
  explicit PMeshRStream(std::istream& is, PMesh* ppm_construct = nullptr);
  ~PMeshRStream();
  void read_base_mesh(AWMesh* bmesh = nullptr);  // always call this first!
  [[nodiscard]] const AWMesh& base_mesh();
  [[nodiscard]] bool is_reversible() const { return !!_pm; }
  const Vsplit* next_vsplit();
  const Vsplit* prev_vsplit();                     // die if !is_reversible()
  [[nodiscard]] const Vsplit* peek_next_vsplit();  // peek without using it
  PMeshInfo _info;

 private:
  friend PMeshIter;
  friend PMesh;             // for PMesh::truncate_*()
  std::istream* _is;        // may be nullptr
  PMesh* _pm;               // may be nullptr
  int _vspliti{-1};         // def if _pm, next to read from _pm->_vsplits; -1 before base_mesh is read
  Vsplit _tmp_vspl;         // def if !_pm
  bool _vspl_ready{false};  // def if !_pm, true if _vspl is only peeked
  AWMesh _lbase_mesh;       // used to store basemesh if !_pm
};

// Progressive mesh iterator (is a AWMesh!)
class PMeshIter : public AWMesh {
 public:
  explicit PMeshIter(PMeshRStream& pmrs);
  bool next() { return next_ancestry(nullptr); }  // ret: success
  bool prev();                                    // ret: success; die if !_pmrs.is_reversible()
  bool goto_nvertices(int nv) { return goto_nvertices_ancestry(nv, nullptr); }  // ret: success
  bool goto_nfaces(int nf) { return goto_nfaces_ancestry(nf, nullptr); }        // within +- 1, favor 0 or -1
  [[nodiscard]] PMeshRStream& rstream() { return _pmrs; }
  [[nodiscard]] const PMeshRStream& rstream() const { return _pmrs; }
  [[nodiscard]] GMesh extract_gmesh() const { return AWMesh::extract_gmesh(rstream()._info); }

 private:
  friend class Geomorph;
  friend PMesh;  // for PMesh::truncate_*()
  PMeshRStream& _pmrs;
  bool next_ancestry(Ancestry* ancestry);
  bool goto_nvertices_ancestry(int nvertices, Ancestry* ancestry);
  bool goto_nfaces_ancestry(int nfaces, Ancestry* ancestry);
  // Default operator=() is disabled due to reference; default copy_constructor is safe.
};

// Records vertex and wedge ancestry during PM traversal in order to construct geomorphs.
class Ancestry {
 public:
  Array<PmVertexAttrib> _vancestry;
  Array<PmWedgeAttrib> _wancestry;
};

// Geomorph endstates: pair of attributes for a changing vertex.
struct PmVertexAttribG {
  int vertex;
  Vec2<PmVertexAttrib> attribs;
};

// Geomorph endstates: pair of attributes for a changing wedge.
struct PmWedgeAttribG {
  int wedge;
  Vec2<PmWedgeAttrib> attribs;
};

// A geomorph is a mesh which is able to smoothly transition between two endstates.
// It is a mesh together with a list of vertices and wedges that change and their endstates.
// It can be evaluated over a continuous blend parameter 0 <= alpha <= 1.
// It is created by applying a sequence of vsplits to a PM iterator.
class Geomorph : public WMesh {
 public:
  // ** Construction:
  // Create a geomorph from pmi's current mesh to the mesh obtained after applying n vsplits to pmi.
  // Note side-effect on pmi!
  // Ret: was_able_to_go_all_the_way; die if !empty
  [[nodiscard]] bool construct_next(PMeshIter& pmi, int nvsplits);
  // Same up to nvertices
  // Ret: success
  bool construct_goto_nvertices(PMeshIter& pmi, int nvertices);
  // Same up to nfaces (or nfaces-1)
  // Ret: success
  bool construct_goto_nfaces(PMeshIter& pmi, int nfaces);

  // ** Evaluation:
  // Modify each vertex and wedge attributes of this mesh by linearly
  //  interpolating between attribs[0] and attribs[1].
  // Could optimize this by:
  //  (1) creating a list of only the changing attributes  (done)
  //  (2) precomputing (attribs[1] - attribs[0])
  //  (3) renormalizing only normals that vary significantly
  void evaluate(float alpha);  // 0 <= alpha <= 1 (0 == coarse)
 private:
  friend class SGeomorph;
  Array<PmVertexAttribG> _vgattribs;
  Array<PmWedgeAttribG> _wgattribs;
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
struct PmSVertexAttrib {
  PmVertexAttrib v;
  PmWedgeAttrib w;
};

void interp(PmSVertexAttrib& a, const PmSVertexAttrib& a1, const PmSVertexAttrib& a2, float frac1);

struct PmSVertex {
  PmSVertexAttrib attrib;
};

struct PmSFace {
  Vec3<int> vertices;
  PmFaceAttrib attrib;
};

struct PmSVertexAttribG {
  int vertex;
  Vec2<PmSVertexAttrib> attribs;
};
static_assert(std::is_standard_layout_v<PmSVertexAttribG>);
static_assert(std::is_trivially_default_constructible_v<PmSVertexAttribG>);
static_assert(std::is_trivially_copyable_v<PmSVertexAttribG>);

// Simple mesh: faces -> vertices.
// Split wedges into independent vertices.
class SMesh {
 public:
  explicit SMesh(const WMesh& wmesh);
  [[nodiscard]] GMesh extract_gmesh(int has_rgb, int has_uv) const;

 public:
  Materials _materials;
  Array<PmSVertex> _vertices;
  Array<PmSFace> _faces;
  // Default operator=() and copy_constructor are safe.
};

// Simple geomorph.
// Efficiency trade-off:
// - a simple mesh is quicker to render since there is one less level of indirection (no wedges).
// - but, it may be more expensive to geomorph since duplicated vertex positions must be interpolated separately.
class SGeomorph : public SMesh {
 public:
  explicit SGeomorph(const Geomorph& geomorph);
  void evaluate(float alpha);  // 0 <= alpha <= 1 (0 == coarse)
  [[nodiscard]] GMesh extract_gmesh(int has_rgb, int has_uv) const;

 private:
  Array<PmSVertexAttribG> _vgattribs;
  // Default operator=() and copy_constructor are safe.
};

//----------------------------------------------------------------------------

inline Vec3<int> WMesh::face_vertices(int f) const {
  Vec3<int> vertices;
  for_int(j, 3) vertices[j] = _wedges[_faces[f].wedges[j]].vertex;
  return vertices;
}

inline Vec3<Point> WMesh::face_points(int f) const {
  Vec3<Point> points;
  for_int(j, 3) points[j] = _vertices[_wedges[_faces[f].wedges[j]].vertex].attrib.point;
  return points;
}

#if defined(HH_DEBUG)
inline int WMesh::get_jvf(int v, int f) const {
  ASSERTX(_vertices.ok(v));
  ASSERTX(_faces.ok(f));
  for_int(j, 3) {
    if (_wedges[_faces[f].wedges[j]].vertex == v) return j;
  }
  assertnever("");
}
inline int WMesh::get_wvf(int v, int f) const { return _faces[f].wedges[get_jvf(v, f)]; }
#else
inline int WMesh::get_jvf(int v, int f) const {
  return _wedges[_faces[f].wedges[0]].vertex == v ? 0 : _wedges[_faces[f].wedges[1]].vertex == v ? 1 : 2;
}
inline int WMesh::get_wvf(int v, int f) const {
  int w0 = _faces[f].wedges[0];
  if (_wedges[w0].vertex == v) return w0;
  int w1 = _faces[f].wedges[1];
  if (_wedges[w1].vertex == v) return w1;
  int w2 = _faces[f].wedges[2];
  return w2;
}
#endif  // defined(HH_DEBUG)

inline bool Vsplit::adds_two_faces() const { return vlr_offset1 > 1; }

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_PMESH_H_
