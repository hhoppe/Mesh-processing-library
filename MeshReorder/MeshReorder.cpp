// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt

#include <cstring>  // strcmp() etc.

#include "libHh/Args.h"
#include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"
#include "libHh/GMesh.h"
#include "libHh/Queue.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Set.h"
#include "libHh/Stat.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
#include "libHh/VertexCache.h"
using namespace hh;

namespace {

// TODO:
//
// - When selecting new component, try to make same material;
//   possibly using "Set<string>" as in MeshSimplify,
//   in order to reduce the material state changes for rendering.

// Assumptions:
//
// - MeshReorder does not examine corner attributes at all!
//
// - If the mesh has corner information and the graphics API does not
//   support indexed wedges (as does OpenGL IndexedFaceSet), then the
//   input mesh must be pre-processed to introduce duplicate vertices,
//   using "Filtermesh mesh.m -splitcorners >mesh_split.m".
//
// - If the mesh has face materials and the graphics API does not support
//   sharing of vertices across material boundaries (as does OpenGL
//   IndexedFaceSet), then the input mesh must be pre-processed to introduce
//   duplicate vertices,
//   using "Filtermesh mesh.m -splitmatbnd >mesh_split.m".
//
// - MeshReorder does examine Face attributes, as it treats separately each
//   face-connected component with the same face material.
//   This is important for the OpenGL IndexedFaceSet model since the separate
//   face_material components may still be connected in the input mesh.

// Thus two modes of operation:
//
// - for D3D: pre-process the mesh using
//   "Filtermesh mesh.m -splitcorners -splitmatbnd >mesh_split.m",
//   and MeshReorder works as expected.
//
// - OpenGL_IndexedFaceSet: don't pre-process the mesh,
//   and MeshReorder automatically treats separately components with
//   different materials.

// Notes:
//
// - Inlining FifoVertexCache::access() using defined(FAST_FIFO) did not improve performance significantly.

// This number could be set according to the flexible vertex format size.
constexpr int k_bytes_per_vertex = 32;  // default position + normal + uv

// This assumes D3D limit of 64 Ki vertices.
constexpr int k_bytes_per_vindex = 2;

// Penalty for restarting triangle strips in terms of vertex indices:
//  is 0 or 1 if separate DrawIndexedPrimitiveVB() calls are used
//  is 1 if a single "-1" index causes a strip restart
//  is 2 if two duplicate indices specify a strip restart.
constexpr int k_strip_restart_nvindices = 1;

// Cache type must be set explicitly on the command line.
VertexCache::EType cache_type = VertexCache::EType::notype;

int cache_size = 16;              // default size is 16-entry cache
bool favor_spiral = true;         // only used by greedy_stripify
bool color_corners = false;       // color corners according to cache misses
bool color_fmiss = false;         // color faces according to cache misses
bool color_forder = false;        // color faces according to rendering order
bool old_strip_order = false;     // obsolete old way of indexing strips
bool reorder_vertices = false;    // reorder vertices according to first use
bool duplicate_vertices = false;  // strict linear read order
bool nooutput = false;            // if 1, do no write out final reordered mesh
int verb = 1;                     // verbosity level of output

// Override the variable k used in lookahead simulation.
const int simulate_nf = getenv_int("SIMULATE_NF");

// Global data structures

GMesh mesh;
Array<int> ar_verts;   // [3*mesh.num_faces()] -> 1..num_vertices()
Array<Face> ar_faces;  // [mesh.num_faces()] -> original Face
string gfilename;

// This data is only used by "-diff_corners" to compare cache misses
//  before and after reordering.
HH_SAC_ALLOCATE_FUNC(Mesh::MFace, int, f_oldfid);

// *** Helper

inline bool same_string(const char* s1, const char* s2) {
  return !s1 && !s2 ? true : !s1 || !s2 ? false : !strcmp(s1, s2);
}

void show_rate(Timer& timer) {
  timer.stop();
  double runtime = timer.cpu();
  showdf("reordering rate: %.0f faces / sec\n", mesh.num_faces() / max(runtime, 1e-6));
  timer.start();
}

// Given a face, get its 3 vertices and its 3 adjacent faces.
// Note: fa[] may contain zeros if at a mesh boundary.
void face_vertices_neighbors(Face f, Array<Vertex>& va, Array<Face>& fa) {
  mesh.get_vertices(f, va);
  fa.init(0);
  for (Vertex v : va) fa.push(mesh.opp_face(v, f));  // may be zero
}

// Record the visit of a face into the global data structure.
// Access the face's vertices starting at the given corner
//  and using the cache.
int record_face(Corner cc, VertexCache& vcache) {
  int nmiss = 0;
  for_int(j, 3) {
    Vertex v = mesh.corner_vertex(cc);
    ar_verts.push(mesh.vertex_id(v));
    bool miss = !vcache.access_hits(mesh.vertex_id(v));
    nmiss += miss;
    cc = mesh.ccw_face_corner(cc);
  }
  ar_faces.push(mesh.corner_face(cc));
  return nmiss;
}

// For a given face, return the number of vertices currently in the cache.
int face_nvcached(Face f, const VertexCache& vcache) {
  int nvcached = 0;
  for (Vertex v : mesh.vertices(f)) nvcached += vcache.contains(mesh.vertex_id(v));
  return nvcached;
}

// *** Main

void fixup_mesh() {
  // Renumber the mesh vertices and faces if necessary.
  int maxvid = 0;
  for (Vertex v : mesh.vertices()) maxvid = max(maxvid, mesh.vertex_id(v));
  int maxfid = 0;
  for (Face f : mesh.faces()) maxfid = max(maxfid, mesh.face_id(f));
  if (!assertw(maxvid <= mesh.num_vertices()) || !assertw(maxfid <= mesh.num_faces())) {
    Warning("Renumbering mesh vertices and faces");
    mesh.renumber();
  }
  showdf("Mesh v=%d f=%d\n", mesh.num_vertices(), mesh.num_faces());
  assertx(mesh.num_faces());
  // Save some data, only used by the diff_corners() option.
  string str;
  for (Face f : mesh.faces()) {
    const char* s = GMesh::string_key(str, mesh.get_string(f), "oldfid");
    f_oldfid(f) = s ? to_int(s) : 0;
    mesh.update_string(f, "oldfid", nullptr);
  }
}

// For use with:
//  MeshReorder mesh1.m -diff_corner mesh2.m >mesh3.m
// Compares the cache misses in the two orderings mesh1 and mesh2
//  and outputs the mesh mesh3 which is identical to mesh1 except that
//  the misses in mesh1 that are eliminated in mesh2 are colored blue.
void do_diff_corners(Args& args) {
  GMesh mesh2{RFile(args.get_filename())()};
  assertx(mesh2.num_vertices() == mesh.num_vertices());
  assertx(mesh2.num_faces() == mesh.num_faces());
  Map<int, int> mapfoldf;
  string str;
  for (Face f2 : mesh2.faces()) {
    int oldfid = to_int(assertx(GMesh::string_key(str, mesh.get_string(f2), "oldfid")));
    mapfoldf.enter(oldfid, mesh.face_id(f2));
  }
  string str2;
  for (Vertex v : mesh.vertices()) {
    for (Corner c : mesh.corners(v)) {
      Face f = mesh.corner_face(c);
      Vertex v2 = mesh2.id_vertex(mesh.vertex_id(v));
      Face f2 = mesh2.id_face(mapfoldf.get(mesh.face_id(f)));
      Corner c2 = mesh2.corner(v2, f2);
      const char* s = assertx(GMesh::string_key(str, mesh.get_string(c), "rgb"));
      const char* s2 = assertx(GMesh::string_key(str2, mesh2.get_string(c2), "rgb"));
      if (!strcmp(s, s2)) continue;
      bool was_red = !strstr(s, ".0 .0");
      if (!was_red) continue;
      mesh.update_string(c, "rgb", "(0. 0. .6)");
    }
  }
  color_corners = false;
}

// Go from the GMesh to the two global arrays.
void extract_mesh() {
  ar_verts.reserve(mesh.num_faces() * 3);
  ar_faces.reserve(mesh.num_faces());
  for_int(fi, mesh.num_faces()) {
    Face f = mesh.id_face(1 + fi);
    assertx(mesh.is_triangle(f));
    for (Vertex v : mesh.vertices(f)) {
      int vid = mesh.vertex_id(v);
      ar_verts.push(vid);
    }
    ar_faces.push(f);
  }
}

// Use the two (modified) global arrays to reorder the faces in GMesh.
void replace_mesh() {
  HH_PTIMER("_replace_mesh");
  if (0) {  // simplest version
    assertx(!color_forder && !color_fmiss && !color_corners);
    assertx(reorder_vertices);
    GMesh nmesh;
    Map<int, int> movivi;  // 1..num_vertices() --> 1..num_vertices()
    for_int(vi, mesh.num_vertices()) nmesh.create_vertex();
    Vec3<Vertex> va;
    for_int(fi, mesh.num_faces()) {
      for_int(j, 3) {
        int ovi = ar_verts[fi * 3 + j];
        bool is_new;
        int& ni = movivi.enter(ovi, movivi.num() + 1, is_new);
        dummy_use(is_new);
        va[j] = nmesh.id_vertex(ni);
      }
      nmesh.create_face(va);
    }
    for (Vertex v : mesh.vertices()) {
      nmesh.set_point(nmesh.id_vertex(movivi.get(mesh.vertex_id(v))), mesh.point(v));
    }
    // For now, ignore all string information.
    mesh.copy(nmesh);
    return;
  }
  if (duplicate_vertices) {
    assertx(!color_forder && !color_fmiss && !color_corners);
    assertx(reorder_vertices);
    GMesh nmesh;
    Map<int, int> movivi;  // 1..num_vertices() --> 1..num_vertices()
    const int allowed_num_verts = 2 * mesh.num_vertices();
    auto up_vcache = VertexCache::make(cache_type, 1 + allowed_num_verts, cache_size);
    VertexCache& vcache = *up_vcache;
    Vec3<Vertex> va;
    for_int(fi, mesh.num_faces()) {
      for_int(j, 3) {
        int ovi = ar_verts[fi * 3 + j];
        bool is_new;
        int& vi = movivi.enter(ovi, 0, is_new);
        if (is_new || !vcache.contains(vi)) {
          assertx(nmesh.num_vertices() < allowed_num_verts);
          Vertex v = nmesh.create_vertex();
          nmesh.set_point(v, mesh.point(mesh.id_vertex(ovi)));
          vi = nmesh.vertex_id(v);
        }
        vcache.access_hits(vi);
        va[j] = nmesh.id_vertex(vi);
      }
      nmesh.create_face(va);
    }
    // For now, ignore all string information.
    mesh.copy(nmesh);
    return;
  }
  Map<int, int> movivi;  // 1..num_vertices() --> 1..num_vertices()
  if (reorder_vertices) {
    for_int(fi, mesh.num_faces()) {
      for_int(j, 3) {
        int ovi = ar_verts[fi * 3 + j];
        bool is_new;
        movivi.enter(ovi, movivi.num() + 1, is_new);
        dummy_use(is_new);
      }
    }
  } else {
    for_int(i, mesh.num_vertices()) movivi.enter(i + 1, i + 1);
  }
  GMesh nmesh;
  for (Vertex v : mesh.vertices()) {
    Vertex vn = nmesh.create_vertex_private(movivi.get(mesh.vertex_id(v)));
    nmesh.set_string(vn, mesh.get_string(v));
    nmesh.set_point(vn, mesh.point(v));
  }
  Vec3<Vertex> va;
  string str;
  for_int(fi, mesh.num_faces()) {
    Face f = ar_faces[fi];
    for_int(j, 3) va[j] = nmesh.id_vertex(movivi.get(ar_verts[fi * 3 + j]));
    Face fn = nmesh.create_face_private(1 + fi, va);
    nmesh.set_string(fn, mesh.get_string(f));
    if (f_oldfid(f)) nmesh.update_string(fn, "oldfid", csform(str, "%d", f_oldfid(f)));
    for (Corner c : mesh.corners(f)) {
      if (!mesh.get_string(c)) continue;
      Vertex v = mesh.corner_vertex(c);
      Vertex vn = nmesh.id_vertex(mesh.vertex_id(v));
      Corner cn = nmesh.corner(vn, fn);
      nmesh.set_string(cn, mesh.get_string(c));
    }
  }
  for (Edge e : mesh.edges()) assertx(!mesh.get_string(e));  // not handled at present
  mesh.copy(nmesh);
  if (color_forder) {
    for (Face f : mesh.faces()) {
      for (Corner c : mesh.corners(f)) mesh.update_string(c, "rgb", nullptr);
    }
    int nf = 0;
    Face fo = nullptr;
    for_int(fi, mesh.num_faces()) {
      Face f = mesh.id_face(1 + fi);
      const bool strip_continuation = contains(mesh.faces(f), fo);
      fo = f;
      nf++;
      if (!strip_continuation) nf = 0;
      const float ftot = float(fi) / mesh.num_faces();
      const int max_nf = 20;
      const float floc = (min(nf, max_nf - 1) % max_nf) / float(max_nf) * .7f + .15f;
      Vector rgb(ftot, floc, .5f);
      mesh.update_string(f, "rgb", csform_vec(str, rgb));
    }
  } else if (color_fmiss && cache_type != VertexCache::EType::notype) {
    HH_PTIMER("__color_fmiss");
    auto up_vcache = VertexCache::make(cache_type, 1 + mesh.num_vertices(), cache_size);
    VertexCache& vcache = *up_vcache;
    const Vec<const char*, 4> color_nmiss = {
        // "(.6 .8 .6)",
        // "(.8 .45 .3)",
        // "(.8 .0 .0)",
        // "(.8 .0 .0)",
        "(.6 .8 .6)",
        "(.9 .45 .0)",
        "(.65 .0 .0)",
        "(.0 .0 .6)",
    };
    for_int(fi, mesh.num_faces()) {
      Face f = mesh.id_face(1 + fi);
      int nmiss = 0;
      for_int(j, 3) {
        int vi = movivi.get(ar_verts[fi * 3 + j]);
        bool miss = !vcache.access_hits(vi);
        nmiss += miss;
      }
      mesh.update_string(f, "rgb", color_nmiss[nmiss]);
    }
    for (Face f : mesh.faces()) {
      for (Corner c : mesh.corners(f)) mesh.update_string(c, "rgb", nullptr);
    }
  } else if (color_corners && cache_type != VertexCache::EType::notype) {
    HH_PTIMER("__color_corners");
    auto up_vcache = VertexCache::make(cache_type, 1 + mesh.num_vertices(), cache_size);
    VertexCache& vcache = *up_vcache;
    const char* color_miss = "(.8 .0 .0)";
    const char* color_hit = "(.6 .8 .6)";
    for_int(fi, mesh.num_faces()) {
      Face f = mesh.id_face(1 + fi);
      for_int(j, 3) {
        int vi = movivi.get(ar_verts[fi * 3 + j]);
        Vertex v = mesh.id_vertex(vi);
        Corner c = mesh.corner(v, f);
        bool miss = !vcache.access_hits(vi);
        mesh.update_string(c, "rgb", (miss ? color_miss : color_hit));
      }
    }
  }
}

void do_fifo() { cache_type = VertexCache::EType::fifo; }

void do_lru() { cache_type = VertexCache::EType::lru; }

// Print some statistics and return number of vertex cache misses.
int analyze_mesh(int cs) {
  auto up_vcache = VertexCache::make(cache_type, 1 + mesh.num_vertices(), cs);
  VertexCache& vcache = *up_vcache;
  int nmiss = 0;
  for_int(i, ar_verts.num()) nmiss += !vcache.access_hits(ar_verts[i]);
  showdf("cs=%-6d nmiss=%-6d %5.1f%%  v/t=%5.3f  v/v=%5.3f\n",  //
         cs, nmiss, float(nmiss) / ar_verts.num() * 100.f, float(nmiss) / mesh.num_faces(),
         float(nmiss) / mesh.num_vertices());
  return nmiss;
}

void analyze_strips(int& pnverts, int& pnstrips) {
  const bool debug = false;
  // oldway: vertices to reuse in next face are in vid[1..2]
  //          expected_j: 2, 0, 2, 0, ...
  // newway: vertices reused from prev face are in vid[0..1]
  //          expected_j: 1, 2, 1, 2, ...
  bool oldway = old_strip_order;
  // Assumption: turn face1-face2-face3 is expected to be ccw.
  const int first_expected_j = oldway ? 2 : 1;
  const int sum_expected_j = oldway ? 2 : 3;
  const char* last_mat_string = "? ?";
  Vec3<int> ovid;
  dummy_init(ovid);
  int expected_j;
  dummy_init(expected_j);
  int nstrips = 0;
  int nverts = 0;
  for_int(fi, mesh.num_faces()) {
    const char* mat_string = mesh.get_string(ar_faces[fi]);
    if (!same_string(last_mat_string, mat_string)) {
      last_mat_string = mat_string;
      // force new strip at mat boundary
      fill(ovid, -1);
    }
    int j = 3;
    for_int(k, 3) {
      if (oldway) {
        if (ar_verts[fi * 3 + mod3(k + 0)] == ovid[2] && ar_verts[fi * 3 + mod3(k + 1)] == ovid[1]) {
          j = k;
          break;
        }
      } else {
        if (ovid[mod3(k + 0)] == ar_verts[fi * 3 + 1] && ovid[mod3(k + 1)] == ar_verts[fi * 3 + 0]) {
          j = k;
          break;
        }
      }
    }
    if (j == 3) {  // not face-face connected
      if (debug) std::cerr << " H";
      nstrips++;
      if (fi) nverts += k_strip_restart_nvindices;
      nverts += 3;
      expected_j = first_expected_j;
    } else if (j == expected_j) {
      if (debug) std::cerr << ".";
      nverts += 1;
      expected_j = sum_expected_j - expected_j;
    } else if (j == sum_expected_j - expected_j) {
      if (debug) std::cerr << ":";
      nverts += 2;
      // expected_j stays the same:  LRLR*R*LRLR
    } else {
      if (debug) std::cerr << "*";
      if (0) Warning("Strip turns on itself");
      nstrips++;
      if (fi) nverts += k_strip_restart_nvindices;
      nverts += 3;
      expected_j = first_expected_j;
    }
    for_int(c, 3) ovid[c] = ar_verts[fi * 3 + c];
  }
  if (debug) std::cerr << "\n";
  showdf("Vertex_indices: nstrips=%d, nverts=%d, vi/t=%-5.3f\n", nstrips, nverts, float(nverts) / mesh.num_faces());
  pnverts = nverts;
  pnstrips = nstrips;
}

// Analyze the bandwidth of the mesh under the transparent vertex caching framework,
//  using the current cache type and size.
void do_analyze() {
  HH_PTIMER("_analyze");
  showdf("Mesh analysis (%s)\n", VertexCache::type_string(cache_type).c_str());
  int nmiss = analyze_mesh(cache_size);
  int nverts, nstrips;
  analyze_strips(nverts, nstrips);
  float b_v = float(nmiss * k_bytes_per_vertex) / mesh.num_faces();
  float b_i = float(nverts * k_bytes_per_vindex) / mesh.num_faces();
  float b_t = b_v + b_i;
  showdf("Bandwidth: vertices %4.2f b/t, indices %4.2f b/t, Total %4.2f byte/tri\n", b_v, b_i, b_t);
  string nametail = get_path_tail(gfilename);
  showdf("%-14.14s v/t=%5.3f v/v=%5.3f slen=%4.1f bv=%4.2f bi=%4.2f bt=%4.2f\n",  //
         nametail.c_str(), float(nmiss) / mesh.num_faces(), float(nmiss) / mesh.num_vertices(),
         float(mesh.num_faces()) / nstrips, b_v, b_i, b_t);
}

// Analyze the badnwidth of the mesh under the traditional triangle strip
//  framework (without vertex caching).
void do_strip_analyze() {
  HH_PTIMER("_strip_analyze");
  showdf("Strip analysis\n");
  int nverts, nstrips;
  analyze_strips(nverts, nstrips);
  // number of vertices transferred
  int nvt = nverts - (nstrips - 1) * k_strip_restart_nvindices;
  showdf("nverts_transf=%d     %5.1f%%  v/t=%5.3f  v/v=%5.3f\n",  //
         nvt, float(nvt) / ar_verts.num() * 100.f, float(nvt) / mesh.num_faces(), float(nvt) / mesh.num_vertices());
  showdf("Bandwidth:                                       Total %4.2f byte/tri\n",
         float(nvt * k_bytes_per_vertex) / mesh.num_faces());
}

void do_fanalyze() {
  showdf("Mesh full analysis (%s)\n", VertexCache::type_string(cache_type).c_str());
  for (int cs = 1;;) {
    analyze_mesh(cs);
    if (cs > mesh.num_vertices()) break;
    if (cs < 8)
      cs++;
    else if (cs == cache_size + 0)
      cs++;
    else if (cs == cache_size + 1)
      cs++;
    else if (cs == cache_size + 2)
      cs = (cs - 2) * 2;
    else
      cs *= 2;
  }
  do_analyze();
}

// Given an order of faces in the mesh,
//  reorder the vertices within each face so as to fit the access pattern
//  of an indexed triangle strip representation.
void do_fixup_indices() {
  Array<Vertex> va;
  Array<Face> fa;
  if (old_strip_order) {
    // Note: running this routine on the output of my meshify*() algorithms
    //  is not recommended.
    // Although it will generally reduce the number of vertex indices,
    //  it will also slightly increase the number of vertex misses because
    //  of slight changes in strip end conditions.
    // Example with bunny.lru.meshify2.m:
    //Before:
    // # cs=16     nmiss=42823   20.5%  v/t=0.616  v/v=1.229
    // # Vertex_indices: nstrips=6643, nverts=96717, vi/t=1.488
    // # Bandwidth: vertices 19.72 b/t, indices 2.98 b/t, Total 22.70 b/t
    //After:
    // # cs=16     nmiss=42920   20.6%  v/t=0.618  v/v=1.232
    // # Vertex_indices: nstrips=6600, nverts=95781, vi/t=1.474
    // # Bandwidth: vertices 19.77 b/t, indices 2.95 b/t, Total 22.72 b/t
    Vertex ov1 = nullptr;  // previous face not adjacent
    bool last_clw;
    dummy_init(last_clw);
    for_int(fi, mesh.num_faces()) {
      Face f = ar_faces[fi];
      Face fn = fi < mesh.num_faces() - 1 ? ar_faces[fi + 1] : nullptr;
      face_vertices_neighbors(f, va, fa);
      for_int(k, 3) ASSERTX(va.contains(mesh.id_vertex(ar_verts[fi * 3 + k])));
      int ifn = !fn ? -1 : fa.index(fn);  // -1 if next face not adjacent
      int iov1 = va.index(ov1);           // -1 if prev face not adjacent
      int j;
      if (ifn >= 0) {  // have next face, so that determines order
        j = ifn;
        if (iov1 >= 0) {
          last_clw = j != iov1;
        } else {
          last_clw = true;  // second turn should be ccw
        }
      } else if (iov1 >= 0) {  // no next face, so sequential from prev
        if (last_clw) {
          j = iov1;  // ccw turn
        } else {
          j = mod3(iov1 + 2);  // clw turn
        }
      } else {  // no next face or previous face; anything
        j = index(va, mesh.id_vertex(ar_verts[fi * 3 + 0]));
      }
      for_int(k, 3) ar_verts[fi * 3 + k] = mesh.vertex_id(va[mod3(j + k)]);
      ov1 = ifn < 0 ? nullptr : va[mod3(j + 1)];
    }
  } else {
    Vec3<Vertex> ov;
    dummy_init(ov);
    for_int(fi, mesh.num_faces()) {
      Face f = ar_faces[fi];
      face_vertices_neighbors(f, va, fa);
      for_int(k, 3) ASSERTX(va.contains(mesh.id_vertex(ar_verts[fi * 3 + k])));
      int j = 3;
      for_int(kc, 3) {
        for_int(kp, 3) {
          if (va[mod3(kc + 0)] == ov[mod3(kp + 1)] && va[mod3(kc + 1)] == ov[mod3(kp + 0)]) {
            j = kc;
            break;
          }
        }
      }
      if (j < 3) {  // have connection with previous face
                    // j is OK as is
      } else {      // no previous face, examine next face
        Face fn = fi < mesh.num_faces() - 1 ? ar_faces[fi + 1] : nullptr;
        int ifn = !fn ? -1 : fa.index(fn);  // -1 if next face not adjacent
        if (ifn >= 0) {
          j = ifn;
        } else {  // no next face; anything goes
          j = index(va, mesh.id_vertex(ar_verts[fi * 3 + 0]));
        }
      }
      for_int(k, 3) {
        Vertex v = va[mod3(j + k)];
        ar_verts[fi * 3 + k] = mesh.vertex_id(v);
        ov[k] = v;
      }
    }
  }
}

// Print basic information on current face ordering.
void do_info() {
  HH_PTIMER("_info");
  analyze_mesh(cache_size);
}

// Randomly reorder the faces.
void do_randomize_faces() {
  Array<int> ar(mesh.num_faces());
  for_int(fi, mesh.num_faces()) ar[fi] = fi;
  shuffle(ar, Random::G);
  ar_verts.init(0);
  ar_faces.init(0);
  Array<Vertex> va;
  for (int fi : ar) {
    Face f = mesh.id_face(1 + fi);
    mesh.get_vertices(f, va);
    int rot = Random::G.get_unsigned(3);
    for_int(j, 3) ar_verts.push(mesh.vertex_id(va[mod3(rot + j)]));
    ar_faces.push(f);
  }
}

// *** MeshStatus

const int random_initial_face = getenv_int("RANDOM_INITIAL_FACE");

// Faces are exclusively in one of the 5 linked lists:
//  MeshStatus::_l_uco and MeshStatus::_l_unp_nnei[0..3]
// Initially, they are in the list of "unvisited components" l_uco.
// Then, when a connected component is visited, the faces are pulled out
//  of l_uco and into the lists of "unprocessed faces with n neighbors"
//  l_unp_nnei where n is 0..3 .
// Finally, when a face is processed (within the current component),
//  it is removed from l_unp_nnei and no longer appears on any list.

struct FaceEList {
  Face f;            // pointer back to containing face
  EListNode el_uco;  // within unvisited components
  EListNode el_unp;  // unprocessed within current component
};
HH_SACABLE(FaceEList);  // EListNode requires its constructor

// Associate the above linked_list nodes with each mesh face.
HH_SAC_ALLOCATE_CD_FUNC(Mesh::MFace, FaceEList, f_elist);

// Associate an integer with each mesh face:
//  == std::numeric_limits<int>::max() if the face has already been (globally) processed.
//  == MeshStatus::_sim_num if the face has been visited in the current
//     lookahead simulation
HH_SAC_ALLOCATE_FUNC(Mesh::MFace, int, f_sim_num);

// Only one MeshStatus object may be defined at any time,
// since it operates closely with the global variable mesh.
class MeshStatus {
 public:
  MeshStatus();
  ~MeshStatus();

  // Global state:
  void process(Face f);
  bool processed(Face f) const;
  int face_nnei(Face f) const;  // number of unprocessed neighbors
  Face find_initial_face();
  Corner find_initial_corner(const VertexCache& vcache);

  // Lookahead simulation (does not affect global state):
  void sim_init();
  bool sim_face_visited(Face f) const;
  void sim_visit_face(Face f);

 private:
  Vec<EList, 4> _l_unp_nnei;         // unprocessed faces with 0..3 unpr. neighbors
  EList _l_uco;                      // faces within unvisited components
  int _sim_num{0};                   // simulation number
  bool initialize_next_component();  // ret: true=success, false=no_more
};

inline bool MeshStatus::processed(Face f) const {
  // Cannot do ASSERTX(!f_elist(f).el_uco.linked())
  // because this may be a neigbhoring face (as in face_nnei()) which
  // is in l_uco (due to a material boundary).
  // But in this case this function declares the face as processed,
  // which is OK.
  return !f_elist(f).el_unp.linked();
}

MeshStatus::MeshStatus() {
  if (random_initial_face) Warning("random_initial_face");
  if (random_initial_face >= 2) Warning("random_initial_corner");
  for (Face f : mesh.ordered_faces()) {
    f_elist(f).f = f;
    f_elist(f).el_uco.link_before(_l_uco.delim());
    f_sim_num(f) = _sim_num;
  }
}

MeshStatus::~MeshStatus() {
  for_int(nnei, 4) ASSERTX(_l_unp_nnei[nnei].empty());
  ASSERTX(_l_uco.empty());
  for (Face f : mesh.faces()) ASSERTX(processed(f));
}

int MeshStatus::face_nnei(Face f) const {
  int nnei = 0;
  for (Face ff : mesh.faces(f)) nnei += !processed(ff);
  return nnei;
}

void MeshStatus::process(Face f) {
  ASSERTX(!processed(f));
  f_elist(f).el_unp.unlink();
  if (!random_initial_face) {
    // Decrement nnei for each of the face's neighbors.
    for (Face ff : mesh.faces(f)) {
      if (processed(ff)) continue;
      int nnei = face_nnei(ff);
      f_elist(ff).el_unp.relink_after(_l_unp_nnei[nnei].delim());
    }
  }
  f_sim_num(f) = std::numeric_limits<int>::max();
}

void MeshStatus::sim_init() {
  _sim_num++;
  assertx(_sim_num < std::numeric_limits<int>::max());
}

inline bool MeshStatus::sim_face_visited(Face f) const {
  // face is visited either in current lookahead simulation (== _sim_num)
  //  or is globally visited (== std::numeric_limits<int>::max()).
  return f_sim_num(f) >= _sim_num;
}

inline void MeshStatus::sim_visit_face(Face f) {
  ASSERTX(!sim_face_visited(f));
  f_sim_num(f) = _sim_num;
}

bool MeshStatus::initialize_next_component() {
  for_int(nnei, 4) assertx(_l_unp_nnei[nnei].empty());
  if (_l_uco.empty()) return false;
  // Gather faces in component containing some initial face f.
  // Identify these faces by placing them in list l_c for now and
  //  removing them from _l_uco.
  EList l_c;
  int numfc = 0;
  {
    Face f = HH_ELIST_OUTER(FaceEList, el_uco, _l_uco.delim()->next())->f;
    Queue<Face> queue;
    f_elist(f).el_uco.unlink();
    f_elist(f).el_unp.link_before(l_c.delim());
    numfc++;
    for (;;) {
      for (Face f2 : mesh.faces(f)) {
        if (!same_string(mesh.get_string(f), mesh.get_string(f2))) continue;
        if (f_elist(f2).el_unp.linked()) continue;  // already added in component
        queue.enqueue(f2);
        f_elist(f2).el_uco.unlink();
        f_elist(f2).el_unp.link_before(l_c.delim());
        numfc++;
      }
      if (queue.empty()) break;
      f = queue.dequeue();
    }
  }
  if (verb >= 2) showdf("***New component: %d faces\n", numfc);
  HH_SSTAT(Scompnf, numfc);
  for (EListNode* nodee = l_c.delim()->next(); nodee != l_c.delim();) {
    Face f = HH_ELIST_OUTER(FaceEList, el_unp, nodee)->f;
    EListNode* next_nodee = nodee->next();
    int nnei = face_nnei(f);
    if (random_initial_face) nnei = 0;
    nodee->relink_before(_l_unp_nnei[nnei].delim());
    nodee = next_nodee;
  }
  assertx(l_c.empty());
  return true;
}

Face MeshStatus::find_initial_face() {
  for (;;) {
    for_int(nnei, 4) {
      if (_l_unp_nnei[nnei].empty()) continue;
      EListNode* nodee = _l_unp_nnei[nnei].delim()->next();
      Face f = HH_ELIST_OUTER(FaceEList, el_unp, nodee)->f;
      ASSERTX(!processed(f));
      return f;
    }
    // All unprocessed lists are empty, so replenish them.
    if (!initialize_next_component()) return nullptr;
    // Since found next component, a face is guaranteed, so jump back.
  }
}

// In these algorithms, when a Corner refers to a face to visit, it refers
//  to the left corner as the algorithm crosses the edge to visit the face.
Corner MeshStatus::find_initial_corner(const VertexCache& vcache) {
  Set<Face> setf;
  {  // unprocessed faces adjacent to verts in cache
    auto up_vci = vcache.make_iterator();
    VertexCache::Iter& vci = *up_vci;
    for (;;) {
      int vi = vci.next();
      if (!vi) break;
      Vertex v = mesh.id_vertex(vi);
      for (Face f : mesh.faces(v)) {
        if (!processed(f)) setf.add(f);
      }
    }
  }
  HH_SSTAT(Sinitnf, setf.num());
  Face best_f = nullptr;
  int max_nvcached = -1;
  int min_nnei = std::numeric_limits<int>::max();
  for (Face f : setf) {
    int nvcached = face_nvcached(f, vcache), nnei;
    if (nvcached > max_nvcached ||
        (nvcached == max_nvcached &&
         (nnei = face_nnei(f), nnei < min_nnei ||
                                   // to get reproducible reordering
                                   (nnei == min_nnei && mesh.face_id(f) < mesh.face_id(best_f))))) {
      best_f = f;
      max_nvcached = nvcached;
      min_nnei = face_nnei(f);
    }
  }
  Face f = best_f;
  if (!f || random_initial_face) {
    f = find_initial_face();
    if (!f) return nullptr;
    max_nvcached = 0;
    min_nnei = face_nnei(f);
  }
  HH_SSTAT(Sinitnvc, max_nvcached);
  HH_SSTAT(Sinitnnei, min_nnei);
  Corner c0;
  dummy_init(c0);
  for (Corner c : mesh.corners(f)) {
    c0 = c;
    if (1) break;
  }
  if (random_initial_face >= 2) return c0;
  for (Corner c : mesh.corners(f)) {
    Corner cc = mesh.clw_corner(c);
    if (!cc || processed(mesh.corner_face(cc))) {
      c0 = c;
      break;
    }
  }
  {
    Corner cc = mesh.ccw_corner(c0);
    if (!cc || processed(mesh.corner_face(cc))) c0 = mesh.clw_face_corner(c0);
  }
  // The second face to visit will be ccw turn; fintnei == mesh.ccw_corner(c0)
  return c0;
}

// *** Greedy stripify

void do_greedy_stripify() {
  HH_TIMER("_greedy_stripify");
  old_strip_order = true;  // never got around to fixing this.
  ar_verts.init(0);
  ar_faces.init(0);
  MeshStatus ms;
  Array<Vertex> va;
  Array<Face> fa;
  for (;;) {
    Face f = ms.find_initial_face();
    if (!f) break;
    face_vertices_neighbors(f, va, fa);
    int i;
    for (i = 0; i < 3; i++) {
      if (fa[i] && !ms.processed(fa[i])) break;
    }
    // Current approach: last two vertices output for triangle are the
    //  ones that get reused in the next triangle if the strip continues.
    // Alternative approach would have been to always set first two
    //  vertices to be the ones reused from previous triangle.
    Vertex vo = va[mod3(i + 0)];  // Note: i may equal 3!
    Vertex v1 = va[mod3(i + 1)];
    Vertex v2 = va[mod3(i + 2)];
    int len = 0;
    for (;;) {
      // Have f={vo, v1, v2} and next face opposite vo == va[i], unless i == 3.
      ASSERTX(i == 3 || va[i] == vo);
      ms.process(f);
      ar_verts.push_array(V(mesh.vertex_id(vo), mesh.vertex_id(v1), mesh.vertex_id(v2)));
      ar_faces.push(f);
      len++;
      if (i == 3) break;
      f = fa[i];
      ASSERTX(f && !ms.processed(f));
      face_vertices_neighbors(f, va, fa);
      int j;  // location of new vertex on f
      if (va[0] == v2) {
        j = 2;
        ASSERTX(va[1] == v1);
      } else if (va[0] == v1) {
        j = 1;
        ASSERTX(va[2] == v2);
      } else {
        j = 0;
        ASSERTX(va[1] == v2 && va[2] == v1);
      }
      if (0) {
      } else if (!favor_spiral && Random::G.get_unsigned(2) && fa[mod3(j + 2)] &&
                 !ms.processed(fa[mod3(j + 2)])) {  // ccw
        vo = v1;
        v1 = va[j];
        i = mod3(j + 2);
      } else if (fa[mod3(j + 1)] && !ms.processed(fa[mod3(j + 1)])) {  // clw
        vo = v2;
        v2 = va[j];
        i = mod3(j + 1);
      } else if (fa[mod3(j + 2)] && !ms.processed(fa[mod3(j + 2)])) {  // ccw
        vo = v1;
        v1 = va[j];
        i = mod3(j + 2);
      } else {
        if (0) {  // old way: output new vert first
          std::swap(v1, v2);
          vo = va[j];
          i = 3;
        } else {  // new way: reuse {v2, v1} first
          vo = v2;
          v2 = va[j];
          i = 3;
        }
      }
    }
    HH_SSTAT(Sstriplen, len);
  }
}

// *** SGI stripify  (implements the greedy "tomesh.c" algorithm from SGI).

int find_next_face(CArrayView<Face> fa, const MeshStatus& ms) {
  // Assumes that current face has been removed from pqfnnei.
  int ret = 3;
  int min_nnei = std::numeric_limits<int>::max(), min_min_next_nnei;
  dummy_init(min_min_next_nnei);
  for_int(i, 3) {
    if (!fa[i]) continue;
    if (ms.processed(fa[i])) continue;  // already visited
    int nnei = ms.face_nnei(fa[i]);
    assertx(nnei < 3);
    int min_next_nnei = std::numeric_limits<int>::max();
    for (Face ff : mesh.faces(fa[i])) {
      if (ms.processed(ff)) continue;
      int next_nnei = ms.face_nnei(ff);
      if (next_nnei >= 0 && next_nnei < min_next_nnei) min_next_nnei = next_nnei;
    }
    if (min_next_nnei == std::numeric_limits<int>::max()) min_next_nnei = -1;  // the best
    if (nnei < min_nnei) {
      ret = i;
      min_nnei = nnei;
      min_min_next_nnei = min_next_nnei;
    } else if (nnei == min_nnei && min_next_nnei < min_min_next_nnei) {
      ret = i;
      min_min_next_nnei = min_next_nnei;
    }
  }
  return ret;
}

void do_sgi_stripify() {
  HH_TIMER("_sgi_stripify");
  ar_verts.init(0);
  ar_faces.init(0);
  MeshStatus ms;
  Array<Vertex> va;
  Array<Face> fa;
  for (;;) {
    Face f = ms.find_initial_face();
    if (!f) break;
    ms.process(f);
    face_vertices_neighbors(f, va, fa);
    int i = find_next_face(fa, ms);
    Vertex vo = va[mod3(i + 0)];  // Note: i may equal 3!
    Vertex v1 = va[mod3(i + 1)];
    Vertex v2 = va[mod3(i + 2)];
    int len = 0;
    bool last_clw = true;  // so that second turn is expected ccw
    for (;;) {
      // Have f={vo, v1, v2} and next face opposite vo == va[i], unless i == 3.
      ASSERTX(i == 3 || va[i] == vo);
      ar_verts.push_array(V(mesh.vertex_id(vo), mesh.vertex_id(v1), mesh.vertex_id(v2)));
      ar_faces.push(f);
      len++;
      if (i == 3) break;
      f = fa[i];
      ASSERTX(f && !ms.processed(f));
      ms.process(f);
      face_vertices_neighbors(f, va, fa);
      int j;  // location of new vertex on f
      if (va[0] == v2) {
        j = 2;
        ASSERTX(va[1] == v1);
      } else if (va[0] == v1) {
        j = 1;
        ASSERTX(va[2] == v2);
      } else {
        j = 0;
        ASSERTX(va[1] == v2 && va[2] == v1);
      }
      i = find_next_face(fa, ms);
      if (i == j) {
        assertnever("cannot go back to previous face");
      } else if (i == mod3(j + 1)) {  // clw turn
        vo = v2;
        v2 = va[j];
        last_clw = true;
      } else if (i == mod3(j + 2)) {  // ccw turn
        vo = v1;
        v1 = va[j];
        last_clw = false;
      } else {  // i == 3
        ASSERTX(i == 3);
        if (last_clw) {  // pretend there would be a ccw turn
          vo = v1;
          v1 = va[j];
        } else {  // pretend there would be a clw turn
          vo = v2;
          v2 = va[j];
        }
      }
    }
    HH_SSTAT(Sstriplen, len);
  }
  // To take care of old_strip_order = 0.
  do_fixup_indices();
}

// *** meshify5

// Lookahead simulation, starting at corner oc, over the next maxnf faces.
// Force a strip restart after exactly nfcontinue faces are visited.
// ostripnf is the current number of faces in the strip processed so far.
// oqnextc is the queue of face locations for strip restarts.
// ovcache is the global vertex cache.
// vcache is a temporary vertex cache that gets cloned from ovcache.
// Returns the goodness of the simulation in pvalue.
void simulate5(int nfcontinue, MeshStatus& ms, Corner oc, int ostripnf, const Queue<Corner>& oqnextc,
               const VertexCache& ovcache, VertexCache& vcache, int maxnf, float& pvalue) {
  assertx(nfcontinue < maxnf);
  ms.sim_init();
  int stripnf = ostripnf;
  Queue<Corner> qnextc;
  qnextc = oqnextc;  // qnextc.copy(oqnextc);
  vcache.copy(ovcache);
  Corner c = oc;
  Face f = mesh.corner_face(c);
  int nfvis = 0;  // number of faces visited
  int ngood = 0;  // # vertex references which hit vcache
  ASSERTX(!ms.sim_face_visited(f));
  int nfevent = nfcontinue;  // next face event
  for (;;) {
    if (nfvis == nfevent) {
      if (nfvis == maxnf) break;
      nfevent = nfvis < nfcontinue ? nfcontinue : maxnf;
      for (;;) {
        if (qnextc.empty()) {
          c = nullptr;
          break;
        }
        c = qnextc.dequeue();
        f = mesh.corner_face(c);
        if (!ms.sim_face_visited(f)) break;
      }
      if (!c) break;
      qnextc.clear();
      stripnf = 0;
    }
    nfvis++;
    ms.sim_visit_face(f);
    {
      Corner cc = c;
      if (!stripnf++) cc = mesh.ccw_face_corner(cc);
      for_int(j, 3) {
        int vid = mesh.vertex_id(mesh.corner_vertex(cc));
        // bool hit = reinterpret_cast<FifoVertexCache&>(vcache).FifoVertexCache::access_hits(vid);
        bool hit = vcache.access_hits(vid);
        ngood += hit;
        // if (verb>=4) std::cerr << sform("%c", !hit?'*':'.');
        cc = mesh.ccw_face_corner(cc);
      }
    }
    // if (verb>=4) std::cerr << " ";
    // Corner cint = mesh.ccw_corner(c);
    // Corner cext = mesh.ccw_corner(mesh.clw_face_corner(c));
    // bool fintnei = cint && !ms.sim_face_visited(mesh.corner_face(cint));
    // bool fextnei = cext && !ms.sim_face_visited(mesh.corner_face(cext));
    Corner cint = mesh.ccw_corner(c), cext;
    Face f2;
    if (cint && (f2 = mesh.corner_face(cint), !ms.sim_face_visited(f2))) {
      cext = mesh.ccw_corner(mesh.clw_face_corner(c));
      if (cext && !ms.sim_face_visited(mesh.corner_face(cext))) qnextc.enqueue(cext);
      c = cint;
      f = f2;
    } else if ((cext = mesh.ccw_corner(mesh.clw_face_corner(c))) != nullptr &&
               (f2 = mesh.corner_face(cext), !ms.sim_face_visited(f2))) {
      c = cext;
      f = f2;
    } else {  // strip cannot continue
      nfevent = nfvis;
    }
  }
  switch (cache_type) {
    case VertexCache::EType::fifo: pvalue = !nfvis ? 0.f : float(ngood) / nfvis; break;
    case VertexCache::EType::lru: pvalue = !nfvis ? 0.f : float(ngood) / nfvis; break;
    default: assertnever("");
  }
  if (verb >= 4) std::cerr << sform("nf%d v%.2f\n", nfvis, pvalue);
}

void do_meshify5() {
  Timer timer("_meshify5");
  ConsoleProgress cprogress;
  old_strip_order = false;
  ar_verts.init(0);
  ar_faces.init(0);
  MeshStatus ms;
  auto up_vcache = VertexCache::make(cache_type, 1 + mesh.num_vertices(), cache_size);
  VertexCache& vcache = *up_vcache;
  auto up_tvcache = VertexCache::make(cache_type, 1 + mesh.num_vertices(), cache_size);
  VertexCache& tvcache = *up_tvcache;
  Queue<Corner> qnextc;
  Corner c = nullptr;
  int stripnf = 0;
  string sstatus;
  const int initial_best_nfcont = (cache_type == VertexCache::EType::fifo  ? cache_size / 2
                                   : cache_type == VertexCache::EType::lru ? cache_size / 2
                                                                           : (assertnever(""), 0));
  int best_nfcont;  // this is the variable i_min in the paper
  dummy_init(best_nfcont);
  for (;;) {
    if (!c) {
      // Restart a strip.
      if (stripnf) {
        if (verb >= 2) showdf(" [%s]\n", sstatus.c_str());
        HH_SSTAT(Sstripnf, stripnf);
      }
      stripnf = 0;
      sstatus = "";
      best_nfcont = initial_best_nfcont;
      if (verb >= 3) SHOW("*Restart\n");
      for (;;) {
        if (qnextc.empty()) {
          c = nullptr;
          break;
        }
        c = qnextc.dequeue();
        if (!ms.processed(mesh.corner_face(c))) break;
      }
      qnextc.clear();
      if (!c) {
        HH_SSTAT(Sscratch, 1);
        if (verb >= 2) showdf("**From scratch\n");
        c = ms.find_initial_corner(vcache);
        if (!c) break;
      }
    }
    ASSERTX(!ms.processed(mesh.corner_face(c)));
    while (!qnextc.empty() &&
           (ms.processed(mesh.corner_face(qnextc.front())) || mesh.corner_face(qnextc.front()) == mesh.corner_face(c)))
      qnextc.dequeue();
    // Decide to either add this face or restart.
    const int maxnf = (simulate_nf                              ? simulate_nf
                       : cache_type == VertexCache::EType::fifo ? cache_size + 5
                       : cache_type == VertexCache::EType::lru  ? cache_size / 2 + 5
                                                                : (assertnever(""), 0));
    int nsim = 0;
    bool brestart;
    float max_value;
    for (;;) {
      // not needed: if (qnextc.empty()) { brestart = false; break; }
      float value0;
      simulate5(0, ms, c, stripnf, qnextc, vcache, tvcache, maxnf, value0);
      nsim++;
      // empirically, does not seem beneficial:
      if (0 && best_nfcont > 1) --best_nfcont;
      if (best_nfcont >= maxnf) best_nfcont = maxnf - 1;
      simulate5(best_nfcont, ms, c, stripnf, qnextc, vcache, tvcache, maxnf, max_value);
      nsim++;
      if (max_value >= value0) {
        brestart = false;
        break;
      }
      int old_best_nfcont = best_nfcont;
      for_intL(nfcont, 1, maxnf) {
        if (nfcont == old_best_nfcont) continue;
        float value;
        simulate5(nfcont, ms, c, stripnf, qnextc, vcache, tvcache, maxnf, value);
        nsim++;
        if (value >= max_value) {
          max_value = value;
          best_nfcont = nfcont;
          if (max_value >= value0) break;
        }
      }
      brestart = !(max_value >= value0);
      break;
    }
    HH_SSTAT(Snsim, nsim);
    if (verb >= 3) showf("best_nfcont=%d max_value=%.2f\n", best_nfcont, max_value);
    if (brestart) {
      c = nullptr;  // force restart
      continue;
    }
    ms.process(mesh.corner_face(c));
    int nfmiss;
    {
      Corner cc = c;
      if (0 && !stripnf) cc = mesh.ccw_face_corner(cc);
      nfmiss = record_face(cc, vcache);
    }
    stripnf++;
    if (verb == 1) cprogress.update(float(ar_faces.num()) / mesh.num_faces());
    if (verb >= 2) sstatus += sform("%d", nfmiss);
    Corner cint = mesh.ccw_corner(c);
    Corner cext = mesh.ccw_corner(mesh.clw_face_corner(c));
    bool fintnei = cint && !ms.processed(mesh.corner_face(cint));
    bool fextnei = cext && !ms.processed(mesh.corner_face(cext));
    if (fintnei) {
      if (fextnei) qnextc.enqueue(cext);
      c = cint;
    } else if (fextnei) {
      c = cext;
    } else {
      c = nullptr;  // strip cannot continue;
    }
  }
  cprogress.clear();
  show_rate(timer);
}

// *** meshify8

void do_meshify8() {
  ConsoleProgress cprogress;
  old_strip_order = false;
  ar_verts.init(0);
  ar_faces.init(0);
  MeshStatus ms;
  auto up_vcache = VertexCache::make(cache_type, 1 + mesh.num_vertices(), cache_size);
  VertexCache& vcache = *up_vcache;
  Corner cnext;
  int locnext;
  Corner c;
  const int desiredloc = simulate_nf ? simulate_nf : cache_size - 9;
  Timer timer("_meshify8");
  for (;;) {
  GOTO_STRIP_RESTART_FROM_SCRATCH:
    c = ms.find_initial_corner(vcache);
    if (!c) break;
    for (;;) {    // form one strip
      for (;;) {  // form ring(s) while having no cnext
        ms.process(mesh.corner_face(c));
        record_face(c, vcache);
        Corner cint = mesh.ccw_corner(c);
        Corner cext = mesh.ccw_corner(mesh.clw_face_corner(c));
        bool fintnei = cint && !ms.processed(mesh.corner_face(cint));
        bool fextnei = cext && !ms.processed(mesh.corner_face(cext));
        if (fintnei) {
          c = cint;
          if (fextnei) {
            cnext = cext;
            locnext = 0;
            break;  // finish current ring in next loop
          }
        } else if (fextnei) {
          c = cext;
        } else {
          goto GOTO_STRIP_RESTART_FROM_SCRATCH;  // "continue" on outermost loop
        }
      }
      for (;;) {  // form ring(s) while having cnext
        if (verb == 1) cprogress.update(float(ar_faces.num()) / mesh.num_faces());
        ms.process(mesh.corner_face(c));
        int nmiss = record_face(c, vcache);
        locnext += nmiss;
        Corner cint = mesh.ccw_corner(c);
        bool fintnei = cint && !ms.processed(mesh.corner_face(cint));
        if (fintnei) {
          c = cint;
          continue;
        }
        Corner cext = mesh.ccw_corner(mesh.clw_face_corner(c));
        bool fextnei = cext && !ms.processed(mesh.corner_face(cext));
        if (!fextnei) break;
        c = cext;
        // Decide to add next ring (e.g. 2 faces) or restart.
        int nf = 1;
        for (Corner cc = c;;) {
          Corner cn = mesh.ccw_corner(cc);
          if (!cn || ms.processed(mesh.corner_face(cn))) break;
          nf++;
          cc = cn;
        }
        if (locnext + (nf - 1) > desiredloc) break;
      }
      c = cnext;
      if (ms.processed(mesh.corner_face(c))) break;
    }
  }
  cprogress.clear();
  show_rate(timer);
}

// *** meshify9

// Because of bad performance on ribbon, keep queue of restart locations.

void do_meshify9() {
  ConsoleProgress cprogress;
  old_strip_order = false;
  ar_verts.init(0);
  ar_faces.init(0);
  MeshStatus ms;
  auto up_vcache = VertexCache::make(cache_type, 1 + mesh.num_vertices(), cache_size);
  VertexCache& vcache = *up_vcache;
  Corner cnext = nullptr;
  Queue<Corner> cnexta;  // second resort for cnext
  int locnext = 0;       // dont_care (only care if cnext != 0)
  Corner c;
  const int desiredloc = simulate_nf ? simulate_nf : cache_size - 9;
  Timer timer("_meshify9");
  for (;;) {
  GOTO_STRIP_RESTART_FROM_SCRATCH:
    ASSERTX(!cnext);
    c = ms.find_initial_corner(vcache);
    if (!c) break;
    for (;;) {
      ASSERTX(!ms.processed(mesh.corner_face(c)));
      // Decide to either add next RING OF FACES (e.g. 2) or restart.
      if (cnext) {
        int nf = 1;
        for (Corner cc = c;;) {
          Corner cn = mesh.ccw_corner(cc);
          if (!cn || ms.processed(mesh.corner_face(cn))) break;
          nf++;
          cc = cn;
        }
        if (locnext + (nf - 1) > desiredloc) {  // restart desired
          for (;;) {
            if (!ms.processed(mesh.corner_face(cnext))) {
              c = cnext;
              cnext = nullptr;
              break;  // restart now!
            }
            cnext = nullptr;
            if (cnexta.empty()) break;  // keep growing strip
            cnext = cnexta.dequeue();
          }
        }
      }
      for (;;) {
        ms.process(mesh.corner_face(c));
        int nmiss = record_face(c, vcache);
        locnext += nmiss;
        Corner cint = mesh.ccw_corner(c);
        Corner cext = mesh.ccw_corner(mesh.clw_face_corner(c));
        bool fintnei = cint && !ms.processed(mesh.corner_face(cint));
        bool fextnei = cext && !ms.processed(mesh.corner_face(cext));
        if (fintnei) {
          if (fextnei) {
            if (!cnext) {
              cnext = cext;
              locnext = 0;
              cnexta.clear();
            } else {
              cnexta.enqueue(cext);
            }
          }
          c = cint;
        } else if (fextnei) {
          c = cext;
          break;
        } else {  // restart forced
          c = cnext;
          cnext = nullptr;
          for (;;) {
            if (c && !ms.processed(mesh.corner_face(c))) break;        // restart now!
            if (cnexta.empty()) goto GOTO_STRIP_RESTART_FROM_SCRATCH;  // "continue" on outermost loop
            c = cnexta.dequeue();
          }
        }
      }
      if (verb == 1) cprogress.update(float(ar_faces.num()) / mesh.num_faces());
    }
  }
  cprogress.clear();
  show_rate(timer);
}

// *** meshify10

// When restarting at cnext, first go clw.

void try_starting_earlier(const MeshStatus& ms, Corner& c) {
  {
    Corner cn = mesh.ccw_corner(c);
    if (!cn || ms.processed(mesh.corner_face(cn))) return;
  }
  c = mesh.ccw_face_corner(c);
  for (;;) {
    Corner cn = mesh.clw_corner(c);
    if (!cn || ms.processed(mesh.corner_face(cn))) break;
    c = cn;
  }
}

void do_meshify10() {
  ConsoleProgress cprogress;
  old_strip_order = false;
  ar_verts.init(0);
  ar_faces.init(0);
  MeshStatus ms;
  auto up_vcache = VertexCache::make(cache_type, 1 + mesh.num_vertices(), cache_size);
  VertexCache& vcache = *up_vcache;
  Corner cnext = nullptr;
  Queue<Corner> cnexta;  // second resort for cnext
  int locnext = 0;       // dont_care (only care if cnext != 0)
  Corner c;
  const int desiredloc = simulate_nf ? simulate_nf : cache_size - 9;
  Timer timer("_meshify10");
  const bool only_first_ring = getenv_bool("ONLY_FIRST_RING");
  for (;;) {
  GOTO_STRIP_RESTART_FROM_SCRATCH:
    ASSERTX(!cnext);
    c = ms.find_initial_corner(vcache);
    if (!c) break;
    for (;;) {
      ASSERTX(!ms.processed(mesh.corner_face(c)));
      // Decide to either add next RING OF FACES (e.g. 2) or restart.
      if (cnext) {
        int nf = 1;
        for (Corner cc = c;;) {
          Corner cn = mesh.ccw_corner(cc);
          if (!cn || ms.processed(mesh.corner_face(cn))) break;
          nf++;
          cc = cn;
        }
        if (locnext + (nf - 1) > desiredloc) {  // restart desired
          for (;;) {
            if (!ms.processed(mesh.corner_face(cnext))) {
              c = cnext;
              cnext = nullptr;
              try_starting_earlier(ms, c);
              break;  // restart now!
            }
            cnext = nullptr;
            if (cnexta.empty()) break;  // keep growing strip
            cnext = cnexta.dequeue();
          }
        }
      }
      for (;;) {
        ms.process(mesh.corner_face(c));
        int nmiss = record_face(c, vcache);
        locnext += nmiss;
        Corner cint = mesh.ccw_corner(c);
        Corner cext = mesh.ccw_corner(mesh.clw_face_corner(c));
        bool fintnei = cint && !ms.processed(mesh.corner_face(cint));
        bool fextnei = cext && !ms.processed(mesh.corner_face(cext));
        if (fintnei) {
          if (fextnei) {
            if (!cnext) {
              cnext = cext;
              locnext = 0;
              cnexta.clear();
            } else {
              if (1) cnexta.enqueue(cext);
            }
          }
          c = cint;
        } else if (fextnei) {
          c = cext;
          break;
        } else {  // restart forced
          c = cnext;
          cnext = nullptr;
          for (;;) {
            if (c && !ms.processed(mesh.corner_face(c))) {
              try_starting_earlier(ms, c);
              break;  // restart now!
            }
            if (cnexta.empty()) goto GOTO_STRIP_RESTART_FROM_SCRATCH;
            c = cnexta.dequeue();
          }
        }
      }
      if (verb == 1) cprogress.update(float(ar_faces.num()) / mesh.num_faces());
      if (only_first_ring) break;
    }
  }
  cprogress.clear();
  show_rate(timer);
}

// *** misc

void do_timingtest(Args& args) {
  int niter = args.get_int();
  for_int(iter, niter) do_meshify8();
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSC("A mesh is read from stdin or first arg.  Subsequent options are:");
  HH_ARGSD(fifo, ": set cache type to FIFO");
  HH_ARGSD(lru, ": set cache type to LRU");
  HH_ARGSP(cache_size, "n : set number of cache entries");
  HH_ARGSP(favor_spiral, "bool : favor spiraling strips?");
  HH_ARGSP(verb, "i : verbosity level (0=none)");
  HH_ARGSP(color_corners, "bool : show cache misses in mesh");
  HH_ARGSP(color_fmiss, "bool : show cache misses per face");
  HH_ARGSP(color_forder, "bool : show order of faces");
  HH_ARGSP(old_strip_order, "bool : last 2 verts reused in next face");
  HH_ARGSF(nooutput, ": do not write final mesh");
  HH_ARGSC(HH_ARGS_INDENT "Miscellaneous functions:");
  HH_ARGSD(analyze, ": analyze mesh bandwidth for current cache");
  HH_ARGSD(fanalyze, ": same, with different cache sizes");
  HH_ARGSD(strip_analyze, ": analysis assuming OGL sequential strips");
  HH_ARGSD(info, ": just analyze current vertex cache");
  HH_ARGSD(randomize_faces, ": reorder faces randomly");
  HH_ARGSD(fixup_indices, ": optimally reorder vindices within faces");
  HH_ARGSD(diff_corners, "mesh2.m : color changing corners blue");
  HH_ARGSF(reorder_vertices, ": reorder vertices according to first use");
  HH_ARGSF(duplicate_vertices, ": dup vertices upon cache misses");
  HH_ARGSC(HH_ARGS_INDENT "Reordering algorithms:");
  HH_ARGSD(greedy_stripify, ": generalized triangle strips");
  HH_ARGSD(sgi_stripify, ": better greedy heuristic");
  // HH_ARGSD(meshify1,       ": advancing front (simulate, heuristic)");
  // HH_ARGSD(meshify2,       ": advancing front (simulate, 1dof restart)");
  // HH_ARGSD(meshify4,       ": like 2, but better nverts sim");
  HH_ARGSD(meshify5, ": like 2, cleaner (~results in 1999 paper)");
  // HH_ARGSD(meshify6,       ": fast heuristic");
  // HH_ARGSD(meshify7,       ": obsolete");
  HH_ARGSD(meshify8, ": fast heuristic per ring");
  HH_ARGSD(meshify9, ": like 8, queue of restarts");
  HH_ARGSD(meshify10, ": like 9, go clockwise after restart (simpler, faster, and even better)");
  HH_ARGSD(timingtest, "niter : run timing test");
  {
    HH_TIMER("MeshReorder");
    string arg0 = args.num() ? args.peek_string() : "";
    if (!ParseArgs::special_arg(arg0)) {
      string filename = "-";
      if (args.num() && (arg0 == "-" || arg0[0] != '-')) filename = args.get_filename();
      gfilename = filename;
      RFile fi(filename);
      HH_TIMER("_readmesh");
      for (string line; fi().peek() == '#';) {
        assertx(my_getline(fi(), line));
        if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
      }
      mesh = GMesh(fi());
      showff("%s", args.header().c_str());
      fixup_mesh();
      extract_mesh();
    }
    args.parse();
  }
  hh_clean_up();
  if (!nooutput) {
    replace_mesh();
    mesh.write(std::cout);
  }
  return 0;
}
