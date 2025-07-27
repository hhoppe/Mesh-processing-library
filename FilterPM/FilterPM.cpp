// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/A3dStream.h"
#include "libHh/Args.h"
#include "libHh/Bbox.h"
#include "libHh/Encoding.h"  // Encoding, DeltaEncoding
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/Graph.h"   // do_reorder_vspl()
#include "libHh/MeshOp.h"  // mesh_genus_string()
#include "libHh/PMesh.h"
#include "libHh/Polygon.h"
#include "libHh/STree.h"  // do_reorder_vspl()
#include "libHh/Stat.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
#include "libHh/VertexCache.h"

#define DEF_SR

#if defined(DEF_SR)
#include "libHh/SrMesh.h"
#endif

using namespace hh;

namespace {

const bool sdebug = getenv_bool("FILTERPM_DEBUG");

bool nooutput = false;
int verb = 1;
bool gzip = false;
string gfilename;

PMesh pmesh;
unique_ptr<PMeshRStream> pmrs;
unique_ptr<PMeshIter> pmi;

RFile* pfi = nullptr;
SrMesh srmesh;

void ensure_pm_loaded() {
  int n = 0;
  for (;;) {
    if (!pmrs->next_vsplit()) break;
    n++;
  }
  for_int(i, n) assertx(pmrs->prev_vsplit());
}

void do_info() {
  showdf("Mesh vspli=%d nv=%d nw=%d nf=%d\n",  //
         pmi->_vertices.num() - pmesh._base_mesh._vertices.num(), pmi->_vertices.num(), pmi->_wedges.num(),
         pmi->_faces.num());
}

void do_minfo() {
  do_info();
  ensure_pm_loaded();
  int vspli = pmi->_vertices.num() - pmesh._base_mesh._vertices.num();
  float maxresidd = 0.f;
  for (; vspli < pmesh._vsplits.num(); vspli++) {
    const Vsplit& vspl = pmesh._vsplits[vspli];
    maxresidd = max(maxresidd, vspl.resid_dir);
  }
  showdf(" maxresidd=%g\n", maxresidd);
}

void do_nvertices(Args& args) {
  int nvertices = args.get_int();
  HH_TIMER("_goto");
  if (sdebug) pmi->ok();
  pmi->goto_nvertices(nvertices);
  if (sdebug) pmi->ok();
  do_info();
}

void do_nfaces(Args& args) {
  int nfaces = args.get_int();
  HH_TIMER("_goto");
  if (sdebug) pmi->ok();
  pmi->goto_nfaces(nfaces);
  if (sdebug) pmi->ok();
  do_info();
}

int count_nedges(const AWMesh& mesh) {
  int nedges_t2 = 0;  // Twice the number of edges.
  for_int(i, mesh._faces.num()) for_int(j, 3) nedges_t2 += mesh._fnei[i].faces[j] >= 0 ? 1 : 2;
  assertx(nedges_t2 % 2 == 0);
  return nedges_t2 / 2;
}

void do_nedges(Args& args) {
  int nedges = args.get_int();
  HH_TIMER("_goto");
  for (;;) {
    int cnedges = count_nedges(*pmi);
    if (cnedges >= nedges) break;
    // Conservative upper bound on how many vsplits we can safely advance without exceeding requested number of edges.
    int nextra = (nedges - cnedges) / 3 - 1;
    if (!pmi->next()) break;
    for_int(i, nextra) {
      if (!pmi->next()) break;
    }
  }
  showdf("Found mesh with %d edges\n", count_nedges(*pmi));
  do_info();
}

void do_nsplits(Args& args) {
  int nsplits = args.get_int();
  HH_TIMER("_goto");
  pmi->goto_nvertices(pmesh._base_mesh._vertices.num() + nsplits);
  do_info();
}

void do_maxresidd(Args& args) {
  float residd = args.get_float();
  ensure_pm_loaded();
  int vspli;
  for (vspli = pmesh._vsplits.num() - 1; vspli >= 0; --vspli) {
    const Vsplit& vspl = pmesh._vsplits[vspli];
    if (vspl.resid_dir > residd) break;
  }
  assertx(vspli >= -1);
  // Vsplit at vspli has residual that is too high, so keep one more vertex.
  pmi->goto_nvertices(pmesh._base_mesh._vertices.num() + vspli + 1);
  do_info();
}

void do_coarsest() {
  pmi->goto_nvertices(0);
  do_info();
}

void do_finest() {
  pmi->goto_nvertices(std::numeric_limits<int>::max());
  do_info();
}

void do_outmesh() {
  HH_TIMER("_write_mesh");
  GMesh gmesh = pmi->extract_gmesh();
  if (nooutput) std::cout << "o 1 1 0\n";
  gmesh.write(std::cout);
  nooutput = true;
}

void do_outsmesh() {
  // I had forgotten that the PMesh::apply_vsplit() code is annoyingly complicated.
  // Note that the two new wedges can be on different vertices even though ii == 2; yuck.
  // cd ~/proj/octaflat; r=fandisk; FilterPM data/$r.octaflat.pm -nf 500 -truncate_beyond -outsmesh >$r.m
  // ...
  // vspli=1 isr=1 num_new_wedges=2 wlclw=7 wn1=9 wn2=10
  // wvsfl=7 wvtfl=9 wvlfl=8
  // wvsfr=10 wvrfr=2 wvtfr=4
  // assertion warning: assertw(wvtfr == wn2) in line 201 of file c:/hh/git/mesh_processing/FilterPM/FilterPM.cpp

  HH_TIMER("_write_smesh");
  ensure_pm_loaded();
  pmi->goto_nvertices(0);
  const int k_no_parent = -1;
  Array<int> parents(pmi->_wedges.num(), k_no_parent);

  const auto swap_wedges = [&](const Vec2<int>& wedges, int f) {
    assertx(pmi->_wedges[wedges[0]].vertex != pmi->_wedges[wedges[1]].vertex);
    Set<int> faces;
    for_int(i, 2) for (const int f2 : pmi->ccw_faces(pmi->_wedges[wedges[i]].vertex, f)) faces.add(f2);
    for (const int f2 : faces) {
      for_int(j, 3) {
        int& w = pmi->_faces[f2].wedges[j];
        if (w == wedges[0])
          w = wedges[1];
        else if (w == wedges[1])
          w = wedges[0];
      }
    }
    std::swap(pmi->_wedges[wedges[0]], pmi->_wedges[wedges[1]]);
  };

  for_int(vspli, pmesh._info._tot_nvsplits) {
    const Vsplit& vspl = pmesh._vsplits[vspli];
    const int ii = (vspl.code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
    assertx(ii == 2);  // Assume PM is created using "MeshSimplify -minii2".
    const int isr = vspl.adds_two_faces();
    const int fl = pmi->_faces.num(), fr = isr ? fl + 1 : -1;
    const int wn0 = pmi->_wedges.num();

    pmi->next();
    const int num_new_wedges = pmi->_wedges.num() - wn0;
    if (0) SHOW(isr, wn0, num_new_wedges);
    assertx(num_new_wedges == 1 || (isr && num_new_wedges == 2));
    const bool two_new_wedges = num_new_wedges == 2;

    int wvsfl = pmi->_faces[fl].wedges[0], wvtfl = pmi->_faces[fl].wedges[1], wvlfl = pmi->_faces[fl].wedges[2];
    int wvsfr = -1, wvrfr = -1, wvtfr = -1;
    if (isr) wvsfr = pmi->_faces[fr].wedges[0], wvrfr = pmi->_faces[fr].wedges[1], wvtfr = pmi->_faces[fr].wedges[2];
    dummy_use(wvlfl, wvrfr);
    if (0) SHOW(vspli, isr, wn0, num_new_wedges, fl, fr), SHOW(wvsfl, wvtfl, wvlfl), SHOW(wvsfr, wvrfr, wvtfr);
    if (wvsfl >= wn0) {  // A new wedge was added on the wrong vertex (vs); we will need to swap it with wvtfl.
      assertx(wvtfl < wn0);
      if (two_new_wedges) {
        if (wvsfr == wvsfl)
          wvsfr = wvtfl;
        else if (wvtfr == wvtfl)
          wvtfr = wvsfl;
      }
      swap_wedges(V(wvsfl, wvtfl), fl), std::swap(wvsfl, wvtfl);
    }
    if (two_new_wedges && wvsfr >= wn0) {  // Same check on fr.  We will need to swap the new wedge with wvtfr.
      assertx(wvsfr != wvsfl && wvsfr != wvtfl);
      assertx(wvtfr < wn0);
      swap_wedges(V(wvsfr, wvtfr), fr), std::swap(wvsfr, wvtfr);
    }
    if (0) pmi->ok();
    assertx(wvtfl >= wn0);
    if (two_new_wedges) assertx(wvtfr >= wn0);
    if (two_new_wedges && wvtfl > wvtfr) {      // We need to match the order of wedges pushed onto the parents array.
      if (0) swap_wedges(V(wvsfl, wvsfr), fl);  // Unnecessary.
      std::swap(wvsfl, wvsfr);
    }
    parents.push(wvsfl);
    if (two_new_wedges) parents.push(wvsfr);
  }

  const SMesh smesh(*pmi);  // Split the mesh wedges into vertices.
  GMesh gmesh = smesh.extract_gmesh(pmi->rstream()._info._has_rgb, pmi->rstream()._info._has_uv);
  string str;
  for_int(v, parents.num()) {
    const int parent = parents[v];
    assertx(parent < v);
    gmesh.update_string(gmesh.id_vertex(v + 1), "parent", csform(str, "%d", parent + 1));
  }
  gmesh.write(std::cout);
  nooutput = true;
}

void do_geom_nfaces(Args& args) {
  int nfaces = args.get_int();
  Geomorph geomorph;
  {
    HH_TIMER("_geomorph");
    geomorph.construct_goto_nfaces(*pmi, nfaces);
  }
  do_info();
  if (nooutput) std::cout << "o 1 1 0\n";
  {
    Timer timer("_sgeomorph");
    SGeomorph sgeomorph(geomorph);
    timer.terminate();
    HH_TIMER("_write");
    GMesh gmesh = sgeomorph.extract_gmesh(pmi->rstream()._info._has_rgb, pmi->rstream()._info._has_uv);
    gmesh.write(std::cout);
  }
  nooutput = true;
}

// *** Selective Refinement

void read_srmesh() {
  if (pmi) {
    HH_TIMER("__sr_read");
    srmesh.read_pm(*pmrs);
  } else {
    HH_TIMER("__srm_read");
    srmesh.read_srm((*assertx(pfi))());
  }
  if (sdebug) srmesh.ok();
}

void do_srout(Args& args) {
  Vec<SrViewParams, 1> views;
  for_int(i, views.num()) {
    std::istringstream iss(args.get_string());
    const ObjectFrame object_frame = FrameIO::read(iss).value();
    views[i].set_frame(object_frame.frame);
    views[i].set_zooms(twice(object_frame.zoom));
    views[i].set_screen_thresh(args.get_float());
    views[i].set_hither(0.f);
    // Note: hither and yonder may be different in G3dOGL.
  }
  HH_TIMER("_srout");
  read_srmesh();
  {
    HH_TIMER("__adapt");
    srmesh.set_view_params(views[0]);
    srmesh.adapt_refinement();
  }
  {
    HH_TIMER("__sr_write");
    GMesh gmesh = srmesh.extract_gmesh();
    showdf("%s\n", mesh_genus_string(gmesh).c_str());
    gmesh.write(std::cout);
  }
  nooutput = true;
}

void do_srgeomorph(Args& args) {
  Vec2<SrViewParams> views;
  for_int(i, views.num()) {
    std::istringstream iss(args.get_string());
    const ObjectFrame object_frame = FrameIO::read(iss).value();
    views[i].set_frame(object_frame.frame);
    views[i].set_zooms(twice(object_frame.zoom));
    views[i].set_screen_thresh(args.get_float());
    views[i].set_hither(0.f);
    // Note: hither and yonder may be different in G3dOGL.
  }
  HH_TIMER("_srgeomorph");
  read_srmesh();
  {
    HH_TIMER("__adapt1");
    srmesh.set_view_params(views[0]);
    srmesh.adapt_refinement();
  }
  SrGeomorphInfo geoinfo;
  {
    HH_TIMER("__construct_geomorph");
    srmesh.set_view_params(views[1]);
    srmesh.construct_geomorph(geoinfo);
  }
  {
    HH_TIMER("__sr_write");
    GMesh gmesh = srmesh.extract_gmesh(geoinfo);
    gmesh.write(std::cout);
  }
  nooutput = true;
}

int srfly_grtime = 0;
int srfly_gctime = 0;

void do_srfgeo(Args& args) {
  srfly_grtime = args.get_int();
  srfly_gctime = args.get_int();
}

void do_srfly(Args& args) {
  RFile fiframes(args.get_filename());
  read_srmesh();
  HH_TIMER("_srfly");
  float screen_thresh = args.get_float();
  srmesh.set_refine_morph_time(srfly_grtime);
  srmesh.set_coarsen_morph_time(srfly_gctime);
  for (;;) {
    const auto object_frame = FrameIO::read(fiframes());
    if (!object_frame) break;
    SrViewParams view;
    view.set_frame(object_frame->frame);
    view.set_zooms(twice(object_frame->zoom));
    view.set_screen_thresh(screen_thresh);
    view.set_hither(0.f);
    // Note: hither and yonder may be different in G3dOGL.
    srmesh.set_view_params(view);
    srmesh.adapt_refinement();
    HH_SSTAT(Sflynfaces, srmesh.num_active_faces());
  }
  nooutput = true;
}

void do_tosrm() {
  HH_TIMER("_tosrm");
  {
    PMeshRStream lpmrs((*assertx(pfi))(), nullptr);
    HH_TIMER("__sr_read");
    srmesh.read_pm(lpmrs);
  }
  {
    HH_TIMER("__write_srm");
    srmesh.write_srm(std::cout);
  }
  nooutput = true;
}

// *** Modify PM.

void do_truncate_beyond() {
  if (1) {
    // Have to do it since ensure_pm_loaded() is called at end of main.
    ensure_pm_loaded();
  }
  pmesh.truncate_beyond(*pmi);
}

void do_truncate_prior() {
  if (1) {
    // Have to do it since ensure_pm_loaded() is called at end of main.
    ensure_pm_loaded();
  }
  pmesh.truncate_prior(*pmi);
}

constexpr int k_point_quantization = 16;
constexpr int k_normal_quantization = 8;
constexpr int k_uv_quantization = 16;
constexpr float k_point_qf = float(1u << k_point_quantization);
constexpr float k_normal_qf = float(1u << k_normal_quantization);
constexpr float k_uv_qf = float(1u << k_uv_quantization);

void quantize_mesh(WMesh& mesh, const PMeshInfo& pminfo) {
  const float obj_diameter = pmesh._info._full_bbox.max_side();
  for_int(v, mesh._vertices.num()) {
    Point& p = mesh._vertices[v].attrib.point;
    for_int(c, 3) {
      p[c] = (int((p[c] - pmesh._info._full_bbox[0][c]) / obj_diameter * k_point_qf) / k_point_qf * obj_diameter +
              pmesh._info._full_bbox[0][c]);
    }
  }
  assertx(!pminfo._has_rgb);
  for_int(w, mesh._wedges.num()) {
    Vector& n = mesh._wedges[w].attrib.normal;
    for_int(c, 3) n[c] = int(n[c] * k_normal_qf) / k_normal_qf;
    if (pminfo._has_uv) {
      Uv& uv = mesh._wedges[w].attrib.uv;
      for_int(c, 2) uv[c] = int(uv[c] * k_uv_qf) / k_uv_qf;
    }
  }
}

void quantize_mesh_int(WMesh& mesh, const PMeshInfo& pminfo) {
  const float obj_diameter = pmesh._info._full_bbox.max_side();
  for_int(v, mesh._vertices.num()) {
    Point& p = mesh._vertices[v].attrib.point;
    for_int(c, 3) p[c] = floor((p[c] - pmesh._info._full_bbox[0][c]) / obj_diameter * k_point_qf);
  }
  assertx(!pminfo._has_rgb);
  for_int(w, mesh._wedges.num()) {
    Vector& n = mesh._wedges[w].attrib.normal;
    for_int(c, 3) n[c] = floor(n[c] * k_normal_qf);
    if (pminfo._has_uv) {
      Uv& uv = mesh._wedges[w].attrib.uv;
      for_int(c, 2) uv[c] = floor(uv[c] * k_uv_qf);
    }
  }
}

void quantize_vsplit(Vsplit& vspl, const PMeshInfo& pminfo) {
  const float obj_diameter = pmesh._info._full_bbox.max_side();
  {
    Vector& v = vspl.vad_large.dpoint;
    for_int(c, 3) v[c] = int(v[c] / obj_diameter * k_point_qf) / k_point_qf * obj_diameter;
  }
  {
    Vector& v = vspl.vad_small.dpoint;
    for_int(c, 3) v[c] = int(v[c] / obj_diameter * k_point_qf) / k_point_qf * obj_diameter;
  }
  assertx(!pminfo._has_rgb);
  for_int(i, vspl.ar_wad.num()) {
    Vector& n = vspl.ar_wad[i].dnormal;
    for_int(c, 3) n[c] = int(n[c] * k_normal_qf) / k_normal_qf;
    if (pminfo._has_uv) {
      Uv& uv = vspl.ar_wad[i].duv;
      for_int(c, 2) uv[c] = int(uv[c] * k_uv_qf) / k_uv_qf;
    }
  }
}

void quantize_vsplit_int(Vsplit& vspl, const PMeshInfo& pminfo) {
  const float obj_diameter = pmesh._info._full_bbox.max_side();
  {
    Vector& v = vspl.vad_large.dpoint;
    for_int(c, 3) v[c] = floor(v[c] / obj_diameter * k_point_qf);
  }
  {
    Vector& v = vspl.vad_small.dpoint;
    for_int(c, 3) v[c] = floor(v[c] / obj_diameter * k_point_qf);
  }
  assertx(!pminfo._has_rgb);
  for_int(i, vspl.ar_wad.num()) {
    Vector& n = vspl.ar_wad[i].dnormal;
    for_int(c, 3) n[c] = floor(n[c] * k_normal_qf);
    if (pminfo._has_uv) {
      Uv& uv = vspl.ar_wad[i].duv;
      for_int(c, 2) uv[c] = floor(uv[c] * k_uv_qf);
    }
  }
}

void do_quantize() {
  HH_TIMER("_quantize");
  ensure_pm_loaded();
  quantize_mesh(pmesh._base_mesh, pmesh._info);
  for_int(vspli, pmesh._vsplits.num()) quantize_vsplit(pmesh._vsplits[vspli], pmesh._info);
}

// *** Analyze PM

// *** compression

// Rescale a vector to lie in integer space after quantization.
Vector encode_dpoint(const Vector& v) {
  const float obj_diameter = pmesh._info._full_bbox.max_side();
  return v / obj_diameter * k_point_qf;
}

Vector encode_dnormal(const Vector& v) { return v * k_normal_qf; }

Uv encode_uv(const Uv& uv) {
  Uv ruv;
  ruv[0] = uv[0] * k_uv_qf;
  ruv[1] = uv[1] * k_uv_qf;
  return ruv;
}

string print_int(int i) { return sform("%d", i); }

int encode_mesh(const AWMesh& mesh) {
  // Does not include fnei field!
  int nv = mesh._vertices.num();
  int nw = mesh._wedges.num();
  int nf = mesh._faces.num();
  Encoding<int> enc_materials;
  for_int(i, nf) enc_materials.add(mesh._faces[i].attrib.matid, 1.f);
  float matbits = enc_materials.norm_entropy();
  if (verb >= 2) SHOW(matbits);
  return int(nv * (3 * k_point_quantization) +
             nw * (std::log2(float(nv)) + 3 * k_normal_quantization + 2 * k_uv_quantization) +
             nf * (3 * std::log2(float(nw)) + matbits));
}

int wrap_dflclw(int dflclw, int nfaces) {
  if (dflclw < -(nfaces - 1) / 2) dflclw += nfaces;
  if (dflclw > nfaces / 2) dflclw -= nfaces;
  return dflclw;
}

void do_compression() {
  HH_TIMER("_compression");
  assertx(pmesh._info._full_bbox[0][0] != BIGFLOAT);
  assertx(!pmesh._info._has_rgb);
  pmi->goto_nvertices(0);
  int base_nv = pmi->_vertices.num();
  int base_nw = pmi->_wedges.num();
  int base_nf = pmi->_faces.num();
  int bits_basemesh = encode_mesh(*pmi);
  if (verb >= 2) SHOW(bits_basemesh);
  if (verb >= 2) SHOW(bits_basemesh / base_nv);
  ensure_pm_loaded();
  pmi->goto_nvertices(std::numeric_limits<int>::max());  // Goto fully detailed mesh.
  int full_nv = pmesh._info._full_nvertices;
  int full_nw = pmesh._info._full_nwedges;
  int full_nf = pmesh._info._full_nfaces;
  int nvsplits = pmesh._vsplits.num();
  assertx(nvsplits == full_nv - base_nv);
  int bits_fullmesh = encode_mesh(*pmi);
  if (verb >= 2) SHOW(bits_fullmesh);
  if (verb >= 2) SHOW(bits_fullmesh / full_nv);
  float bits_m_mesh = 0.f, bits_g_mesh = 0.f, bits_e_mesh = 0.f;
  float bits_m_pmesh = 0.f, bits_g_pmesh = 0.f, bits_e_pmesh = 0.f;
  {  // Consider memory-resident Mesh.  Does not include fnei field!
    int bits = full_nv * (3 * 32) + full_nw * (1 * 32 + 5 * 32) + full_nf * (3 * 32 + 16);
    showf("Memory Mesh: %d bits (%.1f bits/vertex)\n", bits, float(bits) / full_nv);
    bits_m_mesh = float(bits) / full_nv;
  }
  {  // Consider encoded Mesh.  Does not include fnei field!
    showf("Encoded mesh: %d bits (%.1f bits/vertex)\n", bits_fullmesh, float(bits_fullmesh) / full_nv);
    bits_e_mesh = float(bits_fullmesh) / full_nv;
  }
  {  // Consider memory-resident PMesh.
    int bits = base_nv * (3 * 32) + base_nw * (1 * 32 + 5 * 32) + base_nf * (6 * 32 + 16);
    for_int(vspli, nvsplits) {
      const Vsplit& vspl = pmesh._vsplits[vspli];
      bits += 32 + 16 + 16 + 16 * 2 + 2 * 3 * 32 + vspl.ar_wad.num() * 5 * 32;
    }
    showf("Memory PMesh: %d bits (%.1f bits/vertex)\n", bits, float(bits) / full_nv);
    bits_m_pmesh = float(bits) / full_nv;
  }
  {  // Consider Encoded PMesh.
    int nwads = 0;
    float bits_flclw = 0.f;
    DeltaEncoding de_dflclw;
    float bits_vs_index = 0.f;
    Encoding<int> enc_vlr_offset;
    Encoding<int> enc_corners_ii_matp;
    Encoding<int> enc_flr_matid;
    Encoding<int> enc_just_flrn;
    DeltaEncoding de_pl;  // vad_l, using 3*max_bits(coord).
    DeltaEncoding de_ps;  // vad_s.
    DeltaEncoding de_n;
    DeltaEncoding de_uv;
    int flclwo = 0;
    int cur_nf = base_nf;
    for_int(vspli, nvsplits) {
      const Vsplit& vspl = pmesh._vsplits[vspli];
      bits_flclw += std::log2(float(base_nv + vspli));
      int dflclw = vspl.flclw - flclwo;
      dflclw = wrap_dflclw(dflclw, cur_nf);
      float fdflclw = float(dflclw);
      flclwo = vspl.flclw;
      de_dflclw.enter_vector(V(fdflclw));
      bits_vs_index += std::log2(3.f);
      enc_vlr_offset.add(vspl.vlr_offset1, 1.f);
      int mcode = vspl.code & (Vsplit::II_MASK | Vsplit::S_MASK | Vsplit::T_MASK | Vsplit::L_MASK | Vsplit::R_MASK |
                               Vsplit::FLN_MASK | Vsplit::FRN_MASK);
      enc_corners_ii_matp.add(mcode, 1.f);
      enc_just_flrn.add(vspl.code & (Vsplit::FLN_MASK | Vsplit::FRN_MASK), 1.f);
      if (vspl.code & Vsplit::FLN_MASK) enc_flr_matid.add(vspl.fl_matid, 1.f);
      if (vspl.code & Vsplit::FRN_MASK) enc_flr_matid.add(vspl.fr_matid, 1.f);
      de_pl.enter_vector(encode_dpoint(vspl.vad_large.dpoint));
      de_ps.enter_vector(encode_dpoint(vspl.vad_small.dpoint));
      for_int(j, vspl.ar_wad.num()) {
        nwads++;
        de_n.enter_vector(encode_dnormal(vspl.ar_wad[j].dnormal));
        Uv tuv = encode_uv(vspl.ar_wad[j].duv);
        de_uv.enter_vector(tuv);
      }
      cur_nf += vspl.adds_two_faces() ? 2 : 1;
    }
    showf("Avg |wad| = %.2f\n", float(nwads) / nvsplits);
    showf("bits_flclw: %.1f\n", bits_flclw / nvsplits);
    { de_dflclw.analyze("de_dflclw"); }
    if (verb >= 2) showf("bits_vs_index: %.1f\n", bits_vs_index / nvsplits);
    enc_vlr_offset.print_top_entries("vlr_offset", 5, print_int);
    enc_corners_ii_matp.print_top_entries("corners_ii_matp", verb >= 2 ? 20 : 5, print_int);
    if (verb >= 2) showf("bits_flr_matid: %.1f\n", enc_flr_matid.entropy() / nvsplits);
    if (verb >= 2) enc_just_flrn.print_top_entries("just_flrn", 5, print_int);
    {
      de_pl.analyze("de_pl");
      de_ps.analyze("de_ps");
      int b3 = de_n.analyze("de_n");
      int b4 = de_uv.analyze("de_uv");
      if (verb >= 2) showf("bits de_n/nvsplits: %.1f\n", b3 / float(nvsplits));
      if (verb >= 2) showf("bits de_uv/nvsplits: %.1f\n", b4 / float(nvsplits));
    }
    if (de_dflclw.total_entropy() < bits_flclw) {
      if (0) Warning("Using dflclw for flclw encoding!");
      bits_flclw = de_dflclw.total_entropy();
    }
    int tot_bits = int(bits_flclw + bits_vs_index + enc_vlr_offset.entropy() + enc_corners_ii_matp.entropy() +
                       enc_flr_matid.entropy() + de_pl.total_entropy() + de_ps.total_entropy() + de_n.total_entropy() +
                       de_uv.total_entropy());
    if (verb >= 2) showf("Sum of bit fields per vsplit: %.1f\n", tot_bits / float(nvsplits));
    showf("Encoded PMesh: %d bits (%.1f bits/vertex)\n",  //
          bits_basemesh + tot_bits, float(bits_basemesh + tot_bits) / full_nv);
    bits_e_pmesh = float(bits_basemesh + tot_bits) / full_nv;
    showf("** %.1f & %.1f & %.1f & %.1f & %.1f & %.1f & %.1f & %.1f & %.1f & %.1f\n",  //
          bits_flclw / nvsplits, bits_vs_index / nvsplits, enc_vlr_offset.entropy() / nvsplits,
          enc_corners_ii_matp.entropy() / nvsplits, enc_flr_matid.entropy() / nvsplits,
          de_pl.total_entropy() / nvsplits, de_ps.total_entropy() / nvsplits, de_n.total_entropy() / nvsplits,
          de_uv.total_entropy() / nvsplits, float(tot_bits) / nvsplits);
  }
  if (gzip) {
    {
      WFile fo("| gzip | wc -c >v.FilterPM");
      // Remove materials.
      Materials no_materials;
      pmi->_materials = no_materials;
      quantize_mesh_int(*pmi, pmi->rstream()._info);
      pmi->write(fo(), pmi->rstream()._info);
      // pmi is messed up, so clear it.
      pmi->_vertices.init(0);
      pmi->_wedges.init(0);
      pmi->_faces.init(0);
    }
    int nbytes;
    {
      RFile fi("v.FilterPM");
      assertx(fi() >> nbytes);
    }
    assertx(remove_file("v.FilterPM"));
    bits_g_mesh = nbytes * 8.f / full_nv;
    if (verb >= 2) showf("Gzipped Mesh: %.1f bits/vertex\n", bits_g_mesh);
  }
  if (gzip) {
    {
      WFile fo("| gzip | wc -c >v.FilterPM");
      AWMesh lbasemesh = pmesh._base_mesh;
      // Remove materials.
      Materials no_materials;
      lbasemesh._materials = no_materials;
      lbasemesh.write(fo(), pmi->rstream()._info);
      quantize_mesh_int(lbasemesh, pmi->rstream()._info);
      for_int(vspli, pmesh._vsplits.num()) {
        Vsplit vspl = pmesh._vsplits[vspli];
        quantize_vsplit_int(vspl, pmi->rstream()._info);
        vspl.write(fo(), pmesh._info);
      }
    }
    int nbytes;
    {
      RFile fi("v.FilterPM");
      assertx(fi() >> nbytes);
    }
    assertx(remove_file("v.FilterPM"));
    bits_g_pmesh = nbytes * 8.f / full_nv;
    if (verb >= 2) showf("Gzipped PMesh: %.1f bits/vertex\n", bits_g_pmesh);
  }
  showf("** %.0f & %.0f & %.0f &  %.0f & %.0f & %.0f\n",  //
        bits_m_mesh, bits_g_mesh, bits_e_mesh, bits_m_pmesh, bits_g_pmesh, bits_e_pmesh);
  nooutput = true;
}

void do_gcompression() {
  HH_TIMER("_gcompression");
  assertx(pmesh._info._full_bbox[0][0] != BIGFLOAT);
  pmi->goto_nvertices(0);
  ensure_pm_loaded();
  DeltaEncoding de_p;     // vad_l, before pred., using 3*max_bits(coord).
  DeltaEncoding de_pp;    // After prediction.
  DeltaEncoding de_ppc;   // After prediction, separate coordinates.
  DeltaEncoding de_ppl;   // After prediction, local frame.
  DeltaEncoding de_pplx;  // After prediction, local frame, per coord.
  DeltaEncoding de_pply;  // After prediction, local frame, per coord.
  DeltaEncoding de_pplz;  // After prediction, local frame, per coord.
  Polygon poly;
  for_int(vspli, pmesh._vsplits.num()) {
    const Vsplit& vspl = pmesh._vsplits[vspli];
    int ii = (vspl.code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
    assertx(ii == 2);
    assertx(is_zero(vspl.vad_small.dpoint));
    de_p.enter_vector(encode_dpoint(vspl.vad_large.dpoint));
    // Traverse faces above vs to find "North" ring of vertices.
    // Then compute centroid of these vertices as prediction for new vt.
    poly.init(0);
    int f = vspl.flclw;
    int vs_index = (vspl.code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
    int vs = pmi->_wedges[pmi->_faces[f].wedges[vs_index]].vertex;
    int nrot = vspl.vlr_offset1 - 1;
    poly.push(pmi->_vertices[vs].attrib.point);
    if (nrot < 0) {
      // Extend beyond a corner.  Use only vs itself as prediction (this makes sense).
    } else if (nrot == 0) {
      // Along a boundary.  Use vs and the next vertex along the boundary.
      for (;;) {  // rotate clw
        int j = pmi->get_jvf(vs, f);
        int fn = pmi->_fnei[f].faces[mod3(j + 2)];
        if (fn < 0) break;
        f = fn;
      }
      int j = pmi->get_jvf(vs, f);
      int vo = pmi->_wedges[pmi->_faces[f].wedges[mod3(j + 1)]].vertex;
      poly.push(pmi->_vertices[vo].attrib.point);
    } else {
      // Interior vertex.  Note: polygon will be oriented clockwise.
      {  // first add vl
        int j = pmi->get_jvf(vs, f);
        int vl = pmi->_wedges[pmi->_faces[f].wedges[mod3(j + 2)]].vertex;
        poly.push(pmi->_vertices[vl].attrib.point);
      }
      // Now add remaining vertices on ring.
      for_int(i, nrot) {
        assertx(f >= 0);
        int j = pmi->get_jvf(vs, f);
        int vo = pmi->_wedges[pmi->_faces[f].wedges[mod3(j + 1)]].vertex;
        poly.push(pmi->_vertices[vo].attrib.point);
        f = pmi->_fnei[f].faces[mod3(j + 2)];
      }
    }
    // SHOW(poly.num());
    // for_int(i, poly.num()) SHOW(poly[i]);
    Point pcentroid = mean(poly);
    Point pvs = pmi->_vertices[vs].attrib.point;
    Point pnew = pvs + vspl.vad_large.dpoint;
    Vector vdiff = pnew - pcentroid;
    // Compute local frame.
    Frame lframe;
    if (poly.num() < 3) {
      Warning("Using identity local frame");
      lframe = Frame::identity();
    } else {
      // Orientation of frame does not matter, as long as it is consistent.
      lframe.v(0) = poly.get_normal();  // Pointing in; does not matter.
      assertx(!is_zero(lframe.v(0)));
      // lframe.v(1) = poly[1] - pvs;  // Equal to vl - vs.
      lframe.v(1) = pcentroid - pvs;
      assertx(lframe.v(1).normalize());
      lframe.v(2) = cross(lframe.v(0), lframe.v(1));
      lframe.p() = Point(0.f, 0.f, 0.f);  // Unused.
    }
    // SHOW(vdiff);
    Vector vdiffl = vdiff * ~lframe;
    // SHOW(vdiffl);
    de_pp.enter_vector(encode_dpoint(vdiff));
    de_ppc.enter_coords(encode_dpoint(vdiff));
    de_ppl.enter_vector(encode_dpoint(vdiffl));
    Vector encvdiffl = encode_dpoint(vdiffl);
    de_pplx.enter_coords(V(encvdiffl[0]));
    de_pply.enter_coords(V(encvdiffl[1]));
    de_pplz.enter_coords(V(encvdiffl[2]));
    assertx(pmi->next());
  }
  {
    de_p.analyze("de_p");
    de_pp.analyze("de_pp");
    de_ppc.analyze("de_ppc");
    de_ppl.analyze("de_ppl");
    de_pplx.analyze("de_pplx");
    de_pply.analyze("de_pply");
    de_pplz.analyze("de_pplz");
  }
  nooutput = true;
}

void do_testiterate(Args& args) {
  int niter = args.get_int();
  {
    SHOW("loading file");
    ensure_pm_loaded();
    pmi->goto_nvertices(std::numeric_limits<int>::max());  // Go to end.
    SHOW("file loaded");
    pmi->goto_nvertices(0);  // Go back to base mesh.
    SHOW("back to base mesh");
  }
  Timer timer_max;
  Timer timer_min;
  for_int(i, niter) {
    {
      HH_TIMER("_gotomax");
      timer_max.start();
      pmi->goto_nvertices(std::numeric_limits<int>::max());
      timer_max.stop();
    }
    {
      HH_TIMER("_gotomin");
      timer_min.start();
      pmi->goto_nvertices(0);
      timer_min.stop();
    }
  }
  int nvsplits = pmesh._vsplits.num();
  showf("Goto_max: number of vertices / sec: %.f\n", nvsplits * float(niter) / timer_max.cpu());
  showf("Goto_min: number of vertices / sec: %.f\n", nvsplits * float(niter) / timer_min.cpu());
  nooutput = true;
}

void do_zero_vadsmall() {
  HH_TIMER("_zero_vadsmall");
  ensure_pm_loaded();
  pmi->goto_nvertices(std::numeric_limits<int>::max());
  // Now work backwards.
  for (;;) {
    const Vsplit* pcvspl = pmrs->prev_vsplit();
    if (!pcvspl) break;
    Vsplit& vspl = *const_cast<Vsplit*>(pcvspl);  // Vertex split modified by diff() below.
    assertx(pmrs->next_vsplit());
    unsigned code = vspl.code;
    int ii = (code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
    int fl = pmi->_faces.num() - (vspl.vlr_offset1 > 1 ? 2 : 1);
    int wvsfl = pmi->_faces[fl].wedges[0];
    int vs = pmi->_wedges[wvsfl].vertex;
    int vt = pmi->_vertices.num() - 1;
    assertx(pmi->_wedges[pmi->_faces[fl].wedges[1]].vertex == vt);
    const PmVertexAttrib& va_s = pmi->_vertices[vs].attrib;
    const PmVertexAttrib& va_t = pmi->_vertices[vt].attrib;
    switch (ii) {
      case 2:
        diff(vspl.vad_large, va_t, va_s);
        diff(vspl.vad_small, va_s, va_s);  // Set to zero.
        break;
      case 0:
        diff(vspl.vad_large, va_s, va_t);
        diff(vspl.vad_small, va_s, va_s);  // Set to zero.
        break;
      case 1: {
        PmVertexAttrib va_m;
        interp(va_m, va_s, va_t, 0.5f);
        diff(vspl.vad_large, va_t, va_m);
        diff(vspl.vad_small, va_s, va_s);  // Set to zero.
        break;
      }
      default: assertnever("");
    }
    assertx(pmi->prev());
  }
  // Now update the base mesh to reflect the new vertex positions.
  pmesh._base_mesh = *pmi;
}

void do_zero_normal() {
  ensure_pm_loaded();
  pmi->goto_nvertices(0);
  for_int(vspli, pmesh._vsplits.num()) {
    Vsplit& vspl = pmesh._vsplits[vspli];
    for_int(j, vspl.ar_wad.num()) vspl.ar_wad[j].dnormal = Vector(0.f, 0.f, 0.f);
  }
  for_int(w, pmesh._base_mesh._wedges.num()) pmesh._base_mesh._wedges[w].attrib.normal = Vector(0.f, 0.f, 0.f);
  implicit_cast<AWMesh&>(*pmi) = pmesh._base_mesh;
}

void do_zero_uvrgb() {
  ensure_pm_loaded();
  pmi->goto_nvertices(0);
  for_int(vspli, pmesh._vsplits.num()) {
    Vsplit& vspl = pmesh._vsplits[vspli];
    for_int(j, vspl.ar_wad.num()) {
      vspl.ar_wad[j].drgb = A3dColor(0.f, 0.f, 0.f);
      vspl.ar_wad[j].duv = Uv(0.f, 0.f);
    }
  }
  for_int(w, pmesh._base_mesh._wedges.num()) {
    pmesh._base_mesh._wedges[w].attrib.rgb = A3dColor(0.f, 0.f, 0.f);
    pmesh._base_mesh._wedges[w].attrib.uv = Uv(0.f, 0.f);
  }
  implicit_cast<AWMesh&>(*pmi) = pmesh._base_mesh;
  pmesh._info._has_rgb = false;
  pmesh._info._has_uv = false;
}

void do_zero_resid() {
  ensure_pm_loaded();
  for_int(vspli, pmesh._vsplits.num()) {
    Vsplit& vspl = pmesh._vsplits[vspli];
    vspl.resid_uni = 0.f;
    vspl.resid_dir = 0.f;
  }
  pmesh._info._has_resid = false;
}

PmWedgeAttrib zero_wad;  // Cannot declare const because default constructor leaves uninitialized.

void do_compute_nor() {
  if (1) assertnever("compute_nor() abandoned for now");
  // Compute normals in original mesh.
  ensure_pm_loaded();
  pmi->goto_nvertices(std::numeric_limits<int>::max());
  // Propagate normals down vsplits.
  for (;;) {
    const Vsplit* pcvspl = pmrs->prev_vsplit();
    if (!pcvspl) break;
    Vsplit& vspl = *const_cast<Vsplit*>(pcvspl);
    assertx(pmrs->next_vsplit());
    const int k_undefined = std::numeric_limits<int>::min();
    unsigned code = vspl.code;
    // int ii = (code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
    bool isr = vspl.adds_two_faces();
    int fl = pmi->_faces.num() - (isr ? 2 : 1);
    int fr = isr ? pmi->_faces.num() - 1 : k_undefined;
    int wvsfl = pmi->_faces[fl].wedges[0];
    int wvtfl = pmi->_faces[fl].wedges[1];
    int wvlfl = pmi->_faces[fl].wedges[2];
    int wvsfr = isr ? pmi->_faces[fr].wedges[0] : k_undefined;
    int wvtfr = isr ? pmi->_faces[fr].wedges[2] : k_undefined;
    int wvrfr = isr ? pmi->_faces[fr].wedges[1] : k_undefined;
    int vs = pmi->_wedges[wvsfl].vertex;
    int vt = pmi->_vertices.num() - 1;
    assertx(pmi->_wedges[wvtfl].vertex == vt);
    int lnum = 0;
    ArrayView<PmWedgeAttribD> ar_wad = vspl.ar_wad;
    if (1) {
      bool nt = !(code & Vsplit::T_LSAME);
      bool ns = !(code & Vsplit::S_LSAME);
      if (nt && ns) {
        diff(ar_wad[lnum++], pmi->_wedges[wvtfl].attrib, zero_wad);
        diff(ar_wad[lnum++], pmi->_wedges[wvsfl].attrib, zero_wad);
      } else {
        // Abandoned for now.
        dummy_use(wvlfl, wvsfr, wvtfr, wvrfr, vs);
      }
    }
    if (isr) {
    }
    // Abandoned for now.
    assertx(pmi->prev());
  }
  // Save normals of base mesh.
  pmesh._base_mesh = *pmi;
}

void do_transf(Args& args) {
  Frame frame = FrameIO::parse_frame(args.get_string());
  ensure_pm_loaded();
  showdf("Applying transform; normals (if any) may be wrong\n");
  pmi->goto_nvertices(0);
  for_int(vspli, pmesh._vsplits.num()) {
    Vsplit& vspl = pmesh._vsplits[vspli];
    vspl.vad_large.dpoint *= frame;
    vspl.vad_small.dpoint *= frame;
  }
  for_int(v, pmesh._base_mesh._vertices.num()) pmesh._base_mesh._vertices[v].attrib.point *= frame;
  implicit_cast<AWMesh&>(*pmi) = pmesh._base_mesh;
  for_int(min_max, 2) static_cast<Point&>(pmesh._info._full_bbox[min_max]) *= frame;  // Only valid for non-rotations.
}

// *** reorder_vspl

auto gather_faces(int vs, int f0) {
  Array<int> faces;
  faces.push(f0);
  int f;
  if (1) {
    // Rotate clw.
    f = f0;
    for (;;) {
      int j = pmi->get_jvf(vs, f);
      f = pmi->_fnei[f].faces[mod3(j + 2)];
      if (f < 0 || f == f0) break;
      faces.push(f);
    }
  }
  if (f < 0) {
    // Rotate ccw.
    f = f0;
    for (;;) {
      int j = pmi->get_jvf(vs, f);
      f = pmi->_fnei[f].faces[mod3(j + 1)];
      assertx(f != f0);
      if (f < 0) break;
      faces.push(f);
    }
  }
  return faces;
}

void global_reorder_vspl(int first_ivspl, int last_ivspl) {
  HH_ATIMER("_reorder");
  ensure_pm_loaded();
  assertx(first_ivspl >= 0);
  assertx(first_ivspl < last_ivspl);
  assertx(last_ivspl <= pmesh._info._tot_nvsplits);
  // ivspl refers to number of vsplits after first_ivspl !
  // Compute dependency graphs, record fl's in original sequence, and set up face renaming.
  Graph<int> gdep;       // ivspl -> previous ivspl on which it depends.
  Graph<int> gidep;      // Inverse relation of above.
  Array<int> ivspl_fl;   // ivspl -> face fl it creates.
  Array<int> oldf_newf;  // New face indexing (renaming) used later.
  {
    HH_ATIMER("__compute_depend");
    pmi->goto_nvertices(pmesh._base_mesh._vertices.num() + first_ivspl);
    Array<int> f_ivspldep;  // For each face, ivspl it depends on (or -1).
    for_int(f, pmi->_faces.num()) f_ivspldep.push(-1);
    for_int(f, pmi->_faces.num()) oldf_newf.push(f);
    Array<int> faces;
    for_int(ivspl, last_ivspl - first_ivspl) {
      const Vsplit& vspl = pmesh._vsplits[first_ivspl + ivspl];
      int f0 = vspl.flclw;  // Some face adjacent to vs.
      unsigned code = vspl.code;
      int vs_index = (code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
      int vs = pmi->_wedges[pmi->_faces[f0].wedges[vs_index]].vertex;
      bool isr = vspl.adds_two_faces();
      faces = gather_faces(vs, f0);
      gdep.enter(ivspl);
      gidep.enter(ivspl);
      for (int f : faces) {
        assertx(f_ivspldep.ok(f));
        int ivspldep = f_ivspldep[f];  // vsplit is dependent on ivspldep.
        if (ivspldep >= 0 && !gdep.contains(ivspl, ivspldep)) {
          gdep.enter(ivspl, ivspldep);
          gidep.enter(ivspldep, ivspl);
        }
        f_ivspldep[f] = ivspl;  // The face now depends on this vsplit.
      }
      // new 1 or 2 faces depend on this vsplit
      f_ivspldep.push(ivspl);
      if (isr) f_ivspldep.push(ivspl);
      oldf_newf.push(-1);
      if (isr) oldf_newf.push(-1);
      ivspl_fl.push(pmi->_faces.num());
      assertx(pmi->next());
      assertx(f_ivspldep.num() == pmi->_faces.num());
    }
    assertx(oldf_newf.num() == pmi->_faces.num());
  }
  // Now reorder refinement.
  Array<Vsplit> new_vsplits;
  for_int(i, first_ivspl) new_vsplits.push(pmesh._vsplits[i]);
  AWMesh temp_mesh;
  {
    HH_ATIMER("__new_vsplits");
    pmi->goto_nvertices(pmesh._base_mesh._vertices.num() + first_ivspl);
    temp_mesh = *pmi;
    struct Sivspl {
      int ivspl;
      int flclw1;  // flclw + 1, to reserve 0 for undefined Sivspl()
    };
    struct less_Sivspl {
      bool operator()(const Sivspl& s1, const Sivspl& s2) const {
        return s1.flclw1 < s2.flclw1;  // flclw1 may be zero
      }
    };
    STree<Sivspl, less_Sivspl> stivspl;  // current legal ivspl's
    Array<bool> ivspl_done;              // was ivspl already done?
    int ncand = 0;                       // size of stivspl
    for_int(ivspl, last_ivspl - first_ivspl) ivspl_done.push(false);
    if (1) {
      for (int ivspl : gdep.vertices()) {
        HH_SSTAT(Sdep_outdeg, gdep.out_degree(ivspl));
        HH_SSTAT(Sidep_outd, gidep.out_degree(ivspl));
      }
    }
    // Enter into STree all ivspl which do not depend on anything
    for (int ivspl : gdep.vertices()) {
      if (gdep.out_degree(ivspl)) continue;
      Sivspl n;
      n.ivspl = ivspl;
      int flclw = pmesh._vsplits[first_ivspl + ivspl].flclw;
      n.flclw1 = flclw + 1;
      assertx(oldf_newf[flclw] == flclw);
      assertx(stivspl.enter(n));
      ncand++;
    }
    Sivspl nlast;
    nlast.flclw1 = 0;
    static const bool bothways = getenv_bool("BOTH_WAYS");
    assertw(!bothways);
    while (!stivspl.empty()) {
      // number of mesh faces per candidate legal vsplit
      HH_SSTAT(Sfpcand, pmi->_faces.num() / float(ncand));
      Sivspl nmin;
      if (bothways) {
        Sivspl n1 = stivspl.pred_eq(nlast);
        Sivspl n2 = stivspl.succ(nlast);
        if (!n1.flclw1) n1 = stivspl.max();
        if (!n2.flclw1) n2 = stivspl.min();
        nmin = abs(n1.flclw1 - nlast.flclw1) < abs(n2.flclw1 - nlast.flclw1) ? n1 : n2;
      } else {
        nmin = stivspl.succ_eq(nlast);
        if (!nmin.flclw1) nmin = stivspl.min();
      }
      int ivspl = nmin.ivspl;
      int dflclw = nmin.flclw1 - nlast.flclw1;
      dflclw = wrap_dflclw(dflclw, pmi->_faces.num());
      if (sdebug) showf("ivspl=%5d   dflclw=%4d   ncand=%4d  nf=%5d\n", ivspl, dflclw, ncand, pmi->_faces.num());
      HH_SSTAT(Sdflclw, dflclw);
      HH_SSTAT(Sadflclw, abs(dflclw));
      assertx(stivspl.remove(nmin));
      --ncand;
      for (int ivspldep : gdep.edges(ivspl)) assertx(ivspl_done[ivspldep]);
      assertx(!ivspl_done[ivspl]);
      ivspl_done[ivspl] = true;
      // Construct new vsplit record.
      new_vsplits.push(pmesh._vsplits[first_ivspl + ivspl]);
      Vsplit& new_vspl = new_vsplits.last();
      assertx(oldf_newf[new_vspl.flclw] >= 0);
      new_vspl.flclw = oldf_newf[new_vspl.flclw];
      nlast.flclw1 = new_vspl.flclw + 1;
      for_int(j, new_vspl.adds_two_faces() ? 2 : 1) {
        assertx(oldf_newf[ivspl_fl[ivspl] + j] < 0);
        oldf_newf[ivspl_fl[ivspl] + j] = pmi->_faces.num() + j;
      }
      for (int ivsplnext : gidep.edges(ivspl)) {
        bool legal = true;
        for (int ivsplnextdep : gdep.edges(ivsplnext))
          if (!ivspl_done[ivsplnextdep]) legal = false;
        if (!legal) continue;
        Sivspl n;
        n.ivspl = ivsplnext;
        int oldflclw = pmesh._vsplits[first_ivspl + ivsplnext].flclw;
        assertx(oldf_newf[oldflclw] >= 0);
        n.flclw1 = oldf_newf[oldflclw] + 1;
        assertx(stivspl.enter(n));
        ncand++;
      }
      pmi->apply_vsplit_private(new_vspl, pmesh._info, nullptr);
    }
    assertx(ncand == 0);
    for (bool done : ivspl_done) assertx(done);
    for_int(i, pmesh._vsplits.num() - last_ivspl) {
      new_vsplits.push(pmesh._vsplits[last_ivspl + i]);
      Vsplit& new_vspl = new_vsplits.last();
      if (new_vspl.flclw < oldf_newf.num()) {
        assertx(oldf_newf[new_vspl.flclw] >= 0);
        new_vspl.flclw = oldf_newf[new_vspl.flclw];
      }
      pmi->apply_vsplit_private(new_vspl, pmesh._info, nullptr);
    }
    implicit_cast<AWMesh&>(*pmi) = temp_mesh;  // since PMeshRStream not advanced!
  }
  assertx(new_vsplits.num() == pmesh._vsplits.num());
  pmesh._vsplits = new_vsplits;
}

void do_reorder_vspl() {
  // e.g.: FilterPM ~/data/terrain/gcanyon_sq40.pm -reorder_vspl -compression
  // # DE de_dflclw: n=1596  nbits=3.0  sign=0.0  bdelta=3.7  total=6.7  (10693)
  // Encoded PMesh: 76079 bits (47.5 bits/vertex)
  //  6.7 & 1.6 & 2.5 & 0.8 & 0.0 & 35.9 & 0.0 & 0.0 & 0.0 & 47.4
  //  501 & 0 & 175 &  448 & 0 & 48
  global_reorder_vspl(0, pmesh._info._tot_nvsplits);
}

void do_lreorder_vspl(Args& args) {
  int first_ivspl = args.get_int();
  int last_ivspl = args.get_int();
  assertx(first_ivspl >= 0);
  assertx(first_ivspl < last_ivspl);
  assertx(last_ivspl <= pmesh._info._tot_nvsplits);
  global_reorder_vspl(first_ivspl, last_ivspl);
}

void do_exp_reorder(Args& args) {
  float fac = args.get_float();
  assertx(fac > 0.f);
  ensure_pm_loaded();
  int base_nv = pmesh._base_mesh._vertices.num();
  int full_nv = pmesh._info._full_nvertices;
  int nsteps = int(std::log(float(full_nv) / base_nv) / log(fac) + .5f);
  fac = pow(float(full_nv) / base_nv, 1.f / nsteps);
  showdf("Reordering: %d segments between %d and %d vertices, fac=%g\n", nsteps, base_nv, full_nv, fac);
  float nvf = float(base_nv);
  for_int(i, nsteps) {
    int nv0 = int(nvf + .5f);
    nvf *= fac;
    int nv1 = int(nvf + .5f);
    if (nv0 == nv1) {
      Warning("Reorder: empty segment");
      continue;
    }
    showdf(" reordering vspl in range [%d, %d]\n", nv0 - base_nv, nv1 - base_nv);
    global_reorder_vspl(nv0 - base_nv, nv1 - base_nv);
  }
}

void do_stat() {
  nooutput = true;
  ensure_pm_loaded();
  const AWMesh& bm = pmesh._base_mesh;
  showdf("Basemesh nv=%d nw=%d nf=%d\n", bm._vertices.num(), bm._wedges.num(), bm._faces.num());
  showdf("Fullmesh nv=%d nw=%d nf=%d\n",  //
         pmesh._info._full_nvertices, pmesh._info._full_nwedges, pmesh._info._full_nfaces);
  int nvsplits = pmesh._info._tot_nvsplits;
  showdf("Nvsplits=%d\n", nvsplits);
  if (nvsplits) {
    Vec3<int> ar_ii;
    fill(ar_ii, 0);
    int vlroffsetn1 = 0, vlroffset00 = 0;
    HH_STAT(Sresidu);
    HH_STAT(Sresidd);
    for_int(vspli, nvsplits) {
      const Vsplit& vspl = pmesh._vsplits[vspli];
      unsigned code = vspl.code;
      int ii = (code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
      ar_ii[ii]++;
      if (vspl.vlr_offset1 == 0) vlroffsetn1++;
      if (vspl.vlr_offset1 == 1) vlroffset00++;
      Sresidu.enter(vspl.resid_uni);
      Sresidd.enter(vspl.resid_dir);
    }
    for_int(ii, 3) showdf("ii==%d: %-5d %.1f%%\n", ii, ar_ii[ii], ar_ii[ii] * 100.f / nvsplits);
    showdf("#vlr_offsetn1=%d #vlr_offsetn2=%d\n", vlroffsetn1, vlroffset00);
    showdf("first_residu=%g first_residd=%g\n", pmesh._vsplits[0].resid_uni, pmesh._vsplits[0].resid_dir);
    showdf("last_residu=%g last_residd=%g\n", pmesh._vsplits.last().resid_uni, pmesh._vsplits.last().resid_dir);
  }
}

void do_write_resid_uni() {
  nooutput = true;
  ensure_pm_loaded();
  for_int(vspli, pmesh._vsplits.num()) {
    Vsplit& vspl = pmesh._vsplits[vspli];
    std::cout << sform("%g\n", vspl.resid_uni);
  }
}

void do_write_resid_dir() {
  nooutput = true;
  ensure_pm_loaded();
  for_int(vspli, pmesh._vsplits.num()) {
    Vsplit& vspl = pmesh._vsplits[vspli];
    std::cout << sform("%g\n", vspl.resid_dir);
  }
}

// *** TVC

// Cache type must be set explicitly on the command line.
VertexCache::EType cache_type = VertexCache::EType::notype;
int cache_size = 16;  // default size is 16-entry cache

// see MeshReorder.cpp
constexpr int k_bytes_per_vertex = 32;  // default position + normal + uv
constexpr int k_bytes_per_vindex = 2;
constexpr int k_strip_restart_nvindices = 1;

void do_fifo() { cache_type = VertexCache::EType::fifo; }

void do_lru() { cache_type = VertexCache::EType::lru; }

// Print some statistics and return number of vertex cache misses.
int analyze_mesh(int cs) {
  auto up_vcache = VertexCache::make(cache_type, 1 + pmi->_wedges.num(), cs);
  VertexCache& vcache = *up_vcache;
  int nmiss = 0;
  for_int(fi, pmi->_faces.num()) {
    for_int(j, 3) {
      int wi = pmi->_faces[fi].wedges[j];
      nmiss += !vcache.access_hits(1 + wi);
    }
  }
  if (0)
    showdf("cs=%-6d nmiss=%-6d %5.1f%%  v/t=%5.3f  v/v=%5.3f\n",  //
           cs, nmiss, float(nmiss) / (3 * pmi->_faces.num()) * 100.f, float(nmiss) / pmi->_faces.num(),
           float(nmiss) / pmi->_wedges.num());
  return nmiss;
}

void analyze_strips(int& pnverts, int& pnstrips) {
  const bool debug = false;
  // newway: vertices reused from prev face are in vid[0..1]; expected_j: 1, 2, 1, 2, ...
  // Assumption: turn face1-face2-face3 is expected to be ccw.
  const int first_expected_j = 1;
  const int sum_expected_j = 3;
  int last_matid = -1;
  Vec3<int> ovid;
  fill(ovid, -1);
  int expected_j;
  dummy_init(expected_j);
  int nstrips = 0;
  int nverts = 0;
  for_int(fi, pmi->_faces.num()) {
    int matid = pmi->_faces[fi].attrib.matid;
    if (matid != last_matid) {
      last_matid = matid;
      // Force new strip at mat boundary.
      fill(ovid, -1);
    }
    int j = 3;
    for_int(k, 3) {
      if (ovid[mod3(k + 0)] == pmi->_faces[fi].wedges[1] && ovid[mod3(k + 1)] == pmi->_faces[fi].wedges[0]) {
        j = k;
        break;
      }
    }
    if (j == 3) {  // Not face-face connected.
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
      // expected_j stays the same:  LRLR*R*LRLR.
    } else {
      if (debug) std::cerr << "*";
      if (0) Warning("Strip turns on itself");
      nstrips++;
      if (fi) nverts += k_strip_restart_nvindices;
      nverts += 3;
      expected_j = first_expected_j;
    }
    ovid = pmi->_faces[fi].wedges;
  }
  if (debug) std::cerr << "\n";
  if (0)
    showdf("Vertex_indices: nstrips=%d, nverts=%d, vi/t=%-5.3f\n", nstrips, nverts, float(nverts) / pmi->_faces.num());
  pnverts = nverts;
  pnstrips = nstrips;
}

// Analyze the bandwidth of the mesh under the transparent vertex caching framework,
//  using the current cache type and size.
void do_tvc_analyze() {
  HH_PTIMER("_tvc_analyze");
  showdf("Mesh analysis (%s)\n", VertexCache::type_string(cache_type).c_str());
  int nmiss = analyze_mesh(cache_size);
  int nverts, nstrips;
  analyze_strips(nverts, nstrips);
  float b_v = float(nmiss * k_bytes_per_vertex) / pmi->_faces.num();
  float b_i = float(nverts * k_bytes_per_vindex) / pmi->_faces.num();
  float b_t = b_v + b_i;
  if (0) showdf("Bandwidth: vertices %4.2f b/t, indices %4.2f b/t, Total %4.2f byte/tri\n", b_v, b_i, b_t);
  string nametail = get_path_tail(gfilename);
  if (1)
    showdf("%-14.14s v/t=%5.3f v/v=%5.3f slen=%4.1f bv=%4.2f bi=%4.2f bt=%4.2f\n",  //
           nametail.c_str(), float(nmiss) / pmi->_faces.num(), float(nmiss) / pmi->_vertices.num(),
           float(pmi->_faces.num()) / nstrips, b_v, b_i, b_t);
  nooutput = true;
}

void do_graph_tvc() {
  int nf = pmi->_faces.num();
  for (;;) {
    int nmiss = analyze_mesh(cache_size);
    if (1) std::cout << sform("%d %g\n", nf, float(nmiss) / pmi->_vertices.num());
    if (nf == pmesh._info._full_nfaces) break;
    nf = max(nf + 10, int(nf * 1.1f));
    nf = min(nf, pmesh._info._full_nfaces);
    pmi->goto_nfaces(nf);
  }
  nooutput = true;
}

void do_polystream() {
  pmi->goto_nvertices(0);
  ensure_pm_loaded();
  WSA3dStream ws(std::cout);
  A3dElem el;
  ws.write_comment("Beg of base mesh");
  for_int(f, pmi->_faces.num()) {
    el.init(A3dElem::EType::polygon);
    for_int(j, 3) {
      int w = pmi->_faces[f].wedges[j];
      int v = pmi->_wedges[w].vertex;
      el.push(A3dVertex(pmi->_vertices[v].attrib.point, pmi->_wedges[w].attrib.normal,
                        A3dVertexColor(pmi->_wedges[w].attrib.rgb)));
    }
    ws.write(el);
  }
  ws.write_comment("End of base mesh");
  for_int(vspli, pmesh._vsplits.num()) {
    ws.write_comment("Beg of vsplit");
    const Vsplit& vspl = pmesh._vsplits[vspli];
    int vs;
    {
      // int ii = (vspl.code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
      int f = vspl.flclw;
      int vs_index = (vspl.code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
      vs = pmi->_wedges[pmi->_faces[f].wedges[vs_index]].vertex;
      int nrot = vspl.vlr_offset1 - 1;
      assertx(nrot >= 0);
    }
    pmi->next();
    int vt = pmi->_vertices.num() - 1;
    int fl = pmi->_faces.num() - 2, fr = fl + 1;
    for_int(vvi, 2) {
      int vc = vvi ? vt : vs;
      int f = fl;
      for (;;) {
        assertx(f >= 0);
        el.init(A3dElem::EType::polygon);
        int j0 = pmi->get_jvf(vc, f);
        if (!(vc == vt && (f == fl || f == fr))) {
          for_int(jj, 3) {
            int j = mod3(j0 + jj);
            int w = pmi->_faces[f].wedges[j];
            int v = pmi->_wedges[w].vertex;
            el.push(A3dVertex(pmi->_vertices[v].attrib.point, pmi->_wedges[w].attrib.normal,
                              A3dVertexColor(pmi->_wedges[w].attrib.rgb)));
          }
          ws.write(el);
        }
        f = pmi->_fnei[f].faces[mod3(j0 + 2)];
        if (f == fl) break;
      }
    }
    ws.write_comment("End of vsplit");
  }
  nooutput = true;
}

Point sph_from_lonlat(const Uv& uv) {
  assertx(uv[0] >= 0.f && uv[0] <= 1.f);
  assertx(uv[1] >= 0.f && uv[1] <= 1.f);
  float lon = (uv[0] - .5f) * TAU;        // -TAU / 2 .. +TAU / 2.
  float lat = (uv[1] - .5f) * (TAU / 2);  // -TAU / 4 .. +TAU / 4.
  // My coordinate system.
  return Point(std::cos(lon) * std::cos(lat), std::sin(lon) * std::cos(lat), std::sin(lat));
}

// Problems that make this visualization useless:
// - The base tetrahedron is not at all regular.
// - Currently, the PM file does not have the final registration rotation that the .sphparam.m file does, so the
//   normals are all wrong.
void do_uvsphtopos() {
  // Assumes: 1 wedge per vertex, ii == 2 everywhere.
  assertx(pmesh._info._has_uv);
  Array<int> array_vs;  // vspli -> vs.
  ensure_pm_loaded();
  pmi->goto_nvertices(0);
  for_int(vspli, pmesh._vsplits.num()) {
    const Vsplit& vspl = pmesh._vsplits[vspli];
    int ii = (vspl.code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
    assertx(ii == 2);
    assertx(vspl.ar_wad.num() == 1);
    int f = vspl.flclw;
    int vs_index = (vspl.code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
    int vs = pmi->_wedges[pmi->_faces[f].wedges[vs_index]].vertex;
    array_vs.push(vs);
    pmi->next();
  }
  assertx(pmi->_wedges.num() == pmi->_vertices.num());
  Array<Point> sphpoints(pmi->_wedges.num());
  for_int(w, pmi->_wedges.num()) {
    assertx(pmi->_wedges[w].vertex == w);
    const Uv& uv = pmi->_wedges[w].attrib.uv;
    Point sph = sph_from_lonlat(uv);
    sphpoints[w] = sph;
  }
  AWMesh& bmesh = pmesh._base_mesh;
  for_int(w, bmesh._wedges.num()) bmesh._vertices[w].attrib.point = sphpoints[w];
  for_int(vspli, pmesh._vsplits.num()) {
    Vsplit& vspl = pmesh._vsplits[vspli];
    int vs = array_vs[vspli];
    int vt = pmesh._base_mesh._vertices.num() + vspli;
    vspl.vad_large.dpoint = sphpoints[vt] - sphpoints[vs];
    assertx(is_zero(vspl.vad_small.dpoint));
  }
  pmesh._info._full_bbox = Bbox{sphpoints};
  pmesh._info._has_uv = false;  // Clear the uv coordinates.
}

}  // namespace

int main(int argc, const char** argv) {
  assertw(!sdebug);
  ParseArgs args(argc, argv);
  HH_ARGSC("A progressive mesh is read from stdin or first arg.  Subsequent options are:");
  HH_ARGSC(HH_ARGS_INDENT "Go to specific mesh:");
  HH_ARGSD(nvertices, "nverts : goto mesh with that many vertices");
  HH_ARGSD(nfaces, "nfaces : goto mesh with that many faces");
  HH_ARGSD(nedges, "nedges : goto mesh with that many edges");
  HH_ARGSD(nsplits, "nsplits : goto mesh after that many vsplits");
  HH_ARGSD(maxresidd, "residd : goto mesh with <=resid_dir error");
  HH_ARGSD(coarsest, ": goto to base mesh");
  HH_ARGSD(finest, ": goto to fully detailed mesh");
  HH_ARGSC(HH_ARGS_INDENT "Act on current mesh:");
  HH_ARGSD(info, ": output stats on current mesh");
  HH_ARGSD(minfo, ": output more stats on current mesh");
  HH_ARGSD(outmesh, ": output mesh");
  HH_ARGSD(outsmesh, ": output simple mesh (split wedges)");
  HH_ARGSD(geom_nfaces, "nf : output geomorph up to nf faces");
  HH_ARGSC(HH_ARGS_INDENT "Output selectively refined meshes and geomorphs:");
  HH_ARGSD(srout, "'frame' srthresh : create SR mesh");
  HH_ARGSD(srgeomorph, "{'frame' srthresh} * 2 : create SR geomorph");
  HH_ARGSD(srfgeo, "rtime ctime :  set fly parameters");
  HH_ARGSD(srfly, "file.frames scthresh : (for timing)");
  HH_ARGSD(tosrm, ": convert to .srm format");
  HH_ARGSC(HH_ARGS_INDENT "Modify progressive mesh:");
  HH_ARGSD(truncate_beyond, ": truncate PM beyond current mesh");
  HH_ARGSD(truncate_prior, ": advance base_mesh to current mesh");
  HH_ARGSD(quantize, ": quantize and output PM");
  HH_ARGSD(zero_vadsmall, ": recompute to keep only 1 delta per vertex");
  HH_ARGSD(zero_normal, ": zero out all normals");
  HH_ARGSD(zero_uvrgb, ": zero out all {uv, rgb} information");
  HH_ARGSD(zero_resid, ": zero out all residuals");
  HH_ARGSD(compute_nor, ": recompute normals based on wedges");
  HH_ARGSD(transf, "'frame' : affine transform all vertices");
  HH_ARGSD(reorder_vspl, ": reorder vsplits to improve compression");
  HH_ARGSD(lreorder_vspl, "fvspl lvspl : reorder within vsplit range");
  HH_ARGSD(exp_reorder, "fac : several reorders; verts *= fac");
  HH_ARGSC(HH_ARGS_INDENT "Analyze PM:");
  HH_ARGSD(stat, ": output some stats on file");
  HH_ARGSP(verb, "level : verbosity level");
  HH_ARGSF(gzip, ": in compression, include gzip analysis");
  HH_ARGSD(compression, ": analyze compression of vsplits");
  HH_ARGSD(gcompression, ": try improved geometry compression");
  HH_ARGSD(write_resid_uni, ": output uniform residuals");
  HH_ARGSD(write_resid_dir, ": output directional residuals");
  HH_ARGSC(HH_ARGS_INDENT "Vertex caching:");
  HH_ARGSD(fifo, ": set cache type to FIFO");
  HH_ARGSD(lru, ": set cache type to LRU");
  HH_ARGSP(cache_size, "n : set number of cache entries");
  HH_ARGSD(tvc_analyze, ": analyze vertex caching of current mesh");
  HH_ARGSD(graph_tvc, ": output sequence {(nf, vmiss / v)}");
  HH_ARGSC(HH_ARGS_INDENT "Misc:");
  HH_ARGSD(testiterate, "n : run n iterations back and forth");
  HH_ARGSD(polystream, ": for progressive hull, refine polygons");
  HH_ARGSD(uvsphtopos, ": transfer uv longlat to sphere pos");
  HH_ARGSF(nooutput, ": do not output final PM");

  const string arg0 = args.num() ? args.peek_string() : "";
  if (ParseArgs::special_arg(arg0)) args.parse(), exit(0);
  const string filename = args.num() && (arg0 == "-" || arg0[0] != '-') ? args.get_filename() : "-";
  gfilename = filename;
  RFile fi(filename);  // Opened out here because &fi is captured below.
  {
    HH_TIMER("FilterPM");
    for (string line; fi().peek() == '#';) {
      assertx(my_getline(fi(), line));
      if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
    }
    assertx(fi().peek() == 'P' || fi().peek() == 'S');
    bool srm_input = fi().peek() == 'S';
    showff("%s", args.header().c_str());
    if (arg0 == "-tosrm") {
      // It will do its own efficient parsing.
      pfi = &fi;
    } else if (srm_input) {
      nooutput = true;
      pfi = &fi;
    } else {
      pmrs = make_unique<PMeshRStream>(fi(), &pmesh);
      pmi = make_unique<PMeshIter>(*pmrs);
    }
    args.parse();
    if (!nooutput) ensure_pm_loaded();
  }
  hh_clean_up();
  if (!nooutput) pmesh.write(std::cout);
  if (filename == "-")  // Read entire input stream to avoid broken pipe.
    while (pmrs->next_vsplit())
      ;
  pmi = nullptr;
  pmrs = nullptr;
  return 0;
}
