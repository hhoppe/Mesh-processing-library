// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "progs/MeshToPm/PmBuild.h"

#include "libHh/RangeOp.h"

namespace hh {

namespace {

// Returns the wedge of vertex v in face f, or -1 if f < 0.
int wedge_in(const WorkMesh& mesh, int v, int f) {
  if (f < 0) return -1;
  for_int(j, 3) if (mesh.vertex(f, j) == v) return mesh._faces[f].wedges[j];
  assertnever("");
}

// As in Filterprog: the codes of the wedges of vs (or vt) in fl and fr relative to those outside.
unsigned encode_wst(int wid_l, int wid_r, int wid_lo, int wid_ro) {
  if (wid_r < 0) {
    assertx(wid_ro < 0);
    return Vsplit::B_RSAME | (wid_l == wid_lo ? Vsplit::B_LSAME : 0u);
  }
  return ((wid_l == wid_lo ? Vsplit::B_LSAME : 0u) | (wid_r == wid_ro ? Vsplit::B_RSAME : 0u) |
          (wid_l == wid_r ? Vsplit::B_CSAME : 0u));
}

// As in Filterprog: the code of the wedge of vl (or vr) relative to the wedges above and below.
unsigned encode_wlr(int wid_s, int wid_a, int wid_b) {
  return wid_s == wid_a ? Vsplit::B_ABOVE : wid_s == wid_b ? Vsplit::B_BELOW : Vsplit::B_NEW;
}

PmWedgeAttribD diff_attrib(const PmWedgeAttrib& a1, const PmWedgeAttrib& a2) {
  PmWedgeAttribD ad;
  diff(ad, a1, a2);
  return ad;
}

PmWedgeAttribD diff_zero(const PmWedgeAttrib& a1) { return diff_attrib(a1, PmWedgeAttrib{}); }

// In face f, replace the neighbor fold by fnew (which may be -1).
void relink(WorkMesh& mesh, int f, int fold, int fnew) {
  if (f < 0) return;
  for_int(j, 3) {
    if (mesh._fnei[f].faces[j] == fold) {
      mesh._fnei[f].faces[j] = fnew < 0 ? AWMesh::k_undefined : fnew;
      return;
    }
  }
  assertnever("");
}

}  // namespace

void start_recording(const WorkMesh& mesh, PmRecorder& recorder) {
  assertx(!recorder.started);
  recorder.started = true;
  recorder.info = full_mesh_info(mesh);
  const int nv = recorder.info._full_nvertices;  // An upper bound on the number of collapses.
  recorder.vsplits.reserve(nv);
  recorder.wad_values.reserve(nv * recorder.wad_stride());  // Typically about one delta per vsplit.
  recorder.removed_original_ids.reserve(nv);
  recorder.num_faces_at_start = recorder.info._full_nfaces;
}

void collapse(WorkMesh& mesh, const EcolConfig& config, PmRecorder& recorder) {
  const int vs = config.vs, vt = config.vt, vl = config.vl, vr = config.vr, fl = config.fl, fr = config.fr;
  assertx(config.ii == 2);
  const bool record = recorder.started;
  // The faces around fl and fr: fa across (vt, vl), fb across (vs, vl), fc across (vt, vr), fd across (vs, vr).
  const int fa = mesh.face_across(fl, vt, vl), fb = mesh.face_across(fl, vs, vl);
  const int fc = mesh.face_across(fr, vt, vr), fd = mesh.face_across(fr, vs, vr);
  const auto attrib = [&](int w) { return w < 0 ? PmWedgeAttrib{} : mesh._wedges[w].attrib; };
  Vsplit vspl;
  unsigned code = 0;
  int flclw_corner = -1;
  PmWedgeAttrib a_vtfl, a_vsfl, a_vlfl, a_vtfr, a_vsfr, a_vrfr;
  if (record) {
    // Encode the vsplit as Filterprog's parse_line2() and process_vsplit() do, from the mesh before the collapse
    // (i.e., after the vsplit).
    code |= unsigned{2} << Vsplit::II_SHIFT;
    const int wvsfl = wedge_in(mesh, vs, fl), wvsfr = wedge_in(mesh, vs, fr);
    const int wvtfl = wedge_in(mesh, vt, fl), wvtfr = wedge_in(mesh, vt, fr);
    code |= encode_wst(wvsfl, wvsfr, wedge_in(mesh, vs, fb), wedge_in(mesh, vs, fd)) << Vsplit::S_SHIFT;
    code |= encode_wst(wvtfl, wvtfr, wedge_in(mesh, vt, fa), wedge_in(mesh, vt, fc)) << Vsplit::T_SHIFT;
    const int wvlfl = wedge_in(mesh, vl, fl);
    code |= encode_wlr(wvlfl, wedge_in(mesh, vl, fa), wedge_in(mesh, vl, fb)) << Vsplit::L_SHIFT;
    const int wvrfr = wedge_in(mesh, vr, fr);
    if (vr < 0)
      code |= Vsplit::R_ABOVE;
    else
      code |= encode_wlr(wvrfr, wedge_in(mesh, vr, fc), wedge_in(mesh, vr, fd)) << Vsplit::R_SHIFT;
    // Predict the face materials (with ii == 2, from the face clw of the new face, else from the ccw one).
    const auto matid = [&](int f) { return f < 0 ? -1 : mesh._faces[f].attrib.matid; };
    const auto predict = [&](int f, int f_a, int f_b, unsigned mask, ushort& fmatid) {
      const int predicted = matid(f_a) >= 0 ? matid(f_a) : matid(f_b);
      fmatid = 0;
      if (matid(f) != predicted) {
        fmatid = narrow_cast<ushort>(matid(f));
        code |= mask;
      }
    };
    predict(fl, fa, fb, Vsplit::FLN_MASK, vspl.fl_matid);
    vspl.fr_matid = 0;
    if (fr >= 0) predict(fr, fc, fd, Vsplit::FRN_MASK, vspl.fr_matid);
    diff(vspl.vad_large, mesh._vertices[vt].attrib, mesh._vertices[vs].attrib);
    vspl.vad_small.dpoint = Vector(0.f, 0.f, 0.f);
    vspl.resid_uni = 0.f;
    vspl.resid_dir = 0.f;
    a_vtfl = attrib(wvtfl), a_vsfl = attrib(wvsfl), a_vlfl = attrib(wvlfl);
    a_vtfr = attrib(wvtfr), a_vsfr = attrib(wvsfr), a_vrfr = attrib(wvrfr);
    // The face flclw just clw of vl about vs after the collapse, and the position of vs in it.
    if (vr < 0) {
      vspl.vlr_offset1 = fa >= 0 ? 1 : 0;  // Special codes.
    } else {
      const int nfaces_vt = narrow_cast<int>(ranges::distance(mesh.ccw_faces(vt, fl)));
      vspl.vlr_offset1 = narrow_cast<short>(nfaces_vt - 1);  // One plus the number of clw steps from vl to vr.
    }
    if (vspl.vlr_offset1 == 0) {
      assertx(fb >= 0);
      vspl.flclw = fb;
      flclw_corner = mesh.get_jvf(vs, fb);
    } else {
      vspl.flclw = fa;
      flclw_corner = mesh.get_jvf(vt, fa);
    }
  }
  // The positions of vs in fl and fr (a face created by a vsplit starts at vs), determined before any wedge of vt
  // is transferred to vs, after which a face could have two corners whose wedges refer to vs.
  const int rotation_fl = mesh.get_jvf(vs, fl), rotation_fr = fr >= 0 ? mesh.get_jvf(vs, fr) : 0;
  // Perform the collapse.
  for (const auto& [c, w] : config.corner_wedges) mesh._faces[c / 3].wedges[c % 3] = w;
  for (const auto& [w, wa] : config.wedge_attribs) mesh._wedges[w].attrib = wa;
  for (const auto& [c, w] : config.corner_wedges)
    if (mesh._wedges[w].vertex == vt) mesh._wedges[w].vertex = vs;
  relink(mesh, fa, fl, fb);
  relink(mesh, fb, fl, fa);
  if (fr >= 0) {
    relink(mesh, fc, fr, fd);
    relink(mesh, fd, fr, fc);
  }
  const int num_removed = fr >= 0 ? 2 : 1;
  int pm_index = -1;
  if (record) {
    recorder.num_faces_removed += num_removed;
    pm_index = recorder.num_faces_at_start - recorder.num_faces_removed;
  }
  for (const int f : {fl, fr}) {
    if (f < 0) continue;
    const int rotation = f == fl ? rotation_fl : rotation_fr;
    mesh._faces[f].wedges = V(WorkMesh::k_face_removed, pm_index < 0 ? -1 : pm_index + (f == fr), rotation);
    mesh._fnei[f].faces = ntimes<3>(AWMesh::k_undefined);
  }
  mesh._someface[vs] = config.corner_wedges[0].first / 3;
  if (mesh._someface[vl] == fl) {
    assertx(fa >= 0 || fb >= 0);
    mesh._someface[vl] = fa >= 0 ? fa : fb;
  }
  if (vr >= 0 && mesh._someface[vr] == fr) {
    assertx(fc >= 0 || fd >= 0);
    mesh._someface[vr] = fc >= 0 ? fc : fd;
  }
  if (mesh._vflags[vt] & WorkMesh::k_vflag_boundary) mesh._vflags[vs] |= WorkMesh::k_vflag_boundary;
  mesh._someface[vt] = -1;
  mesh._vflags[vt] |= WorkMesh::k_vflag_removed;
  if (!record) return;
  // The wedge attributes about vs after the collapse (i.e., before the vsplit).
  const PmWedgeAttrib wavtflo = attrib(wedge_in(mesh, vs, fa)), wavsflo = attrib(wedge_in(mesh, vs, fb));
  const PmWedgeAttrib wavtfro = attrib(wedge_in(mesh, vs, fc)), wavsfro = attrib(wedge_in(mesh, vs, fd));
  // Encode the wedge attributes as Filterprog's process_vsplit() does for ii == 2.
  vspl.ar_wad.init(0);
  {
    const bool nt = !(code & Vsplit::T_LSAME), ns = !(code & Vsplit::S_LSAME);
    if (nt && ns) {
      vspl.ar_wad.push(diff_zero(a_vtfl));
      vspl.ar_wad.push(diff_zero(a_vsfl));
    } else {
      const PmWedgeAttrib& waovsfl = (code & Vsplit::S_LSAME) ? wavsflo : wavsfro;
      const PmWedgeAttrib& waovtfl = (code & Vsplit::T_LSAME) ? wavtflo : wavtfro;
      vspl.ar_wad.push(diff_attrib(a_vtfl, !ns ? waovsfl : waovtfl));
    }
  }
  if (vr >= 0) {
    const bool nt = !(code & Vsplit::T_RSAME), ns = !(code & Vsplit::S_RSAME);
    const bool ut = !(code & Vsplit::T_CSAME), us = !(code & Vsplit::S_CSAME);
    if (nt && ns) {
      if (ut) vspl.ar_wad.push(diff_zero(a_vtfr));
      if (us) vspl.ar_wad.push(diff_zero(a_vsfr));
    } else {
      const PmWedgeAttrib& waovsfr = (code & Vsplit::S_RSAME) ? wavsfro : wavsflo;
      const PmWedgeAttrib& waovtfr = (code & Vsplit::T_RSAME) ? wavtfro : wavtflo;
      if (ut) vspl.ar_wad.push(diff_attrib(a_vtfr, !ns ? waovsfr : waovtfr));
    }
  }
  if (code & Vsplit::L_NEW) vspl.ar_wad.push(diff_zero(a_vlfl));
  if (code & Vsplit::R_NEW) vspl.ar_wad.push(diff_zero(a_vrfr));
  assertx(!(code & Vsplit::VSINDEX_MASK) && flclw_corner >= 0 && flclw_corner < 3);
  code |= unsigned(flclw_corner) << Vsplit::VSINDEX_SHIFT;  // Temporarily; see finalize_pm().
  for (const PmWedgeAttribD& ad : vspl.ar_wad) {
    recorder.wad_values.push_array(ad.dnormal);
    if (recorder.info._has_rgb) recorder.wad_values.push_array(ad.drgb);
    if (recorder.info._has_uv) recorder.wad_values.push_array(ad.duv);
  }
  const int wad_end = recorder.wad_values.num() / recorder.wad_stride();
  recorder.vsplits.push(CompactVsplit{vspl.flclw, wad_end, vspl.vad_large.dpoint, vspl.vlr_offset1,
                                      narrow_cast<ushort>(code), vspl.fl_matid, vspl.fr_matid});
  recorder.removed_original_ids.push(mesh._vertex_original_id[vt]);
}

Vsplit PmRecorder::vsplit(int i) const {
  const CompactVsplit& cv = vsplits[i];
  Vsplit vspl;
  vspl.flclw = cv.flclw;
  vspl.vlr_offset1 = cv.vlr_offset1;
  vspl.code = cv.code;
  vspl.fl_matid = cv.fl_matid;
  vspl.fr_matid = cv.fr_matid;
  vspl.vad_large.dpoint = cv.vad_large;
  vspl.vad_small.dpoint = Vector(0.f, 0.f, 0.f);
  const int wad_start = i ? vsplits[i - 1].wad_end : 0;
  const int stride = wad_stride();
  vspl.ar_wad.init(cv.wad_end - wad_start);
  for_int(k, vspl.ar_wad.num()) {
    PmWedgeAttribD& ad = vspl.ar_wad[k];
    const float* p = &wad_values[(wad_start + k) * stride];
    for_int(c, 3) ad.dnormal[c] = *p++;
    ad.drgb = A3dColor(0.f, 0.f, 0.f);
    if (info._has_rgb) for_int(c, 3) ad.drgb[c] = *p++;
    ad.duv = Uv(0.f, 0.f);
    if (info._has_uv) for_int(c, 2) ad.duv[c] = *p++;
  }
  vspl.resid_uni = 0.f;
  vspl.resid_dir = 0.f;
  return vspl;
}

void finalize_pm(const WorkMesh& mesh, PmRecorder& recorder, AWMesh& base_mesh, PMeshInfo& info) {
  assertx(recorder.started);
  Array<int> base_index(mesh._faces.num(), -1);
  {
    int i = 0;
    for_int(f, mesh._faces.num()) if (mesh.face_alive(f)) base_index[f] = i++;
  }
  for (CompactVsplit& cv : recorder.vsplits) {
    const int f = cv.flclw;
    const bool alive = mesh.face_alive(f);
    const int pm_face = alive ? base_index[f] : mesh._faces[f].wedges[1];
    const int rotation = alive ? 0 : mesh._faces[f].wedges[2];
    assertx(pm_face >= 0);
    cv.flclw = pm_face;
    const int corner = (cv.code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
    const unsigned vs_index = mod3(corner - rotation + 3);
    cv.code = narrow_cast<ushort>((cv.code & ~Vsplit::VSINDEX_MASK) | (vs_index << Vsplit::VSINDEX_SHIFT));
  }
  base_mesh = compact_awmesh(mesh);
  if (k_debug) base_mesh.ok();
  info = recorder.info;
  info._tot_nvsplits = recorder.vsplits.num();
}

void write_pm(std::ostream& os, const AWMesh& base_mesh, const PMeshInfo& info, const PmRecorder& recorder) {
  PMesh::write_header_and_base_mesh(os, info, base_mesh);
  for (int i = recorder.vsplits.num() - 1; i >= 0; --i) recorder.vsplit(i).write(os, info);
  PMesh::write_trailer(os);
}

void make_pmesh(const AWMesh& base_mesh, const PMeshInfo& info, const PmRecorder& recorder, PMesh& pmesh) {
  pmesh._base_mesh = base_mesh;
  pmesh._info = info;
  pmesh._vsplits.init(0);
  pmesh._vsplits.reserve(recorder.vsplits.num());
  for (int i = recorder.vsplits.num() - 1; i >= 0; --i) pmesh._vsplits.push(recorder.vsplit(i));
}

Array<int> pm_original_ids(const WorkMesh& mesh, const PmRecorder& recorder) {
  Array<int> ids;
  for_int(v, mesh._vertices.num()) if (mesh.vertex_alive(v)) ids.push(mesh._vertex_original_id[v]);
  sort(ids);
  for_int(i, recorder.removed_original_ids.num())
      ids.push(recorder.removed_original_ids[recorder.removed_original_ids.num() - 1 - i]);
  return ids;
}

}  // namespace hh
