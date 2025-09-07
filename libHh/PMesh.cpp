// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/PMesh.h"

#include "libHh/BinaryIO.h"   // read_binary_std() and write_binary_std()
#include "libHh/GMesh.h"      // in extract_gmesh()
#include "libHh/HashTuple.h"  // hash<pair<...>>
#include "libHh/PArray.h"     // ar_pwedge
#include "libHh/RangeOp.h"    // fill()
#include "libHh/Set.h"
#include "libHh/Vector4.h"

namespace hh {

constexpr int k_undefined = k_debug ? std::numeric_limits<int>::min() : -1;

// *** Performance

// Analysis of memory requirement:
//
//  PmVertex:   3*4     *1v/v   == 12 bytes/vertex
//  PmWedge:    6*4     *1w/v   == 24 bytes/vertex (** Assume: 1 wedge/vertex)
//  PmFace:     4*4     *2f/v   == 32 bytes/vertex
//  PmFaceN:    3*4     *2f/v   == 24 bytes/vertex
//
//  WMesh:      68 bytes/vertex
//
//  AWMesh:     92 bytes/vertex
//
//  PmSVertex:  8*4     *1v/v   == 32 bytes/vertex
//  PmSFace:    4*4     *2f/v   == 32 bytes/vertex
//
//  SMesh:      64 bytes/vertex
//
//  Vsplit:     12+2*12+1*20 *1vsp/v    == 56+ bytes/vertex
//
//   if compacted: 8+2*12+1*20 + fl_matid's + extra wads == 52+ bytes/vertex
//
//  PMesh:      56 bytes/vertex (smaller than WMesh and SMesh!)
//
// optimize:
// - if redoing it all again, would require ii == 2 always, and remove the vad_small field.
// - place all ar_wad in a separate array.  let PMeshIter keep pointer into it.
// - do the same for fl_matid and fr_matid
//
// Using shorts for flclw and all indices, separate matid array, and no
//  vad_small, we get 38n + 58m bytes for PMesh  (above is 56n + 92m bytes)

// So practically speaking (1999-05-21):
//  D3D mesh : 44n bytes
//  PM  mesh : 40n + 60m bytes

// *** Misc

namespace {

inline void attrib_ok(PmWedgeAttrib& a) {
  float len2 = mag2(a.normal);
  // Normal could be all-zero from MeshSimplify,
  //  either because it was zero in original model,
  //  or (less likely) if it became zero in simplification.
  // ABOVE IS NO LONGER TRUE:
  //  Now, MeshSimplify is such that normals are always non-zero.
  // First had the thresh at 1e-4f but numerical imprecision creeped in for large models like the banklamp.
  // const float thresh = 1e-4f;
  const float thresh = 1e-2f;
  if (!assertw(abs(len2) > 1e-6f)) return;
  assertx(abs(len2 - 1.f) < thresh);
}

inline int face_prediction(int fa, int fb, int ii) {
  ASSERTX(fa >= 0 || fb >= 0);
  return ii == 0 ? (fb >= 0 ? fb : fa) : (fa >= 0 ? fa : fb);
}

}  // namespace

// *** Attrib

void interp(PmVertexAttrib& a, const PmVertexAttrib& a1, const PmVertexAttrib& a2, float frac1) {
  a.point = interp(a1.point, a2.point, frac1);
}

void interp(PmWedgeAttrib& a, const PmWedgeAttrib& a1, const PmWedgeAttrib& a2, float frac1) {
  a.normal = ok_normalized(interp(a1.normal, a2.normal, frac1));
  a.rgb = interp(a1.rgb, a2.rgb, frac1);
  a.uv = interp(a1.uv, a2.uv, frac1);
}

void interp(PmSVertexAttrib& a, const PmSVertexAttrib& a1, const PmSVertexAttrib& a2, float frac1) {
  interp(a.v, a1.v, a2.v, frac1);
  interp(a.w, a1.w, a2.w, frac1);
}

void add(PmVertexAttrib& a, const PmVertexAttrib& a1, const PmVertexAttribD& ad) { a.point = a1.point + ad.dpoint; }

void sub(PmVertexAttrib& a, const PmVertexAttrib& a1, const PmVertexAttribD& ad) { a.point = a1.point - ad.dpoint; }

void diff(PmVertexAttribD& ad, const PmVertexAttrib& a1, const PmVertexAttrib& a2) { ad.dpoint = a1.point - a2.point; }

void add(PmWedgeAttrib& a, const PmWedgeAttrib& a1, const PmWedgeAttribD& ad) {
  a.normal = a1.normal + ad.dnormal;
  a.rgb = a1.rgb + ad.drgb;
  a.uv = a1.uv + ad.duv;
}

void add_zero(PmWedgeAttrib& a, const PmWedgeAttribD& ad) {
  a.normal = ad.dnormal;
  a.rgb = ad.drgb;
  a.uv = ad.duv;
}

void sub_noreflect(PmWedgeAttrib& a, const PmWedgeAttrib& abase, const PmWedgeAttribD& ad) {
  a.normal = abase.normal - ad.dnormal;
  a.rgb = abase.rgb - ad.drgb;
  a.uv = abase.uv - ad.duv;
}

void sub_reflect(PmWedgeAttrib& a, const PmWedgeAttrib& abase, const PmWedgeAttribD& ad) {
  // note: may have abase == a -> not really const
  const Vector& n = abase.normal;
  const Vector& d = ad.dnormal;
  // dr == -d + 2 * dot(d, n) * n
  // an = n + dr
  a.normal = -d + ((2.f) * dot(d, n) + 1.f) * n;
  a.rgb = abase.rgb - ad.drgb;
  a.uv = abase.uv - ad.duv;
}

void diff(PmWedgeAttribD& ad, const PmWedgeAttrib& a1, const PmWedgeAttrib& a2) {
  ad.dnormal = a1.normal - a2.normal;
  float a1rgb0 = a1.rgb[0], a1rgb1 = a1.rgb[1], a1rgb2 = a1.rgb[2];
  float a2rgb0 = a2.rgb[0], a2rgb1 = a2.rgb[1], a2rgb2 = a2.rgb[2];
  // Handle BIGFLOAT for use in Filterprog.
  if (a1rgb0 == BIGFLOAT) a1rgb0 = 0.f;
  if (a1rgb1 == BIGFLOAT) a1rgb1 = 0.f;
  if (a1rgb2 == BIGFLOAT) a1rgb2 = 0.f;
  if (a2rgb0 == BIGFLOAT) a2rgb0 = 0.f;
  if (a2rgb1 == BIGFLOAT) a2rgb1 = 0.f;
  if (a2rgb2 == BIGFLOAT) a2rgb2 = 0.f;
  ad.drgb[0] = a1rgb0 - a2rgb0;
  ad.drgb[1] = a1rgb1 - a2rgb1;
  ad.drgb[2] = a1rgb2 - a2rgb2;
  float a1uv0 = a1.uv[0], a1uv1 = a1.uv[1];
  float a2uv0 = a2.uv[0], a2uv1 = a2.uv[1];
  if (a1uv0 == BIGFLOAT) a1uv0 = 0.f;
  if (a1uv1 == BIGFLOAT) a1uv1 = 0.f;
  if (a2uv0 == BIGFLOAT) a2uv0 = 0.f;
  if (a2uv1 == BIGFLOAT) a2uv1 = 0.f;
  ad.duv[0] = a1uv0 - a2uv0;
  ad.duv[1] = a1uv1 - a2uv1;
}

int compare(const PmVertexAttrib& a1, const PmVertexAttrib& a2) { return compare(a1.point, a2.point); }

int compare(const PmVertexAttrib& a1, const PmVertexAttrib& a2, float tol) { return compare(a1.point, a2.point, tol); }

int compare(const PmWedgeAttrib& a1, const PmWedgeAttrib& a2) {
  if (int r = compare(a1.normal, a2.normal); r != 0) return r;
  if (int r = compare(a1.rgb, a2.rgb); r != 0) return r;
  if (int r = compare(a1.uv, a2.uv); r != 0) return r;
  return 0;
}

int compare(const PmWedgeAttrib& a1, const PmWedgeAttrib& a2, float tol) {
  if (int r = compare(a1.normal, a2.normal, tol); r != 0) return r;
  if (int r = compare(a1.rgb, a2.rgb, tol); r != 0) return r;
  if (int r = compare(a1.uv, a2.uv, tol); r != 0) return r;
  return 0;
}

// *** WMesh

void WMesh::read(std::istream& is, const PMeshInfo& pminfo) {
  assertx(!_vertices.num());
  _materials.read(is);
  int nvertices, nwedges, nfaces;
  {
    string line;
    assertx(my_getline(is, line));
    const char* s = line.c_str();
    s = assertx(after_prefix(s, "nvertices=")), nvertices = int_from_chars(s);
    s = assertx(after_prefix(s, " nwedges=")), nwedges = int_from_chars(s);
    s = assertx(after_prefix(s, " nfaces=")), nfaces = int_from_chars(s);
    assert_no_more_chars(s);
  }
  _vertices.init(nvertices);
  _wedges.init(nwedges);
  _faces.init(nfaces);
  for_int(v, nvertices) assertx(read_binary_std(is, _vertices[v].attrib.point.view()));
  // PM base_mesh has property that _wedges[w].vertex == w for w < _vertices.num().
  // This could be exploited. optimize.
  const int nrgb = pminfo._read_version < 2 ? 2 : pminfo._has_rgb * 3 + pminfo._has_uv * 2;
  Array<float> buf(3 + nrgb);
  for_int(w, nwedges) {
    assertx(read_binary_std(is, ArView(_wedges[w].vertex)));
    assertx(_vertices.ok(_wedges[w].vertex));
    assertx(read_binary_std(is, buf));
    Vector& nor = _wedges[w].attrib.normal;
    A3dColor& rgb = _wedges[w].attrib.rgb;
    Uv& uv = _wedges[w].attrib.uv;
    const float* p = buf.data();
    for_int(c, 3) nor[c] = *p++;
    if (pminfo._has_rgb) {
      for_int(c, 3) rgb[c] = *p++;
    } else {
      fill(rgb, 0.f);
    }
    if (pminfo._has_uv) {
      for_int(c, 2) uv[c] = *p++;
    } else {
      fill(uv, 0.f);
      if (pminfo._read_version < 2) p += 2;
    }
    ASSERTX(p == buf.end());
  }
  for_int(f, nfaces) {
    assertx(read_binary_std(is, _faces[f].wedges.view()));
    ushort lmatid;
    assertx(read_binary_std(is, ArView(lmatid)));
    int& matid = _faces[f].attrib.matid;
    matid = lmatid;
    assertx(_materials.ok(matid));
  }
}

void WMesh::write(std::ostream& os, const PMeshInfo& pminfo) const {
  _materials.write(os);
  os << "nvertices=" << _vertices.num() << " nwedges=" << _wedges.num() << " nfaces=" << _faces.num() << '\n';
  for_int(v, _vertices.num()) write_binary_std(os, _vertices[v].attrib.point.view());
  for_int(w, _wedges.num()) {
    write_binary_std(os, ArView(_wedges[w].vertex));
    if (1) write_binary_std(os, _wedges[w].attrib.normal.view());
    if (pminfo._has_rgb) write_binary_std(os, _wedges[w].attrib.rgb.view());
    if (pminfo._has_uv) write_binary_std(os, _wedges[w].attrib.uv.view());
  }
  for_int(f, _faces.num()) {
    write_binary_std(os, _faces[f].wedges.view());
    int matid = _faces[f].attrib.matid & ~AWMesh::k_Face_visited_mask;
    ushort lmatid = narrow_cast<ushort>(matid);
    write_binary_std(os, ArView(lmatid));
  }
  assertx(os);
}

inline Pixel pack_color(const A3dColor& col) { return Vector4(col[0], col[1], col[2], 1.f).pixel(); }

void WMesh::write_ply(std::ostream& os, const PMeshInfo& pminfo, bool binary) const {
  const bool uv_in_vertex = true;  // Else in face, which is less compact.
  os << "ply\n";
  // Big Endian is network order, compatible with write_binary_std(...);
  os << "format " << (binary ? "binary_big_endian" : "ascii") << " 1.0\n";
  os << "comment PM_nvertices=" << _vertices.num() << "\n";
  os << "element vertex " << _wedges.num() << "\n";
  os << "property float x\n";
  os << "property float y\n";
  os << "property float z\n";
  os << "property float nx\n";
  os << "property float ny\n";
  os << "property float nz\n";
  if (pminfo._has_rgb) {
    os << "property uchar red\n";
    os << "property uchar green\n";
    os << "property uchar blue\n";
  }
  if (pminfo._has_uv && uv_in_vertex) {
    os << "property float s\n";  // Or "texcoord_u".
    os << "property float t\n";  // Or "texcoord_v".
  }
  os << "element face " << _faces.num() << "\n";
  os << "property list uchar int vertex_indices\n";
  if (pminfo._has_uv && !uv_in_vertex) os << "property list uchar float texcoord\n";
  os << "end_header\n";
  if (binary) {
    for_int(w, _wedges.num()) {
      const int v = _wedges[w].vertex;
      write_binary_std(os, _vertices[v].attrib.point.const_view());
      write_binary_std(os, _wedges[w].attrib.normal.const_view());
      if (pminfo._has_rgb) write_binary_raw(os, pack_color(_wedges[w].attrib.rgb).head<3>().const_view());
      if (pminfo._has_uv && uv_in_vertex) write_binary_std(os, _wedges[w].attrib.uv.const_view());
    }
    for_int(f, _faces.num()) {
      write_binary_raw(os, V(uchar(3)).const_view());
      write_binary_std(os, _faces[f].wedges.const_view());
      if (pminfo._has_uv && !uv_in_vertex) {
        write_binary_raw(os, V(uchar(6)).const_view());
        for_int(j, 3) write_binary_std(os, _wedges[_faces[f].wedges[j]].attrib.uv.const_view());
      }
    }
  } else {
    const auto write_ascii = [&](const auto& v, bool space = true) {
      for_int(c, v.Num) os << (space || c > 0 ? " " : "") << v[c];
    };
    for_int(w, _wedges.num()) {
      const int v = _wedges[w].vertex;
      write_ascii(_vertices[v].attrib.point, false);
      write_ascii(_wedges[w].attrib.normal);
      if (pminfo._has_rgb) write_ascii(convert<int>(pack_color(_wedges[w].attrib.rgb).head<3>()));
      if (pminfo._has_uv && uv_in_vertex) write_ascii(_wedges[w].attrib.uv);
      os << "\n";
    }
    for_int(f, _faces.num()) {
      os << "3";
      write_ascii(_faces[f].wedges);
      os << "\n";
      if (pminfo._has_uv && !uv_in_vertex) {
        os << "6";
        for_int(j, 3) write_ascii(_wedges[_faces[f].wedges[j]].attrib.uv);
        os << "\n";
      }
    }
  }
}

GMesh WMesh::extract_gmesh(const PMeshInfo& pminfo) const {
  GMesh gmesh;
  const int k_no_ref = -1, k_multiple_refs = -2;
  Array<int> wedgeref(_vertices.num(), k_no_ref);
  for_int(w, _wedges.num()) {
    int v = _wedges[w].vertex;
    wedgeref[v] = wedgeref[v] == k_no_ref ? w : k_multiple_refs;
  }
  string str;
  for_int(v, _vertices.num()) {
    Vertex gv = gmesh.create_vertex();
    ASSERTX(gmesh.vertex_id(gv) == v + 1);
    gmesh.set_point(gv, _vertices[v].attrib.point);
    int wr = wedgeref[v];
    assertx(wr != k_no_ref);
    if (wr != k_multiple_refs) {
      gmesh.update_string(gv, "wid", csform(str, "%d", wr + 1));
      const Vector& nor = _wedges[wr].attrib.normal;
      gmesh.update_string(gv, "normal", csform_vec(str, nor));
      const A3dColor& rgb = _wedges[wr].attrib.rgb;
      if (pminfo._has_rgb) gmesh.update_string(gv, "rgb", csform_vec(str, rgb));
      const Uv& uv = _wedges[wr].attrib.uv;
      if (pminfo._has_uv) gmesh.update_string(gv, "uv", csform_vec(str, uv));
    }
  }
  Array<Vertex> gva;
  for_int(f, _faces.num()) {
    gva.init(0);
    for_int(j, 3) {
      int w = _faces[f].wedges[j];
      int v = _wedges[w].vertex;
      gva.push(gmesh.id_vertex(v + 1));
    }
    Face gf = gmesh.create_face(gva);
    int matid = _faces[f].attrib.matid & ~AWMesh::k_Face_visited_mask;
    gmesh.set_string(gf, _materials.get(matid).c_str());
    for_int(j, 3) {
      int w = _faces[f].wedges[j];
      int v = _wedges[w].vertex;
      if (wedgeref[v] != k_multiple_refs) continue;
      Corner gc = gmesh.corner(gva[j], gf);
      gmesh.update_string(gc, "wid", csform(str, "%d", w + 1));
      const Vector& nor = _wedges[w].attrib.normal;
      gmesh.update_string(gc, "normal", csform_vec(str, nor));
      const A3dColor& rgb = _wedges[w].attrib.rgb;
      if (pminfo._has_rgb) gmesh.update_string(gc, "rgb", csform_vec(str, rgb));
      const Uv& uv = _wedges[w].attrib.uv;
      if (pminfo._has_uv) gmesh.update_string(gc, "uv", csform_vec(str, uv));
    }
  }
  return gmesh;
}

void WMesh::ok() const {
  for_int(w, _wedges.num()) {
    int v = _wedges[w].vertex;
    assertx(_vertices.ok(v));
  }
  for_int(f, _faces.num()) {
    Set<int> setw;
    for_int(j, 3) {
      int w = _faces[f].wedges[j];
      assertx(_wedges.ok(w));
      assertx(setw.add(w));
    }
    int matid = _faces[f].attrib.matid & ~AWMesh::k_Face_visited_mask;
    assertx(_materials.ok(matid));
  }
}

Array<int> WMesh::WMesh::gather_someface() const {
  Array<int> someface(_vertices.num());
  for_int(f, _faces.num()) for (const int v : face_vertices(f)) someface[v] = f;
  return someface;
}

// *** Vsplit

void Vsplit::read(std::istream& is, const PMeshInfo& pminfo) {
  assertx(read_binary_std(is, ArView(flclw)));
  assertx(read_binary_std(is, ArView(vlr_offset1)));
  assertx(read_binary_std(is, ArView(code)));
  if (code & (FLN_MASK | FRN_MASK)) {
    assertx(read_binary_std(is, ArView(fl_matid)));
    assertx(read_binary_std(is, ArView(fr_matid)));
  } else {
    fl_matid = 0;
    fr_matid = 0;
  }
  const int max_nwa = 6;
  const int max_buf_size = 6 + max_nwa * (3 + 3 + 2) + 2;
  int nwa = expected_wad_num(pminfo);
  ASSERTX(nwa <= max_nwa);
  const int nrgb = pminfo._has_rgb * 3 + pminfo._has_uv * 2;
  int wadlength = 3 + nrgb;
  Vec<float, max_buf_size> buf;
  const int bufn = 6 + nwa * wadlength + 2 * pminfo._has_resid;
  assertx(bufn <= buf.num());
  assertx(read_binary_std(is, buf.head(bufn)));
  Vector& v_large = vad_large.dpoint;
  for_int(c, 3) v_large[c] = buf[0 + c];
  Vector& v_small = vad_small.dpoint;
  for_int(c, 3) v_small[c] = buf[3 + c];
  ar_wad.init(nwa);
  for_int(i, nwa) {
    int bufw = 6 + i * wadlength;
    Vector& nor = ar_wad[i].dnormal;
    A3dColor& rgb = ar_wad[i].drgb;
    Uv& uv = ar_wad[i].duv;
    float* p = &buf[bufw];
    for_int(c, 3) nor[c] = *p++;
    if (pminfo._has_rgb) {
      for_int(c, 3) rgb[c] = *p++;
    } else {
      fill(rgb, 0.f);
    }
    if (pminfo._has_uv) {
      for_int(c, 2) uv[c] = *p++;
    } else {
      fill(uv, 0.f);
    }
  }
  if (pminfo._has_resid) {
    float* p = &buf[6 + nwa * wadlength + 0];
    resid_uni = *p++;
    resid_dir = *p++;
  } else {
    resid_uni = 0.f;
    resid_dir = 0.f;
  }
}

void Vsplit::write(std::ostream& os, const PMeshInfo& pminfo) const {
  write_binary_std(os, ArView(flclw));
  write_binary_std(os, ArView(vlr_offset1));
  write_binary_std(os, ArView(code));
  if (code & (FLN_MASK | FRN_MASK)) {
    write_binary_std(os, ArView(fl_matid));
    write_binary_std(os, ArView(fr_matid));
  }
  write_binary_std(os, vad_large.dpoint.view());
  write_binary_std(os, vad_small.dpoint.view());
  for (const PmWedgeAttribD& wad : ar_wad) {
    if (1) write_binary_std(os, wad.dnormal.view());
    if (pminfo._has_rgb) write_binary_std(os, wad.drgb.view());
    if (pminfo._has_uv) write_binary_std(os, wad.duv.view());
  }
  if (pminfo._has_resid) {
    write_binary_std(os, ArView(resid_uni));
    write_binary_std(os, ArView(resid_dir));
  }
}

void Vsplit::ok() const {
  // assertx(ar_wad.num() == expected_wad_num(now_missing_pminfo));
}

int Vsplit::expected_wad_num(const PMeshInfo& pminfo) const {
  if (pminfo._has_wad2) return 2;
  // optimize: construct static const lookup table on (S_MASK | T_MASK).
  int nwa = 0;
  if (1) {
    bool nt = !(code & T_LSAME);
    bool ns = !(code & S_LSAME);
    nwa += nt && ns ? 2 : 1;
  }
  if (vlr_offset1 > 1) {
    bool nt = !(code & T_RSAME);
    bool ns = !(code & S_RSAME);
    if (nt && ns) {
      if (!(code & T_CSAME)) nwa++;
      if (!(code & S_CSAME)) nwa++;
    } else {
      int ii = (code & II_MASK) >> II_SHIFT;
      switch (ii) {
        case 2:
          if (!(code & T_CSAME)) nwa++;
          break;
        case 0:
          if (!(code & S_CSAME)) nwa++;
          break;
        case 1:
          if (!(code & T_CSAME) || !(code & S_CSAME)) nwa++;
          break;
        default: assertnever("");
      }
    }
  }
  if (code & L_NEW) nwa++;
  if (code & R_NEW) nwa++;
  return nwa;
}

// *** AWMesh

int AWMesh::most_clw_face(int v, int f) const {
  int ff = f, lastf;
  do {
    lastf = ff;
    ff = _fnei[ff].faces[mod3(get_jvf(v, ff) + 2)];
  } while (ff != k_undefined && ff != f);
  return (ff == k_undefined) ? lastf : k_undefined;
}

int AWMesh::most_ccw_face(int v, int f) const {
  int ff = f, lastf;
  do {
    lastf = ff;
    ff = _fnei[ff].faces[mod3(get_jvf(v, ff) + 1)];
  } while (ff != k_undefined && ff != f);
  return (ff == k_undefined) ? lastf : k_undefined;
}

bool AWMesh::is_boundary(int v, int f) const { return most_ccw_face(v, f) != k_undefined; }

void AWMesh::read(std::istream& is, const PMeshInfo& pminfo) {
  WMesh::read(is, pminfo);
  construct_adjacency();
}

void AWMesh::write(std::ostream& os, const PMeshInfo& pminfo) const { WMesh::write(os, pminfo); }

void AWMesh::apply_vsplit(const Vsplit& vspl, const PMeshInfo& pminfo, Ancestry* ancestry) {
  // SHOW("**vsplit");
  // Sanity checks
  ASSERTX(_faces.ok(vspl.flclw));
  const bool isl = true;
  const bool isr = vspl.vlr_offset1 > 1;
  // Allocate space for new faces now, since ar_pwedges points into _faces array.
  _faces.add(isr ? 2 : 1), _fnei.add(isr ? 2 : 1);  // !remember _fnei
  // Get vertices, faces, and wedges in neighborhood.
  int vs;
  unsigned code = vspl.code;
  int ii = (code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
  int vs_index = (code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
  int flccw, flclw;                // either (not both) may be k_undefined
  int frccw, frclw;                // either (or both) may be k_undefined
  int wlccw, wlclw, wrccw, wrclw;  // == k_undefined if faces do not exist
  int jlccw, jlclw, jrccw, jrclw;  // only defined if faces exist
  dummy_init(jlccw, jlclw, jrccw, jrclw);
  if (k_debug) jlccw = jlclw = jrccw = jrclw = std::numeric_limits<int>::max();
  PArray<int*, 10> ar_pwedges;
  if (vspl.vlr_offset1 == 0) {
    // Extremely rare case when flclw does not exist.
    flclw = k_undefined;
    wlclw = k_undefined;
    flccw = vspl.flclw;
    jlccw = vs_index;
    wlccw = _faces[flccw].wedges[jlccw];
    vs = _wedges[wlccw].vertex;
    frccw = k_undefined;
    frclw = k_undefined;
    wrccw = k_undefined;
    wrclw = k_undefined;
  } else {
    flclw = vspl.flclw;
    jlclw = vs_index;
    int* pwlclw = &_faces[flclw].wedges[jlclw];
    wlclw = *pwlclw;
    vs = _wedges[wlclw].vertex;
    flccw = _fnei[flclw].faces[mod3(jlclw + 1)];
    if (flccw == k_undefined) {
      wlccw = k_undefined;
    } else {
      jlccw = get_jvf(vs, flccw);
      wlccw = _faces[flccw].wedges[jlccw];
    }
    if (!isr) {
      frccw = k_undefined;
      frclw = k_undefined;
      wrccw = k_undefined;
      wrclw = k_undefined;
      ar_pwedges.push(pwlclw);
      // Rotate around and record all wedges CLW from wlclw.
      int j0 = jlclw;
      int f = flclw;
      for (;;) {
        f = _fnei[f].faces[mod3(j0 + 2)];
        if (f < 0) break;
        ASSERTX(f != flclw && f != flccw);
        j0 = get_jvf(vs, f);
        ar_pwedges.push(&_faces[f].wedges[j0]);
      }
    } else {
      ar_pwedges.init(vspl.vlr_offset1 - 1);
      ar_pwedges[0] = pwlclw;
      // Rotate around the first x - 1 faces.
      int j0 = jlclw;
      int f = flclw;
      for_int(count, vspl.vlr_offset1 - 2) {
        f = _fnei[f].faces[mod3(j0 + 2)];
        ASSERTX(f >= 0 && f != flclw && f != flccw);
        j0 = get_jvf(vs, f);
        ar_pwedges[count + 1] = &_faces[f].wedges[j0];
      }
      frccw = f;
      // On the last face, find adjacent faces.
      jrccw = j0;
      wrccw = _faces[frccw].wedges[jrccw];
      frclw = _fnei[frccw].faces[mod3(j0 + 2)];
      if (frclw == k_undefined) {
        wrclw = k_undefined;
      } else {
        jrclw = get_jvf(vs, frclw);
        wrclw = _faces[frclw].wedges[jrclw];
      }
    }
  }
  ASSERTX(flccw < 0 || jlccw == get_jvf(vs, flccw));
  ASSERTX(flclw < 0 || jlclw == get_jvf(vs, flclw));
  ASSERTX(frccw < 0 || jrccw == get_jvf(vs, frccw));
  ASSERTX(frclw < 0 || jrclw == get_jvf(vs, frclw));
  // Add a new vertex.
  int vt = _vertices.add(1);
  // Check equivalence of wedges across (vs, vl) and (vs, vr)
#if defined(HH_DEBUG)
  {
    bool thru_l = isl && (code & Vsplit::S_LSAME) && (code & Vsplit::T_LSAME);
    bool thru_r = isr && (code & Vsplit::S_RSAME) && (code & Vsplit::T_RSAME);
    bool both_f_l = isl && flccw >= 0 && flclw >= 0;
    bool both_f_r = isr && frccw >= 0 && frclw >= 0;
    bool all_same = both_f_l && both_f_r && wlccw == wlclw && wrccw == wrclw && wlccw == wrccw;
    if (both_f_l) assertx((wlccw == wlclw) == thru_l || (!thru_l && all_same && thru_r));
    if (both_f_r) assertx((wrccw == wrclw) == thru_r || (!thru_r && all_same && thru_l));
    // This indicates constraints between S_MASK, T_MASK, and
    //  equivalence of corners in M^i.
    // Could predict S_MASK from ii,
    //  but in practice Huffman coding of
    //   (II_MASK | S_MASK | T_MASK | L_MASK | R_MASK) symbol
    //  should take care of that.
    if (!isr) {
      assertx((code & Vsplit::S_RSAME) && (code & Vsplit::T_RSAME));
      assertx(!(code & Vsplit::S_CSAME) && !(code & Vsplit::T_CSAME));
    }
  }
#endif
  // Save current number of wedges if ancestry.
  int onumwedges;
  dummy_init(onumwedges);  // defined if ancestry
  if (k_debug) onumwedges = std::numeric_limits<int>::max();
  if (ancestry) onumwedges = _wedges.num();
  // First un-share wedges around vt (may be gap on top).  May modify wlclw and wrccw!
  int wnl = k_undefined, wnr = k_undefined;
  int iil = 0, iir = ar_pwedges.num() - 1;
  if (isl && wlclw == wlccw) {  // first go clw.
    if (1) {
      wnl = _wedges.add(1);
      _wedges[wnl].vertex = vt;
      _wedges[wnl].attrib = _wedges[wlccw].attrib;
    }
    wlclw = wnl;  // has been changed
    ASSERTX(*ar_pwedges[iil] == wlccw);
    for (;;) {
      *ar_pwedges[iil] = wnl;
      iil++;
      if (iil > iir) {
        wrccw = wnl;  // has been changed
        break;
      }
      if (*ar_pwedges[iil] != wlccw) break;
    }
  }
  if (isr && wrccw == wrclw) {  // now go ccw from other side.
    if (wrclw == wlccw && wnl >= 0) {
      wnr = wnl;
    } else {
      wnr = _wedges.add(1);
      _wedges[wnr].vertex = vt;
      _wedges[wnr].attrib = _wedges[wrclw].attrib;
    }
    wrccw = wnr;  // has been changed
    ASSERTX(*ar_pwedges[iir] == wrclw);
    for (;;) {
      *ar_pwedges[iir] = wnr;
      --iir;
      if (iir < iil) {
        if (iir < 0) wlclw = wnr;  // has been changed
        break;
      }
      if (*ar_pwedges[iir] != wrclw) break;
    }
  }
  // Add other new wedges and record wedge ancestries
  int wvtfl, wvtfr;
  if (!isr) {
    wvtfr = k_undefined;
    switch (code & Vsplit::T_MASK) {
      case Vsplit::T_LSAME | Vsplit::T_RSAME: wvtfl = wlclw; break;
      case Vsplit::T_RSAME:
        wvtfl = _wedges.add(1);
        _wedges[wvtfl].vertex = vt;
        break;
      default: assertnever("");
    }
    ASSERTX(wvtfl >= 0);
  } else {
    switch (code & Vsplit::T_MASK) {
      case Vsplit::T_LSAME | Vsplit::T_RSAME | Vsplit::T_CSAME:
        wvtfl = wlclw;
        wvtfr = wrccw;
        ASSERTX(wvtfl == wvtfr);
        break;
      case Vsplit::T_LSAME | Vsplit::T_RSAME:
        wvtfl = wlclw;
        wvtfr = wrccw;
        ASSERTX(wvtfl != wvtfr);
        break;
      case Vsplit::T_LSAME | Vsplit::T_CSAME:
        wvtfl = wlclw;
        wvtfr = wvtfl;
        break;
      case Vsplit::T_RSAME | Vsplit::T_CSAME:
        wvtfl = wrccw;
        wvtfr = wvtfl;
        break;
      case Vsplit::T_LSAME:
        wvtfl = wlclw;
        wvtfr = _wedges.add(1);
        _wedges[wvtfr].vertex = vt;
        break;
      case Vsplit::T_RSAME:
        wvtfl = _wedges.add(1);
        _wedges[wvtfl].vertex = vt;
        wvtfr = wrccw;
        break;
      case Vsplit::T_CSAME:
        wvtfl = _wedges.add(1);
        _wedges[wvtfl].vertex = vt;
        wvtfr = wvtfl;
        break;
      case 0:
        wvtfl = _wedges.add(1);
        _wedges[wvtfl].vertex = vt;
        wvtfr = _wedges.add(1);
        _wedges[wvtfr].vertex = vt;
        break;
      default: assertnever("");
    }
    ASSERTX(wvtfl >= 0 && wvtfr >= 0);
  }
  int wvsfl, wvsfr;
  if (!isr) {
    wvsfr = k_undefined;
    switch (code & Vsplit::S_MASK) {
      case Vsplit::S_LSAME | Vsplit::S_RSAME: wvsfl = wlccw; break;
      case Vsplit::S_RSAME:
        wvsfl = _wedges.add(1);
        _wedges[wvsfl].vertex = vs;
        break;
      default: assertnever("");
    }
    ASSERTX(wvsfl >= 0);
  } else {
    switch (code & Vsplit::S_MASK) {
      case Vsplit::S_LSAME | Vsplit::S_RSAME | Vsplit::S_CSAME:
        wvsfl = wlccw;
        wvsfr = wrclw;
        ASSERTX(wvsfl == wvsfr);
        break;
      case Vsplit::S_LSAME | Vsplit::S_RSAME:
        wvsfl = wlccw;
        wvsfr = wrclw;
        ASSERTX(wvsfl != wvsfr);
        break;
      case Vsplit::S_LSAME | Vsplit::S_CSAME:
        wvsfl = wlccw;
        wvsfr = wvsfl;
        break;
      case Vsplit::S_RSAME | Vsplit::S_CSAME:
        wvsfl = wrclw;
        wvsfr = wvsfl;
        break;
      case Vsplit::S_LSAME:
        wvsfl = wlccw;
        wvsfr = _wedges.add(1);
        _wedges[wvsfr].vertex = vs;
        break;
      case Vsplit::S_RSAME:
        wvsfl = _wedges.add(1);
        _wedges[wvsfl].vertex = vs;
        wvsfr = wrclw;
        break;
      case Vsplit::S_CSAME:
        wvsfl = _wedges.add(1);
        _wedges[wvsfl].vertex = vs;
        wvsfr = wvsfl;
        break;
      case 0:
        wvsfl = _wedges.add(1);
        _wedges[wvsfl].vertex = vs;
        wvsfr = _wedges.add(1);
        _wedges[wvsfr].vertex = vs;
        break;
      default: assertnever("");
    }
    ASSERTX(wvsfl >= 0 && wvsfr >= 0);
  }
  int wvlfl, wvrfr;
  if (isl) {
    switch (code & Vsplit::L_MASK) {
      case Vsplit::L_ABOVE: wvlfl = _faces[flclw].wedges[mod3(jlclw + 2)]; break;
      case Vsplit::L_BELOW: wvlfl = _faces[flccw].wedges[mod3(jlccw + 1)]; break;
      case Vsplit::L_NEW: {
        wvlfl = _wedges.add(1);
        int vl = _wedges[(flclw >= 0 ? _faces[flclw].wedges[mod3(jlclw + 2)] : _faces[flccw].wedges[mod3(jlccw + 1)])]
                     .vertex;
        _wedges[wvlfl].vertex = vl;
        break;
      }
      default: assertnever("");
    }
  }
  if (!isr) {
    wvrfr = k_undefined;
  } else {
    switch (code & Vsplit::R_MASK) {
      case Vsplit::R_ABOVE: wvrfr = _faces[frccw].wedges[mod3(jrccw + 1)]; break;
      case Vsplit::R_BELOW: wvrfr = _faces[frclw].wedges[mod3(jrclw + 2)]; break;
      case Vsplit::R_NEW: {
        wvrfr = _wedges.add(1);
        int vr = _wedges[_faces[frccw].wedges[mod3(jrccw + 1)]].vertex;
        _wedges[wvrfr].vertex = vr;
        break;
      }
      default: assertnever("");
    }
  }
  // Add 1 or 2 faces, and update adjacency information.
  int fl, fr;
  if (isr) {
    fr = _faces.num() - 1;
    fl = fr - 1;
  } else {
    fr = k_undefined;
    fl = _faces.num() - 1;
  }
  if (isl) {
    _faces[fl].wedges[0] = wvsfl;
    _faces[fl].wedges[1] = wvtfl;
    _faces[fl].wedges[2] = wvlfl;
    if (flccw >= 0) _fnei[flccw].faces[mod3(jlccw + 2)] = fl;
    if (flclw >= 0) _fnei[flclw].faces[mod3(jlclw + 1)] = fl;
    // could use L_MASK instead of ii for prediction instead.
    _faces[fl].attrib.matid =
        (((code & Vsplit::FLN_MASK) ? vspl.fl_matid : _faces[face_prediction(flclw, flccw, ii)].attrib.matid) |
         _cur_frame_mask);
    ASSERTX(_materials.ok(_faces[fl].attrib.matid & ~k_Face_visited_mask));
    _fnei[fl].faces[0] = flclw;
    _fnei[fl].faces[1] = flccw;
    _fnei[fl].faces[2] = fr;
  }
  if (isr) {
    _faces[fr].wedges[0] = wvsfr;
    _faces[fr].wedges[1] = wvrfr;
    _faces[fr].wedges[2] = wvtfr;
    if (frccw >= 0) _fnei[frccw].faces[mod3(jrccw + 2)] = fr;
    if (frclw >= 0) _fnei[frclw].faces[mod3(jrclw + 1)] = fr;
    _faces[fr].attrib.matid =
        (((code & Vsplit::FRN_MASK) ? vspl.fr_matid : _faces[face_prediction(frccw, frclw, ii)].attrib.matid) |
         _cur_frame_mask);
    ASSERTX(_materials.ok(_faces[fr].attrib.matid & ~k_Face_visited_mask));
    _fnei[fr].faces[0] = frccw;
    _fnei[fr].faces[1] = fl;
    _fnei[fr].faces[2] = frclw;
  }
  // Update wedge vertices.
  for (; iil <= iir; iil++) {
    int w = *ar_pwedges[iil];
    ASSERTX(_wedges.ok(w));
    _wedges[w].vertex = vt;
  }
  ASSERTX(!isl || _wedges[_faces[fl].wedges[0]].vertex == vs);
  ASSERTX(!isr || _wedges[_faces[fr].wedges[0]].vertex == vs);
  ASSERTX(!isl || _wedges[_faces[fl].wedges[1]].vertex == vt);
  ASSERTX(!isr || _wedges[_faces[fr].wedges[2]].vertex == vt);
  // Update vertex attributes.
  {
    PmVertexAttrib& va_s = _vertices[vs].attrib;
    PmVertexAttrib& va_t = _vertices[vt].attrib;
    switch (ii) {
      case 2:
        add(va_t, va_s, vspl.vad_large);
        add(va_s, va_s, vspl.vad_small);
        break;
      case 0:
        add(va_t, va_s, vspl.vad_small);
        add(va_s, va_s, vspl.vad_large);
        break;
      case 1: {
        PmVertexAttrib va_m;
        add(va_m, va_s, vspl.vad_small);
        add(va_t, va_m, vspl.vad_large);
        sub(va_s, va_m, vspl.vad_large);
        break;
      }
      default: assertnever("");
    }
  }
  // Update wedge attributes.
  PmWedgeAttrib awvtfr, awvsfr;
  dummy_init(awvtfr, awvsfr);
  int lnum = 0;
  if (pminfo._has_wad2) {
    assertx(vspl.ar_wad.num() == 2);
    int ns = !(code & Vsplit::S_LSAME);
    if (ns) _wedges[wvsfl].attrib = _wedges[wvtfl].attrib;
    add(_wedges[wvtfl].attrib, _wedges[wvsfl].attrib, vspl.ar_wad[0]);
    add(_wedges[wvsfl].attrib, _wedges[wvsfl].attrib, vspl.ar_wad[1]);
    assertx(!ancestry);
    goto GOTO_VSPLIT_WAD2;
  }
  if (isr) {
    awvtfr = _wedges[wvtfr].attrib;  // backup for isr
    awvsfr = _wedges[wvsfr].attrib;  // backup for isr
  }
  if (isl) {
    bool nt = !(code & Vsplit::T_LSAME);
    bool ns = !(code & Vsplit::S_LSAME);
    if (nt && ns) {
      add_zero(_wedges[wvtfl].attrib, vspl.ar_wad[lnum++]);
      add_zero(_wedges[wvsfl].attrib, vspl.ar_wad[lnum++]);
    } else {
      switch (ii) {
        case 2:
          if (ns) _wedges[wvsfl].attrib = _wedges[wvtfl].attrib;
          // remove !ns?: test below?
          add(_wedges[wvtfl].attrib, _wedges[!ns ? wvsfl : wvtfl].attrib, vspl.ar_wad[lnum++]);
          break;
        case 0:
          if (nt) _wedges[wvtfl].attrib = _wedges[wvsfl].attrib;
          add(_wedges[wvsfl].attrib, _wedges[!nt ? wvtfl : wvsfl].attrib, vspl.ar_wad[lnum++]);
          break;
        case 1: {
          const PmWedgeAttribD& wad = vspl.ar_wad[lnum];
          if (!ns) {
            const PmWedgeAttrib& wabase = _wedges[wvsfl].attrib;
            add(_wedges[wvtfl].attrib, wabase, wad);
            sub_reflect(_wedges[wvsfl].attrib, wabase, wad);
          } else {
            const PmWedgeAttrib& wabase = _wedges[wvtfl].attrib;
            sub_reflect(_wedges[wvsfl].attrib, wabase, wad);
            add(_wedges[wvtfl].attrib, wabase, wad);
          }
          lnum++;
          break;
        }
        default: assertnever("");
      }
    }
  }
  if (isr) {
    bool nt = !(code & Vsplit::T_RSAME);
    bool ns = !(code & Vsplit::S_RSAME);
    bool ut = !(code & Vsplit::T_CSAME);
    bool us = !(code & Vsplit::S_CSAME);
    if (nt && ns) {
      if (ut) add_zero(_wedges[wvtfr].attrib, vspl.ar_wad[lnum++]);
      if (us) add_zero(_wedges[wvsfr].attrib, vspl.ar_wad[lnum++]);
    } else {
      switch (ii) {
        case 2:
          if (us && ns) _wedges[wvsfr].attrib = awvtfr;
          if (ut) add(_wedges[wvtfr].attrib, (!ns ? awvsfr : awvtfr), vspl.ar_wad[lnum++]);
          break;
        case 0:
          if (ut && nt) _wedges[wvtfr].attrib = awvsfr;
          if (us) add(_wedges[wvsfr].attrib, (!nt ? awvtfr : awvsfr), vspl.ar_wad[lnum++]);
          break;
        case 1: {
          if (!ns) {
            const PmWedgeAttrib& wabase = awvsfr;
            if (ut) add(_wedges[wvtfr].attrib, wabase, vspl.ar_wad[lnum]);
            if (us) sub_reflect(_wedges[wvsfr].attrib, wabase, vspl.ar_wad[lnum]);
          } else {
            const PmWedgeAttrib& wabase = awvtfr;
            if (us) sub_reflect(_wedges[wvsfr].attrib, wabase, vspl.ar_wad[lnum]);
            if (ut) add(_wedges[wvtfr].attrib, wabase, vspl.ar_wad[lnum]);
          }
          if (ut || us) lnum++;
          break;
        }
        default: assertnever("");
      }
    }
  }
  if (code & Vsplit::L_NEW) add_zero(_wedges[wvlfl].attrib, vspl.ar_wad[lnum++]);
  if (code & Vsplit::R_NEW) add_zero(_wedges[wvrfr].attrib, vspl.ar_wad[lnum++]);
  ASSERTX(lnum == vspl.ar_wad.num());
  ASSERTX(!isl || (attrib_ok(_wedges[wvtfl].attrib), true));
  ASSERTX(!isl || (attrib_ok(_wedges[wvsfl].attrib), true));
  ASSERTX(!isl || (attrib_ok(_wedges[wvlfl].attrib), true));
  ASSERTX(!isr || (attrib_ok(_wedges[wvtfr].attrib), true));
  ASSERTX(!isr || (attrib_ok(_wedges[wvsfr].attrib), true));
  ASSERTX(!isr || (attrib_ok(_wedges[wvrfr].attrib), true));
  // Deal with ancestry
  if (ancestry) apply_vsplit_ancestry(ancestry, vs, isr, onumwedges, code, wvlfl, wvrfr, wvsfl, wvsfr, wvtfl, wvtfr);
    // Final check.
#if defined(HH_DEBUG)
  {
    int wvsflo = flccw == k_undefined ? k_undefined : get_wvf(vs, flccw);
    int wvsfro = frclw == k_undefined ? k_undefined : get_wvf(vs, frclw);
    assertx((code & Vsplit::S_MASK) ==
            unsigned((wvsfl == wvsflo ? Vsplit::S_LSAME : 0u) | (wvsfr == wvsfro ? Vsplit::S_RSAME : 0u) |
                     (wvsfl == wvsfr ? Vsplit::S_CSAME : 0u)));
    int wvtflo = flclw == k_undefined ? k_undefined : get_wvf(vt, flclw);
    int wvtfro = frccw == k_undefined ? k_undefined : get_wvf(vt, frccw);
    assertx((code & Vsplit::T_MASK) ==
            unsigned((wvtfl == wvtflo ? Vsplit::T_LSAME : 0u) | (wvtfr == wvtfro ? Vsplit::T_RSAME : 0u) |
                     (wvtfl == wvtfr ? Vsplit::T_CSAME : 0u)));
  }
#endif
GOTO_VSPLIT_WAD2:
  void();  // empty statement

  // ok();
}

void AWMesh::apply_vsplit_private(const Vsplit& vspl, const PMeshInfo& pminfo, Ancestry* ancestry) {
  apply_vsplit(vspl, pminfo, ancestry);
}

void AWMesh::apply_vsplit_ancestry(Ancestry* ancestry, int vs, bool isr, int onumwedges, int code, int wvlfl,
                                   int wvrfr, int wvsfl, int wvsfr, int wvtfl, int wvtfr) {
  const bool isl = true;
  // Vertex ancestry.
  {
    Array<PmVertexAttrib>& vancestry = ancestry->_vancestry;
    int vi = vancestry.add(1);
    // ASSERTX(vi == vt);
    vancestry[vi] = vancestry[vs];
  }
  // Wedge ancestry.
  Array<PmWedgeAttrib>& wancestry = ancestry->_wancestry;
  wancestry.resize(_wedges.num());
  // If ambiguities in inside wedges, set no ancestor.
  if (0) {
  } else if (isr && wvtfl != wvtfr && wvsfl == wvsfr && wvsfl >= onumwedges && wvtfl < onumwedges &&
             wvtfr < onumwedges) {
    wancestry[wvsfl] = _wedges[wvsfl].attrib;
  } else if (isr && wvsfl != wvsfr && wvtfl == wvtfr && wvtfl >= onumwedges && wvsfl < onumwedges &&
             wvsfr < onumwedges) {
    wancestry[wvtfl] = _wedges[wvtfl].attrib;
  } else {
    if (isl) {
      int nwvsfl = wvsfl >= onumwedges;
      int nwvtfl = wvtfl >= onumwedges;
      if (nwvsfl) wancestry[wvsfl] = nwvtfl ? _wedges[wvsfl].attrib : wancestry[wvtfl];
      if (nwvtfl) wancestry[wvtfl] = nwvsfl ? _wedges[wvtfl].attrib : wancestry[wvsfl];
    }
    if (isr) {
      int nwvsfr = wvsfr >= onumwedges;
      int nwvtfr = wvtfr >= onumwedges;
      if (nwvsfr) wancestry[wvsfr] = nwvtfr ? _wedges[wvsfr].attrib : wancestry[wvtfr];
      if (nwvtfr) wancestry[wvtfr] = nwvsfr ? _wedges[wvtfr].attrib : wancestry[wvsfr];
    }
  }
  if (code & Vsplit::L_NEW) wancestry[wvlfl] = _wedges[wvlfl].attrib;
  if (code & Vsplit::R_NEW) wancestry[wvrfr] = _wedges[wvrfr].attrib;
}

void AWMesh::undo_vsplit(const Vsplit& vspl, const PMeshInfo& pminfo) {
  if (0) SHOW("**ecol");
  ASSERTX(_faces.ok(vspl.flclw));
  unsigned code = vspl.code;
  int ii = (code & Vsplit::II_MASK) >> Vsplit::II_SHIFT;
  ASSERTX(ii >= 0 && ii <= 2);
  const bool isl = true;
  bool isr;
  int fl, fr;
  if (vspl.vlr_offset1 > 1) {
    isr = true;
    fl = _faces.num() - 2;
    fr = _faces.num() - 1;
    ASSERTX(fl >= 0 && fr >= 0);
  } else {
    isr = false;
    fl = _faces.num() - 1;
    fr = k_undefined;
    ASSERTX(fl >= 0);
  }
  // Get wedges in neighborhood.
  int wvsfl, wvtfl;
  int wvsfr, wvtfr;
  wvsfl = _faces[fl].wedges[0];
  wvtfl = _faces[fl].wedges[1];
  if (!isr) {
    wvsfr = k_undefined;
    wvtfr = k_undefined;
  } else {
    wvsfr = _faces[fr].wedges[0];
    wvtfr = _faces[fr].wedges[2];
  }
  ASSERTX(!isl || (wvsfl >= 0 && wvtfl >= 0));
  ASSERTX(!isr || (wvsfr >= 0 && wvtfr >= 0));
  int vs = _wedges[wvsfl].vertex;
  int vt = _vertices.num() - 1;
  ASSERTX(!isl || _wedges[wvsfl].vertex == vs);
  ASSERTX(!isr || _wedges[wvsfr].vertex == vs);
  ASSERTX(!isl || _wedges[wvtfl].vertex == vt);
  ASSERTX(!isr || _wedges[wvtfr].vertex == vt);
  // Get adjacent faces and wedges on left and right.
  // really needed?
  int flccw, flclw;  // either (not both) may be k_undefined
  int frccw, frclw;  // either (or both) may be k_undefined
  // Also find index of vs within those adjacent faces
  int jlccw, jlclw, jrccw, jrclw;  // only defined if faces exist
  dummy_init(jlccw, jlclw, jrccw, jrclw);
  int wlccw, wlclw, wrccw, wrclw;  // k_undefined if faces does not exist
  if (isl) {
    flccw = _fnei[fl].faces[1];
    flclw = _fnei[fl].faces[0];
    if (flccw == k_undefined) {
      wlccw = k_undefined;
    } else {
      jlccw = get_jvf(vs, flccw);
      wlccw = _faces[flccw].wedges[jlccw];
    }
    ASSERTX((flclw == k_undefined) == (vspl.vlr_offset1 == 0));
    if (flclw == k_undefined) {
      wlclw = k_undefined;
    } else {
      jlclw = get_jvf(vt, flclw);
      wlclw = _faces[flclw].wedges[jlclw];
    }
  }
  if (!isr) {
    frccw = k_undefined;
    frclw = k_undefined;
    wrccw = k_undefined;
    wrclw = k_undefined;
  } else {
    frccw = _fnei[fr].faces[0];
    frclw = _fnei[fr].faces[2];
    if (frccw == k_undefined) {
      wrccw = k_undefined;
    } else {
      jrccw = get_jvf(vt, frccw);
      wrccw = _faces[frccw].wedges[jrccw];
    }
    if (frclw == k_undefined) {
      wrclw = k_undefined;
    } else {
      jrclw = get_jvf(vs, frclw);
      wrclw = _faces[frclw].wedges[jrclw];
    }
  }
  bool thru_l = wlccw == wvsfl && wlclw == wvtfl;
  bool thru_r = wrclw == wvsfr && wrccw == wvtfr;
#if defined(HH_DEBUG)
  {
    int vs_index = (code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
    assertx(vspl.flclw == (vspl.vlr_offset1 == 0 ? flccw : flclw));
    if (vspl.vlr_offset1 == 0)
      assertx(_wedges[_faces[flccw].wedges[vs_index]].vertex == vs);
    else
      assertx(_wedges[_faces[flclw].wedges[vs_index]].vertex == vt);
    assertx(fr == _fnei[fl].faces[2]);
    if (fr >= 0) assertx(_fnei[fr].faces[1] == fl);

    if (!isr) assertx((code & Vsplit::S_RSAME) && !(code & Vsplit::T_CSAME));
    if (!isr) assertx((code & Vsplit::T_RSAME) && !(code & Vsplit::T_CSAME));
    if (isl) {
      assertx((wlccw == wvsfl) == !!(code & Vsplit::S_LSAME));
      assertx((wlclw == wvtfl) == !!(code & Vsplit::T_LSAME));
    }
    if (isr) {
      assertx((wrclw == wvsfr) == !!(code & Vsplit::S_RSAME));
      assertx((wrccw == wvtfr) == !!(code & Vsplit::T_RSAME));
    }
    assertx((wvsfl == wvsfr) == !!(code & Vsplit::S_CSAME));
    assertx((wvtfl == wvtfr) == !!(code & Vsplit::T_CSAME));
  }
#endif
  // Update adjacency information.
  if (flccw >= 0) _fnei[flccw].faces[mod3(jlccw + 2)] = flclw;
  if (flclw >= 0) _fnei[flclw].faces[mod3(jlclw + 1)] = flccw;
  if (frccw >= 0) _fnei[frccw].faces[mod3(jrccw + 2)] = frclw;
  if (frclw >= 0) _fnei[frclw].faces[mod3(jrclw + 1)] = frccw;
  // Propagate wedges id's across collapsed faces if can go through.
  int ffl = flclw, ffr = frccw;
  int jjl = jlclw, jjr = jrccw;
  int* pwwl;
  dummy_init(pwwl);
  if (ffl >= 0)
    pwwl = &_faces[ffl].wedges[jjl];
  else if (k_debug)
    pwwl = reinterpret_cast<int*>(intptr_t{k_undefined});
  if (thru_l) {  // first go clw
    ASSERTX(pwwl && *pwwl == wlclw);
    ASSERTX(wlccw >= 0);
    for (;;) {
      *pwwl = wlccw;
      if (ffl == ffr) {
        ffl = ffr = k_undefined;  // all wedges seen
        break;
      }
      ffl = _fnei[ffl].faces[mod3(jjl + 2)];
      if (ffl < 0) break;
      jjl = get_jvf(vt, ffl);
      pwwl = &_faces[ffl].wedges[jjl];
      if (*pwwl != wlclw) break;
    }
  }
  if (ffr >= 0 && thru_r) {  // now go ccw from other side
    int* pw = &_faces[ffr].wedges[jjr];
    ASSERTX(*pw == wrccw);
    ASSERTX(wrclw >= 0);
    for (;;) {
      *pw = wrclw;
      if (ffr == ffl) {
        ffl = ffr = k_undefined;  // all wedges seen
        break;
      }
      ffr = _fnei[ffr].faces[mod3(jjr + 1)];
      if (ffr < 0) break;
      jjr = get_jvf(vt, ffr);
      pw = &_faces[ffr].wedges[jjr];
      if (*pw != wrccw) break;
    }
  }
  // Identify those wedges that will need to be updated to vs.
  //  (wmodif may contain some duplicates)
  PArray<int, 10> ar_wmodif;
  if (ffl >= 0) {
    for (;;) {
      int w = *pwwl;
      ar_wmodif.push(w);
      if (ffl == ffr) {
        ffl = ffr = k_undefined;
        break;
      }
      ffl = _fnei[ffl].faces[mod3(jjl + 2)];
      if (ffl < 0) break;
      jjl = get_jvf(vt, ffl);
      pwwl = &_faces[ffl].wedges[jjl];
    }
  }
  ASSERTX(ffl < 0 && ffr < 0);
  // Update wedge vertices to vs.
  for (int w : ar_wmodif) _wedges[w].vertex = vs;
  // Update vertex attributes.
  {
    PmVertexAttrib& va_s = _vertices[vs].attrib;
    PmVertexAttrib& va_t = _vertices[vt].attrib;
    switch (ii) {
      case 2: sub(va_s, va_s, vspl.vad_small); break;
      case 0: sub(va_s, va_t, vspl.vad_small); break;
      case 1:
        if (0) {
          PmVertexAttrib va_m;
          interp(va_m, va_s, va_t, 0.5f);
          sub(va_s, va_m, vspl.vad_small);
        } else {
          // slightly faster
          sub(va_s, va_t, vspl.vad_large);
          sub(va_s, va_s, vspl.vad_small);
        }
        break;
      default: assertnever("");
    }
  }
  // Update wedge attributes.
  PmWedgeAttrib awvtfr, awvsfr;
  dummy_init(awvtfr, awvsfr);
  bool problem = false;
  if (pminfo._has_wad2) {
    assertx(vspl.ar_wad.num() == 2);
    sub_noreflect(_wedges[wvsfl].attrib, _wedges[wvsfl].attrib, vspl.ar_wad[1]);
    _wedges[wvtfl].attrib = _wedges[wvsfl].attrib;
    goto GOTO_UNDO_WAD2;
  }
  //  they are currently predicted exactly.
  if (isr) {
    awvtfr = _wedges[wvtfr].attrib;
    awvsfr = _wedges[wvsfr].attrib;
  }
  if (isl) {
    bool nt = !(code & Vsplit::T_LSAME);
    bool ns = !(code & Vsplit::S_LSAME);
    if (nt && ns) {
      problem = true;
    } else {
      switch (ii) {
        case 2:
          if (!thru_l) _wedges[wvtfl].attrib = _wedges[wvsfl].attrib;
          break;
        case 0: _wedges[wvsfl].attrib = _wedges[wvtfl].attrib; break;
        case 1: {
          PmWedgeAttrib wa;
          if (0) {
            interp(wa, _wedges[wvsfl].attrib, _wedges[wvtfl].attrib, 0.5f);
          } else {
            // faster because avoid normalization of interp()
            const int lnum = 0;
            const PmWedgeAttribD& wad = vspl.ar_wad[lnum];
            sub_noreflect(wa, _wedges[wvtfl].attrib, wad);
          }
          _wedges[wvsfl].attrib = wa;
          if (!thru_l) _wedges[wvtfl].attrib = wa;
          break;
        }
        default: assertnever("");
      }
    }
  }
  if (isr) {
    bool nt = !(code & Vsplit::T_RSAME);
    bool ns = !(code & Vsplit::S_RSAME);
    bool ut = !(code & Vsplit::T_CSAME);
    bool us = !(code & Vsplit::S_CSAME);
    if (problem || us || ut) {
      switch (ii) {
        case 2:
          // If thru_r, then wvtfr & wrccw no longer exist.
          // This may be duplicating some work already done for isl.
          if (!nt && !thru_r) _wedges[wvtfr].attrib = awvsfr;
          break;
        case 0:
          // This may be duplicating some work already done for isl.
          if (!ns) _wedges[wvsfr].attrib = awvtfr;
          break;
        case 1: {
          PmWedgeAttrib wa;
          interp(wa, awvsfr, awvtfr, 0.5f);
          if (!ns) _wedges[wvsfr].attrib = wa;
          if (!nt && !thru_r) _wedges[wvtfr].attrib = wa;
          break;
        }
        default: assertnever("");
      }
    }
  }
  ASSERTX(wlccw < 0 || (attrib_ok(_wedges[wlccw].attrib), true));
  ASSERTX(wlclw < 0 || (attrib_ok(_wedges[wlclw].attrib), true));
  ASSERTX(wrccw < 0 || (attrib_ok(_wedges[wrccw].attrib), true));
  ASSERTX(wrclw < 0 || (attrib_ok(_wedges[wrclw].attrib), true));
GOTO_UNDO_WAD2:
  // Remove faces.
  _faces.resize(fl), _fnei.resize(fl);  // remove 1 or 2, !remember _fnei
  // Remove vertex.
  _vertices.sub(1);  // remove 1 vertex (vt)
  // Remove wedges.
  // optimize: construct static const lookup table on (S_MASK | T_MASK)
  bool was_wnl = isl && (code & Vsplit::T_LSAME) && (code & Vsplit::S_LSAME);
  bool was_wnr = isr && (code & Vsplit::T_RSAME) && (code & Vsplit::S_RSAME) && !(was_wnl && (code & Vsplit::T_CSAME));
  bool was_wntl = isl && (!(code & Vsplit::T_LSAME) && (!(code & Vsplit::T_CSAME) || !(code & Vsplit::T_RSAME)));
  bool was_wntr = isr && (!(code & Vsplit::T_CSAME) && !(code & Vsplit::T_RSAME));
  bool was_wnsl = isl && (!(code & Vsplit::S_LSAME) && (!(code & Vsplit::S_CSAME) || !(code & Vsplit::S_RSAME)));
  bool was_wnsr = isr && (!(code & Vsplit::S_CSAME) && !(code & Vsplit::S_RSAME));
  bool was_wnol = isl && (code & Vsplit::L_NEW);
  bool was_wnor = isr && (code & Vsplit::R_NEW);
  int nwr = (int(was_wnl) + int(was_wnr) + int(was_wntl) + int(was_wntr) + int(was_wnsl) + int(was_wnsr) +
             int(was_wnol) + int(was_wnor));
#if defined(HH_DEBUG)
  {
    // nwr == 0 possible when ws == LSAME | CSAME and wt == CSAME | RSAME
    assertx(nwr >= 0 && nwr <= 6);
    // Verify count with _wedges array.
    int cur = _wedges.num();
    if (was_wnor) --cur;  // "wvrfr"
    if (was_wnol) --cur;  // "wvlfl"
    if (was_wnsr) assertx(--cur == wvsfr);
    if (was_wnsl) assertx(--cur == wvsfl);
    if (was_wntr) assertx(--cur == wvtfr);
    if (was_wntl) assertx(--cur == wvtfl);
    if (was_wnr) assertx(--cur == wrccw);
    if (was_wnl) assertx(--cur == wlclw);
  }
#endif
  _wedges.sub(nwr);
  // Final check.
  // ok();
}

void AWMesh::ok() const {
  WMesh::ok();
  assertx(_fnei.num() == _faces.num());
  for_int(f, _faces.num()) {
    Set<int> setfnei;
    for_int(j, 3) {
      int ff = _fnei[f].faces[j];
      if (ff == k_undefined) continue;
      assertx(_faces.ok(ff));
      assertx(setfnei.add(ff));  // no valence_2 vertices
      int v1 = _wedges[_faces[f].wedges[mod3(j + 1)]].vertex;
      int v2 = _wedges[_faces[f].wedges[mod3(j + 2)]].vertex;
      assertx(v1 != v2);
      int ov1 = get_jvf(v1, ff);
      int ov2 = get_jvf(v2, ff);
      assertx(_fnei[ff].faces[mod3(ov1 + 1)] == f);
      assertx(mod3(ov1 - ov2 + 3) == 1);
    }
  }
  // Check that wedges are consecutive around each vertex.
  {
    int count = 0;
    // vertex -> most clw face (or beg of wedge (or any face))
    Array<int> mvf(_vertices.num(), k_undefined);
    for_int(f, _faces.num()) {
      for_int(j, 3) {
        int w = _faces[f].wedges[j];
        int v = _wedges[w].vertex;
        int fclw = _fnei[f].faces[mod3(j + 2)];
        bool is_beg_wedge = fclw >= 0 && get_wvf(v, fclw) != w;
        if (is_beg_wedge || mvf[v] == k_undefined) mvf[v] = f;
      }
    }
    for_int(f, _faces.num()) {
      for_int(j, 3) {
        int w = _faces[f].wedges[j];
        int v = _wedges[w].vertex;
        int fclw = _fnei[f].faces[mod3(j + 2)];
        bool is_most_clw = fclw == k_undefined;
        if (is_most_clw) mvf[v] = f;
      }
    }
    for_int(v, _vertices.num()) {
      int f0 = mvf[v];
      int wp = k_undefined;
      Set<int> setw;
      for (int f = f0;;) {
        int j = get_jvf(v, f);
        int w = _faces[f].wedges[j];
        if (w != wp) {
          assertx(setw.add(w));
          wp = w;
        }
        count++;
        f = _fnei[f].faces[mod3(j + 1)];  // go ccw
        if (f == k_undefined || f == f0) break;
      }
    }
    assertx(count == _faces.num() * 3);
  }
}

void AWMesh::split_edge(int f, int j, float frac1) {
  assertx(_wedges.num() == _vertices.num());  // Simpler case for now.
  const int w1 = _faces[f].wedges[j];
  const int w2 = _faces[f].wedges[mod3(j + 1)];
  const int v1 = _wedges[w1].vertex;
  const int v2 = _wedges[w2].vertex;
  const int f2 = _fnei[f].faces[mod3(j + 2)];
  assertx(f2 >= 0);                // Mesh is assumed to have no boundaries.
  const int j2 = get_jvf(v2, f2);  // Index of v2 within f2; mod3(j2 + 1) is index of v1 within f2.
  const int ws1 = _faces[f].wedges[mod3(j + 2)];
  const int ws2 = _faces[f2].wedges[mod3(j2 + 2)];
  const int vnew = _vertices.add(1);
  const int wnew = _wedges.add(1);
  const int fnew1 = _faces.add(1);
  const int fnew2 = _faces.add(1);
  _fnei.add(2);
  interp(_vertices[vnew].attrib, _vertices[v1].attrib, _vertices[v2].attrib, frac1);
  _wedges[wnew].vertex = vnew;
  interp(_wedges[wnew].attrib, _wedges[w1].attrib, _wedges[w2].attrib, frac1);
  _faces[fnew1].wedges = V(wnew, w2, ws1);
  _faces[fnew1].attrib = _faces[f].attrib;
  _faces[fnew2].wedges = V(w2, wnew, ws2);
  _faces[fnew2].attrib = _faces[f2].attrib;
  _faces[f].wedges[mod3(j + 1)] = wnew;
  _faces[f2].wedges[j2] = wnew;
  const auto fnei1 = _fnei[f].faces;
  const auto fnei2 = _fnei[f2].faces;
  const auto replace = [](auto& vec, auto oldval, auto newval) { vec[index(vec, oldval)] = newval; };
  replace(_fnei[fnei1[j]].faces, f, fnew1);
  replace(_fnei[fnei2[mod3(j2 + 1)]].faces, f2, fnew2);
  _fnei[f].faces[j] = fnew1;
  _fnei[f2].faces[mod3(j2 + 1)] = fnew2;
  _fnei[fnew1].faces = V(fnei1[j], f, fnew2);
  _fnei[fnew2].faces = V(f2, fnei2[mod3(j2 + 1)], fnew1);
}

void AWMesh::construct_adjacency() {
  _fnei.init(_faces.num());
  Map<std::pair<int, int>, int> mvv_face;  // (Vertex, Vertex) -> Face
  for_int(f, _faces.num()) {
    for_int(j, 3) {
      int j1 = mod3(j + 1), j2 = mod3(j + 2);
      int v0 = _wedges[_faces[f].wedges[j1]].vertex;
      int v1 = _wedges[_faces[f].wedges[j2]].vertex;
      mvv_face.enter(std::pair(v0, v1), f);
    }
  }
  for_int(f, _faces.num()) {
    for_int(j, 3) {
      int j1 = mod3(j + 1), j2 = mod3(j + 2);
      int v0 = _wedges[_faces[f].wedges[j1]].vertex;
      int v1 = _wedges[_faces[f].wedges[j2]].vertex;
      bool present;
      int fn = mvv_face.retrieve(std::pair(v1, v0), present);
      _fnei[f].faces[j] = present ? fn : k_undefined;
    }
  }
}

// *** PMesh

PMesh::PMesh() {
  _info._tot_nvsplits = 0;
  _info._full_nvertices = 0;
  _info._full_nwedges = 0;
  _info._full_nfaces = 0;
  _info._full_bbox[0] = Point(BIGFLOAT, BIGFLOAT, BIGFLOAT);
  _info._full_bbox[1] = Point(BIGFLOAT, BIGFLOAT, BIGFLOAT);
}

PMesh::PMesh(AWMesh&& awmesh, const PMeshInfo& pminfo) : _base_mesh(std::move(awmesh)), _info(pminfo) {
  _info._tot_nvsplits = 0;
  _info._full_nvertices = _base_mesh._vertices.num();
  _info._full_nwedges = _base_mesh._wedges.num();
  _info._full_nfaces = _base_mesh._faces.num();
  _info._full_bbox = Bbox{transform(_base_mesh._vertices, [](auto& v) { return v.attrib.point; })};
}

void PMesh::read(std::istream& is) {
  PMeshRStream pmrs(is, this);
  pmrs.read_base_mesh();
  for (;;)
    if (!pmrs.next_vsplit()) break;
}

void PMesh::write(std::ostream& os) const {
  os << "PM\n";
  os << "version=2\n";
  os << sform("nvsplits=%d nvertices=%d nwedges=%d nfaces=%d\n",  //
              _info._tot_nvsplits, _info._full_nvertices, _info._full_nwedges, _info._full_nfaces);
  const auto& bbox = _info._full_bbox;
  os << sform("bbox %g %g %g  %g %g %g\n", bbox[0][0], bbox[0][1], bbox[0][2], bbox[1][0], bbox[1][1], bbox[1][2]);
  os << sform("has_rgb=%d\n", _info._has_rgb);
  os << sform("has_uv=%d\n", _info._has_uv);
  os << sform("has_resid=%d\n", _info._has_resid);
  if (_info._has_wad2) os << sform("has_wad2=%d\n", _info._has_wad2);
  os << "PM base mesh:\n";
  _base_mesh.write(os, _info);
  for_int(i, _vsplits.num()) _vsplits[i].write(os, _info);
  os << uchar(k_magic_first_byte);
  os << "End of PM\n";
  assertx(os);
}

PMeshInfo PMesh::read_header(std::istream& is) {
  PMeshInfo pminfo;
  for (string line;;) {
    assertx(my_getline(is, line));
    if (line == "" || line[0] == '#') continue;
    assertx(line == "PM");
    break;
  }
  // default for version 1 compatibility
  pminfo._read_version = 1;
  // default for version 0 compatibility
  pminfo._has_rgb = false;
  pminfo._has_uv = true;
  pminfo._has_resid = false;
  pminfo._has_wad2 = false;

  pminfo._tot_nvsplits = 0;
  pminfo._full_nvertices = 0;
  pminfo._full_nwedges = 0;
  pminfo._full_nfaces = 0;
  pminfo._full_bbox[0] = Point(BIGFLOAT, BIGFLOAT, BIGFLOAT);
  pminfo._full_bbox[1] = Point(BIGFLOAT, BIGFLOAT, BIGFLOAT);
  for (string line;;) {
    assertx(my_getline(is, line));
    const char* sline = line.c_str();
    if (const char* s = after_prefix(sline, "version=")) {
      pminfo._read_version = to_int(s);
      continue;
    }
    if (const char* s = after_prefix(sline, "nvsplits=")) {
      pminfo._tot_nvsplits = int_from_chars(s);
      s = assertx(after_prefix(s, " nvertices=")), pminfo._full_nvertices = int_from_chars(s);
      s = assertx(after_prefix(s, " nwedges=")), pminfo._full_nwedges = int_from_chars(s);
      s = assertx(after_prefix(s, " nfaces=")), pminfo._full_nfaces = int_from_chars(s);
      assert_no_more_chars(s);
      continue;
    }
    if (const char* s = after_prefix(sline, "bbox ")) {
      for_int(i, 2) for_int(j, 3) pminfo._full_bbox[i][j] = float_from_chars(s);
      assert_no_more_chars(s);
      continue;
    }
    if (const char* s = after_prefix(sline, "has_rgb=")) {
      pminfo._has_rgb = narrow_cast<bool>(to_int(s));
      continue;
    }
    if (const char* s = after_prefix(sline, "has_uv=")) {
      pminfo._has_uv = narrow_cast<bool>(to_int(s));
      continue;
    }
    if (const char* s = after_prefix(sline, "has_resid=")) {
      pminfo._has_resid = narrow_cast<bool>(to_int(s));
      continue;
    }
    if (const char* s = after_prefix(sline, "has_wad2=")) {
      pminfo._has_wad2 = narrow_cast<bool>(to_int(s));
      continue;
    }
    if (!strcmp(sline, "PM base mesh:")) break;
    Warning("PMesh header string unknown");
    showf("PMesh header string not recognized '%s'\n", sline);
  }
  // assertw(pminfo._tot_nvsplits);
  assertw(pminfo._full_nvertices);
  assertw(pminfo._full_nwedges);
  assertw(pminfo._full_nfaces);
  assertw(pminfo._full_bbox[0][0] != BIGFLOAT);
  return pminfo;
}

void PMesh::truncate_beyond(PMeshIter& pmi) {
  PMeshRStream& pmrs = pmi._pmrs;
  _vsplits.resize(pmrs._vspliti);
  _info._tot_nvsplits = _vsplits.num();
  _info._full_nvertices = pmi._vertices.num();
  _info._full_nwedges = pmi._wedges.num();
  _info._full_nfaces = pmi._faces.num();
  pmrs._info = _info;
  // really, should update all PMeshRStreams which are open on PMesh.
}

void PMesh::truncate_prior(PMeshIter& pmi) {
  PMeshRStream& pmrs = pmi._pmrs;
  _base_mesh = pmi;
  _vsplits.erase(0, pmrs._vspliti);
  _info._tot_nvsplits = _vsplits.num();
  pmrs._info = _info;
  pmrs._vspliti = 0;
  // really, should update all PMeshRStreams which are open on PMesh.
}

// *** PMeshRStream

PMeshRStream::PMeshRStream(const PMesh& pm) : _is(nullptr), _pm(const_cast<PMesh*>(&pm)) { _info = _pm->_info; }

PMeshRStream::PMeshRStream(std::istream& is, PMesh* ppm_construct) : _is(&is), _pm(ppm_construct) {
  _info = PMesh::read_header(*_is);
  if (_pm) _pm->_info = _info;
}

PMeshRStream::~PMeshRStream() {
  // Should not check for PMesh trailer here since not all records may have been read.
}

void PMeshRStream::read_base_mesh(AWMesh* bmesh) {
  assertx(_vspliti == -1);
  _vspliti = 0;
  if (!_is) {
    assertx(_pm);
    if (bmesh) *bmesh = _pm->_base_mesh;
  } else {
    if (!_pm && !bmesh) Warning("strange, why are we doing this?");
    auto tbmesh = make_optional_if<AWMesh>(!_pm && !bmesh);
    AWMesh& rbmesh = _pm ? _pm->_base_mesh : bmesh ? *bmesh : *tbmesh;
    rbmesh.read(*_is, _info);
    if (_pm && bmesh) *bmesh = _pm->_base_mesh;
  }
}

const AWMesh& PMeshRStream::base_mesh() {
  if (_vspliti == -1) {
    if (!_pm) {
      read_base_mesh(&_lbase_mesh);
    } else {
      read_base_mesh(nullptr);
    }
  }
  return _pm ? _pm->_base_mesh : _lbase_mesh;
}

const Vsplit* PMeshRStream::peek_next_vsplit() {
  assertx(_vspliti >= 0);
  if (_pm) {
    if (_vspliti < _pm->_vsplits.num()) return &_pm->_vsplits[_vspliti];
    if (!_is) return nullptr;  // end of array
  } else if (_vspl_ready) {
    // have buffer record
    return &_tmp_vspl;
  }
  assertx(*_is);
  if (PMesh::at_trailer(*_is)) return nullptr;
  Vsplit* pvspl;
  if (_pm) {
    Array<Vsplit>& vsplits = _pm->_vsplits;
    if (!vsplits.num()) vsplits.reserve(_pm->_info._tot_nvsplits);  // Note: initializes all ar_wad members.
    vsplits.access(_vspliti);
    pvspl = &vsplits[_vspliti];
  } else {
    pvspl = &_tmp_vspl;
    _vspl_ready = true;
  }
  pvspl->read(*_is, _info);
  return pvspl;
}

const Vsplit* PMeshRStream::next_vsplit() {
  assertx(_vspliti >= 0);
  if (_pm) {
    Array<Vsplit>& vsplits = _pm->_vsplits;
    if (_vspliti < vsplits.num()) {
      _vspliti++;
      return &vsplits[_vspliti - 1];
    }
    if (!_is) return nullptr;  // end of array
  } else if (_vspl_ready) {
    // use up buffer record
    _vspl_ready = false;
    return &_tmp_vspl;
  }
  assertx(*_is);
  if (PMesh::at_trailer(*_is)) return nullptr;
  Vsplit* pvspl;
  if (_pm) {
    Array<Vsplit>& vsplits = _pm->_vsplits;
    if (!vsplits.num()) vsplits.reserve(_pm->_info._tot_nvsplits);
    _vspliti++;
    vsplits.resize(_vspliti);
    pvspl = &vsplits.last();
  } else {
    pvspl = &_tmp_vspl;
  }
  pvspl->read(*_is, _info);
  return pvspl;
}

const Vsplit* PMeshRStream::prev_vsplit() {
  assertx(_vspliti >= 0);
  assertx(is_reversible());       // die if !_pm
  if (!_vspliti) return nullptr;  // end of array
  --_vspliti;
  return &_pm->_vsplits[_vspliti];
}

// *** PMeshIter

PMeshIter::PMeshIter(PMeshRStream& pmrs) : _pmrs(pmrs) { _pmrs.read_base_mesh(this); }

bool PMeshIter::next_ancestry(Ancestry* ancestry) {
  const Vsplit* vspl = _pmrs.next_vsplit();
  if (!vspl) return false;
  apply_vsplit(*vspl, rstream()._info, ancestry);
  return true;
}

bool PMeshIter::prev() {
  const Vsplit* vspl = _pmrs.prev_vsplit();
  if (!vspl) return false;
  undo_vsplit(*vspl, rstream()._info);
  return true;
}

bool PMeshIter::goto_nvertices_ancestry(int nvertices, Ancestry* ancestry) {
  // If have PM and about to go to full mesh, reserve.
  if (_pmrs._pm) {
    const PMesh& pm = *_pmrs._pm;
    if (nvertices >= pm._info._full_nvertices) {
      _vertices.reserve(pm._info._full_nvertices);
      _wedges.reserve(pm._info._full_nwedges);
      _faces.reserve(pm._info._full_nfaces);
    }
  }
  for (;;) {
    int cn = _vertices.num();
    if (cn < nvertices) {
      if (!next_ancestry(ancestry)) return false;
    } else if (cn > nvertices) {
      assertx(!ancestry);
      if (!prev()) return false;
    } else {
      break;
    }
  }
  return true;
}

bool PMeshIter::goto_nfaces_ancestry(int nfaces, Ancestry* ancestry) {
  // If have PM and about to go to full mesh, reserve.
  if (_pmrs._pm) {
    const PMesh& pm = *_pmrs._pm;
    if (nfaces >= pm._info._full_nfaces) {
      _vertices.reserve(pm._info._full_nvertices);
      _wedges.reserve(pm._info._full_nwedges);
      _faces.reserve(pm._info._full_nfaces);
    }
  }
  if (_faces.num() < nfaces) {
    while (_faces.num() < nfaces - 1) {
      if (!next_ancestry(ancestry)) return false;
    }
    if (_faces.num() == nfaces - 1) {
      // Avoid over-shooting since may not be reversible,
      //  so peek at next vsplit and apply it if it adds only 1 face.
      const Vsplit* pvspl = _pmrs.peek_next_vsplit();
      if (pvspl && !pvspl->adds_two_faces()) {
        assertx(next_ancestry(ancestry));
        assertx(_faces.num() == nfaces);
      }
    }
  } else if (_faces.num() > nfaces) {
    assertx(!ancestry);
    while (_faces.num() > nfaces) {
      if (!prev()) return false;
    }
    // Favor (nfaces - 1) over (nfaces + 1).
  }
  return true;
}

// *** Geomorph

bool Geomorph::construct(PMeshIter& pmi, EWant want, int num) {
  // Note that this construction is not identical to that in Filterprog.cpp, at least for wedges,
  //  since Filterprog.cpp records for each final wedge its original wid
  //  in M^c, or 0 if the wedge was introduced subsequently.
  // However, a newly added wedge introduced in M^j, j > c, can have further descendants in M^k, k > j,
  //  with different attributes, and these perhaps should be morphed (between their original values in
  //  j and their final values in k).
  // They are morphed here but not in Filterprog.cpp.
  // 1997-05-16: this is the reason why we didn't let the Ancestry class contain ancestry relations (and record the
  // attributes of M^c): we would be missing the original attributes of an orphan wedge introduced in M^j that was
  // subsequently changed.  The only solution is to carry the attributes explicitly as is currently done,
  // or to allocate a separate array to record the attributes of orphan wedges when they are first created,
  // which does not seem so nice.
  bool ret = true;
  assertx(!_vertices.num() && !_vgattribs.num() && !_wgattribs.num());
  // Initialize ancestry to current attributes (identity).
  // optimize: could reserve final size for arrays.
  Ancestry ancestry;
  ancestry._vancestry.init(pmi._vertices.num());
  for_int(v, ancestry._vancestry.num()) ancestry._vancestry[v] = pmi._vertices[v].attrib;
  ancestry._wancestry.init(pmi._wedges.num());
  for_int(w, ancestry._wancestry.num()) ancestry._wancestry[w] = pmi._wedges[w].attrib;
  // Perform vsplits.
  switch (want) {
    case EWant::vsplits:
      for_int(i, num) {
        if (!pmi.next_ancestry(&ancestry)) {
          ret = false;
          break;
        }
      }
      break;
    case EWant::nfaces:
      assertx(num >= pmi._faces.num());
      if (!pmi.goto_nfaces_ancestry(num, &ancestry)) ret = false;
      break;
    case EWant::nvertices:
      assertx(num >= pmi._vertices.num());
      if (!pmi.goto_nvertices_ancestry(num, &ancestry)) ret = false;
      break;
    default: assertnever("");
  }
  // Grab final mesh
  implicit_cast<WMesh&>(*this) = pmi;  // copy pmi::AWMesh to this->WMesh
  ASSERTX(_vertices.num() == ancestry._vancestry.num());
  ASSERTX(_wedges.num() == ancestry._wancestry.num());
  // Fill attribute arrays _vattribs and _wattribs.
  {
    int nv = 0;
    for_int(v, ancestry._vancestry.num()) {
      if (compare(ancestry._vancestry[v], _vertices[v].attrib)) nv++;
    }
    _vgattribs.init(nv);
    nv = 0;
    for_int(v, ancestry._vancestry.num()) {
      if (!compare(ancestry._vancestry[v], _vertices[v].attrib)) continue;
      _vgattribs[nv].vertex = v;
      _vgattribs[nv].attribs[0] = ancestry._vancestry[v];
      _vgattribs[nv].attribs[1] = _vertices[v].attrib;
      nv++;
    }
    assertx(nv == _vgattribs.num());
  }
  {
    int nw = 0;
    for_int(w, ancestry._wancestry.num()) {
      if (compare(ancestry._wancestry[w], _wedges[w].attrib)) nw++;
    }
    _wgattribs.init(nw);
    nw = 0;
    for_int(w, ancestry._wancestry.num()) {
      if (!compare(ancestry._wancestry[w], _wedges[w].attrib)) continue;
      _wgattribs[nw].wedge = w;
      _wgattribs[nw].attribs[0] = ancestry._wancestry[w];
      _wgattribs[nw].attribs[1] = _wedges[w].attrib;
      nw++;
    }
    assertx(nw == _wgattribs.num());
  }
  return ret;
}

bool Geomorph::construct_next(PMeshIter& pmi, int nvsplits) { return construct(pmi, EWant::vsplits, nvsplits); }

bool Geomorph::construct_goto_nvertices(PMeshIter& pmi, int nvertices) {
  return construct(pmi, EWant::nvertices, nvertices);
}

bool Geomorph::construct_goto_nfaces(PMeshIter& pmi, int nfaces) { return construct(pmi, EWant::nfaces, nfaces); }

void Geomorph::evaluate(float alpha) {
  assertx(alpha >= 0.f && alpha < 1.f);
  const float frac1 = 1.f - alpha;
  for (const PmVertexAttribG& ag : _vgattribs)
    interp(_vertices[ag.vertex].attrib, ag.attribs[0], ag.attribs[1], frac1);
  for (const PmWedgeAttribG& ag : _wgattribs) interp(_wedges[ag.wedge].attrib, ag.attribs[0], ag.attribs[1], frac1);
}

// *** SMesh

SMesh::SMesh(const WMesh& wmesh)
    : _materials(wmesh._materials), _vertices(wmesh._wedges.num()), _faces(wmesh._faces.num()) {
  for_int(v, _vertices.num()) {
    int ov = wmesh._wedges[v].vertex;
    _vertices[v].attrib.v = wmesh._vertices[ov].attrib;
    _vertices[v].attrib.w = wmesh._wedges[v].attrib;
  }
  for_int(f, _faces.num()) {
    for_int(j, 3) _faces[f].vertices[j] = wmesh._faces[f].wedges[j];
    _faces[f].attrib = wmesh._faces[f].attrib;
  }
}

GMesh SMesh::extract_gmesh(int has_rgb, int has_uv) const {
  GMesh gmesh;
  string str;
  for_int(v, _vertices.num()) {
    Vertex gv = gmesh.create_vertex();
    ASSERTX(gmesh.vertex_id(gv) == v + 1);
    gmesh.set_point(gv, _vertices[v].attrib.v.point);
    gmesh.update_string(gv, "normal", csform_vec(str, _vertices[v].attrib.w.normal));
    if (has_rgb) gmesh.update_string(gv, "rgb", csform_vec(str, _vertices[v].attrib.w.rgb));
    if (has_uv) gmesh.update_string(gv, "uv", csform_vec(str, _vertices[v].attrib.w.uv));
  }
  Array<Vertex> gva;
  for_int(f, _faces.num()) {
    gva.init(0);
    for_int(j, 3) {
      int v = _faces[f].vertices[j];
      gva.push(gmesh.id_vertex(v + 1));
    }
    Face gf = gmesh.create_face(gva);
    int matid = _faces[f].attrib.matid & ~AWMesh::k_Face_visited_mask;
    gmesh.set_string(gf, _materials.get(matid).c_str());
  }
  return gmesh;
}

// *** SGeomorph

SGeomorph::SGeomorph(const Geomorph& geomorph) : SMesh(geomorph) {
  Array<int> mvmodif(geomorph._vertices.num(), k_undefined);
  Array<int> mwmodif(geomorph._wedges.num(), k_undefined);
  for_int(ovi, geomorph._vgattribs.num()) mvmodif[geomorph._vgattribs[ovi].vertex] = ovi;
  for_int(owi, geomorph._wgattribs.num()) mwmodif[geomorph._wgattribs[owi].wedge] = owi;
  for_int(ow, geomorph._wedges.num()) {
    int ov = geomorph._wedges[ow].vertex;
    int ovi = mvmodif[ov];
    int owi = mwmodif[ow];
    if (ovi == k_undefined && owi == k_undefined) continue;
    int gnum = _vgattribs.add(1);
    _vgattribs[gnum].vertex = ow;
    for_int(a, 2) {
      _vgattribs[gnum].attribs[a].v =
          (ovi != k_undefined ? geomorph._vgattribs[ovi].attribs[a] : geomorph._vertices[ov].attrib);
      _vgattribs[gnum].attribs[a].w =
          (owi != k_undefined ? geomorph._wgattribs[owi].attribs[a] : geomorph._wedges[ow].attrib);
    }
  }
  _vgattribs.shrink_to_fit();
}

void SGeomorph::evaluate(float alpha) {
  assertx(alpha >= 0.f && alpha < 1.f);
  const float frac1 = 1.f - alpha;
  for (const PmSVertexAttribG& ag : _vgattribs)
    interp(_vertices[ag.vertex].attrib, ag.attribs[0], ag.attribs[1], frac1);
}

GMesh SGeomorph::extract_gmesh(int has_rgb, int has_uv) const {
  GMesh gmesh = SMesh::extract_gmesh(has_rgb, has_uv);
  Array<int> mvvg(_vertices.num(), -1);
  for_int(vg, _vgattribs.num()) mvvg[_vgattribs[vg].vertex] = vg;
  string str;
  for_int(v, _vertices.num()) {
    Vertex gv = gmesh.id_vertex(v + 1);
    int vg = mvvg[v];
    if (vg >= 0) assertx(_vgattribs[vg].vertex == v);
    const PmSVertexAttrib& va0 = vg < 0 ? _vertices[v].attrib : _vgattribs[vg].attribs[0];
    const Point& opos = va0.v.point;
    gmesh.update_string(gv, "Opos", csform_vec(str, opos));
    const Vector& onor = va0.w.normal;
    gmesh.update_string(gv, "Onormal", csform_vec(str, onor));
    if (has_rgb) {
      const A3dColor& col = va0.w.rgb;
      gmesh.update_string(gv, "Orgb", csform_vec(str, col));
    }
    if (has_uv) {
      const Uv& uv = va0.w.uv;
      gmesh.update_string(gv, "Ouv", csform_vec(str, uv));
    }
  }
  return gmesh;
}

}  // namespace hh
