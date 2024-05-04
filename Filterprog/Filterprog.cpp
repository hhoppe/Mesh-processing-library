// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt

#include <cctype>   // isspace(), isdigit(), etc.
#include <cstdlib>  // atoi()
#include <cstring>  // strncmp() etc.

#include "libHh/A3dStream.h"  // A3dColor
#include "libHh/Args.h"
#include "libHh/Bbox.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/Geometry.h"
#include "libHh/MeshOp.h"  // mesh_genus_string(), Vnors
#include "libHh/PMesh.h"   // S_vsplit
#include "libHh/RangeOp.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

// TODO: let REFINE() function look at mesh after vsplit?

int maxnfaces = 0;            // number of faces in next request
bool stringent = false;       // in sel.ref., condition (1) instead of (1')
bool use_silhouette = false;  // REFINE criterion
bool use_area = false;        // REFINE criterion
bool splitcorners = false;    // output meshes: split wid into vertices
string append_old_pm;         // name of old PM file

unique_ptr<RFile> pfi_prog;   // progressive file being read
GMesh mesh;                   // current mesh
bool record_changes = false;  // output stream of mesh changes
bool sel_refinement = false;  // selective refinement is active
Frame view_frame;             // if sel_refinement
float view_zoom = 0.f;        // if sel_refinement
Frame view_iframe;            // inverse(view_frame)
const bool sdebug = getenv_bool("FILTERPROG_DEBUG");

Array<string> pm_material_strings;
int pm_nvsplits, pm_nvertices, pm_nwedges, pm_nfaces;
Bbox pm_bbox;
bool pm_has_wad2 = false;

string g_header;

constexpr float k_undefined = BIGFLOAT;  // undefined scalar attribute

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Point, v_opos);  // for morph, old vertex position

struct WedgeInfo {
  Vector nor;
  A3dColor col;
  UV uv;
};

// Used to store the scalar attributes associate with an integer wedge id.
class MapWedgeInfo {
 public:
  void clear() { _map.clear(); }
  bool contains(int wid) const {
    assertx(wid);
    return _map.contains(wid);
  }
  void set(int wid, WedgeInfo wi) {
    assertx(wid);
    _map[wid] = wi;
  }
  const WedgeInfo& get(int wid) const { return _map.get(wid); }
  void copy_from(const MapWedgeInfo& mwi);

 private:
  Map<int, WedgeInfo> _map;
};

void MapWedgeInfo::copy_from(const MapWedgeInfo& mwi) {
  assertx(!_map.num());
  for_map_key_value(mwi._map, [&](int wid, const WedgeInfo& wi) { set(wid, wi); });
}

MapWedgeInfo gcwinfo;  // current wedge information
MapWedgeInfo gowinfo;  // old wedge information

HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, int, c_cwedge_id);  // current wedge id's
HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, int, c_owedge_id);  // old wedge id's (for morph)

// Used to store the closest living ancestor of vertices.
// If !sel_refinement, this is the identity map.
// If sel_refinement, some vertices in PM may not exist in mesh; this map
//  finds their closest ancestor that does exist in mesh.
class LivingAncestor {
 public:
  int existing_id(int vid) const { return _is_active ? _mapvidvid.get(vid) : vid; }
  void activate();
  void set_descendant(int vnew, int vold);
  void set_vertex(int vnew) {
    if (_is_active) _mapvidvid.enter(vnew, vnew);
  }

 private:
  bool _is_active{false};
  Map<int, int> _mapvidvid;  // vid -> closest living ancestor vid
};

void LivingAncestor::activate() {
  assertx(!_is_active);
  _is_active = true;
  for (Vertex v : mesh.vertices()) set_vertex(mesh.vertex_id(v));
}

void LivingAncestor::set_descendant(int vnew, int vold) {
  assertx(_is_active);
  assertx(_mapvidvid.get(vold) == vold);
  _mapvidvid.enter(vnew, vold);
}

LivingAncestor vlineage;  // closest living ancestor relation

// *** Beginning of app functions.

void clear_vertex_corner_strings(Vertex v) {
  mesh.set_string(v, nullptr);
  for (Corner c : mesh.corners(v)) mesh.set_string(c, nullptr);
}

void clear_mesh_strings() {
  for (Vertex v : mesh.vertices()) clear_vertex_corner_strings(v);
  // Face string info such as groups and materials is kept intact and propagated through.
}

// Parse the string information associated with a corner and create a WedgeInfo structure.
WedgeInfo create_winfo(Corner c) {
  WedgeInfo wi;
  Vector& nor = wi.nor;
  fill(nor, k_undefined);
  mesh.parse_corner_key_vec(c, "normal", nor);
  assertx(nor[0] != k_undefined);  // normals always present
  if (abs(mag2(nor) - 1.f) > 1e-4f) {
    Warning("Renormalizing normal");
    assertw(nor.normalize());
  }
  A3dColor& col = wi.col;
  fill(col, k_undefined);
  mesh.parse_corner_key_vec(c, "rgb", col);
  UV& uv = wi.uv;
  fill(uv, k_undefined);
  mesh.parse_corner_key_vec(c, "uv", uv);
  return wi;
}

// Upon application of a vertex split, for affected corners of the mesh, record their "ancestor" wedge id's.
// With non-strict PM, some of these ancestor wid's may be assigned 0 to indicate that no ancestor corner exists.
void carry_old_corner_info(Edge e) {
  for (Face f : mesh.faces(e)) {
    for (Corner c : mesh.corners(f)) c_owedge_id(c) = 0;
  }
  for (Face ff : mesh.faces(e)) {
    for (Corner crep : mesh.corners(ff)) {
      int crepcwid = c_cwedge_id(crep);
      int owid = 0;
      for_int(dir, 2) {
        for (Corner c = crep;;) {
          c = !dir ? mesh.ccw_corner(c) : mesh.clw_corner(c);
          if (!c || c == crep || c_cwedge_id(c) != crepcwid) break;
          int towid = c_owedge_id(c);
          if (towid) {
            owid = towid;
            break;
          }
        }
        if (owid) break;
      }
      if (owid) {
        c_owedge_id(crep) = owid;
      } else {
        // hopefully, will be found below in opposite corner
      }
    }
  }
  for (Face f : mesh.faces(e)) {
    for (Vertex v : mesh.vertices(e)) {
      Corner c = mesh.corner(v, f);
      if (c_owedge_id(c)) continue;
      Vertex vo = mesh.opp_vertex(v, e);
      Corner co = mesh.corner(vo, f);
      int towid = c_owedge_id(co);
      if (towid) {
        c_owedge_id(c) = towid;
      } else {
        if (sdebug) {
          // It's ok, there was a non-strict edge collapse.
          Warning("New wedge not morphed");
          // Will use the final value as old one -> constant.
          // Keep c_owedge_id(co) == 0 to indicate this.
        }
      }
    }
    if (sdebug) {
      Vertex vo = mesh.opp_vertex(e, f);
      if (!c_owedge_id(mesh.corner(vo, f))) {
        // It's ok, there was a non-strict edge collapse.
        Warning("New side wedge not morphed");
      }
    }
  }
}

// Project v onto the screen and return screen coordinates.
Point compute_screenpoint(Vertex v) {
  Point pview = mesh.point(v) * view_iframe;
  if (pview[0] > 0) {
    float denom = pview[0] * view_zoom;
    pview[1] /= denom;
    pview[2] /= denom;
  }
  return pview;
}

// Do screen coordinates lie within view frustum?
bool screenpoint_within_frustum(const Point& pscreen) {
  return pscreen[0] > 0 && abs(pscreen[1]) <= 1 && abs(pscreen[2]) <= 1;
}

// REFINE(vs) function.
bool should_perform_vsplit(Vertex vs) {
  if (!sel_refinement) return true;
  Point p1screen = compute_screenpoint(vs);
  bool in_frustum = screenpoint_within_frustum(p1screen);
  bool some_in_frustum = in_frustum;
  Array<Point> ar_pscreen;
  for (Vertex v : mesh.ccw_vertices(vs)) {
    Point pscreen = compute_screenpoint(v);
    ar_pscreen.push(pscreen);
    if (screenpoint_within_frustum(pscreen)) some_in_frustum = true;
  }
  bool facing_front = false, facing_back = false;
  float area2 = 0.f;
  for_int(i, ar_pscreen.num()) {
    int i1 = (i + 1) % ar_pscreen.num();
    if (!i1 && mesh.is_boundary(vs)) continue;                      // not a face
    if (ar_pscreen[i][0] <= 0 || ar_pscreen[i1][0] <= 0) continue;  // behind viewer
    float x1 = p1screen[1], y1 = p1screen[2];
    float x2 = ar_pscreen[i][1], y2 = ar_pscreen[i][2];
    float x3 = ar_pscreen[i1][1], y3 = ar_pscreen[i1][2];
    float vcross = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);  // expansion about 1st p
    vcross = -vcross;                                              // -x axis towards viewer
    if (vcross >= 0)
      facing_front = true;
    else
      facing_back = true;
    if (vcross >= 0) area2 += vcross;
  }
  bool near_silhouette = facing_front && facing_back;
  const float frac_screen = 80.f / 500.f;
  bool sufficient_area = area2 > square(frac_screen);
  if (!some_in_frustum) return false;
  if (!use_silhouette && !use_area) return true;
  return (use_silhouette && near_silhouette) || (use_area && sufficient_area);
}

// Get the string within the braces.  Note the side-effect on str!
const char* get_sinfo(char* str) {
  char* sinfo = strchr(str, '{');
  if (sinfo) {
    *sinfo++ = 0;
    char* s = strchr(sinfo, '}');
    if (!s) {
      if (Warning("Mesh info string has no matching '}'")) SHOW(str, sinfo);
      return nullptr;
    }
    *s = 0;
  }
  return sinfo;
}

struct _save0 {
  Vertex vs, vt;
  int ii;
  bool skip_current;
  Set<int> wid_read;
};
_save0 save0;

// Read and parse a single line of a vsplit record.
bool parse_line(char* sline, bool& after_vsplit, bool carry_old) {
  // Adapted from GMesh::read_line().
  if (!strcmp(sline, "# Beg REcol")) {
    if (!save0.skip_current) {
      if (carry_old) carry_old_corner_info(mesh.edge(save0.vs, save0.vt));
      if (record_changes) std::cout << "# frame\n";
    }
    save0.skip_current = false;
    save0.wid_read.clear();
    return false;
  }
  if (sline[0] == '#') return true;
  string orig_sline;
  if (record_changes) orig_sline = sline;
  if (!strncmp(sline, "REcol ", 6)) {
    assertx(!after_vsplit);
    assertx(!save0.skip_current);
    after_vsplit = true;
    int vni, vsi, vti, vli, vri, fli, fri, ii;
    assertx(sscanf(sline, "REcol %d %d %d %d %d %d %d %d", &vni, &vsi, &vti, &vli, &vri, &fli, &fri, &ii) == 8);
    assertx(vni == vsi);  // redundancy
    int vsie = vlineage.existing_id(vsi);
    int vlie = vlineage.existing_id(vli);
    int vrie = vri ? vlineage.existing_id(vri) : 0;
    Vertex vs = mesh.id_vertex(vsie);
    Vertex vl = mesh.id_vertex(vlie);
    Vertex vr = vrie ? mesh.id_vertex(vrie) : nullptr;
    bool possible = true;
    if (stringent) {
      // SIGGRAPH 96 conditions (1) and (2)
      if (vsie != vsi) possible = false;
      if (vl && vlie != vli) possible = false;
      if (vr && vrie != vri) possible = false;
    } else {
      // With conditions (1\') and (2), the edges (vs, vl) or (vs, vr)
      //  may not exist.
      // See my notes 1996-06-21.  I don't see any easy fix for this.
      if (0) {
        // SIGGRAPH 96 conditions (1\') and (2)
        if (vsie != vsi) possible = false;
        if (vl && (vs == vl || !mesh.query_edge(vs, vl))) possible = false;
        if (vr && (vs == vr || !mesh.query_edge(vs, vr))) possible = false;
        if (vl == vr) possible = false;
      } else {
        // Even less stringent:
        //   vertex vs need not exist, if so then use its ancestor.
        // Note: exposes explicit dependency of this vsplit on
        //  exactly 2 prior vsplits that distinguish vs, vl, and vr.
        if (vsie == vlie) possible = false;
        if (vsie == vrie) possible = false;
        if (vlie == vrie) possible = false;
        // Problem: the edge (vs, vl) may not be present.
        if (possible && vl && !mesh.query_edge(vs, vl)) possible = false;
        if (possible && vr && !mesh.query_edge(vs, vr)) possible = false;
        // Overall, I think this looks nicer than the SIGGRAPH 96
        //  (1\') and (2) criteria; 1996-06-21.
      }
    }
    if (!possible || !should_perform_vsplit(vs)) {
      assertx(sel_refinement);
      vlineage.set_descendant(vti, vsie);
      save0.skip_current = true;
      return true;
    }
    vlineage.set_vertex(vti);
    // DO IT:
    if (record_changes) mesh.record_changes(&std::cout);
    Vertex vt = mesh.split_vertex(vs, vl, vr, vti);
    if (record_changes) mesh.record_changes(nullptr);
    save0.vs = vs;
    save0.vt = vt;
    save0.ii = ii;
    if (carry_old) v_opos(vt) = v_opos(vs);
  } else if (sline[0] == 'F' && !strncmp(sline, "Face ", 5)) {
    if (save0.skip_current) return true;
    const char* sinfo = get_sinfo(sline);  // note side-effect!
    PArray<Vertex, 8> va;
    char* s = sline + 4;
    int fi = -1;
    for (;;) {
      while (*s && isspace(*s)) s++;
      if (!*s) break;
      char* beg = s;
      while (*s && isdigit(*s)) s++;
      assertx(!*s || isspace(*s));
      int j = to_int(beg);
      if (fi < 0) {
        fi = j;
        continue;
      }
      va.push(mesh.id_vertex(vlineage.existing_id(j)));
    }
    assertx(va.num() == 3);
    assertx(mesh.legal_create_face(va));
    assertx(fi >= 1);
    Face f = mesh.create_face_private(fi, va);
    if (sinfo) mesh.set_string(f, sinfo);
    if (record_changes) std::cout << orig_sline << '\n';
  } else if (sline[0] == 'E' && !strncmp(sline, "Edge ", 5)) {
    assertnever("Edge flags not used anymore");
  } else if (!strncmp(sline, "MVertex ", 8)) {
    if (save0.skip_current) return true;
    if (!after_vsplit) {
      // assertx(mesh.point(v) == p);
      return true;
    }
    Point p;
    int vi;
    assertx(sscanf(sline, "MVertex %d %g %g %g", &vi, &p[0], &p[1], &p[2]) == 4);
    Vertex v = mesh.id_vertex(vlineage.existing_id(vi));
    mesh.set_point(v, p);
    assertx(!get_sinfo(sline));  // No longer supported.  Too hard.
    if (record_changes) std::cout << sline << '\n';
  } else if (sline[0] == 'C' && !strncmp(sline, "Corner ", 7)) {
    if (save0.skip_current) return true;
    if (!after_vsplit) return true;
    int vi;
    int fi;
    assertx(sscanf(sline, "Corner %d %d", &vi, &fi) == 2);
    Vertex v = mesh.id_vertex(vlineage.existing_id(vi));
    Corner c = nullptr;
    for (Corner cc : mesh.corners(v)) {
      if (mesh.face_id(mesh.corner_face(cc)) == fi) {
        c = cc;
        break;
      }
    }
    if (!c) {
      Warning("Skipping non-existent corner");
      return true;
    }
    const char* sinfo = get_sinfo(sline);  // note side-effect!
    string str;
    int wid = assertx(to_int(assertx(GMesh::string_key(str, sinfo, "wid"))));
    c_cwedge_id(c) = wid;
    if (save0.wid_read.add(wid)) {
      mesh.set_string(c, sinfo);
      WedgeInfo wi = create_winfo(c);
      mesh.set_string(c, nullptr);
      gcwinfo.set(wid, wi);
    }
    if (record_changes) std::cout << orig_sline << '\n';
  } else {
    assertnever(string() + "Cannot parse line '" + sline + "'");
  }
  // Can return earlier!
  return true;
}

// Read and parse a vsplit record.
void read_record(std::istream& is, bool carry_old) {
  bool after_vsplit = false;
  for (string sline; my_getline(is, sline);) {
    if (!parse_line(const_cast<char*>(sline.c_str()), after_vsplit, carry_old)) break;
  }
  if (is) assertx(after_vsplit);
}

// Look around a vertex to see if the corners have a unique wid.
// Return -1 if they don't.
int get_unique_wid(Vertex v, bool use_old) {
  int num = 0, u_wid;
  dummy_init(u_wid);
  bool same_wid = true;
  for (Corner c : mesh.corners(v)) {
    int wid = !use_old ? c_cwedge_id(c) : c_owedge_id(c);
    if (!num++) {
      u_wid = wid;
    } else {
      if (u_wid != wid) same_wid = false;
    }
  }
  assertx(num);
  return same_wid ? u_wid : -1;
}

// Create a portion of the corner string from a wedge info.
string create_attrib_string(const WedgeInfo& wi, bool old_flag) {
  const Vector& nor = wi.nor;
  const A3dColor& col = wi.col;
  const UV& uv = wi.uv;
  const char* sflag = old_flag ? "O" : "";
  string sret;
  if (nor[0] != k_undefined) sret += sform(" %snormal=(%g %g %g)", sflag, nor[0], nor[1], nor[2]);
  if (col[0] != k_undefined) sret += sform(" %srgb=(%g %g %g)", sflag, col[0], col[1], col[2]);
  if (uv[0] != k_undefined) sret += sform(" %suv=(%g %g)", sflag, uv[0], uv[1]);
  return sret;
}

// Create the Opos portion of the corner or vertex string
string write_p_string(Vertex v) {
  const Point& op = v_opos(v);
  return sform("Opos=(%g %g %g)", op[0], op[1], op[2]);
}

string write_a_string(int wid, int owid, int write_morph) {
  assertx(wid > 0);
  const WedgeInfo& wi = gcwinfo.get(wid);
  const WedgeInfo* powi = nullptr;
  if (write_morph) {
    assertx(owid >= 0);
    powi = owid ? &gowinfo.get(owid) : &wi;
  }
  return sform("wid=%d", wid) + create_attrib_string(wi, false) + (powi ? create_attrib_string(*powi, true) : "");
}

string write_pa_string(Vertex v, int wid, int owid, bool write_morph) {
  if (write_morph)
    return write_p_string(v) + " " + write_a_string(wid, owid, write_morph);
  else
    return write_a_string(wid, owid, write_morph);
}

// Prepare the vertex and corner strings of v for the mesh write.
void write_init_vertex(Vertex v, bool write_morph) {
  int u_nwid = get_unique_wid(v, false);
  int u_owid = write_morph ? get_unique_wid(v, true) : 0;
  assertx(u_nwid != 0);  // impossible
  if (u_nwid > 0 && u_owid < 0) {
    for (Corner c : mesh.corners(v)) {
      // int nwid = c_cwedge_id(c);
      int owid = c_owedge_id(c);
      // Rare case in which partial sharp edge collapses into
      //  a boundary vertex.
      if (owid && owid != u_nwid) {
        Warning("Rare case of u_nwid > 0 && u_owid < 0");
        u_nwid = 0;
        break;
      }
    }
  }
  if (u_nwid > 0) {
    if (u_owid < 0) u_owid = 0;
    mesh.set_string(v, write_pa_string(v, u_nwid, u_owid, write_morph).c_str());
  } else {
    mesh.set_string(v, write_morph ? write_p_string(v).c_str() : nullptr);
    for (Corner c : mesh.corners(v)) {
      mesh.set_string(c, write_a_string(c_cwedge_id(c), c_owedge_id(c), write_morph).c_str());
    }
  }
}

// Write the current mesh out, possibly including Old attributes if a morph.
void write_mesh(bool write_morph = false) {
  // Write separator if multiple writes.
  static int nwrites = 0;
  if (nwrites++) {
    std::cout << "o 1 1 0\n";  // object separator
  }
  if (splitcorners) {
    GMesh nmesh;
    Map<Corner, Vertex> mcv;
    for (Vertex v : mesh.vertices()) {
      Set<Corner> setcvis;
      for (Corner crep : mesh.corners(v)) {
        if (setcvis.contains(crep)) continue;
        int crepcwid = c_cwedge_id(crep);
        Vertex vn = nmesh.create_vertex();
        nmesh.set_point(vn, mesh.point(mesh.corner_vertex(crep)));
        nmesh.set_string(vn, write_pa_string(v, crepcwid, c_owedge_id(crep), write_morph).c_str());
        mcv.enter(crep, vn);
        for_int(dir, 2) {
          for (Corner c = crep;;) {
            c = !dir ? mesh.ccw_corner(c) : mesh.clw_corner(c);
            if (!c || c == crep || c_cwedge_id(c) != crepcwid || !setcvis.add(c)) break;
            mcv.enter(c, vn);
          }
        }
      }
    }
    for (Face f : mesh.faces()) {
      Array<Vertex> van;
      for (Corner c : mesh.corners(f)) van.push(mcv.get(c));
      Face fn = nmesh.create_face(van);
      nmesh.set_string(fn, mesh.get_string(f));
    }
    {
      HH_ATIMER("__mesh_write_split");
      nmesh.write(std::cout);
      showdf("Wrote %s\n", mesh_genus_string(nmesh).c_str());
    }
  } else {
    for (Vertex v : mesh.vertices()) write_init_vertex(v, write_morph);
    {
      HH_ATIMER("__mesh_write");
      mesh.write(std::cout);
    }
    showdf("Wrote %s\n", mesh_genus_string(mesh).c_str());
    clear_mesh_strings();
  }
}

// Parse sequence of vsplit records until nfaces, forming a morph, and write the morph out.
bool next_morph(int nfaces) {
  if (mesh.num_faces() >= nfaces) return false;
  // Record attributes of current mesh.
  {
    HH_ATIMER("__record_attrib");
    for (Vertex v : mesh.vertices()) {
      v_opos(v) = mesh.point(v);
      for (Corner c : mesh.corners(v)) c_owedge_id(c) = c_cwedge_id(c);
    }
    gowinfo.copy_from(gcwinfo);
  }
  RFile& fi_prog = *assertx(pfi_prog);
  std::istream& is = fi_prog();
  {
    HH_ATIMER("__read_records");
    for (;;) {  // Read a record and refine mesh.
      read_record(is, true);
      if (!is || mesh.num_faces() >= nfaces) break;
    }
  }
  {
    int num_will_be_constant = 0;
    for (Vertex v : mesh.vertices()) {
      for (Corner c : mesh.corners(v)) {
        assertx(c_owedge_id(c) >= 0);
        if (!c_owedge_id(c)) num_will_be_constant++;
      }
    }
    if (num_will_be_constant) showdf("(%d corners not interpolated)\n", num_will_be_constant);
  }
  // Write morph.
  // Add current and old strings, write, and clear all strings.
  write_mesh(true);
  // Clear old info.
  gowinfo.clear();
  for (Vertex v : mesh.vertices()) {
    v_opos(v) = Point(k_undefined, k_undefined, k_undefined);  // optional
    for (Corner c : mesh.corners(v)) c_owedge_id(c) = -1;      // optional
  }
  return !!is;
}

// Read the base mesh.
void do_fbasemesh(Args& args) {
  {
    // HH_TIMER("_read_mesh");
    RFile fi(args.get_filename());
    int nmaterials = 0;
    for (string sline; fi().peek() == '#';) {
      assertx(my_getline(fi(), sline));
      if (nmaterials) {
        --nmaterials;
        assertx(sline.size() > 1);
        pm_material_strings.push(sline.substr(2));
        continue;
      }
      if (sline.size() < 2) continue;
      showff("|%s\n", sline.substr(2).c_str());
      int n1, n2, n3;
      Bbox bbox;
      if (sscanf(sline.c_str(), "# nmaterials=%d", &n1) == 1) {
        nmaterials = n1;
      } else if (sscanf(sline.c_str(), "# PM: nvertices=%d nwedges=%d nfaces=%d", &n1, &n2, &n3) == 3) {
        pm_nvertices = n1;
        pm_nwedges = n2;
        pm_nfaces = n3;
      } else if (sscanf(sline.c_str(), "# PM: necols=%d", &n1) == 1) {
        pm_nvsplits = n1;
      } else if (sscanf(sline.c_str(), "# PM: bounding box %g %g %g  %g %g %g", &bbox[0][0], &bbox[0][1], &bbox[0][2],
                        &bbox[1][0], &bbox[1][1], &bbox[1][2]) == 6) {
        pm_bbox = bbox;
      } else if (begins_with(sline, "# PM: has_wad2")) {
        pm_has_wad2 = true;
      }
    }
    showff("%s", g_header.c_str());
    showff("Found %d materials\n", pm_material_strings.num());
    mesh = GMesh(fi());
  }
  // HH_TIMER("_parse_mesh");
  showdf("Read %s\n", mesh_genus_string(mesh).c_str());
  string str;
  for (Vertex v : mesh.vertices()) {
    for (Corner c : mesh.corners(v)) {
      const char* s = assertx(mesh.corner_key(str, c, "wid"));
      int wid = assertx(to_int(s));
      c_cwedge_id(c) = wid;
      if (!gcwinfo.contains(wid)) gcwinfo.set(wid, create_winfo(c));
    }
  }
  for (Edge e : mesh.edges()) assertx(!mesh.get_string(e));
  clear_mesh_strings();
}

// Open the PM file stream.
void do_fprogressive(Args& args) { pfi_prog = make_unique<RFile>(args.get_filename()); }

// Write a sequence of morphs forming an arithmetic sequence.
void do_arithseq(Args& args) {
  int deltanf = args.get_int();
  for (;;) {
    int nfaces = mesh.num_faces() + deltanf;
    if (maxnfaces) nfaces = min(nfaces, maxnfaces);
    if (!next_morph(nfaces)) break;
  }
}

// Write a sequence of morphs forming a geometric sequence.
void do_geomseq(Args& args) {
  float factornf = args.get_float();
  if (maxnfaces) {
    float oldfactornf = factornf;
    int cnfaces = mesh.num_faces();
    int wnfaces = maxnfaces;
    const int fudge = 1;
    float ratio = float(wnfaces + fudge) / cnfaces;
    int nsteps = int(std::log(ratio) / std::log(factornf) + .5f);
    factornf = pow(ratio, 1.f / nsteps);
    showdf("rounded factor %.3f to %.3f for %d steps (%d to %d faces)\n",  //
           oldfactornf, factornf, nsteps, cnfaces, wnfaces);
  }
  float fnfaces = float(mesh.num_faces());
  for (;;) {
    fnfaces *= factornf;
    int nfaces = int(fnfaces);
    if (nfaces <= mesh.num_faces()) continue;
    if (maxnfaces) nfaces = min(nfaces, maxnfaces);
    if (!next_morph(nfaces)) break;
  }
}

// Write a single morph from the current mesh up to the mesh with nfaces.
void do_to(Args& args) {
  int nfaces = args.get_int();
  next_morph(nfaces);
}

// Parse vsplit records until mesh has nfaces.
void do_from(Args& args) {
  int nfaces = args.get_int();
  RFile& fi_prog = *assertx(pfi_prog);
  std::istream& is = fi_prog();
  {
    HH_ATIMER("__skip_records");
    for (;;) {  // Read a record and refine mesh.
      if (mesh.num_faces() >= nfaces) break;
      read_record(is, false);
      if (!is) break;
    }
  }
}

// Parse a fixed number of vsplit records.  (Useful for considering a fixed
//  PM prefix in conjunction with sel_refinement.)
void do_consider(Args& args) {
  int nrecords = args.get_int();
  RFile& fi_prog = *assertx(pfi_prog);
  std::istream& is = fi_prog();
  int nrec = 0;
  {
    HH_ATIMER("__skip_records");
    for (;;) {  // Read a record and refine mesh.
      if (nrec >= nrecords) break;
      read_record(is, false);
      if (!is) break;
      nrec++;
    }
  }
}

// Write current mesh.
void do_mesh() { write_mesh(); }

// Parse vsplit records until nfaces, then write mesh.
void do_at(Args& args) {
  do_from(args);
  do_mesh();
}

// Begin sel_refinement using the specified s3d file as viewing parameter.
void do_view(Args& args) {
  string filename = args.get_string();  // may be "" on next line
  if (filename == "") {
    sel_refinement = false;
    return;
  }
  RFile fi(filename);
  for (;;) {
    if (fi().peek() < 0) break;
    ObjectFrame object_frame;
    if (!assertw(FrameIO::read(fi(), object_frame))) break;
    if (object_frame.obn) {
      Warning("Skipping non-object0 frame");
      continue;
    }
    sel_refinement = true;  // must activate vlineage!
    vlineage.activate();
    view_frame = object_frame.frame;
    view_zoom = object_frame.zoom;
    view_iframe = inverse(view_frame);
  }
}

// Parse vsplit records until mesh has nfaces, recording changes on stdout.
void do_animateto(Args& args) {
  int nfaces = args.get_int();
  RFile& fi_prog = *assertx(pfi_prog);
  std::istream& is = fi_prog();
  {
    write_mesh();
    record_changes = true;
    HH_ATIMER("__animate");
    for (;;) {  // Read a record and refine mesh.
      read_record(is, false);
      if (!is || mesh.num_faces() >= nfaces) break;
    }
    record_changes = false;
  }
}

// *** PM Encoding

constexpr bool has_normal = true;
bool has_rgb = false;
bool has_uv = false;
bool has_resid = false;

int retrieve_wid(Corner c) { return !c ? 0 : assertx(c_cwedge_id(c)); }

unsigned encode_wst(int wid_l, int wid_r, int wid_lo, int wid_ro) {
  if (!wid_r) {
    assertx(!wid_ro);
    return Vsplit::B_RSAME | (wid_l == wid_lo ? Vsplit::B_LSAME : 0u);
  } else {
    return ((wid_l == wid_lo ? Vsplit::B_LSAME : 0u) | (wid_r == wid_ro ? Vsplit::B_RSAME : 0u) |
            (wid_l == wid_r ? Vsplit::B_CSAME : 0u));
  }
}

unsigned encode_wlr(int wid_s, int wid_a, int wid_b) {
  if (wid_s == wid_a)
    return Vsplit::B_ABOVE;
  else if (wid_s == wid_b)
    return Vsplit::B_BELOW;
  else
    return Vsplit::B_NEW;
}

int parse_matid(Face f) {
  string str;
  int i = to_int(assertx(GMesh::string_key(str, mesh.get_string(f), "matid")));
  assertx(i >= 0);
  return i;
}

int face_prediction(Face f, Face fa, Face fb, int ii) {
  assertx(fa || fb);
  int m = parse_matid(f);
  int ma = fa ? parse_matid(fa) : -1;
  int mb = fb ? parse_matid(fb) : -1;
  int predicted_matid = (ii == 0 ? (mb >= 0 ? mb : ma) : (ma >= 0 ? ma : mb));
  return m == predicted_matid ? -1 : m;
}

PmWedgeAttrib get_wattrib(Corner c) {
  assertx(c);
  PmWedgeAttrib wa;
  int wid = c_cwedge_id(c);
  const WedgeInfo& wi = gcwinfo.get(wid);
  assertx(has_normal);
  assertx(wi.nor[0] != k_undefined);
  wa.normal = wi.nor;
  fill(wa.rgb, 0.f);
  fill(wa.uv, 0.f);
  if (has_rgb) {
    assertx(wi.col[0] != k_undefined);
    wa.rgb = wi.col;
  }
  if (has_uv) {
    assertx(wi.uv[0] != k_undefined);
    wa.uv = wi.uv;
  }
  if (!has_rgb) assertx(wi.col[0] == k_undefined);
  if (!has_uv) assertx(wi.uv[0] == k_undefined || (!wi.uv[0] && !wi.uv[1]));
  return wa;
}

PmWedgeAttrib retrieve_wattrib(Corner c) {
  if (c) return get_wattrib(c);
  // to help debug
  PmWedgeAttrib wa;
  fill(wa.normal, BIGFLOAT);
  fill(wa.rgb, BIGFLOAT);
  fill(wa.uv, BIGFLOAT);
  return wa;
}

PmWedgeAttribD diff(const PmWedgeAttrib& a1, const PmWedgeAttrib& a2) {
  PmWedgeAttribD ad;
  diff(ad, a1, a2);
  return ad;
}

PmWedgeAttribD diff_zero(const PmWedgeAttrib& a1) {
  static PmWedgeAttrib zero;  // cannot declare const because default constructor leaves uninitialized
  PmWedgeAttribD ad;
  diff(ad, a1, zero);
  return ad;
}

Map<Face, int> mfrenumber;  // contains index of Face (starting with 0)

// For append_old_pm, information for faces in old base mesh; these have been modified.
Array<int> append_f_renumber;    // old face index -> new face index
Array<int> append_f_vsi_offset;  // old face index -> offset for vsindex

struct _save {
  // Prior to vsplit
  PmVertexAttrib vaovs;
  PmWedgeAttrib wavtflo, wavsflo, wavtfro, wavsfro;
  // Vsplit parameters
  Vertex vs, vt, vl, vr;  // vr may be nullptr
  int ii;
  // After vsplit
  Set<int> wid_read;
};
_save save;

Vsplit vspl;

void process_vsplit() {
  unsigned code = vspl.code;
  save.wid_read.clear();
  Edge e = mesh.edge(save.vs, save.vt);
  // Analyze wedges.
  {
    Corner cl = mesh.ccw_corner(save.vs, e);
    Corner cr = mesh.clw_corner(save.vs, e);
    int wid_vsfl = retrieve_wid(cl);
    int wid_vsfr = retrieve_wid(cr);
    assertx(wid_vsfl);
    assertx(!wid_vsfr == !save.vr);
    int wid_vsflo = retrieve_wid(mesh.ccw_corner(cl));
    int wid_vsfro = retrieve_wid(cr ? mesh.clw_corner(cr) : nullptr);
    code |= (encode_wst(wid_vsfl, wid_vsfr, wid_vsflo, wid_vsfro) << Vsplit::S_SHIFT);
  }
  {
    Corner cl = mesh.clw_corner(save.vt, e);
    Corner cr = mesh.ccw_corner(save.vt, e);
    int wid_vtfl = retrieve_wid(cl);
    int wid_vtfr = retrieve_wid(cr);
    assertx(wid_vtfl);
    assertx(!wid_vtfr == !save.vr);
    int wid_vtflo = retrieve_wid(mesh.clw_corner(cl));
    int wid_vtfro = retrieve_wid(cr ? mesh.ccw_corner(cr) : nullptr);
    code |= (encode_wst(wid_vtfl, wid_vtfr, wid_vtflo, wid_vtfro) << Vsplit::T_SHIFT);
  }
  if (1) {
    Corner c = mesh.corner(save.vl, mesh.face(save.vl, save.vs));
    assertx(mesh.corner_vertex(mesh.clw_face_corner(c)) == save.vt);
    int wid_vlfl = assertx(retrieve_wid(c));
    int wid_a = retrieve_wid(mesh.ccw_corner(c));
    int wid_b = retrieve_wid(mesh.clw_corner(c));
    code |= (encode_wlr(wid_vlfl, wid_a, wid_b) << Vsplit::L_SHIFT);
  }
  if (!save.vr) {
    code |= Vsplit::R_ABOVE;
  } else {
    Corner c = mesh.corner(save.vr, mesh.face(save.vr, save.vt));
    assertx(mesh.corner_vertex(mesh.clw_face_corner(c)) == save.vs);
    int wid_vrfr = assertx(retrieve_wid(c));
    int wid_a = retrieve_wid(mesh.clw_corner(c));
    int wid_b = retrieve_wid(mesh.ccw_corner(c));
    code |= (encode_wlr(wid_vrfr, wid_a, wid_b) << Vsplit::R_SHIFT);
  }
  // Predict face attributes.
  Face fl = mesh.ccw_face(save.vs, e);
  Face fr = mesh.clw_face(save.vs, e);
  if (1) {
    assertx(fl);
    Face fa = mesh.clw_face(save.vt, fl);
    Face fb = mesh.ccw_face(save.vs, fl);
    int matid = face_prediction(fl, fa, fb, save.ii);
    // HH_SSTAT(Sflmatid, matid >= 0);
    if (matid < 0) {
      vspl.fl_matid = 0;
    } else {
      vspl.fl_matid = narrow_cast<ushort>(matid);
      code |= Vsplit::FLN_MASK;
    }
  }
  if (save.vr) {
    assertx(fr);
    Face fa = mesh.ccw_face(save.vt, fr);
    Face fb = mesh.clw_face(save.vs, fr);
    int matid = face_prediction(fr, fa, fb, save.ii);
    // HH_SSTAT(Sfrmatid, matid >= 0);
    if (matid < 0) {
      vspl.fr_matid = 0;
    } else {
      vspl.fr_matid = narrow_cast<ushort>(matid);
      code |= Vsplit::FRN_MASK;
    }
  }
  vspl.code = narrow_cast<ushort>(code);
  // Encode vertex attributes.
  {
    PmVertexAttrib vas;
    vas.point = mesh.point(save.vs);
    PmVertexAttrib vat;
    vat.point = mesh.point(save.vt);
    switch (save.ii) {
      case 2:
        diff(vspl.vad_large, vat, save.vaovs);
        diff(vspl.vad_small, vas, save.vaovs);
        break;
      case 0:
        diff(vspl.vad_large, vas, save.vaovs);
        diff(vspl.vad_small, vat, save.vaovs);
        break;
      case 1: {
        PmVertexAttrib vam;
        interp(vam, vas, vat, 0.5f);
        diff(vspl.vad_large, vat, vam);
        diff(vspl.vad_small, vam, save.vaovs);
        break;
      }
      default: assertnever("");
    }
  }
  // Encode wedge attributes.
  vspl.ar_wad.init(0);
  static const bool big_rgb = getenv_bool("BIG_RGB");
  const bool verify_reflection = !pm_has_wad2 && !big_rgb;
  const float tolerance = 1e-4f;
  if (1) {
    bool nt = !(code & Vsplit::T_LSAME);
    bool ns = !(code & Vsplit::S_LSAME);
    if (nt && ns) {
      vspl.ar_wad.push(diff_zero(get_wattrib(mesh.corner(save.vt, fl))));
      vspl.ar_wad.push(diff_zero(get_wattrib(mesh.corner(save.vs, fl))));
    } else {
      const PmWedgeAttrib& waovsfl = (code & Vsplit::S_LSAME) ? save.wavsflo : save.wavsfro;
      const PmWedgeAttrib& waovtfl = (code & Vsplit::T_LSAME) ? save.wavtflo : save.wavtfro;
      switch (save.ii) {
        case 2:
          vspl.ar_wad.push(diff(get_wattrib(mesh.corner(save.vt, fl)), !ns ? waovsfl : waovtfl));
          if (verify_reflection)
            assertx(!compare(get_wattrib(mesh.corner(save.vs, fl)), !nt ? waovtfl : waovsfl, tolerance));
          break;
        case 0:
          vspl.ar_wad.push(diff(get_wattrib(mesh.corner(save.vs, fl)), !nt ? waovtfl : waovsfl));
          if (verify_reflection)
            assertx(!compare(get_wattrib(mesh.corner(save.vt, fl)), !ns ? waovsfl : waovtfl, tolerance));
          break;
        case 1:
          // Added check on ns to deal with (ws=5 wt=6) case.
          vspl.ar_wad.push(diff(get_wattrib(mesh.corner(save.vt, fl)), !ns ? waovsfl : waovtfl));
          if (verify_reflection) {
            PmWedgeAttrib wat;
            sub_reflect(wat, !ns ? waovsfl : waovtfl, vspl.ar_wad.last());
            assertw(!compare(get_wattrib(mesh.corner(save.vs, fl)), wat, tolerance));
          }
          break;
        default: assertnever("");
      }
    }
  }
  if (save.vr) {
    bool nt = !(code & Vsplit::T_RSAME);
    bool ns = !(code & Vsplit::S_RSAME);
    bool ut = !(code & Vsplit::T_CSAME);
    bool us = !(code & Vsplit::S_CSAME);
    if (nt && ns) {
      if (ut) vspl.ar_wad.push(diff_zero(get_wattrib(mesh.corner(save.vt, fr))));
      if (us) vspl.ar_wad.push(diff_zero(get_wattrib(mesh.corner(save.vs, fr))));
    } else {
      const PmWedgeAttrib& waovsfr = (code & Vsplit::S_RSAME) ? save.wavsfro : save.wavsflo;
      const PmWedgeAttrib& waovtfr = (code & Vsplit::T_RSAME) ? save.wavtfro : save.wavtflo;
      switch (save.ii) {
        case 2:
          if (ut) vspl.ar_wad.push(diff(get_wattrib(mesh.corner(save.vt, fr)), !ns ? waovsfr : waovtfr));
          break;
        case 0:
          if (us) vspl.ar_wad.push(diff(get_wattrib(mesh.corner(save.vs, fr)), !nt ? waovtfr : waovsfr));
          break;
        case 1:
          // Added check on ns only for symmetry with above.
          if (us || ut) vspl.ar_wad.push(diff(get_wattrib(mesh.corner(save.vt, fr)), !ns ? waovsfr : waovtfr));
          break;
        default: assertnever("");
      }
    }
  }
  if (code & Vsplit::L_NEW) vspl.ar_wad.push(diff_zero(get_wattrib(mesh.corner(save.vl, fl))));
  if (code & Vsplit::R_NEW) vspl.ar_wad.push(diff_zero(get_wattrib(mesh.corner(save.vr, fr))));
  if (pm_has_wad2) {
    assertx(vspl.ar_wad.num() == 1);
    bool ns = !(code & Vsplit::S_LSAME);
    vspl.ar_wad.init(0);
    const PmWedgeAttrib& waovsfl = (code & Vsplit::S_LSAME) ? save.wavsflo : save.wavsfro;
    const PmWedgeAttrib& waovtfl = (code & Vsplit::T_LSAME) ? save.wavtflo : save.wavtfro;
    const PmWedgeAttrib& wao = ns ? waovtfl : waovsfl;
    vspl.ar_wad.push(diff(get_wattrib(mesh.corner(save.vt, fl)), wao));
    vspl.ar_wad.push(diff(get_wattrib(mesh.corner(save.vs, fl)), wao));
  }
  // HH_SSTAT(Swadnum, vspl.ar_wad.num());
  if (sdebug) vspl.ok();
}

// Read and parse a single line of a vsplit record.
// Ret: 0 at end_of_record.
bool parse_line2(char* sline, bool& after_vsplit) {
  // Adapted from GMesh::read_line().
  if (sline[0] == '#') {
    if (!strcmp(sline, "# Beg REcol")) {
      // End of record.
      process_vsplit();
      return false;
    }
    if (!strncmp(sline, "# Residuals", 11)) {
      assertx(sscanf(sline, "# Residuals %g %g", &vspl.resid_uni, &vspl.resid_dir) == 2);
      has_resid = true;
    }
    return true;
  }
  if (!strncmp(sline, "REcol ", 6)) {
    assertx(!after_vsplit);
    after_vsplit = true;
    int voi, vsi, vti, vli, vri, fli, fri, ii;
    assertx(sscanf(sline, "REcol %d %d %d %d %d %d %d %d", &voi, &vsi, &vti, &vli, &vri, &fli, &fri, &ii) == 8);
    assertx(voi == vsi);  // redundancy
    Vertex vs = mesh.id_vertex(vsi);
    Vertex vl = mesh.id_vertex(vli), vr = vri ? mesh.id_vertex(vri) : nullptr;
    unsigned code = 0;
    assertx(ii >= 0 && ii <= 2);
    code |= (ii << Vsplit::II_SHIFT);
    Edge esl = mesh.edge(vs, vl);
    Edge esr = vr ? mesh.edge(vs, vr) : nullptr;
    {
      if (!vr) {
        // No vr, so simply pick one existing face adjacent to esl.
        if (mesh.clw_face(vs, esl)) {
          vspl.vlr_offset1 = 1;  // special code
        } else if (mesh.ccw_face(vs, esl)) {
          vspl.vlr_offset1 = 0;  // special code; flclw does not exist!
        } else {
          assertnever("");
        }
      } else {
        // Find number of CLW rotations from vl to vr.
        Vertex v = vl;
        int j = 0;
        for (;;) {
          j++;
          v = mesh.clw_vertex(vs, v);
          assertx(v && v != vl);
          if (v == vr) break;
        }
        vspl.vlr_offset1 = narrow_cast<short>(j + 1);
        assertx(vspl.vlr_offset1 == j + 1);
      }
    }
    // Get one face adjacent to (vs, vl).
    Face f = vspl.vlr_offset1 == 0 ? mesh.ccw_face(vs, esl) : mesh.clw_face(vs, esl);
    assertx(f);
    vspl.flclw = mfrenumber.get(f);
    assertx(vspl.flclw < mesh.num_faces());
    // Encode location of vs within that face.
    {
      Vec3<Vertex> va = mesh.triangle_vertices(f);
      int vs_index = va.view().index(vs);
      assertx(vs_index >= 0 && vs_index <= 2);
      code |= (vs_index << Vsplit::VSINDEX_SHIFT);
    }
    vspl.code = narrow_cast<ushort>(code);
    // HH_SSTAT(Svspl_vlr1, vspl.vlr_offset1);
    // Save current vertex attributes.
    save.vaovs.point = mesh.point(vs);
    // Save current wedge attributes.
    save.wavtflo = retrieve_wattrib(!esl ? nullptr : mesh.clw_corner(vs, esl));
    save.wavsflo = retrieve_wattrib(!esl ? nullptr : mesh.ccw_corner(vs, esl));
    save.wavtfro = retrieve_wattrib(!esr ? nullptr : mesh.ccw_corner(vs, esr));
    save.wavsfro = retrieve_wattrib(!esr ? nullptr : mesh.clw_corner(vs, esr));
    // DO IT:
    Vertex vt = mesh.split_vertex(vs, vl, vr, vti);
    // Save what was done
    save.vs = vs;
    save.vt = vt;
    save.ii = ii;
    save.vl = vl;
    save.vr = vr;
  } else if (sline[0] == 'F' && !strncmp(sline, "Face ", 5)) {
    // NOTE: because of reverselines, face2 arrives before face1!
    // Note: still true with std::swap(vs, vt) and collapse_edge_vertex
    const char* sinfo = get_sinfo(sline);  // note side-effect!
    PArray<Vertex, 8> va;
    char* s = sline + 4;
    int fi = -1;
    for (;;) {
      while (*s && isspace(*s)) s++;
      if (!*s) break;
      char* beg = s;
      while (*s && isdigit(*s)) s++;
      assertx(!*s || isspace(*s));
      int j = atoi(beg);  // terminated by ' ' so cannot use to_int()
      if (fi < 0) {
        fi = j;
        continue;
      }
      va.push(mesh.id_vertex(j));
    }
    assertx(va.num() == 3);
    assertx(mesh.legal_create_face(va));
    bool is_face2 = false;
    int voffset = 0;
    if (va.contains(save.vl)) {
      if (va[0] == save.vs) {
        voffset = 0;
      } else if (va[1] == save.vs) {
        voffset = 2;
        va[1] = va[2];
        va[2] = va[0];
        va[0] = save.vs;
      } else if (va[2] == save.vs) {
        voffset = 1;
        va[2] = va[1];
        va[1] = va[0];
        va[0] = save.vs;
      } else {
        assertnever("");
      }
    } else if (va.contains(save.vr)) {
      is_face2 = true;
      if (va[0] == save.vs) {
        voffset = 0;
      } else if (va[1] == save.vs) {
        voffset = 2;
        va[1] = va[2];
        va[2] = va[0];
        va[0] = save.vs;
      } else if (va[2] == save.vs) {
        voffset = 1;
        va[2] = va[1];
        va[1] = va[0];
        va[0] = save.vs;
      } else {
        assertnever("");
      }
    } else {
      assertnever("");
    }
    assertx(mesh.legal_create_face(va));
    assertx(fi >= 1);
    Face f = mesh.create_face_private(fi, va);
    int renum;
    if (save.vr) {
      // reverse the order
      renum = mesh.num_faces() + (is_face2 ? 0 : -2);
    } else {
      renum = mesh.num_faces() - 1;
    }
    mfrenumber.enter(f, renum);
    if (append_old_pm != "") {
      int old_f_num = fi - 1;
      append_f_renumber.access(old_f_num);
      append_f_renumber[old_f_num] = renum;
      append_f_vsi_offset.access(old_f_num);
      append_f_vsi_offset[old_f_num] = voffset;
    }
    if (sinfo) mesh.set_string(f, sinfo);
  } else if (sline[0] == 'E' && !strncmp(sline, "Edge ", 5)) {
    assertnever("Edge flags not used anymore");
  } else if (!strncmp(sline, "MVertex ", 8)) {
    if (!after_vsplit) return true;
    Point p;
    int vi;
    assertx(sscanf(sline, "MVertex %d %g %g %g", &vi, &p[0], &p[1], &p[2]) == 4);
    Vertex v = mesh.id_vertex(vi);
    mesh.set_point(v, p);
    assertx(!get_sinfo(sline));  // No longer supported.  Too hard.
  } else if (sline[0] == 'C' && !strncmp(sline, "Corner ", 7)) {
    if (!after_vsplit) return true;
    int vi;
    int fi;
    assertx(sscanf(sline, "Corner %d %d", &vi, &fi) == 2);
    Vertex v = mesh.id_vertex(vi);
    Corner c = nullptr;
    for (Corner cc : mesh.corners(v)) {
      if (mesh.face_id(mesh.corner_face(cc)) == fi) {
        c = cc;
        break;
      }
    }
    assertx(c);
    const char* sinfo = get_sinfo(sline);  // note side-effect!
    string str;
    int wid = assertx(to_int(assertx(GMesh::string_key(str, sinfo, "wid"))));
    c_cwedge_id(c) = wid;
    if (save.wid_read.add(wid)) {
      mesh.set_string(c, sinfo);
      WedgeInfo wi = create_winfo(c);
      mesh.set_string(c, nullptr);
      gcwinfo.set(wid, wi);
    }
  } else {
    assertnever(string() + "Cannot parse line '" + sline + "'");
  }
  // Can return earlier!
  return true;
}

void do_pm_encode() {
  PMesh pmesh;
  // Create base mesh.
  AWMesh& bmesh = pmesh._base_mesh;
  for_int(i, pm_material_strings.num()) bmesh._materials.set(i, pm_material_strings[i]);
  Map<Vertex, int> mvrenumber;
  {
    bmesh._vertices.init(mesh.num_vertices());
    int i = 0;
    // Ordered for StitchPM.
    for (Vertex v : mesh.ordered_vertices()) {
      bmesh._vertices[i].attrib.point = mesh.point(v);
      mvrenumber.enter(v, i);
      i++;
    }
  }
  for (Face f : mesh.faces()) {
    for (Corner c : mesh.corners(f)) {
      int wid = c_cwedge_id(c);
      const WedgeInfo& wi = gcwinfo.get(wid);
      // has_normal=true already.
      if (wi.col[0] != k_undefined) has_rgb = true;
      if (wi.uv[0] != k_undefined && wi.uv[0] && wi.uv[1]) has_uv = true;
    }
  }
  Map<int, int> mwrenumber;
  {
    bmesh._wedges.init(mesh.num_vertices());  // will likely increase!
    int i0 = 0;
    // Ordered for StitchPM.
    for (Vertex v : mesh.ordered_vertices()) {
      int rvi = mvrenumber.get(v);
      int nunique = 0;
      for (Corner c : mesh.corners(v)) {
        int wid = c_cwedge_id(c);
        bool is_new;
        int& i = mwrenumber.enter(wid, 0, is_new);
        if (!is_new) continue;
        i = !nunique++ ? i0++ : bmesh._wedges.add(1);
        bmesh._wedges[i].vertex = rvi;
        PmWedgeAttrib& wa = bmesh._wedges[i].attrib;
        const WedgeInfo& wi = gcwinfo.get(wid);
        assertx(has_normal);
        assertx(wi.nor[0] != k_undefined);
        wa.normal = wi.nor;
        fill(wa.rgb, 0.f);
        if (has_rgb) {
          assertx(wi.col[0] != k_undefined);
          wa.rgb = wi.col;
        } else if (has_uv) {
          assertx(wi.uv[0] != k_undefined);
          wa.uv = wi.uv;
        }
      }
    }
    assertx(i0 == mesh.num_vertices());
  }
  {
    bmesh._faces.init(mesh.num_faces());
    {
      int i = 0;
      string str;
      // Ordered just in case (maybe unnecessarily).
      for (Face f : mesh.ordered_faces()) {
        mfrenumber.enter(f, i);
        if (append_old_pm != "") {
          int old_f_num = mesh.face_id(f) - 1;
          // face renumbered
          append_f_renumber.access(old_f_num);
          append_f_renumber[old_f_num] = i;
          // vertex order within face unchanged
          append_f_vsi_offset.access(old_f_num);
          append_f_vsi_offset[old_f_num] = 0;
        }
        int j = 0;
        for (Corner c : mesh.corners(f)) {
          int wid = c_cwedge_id(c);
          int rwid = mwrenumber.get(wid);
          bmesh._faces[i].wedges[j] = rwid;
          j++;
        }
        int matid = to_int(assertx(GMesh::string_key(str, mesh.get_string(f), "matid")));
        bmesh._faces[i].attrib.matid = matid;
        i++;
      }
    }
    bmesh._fnei.init(mesh.num_faces());
    for (Face f : mesh.faces()) {
      int i = mfrenumber.get(f);
      int j = 0;
      for (Corner c : mesh.corners(f)) {
        Face fn = mesh.opp_face(mesh.corner_vertex(c), f);
        int fni = fn ? mfrenumber.get(fn) : -1;
        bmesh._fnei[i].faces[j] = fni;
        j++;
      }
    }
  }
  // Information about the original detailed mesh.
  pmesh._info._tot_nvsplits = pm_nvsplits;
  pmesh._info._full_nvertices = pm_nvertices;
  pmesh._info._full_nwedges = pm_nwedges;
  pmesh._info._full_nfaces = pm_nfaces;
  pmesh._info._full_bbox = pm_bbox;
  pmesh._info._has_wad2 = pm_has_wad2;
  pmesh._vsplits.reserve(pmesh._info._tot_nvsplits);
  assertx(!pmesh._vsplits.num());
  // Parse vsplit records and encode.
  {
    RFile& fi_prog = *assertx(pfi_prog);
    std::istream& is = fi_prog();
    for (;;) {
      vspl.resid_uni = 0.f;
      vspl.resid_dir = 0.f;
      bool after_vsplit = false;
      for (string sline; my_getline(is, sline);) {
        if (!parse_line2(const_cast<char*>(sline.c_str()), after_vsplit)) break;
      }
      if (!is) {
        assertx(!after_vsplit);
        break;
      }
      assertx(after_vsplit);
      pmesh._vsplits.push(vspl);
    }
  }
  pmesh._info._has_rgb = has_rgb;
  pmesh._info._has_uv = has_uv;
  pmesh._info._has_resid = has_resid;
  if (append_old_pm != "") {
    showdf("Now appending file %s\n", append_old_pm.c_str());
    {  // reclaim meory
      mvrenumber.clear();
      mwrenumber.clear();
      mfrenumber.clear();
      gcwinfo.clear();
    }
    {  // reclaim memory
      HH_TIMER("_mesh_clear");
      mesh.clear();
    }
    RFile fi(append_old_pm);
    PMeshRStream pmrs(fi(), nullptr);
    int num_old_bmesh_faces;
    {
      HH_TIMER("_read_old_bmesh");
      AWMesh old_bmesh;
      pmrs.read_base_mesh(&old_bmesh);
      // Assert old base mesh is same size as new final mesh.
      assertx(old_bmesh._vertices.num() == pmesh._info._full_nvertices);
      assertx(old_bmesh._wedges.num() == pmesh._info._full_nwedges);
      assertx(old_bmesh._faces.num() == pmesh._info._full_nfaces);
      // Assert all materials are identical.
      const Materials& materials = bmesh._materials;
      assertx(old_bmesh._materials.num() == materials.num());
      for_int(i, materials.num()) assertx(old_bmesh._materials.get(i) == materials.get(i));
      assertx(pmrs._info._has_rgb == has_rgb);
      assertx(pmrs._info._has_uv == has_uv);
      assertx(pmrs._info._has_resid == has_resid);
      // Sanity check
      assertx(append_f_renumber.num() == old_bmesh._faces.num());
      assertx(append_f_vsi_offset.num() == old_bmesh._faces.num());
      num_old_bmesh_faces = old_bmesh._faces.num();
    }
    // Append the vsplits from old pm onto current pm,
    //  while renumbering the flclw and vsindex fields if necessary.
    for (;;) {
      const Vsplit* pvspl = pmrs.next_vsplit();
      if (!pvspl) break;
      pmesh._vsplits.push(*pvspl);
      Vsplit& vspl1 = pmesh._vsplits.last();
      if (vspl1.flclw >= num_old_bmesh_faces) continue;
      int old_flclw = vspl1.flclw;
      if (0) {
        // mesh is now cleared
        assertx(append_f_renumber[old_flclw] == mfrenumber.get(mesh.id_face(old_flclw + 1)));  // ?
      }
      vspl1.flclw = append_f_renumber[old_flclw];
      int old_vsindex = (vspl1.code & Vsplit::VSINDEX_MASK) >> Vsplit::VSINDEX_SHIFT;
      assertx(old_vsindex >= 0 && old_vsindex <= 2);
      int new_vsindex = mod3(old_vsindex + append_f_vsi_offset[old_flclw]);
      vspl1.code = narrow_cast<ushort>((vspl1.code & ~Vsplit::VSINDEX_MASK) | (new_vsindex << Vsplit::VSINDEX_SHIFT));
      // note 'continue' above.
    }
    pmesh._info._tot_nvsplits += pmrs._info._tot_nvsplits;
    pmesh._info._full_nvertices = pmrs._info._full_nvertices;
    pmesh._info._full_nwedges = pmrs._info._full_nwedges;
    pmesh._info._full_nfaces = pmrs._info._full_nfaces;
    pmesh._info._full_bbox = pmrs._info._full_bbox;
  }
  pmesh.write(std::cout);
  std::cout.flush();
  if (1) exit_immediately(0);  // no destruction of GMesh and PMesh
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSD(fbasemesh, "file.m : base mesh");
  HH_ARGSD(fprogressive, "file.rprog : progressive vsplit stream");
  HH_ARGSC("", ":Next mostly made obsolete by FilterPM");
  HH_ARGSP(maxnfaces, "nfaces :  for arithseq & geomseq");
  HH_ARGSD(arithseq, "delta : geomorph every delta nfaces");
  HH_ARGSD(geomseq, "factor : geomorph every factor nfaces");
  HH_ARGSD(to, "nfaces : geomorph up to nfaces");
  HH_ARGSD(from, "nfaces : skip until at least nfaces");
  HH_ARGSD(mesh, ": output current static mesh");
  HH_ARGSD(at, "nfaces : -from nfaces -mesh");
  HH_ARGSD(view, "s3d_filename : selective refinement");
  HH_ARGSD(consider, "nrecords : skip until at least nrecords");
  HH_ARGSD(animateto, "nfaces : animate vsplits up to nfaces");
  HH_ARGSP(stringent, "bool : use stringent test for sel. refin.");
  HH_ARGSF(use_silhouette, ": sel. refin. based on silhouette");
  HH_ARGSF(use_area, ": sel. refin. based on face area");
  HH_ARGSP(splitcorners, "b : split corners into vertices");
  HH_ARGSC("", ":Construction of PM");
  HH_ARGSP(append_old_pm, "file.pm :  append old vsplit sequence");
  HH_ARGSD(pm_encode, ": output PM format");
  // HH_TIMER("Filterprog");
  g_header = args.header();
  args.parse();
  pfi_prog = nullptr;
  return 0;
}
