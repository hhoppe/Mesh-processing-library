// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "progs/MeshToPm/MeshInput.h"

#include "libHh/FileIO.h"
#include "libHh/GMesh.h"  // GMesh::string_key(), parse_key_vec() (but no GMesh object).
#include "libHh/Geometry.h"
#include "libHh/Map.h"
#include "libHh/Parallel.h"
#include "libHh/Ply.h"
#include "libHh/StringOp.h"

namespace hh {

namespace {

constexpr float k_undefined = BIGFLOAT;  // Undefined scalar attributes.
constexpr float k_tol = 1e-6f;           // Scalar attribute equality tolerance (as in MeshSimplify).

// The attributes given in the string of a vertex or a corner.
struct AttribInfo {
  Vector normal{k_undefined, k_undefined, k_undefined};
  A3dColor rgb{k_undefined, k_undefined, k_undefined};
  Uv uv{k_undefined, k_undefined};
  int wid{0};  // Zero if undefined.
};

struct CornerRecord {
  int v;
  int f;
  AttribInfo info;
};

// The information read from a file, beyond the vertex positions and face vertices stored in the mesh.
struct RawMesh {
  bool any_strings{false};
  Array<int> vertex_info;  // For each vertex, an index into vertex_infos, or -1.
  Array<AttribInfo> vertex_infos;
  Array<int> global_vertices;
  Array<int> face_string;      // For each face, an index into face_strings.
  Array<string> face_strings;  // The distinct face strings, in order of first occurrence.
  Map<string, int> index_of_face_string;
  Array<CornerRecord> corners;
  Array<Vec2<int>> sharp_edges;  // Pairs of vertices.
};

void skip_spaces(const char*& s) {
  while (*s == ' ' || *s == '\t') s++;
}

// Returns the string within the braces following s, or nullptr if s has only trailing whitespace.
const char* get_string_info(const char* s, string& str) {
  skip_spaces(s);
  if (!*s) return nullptr;
  if (*s != '{') assertnever("Unexpected character (not '{') at start of '" + string(s) + "'");
  const char* s2 = strrchr(s, '}');
  if (!s2) assertnever("No matching '}' in '" + string(s) + "'");
  str.assign(s + 1, s2);
  return str.c_str();
}

AttribInfo parse_attrib_info(const char* sinfo) {
  AttribInfo info;
  parse_key_vec(sinfo, "normal", info.normal);
  parse_key_vec(sinfo, "rgb", info.rgb);
  parse_key_vec(sinfo, "uv", info.uv);
  string str;
  if (const char* s = GMesh::string_key(str, sinfo, "wid")) info.wid = assertx(to_int(s));
  return info;
}

// Returns the index of the element with the given id, or -1 if there is none.
int lookup_id(CArrayView<int> index_of_id, int id) { return id > 0 && id < index_of_id.num() ? index_of_id[id] : -1; }

int index_of_id(Array<int>& index_of_id, int id, const char* kind) {
  if (id <= 0 || id >= (1 << 30)) assertnever(sform("%s id %d is out of range", kind, id));
  if (id >= index_of_id.num()) {
    const int old_num = index_of_id.num();
    index_of_id.resize(max(id + 1, old_num * 2));
    fill(index_of_id.slice(old_num, index_of_id.num()), -1);
  }
  return index_of_id[id];
}

void read_mesh_m(std::istream& is, WorkMesh& mesh, RawMesh& raw) {
  Array<int> vertex_of_id;                      // Index of the vertex with a given original id, or -1.
  Array<int> face_of_id;                        // Index of the face with a given original id, or -1.
  Array<Vec2<int>> corner_ids, sharp_edge_ids;  // Parallel to raw.corners and raw.sharp_edges until resolved.
  string str;
  for (string line; my_getline(is, line);) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const char* sline = line.c_str();
    if (sline[0] == '#' || sline[0] == 0) continue;
    if (const char* s = after_prefix(sline, "Vertex ")) {
      const int id = int_from_chars(s);
      if (index_of_id(vertex_of_id, id, "Vertex") >= 0) assertnever(sform("Vertex %d is defined twice", id));
      Point p{};
      for_int(c, 3) p[c] = float_from_chars(s);
      const int v = mesh._vertices.num();
      vertex_of_id[id] = v;
      mesh._vertices.push(PmVertex{{p}});
      mesh._vertex_original_id.push(id);
      int info_index = -1;
      if (const char* sinfo = get_string_info(s, str)) {
        const AttribInfo info = parse_attrib_info(sinfo);
        const bool global = GMesh::string_has_key(sinfo, "global");
        // Other keys (e.g., "sph" or "filledcenter") are ignored.
        if (info.normal[0] != k_undefined || info.rgb[0] != k_undefined || info.uv[0] != k_undefined || info.wid ||
            global) {
          raw.any_strings = true;
          info_index = raw.vertex_infos.num();
          raw.vertex_infos.push(info);
          if (global) raw.global_vertices.push(v);
        }
      }
      raw.vertex_info.push(info_index);
      continue;
    }
    if (const char* s = after_prefix(sline, "Face ")) {
      const int id = int_from_chars(s);
      if (index_of_id(face_of_id, id, "Face") >= 0) assertnever(sform("Face %d is defined twice", id));
      Vec3<int> vertices{};
      int nv = 0;
      for (;;) {
        skip_spaces(s);
        if (!*s || *s == '{') break;
        const int vid = int_from_chars(s);
        if (nv == 3) assertnever(sform("Face %d is not a triangle", id));
        const int v = lookup_id(vertex_of_id, vid);
        if (v < 0) assertnever(sform("Face %d refers to undefined vertex %d", id, vid));
        vertices[nv++] = v;
      }
      if (nv != 3) assertnever(sform("Face %d is not a triangle", id));
      if (vertices[0] == vertices[1] || vertices[1] == vertices[2] || vertices[2] == vertices[0])
        assertnever(sform("Face %d has a repeated vertex", id));
      const char* sinfo = get_string_info(s, str);
      if (sinfo) raw.any_strings = true;
      bool is_new;
      int& string_index = raw.index_of_face_string.enter(sinfo ? sinfo : "", raw.face_strings.num(), is_new);
      if (is_new) raw.face_strings.push(sinfo ? sinfo : "");
      raw.face_string.push(string_index);
      face_of_id[id] = mesh._faces.num();
      PmFace face;
      face.wedges = vertices;  // Temporarily, one wedge per vertex.
      face.attrib.matid = 0;
      mesh._faces.push(face);
      mesh._face_original_id.push(id);
      continue;
    }
    if (const char* s = after_prefix(sline, "Corner ")) {
      const int vertex_id = int_from_chars(s), face_id = int_from_chars(s);
      const char* sinfo = get_string_info(s, str);
      if (!sinfo) assertnever("Corner line lacks a string: '" + line + "'");
      raw.any_strings = true;
      raw.corners.push(CornerRecord{-1, -1, parse_attrib_info(sinfo)});
      corner_ids.push(V(vertex_id, face_id));
      continue;
    }
    if (const char* s = after_prefix(sline, "Edge ")) {
      const int vertex_id1 = int_from_chars(s), vertex_id2 = int_from_chars(s);
      if (const char* sinfo = get_string_info(s, str); sinfo && GMesh::string_has_key(sinfo, "sharp")) {
        raw.any_strings = true;
        sharp_edge_ids.push(V(vertex_id1, vertex_id2));
      }
      continue;
    }
    assertnever("Cannot parse line '" + line + "'");
  }
  // Resolve the ids in the Corner and Edge lines, which may precede the Vertex and Face lines they refer to.
  int num_corners = 0;
  for_int(i, raw.corners.num()) {
    const int v = lookup_id(vertex_of_id, corner_ids[i][0]), f = lookup_id(face_of_id, corner_ids[i][1]);
    if (v < 0) {
      Warning("Corner vertex does not exist");
    } else if (f < 0) {
      Warning("Corner face does not exist");
    } else {
      raw.corners[num_corners++] = CornerRecord{v, f, raw.corners[i].info};
    }
  }
  raw.corners.resize(num_corners);
  for (const Vec2<int>& ids : sharp_edge_ids) {
    const int a = lookup_id(vertex_of_id, ids[0]), b = lookup_id(vertex_of_id, ids[1]);
    if (a < 0 || b < 0)
      Warning("Did not find sharp edge in mesh");
    else
      raw.sharp_edges.push(V(a, b));
  }
}

// Fill the empty mesh from a PLY mesh, whose faces must be triangles.  As in GMesh::from_ply(), the mesh vertices
// are given by the trailing element "wedge_vertex" if present, else they are the PLY vertices; the attributes of the
// PLY vertices (wedges) are recorded in raw as those of the vertex strings (for a vertex with a single wedge) or else
// of the corner strings of a .m file; the face strings are those of the materials; and the "wedge_global" flags mark
// global vertices.  The wedges are not imposed (as wid keys would be): as for a .m file without wid keys, the corners
// are grouped by their attributes (normal, rgb, and uv), whereas the PLY wedges may also separate other attributes
// (e.g., Ouv).  Returns the vertex normals if present and the mesh needs no strings.
Array<Vector> mesh_from_ply(PlyMesh& ply, WorkMesh& mesh, RawMesh& raw) {
  const int nw = ply.points.num(), nf = ply.num_faces();
  const bool has_wid = ply.wedge_vertex.num() > 0;
  const int nv = ply.num_mesh_vertices();
  const auto vertex_of_wedge = [&](int w) { return has_wid ? ply.wedge_vertex[w] : w; };
  Array<int> num_wedges(nv, 0), first_wedge(nv, -1);
  for_int(w, nw) {
    const int v = vertex_of_wedge(w);
    if (num_wedges[v]++ == 0) {
      first_wedge[v] = w;
    } else if (ply.points[w] != ply.points[first_wedge[v]]) {
      assertnever(sform("ply: the wedges %d and %d of vertex %d have different positions", first_wedge[v], w, v));
    }
  }
  mesh._vertices.init(nv);
  mesh._vertex_original_id.init(nv);
  for_int(v, nv) {
    if (!num_wedges[v]) assertnever(sform("ply: vertex %d has no wedges", v));
    mesh._vertices[v].attrib.point = ply.points[first_wedge[v]];
    mesh._vertex_original_id[v] = ply.vertex_ids.num() ? ply.vertex_ids[v] : v + 1;
  }
  mesh._faces.init(nf);
  mesh._face_original_id.init(nf);
  for_int(f, nf) {
    const CArrayView<int> wedges = ply.face(f);
    if (wedges.num() != 3) assertnever(sform("ply: face %d is not a triangle", f));
    const Vec3<int> vertices = transformed(Vec3<int>(wedges[0], wedges[1], wedges[2]), vertex_of_wedge);
    if (vertices[0] == vertices[1] || vertices[1] == vertices[2] || vertices[2] == vertices[0])
      assertnever(sform("ply: face %d has a repeated vertex", f));
    mesh._faces[f].wedges = vertices;  // Temporarily, one wedge per vertex.
    mesh._faces[f].attrib.matid = 0;
    mesh._face_original_id[f] = ply.face_ids.num() ? ply.face_ids[f] : f + 1;
  }
  const PlyMesh::WedgeKey* global_key = nullptr;
  for (const PlyMesh::WedgeKey& wedge_key : ply.wedge_keys)
    if (wedge_key.name == "global" && wedge_key.dim == 0) global_key = &wedge_key;
  const bool needs_strings =
      has_wid || ply.colors.num() || ply.uvs.num() || ply.corner_uvs.num() || ply.materials.num() || global_key;
  if (!needs_strings) return std::move(ply.normals);
  raw.any_strings = true;
  // The face strings.
  raw.face_string.init(nf);
  for_int(f, nf) {
    const int matid = ply.face_matid(f);
    const string& face_string = matid < ply.materials.num() ? ply.materials[matid] : "";
    bool is_new;
    const int index = raw.index_of_face_string.enter(face_string, raw.face_strings.num(), is_new);
    if (is_new) raw.face_strings.push(face_string);
    raw.face_string[f] = index;
  }
  // The attributes of the wedges.
  const auto wedge_info = [&](int w) {
    AttribInfo info;
    if (ply.normals.num()) info.normal = ply.normals[w];
    if (ply.colors.num()) for_int(c, 3) info.rgb[c] = ply.colors[w][c] / 255.f;
    if (ply.uvs.num()) info.uv = ply.uvs[w];
    return info;
  };
  raw.vertex_info.init(nv, -1);
  for_int(v, nv) {
    if (num_wedges[v] != 1) continue;
    raw.vertex_info[v] = raw.vertex_infos.num();
    raw.vertex_infos.push(wedge_info(first_wedge[v]));
  }
  for_int(f, nf) for_int(j, 3) {
    const int w = ply.face(f)[j], v = mesh._faces[f].wedges[j];
    if (num_wedges[v] > 1) raw.corners.push(CornerRecord{v, f, wedge_info(w)});
    if (ply.corner_uvs.num()) {
      CornerRecord record{v, f, AttribInfo{}};
      record.info.uv = ply.corner_uvs[3 * f + j];
      raw.corners.push(record);
    }
  }
  if (global_key) {
    Array<bool> is_global(nv, false);
    for_int(w, nw) if (global_key->values[w]) is_global[vertex_of_wedge(w)] = true;
    for_int(v, nv) if (is_global[v]) raw.global_vertices.push(v);
  }
  return {};
}

// Returns the sum over the faces of vertex v of the face normals weighted by the face angles at v, as
// Vnors::EType::angle computes it.
Vector angle_weighted_normal(const WorkMesh& mesh, int v, CArrayView<int> faces) {
  const Point& vp = mesh._vertices[v].attrib.point;
  Vector sum{};
  for (const int f : faces) {
    const int j = mesh.get_jvf(v, f);
    const Point& p1 = mesh._vertices[mesh.vertex(f, mod3(j + 1))].attrib.point;
    const Point& p2 = mesh._vertices[mesh.vertex(f, mod3(j + 2))].attrib.point;
    const Vector face_normal = ok_normalized(cross(vp, p1, p2));
    // A degenerate face (e.g., with coincident vertices) has a zero normal and contributes nothing; skipping it also
    // avoids passing a zero vector to angle_between_unit_vectors().
    if (is_zero(face_normal)) continue;
    const float angle = angle_between_unit_vectors(ok_normalized(p1 - vp), ok_normalized(p2 - vp));
    sum += face_normal * angle;
  }
  return sum;
}

// For a mesh without any strings: one wedge per vertex, with the given normal or else a computed one.
void create_simple_wedges(WorkMesh& mesh, CArrayView<Vector> given_normals) {
  mesh._materials.set(0, "matid=0");
  std::atomic<int> num_zero_normals = 0;
  parallel_for(range(mesh._vertices.num()), [&](const int v) {
    PmWedgeAttrib& wa = mesh._wedges[v].attrib;
    wa = PmWedgeAttrib{};
    if (!mesh.vertex_alive(v)) return;
    if (given_normals.num()) {
      wa.normal = given_normals[v];
    } else {
      InlinedArray<int, 32> faces;  // Without heap allocation for typical vertex degrees.
      for (const int f : mesh.ccw_faces(v, mesh._someface[v])) faces.push(f);
      wa.normal = angle_weighted_normal(mesh, v, faces);
      wa.normal.normalize();  // As in Vnors, before the normalization below (as in MeshSimplify's construct_wi()).
    }
    if (!wa.normal.normalize()) {
      wa.normal = Vector(1.f, 0.f, 0.f);
      num_zero_normals++;
    }
  });
  if (num_zero_normals) {
    Warning("Normal is zero, setting arbitrarily to (1.f, 0.f, 0.f)");
    showdf("(%d zero normals)\n", int(num_zero_normals));
  }
}

// Assign material identifiers from the face strings, as MeshSimplify's parse_mesh_material_identifiers() does.
void assign_materials(WorkMesh& mesh, const RawMesh& raw) {
  Array<string> material_strings;
  Array<int> matid_of_string(raw.face_strings.num(), -1);
  string str;
  for_int(i, raw.face_strings.num()) {
    if (const char* smat = GMesh::string_key(str, raw.face_strings[i].c_str(), "matid")) {
      const int matid = to_int(smat);
      assertx(matid >= 0);
      material_strings.access(matid);
      assertx(material_strings[matid] == "");  // No duplicate matid.
      material_strings[matid] = raw.face_strings[i];
      matid_of_string[i] = matid;
    }
  }
  for_int(i, raw.face_strings.num()) {
    if (matid_of_string[i] >= 0) continue;
    const int matid = material_strings.add(1);
    material_strings[matid] = GMesh::string_update(raw.face_strings[i], "matid", csform(str, "%d", matid));
    matid_of_string[i] = matid;
  }
  if (material_strings.num() > 65536) assertnever("The progressive mesh format allows at most 65536 materials");
  for_int(matid, material_strings.num()) mesh._materials.set(matid, material_strings[matid]);
  for_int(f, mesh._faces.num()) mesh._faces[f].attrib.matid = matid_of_string[raw.face_string[f]];
}

// For a mesh with strings: normals, colors, and uv per corner, then wedges, as MeshSimplify's
// parse_mesh_wedge_identifiers() and Vnors do.  The corners of each vertex are processed independently (in
// parallel), and the wedges are numbered in the order of their first corner in the faces.
void create_general_wedges(WorkMesh& mesh, RawMesh& raw) {
  const int nf = mesh._faces.num(), nv = mesh._vertices.num();
  assign_materials(mesh, raw);
  for (const int v : raw.global_vertices) mesh._vflags[v] |= WorkMesh::k_vflag_global;
  // Corner c = 3 * f + j is the corner j of face f.
  Array<int> corner_record(3 * nf, -1);
  for_int(i, raw.corners.num()) {
    const int v = raw.corners[i].v, f = raw.corners[i].f;
    int j = 0;
    while (j < 3 && mesh.vertex(f, j) != v) j++;
    if (j == 3)
      assertnever(sform("Corner %d %d: the face does not contain the vertex", mesh._vertex_original_id[v],
                        mesh._face_original_id[f]));
    corner_record[3 * f + j] = i;
  }
  // The edge opposite corner c is sharp.
  Array<bool> edge_sharp(3 * nf, false);
  for (const auto [a, b] : raw.sharp_edges) {
    bool found = false;
    if (mesh._someface[a] >= 0) {
      for (const int f : mesh.ccw_faces(a, mesh._someface[a])) {
        const int ja = mesh.get_jvf(a, f);
        int jb = -1;
        for_int(j, 3) if (mesh.vertex(f, j) == b) jb = j;
        if (jb < 0) continue;
        const int k = 3 - ja - jb;  // The corner opposite edge (a, b).
        edge_sharp[3 * f + k] = true;
        found = true;
      }
    }
    if (!found) Warning("Did not find sharp edge in mesh");
  }
  const auto vinfo = [&](int v) -> const AttribInfo* {
    return raw.vertex_info[v] >= 0 ? &raw.vertex_infos[raw.vertex_info[v]] : nullptr;
  };
  const auto cinfo = [&](int c) -> const AttribInfo* {
    return corner_record[c] >= 0 ? &raw.corners[corner_record[c]].info : nullptr;
  };
  // Remove explicitly zero normals.
  for (AttribInfo& info : raw.vertex_infos)
    if (info.normal[0] != k_undefined && is_zero(info.normal)) {
      Warning("Removing explicit zero normal from vertex");
      info.normal = Vector(k_undefined, k_undefined, k_undefined);
    }
  for (CornerRecord& record : raw.corners)
    if (record.info.normal[0] != k_undefined && is_zero(record.info.normal)) {
      Warning("Removing explicit zero normal from corner");
      record.info.normal = Vector(k_undefined, k_undefined, k_undefined);
    }
  // If any corner has a wedge id (wid), the wedges are defined by the wids (and all corners must have one).
  bool any_wid = false;
  for_int(v, nv) if (mesh.vertex_alive(v) && vinfo(v) && vinfo(v)->wid) any_wid = true;
  for (const int i : corner_record)
    if (i >= 0 && raw.corners[i].info.wid) any_wid = true;
  const auto same_attrib = [](const PmWedgeAttrib& a1, const PmWedgeAttrib& a2) {
    return compare(a1.rgb, a2.rgb, k_tol) == 0 && compare(a1.normal, a2.normal, k_tol) == 0 &&
           compare(a1.uv, a2.uv, k_tol) == 0;
  };
  // Process the vertices in chunks, each with its own array of wedges.  For each corner, corner_wedge holds the
  // index of its wedge in the array of the chunk of its vertex.
  constexpr int k_chunk_size = 1024;
  const int num_chunks = (nv + k_chunk_size - 1) / k_chunk_size;
  Array<Array<PmWedge>> chunk_wedges(num_chunks);
  Array<int> corner_wedge(3 * nf, -1);
  std::atomic<int> num_rgb = 0, num_uv = 0, num_wid = 0;
  const auto process_vertex = [&](int v, Array<PmWedge>& wedges, Vec3<int>& counts) {
    InlinedArray<int, 32> faces;  // Without heap allocation for typical vertex degrees.
    for (const int f : mesh.ccw_faces(v, mesh._someface[v])) faces.push(f);
    const int n = faces.num();
    const auto corner = [&](int i) { return 3 * faces[i] + mesh.get_jvf(v, faces[i]); };
    const int f_last = faces.last();
    const bool closed = mesh._fnei[f_last].faces[mod3(mesh.get_jvf(v, f_last) + 1)] == faces[0];
    const AttribInfo* vi = vinfo(v);
    const bool has_vertex_normal = vi && vi->normal[0] != k_undefined;
    int num_corner_normals = 0;
    for_int(i, n) {
      const AttribInfo* ci = cinfo(corner(i));
      num_corner_normals += ci && ci->normal[0] != k_undefined;
    }
    InlinedArray<Vector, 32> normals(n);
    Vector unique_normal{};
    if (has_vertex_normal && !num_corner_normals) {
      unique_normal = vi->normal;
      fill(normals, unique_normal);
    } else {
      if (has_vertex_normal) Warning("Have both vertex and corner normals");
      // An edge between consecutive faces i and i + 1 (in ccw order) is sharp if it is flagged as such, or if the
      // corner strings on its two sides have different wid keys (as in MeshOp.cpp's sharp()).
      const auto sharp_after = [&](int i) {
        if (!closed && i == n - 1) return true;
        const int c1 = corner(i), c2 = corner((i + 1) % n);
        if (edge_sharp[3 * (c1 / 3) + mod3(c1 % 3 + 1)]) return true;
        const AttribInfo* info2 = cinfo(c2);
        if (info2 && info2->wid) {
          const AttribInfo* info1 = cinfo(c1);
          if (info1 && info1->wid && info1->wid != info2->wid) return true;
        }
        return false;
      };
      int first = 0;  // The first face of a fan.
      if (closed) {
        int i_sharp = -1;
        for_int(i, n) if (sharp_after(i)) i_sharp = i;
        first = i_sharp < 0 ? 0 : (i_sharp + 1) % n;
      }
      for (int k = 0; k < n;) {
        InlinedArray<int, 32> fan_faces;
        int i = (first + k) % n;
        for (;;) {
          fan_faces.push(faces[i]);
          k++;
          if (k == n || sharp_after(i)) break;
          i = (i + 1) % n;
        }
        Vector vec = angle_weighted_normal(mesh, v, fan_faces);
        assertw(vec.normalize());
        for_int(kk, fan_faces.num()) normals[(i - kk + n) % n] = vec;
        if (fan_faces.num() == n && !has_vertex_normal) unique_normal = vec;
      }
      if (has_vertex_normal) unique_normal = vi->normal;
      if (num_corner_normals) {
        for_int(i, n) {
          const AttribInfo* ci = cinfo(corner(i));
          if (ci && ci->normal[0] != k_undefined) {
            normals[i] = ci->normal;
          } else {
            Warning(has_vertex_normal ? "Missing corner normal, using vertex normal"
                                      : "Missing corner normal, using estimated normal");
            normals[i] = unique_normal;
          }
        }
      }
    }
    InlinedArray<PmWedgeAttrib, 32> attribs(n);
    InlinedArray<int, 32> wids(n);
    for_int(i, n) {
      PmWedgeAttrib& wa = attribs[i];
      wa.normal = normals[i];
      if (!wa.normal.normalize()) {
        Warning("Normal is zero, setting arbitrarily to (1.f, 0.f, 0.f)");
        wa.normal = Vector(1.f, 0.f, 0.f);
      }
      const AttribInfo* ci = cinfo(corner(i));
      const auto choose = [&](auto member) -> const AttribInfo* {
        const bool in_corner = ci && (ci->*member)[0] != k_undefined;
        const bool in_vertex = vi && (vi->*member)[0] != k_undefined;
        if (in_corner && in_vertex) Warning("Have both vertex and corner info");
        return in_corner ? ci : in_vertex ? vi : nullptr;
      };
      wa.rgb = A3dColor(k_undefined, k_undefined, k_undefined);
      if (const AttribInfo* info = choose(&AttribInfo::rgb)) wa.rgb = info->rgb, counts[0]++;
      wa.uv = Uv(k_undefined, k_undefined);
      if (const AttribInfo* info = choose(&AttribInfo::uv)) wa.uv = info->uv, counts[1]++;
      if (ci && ci->wid && vi && vi->wid) Warning("Have both vertex and corner info");
      wids[i] = ci && ci->wid ? ci->wid : vi ? vi->wid : 0;
      counts[2] += wids[i] != 0;
    }
    // Group the contiguous corners with the same attributes (or wid) into wedges.  The face ccw (respectively
    // clw) of faces[i] is faces[i + 1] (respectively faces[i - 1]), cyclically if the vertex is closed.
    InlinedArray<int, 32> wedge_of(n, -1);  // Index into wedges.
    for_int(irep, n) {
      if (wedge_of[irep] >= 0) continue;
      const int w = wedges.add(1);
      wedges[w] = PmWedge{v, attribs[irep]};
      wedge_of[irep] = w;
      const int matid = mesh._faces[faces[irep]].attrib.matid;
      bool wrapped = false;
      for_int(dir, 2) {  // Two directions (ccw, clw).
        for (int i = irep;;) {
          if (dir == 0) {
            if (++i == n) {
              if (!closed) break;
              i = 0;
            }
          } else {
            if (i-- == 0) {
              if (!closed) break;
              i = n - 1;
            }
          }
          if (i == irep) {
            wrapped = true;
            break;
          }
          const bool differ =
              any_wid ? wids[i] != wids[irep]
                      : mesh._faces[faces[i]].attrib.matid != matid || !same_attrib(attribs[irep], attribs[i]);
          if (differ || wedge_of[i] >= 0) break;
          wedge_of[i] = w;
        }
        if (wrapped) break;
      }
    }
    if (any_wid) {  // Corners with the same wid are in the same wedge even if not contiguous.
      InlinedArray<std::pair<int, int>, 8> wedge_of_wid;
      for_int(i, n) {
        auto* const it = ranges::find(wedge_of_wid, wids[i], &std::pair<int, int>::first);
        if (it == wedge_of_wid.end())
          wedge_of_wid.push({wids[i], wedge_of[i]});
        else
          wedge_of[i] = it->second;
      }
    }
    for_int(i, n) corner_wedge[corner(i)] = wedge_of[i];
  };
  parallel_for(range(num_chunks), [&](const int chunk) {
    Vec3<int> counts{0, 0, 0};  // Numbers of corners with rgb, uv, and wid.
    for (int v = chunk * k_chunk_size; v < min((chunk + 1) * k_chunk_size, nv); v++)
      if (mesh.vertex_alive(v)) process_vertex(v, chunk_wedges[chunk], counts);
    num_rgb += counts[0];
    num_uv += counts[1];
    num_wid += counts[2];
  });
  const int num_corners = 3 * mesh.num_live_faces();
  mesh._has_rgb = num_rgb > 0;
  mesh._has_uv = num_uv > 0;
  if (num_rgb && num_rgb != num_corners) assertnever("Colors are defined on only some corners");
  if (num_uv && num_uv != num_corners) assertnever("Uv coordinates are defined on only some corners");
  if (num_wid && num_wid != num_corners) assertnever("Wedge ids are defined on only some corners");
  // Number the wedges in the order of their first corner in the faces (which also drops the wedges that lost all
  // their corners in a merge by wid).
  Array<int> chunk_offset(num_chunks + 1, 0);
  for_int(chunk, num_chunks) chunk_offset[chunk + 1] = chunk_offset[chunk] + chunk_wedges[chunk].num();
  Array<int> new_index(chunk_offset.last(), -1);
  Array<PmWedge> used_wedges;
  for_int(f, nf) for_int(j, 3) {
    const int v = mesh.vertex(f, j);  // The face still refers to the temporary wedge v of vertex v.
    const int chunk = v / k_chunk_size, local_index = corner_wedge[3 * f + j];
    int& w = new_index[chunk_offset[chunk] + local_index];
    if (w < 0) {
      w = used_wedges.num();
      PmWedge wedge = chunk_wedges[chunk][local_index];
      if (!mesh._has_rgb) wedge.attrib.rgb = A3dColor(0.f, 0.f, 0.f);
      if (!mesh._has_uv) wedge.attrib.uv = Uv(0.f, 0.f);
      used_wedges.push(wedge);
    }
    mesh._faces[f].wedges[j] = w;
  }
  mesh._wedges = std::move(used_wedges);
}

}  // namespace

MeshFileHeader read_mesh_header(std::istream& is) {
  MeshFileHeader header;
  if (is.peek() == 'p') {  // A .m file cannot start with 'p'.
    header.is_ply = true;
    header.ply_lines = read_ply_header(is);
    // Echo the header comments (the provenance), as for the '#' lines of a mesh file.
    for (const string& comment : ply_header_comments(header.ply_lines)) showff("|%s\n", comment.c_str());
  } else {
    string line;
    while (is.peek() == '#') {
      assertx(my_getline(is, line));
      if (line.size() > 1) showff("|%s\n", line.substr(line[1] == ' ' ? 2 : 1).c_str());
    }
  }
  return header;
}

void read_mesh(std::istream& is, const MeshFileHeader& header, WorkMesh& mesh) {
  assertx(mesh._vertices.empty());
  RawMesh raw;
  Array<Vector> ply_normals;
  if (header.is_ply) {
    PlyMesh ply = read_ply_body(is, header.ply_lines);
    ply_normals = mesh_from_ply(ply, mesh, raw);
  } else {
    read_mesh_m(is, mesh, raw);
  }
  if (mesh._faces.empty()) assertnever("The mesh has no faces");
  // Temporarily, each vertex has one wedge, with the same index.
  mesh._wedges.init(mesh._vertices.num());
  for_int(v, mesh._vertices.num()) mesh._wedges[v].vertex = v;
  mesh._vflags.init(mesh._vertices.num(), uchar{0});
  build_adjacency(mesh);
  if (raw.any_strings)
    create_general_wedges(mesh, raw);
  else
    create_simple_wedges(mesh, ply_normals);
}

}  // namespace hh
