// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "progs/MeshToPm/MeshInput.h"

#include <bit>  // bit_cast(), byteswap(), endian
#include <sstream>

#include "libHh/FileIO.h"
#include "libHh/GMesh.h"  // GMesh::string_key(), parse_key_vec() (but no GMesh object).
#include "libHh/Geometry.h"
#include "libHh/Map.h"
#include "libHh/Parallel.h"
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
  int vertex_id;
  int face_id;
  AttribInfo info;
};

// The information read from a file, beyond the vertex positions and face vertices stored in the mesh.
struct RawMesh {
  Array<int> vertex_of_id;  // Index of the vertex with a given original id, or -1.
  Array<int> face_of_id;    // Index of the face with a given original id, or -1.
  bool any_strings{false};
  Array<int> vertex_info;  // For each vertex, an index into vertex_infos, or -1.
  Array<AttribInfo> vertex_infos;
  Array<int> global_vertices;
  Array<int> face_string;      // For each face, an index into face_strings.
  Array<string> face_strings;  // The distinct face strings, in order of first occurrence.
  Map<string, int> index_of_face_string;
  Array<CornerRecord> corners;
  Array<Vec2<int>> sharp_edges;  // Pairs of vertex ids.
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
  bool seen_element = false;
  string str;
  for (string line; my_getline(is, line);) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const char* sline = line.c_str();
    if (sline[0] == '#' || sline[0] == 0) {
      if (!seen_element && line.size() > 1) showff("|%s\n", line.substr(2).c_str());
      continue;
    }
    seen_element = true;
    if (const char* s = after_prefix(sline, "Vertex ")) {
      const int id = int_from_chars(s);
      if (index_of_id(raw.vertex_of_id, id, "Vertex") >= 0) assertnever(sform("Vertex %d is defined twice", id));
      Point p{};
      for_int(c, 3) p[c] = float_from_chars(s);
      const int v = mesh._vertices.num();
      raw.vertex_of_id[id] = v;
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
      if (index_of_id(raw.face_of_id, id, "Face") >= 0) assertnever(sform("Face %d is defined twice", id));
      Vec3<int> vertices{};
      int nv = 0;
      for (;;) {
        skip_spaces(s);
        if (!*s || *s == '{') break;
        const int vid = int_from_chars(s);
        if (nv == 3) assertnever(sform("Face %d is not a triangle", id));
        const int v = vid > 0 && vid < raw.vertex_of_id.num() ? raw.vertex_of_id[vid] : -1;
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
      raw.face_of_id[id] = mesh._faces.num();
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
      raw.corners.push(CornerRecord{vertex_id, face_id, parse_attrib_info(sinfo)});
      continue;
    }
    if (const char* s = after_prefix(sline, "Edge ")) {
      const int vertex_id1 = int_from_chars(s), vertex_id2 = int_from_chars(s);
      if (const char* sinfo = get_string_info(s, str); sinfo && GMesh::string_has_key(sinfo, "sharp")) {
        raw.any_strings = true;
        raw.sharp_edges.push(V(vertex_id1, vertex_id2));
      }
      continue;
    }
    assertnever("Cannot parse line '" + line + "'");
  }
}

// A scalar property type of the PLY format.
enum class PlyType { int8, uint8, int16, uint16, int32, uint32, float32, float64 };

PlyType ply_type(const string& name) {
  if (name == "char" || name == "int8") return PlyType::int8;
  if (name == "uchar" || name == "uint8") return PlyType::uint8;
  if (name == "short" || name == "int16") return PlyType::int16;
  if (name == "ushort" || name == "uint16") return PlyType::uint16;
  if (name == "int" || name == "int32") return PlyType::int32;
  if (name == "uint" || name == "uint32") return PlyType::uint32;
  if (name == "float" || name == "float32") return PlyType::float32;
  if (name == "double" || name == "float64") return PlyType::float64;
  assertnever("ply: unrecognized property type '" + name + "'");
}

struct PlyProperty {
  string name;
  PlyType type;
  bool is_list{false};
  PlyType count_type{PlyType::uint8};  // Defined if is_list.
};

struct PlyElement {
  string name;
  int count;
  Array<PlyProperty> properties;
};

// Reads PLY values, in ASCII or in binary with either byte order.
class PlyValueReader {
 public:
  PlyValueReader(std::istream& is, bool binary, bool big_endian)
      : _is(is), _binary(binary), _swap(big_endian != (std::endian::native == std::endian::big)) {}
  double read(PlyType type) {
    if (!_binary) {
      double value;
      if (!(_is >> value)) assertnever("ply: error reading ASCII data");
      return value;
    }
    switch (type) {
      case PlyType::int8: return read_binary<int8_t>();
      case PlyType::uint8: return read_binary<uint8_t>();
      case PlyType::int16: return read_binary<int16_t>();
      case PlyType::uint16: return read_binary<uint16_t>();
      case PlyType::int32: return read_binary<int32_t>();
      case PlyType::uint32: return read_binary<uint32_t>();
      case PlyType::float32: return std::bit_cast<float>(read_binary<uint32_t>());
      case PlyType::float64: return std::bit_cast<double>(read_binary<uint64_t>());
      default: assertnever("");
    }
  }

 private:
  std::istream& _is;
  bool _binary;
  bool _swap;
  template <typename T> T read_binary() {
    T value;
    if (!_is.read(reinterpret_cast<char*>(&value), sizeof(T))) assertnever("ply: premature end of binary data");
    if constexpr (sizeof(T) > 1) {
      if (_swap) value = std::byteswap(value);
    }
    return value;
  }
};

// Read a PLY file with a vertex element (x, y, z, and optionally nx, ny, nz) followed by a face element (a
// vertex_indices list of triangles); other properties are skipped and later elements are ignored.  Returns the
// vertex normals if present.
Array<Vector> read_mesh_ply(std::istream& is, WorkMesh& mesh) {
  string line;
  if (!my_getline(is, line) || (line != "ply" && line != "ply\r")) assertnever("ply: missing 'ply' magic line");
  bool binary = false, big_endian = false;
  Array<PlyElement> elements;
  for (;;) {
    if (!my_getline(is, line)) assertnever("ply: premature end of header");
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::istringstream iss(line);
    string keyword;
    iss >> keyword;
    if (keyword == "format") {
      string format;
      iss >> format;
      if (format == "binary_little_endian") {
        binary = true;
      } else if (format == "binary_big_endian") {
        binary = true;
        big_endian = true;
      } else if (format != "ascii") {
        assertnever("ply: unrecognized format '" + format + "'");
      }
    } else if (keyword == "element") {
      PlyElement element;
      if (!(iss >> element.name >> element.count) || element.count < 0) assertnever("ply: bad line '" + line + "'");
      elements.push(std::move(element));
    } else if (keyword == "property") {
      if (elements.empty()) assertnever("ply: property before element");
      PlyProperty property;
      string type;
      iss >> type;
      if (type == "list") {
        string count_type, item_type;
        iss >> count_type >> item_type;
        property.is_list = true;
        property.count_type = ply_type(count_type);
        property.type = ply_type(item_type);
      } else {
        property.type = ply_type(type);
      }
      if (!(iss >> property.name)) assertnever("ply: bad line '" + line + "'");
      elements.last().properties.push(std::move(property));
    } else if (keyword == "end_header") {
      break;
    } else if (keyword != "comment" && keyword != "obj_info" && keyword != "") {
      assertnever("ply: unrecognized header line '" + line + "'");
    }
  }
  if (elements.num() < 2 || elements[0].name != "vertex" || elements[1].name != "face")
    assertnever("ply: expected a vertex element followed by a face element");
  const PlyElement& evertex = elements[0];
  const PlyElement& eface = elements[1];
  const auto find_property = [](const PlyElement& element, const char* name) {
    for_int(i, element.properties.num()) if (element.properties[i].name == name) return i;
    return -1;
  };
  const Vec3<int> ipos{find_property(evertex, "x"), find_property(evertex, "y"), find_property(evertex, "z")};
  const Vec3<int> inor{find_property(evertex, "nx"), find_property(evertex, "ny"), find_property(evertex, "nz")};
  if (min(ipos) < 0) assertnever("ply: vertex lacks x, y, or z");
  const bool has_normals = min(inor) >= 0;
  for (const PlyProperty& property : evertex.properties)
    if (property.is_list) assertnever("ply: vertex list property '" + property.name + "' is not supported");
  const int iindices = find_property(eface, "vertex_indices");
  if (iindices < 0 || !eface.properties[iindices].is_list) assertnever("ply: face lacks a vertex_indices list");
  PlyValueReader reader(is, binary, big_endian);
  const int nv = evertex.count, nf = eface.count;
  mesh._vertices.init(nv);
  mesh._vertex_original_id.init(nv);
  Array<Vector> normals(has_normals ? nv : 0);
  Array<double> values(evertex.properties.num());
  for_int(v, nv) {
    for_int(i, values.num()) values[i] = reader.read(evertex.properties[i].type);
    for_int(c, 3) mesh._vertices[v].attrib.point[c] = float(values[ipos[c]]);
    if (has_normals) for_int(c, 3) normals[v][c] = float(values[inor[c]]);
    mesh._vertex_original_id[v] = v + 1;
  }
  mesh._faces.init(nf);
  mesh._face_original_id.init(nf);
  for_int(f, nf) {
    for_int(i, eface.properties.num()) {
      const PlyProperty& property = eface.properties[i];
      if (!property.is_list) {
        (void)reader.read(property.type);
        continue;
      }
      const int count = int(reader.read(property.count_type));
      if (i == iindices && count != 3) assertnever(sform("ply: face %d is not a triangle", f));
      for_int(k, count) {
        const double value = reader.read(property.type);
        if (i != iindices) continue;
        const int v = int(value);
        if (v < 0 || v >= nv) assertnever(sform("ply: face %d refers to undefined vertex %d", f, v));
        mesh._faces[f].wedges[k] = v;
      }
    }
    const Vec3<int>& vertices = mesh._faces[f].wedges;
    if (vertices[0] == vertices[1] || vertices[1] == vertices[2] || vertices[2] == vertices[0])
      assertnever(sform("ply: face %d has a repeated vertex", f));
    mesh._faces[f].attrib.matid = 0;
    mesh._face_original_id[f] = f + 1;
  }
  return normals;
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
// parse_mesh_wedge_identifiers() and Vnors do.
void create_general_wedges(WorkMesh& mesh, RawMesh& raw) {
  const int nf = mesh._faces.num(), nv = mesh._vertices.num();
  assign_materials(mesh, raw);
  for (const int v : raw.global_vertices) mesh._vflags[v] |= WorkMesh::k_vflag_global;
  // Corner c = 3 * f + j is the corner j of face f.
  Array<int> corner_record(3 * nf, -1);
  for_int(i, raw.corners.num()) {
    const CornerRecord& record = raw.corners[i];
    const int v = lookup_id(raw.vertex_of_id, record.vertex_id), f = lookup_id(raw.face_of_id, record.face_id);
    if (v < 0) {
      Warning("Corner vertex does not exist");
    } else if (f < 0) {
      Warning("Corner face does not exist");
    } else {
      int j = 0;
      while (j < 3 && mesh.vertex(f, j) != v) j++;
      if (j == 3)
        assertnever(sform("Corner %d %d: the face does not contain the vertex", record.vertex_id, record.face_id));
      corner_record[3 * f + j] = i;
    }
  }
  // The edge opposite corner c is sharp.
  Array<bool> edge_sharp(3 * nf, false);
  for (const Vec2<int>& vertex_ids : raw.sharp_edges) {
    const int a = lookup_id(raw.vertex_of_id, vertex_ids[0]), b = lookup_id(raw.vertex_of_id, vertex_ids[1]);
    bool found = false;
    if (a >= 0 && b >= 0 && mesh._someface[a] >= 0) {
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
  // Per-corner attributes.
  Array<PmWedgeAttrib> corner_attrib(3 * nf);
  Array<int> corner_wid(3 * nf, 0);
  int num_rgb = 0, num_uv = 0, num_wid = 0;
  for_int(v, nv) {
    if (!mesh.vertex_alive(v)) continue;
    const Array<int> faces(mesh.ccw_faces(v, mesh._someface[v]));
    const int n = faces.num();
    const auto corner = [&](int i) { return 3 * faces[i] + mesh.get_jvf(v, faces[i]); };
    const AttribInfo* vi = vinfo(v);
    const bool has_vertex_normal = vi && vi->normal[0] != k_undefined;
    int num_corner_normals = 0;
    for_int(i, n) {
      const AttribInfo* ci = cinfo(corner(i));
      num_corner_normals += ci && ci->normal[0] != k_undefined;
    }
    Array<Vector> normals(n);
    Vector unique_normal{};
    if (has_vertex_normal && !num_corner_normals) {
      unique_normal = vi->normal;
      fill(normals, unique_normal);
    } else {
      if (has_vertex_normal) Warning("Have both vertex and corner normals");
      // An edge between consecutive faces i and i + 1 (in ccw order) is sharp if it is flagged as such, or if the
      // corner strings on its two sides have different wid keys (as in MeshOp.cpp's sharp()).
      const int f_last = faces.last();
      const bool closed = mesh._fnei[f_last].faces[mod3(mesh.get_jvf(v, f_last) + 1)] == faces[0];
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
        Array<int> fan_faces;
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
    for_int(i, n) {
      const int c = corner(i);
      PmWedgeAttrib& wa = corner_attrib[c];
      wa.normal = normals[i];
      if (!wa.normal.normalize()) {
        Warning("Normal is zero, setting arbitrarily to (1.f, 0.f, 0.f)");
        wa.normal = Vector(1.f, 0.f, 0.f);
      }
      const AttribInfo* ci = cinfo(c);
      const auto choose = [&](auto member) -> const AttribInfo* {
        const bool in_corner = ci && (ci->*member)[0] != k_undefined;
        const bool in_vertex = vi && (vi->*member)[0] != k_undefined;
        if (in_corner && in_vertex) Warning("Have both vertex and corner info");
        return in_corner ? ci : in_vertex ? vi : nullptr;
      };
      wa.rgb = A3dColor(k_undefined, k_undefined, k_undefined);
      if (const AttribInfo* info = choose(&AttribInfo::rgb)) wa.rgb = info->rgb, num_rgb++;
      wa.uv = Uv(k_undefined, k_undefined);
      if (const AttribInfo* info = choose(&AttribInfo::uv)) wa.uv = info->uv, num_uv++;
      if (ci && ci->wid && vi && vi->wid) Warning("Have both vertex and corner info");
      corner_wid[c] = ci && ci->wid ? ci->wid : vi ? vi->wid : 0;
      num_wid += corner_wid[c] != 0;
    }
  }
  const int num_corners = 3 * mesh.num_live_faces();
  mesh._has_rgb = num_rgb > 0;
  mesh._has_uv = num_uv > 0;
  if (num_rgb && num_rgb != num_corners) assertnever("Colors are defined on only some corners");
  if (num_uv && num_uv != num_corners) assertnever("Uv coordinates are defined on only some corners");
  if (num_wid && num_wid != num_corners) assertnever("Wedge ids are defined on only some corners");
  // Group the corners of each vertex into wedges.
  const auto same_attrib = [](const PmWedgeAttrib& a1, const PmWedgeAttrib& a2) {
    return compare(a1.rgb, a2.rgb, k_tol) == 0 && compare(a1.normal, a2.normal, k_tol) == 0 &&
           compare(a1.uv, a2.uv, k_tol) == 0;
  };
  Array<PmWedge> wedges;
  Array<int> corner_wedge(3 * nf, -1);
  for_int(v, nv) {
    if (!mesh.vertex_alive(v)) continue;
    const Array<int> faces(mesh.ccw_faces(v, mesh._someface[v]));
    for (const int frep : faces) {
      const int crep = 3 * frep + mesh.get_jvf(v, frep);
      if (corner_wedge[crep] >= 0) continue;
      const int w = wedges.add(1);
      wedges[w] = PmWedge{v, corner_attrib[crep]};
      corner_wedge[crep] = w;
      const int matid = mesh._faces[frep].attrib.matid;
      for_int(dir, 2) {  // Two directions (ccw, clw).
        int f = frep;
        for (;;) {
          f = mesh._fnei[f].faces[mod3(mesh.get_jvf(v, f) + (dir ? 2 : 1))];
          if (f == AWMesh::k_undefined || f == frep) break;
          const int c = 3 * f + mesh.get_jvf(v, f);
          const bool differ =
              num_wid ? corner_wid[c] != corner_wid[crep]
                      : mesh._faces[f].attrib.matid != matid || !same_attrib(corner_attrib[crep], corner_attrib[c]);
          if (differ || corner_wedge[c] >= 0) break;
          corner_wedge[c] = w;
        }
        if (f == frep) break;
      }
    }
    if (num_wid) {  // Corners with the same wid are in the same wedge even if not contiguous.
      Map<int, int> wedge_of_wid;
      for (const int f : faces) {
        const int c = 3 * f + mesh.get_jvf(v, f);
        bool is_new;
        const int w = wedge_of_wid.enter(corner_wid[c], corner_wedge[c], is_new);
        corner_wedge[c] = w;
      }
    }
  }
  for (PmWedge& wedge : wedges) {
    if (!mesh._has_rgb) wedge.attrib.rgb = A3dColor(0.f, 0.f, 0.f);
    if (!mesh._has_uv) wedge.attrib.uv = Uv(0.f, 0.f);
  }
  for_int(f, nf) for_int(j, 3) mesh._faces[f].wedges[j] = corner_wedge[3 * f + j];
  // Wedges that lost all their corners (merged by wid) remain, unused, in the array; reindex to remove them.
  Array<int> new_index(wedges.num(), -1);
  Array<PmWedge> used_wedges;
  for_int(f, nf) for_int(j, 3) {
    int& w = mesh._faces[f].wedges[j];
    if (new_index[w] < 0) {
      new_index[w] = used_wedges.num();
      used_wedges.push(wedges[w]);
    }
    w = new_index[w];
  }
  mesh._wedges = std::move(used_wedges);
}

}  // namespace

void read_mesh(const string& filename, WorkMesh& mesh) {
  assertx(mesh._vertices.empty());
  RawMesh raw;
  Array<Vector> ply_normals;
  {
    RFile fi(filename);
    if (filename.ends_with(".ply") || filename.ends_with(".ply.gz"))
      ply_normals = read_mesh_ply(fi(), mesh);
    else
      read_mesh_m(fi(), mesh, raw);
  }
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
