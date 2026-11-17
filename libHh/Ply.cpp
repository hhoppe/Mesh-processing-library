// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/Ply.h"

#include <bit>       // bit_cast(), byteswap(), endian
#include <charconv>  // to_chars()
#include <cstring>   // memcpy(), memmove()
#include <sstream>

namespace hh {

namespace {

// A scalar property type of the PLY format.
enum class PlyType : uint8_t { int8, uint8, int16, uint16, int32, uint32, float32, float64 };

PlyType ply_type(const string& name, const string& line) {
  if (name == "char" || name == "int8") return PlyType::int8;
  if (name == "uchar" || name == "uint8") return PlyType::uint8;
  if (name == "short" || name == "int16") return PlyType::int16;
  if (name == "ushort" || name == "uint16") return PlyType::uint16;
  if (name == "int" || name == "int32") return PlyType::int32;
  if (name == "uint" || name == "uint32") return PlyType::uint32;
  if (name == "float" || name == "float32") return PlyType::float32;
  if (name == "double" || name == "float64") return PlyType::float64;
  assertnever("ply: unrecognized property type '" + name + "' in '" + line + "'");
}

int type_size(PlyType type) {
  switch (type) {
    case PlyType::int8:
    case PlyType::uint8: return 1;
    case PlyType::int16:
    case PlyType::uint16: return 2;
    case PlyType::int32:
    case PlyType::uint32:
    case PlyType::float32: return 4;
    case PlyType::float64: return 8;
    default: assertnever("");
  }
}

bool is_float_type(PlyType type) { return type == PlyType::float32 || type == PlyType::float64; }

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
  [[nodiscard]] bool has_list() const {
    return ranges::any_of(properties, [](const PlyProperty& property) { return property.is_list; });
  }
  // The size in bytes of a binary instance; exact if !has_list(), else a lower bound (with empty lists).
  [[nodiscard]] int min_record_size() const {
    int size = 0;
    for (const PlyProperty& property : properties)
      size += type_size(property.is_list ? property.count_type : property.type);
    return size;
  }
  // Returns the index of the scalar property with the given name, or -1.
  [[nodiscard]] int find_scalar(std::string_view property_name) const {
    for_int(i, properties.num()) if (properties[i].name == property_name && !properties[i].is_list) return i;
    return -1;
  }
};

template <typename T> T load_raw(const char* p, bool swap) {
  T value;
  std::memcpy(&value, p, sizeof(T));
  if constexpr (sizeof(T) > 1) {
    if (swap) value = std::byteswap(value);
  }
  return value;
}

double load_value(const char* p, PlyType type, bool swap) {
  switch (type) {
    case PlyType::int8: return load_raw<int8_t>(p, swap);
    case PlyType::uint8: return load_raw<uint8_t>(p, swap);
    case PlyType::int16: return load_raw<int16_t>(p, swap);
    case PlyType::uint16: return load_raw<uint16_t>(p, swap);
    case PlyType::int32: return load_raw<int32_t>(p, swap);
    case PlyType::uint32: return load_raw<uint32_t>(p, swap);
    case PlyType::float32: return std::bit_cast<float>(load_raw<uint32_t>(p, swap));
    case PlyType::float64: return std::bit_cast<double>(load_raw<uint64_t>(p, swap));
    default: assertnever("");
  }
}

// The number of bytes of fixed-size records that are read at once in binary.
constexpr int k_chunk_bytes = 1 << 20;

// Reads the values of the element instances: in ASCII, one line per instance; in binary, through a buffer that is
// refilled with the requested bytes plus at most the minimum size of the instances not yet begun, so that the stream
// is never read beyond the PLY data (which other data may follow).
class PlyValueReader {
 public:
  PlyValueReader(std::istream& is, bool binary, bool big_endian, int64_t min_data_size)
      : _is(is),
        _binary(binary),
        _swap(big_endian != (std::endian::native == std::endian::big)),
        _unbegun_min_size(min_data_size) {}
  [[nodiscard]] bool binary() const { return _binary; }
  [[nodiscard]] bool swap() const { return _swap; }
  // Begin n instances of the element, which are read in binary through a single read_bytes().
  void begin_instances(const PlyElement& element, int n) {
    _unbegun_min_size -= int64_t{n} * element.min_record_size();
    ASSERTX(_unbegun_min_size >= 0);
  }
  // Begin instance i of the element; in ASCII, read its line.
  void begin_instance(const PlyElement& element, int i) {
    begin_instances(element, 1);
    if (_binary) return;
    for (;;) {
      if (!my_getline(_is, _line, false))
        assertnever(sform("ply: premature end of file in %s %d", element.name.c_str(), i));
      _s = _line.c_str();
      skip_spaces();
      if (*_s) break;  // Blank lines are ignored.
    }
  }
  // In ASCII, check that the line of the instance has no more values.
  void end_instance() {
    if (_binary) return;
    skip_spaces();
    if (*_s) assertnever("ply: too many values on line '" + _line + "'");
  }
  double read(PlyType type) {
    if (_binary) return load_value(read_bytes(type_size(type)), type, _swap);
    skip_spaces();
    if (!*_s) assertnever("ply: too few values on line '" + _line + "'");
    char* end;
    // Parse a float directly (rather than through a double) to get the correctly rounded value.
    const double value = type == PlyType::float32 ? std::strtof(_s, &end) : std::strtod(_s, &end);
    if (end == _s) assertnever("ply: cannot parse a value in line '" + _line + "'");
    _s = end;
    return value;
  }
  // Read count values of the given type, calling f(value) on each.
  void read_values(PlyType type, int count, const auto& f) {
    if (_binary) {
      const int size = type_size(type);
      const char* p = read_bytes(assert_narrow_cast<int>(int64_t{count} * size));
      for_int(k, count) {
        f(load_value(p, type, _swap));
        p += size;
      }
    } else {
      for_int(k, count) f(read(type));
    }
  }
  void skip(PlyType type, int count) {
    if (_binary) {
      (void)read_bytes(assert_narrow_cast<int>(int64_t{count} * type_size(type)));
    } else {
      for_int(k, count) { (void)read(type); }
    }
  }
  // Read the count of a list property.
  int read_count(PlyType type) {
    const double value = read(type);
    if (!(value >= 0. && value <= 1e9 && value == std::floor(value)))
      assertnever(sform("ply: invalid list count %g", value));
    return int(value);
  }
  // In binary, returns a pointer to the next n bytes, valid until the next read.
  const char* read_bytes(int n) {
    if (_end - _pos < n) refill(n);
    ASSERTX(_end - _pos >= n);
    const char* p = _buf.data() + _pos;
    _pos += n;
    return p;
  }

 private:
  std::istream& _is;
  bool _binary;
  bool _swap;
  string _line;  // ASCII.
  const char* _s{nullptr};
  Array<char> _buf;  // Binary; the bytes [_pos, _end) are not yet consumed.
  int _pos{0};
  int _end{0};
  int64_t _unbegun_min_size;  // A lower bound on the size of the binary data of the instances not yet begun.
  void skip_spaces() {
    while (*_s == ' ' || *_s == '\t' || *_s == '\r') _s++;
  }
  void refill(int n) {
    const int remaining = _end - _pos;
    if (remaining) std::memmove(_buf.data(), _buf.data() + _pos, remaining);
    _pos = 0;
    _end = remaining;
    const int size = n + int(min(_unbegun_min_size, int64_t{k_chunk_bytes}));
    if (size > _buf.num()) _buf.resize(size);
    _end += int(_is.rdbuf()->sgetn(_buf.data() + _end, size - _end));
    if (_end < n) assertnever("ply: premature end of binary data");
  }
};

// Skip all the instances of an element.
void skip_element(PlyValueReader& reader, const PlyElement& element) {
  if (reader.binary() && !element.has_list()) {
    const int record_size = element.min_record_size();
    const int chunk = max(k_chunk_bytes / max(record_size, 1), 1);
    for (int i = 0; i < element.count; i += chunk) {
      const int n = min(chunk, element.count - i);
      reader.begin_instances(element, n);
      (void)reader.read_bytes(n * record_size);
    }
    return;
  }
  for_int(i, element.count) {
    reader.begin_instance(element, i);
    for (const PlyProperty& property : element.properties) {
      const int count = property.is_list ? reader.read_count(property.count_type) : 1;
      reader.skip(property.type, count);
    }
    reader.end_instance();
  }
}

uint8_t color_component(double value, PlyType type) {
  if (is_float_type(type)) {
    const double scaled = value * 255.;
    return scaled >= 255. ? 255 : scaled > 0. ? uint8_t(scaled + .5) : 0;  // Also maps NaN to 0.
  }
  if (!(value >= 0. && value <= 255.)) assertnever(sform("ply: color value %g is outside [0, 255]", value));
  return uint8_t(value);
}

// The vertex properties that are used.
enum EVertexSlot { k_x, k_y, k_z, k_nx, k_ny, k_nz, k_red, k_green, k_blue, k_alpha, k_u, k_v, k_num_slots };

void read_vertices(PlyValueReader& reader, const PlyElement& element, PlyMesh& mesh, Array<string>& ignored) {
  Vec<int, k_num_slots> iprop;  // Index of the property of each slot, or -1.
  fill(iprop, -1);
  // Assign the named properties to the consecutive slots starting at slot0 if all of them are present.
  const auto assign = [&](int slot0, std::initializer_list<const char*> names) {
    for (const char* name : names)
      if (element.find_scalar(name) < 0) return false;
    int slot = slot0;
    for (const char* name : names) iprop[slot++] = element.find_scalar(name);
    return true;
  };
  for (const char* name : {"x", "y", "z"})
    if (element.find_scalar(name) < 0) assertnever(sform("ply: the vertex element lacks a property '%s'", name));
  assertx(assign(k_x, {"x", "y", "z"}));
  const bool has_normals = assign(k_nx, {"nx", "ny", "nz"});
  const bool has_colors = assign(k_red, {"red", "green", "blue"});
  const bool has_alpha = has_colors && assign(k_alpha, {"alpha"});
  const bool has_uvs = assign(k_u, {"s", "t"}) || assign(k_u, {"u", "v"}) || assign(k_u, {"texture_u", "texture_v"});
  for_int(i, element.properties.num()) {
    if (!contains(iprop, i)) ignored.push("vertex property '" + element.properties[i].name + "'");
  }
  Array<int> slot(element.properties.num(), -1);
  for_int(s, k_num_slots) if (iprop[s] >= 0) slot[iprop[s]] = s;
  Vec<PlyType, k_num_slots> slot_type;
  fill(slot_type, PlyType::float32);
  for_int(s, k_num_slots) if (iprop[s] >= 0) slot_type[s] = element.properties[iprop[s]].type;
  const int nv = element.count;
  mesh.points.init(nv);
  if (has_normals) mesh.normals.init(nv);
  if (has_colors) mesh.colors.init(nv);
  if (has_uvs) mesh.uvs.init(nv);
  Vec<double, k_num_slots> values;
  fill(values, 0.);
  const auto store = [&](int v) {
    for_int(c, 3) mesh.points[v][c] = float(values[k_x + c]);
    if (has_normals) for_int(c, 3) mesh.normals[v][c] = float(values[k_nx + c]);
    if (has_colors) {
      Pixel& pixel = mesh.colors[v];
      for_int(c, 3) pixel[c] = color_component(values[k_red + c], slot_type[k_red + c]);
      pixel[3] = has_alpha ? color_component(values[k_alpha], slot_type[k_alpha]) : 255;
    }
    if (has_uvs) for_int(c, 2) mesh.uvs[v][c] = float(values[k_u + c]);
  };
  if (reader.binary() && !element.has_list()) {  // Fast path: fixed-size records.
    const int record_size = element.min_record_size();
    Vec<int, k_num_slots> offset;
    fill(offset, 0);
    int off = 0;
    for_int(i, element.properties.num()) {
      if (slot[i] >= 0) offset[slot[i]] = off;
      off += type_size(element.properties[i].type);
    }
    Array<int> used_slots;
    for_int(s, k_num_slots) if (iprop[s] >= 0) used_slots.push(s);
    const bool swap = reader.swap();
    const int chunk = max(k_chunk_bytes / max(record_size, 1), 1);
    for (int v0 = 0; v0 < nv; v0 += chunk) {
      const int n = min(chunk, nv - v0);
      reader.begin_instances(element, n);
      const char* record = reader.read_bytes(n * record_size);
      for_int(i, n) {
        for (const int s : used_slots) values[s] = load_value(record + offset[s], slot_type[s], swap);
        store(v0 + i);
        record += record_size;
      }
    }
    return;
  }
  for_int(v, nv) {
    reader.begin_instance(element, v);
    for_int(i, element.properties.num()) {
      const PlyProperty& property = element.properties[i];
      if (property.is_list) {
        const int count = reader.read_count(property.count_type);
        reader.skip(property.type, count);
      } else {
        const double value = reader.read(property.type);
        if (slot[i] >= 0) values[slot[i]] = value;
      }
    }
    reader.end_instance();
    store(v);
  }
}

void read_faces(PlyValueReader& reader, const PlyElement& element, int nv, PlyMesh& mesh, Array<string>& ignored) {
  const auto find_list = [&](std::string_view name) {
    for_int(i, element.properties.num()) {
      const PlyProperty& property = element.properties[i];
      if (property.name != name) continue;
      if (!property.is_list) assertnever("ply: face property '" + property.name + "' is not a list");
      return i;
    }
    return -1;
  };
  int iindices = find_list("vertex_indices");
  if (iindices < 0) iindices = find_list("vertex_index");  // A less common spelling.
  if (iindices < 0) assertnever("ply: the face element lacks a list 'vertex_indices'");
  const int itexcoord = find_list("texcoord");
  for_int(i, element.properties.num()) {
    if (i != iindices && i != itexcoord) ignored.push("face property '" + element.properties[i].name + "'");
  }
  const int nf = element.count;
  const int f0 = mesh.num_faces();
  mesh.face_start.reserve(f0 + nf + 1);
  mesh.face_vertices.reserve(mesh.face_vertices.num() + int(min(int64_t{nf} * 3, int64_t{1} << 30)));
  Array<float> texcoords;
  int num_small_faces = 0;
  for_int(f, nf) {
    reader.begin_instance(element, f);
    int nfv = 0;
    texcoords.clear();
    for_int(i, element.properties.num()) {
      const PlyProperty& property = element.properties[i];
      const int count = property.is_list ? reader.read_count(property.count_type) : 1;
      if (i == iindices) {
        nfv = count;
        reader.read_values(property.type, count, [&](double value) {
          if (!(value >= 0. && value < nv && value == std::floor(value)))
            assertnever(sform("ply: face %d has an invalid vertex index %.17g (the number of vertices is %d)", f0 + f,
                              value, nv));
          mesh.face_vertices.push(int(value));
        });
      } else if (i == itexcoord) {
        reader.read_values(property.type, count, [&](double value) { texcoords.push(float(value)); });
      } else {
        reader.skip(property.type, count);
      }
    }
    reader.end_instance();
    if (nfv < 3) num_small_faces++;
    mesh.face_start.push(mesh.face_vertices.num());
    if (itexcoord >= 0) {
      if (f0) assertnever("ply: a face list texcoord after other faces");
      if (texcoords.num() != 2 * nfv)
        assertnever(sform("ply: face %d has %d texcoord values for its %d vertices", f, texcoords.num(), nfv));
      for_int(k, nfv) mesh.corner_uvs.push(Uv(texcoords[2 * k + 0], texcoords[2 * k + 1]));
    }
  }
  if (num_small_faces) {
    Warning("ply: face has fewer than 3 vertices");
    showdf("(ply: %d faces have fewer than 3 vertices)\n", num_small_faces);
  }
}

// Read the triangle strips of the element "tristrips" (its list "vertex_indices", in which each index -1 starts a new
// strip) as triangle faces, omitting the degenerate ones; triangle k of a strip is (k, k + 1, k + 2) if k is even, and
// (k + 1, k, k + 2) if k is odd, so that the triangles have consistent orientations.
void read_tristrips(PlyValueReader& reader, const PlyElement& element, int nv, PlyMesh& mesh, Array<string>& ignored) {
  int iindices = -1;
  for_int(i, element.properties.num()) {
    const PlyProperty& property = element.properties[i];
    if (property.name == "vertex_indices" && property.is_list) {
      iindices = i;
    } else {
      ignored.push("tristrips property '" + property.name + "'");
    }
  }
  if (iindices < 0) assertnever("ply: the tristrips element lacks a list 'vertex_indices'");
  if (mesh.corner_uvs.num()) assertnever("ply: tristrips after faces with texcoord");
  Array<int> strip;
  const auto end_strip = [&] { strip.init(0); };
  for_int(i, element.count) {
    reader.begin_instance(element, i);
    for_int(k, element.properties.num()) {
      const PlyProperty& property = element.properties[k];
      const int count = property.is_list ? reader.read_count(property.count_type) : 1;
      if (k != iindices) {
        reader.skip(property.type, count);
        continue;
      }
      reader.read_values(property.type, count, [&](double value) {
        if (value == -1. || value == 4294967295.) return end_strip();  // (The latter if read as unsigned.)
        if (!(value >= 0. && value < nv && value == std::floor(value)))
          assertnever(
              sform("ply: tristrips has an invalid vertex index %.17g (the number of vertices is %d)", value, nv));
        strip.push(int(value));
        const int n = strip.num();
        if (n < 3) return;
        const int v0 = strip[n - 3], v1 = strip[n - 2], v2 = strip[n - 1];
        if (v0 == v1 || v1 == v2 || v2 == v0) return;
        const bool odd = (n - 3) % 2 != 0;
        for (const int v : {odd ? v1 : v0, odd ? v0 : v1, v2}) mesh.face_vertices.push(v);
        mesh.face_start.push(mesh.face_vertices.num());
      });
    }
    reader.end_instance();
    end_strip();
  }
}

// Read all instances of an element that has only scalar properties, calling f(i, k, value) for property k of
// instance i.
void read_scalar_element(PlyValueReader& reader, const PlyElement& element, const auto& f) {
  if (element.has_list()) assertnever("ply: element '" + element.name + "' has a list property");
  const int np = element.properties.num();
  if (reader.binary()) {
    const int record_size = element.min_record_size();
    Array<int> offset(np);
    int off = 0;
    for_int(k, np) {
      offset[k] = off;
      off += type_size(element.properties[k].type);
    }
    const bool swap = reader.swap();
    const int chunk = max(k_chunk_bytes / max(record_size, 1), 1);
    for (int i0 = 0; i0 < element.count; i0 += chunk) {
      const int n = min(chunk, element.count - i0);
      reader.begin_instances(element, n);
      const char* record = reader.read_bytes(n * record_size);
      for_int(i, n) {
        for_int(k, np) f(i0 + i, k, load_value(record + offset[k], element.properties[k].type, swap));
        record += record_size;
      }
    }
    return;
  }
  for_int(i, element.count) {
    reader.begin_instance(element, i);
    for_int(k, np) f(i, k, reader.read(element.properties[k].type));
    reader.end_instance();
  }
}

// Returns the value as a nonnegative integer less than bound, or dies.
int checked_index(double value, int bound, const char* what) {
  if (!(value >= 0. && value < bound && value == std::floor(value)))
    assertnever(sform("ply: invalid %s %.17g (the bound is %d)", what, value, bound));
  return int(value);
}

// If the element is a "wedge_<key>" element in one of the two recognized forms (see Ply.h), returns the number of
// components of its values (0 for a flag), else -1.
int wedge_key_dim(const PlyElement& element) {
  const auto& properties = element.properties;
  if (properties.num() == 1 && properties[0].name == "flag" && !properties[0].is_list &&
      properties[0].type == PlyType::uint8)
    return 0;
  if (properties.empty()) return -1;
  for_int(k, properties.num()) {
    if (properties[k].name != sform("c%d", k) || properties[k].is_list || !is_float_type(properties[k].type))
      return -1;
  }
  return properties.num();
}

}  // namespace

int PlyMesh::num_mesh_vertices() const {
  if (wedge_vertex.empty()) return points.num();
  int nv = 0;
  for (const int v : wedge_vertex) nv = max(nv, v + 1);
  return nv;
}

Array<string> read_ply_header(std::istream& is) {
  Array<string> lines;
  string line;
  for (;;) {
    if (!my_getline(is, line, false))
      assertnever(lines.empty() ? "ply: the file is empty" : "ply: premature end of file in the header");
    while (!line.empty() && (line.back() == ' ' || line.back() == '\t' || line.back() == '\r')) line.pop_back();
    if (lines.empty() && line != "ply") assertnever("ply: the file starts with '" + line + "' rather than 'ply'");
    lines.push(line);
    if (line == "end_header") break;
  }
  return lines;
}

PlyMesh read_ply_body(std::istream& is, CArrayView<string> header_lines) {
  if (header_lines.empty() || header_lines[0] != "ply") assertnever("ply: the header does not start with 'ply'");
  PlyMesh mesh;
  mesh.face_start.init(1, 0);
  string format;
  Array<PlyElement> elements;
  for (const string& line : header_lines.slice(1)) {
    std::istringstream iss(line);
    string keyword;
    iss >> keyword;
    if (keyword == "format") {
      if (!(iss >> format)) assertnever("ply: bad line '" + line + "'");
    } else if (keyword == "element") {
      PlyElement element;
      if (!(iss >> element.name >> element.count) || element.count < 0) assertnever("ply: bad line '" + line + "'");
      elements.push(std::move(element));
    } else if (keyword == "property") {
      if (elements.empty()) assertnever("ply: property before any element in '" + line + "'");
      PlyProperty property;
      string type;
      iss >> type;
      if (type == "list") {
        string count_type, item_type;
        iss >> count_type >> item_type;
        property.is_list = true;
        property.count_type = ply_type(count_type, line);
        property.type = ply_type(item_type, line);
      } else {
        property.type = ply_type(type, line);
      }
      if (!(iss >> property.name)) assertnever("ply: bad line '" + line + "'");
      elements.last().properties.push(std::move(property));
    } else if (keyword == "end_header") {
      break;
    } else if (keyword == "comment") {
      const std::string_view rest = std::string_view(line).substr(min(line.size(), size_t{8}));  // After "comment ".
      if (rest.starts_with("material ")) {
        std::istringstream iss2{string(rest.substr(9))};
        int matid;
        if (!(iss2 >> matid) || matid < 0 || matid >= 65536) assertnever("ply: bad line '" + line + "'");
        if (iss2.peek() == ' ') iss2.get();
        string material;
        std::getline(iss2, material);
        if (matid >= mesh.materials.num()) mesh.materials.resize(matid + 1);
        mesh.materials[matid] = material;
      } else {
        mesh.comments.push(string(rest));
      }
    } else if (keyword != "obj_info" && keyword != "") {
      assertnever("ply: unrecognized header line '" + line + "'");
    }
  }
  bool binary = false, big_endian = false;
  if (format == "binary_little_endian") {
    binary = true;
  } else if (format == "binary_big_endian") {
    binary = true;
    big_endian = true;
  } else if (format != "ascii") {
    assertnever(format == "" ? "ply: the header lacks a format line" : "ply: unrecognized format '" + format + "'");
  }
  int ivertex = -1, iface = -1;
  for_int(i, elements.num()) {
    for (auto [name, index] : {std::pair{"vertex", &ivertex}, std::pair{"face", &iface}}) {
      if (elements[i].name != name) continue;
      if (*index >= 0) assertnever("ply: more than one element '" + elements[i].name + "'");
      *index = i;
    }
  }
  if (ivertex < 0) assertnever("ply: the header lacks a vertex element");
  const int nv = elements[ivertex].count;
  Array<string> ignored;
  int64_t min_data_size = 0;
  for (const PlyElement& element : elements) min_data_size += int64_t{element.count} * element.min_record_size();
  PlyValueReader reader(is, binary, big_endian, min_data_size);
  for_int(i, elements.num()) {
    const PlyElement& element = elements[i];
    const auto check_count = [&](int count) {
      if (element.count != count)
        assertnever(
            sform("ply: element '%s' has %d instances rather than %d", element.name.c_str(), element.count, count));
    };
    if (i == ivertex) {
      read_vertices(reader, element, mesh, ignored);
    } else if (i == iface) {
      read_faces(reader, element, nv, mesh, ignored);
    } else if (element.name == "tristrips") {
      read_tristrips(reader, element, nv, mesh, ignored);
    } else if (element.name == "wedge_vertex" && element.properties.num() == 1 &&
               element.properties[0].name == "vertex") {
      check_count(nv);
      mesh.wedge_vertex.init(nv);
      read_scalar_element(reader, element, [&](int w, int, double value) {
        mesh.wedge_vertex[w] = checked_index(value, nv, "wedge vertex");
      });
    } else if (element.name == "face_material" && element.properties.num() == 1 &&
               element.properties[0].name == "matid") {
      if (iface < 0) assertnever("ply: element face_material without a face element");
      check_count(elements[iface].count);
      mesh.face_matids.init(element.count);
      read_scalar_element(reader, element, [&](int f, int, double value) {
        mesh.face_matids[f] = checked_index(value, 65536, "face matid");
      });
    } else if (element.name.starts_with("wedge_") && element.name.size() > 6 && wedge_key_dim(element) >= 0) {
      check_count(nv);
      const int dim = wedge_key_dim(element);
      mesh.wedge_keys.push(PlyMesh::WedgeKey{element.name.substr(6), dim, {}});
      PlyMesh::WedgeKey& wedge_key = mesh.wedge_keys.last();
      wedge_key.values.init(nv * max(dim, 1));
      read_scalar_element(reader, element, [&](int w, int k, double value) {
        wedge_key.values[w * max(dim, 1) + k] = dim ? float(value) : value ? 1.f : 0.f;
      });
    } else {
      ignored.push("element '" + element.name + "'");
      skip_element(reader, element);
    }
  }
  if (mesh.face_matids.num() && mesh.face_matids.num() != mesh.num_faces())
    assertnever("ply: element face_material together with tristrips");
  if (mesh.face_matids.num()) {
    const int max_matid = max(mesh.face_matids);
    if (max_matid >= mesh.materials.num()) mesh.materials.resize(max_matid + 1);
  }
  if (!ignored.empty()) {
    string s;
    for (const string& item : ignored) s += (s == "" ? "" : ", ") + item;
    showdf("ply: ignoring %s\n", s.c_str());
  }
  return mesh;
}

PlyMesh read_ply(std::istream& is) {
  const Array<string> header_lines = read_ply_header(is);
  return read_ply_body(is, header_lines);
}

namespace {

// Writes the values of the PLY body, in binary big-endian (through a buffer) or in ASCII (one line per instance).
class PlyValueWriter {
 public:
  PlyValueWriter(std::ostream& os, bool binary) : _os(os), _binary(binary) {}
  ~PlyValueWriter() { flush(); }
  PlyValueWriter(const PlyValueWriter&) = delete;
  PlyValueWriter& operator=(const PlyValueWriter&) = delete;
  void put(float value) {
    if (_binary) return put_raw(std::bit_cast<uint32_t>(value));
    separate();
    char buf[32];
    _buf.append(buf, std::to_chars(buf, buf + sizeof(buf), value).ptr);  // The shortest exact representation.
  }
  template <typename T> void put_int(T value) {
    static_assert(std::is_integral_v<T>);
    if (_binary) return put_raw(value);
    separate();
    char buf[32];
    _buf.append(buf, std::to_chars(buf, buf + sizeof(buf), value).ptr);
  }
  void end_instance() {
    if (!_binary) _buf += '\n', _at_line_start = true;
    if (_buf.size() >= k_flush_size) flush();
  }

 private:
  static constexpr size_t k_flush_size = 1 << 20;
  std::ostream& _os;
  bool _binary;
  bool _at_line_start{true};
  string _buf;
  void separate() {
    if (!_at_line_start) _buf += ' ';
    _at_line_start = false;
  }
  template <typename T> void put_raw(T value) {
    if constexpr (sizeof(T) > 1 && std::endian::native != std::endian::big) value = std::byteswap(value);
    char bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    _buf.append(bytes, sizeof(T));
  }
  void flush() {
    _os.write(_buf.data(), std::streamsize(_buf.size()));
    _buf.clear();
  }
};

}  // namespace

void write_ply(std::ostream& os, const PlyMesh& mesh, bool binary) {
  const int nv = mesh.points.num(), nf = mesh.num_faces();
  assertx(mesh.normals.empty() || mesh.normals.num() == nv);
  assertx(mesh.colors.empty() || mesh.colors.num() == nv);
  assertx(mesh.uvs.empty() || mesh.uvs.num() == nv);
  assertx(mesh.corner_uvs.empty() || mesh.corner_uvs.num() == mesh.face_vertices.num());
  assertx(mesh.wedge_vertex.empty() || mesh.wedge_vertex.num() == nv);
  assertx(mesh.face_matids.empty() || mesh.face_matids.num() == nf);
  const bool has_alpha = ranges::any_of(mesh.colors, [](const Pixel& pixel) { return pixel[3] != 255; });
  int max_face_size = 0;
  for_int(f, nf) max_face_size = max(max_face_size, mesh.face_start[f + 1] - mesh.face_start[f]);
  const bool int_count = max_face_size > 255;
  os << "ply\n";
  os << "format " << (binary ? "binary_big_endian" : "ascii") << " 1.0\n";
  for (const string& comment : mesh.comments) {
    assertx(comment.find('\n') == string::npos);
    os << "comment" << (comment == "" ? "" : " ") << comment << "\n";
  }
  for_int(matid, mesh.materials.num()) {
    assertx(mesh.materials[matid].find('\n') == string::npos);
    const string& material = mesh.materials[matid];
    os << "comment material " << matid << (material == "" ? "" : " ") << material << "\n";
  }
  os << "element vertex " << nv << "\n";
  for (const char* name : {"x", "y", "z"}) os << "property float " << name << "\n";
  if (mesh.normals.num())
    for (const char* name : {"nx", "ny", "nz"}) os << "property float " << name << "\n";
  if (mesh.colors.num())
    for (const char* name : {"red", "green", "blue", "alpha"})
      if (name != std::string_view("alpha") || has_alpha) os << "property uchar " << name << "\n";
  if (mesh.uvs.num())
    for (const char* name : {"s", "t"}) os << "property float " << name << "\n";
  os << "element face " << nf << "\n";
  os << "property list " << (int_count ? "int" : "uchar") << " int vertex_indices\n";
  if (mesh.corner_uvs.num()) os << "property list " << (int_count ? "int" : "uchar") << " float texcoord\n";
  if (mesh.wedge_vertex.num()) os << "element wedge_vertex " << nv << "\nproperty int vertex\n";
  if (mesh.face_matids.num()) os << "element face_material " << nf << "\nproperty ushort matid\n";
  for (const PlyMesh::WedgeKey& wedge_key : mesh.wedge_keys) {
    assertx(wedge_key.name != "" && wedge_key.name != "vertex" &&
            wedge_key.name.find_first_of(" \t\n") == string::npos);
    assertx(wedge_key.dim >= 0 && wedge_key.values.num() == nv * max(wedge_key.dim, 1));
    os << "element wedge_" << wedge_key.name << " " << nv << "\n";
    if (wedge_key.dim == 0) os << "property uchar flag\n";
    for_int(k, wedge_key.dim) os << "property float c" << k << "\n";
  }
  os << "end_header\n";
  PlyValueWriter writer(os, binary);
  for_int(v, nv) {
    for_int(c, 3) writer.put(mesh.points[v][c]);
    if (mesh.normals.num()) for_int(c, 3) writer.put(mesh.normals[v][c]);
    if (mesh.colors.num()) for_int(c, has_alpha ? 4 : 3) writer.put_int(mesh.colors[v][c]);
    if (mesh.uvs.num()) for_int(c, 2) writer.put(mesh.uvs[v][c]);
    writer.end_instance();
  }
  const auto put_count = [&](int count) {
    if (int_count) {
      writer.put_int(int32_t{count});
    } else {
      writer.put_int(uint8_t(count));
    }
  };
  for_int(f, nf) {
    const CArrayView<int> face = mesh.face(f);
    put_count(face.num());
    for (const int v : face) {
      assertx(v >= 0 && v < nv);
      writer.put_int(int32_t{v});
    }
    if (mesh.corner_uvs.num()) {
      put_count(2 * face.num());
      for_int(k, face.num()) for_int(c, 2) writer.put(mesh.corner_uvs[mesh.face_start[f] + k][c]);
    }
    writer.end_instance();
  }
  for (const int v : mesh.wedge_vertex) {
    writer.put_int(int32_t{v});
    writer.end_instance();
  }
  for (const int matid : mesh.face_matids) {
    writer.put_int(assert_narrow_cast<uint16_t>(matid));
    writer.end_instance();
  }
  for (const PlyMesh::WedgeKey& wedge_key : mesh.wedge_keys) {
    for_int(v, nv) {
      if (wedge_key.dim == 0) {
        writer.put_int(uint8_t(wedge_key.values[v] ? 1 : 0));
      } else {
        for_int(k, wedge_key.dim) writer.put(wedge_key.values[v * wedge_key.dim + k]);
      }
      writer.end_instance();
    }
  }
}

}  // namespace hh
