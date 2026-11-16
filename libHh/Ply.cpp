// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/Ply.h"

#include <bit>      // bit_cast(), byteswap(), endian
#include <cstring>  // memcpy(), memmove()
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
  [[nodiscard]] int record_size() const {  // Defined if !has_list().
    int size = 0;
    for (const PlyProperty& property : properties) size += type_size(property.type);
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

// Reads the values of the element instances: in ASCII, one line per instance; in binary, through a buffer that is
// refilled in large chunks.
class PlyValueReader {
 public:
  PlyValueReader(std::istream& is, bool binary, bool big_endian)
      : _is(is), _binary(binary), _swap(big_endian != (std::endian::native == std::endian::big)) {
    if (_binary) _buf.init(k_buffer_size);
  }
  [[nodiscard]] bool binary() const { return _binary; }
  [[nodiscard]] bool swap() const { return _swap; }
  // In ASCII, read the line of instance i of the named element.
  void begin_instance(const string& element_name, int i) {
    if (_binary) return;
    for (;;) {
      if (!my_getline(_is, _line, false))
        assertnever(sform("ply: premature end of file in %s %d", element_name.c_str(), i));
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
  void skip(PlyType type, int count) {
    for_int(k, count) { (void)read(type); }
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
    const char* p = _buf.data() + _pos;
    _pos += n;
    return p;
  }

 private:
  static constexpr int k_buffer_size = 1 << 20;
  std::istream& _is;
  bool _binary;
  bool _swap;
  string _line;  // ASCII.
  const char* _s{nullptr};
  Array<char> _buf;  // Binary; the bytes [_pos, _end) are not yet consumed.
  int _pos{0};
  int _end{0};
  void skip_spaces() {
    while (*_s == ' ' || *_s == '\t' || *_s == '\r') _s++;
  }
  void refill(int n) {
    const int remaining = _end - _pos;
    std::memmove(_buf.data(), _buf.data() + _pos, remaining);
    _pos = 0;
    _end = remaining;
    if (n > _buf.num()) _buf.resize(n);
    _is.read(_buf.data() + _end, _buf.num() - _end);
    _end += int(_is.gcount());
    if (_end < n) assertnever("ply: premature end of binary data");
  }
};

// Skip all the instances of an element.
void skip_element(PlyValueReader& reader, const PlyElement& element) {
  if (reader.binary() && !element.has_list()) {
    const int record_size = element.record_size();
    const int chunk = max((1 << 20) / max(record_size, 1), 1);
    for (int i = 0; i < element.count; i += chunk)
      (void)reader.read_bytes(min(chunk, element.count - i) * record_size);
    return;
  }
  for_int(i, element.count) {
    reader.begin_instance(element.name, i);
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
    const int record_size = element.record_size();
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
    for_int(v, nv) {
      const char* record = reader.read_bytes(record_size);
      for (const int s : used_slots) values[s] = load_value(record + offset[s], slot_type[s], swap);
      store(v);
    }
    return;
  }
  for_int(v, nv) {
    reader.begin_instance(element.name, v);
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
  mesh.face_start.init(nf + 1);
  mesh.face_start[0] = 0;
  mesh.face_vertices.reserve(int(min(int64_t{nf} * 3, int64_t{1} << 30)));
  Array<float> texcoords;
  int num_small_faces = 0;
  for_int(f, nf) {
    reader.begin_instance(element.name, f);
    int nfv = 0;
    texcoords.clear();
    for_int(i, element.properties.num()) {
      const PlyProperty& property = element.properties[i];
      const int count = property.is_list ? reader.read_count(property.count_type) : 1;
      if (i == iindices) {
        nfv = count;
        for_int(k, count) {
          const double value = reader.read(property.type);
          if (!(value >= 0. && value < nv && value == std::floor(value)))
            assertnever(
                sform("ply: face %d has an invalid vertex index %.17g (the number of vertices is %d)", f, value, nv));
          mesh.face_vertices.push(int(value));
        }
      } else if (i == itexcoord) {
        for_int(k, count) texcoords.push(float(reader.read(property.type)));
      } else {
        reader.skip(property.type, count);
      }
    }
    reader.end_instance();
    if (nfv < 3) num_small_faces++;
    mesh.face_start[f + 1] = mesh.face_vertices.num();
    if (itexcoord >= 0) {
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

}  // namespace

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
    } else if (keyword != "comment" && keyword != "obj_info" && keyword != "") {
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
  PlyMesh mesh;
  mesh.face_start.init(1, 0);
  Array<string> ignored;
  PlyValueReader reader(is, binary, big_endian);
  const int ilast = max(ivertex, iface);  // The elements beyond it need not be read.
  for_int(i, elements.num()) {
    const PlyElement& element = elements[i];
    if (i == ivertex) {
      read_vertices(reader, element, mesh, ignored);
    } else if (i == iface) {
      read_faces(reader, element, nv, mesh, ignored);
    } else {
      ignored.push("element '" + element.name + "'");
      if (i < ilast) skip_element(reader, element);
    }
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

}  // namespace hh
