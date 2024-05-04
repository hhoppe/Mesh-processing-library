// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/FrameIO.h"

#include "libHh/BinaryIO.h"
#include "libHh/Buffer.h"
#include "libHh/NetworkOrder.h"

namespace hh {

namespace {

constexpr int k_binary_code = 2;

constexpr float k_Frame_fnan = 1e30f;

struct frame_binary_buf {
  Vec2<char> magic;
  ushort short_obn;
  Vec4<Vec3<float>> f;
  float zoom;
};

void decode(std::istream& is, ObjectFrame& object_frame) {
  string sline;
  assertx(my_getline(is, sline));
  assertx(sline != "");
  assertx(sline[0] == 'F' && sline[1] == ' ');
  Frame& f = object_frame.frame;
  assertx(sscanf(sline.c_str() + 1, "%d  %f %f %f  %f %f %f  %f %f %f  %f %f %f  %f",  // + 1 to skip ch
                 &object_frame.obn, &f[0][0], &f[0][1], &f[0][2], &f[1][0], &f[1][1], &f[1][2], &f[2][0], &f[2][1],
                 &f[2][2], &f[3][0], &f[3][1], &f[3][2], &object_frame.zoom) == 14);
  object_frame.binary = false;
}

}  // namespace

namespace FrameIO {

ERecognize recognize(RBuffer& b) {
  if (b.num() < 2) return ERecognize::no;
  if (b[0] == k_binary_code) return b.num() >= 14 * 4 ? ERecognize::yes : ERecognize::partial;  // binary record
  if (b[0] == 'F' && b[1] == ' ') {
    return b.has_line() ? ERecognize::yes : ERecognize::partial;
  }
  return ERecognize::no;
}

bool read(std::istream& is, ObjectFrame& object_frame) {
  int c = is.peek();
  if (c < 0) return false;
  object_frame.binary = c == k_binary_code;
  if (object_frame.binary) {
    frame_binary_buf buf;
    if (!read_binary_raw(is, ArView(buf))) return false;
    assertx(buf.magic[1] == 0);
    from_std(&buf.short_obn);
    object_frame.obn = buf.short_obn;
    for_int(i, 4) {
      for_int(j, 3) {
        from_std(&buf.f[i][j]);
        object_frame.frame[i][j] = buf.f[i][j];
      }
    }
    from_std(&buf.zoom);
    object_frame.zoom = buf.zoom;
    return true;
  } else {
    decode(is, object_frame);
    return bool(is);
  }
}

bool read(RBuffer& b, ObjectFrame& object_frame) {
  if (!b.num()) return false;
  object_frame.binary = b[0] == k_binary_code;
  if (object_frame.binary) {
    if (b.num() < 14 * 4) return false;
    object_frame.obn = b.get_short(2);
    for_int(i, 4) for_int(j, 3) object_frame.frame[i][j] = b.get_float((1 + (i * 3) + j) * 4);
    object_frame.zoom = b.get_float(13 * 4);
    b.extract(14 * 4);
    return true;
  } else {
    if (b[0] != 'F') return false;
    string str;
    if (!b.extract_line(str)) return false;
    std::istringstream iss(str);
    decode(iss, object_frame);
    return bool(iss);
  }
}

bool write(std::ostream& os, const ObjectFrame& object_frame) {
  if (object_frame.binary) {
    frame_binary_buf buf;
    buf.magic[0] = k_binary_code;
    buf.magic[1] = 0;
    buf.short_obn = assert_narrow_cast<ushort>(object_frame.obn);
    to_std(&buf.short_obn);
    for_int(i, 4) {
      for_int(j, 3) {
        buf.f[i][j] = object_frame.frame[i][j];
        to_std(&buf.f[i][j]);
      }
    }
    buf.zoom = object_frame.zoom;
    to_std(&buf.zoom);
    write_binary_raw(os, ArView(buf));
  } else {
    os << create_string(object_frame);
  }
  return bool(os);
}

bool write(WBuffer& b, const ObjectFrame& object_frame) {
  if (object_frame.binary) {
    b.put(char{k_binary_code});
    b.put('\0');
    b.put(assert_narrow_cast<short>(object_frame.obn));
    for_int(i, 4) for_int(j, 3) b.put(object_frame.frame[i][j]);
    b.put(object_frame.zoom);
  } else {
    string s = create_string(object_frame);
    b.put(s.c_str(), narrow_cast<int>(s.size()));
  }
  return true;
}

string create_string(const ObjectFrame& object_frame) {
  // 2014-05-02: changed from %.7g .
  const Frame& f = object_frame.frame;
  return sform("F %d  %.9g %.9g %.9g  %.9g %.9g %.9g  %.9g %.9g %.9g  %.9g %.9g %.9g  %.9g\n",  //
               object_frame.obn, f[0][0], f[0][1], f[0][2], f[1][0], f[1][1], f[1][2], f[2][0], f[2][1], f[2][2],
               f[3][0], f[3][1], f[3][2], object_frame.zoom);
}

Frame parse_frame(const string& s) {
  ObjectFrame object_frame;
  std::istringstream iss(s);
  if (!read(iss, object_frame)) assertnever("Could not parse Frame '" + s + "'");
  return object_frame.frame;
}

bool is_not_a_frame(const Frame& f) { return f[0][0] == k_Frame_fnan; }

Frame get_not_a_frame() {
  Frame f = Frame::identity();
  f[0][0] = k_Frame_fnan;
  return f;
}

}  // namespace FrameIO

}  // namespace hh
