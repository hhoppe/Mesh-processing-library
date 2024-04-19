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
  ushort shortobn;
  Vec4<Vec3<float>> f;
  float zoom;
};

void decode(std::istream& is, Frame& f, int& obn, float& zoom) {
  if (1) {  // to avoid bug_istream_float.cpp in VC12
    string sline;
    assertx(my_getline(is, sline));
    assertx(sline != "");
    assertx(sline[0] == 'F' && sline[1] == ' ');
    assertx(sscanf(sline.c_str() + 1, "%d  %f %f %f  %f %f %f  %f %f %f  %f %f %f  %f",  // + 1 to skip ch
                   &obn, &f[0][0], &f[0][1], &f[0][2], &f[1][0], &f[1][1], &f[1][2], &f[2][0], &f[2][1], &f[2][2],
                   &f[3][0], &f[3][1], &f[3][2], &zoom) == 14);
  } else {
    char ch = '\0';
    is.get(ch);  // is >> ch would eat up white space
    assertx(is);
    assertx(ch == 'F');
    // if (ch != 'F') is.clear(std::ios::badbit);
    assertx(is >> obn);
    // f = Frame::identity();
    for_int(i, 4) for_int(j, 3) assertx(is >> f[i][j]);
    assertx(is >> zoom);
  }
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

bool read(std::istream& is, Frame& f, int& obn, float& zoom, bool& bin) {
  int c = is.peek();
  if (c < 0) return false;
  bin = c == k_binary_code;
  if (bin) {  // binary
    frame_binary_buf buf;
    if (!read_binary_raw(is, ArView(buf))) return false;
    assertx(buf.magic[1] == 0);
    from_std(&buf.shortobn);
    obn = buf.shortobn;
    for_int(i, 4) {
      for_int(j, 3) {
        from_std(&buf.f[i][j]);
        f[i][j] = buf.f[i][j];
      }
    }
    from_std(&buf.zoom);
    zoom = buf.zoom;
    return true;
  } else {
    decode(is, f, obn, zoom);
    bool good = !!is;
    if (0) {
      // The newline is now read as part of decode() using my_getline().
      is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // read past newline
    }
    return good;
  }
}

bool read(RBuffer& b, Frame& f, int& obn, float& zoom, bool& bin) {
  if (!b.num()) return false;
  bin = b[0] == k_binary_code;
  if (bin) {  // binary
    if (b.num() < 14 * 4) return false;
    obn = b.get_short(2);
    for_int(i, 4) for_int(j, 3) f[i][j] = b.get_float((1 + (i * 3) + j) * 4);
    zoom = b.get_float(13 * 4);
    b.extract(14 * 4);
    return true;
  } else {
    if (b[0] != 'F') return false;
    string str;
    if (!b.extract_line(str)) return false;
    std::istringstream iss(str);
    decode(iss, f, obn, zoom);
    return !!iss;
  }
}

bool write(std::ostream& os, const Frame& f, int obn, float zoom, bool bin) {
  if (bin) {
    frame_binary_buf buf;
    buf.magic[0] = k_binary_code;
    buf.magic[1] = 0;
    assertw(obn >= -32768 && obn < 32768);
    buf.shortobn = narrow_cast<ushort>(obn);
    to_std(&buf.shortobn);
    for_int(i, 4) {
      for_int(j, 3) {
        buf.f[i][j] = f[i][j];
        to_std(&buf.f[i][j]);
      }
    }
    buf.zoom = zoom;
    to_std(&buf.zoom);
    write_binary_raw(os, ArView(buf));
  } else {
    os << create_string(f, obn, zoom);
  }
  return !!os;
}

bool write(WBuffer& b, const Frame& f, int obn, float zoom, bool bin) {
  if (bin) {
    b.put(char{k_binary_code});
    b.put('\0');
    b.put(narrow_cast<short>(obn));
    for_int(i, 4) for_int(j, 3) b.put(f[i][j]);
    b.put(zoom);
  } else {
    string s = create_string(f, obn, zoom);
    b.put(s.c_str(), narrow_cast<int>(s.size()));
  }
  return true;
}

string create_string(const Frame& f, int obn, float zoom) {
  // Maybe should use %.9g for exact representation.  Yes, changed from %.7g on 20140502.
  return sform("F %d  %.9g %.9g %.9g  %.9g %.9g %.9g  %.9g %.9g %.9g  %.9g %.9g %.9g  %.9g\n",  //
               obn, f[0][0], f[0][1], f[0][2], f[1][0], f[1][1], f[1][2], f[2][0], f[2][1], f[2][2], f[3][0], f[3][1],
               f[3][2], zoom);
}

Frame parse_frame(const string& s) {
  Frame frame;
  int obn;
  float zoom;
  bool bin;
  {
    std::istringstream iss(s);
    if (!read(iss, frame, obn, zoom, bin)) assertnever("Could not parse Frame '" + s + "'");
  }
  return frame;
}

bool is_not_a_frame(const Frame& f) { return f[0][0] == k_Frame_fnan; }

void create_not_a_frame(Frame& f) {
  f = Frame::identity();
  f[0][0] = k_Frame_fnan;
}

}  // namespace FrameIO

}  // namespace hh
