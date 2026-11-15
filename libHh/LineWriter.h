// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_LINEWRITER_H_
#define MESH_PROCESSING_LIBHH_LINEWRITER_H_

#include <charconv>  // to_chars(), chars_format
#include <cstring>   // memcpy()

#include "libHh/Hh.h"

namespace hh {

// Builds a line of text in a fixed-size buffer, for the fast output of data files.  Integers and floats are
// written using std::to_chars().  A float is written with the fewest significant digits that read back as the same
// float, in the notation of printf("%g"); e.g., 0.1f is written as "0.1" rather than "0.100000001".  A double is
// rejected at compile time, because its shortest form would have up to 17 digits.
//   LineWriter w;
//   w << "Vertex " << vertex_id(v) << ' ';
//   for_int(c, 3) w << ' ' << point(v)[c];
//   w.write_line(std::cout);
class LineWriter {
 public:
  LineWriter() = default;
  LineWriter(const LineWriter&) = delete;  // Because _s points into _buf.
  LineWriter& operator=(const LineWriter&) = delete;
  LineWriter& operator<<(char ch) {
    assertx(_s < _buf + k_capacity);
    *_s++ = ch;
    return *this;
  }
  // (Passing sv by value makes MSVC warn C4866 that chained calls may not evaluate their operands left to right.)
  LineWriter& operator<<(const std::string_view& sv) {
    assertx(sv.size() <= size_t(_buf + k_capacity - _s));
    std::memcpy(_s, sv.data(), sv.size());
    _s += sv.size();
    return *this;
  }
  template <typename T> requires((std::integral<T> && !std::same_as<T, bool>) || std::same_as<T, float>)
  LineWriter& operator<<(T value) {
    std::to_chars_result result;
    if constexpr (std::same_as<T, float>) {
      result = std::to_chars(_s, _buf + k_capacity, value, std::chars_format::general);
    } else {
      result = std::to_chars(_s, _buf + k_capacity, value);
    }
    assertx(result.ec == std::errc{});
    _s = result.ptr;
    return *this;
  }
  [[nodiscard]] std::string_view view() const { return std::string_view(_buf, _s); }
  void clear() { _s = _buf; }
  // Appends '\n', writes the line to os, and clears the buffer.
  void write_line(std::ostream& os) {
    *this << '\n';
    assertx(os.write(_buf, _s - _buf));
    _s = _buf;
  }

 private:
  static constexpr int k_capacity = 1000;
  char _buf[k_capacity];
  char* _s{_buf};  // A pointer rather than an index into _buf, for which g++ generates slower code.
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_LINEWRITER_H_
