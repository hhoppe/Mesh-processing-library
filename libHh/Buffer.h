// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BUFFER_H_
#define MESH_PROCESSING_LIBHH_BUFFER_H_

#include <cstring>  // memcpy().

#include "libHh/Array.h"
#include "libHh/NetworkOrder.h"

namespace hh {

// A streaming buffer open on a file descriptor.  It supports non-blocking reads on file descriptor 0 (stdin).
// It lets one peek into the buffer (not just the first character) to identify if a full data record is available.
// Such a record can be a line of text or a binary structure.
// After extracting data from the buffer, it shifts the buffer contents for proper alignment.

class Buffer : noncopyable {
 public:
  [[nodiscard]] bool eof() const { return _eof; }  // end of file
  [[nodiscard]] bool err() const { return _err; }  // error in system call
 protected:
  explicit Buffer(int fd) : _fd(fd) { assertx(_fd >= 0); }
  int _fd;  // file descriptor associated
  Array<char> _ar;
  int _beg{0};  // index of first element in _ar[]
  int _n{0};    // num of elements in buffer (_beg + _n <= _ar.num())
  bool _eof{false};
  bool _err{false};

  void shift();   // shift data to beginning of buffer
  void expand();  // increase buffer size
};

class RBuffer : public Buffer {
 public:
  explicit RBuffer(int fd);
  enum class ERefill { no, yes, other };
  [[nodiscard]] ERefill refill();
  void extract(int n);  // have read n bytes
  [[nodiscard]] int num() const { return _n; }
  [[nodiscard]] size_t size() const { return num(); }
  [[nodiscard]] char operator[](int bi) const;
  [[nodiscard]] bool has_line() const;
  // next dies if len not sufficient, includes '\n', ret success
  [[nodiscard]] bool extract_line(string& str);
  [[nodiscard]] char get_char(int bi) const;  // same as operator[]
  [[nodiscard]] int get_int(int bi) const;
  [[nodiscard]] short get_short(int bi) const;
  [[nodiscard]] float get_float(int bi) const;
  void wait_for_input();
};

class WBuffer : public Buffer {
 public:
  explicit WBuffer(int fd) : Buffer(fd) {}
  ~WBuffer() { assertw(!_n); }
  enum class EFlush { part, all, other };
  EFlush flush(int nb = 0);  // (nb == 0 is all)
  void put(const void* buf, int nbytes);
  void put(char ch);
  void put(short i);
  void put(int i);
  void put(float f);
};

//----------------------------------------------------------------------------

inline char RBuffer::operator[](int bi) const {
  ASSERTX(bi >= 0 && bi < _n);
  return _ar[_beg + bi];
}

inline char RBuffer::get_char(int bi) const { return (*this)[bi]; }

inline int RBuffer::get_int(int bi) const {
  ASSERTX(bi >= 0 && bi + 4 <= _n);
  int32_t i;
  std::memcpy(&i, _ar.data() + _beg + bi, sizeof(i));
  from_std(&i);
  return i;
}

inline short RBuffer::get_short(int bi) const {
  ASSERTX(bi >= 0 && bi + 2 <= _n);
  int16_t i;
  std::memcpy(&i, _ar.data() + _beg + bi, sizeof(i));
  from_std(&i);
  return i;
}

inline float RBuffer::get_float(int bi) const {
  ASSERTX(bi >= 0 && bi + 4 <= _n);
  float f;
  std::memcpy(&f, _ar.data() + _beg + bi, sizeof(f));
  from_std(&f);
  return f;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BUFFER_H_
