// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BUFFER_H_
#define MESH_PROCESSING_LIBHH_BUFFER_H_

#include "libHh/Array.h"
#include "libHh/NetworkOrder.h"

namespace hh {

// A streaming buffer open on a file descriptor.  It supports non-blocking reads on file descriptor 0 (stdin).
// It lets one peek into the buffer (not just the first character) to identify if a full data record is available.
// Such a record can be a line of text or a binary structure.
// After extracting data from the buffer, it shifts the buffer contents for proper alignment.

class Buffer : noncopyable {
 public:
  bool eof() const;  // end of file
  bool err() const;  // error in system call
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
  ERefill refill();
  void extract(int n);  // have read n bytes
  int num() const;
  size_t size() const { return num(); }
  char operator[](int bi) const;
  bool has_line() const;
  // next dies if len not sufficient, includes '\n', ret success
  [[nodiscard]] bool extract_line(string& str);
  char get_char(int bi) const;  // same as operator[]
  int get_int(int bi) const;
  short get_short(int bi) const;
  float get_float(int bi) const;
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

inline bool Buffer::eof() const { return _eof; }
inline bool Buffer::err() const { return _err; }
inline int RBuffer::num() const { return _n; }

inline char RBuffer::operator[](int bi) const {
  ASSERTX(bi >= 0 && bi < _n);
  return _ar[_beg + bi];
}

inline char RBuffer::get_char(int bi) const { return (*this)[bi]; }

inline int RBuffer::get_int(int bi) const {
  ASSERTX(bi >= 0 && bi + 4 <= _n);
  const char* p = &_ar[_beg + bi];
  union {
    int32_t i;
    char s[4];
  } u;
  for (int c = 0; c < 4; c++) u.s[c] = p[c];  // "for (const int c : range(4))" confuses clang-tidy.
  from_std(&u.i);
  return u.i;
}

inline short RBuffer::get_short(int bi) const {
  ASSERTX(bi >= 0 && bi + 2 <= _n);
  const char* p = &_ar[_beg + bi];
  union {
    int16_t i;
    char s[2];
  } u;
  for (int c = 0; c < 2; c++) u.s[c] = p[c];  // "for (const int c : range(2))" confuses clang-tidy.
  from_std(&u.i);
  return u.i;
}

inline float RBuffer::get_float(int bi) const {
  ASSERTX(bi >= 0 && bi + 4 <= _n);
  const char* p = &_ar[_beg + bi];
  union {
    float f;
    char s[4];
  } u;
  for (int c = 0; c < 4; c++) u.s[c] = p[c];  // "for (const int c : range(4))" confuses clang-tidy.
  from_std(&u.f);
  return u.f;
}

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BUFFER_H_
