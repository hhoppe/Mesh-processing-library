// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_BUFFEREDA3DSTREAM_H_
#define MESH_PROCESSING_LIBHH_BUFFEREDA3DSTREAM_H_

#include "libHh/A3dStream.h"
#include "libHh/Buffer.h"

namespace hh {

// Read/write an A3dStream using a RBuffer/WBuffer.

class RBufferedA3dStream : public RA3dStream {  // Read from RBuffer
 public:
  explicit RBufferedA3dStream(RBuffer& b) : _buf(b) {}
  enum class ERecognize { parse_error, no, partial, yes };
  ERecognize recognize() const;

 private:
  RBuffer& _buf;
  bool read_line(bool& binary, char& ctype, Vec3<float>& f, string& comment) override;
};

class WBufferedA3dStream : public WA3dStream {  // Write to WBuffer
 public:
  explicit WBufferedA3dStream(WBuffer& b) : _buf(b) {}
  ~WBufferedA3dStream() override { flush(); }
  void flush() override { _buf.flush(); }

 private:
  WBuffer& _buf;
  string _stmp;
  void output(bool binary, char ctype, const Vec3<float>& f) override;
  void output_comment(const string& s) override;
  void blank_line() override;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_BUFFEREDA3DSTREAM_H_
