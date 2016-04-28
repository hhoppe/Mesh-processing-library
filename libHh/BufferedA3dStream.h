// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "A3dStream.h"
#include "Buffer.h"

namespace hh {

// Read/write an A3dStream using a Buffer.

class RBufferedA3dStream : public RA3dStream { // Read from Buffer
 public:
    explicit RBufferedA3dStream(RBuffer& b)     : _buf(b) { }
    enum class ERecognize { parse_error, no, partial, yes };
    ERecognize recognize() const;
 protected:
    bool read_line(bool& binary, char& ctype, Vec3<float>& f, string& comment) override;
 private:
    RBuffer& _buf;
};

class WBufferedA3dStream : public WA3dStream { // Write to Buffer
 public:
    explicit WBufferedA3dStream(WBuffer& b)     : _buf(b) { }
    ~WBufferedA3dStream()                       { flush(); }
    void flush() override                       { _buf.flush(); }
 protected:
    void output(bool binary, char ctype, const Vec3<float>& f) override;
    void output_comment(const string& s) override;
    void blank_line() override;
 private:
    WBuffer& _buf;
    string _stmp;
};

} // namespace hh
