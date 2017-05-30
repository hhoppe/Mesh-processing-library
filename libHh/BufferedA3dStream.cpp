// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "BufferedA3dStream.h"

#include <cstdio>               // sscanf()

namespace hh {

// *** RBufferedA3dStream

bool RBufferedA3dStream::read_line(bool& binary, char& ctype, Vec3<float>& f, string& comment) {
    // read first character.  Read past optional newlines
    {
        int i = 0;
        for (; ; i++) {
            assertx(i<_buf.num());
            if (_buf[i]!='\n') break;
        }
        if (i) _buf.extract(i);
    }
    binary = _buf[0]==k_a3d_binary_code;
    if (binary) {
        assertx(_buf.num()>=16);
        ctype = narrow_cast<char>(_buf.get_short(2));
        for_int(i, 3) { f[i] = _buf.get_float(i*4+4); }
        _buf.extract(16);
        return true;
    }
    ctype = _buf[0];
    string str;
    assertx(_buf.extract_line(str));
    if (ctype==char(A3dElem::EType::comment)) {
        comment = str.substr(1);
        return true;
    }
    assertx(str.size()>1 && str[1]==' ');
    assertx(sscanf(str.c_str()+2, "%g %g %g", &f[0], &f[1], &f[2])==3);
    return true;
}

RBufferedA3dStream::ERecognize RBufferedA3dStream::recognize() const {
    // Skip leading newlines
    int i;
    for (i = 0; i<_buf.num(); i++)
        if (_buf[i]!='\n') break;
    if (i==_buf.num()) return ERecognize::no;
    {
        char ch = _buf[i];
        bool have_space = i+1<_buf.num() && _buf[i+1]==' ';
        A3dElem::EType ct = A3dElem::EType(ch);
        if (!(ct==A3dElem::EType::comment || ch==k_a3d_binary_code ||
              (have_space && (A3dElem::command_type(ct) || ct==A3dElem::EType::point ||
                             ct==A3dElem::EType::polygon || ct==A3dElem::EType::polyline ||
                             A3dElem::status_type(ct) || ch=='n')))) return ERecognize::no;
    }
    for (;;) {
        if (i>=_buf.num()) return ERecognize::partial;
        char ch = _buf[i];
        if (ch=='\n') { i++; continue; }
        if (ch==k_a3d_binary_code) { // binary record
            if (i+16>_buf.num()) return ERecognize::partial;
            ch = narrow_cast<char>(_buf.get_short(i+2));
            i += 16;
        } else {
            for (i++; ; ) {
                if (i>=_buf.num()) return ERecognize::partial;
                if (_buf[i++]=='\n') break;
            }
        }
        A3dElem::EType ct = A3dElem::EType(ch);
        if (ct==A3dElem::EType::comment || A3dElem::command_type(ct) || ct==A3dElem::EType::point || ch=='E')
            return ERecognize::yes;
        if (!(ct==A3dElem::EType::polygon || ct==A3dElem::EType::polyline || A3dElem::status_type(ct) ||
              ch=='n' || ch=='v'))
            return ERecognize::parse_error;
    }
}

// *** WBufferedA3dStream

void WBufferedA3dStream::output(bool binary, char ctype, const Vec3<float>& f) {
    if (binary) {
        _buf.put(char{k_a3d_binary_code});
        _buf.put('\0');
        _buf.put(short{ctype});
        _buf.put(f[0]); _buf.put(f[1]); _buf.put(f[2]);
    } else {
        ssform(_stmp, "%c %g %g %g\n", ctype, f[0], f[1], f[2]);
        _buf.put(_stmp.c_str(), narrow_cast<int>(_stmp.size()));
    }
}

void WBufferedA3dStream::output_comment(const string& s) {
    _buf.put("#", 1);
    _buf.put(s.c_str(), narrow_cast<int>(s.size()));
    _buf.put("\n", 1);
}

void WBufferedA3dStream::blank_line() {
    _buf.put('\n');
}

} // namespace hh
