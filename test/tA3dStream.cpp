// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "BufferedA3dStream.h"

#include "RangeOp.h"            // is_zero()
using namespace hh;

int main() {
    const bool debug = getenv_bool("DEBUG");
    const bool is_bi = getenv_bool("BI");
    const bool is_bo = getenv_bool("BO");
    unique_ptr<RBuffer> pbi;
    unique_ptr<RA3dStream> pia3d;
    if (is_bi) {
        pbi = make_unique<RBuffer>(0);
        pia3d = make_unique<RBufferedA3dStream>(*pbi);
    } else {
        pia3d = make_unique<RSA3dStream>(std::cin);
    }
    unique_ptr<WBuffer> pbo;
    unique_ptr<WA3dStream> poa3d;
    if (is_bo) {
        pbo = make_unique<WBuffer>(1);
        poa3d = make_unique<WBufferedA3dStream>(*pbo);
    } else {
        poa3d = make_unique<WSA3dStream>(std::cout);
    }
    RA3dStream& ia3d = *pia3d;
    WA3dStream& oa3d = *poa3d;
    A3dElem el;
    for (;;) {
        if (is_bi) {
            RBufferedA3dStream::ERecognize st = down_cast<RBufferedA3dStream*>(pia3d.get())->recognize();
            assertx(st!=RBufferedA3dStream::ERecognize::parse_error);
            if (st!=RBufferedA3dStream::ERecognize::yes) {
                RBuffer::ERefill ret = pbi->refill();
                if (pbi->eof()) break;
                assertx(ret!=RBuffer::ERefill::other);
                if (0 && ret==RBuffer::ERefill::no) break;
                continue;
            }
        }
        ia3d.read(el);
        if (debug) std::cerr << el;
        if (el.type()==A3dElem::EType::endfile) break;
        oa3d.write(el);
    }
}
