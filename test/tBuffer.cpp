// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Buffer.h"
#include "Args.h"
using namespace hh;

// run with:
//      tBuffer -out 1 -bsize 23 | tBuffer -out 0

namespace {

int bsize = 100;

void perform_out() {
    WBuffer wb(1);
    for_int(i, bsize) {
        wb.put("a", 1);
        wb.put("b", 1);
        wb.flush();
        if (i%2==0) my_sleep(0.03); // was 0.1
        if (i%3==0) my_sleep(0.06); // was 0.2
    }
    wb.flush();
}

void perform_in() {
    int i = 0;
    RBuffer rb(0);
    if (0) assertw(set_fd_no_delay(0, true));
    for (;;) {
        if (rb.num()) {
            showf("Found char '%c'\n", rb.get_char(0));
            i++;
            rb.extract(1);
            continue;
        }
        std::cerr << "RBuffer empty, try fill:";
        auto ret = rb.refill();
        std::cerr << "now contains " << rb.num() << " bytes\n";
        if (ret==RBuffer::ERefill::no) { SHOW("blocked"); my_sleep(0.14); continue; }
        if (ret==RBuffer::ERefill::other && rb.eof()) { SHOW("goteof"); break; }
        if (ret==RBuffer::ERefill::other) assertnever("read");
    }
    SHOW(i);
}

} // namespace

int main(int argc, const char** argv) {
    ParseArgs args(argc, argv);
    bool out = false;
    ARGSP(out,                  "bool : 0=in, 1=out");
    ARGSP(bsize,                "bytes : size of transmission");
    args.parse();
    if (out) perform_out(); else perform_in();
    return 0;
}
