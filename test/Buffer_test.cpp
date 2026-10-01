// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Buffer.h"

#include "libHh/Args.h"
using namespace hh;

// Run with:
//   Buffer_test -out 1 -bsize 23 | Buffer_test -out 0
//   Buffer_test -records -out 1 | Buffer_test -records -out 0

namespace {

int bsize = 100;

// Each record is a text line followed by a char, a short, an int, and a float, in network order.
constexpr int k_num_records = 500;  // Enough bytes to expand both buffers and to flush within WBuffer::put().
constexpr int k_record_binary_size = 1 + 2 + 4 + 4;

string record_line(int i) { return "record " + std::to_string(i); }
char record_char(int i) { return char('A' + i % 26); }
short record_short(int i) { return short(-i); }
int record_int(int i) { return i * 100'003 - 7; }
float record_float(int i) { return float(i) * .25f - 3.f; }

void perform_out() {
  WBuffer wb(1);
  for_int(i, bsize) {
    wb.put("a", 1);
    wb.put("b", 1);
    wb.flush();
    if (i % 2 == 0) my_precise_sleep(0.03);
    if (i % 3 == 0) my_precise_sleep(0.06);
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
    if (ret == RBuffer::ERefill::no) {
      SHOW("blocked");
      my_precise_sleep(0.14);
      continue;
    }
    if (ret == RBuffer::ERefill::other && rb.eof()) {
      SHOW("goteof");
      break;
    }
    if (ret == RBuffer::ERefill::other) assertnever("read");
  }
  SHOW(i);
}

void perform_out_records() {
  WBuffer wb(1);
  for_int(i, k_num_records) {
    const string line = record_line(i) + "\n";
    wb.put(line.data(), narrow_cast<int>(line.size()));
    wb.put(record_char(i));
    wb.put(record_short(i));
    wb.put(record_int(i));
    wb.put(record_float(i));
  }
  assertx(wb.flush() == WBuffer::EFlush::all);
}

void perform_in_records() {
  RBuffer rb(0);
  int nrecords = 0;
  string line;
  bool have_line = false;  // The text line of record nrecords has been extracted.
  for (;;) {
    if (!have_line && rb.has_line()) {
      assertx(rb.extract_line(line));  // It omits the '\n'.
      assertx(line == record_line(nrecords));
      have_line = true;
      continue;
    }
    if (have_line && rb.num() >= k_record_binary_size) {
      const int i = nrecords;
      assertx(rb[0] == record_char(i) && rb.get_char(0) == record_char(i));
      assertx(rb.get_short(1) == record_short(i));
      assertx(rb.get_int(3) == record_int(i));
      assertx(rb.get_float(7) == record_float(i));
      if (i < 3 || i == k_num_records - 1)
        showf("%s: %c %d %d %g\n", line.c_str(), rb.get_char(0), rb.get_short(1), rb.get_int(3), rb.get_float(7));
      rb.extract(k_record_binary_size);
      have_line = false;
      nrecords++;
      continue;
    }
    const RBuffer::ERefill ret = rb.refill();
    if (ret == RBuffer::ERefill::no) {
      rb.wait_for_input();
      continue;
    }
    if (ret == RBuffer::ERefill::other) {
      assertx(rb.eof() && !rb.err());
      break;
    }
  }
  assertx(!have_line && rb.num() == 0);  // No partial record remains.
  SHOW(nrecords);
  assertx(nrecords == k_num_records);
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  bool out = false;
  HH_ARGSP(out, "bool : 0=in, 1=out");
  HH_ARGSP(bsize, "bytes : size of transmission");
  bool records = false;
  HH_ARGSF(records, ": instead transmit records of text lines and binary values");
  args.parse();
  if (records)
    out ? perform_out_records() : perform_in_records();
  else if (out)
    perform_out();
  else
    perform_in();
  return 0;
}
