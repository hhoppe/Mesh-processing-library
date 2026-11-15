// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/LineWriter.h"

#include <sstream>  // ostringstream
using namespace hh;

int main() {
  {
    LineWriter w;
    w << "Vertex " << 17 << ' ';
    for (const float f : {0.1f, -2.5f, 0.f}) w << ' ' << f;
    w << " {" << std::string_view("normal=(0 0 1)") << '}';
    w.write_line(std::cout);
  }
  {
    LineWriter w;
    w << "ints:";
    w << ' ' << -7 << ' ' << uint8_t{200} << ' ' << int64_t{-1'234'567'890'123} << ' ' << size_t{0};
    w << ' ' << std::numeric_limits<int>::min() << ' ' << std::numeric_limits<uint64_t>::max();
    w.write_line(std::cout);
  }
  {
    LineWriter w;
    w << "floats:";
    for (const float f : {1e-5f, 0.0001f, 0.18f, 1.f / 3.f, 1000.123f, 0.123456789f, 100000.f, 1234567.f, -3e30f})
      w << ' ' << f;
    w.write_line(std::cout);
  }
  {
    LineWriter w;
    w << "abc" << 1;
    SHOW(w.view());
    w.clear();
    SHOW(w.view() == "");
    w << 'x';
    std::ostringstream oss;
    w.write_line(oss);
    w.write_line(oss);  // The buffer was cleared, so this writes only "\n".
    SHOW(oss.str() == "x\n\n");
  }
}

template LineWriter& LineWriter::operator<<(int);
template LineWriter& LineWriter::operator<<(float);
