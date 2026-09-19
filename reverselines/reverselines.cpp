// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt

#include "libHh/Args.h"
#include "libHh/FileIO.h"  // ReversedLinesReader
using namespace hh;

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSC("", "filename : output the lines in file in reverse order");
  args.other_args_ok();
  args.parse();
  const string filename = args.get_filename();
  if (args.num()) args.problem("expect a single argument");
  ReversedLinesReader reader(filename);
  for (string line; reader.getline(line);) std::cout << line << '\n';
  std::cout.flush();
  assertx(std::cout);
  return 0;
}
