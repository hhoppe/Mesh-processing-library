// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt

#include "libHh/Args.h"
using namespace hh;

namespace {

void echo_args(Args& args) {
  SHOW(args.get_string());
  SHOW(args.get_int());
}

void phase0() { echo_args(as_lvalue(Args{"string", "3"})); }

void do_show1p1() { SHOW(1 + 1); }

bool flag2 = false;
Vec2<int> vec2 = {0, 0};

void phase1(int argc, const char** argv) {
  SHOW(CArrayView(argv, argc));
  ParseArgs args(argc, argv);
  bool flag = false, flap = false, flac = false;
  int val1 = 0, val2 = 0;
  float fa[2] = {0.f, 0.f};  // test C-array
  Vec2<float> fb{0.f, 0.f};
  Vec2<float> fc{0.f, 0.f};
  HH_ARGSF(flag, ": enable flag");
  HH_ARGSF(flap, ": turn on the flaps");
  args.f("-flac", flac, ": send out flacs");
  args.p("-val1", val1, "f : set value1 coefficient");
  HH_ARGSP(val2, "i : comment");
  args.p("-farr", fa, 2, "f1 f2 : sets two element array");
  HH_ARGSC("", ":");
  HH_ARGSP(fb, "a b : set variables");
  HH_ARGSP(fc, "c1 c2 : set the fc variables");
  HH_ARGSD(show1p1, ": comment");
  args.other_args_ok();
  args.other_options_ok();
  Array<string> ar_unrecog;
  bool optsparse = args.parse_and_extract(ar_unrecog);
  SHOW(optsparse, ar_unrecog);
  SHOW(flag, flap, flac, val1, val2);
  SHOW(fa[0], fa[1]);
  SHOW(fb[0], fb[1]);
  SHOW(fc[0], fc[1]);
}

void phase2(int argc, const char** argv) {
  const auto do_showar = [](Args& args) {
    int i = args.get_int();
    SHOW("showar", i, vec2[i]);
  };
  const auto do_vlp = [](Args& args) { SHOW("reading vlp", args.get_filename(), vec2); };
  const auto do_file = [](Args& args) { SHOW("reading file", args.get_filename(), vec2); };
  const auto do_string = [](Args& args) { SHOW("string", args.get_string(), vec2); };
  SHOW(CArrayView(argv, argc));
  ParseArgs args(argc, argv);
  HH_ARGSF(flag2, ": enable flag");
  HH_ARGSP(vec2, "i1 i2 : set two coefficients");
  HH_ARGSD(showar, "i : show coefficient indexed i");
  args.p("*.vlp", do_vlp, "file.vlp : read the file");
  args.p("*", do_file, "file : read any other file type");
  args.p("*.string", do_string, "string : print string");
  bool optsparse = args.parse();
  SHOW(optsparse);
  SHOW(flag2, vec2);
}

}  // namespace

int main(int argc, const char** argv) {
  switch (getenv_int("TARGS_PHASE", 0)) {
    case 0: phase0(); break;
    case 1: phase1(argc, argv); break;
    case 2: phase2(argc, argv); break;
    default: assertnever("");
  }
  return 0;
}
