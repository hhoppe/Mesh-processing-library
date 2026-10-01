// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt

#include "libHh/Args.h"
using namespace hh;

namespace {

void echo_args(Args& args) {
  SHOW(args.get_string());
  SHOW(args.get_int());
}

void test_static_checks() {
  for (const char* str : {"0", "1", "true", "false"}) assertx(Args::check_bool(str));
  for (const char* str : {"", "2", "True", "yes", "00"}) assertx(!Args::check_bool(str));
  assertx(Args::parse_bool("true") && !Args::parse_bool("0"));
  assertx(Args::check_char("x") && !Args::check_char("") && !Args::check_char("xy"));
  assertx(Args::parse_char("-") == '-');
  for (const char* str : {"0", "-12", "+7", "2147483647"}) assertx(Args::check_int(str));
  for (const char* str : {"", "1.", "1e3", "12a", "--1", " 1", "0x10"}) assertx(!Args::check_int(str));
  assertx(Args::parse_int("-12") == -12 && Args::parse_int("+7") == 7);
  for (const char* str : {"1", "-1.5", ".5", "1e-3", "+2.5e+2"}) assertx(Args::check_float(str));
  for (const char* str : {"1,5", "1.5f", "inf", "nan", " 1"}) assertx(!Args::check_float(str));
  assertx(Args::parse_float("-1.5") == -1.5f && Args::parse_float("+2.5e+2") == 250.f);
  assertx(Args::check_double("1e-300") && Args::parse_double("1e-300") == 1e-300);
  for (const char* str : {"file", "-", "a/b.c", "cmd args |", "https://a.b/c?d"}) assertx(Args::check_filename(str));
  for (const char* str : {"", "-file", "a*b", "a?b", "a<b", "a>b", "a\"b"}) assertx(!Args::check_filename(str));
  for (const char* str : {"-?", "--help", "--version"}) assertx(ParseArgs::special_arg(str));
  for (const char* str : {"-h", "--", "-help"}) assertx(!ParseArgs::special_arg(str));
}

void test_args_stream() {
  Args args{"1", "x", "-12", "2.5", "1e3", "str", "dir\\file.txt", "-"};
  assertx(args.num() == 8 && args.size() == 8 && args.peek_string() == "1");
  assertx(args.get_bool());
  assertx(args.num() == 7 && args.peek_string() == "x");
  assertx(args.get_char() == 'x');
  assertx(args.get_int() == -12);
  assertx(args.get_float() == 2.5f);
  assertx(args.get_double() == 1e3);
  assertx(args.get_string() == "str");
  assertx(args.get_filename() == "dir/file.txt");  // Backslashes are translated.
  assertx(args.get_filename() == "-");
  assertx(args.num() == 0 && args.size() == 0);
}

int g_num_func0_calls = 0;

void do_func0() { g_num_func0_calls++; }

void do_consume(Args& args) { assertx(args.get_string() == "c1" && args.get_string() == "c2"); }

void test_parse_args() {
  bool flag = false, b = true;
  char ch = 'a';
  int niter = 0;
  float nooutput = 0.f;
  double scale = 1.;
  string str = "default";
  int ivec[3] = {0, 0, 0};
  Vec2<double> dvec{0., 0.};
  ParseArgs args(V<string>("prog", "-flag", "-b", "false", "-ch", "z", "-ni", "7", "-noout", "2.5", "-sc", "3", "-str",
                           "hello world", "-ivec", "1", "-2", "3", "-dvec", "0.5", "1e-1", "-func0", "-consume", "c1",
                           "c2", "file1", "-func0", "file2"));
  HH_ARGSC("", ":Comment line");
  HH_ARGSF(flag, ": set a flag");
  HH_ARGSP(b, "bool : a boolean parameter");
  HH_ARGSP(ch, "c : a character parameter");
  HH_ARGSP(niter, "n : number of iterations");
  HH_ARGSP(nooutput, "f : a float parameter whose name shares the prefix '-n'");
  args.p("-sc[ale]", scale, "s : a parameter with a minimum prefix");
  HH_ARGSP(str, "string : a string parameter");
  HH_ARGSP(ivec, "i1 i2 i3 : three integers");
  HH_ARGSP(dvec, "d1 d2 : two doubles");
  args.p("-func0", do_func0, ": call a function");
  HH_ARGSD(consume, "a b : consume two arguments");
  args.other_args_ok();
  assertx(args.header().contains(" prog -flag -b false -ch z"));
  assertx(args.parse());
  assertx(flag && !b && ch == 'z' && niter == 7 && nooutput == 2.5f && scale == 3. && str == "hello world");
  assertx(ivec[0] == 1 && ivec[1] == -2 && ivec[2] == 3 && dvec == V(.5, .1) && g_num_func0_calls == 2);
  // The unrecognized (non-option) arguments remain available.
  assertx(args.num() == 2 && args.get_filename() == "file1" && args.get_filename() == "file2");
  args.print_help();  // It shows the current values.
}

void test_prefixes() {
  {
    // An ambiguous prefix is reported, and the shortest matching option is assumed.
    bool niter = false, nooutput = false;
    ParseArgs args(V<string>("prog", "-n", "-noo"));
    HH_ARGSF(niter, ": first flag");
    HH_ARGSF(nooutput, ": second flag");
    assertx(args.parse() && niter && nooutput);
  }
  {
    // An exact match takes precedence over a longer option that it prefixes.
    bool ab = false, abc = false;
    ParseArgs args(V<string>("prog", "-ab"));
    HH_ARGSF(ab, ": exact");
    HH_ARGSF(abc, ": longer");
    assertx(args.parse() && ab && !abc);
  }
  {
    // With disallow_prefixes(), only whole option names match.  Unrecognized options are kept, as are "--" and
    // all the arguments after it.
    bool verbose = false;
    ParseArgs args(V<string>("prog", "-verb", "file", "-verbose", "--", "-verbose"));
    args.disallow_prefixes();
    args.other_options_ok();
    args.other_args_ok();
    HH_ARGSF(verbose, ": be verbose");
    Array<string> unrecognized;
    assertx(args.parse_and_extract(unrecognized));
    assertx(verbose);
    SHOW(unrecognized);
  }
}

void phase0() {
  echo_args(Args{"string", "3"}.use());
  test_static_checks();
  test_args_stream();
  test_parse_args();
  test_prefixes();
}

void do_show1p1() { SHOW(1 + 1); }

bool flag2 = false;
Vec2<int> vec2 = {0, 0};

void phase1(int argc, const char** argv) {
  SHOW(CArrayView(argv, argc));
  ParseArgs args(argc, argv);
  bool flag = false, flap = false, flac = false;
  int val1 = 0, val2 = 0;
  float fa[2] = {0.f, 0.f};  // Test C-array.
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
  const bool optsparse = args.parse_and_extract(ar_unrecog);
  SHOW(optsparse, ar_unrecog);
  SHOW(flag, flap, flac, val1, val2);
  SHOW(fa[0], fa[1]);
  SHOW(fb[0], fb[1]);
  SHOW(fc[0], fc[1]);
}

void phase2(int argc, const char** argv) {
  const auto do_showar = [](Args& args) {
    const int i = args.get_int();
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
