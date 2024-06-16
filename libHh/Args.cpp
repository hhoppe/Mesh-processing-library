// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Args.h"

#include <cctype>  // isdigit()

#include "libHh/FileIO.h"   // is_pipe(), is_url()
#include "libHh/RangeOp.h"  // contains()
#include "libHh/StringOp.h"

namespace hh {

namespace {

inline bool convert_bool(const string& s) { return s == "1" || s == "true"; }
inline char convert_char(const string& s) { return s[0]; }
inline string show_bool(bool b) { return b ? "true" : "false"; }

inline string show_float(float f) {
  string s = make_string(f);  // Nicely Produces "0" rather than the "0.000000" of std::to_string(f).
  if (s.find_first_of(".e") == string::npos) s += ".";
  return s;
}

inline string show_double(double f) {
  string s = make_string(f);
  if (s.find_first_of(".e") == string::npos) s += ".";
  return s;
}

}  // namespace

bool Args::check_bool(const string& s) { return s == "0" || s == "1" || s == "true" || s == "false"; }

bool Args::check_char(const string& s) { return s.size() == 1; }

bool Args::check_int(const string& s) {
  if (s.empty()) return false;
  for_int(i, narrow_cast<int>(s.size())) {
    char ch = s[i];
    if (i == 0 && (ch == '-' || ch == '+')) continue;
    if (std::isdigit(ch)) continue;
    return false;
  }
  return true;
}

bool Args::check_float(const string& s) {
  if (s.find_first_not_of("0123456789-+.e") != string::npos) return false;
  return true;
}

bool Args::check_double(const string& s) { return check_float(s); }

bool Args::check_filename(const string& s) {
  if (s.empty()) return false;
  if (s[0] == '-' && s != "-") return false;
  if (is_pipe(s) || is_url(s)) return true;
  if (s.find_first_of("*?\"<>|") != string::npos) return false;
  return true;
}

bool Args::get_bool() {
  const string& s = get_string();
  if (!check_bool(s)) problem("invalid boolean");
  return convert_bool(s);
}

char Args::get_char() {
  const string& s = get_string();
  if (!check_char(s)) problem("invalid char");
  return convert_char(s);
}

int Args::get_int() {
  const string& s = get_string();
  if (!check_int(s)) problem("invalid int");
  return to_int(s);
}

float Args::get_float() {
  const string& s = get_string();
  if (!check_float(s)) problem("invalid float");
  return to_float(s);
}

double Args::get_double() {
  const string& s = get_string();
  if (!check_double(s)) problem("invalid double");
  return to_double(s);
}

const string& Args::get_string() {
  if (!num()) problem("missing argument(s)");
  return shift_args();
}

string Args::get_filename() {
  const string& s = get_string();
  if (!check_filename(s)) problem("invalid filename");
  return get_canonical_path(s);
}

bool Args::parse_bool(const string& s) {
  if (!check_bool(s)) assertnever("Argument '" + s + "' is not a boolean (0, 1, false, true)");
  return convert_bool(s);
}

char Args::parse_char(const string& s) {
  if (!check_char(s)) assertnever("Argument '" + s + "' is not a single character");
  return convert_char(s);
}

int Args::parse_int(const string& s) {
  if (!check_int(s)) assertnever("Argument '" + s + "' is not an integer");
  return to_int(s);
}

float Args::parse_float(const string& s) {
  if (!check_float(s)) assertnever("Argument '" + s + "' is not a single-precision floating-point");
  return to_float(s);
}

double Args::parse_double(const string& s) {
  if (!check_double(s)) assertnever("Argument '" + s + "' is not a double-precision floating-point");
  return to_double(s);
}

void Args::problem(const string& s) {
  string mes = "Args error : " + s;
  if (_iarg) mes += " at args[" + sform("%d", _iarg - 1) + "]=='" + _args[_iarg - 1] + "'";
  assertnever(mes);
}

//----------------------------------------------------------------------------

ParseArgs::ParseArgs(int& argc, const char**& argv) : _name("") {
  assertx(argc > 0);
  ensure_utf8_encoding(argc, argv);
  _argv0 = assertx(argv[0]);
  _args.init(argc - 1);
  for_int(i, argc - 1) _args[i] = assertx(argv[1 + i]);
  common_construction();
  // We have taken ownership.
  argc = 0;
  argv = nullptr;
}

ParseArgs::ParseArgs(CArrayView<string> aargs, string name) : _name(std::move(name)) {
  assertx(aargs.num() > 0);
  _argv0 = aargs[0];
  _args = aargs.slice(1, aargs.num());
  common_construction();
}

void ParseArgs::common_construction() {
  _argv0 = get_canonical_path(_argv0);
  if (_argv0 != "") {
    iadd(option{"-?", 0, &ParseArgs::fquestion, nullptr, ": print available options (also --help, --version)"});
    iadd(option{"--help", 0, &ParseArgs::fquestion, nullptr, ": (<unlisted>) show help"});
    iadd(option{"--version", 0, &ParseArgs::fversion, nullptr, ": (<unlisted>) show version"});
  }
}

void ParseArgs::f(string str, bool& arg, string doc) {
  if (arg) assertnever("ParseArgs: flag '" + str + "' should not already be set");
  iadd(option{std::move(str), 0, &ParseArgs::fbool, &arg, std::move(doc)});
}

void ParseArgs::p(string str, bool& arg, string doc) {
  iadd(option{std::move(str), 1, &ParseArgs::fbool, &arg, std::move(doc)});
}

void ParseArgs::p(string str, char& arg, string doc) {
  iadd(option{std::move(str), 1, &ParseArgs::fchar, &arg, std::move(doc)});
}

void ParseArgs::p(string str, int& arg, string doc) {
  iadd(option{std::move(str), 1, &ParseArgs::fint, &arg, std::move(doc)});
}

void ParseArgs::p(string str, float& arg, string doc) {
  iadd(option{std::move(str), 1, &ParseArgs::ffloat, &arg, std::move(doc)});
}

void ParseArgs::p(string str, double& arg, string doc) {
  iadd(option{std::move(str), 1, &ParseArgs::fdouble, &arg, std::move(doc)});
}

void ParseArgs::p(string str, string& arg, string doc) {
  iadd(option{std::move(str), 1, &ParseArgs::fstring, &arg, std::move(doc)});
}

void ParseArgs::p(string str, int* argp, int narg, string doc) {
  assertx(narg > 0);
  iadd(option{std::move(str), narg, &ParseArgs::fint, argp, std::move(doc)});
}

void ParseArgs::p(string str, float* argp, int narg, string doc) {
  assertx(narg > 0);
  iadd(option{std::move(str), narg, &ParseArgs::ffloat, argp, std::move(doc)});
}

void ParseArgs::p(string str, double* argp, int narg, string doc) {
  assertx(narg > 0);
  iadd(option{std::move(str), narg, &ParseArgs::fdouble, argp, std::move(doc)});
}

void ParseArgs::c(string str, string doc) { iadd(option{std::move(str), 0, nullptr, nullptr, std::move(doc)}); }

void ParseArgs::p(string str, PARSE_FUNC parse_func, string doc) {
  iadd(option{std::move(str), -1, parse_func, nullptr, std::move(doc)});
}

void ParseArgs::p(string str, PARSE_FUNC0 parse_func0, string doc) {
  using voidp = void*;
  iadd(option{std::move(str), -2, PARSE_FUNC(voidp(parse_func0)), nullptr, std::move(doc)});
}

bool ParseArgs::special_arg(const string& s) {
  return s == "-?" || s == "--help" || s == "--version";  // Maybe also s == "--".
}

void ParseArgs::print_help() {
  std::cerr << get_ename() << " Options:" << (_disallow_prefixes ? " (no implicit prefixes)" : "") << "\n";
  for (const option& o : _aroptions) {
    string sdefault;
    if (o.parse_func == &ParseArgs::fquestion && _name != "") continue;
    if (contains(o.doc, "(<unlisted>)")) continue;
    if (o.narg > 0) {
      sdefault = "[";
      for_int(i, o.narg) {
        string s0 = (o.parse_func == &ParseArgs::fbool     ? show_bool(static_cast<bool*>(o.argp)[i])
                     : o.parse_func == &ParseArgs::fchar   ? string(1, static_cast<char*>(o.argp)[i])
                     : o.parse_func == &ParseArgs::fint    ? sform("%d", static_cast<int*>(o.argp)[i])
                     : o.parse_func == &ParseArgs::ffloat  ? show_float(static_cast<float*>(o.argp)[i])
                     : o.parse_func == &ParseArgs::fdouble ? show_double(static_cast<double*>(o.argp)[i])
                     : o.parse_func == &ParseArgs::fstring ? static_cast<string*>(o.argp)[i]
                                                           : "?");
        if (i > 0) sdefault += " ";
        sdefault += s0;
      }
      sdefault += "]";
    }
    auto i = o.doc.find(':');
    int pref = i != string::npos ? int(i) : 0;
    int prefm1 = pref ? pref - 1 : pref;
    string s1 = o.str;
    if (contains(o.str, '[') && !contains(o.str, ']')) s1 += "]";
    s1 += sform(" %.*s", prefm1, o.doc.c_str());
    s1 = sform(" %-28s %s", s1.c_str(), o.doc.c_str() + pref);
    if (sdefault != "") s1 = sform("%-84s %s", s1.c_str(), sdefault.c_str());
    std::cerr << s1.c_str() << "\n";
  }
}

string ParseArgs::get_ename() {
  string s = get_path_tail(_argv0);
  if (_name != "") s += " " + _name;
  return s;
}

void ParseArgs::iadd(option o) {
  if (1 && o.doc != "") {  // Enforce my "conventions" to properly distinguish flags and parameters.
    if (o.narg > 0) {
      if (o.doc[0] == ':' || !contains(o.doc, ':') || o.doc[o.doc.find(':') - 1] != ' ') {
        SHOW(o.str, o.doc);
        Warning("Possibly inconsistent parameter option comment");
      }
    }
    if (!o.narg && o.parse_func) {
      if (o.doc[0] != ':') {
        SHOW(o.str, o.doc);
        Warning("Possibly inconsistent flag option comment");
      }
    }
  }
  _aroptions.push(std::move(o));
}

auto ParseArgs::match(const string& s, bool skip_options) -> const option* {
  option* omatch = nullptr;
  int nmatches = 0, minlfound = std::numeric_limits<int>::max();
  int ls = narrow_cast<int>(s.size());
  for (option& o : _aroptions) {
    if (!o.parse_func) continue;
    if (starts_with(o.str, "*") && (s[0] != '-' || skip_options)) {
      bool allow_case_independent_wildcard = true;
      if (o.str != to_lower(o.str)) allow_case_independent_wildcard = false;
      if (ends_with(allow_case_independent_wildcard ? to_lower(s) : s, o.str.substr(1)) &&
          (!omatch || o.str.size() > omatch->str.size()))
        omatch = &o;
      continue;
    }
    if (o.str[0] == '-' && skip_options) continue;
    if (o.str == "-" && s != "-") continue;  // Require exact match of "-".
    int lo = narrow_cast<int>(o.str.size());
    auto i = o.str.find('[');
    int minfit = i != string::npos ? narrow_cast<int>(i) : _disallow_prefixes ? narrow_cast<int>(o.str.size()) : 0;
    int nchar = clamp(ls, 2, max(lo, ls));
    if (minfit) nchar = minfit;
    if (!o.str.compare(0, nchar, s, 0, nchar)) {
      nmatches++;
      if (nchar == ls && lo == ls) {  // Exact match.
        // Error if 2 exact matches.
        assertx(minlfound);
        minlfound = 0;
        omatch = &o;
      }
      if (!omatch || lo < minlfound) {
        minlfound = lo;
        omatch = &o;
      }
    }
  }
  if (!minlfound) nmatches = 1;
  if (nmatches > 1) showf("Argument '%s' is ambiguous, '%s' assumed\n", s.c_str(), assertx(omatch)->str.c_str());
  return omatch;
}

bool ParseArgs::parse_internal() {
  assertx(_icur != -2);  // Did not already parse.
  bool skip = false;
  while (num()) {
    _icur = _iarg;
    const string& arg = peek_string();
    _curopt = nullptr;
    if (arg == "--") {
      skip = true;  // Treat all remaining arguments as non-options.
      if (!_other_options_ok) {
        shift_args();
        continue;
      }
    } else {
      _curopt = match(arg, skip);
    }
    bool is_option = arg[0] == '-' && arg[1] && !skip;
    if (_curopt && !is_option) {
      assertx(_curopt->narg == -1);  // Wildcard option; do not advance; let client parse_func advance it.
    } else {
      shift_args();
    }
    if (!_curopt) {  // Not found.
      if ((_other_args_ok && !is_option) || (_other_options_ok && is_option)) {
        if (arg != "--" || _other_options_ok) _unrecognized_args.push(arg);
        continue;
      } else {
        _icur = -1;  // Not recognized as an option.
        problem("option not recognized");
      }
    }
    int narg = _curopt->narg;
    if (narg == -2) {
      using voidp = void*;
      PARSE_FUNC0(voidp(_curopt->parse_func))();
    } else {
      _curopt->parse_func(*this);
    }
    if (arg == "-?" || arg == "--help") {
      _unrecognized_args.push(arg);
      return false;
    }
  }
  _curopt = nullptr;
  _icur = -2;
  return true;
}

bool ParseArgs::parse() {
  if (!parse_internal()) return false;
  // Ready to get_*() any leftover arguments.
  _args = std::move(_unrecognized_args);
  _iarg = 0;
  return true;
}

bool ParseArgs::parse_and_extract(Array<string>& aargs) {
  bool ret = parse_internal();
  aargs.init(0);
  aargs.push(_argv0);
  aargs.push_array(std::move(_unrecognized_args));
  return ret;
}

void ParseArgs::problem(const string& s) {
  string mes = get_ename() + " : ParseArgs error : " + s;
  if (_iarg && _iarg - 1 != _icur) mes += " at '" + _args[_iarg - 1] + "'";
  if (_icur >= 0) mes += " when parsing option '" + _args[_icur] + "'";
  if (_curopt && _args[_icur] != _curopt->str) mes += " (interpreted as '" + _curopt->str + "')";
  mes += ".\n";
  std::cerr << mes;
  if (_argv0 != "") {
    std::cerr << "(Use '" << get_path_tail(_argv0) << " -?' to view options.)\n";
    exit_immediately(1);
  } else {
    assertnever("ParseArgs parsing error");
  }
}

void ParseArgs::copy_parse(const ParseArgs& pa) {
  *this = pa;
  _name = "Copy " + pa._name;
}

void ParseArgs::fbool(Args& args) {
  ParseArgs& pargs = static_cast<ParseArgs&>(args);
  auto* argp = static_cast<bool*>(pargs._curopt->argp);
  int n = pargs._curopt->narg;
  if (!n) {  // Set a flag variable.
    if (0 && *argp) {
      SHOW(pargs._curopt->str);
      Warning("ParseArgs: flag is already set");
    }
    *argp = true;
  } else {  // Set a parameter variable.
    for_int(i, n) argp[i] = pargs.get_bool();
  }
}

void ParseArgs::fchar(Args& args) {
  ParseArgs& pargs = static_cast<ParseArgs&>(args);
  auto* argp = static_cast<char*>(pargs._curopt->argp);
  int n = pargs._curopt->narg;
  for_int(i, n) argp[i] = pargs.get_char();
}

void ParseArgs::fint(Args& args) {
  ParseArgs& pargs = static_cast<ParseArgs&>(args);
  auto* argp = static_cast<int*>(pargs._curopt->argp);
  int n = pargs._curopt->narg;
  for_int(i, n) argp[i] = pargs.get_int();
}

void ParseArgs::ffloat(Args& args) {
  ParseArgs& pargs = static_cast<ParseArgs&>(args);
  auto* argp = static_cast<float*>(pargs._curopt->argp);
  int n = pargs._curopt->narg;
  for_int(i, n) argp[i] = pargs.get_float();
}

void ParseArgs::fdouble(Args& args) {
  ParseArgs& pargs = static_cast<ParseArgs&>(args);
  auto* argp = static_cast<double*>(pargs._curopt->argp);
  int n = pargs._curopt->narg;
  for_int(i, n) argp[i] = pargs.get_double();
}

void ParseArgs::fstring(Args& args) {
  ParseArgs& pargs = static_cast<ParseArgs&>(args);
  auto* argp = static_cast<string*>(pargs._curopt->argp);
  int n = pargs._curopt->narg;
  for_int(i, n) argp[i] = pargs.get_string();
}

void ParseArgs::fquestion(Args& args) {
  ParseArgs& pargs = static_cast<ParseArgs&>(args);
  pargs.print_help();
  if (!pargs._other_options_ok) exit_immediately(0);
}

void ParseArgs::fversion(Args& args) {
  ParseArgs& pargs = static_cast<ParseArgs&>(args);
  showf("%s", pargs.header().c_str());
  string str;
#if defined(_MSC_VER)
  str += sform(" MSC=%d", _MSC_VER);
#endif
#if defined(__VERSION__)  // For __GNUC__.
  str += " version=" __VERSION__;
#endif
  str += sform(" cplusplus=%d", int(__cplusplus));
#if defined(_M_IX86_FP)
  str += sform(" IX86_FP=%d", _M_IX86_FP);
#endif
#if defined(_OPENMP)
  str += sform(" OpenMP=%d", _OPENMP);
#endif
// #if defined(__DATE__) && defined(__TIME__) // Not so useful because compilation time of this particular file.
//     str += sform(" built=[%s %s]", __DATE__, __TIME__);
// #endif
#if defined(__AVX__)
  str += " AVX";
#endif
#if defined(__AVX2__)
  str += " AVX2";
#endif
#if defined(__CLR_VER)
  str += sform(" clr=%d", __CLR_VER);
#endif
  showf("%s\n", str.c_str());
  exit_immediately(0);
}

string ParseArgs::header() {
  string smain = "Created at " + get_header_info() + " using:\n";
  const int thresh_line_len = 120 - 5;
  int len = 0;
  string s = g_comment_prefix_string;
  for_int(i, 1 + _args.num()) {
    const string& arg = !i ? _argv0 : _args[i - 1];
    len += narrow_cast<int>(arg.size()) + 1;
    if (i && len > thresh_line_len) {
      s = s + "\n" + g_comment_prefix_string + " ";
      len = narrow_cast<int>(arg.size()) + 1;
    }
    s = s + " " + arg;
  }
  s = s + "\n";
  return smain + s;
}

}  // namespace hh
