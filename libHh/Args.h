// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ARGS_H_
#define MESH_PROCESSING_LIBHH_ARGS_H_

#include "libHh/Array.h"
#include "libHh/Vec.h"

// Parsing of command-line arguments.

#if 0
{
  namespace {
  bool nooutput = false;
  int niter = 0;
  void do_png() { some_action(); }
  void do_outfile(Args& args) { string filename = args.get_filename(); }
  }  // namespace

  int main(int argc, const char** argv) {
    ParseArgs args(argc, argv);
    HH_ARGSC("", ":Comment : Directly set parameters:");
    HH_ARGSF(nooutput, ": do not output final image on stdout");
    HH_ARGSP(niter, "n : number of iterations");
    HH_ARGSC("", ":Comment : Directly call functions:");
    HH_ARGSD(png, ": set output format");
    HH_ARGSD(outfile, "filename : output an intermediate image");
    args.parse();
    if (!nooutput) write_some_output();
  }
  return 0;
}
#endif

namespace hh {

// Args represents a stream of string arguments.
class Args {
 public:
  explicit Args(CArrayView<string> aargs) : _args(aargs) {}
  explicit Args(std::initializer_list<string> l) : Args(CArrayView<string>(l)) {}
  virtual ~Args() {}
  int num() const { return _args.num() - _iarg; }  // Number of arguments left.
  size_t size() const { return _args.size() - _iarg; }
  void restart() { _iarg = 0; }
  void ensure_at_least(int n);  // Asserts num() >= n.
  const string& peek_string() const { return (assertx(num() > 0), _args[_iarg]); }
  bool get_bool();
  char get_char();
  int get_int();
  float get_float();
  double get_double();
  const string& get_string();
  string get_filename();  // Check for legal characters; translate '\'; may not begin with '-' (unless equal to "-").

  // For misc use:
  Args& use() && { return *this; }
  static bool check_bool(const string& s);
  static bool check_char(const string& s);
  static bool check_int(const string& s);
  static bool check_float(const string& s);
  static bool check_double(const string& s);
  static bool check_filename(const string& s);
  static bool parse_bool(const string& s);      // Or die.
  static char parse_char(const string& s);      // Or die.
  static int parse_int(const string& s);        // Or die.
  static float parse_float(const string& s);    // Or die.
  static double parse_double(const string& s);  // Or die.
  virtual void problem(const string& s);

 private:
  friend class ParseArgs;
  Array<string> _args;  // Note that _args[0] corresponds to argv[1].
  int _iarg{0};         // Next argument in _args.
  const string& shift_args() { return (assertx(num() > 0), _args[_iarg++]); }
  Args& operator=(const Args&) = default;  // Used in friend ParseArgs: copy_parse().
  Args() = default;                        // Used in friend ParseArgs.
  Args(const Args&) = delete;              // Not noncopyable because operator=() is defined above.
};

// ParseArgs adds functionality for parsing the stream of string arguments using a set of options.
// It automatically updates variables or calls functions specified by the recognized options.
class ParseArgs : public Args {
  using PARSE_FUNC = void (*)(Args& args);
  using PARSE_FUNC0 = void (*)();

 public:
  // e.g. name is "", "HW", "HB_GL".
  // Takes ownership; sets argc = 0, argv = nullptr; ensure_utf8_encoding().
  explicit ParseArgs(int& argc, const char**& argv);
  explicit ParseArgs(CArrayView<string> aargs, string name = "");

  // Define options: (any prefix of string str is recognized unless it contains a '[' or disallow_prefixes() is set):
  void f(string str, bool& arg, string doc = "");  // Sets flag to true.
  void p(string str, bool& arg, string doc = "");  // Reads one parameter.
  void p(string str, char& arg, string doc = "");
  void p(string str, int& arg, string doc = "");
  void p(string str, float& arg, string doc = "");
  void p(string str, double& arg, string doc = "");
  void p(string str, string& arg, string doc = "");
  void p(string str, int* argp, int narg, string doc = "");  // Reads narg parameters.
  void p(string str, float* argp, int narg, string doc = "");
  void p(string str, double* argp, int narg, string doc = "");
  template <typename T, int narg> void p(string str, Vec<T, narg>& arg, string doc = "") {
    p(str, arg.data(), narrow_cast<int>(narg), doc);
  }
  template <typename T, size_t narg> void p(string str, T (&arg)[narg], string doc = "") {
    p(str, arg, narrow_cast<int>(narg), doc);
  }
  void c(string str = "", string doc = "");                      // Add a comment in the options list.
  void p(string str, PARSE_FUNC parse_func, string doc = "");    // Parsing function taking args.
  void p(string str, PARSE_FUNC0 parse_func0, string doc = "");  // Parsing function without args.
  // By default, all arguments must parse.
  void other_args_ok() { _other_args_ok = true; }          // Non -* are ok (filenames); after "--", all args ok.
  void other_options_ok() { _other_options_ok = true; }    // Unrecognized -* are ok.
  void disallow_prefixes() { _disallow_prefixes = true; }  // Options implicitly end with '[' if none is present.
  // Note: usually, other_options_ok() implies that disallow_prefixes() should be set too.
  static bool special_arg(const string& s);  // True if "-?" or "--help" or "--version".
  void print_help();

  // Perform argument parsing:
  bool parse();  // Main function; returns success (false if "-?" is found).
  bool parse_and_extract(Array<string>& aargs);
  void copy_parse(const ParseArgs& pa);
  string header();
  void problem(const string& s) override;

 private:
  struct option {
    option() = default;
    option(string pstr, int narg_, PARSE_FUNC parse_func_, void* argp_, string doc_)
        : str(std::move(pstr)), narg(narg_), parse_func(parse_func_), argp(argp_), doc(std::move(doc_)) {}
    string str;
    int narg;  // Number of arguments; >= 0 if built-in PARSE_FUNC, -1 if PARSE_FUNC, -2 if PARSE_FUNC0.
    PARSE_FUNC parse_func;
    void* argp;
    string doc;
  };
  string _name;  // Name of options (e.g. "HW").
  string _argv0;
  bool _other_args_ok{false};
  bool _other_options_ok{false};
  bool _disallow_prefixes{false};
  Array<option> _aroptions;
  const option* _curopt{nullptr};  // Option currently being parsed.
  int _icur{-1};                   // Index of current option in _args.
  Array<string> _unrecognized_args;
  bool parse_internal();  // Returns success (false if "-?" is found).
  void common_construction();
  string get_ename();
  void iadd(option o);
  const option* match(const string& s, bool skip_options);
  static void fbool(Args& args);
  static void fchar(Args& args);
  static void fint(Args& args);
  static void ffloat(Args& args);
  static void fdouble(Args& args);
  static void fstring(Args& args);
  static void fquestion(Args& args);
  static void fversion(Args& args);
  ParseArgs& operator=(const ParseArgs&) = default;  // Used in copy_parse().
  ParseArgs(const ParseArgs&) = delete;              // Not noncopyable because operator=() is defined above.
};

#define HH_ARGSF(var, comment) args.f("-" #var, var, comment)
#define HH_ARGSP(var, comment) args.p("-" #var, var, comment)
#define HH_ARGSD(var, comment) args.p("-" #var, do_##var, comment)
#define HH_ARGSC(header_comment, comment) args.c(header_comment, comment)
#define HH_ARGSF_O(var, comment) args.f("-" #var, options.var, comment)
#define HH_ARGSP_O(var, comment) args.p("-" #var, options.var, comment)
#define HH_ARGS_INDENT "                            "

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_ARGS_H_
