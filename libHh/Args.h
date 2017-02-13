// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ARGS_H_
#define MESH_PROCESSING_LIBHH_ARGS_H_

#include "Vec.h"
#include "Array.h"

// Parsing of command-line arguments.

#if 0
namespace {
bool nooutput = false;
int niter = 0;
void do_png() { ...; }
void do_outfile(Args& args) { string filename = args.get_filename(); ...; }
} // namespace
int main(int argc, const char** argv) {
    ParseArgs args(argc, argv);
    ARGSC("",                   ":Comment : Directly set parameters:");
    ARGSF(nooutput,             ": do not output final image on stdout");
    ARGSP(niter,                "n : number of iterations");
    ARGSC("",                   ":Comment : Directly call functions:");
    ARGSD(png,                  ": set output format");
    ARGSD(outfile,              "filename : output an intermediate image");
    args.parse();
    if (!nooutput) { ...; }
}
#endif

namespace hh {

// Args represents a stream of string arguments.
class Args {
 public:
    explicit Args(CArrayView<string> aargs)     : _args(aargs) { }
    explicit Args(std::initializer_list<string> l) : Args(CArrayView<string>(l)) { }
    virtual ~Args()                             { }
    int num() const                             { return _args.num()-_iarg; } // number of arguments left
    size_t size() const                         { return _args.size()-_iarg; }
    void restart()                              { _iarg = 0; }
    void ensure_at_least(int n); // assert num()>=n
    const string& peek_string() const           { assertx(num()>0); return _args[_iarg]; }
    bool          get_bool();
    char          get_char();
    int           get_int();
    float         get_float();
    double        get_double();
    const string& get_string();
    string get_filename(); // check for legal characters; translate '\'; may not begin with '-' (unless equal to "-")
// For misc use:
    static bool check_bool  (const string& s);
    static bool check_char  (const string& s);
    static bool check_int   (const string& s);
    static bool check_float (const string& s);
    static bool check_double(const string& s);
    static bool check_filename(const string& s);
    static bool   parse_bool  (const string& s); // or die
    static char   parse_char  (const string& s); // or die
    static int    parse_int   (const string& s); // or die
    static float  parse_float (const string& s); // or die
    static double parse_double(const string& s); // or die
    virtual void problem(const string& s);
 private:
    friend class ParseArgs;
    Array<string> _args;        // (_args[0] corresponds to argv[1])
    int _iarg {0};              // next argument in _args
    const string& shift_args()                  { assertx(num()>0); return _args[_iarg++]; }
    Args& operator=(const Args&)                = default; // used in friend ParseArgs: copy_parse()
    Args()                                      = default; // used in friend ParseArgs
    Args(const Args&)                           = delete;  // not noncopyable because operator=() is defined above
};

// ParseArgs adds functionality for parsing the stream of string arguments using a set of options.
// It automatically updates variables or calls functions specified by the recognized options.
class ParseArgs : public Args {
    using PARSEF  = void (*)(Args& args);
    using PARSEF0 = void (*)();
 public:
    // e.g. name is "", "HW", "HB_GL"
    ParseArgs(int& argc, const char**& argv); // takes ownership; sets argc=0, argv=nullptr; ensure_utf8_encoding()
    explicit ParseArgs(CArrayView<string> aargs, const string& name = "");
// Define options: (any prefix of string str is recognized unless it contains a '[' or disallow_prefixes() is set)
    void f(string str, bool&        arg,        string doc = ""); // sets flag to true
    void p(string str, bool&        arg,        string doc = ""); // reads one parameter
    void p(string str, char&        arg,        string doc = "");
    void p(string str, int&         arg,        string doc = "");
    void p(string str, float&       arg,        string doc = "");
    void p(string str, double&      arg,        string doc = "");
    void p(string str, string&      arg,        string doc = "");
    void p(string str, int*   argp,  int narg,  string doc = ""); // reads narg parameters
    void p(string str, float* argp,  int narg,  string doc = "");
    void p(string str, double* argp, int narg,  string doc = "");
    template<typename T, int narg>
    void p(string str, Vec<T,narg>& arg,     string doc = "") { p(str, arg.data(), int(narg), doc); }
    template<typename T, size_t narg>
    void p(string str, T(&arg)[narg],           string doc = "") { p(str, arg, int(narg), doc); }
    void c(string str = "",                     string doc = ""); // add a comment in the options list
    void p(string str, PARSEF parsef,           string doc = ""); // parsing function taking args
    void p(string str, PARSEF0 parsef0,         string doc = ""); // parsing function without args
    // by default, all arguments must parse
    void other_args_ok()        { _other_args_ok = true; } // non -* are ok (filenames); after "--", all args ok
    void other_options_ok()     { _other_options_ok = true; } // unrecognized -* are ok
    void disallow_prefixes()    { _disallow_prefixes = true; } // options implicitly end with '[' if none is present
    // Note: usually, other_options_ok() implies that disallow_prefixes() should be set too.
    static bool special_arg(const string& s); // true if "-?" or "--help" or "--version"
    void print_help();
// Perform argument parsing:
    bool parse();               // main function; returns success (false if "-?" is found)
    bool parse_and_extract(Array<string>& aargs);
    void copy_parse(const ParseArgs& pa);
    string header();
    void problem(const string& s) override;
 private:
    struct option {
        option()                                = default;
        option(string pstr, int pnarg, PARSEF pparsef, void* pargp, string pdoc)
            : str(std::move(pstr)), narg(pnarg), parsef(pparsef), argp(pargp), doc(std::move(pdoc)) { }
        string str;
        int narg;               // number of arguments; >=0 if built-in PARSEF, -1 if PARSEF, -2 if PARSEF0
        PARSEF parsef;
        void* argp;
        string doc;
    };
    string _name;               // name of options (e.g. "HW")
    string _argv0;
    bool _other_args_ok {false};
    bool _other_options_ok {false};
    bool _disallow_prefixes {false};
    Array<option> _aroptions;
    const option* _curopt {nullptr}; // option currently being parsed
    int _icur {-1};                  // index of current option in _args
    Array<string> _unrecognized_args;
    bool parse_internal();      // returns success (false if "-?" is found)
    void common_construction();
    string get_ename();
    void iadd(option o);
    const option* match(const string& s, bool skip_options);
    static void fbool    (Args& args);
    static void fchar    (Args& args);
    static void fint     (Args& args);
    static void ffloat   (Args& args);
    static void fdouble  (Args& args);
    static void fstring  (Args& args);
    static void fquestion(Args& args);
    static void fversion (Args& args);
    ParseArgs& operator=(const ParseArgs&)      = default; // used in copy_parse()
    ParseArgs(const ParseArgs&)                 = delete;  // not noncopyable because operator=() is defined above
};

#define ARGSF(var, comment)   args.f("-" #var, var, comment)
#define ARGSP(var, comment)   args.p("-" #var, var, comment)
#define ARGSD(var, comment)   args.p("-" #var, do_##var, comment)
#define ARGSC(comment1, com2) args.c(comment1, com2)

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_ARGS_H_
