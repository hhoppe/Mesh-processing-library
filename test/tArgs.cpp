// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#if 0 && defined(_MSC_VER) && _MSC_VER==1900
// buggy VS2015 warning about shadowed variable:
//  declaration of 'ar' (in Array.h) hides global declaration (::ar)
// fails to have effect in VS 2015 update 1 with rel=1 due to precompiled headers?
// Instead, moved "ar" inside main()
#pragma warning(disable:4459)
#endif

#include "Args.h"
using namespace hh;

namespace {

void do_show1p1() { SHOW(1+1); }

bool flag2 = false;

} // namespace

int main(int argc, const char** argv) {
    static Vec2<int> ar = {0, 0}; // see comment above
    if (!getenv_bool("TARGS_PHASE2")) {
        SHOW(ArView(argv, argc));
        ParseArgs args(argc, argv);
        bool flag = false, flap = false, flac = false;
        int val1 = 0, val2 = 0;
        float fa[2] = {0.f, 0.f};   // test C-array
        Vec2<float> fb = {0.f, 0.f};
        Vec2<float> fc = {0.f, 0.f};
        HH_ARGSF(flag,                  ": enable flag");
        HH_ARGSF(flap,                  ": turn on the flaps");
        args.f("-flac", flac,           ": send out flacs");
        args.p("-val1", val1,           "f : set value1 coefficient");
        HH_ARGSP(val2,                  "i : comment");
        args.p("-farr", fa, 2,          "f1 f2 : sets two element array");
        HH_ARGSC("",                    ":");
        HH_ARGSP(fb,                    "a b : set variables");
        HH_ARGSP(fc,                    "c1 c2 : set the fc variables");
        HH_ARGSD(show1p1,               ": comment");
        args.other_args_ok(); args.other_options_ok();
        Array<string> ar_unrecog;
        bool optsparse = args.parse_and_extract(ar_unrecog);
        SHOW(optsparse, ar_unrecog);
        SHOW(flag, flap, flac, val1, val2);
        SHOW(fa[0], fa[1]);
        SHOW(fb[0], fb[1]);
        SHOW(fc[0], fc[1]);
    } else {
        auto do_showar = [](Args& args) { int i = args.get_int(); SHOW("showar", i, ar[i]); };
        auto do_vlp = [](Args& args) { SHOW("reading vlp", args.get_filename(), ar); };
        auto do_file = [](Args& args) { SHOW("reading file", args.get_filename(), ar); };
        auto do_string = [](Args& args) { SHOW("string", args.get_string(), ar); };
        SHOW(ArView(argv, argc));
        ParseArgs args(argc, argv);
        HH_ARGSF(flag2,                 ": enable flag");
        HH_ARGSP(ar,                    "i1 i2 : set two coefficients");
        HH_ARGSD(showar,                "i : show coefficient indexed i");
        args.p("*.vlp", do_vlp,         "file.vlp : read the file");
        args.p("*", do_file,            "file : read any other file type");
        args.p("*.string", do_string,   "string : print string");
        bool optsparse = args.parse();
        SHOW(optsparse);
        SHOW(flag2, ar);
    }
    return 0;
}
