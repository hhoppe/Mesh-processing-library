// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Random.h"
#include "Array.h"
#include "Geometry.h"
#include "FileIO.h"
#include "Advanced.h"
#include "RangeOp.h"            // reverse(), sort()
#include "StringOp.h"
using namespace hh;

#if defined(_WIN32)
#include <process.h>       // _spawnvp()
#endif

namespace {

const string tmpf = "v.tHh.txt";

void try_it(const string& stest) {
    if (0) SHOW(stest);
    for_int(method, 2) {        // csh, sh, cmd
        if (0) SHOW(method);
        string s1 = quote_arg_for_sh(stest); // stronger than quote_arg_for_shell()
        if (0) SHOW(s1);
        if (0) {
            string s2 = "echo " + s1 + " >" + tmpf;
            // Array<string> sargv = {"csh", "-c", s2}; // works always also
            // Array<string> sargv = {"sh", "-c", s2}; // fails
            // Array<string> sargv = {"bash", "-c", s2}; // fails too

            // Going to a Cygwin process -- ouch!
            // Array<string> sargv = {"sh", "-v", "-c", s2};
            // stest = hh1 "a' b.txt" "a' b.txt" "a' b.txt" |
            // s1 = hh1\ \"a\'\ b\.txt\"\ \"a\'\ b\.txt\"\ \"a\'\ b\.txt\"\ \|
            // echo hh1\ \a\"\" b"\".txt"\""\"""\" "\""\""a"\"

            Array<string> sargv = {"c:/mingw/msys/1.0/bin/sh", "-v", "-c", s2};

            // Array<string> sargv = {"csh", "-v", "-c", s2};
            // stest = hh1 "a' b.txt" "a' b.txt" "a' b.txt" |
            // s1 = hh1\ \"a\'\ b\.txt\"\ \"a\'\ b\.txt\"\ \"a\'\ b\.txt\"\ \|
            // echo hh1\ \"a\'\ b\.txt\"\ \"a\'\ b\.txt\"\ \"a\'\ b\.txt\"\ \| > v.tHh.txt

            assertx(!my_spawn(sargv, true));
        } else if (method==0) { // works always
            string s2 = "echo " + s1 + " >" + tmpf;
            Array<string> largv = {"csh", "-c", s2}; // or "-vc"
            assertx(!my_spawn(largv, true));
        } else if (0 && method==1) { // equivalent to the one below
            string s2 = "echo " + s1 + " >" + tmpf;
            // Array<string> largv = {"c:/cygwin/bin/bash", "-c", s2}; // or "-vc"
            Array<string> largv = {"sh", "-c", s2}; // or "-vc"
            assertx(!my_spawn(largv, true));
        } else if (1 && method==1) {
            string s2 = "echo " + s1 + " >" + tmpf;
            assertx(!my_sh(s2));
        } else if (method==2) { // I give up on this.
            string s1cmd = '"' + stest + '"';
            // string s1cmd = windows_spawn_quote(stest);
            // string s2 = "echo " + s1cmd + " >" + tmpf; // DOS eol; extra space at eol; echo ignores quotes
            // string s2 = "/cygwin/bin/echo " + s1cmd + " >" + tmpf; // cygwin quote parsing is poor
            string s2 = "/mingw/msys/1.0/bin/echo " + s1cmd + " >" + tmpf; // still does not handle double-quotes
            string sbu = getenv_string("FORCE_CMD"); // "" if undefined
            my_setenv("FORCE_CMD", "1");
            assertx(!my_sh(s2));
            my_setenv("FORCE_CMD", sbu);
        }
        RFile fi(tmpf);
        string sline; assertx(my_getline(fi(), sline));
        // SHOW(sline);
        // Here I once saw the error "method=1 stest=a b c sline=a\ b\ c"; it was due to a setting of env
        //  variable CYGWIN="noglob nodosfilewarning" likely set during an interrupted Emacs grep -- since fixed.
        if (sline!=stest) SHOW(method, stest, sline);
        assertx(sline==stest);
    }
}

void test_spawn() {
    {
        try_it("abc");
        try_it("a b c");
        try_it("'a b' c");
        try_it("\"a b\" c");
        try_it("hh1 \"a' b.txt\" \"a' b.txt\" \"a' b.txt\" |");
        try_it("hh1 \"a' \\ b.txt\" \"a' b.txt\" \"a' b.txt\" |");
        try_it("hh1 \"a' \" b");
        try_it("hh1 \"a' \" b.txt\" \"a' ");
        try_it("\"|");
        try_it("hh1 \"a' \" b.txt\" \"a' b.txt\" \"a' b.txt\" |");
        try_it("hh1 \"a' \" b.txt\" \"a' b.txt\" \"a' b.txt\" &");
        try_it("hh1 \"a' | & \" b.txt\" \"a' b.txt\" \"a' b.txt\" &");
        try_it("hh1 \"a' | & \" b.txt\" \"a' b.txt\" \"a' b.txt\" &~!@#$%^&*()`[]{};:,.<>/?-_=\\+");
        try_it("%^&*()`[]{};:,.<>/?-_=\\+");
        try_it("%^&*()");
    }
    {
        const string arcands = "abcdefgh       `~!@#$%^&*()-_=+[{]}\\|;:'\"\",<.>/?"; // double '"' frequency
        for_int(itry, 10) {         // tried up to 10000
            const int len = 20;
            string str(len, ' ');
            for_int(i, len) {
                str[i] = arcands[Random::G.get_unsigned(unsigned(arcands.size()))];
            }
            try_it(str);
        }
    }
    assertx(!unlink(tmpf.c_str()));
}

void test_spawn2() {
#if defined(_WIN32)
#define E SHOW("*", s); SHOW(_spawnvp(P_WAIT, shell0, V(shell, "-c", s, nullptr).data())) // or "-vc"
    // echo 'a'\''b' // a'b
    // csh -c 'echo '\''a'\''\'\'''\''b'\''' // a'b
    // sh -c  'echo '\''a'\''\'\'''\''b'\''' // a'b
    const char* s; const char* shell; const char* shell0;
    if (0) {
        // csh parses command using double-quotes with "\"" for inner double quotes;
        //  it does not recognize single-quotes.  (due to crt/stdargv.c : parse_cmdline())
        shell = "csh"; shell0 = shell; SHOW("**", shell, shell0);
        if (0) shell0 = "sh";       // Then it behaves differently; it parses args more like cygwin bash!
        //
        s = R"(echo a b)"; E;       // echo         null
        s = R"('echo a b')"; E;     // 'echo        Unmatched '.
        s = R"("echo a b")"; E;     // echo a b     a b
        s = R"("echo 'a b'")"; E;   // echo 'a b'   a b
        s = R"('echo "a b"')"; E;   // 'echo        Unmatched '.
        s = R"(echo\ \'a b\')"; E;  // Argument for -c ends in backslash.
        s = R"(echo" "\"a b\")"; E; // echo "a      Unmatched ".
        s = R"('echo \"a b\"')"; E; // 'echo        Unmatched '.
        // The outer double quotes (parsed by Windows) allow backslash of inner double quotes, just like csh/tcsh.
        s = R"("echo \\\"a b\\\"")"; E; // echo \"a b\"      "a b"
        s = R"("echo \'a b\'")"; E;     // echo \'a b\'      'a b'
        // The outer single quotes (parsed by Windows) prefer backslash of inner single quotes (\')
        //  whereas the inner single quotes (parsed by sh/csh) require exiting to backslash inner quote ('\'').
        s = R"('echo '\''a'\''\'\'''\''b'\''')"; E; // 'echo        Unmatched '.
        s = R"('echo \''a'\''\'\'''\''b'\''')"; E;  // 'echo        Unmatched '.
        s = R"('echo \'a\'\\\'\'b\'')"; E;          // 'echo        Unmatched '.
        // Try using double-quotes with "\"" for inner double quotes
        s = R"("echo "\""a b"\""")"; E;                 // echo "a b"       a b
        //  echo\ \"a\ b\"
        s = R"("echo"\\" "\\""\""a"\\" b"\\""\\"")"; E; // echo\ \"a\ b\\   echo "a b\: Command not found.
    }
    if (1) {
        // cygwin sh does not use windows crt, so has its own scheme for parsing command
        shell = "sh"; shell0 = shell; SHOW("**", shell, shell0);
        s = R"(echo a b)"; E;       // echo             null
        s = R"('echo a b')"; E;     // echo a b         a b
        s = R"("echo a b")"; E;     // echo a b         a b
        s = R"("echo 'a b'")"; E;   // echo 'a b'       a b
        s = R"('echo "a b"')"; E;   // echo "a b"       a b
        s = R"(echo\ \'a b\')"; E;  // echo\            null
        s = R"(echo" "\"a b\")"; E; // echo \a b"       err
        s = R"('echo \"a b\"')"; E; // echo \"a b\"     "a b"
        // The outer double quotes (parsed by Windows) allow backslash of inner double quotes, just like csh/tcsh.
        s = R"("echo \\\"a b\\\"")"; E; // echo \"a b\"      "a b"
        s = R"("echo \'a b\'")"; E;     // echo \'a b\'      'a b'
        // The outer single quotes (parsed by Windows) prefer backslash of inner single quotes (\')
        //  whereas the inner single quotes (parsed by sh/csh) require exiting to backslash inner quote ('\'').
        s = R"('echo '\''a'\''\'\'''\''b'\''')"; E; // echo a'\''b'      a\b   unexpected
        s = R"('echo \''a'\''\'\'''\''b'\''')"; E;  // echo 'a'\''b'     a'b   enexpected; works
        s = R"('echo \'a\'\\\'\'b\'')"; E;          // echo 'a'\''b'     a'b   unexpected; works
        // Try using double-quotes with "\"" for inner double quotes
        s = R"("echo "\""a b"\""")"; E;                 // echo a    a    unexpected
        // Instead backslash inner double quotes
        s = R"("echo \"a b\"")"; E; // echo "a b"       a b     works
        s = R"("echo \"a b\"")"; E; // echo "a b"       a b     works  (just backslash inner double-quotes)
        s = R"("echo '\"'")"; E;    // echo '"'         "       works
        s = R"("echo \\\"")"; E;    // echo \"          "       works
    }
#undef E
#endif
}

void test_implicit_default_virtual_destructor() {
    struct A {
        ~A() { SHOW("~A()"); }
    };
    struct Base {
        virtual ~Base() { }
    };
    struct Derived : Base {
        A _a;
    };
    { Derived derived; }
    {
        Base* p = make_unique<Derived>().release();
        delete p;
    }
}

// This does not do shortcut evaluation!   "const T&" gives wrong results on VS2013.
template<typename T, typename... Args> T first_of(T a1, Args... args);
template<typename T> T first_of(T a1) { return a1; }
template<typename T, typename... Args> T first_of(T a1, Args... args) { return a1 ? a1 : first_of(args...); }

void func1(uchar c1, uchar c2, uchar c3, uchar c4) {
    SHOW(int(c1), int(c2), int(c3), int(c4));
}

void func2() {
    throw std::runtime_error("func2_err");
}

void func3() {
    struct S {
        S(string s) : _s(s) { SHOW("start " + _s); }
        ~S() { SHOW("end " + _s); }
        string _s;
    };
    S s0("s0");
    struct SS {
        SS() : _s1("s1"), _s2("s2"), _s3((func2(), "s3")), _s4("s4") { SHOW("SS never"); }
        ~SS() { SHOW("~SS never"); }
        S _s1, _s2, _s3, _s4;
    };
    SS ss;
}

void func4() {
    struct S {
        S() { SHOW("default constructor"); }
        // S(string s) : _s(s) { SHOW("start " + _s); }
        ~S() { SHOW("end " + _s); }
        string _s;
    };
    struct SS {
        SS() { _s1._s = "s1"; _s2._s = "s2"; func2(); } // see if _s1 and _s2 are destructed; yes they are.
        ~SS() { SHOW("~SS never"); }
        S _s1, _s2, _s3, _s4;
    };
    SS ss;
}

void test_exceptions() {
    SHOW("test func3");
    try {
        func3();
    }
    catch (const std::exception& ex) {
        SHOW("catch3", ex.what());
    }
    SHOW("test func4");
    try {
        func4();
    }
    catch (const std::exception& ex) {
        SHOW("catch4", ex.what());
    }
    if (0) {
        throw std::runtime_error("err2"); // excercise my_terminate() or my_top_level_exception_filter()
        // instead use "HTest -exception"
    }
}

} // namespace


int main() {
    if (0) { test_spawn2(); return 0; }
    if (1) {
        test_spawn();
        if (0) return 0;
    }
    {
        assertx(!getenv("ZZ"));
        assertx(!getenv_bool("ZZ"));
        assertx(getenv_int("ZZ")==0);
        assertx(getenv_float("ZZ", 4.f)==4.f);
        assertx(getenv_string("ZZ")=="");
        const Array<const char*> strings = { "ABC", "LONG_WORD", "MIX8WORD", "OTHER" };
        for_int(i, 100) {
            const char* s = strings[Random::G.get_unsigned(strings.num())];
            if (Random::G.unif()<.5f) {
                my_setenv(s, "1");
                assertx(getenv(s));
                assertx(getenv(s)==string("1"));
                assertx(getenv_bool(s));
                assertx(getenv_int(s)==1);
                assertx(getenv_float(s, 4.f)==1.f);
                assertx(getenv_string(s)=="1");
            } else {
                my_setenv(s, "");
                assertx(!getenv(s));
                assertx(!getenv_bool(s));
                assertx(getenv_int(s)==0);
                assertx(getenv_float(s, 4.f)==4.f);
                assertx(getenv_string(s)=="");
            }
        }
        for_int(i, 11) { Warning("Should occur 11 times"); }
        for_int(i, 3) { if (!assertw(i<0)) Warning("Should occur 3 times"); }
        Warning("A is chronologically earliest");
        Warning("z is chronologically last");
    }
    {                           // test SHOW
        {
            Array<int> ar;
            for_int(i, 8) { ar.push(i); }
            SHOW(ar);
        }
        {
            Array<Point> ar;
            for_int(i, 8) { ar.push(Point(float(i), 0.f, 0.f)); }
            SHOW(ar);
        }
        {
            Point p(1.f, 2.f, 3.f);
            Vector v(4.f, 5.f, 6.f);
            SHOW(1);
            SHOW(p);
            SHOW(v);
            SHOW(p+v);
            SHOW(2.f*v);
        }
        SHOW(1.f/3.f);
        SHOWP(1.f/3.f);
        SHOW(1.f/3.f, "string", 4.f/7.f);
        SHOWP(1.f/3.f, "string", 4.f/7.f);
    }
    test_implicit_default_virtual_destructor();
    {
        string s1 = "ab\"!''\"\"'&cdefg";
        string s2 = "hh1 \"a' | & \" b.txt\" \"a' b.txt\" \"a' b.txt\" &~!@#$%^&*()`[]{};:,.<>/?-_=\\+";
        SHOW(quote_arg_for_sh(s1));
        SHOW(quote_arg_for_sh(s2));
    }
    {
        std::cout << sform("%d %d\n", 37, 29);
        string s;
        for_int(i, 30) {
            ssform(s, "%d", (1<<i)); SHOW(s);
        }
    }
    {
        SHOW(first_of(0, 0, 2, 1));
        SHOW(first_of(2));
        SHOW(first_of(0));
        SHOW(first_of(0, 2));
        SHOW(first_of(2, 0));
    }
    {
        // raw string literal examples
        string s0 = R"(new literal\t string)";
        string s1 = R"(line 1
line2)";
        string s2 = R"(it indents ok)";
        SHOW(s0);
        SHOW(s1);
        SHOW(s2);
    }
    {
        Array<int> ar {3, 2, 1, 4, 5};
        SHOW(ar);
        SHOW(reverse(clone(ar)));
        SHOW(ar);
        SHOW(reverse(ar));
        SHOW(ar);
        SHOW(sort(clone(ar)));
        SHOW(ar);
        SHOW(sort(ar));
        SHOW(ar);
#if 0
        ArrayView<int> arv(ar); auto arv2 = clone(arv); dummy_use(arv2); // correctly fails static_assert
#endif
    }
    {
        struct A {
            A()                                 { SHOW("A default construct"); }
            A(const A&)                         { SHOW("A copy construct"); }
            A(A&&)                              { SHOW("A move construct"); }
            ~A()                                { SHOW("A destruct"); }
        };
        SHOW("1");
        { A a; }
        SHOW("2");
        { A(); }
        SHOW("3");
        { A a; A a2(a); }
        SHOW("4");
        { clone(A()); }
        SHOW("5");
        { A a; clone(a); }
        SHOW("6");
    }
    {
        SHOW(type_name<sum_type_t<float>>());
        SHOW(type_name<sum_type_t<double>>());
        SHOW(type_name<sum_type_t<char>>());
        SHOW(type_name<sum_type_t<uchar>>());
        SHOW(type_name<sum_type_t<short>>());
        SHOW(type_name<sum_type_t<ushort>>());
        SHOW(type_name<sum_type_t<int>>());
        SHOW(type_name<sum_type_t<unsigned>>());
        SHOW(type_name<sum_type_t<char*>>());
    }
    {
        constexpr auto vsign = sign(-TAU); SHOW(vsign, type_name<decltype(vsign)>());
        constexpr auto vsignz = signz(-TAU); SHOW(vsignz, type_name<decltype(vsignz)>());
        constexpr auto vsignd = sign(-D_TAU); SHOW(vsignd, type_name<decltype(vsignd)>());
        constexpr auto vsignzd = signz(-D_TAU); SHOW(vsignzd, type_name<decltype(vsignzd)>());
        constexpr auto vsquare5 = square(5); SHOW(vsquare5);
        // constexpr auto vclamped10 = clamp(-11, 10, 20); SHOW(vclamped10);
        const int v5mod3 = mod3(5); SHOW(v5mod3);
        const int vint33 = assert_narrow_cast<int>(33ll); SHOW(vint33); // C++14 constexpr
    }
    {
        Array<uchar> buf(10);
        SHOW(sizeof(buf[0]));
    }
    {
        Array<ushort> buf(0);
        SHOW(sizeof(buf[0]));
    }
    {
#define F_EACH(x) 2*x
#define F(...) HH_APPLY((F_EACH, __VA_ARGS__))
        float a = pow(F(1.f, 2.f));
        SHOW(a);
        float b[] = {F(1.f, 2.f)};
        SHOW(b[0]);
        SHOW(b[1]);
#undef F
#undef F_EACH
    }
    {
        int n;
        n = HH_NUM_ARGS(1); SHOW(n);
        n = HH_NUM_ARGS(1, 2); SHOW(n);
        n = HH_NUM_ARGS(1, 2, 3); SHOW(n);
        n = HH_NUM_ARGS(1, 2, 3, 4); SHOW(n);
        n = HH_NUM_ARGS(1, 2, 3, 4, 5); SHOW(n);
        SHOW(HH_NUM_ARGS(a, b, c, d, e, f));
    }
    {
        // SHOW(HH_NUM_ARGS());
        SHOW(HH_NUM_ARGS(1));
        SHOW(HH_NUM_ARGS(1, 2));
        SHOW(HH_NUM_ARGS(1, 2, 3));
        //
        // SHOW(HH_HAVE_ARGS());
        // SHOW(HH_HAVE_ARGS(1));
        // SHOW(HH_HAVE_ARGS(1, 2));
        // SHOW(HH_HAVE_ARGS(1, 2, 3));
        //
        // SHOW(HH_HAVE_GT1_ARGS());
        SHOW(HH_HAVE_GT1_ARGS(1));
        SHOW(HH_HAVE_GT1_ARGS(1, 2));
        SHOW(HH_HAVE_GT1_ARGS(1, 2, 3));
    }
    {
        float a = 16.f;
        int n = 5;
        auto b = V(2.f, 4.f);
        SHOW(a);
        SHOW(a, n, a);
        SHOW(1, 2, 3, 4, 5);
        SHOW(1, 2, 3, 4, 5, 6);
        SHOW(1, 2, 3, 4, 5, 6, 7);
        SHOW(1, 2, 3, 4, 5, 6, 7, 8);
        SHOW(1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);
        SHOW(a, b[0], b[1], n, a, a, a);
        SHOW(n, a, b[0], b[1], a, n);
    }
    {
#define E(...) #__VA_ARGS__
        SHOW(E(1, 2, 3));
        SHOW(E(1));
#undef E
    }
    {
        for (int i : {INT_MIN, INT_MIN+1, -257, -256, -255, -1, 0, 1, 2, 127, 254, 255, 256, 257, INT_MAX}) {
            SHOW(i, int(clamp_to_uchar(i)));
        }
    }
    {
        SHOW("This is string1.");
        SHOW("status", 9-5);
        SHOW("", 9-3, "string1", 3+1, "", 4-1);
        SHOW("");
        SHOW(9-3);
        SHOW("string");
        SHOW("");
        SHOW("done");
    }
    {
        unsigned col = 12345+g_unoptimized_zero;
        func1(// uchar(col>>0), // Run-Time Check Failure #1 - A cast to a smaller data type has caused a loss of data.
              uchar((col>>0 )&0xFF),
              uchar((col>>8 )&0xFF),
              uchar((col>>16)&0xFF),
              uchar((col>>24)));
    }
    test_exceptions();
    if (1) {
        const int len = 5000;
        string format = string(len, 'H') + "%d%s"; // a long format specifier
        string str = sform_nonliteral(format.c_str(), 123, "hello");
        assertx(str.size()==len+3+5);
        assertx(ends_with(str, "HHHHHHHHHH123hello"));
        //
        // str.resize(17);
        // ssform(str, format.c_str(), 123, "hello");
        // assertx(str.size()==len+3+5);
        // assertx(ends_with(str, "HHHHHHHHHH123hello"));
    }
    if (1) {
        string str;
        for_int(len, 1000) {
            string sref; for_int(i, len) sref += '0'+(i%10);
            const string& str2 = ssform(str, "%s", sref.c_str());
            if (str2!=sref) { SHOW(sref, str2); assertx(str2==sref); }
        }
    }
    if (1) {
        RFile fi("echo ' abc' |"); // test to verify that we read the first space using getline
        string sline;
        assertx(my_getline(fi(), sline));
        assertx(sline==" abc");
    }
    {
        // test std::forward in assertx()
        const int i = assertx(55); assertx(i==55); // constexpr in C++14
        int j = 56;
        const int& j2 = assertx(j); assertx(&j2==&j); assertx(j2==56);
        int& j3 = assertx(j); assertx(&j3==&j); assertx(j3==56);
        unique_ptr<int> p = make_unique<int>(57);
        unique_ptr<int> p2 = std::move(assertx(p)); assertx(*p2==57 && !p);
        unique_ptr<int> p3 = assertx(std::move(p2)); assertx(*p3==57 && !p2);
    }
    if (0) {
        SHOW(HH_ALIGNOF(ArrayView<char>)); // 8 on win, mingw
        SHOW(sizeof(ArrayView<char>));     // 16 on win, mingw
        SHOW(HH_ALIGNOF(Array<char>));     // 8 on win, mingw
        SHOW(sizeof(Array<char>));         // 24 on win; 16 on mingw (pleasant surprise), 12 on clang (32-bit)
        // see ~/src/test/native/bug_size.cpp
    }
}
