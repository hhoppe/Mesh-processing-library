// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Stat.h"

#include <vector>

namespace hh {

class Stats {                   // friend of Stat
 public:
    ~Stats()                                    { if (0) flush(); } // unlikely to come before all static ~Stat()
    void flush() {
        if (_vecstat.empty()) return;
        int ntoprint = 0;
        for (Stat* stat : _vecstat) {
            if (stat->_print && stat->num()) ntoprint++;
        }
        if (ntoprint) showdf("Summary of statistics:\n");
        for (Stat* stat : _vecstat) { stat->terminate(); }
        _vecstat.clear();
    }
    std::vector<Stat*> _vecstat; // do not take dependency on Array.h
};

namespace {

class Stats_init {
    Stats* _ptr;                // statically initialized to nullptr
    bool _post_destruct;        // statically initialized to false
 public:
    // Unusual constructors and destructors to be robust even before or after static lifetime.
    Stats* get()                                { return !_ptr && !_post_destruct ? (_ptr = new Stats) : _ptr; }
    ~Stats_init()                               { delete _ptr; _ptr = nullptr; _post_destruct = true; }
} g_pstats;

} // namespace

void flush_stats() { if (Stats* pstats = g_pstats.get()) pstats->flush(); }

static const bool b_stat_files = getenv_bool("STAT_FILES");

Stat::Stat(string pname, bool pprint, bool is_static) : _name(std::move(pname)), _print(pprint) {
    zero();
    if (_name!="" && b_stat_files) {
        Warning("Creating Stat.* files");
        string filename = "Stat." + _name; // the name is assumed ASCII; no need to worry about UTF-8.
        _pofs = make_unique<std::ofstream>(filename);
    }
    if (is_static) {
        if (Stats* pstats = g_pstats.get()) {
            pstats->_vecstat.push_back(this);
        } else {
            static int count = 0;
            if (++count<=2) showf("Stat '%s' terminating beyond lifetime of Stats\n", _name.c_str());
        }
    }
}

Stat::Stat(const char* pname, bool pprint, bool is_static) : Stat(string(pname ? pname : ""), pprint, is_static) { }

void swap(Stat& l, Stat& r) noexcept {
    using std::swap;
    swap(l._name, r._name);
    swap(l._print, r._print);
    swap(l._setrms, r._setrms);
    swap(l._n, r._n);
    swap(l._sum, r._sum);
    swap(l._sum2, r._sum2);
    swap(l._min, r._min);
    swap(l._max, r._max);
    swap(l._pofs, r._pofs);
}

void Stat::terminate() {
    if (_print && num()) showdf("%s", name_string().c_str());
    _print = false;
}

void Stat::zero() {
    assertw(!_pofs);            // just a warning
    _n = 0;
    _sum = _sum2 = 0.;
    _min = BIGFLOAT;    // could be std::numeric_limits<float>::max() or std::numeric_limits<float>::infinity()
    _max = -BIGFLOAT;   // could be std::numeric_limits<float>::min() or -std::numeric_limits<float>::infinity()
}

void Stat::add(const Stat& st) {
    assertw(!_pofs);            // just a warning
    _n += st._n;
    if (st._min<_min) _min = st._min;
    if (st._max>_max) _max = st._max;
    _sum += st._sum;
    _sum2 += st._sum2;
}

string Stat::short_string() const {
    float tavg = _n>0 ? avg() : 0.f, tsdv = _n>1 ? sdv() : 0.f, trms = _n>0 ? rms() : 0.f;
    // (on _WIN32, could also use "(%-7I64d)")
    // Use %14.8g rather than %12.6g because avg and sdv are double-precision.
    long long ln = _n;
    return sform("(%-7lld)%12g:%-12g av=%-14.8g %s=%.8g",
                 ln,
                 _min, _max, tavg,
                 (!_setrms ? "sd" : "rms"), (!_setrms ? tsdv : trms));
}

string Stat::name_string() const {
    // 20130703: changed from %-12.20s and substr(0, 19)
    return (_name=="" ? "" : sform("%-20.28s", (_name.substr(0, 27) + ":").c_str())) + short_string() + "\n";
}

void Stat::output(float f) const {
    (*_pofs) << f << '\n';
}

} // namespace hh
