// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_STAT_H_
#define MESH_PROCESSING_LIBHH_STAT_H_

#include <fstream>              // std::ofstream
#include "Range.h"              // enable_if_range_t<>

#if 0
{
    { HH_STAT(Svdeg); for_int(i, 10) Svdeg.enter(vdeg[i]); }
    HH_SSTAT(Svanum, va.num());
    SHOW(Stat(V(1., 4., 5., 6.)).sdv());
    // getenv_bool("STAT_FILES") -> store all data values in files.
}
#endif

namespace hh {

// Accumulate statistics for a stream of arithmetic values.
class Stat {
 public:
    explicit Stat(string pname = "", bool pprint = false, bool is_static = false);
    explicit Stat(const char* pname, bool pprint = false, bool is_static = false);
    Stat(Stat&& s) noexcept                     : _print(false) { swap(*this, s); } // not default
    template<typename R, typename = enable_if_range_t<R> > explicit Stat(R&& range);
    ~Stat()                                     { terminate(); }
    Stat& operator=(Stat&& s) noexcept          { _pofs.reset(); _print = false; swap(*this, s); return *this; }
    void set_name(string pname)                 { _name = std::move(pname); }
    void set_print(bool pprint)                 { _print = pprint; }
    void set_rms()                              { _setrms = true; } // show rms instead of sdv
    void zero();
    void terminate();
    void enter(float f);
    void enter(double f)                        { enter(static_cast<float>(f)); }
    void enter(int f)                           { enter(static_cast<float>(f)); }
    void enter(unsigned f)                      { enter(static_cast<float>(f)); }
    void enter_multiple(float f, int fac); // fac could be negative
    void remove(float f)                        { enter_multiple(f, -1); }
    void add(const Stat& st);
    const string& name() const                  { return _name; }
    int64_t num() const                         { return _n; }
    int inum() const                            { return narrow_cast<int>(_n); }
    float min() const                           { return _min; }
    float max() const                           { return _max; }
    float avg() const;
    float var() const;                  // sample variance (rather than population variance)
    float sdv() const                           { return sqrt(var()); }
    float ssd() const;
    float sum() const                           { return static_cast<float>(_sum); }
    float rms() const;
    float max_abs() const                       { return std::max(abs(min()), abs(max())); }
    string short_string() const; // no leading name, no trailing '\n'
    string name_string() const;  // operator<<() uses namestring format
    friend void swap(Stat& l, Stat& r) noexcept;
    friend std::ostream& operator<<(std::ostream& os, const Stat& st) { return os << st.name_string(); }
 private:
    string _name;
    bool _print;                // print statistics in destructor
    bool _setrms {false};
    int64_t _n;
    double _sum;
    double _sum2;
    float _min;
    float _max;
    unique_ptr<std::ofstream> _pofs; // if getenv_bool("STAT_FILES")
    // if add any member variables, be sure to update swap()
    friend class Stats;
    void output(float f) const;
};

template<> HH_DECLARE_OSTREAM_EOL(Stat);

// Like Stat(range), but later specialized to operate on magnitude of Vector4 elements.
template<typename R, typename = enable_if_range_t<R> > Stat range_stat(const R& range);

// Scale and offset a range of values such that their mean==0 and sdv==1.
// This is equivalent to converting each value to its z-score in the distribution.
// Note that this modifies the range in-place, so use standardize(clone(range)) to preserve it.
template<typename R, typename = enable_if_range_t<R> > R standardize(R&& range);

// Scale a range of values such that their rms==1.
// Note that this modifies the range in-place, so use standardize_rms(clone(range)) to preserve it.
template<typename R, typename = enable_if_range_t<R> > R standardize_rms(R&& range);

#define HH_STAT(S) hh::Stat S{#S, true}
#define HH_STATNP(S) hh::Stat S{#S, false} // no print
#define HH_SSTAT(S, v) do { static hh::Stat S(#S, true, true); S.enter(v); } while (false) // static Stat
#define HH_SSTAT_RMS(S, v) do { static hh::Stat S(#S, true, true); S.set_rms(); S.enter(v); } while (false)
#define HH_RSTAT(S, range) do { HH_STAT(S); for (auto e : range) { S.enter(e); } } while (false) // range Stat
#define HH_RSTAT_RMS(S, range) do { HH_STAT(S); S.set_rms(); for (auto e : range) { S.enter(e); } } while (false)


//----------------------------------------------------------------------------

template<typename R, typename> Stat::Stat(R&& range) : Stat{} {
    for (const auto& e : range) { enter(e); }
}

inline void Stat::enter(float f) {
    _n++; _sum += static_cast<double>(f); _sum2 += square(static_cast<double>(f));
    if (f<_min) _min = f;
    if (f>_max) _max = f;
    if (_pofs) output(f);
}

inline void Stat::enter_multiple(float f, int fac) {
    _n += fac; _sum += static_cast<double>(f)*fac; _sum2 += square(static_cast<double>(f))*fac;
    if (f<_min) _min = f;
    if (f>_max) _max = f;
    if (_pofs) for_int(i, fac) { output(f); }
}

inline float Stat::avg() const {
    if (!_n) { Warning("avg() of empty"); return 0.f; }
    return static_cast<float>(_sum/static_cast<double>(_n));
}

inline float Stat::var() const {
    if (_n<2) { Warning("Stat::var() of fewer than 2 elements"); return 0.f; }
    return static_cast<float>(std::max((_sum2-_sum*_sum/static_cast<double>(_n))/(_n-1.), 0.));
}

inline float Stat::ssd() const {
    if (!_n) { Warning("ssd() of empty"); return 0.f; }
    return static_cast<float>(std::max(_sum2-_sum*_sum/static_cast<double>(_n), 0.));
}

inline float Stat::rms() const {
    if (!_n) { Warning("rms() of empty"); return 0.f; }
    return sqrt(static_cast<float>(_sum2/static_cast<double>(_n)));
}

template<typename R, typename> Stat range_stat(const R& range) {
    Stat stat; for (auto e : range) { stat.enter(e); } return stat;
}

template<typename R, typename> R standardize(R&& range) {
    Stat stat = range_stat(range);
    const float sdv = stat.sdv();
    if (!sdv) {
        Warning("standardize() of range with zero sdv");
    } else {
        const float avg = stat.avg(), rsdv = 1.f/sdv;
        for (float& e : range) { e = (e-avg)*rsdv; }
    }
    return std::forward<R>(range);
}
// R standardized(const R& range) { return standardize(clone(range)); }

template<typename R, typename> R standardize_rms(R&& range) {
    Stat stat = range_stat(range);
    const float rms = stat.rms();
    if (!rms) {
        Warning("standardize() of range with zero rms");
    } else {
        const float rrms = 1.f/rms;
        for (auto& e : range) { e *= rrms; }
    }
    return std::forward<R>(range);
}
// R standardized_rms(const R& range) { return standardize_rms(clone(range)); }

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_STAT_H_
