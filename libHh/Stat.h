// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_STAT_H_
#define MESH_PROCESSING_LIBHH_STAT_H_

#include <fstream>  // std::ofstream

#include "libHh/Range.h"  // enable_if_range_t<>

#if 0
{
  {
    HH_STAT(Svdeg);
    for_int(i, 10) Svdeg.enter(vdeg[i]);
  }
  HH_SSTAT(Svanum, va.num());
  SHOW(Stat(V(1., 4., 5., 6.)).sdv());
  // getenv_bool("STAT_FILES") : If true, store all entered data values in ./Stat.* files.
  Stat::set_show_stats(-1);  // Or "export SHOW_STATS=-1": only print in showff().
  Stat::set_show_stats(-2);  // Or "export SHOW_STATS=-2": disable printing of all statistics.
  // getenv_bool("HH_HIDE_SUMMARIES") : If true, omit summary of statistics.
}
#endif

namespace hh {

// Accumulate statistics for a stream of arithmetic values.
class Stat {
 public:
  explicit Stat(string pname = "", bool print = false, bool is_static = false);
  explicit Stat(const char* pname, bool print = false, bool is_static = false);
  Stat(Stat&& s) noexcept : _print(false) { swap(*this, s); }  // Not "= default".
  template <typename Range, typename = enable_if_range_t<Range>> explicit Stat(Range&& range);
  ~Stat();
  Stat& operator=(Stat&& s) noexcept;
  void set_name(string name_) { _name = std::move(name_); }
  void set_print(bool print) { _print = print; }
  void set_rms() { _use_rms = true; }  // Show rms instead of sdv.
  void zero();
  void enter(float value);
  void enter(double value);
  void enter(int value) { enter(double(value)); }
  void enter(unsigned value) { enter(double(value)); }
  void enter_multiple(float value, int factor);  // Factor may be negative.
  void remove(float value) { enter_multiple(value, -1); }
  void add(const Stat& st);
  const string& name() const { return _name; }
  int64_t num() const { return _n; }
  int inum() const { return narrow_cast<int>(_n); }
  float min() const { return _min; }
  float max() const { return _max; }
  float avg() const;
  float var() const;  // Sample variance (rather than population variance).
  float sdv() const { return sqrt(var()); }
  float ssd() const;
  float sum() const { return float(_sum); }
  float rms() const;
  float max_abs() const { return std::max(abs(min()), abs(max())); }
  string short_string() const;  // No leading name, no trailing '\n'.
  string name_string() const;   // Used by operator<<().
  friend void swap(Stat& l, Stat& r) noexcept;
  friend std::ostream& operator<<(std::ostream& os, const Stat& st) { return os << st.name_string(); }
  //
  static int show_stats() { return _s_show; }
  static void set_show_stats(int val) { _s_show = val; }

 private:
  string _name;
  bool _print;  // Print statistics in destructor.
  bool _use_rms{false};
  int64_t _n;
  double _sum;
  double _sum2;
  float _min;
  float _max;
  unique_ptr<std::ofstream> _ofs;  // Defined if getenv_bool("STAT_FILES").
  static int _s_show;
  // If add any member variables, be sure to update member function swap().
  friend class Stats;
  void output(float value) const;
  void summary_terminate();
};

template <> HH_DECLARE_OSTREAM_EOL(Stat);

// Like Stat(range), but later specialized to operate on magnitude of Vector4 elements.
template <typename Range, typename = enable_if_range_t<Range>> Stat range_stat(const Range& range);

// Scale and offset a range of values such that they have mean == 0 and sdv == 1.
// This is equivalent to converting each value to its z-score in the distribution.
// Note that this modifies the range in-place, so use standardize(clone(range)) to preserve it.
template <typename Range, typename = enable_if_range_t<Range>> Range standardize(Range&& range);

// Scale a range of values such that they have rms == 1.
// Note that this modifies the range in-place, so use standardize_rms(clone(range)) to preserve it.
template <typename Range, typename = enable_if_range_t<Range>> Range standardize_rms(Range&& range);

#define HH_STAT(S) \
  hh::Stat S { #S, true }
#define HH_STAT_NP(S) \
  hh::Stat S { #S, false }  // No print.

// Static Stat, which gets reported at program termination (or in hh_clean_up()).
#define HH_SSTAT(S, v)                                  \
  do {                                                  \
    static hh::Stat& S = *new hh::Stat(#S, true, true); \
    S.enter(v);                                         \
  } while (false)
#define HH_SSTAT_RMS(S, v)                              \
  do {                                                  \
    static hh::Stat& S = *new hh::Stat(#S, true, true); \
    S.set_rms();                                        \
    S.enter(v);                                         \
  } while (false)

// Range Stat.
#define HH_RSTAT(S, range) \
  do {                     \
    HH_STAT(S);            \
    for (auto e : range) { \
      S.enter(e);          \
    }                      \
  } while (false)
#define HH_RSTAT_RMS(S, range) \
  do {                         \
    HH_STAT(S);                \
    S.set_rms();               \
    for (auto e : range) {     \
      S.enter(e);              \
    }                          \
  } while (false)

//----------------------------------------------------------------------------

template <typename Range, typename> Stat::Stat(Range&& range) : Stat{} {
  for (const auto& e : range) enter(e);
}

inline Stat& Stat::operator=(Stat&& s) noexcept {
  _ofs = nullptr;
  _print = false;
  swap(*this, s);
  return *this;
}

inline void Stat::enter(float value) {
  _n++;
  const double d = value;
  _sum += d;
  _sum2 += square(d);
  if (value < _min) _min = value;
  if (value > _max) _max = value;
  if (_ofs) output(value);
}

inline void Stat::enter(double d) {
  _n++;
  _sum += d;
  _sum2 += square(d);
  const float value = float(d);
  if (value < _min) _min = value;
  if (value > _max) _max = value;
  if (_ofs) output(value);
}

inline void Stat::enter_multiple(float value, int factor) {
  _n += factor;
  const double d = value;
  _sum += d * factor;
  _sum2 += square(d) * factor;
  if (value < _min) _min = value;
  if (value > _max) _max = value;
  if (_ofs) for_int(i, factor) output(value);
}

inline float Stat::avg() const {
  if (!_n) {
    Warning("avg() of empty");
    return 0.f;
  }
  return float(_sum / _n);
}

inline float Stat::var() const {
  if (_n < 2) {
    Warning("Stat::var() of fewer than 2 elements");
    return 0.f;
  }
  return float(std::max((_sum2 - _sum * _sum / _n) / (_n - 1.), 0.));
}

inline float Stat::ssd() const {
  if (!_n) {
    Warning("ssd() of empty");
    return 0.f;
  }
  return float(std::max(_sum2 - _sum * _sum / _n, 0.));
}

inline float Stat::rms() const {
  if (!_n) {
    Warning("rms() of empty");
    return 0.f;
  }
  return sqrt(float(_sum2 / _n));
}

// Specialized in Multigrid.h.
template <typename Range, typename> Stat range_stat(const Range& range) { return Stat(range); }

template <typename Range, typename> Range standardize(Range&& range) {
  Stat stat = range_stat(range);
  const float sdv = stat.sdv();
  if (!sdv) {
    Warning("standardize() of range with zero sdv");
  } else {
    const float avg = stat.avg(), rsdv = 1.f / sdv;
    for (float& e : range) e = (e - avg) * rsdv;
  }
  return std::forward<Range>(range);
}
// Range standardized(const Range& range) { return standardize(clone(range)); }

template <typename Range, typename> Range standardize_rms(Range&& range) {
  Stat stat = range_stat(range);
  const float rms = stat.rms();
  if (!rms) {
    Warning("standardize() of range with zero rms");
  } else {
    const float rrms = 1.f / rms;
    for (auto& e : range) e *= rrms;
  }
  return std::forward<Range>(range);
}
// Range standardized_rms(const Range& range) { return standardize_rms(clone(range)); }

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_STAT_H_
