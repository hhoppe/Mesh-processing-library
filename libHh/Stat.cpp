// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Stat.h"

#include <vector>

namespace hh {

int Stat::_s_show = -10;

class Stats {
 public:
  static void add(Stat* stat) { instance()._vec.push_back(stat); }
  static void flush() { instance().flush_internal(); }

 private:
  static Stats& instance() {
    static Stats& stats = *new Stats;
    return stats;
  }
  Stats() { hh_at_clean_up(Stats::flush); }
  ~Stats() = delete;
  void flush_internal() {
    if (_vec.empty()) return;
    int ntoprint = 0;
    for (Stat* stat : _vec) {
      if (stat->_print && stat->num()) ntoprint++;
    }
    if (ntoprint) {
      if (Stat::_s_show == -1) {
        showff("Summary of statistics:\n");
      } else {
        showdf("Summary of statistics:\n");
      }
    }
    for (Stat* stat : _vec) stat->terminate();
    _vec.clear();
  }
  std::vector<Stat*> _vec;
};

Stat::Stat(string pname, bool print, bool is_static) : _name(std::move(pname)), _print(print) {
  zero();
  static const bool stat_files = getenv_bool("STAT_FILES");
  if (_name != "" && stat_files) {
    Warning("Creating Stat.* files");
    string filename = "Stat." + _name;  // the name is assumed ASCII; no need to worry about UTF-8.
    _pofs = make_unique<std::ofstream>(filename);
  }
  if (_s_show == -10) _s_show = getenv_int("SHOW_STATS");
  if (_s_show <= -2) {
    _print = false;
  } else if (is_static) {
    Stats::add(this);
  }
}

Stat::Stat(const char* pname, bool print, bool is_static) : Stat(string(pname ? pname : ""), print, is_static) {}

Stat::~Stat() { terminate(); }

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
  if (_print && num()) {
    if (_s_show == -1) {
      showff("%s", name_string().c_str());
    } else {
      showdf("%s", name_string().c_str());
    }
  }
  _print = false;
}

void Stat::zero() {
  assertw(!_pofs);  // just a warning
  _n = 0;
  _sum = _sum2 = 0.;
  _min = BIGFLOAT;   // could be std::numeric_limits<float>::max() or std::numeric_limits<float>::infinity()
  _max = -BIGFLOAT;  // could be std::numeric_limits<float>::lowest() or -std::numeric_limits<float>::infinity()
}

void Stat::add(const Stat& st) {
  assertw(!_pofs);  // just a warning
  _n += st._n;
  _sum += st._sum;
  _sum2 += st._sum2;
  if (st._min < _min) _min = st._min;
  if (st._max > _max) _max = st._max;
}

string Stat::short_string() const {
  const double d_avg = _n > 0 ? _sum / _n : 0.;  // Higher precision than avg().
  const double d_sdv = _n > 1 ? sqrt(std::max((_sum2 - _sum * _sum / _n) / (_n - 1.), 0.)) : 0.;
  const double d_rms = _n > 0 ? sqrt(_sum2 / _n) : 0.;
  // (on _WIN32, could also use "(%-7I64d)")
  // Use %14.8g rather than %12.6g because d_avg and d_sdv are double-precision.
  long long ln = _n;
  return sform("(%-7lld)%12g:%-12g av=%-14.8g %s=%.8g", ln, _min, _max, d_avg, (!_setrms ? "sd" : "rms"),
               (!_setrms ? d_sdv : d_rms));
}

string Stat::name_string() const {
  // 20130703: changed from %-12.20s and substr(0, 19)
  return (_name == "" ? "" : sform("%-20.28s", (_name.substr(0, 27) + ":").c_str())) + short_string() + "\n";
}

void Stat::output(float f) const { (*_pofs) << f << '\n'; }

}  // namespace hh
