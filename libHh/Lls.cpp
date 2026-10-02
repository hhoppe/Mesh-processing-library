// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Lls.h"

#include "libHh/MatrixOp.h"  // mat_mul()
#include "libHh/RangeOp.h"
#include "libHh/SingularValueDecomposition.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"
#include "libHh/my_lapack.h"

namespace hh {

static const int sdebug = getenv_int("LLS_DEBUG");  // 0, 1, or 2.

// *** Factory: select an Lls implementation based on environment overrides and problem size/sparsity

unique_ptr<Lls> Lls::make(int m, int n, int nd, float nonzerofrac) {
  if (getenv_bool("SPARSE_LLS")) return Warning("Using SparseLls"), make_unique<SparseLls>(m, n, nd);
  if (getenv_bool("LUD_LLS")) return Warning("Using LudLls"), make_unique<LudLls>(m, n, nd);
  if (getenv_bool("GIVENS_LLS")) return Warning("Using GivensLls"), make_unique<GivensLls>(m, n, nd);
  if (getenv_bool("SVD_LLS")) return Warning("Using SvdLls"), make_unique<SvdLls>(m, n, nd);
  if (getenv_bool("SVD_DOUBLE_LLS")) return Warning("Using SvdDoubleLls"), make_unique<SvdDoubleLls>(m, n, nd);
  if (getenv_bool("QRD_LLS")) return Warning("Using QrdLls"), make_unique<QrdLls>(m, n, nd);
  const int64_t size = int64_t{m} * n;
  if (size >= 1000 * 40 && nonzerofrac < .3f) return make_unique<SparseLls>(m, n, nd);
  return make_unique<QrdLls>(m, n, nd);
}

// *** Lls

Lls::Lls(int m, int n, int nd) : _m(m), _n(n), _nd(nd), _b(nd, m), _x(nd, n) {
  assertx(_m > 0 && _n > 0 && _nd > 0 && _m >= _n);
  Lls::clear();
}

void Lls::clear() {
  fill(_b, 0.f);
  fill(_x, 0.f);
  _solved = false;
}

void Lls::enter_a(CMatrixView<float> mat) {
  assertx(mat.ysize() == _m && mat.xsize() == _n);
  for_int(r, _m) enter_a_r(r, mat[r]);
}

void Lls::enter_b(CMatrixView<float> mat) {
  assertx(mat.ysize() == _m && mat.xsize() == _nd);
  for_int(r, _m) enter_b_r(r, mat[r]);
}

void Lls::enter_xest(CMatrixView<float> mat) {
  assertx(mat.ysize() == _n && mat.xsize() == _nd);
  for_int(r, _n) enter_xest_r(r, mat[r]);
}

void Lls::get_x(MatrixView<float> mat) {
  assertx(mat.ysize() == _n && mat.xsize() == _nd);
  for_int(r, _n) get_x_r(r, mat[r]);
}

// *** SparseLls

void SparseLls::clear() {
  _entries.clear();
  _row_start.clear(), _col_start.clear();
  _row_ivals.clear(), _col_ivals.clear();
  Lls::clear();
}

void SparseLls::enter_a_rc(int r, int c, float val) {
  ASSERTX(r >= 0 && r < _m && c >= 0 && c < _n);
  _entries.push(Entry{r, c, val});
}

void SparseLls::enter_a_r(int r, CArrayView<float> ar) {
  ASSERTX(ar.num() == _n);
  for_int(c, _n) {
    if (ar[c]) enter_a_rc(r, c, ar[c]);
  }
}

void SparseLls::enter_a_c(int c, CArrayView<float> ar) {
  ASSERTX(ar.num() == _m);
  for_int(r, _m) {
    if (ar[r]) enter_a_rc(r, c, ar[r]);
  }
}

// Sort the entries into the row-major and column-major arrays, using a counting sort, and release the entries.
void SparseLls::compress() {
  _row_start.init(_m + 1, 0), _col_start.init(_n + 1, 0);
  for (const Entry& e : _entries) _row_start[e._r + 1]++, _col_start[e._c + 1]++;
  for_int(i, _m) _row_start[i + 1] += _row_start[i];
  for_int(j, _n) _col_start[j + 1] += _col_start[j];
  _row_ivals.init(_entries.num()), _col_ivals.init(_entries.num());
  Array<int> row_pos(_row_start.head(_m)), col_pos(_col_start.head(_n));
  for (const Entry& e : _entries) {
    _row_ivals[row_pos[e._r]++] = Ival{e._c, e._v};
    _col_ivals[col_pos[e._c]++] = Ival{e._r, e._v};
  }
  _entries.clear();  // (This also deallocates its memory.)
}

Array<float> SparseLls::mult_m_v(CArrayView<float> vi) const {
  Array<float> vo(_m);
  // vo[m] = _a[m, n]*vi[n];
  for_int(i, _m) {
    double sum = 0.;
    for_intL(k, _row_start[i], _row_start[i + 1]) sum += double(_row_ivals[k]._v) * vi[_row_ivals[k]._i];
    vo[i] = float(sum);
  }
  return vo;
}

Array<float> SparseLls::mult_mt_v(CArrayView<float> vi) const {
  Array<float> vo(_n);
  // vo[n] = uT[n, m]*vi[m];
  for_int(j, _n) {
    double sum = 0.;
    for_intL(k, _col_start[j], _col_start[j + 1]) sum += double(_col_ivals[k]._v) * vi[_col_ivals[k]._i];
    vo[j] = float(sum);
  }
  return vo;
}

bool SparseLls::do_cg(ArrayView<float> x, CArrayView<float> h, double* prssb, double* prssa) const {
  // x(_n), h(_m)
  Array<float> rc, gc, gp, dc, tc;
  rc = mult_m_v(x);
  for_int(i, _m) rc[i] -= h[i];
  const double rssb = mag2(rc);
  gc = mult_mt_v(rc);
  for_int(i, _n) gc[i] = -gc[i];
  const int fudge_for_small_systems = 20;
  const int kmax = _n + fudge_for_small_systems;
  float gm2;
  int k;
  for (k = 0;; k++) {
    gm2 = float(mag2(gc));
    if (sdebug >= 2) showf("k=%-4d gm2=%g\n", k, gm2);
    if (gm2 < _tolerance) break;
    if (k == _max_iter) break;
    if (k == kmax) break;
    if (k > 0) {
      const float bi = gm2 / float(mag2(gp));
      for_int(i, _n) dc[i] = gc[i] + bi * dc[i];
    } else {
      dc = gc;
    }
    tc = mult_m_v(dc);
    const float ai = gm2 / float(mag2(tc));
    for_int(i, _n) x[i] += ai * dc[i];
    for_int(i, _m) rc[i] += ai * tc[i];
    gp = gc;
    gc = mult_mt_v(rc);
    for_int(i, _n) gc[i] = -gc[i];
  }
  const double rssa = mag2(rc);
  // Print final gradient norm squared and final residual norm squared.
  if (sdebug || _verb) showf("CG: %d iter (gm2=%.10g, rssb=%.10g, rssa=%.10g)\n", k, gm2, rssb, rssa);
  if (prssb) *prssb += rssb;
  if (prssa) *prssa += rssa;
  return gm2 < _tolerance;
}

bool SparseLls::solve(double* prssb, double* prssa) {
  auto up_timer = _verb ? make_unique<Timer>("_____SparseLls", Timer::EMode::abbrev) : nullptr;
  assertx(!_solved);
  _solved = true;
  if (sdebug) showf("SparseLls: solving %dx%d system, nonzerofrac=%f\n", _m, _n, float(_entries.num()) / _m / _n);
  compress();
  if (prssb) *prssb = 0.;
  if (prssa) *prssa = 0.;
  Array<float> x(_n), rhv(_m);
  bool success = true;
  for_int(di, _nd) {
    for_int(i, _m) rhv[i] = _b[di, i];
    for_int(j, _n) x[j] = _x[di, j];
    if (!do_cg(x, rhv, prssb, prssa)) {
      success = false;
      if (0 && _max_iter == std::numeric_limits<int>::max()) continue;
    }
    for_int(j, _n) _x[di, j] = x[j];
  }
  return success;
}

void SparseLls::set_tolerance(float tolerance) { _tolerance = tolerance; }

void SparseLls::set_max_iter(int max_iter) { _max_iter = max_iter; }

void SparseLls::set_verbose(int verb) { _verb = verb; }

// *** FullLls

void FullLls::clear() {
  fill(_a, 0.f);
  Lls::clear();
}

bool FullLls::solve(double* prssb, double* prssa) {
  assertx(_m >= _n);
  assertx(!_solved);
  _solved = true;
  if (prssb) *prssb = get_rss(_a, _b);
  // Some solve_aux() overwrite _a and _b (GivensLls, and LudLls if _m == _n), so keep copies for the final residual.
  Matrix<float> a, b;
  if (prssa) a = _a, b = _b;
  const bool success = solve_aux();
  if (prssa) *prssa = get_rss(a, b);
  return success;
}

double FullLls::get_rss(CMatrixView<float> a, CMatrixView<float> b) const {
  double rss = 0.;
  for_int(di, _nd) for_int(i, _m) rss += square(dot(a[i], _x[di]) - b[di, i]);
  return rss;
}

// *** LudLls

bool LudLls::solve_aux() {
  auto up_ta = _m != _n ? make_unique<Matrix<float>>(_n, _n) : nullptr;
  MatrixView<float> a = up_ta ? *up_ta : _a;
  if (_m != _n) {
    for_int(i, _n) {
      for_int(j, _n) {
        double s = 0.;
        for_int(k, _m) s += double(_a[k, i]) * _a[k, j];
        a[i, j] = float(s);
      }
    }
  }
  Array<int> rindx(_n);
  Array<float> t(_n);
  for_int(i, _n) {
    float vmax = 0.f;
    for_int(j, _n) {
      const float v = abs(a[i, j]);
      if (v > vmax) vmax = v;
    }
    if (!vmax) return false;
    t[i] = 1.f / vmax;
  }
  int imax = 0;  // Undefined.
  for_int(j, _n) {
    for_int(i, j) {
      double s = a[i, j];
      for_int(k, i) s -= double(a[i, k]) * a[k, j];
      a[i, j] = float(s);
    }
    float vmax = 0.f;
    for_intL(i, j, _n) {
      double s = a[i, j];
      for_int(k, j) s -= double(a[i, k]) * a[k, j];
      a[i, j] = float(s);
      const float v = t[i] * abs(a[i, j]);
      if (v >= vmax) {
        vmax = v;
        imax = i;
      }
    }
    if (imax != j) {
      swap_elements(a[imax], a[j]);
      t[imax] = t[j];
    }
    rindx[j] = imax;
    if (!a[j, j]) return false;
    if (j < _n - 1) {
      const float v = 1.f / a[j, j];
      for_intL(i, j + 1, _n) a[i, j] *= v;
    }
  }
  for_int(di, _nd) {
    if (_m == _n) {
      for_int(j, _n) t[j] = _b[di, j];
    } else {
      for_int(j, _n) {
        double s = 0.;
        for_int(i, _m) s += double(_a[i, j]) * _b[di, i];
        t[j] = float(s);
      }
    }
    int ii = -1;
    for_int(i, _n) {
      const int ip = rindx[i];
      double s = t[ip];
      t[ip] = t[i];
      if (ii >= 0) {
        for_intL(j, ii, i) s -= double(a[i, j]) * t[j];
      } else if (s) {
        ii = i;
      }
      t[i] = float(s);
    }
    for (int i = _n - 1; i >= 0; --i) {
      double s = t[i];
      for_intL(j, i + 1, _n) s -= double(a[i, j]) * t[j];
      t[i] = float(s / a[i, i]);
    }
    for_int(j, _n) _x[di, j] = t[j];
  }
  return true;
}

constexpr float k_float_cond_warning = 1e4f;
constexpr float k_float_cond_max = 1e5f;

constexpr double k_double_cond_warning = 1e8;
constexpr double k_double_cond_max = 1e12;

// *** GivensLls

// Rejecting an ill-conditioned system, as SvdLls does, is disabled.  The test below compares the diagonal elements of
// the triangular factor R, whose ratio max|r_ii| / min|r_ii| is a lower bound on the condition number.  Without it,
// GivensLls fails only on an exactly zero diagonal element, so it "succeeds" on a rank-deficient system (e.g., one
// with two equal columns) and returns a huge solution.  Enabling it might reject systems that callers now "solve".
constexpr bool k_givens_reject_ill_conditioned = false;

bool GivensLls::solve_aux() {
  int nposs = 0, ngivens = 0;
  for_int(i, _n) {
    for_intL(k, i + 1, _m) {
      nposs++;
      if (!_a[k, i]) continue;
      ngivens++;
      const float xi = _a[i, i];
      const float xk = _a[k, i];
      float c, s;
      if (abs(xk) > abs(xi)) {
        const float t = xi / xk;
        s = 1.f / sqrt(1.f + square(t));
        c = s * t;
      } else {
        const float t = xk / xi;
        c = 1.f / sqrt(1.f + square(t));
        s = c * t;
      }
      for_intL(j, i, _n) {
        const float xij = _a[i, j], xkj = _a[k, j];
        _a[i, j] = c * xij + s * xkj;
        _a[k, j] = -s * xij + c * xkj;
      }
      for_int(di, _nd) {
        const float xij = _b[di, i], xkj = _b[di, k];
        _b[di, i] = c * xij + s * xkj;
        _b[di, k] = -s * xij + c * xkj;
      }
    }
    if (!_a[i, i]) {
      Warning("GivensLls solution fails");
      return false;
    }
  }
  if (sdebug) showf("Givens: %d/%d rotations done\n", ngivens, nposs);
  if constexpr (k_givens_reject_ill_conditioned) {
    float rmin = BIGFLOAT, rmax = 0.f;
    for_int(i, _n) rmin = min(rmin, abs(_a[i, i])), rmax = max(rmax, abs(_a[i, i]));
    if (rmin <= rmax / k_float_cond_max) return false;  // Rank-deficient or ill-conditioned.
  }
  // Backsubstitutions.
  for_int(di, _nd) {
    for (int i = _n - 1; i >= 0; --i) {
      float sum = _b[di, i];
      for_intL(j, i + 1, _n) sum -= _a[i, j] * _b[di, j];
      _b[di, i] = sum / _a[i, i];
      _x[di, i] = _b[di, i];
    }
  }
  return true;
}

#if defined(HH_HAVE_LAPACK)  // ***

namespace {

inline int work_size(int m, int n) {
  return max(n * 67, m * 8) + 128;  // Picked somewhat arbitrarily.
}

}  // namespace

// *** SvdLls

SvdLls::SvdLls(int m, int n, int nd)
    : FullLls(m, n, nd), _fa(_m * _n), _fb(_m * _nd), _s(_n), _work(work_size(_m, _n)) {}

bool SvdLls::solve_aux() {
  {
    float* ap = _fa.data();
    for_int(j, _n) for_int(i, _m) { *ap++ = _a[i, j]; }
  }
  {
    float* bp = _fb.data();
    for_int(d, _nd) for_int(i, _m) { *bp++ = _b[d, i]; }
  }
  float rcond = 1.f / k_float_cond_max;
  if (0) rcond = -1.f;  // Use machine precision to determine the rank.
  lapack_int irank, lwork = _work.num(), info;
  lapack_int lm = _m, ln = _n, lnd = _nd;
  sgelss_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, _s.data(), &rcond, &irank, _work.data(), &lwork, &info);
  assertx(info >= 0);
  if (info > 0) return false;
  if (irank < _n) return false;
  // _s contains singular values in decreasing order.
  assertx(_s[_n - 1]);
  float cond = _s[0] / _s[_n - 1];
  if (cond > k_float_cond_warning) HH_SSTAT(Ssvdlls_cond, cond);
  {
    float* bp = _fb.data();
    for_int(d, _nd) {
      for_int(j, _n) _x[d, j] = *bp++;
      bp += _m - _n;
    }
  }
  return true;
}

// *** SvdDoubleLls

SvdDoubleLls::SvdDoubleLls(int m, int n, int nd)
    : FullLls(m, n, nd), _fa(_m * _n), _fb(_m * _nd), _s(_n), _work(work_size(_m, _n)) {}

bool SvdDoubleLls::solve_aux() {
  {
    double* ap = _fa.data();
    for_int(j, _n) for_int(i, _m) { *ap++ = _a[i, j]; }
  }
  {
    double* bp = _fb.data();
    for_int(d, _nd) for_int(i, _m) { *bp++ = _b[d, i]; }
  }
  double rcond = 1. / k_double_cond_max;
  if (0) rcond = -1.;  // Use machine precision to determine the rank.
  lapack_int irank, lwork = _work.num(), info;
  lapack_int lm = _m, ln = _n, lnd = _nd;
  dgelss_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, _s.data(), &rcond, &irank, _work.data(), &lwork, &info);
  assertx(info >= 0);
  if (info > 0) return false;
  if (irank < _n) return false;
  // _s contains singular values in decreasing order.
  assertx(_s[_n - 1]);
  double cond = _s[0] / _s[_n - 1];
  if (cond > k_double_cond_warning) HH_SSTAT(Ssvdlls_cond, cond);
  {
    double* bp = _fb.data();
    for_int(d, _nd) {
      for_int(j, _n) _x[d, j] = float(*bp++);
      bp += _m - _n;
    }
  }
  return true;
}

// *** QrdLls

QrdLls::QrdLls(int m, int n, int nd)
    : FullLls(m, n, nd),
      _fa(_m * _n),
      _fb(_m * _nd)
// _work(max(4 * _n, 2 * _n + _nd)) was for old sgelsx
{}

bool QrdLls::solve_aux() {
  {
    float* ap = _fa.data();
    for_int(j, _n) for_int(i, _m) { *ap++ = _a[i, j]; }
  }
  {
    float* bp = _fb.data();
    for_int(d, _nd) for_int(i, _m) { *bp++ = _b[d, i]; }
  }
  Array<lapack_int> jpvt(_n);
  fill(jpvt, 0);  // All columns free to pivot.
  float rcond = 1.f / k_float_cond_max;
  lapack_int irank, info;
  lapack_int lm = _m, ln = _n, lnd = _nd;
  // sgelsx_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, jpvt.data(), &rcond, &irank, _work.data(), &info);
  _work.init(1, 0);
  lapack_int lwork = -1;
  info = 0;  // Query.
  sgelsy_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, jpvt.data(), &rcond, &irank, _work.data(), &lwork,
          &info);  // Query the optimal work size.
  assertx(info == 0);
  _work.init(int(_work[0]));
  lwork = _work.num();
  sgelsy_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, jpvt.data(), &rcond, &irank, _work.data(), &lwork, &info);
  assertx(info == 0);
  if (irank < _n) return false;
  // If need to use jpvt, remember that fortran indices start at 1.
  {
    float* bp = _fb.data();
    for_int(d, _nd) {
      for_int(j, _n) _x[d, j] = *bp++;
      bp += _m - _n;
    }
  }
  return true;
}

#else  // *** !defined(HH_HAVE_LAPACK)

// *** SvdLls

SvdLls::SvdLls(int m, int n, int nd) : FullLls(m, n, nd), _work(n), _mU(m, n), _mS(_n), _mVT(_n, _n) {}

bool SvdLls::solve_aux() {
  if (!singular_value_decomposition(_a, _mU, _mS, _mVT)) return false;
  sort_singular_values(_mU, _mS, _mVT);
  // As in the LAPACK version, a singular value counts as zero unless it exceeds rcond times the largest one.
  const float rcond = 1.f / k_float_cond_max;
  if (_mS.last() <= rcond * _mS[0]) return false;  // Rank-deficient.
  const float cond = _mS[0] / _mS.last();
  if (cond > k_float_cond_warning) HH_SSTAT(Ssvdlls_cond, cond);
  // SHOW(_a, _b, _mU, _mS, _mVT);
  // SHOW(mat_mul(mat_mul(_mU, diag_mat(_mS)), transpose(_mVT)));
  for (float& s : _mS) s = 1.f / s;
  for_int(d, _nd) {
    // _x[d].assign(mat_mul(_mVT, mat_mul(_b[d], _mU) * _mS));
    mat_mul(_b[d], _mU, _work);
    _work *= _mS;
    mat_mul(_mVT, _work, _x[d]);
  }
  return true;
}

SvdDoubleLls::SvdDoubleLls(int m, int n, int nd) : FullLls(m, n, nd), _mU(m, n), _mS(_n), _mVT(_n, _n) {}

// Rejecting a rank-deficient system, as in the LAPACK version (where a singular value counts as zero unless it exceeds
// rcond times the largest one), is disabled.  MeshSimplify solves quadric systems with condition numbers up to about
// 1e19, which this version has always accepted; with the test enabled, the MeshSimplify demo instead reports 8416
// times that it is "Assuming current attributes are just fine".
constexpr bool k_svd_double_reject_rank_deficient = false;

bool SvdDoubleLls::solve_aux() {
  const Matrix<double> A = convert<double>(_a);
  if (!singular_value_decomposition(A, _mU, _mS, _mVT)) return false;
  sort_singular_values(_mU, _mS, _mVT);
  if (!_mS.last()) return false;
  if constexpr (k_svd_double_reject_rank_deficient)
    if (_mS.last() <= _mS[0] / k_double_cond_max) return false;  // Rank-deficient.
  const double cond = _mS[0] / _mS.last();
  if (cond > k_double_cond_warning) HH_SSTAT(Ssvdlls_cond, cond);
  for (double& s : _mS) s = 1. / s;
  for_int(d, _nd) {
    _x[d].assign(convert<float>(mat_mul(_mVT, mat_mul(convert<double>(_b[d]), _mU) * _mS)));  // Slow.
  }
  return true;
}

QrdLls::QrdLls(int m, int n, int nd) : FullLls(m, n, nd), _work(n), _mU(m, n), _mS(_n), _mVT(_n, _n) {}  // == SvdLls

bool QrdLls::solve_aux() {
  if (!singular_value_decomposition(_a, _mU, _mS, _mVT)) return false;
  sort_singular_values(_mU, _mS, _mVT);
  // As in the LAPACK version, a singular value counts as zero unless it exceeds rcond times the largest one.
  const float rcond = 1.f / k_float_cond_max;
  if (_mS.last() <= rcond * _mS[0]) return false;  // Rank-deficient.
  const float cond = _mS[0] / _mS.last();
  if (cond > k_float_cond_warning) HH_SSTAT(Ssvdlls_cond, cond);
  for (float& s : _mS) s = 1.f / s;
  for_int(d, _nd) {
    // _x[d].assign(mat_mul(_mVT, mat_mul(_b[d], _mU) * _mS));
    mat_mul(_b[d], _mU, _work);
    _work *= _mS;
    mat_mul(_mVT, _work, _x[d]);
  }
  return true;
}

#endif  // defined(HH_HAVE_LAPACK)

}  // namespace hh
