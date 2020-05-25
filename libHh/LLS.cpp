// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "LLS.h"

#include "Timer.h"
#include "RangeOp.h"
#include "Stat.h"
#include "SingularValueDecomposition.h"
#include "MatrixOp.h"           // mat_mul()

#if !defined(HH_NO_LAPACK)
#include "my_lapack.h"
#endif

namespace hh {

static const int sdebug = getenv_int("LLS_DEBUG"); // 0, 1, or 2

// *** virtual constructor

unique_ptr<LLS> LLS::make(int m, int n, int nd, float nonzerofrac) {
    if (getenv_bool("SPARSE_LLS")) {
        Warning("Using SparseLLS");
        return make_unique<SparseLLS>(m, n, nd);
    }
    if (getenv_bool("LUD_LLS")) {
        Warning("Using LudLLS");
        return make_unique<LudLLS>(m, n, nd);
    }
    if (getenv_bool("GIVENS_LLS")) {
        Warning("Using GivensLLS");
        return make_unique<GivensLLS>(m, n, nd);
    }
    if (getenv_bool("SVD_LLS")) {
        Warning("Using SvdLLS");
        return make_unique<SvdLLS>(m, n, nd);
    }
    if (getenv_bool("SVD_DOUBLE_LLS")) {
        Warning("Using SvdDoubleLLS");
        return make_unique<SvdDoubleLLS>(m, n, nd);
    }
    if (getenv_bool("QRD_LLS")) {
        Warning("Using QrdLLS");
        return make_unique<QrdLLS>(m, n, nd);
    }
    int64_t size = int64_t{m}*n;
    if (size<1000*40) {         // small system
        return make_unique<QrdLLS>(m, n, nd);
    } else {                    // large system
        if (nonzerofrac<.3f) return make_unique<SparseLLS>(m, n, nd);
        return make_unique<QrdLLS>(m, n, nd);
    }
}

// *** LLS

LLS::LLS(int m, int n, int nd) : _m(m), _n(n), _nd(nd), _b(nd, m), _x(nd, n) {
    assertx(_m>0 && _n>0 && _nd>0 && _m>=_n);
    clear();
}

void LLS::clear() {
    fill(_b, 0.f); fill(_x, 0.f); _solved = false;
}

void LLS::enter_a(CMatrixView<float> mat) {
    assertx(mat.ysize()==_m && mat.xsize()==_n);
    for_int(r, _m) { enter_a_r(r, mat[r]); }
}

void LLS::enter_b(CMatrixView<float> mat) {
    assertx(mat.ysize()==_m && mat.xsize()==_nd);
    for_int(r, _m) { enter_b_r(r, mat[r]); }
}

void LLS::enter_xest(CMatrixView<float> mat) {
    assertx(mat.ysize()==_n && mat.xsize()==_nd);
    for_int(r, _n) { enter_xest_r(r, mat[r]); }
}

void LLS::get_x(MatrixView<float> mat) {
    assertx(mat.ysize()==_n && mat.xsize()==_nd);
    for_int(r, _n) { get_x_r(r, mat[r]); }
}

// *** SparseLLS

void SparseLLS::clear() {
    _rows.clear();
    _cols.clear();
    LLS::clear();
}

void SparseLLS::enter_a_rc(int r, int c, float val) {
    _rows[r].push(Ival(c, val));
    _cols[c].push(Ival(r, val));
    _nentries++;
}

void SparseLLS::enter_a_r(int r, CArrayView<float> ar) {
    ASSERTX(ar.num()==_n);
    for_int(c, _n) { if (ar[c]) enter_a_rc(r, c, ar[c]); }
}

void SparseLLS::enter_a_c(int c, CArrayView<float> ar) {
    ASSERTX(ar.num()==_m);
    for_int(r, _m) { if (ar[r]) enter_a_rc(r, c, ar[r]); }
}

Array<float> SparseLLS::mult_m_v(CArrayView<float> vi) const {
    Array<float> vo(_m);
    // vo[m] = _a[m][n]*vi[n];
    for_int(i, _m) {
        double sum = 0.;
        for (const Ival& ival : _rows[i]) { sum += double(ival._v)*vi[ival._i]; }
        vo[i] = float(sum);
    }
    return vo;
}

Array<float> SparseLLS::mult_mt_v(CArrayView<float> vi) const {
    Array<float> vo(_n);
    // vo[n] = uT[n][m]*vi[m];
    for_int(j, _n) {
        double sum = 0.;
        for (const Ival& ival : _cols[j]) { sum += double(ival._v)*vi[ival._i]; }
        vo[j] = float(sum);
    }
    return vo;
}

bool SparseLLS::do_cg(Array<float>& x, CArrayView<float> h, double* prssb, double* prssa) const {
    // x(_n), h(_m)
    Array<float> rc, gc, gp, dc, tc;
    rc = mult_m_v(x);
    for_int(i, _m) rc[i] -= h[i];
    double rssb = mag2(rc);
    gc = mult_mt_v(rc);
    for_int(i, _n) gc[i] = -gc[i];
    const int fudge_for_small_systems = 20;
    const int kmax = _n+fudge_for_small_systems;
    float gm2;
    int k;
    for (k = 0; ; k++) {
        gm2 = float(mag2(gc));
        if (sdebug>=2) showf("k=%-4d gm2=%g\n", k, gm2);
        if (gm2<_tolerance) break;
        if (k==_max_iter) break;
        if (k==kmax) break;
        if (k>0) {
            float bi = gm2/float(mag2(gp));
            for_int(i, _n) dc[i] = gc[i]+bi*dc[i];
        } else {
            dc = gc;
        }
        tc = mult_m_v(dc);
        float ai = gm2/float(mag2(tc));
        for_int(i, _n) x[i] += ai*dc[i];
        for_int(i, _m) rc[i] += ai*tc[i];
        gp = gc;
        gc = mult_mt_v(rc);
        for_int(i, _n) gc[i] = -gc[i];
    }
    double rssa = mag2(rc);
    // Print final gradient norm squared and final residual norm squared.
    if (sdebug || _verb)
        showf("CG: %d iter (gm2=%.10g, rssb=%.10g, rssa=%.10g)\n", k, gm2, rssb, rssa);
    if (prssb) *prssb += rssb;
    if (prssa) *prssa += rssa;
    return gm2<_tolerance;
}

bool SparseLLS::solve(double* prssb, double* prssa) {
    auto up_timer = _verb ? make_unique<Timer>("_____SparseLLS", Timer::EMode::abbrev) : nullptr;
    assertx(!_solved); _solved = true;
    if (sdebug) showf("SparseLLS: solving %dx%d system, nonzerofrac=%f\n", _m, _n, float(_nentries)/_m/_n);
    if (prssb) *prssb = 0.;
    if (prssa) *prssa = 0.;
    Array<float> x(_n), rhv(_m);
    bool success = true;
    for_int(di, _nd) {
        for_int(i, _m) { rhv[i] = _b[di][i]; }
        for_int(j, _n) { x[j] = _x[di][j]; }
        if (!do_cg(x, rhv, prssb, prssa)) {
            success = false;
            if (0 && _max_iter==INT_MAX) continue;
        }
        for_int(j, _n) { _x[di][j] = x[j]; }
    }
    return success;
}

void SparseLLS::set_tolerance(float tolerance) {
    _tolerance = tolerance;
}

void SparseLLS::set_max_iter(int max_iter) {
    _max_iter = max_iter;
}

void SparseLLS::set_verbose(int verb) {
    _verb = verb;
}

// *** FullLLS

void FullLLS::clear() {
    fill(_a, 0.f); LLS::clear();
}

bool FullLLS::solve(double* prssb, double* prssa) {
    assertx(_m>=_n);
    assertx(!_solved); _solved = true;
    if (prssb) *prssb = get_rss();
    bool success = solve_aux();
    if (prssa) *prssa = get_rss();
    return success;
}

double FullLLS::get_rss() {
    double rss = 0.;
    for_int(di, _nd) {
        for_int(i, _m) {
            rss += square(dot(_a[i], _x[di])-_b[di][i]);
        }
    }
    return rss;
}

// *** LudLLS

bool LudLLS::solve_aux() {
    auto up_ta = _m!=_n ? make_unique<Matrix<float>>(_n, _n) : nullptr;
    MatrixView<float> a = up_ta ? *up_ta : _a;
    if (_m!=_n) {
        for_int(i, _n) {
            for_int(j, _n) {
                double s = 0.; for_int(k, _m) { s += double(_a[k][i])*_a[k][j]; }
                a[i][j] = float(s);
            }
        }
    }
    Array<int> rindx(_n);
    Array<float> t(_n);
    for_int(i, _n) {
        float vmax = 0.f;
        for_int(j, _n) { float v = abs(a[i][j]); if (v>vmax) vmax = v; }
        if (!vmax) return false;
        t[i] = 1.f/vmax;
    }
    int imax = 0;               // undefined
    for_int(j, _n) {
        for_int(i, j) {
            double s = a[i][j];
            for_int(k, i) { s -= double(a[i][k])*a[k][j]; }
            a[i][j] = float(s);
        }
        float vmax = 0.f;
        for_intL(i, j, _n) {
            double s = a[i][j];
            for_int(k, j) { s -= double(a[i][k])*a[k][j]; }
            a[i][j] = float(s);
            float v = t[i]*abs(a[i][j]);
            if (v>=vmax) { vmax = v; imax = i; }
        }
        if (imax!=j) { swap_ranges(a[imax], a[j]); t[imax] = t[j]; }
        rindx[j] = imax;
        if (!a[j][j]) return false;
        if (j<_n-1) {
            float v = 1.f/a[j][j];
            for_intL(i, j+1, _n) { a[i][j] *= v; }
        }
    }
    for_int(di, _nd) {
        if (_m==_n) {
            for_int(j, _n) { t[j] = _b[di][j]; }
        } else {
            for_int(j, _n) {
                double s = 0.; for_int(i, _m) { s += double(_a[i][j])*_b[di][i]; }
                t[j] = float(s);
            }
        }
        int ii = -1;
        for_int(i, _n) {
            int ip = rindx[i];
            double s = t[ip];
            t[ip] = t[i];
            if (ii>=0) {
                for_intL(j, ii, i) { s -= double(a[i][j])*t[j]; }
            } else if (s) {
                ii = i;
            }
            t[i] = float(s);
        }
        for (int i = _n-1; i>=0; --i) {
            double s = t[i];
            for_intL(j, i+1, _n) { s -= double(a[i][j])*t[j]; }
            t[i] = float(s/a[i][i]);
        }
        for_int(j, _n) { _x[di][j] = t[j]; }
    }
    return true;
}

// *** GivensLLS

bool GivensLLS::solve_aux() {
    int nposs = 0, ngivens = 0;
    for_int(i, _n) {
        for_intL(k, i+1, _m) {
            nposs++;
            if (!_a[k][i]) continue;
            ngivens++;
            float xi = _a[i][i];
            float xk = _a[k][i];
            float c, s;
            if (abs(xk)>abs(xi)) {
                float t = xi/xk; s = 1.f/sqrt(1.f+square(t)); c = s*t;
            } else {
                float t = xk/xi; c = 1.f/sqrt(1.f+square(t)); s = c*t;
            }
            for_intL(j, i, _n) {
                float xij = _a[i][j], xkj = _a[k][j];
                _a[i][j] = c*xij+s*xkj;
                _a[k][j] = -s*xij+c*xkj;
            }
            for_int(di, _nd) {
                float xij = _b[di][i], xkj = _b[di][k];
                _b[di][i] = c*xij+s*xkj;
                _b[di][k] = -s*xij+c*xkj;
            }
        }
        if (!_a[i][i]) { Warning("GivensLLS solution fails"); return false; }
    }
    if (sdebug) showf("Givens: %d/%d rotations done\n", ngivens, nposs);
    // Backsubstitutions
    for_int(di, _nd) {
        for (int i = _n-1; i>=0; --i) {
            float sum = _b[di][i];
            for_intL(j, i+1, _n) {
                sum -= _a[i][j]*_b[di][j];
            }
            _b[di][i] = sum/_a[i][i];
            _x[di][i] = _b[di][i];
        }
    }
    return true;
}

constexpr float k_float_cond_warning = 1e4f;
constexpr float k_float_cond_max = 1e5f;

constexpr double k_double_cond_warning = 1e8;
constexpr double k_double_cond_max = 1e12;

#if defined(HH_NO_LAPACK)

// *** SvdLLS

SvdLLS::SvdLLS(int m, int n, int nd) : FullLLS(m, n, nd), _work(n), _mU(m, n), _mS(_n), _mVT(_n, _n) { }

bool SvdLLS::solve_aux() {
    dummy_use(k_float_cond_max);
    if (!singular_value_decomposition(_a, _mU, _mS, _mVT)) return false;
    if (0) sort_singular_values(_mU, _mS, _mVT);
    if (!_mS.last()) return false;
    float cond = _mS[0] / _mS.last();
    if (cond>k_float_cond_warning) { HH_SSTAT(Ssvdlls_cond, cond); }
    // SHOW(_a, _b, _mU, _mS, _mVT);
    // SHOW(mat_mul(mat_mul(_mU, diag_mat(_mS)), transpose(_mVT)));
    for (float& s : _mS) { s = 1.f/s; }
    for_int(d, _nd) {
        // _x[d].assign(mat_mul(_mVT, mat_mul(_b[d], _mU)*_mS));
        mat_mul(_b[d], _mU, _work); _work *= _mS; mat_mul(_mVT, _work, _x[d]);
    }
    return true;
}

SvdDoubleLLS::SvdDoubleLLS(int m, int n, int nd) : FullLLS(m, n, nd), _mU(m, n), _mS(_n), _mVT(_n, _n) { }

bool SvdDoubleLLS::solve_aux() {
    dummy_use(k_double_cond_max);
    Matrix<double> A = convert<double>(_a);
    if (!singular_value_decomposition(A, _mU, _mS, _mVT)) return false;
    if (0) sort_singular_values(_mU, _mS, _mVT);
    if (!_mS.last()) return false;
    double cond = _mS[0] / _mS.last();
    if (cond>k_double_cond_warning) { HH_SSTAT(Ssvdlls_cond, cond); }
    for (double& s : _mS) { s = 1./s; }
    for_int(d, _nd) {
        _x[d].assign(convert<float>(mat_mul(_mVT, mat_mul(convert<double>(_b[d]), _mU)*_mS))); // slow
    }
    return true;
}

QrdLLS::QrdLLS(int m, int n, int nd) : FullLLS(m, n, nd), _work(n), _mU(m, n), _mS(_n), _mVT(_n, _n) { } // == SvdLLS

bool QrdLLS::solve_aux() {
    dummy_use(k_float_cond_max);
    if (!singular_value_decomposition(_a, _mU, _mS, _mVT)) return false;
    if (0) sort_singular_values(_mU, _mS, _mVT);
    if (!_mS.last()) return false;
    float cond = _mS[0] / _mS.last();
    if (cond>k_float_cond_warning) { HH_SSTAT(Ssvdlls_cond, cond); }
    for (float& s : _mS) { s = 1.f/s; }
    for_int(d, _nd) {
        // _x[d].assign(mat_mul(_mVT, mat_mul(_b[d], _mU)*_mS));
        mat_mul(_b[d], _mU, _work); _work *= _mS; mat_mul(_mVT, _work, _x[d]);
    }
    return true;
}

#else  // defined(HH_NO_LAPACK)

// *** NO_LAPACK

namespace {

inline int work_size(int m, int n) {
    return max(n*67, m*8)+128;  // picked somewhat arbitrarily
}

} // namespace

// *** SvdLLS

SvdLLS::SvdLLS(int m, int n, int nd)
    : FullLLS(m, n, nd), _fa(_m*_n), _fb(_m*_nd), _s(_n), _work(work_size(_m, _n)) { }

bool SvdLLS::solve_aux() {
    { float* ap = _fa.data(); for_int(j, _n) for_int(i, _m) *ap++ = _a[i][j]; }
    { float* bp = _fb.data(); for_int(d, _nd) for_int(i, _m) *bp++ = _b[d][i]; }
    float rcond = 1.f/k_float_cond_max;
    if (0) rcond = -1.f;        // use machine precision to determine rank
    lapack_int irank, lwork = _work.num(), info;
    lapack_int lm = _m, ln = _n, lnd = _nd;
    sgelss_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, _s.data(), &rcond, &irank, _work.data(), &lwork, &info);
    assertx(info>=0);
    if (info>0) return false;
    if (irank<_n) return false;
    // _s contains singular values in decreasing order.
    assertx(_s[_n-1]);
    float cond = _s[0]/_s[_n-1];
    if (cond>k_float_cond_warning) { HH_SSTAT(Ssvdlls_cond, cond); }
    {
        float* bp = _fb.data();
        for_int(d, _nd) {
            for_int(j, _n) { _x[d][j] = *bp++; }
            bp += (_m-_n);
        }
    }
    return true;
}

// *** SvdDoubleLLS

SvdDoubleLLS::SvdDoubleLLS(int m, int n, int nd)
    : FullLLS(m, n, nd), _fa(_m*_n), _fb(_m*_nd), _s(_n), _work(work_size(_m, _n)) { }

bool SvdDoubleLLS::solve_aux() {
    { double* ap = _fa.data(); for_int(j, _n) for_int(i, _m) *ap++ = _a[i][j]; }
    { double* bp = _fb.data(); for_int(d, _nd) for_int(i, _m) *bp++ = _b[d][i]; }
    double rcond = 1./k_double_cond_max;
    if (0) rcond = -1.;         // use machine precision to determine rank
    lapack_int irank, lwork = _work.num(), info;
    lapack_int lm = _m, ln = _n, lnd = _nd;
    dgelss_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, _s.data(), &rcond, &irank, _work.data(), &lwork, &info);
    assertx(info>=0);
    if (info>0) return false;
    if (irank<_n) return false;
    // _s contains singular values in decreasing order.
    assertx(_s[_n-1]);
    double cond = _s[0]/_s[_n-1];
    if (cond>k_double_cond_warning) { HH_SSTAT(Ssvdlls_cond, cond); }
    {
        double* bp = _fb.data();
        for_int(d, _nd) {
            for_int(j, _n) { _x[d][j] = float(*bp++); }
            bp += (_m-_n);
        }
    }
    return true;
}

// *** QrdLLS

QrdLLS::QrdLLS(int m, int n, int nd)
    : FullLLS(m, n, nd), _fa(_m*_n), _fb(_m*_nd)
      // _work(max(4*_n, 2*_n+_nd)) was for old sgelsx
{ }

bool QrdLLS::solve_aux() {
    { float* ap = _fa.data(); for_int(j, _n) for_int(i, _m) *ap++ = _a[i][j]; }
    { float* bp = _fb.data(); for_int(d, _nd) for_int(i, _m) *bp++ = _b[d][i]; }
    Array<lapack_int> jpvt(_n); fill(jpvt, 0); // all columns free to pivot
    float rcond = 1.f/k_float_cond_max;
    lapack_int irank, info;
    lapack_int lm = _m, ln = _n, lnd = _nd;
    // sgelsx_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, jpvt.data(), &rcond, &irank, _work.data(), &info);
    _work.init(1, 0); lapack_int lwork = -1; info = 0; // query
    sgelsy_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, jpvt.data(), &rcond, &irank,
            _work.data(), &lwork, &info); // query optimal work size
    assertx(info==0); _work.init(int(_work[0])); lwork = _work.num();
    sgelsy_(&lm, &ln, &lnd, _fa.data(), &lm, _fb.data(), &lm, jpvt.data(), &rcond, &irank,
            _work.data(), &lwork, &info);
    assertx(info==0);
    if (irank<_n) return false;
    // If need to use jpvt, remember that fortran indices start at 1.
    {
        float* bp = _fb.data();
        for_int(d, _nd) {
            for_int(j, _n) { _x[d][j] = *bp++; }
            bp += (_m-_n);
        }
    }
    return true;
}

#endif  // defined(HH_NO_LAPACK)

} // namespace hh
