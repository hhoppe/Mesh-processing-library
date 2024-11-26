// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "MeshSimplify/Qem.h"

#include "libHh/Lls.h"
#include "libHh/Polygon.h"  // orthogonal_vector()
#include "libHh/RangeOp.h"
#include "libHh/SGrid.h"
#include "libHh/Vec.h"

namespace hh {

namespace {

void print_matrix(CMatrixView<double> m) {
  std::cerr << "Matrix{\n";
  for_int(i, m.ysize()) {
    std::cerr << " ";
    for_int(j, m.xsize()) std::cerr << sform(" %-12g", m[i][j]);
    std::cerr << "\n";
  }
  std::cerr << "} EndMatrix\n";
}

}  // namespace

// Many static data structures below -- lots of code is not threadsafe.

template <typename T, int n> void Qem<T, n>::set_zero() {
  fill(_a, T{0});
  fill(_b, T{0});
  _c = T{0};
}

template <typename T, int n> void Qem<T, n>::scale(float f) {
  _a *= T{f};
  _b *= T{f};
  _c *= T{f};
}

template <typename T, int n> void Qem<T, n>::set_d2_from_plane(const float* dir, float d) {
  // Qem = (dir^T * x - d) ^ 2
  // A = dir * dir^T
  // b = d * dir
  // c = square(d)
  {
    T* pa = _a.data();
    for_int(i, n) for_intL(j, i, n)* pa++ = T{dir[i]} * dir[j];
  }
  for_int(i, n) _b[i] = T{d} * dir[i];
  _c = square(T{d});
}

template <typename T, int n> void Qem<T, n>::set_d2_from_point(const float* p0) {
  // Qem = \Sum (x[i] - p0[i]) ^ 2  =  (x - p0)^T * (x - p0)
  // A = I
  // b = -p0
  // c = mag2(p0)
  {
    T* pa = _a.data();
    for_int(i, n) {
      *pa++ = T{1};
      for_intL(j, i + 1, n) { *pa++ = T{0}; }
    }
  }
  for_int(i, n) _b[i] = -T{p0[i]};
  T a = T{0};
  for_int(i, n) a += square(p0[i]);
  _c = a;
}

template <typename T, int n> void Qem<T, n>::set_distance_gh98(const float* p0, const float* p1, const float* p2) {
  Vec<T, n> e1, e2;
  // e1 = p1 - p0,  e2 = p2 - p0
  for_int(i, n) {
    e1[i] = T{p1[i]} - p0[i];
    e2[i] = T{p2[i]} - p0[i];
  }
  // Now orthonormalize {e1, e2}.
  {
    // e1_nor = e1.normalize()
    T a;
    a = T{0};
    for_int(c, n) a += square(e1[c]);
    if (!assertw(a)) {
      set_zero();
      return;
    }
    a = T{1} / sqrt(a);
    for_int(c, n) e1[c] *= a;
    // e2_nor = (e2 - dot(e2, e1_nor) * e1).normalize()
    a = T{0};
    for_int(c, n) a += e1[c] * e2[c];
    for_int(c, n) e2[c] -= a * e1[c];
    a = T{0};
    for_int(c, n) a += square(e2[c]);
    if (!assertw(a)) {
      set_zero();
      return;
    }
    a = T{1} / sqrt(a);
    for_int(c, n) e2[c] *= a;
  }
  // From Garland & Heckbert paper in Visualization 98 [G&H98]:
  // A = I - e1 * e1^T - e2 * e2^T
  // b = dot(p, e1) * e1 + dot(p, e2) * e2 - p
  // c = dot(p, p) - square(dot(p, e1)) - square(dot(p, e2))
  {
    T* pa = _a.data();
    for_int(i, n) {
      *pa++ = T{1} - e1[i] * e1[i] - e2[i] * e2[i];
      for_intL(j, i + 1, n) { *pa++ = T{0} - e1[i] * e1[j] - e2[i] * e2[j]; }
    }
  }
  const float* p = p0;
  T dot_p_e1 = T{0}, dot_p_e2 = T{0}, dot_p_p = T{0};
  for_int(i, n) {
    dot_p_e1 += p[i] * e1[i];
    dot_p_e2 += p[i] * e2[i];
    dot_p_p += square(T{p[i]});
  }
  for_int(i, n) _b[i] = dot_p_e1 * e1[i] + dot_p_e2 * e2[i] - p[i];
  _c = dot_p_p - square(dot_p_e1) - square(dot_p_e2);
}

template <typename T, int n> void Qem<T, n>::set_distance_hh99(const float* p0, const float* p1, const float* p2) {
  const int ngeom = 3, nattrib = n - ngeom;
  Vec<T, ngeom> nor;
  // Vector nor = cross(Point(p0[0], p0[1], p0[2]),
  //                    Point(p1[0], p1[1], p1[2]),
  //                    Point(p2[0], p2[1], p2[2]));
  // assertx(nor.normalize());
  {
    T p0x = p0[0], p0y = p0[1], p0z = p0[2];
    T v1x = p1[0] - p0x, v1y = p1[1] - p0y, v1z = p1[2] - p0z;
    T v2x = p2[0] - p0x, v2y = p2[1] - p0y, v2z = p2[2] - p0z;
    nor[0] = v1y * v2z - v1z * v2y;
    nor[1] = v1z * v2x - v1x * v2z;
    nor[2] = v1x * v2y - v1y * v2x;
    T sum2 = square(nor[0]) + square(nor[1]) + square(nor[2]);
    if (!assertw(sum2)) {
      set_zero();
      return;
    }
    T fac = T{1} / sqrt(sum2);
    nor[0] *= fac;
    nor[1] *= fac;
    nor[2] *= fac;
  }
  // Introduce the geometric error quadric component (adapted from set_d2_from_plane).
  {
    // float d = -dot(Point(p0[0], p0[1], p0[2]), nor);
    T d = -(p0[0] * nor[0] + p0[1] * nor[1] + p0[2] * nor[2]);
    {
      T* pa = _a.data();
      for_int(i, ngeom) {  // note: only traverse first ngeom rows
        for_intL(j, i, ngeom) { *pa++ = nor[i] * nor[j]; }
        pa += nattrib;
      }
    }
    for_int(i, ngeom) _b[i] = nor[i] * d;
    _c = d * d;
  }
  // Introduce the scalar error quadric components.
  if (nattrib) {
    //  (  v0   1 )   (     )       ( s0 )
    //  (  v1   1 ) * ( g_s )   =   ( s1 )
    //  (  v2   1 )   (     )       ( s2 )
    //  (  n    0 )   ( d_s )       ( 0  )
    //
    // static LudLls lls(4, 4, nattrib);
    // lls.clear();
    LudLls lls(4, 4, nattrib);  // Threadsafe.
    for_int(c, 3) {
      lls.enter_a_rc(0, c, p0[c]);
      lls.enter_a_rc(1, c, p1[c]);
      lls.enter_a_rc(2, c, p2[c]);
      lls.enter_a_rc(3, c, float(nor[c]));
    }
    lls.enter_a_rc(0, 3, 1.f);
    lls.enter_a_rc(1, 3, 1.f);
    lls.enter_a_rc(2, 3, 1.f);
    lls.enter_a_rc(3, 3, 0.f);
    lls.enter_b_r(0, CArrayView(&p0[ngeom], nattrib));
    lls.enter_b_r(1, CArrayView(&p1[ngeom], nattrib));
    lls.enter_b_r(2, CArrayView(&p2[ngeom], nattrib));
    // Row 3 of right-hand-side is kept zero.
    if (!lls.solve()) {
      Warning("set_distance_hh99: lls.solve() failed");
      // geometric component ok, set qem to zero for scalars
      for_int(si, nattrib) {
        {
          T* pa = _a.data();
          for_int(i, n) {
            for_intL(j, i, n) {
              if (j == ngeom + si) *pa = T{0};
              pa++;
            }
          }
        }
        _b[ngeom + si] = T{0};
      }
      return;
    }
    for_int(si, nattrib) {
      Vec4<float> sol;
      lls.get_x_c(si, sol);
      const Vec3<T> g_s = {sol[0], sol[1], sol[2]};
      T d_s = sol[3];
      {
        T* pa = _a.data();
        for_int(i, n) {
          for_intL(j, i, n) {
            if (j < ngeom)
              *pa += g_s[i] * g_s[j];
            else if (j == ngeom + si)
              *pa = i < ngeom ? -g_s[i] : i == ngeom + si ? T{1} : T{0};
            // (The non-modified *pa are set in other iterations of the "si" loop.)
            pa++;
          }
        }
      }
      for_int(i, ngeom) _b[i] += d_s * g_s[i];
      _b[ngeom + si] = -d_s;
      _c += square(d_s);
    }
  }
}

template <typename T, int n> float Qem<T, n>::evaluate(const float* p) const {
  // evaluation = v^T * A * v + 2 * b^T * v + c
  T sum1 = T{0}, sum2 = T{0};
  {
    const T* pa = _a.data();
    for_int(i, n) {
      sum1 += *pa++ * square(T{p[i]});
      for_intL(j, i + 1, n) sum2 += *pa++ * T{p[i]} * p[j];
    }
  }
  // for_int(i, n) sum2 += _b[i] * p[i];  // GCC4.8.1 array subscript is above array bounds
  const T* bt = _b.data();
  for_int(i, n) sum2 += bt[i] * p[i];
  sum1 += _c;
  T sum = sum1 + sum2 + sum2;
  return float(sum);
}

// minp unchanged if unsuccessful !
template <typename T, int n> bool Qem<T, n>::compute_minp(float* minp) const {
  // minp = - A^-1 b        or     A * minp = -b
  static SvdDoubleLls lls(n, n, 1);  // not threadsafe!
  lls.clear();
  {
    const T* pa = _a.data();
    for_int(i, n) {
      lls.enter_a_rc(i, i, float(*pa));
      pa++;
      for_intL(j, i + 1, n) {
        lls.enter_a_rc(i, j, float(*pa));
        lls.enter_a_rc(j, i, float(*pa));
        pa++;
      }
    }
  }
  for_int(i, n) lls.enter_b_rc(i, 0, float(-_b[i]));
  if (!lls.solve()) return false;
  lls.get_x_c(0, ArrayView(minp, n));
  return true;
}

// minp unchanged if unsuccessful !
template <typename T, int n> bool Qem<T, n>::compute_minp_constr_first(float* minp, int nfixed) const {
  const int nf = nfixed;
  assertx(nf > 0 && nf < n);
  // Given fixed minp[0 .. nf - 1], optimize for minp[nf .. n - 1] .
  //  A_22 * x_2 = (-b_2 - A_21 * x_1)    (A_21 = A_12^T)
  static unique_ptr<SvdDoubleLls> plls;
  static int prev_nf;
  if (!plls || prev_nf != nf) {
    plls = make_unique<SvdDoubleLls>(n - nf, n - nf, 1);
    prev_nf = nf;
  }
  SvdDoubleLls& lls = *plls;
  Vec<double, n> b;
  // for_int(i, n - nf) { b[i] = -_b[nf + i]; } // GCC4.8.1 [-Werror=array-bounds]
  const T* bt = _b.data();
  for_int(i, n - nf) b[i] = -bt[nf + i];
  {
    const T* pa = _a.data();
    for_int(i, n) {
      for_intL(j, i, n) {
        if (i >= nf && j >= nf) {
          lls.enter_a_rc(i - nf, j - nf, float(*pa));
          lls.enter_a_rc(j - nf, i - nf, float(*pa));
        }
        if (i < nf && j >= nf) b[j - nf] -= *pa * minp[i];
        pa++;
      }
    }
  }
  for_int(i, n - nf) lls.enter_b_rc(i, 0, float(b[i]));
  if (!lls.solve()) return false;
  lls.get_x_c(0, ArrayView(&minp[nf], n - nf));
  return true;
}

// minp unchanged if unsuccessful !
template <typename T, int n> bool Qem<T, n>::compute_minp_constr_lf(float* minp, const float* lf) const {
  // Constraint: lf[0 .. n - 1] * x + lf[n] == 0
  //  here special case of constraint A' * x == b'
  // Let (columns of) Z span the null space of A'
  //  so in this case n - 1 vectors orthogonal to lf.
  // Then want gradient to be orthogonal to Z:
  //  Z^T * (A * x + b) == 0
  //  (Z^T * A) * x == (-Z^T * b)
  // These form n - 1 additional constraints -> solve n x n linear system.
  // Note: when number m of constraints is large, it is better to
  //  set up a new m x m linear system:
  //  x = x0 + Z * w
  //  (Z^T * A * Z) * w = (-Z^T * (A * x0 + b))
  assertx(n >= 2);
  static Matrix<double> a;
  if (!a.ysize()) a.init(n, n);
  {
    const T* pa = _a.data();
    for_int(i, n) {
      a[i][i] = *pa;
      pa++;
      for_intL(j, i + 1, n) {
        a[i][j] = *pa;
        a[j][i] = *pa;
        pa++;
      }
    }
    if (0) print_matrix(a);
  }
  static Matrix<double> zt;
  if (!zt.ysize()) zt.init(n - 1, n);
  {
    assertx(n >= 3);
    // Note: at present only handle extremely restricted case.
    for_intL(i, 3, n) assertx(lf[i] == 0.f);
    for_int(i, n - 1) for_int(j, n) zt[i][j] = 0.;
    Vec2<Vector> voa;
    {
      Vector vlf(lf[0], lf[1], lf[2]);
      voa[0] = orthogonal_vector(vlf);
      // instead of normalizing, could just scale max(abs) to 1.
      assertx(voa[0].normalize());
      voa[1] = cross(voa[0], vlf);
      assertx(voa[1].normalize());
    }
    for_int(i, 2) for_int(c, 3) zt[i][c] = voa[i][c];
    for_intL(i, 3, n) zt[i - 1][i] = 1.;
    if (0) print_matrix(zt);
  }
  static SvdDoubleLls lls(n, n, 1);
  lls.clear();
  for_int(i, n - 1) {
    for_int(j, n) {
      // row i of Z^T times column j of A
      double v = 0.;
      for_int(k, n) v += zt[i][k] * a[k][j];
      lls.enter_a_rc(i, j, float(v));
    }
  }
  for_int(i, n - 1) {
    // row i of Z^T times -b
    double v = 0.;
    for_int(k, n) v += -zt[i][k] * _b[k];
    lls.enter_b_rc(i, 0, float(v));
  }
  lls.enter_a_r(n - 1, CArrayView(lf, n));
  lls.enter_b_rc(n - 1, 0, -lf[n]);
  if (!lls.solve()) return false;
  lls.get_x_c(0, ArrayView(minp, n));
  return true;
}

// minp unchanged if unsuccessful !
template <typename T, int n> bool Qem<T, n>::fast_minp_constr_lf(float* minp, const float* lf) const {
  const int ngeom = 3, nattrib = n - ngeom;
  assertx(nattrib >= 0);
  // lf has n + 1 entries.  lf[0 .. n - 1] is the vector g_vol, lf[n] is d_vol
  // This function only works when the QEM matrix A is diagonal beyond
  // the leading 3x3 block.  The solution is based on the block structure:
  // [   C   g_v     B   ] [ p  ]    [  b1  ]
  // [ g_v^T  0      0   ] [ gm ] == [ -d_v ]
  // [  B^T   0   al * I ] [ s  ]    [  b2  ]
  // (g_v, d_v, and gm are the volume-preservation constraint)
  // (b is [-b1; -b2])
  // The procedure is:
  //   C1 = (C - B * B^T / al);
  //   [p; gm] = [C1 g; g^T 0]^(-1) * [b1 - B * b2 / al; -d_v];
  //   s = (b2 - B^T * p) / al;
  static Matrix<double> c;
  if (!c.ysize()) c.init(ngeom, ngeom);
  static Matrix<double> b;
  if (!b.ysize() && nattrib) b.init(ngeom, nattrib);
  double alinv;  // 1.0 / al
  {
    const T* pa = _a.data();
    for_int(i, ngeom) {
      c[i][i] = *pa;
      pa++;
      for_intL(j, i + 1, ngeom) {
        c[i][j] = *pa;
        c[j][i] = *pa;
        pa++;
      }
      for_int(j, nattrib) {
        b[i][j] = *pa;
        pa++;
      }
    }
    double al = *pa;
    alinv = 1. / al;
    if (0) {
      for_intL(i, ngeom, n) {
        assertx(*pa == al);
        pa++;
        for_intL(j, i + 1, n) {
          assertx(*pa == 0);
          pa++;
        }
        assertx(lf[i] == 0);
      }
    }
    if (0) print_matrix(c);
    if (0) print_matrix(b);
  }
  static SvdDoubleLls lls(ngeom + 1, ngeom + 1, 1);
  lls.clear();
  for_int(i, ngeom) {
    for_int(j, ngeom) {
      // enter C - B * B^T / al
      double x = 0.;
      for_int(k, nattrib) x += b[i][k] * b[j][k];
      lls.enter_a_rc(i, j, float(c[i][j] - alinv * x));
    }
    // enter b1 - B * b2 / al
    double x = 0.;
    for_int(k, nattrib) x += b[i][k] * -_b[ngeom + k];
    lls.enter_b_rc(i, 0, float(-_b[i] - alinv * x));
    lls.enter_a_rc(ngeom, i, lf[i]);
    lls.enter_a_rc(i, ngeom, lf[i]);
  }
  // enter -d_v
  lls.enter_b_rc(ngeom, 0, -lf[n]);
  if (!lls.solve()) return false;
  for_int(i, ngeom) minp[i] = lls.get_x_rc(i, 0);
  for_int(i, nattrib) {
    double v = -_b[ngeom + i];
    for_int(k, ngeom) v -= b[k][i] * minp[k];
    minp[ngeom + i] = float(alinv * v);
  }
  return true;
}

// minp unchanged if unsuccessful !
template <typename T, int n>
bool Qem<T, n>::ar_compute_minp(CArrayView<Qem<T, n>*> ar_q, MatrixView<float> minp) const {
  assertx(ar_q[0] == this);
  const int nw = ar_q.num();
  assertx(minp.ysize() >= nw && minp.xsize() >= n);
  const int ngeom = 3, nattrib = n - ngeom;
  assertx(nattrib >= 0);
  const int msize = ngeom + nattrib * nw;
  // cache previous size
  static unique_ptr<SvdDoubleLls> plls;
  static int psize;
  if (msize != psize) {
    plls = make_unique<SvdDoubleLls>(msize, msize, 1);
    psize = msize;
  }
  SvdDoubleLls& lls = *plls;
  lls.clear();
  SGrid<double, ngeom, ngeom> msum;
  fill(msum, 0.);
  Vec<double, ngeom> vsum;
  fill(vsum, 0.);
  for_int(wi, nw) {
    const Qem<T, n>& qem = *ar_q[wi];  // be careful never to use *this!
    const int inc = nattrib * wi;
    {
      const T* pa = qem._a.data();
      for_int(i, ngeom) {
        msum[i][i] += *pa;
        pa++;
        for_intL(j, i + 1, ngeom) {
          msum[i][j] += *pa;
          msum[j][i] += *pa;
          pa++;
        }
        for_intL(j, ngeom, n) {
          lls.enter_a_rc(i, inc + j, float(*pa));
          lls.enter_a_rc(inc + j, i, float(*pa));
          pa++;
        }
      }
      for_intL(i, ngeom, n) {
        lls.enter_a_rc(inc + i, inc + i, float(*pa));
        pa++;
        for_intL(j, i + 1, n) {
          lls.enter_a_rc(inc + i, inc + j, float(*pa));
          lls.enter_a_rc(inc + j, inc + i, float(*pa));
          pa++;
        }
      }
    }
    for_int(i, ngeom) vsum[i] += qem._b[i];
    for_intL(i, ngeom, n) lls.enter_b_rc(inc + i, 0, float(-qem._b[i]));
  }
  for_int(i, ngeom) for_int(j, ngeom) lls.enter_a_rc(i, j, float(msum[i][j]));
  for_int(i, ngeom) lls.enter_b_rc(i, 0, float(-vsum[i]));
  if (!lls.solve()) return false;
  for_int(wi, nw) {
    const int inc = nattrib * wi;
    for_int(i, ngeom) minp[wi][i] = lls.get_x_rc(i, 0);
    for_intL(i, ngeom, n) minp[wi][i] = lls.get_x_rc(inc + i, 0);
  }
  return true;
}

#define DEF_LAGRANGE

// minp unchanged if unsuccessful !
template <typename T, int n>
bool Qem<T, n>::ar_compute_minp_constr_lf(CArrayView<Qem<T, n>*> ar_q, MatrixView<float> minp, const float* lf) const {
  assertx(ar_q[0] == this);
  const int nw = ar_q.num();
  assertx(minp.ysize() >= nw && minp.xsize() >= n);
  const int ngeom = 3, nattrib = n - ngeom;
  assertx(nattrib >= 0);
  const int msize = ngeom + nattrib * nw;
  static Matrix<double> a;
  if (a.ysize() != msize) a.init(msize, msize);
  fill(a, 0.);
  static Array<double> b;
  if (b.num() != msize) b.init(msize);
  fill(b, 0.);
  for_int(wi, nw) {
    const Qem<T, n>& qem = *ar_q[wi];  // be careful never to use *this!
    const int inc = nattrib * wi;
    {
      const T* pa = qem._a.data();
      for_int(i, ngeom) {
        a[i][i] += *pa;
        pa++;
        for_intL(j, i + 1, ngeom) {
          a[i][j] += *pa;
          a[j][i] += *pa;
          pa++;
        }
        for_intL(j, ngeom, n) {
          a[i][inc + j] = *pa;
          a[inc + j][i] = *pa;
          pa++;
        }
      }
      for_intL(i, ngeom, n) {
        a[inc + i][inc + i] = *pa;
        pa++;
        for_intL(j, i + 1, n) {
          a[inc + i][inc + j] = *pa;
          a[inc + j][inc + i] = *pa;
          pa++;
        }
      }
    }
    for_int(i, ngeom) b[i] += qem._b[i];
    for_intL(i, ngeom, n) b[inc + i] = qem._b[i];
  }
#if defined(DEF_LAGRANGE)
  const int msize1 = msize + 1;
#else
  const int msize1 = msize;
#endif
  // cache previous size
  static unique_ptr<SvdDoubleLls> plls;
  static int psize1;
  if (msize1 != psize1) {
    plls = make_unique<SvdDoubleLls>(msize1, msize1, 1);
    psize1 = msize1;
  }
  SvdDoubleLls& lls = *plls;
  lls.clear();
#if defined(DEF_LAGRANGE)
  for_int(i, msize) for_int(j, msize) lls.enter_a_rc(i, j, float(a[i][j]));
  for_int(i, msize) lls.enter_b_rc(i, 0, float(-b[i]));
  // At present only handle extremely restricted case.
  for_intL(i, 3, n) assertx(lf[i] == 0.f);
  for_int(j, ngeom) lls.enter_a_rc(msize, j, lf[j]);
  for_int(j, ngeom) lls.enter_a_rc(j, msize, lf[j]);
  lls.enter_b_rc(msize, 0, -lf[n]);
#else
  static Matrix<double> zt;
  if (!zt.num() != msize - 1) zt.init(msize - 1, msize);
  {
    // Note: at present only handle extremely restricted case.
    for_intL(i, 3, n) assertx(lf[i] == 0.f);
    for_int(i, msize - 1) for_int(j, msize) zt[i][j] = 0.f;
    Vec2<Vector> voa;
    {
      Vector vlf(lf[0], lf[1], lf[2]);
      voa[0] = orthogonal_vector(vlf);
      // instead of normalizing, could just scale max(abs) to 1.
      assertx(voa[0].normalize());
      voa[1] = cross(voa[0], vlf);
      assertx(voa[1].normalize());
    }
    for_int(i, 2) for_int(c, 3) zt[i][c] = voa[i][c];
    for_intL(i, 3, msize) zt[i - 1][i] = 1.f;
  }
  for_int(i, msize - 1) {
    for_int(j, msize) {
      // row i of Z^T times column j of A
      double v = 0.;
      for_int(k, msize) v += zt[i][k] * a[k][j];
      lls.enter_a_rc(i, j, v);
    }
  }
  for_int(i, msize - 1) {
    // row i of Z^T times -b
    double v = 0.;
    for_int(k, msize) v += -zt[i][k] * b[k];
    lls.enter_b_rc(i, 0, v);
  }
  for_int(j, ngeom) lls.enter_a_rc(msize - 1, j, lf[j]);
  lls.enter_b_rc(msize - 1, 0, -lf[n]);
#endif  // defined(DEF_LAGRANGE)
  if (!lls.solve()) return false;
  for_int(wi, nw) {
    const int inc = nattrib * wi;
    for_int(i, ngeom) minp[wi][i] = lls.get_x_rc(i, 0);
    for_intL(i, ngeom, n) minp[wi][i] = lls.get_x_rc(inc + i, 0);
  }
  return true;
}

template class Qem<float, 3>;
template class Qem<double, 3>;
template class Qem<float, 6>;
template class Qem<double, 6>;
template class Qem<float, 9>;
template class Qem<double, 9>;

}  // namespace hh
