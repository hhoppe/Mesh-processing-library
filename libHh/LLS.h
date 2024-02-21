// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_LLS_H_
#define MESH_PROCESSING_LIBHH_LLS_H_

#include "libHh/Array.h"
#include "libHh/Matrix.h"

namespace hh {

// Solve a linear-least-squares system, i.e., for system A * x = b, find argmin_x |A * x - b|.
class Lls : noncopyable {
 public:
  // virtual constructor
  static unique_ptr<Lls> make(int m, int n, int nd, float nonzerofrac);  // A(m, n), x(n, nd), b(m, nd)
  virtual ~Lls() {}
  virtual void clear();
  // All entries will be zero unless entered as below.
  void enter_a(CMatrixView<float> mat);                     // [_m][_n]
  virtual void enter_a_r(int r, CArrayView<float> ar) = 0;  // r < _m, ar.num() == _n
  virtual void enter_a_c(int c, CArrayView<float> ar) = 0;  // c < _n, ar.num() == _m
  virtual void enter_a_rc(int r, int c, float val) = 0;     // r < _m, c < _n
  void enter_b(CMatrixView<float> mat);                     // [_m][_nd]
  void enter_b_r(int r, CArrayView<float> ar) {
    ASSERTX(ar.num() == _nd);
    for_int(c, _nd) enter_b_rc(r, c, ar[c]);
  }
  void enter_b_c(int c, CArrayView<float> ar) {
    ASSERTX(ar.num() == _m);
    for_int(r, _m) enter_b_rc(r, c, ar[r]);
  }
  void enter_b_rc(int r, int c, float val) { _b[c][r] = val; }
  void enter_xest(CMatrixView<float> mat);  // [_n][_nd]
  void enter_xest_r(int r, CArrayView<float> ar) {
    ASSERTX(ar.num() == _nd);
    for_int(c, _nd) enter_xest_rc(r, c, ar[c]);
  }
  void enter_xest_c(int c, CArrayView<float> ar) {
    ASSERTX(ar.num() == _n);
    for_int(r, _n) enter_xest_rc(r, c, ar[r]);
  }
  void enter_xest_rc(int r, int c, float val) { _x[c][r] = val; }          // r < _n, c < _nd
  virtual bool solve(double* rssb = nullptr, double* rssa = nullptr) = 0;  // ret: success
  void get_x(MatrixView<float> mat);                                       // [_n][_nd]
  void get_x_r(int r, ArrayView<float> ar) {
    ASSERTX(ar.num() == _nd);
    for_int(c, _nd) ar[c] = get_x_rc(r, c);
  }
  void get_x_c(int c, ArrayView<float> ar) {
    ASSERTX(ar.num() == _n);
    for_int(r, _n) ar[r] = get_x_rc(r, c);
  }
  float get_x_rc(int r, int c) { return _x[c][r]; }  // r < _n, c < _nd
  int num_rows() const { return _m; }

 protected:
  int _m, _n, _nd;
  Matrix<float> _b;     // [_nd][_m]; transpose of client view
  Matrix<float> _x;     // [_nd][_n]; transpose of client view
  bool _solved{false};  // solve() can destroy A, so check
  Lls(int m, int n, int nd);
};

// Sparse conjugate-gradient approach.
class SparseLls : public Lls {
 public:
  explicit SparseLls(int m, int n, int nd) : Lls(m, n, nd), _rows(m), _cols(n), _tolerance(square(8e-7f) * m) {}
  void clear() override;
  void enter_a_rc(int r, int c, float val) override;
  void enter_a_r(int r, CArrayView<float> ar) override;
  void enter_a_c(int c, CArrayView<float> ar) override;
  bool solve(double* rssb = nullptr, double* rssa = nullptr) override;
  void set_tolerance(float tolerance);  // default square(8e-7) * m  (because x is float) (was 1e-10f)
  void set_max_iter(int max_iter);      // default std::numeric_limits<int>::max()
  void set_verbose(int verb);           // default 0
 private:
  struct Ival {
    Ival() = default;
    Ival(int i, float v) : _i(i), _v(v) {}
    int _i;
    float _v;
  };
  Array<Array<Ival>> _rows;
  Array<Array<Ival>> _cols;
  float _tolerance;
  int _max_iter{std::numeric_limits<int>::max()};
  int _verb{0};
  int _nentries{0};
  Array<float> mult_m_v(CArrayView<float> vi) const;
  Array<float> mult_mt_v(CArrayView<float> vi) const;
  bool do_cg(Array<float>& x, CArrayView<float> h, double* prssb, double* prssa) const;
};

// Base class for full (non-sparse) approaches.
class FullLls : public Lls {
 public:
  explicit FullLls(int m, int n, int nd) : Lls(m, n, nd), _a(m, n) { FullLls::clear(); }
  void clear() override;
  void enter_a_rc(int r, int c, float val) override { _a[r][c] = val; }
  void enter_a_r(int r, CArrayView<float> ar) override {
    ASSERTX(ar.num() == _n);
    for_int(c, _n) _a[r][c] = ar[c];
  }
  void enter_a_c(int c, CArrayView<float> ar) override {
    ASSERTX(ar.num() == _m);
    for_int(r, _m) _a[r][c] = ar[r];
  }
  bool solve(double* rssb = nullptr, double* rssa = nullptr) override;

 protected:
  Matrix<float> _a;              // [_m][_n]
  virtual bool solve_aux() = 0;  // abstract class
 private:
  double get_rss();
};

// LU decomposition on A^t * A = A^t * b (slow).
class LudLls : public FullLls {
 public:
  explicit LudLls(int m, int n, int nd) : FullLls(m, n, nd) {}

 private:
  bool solve_aux() override;
};

// Givens substitution approach.
class GivensLls : public FullLls {
 public:
  explicit GivensLls(int m, int n, int nd) : FullLls(m, n, nd) {}

 private:
  bool solve_aux() override;
};

// Singular value decomposition.
class SvdLls : public FullLls {
 public:
  explicit SvdLls(int m, int n, int nd);

 private:
  bool solve_aux() override;
  Array<float> _fa, _fb, _s, _work;
  Matrix<float> _mU;
  Array<float> _mS;
  Matrix<float> _mVT;
};

// Double-precision version of SVD.
class SvdDoubleLls : public FullLls {
 public:
  explicit SvdDoubleLls(int m, int n, int nd);

 private:
  bool solve_aux() override;
  Array<double> _fa, _fb, _s, _work;
  Matrix<double> _mU;
  Array<double> _mS;
  Matrix<double> _mVT;
};

// QR decomposition.
class QrdLls : public FullLls {
 public:
  explicit QrdLls(int m, int n, int nd);

 private:
  bool solve_aux() override;
  Array<float> _fa, _fb, _work;
  Matrix<float> _mU;
  Array<float> _mS;
  Matrix<float> _mVT;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_LLS_H_
