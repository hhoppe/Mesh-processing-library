// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_MESHSIMPLIFY_QEM_H_
#define MESH_PROCESSING_MESHSIMPLIFY_QEM_H_

#include "libHh/Array.h"
#include "libHh/Matrix.h"

namespace hh {

// Quadric error metric used in mesh simplification (see also BQem.h).
template <typename T, int n> class Qem {
 public:
  void set_zero();
  void add(const Qem<T, n>& qem);
  void scale(float f);

  // Create:
  void set_d2_from_plane(const float* dir, float d);  // Requires: dir * p + d == 0 !
  void set_d2_from_point(const float* p0);
  void set_distance_gh98(const float* p0, const float* p1, const float* p2);
  void set_distance_hh99(const float* p0, const float* p1, const float* p2);

  // Evaluate:
  float evaluate(const float* p) const;

  // Find min.  All these return "success"; minp unchanged if unsuccessful:
  // Regular version.
  [[nodiscard]] bool compute_minp(float* minp) const;
  // First nfixed variables are fixed as given; solve for remainder.
  [[nodiscard]] bool compute_minp_constr_first(float* minp, int nfixed) const;
  // Linear functional constraint: lf[0 .. n - 1] * x + lf[n] == 0.
  [[nodiscard]] bool compute_minp_constr_lf(float* minp, const float* lf) const;
  // Sparse version w/ independent attribs and lf constr only on geom.
  [[nodiscard]] bool fast_minp_constr_lf(float* minp, const float* lf) const;

  // Special versions for wedge-based mesh simplification:
  // They take multiple quadrics (*this == ar_q[0]) and assume that the first 3 rows of matrix variables are shared.
  // Regular version.
  [[nodiscard]] bool ar_compute_minp(CArrayView<Qem<T, n>*> ar_q, MatrixView<float> minp) const;
  // Linear functional constraint must be volumetric!
  [[nodiscard]] bool ar_compute_minp_constr_lf(CArrayView<Qem<T, n>*> ar_q, MatrixView<float> minp,
                                               const float* lf) const;

  friend std::ostream& operator<<(std::ostream& os, const Qem<T, n>& qem) {
    os << "Qem{\n a={\n";
    const T* pa = qem._a.data();
    for_int(i, n) {
      for_int(j, i) os << "          ";
      for_intL(j, i, n) os << sform(" %-9.3g", *pa++);
      os << "\n";
    }
    os << "}\n b={\n";
    for_int(i, n) os << sform(" %-9.3g", qem._b[i]);
    return os << "\n}\n c=" << sform(" %-9.3g", qem._c) << "\n}\n";
  }

 private:
  // Representing: qem_energy(v) = (v^T * _a * v) + (2 * _b * v) + _c.
  Vec<T, (n * (n + 1)) / 2> _a;  // Upper triangle of symmetric matrix.
  Vec<T, n> _b;
  T _c;
};

template <typename T, int n> HH_DECLARE_OSTREAM_EOL(Qem<T, n>);

extern template class Qem<float, 3>;
extern template class Qem<double, 3>;
extern template class Qem<float, 6>;
extern template class Qem<double, 6>;
extern template class Qem<float, 9>;
extern template class Qem<double, 9>;

//----------------------------------------------------------------------------

template <typename T, int n> void Qem<T, n>::add(const Qem<T, n>& qem) {
  _a += qem._a;
  _b += qem._b;
  _c += qem._c;
}

}  // namespace hh

#endif  // MESH_PROCESSING_MESHSIMPLIFY_QEM_H_
