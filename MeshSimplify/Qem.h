// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_MESHSIMPLIFY_QEM_H_
#define MESH_PROCESSING_MESHSIMPLIFY_QEM_H_

#include "Array.h"
#include "Matrix.h"

namespace hh {

// Quadric error metric used in mesh simplification (see also BQem.h).
template<typename T, int n> class Qem {
 public:
    void set_zero();
    void add(const Qem<T,n>& qem);
    void scale(float f);
// Create
    void set_d2_from_plane(const float* dir, float d); // dir*p+d==0 !
    void set_d2_from_point(const float* p0);
    void set_distance_gh98(const float* p0, const float* p1, const float* p2);
    void set_distance_hh99(const float* p0, const float* p1, const float* p2);
// Evaluate
    float evaluate(const float* p) const;
// Find min.  All these return "success"; minp unchanged if unsuccessful
    // regular version
    bool compute_minp(float* minp) const;
    // First nfixed variables are fixed as given; solve for remainder.
    bool compute_minp_constr_first(float* minp, int nfixed) const;
    // Linear functional constraint: lf[0..n-1] * x + lf[n] == 0
    bool compute_minp_constr_lf(float* minp, const float* lf) const;
    // Sparse version w/ independent attribs and lf constr only on geom
    bool fast_minp_constr_lf(float* minp, const float* lf) const;
// Special versions for wedge-based mesh simplification.
    // They take multiple quadrics (*this==ar_q[0]) and assume that the
    //  first 3 rows of matrix variables are shared.
    // regular version
    bool ar_compute_minp(CArrayView<Qem<T,n>*> ar_q, MatrixView<float> minp) const;
    // linear functional constraint must be volumetric!
    bool ar_compute_minp_constr_lf(CArrayView<Qem<T,n>*> ar_q, MatrixView<float> minp, const float* lf) const;
    friend std::ostream& operator<<(std::ostream& os, const Qem<T,n>& qem) {
        os << "Qem{\n a={\n";
        const T* pa = qem._a.data();
        for_int(i, n) {
            for_int(j, i) { os << "          "; }
            for_intL(j, i, n) { os << sform(" %-9.3g", *pa++); }
            os << "\n";
        }
        os << "}\n b={\n";
        for_int(i, n) { os << sform(" %-9.3g", qem._b[i]); }
        return os << "\n}\n c=" << sform(" %-9.3g", qem._c) << "\n}\n";
    }
 private:
    // qem(v) = v*a*v + 2*b*v + c
    Vec<T, (n*(n+1))/2> _a;  // upper triangle of symmetric matrix
    Vec<T,n> _b;
    T _c;
    template<typename TT, int n1, int n2> friend void qem_add_submatrix(Qem<TT,n1>& q1, const Qem<TT,n2>& q2);
};

template<typename T, int n> HH_DECLARE_OSTREAM_EOL(Qem<T,n>);

} // namespace hh

#endif // MESH_PROCESSING_MESHSIMPLIFY_QEM_H_
