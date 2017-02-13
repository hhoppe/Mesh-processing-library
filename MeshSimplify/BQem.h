// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_MESHSIMPLIFY_BQEM_H_
#define MESH_PROCESSING_MESHSIMPLIFY_BQEM_H_

#include "Qem.h"
#include "PArray.h"

namespace hh {

// BQem is an abstract base class for a set of derived DQem classes, each containing an instance of Qem.
// By using BQem, an application can decide at runtime which size Qem to use.
template<typename T> class BQem : noncopyable {
    using type = BQem<T>;
 public:
    virtual ~BQem()                             { }
    virtual void set_zero() = 0;
    virtual void copy(const type& qem) = 0;
    virtual void add(const type& qem) = 0;
    virtual void scale(float f) = 0;
    virtual void set_d2_from_plane(const float* dir, float d) = 0;
    virtual void set_d2_from_point(const float* p0) = 0;
    virtual void set_distance_gh98(const float* p0, const float* p1, const float* p2) = 0;
    virtual void set_distance_hh99(const float* p0, const float* p1, const float* p2) = 0;
    virtual float evaluate(const float* p) const = 0;
    virtual bool compute_minp(float* minp) const = 0;
    virtual bool compute_minp_constr_first(float* minp, int nfixed) const = 0;
    virtual bool compute_minp_constr_lf(float* minp, const float* lf) const = 0;
    virtual bool fast_minp_constr_lf(float* minp, const float* lf) const = 0;
    virtual bool ar_compute_minp(CArrayView<type*> ar_q, MatrixView<float> minp) const = 0;
    virtual bool ar_compute_minp_constr_lf(CArrayView<type*> ar_q, MatrixView<float> minp, const float* lf) const = 0;
    virtual bool check_type(int ptsize, int pn) const = 0;
    friend std::ostream& operator<<(std::ostream& os, const type& qem) { qem.serialize(os); return os; }
 protected:
    virtual void serialize(std::ostream& os) const = 0;
};

template<typename T, int n> class DQem : public BQem<T> {
    using type = DQem<T,n>;
    using base = BQem<T>;
 public:
    void set_zero() override { _q.set_zero(); }
    void copy(const base& qem) override         { ASSERTXX(check(qem)); _q = static_cast<const type&>(qem)._q; }
    void add(const base& qem) override          { ASSERTXX(check(qem)); _q.add(static_cast<const type&>(qem)._q); }
    void scale(float f) override                { _q.scale(f); }
    void set_d2_from_plane(const float* dir, float d) override { _q.set_d2_from_plane(dir, d); }
    void set_d2_from_point(const float* p0) override { _q.set_d2_from_point(p0); }
    void set_distance_gh98(const float* p0, const float* p1, const float* p2) override {
        _q.set_distance_gh98(p0, p1, p2);
    }
    void set_distance_hh99(const float* p0, const float* p1, const float* p2) override {
        _q.set_distance_hh99(p0, p1, p2);
    }
    float evaluate(const float* p) const override { return _q.evaluate(p); }
    bool compute_minp(float* minp) const override { return _q.compute_minp(minp); }
    bool compute_minp_constr_first(float* minp, int nfixed) const override {
        return _q.compute_minp_constr_first(minp, nfixed);
    }
    bool compute_minp_constr_lf(float* minp, const float* lf) const override {
        return _q.compute_minp_constr_lf(minp, lf);
    }
    bool fast_minp_constr_lf(float* minp, const float* lf) const override { return _q.fast_minp_constr_lf(minp, lf); }
    bool ar_compute_minp(CArrayView<base*> ar_q, MatrixView<float> minp) const override {
        CArrayView<type*> ar_qv(reinterpret_cast<type*const*>(ar_q.data()), ar_q.num());
        ASSERTX(ar_qv[0]==this);
        PArray<Qem<T,n>*, 20> ar_q2(ar_q.num()); for_int(i, ar_q.num()) { ar_q2[i] = &ar_qv[i]->_q; }
        ASSERTX(ar_q2[0]==&_q);
        return _q.ar_compute_minp(ar_q2, minp);
    }
    bool ar_compute_minp_constr_lf(CArrayView<base*> ar_q, MatrixView<float> minp, const float* lf) const override {
        CArrayView<type*> ar_qv(reinterpret_cast<type*const*>(ar_q.data()), ar_q.num());
        ASSERTX(ar_qv[0]==this);
        PArray<Qem<T,n>*, 20> ar_q2(ar_q.num()); for_int(i, ar_q.num()) { ar_q2[i] = &ar_qv[i]->_q; }
        ASSERTX(ar_q2[0]==&_q);
        return _q.ar_compute_minp_constr_lf(ar_q2, minp, lf);
    }
    bool check_type(int ptsize, int pn) const override { return ptsize==sizeof(T) && pn==n; }
 private:
    Qem<T,n> _q;
    bool check(const base& qem)                 { return qem.check_type(sizeof(T), n); }
    void serialize(std::ostream& os) const override { os << _q; }
};

template<typename T> HH_DECLARE_OSTREAM_EOL(BQem<T>);
template<typename T, int n> HH_DECLARE_OSTREAM_EOL(DQem<T,n>);

} // namespace hh

#endif // MESH_PROCESSING_MESHSIMPLIFY_BQEM_H_
