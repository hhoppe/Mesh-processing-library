// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Pool.h"

// A Sac reserves space at the end of a structure for additional storage requested by other program components.

#if 0
{
    class MVertex {
        ...;
     public:
        HH_MAKE_SAC(MVertex);   // must be last entry of class!
    };
    namespace hh { HH_SAC_INITIALIZATION(MVertex); }
    //
    HH_SAC_ALLOCATE_FUNC(MVertex, Point, v_point);
    v_point(v) = Point(1.f, 2.f, 3.f); SHOW(v_point(v));
    //
    using ArrayInt = Array<int>;
    HH_SACABLE(ArrayInt);
    HH_SAC_ALLOCATE_CD_FUNC(MVertex, ArrayInt, v_ar); // CD means call constructor and destructor
    v_ar(v).push(7);
    //
    static int key_a = HH_SAC_ALLOCATE_CD(MVertex, A);
    sac_access<A>(v, key_a).b = 3;
}
#endif

namespace hh {

class BSac : noncopyable { // noncopyable for safety -- could be removed if careful; if so, make Sac<> noncopyable
 public:
    using Func = void (*)(void*);
    static constexpr int k_dummy = 4;
// per-object
    void* access(int k) { return _a+k; }
 protected:
    void call_funcs(int num, int akeys[], Func afuncs[]) {
        for_int(i, num) {
            // SHOW(i, uintptr_t(_a), akeys[i]);
            afuncs[i](_a+akeys[i]);
        }
    }
    HH_ALIGNAS(16) char _a[k_dummy];
};

template<typename T> class Sac : public BSac {
    static constexpr int k_max = 50;
 public:
// static
    template<typename T2> static int allocate() {
        unsigned s = sizeof(T2);
        int k = size;
        int align = alignof(T2);
        max_align = max(max_align, align);
        k = int((k+align-1)/align)*align;
        if (0) SHOW(type_name<T>(), type_name<T2>(), s, size, align, max_align, k);
        size = k+s;
        return k;
    }
    template<typename T2> static int allocate_cd(Func fcons, Func fdest) {
        int k = allocate<T2>();
        if (fcons) {
            assertx(cnum<k_max); HH_ASSUME(cnum<k_max);
            ckeys[cnum] = k; cfuncs[cnum] = fcons;
            cnum++;
        }
        if (fdest) {
            assertx(dnum<k_max); HH_ASSUME(dnum<k_max);
            dkeys[dnum] = k; dfuncs[dnum] = fdest;
            dnum++;
        }
        return k;
    }
    static int get_size()                       { return size; }
    static int get_max_align()                  { return max_align; }
// per-object
    Sac()                                       { if (cnum) call_funcs(cnum, ckeys, cfuncs); }
    ~Sac()                                      { if (dnum) call_funcs(dnum, dkeys, dfuncs); }
 private:
    static int size;
    static int max_align;
    // Cannot use Arrays here because their constructors might not yet be called at initialization time!
    static int cnum;
    static int ckeys[k_max];
    static Func cfuncs[k_max];
    static int dnum;
    static int dkeys[k_max];
    static Func dfuncs[k_max];
};

#define HH_MAKE_SAC(T)                                                                          \
    static void* operator new(size_t s) {                                                       \
        ASSERTX(s==sizeof(T));                                                                  \
        return assertx(hh::aligned_malloc(sizeof(T)-hh::BSac::k_dummy+hh::Sac<T>::get_size(),   \
                                          hh::Sac<T>::get_max_align()));                        \
    }                                                                                           \
    static void operator delete(void* p, size_t) { if (p) hh::aligned_free(p); }                \
    hh::Sac<T> sac

#define HH_MAKE_POOLED_SAC(T)                                                           \
    hh::Sac<T> sac;                                                                     \
    static void* operator new(size_t s) {                                               \
        ASSERTX(s==sizeof(T));                                                          \
        return pool.alloc_size(sizeof(T)-hh::BSac::k_dummy+hh::Sac<T>::get_size(),      \
                               hh::Sac<T>::get_max_align());                            \
    }                                                                                   \
    static void operator delete(void* p, size_t) {                                      \
        if (p) pool.free_size(p, sizeof(T)-hh::BSac::k_dummy+hh::Sac<T>::get_size());   \
    }                                                                                   \
    static void* operator new[](size_t) = delete;                                       \
    static void operator delete[](void*, size_t) = delete;                              \
    HH_POOL_ALLOCATION_3(T)

// size, cnum, and dnum intially zero
#define HH_SAC_INITIALIZATION(T)                                              \
    template<> int hh::Sac<T>::size = 0;                                      \
    template<> int hh::Sac<T>::max_align = alignof(T);                        \
    template<> int hh::Sac<T>::cnum = 0;                                      \
    template<> int hh::Sac<T>::ckeys[k_max] = {};                             \
    template<> hh::BSac::Func hh::Sac<T>::cfuncs[k_max] = {};                 \
    template<> int hh::Sac<T>::dnum = 0;                                      \
    template<> int hh::Sac<T>::dkeys[k_max] = {};                             \
    template<> hh::BSac::Func hh::Sac<T>::dfuncs[k_max] = {}

#define HH_SACABLE(T)                                                                   \
    static void sac_construct_##T(void* p) { new(p)T; }                                 \
    static void sac_destruct_##T(void* p) { dummy_use(p); static_cast<T*>(p)->~T(); }   \
    HH_EAT_SEMICOLON

template<typename T, typename T2> T& sac_access(T2& ob, int key) {
    return *reinterpret_cast<T*>(ob->sac.access(key));
}

#define HH_SAC_ALLOCATE_CD(sac, type)                                               \
    hh::Sac<sac>::allocate_cd<type>(&sac_construct_##type, &sac_destruct_##type)

#define HH_SAC_ALLOCATE(sac, type)                                            \
    hh::Sac<sac>::allocate<type>()

#define HH_SAC_ALLOCATE_FUNC(sac, type, accessfunc)                           \
    static int HH_ID(accessfunc) = HH_SAC_ALLOCATE(sac, type);                \
    type& accessfunc(sac* e) {                                                \
        return hh::sac_access<type>(e, HH_ID(accessfunc));                    \
    }                                                                         \
    HH_EAT_SEMICOLON

#define HH_SAC_ALLOCATE_CD_FUNC(sac, type, accessfunc)                        \
    static int HH_ID(accessfunc) = HH_SAC_ALLOCATE_CD(sac, type);             \
    type& accessfunc(sac* e) {                                                \
        return hh::sac_access<type>(e, HH_ID(accessfunc));                    \
    }                                                                         \
    HH_EAT_SEMICOLON

} // namespace hh
