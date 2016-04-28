// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include <type_traits>

#if defined(__GNUC__) && __GNUC__*100+__GNUC_MINOR__<410 // not yet C++11
#include <tr1/type_traits>
// see old forms in http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2010/n3142.html
namespace std {
template<typename T> struct is_trivially_copyable : tr1::has_trivial_copy<T> { };
template<typename T> struct is_trivially_default_constructible : has_trivial_default_constructor<T> { };
} // namespace std
#endif

#include "Array.h"
#include "Vec.h"
#include "RangeOp.h"
#include "Set.h"
using namespace hh;

namespace hh {

template<typename T, int N, size_t... Is>
CONSTEXPR Vec<T, (N-1)> V_rest_aux(const Vec<T,N>& u, std::index_sequence<Is...>) {
    return Vec<T,N-1>(u[Is+1]...);
}

template<typename T, int N>
CONSTEXPR Vec<T, (N-1)> V_rest(const Vec<T,N>& u) {
    return V_rest_aux(u, std::make_index_sequence<N-1>());
}


template<int n, typename T, int N, size_t... Is>
CONSTEXPR Vec<T,n> V_segment_aux(const Vec<T,N>& u, int i, std::index_sequence<Is...>) {
    return Vec<T,n>(u[Is+i]...);
}

template<int n, typename T, int N>
CONSTEXPR Vec<T,n> V_segment(const Vec<T,N>& u, int i) {
    return V_segment_aux<n, T, N>(u, i, std::make_index_sequence<n>());
}


template<int N> CONSTEXPR size_t V_dot_slow(const Vec<int,N>& u1, const Vec<int,N>& u2) {
    return u1[0]*u2[0]+V_dot_slow(V_rest(u1), V_rest(u2));
}
template<> inline CONSTEXPR size_t V_dot_slow<1>(const Vec<int,1>& u1, const Vec<int,1>& u2) {
    return u1[0]*u2[0];
}


template<typename T> CONSTEXPR T list_sum(const T& t0) { return t0; }
template<typename T, typename... U> CONSTEXPR T list_sum(const T& t0, U&&... ts) { return t0+list_sum(ts...); }


template<typename T, int N, size_t... Is>
CONSTEXPR T V_dot_aux(const Vec<T,N>& u1, const Vec<T,N>& u2, std::index_sequence<Is...>) {
    return list_sum(u1[Is]*u2[Is]...);
}

template<typename T, int N> CONSTEXPR T V_dot(const Vec<T,N>& u1, const Vec<T,N>& u2) {
    return V_dot_aux(u1, u2, std::make_index_sequence<N>());
}


namespace details {
template<int N, size_t... Is> CONSTEXPR Vec<int,N> V_iota_aux(std::index_sequence<Is...>) {
    return Vec<int,N>(int(Is)...);
}
template<int N, size_t... Is> CONSTEXPR Vec<int,N> V_rev_iota_aux(std::index_sequence<Is...>) {
    return Vec<int,N>(int(N-1-Is)...);
}
} // namespace details

template<int N> CONSTEXPR Vec<int,N> V_iota() {
    return details::V_iota_aux<N>(std::make_index_sequence<N>());
}

template<int N> CONSTEXPR Vec<int,N> V_rev_iota() {
    return details::V_rev_iota_aux<N>(std::make_index_sequence<N>());
}

} // namespace hh

int main() {
    struct S : Vec2<int> {
        // void f() const { SHOW(*this); }
    };
    struct S2 {
        int v[2];
    };
    struct S3 {
        S3()                                        = default;
        S3(int a, int b)                            { v[0] = a; v[1] = b; }
        int v[2];
    };
    struct S4 {
        float* p;
    };
    {
        { constexpr Vec<int,5> ar1 = concat(V(1), V(2), V(3, 4, 5)); SHOW(ar1); }
        { constexpr Vec<int,5> ar2 = concat(V(1, 2), V(3, 4, 5)); SHOW(ar2); }
        { constexpr Vec<int,5> ar3 = concat(V(1, 2), V(3, 4), V(5)); SHOW(ar3); }
        { constexpr Vec<int,5> ar4 = concat(V(1, 2, 3, 4, 5)); SHOW(ar4); }
    }
    {
        { constexpr Vec<int,2> ar = V(4, 5); constexpr int t5 = ar[1]; SHOW(t5); }
        { constexpr Vec<int,3> rest654 = V_rest(V(7, 6, 5, 4)); SHOW(rest654); }
        { constexpr Vec<int,2> segment54 = V_segment<2>(V(7, 6, 5, 4, 3), 2); SHOW(segment54); }
        { constexpr size_t dotslow = V_dot_slow(V(7, 5), V(3, 1)); SHOW(dotslow); }
        { constexpr size_t dot = V_dot(V(7, 5), V(3, 1)); SHOW(dot); }
        { constexpr Vec<int,5> iota5 = V_iota<5>(); SHOW(iota5); }
        { constexpr Vec<int,5> reviota5 = V_rev_iota<5>(); SHOW(reviota5); }
    }
    {
        Vec<char,2> magic { 'B', 'M' }; static_assert(sizeof(magic)==2, "");
        Vec<uchar,4> buf2 { uchar(0), uchar(0), uchar(0), uchar(0) }; static_assert(sizeof(buf2)==4, "");
    }
    {
        Vec2<float> p(1.f, 2.f), q(8.f, 7.f), r;
        SHOW(p);
        SHOW(q);
        SHOW(p-q);
        SHOW(p+q);
        SHOW(dot(p, q));
        SHOW(p*q);
        SHOW(mag(p));
        SHOW(2.f*p+3.f*q);
        r = p+q;
        SHOW(r);
        { using std::swap; swap(p, q); SHOW(p); SHOW(q); }
    }
    {
        Vec<int,3> a1; fill(a1, 1);
        Vec<int,3> a2 = { 2, 1, 3 };
        SHOW(product(a1), sum(a1));
        SHOW(product(a2), sum(a2));
        SHOW(a1==ntimes<3>(1));
        SHOW(a1==ntimes<3>(2));
        SHOW(a2==ntimes<3>(1));
        SHOW(a2.rev());
        SHOW(a2[2]);
    }
    {
        // can a class derived from Vec be automatically converted to a CArrayView argument? yes.
        auto func = [](CArrayView<int> a) {
            assertx(a.num()==2);
            SHOW(a[1]);
        };
        S s; s[0] = 11; s[1] = 12;
        func(s);
    }
    {
        Vec3<int> a1(3, 4, 5);
        SHOW(a1);
#if 0                           // fails because a1 contains int and 3.f is a float
        SHOW(a1/3.f);
        Vec3<float> a2 = a1/3.f;
        SHOW(a2);
#endif
    }
    {
        Vec2<Vec2<int>> pp{V(3, 4), Vec2<int>{5, 6}}; // test both ways
        SHOW(pp);
        SHOW((pp+V(10, 10)));
        SHOW((Vec2<int>(10, 10)-pp));
        SHOW(-pp);
    }
    {
        static_assert(std::is_standard_layout<Vec3<int>>::value==true, "");
        static_assert(std::is_trivially_default_constructible<Vec3<int>>::value==true, "");
        static_assert(std::is_trivially_copyable<Vec3<int>>::value==true, "");
        static_assert(std::is_trivially_copyable<S>::value==true, "");
        static_assert(std::is_trivially_copyable<S3>::value==true, "");
        static_assert(std::is_trivially_copyable<S4>::value==true, "");
        static_assert(std::is_trivial<Vec3<int>>::value==true, "");
        static_assert(std::is_trivial<S>::value==true, "");
        static_assert(std::is_trivial<S3>::value==true, "");
        static_assert(std::is_standard_layout<Vec3<int>>::value==true, "");
        static_assert(std::is_standard_layout<S>::value==true, "");
        static_assert(std::is_standard_layout<S3>::value==true, "");
#if 0
        // GCC and clang complain that both S3 and Vec3<int> have a user-declared constructor,
        // even though http://en.cppreference.com/w/cpp/types/is_pod
        // states that is_pod corresponds to is_trivial and is_standard_layout.
        // Visual Studio 2015 now also fails on these assertions.
        static_assert(std::is_pod<S3>::value==false, "");
        static_assert(std::is_pod<Vec3<int>>::value==false, "");
#endif
        static_assert(std::is_pod<S2>::value==true, "");
        static_assert(std::is_pod<S4>::value==true, "");
        S3 dummy1, dummy2(1, 2); dummy_use(dummy1, dummy2);
    }
    {
        Set<size_t> set;
        for_int(i, 100) for_int(j, 100) {
            size_t h = std::hash<Vec2<int>>()(V(i, j));
            // SHOW(h);
            assertx(set.add(h)); // all 10'000 are unique
        }
    }
    {
        constexpr auto ar1 = V(4, 5, 6);
        constexpr int v5 = ar1[1]; SHOW(v5);
        constexpr auto triple6a = Vec3<float>::all(6); SHOW(triple6a, type_name<decltype(triple6a)>());
        constexpr auto triple6b = ntimes<3>(6);        SHOW(triple6b, type_name<decltype(triple6b)>());
        constexpr auto triple6c = ntimes<3>(6.f);      SHOW(triple6c, type_name<decltype(triple6c)>());
        constexpr auto triple7 = thrice(7); SHOW(triple7);
        constexpr auto ntimes7 = ntimes<3>(7); SHOW(ntimes7);
        const auto vonly3is2 = ntimes<5>(0).with(3, 2); SHOW(vonly3is2);
    }
    {
        Vec<int,3> ar(1, 2, 3);
        my_zero(ar);
        SHOW(ar);
    }
    {
        auto ar = V(1, 2, 3, 4, 5);
        SHOW(ar.segment<3>(2));
        SHOW(ar.head<3>());
        SHOW(ar.tail<3>());
    }
    {
        auto ar0 = Vec<int,5>::create([](int i) { return i*5+3; });
        SHOW(ar0);
        auto ar1 = V(1, 2, 3, 4, 5);
        auto ar2 = map(ar1, [](int e) { return e*10; });
        SHOW(ar2);
    }
    if (0) {
        // VS2013: does not use move constructor on Vec1<P>.
        // This could be due to the fact that Vec1 contains std::array<P,1> which is non-trivial.
        // (If it contained P[1], it would still be non-trivial since P has explicit copy-constructuor.)
        // Thus, I may need to introduce explicit "=default" on move constructor and assignment in Vec,
        //  something that is not supported in VS2013.
        // Somehow gcc still defines implicit move constructor for Vec1<P>.
        struct P {
            P()                                 { SHOW("P::P()"); }
            ~P()                                { SHOW("P::~P()"); }
            P(const P&)                         { SHOW("P::P(const P&)"); }
            P(P&&)                              { SHOW("P::P(P&&)"); }
            P& operator=(const P&)              { SHOW("P::operator=(const P&)"); return *this; }
            // P& operator=(P&&)                   { SHOW("P::operator=(P&&)"); return *this; }
        };
        static_assert(std::is_trivial<P>::value==false, "");
        static_assert(std::is_trivial<Vec1<P>>::value==false, "");
        static_assert(std::is_trivially_copyable<P>::value==false, "");
        static_assert(std::is_trivially_copyable<Vec1<P>>::value==false, "");
        Vec1<P> s1;
        Vec1<P> s2 = s1;
        Vec1<P> s3 = std::move(s1);
    }
    {
#if defined(_MSC_VER) && _MSC_VER<1900
#else
        using SU2 = Vec2<unique_ptr<int>>;
        SU2 s1 = V(make_unique<int>(1), make_unique<int>(2));
        assertx(*s1[0]==1);
        assertx(*s1[1]==2);
        SU2 s2 = std::move(s1);
        assertx(*s2[0]==1);
        assertx(*s2[1]==2);
        assertx(!s1[0]);
        assertx(!s1[1]);
#endif
    }
}

namespace hh {
template class Vec<int,4>;
template class Vec<double,1>;
template class Vec<float,2>;
template class Vec<ushort,3>;
template class Vec<Vec<unsigned,2>, 2>;
template class Vec<void*,3>;
} // namespace hh
