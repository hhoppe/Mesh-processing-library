// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Array.h"              // ArrayView<>, CArrayView<>

namespace hh {

namespace details {
template<typename T, int n> struct Vec_base;
} // namespace details

// Allocated fixed-size 1D array with n elements of type T.
// Like std::array<T,n>, but with constructors and "empty base class optimization" support for n==0.
template<typename T, int n> class Vec : details::Vec_base<T,n> {
    static_assert(n>=0, "");
    using type = Vec<T,n>;
    using base = details::Vec_base<T,n>;
 public:
    Vec()                                       = default;
    Vec(CArrayView<T> ar)                       { assign(ar); }
    // Include arg0 to disambiguate from default constructor.  arg0 may be either const l-value or r-value reference.
    template<typename... Args> constexpr Vec(const T& arg0, Args&&... args1) noexcept
        : base(nullptr, arg0,            std::forward<Args>(args1)...) { }
    template<typename... Args> constexpr Vec(T&& arg0,      Args&&... args1) noexcept
        : base(nullptr, std::move(arg0), std::forward<Args>(args1)...) { }
#if 0
    // If I defined these operator=() functions, I would also have to define the default copy constructor, etc.
    // type& operator=(const type& ar)             = default;
    // type& operator=(type&& ar)                  = default;
    // type& operator=(CArrayView<T> ar)           { assign(ar); return *this; }
    // type& operator=(std::initializer_list<T> l) { return *this = CArrayView<T>(l); }
    // CONFIG=win : VS2013 does not create implicit move constructor and move assignment.
    //  I cannot define them =default, and if I define them, the class is no longer trivial.
    //  See SA in tVec.cpp
    // To allow class to be trivial, and to allow generation of implicit move constructor and assignment,
    //  it is safest to not include any copy-constructor, not even a default one.
    Vec(const type&)                            = default;
    type& operator=(const type& ar)             = default;
    Vec(type&&)                                 = default;
    type& operator=(type&&)                     = default;
    // Vec(type&& ar)                           { for_int(i, n) { a()[i] = std::move(ar.a()[i]); } }
    // type& operator=(type&& ar)               { for_int(i, n) { a()[i] = std::move(ar.a()[i]); } return *this; }
#endif
    constexpr int num() const                   { return n; }
    constexpr size_t size() const               { return n; }
    T&                 operator[](int i)        { return (HH_CHECK_BOUNDS(i, n), base::operator[](i)); }
    constexpr const T& operator[](int i) const  { return (HH_CHECK_BOUNDS(i, n), base::operator[](i)); }
    T& last()                                   { return (*this)[n-1]; }
    const T& last() const                       { return (*this)[n-1]; }
    bool ok(int i) const                        { return i>=0 && i<n; }
    type with(int i, const T& e) const          { type ar(*this); ar[i] = e; return ar; }
    void assign(CArrayView<T> ar)               { assign_i(ar); }
    constexpr type rev() const                  { return rev_aux(std::make_index_sequence<n>()); }
    // type with(int i, T e) const { type ar(*this); ar[i] = std::move(e); return ar; } // align error on T=Vector4
    // type with(int i, T e) &&                    { operator[](i) = std::move(e); return *this; } // C++14
    bool operator==(const type& p) const        { for_int(i, n) { if (!(a()[i]==p[i])) return false; } return true; }
    bool operator!=(const type& p) const        { return !(*this==p); }
    operator ArrayView<T>()                     { return view(); }
    operator CArrayView<T>() const              { return view(); }
    ArrayView<T>  view()                        { return ArrayView<T>(a(), n); }
    CArrayView<T> view() const                  { return CArrayView<T>(a(), n); }
    template<int s>       Vec<T,s>& head()       { return segment<s>(0); } // V(1, 2, 3).head<2>()==V(1, 2)
    template<int s> const Vec<T,s>& head() const { return segment<s>(0); }
    ArrayView<T> head(int s)                    { return segment(0, s); }
    CArrayView<T> head(int s) const             { return segment(0, s); }
    template<int s>       Vec<T,s>& tail()       { return segment<s>(n-s); } // V(1, 2, 3).tail<2>()==V(2, 3)
    template<int s> const Vec<T,s>& tail() const { return segment<s>(n-s); }
    ArrayView<T> tail(int s)                    { return segment(n-s, s); }
    CArrayView<T> tail(int s) const             { return segment(n-s, s); }
    template<int i, int s> Vec<T,s>& segment() { // V(1, 2, 3, 4).segment<2,1>==V(2, 3)
        static_assert(i>=0 && s>=0 && i+s<=n, ""); return *reinterpret_cast<Vec<T,s>*>(a()+i);
    }
    template<int i, int s> const Vec<T,s>& segment() const {
        static_assert(i>=0 && s>=0 && i+s<=n, ""); return *reinterpret_cast<const Vec<T,s>*>(a()+i);
    }
    template<int s> Vec<T,s>& segment(int i) { // V(1, 2, 3, 4).segment<2>(1)==V(2, 3)
        static_assert(s>0 && s<=n, ""); ASSERTXX(check(i, s)); return *reinterpret_cast<Vec<T,s>*>(a()+i);
    }
    template<int s> const Vec<T,s>& segment(int i) const {
        static_assert(s>0 && s<=n, ""); ASSERTXX(check(i, s)); return *reinterpret_cast<const Vec<T,s>*>(a()+i);
    }
    ArrayView<T> segment(int i, int s)          { ASSERTXX(check(i, s)); return  ArrayView<T>(a()+i, s); }
    CArrayView<T> segment(int i, int s) const   { ASSERTXX(check(i, s)); return CArrayView<T>(a()+i, s); }
    ArrayView<T>  slice(int ib, int ie)         { return segment(ib, ie-ib); }
    CArrayView<T> slice(int ib, int ie) const   { return segment(ib, ie-ib); }
    using value_type = T;
    using iterator = T*;
    using const_iterator = const T*;
    T* begin()                                  { return a(); }
    const T* begin() const                      { return a(); }
    T* end()                                    { return a()+n; }
    const T* end() const                        { return a()+n; }
    T* data()                                   { return a(); }
    const T* data() const                       { return a(); }
    static constexpr type all(const T& e)       { return all_aux(e, std::make_index_sequence<n>()); }
    // static constexpr type all(const T& e)    { return create([=](int) { return e; }); } // no lambda in constexpr
    static constexpr int Num = n;
    template<typename Func = T(int)> static type create(Func func) {
        return create_aux(func, std::make_index_sequence<n>());
    }
 private:
    // C++ requires empty classes to have nonzero size to ensure object identity.
    // Therefore, even with an empty struct, it is necessary that sizeof(Vec<T,0>)>0.
    // However, using the "empty base class optimization", a class derived from Vec<T,0> has zero space overhead.
    void assign_i(CArrayView<T> ar) {
        // ASSERTX(ar.num()==n); for_int(i, n) a()[i] = ar[i];
        ASSERTX(ar.num()==n); std::copy(ar.data(), ar.data()+n, a());
    }
    bool check(int i, int s) const { if (i>=0 && s>=0 && i+s<=n) return true; SHOW(i, s, n); return false; }
    template<size_t... Is> constexpr type rev_aux(std::index_sequence<Is...>) const {
        return type(base::operator[](n-1-Is)...);
    }
    template<size_t... Is> static constexpr type all_aux(const T& e, std::index_sequence<Is...>) {
        return type((void(Is), e)...);
    }
    template<typename Func = T(int), size_t... Is> static type create_aux(Func func, std::index_sequence<Is...>) {
        return type(func(Is)...);
    }
    using base::a;
    // Default operator=() and copy_constructor are safe.
};

template<typename T, int n> using SArray = Vec<T,n>; // backwards compatibility
template<typename T> using Vec0 = Vec<T,0>;
template<typename T> using Vec1 = Vec<T,1>;
template<typename T> using Vec2 = Vec<T,2>;
template<typename T> using Vec3 = Vec<T,3>;
template<typename T> using Vec4 = Vec<T,4>;

// Construct an Vec from an immediate list of elements, inferring the element type and array size automatically.
template<typename T, typename... Ts>
constexpr Vec<std::decay_t<T>, (1+sizeof...(Ts))> V(const T& t, Ts... ts) {
    return Vec<std::decay_t<T>, 1+sizeof...(ts)>(t,            std::forward<Ts>(ts)...);
}

// Construct an Vec from an immediate list of elements, inferring the element type and array size automatically.
template<typename T, typename... Ts>
constexpr Vec<std::decay_t<T>, (1+sizeof...(Ts))> V(T&& t,      Ts... ts) {
    return Vec<std::decay_t<T>, 1+sizeof...(ts)>(std::move(t), std::forward<Ts>(ts)...);
}

// Construct an Vec with two identical elements, e.g. twice(v)==V(v, v).
template<typename T> constexpr Vec2<T> twice(const T& v)     { return {v, v}; }

// Construct an Vec with three identical elements, e.g. thrice(v)==V(v, v, v).
template<typename T> constexpr Vec3<T> thrice(const T& v)    { return {v, v, v}; }

// Construct an Vec with identical elements, e.g. ntimes<4>(.5f)==V(.5f, .5f, .5f, .5f).
template<int n, typename T> constexpr Vec<T,n> ntimes(const T& v) { return Vec<T,n>::all(v); }

// Given container c, evaluate func() on each element (possibly changing the element type) and return new container.
template<typename T, int n, typename Func> auto map(const Vec<T,n>& c, Func func)
    -> Vec<decltype(func(T{})), n> {
    Vec<decltype(func(T{})), n> nc; for_int(i, n) { nc[i] = func(c[i]); } return nc;
}

namespace details {
template<typename T, typename... A> struct concat_n { static constexpr int value = T::Num + concat_n<A...>::value; };
template<typename T> struct concat_n<T> { static constexpr int value = T::Num; };
template<typename T, int n1, int n2, size_t... Is> constexpr
Vec<T, (n1+n2)> concat_aux(const Vec<T,n1>& a1, const Vec<T,n2>& a2, std::index_sequence<Is...>) {
    return Vec<T,n1+n2>( (Is<n1 ? a1[Is] : a2[Is-n1])... );
}
} // namespace details

// Concatenate several Vec's to create single Vec, e.g. concat(V(1, 2), V(3), V(4, 5))==V(1, 2, 3, 4, 5).
template<typename T, int n1, int n2, typename... A> constexpr
Vec<T, (n1+details::concat_n<Vec<T,n2>, A...>::value)>
concat(const Vec<T,n1>& a1, const Vec<T,n2>& a2, A... arr) {
    return concat(details::concat_aux(a1, a2, std::make_index_sequence<n1+n2>()), arr...);
}
template<typename T, int n1> constexpr Vec<T,n1> concat(const Vec<T,n1>& a1) { return a1; }


//----------------------------------------------------------------------------

namespace details {

template<typename T, int n> struct Vec_base { // allocates a member variable only if n>0.
    Vec_base()                                  = default;
    template<typename... Args> constexpr Vec_base(void*, Args&&... args) noexcept
        : _a{std::forward<Args>(args)...} { static_assert(sizeof...(args)==n, "#args"); }
    T _a[n];
    T* a() noexcept                             { return &_a[0]; }
    const T* a() const noexcept                 { return &_a[0]; }
    T&                 operator[](int i)        { return _a[i]; }
    constexpr const T& operator[](int i) const  { return _a[i]; } // operator[] needed for constexpr
};

template<typename T> struct Vec_base<T,0> {
    Vec_base()                                  = default;
    template<typename... Args> Vec_base(void*, Args&&... args) noexcept = delete;
    T* a() noexcept                             { return nullptr; }
    const T* a() const noexcept                 { return nullptr; }
    T&                 operator[](int)          { return *static_cast<T*>(nullptr); }
    constexpr const T& operator[](int) const    { return *static_cast<T*>(nullptr); }
    // No member variable at all.
};

} // namespace details


//----------------------------------------------------------------------------

template<typename T, int n> std::ostream& operator<<(std::ostream& os, const Vec<T,n>& ar) {
    if (has_ostream_eol<T>()) {
        os << "Vec<" << type_name<T>() << "," << n << "> {\n";
        for_int(i, n) { os << "  " << ar[i]; }
        return os << "}\n";
    } else {
        os << "[";
        for_int(i, n) { os << (i ? ", " : "") << ar[i]; }
        return os << "]";
    }
}
// Unlike would-be HH_DECLARE_OSTREAM_EOL(Vec<T,n>), it considers has_ostream_eol<T>.
template<typename T, int n> struct has_ostream_eol_aux<Vec<T,n>> {
    static constexpr bool value() { return has_ostream_eol<T>(); }
};


//----------------------------------------------------------------------------

// These Vec operations may be more efficient than similar ArrayView operations because
//  (1) n is known, and
//  (2) there is no heap allocation.

#if 0
// I could define the following:
template<typename T, typename T2, int n, typename RT = Vec<std::common_type_t<T,T2> > >
Vec<RT,n> operator+(const Vec<T,n>& a1, const Vec<T2,n>& a2) {
    Vec<RT,n> ar; for_int(i, n) { ar[i] = a1[i]+a2[i]; } return ar;
}
//  (Stroustrup book had dubious value_type:  =Vec<Common_type<Value_type<T>,Value_type<T2>>,n> )
#endif

// Set of functions common to Vec.h, SGrid.h, Array.h, Grid.h
// Note that RangeOp.h functions are valid here: mag2(), mag(), dist2(), dist(), dot(), is_zero(), compare().
// See also floor(), ceil(), abs() generalized to Vec<> in MathOp.h
#define TT template<typename T, int n>
#define G Vec<T,n>
#define F for_int(i, n)

// C++14 make all these constexpr

TT G operator+(const G& g1, const G& g2) { G g; F { g[i] = g1[i]+g2[i]; } return g; }
TT G operator-(const G& g1, const G& g2) { G g; F { g[i] = g1[i]-g2[i]; } return g; }
TT G operator*(const G& g1, const G& g2) { G g; F { g[i] = g1[i]*g2[i]; } return g; }
TT G operator/(const G& g1, const G& g2) { G g; F { g[i] = g1[i]/g2[i]; } return g; }
TT G operator%(const G& g1, const G& g2) { G g; F { g[i] = g1[i]%g2[i]; } return g; }

TT G operator+(const G& g1, T v) { G g; F { g[i] = g1[i]+v; } return g; }
TT G operator-(const G& g1, T v) { G g; F { g[i] = g1[i]-v; } return g; }
TT G operator*(const G& g1, T v) { G g; F { g[i] = g1[i]*v; } return g; }
TT G operator/(const G& g1, T v) { G g; F { g[i] = g1[i]/v; } return g; }
TT G operator%(const G& g1, T v) { G g; F { g[i] = g1[i]%v; } return g; }

TT G operator+(T v, const G& g1) { G g; F { g[i] = v+g1[i]; } return g; }
TT G operator-(T v, const G& g1) { G g; F { g[i] = v-g1[i]; } return g; }
TT G operator*(T v, const G& g1) { G g; F { g[i] = v*g1[i]; } return g; }
TT G operator/(T v, const G& g1) { G g; F { g[i] = v/g1[i]; } return g; }
TT G operator%(T v, const G& g1) { G g; F { g[i] = v%g1[i]; } return g; }

TT G& operator+=(G& g1, const G& g2) { F { g1[i] += g2[i]; } return g1; }
TT G& operator-=(G& g1, const G& g2) { F { g1[i] -= g2[i]; } return g1; }
TT G& operator*=(G& g1, const G& g2) { F { g1[i] *= g2[i]; } return g1; }
TT G& operator/=(G& g1, const G& g2) { F { g1[i] /= g2[i]; } return g1; }
TT G& operator%=(G& g1, const G& g2) { F { g1[i] %= g2[i]; } return g1; }

TT G& operator+=(G& g1, const T& v) { F { g1[i] += v; } return g1; }
TT G& operator-=(G& g1, const T& v) { F { g1[i] -= v; } return g1; }
TT G& operator*=(G& g1, const T& v) { F { g1[i] *= v; } return g1; }
TT G& operator/=(G& g1, const T& v) { F { g1[i] /= v; } return g1; }
TT G& operator%=(G& g1, const T& v) { F { g1[i] %= v; } return g1; }

TT G operator-(const G& g1) { G g; F { g[i] = -g1[i]; } return g; }

TT G min(const G& g1, const G& g2) { G g; F { g[i] = min(g1[i], g2[i]); } return g; }
TT G max(const G& g1, const G& g2) { G g; F { g[i] = max(g1[i], g2[i]); } return g; }

TT G interp(const G& g1, const G& g2, float f1 = 0.5f) {
    G g; F { g[i] = f1*g1[i]+(1.f-f1)*g2[i]; } return g;
}
TT G interp(const G& g1, const G& g2, const G& g3, float f1 = 1.f/3.f, float f2 = 1.f/3.f) {
    G g; F { g[i] = f1*g1[i]+f2*g2[i]+(1.f-f1-f2)*g3[i]; } return g;
}
TT G interp(const G& g1, const G& g2, const G& g3, const Vec3<float>& bary) {
    // Vec3<float> == Bary;  may have bary[0]+bary[1]+bary[2]!=1.f
    G g; F { g[i] = bary[0]*g1[i]+bary[1]*g2[i]+bary[2]*g3[i]; } return g;
}

#undef F
#undef G
#undef TT

} // namespace hh


//----------------------------------------------------------------------------

namespace std {
template<typename T, int n> struct hash<hh::Vec<T,n>> {
    size_t operator()(const hh::Vec<T,n>& ar) const {
        size_t h = hash<T>()(ar[0]); for_intL(i, 1, n) { h = h*2039+hash<T>()(ar[i]); } return h;
    }
};
} // namespace std
