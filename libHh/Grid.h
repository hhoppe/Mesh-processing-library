// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_GRID_H_
#define MESH_PROCESSING_LIBHH_GRID_H_

#include "Vec.h"
#include "RangeOp.h"
#include "Parallel.h"

#if 0
{
  Grid<2, int> grid(V(20, 10), -1);        // dimensions (ny = 20, nx = 10), and optional initial value
  int y, x;                                // individual coordinates
  Vec2<int> u = V(y, x);                   // array of coordinates (in Big Endian order)
  size_t i = ravel_index(grid.dims(), u);  // index into "vectorized" flat view of grid
  assertx(&grid[y][x] == &grid[u]);
  assertx(&grid(y, x) == &grid[u]);
  assertx(&grid.flat(i) == &grid[u]);
  assertx(i == ravel_index_list(grid.dims(), y, x));
  assertx(unravel_index(grid.dims(), i) == u);
  //
  grid[18][5] = 1;
  grid(18, 6) = 2;
  grid[{18, 7}] = 3;
  grid.flat(ravel_index(grid.dims(), V(18, 8))) = 4;
  for_int(y, grid.dim(0)) for_int(x, grid.dim(1)) grid[y][x] *= 2;  // iterate using individual coordinates
  for (const auto& u : range(grid.dims())) grid[u] *= 2;            // iterate using array of coordinate indices
  for_size_t(i, grid.size()) grid.flat(i) *= 2;                     // iterate using raster order (fastest)
  for (auto& e : grid) e *= 2;                                      // iterate over elements (also fastest)
}
#endif

namespace hh {

template<int D, typename T> class CGridView;
template<int D, typename T> class GridView;

namespace details {

template<int D, typename T> struct Grid_aux      { using CRet = CGridView<D-1,T>; using Ret = GridView<D-1,T>; };
template<typename T>        struct Grid_aux<1,T> { using CRet = const T&;         using Ret = T&; };
template<typename T>        struct Grid_aux<2,T> { using CRet = CArrayView<T>;    using Ret = ArrayView<T>; };
template<int D, typename T> struct Grid_get {
    static CGridView<D-1,T> cget(const T* a, const int* dims, int r);
    static  GridView<D-1,T>  get(      T* a, const int* dims, int r);
};
template<int DD, typename T> struct Grid_get2 {
    template<int n> static typename details::Grid_aux<DD+1,T>::CRet
    cget2(const T* a, const Vec<int,n+DD>& dims, const Vec<int,n>& u);
    template<int n> static typename details::Grid_aux<DD+1,T>::Ret
    get2(       T* a, const Vec<int,n+DD>& dims, const Vec<int,n>& u);
};

template<int D, typename T> struct nested_initializer_list {
    using type = std::initializer_list<typename nested_initializer_list<D-1,T>::type>;
};
template<typename T> struct nested_initializer_list<0,T> { using type = T; };
template<int D, typename T> using nested_initializer_list_t = typename nested_initializer_list<D,T>::type;

template<int D, typename T> struct nested_list_dims;
template<int D, typename T> struct nested_list_retrieve;

} // namespace details

// Given a coordinate u within a grid with dimensions dims, return the flat index in the linearized representation.
template<int D> constexpr size_t ravel_index(const Vec<int,D>& dims, const Vec<int,D>& u);

// Given a flat index i within a grid with dimensions dims, return its grid coordinates.
template<int D> Vec<int,D> unravel_index(const Vec<int,D>& dims, size_t i);

// Do the same for a list of coordinates.
template<int D, typename... A> constexpr size_t ravel_index_list(const Vec<int,D>& dims, A... dd); // dd... are int

// Find stride (in number of elements) of dimension d in the grid.
template<int D> size_t grid_stride(const Vec<int,D>& dims, int dim);

// Compute the size_t product of a small fixed set of numbers.
template<int D> size_t product_dims(const int* ar) {
    static_assert(D>0, "");
    size_t v = ar[0]; if (D>1) { for_intL(i, 1, D) { v *= ar[i]; } } return v;
}

// View of a contiguous D-dimensional grid with constant data of type T; often refers to a "const Grid<D,T>".
// Any or all of the dimensions may be zero.
template<int D, typename T> class CGridView {
    using type = CGridView<D,T>;
    static_assert(D>=1, "dimension problem");
 public:
    explicit CGridView(const T* a, const Vec<int,D>& dims) : _a(const_cast<T*>(a)), _dims(dims) { }
    CGridView(const type&)                      = default;
    explicit CGridView(CArrayView<T> ar)        : CGridView(ar.data(), V(ar.num())) { static_assert(D==1, ""); }
    // We cannot initialize from a nested_initializer_list_t because it is not a (linear) contiguous array T[].
    void reinit(type g)                         { *this = g; }
    template<typename T2> friend bool same_size(type g1, CGridView<D,T2> g2) { return g1.dims()==g2.dims(); }
    constexpr int ndim() const                  { return D; }
    const Vec<int,D>& dims() const              { return _dims; }
    int dim(int c) const                        { return _dims[c]; }
    size_t size() const                         { return product_dims<D>(_dims.data()); }
    typename details::Grid_aux<D,T>::CRet operator[](int r) const;
    template<int n> typename details::Grid_aux<D-n+1,T>::CRet operator[](const Vec<int,n>& u) const;
    template<typename... A> const T& operator()(A... dd) const; // dd... are int
    const T& flat(size_t i) const               { ASSERTXX(i<size()); return _a[i]; }
    bool ok(const Vec<int,D>& u) const {
        for_int(c, D) { if (u[c]<0 || u[c]>=_dims[c]) return false; } return true;
    }
    template<typename... A> bool ok(A... dd) const { return ok(V(dd...)); }
    bool map_inside(Vec<int,D>& u, const Vec<Bndrule,D>& bndrules) const { // ret: false outside Border
        for_int(c, D) { if (!map_boundaryrule_1D(u[c], _dims[c], bndrules[c])) return false; }
        return true;
    }
    const T& inside(const Vec<int,D>& u, const Vec<Bndrule,D>& bndrules) const {
        Vec<int,D> ut(u); assertx(map_inside(ut, bndrules)); return (*this)[ut];
    }
    const T& inside(const Vec<int,D>& u, const Vec<Bndrule,D>& bndrules, const T* bordervalue) const {
        Vec<int,D> ut(u); if (!map_inside(ut, bndrules)) { ASSERTX(bordervalue); return *bordervalue; }
        return (*this)[ut];
    }
    type slice(int ib, int ie) const { // view of grid truncated in 0th dimension
        assertx(ib>=0 && ib<=ie && ie<=_dims[0]); return type(_a+ib*grid_stride(_dims, 0), _dims.with(0, ie-ib));
    }
    using value_type = T;
    using iterator = const T*;
    using const_iterator = const T*;
    const T* begin() const                      { return _a; }
    const T* end() const                        { return _a+size(); }
    const T* data() const                       { return _a; }
    CArrayView<T> array_view() const            { return CArrayView<T>(data(), narrow_cast<int>(size())); }
    // For implementation of Matrix (D==2):
    int ysize() const                           { static_assert(D==2, ""); return dim(0); }
    int xsize() const                           { static_assert(D==2, ""); return dim(1); }
    bool map_inside(int& y, int& x, Bndrule bndrule) const; // ret: false if bndrule==Border and i is outside
    const T& inside(int y, int x, Bndrule bndrule) const {
        bool b = map_inside(y, x, bndrule); ASSERTX(b); return (*this)[y][x];
    }
    const T& inside(int y, int x, Bndrule bndrule, const T* bordervalue) const;
 protected:
    T* _a {nullptr};            // [0, _dims[0]-1] * [0, _dims[1]-1] * ... * [0, _dims[D-1]-1]
    Vec<int,D> _dims {ntimes<D>(0)};
    bool check(int r) const             { if (r>=0 && r<_dims[0]) return true; SHOW(r, _dims); return false; }
    bool check(const Vec<int,D>& u) const       { if (ok(u)) return true; SHOW(u, _dims); return false; }
    CGridView()                                 { }
    type& operator=(const type&)                = default;
};

// View of a contiguous D-dimensional grid with modifiable data of type T; often refers to a Grid<D,T>.
template<int D, typename T> class GridView : public CGridView<D,T> {
    using type = GridView<D,T>;
    using base = CGridView<D,T>;
 public:
    explicit GridView(T* a, const Vec<int,D>& dims) : base(a, dims) { } // stop recursion if ArrayView==GridView
    GridView(const type&)                       = default;          // because it has explicit copy assignment
    explicit GridView(ArrayView<T> ar)          : GridView(ar.data(), V(ar.num())) { static_assert(D==1, ""); }
    void reinit(type g)                         { *this = g; }
    using base::size;
    typename details::Grid_aux<D,T>::Ret  operator[](int r);
    typename details::Grid_aux<D,T>::CRet operator[](int r) const { return base::operator[](r); }
    template<int n> typename details::Grid_aux<D-n+1,T>::Ret  operator[](const Vec<int,n>& u);
    template<int n> typename details::Grid_aux<D-n+1,T>::CRet operator[](const Vec<int,n>& u) const;
    template<typename... A> T& operator()(A... dd); // dd... are int
    template<typename... A> const T& operator()(A... dd) const;
    T&       flat(size_t i)                     { ASSERTXX(i<size()); return _a[i]; }
    const T& flat(size_t i) const               { return base::flat(i); }
    using base::map_inside;
    T& inside(const Vec<int,D>& u, const Vec<Bndrule,D>& bndrules) {
        Vec<int,D> ut(u); bool b = map_inside(ut, bndrules); ASSERTX(b); return (*this)[ut];
    }
    const T& inside(const Vec<int,D>& u, const Vec<Bndrule,D>& bndrules) const {
        return base::inside(u, bndrules);
    }
    const T& inside(const Vec<int,D>& u, const Vec<Bndrule,D>& bndrules, const T* bordervalue) const {
        return base::inside(u, bndrules, bordervalue);
    }
    type slice(int ib, int ie) { // view of grid truncated in 0th dimension
        assertx(ib>=0 && ib<=ie && ie<=_dims[0]); return type(_a+ib*grid_stride(_dims, 0), _dims.with(0, ie-ib));
    }
    base slice(int ib, int ie) const            { return base::slice(ib, ie); }
    void assign(CGridView<D,T> g);
    using iterator = T*;
    using const_iterator = const T*;
    T* begin()                                  { return _a; }
    const T* begin() const                      { return _a; }
    T* end()                                    { return _a+size(); }
    const T* end() const                        { return _a+size(); }
    T* data()                                   { return _a; }
    const T* data() const                       { return _a; }
    ArrayView<T> array_view()                   { return ArrayView<T>(data(), narrow_cast<int>(size())); }
    CArrayView<T> array_view() const            { return CArrayView<T>(data(), narrow_cast<int>(size())); }
    // For implementation of Matrix (D==2):
    T& inside(int y, int x, Bndrule bndrule1) {
        // (renamed bndrule to bndrule1 due to buggy VS2015 warning about shadowed variable)
        bool b = map_inside(y, x, bndrule1); ASSERTX(b); return (*this)[y][x];
    }
    const T& inside(int y, int x, Bndrule bndrule) const {
        bool b = map_inside(y, x, bndrule); ASSERTX(b); return (*this)[y][x];
    }
    const T& inside(int y, int x, Bndrule bndrule, const T* bordervalue) const;
    void reverse_y() {
        static_assert(D==2, ""); const int ny = this->ysize();
        parallel_for_each(range(ny/2), [&](const int y) {
            swap_ranges((*this)[y], (*this)[ny-1-y]);
        }, this->xsize()*2);
    }
    void reverse_x() {
        static_assert(D==2, ""); const int ny = this->ysize();
        parallel_for_each(range(ny), [&](const int y) { reverse((*this)[y]); }, this->xsize()*2);
    }
 protected:
    using base::_a; using base::_dims;
    using base::check;
    GridView()                                  = default;
    type& operator=(const type&)                = default;
};

// Create a CGridView<1,T> referencing the single specified element.
template<typename T> CGridView<1,T> CGrid1View(const T& e) { return CGridView<1,T>(&e, V(1)); }

// Create an GridView<1,T> referencing the single specified element.
template<typename T> GridView<1,T> Grid1View(T& e) { return GridView<1,T>(&e, V(1)); }

// Heap-allocated D-dimensional contiguous grid.  Any or all of the dimensions may be zero.
// Grid(10, 20) or Grid(V(10, 20)) both create a 10x20 grid, whereas Grid({10, 20}) creates a 1x2 grid.
template<int D, typename T> class Grid : public GridView<D,T> {
    using base = GridView<D,T>;
    using type = Grid<D,T>;
    using initializer_type = details::nested_initializer_list_t<D,T>;
    using nested_dims = details::nested_list_dims<D,T>;
    using nested_retrieve = details::nested_list_retrieve<D,T>;
 public:
    Grid()                                      = default;
    explicit Grid(const Vec<int,D>& dims)       { init(dims); }
    template<typename... A> explicit Grid(int d0, A... dr) { init(d0, dr...); }
    explicit Grid(const Vec<int,D>& dims, const T& v) { init(dims, v); }
    explicit Grid(const type& g)                : Grid(g.dims()) { base::assign(g); }
    explicit Grid(CGridView<D,T> g)             : Grid(g.dims()) { base::assign(g); }
    Grid(type&& g) noexcept                     { swap(*this, g); } // not default
    Grid(initializer_type l)                    : Grid() { *this = l; }
    ~Grid()                                     { clear(); }
    type& operator=(CGridView<D,T> g)           { init(g.dims()); base::assign(g); return *this; }
    type& operator=(const type& g)              { init(g.dims()); base::assign(g); return *this; }
    type& operator=(initializer_type l)         { init(nested_dims()(l)); nested_retrieve()(*this, l); return *this; }
    type& operator=(type&& g) noexcept          { clear(); swap(*this, g); return *this; }
    template<typename... A> void init(int d0, A... dr) { init(Vec<int,D>(d0, dr...)); }
    using base::size;
    void init(const Vec<int,D>& dims) {
        if (dims==_dims) return;
        assertx(min(dims)>=0);
        size_t vol = product_dims<D>(dims.data());
        if (vol!=size()) { delete[] _a; _a = vol ? new T[vol] : nullptr; }
        // if (vol!=size()) { aligned_delete<T>(_a); _a = vol ? aligned_new<T>(vol) : nullptr; }
        _dims = dims;
    }
    void init(const Vec<int,D>& dims, const T& v) { init(dims); fill(*this, v); }
    void clear()                                { if (_a) init(ntimes<D>(0)); }
    friend void swap(Grid& l, Grid& r) noexcept { using std::swap; swap(l._a, r._a); swap(l._dims, r._dims); }
    void special_reduce_dim0(int i)             { assertx(i>=0 && i<=_dims[0]); _dims[0] = i; }
    // (must declare template parameters because these functions access private _a of <D-1,T> and <D+1,T>)
    template<int DD, typename TT> friend Grid<DD-1,TT> reduce_grid_rank(Grid<DD,TT>&& grid);
    template<int DD, typename TT> friend Grid<DD+1,TT> increase_grid_rank(Grid<DD,TT>&& grid);
 private:
    using base::_a; using base::_dims;
    using base::reinit;         // hide it
};

// Given container c, evaluate func() on each element (possibly changing the element type) and return new container.
template<int D, typename T, typename Func> auto map(CGridView<D,T>& c, Func func)
    -> Grid<D, decltype(func(std::declval<T>()))> { // VS2015 Intellisense does not like T{} here
    Grid<D, decltype(func(T{}))> nc(c.dims()); for_size_t(i, c.size()) { nc.flat(i) = func(c.flat(i)); }
    return nc;
}

//----------------------------------------------------------------------------

namespace details {
inline constexpr size_t ravel_index_aux2(std::pair<int,int> p) { return p.second; }
template<typename... P> constexpr size_t ravel_index_aux2(std::pair<int,int> p0, P... ps) {
    return (ASSERTXX(p0.second>=0 && p0.second<p0.first),
            ravel_index_aux2(ps...)*p0.first+p0.second);
}

template<int D, size_t... Is>
constexpr size_t ravel_index_aux1(const Vec<int,D>& dims, const Vec<int,D>& u, std::index_sequence<Is...>) {
    return ravel_index_aux2(std::make_pair(dims[D-1-Is], u[D-1-Is])...);
}
} // namespace details
template<int D> constexpr size_t ravel_index(const Vec<int,D>& dims, const Vec<int,D>& u) {
    return details::ravel_index_aux1(dims, u, std::make_index_sequence<D>());
}
template<> inline constexpr size_t ravel_index(const Vec3<int>& dims, const Vec3<int>& u) {
    return (HH_CHECK_BOUNDS(u[0], dims[0]), HH_CHECK_BOUNDS(u[1], dims[1]), HH_CHECK_BOUNDS(u[2], dims[2]),
            (intptr_t{u[0]}*dims[1]+u[1])*dims[2]+u[2]);
}
template<> inline constexpr size_t ravel_index(const Vec2<int>& dims, const Vec2<int>& u) {
    return (HH_CHECK_BOUNDS(u[0], dims[0]), HH_CHECK_BOUNDS(u[1], dims[1]),
            intptr_t{u[0]}*dims[1]+u[1]);
}
template<> inline constexpr size_t ravel_index(const Vec1<int>& dims, const Vec1<int>& u) {
    return (HH_CHECK_BOUNDS(u[0], dims[0]), void(dims), // dummy_use(dims) returns void so cannot be constexpr
            u[0]);
}
template<> inline constexpr size_t ravel_index(const Vec<int,0>& dims, const Vec<int,0>& u) {
    return (void(dims), void(u), 0);    // used in GridView[V()]
}

template<int D> Vec<int,D> unravel_index(const Vec<int,D>& dims, size_t i) {
    static_assert(D>=1, "");
    Vec<int,D> u;
    for (int d = D-1; d>=1; --d) {
        size_t t = i/dims[d];
        u[d] = narrow_cast<int>(i-t*dims[d]); // remainder
        i = t;
    }
    u[0] = narrow_cast<int>(i);
    return u;
}

namespace details {
template<int D> struct ravel_index_rec {
    template<typename... A> constexpr size_t operator()(size_t v, const Vec<int,D>& dims, int d0, A... dd) const {
        return (HH_CHECK_BOUNDS(d0, dims[0]),
                ravel_index_rec<D-1>()((v+d0)*dims[1], dims.template segment<D-1>(1), dd...));
    }
};
template<> struct ravel_index_rec<1> {
    constexpr size_t operator()(size_t v, const Vec<int,1>& dims, int d0) const {
        return (HH_CHECK_BOUNDS(d0, dims[0]), void(dims),
                v+d0);
    }
};
template<int D> struct ravel_index_list_aux {
    template<typename... A> constexpr size_t operator()(const Vec<int,D>& dims, int d0, A... dd) const {
        return (HH_CHECK_BOUNDS(d0, dims[0]),
                ravel_index_rec<D-1>()(intptr_t{d0}*dims[1], dims.template segment<D-1>(1), dd...));
    }
};
template<> struct ravel_index_list_aux<1> {
    constexpr size_t operator()(const Vec1<int>& dims, int d0) const {
        return (HH_CHECK_BOUNDS(d0, dims[0]), void(dims),
                d0);
    }
};
template<> struct ravel_index_list_aux<2> {
    constexpr size_t operator()(const Vec2<int>& dims, int d0, int d1) const {
        return (HH_CHECK_BOUNDS(d0, dims[0]), HH_CHECK_BOUNDS(d1, dims[1]),
                intptr_t{d0}*dims[1]+d1);
    }
};
template<> struct ravel_index_list_aux<3> {
    constexpr size_t operator()(const Vec3<int>& dims, int d0, int d1, int d2) const {
        return (HH_CHECK_BOUNDS(d0, dims[0]), HH_CHECK_BOUNDS(d1, dims[1]), HH_CHECK_BOUNDS(d2, dims[2]),
                (intptr_t{d0}*dims[1]+d1)*dims[2]+d2);
    }
};
} // namespace details
template<int D, typename... A> constexpr size_t ravel_index_list(const Vec<int,D>& dims, A... dd) {
    return details::ravel_index_list_aux<D>()(dims, dd...);
}

template<int D> size_t grid_stride(const Vec<int,D>& dims, int dim) {
    ASSERTXX(dim>=0 && dim<D);
    if (dim+1>=D) return 1;
    size_t i = dims[dim+1]; for_intL(d, dim+2, D) { i *= dims[d]; } return i;
}

//----------------------------------------------------------------------------

namespace details {
template<typename T> struct Grid_get<1,T> {
    static const T& cget(const T* a, const int*, int r) { return a[r]; }
    static       T&  get(      T* a, const int*, int r) { return a[r]; }
};
template<typename T> struct Grid_get<2,T> {
    static CArrayView<T> cget(const T* a, const int* dims, int r) {
        return CArrayView<T>(a+intptr_t{r}*dims[1], dims[1]);
    }
    static  ArrayView<T>  get(T* a, const int* dims, int r) {
        return  ArrayView<T>(a+intptr_t{r}*dims[1], dims[1]);
    }
};
template<typename T> struct Grid_get<3,T> { // specialization for speedup
    static CGridView<2,T> cget(const T* a, const int* dims, int r) {
        return CGridView<2,T>(a+intptr_t{r}*dims[1]*dims[2], Vec2<int>(dims[1], dims[2]));
    }
    static  GridView<2,T>  get(T* a, const int* dims, int r) {
        return  GridView<2,T>(a+intptr_t{r}*dims[1]*dims[2], Vec2<int>(dims[1], dims[2]));
    }
};
template<int D, typename T> CGridView<D-1,T> details::Grid_get<D,T>::cget(const T* a, const int* dims, int r) {
    static_assert(D>=4, "");
    return CGridView<D-1,T>(a+r*product_dims<D-1>(&dims[1]), CArrayView<int>(dims+1, D-1));
}
//
template<int D, typename T> GridView<D-1,T> details::Grid_get<D,T>::get(T* a, const int* dims, int r) {
    static_assert(D>=4, "");
    return  GridView<D-1,T>(a+r*product_dims<D-1>(&dims[1]), CArrayView<int>(dims+1, D-1));
}

template<typename T> struct Grid_get2<0,T> {
    template<int n> static const T& cget2(const T* a, const Vec<int,n>& dims, const Vec<int,n>& u) {
        return a[ravel_index(dims, u)];
    }
    template<int n> static       T&  get2(      T* a, const Vec<int,n>& dims, const Vec<int,n>& u) {
        return a[ravel_index(dims, u)];
    }
};
template<typename T> struct Grid_get2<1,T> {
    template<int n> static CArrayView<T> cget2(const T* a, const Vec<int,n+1>& dims, const Vec<int,n>& u) {
        return CArrayView<T>(a+ravel_index(dims.template head<n>(), u)*dims[n], dims[n]);
    }
    template<int n> static  ArrayView<T>  get2(      T* a, const Vec<int,n+1>& dims, const Vec<int,n>& u) {
        return ArrayView<T>(a+ravel_index(dims.template head<n>(), u)*dims[n], dims[n]);
    }
};
template<int DD, typename T> template<int n> typename details::Grid_aux<DD+1,T>::CRet
details::Grid_get2<DD,T>::cget2(const T* a, const Vec<int,n+DD>& dims, const Vec<int,n>& u) {
    using T2 = typename details::Grid_aux<DD+1,T>::CRet; // Grid dimension D = n+DD;
    return T2(a+ravel_index(dims.template head<n>(), u)*product_dims<DD>(dims.data()+n),
              CArrayView<int>(dims.data()+n, DD));
}
template<int DD, typename T> template<int n> typename details::Grid_aux<DD+1,T>::Ret
details::Grid_get2<DD,T>::get2(T* a, const Vec<int,n+DD>& dims, const Vec<int,n>& u) {
    using T2 = typename details::Grid_aux<DD+1,T>::Ret; // Grid dimension D = n+DD;
    return T2(a+ravel_index(dims.template head<n>(), u)*product_dims<DD>(dims.data()+n),
              CArrayView<int>(dims.data()+n, DD));
}

} // namespace details

//----------------------------------------------------------------------------

template<int D, typename T> typename details::Grid_aux<D,T>::CRet CGridView<D,T>::operator[](int r) const {
    ASSERTXX(check(r)); return details::Grid_get<D,T>::cget(_a, _dims.data(), r);
}

template<int D, typename T> template<int n>
typename details::Grid_aux<D-n+1,T>::CRet CGridView<D,T>::operator[](const Vec<int,n>& u) const {
    static_assert(n>=0 && n<=D, "");
    return details::Grid_get2<D-n,T>::template cget2<n>(_a, _dims, u);
}

template<int D, typename T> template<typename... A> const T& CGridView<D,T>::operator()(A... dd) const {
    static_assert(sizeof...(dd)==D, ""); return _a[ravel_index_list(_dims, dd...)];
}

template<int D, typename T> bool CGridView<D,T>::map_inside(int& y, int& x, Bndrule bndrule1) const {
    // (renamed bndrule to bndrule1 due to buggy VS2015 warning about shadowed variable)
    static_assert(D==2, "");
    return map_boundaryrule_1D(y, ysize(), bndrule1) && map_boundaryrule_1D(x, xsize(), bndrule1);
}

template<int D, typename T> const T& CGridView<D,T>::inside(int y, int x, Bndrule bndrule,
                                                            const T* bordervalue) const {
    if (!map_inside(y, x, bndrule)) { ASSERTX(bordervalue); return *bordervalue; }
    return (*this)[y][x];
}

//----------------------------------------------------------------------------

template<int D, typename T> typename details::Grid_aux<D,T>::Ret GridView<D,T>::operator[](int r) {
    ASSERTXX(check(r)); return details::Grid_get<D,T>::get(_a, _dims.data(), r);
}

template<int D, typename T> template<int n>
typename details::Grid_aux<D-n+1,T>::Ret GridView<D,T>::operator[](const Vec<int,n>& u) {
    static_assert(n>=0 && n<=D, "");
    return details::Grid_get2<D-n,T>::template get2<n>(_a, _dims, u);
}

template<int D, typename T> template<int n>
typename details::Grid_aux<D-n+1,T>::CRet GridView<D,T>::operator[](const Vec<int,n>& u) const {
    static_assert(n>=0 && n<=D, "");
    return details::Grid_get2<D-n,T>::template cget2<n>(_a, _dims, u);
}

template<int D, typename T> template<typename... A> T& GridView<D,T>::operator()(A... dd) {
    static_assert(sizeof...(dd)==D, ""); return _a[ravel_index_list(_dims, dd...)];
}

template<int D, typename T> template<typename... A> const T& GridView<D,T>::operator()(A... dd) const {
    static_assert(sizeof...(dd)==D, ""); return _a[ravel_index_list(_dims, dd...)];
}

template<int D, typename T> const T& GridView<D,T>::inside(int y, int x, Bndrule bndrule,
                                                           const T* bordervalue) const {
    if (!map_inside(y, x, bndrule)) { ASSERTX(bordervalue); return *bordervalue; }
    return (*this)[y][x];
}

template<int D, typename T> void GridView<D,T>::assign(CGridView<D,T> g) {
    assertx(same_size(*this, g));
    if (g.data()==data()) return;
    // for_size_t(i, size()) _a[i] = g.flat(i);
    // if (_a) std::memcpy(_a, g.begin(), (g.end()-g.begin())*sizeof(T)); // no faster, and unsafe for general T
    if (_a) std::copy(g.begin(), g.end(), _a);
}

template<int D, typename T> Grid<D-1,T> reduce_grid_rank(Grid<D,T>&& grid) {
    assertx(grid.dim(0)==1); // perhaps could be <=1
    Grid<D-1,T> ngrid;
    using std::swap; swap(ngrid._a, grid._a);
    ngrid._dims = grid._dims.template tail<D-1>(); grid._dims = ntimes<D>(0);
    return ngrid;
}

template<int D, typename T> Grid<D+1,T> increase_grid_rank(Grid<D,T>&& grid) {
    Grid<D+1,T> ngrid;
    using std::swap; swap(ngrid._a, grid._a);
    ngrid._dims = concat(V(1), grid._dims); grid._dims = ntimes<D>(0);
    return ngrid;
}

//----------------------------------------------------------------------------

namespace details {
template<int D, typename T> struct nested_list_dims {
    Vec<int,D> operator()(nested_initializer_list_t<D,T> l) const {
        Vec<int,D> dims; dims[0] = narrow_cast<int>(l.size()); assertx(dims[0]>0);
        dims.template segment<D-1>(1) = nested_list_dims<D-1,T>()(l.begin()[0]);
        return dims;
    }
};
template<typename T> struct nested_list_dims<1,T> {
    Vec<int,1> operator()(std::initializer_list<T> l) const { return V(narrow_cast<int>(l.size())); }
};

template<int D, typename T> struct nested_list_retrieve {
    void operator()(GridView<D,T> grid, nested_initializer_list_t<D,T> l) const {
        assertx(grid.dim(0)==narrow_cast<int>(l.size()));
        for_int(i, narrow_cast<int>(l.size()))
            nested_list_retrieve<D-1,T>()(grid[i], l.begin()[i]);
    }
    void operator()(ArrayView<T> grid, nested_initializer_list_t<1,T> l) const { // ArrayView<T>!=GridView<1,T>
        assertx(grid.num()==narrow_cast<int>(l.size()));
        for_int(i, narrow_cast<int>(l.size())) grid[i] = l.begin()[i];
    }
};
template<typename T> struct nested_list_retrieve<0,T> {
    void operator()(T& gridv, const T& lv) const { gridv = lv; }
};
} // namespace details

//----------------------------------------------------------------------------

namespace details {
template<int D, typename T> struct grid_output {
    std::ostream& operator()(std::ostream& os, CGridView<D,T> g) const {
        os << "Grid<" << type_name<T>() << ">(";
        for_int(i, g.dims().num()) { os << (i ? ", " : "") << g.dims()[i]; }
        os << ") {\n";
        for (const auto& p : range(g.dims())) {
            os << "  " << p << " = " << g[p] << (has_ostream_eol<T>() ? "" : "\n");
        }
        return os << "}\n";
    }
};
template<typename T> struct grid_output<2,T> {
    std::ostream& operator()(std::ostream& os, CGridView<2,T> g) const {
        const int ny = g.dim(0), nx = g.dim(1);
        os << "Matrix<" << type_name<T>() << ">(" << ny << ", " << nx << ") {\n";
        for_int(y, ny) {
            if (has_ostream_eol<T>()) {
                for_int(x, nx) { os << sform("  [%d, %d] = ", y, x) << g[y][x]; }
            } else {
                os << " ";
                for_int(x, nx) { os << " " << g[y][x]; }
                os << "\n";
            }
        }
        return os << "}\n";
    }
};
template<typename T> struct grid_output<1,T> {
    std::ostream& operator()(std::ostream& os, CGridView<1,T> g) const {
        const int n = g.dim(0);
        os << "Array<" << type_name<T>() << ">(" << n << ") {\n";
        for_int(i, n) {
            os << "  " << g[i] << (has_ostream_eol<T>() ? "" : "\n");
        }
        return os << "}\n";
    }
};
} // namespace details
template<int D, typename T> std::ostream& operator<<(std::ostream& os, CGridView<D,T> g) {
    return details::grid_output<D,T>()(os, g);
}
template<int D, typename T> HH_DECLARE_OSTREAM_EOL(CGridView<D,T>);
template<int D, typename T> HH_DECLARE_OSTREAM_EOL(GridView<D,T>); // implemented by CGridView<D,T>
template<int D, typename T> HH_DECLARE_OSTREAM_EOL(Grid<D,T>);     // implemented by CGridView<D,T>

//----------------------------------------------------------------------------

// Set of functions common to Vec.h, SGrid.h, Array.h, Grid.h
// Note that RangeOp.h functions are valid here: mag2(), mag(), dist2(), dist(), dot(), is_zero(), compare().
#define TT template<int D, typename T>
#define G Grid<D, T>
#define CG CGridView<D, T>
#define SS ASSERTX(same_size(g1, g2))
#define F(g) for_size_t(i, g.size())
#define PF(g, code) parallel_for_each(range(g.size()), [&](const size_t i) { code; }, 1)
// clang-format off

TT G operator+(CG g1, CG g2) { SS; G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) + g2.flat(i); } return g; }
TT G operator-(CG g1, CG g2) { SS; G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) - g2.flat(i); } return g; }
TT G operator*(CG g1, CG g2) { SS; G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) * g2.flat(i); } return g; }
TT G operator/(CG g1, CG g2) { SS; G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) / g2.flat(i); } return g; }
TT G operator%(CG g1, CG g2) { SS; G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) % g2.flat(i); } return g; }

TT G operator+(CG g1, const T& e) { G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) + e; } return g; }
TT G operator-(CG g1, const T& e) { G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) - e; } return g; }
TT G operator*(CG g1, const T& e) { G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) * e; } return g; }
TT G operator/(CG g1, const T& e) { G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) / e; } return g; }
TT G operator%(CG g1, const T& e) { G g(g1.dims()); F(g) { g.flat(i) = g1.flat(i) % e; } return g; }

TT G operator+(const T& e, CG g1) { G g(g1.dims()); F(g) { g.flat(i) = e + g1.flat(i); } return g; }
TT G operator-(const T& e, CG g1) { G g(g1.dims()); F(g) { g.flat(i) = e - g1.flat(i); } return g; }
TT G operator*(const T& e, CG g1) { G g(g1.dims()); F(g) { g.flat(i) = e * g1.flat(i); } return g; }
TT G operator/(const T& e, CG g1) { G g(g1.dims()); F(g) { g.flat(i) = e / g1.flat(i); } return g; }
TT G operator%(const T& e, CG g1) { G g(g1.dims()); F(g) { g.flat(i) = e % g1.flat(i); } return g; }

// Parallelized and optimized, for Multigrid<>.
TT GridView<D,T> operator+=(GridView<D,T> g1, CG g2) {
    SS; T* a = g1.data(); const T* b = g2.data(); PF(g1, a[i] += b[i]); return g1;
}
TT GridView<D,T> operator-=(GridView<D,T> g1, CG g2) { SS; F(g1) { g1.flat(i) -= g2.flat(i); } return g1; }
TT GridView<D,T> operator*=(GridView<D,T> g1, CG g2) { SS; PF(g1, g1.flat(i) *= g2.flat(i)); return g1; }
TT GridView<D,T> operator/=(GridView<D,T> g1, CG g2) { SS; F(g1) { g1.flat(i) /= g2.flat(i); } return g1; }
TT GridView<D,T> operator%=(GridView<D,T> g1, CG g2) { SS; F(g1) { g1.flat(i) %= g2.flat(i); } return g1; }

TT GridView<D,T> operator+=(GridView<D,T> g1, const T& e) { F(g1) { g1.flat(i) += e; } return g1; }
TT GridView<D,T> operator-=(GridView<D,T> g1, const T& e) { F(g1) { g1.flat(i) -= e; } return g1; }
TT GridView<D,T> operator*=(GridView<D,T> g1, const T& e) { F(g1) { g1.flat(i) *= e; } return g1; }
TT GridView<D,T> operator/=(GridView<D,T> g1, const T& e) { F(g1) { g1.flat(i) /= e; } return g1; }
TT GridView<D,T> operator%=(GridView<D,T> g1, const T& e) { F(g1) { g1.flat(i) %= e; } return g1; }

TT GridView<D,T> operator-(GridView<D,T> g1) { G g(g1.dims()); F(g) { g.flat(i) = -g1.flat(i); } return g; }

TT G min(CG g1, CG g2) { SS; G g(g1.dims()); F(g) { g.flat(i) = min(g1.flat(i), g2.flat(i)); } return g; }
TT G max(CG g1, CG g2) { SS; G g(g1.dims()); F(g) { g.flat(i) = max(g1.flat(i), g2.flat(i)); } return g; }

TT G interp(CG g1, CG g2, float f1 = 0.5f) {
  SS; G g(g1.dims()); F(g) { g.flat(i) = f1 * g1.flat(i) + (1.f - f1) * g2.flat(i); } return g;
}
TT G interp(CG g1, CG g2, CG g3, float f1 = 1.f / 3.f, float f2 = 1.f / 3.f) {
  ASSERTX(same_size(g1, g2) && same_size(g1, g3));
  G g(g1.dims()); F(g) { g.flat(i) = f1 * g1.flat(i) + f2 * g2.flat(i) + (1.f - f1 - f2) * g3.flat(i); } return g;
}
TT G interp(CG g1, CG g2, CG g3, const Vec3<float>& bary) {
  // Vec3<float> == Bary;   may have bary[0] + bary[1] + bary[2] != 1.f
  ASSERTX(same_size(g1, g2) && same_size(g1, g3));
  G g(g1.dims()); F(g) { g.flat(i) = bary[0] * g1.flat(i) + bary[1] * g2.flat(i) + bary[2] * g3.flat(i); }
  return g;
}

// clang-format on
#undef PF
#undef F
#undef SS
#undef CG
#undef G
#undef TT

} // namespace hh

#endif // MESH_PROCESSING_LIBHH_GRID_H_
