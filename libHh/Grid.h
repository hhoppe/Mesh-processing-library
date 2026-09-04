// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_GRID_H_
#define MESH_PROCESSING_LIBHH_GRID_H_

#include "libHh/Parallel.h"
#include "libHh/RangeOp.h"
#include "libHh/Vec.h"

#if 0
{
  Grid<2, int> grid(V(20, 10), -1);        // Dimensions (ny = 20, nx = 10), and optional initial value.
  int y, x;                                // Individual coordinates.
  Vec2<int> u = V(y, x);                   // Array of coordinates (in Big Endian order).
  size_t i = ravel_index(grid.dims(), u);  // Index into "vectorized" flat view of grid.
  assertx(&grid[y][x] == &grid[u]);
  assertx(&grid[y, x] == &grid[u]);
  assertx(&grid.flat(i) == &grid[u]);
  assertx(i == ravel_index_list(grid.dims(), y, x));
  assertx(unravel_index(grid.dims(), i) == u);

  grid[18][5] = 1;
  grid[18, 6] = 2;
  grid[{18, 7}] = 3;
  grid.flat(ravel_index(grid.dims(), V(18, 8))) = 4;
  for_int(y, grid.dim(0)) for_int(x, grid.dim(1)) grid[y, x] *= 2;  // Iterate using individual coordinates.
  for (const auto& u : range(grid.dims())) grid[u] *= 2;            // Iterate using array of coordinate indices.
  for (const size_t i : range(grid.size())) grid.flat(i) *= 2;      // Iterate using raster order (fastest).
  for (auto& e : grid) e *= 2;                                      // Iterate over elements (also fastest).
}
#endif

namespace hh {

template <int D, typename T> class CGridView;
template <int D, typename T> class GridView;

namespace details {

template <int D, typename T> struct nested_initializer_list {
  using type = std::initializer_list<typename nested_initializer_list<D - 1, T>::type>;
};
template <typename T> struct nested_initializer_list<0, T> {
  using type = T;
};
template <int D, typename T> using nested_initializer_list_t = typename nested_initializer_list<D, T>::type;

template <int D, typename T> inline Vec<int, D> nested_list_dims(nested_initializer_list_t<D, T> l);
template <int D, typename T> constexpr void nested_list_retrieve(auto&& grid, nested_initializer_list_t<D, T> l);

}  // namespace details

// Given a coordinate u within a grid with dimensions dims, return the flat index in the linearized representation.
template <int D> constexpr size_t ravel_index(const Vec<int, D>& dims, const Vec<int, D>& u);

// Given a flat index i within a grid with dimensions dims, return its grid coordinates.
template <int D> constexpr Vec<int, D> unravel_index(const Vec<int, D>& dims, size_t i);

// Do the same for a list of coordinates.
template <int D>
constexpr size_t ravel_index_list(const Vec<int, D>& dims, std::integral auto d0, std::integral auto... dd);

// Find stride (in number of elements) of dimension d in the grid.
template <int D> constexpr size_t grid_stride(const Vec<int, D>& dims, int dim);

// Compute the size_t product of a small fixed set of numbers.
template <int D> constexpr size_t product_dims(const int* ar) {
  static_assert(D >= 0);
  if constexpr (D == 0) {
    return 1;
  } else {
    size_t v = ar[0];
    if constexpr (D > 1) for_intL(i, 1, D) v *= ar[i];
    return v;
  }
}

// View of a contiguous D-dimensional grid with constant data of type T; often refers to a "const Grid<D, T>".
// Any or all of the dimensions may be zero.
template <int D, typename T> class CGridView {
  using type = CGridView<D, T>;
  static_assert(D >= 1, "dimension problem");

 public:
  explicit CGridView(const T* a, const Vec<int, D>& dims) : _a(const_cast<T*>(a)), _dims(dims) {}
  CGridView(const type&) = default;
  explicit CGridView(CArrayView<T> ar) requires(D == 1) : CGridView(ar.data(), V(ar.num())) {}
  // Reseat the view.  Defined to enable movable<T> for view<T>.  The rvalue source and lvalue-only keep
  // `gridview = grid` and `grid[0] = grid[1]` ill-formed.  Use assign() to copy elements, reinit() to reseat.
  type& operator=(type&& g) & { return _a = g._a, _dims = g._dims, *this; }
  void reinit(type g) { *this = g; }
  template <typename T2> [[nodiscard]] friend bool same_size(type g1, CGridView<D, T2> g2) {
    return g1.dims() == g2.dims();
  }
  [[nodiscard]] static constexpr int ndim() { return D; }
  [[nodiscard]] const Vec<int, D>& dims() const { return _dims; }
  [[nodiscard]] int dim(int c) const { return _dims[c]; }
  [[nodiscard]] size_t size() const { return product_dims<D>(_dims.data()); }
  template <std::integral... A> [[nodiscard]] constexpr decltype(auto) operator[](this auto&& self, A... dd);
  template <int n> [[nodiscard]] constexpr decltype(auto) operator[](this auto&& self, const Vec<int, n>& u);
  [[nodiscard]] auto& flat(this auto&& self, size_t i) { return ASSERTXX(i < self.size()), self.data()[i]; }
  [[nodiscard]] bool ok(const Vec<int, D>& u) const {
    for_int(c, D) {
      if (u[c] < 0 || u[c] >= _dims[c]) return false;
    }
    return true;
  }
  [[nodiscard]] bool ok(std::integral auto... dd) const { return ok(V(dd...)); }
  bool map_inside(Vec<int, D>& u, const Vec<Bndrule, D>& bndrules) const {  // Return false outside Border.
    for_int(c, D) {
      if (!map_boundaryrule_1D(u[c], _dims[c], bndrules[c])) return false;
    }
    return true;
  }
  [[nodiscard]] auto& inside(this auto&& self, const Vec<int, D>& u, const Vec<Bndrule, D>& bndrules) {
    Vec<int, D> ut(u);
    assertx(self.map_inside(ut, bndrules));
    return self[ut];
  }
  [[nodiscard]] const T& inside(const Vec<int, D>& u, const Vec<Bndrule, D>& bndrules, const T* bordervalue) const {
    Vec<int, D> ut(u);
    if (!map_inside(ut, bndrules)) return ASSERTX(bordervalue), *bordervalue;
    return (*this)[ut];
  }
  [[nodiscard]] auto slice(this auto&& self, int ib, int ie) {  // View of grid truncated in 0th dimension.
    const Vec<int, D>& dims = self.dims();
    assertx(ib >= 0 && ib <= ie && ie <= dims[0]);
    return grid_view_t<D, decltype(self.data())>(self.data() + ib * grid_stride(dims, 0), dims.with(0, ie - ib));
  }
  [[nodiscard]] auto slices(this auto&& self) requires(D >= 2) {
    // We capture the pointer and dims rather than a view because a captured view would be const in the lambda,
    // and subscripting a const GridView yields a CGridView.
    using View = grid_view_t<D, decltype(self.data())>;
    return range(self.dim(0)) |
           views::transform([a = self.data(), dims = self.dims()](int i) { return View(a, dims)[i]; });
  }
  using value_type = T;
  using iterator = const T*;
  using const_iterator = const T*;
  [[nodiscard]] auto begin(this auto&& self) { return self.data(); }
  [[nodiscard]] auto end(this auto&& self) { return self.data() + self.size(); }
  [[nodiscard]] const T* data() const { return _a; }
  [[nodiscard]] auto array_view(this auto&& self) {
    return array_view_t<decltype(self.data())>(self.data(), narrow_cast<int>(self.size()));
  }
  // For implementation of Matrix (D == 2):
  [[nodiscard]] int ysize() const requires(D == 2) { return dim(0); }
  [[nodiscard]] int xsize() const requires(D == 2) { return dim(1); }
  // Return false if bndrule == Border and i is outside.
  bool map_inside(int& y, int& x, Bndrule bndrule) const requires(D == 2);
  [[nodiscard]] auto& inside(this auto&& self, int y, int x, Bndrule bndrule) requires(D == 2) {
    bool b = self.map_inside(y, x, bndrule);
    ASSERTX(b);
    return self[y, x];
  }
  [[nodiscard]] const T& inside(int y, int x, Bndrule bndrule, const T* bordervalue) const requires(D == 2);

 protected:
  // See class CArrayView for a discussion of constness and protection of operator=().
  T* _a{nullptr};  // [0, _dims[0] - 1] * [0, _dims[1] - 1] * ... * [0, _dims[D - 1] - 1].
  Vec<int, D> _dims{ntimes<D>(0)};
  [[nodiscard]] bool check(int r) const {
    if (r >= 0 && r < _dims[0]) return true;
    if !consteval {
      SHOW(r, _dims);
    }
    return false;
  }
  [[nodiscard]] bool check(const Vec<int, D>& u) const {
    if (ok(u)) return true;
    if !consteval {
      SHOW(u, _dims);
    }
    return false;
  }
  CGridView() = default;
  type& operator=(const type&) = default;
};

// View of a contiguous D-dimensional grid with modifiable data of type T; often refers to a Grid<D, T>.
template <int D, typename T> class [[HH_NO_DANGLING]] GridView : public CGridView<D, T> {
  using base = CGridView<D, T>;
  using type = GridView<D, T>;

 public:
  explicit GridView(T* a, const Vec<int, D>& dims) : base(a, dims) {}  // Stop recursion if ArrayView == GridView.
  GridView(const type&) = default;                                     // Because it has explicit copy assignment.
  explicit GridView(ArrayView<T> ar) requires(D == 1) : GridView(ar.data(), V(ar.num())) {}
  // Reseat the view.  Defined to enable movable<T> for view<T>.  The rvalue source and lvalue-only keep
  // `gridview = grid` and `grid[0] = grid[1]` ill-formed.  Use assign() to copy elements, reinit() to reseat.
  type& operator=(type&& g) & { return base::operator=(std::move(g)), *this; }
  void reinit(type g) { *this = g; }
  void assign(CGridView<D, T> g) requires Copyable<T>;
  using value_type = T;
  using iterator = T*;
  using const_iterator = const T*;
  [[nodiscard]] T* data() { return _a; }
  [[nodiscard]] const T* data() const { return _a; }
  void reverse_y() requires(D == 2) {
    const int ny = this->ysize();
    parallel_for({.cycles_per_elem = uint64_t(this->xsize()) * 2}, range(ny / 2), [&](const int y) {  //
      swap_elements((*this)[y], (*this)[ny - 1 - y]);
    });
  }
  void reverse_x() requires(D == 2) {
    const int ny = this->ysize();
    parallel_for({.cycles_per_elem = uint64_t(this->xsize()) * 2}, range(ny), [&](const int y) {  //
      reverse((*this)[y]);
    });
  }

 protected:
  using base::_a;
  using base::_dims;
  using base::check;
  GridView() = default;
  type& operator=(const type&) = default;
};

// Create a CGridView<1, T> referencing the single specified element.
template <typename T> [[nodiscard]] CGridView<1, T> CGrid1View(const T& e) { return CGridView<1, T>(&e, V(1)); }
template <typename T> CGridView<1, T> CGrid1View(const T&&) = delete;

// Create an GridView<1, T> referencing the single specified element.
template <typename T> [[nodiscard]] GridView<1, T> Grid1View(T& e) { return GridView<1, T>(&e, V(1)); }

// Heap-allocated D-dimensional contiguous grid.  Any or all of the dimensions may be zero.
// Grid(10, 20) or Grid(V(10, 20)) both create a 10x20 grid, whereas Grid({10, 20}) creates a 1x2 grid.
template <int D, typename T> class Grid : public GridView<D, T> {
  using base = GridView<D, T>;
  using type = Grid<D, T>;

 public:
  Grid() = default;
  explicit Grid(const Vec<int, D>& dims) { init(dims); }
  template <typename... A> requires(std::convertible_to<A, int> && ...) explicit Grid(int d0, A... dr) {
    init(d0, dr...);
  }
  explicit Grid(const Vec<int, D>& dims, const T& v) requires Copyable<T> { init(dims, v); }
  explicit Grid(const type& g) requires Copyable<T> : Grid(g.dims()) { base::assign(g); }
  explicit Grid(CGridView<D, T> g) requires Copyable<T> : Grid(g.dims()) { base::assign(g); }
  Grid(type&& g) noexcept { swap(*this, g); }  // Not "== default".
  Grid(details::nested_initializer_list_t<D, T> l) requires Copyable<T> : Grid() { *this = l; }
  ~Grid() { clear(); }
  type& operator=(CGridView<D, T> g) requires Copyable<T> {
    init(g.dims());
    base::assign(g);
    return *this;
  }
  type& operator=(const type& g) requires Copyable<T> {
    init(g.dims());
    base::assign(g);
    return *this;
  }
  type& operator=(details::nested_initializer_list_t<D, T> l) requires Copyable<T> {
    init(details::nested_list_dims<D, T>(l));
    details::nested_list_retrieve<D, T>(base(*this), l);
    return *this;
  }
  type& operator=(type&& g) noexcept { return clear(), swap(*this, g), *this; }
  void init(int d0, std::integral auto... dr) { init(Vec<int, D>(d0, dr...)); }
  using base::size;
  void init(const Vec<int, D>& dims) {
    if (dims == _dims) return;
    assertx(min(dims) >= 0);
    size_t vol = product_dims<D>(dims.data());
    if (vol != size()) {
      delete[] _a;
      _a = vol ? new T[vol] : nullptr;
    }
    _dims = dims;
  }
  void init(const Vec<int, D>& dims, const T& v) requires Copyable<T> {
    init(dims);
    fill(*this, v);
  }
  void clear() {
    if (_a) init(ntimes<D>(0));
  }
  friend void swap(Grid& l, Grid& r) noexcept {
    ranges::swap(l._a, r._a);
    ranges::swap(l._dims, r._dims);
  }
  void special_reduce_dim0(int i) { assertx(i >= 0 && i <= _dims[0]), _dims[0] = i; }
  // (Must declare template parameters because these functions access private _a of <D - 1, T> and <D + 1, T>.)
  template <int DD, typename TT> friend Grid<DD - 1, TT> reduce_grid_rank(Grid<DD, TT>&& grid);
  template <int DD, typename TT> friend Grid<DD + 1, TT> increase_grid_rank(Grid<DD, TT>&& grid);
  using owns_elements = void;  // Marker for hh::clone().

 private:
  using base::_a;
  using base::_dims;
  using base::reinit;  // Hide it.
};

// Construct a Grid with the given dims from a flat range of values, in raster (Grid::flat()) order.
// E.g.: const auto grid = grid_from_flat(image.dims(), image | views::transform(to_luminance));
template <int D, ranges::input_range R> requires(ranges::forward_range<R> || ranges::sized_range<R>)
[[nodiscard]] auto grid_from_flat(const Vec<int, D>& dims, R&& range) -> Grid<D, range_value_t<R>> {
  Grid<D, range_value_t<R>> grid(dims);
  ASSERTXX(size_t(ranges::distance(range)) == grid.size());
  ranges::copy(range, grid.begin());  // Moves elements only if the caller opts in via views::as_rvalue.
  return grid;
}

// Given container c, evaluate func() on each element (possibly changing the element type) and return new container.
template <int D, typename T, typename Func> [[nodiscard]] auto transformed(CGridView<D, T> c, Func func) {
  return grid_from_flat(c.dims(), c | views::transform(func));
}

//----------------------------------------------------------------------------

template <int D> [[nodiscard]] constexpr size_t ravel_index(const Vec<int, D>& dims, const Vec<int, D>& u) {
  if constexpr (D == 0) {
    return 0;
  } else {
    HH_CHECK_BOUNDS(u[0], dims[0]);
    size_t i = u[0];
    for_intL(d, 1, D) {
      HH_CHECK_BOUNDS(u[d], dims[d]);
      i = i * dims[d] + u[d];
    }
    return i;
  }
}

template <int D> [[nodiscard]] constexpr Vec<int, D> unravel_index(const Vec<int, D>& dims, size_t i) {
  static_assert(D >= 1);
  ASSERTXX(i < product_dims<D>(dims.data()));
  Vec<int, D> u;
  for (int d = D - 1; d >= 1; --d) {
    // Using std::div() is unadvised as it does not support unsigned and would introduce a function call on gcc.
    u[d] = narrow_cast<int>(i % dims[d]);
    i /= dims[d];
  }
  u[0] = narrow_cast<int>(i);
  return u;
}

// The loop is written out explicitly rather than delegating to ravel_index() on a Vec<int, D> temporary; the
// temporary defeats the strength reduction that flattens a nested subscript loop into a contiguous pointer walk.
template <int D>
[[nodiscard]] constexpr size_t ravel_index_list(const Vec<int, D>& dims, std::integral auto d0,
                                                std::integral auto... dd) {
  static_assert(1 + sizeof...(dd) == D);
  HH_CHECK_BOUNDS(narrow_cast<int>(d0), dims[0]);
  size_t i = d0;
  int d = 0;
  dummy_use(d);
  ((++d, HH_CHECK_BOUNDS(narrow_cast<int>(dd), dims[d]), i = i * dims[d] + dd), ...);
  return i;
}

template <int D> [[nodiscard]] constexpr size_t grid_stride(const Vec<int, D>& dims, int dim) {
  ASSERTXX(dim >= 0 && dim < D);
  if (dim + 1 >= D) return 1;
  size_t i = dims[dim + 1];
  for_intL(d, dim + 2, D) i *= dims[d];
  return i;
}

//----------------------------------------------------------------------------

namespace details {

// Result of a subscript leaving rank R over elements of type T, which may be const-qualified.
// (The CGridView<0, T> named in the discarded branch for R < 2 is never instantiated.)
template <int R, typename T>
using grid_ret_t = std::conditional_t<R == 0, T&, std::conditional_t<R == 1, array_view_t<T*>, grid_view_t<R, T*>>>;

// Construct the element reference or sub-view of rank D - n at pointer p within a grid with dimensions dims.
template <int D, int n, typename T> grid_ret_t<D - n, T> grid_view_at(T* p, const Vec<int, D>& dims) {
  using Ret = grid_ret_t<D - n, T>;
  if constexpr (n == D)
    return static_cast<Ret>(*p);
  else if constexpr (n == D - 1)
    return Ret(p, dims[n]);
  else
    return Ret(p, dims.template segment<n, D - n>());
}

}  // namespace details

//----------------------------------------------------------------------------

template <int D, typename T>
template <std::integral... A>
constexpr decltype(auto) CGridView<D, T>::operator[](this auto&& self, A... dd) {
  constexpr int n = sizeof...(dd);
  static_assert(n <= D);
  const Vec<int, D>& dims = self.dims();
  auto* p = self.data();
  if constexpr (n > 0) p += ravel_index_list(dims.template head<n>(), dd...) * product_dims<D - n>(dims.data() + n);
  return details::grid_view_at<D, n>(p, dims);
}

template <int D, typename T>
template <int n>
constexpr decltype(auto) CGridView<D, T>::operator[](this auto&& self, const Vec<int, n>& u) {
  static_assert(n >= 0 && n <= D);
  const Vec<int, D>& dims = self.dims();
  auto* p = self.data() + ravel_index(dims.template head<n>(), u) * product_dims<D - n>(dims.data() + n);
  return details::grid_view_at<D, n>(p, dims);
}

template <int D, typename T> bool CGridView<D, T>::map_inside(int& y, int& x, Bndrule bndrule) const requires(D == 2) {
  return map_boundaryrule_1D(y, ysize(), bndrule) && map_boundaryrule_1D(x, xsize(), bndrule);
}

template <int D, typename T>
const T& CGridView<D, T>::inside(int y, int x, Bndrule bndrule, const T* bordervalue) const requires(D == 2) {
  if (!map_inside(y, x, bndrule)) {
    ASSERTX(bordervalue);
    return *bordervalue;
  }
  return (*this)[y, x];
}

template <int D, typename T> void GridView<D, T>::assign(CGridView<D, T> g) requires Copyable<T> {
  assertx(same_size(*this, g));
  if (g.data() == data()) return;
  ranges::copy(g, data());
}

template <int D, typename T> [[nodiscard]] Grid<D - 1, T> reduce_grid_rank(Grid<D, T>&& grid) {
  assertx(grid.dim(0) == 1);  // Perhaps could be <= 1.
  Grid<D - 1, T> ngrid;
  ranges::swap(ngrid._a, grid._a);
  ngrid._dims = grid._dims.template tail<D - 1>();
  grid._dims = ntimes<D>(0);
  return ngrid;
}

template <int D, typename T> [[nodiscard]] Grid<D + 1, T> increase_grid_rank(Grid<D, T>&& grid) {
  Grid<D + 1, T> ngrid;
  ranges::swap(ngrid._a, grid._a);
  ngrid._dims = concat(V(1), grid._dims);
  grid._dims = ntimes<D>(0);
  return ngrid;
}

//----------------------------------------------------------------------------

namespace details {

template <int D, typename T> inline Vec<int, D> nested_list_dims(nested_initializer_list_t<D, T> l) {
  const int n = narrow_cast<int>(l.size());
  if constexpr (D == 1) {
    return V(n);
  } else {
    assertx(n > 0);
    return concat(V(n), nested_list_dims<D - 1, T>(l.begin()[0]));
  }
}

template <int D, typename T> constexpr void nested_list_retrieve(auto&& grid, nested_initializer_list_t<D, T> l) {
  const int n = narrow_cast<int>(l.size());
  if constexpr (D == 1) {
    assertx(grid.size() == l.size());
    for_int(i, n) grid[i] = l.begin()[i];
  } else {
    if constexpr (requires { grid.dim(0); })
      assertx(grid.dim(0) == n);  // Here grid is a GridView<D, T>.
    else
      assertx(grid.num() == n);  // Here grid is a Vec<slice, d0>, whose elements are nested Vec.
    for_int(i, n) nested_list_retrieve<D - 1, T>(grid[i], l.begin()[i]);
  }
}

}  // namespace details

//----------------------------------------------------------------------------

template <int D, typename T> std::ostream& operator<<(std::ostream& os, CGridView<D, T> g) {
  if constexpr (D == 1) {
    os << "Array<" << type_name<T>() << ">(" << g.dim(0) << ") {\n";
    for_int(i, g.dim(0)) os << "  " << g[i] << (has_ostream_eol_v<T> ? "" : "\n");
    return os << "}\n";
  } else if constexpr (D == 2) {
    const int ny = g.dim(0), nx = g.dim(1);
    os << "Matrix<" << type_name<T>() << ">(" << ny << ", " << nx << ") {\n";
    for_int(y, ny) {
      if (has_ostream_eol_v<T>) {
        for_int(x, nx) os << sform("  [%d, %d] = ", y, x) << g[y, x];
      } else {
        os << " ";
        for_int(x, nx) os << " " << g[y, x];
        os << "\n";
      }
    }
    return os << "}\n";
  } else {
    os << "Grid<" << type_name<T>() << ">(";
    for_int(i, g.dims().num()) os << (i ? ", " : "") << g.dims()[i];
    os << ") {\n";
    for (const auto& p : range(g.dims())) os << "  " << p << " = " << g[p] << (has_ostream_eol_v<T> ? "" : "\n");
    return os << "}\n";
  }
}
template <int D, typename T> HH_DECLARE_OSTREAM_EOL(CGridView<D, T>);
template <int D, typename T> HH_DECLARE_OSTREAM_EOL(GridView<D, T>);  // Implemented by CGridView<D, T>.
template <int D, typename T> HH_DECLARE_OSTREAM_EOL(Grid<D, T>);      // Implemented by CGridView<D, T>.

// Template deduction guides:
template <int D, typename T> Grid(CGridView<D, T>) -> Grid<D, T>;
// For most constructions, we cannot infer both the rank and the element type.
// One practical solution is to construct using `grid_from_flat(dims, range)`.

//----------------------------------------------------------------------------

// Set of functions common to Array.h, Grid.h, Vec.h.
// Note that RangeOp.h functions are valid here: mag2(), mag(), dist2(), dist(), dot(), is_zero(), compare().
#define TT template <int D, typename T>
#define TTN TT [[nodiscard]]
#define G Grid<D, T>
#define CG CGridView<D, T>
#define GV GridView<D, T>
#define SS ASSERTXX(same_size(g1, g2))
#define F(g) for (const size_t i : range(g.size()))
#define PF(g, code) parallel_for({.cycles_per_elem = 1}, range(g.size()), [&](const size_t i) { code; })
// clang-format off

// Function bodies for the elementwise operations below.  NEW_* returns a new grid; MOD_* modifies g1 in place.
// Within code, a is the destination data pointer, and b and c are the data pointers of g1 and g2.
// The pointers are hoisted out of the loop because going through flat() prevents the compiler from vectorizing.
#define NEW_GG(code) \
  { SS; G g(g1.dims()); T* a = g.data(); const T* b = g1.data(); const T* c = g2.data(); PF(g, code); return g; }
#define NEW_G(code) { G g(g1.dims()); T* a = g.data(); const T* b = g1.data(); PF(g, code); return g; }
#define MOD_GG(code) { SS; T* a = g1.data(); const T* b = g2.data(); PF(g1, code); return g1; }
#define MOD_G(code) { T* a = g1.data(); PF(g1, code); return g1; }

#define HH_OPERATIONS(OP)                                        \
  TTN G operator OP(CG g1, CG g2) NEW_GG(a[i] = b[i] OP c[i])    \
  TTN G operator OP(CG g1, const T& e) NEW_G(a[i] = b[i] OP e)   \
  TTN G operator OP(const T& e, CG g1) NEW_G(a[i] = e OP b[i])   \
  TT GV operator OP##=(GV g1, CG g2) MOD_GG(a[i] OP##= b[i])     \
  TT GV operator OP##=(GV g1, const T& e) MOD_G(a[i] OP##= e)    \
  HH_EAT_SEMICOLON

HH_OPERATIONS(+); HH_OPERATIONS(-); HH_OPERATIONS(*); HH_OPERATIONS(/); HH_OPERATIONS(%);

TTN G operator-(CG g1) NEW_G(a[i] = -b[i])

TTN G min(CG g1, CG g2) { SS; G g(g1.dims()); F(g) { g.flat(i) = min(g1.flat(i), g2.flat(i)); } return g; }
TTN G max(CG g1, CG g2) { SS; G g(g1.dims()); F(g) { g.flat(i) = max(g1.flat(i), g2.flat(i)); } return g; }

TTN G interp(CG g1, CG g2, float f1 = 0.5f) {
  SS; G g(g1.dims()); F(g) { g.flat(i) = static_cast<T>(f1 * g1.flat(i) + (1.f - f1) * g2.flat(i)); } return g;
}
TTN G interp(CG g1, CG g2, CG g3, float f1, float f2) {
  ASSERTXX(same_size(g1, g2) && same_size(g1, g3));
  G g(g1.dims());
  F(g) { g.flat(i) = static_cast<T>(f1 * g1.flat(i) + f2 * g2.flat(i) + (1.f - f1 - f2) * g3.flat(i)); } return g;
}
TTN G interp(CG g1, CG g2, CG g3) { return interp(g1, g2, g3, 1.f / 3.f, 1.f / 3.f); }
TTN G interp(CG g1, CG g2, CG g3, const Vec3<float>& bary) {
  // Vec3<float> == Bary;   May have bary[0] + bary[1] + bary[2] != 1.f.
  ASSERTXX(same_size(g1, g2) && same_size(g1, g3));
  G g(g1.dims());
  F(g) { g.flat(i) = static_cast<T>(bary[0] * g1.flat(i) + bary[1] * g2.flat(i) + bary[2] * g3.flat(i)); }
  return g;
}

// clang-format on
#undef PF
#undef F
#undef SS
#undef HH_OPERATIONS
#undef MOD_G
#undef MOD_GG
#undef NEW_G
#undef NEW_GG
#undef GV
#undef CG
#undef G
#undef TTN
#undef TT

}  // namespace hh

template <int D, typename T> inline constexpr bool std::ranges::enable_view<hh::CGridView<D, T>> = true;
template <int D, typename T> inline constexpr bool std::ranges::enable_borrowed_range<hh::CGridView<D, T>> = true;
template <int D, typename T> inline constexpr bool std::ranges::enable_view<hh::GridView<D, T>> = true;
template <int D, typename T> inline constexpr bool std::ranges::enable_borrowed_range<hh::GridView<D, T>> = true;

#endif  // MESH_PROCESSING_LIBHH_GRID_H_
