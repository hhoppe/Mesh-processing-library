// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ARRAY_H_
#define MESH_PROCESSING_LIBHH_ARRAY_H_

#include "libHh/Range.h"

// Array is a dynamically resizable 1D array like std::vector, but it is derived from CArrayView and ArrayView
//  and it constructs/destructs elements based on capacity() rather than num().

#if 0
{
  Array<int> ar;
  for (auto element : container) {
    ar.init(element.size());
    process(element, ar);
  }
}
{
  Array<Point> ar;
  ar.push(Point(1.f, 2.f, 3.f));
  SHOW(ar.pop());
}
#endif

namespace hh {

// Define behavior beyond boundary of 1D range.
//  (reflected = Matlab:symmetric / Mathematica:reversed / DirectX:Mirror)
//  (reflected101 == OpenCV BORDER_REFLECT_101)
enum class Bndrule { reflected, periodic, clamped, border, reflected101, undefined };

// Convert a string ("reflected", "periodic", "clamped", "border", "101reflected") to a boundary rule by examining
// first letter.
Bndrule parse_boundaryrule(const string& s);

// Convert a boundary rule to a string.
string boundaryrule_name(Bndrule bndrule);

inline std::ostream& operator<<(std::ostream& os, Bndrule bndrule) {
  return os << "Bndrule{" << boundaryrule_name(bndrule) << "}";
}

// Modify index i to be within domain [0, n-1] using boundary rule; ret false if bndrule == Border and i is outside.
bool map_boundaryrule_1D(int& i, int n, Bndrule bndrule);

// View of a variable-sized 1D array with constant data of type T; e.g. refers to a const C-array,
//  std::array<T>, std::vector<T>, Vec<T>, Array<T>, PArray<T>, Matrix<T>[row], initializer_list<T>, etc.
template <typename T> class CArrayView {
  using type = CArrayView<T>;

 public:
  explicit CArrayView(const T* a, int n) : _a(const_cast<T*>(a)), _n(n) { ASSERTXX(n >= 0); }
  CArrayView(const type& a) = default;
  CArrayView(std::initializer_list<T> l) : CArrayView(l.begin(), narrow_cast<int>(l.size())) {}
  template <size_t n> CArrayView(const T (&a)[n]) : CArrayView(a, narrow_cast<int>(n)) {}  // for: const T a[n];
  template <size_t n> CArrayView(T (&a)[n]) : CArrayView(a, narrow_cast<int>(n)) {}        // for: T a[n];
  // template<int n> CArrayView(const Vec<T, n>&);  // implemented as conversion operator in Vec
  template <typename T2> friend bool same_size(type ar1, CArrayView<T2> ar2) { return ar1.num() == ar2.num(); }
  void reinit(type a) { *this = a; }
  int num() const { return _n; }
  size_t size() const { return narrow_cast<size_t>(_n); }
  const T& operator[](int i) const { return HH_CHECK_BOUNDS(i, _n), _a[i]; }
  const T& last() const { return (*this)[_n - 1]; }
  bool ok(int i) const { return i >= 0 && i < _n; }
  bool ok(const T* e) const { return ok(narrow_cast<int>(e - _a)); }
  bool map_inside(int& i, Bndrule bndrule) const;  // ret: false if bndrule == Border and i is outside
  const T& inside(int i, Bndrule bndrule) const { return (assertx(map_inside(i, bndrule)), (*this)[i]); }
  const T& inside(int i, Bndrule bndrule, const T* bordervalue) const;
  bool contains(const T& e) const;
  int index(const T& e) const;  // or -1 if not found
  bool operator==(type o) const;
  bool operator!=(type o) const { return !(*this == o); }
  type head(int n) const { return segment(0, n); }
  type tail(int n) const { return segment(_n - n, n); }
  type segment(int i, int s) const { return (ASSERTXX(check(i, s)), type(_a + i, s)); }
  type slice(int ib, int ie) const { return segment(ib, ie - ib); }
  using value_type = T;
  using iterator = const T*;
  using const_iterator = const T*;
  const T* begin() const { return _a; }
  const T* end() const { return _a + _n; }
  const T* data() const { return _a; }

 protected:
  bool check(int i, int s) const;
  T* _a{nullptr};
  int _n{0};
  CArrayView() {}
  type& operator=(const type&) = default;
};

// View of a variable-sized 1D array with modifiable data of type T, e.g. refers to a C-array,
//  std::array<T>, std::vector<T>, Vec<T>, Array<T>, PArray<T>, Matrix<T>[row], etc.
template <typename T> class ArrayView : public CArrayView<T> {
  using base = CArrayView<T>;
  using type = ArrayView<T>;

 public:
  explicit ArrayView(T* a, int n) : base(a, n) {}
  template <size_t n> ArrayView(T (&a)[n]) : base(a, n) {}  // for: T a[n];
  ArrayView(const type&) = default;                         // because it has explicit copy assignment
  // template<int n> ArrayView(Vec<T, n>&);  // implemented as conversion operator in Vec
  // ArrayView(std::vector<T>& a) : base(a) { }
  // template<size_t n> ArrayView(std::array<T, n>& a) : base(a) { }
  void reinit(type a) { *this = a; }
  T& operator[](int i) { return HH_CHECK_BOUNDS(i, _n), _a[i]; }
  const T& operator[](int i) const { return HH_CHECK_BOUNDS(i, _n), _a[i]; }
  T& last() { return (*this)[_n - 1]; }
  const T& last() const { return base::last(); }
  T& inside(int i, Bndrule bndrule) { return (assertx(map_inside(i, bndrule)), (*this)[i]); }
  const T& inside(int i, Bndrule bndrule) const { return (assertx(map_inside(i, bndrule)), (*this)[i]); }
  const T& inside(int i, Bndrule bndrule, const T* bordervalue) const;
  void assign(base ar);
  type head(int n) { return segment(0, n); }
  base head(int n) const { return base::head(n); }
  type tail(int n) { return segment(_n - n, n); }
  base tail(int n) const { return base::tail(n); }
  type segment(int i, int n) { return (ASSERTXX(check(i, n)), type(_a + i, n)); }
  base segment(int i, int n) const { return base::segment(i, n); }
  type slice(int ib, int ie) { return segment(ib, ie - ib); }
  base slice(int ib, int ie) const { return base::slice(ib, ie); }
  using iterator = T*;
  using const_iterator = const T*;
  T* begin() { return _a; }
  const T* begin() const { return _a; }
  T* end() { return _a + _n; }
  const T* end() const { return _a + _n; }
  T* data() { return _a; }
  const T* data() const { return _a; }
  using base::map_inside;
  using base::num;
  using base::ok;

 protected:
  using base::_a;
  using base::_n;
  using base::check;
  ArrayView() = default;
  type& operator=(const type&) = default;
};

// Construct a CArrayView<T> using a base pointer and number of elements, inferring type T automatically.
template <typename T> CArrayView<T> ArView(const T* a, int n) { return CArrayView<T>(a, n); }

// Construct an ArrayView<T> using a base pointer and number of elements, inferring type T automatically.
template <typename T> ArrayView<T> ArView(T* a, int n) { return ArrayView<T>(a, n); }

// Create a CArrayView<T> referencing the single specified element.
template <typename T> CArrayView<T> ArView(const T& e) { return CArrayView<T>(&e, 1); }

// Create an ArrayView<T> referencing the single specified element.
template <typename T> ArrayView<T> ArView(T& e) { return ArrayView<T>(&e, 1); }

// Construct a CArrayView<T> using a bounded C-array; for use on arguments to help template function deduction.
template <typename T, size_t n> CArrayView<T> ArView(const T (&a)[n]) { return CArrayView<T>(a, narrow_cast<int>(n)); }

// Construct an ArrayView<T> using a bounded C-array; for use on arguments to help template function deduction.
template <typename T, size_t n> ArrayView<T> ArView(T (&a)[n]) { return ArrayView<T>(a, narrow_cast<int>(n)); }

// Determine if two views have any overlap (to avoid aliasing issues).
template <typename T> bool have_overlap(CArrayView<T> v1, CArrayView<T> v2) {
  return v1.begin() < v2.end() && v2.begin() < v1.end();
}

// Heap-allocated resizable 1D array with elements of type T.
// Type T must have a public operator= (which may be operator=(&&)).
// Unlike std::vector<T>, Array<T> constructs/destructs elements based on capacity() rather than num().
template <typename T> class Array : public ArrayView<T> {
  using base = ArrayView<T>;
  using type = Array<T>;

 public:
  Array() = default;
  explicit Array(int n) : base(n ? new T[narrow_cast<size_t>(n)] : nullptr, n), _cap(n) { ASSERTX(n >= 0); }
  explicit Array(int n, const T& v) : Array(n) { for_int(i, n) _a[i] = v; }
  explicit Array(const type& ar) : Array(ar.num()) { base::assign(ar); }
  explicit Array(CArrayView<T> ar) : Array(ar.num()) { base::assign(ar); }
  Array(std::initializer_list<T> l) : Array(CArrayView<T>(l)) {}
  Array(type&& ar) noexcept : base(ar._a, ar._n), _cap(ar._cap) { ar._a = nullptr; }
  template <typename I> explicit Array(I b, I e) : Array() {
    for (; b != e; ++b) push(*b);
  }
  template <typename Range, typename = enable_if_range_t<Range>>
  explicit Array(Range&& range) : Array(range.begin(), range.end()) {}
  ~Array() { delete[] _a; }
  type& operator=(CArrayView<T> ar) {
    init(ar.num());
    base::assign(ar);
    return *this;
  }
  type& operator=(const type& ar) {
    init(ar.num());
    base::assign(ar);
    return *this;
  }
  type& operator=(std::initializer_list<T> l) { return *this = CArrayView<T>(l); }
  type& operator=(type&& ar) noexcept {
    clear();
    swap(*this, ar);
    return *this;
  }
  void clear() {
    if (_a) {
      delete[] _a;
      _a = nullptr, _n = 0, _cap = 0;
    }
  }
  void init(int n);  // allocate n, DISCARD old values if too small
  void init(int n, const T& v) {
    init(n);
    for_int(i, n) _a[i] = v;
  }
  void resize(int n) {  // allocate n, RETAIN old values (using move if too small)
    ASSERTX(n >= 0);
    if (n > _cap) grow_to_at_least(n);
    _n = n;
  }
  void access(int i);  // allocate at least i + 1, RETAIN old values (using move if too small)
  int add(int n) {     // ret: prev num
    ASSERTX(n >= 0);
    int t = _n;
    resize(_n + n);
    return t;
  }
  void sub(int n) {
    ASSERTX(n >= 0);
    _n -= n;
    ASSERTX(_n >= 0);
  }
  void shrink_to_fit() {
    if (_n < _cap) set_capacity(_n);
  }
  void reserve(int s) {
    ASSERTX(s >= 0);
    if (_cap < s) set_capacity(s);
  }
  int capacity() const { return _cap; }
  void insert(int i, int n) { ASSERTX(i >= 0 && i <= _n), insert_i(i, n); }
  void erase(int i, int n) { ASSERTX(i >= 0 && n >= 0 && i + n <= _n), erase_i(i, n); }
  void erase(T* b, T* e) { erase(narrow_cast<int>(b - base::begin()), narrow_cast<int>(e - b)); }
  bool remove_ordered(const T& e);    // ret: was there
  bool remove_unordered(const T& e);  // ret: was there
  T pop() {
    ASSERTX(_n);
    T e = std::move(_a[_n - 1]);
    sub(1);
    return e;
  }
  type pop(int n);
  void push(const T& e) {  // avoid a.push(a[..])!
    if (_n >= _cap) grow_to_at_least(_n + 1);
    _a[_n++] = e;
  }
  void push(T&& e) {
    if (_n >= _cap) grow_to_at_least(_n + 1);
    _a[_n++] = std::move(e);
  }
  void push_array(CArrayView<T> ar) {
    int n = ar.num();
    add(n);
    for_int(i, n) _a[_n - n + i] = ar[i];
  }
  void push_array(type&& ar) {
    int n = ar.num();
    add(n);
    for_int(i, n) _a[_n - n + i] = std::move(ar[i]);
  }
  void push_array(std::initializer_list<T> l) { push_array(CArrayView<T>(l)); }
  T shift() {
    ASSERTX(_n);
    T e = std::move(_a[0]);
    erase_i(0, 1);
    return e;
  }
  type shift(int n);
  void unshift(const T& e) { insert_i(0, 1), _a[0] = e; }
  void unshift(T&& e) { insert_i(0, 1), _a[0] = std::move(e); }
  void unshift(CArrayView<T> ar) {
    int n = ar.num();
    insert_i(0, n);
    for_int(i, n) _a[i] = ar[i];
  }
  void unshift(type&& ar) {
    int n = ar.num();
    insert_i(0, n);
    for_int(i, n) _a[i] = std::move(ar[i]);
  }
  friend void swap(Array& l, Array& r) noexcept {
    std::swap(l._a, r._a), std::swap(l._n, r._n), std::swap(l._cap, r._cap);
  }
  bool operator==(CArrayView<T> o) const { return _n == o.num() && static_cast<const CArrayView<T>&>(*this) == o; }
  bool operator==(const type& o) const { return _n == o.num() && static_cast<const CArrayView<T>&>(*this) == o; }
  // iterator is inherited from ArrayView
 private:
  using base::_a;
  using base::_n;
  int _cap{0};
  void set_capacity(int ncap);
  void grow_to_at_least(int n) { set_capacity(max(_n + (_n / 2) + 3, n)); }
  void insert_i(int i, int n) {
    add(n);
    for (int j = _n - n - 1; j >= i; --j) _a[j + n] = std::move(_a[j]);
  }
  void erase_i(int i, int n) {
    for_intL(j, i, _n - n) _a[j] = std::move(_a[j + n]);
    sub(n);
  }
  using base::reinit;                   // hide it
  type& operator+=(const T&) = delete;  // dangerous because ambiguous (push() or add to all elements)
};

// See also Vec.h, PArray.h, and Matrix.h

// Given container c, evaluate func() on each element (possibly changing the element type) and return new container.
template <typename T, typename Func> auto map(CArrayView<T>& c, Func func) {
  Array<decltype(func(std::declval<T>()))> nc(c.num());
  for_int(i, c.num()) nc[i] = func(c[i]);
  return nc;
}

//----------------------------------------------------------------------------

inline Bndrule parse_boundaryrule(const string& s) {
  Bndrule bndrule;
  // assertx(s.size() == 1);
  char ch = s[0];
  switch (ch) {
    case 'r': bndrule = Bndrule::reflected; break;
    case 'p': bndrule = Bndrule::periodic; break;
    case 'c': bndrule = Bndrule::clamped; break;
    case 'b': bndrule = Bndrule::border; break;
    case '1': bndrule = Bndrule::reflected101; break;
    default: assertnever(string() + "Boundary rule '" + s + "' not recognized");
  }
  if (s.size() > 1) assertx(s == boundaryrule_name(bndrule));
  return bndrule;
}

inline string boundaryrule_name(Bndrule bndrule) {
  switch (bndrule) {
    case Bndrule::reflected: return "reflected";
    case Bndrule::periodic: return "periodic";
    case Bndrule::clamped: return "clamped";
    case Bndrule::border: return "border";
    case Bndrule::reflected101: return "101reflected";
    case Bndrule::undefined: return "undefined";
    default: assertnever("");
  }
}

inline bool map_boundaryrule_1D(int& i, int n, Bndrule bndrule) {
  ASSERTX(n >= 1);
  switch (bndrule) {
    case Bndrule::reflected:
      for (;;) {
        if (i < 0)
          i = -i - 1;
        else if (i >= n)
          i = 2 * n - i - 1;
        else
          break;
      }
      break;
    case Bndrule::periodic:
      for (;;) {
        if (i < 0)
          i += n;
        else if (i >= n)
          i -= n;
        else
          break;
      }
      break;
    case Bndrule::clamped:
      if (i < 0)
        i = 0;
      else if (i >= n)
        i = n - 1;
      break;
    case Bndrule::border:
      if (i < 0 || i >= n) return false;
      break;
    case Bndrule::reflected101:
      if (n == 1) {
        i = 0;
        break;
      }
      for (;;) {
        if (i < 0)
          i = -i;
        else if (i >= n)
          i = 2 * n - i - 2;
        else
          break;
      }
      break;
    default: assertnever("");
  }
  return true;
}

//----------------------------------------------------------------------------

template <typename T> bool CArrayView<T>::map_inside(int& i, Bndrule bndrule) const {
  return map_boundaryrule_1D(i, _n, bndrule);
}

template <typename T> const T& CArrayView<T>::inside(int i, Bndrule bndrule, const T* bordervalue) const {
  if (!map_inside(i, bndrule)) {
    ASSERTX(bordervalue);
    return *bordervalue;
  }
  return (*this)[i];
}

template <typename T> bool CArrayView<T>::contains(const T& e) const {
  for_int(i, _n) {
    if (_a[i] == e) return true;
  }
  return false;
}

template <typename T> int CArrayView<T>::index(const T& e) const {
  for_int(i, _n) {
    if (_a[i] == e) return i;
  }
  return -1;
}

template <typename T> bool CArrayView<T>::operator==(type o) const {
  ASSERTX(_n == o._n);
  for_int(i, _n) {
    if (_a[i] != o._a[i]) return false;
  }
  return true;
}

template <typename T> bool CArrayView<T>::check(int i, int s) const {
  if (i >= 0 && s >= 0 && i + s <= _n) return true;
  SHOW(i, s, _n);
  return false;
}

//----------------------------------------------------------------------------

template <typename T> const T& ArrayView<T>::inside(int i, Bndrule bndrule, const T* bordervalue) const {
  if (!map_inside(i, bndrule)) {
    ASSERTX(bordervalue);
    return *bordervalue;
  }
  return (*this)[i];
}

template <typename T> void ArrayView<T>::assign(base ar) {
  ASSERTX(_n == ar.num());
  if (ar.data() == data()) return;
  // for_int(i, _n) _a[i] = ar[i];
  // std::memcpy() would be unsafe here for general T.
  // std::copy() and std::move() perform std::memmove() if std::is_trivially_copyable_v<T>
  //  see https://stackoverflow.com/questions/17625635/moving-an-object-in-memory-using-stdmemcpy
  // Some C++ standard libraries are unhappy if _a is nullptr even when ar.begin() == ar.end();
  //  see https://stackoverflow.com/questions/19480609/
  if (_a) std::copy(ar.begin(), ar.end(), _a);
}

//----------------------------------------------------------------------------

template <typename T> void Array<T>::init(int n) {
  ASSERTX(n >= 0);
  if (n > _cap) {
    delete[] _a;
    _a = new T[narrow_cast<size_t>(n)];
    _cap = n;
  }
  _n = n;
}

template <typename T> void Array<T>::access(int i) {
  ASSERTX(i >= 0);
  int n = i + 1;
  if (n > _cap) grow_to_at_least(n);
  if (n > _n) _n = n;
}

template <typename T> void Array<T>::set_capacity(int ncap) {
  _cap = ncap;
  ASSERTX(_n <= _cap);
  T* na = _cap ? new T[narrow_cast<size_t>(_cap)] : nullptr;
  if (na) std::move(_a, _a + _n, na);
  delete[] _a;
  _a = na;
}

template <typename T> bool Array<T>::remove_ordered(const T& e) {
  for_int(i, _n) {
    if (_a[i] == e) {
      erase(i, 1);
      return true;
    }
  }
  return false;
}

template <typename T> bool Array<T>::remove_unordered(const T& e) {
  for_int(i, _n) {
    if (_a[i] == e) {
      if (i < _n - 1) {
        _a[i] = std::move(_a[_n - 1]);
      }
      sub(1);
      return true;
    }
  }
  return false;
}

template <typename T> Array<T> Array<T>::shift(int n) {
  ASSERTX(n >= 0 && n <= _n);
  Array<T> ar(n);
  for_int(i, n) ar[i] = std::move(_a[i]);
  erase_i(0, n);
  return ar;
}

template <typename T> Array<T> Array<T>::pop(int n) {
  ASSERTX(n >= 0 && n <= _n);
  Array<T> ar(n);
  for_int(i, n) ar[i] = std::move(_a[_n - n + i]);
  sub(n);
  return ar;
}

//----------------------------------------------------------------------------

template <typename T> std::ostream& operator<<(std::ostream& os, CArrayView<T> a) {
  os << "Array<" << type_name<T>() << ">(" << a.num() << ") {\n";
  for_int(i, a.num()) {
    os << "  " << a[i] << (has_ostream_eol<T>() ? "" : "\n");  // skip linefeed if already printed
  }
  return os << "}\n";
}
template <typename T> HH_DECLARE_OSTREAM_EOL(CArrayView<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(ArrayView<T>);  // implemented by CArrayView<T>
template <typename T> HH_DECLARE_OSTREAM_EOL(Array<T>);      // implemented by CArrayView<T>

//----------------------------------------------------------------------------

// Set of functions common to Vec.h, SGrid.h, Array.h, Grid.h
// Note that RangeOp.h functions are valid here: mag2(), mag(), dist2(), dist(), dot(), is_zero(), compare().
#define TT template <typename T>
#define G Array<T>
#define CG CArrayView<T>
#define SS ASSERTX(same_size(g1, g2))
#define F(g) for_int(i, g.num())
// clang-format off

TT G operator+(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] + g2[i]; } return g; }
TT G operator-(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] - g2[i]; } return g; }
TT G operator*(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] * g2[i]; } return g; }
TT G operator/(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] / g2[i]; } return g; }
TT G operator%(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] % g2[i]; } return g; }

TT G operator+(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] + v; } return g; }
TT G operator-(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] - v; } return g; }
TT G operator*(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] * v; } return g; }
TT G operator/(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] / v; } return g; }
TT G operator%(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] % v; } return g; }

TT G operator+(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v + g1[i]; } return g; }
TT G operator-(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v - g1[i]; } return g; }
TT G operator*(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v * g1[i]; } return g; }
TT G operator/(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v / g1[i]; } return g; }
TT G operator%(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v % g1[i]; } return g; }

TT ArrayView<T> operator+=(ArrayView<T> g1, CG g2) { SS; F(g1) { g1[i] += g2[i]; } return g1; }
TT ArrayView<T> operator-=(ArrayView<T> g1, CG g2) { SS; F(g1) { g1[i] -= g2[i]; } return g1; }
TT ArrayView<T> operator*=(ArrayView<T> g1, CG g2) { SS; F(g1) { g1[i] *= g2[i]; } return g1; }
TT ArrayView<T> operator/=(ArrayView<T> g1, CG g2) { SS; F(g1) { g1[i] /= g2[i]; } return g1; }
TT ArrayView<T> operator%=(ArrayView<T> g1, CG g2) { SS; F(g1) { g1[i] %= g2[i]; } return g1; }

TT ArrayView<T> operator+=(ArrayView<T> g1, const T& v) { F(g1) { g1[i] += v; } return g1; }
TT ArrayView<T> operator-=(ArrayView<T> g1, const T& v) { F(g1) { g1[i] -= v; } return g1; }
TT ArrayView<T> operator*=(ArrayView<T> g1, const T& v) { F(g1) { g1[i] *= v; } return g1; }
TT ArrayView<T> operator/=(ArrayView<T> g1, const T& v) { F(g1) { g1[i] /= v; } return g1; }
TT ArrayView<T> operator%=(ArrayView<T> g1, const T& v) { F(g1) { g1[i] %= v; } return g1; }

TT G operator-(ArrayView<T> g1) { G g(g1.num()); F(g1) { g[i] = -g1[i]; } return g; }

TT G min(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = min(g1[i], g2[i]); } return g; }
TT G max(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = max(g1[i], g2[i]); } return g; }

TT G interp(CG g1, CG g2, float f1 = 0.5f) {
  SS; G g(g1.num()); F(g) { g[i] = f1 * g1[i] + (1.f - f1) * g2[i]; } return g;
}
TT G interp(CG g1, CG g2, CG g3, float f1 = 1.f / 3.f, float f2 = 1.f / 3.f) {
  ASSERTX(same_size(g1, g2) && same_size(g1, g3));
  SS; G g(g1.num()); F(g) { g[i] = f1 * g1[i] + f2 * g2[i] + (1.f - f1 - f2) * g3[i]; } return g;
}
// TT G interp(CG g1, CG g2, CG g3, const Vec3<float>& bary)  // avoid dependency on Vec

// clang-format on
#undef F
#undef SS
#undef CG
#undef G
#undef TT

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_ARRAY_H_
