// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_ARRAY_H_
#define MESH_PROCESSING_LIBHH_ARRAY_H_

#include "libHh/Range.h"

// Array is a dynamically resizable 1D array like std::vector, but it is derived from CArrayView and ArrayView
// and it constructs/destructs elements based on capacity() rather than num().
//
// Aliasing: no argument may refer into the array itself, because any operation that changes the size may reallocate
// the storage and thereby invalidate the argument; e.g. avoid a.push(a[0]), a.push_array(a.head(2)), a = a.tail(3).

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
// its first letter.
[[nodiscard]] constexpr Bndrule parse_boundaryrule(std::string_view s);

// Convert a boundary rule to a string.
[[nodiscard]] constexpr std::string_view boundaryrule_name(Bndrule bndrule);

inline std::ostream& operator<<(std::ostream& os, Bndrule bndrule) {
  return os << "Bndrule{" << boundaryrule_name(bndrule) << "}";
}

// Modify index i to be within domain [0, n-1] using boundary rule; return false if bndrule == Border and i is outside.
[[nodiscard]] constexpr bool map_boundaryrule_1D(int& i, int n, Bndrule bndrule);

template <typename T> class CArrayView;
template <typename T> class ArrayView;

// The view type (const or mutable) corresponding to an element pointer type Ptr.
template <typename Ptr>
using array_view_t = std::conditional_t<std::is_const_v<std::remove_pointer_t<Ptr>>,
                                        CArrayView<std::remove_cv_t<std::remove_pointer_t<Ptr>>>,
                                        ArrayView<std::remove_pointer_t<Ptr>>>;

// View of a variable-sized 1D array with constant data of type T; e.g. refers to a const C-array,
//  std::array<T>, std::vector<T>, Vec<T>, Array<T>, PArray<T>, Matrix<T>[row], etc.
template <typename T> class CArrayView {
  using type = CArrayView<T>;

 public:
  explicit CArrayView(const T* a, int n) : _a(const_cast<T*>(a)), _n(n) { ASSERTXX(n >= 0); }
  CArrayView(const type& a) = default;
  template <size_t n> CArrayView(const T (&a)[n]) : CArrayView(a, narrow_cast<int>(n)) {}  // For: const T a[n];
  template <size_t n> CArrayView(T (&a)[n]) : CArrayView(a, narrow_cast<int>(n)) {}        // For: T a[n];
  // template <int n> CArrayView(const Vec<T, n>&);  // Implemented as conversion operator in Vec.
  template <typename T2> [[nodiscard]] friend constexpr bool same_size(type ar1, CArrayView<T2> ar2) {
    return ar1.num() == ar2.num();
  }
  void reinit(type a) { *this = a; }
  [[nodiscard]] constexpr int num() const { return _n; }
  [[nodiscard]] constexpr size_t size() const { return narrow_cast<size_t>(_n); }
  [[nodiscard]] [[HH_NO_DANGLING]] constexpr auto& operator[](this auto&& self, int i) {
    HH_CHECK_BOUNDS(i, self.num());
    return self.data()[i];
  }
  [[nodiscard]] constexpr auto& last(this auto&& self) { return self[self.num() - 1]; }
  [[nodiscard]] constexpr bool ok(int i) const { return i >= 0 && i < _n; }
  [[nodiscard]] constexpr bool ok(const T* e) const { return ok(narrow_cast<int>(e - _a)); }
  [[nodiscard]] bool map_inside(int& i, Bndrule bndrule) const;  // Return false if bndrule == Border and i is outside.
  [[nodiscard]] auto& inside(this auto&& self, int i, Bndrule bndrule) {
    return (assertx(self.map_inside(i, bndrule)), self[i]);
  }
  [[nodiscard]] const T& inside(int i, Bndrule bndrule, const T* bordervalue) const;
  [[nodiscard]] constexpr bool operator==(type rhs) const;
  [[nodiscard]] constexpr auto head(this auto&& self, int n) { return self.segment(0, n); }
  [[nodiscard]] constexpr auto tail(this auto&& self, int n) { return self.segment(self.num() - n, n); }
  [[nodiscard]] constexpr auto segment(this auto&& self, int i, int s) {
    ASSERTXX(implicit_cast<const type&>(self).check(i, s));  // Access the protected check() through this base class.
    return array_view_t<decltype(self.data())>(self.data() + i, s);
  }
  [[nodiscard]] constexpr auto slice(this auto&& self, int ib, int ie) { return self.segment(ib, ie - ib); }
  using value_type = T;
  using iterator = const T*;
  using const_iterator = const T*;
  constexpr auto begin(this auto&& self) { return self.data(); }
  constexpr auto end(this auto&& self) { return self.data() + self.num(); }
  constexpr const T* data() const { return _a; }

 protected:
  // The pointer is declared non-const even though CArrayView's elements are logically const.  This lets the derived
  // ArrayView<T> and Array<T> expose a mutable data() without a second pointer member, so all three classes share a
  // single layout and derived-to-base conversion is free.  Const-correctness is therefore not enforced by the type of
  // _a but by the member function signatures: CArrayView never returns _a as non-const, and there is deliberately no
  // conversion from CArrayView<T> to ArrayView<T>.  Do not add one, and do not expose _a in a public member.
  T* _a{nullptr};
  int _n{0};
  [[nodiscard]] constexpr bool check(int i, int s) const {
    if (i >= 0 && s >= 0 && i + s <= _n) return true;
    if !consteval {
      SHOW(i, s, _n);
    }
    return false;
  }
  CArrayView() = default;
  type& operator=(const type&) = default;
};

// View of a variable-sized 1D array with modifiable data of type T, e.g. refers to a C-array,
//  std::array<T>, std::vector<T>, Vec<T>, Array<T>, PArray<T>, Matrix<T>[row], etc.
template <typename T> class ArrayView : public CArrayView<T> {
  using base = CArrayView<T>;
  using type = ArrayView<T>;

 public:
  explicit ArrayView(T* a, int n) : base(a, n) {}
  template <size_t n> ArrayView(T (&a)[n]) : base(a, n) {}  // For: T a[n];
  ArrayView(const type&) = default;                         // Because it has explicit copy assignment.
  // template <int n> ArrayView(Vec<T, n>&);  // Implemented as conversion operator in Vec.
  // ArrayView(std::vector<T>& a) : base(a) { }
  // template <size_t n> ArrayView(std::array<T, n>& a) : base(a) { }
  void reinit(type a) { *this = a; }
  void assign(base ar) requires(Copyable<T>);
  using iterator = T*;
  using const_iterator = const T*;
  T* data() { return _a; }
  const T* data() const { return _a; }
  using base::num;
  using base::ok;
  using base::size;

 protected:
  using base::_a;
  using base::_n;
  using base::check;
  ArrayView() = default;
  type& operator=(const type&) = default;
};

// Create an ArrayView<T> referencing the single specified element.
template <typename T> [[nodiscard]] constexpr ArrayView<T> ArView(T& e) { return ArrayView<T>(&e, 1); }

// Determine if two views have any overlap (to avoid aliasing issues).
template <typename T> [[nodiscard]] constexpr bool have_overlap(CArrayView<T> v1, CArrayView<T> v2) {
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
  explicit Array(int n, const T& v) requires Copyable<T> : Array(n) { for_int(i, n) _a[i] = v; }
  explicit Array(const type& ar) requires Copyable<T> : Array(ar.num()) { base::assign(ar); }
  explicit Array(CArrayView<T> ar) requires Copyable<T> : Array(ar.num()) { base::assign(ar); }
  Array(std::initializer_list<T> l) requires Copyable<T>
      : Array(CArrayView<T>(l.begin(), narrow_cast<int>(l.size()))) {}
  Array(type&& ar) noexcept : base(ar._a, ar._n), _cap(ar._cap) { ar._a = nullptr, ar._n = 0, ar._cap = 0; }
  template <typename Iterator, typename = std::enable_if_t<
                                   !std::is_same_v<typename std::iterator_traits<Iterator>::iterator_category, void>>>
  explicit Array(Iterator b, Iterator e) : Array() {
    // Note that if an Iterator is a native pointer (T*), it is automatically recognized as a random-access iterator.
    if constexpr (random_access_iterator_v<Iterator>) reserve(narrow_cast<int>(e - b));
    for (; b != e; ++b) push(*b);
  }
  template <typename Range, typename = enable_if_range_t<Range>> explicit Array(Range&& range) {
    if constexpr (range_has_size_v<Range>)
      reserve(narrow_cast<int>(range.size()));
    else if constexpr (random_access_range_v<Range>)
      reserve(narrow_cast<int>(range.end() - range.begin()));
    for (const auto& e : range) push(e);
  }
  ~Array() { delete[] _a; }
  type& operator=(CArrayView<T> ar) requires Copyable<T> {
    init(ar.num());
    base::assign(ar);
    return *this;
  }
  type& operator=(const type& ar) requires Copyable<T> {
    init(ar.num());
    base::assign(ar);
    return *this;
  }
  type& operator=(type&& ar) noexcept {
    clear();
    swap(*this, ar);
    return *this;
  }
  void clear() {
    delete[] _a;
    _a = nullptr, _n = 0, _cap = 0;
  }
  void init(int n);  // Allocate n, DISCARD old values if too small.
  void init(int n, const T& v) requires Copyable<T> {
    init(n);
    for_int(i, n) _a[i] = v;
  }
  void resize(int n) {  // Allocate n, RETAIN old values (using move if too small).
    ASSERTX(n >= 0);
    if (n > _cap) grow_to_at_least(n);
    _n = n;
  }
  void access(int i);  // Allocate at least i + 1, RETAIN old values (using move if too small).
  int add(int n) {     // Return: previous num().
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
  [[nodiscard]] int capacity() const { return _cap; }
  void insert(int i, int n) { ASSERTX(i >= 0 && i <= _n), insert_i(i, n); }
  void erase(int i, int n) { ASSERTX(i >= 0 && n >= 0 && i + n <= _n), erase_i(i, n); }
  void erase(T* b, T* e) { erase(narrow_cast<int>(b - base::begin()), narrow_cast<int>(e - b)); }
  bool remove_ordered(const T& e);    // Return: was there.
  bool remove_unordered(const T& e);  // Return: was there.
  T pop() {
    ASSERTXX(_n);
    T e = std::move(_a[_n - 1]);
    sub(1);
    return e;
  }
  type pop(int n);
  void push(const T& e) requires Copyable<T> {  // Avoid a.push(a[..])!
    if (_n >= _cap) grow_to_at_least(_n + 1);
    _a[_n++] = e;
  }
  void push(T&& e) {
    if (_n >= _cap) grow_to_at_least(_n + 1);
    _a[_n++] = std::move(e);
  }
  template <std::ranges::input_range R> requires std::assignable_from<T&, std::ranges::range_reference_t<R>>
  void push_array(R&& range) {
    if constexpr (std::ranges::forward_range<R> || std::ranges::sized_range<R>) {
      const int t = add(narrow_cast<int>(std::ranges::distance(range)));
      std::ranges::copy(range, _a + t);  // Moves elements only if the caller opts in via std::views::as_rvalue.
    } else {
      for (auto&& e : range) push(std::forward<decltype(e)>(e));
    }
  }
  void push_array(type&& ar) {
    int n = ar.num();
    add(n);
    for_int(i, n) _a[_n - n + i] = std::move(ar[i]);
  }
  T shift() {
    ASSERTX(_n);
    T e = std::move(_a[0]);
    erase_i(0, 1);
    return e;
  }
  type shift(int n);
  void unshift(const T& e) requires Copyable<T> { insert_i(0, 1), _a[0] = e; }
  friend void swap(Array& l, Array& r) noexcept {
    std::swap(l._a, r._a), std::swap(l._n, r._n), std::swap(l._cap, r._cap);
  }
  // Note that Array iterator is inherited from ArrayView.
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
  using base::reinit;                   // Hide it.
  type& operator+=(const T&) = delete;  // Dangerous because meaning is ambiguous (either push() or add to elements).
};

// See also Vec.h, PArray.h, and Matrix.h.

// Given container c, evaluate func() on each element (possibly changing the element type) and return new container.
template <typename T, typename Func>
[[nodiscard]] auto map(CArrayView<T> c, Func func) -> Array<decltype(func(std::declval<T>()))> {
  Array<decltype(func(std::declval<T>()))> nc(c.num());
  for_int(i, c.num()) nc[i] = func(c[i]);
  return nc;
}

//----------------------------------------------------------------------------

[[nodiscard]] constexpr Bndrule parse_boundaryrule(const std::string_view s) {
  Bndrule bndrule;
  assertx(s.size() >= 1);
  char ch = s[0];
  switch (ch) {
    case 'r': bndrule = Bndrule::reflected; break;
    case 'p': bndrule = Bndrule::periodic; break;
    case 'c': bndrule = Bndrule::clamped; break;
    case 'b': bndrule = Bndrule::border; break;
    case '1': bndrule = Bndrule::reflected101; break;
    default: assertnever("Boundary rule '" + string(s) + "' not recognized");
  }
  if (s.size() > 1 && s != boundaryrule_name(bndrule)) assertnever("Boundary rule '" + string(s) + "' not recognized");
  return bndrule;
}

[[nodiscard]] constexpr std::string_view boundaryrule_name(Bndrule bndrule) {
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

constexpr bool map_boundaryrule_1D(int& i, int n, Bndrule bndrule) {
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

template <typename T> [[nodiscard]] bool CArrayView<T>::map_inside(int& i, Bndrule bndrule) const {
  return map_boundaryrule_1D(i, _n, bndrule);
}

template <typename T>
[[nodiscard]] const T& CArrayView<T>::inside(int i, Bndrule bndrule, const T* bordervalue) const {
  if (!map_inside(i, bndrule)) {
    ASSERTX(bordervalue);
    return *bordervalue;
  }
  return (*this)[i];
}

template <typename T> [[nodiscard]] constexpr bool CArrayView<T>::operator==(type rhs) const {
  if (_n != rhs._n) return false;
  for_int(i, _n) {
    if (_a[i] != rhs._a[i]) return false;
  }
  return true;
}

//----------------------------------------------------------------------------

template <typename T> void ArrayView<T>::assign(base ar) requires(Copyable<T>) {
  ASSERTX(_n == ar.num());
  if (ar.data() == data()) return;
#if 0
  for_int(i, ar.num()) _a[i] = ar[i];
#else
  // std::memcpy() would be unsafe for general T; std::copy() uses std::memmove() when T is trivially copyable.
  std::copy(ar.begin(), ar.end(), _a);
#endif
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
  ASSERTXX(i >= 0);
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
      if (i < _n - 1) _a[i] = std::move(_a[_n - 1]);
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
    os << "  " << a[i] << (has_ostream_eol_v<T> ? "" : "\n");  // Skip linefeed if already printed.
  }
  return os << "}\n";
}
template <typename T> HH_DECLARE_OSTREAM_EOL(CArrayView<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(ArrayView<T>);  // Implemented by CArrayView<T>.
template <typename T> HH_DECLARE_OSTREAM_EOL(Array<T>);      // Implemented by CArrayView<T>.

// Template deduction guides:
template <typename T> CArrayView(const T* a, int) -> CArrayView<T>;
template <typename T, size_t n> CArrayView(const T (&)[n]) -> CArrayView<T>;
template <typename T, size_t n> CArrayView(T (&)[n]) -> CArrayView<T>;
template <typename T> ArrayView(T* a, int) -> ArrayView<T>;
template <typename T, size_t n> ArrayView(T (&)[n]) -> ArrayView<T>;
template <std::input_iterator Iterator> Array(Iterator, Iterator) -> Array<std::iter_value_t<Iterator>>;
template <typename Range, typename = enable_if_range_t<Range>> Array(Range&&) -> Array<range_value_t<Range>>;
// template <std::ranges::input_range Range> Array(Range&&) -> Array<std::ranges::range_value_t<Range>>;

//----------------------------------------------------------------------------

// Set of functions common to Vec.h, SGrid.h, Array.h, Grid.h.
// Note that RangeOp.h functions are valid here: mag2(), mag(), dist2(), dist(), dot(), is_zero(), compare().
#define TT template <typename T>
#define TTN TT [[nodiscard]]
#define G Array<T>
#define CG CArrayView<T>
#define SS ASSERTXX(same_size(g1, g2))
#define F(g) for_int(i, g.num())
// clang-format off

TTN G operator+(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] + g2[i]; } return g; }
TTN G operator-(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] - g2[i]; } return g; }
TTN G operator*(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] * g2[i]; } return g; }
TTN G operator/(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] / g2[i]; } return g; }
TTN G operator%(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = g1[i] % g2[i]; } return g; }

TTN G operator+(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] + v; } return g; }
TTN G operator-(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] - v; } return g; }
TTN G operator*(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] * v; } return g; }
TTN G operator/(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] / v; } return g; }
TTN G operator%(CG g1, const T& v) { G g(g1.num()); F(g) { g[i] = g1[i] % v; } return g; }

TTN G operator+(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v + g1[i]; } return g; }
TTN G operator-(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v - g1[i]; } return g; }
TTN G operator*(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v * g1[i]; } return g; }
TTN G operator/(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v / g1[i]; } return g; }
TTN G operator%(const T& v, CG g1) { G g(g1.num()); F(g) { g[i] = v % g1[i]; } return g; }

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

TTN G operator-(CG g1) { G g(g1.num()); F(g1) { g[i] = -g1[i]; } return g; }

TTN G min(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = min(g1[i], g2[i]); } return g; }
TTN G max(CG g1, CG g2) { SS; G g(g1.num()); F(g) { g[i] = max(g1[i], g2[i]); } return g; }

TTN G interp(CG g1, CG g2, float f1 = 0.5f) {
  SS; G g(g1.num()); F(g) { g[i] = f1 * g1[i] + (1.f - f1) * g2[i]; } return g;
}
TTN G interp(CG g1, CG g2, CG g3, float f1, float f2) {
  ASSERTXX(same_size(g1, g2) && same_size(g1, g3));
  G g(g1.num()); F(g) { g[i] = f1 * g1[i] + f2 * g2[i] + (1.f - f1 - f2) * g3[i]; } return g;
}
TTN G interp(CG g1, CG g2, CG g3) { return interp(g1, g2, g3, 1.f / 3.f, 1.f / 3.f); }
// TTN G interp(CG g1, CG g2, CG g3, const Vec3<float>& bary)  // Omit to avoid dependency on Vec.

// clang-format on
#undef F
#undef SS
#undef CG
#undef G
#undef TTN
#undef TT

}  // namespace hh

template <typename T> inline constexpr bool std::ranges::enable_borrowed_range<hh::CArrayView<T>> = true;
template <typename T> inline constexpr bool std::ranges::enable_borrowed_range<hh::ArrayView<T>> = true;

#endif  // MESH_PROCESSING_LIBHH_ARRAY_H_
