// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_STRIDEDARRAYVIEW_H_
#define MESH_PROCESSING_LIBHH_STRIDEDARRAYVIEW_H_

#include <cstddef>  // ptrdiff_t

#include "libHh/Hh.h"

#if 0
{
  Array<int> ar(100);
  StridedArrayView ar10(ar.data(), 10);
  assertx(&ar10[3] == &ar[30]);
}
#endif

namespace hh {

template <typename T> class StridedArrayView;

// CStridedArrayView is like an CArrayView except its elements are separate by a stride (i.e. not necessarily 1).
template <typename T> class CStridedArrayView {
  using type = CStridedArrayView<T>;

 public:
  explicit CStridedArrayView(const T* a, int n, ptrdiff_t stride) : _a(const_cast<T*>(a)), _n(n), _stride(stride) {
    ASSERTX(_stride > 0);  // Else <=>() uses "_stride > 0 ? _p <=> rhs._p : rhs._p <=> _p".
  }
  CStridedArrayView(const type& a) = default;
  type& operator=(type&& a) & { return _a = a._a, _n = a._n, _stride = a._stride, *this; }  // For view<T>.
  [[nodiscard]] int num() const { return _n; }
  [[nodiscard]] size_t size() const { return _n; }
  [[nodiscard]] ptrdiff_t stride() const { return _stride; }
  [[nodiscard]] auto& operator[](this auto&& self, int i) {
    HH_CHECK_BOUNDS(i, self.num());
    return self.data()[i * self.stride()];
  }
  [[nodiscard]] auto& last(this auto&& self) { return self[self.num() - 1]; }
  [[nodiscard]] bool ok(int i) const { return i >= 0 && i < _n; }
  using value_type = T;

  // Iterator over elements of type U, which is const T in CStridedArrayView and T in StridedArrayView.
  template <typename U> class Iterator {
    using type = Iterator;

   public:
    using iterator_concept = std::random_access_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    Iterator() = default;
    template <typename U2> requires std::is_same_v<U, const U2>  // Conversion from iterator to const_iterator.
    Iterator(const Iterator<U2>& rhs) : _p(rhs._p), _stride(rhs._stride) {}
    bool operator==(const type& rhs) const { return _p == rhs._p; }
    U& operator*() const { return *_p; }
    U* operator->() const { return _p; }
    type& operator++() { return (_p += _stride), *this; }
    type& operator--() { return (_p -= _stride), *this; }
    type operator++(int) { return postfix_increment(*this); }
    type operator--(int) { return postfix_decrement(*this); }
    type& operator+=(std::ptrdiff_t i) { return (_p += i * _stride), *this; }
    type& operator-=(std::ptrdiff_t i) { return (_p -= i * _stride), *this; }
    type operator+(std::ptrdiff_t i) const { return type(_p + i * _stride, _stride); }
    type operator-(std::ptrdiff_t i) const { return type(_p - i * _stride, _stride); }
    friend type operator+(std::ptrdiff_t i, const type& rhs) { return rhs + i; }
    U& operator[](std::ptrdiff_t i) const { return _p[i * _stride]; }
    std::ptrdiff_t operator-(const type& rhs) const {
      return ASSERTXX((_p - rhs._p) % _stride == 0), (_p - rhs._p) / _stride;
    }
    std::strong_ordering operator<=>(const type& rhs) const {
      return ASSERTXX((_p - rhs._p) % _stride == 0), _p <=> rhs._p;
    }

   private:
    U* _p;
    ptrdiff_t _stride;
    Iterator(U* p, ptrdiff_t stride) : _p(p), _stride(stride) {}
    friend CStridedArrayView;
  };

  using iterator = Iterator<const T>;
  using const_iterator = iterator;
  [[nodiscard]] auto begin(this auto&& self) { return self.iterator_at(0); }
  [[nodiscard]] auto end(this auto&& self) { return self.iterator_at(self.num()); }
  [[nodiscard]] const T* data() const { return _a; }

 protected:
  // The pointer is declared non-const even though CStridedArrayView's elements are logically const.  This lets the
  // derived StridedArrayView<T> expose a mutable data() without a second pointer member, so both classes share a
  // single layout and derived-to-base conversion is free.  Const-correctness is therefore not enforced by the type of
  // _a but by the member function signatures: CStridedArrayView never returns _a as non-const, and there is
  // deliberately no conversion from CStridedArrayView<T> to StridedArrayView<T>.
  T* _a{nullptr};
  int _n{0};
  ptrdiff_t _stride;
  CStridedArrayView() = default;
  type& operator=(const type&) = delete;

 private:
  // The iterator at element index i.  Its element type follows the constness of self.data(), so this single
  // definition serves begin() and end() of both CStridedArrayView (const) and StridedArrayView (const and mutable).
  [[nodiscard]] auto iterator_at(this auto&& self, int i) {
    return Iterator<std::remove_pointer_t<decltype(self.data())>>(self.data() + i * self.stride(), self.stride());
  }
};

// StridedArrayView is like an ArrayView except its elements are separate by a stride (i.e. not necessarily 1).
template <typename T> class StridedArrayView : public CStridedArrayView<T> {
  using base = CStridedArrayView<T>;
  using type = StridedArrayView<T>;

 public:
  explicit StridedArrayView(T* a, int n, ptrdiff_t stride) : base(a, n, stride) {}
  StridedArrayView(const type& a) = default;
  type& operator=(type&& a) & { return base::operator=(std::move(a)), *this; }  // For view<T>.
  using value_type = T;
  using iterator = typename base::template Iterator<T>;
  using const_iterator = typename base::const_iterator;
  [[nodiscard]] T* data() { return _a; }
  [[nodiscard]] const T* data() const { return _a; }

 protected:
  using base::_a;
  StridedArrayView() = default;
  type& operator=(const type&) = delete;
};

template <typename T> std::ostream& operator<<(std::ostream& os, CStridedArrayView<T> a) {
  os << "StridedArray<" << type_name<T>() << ">(" << a.num() << ") {\n";
  for_int(i, a.num()) {
    os << "  " << a[i] << (has_ostream_eol_v<T> ? "" : "\n");  // Skip linefeed if already printed.
  }
  return os << "}\n";
}
template <typename T> HH_DECLARE_OSTREAM_EOL(CStridedArrayView<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(StridedArrayView<T>);  // Implemented by CStridedArrayView<T>.

}  // namespace hh

template <typename T> inline constexpr bool std::ranges::enable_view<hh::CStridedArrayView<T>> = true;
template <typename T> inline constexpr bool std::ranges::enable_borrowed_range<hh::CStridedArrayView<T>> = true;

template <typename T> inline constexpr bool std::ranges::enable_view<hh::StridedArrayView<T>> = true;
template <typename T> inline constexpr bool std::ranges::enable_borrowed_range<hh::StridedArrayView<T>> = true;

#endif  // MESH_PROCESSING_LIBHH_STRIDEDARRAYVIEW_H_
