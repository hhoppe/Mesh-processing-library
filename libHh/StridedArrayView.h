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
  [[nodiscard]] const T& operator[](int i) const { return HH_CHECK_BOUNDS(i, _n), _a[i * _stride]; }
  [[nodiscard]] const T& last() const { return (*this)[_n - 1]; }
  [[nodiscard]] bool ok(int i) const { return i >= 0 && i < _n; }
  using value_type = T;
  class iterator {
    using type = iterator;

   public:
    using iterator_concept = std::random_access_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    iterator() = default;
    bool operator==(const type& rhs) const { return _p == rhs._p; }
    const T& operator*() const { return *_p; }
    const T* operator->() const { return _p; }
    type& operator++() { return (_p += _stride), *this; }
    type& operator--() { return (_p -= _stride), *this; }
    type operator++(int) { return postfix_increment(*this); }
    type operator--(int) { return postfix_decrement(*this); }
    type& operator+=(std::ptrdiff_t i) { return (_p += i * _stride), *this; }
    type& operator-=(std::ptrdiff_t i) { return (_p -= i * _stride), *this; }
    type operator+(std::ptrdiff_t i) const { return type(_p + i * _stride, _stride); }
    type operator-(std::ptrdiff_t i) const { return type(_p - i * _stride, _stride); }
    friend type operator+(std::ptrdiff_t i, const type& rhs) { return rhs + i; }
    const T& operator[](std::ptrdiff_t i) const { return _p[i * _stride]; }
    std::ptrdiff_t operator-(const type& rhs) const {
      return ASSERTXX((_p - rhs._p) % _stride == 0), (_p - rhs._p) / _stride;
    }
    std::strong_ordering operator<=>(const type& rhs) const {
      return ASSERTXX((_p - rhs._p) % _stride == 0), _p <=> rhs._p;
    }

   private:
    const T* _p;
    ptrdiff_t _stride;
    iterator(const T* p, ptrdiff_t stride) : _p(p), _stride(stride) {}
    friend CStridedArrayView;
    friend StridedArrayView<T>;
  };
  using const_iterator = iterator;
  [[nodiscard]] iterator begin() const { return iterator(_a, _stride); }
  [[nodiscard]] iterator end() const { return iterator(_a + (_n * _stride), _stride); }

 protected:
  T* _a{nullptr};
  int _n{0};
  ptrdiff_t _stride;
  CStridedArrayView() = default;
  type& operator=(const type&) = delete;
};

// StridedArrayView is like an ArrayView except its elements are separate by a stride (i.e. not necessarily 1).
template <typename T> class StridedArrayView : public CStridedArrayView<T> {
  using base = CStridedArrayView<T>;
  using type = StridedArrayView<T>;

 public:
  explicit StridedArrayView(T* a, int n, ptrdiff_t stride) : base(a, n, stride) {}
  StridedArrayView(const type& a) = default;
  type& operator=(type&& a) & { return base::operator=(std::move(a)), *this; }  // For view<T>.
  [[nodiscard]] T& operator[](int i) { return HH_CHECK_BOUNDS(i, _n), _a[i * _stride]; }
  [[nodiscard]] const T& operator[](int i) const { return HH_CHECK_BOUNDS(i, _n), _a[i * _stride]; }
  [[nodiscard]] T& last() { return (*this)[_n - 1]; }
  [[nodiscard]] const T& last() const { return base::last(); }
  class iterator {
    using type = iterator;

   public:
    using iterator_concept = std::random_access_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    iterator() = default;
    bool operator==(const type& rhs) const { return _p == rhs._p; }
    T& operator*() const { return *_p; }
    T* operator->() const { return _p; }
    type& operator++() { return (_p += _stride), *this; }
    type& operator--() { return (_p -= _stride), *this; }
    type operator++(int) { return postfix_increment(*this); }
    type operator--(int) { return postfix_decrement(*this); }
    type& operator+=(std::ptrdiff_t i) { return (_p += i * _stride), *this; }
    type& operator-=(std::ptrdiff_t i) { return (_p -= i * _stride), *this; }
    type operator+(std::ptrdiff_t i) const { return type(_p + i * _stride, _stride); }
    type operator-(std::ptrdiff_t i) const { return type(_p - i * _stride, _stride); }
    friend type operator+(std::ptrdiff_t i, const type& rhs) { return rhs + i; }
    T& operator[](std::ptrdiff_t i) const { return _p[i * _stride]; }
    std::ptrdiff_t operator-(const type& rhs) const {
      return ASSERTXX((_p - rhs._p) % _stride == 0), (_p - rhs._p) / _stride;
    }
    std::strong_ordering operator<=>(const type& rhs) const {
      return ASSERTXX((_p - rhs._p) % _stride == 0), _p <=> rhs._p;
    }

   private:
    T* _p;
    ptrdiff_t _stride;
    iterator(T* p, ptrdiff_t stride) : _p(p), _stride(stride) {}
    friend StridedArrayView;
  };
  using const_iterator = typename base::iterator;
  [[nodiscard]] iterator begin() { return iterator(_a, _stride); }
  [[nodiscard]] const_iterator begin() const { return const_iterator(_a, _stride); }
  [[nodiscard]] iterator end() { return iterator(_a + (_n * _stride), _stride); }
  [[nodiscard]] const_iterator end() const { return const_iterator(_a + (_n * _stride), _stride); }

 protected:
  using base::_a;
  using base::_n;
  using base::_stride;
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
