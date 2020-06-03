// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_STRIDEDARRAYVIEW_H_
#define MESH_PROCESSING_LIBHH_STRIDEDARRAYVIEW_H_

#include <cstddef>  // ptrdiff_t

#include "Hh.h"

#if 0
{
  Array<int> ar(100);
  StridedArrayView ar10(ar.data(), 10);
  assertx(&ar10[3] == &ar[30]);
}
#endif

namespace hh {

template <typename T> class StridedArrayView;

// CStridedArrayView is like an CArrayView except its elements are separate by a stride (i.e. not necesssarily 1).
template <typename T> class CStridedArrayView {
  using type = CStridedArrayView<T>;

 public:
  explicit CStridedArrayView(const T* a, int n, size_t stride) : _a(const_cast<T*>(a)), _n(n), _stride(stride) {}
  CStridedArrayView(const type& a) = default;
  int num() const { return _n; }
  size_t size() const { return _n; }
  const T& operator[](int i) const { return (HH_CHECK_BOUNDS(i, _n), _a[i * _stride]); }
  const T& last() const { return (*this)[_n - 1]; }
  bool ok(int i) const { return i >= 0 && i < _n; }
  using value_type = T;
  class iterator {
   public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    iterator() = default;
    bool operator==(const iterator& it) const { return _p == it._p; }
    bool operator!=(const iterator& it) const { return _p != it._p; }
    const T& operator*() const { return *_p; }
    const T* operator->() const { return _p; }
    iterator& operator++() {
      _p += _stride;
      return *this;
    }
    iterator& operator--() {
      _p -= _stride;
      return *this;
    }
    iterator operator+(std::ptrdiff_t i) { return iterator(_p + i * _stride, _stride); }
    iterator operator-(std::ptrdiff_t i) { return iterator(_p - i * _stride, _stride); }
    const T& operator[](std::ptrdiff_t i) const { return _p[i * _stride]; }
    std::ptrdiff_t operator-(const iterator& it) const {
      return (ASSERTXX((_p - it._p) % _stride == 0), (_p - it._p) / _stride);
    }
    bool operator<(const iterator& it) const { return (ASSERTXX((_p - it._p) % _stride == 0), _p < it._p); }
    bool operator<=(const iterator& it) const { return (ASSERTXX((_p - it._p) % _stride == 0), _p <= it._p); }

   private:
    const T* _p;
    size_t _stride;
    iterator(const T* p, size_t stride) : _p(p), _stride(stride) {}
    friend CStridedArrayView;
    friend StridedArrayView<T>;
  };
  using const_iterator = iterator;
  iterator begin() const { return iterator(_a, _stride); }
  iterator end() const { return iterator(_a + (_n * _stride), _stride); }

 protected:
  T* _a{nullptr};
  int _n{0};
  size_t _stride;
  CStridedArrayView() = default;
  type& operator=(const type&) = delete;
};

// StridedArrayView is like an ArrayView except its elements are separate by a stride (i.e. not necesssarily 1).
template <typename T> class StridedArrayView : public CStridedArrayView<T> {
  using base = CStridedArrayView<T>;
  using type = StridedArrayView<T>;

 public:
  explicit StridedArrayView(T* a, int n, size_t stride) : base(a, n, stride) {}
  StridedArrayView(const type& a) = default;
  T& operator[](int i) { return (HH_CHECK_BOUNDS(i, _n), _a[i * _stride]); }
  const T& operator[](int i) const { return (HH_CHECK_BOUNDS(i, _n), _a[i * _stride]); }
  T& last() { return (*this)[_n - 1]; }
  const T& last() const { return base::last(); }
  class iterator {
   public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = value_type*;
    using reference = value_type&;
    iterator() = default;
    bool operator==(const iterator& it) const { return _p == it._p; }
    bool operator!=(const iterator& it) const { return _p != it._p; }
    T& operator*() const { return *_p; }
    T* operator->() const { return _p; }
    iterator& operator++() {
      _p += _stride;
      return *this;
    }
    iterator& operator--() {
      _p -= _stride;
      return *this;
    }
    iterator operator+(std::ptrdiff_t i) { return iterator(_p + i * _stride, _stride); }
    iterator operator-(std::ptrdiff_t i) { return iterator(_p - i * _stride, _stride); }
    T& operator[](std::ptrdiff_t i) const { return _p[i * _stride]; }
    std::ptrdiff_t operator-(const iterator& it) const {
      return (ASSERTXX((_p - it._p) % _stride == 0), (_p - it._p) / _stride);
    }
    bool operator<(const iterator& it) const { return (ASSERTXX((_p - it._p) % _stride == 0), _p < it._p); }
    bool operator<=(const iterator& it) const { return (ASSERTXX((_p - it._p) % _stride == 0), _p <= it._p); }

   private:
    T* _p;
    size_t _stride;
    iterator(T* p, size_t stride) : _p(p), _stride(stride) {}
    friend StridedArrayView;
  };
  using const_iterator = typename base::iterator;
  iterator begin() { return iterator(_a, _stride); }
  const_iterator begin() const { return const_iterator(_a, _stride); }
  iterator end() { return iterator(_a + (_n * _stride), _stride); }
  const_iterator end() const { return const_iterator(_a + (_n * _stride), _stride); }

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
    os << "  " << a[i] << (has_ostream_eol<T>() ? "" : "\n");  // skip linefeed if already printed
  }
  return os << "}\n";
}
template <typename T> HH_DECLARE_OSTREAM_EOL(CStridedArrayView<T>);
template <typename T> HH_DECLARE_OSTREAM_EOL(StridedArrayView<T>);  // implemented by CStridedArrayView<T>

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_STRIDEDARRAYVIEW_H_
