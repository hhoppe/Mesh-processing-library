// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_WINDOWS_COM_H_
#define MESH_PROCESSING_LIBHH_WINDOWS_COM_H_

#if defined(_WIN32)

#include <combaseapi.h>

#include "libHh/Hh.h"

namespace hh {

template <typename T> class com_ptr;

// Helper class for com_ptr<T>, to allow "com_ptr<Interface> interface; SomeCreate(parameters, &interface)".
template <typename T> class com_ptr_ref {
 public:
  com_ptr_ref(com_ptr<T>& cp) : _cp(cp) { assertx(!_cp); }
  ~com_ptr_ref() { assertx(!_cp), _cp.reset(_p); }
  com_ptr_ref(const com_ptr_ref<T>& cp) = default;
  operator T**() {
    assertx(!_num++);
    assertx(!_p);
    return &_p;
  }

 private:
  com_ptr<T>& _cp;
  T* _p{nullptr};
  int _num{0};
};

// Wrapper around COM pointer to provide exception-safe Release().
template <typename T> class com_ptr : public unique_ptr<T, void (*)(T*)> {
  using base = unique_ptr<T, void (*)(T*)>;
  static void SafeRelease(T* p) {
    if (p) p->Release();
  }

 public:
  com_ptr() : base(nullptr, SafeRelease) {}
  com_ptr(com_ptr<T>&& p) : base(std::move(p)) {}
  com_ptr<T>& operator=(com_ptr<T>&& p) {
    base::operator=(std::move(p));
    return *this;
  }
  operator T*() { return base::get(); }
  com_ptr_ref<T> operator&() { return com_ptr_ref<T>(*this); }
};

// Helper for IID_PPV_ARGS in "com_ptr<IMFByteStream> pbs; AS(pSource->QueryInterface(IID_PPV_ARGS(&pbs)));".
template <typename T> void** IID_PPV_ARGS_Helper(com_ptr_ref<T>&& p) {
  static_assert(std::is_base_of_v<IUnknown, T>, "T has to derive from IUnknown");
  return reinterpret_cast<void**>(static_cast<T**>(p));
}

}  // namespace hh

#endif  // defined(_WIN32)

#endif  // MESH_PROCESSING_LIBHH_WINDOWS_COM_H_
