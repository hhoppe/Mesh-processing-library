// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_GAMMA_H_
#define MESH_PROCESSING_LIBHH_GAMMA_H_

#include <mutex>  // once_flag

#include "libHh/Array.h"

namespace hh {

// A component transfer function between the stored (encoded) sample values and the linear values that filtering
// operates on, following the Python package `resampler`.  Values are in the range [0, 1]; integer sample types are
// mapped onto that range by the resampling functions.  Instances are immortal named singletons obtained through
// Gamma::get(); user-defined transfer functions derive from this class.
class Gamma : noncopyable {
 public:
  // identity:  linear = encoded.
  // power2:    linear = encoded ^ 2 (a fast approximation of sRGB).
  // power22:   linear = encoded ^ 2.2.
  // srgb:      the sRGB transfer function.
  [[HH_NO_DANGLING]] [[nodiscard]] static const Gamma& get(std::string_view name);
  [[nodiscard]] const string& name() const { return _name; }
  [[nodiscard]] bool is_identity() const { return _is_identity; }

  // Convert an encoded value to a linear value.
  [[nodiscard]] virtual float decode(float encoded) const = 0;
  [[nodiscard]] virtual double decode(double encoded) const = 0;

  // Convert a linear value to an encoded value (a negative value is treated as zero).
  [[nodiscard]] virtual float encode(float linear) const = 0;
  [[nodiscard]] virtual double encode(double linear) const = 0;

  // The decoded values of the 256 uint8_t sample values, and of the 65536 uint16_t ones (lazily computed).
  [[nodiscard]] CArrayView<float> decode_table_uint8() const;
  [[nodiscard]] CArrayView<float> decode_table_uint16() const;

  friend std::ostream& operator<<(std::ostream& os, const Gamma& gamma) {
    return os << "Gamma{" << gamma.name() << "}";
  }

 protected:
  explicit Gamma(string name) : _name(std::move(name)) {}
  virtual ~Gamma() = default;
  bool _is_identity{false};

 private:
  string _name;
  mutable std::once_flag _table_uint8_once, _table_uint16_once;
  mutable Array<float> _table_uint8, _table_uint16;
};

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_GAMMA_H_
