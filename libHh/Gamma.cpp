// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Gamma.h"

namespace hh {

namespace {

// The float overloads compute in double precision so that encode(decode(x)) reproduces x to float precision.
#define HH_GAMMA_FLOAT_OVERLOADS                                                                      \
  [[nodiscard]] float decode(float encoded) const override { return float(decode(double(encoded))); } \
  [[nodiscard]] float encode(float linear) const override { return float(encode(double(linear))); }   \
  HH_EAT_SEMICOLON

struct GammaIdentity final : Gamma {
  GammaIdentity() : Gamma("identity") { _is_identity = true; }
  [[nodiscard]] float decode(float encoded) const override { return encoded; }
  [[nodiscard]] float encode(float linear) const override { return linear; }
  [[nodiscard]] double decode(double encoded) const override { return encoded; }
  [[nodiscard]] double encode(double linear) const override { return linear; }
};

struct GammaPower2 final : Gamma {
  GammaPower2() : Gamma("power2") {}
  [[nodiscard]] double decode(double encoded) const override { return encoded * encoded; }
  [[nodiscard]] double encode(double linear) const override { return std::sqrt(std::max(linear, 0.)); }
  HH_GAMMA_FLOAT_OVERLOADS;
};

struct GammaPower22 final : Gamma {
  GammaPower22() : Gamma("power22") {}
  [[nodiscard]] double decode(double encoded) const override { return std::pow(encoded, 2.2); }
  [[nodiscard]] double encode(double linear) const override { return std::pow(std::max(linear, 0.), 1. / 2.2); }
  HH_GAMMA_FLOAT_OVERLOADS;
};

struct GammaSrgb final : Gamma {
  GammaSrgb() : Gamma("srgb") {}
  [[nodiscard]] double decode(double encoded) const override {
    return encoded > 0.04045 ? std::pow((encoded + 0.055) / 1.055, 2.4) : encoded / 12.92;
  }
  [[nodiscard]] double encode(double linear) const override {
    linear = std::max(linear, 0.);
    return linear > 0.0031308 ? std::pow(linear, 1. / 2.4) * 1.055 - 0.055 : linear * 12.92;
  }
  HH_GAMMA_FLOAT_OVERLOADS;
};

#undef HH_GAMMA_FLOAT_OVERLOADS

}  // namespace

const Gamma& Gamma::get(std::string_view name) {
  assertx(name != "");
  // As in Filter::get(), the singletons are lazily created and intentionally never destroyed.
  static const auto& gammas = *new Array<const Gamma*>{
      new GammaIdentity,
      new GammaPower2,
      new GammaPower22,
      new GammaSrgb,
  };
  for (const Gamma* gamma : gammas)
    if (gamma->name() == name) return *gamma;
  assertnever("Gamma '" + string(name) + "' not recognized");
}

CArrayView<float> Gamma::decode_table_uint8() const {
  std::call_once(_table_uint8_once, [&] {
    _table_uint8.init(256);
    for_int(i, 256) _table_uint8[i] = decode(i / 255.f);
  });
  return _table_uint8;
}

}  // namespace hh
