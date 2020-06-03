// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_G3DOGL_NORMALMAPPING_H_
#define MESH_PROCESSING_G3DOGL_NORMALMAPPING_H_

#include "Geometry.h"
#include "Pixel.h"

namespace hh {

class NormalMapping {
 public:
  virtual ~NormalMapping() {}
  virtual string name() const = 0;
  virtual bool is_supported() const = 0;
  virtual void init() = 0;
  virtual void set_parameters(const Vector& lightdirmodel, const Vector& eyedirmodel, float lambient,
                              float lightsource, const Pixel& meshcolor_s) = 0;
  virtual void activate() = 0;
  virtual void deactivate() = 0;
  static NormalMapping* get();
};

}  // namespace hh

#endif  // MESH_PROCESSING_G3DOGL_NORMALMAPPING_H_
