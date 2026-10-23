// -*- C++ -*-
#ifndef MESH_PROCESSING_LIBHH_SPHEREIMAGE_H_
#define MESH_PROCESSING_LIBHH_SPHEREIMAGE_H_

#include "libHh/Image.h"

namespace hh {

// The image of a spherical domain, as created by "SphereSample", maps the sphere through the faces of a polyhedron
// ("tetra", "octa", "octaflat", or "cube") unfolded into a square image.

// Returns true if `domain` names the image of a spherical domain.
[[nodiscard]] bool is_sphere_image_domain(std::string_view domain);

// Blends the pixels across the discontinuities of the image of a spherical domain, so that texture lookups near
// each discontinuity agree from both of its sides.  The texturing mode is assumed to be "clamp to edge".
void feather_sphere_image(Image& image, std::string_view domain);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_SPHEREIMAGE_H_
