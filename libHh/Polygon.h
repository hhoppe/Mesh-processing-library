// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_POLYGON_H_
#define MESH_PROCESSING_LIBHH_POLYGON_H_

#include "libHh/Bbox.h"
#include "libHh/Geometry.h"
#include "libHh/PArray.h"
#include "libHh/Pool.h"

namespace hh {

// A Polygon is an array of Points with additional functionality.
class Polygon : public PArray<Point, 4> {
  using base = PArray<Point, 4>;

 public:
  Polygon() = default;
  Polygon(const Polygon& p) = default;
  Polygon& operator=(const Polygon& p) = default;
  using base::base;               // Inherit constructors.
  Vector get_normal_dir() const;  // non-normalized normal
  Vector get_normal() const;      // user should check !is_zero()
  float get_planec(const Vector& pnor) const;
  float get_tolerance(const Vector& pnor, float d) const;
  float get_area() const;
  // The polygon centroid can be obtained as mean(polygon) using RangeOp.
  // Finds intersection of polygon with halfspace in +hn direction
  bool intersect_hyperplane(const Point& hp, const Vector& hn);  // ret: is_modified
  bool intersect_bbox(const Bbox<float, 3>& bbox);               // ret: is_modified
  std::optional<Point> intersect_segment(const Point& p1, const Point& p2) const;
  std::optional<Point> intersect_line(const Point& p, const Vector& v) const;
  // Intersect with plane defined by (plane_normal, plane_d, plane_tol); report intersection as array of points pa.
  void intersect_plane(const Vector& poly_normal, const Vector& plane_normal, float plane_d, float plane_tol,
                       Array<Point>& pa) const;
  bool point_inside(const Vector& pnor, const Point& point) const;
  bool is_convex() const;
  using base::operator=;
  HH_POOL_ALLOCATION(Polygon);
};

// Intersect 2 polygons.  pairs (pa[2 * i], pa[2 * i + 1]) are resulting segments.
Array<Point> intersect_poly_poly(const Polygon& p1, const Polygon& p2);  // If compiler error on this line, then
// it is likely that '#include <Windows.h>' invoked '#include <wingdi.h>' which defined Polygon().
// If so, use '#define NOGDI' prior to '#include <Windows.h>',
// or wrap '#include <Windows.h>' with '#define Polygon Win32_Polygon' and '#undef Polygon'.

std::ostream& operator<<(std::ostream& os, const Polygon& poly);
template <> HH_DECLARE_OSTREAM_EOL(Polygon);  // implemented by CArrayView<Point>

HH_INITIALIZE_POOL(Polygon);

// Returns the intersection point.
std::optional<Point> intersect_plane_segment(const Vector& normal, float d, const Point& p1, const Point& p2);

// Find a vector perpendicular to v (any such vector).
Vector orthogonal_vector(const Vector& v);

// Assign a predictable direction to v.
void vector_standard_direction(Vector& v);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_POLYGON_H_
