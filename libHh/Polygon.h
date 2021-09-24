// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_POLYGON_H_
#define MESH_PROCESSING_LIBHH_POLYGON_H_

#include "libHh/Geometry.h"
#include "libHh/PArray.h"
#include "libHh/Pool.h"

namespace hh {

struct Bbox;

// A Polygon is an array of Points with additional functionality.
class Polygon : public PArray<Point, 4> {
  using base = PArray<Point, 4>;

 public:
  Polygon() = default;
  Polygon(const Polygon& p) = default;
  Polygon& operator=(const Polygon& p) = default;
  using base::base;  // C++11 inherit constructors
  void get_bbox(Bbox& bbox) const;
  Vector get_normal_dir() const;  // non-normalized normal
  Vector get_normal() const;      // user should check !is_zero()
  float get_planec(const Vector& pnor) const;
  float get_tolerance(const Vector& pnor, float d) const;
  float get_area() const;
  // Because Polygon is a CArrayView<Point>, we can call centroid(Polygon) -- see Geometry.h
  // Finds intersection of polygon with halfspace in +hn direction
  bool intersect_hyperplane(const Point& hp, const Vector& hn);                 // ret: is_modified
  bool intersect_bbox(const Bbox& bbox);                                        // ret: is_modified
  bool intersect_segment(const Point& p1, const Point& p2, Point& pint) const;  // ret: is_intersection
  bool intersect_line(const Point& p, const Vector& v, Point& pint) const;      // ret: is_intersection
  // Intersect with plane defined by (planenor, planed, planetol); report intersection as array of points pa.
  void intersect_plane(const Vector& polynor, const Vector& planenor, float planed, float planetol,
                       Array<Point>& pa) const;
  bool point_inside(const Vector& pnor, const Point& point) const;  // ret: point_is_inside
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

// Returns is_intersection and if there is, intersection point pint.
bool intersect_plane_segment(const Vector& normal, float d, const Point& p1, const Point& p2, Point& pint);

// Find a vector perpendicular to v (any such vector).
Vector orthogonal_vector(const Vector& v);

// Assign a predictable direction to v.
void vector_standard_direction(Vector& v);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_POLYGON_H_
