// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#pragma once
#include "Geometry.h"
#include "Polygon.h"

#if 0
{
    Vec3<float> ang = frame_to_euler_angles(f);
    euler_angles_to_frame(ang, f);
}
#endif

namespace hh {

// *** Radii

// Compute the circumscribed radius of the 3 points p0, p1, p2.
float circum_radius(const Point& p0, const Point& p1, const Point& p2);

// Compute the inscribed radius of the 3 points p0, p1, p2.
float inscribed_radius(const Point& p0, const Point& p1, const Point& p2);

float aspect_ratio(const Point& p0, const Point& p1, const Point& p2);

// *** Angles

// ret: cosf of signed angle away from "flattness" (==exterior angle)
// range -1..1  (or -2 if a triangle is degenerate)
float dihedral_angle_cos(const Point& p1, const Point& p2, const Point& po1, const Point& po2);

// return angle from -TAU/2 to TAU/2 (negative is concave), or -10 if degeneracy
//  (==exterior angle)  (cos(signed_dihedral_angle())==dihedral_angle_cos())
float signed_dihedral_angle(const Point& p1, const Point& p2, const Point& po1, const Point& po2);

// Compute solid angle at p surrounded by ordered loop pa[]
// range 0..TAU*2
float solid_angle(const Point& p, CArrayView<Point> pa);

// ret: cosf of signed angle away from "flattness" (==exterior angle)
// range -1..1  (or -2 if an edge is degenerate)
float angle_cos(const Point& p1, const Point& p2, const Point& p3);

// *** Frames and Euler angles

// Compute Euler angles of f.
Vec3<float> frame_to_euler_angles(const Frame& f);

// Modify f by setting v[0..2] according to Euler angles.  f.p() is unchanged.
void euler_angles_to_frame(const Vec3<float>& ang, Frame& f);

// Modify f so that its x axis points towards p and its y axis is vertical;
// f.p() is ignored and unchanged.
void frame_aim_at(Frame& f, const Vector& v);

// Modify f so that its y axis lies in the xy plane
void frame_make_level(Frame& f);

// Modify f so that its x and y axes lie in the xy plane.
void frame_make_horiz(Frame& f);

// Affinely broaden the triangle (in all directions) by the factor 1.f+eps*.5f .
void widen_triangle(ArrayView<Point> poly, float eps);

} // namespace hh
