// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#ifndef MESH_PROCESSING_LIBHH_FRAMEIO_H_
#define MESH_PROCESSING_LIBHH_FRAMEIO_H_

#include "libHh/Geometry.h"

namespace hh {

class RBuffer;
class WBuffer;

struct ObjectFrame {
  Frame frame;
  int obn = 0;
  float zoom = 0.f;
  bool binary = false;
};

namespace FrameIO {

// Read Frame objects from std::stream or RBuffer.
enum class ERecognize { parse_error, no, partial, yes };
ERecognize recognize(RBuffer& b);
bool read(std::istream& is, ObjectFrame& object_frame);  // ret is_success
bool read(RBuffer& b, ObjectFrame& object_frame);        // ret is_success
Frame parse_frame(const string& s);

// Write Frame objects to std::stream or WBuffer.
bool write(std::ostream& os, const ObjectFrame& object_frame);  // ret is_success
bool write(WBuffer& b, const ObjectFrame& object_frame);        // ret is_success
string create_string(const ObjectFrame& object_frame);

// Detect special frames.
bool is_not_a_frame(const Frame& f);
Frame get_not_a_frame();

}  // namespace FrameIO

}  // namespace hh

//----------------------------------------------------------------------------
//
// NAME
//   Frame - ascii representation of a 4 * 3 affine transformation (plus a field-of-view scalar).
//
// DESCRIPTION
//   A frame is stored as a single-line beginning with 'F' and consisting of 1 integer and 13 float numbers:
//
//   F object_id  v00 v01 v02  v10 v11 v12  v20 v21 v22  p0 p1 p2  zoom
//
//   The object_id integer specifies the object associated with the frame.
//   By convention, object 0 refers to the eye-to-world transformation,
//   and object 1 refers to the object1-to-world transformation.
//
//   The next 9 floating-point values are the world-space coordinates of the (front, left, and up) unit vectors.
//   The next 3 floating-point values are the world-space coordinates of the frame origin.
//
//   The final floating-point value is the "zoom" factor.  It is equal to tan(half_angle) where half_angle is
//   the angle between the front vector and the nearest edge of the viewport.
//   Thus, zoom == 1.f for a square window corresponds to a horizontal field-of-view of 90 degrees.
//   The zoom value is typically used only for object 0 (the eye frame).
//
//   In many contexts, the X, Y, Z world axes area associated with "forward", "left", and "up" directions,
//   consistent with the ordering of the axes represented by the frame.
//
//   The identify frame is
//   F 0  1 0 0  0 1 0  0 0 1  0 0 0  0
//   where the object_id == 0 and zoom == 0 values are often unused.
//
//   For example, the eye-to-world frame that forms a good viewpoint for the demos/data/dragon.pm model is stored
//   in demos/data/dragon.s3d :
//   F 0  0.43794 -0.0692833 -0.896331  -0.893573 0.0759494 -0.442457  0.0987296 0.994707 -0.0286471  -0.207925 0.15072 0.389383  0.224572
//   Note that all 3 direction vectors have unit norm, and that for a square window the zoom == 0.224572 corresponds
//   to a horizontal field-of-view of 25.31 degrees.

#endif  // MESH_PROCESSING_LIBHH_FRAMEIO_H_
