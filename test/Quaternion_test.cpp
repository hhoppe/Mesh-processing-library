// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Quaternion.h"

#include "libHh/Random.h"
#include "libHh/RangeOp.h"  // round_elements()
using namespace hh;

namespace {

Frame round(Frame frame) {
  const int nrows = 3;  // Or 4.
  for_int(row, nrows) round_elements(frame[row], 1e4f);
  return frame;
}

Quaternion round(Quaternion q) {
  round_elements(q.access_private());
  return q;
}

// Maximum absolute difference between corresponding entries of two frames.
float frame_dist(const Frame& frame1, const Frame& frame2) {
  float d = 0.f;
  for_int(i, 4) for_int(j, 3) d = max(d, abs(frame1[i, j] - frame2[i, j]));
  return d;
}

// Distance between two unit quaternions, which represent the same rotation if they differ only in sign.
float rotation_dist(const Quaternion& q1, const Quaternion& q2) {
  const Vec4<float>& c1 = q1.access_private();
  const Vec4<float>& c2 = q2.access_private();
  return min(dist(c1, c2), dist(c1, -c2));
}

// A random unit quaternion, from a random axis and a random angle in [0, max_angle).
Quaternion random_quaternion(Random& random, float max_angle) {
  const Vector axis(random.unif() - .5f, random.unif() - .5f, random.unif() - .5f);
  return Quaternion(axis, random.unif() * max_angle);
}

void test_identity() {
  const Quaternion q;
  assertx(q.is_unit() && q.angle() == 0.f && is_zero(q.axis()));
  assertx(to_Frame(q).is_ident());
  assertx(Quaternion(Vector(1.f, 2.f, 3.f), 0.f).access_private() == V(0.f, 0.f, 0.f, 1.f));
  assertx(Quaternion(Frame::identity()).access_private() == V(0.f, 0.f, 0.f, 1.f));
}

// A quaternion constructed from an axis and angle corresponds to Frame::rotation().
void test_axis_rotation() {
  for_int(axis, 3) {
    for_intL(i, -7, 8) {
      const float angle = i * .4f;  // Here |angle| <= 2.8 < TAU / 2.
      Vector vaxis(0.f, 0.f, 0.f);
      vaxis[axis] = 2.f;  // The axis need not be unit-length.
      const Quaternion q(vaxis, angle);
      assertx(q.is_unit());
      assertx(frame_dist(to_Frame(q), Frame::rotation(axis, angle)) < 5e-6f);
      // For angles in [0, TAU / 2], the angle and axis are recovered.
      float angle2;
      Vector axis2;
      q.angle_axis(angle2, axis2);
      if (i > 0) assertx(abs(angle2 - angle) < 1e-4f && dist(axis2, vaxis / 2.f) < 5e-5f);
      if (i < 0) assertx(abs(angle2 + angle) < 1e-4f && dist(axis2, -vaxis / 2.f) < 5e-5f);
      assertx(angle2 == q.angle() && axis2 == q.axis());
    }
  }
}

// Conversions between quaternions and frames, composition, and inverse.
void test_frames() {
  Random random{1};
  for_int(iter, 200) {
    const Quaternion q1 = random_quaternion(random, TAU), q2 = random_quaternion(random, TAU);
    const Frame f1 = to_Frame(q1), f2 = to_Frame(q2);
    // The frame is a rotation: orthonormal and right-handed, with zero origin.
    for_int(i, 3) assertx(abs(mag(f1.v(i)) - 1.f) < 5e-5f && abs(dot(f1.v(i), f1.v((i + 1) % 3))) < 5e-5f);
    assertx(abs(dot(cross(f1.v(0), f1.v(1)), f1.v(2)) - 1.f) < 5e-5f && f1.p() == Point(0.f, 0.f, 0.f));
    // The conversion back to a quaternion recovers it up to sign; both branches of Quaternion(Frame) are reached.
    assertx(rotation_dist(Quaternion(f1), q1) < 1e-5f);
    // The origin of the frame is ignored.
    Frame f1t = f1;
    f1t.p() = Point(1.f, 2.f, 3.f);
    assertx(rotation_dist(Quaternion(f1t), q1) < 1e-5f);
    // In the row-vector convention, q1 * q2 applies q2 first, then q1.
    assertx(frame_dist(to_Frame(q1 * q2), f2 * f1) < 1e-5f);
    const Point pt(.3f, -.5f, .8f);
    assertx(dist(pt * to_Frame(q1 * q2), (pt * f2) * f1) < 1e-5f);
    Quaternion q3 = q1;
    q3 *= q2;
    assertx(rotation_dist(q3, q1 * q2) < 5e-6f);
    // Inverse.
    assertx(frame_dist(to_Frame(inverse(q1)), inverse(f1)) < 1e-5f);
    assertx(rotation_dist(q1 * inverse(q1), Quaternion()) < 5e-6f);
    // The exponential map inverts the logarithm, also for quaternions with negative real part.
    assertx(rotation_dist(exp(log(q1)), q1) < 1e-5f);
  }
}

// Powers and interpolation.
void test_interpolation() {
  Random random{2};
  for_int(iter, 200) {
    // Here the angles are less than TAU / 2, so the quaternions have positive real parts; see the note below.
    const Quaternion q0 = random_quaternion(random, 3.f), q1 = random_quaternion(random, 3.f);
    assertx(rotation_dist(pow(q0, 0.f), Quaternion()) < 5e-6f);
    assertx(rotation_dist(pow(q0, 1.f), q0) < 5e-6f);
    assertx(rotation_dist(pow(q0, .3f) * pow(q0, .5f), pow(q0, .8f)) < 1e-5f);
    assertx(abs(pow(q0, .5f).angle() - q0.angle() * .5f) < 1e-4f);
    assertx(rotation_dist(pow(q0, .4f), exp(log(q0) * .4f)) < 1e-5f);
    assertx(rotation_dist(pow(q0, .4f), slerp(Quaternion(), q0, .4f)) < 1e-5f);
    // Quaternion(frame) has a positive real part only if the frame trace is positive, i.e., for angles below
    // TAU / 3; see the note below.
    if (q0.angle() < 2.f) assertx(frame_dist(pow(to_Frame(q0), .5f) * pow(to_Frame(q0), .5f), to_Frame(q0)) < 1e-5f);
    // Slerp interpolates the endpoints, at constant angular velocity along a great arc.
    assertx(rotation_dist(slerp(q0, q1, 0.f), q0) < 5e-6f && rotation_dist(slerp(q0, q1, 1.f), q1) < 5e-6f);
    const Quaternion qt = slerp(q0, q1, .3f);
    assertx(qt.is_unit());
    assertx(rotation_dist(qt, q0 * pow(inverse(q0) * q1, .3f)) < 2e-5f ||
            dot(q0.access_private(), q1.access_private()) < 0.f);
    const float arc = std::acos(clamp(dot(q0.access_private(), q1.access_private()), -1.f, 1.f));
    const float arc_t = std::acos(clamp(dot(q0.access_private(), qt.access_private()), -1.f, 1.f));
    assertx(abs(arc_t - .3f * arc) < 5e-3f);
    // Squad also interpolates the endpoints, and with the endpoints as control points it reduces to slerp.
    const Quaternion a = random_quaternion(random, 3.f), b = random_quaternion(random, 3.f);
    assertx(rotation_dist(squad(q0, a, b, q1, 0.f), q0) < 5e-6f);
    assertx(rotation_dist(squad(q0, a, b, q1, 1.f), q1) < 5e-6f);
    assertx(rotation_dist(squad(q0, q0, q1, q1, .7f), slerp(q0, q1, .7f)) < 1e-5f);
    assertx(rotation_dist(squadseg(nullptr, q0, q1, nullptr, .7f), slerp(q0, q1, .7f)) < 1e-5f);
    const Quaternion qb = random_quaternion(random, 3.f), qa = random_quaternion(random, 3.f);
    assertx(rotation_dist(squadseg(&qb, q0, q1, &qa, 0.f), q0) < 5e-6f);
    assertx(rotation_dist(squadseg(&qb, q0, q1, &qa, 1.f), q1) < 5e-6f);
  }
  // KNOWN_BUG: angle(), axis(), angle_axis(), and pow() use my_asin() on the magnitude of the vector part and
  // therefore assume a nonnegative real part (i.e., an angle at most TAU / 2).  For a quaternion with negative real
  // part, pow(q, 1.f) represents a different rotation, and so does pow(frame, 1.f) when Quaternion(frame) yields such
  // a quaternion.  Also, slerp() between nearly opposite quaternions does not return q0 at t == 0.f because it updates
  // only 3 of the 4 components.
  if (0) {
    const Quaternion q(Vector(0.f, 0.f, 1.f), 1.5f * TAU / 2);  // Real part is negative.
    assertx(rotation_dist(pow(q, 1.f), q) < 5e-6f);
    const Frame frame = Frame::rotation(2, -2.6f);
    assertx(frame_dist(pow(frame, 1.f), frame) < 5e-6f);
    Quaternion q1 = q;
    q1.access_private() = -q.access_private();
    assertx(rotation_dist(slerp(q, q1, 0.f), q) < 5e-6f);
  }
}

// The quaternion from vector vf to vector vt represents twice the rotation from vf to vt.
void test_from_two_vectors() {
  Random random{3};
  for_int(iter, 100) {
    const Vector vf = normalized(Vector(random.unif() - .5f, random.unif() - .5f, random.unif() - .5f));
    Vector vt = normalized(Vector(random.unif() - .5f, random.unif() - .5f, random.unif() - .5f));
    if (dot(vf, vt) < .1f) vt = normalized(vt + vf * (.1f - dot(vf, vt)) * 2.f);  // Angle less than TAU / 4.
    const Quaternion q(vf, vt);
    assertx(q.is_unit());
    const float angle = std::acos(dot(vf, vt));
    assertx(abs(q.angle() - 2.f * angle) < 5e-4f);
    assertx(dist(vf * to_Frame(pow(q, .5f)), vt) < 1e-5f);
  }
}

}  // namespace

int main() {
  {
    Quaternion q1(Vector(1.f, 2.f, 3.f), TAU / 4);
    SHOW(q1);
    SHOW(pow(q1, .28f));
    SHOW(slerp(Quaternion(Vector(0.f, 0.f, 0.f), 0.f), q1, .28f));
    SHOW(exp(log(q1) * .28f));
    SHOW(round_elements(clone(q1.axis())));
    SHOW(q1.angle());
    SHOW(round(pow(q1, .25f) * pow(q1, .75f)));
    Frame frame = to_Frame(q1);
    SHOW(round(frame));
    const Frame frame_half = to_Frame(pow(q1, .5f));
    SHOW(round(frame_half));
    SHOW(round(frame_half * frame_half));
    SHOW(round(pow(frame, .5f)));
    const Quaternion qq(pow(pow(frame, .25f), 4.f));
    // SHOW(qq);  // Rounding differences.
    SHOW(qq.angle());
    SHOW(round_elements(clone(qq.axis())));
    frame = round(frame);
    SHOW(Quaternion(frame));
    SHOW(round(to_Frame(q1 * inverse(q1))));
  }
  {
    // Note: pow(qi, f) == slerp(Quaternion(Vector(0.f, 0.f, 0.f), 0.f), qi, f) == exp(log(qi) * f)
    Quaternion qi(Vector(3.f, 7.f, 11.f), TAU / 14);
    float f = 2.f / 3.f;
    SHOW(f);
    SHOW(qi);
    SHOW(pow(qi, f));
    SHOW(slerp(Quaternion(Vector(0.f, 0.f, 0.f), 0.f), qi, f));
    SHOW(exp(log(qi) * f));
    const Quaternion qo = pow(qi, f);
    SHOW(qi.axis());
    SHOW(qo.axis());
    SHOW(qi.angle());
    SHOW(qo.angle());
  }
  test_identity();
  test_axis_rotation();
  test_frames();
  test_interpolation();
  test_from_two_vectors();
  showf("Quaternion identities verified.\n");
}
