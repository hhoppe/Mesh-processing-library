// -*- C++ -*-  Copyright (c) Microsoft Corp; see LICENSE.
#include <regex>

#include "libHh/Args.h"
#include "libHh/Array.h"
#include "libHh/BinaryIO.h"
#include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"
#include "libHh/HashFloat.h"
#include "libHh/Image.h"
#include "libHh/Map.h"
#include "libHh/Matrix.h"
#include "libHh/MeshOp.h"
#include "libHh/MeshSearch.h"
#include "libHh/Parallel.h"
#include "libHh/Polygon.h"
#include "libHh/Random.h"
#include "libHh/Spatial.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"
using namespace hh;

// The UV coordinates in this file do not take into account my recent Y flip of Image domain?

namespace {

string domain = "octa";  // {tetra, octa, cube, octaflat}.
string scheme = "";      // {best, Qdomain, Qsphere, ... , Tdomain, Tsphere, Tslerps, ...}.
int gridn = 0;
int checkern = 0;
bool octa8colors = false;
bool baseball = false;
bool omit_faces = false;
string param_file;  // Filename for param_mesh.
string rotate_s3d;  // Filename for s3d file.
string signal_;     // Surface signal {G, N, C} (for geometry, normal, or color).
Array<string> key_names{"sph"};
int verbose = 1;
bool first_domain_face = false;
bool nooutput = false;

struct DomainFace {
  Polygon poly;          // Points on regular polyhedron in 3D.
  Array<UV> stretchuvs;  // For stretch optimization domain -> sphere.
  Array<UV> imageuvs;    // For texture-mapping.
  Pixel facecolor;
  Pixel pixel_checker;
};

GMesh mesh;
bool is_remeshed = false;

constexpr Vector k_undefined_vector(-2.f, 0.f, 0.f);

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Point, v_domainp);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, UV, v_stretchuv);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, UV, v_imageuv);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, UV, v_ijuv);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Point, v_sph);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Vector, v_normal);  // May be k_undefined_vector.
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Vector, v_rgb);     // May be k_undefined_vector.
HH_SAC_ALLOCATE_FUNC(Mesh::MFace, int, f_domainf);

const Frame k_fdomain_to_spatialbb{Vector(.4f, .0f, .0f), Vector(.0f, .4f, .0f), Vector(.0f, .0f, .4f),
                                   Point(.5f, .5f, .5f)};
const Pixel k_pixel_gray = Pixel::gray(235);
const Pixel k_pixel_orange{255, 153, 0, 255};

// *** General functions.

inline Point bilerp(const Vec<Point, 4>& pa, float u, float v) { return bilerp(pa[0], pa[1], pa[2], pa[3], u, v); }

bool in_spheretri(const Point& p, const Vec<Point, 3>& tri) {
  ASSERTX(is_unit(p));
  for_int(i, 3) ASSERTX(is_unit(tri[i]));
  const float dotcross_eps = 2e-7f;
  for_int(i, 3) if (dot(Vector(p), cross(p, tri[i], tri[mod3(i + 1)])) < -dotcross_eps) return false;
  return true;
}

float angle_between_unit_vectors_and_sincos(const Vector& v1, const Vector& v2, float& sina, float& cosa) {
  cosa = dot(v1, v2);
  const float thresh = 0.9475f;
  if (cosa > +thresh) {
    sina = mag(cross(v1, v2));
    return std::asin(sina);
  } else if (cosa < -thresh) {
    sina = mag(cross(v1, v2));
    return float(D_TAU / 2) - std::asin(sina);
  } else {
    sina = sqrt(1.f - square(cosa));
    return std::acos(cosa);
  }
}

Point spheremap_linear_reproject(const Point& pa, const Point& pb, const Point& pc, const Bary& b) {
  ASSERTX(b.is_convex());
  return ok_normalized(b[0] * pa + b[1] * pb + b[2] * pc);
}

[[maybe_unused]] Point spheremap_inverse_linear_reproject(const Point& pa, const Point& pb, const Point& pc,
                                                          const Point& p) {
  const Vector normal = ok_normalized(cross(pb - pa, pc - pa));
  const Point pp = p + normal * dot(pa - p, normal);  // Projection of p to plane.
  const float pab = dot(normal, cross(pp - pa, pp - pb));
  const float pbc = dot(normal, cross(pp - pb, pp - pc));
  const float pca = dot(normal, cross(pp - pc, pp - pa));
  const float value = pab + pbc + pca;
  return V(pbc, pca, pab) / value;
}

Frame get_rotate_frame() {
  if (rotate_s3d == "") return Frame::identity();

  RFile fi(rotate_s3d);
  for (string line; fi().peek() == '#';) assertx(my_getline(fi(), line));
  const ObjectFrame object_frame = *assertx(FrameIO::read(fi()));
  Frame frame = object_frame.frame;
  assertw(mag(cross(frame.v(0), frame.v(1)) - frame.v(2)) < 1e-4f);
  frame = ~frame;

  // Snap the frame axes to the nearest canonical axes.
  for_int(i, 3) {
    Vector& vec = frame.v(i);
    int axis = arg_max(abs(vec));
    for_int(j, 3) vec[j] = j == axis ? sign(vec[j]) : 0.f;
  }

  frame.p() = Point(0.f, 0.f, 0.f);
  return frame;
}

// *** Create domain.

Array<DomainFace> get_domain_faces() {
  Array<DomainFace> domain_faces;
  if (0) {
  } else if (domain == "tetra") {
    // Tetrahedron: vertex 3 at +Z (0, 0, 1), and vertex 0 along -X direction.
    //  image order   (edge 1-2 occluded)
    /*
        //   2 - 3 - 2        / 3 \
        //   | / | / |       1  |  2
        //   1 - 0 - 1        \ 0 /
        */
    // Tetrahedral faces are 103, 213, 023, 012.
    domain_faces.init(4);
    domain_faces[0].facecolor = Pixel::green();
    domain_faces[1].facecolor = Pixel::blue();
    domain_faces[2].facecolor = Pixel::red();
    domain_faces[3].facecolor = k_pixel_orange;
    const Vec<Vector, 4> vtmp = []() {
      Vec<Vector, 4> v;
      v[0] = Vector(-1.f / sqrt(3.f), 0.f, 0.f);
      v[1] = Vector(+sqrt(3.f) / 6.f, +.5f, 0.f);
      v[2] = Vector(+sqrt(3.f) / 6.f, -.5f, 0.f);
      v[3] = Vector(0.f, 0.f, sqrt(6.f) / 3.f);
      for_int(i, 4) {
        v[i] -= v[3] / 4.f;
        v[i].normalize();
      }
      return v;
    }();
    for_int(i, domain_faces.num()) {
      domain_faces[i].poly.init(3);
      domain_faces[i].stretchuvs.init(3);
      domain_faces[i].imageuvs.init(3);
      if (i < 3) {
        domain_faces[i].poly[0] = vtmp[mod3(i + 1)];
        domain_faces[i].poly[1] = vtmp[i];
        domain_faces[i].poly[2] = vtmp[3];
      } else {
        domain_faces[i].poly[0] = vtmp[0];
        domain_faces[i].poly[1] = vtmp[1];
        domain_faces[i].poly[2] = vtmp[2];
      }
      domain_faces[i].stretchuvs[0] = UV(0.f, 0.f);
      domain_faces[i].stretchuvs[1] = UV(1.f, 0.f);
      domain_faces[i].stretchuvs[2] = UV(.5f, .5f * sqrt(3.f));
      // The imageuv's are defined below.
    }
    int i = 0;
    // 103: face 1 in left-to-right image order.
    domain_faces[i].imageuvs[0] = UV(.00f, .00f);
    domain_faces[i].imageuvs[1] = UV(.50f, .00f);
    domain_faces[i].imageuvs[2] = UV(.50f, .50f);
    i++;
    // 213: face 0 in left-to-right image order.
    domain_faces[i].imageuvs[0] = UV(.00f, .50f);
    domain_faces[i].imageuvs[1] = UV(.00f, .00f);
    domain_faces[i].imageuvs[2] = UV(.50f, .50f);
    i++;
    // 023: face 2 in left-to-right image order.
    domain_faces[i].imageuvs[0] = UV(.50f, .00f);
    domain_faces[i].imageuvs[1] = UV(1.0f, .50f);
    domain_faces[i].imageuvs[2] = UV(.50f, .50f);
    i++;
    // 012: face 3 in left-to-right image order.
    domain_faces[i].imageuvs[0] = UV(.50f, .00f);
    domain_faces[i].imageuvs[1] = UV(1.0f, .00f);
    domain_faces[i].imageuvs[2] = UV(1.0f, .50f);
    i++;
    assertx(i == domain_faces.num());
  } else if (domain == "octa" || domain == "octaflat") {
    // Image face order:
    //  -X vertex at image center,
    //  +X vertex at 4 image corners,
    //  (-Y, +Z) to image (+X, +Y).
    domain_faces.init(8);
    if (octa8colors) {
      domain_faces[0].facecolor = Pixel::red();
      domain_faces[1].facecolor = Pixel::green();
      domain_faces[2].facecolor = Pixel::blue();
      domain_faces[3].facecolor = k_pixel_orange;
      domain_faces[4].facecolor = Pixel(70, 20, 180);
      domain_faces[5].facecolor = Pixel(190, 153, 190);
      domain_faces[6].facecolor = Pixel(180, 70, 20);
      domain_faces[7].facecolor = Pixel(20, 180, 70);
    } else {
      domain_faces[0].facecolor = Pixel::red();
      domain_faces[1].facecolor = Pixel::green();
      domain_faces[2].facecolor = Pixel::blue();
      domain_faces[3].facecolor = k_pixel_orange;
      for_int(i, 4) domain_faces[4 + i].facecolor = domain_faces[(i + 2) % 4].facecolor;
    }
    domain_faces[0].pixel_checker = Pixel::red();
    domain_faces[1].pixel_checker = Pixel::green();
    domain_faces[2].pixel_checker = Pixel::blue();
    domain_faces[3].pixel_checker = k_pixel_orange;
    domain_faces[4].pixel_checker = Pixel(120, 190, 190);
    domain_faces[5].pixel_checker = Pixel(180, 180, 120);
    domain_faces[6].pixel_checker = Pixel(190, 110, 190);
    domain_faces[7].pixel_checker = Pixel(160, 190, 120);
    constexpr Vec4<Vector> dir =
        V(Vector(+1.f, 0.f, 0.f), Vector(0.f, +1.f, 0.f), Vector(-1.f, 0.f, 0.f), Vector(0.f, -1.f, 0.f));
    for_int(i, domain_faces.num()) {
      domain_faces[i].poly.init(3);
      domain_faces[i].stretchuvs.init(3);
      domain_faces[i].imageuvs.init(3);
      const bool front = i < 4;
      const int q = front ? i : i - 4;
      Vector dir0 = dir[q], dir1 = dir[(q + 1) % 4];
      if (!front) std::swap(dir0, dir1);
      domain_faces[i].poly[0] = Point(0.f, -dir0[0], dir0[1]);
      domain_faces[i].poly[1] = Point(0.f, -dir1[0], dir1[1]);
      domain_faces[i].poly[2] = Point(front ? -1.f : +1.f, 0.f, 0.f);
      domain_faces[i].stretchuvs[0] = UV(0.f, 0.f);
      domain_faces[i].stretchuvs[1] = UV(1.f, 0.f);
      domain_faces[i].stretchuvs[2] = UV(.5f, .5f * sqrt(3.f));
      domain_faces[i].imageuvs[0] = UV(.5f * (1.f + dir0[0]), .5f * (1.f + dir0[1]));
      domain_faces[i].imageuvs[1] = UV(.5f * (1.f + dir1[0]), .5f * (1.f + dir1[1]));
      if (front) {
        domain_faces[i].imageuvs[2] = UV(.5f, .5f);
      } else {
        domain_faces[i].imageuvs[2] = UV(.5f * (1.f + dir0[0] + dir1[0]), .5f * (1.f + dir0[1] + dir1[1]));
      }
    }
  } else if (domain == "cube") {
    // First  vertex in each face is (lower-left)  in below diagrams.
    // Second vertex in each face is (lower-right) in below diagrams.
    //
    // Cross image face order:
    //      5Z+
    //  0Y+ 1X- 2Y- 3X+
    //      4Z-
    // Baseball image face order:
    //  2Y- 5Z-
    //  1X- 4X+
    //  0Y+ 3Z+
    // Fully supporting baseball would require changes in adjust_for_domain(), non 1-1 mapping in do_write_texture(),
    //  etc.
    domain_faces.init(6);
    domain_faces[0].facecolor = Pixel::green();
    domain_faces[1].facecolor = Pixel::red();
    domain_faces[2].facecolor = Pixel::green();
    domain_faces[3].facecolor = Pixel::red();
    domain_faces[4].facecolor = Pixel::blue();
    domain_faces[5].facecolor = Pixel::blue();
    for_int(i, domain_faces.num()) {
      domain_faces[i].poly.init(4);
      domain_faces[i].stretchuvs.init(4);
      domain_faces[i].imageuvs.init(4);
      // Poly's are defined below.
      domain_faces[i].stretchuvs[0] = UV(0.f, 0.f);
      domain_faces[i].stretchuvs[1] = UV(1.f, 0.f);
      domain_faces[i].stretchuvs[2] = UV(1.f, 1.f);
      domain_faces[i].stretchuvs[3] = UV(0.f, 1.f);
      hh::Vec2<float> s;
      UV uvo;
      if (!baseball) {  // cross.
        s = twice(1.f / 4.f);
        uvo[0] = s[1] * (i < 4 ? i : 1);
        uvo[1] = s[0] * (i < 4 ? 1 : i == 4 ? 0 : 2);
      } else {  // baseball.
        s = V(1.f / 3.f, 1.f / 2.f);
        uvo[0] = i < 3 ? 0.f : s[1];
        uvo[1] = s[0] * V(0, 1, 2, 0, 1, 2)[i];
      }
      domain_faces[i].imageuvs[0] = UV(uvo[0] + 0, uvo[1] + 0);
      domain_faces[i].imageuvs[1] = UV(uvo[0] + s[1], uvo[1] + 0);
      domain_faces[i].imageuvs[2] = UV(uvo[0] + s[1], uvo[1] + s[0]);
      domain_faces[i].imageuvs[3] = UV(uvo[0] + 0, uvo[1] + s[0]);
    }
    const float s = sqrt(1.f / 3.f);
    Vec<Vec4<hh::Vec3<int>>, 6> fvcoords;
    if (!baseball) {                                                               // cross.
      fvcoords = V(V(V(+1, +1, -1), V(-1, +1, -1), V(-1, +1, +1), V(+1, +1, +1)),  // Y+
                   V(V(-1, +1, -1), V(-1, -1, -1), V(-1, -1, +1), V(-1, +1, +1)),  // X-
                   V(V(-1, -1, -1), V(+1, -1, -1), V(+1, -1, +1), V(-1, -1, +1)),  // Y-
                   V(V(+1, -1, -1), V(+1, +1, -1), V(+1, +1, +1), V(+1, -1, +1)),  // X+
                   V(V(+1, +1, -1), V(+1, -1, -1), V(-1, -1, -1), V(-1, +1, -1)),  // Z-
                   V(V(-1, +1, +1), V(-1, -1, +1), V(+1, -1, +1), V(+1, +1, +1))   // Z+
      );
    } else {                                                                       // baseball.
      fvcoords = V(V(V(+1, +1, +1), V(+1, +1, -1), V(-1, +1, -1), V(-1, +1, +1)),  // Y+
                   V(V(-1, +1, +1), V(-1, +1, -1), V(-1, -1, -1), V(-1, -1, +1)),  // X-
                   V(V(-1, -1, +1), V(-1, -1, -1), V(+1, -1, -1), V(+1, -1, +1)),  // Y-
                   V(V(-1, +1, +1), V(-1, -1, +1), V(+1, -1, +1), V(+1, +1, +1)),  // Z+
                   V(V(+1, +1, +1), V(+1, -1, +1), V(+1, -1, -1), V(+1, +1, -1)),  // X+
                   V(V(+1, +1, -1), V(+1, -1, -1), V(-1, -1, -1), V(-1, +1, -1))   // Z-
      );
    }
    for_int(i, 6) for_int(j, 4) domain_faces[i].poly[j] = convert<float>(fvcoords[i][j]) * Point(s, s, s);
  } else {
    assertnever("domain name '" + domain + "' not recognized");
  }
  return domain_faces;
}

Pixel get_color(const DomainFace& domain_face, UV uv) {
  bool is_quad = domain_face.poly.num() == 4;
  int local_checkern = checkern;
  Pixel domain_face_color = domain_face.facecolor;
  if (domain == "octaflat") {
    is_quad = true;
    local_checkern = 2 * checkern;
    domain_face_color = Pixel::blue();
    const Bary bary(1.f - uv[0] - uv[1], uv[0], uv[1]);
    uv = interp(domain_face.imageuvs[0], domain_face.imageuvs[1], domain_face.imageuvs[2], bary);
  }
  if (1) {
    const float eps = 1e-6f;
    // Pull uv inside the margins of the axis-aligned edges.
    for_int(c, 2) uv[c] = clamp(uv[c], eps, 1.f - eps);
    if (!is_quad && 1) {
      // Pull uv inside the margin of the diagonal edge.
      const float d = max(uv[0] + uv[1] - (1.f - eps), 0.f) / sqrt(2.f);
      uv -= d;
      if (uv[0] < eps)
        uv = UV(eps, 1.f - 2.f * eps);
      else if (uv[1] < eps)
        uv = UV(1.f - 2.f * eps, eps);
      // (The checker color is incorrect at checker intersections along the diagonal; oh well.)
    }
  }
  const int gi = int(uv[0] * local_checkern);
  const int gj = int(uv[1] * local_checkern);
  const int gd = int((uv[0] + uv[1]) * local_checkern);
  const bool is_on = is_quad ? (gi + gj) % 2 == 1 : (gi + gj + gd) % 2 == 1;
  return is_on ? k_pixel_gray : domain_face_color;
}

// *** Spherical maps.

Point spheremap_2slerp0(const Point& pa, const Point& pb, const Point& pc, const Bary& b) {
  ASSERTX(b.is_convex());
  const float b12 = b[1] + b[2];
  const float srev = b12 != 0.f ? b[1] / b12 : 0.f;  // == 1.f - s
  const Point pd = slerp(pb, pc, srev);
  return slerp(pd, pa, b12);
}

template <typename Function>
Point symmetrize(Function function, const Point& pa, const Point& pb, const Point& pc, const Bary& b) {
  ASSERTX(b.is_convex());
  const Point result0 = function(pa, pb, pc, b);
  const Point result1 = function(pb, pc, pa, V(b[1], b[2], b[0]));
  const Point result2 = function(pc, pa, pb, V(b[2], b[0], b[1]));
  return ok_normalized(result0 + result1 + result2);
}

Point spheremap_sym_2slerps(const Point& pa, const Point& pb, const Point& pc, const Bary& b) {
  return symmetrize(spheremap_2slerp0, pa, pb, pc, b);
}

Point spheremap_arvo0(const Point& pa, const Point& pb, const Point& pc, const Bary& b) {
  ASSERTX(b.is_convex());
  // Compute the inverse of map square -> triangle with uniform distribution.
  float s, t;
  if (b[0] == 1.f) {
    s = t = 0.f;
  } else {
    s = b[2] / (1.f - b[0]);
    t = square(1.f - b[0]);
  }

  // Compute edge directions for spherical triangle.
  const Vector v_ca = ok_normalized(pc - pa * dot<float>(pc, pa));
  const Vector v_ba = ok_normalized(pb - pa * dot<float>(pb, pa));
  const Vector v_ab = ok_normalized(pa - pb * dot<float>(pb, pa));
  const Vector v_cb = ok_normalized(pc - pb * dot<float>(pb, pc));
  const Vector v_bc = ok_normalized(pb - pc * dot<float>(pb, pc));
  const Vector v_ac = ok_normalized(pa - pc * dot<float>(pa, pc));
  // Compute angles.
  const float angle_a = angle_between_unit_vectors(v_ca, v_ba);
  float cos_angle_b, sin_angle_b;
  const float angle_b = angle_between_unit_vectors_and_sincos(v_ab, v_cb, sin_angle_b, cos_angle_b);
  const float angle_c = angle_between_unit_vectors(v_ac, v_bc);
  const float area = angle_a + angle_b + angle_c - float(D_TAU / 2);

  // Use one random variable to choose area of subtriangle.
  const float abd = s * area;
  const float ss = std::sin(abd - angle_b), tt = std::cos(abd - angle_b);
  const float u = tt - cos_angle_b, v = ss + sin_angle_b * dot(Vector(pa), Vector(pb));
  const float q = ((v * tt - u * ss) * cos_angle_b - v) / ((v * ss + u * tt) * sin_angle_b);
  const Vector pd = q * pb + my_sqrt(1.f - q * q) * v_cb;  // Numerical problems when q is very close to 1.f!
  const Vector v_da = ok_normalized(pd - pa * dot(pd, Vector(pa)));

  // Use other variable to choose a point on this new line.
  const float z = 1.f - t * (1.f - dot(pd, Vector(pa)));
  return z * pa + my_sqrt(1.f - z * z) * v_da;
}

Point spheremap_sym_arvo(const Point& pa, const Point& pb, const Point& pc, const Bary& b) {
  return symmetrize(spheremap_arvo0, pa, pb, pc, b);
}

Point spheremap_sym_buss_fillmore(const Point& pa, const Point& pb, const Point& pc, const Bary& b) {
  ASSERTX(b.is_convex());
  const Vec3<Point> pp = V(pa, pb, pc);
  Point p = ok_normalized(b[0] * pa + b[1] * pb + b[2] * pc);
  SGrid<float, 3, 3> tp;
  const float eps = 1e-4f;
  for_int(i, 100) {
    Vector q{};
    // Map A, B, C to tangent plane at p using inverse exponential map.
    for_int(v, 3) {
      float sin_theta, cos_theta;
      const float theta = angle_between_unit_vectors_and_sincos(p, pp[v], sin_theta, cos_theta);
      if (cos_theta >= 1.f || abs(sin_theta) < eps / 10) {
        ASSERTX(b[v] > 1.f - eps);
        return pp[v] * b[v];
      }
      ASSERTX(sin_theta != 0.f);
      for_int(k, 3) {
        tp[v][k] = (pp[v][k] - p[k] * cos_theta) / sin_theta;  // Unit vector perpendicular to p.
        tp[v][k] *= theta;
        q[k] += tp[v][k] * b[v];
      }
    }
    // Map q back to sphere using exponential map.
    const float r = mag(q);
    if (r < eps) break;
    q /= r;
    p = p * std::cos(r) + q * std::sin(r);
    assertx(is_unit(p));  // Check that it is at least approximately unit length.
    p.normalize();
  }
  return p;
}

Point spheremap_area_ratio(const Point& pa, const Point& pb, const Point& pc, const Bary& b) {
  ASSERTX(b.is_convex());
  const float area = spherical_triangle_area(V(pa, pb, pc));

  Vector normal_locusE, normal_locusD;
  {
    // The locus of points that together with edge bc form a triangle with a given area is a small circle that
    // passes through -B and -C.  We find the pole of that circle.
    // U is the unit vector to the halfpoint between -B, -C.
    // V is the unit vector orthogonal to the plane B, C, -B, -C.
    Vector pu, pv;
    float tgalpha, denom;

    const float mag2_bc = mag2(pb - pc), mag_bc = sqrt(mag2_bc);
    pu = (pb + pc) / -sqrt(4.f - mag2_bc);
    pv = cross(pu, Vector(pb)) * (2.f / mag_bc);
    ASSERTX(is_unit(pu) && is_unit(pv));
    tgalpha = tan((float(D_TAU / 2) - b[0] * area) * .5f);
    denom = sqrt(4.f + mag2_bc * square(tgalpha));
    normal_locusE = pv * (mag_bc * tgalpha / denom) + pu * (2.f / denom);

    // We now do the same for edge ab.
    const float mag2_ab = mag2(pa - pb), mag_ab = sqrt(mag2_ab);
    pu = (pa + pb) / -sqrt(4.f - mag2_ab);
    pv = cross(pu, Vector(pa)) * (2.f / mag_ab);
    ASSERTX(is_unit(pu) && is_unit(pv));
    tgalpha = tan((float(D_TAU / 2) - b[2] * area) * .5f);
    denom = sqrt(4.f + mag2_ab * square(tgalpha));
    normal_locusD = pv * (mag_ab * tgalpha / denom) + pu * (2.f / denom);
  }

  // Intersect the 2 small circles (planes).  One intersection point will be -B; the other is the one we want.
  const Vector dir_isect = ok_normalized(cross(normal_locusE, normal_locusD));
  return ok_normalized(-pb + dir_isect * (2.f * dot(dir_isect, Vector(pb))));
}

// *** Maps.

using Trispheremap = Point (*)(const Vec<Point, 3>& pt, const Bary& bary);

Point map_sphere(const Vec<Point, 3>& pt, const Bary& bary) {
  // return normalized(interp(pt[0], pt[1], pt[2], bary[0], bary[1]));
  return spheremap_linear_reproject(pt[0], pt[1], pt[2], bary);
}

Point map_2slerp0(const Vec<Point, 3>& pt, const Bary& bary) { return spheremap_2slerp0(pt[0], pt[1], pt[2], bary); }

Point map_2slerp1(const Vec<Point, 3>& pt, const Bary& bary) {
  return spheremap_2slerp0(pt[1], pt[2], pt[0], Bary(bary[1], bary[2], bary[0]));
}

Point map_2slerp2(const Vec<Point, 3>& pt, const Bary& bary) {
  return spheremap_2slerp0(pt[2], pt[0], pt[1], Bary(bary[2], bary[0], bary[1]));
}

Point map_2slerps(const Vec<Point, 3>& pt, const Bary& bary) {
  return spheremap_sym_2slerps(pt[0], pt[1], pt[2], bary);
}

Point map_arvo0(const Vec<Point, 3>& pt, const Bary& bary) { return spheremap_arvo0(pt[0], pt[1], pt[2], bary); }

Point map_arvo1(const Vec<Point, 3>& pt, const Bary& bary) {
  return spheremap_arvo0(pt[1], pt[2], pt[0], Bary(bary[1], bary[2], bary[0]));
}

Point map_arvo2(const Vec<Point, 3>& pt, const Bary& bary) {
  return spheremap_arvo0(pt[2], pt[0], pt[1], Bary(bary[2], bary[0], bary[1]));
}

Point map_arvos(const Vec<Point, 3>& pt, const Bary& bary) { return spheremap_sym_arvo(pt[0], pt[1], pt[2], bary); }

Point map_buss(const Vec<Point, 3>& pt, const Bary& bary) {
  return spheremap_sym_buss_fillmore(pt[0], pt[1], pt[2], bary);
}

Point map_area(const Vec<Point, 3>& pt, const Bary& bary) { return spheremap_area_ratio(pt[0], pt[1], pt[2], bary); }

Point map_trisub(const Vec<Point, 3>& pt, const Bary& bary) {
  ASSERTX(bary.is_convex());
  Vec<Point, 3> pc = pt;
  Bary bc = bary;
  for_int(iter, 100) {
    for_int(i, 3) ASSERTX(bc[i] >= 0.f && bc[i] <= 1.f);
    const float eps = 3e-7f;
    if (dist2(pc[0], pc[1]) < square(eps) && dist2(pc[0], pc[2]) < square(eps) && dist2(pc[1], pc[2]) < square(eps))
      return pc[0];
    for_int(i, 3) {
      if (bc[i] > .5f) {
        const int i1 = mod3(i + 1), i2 = mod3(i + 2);
        pc = V(pc[i], slerp(pc[i], pc[i1], .5f), slerp(pc[i], pc[i2], .5f));
        bc = V(2 * (bc[i] - .5f), 2 * bc[i1], 2 * bc[i2]);
        break;
      }
      if (i == 2) {
        pc = V(slerp(pc[1], pc[2], .5f), slerp(pc[2], pc[0], .5f), slerp(pc[0], pc[1], .5f));
        bc = 1.f - 2.f * bc;
      }
    }
  }
  assertnever("no convergence: " + SSHOW(dist(pc[0], pc[1]), dist(pc[1], pc[2]), dist(pc[2], pc[0])));
}

struct S_Trispheremaps {
  const char* name;
  Trispheremap map;
};

const Array<S_Trispheremaps> k_trispheremaps = {
#define E(x) \
  { #x, map_##x }
    E(sphere), E(2slerp0), E(2slerp1), E(2slerp2), E(2slerps), E(arvo0),
    E(arvo1),  E(arvo2),   E(arvos),   E(buss),    E(area),    E(trisub),
#undef E
};

Trispheremap get_map(const string& tmapname) {
  for (const auto& trispheremap : k_trispheremaps)
    if (tmapname == trispheremap.name) return trispheremap.map;
  assertnever("map '" + tmapname + "' not found");
}

// *** domain -> sphere grid mapping.

// Given uv coordinates within spherical quad, return a spherical triangle pt and barycentric coordinates within it.
void split_quad_2tris(const Vec<Point, 4>& po, float fi, float fj, Vec<Point, 3>& pt, Bary& bary) {
  if (fi + fj <= 1.f) {
    pt = V(po[0], po[1], po[3]);
    bary = Bary(1.f - fi - fj, fj, fi);
  } else {
    pt = V(po[2], po[3], po[1]);
    bary = Bary(fi + fj - 1.f, 1.f - fj, 1.f - fi);
  }
}

void split_quad_4tris(const Vec<Point, 4>& po, float fi, float fj, Vec<Point, 3>& pt, Bary& bary) {
  // Find wedge in face, adjacent to vertices {q, q + 1}.
  int q;
  float bq0, bq1, bc;
  if (fi + fj <= 1.f) {
    if (fi <= fj) {
      q = 0;
      bq0 = 1.f - fi - fj;
      bq1 = fj - fi;
      bc = 2 * fi;
    } else {
      q = 3;
      bq0 = fi - fj;
      bq1 = 1.f - fi - fj;
      bc = 2 * fj;
    }
  } else {
    if (fi <= fj) {
      q = 1;
      bq0 = fj - fi;
      bq1 = fi + fj - 1.f;
      bc = (1.f - fj) * 2;
    } else {
      q = 2;
      bq0 = fi + fj - 1.f;
      bq1 = fi - fj;
      bc = (1.f - fi) * 2;
    }
  }
  ASSERTX(abs(bq0 + bq1 + bc - 1.f) < 1e-6f);
  pt = V(po[q], po[(q + 1) % 4], normalized(bilerp(po, .5f, .5f)));
  bary = Bary(bq0, bq1, bc);
}

void split_quad_8tris(const Vec<Point, 4>& po, float fi, float fj, Vec<Point, 3>& pt, Bary& bary,
                      Trispheremap trimap) {
  // Find quadrant in face, adjacent to vertex q = 0..3.
  int q;
  float s, t;
  if (fj <= .5f) {
    if (fi <= .5f) {
      q = 0;
      s = fj * 2.f;
      t = fi * 2.f;
    } else {
      q = 3;
      s = (1.f - fi) * 2.f;
      t = fj * 2.f;
    }
  } else {
    if (fi <= .5f) {
      q = 1;
      s = fi * 2.f;
      t = (1.f - fj) * 2.f;
    } else {
      q = 2;
      s = (1.f - fj) * 2.f;
      t = (1.f - fi) * 2.f;
    }
  }
  assertx(s >= 0.f && s <= 1.f && t >= 0.f && t <= 1.f);
  // Split quadrant into two triangles.
  if (t <= s) {
    pt = V(po[q], normalized(interp(po[q], po[(q + 1) % 4])), normalized(bilerp(po, .5f, .5f)));
  } else {
    // Flip the triangle to obtain symmetry --- that is OK with most trimap function.
    pt = V(po[q], normalized(interp(po[q], po[(q + 3) % 4])), normalized(bilerp(po, .5f, .5f)));
    bary = Bary(max(0.f, 1.f - t), t - s, s);
    if (trimap == map_area) {
      // Flipped triangles not handled correctly by current map_area. Since it is symmetric, undoing reflection is OK.
      std::swap(pt[0], pt[1]);
      std::swap(bary[0], bary[1]);
    }
  }
}

// ***

// Tessellate the domain faces, and map onto sphere.  (Bottleneck is Mesh::create_face().)
void create(bool b_triangulate) {
  assertx(gridn > 0);
  assertx(scheme != "");
  // For cube:
  //  'Q': perform interpolation over quadrilateral faces.
  //  '2T': split each quad into 4 triangles prior to interpolation.
  //  '4T': split each quad into 4 triangles prior to interpolation.
  //  '8T': split each quad into 8 triangles prior to interpolation.
  // For tetra, octa:
  //  'T': triangle interpolation.
  const Array<DomainFace> domain_faces = get_domain_faces();
  const bool quad_domain = domain_faces[0].poly.num() == 4;  // Cube.
  if (scheme == "best") {
    if (0) {
    } else if (domain == "tetra") {
      scheme = "Tbuss";
    } else if (domain == "octa" || domain == "octaflat") {
      scheme = "Tarea";
    } else if (domain == "cube") {
      scheme = "4Tarea";
    } else {
      assertnever("domain '" + domain + "' not recognized");
    }
    showdf("Best scheme is %s\n", scheme.c_str());
  }
  if (scheme == "domain") {
    if (0) {
    } else if (domain == "tetra") {
      scheme = "Tdomain";
    } else if (domain == "octa" || domain == "octaflat") {
      scheme = "Tdomain";
    } else if (domain == "cube") {
      scheme = "Qdomain";
    } else {
      assertnever("domain '" + domain + "' not recognized");
    }
  }
  int split_quad_ntris = 0;
  bool domain_interp = false;
  bool quad_interp = false;
  string qmapname;
  Trispheremap trimap = nullptr;
  switch (scheme[0]) {
    case 'Q':
      assertx(quad_domain);
      quad_interp = true;
      qmapname = scheme.substr(1);
      if (qmapname == "domain") domain_interp = true;
      break;
    case '2':
      assertx(quad_domain);
      split_quad_ntris = 2;
      scheme.erase(0, 1);
      break;
    case '4':
      assertx(quad_domain);
      split_quad_ntris = 4;
      scheme.erase(0, 1);
      break;
    case '8':
      assertx(quad_domain);
      split_quad_ntris = 8;
      scheme.erase(0, 1);
      break;
    default: void();  // OK, do nothing.
  }
  if (quad_interp) {
  } else if (scheme[0] == 'T') {
    if (quad_domain) assertx(split_quad_ntris);
    string mapname = scheme.substr(1);
    if (mapname == "domain") {
      domain_interp = true;
    } else {
      trimap = get_map(mapname);
    }
  } else {
    assertnever("scheme not recognized: " + scheme);
  }
  //
  assertx(!mesh.num_vertices());
  //
  HH_TIMER("_create");
  ConsoleProgress cprogress;
  HashFloat hashf;  // For -slowcornermerge of uv.
  const int ndomainf = first_domain_face ? 1 : domain_faces.num();
  const bool write_imagen = getenv_bool("SPHERESAMPLE_WRITE_IMAGEN");
  for_int(domainf, ndomainf) {
    cprogress.update(float(domainf) / ndomainf);
    const DomainFace& df = domain_faces[domainf];
    const Polygon& po = df.poly;
    if (write_imagen) assertx(domain_faces.num() == 8);  // Octahedron.
    // If triangle domain face, use UL-half of matrix.
    string str;

    Matrix<Vertex> verts(gridn + 1, gridn + 1);
    for_int(i, gridn + 1) for_int(j, !quad_domain ? gridn - i + 1 : gridn + 1) {
      Vertex v = mesh.create_vertex();
      verts[i][j] = v;
      v_normal(v) = k_undefined_vector;
      v_rgb(v) = k_undefined_vector;
      Bary bary;  // For quad, bary[3] defined as 1.f - bary[0..2].
      if (!quad_domain) {
        //   v0 - v2     (+i down,  +j right)
        //   |  /
        //   v1
        const float b1 = float(i) / gridn, b2 = float(j) / gridn;
        bary = Bary(max(0.f, 1.f - b1 - b2), b1, b2);
        ASSERTX(bary.is_convex());
      } else {
        //   v0 - v1     (+i, +v down,  +j, +u right)
        //   |     |
        //   v3 - v2
        float tu = float(j) / gridn, tv = float(i) / gridn;
        bary = Bary(max(0.f, (1.f - tu) * (1.f - tv)), max(0.f, tu * (1.f - tv)), tu * tv);
      }
      Point p;
      if (!quad_domain) {
        p = interp(df.poly[0], df.poly[1], df.poly[2], bary);
      } else {
        p = qinterp(df.poly[0], df.poly[1], df.poly[2], df.poly[3], bary);
      }
      v_domainp(v) = p;
      {
        UV uv;
        if (!quad_domain) {
          uv = interp(df.stretchuvs[0], df.stretchuvs[1], df.stretchuvs[2], bary);
        } else {
          uv = qinterp(df.stretchuvs[0], df.stretchuvs[1], df.stretchuvs[2], df.stretchuvs[3], bary);
        }
        v_stretchuv(v) = uv;
      }
      {
        UV uv;
        if (!quad_domain) {
          uv = interp(df.imageuvs[0], df.imageuvs[1], df.imageuvs[2], bary);
        } else {
          uv = qinterp(df.imageuvs[0], df.imageuvs[1], df.imageuvs[2], df.imageuvs[3], bary);
        }
        uv = UV(hashf.enter(uv[0]), hashf.enter(uv[1]));
        v_imageuv(v) = uv;
      }
      if (write_imagen) {
        const UV& uv = v_imageuv(v);
        int imi = int(uv[0] * (2 * gridn) + .5f);
        int imj = int(uv[1] * (2 * gridn) + .5f);
        int imn = imj * (2 * gridn + 1) + imi + 1;
        mesh.update_string(v, "imagen", csform(str, "%d", imn));
      }
      if ((i == 0 || i == gridn) && (j == 0 || j == gridn) && contains(key_names, "domaincorner"))
        mesh.update_string(v, "domaincorner", "");
      v_ijuv(v) = UV(float(i) / gridn, float(j) / gridn);
      if (checkern) {
        const Pixel& pixel = get_color(df, v_ijuv(v));
        Vector rgb;
        for_int(c, 3) rgb[c] = float(pixel[c]) / 255.f;
        v_rgb(v) = rgb;
      }
    }

    if (quad_interp && (domain_interp || qmapname == "sphere")) {
      // Quad rasterization.
      assertx(gridn >= 1);
      for_int(i, gridn + 1) for_int(j, gridn + 1) {
        Vertex v = verts[i][j];
        const UV uv(float(j) / gridn, float(i) / gridn);
        Point p;
        if (domain_interp) {
          p = bilerp(po, uv[0], uv[1]);
        } else {  // Sphere map.
          p = normalized(bilerp(po, uv[0], uv[1]));
        }
        mesh.set_point(v, p);
      }

    } else if (quad_interp) {
      // Recursive quad subdivision.
      if (!is_pow2(gridn)) SHOW(gridn);
      assertx(gridn >= 1 && is_pow2(gridn));
      const bool is_warp = qmapname == "warp";
      const bool is_diags = qmapname == "diags";
      const bool is_diagl = qmapname == "diagl";
      assertx(is_warp || is_diags || is_diagl);
      // First assign corners.
      for_int(ii, 2) for_int(jj, 2) {
        const int i = ii * gridn;
        const int j = jj * gridn;
        Vertex v = verts[i][j];
        const UV uv(float(j) / gridn, float(i) / gridn);
        const Point p = normalized(bilerp(po, uv[0], uv[1]));  // Corners.
        mesh.set_point(v, p);
      }
      // Perform recursive quad subdivision.
      for (int s = gridn / 2; s >= 1; s /= 2) {
        for (int i = 0; i <= gridn; i += s * 2) {
          for (int j = s; j < gridn; j += s * 2) {
            const Point p = normalized(interp(mesh.point(verts[i][j - s]), mesh.point(verts[i][j + s])));
            mesh.set_point(verts[i][j], p);
          }
        }
        for (int j = 0; j <= gridn; j += s * 2) {
          for (int i = s; i < gridn; i += s * 2) {
            const Point p = normalized(interp(mesh.point(verts[i - s][j]), mesh.point(verts[i + s][j])));
            mesh.set_point(verts[i][j], p);
          }
        }
        for (int i = s; i < gridn; i += s * 2) {
          for (int j = s; j < gridn; j += s * 2) {
            const Point& p00 = mesh.point(verts[i - s][j - s]);
            const Point& p01 = mesh.point(verts[i - s][j + s]);
            const Point& p10 = mesh.point(verts[i + s][j - s]);
            const Point& p11 = mesh.point(verts[i + s][j + s]);
            Point p;
            if (0) {
            } else if (is_warp) {  // Centroid of 4 points.
              p = interp(interp(p00, p01), interp(p10, p11));
            } else if (is_diags) {  // Midpoint on shorter diagonal.
              p = dist2(p00, p11) < dist2(p01, p10) ? interp(p00, p11) : interp(p10, p01);
            } else if (is_diagl) {  // Midpoint on longer diagonal.
              p = dist2(p00, p11) > dist2(p01, p10) ? interp(p00, p11) : interp(p10, p01);
            } else {
              assertnever("");
            }
            p = normalized(p);
            mesh.set_point(verts[i][j], p);
          }
        }
      }

    } else {
      // Compute one or more spherical triangle map.
      if (!quad_domain) {
        assertx(gridn >= 1);
      } else {
        assertx(gridn >= 2 && (gridn / 2) * 2 == gridn);
      }
      for_int(i, gridn + 1) for_int(j, !quad_domain ? gridn - i + 1 : gridn + 1) {
        Vec<Point, 3> pt;  // Desired spherical triangle vertices.
        Bary bary;         // Barycentric coordinates within pt.
        const float fi = float(i) / gridn, fj = float(j) / gridn;
        if (!quad_domain) {
          for_int(c, 3) pt[c] = po[c];
          bary = Bary(max(0.f, 1.f - fi - fj), fi, fj);
        } else if (split_quad_ntris == 2) {
          split_quad_2tris(po, fi, fj, pt, bary);
        } else if (split_quad_ntris == 4) {
          split_quad_4tris(po, fi, fj, pt, bary);
        } else if (split_quad_ntris == 8) {
          split_quad_8tris(po, fi, fj, pt, bary, trimap);
        } else {
          assertnever("");
        }
        for_int(c, 3) ASSERTX(is_unit(pt[c]));
        Point p;
        if (domain_interp) {
          p = interp(pt[0], pt[1], pt[2], bary);
        } else {
          p = trimap(pt, bary);
          assertx(is_unit(p));
        }
        mesh.set_point(verts[i][j], p);
      }
    }

    if (omit_faces) {
      // Do not create any mesh faces.
    } else if (!quad_domain) {
      for_int(i, gridn) for_int(j, gridn - i) {
        if (1) {
          Face f = mesh.create_face(verts[i + 0][j + 0], verts[i + 1][j + 0], verts[i + 0][j + 1]);
          f_domainf(f) = domainf;
        }
        if (i) {
          Face f = mesh.create_face(verts[i + 0][j + 0], verts[i + 0][j + 1], verts[i - 1][j + 1]);
          f_domainf(f) = domainf;
        }
      }
    } else {
      for_int(i, gridn) for_int(j, gridn) {
        const Vec<Vertex, 4> va{verts[i + 0][j + 0], verts[i + 0][j + 1], verts[i + 1][j + 1], verts[i + 1][j + 0]};
        Face f;
        if (!b_triangulate) {
          f = mesh.create_face(va);
          f_domainf(f) = domainf;
        } else {
          const int k = ((i >= gridn / 2) ^ (j >= gridn / 2));
          f = mesh.create_face(va[(k + 0) % 4], va[(k + 1) % 4], va[(k + 2) % 4]);
          f_domainf(f) = domainf;
          f = mesh.create_face(va[(k + 0) % 4], va[(k + 2) % 4], va[(k + 3) % 4]);
          f_domainf(f) = domainf;
        }
      }
    }
  }
}

int adjust_for_domain(int en) {
  int n = 0;
  if (0) {
  } else if (domain == "tetra") {
    n = int(sqrt(square(en + 1) / 2.f));
  } else if (domain == "octa" || domain == "octaflat") {
    n = en / 2;
  } else if (domain == "cube") {
    n = int(sqrt(square(en) / 6.f));
    if (n % 2 == 1) n++;  // Make an even number on cube.
  } else {
    assertnever("domain '" + domain + "' not recognized");
  }
  return assertx(n);
}

void do_egridn(Args& args) {
  const int egridn = args.get_int();
  assertx(egridn > 0);
  assertx(!gridn);
  gridn = adjust_for_domain(egridn);
  showdf("Assigning gridn=%d\n", gridn);
}

void do_echeckern(Args& args) {
  const int echeckern = args.get_int();
  assertx(echeckern > 0);
  assertx(!checkern);
  checkern = adjust_for_domain(echeckern);
  showdf("Assigning checkern=%d\n", checkern);
}

void do_mesh_sphere() {
  if (!mesh.num_vertices()) create(true);
  is_remeshed = true;
}

// A remesh is a mesh with vertices in raster-scan order within each domain face.
// The vertex positions are on a resampled surface.
void do_load_remesh(Args& args) {
  assertx(!mesh.num_vertices());
  assertx(!gridn);
  GMesh remesh;
  {
    HH_TIMER("_read_mesh");
    remesh.read(RFile(args.get_filename())());
    assertx(remesh.num_faces());
    showdf("domain -> surface remesh: nv=%d nf=%d\n", remesh.num_vertices(), remesh.num_faces());
  }
  // Set gridn.
  if (0) {
  } else if (domain == "tetra") {
    // Number of vertices is 4 * (n + 1) * (n + 2) / 2 == 2 * n ^ 2 + 6 * n + 4.
    gridn = int(sqrt(remesh.num_vertices() / 2.f)) - 1;
    assertx(2 * square(gridn) + 6 * gridn + 4 == remesh.num_vertices());
  } else if (domain == "octa" || domain == "octaflat") {
    // Number of vertices is 8 * (n + 1) * (n + 2) / 2 == 4 * n ^ 2 + 12 * n + 8.
    gridn = int(sqrt(remesh.num_vertices() / 4.f)) - 1;
    assertx(4 * square(gridn) + 12 * gridn + 8 == remesh.num_vertices());
  } else if (domain == "cube") {
    // Number of vertices is 6 * square(n + 1).
    gridn = int(sqrt(remesh.num_vertices() / 6.f) + .01f) - 1;
    assertx(6 * square(gridn + 1) == remesh.num_vertices());
  } else {
    assertnever("domain '" + domain + "' not recognized");
  }
  assertx(scheme == "");  // Not modified on command line.
  scheme = "domain";      // Any scheme will do.
  create(false);
  assertx(mesh.num_vertices() == remesh.num_vertices());
  Array<char> key, val;
  for (Vertex vp : remesh.vertices()) {
    Vertex v = assertx(mesh.id_retrieve_vertex(remesh.vertex_id(vp)));
    mesh.set_point(v, remesh.point(vp));
    // Copy normal, rgb, etc.
    for_cstring_key_value(remesh.get_string(vp), key, val, [&] { mesh.update_string(v, key.data(), val.data()); });
  }
  is_remeshed = true;
}

int get_gridn(int numverts) {
  int n = 0;
  if (0) {
  } else if (domain == "tetra") {
    // Number of vertices is 2 * square(n) + 2.
    n = int(sqrt(numverts / 2.f));
    assertx(2 * square(n) + 2 == numverts);
  } else if (domain == "octa" || domain == "octaflat") {
    // Number of vertices is 4 * square(n) + 2.
    n = int(sqrt(numverts / 4.f));
    assertx(4 * square(n) + 2 == numverts);
  } else if (domain == "cube") {
    // Number of vertices is 6 * square(n) + 2.
    n = int(sqrt(numverts / 6.f));
    assertx(6 * square(n) + 2 == numverts);
  } else {
    assertnever("domain '" + domain + "' not recognized");
  }
  return n;
}

void verify_good_sphparam(const GMesh& mesh2) {
  for (Vertex v : mesh2.vertices()) assertx(is_unit(v_sph(v)));
  for (Face f : mesh2.faces()) {
    const Vec3<Point> sphs = map(mesh2.triangle_vertices(f), [&](Vertex v) { return v_sph(v); });
    const float sarea = spherical_triangle_area(sphs);
    HH_SSTAT(Ssarea, sarea);
    if (!(sarea < TAU)) SHOW(sarea, sphs);
    assertw(sarea < TAU);                    // Else it is flipped.
    if (verbose >= 2) assertw(sarea > 0.f);  // Else it is degenerate.
    if (0 && !(sarea > 0.f)) SHOW(sarea, sphs);
  }
}

GMesh read_param_mesh(const string& filename) {
  HH_TIMER("_read_param_mesh");
  assertx(filename != "");
  RFile fi(filename);
  for (string line; fi().peek() == '#';) {
    assertx(my_getline(fi(), line));
    if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
  }
  GMesh param_mesh;
  param_mesh.read(fi());
  assertx(param_mesh.num_faces());
  showdf("surface -> sphere map: nv=%d nf=%d\n", param_mesh.num_vertices(), param_mesh.num_faces());
  const bool normsph = getenv_bool("SPHERESAMPLE_NORMSPH");
  for (Vertex v : param_mesh.vertices()) {
    Point p;  // On sphere.
    assertx(parse_key_vec(param_mesh.get_string(v), "sph", p));
    if (!normsph)
      assertx(is_unit(p));
    else
      p = normalized(p);
    v_sph(v) = p;
  }
  verify_good_sphparam(param_mesh);
  return param_mesh;
}

// A map is a gmerged mesh with vertex positions on domain, and sph strings for parameterization on sphere.
void do_load_map(Args& args) {
  assertx(!gridn);
  assertx(!mesh.num_vertices());
  const GMesh param_mesh = read_param_mesh(args.get_filename());
  gridn = get_gridn(param_mesh.num_vertices());
  assertx(scheme == "");  // Not modified on command line.
  scheme = "domain";      // Any scheme will do.
  create(false);
  for (Vertex v : mesh.vertices()) mesh.set_point(v, Point(BIGFLOAT, 0.f, 0.f));
  Array<Point> arp;
  arp.reserve(mesh.num_vertices());  // Necessary to prevent reallocation.
  PointSpatial<Vertex> psp(max(10, gridn));
  for (Vertex v : mesh.vertices()) {
    arp.push(v_domainp(v) * k_fdomain_to_spatialbb);
    psp.enter(v, &arp.last());
  }
  for (Vertex param_v : param_mesh.vertices()) {
    const Point& sphp = v_sph(param_v);
    const Point& domainp = param_mesh.point(param_v);
    int nfaces = 0;
    SpatialSearch<Vertex> ss(&psp, domainp * k_fdomain_to_spatialbb);
    for (;;) {
      float dis2;
      Vertex vv = ss.next(&dis2);
      if (dis2 > 1e-12f) break;
      nfaces++;
      assertx(mesh.point(vv)[0] == BIGFLOAT);
      mesh.set_point(vv, sphp);
    }
    if (nfaces < 1 || nfaces > 4) SHOW(nfaces, domainp, sphp);
    assertx(nfaces >= 1 && nfaces <= 4);  // Max valence is 3 or 4.
  }
  for (Vertex v : mesh.vertices()) assertx(mesh.point(v)[0] != BIGFLOAT);
}

void do_sample_map(Args& args) {
  assertx(gridn);
  assertx(!mesh.num_vertices());
  const GMesh param_mesh = read_param_mesh(args.get_filename());
  const MeshSearch msearch(param_mesh, {true});
  if (scheme == "") scheme = "domain";
  create(scheme != "domain");
  {
    HH_TIMER("_resample0");
    ConsoleProgress cprogress;
    int nv = 0;
    Face hintf = nullptr;
    for (Vertex v : mesh.ordered_vertices()) {
      if ((nv++ & 0xff) == 0) cprogress.update(float(nv) / mesh.num_vertices());
      auto [param_f, bary, unused_clp, d2] = msearch.search(v_domainp(v), hintf);
      hintf = param_f;
      assertx(d2 < 1e-12f);
      Vector sum{};
      const Vec3<Vertex> param_face_vertices = param_mesh.triangle_vertices(param_f);
      for_int(i, 3) sum += bary[i] * v_sph(param_face_vertices[i]);
      mesh.set_point(v, normalized(sum));
    }
  }
}

Bary get_bary_sub(const Point& p, const Vec<Point, 3>& pt, Trispheremap trimap) {
  assertx(trimap == &map_sphere);
  // Problem is that most maps do not have the property that pn[] are on spherical arcs of pc[]!
  ASSERTX(is_unit(p));
  for_int(i, 3) ASSERTX(is_unit(pt[i]));
  Vec<Bary, 3> bc;
  Vec<Point, 3> pc;
  for_int(i, 3) {
    bc[i] = Bary(i == 0, i == 1, i == 2);
    pc[i] = pt[i];
  }
  for_int(iter, 100) {
    const float eps = 2e-7f;
    if (dist2(pc[0], pc[1]) < square(eps) && dist2(pc[0], pc[2]) < square(eps) && dist2(pc[1], pc[2]) < square(eps))
      return bc[0];
    Vec<Bary, 3> bn;
    Vec<Point, 3> pn;
    for_int(i, 3) {
      bn[i] = interp(bc[mod3(i + 1)], bc[mod3(i + 2)]);
      pn[i] = trimap(pt, bn[i]);
    }
    for_int(i, 3) {
      const Vec<Point, 3> pp{pc[i], pn[mod3(i + 2)], pn[mod3(i + 1)]};
      if (in_spheretri(p, pp)) {
        bc = {bc[i], bn[mod3(i + 2)], bn[mod3(i + 1)]};
        pc = {pc[i], pn[mod3(i + 2)], pn[mod3(i + 1)]};
        break;
      } else if (i == 2) {
        if (!in_spheretri(p, pn)) SHOW(pt, pc, pn, bc, dist2(pc[0], pc[1]), dist2(pc[0], pc[2]), dist2(pc[1], pc[2]));
        assertx(in_spheretri(p, pn));
        bc = bn;
        pc = pn;
      }
    }
  }
  assertnever("no convergence");
}

Bary get_bary(const Point& p, const Vec<Point, 3>& pt, Trispheremap trimap) {
  ASSERTX(in_spheretri(p, pt));
  // Hard-code the inverse-map for map_sphere.
  assertx(trimap == &map_sphere);
  // Spherically project the point onto the triangle spanning pt[3],
  //  i.e. intersect the segment (origin, p) with the triangle.
  Polygon poly(3);
  for_int(i, 3) poly[i] = pt[i];
  const Point origin(0.f, 0.f, 0.f);
  Point pint;
  if (!poly.intersect_segment(origin, p, pint)) {
    // Warning("widening polygon");
    if (0) SHOW(poly, spherical_triangle_area(poly));
    widen_triangle(poly, 1e-4f);
    if (0) SHOW(poly);
    if (!poly.intersect_segment(origin, p, pint)) {
      // Warning("intersect_segment failed again");
      // Well, resort to closest point if necessary.
      Bary bary;
      project_point_triangle2(p, pt[0], pt[1], pt[2], bary, pint);
    }
  }
  // Compute bary for the intersection point pint.
  Bary bary;
  Point clp;
  const float d2 = project_point_triangle2(pint, pt[0], pt[1], pt[2], bary, clp);
  assertw(d2 < 1e-10f);
  return bary;
}

// Given point p on sphere, and some nearby spherical triangle f in mesh2, find the true spherical
// triangle f containing p, and the barycentric coordinates of p in f.
void search_bary(const Point& p, const GMesh& mesh2, Face& f, Bary& bary) {
  const Trispheremap trimap = &map_sphere;
  Vec3<Point> points;
  {
    // Find the spherical triangle f containing p.
    int nfchanges = 0;
    for (;;) {
      points = mesh2.triangle_points(f);
      // Adapted from MeshSearch.cpp .
      const float dotcross_eps = 2e-7f;
      Vec<bool, 3> outside;
      for_int(i, 3) outside[i] = dot(Vector(p), cross(p, points[mod3(i + 1)], points[mod3(i + 2)])) < -dotcross_eps;
      int noutside = sum<int>(outside);
      if (noutside == 0) break;
      const Vec<Vertex, 3> va = mesh2.triangle_vertices(f);
      if (noutside == 2) {
        const int side = index(outside, false);
        // Fastest: jump across the vertex.
        Vertex v = va[side];
        const int val = mesh2.degree(v);
        // const int nrot = ((val - 1) / 2) + (Random::G.unif() < 0.5f);  // Ideal, but Random is not thread-safe.
        constexpr auto pseudo_randoms = V(0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1);
        const int nrot = ((val - 1) / 2) + pseudo_randoms[nfchanges % pseudo_randoms.Num];
        for_int(i, nrot) f = assertx(mesh2.ccw_face(v, f));
      } else if (noutside == 1) {
        const int side = index(outside, true);
        f = assertx(mesh2.opp_face(va[side], f));
      } else {
        assertnever("");
      }
      nfchanges++;
      assertx(nfchanges < 200);
    }
    HH_SSTAT(Snfchanges, nfchanges);
  }
  if (1) {
    bary = get_bary(p, points, trimap);
  } else {
    bary = get_bary_sub(p, points, trimap);
  }
}

// Returns k_undefined_vector if undefined.
Vector interp_f_normal(const GMesh& mesh2, Face f, const Bary& bary) {
  const Vec3<Corner> corners = mesh2.triangle_corners(f);
  Vector sum_normal{};
  int num_defined = 0;
  for_int(i, 3) {
    Corner c = corners[i];
    Vector normal;
    if (GMesh::string_has_key(mesh2.get_string(c), "normal")) {
      assertx(mesh2.parse_corner_key_vec(c, "normal", normal));
    } else if (Vertex v = mesh2.corner_vertex(c); v_normal(v) != k_undefined_vector) {
      normal = v_normal(v);
    } else {
      continue;
    }
    num_defined++;
    sum_normal += normal * bary[i];
  }
  if (num_defined == 0) return k_undefined_vector;
  assertx(num_defined == 3);
  return normalized(sum_normal);
}

// Returns k_undefined_vector if undefined.
Vector interp_f_rgb(const GMesh& mesh2, Face f, const Bary& bary) {
  const Vec3<Corner> corners = mesh2.triangle_corners(f);
  Vector sum_rgb{};
  int num_defined = 0;
  for_int(i, 3) {
    Corner c = corners[i];
    Vector rgb;
    if (GMesh::string_has_key(mesh2.get_string(c), "rgb")) {
      assertx(mesh2.parse_corner_key_vec(c, "rgb", rgb));
    } else if (Vertex v = mesh2.corner_vertex(c); v_rgb(v) != k_undefined_vector) {
      rgb = v_rgb(v);
    } else {
      continue;
    }
    num_defined++;
    sum_rgb += rgb * bary[i];
  }
  if (num_defined == 0) return k_undefined_vector;
  assertx(num_defined == 3);
  return sum_rgb;
}

// *** Inverse map.

void internal_remesh() {
  if (is_remeshed) return;
  assertx(param_file != "");
  GMesh param_mesh = read_param_mesh(param_file);
  // This mesh has vertex positions on original model surface, and sph strings for parameterization on sphere.
  for (Vertex param_v : param_mesh.vertices()) {
    v_domainp(param_v) = param_mesh.point(param_v);
    param_mesh.set_point(param_v, v_sph(param_v));
    Vector normal;
    if (!parse_key_vec(param_mesh.get_string(param_v), "normal", normal)) normal = k_undefined_vector;
    v_normal(param_v) = normal;
    Vector rgb;
    if (!parse_key_vec(param_mesh.get_string(param_v), "rgb", rgb)) rgb = k_undefined_vector;
    v_rgb(param_v) = rgb;
  }
  // int spgrid = max({10, int(sqrt(param_mesh.num_faces() * .05f)), gridn / 2});
  MeshSearch::Options options;
  options.allow_local_project = true;
  options.allow_off_surface = true;
  const MeshSearch msearch(param_mesh, options);
  if (!mesh.num_vertices()) {
    // Create domain grid mesh.
    HH_STIMER("_create_tess");
    create(false);
  }
  {
    HH_TIMER("_resample");
    // ConsoleProgress cprogress;
    // int nv = 0;
    // HashFloat hashf1;  // For -slowcornermerge of sph.
    // HashFloat hashf2;  // For -slowcornermerge of normal.
    const int num_threads = get_max_threads();
    parallel_for_chunk(Array<Vertex>(mesh.vertices()), num_threads, [&](const int thread_index, auto subrange) {
      dummy_use(thread_index);
      string str;
      Face hintf = nullptr;
      for (Vertex v : subrange) {
        // if ((nv++ & 0xff) == 0) cprogress.update(float(nv) / mesh.num_vertices());
        Point p = mesh.point(v);  // Point on sphere.
        assertx(is_unit(p));
        auto [param_f, bary, unused_clp, unused_d2] = msearch.search(p, hintf);
        search_bary(p, param_mesh, param_f, bary);  // May modify param_f.
        hintf = param_f;
        const Vec3<Point> points = map(mesh.triangle_vertices(param_f), [&](Vertex v) { return v_domainp(v); });
        const Point newp = interp(points[0], points[1], points[2], bary);
        mesh.set_point(v, newp);
        // for_int(i, 3) p[i] = hashf1.enter(p[i]);
        v_sph(v) = p;
        v_normal(v) = interp_f_normal(param_mesh, param_f, bary);
        if (checkern) {
          // Ignore mesh color since checkering.
        } else {
          Vector rgb = interp_f_rgb(param_mesh, param_f, bary);
          if (rgb == k_undefined_vector)
            if (Vector rgb2; parse_key_vec(param_mesh.get_string(param_f), "rgb", rgb2)) rgb = rgb2;
          v_rgb(v) = rgb;
        }
        // Before: { uv=stretchuv  imageuv }.
        // After:  { uv=imageuv  stretchuv }.
        // mesh.update_string(v, "stretchuv", assertx(GMesh::string_key(str, mesh.get_string(v), "uv")));
        // mesh.update_string(v, "uv", assertx(GMesh::string_key(str, mesh.get_string(v), "imageuv")));
        // mesh.update_string(v, "imageuv", nullptr);
      }
    });
  }
  is_remeshed = true;
}

void triangulate_short_diag() {
  HH_STIMER("_triangulate_short");
  Array<Vertex> va;
  for (Face f : Array<Face>(mesh.faces())) {
    mesh.get_vertices(f, va);
    if (va.num() == 3) continue;
    assertx(va.num() == 4);
    Vertex va0 = va[0], va2 = va[2];
    const bool other_diag = dist2(mesh.point(va[1]), mesh.point(va[3])) < dist2(mesh.point(va[0]), mesh.point(va[2]));
    if (other_diag) {
      va0 = va[1];
      va2 = va[3];
    }
    const int domainf = f_domainf(f);
    Edge e = mesh.split_face(f, va0, va2);
    for (Face ff : mesh.faces(e)) f_domainf(ff) = domainf;
  }
}

void do_keys(Args& args) {
  const string keys = args.get_string();
  const std::regex pattern(",");  // Pattern for splitting keys.
  key_names =
      Array<string>(std::sregex_token_iterator(keys.begin(), keys.end(), pattern, -1), std::sregex_token_iterator());
  if (key_names.num() == 1 && key_names[0] == "") key_names.clear();
  const std::regex alphanumeric("^[a-zA-Z]+$");
  const auto k_valid_keys = V<string>("domainp", "stretchuv", "imageuv", "sph", "ll", "domaincorner", "domainf");
  for (const string& key : key_names) {
    if (!std::regex_match(key, alphanumeric) || !contains(k_valid_keys, key))
      assertnever("Error parsing: '" + keys + "': not comma-joined subset of " + make_string(k_valid_keys));
  }
}

void do_remesh() {
  internal_remesh();
  if (domain == "cube") triangulate_short_diag();
}

void do_signal(Args& args) {
  signal_ = args.get_string();
  assertx(contains(V<string>("G", "N", "C"), signal_));
}

void do_write_texture(Args& args) {
  const string imagename = args.get_filename();
  assertx(signal_ != "");
  internal_remesh();
  // Note that the samples from the remesh match 1-to-1 with the geometric vertices, which does NOT correspond
  //  to current texture-mapping hardware.
  // To do things correctly, we would have to offset the imageuv coords taking into account the resolution of
  //  the normal-map.
  // However, this would NOT work for coarse normal-maps since the samples between adjacent domain faces
  //  must be shared.
  HH_TIMER("_write_texture");
  assertx(gridn);
  // For tetra and cube, the righthand boundary of the image correctly wraps around to the lefthand boundary,
  //  and the top boundary of the domain is still in the interior.
  // For octa, we need the extra border of samples on all sides.
  const int imagesize = (domain == "tetra"                          ? gridn * 2
                         : domain == "octa" || domain == "octaflat" ? gridn * 2 + 1
                         : domain == "cube"                         ? gridn * 4
                                            : (assertnever("domain '" + domain + "' not recognized"), 0));
  const int scale = (domain == "tetra"                          ? gridn * 2
                     : domain == "octa" || domain == "octaflat" ? gridn * 2
                     : domain == "cube"                         ? gridn * 4
                                        : (assertnever("domain '" + domain + "' not recognized"), 0));
  const Pixel background = Pixel::white();  // Outside color.
  Image image(V(imagesize, imagesize), background);
  Image image_right_column(V(image.ysize(), 1), background);
  const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
  const Frame rotate_frame = get_rotate_frame();
  if (signal_ == "N") assertw(rotate_s3d != "");
  for (Vertex v : mesh.ordered_vertices()) {  // Ordered so as to "consistently" break ties along domain edges.
    const UV uv = v_imageuv(v);
    int y = int(uv[1] * scale + .5f);
    const int x = int(uv[0] * scale + .5f);
    // We flip the image vertically because the OpenGL UV coordinate origin is at the image lower-left.
    y = image.ysize() - 1 - y;
    Pixel pixel;
    pixel[3] = 255;
    switch (signal_[0]) {
      case 'G': {
        const Point& p = mesh.point(v);
        for_int(z, 3) {
          assertx(p[z] >= bbox[0][z] && p[z] <= bbox[1][z]);
          const float f = (p[z] - bbox[0][z]) / (bbox[1][z] - bbox[0][z]);
          pixel[z] = uint8_t(f * 255.f + .5f);
        }
        break;
      }
      case 'N': {
        const Vector normal = v_normal(v) * rotate_frame;
        for_int(z, 3) {
          assertx(abs(normal[z]) <= 1.f + 1e-6f);
          pixel[z] = uint8_t((normal[z] * .5f + .5f) * 255.f + .5f);
        }
        break;
      }
      case 'C': {
        const Vector& rgb = v_rgb(v);
        for_int(z, 3) {
          assertx(rgb[z] >= 0.f && rgb[z] <= 1.f + 1e-6f);
          pixel[z] = uint8_t(rgb[z] * 255.f + .5f);
        }
        break;
      }
      default: assertnever("signal '" + signal_ + "' not recognized");
    }
    if (x == image.xsize()) {
      // Ignore last column; verify later that it equals first column.
      image_right_column[y][0] = pixel;
    } else {
      image[y][x] = pixel;
    }
  }
  if (!checkern) {
    for_int(y, image.ysize()) {
      const Pixel& pixel = image_right_column[y][0];
      if (pixel != background) assertw(pixel == image[y][0]);
    }
  }
  image.write_file(imagename);
  nooutput = true;
}

void do_write_lonlat_texture(Args& args) {
  const string imagename = args.get_filename();
  assertx(gridn);
  assertx(param_file != "");
  assertx(signal_ != "");
  assertx(!mesh.num_vertices());
  const Frame rotate_frame = get_rotate_frame();
  mesh = read_param_mesh(param_file);
  for (Vertex v : mesh.vertices()) {
    v_domainp(v) = mesh.point(v);
    mesh.set_point(v, v_sph(v));
  }
  const Bbox bbox{transform(mesh.vertices(), [&](Vertex v) { return mesh.point(v); })};
  switch (signal_[0]) {
    case 'G': break;
    case 'N':
      assertw(rotate_s3d != "");
      for (Vertex v : mesh.vertices()) {
        Vector normal;
        if (!parse_key_vec(mesh.get_string(v), "normal", normal)) normal = k_undefined_vector;
        v_normal(v) = normal;
      }
      break;
    case 'C':
      for (Vertex v : mesh.vertices()) {
        Vector rgb;
        if (!parse_key_vec(mesh.get_string(v), "rgb", rgb)) rgb = k_undefined_vector;
        v_rgb(v) = rgb;
      }
      break;
    default: assertnever("signal '" + signal_ + "' not recognized");
  }

  const MeshSearch msearch(mesh, {true});

  const int imagesize = gridn;
  Image image(V(imagesize, imagesize));

  const int num_threads = get_max_threads();
  parallel_for_chunk(range(image.ysize()), num_threads, [&](const int thread_index, auto subrange) {
    dummy_use(thread_index);
    Face hintf = nullptr;
    for (const int y : subrange) {
      for_int(x, image.xsize()) {
        // We flip the image vertically because the OpenGL UV coordinate origin is at the image lower-left.
        const int yy = image.ysize() - 1 - y;
        Pixel& pixel = image[yy][x];
        const UV lonlat((x + .5f) / image.xsize(), (y + .5f) / image.ysize());
        const Point sph = sph_from_lonlat(lonlat);

        auto [f, bary, unused_clp, unused_d2] = msearch.search(sph, hintf);
        hintf = f;
        search_bary(sph, mesh, f, bary);  // May modify f.
        switch (signal_[0]) {
          case 'G': {
            const Vec3<Vertex> face_vertices = mesh.triangle_vertices(f);
            Point p{};
            for_int(i, 3) p += bary[i] * v_domainp(face_vertices[i]);
            for_int(z, 3) {
              assertx(p[z] >= bbox[0][z] && p[z] <= bbox[1][z]);
              const float frac = (p[z] - bbox[0][z]) / (bbox[1][z] - bbox[0][z]);
              pixel[z] = uint8_t(frac * 255.f + .5f);
            }
            break;
          }
          case 'N': {
            Vector normal = interp_f_normal(mesh, f, bary);
            normal *= rotate_frame;
            for_int(z, 3) {
              assertx(abs(normal[z]) <= 1.f + 1e-6f);
              pixel[z] = uint8_t((normal[z] * .5f + .5f) * 255.f + .5f);
            }
            break;
          }
          case 'C': {
            Vector rgb = interp_f_rgb(mesh, f, bary);
            for_int(z, 3) {
              assertx(rgb[z] >= 0.f && rgb[z] <= 1.f + 1e-6f);
              pixel[z] = uint8_t(rgb[z] * 255.f + .5f);
            }
            break;
          }
          default: assertnever("signal '" + signal_ + "' not recognized");
        }
      }
    }
  });

  image.write_file(imagename);
  nooutput = true;
}

// Tessellate a single spherical triangle.
void generate_tess(const Vec<Point, 3>& pa, int n, Trispheremap trimap) {
  for_int(i, 3) assertx(is_unit(pa[i]));
  Matrix<Vertex> verts(n + 1, n + 1);  // Upper-left half is defined.
  for_int(i, n + 1) for_int(j, n - i + 1) {
    Vertex v = mesh.create_vertex();
    v_normal(v) = k_undefined_vector;
    v_rgb(v) = k_undefined_vector;
    verts[i][j] = v;
    v_imageuv(v) = UV(i / float(n) + j / float(n) * .5f, j / float(n) * sqrt(3.f) / 2.f);
    const Bary bary((n - i - j) / float(n), i / float(n), j / float(n));
    const Point p = trimap(pa, bary);
    assertx(is_unit(p));
    mesh.set_point(v, p);
    v_sph(v) = p;
  }
  for_int(i, n) for_int(j, n - i) {
    if (1) mesh.create_face(verts[i + 0][j + 0], verts[i + 1][j + 0], verts[i + 0][j + 1]);
    if (i) mesh.create_face(verts[i + 0][j + 0], verts[i + 0][j + 1], verts[i - 1][j + 1]);
  }
}

void do_triface1() {
  assertx(gridn > 0);
  assertx(scheme[0] == 'T');
  const Trispheremap trimap = get_map(scheme.substr(1));
  const Point refp1(0.f, 1.f, 0.f), refp2(-1.f, 0.f, 0.f), refp3 = normalized(Point(-.2f, -.8f, .4f));
  generate_tess(V(refp1, refp2, refp3), gridn, trimap);
}

void do_test_properties() {
  assertx(scheme[0] == 'T');
  const Trispheremap trimap = get_map(scheme.substr(1));
  if (!gridn) gridn = 16;
  //
  const Point refp1(0.f, 1.f, 0.f), refp2(-1.f, 0.f, 0.f);
  const Point refp3 = normalized(Point(-.3f, 0.f, .9f)), refp4 = normalized(Point(.4f, -.8f, .3f));
  //
  assertx(!mesh.num_vertices());
  const SGrid<Point, 3, 3> faces = {
      {refp1, refp2, refp3},
      {refp3, refp2, refp4},  // Shares the same boundary vertices iff edge-continuous. (check visually).
      {refp2, refp3, refp1}   // This rotated first face is identical iff symmetric. (check visually).
  };
  for_int(domainf, faces.num()) {
    Vec<Point, 3> pt;
    for_int(k, 3) pt[k] = normalized(faces[domainf][k]);
    generate_tess(pt, gridn, trimap);
  }
  {
    Point po;
    dummy_init(po);
    for_int(i, gridn) {
      const Bary bary((gridn - i) / float(gridn), i / float(gridn), 0.f);
      const Point p = trimap(faces[0], bary);
      if (i) HH_SSTAT(Sbndlen, dist(p, po));
      // These lengths are identical iff arc-length boundaries.
      po = p;
    }
  }
  for_int(i, 3) {
    Bary bary{};
    bary[i] = 1.f;
    const Point p0 = trimap(faces[0], bary);
    const float eps = 1e-7f;
    bary[i] -= eps;
    bary[mod3(i + 1)] += eps;
    const Point p1 = trimap(faces[0], bary);
    SHOW(dist(p0, p1));
    // This length will be abnormally large iff unbounded derivatives.
  }
  nooutput = true;
}

void do_create_lonlat_sphere() {
  assertx(!mesh.num_vertices());
  assertx(gridn >= 2);
  Matrix<Vertex> matv(gridn, gridn);
  string str;
  for_int(i, gridn) for_int(j, gridn) {
    Vertex v = mesh.create_vertex();
    matv[i][j] = v;
    // We reverse the index i to obtain the correct outward orientation of the sphere.
    const UV lonlat(j / (gridn - 1.f), (gridn - 1 - i) / (gridn - 1.f));
    const Point sph = sph_from_lonlat(lonlat);
    mesh.set_point(v, sph);
    v_sph(v) = sph;
    v_normal(v) = sph;
    v_rgb(v) = k_undefined_vector;
    mesh.update_string(v, "uv", csform_vec(str, lonlat));
  }
  for_int(i, gridn - 1) for_int(j, gridn - 1) {
    if (0) {
      mesh.create_face(V(matv[i + 0][j + 0], matv[i + 1][j + 0], matv[i + 1][j + 1], matv[i + 0][j + 1]));
    } else {
      mesh.create_face(V(matv[i + 0][j + 0], matv[i + 1][j + 0], matv[i + 1][j + 1]));
      mesh.create_face(V(matv[i + 0][j + 0], matv[i + 1][j + 1], matv[i + 0][j + 1]));
    }
  }
}

// Create a texture image parameterized by (longitude, latitude) that shows the domain grid samples mapped
// onto the sphere.
void do_create_lonlat_checker(Args& args) {
  int imagesize = args.get_int();
  assertx(imagesize > 1);
  string imagename = args.get_filename();
  //
  assertx(checkern);
  if (!gridn) gridn = 128;
  {
    HH_TIMER("_create_tess2");
    if (!mesh.num_vertices()) {
      create(true);
    } else {
      if (domain == "cube") triangulate_short_diag();
    }
  }
  MeshSearch::Options options;
  options.allow_off_surface = true;
  const MeshSearch msearch(mesh, options);
  const Array<DomainFace> domain_faces = get_domain_faces();
  {
    HH_TIMER("_create_image");
    Image image(V(imagesize, imagesize));
    const int num_threads = get_max_threads();
    parallel_for_chunk(range(image.ysize()), num_threads, [&](const int thread_index, auto subrange) {
      dummy_use(thread_index);
      Face hintf = nullptr;
      for (const int y : subrange) {
        for_int(x, image.xsize()) {
          // We flip the image vertically because the OpenGL UV coordinate origin is at the image lower-left.
          const int yy = image.ysize() - 1 - y;
          Pixel& pixel = image[yy][x];
          const UV lonlat((x + .5f) / image.xsize(), (y + .5f) / image.ysize());
          const Point sph = sph_from_lonlat(lonlat);
          auto [f, bary, unused_clp, unused_d2] = msearch.search(sph, hintf);
          // Given that `mesh` has disjoint components, this call to search_bary() ought to fail sometimes?
          search_bary(sph, mesh, f, bary);  // May modify f.
          hintf = f;
          const Vec3<Vertex> va = mesh.triangle_vertices(f);
          UV uv{};
          for_int(i, 3) uv += bary[i] * v_ijuv(va[i]);
          // Mesh vertices have colors according to checkern; for precision, we ignore these and recompute using uv.
          const int domainf = f_domainf(f);
          pixel = get_color(domain_faces[domainf], uv);
        }
      }
    });
    image.write_file(imagename);
  }
  nooutput = true;
}

void add_mesh_strings() {
  // Use HashFloat on sph and normal?
  string str;
  if (contains(key_names, "domainp"))
    for (Vertex v : mesh.vertices()) mesh.update_string(v, "domainp", csform_vec(str, v_domainp(v)));
  if (contains(key_names, "stretchuv"))
    for (Vertex v : mesh.vertices()) mesh.update_string(v, "stretchuv", csform_vec(str, v_stretchuv(v)));
  if (contains(key_names, "imageuv"))
    for (Vertex v : mesh.vertices()) mesh.update_string(v, "imageuv", csform_vec(str, v_imageuv(v)));
  if (contains(key_names, "sph"))
    for (Vertex v : mesh.vertices()) mesh.update_string(v, "sph", csform_vec(str, v_sph(v)));
  if (contains(key_names, "ll"))
    for (Vertex v : mesh.vertices()) {
      const Point& sph = v_sph(v);
      UV lonlat = lonlat_from_sph(sph);
      const bool near_prime_meridian = abs(sph[0]) < 1e-5f && sph[1] > 1e-5f;
      if (near_prime_meridian) {
        Face f = mesh.most_ccw_face(v);  // Actually, any adjacent face in this connected component.
        const Vec3<Point> sphs = map(mesh.triangle_vertices(f), [&](Vertex v2) { return v_sph(v2); });
        const Point center = mean(sphs);
        lonlat[0] = center[0] < 0.f ? 0.f : 1.f;
      }
      mesh.update_string(v, "ll", csform_vec(str, lonlat));
    }
  if (1) {
    for (Vertex v : mesh.vertices()) {
      if (v_normal(v) != k_undefined_vector) mesh.update_string(v, "normal", csform_vec(str, v_normal(v)));
      if (v_rgb(v) != k_undefined_vector) mesh.update_string(v, "rgb", csform_vec(str, v_rgb(v)));
    }
  }
  // Note: "domaincorner" was handled earlier.
  if (contains(key_names, "domainf"))
    for (Face f : mesh.faces()) mesh.update_string(f, "domainf", csform(str, "%d", f_domainf(f)));
}

}  // namespace

// *** main

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSP(domain, "name : select {tetra, octa, cube, octaflat}");
  HH_ARGSP(scheme, "name : select procedural mapping {best, Q*, T*}");
  HH_ARGSP(gridn, "n : domain face tessellation grid resolution");
  HH_ARGSD(egridn, "n : effective single-gim resolution, domain-adjusted");
  HH_ARGSP(checkern, "n : create domain grid of vertex colors");
  HH_ARGSD(echeckern, "n : same, adjusting n for domain type");
  HH_ARGSF(octa8colors, ": use 8 instead of 4 face colors");
  HH_ARGSF(baseball, ": use baseball cube unwrapping instead of cross");
  HH_ARGSC(HH_ARGS_INDENT "Sample sphere:");
  HH_ARGSF(omit_faces, ": let remesh have only vertices (faster write_texture)");
  HH_ARGSD(mesh_sphere, ": map domain grid onto sphere");
  HH_ARGSD(load_remesh, "mesh.remesh.m : load a previous remesh");
  HH_ARGSD(load_map, "domain.invmap.m : domain -> sphere, instead of scheme");
  HH_ARGSD(sample_map, "domain.invmap.m : domain -> sphere, sampled using gridn");
  HH_ARGSP(param_file, "mesh.sphparam.m : specify surface parameterization");
  HH_ARGSP(rotate_s3d, "file.s3d : rotate normals based on view (snapped to axes)");
  HH_ARGSD(keys, "comma_separated_keys : mesh fields: domainp,stretchuv,imageuv,sph,ll,domaincorner,domainf");
  HH_ARGSD(remesh, ": resample mesh and triangulate");
  HH_ARGSD(signal, "l : select G, N, or C");
  HH_ARGSD(write_texture, "image : resample signal as texturemap");
  HH_ARGSD(write_lonlat_texture, "image : resample signal as (lon, lat) texture");
  HH_ARGSC(HH_ARGS_INDENT "Misc:");
  HH_ARGSP(verbose, "level : verbosity level (1=normal)");
  HH_ARGSF(first_domain_face, ": only generate mesh for the first domain face");
  HH_ARGSD(triface1, ": tessellate 1 spherical triangle");
  HH_ARGSD(test_properties, ": examine map properties");
  HH_ARGSD(create_lonlat_sphere, ": create a mesh with lon-lat uv param");
  HH_ARGSD(create_lonlat_checker, "size file : write image with longlat domain checker");
  HH_ARGSF(nooutput, ": do not print mesh at program end");
  {
    showff("%s", args.header().c_str());
    HH_TIMER("SphereSample");
    args.parse();
  }
  hh_clean_up();
  if (!nooutput) {
    add_mesh_strings();
    mesh.write(std::cout);
    std::cout.flush();
  }
  if (!k_debug) exit_immediately(0);
  return 0;
}
