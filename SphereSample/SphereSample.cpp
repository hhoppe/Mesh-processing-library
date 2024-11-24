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
#include "libHh/Multigrid.h"
#include "libHh/Parallel.h"
#include "libHh/Polygon.h"
#include "libHh/Random.h"
#include "libHh/Spatial.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"
#include "libHh/Vector4.h"
using namespace hh;

namespace {

string domain = "octa";  // {tetra, octa, cube, octaflat}.
string scheme = "";      // {best, Qdomain, Qsphere, ... , Tdomain, Tsphere, Tslerps, ...}.
int gridn = 0;
int checkern = 0;
bool octa8colors = false;
bool baseball = false;
bool omit_faces = false;
string domain_file;              // Filename for domain_mesh (D -> S).
string param_file;               // Filename for param_mesh (M -> S and after inversion, S -> M).
string rotate_s3d;               // Filename for s3d file.
Array<string> key_names{"sph"};  // Set of string attributes written to output mesh.
string signal_;                  // Surface signal ("G", "N", "C", "T") (for geometry, normal, color, texture).
bool feather_texture = true;     // Blend texture discontinuities in do_write_texture().
Matrix<Vector4> texture_image_vector4;
int verbose = 1;
bool first_domain_face = false;
bool nooutput = false;

struct DomainFace {
  Polygon poly;          // Points on regular polyhedron in 3D.
  Array<Uv> stretchuvs;  // For stretch optimization domain -> sphere.
  Array<Uv> imageuvs;    // For texture-mapping.
  Pixel facecolor;
  Pixel pixel_checker;
};

GMesh g_mesh;
bool is_remeshed = false;

constexpr Vector k_undefined_vector(-2.f, 0.f, 0.f);
constexpr Uv k_undefined_uv(-2.f, 0.f);

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Point, v_domainp);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Uv, v_stretchuv);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Uv, v_imageuv);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Uv, v_ijuv);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Point, v_sph);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Vector, v_normal);  // May be k_undefined_vector.
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Vector, v_rgb);     // May be k_undefined_vector.
HH_SAC_ALLOCATE_FUNC(Mesh::MFace, int, f_domainf);

const Frame k_fdomain_to_spatialbb{Vector(.4f, .0f, .0f), Vector(.0f, .4f, .0f), Vector(.0f, .0f, .4f),
                                   Point(.5f, .5f, .5f)};
const Pixel k_pixel_gray = Pixel::gray(235);
const Pixel k_pixel_orange{255, 153, 0, 255};

constexpr Pixel k_color_special_filled{0, 0, 0, 0};  // Transparent.

// *** General functions.

inline Point bilerp(const Vec4<Point>& pa, float u, float v) { return bilerp(pa[0], pa[1], pa[2], pa[3], u, v); }

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
  const ObjectFrame object_frame = FrameIO::read(fi()).value();
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
    //
    //   2 - 3 - 2        / 3 \      |
    //   | / | / |       1  |  2     |
    //   1 - 0 - 1        \ 0 /      |
    const Vec4<Point> vtmp = []() {
      Vec4<Point> v{
          Vector(-1.f / sqrt(3.f), 0.f, 0.f),
          Vector(+sqrt(3.f) / 6.f, +.5f, 0.f),
          Vector(+sqrt(3.f) / 6.f, -.5f, 0.f),
          Vector(0.f, 0.f, sqrt(6.f) / 3.f),
      };
      for_int(i, 4) v[i] = normalized(v[i] - v[3] / 4.f);
      return v;
    }();
    domain_faces.init(4);  // Tetrahedral faces are 103, 213, 023, 012.
    for_int(i, domain_faces.num()) {
      domain_faces[i].facecolor = V(Pixel::green(), Pixel::blue(), Pixel::red(), k_pixel_orange)[i];
      if (i < 3) {
        domain_faces[i].poly = V(vtmp[mod3(i + 1)], vtmp[i], vtmp[3]);
      } else {
        domain_faces[i].poly = vtmp.head<3>();
      }
      domain_faces[i].stretchuvs = V(Uv(0.f, 0.f), Uv(1.f, 0.f), Uv(.5f, .5f * sqrt(3.f)));
      // The imageuv's are defined below.
    }
    // 103: face 1 in left-to-right image order.
    domain_faces[0].imageuvs = V(Uv(.00f, .00f), Uv(.50f, .00f), Uv(.50f, .50f));
    // 213: face 0 in left-to-right image order.
    domain_faces[1].imageuvs = V(Uv(.00f, .50f), Uv(.00f, .00f), Uv(.50f, .50f));
    // 023: face 2 in left-to-right image order.
    domain_faces[2].imageuvs = V(Uv(.50f, .00f), Uv(1.0f, .50f), Uv(.50f, .50f));
    // 012: face 3 in left-to-right image order.
    domain_faces[3].imageuvs = V(Uv(.50f, .00f), Uv(1.0f, .00f), Uv(1.0f, .50f));
  } else if (domain == "octa" || domain == "octaflat") {
    // Image face order:
    //  -X vertex at image center,
    //  +X vertex at 4 image corners,
    //  (-Y, +Z) to image (+X, +Y).
    constexpr Vec4<Vector> dir =
        V(Vector(+1.f, 0.f, 0.f), Vector(0.f, +1.f, 0.f), Vector(-1.f, 0.f, 0.f), Vector(0.f, -1.f, 0.f));
    domain_faces.init(8);
    for_int(i, domain_faces.num()) {
      domain_faces[i].facecolor = V(Pixel::red(), Pixel::green(), Pixel::blue(), k_pixel_orange, Pixel(70, 20, 180),
                                    Pixel(190, 153, 190), Pixel(180, 70, 20), Pixel(20, 180, 70))[i];
      if (!octa8colors && i >= 4) domain_faces[i].facecolor = domain_faces[(i + 2) % 4].facecolor;
      domain_faces[i].pixel_checker =
          V(Pixel::red(), Pixel::green(), Pixel::blue(), k_pixel_orange, Pixel(120, 190, 190), Pixel(180, 180, 120),
            Pixel(190, 110, 190), Pixel(160, 190, 120))[i];
      const bool front = i < 4;
      const int q = front ? i : i - 4;
      Vector dir0 = dir[q], dir1 = dir[(q + 1) % 4];
      if (!front) std::swap(dir0, dir1);
      domain_faces[i].poly =
          V(Point(0.f, -dir0[0], dir0[1]), Point(0.f, -dir1[0], dir1[1]), Point(front ? -1.f : +1.f, 0.f, 0.f));
      domain_faces[i].stretchuvs = V(Uv(0.f, 0.f), Uv(1.f, 0.f), Uv(.5f, .5f * sqrt(3.f)));
      domain_faces[i].imageuvs =
          V(Uv(.5f * (1.f + dir0[0]), .5f * (1.f + dir0[1])),  //
            Uv(.5f * (1.f + dir1[0]), .5f * (1.f + dir1[1])),
            front ? Uv(.5f, .5f) : Uv(.5f * (1.f + dir0[0] + dir1[0]), .5f * (1.f + dir0[1] + dir1[1])));
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
    // Fully supporting baseball would require changes in adjust_for_domain(), non 1-1 mapping in
    // do_write_primal_texture(),  etc.
    domain_faces.init(6);
    for_int(i, domain_faces.num()) {
      domain_faces[i].facecolor =
          V(Pixel::green(), Pixel::red(), Pixel::green(), Pixel::red(), Pixel::blue(), Pixel::blue())[i];
      domain_faces[i].poly.init(4);  // Defined later below.
      domain_faces[i].stretchuvs = V(Uv(0.f, 0.f), Uv(1.f, 0.f), Uv(1.f, 1.f), Uv(0.f, 1.f));
      Vec2<float> s;
      Uv uvo;
      if (!baseball) {  // Cross.
        s = twice(1.f / 4.f);
        uvo = Uv(s[1] * (i < 4 ? i : 1), s[0] * (i < 4 ? 1 : i == 4 ? 0 : 2));
      } else {  // Baseball.
        s = V(1.f / 3.f, 1.f / 2.f);
        uvo = Uv(i < 3 ? 0.f : s[1], s[0] * V(0, 1, 2, 0, 1, 2)[i]);
      }
      domain_faces[i].imageuvs = V(Uv(uvo[0] + 0, uvo[1] + 0), Uv(uvo[0] + s[1], uvo[1] + 0),
                                   Uv(uvo[0] + s[1], uvo[1] + s[0]), Uv(uvo[0] + 0, uvo[1] + s[0]));
    }
    const float s = sqrt(1.f / 3.f);
    Vec<Vec4<Vec3<int>>, 6> fvcoords;
    if (!baseball) {                                                               // Cross.
      fvcoords = V(V(V(+1, +1, -1), V(-1, +1, -1), V(-1, +1, +1), V(+1, +1, +1)),  // Y+
                   V(V(-1, +1, -1), V(-1, -1, -1), V(-1, -1, +1), V(-1, +1, +1)),  // X-
                   V(V(-1, -1, -1), V(+1, -1, -1), V(+1, -1, +1), V(-1, -1, +1)),  // Y-
                   V(V(+1, -1, -1), V(+1, +1, -1), V(+1, +1, +1), V(+1, -1, +1)),  // X+
                   V(V(+1, +1, -1), V(+1, -1, -1), V(-1, -1, -1), V(-1, +1, -1)),  // Z-
                   V(V(-1, +1, +1), V(-1, -1, +1), V(+1, -1, +1), V(+1, +1, +1))   // Z+
      );
    } else {                                                                       // Baseball.
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

Pixel get_color(const DomainFace& domain_face, Uv uv) {
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
        uv = Uv(eps, 1.f - 2.f * eps);
      else if (uv[1] < eps)
        uv = Uv(1.f - 2.f * eps, eps);
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
  const Vec3<Point> triangle = V(pa, pb, pc);
  Point p = ok_normalized(b[0] * pa + b[1] * pb + b[2] * pc);
  SGrid<float, 3, 3> tp;
  const float eps = 1e-4f;
  for_int(i, 100) {
    Vector q{};
    // Map A, B, C to tangent plane at p using inverse exponential map.
    for_int(v, 3) {
      float sin_theta, cos_theta;
      const float theta = angle_between_unit_vectors_and_sincos(p, triangle[v], sin_theta, cos_theta);
      if (cos_theta >= 1.f || abs(sin_theta) < eps / 10) {
        ASSERTX(b[v] > 1.f - eps);
        return triangle[v] * b[v];
      }
      ASSERTX(sin_theta != 0.f);
      for_int(k, 3) {
        tp[v][k] = (triangle[v][k] - p[k] * cos_theta) / sin_theta;  // Unit vector perpendicular to p.
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

using TriangleSpheremap = Point (*)(const Vec3<Point>& triangle, const Bary& bary);

Point map_sphere(const Vec3<Point>& triangle, const Bary& bary) {
  // return normalized(interp(triangle, bary));
  return spheremap_linear_reproject(triangle[0], triangle[1], triangle[2], bary);
}

Point map_2slerp0(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_2slerp0(triangle[0], triangle[1], triangle[2], bary);
}

Point map_2slerp1(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_2slerp0(triangle[1], triangle[2], triangle[0], Bary(bary[1], bary[2], bary[0]));
}

Point map_2slerp2(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_2slerp0(triangle[2], triangle[0], triangle[1], Bary(bary[2], bary[0], bary[1]));
}

Point map_2slerps(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_sym_2slerps(triangle[0], triangle[1], triangle[2], bary);
}

Point map_arvo0(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_arvo0(triangle[0], triangle[1], triangle[2], bary);
}

Point map_arvo1(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_arvo0(triangle[1], triangle[2], triangle[0], Bary(bary[1], bary[2], bary[0]));
}

Point map_arvo2(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_arvo0(triangle[2], triangle[0], triangle[1], Bary(bary[2], bary[0], bary[1]));
}

Point map_arvos(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_sym_arvo(triangle[0], triangle[1], triangle[2], bary);
}

Point map_buss(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_sym_buss_fillmore(triangle[0], triangle[1], triangle[2], bary);
}

Point map_area(const Vec3<Point>& triangle, const Bary& bary) {
  return spheremap_area_ratio(triangle[0], triangle[1], triangle[2], bary);
}

Point map_trisub(const Vec3<Point>& triangle, const Bary& bary) {
  ASSERTX(bary.is_convex());
  Vec3<Point> pc = triangle;
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

struct S_TriangleSpheremap {
  const char* name;
  TriangleSpheremap map;
};

#define E(x) \
  { #x, map_##x }
const Array<S_TriangleSpheremap> k_triangle_maps = {
    E(sphere), E(2slerp0), E(2slerp1), E(2slerp2), E(2slerps), E(arvo0),
    E(arvo1),  E(arvo2),   E(arvos),   E(buss),    E(area),    E(trisub),
#undef E
};

TriangleSpheremap get_map(const string& triangle_map_name) {
  for (const auto& triangle_map : k_triangle_maps)
    if (triangle_map_name == triangle_map.name) return triangle_map.map;
  assertnever("map '" + triangle_map_name + "' not found");
}

// *** domain -> sphere grid mapping.

// Given uv coordinates within spherical quad, return a spherical triangle and barycentric coordinates within it.
void split_quad_2tris(const Vec4<Point>& po, float fi, float fj, Vec3<Point>& triangle, Bary& bary) {
  if (fi + fj <= 1.f) {
    triangle = V(po[0], po[1], po[3]);
    bary = Bary(1.f - fi - fj, fj, fi);
  } else {
    triangle = V(po[2], po[3], po[1]);
    bary = Bary(fi + fj - 1.f, 1.f - fj, 1.f - fi);
  }
}

void split_quad_4tris(const Vec4<Point>& po, float fi, float fj, Vec3<Point>& triangle, Bary& bary) {
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
  triangle = V(po[q], po[(q + 1) % 4], normalized(bilerp(po, .5f, .5f)));
  bary = Bary(bq0, bq1, bc);
}

void split_quad_8tris(const Vec4<Point>& po, float fi, float fj, Vec3<Point>& triangle, Bary& bary,
                      TriangleSpheremap triangle_map) {
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
    triangle = V(po[q], normalized(interp(po[q], po[(q + 1) % 4])), normalized(bilerp(po, .5f, .5f)));
  } else {
    // Flip the triangle to obtain symmetry --- that is OK with most triangle_map functions.
    triangle = V(po[q], normalized(interp(po[q], po[(q + 3) % 4])), normalized(bilerp(po, .5f, .5f)));
    bary = Bary(max(0.f, 1.f - t), t - s, s);
    if (triangle_map == map_area) {
      // Flipped triangles not handled correctly by current map_area. Since it is symmetric, undoing reflection is OK.
      std::swap(triangle[0], triangle[1]);
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
  string quad_map_name;
  TriangleSpheremap triangle_map = nullptr;
  switch (scheme[0]) {
    case 'Q':
      assertx(quad_domain);
      quad_interp = true;
      quad_map_name = scheme.substr(1);
      if (quad_map_name == "domain") domain_interp = true;
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
    string map_name = scheme.substr(1);
    if (map_name == "domain") {
      domain_interp = true;
    } else {
      triangle_map = get_map(map_name);
    }
  } else {
    assertnever("scheme not recognized: " + scheme);
  }
  //
  assertx(g_mesh.empty());
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
      Vertex v = g_mesh.create_vertex();
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
        Uv uv;
        if (!quad_domain) {
          uv = interp(df.stretchuvs[0], df.stretchuvs[1], df.stretchuvs[2], bary);
        } else {
          uv = qinterp(df.stretchuvs[0], df.stretchuvs[1], df.stretchuvs[2], df.stretchuvs[3], bary);
        }
        v_stretchuv(v) = uv;
      }
      {
        Uv uv;
        if (!quad_domain) {
          uv = interp(df.imageuvs[0], df.imageuvs[1], df.imageuvs[2], bary);
        } else {
          uv = qinterp(df.imageuvs[0], df.imageuvs[1], df.imageuvs[2], df.imageuvs[3], bary);
        }
        uv = Uv(hashf.enter(uv[0]), hashf.enter(uv[1]));
        v_imageuv(v) = uv;
      }
      if (write_imagen) {
        const Uv& uv = v_imageuv(v);
        int imi = int(uv[0] * (2 * gridn) + .5f);
        int imj = int(uv[1] * (2 * gridn) + .5f);
        int imn = imj * (2 * gridn + 1) + imi + 1;
        g_mesh.update_string(v, "imagen", csform(str, "%d", imn));
      }
      if ((i == 0 || i == gridn) && (j == 0 || j == gridn) && contains(key_names, "domaincorner"))
        g_mesh.update_string(v, "domaincorner", "");
      v_ijuv(v) = Uv(float(i) / gridn, float(j) / gridn);
      if (checkern) {
        const Pixel& pixel = get_color(df, v_ijuv(v));
        const Vector rgb = convert<float>(pixel.head<3>()) / 255.f;
        v_rgb(v) = rgb;
      }
    }

    if (quad_interp && (domain_interp || quad_map_name == "sphere")) {
      // Quad rasterization.
      assertx(gridn >= 1);
      for_int(i, gridn + 1) for_int(j, gridn + 1) {
        Vertex v = verts[i][j];
        const Uv uv(float(j) / gridn, float(i) / gridn);
        Point p;
        if (domain_interp) {
          p = bilerp(po, uv[0], uv[1]);
        } else {  // Sphere map.
          p = normalized(bilerp(po, uv[0], uv[1]));
        }
        g_mesh.set_point(v, p);
      }

    } else if (quad_interp) {
      // Recursive quad subdivision.
      if (!is_pow2(gridn)) SHOW(gridn);
      assertx(gridn >= 1 && is_pow2(gridn));
      const bool is_warp = quad_map_name == "warp";
      const bool is_diags = quad_map_name == "diags";
      const bool is_diagl = quad_map_name == "diagl";
      assertx(is_warp || is_diags || is_diagl);
      // First assign corners.
      for_int(ii, 2) for_int(jj, 2) {
        const int i = ii * gridn;
        const int j = jj * gridn;
        Vertex v = verts[i][j];
        const Uv uv(float(j) / gridn, float(i) / gridn);
        const Point p = normalized(bilerp(po, uv[0], uv[1]));  // Corners.
        g_mesh.set_point(v, p);
      }
      // Perform recursive quad subdivision.
      for (int s = gridn / 2; s >= 1; s /= 2) {
        for (int i = 0; i <= gridn; i += s * 2) {
          for (int j = s; j < gridn; j += s * 2) {
            const Point p = normalized(interp(g_mesh.point(verts[i][j - s]), g_mesh.point(verts[i][j + s])));
            g_mesh.set_point(verts[i][j], p);
          }
        }
        for (int j = 0; j <= gridn; j += s * 2) {
          for (int i = s; i < gridn; i += s * 2) {
            const Point p = normalized(interp(g_mesh.point(verts[i - s][j]), g_mesh.point(verts[i + s][j])));
            g_mesh.set_point(verts[i][j], p);
          }
        }
        for (int i = s; i < gridn; i += s * 2) {
          for (int j = s; j < gridn; j += s * 2) {
            const Point& p00 = g_mesh.point(verts[i - s][j - s]);
            const Point& p01 = g_mesh.point(verts[i - s][j + s]);
            const Point& p10 = g_mesh.point(verts[i + s][j - s]);
            const Point& p11 = g_mesh.point(verts[i + s][j + s]);
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
            g_mesh.set_point(verts[i][j], p);
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
        Vec3<Point> triangle;  // Desired spherical triangle vertices.
        Bary bary;             // Barycentric coordinates within triangle.
        const float fi = float(i) / gridn, fj = float(j) / gridn;
        if (!quad_domain) {
          for_int(c, 3) triangle[c] = po[c];
          bary = Bary(max(0.f, 1.f - fi - fj), fi, fj);
        } else if (split_quad_ntris == 2) {
          split_quad_2tris(po, fi, fj, triangle, bary);
        } else if (split_quad_ntris == 4) {
          split_quad_4tris(po, fi, fj, triangle, bary);
        } else if (split_quad_ntris == 8) {
          split_quad_8tris(po, fi, fj, triangle, bary, triangle_map);
        } else {
          assertnever("");
        }
        for_int(c, 3) ASSERTX(is_unit(triangle[c]));
        Point p;
        if (domain_interp) {
          p = interp(triangle, bary);
        } else {
          p = triangle_map(triangle, bary);
          assertx(is_unit(p));
        }
        g_mesh.set_point(verts[i][j], p);
      }
    }

    if (omit_faces) {
      // Do not create any mesh faces.
    } else if (!quad_domain) {
      for_int(i, gridn) for_int(j, gridn - i) {
        if (1) {
          Face f = g_mesh.create_face(verts[i + 0][j + 0], verts[i + 1][j + 0], verts[i + 0][j + 1]);
          f_domainf(f) = domainf;
        }
        if (i) {
          Face f = g_mesh.create_face(verts[i + 0][j + 0], verts[i + 0][j + 1], verts[i - 1][j + 1]);
          f_domainf(f) = domainf;
        }
      }
    } else {
      for_int(i, gridn) for_int(j, gridn) {
        const Vec4<Vertex> va{verts[i + 0][j + 0], verts[i + 0][j + 1], verts[i + 1][j + 1], verts[i + 1][j + 0]};
        Face f;
        if (!b_triangulate) {
          f = g_mesh.create_face(va);
          f_domainf(f) = domainf;
        } else {
          const int k = ((i >= gridn / 2) ^ (j >= gridn / 2));
          f = g_mesh.create_face(va[(k + 0) % 4], va[(k + 1) % 4], va[(k + 2) % 4]);
          f_domainf(f) = domainf;
          f = g_mesh.create_face(va[(k + 0) % 4], va[(k + 2) % 4], va[(k + 3) % 4]);
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
  if (g_mesh.empty()) create(true);
  is_remeshed = true;
}

// A remesh is a mesh with vertices in raster-scan order within each domain face.
// The vertex positions are on a resampled surface.
void do_load_remesh(Args& args) {
  assertx(g_mesh.empty());
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
  assertx(g_mesh.num_vertices() == remesh.num_vertices());
  Array<char> key, val;
  for (Vertex vp : remesh.vertices()) {
    Vertex v = assertx(g_mesh.id_retrieve_vertex(remesh.vertex_id(vp)));
    g_mesh.set_point(v, remesh.point(vp));
    // Copy normal, rgb, etc.
    for_cstring_key_value(remesh.get_string(vp), key, val, [&] { g_mesh.update_string(v, key.data(), val.data()); });
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

void verify_good_sphparam(const GMesh& mesh) {
  for (Vertex v : mesh.vertices()) assertx(is_unit(v_sph(v)));
  for (Face f : mesh.faces()) {
    const Vec3<Point> sphs = map(mesh.triangle_vertices(f), v_sph);
    const float sarea = spherical_triangle_area(sphs);
    HH_SSTAT(Ssarea, sarea);
    if (!(sarea < TAU)) SHOW(sarea, sphs);
    assertw(sarea < TAU);                    // Else it is flipped.
    if (verbose >= 2) assertw(sarea > 0.f);  // Else it is degenerate.
    if (0 && !(sarea > 0.f)) SHOW(sarea, sphs);
  }
}

GMesh read_sphparam_mesh(const string& filename) {
  HH_TIMER("_read_sphparam_mesh");
  assertx(filename != "");
  RFile fi(filename);
  for (string line; fi().peek() == '#';) {
    assertx(my_getline(fi(), line));
    if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
  }
  GMesh param_mesh;
  param_mesh.read(fi());
  assertx(param_mesh.num_faces());
  showdf("sphere map: nv=%d nf=%d\n", param_mesh.num_vertices(), param_mesh.num_faces());
  const bool normsph = getenv_bool("SPHERESAMPLE_NORMSPH");
  for (Vertex v : param_mesh.vertices()) {
    Point sph;
    assertx(parse_key_vec(param_mesh.get_string(v), "sph", sph));
    if (!normsph)
      assertx(is_unit(sph));
    else
      sph = normalized(sph);
    v_sph(v) = sph;
  }
  verify_good_sphparam(param_mesh);
  return param_mesh;
}

// A map is a gmerged mesh with vertex positions on domain, and sph strings for parameterization on sphere.
void do_load_map(Args& args) {
  assertx(!gridn);
  assertx(g_mesh.empty());
  const GMesh domain_mesh = read_sphparam_mesh(args.get_filename());  // Map: domain D -> sphere S (v -> v_sph(v)).
  gridn = get_gridn(domain_mesh.num_vertices());
  assertx(scheme == "");  // Not modified on command line.
  scheme = "domain";      // Any scheme will do.
  create(false);
  for (Vertex v : g_mesh.vertices()) g_mesh.set_point(v, Point(BIGFLOAT, 0.f, 0.f));
  Array<Point> arp;
  arp.reserve(g_mesh.num_vertices());  // Necessary to prevent reallocation.
  PointSpatial<Vertex> spatial(max(10, gridn));
  for (Vertex v : g_mesh.vertices()) {
    arp.push(v_domainp(v) * k_fdomain_to_spatialbb);
    spatial.enter(v, &arp.last());
  }
  for (Vertex domain_v : domain_mesh.vertices()) {
    const Point& sphp = v_sph(domain_v);
    const Point& domainp = domain_mesh.point(domain_v);
    int nfaces = 0;
    SpatialSearch<Vertex> ss(&spatial, domainp * k_fdomain_to_spatialbb);
    for (;;) {
      const auto [vv, d2] = ss.next();
      if (d2 > 1e-12f) break;
      nfaces++;
      assertx(g_mesh.point(vv)[0] == BIGFLOAT);
      g_mesh.set_point(vv, sphp);
    }
    if (nfaces < 1 || nfaces > 4) SHOW(nfaces, domainp, sphp);
    assertx(nfaces >= 1 && nfaces <= 4);  // Max valence is 3 or 4.
  }
  for (Vertex v : g_mesh.vertices()) assertx(g_mesh.point(v)[0] != BIGFLOAT);
}

void do_sample_map(Args& args) {
  assertx(gridn);
  assertx(g_mesh.empty());
  const GMesh domain_mesh = read_sphparam_mesh(args.get_filename());  // Map: domain D -> sphere S (v -> v_sph(v)).

  if (scheme == "") scheme = "domain";
  create(scheme != "domain");

  const MeshSearch mesh_search(domain_mesh, {true});
  HH_TIMER("_resample0");
  ConsoleProgress cprogress;
  int nv = 0;
  Face hint_f = nullptr;
  for (Vertex v : g_mesh.ordered_vertices()) {
    if ((nv++ & 0xff) == 0) cprogress.update(float(nv) / g_mesh.num_vertices());
    auto [domain_f, bary, unused_clp, d2] = mesh_search.search(v_domainp(v), hint_f);
    hint_f = domain_f;
    assertx(d2 < 1e-12f);
    Vector sum{};
    const Vec3<Vertex> domain_face_vertices = domain_mesh.triangle_vertices(domain_f);
    for_int(i, 3) sum += bary[i] * v_sph(domain_face_vertices[i]);
    g_mesh.set_point(v, normalized(sum));
  }
}

// Returns k_undefined_vector if undefined.
Vector interp_f_normal(const GMesh& mesh, Face f, const Bary& bary) {
  const Vec3<Corner> corners = mesh.triangle_corners(f);
  Vector sum_normal{};
  int num_defined = 0;
  for_int(i, 3) {
    Corner c = corners[i];
    Vector normal;
    if (GMesh::string_has_key(mesh.get_string(c), "normal")) {
      assertx(mesh.parse_corner_key_vec(c, "normal", normal));
    } else if (Vertex v = mesh.corner_vertex(c); v_normal(v) != k_undefined_vector) {
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
Vector interp_f_rgb(const GMesh& mesh, Face f, const Bary& bary) {
  const Vec3<Corner> corners = mesh.triangle_corners(f);
  Vector sum_rgb{};
  int num_defined = 0;
  for_int(i, 3) {
    Corner c = corners[i];
    Vector rgb;
    if (GMesh::string_has_key(mesh.get_string(c), "rgb")) {
      assertx(mesh.parse_corner_key_vec(c, "rgb", rgb));
    } else if (Vertex v = mesh.corner_vertex(c); v_rgb(v) != k_undefined_vector) {
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

// Returns k_undefined_uv if undefined.
Uv interp_f_uv(const GMesh& mesh, Face f, const Bary& bary) {
  const Vec3<Corner> corners = mesh.triangle_corners(f);
  Uv sum_uv{};
  int num_defined = 0;
  for_int(i, 3) {
    Corner c = corners[i];
    Uv uv;
    if (!mesh.parse_corner_key_vec(c, "Ouv", uv)) continue;
    num_defined++;
    sum_uv += uv * bary[i];
  }
  if (num_defined == 0) return k_undefined_uv;
  assertx(num_defined == 3);
  return sum_uv;
}

// *** Inverse map.

void internal_remesh() {
  if (is_remeshed) return;
  GMesh param_mesh = read_sphparam_mesh(param_file);

  if (g_mesh.empty()) {
    // Create domain grid mesh.
    HH_STIMER("_create_tess");
    create(false);
  }

  // This mesh has vertex positions on original model surface, and parameterization v_sph(v) on sphere.
  for (Vertex param_v : param_mesh.vertices()) {
    v_domainp(param_v) = param_mesh.point(param_v);
    param_mesh.set_point(param_v, v_sph(param_v));
    Vector& normal = v_normal(param_v);
    if (!parse_key_vec(param_mesh.get_string(param_v), "normal", normal)) normal = k_undefined_vector;
    Vector& rgb = v_rgb(param_v);
    if (!parse_key_vec(param_mesh.get_string(param_v), "rgb", rgb)) rgb = k_undefined_vector;
  }
  MeshSearch::Options options;
  options.allow_local_project = true;
  options.allow_off_surface = true;
  const MeshSearch mesh_search(param_mesh, options);

  HH_TIMER("_resample");
  const int num_threads = get_max_threads();
  parallel_for_chunk(Array<Vertex>(g_mesh.vertices()), num_threads, [&](const int thread_index, auto subrange) {
    dummy_use(thread_index);
    string str;
    Face hint_f = nullptr;
    for (Vertex v : subrange) {
      const Point& sph = g_mesh.point(v);  // Point on sphere.
      v_sph(v) = sph;
      assertx(is_unit(sph));
      auto [param_f, bary] = mesh_search.search_on_sphere(sph, hint_f);
      hint_f = param_f;
      const Vec3<Point> triangle = map(g_mesh.triangle_vertices(param_f), v_domainp);
      const Point newp = interp(triangle, bary);
      g_mesh.set_point(v, newp);
      v_normal(v) = interp_f_normal(param_mesh, param_f, bary);
      if (checkern) {
        // Ignore mesh color since checkering.
      } else {
        Vector rgb = interp_f_rgb(param_mesh, param_f, bary), rgb2;
        if (rgb == k_undefined_vector && parse_key_vec(param_mesh.get_string(param_f), "rgb", rgb2)) rgb = rgb2;
        v_rgb(v) = rgb;
      }
    }
  });
  is_remeshed = true;
}

void triangulate_short_diag() {
  HH_STIMER("_triangulate_short");
  Array<Vertex> va;
  for (Face f : Array<Face>(g_mesh.faces())) {
    g_mesh.get_vertices(f, va);
    if (va.num() == 3) continue;
    assertx(va.num() == 4);
    Vertex va0 = va[0], va2 = va[2];
    const bool other_diag =
        dist2(g_mesh.point(va[1]), g_mesh.point(va[3])) < dist2(g_mesh.point(va[0]), g_mesh.point(va[2]));
    if (other_diag) {
      va0 = va[1];
      va2 = va[3];
    }
    const int domainf = f_domainf(f);
    Edge e = g_mesh.split_face(f, va0, va2);
    for (Face ff : g_mesh.faces(e)) f_domainf(ff) = domainf;
  }
}

// Sets key_names.
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
  assertx(contains(V<string>("G", "N", "C", "T"), signal_));
}

void do_texture_file(Args& args) {
  string filename = args.get_string();
  Image texture_image{filename};
  texture_image_vector4.init(texture_image.dims());
  convert(texture_image, texture_image_vector4);
}

Pixel assign_signal(const GMesh& mesh, const Bbox<float, 3>& bbox, const Frame& rotate_frame, Face f,
                    const Bary& bary) {
  if (GMesh::string_has_key(mesh.get_string(f), "filled")) return k_color_special_filled;
  Pixel pixel(255, 255, 255, 255);
  switch (signal_[0]) {
    case 'G': {
      const Vec3<Vertex> face_vertices = mesh.triangle_vertices(f);
      Point p_m{};
      for_int(i, 3) p_m += bary[i] * v_domainp(face_vertices[i]);
      for_int(z, 3) {
        const float frac = (p_m[z] - bbox[0][z]) / (bbox[1][z] - bbox[0][z]);
        assertx(frac >= 0.f && frac <= 1.f);
        pixel[z] = uint8_t(frac * 255.f + .5f);
      }
      break;
    }
    case 'N': {
      const Vector normal = interp_f_normal(mesh, f, bary) * rotate_frame;
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
    case 'T': {
      assertx(texture_image_vector4.ysize());
      const Uv uv = interp_f_uv(mesh, f, bary);
      // We flip the image vertically because the OpenGL Uv coordinate origin is at the image lower-left.
      const Uv yx = Uv(1.f - uv[1], uv[0]);  // Also, we expect (y, x) sampling order.
      const string filtername = 1 ? "keys" : "triangle";
      const auto filterbs = twice(FilterBnd(Filter::get(filtername), Bndrule::reflected));
      pixel = sample_domain(texture_image_vector4, yx, filterbs).pixel();
      break;
    }
    default: assertnever("signal '" + signal_ + "' not recognized");
  }
  return pixel;
}

void handle_zero_alpha_pixels_for_filled_faces(Image& image) {
  assertx(k_color_special_filled[3] == 0);
  if (getenv_bool("FILLED_AS_PINK")) {
    const Pixel k_color_pink(255, 192, 203, 255);
    parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) {
      if (image[yx][3] == 0) image[yx] = k_color_pink;
    });
    return;
  }
  // Gradient-domain fill; adapted from "Filterimage -gdfill".
  HH_TIMER("_gdfill");
  Grid<2, Vector4> grid_orig(image.dims());
  parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) { grid_orig[yx] = Vector4(image[yx]); });
  const auto masked = [&](const Vec2<int>& yx) { return image[yx][3] < 255; };
  Vector4 vmean{};
  for_coords(image.dims(), [&](const Vec2<int>& yx) {  // Sequential because of reduction.
    if (!masked(yx)) vmean += grid_orig[yx];
  });
  vmean /= vmean[3];
  parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) {
    if (masked(yx)) grid_orig[yx] = vmean;
  });
  Multigrid<2, Vector4> multigrid(image.dims());
  // Solve for color offsets; rhs and residual are sparse.
  fill(multigrid.initial_estimate(), Vector4(0.f));
  multigrid.set_desired_mean(Vector4(0.f));
  parallel_for_coords({100}, image.dims(), [&](const Vec2<int>& yx) {
    Vector4 vrhs{};
    for (auto yxd : {V(-1, 0), V(+1, 0), V(0, -1), V(0, +1)}) {
      const auto yxn = yx + yxd;
      if (grid_orig.ok(yxn) && masked(yx) != masked(yxn)) vrhs += (grid_orig[yx] - grid_orig[yxn]);
    }
    multigrid.rhs()[yx] = vrhs;
  });
  multigrid.set_num_vcycles(1);
  multigrid.solve();
  auto grid_result = multigrid.result();
  parallel_for_coords(image.dims(), [&](const Vec2<int>& yx) {
    image[yx] = (grid_orig[yx] + grid_result[yx]).pixel();  // (Alpha value is overwritten with 255.)
  });
}

void blend_pixels(Pixel& pixel0, Pixel& pixel1) {
  const Pixel average_pixel = interp(Vector4(pixel0), Vector4(pixel1)).pixel();
  pixel0 = pixel1 = average_pixel;
}

void apply_feathering(Image& image) {
  const int ny = image.ysize(), nx = image.xsize();
  const bool expecting_periodic_texturing_mode = false;  // By default, the texturing mode is often GL_CLAMP_TO_EDGE.
  if (0) {
  } else if (domain == "tetra") {
    assertx(ny == nx && ny % 2 == 0);
    const int h = ny / 2;
    // Blend across the left and right sides of the image.
    if (!expecting_periodic_texturing_mode)
      for (const int i : range(h)) blend_pixels(image[h + i][0], image[h + i][h * 2 - 1]);
    // Assign pixel values above the rectangle.
    for (const int i : range(h * 2)) image[h - 1][i] = image[h][h * 2 - 1 - i];
    // Blend pixel values in the lowest row of the rectangle.
    for (const int i : range(h)) blend_pixels(image[h * 2 - 1][i], image[h * 2 - 1][h * 2 - 1 - i]);

  } else if (domain == "octa" || domain == "octaflat") {
    // For each of the four image boundaries, we must establish mirror symmmetry within the pixels on the boundary.
    // This leads to a narrow (1-texel width) band of zero-derivative signal across the texture boundary, but this
    // is much better than leaving a visible discontinuity.
    for (const int axis : {0, 1})
      for (const int column_index : {0, image.dim(axis) - 1}) {  // (General "column" is either row or col.)
        StridedArrayView<Pixel> single_border = grid_column(image, 1 - axis, twice(0).with(axis, column_index));
        const int num = single_border.num();
        for (const int i : range(num / 2)) blend_pixels(single_border[i], single_border[num - 1 - i]);
      }

  } else if (domain == "cube") {
    // Blend across the left and right sides of the image.
    assertx(ny == nx && ny % 4 == 0);
    const int q = ny / 4;
    if (!expecting_periodic_texturing_mode)
      for (const int i : range(q)) blend_pixels(image[q * 2 + i][0], image[q * 2 + i][q * 4 - 1]);
    // Assign pixel values in the horizontal segment above the top of the "+" shape in the image.
    for (const int i : range(q)) image[q - 1][q + i] = image[q * 2][q * 4 - 1 - i];
    // Blend the horizontal segment at the base of the "+" (lowest image row) with the adjacent cube face.
    for (const int i : range(q)) blend_pixels(image[q * 4 - 1][q + i], image[q * 3 - 1][q * 4 - 1 - i]);  // (A).
    // Assign pixel values above the left side of the "+" shape.
    for (const int i : range(q)) image[q * 2 - 1][i] = image[q * 2 - 1 - i][q];
    // Assign pixel values below the left side of the "+" shape.
    for (const int i : range(q)) image[q * 3][i] = image[q * 4 - 1 - i][q];
    // Assign pixel values above the right side of the "+" shape.
    for (const int i : range(q)) image[q * 2 - 1][q * 2 + i] = image[q * 2 - 1 - i][q * 2 - 1];
    for (const int i : range(q)) image[q * 2 - 1][q * 3 + i] = image[q][q * 2 - 1 - i];
    // Assign pixel values below the right side of the "+" shape.
    for (const int i : range(q)) image[q * 3][q * 2 + i] = image[q * 4 - 1 - i][q * 2 - 1];
    for (const int i : range(q)) image[q * 3][q * 3 + i] = image[q * 4 - 1][q * 2 - 1 - i];  // Already blended in (A).
    // Assign pixel values left of the top part of the "+" shape.
    for (const int i : range(q)) image[q + i][q - 1] = image[q * 2][i];
    // Assign pixel values left of the bottom part of the "+" shape.
    for (const int i : range(q)) image[q * 3 + i][q - 1] = image[q * 3 - 1][q - 1 - i];
    // Assign pixel values right of the top part of the "+" shape.
    for (const int i : range(q)) image[q + i][q * 2] = image[q * 2][q * 3 - 1 - i];
    // Assign pixel values right of the bottom part of the "+" shape.
    for (const int i : range(q)) image[q * 3 + i][q * 2] = image[q * 3 - 1][q * 2 + i];
    // Assign two leftover pixels.
    image[q - 1][q - 1] = image[q * 2][q * 4 - 1];
    image[q - 1][q * 2] = image[q * 2][q * 3];
  } else {
    assertnever("domain name '" + domain + "' not recognized");
  }
}

void do_write_texture(Args& args) {
  const string image_name = args.get_filename();
  assertx(signal_ != "");
  assertx(domain_file != "");
  assertx(param_file != "");
  assertx(gridn);
  assertx(g_mesh.empty());

  HH_TIMER("_write_texture");
  const GMesh domain_mesh = read_sphparam_mesh(domain_file);  // Map: domain D -> sphere S (v -> v_sph(v)).
  GMesh param_mesh = read_sphparam_mesh(param_file);          // Map (initially): mesh M -> sphere S (v -> v_sph(v)).
  const Bbox bbox{transform(param_mesh.vertices(), [&](Vertex v) { return param_mesh.point(v); })};
  const Frame rotate_frame = get_rotate_frame();

  GMesh mesh_i;  // Map: image I -> domain D  (v -> v_domainp(v)).
  {
    const int orig_gridn = std::exchange(gridn, 1);  // Create an untessellated domain mesh.
    scheme = "domain";  // The choice does not matter because the domain is not tessellated.
    create(true);
    gridn = orig_gridn;
    //
    mesh_i.copy(g_mesh);
    for (Vertex v : mesh_i.vertices()) {
      Vertex vv = g_mesh.id_vertex(g_mesh.vertex_id(v));
      v_domainp(v) = g_mesh.point(vv);
      mesh_i.set_point(v, concat(v_imageuv(vv), V(0.f)));
    }
  }

  // Invert the mapping of param_mesh to create the map: sphere S -> mesh M (v -> v_domainp(v)).
  for (Vertex v : param_mesh.vertices()) {
    v_domainp(v) = param_mesh.point(v);
    param_mesh.set_point(v, v_sph(v));
    Vector& normal = v_normal(v);
    if (!parse_key_vec(param_mesh.get_string(v), "normal", normal)) normal = k_undefined_vector;
    Vector& rgb = v_rgb(v);
    if (!parse_key_vec(param_mesh.get_string(v), "rgb", rgb)) rgb = k_undefined_vector;
  }

  MeshSearch::Options options_i;
  options_i.bbox = Bbox(Point(0.f, 0.f, 0.f), Point(1.f, 1.f, 0.f));
  const MeshSearch msearch_i(mesh_i, options_i);                // Map: image I -> domain D (v -> v_domainp(v)).
  const MeshSearch msearch_d(domain_mesh, {true});              // Map: domain D -> sphere S (v -> v_sph(v)).
  const MeshSearch msearch_s(param_mesh, {true, false, true});  // Map: sphere S -> mesh M (v -> v_domainp(v)).

  Image image(V(gridn, gridn));
  HH_TIMER("_write_pixels");
  const int num_threads = get_max_threads();
  parallel_for_chunk(range(image.ysize()), num_threads, [&](const int thread_index, auto subrange) {
    dummy_use(thread_index);
    Face hint_f_d = nullptr, hint_f_s = nullptr;
    for (const int y : subrange) {
      for_int(x, image.xsize()) {
        // We flip the image vertically because the OpenGL Uv coordinate origin is at the image lower-left.
        const int yy = image.ysize() - 1 - y;
        Pixel& pixel = image[yy][x];
        Point p_i, p_d, p_s;
        p_i = Point((x + 0.5f) / image.xsize(), (y + 0.5f) / image.ysize(), 0.f);  // Dual sampling.
        {
          auto [f, bary, unused_clp, d2] = msearch_i.search(p_i, nullptr);
          if (d2 > 0.f) {
            pixel = Pixel(255, 255, 255, 255);
            continue;
          }
          const Vec3<Point> triangle = map(mesh_i.triangle_vertices(f), v_domainp);
          p_d = interp(triangle, bary);
        }
        {
          auto [f, bary, unused_clp, d2] = msearch_d.search(p_d, hint_f_d);
          hint_f_d = f;
          if (d2 >= 1e-12f) assertnever(SSHOW(p_i, p_d, f, bary, unused_clp, d2));
          const Vec3<Vertex> face_vertices = domain_mesh.triangle_vertices(f);
          Vector sum{};
          for_int(i, 3) sum += bary[i] * v_sph(face_vertices[i]);
          p_s = normalized(sum);
        }
        auto [f, bary] = msearch_s.search_on_sphere(p_s, hint_f_s);
        hint_f_s = f;
        pixel = assign_signal(param_mesh, bbox, rotate_frame, f, bary);
      }
    }
  });

  handle_zero_alpha_pixels_for_filled_faces(image);
  if (feather_texture) apply_feathering(image);
  image.write_file(image_name);
  nooutput = true;
  if (1 && !k_debug) hh_clean_up(), exit_immediately(0);  // Avoid ~Mesh().  We lose running timers though.
}

void do_write_primal_texture(Args& args) {
  const string image_name = args.get_filename();
  assertx(signal_ != "");
  internal_remesh();
  // Note that the "primal" remesh vertices do not correspond to the "dual" sampling of texture-mapping hardware.
  HH_TIMER("_write_primal_texture");
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
  const Bbox bbox{transform(g_mesh.vertices(), [&](Vertex v) { return g_mesh.point(v); })};
  const Frame rotate_frame = get_rotate_frame();
  for (Vertex v : g_mesh.ordered_vertices()) {  // Ordered so as to "consistently" break ties along domain edges.
    const Uv uv = v_imageuv(v);
    const int y0 = int(uv[1] * scale + .5f), x = int(uv[0] * scale + .5f);  // Primal sampling.
    // We flip the image vertically because the OpenGL Uv coordinate origin is at the image lower-left.
    const int yy = image.ysize() - 1 - y0;
    Pixel pixel;
    pixel[3] = 255;
    switch (signal_[0]) {
      case 'G': {
        const Point& p = g_mesh.point(v);
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
      case 'T': assertnever("Unsupported");
      default: assertnever("signal '" + signal_ + "' not recognized");
    }
    if (x == image.xsize()) {
      // Ignore last column; verify later that it equals first column.
      image_right_column[yy][0] = pixel;
    } else {
      image[yy][x] = pixel;
    }
  }
  if (!checkern) {
    for_int(y, image.ysize()) {
      const Pixel& pixel = image_right_column[y][0];
      if (pixel != background) assertw(pixel == image[y][0]);
    }
  }
  image.write_file(image_name);
  nooutput = true;
}

void do_write_lonlat_texture(Args& args) {
  const string image_name = args.get_filename();
  assertx(gridn);
  assertx(param_file != "");
  assertx(signal_ != "");
  assertx(g_mesh.empty());
  const Frame rotate_frame = get_rotate_frame();
  GMesh param_mesh = read_sphparam_mesh(param_file);
  for (Vertex v : param_mesh.vertices()) {
    v_domainp(v) = param_mesh.point(v);
    param_mesh.set_point(v, v_sph(v));
  }
  const Bbox bbox{transform(param_mesh.vertices(), [&](Vertex v) { return param_mesh.point(v); })};
  switch (signal_[0]) {
    case 'G': break;
    case 'N':
      for (Vertex v : param_mesh.vertices()) {
        Vector& normal = v_normal(v);
        if (!parse_key_vec(param_mesh.get_string(v), "normal", normal)) normal = k_undefined_vector;
      }
      break;
    case 'C':
      for (Vertex v : param_mesh.vertices()) {
        Vector& rgb = v_rgb(v);
        if (!parse_key_vec(param_mesh.get_string(v), "rgb", rgb)) rgb = k_undefined_vector;
      }
      break;
    case 'T': break;
    default: assertnever("signal '" + signal_ + "' not recognized");
  }

  const MeshSearch mesh_search(param_mesh, {true});

  const int imagesize = gridn;
  Image image(V(imagesize, imagesize));

  const int num_threads = get_max_threads();
  parallel_for_chunk(range(image.ysize()), num_threads, [&](const int thread_index, auto subrange) {
    dummy_use(thread_index);
    Face hint_f = nullptr;
    for (const int y : subrange) {
      for_int(x, image.xsize()) {
        // We flip the image vertically because the OpenGL Uv coordinate origin is at the image lower-left.
        const int yy = image.ysize() - 1 - y;
        Pixel& pixel = image[yy][x];
        const Uv lonlat((x + .5f) / image.xsize(), (y + .5f) / image.ysize());  // Dual sampling.
        const Point sph = sph_from_lonlat(lonlat);
        auto [f, bary] = mesh_search.search_on_sphere(sph, hint_f);
        hint_f = f;
        pixel = assign_signal(param_mesh, bbox, rotate_frame, f, bary);
      }
    }
  });

  handle_zero_alpha_pixels_for_filled_faces(image);
  image.write_file(image_name);
  nooutput = true;
}

// Tessellate a single spherical triangle.
void generate_tess(const Vec3<Point>& triangle, int n, TriangleSpheremap triangle_map) {
  for_int(i, 3) assertx(is_unit(triangle[i]));
  Matrix<Vertex> verts(n + 1, n + 1);  // Upper-left half is defined.
  for_int(i, n + 1) for_int(j, n - i + 1) {
    Vertex v = g_mesh.create_vertex();
    v_normal(v) = k_undefined_vector;
    v_rgb(v) = k_undefined_vector;
    verts[i][j] = v;
    v_imageuv(v) = Uv(i / float(n) + j / float(n) * .5f, j / float(n) * sqrt(3.f) / 2.f);
    const Bary bary((n - i - j) / float(n), i / float(n), j / float(n));
    const Point p = triangle_map(triangle, bary);
    assertx(is_unit(p));
    g_mesh.set_point(v, p);
    v_sph(v) = p;
  }
  for_int(i, n) for_int(j, n - i) {
    if (1) g_mesh.create_face(verts[i + 0][j + 0], verts[i + 1][j + 0], verts[i + 0][j + 1]);
    if (i) g_mesh.create_face(verts[i + 0][j + 0], verts[i + 0][j + 1], verts[i - 1][j + 1]);
  }
}

void do_triface1() {
  assertx(gridn > 0);
  assertx(scheme[0] == 'T');
  const TriangleSpheremap triangle_map = get_map(scheme.substr(1));
  const Point refp1(0.f, 1.f, 0.f), refp2(-1.f, 0.f, 0.f), refp3 = normalized(Point(-.2f, -.8f, .4f));
  generate_tess(V(refp1, refp2, refp3), gridn, triangle_map);
}

void do_test_properties() {
  assertx(scheme[0] == 'T');
  const TriangleSpheremap triangle_map = get_map(scheme.substr(1));
  if (!gridn) gridn = 16;
  //
  const Point refp1(0.f, 1.f, 0.f), refp2(-1.f, 0.f, 0.f);
  const Point refp3 = normalized(Point(-.3f, 0.f, .9f)), refp4 = normalized(Point(.4f, -.8f, .3f));
  //
  assertx(g_mesh.empty());
  const SGrid<Point, 3, 3> faces = {
      {refp1, refp2, refp3},
      {refp3, refp2, refp4},  // Shares the same boundary vertices iff edge-continuous. (check visually).
      {refp2, refp3, refp1}   // This rotated first face is identical iff symmetric. (check visually).
  };
  for_int(domainf, faces.num()) {
    Vec3<Point> triangle;
    for_int(k, 3) triangle[k] = normalized(faces[domainf][k]);
    generate_tess(triangle, gridn, triangle_map);
  }
  {
    Point po;
    dummy_init(po);
    for_int(i, gridn) {
      const Bary bary((gridn - i) / float(gridn), i / float(gridn), 0.f);
      const Point p = triangle_map(faces[0], bary);
      if (i) HH_SSTAT(Sbndlen, dist(p, po));
      // These lengths are identical iff arc-length boundaries.
      po = p;
    }
  }
  for_int(i, 3) {
    Bary bary{};
    bary[i] = 1.f;
    const Point p0 = triangle_map(faces[0], bary);
    const float eps = 1e-7f;
    bary[i] -= eps;
    bary[mod3(i + 1)] += eps;
    const Point p1 = triangle_map(faces[0], bary);
    SHOW(dist(p0, p1));
    // This length will be abnormally large iff unbounded derivatives.
  }
  nooutput = true;
}

void do_create_lonlat_sphere() {
  assertx(g_mesh.empty());
  assertx(gridn >= 2);
  Matrix<Vertex> matv(gridn, gridn);
  string str;
  for_int(i, gridn) for_int(j, gridn) {
    Vertex v = g_mesh.create_vertex();
    matv[i][j] = v;
    // We reverse the index i to obtain the correct outward orientation of the sphere.
    const Uv lonlat(j / (gridn - 1.f), (gridn - 1 - i) / (gridn - 1.f));
    const Point sph = sph_from_lonlat(lonlat);
    g_mesh.set_point(v, sph);
    v_sph(v) = sph;
    v_normal(v) = sph;
    v_rgb(v) = k_undefined_vector;
    g_mesh.update_string(v, "uv", csform_vec(str, lonlat));
  }
  for_int(i, gridn - 1) for_int(j, gridn - 1) {
    if (0) {
      g_mesh.create_face(V(matv[i + 0][j + 0], matv[i + 1][j + 0], matv[i + 1][j + 1], matv[i + 0][j + 1]));
    } else {
      g_mesh.create_face(V(matv[i + 0][j + 0], matv[i + 1][j + 0], matv[i + 1][j + 1]));
      g_mesh.create_face(V(matv[i + 0][j + 0], matv[i + 1][j + 1], matv[i + 0][j + 1]));
    }
  }
}

// Create a texture image parameterized by (longitude, latitude) that shows the domain grid samples mapped
// onto the sphere.
void do_create_lonlat_checker(Args& args) {
  int imagesize = args.get_int();
  assertx(imagesize > 1);
  string image_name = args.get_filename();
  //
  assertx(checkern);
  if (!gridn) gridn = 128;
  {
    HH_TIMER("_create_tess2");
    if (g_mesh.empty()) {
      create(true);
    } else {
      if (domain == "cube") triangulate_short_diag();
    }
  }
  MeshSearch::Options options;
  options.allow_off_surface = true;
  const MeshSearch mesh_search(g_mesh, options);
  const Array<DomainFace> domain_faces = get_domain_faces();
  {
    HH_TIMER("_create_image");
    Image image(V(imagesize, imagesize));
    const int num_threads = get_max_threads();
    parallel_for_chunk(range(image.ysize()), num_threads, [&](const int thread_index, auto subrange) {
      dummy_use(thread_index);
      Face hint_f = nullptr;
      for (const int y : subrange) {
        for_int(x, image.xsize()) {
          // We flip the image vertically because the OpenGL Uv coordinate origin is at the image lower-left.
          const int yy = image.ysize() - 1 - y;
          Pixel& pixel = image[yy][x];
          const Uv lonlat((x + .5f) / image.xsize(), (y + .5f) / image.ysize());
          const Point sph = sph_from_lonlat(lonlat);
          // Because `g_mesh` has disjoint components, we get warning "assertw(f2)" in gnomonic_search_bary() for
          // a tiny fraction of pixels.
          auto [f, bary] = mesh_search.search_on_sphere(sph, hint_f);
          hint_f = f;
          const Vec3<Vertex> va = g_mesh.triangle_vertices(f);
          Uv uv{};
          for_int(i, 3) uv += bary[i] * v_ijuv(va[i]);
          // Mesh vertices have colors according to checkern; for precision, we ignore these and recompute using uv.
          const int domainf = f_domainf(f);
          pixel = get_color(domain_faces[domainf], uv);
        }
      }
    });
    image.write_file(image_name);
  }
  nooutput = true;
}

void add_mesh_strings() {
  // Use HashFloat on sph and normal?
  string str;
  if (contains(key_names, "domainp"))
    for (Vertex v : g_mesh.vertices()) g_mesh.update_string(v, "domainp", csform_vec(str, v_domainp(v)));
  if (contains(key_names, "stretchuv"))
    for (Vertex v : g_mesh.vertices()) g_mesh.update_string(v, "stretchuv", csform_vec(str, v_stretchuv(v)));
  if (contains(key_names, "imageuv"))
    for (Vertex v : g_mesh.vertices()) g_mesh.update_string(v, "imageuv", csform_vec(str, v_imageuv(v)));
  if (contains(key_names, "sph"))
    for (Vertex v : g_mesh.vertices()) g_mesh.update_string(v, "sph", csform_vec(str, v_sph(v)));
  if (contains(key_names, "ll"))
    for (Vertex v : g_mesh.vertices()) {
      const Point& sph = v_sph(v);
      Uv lonlat = lonlat_from_sph(sph);
      const bool near_prime_meridian = abs(sph[0]) < 1e-5f && sph[1] > 1e-5f;
      if (near_prime_meridian) {
        Face f = g_mesh.most_ccw_face(v);  // Actually, any adjacent face in this connected component.
        const Vec3<Point> sphs = map(g_mesh.triangle_vertices(f), v_sph);
        const Point center = mean(sphs);
        lonlat[0] = center[0] < 0.f ? 0.f : 1.f;
      }
      g_mesh.update_string(v, "ll", csform_vec(str, lonlat));
    }
  if (1) {
    for (Vertex v : g_mesh.vertices()) {
      if (v_normal(v) != k_undefined_vector) g_mesh.update_string(v, "normal", csform_vec(str, v_normal(v)));
      if (v_rgb(v) != k_undefined_vector) g_mesh.update_string(v, "rgb", csform_vec(str, v_rgb(v)));
    }
  }
  // Note: "domaincorner" was handled earlier.
  if (contains(key_names, "domainf"))
    for (Face f : g_mesh.faces()) g_mesh.update_string(f, "domainf", csform(str, "%d", f_domainf(f)));
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
  HH_ARGSF(omit_faces, ": let remesh have only vertices (faster write_primal_texture)");
  HH_ARGSD(mesh_sphere, ": map domain grid onto sphere");
  HH_ARGSD(load_remesh, "mesh.remesh.m : load a previous remesh");
  HH_ARGSD(load_map, "domain.uv.sphparam.m : domain -> sphere, instead of scheme");
  HH_ARGSD(sample_map, "domain.uv.sphparam.m : domain -> sphere, sampled using gridn");
  HH_ARGSP(domain_file, "mesh.uv.sphparam.m : specify domain-to-sphere map (D->S)");
  HH_ARGSP(param_file, "mesh.sphparam.m : specify surface parameterization (M->S)");
  HH_ARGSP(rotate_s3d, "file.s3d : rotate normals based on view (snapped to axes)");
  HH_ARGSD(keys, "comma_separated_keys : fields to write in output mesh (default: sph)");
  HH_ARGSC("   possible keys: domainp,stretchuv,imageuv,sph,ll,domaincorner,domainf");
  HH_ARGSD(remesh, ": resample mesh and triangulate");
  HH_ARGSD(signal, "ch : select G=geometry, N=normal, C=color, T=texture");
  HH_ARGSD(texture_file, "imagefile : source content for 'T' signal");
  HH_ARGSP(feather_texture, "bool : blend across texture border discontinuities");
  HH_ARGSD(write_texture, "image : resample signal as texturemap (dual sampling)");
  HH_ARGSD(write_primal_texture, "image : resample signal as texturemap (primal sampling)");
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
    g_mesh.write(std::cout);
    std::cout.flush();
  }
  if (!k_debug) exit_immediately(0);
  return 0;
}
