// -*- C++ -*-  Copyright (c) Microsoft Corp; see LICENSE.
#include "SphereMapper.h"

#include "libHh/Args.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"
#include "libHh/MeshSearch.h"
#include "libHh/PMesh.h"
#include "libHh/Set.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

string orig_mesh;
string orig_indices;
bool keep_uv = false;

GMesh mesh_uv;
Map<Corner, Uv> map_c_uv;  // From a sparse set of corners in mesh_uv to Uv coordinates.

// Read a rotation frame from the specified filename and snap its axes to the canonical axes.
Frame get_rotate_frame(const string& rotate_s3d) {
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

// Return the vertices of a unit-radius tetrahedron centered at the origin.
Vec4<Point> create_regular_tetrahedron() {
  // Create an initial tetrahedron with its first three vertices on the plane z = 0.
  const Vec4<Point> tetrav0 = V(Point(-1.f / sqrt(3.f), 0.f, 0.f), Point(sqrt(3.f) / 6.f, +.5f, 0.f),
                                Point(sqrt(3.f) / 6.f, -.5f, 0.f), Point(0.f, 0.f, sqrt(6.f) / 3));
  const Point center0 = tetrav0[3] / 4.f;
  assertx(dist(mean(tetrav0), center0) < 1e-6f);

  // Produce a final tetrahedron translated such that its centroid is at the origin.
  Vec4<Point> tetrav;
  for_int(i, 4) tetrav[i] = normalized(tetrav0[i] - center0);
  return tetrav;
}

// Create the spherical parameterization for the current `pmi` mesh (presumably the base mesh).
Array<Point> get_base_sphmap(const PMeshIter& pmi, const string& base_param_scheme, const string& mesh_for_base) {
  Array<Point> base_sphmap(pmi._vertices.num());

  if (base_param_scheme == "tetra") {  // Initialize the spherical map using a regular tetrahedron.
    assertx(pmi._vertices.num() == 4 && pmi._faces.num() == 4);
    const Vec4<Point> tetrav = create_regular_tetrahedron();
    const int f0 = 0;
    const int v0 = pmi.face_vertices(f0)[0];
    int i = 0;
    base_sphmap[v0] = tetrav[i++];
    for (const auto& [vv, unused_ff] : pmi.ccw_vertices(v0, f0)) base_sphmap[vv] = tetrav[i++];

  } else if (base_param_scheme == "projection") {  // Use spherical projection.
    Array<Point> points;
    for_int(v, pmi._vertices.num()) points.push(pmi._vertices[v].attrib.point);
    const Point center = mean(points);
    for_int(v, pmi._vertices.num()) base_sphmap[v] = normalized(points[v] - center);

  } else if (base_param_scheme == "uv") {  // Use the PMesh uv (lon, lat) values.
    for_int(w, pmi._wedges.num()) {
      const Uv lonlat = pmi._wedges[w].attrib.uv;
      const Point sph = sph_from_lonlat(lonlat);
      const int v = pmi._wedges[w].vertex;
      base_sphmap[v] = sph;
    }

  } else if (base_param_scheme == "coord") {  // Use the mesh vertex positions, which must be on the sphere.
    for_int(w, pmi._wedges.num()) {
      const int v = pmi._wedges[w].vertex;
      base_sphmap[v] = pmi._vertices[v].attrib.point;
      assertx(is_unit(base_sphmap[v]));
    }

  } else if (base_param_scheme == "mesh") {  // Use the 'sph' keys in the mesh file named by mesh_for_base.
    assertx(mesh_for_base != "");
    RFile fi2(mesh_for_base);
    for (string line; fi2().peek() == '#';) {
      assertx(my_getline(fi2(), line));
      if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
    }
    GMesh mesh;
    mesh.read(fi2());
    assertx(mesh.num_vertices() == pmi._vertices.num());
    for (Vertex v : mesh.vertices()) {
      const int vi = mesh.vertex_id(v) - 1;
      assertx(vi < mesh.num_vertices());
      assertx(parse_key_vec(mesh.get_string(v), "sph", base_sphmap[vi]));
      base_sphmap[vi] = normalized(base_sphmap[vi]);
    }

  } else {
    assertnever("The parameter base_param_scheme '" + base_param_scheme + "' is unrecognized.");
  }
  return base_sphmap;
}

// *** Mesh output.

constexpr int k_axis0 = 0;  // Axis whose zero value defines the plane of the prime meridian.
constexpr int k_axis1 = 1;  // Axis whose positive range defines the halfspace containing the prime meridian.

constexpr float k_uv_undefined = -1.f;

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Point, v_sph);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Uv, v_uv);

Vertex split_mesh_edge(GMesh& mesh, Edge e, float frac1) {
  Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e), vs1 = mesh.side_vertex1(e), vs2 = mesh.side_vertex2(e);
  Face f1 = mesh.face1(e), f2 = mesh.face2(e);
  assertx(vs2 && f2);
  Vector nor_v1f1, nor_v2f1, nor_v1f2, nor_v2f2, nor_vs1f1, nor_vs2f2;
  int num_nors = (int(mesh.parse_corner_key_vec(mesh.corner(v1, f1), "normal", nor_v1f1)) +
                  int(mesh.parse_corner_key_vec(mesh.corner(v2, f1), "normal", nor_v2f1)) +
                  int(mesh.parse_corner_key_vec(mesh.corner(v1, f2), "normal", nor_v1f2)) +
                  int(mesh.parse_corner_key_vec(mesh.corner(v2, f2), "normal", nor_v2f2)) +
                  int(mesh.parse_corner_key_vec(mesh.corner(vs1, f1), "normal", nor_vs1f1)) +
                  int(mesh.parse_corner_key_vec(mesh.corner(vs2, f2), "normal", nor_vs2f2)));
  assertx(num_nors == 0 || num_nors == 6);
  Vertex v = mesh.split_edge(e);
  mesh.set_point(v, interp(mesh.point(v1), mesh.point(v2), frac1));
  if (num_nors) {
    string str;
    if (nor_v1f1 == nor_v1f2 && nor_v2f1 == nor_v2f2) {
      const char* s_vf = csform_vec(str, normalized(nor_v1f1 * frac1 + nor_v2f1 * (1.f - frac1)));
      mesh.update_string(v, "normal", s_vf);
    } else {
      const char* s_vf1 = csform_vec(str, normalized(nor_v1f1 * frac1 + nor_v2f1 * (1.f - frac1)));
      mesh.update_string(mesh.ccw_corner(v, mesh.edge(v, vs1)), "normal", s_vf1);
      mesh.update_string(mesh.clw_corner(v, mesh.edge(v, vs1)), "normal", s_vf1);
      const char* s_vf2 = csform_vec(str, normalized(nor_v1f2 * frac1 + nor_v2f2 * (1.f - frac1)));
      mesh.update_string(mesh.ccw_corner(v, mesh.edge(v, vs2)), "normal", s_vf2);
      mesh.update_string(mesh.clw_corner(v, mesh.edge(v, vs2)), "normal", s_vf2);
    }
    if (!GMesh::string_has_key(mesh.get_string(v1), "normal")) {
      mesh.update_string(mesh.ccw_corner(v1, mesh.edge(v1, v)), "normal", csform_vec(str, nor_v1f1));
      mesh.update_string(mesh.clw_corner(v1, mesh.edge(v1, v)), "normal", csform_vec(str, nor_v1f2));
    }
    if (!GMesh::string_has_key(mesh.get_string(v2), "normal")) {
      mesh.update_string(mesh.clw_corner(v2, mesh.edge(v2, v)), "normal", csform_vec(str, nor_v2f1));
      mesh.update_string(mesh.ccw_corner(v2, mesh.edge(v2, v)), "normal", csform_vec(str, nor_v2f2));
    }
    if (!GMesh::string_has_key(mesh.get_string(vs1), "normal")) {
      mesh.update_string(mesh.ccw_corner(vs1, mesh.edge(vs1, v)), "normal", csform_vec(str, nor_vs1f1));
      mesh.update_string(mesh.clw_corner(vs1, mesh.edge(vs1, v)), "normal", csform_vec(str, nor_vs1f1));
    }
    if (!GMesh::string_has_key(mesh.get_string(vs2), "normal")) {
      mesh.update_string(mesh.ccw_corner(vs2, mesh.edge(vs2, v)), "normal", csform_vec(str, nor_vs2f2));
      mesh.update_string(mesh.clw_corner(vs2, mesh.edge(vs2, v)), "normal", csform_vec(str, nor_vs2f2));
    }
  }
  return v;
}

// Split some `mesh` edges (by adding new vertices) such that no triangle face straddles the prime meridian.
void split_mesh_along_prime_meridian(GMesh& mesh) {
  Array<Vertex> new_vertices;

  const auto split_edge = [&](Edge e, int axis) {
    Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
    const Point sph1 = v_sph(v1), sph2 = v_sph(v2);
    const float sph_frac1 = sph1[axis] / (sph1[axis] - sph2[axis]);
    const Point sph_new = snap_coordinates(normalized((1.f - sph_frac1) * sph1 + sph_frac1 * sph2));
    const float frac1 = angle_between_unit_vectors(sph_new, sph2) / angle_between_unit_vectors(sph1, sph2);
    Vertex v = split_mesh_edge(mesh, e, frac1);
    new_vertices.push(v);
    v_sph(v) = sph_new;
  };

  // Split the mesh edges on faces overlapping the zero meridian (along the plane x = 0, in halfspace y > 0).
  const float eps = 1e-5f;
  Set<Edge> edges_to_split;
  for (Face f : mesh.faces()) {
    const Vec3<Point> sphs = map(mesh.triangle_vertices(f), [&](Vertex v) { return v_sph(v); });
    const Bbox bbox{sphs};
    const bool face_overlaps_meridian = bbox[0][k_axis0] < -eps && bbox[1][k_axis0] > eps && bbox[1][k_axis1] > eps;
    if (face_overlaps_meridian)
      for (Edge e : mesh.edges(f)) {
        const Vector sph1 = v_sph(mesh.vertex1(e)), sph2 = v_sph(mesh.vertex2(e));
        const bool edge_crosses_meridian =
            (sph1[k_axis0] < -eps && sph2[k_axis0] > eps) || (sph2[k_axis0] < -eps && sph1[k_axis0] > eps);
        if (edge_crosses_meridian) edges_to_split.add(e);
      }
  }
  for (Edge e : edges_to_split) split_edge(e, k_axis0);
  edges_to_split.clear();

  for (Vertex v : new_vertices) {
    for (;;) {
      bool modified = false;
      for (Edge e : mesh.edges(v)) {
        Vertex v2 = mesh.opp_vertex(v, e);
        if (dist(v_sph(v), v_sph(v2)) < 1e-5f) {
          Warning("Collapsing a zero-param-length edge adjacent to a newly introduced meridian vertex");
          mesh.collapse_edge_vertex(e, v);  // The just-introduced vertex v is kept.
          modified = true;
          break;
        }
      }
      if (!modified) break;
    }
  }

  if (0) {  // There seems to be no clean solution near the pole.
    // Split the (generally two) edges containing either of the two poles.
    for (Vertex new_vert : new_vertices) {
      for (Vertex vv : mesh.vertices(new_vert)) {
        if (abs(v_sph(vv)[k_axis0]) < eps && mesh.vertex_id(vv) < mesh.vertex_id(new_vert)) {
          const float val1 = v_sph(new_vert)[k_axis1], val2 = v_sph(vv)[k_axis1];
          if ((val1 < -eps && val2 > eps) || (val2 < -eps && val1 > eps)) edges_to_split.add(mesh.edge(new_vert, vv));
        }
      }
    }
    for (Edge e : edges_to_split) split_edge(e, k_axis1);
  }
}

void split_mesh_along_octa(GMesh& mesh) {
  Array<Vertex> new_vertices;

  const auto split_edge = [&](Edge e, int axis) {
    Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
    const Point sph1 = v_sph(v1), sph2 = v_sph(v2);
    const float sph_frac1 = sph1[axis] / (sph1[axis] - sph2[axis]);
    const Point sph_new = snap_coordinates(normalized((1.f - sph_frac1) * sph1 + sph_frac1 * sph2));
    const float frac1 = angle_between_unit_vectors(sph_new, sph2) / angle_between_unit_vectors(sph1, sph2);
    Vertex v = split_mesh_edge(mesh, e, frac1);
    new_vertices.push(v);
    v_sph(v) = sph_new;
  };

  const float eps = 1e-5f;
  for_int(axis, 3) {
    Set<Edge> edges_to_split;
    for (Edge e : mesh.edges()) {
      const Bbox bbox{V(v_sph(mesh.vertex1(e)), v_sph(mesh.vertex2(e)))};
      if (bbox[0][axis] < -eps && bbox[1][axis] > eps) edges_to_split.add(e);
    }
    for (Edge e : edges_to_split) split_edge(e, axis);
  }

  for (Vertex v : new_vertices) {
    for (;;) {
      bool modified = false;
      for (Edge e : mesh.edges(v)) {
        Vertex v2 = mesh.opp_vertex(v, e);
        if (dist(v_sph(v), v_sph(v2)) < 1e-5f) {
          Warning("Collapsing a zero-param-length edge adjacent to a newly introduced vertex");
          mesh.collapse_edge_vertex(e, v);  // The just-introduced vertex v is kept.
          modified = true;
          break;
        }
      }
      if (!modified) break;
    }
  }
}

template <typename Precision = double>
bool spherical_triangle_is_flipped(const Vec3<Point>& pt, float tolerance = 0.f) {
  // The signed volume of the tetrahedron formed by the origin and the points p1, p2, and p3 is given by
  //  (1.f/6.f) * dot(p1, cross(p2, p3).
  // return dot(pt[0], cross(pt[1], pt[2])) < 0.f;
  const Point& p1 = pt[1];
  const Point& p2 = pt[2];
  Vec3<Precision> vcross(Precision(p1[1]) * p2[2] - Precision(p1[2]) * p2[1],
                         Precision(p1[2]) * p2[0] - Precision(p1[0]) * p2[2],
                         Precision(p1[0]) * p2[1] - Precision(p1[1]) * p2[0]);
  return dot<Precision>(pt[0], vcross) < -tolerance;
}

// Next two functions copied from SphereSample.cpp ??

// Given the point p on the unit sphere and a spherical triangle pt assumed to enclose it, return the barycentric
// coordinates of the spherical projection of p onto the triangle.
Bary gnomonic_get_bary(const Point& p, const Vec3<Point>& pt) {
  // Compute the Intersection `pint` of the segment (origin, p) with the triangle.
  Polygon poly(3);
  for_int(i, 3) poly[i] = pt[i];
  const Point origin(0.f, 0.f, 0.f);
  const Point further_p = p * 1.01f;  // Extend the segment to remove any ambiguity at its endpoint.
  Point pint;
  if (!poly.intersect_segment(origin, further_p, pint)) {
    if (0) Warning("widening polygon");
    const bool debug = false;
    if (debug) SHOW(p, further_p);
    if (debug) SHOW(poly, spherical_triangle_area(poly));
    const Polygon poly_bu = poly;
    widen_triangle(poly, 1e-3f);  // Larger due to nonzero `tolerance` below.
    if (debug) SHOW(poly);
    if (!poly.intersect_segment(origin, further_p, pint)) {
      Warning("intersect_segment failed again");
      if (1) SHOW_PRECISE(p, further_p, poly_bu, poly);
      // In this unexpected worst case, fall back to computing the closest point.
      Bary bary;
      project_point_triangle2(p, pt[0], pt[1], pt[2], bary, pint);
    }
  }
  // Compute the barycentric coordinates of the intersection point `pint`.
  Bary bary;
  Point clp;
  const float d2 = project_point_triangle2(pint, pt[0], pt[1], pt[2], bary, clp);
  assertw(d2 < 1e-10f);
  return bary;
}

// Given point p on the unit sphere, and some "nearby" spherical triangle f in mesh2, find the actual spherical
// triangle f containing p, and the barycentric coordinates of the spherical projection of p onto f.
void gnomonic_search_bary(const Point& p, const GMesh& mesh2, Face& f, Bary& bary, float tolerance = 0.f) {
  Vec3<Point> pt;
  {
    // Find the spherical triangle f (with vertex points pt) containing p.
    int nfchanges = 0;
    for (;;) {
      pt = mesh2.triangle_points(f);
      // Adapted from MeshSearch.cpp .
      Vec3<bool> outside;
      for_int(i, 3) outside[i] = spherical_triangle_is_flipped(V(p, pt[mod3(i + 1)], pt[mod3(i + 2)]), tolerance);
      int num_outside = sum<int>(outside);
      if (num_outside == 0) break;
      const Vec3<Vertex> va = mesh2.triangle_vertices(f);
      if (num_outside == 2) {
        const int side = index(outside, false);
        // Fastest: jump across the vertex.
        Vertex v = va[side];
        const int val = mesh2.degree(v);
        // const int nrot = ((val - 1) / 2) + (Random::G.unif() < 0.5f);  // Ideal, but Random is not thread-safe.
        constexpr auto pseudo_randoms = V(0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 0, 1, 0, 1);
        const int nrot = ((val - 1) / 2) + pseudo_randoms[nfchanges % pseudo_randoms.Num];
        for_int(i, nrot) f = assertx(mesh2.ccw_face(v, f));
      } else if (num_outside == 1) {
        const int side = index(outside, true);
        Face f2 = mesh2.opp_face(va[side], f);
        if (!assertw(f2)) break;
        f = f2;
      } else {
        assertnever("");
      }
      nfchanges++;
      assertx(nfchanges < 200);
    }
    HH_SSTAT(Snfchanges, nfchanges);
  }
  bary = gnomonic_get_bary(p, pt);
}

Uv snap_uv(Uv uv) {
  for_int(c, 2) if (abs(uv[c]) < 1e-8f) uv[c] = 0.f;
  return uv;
}

template <int n> Vec<float, n> normalized_double(const Vec<float, n>& vec) {
  double val = assertx(mag<double>(vec));
  return vec / float(val);
}

// Given a GMesh, add "sph" and "uv" strings, optionally split its domain discontinuities, and write it to std::cout.
void write_parameterized_gmesh(GMesh& gmesh, bool split_meridian) {
  assertx(!(split_meridian && !mesh_uv.empty()));
  if (split_meridian) split_mesh_along_prime_meridian(gmesh);
  if (!mesh_uv.empty()) {
    if (0)
      for (Face f : mesh_uv.faces()) assertx(!spherical_triangle_is_flipped(mesh_uv.triangle_points(f)));
    split_mesh_along_octa(gmesh);  // TODO: instead, use mesh_uv "uv" discontinuities.
  }
  MeshSearch::Options options;
  options.allow_off_surface = true;
  const MeshSearch msearch(mesh_uv, options);

  const int num_threads = get_max_threads();
  parallel_for_chunk(Array<Vertex>{gmesh.vertices()}, num_threads, [&](const int thread_index, auto subrange) {
    dummy_use(thread_index);
    string str;
    Face hintf = nullptr;
    for (Vertex v : subrange) {
      const Point& sph = v_sph(v);
      gmesh.update_string(v, "sph", csform_vec(str, sph));
      if (keep_uv) {
      } else if (!mesh_uv.empty()) {
        const auto get_uvs = [&](Face f) {
          return map(mesh_uv.triangle_corners(f), [&](Corner c) {
            Vertex vv = mesh_uv.corner_vertex(c);
            return v_uv(vv)[0] != k_uv_undefined ? v_uv(vv) : map_c_uv.get(c);
          });
        };
        // const bool near_a_cut = any_of(sph, [](float v) { return abs(v) < 1e-5f; });
        const bool near_a_cut = (abs(sph[1]) < 1e-5f || abs(sph[2]) < 1e-5f) && sph[0] > -1e-5f;
        if (!near_a_cut) {
          auto [f, bary, unused_clp, unused_d2] = msearch.search(sph, hintf);
          gnomonic_search_bary(sph, mesh_uv, f, bary);  // May modify f.
          hintf = f;
          const Vec3<Uv> uvs = get_uvs(f);
          const Uv uv = snap_uv(interp(uvs[0], uvs[1], uvs[2], bary));
          gmesh.update_string(v, "uv", csform_vec(str, uv));
        } else {
          gmesh.update_string(v, "uv", nullptr);
          for (Corner c : gmesh.corners(v)) {
            Face gf = gmesh.corner_face(c);
            const Vec3<Point> sphs = map(gmesh.triangle_vertices(gf), [&](Vertex v2) { return v_sph(v2); });
            // Compute a sphere point that is perturbed slightly toward the centroid of the adjacent face, to find
            // an initial face f on the correct side of the parametric uv discontinuity.
            const Point sph_center = mean(sphs);
            const Point sph_perturbed = normalized_double(sph + normalized_double(sph_center - sph) * 5e-4f);
            hintf = nullptr;  // ??
            auto [f, bary, unused_clp, unused_d2] = msearch.search(sph_perturbed, hintf);
            gnomonic_search_bary(sph_perturbed, mesh_uv, f, bary);  // May modify f.
            // Now use the obtained face f but search for the unperturbed sph and use some nonzero tolerance to
            // hopefully avoid crossing over to the wrong side of the parametric uv discontinuity.
            const float tolerance = 1e-7f;
            gnomonic_search_bary(sph, mesh_uv, f, bary, tolerance);  // May modify f.
            hintf = f;
            const Vec3<Uv> uvs = get_uvs(f);
            const Uv uv = snap_uv(interp(uvs[0], uvs[1], uvs[2], bary));
            gmesh.update_string(c, "uv", csform_vec(str, uv));
          }
        }
      } else {  // Replace any "uv" with longitude-latitude.
        // Note the rendering challenges with lonlat parameterization: https://gamedev.stackexchange.com/a/197936.
        const Uv lonlat = lonlat_from_sph(sph);
        const bool near_prime_meridian = abs(sph[k_axis0]) < 1e-5f && sph[k_axis1] > -1e-5f;
        if (!near_prime_meridian) {
          // If not near the discontinuity on the +Y (k_axis1) axis.
          gmesh.update_string(v, "uv", csform_vec(str, lonlat));
        } else {
          // Tweak texture coordinates for correct rendering assuming split_mesh_along_prime_meridian has been called.
          gmesh.update_string(v, "uv", nullptr);
          for (Corner c : gmesh.corners(v)) {
            Face f = gmesh.corner_face(c);
            const Vec3<Point> sphs = map(gmesh.triangle_vertices(f), [&](Vertex v2) { return v_sph(v2); });
            const Point center = mean(sphs);
            const float lon2 = center[0] < 0.f ? 0.f : 1.f;
            gmesh.update_string(c, "uv", csform_vec(str, Uv(lon2, lonlat[1])));
          }
        }
      }
    }
  });
  hh_clean_up();
  gmesh.write(std::cout);
  if (!k_debug) exit_immediately(0);  // Skip GMesh destruction for speedup.
}

// Extract a GMesh from the progressive mesh iterator, add "sph" and "uv" strings, and write it to std::cout.
void write_parameterized_mesh(PMeshIter pmi, CArrayView<Point> sphmap, bool split_meridian) {
  GMesh gmesh = pmi.extract_gmesh();
  for (Vertex v : gmesh.vertices()) {
    gmesh.update_string(v, "wid", nullptr);
    v_sph(v) = sphmap[gmesh.vertex_id(v) - 1];
  }
  write_parameterized_gmesh(gmesh, split_meridian);
}

// Read the PM's original mesh and original_vertex_indices.txt, add "sph" and "uv" strings, and write it to std::cout.
void write_orig_mesh(PMeshIter pmi, CArrayView<Point> sphmap, bool split_meridian) {
  assertx(orig_mesh != "");
  assertx(orig_indices != "");
  Array<int> orig_vertex_indices;
  {
    RFile fi(orig_indices);
    string str;
    assertx(my_getline(fi(), str));
    const int num_vertices = to_int(str);
    assertx(num_vertices == pmi._vertices.num());
    for_int(vi, num_vertices) {
      assertx(my_getline(fi(), str));
      orig_vertex_indices.push(to_int(str));
    }
  }
  GMesh gmesh;
  {
    gmesh.read(RFile{orig_mesh}());
    for_int(vi, pmi._vertices.num()) {
      const int orig_vi = orig_vertex_indices[vi];  // (Index starting at 1.)
      Vertex v = gmesh.id_vertex(orig_vi);
      v_sph(v) = sphmap[vi];
    }
  }
  write_parameterized_gmesh(gmesh, split_meridian);
}

// *** PMesh output.

// Split the edges on faces overlapping the zero meridian, adding new vertices and triangles but no extra wedges.
void split_awmesh_faces_along_meridian(AWMesh& awmesh) {
  const float eps = 1e-5f;
  for (int f = 0; f < awmesh._faces.num(); f++) {  // Note that awmesh._faces grows within the loop.
    const Vec3<Point> sphs{map(V(0, 1, 2), [&](int j) {
      const int w = awmesh._faces[f].wedges[j];
      return sph_from_lonlat(awmesh._wedges[w].attrib.uv);
    })};
    const Bbox bbox{sphs};
    bool changed = false;
    const bool face_overlaps_meridian = bbox[0][k_axis0] < -eps && bbox[1][k_axis0] > eps && bbox[1][k_axis1] > eps;
    const bool face_is_adjacent_to_pole =
        bbox[0][k_axis0] < eps && bbox[1][k_axis0] > -eps && bbox[0][k_axis1] < -eps && bbox[1][k_axis1] > eps;
    if (face_overlaps_meridian) {
      for_int(j, 3) {
        const Point sph1 = sphs[j], sph2 = sphs[mod3(j + 1)];
        const bool edge_crosses_meridian =
            (sph1[k_axis0] < -eps && sph2[k_axis0] > eps) || (sph2[k_axis0] < -eps && sph1[k_axis0] > eps);
        if (edge_crosses_meridian) {
          const float sph_frac1 = sph1[k_axis0] / (sph1[k_axis0] - sph2[k_axis0]);
          const Point sph_new = snap_coordinates(normalized((1.f - sph_frac1) * sph1 + sph_frac1 * sph2));
          const float frac1 = angle_between_unit_vectors(sph_new, sph2) / angle_between_unit_vectors(sph1, sph2);
          awmesh.split_edge(f, j, frac1);
          awmesh._wedges.last().attrib.uv = lonlat_from_sph(sph_new);
          changed = true;
          break;
        }
      }
    } else if (face_is_adjacent_to_pole && 0) {  // There seems to be no clean solution near the pole.
      for_int(j, 3) {
        const Point sph1 = sphs[j], sph2 = sphs[mod3(j + 1)];
        const bool edge_crosses_pole =
            abs(sph1[k_axis0]) < eps && abs(sph2[k_axis0]) < eps && (sph1[k_axis1] < -eps && sph2[k_axis1] > eps);
        if (edge_crosses_pole) {
          const float sph_frac1 = sph1[k_axis1] / (sph1[k_axis1] - sph2[k_axis1]);
          const Point sph_new = snap_coordinates(normalized((1.f - sph_frac1) * sph1 + sph_frac1 * sph2));
          const float frac1 = angle_between_unit_vectors(sph_new, sph2) / angle_between_unit_vectors(sph1, sph2);
          awmesh.split_edge(f, j, frac1);
          awmesh._wedges.last().attrib.uv = lonlat_from_sph(sph_new);
          changed = true;
          break;
        }
      }
    }
    if (changed) f--;  // Repeat the loop with the current (modified) face.
  }
}

// Split the vertices along the zero meridian into pairs of wedges and reconnect the existing triangles.
void split_awmesh_vertices_along_meridian(AWMesh& awmesh) {
  assertx(awmesh._wedges.num() == awmesh._vertices.num());
  const float eps = 1e-5f;
  const Array<int> someface = awmesh.gather_someface();
  for_int(w, awmesh._wedges.num()) {
    const Point sph = sph_from_lonlat(awmesh._wedges[w].attrib.uv);
    const bool near_prime_meridian = abs(sph[k_axis0]) < eps && sph[k_axis1] > -eps;
    if (!near_prime_meridian) continue;
    const int v = awmesh._wedges[w].vertex;
    const int wnew = awmesh._wedges.add(1);
    awmesh._wedges[wnew].vertex = v;
    awmesh._wedges[wnew].attrib = awmesh._wedges[w].attrib;
    awmesh._wedges[w].attrib.uv[0] = 0.f;     // Leftmost longitude.
    awmesh._wedges[wnew].attrib.uv[0] = 1.f;  // Rightmost longitude.
    for (const int f : awmesh.ccw_faces(v, someface[v])) {
      const Vec3<Point> sphs{map(V(0, 1, 2), [&](int j) {
        const int w = awmesh._faces[f].wedges[j];
        return sph_from_lonlat(awmesh._wedges[w].attrib.uv);
      })};
      const Point center = mean(sphs);
      if (center[0] <= 0.f) continue;  // Keep current assignment of face to wedge w; else change it to wnew.
      const int j = index(awmesh._faces[f].wedges, w);
      awmesh._faces[f].wedges[j] = wnew;
    }
  }
}

// Split some edges (by adding new vertices/wedges) such that no triangle face straddles the prime meridian.
void split_awmesh_along_prime_meridian(AWMesh& awmesh) {
  assertx(awmesh._wedges.num() == awmesh._vertices.num());
  for_int(w, awmesh._wedges.num()) assertw(awmesh._wedges[w].vertex == w);  // Possibly we could exploit this.
  split_awmesh_faces_along_meridian(awmesh);
  split_awmesh_vertices_along_meridian(awmesh);
}

// Write a single-resolution progressive mesh, encoding the spherical parameterization as lat-lon uv coordinates.
void write_parameterized_pm(PMeshIter pmi, CArrayView<Point> sphmap, bool split_meridian) {
  dummy_use(split_meridian);
  PMeshInfo pminfo = pmi.rstream()._info;
  pminfo._has_uv = true;
  for_int(w, pmi._wedges.num()) {
    const int v = pmi._wedges[w].vertex;
    pmi._wedges[w].attrib.uv = lonlat_from_sph(sphmap[v]);
  }
  if (split_meridian) split_awmesh_along_prime_meridian(pmi);
  hh_clean_up();
  PMesh pmesh(std::move(pmi), pminfo);
  pmesh.write(std::cout);
}

// *** Ply output.

// Write a *.ply mesh file, encoding the spherical parameterization as lat-lon uv coordinates on corners.
void write_parameterized_ply(PMeshIter pmi, CArrayView<Point> sphmap, bool split_meridian) {
  dummy_use(split_meridian);
  PMeshInfo& pminfo = pmi.rstream()._info;
  pminfo._has_uv = true;
  for_int(w, pmi._wedges.num()) {
    const int v = pmi._wedges[w].vertex;
    pmi._wedges[w].attrib.uv = lonlat_from_sph(sphmap[v]);
  }
  if (split_meridian) split_awmesh_along_prime_meridian(pmi);
  hh_clean_up();
  const bool binary = true;
  pmi.write_ply(std::cout, pminfo, binary);
}

bool will_write_ply(const ParseArgs& args_) {
  ParseArgs args{Array<string>{"dummy_arg0"}};
  args.copy_parse(args_);
  Array<string> strings;
  while (args.num()) strings.push(args.get_string());
  return strings.num() > 0 && strings.last() == "ply";
}

}  // namespace

int main(int argc, const char** argv) {
  SphereMapper::Options options;
  string base_param_scheme = "tetra";
  string mesh_for_base;
  string rotate_s3d;
  bool no_rot_align = false;
  bool split_meridian = false;
  string uv_map;
  string to = "mesh";
  bool nooutput = false;

  ParseArgs args(argc, argv);
  HH_ARGSC("A progressive mesh is read from stdin or first arg.  Subsequent options are:");
  HH_ARGSP_O(verbose, "level : set output verbosity level (0..2)");
  HH_ARGSP_O(effort, "val : adjust optimization thoroughness (0..5)");
  HH_ARGSF_O(visualize, ": pipe interactive parameterization to G3dOGL viewer");
  HH_ARGSF_O(fix_base, ": freeze all vertices of the base mesh");
  HH_ARGSF_O(optimize_inverse, ": minimize reciprocal stretch sphere_dist/mesh_dist");
  HH_ARGSP_O(conformal_weight, "w : weight for regularization energy term");
  HH_ARGSP_O(hole_weight, "w : factor for faces marked with 'hole'");
  HH_ARGSF_O(respect_sharp_edges, ": for consistent parameterization");
  HH_ARGSF_O(flatten_to_x0, ": set surface coordinate x = 0 (for flat octahedron)");
  HH_ARGSP(base_param_scheme, "method : Base mesh map: tetra, projection, uv, coord, or mesh");
  HH_ARGSP(mesh_for_base, "mesh.m : To use with '-base_param_scheme mesh'");
  HH_ARGSP(rotate_s3d, "file.s3d : rotate spherical param using view (snapped to axes)");
  HH_ARGSF(no_rot_align, ": do not rotationally align map with face normals");
  HH_ARGSF(split_meridian, ": split mesh faces at prime meridian (zero lon)");
  HH_ARGSP(orig_mesh, "file.m : file containing the original mesh");
  HH_ARGSP(orig_indices, "file.txt : file containing the original vertex indices");
  HH_ARGSP(uv_map, "file.uv.sphparam.m : define uv from sph by mapping through inverse of a spherical map");
  HH_ARGSP(keep_uv, "bool : do not overwrite GMesh uv with lonlat coordinates");
  HH_ARGSP(to, "format : set mesh output (mesh, pm, ply, orig_mesh)");
  HH_ARGSF(nooutput, ": do not output parameterized mesh");
  HH_ARGSC("Examples:");
  HH_ARGSC("  SphereParam cow.pm -rot s3d/cow.s3d >cow.sphparam.m");
  HH_ARGSC("  SphereParam cow.pm -rot s3d/cow.s3d -visualize -nooutput");

  const string arg0 = args.num() ? args.peek_string() : "";
  if (ParseArgs::special_arg(arg0)) args.parse(), exit(0);
  if (will_write_ply(args)) my_setenv("NO_DIAGNOSTICS_IN_STDOUT", "1");
  string filename = "-";
  if (args.num() && (arg0 == "-" || arg0[0] != '-')) filename = args.get_filename();
  RFile fi(filename);
  for (string line; fi().peek() == '#';) {
    assertx(my_getline(fi(), line));
    if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
  }
  showff("%s", args.header().c_str());
  Timer timer("SphereParam");
  args.parse();

  const Frame rotate_frame = get_rotate_frame(rotate_s3d);
  if (uv_map != "") {
    mesh_uv.read(RFile{uv_map}());
    string str;
    for (Vertex v : mesh_uv.vertices()) {
      if (!parse_key_vec(mesh_uv.get_string(v), "uv", v_uv(v))) {
        v_uv(v) = twice(k_uv_undefined);
        for (Corner c : mesh_uv.corners(v)) {
          Uv uv;
          assertx(parse_key_vec(mesh_uv.get_string(c), "uv", uv));
          map_c_uv.enter(c, uv);
        }
      }
      Point sph;
      assertx(parse_key_vec(mesh_uv.get_string(v), "sph", sph));
      mesh_uv.set_point(v, sph);
    }
  }
  const Map<string, decltype(&write_parameterized_mesh)> output_formatters{
      {"mesh", &write_parameterized_mesh},
      {"pm", &write_parameterized_pm},
      {"ply", &write_parameterized_ply},
      {"orig_mesh", &write_orig_mesh},
  };
  const auto output_formatter = output_formatters.get(to);

  PMeshRStream pmrs(fi());
  PMeshIter pmi(pmrs);
  SphereMapper mapper(pmi, options);
  mapper.show_parameters();

  // Compute the spherical parameterization for the base mesh.
  if (mesh_for_base != "") base_param_scheme = "mesh";
  Array<Point> base_sphmap = get_base_sphmap(pmi, base_param_scheme, mesh_for_base);

  // Run a coarse-to-fine optimization to obtain the spherical parameterization for the full-res mesh.
  Array<Point> sphmap{mapper.compute(base_sphmap)};

  if (options.verbose >= 1) mapper.show_total_stretch();
  timer.terminate();
  hh_clean_up();

  if (!nooutput) {
    if (!options.fix_base && !no_rot_align) {
      const Frame frame = mapper.frame_aligning_sphmap_to_surface_normals() * rotate_frame;
      for (Point& sph : sphmap) sph *= frame;
    }
    output_formatter(std::move(pmi), sphmap, split_meridian);
  }
  return 0;
}
