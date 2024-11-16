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
    if (1) {
      showff("Header information from %s:\n", mesh_for_base.c_str());
      for (string line; fi2().peek() == '#';) {
        assertx(my_getline(fi2(), line));
        if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
      }
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

template <int n, bool need_normalize> class SaveSplitEdgeAttrib {
  using VecF = Vec<float, n>;

 public:
  SaveSplitEdgeAttrib(GMesh& mesh_, Edge e, const char* key_) : _mesh(mesh_), _key(key_) {
    _v1 = _mesh.vertex1(e), _v2 = _mesh.vertex2(e), _vs1 = _mesh.side_vertex1(e), _vs2 = _mesh.side_vertex2(e);
    Face f1 = _mesh.face1(e), f2 = _mesh.face2(e);
    assertx(_vs2 && f2);
    int num_defined = (int(_mesh.parse_corner_key_vec(_mesh.corner(_v1, f1), _key, _vec_v1f1)) +
                       int(_mesh.parse_corner_key_vec(_mesh.corner(_v2, f1), _key, _vec_v2f1)) +
                       int(_mesh.parse_corner_key_vec(_mesh.corner(_v1, f2), _key, _vec_v1f2)) +
                       int(_mesh.parse_corner_key_vec(_mesh.corner(_v2, f2), _key, _vec_v2f2)) +
                       int(_mesh.parse_corner_key_vec(_mesh.corner(_vs1, f1), _key, _vec_vs1f1)) +
                       int(_mesh.parse_corner_key_vec(_mesh.corner(_vs2, f2), _key, _vec_vs2f2)));
    assertx(num_defined == 0 || num_defined == 6);
    _is_defined = num_defined > 0;
  }
  void reintroduce_after_split_edge(Vertex v, float frac1) {
    if (!_is_defined) return;
    const auto norm = [](const VecF& vec) { return need_normalize ? normalized(vec) : vec; };
    string str;
    if (_vec_v1f1 == _vec_v1f2 && _vec_v2f1 == _vec_v2f2) {
      const char* s_vf = csform_vec(str, norm(interp(_vec_v1f1, _vec_v2f1, frac1)));
      _mesh.update_string(v, _key, s_vf);
    } else {
      const char* s_vf1 = csform_vec(str, norm(interp(_vec_v1f1, _vec_v2f1, frac1)));
      _mesh.update_string(_mesh.ccw_corner(v, _mesh.edge(v, _vs1)), _key, s_vf1);
      _mesh.update_string(_mesh.clw_corner(v, _mesh.edge(v, _vs1)), _key, s_vf1);
      const char* s_vf2 = csform_vec(str, norm(interp(_vec_v1f2, _vec_v2f2, frac1)));
      _mesh.update_string(_mesh.ccw_corner(v, _mesh.edge(v, _vs2)), _key, s_vf2);
      _mesh.update_string(_mesh.clw_corner(v, _mesh.edge(v, _vs2)), _key, s_vf2);
    }
    if (!GMesh::string_has_key(_mesh.get_string(_v1), _key)) {
      _mesh.update_string(_mesh.ccw_corner(_v1, _mesh.edge(_v1, v)), _key, csform_vec(str, _vec_v1f1));
      _mesh.update_string(_mesh.clw_corner(_v1, _mesh.edge(_v1, v)), _key, csform_vec(str, _vec_v1f2));
    }
    if (!GMesh::string_has_key(_mesh.get_string(_v2), _key)) {
      _mesh.update_string(_mesh.clw_corner(_v2, _mesh.edge(_v2, v)), _key, csform_vec(str, _vec_v2f1));
      _mesh.update_string(_mesh.ccw_corner(_v2, _mesh.edge(_v2, v)), _key, csform_vec(str, _vec_v2f2));
    }
    if (!GMesh::string_has_key(_mesh.get_string(_vs1), _key)) {
      _mesh.update_string(_mesh.ccw_corner(_vs1, _mesh.edge(_vs1, v)), _key, csform_vec(str, _vec_vs1f1));
      _mesh.update_string(_mesh.clw_corner(_vs1, _mesh.edge(_vs1, v)), _key, csform_vec(str, _vec_vs1f1));
    }
    if (!GMesh::string_has_key(_mesh.get_string(_vs2), _key)) {
      _mesh.update_string(_mesh.ccw_corner(_vs2, _mesh.edge(_vs2, v)), _key, csform_vec(str, _vec_vs2f2));
      _mesh.update_string(_mesh.clw_corner(_vs2, _mesh.edge(_vs2, v)), _key, csform_vec(str, _vec_vs2f2));
    }
  }

 private:
  GMesh& _mesh;
  const char* _key;
  Vertex _v1, _v2, _vs1, _vs2;
  bool _is_defined;
  VecF _vec_v1f1, _vec_v2f1, _vec_v1f2, _vec_v2f2, _vec_vs1f1, _vec_vs2f2;
};

Vertex split_mesh_edge(GMesh& mesh, Edge e, float frac1) {
  SaveSplitEdgeAttrib<3, true> save_normal(mesh, e, "normal");
  SaveSplitEdgeAttrib<2, false> save_uv(mesh, e, "uv");
  SaveSplitEdgeAttrib<3, false> save_rgb(mesh, e, "rgb");
  const Point newp = interp(mesh.point(mesh.vertex1(e)), mesh.point(mesh.vertex2(e)), frac1);
  Vertex v = mesh.split_edge(e);
  save_normal.reintroduce_after_split_edge(v, frac1);
  save_uv.reintroduce_after_split_edge(v, frac1);
  save_rgb.reintroduce_after_split_edge(v, frac1);
  mesh.set_point(v, newp);
  return v;
}

void collapse_zero_param_length_edges(GMesh& mesh, Set<Vertex>& new_vertices) {
  while (!new_vertices.empty()) {
    Vertex v = new_vertices.remove_one();
    for (;;) {
      bool modified = false;
      for (Edge e : mesh.edges(v)) {
        Vertex v2 = mesh.opp_vertex(v, e);
        if (dist(v_sph(v), v_sph(v2)) < 1e-5f) {
          if (!mesh.legal_edge_collapse(e)) {
            if (0) Warning("Skipping an illegal edge collapse of a zero-param-length edge");
          } else {
            if (0) Warning("Collapsing a zero-param-length edge adjacent to a newly introduced vertex");
            new_vertices.remove(v2);  // (In most cases, it is not present.)
            // The just-introduced vertex v is kept; vertex v2 is destroyed.
            mesh.collapse_edge_vertex_saving_attribs(e, v);
            modified = true;
            break;
          }
        }
      }
      if (!modified) break;
    }
  }
}

// Split some `mesh` edges (by adding new vertices) such that no triangle face straddles the prime meridian.
void split_mesh_along_prime_meridian(GMesh& mesh) {
  Set<Vertex> new_vertices;

  const auto split_edge = [&](Edge e, int axis) -> Vertex {
    Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
    const Point sph1 = v_sph(v1), sph2 = v_sph(v2);
    const float sph_frac1 = sph1[axis] / (sph1[axis] - sph2[axis]);
    const Point sph_new = snap_coordinates(normalized((1.f - sph_frac1) * sph1 + sph_frac1 * sph2));
    const float frac1 = angle_between_unit_vectors(sph_new, sph2) / angle_between_unit_vectors(sph1, sph2);
    Vertex v = split_mesh_edge(mesh, e, frac1);
    new_vertices.enter(v);
    v_sph(v) = sph_new;
    return v;
  };

  // Split the mesh edges on faces overlapping the zero meridian (along the plane x = 0, in halfspace y > 0).
  const float eps = 1e-5f;
  Set<Face> to_visit;
  for (Face f : mesh.faces()) to_visit.enter(f);
  while (!to_visit.empty()) {
    Face f = to_visit.remove_one();
    const Vec3<Point> sphs = map(mesh.triangle_vertices(f), v_sph);
    const Bbox bbox{sphs};
    const bool face_overlaps_meridian = bbox[0][k_axis0] < -eps && bbox[1][k_axis0] > eps && bbox[1][k_axis1] > eps;
    if (!face_overlaps_meridian) continue;
    for (Edge e : mesh.edges(f)) {
      const Vector sph1 = v_sph(mesh.vertex1(e)), sph2 = v_sph(mesh.vertex2(e));
      const bool edge_crosses_meridian = ((sph1[k_axis0] < -eps && sph2[k_axis0] > eps) ||  //
                                          (sph2[k_axis0] < -eps && sph1[k_axis0] > eps));
      if (!edge_crosses_meridian) continue;
      for (Face f2 : mesh.faces(e)) to_visit.remove(f2);
      Vertex v = split_edge(e, k_axis0);
      for (Face f2 : mesh.faces(v)) to_visit.enter(f2);
      break;
    }
  }

  collapse_zero_param_length_edges(mesh, new_vertices);
}

void split_mesh_along_octa(GMesh& mesh) {
  Set<Vertex> new_vertices;

  const auto split_edge = [&](Edge e, int axis) {
    Vertex v1 = mesh.vertex1(e), v2 = mesh.vertex2(e);
    const Point sph1 = v_sph(v1), sph2 = v_sph(v2);
    const float sph_frac1 = sph1[axis] / (sph1[axis] - sph2[axis]);
    const Point sph_new = snap_coordinates(normalized((1.f - sph_frac1) * sph1 + sph_frac1 * sph2));
    const float frac1 = angle_between_unit_vectors(sph_new, sph2) / angle_between_unit_vectors(sph1, sph2);
    Vertex v = split_mesh_edge(mesh, e, frac1);
    new_vertices.enter(v);
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

  collapse_zero_param_length_edges(mesh, new_vertices);
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
  assertx(!(split_meridian && !mesh_uv.empty()));  // (!mesh_uv.empty() uses split_mesh_along_octa() instead.)
  if (split_meridian) split_mesh_along_prime_meridian(gmesh);
  if (!mesh_uv.empty()) {
    if (0)
      for (Face f : mesh_uv.faces()) assertx(!spherical_triangle_is_flipped(mesh_uv.triangle_points(f)));
    split_mesh_along_octa(gmesh);
    // Ideally, we should use mesh_uv "uv" discontinuities to perform splitting; this would be required for
    // cube and tetra domains; also it would result in fewer cuts for octa and octaflat.
  }
  MeshSearch::Options options;
  options.allow_off_surface = true;
  const MeshSearch mesh_search(mesh_uv, options);

  const int num_threads = get_max_threads();
  parallel_for_chunk(Array<Vertex>{gmesh.vertices()}, num_threads, [&](const int thread_index, auto subrange) {
    dummy_use(thread_index);
    string str;
    Face hint_f = nullptr;
    for (Vertex v : subrange) {
      const Point& sph = v_sph(v);
      gmesh.update_string(v, "sph", csform_vec(str, sph));
      if (keep_uv) continue;
      gmesh.update_string(v, "Ouv", GMesh::string_key(str, gmesh.get_string(v), "uv"));
      gmesh.update_string(v, "uv", nullptr);
      for (Corner c : gmesh.corners(v)) {
        gmesh.update_string(c, "Ouv", GMesh::string_key(str, gmesh.get_string(c), "uv"));
        gmesh.update_string(c, "uv", nullptr);
      }
      if (!mesh_uv.empty()) {
        const auto get_uvs = [&](Face f) {
          return map(mesh_uv.triangle_corners(f), [&](Corner c) {
            Vertex vv = mesh_uv.corner_vertex(c);
            return v_uv(vv)[0] != k_uv_undefined ? v_uv(vv) : map_c_uv.get(c);
          });
        };
        // const bool near_a_cut = any_of(sph, [](float v) { return abs(v) < 1e-5f; });
        const bool near_a_cut = (abs(sph[1]) < 1e-5f || abs(sph[2]) < 1e-5f) && sph[0] > -1e-5f;
        if (!near_a_cut) {
          auto [f, bary] = mesh_search.search_on_sphere(sph, hint_f);
          hint_f = f;
          const Vec3<Uv> uvs = get_uvs(f);
          const Uv uv = snap_uv(interp(uvs, bary));
          gmesh.update_string(v, "uv", csform_vec(str, uv));
        } else {
          gmesh.update_string(v, "uv", nullptr);
          for (Corner c : gmesh.corners(v)) {
            Face gf = gmesh.corner_face(c);
            const Vec3<Point> sphs = map(gmesh.triangle_vertices(gf), v_sph);
            // Compute a sphere point that is perturbed slightly toward the centroid of the adjacent face, to find
            // an initial face f on the correct side of the parametric uv discontinuity.
            const Point sph_center = mean(sphs);
            const Point sph_perturbed = normalized_double(sph + normalized_double(sph_center - sph) * 5e-4f);
            auto [f, bary] = mesh_search.search_on_sphere(sph_perturbed, hint_f, &sph);
            hint_f = f;
            const Vec3<Uv> uvs = get_uvs(f);
            const Uv uv = snap_uv(interp(uvs, bary));
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
            const Vec3<Point> sphs = map(gmesh.triangle_vertices(f), v_sph);
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
        const int w2 = awmesh._faces[f].wedges[j];
        return sph_from_lonlat(awmesh._wedges[w2].attrib.uv);
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
  HH_ARGSF_O(wait_on_visualizer, ": exit only after user closes G3dOGL viewer");
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
  HH_ARGSP(uv_map, "file.uv.sphparam.m : define uv from sph using inverse of a spherical map");
  HH_ARGSP(keep_uv, "bool : do not overwrite GMesh uv with lonlat coordinates");
  HH_ARGSP(to, "format : set mesh output (mesh, pm, ply, orig_mesh)");
  HH_ARGSF(nooutput, ": do not output parameterized mesh");
  HH_ARGSC("Examples:");
  HH_ARGSC("  SphereParam cow.pm -rot s3d/cow.s3d >cow.sphparam.m");
  HH_ARGSC("  SphereParam cow.pm -rot s3d/cow.s3d -visualize -nooutput");

  const string arg0 = args.num() ? args.peek_string() : "";
  if (ParseArgs::special_arg(arg0)) args.parse(), exit(0);
  if (will_write_ply(args)) my_setenv("NO_DIAGNOSTICS_IN_STDOUT", "1");
  const string filename = args.num() && (arg0 == "-" || arg0[0] != '-') ? args.get_filename() : "-";
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
