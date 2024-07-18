// -*- C++ -*-  Copyright (c) Microsoft Corp; see LICENSE.
#include "SphereMapper.h"

#include "libHh/Args.h"
#include "libHh/FileIO.h"
#include "libHh/FrameIO.h"
#include "libHh/GMesh.h"
#include "libHh/GeomOp.h"
#include "libHh/PMesh.h"
#include "libHh/Set.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

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
      const UV lonlat = pmi._wedges[w].attrib.uv;
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

  } else if (base_param_scheme == "mesh") {  // Use the 'sph' keys in the mesh file named mesh_for_base.
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

HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Point, v_sph);

// Split some `mesh` edges (by adding new vertices) such that no triangle face straddles the prime meridian.
void split_mesh_along_prime_meridian(GMesh& mesh) {
  Array<Vertex> new_vertices;
  string str;

  auto split_edge = [&](Edge e, int axis) {
    const Point sph1 = v_sph(mesh.vertex1(e)), sph2 = v_sph(mesh.vertex2(e));
    const Point p1 = mesh.point(mesh.vertex1(e)), p2 = mesh.point(mesh.vertex2(e));
    Vector nor1, nor2;
    const bool have_nor = (parse_key_vec(mesh.get_string(mesh.vertex1(e)), "normal", nor1) &&
                           parse_key_vec(mesh.get_string(mesh.vertex2(e)), "normal", nor2));
    Vertex v = mesh.split_edge(e);
    new_vertices.push(v);
    const float sph_frac1 = sph1[axis] / (sph1[axis] - sph2[axis]);
    const Point sph_new = snap_coordinates(normalized((1.f - sph_frac1) * sph1 + sph_frac1 * sph2));
    const float frac1 = angle_between_unit_vectors(sph_new, sph2) / angle_between_unit_vectors(sph1, sph2);
    v_sph(v) = sph_new;
    mesh.set_point(v, interp(p1, p2, frac1));
    if (have_nor) {
      const Vector nor = normalized(nor1 * frac1 + nor2 * (1.f - frac1));
      mesh.update_string(v, "normal", csform_vec(str, nor));
    }
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
    for (Edge e : mesh.edges(v)) {
      Vertex v2 = mesh.opp_vertex(v, e);
      if (dist(v_sph(v), v_sph(v2)) < 1e-6f) {
        Warning("Collapsing a zero-param-length edge adjacent to a newly introduced meridian vertex");
        mesh.collapse_edge_vertex(e, v2);
        break;
      }
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

// Extract a GMesh from the progressive mesh iterator, add "sph" and "uv" strings, and write it to std::cout.
void write_parameterized_mesh(PMeshIter pmi, CArrayView<Point> sphmap, bool split_meridian) {
  GMesh gmesh = pmi.extract_gmesh();
  for (Vertex v : gmesh.vertices()) v_sph(v) = sphmap[gmesh.vertex_id(v) - 1];
  if (split_meridian) split_mesh_along_prime_meridian(gmesh);

  string str;
  for (Vertex v : gmesh.vertices()) {
    gmesh.update_string(v, "wid", nullptr);
    const Point& sph = v_sph(v);
    gmesh.update_string(v, "sph", csform_vec(str, sph));
    // Replace any "uv" with longitude-latitude.
    const UV lonlat = lonlat_from_sph(sph);
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
        gmesh.update_string(c, "uv", csform_vec(str, UV(lon2, lonlat[1])));
      }
    }
  }
  gmesh.write(std::cout);
  if (!k_debug) exit_immediately(0);  // Skip GMesh destruction for speedup.
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

// Write a single-resolution progressive mesh, encoding the spherical parameteriztion as lat-lon uv coordinates.
void write_parameterized_pm(PMeshIter pmi, CArrayView<Point> sphmap, bool split_meridian) {
  dummy_use(split_meridian);
  PMeshInfo pminfo = pmi.rstream()._info;
  pminfo._has_uv = true;
  for_int(w, pmi._wedges.num()) {
    const int v = pmi._wedges[w].vertex;
    pmi._wedges[w].attrib.uv = lonlat_from_sph(sphmap[v]);
  }
  if (split_meridian) split_awmesh_along_prime_meridian(pmi);
  PMesh pmesh(std::move(pmi), pminfo);
  pmesh.write(std::cout);
}

// *** Ply output.

// Write a *.ply mesh file, encoding the spherical parameteriztion as lat-lon uv coordinates on corners.
void write_parameterized_ply(PMeshIter pmi, CArrayView<Point> sphmap, bool split_meridian) {
  dummy_use(split_meridian);
  PMeshInfo& pminfo = pmi.rstream()._info;
  pminfo._has_uv = true;
  for_int(w, pmi._wedges.num()) {
    const int v = pmi._wedges[w].vertex;
    pmi._wedges[w].attrib.uv = lonlat_from_sph(sphmap[v]);
  }
  if (split_meridian) split_awmesh_along_prime_meridian(pmi);
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
  string to = "mesh";
  bool nooutput = false;

  ParseArgs args(argc, argv);
  HH_ARGSC("A progressive mesh is read from stdin or first arg.  Subsequent options are:");
  HH_ARGSP_O(verbose, "level : set output verbosity level (0..2)");
  HH_ARGSP_O(effort, "val : adjust optimization thoroughness (0..5)");
  HH_ARGSF_O(visualize, ": pipe interactive parameterization to G3dOGL viewer");
  HH_ARGSF_O(fix_base, ": freeze all vertices of the base mesh");
  HH_ARGSF_O(optimize_inverse, ": minimize reciprocal stretch sphere_dist/mesh_dist");
  HH_ARGSP_O(conformal_weight, "w : weight for added regularization energy term");
  HH_ARGSP_O(hole_weight, "w : factor for any faces marked with 'hole'");
  HH_ARGSF_O(respect_sharp_edges, ": for consistent parameterization");
  HH_ARGSF_O(flatten_to_x0, ": set surface coordinate x = 0 (for flat octahedron)");
  HH_ARGSP(base_param_scheme, "method : Base mesh map: tetra, projection, uv, coord, or mesh");
  HH_ARGSP(mesh_for_base, "mesh.m : To use with '-base_param_scheme mesh'");
  HH_ARGSP(rotate_s3d, "file.s3d : rotate spherical param using view (snapped to axes)");
  HH_ARGSF(no_rot_align, ": do not rotationally align map with face normals");
  HH_ARGSF(split_meridian, ": split mesh faces at prime meridian (zero lon)");
  HH_ARGSP(to, "format : set mesh output (mesh, pm, ply)");
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
  const Map<string, decltype(&write_parameterized_mesh)> output_formatters{
      {"mesh", &write_parameterized_mesh}, {"pm", &write_parameterized_pm}, {"ply", &write_parameterized_ply}};
  assertx(output_formatters.contains(to));

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
    output_formatters.get(to)(std::move(pmi), sphmap, split_meridian);
  }
  return 0;
}
