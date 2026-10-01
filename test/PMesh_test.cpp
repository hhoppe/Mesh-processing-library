// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/PMesh.h"

#include <sstream>  // std::stringstream

#include "libHh/Array.h"
#include "libHh/HashTuple.h"
#include "libHh/Map.h"
#include "libHh/Random.h"
using namespace hh;

namespace {

// The original eager implementations, kept verbatim as the reference.
struct Ref_VF : InlinedArray<int, 10> {
  Ref_VF(const AWMesh& mesh, int v, int f) {
    int ff = f, lastf, stopf;
    do {
      lastf = ff;
      ff = mesh._fnei[ff].faces[mod3(mesh.get_jvf(v, ff) + 2)];  // Go clw.
    } while (ff >= 0 && ff != f);
    if (ff < 0) {
      stopf = ff;
      ff = lastf;
    } else {
      stopf = f;
    }
    for (;;) {
      push(ff);
      ff = mesh._fnei[ff].faces[mod3(mesh.get_jvf(v, ff) + 1)];  // Go ccw.
      if (ff == stopf) break;
    }
  }
};

struct Ref_VV : InlinedArray<std::pair<int, int>, 10> {
  Ref_VV(const AWMesh& mesh, int v, int f) {
    int ff = f, lastf;
    do {
      lastf = ff;
      ff = mesh._fnei[ff].faces[mod3(mesh.get_jvf(v, ff) + 2)];  // Go clw.
    } while (ff >= 0 && ff != f);
    if (ff < 0) ff = lastf;
    int j = mesh.get_jvf(v, ff);
    int vv = mesh._wedges[mesh._faces[ff].wedges[mod3(j + 1)]].vertex;
    const int stopv = vv;
    int nextv = mesh._wedges[mesh._faces[ff].wedges[mod3(j + 2)]].vertex;
    while (vv >= 0) {
      push(std::pair{vv, ff});
      vv = nextv;
      lastf = ff;
      ff = mesh._fnei[ff].faces[mod3(j + 1)];
      if (ff < 0) {
        nextv = -1;
        ff = lastf;
      } else {
        nextv = mesh._wedges[mesh._faces[ff].wedges[mod3((j = mesh.get_jvf(v, ff)) + 2)]].vertex;
        if (nextv == stopv) nextv = -1;
      }
    }
  }
};

// Build a triangulated ny x nx grid; if wrap, identify the borders to obtain a closed torus.
AWMesh make_mesh(int ny, int nx, bool wrap) {
  AWMesh mesh;
  mesh._materials.set(0, "matid=0");
  const int nv = ny * nx;
  mesh._vertices.init(nv);
  mesh._wedges.init(nv);
  for_int(v, nv) {
    mesh._wedges[v].vertex = v;
    mesh._wedges[v].attrib = PmWedgeAttrib{Vector(0.f, 0.f, 1.f), A3dColor(0.f, 0.f, 0.f), Uv(0.f, 0.f)};
    const int x = v % nx, y = v / nx;
    mesh._vertices[v].attrib.point = Point(float(x), float(y), 0.f);
  }
  const auto vid = [&](int y, int x) { return (y % ny) * nx + (x % nx); };
  const int ylim = wrap ? ny : ny - 1, xlim = wrap ? nx : nx - 1;
  for_int(y, ylim) for_int(x, xlim) {
    const int v00 = vid(y, x), v01 = vid(y, x + 1), v10 = vid(y + 1, x), v11 = vid(y + 1, x + 1);
    for (const Vec3<int>& tri : {Vec3<int>{v00, v01, v11}, Vec3<int>{v00, v11, v10}}) {
      PmFace face;
      for_int(j, 3) face.wedges[j] = tri[j];
      face.attrib.matid = 0;
      mesh._faces.push(face);
    }
  }
  // Construct the dual adjacency: _fnei[f].faces[j] is across the edge opposite wedges[j].
  Map<std::pair<int, int>, std::pair<int, int>> edge_map;  // (v1, v2) -> (face, j).
  mesh._fnei.init(mesh._faces.num());
  for_int(f, mesh._faces.num()) for_int(j, 3) mesh._fnei[f].faces[j] = AWMesh::k_undefined;
  for_int(f, mesh._faces.num()) for_int(j, 3) {
    const int v1 = mesh._faces[f].wedges[mod3(j + 1)], v2 = mesh._faces[f].wedges[mod3(j + 2)];
    edge_map.enter(std::pair{v1, v2}, std::pair{f, j});
  }
  for_int(f, mesh._faces.num()) for_int(j, 3) {
    const int v1 = mesh._faces[f].wedges[mod3(j + 1)], v2 = mesh._faces[f].wedges[mod3(j + 2)];
    if (const std::pair<int, int>* fj = edge_map.find_ptr(std::pair{v2, v1})) mesh._fnei[f].faces[j] = fj->first;
  }
  return mesh;
}

void check(const AWMesh& mesh, const char* name) {
  mesh.ok();
  int num_checks = 0;
  Array<int> someface(mesh._vertices.num(), -1);
  for_int(f, mesh._faces.num()) for_int(j, 3) {
    const int v = mesh._wedges[mesh._faces[f].wedges[j]].vertex;
    // Verify from *every* incident face, not just one, to exercise all starting positions.
    const Ref_VF ref_faces(mesh, v, f);
    const Array<int> new_faces(mesh.ccw_faces(v, f));
    assertx(ref_faces == new_faces);
    const Ref_VV ref_vertices(mesh, v, f);
    const Array<std::pair<int, int>> new_vertices(mesh.ccw_vertices(v, f));
    assertx(ref_vertices.num() == new_vertices.num());
    for_int(i, ref_vertices.num()) assertx(ref_vertices[i] == new_vertices[i]);
    someface[v] = f;
    num_checks++;
  }
  for_int(v, mesh._vertices.num()) assertx(someface[v] >= 0);
  // A vertex is on the boundary iff its ccw face traversal starts at most_clw_face() and ends at most_ccw_face().
  const Array<int> gathered = mesh.gather_someface();
  for_int(v, mesh._vertices.num()) {
    const int f = gathered[v];
    assertx(contains(mesh.face_vertices(f), v));
    const int j = mesh.get_jvf(v, f);
    assertx(mesh._wedges[mesh.get_wvf(v, f)].vertex == v && mesh._faces[f].wedges[j] == mesh.get_wvf(v, f));
    assertx(mesh.face_points(f)[j] == mesh._vertices[v].attrib.point);
    const Array<int> faces(mesh.ccw_faces(v, f));
    const Array<std::pair<int, int>> vertices(mesh.ccw_vertices(v, f));
    if (mesh.is_boundary(v, f)) {
      assertx(mesh.most_clw_face(v, f) == faces[0] && mesh.most_ccw_face(v, f) == faces.last());
      assertx(vertices.num() == faces.num() + 1);
    } else {
      assertx(mesh.most_clw_face(v, f) < 0 && mesh.most_ccw_face(v, f) < 0);
      assertx(vertices.num() == faces.num());
    }
  }
  showf("%-24s nv=%-5d nf=%-5d checks=%-6d ok\n", name, mesh._vertices.num(), mesh._faces.num(), num_checks);
}

void test_attribs() {
  const PmVertexAttrib va1{Point(1.f, 2.f, 3.f)}, va2{Point(3.f, 2.f, 1.f)};
  PmVertexAttrib va;
  interp(va, va1, va2, .25f);
  SHOW(va.point);
  PmVertexAttribD vad;
  diff(vad, va1, va2);
  SHOW(vad.dpoint);
  add(va, va2, vad);
  assertx(compare(va, va1) == 0);
  sub(va, va1, vad);
  assertx(compare(va, va2) == 0);
  assertx(compare(va1, va2) < 0 && compare(va2, va1) > 0);
  const PmVertexAttrib va3{Point(1.f, 2.f, 3.0001f)};
  assertx(compare(va1, va3) != 0 && compare(va1, va3, 1e-3f) == 0);
  const PmWedgeAttrib wa1{Vector(1.f, 0.f, 0.f), A3dColor(1.f, 0.f, 0.f), Uv(0.f, 0.f)};
  const PmWedgeAttrib wa2{Vector(0.f, 1.f, 0.f), A3dColor(0.f, 1.f, 0.f), Uv(1.f, .5f)};
  PmWedgeAttrib wa;
  interp(wa, wa1, wa2, .5f);  // The interpolated normal is renormalized.
  SHOW(transformed(wa.normal, [](float v) { return round_fraction_digits(v, 1e4f); }), wa.rgb, wa.uv);
  PmWedgeAttribD wad;
  diff(wad, wa2, wa1);
  add(wa, wa1, wad);
  assertx(compare(wa, wa2) == 0);
  sub_noreflect(wa, wa2, wad);
  assertx(compare(wa, wa1) == 0);
  add_zero(wa, wad);
  assertx(wa.normal == wad.dnormal && wa.rgb == wad.drgb && wa.uv == wad.duv);
  // With sub_reflect(), the delta of the normal is reflected about the base normal.
  sub_reflect(wa, wa2, wad);
  SHOW(wa.normal, wa.rgb, wa.uv);
  assertx(compare(wa1, wa2) != 0 && compare(wa1, wa1, 0.f) == 0);
}

void test_extract_and_split() {
  PMeshInfo pminfo{};
  pminfo._read_version = 2;
  {
    // The Euler characteristic of the extracted grid is 1, and that of the torus is 0.
    for (const bool wrap : {false, true}) {
      const AWMesh mesh = make_mesh(4, 5, wrap);
      const GMesh gmesh = mesh.extract_gmesh(pminfo);
      gmesh.ok();
      assertx(gmesh.is_nice());
      SHOW(wrap, gmesh.num_vertices(), gmesh.num_edges(), gmesh.num_faces());
      SHOW(gmesh.num_vertices() - gmesh.num_edges() + gmesh.num_faces());
      const Face f = gmesh.id_face(1);
      SHOW(gmesh.get_string(f), gmesh.get_string(gmesh.id_vertex(2)));
    }
  }
  {
    // A simple mesh has one vertex per wedge.
    const AWMesh mesh = make_mesh(3, 3, false);
    const SMesh smesh(mesh);
    assertx(smesh._vertices.num() == mesh._wedges.num() && smesh._faces.num() == mesh._faces.num());
  }
  {
    // Split some edges of the closed torus.
    AWMesh mesh = make_mesh(4, 5, true);
    const int nv = mesh._vertices.num(), nf = mesh._faces.num();
    mesh.split_edge(0, 0, .25f);
    mesh.split_edge(7, 1, .5f);
    mesh.split_edge(nf, 2, .5f);  // Split an edge of a new face.
    check(mesh, "torus_after_split_edge");
    assertx(mesh._vertices.num() == nv + 3 && mesh._faces.num() == nf + 6);
    SHOW(mesh._vertices[nv].attrib.point, mesh._vertices[nv + 1].attrib.point);
    const GMesh gmesh = mesh.extract_gmesh(pminfo);
    gmesh.ok();
    assertx(gmesh.num_vertices() - gmesh.num_edges() + gmesh.num_faces() == 0);
  }
}

void test_io() {
  PMeshInfo pminfo{};
  pminfo._read_version = 2;
  AWMesh mesh = make_mesh(4, 5, false);
  mesh._wedges[3].attrib.normal = Vector(0.f, 1.f, 0.f);
  mesh._faces[2].attrib.matid = 1;
  mesh._materials.set(1, "matid=1 rgb=(1 0 0)");
  {
    // Write and read an AWMesh; its adjacency is reconstructed.
    std::stringstream ss;
    mesh.write(ss, pminfo);
    AWMesh mesh2;
    mesh2.read(ss, pminfo);
    mesh2.ok();
    assertx(mesh2._vertices.num() == mesh._vertices.num() && mesh2._faces.num() == mesh._faces.num());
    for_int(v, mesh._vertices.num()) assertx(mesh2._vertices[v].attrib.point == mesh._vertices[v].attrib.point);
    for_int(w, mesh._wedges.num()) {
      assertx(mesh2._wedges[w].vertex == mesh._wedges[w].vertex);
      assertx(compare(mesh2._wedges[w].attrib, mesh._wedges[w].attrib) == 0);
    }
    for_int(f, mesh._faces.num()) {
      assertx(mesh2._faces[f].wedges == mesh._faces[f].wedges);
      assertx(mesh2._faces[f].attrib.matid == mesh._faces[f].attrib.matid);
      assertx(mesh2._fnei[f].faces == mesh._fnei[f].faces);
    }
    SHOW(mesh2._materials.num(), mesh2._materials.get(1));
  }
  {
    // A PMesh without any vertex splits.
    const PMesh pmesh(AWMesh(mesh), pminfo);
    SHOW(pmesh._info._full_nvertices, pmesh._info._full_nfaces, pmesh._info._full_bbox);
    std::stringstream ss;
    pmesh.write(ss);
    const string str = ss.str();
    SHOW(str.substr(0, str.find("PM base mesh:")));
    PMesh pmesh2;
    pmesh2.read(ss);
    assertx(pmesh2._vsplits.num() == 0 && pmesh2._base_mesh._faces.num() == mesh._faces.num());
    assertx(pmesh2._info._full_bbox == pmesh._info._full_bbox);
    std::stringstream ss2;
    pmesh2.write(ss2);
    assertx(ss2.str() == str);
    // Iterate over the PMesh.
    PMeshRStream pmrs(pmesh2);
    PMeshIter pmi(pmrs);
    assertx(pmrs.is_reversible() && !pmi.next() && !pmi.prev());
    assertx(pmi.goto_nvertices(mesh._vertices.num()) && !pmi.goto_nvertices(mesh._vertices.num() + 1));
    assertx(pmi._faces.num() == mesh._faces.num());
    const GMesh gmesh = pmi.extract_gmesh();
    SHOW(gmesh.num_vertices(), gmesh.num_faces());
    // Stream the PMesh from the input stream.
    std::stringstream ss3(str);
    PMeshRStream pmrs3(ss3);
    assertx(!pmrs3.is_reversible());
    PMeshIter pmi3(pmrs3);
    assertx(!pmrs3.peek_next_vsplit() && !pmi3.next());
    assertx(pmi3._vertices.num() == mesh._vertices.num());
  }
}

}  // namespace

int main() {
  check(make_mesh(5, 6, false), "grid_with_boundary");
  check(make_mesh(6, 7, true), "closed_torus");
  check(make_mesh(3, 4, true), "small_closed");
  check(make_mesh(3, 3, false), "tiny_with_boundary");
  {  // Verify the range concepts and lazy (allocation-free) iteration.
    const AWMesh mesh = make_mesh(4, 4, true);
    using VFR = decltype(mesh.ccw_faces(0, 0));
    using VVR = decltype(mesh.ccw_vertices(0, 0));
    static_assert(ranges::forward_range<VFR> && ranges::view<VFR> && ranges::viewable_range<VFR>);
    static_assert(ranges::forward_range<VVR> && ranges::view<VVR> && ranges::viewable_range<VVR>);
    static_assert(std::forward_iterator<ranges::iterator_t<VFR>>);
    static_assert(std::forward_iterator<ranges::iterator_t<VVR>>);
    SHOW(sizeof(VFR), sizeof(VVR));
    SHOW(Array(mesh.ccw_faces(5, 0) | views::filter([](int f) { return f % 2 == 0; })));
  }
  test_attribs();
  test_extract_and_split();
  test_io();
  return 0;
}
