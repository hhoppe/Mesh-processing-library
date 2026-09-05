// -*- C++ -*-
#include "libHh/PMesh.h"

#include "libHh/HashTuple.h"
#include "libHh/Map.h"
#include "libHh/PArray.h"
#include "libHh/Random.h"
using namespace hh;

namespace {

// The original eager implementations, kept verbatim as the reference.
struct Ref_VF : PArray<int, 10> {
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

struct Ref_VV : PArray<std::pair<int, int>, 10> {
  Ref_VV(const AWMesh& mesh, int v, int f) {
    int ff = f, lastf;
    do {
      lastf = ff;
      ff = mesh._fnei[ff].faces[mod3(mesh.get_jvf(v, ff) + 2)];  // Go clw.
    } while (ff >= 0 && ff != f);
    if (ff < 0) ff = lastf;
    int j = mesh.get_jvf(v, ff);
    int vv = mesh._wedges[mesh._faces[ff].wedges[mod3(j + 1)]].vertex;
    int stopv = vv;
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
  const int nv = ny * nx;
  mesh._vertices.init(nv);
  mesh._wedges.init(nv);
  for_int(v, nv) {
    mesh._wedges[v].vertex = v;
    const int x = v % nx, y = v / ny;
    mesh._vertices[v].attrib.point = Point(float(x), float(y), 0.f);
  }
  const auto vid = [&](int y, int x) { return (y % ny) * nx + (x % nx); };
  const int ylim = wrap ? ny : ny - 1, xlim = wrap ? nx : nx - 1;
  for_int(y, ylim) for_int(x, xlim) {
    const int v00 = vid(y, x), v01 = vid(y, x + 1), v10 = vid(y + 1, x), v11 = vid(y + 1, x + 1);
    for (const Vec3<int>& tri : {Vec3<int>{v00, v01, v11}, Vec3<int>{v00, v11, v10}}) {
      PmFace face;
      for_int(j, 3) face.wedges[j] = tri[j];
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
    bool present;
    const auto [f2, j2] = edge_map.retrieve(std::pair{v2, v1}, present);
    if (present) mesh._fnei[f].faces[j] = f2;
    dummy_use(j2);
  }
  return mesh;
}

void check(const AWMesh& mesh, const char* name) {
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
  showf("%-24s nv=%-5d nf=%-5d checks=%-6d ok\n", name, mesh._vertices.num(), mesh._faces.num(), num_checks);
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
  return 0;
}
