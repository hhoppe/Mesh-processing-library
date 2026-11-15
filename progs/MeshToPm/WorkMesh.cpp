// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "progs/MeshToPm/WorkMesh.h"

#include "libHh/LineWriter.h"
#include "libHh/Parallel.h"
#include "libHh/RangeOp.h"
#include "libHh/StringOp.h"

namespace hh {

int WorkMesh::num_live_faces() const {
  int n = 0;
  for_int(f, _faces.num()) n += face_alive(f);
  return n;
}

int WorkMesh::num_live_vertices() const {
  int n = 0;
  for_int(v, _vertices.num()) n += vertex_alive(v);
  return n;
}

Bbox<float, 3> WorkMesh::bbox() const {
  Bbox<float, 3> bbox;
  for_int(v, _vertices.num()) if (vertex_alive(v)) bbox.union_with(_vertices[v].attrib.point);
  return bbox;
}

int WorkMesh::degree(int v) const {
  return narrow_cast<int>(ranges::distance(ccw_faces(v, _someface[v]))) + is_boundary_vertex(v);
}

int WorkMesh::face_across(int f, int a, int b) const {
  if (f < 0) return -1;
  for_int(j, 3) {
    const int v = vertex(f, j);
    if (v != a && v != b) {
      const int fn = _fnei[f].faces[j];
      return fn == k_undefined ? -1 : fn;
    }
  }
  assertnever("");
}

bool WorkMesh::vertex_has_multiple_wedges(int v) const {
  const int w0 = get_wvf(v, _someface[v]);
  return ranges::any_of(ccw_faces(v, _someface[v]), [&](int f) { return get_wvf(v, f) != w0; });
}

void build_adjacency(WorkMesh& mesh) {
  const int nv = mesh._vertices.num(), nf = mesh._faces.num();
  // The half-edge of corner c = 3 * f + j goes from the vertex of corner mod3(j + 1) to that of corner mod3(j + 2).
  const auto hedge_start = [&](int c) { return mesh.vertex(c / 3, mod3(c % 3 + 1)); };
  const auto hedge_end = [&](int c) { return mesh.vertex(c / 3, mod3(c % 3 + 2)); };
  // Counting sort of the half-edges by their start vertex.
  Array<int> hstart(nv + 1, 0);
  for_int(f, nf) for_int(j, 3) hstart[mesh.vertex(f, j) + 1]++;
  for_int(v, nv) hstart[v + 1] += hstart[v];
  Array<int> hlist(3 * nf);
  {
    Array<int> fill(hstart.head(nv));
    for_int(c, 3 * nf) hlist[fill[hedge_start(c)]++] = c;
  }
  const auto vertex_id = [&](int v) { return mesh._vertex_original_id[v]; };
  mesh._fnei.init(nf);
  parallel_for(range(nf), [&](const int f) {
    for_int(j, 3) {
      const int c = 3 * f + j, a = hedge_start(c), b = hedge_end(c);
      int nsame = 0;
      for_intL(i, hstart[a], hstart[a + 1]) nsame += hedge_end(hlist[i]) == b;
      if (nsame != 1)
        assertnever(
            sform("Edge (%d, %d) occurs in %d faces with the same orientation (non-manifold or inconsistently "
                  "oriented mesh)",
                  vertex_id(a), vertex_id(b), nsame));
      int fn = AWMesh::k_undefined, nopp = 0;
      for_intL(i, hstart[b], hstart[b + 1]) {
        const int c2 = hlist[i];
        if (hedge_end(c2) == a) fn = c2 / 3, nopp++;
      }
      if (nopp > 1) assertnever(sform("Edge (%d, %d) is adjacent to more than two faces", vertex_id(a), vertex_id(b)));
      mesh._fnei[f].faces[j] = fn;
    }
  });
  mesh._someface.init(nv);
  mesh._vflags.resize(nv);
  int num_unreferenced = 0;
  for_int(v, nv) {
    if (hstart[v] == hstart[v + 1]) {
      mesh._someface[v] = -1;
      mesh._vflags[v] |= WorkMesh::k_vflag_removed;
      num_unreferenced++;
    } else {
      mesh._someface[v] = hlist[hstart[v]] / 3;
    }
  }
  if (num_unreferenced) {
    Warning("Ignoring vertices referenced by no face");
    showdf("(%d unreferenced vertices)\n", num_unreferenced);
  }
  // Verify that the faces of each vertex form a single fan.
  parallel_for(range(nv), [&](const int v) {
    if (mesh._someface[v] < 0) return;
    const int nfaces = narrow_cast<int>(ranges::distance(mesh.ccw_faces(v, mesh._someface[v])));
    if (nfaces != hstart[v + 1] - hstart[v])
      assertnever(sform("Vertex %d is not nice: its %d faces do not form a single fan", vertex_id(v),
                        hstart[v + 1] - hstart[v]));
  });
}

namespace {

void write_wedge_attributes(LineWriter& w, const WorkMesh& mesh, int wedge) {
  const PmWedgeAttrib& wa = mesh._wedges[wedge].attrib;
  w << "wid=" << (wedge + 1) << " normal=(" << wa.normal[0] << ' ' << wa.normal[1] << ' ' << wa.normal[2] << ')';
  if (mesh._has_rgb) w << " rgb=(" << wa.rgb[0] << ' ' << wa.rgb[1] << ' ' << wa.rgb[2] << ')';
  if (mesh._has_uv) w << " uv=(" << wa.uv[0] << ' ' << wa.uv[1] << ')';
}

}  // namespace

void write_mesh_m(std::ostream& os, const WorkMesh& mesh) {
  LineWriter w;
  for_int(v, mesh._vertices.num()) {
    if (!mesh.vertex_alive(v)) continue;
    const Point& p = mesh._vertices[v].attrib.point;
    w << "Vertex " << mesh._vertex_original_id[v] << ' ';
    for_int(c, 3) w << ' ' << p[c];
    const bool unique_wedge = !mesh.vertex_has_multiple_wedges(v);
    const bool global = mesh._vflags[v] & WorkMesh::k_vflag_global;
    if (unique_wedge || global) {
      w << " {";
      if (unique_wedge) write_wedge_attributes(w, mesh, mesh.get_wvf(v, mesh._someface[v]));
      if (global) w << (unique_wedge ? " global" : "global");
      w << '}';
    }
    w.write_line(os);
  }
  for_int(f, mesh._faces.num()) {
    if (!mesh.face_alive(f)) continue;
    w << "Face " << mesh._face_original_id[f] << ' ';
    for_int(j, 3) w << ' ' << mesh._vertex_original_id[mesh.vertex(f, j)];
    const string& material = mesh._materials.get(mesh._faces[f].attrib.matid);
    if (material != "") w << " {" << material << '}';
    w.write_line(os);
  }
  for_int(v, mesh._vertices.num()) {
    if (!mesh.vertex_alive(v) || !mesh.vertex_has_multiple_wedges(v)) continue;
    for (const int f : mesh.ccw_faces(v, mesh._someface[v])) {
      w << "Corner " << mesh._vertex_original_id[v] << ' ' << mesh._face_original_id[f] << " {";
      write_wedge_attributes(w, mesh, mesh.get_wvf(v, f));
      w << '}';
      w.write_line(os);
    }
  }
  assertx(os);
}

AWMesh compact_awmesh(const WorkMesh& mesh) {
  AWMesh awmesh;
  awmesh._materials = mesh._materials;
  Array<int> live_vertices;
  for_int(v, mesh._vertices.num()) if (mesh.vertex_alive(v)) live_vertices.push(v);
  sort(live_vertices, [&](int v1, int v2) { return mesh._vertex_original_id[v1] < mesh._vertex_original_id[v2]; });
  const int nv = live_vertices.num();
  Array<int> new_vertex(mesh._vertices.num(), -1);
  awmesh._vertices.init(nv);
  for_int(i, nv) {
    new_vertex[live_vertices[i]] = i;
    awmesh._vertices[i] = mesh._vertices[live_vertices[i]];
  }
  Array<int> new_wedge(mesh._wedges.num(), -1);
  awmesh._wedges.init(nv);
  for_int(i, nv) {
    const int v = live_vertices[i];
    bool first = true;
    for (const int f : mesh.ccw_faces(v, mesh._someface[v])) {
      const int w = mesh.get_wvf(v, f);
      if (new_wedge[w] >= 0) continue;
      new_wedge[w] = first ? i : awmesh._wedges.add(1);
      first = false;
      awmesh._wedges[new_wedge[w]] = PmWedge{i, mesh._wedges[w].attrib};
    }
  }
  Array<int> new_face(mesh._faces.num(), -1);
  for_int(f, mesh._faces.num()) {
    if (!mesh.face_alive(f)) continue;
    new_face[f] = awmesh._faces.add(1);
    PmFace& face = awmesh._faces.last();
    for_int(j, 3) face.wedges[j] = new_wedge[mesh._faces[f].wedges[j]];
    face.attrib = mesh._faces[f].attrib;
  }
  awmesh._fnei.init(awmesh._faces.num());
  for_int(f, mesh._faces.num()) {
    if (new_face[f] < 0) continue;
    for_int(j, 3) {
      const int fn = mesh._fnei[f].faces[j];
      awmesh._fnei[new_face[f]].faces[j] = fn == AWMesh::k_undefined ? AWMesh::k_undefined : new_face[fn];
    }
  }
  return awmesh;
}

PMeshInfo full_mesh_info(const WorkMesh& mesh) {
  PMeshInfo pminfo;
  pminfo._read_version = 2;
  pminfo._has_rgb = mesh._has_rgb;
  pminfo._has_uv = mesh._has_uv;
  pminfo._full_nvertices = mesh.num_live_vertices();
  pminfo._full_nfaces = mesh.num_live_faces();
  pminfo._full_bbox = mesh.bbox();
  return pminfo;
}

}  // namespace hh
