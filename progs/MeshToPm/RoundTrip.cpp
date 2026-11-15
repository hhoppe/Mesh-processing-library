// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "progs/MeshToPm/RoundTrip.h"

#include "libHh/RangeOp.h"
#include "libHh/StringOp.h"

namespace hh {

namespace {

using RFace = RoundTripReference::Face;

// Rotate the face so that its smallest id is first.
RFace normalized_face(const Vec3<int>& ids, const Vec3<PmWedgeAttrib>& attribs, const Vec3<int>& wedges, int matid) {
  const int j0 = ids[0] < ids[1] ? (ids[0] < ids[2] ? 0 : 2) : (ids[1] < ids[2] ? 1 : 2);
  RFace face;
  for_int(k, 3) {
    face.ids[k] = ids[mod3(j0 + k)];
    face.attribs[k] = attribs[mod3(j0 + k)];
    face.wedges[k] = wedges[mod3(j0 + k)];
  }
  face.matid = matid;
  return face;
}

void sort_faces(Array<RFace>& faces) {
  sort(faces, [](const RFace& f1, const RFace& f2) { return f1.ids < f2.ids; });
}

float max_abs_diff(const PmWedgeAttrib& a1, const PmWedgeAttrib& a2) {
  float d = 0.f;
  for_int(c, 3) d = max(d, abs(a1.normal[c] - a2.normal[c]));
  for_int(c, 3) d = max(d, abs(a1.rgb[c] - a2.rgb[c]));
  for_int(c, 2) d = max(d, abs(a1.uv[c] - a2.uv[c]));
  return d;
}

}  // namespace

RoundTripReference make_round_trip_reference(const WorkMesh& mesh) {
  RoundTripReference ref;
  for_int(v, mesh._vertices.num()) {
    if (!mesh.vertex_alive(v)) continue;
    ref.original_ids.push(mesh._vertex_original_id[v]);
    ref.points.push(mesh._vertices[v].attrib.point);
  }
  for_int(f, mesh._faces.num()) {
    if (!mesh.face_alive(f)) continue;
    Vec3<int> ids;
    Vec3<PmWedgeAttrib> attribs;
    for_int(j, 3) {
      ids[j] = mesh._vertex_original_id[mesh.vertex(f, j)];
      attribs[j] = mesh._wedges[mesh._faces[f].wedges[j]].attrib;
    }
    ref.faces.push(normalized_face(ids, attribs, mesh._faces[f].wedges, mesh._faces[f].attrib.matid));
  }
  sort_faces(ref.faces);
  ref.materials = mesh._materials;
  Array<bool> used(mesh._wedges.num(), false);
  for (const RFace& face : ref.faces)
    for (const int w : face.wedges) used[w] = true;
  for (const bool b : used) ref.num_wedges += b;
  return ref;
}

void check_round_trip(const PMesh& pmesh, const RoundTripReference& ref, CArrayView<int> pm_original_ids) {
  PMeshRStream pmrs(pmesh);
  PMeshIter pmi(pmrs);
  assertx(pmi.goto_nvertices(pmesh._info._full_nvertices));
  const int nv = pmi._vertices.num();
  if (nv != ref.points.num()) assertnever(sform("roundtrip: %d vertices, expected %d", nv, ref.points.num()));
  if (pmi._faces.num() != ref.faces.num())
    assertnever(sform("roundtrip: %d faces, expected %d", pmi._faces.num(), ref.faces.num()));
  assertx(pm_original_ids.num() == nv);
  // Positions.
  int max_id = 0;
  for (const int id : ref.original_ids) max_id = max(max_id, id);
  Array<int> ref_index(max_id + 1, -1);
  for_int(i, ref.original_ids.num()) ref_index[ref.original_ids[i]] = i;
  float max_coord = 0.f;
  for (const Point& p : ref.points) max_coord = max(max_coord, max_abs_element(p));
  const float position_tolerance = 16.f * std::numeric_limits<float>::epsilon() * max_coord;
  const float attrib_tolerance = 16.f * std::numeric_limits<float>::epsilon() * 4.f;
  float max_position_diff = 0.f;
  int num_position_diffs = 0;
  for_int(v, nv) {
    const int id = pm_original_ids[v];
    const int i = id <= max_id ? ref_index[id] : -1;
    if (i < 0) assertnever(sform("roundtrip: vertex %d has unexpected original id %d", v, id));
    const float d = max_abs_element(pmi._vertices[v].attrib.point - ref.points[i]);
    max_position_diff = max(max_position_diff, d);
    num_position_diffs += d > 0.f;
  }
  // Faces, materials, and wedge attributes.
  Array<RFace> faces;
  for_int(f, pmi._faces.num()) {
    Vec3<int> ids;
    Vec3<PmWedgeAttrib> attribs;
    for_int(j, 3) {
      const int w = pmi._faces[f].wedges[j];
      ids[j] = pm_original_ids[pmi._wedges[w].vertex];
      attribs[j] = pmi._wedges[w].attrib;
    }
    faces.push(normalized_face(ids, attribs, pmi._faces[f].wedges,
                               pmi._faces[f].attrib.matid & ~AWMesh::k_Face_visited_mask));
  }
  sort_faces(faces);
  if (pmi._wedges.num() != ref.num_wedges || pmi._wedges.num() != pmesh._info._full_nwedges)
    assertnever(sform("roundtrip: %d wedges, expected %d (header %d)", pmi._wedges.num(), ref.num_wedges,
                      pmesh._info._full_nwedges));
  // The corners must be grouped into wedges identically: a consistent one-to-one map between the wedges.
  int max_ref_wedge = 0;
  for (const RFace& rface : ref.faces) max_ref_wedge = max(max_ref_wedge, max(rface.wedges));
  Array<int> pm_of_ref(max_ref_wedge + 1, -1), ref_of_pm(pmi._wedges.num(), -1);
  float max_attrib_diff = 0.f;
  int num_attrib_diffs = 0;
  for_int(i, faces.num()) {
    const RFace& face = faces[i];
    const RFace& rface = ref.faces[i];
    if (face.ids != rface.ids)
      assertnever(sform("roundtrip: face (%d %d %d) differs from expected (%d %d %d)", face.ids[0], face.ids[1],
                        face.ids[2], rface.ids[0], rface.ids[1], rface.ids[2]));
    if (pmi._materials.get(face.matid) != ref.materials.get(rface.matid))
      assertnever(sform("roundtrip: face (%d %d %d) has material '%s' rather than '%s'", face.ids[0], face.ids[1],
                        face.ids[2], pmi._materials.get(face.matid).c_str(), ref.materials.get(rface.matid).c_str()));
    for_int(k, 3) {
      int& w1 = pm_of_ref[rface.wedges[k]];
      int& w2 = ref_of_pm[face.wedges[k]];
      if (w1 < 0 && w2 < 0) w1 = face.wedges[k], w2 = rface.wedges[k];
      if (w1 != face.wedges[k] || w2 != rface.wedges[k])
        assertnever(sform("roundtrip: the wedges at vertex %d of face (%d %d %d) differ from the expected grouping",
                          face.ids[k], face.ids[0], face.ids[1], face.ids[2]));
      const float d = max_abs_diff(face.attribs[k], rface.attribs[k]);
      max_attrib_diff = max(max_attrib_diff, d);
      num_attrib_diffs += d > 0.f;
    }
  }
  showdf("roundtrip: max_position_diff=%g (%d vertices differ) max_attrib_diff=%g (%d corners differ)\n",
         max_position_diff, num_position_diffs, max_attrib_diff, num_attrib_diffs);
  if (max_position_diff > position_tolerance)
    assertnever(
        sform("roundtrip: position difference %g exceeds tolerance %g", max_position_diff, position_tolerance));
  if (max_attrib_diff > attrib_tolerance)
    assertnever(sform("roundtrip: attribute difference %g exceeds tolerance %g", max_attrib_diff, attrib_tolerance));
  showdf("roundtrip: ok nvertices=%d nfaces=%d\n", nv, faces.num());
}

}  // namespace hh
