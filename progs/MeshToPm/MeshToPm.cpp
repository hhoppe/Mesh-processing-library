// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/Args.h"
#include "libHh/ConsoleProgress.h"
#include "libHh/FileIO.h"
#include "libHh/PMesh.h"
#include "libHh/Parallel.h"
#include "libHh/RangeOp.h"
#include "libHh/StringOp.h"
#include "libHh/Timer.h"
#include "progs/MeshToPm/EcolEval.h"
#include "progs/MeshToPm/MeshInput.h"
#include "progs/MeshToPm/PmBuild.h"
#include "progs/MeshToPm/RoundTrip.h"
#include "progs/MeshToPm/WorkMesh.h"
using namespace hh;

namespace {

int verb = 1;                  // Verbosity level.
bool nooutput = false;         // Do not write the progressive mesh.
int nfaces = 0;                // Target number of faces of the next '-simplify'.
int nvertices = 0;             // Target number of vertices of the next '-simplify'.
bool roundtrip_check = false;  // Verify that the progressive mesh reproduces its full mesh.
string original_indices;       // File to which to write the original vertex ids of the progressive mesh.

WorkMesh mesh;        // Current mesh.
EcolOptions options;  // Simplification parameters.
PmRecorder recorder;  // Records the vsplits.
std::optional<RoundTripReference> roundtrip_reference;
Array<int> chart_num_boundary_vertices;  // For min_chart_bnd_vertices: the boundary vertex count of each chart.

// Returns the canonical edges of the live mesh: each interior edge once (from its lower-indexed face), each boundary
// edge from its face.
Array<EdgeHandle> canonical_edges() {
  Array<EdgeHandle> edges;
  for_int(f, mesh._faces.num()) {
    if (!mesh.face_alive(f)) continue;
    for_int(j, 3) {
      const int fn = mesh._fnei[f].faces[j];
      if (fn == AWMesh::k_undefined || f < fn) edges.push(EdgeHandle{f, j});
    }
  }
  return edges;
}

// Print the number of legal edge collapses (for testing).
void do_count_legal() {
  const Array<EdgeHandle> edges = canonical_edges();
  Array<bool> legal(edges.num());
  parallel_for(range(edges.num()),
               [&](const int i) { legal[i] = evaluate_ecol(mesh, edges[i], options).cost < BIGFLOAT; });
  showf("legal edges: %d of %d\n", int(ranges::count(legal, true)), edges.num());
}

// Write the cost of each edge collapse (for testing): the original ids of its vertices, then the cost.
void do_print_costs(Args& args) {
  WFile fi(args.get_filename());
  const Array<EdgeHandle> edges = canonical_edges();
  Array<float> costs(edges.num());
  parallel_for(range(edges.num()), [&](const int i) { costs[i] = evaluate_ecol(mesh, edges[i], options).cost; });
  string str;
  for_int(i, edges.num()) {
    const EdgeHandle e = edges[i];
    const int id1 = mesh._vertex_original_id[mesh.vertex(e.f, mod3(e.j + 1))];
    const int id2 = mesh._vertex_original_id[mesh.vertex(e.f, mod3(e.j + 2))];
    fi() << csform(str, "%d %d %.9g\n", min(id1, id2), max(id1, id2), costs[i]);
  }
}

// Returns the topology of the live mesh in the format of mesh_genus_string() (without sharp edges): the numbers of
// connected components, boundary loops, vertices, faces, and edges, and the genus.
string genus_string() {
  HH_TIMER("_genus");
  const int num_vertices = mesh._vertices.num();
  // Union-find structures over the vertices: one for the components, and one for the boundary loops.
  Array<int> component(range(num_vertices)), loop(range(num_vertices));
  const auto find = [](Array<int>& parent, int v) {
    while (parent[v] != v) v = parent[v] = parent[parent[v]];
    return v;
  };
  const auto unite = [&](Array<int>& parent, int v1, int v2) { parent[find(parent, v1)] = find(parent, v2); };
  Array<bool> has_face(num_vertices, false), on_boundary(num_vertices, false);
  int nf = 0, num_boundary_edges = 0;
  for_int(f, mesh._faces.num()) {
    if (!mesh.face_alive(f)) continue;
    nf++;
    for_int(j, 3) {
      has_face[mesh.vertex(f, j)] = true;
      unite(component, mesh.vertex(f, j), mesh.vertex(f, mod3(j + 1)));
      if (mesh._fnei[f].faces[j] != AWMesh::k_undefined) continue;
      num_boundary_edges++;
      const int v1 = mesh.vertex(f, mod3(j + 1)), v2 = mesh.vertex(f, mod3(j + 2));
      unite(loop, v1, v2);
      on_boundary[v1] = on_boundary[v2] = true;
    }
  }
  int nv = 0, nc = 0, nb = 0;
  for_int(v, num_vertices) {
    if (!mesh.vertex_alive(v)) continue;
    nv++;
    if (has_face[v] && find(component, v) == v) nc++;  // (As mesh_stat_components(), counting faces.)
    if (on_boundary[v] && find(loop, v) == v) nb++;
  }
  assertx((3 * nf + num_boundary_edges) % 2 == 0);
  const int ne = (3 * nf + num_boundary_edges) / 2;
  const float genus = (nc * 2 - (nv - ne + nf) - nb) / 2.f;
  return sform("Genus: c=%d b=%d  v=%d f=%d e=%d  genus=%g", nc, nb, nv, nf, ne, genus);
}

void perhaps_start_recording() {
  if (recorder.started) return;
  showdf("%s\n", genus_string().c_str());  // Of the finest mesh of the progressive mesh, as a header comment.
  start_recording(mesh, recorder);
  if (roundtrip_check) roundtrip_reference = make_round_trip_reference(mesh);
}

// Simplify the mesh until it has <= nfaces or <= nvertices, using sweeps as MeshSimplify's parallel_optimize():
// evaluate all edges, sort them by cost, and collapse the cheapest ones whose neighborhoods are unaffected by the
// preceding collapses of the sweep.
void simplify_mesh() {
  if (options.min_chart_bnd_vertices) assertx(options.strict_sharp >= 2);
  if (options.minaps && !mesh._has_uv) assertnever("'-minaps' requires uv coordinates");
  if (options.strict_sharp == 2) {  // As in MeshSimplify's optimize().
    for_int(f, mesh._faces.num()) {
      if (!mesh.face_alive(f)) continue;
      Vec3<int> fn;
      for_int(j, 3) {
        const int fj = mesh._fnei[f].faces[j];
        fn[j] = fj == AWMesh::k_undefined ? -1 : fj;
      }
      if (strict_mat_neighbors(mesh, f, fn)) continue;
      if (int(fn[0] < 0) + int(fn[1] < 0) + int(fn[2] < 0) >= 2)
        Warning("Valence 2 vertex on boundary will be degenerate");
      else
        assertnever("!strict_mat_neighbors prior to optimize()");
    }
  }
  if (options.min_chart_bnd_vertices) {
    chart_num_boundary_vertices.init(mesh._materials.num(), 0);
    for_int(v, mesh._vertices.num()) if (mesh.vertex_alive(v)) for (const int chart : charts_with_boundary_vertex(
                                                                        mesh, v)) chart_num_boundary_vertices[chart]++;
    options.chart_num_boundary_vertices = &chart_num_boundary_vertices;
  }
  int num_live_faces = mesh.num_live_faces(), num_live_vertices = mesh.num_live_vertices();
  const int orig_nfaces = num_live_faces, orig_nvertices = num_live_vertices;
  ConsoleProgress cprogress;
  int num_sweeps = 0;
  struct EdgeCost {
    EdgeHandle e;
    float cost;
    Vec2<int> vertices;  // The edge vertices, which remain valid even after a collapse destroys the edge.
  };
  const auto edge_less = [](const EdgeCost& ec1, const EdgeCost& ec2) {
    if (ec1.cost != ec2.cost) return ec1.cost < ec2.cost;
    if (ec1.e.f != ec2.e.f) return ec1.e.f < ec2.e.f;
    return ec1.e.j < ec2.e.j;
  };
  Array<EdgeCost> edge_costs;  // Its capacity is reused across sweeps.
  for (;;) {
    if (verb == 1) {
      const float fraction =
          nfaces ? (orig_nfaces - float(num_live_faces)) / max(1.f, orig_nfaces - float(nfaces))
                 : (orig_nvertices - float(num_live_vertices)) / max(1.f, orig_nvertices - float(nvertices));
      cprogress.update(fraction);
    }
    if (num_live_faces <= nfaces || num_live_vertices <= nvertices) break;
    num_sweeps++;
    if (edge_costs.capacity() > 3 * num_live_faces) edge_costs.clear();  // Release excessive capacity.
    edge_costs.init(0);                                                  // (Retaining the capacity.)
    for_int(f, mesh._faces.num()) {  // The canonical edges, as in canonical_edges() (without a separate array).
      if (!mesh.face_alive(f)) continue;
      for_int(j, 3) {
        const int fn = mesh._fnei[f].faces[j];
        if (fn == AWMesh::k_undefined || f < fn) edge_costs.push(EdgeCost{EdgeHandle{f, j}, 0.f, {}});
      }
    }
    parallel_for(range(edge_costs.num()), [&](const int i) {
      EdgeCost& edge_cost = edge_costs[i];
      const EdgeHandle e = edge_cost.e;
      edge_cost.cost = evaluate_ecol(mesh, e, options).cost;
      edge_cost.vertices = V(mesh.vertex(e.f, mod3(e.j + 1)), mesh.vertex(e.f, mod3(e.j + 2)));
    });
    // Sort only the prefix of edges that the commit loop can reach once it has collapsed an edge (the rest is sorted
    // only if needed); the ordering is a strict total order, so the result equals that of a full sort.
    const int num_prefix = min(int(0.15f * edge_costs.num()) + 1, edge_costs.num());
    std::nth_element(edge_costs.begin(), edge_costs.begin() + num_prefix, edge_costs.end(), edge_less);
    std::sort(edge_costs.begin(), edge_costs.begin() + num_prefix, edge_less);
    // Re-evaluate the prefix, retaining the best configurations.  To bound the memory, the prefix is evaluated in
    // chunks, each just before it is reached; an edge whose vertices are unchanged by the preceding collapses of the
    // sweep evaluates as it would have at the start of the sweep, and the other edges are skipped.
    constexpr int k_chunk_size = 1 << 16;
    Array<EcolResult> chunk_results;
    int chunk_start = 0;
    Array<bool> vertex_changed(mesh._vertices.num(), false);
    const auto evaluate_chunk = [&](int start) {
      chunk_start = start;
      chunk_results.init(min(k_chunk_size, num_prefix - start));
      parallel_for(range(chunk_results.num()), [&](const int k) {
        const EdgeCost& edge_cost = edge_costs[start + k];
        if (edge_cost.cost < BIGFLOAT && !vertex_changed[edge_cost.vertices[0]] &&
            !vertex_changed[edge_cost.vertices[1]])
          chunk_results[k] = evaluate_ecol(mesh, edge_cost.e, options);
      });
    };
    int num_collapsed = 0;
    for_int(i, edge_costs.num()) {
      if (num_collapsed > 0 && i >= num_prefix) break;
      if (i == num_prefix && num_prefix < edge_costs.num())  // The rest of the edges is needed after all.
        std::sort(edge_costs.begin() + num_prefix, edge_costs.end(), edge_less);
      const EdgeCost& edge_cost = edge_costs[i];
      if (edge_cost.cost == BIGFLOAT) break;
      if (num_live_faces <= nfaces || num_live_vertices <= nvertices) break;
      if (vertex_changed[edge_cost.vertices[0]] || vertex_changed[edge_cost.vertices[1]]) continue;
      if (i < num_prefix && i >= chunk_start + chunk_results.num()) evaluate_chunk(i);
      const EcolResult result =
          i < num_prefix ? std::move(chunk_results[i - chunk_start]) : evaluate_ecol(mesh, edge_cost.e, options);
      // An evaluation in a later chunk can differ from that at the start of the sweep if it depends on something
      // outside the marked vertices (e.g., the chart counts of min_chart_bnd_vertices, or the degrees of vo1 and vo2
      // used by bndfac); the edge is then left to the next sweep.
      if (result.cost != edge_cost.cost) continue;
      const EcolConfig& config = result.config;
      if (options.min_chart_bnd_vertices) {
        // The counts may have changed since the evaluation, due to other collapses in the charts.
        const bool ok = ranges::none_of(charts_with_boundary_vertex(mesh, config.vt), [&](int chart) {
          return chart_num_boundary_vertices[chart] <= options.min_chart_bnd_vertices;
        });
        if (!ok) continue;
        for (const int chart : charts_with_boundary_vertex(mesh, config.vt)) chart_num_boundary_vertices[chart]--;
      }
      // With half-edge collapses, the faces of vs that are not adjacent to vt are unchanged, so only the neighbors
      // of vt (including vs) are marked.
      vertex_changed[config.vt] = true;
      for (const auto [vv, ff] : mesh.ccw_vertices(config.vt, config.fl)) vertex_changed[vv] = true;
      collapse(mesh, config, recorder);
      num_collapsed++;
      num_live_faces -= config.fr >= 0 ? 2 : 1;
      num_live_vertices--;
    }
    if (verb >= 2) showdf("Sweep: %8d edges, %8d collapsed\n", edge_costs.num(), num_collapsed);
    if (num_collapsed == 0) break;
  }
  cprogress.clear();
  showdf("Simplified mesh: nvertices=%d nfaces=%d (%d sweeps)\n", num_live_vertices, num_live_faces, num_sweeps);
  nfaces = 0, nvertices = 0;  // Default for the next '-simplify'.
}

void do_roundtrip_check() {
  if (recorder.started) assertnever("'-roundtrip_check' must precede the first '-simplify'");
  roundtrip_check = true;
}

// Simplify, recording the edge collapses into the progressive mesh.
void do_simplify() {
  HH_TIMER("_simplify");
  perhaps_start_recording();
  simplify_mesh();
}

// Simplify without recording the edge collapses, so that the progressive mesh starts from the simplified mesh.
void do_presimplify() {
  HH_TIMER("_presimplify");
  if (recorder.started) assertnever("'-presimplify' must precede '-simplify'");
  simplify_mesh();
}

void do_outmesh(Args& args) {
  WFile fi(args.get_filename());
  write_mesh_m(fi(), mesh);
}

void do_outply(Args& args) {
  WFile fi(args.get_filename());
  const bool binary = true;
  compact_awmesh(mesh).write_ply(fi(), full_mesh_info(mesh), binary);
  assertx(fi());
}

}  // namespace

int main(int argc, const char** argv) {
  // A final '-simplify' is implied unless the last argument is '-simplify' (or an abbreviation such as '-simp').
  // (This is determined before ParseArgs takes ownership of argv.)
  const CArrayView<const char*> argv_view(argv, argc);
  const string last_arg = argc > 1 ? argv_view.last() : "";
  const bool last_arg_is_simplify = last_arg.size() >= 5 && std::string_view("-simplify").starts_with(last_arg);
  ParseArgs args(argc, argv);
  HH_ARGSC("A mesh (.m or .ply, possibly .gz) is read from stdin or first arg.  Subsequent options are:");
  HH_ARGSP(nfaces, "n : number of faces desired");
  HH_ARGSP(nvertices, "n : number of vertices desired");
  HH_ARGSD(simplify, ": simplify (until nfaces/nvertices, or no legal collapse); implied at end");
  HH_ARGSD(presimplify, ": simplify without recording (before '-simplify')");
  HH_ARGSC("", ":");
  HH_ARGSP_O(colfac, "f : weight of color (fraction of diam.)");
  HH_ARGSP_O(norfac, "f : weight of normals (fraction of diam.)");
  HH_ARGSP_O(bndfac, "f : weight to encourage boundary simpl. (1e10)");
  HH_ARGSP_O(neptfac, "f : weight of sharp and boundary edges (1)");
  HH_ARGSP_O(trishapeafac, "f : weight on tri aspect ratio times area");
  HH_ARGSP_O(maxvalence, "val : prevent ecols creating verts > valence");
  HH_ARGSF_O(dihallow, ": allow bad dihedral as last resort");
  HH_ARGSP_O(strict_sharp, "i : preserve topology of discont. curves");
  HH_ARGSP_O(min_chart_bnd_vertices, "n : with -strict 2, keep >= n bnd vertices per chart");
  HH_ARGSP_O(keepglobalv, "bool : do not simplify vertices tagged global");
  HH_ARGSF_O(minaps, ": use appearance-preserving simplification (needs uv)");
  HH_ARGSP_O(aps_bnd_fac, "factor : boost perceptual importance of bnd vertices");
  HH_ARGSC("", ":");
  HH_ARGSP(original_indices, "file.txt : output original vertex indices in creation order");
  HH_ARGSD(outmesh, "file.m : write the current mesh");
  HH_ARGSD(outply, "file.ply : write the current mesh (one ply vertex per wedge)");
  HH_ARGSF(nooutput, ": do not write the progressive mesh on stdout");
  HH_ARGSD(roundtrip_check, ": verify that the progressive mesh reproduces its full mesh");
  HH_ARGSD(count_legal, ": print the number of legal edge collapses");
  HH_ARGSD(print_costs, "file : write the cost of each edge collapse");
  HH_ARGSP(verb, "i : verbosity level (1=avg, 2=more)");
  const string arg0 = args.num() ? args.peek_string() : "";
  if (ParseArgs::special_arg(arg0)) args.parse(), exit(0);
  const string filename = args.num() && (arg0 == "-" || arg0[0] != '-') ? args.get_filename() : "-";
  RFile fi(filename);
  const MeshFileHeader header = read_mesh_header(fi());
  showdf("%s", args.header().c_str());
  Timer timer("MeshToPm", Timer::EMode::always);
  {
    HH_TIMER("_readmesh");
    read_mesh(fi(), header, mesh);
  }
  showdf("Read mesh: nvertices=%d nwedges=%d nfaces=%d nmaterials=%d\n", mesh.num_live_vertices(),
         full_mesh_info(mesh)._full_nwedges, mesh.num_live_faces(), mesh._materials.num());
  options.diam = assertx(mesh.bbox().max_side());
  args.parse();
  if (!last_arg_is_simplify && !nooutput) do_simplify();  // An implicit final '-simplify'.
  perhaps_start_recording();                              // E.g., with '-nooutput' and only '-outmesh' snapshots.
  const Array<int> pm_ids = pm_original_ids(mesh, recorder);
  if (original_indices != "") {
    WFile fi_indices(original_indices);
    fi_indices() << pm_ids.num() << '\n';
    for (const int id : pm_ids) fi_indices() << id << '\n';
  }
  AWMesh base_mesh;
  PMeshInfo pminfo;
  if (!nooutput || roundtrip_reference) finalize_pm(mesh, recorder, base_mesh, pminfo);
  if (roundtrip_reference) {
    PMesh pmesh;
    make_pmesh(base_mesh, pminfo, recorder, pmesh);
    check_round_trip(pmesh, *roundtrip_reference, pm_ids);
  }
  timer.terminate();
  hh_clean_up();  // Write the diagnostics before the progressive mesh, so that nothing follows it on stdout.
  if (!nooutput) {
    write_pm(std::cout, base_mesh, pminfo, recorder);
    std::cout.flush();
  }
  if (k_fast_exit) exit_immediately(0);
  return 0;
}
