// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/A3dStream.h"  // A3dColor
#include "libHh/Args.h"
#include "libHh/Bbox.h"
#include "libHh/BinarySearch.h"
#include "libHh/Facedistance.h"  // project_point_triangle2()
#include "libHh/FileIO.h"
#include "libHh/GMesh.h"
#include "libHh/MathOp.h"
#include "libHh/MeshOp.h"      // Vnors
#include "libHh/MeshSearch.h"  // PolygonFaceSpatial
#include "libHh/Parallel.h"
#include "libHh/Polygon.h"
#include "libHh/Random.h"
#include "libHh/RangeOp.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, A3dColor, c_color);
HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, Vector, c_normal);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Vector, v_normal);

int verbose = 1;
bool bothdir = true;
float nptfac = 1.f;
bool errmesh = false;
bool vertexpts = false;
bool unitcube0 = false;
bool unitdiag0 = true;
bool maxerror = false;

Array<GMesh> meshes;  // meshes to compare
Frame xform;          // space -> "small" unit cube around all meshes
int numpts;
float g_side0;
float g_diag0;
float bbdiag;

void do_mfile(Args& args) {
  const string filename = args.get_filename();
  assertx(meshes.num() < 2);
  meshes.add(1);
  GMesh& mesh = meshes.last();
  mesh.read(RFile(filename)());
  for (Vertex v : mesh.vertices()) {
    Vnors vnors(mesh, v);
    v_normal(v) = vnors.is_unique() ? vnors.unique_nor() : Vector(BIGFLOAT, BIGFLOAT, BIGFLOAT);
    for (Corner c : mesh.corners(v)) {
      {
        A3dColor& a = c_color(c);
        if (!mesh.parse_corner_key_vec(c, "rgb", a)) fill(a, BIGFLOAT);
      }
      {
        Vector& a = c_normal(c);
        Face f = mesh.corner_face(c);
        a = vnors.is_unique() ? vnors.unique_nor() : vnors.face_nor(f);
      }
    }
  }
}

struct PStats {
  Stat Sgd2;  // geometric error
  Stat Scd2;  // color error
  Stat Snd2;  // normal error
  void add(const PStats& pstats) {
    Sgd2.add(pstats.Sgd2);
    Scd2.add(pstats.Scd2);
    Snd2.add(pstats.Snd2);
  }
};

void project_point(GMesh& mesh_s, const Point& ps, const A3dColor& pscol, const Vector& psnor, const GMesh& mesh_d,
                   const PolygonFaceSpatial& psp, Vertex vv, string& str, PStats& pstats) {
  SpatialSearch<PolygonFace*> ss(&psp, ps * xform);
  PolygonFace* polyface = ss.next();
  Face fd = polyface->face;
  Vec3<Corner> cad = mesh_d.triangle_corners(fd);
  Bary baryd;
  {
    Point clp;
    float d2 = project_point_triangle2(ps, mesh_d.point(mesh_d.corner_vertex(cad[0])),
                                       mesh_d.point(mesh_d.corner_vertex(cad[1])),
                                       mesh_d.point(mesh_d.corner_vertex(cad[2])), baryd, clp);
    pstats.Sgd2.enter(d2);
    if (errmesh && vv) {
      float g_K = 1'000'000.f / bbdiag;
      float g_MK = 0.f;
      if (0) g_MK = 75.f / bbdiag;
      d2 = abs(d2);
      float val = g_K * std::log(d2 + 1.f);
      HH_SSTAT(Serrval, val);
      // showdf("val %f first cut %f d2 %f\n", val, bbdiag / 100, d2 * 100);
      // if (val < 1.f)
      if (val < bbdiag / 100) {
        mesh_s.update_string(vv, "rgb", csform(str, "(%g %g %g)", 1.f, max(0.f, 1.f - val), max(0.f, 1.f - val)));
      } else if (val < bbdiag / 50) {  // else if (val < 2.f)
        mesh_s.update_string(vv, "rgb", csform(str, "(%g %g %g)", 1.f, min(val - 1.f, 1.f), 0.f));
      } else {
        // val = val - 2;
        if (0) val /= g_MK;
        mesh_s.update_string(vv, "rgb", csform(str, "(%g %g %g)", max(0.f, 1.f - val), 0.f, 0.f));
      }
    }
  }
  pstats.Scd2.enter(dist2(pscol, interp(c_color(cad[0]), c_color(cad[1]), c_color(cad[2]), baryd[0], baryd[1])));
  pstats.Snd2.enter(dist2(psnor, interp(c_normal(cad[0]), c_normal(cad[1]), c_normal(cad[2]), baryd[0], baryd[1])));
}

void project_point(GMesh& mesh_s, Face fs, const Bary& barys, const GMesh& mesh_d, const PolygonFaceSpatial& psp,
                   Vertex vv, string& str, PStats& pstats) {
  dummy_use(fs);
  Vec3<Corner> cas = mesh_s.triangle_corners(fs);
  Point ps = interp(mesh_s.point(mesh_s.corner_vertex(cas[0])), mesh_s.point(mesh_s.corner_vertex(cas[1])),
                    mesh_s.point(mesh_s.corner_vertex(cas[2])), barys[0], barys[1]);
  A3dColor pscol = interp(c_color(cas[0]), c_color(cas[1]), c_color(cas[2]), barys[0], barys[1]);
  Vector psnor = interp(c_normal(cas[0]), c_normal(cas[1]), c_normal(cas[2]), barys[0], barys[1]);
  project_point(mesh_s, ps, pscol, psnor, mesh_d, psp, vv, str, pstats);
}

void print_it(const string& s, const PStats& pstats) {
  if (1) {
    float vg = my_sqrt(pstats.Sgd2.avg());
    string s_g = (unitcube0   ? sform("uL2=%%%#-10.4f", vg / g_side0 * 100.f)
                  : unitdiag0 ? sform("dL2=%%%#-10.4f", vg / g_diag0 * 100.f)
                              : sform(" L2=%#-10.5f", vg));
    showdf("%s(%7d)  %s  cL2=%#-10.4g  nL2=%#-10.4g\n",  //
           s.c_str(), pstats.Sgd2.inum(), s_g.c_str(), my_sqrt(pstats.Scd2.avg()), my_sqrt(pstats.Snd2.avg()));
    {
      float d = vg;
      float peak = g_diag0;
      float psnr = 20.f * std::log10(peak / d);
      float npsnr = 20.f * std::log10(2.f / my_sqrt(pstats.Snd2.avg()));
      showdf("PSNR=%.1f  nPSNR=%.1f\n", psnr, npsnr);
    }
  }
  if (maxerror) {
    float vg = my_sqrt(pstats.Sgd2.max());
    string s_g = (unitcube0   ? sform("uLi=%%%#-10.4f", vg / g_side0 * 100.f)
                  : unitdiag0 ? sform("dLi=%%%#-10.4f", vg / g_diag0 * 100.f)
                              : sform(" Li=%#-10.5f", vg));
    showdf("%s(%7d)  %s  cLi=%#-10.4g  nLi=%#-10.4g\n",  //
           s.c_str(), pstats.Sgd2.inum(), s_g.c_str(), my_sqrt(pstats.Scd2.max()), my_sqrt(pstats.Snd2.max()));
  }
}

void compute_mesh_distance(GMesh& mesh_s, const GMesh& mesh_d, PStats& pastats) {
  const bool use_parallelism = !errmesh;
  const Bbox bbox{transform(mesh_d.vertices(), [&](Vertex v) { return mesh_d.point(v); })};
  bbdiag = mag(bbox[0] - bbox[1]);
  // showdf("size of the diag %f\n", bbdiag);
  // PolygonFaceSpatial psp(max(10, int(sqrt(float(mesh_d.num_vertices())) / 5.f + .5f)));
  int psp_size = (mesh_d.num_vertices() < 20'000    ? 25
                  : mesh_d.num_vertices() < 30'000  ? 32
                  : mesh_d.num_vertices() < 100'000 ? 40
                  : mesh_d.num_vertices() < 300'000 ? 70
                                                    : 100);
  psp_size = getenv_int("PSP_SIZE", psp_size, true);
  Array<PolygonFace> ar_polyface;
  ar_polyface.reserve(mesh_d.num_faces());
  PolygonFaceSpatial psp(psp_size);
  {
    HH_TIMER("_create_spatial");
    for (Face fd : mesh_d.faces()) {
      Polygon poly(3);
      mesh_d.polygon(fd, poly);
      assertx(poly.num() == 3);
      for_int(i, poly.num()) poly[i] *= xform;
      ar_polyface.push(PolygonFace(std::move(poly), fd));
    }
    for (PolygonFace& polyface : ar_polyface) psp.enter(&polyface);
  }
  HH_TIMER("_sample_distances");
  if (numpts) {
    PStats pstats;
    // showdf("- random sampling of %d points\n", numpts);
    Array<Face> fface;    // Face of this index (nf)
    Array<float> fcarea;  // cumulative area (nf + 1)
    {
      double sum_area = 0.;  // for accuracy
      for (Face f : mesh_s.faces()) {
        float area = mesh_s.area(f);
        fface.push(f);
        fcarea.push(float(sum_area));
        sum_area += area;
      }
      for_int(face_index, fface.num()) fcarea[face_index] /= float(sum_area);
      fcarea.push(1.00001f);
    }
    Array<float> randoms;
    for_int(i, numpts * 3) randoms.push(Random::G.unif());
    const int num_threads = use_parallelism ? get_max_threads() : 1;
    Array<PStats> ar_pstats(num_threads);
    parallel_for_chunk(range(numpts), num_threads, [&](int thread_index, auto subrange) {
      string str;
      for (const int i : subrange) {
        const int face_index = discrete_binary_search(fcarea, 0, fface.num(), randoms[i * 3 + 0]);
        Face f = fface[face_index];
        float a = randoms[i * 3 + 1], b = randoms[i * 3 + 2];
        if (a + b > 1.f) a = 1.f - a, b = 1.f - b;
        Bary bary(a, b, 1.f - a - b);
        project_point(mesh_s, f, bary, mesh_d, psp, nullptr, str, ar_pstats[thread_index]);
      }
    });
    for_int(thread_index, num_threads) pstats.add(ar_pstats[thread_index]);
    if (verbose >= 2) print_it(" r", pstats);
    pastats.add(pstats);
  }
  if (vertexpts) {
    PStats pstats;
    {
      const int num_threads = use_parallelism ? get_max_threads() : 1;
      Array<PStats> ar_pstats(num_threads);
      parallel_for_chunk(Array<Vertex>(mesh_s.vertices()), num_threads, [&](int thread_index, auto subrange) {
        string str;
        for (Vertex v : subrange) {
          const A3dColor pscol(0.f, 0.f, 0.f);
          const Vector& psnor = v_normal(v);
          project_point(mesh_s, mesh_s.point(v), pscol, psnor, mesh_d, psp, v, str, ar_pstats[thread_index]);
        }
      });
      for_int(thread_index, num_threads) pstats.add(ar_pstats[thread_index]);
    }
    if (verbose >= 2) print_it(" v", pstats);
    pastats.add(pstats);
  }
}

void do_distance() {
  HH_TIMER("_distance");
  assertx(meshes.num() == 2);
  int maxnfaces = 0;
  for_int(imesh, 2) {
    assertx(meshes[imesh].num_faces());
    maxnfaces = max(maxnfaces, meshes[imesh].num_faces());
  }
  const Bbox bbox0{transform(meshes[0].vertices(), [&](Vertex v) { return meshes[0].point(v); })};
  const Bbox bbox1{transform(meshes[1].vertices(), [&](Vertex v) { return meshes[1].point(v); })};
  const Bbox bbox = bbox_union(bbox0, bbox1);
  xform = bbox.get_frame_to_small_cube();
  g_side0 = bbox0.max_side();
  g_diag0 = dist(bbox0[0], bbox0[1]);
  numpts = int(maxnfaces * nptfac + .5f);
  PStats pbstats;
  for_int(idir, 2) {
    PStats pastats;
    if (!bothdir && idir == 1) continue;
    if (bothdir && verbose >= 2) showdf("Distance mesh%d -> mesh%d\n", idir, 1 - idir);
    compute_mesh_distance(meshes[idir], meshes[1 - idir], pastats);
    pbstats.add(pastats);
    if (!bothdir || verbose >= 2) print_it(sform(" %c", '0' + idir), pastats);
  }
  if (bothdir) print_it(" B", pbstats);
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSD(mfile, "filename : read a mesh file");
  HH_ARGSP(bothdir, "bool : also compute dist(mesh2, mesh1)");
  HH_ARGSP(nptfac, "fac : random points samples");
  HH_ARGSP(vertexpts, "bool : also project vertices");
  HH_ARGSP(errmesh, "bool : output mesh with error");
  HH_ARGSP(verbose, "int : verbosity level");
  HH_ARGSP(unitcube0, "bool : normalize distance by mesh0 bbox side");
  HH_ARGSP(unitdiag0, "bool : normalize distance by mesh0 bbox diag");
  HH_ARGSP(maxerror, "bool : include Linf norm");
  HH_ARGSD(distance, ": compute inter-mesh distances");
  HH_ARGSC("");
  HH_ARGSC("Example command: MeshDistance -mf mesh1.m -mf mesh2.m -maxerror 1 -distance");
  HH_TIMER("MeshDistance");
  args.parse();
  hh_clean_up();
  if (errmesh) meshes[0].write(std::cout);
  if (!k_debug) exit_immediately(0);
  return 0;
}
