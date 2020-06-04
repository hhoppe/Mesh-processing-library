// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "A3dStream.h"  // A3dColor
#include "Args.h"
#include "Bbox.h"
#include "BinarySearch.h"
#include "Facedistance.h"  // project_point_triangle2()
#include "FileIO.h"
#include "GMesh.h"
#include "MathOp.h"
#include "MeshOp.h"      // Vnors
#include "MeshSearch.h"  // PolygonFaceSpatial
#include "Polygon.h"
#include "Random.h"
#include "RangeOp.h"
#include "Timer.h"
using namespace hh;

namespace {

HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, A3dColor, c_color);
HH_SAC_ALLOCATE_FUNC(Mesh::MCorner, Vector, c_normal);
HH_SAC_ALLOCATE_FUNC(Mesh::MVertex, Vector, v_normal);

int verb = 1;
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
  RFile fi(args.get_filename());
  assertx(meshes.num() < 2);
  meshes.add(1);
  GMesh& mesh = meshes.last();
  mesh.read(fi());
  Vnors vnors;
  for (Vertex v : mesh.vertices()) {
    vnors.compute(mesh, v);
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

void project_point(GMesh& meshs, const Point& ps, const A3dColor& pscol, const Vector& psnor, const GMesh& meshd,
                   const PolygonFaceSpatial& psp, Vertex vv, PStats& pstats) {
  SpatialSearch<PolygonFace*> ss(&psp, ps * xform);
  PolygonFace* polyface = ss.next();
  Face fd = polyface->face;
  Array<Corner> cad = meshd.get_corners(fd);
  Bary baryd;
  {
    Point clp;
    float d2 =
        project_point_triangle2(ps, meshd.point(meshd.corner_vertex(cad[0])), meshd.point(meshd.corner_vertex(cad[1])),
                                meshd.point(meshd.corner_vertex(cad[2])), baryd, clp);
    pstats.Sgd2.enter(d2);
    if (errmesh && vv) {
      float g_K = 1000000.f * (1.0f / bbdiag);
      float g_MK = 0.f;
      if (0) g_MK = 75.f * (1.0f / bbdiag);
      d2 = abs(d2);
      float val = g_K * log(d2 + 1.f);
      HH_SSTAT(Serrval, val);
      // showdf("val %f first cut %f d2 %f\n", val, bbdiag / 100, d2 * 100);
      // if (val < 1.f)
      if (val < bbdiag / 100) {
        meshs.update_string(vv, "rgb", sform("(%g %g %g)", 1.f, max(0.f, 1.f - val), max(0.f, 1.f - val)).c_str());
      } else if (val < bbdiag / 50) {  // else if (val < 2.f)
        meshs.update_string(vv, "rgb", sform("(%g %g %g)", 1.f, min(val - 1.f, 1.f), 0.f).c_str());
      } else {
        // val = val - 2;
        if (0) val /= g_MK;
        meshs.update_string(vv, "rgb", sform("(%g %g %g)", max(0.f, 1.f - val), 0.f, 0.f).c_str());
      }
    }
  }
  pstats.Scd2.enter(dist2(pscol, interp(c_color(cad[0]), c_color(cad[1]), c_color(cad[2]), baryd[0], baryd[1])));
  pstats.Snd2.enter(dist2(psnor, interp(c_normal(cad[0]), c_normal(cad[1]), c_normal(cad[2]), baryd[0], baryd[1])));
}

void project_point(GMesh& meshs, Face fs, CArrayView<Corner> cas, const Bary& barys, const GMesh& meshd,
                   const PolygonFaceSpatial& psp, Vertex vv, PStats& pstats) {
  dummy_use(fs);
  Point ps = interp(meshs.point(meshs.corner_vertex(cas[0])), meshs.point(meshs.corner_vertex(cas[1])),
                    meshs.point(meshs.corner_vertex(cas[2])), barys[0], barys[1]);
  A3dColor pscol = interp(c_color(cas[0]), c_color(cas[1]), c_color(cas[2]), barys[0], barys[1]);
  Vector psnor = interp(c_normal(cas[0]), c_normal(cas[1]), c_normal(cas[2]), barys[0], barys[1]);
  project_point(meshs, ps, pscol, psnor, meshd, psp, vv, pstats);
}

void print_it(const string& s, const PStats& pstats) {
  if (1) {
    float vg = my_sqrt(pstats.Sgd2.avg());
    string sg = (unitcube0 ? sform("uL2=%%%#-10.4f", vg / g_side0 * 100.f)
                           : unitdiag0 ? sform("dL2=%%%#-10.4f", vg / g_diag0 * 100.f) : sform(" L2=%#-10.5f", vg));
    showdf("%s(%7d)  %s  cL2=%#-10.4g  nL2=%#-10.4g\n", s.c_str(), pstats.Sgd2.inum(), sg.c_str(),
           my_sqrt(pstats.Scd2.avg()), my_sqrt(pstats.Snd2.avg()));
    {
      float d = vg;
      float peak = g_diag0;
      float psnr = 20.f * log10(peak / d);
      float npsnr = 20.f * log10(2.f / my_sqrt(pstats.Snd2.avg()));
      showdf("PSNR=%.1f  nPSNR=%.1f\n", psnr, npsnr);
    }
  }
  if (maxerror) {
    float vg = my_sqrt(pstats.Sgd2.max());
    string sg = (unitcube0 ? sform("uLi=%%%#-10.4f", vg / g_side0 * 100.f)
                           : unitdiag0 ? sform("dLi=%%%#-10.4f", vg / g_diag0 * 100.f) : sform(" Li=%#-10.5f", vg));
    showdf("%s(%7d)  %s  cLi=%#-10.4g  nLi=%#-10.4g\n", s.c_str(), pstats.Sgd2.inum(), sg.c_str(),
           my_sqrt(pstats.Scd2.max()), my_sqrt(pstats.Snd2.max()));
  }
}

void compute_mesh_distance(GMesh& meshs, const GMesh& meshd, PStats& pastats) {
  Bbox bbox;
  // compute the meshes bounding box
  for (Face f : meshd.faces()) {
    for (Vertex v : meshd.vertices(f)) bbox.union_with(meshd.point(v));
  }
  bbdiag = mag(bbox[0] - bbox[1]);
  // showdf("size of the diag %f\n", bbdiag);
  // PolygonFaceSpatial psp(max(10, int(sqrt(float(meshd.num_vertices())) / 5.f + .5f)));
  int psp_size = (meshd.num_vertices() < 20000
                      ? 25
                      : meshd.num_vertices() < 30000
                            ? 32
                            : meshd.num_vertices() < 100000 ? 40 : meshd.num_vertices() < 300000 ? 70 : 100);
  psp_size = getenv_int("PSP_SIZE", psp_size, true);
  Array<PolygonFace> ar_polyface;
  ar_polyface.reserve(meshd.num_faces());
  PolygonFaceSpatial psp(psp_size);
  {
    HH_TIMER(_create_spatial);
    for (Face fd : meshd.faces()) {
      Polygon poly(3);
      meshd.polygon(fd, poly);
      assertx(poly.num() == 3);
      for_int(i, poly.num()) poly[i] *= xform;
      ar_polyface.push(PolygonFace(std::move(poly), fd));
    }
    for (PolygonFace& polyface : ar_polyface) psp.enter(&polyface);
  }
  HH_TIMER(_sample_distances);
  if (numpts) {
    PStats pstats;
    // showdf("- random sampling of %d points\n", numpts);
    Array<Face> fface;    // Face of this index (nf)
    Array<float> fcarea;  // cumulative area (nf + 1)
    {
      double sum_area = 0.;  // for accuracy
      for (Face f : meshs.faces()) {
        float area = meshs.area(f);
        fface.push(f);
        fcarea.push(float(sum_area));
        sum_area += area;
      }
      for_int(i, fface.num()) fcarea[i] /= float(sum_area);
      fcarea.push(1.00001f);
    }
    Array<Corner> cas;
    for_int(i, numpts) {
      int fi = discrete_binary_search(fcarea, 0, fface.num(), Random::G.unif());
      Face f = fface[fi];
      cas = meshs.get_corners(f, std::move(cas));
      float a = Random::G.unif(), b = Random::G.unif();
      if (a + b > 1.f) {
        a = 1.f - a;
        b = 1.f - b;
      }
      Bary bary(a, b, 1.f - a - b);
      project_point(meshs, f, cas, bary, meshd, psp, nullptr, pstats);
    }
    if (verb >= 2) print_it(" r", pstats);
    pastats.add(pstats);
  }
  if (vertexpts) {
    PStats pstats;
    // showdf("- vertex sampling\n");
    Array<Corner> cas;
    for (Vertex v : meshs.vertices()) {
      if (1) {  // works on mesh containing just isolated vertices
        const Vector& psnor = v_normal(v);
        const A3dColor pscol(0.f, 0.f, 0.f);
        project_point(meshs, meshs.point(v), pscol, psnor, meshd, psp, v, pstats);
      } else {
        Face f = assertx(meshs.most_clw_face(v));
        cas = meshs.get_corners(f, std::move(cas));
        Bary bary(0.f, 0.f, 0.f);
        int cai = cas.index(meshs.corner(v, f));
        assertx(cai >= 0);
        bary[cai] = 1.f;
        project_point(meshs, f, cas, bary, meshd, psp, v, pstats);
      }
    }
    if (verb >= 2) print_it(" v", pstats);
    pastats.add(pstats);
  }
}

void do_distance() {
  HH_TIMER(_distance);
  assertx(meshes.num() == 2);
  int maxnfaces = 0;
  Bbox bbox;
  Bbox bbox0;
  for_int(imesh, 2) {
    assertx(meshes[imesh].num_faces());
    maxnfaces = max(maxnfaces, meshes[imesh].num_faces());
    for (Vertex v : meshes[imesh].vertices()) bbox.union_with(meshes[imesh].point(v));
    if (!imesh) bbox0 = bbox;
  }
  xform = bbox.get_frame_to_small_cube();
  g_side0 = bbox0.max_side();
  g_diag0 = dist(bbox0[0], bbox0[1]);
  numpts = int(maxnfaces * nptfac + .5f);
  PStats pbstats;
  for_int(idir, 2) {
    PStats pastats;
    if (!bothdir && idir == 1) continue;
    if (bothdir && verb >= 2) showdf("Distance mesh%d -> mesh%d\n", idir, 1 - idir);
    compute_mesh_distance(meshes[idir], meshes[1 - idir], pastats);
    pbstats.add(pastats);
    if (!bothdir || verb >= 2) print_it(sform(" %c", '0' + idir), pastats);
  }
  if (bothdir) print_it(" B", pbstats);
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSC("Example command", ": MeshDistance -mf mesh1.m -mf mesh2.m -maxerror 1 -distance");
  HH_ARGSD(mfile, "filename : read a mesh file");
  HH_ARGSP(bothdir, "bool : also compute dist(mesh2, mesh1)");
  HH_ARGSP(nptfac, "fac : random points samples");
  HH_ARGSP(vertexpts, "bool : also project vertices");
  HH_ARGSP(errmesh, "bool : output mesh with error");
  HH_ARGSP(verb, "int : verbosity level");
  HH_ARGSP(unitcube0, "bool : normalize distance by mesh0 bbox side");
  HH_ARGSP(unitdiag0, "bool : normalize distance by mesh0 bbox diag");
  HH_ARGSP(maxerror, "bool : include Linf norm");
  HH_ARGSD(distance, ": compute inter-mesh distances");
  HH_TIMER(MeshDistance);
  args.parse();
  if (errmesh) meshes[0].write(std::cout);
  return 0;
}
