// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Args.h"
#include "libHh/FileIO.h"
#include "libHh/GMesh.h"
#include "libHh/Matrix.h"
#include "libHh/PMesh.h"
#include "libHh/Stat.h"
#include "libHh/Timer.h"
using namespace hh;

namespace {

string rootname;
int blockx;
int blocky;
int blocks;

Matrix<PMesh> pmeshes;
PMesh pmesh;

void read_pms() {
  pmeshes.init(blockx, blocky);
  for_int(bx, blockx) {
    for_int(by, blocky) {
      string filename = sform("%s.x%d.y%d.pm", rootname.c_str(), bx, by);
      RFile fi(filename);
      if (!bx && !by) {
        for (string line; fi().peek() == '#';) {
          assertx(my_getline(fi(), line));
          if (line.size() > 1) showff("|%s\n", line.substr(2).c_str());
        }
      }
      pmeshes[bx][by].read(fi());
    }
  }
  // Other assertions
  for_int(bx, blockx) {
    for_int(by, blocky) {
      const PMesh& pmxy = pmeshes[bx][by];
      const AWMesh& bmeshxy = pmxy._base_mesh;
      // Assert all vertices have one wedge.
      for_int(wi, bmeshxy._wedges.num()) assertx(bmeshxy._wedges[wi].vertex == wi);
      // Assert that we have all info.
      assertx(pmxy._info._full_nvertices);
      assertx(pmxy._info._full_nwedges);
      assertx(pmxy._info._full_nfaces);
    }
  }
}

int compute_nvi(int bx, int by, int vi) {
  int x, y;
  if (vi <= blocks) {
    x = vi;
    y = 0;
  } else if (vi <= 2 * blocks + 1) {
    x = vi - (blocks + 1);
    y = blocks;
  } else if (vi <= 3 * blocks) {
    x = 0;
    y = vi - (2 * blocks + 1);
  } else {
    x = blocks;
    y = vi - (3 * blocks);
  }
  if (0) {
    // old scheme.
    if (0)
      ASSERTX((blocky + 1) * (blockx * blocks + 1) + (blockx + 1) * ((blocky * (blocks - 1))) ==
              2 * blockx * blocky * blocks + blocks * (blockx + blocky) - blockx * blocky + 1);
    if (y == 0 || y == blocks) {
      return (by + (y == blocks)) * (blockx * blocks + 1) + (bx * blocks + x);
    } else if (x == 0 || x == blocks) {
      return (blocky + 1) * (blockx * blocks + 1) + (bx + (x == blocks)) * (blocky * (blocks - 1)) +
             by * (blocks - 1) + (y - 1);
    } else {
      assertnever("");
    }
  } else {
    // new scheme that numbers outer boundary first.
    if (0)
      ASSERTX(((blockx * blocks + 1) * 2 + (blocky * blocks - 1) * 2 + (blocky - 1) * (blockx * blocks - 1) +
               (blockx - 1) * (blocky * (blocks - 1))) ==
              (2 * blockx * blocky * blocks + blocks * (blockx + blocky) - blockx * blocky + 1));
    if (y == 0 && by == 0) {
      return bx * blocks + x;
    } else if (y == blocks && by == blocky - 1) {
      return blockx * blocks + 1 + bx * blocks + x;
    } else if (x == 0 && bx == 0) {
      return (blockx * blocks + 1) * 2 + by * blocks + (y - 1);
    } else if (x == blocks && bx == blockx - 1) {
      return (blockx * blocks + 1) * 2 + blocky * blocks - 1 + by * blocks + (y - 1);
    } else if (y == 0 || y == blocks) {
      return (blockx * blocks + 1) * 2 + (blocky * blocks - 1) * 2 + (by + (y == blocks) - 1) * (blockx * blocks - 1) +
             bx * blocks + (x - 1);
    } else if (x == 0 || x == blocks) {
      return (blockx * blocks + 1) * 2 + (blocky * blocks - 1) * 2 + (blocky - 1) * (blockx * blocks - 1) +
             (bx + (x == blocks) - 1) * (blocky * (blocks - 1)) + (by * (blocks - 1) + (y - 1));
    } else {
      assertnever("");
    }
  }
}

void do_stitch() {
  assertx(blockx > 0 && blocky > 0 && blocks > 0);
  read_pms();
  // First, construct stitched base mesh.
  AWMesh& bmesh = pmesh._base_mesh;
  Matrix<int> m_basematid;  // [bx][by] -> first matid in base mesh
  m_basematid.init(blockx, blocky);
  assertx(!bmesh._materials.num());
  string str;
  for_int(bx, blockx) {
    for_int(by, blocky) {
      const PMesh& pmxy = pmeshes[bx][by];
      const AWMesh& bmeshxy = pmxy._base_mesh;
      m_basematid[bx][by] = bmesh._materials.num();
      for_int(matid, bmeshxy._materials.num()) {
        string s = bmeshxy._materials.get(matid);
        int nmatid = bmesh._materials.num();
        s = GMesh::string_update(s, "matid", csform(str, "%d", nmatid));
        if (!GMesh::string_has_key(s.c_str(), "rgb")) {
          int odd = (bx + by) % 2;
          Vector rgb(.6f + .2f * odd, .6f + .2f * !odd, .6f);
          s = GMesh::string_update(s, "rgb", csform_vec(str, rgb));
        }
        bmesh._materials.set(nmatid, s);
      }
    }
  }
  Matrix<Array<int>> f_renumber;  // [bx][by][old_face_id] -> new_face_id
  f_renumber.init(blockx, blocky);
  // Vertices of stitched base mesh are numbered as:
  // - first, (blocky + 1) rows of length (blockx * blocks + 1)
  // - next, (blockx + 1) broken columns of length (blocky * (blocks - 1))
  // - finally, internal vertices of blocks
  int tot_bnd_vertices = (blocky + 1) * (blockx * blocks + 1) + (blockx + 1) * (blocky * (blocks - 1));
  bmesh._vertices.init(tot_bnd_vertices);
  bmesh._wedges.init(tot_bnd_vertices);
  for_int(bx, blockx) {
    for_int(by, blocky) {
      const PMesh& pmxy = pmeshes[bx][by];
      const AWMesh& bmeshxy = pmxy._base_mesh;
      int vertex_offset = bmesh._vertices.num();
      int nbnd_vertices = 4 * blocks;
      int nint_vertices = bmeshxy._vertices.num() - nbnd_vertices;
      bmesh._vertices.add(nint_vertices);
      bmesh._wedges.add(nint_vertices);
      f_renumber[bx][by].init(pmxy._info._full_nfaces);
      for_int(fi, bmeshxy._faces.num()) {
        int nfi = bmesh._faces.add(1);
        f_renumber[bx][by][fi] = nfi;
        for_int(j, 3) {
          int vi = bmeshxy._faces[fi].wedges[j], nvi;
          if (vi >= 4 * blocks) {  // vertex internal to block
            nvi = vertex_offset + vi - (4 * blocks);
          } else {  // vertex on block boundary
            nvi = compute_nvi(bx, by, vi);
          }
          bmesh._faces[nfi].wedges[j] = nvi;
          // Next are inefficient (done many times!). ok for now.
          bmesh._vertices[nvi].attrib = bmeshxy._vertices[vi].attrib;
          // copy *one* of the normals
          //  (will have different normals at stitch boundary, so this is inexact!)
          bmesh._wedges[nvi].attrib = bmeshxy._wedges[vi].attrib;
        }
        bmesh._faces[nfi].attrib.matid = bmeshxy._faces[fi].attrib.matid + m_basematid[bx][by];
      }
    }
  }
  for_int(vi, bmesh._vertices.num()) bmesh._wedges[vi].vertex = vi;
  // Initialize some fields.
  pmesh._info = pmeshes[0][0]._info;  // including _has_*
  pmesh._info._full_bbox.clear();
  pmesh._vsplits.init(0);
  // Finally, collect together all vertex split records.
  int pmesh_nvertices = bmesh._vertices.num();
  int pmesh_nfaces = bmesh._faces.num();
  for_int(bx, blockx) {
    for_int(by, blocky) {
      const PMesh& pmxy = pmeshes[bx][by];
      int pmxy_nfaces = pmxy._base_mesh._faces.num();
      for_int(vspli, pmxy._vsplits.num()) {
        const Vsplit& vspl = pmxy._vsplits[vspli];
        pmesh._vsplits.push(vspl);
        pmesh._vsplits.last().flclw = f_renumber[bx][by][vspl.flclw];
        for_int(count, vspl.adds_two_faces() ? 2 : 1) f_renumber[bx][by][pmxy_nfaces++] = pmesh_nfaces++;
        pmesh_nvertices++;
      }
      assertx(pmxy_nfaces == pmxy._info._full_nfaces);
      pmesh._info._full_bbox.union_with(pmxy._info._full_bbox);
      if (1) {  // optional (save memory)
        f_renumber[bx][by].init(0);
        pmeshes[bx][by]._base_mesh._vertices.clear();
        pmeshes[bx][by]._base_mesh._wedges.clear();
        pmeshes[bx][by]._base_mesh._faces.clear();
        pmeshes[bx][by]._base_mesh._fnei.clear();
        pmeshes[bx][by]._vsplits.clear();
      }
    }
  }
  // Fill in the PMesh information fields.
  pmesh._info._tot_nvsplits = pmesh_nvertices - bmesh._vertices.num();
  pmesh._info._full_nvertices = pmesh_nvertices;
  pmesh._info._full_nwedges = pmesh_nvertices;
  pmesh._info._full_nfaces = pmesh_nfaces;
  // Write out stitched PMesh.
  pmesh.write(std::cout);
}

}  // namespace

int main(int argc, const char** argv) {
  ParseArgs args(argc, argv);
  HH_ARGSP(rootname, "rootname : prefix of PM files (*.x0.y0.pm)");
  HH_ARGSP(blockx, "nx : number of blocks along x axis");
  HH_ARGSP(blocky, "ny : number of blocks along y axis");
  HH_ARGSP(blocks, "n : block size (num_vertices - 1 per side)");
  HH_ARGSD(stitch, ": stitch the PM's together");
  showdf("%s", args.header().c_str());
  HH_TIMER("StitchPM");
  args.parse();
  return 0;
}
