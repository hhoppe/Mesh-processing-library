// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.
#include "libHh/PointsInput.h"

#include "libHh/A3dStream.h"
#include "libHh/Ply.h"

namespace hh {

namespace {

// Returns the character after the next one, which is put back.
int peek_second_char(std::istream& is) {
  is.get();
  const int next = is.peek();
  assertx(is.unget());
  return next;
}

}  // namespace

PointsInput::PointsInput(std::istream& is) : _is(is) {
  if (_is.peek() == '#') {
    for (string line; _is.peek() == '#';) {
      assertx(my_getline(_is, line));
      if (line.size() > 1) _provenance.push(line.substr(line[1] == ' ' ? 2 : 1));
    }
    // E.g. diagnostics leaked into the stdout of a program that wrote the PLY file, which other readers reject.
    if (_is.peek() == 'p' && peek_second_char(_is) == 'l')
      assertnever("points: a PLY file must not be preceded by '#' lines");
  } else if (_is.peek() == 'p') {
    // Either a PLY file ("ply") or a text A3d stream starting with a point ("p x y z"); a binary A3d record starts with
    // the byte 3.
    const int next = peek_second_char(_is);
    if (next == ' ' || next == '\t') return;  // A3d.
    if (next != 'l') assertnever("points: the input starts with 'p' but is neither a PLY file nor an A3d stream");
    _ply_header_lines = read_ply_header(_is);
    _provenance = ply_header_comments(_ply_header_lines);
  }
}

void PointsInput::echo_provenance() const {
  for (const string& line : _provenance) showff("|%s\n", line.c_str());
}

PointSet PointsInput::read() {
  PointSet point_set;
  if (_ply_header_lines.num()) {
    PlyMesh ply = read_ply_body(_is, _ply_header_lines);
    if (ply.num_faces())
      assertnever(sform("points: the PLY file has %d faces, whereas a set of points is expected", ply.num_faces()));
    point_set.points = std::move(ply.points);
    point_set.normals = std::move(ply.normals);
    return point_set;
  }
  RSA3dStream ia3d(_is);
  A3dElem el;
  bool has_normal = false;
  for (;;) {
    ia3d.read(el);
    if (el.type() == A3dElem::EType::endfile) break;
    if (el.type() == A3dElem::EType::comment) continue;
    if (el.type() == A3dElem::EType::polygon || el.type() == A3dElem::EType::polyline)
      assertnever("points: the A3d stream contains polygons or polylines, whereas a set of points is expected");
    if (el.type() != A3dElem::EType::point) continue;  // E.g., an end-of-frame.
    point_set.points.push(el[0].p);
    point_set.normals.push(el[0].n);
    has_normal = has_normal || !is_zero(el[0].n);
  }
  if (!has_normal) point_set.normals.init(0);
  return point_set;
}

}  // namespace hh
