// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Mk3d.h"

namespace hh {

Mk3d::Mk3d(WA3dStream& pos) : _os(pos) { _os.write_comment(" Init Mk3d"); }

Mk3d::~Mk3d() {
  _os.write_comment(sform(" End Mk3d: %dgons %dlines %dpoints, %dverts(max %d)",  //
                          _total_polygons, _total_polylines, _total_points, _total_vertices, _max_vertices));
  _os.flush();
  assertw(_stack_frame.empty());
  assertw(_ctm.is_ident());
  assertw(_stack_color.empty());
  assertw(_stack_force_polyline.empty());
  assertw(_stack_force_flip.empty());
  assertw(!_el.num());
}

void Mk3d::scale(float x, float y, float z) {
  if ((x <= 0 && x != -1) || (y <= 0 && y != -1) || (z <= 0 && z != -1)) {
    if (Warning("mk3d: strange scale()")) SHOW(x, y, z);
  }
  apply(Frame::scaling(V(x, y, z)));
}

void Mk3d::scale_color(float sr, float sg, float sb) {
  _cc.d *= V(sr, sg, sb);
  _cc.s *= V(sr, sg, sb);
}

void Mk3d::normal(const Vector& normal) {
  bool modified = false;
  assertx(_el.num());
  Vector tn = transform(normal);
  assertw(tn.normalize());
  for_int(i, 3) {
    if (abs(tn[i]) < 1e-5f) {
      tn[i] = 0.f;
      modified = true;
    }
  }
  if (modified) assertw(tn.normalize());
  _el[_el.num() - 1].n = tn;
}

void Mk3d::end_polygon() {
  assertx(_el.num() >= 3);
  if (_force_flip) flip_poly();
  if (_force_polyline) {
    _el.push(_el[0]);
    end_polyline();
  } else {
    output_poly();
    _el.init(A3dElem::EType::polygon);  // clear it
  }
}

void Mk3d::end_2polygon() {
  if (_force_polyline) {
    end_polygon();
  } else {
    output_poly();
    flip_poly();
    output_poly();
  }
  _el.init(A3dElem::EType::polygon);
}

void Mk3d::end_polyline() {
  assertx(_el.num() >= 2);
  _el.update(A3dElem::EType::polyline);
  _os.write(_el);
  _total_polylines++;
  _total_vertices += _el.num();
  if (_el.num() > _max_vertices) _max_vertices = _el.num();
  _el.init(A3dElem::EType::polygon);
}

void Mk3d::end_point() {
  assertx(_el.num() == 1);
  _el.update(A3dElem::EType::point);
  _os.write(_el);
  _total_points++;
  _total_vertices += 1;
  if (_el.num() > _max_vertices) _max_vertices = _el.num();
  _el.init(A3dElem::EType::polygon);
}

void Mk3d::output_poly() {
  assertx(_el.num() >= 3);
  assertx(_el.type() == A3dElem::EType::polygon);  // optional
  _os.write(_el);
  _total_polygons++;
  _total_vertices += _el.num();
  if (_el.num() > _max_vertices) _max_vertices = _el.num();
}

void Mk3d::flip_poly() {
  int n = _el.num();
  for_intL(i, 1, (n - 1) / 2 + 1) std::swap(_el[i], _el[n - i]);  // slightly different from reverse()
  for_int(i, n) _el[i].n = -_el[i].n;
}

}  // namespace hh
