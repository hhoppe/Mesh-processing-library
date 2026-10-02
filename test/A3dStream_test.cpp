// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/BufferedA3dStream.h"

#include <sstream>  // std::stringstream

#include "libHh/Polygon.h"
#include "libHh/RangeOp.h"  // is_zero()
using namespace hh;

namespace {

bool same_vertices(const A3dElem& el1, const A3dElem& el2) {
  if (el1.type() != el2.type() || el1.num() != el2.num()) return false;
  for_int(i, el1.num()) {
    const A3dVertex &v1 = el1[i], &v2 = el2[i];
    if (v1.p != v2.p || v1.n != v2.n || v1.c.d != v2.c.d || v1.c.s != v2.c.s || v1.c.g != v2.c.g) return false;
  }
  return true;
}

// Self-checks, which produce no output (because stdout is the filtered stream of elements).
void self_test() {
  const A3dVertexColor color1(A3dColor(1.f, .5f, .25f)), color2(A3dColor(1.f, 0.f, .2f)),
      color3(A3dColor(.5f, .5f, .5f), A3dColor(.25f, .25f, .25f), A3dColor(4.f, 0.f, 0.f));
  assertx(color1.s == A3dColor(1.f, 1.f, 1.f) && color1.g == A3dColor(1.f, 0.f, 0.f));
  // The colors in the elements below are written as text, so they must have short exact decimal representations.
  // (Although 51 / 255.f == .2f in IEEE arithmetic, /fp:fast may give a nearby value instead.)
  assertx(dist(A3dVertexColor(Pixel(255, 0, 51)).d, A3dColor(1.f, 0.f, .2f)) < 1e-6f);
  // A polygon, its normal, and its Polygon.
  A3dElem polygon(A3dElem::EType::polygon);
  polygon.push(A3dVertex(Point(0.f, 0.f, 0.f), Vector(0.f, 0.f, 1.f), color1));
  polygon.push(A3dVertex(Point(2.f, 0.f, 0.f), Vector(0.f, 0.f, 1.f), color2));
  polygon.push(A3dVertex(Point(2.f, 2.f, 0.f), Vector(0.f, 0.f, 0.f), color3));
  polygon.push(A3dVertex(Point(0.f, 2.f, 0.f), Vector(0.f, .5f, .5f), color3));
  assertx(polygon.num() == 4 && polygon.pnormal() == Vector(0.f, 0.f, 1.f));
  Polygon poly;
  polygon.get_polygon(poly);
  assertx(poly.num() == 4 && poly[2] == Point(2.f, 2.f, 0.f));
  A3dElem polyline(A3dElem::EType::polyline, false, 2);
  polyline[0] = A3dVertex(Point(1.f, 2.f, 3.f), Vector(0.f, 0.f, 0.f), color2);
  polyline[1] = A3dVertex(Point(-1.f, 2.5f, 3e6f), Vector(0.f, 0.f, 0.f), color2);
  A3dElem point(A3dElem::EType::point);
  point.push(A3dVertex(Point(.5f, .25f, .125f), Vector(1.f, 0.f, 0.f), color1));
  A3dElem comment(A3dElem::EType::comment);
  comment.set_comment(" A comment.");
  // Changing the type of an element.
  A3dElem el = polyline;
  el.update(A3dElem::EType::polygon);
  assertx(el.type() == A3dElem::EType::polygon && el.num() == 2);
  el.update(A3dElem::EType::polyline, true);
  assertx(el.binary());
  assertx(A3dElem::command_type(A3dElem::EType::endframe) && !A3dElem::command_type(A3dElem::EType::point));
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange): 'd' is a reserved status type.
  assertx(A3dElem::status_type(A3dElem::EType('d')) && !A3dElem::status_type(A3dElem::EType::polygon));
  // Write the elements in text and binary forms, and read them back.
  for (const bool binary : {false, true}) {
    std::stringstream ss;
    {
      WSA3dStream oa3d(ss);
      for (A3dElem* pel : {&polygon, &polyline, &point}) {
        pel->set_binary(binary);
        oa3d.write(*pel);
      }
      oa3d.write(comment);
      oa3d.write_comment(" Line1.\n Line2.");
      oa3d.write_end_object(binary, 2.f, 3.f);
      oa3d.write_clear_object(binary);
      oa3d.write_end_frame(binary);
    }
    RSA3dStream ia3d(ss);
    A3dElem el2;
    ia3d.read(el2);
    assertx(el2.type() == A3dElem::EType::comment && el2.comment().starts_with(" Created by WA3dStream"));
    for (const A3dElem* pel : {&polygon, &polyline, &point}) {
      ia3d.read(el2);
      assertx(same_vertices(el2, *pel));
    }
    Array<string> comments;
    for_int(i, 3) {
      ia3d.read(el2);
      assertx(el2.type() == A3dElem::EType::comment);
      comments.push(el2.comment());
    }
    assertx(comments == Array<string>{" A comment.", " Line1.", " Line2."});
    ia3d.read(el2);
    assertx(el2.type() == A3dElem::EType::endobject && el2.f() == V(2.f, 3.f, 0.f));
    ia3d.read(el2);
    assertx(el2.type() == A3dElem::EType::editobject && el2.f() == V(1.f, 0.f, 0.f));
    ia3d.read(el2);
    assertx(el2.type() == A3dElem::EType::endframe);
    ia3d.read(el2);
    assertx(el2.type() == A3dElem::EType::endfile);
  }
}

}  // namespace

int main() {
  self_test();
  const bool debug = getenv_bool("DEBUG");
  const bool is_bi = getenv_bool("BI");
  const bool is_bo = getenv_bool("BO");
  unique_ptr<RBuffer> pbi;
  unique_ptr<RA3dStream> pia3d;
  if (is_bi) {
    pbi = make_unique<RBuffer>(0);
    pia3d = make_unique<RBufferedA3dStream>(*pbi);
  } else {
    pia3d = make_unique<RSA3dStream>(std::cin);
  }
  unique_ptr<WBuffer> pbo;
  unique_ptr<WA3dStream> poa3d;
  if (is_bo) {
    pbo = make_unique<WBuffer>(1);
    poa3d = make_unique<WBufferedA3dStream>(*pbo);
  } else {
    poa3d = make_unique<WSA3dStream>(std::cout);
  }
  RA3dStream& ia3d = *pia3d;
  WA3dStream& oa3d = *poa3d;
  A3dElem el;
  for (;;) {
    if (is_bi) {
      const RBufferedA3dStream::ERecognize st = down_cast<RBufferedA3dStream*>(pia3d.get())->recognize();
      assertx(st != RBufferedA3dStream::ERecognize::parse_error);
      if (st != RBufferedA3dStream::ERecognize::yes) {
        const RBuffer::ERefill ret = pbi->refill();
        if (pbi->eof()) break;
        assertx(ret != RBuffer::ERefill::other);
        if (0 && ret == RBuffer::ERefill::no) break;
        continue;
      }
    }
    ia3d.read(el);
    if (debug) std::cerr << el;
    if (el.type() == A3dElem::EType::endfile) break;
    oa3d.write(el);
  }
}
