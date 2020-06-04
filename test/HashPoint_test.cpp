// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/HashPoint.h"
using namespace hh;

int main() {
  HashPoint hp;
  SHOW(hp.enter(Point(1.f, 1.f, 1.f)));
  SHOW(hp.enter(Point(1.f, 1.f, 2.f)));
  SHOW(hp.enter(Point(1.f, 2.f, 1.f)));
  SHOW(hp.enter(Point(1.f, 1.f, 1.00001f)));
  SHOW(hp.enter(Point(1.f, 1.f, 2.01f)));
  SHOW(hp.enter(Point(1.f, 1.f, 2.00001f)));
  SHOW(hp.enter(Point(0.850299f, 0.453924f, 0.0222916f)));
  SHOW(hp.enter(Point(0.850299f, 0.453923f, 0.0222920f)));
}
