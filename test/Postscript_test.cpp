// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/Postscript.h"
using namespace hh;

int main() {
  Postscript ps(std::cout, 200, 200);
  ps.point(.2f, .1f);
  ps.point(.3f, .1f);
  ps.point(.4f, .12f);
  ps.line(.6f, .2f, .6f, .6f);
  ps.line(.6f, .2f, .8f, .2f);
  ps.line(.8f, .23f, .8f, .2f);
  ps.line(.8f, .23f, .77f, .23f);
  ps.line(-.1f, .2f, .3f, -.05f);
  ps.point(.4f, .15f);
}
