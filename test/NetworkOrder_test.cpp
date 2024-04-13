// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "libHh/NetworkOrder.h"

#include "libHh/Array.h"
#include "libHh/BinaryIO.h"
#include "libHh/FileIO.h"
#include "libHh/Matrix.h"
#include "libHh/Stat.h"
using namespace hh;

int main() {
  {
    string ter_grid = "NetworkOrder_test.inp";
    RFile fi(ter_grid);
    int gridx, gridy;
    float fx;
    read_binary_std(fi(), ArView(fx));
    gridx = int(fx);
    float fy;
    read_binary_std(fi(), ArView(fy));
    gridy = int(fy);
    SHOW(gridx, gridy);
    assertx(gridx >= 4 && gridy >= 4);
    Matrix<float> ggridf(gridx, gridy);
    assertx(read_binary_std(fi(), ggridf.array_view()));
    HH_RSTAT(Sgrid, ggridf);
  }
}
