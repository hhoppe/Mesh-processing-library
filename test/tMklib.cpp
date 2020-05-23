// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Mklib.h"
#include "RangeOp.h"            // round_elements()
using namespace hh;

int main(int argc, char**) {
    if (argc>1) {
        RSA3dStream ia3d(std::cin);
        WSA3dStream oa3d(std::cout);
        A3dElem el;
        for (;;) {
            ia3d.read(el);
            if (el.type()==A3dElem::EType::endfile) break;
            if (el.type()==A3dElem::EType::polygon || el.type()==A3dElem::EType::polyline) {
                for_int(i, el.num()) {
                    const float fac = 1e4f;
                    round_elements(el[i].p, fac);
                    round_elements(el[i].n, fac);
                }
            }
            oa3d.write(el);
        }
        return 0;
    }
    WSA3dStream os(std::cout);
    Mk3d mk(os);
    Mklib mkl(mk);
    os.write_comment(" begin test of mklib");
    os.write_comment("cubeO");
    { mk_save; mk.translate(2, 0, 0); mkl.cubeO(); }
    os.write_comment("cubeU");
    { mk_save; mk.translate(4, 0, 0); mkl.cubeU(); }
    {
        mk_save; mk.translate(6, 0, 0); mk.rotate(2, TAU/4); mk.scale(1, 1, .5);
        os.write_comment("tetra");
        mkl.tetra();
        os.write_comment("polygon");
        mk.point(10, 2, 3);
        mk.point(11, 3, 5);
        mk.point(11, 5, 2);
        mk.end_polygon();
    }
    os.write_comment("tetra");
    { mk_save; mk.translate(2, 5, 0); mkl.tetra(); }
    os.write_comment("cylinderU");
    { mk_save; mk.translate(5, 5, 0); mkl.cylinderU(7); }
    {
        mk_save_color; mk.diffuse(1, 1, 1); mk.specular(.5f, .5f, .2f); mk.phong(4);
        os.write_comment("volume_ringU");
        { mk_save; mk.translate(8, 5, 0); mkl.volume_ringU(5, .7f); }
        os.write_comment("capU");
        { mk_save; mk.translate(2, 8, 0); mkl.capU(3); }
    }
    os.write_comment("sphere");
    { mk_save; mk.translate(4, 8, 2); mkl.sphere(4, 5); }
    os.write_comment("tetraU");
    { mk_save; mk.translate(7, 8, 2); mkl.tetraU(); }
    return 0;
}
