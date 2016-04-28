// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "CloseMinCycles.h"
#include "Args.h"
#include "Timer.h"
#include "FileIO.h"
using namespace hh;

namespace {

float maxcyclelength = BIGFLOAT;
int maxcyclenedges = INT_MAX;
int ncycles = INT_MAX;
int genus = 0;
float fraccyclelength = 1.f;
bool nooutput = false;

GMesh mesh;

void do_closecycles() {
    CloseMinCycles cmc(mesh);
    cmc._max_cycle_length = maxcyclelength;
    cmc._max_cycle_nedges = maxcyclenedges;
    cmc._ncycles = ncycles;
    cmc._desired_genus = genus;
    cmc._frac_cycle_length = fraccyclelength;
    cmc.compute();
}

} // namespace

int main(int argc, const char** argv) {
    ParseArgs args(argc, argv);
    ARGSC("",                   ":* Criteria for stopping topological simplification:");
    ARGSP(maxcyclelength,       "len : when smallest cycle exceeds specified length");
    ARGSP(maxcyclenedges,       "n : when smallest cycle has >n edges");
    ARGSP(ncycles,              "n : after removing this number of cycles");
    ARGSP(genus,                "g : when mesh genus <=g");
    ARGSC("",                   ":*");
    ARGSP(fraccyclelength,      "frac>=1 : allow finding cycles with length fractionally greater than minimal");
    ARGSC("",                   ":*");
    ARGSD(closecycles,          ": perform topological simplification");
    ARGSF(nooutput,             ": do not print mesh at program end");
    HH_TIMER(main);
    string arg0 = args.num() ? args.peek_string() : "";
    if (!ParseArgs::special_arg(arg0)) {
        string filename = "-"; if (args.num() && (arg0=="-" || arg0[0]!='-')) filename = args.get_filename();
        RFile fi(filename);
        HH_TIMER(_readmesh);
        for (string sline; fi().peek()=='#'; ) {
            assertx(my_getline(fi(), sline));
            if (sline.size()>1) showff("|%s\n", sline.substr(2).c_str());
        }
        mesh.read(fi());
        showff("%s", args.header().c_str());
    }
    args.parse();
    hh_clean_up();
    if (!nooutput) { HH_TIMER(_writemesh); mesh.write(std::cout); }
    return 0;
}
