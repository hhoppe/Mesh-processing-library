// -*- C++ -*-  Copyright (c) Microsoft Corporation; see license.txt
#include "Args.h"
#include "FrameIO.h"
#include "Stat.h"
#include "Quaternion.h"         // pow(Frame, float)
#include "GeomOp.h"             // euler_angles_to_frame()
using namespace hh;

namespace {

bool noinput = false;
int every = 0;
int object = -1;
bool ginverse = false;
bool toasciit = false;          //  "toascii" seems to be a reserved identifier in Win32
bool tobinary = false;
double delay = 0.;
bool is_pretransf = false;
bool is_transf = false;
Frame cpretransf, ctransf;
int icount = 0;
int ocount = 0;
bool b_frame = false;
bool statistics = false;
bool eof1 = false;
int repeat = 1;
bool orthonormalize = false;
float induce_roll = 0.f;
bool add_mid_frames = false;
float lowpass = 1.f;            // 1.f == no filtering

void do_create_euler(Args& args) {
    float yaw = to_rad(args.get_float());
    float pitch = to_rad(args.get_float());
    float roll = to_rad(args.get_float());
    Vec3<float> angles(yaw, pitch, roll);
    Frame frame = Frame::identity();
    euler_angles_to_frame(angles, frame);
    int obn = 0; float zoom = 0.f; bool bin = false;
    assertx(FrameIO::write(std::cout, frame, obn, zoom, bin));
    noinput = true;
}

void do_induce_roll(Frame& t) {
    static int icount1 = 0;
    static Point p0, p1, p2;
    p0 = p1; p1 = p2; p2 = t.p();
    p2[2] = 0;                  // set elevations to zero
    icount1++;
    if (icount1<3) return;
    double v1x = double(p1[0])-p0[0];
    double v1y = double(p1[1])-p0[1];
    double v2x = double(p2[0])-p0[0];
    double v2y = double(p2[1])-p0[1];
    float sarea = float(v1y*v2x-v1x*v2y);
    const int num = 5;
    static Vec<float,num> sa;
    for_int(i, num-1) { sa[i] = sa[i+1]; }
    sa[num-1] = sarea;
    float s = 0.f; for_int(i, num) { s += sa[i]; }
    s /= num;
    float bank = s*induce_roll;
    t = Frame::rotation(0, bank)*t;
    HH_SSTAT(Sbank, bank);
    // Not used: it couldn't estimate curvature well enough from
    //  my gcanyon_4k2k_fly2.frame file.
}

bool loop() {
    Frame t; int obn; float zoom; bool bin;
    if (!FrameIO::read(std::cin, t, obn, zoom, bin)) return true;
    icount++;
    if (object>=0) obn = object;
    if (ginverse) assertw(t.invert());
    if (is_pretransf) t = cpretransf*t;
    if (is_transf) t = t*ctransf;
    if (lowpass!=1.f) {
        static Frame to; static float zo;
        if (icount>1) {
            t = to*pow(~to*t, lowpass);
            zoom = zo*(1.f-lowpass)+zoom*lowpass;
        }
        to = t; zo = zoom;
    }
    if (orthonormalize) {
        for_int(i, 3) {
            assertw(t.v(i).normalize());
        }
        Vector v2 = cross(t.v(0), t.v(1));
        float vdot = dot(v2, t.v(2));
        assertx(abs(vdot)>.99f);
        t.v(2) = v2*sign(vdot);
    }
    if (induce_roll) do_induce_roll(t);
    if (add_mid_frames) {
        static Frame to; static float zo;
        if (icount>1) {
            Frame tn = to*pow(~to*t, .5f);
            tn.p() = interp(to.p(), t.p(), .5f);
            float zn = (zoom+zo)*.5f;
            if (!FrameIO::write(std::cout, tn, obn, zn, bin)) return true;
        }
        to = t; zo = zoom;
    }
    if (toasciit) bin = false;
    if (tobinary) bin = true;
    if (statistics) {
        static Point plast;
        if (icount>1) {
            HH_SSTAT(Sdisp, dist(t.p(), plast));
        }
        plast = t.p();
        HH_SSTAT(Sxlen, mag(t.v(0)));
        HH_SSTAT(Sylen, mag(t.v(1)));
        HH_SSTAT(Szlen, mag(t.v(2)));
        HH_SSTAT(Sxydot, dot(t.v(0), t.v(1)));
        HH_SSTAT(Syzdot, dot(t.v(1), t.v(2)));
        HH_SSTAT(Szxdot, dot(t.v(2), t.v(0)));
        return false;
    }
    for_int(irepeat, repeat) {
        if (delay) my_sleep(delay);
        if (!FrameIO::write(std::cout, t, obn, zoom, bin)) return true;
        ocount++;
        if (b_frame) std::cout << "f 0 0 0\n";
        if (ocount==1 && eof1) std::cout << "q 0 0 0\n";
        std::cout.flush();
    }
    return false;
}

void process() {
    for (;;) {
        if (loop()) break;
    }
    if (!std::cin.good() && !std::cin.eof()) {
        perror("read");
    } else if (!std::cout.good()) {
        perror("write");
    } else if (!icount) {
        SHOW("warning: no frames read");
    }
}

} // namespace

int main(int argc, const char** argv) {
    string transf;
    string pretransf;
    bool frame = false, stat = false;
    ParseArgs args(argc, argv);
    HH_ARGSD(create_euler,      "yaw pitch roll : create frame from Euler angles (degrees)");
    HH_ARGSC("",                ":**");
    HH_ARGSP(every,             "i : use only every ith element");
    HH_ARGSP(object,            "obn : force object # to obn");
    args.f("-inverse", ginverse, ": normalize normals");
    HH_ARGSP(pretransf,         "'frame' : pre-transform by frame");
    HH_ARGSP(transf,            "'frame' : post-transform by frame");
    HH_ARGSF(orthonormalize,    ": orthonormalize frame");
    HH_ARGSP(induce_roll,       "factor : for flight over terrain");
    HH_ARGSP(lowpass,           "factor : contribution of new frame [0..1]");
    HH_ARGSF(add_mid_frames,    ": add interpolating frame between each pair");
    HH_ARGSP(delay,             "fsec : introduce delay between frames");
    HH_ARGSP(repeat,            "n : repeat every frame n times");
    HH_ARGSF(frame,             ": introduce 'f 0 0 0' between frames");
    HH_ARGSF(stat,              ": output statistics");
    HH_ARGSF(eof1,              ": introduce 'q 0 0 0' after frame 1");
    HH_ARGSF(toasciit,          ": force output to be ascii text");
    HH_ARGSF(tobinary,          ": force output to be binary");
    args.parse();
    if (pretransf!="") {
        is_pretransf = true;
        cpretransf = FrameIO::parse_frame(pretransf);
    }
    if (transf!="") {
        is_transf = true;
        ctransf = FrameIO::parse_frame(transf);
    }
    statistics = stat;
    b_frame = frame;
    if (!noinput) process();
    hh_clean_up();
    return 0;
}
