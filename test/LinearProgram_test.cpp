// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#include "libHh/LinearProgram.h"

#include <sstream>  // std::istringstream

#include "libHh/Random.h"
#include "libHh/RangeOp.h"
using namespace hh;

namespace {

std::string_view status_name(ELpStatus status) {
  switch (status) {
    case ELpStatus::optimal: return "optimal";
    case ELpStatus::infeasible: return "infeasible";
    case ELpStatus::unbounded: return "unbounded";
    default: assertnever("");
  }
}

// Round to 6 decimals, mapping -0 to 0, so that the output is identical across configurations.
double round6(double v) {
  const double r = std::round(v * 1e6) / 1e6;
  return r == 0. ? 0. : r;
}

void show(const string& name, const LpResult& result) {
  string s = name + ": " + string(status_name(result.status));
  if (result.status == ELpStatus::optimal) {
    s += sform(" value=%g x=(", round6(result.value));
    for_int(k, result.x.num()) s += sform("%s%g", k ? " " : "", round6(result.x[k]));
    s += ')';
  }
  showf("%s\n", s.c_str());
}

Matrix<double> matrix(int ysize, int xsize, CArrayView<double> values) {
  assertx(values.num() == ysize * xsize);
  Matrix<double> m(ysize, xsize);
  for_int(y, ysize) for_int(x, xsize) m[y, x] = values[y * xsize + x];
  return m;
}

void test_small() {
  {  // Textbook example: maximize 3x + 5y with x <= 4, 2y <= 12, 3x + 2y <= 18, x >= 0, y >= 0.
    const Matrix<double> a = matrix(5, 2, V(1., 0., 0., 2., 3., 2., -1., 0., 0., -1.));
    show("textbook", maximize_linear_program(V(3., 5.), a, V(4., 12., 18., 0., 0.)));
  }
  {  // Contradictory bounds x <= 1 and x >= 2.
    show("infeasible", maximize_linear_program(V(1.), matrix(2, 1, V(1., -1.)), V(1., -2.)));
  }
  {  // Maximize x with only x >= 0.
    show("unbounded", maximize_linear_program(V(1.), matrix(1, 1, V(-1.)), V(0.)));
  }
  {  // The variables are unrestricted in sign: minimize x + y with x >= -3, y >= -2.
    show("negative", maximize_linear_program(V(-1., -1.), matrix(2, 2, V(-1., 0., 0., -1.)), V(3., 2.)));
  }
  {  // Equality: maximize x + y with x + 2y == 4 and x <= 2.
    show("equality",
         maximize_linear_program(V(1., 1.), matrix(1, 2, V(1., 0.)), V(2.), matrix(1, 2, V(1., 2.)), V(4.)));
  }
  {  // Redundant equalities x + y == 2 and 2x + 2y == 4 (and a negative right-hand side) with x <= 5.
    show("redundant", maximize_linear_program(V(1., 0.), matrix(1, 2, V(1., 0.)), V(5.),
                                              matrix(3, 2, V(1., 1., 2., 2., -1., -1.)), V(2., 4., -2.)));
  }
  {  // Inconsistent equalities x == 1 and x == 2.
    show("inconsistent",
         maximize_linear_program(V(1.), matrix(1, 1, V(1.)), V(5.), matrix(2, 1, V(1., 1.)), V(1., 2.)));
  }
  {  // Rows with all-zero coefficients: 0 <= 1 holds, whereas 0 <= -1 does not.
    show("zero_row_ok", maximize_linear_program(V(1.), matrix(2, 1, V(0., 1.)), V(1., 3.)));
    show("zero_row_bad", maximize_linear_program(V(1.), matrix(2, 1, V(0., 1.)), V(-1., 3.)));
  }
  {  // A zero objective returns some feasible point.
    const LpResult result = maximize_linear_program(V(0., 0.), matrix(2, 2, V(1., 0., 0., 1.)), V(-1., -1.));
    assertx(result.status == ELpStatus::optimal && result.x[0] <= -1. + 1e-9 && result.x[1] <= -1. + 1e-9);
    SHOW(result.value);
  }
  {  // Scale invariance: the textbook example with the constraints scaled by 1e-6 and the objective by 1e8.
    const Matrix<double> a = matrix(5, 2, V(1e-6, 0., 0., 2e-6, 3e-6, 2e-6, -1e-6, 0., 0., -1e-6));
    show("scaled", maximize_linear_program(V(3e8, 5e8), a, V(4e-6, 12e-6, 18e-6, 0., 0.)));
  }
}

// Beale's example, on which the simplex method cycles forever when it enters the column of largest reduced cost
// and breaks ratio ties by the lowest row.
void test_beale_cycling() {
  const Matrix<double> a = matrix(7, 4,
                                  V(.25, -8., -1., 9.,  //
                                    .5, -12., -.5, 3.,  //
                                    0., 0., 1., 0.,     //
                                    -1., 0., 0., 0.,    //
                                    0., -1., 0., 0.,    //
                                    0., 0., -1., 0.,    //
                                    0., 0., 0., -1.));
  show("beale", maximize_linear_program(V(.75, -20., .5, -6.), a, V(0., 0., 1., 0., 0., 0., 0.)));
}

// A highly degenerate vertex: many planes through the apex of a pyramid, as in the progressive hull.
void test_degenerate_apex() {
  const int n = 12;
  Array<LinearFunc> halfspaces;
  const Point apex(1.f, 2.f, 3.f);
  for_int(i, n) {
    const float angle = float(i) * TAU / n;
    const Vector normal = -Vector(std::cos(angle), std::sin(angle), 1.f);  // The interior is below the apex.
    halfspaces.push(LinearFunc(normal, apex));
    halfspaces.push(LinearFunc(normal, apex));  // Duplicate constraints.
  }
  halfspaces.push(LinearFunc(Vector(0.f, 0.f, 1.f), Point(0.f, 0.f, 0.f)));  // z >= 0.
  const LpPointResult result = maximize_over_halfspaces(Vector(0.f, 0.f, 1.f), halfspaces);
  assertx(result.status == ELpStatus::optimal);
  assertx(dist(result.point, apex) < 1e-5f);
  showf("apex: optimal\n");
  const LpPointResult result2 = maximize_over_halfspaces(Vector(0.f, 0.f, -1.f), halfspaces);
  assertx(result2.status == ELpStatus::optimal && abs(result2.point[2]) < 1e-5f);
  showf("apex_bottom: optimal\n");
  halfspaces.pop();
  showf("apex_unbounded: %s\n",
        string(status_name(maximize_over_halfspaces(Vector(0.f, 0.f, -1.f), halfspaces).status)).c_str());
}

// Compare random bounded problems in 3D against the brute-force enumeration of the vertices of their polytopes.
void test_random_3d() {
  Random random(17);
  const int num_problems = 300;
  int num_agree = 0;
  for_int(iproblem, num_problems) {
    const int m = 4 + iproblem % 30;
    // Halfspaces dot(normal, p) <= 1 with random unit normals, plus a box |p[c]| <= 3 that bounds the polytope.
    Matrix<double> a(m + 6, 3);
    Array<double> b(m + 6);
    for_int(i, m) {
      Vec3<double> normal;
      for_int(c, 3) normal[c] = random.dunif() * 2. - 1.;
      normal /= mag(normal);
      for_int(c, 3) a[i, c] = normal[c];
      b[i] = 1. + (iproblem % 3 == 0 ? 0. : random.dunif());
    }
    for_int(c, 3) for_int(s, 2) {
      const int i = m + c * 2 + s;
      for_int(cc, 3) a[i, cc] = cc == c ? (s ? -1. : 1.) : 0.;
      b[i] = 3.;
    }
    Vec3<double> obj;
    for_int(c, 3) obj[c] = random.dunif() * 2. - 1.;
    const LpResult result = maximize_linear_program(obj, a, b);
    assertx(result.status == ELpStatus::optimal);
    for_int(i, a.ysize()) assertx(dot(a[i], result.x) <= b[i] + 1e-9);
    // Brute force: the maximum of the objective over all feasible intersections of three planes.
    double best = -BIGFLOAT;
    const int nrows = a.ysize();
    for_int(i, nrows) for_intL(j, i + 1, nrows) for_intL(k, j + 1, nrows) {
      const Vec3<double> r0(a[i, 0], a[i, 1], a[i, 2]), r1(a[j, 0], a[j, 1], a[j, 2]), r2(a[k, 0], a[k, 1], a[k, 2]);
      const double det = dot(r0, cross(r1, r2));
      if (abs(det) < 1e-12) continue;
      const Vec3<double> p = (b[i] * cross(r1, r2) + b[j] * cross(r2, r0) + b[k] * cross(r0, r1)) / det;
      bool feasible = true;
      for_int(l, nrows) {
        if (dot(a[l], p) > b[l] + 1e-9) {
          feasible = false;
          break;
        }
      }
      if (feasible) best = max(best, dot(obj, p));
    }
    if (abs(best - result.value) < 1e-7) num_agree++;
  }
  showf("random_3d: %d of %d agree with brute force\n", num_agree, num_problems);
}

// Problems like those of the progressive hull: the tangent planes at the vertices of a fan on a gently curved
// surface, translated far from the origin, so that the planes are nearly coplanar and their vertices are degenerate
// and ill-conditioned.
void test_near_coplanar() {
  Random random(23);
  const int num_problems = 300;
  int num_agree = 0;
  double max_violation = 0.;
  for_int(iproblem, num_problems) {
    const int m = 5 + iproblem % 12;
    const double curvature = std::pow(10., -1. - 3. * random.dunif());  // From 1e-1 to 1e-4.
    const Vec3<double> offset(10. + random.dunif(), 10. + random.dunif(), 10. + random.dunif());
    // The surface z = curvature * (x^2 + y^2); the region above its tangent planes, z >= tangent plane.
    Matrix<double> a(m + 1, 3);
    Array<double> b(m + 1);
    for_int(i, m) {
      const double angle = (i + .3 * random.dunif()) * D_TAU / m, radius = .5 + .5 * random.dunif();
      const double x = radius * std::cos(angle), y = radius * std::sin(angle);
      const double z = curvature * (x * x + y * y);
      const Vec3<double> grad(2. * curvature * x, 2. * curvature * y, -1.);  // Outward normal (pointing down).
      const Vec3<double> p = Vec3<double>(x, y, z) + offset;
      for_int(c, 3) a[i, c] = grad[c];
      b[i] = dot(grad, p);
    }
    // An upper bound z <= offset[2] + 1 keeps the polytope bounded.
    a[m, 0] = 0., a[m, 1] = 0., a[m, 2] = 1.;
    b[m] = offset[2] + 1.;
    // Minimize the height plus a small tilt, i.e., push the point down into the tangent planes.
    const Vec3<double> obj(.1 * (random.dunif() - .5), .1 * (random.dunif() - .5), -1.);
    const LpResult result = maximize_linear_program(obj, a, b);
    assertx(result.status == ELpStatus::optimal);
    for_int(i, a.ysize()) max_violation = max(max_violation, dot(a[i], result.x) - b[i]);
    double best = -BIGFLOAT;
    const int nrows = a.ysize();
    for_int(i, nrows) for_intL(j, i + 1, nrows) for_intL(k, j + 1, nrows) {
      const Vec3<double> r0(a[i, 0], a[i, 1], a[i, 2]), r1(a[j, 0], a[j, 1], a[j, 2]), r2(a[k, 0], a[k, 1], a[k, 2]);
      const double det = dot(r0, cross(r1, r2));
      if (abs(det) < 1e-15) continue;
      const Vec3<double> p = (b[i] * cross(r1, r2) + b[j] * cross(r2, r0) + b[k] * cross(r0, r1)) / det;
      bool feasible = true;
      for_int(l, nrows) {
        if (dot(a[l], p) > b[l] + 1e-9) {
          feasible = false;
          break;
        }
      }
      if (feasible) best = max(best, dot(obj, p));
    }
    if (abs(best - result.value) < 1e-6) num_agree++;
  }
  showf("near_coplanar: %d of %d agree with brute force\n", num_agree, num_problems);
  assertx(max_violation < 1e-9);
}

// Problems captured from the progressive hull of a mesh (in its earlier coordinates, far from the origin), on
// which an earlier version of the solver either failed in phase 1 or returned a basis violating a constraint.
// Each string holds n, m, c[n], and m rows of a_le[n] and b_le.
void test_captured_hull_problems() {
  const Vec3<std::string_view> problems = {
      "3 10 -0.59741425514221191 -0.46844622492790222 0.65088737010955811 -0.42254012823104858 "
      "-0.90610402822494507 0.02086760476231575 -13.115317344665527 -0.59643083810806274 "
      "-0.42871332168579102 0.67858332395553589 -3.4394311904907227 -0.42252084612846375 "
      "-0.90629470348358154 0.010299018584191799 -13.22389030456543 -0.42249783873558044 "
      "-0.90612578392028809 0.020771922543644905 -13.116073608398438 -0.5964013934135437 "
      "-0.4287227988243103 0.67860305309295654 -3.4390308856964111 -0.40463528037071228 "
      "-0.91410619020462036 0.026079164817929268 -12.963034629821777 -0.40423843264579773 "
      "-0.91428297758102417 0.026037709787487984 -12.961238861083984 -0.42245957255363464 "
      "-0.90614187717437744 0.020852833986282349 -13.115034103393555 -0.59641462564468384 "
      "-0.42871823906898499 0.67859423160552979 -3.439206600189209 -0.59615761041641235 "
      "-0.42879214882850647 0.67877352237701416 -3.435556173324585",
      "3 16 0.40298193693161011 -0.28548493981361389 -0.86954241991043091 0.55023032426834106 "
      "-0.018086310476064682 -0.83481699228286743 -3.0267295837402344 0.5502479076385498 "
      "-0.018090814352035522 -0.83480536937713623 -3.0264809131622314 -0.52117270231246948 "
      "-0.85277599096298218 -0.033940568566322327 -15.034089088439941 -0.52131736278533936 "
      "-0.85269790887832642 -0.033681981265544891 -15.032093048095703 -0.52040910720825195 "
      "-0.85318666696548462 -0.035311236977577209 -15.044668197631836 -0.52117228507995605 "
      "-0.85277628898620605 -0.033940497785806656 -15.034088134765625 -0.5214957594871521 "
      "-0.85259991884231567 -0.03340144082903862 -15.030001640319824 -0.52096503973007202 "
      "-0.85289293527603149 -0.034193802624940872 -15.035811424255371 0.55021458864212036 "
      "-0.018080580979585648 -0.8348274827003479 -3.0269346237182617 -0.5208696722984314 "
      "-0.85294616222381592 -0.034318067133426666 -15.036678314208984 0.55011832714080811 "
      "-0.018042748793959618 -0.83489173650741577 -3.0281574726104736 -0.25119227170944214 "
      "-0.86082339286804199 -0.44258970022201538 -16.500316619873047 -0.52115851640701294 "
      "-0.85278671979904175 -0.033892035484313965 -15.0335693359375 0.54992187023162842 "
      "-0.017994508147239685 -0.83502215147018433 -3.0309646129608154 0.55030721426010132 "
      "-0.018099470064043999 -0.83476608991622925 -3.0255694389343262 0.55022025108337402 "
      "-0.018082352355122566 -0.83482372760772705 -3.0268580913543701",
      "3 10 -0.89359414577484131 -0.028826029971241951 0.44794943928718567 -0 0 -1 -10 -0 -0 -1 -10 "
      "-0.84387123584747314 -0.030049778521060944 0.53570359945297241 -3.3875546455383301 "
      "-0.85025638341903687 -0 0.52636885643005371 -3.2417750358581543 -0.86669903993606567 "
      "-0.02905365452170372 0.49798446893692017 -3.9885399341583252 -0.86167633533477783 "
      "-9.6795123027959562e-09 0.50745820999145508 -3.5478906631469727 -0.84879392385482788 "
      "-0.059842895716428757 0.5253261923789978 -3.8438217639923096 -0.85025632381439209 -0 "
      "0.52636873722076416 -3.2417752742767334 -0.86020785570144653 -0.058551639318466187 "
      "0.50657111406326294 -4.1352238655090332 -0.84870398044586182 -0.059662405401468277 "
      "0.52549195289611816 -3.8394122123718262",
  };
  for (const std::string_view problem : problems) {
    std::istringstream iss{string(problem)};
    int n, m;
    iss >> n >> m;
    Array<double> c(n);
    for_int(k, n) iss >> c[k];
    Matrix<double> a(m, n);
    Array<double> b(m);
    for_int(i, m) {
      for_int(k, n) iss >> a[i, k];
      iss >> b[i];
    }
    assertx(iss);
    const LpResult result = maximize_linear_program(c, a, b);
    assertx(result.status == ELpStatus::optimal);
    for_int(i, m) assertx(dot(a[i], result.x) <= b[i] + 1e-9);
    showf("captured: value=%g\n", round6(result.value));
  }
}

// Problems captured from the progressive hull of the demo mesh standingblob, in normalized coordinates, for
// maximize_over_halfspaces().  Earlier versions of the solver returned for them points violating constraints by up
// to 0.5 (owing to roundoff-level coefficients in the normals of faces nearly aligned with the axes, and to the
// clamping of slightly negative basic variables), or reported the infeasible ones as optimal.  Each string holds n
// (== 3), m, the direction c[3], and m rows of a[3] and b, for the halfspaces dot(-a, p) + b >= 0.
void test_captured_halfspace_problems() {
  const Vec<std::string_view, 7> problems = {
      "3 10 0.6556870341300964 -0.7337393164634705 0.17804840207099915 -0.5211580395698547 -0.8527839779853821 "
      "-0.03396671637892723 -0.44958168268203735 -0.5211524367332458 -0.8527868390083313 -0.03398197144269943 "
      "-0.4495859444141388 -0.5211266279220581 -0.8528026938438416 -0.03398110717535019 -0.44959136843681335 "
      "0.48026999831199646 -0.54994136095047 0.6833046674728394 -0.075831338763237 0.412159264087677 "
      "-0.9089211225509644 -0.06314423680305481 0.2639235854148865 0.8606716990470886 -0.43870171904563904 "
      "0.2584281265735626 -0.08767127245664597 0.946813702583313 -0.30764418840408325 -0.0943344309926033 "
      "-0.14167074859142303 0.42990022897720337 -0.9012196063995361 -0.05467189848423004 0.2548849880695343 "
      "0.8607044219970703 -0.43868738412857056 0.25834327936172485 -0.08768056333065033 0.4947686791419983 "
      "-0.8663545846939087 -0.06807011365890503 0.22230497002601624",
      "3 7 0.03658690303564072 0.08017570525407791 -0.9961090087890625 -0.16341879963874817 0.10051412880420685 "
      "-0.9814230799674988 -0.007497857324779034 -0.5212159752845764 -0.8527633547782898 -0.03359325975179672 "
      "-0.4231642782688141 0.028716394677758217 0.08753950893878937 -0.9957471489906311 -0.0014500594697892666 "
      "0.02858223207294941 0.08755404502153397 -0.9957495927810669 -0.0014523336431011558 0.21576540172100067 "
      "0.06250610202550888 -0.9744425415992737 -0.007352809887379408 0.028643259778618813 0.087543785572052 "
      "-0.99574875831604 -0.0014529682230204344 -0.5210879445075989 -0.8527706861495972 -0.0353475846350193 "
      "-0.4232352674007416",
      "3 7 0.9209326505661011 -0.10775966197252274 0.3745277225971222 0.07624344527721405 0.9955825805664062 "
      "-0.05479370430111885 -0.39011114835739136 0.9490311741828918 0.0062453001737594604 -0.31512048840522766 "
      "-0.19899657368659973 0.8522254824638367 -0.47880032658576965 -0.21086034178733826 -0.12428677082061768 "
      "-3.5169449574823375e-09 5.367285993429505e-08 1.0 -0.5 0.9288944005966187 -0.3147164285182953 "
      "-0.19521428644657135 -0.12600883841514587 -3.352267015088728e-07 -0.9153112769126892 0.4027472138404846 "
      "-0.3196412920951843 2.6598975799707603e-10 5.722432661059429e-08 1.0 -0.5",
      "3 16 0.28448355197906494 0.284464567899704 -0.9155047535896301 0.606360912322998 0.6149827241897583 "
      "0.5041056871414185 -0.17014943063259125 0.5524730682373047 0.6466109156608582 0.525992214679718 "
      "-0.16439874470233917 0.6519224643707275 0.55826735496521 0.5131613612174988 -0.1676516979932785 -0.0 -0.0 "
      "-1.0 -0.08091393858194351 -2.998035313339642e-07 4.206079893265269e-07 -1.0 -0.08091399073600769 -0.0 -0.0 "
      "-1.0 -0.08091393858194351 -0.0 1.0 -1.993631428831577e-07 -0.5 -0.0 -0.0 -1.0 -0.08091393858194351 1.0 "
      "4.011950949234233e-08 -1.9936315709401242e-07 -0.49999919533729553 -0.0 -0.0 -1.0 -0.08091393858194351 -0.0 "
      "-0.0 -1.0 -0.08091393858194351 3.4742319598990434e-07 -3.440857199166203e-07 -1.0 -0.08091393858194351 "
      "-4.05659284297144e-06 2.0460738596739247e-06 -1.0 -0.08091293275356293 -0.0 -0.0 -1.0 -0.08091393858194351 "
      "-0.0 -0.0 -0.9999999403953552 -0.08091393113136292 -0.0 -0.0 -1.0 -0.08091393858194351",
      "3 18 0.2899244427680969 0.28471460938453674 -0.9137184619903564 0.816653311252594 0.26362767815589905 "
      "0.5133983492851257 -0.173011913895607 0.817740797996521 0.2993888854980469 0.4915957450866699 "
      "-0.17823976278305054 0.7793386578559875 0.38876858353614807 0.4914165139198303 -0.1782323569059372 "
      "0.8000526428222656 0.3516436815261841 0.4860682487487793 -0.1796886920928955 0.7553468942642212 "
      "0.40998461842536926 0.5112374424934387 -0.17348577082157135 -0.0 -0.0 -1.0 -0.07038391381502151 "
      "-2.998035313339642e-07 4.206079893265269e-07 -1.0 -0.0703839659690857 -0.0 -0.0 -1.0 -0.07038391381502151 "
      "-0.0 1.0 -1.993631428831577e-07 -0.4999999701976776 -0.0 -0.0 -1.0 -0.07038391381502151 1.0 "
      "4.011950949234233e-08 -1.9936315709401242e-07 -0.49999916553497314 -0.0 -0.0 -1.0 -0.07038391381502151 -0.0 "
      "-0.0 -1.0 -0.07038391381502151 3.4742319598990434e-07 -3.440857199166203e-07 -1.0 -0.07038391381502151 "
      "-4.05659284297144e-06 2.0460738596739247e-06 -1.0 -0.07038290798664093 -0.0 -0.0 -1.0 -0.07038391381502151 "
      "-0.0 -0.0 -0.9999999403953552 -0.07038390636444092 -0.0 -0.0 -1.0 -0.07038391381502151",
      "3 6 -0.19728171825408936 -8.892975245089474e-09 0.9803469181060791 -3.999141123101459e-15 -1.0 "
      "-1.9936311446144828e-07 -0.5 -0.9999999403953552 -0.0 -0.0 -0.49999913573265076 -1.5094416170064883e-15 1.0 "
      "-1.993631428831577e-07 -0.4999999701976776 -1.0 -0.0 -0.0 -0.49999916553497314 7.571316906762604e-09 "
      "6.224533333210047e-09 1.0 -0.1006181463599205 3.868913633908733e-09 2.522135611471299e-09 1.0 "
      "-0.1006181463599205",
      "3 6 -0.19728173315525055 -2.27473040403936e-09 0.9803469181060791 -3.999141123101459e-15 -1.0 "
      "-1.9936311446144828e-07 -0.5 -0.9999999403953552 -0.0 -0.0 -0.49999913573265076 -1.5094414052482514e-15 "
      "0.9999999403953552 -1.9936311446144828e-07 -0.4999999403953552 -0.9999999403953552 -0.0 -0.0 "
      "-0.49999913573265076 7.571316906762604e-09 6.224533333210047e-09 1.0 -0.1006181463599205 "
      "3.868913633908733e-09 2.522135611471299e-09 1.0 -0.1006181463599205",
  };
  for (const std::string_view problem : problems) {
    std::istringstream iss{string(problem)};
    int n, m;
    iss >> n >> m;
    assertx(n == 3);
    Vec3<float> c;
    for_int(k, 3) iss >> c[k];
    Array<LinearFunc> halfspaces(m);
    for (LinearFunc& lf : halfspaces) {
      for_int(k, 3) {
        iss >> lf.v[k];
        lf.v[k] = -lf.v[k];
      }
      iss >> lf.offset;
    }
    assertx(iss);
    const LpPointResult result = maximize_over_halfspaces(Vector(c[0], c[1], c[2]), halfspaces);
    string s = string(status_name(result.status));
    if (result.status == ELpStatus::optimal) {
      for (const LinearFunc& lf : halfspaces) assertx(lf.eval(result.point) >= -1e-5f);
      s += sform(" value=%g", round6(dot(Vector(c[0], c[1], c[2]), result.point)));
    }
    showf("captured_halfspaces: %s\n", s.c_str());
  }
}

// Solve the problems read from std::cin (with the argument --solve), for a fuzzer that compares the results with
// those of the HiGHS solver of SciPy; that script is kept outside this repository, in
// ~/other/claude/mesh_processing/tools/fuzz_linear_program.py.  Each input line holds a kind
// ("lp" for maximize_linear_program(), or "hs" for maximize_over_halfspaces() with n == 3), n, m_le, m_eq, c[n],
// the m_le rows of a_le[n] and b_le, and the m_eq rows of a_eq[n] and b_eq.  For a problem of kind "hs", each row
// of a_le and b_le is the halfspace LinearFunc(-a_le[i], b_le[i]), and m_eq is zero.  Each output line holds the
// status and, if optimal, the value and x[n]; it is flushed, so that the output identifies a problem that aborts.
void solve_problems() {
  std::cout.precision(17);
  string line;
  while (std::getline(std::cin, line)) {
    std::istringstream iss{line};
    string kind;
    int n, m_le, m_eq;
    iss >> kind >> n >> m_le >> m_eq;
    Array<double> c(n), b_le(m_le), b_eq(m_eq);
    Matrix<double> a_le(m_le, n), a_eq(m_eq, n);
    for_int(k, n) iss >> c[k];
    for_int(i, m_le) {
      for_int(k, n) iss >> a_le[i, k];
      iss >> b_le[i];
    }
    for_int(i, m_eq) {
      for_int(k, n) iss >> a_eq[i, k];
      iss >> b_eq[i];
    }
    assertx(iss);
    LpResult result;
    if (kind == "lp") {
      result = maximize_linear_program(c, a_le, b_le, a_eq, b_eq);
    } else {
      assertx(kind == "hs" && n == 3 && m_eq == 0);
      Array<LinearFunc> halfspaces;
      for_int(i, m_le) {
        LinearFunc lf;
        lf.v = Vector(-float(a_le[i, 0]), -float(a_le[i, 1]), -float(a_le[i, 2]));
        lf.offset = float(b_le[i]);
        halfspaces.push(lf);
      }
      const LpPointResult point_result =
          maximize_over_halfspaces(Vector(float(c[0]), float(c[1]), float(c[2])), halfspaces);
      result.status = point_result.status;
      if (result.status == ELpStatus::optimal) {
        result.x.init(3);
        for_int(k, 3) result.x[k] = point_result.point[k];
        result.value = dot(c, result.x);
      }
    }
    std::cout << status_name(result.status);
    if (result.status == ELpStatus::optimal) {
      std::cout << ' ' << result.value;
      for (const double x : result.x) std::cout << ' ' << x;
    }
    std::cout << '\n' << std::flush;
  }
}

}  // namespace

int main(int argc, const char** argv) {
  if (argc == 2 && string(argv[1]) == "--solve") {
    solve_problems();
    return 0;
  }
  test_small();
  test_beale_cycling();
  test_degenerate_apex();
  test_random_3d();
  test_near_coplanar();
  test_captured_hull_problems();
  test_captured_halfspace_problems();
}
