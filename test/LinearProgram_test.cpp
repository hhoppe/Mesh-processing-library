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

}  // namespace

int main() {
  test_small();
  test_beale_cycling();
  test_degenerate_apex();
  test_random_3d();
  test_near_coplanar();
  test_captured_hull_problems();
}
