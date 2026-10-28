// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#ifndef MESH_PROCESSING_LIBHH_LINEARPROGRAM_H_
#define MESH_PROCESSING_LIBHH_LINEARPROGRAM_H_

#include "libHh/Array.h"
#include "libHh/Geometry.h"
#include "libHh/LinearFunc.h"
#include "libHh/Matrix.h"

namespace hh {

// Outcome of solving a linear program.
enum class ELpStatus { optimal, infeasible, unbounded };

struct LpResult {
  ELpStatus status;
  Array<double> x;   // The optimal solution (empty unless status == ELpStatus::optimal).
  double value{0.};  // The optimal value dot(c, x).
};

// Find a vector x in R^n maximizing dot(c, x), where n = c.num(), subject to a_le * x <= b_le and a_eq * x == b_eq.
// Each matrix has n columns (unless it has no rows), and the variables are unrestricted in sign.  If the optimum is
// not unique (e.g., c is orthogonal to a face of the feasible polytope), some optimal vertex is returned.
// The solver is a dense two-phase simplex method using Bland's rule, so it terminates even on degenerate problems,
// and it refactors its tableau from the original coefficients to bound the accumulation of roundoff.
// To minimize instead, negate c.
[[nodiscard]] LpResult maximize_linear_program(CArrayView<double> c, CMatrixView<double> a_le, CArrayView<double> b_le,
                                               CMatrixView<double> a_eq, CArrayView<double> b_eq);

// Same, without equality constraints.
[[nodiscard]] inline LpResult maximize_linear_program(CArrayView<double> c, CMatrixView<double> a_le,
                                                      CArrayView<double> b_le) {
  return maximize_linear_program(c, a_le, b_le, Matrix<double>(), Array<double>());
}

struct LpPointResult {
  ELpStatus status;
  Point point;  // The optimal point (meaningful only if status == ELpStatus::optimal).
};

// Find a point p maximizing dot(direction, p) while also satisfying lf.eval(p) >= 0 for every lf in halfspaces.
[[nodiscard]] LpPointResult maximize_over_halfspaces(const Vector& direction, CArrayView<LinearFunc> halfspaces);

}  // namespace hh

#endif  // MESH_PROCESSING_LIBHH_LINEARPROGRAM_H_
