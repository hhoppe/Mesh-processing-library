// -*- C++ -*-  Copyright (c) Microsoft Corporation; see LICENSE.
#include "libHh/LinearProgram.h"

namespace hh {

// The linear program is converted to the standard form: maximize dot(cost, y) subject to t * y == rhs and y >= 0,
// with rhs >= 0.  Each unrestricted variable x[k] becomes y[k] - y[n + k], each inequality row gets a slack
// variable, and each row that lacks an initial basic variable (an equality, or an inequality whose right-hand side
// is negative) gets an artificial variable.  Phase 1 drives the artificial variables to zero, and phase 2 optimizes
// the objective while excluding them.  Bland's rule (the lowest-index entering column, and the lowest-index basic
// variable among tied ratios) guarantees termination.  To limit the accumulation of roundoff, the tableau is
// recomputed from the original coefficients after any small pivot, whenever the tableau becomes inconsistent, and
// before reading the final solution.
LpResult maximize_linear_program(CArrayView<double> c, CMatrixView<double> a_le, CArrayView<double> b_le,
                                 CMatrixView<double> a_eq, CArrayView<double> b_eq) {
  const int n = c.num(), m_le = b_le.num(), m_eq = b_eq.num();
  assertx(a_le.ysize() == m_le && (m_le == 0 || a_le.xsize() == n));
  assertx(a_eq.ysize() == m_eq && (m_eq == 0 || a_eq.xsize() == n));
  // Since each row is normalized to a maximum coefficient magnitude of 1, as is the objective, the tolerance is
  // absolute for coefficients and reduced costs, and relative to the largest right-hand side for feasibility.
  const double tolerance = 1e-9;
  LpResult result{ELpStatus::infeasible, {}, 0.};

  // Gather the normalized rows, omitting those whose coefficients are all zero.
  struct Row {
    Array<double> coefs;
    double rhs;
    int slack_sign;  // +1 or -1 for an inequality (-1 after negation), or 0 for an equality.
  };
  Array<Row> rows;
  const auto add_row = [&](CArrayView<double> coefs, double rhs, bool is_equality) {
    const double scale = coefs.num() ? max_abs_element(coefs) : 0.;
    if (scale == 0.) return is_equality ? abs(rhs) <= tolerance : rhs >= -tolerance;
    Row row{Array<double>(coefs), rhs, is_equality ? 0 : 1};
    for (double& coef : row.coefs) coef /= scale;
    row.rhs /= scale;
    if (row.rhs < 0.) {
      for (double& coef : row.coefs) coef = -coef;
      row.rhs = -row.rhs;
      row.slack_sign = -row.slack_sign;
    }
    rows.push(std::move(row));
    return true;
  };
  for_int(i, m_le) if (!add_row(a_le[i], b_le[i], false)) return result;
  for_int(i, m_eq) if (!add_row(a_eq[i], b_eq[i], true)) return result;

  // Columns: [0, 2 * n) for the split variables, then the m_le slacks, then the artificials, then the rhs.
  const int nrows = rows.num();
  const int first_slack = 2 * n, first_artificial = first_slack + m_le;
  const int num_artificials = int(ranges::count_if(rows, [](const Row& row) { return row.slack_sign != 1; }));
  const int ncols = first_artificial + num_artificials;
  Matrix<double> t(V(nrows, ncols + 1), 0.);
  Array<int> basis(nrows);
  {
    int slack = first_slack, artificial = first_artificial;
    for_int(i, nrows) {
      const Row& row = rows[i];
      for_int(k, n) t[i, k] = row.coefs[k], t[i, n + k] = -row.coefs[k];
      if (row.slack_sign != 0) t[i, slack] = row.slack_sign;
      if (row.slack_sign == 1) {
        basis[i] = slack;
      } else {
        t[i, artificial] = 1.;
        basis[i] = artificial++;
      }
      if (row.slack_sign != 0) slack++;
      t[i, ncols] = row.rhs;
    }
  }
  const Matrix<double> t0(t);  // The original tableau, for refactorization.
  const double max_rhs = nrows ? max(rows | views::transform([](const Row& row) { return row.rhs; })) : 0.;

  Array<double> d(ncols + 1);  // Reduced costs; d[ncols] is the negated objective value.
  const auto set_cost = [&](CArrayView<double> cost) {
    for_int(j, ncols) d[j] = cost[j];
    d[ncols] = 0.;
    for_int(i, nrows) {
      const double cb = cost[basis[i]];
      if (cb) for_int(j, ncols + 1) d[j] -= cb * t[i, j];
    }
  };
  const auto pivot = [&](int p, int q) {
    const double inv = 1. / t[p, q];
    for_int(j, ncols + 1) t[p, j] *= inv;
    t[p, q] = 1.;
    for_int(i, nrows) {
      if (i == p || t[i, q] == 0.) continue;
      const double f = t[i, q];
      for_int(j, ncols + 1) t[i, j] -= f * t[p, j];
      t[i, q] = 0.;
      if (t[i, ncols] < 0.) t[i, ncols] = 0.;  // Remove any roundoff below the bound rhs >= 0.
    }
    if (const double f = d[q]; f != 0.) {
      for_int(j, ncols + 1) d[j] -= f * t[p, j];
      d[q] = 0.;
    }
    basis[p] = q;
  };
  // Recompute the tableau as inverse(B) * t0 from the original coefficients, where B = t0[:, basis], using
  // Gauss-Jordan elimination with partial pivoting.  This removes the roundoff accumulated over the pivots.
  // Return false if B is numerically singular.
  const auto refactor = [&] {
    Matrix<double> bmat(V(nrows, nrows));
    for_int(i, nrows) for_int(k, nrows) bmat[i, k] = t0[i, basis[k]];
    Matrix<double> nt(t0);
    for_int(k, nrows) {
      int pr = k;
      for_intL(i, k + 1, nrows) if (abs(bmat[i, k]) > abs(bmat[pr, k])) pr = i;
      if (abs(bmat[pr, k]) < 1e-12) return false;
      if (pr != k) {
        for_int(j, nrows) std::swap(bmat[k, j], bmat[pr, j]);
        for_int(j, ncols + 1) std::swap(nt[k, j], nt[pr, j]);
      }
      const double inv = 1. / bmat[k, k];
      for_intL(j, k, nrows) bmat[k, j] *= inv;
      for_int(j, ncols + 1) nt[k, j] *= inv;
      for_int(i, nrows) {
        if (i == k || bmat[i, k] == 0.) continue;
        const double f = bmat[i, k];
        for_intL(j, k, nrows) bmat[i, j] -= f * bmat[k, j];
        for_int(j, ncols + 1) nt[i, j] -= f * nt[k, j];
      }
    }
    // Row k of the result now corresponds to the basic variable basis[k].
    for_int(i, nrows) for_int(j, ncols + 1) t[i, j] = nt[i, j];
    for_int(i, nrows) t[i, ncols] = max(t[i, ncols], 0.);
    return true;
  };
  int num_pivots = 0;
  const int max_pivots = 1000 + 100 * (nrows + ncols);
  // Optimize the cost using the columns [0, num_enterable); return false if the objective is unbounded.  A column
  // whose reduced cost is positive although none of its entries is positive cannot occur in exact arithmetic for a
  // bounded objective.  It prompts a refactorization; if it persists, the column is reported as an unbounded
  // direction if its reduced cost is significant and unboundedness is possible, and is otherwise skipped.
  const double unbounded_tolerance = 1e-6;
  const auto optimize = [&](CArrayView<double> cost, int num_enterable, bool may_be_unbounded) {
    bool refactored = false;
    for (;;) {
      int p = -1, q = -1;
      bool suspicious = false;
      for_int(j, num_enterable) {
        if (!(d[j] > tolerance)) continue;
        // Among the rows attaining the minimum ratio, Bland's rule picks the one whose basic variable has the
        // lowest index.  Ratios count as tied if the difference would make no right-hand side more negative than
        // the tolerance.
        double min_ratio = std::numeric_limits<double>::infinity(), max_entry = 0.;
        for_int(i, nrows) {
          if (t[i, j] > tolerance) {
            min_ratio = min(min_ratio, t[i, ncols] / t[i, j]);
            max_entry = max(max_entry, t[i, j]);
          }
        }
        for_int(i, nrows) {
          if (t[i, j] > tolerance && (t[i, ncols] / t[i, j] - min_ratio) * max_entry <= tolerance &&
              (p < 0 || basis[i] < basis[p]))
            p = i;
        }
        if (p >= 0) {
          q = j;
          break;
        }
        if (d[j] > unbounded_tolerance) {
          if (!refactored) {
            suspicious = true;
            break;
          }
          if (may_be_unbounded) return false;
        }
      }
      if (suspicious) {
        (void)refactor();  // On failure, the pivoted tableau remains.
        set_cost(cost);
        refactored = true;
        continue;
      }
      if (q < 0) return true;
      const bool small_pivot = abs(t[p, q]) < 1e-4;
      pivot(p, q);
      // A pivot on a small element amplifies roundoff throughout the tableau.
      if (small_pivot && refactor()) set_cost(cost);
      refactored = false;
      if (++num_pivots > max_pivots) assertnever("Linear program exceeds the expected number of pivots");
    }
  };

  // Phase 1: maximize the negated sum of the artificial variables.
  if (num_artificials) {
    Array<double> cost(ncols, 0.);
    for_intL(j, first_artificial, ncols) cost[j] = -1.;
    set_cost(cost);
    // Phase 1 is bounded above by zero.
    assertx(optimize(cost, ncols, false));
    if (d[ncols] > tolerance * max(1., max_rhs)) return result;  // The artificials cannot all reach zero.
    // Pivot any remaining (zero-valued) basic artificial variable out of the basis; if its row has no nonzero
    // coefficient outside the artificial columns, the row is redundant and its artificial remains harmlessly basic.
    for_int(i, nrows) {
      if (basis[i] < first_artificial) continue;
      int q = -1;
      for_int(j, first_artificial) {
        if (abs(t[i, j]) > tolerance && (q < 0 || abs(t[i, j]) > abs(t[i, q]))) q = j;
      }
      if (q >= 0) {
        t[i, ncols] = 0.;
        pivot(i, q);
      }
    }
  }

  // Phase 2: maximize the normalized objective over the original and slack variables.
  {
    const double scale = n ? max_abs_element(c) : 0.;
    Array<double> cost(ncols, 0.);
    if (scale) for_int(k, n) cost[k] = c[k] / scale, cost[n + k] = -c[k] / scale;
    set_cost(cost);
    if (!optimize(cost, first_artificial, true)) {
      result.status = ELpStatus::unbounded;
      return result;
    }
  }

  // Read the basic solution from a refactored tableau, rather than from the tableau updated by the pivots.
  if (!refactor()) Warning("Linear program: singular basis; using the pivoted tableau");
  Array<double> y(ncols, 0.);
  for_int(i, nrows) y[basis[i]] = t[i, ncols];
  result.status = ELpStatus::optimal;
  result.x.init(n);
  for_int(k, n) result.x[k] = y[k] - y[n + k];
  result.value = dot(c, result.x);
  return result;
}

LpPointResult maximize_over_halfspaces(const Vector& direction, CArrayView<LinearFunc> halfspaces) {
  // Each constraint dot(lf.v, p) + lf.offset >= 0 becomes -dot(lf.v, p) <= lf.offset.
  const int m = halfspaces.num();
  Matrix<double> a_le(m, 3);
  Array<double> b_le(m);
  for_int(i, m) {
    for_int(c, 3) a_le[i, c] = -double(halfspaces[i].v[c]);
    b_le[i] = halfspaces[i].offset;
  }
  const Vec3<double> c{double(direction[0]), double(direction[1]), double(direction[2])};
  const LpResult result = maximize_linear_program(c, a_le, b_le);
  LpPointResult point_result{result.status, Point(0.f, 0.f, 0.f)};
  if (result.status == ELpStatus::optimal)
    point_result.point = Point(float(result.x[0]), float(result.x[1]), float(result.x[2]));
  return point_result;
}

}  // namespace hh
