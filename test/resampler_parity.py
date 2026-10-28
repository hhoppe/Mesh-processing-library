#!/usr/bin/env python3
# Copyright (c) Hugues Hoppe; see LICENSE.
"""Generate Resample_parity.inc: results of the Python package resampler, for Resample_test.cpp.

The file defines the table `k_parity_cases`, whose entries `"<function> <parameters> = <values>"`
name a function of the package and its parameters, and list its results.  The values are rounded to
6 decimals (well within the 1e-5 tolerance of the test), so that regenerating the file changes it
only when a result really changes.  Each entry is on one line, which clang-format leaves alone.

Usage: make -C test resampler_parity
  (or: python3 -I test/resampler_parity.py > test/Resample_parity.inc)
"""

import itertools
import sys

import numpy as np
import resampler

BOUNDARIES = resampler.BOUNDARIES
FILTERS = 'box trapezoid triangle cubic lanczos3 mitchell cardinal3 cardinal5 omoms3 omoms5'.split()
# Source and destination grid types; the pairs other than dual/dual use a subset of the filters and
# no scaling.
GRIDTYPE_PAIRS = [('dual', 'dual'), ('primal', 'primal'), ('dual', 'primal'), ('primal', 'dual')]
SUBSET_FILTERS = 'box triangle cubic cardinal3 omoms5'.split()
SRC_SIZES = [3, 5]
DST_SIZES = [2, 7]
SCALE_TRANSLATE = [(1.0, 0.0), (0.7, 0.2)]
CVAL = -1.0
# The kernels usable in resample() (which excludes the adaptive trapezoid).
RESAMPLE_FILTERS = 'box triangle cubic lanczos3 mitchell cardinal3 omoms5'.split()
# Per-dimension (boundary, filter, gridtype) combinations of resample().
RESAMPLE_PER_DIM = [
    ('wrap,clamp', 'cubic,lanczos3', 'dual,primal'),
    ('reflect,natural', 'cardinal3,triangle', 'primal,dual'),
    ('tile,border', 'omoms5,cubic', 'dual,dual'),
]
UNIFORM_FILTERS = 'box trapezoid triangle cubic lanczos3'.split()
UNIFORM_SHAPES = [(2, 7), (5, 3), (3, 3)]
INTEGER_SOURCES = {'uint8': [0, 30, 200, 255, 90], 'uint16': [0, 9000, 50000, 65535, 20000]}
GAMMA_FILTERS = 'triangle cubic lanczos3'.split()
GAMMA_DST_SIZES = [3, 8]


def source_2d() -> np.ndarray:
  """Return the 2D source grid (y * y + 3 * x - x * y) / 16 of shape (4, 5)."""
  y, x = np.meshgrid(np.arange(4.0), np.arange(5.0), indexing='ij')
  return (y * y + 3 * x - x * y) / 16


def coords_line() -> np.ndarray:
  """Return 8 points along a line that leaves the unit domain at both ends (magnifying)."""
  i = np.arange(8.0)[:, None]
  return np.array([-0.25, -0.125]) + i * 0.1875


def coords_grid() -> np.ndarray:
  """Return a 3x4 grid of points, an affine map of the indices, partly outside the unit domain."""
  i, j = np.meshgrid(np.arange(3.0), np.arange(4.0), indexing='ij')
  return np.stack([-0.25 + 0.375 * i + 0.125 * j, -0.125 + 0.0625 * i + 0.3125 * j], axis=-1)


def per_dim(word: str) -> str | tuple[str, ...]:
  """Return a per-dimension parameter from a word `a` (the same in every dimension) or `a,b`."""
  return tuple(word.split(',')) if ',' in word else word


def format_value(value: float) -> str:
  """Return the value rounded to 6 decimals, without trailing zeros."""
  s = f'{value:.6f}'.rstrip('0').rstrip('.')
  return '0' if s == '-0' else s


def emit(spec: str, result: np.ndarray) -> None:
  values = ' '.join(format_value(float(v)) for v in result.flat)
  print(f'    "{spec} = {values}",')


def emit_resize() -> None:
  for src_gridtype, dst_gridtype in GRIDTYPE_PAIRS:
    full = src_gridtype == 'dual' and dst_gridtype == 'dual'
    filters = FILTERS if full else SUBSET_FILTERS
    scale_translates = SCALE_TRANSLATE if full else SCALE_TRANSLATE[:1]
    cases = itertools.product(BOUNDARIES, filters, SRC_SIZES, DST_SIZES, scale_translates)
    for boundary, filter, src_size, dst_size, (scale, translate) in cases:
      source = np.arange(src_size, dtype=np.float64) ** 2 / 25.0
      result = resampler.resize(
          source,
          (dst_size,),
          src_gridtype=src_gridtype,
          dst_gridtype=dst_gridtype,
          boundary=boundary,
          cval=CVAL,
          filter=filter,
          scale=scale,
          translate=translate,
      )
      parameters = f'{boundary} {filter} {src_gridtype} {dst_gridtype} {scale:g} {translate:g}'
      emit(f'resize {parameters} {src_size} {dst_size}', result)


def emit_resample() -> None:
  cases = [
      *itertools.product(['line'], BOUNDARIES, RESAMPLE_FILTERS, ['dual', 'primal']),
      *(('line', *case) for case in RESAMPLE_PER_DIM),
      *itertools.product(['grid'], BOUNDARIES, ['cubic', 'cardinal3'], ['dual']),
  ]
  for points, boundary, filter, gridtype in cases:
    coords = coords_line() if points == 'line' else coords_grid()
    result = resampler.resample(
        source_2d(),
        coords,
        gridtype=per_dim(gridtype),
        boundary=per_dim(boundary),
        cval=CVAL,
        filter=per_dim(filter),
    )
    emit(f'resample_{points} {boundary} {filter} {gridtype}', result)


def emit_uniform_resize() -> None:
  fits, gridtypes = ['contain', 'cover'], ['dual', 'primal']
  cases = itertools.product(fits, UNIFORM_FILTERS, gridtypes, UNIFORM_SHAPES)
  for object_fit, filter, gridtype, shape in cases:
    result = resampler.uniform_resize(
        source_2d(), shape, object_fit=object_fit, gridtype=gridtype, cval=CVAL, filter=filter
    )
    emit(f'uniform_resize {object_fit} {filter} {gridtype} {shape[0]} {shape[1]}', result)


def emit_gamma() -> None:
  cases = itertools.product(INTEGER_SOURCES, resampler.GAMMAS, GAMMA_FILTERS, GAMMA_DST_SIZES)
  for dtype, gamma, filter, dst_size in cases:
    source = np.array(INTEGER_SOURCES[dtype], dtype=dtype)
    result = resampler.resize(source, (dst_size,), gamma=gamma, filter=filter)
    emit(f'gamma {dtype} {gamma} {filter} {dst_size}', result)


def main() -> None:
  sys.stdout.reconfigure(newline='\n')
  print('// -*- C++ -*-  Copyright (c) Hugues Hoppe; see LICENSE.')
  print(f'// Generated by resampler_parity.py from resampler {resampler.__version__}; do not edit.')
  print('#include <string_view>\n')
  print('// The parity cases asserted by Resample_test.cpp: "<function> <parameters> = <values>".')
  print('// clang-format off')  # Keep each entry on one line.
  print('constexpr std::string_view k_parity_cases[] = {')
  emit_resize()
  emit_resample()
  emit_uniform_resize()
  emit_gamma()
  print('};')
  print('// clang-format on')


if __name__ == '__main__':
  main()
