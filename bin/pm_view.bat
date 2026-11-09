@echo off

:: View a progressive mesh using G3dOGL; this is the Windows counterpart of the bash script pm_view.
::
:: Examples:
::  pm_view demos/results/cessna.pm -st demos/data/cessna.s3d -key Da
::  mesh_to_pm.bat <demos/data/57chevy.orig.m | pm_view - -st demos/data/57chevy.s3d -key Da
::  g3d_cmp -g3d pm_view demos/results/bunny.outer_hull.pm demos/results/bunny.inner_hull.pm ^
::    -key De -st demos/data/bunny.s3d

G3dOGL -pm_mode %*
