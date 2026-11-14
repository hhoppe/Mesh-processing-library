@echo off

:: View a Stanford ply mesh using G3dOGL; this is the Windows counterpart of the bash script ply_view.
::
:: Examples:
::  ply_view bunny.poissonrecon.ply -st bunny.s3d -key DmDb
::  g3d_cmp -g3d ply_view bunny.poissonrecon.ply bunny.other.ply -key DmDb -st bunny.s3d

G3dOGL -ply_mode %*
