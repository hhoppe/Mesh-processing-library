# bin

Subdirectories into which the programs are built, and scripts that complement the programs.

The directories `unix`, `win`, `clang`, `mingw`, and `cygwin` receive the executables built by `make` for the
corresponding [configuration](../make/README.md#build-using-gnu-make), and `msbuild` and `msbuild_debug` those
built by Visual Studio.

- `mesh_to_pm` creates a progressive mesh from a mesh (using `MeshSimplify` and `Filterprog`), and
  `pm_simplify` further simplifies the base mesh of a progressive mesh.
- `obj_to_mesh`, `ply_to_mesh`, `mesh_to_obj`, and `mesh_to_ply` (Perl scripts) convert between the mesh format
  (`*.m`) and Wavefront `*.obj` or Stanford `*.ply` files; `Materials_lib.pm` and `Materials_rgb.db` provide
  the material colors used by `obj_to_mesh`.
- `hcheck` runs the [unit tests](../make/README.md#unit-tests) and compares their outputs (`*.ou`)
  with the expected reference files (`*.ref`).
- `check_reference_values` checks generated files against reference values (statistics of images and videos,
  sizes of other files) within tolerances; it checks the demo results (`demos/check_created_outputs.sh`) and the
  screenshots and images of the README pages.
- `ensure_x11_server` starts an X server under Cygwin, for the viewers of the `cygwin` build.
- `build_and_test_using_clang` and `build_and_test_using_gcc` are simple alternatives to `make`,
  compiling all files sequentially.
