# bin

Scripts that complement the programs, and the directories into which the programs are built.

- `mesh_to_pm` creates a progressive mesh from a mesh (using `MeshSimplify` and `Filterprog`), and
  `pm_simplify` further simplifies the base mesh of a progressive mesh.
- `obj_to_mesh`, `ply_to_mesh`, `mesh_to_obj`, and `mesh_to_ply` (Perl scripts) convert between the mesh format
  (`*.m`) and Wavefront `*.obj` or Stanford `*.ply` files; `Materials_lib.pm` and `Materials_rgb.db` provide
  the material colors used by `obj_to_mesh`.
- `hcheck` runs the [unit tests](../make/README.md#unit-tests) and compares their outputs with the expected ones.
- `ensure_x11_server` starts an X server under Cygwin, for the viewers of the `cygwin` build.
- `build_and_test_using_clang` and `build_and_test_using_gcc` are simple alternatives to `make`,
  compiling all files sequentially.

The directories `unix`, `win`, `clang`, `mingw`, and `cygwin` receive the executables built by `make` for the
corresponding [configuration](../make/README.md#build-using-gnu-make), and `msbuild` and `msbuild_debug` those
built by Visual Studio.
