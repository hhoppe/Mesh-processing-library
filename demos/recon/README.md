# Surface reconstruction

Point sets (`*.pts`, as text A3d streams) from the original experiments in surface reconstruction (1993-1994),
with their viewer state files (`*.s3d`).
See [Recon](../../progs/README.md#prog_recon) for how to convert them to and from PLY files.

- `recon_process` reconstructs a surface from a point set in three phases: `Recon` (followed by `Filtermesh`,
  which removes small components and fills holes), `Meshfit`, and `Subdivfit`.
  Its parameters are the environment variables `RECON_*` listed at the top of the script, and it writes its
  results into `results/` (relative to the current directory).
  The demos [`create_reconstruction_animated.sh`](../create_reconstruction_animated.sh) and
  [`create_reconstruction_mannequin.sh`](../create_reconstruction_mannequin.sh) also use it, and
  `recon_process.bat` does the same in the Windows command prompt.
  With `RECON_RECORD=1`, it also writes the stages that
  [`view_reconstruction_animated.sh`](../view_reconstruction_animated.sh) shows as an animation.
- `recon_process_all` reconstructs all these point sets, each with its own parameters (in about two minutes).
  `make -C demos recon`, which the nightly CI runs, calls it and checks the results against
  `results_reference.txt`.

For example:

```bash
RECON_ROOT=mechpart RECON_SAMPLINGD=.035 RECON_GRID=60 RECON_FILLHOLES=0 RECON_ANGLE=50 RECON_RECORD=1 ./recon_process
RECON_PTS=recon/mechpart.pts RECON_OUT=recon/results/mechpart RECON_S3D=recon/mechpart.s3d \
  ../view_reconstruction_animated.sh
```
