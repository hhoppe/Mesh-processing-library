# Surface reconstruction

Point sets (`*.pts`, as text A3d streams) from the original experiments in surface reconstruction (1993-1994),
with their viewer state files (`*.s3d`).
See [Recon](../../progs/README.md#prog_recon) for how to convert them to and from PLY files.

- `recon_process` reconstructs a surface from a point set in three phases: `Recon` (followed by `Filtermesh`,
  which removes small components and fills holes), `Meshfit`, and `Subdivfit`, as in the demo
  [`create_reconstruction_animated.sh`](../create_reconstruction_animated.sh).
  Its parameters are the environment variables `RECON_*` listed at the top of the script, and it writes its
  results into `results/`.
  With `RECON_VIEW=1`, it also shows the reconstruction as an animation, using
  [`view_reconstruction_animated.sh`](../view_reconstruction_animated.sh).
- `recon_process_all` reconstructs all these point sets, each with its own parameters (in about two minutes).
  It is one of the scripts run by `make -C demos create`, which by default sets `DEMOS_RECON_QUICK=1` so that it
  stops after the first two point sets (in about 15 seconds); the nightly CI sets `DEMOS_RECON_QUICK=0`.

For example:

```bash
RECON_ROOT=mechpart RECON_SAMPLINGD=.035 RECON_GRID=60 RECON_FILLHOLES=0 RECON_ANGLE=50 RECON_VIEW=1 ./recon_process
```
