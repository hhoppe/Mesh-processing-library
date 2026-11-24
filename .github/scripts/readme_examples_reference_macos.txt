# Reference values for the viewer screenshots taken by run_readme_examples.py on macOS, using Apple's software
# OpenGL renderer (the per-channel "mean sd" of each image, each optionally followed by a tolerance "tol=..."),
# checked using bin/check_reference_values.  Each file is named after a hash of the text of its block in
# progs/README.md and its index within the block.  Update them from the screenshots of a macOS CI run (uploaded in
# readme_examples/screenshots/) with "bin/check_reference_values --update REFERENCE_FILE DIR".

# FilterPM demos/data/standingblob.pm -info -nfaces 1000 -outmesh | (screenshot 1)
a5d28dee_1.png 229.17 66.16 229.17 66.16 229.17 66.16
# FilterPM demos/data/spheretext.pm -nf 2000 -outmesh | (screenshot 1)
0c5a400e_1.png 165.11 95.67 164.97 95.23 170.05 91.33
# Recon demos/data/distcap.pts -samplingd 0.02 | (screenshot 1)
8007d0da_1.png 191.70 86.65 191.70 86.65 191.97 86.47
# Recon demos/data/distcap.pts -samplingd 0.02 -what c | (screenshot 1)
bfab6ef7_1.png 225.09 62.79 162.31 106.24 148.26 121.21
# Recon demos/data/distcap.pts -samplingd 0.02 -what m | Filtermesh -toa3d | (screenshot 1)
4c5a220c_1.png 203.84 71.27 204.52 70.23 203.84 71.27
# Recon demos/data/curve1.pts -samplingd 0.06 -grid 30 | (screenshot 1)
da8d483d_1.png 254.76 4.90 254.55 8.00 254.45 9.74
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts -crep 1e-5 -reconstruct | (screenshot 1)
c3298c38_1.png 195.81 79.16 195.81 79.16 196.09 78.95
# Filtermesh demos/data/blob5.orig.m -randpts 10000 -vertexpts | (screenshot 1)
c520f53d_1.png 207.14 67.18 207.14 67.18 207.43 66.82
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts (screenshot 1)
5a7ea2ef_1.png 195.81 79.14 195.81 79.14 196.09 78.93
# Polyfit -pfile curve1.a3d -file demos/data/curve1.pts -crep 3e-4 -spring 1 -reconstruct | (screenshot 1)
5e0f6f3e_1.png 254.37 10.88 254.37 10.88 254.37 10.88
# G3dOGL distcap.sub0.m "Subdivfit -mf distcap.sub0.m -nsub 2 -outn |" (screenshot 1)
a8bcaba1_1.png 194.27 79.71 194.31 79.65 196.39 77.86
# PM_LOD_LEVEL=0.05 G3dOGL 57chevy.pm -st demos/data/57chevy.s3d -lightambient .4 -key De (screenshot 1)
d07ba9af_1.png 200.35 86.12 217.62 62.03 218.08 61.60
# FilterPM demos/data/standingblob.pm -nf 300 -truncate_prior -nf 10000 -truncate_beyond | (screenshot 1)
c4f3f287_1.png 219.03 62.77 219.03 62.77 219.03 62.77
# Filterimage demos/data/gaudipark.png -scaletox 200 -tomesh | (screenshot 1)
fa2c4131_1.png 143.31 77.96 158.87 74.17 173.44 77.60
# G3dOGL -eyeob demos/data/unit_frustum.a3d -sr_mode office.sr.pm -st demos/data/office_srfig.s3d (screenshot 1)
bade6723_1.png 202.17 74.19 202.34 73.95 202.17 74.19
# (common="-eyeob demos/data/unit_frustum.a3d -sr_mode gcanyon_sq200.pm -st demos/data/gcany (screenshot 1)
# Apple's software OpenGL renderer drops some large triangles that cross the near plane (Mesa clips them), so the
# coarse foreground of this terrain has build-dependent gaps; the means are unchecked and the tolerance of the
# standard deviations only rejects a blank image.
d8b6f22c_1.png 135.64 46.79 98.64 49.07 43.56 63.15 tol=5,0.95,5,0.95,5,0.95
# FilterPM demos/data/office.pm -nf 200000 -outmesh | (screenshot 1)
09b4df8a_1.png 237.97 27.65 237.97 27.65 238.01 27.50
# G3dOGL bunny.vertexcache.m -st demos/data/bunny.s3d -key DmDTDC (screenshot 1)
610509bb_1.png 198.48 89.69 197.12 91.53 228.38 53.24
# Filtermesh bunny.sphparam.m -renamekey v sph P | (screenshot 1)
66005128_1.png 139.78 112.04 139.78 112.04 139.78 112.04
# G3dOGL bunny.spheresample.remesh.m -st demos/data/bunny.s3d (screenshot 1)
2e3cacbb_1.png 197.95 88.32 197.95 88.32 198.70 87.63
# FilterPM demos/data/spheretext.pm -nf 4000 -outmesh | (screenshot 1)
349b0de4_1.png 236.91 56.00 236.91 56.00 236.91 56.00
