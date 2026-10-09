# Reference values for the viewer screenshots taken by run_readme_examples.py on macOS, using Apple's software
# OpenGL renderer (the per-channel "mean sd" of each image, each optionally followed by a tolerance "tol=..."),
# checked using bin/check_reference_values.  Each file is named after a hash of the text of its block in
# progs/README.md and its index within the block.  Update them from the screenshots of a macOS CI run (uploaded in
# readme_examples/screenshots/) with "bin/check_reference_values --update REFERENCE_FILE DIR".

# FilterPM demos/data/standingblob.pm -info -nfaces 1000 -outmesh | (screenshot 1)
6df341b8_1.png 229.17 66.17 229.17 66.17 229.17 66.17
# FilterPM demos/data/spheretext.pm -nf 2000 -outmesh | (screenshot 1)
c1711555_1.png 165.09 95.70 164.97 95.25 170.07 91.34
# Recon <demos/data/distcap.pts -samplingd 0.02 | (screenshot 1)
345cdca1_1.png 191.68 86.67 191.68 86.67 191.95 86.48
# Recon <demos/data/distcap.pts -samplingd 0.02 -what c | (screenshot 1)
3ff46737_1.png 225.09 62.79 162.31 106.24 148.26 121.21
# Recon <demos/data/distcap.pts -samplingd 0.02 -what m | Filtermesh -toa3d | (screenshot 1)
5299d02f_1.png 203.84 71.27 204.52 70.23 203.84 71.27
# Recon <demos/data/curve1.pts -samplingd 0.06 -grid 30 | (screenshot 1)
cb9da702_1.png 254.76 4.90 254.55 8.00 254.45 9.74
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts -crep 1e-5 -reconstruct | (screenshot 1)
2849faad_1.png 195.85 79.17 195.85 79.17 196.10 78.99
# Filtermesh demos/data/blob5.orig.m -randpts 10000 -vertexpts | (screenshot 1)
4e901b46_1.png 207.00 67.40 207.00 67.40 207.29 67.04
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts (screenshot 1)
1f8ea1c5_1.png 195.85 79.17 195.85 79.17 196.09 78.99
# Polyfit -pfile curve1.a3d -file demos/data/curve1.pts -crep 3e-4 -spring 1 -reconstruct | (screenshot 1)
a68fe7c5_1.png 254.37 10.86 254.37 10.86 254.37 10.86
# G3dOGL distcap.sub0.m "Subdivfit -mf distcap.sub0.m -nsub 2 -outn |" (screenshot 1)
a8bcaba1_1.png 194.37 79.65 194.37 79.65 196.32 77.97
# PM_LOD_LEVEL=0.05 G3dOGL -pm_mode club.pm -st demos/data/club.s3d -lightambient .4 -key De (screenshot 1)
eb40eefc_1.png 226.53 59.32 226.53 59.32 226.53 59.32
# FilterPM demos/data/standingblob.pm -nf 300 -truncate_prior -nf 10000 -truncate_beyond | (screenshot 1)
018dc595_1.png 218.50 63.06 218.50 63.06 218.50 63.06
# Filterimage demos/data/gaudipark.png -scaletox 200 -tomesh | (screenshot 1)
7551c05c_1.png 143.34 77.98 158.82 74.23 173.30 77.63
# G3dOGL -eyeob demos/data/unit_frustum.a3d -sr_mode office.sr.pm -st demos/data/office_srfig.s3d (screenshot 1)
dd707799_1.png 228.55 64.62 228.76 64.24 228.55 64.62
# (common="-eyeob demos/data/unit_frustum.a3d -sr_mode gcanyon_sq200.pm -st demos/data/gcany (screenshot 1)
1ce9dfe6_1.png 135.64 46.79 98.64 49.07 43.56 63.15
# FilterPM demos/data/office.pm -nf 200000 -outmesh | (screenshot 1)
4e13147d_1.png 237.97 27.65 237.97 27.65 238.01 27.50
# G3dOGL bunny.vertexcache.m -st demos/data/bunny.s3d -key DmDTDC (screenshot 1)
610509bb_1.png 198.48 89.69 197.12 91.53 228.38 53.24
# Filtermesh bunny.sphparam.m -renamekey v sph P | (screenshot 1)
7da6b12f_1.png 139.75 112.06 139.75 112.06 139.75 112.06
# G3dOGL bunny.spheresample.remesh.m -st demos/data/bunny.s3d (screenshot 1)
2e3cacbb_1.png 197.98 88.28 197.98 88.28 198.73 87.58
# FilterPM demos/data/spheretext.pm -nf 4000 -outmesh | (screenshot 1)
c79d8a1d_1.png 236.91 56.00 236.91 56.00 236.91 56.00
