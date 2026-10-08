# Reference values for the viewer screenshots taken by run_readme_examples.py on Linux, using Mesa's llvmpipe
# renderer (the per-channel "mean sd" of each image, each optionally followed by a tolerance "tol=..."), checked
# using bin/check_reference_values.  Each file is named after a hash of the text of its block in progs/README.md and
# its index within the block.  Update them with "run_readme_examples.py --update".

# FilterPM demos/data/standingblob.pm -info -nfaces 1000 -outmesh | (screenshot 1)
6df341b8_1.png 229.09 66.23 229.09 66.23 229.09 66.23
# FilterPM demos/data/spheretext.pm -nf 2000 -outmesh | (screenshot 1)
c1711555_1.png 164.90 95.73 164.80 95.25 170.22 91.13
# Recon <demos/data/distcap.pts -samplingd 0.02 | (screenshot 1)
345cdca1_1.png 191.53 86.63 191.53 86.63 191.88 86.40
# Recon <demos/data/distcap.pts -samplingd 0.02 -what c | (screenshot 1)
3ff46737_1.png 225.06 62.79 162.32 106.21 148.26 121.19
# Recon <demos/data/distcap.pts -samplingd 0.02 -what m | Filtermesh -toa3d | (screenshot 1)
5299d02f_1.png 203.41 71.82 204.09 70.80 203.41 71.82
# Recon <demos/data/curve1.pts -samplingd 0.06 -grid 30 | (screenshot 1)
cb9da702_1.png 254.74 5.20 254.53 8.04 254.43 9.71
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts -crep 1e-5 -reconstruct | (screenshot 1)
2849faad_1.png 195.63 79.22 195.63 79.22 195.99 78.95
# Filtermesh demos/data/blob5.orig.m -randpts 10000 -vertexpts | (screenshot 1)
4e901b46_1.png 206.85 67.41 206.85 67.41 207.31 66.84
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts (screenshot 1)
1f8ea1c5_1.png 195.65 79.20 195.65 79.20 196.01 78.92
# Polyfit -pfile curve1.a3d -file demos/data/curve1.pts -crep 3e-4 -spring 1 -reconstruct | (screenshot 1)
a68fe7c5_1.png 254.36 10.72 254.36 10.72 254.36 10.72
# G3dOGL distcap.sub0.m "Subdivfit -mf distcap.sub0.m -nsub 2 -outn |" (screenshot 1)
a8bcaba1_1.png 194.16 79.79 194.17 79.77 196.42 77.78
# PM_LOD_LEVEL=0.05 G3dOGL -pm_mode club.pm -st demos/data/club.s3d -lightambient .4 -key De (screenshot 1)
eb40eefc_1.png 226.47 59.31 226.47 59.31 226.47 59.31
# FilterPM demos/data/standingblob.pm -nf 300 -truncate_prior -nf 10000 -truncate_beyond | (screenshot 1)
018dc595_1.png 218.54 62.99 218.54 62.99 218.54 62.99
# Filterimage demos/data/gaudipark.png -scaletox 200 -tomesh | (screenshot 1)
7551c05c_1.png 143.41 77.63 158.91 73.85 173.43 77.26
# G3dOGL -eyeob demos/data/unit_frustum.a3d -sr_mode office.sr.pm -st demos/data/office_srfig.s3d (screenshot 1)
dd707799_1.png 226.94 66.50 227.22 66.00 226.94 66.50
# (common="-eyeob demos/data/unit_frustum.a3d -sr_mode gcanyon_sq200.pm -st demos/data/gcanyon_fly_v98.s3d (screenshot 1)
1ce9dfe6_1.png 135.54 46.77 98.54 48.97 43.54 63.05
# FilterPM demos/data/office.pm -nf 200000 -outmesh | (screenshot 1)
4e13147d_1.png 238.09 27.62 238.09 27.62 238.14 27.45
# G3dOGL bunny.vertexcache.m -st demos/data/bunny.s3d -key DmDTDC (screenshot 1)
610509bb_1.png 198.42 89.50 197.07 91.26 228.68 52.48
# Filtermesh bunny.sphparam.m -renamekey v sph P | (screenshot 1)
7da6b12f_1.png 139.71 111.94 139.71 111.94 139.71 111.94
# G3dOGL bunny.spheresample.remesh.m -st demos/data/bunny.s3d (screenshot 1)
2e3cacbb_1.png 197.77 88.17 197.77 88.17 198.62 87.37
# FilterPM demos/data/spheretext.pm -nf 4000 -outmesh | (screenshot 1)
c79d8a1d_1.png 236.95 54.68 236.95 54.68 236.95 54.68
