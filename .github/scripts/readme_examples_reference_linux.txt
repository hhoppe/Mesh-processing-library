# Reference values for the viewer screenshots taken by run_readme_examples.py on Linux, using Mesa's llvmpipe
# renderer (the per-channel "mean sd" of each image, each optionally followed by a tolerance "tol=..."), checked
# using bin/check_reference_values.  Each file is named after a hash of the text of its block in progs/README.md and
# its index within the block.  Update them with "run_readme_examples.py --update".

# FilterPM demos/data/standingblob.pm -info -nfaces 1000 -outmesh | (screenshot 1)
a5d28dee_1.png 229.66 64.51 229.66 64.51 229.66 64.51
# FilterPM demos/data/spheretext.pm -nf 2000 -outmesh | (screenshot 1)
0c5a400e_1.png 165.64 94.84 165.69 94.26 170.53 90.65
# Recon demos/data/distcap.pts -samplingd 0.02 | (screenshot 1)
8007d0da_1.png 192.30 85.28 192.30 85.28 192.60 85.05
# Recon demos/data/distcap.pts -samplingd 0.02 -what c | (screenshot 1)
bfab6ef7_1.png 225.05 63.33 162.32 106.34 148.26 121.30
# Recon demos/data/distcap.pts -samplingd 0.02 -what m | Filtermesh -toa3d | (screenshot 1)
4c5a220c_1.png 203.87 71.87 204.55 70.85 203.87 71.87
# Recon demos/data/curve1.pts -samplingd 0.06 -grid 30 | (screenshot 1)
da8d483d_1.png 254.87 4.80 254.67 7.11 254.51 9.12
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts -crep 1e-5 -reconstruct | (screenshot 1)
c3298c38_1.png 195.86 78.81 195.86 78.81 196.19 78.53
# Filtermesh demos/data/blob5.orig.m -randpts 10000 -vertexpts | (screenshot 1)
c520f53d_1.png 207.04 66.76 207.04 66.76 207.45 66.26
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts (screenshot 1)
5a7ea2ef_1.png 195.82 78.84 195.82 78.84 196.15 78.56
# Polyfit -pfile curve1.a3d -file demos/data/curve1.pts -crep 3e-4 -spring 1 -reconstruct | (screenshot 1)
5e0f6f3e_1.png 254.37 10.61 254.37 10.61 254.37 10.61
# G3dOGL distcap.sub0.m "Subdivfit -mf distcap.sub0.m -nsub 2 -outn |" (screenshot 1)
a8bcaba1_1.png 194.77 79.33 194.79 79.30 197.18 77.19
# PM_LOD_LEVEL=0.05 G3dOGL 57chevy.pm -st demos/data/57chevy.s3d -lightambient .4 -key De (screenshot 1)
d07ba9af_1.png 200.60 85.63 217.90 61.20 218.36 60.76
# FilterPM demos/data/standingblob.pm -nf 300 -truncate_prior -nf 10000 -truncate_beyond | (screenshot 1)
c4f3f287_1.png 219.07 62.79 219.07 62.79 219.07 62.79
# Filterimage demos/data/gaudipark.png -scaletox 200 -tomesh | (screenshot 1)
fa2c4131_1.png 144.47 76.20 159.95 72.13 174.43 75.52
# G3dOGL -eyeob demos/data/unit_frustum.a3d -sr_mode office.sr.pm -st demos/data/office_srfig.s3d (screenshot 1)
bade6723_1.png 202.19 74.23 202.37 73.96 202.19 74.23
# (common="-eyeob demos/data/unit_frustum.a3d -sr_mode gcanyon_sq200.pm -st demos/data/gcany (screenshot 1)
d8b6f22c_1.png 135.65 46.79 98.71 49.16 43.66 63.22
# FilterPM demos/data/office.pm -nf 200000 -outmesh | (screenshot 1)
09b4df8a_1.png 238.08 27.89 238.08 27.89 238.13 27.69
# G3dOGL bunny.vertexcache.m -st demos/data/bunny.s3d -key DmDTDC (screenshot 1)
610509bb_1.png 200.22 90.49 198.96 92.94 227.18 58.40
# Filtermesh bunny.sphparam.m -renamekey v sph P | (screenshot 1)
66005128_1.png 142.43 109.29 142.43 109.29 142.43 109.29
# G3dOGL bunny.spheresample.remesh.m -st demos/data/bunny.s3d (screenshot 1)
2e3cacbb_1.png 199.54 84.89 199.54 84.89 200.30 84.17
# FilterPM demos/data/spheretext.pm -nf 4000 -outmesh | (screenshot 1)
349b0de4_1.png 238.60 62.55 238.60 62.55 238.60 62.55
