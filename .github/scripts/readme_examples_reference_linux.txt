# Reference values for the viewer screenshots taken by run_readme_examples.py on Linux, using Mesa's llvmpipe
# renderer (the per-channel "mean sd" of each image, each optionally followed by a tolerance "tol=..."), checked
# using bin/check_reference_values.  Each file is named after a hash of the text of its block in progs/README.md and
# its index within the block.  Update them with "run_readme_examples.py --update".

# FilterPM demos/data/standingblob.pm -info -nfaces 1000 -outmesh | (screenshot 1)
6df341b8_1.png 229.59 64.71 229.59 64.71 229.59 64.71
# FilterPM demos/data/spheretext.pm -nf 2000 -outmesh | (screenshot 1)
c1711555_1.png 165.66 94.81 165.72 94.22 170.50 90.67
# Recon <demos/data/distcap.pts -samplingd 0.02 | (screenshot 1)
345cdca1_1.png 192.24 85.36 192.24 85.36 192.54 85.13
# Recon <demos/data/distcap.pts -samplingd 0.02 -what c | (screenshot 1)
3ff46737_1.png 225.05 63.33 162.32 106.34 148.26 121.30
# Recon <demos/data/distcap.pts -samplingd 0.02 -what m | Filtermesh -toa3d | (screenshot 1)
5299d02f_1.png 203.87 71.87 204.55 70.85 203.87 71.87
# Recon <demos/data/curve1.pts -samplingd 0.06 -grid 30 | (screenshot 1)
cb9da702_1.png 254.87 4.80 254.67 7.11 254.51 9.12
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts -crep 1e-5 -reconstruct | (screenshot 1)
2849faad_1.png 195.80 78.91 195.80 78.91 196.13 78.62
# Filtermesh demos/data/blob5.orig.m -randpts 10000 -vertexpts | (screenshot 1)
4e901b46_1.png 207.09 66.68 207.09 66.68 207.50 66.17
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts (screenshot 1)
1f8ea1c5_1.png 195.83 78.88 195.83 78.88 196.15 78.59
# Polyfit -pfile curve1.a3d -file demos/data/curve1.pts -crep 3e-4 -spring 1 -reconstruct | (screenshot 1)
a68fe7c5_1.png 254.37 10.63 254.37 10.63 254.37 10.63
# G3dOGL distcap.sub0.m "Subdivfit -mf distcap.sub0.m -nsub 2 -outn |" (screenshot 1)
a8bcaba1_1.png 194.38 79.56 194.41 79.51 196.43 77.76
# PM_LOD_LEVEL=0.05 G3dOGL -pm_mode club.pm -st demos/data/club.s3d -lightambient .4 -key De (screenshot 1)
eb40eefc_1.png 226.69 58.53 226.69 58.53 226.69 58.53
# FilterPM demos/data/standingblob.pm -nf 300 -truncate_prior -nf 10000 -truncate_beyond | (screenshot 1)
018dc595_1.png 218.54 63.06 218.54 63.06 218.54 63.06
# Filterimage demos/data/gaudipark.png -scaletox 200 -tomesh | (screenshot 1)
7551c05c_1.png 144.43 76.28 159.88 72.25 174.31 75.64
# G3dOGL -eyeob demos/data/unit_frustum.a3d -sr_mode office.sr.pm -st demos/data/office_srfig.s3d (screenshot 1)
dd707799_1.png 227.68 64.49 227.97 63.94 227.68 64.49
# (common="-eyeob demos/data/unit_frustum.a3d -sr_mode gcanyon_sq200.pm -st demos/data/gcany (screenshot 1)
1ce9dfe6_1.png 135.54 46.81 98.54 49.02 43.54 63.11
# FilterPM demos/data/office.pm -nf 200000 -outmesh | (screenshot 1)
4e13147d_1.png 238.09 27.88 238.09 27.88 238.13 27.69
# G3dOGL bunny.vertexcache.m -st demos/data/bunny.s3d -key DmDTDC (screenshot 1)
610509bb_1.png 200.22 90.49 198.96 92.94 227.18 58.40
# Filtermesh bunny.sphparam.m -renamekey v sph P | (screenshot 1)
7da6b12f_1.png 142.36 109.33 142.36 109.33 142.36 109.33
# G3dOGL bunny.spheresample.remesh.m -st demos/data/bunny.s3d (screenshot 1)
2e3cacbb_1.png 199.62 84.84 199.62 84.84 200.40 84.09
# FilterPM demos/data/spheretext.pm -nf 4000 -outmesh | (screenshot 1)
c79d8a1d_1.png 238.60 62.55 238.60 62.55 238.60 62.55
