# Reference values for the viewer screenshots taken by run_readme_examples.py (the per-channel "mean sd" of each
# image, each optionally followed by a tolerance "tol=..."), checked using bin/check_reference_values.  Each file is
# named after a hash of the text of its block in progs/README.md and its index within the block.
# Update them with "run_readme_examples.py --update".

# FilterPM demos/data/standingblob.pm -info -nfaces 1000 -outmesh | (screenshot 1)
6df341b8_1.png 227.28 70.56 227.28 70.56 227.28 70.56
# FilterPM demos/data/spheretext.pm -nf 2000 -outmesh | (screenshot 1)
c1711555_1.png 162.50 98.12 162.32 97.72 164.35 95.75
# Recon <demos/data/distcap.pts -samplingd 0.02 | (screenshot 1)
345cdca1_1.png 188.08 91.59 188.08 91.59 188.08 91.59
# Recon <demos/data/distcap.pts -samplingd 0.02 -what c | (screenshot 1)
3ff46737_1.png 224.98 62.88 162.36 106.17 148.27 121.18
# Recon <demos/data/distcap.pts -samplingd 0.02 -what m | Filtermesh -toa3d | (screenshot 1)
5299d02f_1.png 202.05 73.28 203.19 71.54 202.05 73.28
# Recon <demos/data/curve1.pts -samplingd 0.06 -grid 30 | (screenshot 1)
cb9da702_1.png 254.68 5.64 254.39 9.46 254.25 11.60
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts -crep 1e-5 -reconstruct | (screenshot 1)
2849faad_1.png 194.01 81.83 194.01 81.83 194.01 81.83
# Filtermesh demos/data/blob5.orig.m -randpts 10000 -vertexpts | (screenshot 1)
4e901b46_1.png 203.93 72.42 203.93 72.42 203.93 72.42
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts (screenshot 1)
1f8ea1c5_1.png 194.04 81.81 194.04 81.81 194.29 81.64
# Polyfit -pfile curve1.a3d -file demos/data/curve1.pts -crep 3e-4 -spring 1 -reconstruct | (screenshot 1)
a68fe7c5_1.png 254.15 12.94 254.15 12.94 254.15 12.94
# G3dOGL distcap.sub0.m "Subdivfit -mf distcap.sub0.m -nsub 2 -outn |" (screenshot 1)
a8bcaba1_1.png 193.26 81.28 193.26 81.27 193.26 81.28
# PM_LOD_LEVEL=0.05 G3dOGL -pm_mode club.pm -st demos/data/club.s3d -lightambient .4 -key De (screenshot 1)
eb40eefc_1.png 224.96 63.00 224.96 63.00 224.96 63.00
# FilterPM demos/data/standingblob.pm -nf 300 -truncate_prior -nf 10000 -truncate_beyond | (screenshot 1)
018dc595_1.png 218.53 63.00 218.53 63.00 218.53 63.00
# Filterimage demos/data/gaudipark.png -scaletox 200 -tomesh | (screenshot 1)
7551c05c_1.png 139.39 81.04 154.92 78.08 168.36 82.33
# G3dOGL -eyeob demos/data/unit_frustum.a3d -sr_mode office.sr.pm -st demos/data/office_srfig.s3d (screenshot 1)
dd707799_1.png 221.82 74.66 222.07 74.28 221.82 74.66
# (common="-eyeob demos/data/unit_frustum.a3d -sr_mode gcanyon_sq200.pm -st demos/data/gcanyon_fly_v98.s3d (screenshot 1)
1ce9dfe6_1.png 135.14 47.61 97.99 49.39 43.66 63.14
# FilterPM demos/data/office.pm -nf 200000 -outmesh | (screenshot 1)
4e13147d_1.png 238.72 27.59 238.72 27.59 238.76 27.46
# G3dOGL bunny.vertexcache.m -st demos/data/bunny.s3d -key DmDTDC (screenshot 1)
610509bb_1.png 204.48 81.14 203.16 83.28 223.24 56.95
# Filtermesh bunny.sphparam.m -renamekey v sph P | (screenshot 1)
7da6b12f_1.png 136.58 115.26 136.58 115.26 136.58 115.26
# G3dOGL bunny.spheresample.remesh.m -st demos/data/bunny.s3d (screenshot 1)
2e3cacbb_1.png 192.32 95.39 192.32 95.39 192.32 95.39
# FilterPM demos/data/spheretext.pm -nf 4000 -outmesh | (screenshot 1)
c79d8a1d_1.png 230.60 66.25 230.60 66.25 230.60 66.25
