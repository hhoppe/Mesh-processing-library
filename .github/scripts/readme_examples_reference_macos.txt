# Reference values for the viewer screenshots taken by run_readme_examples.py on macOS, using Apple's software
# OpenGL renderer (the per-channel "mean sd" of each image, each optionally followed by a tolerance "tol=..."),
# checked using bin/check_reference_values.  Each file is named after a hash of the text of its block in
# progs/README.md and its index within the block.  Update them from the screenshots of a macOS CI run (uploaded in
# readme_examples/screenshots/) with "bin/check_reference_values --update REFERENCE_FILE DIR".

# FilterPM demos/data/standingblob.pm -info -nfaces 1000 -outmesh | (screenshot 1)
6df341b8_1.png
# FilterPM demos/data/spheretext.pm -nf 2000 -outmesh | (screenshot 1)
c1711555_1.png
# Recon <demos/data/distcap.pts -samplingd 0.02 | (screenshot 1)
345cdca1_1.png
# Recon <demos/data/distcap.pts -samplingd 0.02 -what c | (screenshot 1)
3ff46737_1.png
# Recon <demos/data/distcap.pts -samplingd 0.02 -what m | Filtermesh -toa3d | (screenshot 1)
5299d02f_1.png
# Recon <demos/data/curve1.pts -samplingd 0.06 -grid 30 | (screenshot 1)
cb9da702_1.png
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts -crep 1e-5 -reconstruct | (screenshot 1)
2849faad_1.png
# Filtermesh demos/data/blob5.orig.m -randpts 10000 -vertexpts | (screenshot 1)
4e901b46_1.png
# Meshfit -mfile distcap.recon.m -file demos/data/distcap.pts (screenshot 1)
1f8ea1c5_1.png
# Polyfit -pfile curve1.a3d -file demos/data/curve1.pts -crep 3e-4 -spring 1 -reconstruct | (screenshot 1)
a68fe7c5_1.png
# G3dOGL distcap.sub0.m "Subdivfit -mf distcap.sub0.m -nsub 2 -outn |" (screenshot 1)
a8bcaba1_1.png
# PM_LOD_LEVEL=0.05 G3dOGL -pm_mode club.pm -st demos/data/club.s3d -lightambient .4 -key De (screenshot 1)
eb40eefc_1.png
# FilterPM demos/data/standingblob.pm -nf 300 -truncate_prior -nf 10000 -truncate_beyond | (screenshot 1)
018dc595_1.png
# Filterimage demos/data/gaudipark.png -scaletox 200 -tomesh | (screenshot 1)
7551c05c_1.png
# G3dOGL -eyeob demos/data/unit_frustum.a3d -sr_mode office.sr.pm -st demos/data/office_srfig.s3d (screenshot 1)
dd707799_1.png
# (common="-eyeob demos/data/unit_frustum.a3d -sr_mode gcanyon_sq200.pm -st demos/data/gcany (screenshot 1)
1ce9dfe6_1.png
# FilterPM demos/data/office.pm -nf 200000 -outmesh | (screenshot 1)
4e13147d_1.png
# G3dOGL bunny.vertexcache.m -st demos/data/bunny.s3d -key DmDTDC (screenshot 1)
610509bb_1.png
# Filtermesh bunny.sphparam.m -renamekey v sph P | (screenshot 1)
7da6b12f_1.png
# G3dOGL bunny.spheresample.remesh.m -st demos/data/bunny.s3d (screenshot 1)
2e3cacbb_1.png
# FilterPM demos/data/spheretext.pm -nf 4000 -outmesh | (screenshot 1)
c79d8a1d_1.png
