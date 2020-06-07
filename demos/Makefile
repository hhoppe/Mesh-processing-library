MeshRoot = ..
orig_CONFIG := $(CONFIG)#  it may be null, in which case we use the executables created by msbuild in $(MeshRoot)/bin
include $(MeshRoot)/make/Makefile_defs

ifneq ($(CONFIG),all)

ifeq ($(orig_CONFIG),)
# undefine CONFIG#  recent feature of make
CONFIG:=
unexport CONFIG
$(call prepend_PATH,$(MeshRoot)/bin)
else
$(call prepend_PATH,$(MeshRoot)/bin/$(CONFIG))
endif

$(call prepend_PATH,$(MeshRoot)/demos)  # allow running bash scripts in local directory

create_scripts = \
  create_recon_cactus.sh \
  create_recon_distcap.sh \
  create_recon_polygon.sh \
  create_simplified_using_meshopt.sh \
  create_pm_gaudipark.sh \
  create_pm_club.sh \
  determine_approximation_error.sh \
  create_geomorphs.sh \
  create_sr_office.sh \
  create_sr_terrain.sh \
  create_terrain_hierarchy.sh \
  create_topologically_simplified.sh \
  create_rendered_mechpart_image.sh \
  create_rendered_mechpart_video.sh \
  create_voronoi_fillin.sh \

create_scripts_other = \
  create_gallery.sh \

view_scripts = \
  view_recon_cactus.sh \
  view_recon_distcap.sh \
  view_recon_polygon.sh \
  view_simplified_using_meshopt.sh \
  view_pm_gaudipark.sh \
  view_pm_club.sh \
  view_geomorphs.sh \
  view_sr_office.sh \
  view_sr_terrain.sh \
  view_terrain_hierarchy.sh \
  view_gcanyon_frames.sh \
  view_psc_drumset.sh \
  view_topologically_simplified.sh \
  view_rendered_mechpart_image.sh \
  view_rendered_mechpart_video.sh \
  view_voronoi_fillin.sh \
  view_hidden_line_removed.sh \

all:
	Filterimage --version   # show build information
	$(MAKE) create
	ls -alt data
	$(MAKE) view

# Similar to all_demos_create_results.sh but with parallel execution
create: $(create_scripts)

$(create_scripts):
	$(@)

# Just in case these are specified individually.
$(view_scripts):
	$(@)

depend $(make_dep):

# any build-order dependencies
determine_approximation_error.sh: create_pm_club.sh

# Similar to all_demos_view_results.sh (still sequential)
view:
	for i in $(view_scripts); do $$i; done

clean deepclean:
# (relative pathname because prepend_PATH does not update PATH when make goal is clean)
	./all_demos_clean.sh

debug:
	which Recon

.PHONY: all create view clean deepclean $(create_scripts) $(view_scripts) debug depend $(make_dep)

endif  # ifneq ($(CONFIG),all)
