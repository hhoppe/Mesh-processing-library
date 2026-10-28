MeshRoot = ..

new_all: test

# Regenerate Resample_parity.inc with the Python package resampler (importable by python3), and rerun its test.
resampler_parity:
	python3 -I resampler_parity.py >Resample_parity.inc.new
	mv Resample_parity.inc.new Resample_parity.inc
	$(MAKE) Resample_test.ou
phony_targets = resampler_parity

include $(MeshRoot)/make/Makefile_progs
