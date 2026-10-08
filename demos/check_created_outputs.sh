#!/bin/bash

# Check that the files written into results/ by the create scripts look reasonable, using the reference values
# (statistics of each image or video, errors of MeshDistance output, or sizes of other files) in
# results_reference.txt; see ../bin/check_reference_values for the measures and their tolerances.
# Usage: check_created_outputs.sh [--update]
#  --update: rewrite the reference values from the current files instead of checking them.

cd "$(dirname "${BASH_SOURCE[0]}")"
source ./_initdemos.sh

exec check_reference_values "$@" results_reference.txt results
