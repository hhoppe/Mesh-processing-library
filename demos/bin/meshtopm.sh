#!/bin/bash

# Examples:
#  meshtopm.sh <file.m >file.pm
#  meshtopm.sh file.m >file.pm
#  cat file.m | meshtopm.sh -numpts 100000 >file.numpts100000.pm

# set -x  # Echo expanded commands.

# Concerns:
# - on Mac, old version of mktemp does not support "--suffix".
# - on Cygwin, mktemp sometimes uses /tmp even with my TMPDIR override.
# if [[ -n $TEMP ]]; then export TMPDIR="$TEMP"; fi  # sometimes necessary to prevent Cygwin mktemp from using /tmp
# tmpprog=$(mktemp --suffix=.prog)
# tmpbase=$(mktemp --suffix=.base.m)

tmpd=${TEMP:-${TMPDIR:-/tmp}}
[[ $OSTYPE == cygwin ]] && tmpd=$(cygpath -m "$tmpd")  # Native Windows programs cannot open /tmp.
tmproot="$tmpd"/v.$$
tmpprog="$tmproot".prog
tmpbase="$tmproot".base.m

cleanup() {
  rm -f "$tmpprog" "$tmpbase"
}

# (The trap fails when running bash within Windows emacs bash shell because cygwin bash does not detect a tty.)
trap '{ cleanup; exit 255; }' SIGINT SIGQUIT

MeshSimplify "$@" -prog "$tmpprog" -simplify >"$tmpbase" || { t=$?; cleanup ; exit $t; }

Filterprog -fbase "$tmpbase" -fprog "$tmpprog" -pm || { t=$?; cleanup; exit $t; }

cleanup
