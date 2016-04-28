#!/bin/bash

# Examples:
#  meshtopm.sh <file.m >file.pm
#  meshtopm.sh file.m >file.pm
#  cat file.m | meshtopm.sh -numpts 100000 >file.numpts100000.pm

if [[ -n "$TEMP" ]]; then export TMPDIR="$TEMP"; fi  # sometimes necessary to prevent Cygwin mktemp from using /tmp
tmpprog=$(mktemp --suffix=.prog)
tmpbase=$(mktemp --suffix=.base.m)
tmprprog=$(mktemp --suffix=.prog)

cleanup() {
  rm -f "$tmpprog" "$tmpbase" "$tmprprog"
}

# (The trap fails when running bash within Windows emacs bash shell because cygwin bash does not detect a tty.)
trap '{ cleanup; exit 255; }' SIGINT SIGQUIT

MeshSimplify "$@" -prog "$tmpprog" -simplify >"$tmpbase" || { t=$?; cleanup ; exit $t; }

reverselines "$tmpprog" >"$tmprprog" || { t=$?; cleanup; exit $t; }
if true; then rm "$tmpprog"; fi
Filterprog -fbase "$tmpbase" -fprog "$tmprprog" -pm || { t=$?; cleanup; exit $t; }

cleanup
