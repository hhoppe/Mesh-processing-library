#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo '.'
echo 'Use right button to drag slider up/down.'
echo '.'

export PSC_LOD_LEVEL="0.1"

G3dOGL -psc_mode data/drumset_trunc.psc data/drumset.box.a3d -st data/drumset.s3d -key , -lightambient .4 $G3DARGS
