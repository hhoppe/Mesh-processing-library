#!/bin/bash

cd "$(dirname "${BASH_SOURCE[0]}")"
source bin/_initdemos.sh

echo '.'
echo 'Press "Do" to toggle outside_view_frustum view.'
echo 'After a while, press "Dr" to toggle top view.'
echo 'Also, press "De" to toggle edges off/on.'
echo '.'

G3dOGL -eyeob data/unit_frustum.a3d -sr_mode data/office.nf80000.sr.pm -st data/office_srfig.s3d -key ,DnDeDoo---J -lightambient .4 -sr_screen_thresh .002 -frustum_frac 2 -edgecolor '#505050' $G3DARGS
