#!/bin/bash
exposure=112
wb=3850
v0=0
v1=1
if [[ "$1" == "local"* ]];then
  v0=${1:5:1}
  v1=${1:6:1}
fi
./conf_cam.sh -e $exposure -wb $wb /dev/video$v0
./conf_cam.sh -e $exposure -wb $wb /dev/video$v1
