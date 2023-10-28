#!/bin/bash

rootdir=$(dirname "$(readlink -f "$0")")
cd $rootdir
srcdir=${rootdir}/../../../src

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
# touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run -it --rm --name ford_berkeley_project \
       --volume=$XSOCK:$XSOCK:rw \
       --volume=$XAUTH:$XAUTH:rw \
       --env="XAUTHORITY=${XAUTH}" \
       --env="DISPLAY" \
       --device=/dev/dri/card0:/dev/dri/card0 \
       -p 6511:6511/udp \
       -v "${srcdir}:/home/oski/catkin_ws/src" \
       --user="oski" \
       ford-berkeley-project-melodic \
       /entrypoint.sh
