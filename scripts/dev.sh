#!/usr/bin/env bash

readonly image_name="orb-slam3-dev"

if [[ -z "$(sudo docker images -q $image_name 2> /dev/null)" ]]; then
    sudo docker build . -t $image_name
fi

sudo docker run --rm -it \
    -v .:/ORB_SLAM3 \
    $image_name
