#!/usr/bin/env bash

readonly image_name="orb-slam3-dev"

sudo docker build . -t $image_name

sudo docker run --rm -it \
    -v .:/ORB_SLAM3 \
    $image_name
