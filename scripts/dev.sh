#!/usr/bin/env bash

if [[ -z "$(sudo docker images -q orb-slam3-dev 2> /dev/null)" ]]; then
    sudo docker build . -t orb-slam3-dev
fi
sudo docker run -v .:/ORB_SLAM3 -i -t --rm orb-slam3-dev
