#!/usr/bin/env bash

if [ -f "/.dockerenv" ]; then
    echo "cannot launch docker inside docker" 1>&2
else
    sudo docker compose run --build --rm -it dev
fi
