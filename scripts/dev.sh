#!/usr/bin/env bash

readonly image_name="orb-slam3-dev"

sudo docker compose run --build --rm -it dev
