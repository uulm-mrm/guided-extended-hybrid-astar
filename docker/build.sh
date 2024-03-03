#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo ${SCRIPT_DIR}

DOCKER_IMAGE_NAME="guided-extended-hybrid-astar"
TAG="latest"

docker build \
    -t $DOCKER_IMAGE_NAME:$TAG \
    -f Dockerfile ${SCRIPT_DIR}/.. \
    --progress plain
