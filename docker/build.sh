#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo ${SCRIPT_DIR}

MODULE_NAME="guided-extended-hybrid-astar"
DOCKER_IMAGE_NAME=""$MODULE_NAME
TAG="latest"

docker build \
    -t $DOCKER_IMAGE_NAME:$TAG \
    -f Dockerfile ${SCRIPT_DIR}/.. \
    --build-arg MODULE_NAME=$MODULE_NAME \
    --progress plain
