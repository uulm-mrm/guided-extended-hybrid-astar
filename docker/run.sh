#!/bin/sh

MODULE_NAME="guided-extended-hybrid-astar"
DOCKER_IMAGE_NAME=""$MODULE_NAME
TAG="latest"

DIRECTORY="/home/$USER"  #$(dirname $(realpath $(dirname $0)))
GOAL_DIR="/home/$USER"  #opt/${MODULE_NAME}"

xhost +si:localuser:root

echo "mounting directory $DIRECTORY to $GOAL_DIR"
docker run \
    -it --rm \
    --runtime=nvidia --gpus all \
    -v ${DIRECTORY}:${GOAL_DIR} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY=$DISPLAY \
    -e SDL_VIDEODRIVER=x11 \
    -e COLORTERM \
    -w $(dirname $(pwd)) \
    -e COLCON_HOME=$(dirname $(pwd)) \
    "$DOCKER_IMAGE_NAME:$TAG" "$@"
