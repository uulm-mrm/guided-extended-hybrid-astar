#!/bin/bash
set -e

source /opt/ros/iron/setup.bash
echo "auto-sourced ros"

echo ""
echo "Workflow"
echo "1. build all necessary modules with: colcon build"
echo "2. source the packages: source colcon_build/install/setup.bash"
echo "3. run the simulation: ros2 run freespace_planner simulation.py"
echo ""

/bin/bash

exec "$@"
