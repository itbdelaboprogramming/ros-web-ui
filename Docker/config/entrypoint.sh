#!/bin/bash

set -e

# Source ROS
source /opt/ros/noetic/setup.bash
 
# echo "Provided arguments: $@"

exec "$@"