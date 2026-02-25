#!/bin/bash
set -e
source "/ros2_humble/install/setup.bash" 
source "/workspace/install/local_setup.bash"
colcon build
export ROS_DOMAIN_ID=1

exec "$@"


