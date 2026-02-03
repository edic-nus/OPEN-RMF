#!/bin/bash
set -e

# Source ROS2 setup
source /opt/ros/${ROS_DISTRO}/setup.bash
source /root/lift_ws/install/setup.bash

# Execute the command passed to docker run
exec "$@"