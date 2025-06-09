#!/bin/bash
set -e                                # stop on first error

# Source ROS 2 and your workspace
source "/opt/ros/$ROS_DISTRO/setup.bash"
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# Hand execution over to whatever was given to `docker run … <cmd>`
exec "$@"
