#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ -n "$CATKIN_WS" ] && [ -f "$CATKIN_WS/devel/setup.bash" ]; then
  source "$CATKIN_WS/devel/setup.bash"
fi

exec "$@"
