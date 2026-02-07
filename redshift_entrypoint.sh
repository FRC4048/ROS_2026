#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source ./ros2_ws/install/setup.bash --

if [[ "$REDSHIFT_SCRIPT" = "CAMERA" ]]; then
  if [[ "$CAM" = "L" ]]; then
     ros2 launch redshift_odometry logitech_launch.py
  else
     ros2 launch redshift_odometry arducam_launch.py
  fi
elif [[ "$REDSHIFT_SCRIPT" = "INVERSE" ]]; then
  ros2 run redshift_odometry redshift_inverse
elif [[ "$REDSHIFT_SCRIPT" = "ODOMETRY" ]]; then
  ros2 run redshift_odometry redshift_odometry
fi

# exec "$@"
