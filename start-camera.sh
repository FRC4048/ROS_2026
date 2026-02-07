#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source ./ros2_ws/install/setup.bash --
#source ./ros2_ws_apriltag/install/setup.bash --
#start
ros2 launch redshift_odometry camera_launch.py camera_instance:='cam'$CAM camera_type:='A'
