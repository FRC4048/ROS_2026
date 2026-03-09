#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source ./ros2_ws/install/setup.bash --

# Read TCP port from the dedicated TCP config file
CONFIG_FILE="./ros2_ws/src/redshift_odometry/config/tcp_config.yaml"

# Extract TCP port from config file
if [ -f "$CONFIG_FILE" ]; then
    TCP_PORT=$(grep "^tcp_port:" "$CONFIG_FILE" | awk '{print $2}')
    echo "Using TCP port: $TCP_PORT from $CONFIG_FILE"
else
    echo "TCP config file $CONFIG_FILE not found, using default port 5806"
    TCP_PORT=5806
fi

# Start TCP node with the port parameter
ros2 run redshift_odometry redshift_tcp --ros-args -p server_port:=$TCP_PORT


