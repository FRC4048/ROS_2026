#!/bin/sh

mkdir -p contents
rm -rf contents/*

mkdir -p contents/ros2_ws
mkdir -p contents/ros2_ws/log
mkdir -p contents/ros2_ws/build
mkdir -p contents/ros2_ws/install
mkdir -p contents/ros2_ws/redshift_odometry/config

cp -r misc contents/ros2_ws
cp -r src contents/ros2_ws
cp install/* contents/ros2_ws/install

# select FRONT RPi camera configuration
cp misc/front_rpi_cam.yaml contents/ros2_ws/src/redshift_odometry/config/camtable.yaml

# create TCP configuration file for front camera
echo "tcp_port: 5806" > contents/ros2_ws/src/redshift_odometry/config/tcp_config.yaml

cp misc/apriltag_cam1.yaml contents/ros2_ws/src/redshift_odometry/config/apriltag_cam1.yaml
cp misc/apriltag_cam2.yaml contents/ros2_ws/src/redshift_odometry/config/apriltag_cam2.yaml

cp redshift_entrypoint.sh contents
cp start-*.sh contents
chmod +x contents/redshift_entrypoint.sh
chmod +x contents/start-*.sh

docker build --platform linux/arm64 -t frc4048-ros2:latest .

docker save frc4048-ros2:latest -o frc4048-ros2-front.tar
