#!/bin/sh

mkdir -p contents
rm -rf contents/*

mkdir -p contents/ros2_ws
mkdir -p contents/ros2_ws/log
mkdir -p contents/ros2_ws/build
mkdir -p contents/ros2_ws/install
mkdir -p contents/ros2_ws/config

cp -r misc contents/ros2_ws
cp -r src contents/ros2_ws
cp install/* contents/ros2_ws/install

# select FRONT RPi camera configuration
cp misc/camera/front_rpi_camera.yaml contents/ros2_ws/config/camtable.yaml

cp redshift_entrypoint.sh contents
cp start-*.sh contents
chmod +x contents/redshift_entrypoint.sh
chmod +x contents/start-*.sh

docker build --platform linux/arm64 -t frc4048-ros2-front .

docker save frc4048-ros2-front -o frc4048-ros2-front.tar
