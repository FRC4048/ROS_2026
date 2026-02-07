FROM ros:humble-ros-base
SHELL [ "/bin/bash" , "-c" ]

RUN apt update && apt install -y build-essential
RUN apt update && apt install -y ros-humble-usb-cam
RUN apt update && apt install -y ros-humble-image-proc
RUN apt update && apt install -y ros-humble-apriltag-ros
#RUN apt update && apt install -y ros-humble-apriltag-msgs
#RUN apt update && apt install -y ros-humble-apriltag
RUN apt update && apt install -y ros-humble-tf-transformations
RUN apt update && apt install -y ros-humble-tf2
RUN apt update && apt install -y ros-humble-v4l2-camera
RUN apt update && apt install -y python3
RUN apt update && apt install -y python3-pip
RUN pip3 install transforms3d
# we need pynycore fore lifesigns
RUN pip3 install --find-links https://tortall.net/~robotpy/wheels/2023/raspbian/ pyntcore
RUN apt update && apt install -y nano

ENV ROS_DISTRO="humble"

# COPY Contents
WORKDIR /redshift
COPY contents .

#BUILD WORKSPACE
WORKDIR ros2_ws
RUN source ./install/setup.bash && colcon build

#BUILD APRILTAG_ROS
#WORKDIR /redshift
#RUN tar -xvf ros2_ws/misc/apriltag_ros.tar

#WORKDIR ros2_ws_apriltag
#RUN source /opt/ros/humble/setup.bash && colcon build

#COPY MISC FILES
WORKDIR /root/.ros/camera_info
COPY misc/logitech_cam.yaml .
COPY misc/arducam_cam.yaml .
COPY misc/arducam_new.yaml .
COPY *.sh /redshift/

WORKDIR /redshift
