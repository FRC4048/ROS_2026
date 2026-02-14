# ROS_2026

April tag code for 2026

## System Overview

- Two Raspberry Pi 5 devices
- Two Arducam cameras connected to each Raspberry Pi
- ROS 2 Humble
- Code runs inside Docker containers orchestrated with docker-compose
- Development is done on a Linux machine
- Once ready, the code is tested on the Raspberry Pi

---

## Packaging and Deployment

From the terminal (Raspberry Pi IP may be different than shown):

```bash
./build_front.sh
scp frc4048-ros2-front.tar pi@10.40.48.200:
scp docker-compose.yaml pi@10.40.48.200:
ssh pi@10.40.48.200
docker load -i frc4048-ros2-front.tar
docker compose down
docker compose up
```

Cleaning up old images on the rpi:
if the following shows many old images
```bash
docker images
```
you can do some cleanup (images take space):
```bash
docker images prune -a
docker load -i frc4048-ros2-front.tar
```

If you then want to connect to any container and display ROS topics:
```bash
ssh pi@10.40.48.200
docker compose ps
docker execute -it camera1 /bin/bash
source /opt/ros/humble/setup.bash
source ros2-ws/install/setup.bash
ros2 topic list
```

---

## Odometry Process Description

We run the following Docker containers:

1. camera1
2. camera2
3. tcp
4. lifesigns

Each camera container publishes robot pose to `/tf` based on its detections.  
The `tcp` node subscribes to `/tf` and sends the robot poses to the RoboRio using a TCP socket.

`lifesigns` is a node that publishes a counter through NetworkTables to prove Raspberry Pi connectivity.  
This counter should be displayed on the Elastic dashboard.

---

## Camera Container Technical Details

The main node is `redshift_cam_node`, which performs the pose calculation and publishes it.

It is started through a launch file called `camera_launch`.

To start it:

```bash
ros2 launch redshift_odometry camera_launch.py
```

### Launch Parameters

- `camera_instance:='cam1'`  
  Which camera to run

- `camera_type:='A'`  
  - `A` = Arducam  
  - `L` = Logitech (for testing)

Example with parameters:

```bash
ros2 launch redshift_odometry camera_launch.py camera_instance:='cam'$CAM camera_type:='A'
```

### Notes

1. Multiple instances of the camera container are run, each one with a different camera.
2. Each instance uses its own ROS namespace.
3. `camera_instance` is hardcoded in the launch file. If you want to add a camera, the launch file must be updated.

### general functionality

The goal of this node is to find the pose of the middle of the robot relative to the world (field).
The simple explanation:
1. we have a **world -> tag** (TagTable)
2. we get **tag -> camera** transformation (dynamic)
3. we have **camera -> (middle of) robot** transformation (static)
from these 3 transformations, we can find the **world -> robot** transformation which is the pose!

---

## Nodes in Each Camera Instance

### usb_cam  
Camera driver node

- Publishes:
  - `/cam1/camera_info`
  - `/cam1/image`

---

### image_proc  
Rectify image node (removes lens distortion)

- Subscribes:
  - `/cam1/camera_info`
  - `/cam1/image`
- Publishes:
  - `/cam1/image_rect`

---

### apriltag_ros  
AprilTag detection node

- Subscribes:
  - `/cam1/image_rect`
  - `/cam1/camera_info`
- Publishes:
  - `/cam1/detections`
  - `/tf`

---

### redshift_odometry  
Robot odometry node

- Subscribes:
  - `/tf`
  - `/tf_static`
  - `/cam1/detections`
- Publishes:
  - `/pose`
  - `/tf`

---

### Static Transforms

Each camera container also publishes static transforms:

- World → tag
- Robot → camera

---

## Diagnostics Tips

- Development is easier on a Linux machine.
- Once things seem to be working, build the Docker image and test on the Raspberry Pi.
- Echo topics to make sure they are publishing information.
- `/cam1/detections` should show a detection if the camera sees a tag.
- We don't need a static TF for the tag, we just use the TagTable to do the math, but is very helpful to 
  publish them for diagnostics (you can see where the tags are in RVIZ) there is commented out code in the
  launch file to publish that!

If there are no detections:

- Make sure there are no ERRORs or WARNINGs in the camera node logger output.
- Make sure the AprilTag YAML file is correct.
- Make sure the video device is correct (use `v4l2-ctl` if needed).

Use RViz to visualize transformations:

```bash
ros2 run rviz2 rviz2
```

---

## Helpful Commands

### ROS 2

```bash
ros2 node list
ros2 node info /cam1/odometry
ros2 topic list
ros2 topic echo /cam1/detections
```

### Camera / Video

```bash
v4l2-ctl --list-devices
ros2 run image_view image_view --ros-args -r image:=/cam1/image_rect
```
