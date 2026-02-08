import os
import math
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, PathJoinSubstitution
from redshift_odometry.TagTable import *
from redshift_odometry.CamTable import *
 	 	  
def generate_launch_description():
   """
    Generates a ROS2 LaunchDescription to initialize the vision and odometry pipeline.
    THIS IS THE ROS2 LAUNCH ENTRY POINT!

    The pipeline includes camera driver, image rectification, AprilTag detection, pose estimation.
    We utilize Composable Nodes for efficient image processing and  conditional
    logic to handle different hardware (Logitech/Arducam) and multi-camera instances.

    Args:
        camera_instance (LaunchArgument): The namespace and frame ID for the camera
            (e.g., 'cam1', 'cam2'). Defaults to 'cam1'.
        camera_type (LaunchArgument): The hardware driver selector. 
            'L' for Logitech (30fps), 'A' for Arducam (60fps). Defaults to 'L'.

    Nodes Started:
        1. ComposableNodeContainer: Houses the 'cam_driver' and 'rectify' nodes.
        2. apriltag_node: Performs AprilTag detection using instance-specific YAMLs.
        3. redshift_cam_node: Our odometry node for robot pose calculation.
        4. static_transform_publisher: Static transforms defining robot-to-camera and 
           world-to-tag spatial relationships based on CamTable and TagTable.

    Returns:
        launch.LaunchDescription: The complete description of nodes, arguments, 
            and transformations to be executed by the ROS2 launch system.

    Note:
        This function relies on external configuration tables (`CamTable`, `TagTable`)
        and expects AprilTag parameter files to exist at absolute paths on the 
        filesystem (e.g., `/redshift/ros2_ws/misc/`).
    """
   ld = LaunchDescription()

   # to launch different file for each camera use:
   #      ros2 launch redshift_odometry new_logitech_launch.py camera_instance:='cam1' camera_type:='L'
   
   camera_instance = LaunchConfiguration('camera_instance')     # should be values that are in CamTable
   camera_instance_arg = DeclareLaunchArgument('camera_instance', default_value='cam1', description='camera frame')
   camera_type  = LaunchConfiguration('camera_type')            # L for logitech, A for arducam
   camera_type_arg = DeclareLaunchArgument('camera_type', default_value='L', description='camera type')
   
   # temp for testing on my Dell
   parameter_file_path_cam1 = "/home/redshift/ros2_ws/misc/apriltag_cam1.yaml"
   parameter_file_path_cam2 = "/home/redshift/ros2_ws/misc/apriltag_cam2.yaml"

   # real for running on the Pi   
   #parameter_file_path_cam1 = "/redshift/ros2_ws/misc/apriltag_cam1.yaml"
   #parameter_file_path_cam2 = "/redshift/ros2_ws/misc/apriltag_cam2.yaml" 


   logitech_comp = ComposableNode(
                             package='usb_cam',
                             plugin='usb_cam::UsbCamNode',
                             name='cam_driver',
                             namespace=camera_instance,
                             remappings=[(  PathJoinSubstitution(['/',camera_instance,'image_raw'])   ,   PathJoinSubstitution(['/',camera_instance,'image'])  )],
                             parameters=[
                                {'video_device': '/dev/video2'},
                                {'camera_name': 'logitech_cam'},
                                {'frame_id': camera_instance},
                                {'brightness': 133},
                                {'contrast': 256},
                                {'hue': 40.0},
                                {'image_width': 640},
                                {'image_height': 480},
                                {'framerate': 30.0}
                             ],
                             condition=IfCondition(PythonExpression(['"', LaunchConfiguration('camera_type'), '" == "L"']))
                             )
                             
   arducam1_comp = ComposableNode(
                             package='usb_cam',
                             plugin='usb_cam::UsbCamNode',
                             name='cam_driver',
                             namespace=camera_instance,
                             remappings=[(  PathJoinSubstitution(['/',camera_instance,'image_raw'])   ,   PathJoinSubstitution(['/',camera_instance,'image'])  )],
                             parameters=[
                                {'video_device': '/dev/video4'},
                                {'camera_name': 'arducam_cam'},
                                {'frame_id': camera_instance},
                                {'brightness': -16},
                                {'contrast': 64},
                                {'hue': 40.0},
                                {'image_width': 640},
                                {'image_height': 480},
                                {'framerate': 60.0},
                                {'pixel_format': 'mjpeg2rgb'},
                             ],                          
                             condition=IfCondition(PythonExpression(['"', LaunchConfiguration('camera_type'), '" == "A" and "', LaunchConfiguration('camera_instance'), '" == "cam1"']))
                             )
   
   arducam2_comp = ComposableNode(
                             package='usb_cam',
                             plugin='usb_cam::UsbCamNode',
                             name='cam_driver',
                             namespace=camera_instance,
                             remappings=[(  PathJoinSubstitution(['/',camera_instance,'image_raw'])   ,   PathJoinSubstitution(['/',camera_instance,'image'])  )],
                             parameters=[
                                {'video_device': '/dev/video2'},
                                {'camera_name': 'arducam_new'},
                                {'frame_id': camera_instance},
                                {'brightness': -16},
                                {'contrast': 64},
                                {'hue': 40.0},
                                {'image_width': 640},
                                {'image_height': 480},
                                {'framerate': 60.0},
                                {'pixel_format': 'mjpeg2rgb'},
                             ],                          
                             condition=IfCondition(PythonExpression(['"', LaunchConfiguration('camera_type'), '" == "A" and "', LaunchConfiguration('camera_instance'), '" == "cam2"']))
                             )

   rect_comp = ComposableNode(package='image_proc',
                             plugin='image_proc::RectifyNode',
                             name='rectify',
                             namespace=camera_instance,
                             parameters=[
                                {'queue_size': 10}
                             ])
   
   image_processing_node = ComposableNodeContainer(
                             namespace=camera_instance,
                             name='image_processing_container',
                             package='rclcpp_components',
                             executable='component_container',
                             composable_node_descriptions=[
                                 logitech_comp,
                                 arducam1_comp,
                                 arducam2_comp,
                                 rect_comp
                             ])

 
   #---------------------------------------------------------------------------------------------#
   # following is the apriltag node, to support multiple cameras, had to duplicate the node as
   # I couldn't find a better way to create the parm path.
   #---------------------------------------------------------------------------------------------#
   apriltag_cam1_node = Node(
      package='apriltag_ros',
      executable='apriltag_node',
      namespace=camera_instance,
      parameters=[parameter_file_path_cam1],
      condition=IfCondition(PythonExpression(['"', LaunchConfiguration('camera_instance'), '" == "cam1"']))
   )  

   apriltag_cam2_node = Node(
      package='apriltag_ros',
      executable='apriltag_node',
      namespace=camera_instance,
      parameters=[parameter_file_path_cam2],
      condition=IfCondition(PythonExpression(['"', LaunchConfiguration('camera_instance'), '" == "cam2"']))
   )  


   robot_to_cam1_node = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name=camera_instance,
      output='screen',
      arguments=[
         '--x', str(0),
         '--y', str(0),
         '--z', str(0),
         '--roll', str(-1.57),
         '--pitch', str(0),
         '--yaw', str(-1.57),
         '--frame-id', 'robot',
         '--child-frame-id', camera_instance
      ],
      respawn=True,
      respawn_delay=2   
   )
   
   
   redshift_odometry_node = Node(
      package='redshift_odometry',
      executable='redshift_cam_node',
      name='odometry',
      namespace=camera_instance,
      output='screen',
      parameters=[{'camera_instance': camera_instance}],
   )



   # Creation of the static transforms should probably be in a separate launch file.
   # I kept it here for now because Docker --network=host did not seem to share network.
   # These lines should be uncommented when running on the rpi
   # when running on Dell, it is better to use ros2 launch redshift_odometry static-launch.py (you'll get tag transforms)
   
   #for cam_entry in CamTable.cam_table:
   #   ld.add_action(create_robot_to_cam_node(cam_entry))   

   ld.add_action(camera_instance_arg)
   ld.add_action(camera_type_arg)  
   #ld.add_action(PushRosNamespace(camera_instance))  # didn't work, not sure why
   ld.add_action(apriltag_cam1_node)  
   ld.add_action(image_processing_node)
   ld.add_action(apriltag_cam2_node)
   ld.add_action(redshift_odometry_node)
       
   return ld


def create_robot_to_cam_node(entry):
   cam   = entry["camid"]
   x     = entry["x"]
   y     = entry["y"]
   z     = entry["z"]
   qw, qx, qy, qz = CamTable.compound_quat(entry)
   
   nd = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='RobotTo' + cam,
      output='screen',
      arguments=[
         '--x', str(x),
         '--y', str(y),
         '--z', str(z),
         '--qx', str(qx),
         '--qy', str(qy),
         '--qz', str(qz),
         '--qw', str(qw),
         '--frame-id', 'robot',
         '--child-frame-id', cam
      ],
      respawn=True,
      respawn_delay=2   
   )
   return(nd)
   
