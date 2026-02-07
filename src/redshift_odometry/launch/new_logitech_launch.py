import os
import math
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from redshift_odometry.TagTable import *
 	 	  
def generate_launch_description():
   ld = LaunchDescription()

   # to launch different file for each camera use:
   #      ros2 launch redshift_odometry new_logitech_launch.py camera_instance:='cam1' camera_type:='L'
   camera_instance = LaunchConfiguration('camera_instance', default='cam1')
   camera_type  = LaunchConfiguration('camera_type', default = "L")            # L for logitech, A for arducam
   
   # temp for testing on my Dell
   parameter_file_path_cam1 = "/home/redshift/ros2_ws_2025/misc/apriltag_cam1.yaml"
   parameter_file_path_cam2 = "/home/redshift/ros2_ws_2025/misc/apriltag_cam2.yaml"

   # real for running on the Pi   
   #parameter_file_path_cam1 = "/redshift/ros2_ws/misc/apriltag_cam1.yaml"
   #parameter_file_path_cam2 = "/redshift/ros2_ws/misc/apriltag_cam2.yaml" 


   logitech_comp = ComposableNode(
                             package='usb_cam',
                             plugin='usb_cam::UsbCamNode',
                             name='cam_driver',
                             remappings=[('/image_raw', '/image')],
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
                             
   arducam_comp = ComposableNode(
                             package='usb_cam',
                             plugin='usb_cam::UsbCamNode',
                             name='cam_driver',
                             remappings=[('/image_raw', '/image')],   
                             parameters=[
                                {'video_device': '/dev/video2'},
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
                             condition=IfCondition(PythonExpression(['"', LaunchConfiguration('camera_type'), '" == "A"']))
                             )

   rect_comp = ComposableNode(package='image_proc',
                             plugin='image_proc::RectifyNode',
                             name='rectify',
                             parameters=[
                                {'queue_size': 10}
                             ])
   
   image_processing_node = ComposableNodeContainer(
                             namespace='',
                             name='image_processing_container',
                             package='rclcpp_components',
                             executable='component_container',
                             composable_node_descriptions=[
                                 logitech_comp,
                                 arducam_comp,
                                 rect_comp
                             ])

 
   #---------------------------------------------------------------------------------------------#
   # following is the apriltag node, to support multiple cameras, had to duplicate the node as
   # I couldn't find a better way to create the parm path.
   #---------------------------------------------------------------------------------------------#
   apriltag_cam1_node = Node(
      package='apriltag_ros',
      executable='apriltag_node',
      parameters=[parameter_file_path_cam1],
      condition=IfCondition(PythonExpression(['"', LaunchConfiguration('camera_instance'), '" == "cam1"']))
   )  

   apriltag_cam2_node = Node(
      package='apriltag_ros',
      executable='apriltag_node',
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
      output='screen',
      parameters=[{'camera_instance': camera_instance}],
   )


   # BZ - TODO - the following 2 lines as well as create_transform_node function should be deleted from here and we should start static_tf_launch.py for tag transformations
   # kept it here because Docker --network=host did not seem to share network.
   for tag_entry in TagTable.tag_table:
      ld.add_action(create_transform_node(tag_entry))      

   ld.add_action(DeclareLaunchArgument('camera_instance', default_value='cam1', description='camera frame'))
   ld.add_action(DeclareLaunchArgument('camera_type', default_value='L', description='camera type'))    
   ld.add_action(robot_to_cam1_node)   
   ld.add_action(image_processing_node)
   ld.add_action(apriltag_cam1_node)
   ld.add_action(apriltag_cam2_node)
   ld.add_action(redshift_odometry_node)
       
   return ld
   
def create_transform_node(entry):
   # Create a static transform from world to a tag
   # The rotation is applied in a weired order.....Z-Y-X  Yaw-Pitch-Roll 
   # Positive is clockwise (right hand rule)
   tag   = entry["tagid"]
   x     = entry["x"]
   y     = entry["y"]
   z     = entry["z"]
   roll  = math.radians(entry["roll"])
   pitch = math.radians(entry["pitch"])
   yaw   = math.radians(entry["yaw"])
               
   nd = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='tag'+str(tag),
      output='screen',
      arguments=[
         '--x', str(x),
         '--y', str(y),
         '--z', str(z),
         '--roll', str(roll),
         '--pitch', str(pitch),
         '--yaw', str(yaw),
         '--frame-id', 'world',
         '--child-frame-id', 'tag'+str(tag)
      ],
      respawn=True,
      respawn_delay=2   
   )
   return(nd)   
   
   
   
