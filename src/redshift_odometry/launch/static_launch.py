import os
import math
from launch_ros.actions import Node
from launch import LaunchDescription
from redshift_odometry.TagTable import *
from redshift_odometry.CamTable import *
 	 	  
def generate_launch_description():
   ld = LaunchDescription()   
      
   for tag, entry in TagTable.tag_table.items():
      ld.add_action(create_transform_node(tag, entry))      
   
   for cam_entry in CamTable.cam_table:
      ld.add_action(create_robot_to_cam_node(cam_entry))      
             
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

   
def create_transform_node(tag, entry):
   # Create a static transform from world to a tag
   # Positive is clockwise (right hand rule)
   x     = entry["x"]
   y     = entry["y"]
   z     = entry["z"]
   qx  = entry["qx"]
   qy  = entry["qy"]
   qz  = entry["qz"]
   qw  = entry["qw"]
               
   nd = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='tag'+str(tag),
      output='screen',
      arguments=[
         '--x', str(x),
         '--y', str(y),
         '--z', str(z),
         '--qx', str(qx),
         '--qy', str(qy),
         '--qz', str(qz),
         '--qw', str(qw),
         '--frame-id', 'world',
         '--child-frame-id', 'tag'+str(tag)
      ],
      respawn=True,
      respawn_delay=2   
   )
   return(nd)   
   
