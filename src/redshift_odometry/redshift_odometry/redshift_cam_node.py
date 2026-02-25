"""
redshift_cam_node.py
====================
Calculates robot global position (odometry) by triangulating AprilTag detections.

This node transforms tag-relative camera coordinates into world-relative robot 
coordinates using a pre-defined Field Tag Table and the TF2 tree.

Launch Example:
    ros2 run redshift_odometry redshift_cam_node --ros-args -p camera_instance:=cam1
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
from rclpy.duration import Duration
from apriltag_msgs.msg import AprilTagDetectionArray
from apriltag_msgs.msg import AprilTagDetection
from roborio_msgs.msg import RoborioOdometry
from redshift_odometry.TagTable import *

import numpy as np
import math
import tf_transformations as tft
import pprint

# to start this node, use a launch file or ros2 run redshift_odometry redshift_cam_node --ros-args -p camera_instance:=cam1

class TransformNode(Node):
    """
    ROS2 Node that converts AprilTag detections into robot pose estimates.

    It listens for detections from a camera, looks up the tag's known position 
    in the world via `TagTable`, and performs a coordinate transformation 
    to determine the robot's location relative to the field origin.

    Attributes:
        cam_id (str): Unique identifier for the camera instance (e.g., 'cam1').
        debug (int): Logging level (0: off, 1: debug TFs, 2: verbose).
        tf_buffer (Buffer): TF2 buffer for looking up camera-to-robot transforms.
        tf_listener (TransformListener): Listener for the TF2 buffer.
    
    Subscribes:
        detections (apriltag_msgs.msg.AprilTagDetectionArray): Raw tag data from the vision system.
    
    Publishes:
        /tf (tf2_msgs.msg.TFMessage): Debugging transforms (TEMP frames for RViz).
        /pose (roborio_msgs.msg.RoborioOdometry): Calculated robot position and tag distance.
    """
    def __init__(self):
        super().__init__('transform_node')
        
        self.cam_id = self.declare_parameter('camera_instance', 'cam1').get_parameter_value().string_value
        self.debug = 1            # 0 - off 1 - publish TEMP (for rviz), 2 - verbose
        
        # the following is a rotation to adjust rotation of robot frame to be FLU (Forward-Left-Up) like world
        # instaed of RDF (Right-Down-Forward)
        # BZ - turned out it is not needed, but kept it here in case we ever need this.
        # used the following website: https://www.andre-gaschler.com/rotationconverter/
        #self.adjust_dcm = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
        
        self.get_logger().info("Starting Redshift camera node for " + self.cam_id)

        # create TF2 buffer and listener to get camera transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # set up publisher to publish transform from world->robot for debugging
        self.debug_publisher = self.create_publisher(TFMessage, '/tf', 10)
        # set up pose publisher
        self.pose_publisher = self.create_publisher(RoborioOdometry, '/pose', 10) # must remain /pose not pose so namespace does not affect it
               
        # create a /detections callback
        self.create_subscription(AprilTagDetectionArray, 'detections', self.detection_callback, 10)
               
    # -----------------------------------------------------------------------------------------
    # This callback function is used to search for a detection.
    # We loop through all detections, find tf to robot and publish it.
    # -----------------------------------------------------------------------------------------                   
    def detection_callback(self, msg):  
       """
        Processes a batch of AprilTag detections.

        For each tag detected, this method:
        1. Identifies the tag ID.
        2. Looks up the robot's position relative to that tag using TF2.
        3. Combines that with the tag's global position to find the robot in 'world'.
        4. Calculates distance and Euler angles (yaw).
        5. Publishes the resulting odometry data.

        Args:
            msg (AprilTagDetectionArray): Array of detected AprilTags from the camera.
        """     
       for detection in msg.detections:
          tag = detection.id
          tagid = "tag" + str(tag) + self.cam_id
          try:             
             if (self.debug > 1):
                print(self.tf_buffer.all_frames_as_string())
             
             # Step 1: Get robot pose relative to the tag (tag->robot)
             # This transform represents where the robot is in the tag's local coordinate system.
             tf_tr = self.tf_buffer.lookup_transform(tagid, 'robot', rclpy.time.Time(), Duration(seconds = 0.0))  # tag->robot in tag frame 
             
             # Step 2: Calculate world->robot
             tf_wr = self.combine_transforms(tag, tf_tr) # calculate world->robot from world->tag and tag->robot
             
             # Step 3: calculate distance between robot and tag
             distance = math.sqrt((TagTable.tag_table[tag]["x"] - tf_wr.transform.translation.x) ** 2 + 
                                  (TagTable.tag_table[tag]["y"] - tf_wr.transform.translation.y) ** 2)
             
             # pack in TFMessage and publish (debug)
             if (self.debug):
               tf_message = TFMessage(transforms=[tf_wr])
               self.debug_publisher.publish(tf_message)
               
             # Step 4: Convert Quaternions to Euler for the RoboRIO
             angles = tft.euler_from_quaternion([tf_wr.transform.rotation.x, tf_wr.transform.rotation.y,
              					 tf_wr.transform.rotation.z, tf_wr.transform.rotation.w], axes='szyx')
     
             # Step 4.5: Calculate camera-to-tag angle
             cam_to_tag_yaw = self.calculate_cam_to_tag_angle(tag, tf_tr)
     
             # Step 5: Construct and publish custom Odometry message
             pose_message = RoborioOdometry()
             pose_message.tag = tag
             pose_message.x = tf_wr.transform.translation.x
             pose_message.y = tf_wr.transform.translation.y
             pose_message.yaw = math.degrees(angles[0])
             pose_message.distance = distance
             pose_message.cam_to_tag_yaw = cam_to_tag_yaw
             pose_message.header.stamp = tf_tr.header.stamp 
             pose_message.header.frame_id = tf_tr.header.frame_id
             self.pose_publisher.publish(pose_message)
          except Exception as e:
             if (self.debug > 0):
                self.get_logger().info(f'Could not transform: {e}')
    
    

    # -----------------------------------------------------------------------------------------
    # This function gets two transforms:
    #     trans_ab is world -> tag (tag wrt world - in world frame
    #     trans_bc is tag -> robot (robot wrt tag - in tag frame)
    # It then calculates and returns a transform trans_ac which is world -> robot in world frame
    #
    # We publish a world->TEMP tf so we can view in rviz
    #
    # -----------------------------------------------------------------------------------------       
    def combine_transforms(self, tag, trans_bc):  
       """
        Performs 3D coordinate math to link World -> Tag -> Robot.

        Logic:
            pos(a->c) = pos(a->b) + Rot(b->a) * pos(b->c)
            rot(a->c) = rot(a->b) * rot(b->c)
        Where:
            a = World Frame
            b = Tag Frame
            c = Robot Frame

        Args:
            tag (int): The ID of the AprilTag found in the TagTable.
            trans_bc (TransformStamped): The tag-to-robot transform.

        Returns:
            TransformStamped: The world-to-robot transform.
        """     
       trans_ac = TransformStamped()
       trans_ac.header.stamp = self.get_clock().now().to_msg()
       trans_ac.header.frame_id = trans_bc.header.frame_id[:-2]  #remove the c1 from tag1c1
       trans_ac.header.frame_id = "world"
       trans_ac.child_frame_id = "tag"+str(tag)
       trans_ac.child_frame_id = "TEMP-"+ self.cam_id     
       
       pos_ab = [TagTable.tag_table[tag]["x"], TagTable.tag_table[tag]["y"], TagTable.tag_table[tag]["z"]]
       pos_bc = [trans_bc.transform.translation.x, trans_bc.transform.translation.y, trans_bc.transform.translation.z]
       
       if (self.debug > 1):
          print("pos_ab")
          pprint.pprint(pos_ab)
          print ("pos_bc")
          pprint.pprint(pos_bc)
       
       quat_ab = [TagTable.tag_table[tag]["qx"], TagTable.tag_table[tag]["qy"], TagTable.tag_table[tag]["qz"], TagTable.tag_table[tag]["qw"]]
       quat_bc = [trans_bc.transform.rotation.x, trans_bc.transform.rotation.y, trans_bc.transform.rotation.z, trans_bc.transform.rotation.w]
       dcm_ab = self.get_dcm_from_quat(quat_ab)
       dcm_bc = self.get_dcm_from_quat(quat_bc)   
       
       if (self.debug > 1):
          r_ab, p_ab, y_ab = tft.euler_from_matrix(dcm_ab)
          r_bc, p_bc, y_bc = tft.euler_from_matrix(dcm_bc)       
          print("euler_ab="+str(math.degrees(r_ab))+" , "+str(math.degrees(p_ab))+" , "+str(math.degrees(y_ab)))	
          print("euler_bc="+str(math.degrees(r_bc))+" , "+str(math.degrees(p_bc))+" , "+str(math.degrees(y_bc)))	
            
       # Calculate translation:
       #     pos(a->c)(a) = Pos(a->b)(a)      +    Pos(b->c)(a)
       #                  = Pos(a->b)(a)      +    Rot(b->a)*Pos(b->c)(b)
       #                  = Pos(a->b)(a)      +    Rot-1(a->b)*Pos(b->c)(b)
       dcm_ba = np.transpose(dcm_ab)
       pos_ac = np.array(pos_ab) + np.dot(pos_bc, dcm_ba)

       if (self.debug > 1):
          print("pos_ac")
          pprint.pprint(pos_ac)
       
       trans_ac.transform.translation.x = pos_ac[0]
       trans_ac.transform.translation.y = pos_ac[1]       
       trans_ac.transform.translation.z = pos_ac[2]              
       
       # Calculate rotation:
       #     dcm(a->c) = dcm(a->b) * dcm(b->c)
       
       dcm_ac = np.dot(dcm_ab, dcm_bc)
       #dcm_ac = np.dot(dcm_ac, self.adjust_dcm)
       
       dcm_ac_44 = np.eye(4)
       dcm_ac_44[0:3, 0:3] = dcm_ac       
       quat_ac = tft.quaternion_from_matrix(dcm_ac_44)

       trans_ac.transform.rotation.x = quat_ac[0]
       trans_ac.transform.rotation.y = quat_ac[1]       
       trans_ac.transform.rotation.z = quat_ac[2]       
       trans_ac.transform.rotation.w = quat_ac[3]    
       
       return trans_ac

    def calculate_cam_to_tag_angle(self, tag, tf_tr):
        """
        Calculate angle between camera-to-tag line and tag perpendicular (normal vector).
        
        This gives the approach angle - how directly the camera is facing the tag.
        0° = camera directly facing tag surface
        90° = camera parallel to tag surface
        
        Args:
            tag (int): The AprilTag ID
            tf_tr (TransformStamped): The tag->robot transform from detection_callback
            
        Returns:
            float: Angle in degrees between camera-to-tag line and tag normal
        """
        try:
            # Use existing tag->robot transform to calculate camera-to-tag angle
            # We need to get camera's viewing direction relative to tag
            
            # Extract rotation from tag->robot transform
            quat = [tf_tr.transform.rotation.x,
                   tf_tr.transform.rotation.y,
                   tf_tr.transform.rotation.z,
                   tf_tr.transform.rotation.w]
            
            # Convert quaternion to rotation matrix
            rotation_matrix = tft.quaternion_matrix(quat)
            
            # Camera's viewing direction in tag frame (camera Z-axis = [0,0,1] in camera frame)
            # Transform camera's forward vector to tag frame
            camera_forward_tag = rotation_matrix[0:3, 0:3] @ np.array([0.0, 0.0, 1.0])
            
            # Tag's normal vector in tag frame (tag Z-axis points toward camera)
            tag_normal = np.array([0.0, 0.0, 1.0])
            
            # Calculate angle between camera viewing direction and tag normal
            dot_product = np.dot(camera_forward_tag, tag_normal)
            
            # Clamp to avoid numerical errors with arccos
            dot_product = np.clip(dot_product, -1.0, 1.0)
            
            angle_rad = np.arccos(dot_product)
            angle_deg = math.degrees(angle_rad)
            
            # Detailed debug logging
            if (self.debug > 0):
                self.get_logger().info(f'Tag {tag}: quat={quat}')
                self.get_logger().info(f'Tag {tag}: rotation_matrix=\n{rotation_matrix[0:3, 0:3]}')
                self.get_logger().info(f'Tag {tag}: camera_forward_tag={camera_forward_tag}, tag_normal={tag_normal}')
                self.get_logger().info(f'Tag {tag}: dot_product={dot_product:.6f}, angle={angle_deg:.2f}°')
            
            return angle_deg
            
        except Exception as e:
            if (self.debug > 0):
                self.get_logger().info(f'Could not calculate cam-to-tag angle: {e}')
            return 0.0


    # -----------------------------------------------------------------------------------------
    # Helper function to convert a quaternion to DCM
    # -----------------------------------------------------------------------------------------       
    def get_dcm_from_quat(self, quat):
       dcm_44 = tft.quaternion_matrix(quat)
       dcm_33 = dcm_44[0:3,0:3]
       return dcm_33

             
                
def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
