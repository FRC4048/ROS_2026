"""
tcp_client_node.py
==================
This module provides a ROS2 node that sends odometry data from Rpi to the 
RoboRIO via a TCP/IP socket.

The node serializes RoborioOdometry messages into a packed binary format 
to minimize latency and bandwidth.

To start node:
    on rpi - started automatically from docker-compose.yaml
    testing - YOU NEED TO HAVE AT LEAST ONE CAMERA NODE RUNNING AND GETTING 
              DETECTIONS BEFORE TESTING THIS NODE!
        from terminal got to ros2_ws :
            source "/opt/ros/humble/setup.bash"
            source ./install/setup.bash
            ros2 run redshift_odometry redshift_tcp
        then - from a different terminal check the messages:
            ros2 topic echo /pose 

    note: if testing on the robot -  self.server_ip = "10.40.48.2"
          if testing NOT on the robot - self.server_ip = "192.168.ip.where.NT.server.is.running"
"""

import socket
import rclpy
import time
import socket
import struct
import math
from rclpy.node import Node
from std_msgs.msg import String
from roborio_msgs.msg import RoborioOdometry


class TcpClientNode(Node):
    """
    A ROS2 Node that forwards robot odometry to a TCP server.

    This node subscribes to the `/pose` topic, calculates the message latency, 
    and packs the data into a binary format before sending it over a socket.

    Attributes:
        server_ip (str): The IP address of the TCP server (default: 10.40.48.2).
        server_port (int): The port to connect to (default: 5806).
        socket_connected (bool): Tracking state for the TCP connection.
        socket (socket.socket): The active TCP socket object.

    Subscribes:
        /pose (roborio_msgs.msg.RoborioOdometry): The incoming odometry and tag data.

    Protocol (Binary):
        The data is packed using Big-Endian (`!`) byte order:
        - 7 Doubles (8-byte floats): x, y, yaw, distance, cam_to_tag_yaw, latency, std_deviation.
        - 1 Integer (4-byte int): tag ID.
    """

    def __init__(self):
        super().__init__("tcp_client_node")

        # Set up TCP connection parameters
        # self.server_ip = '192.168.1.230'
        self.server_ip = "10.40.48.2"
        self.server_port = 5806
        self.socket_connected = False


        # Subscribe to the /pose topic and use callback to send data over tcp
        self.pose_subscription = self.create_subscription(
            RoborioOdometry, "/pose", self.tcp_callback, 10
        )

        self.connect_to_server()

    def connect_to_server(self):
        """
        Attempts to establish a TCP connection with the server.
        
        This method blocks using a while-loop and sleep until a connection is 
        successful, ensuring the node doesn't proceed without a valid socket.
        """
        while not self.socket_connected:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.server_ip, self.server_port))
                self.socket_connected = True
                self.get_logger().info("Connected to socket...")
            except Exception as e:
                self.get_logger().warning(
                    f"Could not connect to socket: {e}. Trying again..."
                )
                time.sleep(1)

    def tcp_callback(self, pose_msg):
        """
        Processes incoming pose messages and transmits them via TCP.

        Calculates the latency between the message timestamp and current time,
        calculates standard deviation based on distance and angle,
        packs the message components into a binary buffer, and sends it.

        Args:
            pose_msg (RoborioOdometry): The odometry message received from the /pose topic.
        """
        # calculate latency
        diff = self.get_clock().now() - rclpy.time.Time.from_msg(pose_msg.header.stamp)
        latency = round(diff.nanoseconds / 1e6)  # latency is in milliSeconds
        
        # calculate standard deviation: std = d² × constant / |cos(a)|
        # where d is distance in meters, a is angle in radians, constant is 1.0/148.0
        # Use absolute value of cos to ensure positive standard deviation
        constant = 1.0 / 148.0
        distance = pose_msg.distance
        angle = pose_msg.cam_to_tag_yaw
        
        # Use absolute value of cosine to ensure positive standard deviation
        cos_angle = abs(math.cos(angle))
        if cos_angle < 1e-6:  # Prevent division by very small numbers
            cos_angle = 1e-6
        
        std_deviation = (distance * distance) * constant / cos_angle
        
        # Create message buffer with POSE (x, y, theta), DISTANCE of robot to tag, CAM_TO_TAG_YAW, LATENCY, STD_DEVIATION, and the TAG
        msg = [
            pose_msg.x,
            pose_msg.y,
            pose_msg.yaw,
            pose_msg.distance,
            pose_msg.cam_to_tag_yaw,
            latency,
            std_deviation,
            pose_msg.tag,
        ]
        format_string = "!{}d{}i".format(len(msg) - 1, 1)
        data = struct.pack(format_string, *msg)
        if self.socket_connected:
            try:
                self.socket.sendall(data)
            except Exception as e:
                self.get_logger().error(f"Socket error: {e}. Reconnecting...")
                self.socket_connected = False
                self.socket.close()
                self.connect_to_server()


def main(args=None):
    rclpy.init(args=args)

    tcp_client = TcpClientNode()

    rclpy.spin(tcp_client)

    tcp_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
