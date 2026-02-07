"""
redshift_lifesigns.py
=====================
This module contains the Redshift Lifesigns ROS2 node, which acts as a heartbeat 
monitor for the robot system. It increments a counter and broadcasts it to 
ROS2 topics and/or NetworkTables.

Environment Variables:
    PUB_ROS (str): Set to '0' or 'false' to disable ROS2 publishing. Defaults to True.
    PUB_NT (str): Set to '1' or 'true' to enable NetworkTables publishing. Defaults to False.

To start node:
    on rpi - started automatically from docker-compose.yaml
    testing - from terminal got to ros2_ws :
        source "/opt/ros/humble/setup.bash"
        source ./install/setup.bash
        ros2 run redshift_monitor redshift_lifesigns
        then - from a different terminal check the messages:
        ros2 topic echo /redshift/lifesigns

    note: if testing on the robot -  setServer(4048)
          if testing NOT on the robot - setServer("192.168.ip.where.NT.server.is.running")
"""


import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import UInt16
import ntcore

from roborio_msgs.msg import RoborioTags

class RedshiftLifesigns(Node):
    """
    A ROS2 Node that manages system 'lifesigns' (heartbeats).

    This node increments a counter every second and publishes it to ensure
    communication links between RPi, the RoboRIO, and the Driver Station 
    are active.

    Attributes:
        ros_publish (bool): Whether the node is currently publishing to ROS.
        nt_publish (bool): Whether the node is currently publishing to NetworkTables.
        publisher (rclpy.publisher.Publisher): The ROS2 publisher for heartbeat data.
        lifesigns_pub (ntcore.DoublePublisher): The NetworkTable publisher for heartbeat data.
    
    Publishes:
        /redshift/lifesigns (std_msgs.msg.UInt16): The current heartbeat count.
    """

    def __init__(self):
        super().__init__('lifesigns')
        
        self.ros_publish = True
        tmp = os.environ.get('PUB_ROS')
        if (tmp in ('0', 'false', 'False', 'f', 'F')):
           self.ros_publish = False
        if (self.ros_publish):
           self.get_logger().info('Publishing to ROS')

        self.nt_publish = False
        tmp = os.environ.get('PUB_NT')
        if (tmp in ('1', 'true', 'True', 'TRUE', 't', 'T')):
           self.nt_publish = True
        if (self.nt_publish):
           self.get_logger().info("Publishing to NETWORK TABLES")

        # CREATE NETWORK TABLE CONNECTION AND PUBLISHER
        if (self.nt_publish):
            self.inst = ntcore.NetworkTableInstance.getDefault()
            self.inst.startClient4("ROS Client")
            self.inst.setServerTeam(4048)
            #self.inst.setServer("192.168.1.160")
            while not self.inst.isConnected():
                pass
            self.get_logger().info("Connected to NETWORK TABLES")
            self.table = self.inst.getTable("ROS")
            self.inst.startDSClient()
            self.lifesigns_pub = self.table.getDoubleTopic("lifesigns").publish()
        
        self.publisher = self.create_publisher(UInt16, "/redshift/lifesigns", 10)
        self.lifesigns_counter = UInt16();
        self.lifesigns_counter.data = 0;
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_callback)
    
    def publish_callback(self):
        self.lifesigns_counter.data += 1
        
        if (self.nt_publish == True):
           self.lifesigns_pub.set(self.lifesigns_counter.data) 
                  
        if (self.ros_publish == True):
           self.publisher.publish(self.lifesigns_counter)

def main(args=None):
    rclpy.init(args=args)
    publisher = RedshiftLifesigns()
    rclpy.spin(publisher)

if __name__ == '__main__':
    main()
