"""
redshift_lifesigns.py
=====================
This module contains the Redshift Lifesigns ROS2 node, which acts as a heartbeat 
monitor for the robot system. It increments a counter and broadcasts it to 
ROS2 topics and/or NetworkTables.

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
from std_msgs.msg import UInt16, Float64
import ntcore

from roborio_msgs.msg import RoborioTags

class RedshiftLifesigns(Node):
    """
    A ROS2 Node that manages system 'lifesigns' (heartbeats) and reads vision constants.

    This node increments a counter every second and publishes it to ensure
    communication links between RPi, RoboRIO, and Driver Station 
    are active. It also reads vision_constant from NetworkTables and publishes
    it to the parameter server for other nodes to use.

    Attributes:
        publisher (rclpy.publisher.Publisher): The ROS2 publisher for heartbeat data.
        lifesigns_pub (ntcore.DoublePublisher): The NetworkTable publisher for heartbeat data.
        vision_constant_sub (ntcore.DoubleSubscriber): The NetworkTable subscriber for vision constant.
        vision_constant (float): Current vision constant value (default: 1/148).
    
    Publishes:
        /redshift/lifesigns (std_msgs.msg.UInt16): The current heartbeat count.
    
    Parameters:
        /redshift/vision_constant (double): Vision constant from NetworkTables for other nodes.
    """

    def __init__(self):
        super().__init__('lifesigns')
        self.ros_publish = True
        tmp = os.environ.get('PUB_ROS')
        if (tmp in ('0', 'false', 'False', 'f', 'F')):
           self.ros_publish = False
        if (self.ros_publish):
           self.get_logger().info('Publishing to ROS')

        # CREATE NETWORK TABLE CONNECTION AND PUBLISHER
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("ROS Client")
        self.inst.setServerTeam(4048)
        #self.inst.setServer("192.168.2.191")
        while not self.inst.isConnected():
            pass
        self.get_logger().info("Connected to NETWORK TABLES")
        self.table = self.inst.getTable("ROS")
        self.inst.startDSClient()
        self.lifesigns_pub = self.table.getDoubleTopic("lifesigns").publish()
        # Add vision constant subscriber
        self.vision_constant_sub = self.table.getDoubleTopic("vision_constant").subscribe(1.0/148.0)
        
        # CREATE vision_constant PUBLISHER
        self.vision_constant_publisher = self.create_publisher(Float64, "/redshift/vision_constant", 10)
        
        # INITIALIZE COUNTERS AND VALUES
        self.lifesigns_counter = UInt16()
        self.lifesigns_counter.data = 0
        self.vision_constant = 1.0/148.0  # Default vision constant
        
        # CREATE TIMERS
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_callback)
    
    def publish_callback(self):
        # 1. Read vision constant from NetworkTables
        new_vision_constant = self.vision_constant_sub.get()
            
        if new_vision_constant != self.vision_constant:
           self.get_logger().info(f'Vision constant updated: {self.vision_constant:.6f} -> {new_vision_constant:.6f}')
           self.vision_constant = new_vision_constant

        # 2. Publish the constant to a ROS Topic
        vision_msg = Float64()
        vision_msg.data = float(self.vision_constant)
        self.vision_constant_publisher.publish(vision_msg)
        
        # 3. Publish lifesigns
        self.lifesigns_counter.data += 1
        self.lifesigns_pub.set(self.lifesigns_counter.data)  
    
    
def main(args=None):
    rclpy.init(args=args)
    publisher = RedshiftLifesigns()
    rclpy.spin(publisher)

if __name__ == '__main__':
    main()
