"""
redshift_lifesigns.py
=====================
This node contains our communication with network tables. 

To start node:
    on rpi - started automatically from docker-compose.yaml
    testing - from terminal got to ros2_ws :
        source "/opt/ros/humble/setup.bash"
        source ./install/setup.bash
        ros2 run redshift_monitor redshift_lifesigns
        then - from a different terminal check the messages:
        ros2 topic echo /redshift/lifesigns
        ros2 topic echo /redshift/vision_constant

    note: if testing on the robot -  setServer(4048)
          if testing NOT on the robot - setServer("192.168.ip.where.NT.server.is.running")
    
    Environment Variables:
        SERVER_SUFFIX: Suffix to append to lifesigns topic (e.g., "SIDES", "SHOOTER")
"""

import rclpy
import os
import time
from rclpy.node import Node
from std_msgs.msg import UInt16, Float64
import ntcore

from roborio_msgs.msg import RoborioTags

class RedshiftLifesigns(Node):
    """
    A ROS2 Node that manages system 'lifesigns' (heartbeats) and vision constant.

    We have a 1.0 second timer callback that:
    - Increments a counter
    - Publishes the counter to NetworkTables
    - Reads vision_constant from NetworkTables
    - Publishes the vision_constant to ROS2 topic for other nodes to use
    
    Attributes:
        lifesigns_pub (ntcore.DoublePublisher): The NetworkTable publisher for heartbeat data.
        vision_constant_sub: The NetworkTable subscriber for vision constant.
        vision_constant_publisher: The ROS2 publisher for vision constant.
        lifesigns_counter (Uint16): The heartbeat counter message.
        vision_constant (float): Current vision constant value (default: 1/148).
    
    Publishes:
        /redshift/lifesigns (std_msgs.msg.UInt16): The current heartbeat count.
        /redshift/vision_constant (std_msgs.msg.Float64): Vision constant from NetworkTables.
    
    Subscribes (NetworkTables):
        ROS/vision_constant (double): Vision constant value from NetworkTables.
    """

    def __init__(self):
        super().__init__('lifesigns')
        self.ros_publish = True
        #tmp = os.environ.get('PUB_ROS')

        # CREATE NETWORK TABLE CONNECTION AND PUBLISHERS
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("ROS Client")
        self.inst.setServerTeam(4048)
        #self.inst.setServer("192.168.2.191")
        while not self.inst.isConnected():
            time.sleep(0.1)
        self.get_logger().info("Connected to NETWORK TABLES")
        self.table = self.inst.getTable("ROS")
        self.inst.startDSClient()
        
        # Get server suffix from environment variable
        server_suffix = os.environ.get('SERVER_SUFFIX', '')
        lifesigns_topic = f"lifesigns_{server_suffix}" if server_suffix else "lifesigns"
        self.lifesigns_pub = self.table.getDoubleTopic(lifesigns_topic).publish()
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
