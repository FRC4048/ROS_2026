import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import UInt16
import ntcore

from roborio_msgs.msg import RoborioTags

class RedshiftLifesigns(Node):

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
