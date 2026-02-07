import os
import signal
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from collections import deque

# subscrive to /rosout
# look for the out of sync messages
# if so many of them in certain time, exit and have docker compose restart.

class monitorNode(Node):
    def __init__(self):
        super().__init__('subscriber')

        # create a 5 element q that will hold the last 5 timestamps
        self.qsize = 10   # if we get qsize async messages in qtime seconds
        self.qtime = 10
        self.tsq = deque([],self.qsize)
        for i in range (self.qsize):
           self.tsq.append

        # create SUBSCRIBER
        self.subscription = self.create_subscription(
            Log,
            '/rosout',
            self.console_callback,
            10)
        self.subscription  # prevent unused variable warning

    def console_callback(self, msg):
        if   (msg.name == 'apriltag' and
              msg.function == 'checkImagesSynchronized' and
              msg.msg.find("do not appear to be synchronized.") != 0):
            self.tsq.append(msg.stamp.sec)
            if   (len(self.tsq) == self.qsize and
                  self.tsq[-1] - self.tsq[0] < self.qtime):
               self.get_logger().info('No synchronization for Apriltags')

    #def get_out(self):
    #    print("Exiting")
    #    print(os.getppid())
    #    os.kill(os.getppid(), signal.SIGTERM)

def main(args=None):
    rclpy.init(args=args)
    monitor = monitorNode()
    rclpy.spin(monitor)

if __name__ == '__main__':
    main()