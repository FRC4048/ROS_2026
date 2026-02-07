import socket
import rclpy
import time
import socket
import struct
from rclpy.node import Node
from std_msgs.msg import String
from roborio_msgs.msg import RoborioOdometry


class TcpClientNode(Node):
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
        # calculate latency
        diff = self.get_clock().now() - rclpy.time.Time.from_msg(pose_msg.header.stamp)
        latency = round(diff.nanoseconds / 1e6)  # latency is in milliSeconds
        # Create message buffer with POSE (x, y, theta), DISTANCE of robot to tag, and the TAG
        msg = [
            pose_msg.x,
            pose_msg.y,
            pose_msg.yaw,
            pose_msg.distance,
            latency,
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
