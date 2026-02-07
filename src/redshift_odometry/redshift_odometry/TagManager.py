import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage
from redshift_odometry.TagTable import *
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# this class is used to get all the April tag transformations.
# it is meant to be called from a ROS node.

class TagManager:

    def __init__(self, node, logger):
       self.node = node
       self.logger = logger
       self.transforms_found = 0
       self.received_all_static_tf = False
       self.__tag_dict = {}

       self.subscription = self.node.create_subscription(
            msg_type=TFMessage,
            topic="/tf_static",
            callback=self.static_tf_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=30)
        )
       
    def get_tf_for_tag(self, tag):
       return self.__tag_dict[tag]
       
    def all_tags_received(self):
       return self.received_all_static_tf   
          
    # -----------------------------------------------------------------------------------------
    # This callback function is used to find and save all the static world->tag transforms.
    # Once all are received, the function kills itself.
    # -----------------------------------------------------------------------------------------
    def static_tf_callback(self, msg):
       for transform in msg.transforms:
           #self.logger.info(f"Frame: {transform.header.frame_id} -> {transform.child_frame_id}")
           if (transform.header.frame_id == 'world' and transform.child_frame_id[:3] == 'tag'):
              tagid = int(transform.child_frame_id[3:])
              self.logger.info("Found tag " + str(tagid))
              if (tagid not in self.__tag_dict):
                 self.transforms_found = self.transforms_found + 1
              self.__tag_dict[tagid] = transform
           if (self.transforms_found == len(TagTable.tag_table)):
              self.received_all_static_tf = True		
              self.logger.info("All tags received")


