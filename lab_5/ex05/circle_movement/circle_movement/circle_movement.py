import math

from geometry_msgs.msg import Twist, Pose

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('robot_frame_listener')

        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 1)


        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):

        msg = Twist()
        scale_forward_speed = 0.5
        msg.linear.x = 0.5
        
        msg.angular.z = 1.0
 

        self.publisher.publish(msg)



def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
