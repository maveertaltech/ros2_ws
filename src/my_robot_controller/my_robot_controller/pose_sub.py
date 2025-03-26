#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber") # Node name
        self._pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.get_logger().info("Pose Subscriber Node has started")

    def pose_callback(self, msg: Pose):
        self.get_logger().info(f"[ X : {msg.x}, Y : {msg.y}, Theta: {msg.theta} ]")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node) # Keeps the node alive to listen formessages
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()