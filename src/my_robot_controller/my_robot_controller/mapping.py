#!/usr/bin/env python3

import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtlebotMappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.get_logger().info("Mapping Node has started.")

        # Publisher to send movement commands
        self._pose_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        # Subscriber to receive LiDAR data
        self._scan_listener = self.create_subscription(
            LaserScan, "/scan", self.robot_controller, 10
        )

        # Exploration control variables
        self._exploration_counter = 0
        self._max_exploration_wait = 10  # Number of iterations before forcing exploration

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()

        # Define safe distance thresholds (in meters)
        FRONT_THRESHOLD = 1.0   # Close range at the front
        SIDE_THRESHOLD = 1.0    # Safe side distance
        BACK_THRESHOLD = 0.5    # Safe rear distance
        CORNER_THRESHOLD = 0.6  # Threshold for detecting corners

        # Define the width of the laser scan to check (ranges covering 90 degrees left and right)
        a = 30  # 30 points to each side, 60 degrees total for left and right scan range

        # Get distance readings in different directions
        front_range = min(scan.ranges[:a] + scan.ranges[-a:])
        left_range = min(scan.ranges[90-a:90+a+1])
        right_range = min(scan.ranges[270-a:270+a+1])
        rear_range = min(scan.ranges[180-a:180+a+1])

        # Navigation logic based on obstacle detection
        if front_range < FRONT_THRESHOLD:
            # Obstacle ahead
            if left_range > right_range:  # If the left side is clearer
                cmd.linear.x = 0.05  # Slow down
                cmd.angular.z = 0.3  # Gentle left turn (smaller angular velocity)
            else:  # If the right side is clearer or equally blocked
                cmd.linear.x = 0.05  # Slow down
                cmd.angular.z = -0.3  # Gentle right turn (smaller angular velocity)
        elif rear_range < BACK_THRESHOLD:
            # Obstacle behind, reverse if stuck
            cmd.linear.x = -0.05  # Move backward
            cmd.angular.z = 0.0   # No rotation while reversing
        elif left_range < CORNER_THRESHOLD and right_range < CORNER_THRESHOLD:
            # Robot is in a corner (both left and right are blocked)
            self.get_logger().info("In a corner! Trying to reverse.")
            cmd.linear.x = -0.05  # Move backward to escape the corner
            cmd.angular.z = 0.0   # No rotation while reversing
        else:
            # Path is clear ahead, move forward slowly
            cmd.linear.x = 0.2  # Moderate forward speed
            cmd.angular.z = 0.0  # No turn

            # Exploration strategy: Higher probability to explore
            if front_range > FRONT_THRESHOLD and random.random() < 0.4:  # Increased exploration probability
                # Decide whether to turn left or right based on the clearer side
                if left_range > right_range:
                    cmd.angular.z = 0.3  # Turn left (gentler)
                else:
                    cmd.angular.z = -0.3  # Turn right (gentler)

                # Reset exploration counter (because robot is moving in new direction)
                self._exploration_counter = 0
            else:
                # Increment the exploration counter if no exploration move is made
                self._exploration_counter += 1

            # If robot has been moving in the same area for too long, force it to explore
            if self._exploration_counter > self._max_exploration_wait:
                self.get_logger().info("Forcing exploration after waiting too long in the same area.")
                cmd.angular.z = random.choice([0.3, -0.3])  # Turn randomly but gently
                self._exploration_counter = 0  # Reset counter

        # Publish the movement command
        self._pose_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()








