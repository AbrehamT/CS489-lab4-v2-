#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        self.declare_parameter('mode', 'sim')
        self.declare_parameter('ttc', 2.0)
        self.declare_parameter('student', 'richard')

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.ttc = self.get_parameter('ttc').get_parameter_value().double_value
        self.student = self.get_parameter('student').get_parameter_value().string_value

        self.get_logger().info(f"Running in mode: {self.mode}")
        self.get_logger().info(f"Time-to-collision threshold: {self.ttc}")

        self.speed = 0.0
        self.mode_param = '/ego_racecar/odom' if self.get_parameter('mode').value == 'sim' else '/odom'

        # TODO: create ROS subscribers and publishers.
        
        # AckermannDriveStamped publisher
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # LaserScan /scan message subscriber
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Odometry /ego_racecar/odom message subscriber
        self.odom_sub = self.create_subscription(Odometry, self.mode_param, self.odom_callback, 10) 

        self.ttc_threshold = 1.5

        self.get_logger().info("SafetyNode started")

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x
        self.get_logger().info(f"Speed updated: {self.speed} m/s")

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ranges = np.array(scan_msg.ranges)
        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment

        # Handle invalid range values (e.g., inf or nan)
        ranges = np.where(np.isfinite(ranges), ranges, np.inf)

        # Calculate angles for each laser beam
        angles = angle_min + np.arange(len(ranges)) * angle_inc

        # Calculate range rates
        range_rates = -self.speed * np.cos(angles)

        # Avoid division by zero: replace range rates > 0 with np.inf
        range_rates = np.where(range_rates > 0, np.inf, range_rates)

        # Calculate iTTC for each laser beam
        ittc = ranges / np.maximum(-range_rates, 1e-6)
        
        # TODO: publish command to brake
        if np.any(ittc < self.ttc_threshold):
            self.get_logger().warn("Collision imminent! Applying brakes.")
            self.brake()
        else:
            self.get_logger().info("No immediate danger detected.")

    def brake(self):
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = 0.0
        self.drive_pub.publish(brake_msg)
        self.get_logger().info("Braking command sent.")    


def main(args=None):
    rclpy.init(args=args)
    richard_safety_node = SafetyNode()
    rclpy.spin(richard_safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    richard_safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()