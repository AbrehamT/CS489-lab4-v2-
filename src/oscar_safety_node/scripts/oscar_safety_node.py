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
        self.speed = 0.
        self.declare_parameter('mode', 'sim')
        self.declare_parameter('student', 'oscar')
        self.declare_parameter('ttc', 2.0)
        # TODO: create ROS subscribers and publishers.
        self.mode_param = '/ego_racecar/odom' if self.get_parameter('mode').value == 'sim' else '/odom'

        self.odometer_subscriber = self.create_subscription(
            Odometry, self.mode_param, self.odom_callback , 10

        )

        self.laser_subscriber = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped, '/drive', 10
        )

    def odom_callback(self, odom_msg:Odometry):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg:LaserScan):
        # TODO: calculate TTC
        minimum_ttc = float('inf')
        for i in range(456, 620):
            current_angle = np.degrees(scan_msg.angle_min + i * scan_msg.angle_increment)
            range_rate = self.speed * np.cos(current_angle)
            #self.get_logger().info(f"speed: {self.speed} Current Angle {current_angle}")
            if range_rate >= 0:
                ttc = float('inf')
            else:
                ttc = scan_msg.ranges[i]/-range_rate
            minimum_ttc = min(minimum_ttc, ttc)
            self.get_logger().info(f"TTC: {str(ttc)} Angle: {current_angle}")
        # TODO: publish command to brake
        if minimum_ttc < 5.0:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.ackermann_publisher.publish(msg)
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()