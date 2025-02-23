#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

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
        self.declare_parameter('student', 'abe')
        self.declare_parameter('ttc', 2.0)

        # TODO: create ROS subscribers and publishers.

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.ttc_limit = self.get_parameter('ttc').get_parameter_value().double_value
        self.student = self.get_parameter('student').get_parameter_value().string_value
        # self.ttc_limit = self.get_parameter('ttc').value

        self.mode_param = '/ego_racecar/odom' if self.mode == 'sim' else '/odom'


        self.get_logger().info(f"Running in mode: {self.mode_param}")
        self.get_logger().info(f"Time-to-collision threshold: {self.ttc_limit}")

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.subscription_two = self.create_subscription(
            Odometry,
            self.mode_param,
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        drive_cmd = AckermannDriveStamped()
        ittc_min = []
        drive_cmd.drive.speed = float(0)
        # range = 0
        # range = scan_msg.ranges[]
        for i in range(460, 620, 1):    
            dist = scan_msg.ranges[i]
            iTTC = 0
            angle = (i*.25)-135
            approach_v = (-self.speed*np.cos(angle))
            if approach_v > 0:
                iTTC = float('inf')
                continue
            if not math.isinf(iTTC):
                ittc_min.append(dist / abs(approach_v))
        if ittc_min and min(ittc_min) < self.ttc_limit:
            self.get_logger().info(str(min(ittc_min)))
            self.publisher.publish(drive_cmd)

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