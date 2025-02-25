#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.declare_parameter('P', 1.0)
        self.declare_parameter('I', 0.01)
        self.declare_parameter('D', 0.1)
        self.declare_parameter('speed', 1.0)

        # TODO: set PID gains
        # self.kp = 1.5
        # self.kd = 0.2
        # self.ki = 0.002
        self.kp = self.get_parameter('P').value
        self.ki = self.get_parameter('I').value
        self.kd = self.get_parameter('D').value
        self.default_speed = self.get_parameter('speed').value

        # TODO: create subscribers and publishers
        self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need
        # Desired distance to the left wall
        self.desired_distance = 1.0  # meters
        # Lookahead distance
        self.lookahead_distance = 1  # meters

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        """

        #TODO: implement
        index = int((angle - (-np.pi)) / (2 * np.pi) * len(range_data))
        distance = range_data[index]
        if np.isnan(distance) or np.isinf(distance):
            return 10.0  # A large default value in case of bad readings
        
        self.get_logger().info(f"LIDAR angle: {angle:.2f}, distance: {distance:.2f}")
        return distance

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        theta = np.radians(30)  # Angle between beams
        a = self.get_range(range_data, np.pi / 2 - theta)  # Beam at angle
        b = self.get_range(range_data, np.pi / 2)  # Beam perpendicular to car

        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        dt = b * np.cos(alpha)

        if b > 3.0:
            lookahead = max(0.5, self.lookahead_distance * .5) # gradually reduce on turns
        else:
            lookahead = self.lookahead_distance

        dt_plus_1 = dt + lookahead * np.sin(alpha)
        error = dt_plus_1 - dist
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        if abs(error) < 0.05:
            self.integral = 0.0

        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        # Smooth steering damping factor (prevents holding turn too long)
        damping_factor = 0.85  
        steering_angle = (self.kp * error + self.ki * self.integral + self.kd * derivative) * damping_factor

        # TODO: Use kp, ki & kd to implement a PID controller
        # steering_angle = self.kp * error + self.ki * self.integral + self.kd * derivative
        steering_angle = np.clip(steering_angle, -0.35, 0.35)  # Clamp steering angle

        # log pid changes
        self.get_logger().info(f"P: {self.kp:.3f}, I: {self.ki:.3f}, D: {self.kd:.3f}")
        self.get_logger().info(f"Steering Angle: {np.degrees(steering_angle):.2f}°")

        # TODO: fill in drive message and publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # error = 0.0 # TODO: replace with error calculated by get_error()
        # velocity = 0.0 # TODO: calculate desired car velocity based on error
        # self.pid_control(error, velocity) # TODO: actuate the car with PID
        error = self.get_error(msg.ranges, self.desired_distance)
        
        # Speed control based on error magnitude
        if abs(error) < 0.1:
            velocity = 1.5
        elif abs(error) < 0.5:
            velocity = 1.0
        else:
            velocity = 0.5
        
        self.pid_control(error, velocity)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()