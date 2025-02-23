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

        self.declare_parameter('speed', 2.0)
        self.declare_parameter('P', 1.0)
        self.declare_parameter('I', 0.0)
        self.declare_parameter('D', 1.0)

        # TODO: create subscribers and publishers
        self.scan_subscriber = self.create_subscription(
            LaserScan, lidarscan_topic, self.scan_callback, 10
        )
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, drive_topic, 10
        )

        # TODO: set PID gains 
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.kp = self.get_parameter('P').get_parameter_value().double_value
        self.kd = self.get_parameter('D').get_parameter_value().double_value
        self.ki = self.get_parameter('I').get_parameter_value().double_value

        # TODO: store history
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        # TODO: store any necessary values you think you'll need
        self.prev_time = 0
        self.derivative = 0

    def get_range(self, range_data:LaserScan, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        #TODO: implement
        radian = np.radians(angle)
        if radian > range_data.angle_max or radian < range_data.angle_min:
            return None
        index = round((radian - range_data.angle_min)/range_data.angle_increment)
        if index < 0 or index >= len(range_data.ranges):
            return None
        if np.isinf(range_data.ranges[index]) or np.isnan(range_data.ranges[index]):
            return None
        
        return range_data.ranges[index]


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
        L_val = 1.5
        b = self.get_range(range_data, 90)
        a = self.get_range(range_data, 37)

        if a is None or b is None:
            return None
        
        theta = np.radians(53)
        arg1 = a*np.cos(theta)-b
        arg2 = a*np.sin(theta)
        alpha = np.arctan2(arg1, arg2)
        Dt = b * np.cos(alpha)
        Dtl = Dt + (L_val * np.sin(alpha))
        et = Dtl - dist

        time = self.get_clock().now().nanoseconds
        time_difference = (time - self.prev_time) * 1e-9
        self.prev_time = time

        self.prev_error = self.error
        self.error = et 
        self.integral += et * time_difference
        self.integral = np.clip(self.integral, -1.0, 1.0)
        self.derivative = (et - self.prev_error) / time_difference
        return et

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller

        proportional = error * self.kp
        integral = self.integral * self.ki
        derivative = self.derivative * self.kd

        steering_angle = proportional+integral+derivative
        steering_angle = np.clip(steering_angle, -0.4, 0.4)
        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
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
        error = self.get_error(msg, 1.0) # TODO: replace with error calculated by get_error()
        if error is None:
            return
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        if abs(error) < .02 and abs(error) > 0:
            velocity = 1.5
        elif abs(error) < .05 and abs(error) >= .02:
            velocity = 1.0
        elif abs(error) < .5 and abs(error) >= .05:
            velocity = .8
        elif abs(error) < .8 and abs(error) >= .5:
            velocity = .8
        elif abs(error) < 1.0 and abs(error) >= .8:
            velocity = .5
        else:
            velocity = .4

        self.pid_control(error, velocity) # TODO: actuate the car with PID


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