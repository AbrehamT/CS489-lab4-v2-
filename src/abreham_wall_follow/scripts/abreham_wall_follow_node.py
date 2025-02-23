#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# from rclpy.clock import Clock

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        now = self.get_clock().now()
        self.prev_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9


        self.inter_beam_angle = 40
        self.desired_distance_from_wall = 1.0 # In meters
        self.lookahead_dist = 1.5 # In meters
        self.steering_angle = 0

        # TODO: create subscribers and publishers

        # TODO: set PID gains
        self.kp = 1.0
        self.kd = .75
        self.ki = 0

        # TODO: store history
        self.integral = 0 
        self.prev_error = 0 
        self.error = 0

        # TODO: store any necessary values you think you'll need
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.scan_callback,
            10
        )

        self.drive_subscriber = self.create_subscription(
            AckermannDrive,
            '/drive',
            self.drive_callback,
            10
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

    def get_range(self, range_data:LaserScan, angle, ):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:https://github.com/unlv-f1/lab4.git
            range: range measurement in meters at the given angle

        """

        # Converting the angle provided to get the index for the LiDAR reading 
        lidar_angle_inc = 0.25
        index = (angle + 135) / lidar_angle_inc
        # angle = np.radians(angle)
        # index = round((angle - range_data.angle_min) / range_data.angle_increment)        
        index = round(index)

        # self.get_logger().info(f"GETTING range at index: {index}")
        # Getting the reading at that index.   
            # Handling if reading is inf or nan
        range_measurement_at_given_angle = np.where(np.isfinite(range_data.ranges[index]), range_data.ranges[index], np.inf)
        return range_measurement_at_given_angle

        # #TODO: implement
        # return 0.0

    def get_error(self, range_data):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        beam_b = self.get_range(range_data, 90)
        beam_a = self.get_range(range_data, 90 - self.inter_beam_angle)

        # self.get_logger().info(f"Beam B range: {str(beam_b)}")
        # self.get_logger().info(f"Beam A range: {str(beam_a)}")

        # Alpha is the deviation of our car from the center line in degrees 
        numerator = (beam_a * np.cos(np.radians(self.inter_beam_angle))) - beam_b
        denom = beam_a * np.sin(np.radians(self.inter_beam_angle))
        alpha = np.arctan( numerator / denom )

        current_distance_from_wall = beam_b * np.cos(alpha)

        future_distance_from_wall = self.lookahead_dist * np.sin(alpha)

        future_distance_from_center_line = future_distance_from_wall - self.desired_distance_from_wall        
        y = current_distance_from_wall - self.desired_distance_from_wall 
        
        error_value = (y + future_distance_from_wall)

        # error_value = self.desired_distance_from_wall - (current_distance_from_wall + future_distance_from_wall)

        # error_value = self.desired_distance_from_wall - total_deviation
        return error_value
    
        # TODO:implement
        # return 0.0

    def pid_control(self, error, velocity, laser_reading:LaserScan):
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
        now = self.get_clock().now()
        current_time = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
        delta_t = (self.prev_time - current_time)

        self.integral += self.prev_error * delta_t

        proportional_term = self.kp * error
        integral_term = self.ki * self.integral
        derivative_term = self.kd * (error - self.prev_error) / delta_t 
        
        control_output = proportional_term + integral_term + derivative_term
        # self.get_logger().info(f"Current Error is at: {str(error)}")

        # self.get_logger().info(f"Updated Control Output to: {str(np.deg2rad(control_output))}")
        self.get_logger().info(f"Updated Control Output to: {str(control_output)}")

        info = f"""
            State: {laser_reading.ranges[900]},
            Reference: {self.desired_distance_from_wall},
            Error: {error}
            Control Output: {np.radians(control_output)},
            Kp: {self.kp},
            Ki: {self.ki}
            Kd: {self.kd},
        """
        drive_msg = AckermannDriveStamped()
        # drive_msg.drive.steering_angle = np.deg2rad(control_output)
        drive_msg.drive.steering_angle = control_output


        if abs(control_output) > 0 and abs(control_output) <= 10:
            drive_msg.drive.speed = 1.5
        elif abs(control_output) > 10 and abs(control_output) <= 20:
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5

        self.drive_publisher.publish(drive_msg)

        self.prev_time = current_time
        self.prev_error = error

        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.get_error(msg) # TODO: replace with error calculated by get_error()
        velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(error, velocity, msg) # TODO: actuate the car with PID

    def drive_callback(self, msg):
        self.steering_angle = msg.drive.steering_angle

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