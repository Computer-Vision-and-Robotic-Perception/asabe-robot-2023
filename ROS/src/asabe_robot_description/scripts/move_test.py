#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv



class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_move')
        self.publisher = self.create_publisher(Twist, '/base_diff_controller/cmd_vel_unstamped', 10)

    def move_distance(self, distance, linear_x = 0.1, linear_y = 0.1):
        twist = Twist()
        twist.linear.x = linear_x  # Linear velocity of 0.2 m/s (adjust as needed)
        duration = abs(distance / twist.linear.x)  # Calculate the duration based on the distance
        start_time = time.time()

        while (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            self.get_logger().info('Moving forward...')
            time.sleep(0.1)

        # Stop the robot after reaching the desired distance
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Distance reached. Stopping...')

    def rotate_angle(self, angle, angular_z = 0.2):
        twist = Twist()
        twist.angular.z = angular_z  # Angular velocity of 0.5 rad/s (adjust as needed)
        duration = abs(math.radians(angle) / twist.angular.z)  # Calculate the duration based on the angle
        start_time = time.time()

        while (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            self.get_logger().info('Rotating...')
            time.sleep(0.1)

        # Stop the robot after rotating the desired angle
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Rotation completed. Stopping...')


def main(args=None):
    rclpy.init(args=args)
    control_node = RobotControl()
    
    # Move 0.127 (5 inch) meter forward
    control_node.move_distance(distance=0.2, linear_x = 0.1, linear_y = 0.1)

    # # Rotate 90 degrees
    # control_node.rotate_angle(angle = 45, angular_z = -0.1)

    # # Move 0.2 meters forward
    # distance = 0.2  # 0.2 meters (adjust as needed)
    # control_node.move_distance(distance)

    # # Rotate 90 degrees
    # control_node.rotate_angle(angle)

    # -----------------------------------------------------#
    

    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
