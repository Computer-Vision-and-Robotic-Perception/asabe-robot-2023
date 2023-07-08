# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist

# class RobotControl(Node):
#     def __init__(self):
#         super().__init__('robot_base_move_control')
#         self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

#     def move_distance(self, distance):
#         twist = Twist()
#         twist.linear.x = 0.2  # Linear velocity of 0.1 m/s (adjust as needed)
#         distance_to_travel = 0
#         rate = self.create_rate(10)  # 10 Hz update rate
#         print("======================")

#         while distance_to_travel < distance:
#             self.publisher.publish(twist)
#             self.get_logger().info('Moving forward...')
#             distance_to_travel += abs(twist.linear.x * 0.1)  # 0.1 seconds of travel
#             print("--------------------------------")
#             rate.sleep()

#         # Stop the robot after reaching the desired distance
#         twist.linear.x = 0.0
#         self.publisher.publish(twist)
#         self.get_logger().info('Distance reached. Stopping...')

#     def rotate_angle(self, angle):
#         twist = Twist()
#         twist.angular.z = 0.5  # Angular velocity of 0.5 rad/s (adjust as needed)
#         angle_rotated = 0
#         rate = self.create_rate(10)  # 10 Hz update rate

#         while angle_rotated < angle:
#             self.publisher.publish(twist)
#             self.get_logger().info('Rotating...')
#             angle_rotated += abs(twist.angular.z * 0.1)  # 0.1 seconds of rotation
#             rate.sleep()

#         # Stop the robot after rotating the desired angle
#         twist.angular.z = 0.0
#         self.publisher.publish(twist)
#         self.get_logger().info('Rotation completed. Stopping...')

# def main(args=None):
#     rclpy.init(args=args)
#     control_node = RobotControl()
#     distance = 1.0  # 1 meter (adjust as needed)
#     angle = 90  # 90 degrees (adjust as needed)

#     # Move 1 meter forward
#     control_node.move_distance(distance)

#     # Rotate 90 degrees
#     control_node.rotate_angle(angle)

#     # Move 0.2 meters forward
#     distance = 0.2  # 0.2 meters (adjust as needed)
#     control_node.move_distance(distance)

#     # Rotate 90 degrees
#     control_node.rotate_angle(angle)

#     control_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class RobotControl(Node):
    def __init__(self):
        super().__init__('control_move')
        self.publisher = self.create_publisher(Twist, '/base_diff_controller/cmd_vel_unstamped', 10)

    def move_distance(self, distance):
        twist = Twist()
        twist.linear.x = 0.2  # Linear velocity of 0.2 m/s (adjust as needed)
        duration = distance / twist.linear.x  # Calculate the duration based on the distance
        start_time = time.time()

        while (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            self.get_logger().info('Moving forward...')
            time.sleep(0.1)

        # Stop the robot after reaching the desired distance
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Distance reached. Stopping...')

    def rotate_angle(self, angle):
        twist = Twist()
        twist.angular.z = 0.5  # Angular velocity of 0.5 rad/s (adjust as needed)
        duration = math.radians(angle) / twist.angular.z  # Calculate the duration based on the angle
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
    distance = 1.0  # 1 meter (adjust as needed)
    angle = 180  # 90 degrees (adjust as needed)

    # Move 1 meter forward
    control_node.move_distance(distance)

    # Rotate 90 degrees
    control_node.rotate_angle(angle)

    # Move 0.2 meters forward
    distance = 0.2  # 0.2 meters (adjust as needed)
    control_node.move_distance(distance)

    # Rotate 90 degrees
    control_node.rotate_angle(angle)

    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
