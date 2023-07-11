# #!/usr/bin/env python3

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

# Path to the USB disk
# usb_disk_path = '/media/zhengkun/USB DISK/checkboard/result/'  # Replace with the actual path of the USB disk
usb_disk_path = '/home/asabe/Documents/asabe-robot-2023/ROS/src/asabe_robot_description/scripts/'

# team name
team_name = 'ABE_Gator.csv'  # Replace with the actual path of the USB disk

# File path and name
file_path = usb_disk_path + team_name

def InitCsv(file_path):
    rows = [
        ['Row:', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8', 'P9'],
        ['2', 'Value1', 'Value2', 'Value3', 'Value4', 'Value5', 'Value6', 'Value7', 'Value8', 'Value9'],
        ['4', 'Value10', 'Value11', 'Value12', 'Value13', 'Value14', 'Value15', 'Value16', 'Value17', 'Value18'],
        ['6', 'Value19', 'Value20', 'Value21', 'Value22', 'Value23', 'Value24', 'Value25', 'Value26', 'Value27']
    ]

    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(rows)

def change_csv_value(file_path, row, column, new_value):
    with open(file_path, 'r', newline='') as csvfile:
        rows = list(csv.reader(csvfile))
    
    rows[row][column] = new_value

    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(rows)

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.bridge = CvBridge()
        self.color_subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.color_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            'camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            'camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        self.camera_info = None
        self.depth_image = None

    def color_callback(self, msg):
        if self.camera_info is None or self.depth_image is None:
            return

        # Convert the color image message to OpenCV format
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Calculate the center point coordinates
        center_x = color_image.shape[1] // 2
        center_y = color_image.shape[0] // 2
        depth_value = self.depth_image[center_y, center_x]

        # Convert depth value to meters (assuming depth image is in millimeters)
        depth_meters = depth_value / 1000.0

        # Convert center point from image coordinates to real-world coordinates
        point = self.convert_image_to_point(center_x, center_y, depth_value)

        self.get_logger().info(
            f'Center point in color image: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}) meters'
        )

    def depth_callback(self, msg):
        # Convert the depth image message to OpenCV format
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def convert_image_to_point(self, u, v, depth):
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]

        point_x = (u - cx) * depth / fx
        point_y = (v - cy) * depth / fy
        point_z = depth

        return point_x, point_y, point_z

    def cotton_detection(self, colorimage):
        # color-based detection for cotton
        return 0
    
    def ball_detection(self, colorimage):
        # color-based detection for green ball   
        return 0

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control_move')
        self.publisher = self.create_publisher(Twist, '/base_diff_controller/cmd_vel_unstamped', 10)

    def move_distance(self, distance, linear_x = 0.1, linear_y = 0.1, angular_z = 0.5):
        twist = Twist()
        twist.linear.x = linear_x  # Linear velocity of 0.2 m/s (adjust as needed)
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
        twist.angular.z = angular_z  # Angular velocity of 0.5 rad/s (adjust as needed)
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

class BollDetection(Node):
    def __init__(self):
        super().__init__('boll_detection')
        self.bridge = CvBridge()
        self.center_points = []  # Create a class variable to store the center points
        self.color_subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.color_callback,
            1  # Set the queue size to 1
        )
        self.color_subscription  # Store the subscription object

    def color_callback(self, msg):

        # Convert the color image message to OpenCV format
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the image to the HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper range of green color in HSV
        lower_green = np.array([30, 43, 46])
        upper_green = np.array([80, 255, 255])

        # Create a mask based on the defined color range
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Apply morphological operations to remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Detect contours in the binary image
        contours, _ = cv2.findContours(opening, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through the contours and filter for circular shapes with area greater than 1000 pixels
        green_circles = []
        center_points = []
        for contour in contours:
            area = cv2.contourArea(contour)
            # perimeter = cv2.arcLength(contour, True)
            # circularity = 4 * np.pi * (area / (perimeter ** 2))
            # green_circles.append(contour)
            # if circularity > 0.4 and area > 500:
            if area > 2000:
                green_circles.append(contour)
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    # if (center_x > image.shape[1]/4) and (center_x > image.shape[1]*3/4) :
                    #     center_points.append((center_x, center_y))
                    center_points.append((center_x, center_y))
        self.center_points = center_points
        # Unsubscribe after processing the first message
        self.color_subscription.dispose()

    def get_center_points_count(self):
        # Return the length of center_points
        return len(self.center_points)


def main(args=None):
    rclpy.init(args=args)
    control_node = RobotControl()

    boll_detection = BollDetection()
    boll_count = boll_detection.get_center_points_count()

    InitCsv(file_path)
    change_csv_value(file_path, 1, 3, boll_count)
    print("boll count:")
    print(boll_count)



    # # Rotate 90 degrees
    # control_node.rotate_angle(angle)

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
